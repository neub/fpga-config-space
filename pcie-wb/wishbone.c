#include <linux/module.h>

#include <linux/fs.h>
#include <linux/aio.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/socket.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <linux/slab.h>

#include "wishbone.h"

/* Module parameters */
static unsigned int max_devices = WISHBONE_MAX_DEVICES;

/* Module globals */
static LIST_HEAD(wishbone_list); /* Sorted by ascending minor number */
static DEFINE_MUTEX(wishbone_mutex);
static struct class *wishbone_master_class;
static dev_t wishbone_master_dev_first;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,30)

/* missing 'const' in 2.6.30. present in 2.6.31. */
static int compat_memcpy_fromiovecend(unsigned char *kdata, const struct iovec *iov,
                        int offset, int len)
{
        /* Skip over the finished iovecs */
        while (offset >= iov->iov_len) {
                offset -= iov->iov_len;
                iov++;
        }

        while (len > 0) {
                u8 __user *base = iov->iov_base + offset;
                int copy = min_t(unsigned int, len, iov->iov_len - offset);

                offset = 0;
                if (copy_from_user(kdata, base, copy))
                        return -EFAULT;
                len -= copy;
                kdata += copy;
                iov++;
        }

        return 0;
}


/* does not exist in 2.6.30. does in 2.6.31. */
static int compat_memcpy_toiovecend(const struct iovec *iov, unsigned char *kdata,
                       int offset, int len)
 {
         int copy;
         for (; len > 0; ++iov) {
                 /* Skip over the finished iovecs */
                 if (unlikely(offset >= iov->iov_len)) {
                         offset -= iov->iov_len;
                         continue;
                 }
                 copy = min_t(unsigned int, iov->iov_len - offset, len);
                 if (copy_to_user(iov->iov_base + offset, kdata, copy))
                         return -EFAULT;
                 offset = 0;
                 kdata += copy;
                 len -= copy;
         }

         return 0;
}

/* Over-ride with compatible versions */
#define memcpy_toiovecend   compat_memcpy_toiovecend
#define memcpy_fromiovecend compat_memcpy_fromiovecend
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,26)
/* Older linux versions do not have the drvdata 'a' parameter. >= 2.6.37 present. */
#define device_create(c, p, d, a, f, x) device_create(c, p, d, f, x)
#endif

/* Compiler should be able to optimize this to one inlined instruction */
static inline wb_data_t eb_to_cpu(unsigned char* x)
{
	switch (sizeof(wb_data_t)) {
	case 8: return be64_to_cpu(*(wb_data_t*)x);
	case 4: return be32_to_cpu(*(wb_data_t*)x);
	case 2: return be16_to_cpu(*(wb_data_t*)x);
	case 1: return *(wb_data_t*)x;
	}
}

/* Compiler should be able to optimize this to one inlined instruction */
static inline void eb_from_cpu(unsigned char* x, wb_data_t dat)
{
	switch (sizeof(wb_data_t)) {
	case 8: *(wb_data_t*)x = cpu_to_be64(dat); break;
	case 4: *(wb_data_t*)x = cpu_to_be32(dat); break;
	case 2: *(wb_data_t*)x = cpu_to_be16(dat); break;
	case 1: *(wb_data_t*)x = dat;              break;
	}
}

static int deliver_msi(struct etherbone_master_context* context)
{
	unsigned long flags;
	int out;
	struct wishbone *wb = context->wishbone;
	
	/* msi_unread is protected by the spinlock */
	spin_lock_irqsave(&wb->spinlock, flags);
	out =	context->msi_unread > 0             &&
		context->sent == context->processed &&
		context->sent == context->received;
	spin_unlock_irqrestore(&wb->spinlock, flags);   
	
	return out;
}

/* Must be called with wb->spinlock and mutex held */
static void claim_msi(struct etherbone_master_context* context)
{
	unsigned i;
	struct wishbone *wb = context->wishbone;
	
	/* Safe to read msi_index here, because mutex held */
	if (context->msi_index != -1) return;
	
	for (i = 0; i < WISHBONE_MAX_MSI_OPEN; ++i) {
		if (!wb->msi_map[i]) {
			context->msi_index = i;
			wb->msi_map[i] = context;
			break;
		}
	}
}

/* Must be called with wb->spinlock held */
static void advance_msi(struct wishbone* wb)
{
	struct wishbone_request request;
	struct etherbone_master_context *context;
	uint8_t *wptr;
	int index;

	/* Don't process a second MSI while a previous is inflight */
	if (wb->msi_pending) return;
	
retry:	
	/* If nothing to do, stop */
	if (wb->wops->request(wb, &request) == 0) return;
	
	/* The hardware should already have done this, but be safe */
	request.addr &= wb->mask;
	
	/* If no MSI handler, handle it immediately */
	index = request.addr / ((wb->mask/WISHBONE_MAX_MSI_OPEN)+1);
	if (!(context = wb->msi_map[index])) {
		wb->wops->reply(wb, 1, ~(wb_data_t)0);
		goto retry;
	}
	
	/* Fill in the MSI data */
	wptr = &context->msi[0];
	
	wptr[0] = ETHERBONE_BCA;
	wptr[1] = request.mask;
	if (request.write) {
		wptr[2] = 1;
		wptr[3] = 0;
		wptr += sizeof(wb_data_t);
		eb_from_cpu(wptr, request.addr);
		wptr += sizeof(wb_data_t);
		eb_from_cpu(wptr, request.data);
		wptr += sizeof(wb_data_t);
	} else {
		wptr[2] = 0;
		wptr[3] = 1;
		wptr += sizeof(wb_data_t);
		eb_from_cpu(wptr, WBA_DATA);
		wptr += sizeof(wb_data_t);
		eb_from_cpu(wptr, request.addr);
		wptr += sizeof(wb_data_t);
	}
	
	wptr[0] = ETHERBONE_CYC | ETHERBONE_BCA | ETHERBONE_RCA;
	wptr[1] = 0xf;
	wptr[2] = 0;
	wptr[3] = 1;
	wptr += sizeof(wb_data_t);
	
	eb_from_cpu(wptr, WBA_ERR);
	wptr += sizeof(wb_data_t);
	eb_from_cpu(wptr, 4); /* low bits of error status register */
	wptr += sizeof(wb_data_t);
	
	/* Mark the MSI pending */
	context->msi_unread = wptr - &context->msi[0];
	context->msi_pending = 1;
	wb->msi_pending = 1;
	
	/* Wake-up any reader of the device */
	wake_up_interruptible(&context->waitq);
	kill_fasync(&context->fasync, SIGIO, POLL_IN);
}

static wb_data_t handle_read_cfg(struct etherbone_master_context* context, wb_addr_t addr)
{
	/* Safe to read msi_index here, because mutex held */
	struct wishbone *wb = context->wishbone;
	wb_data_t wide = (wb->mask/WISHBONE_MAX_MSI_OPEN)+1;
	switch (addr) {
	case 32: return 0;                             // request high
	case 36: return 0;                             // request low
	case 40: return 0;                             // granted high
	case 44: return context->msi_index != -1;      // granted low
	case 48: return 0;                             // low high
	case 52: return wide*(context->msi_index+0)-0; // low low
	case 56: return 0;                             // high high
	case 60: return wide*(context->msi_index+1)-1; // high low
	default: return wb->wops->read_cfg(wb, addr);
	}
}

static void handle_write_cfg(struct etherbone_master_context* context, wb_addr_t addr, wb_data_t data)
{
	unsigned long flags;
	struct wishbone *wb = context->wishbone;
	
	spin_lock_irqsave(&wb->spinlock, flags);
	switch (addr) {
	case 36:
		if (data == 1) {
			claim_msi(context);
		}
		break;
		
	case WBA_DATA:
		if (context->msi_pending) {
			wb->msi_data = data;
		}
		break;
	
	case WBA_ERR:
		if (context->msi_pending) { 
			context->msi_pending = 0;
			wb->msi_pending = 0;
			wb->wops->reply(wb, data&1, wb->msi_data);
			advance_msi(wb);
		}
		break;
	}
	spin_unlock_irqrestore(&wb->spinlock, flags);
}

static void etherbone_master_process(struct etherbone_master_context* context)
{
	struct wishbone *wb;
	const struct wishbone_operations *wops;
	unsigned int size, left, i, record_len;
	unsigned char *buf;
	
	if (context->state == header) {
		if (context->received < 8) {
			/* no-op */
			return;
		}
		
		context->buf[0] = 0x4E;
		context->buf[1] = 0x6F;
		context->buf[2] = 0x12; /* V.1 Probe-Response */
		context->buf[3] = (sizeof(wb_addr_t)<<4) | sizeof(wb_data_t);
		/* Echo back bytes 4-7, the probe identifier */
		context->processed = 8;
		context->state = idle;
	}
	
	buf = &context->buf[0];
	wb = context->wishbone;
	wops = wb->wops;
	
	i = RING_INDEX(context->processed);
	size = RING_PROC_LEN(context);
	
	for (left = size; left >= 4; left -= record_len) {
		unsigned char flags, be, wcount, rcount;
		
		/* Determine record size */
		flags  = buf[i+0];
		be     = buf[i+1];
		wcount = buf[i+2];
		rcount = buf[i+3];
		
		record_len = 1 + wcount + rcount + (wcount > 0) + (rcount > 0);
		record_len *= sizeof(wb_data_t);
		
		if (left < record_len) break;
		
		/* Configure byte enable and raise cycle line */
		if (context->state == idle) {
			wops->cycle(wb, 1);
			context->state = cycle;
		}
		wops->byteenable(wb, be);

		/* Process the writes */
		if (wcount > 0) {
			wb_addr_t base_address, increment;
			unsigned char j;
			int wff = flags & ETHERBONE_WFF;
			int wca = flags & ETHERBONE_WCA;
			
			/* increment=0 if wff!=0 */
			increment = sizeof(wb_data_t) * (1 - (wff / ETHERBONE_WFF));
			
			/* Erase the header */
			eb_from_cpu(buf+i, 0);
			i = RING_INDEX(i + sizeof(wb_data_t));
			base_address = eb_to_cpu(buf+i);
			
			if (wca) {
				for (j = wcount; j > 0; --j) {
					eb_from_cpu(buf+i, 0);
					i = RING_INDEX(i + sizeof(wb_data_t));
					handle_write_cfg(context, base_address, eb_to_cpu(buf+i));
					base_address += increment;
				}
			} else {
				for (j = wcount; j > 0; --j) {
					eb_from_cpu(buf+i, 0);
					i = RING_INDEX(i + sizeof(wb_data_t));
					wops->write(wb, base_address, eb_to_cpu(buf+i));
					base_address += increment;
				}
			}
		}
		
		buf[i+0] = (flags & ETHERBONE_CYC) | 
		           (((flags & ETHERBONE_RFF) != 0) ? ETHERBONE_WFF : 0) |
		           (((flags & ETHERBONE_BCA) != 0) ? ETHERBONE_WCA : 0);
		buf[i+1] = be;
		buf[i+2] = rcount; /* rcount -> wcount */
		buf[i+3] = 0;
		
		if (rcount > 0) {
			unsigned char j;
			int rca = flags & ETHERBONE_RCA;
			
			/* Move past header, and leave BaseRetAddr intact */
			i = RING_INDEX(i + sizeof(wb_data_t) + sizeof(wb_data_t));
			
			if (rca) {
				for (j = rcount; j > 0; --j) {
					eb_from_cpu(buf+i, handle_read_cfg(context, eb_to_cpu(buf+i)));
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			} else {
				for (j = rcount; j > 0; --j) {
					eb_from_cpu(buf+i, wops->read(wb, eb_to_cpu(buf+i)));
					i = RING_INDEX(i + sizeof(wb_data_t));
				}
			}
		} else {
			i = RING_INDEX(i + sizeof(wb_data_t));
		}
		
		if ((flags & ETHERBONE_CYC) != 0) {
			wops->cycle(wb, 0);
			context->state = idle;
		}
	}
	
	context->processed = RING_POS(context->processed + size - left);
}

static int char_master_open(struct inode *inode, struct file *filep)
{	
	struct etherbone_master_context *context;
	
	context = kmalloc(sizeof(struct etherbone_master_context), GFP_KERNEL);
	if (!context) return -ENOMEM;
	
	context->wishbone = container_of(inode->i_cdev, struct wishbone, master_cdev);
	context->fasync = 0;
	mutex_init(&context->mutex);
	init_waitqueue_head(&context->waitq);
	context->state = header;
	context->sent = 0;
	context->processed = 0;
	context->received = 0;
	context->msi_index = -1;
	context->msi_unread = 0;
	context->msi_pending = 0;
	
	filep->private_data = context;
	
	return 0;
}

static int char_master_release(struct inode *inode, struct file *filep)
{
	unsigned long flags;
	struct etherbone_master_context *context = filep->private_data;
	struct wishbone *wb = context->wishbone;
	
	/* Did the bad user forget to drop the cycle line? */
	if (context->state == cycle) {
		wb->wops->cycle(wb, 0);
	}
	
	spin_lock_irqsave(&wb->spinlock, flags);
	
	/* Finish any unhandled MSI */
	if (context->msi_pending) {
		context->msi_pending = 0;
		wb->msi_pending = 0;
		wb->wops->reply(wb, 1, ~(wb_data_t)0);
		advance_msi(wb);
	}
	
	/* Unhook any MSI access */
	if (context->msi_index != -1) {
		wb->msi_map[context->msi_index] = 0;
	}
	spin_unlock_irqrestore(&wb->spinlock, flags);
	
	kfree(context);
	return 0;
}

static ssize_t char_master_aio_read(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_master_context *context = filep->private_data;
	unsigned int len, iov_len, ring_len, buf_len;
	
	iov_len = iov_length(iov, nr_segs);
	if (unlikely(iov_len == 0)) return 0;
	
	if (mutex_lock_interruptible(&context->mutex))
		return -EINTR;
	
	/* If MSI is pending, deliver it */
	if (deliver_msi(context)) {
		/* We don't need a lock here, because no one will write to the msi_unread or
		 * msi[] while msi_pending stays high.
		 */
		len = min_t(unsigned int, context->msi_unread, iov_len);
		memcpy_toiovecend(iov, context->msi + sizeof(context->msi) - context->msi_unread, 0, len);
		context->msi_unread -= len;
	} else {
		ring_len = RING_READ_LEN(context);
		len = min_t(unsigned int, ring_len, iov_len);
		
		/* How far till we must wrap?  */
		buf_len = sizeof(context->buf) - RING_INDEX(context->sent);
		
		if (buf_len < len) {
			memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, buf_len);
			memcpy_toiovecend(iov, &context->buf[0],            buf_len, len-buf_len);
		} else {
			memcpy_toiovecend(iov, RING_POINTER(context, sent), 0, len);
		}
		context->sent = RING_POS(context->sent + len);
	}
	
	mutex_unlock(&context->mutex);
	
	/* Wake-up polling descriptors */
	wake_up_interruptible(&context->waitq);
	kill_fasync(&context->fasync, SIGIO, POLL_OUT);
	
	if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
		return -EAGAIN;
	
	return len;
}

static ssize_t char_master_aio_write(struct kiocb *iocb, const struct iovec *iov, unsigned long nr_segs, loff_t pos)
{
	struct file *filep = iocb->ki_filp;
	struct etherbone_master_context *context = filep->private_data;
	unsigned int len, iov_len, ring_len, buf_len;
	
	iov_len = iov_length(iov, nr_segs);
	if (unlikely(iov_len == 0)) return 0;
	
	if (mutex_lock_interruptible(&context->mutex))
		return -EINTR;
	
	ring_len = RING_WRITE_LEN(context);
	len = min_t(unsigned int, ring_len, iov_len);
	
	/* How far till we must wrap?  */
	buf_len = sizeof(context->buf) - RING_INDEX(context->received);
	
	if (buf_len < len) {
		memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, buf_len);
		memcpy_fromiovecend(&context->buf[0],                iov, buf_len, len-buf_len);
	} else {
		memcpy_fromiovecend(RING_POINTER(context, received), iov, 0, len);
	}
	context->received = RING_POS(context->received + len);
	
	/* Process buffers */
	etherbone_master_process(context);
	
	mutex_unlock(&context->mutex);
	
	/* Wake-up polling descriptors */
	wake_up_interruptible(&context->waitq);
	kill_fasync(&context->fasync, SIGIO, POLL_IN);
	
	if (len == 0 && (filep->f_flags & O_NONBLOCK) != 0)
		return -EAGAIN;
	
	return len;
}

static unsigned int char_master_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct etherbone_master_context *context = filep->private_data;
	
	poll_wait(filep, &context->waitq, wait);
	
	mutex_lock(&context->mutex);
	
	if (deliver_msi(context))         mask |= POLLIN  | POLLRDNORM;
	if (RING_READ_LEN (context) != 0) mask |= POLLIN  | POLLRDNORM;
	if (RING_WRITE_LEN(context) != 0) mask |= POLLOUT | POLLWRNORM;
	
	mutex_unlock(&context->mutex);
	
	return mask;
}

static int char_master_fasync(int fd, struct file *file, int on)
{
	struct etherbone_master_context* context = file->private_data;

        /* No locking - fasync_helper does its own locking */
        return fasync_helper(fd, file, on, &context->fasync);
}

static const struct file_operations etherbone_master_fops = {
        .owner          = THIS_MODULE,
        .llseek         = no_llseek,
        .read           = do_sync_read,
        .aio_read       = char_master_aio_read,
        .write          = do_sync_write,
        .aio_write      = char_master_aio_write,
        .open           = char_master_open,
        .poll           = char_master_poll,
        .release        = char_master_release,
        .fasync         = char_master_fasync,
};

int wishbone_register(struct wishbone* wb)
{
	struct list_head *list_pos;
	unsigned int devoff, i;
	
	INIT_LIST_HEAD(&wb->list);
	spin_lock_init(&wb->spinlock);
	for (i = 0; i < WISHBONE_MAX_MSI_OPEN; ++i) {
		wb->msi_map[i] = 0;
	}
	wb->msi_pending = 0;
	
	mutex_lock(&wishbone_mutex);
	
	/* Search the list for gaps, stopping past the gap.
	 * If we overflow the list (ie: not gaps), minor already points past end.
	 */
	devoff = 0;
	list_for_each(list_pos, &wishbone_list) {
		struct wishbone *entry =
			container_of(list_pos, struct wishbone, list);
		
		dev_t master_dev_tmp = 
		  MKDEV(
		    MAJOR(wishbone_master_dev_first),
		    MINOR(wishbone_master_dev_first) + devoff);
		
		if (entry->master_dev != master_dev_tmp) {
			/* We found a gap! */
			break;
		} else {
			/* Run out of minors? */
			if (devoff == max_devices-1) goto fail_out;
			
			/* Try the next minor */
			++devoff;
		}
	}
	
	/* Connect the file operations with the cdevs */
	cdev_init(&wb->master_cdev, &etherbone_master_fops);
	wb->master_cdev.owner = wb->wops->owner;
	
	wb->master_dev =
	  MKDEV(
	    MAJOR(wishbone_master_dev_first), 
	    MINOR(wishbone_master_dev_first) + devoff);
	
	/* Connect the major/minor number to the cdev */
	if (cdev_add(&wb->master_cdev, wb->master_dev, 1)) goto fail_out;
	
	/* Create the sysfs entry */
	wb->master_device = device_create(wishbone_master_class, wb->parent, wb->master_dev, NULL, "wbm%d", devoff);
	if (IS_ERR(wb->master_device)) goto fail_master_cdev;
	
	/* Insert the device into the gap */
	list_add_tail(&wb->list, list_pos);
	
	mutex_unlock(&wishbone_mutex);
	
	/* Startup the MSI queue */
	wishbone_slave_ready(wb);
	
	return 0;

fail_master_cdev:
	cdev_del(&wb->master_cdev);
fail_out:
	mutex_unlock(&wishbone_mutex);
	return -ENOMEM;
}

int wishbone_unregister(struct wishbone* wb)
{
	if (WARN_ON(list_empty(&wb->list)))
		return -EINVAL;
	
	mutex_lock(&wishbone_mutex);
	list_del(&wb->list);
	device_destroy(wishbone_master_class, wb->master_dev);
	cdev_del(&wb->master_cdev);
	mutex_unlock(&wishbone_mutex);
	
	return 0;
}

void wishbone_slave_ready(struct wishbone* wb)
{
	unsigned long flags;
	spin_lock_irqsave(&wb->spinlock, flags);
	advance_msi(wb);
	spin_unlock_irqrestore(&wb->spinlock, flags);
}

static int __init wishbone_init(void)
{
	int err;
	dev_t overflow;

	printk(KERN_NOTICE "wishbone: version " __stringify(GIT_REVISION) " loaded\n");
	
	overflow = MKDEV(0, max_devices-1);
	if (MINOR(overflow) != max_devices-1) {
		err = -ENOMEM;
		goto fail_last;
	}
	
	wishbone_master_class = class_create(THIS_MODULE, "wbm");
	if (IS_ERR(wishbone_master_class)) {
		err = PTR_ERR(wishbone_master_class);
		goto fail_last;
	}
	
	if (alloc_chrdev_region(&wishbone_master_dev_first, 0, max_devices, "wbm") < 0) {
		err = -EIO;
		goto fail_master_class;
	}
	
	return 0;

fail_master_class:
	class_destroy(wishbone_master_class);
fail_last:
	return err;
}

static void __exit wishbone_exit(void)
{
	unregister_chrdev_region(wishbone_master_dev_first, max_devices);
	class_destroy(wishbone_master_class);
}

MODULE_AUTHOR("Wesley W. Terpstra <w.terpstra@gsi.de>");
MODULE_DESCRIPTION("Wishbone character device class");
module_param(max_devices, int, 0644);
MODULE_PARM_DESC(max_devices, "Maximum number of attached wishbone devices");
MODULE_LICENSE("GPL");
MODULE_VERSION(WISHBONE_VERSION);

EXPORT_SYMBOL(wishbone_register);
EXPORT_SYMBOL(wishbone_unregister);
EXPORT_SYMBOL(wishbone_slave_ready);

module_init(wishbone_init);
module_exit(wishbone_exit);
