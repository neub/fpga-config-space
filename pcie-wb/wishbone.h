#ifndef WISHBONE_H
#define WISHBONE_H

#include <linux/types.h>
#include <linux/list.h>
#include <linux/cdev.h>

#define WISHBONE_VERSION "0.1"
#define WISHBONE_MAX_DEVICES  32 /* default only */
#define WISHBONE_MAX_MSI_OPEN 16 /* fixed */

#define ETHERBONE_BCA	0x80
#define ETHERBONE_RCA	0x40
#define ETHERBONE_RFF	0x20
#define ETHERBONE_CYC	0x08
#define ETHERBONE_WCA	0x04
#define ETHERBONE_WFF	0x02

#define WBA_DATA	0x8000
#define WBA_ERR		0x8004

/* Implementation assumes these have the same size: */
typedef unsigned int wb_addr_t;
typedef unsigned int wb_data_t;

struct wishbone;
struct etherbone_master_context;

struct wishbone_request
{
	int write; /* 1=write, 0=read */
	wb_addr_t addr;
	wb_data_t data;
	unsigned char mask; /* byte-enable for write */
};

/* The wishbone driver guarantees that only one of these methods
 * is active at a time. Furthermore, they are only ever called in
 * a context where sleeping is safe.
 */
struct wishbone_operations 
{
	/* owning module */
	struct module *owner;
	
	/* master operations */
	void (*cycle)(struct wishbone *wb, int on);
	void (*byteenable)(struct wishbone *wb, unsigned char mask);
	void (*write)(struct wishbone *wb, wb_addr_t addr, wb_data_t);
	wb_data_t (*read)(struct wishbone *wb, wb_addr_t addr);
	wb_data_t (*read_cfg)(struct wishbone *wb, wb_addr_t addr);
	
	/* slave operations */
	int (*request)(struct wishbone *wb, struct wishbone_request*); /* 1=record filled, 0=none pending. re-enable non-MSI interrupts. */
	void (*reply)(struct wishbone *wb, int err, wb_data_t dat);
};

/* One per wishbone backend hardware */
struct wishbone 
{
	const struct wishbone_operations* wops;
	struct device *parent;
	wb_addr_t mask;
	
	/* internal (mutex guarding access to wops and msi_pending) */
	struct mutex device_mutex;
	/* internal (mutex held when MSIs are running) */
	struct mutex msi_mutex;
	
	/* internal (an unack'd MSI has been handed to userspace; guarded by device_mutex) */
	int msi_pending;
	
	/* internal (MSI mapping; guarded by msi_spinlock) */
	spinlock_t msi_spinlock;
	struct etherbone_master_context *msi_map[WISHBONE_MAX_MSI_OPEN];
	
	/* internal (character device; constant after creation) */
	dev_t master_dev;
	struct cdev master_cdev;
	struct device *master_device;
	
	/* internal (workqueue to dispatch MSI to correct master device) */
	struct work_struct msi_handler;
	struct workqueue_struct *msi_workqueue;
	
	/* internal (registration of the device; guarded by global wishbone_mutex) */
	struct list_head list;
};

#define RING_SIZE	8192
#define RING_INDEX(x)	((x) & (RING_SIZE-1))
#define RING_POS(x)	((x) & (RING_SIZE*2-1))

/* One per open of character device */
struct etherbone_master_context
{
	struct wishbone* wishbone;
	
	/* Buffer status; access requires holding context_mutex */
	struct mutex context_mutex;
	enum { header, idle, cycle } state; /* cycle state <=> wishbone->device_mutex held */
	unsigned int sent, processed, received; /* sent <= processed <= received */
	unsigned char buf[RING_SIZE]; /* Ring buffer */

	/* MSI buffer data; access requires holding context_mutex */
	unsigned char msi[sizeof(wb_data_t)*6];
	int msi_unread;
	int msi_pending;
	wb_data_t msi_data;
	
	/* Wakeup polling threads */
	struct fasync_struct *fasync;
	wait_queue_head_t waitq;
	
	/* MSI resource ownership; -1 = nothing; modification requires both context_mutex and msi_spinlock */
	int msi_index;
};

#define RING_READ_LEN(ctx)   RING_POS((ctx)->processed - (ctx)->sent)
#define RING_PROC_LEN(ctx)   RING_POS((ctx)->received  - (ctx)->processed)
#define RING_WRITE_LEN(ctx)  RING_POS((ctx)->sent + RING_SIZE - (ctx)->received)
#define RING_POINTER(ctx, idx) (&(ctx)->buf[RING_INDEX((ctx)->idx)])

int wishbone_register(struct wishbone* wb);
int wishbone_unregister(struct wishbone* wb); /* disable interrupts before calling this */

/* call when device has data pending. disable non-MSI interrupt generation before calling. */
void wishbone_slave_ready(struct wishbone* wb);

#endif
