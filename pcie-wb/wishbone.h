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
	
	/* internal (guarded by global mutex--register/unregister): */
	struct list_head list;
	dev_t master_dev;
	struct cdev master_cdev;
	struct device *master_device;
	
	/* internal (EB-MSI mapping for this hardware) */
	struct mutex mutex;
	struct etherbone_master_context *msi_map[WISHBONE_MAX_MSI_OPEN];
	wb_data_t msi_data;
	int msi_pending;
};

#define RING_SIZE	8192
#define RING_INDEX(x)	((x) & (RING_SIZE-1))
#define RING_POS(x)	((x) & (RING_SIZE*2-1))

/* One per open of character device */
struct etherbone_master_context
{
	struct wishbone* wishbone;
	struct fasync_struct *fasync;
	struct mutex mutex;
	wait_queue_head_t waitq;
	
	enum { header, idle, cycle } state;
	unsigned int sent, processed, received; /* sent <= processed <= received */
	
	unsigned char buf[RING_SIZE]; /* Ring buffer */
	
	/* MSI resource ownership; -1 = nothing */
	int msi_index;
	/* MSI record data */
	unsigned char msi[sizeof(wb_data_t)*6];
	int msi_unread;
	int msi_pending;
};

#define RING_READ_LEN(ctx)   RING_POS((ctx)->processed - (ctx)->sent)
#define RING_PROC_LEN(ctx)   RING_POS((ctx)->received  - (ctx)->processed)
#define RING_WRITE_LEN(ctx)  RING_POS((ctx)->sent + RING_SIZE - (ctx)->received)
#define RING_POINTER(ctx, idx) (&(ctx)->buf[RING_INDEX((ctx)->idx)])

int wishbone_register(struct wishbone* wb);
int wishbone_unregister(struct wishbone* wb);

/* call when device has data pending. disable non-MSI interrupt generation before calling. */
void wishbone_slave_ready(struct wishbone* wb);

#endif
