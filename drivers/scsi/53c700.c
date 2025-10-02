// SPDX-License-Identifier: GPL-2.0-or-later

/* NCR (or Symbios) 53c700 and 53c700-66 Driver
 *
 * Copyright (C) 2001 by James.Bottomley@HansenPartnership.com
**-----------------------------------------------------------------------------
**
**
**-----------------------------------------------------------------------------
 */

/* Notes:
 *
 * This driver is designed exclusively for these chips (virtually the
 * earliest of the scripts engine chips).  They need their own drivers
 * because they are missing so many of the scripts and snazzy register
 * features of their elder brothers (the 710, 720 and 770).
 *
 * The 700 is the lowliest of the line, it can only do async SCSI.
 * The 700-66 can at least do synchronous SCSI up to 10MHz.
 *
 * The 700 chip has no host bus interface logic of its own.  However,
 * it is usually mapped to a location with well defined register
 * offsets.  Therefore, if you can determine the base address and the
 * irq your board incorporating this chip uses, you can probably use
 * this driver to run it (although you'll probably have to write a
 * minimal wrapper for the purpose---see the NCR_D700 driver for
 * details about how to do this).
 *
 *
 * TODO List:
 *
 * 1. Better statistics in the proc fs
 *
 * 2. Implement message queue (queues SCSI messages like commands) and make
 *    the abort and device reset functions use them.
 * */

/* CHANGELOG
 *
 * Version 2.8
 *
 * Fixed bad bug affecting tag starvation processing (previously the
 * driver would hang the system if too many tags starved.  Also fixed
 * bad bug having to do with 10 byte command processing and REQUEST
 * SENSE (the command would loop forever getting a transfer length
 * mismatch in the CMD phase).
 *
 * Version 2.7
 *
 * Fixed scripts problem which caused certain devices (notably CDRWs)
 * to hang on initial INQUIRY.  Updated NCR_700_readl/writel to use
 * __raw_readl/writel for parisc compatibility (Thomas
 * Bogendoerfer). Added missing SCp->request_bufflen initialisation
 * for sense requests (Ryan Bradetich).
 *
 * Version 2.6
 *
 * Following test of the 64 bit parisc kernel by Richard Hirst,
 * several problems have now been corrected.  Also adds support for
 * consistent memory allocation.
 *
 * Version 2.5
 *
 * More Compatibility changes for 710 (now actually works).  Enhanced
 * support for odd clock speeds which constrain SDTR negotiations.
 * correct cacheline separation for scsi messages and status for
 * incoherent architectures.  Use of the pci mapping functions on
 * buffers to begin support for 64 bit drivers.
 *
 * Version 2.4
 *
 * Added support for the 53c710 chip (in 53c700 emulation mode only---no
 * special 53c710 instructions or registers are used).
 *
 * Version 2.3
 *
 * More endianness/cache coherency changes.
 *
 * Better bad device handling (handles devices lying about tag
 * queueing support and devices which fail to provide sense data on
 * contingent allegiance conditions)
 *
 * Many thanks to Richard Hirst <rhirst@linuxcare.com> for patiently
 * debugging this driver on the parisc architecture and suggesting
 * many improvements and bug fixes.
 *
 * Thanks also go to Linuxcare Inc. for providing several PARISC
 * machines for me to debug the driver on.
 *
 * Version 2.2
 *
 * Made the driver mem or io mapped; added endian invariance; added
 * dma cache flushing operations for architectures which need it;
 * added support for more varied clocking speeds.
 *
 * Version 2.1
 *
 * Initial modularisation from the D700.  See NCR_D700.c for the rest of
 * the changelog.
 * */
#define NCR_700_VERSION "2.8"

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/completion.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pgtable.h>
#include <asm/dma.h>
#include <asm/io.h>
#include <asm/byteorder.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_dbg.h>
#include <scsi/scsi_eh.h>
#include <scsi/scsi_host.h>
#include <scsi/scsi_tcq.h>
#include <scsi/scsi_transport.h>
#include <scsi/scsi_transport_spi.h>

#include "53c700.h"

/* NOTE: For 64 bit drivers there are points in the code where we use
 * a non dereferenceable pointer to point to a structure in dma-able
 * memory (which is 32 bits) so that we can use all of the structure
 * operations but take the address at the end.  This macro allows us
 * to truncate the 64 bit pointer down to 32 bits without the compiler
 * complaining */
#define to32bit(x)	((__u32)((unsigned long)(x)))

#ifdef NCR_700_DEBUG
#define STATIC
#else
#define STATIC static
#endif

MODULE_AUTHOR("James Bottomley");
MODULE_DESCRIPTION("53c700 and 53c700-66 Driver");
MODULE_LICENSE("GPL");

/* This is the script */
#include "53c700_d.h"


STATIC int NCR_700_queuecommand(struct Scsi_Host *h, struct scsi_cmnd *);
STATIC int NCR_700_abort(struct scsi_cmnd * SCpnt);
STATIC int NCR_700_host_reset(struct scsi_cmnd * SCpnt);
STATIC void NCR_700_chip_setup(struct Scsi_Host *host);
STATIC void NCR_700_chip_reset(struct Scsi_Host *host);
STATIC int NCR_700_sdev_init(struct scsi_device *SDpnt);
STATIC int NCR_700_sdev_configure(struct scsi_device *SDpnt,
				  struct queue_limits *lim);
STATIC void NCR_700_sdev_destroy(struct scsi_device *SDpnt);
static int NCR_700_change_queue_depth(struct scsi_device *SDpnt, int depth);

STATIC const struct attribute_group *NCR_700_dev_groups[];

STATIC struct scsi_transport_template *NCR_700_transport_template = NULL;

static char *NCR_700_phase[] = {
	"",
	"after selection",
	"before command phase",
	"after command phase",
	"after status phase",
	"after data in phase",
	"after data out phase",
	"during data phase",
};

static char *NCR_700_condition[] = {
	"",
	"NOT MSG_OUT",
	"UNEXPECTED PHASE",
	"NOT MSG_IN",
	"UNEXPECTED MSG",
	"MSG_IN",
	"SDTR_MSG RECEIVED",
	"REJECT_MSG RECEIVED",
	"DISCONNECT_MSG RECEIVED",
	"MSG_OUT",
	"DATA_IN",

};

static char *NCR_700_fatal_messages[] = {
	"unexpected message after reselection",
	"still MSG_OUT after message injection",
	"not MSG_IN after selection",
	"Illegal message length received",
};

static char *NCR_700_SBCL_bits[] = {
	"IO ",
	"CD ",
	"MSG ",
	"ATN ",
	"SEL ",
	"BSY ",
	"ACK ",
	"REQ ",
};

static char *NCR_700_SBCL_to_phase[] = {
	"DATA_OUT",
	"DATA_IN",
	"CMD_OUT",
	"STATE",
	"ILLEGAL PHASE",
	"ILLEGAL PHASE",
	"MSG OUT",
	"MSG IN",
};

/* This translates the SDTR message offset and period to a value
 * which can be loaded into the SXFER_REG.
 *
 * NOTE: According to SCSI-2, the true transfer period (in ns) is
 *       actually four times this period value */
static inline __u8
NCR_700_offset_period_to_sxfer(struct NCR_700_Host_Parameters *hostdata,
			       __u8 offset, __u8 period)
{
	int XFERP;

	__u8 min_xferp = (hostdata->chip710
			  ? NCR_710_MIN_XFERP : NCR_700_MIN_XFERP);
	__u8 max_offset = (hostdata->chip710
			   ? NCR_710_MAX_OFFSET : NCR_700_MAX_OFFSET);

	if(offset == 0)
		return 0;

	if(period < hostdata->min_period) {
		printk(KERN_WARNING "53c700: Period %dns is less than this chip's minimum, setting to %d\n", period*4, NCR_700_MIN_PERIOD*4);
		period = hostdata->min_period;
	}
	XFERP = (period*4 * hostdata->sync_clock)/1000 - 4;
	if(offset > max_offset) {
		printk(KERN_WARNING "53c700: Offset %d exceeds chip maximum, setting to %d\n",
		       offset, max_offset);
		offset = max_offset;
	}
	if(XFERP < min_xferp) {
		XFERP =  min_xferp;
	}
	return (offset & 0x0f) | (XFERP & 0x07)<<4;
}

static inline __u8
NCR_700_get_SXFER(struct scsi_device *SDp)
{
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SDp->host->hostdata[0];

	return NCR_700_offset_period_to_sxfer(hostdata,
					      spi_offset(SDp->sdev_target),
					      spi_period(SDp->sdev_target));
}

static inline dma_addr_t virt_to_dma(struct NCR_700_Host_Parameters *h, void *p)
{
	return h->pScript + ((uintptr_t)p - (uintptr_t)h->script);
}

static inline void dma_sync_to_dev(struct NCR_700_Host_Parameters *h,
		void *addr, size_t size)
{
	if (h->noncoherent)
		dma_sync_single_for_device(h->dev, virt_to_dma(h, addr),
					   size, DMA_BIDIRECTIONAL);
}

static inline void dma_sync_from_dev(struct NCR_700_Host_Parameters *h,
		void *addr, size_t size)
{
	if (h->noncoherent)
		dma_sync_single_for_device(h->dev, virt_to_dma(h, addr), size,
					   DMA_BIDIRECTIONAL);
}

struct Scsi_Host *
NCR_700_detect(struct scsi_host_template *tpnt,
	       struct NCR_700_Host_Parameters *hostdata, struct device *dev)
{
	dma_addr_t pScript, pSlots;
	__u8 *memory;
	__u32 *script;
	struct Scsi_Host *host;
	static int banner = 0;
	int j;

	if (tpnt->sdev_groups == NULL)
		tpnt->sdev_groups = NCR_700_dev_groups;

	memory = dma_alloc_coherent(dev, TOTAL_MEM_SIZE, &pScript, GFP_KERNEL);
	if (!memory) {
		hostdata->noncoherent = 1;
		memory = dma_alloc_noncoherent(dev, TOTAL_MEM_SIZE, &pScript,
					 DMA_BIDIRECTIONAL, GFP_KERNEL);
	}
	if (!memory) {
		printk(KERN_ERR "53c700: Failed to allocate memory for driver, detaching\n");
		return NULL;
	}

	script = (__u32 *)memory;
	hostdata->msgin = memory + MSGIN_OFFSET;
	hostdata->msgout = memory + MSGOUT_OFFSET;
	hostdata->status = memory + STATUS_OFFSET;
	hostdata->slots = (struct NCR_700_command_slot *)(memory + SLOTS_OFFSET);
	hostdata->dev = dev;

	pSlots = pScript + SLOTS_OFFSET;

	/* Fill in the missing routines from the host template */
	tpnt->queuecommand = NCR_700_queuecommand;
	tpnt->eh_abort_handler = NCR_700_abort;
	tpnt->eh_host_reset_handler = NCR_700_host_reset;
	tpnt->can_queue = NCR_700_COMMAND_SLOTS_PER_HOST;
	tpnt->sg_tablesize = NCR_700_SG_SEGMENTS;
	tpnt->cmd_per_lun = NCR_700_CMD_PER_LUN;
	tpnt->sdev_configure = NCR_700_sdev_configure;
	tpnt->sdev_destroy = NCR_700_sdev_destroy;
	tpnt->sdev_init = NCR_700_sdev_init;
	tpnt->change_queue_depth = NCR_700_change_queue_depth;

	if(tpnt->name == NULL)
		tpnt->name = "53c700";
	if(tpnt->proc_name == NULL)
		tpnt->proc_name = "53c700";

	host = scsi_host_alloc(tpnt, 4);
	if (!host)
		return NULL;
	memset(hostdata->slots, 0, sizeof(struct NCR_700_command_slot)
	       * NCR_700_COMMAND_SLOTS_PER_HOST);
	for (j = 0; j < NCR_700_COMMAND_SLOTS_PER_HOST; j++) {
		dma_addr_t offset = (dma_addr_t)((unsigned long)&hostdata->slots[j].SG[0]
					  - (unsigned long)&hostdata->slots[0].SG[0]);
		hostdata->slots[j].pSG = (struct NCR_700_SG_List *)((unsigned long)(pSlots + offset));
		if(j == 0)
			hostdata->free_list = &hostdata->slots[j];
		else
			hostdata->slots[j-1].ITL_forw = &hostdata->slots[j];
		hostdata->slots[j].state = NCR_700_SLOT_FREE;
	}

	for (j = 0; j < ARRAY_SIZE(SCRIPT); j++)
		script[j] = bS_to_host(SCRIPT[j]);

	printk(KERN_DEBUG "53c700: Script loaded, size=%lu words, script@%p, pScript@0x%08x\n",
	       (unsigned long)ARRAY_SIZE(SCRIPT), script, (u32)pScript);

	/* adjust all labels to be bus physical */
	for (j = 0; j < PATCHES; j++)
		script[LABELPATCHES[j]] = bS_to_host(pScript + SCRIPT[LABELPATCHES[j]]);
	printk(KERN_DEBUG "53c700: Applied %d label patches\n", PATCHES);
	/* now patch up fixed addresses. */
	printk(KERN_DEBUG "53c700: Patching script addresses - MessageLocation@0x%08x, StatusAddress@0x%08x, ReceiveMsgAddress@0x%08x\n",
	       (u32)(pScript + MSGOUT_OFFSET), (u32)(pScript + STATUS_OFFSET), (u32)(pScript + MSGIN_OFFSET));
	script_patch_32(hostdata, script, MessageLocation,
			pScript + MSGOUT_OFFSET);
	script_patch_32(hostdata, script, StatusAddress,
			pScript + STATUS_OFFSET);
	script_patch_32(hostdata, script, ReceiveMsgAddress,
			pScript + MSGIN_OFFSET);

	hostdata->script = script;
	hostdata->pScript = pScript;
	dma_sync_single_for_device(hostdata->dev, pScript, sizeof(SCRIPT), DMA_TO_DEVICE);
	hostdata->state = NCR_700_HOST_FREE;
	hostdata->cmd = NULL;
	host->max_id = 8;
	host->max_lun = NCR_700_MAX_LUNS;
	BUG_ON(NCR_700_transport_template == NULL);
	host->transportt = NCR_700_transport_template;
	host->unique_id = (unsigned long)hostdata->base;
	hostdata->eh_complete = NULL;
	host->hostdata[0] = (unsigned long)hostdata;
	/* kick the chip */
	printk(KERN_DEBUG "53c700: Initializing chip, writing 0xff to CTEST9_REG\n");
	NCR_700_writeb(0xff, host, CTEST9_REG);
	if (hostdata->chip710)
		hostdata->rev = (NCR_700_readb(host, CTEST8_REG)>>4) & 0x0f;
	else
		hostdata->rev = (NCR_700_readb(host, CTEST7_REG)>>4) & 0x0f;
	hostdata->fast = (NCR_700_readb(host, CTEST9_REG) == 0);
	printk(KERN_DEBUG "53c700: Chip revision %d, fast=%d\n", hostdata->rev, hostdata->fast);
	if (banner == 0) {
		printk(KERN_NOTICE "53c700: Version " NCR_700_VERSION " By James.Bottomley@HansenPartnership.com\n");
		banner = 1;
	}
	printk(KERN_NOTICE "scsi%d: %s rev %d %s\n", host->host_no,
	       hostdata->chip710 ? "53c710" :
	       (hostdata->fast ? "53c700-66" : "53c700"),
	       hostdata->rev, hostdata->differential ?
	       "(Differential)" : "");
	printk(KERN_DEBUG "53c700: Host initialization - base=0x%p, irq=%d, clock=%dMHz\n",
	       hostdata->base, host->irq, hostdata->clock);
	/* reset the chip */
	NCR_700_chip_reset(host);

	if (scsi_add_host(host, dev)) {
		dev_printk(KERN_ERR, dev, "53c700: scsi_add_host failed\n");
		scsi_host_put(host);
		return NULL;
	}

	spi_signalling(host) = hostdata->differential ? SPI_SIGNAL_HVD :
		SPI_SIGNAL_SE;

	return host;
}

int
NCR_700_release(struct Scsi_Host *host)
{
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)host->hostdata[0];

	if (hostdata->noncoherent)
		dma_free_noncoherent(hostdata->dev, TOTAL_MEM_SIZE,
				hostdata->script, hostdata->pScript,
				DMA_BIDIRECTIONAL);
	else
		dma_free_coherent(hostdata->dev, TOTAL_MEM_SIZE,
				  hostdata->script, hostdata->pScript);
	return 1;
}

static inline __u8
NCR_700_identify(int can_disconnect, __u8 lun)
{
	return IDENTIFY_BASE |
		((can_disconnect) ? 0x40 : 0) |
		(lun & NCR_700_LUN_MASK);
}

/*
 * Function : static int data_residual (Scsi_Host *host)
 *
 * Purpose : return residual data count of what's in the chip.  If you
 * really want to know what this function is doing, it's almost a
 * direct transcription of the algorithm described in the 53c710
 * guide, except that the DBC and DFIFO registers are only 6 bits
 * wide on a 53c700.
 *
 * Inputs : host - SCSI host */
static inline int
NCR_700_data_residual (struct Scsi_Host *host) {
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)host->hostdata[0];
	int count, synchronous = 0;
	unsigned int ddir;

	if(hostdata->chip710) {
		count = ((NCR_700_readb(host, DFIFO_REG) & 0x7f) -
			 (NCR_700_readl(host, DBC_REG) & 0x7f)) & 0x7f;
	} else {
		count = ((NCR_700_readb(host, DFIFO_REG) & 0x3f) -
			 (NCR_700_readl(host, DBC_REG) & 0x3f)) & 0x3f;
	}

	if(hostdata->fast)
		synchronous = NCR_700_readb(host, SXFER_REG) & 0x0f;

	/* get the data direction */
	ddir = NCR_700_readb(host, CTEST0_REG) & 0x01;

	if (ddir) {
		/* Receive */
		if (synchronous)
			count += (NCR_700_readb(host, SSTAT2_REG) & 0xf0) >> 4;
		else
			if (NCR_700_readb(host, SSTAT1_REG) & SIDL_REG_FULL)
				++count;
	} else {
		/* Send */
		__u8 sstat = NCR_700_readb(host, SSTAT1_REG);
		if (sstat & SODL_REG_FULL)
			++count;
		if (synchronous && (sstat & SODR_REG_FULL))
			++count;
	}
#ifdef NCR_700_DEBUG
	if(count)
		printk("RESIDUAL IS %d (ddir %d)\n", count, ddir);
#endif
	return count;
}

/* print out the SCSI wires and corresponding phase from the SBCL register
 * in the chip */
static inline char *
sbcl_to_string(__u8 sbcl)
{
	int i;
	static char ret[256];

	ret[0]='\0';
	for(i=0; i<8; i++) {
		if((1<<i) & sbcl)
			strcat(ret, NCR_700_SBCL_bits[i]);
	}
	strcat(ret, NCR_700_SBCL_to_phase[sbcl & 0x07]);
	return ret;
}

static inline __u8
bitmap_to_number(__u8 bitmap)
{
	__u8 i;

	for(i=0; i<8 && !(bitmap &(1<<i)); i++)
		;
	return i;
}

/* Pull a slot off the free list */
STATIC struct NCR_700_command_slot *
find_empty_slot(struct NCR_700_Host_Parameters *hostdata)
{
	struct NCR_700_command_slot *slot = hostdata->free_list;

	if(slot == NULL) {
		/* sanity check */
		if(hostdata->command_slot_count != NCR_700_COMMAND_SLOTS_PER_HOST)
			printk(KERN_ERR "SLOTS FULL, but count is %d, should be %d\n", hostdata->command_slot_count, NCR_700_COMMAND_SLOTS_PER_HOST);
		return NULL;
	}

	if(slot->state != NCR_700_SLOT_FREE)
		/* should panic! */
		printk(KERN_ERR "BUSY SLOT ON FREE LIST!!!\n");


	hostdata->free_list = slot->ITL_forw;
	slot->ITL_forw = NULL;


	/* NOTE: set the state to busy here, not queued, since this
	 * indicates the slot is in use and cannot be run by the IRQ
	 * finish routine.  If we cannot queue the command when it
	 * is properly build, we then change to NCR_700_SLOT_QUEUED */
	slot->state = NCR_700_SLOT_BUSY;
	slot->flags = 0;
	hostdata->command_slot_count++;

	return slot;
}

STATIC void
free_slot(struct NCR_700_command_slot *slot,
	  struct NCR_700_Host_Parameters *hostdata)
{
	printk("53c700: >>> free_slot called for slot %p, cmnd=%p\n", slot, slot->cmnd);
	if((slot->state & NCR_700_SLOT_MASK) != NCR_700_SLOT_MAGIC) {
		printk(KERN_ERR "53c700: SLOT %p is not MAGIC!!!\n", slot);
	}
	if(slot->state == NCR_700_SLOT_FREE) {
		printk(KERN_ERR "53c700: SLOT %p is FREE!!!\n", slot);
	}

	slot->resume_offset = 0;
	slot->cmnd = NULL;
	slot->state = NCR_700_SLOT_FREE;
	slot->ITL_forw = hostdata->free_list;
	hostdata->free_list = slot;
	hostdata->command_slot_count--;
	printk("53c700: <<< free_slot completed for slot %p\n", slot);
}


/* This routine really does very little.  The command is indexed on
   the ITL and (if tagged) the ITLQ lists in _queuecommand */
STATIC void
save_for_reselection(struct NCR_700_Host_Parameters *hostdata,
		     struct scsi_cmnd *SCp, __u32 dsp)
{
	/* Its just possible that this gets executed twice */
	if(SCp != NULL) {
		struct NCR_700_command_slot *slot =
			(struct NCR_700_command_slot *)SCp->host_scribble;

		slot->resume_offset = dsp;
	}
	hostdata->state = NCR_700_HOST_FREE;
	hostdata->cmd = NULL;
}

STATIC inline void
NCR_700_unmap(struct NCR_700_Host_Parameters *hostdata, struct scsi_cmnd *SCp,
	      struct NCR_700_command_slot *slot)
{
	if(SCp->sc_data_direction != DMA_NONE &&
	   SCp->sc_data_direction != DMA_BIDIRECTIONAL)
		scsi_dma_unmap(SCp);
}

STATIC inline void
NCR_700_scsi_done(struct NCR_700_Host_Parameters *hostdata,
	       struct scsi_cmnd *SCp, int result)
{
	printk("53c700: >>> NCR_700_scsi_done ENTRY: hostdata->state=%s, hostdata->cmd=%p\n",
	       hostdata->state == NCR_700_HOST_FREE ? "FREE" :
	       hostdata->state == NCR_700_HOST_BUSY ? "BUSY" : "UNKNOWN",
	       hostdata->cmd);

	hostdata->state = NCR_700_HOST_FREE;
	hostdata->cmd = NULL;

	if(SCp != NULL) {
		struct NCR_700_command_slot *slot =
			(struct NCR_700_command_slot *)SCp->host_scribble;

		printk("53c700: SCSI command completed - target %d lun %llu, result=0x%08x\n",
		       SCp->device->id, SCp->device->lun, result);
		printk("53c700: Command was: CDB[0]=0x%02x, flags=0x%x, cmd_len=%d\n",
		       SCp->cmnd[0], slot ? slot->flags : 0, SCp->cmd_len);
		printk("53c700: Slot details: slot=%p, state=%d, SCp->host_scribble=%p\n",
		       slot, slot ? slot->state : -1, SCp->host_scribble);

		dma_unmap_single(hostdata->dev, slot->pCmd,
				 MAX_COMMAND_SIZE, DMA_TO_DEVICE);
		if (slot->flags == NCR_700_FLAG_AUTOSENSE) {
			char *cmnd = NCR_700_get_sense_cmnd(SCp->device);

			printk(KERN_DEBUG "53c700: Autosense command completed, original result was 0x%02x\n", cmnd[7]);
			dma_unmap_single(hostdata->dev, slot->dma_handle,
					 SCSI_SENSE_BUFFERSIZE, DMA_FROM_DEVICE);
			/* restore the old result if the request sense was
			 * successful */
			if (result == 0)
				result = cmnd[7];
			/* restore the original length */
			SCp->cmd_len = cmnd[8];
		} else
			NCR_700_unmap(hostdata, SCp, slot);

		free_slot(slot, hostdata);
#ifdef NCR_700_DEBUG
		if(NCR_700_get_depth(SCp->device) == 0 ||
		   NCR_700_get_depth(SCp->device) > SCp->device->queue_depth)
			printk(KERN_ERR "Invalid depth in NCR_700_scsi_done(): %d\n",
			       NCR_700_get_depth(SCp->device));
#endif /* NCR_700_DEBUG */
		NCR_700_set_depth(SCp->device, NCR_700_get_depth(SCp->device) - 1);

		SCp->host_scribble = NULL;
		SCp->result = result;
		printk("53c700: [%lu] About to call scsi_done() for target %d lun %llu, result=0x%08x\n",
		       jiffies, SCp->device->id, SCp->device->lun, result);
		scsi_done(SCp);
		printk("53c700: [%lu] <<< scsi_done() completed for target %d lun %llu\n",
		       jiffies, SCp->device->id, SCp->device->lun);
	} else {
		printk("53c700: NCR_700_scsi_done called with NULL SCp!\n");
		printk(KERN_ERR "53c700: SCSI DONE HAS NULL SCp\n");
	}
	printk("53c700: <<< NCR_700_scsi_done EXIT\n");
}


STATIC void
NCR_700_internal_bus_reset(struct Scsi_Host *host)
{
	/* Bus reset */
	NCR_700_writeb(ASSERT_RST, host, SCNTL1_REG);
	udelay(50);
	NCR_700_writeb(0, host, SCNTL1_REG);

}

STATIC void
NCR_700_chip_setup(struct Scsi_Host *host)
{
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)host->hostdata[0];
	__u8 min_period;
	__u8 min_xferp = (hostdata->chip710 ? NCR_710_MIN_XFERP : NCR_700_MIN_XFERP);

	printk(KERN_DEBUG "53c700: Setting up %s chip (rev %d), clock=%dMHz\n",
	       hostdata->chip710 ? "53c710" : "53c700", hostdata->rev, hostdata->clock);

	if(hostdata->chip710) {
		__u8 burst_disable = 0;
		__u8 burst_length = 0;

		printk(KERN_DEBUG "53c700: Configuring 53c710 mode with burst_length=%d\n",
		       hostdata->burst_length);

		switch (hostdata->burst_length) {
			case 1:
			        burst_length = BURST_LENGTH_1;
			        break;
			case 2:
			        burst_length = BURST_LENGTH_2;
			        break;
			case 4:
			        burst_length = BURST_LENGTH_4;
			        break;
			case 8:
			        burst_length = BURST_LENGTH_8;
			        break;
			default:
			        burst_disable = BURST_DISABLE;
			        break;
		}
		hostdata->dcntl_extra |= COMPAT_700_MODE;

		printk(KERN_DEBUG "53c700: Writing registers - DCNTL=0x%02x, DMODE=0x%02x, CTEST7=0x%02x\n",
		       hostdata->dcntl_extra, burst_length | hostdata->dmode_extra,
		       burst_disable | hostdata->ctest7_extra | (hostdata->differential ? DIFF : 0));

		NCR_700_writeb(hostdata->dcntl_extra, host, DCNTL_REG);
		NCR_700_writeb(burst_length | hostdata->dmode_extra,
			       host, DMODE_710_REG);
		NCR_700_writeb(burst_disable | hostdata->ctest7_extra |
			       (hostdata->differential ? DIFF : 0),
			       host, CTEST7_REG);
		NCR_700_writeb(BTB_TIMER_DISABLE, host, CTEST0_REG);
		NCR_700_writeb(FULL_ARBITRATION | ENABLE_PARITY | PARITY
			       | AUTO_ATN, host, SCNTL0_REG);
	} else {
		printk(KERN_DEBUG "53c700: Configuring 53c700 mode, fast=%d, differential=%d\n",
		       hostdata->fast, hostdata->differential);

		NCR_700_writeb(BURST_LENGTH_8 | hostdata->dmode_extra,
			       host, DMODE_700_REG);
		NCR_700_writeb(hostdata->differential ?
			       DIFF : 0, host, CTEST7_REG);
		if(hostdata->fast) {
			/* this is for 700-66, does nothing on 700 */
			NCR_700_writeb(LAST_DIS_ENBL | ENABLE_ACTIVE_NEGATION
				       | GENERATE_RECEIVE_PARITY, host,
				       CTEST8_REG);
		} else {
			NCR_700_writeb(FULL_ARBITRATION | ENABLE_PARITY
				       | PARITY | AUTO_ATN, host, SCNTL0_REG);
		}
	}

	NCR_700_writeb(1 << host->this_id, host, SCID_REG);
	NCR_700_writeb(0, host, SBCL_REG);
	NCR_700_writeb(ASYNC_OPERATION, host, SXFER_REG);

	NCR_700_writeb(PHASE_MM_INT | SEL_TIMEOUT_INT | GROSS_ERR_INT | UX_DISC_INT
	     | RST_INT | PAR_ERR_INT | SELECT_INT, host, SIEN_REG);

	NCR_700_writeb(ABORT_INT | INT_INST_INT | ILGL_INST_INT, host, DIEN_REG);
	NCR_700_writeb(ENABLE_SELECT, host, SCNTL1_REG);
	if(hostdata->clock > 75) {
		printk(KERN_ERR "53c700: Clock speed %dMHz is too high: 75Mhz is the maximum this chip can be driven at\n", hostdata->clock);
		/* do the best we can, but the async clock will be out
		 * of spec: sync divider 2, async divider 3 */
		DEBUG(("53c700: sync 2 async 3\n"));
		NCR_700_writeb(SYNC_DIV_2_0, host, SBCL_REG);
		NCR_700_writeb(ASYNC_DIV_3_0 | hostdata->dcntl_extra, host, DCNTL_REG);
		hostdata->sync_clock = hostdata->clock/2;
	} else	if(hostdata->clock > 50  && hostdata->clock <= 75) {
		/* sync divider 1.5, async divider 3 */
		DEBUG(("53c700: sync 1.5 async 3\n"));
		NCR_700_writeb(SYNC_DIV_1_5, host, SBCL_REG);
		NCR_700_writeb(ASYNC_DIV_3_0 | hostdata->dcntl_extra, host, DCNTL_REG);
		hostdata->sync_clock = hostdata->clock*2;
		hostdata->sync_clock /= 3;

	} else if(hostdata->clock > 37 && hostdata->clock <= 50) {
		/* sync divider 1, async divider 2 */
		DEBUG(("53c700: sync 1 async 2\n"));
		NCR_700_writeb(SYNC_DIV_1_0, host, SBCL_REG);
		NCR_700_writeb(ASYNC_DIV_2_0 | hostdata->dcntl_extra, host, DCNTL_REG);
		hostdata->sync_clock = hostdata->clock;
	} else if(hostdata->clock > 25 && hostdata->clock <=37) {
		/* sync divider 1, async divider 1.5 */
		DEBUG(("53c700: sync 1 async 1.5\n"));
		NCR_700_writeb(SYNC_DIV_1_0, host, SBCL_REG);
		NCR_700_writeb(ASYNC_DIV_1_5 | hostdata->dcntl_extra, host, DCNTL_REG);
		hostdata->sync_clock = hostdata->clock;
	} else {
		DEBUG(("53c700: sync 1 async 1\n"));
		NCR_700_writeb(SYNC_DIV_1_0, host, SBCL_REG);
		NCR_700_writeb(ASYNC_DIV_1_0 | hostdata->dcntl_extra, host, DCNTL_REG);
		/* sync divider 1, async divider 1 */
		hostdata->sync_clock = hostdata->clock;
	}
	/* Calculate the actual minimum period that can be supported
	 * by our synchronous clock speed.  See the 710 manual for
	 * exact details of this calculation which is based on a
	 * setting of the SXFER register */
	min_period = 1000*(4+min_xferp)/(4*hostdata->sync_clock);
	hostdata->min_period = NCR_700_MIN_PERIOD;
	if(min_period > NCR_700_MIN_PERIOD)
		hostdata->min_period = min_period;
}

STATIC void
NCR_700_chip_reset(struct Scsi_Host *host)
{
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)host->hostdata[0];
	if(hostdata->chip710) {
		printk(KERN_DEBUG "53c700: Issuing 710 software reset via ISTAT_REG\n");
		NCR_700_writeb(SOFTWARE_RESET_710, host, ISTAT_REG);
		udelay(100);

		NCR_700_writeb(0, host, ISTAT_REG);
		printk(KERN_DEBUG "53c700: 710 software reset completed\n");
	} else {
		printk(KERN_DEBUG "53c700: Issuing 700 software reset via DCNTL_REG\n");
		NCR_700_writeb(SOFTWARE_RESET, host, DCNTL_REG);
		udelay(100);

		NCR_700_writeb(0, host, DCNTL_REG);
		printk(KERN_DEBUG "53c700: 700 software reset completed\n");
	}

	mdelay(1000);

	NCR_700_chip_setup(host);
}

/* The heart of the message processing engine is that the instruction
 * immediately after the INT is the normal case (and so must be CLEAR
 * ACK).  If we want to do something else, we call that routine in
 * scripts and set temp to be the normal case + 8 (skipping the CLEAR
 * ACK) so that the routine returns correctly to resume its activity
 * */
STATIC __u32
process_extended_message(struct Scsi_Host *host,
			 struct NCR_700_Host_Parameters *hostdata,
			 struct scsi_cmnd *SCp, __u32 dsp, __u32 dsps)
{
	__u32 resume_offset = dsp, temp = dsp + 8;
	__u8 pun = 0xff, lun = 0xff;

	if(SCp != NULL) {
		pun = SCp->device->id;
		lun = SCp->device->lun;
	}
	printk("53c700: Processing message - dsps=0x%08x, msg[0]=0x%02x\n",
	       dsps, hostdata->msgin[0]);

	switch(hostdata->msgin[2]) {
	case A_SDTR_MSG:
		if(SCp != NULL && NCR_700_is_flag_set(SCp->device, NCR_700_DEV_BEGIN_SYNC_NEGOTIATION)) {
			struct scsi_target *starget = SCp->device->sdev_target;
			__u8 period = hostdata->msgin[3];
			__u8 offset = hostdata->msgin[4];

			if(offset == 0 || period == 0) {
				offset = 0;
				period = 0;
			}

			spi_offset(starget) = offset;
			spi_period(starget) = period;

			if(NCR_700_is_flag_set(SCp->device, NCR_700_DEV_PRINT_SYNC_NEGOTIATION)) {
				spi_display_xfer_agreement(starget);
				NCR_700_clear_flag(SCp->device, NCR_700_DEV_PRINT_SYNC_NEGOTIATION);
			}

			NCR_700_set_flag(SCp->device, NCR_700_DEV_NEGOTIATED_SYNC);
			NCR_700_clear_flag(SCp->device, NCR_700_DEV_BEGIN_SYNC_NEGOTIATION);

			NCR_700_writeb(NCR_700_get_SXFER(SCp->device),
				       host, SXFER_REG);

		} else {
			/* SDTR message out of the blue, reject it */
			shost_printk(KERN_WARNING, host,
				"Unexpected SDTR msg\n");
			hostdata->msgout[0] = A_REJECT_MSG;
			dma_sync_to_dev(hostdata, hostdata->msgout, 1);
			script_patch_16(hostdata, hostdata->script,
			                MessageCount, 1);
			/* SendMsgOut returns, so set up the return
			 * address */
			resume_offset = hostdata->pScript + Ent_SendMessageWithATN;
		}
		break;

	case A_WDTR_MSG:
		printk(KERN_INFO "scsi%d: (%d:%d), Unsolicited WDTR after CMD, Rejecting\n",
		       host->host_no, pun, lun);
		hostdata->msgout[0] = A_REJECT_MSG;
		dma_sync_to_dev(hostdata, hostdata->msgout, 1);
		script_patch_16(hostdata, hostdata->script, MessageCount, 1);
		resume_offset = hostdata->pScript + Ent_SendMessageWithATN;

		break;

	default:
		printk(KERN_INFO "scsi%d (%d:%d): Unexpected message %s: ",
		       host->host_no, pun, lun,
		       NCR_700_phase[(dsps & 0xf00) >> 8]);
		spi_print_msg(hostdata->msgin);
		printk("\n");
		/* just reject it */
		hostdata->msgout[0] = A_REJECT_MSG;
		dma_sync_to_dev(hostdata, hostdata->msgout, 1);
		script_patch_16(hostdata, hostdata->script, MessageCount, 1);
		/* SendMsgOut returns, so set up the return
		 * address */
		resume_offset = hostdata->pScript + Ent_SendMessageWithATN;
	}
	NCR_700_writel(temp, host, TEMP_REG);
	return resume_offset;
}

STATIC __u32
process_message(struct Scsi_Host *host,	struct NCR_700_Host_Parameters *hostdata,
		struct scsi_cmnd *SCp, __u32 dsp, __u32 dsps)
{
	/* work out where to return to */
	__u32 temp = dsp + 8, resume_offset = dsp;
	__u8 pun = 0xff, lun = 0xff;

	if(SCp != NULL) {
		pun = SCp->device->id;
		lun = SCp->device->lun;
	}

	printk(KERN_DEBUG "53c700: Processing message - dsps=0x%08x, msg[0]=0x%02x for target %d lun %d\n",
	       dsps, hostdata->msgin[0], pun, lun);
	printk(KERN_DEBUG "53c700: Full message buffer: %02x %02x %02x %02x %02x %02x %02x %02x\n",
	       hostdata->msgin[0], hostdata->msgin[1], hostdata->msgin[2], hostdata->msgin[3],
	       hostdata->msgin[4], hostdata->msgin[5], hostdata->msgin[6], hostdata->msgin[7]);

#ifdef NCR_700_DEBUG
	printk("scsi%d (%d:%d): message %s: ", host->host_no, pun, lun,
	       NCR_700_phase[(dsps & 0xf00) >> 8]);
	spi_print_msg(hostdata->msgin);
	printk("\n");
#endif

	switch(hostdata->msgin[0]) {

	case A_EXTENDED_MSG:
		resume_offset =  process_extended_message(host, hostdata, SCp,
							  dsp, dsps);
		break;

	case A_REJECT_MSG:
		if(SCp != NULL && NCR_700_is_flag_set(SCp->device, NCR_700_DEV_BEGIN_SYNC_NEGOTIATION)) {
			/* Rejected our sync negotiation attempt */
			spi_period(SCp->device->sdev_target) =
				spi_offset(SCp->device->sdev_target) = 0;
			NCR_700_set_flag(SCp->device, NCR_700_DEV_NEGOTIATED_SYNC);
			NCR_700_clear_flag(SCp->device, NCR_700_DEV_BEGIN_SYNC_NEGOTIATION);
		} else if(SCp != NULL && NCR_700_get_tag_neg_state(SCp->device) == NCR_700_DURING_TAG_NEGOTIATION) {
			/* rejected our first simple tag message */
			scmd_printk(KERN_WARNING, SCp,
				"Rejected first tag queue attempt, turning off tag queueing\n");
			/* we're done negotiating */
			NCR_700_set_tag_neg_state(SCp->device, NCR_700_FINISHED_TAG_NEGOTIATION);
			hostdata->tag_negotiated &= ~(1<<scmd_id(SCp));

			SCp->device->tagged_supported = 0;
			SCp->device->simple_tags = 0;
			scsi_change_queue_depth(SCp->device, host->cmd_per_lun);
		} else {
			shost_printk(KERN_WARNING, host,
				"(%d:%d) Unexpected REJECT Message %s\n",
			       pun, lun,
			       NCR_700_phase[(dsps & 0xf00) >> 8]);
			/* however, just ignore it */
		}
		break;

	case A_PARITY_ERROR_MSG:
		printk(KERN_ERR "scsi%d (%d:%d) Parity Error!\n", host->host_no,
		       pun, lun);
		NCR_700_internal_bus_reset(host);
		break;
	case A_SIMPLE_TAG_MSG:
		printk(KERN_INFO "scsi%d (%d:%d) SIMPLE TAG %d %s\n", host->host_no,
		       pun, lun, hostdata->msgin[1],
		       NCR_700_phase[(dsps & 0xf00) >> 8]);
		/* just ignore it */
		break;
	default:
		printk(KERN_INFO "scsi%d (%d:%d): Unexpected message %s: ",
		       host->host_no, pun, lun,
		       NCR_700_phase[(dsps & 0xf00) >> 8]);

		spi_print_msg(hostdata->msgin);
		printk("\n");
		/* just reject it */
		hostdata->msgout[0] = A_REJECT_MSG;
		dma_sync_to_dev(hostdata, hostdata->msgout, 1);
		script_patch_16(hostdata, hostdata->script, MessageCount, 1);
		/* SendMsgOut returns, so set up the return
		 * address */
		resume_offset = hostdata->pScript + Ent_SendMessageWithATN;

		break;
	}
	NCR_700_writel(temp, host, TEMP_REG);
	/* set us up to receive another message */
	dma_sync_from_dev(hostdata, hostdata->msgin, MSG_ARRAY_SIZE);
	return resume_offset;
}

STATIC __u32
process_script_interrupt(__u32 dsps, __u32 dsp, struct scsi_cmnd *SCp,
			 struct Scsi_Host *host,
			 struct NCR_700_Host_Parameters *hostdata)
{
	__u32 resume_offset = 0;
	__u8 pun = 0xff, lun=0xff;

	printk("53c700: process_script_interrupt called - DSPS=0x%08x, DSP=0x%08x\n", dsps, dsp);

	if(SCp != NULL) {
		pun = SCp->device->id;
		lun = SCp->device->lun;
		printk("53c700: Processing command for target %d lun %d\n", pun, lun);
	} else {
		printk("53c700: Processing interrupt with no active command\n");
		printk("53c700: WARNING: interrupt with null SCp, dsps=0x%08x\n", dsps);
	}

	if(dsps == A_GOOD_STATUS_AFTER_STATUS) {
		printk("53c700: Good status received, status=0x%02x\n", hostdata->status[0]);
		printk("53c700: GOOD_STATUS_AFTER_STATUS detected - processing command completion\n");
		printk("53c700: Command completion - status=0x%02x, SCp=%p\n",
		       hostdata->status[0], SCp);
		DEBUG(("  COMMAND COMPLETE, status=%02x\n",
		       hostdata->status[0]));
		/* OK, if TCQ still under negotiation, we now know it works */
		if (NCR_700_get_tag_neg_state(SCp->device) == NCR_700_DURING_TAG_NEGOTIATION)
			NCR_700_set_tag_neg_state(SCp->device,
						  NCR_700_FINISHED_TAG_NEGOTIATION);

		/* check for contingent allegiance conditions */
		if (hostdata->status[0] == SAM_STAT_CHECK_CONDITION ||
		    hostdata->status[0] == SAM_STAT_COMMAND_TERMINATED) {
			struct NCR_700_command_slot *slot =
				(struct NCR_700_command_slot *)SCp->host_scribble;
			if(slot->flags == NCR_700_FLAG_AUTOSENSE) {
				/* OOPS: bad device, returning another
				 * contingent allegiance condition */
				scmd_printk(KERN_ERR, SCp,
					"broken device is looping in contingent allegiance: ignoring\n");
				NCR_700_scsi_done(hostdata, SCp, hostdata->status[0]);
			} else {
				char *cmnd =
					NCR_700_get_sense_cmnd(SCp->device);
#ifdef NCR_DEBUG
				scsi_print_command(SCp);
				printk("  cmd %p has status %d, requesting sense\n",
				       SCp, hostdata->status[0]);
#endif
				/* we can destroy the command here
				 * because the contingent allegiance
				 * condition will cause a retry which
				 * will re-copy the command from the
				 * saved data_cmnd.  We also unmap any
				 * data associated with the command
				 * here */
				NCR_700_unmap(hostdata, SCp, slot);
				dma_unmap_single(hostdata->dev, slot->pCmd,
						 MAX_COMMAND_SIZE,
						 DMA_TO_DEVICE);

				cmnd[0] = REQUEST_SENSE;
				cmnd[1] = (lun & 0x7) << 5;
				cmnd[2] = 0;
				cmnd[3] = 0;
				cmnd[4] = SCSI_SENSE_BUFFERSIZE;
				cmnd[5] = 0;
				/* Here's a quiet hack: the
				 * REQUEST_SENSE command is six bytes,
				 * so store a flag indicating that
				 * this was an internal sense request
				 * and the original status at the end
				 * of the command */
				cmnd[6] = NCR_700_INTERNAL_SENSE_MAGIC;
				cmnd[7] = hostdata->status[0];
				cmnd[8] = SCp->cmd_len;
				SCp->cmd_len = 6; /* command length for
						   * REQUEST_SENSE */
				slot->pCmd = dma_map_single(hostdata->dev, cmnd, MAX_COMMAND_SIZE, DMA_TO_DEVICE);
				slot->dma_handle = dma_map_single(hostdata->dev, SCp->sense_buffer, SCSI_SENSE_BUFFERSIZE, DMA_FROM_DEVICE);
				slot->SG[0].ins = bS_to_host(SCRIPT_MOVE_DATA_IN | SCSI_SENSE_BUFFERSIZE);
				slot->SG[0].pAddr = bS_to_host(slot->dma_handle);
				slot->SG[1].ins = bS_to_host(SCRIPT_RETURN);
				slot->SG[1].pAddr = 0;
				slot->resume_offset = hostdata->pScript;
				dma_sync_to_dev(hostdata, slot->SG, sizeof(slot->SG[0])*2);
				dma_sync_from_dev(hostdata, SCp->sense_buffer, SCSI_SENSE_BUFFERSIZE);

				/* queue the command for reissue */
				slot->state = NCR_700_SLOT_QUEUED;
				slot->flags = NCR_700_FLAG_AUTOSENSE;
				hostdata->state = NCR_700_HOST_FREE;
				hostdata->cmd = NULL;
			}
		} else {
			// Currently rely on the mid layer evaluation
			// of the tag queuing capability
			//
			//if(status_byte(hostdata->status[0]) == GOOD &&
			//   SCp->cmnd[0] == INQUIRY && SCp->use_sg == 0) {
			//	/* Piggy back the tag queueing support
			//	 * on this command */
			//	dma_sync_single_for_cpu(hostdata->dev,
			//			    slot->dma_handle,
			//			    SCp->request_bufflen,
			//			    DMA_FROM_DEVICE);
			//	if(((char *)SCp->request_buffer)[7] & 0x02) {
			//		scmd_printk(KERN_INFO, SCp,
			//		     "Enabling Tag Command Queuing\n");
			//		hostdata->tag_negotiated |= (1<<scmd_id(SCp));
			//		NCR_700_set_flag(SCp->device, NCR_700_DEV_BEGIN_TAG_QUEUEING);
			//	} else {
			//		NCR_700_clear_flag(SCp->device, NCR_700_DEV_BEGIN_TAG_QUEUEING);
			//		hostdata->tag_negotiated &= ~(1<<scmd_id(SCp));
			//	}
			//}
			printk("53c700: About to call NCR_700_scsi_done for completed command\n");
			NCR_700_scsi_done(hostdata, SCp, hostdata->status[0]);
			printk("53c700: NCR_700_scsi_done completed - command should be done\n");
		}
	} else if((dsps & 0xfffff0f0) == A_UNEXPECTED_PHASE) {
		__u8 i = (dsps & 0xf00) >> 8;

		printk("53c700: UNEXPECTED PHASE interrupt - dsps=0x%08x, phase=%s\n",
		       dsps, NCR_700_phase[i]);
		scmd_printk(KERN_ERR, SCp, "UNEXPECTED PHASE %s (%s)\n",
		       NCR_700_phase[i],
		       sbcl_to_string(NCR_700_readb(host, SBCL_REG)));
		scmd_printk(KERN_ERR, SCp, "         len = %d, cmd =",
			SCp->cmd_len);
		scsi_print_command(SCp);

		NCR_700_internal_bus_reset(host);
	} else if((dsps & 0xfffff000) == A_FATAL) {
		int i = (dsps & 0xfff);

		printk("53c700: FATAL ERROR interrupt - dsps=0x%08x, error_code=%d\n",
		       dsps, i);
		printk(KERN_ERR "scsi%d: (%d:%d) FATAL ERROR: %s\n",
		       host->host_no, pun, lun, NCR_700_fatal_messages[i]);
		if(dsps == A_FATAL_ILLEGAL_MSG_LENGTH) {
			printk(KERN_ERR "     msg begins %02x %02x\n",
			       hostdata->msgin[0], hostdata->msgin[1]);
		}
		NCR_700_internal_bus_reset(host);
	} else if((dsps & 0xfffff0f0) == A_DISCONNECT) {
#ifdef NCR_700_DEBUG
		__u8 i = (dsps & 0xf00) >> 8;

		printk("scsi%d: (%d:%d), DISCONNECTED (%d) %s\n",
		       host->host_no, pun, lun,
		       i, NCR_700_phase[i]);
#endif
		printk("53c700: DISCONNECT interrupt - dsps=0x%08x, saving for reselection\n", dsps);
		save_for_reselection(hostdata, SCp, dsp);

	} else if(dsps == A_RESELECTION_IDENTIFIED) {
		__u8 lun;
		struct NCR_700_command_slot *slot;
		__u8 reselection_id = hostdata->reselection_id;
		struct scsi_device *SDp;

		printk("53c700: RESELECTION_IDENTIFIED interrupt - reselection_id=%d\n", reselection_id);

		lun = hostdata->msgin[0] & 0x1f;

		hostdata->reselection_id = 0xff;
		printk("53c700: Reselection by target %d lun %d\n", reselection_id, lun);
		DEBUG(("scsi%d: (%d:%d) RESELECTED!\n",
		       host->host_no, reselection_id, lun));
		/* clear the reselection indicator */
		SDp = __scsi_device_lookup(host, 0, reselection_id, lun);
		if(unlikely(SDp == NULL)) {
			printk(KERN_ERR "scsi%d: (%d:%d) HAS NO device\n",
			       host->host_no, reselection_id, lun);
			BUG();
		}
		if(hostdata->msgin[1] == A_SIMPLE_TAG_MSG) {
			struct scsi_cmnd *SCp;

			SCp = scsi_host_find_tag(SDp->host, hostdata->msgin[2]);
			if(unlikely(SCp == NULL)) {
				printk(KERN_ERR "scsi%d: (%d:%d) no saved request for tag %d\n",
				       host->host_no, reselection_id, lun, hostdata->msgin[2]);
				BUG();
			}

			slot = (struct NCR_700_command_slot *)SCp->host_scribble;
			DDEBUG(KERN_DEBUG, SDp,
				"reselection is tag %d, slot %p(%d)\n",
				hostdata->msgin[2], slot, slot->tag);
		} else {
			struct NCR_700_Device_Parameters *p = SDp->hostdata;
			struct scsi_cmnd *SCp = p->current_cmnd;

			if(unlikely(SCp == NULL)) {
				sdev_printk(KERN_ERR, SDp,
					"no saved request for untagged cmd\n");
				BUG();
			}
			slot = (struct NCR_700_command_slot *)SCp->host_scribble;
		}

		if(slot == NULL) {
			printk(KERN_ERR "scsi%d: (%d:%d) RESELECTED but no saved command (MSG = %02x %02x %02x)!!\n",
			       host->host_no, reselection_id, lun,
			       hostdata->msgin[0], hostdata->msgin[1],
			       hostdata->msgin[2]);
		} else {
			if(hostdata->state != NCR_700_HOST_BUSY)
				printk(KERN_ERR "scsi%d: FATAL, host not busy during valid reselection!\n",
				       host->host_no);
			resume_offset = slot->resume_offset;
			hostdata->cmd = slot->cmnd;

			/* re-patch for this command */
			script_patch_32_abs(hostdata, hostdata->script,
			                    CommandAddress, slot->pCmd);
			script_patch_16(hostdata, hostdata->script,
					CommandCount, slot->cmnd->cmd_len);
			script_patch_32_abs(hostdata, hostdata->script,
			                    SGScriptStartAddress,
					    to32bit(&slot->pSG[0].ins));

			/* Note: setting SXFER only works if we're
			 * still in the MESSAGE phase, so it is vital
			 * that ACK is still asserted when we process
			 * the reselection message.  The resume offset
			 * should therefore always clear ACK */
			NCR_700_writeb(NCR_700_get_SXFER(hostdata->cmd->device),
				       host, SXFER_REG);
			dma_sync_from_dev(hostdata, hostdata->msgin,
				       MSG_ARRAY_SIZE);
			dma_sync_to_dev(hostdata, hostdata->msgout,
				       MSG_ARRAY_SIZE);
			/* I'm just being paranoid here, the command should
			 * already have been flushed from the cache */
			dma_sync_to_dev(hostdata, slot->cmnd->cmnd,
				       slot->cmnd->cmd_len);



		}
	} else if(dsps == A_RESELECTED_DURING_SELECTION) {

		/* This section is full of debugging code because I've
		 * never managed to reach it.  I think what happens is
		 * that, because the 700 runs with selection
		 * interrupts enabled the whole time that we take a
		 * selection interrupt before we manage to get to the
		 * reselected script interrupt */

		__u8 reselection_id = NCR_700_readb(host, SFBR_REG);
		struct NCR_700_command_slot *slot;

		/* Take out our own ID */
		reselection_id &= ~(1<<host->this_id);

		/* I've never seen this happen, so keep this as a printk rather
		 * than a debug */
		printk(KERN_INFO "scsi%d: (%d:%d) RESELECTION DURING SELECTION, dsp=%08x[%04x] state=%d, count=%d\n",
		       host->host_no, reselection_id, lun, dsp, dsp - hostdata->pScript, hostdata->state, hostdata->command_slot_count);

		{
			/* FIXME: DEBUGGING CODE */
			__u32 SG = (__u32)bS_to_cpu(hostdata->script[A_SGScriptStartAddress_used[0]]);
			int i;

			for(i=0; i< NCR_700_COMMAND_SLOTS_PER_HOST; i++) {
				if(SG >= to32bit(&hostdata->slots[i].pSG[0])
				   && SG <= to32bit(&hostdata->slots[i].pSG[NCR_700_SG_SEGMENTS]))
					break;
			}
			printk(KERN_INFO "IDENTIFIED SG segment as being %08x in slot %p, cmd %p, slot->resume_offset=%08x\n", SG, &hostdata->slots[i], hostdata->slots[i].cmnd, hostdata->slots[i].resume_offset);
			SCp =  hostdata->slots[i].cmnd;
		}

		if(SCp != NULL) {
			slot = (struct NCR_700_command_slot *)SCp->host_scribble;
			/* change slot from busy to queued to redo command */
			slot->state = NCR_700_SLOT_QUEUED;
		}
		hostdata->cmd = NULL;

		if(reselection_id == 0) {
			if(hostdata->reselection_id == 0xff) {
				printk(KERN_ERR "scsi%d: Invalid reselection during selection!!\n", host->host_no);
				return 0;
			} else {
				printk(KERN_ERR "scsi%d: script reselected and we took a selection interrupt\n",
				       host->host_no);
				reselection_id = hostdata->reselection_id;
			}
		} else {

			/* convert to real ID */
			reselection_id = bitmap_to_number(reselection_id);
		}
		hostdata->reselection_id = reselection_id;
		/* just in case we have a stale simple tag message, clear it */
		hostdata->msgin[1] = 0;
		dma_sync_to_dev(hostdata, hostdata->msgin, MSG_ARRAY_SIZE);
		if(hostdata->tag_negotiated & (1<<reselection_id)) {
			resume_offset = hostdata->pScript + Ent_GetReselectionWithTag;
		} else {
			resume_offset = hostdata->pScript + Ent_GetReselectionData;
		}
	} else if(dsps == A_COMPLETED_SELECTION_AS_TARGET) {
		/* we've just disconnected from the bus, do nothing since
		 * a return here will re-run the queued command slot
		 * that may have been interrupted by the initial selection */
		DEBUG((" SELECTION COMPLETED\n"));
	} else if((dsps & 0xfffff0f0) == A_MSG_IN) {
		printk("53c700: MESSAGE IN interrupt - dsps=0x%08x, calling process_message\n", dsps);
		resume_offset = process_message(host, hostdata, SCp,
						dsp, dsps);
	} else if((dsps &  0xfffff000) == 0) {
		__u8 i = (dsps & 0xf0) >> 4, j = (dsps & 0xf00) >> 8;
		printk(KERN_ERR "scsi%d: (%d:%d), unhandled script condition %s %s at %04x\n",
		       host->host_no, pun, lun, NCR_700_condition[i],
		       NCR_700_phase[j], dsp - hostdata->pScript);
		if(SCp != NULL) {
			struct scatterlist *sg;

			scsi_print_command(SCp);
			scsi_for_each_sg(SCp, sg, scsi_sg_count(SCp) + 1, i) {
				printk(KERN_INFO " SG[%d].length = %d, move_insn=%08x, addr %08x\n", i, sg->length, ((struct NCR_700_command_slot *)SCp->host_scribble)->SG[i].ins, ((struct NCR_700_command_slot *)SCp->host_scribble)->SG[i].pAddr);
			}
		}
		NCR_700_internal_bus_reset(host);
	} else if((dsps & 0xfffff000) == A_DEBUG_INTERRUPT) {
		printk(KERN_NOTICE "scsi%d (%d:%d) DEBUG INTERRUPT %d AT %08x[%04x], continuing\n",
		       host->host_no, pun, lun, dsps & 0xfff, dsp, dsp - hostdata->pScript);
		resume_offset = dsp;
	} else {
		printk("53c700: UNKNOWN SCRIPT INTERRUPT - dsps=0x%08x, dsp=0x%08x (offset=0x%04x)\n",
		       dsps, dsp, dsp - hostdata->pScript);
		printk(KERN_ERR "scsi%d: (%d:%d), unidentified script interrupt 0x%x at %04x\n",
		       host->host_no, pun, lun, dsps, dsp - hostdata->pScript);
		NCR_700_internal_bus_reset(host);
	}
	printk("53c700: process_script_interrupt returning resume_offset=0x%08x\n", resume_offset);
	return resume_offset;
}

/* We run the 53c700 with selection interrupts always enabled.  This
 * means that the chip may be selected as soon as the bus frees.  On a
 * busy bus, this can be before the scripts engine finishes its
 * processing.  Therefore, part of the selection processing has to be
 * to find out what the scripts engine is doing and complete the
 * function if necessary (i.e. process the pending disconnect or save
 * the interrupted initial selection */
STATIC inline __u32
process_selection(struct Scsi_Host *host, __u32 dsp)
{
	__u8 id = 0;	/* Squash compiler warning */
	int count = 0;
	__u32 resume_offset = 0;
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)host->hostdata[0];
	struct scsi_cmnd *SCp = hostdata->cmd;
	__u8 sbcl;

	for(count = 0; count < 5; count++) {
		id = NCR_700_readb(host, hostdata->chip710 ?
				   CTEST9_REG : SFBR_REG);

		/* Take out our own ID */
		id &= ~(1<<host->this_id);
		if(id != 0)
			break;
		udelay(5);
	}
	sbcl = NCR_700_readb(host, SBCL_REG);
	if((sbcl & SBCL_IO) == 0) {
		/* mark as having been selected rather than reselected */
		id = 0xff;
	} else {
		/* convert to real ID */
		hostdata->reselection_id = id = bitmap_to_number(id);
		DEBUG(("scsi%d:  Reselected by %d\n",
		       host->host_no, id));
	}
	if(hostdata->state == NCR_700_HOST_BUSY && SCp != NULL) {
		struct NCR_700_command_slot *slot =
			(struct NCR_700_command_slot *)SCp->host_scribble;
		DEBUG(("  ID %d WARNING: RESELECTION OF BUSY HOST, saving cmd %p, slot %p, addr %x [%04x], resume %x!\n", id, hostdata->cmd, slot, dsp, dsp - hostdata->pScript, resume_offset));

		switch(dsp - hostdata->pScript) {
		case Ent_Disconnect1:
		case Ent_Disconnect2:
			save_for_reselection(hostdata, SCp, Ent_Disconnect2 + hostdata->pScript);
			break;
		case Ent_Disconnect3:
		case Ent_Disconnect4:
			save_for_reselection(hostdata, SCp, Ent_Disconnect4 + hostdata->pScript);
			break;
		case Ent_Disconnect5:
		case Ent_Disconnect6:
			save_for_reselection(hostdata, SCp, Ent_Disconnect6 + hostdata->pScript);
			break;
		case Ent_Disconnect7:
		case Ent_Disconnect8:
			save_for_reselection(hostdata, SCp, Ent_Disconnect8 + hostdata->pScript);
			break;
		case Ent_Finish1:
		case Ent_Finish2:
			process_script_interrupt(A_GOOD_STATUS_AFTER_STATUS, dsp, SCp, host, hostdata);
			break;

		default:
			slot->state = NCR_700_SLOT_QUEUED;
			break;
			}
	}
	hostdata->state = NCR_700_HOST_BUSY;
	hostdata->cmd = NULL;
	/* clear any stale simple tag message */
	hostdata->msgin[1] = 0;
	dma_sync_to_dev(hostdata, hostdata->msgin, MSG_ARRAY_SIZE);

	if(id == 0xff) {
		/* Selected as target, Ignore */
		resume_offset = hostdata->pScript + Ent_SelectedAsTarget;
	} else if(hostdata->tag_negotiated & (1<<id)) {
		resume_offset = hostdata->pScript + Ent_GetReselectionWithTag;
	} else {
		resume_offset = hostdata->pScript + Ent_GetReselectionData;
	}
	return resume_offset;
}

static inline void
NCR_700_clear_fifo(struct Scsi_Host *host) {
	const struct NCR_700_Host_Parameters *hostdata
		= (struct NCR_700_Host_Parameters *)host->hostdata[0];
	if(hostdata->chip710) {
		NCR_700_writeb(CLR_FIFO_710, host, CTEST8_REG);
	} else {
		NCR_700_writeb(CLR_FIFO, host, DFIFO_REG);
	}
}

static inline void
NCR_700_flush_fifo(struct Scsi_Host *host) {
	const struct NCR_700_Host_Parameters *hostdata
		= (struct NCR_700_Host_Parameters *)host->hostdata[0];
	if(hostdata->chip710) {
		NCR_700_writeb(FLUSH_DMA_FIFO_710, host, CTEST8_REG);
		udelay(10);
		NCR_700_writeb(0, host, CTEST8_REG);
	} else {
		NCR_700_writeb(FLUSH_DMA_FIFO, host, DFIFO_REG);
		udelay(10);
		NCR_700_writeb(0, host, DFIFO_REG);
	}
}


/* The queue lock with interrupts disabled must be held on entry to
 * this function */
STATIC int
NCR_700_start_command(struct scsi_cmnd *SCp)
{
	struct NCR_700_command_slot *slot =
		(struct NCR_700_command_slot *)SCp->host_scribble;
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SCp->device->host->hostdata[0];
	__u16 count = 1;	/* for IDENTIFY message */
	u8 lun = SCp->device->lun;

	printk("53c700: Starting command for target %d lun %d, CDB[0]=0x%02x\n",
	       SCp->device->id, lun, SCp->cmnd[0]);

	if(hostdata->state != NCR_700_HOST_FREE) {
		/* keep this inside the lock to close the race window where
		 * the running command finishes on another CPU while we don't
		 * change the state to queued on this one */
		slot->state = NCR_700_SLOT_QUEUED;

		printk("53c700: Host busy, queueing command\n");
		DEBUG(("scsi%d: host busy, queueing command %p, slot %p\n",
		       SCp->device->host->host_no, slot->cmnd, slot));
		return 0;
	}
	printk("53c700: Host free, starting command immediately\n");
	hostdata->state = NCR_700_HOST_BUSY;
	hostdata->cmd = SCp;
	slot->state = NCR_700_SLOT_BUSY;
	/* keep interrupts disabled until we have the command correctly
	 * set up so we cannot take a selection interrupt */

	hostdata->msgout[0] = NCR_700_identify((SCp->cmnd[0] != REQUEST_SENSE &&
						slot->flags != NCR_700_FLAG_AUTOSENSE),
					       lun);
	/* for INQUIRY or REQUEST_SENSE commands, we cannot be sure
	 * if the negotiated transfer parameters still hold, so
	 * always renegotiate them */
	if(SCp->cmnd[0] == INQUIRY || SCp->cmnd[0] == REQUEST_SENSE ||
	   slot->flags == NCR_700_FLAG_AUTOSENSE) {
		NCR_700_clear_flag(SCp->device, NCR_700_DEV_NEGOTIATED_SYNC);
	}

	/* REQUEST_SENSE is asking for contingent I_T_L(_Q) status.
	 * If a contingent allegiance condition exists, the device
	 * will refuse all tags, so send the request sense as untagged
	 * */
	if((hostdata->tag_negotiated & (1<<scmd_id(SCp)))
	   && (slot->tag != SCSI_NO_TAG && SCp->cmnd[0] != REQUEST_SENSE &&
	       slot->flags != NCR_700_FLAG_AUTOSENSE)) {
		count += spi_populate_tag_msg(&hostdata->msgout[count], SCp);
	}

	if(hostdata->fast &&
	   NCR_700_is_flag_clear(SCp->device, NCR_700_DEV_NEGOTIATED_SYNC)) {
		count += spi_populate_sync_msg(&hostdata->msgout[count],
				spi_period(SCp->device->sdev_target),
				spi_offset(SCp->device->sdev_target));
		NCR_700_set_flag(SCp->device, NCR_700_DEV_BEGIN_SYNC_NEGOTIATION);
	}

	script_patch_16(hostdata, hostdata->script, MessageCount, count);

	printk("53c700: Script patching details:\n");
	printk("53c700:   MessageCount=%d, Device_ID=0x%02x (target %d)\n",
	       count, 1<<scmd_id(SCp), scmd_id(SCp));
	printk("53c700:   Command: pCmd=0x%08x, cmd_len=%d, CDB[0]=0x%02x\n",
	       slot->pCmd, SCp->cmd_len, SCp->cmnd[0]);
	printk("53c700:   SG: pSG=0x%08x, sg_count=%d\n",
	       to32bit(&slot->pSG[0].ins), SCp->sc_data_direction != DMA_NONE ? scsi_sg_count(SCp) : 0);

	script_patch_ID(hostdata, hostdata->script, Device_ID, 1<<scmd_id(SCp));

	script_patch_32_abs(hostdata, hostdata->script, CommandAddress,
			    slot->pCmd);
	script_patch_16(hostdata, hostdata->script, CommandCount, SCp->cmd_len);
	/* finally plumb the beginning of the SG list into the script
	 * */
	script_patch_32_abs(hostdata, hostdata->script,
	                    SGScriptStartAddress, to32bit(&slot->pSG[0].ins));
	NCR_700_clear_fifo(SCp->device->host);

	if(slot->resume_offset == 0)
		slot->resume_offset = hostdata->pScript;
	/* now perform all the writebacks and invalidates */
	dma_sync_to_dev(hostdata, hostdata->msgout, count);
	dma_sync_from_dev(hostdata, hostdata->msgin, MSG_ARRAY_SIZE);
	dma_sync_to_dev(hostdata, SCp->cmnd, SCp->cmd_len);
	dma_sync_from_dev(hostdata, hostdata->status, 1);

	/* set the synchronous period/offset */
	NCR_700_writeb(NCR_700_get_SXFER(SCp->device),
		       SCp->device->host, SXFER_REG);
	NCR_700_writel(slot->temp, SCp->device->host, TEMP_REG);
	NCR_700_writel(slot->resume_offset, SCp->device->host, DSP_REG);
	printk("53c700: Starting script execution! DSP=0x%08x (offset=0x%04x), temp=0x%08x\n",
	       slot->resume_offset, slot->resume_offset - hostdata->pScript, slot->temp);
	printk("53c700: SXFER=0x%02x, msgout[0]=0x%02x, cmd_len=%d\n",
	       NCR_700_get_SXFER(SCp->device), hostdata->msgout[0], SCp->cmd_len);

	/* DEBUG: Check chip state immediately after starting script */
	{
		__u8 istat_check = NCR_700_readb(SCp->device->host, ISTAT_REG);
		__u8 dstat_check = NCR_700_readb(SCp->device->host, DSTAT_REG);
		__u8 sstat0_check = NCR_700_readb(SCp->device->host, SSTAT0_REG);
		__u8 sstat1_check = NCR_700_readb(SCp->device->host, SSTAT1_REG);
		__u32 dsp_check = NCR_700_readl(SCp->device->host, DSP_REG);
		__u32 dsps_check = NCR_700_readl(SCp->device->host, DSPS_REG);
		__u8 scntl1_check = NCR_700_readb(SCp->device->host, SCNTL1_REG);

		printk("53c700: POST-START CHIP STATE: istat=0x%02x dstat=0x%02x sstat0=0x%02x sstat1=0x%02x\n",
		       istat_check, dstat_check, sstat0_check, sstat1_check);
		printk("53c700: POST-START REGS: dsp=0x%08x dsps=0x%08x scntl1=0x%02x\n",
		       dsp_check, dsps_check, scntl1_check);
		printk("53c700: DSP CHANGED: %s (expected: 0x%08x, actual: 0x%08x)\n",
		       (dsp_check != slot->resume_offset) ? "YES - SCRIPT EXECUTING" : "NO - SCRIPT NOT STARTED",
		       slot->resume_offset, dsp_check);

		/* Wait a tiny bit and check again to see if script makes progress */
		// udelay(10);
		// {
		// 	__u32 dsp_check2 = NCR_700_readl(SCp->device->host, DSP_REG);
		// 	__u8 istat_check2 = NCR_700_readb(SCp->device->host, ISTAT_REG);
		// 	printk("53c700: AFTER 10us: dsp=0x%08x istat=0x%02x %s\n",
		// 	       dsp_check2, istat_check2,
		// 	       (dsp_check2 != dsp_check) ? "SCRIPT PROGRESSING" : "SCRIPT STUCK");
		// }
	}

	return 1;
}

irqreturn_t
NCR_700_intr(int irq, void *dev_id)
{
	struct Scsi_Host *host = (struct Scsi_Host *)dev_id;
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)host->hostdata[0];
	__u8 istat;
	__u32 resume_offset = 0;
	__u8 pun = 0xff, lun = 0xff;
	unsigned long flags;
	int handled = 0;

	/* Use the host lock to serialise access to the 53c700
	 * hardware.  Note: In future, we may need to take the queue
	 * lock to enter the done routines.  When that happens, we
	 * need to ensure that for this driver, the host lock and the
	 * queue lock point to the same thing. */
	spin_lock_irqsave(host->host_lock, flags);
	if((istat = NCR_700_readb(host, ISTAT_REG))
	      & (SCSI_INT_PENDING | DMA_INT_PENDING)) {
		__u32 dsps;
		__u8 sstat0 = 0, dstat = 0;
		__u32 dsp;
		struct scsi_cmnd *SCp = hostdata->cmd;

		printk("53c700: Interrupt triggered! istat=0x%02x\n", istat);
		handled = 1;

		if(istat & SCSI_INT_PENDING) {
			udelay(10);

			sstat0 = NCR_700_readb(host, SSTAT0_REG);
			printk("53c700: SCSI interrupt pending, sstat0=0x%02x\n", sstat0);
		}

		if(istat & DMA_INT_PENDING) {
			udelay(10);

			dstat = NCR_700_readb(host, DSTAT_REG);
			printk("53c700: DMA interrupt pending, dstat=0x%02x\n", dstat);
		}

		dsps = NCR_700_readl(host, DSPS_REG);
		dsp = NCR_700_readl(host, DSP_REG);

		printk("53c700: Script execution - DSP=0x%08x (offset=0x%04x), DSPS=0x%08x\n",
		       dsp, (dsp - (__u32)(hostdata->pScript))/4, dsps);

		DEBUG(("scsi%d: istat %02x sstat0 %02x dstat %02x dsp %04x[%08x] dsps 0x%x\n",
		       host->host_no, istat, sstat0, dstat,
		       (dsp - (__u32)(hostdata->pScript))/4,
		       dsp, dsps));

		if(SCp != NULL) {
			pun = SCp->device->id;
			lun = SCp->device->lun;
		}

		if(sstat0 & SCSI_RESET_DETECTED) {
			struct scsi_device *SDp;
			int i;

			hostdata->state = NCR_700_HOST_BUSY;

			printk(KERN_ERR "scsi%d: Bus Reset detected, executing command %p, slot %p, dsp %08x[%04x]\n",
			       host->host_no, SCp, SCp == NULL ? NULL : SCp->host_scribble, dsp, dsp - hostdata->pScript);

			scsi_report_bus_reset(host, 0);

			/* clear all the negotiated parameters */
			__shost_for_each_device(SDp, host)
				NCR_700_clear_flag(SDp, ~0);

			/* clear all the slots and their pending commands */
			for(i = 0; i < NCR_700_COMMAND_SLOTS_PER_HOST; i++) {
				struct scsi_cmnd *SCp;
				struct NCR_700_command_slot *slot =
					&hostdata->slots[i];

				if(slot->state == NCR_700_SLOT_FREE)
					continue;

				SCp = slot->cmnd;
				printk(KERN_ERR " failing command because of reset, slot %p, cmnd %p\n",
				       slot, SCp);
				free_slot(slot, hostdata);
				SCp->host_scribble = NULL;
				NCR_700_set_depth(SCp->device, 0);
				/* NOTE: deadlock potential here: we
				 * rely on mid-layer guarantees that
				 * scsi_done won't try to issue the
				 * command again otherwise we'll
				 * deadlock on the
				 * hostdata->state_lock */
				SCp->result = DID_RESET << 16;
				scsi_done(SCp);
			}
			mdelay(25);
			NCR_700_chip_setup(host);

			hostdata->state = NCR_700_HOST_FREE;
			hostdata->cmd = NULL;
			/* signal back if this was an eh induced reset */
			if(hostdata->eh_complete != NULL)
				complete(hostdata->eh_complete);
			goto out_unlock;
		} else if(sstat0 & SELECTION_TIMEOUT) {
			printk("53c700: SELECTION TIMEOUT - target %d did not respond\n", pun);
			DEBUG(("scsi%d: (%d:%d) selection timeout\n",
			       host->host_no, pun, lun));
			NCR_700_scsi_done(hostdata, SCp, DID_NO_CONNECT<<16);
		} else if(sstat0 & PHASE_MISMATCH) {
			__u8 sbcl = NCR_700_readb(host, SBCL_REG);
			__u32 dbc = NCR_700_readl(host, DBC_REG);
			__u32 dnad = NCR_700_readl(host, DNAD_REG);

			printk(KERN_DEBUG "53c700: PHASE MISMATCH detected - dsp=0x%08x, expected phase, actual phase=%s\n",
			       dsp, sbcl_to_string(sbcl));
			printk(KERN_DEBUG "53c700: Phase mismatch details - DBC=0x%08x, DNAD=0x%08x, SBCL=0x%02x\n",
			       dbc, dnad, sbcl);

			struct NCR_700_command_slot *slot = (SCp == NULL) ? NULL :
				(struct NCR_700_command_slot *)SCp->host_scribble;

			if(dsp == Ent_SendMessage + 8 + hostdata->pScript) {
				/* It wants to reply to some part of
				 * our message */
#ifdef NCR_700_DEBUG
				__u32 temp = NCR_700_readl(host, TEMP_REG);
				int count = (hostdata->script[Ent_SendMessage/4] & 0xffffff) - ((NCR_700_readl(host, DBC_REG) & 0xffffff) + NCR_700_data_residual(host));
				printk("scsi%d (%d:%d) PHASE MISMATCH IN SEND MESSAGE %d remain, return %p[%04x], phase %s\n", host->host_no, pun, lun, count, (void *)temp, temp - hostdata->pScript, sbcl_to_string(NCR_700_readb(host, SBCL_REG)));
#endif
				resume_offset = hostdata->pScript + Ent_SendMessagePhaseMismatch;
			} else if (slot && dsp >= to32bit(&slot->pSG[0].ins) &&
				  dsp <= to32bit(&slot->pSG[NCR_700_SG_SEGMENTS].ins)) {
				int data_transfer = NCR_700_readl(host, DBC_REG) & 0xffffff;
				int SGcount = (dsp - to32bit(&slot->pSG[0].ins))/sizeof(struct NCR_700_SG_List);
				int residual = NCR_700_data_residual(host);
				int i;
#ifdef NCR_700_DEBUG
				__u32 naddr = NCR_700_readl(host, DNAD_REG);

				printk("scsi%d: (%d:%d) Expected phase mismatch in slot->SG[%d], transferred 0x%x\n",
				       host->host_no, pun, lun,
				       SGcount, data_transfer);
				scsi_print_command(SCp);
				if(residual) {
					printk("scsi%d: (%d:%d) Expected phase mismatch in slot->SG[%d], transferred 0x%x, residual %d\n",
				       host->host_no, pun, lun,
				       SGcount, data_transfer, residual);
				}
#endif
				data_transfer += residual;

				if(data_transfer != 0) {
					int count;
					__u32 pAddr;

					SGcount--;

					count = (bS_to_cpu(slot->SG[SGcount].ins) & 0x00ffffff);
					DEBUG(("DATA TRANSFER MISMATCH, count = %d, transferred %d\n", count, count-data_transfer));
					slot->SG[SGcount].ins &= bS_to_host(0xff000000);
					slot->SG[SGcount].ins |= bS_to_host(data_transfer);
					pAddr = bS_to_cpu(slot->SG[SGcount].pAddr);
					pAddr += (count - data_transfer);
#ifdef NCR_700_DEBUG
					if(pAddr != naddr) {
						printk("scsi%d (%d:%d) transfer mismatch pAddr=%lx, naddr=%lx, data_transfer=%d, residual=%d\n", host->host_no, pun, lun, (unsigned long)pAddr, (unsigned long)naddr, data_transfer, residual);
					}
#endif
					slot->SG[SGcount].pAddr = bS_to_host(pAddr);
				}
				/* set the executed moves to nops */
				for(i=0; i<SGcount; i++) {
					slot->SG[i].ins = bS_to_host(SCRIPT_NOP);
					slot->SG[i].pAddr = 0;
				}
				dma_sync_to_dev(hostdata, slot->SG, sizeof(slot->SG));
				/* and pretend we disconnected after
				 * the command phase */
				resume_offset = hostdata->pScript + Ent_MsgInDuringData;
				/* make sure all the data is flushed */
				NCR_700_flush_fifo(host);
			} else {
				__u8 sbcl = NCR_700_readb(host, SBCL_REG);
				printk(KERN_ERR "scsi%d: (%d:%d) phase mismatch at %04x, phase %s\n",
				       host->host_no, pun, lun, dsp - hostdata->pScript, sbcl_to_string(sbcl));
				NCR_700_internal_bus_reset(host);
			}

		} else if(sstat0 & SCSI_GROSS_ERROR) {
			printk("53c700: SCSI GROSS ERROR detected\n");
			printk(KERN_ERR "scsi%d: (%d:%d) GROSS ERROR\n",
			       host->host_no, pun, lun);
			NCR_700_scsi_done(hostdata, SCp, DID_ERROR<<16);
		} else if(sstat0 & PARITY_ERROR) {
			printk("53c700: PARITY ERROR detected\n");
			printk(KERN_ERR "scsi%d: (%d:%d) PARITY ERROR\n",
			       host->host_no, pun, lun);
			NCR_700_scsi_done(hostdata, SCp, DID_ERROR<<16);
		} else if(dstat & SCRIPT_INT_RECEIVED) {
			printk("53c700: SCRIPT INTERRUPT received, calling process_script_interrupt\n");
			printk("53c700: Script interrupt details: dsps=0x%08x, dsp=0x%08x, SCp=%p\n",
			       dsps, dsp, SCp);
			DEBUG(("scsi%d: (%d:%d) ====>SCRIPT INTERRUPT<====\n",
			       host->host_no, pun, lun));
			resume_offset = process_script_interrupt(dsps, dsp, SCp, host, hostdata);
			printk("53c700: process_script_interrupt returned resume_offset=0x%08x\n",
			       resume_offset);
		} else if(dstat & (ILGL_INST_DETECTED)) {
			printk("53c700: ILLEGAL INSTRUCTION detected at DSP=0x%08x\n", dsp);
			printk(KERN_ERR "scsi%d: (%d:%d) Illegal Instruction detected at 0x%08x[0x%x]!!!\n"
			       "         Please email James.Bottomley@HansenPartnership.com with the details\n",
			       host->host_no, pun, lun,
			       dsp, dsp - hostdata->pScript);
			NCR_700_scsi_done(hostdata, SCp, DID_ERROR<<16);
		} else if(dstat & (WATCH_DOG_INTERRUPT|ABORTED)) {
			printk("53c700: DMA WATCHDOG/ABORTED interrupt, dstat=0x%02x\n", dstat);
			printk(KERN_ERR "scsi%d: (%d:%d) serious DMA problem, dstat=%02x\n",
			       host->host_no, pun, lun, dstat);
			NCR_700_scsi_done(hostdata, SCp, DID_ERROR<<16);
		}


		/* NOTE: selection interrupt processing MUST occur
		 * after script interrupt processing to correctly cope
		 * with the case where we process a disconnect and
		 * then get reselected before we process the
		 * disconnection */
		if(sstat0 & SELECTED) {
			printk("53c700: SELECTED interrupt - processing selection\n");
			/* FIXME: It currently takes at least FOUR
			 * interrupts to complete a command that
			 * disconnects: one for the disconnect, one
			 * for the reselection, one to get the
			 * reselection data and one to complete the
			 * command.  If we guess the reselected
			 * command here and prepare it, we only need
			 * to get a reselection data interrupt if we
			 * guessed wrongly.  Since the interrupt
			 * overhead is much greater than the command
			 * setup, this would be an efficient
			 * optimisation particularly as we probably
			 * only have one outstanding command on a
			 * target most of the time */

			resume_offset = process_selection(host, dsp);

		}

	} else {
		/* No interrupt pending - check if we should be getting one */
		if(hostdata->state == NCR_700_HOST_BUSY && hostdata->cmd) {
			__u32 dsp_current = NCR_700_readl(host, DSP_REG);
			__u8 istat_current = NCR_700_readb(host, ISTAT_REG);
			static unsigned long last_debug_time = 0;

			/* Rate limit this debug output to avoid spam */
			if (time_after(jiffies, last_debug_time + HZ)) {
				printk("53c700: NO INTERRUPT - host busy but no interrupt pending\n");
				printk("53c700: Current DSP=0x%08x, ISTAT=0x%02x, cmd=%p\n",
				       dsp_current, istat_current, hostdata->cmd);
				last_debug_time = jiffies;
			}
		}
	}

	if(resume_offset) {
		if(hostdata->state != NCR_700_HOST_BUSY) {
			printk(KERN_ERR "scsi%d: Driver error: resume at 0x%08x [0x%04x] with non busy host!\n",
			       host->host_no, resume_offset, resume_offset - hostdata->pScript);
			hostdata->state = NCR_700_HOST_BUSY;
		}

		DEBUG(("Attempting to resume at %x\n", resume_offset));
		printk("53c700: Resuming script execution at 0x%08x (offset 0x%04x)\n",
		       resume_offset, resume_offset - hostdata->pScript);
		NCR_700_clear_fifo(host);
		NCR_700_writel(resume_offset, host, DSP_REG);
		printk("53c700: Script resumed - DSP register set to 0x%08x\n", resume_offset);
	} else {
		printk("53c700: No resume_offset - interrupt processing complete\n");
		printk("53c700: Host state after interrupt: %s\n",
		       hostdata->state == NCR_700_HOST_FREE ? "FREE" :
		       hostdata->state == NCR_700_HOST_BUSY ? "BUSY" : "UNKNOWN");
	}
	/* There is probably a technical no-no about this: If we're a
	 * shared interrupt and we got this interrupt because the
	 * other device needs servicing not us, we're still going to
	 * check our queued commands here---of course, there shouldn't
	 * be any outstanding.... */
	if(hostdata->state == NCR_700_HOST_FREE) {
		int i;

		for(i = 0; i < NCR_700_COMMAND_SLOTS_PER_HOST; i++) {
			/* fairness: always run the queue from the last
			 * position we left off */
			int j = (i + hostdata->saved_slot_position)
				% NCR_700_COMMAND_SLOTS_PER_HOST;

			if(hostdata->slots[j].state != NCR_700_SLOT_QUEUED)
				continue;
			if(NCR_700_start_command(hostdata->slots[j].cmnd)) {
				DEBUG(("scsi%d: Issuing saved command slot %p, cmd %p\t\n",
				       host->host_no, &hostdata->slots[j],
				       hostdata->slots[j].cmnd));
				hostdata->saved_slot_position = j + 1;
			}

			break;
		}
	}
 out_unlock:
	spin_unlock_irqrestore(host->host_lock, flags);
	return IRQ_RETVAL(handled);
}

static int NCR_700_queuecommand_lck(struct scsi_cmnd *SCp)
{
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SCp->device->host->hostdata[0];
	__u32 move_ins;
	struct NCR_700_command_slot *slot;

	printk("53c700: [%lu] Queueing command for target %d lun %llu, CDB[0]=0x%02x\n",
	       jiffies, SCp->device->id, SCp->device->lun, SCp->cmnd[0]);

	if(hostdata->command_slot_count >= NCR_700_COMMAND_SLOTS_PER_HOST) {
		/* We're over our allocation, this should never happen
		 * since we report the max allocation to the mid layer */
		printk(KERN_WARNING "scsi%d: Command depth has gone over queue depth\n", SCp->device->host->host_no);
		return 1;
	}
	/* check for untagged commands.  We cannot have any outstanding
	 * commands if we accept them.  Commands could be untagged because:
	 *
	 * - The tag negotiated bitmap is clear
	 * - The blk layer sent and untagged command
	 */
	if(NCR_700_get_depth(SCp->device) != 0
	   && (!(hostdata->tag_negotiated & (1<<scmd_id(SCp)))
	       || !(SCp->flags & SCMD_TAGGED))) {
		CDEBUG(KERN_ERR, SCp, "has non zero depth %d\n",
		       NCR_700_get_depth(SCp->device));
		return SCSI_MLQUEUE_DEVICE_BUSY;
	}
	if(NCR_700_get_depth(SCp->device) >= SCp->device->queue_depth) {
		CDEBUG(KERN_ERR, SCp, "has max tag depth %d\n",
		       NCR_700_get_depth(SCp->device));
		return SCSI_MLQUEUE_DEVICE_BUSY;
	}
	NCR_700_set_depth(SCp->device, NCR_700_get_depth(SCp->device) + 1);

	/* begin the command here */
	/* no need to check for NULL, test for command_slot_count above
	 * ensures a slot is free */
	slot = find_empty_slot(hostdata);

	slot->cmnd = SCp;

	SCp->host_scribble = (unsigned char *)slot;
	printk("53c700: Command queued in slot %p, SCp->host_scribble=%p\n", slot, SCp->host_scribble);

#ifdef NCR_700_DEBUG
	printk("53c700: scsi%d, command ", SCp->device->host->host_no);
	scsi_print_command(SCp);
#endif
	if ((SCp->flags & SCMD_TAGGED)
	   && (hostdata->tag_negotiated &(1<<scmd_id(SCp))) == 0
	   && NCR_700_get_tag_neg_state(SCp->device) == NCR_700_START_TAG_NEGOTIATION) {
		scmd_printk(KERN_ERR, SCp, "Enabling Tag Command Queuing\n");
		hostdata->tag_negotiated |= (1<<scmd_id(SCp));
		NCR_700_set_tag_neg_state(SCp->device, NCR_700_DURING_TAG_NEGOTIATION);
	}

	/* here we may have to process an untagged command.  The gate
	 * above ensures that this will be the only one outstanding,
	 * so clear the tag negotiated bit.
	 *
	 * FIXME: This will royally screw up on multiple LUN devices
	 * */
	if (!(SCp->flags & SCMD_TAGGED)
	   && (hostdata->tag_negotiated &(1<<scmd_id(SCp)))) {
		scmd_printk(KERN_INFO, SCp, "Disabling Tag Command Queuing\n");
		hostdata->tag_negotiated &= ~(1<<scmd_id(SCp));
	}

	if ((hostdata->tag_negotiated & (1<<scmd_id(SCp))) &&
	    SCp->device->simple_tags) {
		slot->tag = scsi_cmd_to_rq(SCp)->tag;
		CDEBUG(KERN_DEBUG, SCp, "sending out tag %d, slot %p\n",
		       slot->tag, slot);
	} else {
		struct NCR_700_Device_Parameters *p = SCp->device->hostdata;

		slot->tag = SCSI_NO_TAG;
		/* save current command for reselection */
		p->current_cmnd = SCp;
	}
	/* sanity check: some of the commands generated by the mid-layer
	 * have an eccentric idea of their sc_data_direction */
	if(!scsi_sg_count(SCp) && !scsi_bufflen(SCp) &&
	   SCp->sc_data_direction != DMA_NONE) {
#ifdef NCR_700_DEBUG
		printk("53c700: Command");
		scsi_print_command(SCp);
		printk("Has wrong data direction %d\n", SCp->sc_data_direction);
#endif
		SCp->sc_data_direction = DMA_NONE;
	}

	switch (SCp->cmnd[0]) {
	case REQUEST_SENSE:
		/* clear the internal sense magic */
		SCp->cmnd[6] = 0;
		fallthrough;
	default:
		/* OK, get it from the command */
		switch(SCp->sc_data_direction) {
		case DMA_BIDIRECTIONAL:
		default:
			printk(KERN_ERR "53c700: Unknown command for data direction ");
			scsi_print_command(SCp);

			move_ins = 0;
			break;
		case DMA_NONE:
			move_ins = 0;
			break;
		case DMA_FROM_DEVICE:
			move_ins = SCRIPT_MOVE_DATA_IN;
			break;
		case DMA_TO_DEVICE:
			move_ins = SCRIPT_MOVE_DATA_OUT;
			break;
		}
	}

	/* now build the scatter gather list */
	if(move_ins != 0) {
		int i;
		int sg_count;
		dma_addr_t vPtr = 0;
		struct scatterlist *sg;
		__u32 count = 0;

		sg_count = scsi_dma_map(SCp);
		BUG_ON(sg_count < 0);

		scsi_for_each_sg(SCp, sg, sg_count, i) {
			vPtr = sg_dma_address(sg);
			count = sg_dma_len(sg);

			slot->SG[i].ins = bS_to_host(move_ins | count);
			DEBUG((" scatter block %d: move %d[%08x] from 0x%lx\n",
			       i, count, slot->SG[i].ins, (unsigned long)vPtr));
			slot->SG[i].pAddr = bS_to_host(vPtr);
		}
		slot->SG[i].ins = bS_to_host(SCRIPT_RETURN);
		slot->SG[i].pAddr = 0;
		dma_sync_to_dev(hostdata, slot->SG, sizeof(slot->SG));
		DEBUG((" SETTING %p to %x\n",
		       (&slot->pSG[i].ins),
		       slot->SG[i].ins));
	}
	slot->resume_offset = 0;
	slot->pCmd = dma_map_single(hostdata->dev, SCp->cmnd,
				    MAX_COMMAND_SIZE, DMA_TO_DEVICE);
	NCR_700_start_command(SCp);
	return 0;
}

STATIC DEF_SCSI_QCMD(NCR_700_queuecommand)

STATIC int
NCR_700_abort(struct scsi_cmnd * SCp)
{
	struct NCR_700_command_slot *slot;
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SCp->device->host->hostdata[0];

	scmd_printk(KERN_INFO, SCp, "abort command\n");

	/* DEBUG: Dump detailed state when abort is called */
	printk("53c700: [%lu] ABORT called for target %d lun %llu, CDB[0]=0x%02x\n",
	       jiffies, SCp->device->id, SCp->device->lun, SCp->cmnd[0]);
	printk("53c700: ABORT timing - SCp=%p, SCp->host_scribble=%p\n",
	       SCp, SCp->host_scribble);

	{
		__u8 istat = NCR_700_readb(SCp->device->host, ISTAT_REG);
		__u8 dstat = NCR_700_readb(SCp->device->host, DSTAT_REG);
		__u8 sstat0 = NCR_700_readb(SCp->device->host, SSTAT0_REG);
		__u8 sstat1 = NCR_700_readb(SCp->device->host, SSTAT1_REG);
		__u32 dsp = NCR_700_readl(SCp->device->host, DSP_REG);
		__u32 dsps = NCR_700_readl(SCp->device->host, DSPS_REG);
		__u32 dbc = NCR_700_readl(SCp->device->host, DBC_REG) & 0x00ffffff;
		__u8 sbcl = NCR_700_readb(SCp->device->host, SBCL_REG);

		printk("53c700: ABORT CHIP STATE: istat=0x%02x dstat=0x%02x sstat0=0x%02x sstat1=0x%02x\n",
		       istat, dstat, sstat0, sstat1);
		printk("53c700: ABORT REGS: dsp=0x%08x (offset=0x%04x) dsps=0x%08x dbc=0x%06x\n",
		       dsp, dsp - hostdata->pScript, dsps, dbc);
		printk("53c700: ABORT SCSI: sbcl=0x%02x (bus phase)\n", sbcl);
		printk("53c700: HOST STATE: %s, cmd=%p\n",
		       hostdata->state == NCR_700_HOST_FREE ? "FREE" :
		       hostdata->state == NCR_700_HOST_BUSY ? "BUSY" : "UNKNOWN",
		       hostdata->cmd);
		printk("53c700: ABORT - hostdata->cmd comparison: hostdata->cmd=%p, SCp=%p, %s\n",
		       hostdata->cmd, SCp, hostdata->cmd == SCp ? "MATCH" : "MISMATCH");
	}

	slot = (struct NCR_700_command_slot *)SCp->host_scribble;

	if(slot == NULL) {
		/* no outstanding command to abort */
		printk("53c700: ABORT - no slot found for command (SCp->host_scribble is NULL)\n");
		printk("53c700: ABORT - this suggests command already completed and cleaned up\n");
		return SUCCESS;
	}

	printk("53c700: ABORT - found slot %p, state=%d\n", slot, slot->state);
	printk("53c700: ABORT - slot->cmnd=%p, slot->flags=0x%x\n", slot->cmnd, slot->flags);
	printk("53c700: ABORT - CRITICAL: slot->cmnd == SCp? %s\n",
	       slot->cmnd == SCp ? "YES" : "NO");

	if(SCp->cmnd[0] == TEST_UNIT_READY) {
		/* FIXME: This is because of a problem in the new
		 * error handler.  When it is in error recovery, it
		 * will send a TUR to a device it thinks may still be
		 * showing a problem.  If the TUR isn't responded to,
		 * it will abort it and mark the device off line.
		 * Unfortunately, it does no other error recovery, so
		 * this would leave us with an outstanding command
		 * occupying a slot.  Rather than allow this to
		 * happen, we issue a bus reset to force all
		 * outstanding commands to terminate here. */
		NCR_700_internal_bus_reset(SCp->device->host);
		/* still drop through and return failed */
	}
	return FAILED;

}

STATIC int
NCR_700_host_reset(struct scsi_cmnd * SCp)
{
	DECLARE_COMPLETION_ONSTACK(complete);
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SCp->device->host->hostdata[0];

	scmd_printk(KERN_INFO, SCp,
		"New error handler wants HOST reset, cmd %p\n\t", SCp);
	scsi_print_command(SCp);

	/* In theory, eh_complete should always be null because the
	 * eh is single threaded, but just in case we're handling a
	 * reset via sg or something */
	spin_lock_irq(SCp->device->host->host_lock);
	while (hostdata->eh_complete != NULL) {
		spin_unlock_irq(SCp->device->host->host_lock);
		msleep_interruptible(100);
		spin_lock_irq(SCp->device->host->host_lock);
	}

	hostdata->eh_complete = &complete;
	NCR_700_internal_bus_reset(SCp->device->host);
	NCR_700_chip_reset(SCp->device->host);

	spin_unlock_irq(SCp->device->host->host_lock);
	wait_for_completion(&complete);
	spin_lock_irq(SCp->device->host->host_lock);

	hostdata->eh_complete = NULL;
	/* Revalidate the transport parameters of the failing device */
	if(hostdata->fast)
		spi_schedule_dv_device(SCp->device);

	spin_unlock_irq(SCp->device->host->host_lock);
	return SUCCESS;
}

STATIC void
NCR_700_set_period(struct scsi_target *STp, int period)
{
	struct Scsi_Host *SHp = dev_to_shost(STp->dev.parent);
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SHp->hostdata[0];

	if(!hostdata->fast)
		return;

	if(period < hostdata->min_period)
		period = hostdata->min_period;

	spi_period(STp) = period;
	spi_flags(STp) &= ~(NCR_700_DEV_NEGOTIATED_SYNC |
			    NCR_700_DEV_BEGIN_SYNC_NEGOTIATION);
	spi_flags(STp) |= NCR_700_DEV_PRINT_SYNC_NEGOTIATION;
}

STATIC void
NCR_700_set_offset(struct scsi_target *STp, int offset)
{
	struct Scsi_Host *SHp = dev_to_shost(STp->dev.parent);
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SHp->hostdata[0];
	int max_offset = hostdata->chip710
		? NCR_710_MAX_OFFSET : NCR_700_MAX_OFFSET;

	if(!hostdata->fast)
		return;

	if(offset > max_offset)
		offset = max_offset;

	/* if we're currently async, make sure the period is reasonable */
	if(spi_offset(STp) == 0 && (spi_period(STp) < hostdata->min_period ||
				    spi_period(STp) > 0xff))
		spi_period(STp) = hostdata->min_period;

	spi_offset(STp) = offset;
	spi_flags(STp) &= ~(NCR_700_DEV_NEGOTIATED_SYNC |
			    NCR_700_DEV_BEGIN_SYNC_NEGOTIATION);
	spi_flags(STp) |= NCR_700_DEV_PRINT_SYNC_NEGOTIATION;
}

STATIC int
NCR_700_sdev_init(struct scsi_device *SDp)
{
	SDp->hostdata = kzalloc(sizeof(struct NCR_700_Device_Parameters),
				GFP_KERNEL);

	if (!SDp->hostdata)
		return -ENOMEM;

	return 0;
}

STATIC int
NCR_700_sdev_configure(struct scsi_device *SDp, struct queue_limits *lim)
{
	struct NCR_700_Host_Parameters *hostdata =
		(struct NCR_700_Host_Parameters *)SDp->host->hostdata[0];

	printk(KERN_DEBUG "53c700: Configuring device %d:%llu - tagged_supported=%d, fast=%d\n",
	       SDp->id, SDp->lun, SDp->tagged_supported, hostdata->fast);

	/* to do here: allocate memory; build a queue_full list */
	if(SDp->tagged_supported) {
		scsi_change_queue_depth(SDp, NCR_700_DEFAULT_TAGS);
		NCR_700_set_tag_neg_state(SDp, NCR_700_START_TAG_NEGOTIATION);
		printk(KERN_DEBUG "53c700: Device %d:%llu enabled for tagged queuing, queue depth=%d\n",
		       SDp->id, SDp->lun, NCR_700_DEFAULT_TAGS);
	}

	if(hostdata->fast) {
		printk(KERN_DEBUG "53c700: Device %d:%llu starting domain validation (fast scsi enabled)\n",
		       SDp->id, SDp->lun);
		/* Find the correct offset and period via domain validation */
		if (!spi_initial_dv(SDp->sdev_target))
			spi_dv_device(SDp);
	} else {
		printk(KERN_DEBUG "53c700: Device %d:%llu configured for async operation (fast scsi disabled)\n",
		       SDp->id, SDp->lun);
		spi_offset(SDp->sdev_target) = 0;
		spi_period(SDp->sdev_target) = 0;
	}

	printk(KERN_DEBUG "53c700: Final config for device %d:%llu - period=%d, offset=%d\n",
	       SDp->id, SDp->lun, spi_period(SDp->sdev_target), spi_offset(SDp->sdev_target));

	return 0;
}

STATIC void
NCR_700_sdev_destroy(struct scsi_device *SDp)
{
	kfree(SDp->hostdata);
	SDp->hostdata = NULL;
}

static int
NCR_700_change_queue_depth(struct scsi_device *SDp, int depth)
{
	if (depth > NCR_700_MAX_TAGS)
		depth = NCR_700_MAX_TAGS;
	return scsi_change_queue_depth(SDp, depth);
}

static ssize_t
NCR_700_show_active_tags(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct scsi_device *SDp = to_scsi_device(dev);

	return snprintf(buf, 20, "%d\n", NCR_700_get_depth(SDp));
}

static struct device_attribute NCR_700_active_tags_attr = {
	.attr = {
		.name =		"active_tags",
		.mode =		S_IRUGO,
	},
	.show = NCR_700_show_active_tags,
};

STATIC struct attribute *NCR_700_dev_attrs[] = {
	&NCR_700_active_tags_attr.attr,
	NULL,
};

ATTRIBUTE_GROUPS(NCR_700_dev);

EXPORT_SYMBOL(NCR_700_detect);
EXPORT_SYMBOL(NCR_700_release);
EXPORT_SYMBOL(NCR_700_intr);

static struct spi_function_template NCR_700_transport_functions =  {
	.set_period	= NCR_700_set_period,
	.show_period	= 1,
	.set_offset	= NCR_700_set_offset,
	.show_offset	= 1,
};

static int __init NCR_700_init(void)
{
	NCR_700_transport_template = spi_attach_transport(&NCR_700_transport_functions);
	if(!NCR_700_transport_template)
		return -ENODEV;
	return 0;
}

static void __exit NCR_700_exit(void)
{
	spi_release_transport(NCR_700_transport_template);
}

module_init(NCR_700_init);
module_exit(NCR_700_exit);

