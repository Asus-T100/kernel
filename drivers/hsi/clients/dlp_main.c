/*
 * dlp_main.c
 *
 * Intel Mobile Communication modem protocol driver for DLP
 * (Data Link Protocl (DLP)). This driver is implementing a 5-channel HSI
 * protocol consisting of:
 * - An internal communication control channel;
 * - A multiplexed channel exporting a TTY interface;
 * - Three dedicated high speed channels exporting each a network interface.
 * All channels are using fixed-length pdus, although of different sizes.
 *
 * Copyright (C) 2010-2011 Intel Corporation. All rights reserved.
 *
 * Contact: Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/hsi/hsi.h>
#include <linux/dma-mapping.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/hsi/hsi.h>
#include <linux/proc_fs.h>

#include "dlp_main.h"

#define DEBUG_TAG 0x1
#define DEBUG_VAR dlp_drv.debug

/* Forward declarations */
static void dlp_pdu_destructor(struct hsi_msg *pdu);

static inline void dlp_ctx_update_state(struct dlp_xfer_ctx *xfer_ctx);

static inline void dlp_fifo_recycled_push(struct dlp_xfer_ctx *xfer_ctx,
					  struct hsi_msg *pdu);

/*
 * Static protocol driver global variables
 */
struct dlp_driver dlp_drv;

/* Module debug parameter */
module_param_named(debug, dlp_drv.debug, int, S_IRUGO | S_IWUSR);

/*
 *
 *
 */
static int dlp_proc_show(struct seq_file *m, void *v)
{
	int i;
	unsigned long flags;
	struct dlp_channel *ch_ctx;

	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = dlp_drv.channels[i];

		seq_printf(m, "\nChannel: %d\n", ch_ctx->hsi_channel);
		seq_printf(m, "-------------\n");
		seq_printf(m, " pdu_size  : %d\n", ch_ctx->pdu_size);
		seq_printf(m, " credits   : %d\n", ch_ctx->credits);

		read_lock_irqsave(&ch_ctx->rx.lock, flags);
		seq_printf(m, " RX ctx:\n");
		seq_printf(m, "   wait_max: %d\n", ch_ctx->rx.wait_max);
		seq_printf(m, "   ctrl_max: %d\n", ch_ctx->rx.ctrl_max);
		seq_printf(m, "   all_len : %d\n", ch_ctx->rx.all_len);
		seq_printf(m, "   ctrl_len: %d\n", ch_ctx->rx.ctrl_len);
		seq_printf(m, "   wait_len: %d\n", ch_ctx->rx.wait_len);
		seq_printf(m, "   seq_num : %d\n", ch_ctx->rx.seq_num);
		read_unlock_irqrestore(&ch_ctx->rx.lock, flags);

		read_lock_irqsave(&ch_ctx->tx.lock, flags);
		seq_printf(m, " TX ctx:\n");
		seq_printf(m, "   wait_max: %d\n", ch_ctx->tx.wait_max);
		seq_printf(m, "   ctrl_max: %d\n", ch_ctx->tx.ctrl_max);
		seq_printf(m, "   all_len : %d\n", ch_ctx->tx.all_len);
		seq_printf(m, "   ctrl_len: %d\n", ch_ctx->tx.ctrl_len);
		seq_printf(m, "   wait_len: %d\n", ch_ctx->tx.wait_len);
		seq_printf(m, "   seq_num : %d\n", ch_ctx->tx.seq_num);
		read_unlock_irqrestore(&ch_ctx->tx.lock, flags);
	}

	return 0;
}

static int dlp_proc_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, dlp_proc_show, PDE(inode)->data);
}

static const struct file_operations dlp_proc_ops = {
	.open = dlp_proc_seq_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release
};

/**
 * from_usecs - translating usecs to jiffies
 * @delay: the dealy in usecs
 *
 * Returns the delay rounded up to the next jiffy and prevent it to be set
 * to zero, as all delayed function calls shall occur to the next jiffy (at
 * least).
 */
inline unsigned long from_usecs(const unsigned long delay)
{
	unsigned long j = usecs_to_jiffies(delay);

	if (j == 0)
		j = 1;

	return j;
}

/****************************************************************************
 *
 * PDU creation and deletion
 *
 ***************************************************************************/

/*
 *
 *
 */
void dlp_pdu_dump(struct hsi_msg *pdu, int as_string)
{
	u32 *addr = sg_virt(pdu->sgt.sgl);

	PROLOG("0x%p [ttype: %s, nents: %d, status: %d, "
	       "channel: %d, actual_len: %d, break_frame: %d]",
	       pdu, (pdu->ttype == HSI_MSG_WRITE) ? "TX" : "RX", pdu->sgt.nents,
	       pdu->status, pdu->channel, pdu->actual_len, pdu->break_frame);

	/* Display as HEX */
	if (pdu->sgt.sgl->length == DLP_CTRL_PDU_SIZE) {
		PDEBUG("\n\t data[1-4]   : %08X", *(addr));
	} else {
		PDEBUG("\n\t data[1-4]   : %08X  %08X  %08X  %08X\n"
		       "\t data[5-8]   : %08X  %08X  %08X  %08X\n"
		       "\t data[9-12]  : %08X  %08X  %08X  %08X\n"
		       "\t data[13-16] : %08X  %08X  %08X  %08X\n",
		       *(addr), *(addr + 1), *(addr + 2), *(addr + 3),
		       *(addr + 4), *(addr + 5), *(addr + 6), *(addr + 7),
		       *(addr + 8), *(addr + 9), *(addr + 10), *(addr + 11),
		       *(addr + 12), *(addr + 13), *(addr + 14), *(addr + 15));

		/* Display as STRING */
		if (as_string) {
			unsigned int len;
			unsigned char *str;

			len = *(addr + 2);
			addr += 4;
			str = (char *)addr;
			str[len] = '\0';

			PDEBUG("\n\t len: %d, data: \"%s\"", len, str);
		}
	}

	EPILOG();
}

/**
 * dlp_buffer_alloc - helper function to allocate memory
 * @buff_size: buffer size
 * @dma_addr: buffer dma address
 *
 * Returns
 *	- a reference to the newly allocated buffer
 *	- NULL if an error occured.
 */
void *dlp_buffer_alloc(unsigned int buff_size, dma_addr_t * dma_addr)
{
	void *buff;
	int flags = in_interrupt()? GFP_ATOMIC : GFP_KERNEL;

	if (dlp_drv.is_dma_capable) {
		buff = dma_alloc_coherent(dlp_drv.controller,
					  buff_size, dma_addr, flags);
	} else {
		if ((buff_size >= PAGE_SIZE)
		    && ((buff_size & (PAGE_SIZE - 1)) == 0)) {
			buff =
			    (void *)__get_free_pages(flags,
						     get_order(buff_size));
		} else {
			buff = kmalloc(buff_size, flags);
		}
	}

	return buff;
}

/**
 * dlp_buffer_free - Free memory alloccted by dlp_buffer_alloc
 * @buff: buffer address
 * @dma_addr: buffer dma address
 * @buff_size: buffer size
 *
 */
void dlp_buffer_free(void *buff, dma_addr_t dma_addr, unsigned int buff_size)
{
	if (dlp_drv.is_dma_capable) {
		dma_free_coherent(dlp_drv.controller,
				  buff_size, buff, dma_addr);
	} else {
		if ((buff_size >= PAGE_SIZE)
		    && ((buff_size & (PAGE_SIZE - 1)) == 0)) {
			free_pages((unsigned int)buff, get_order(buff_size));
		} else {
			kfree(buff);
		}
	}
}

/**
 * dlp_pdu_alloc - helper function to allocate and initialise a new pdu
 * @hsi_channel: the HSI channel number
 * @ttype: pdu transfer type READ/WRITE
 * @buffer_size: pdu buffer size
 * @nb_entries: number of entries in the Scatter Gather table
 * @user_data: pdu context data (user data)
 * @complete_cb: xfer complete callback
 * @destruct_cb: pdu destruct callback
 *
 * Returns a reference to the newly created pdu or NULL if an error occured.
 */
struct hsi_msg *dlp_pdu_alloc(unsigned int hsi_channel,
			      int ttype,
			      int buffer_size,
			      int nb_entries,
			      void *user_data,
			      xfer_complete_cb complete_cb,
			      xfer_complete_cb destruct_cb)
{
	struct hsi_msg *new;
	void *buffer;
	int flags = in_interrupt()? GFP_ATOMIC : GFP_KERNEL;

	PROLOG("%d, %s, size:%d",
	       hsi_channel,
	       (ttype == HSI_MSG_WRITE) ? "TX" : "RX", buffer_size);

	new = hsi_alloc_msg(nb_entries, flags);
	if (!new) {
		CRITICAL("No more memory to allocate hsi_msg struct");
		goto out;
	}

	/* Allocate data buffer */
	buffer = dlp_buffer_alloc(buffer_size, &sg_dma_address(new->sgt.sgl));
	if (!buffer) {
		CRITICAL("No more memory to allocate hsi_msg data buffer");
		goto fail;
	}

	sg_set_buf(new->sgt.sgl, buffer, buffer_size);

	new->cl = dlp_drv.client;
	new->channel = hsi_channel;
	new->ttype = ttype;
	new->context = user_data;
	new->complete = complete_cb;
	new->destructor = destruct_cb;

	EPILOG("0x%p", new);
	return new;

fail:
	hsi_free_msg(new);

out:
	EPILOG("Failed");
	return NULL;
}

/**
 * dlp_pdu_free - helper function to delete and free an existing pdu
 * @pdu: a reference to the pdu to delete
 * @pdu_size: the real pdu size (allocated size)
 *
 * This function shall only be called by the pool of pdu management routines.
 */
void dlp_pdu_free(struct hsi_msg *pdu, unsigned int pdu_size)
{
	PROLOG("0x%p, %s, pdu_size: %d, sgl->length: %d",
	       pdu, (pdu->ttype == HSI_MSG_WRITE) ? "TX" : "RX",
	       pdu_size, pdu->sgt.sgl->length);

	/* Revert to the real allocated size */
	pdu->sgt.sgl->length = pdu_size;

	/* Free the data buffer */
	dlp_buffer_free(sg_virt(pdu->sgt.sgl),
			sg_dma_address(pdu->sgt.sgl), pdu_size);

	hsi_free_msg(pdu);

	EPILOG();
}

/**
 * dlp_pdu_delete - recycle or free a pdu
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 * @pdu: a reference to the pdu to delete
 *
 * This function is either recycling the pdu if there are not too many pdus
 * in the system, otherwise destroy it and free its resource.
 */
void dlp_pdu_delete(struct dlp_xfer_ctx *xfer_ctx, struct hsi_msg *pdu)
{
	int full;

	PROLOG("all_len: %d, wait_max: %d, ctrl_max: %d, pdu [0x%p, %s]",
	       xfer_ctx->all_len, xfer_ctx->wait_max, xfer_ctx->ctrl_max,
	       pdu, (pdu->ttype == HSI_MSG_WRITE) ? "TX" : "RX");

	full = (xfer_ctx->all_len > xfer_ctx->wait_max + xfer_ctx->ctrl_max);

	if (full) {
		dlp_pdu_free(pdu, xfer_ctx->channel->pdu_size);

		xfer_ctx->all_len--;
	} else {
		pdu->status = HSI_STATUS_COMPLETED;
		pdu->actual_len = 0;
		pdu->break_frame = 0;

		xfer_ctx->room += dlp_pdu_room_in(pdu);

		/* Recycle the pdu */
		dlp_fifo_recycled_push(xfer_ctx, pdu);
	}

	EPILOG("all_len: %d, wait_max: %d, ctrl_max: %d",
	       xfer_ctx->all_len, xfer_ctx->wait_max, xfer_ctx->ctrl_max);
}

/**
 * dlp_pdu_recycle - pdu recycling helper function for the RX side
 * @xfer_ctx: a reference to the RX context where recycled pdus FIFO sits
 * @pdu: a reference to the pdu that shall be recycled
 *
 * This helper method is recycling the pdu and pushing a new pdu to the
 * controller if there is room available in the controller FIFO, and finally
 * updating the state of the RX state machine.
 */
void dlp_pdu_recycle(struct dlp_xfer_ctx *xfer_ctx, struct hsi_msg *pdu)
{
	unsigned long flags;
	int have_space;

	PROLOG("0x%p, ctrl_len: %d", pdu, xfer_ctx->ctrl_len);

	/* Recycle or Free the pdu */
	write_lock_irqsave(&xfer_ctx->lock, flags);
	dlp_pdu_delete(xfer_ctx, pdu);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* The CTRL still have space ? */
	read_lock_irqsave(&xfer_ctx->lock, flags);
	have_space = xfer_ctx->ctrl_len < xfer_ctx->ctrl_max;
	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	if (have_space) {
		struct hsi_msg *new;
		int ret;

		/* Get a recycled pdu */
		new = dlp_fifo_recycled_pop(xfer_ctx);

		/* Push the pdu */
		ret = dlp_hsi_controller_push(xfer_ctx, new);
		if (ret)
			dlp_fifo_recycled_push(xfer_ctx, new);
	}

	dlp_ctx_update_state(xfer_ctx);
	EPILOG();
}

/**
 * dlp_pdu_check_header - Check the DLP header (Signature)
 * @pdu: a reference to the considered pdu
 *
 * Returns TRUE if the PDU signature is invalid
 */
inline int dlp_pdu_header_valid(struct hsi_msg *pdu)
{
	u32 *header = sg_virt(pdu->sgt.sgl);

	return ((header[0] & 0xFFFF0000) == DLP_HEADER_SIGNATURE);
}

/**
 * dlp_pdu_set_header - Write the DLP header (Signature+Sequence number)
 * @pdu: a reference to the considered pdu
 * @xfer_ctx: a reference to the xfer context
 *
 */
static inline void dlp_pdu_set_header(struct dlp_xfer_ctx *xfer_ctx,
				      struct hsi_msg *pdu)
{
	u32 *header = sg_virt(pdu->sgt.sgl);

	header[0] = (DLP_HEADER_SIGNATURE | (xfer_ctx->seq_num & 0xFFFF));
}

/**
 * dlp_pdu_get_offset - Get the offset information from the eDLP header
 * @pdu: a reference to the considered pdu
 *
 * Returns the offset information to encode in the header
 */
unsigned int dlp_pdu_get_offset(struct hsi_msg *pdu)
{
	u32 *header = (u32 *)(sg_virt(pdu->sgt.sgl));

	return header[1];
}

/**
 * dlp_pdu_get_length - Get the length information from the eDLP header
 * @pdu: a reference to the considered pdu
 *
 * Returns the length information to encode in the header
 */
inline unsigned int dlp_pdu_get_length(struct hsi_msg *pdu)
{
	u32 *header = (u32 *)(sg_virt(pdu->sgt.sgl));

	return (header[2] & 0x3FFFF);
}


/**
 * dlp_pdu_update - initialise a pdu for entering the RX wait FIFO
 * @ch_ctx: a reference to related channel context
 * @pdu: a reference to the considered pdu
 *
 * This helper function is simply updating the scatterlist information.
 */
void dlp_pdu_update(struct dlp_channel *ch_ctx, struct hsi_msg *pdu)
{
	struct scatterlist *sg = pdu->sgt.sgl;

	/* Use a non null pdu length when an error occur to forward it to
	 * the upper layers.
	 * Do not use the in-pdu length which can be broken */
	if ((!pdu->break_frame) && (pdu->status == HSI_STATUS_COMPLETED)) {
		pdu->actual_len = dlp_pdu_get_length(pdu);
	} else {
		CRITICAL
		    ("Invalid RX pdu (0x%p) status [status: %d, break_frame: %d,"
		     " actual_len: %d", pdu, pdu->status, pdu->break_frame,
		     pdu->actual_len);

		pdu->actual_len = 1;
	}

	/* If the decoded frame size is invalid, we have a big trouble */
	if ((!pdu->actual_len) || (pdu->actual_len > ch_ctx->pdu_size)) {
		CRITICAL("Invalid pdu (0x%p) size (0x%X Bytes)", pdu,
			 pdu->actual_len);

		pdu->status = HSI_STATUS_ERROR;
		pdu->actual_len = 1;
	}

	sg->length = 0;
}

/**
 * dlp_pdu_reset - revert a pdu to a working order
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 * @pdu: a reference to the considered pdu
 * @length: the pdu length to set
 *
 * This helper function is simply updating the scatterlist information.
 */
inline void dlp_pdu_reset(struct dlp_xfer_ctx *xfer_ctx,
			  struct hsi_msg *pdu, unsigned int length)
{
	struct scatterlist *sg = pdu->sgt.sgl;

	PROLOG("0x%p", pdu);

	sg->offset -= sg->length;
	sg->length = length;
	pdu->actual_len = 0;

	EPILOG();
};

/**
 * dlp_pdu_room_in - helper function for getting current room in the pdu
 * @pdu: a reference to the considered pdu
 *
 * Returns the room in byte in the current pdu
 */
inline unsigned int dlp_pdu_room_in(struct hsi_msg *pdu)
{
	unsigned int used_len = pdu->actual_len + DLP_TTY_HEADER_LENGTH;

	return (pdu->sgt.sgl->length - used_len);
}

/**
 * dlp_pdu_destructor - delete or recycle an existing pdu
 * @pdu: a reference to the pdu to delete
 *
 * This function shall only be called as an HSI destruct callback.
 */
static void dlp_pdu_destructor(struct hsi_msg *pdu)
{
	struct dlp_xfer_ctx *xfer_ctx = pdu->context;
	unsigned long flags;

	PROLOG("0x%p", pdu);

	/* Decrease the CTRL fifo size */
	write_lock_irqsave(&xfer_ctx->lock, flags);
	dlp_hsi_controller_pop(xfer_ctx);

	/* Recycle or Free the pdu */
	dlp_pdu_delete(xfer_ctx, pdu);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	if (xfer_ctx->ttype == HSI_MSG_READ)
		dlp_ctx_update_state(xfer_ctx);
	else if (dlp_ctx_is_empty(xfer_ctx))
		wake_up(&xfer_ctx->channel->tx_empty_event);

	EPILOG();
}

/****************************************************************************
 *
 * State handling helper
 *
 ***************************************************************************/

/**
 * _dlp_ctx_get_state - get the global state of a state machine
 * @xfer_ctx: a reference to the state machine context
 *
 * Returns the current state of the requested TX or RX context.
 */
static inline __must_check
    unsigned int _dlp_ctx_get_state(struct dlp_xfer_ctx *xfer_ctx)
{
	return (xfer_ctx->state & DLP_GLOBAL_STATE_MASK);
}

/**
 * dlp_ctx_is_state - checks the global state of a state machine
 * @xfer_ctx: a reference to the state machine context
 * @state: the state to consider
 *
 * Returns a non-zero value if in the requested state.
 */
static inline __must_check int dlp_ctx_is_state(struct dlp_xfer_ctx *xfer_ctx,
						unsigned int state)
{
	int is_state;
	unsigned long flags;
#ifdef DEBUG
	BUG_ON(state & ~DLP_GLOBAL_STATE_MASK);
#endif

	read_lock_irqsave(&xfer_ctx->lock, flags);
	is_state = (_dlp_ctx_get_state(xfer_ctx) == state);
	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	return is_state;
}

/**
 * dlp_ctx_get_state - get the global state of a state machine
 * @xfer_ctx: a reference to the state machine context
 *
 * Returns the current state of the requested TX or RX context.
 *
 * This version adds the spinlock guarding
 */
inline __must_check
    unsigned int dlp_ctx_get_state(struct dlp_xfer_ctx *xfer_ctx)
{
	unsigned int state;
	unsigned long flags;

	read_lock_irqsave(&xfer_ctx->lock, flags);
	state = _dlp_ctx_get_state(xfer_ctx);
	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	return state;
}

/**
 * dlp_ctx_set_state - sets the global state of a state machine
 * @xfer_ctx: a reference to the state machine context
 * @state: the state to set
 */
inline void dlp_ctx_set_state(struct dlp_xfer_ctx *xfer_ctx, unsigned int state)
{
	unsigned long flags;

#ifdef DEBUG
	BUG_ON(state & ~DLP_GLOBAL_STATE_MASK);
#endif

	write_lock_irqsave(&xfer_ctx->lock, flags);
	xfer_ctx->state = (xfer_ctx->state & ~DLP_GLOBAL_STATE_MASK) | state;
	write_unlock_irqrestore(&xfer_ctx->lock, flags);
}

/**
 * dlp_ctx_has_flag - checks if a flag is present in the state
 * @xfer_ctx: a reference to the state machine context
 * @flag: the flag(s) to consider
 *
 * Returns a non-zero value if all requested flags are present.
 */
inline __must_check int dlp_ctx_has_flag(struct dlp_xfer_ctx *xfer_ctx,
					 unsigned int flag)
{
	unsigned long flags;
	int has_flag;

#ifdef DEBUG
	BUG_ON(flag & DLP_GLOBAL_STATE_MASK);
#endif

	read_lock_irqsave(&xfer_ctx->lock, flags);
	has_flag = ((xfer_ctx->state & flag) == flag);
	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	return has_flag;
}

/**
 * dlp_ctx_set_flag - flags some extra information in the state
 * @xfer_ctx: a reference to the state machine context
 * @flag: the flag(s) to set
 */
inline void dlp_ctx_set_flag(struct dlp_xfer_ctx *xfer_ctx, unsigned int flag)
{
	unsigned long flags;

#ifdef DEBUG
	BUG_ON(flag & DLP_GLOBAL_STATE_MASK);
#endif

	write_lock_irqsave(&xfer_ctx->lock, flags);
	xfer_ctx->state |= flag;
	write_unlock_irqrestore(&xfer_ctx->lock, flags);
}

/**
 * dlp_ctx_clear_flag - unflags some extra information in the state
 * @xfer_ctx: a reference to the state machine context
 * @flag: the flag(s) to clear
 */
inline void dlp_ctx_clear_flag(struct dlp_xfer_ctx *xfer_ctx, unsigned int flag)
{
	unsigned long flags;

#ifdef DEBUG
	BUG_ON(flag & DLP_GLOBAL_STATE_MASK);
#endif

	write_lock_irqsave(&xfer_ctx->lock, flags);
	xfer_ctx->state &= ~flag;
	write_unlock_irqrestore(&xfer_ctx->lock, flags);
}

/**
 * dlp_ctx_is_empty_safe - checks if a context is empty (all FIFO are empty)
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 *
 * This helper function is returning a non-zero value if both the wait FIFO and
 * the controller FIFO are empty.
 */
int dlp_ctx_is_empty(struct dlp_xfer_ctx *xfer_ctx)
{
	int ret;
	unsigned long flags;

	PROLOG();

	read_lock_irqsave(&xfer_ctx->lock, flags);
	ret = ((xfer_ctx->wait_len == 0) && (xfer_ctx->ctrl_len == 0));
	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	EPILOG();
	return ret;
}

/**
 * dlp_ctx_have_credits - checks if a context have credits
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 * @ch_ctx: a reference to the channel context
 *
 * This helper function returns TRUE in these cases:
 *		- RX context
 *		- TX context with credits available.
 *		- The driver is used for loopback test mode (for debug).
 */
int dlp_ctx_have_credits(struct dlp_xfer_ctx *xfer_ctx,
			 struct dlp_channel *ch_ctx)
{
	int have_credits = 0;
	unsigned long flags;

	PROLOG();

	spin_lock_irqsave(&ch_ctx->lock, flags);
	if ((xfer_ctx->ttype == HSI_MSG_READ) ||
		((xfer_ctx->ttype == HSI_MSG_WRITE) && (ch_ctx->credits))) {
		have_credits = 1;
	}
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	EPILOG("%d", have_credits);
	return have_credits;
}

/**
 * dlp_ctx_update_status - updating the channel status further to config
 *			    changes (channel, pdu length)
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 */
void dlp_ctx_update_status(struct dlp_xfer_ctx *xfer_ctx)
{
	struct list_head *curr;
	struct hsi_msg *pdu;
	struct scatterlist *sg;
	unsigned long flags;

	PROLOG();

	read_lock_irqsave(&xfer_ctx->lock, flags);

	list_for_each(curr, &xfer_ctx->recycled_pdus) {
		pdu = list_entry(curr, struct hsi_msg, link);
		pdu->channel = xfer_ctx->channel->hsi_channel;

		sg = pdu->sgt.sgl;

		// FIXME: To be removed !!!
		sg->length = xfer_ctx->payload_len + DLP_TTY_HEADER_LENGTH;

		read_unlock_irqrestore(&xfer_ctx->lock, flags);

		write_lock_irqsave(&xfer_ctx->lock, flags);
		xfer_ctx->room += xfer_ctx->payload_len;
		xfer_ctx->room -= (sg->length - DLP_TTY_HEADER_LENGTH);
		write_unlock_irqrestore(&xfer_ctx->lock, flags);

		read_lock_irqsave(&xfer_ctx->lock, flags);
	}

	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	if (xfer_ctx->ttype == HSI_MSG_WRITE)
		dlp_drv.client->tx_cfg = xfer_ctx->config;
	else
		dlp_drv.client->rx_cfg = xfer_ctx->config;

	EPILOG();
}

/**
 * dlp_ctx_update_state_not_active - update the RX state machine upon reception of an
 * @xfer_ctx: a reference to the RX context to consider
 *
 * This helper function updates the RX state in accordance with the status of
 * the RX FIFO.
 */
static inline void dlp_ctx_update_state_not_active(struct dlp_xfer_ctx
						   *xfer_ctx)
{
	PROLOG();

	if (!dlp_ctx_is_empty(xfer_ctx))
		dlp_ctx_set_state(xfer_ctx, TTY);
	else
		dlp_ctx_set_state(xfer_ctx, IDLE);

	EPILOG();
}

/**
 * dlp_ctx_update_state - update the RX state machine upon recycling of a
 *			  RX pdu
 * @xfer_ctx: a reference to the xfer RX context to consider
 *
 * This helper function updates the RX state in accordance with the status of
 * the RX FIFO, unless the RX is required active.
 */
static inline void dlp_ctx_update_state(struct dlp_xfer_ctx *xfer_ctx)
{
	PROLOG();

	if (!dlp_ctx_is_state(xfer_ctx, ACTIVE))
		dlp_ctx_update_state_not_active(xfer_ctx);

	EPILOG();
}

/****************************************************************************
 *
 * FIFO common functions
 *
 ***************************************************************************/

/**
 * dlp_fifo_tail - get a reference to the last pdu of a FIFO or NULL
 *			 if the FIFO is empty
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 * @fifo: a reference of the FIFO to consider
 *
 * Returns a reference to the last pdu of this FIFO or NULL if the FIFO is
 * empty.
 */
inline __must_check
    struct hsi_msg *dlp_fifo_tail(struct dlp_xfer_ctx *xfer_ctx,
				  struct list_head *fifo)
{
	struct hsi_msg *pdu = NULL;

	/* Empty ? */
	if (! list_empty(fifo)) {
		pdu = list_entry(fifo->prev, struct hsi_msg, link);
	}

	return pdu;
}

/**
 * dlp_fifo_empty - deletes the whole content of a FIFO
 * @fifo: a reference to the FIFO to empty
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 *
 * This helper function is emptying a FIFO and deleting all its pdus.
 */
static void dlp_fifo_empty(struct list_head *fifo,
			   struct dlp_xfer_ctx *xfer_ctx)
{
	struct hsi_msg *pdu;
	unsigned long flags;

	PROLOG();

	write_lock_irqsave(&xfer_ctx->lock, flags);

	/* Empty ? */
	if (list_empty(fifo))
		goto out;

	while ((pdu = list_entry(fifo->next, struct hsi_msg, link))) {
		/* Remove the pdu from the list */
		list_del_init(&pdu->link);

		/* pdu free */
		dlp_pdu_free(pdu, xfer_ctx->channel->pdu_size);
	}

	write_unlock_irqrestore(&xfer_ctx->lock, flags);

 out:
	EPILOG();
}

/****************************************************************************
 *
 * Wait FIFO handling methods
 *
 ***************************************************************************/

/**
 * dlp_fifo_wait_pop - pop a pdu from the FIFO of waiting pdus
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 *
 * This function is not only popping the pdu, but also updating the counters
 * related to the FIFO of waiting pdus in the considered context.
 */
struct hsi_msg *dlp_fifo_wait_pop(struct dlp_xfer_ctx *xfer_ctx)
{
	struct list_head *fifo = &xfer_ctx->wait_pdus;
	struct hsi_msg *pdu = NULL;

	PROLOG("wait_len: %d, pdu: 0x%p", xfer_ctx->wait_len, pdu);

	/* Check if the list was not flushed */
	if (list_empty(fifo))
		goto out;

	/* Get the list head */
	pdu = list_entry(fifo->next, struct hsi_msg, link);

	/* Remove the pdu from the list */
	list_del_init(&pdu->link);

	xfer_ctx->wait_len --;
	xfer_ctx->buffered -= pdu->actual_len;

out:

	EPILOG("wait_len: %d", xfer_ctx->wait_len);
	return pdu;
}

/**
 * dlp_fifo_wait_push - push a pdu to the FIFO of waiting pdus
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 * @pdu: a reference to the pdu to push
 *
 * This function is not only pushing the pdu, but also updating the counters
 * related to the FIFO of waiting pdus in the considered context.
 */
inline void dlp_fifo_wait_push(struct dlp_xfer_ctx *xfer_ctx,
			       struct hsi_msg *pdu)
{
	unsigned long flags;

	PROLOG("wait_len: %d, pdu [0x%p, %s]", xfer_ctx->wait_len,
	       pdu, (pdu->ttype == HSI_MSG_WRITE) ? "TX" : "RX");

	write_lock_irqsave(&xfer_ctx->lock, flags);

	xfer_ctx->wait_len++;
	xfer_ctx->buffered += pdu->actual_len;

	/* at the end of the list */
	list_add_tail(&pdu->link, &xfer_ctx->wait_pdus);

	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	EPILOG("wait_len: %d", xfer_ctx->wait_len);
}

/**
 * dlp_fifo_wait_push_back - push back a pdu in the FIFO of waiting pdus
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 * @pdu: a reference to the pdu to push back
 *
 * This function is not only pushing back the pdu, but also updating the
 * counters related to the FIFO of waiting pdus in the considered context.
 */
inline void dlp_fifo_wait_push_back(struct dlp_xfer_ctx *xfer_ctx,
				    struct hsi_msg *pdu)
{
	unsigned long flags;

	PROLOG("wait_len: %d, pdu: 0x%p", xfer_ctx->wait_len, pdu);

	write_lock_irqsave(&xfer_ctx->lock, flags);

	xfer_ctx->wait_len++;
	xfer_ctx->buffered += pdu->actual_len;

	/* at the begining of the list */
	list_add(&pdu->link, &xfer_ctx->wait_pdus);

	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	EPILOG("wait_len: %d", xfer_ctx->wait_len);
}

/**
 * dlp_pop_wait_push_ctrl - transfer the first TX pdu from the wait FIFO to
 *			    the controller FIFO
 * @xfer_ctx: a reference to the TX context to consider
 * @check_pdu: to check if the pdu is being updated (marked as break pdu)
 *
 * This wrapper function is simply transferring the first pdu of the wait
 * FIFO.
 */
void dlp_pop_wait_push_ctrl(struct dlp_xfer_ctx *xfer_ctx,
			    unsigned int check_pdu)
{
	int ok = 1;
	unsigned long flags;
	struct hsi_msg *pdu;

	PROLOG("check_pdu: %d", check_pdu);

	write_lock_irqsave(&xfer_ctx->lock, flags);
	pdu = dlp_fifo_wait_pop(xfer_ctx);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	if (!pdu)
		goto out;

	if (check_pdu) {
		ok = !pdu->break_frame;
	}

	if (ok) {
		/* Note that no error is returned upon transfer failure,
		 * in such cases, the pdu is simply returned back to the
		 * wait FIFO, as nothing else can be done
		 */
		int ret;
		unsigned int actual_len = pdu->actual_len;

		/* */
		ret = dlp_hsi_controller_push(xfer_ctx, pdu);
		if (ret) {
			if (ret == -EACCES) {	/* port released */

				write_lock_irqsave(&xfer_ctx->lock, flags);
				xfer_ctx->room -= dlp_pdu_room_in(pdu);

				/* Recycle or Free the pdu */
				dlp_pdu_delete(xfer_ctx, pdu);

				write_unlock_irqrestore(&xfer_ctx->lock, flags);
			} else {
				/* Push back the pdu */
				dlp_fifo_wait_push_back(xfer_ctx, pdu);
			}
		}
#ifdef CONFIG_HSI_DLP_TTY_STATS
		else {
			xfer_ctx->tty_stats.data_sz += actual_len;
			xfer_ctx->tty_stats.pdus_cnt++;
		}
#endif
	}

 out:
	EPILOG();
}

/****************************************************************************
 *
 * Frame recycling helper functions
 *
 ***************************************************************************/

static inline void dlp_fifo_recycled_push(struct dlp_xfer_ctx *xfer_ctx,
					  struct hsi_msg *pdu)
{
	PROLOG("pdu [0x%p, %s, sgl->len: %d]",
	       pdu,
	       (pdu->ttype == HSI_MSG_WRITE) ? "TX" : "RX",
	       pdu->sgt.sgl->length);

	/* at the end of the list */
	list_add_tail(&pdu->link, &xfer_ctx->recycled_pdus);

	EPILOG();
}

/**
 * dlp_fifo_recycled_pop - creating a new empty file from the recycling FIFO
 * @xfer_ctx: a reference to the xfer context (RX or TX) to consider
 *
 * Returns a reference to the new empty pdu or NULL if there are no recycled
 * pdus left.
 */
struct hsi_msg *dlp_fifo_recycled_pop(struct dlp_xfer_ctx *xfer_ctx)
{
	struct hsi_msg *pdu = NULL;
	struct list_head *first;
	unsigned long flags;

	PROLOG();

	read_lock_irqsave(&xfer_ctx->lock, flags);
	first = &xfer_ctx->recycled_pdus;

	/* Empty ? */
	if (first != first->next) {
		/* Get the fist pdu */
		pdu = list_entry(first->next, struct hsi_msg, link);

		read_unlock_irqrestore(&xfer_ctx->lock, flags);

		/* Remove the pdu from the list */
		write_lock_irqsave(&xfer_ctx->lock, flags);
		list_del_init(&pdu->link);
		write_unlock_irqrestore(&xfer_ctx->lock, flags);

		EPILOG("pdu [0x%p, %s, %d]",
		       pdu,
		       (pdu->ttype == HSI_MSG_WRITE) ? "TX" : "RX",
		       pdu->sgt.sgl->length);
	} else {
		read_unlock_irqrestore(&xfer_ctx->lock, flags);

		EPILOG("pdu [NULL]");
	}

	return pdu;
}

/**
 * dlp_pop_recycled_push_ctrl - Push as many recycled pdus as possible to the
 *			     controller FIFO
 * @xfer_ctx: a reference to the RX context where the FIFO of recycled pdus sits
 *
 * Returns 0 upon success or an error code.
 *
 * This helper method is returning 0 on success, or an error code.
 */
__must_check int dlp_pop_recycled_push_ctrl(struct dlp_xfer_ctx *xfer_ctx)
{
	int ret = 0;
	struct hsi_msg *new;
	unsigned long flags;

	PROLOG();

	read_lock_irqsave(&xfer_ctx->lock, flags);

	while (xfer_ctx->ctrl_len < xfer_ctx->ctrl_max) {
		read_unlock_irqrestore(&xfer_ctx->lock, flags);

		new = dlp_fifo_recycled_pop(xfer_ctx);
		if (!new) {
			ret = -ENOMEM;
			goto out;
		}

		ret = dlp_hsi_controller_push(xfer_ctx, new);
		if (ret) {
			dlp_fifo_recycled_push(xfer_ctx, new);

			ret = -EAGAIN;
			goto out;
		}

		read_lock_irqsave(&xfer_ctx->lock, flags);
	}

	read_unlock_irqrestore(&xfer_ctx->lock, flags);

out:
	EPILOG("ret: %d", ret);
	return ret;
}

/****************************************************************************
 *
 * HSI Controller
 *
 ***************************************************************************/

/**
 * dlp_hsi_controller_push - push a pdu to the HSI controller FIFO
 * @xfer_ctx: a reference to the xfer context (TX or RX) to consider
 * @pdu: a reference to the pdu to push
 *
 * Returns 0 on success or an error code on failure.
 *
 * This function is not only pushing the pdu, but also updating the counters
 * related to the FIFO of outstanding pdus in the considered context.
 */
int dlp_hsi_controller_push(struct dlp_xfer_ctx *xfer_ctx, struct hsi_msg *pdu)
{
	unsigned int lost_room = dlp_pdu_room_in(pdu);
	struct dlp_channel *ch_ctx = xfer_ctx->channel;
	unsigned long flags;
	int err = 0;

	PROLOG("ctrl_len: %d, credits: %d, pdu [0x%p, %s, %d]",
	       xfer_ctx->ctrl_len, xfer_ctx->channel->credits,
	       pdu,
	       (pdu->ttype == HSI_MSG_WRITE) ? "TX" : "RX",
	       pdu->sgt.sgl->length);

	/* Check credits */
	if (!dlp_ctx_have_credits(xfer_ctx, ch_ctx)) {
		CRITICAL("NO credits avaible");
		goto out;
	}

	/* Decrease counters values */
	write_lock_irqsave(&xfer_ctx->lock, flags);

	xfer_ctx->room -= lost_room;
	xfer_ctx->ctrl_len++;

	if (pdu->ttype == HSI_MSG_WRITE)
		xfer_ctx->channel->credits--;

	xfer_ctx->seq_num++;
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* Set the DLP signature + seq_num */
	dlp_pdu_set_header(xfer_ctx, pdu);

	/* Dump the PDU */
	if (pdu->ttype == HSI_MSG_WRITE) {
		dlp_pdu_dump(pdu, 0);
		/* dlp_dbg_dump_pdu(pdu, 16, 160, 0); */
	}

	err = hsi_async(pdu->cl, pdu);
	if (!err) {
		if ((pdu->ttype == HSI_MSG_WRITE) && (xfer_ctx->ctrl_len)) {
			mod_timer(&ch_ctx->hangup.timer,
				  jiffies + usecs_to_jiffies(DLP_HANGUP_DELAY));
		}
	} else {
		unsigned int ctrl_len;

		CRITICAL("hsi_async() failed (%d)", err);

		/* Failed to send pdu, set back counters values */
		write_lock_irqsave(&xfer_ctx->lock, flags);

		xfer_ctx->seq_num--;
		xfer_ctx->room += lost_room;
		xfer_ctx->ctrl_len--;

		if (pdu->ttype == HSI_MSG_WRITE)
			xfer_ctx->channel->credits++;

		ctrl_len = xfer_ctx->ctrl_len;

		write_unlock_irqrestore(&xfer_ctx->lock, flags);

		if (!ctrl_len) {
			struct dlp_channel *ch_ctx = xfer_ctx->channel;
			del_timer(&ch_ctx->hangup.timer);
		}
	}

out:
	EPILOG("ctrl_len: %d, credits: %d",
	       xfer_ctx->ctrl_len, xfer_ctx->channel->credits);
	return err;
}

/**
 * dlp_hsi_controller_pop - pop a pdu from the HSI controller FIFO
 * @xfer_ctx: a reference to the xfer context (TX or RX) to consider
 *
 * This function is only updating the counters related to the FIFO of
 * outstanding pdus in the considered context.
 */
inline void dlp_hsi_controller_pop(struct dlp_xfer_ctx *xfer_ctx)
{
	PROLOG("ctrl_len: %d", xfer_ctx->ctrl_len);

	xfer_ctx->ctrl_len--;

	EPILOG("ctrl_len: %d", xfer_ctx->ctrl_len);
}

/**
 * dlp_hsi_start_tx - update the TX state machine on every new transfer
 * @xfer_ctx: a reference to the TX context to consider
 *
 * This helper function updates the TX state if it is currently idle and
 * inform the HSI pduwork and attached controller.
 */
void dlp_hsi_start_tx(struct dlp_xfer_ctx *xfer_ctx)
{
	PROLOG();

	/* we found the case that we called hsi_port_shutdown and
	 * stop_tx, while the tty want to flush data, in this case
	 * just ignore the start_tx, and the code in ctl push already
	 * ignore the flush and discard the packet.
	 */
	if (!hsi_port_claimed(dlp_drv.client))
		return;

	if (dlp_ctx_is_state(xfer_ctx, IDLE)) {
		int ret;

		dlp_ctx_set_state(xfer_ctx, ACTIVE);

		ret = hsi_start_tx(dlp_drv.client);
		if (ret)
			dlp_ctx_set_state(xfer_ctx, IDLE);
	} else {
		del_timer(&xfer_ctx->timer);
	}

	EPILOG();
}

/**
 * dlp_stop_tx - update the TX state machine after expiration of the TX active
 *		  timeout further to a no outstanding TX transaction status
 * @xfer_ctx: a reference to the TX context to consider
 *
 * This helper function updates the TX state if it is currently active and
 * inform the HSI pduwork and attached controller.
 */
void dlp_stop_tx(struct dlp_xfer_ctx *xfer_ctx)
{
	PROLOG();

	if (dlp_ctx_is_state(xfer_ctx, ACTIVE)) {
		dlp_ctx_set_state(xfer_ctx, IDLE);

		hsi_stop_tx(dlp_drv.client);
	}

	EPILOG();
}

/**
 * dlp_stop_rx - update the internal RX state machine
 * @xfer_ctx: a reference to the RX context to consider
 * @ch_ctx: a reference to related channel context
 *
 * This helper function updates the RX state and allows the HSI device to
 * sleep.
 */
inline void dlp_stop_rx(struct dlp_xfer_ctx *xfer_ctx,
			struct dlp_channel *ch_ctx)
{
	PROLOG();

	dlp_ctx_update_state_not_active(xfer_ctx);

	EPILOG();
}

/**
 * dlp_hsi_port_claim - Claim & setup the HSI port
 *
 */
int dlp_hsi_port_claim(void)
{
	int ret = 0;

	PROLOG();

	/* Claim the HSI port (if not already done) */
	if (hsi_port_claimed(dlp_drv.client))
		goto out;

	/* Claim the HSI port */
	ret = hsi_claim_port(dlp_drv.client, 1);
	if (unlikely(ret)) {
		CRITICAL("hsi_claim_port() failed (%d)", ret);
		goto out;
	}

	/* Setup the HSI controller */
	ret = hsi_setup(dlp_drv.client);
	if (unlikely(ret)) {
		CRITICAL("hsi_setup() failed (%d)", ret);
		hsi_release_port(dlp_drv.client);
		goto out;
	}

out:
	EPILOG();
	return ret;
}

/**
 * dlp_hsi_port_unclaim - UnClaim (release) the HSI port
 *
 */
inline void dlp_hsi_port_unclaim(void)
{
	PROLOG();

	if (hsi_port_claimed(dlp_drv.client))
		hsi_release_port(dlp_drv.client);

	EPILOG();
}

/**
 * dlp_hsi_start_rx_cb - update the internal RX state machine
 * @cl: a reference to HSI client to consider
 *
 * This helper function updates the RX state and wakes the device.
 */
static void dlp_hsi_start_rx_cb(struct hsi_client *cl)
{
	// FIXME: only ch_ctx[1]
	struct dlp_channel *ch_ctx = (struct dlp_channel *)hsi_client_drvdata(cl);
	struct dlp_xfer_ctx *xfer_ctx = &ch_ctx->rx;

	PROLOG();

	dlp_ctx_set_state(xfer_ctx, ACTIVE);

	EPILOG();
}

/**
 * dlp_hsi_stop_rx_cb - update the internal RX state machine
 * @cl: a reference to HSI client to consider
 *
 * This helper function updates the RX state and allows the HSI device to
 * sleep.
 */
static void dlp_hsi_stop_rx_cb(struct hsi_client *cl)
{
	// FIXME: only ch_ctx[1]
	struct dlp_channel *ch_ctx = (struct dlp_channel *)hsi_client_drvdata(cl);
	struct dlp_xfer_ctx *xfer_ctx = &ch_ctx->rx;

	PROLOG();

	dlp_stop_rx(xfer_ctx, ch_ctx);

	EPILOG();
}

/*
* @brief This function is used to deactivate the HSI client RX callbacks
*
* @param start_rx_cb : Start RX callback backup function
* @param stop_rx_cb : Stop RX callback backup function
*/
void dlp_save_rx_callbacks(hsi_client_cb *start_rx_cb, hsi_client_cb *stop_rx_cb)
{
	PROLOG();

	/* Save the current client CB */
	(*start_rx_cb) = dlp_drv.client->hsi_start_rx;
	(*stop_rx_cb) = dlp_drv.client->hsi_stop_rx;

	/* Set to NULL the CB pointer */
	dlp_drv.client->hsi_start_rx = NULL;
	dlp_drv.client->hsi_stop_rx = NULL;

	EPILOG();
}

/*
* @brief This function is used to reactivate the HSI client RX callbacks
*
* @param start_rx_cb : Start RX callback to set
* @param stop_rx_cb : Stop RX callback to set
*/
void dlp_restore_rx_callbacks(hsi_client_cb *start_rx_cb, hsi_client_cb *stop_rx_cb)
{
	PROLOG();

	/* Restore the client CB */
	dlp_drv.client->hsi_start_rx = (*start_rx_cb);
	dlp_drv.client->hsi_stop_rx = (*stop_rx_cb);

	/* Set to NULL the CB pointer */
	start_rx_cb = NULL;
	stop_rx_cb = NULL;

	EPILOG();
}

/**
 * dlp_increase_pdus_pool - background work aimed at creating new pdus
 * @work: a reference to the work context
 *
 * This function is called as a background job (in the dlp_recycle_wq work
 * queue) for performing the pdu resource allocation (which can then sleep).
 *
 * An error message is sent upon the failure of DLP_PDU_ALLOC_RETRY_MAX_CNT
 * allocation requests.
 */
static void dlp_increase_pdus_pool(struct work_struct *work)
{
	struct dlp_xfer_ctx *xfer_ctx = container_of(work, struct dlp_xfer_ctx,
						     increase_pool);
	struct hsi_msg *new;
	unsigned long flags;
	int retry;

	PROLOG();

	read_lock_irqsave(&xfer_ctx->lock, flags);

	while (xfer_ctx->all_len < (xfer_ctx->wait_max + xfer_ctx->ctrl_max)) {
		read_unlock_irqrestore(&xfer_ctx->lock, flags);

		retry = 0;
		new = dlp_pdu_alloc(xfer_ctx->channel->hsi_channel,
				    xfer_ctx->ttype,
				    xfer_ctx->channel->pdu_size,
				    1,
				    xfer_ctx,
				    xfer_ctx->complete_cb, dlp_pdu_destructor);

		while (!new) {
			++retry;
			if (retry == DLP_PDU_ALLOC_RETRY_MAX_CNT) {
				CRITICAL("Cannot allocate a pdu after %d retries...", retry);
				retry = 0;
			}

			/* No memory available: do something more urgent ! */
			schedule();

			new = dlp_pdu_alloc(xfer_ctx->channel->hsi_channel,
					    xfer_ctx->ttype,
					    xfer_ctx->channel->pdu_size,
					    1,
					    xfer_ctx,
					    xfer_ctx->complete_cb,
					    dlp_pdu_destructor);
		}

		write_lock_irqsave(&xfer_ctx->lock, flags);
		xfer_ctx->room += xfer_ctx->payload_len;
		xfer_ctx->all_len++;

		dlp_fifo_recycled_push(xfer_ctx, new);

		write_unlock_irqrestore(&xfer_ctx->lock, flags);

		read_lock_irqsave(&xfer_ctx->lock, flags);
	}

	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	EPILOG("%s pdu's pool created (hsi_channel: %d)",
	       (xfer_ctx->ttype == HSI_MSG_WRITE ? "TX" : "RX"),
	       xfer_ctx->channel->hsi_channel);
}

/****************************************************************************
 *
 * Hangup/Reset management
 *
 ***************************************************************************/
/*
* dlp_hangup_ctx_init - Initialises the given hangup context
*
* @param ch_ctx : Channel context to consider
* @param work_func : Work callback
* @param timeout_func : Timeout callback
* @param data : Timeout callback user data
*/
void dlp_hangup_ctx_init(struct dlp_channel *ch_ctx,
		void (* work_func)(struct work_struct *work),
		void (* timeout_func)(unsigned long int param),
		void *data)
{
	PROLOG();

	/* Init values */
	ch_ctx->hangup.cause = 0;
	ch_ctx->hangup.last_cause = 0;

	/* Worker function */
	INIT_WORK(&ch_ctx->hangup.work, work_func);

	/* TX Timeout timer */
	init_timer(&ch_ctx->hangup.timer);
	ch_ctx->hangup.timer.function = timeout_func;
	ch_ctx->hangup.timer.data = (unsigned long int)data;

	EPILOG();
}

/**
 * dlp_hangup_ctx_deinit - Clears a hangup context
 * @hangup_ctx: a reference to the considered hangup context
 */
void dlp_hangup_ctx_deinit(struct dlp_channel *ch_ctx)
{
	struct dlp_xfer_ctx *xfer_ctx = &ch_ctx->tx;
	unsigned long flags;
	int is_hunging_up;

	PROLOG();

	write_lock_irqsave(&xfer_ctx->lock, flags);
	is_hunging_up = (ch_ctx->hangup.cause);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* No need to wait for the end of the calling work! */
	if (! is_hunging_up) {
		if (del_timer_sync(&ch_ctx->hangup.timer))
			cancel_work_sync(&ch_ctx->hangup.work);
		else
			flush_work(&ch_ctx->hangup.work);
	}

	EPILOG();
}

/****************************************************************************
 *
 * RX/TX xfer contexts
 *
 ***************************************************************************/

/**
 * dlp_xfer_ctx_init - initialise a TX or RX context after its creation
 * @ch_ctx: a reference to its related channel context
 * @xfer_ctx: a reference to the considered TX or RX context
 * @delay: the initial delay for the timer related to the TX or RX context
 * @wait_max: the maximal size of the wait FIFO for this context
 * @ctrl_max: the maximal size of the HSI controller FIFO for this context
 * @complete_cb: HSI transfer complete callback
 * @ttype: HSI transfer type
 *
 * This helper function is simply filling in the initial data into a newly
 * created TX or RX context.
 */
void dlp_xfer_ctx_init(struct dlp_channel *ch_ctx,
		       struct dlp_xfer_ctx *xfer_ctx,
		       unsigned int delay,
		       unsigned int wait_max,
		       unsigned int ctrl_max,
		       xfer_complete_cb complete_cb, unsigned int ttype)
{
	PROLOG("delay: %d, wait_max: %d, ctrl_max: %d",
	       delay, wait_max, ctrl_max);

	INIT_LIST_HEAD(&xfer_ctx->wait_pdus);
	INIT_LIST_HEAD(&xfer_ctx->recycled_pdus);

	init_timer(&xfer_ctx->timer);
	rwlock_init(&xfer_ctx->lock);

	xfer_ctx->timer.data = (unsigned long)xfer_ctx;
	xfer_ctx->delay = from_usecs(delay);
	xfer_ctx->state = IDLE;
	xfer_ctx->wait_max = wait_max;
	xfer_ctx->ctrl_max = ctrl_max;
	xfer_ctx->channel = ch_ctx;
	xfer_ctx->payload_len = DLP_TTY_PAYLOAD_LENGTH;
	xfer_ctx->ttype = ttype;
	xfer_ctx->complete_cb = complete_cb;
	INIT_WORK(&xfer_ctx->increase_pool, dlp_increase_pdus_pool);

	EPILOG();
}

/**
 * dlp_xfer_ctx_clear - clears a TX or RX context prior to its deletion
 * @xfer_ctx: a reference to the considered TX or RX context
 *
 * This helper function is simply calling the relevant destructors
 * and reseting the context information.
 */
void dlp_xfer_ctx_clear(struct dlp_xfer_ctx *xfer_ctx)
{
	unsigned long flags;

	PROLOG();

	write_lock_irqsave(&xfer_ctx->lock, flags);

	xfer_ctx->wait_max = 0;
	xfer_ctx->ctrl_max = 0;
	xfer_ctx->state = IDLE;
	del_timer(&xfer_ctx->timer);

	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	dlp_fifo_empty(&xfer_ctx->wait_pdus, xfer_ctx);
	dlp_fifo_empty(&xfer_ctx->recycled_pdus, xfer_ctx);

	flush_work(&xfer_ctx->increase_pool);

	EPILOG();
}

/****************************************************************************
 *
 * Protocol driver functions
 *
 ***************************************************************************/

/**
 * dlp_driver_cleanup - Release driver allocated resources
 *
 * This helper function calls the "delete" function for each
 *	channel context.
 */
static void dlp_driver_cleanup(void)
{
	int i = 0;
	/* Contexts allocation functions */
	/* NOTE : this array should be aligned with the context enum
	 *                defined in the .h file */
	dlp_context_delete delete_funcs[DLP_CHANNEL_COUNT] = {
		dlp_ctrl_ctx_delete,	/* CTRL */
		dlp_tty_ctx_delete,		/* TTY  */
		dlp_net_ctx_delete,		/* NET  */
		dlp_net_ctx_delete,		/* NET  */
		dlp_net_ctx_delete};	/* NET  */

	PROLOG();

	/* Free DLP contexts */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		if (dlp_drv.channels[i]) {
			delete_funcs[i] (dlp_drv.channels[i]);
			dlp_drv.channels[i] = NULL;
		}
	}

	EPILOG();
}

/**
 * dlp_driver_probe - creates a new context in the DLP driver
 * @dev: a reference to the HSI device requiring a context
 *
 * Returns 0 upon success or an error in case of an error
 *
 * This function is creating a new context per HSI controller requiring a
 * DLP protocol driver, creates the related TTY port and TTY entry in the
 * filesystem.
 */
static int __init dlp_driver_probe(struct device *dev)
{
	/* The parent of our device is the HSI port,
	 * the parent of the HSI port is the HSI controller device */
	struct device *controller = dev->parent->parent;
	struct hsi_client *client = to_hsi_client(dev);
	int i, ret = 0;

	/* Contexts allocation functions */
	/* NOTE : this array should be aligned with the context enum
	 *                defined in the .h file */
	dlp_context_create create_funcs[DLP_CHANNEL_COUNT] = {
		dlp_ctrl_ctx_create,	/* CTRL */
		dlp_tty_ctx_create,		/* TTY  */
		dlp_net_ctx_create,		/* NET  */
		dlp_net_ctx_create,		/* NET  */
		dlp_net_ctx_create};	/* NET  */

	PROLOG();

	/* Save the controller & client */
	dlp_drv.controller = controller;
	dlp_drv.client = client;
	dlp_drv.is_dma_capable = is_device_dma_capable(controller);
	spin_lock_init(&dlp_drv.lock);

	/* Warn if no DMA capability */
	if (!dlp_drv.is_dma_capable) {
		WARNING("HSI device is not DMA capable");
	}

	/* Save IPC controller configs */
	dlp_drv.ipc_rx_cfg = client->rx_cfg;
	dlp_drv.ipc_tx_cfg = client->tx_cfg;

	/* Save the Boot/Flashing controller config */
	/* And deactivate the "Channel description" bits */
	/* because not managed by the modem */
	dlp_drv.flash_rx_cfg = client->rx_cfg;
	dlp_drv.flash_tx_cfg = client->tx_cfg;
	dlp_drv.flash_rx_cfg.channels = 1;
	dlp_drv.flash_tx_cfg.channels = 1;

	/* FIXME: Claim the HSI port */
	ret = dlp_hsi_port_claim();
	if (ret) {
		goto out;
	}

	/* Create DLP contexts */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		dlp_drv.channels[i] = create_funcs[i] (i, dev);
		if (!dlp_drv.channels[i]) {
			goto cleanup;
		}
	}

	/*  */
	client->hsi_start_rx = dlp_hsi_start_rx_cb;
	client->hsi_stop_rx = dlp_hsi_stop_rx_cb;

	/* FIXME : other channels ? */
	hsi_client_set_drvdata(client, dlp_drv.channels[DLP_CHANNEL_TTY]);

	/* Create /proc/hsi-dlp */
	// FIXME: Will be removed
	//if (dlp_drv.debug)
	{
		proc_create_data(DRVNAME, S_IRUGO, NULL, &dlp_proc_ops, NULL);
	}

	EPILOG();
	return 0;

cleanup:
	dlp_driver_cleanup();

out:
	EPILOG("%d", ret);
	return ret;
}

/**
 * dlp_driver_remove - removes a context from the DLP driver
 * @dev: a reference to the device requiring the context
 *
 * Returns 0 on success or an error code
 *
 * This function is freeing all resources hold by the context attached to the
 * requesting HSI device.
 */
static int __exit dlp_driver_remove(struct device *dev)
{
	struct hsi_client *client = to_hsi_client(dev);

	PROLOG();

	client->hsi_start_rx = NULL;
	client->hsi_stop_rx = NULL;
	hsi_client_set_drvdata(client, NULL);

	dlp_driver_cleanup();

	/* UnClaim the HSI port */
	dlp_hsi_port_unclaim();

	EPILOG();
	return 0;
}

/*
 * dlp_driver_setup - configuration of the DLP driver
 */
static struct hsi_client_driver dlp_driver_setup = {
	.driver = {
		   .name = DRVNAME,
		   .owner = THIS_MODULE,
		   .probe = dlp_driver_probe,
		   .remove = dlp_driver_remove,
		   },
};

/**
 * dlp_module_init - initialises the DLP driver common parts
 *
 * Returns 0 on success or an error code
 */
static int __init dlp_module_init(void)
{
	int err, debug_value;

	/* Save the debug param value */
	debug_value = dlp_drv.debug;

	/* Initialization */
	memset(&dlp_drv, 0, sizeof(struct dlp_driver));

	/* Restore the debug param value */
	dlp_drv.debug = debug_value;

	PROLOG();

	/* Create the workqueue for allocating pdus */
	dlp_drv.recycle_wq = create_singlethread_workqueue(DRVNAME "-wq");
	if (unlikely(!dlp_drv.recycle_wq)) {
		CRITICAL("Unable to create pool-handling workqueue");
		err = -EFAULT;
		goto out;
	}

	/* Create the workqueue for tx hangup */
	dlp_drv.tx_hangup_wq = create_singlethread_workqueue(DRVNAME "-hwq");
	if (unlikely(!dlp_drv.tx_hangup_wq)) {
		CRITICAL("Unable to create tx hangup workqueue");
		err = -EFAULT;
		goto del_wq;
	}

	/* Create the workqueue for TTY line discipline buffer flush */
	dlp_drv.forwarding_wq = create_singlethread_workqueue(DRVNAME "-hwq");
	if (unlikely(!dlp_drv.forwarding_wq)) {
		CRITICAL("Unable to create TTY forwarding workqueue");
		err = -EFAULT;
		goto del_2wq;
	}

	/* Now, register the client */
	err = hsi_register_client_driver(&dlp_driver_setup);
	if (unlikely(err)) {
		CRITICAL("hsi_register_client_driver() failed (%d)", err);
		goto del_3wq;
	}

	EPILOG("driver initialized");
	return 0;

del_3wq:
	destroy_workqueue(dlp_drv.forwarding_wq);

del_2wq:
	destroy_workqueue(dlp_drv.tx_hangup_wq);

del_wq:
	destroy_workqueue(dlp_drv.recycle_wq);

out:
	EPILOG("Failed");
	return err;
}

/**
 * dlp_driver_exit - frees the resources taken by the DLP driver common parts
 */
static void __exit dlp_module_exit(void)
{
	PROLOG();

	destroy_workqueue(dlp_drv.forwarding_wq);
	destroy_workqueue(dlp_drv.recycle_wq);
	destroy_workqueue(dlp_drv.tx_hangup_wq);

	hsi_unregister_client_driver(&dlp_driver_setup);

	EPILOG("driver removed");
}

module_init(dlp_module_init);
module_exit(dlp_module_exit);

MODULE_AUTHOR("Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>");
MODULE_AUTHOR("Faouaz Tenoutit <faouazx.tenoutit@intel.com>");
MODULE_DESCRIPTION("LTE protocol driver over HSI for IMC modems");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.1-HSI-LTE");
