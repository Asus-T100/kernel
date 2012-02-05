/*
 *  intel_sst_pvt.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10	Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Harsha Priya <priya.harsha@intel.com>
 *		Dharageswari R <dharageswari.r@intel.com>
 *		KP Jeeja <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This driver exposes the audio engine functionalities to the ALSA
 *	and middleware.
 *
 *  This file contains all private functions
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <sound/intel_sst.h>
#include <sound/intel_sst_ioctl.h>
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

/*
 * sst_get_block_stream - get a new block stream
 *
 * @sst_drv_ctx: Driver context structure
 *
 * This function assigns a block for the calls that dont have stream context yet
 * the blocks are used for waiting on Firmware's response for any operation
 * Should be called with stream lock held
 */
int sst_get_block_stream(struct intel_sst_drv *sst_drv_ctx)
{
	int i;

	for (i = 0; i < MAX_ACTIVE_STREAM; i++) {
		if (sst_drv_ctx->alloc_block[i].sst_id == BLOCK_UNINIT) {
			sst_drv_ctx->alloc_block[i].ops_block.condition = false;
			sst_drv_ctx->alloc_block[i].ops_block.ret_code = 0;
			sst_drv_ctx->alloc_block[i].sst_id = 0;
			break;
		}
	}
	if (i == MAX_ACTIVE_STREAM) {
		pr_err("max alloc_stream reached\n");
		i = -EBUSY; /* active stream limit reached */
	}
	return i;
}

/*
 * sst_wait_interruptible - wait on event
 *
 * @sst_drv_ctx: Driver context
 * @block: Driver block to wait on
 *
 * This function waits without a timeout (and is interruptable) for a
 * given block event
 */
int sst_wait_interruptible(struct intel_sst_drv *sst_drv_ctx,
				struct sst_block *block)
{
	int retval = 0;

	if (!wait_event_interruptible(sst_drv_ctx->wait_queue,
				block->condition)) {
		/* event wake */
		if (block->ret_code < 0) {
			pr_err("stream failed %d\n", block->ret_code);
			retval = -EBUSY;
		} else {
			pr_debug("event up\n");
			retval = 0;
		}
	} else {
		pr_err("signal interrupted\n");
		retval = -EINTR;
	}
	return retval;

}

/*
 * sst_wait_timeout - wait on event for timeout
 *
 * @sst_drv_ctx: Driver context
 * @block: Driver block to wait on
 *
 * This function waits with a timeout value (and is not interruptible) on a
 * given block event
 */
int sst_wait_timeout(struct intel_sst_drv *sst_drv_ctx, struct sst_block *block)
{
	int retval = 0;

	/* NOTE:
	Observed that FW processes the alloc msg and replies even
	before the alloc thread has finished execution */
	pr_debug("sst: waiting for condition %x\n",
		       block->condition);
	if (wait_event_timeout(sst_drv_ctx->wait_queue,
				block->condition,
				msecs_to_jiffies(SST_BLOCK_TIMEOUT))) {
		/* event wake */
		pr_debug("sst: Event wake %x\n", block->condition);
		pr_debug("sst: message ret: %d\n", block->ret_code);
		retval = block->ret_code;
	} else {
		block->on = false;
		pr_err("sst: Wait timed-out %x\n", block->condition);
		/* settign firmware state as uninit so that the
		firmware will get redownloaded on next request
		this is because firmare not responding for 5 sec
		is equalant to some unrecoverable error of FW */
		retval = -EBUSY;
	}
	return retval;
}

/*
 * sst_create_large_msg - create a large IPC message
 *
 * @arg: ipc message
 *
 * this function allocates structures to send a large message to the firmware
 */
int sst_create_large_msg(struct ipc_post **arg)
{
	struct ipc_post *msg;

	msg = kzalloc(sizeof(struct ipc_post), GFP_ATOMIC);
	if (!msg) {
		pr_err("kzalloc msg failed\n");
		return -ENOMEM;
	}

	msg->mailbox_data = kzalloc(SST_MAILBOX_SIZE, GFP_ATOMIC);
	if (!msg->mailbox_data) {
		kfree(msg);
		pr_err("kzalloc mailbox_data failed");
		return -ENOMEM;
	}
	*arg = msg;
	return 0;
}

/*
 * sst_create_short_msg - create a short IPC message
 *
 * @arg: ipc message
 *
 * this function allocates structures to send a short message to the firmware
 */
int sst_create_short_msg(struct ipc_post **arg)
{
	struct ipc_post *msg;

	msg = kzalloc(sizeof(*msg), GFP_ATOMIC);
	if (!msg) {
		pr_err("kzalloc msg failed\n");
		return -ENOMEM;
	}
	msg->mailbox_data = NULL;
	*arg = msg;
	return 0;
}

/*
 * sst_clean_stream - clean the stream context
 *
 * @stream: stream structure
 *
 * this function resets the stream contexts
 * should be called in free
 */
void sst_clean_stream(struct stream_info *stream)
{
	struct sst_stream_bufs *bufs = NULL, *_bufs;
	stream->status = STREAM_UN_INIT;
	stream->prev = STREAM_UN_INIT;
	mutex_lock(&stream->lock);
	list_for_each_entry_safe(bufs, _bufs, &stream->bufs, node) {
		list_del(&bufs->node);
		kfree(bufs);
	}
	mutex_unlock(&stream->lock);

	if (stream->ops != STREAM_OPS_PLAYBACK_DRM)
		kfree(stream->decode_ibuf);
}

/*
 * sst_wake_up_alloc_block - wake up waiting block
 *
 * @sst_drv_ctx: Driver context
 * @sst_id: stream id
 * @status: status of wakeup
 * @data: data pointer of wakeup
 *
 * This function wakes up a sleeping block event based on the response
 */
void sst_wake_up_alloc_block(struct intel_sst_drv *sst_drv_ctx,
		u8 sst_id, int status, void *data)
{
	int i;

	/* Unblock with retval code */
	for (i = 0; i < MAX_ACTIVE_STREAM; i++) {
		if (sst_id == sst_drv_ctx->alloc_block[i].sst_id) {
			sst_drv_ctx->alloc_block[i].ops_block.condition = true;
			sst_drv_ctx->alloc_block[i].ops_block.ret_code = status;
			sst_drv_ctx->alloc_block[i].ops_block.data = data;
			wake_up(&sst_drv_ctx->wait_queue);
			break;
		}
	}
}

