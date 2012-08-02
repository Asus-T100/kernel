/*
 *  intel_sst_ipc.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corporation
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
 *  This file defines all ipc functions
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

/**
 * sst_send_ipc_msg_nowait - send ipc msg for algorithm parameters
 *		and returns immediately without waiting for reply
 *
 * @msg: post msg pointer
 *
 * This function is called to send ipc msg
 */
static int sst_send_ipc_msg_nowait(struct ipc_post **msg)
{
	unsigned long irq_flags;
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&(*msg)->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	return  0;
}

/*
 * sst_send_runtime_param - send runtime param to SST
 *
 * this function sends the runtime parameter to sst dsp engine
 */
static int sst_send_runtime_param(struct snd_sst_runtime_params *params)
{
	struct ipc_post *msg = NULL;
	int ret_val;

	pr_debug("Enter:%s\n", __func__);
	ret_val = sst_create_large_msg(&msg);
	if (ret_val)
		return ret_val;
	sst_fill_header(&msg->header, IPC_IA_SET_RUNTIME_PARAMS, 1,
							params->str_id);
	msg->header.part.data = sizeof(u32) + sizeof(*params) + params->size;
	memcpy(msg->mailbox_data, &msg->header.full, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), params, sizeof(*params));
	/* driver doesn't need to send address, so overwrite addr with data */
	memcpy(msg->mailbox_data + sizeof(u32) + sizeof(*params)
			- sizeof(params->addr),
			params->addr, params->size);
	return sst_send_ipc_msg_nowait(&msg);
}

/*
 * sst_send_algo_param - send LPE Mixer param to SST
 *
 * this function sends the algo parameter to sst dsp engine
 */
int sst_send_algo_param(struct snd_ppp_params *algo_params)
{
	u32 header_size = 0;
	struct ipc_post *msg = NULL;
	u32 ipc_msg_size = sizeof(u32) + sizeof(*algo_params)
			 - sizeof(algo_params->params) + algo_params->size;
	u32 offset = 0;

	if (ipc_msg_size > SST_MAILBOX_SIZE)
		return -ENOMEM;
	if (sst_create_large_msg(&msg))
		return -ENOMEM;
	sst_fill_header(&msg->header,
			IPC_IA_ALG_PARAMS, 1, algo_params->str_id);
	msg->header.part.data = sizeof(u32) + sizeof(*algo_params)
			 - sizeof(algo_params->params) + algo_params->size;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	offset = sizeof(u32);
	header_size = sizeof(*algo_params) - sizeof(algo_params->params);
	memcpy(msg->mailbox_data + sizeof(u32), algo_params,
		sizeof(*algo_params) - sizeof(algo_params->params));
	offset += header_size;
	memcpy(msg->mailbox_data + offset , algo_params->params,
			algo_params->size);
	return sst_send_ipc_msg_nowait(&msg);
}


void sst_post_message_mrfld(struct work_struct *work)
{
	struct ipc_post *msg;
	union ipc_header_mrfld header;
	unsigned long irq_flags;

	pr_debug("sst: post message called\n");
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	/* check list */
	if (list_empty(&sst_drv_ctx->ipc_dispatch_list)) {
		/* queue is empty, nothing to send */
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		pr_debug("Empty msg queue... NO Action\n");
		return;
	}

	/* check busy bit */
	header.f = sst_shim_read64(sst_drv_ctx->shim, SST_IPCX);
	if (header.p.header_high.part.busy) {
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		pr_debug("Busy not free... post later\n");
		return;
	}
	/* copy msg from list */
	msg = list_entry(sst_drv_ctx->ipc_dispatch_list.next,
			struct ipc_post, node);
	list_del(&msg->node);
	pr_debug("sst: size: = %x\n", msg->mrfld_header.p.header_low_payload);
	if (msg->mrfld_header.p.header_high.part.large)
		memcpy_toio(sst_drv_ctx->mailbox + SST_MAILBOX_SEND,
		msg->mailbox_data, msg->mrfld_header.p.header_low_payload);
	sst_shim_write64(sst_drv_ctx->shim, SST_IPCX, msg->mrfld_header.f);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	pr_debug("sst: Post message: header = %x\n",
					msg->mrfld_header.p.header_high.full);
	kfree(msg->mailbox_data);
	kfree(msg);
	return;
}

/**
* sst_post_message - Posts message to SST
*
* @work: Pointer to work structure
*
* This function is called by any component in driver which
* wants to send an IPC message. This will post message only if
* busy bit is free
*/
void sst_post_message_mfld(struct work_struct *work)
{
	struct ipc_post *msg;
	union ipc_header header;
	unsigned long irq_flags;

	pr_debug("post message called\n");

	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	/* check list */
	if (list_empty(&sst_drv_ctx->ipc_dispatch_list)) {
		/* queue is empty, nothing to send */
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		pr_debug("Empty msg queue... NO Action\n");
		return;
	}

	/* check busy bit */
	header.full = sst_shim_read(sst_drv_ctx->shim, SST_IPCX);
	if (header.part.busy) {
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		pr_debug("Busy not free... Post later\n");
		return;
	}
	/* copy msg from list */
	msg = list_entry(sst_drv_ctx->ipc_dispatch_list.next,
			struct ipc_post, node);
	list_del(&msg->node);
	pr_debug("size: = %x\n", msg->header.part.data);
	if (msg->header.part.large)
		memcpy_toio(sst_drv_ctx->mailbox + SST_MAILBOX_SEND,
			msg->mailbox_data, msg->header.part.data);

	sst_shim_write(sst_drv_ctx->shim, SST_IPCX, msg->header.full);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	pr_debug("Posted message: header = %x\n", msg->header.full);

	kfree(msg->mailbox_data);
	kfree(msg);
	return;
}


int sst_sync_post_message_mrfld(struct ipc_post *msg)
{
	union ipc_header_mrfld header;
	unsigned int loop_count = 0;
	int retval = 0;
	unsigned long irq_flags;

	pr_debug("sst: post message called\n");
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);

	/* check busy bit */
	header.f = sst_shim_read64(sst_drv_ctx->shim, SST_IPCX);
	while (header.p.header_high.part.busy) {
		if (loop_count > 10) {
			pr_err("sst: Busy wait failed, cant send this msg\n");
			retval = -EBUSY;
			goto out;
		}
		udelay(500);
		loop_count++;
		header.f = sst_shim_read64(sst_drv_ctx->shim, SST_IPCX);
	};
	pr_debug("sst: Post message: header = %x\n",
					msg->mrfld_header.p.header_high.full);
	pr_debug("sst: size: = %x\n", msg->mrfld_header.p.header_low_payload);
	if (msg->mrfld_header.p.header_high.part.large)
		memcpy_toio(sst_drv_ctx->mailbox + SST_MAILBOX_SEND,
		msg->mailbox_data, msg->mrfld_header.p.header_low_payload);
	sst_shim_write64(sst_drv_ctx->shim, SST_IPCX, msg->mrfld_header.f);

out:
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	kfree(msg->mailbox_data);
	kfree(msg);
	return retval;
}

/* use this for trigger ops to post syncronous msgs
 */
int sst_sync_post_message_mfld(struct ipc_post *msg)
{
	union ipc_header header;
	unsigned int loop_count = 0;
	int retval = 0;
	unsigned long irq_flags;
	pr_debug("sst: sync post message called\n");
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);

	/* check busy bit */
	header.full = sst_shim_read(sst_drv_ctx->shim, SST_IPCX);
	while (header.part.busy) {
		if (loop_count > 10) {
			pr_err("busy wait failed, cant send this msg\n");
			retval = -EBUSY;
			goto out;
		}
		udelay(500);
		loop_count++;
		header.full = sst_shim_read(sst_drv_ctx->shim, SST_IPCX);
	};
	pr_debug("sst: Post message: header = %x\n", msg->header.full);
	pr_debug("sst: size: = %x\n", msg->header.part.data);
	if (msg->header.part.large)
		memcpy_toio(sst_drv_ctx->mailbox + SST_MAILBOX_SEND,
			msg->mailbox_data, msg->header.part.data);

	sst_shim_write(sst_drv_ctx->shim, SST_IPCX, msg->header.full);

out:
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	kfree(msg->mailbox_data);
	kfree(msg);
	return retval;
}

/*
 * sst_clear_interrupt - clear the SST FW interrupt
 *
 * This function clears the interrupt register after the interrupt
 * bottom half is complete allowing next interrupt to arrive
 */
void intel_sst_clear_intr_mfld(void)
{
	union interrupt_reg isr;
	union interrupt_reg imr;
	union ipc_header clear_ipc;
	unsigned long irq_flags;

	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	imr.full = sst_shim_read(sst_drv_ctx->shim, SST_IMRX);
	isr.full = sst_shim_read(sst_drv_ctx->shim, SST_ISRX);
	/*  write 1 to clear  */;
	isr.part.busy_interrupt = 1;
	sst_shim_write(sst_drv_ctx->shim, SST_ISRX, isr.full);
	/* Set IA done bit */
	clear_ipc.full = sst_shim_read(sst_drv_ctx->shim, SST_IPCD);
	clear_ipc.part.busy = 0;
	clear_ipc.part.done = 1;
	clear_ipc.part.data = IPC_ACK_SUCCESS;
	sst_shim_write(sst_drv_ctx->shim, SST_IPCD, clear_ipc.full);
	/* un mask busy interrupt */
	imr.part.busy_interrupt = 0;
	sst_shim_write(sst_drv_ctx->shim, SST_IMRX, imr.full);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
}


void intel_sst_clear_intr_mrfld(void)
{
	union interrupt_reg_mrfld isr;
	union interrupt_reg_mrfld imr;
	union ipc_header_mrfld clear_ipc;
	unsigned long irq_flags;

	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	imr.full = sst_shim_read64(sst_drv_ctx->shim, SST_IMRX);
	isr.full = sst_shim_read64(sst_drv_ctx->shim, SST_ISRX);

	/*  write 1 to clear  */
	isr.part.busy_interrupt = 1;
	sst_shim_write64(sst_drv_ctx->shim, SST_ISRX, isr.full);

	/* Set IA done bit */
	clear_ipc.f = sst_shim_read64(sst_drv_ctx->shim, SST_IPCD);

	clear_ipc.p.header_high.part.busy = 0;
	clear_ipc.p.header_high.part.done = 1;
	clear_ipc.p.header_high.part.result = IPC_ACK_SUCCESS;
	sst_shim_write64(sst_drv_ctx->shim, SST_IPCD, clear_ipc.f);
	/* un mask busy interrupt */
	imr.part.busy_interrupt = 0;
	sst_shim_write64(sst_drv_ctx->shim, SST_IMRX, imr.full);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
}


/*
 * process_fw_init - process the FW init msg
 *
 * @msg: IPC message from FW
 *
 * This function processes the FW init msg from FW
 * marks FW state and prints debug info of loaded FW
 */
static int process_fw_init(struct sst_ipc_msg_wq *msg)
{
	struct ipc_header_fw_init *init =
		(struct ipc_header_fw_init *)msg->mailbox;
	int retval = 0;

	pr_debug("*** FW Init msg came***\n");
	if (init->result) {
		sst_set_fw_state_locked(sst_drv_ctx, SST_ERROR);
		pr_debug("FW Init failed, Error %x\n", init->result);
		pr_err("FW Init failed, Error %x\n", init->result);
		retval = -init->result;
		return retval;
	}
	/* If there any runtime parameter to set, send it */
	if (sst_drv_ctx->runtime_param.param.addr)
		sst_send_runtime_param(&(sst_drv_ctx->runtime_param.param));
	mutex_lock(&sst_drv_ctx->sst_lock);
	mutex_unlock(&sst_drv_ctx->sst_lock);
	if (sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID) {
		pr_debug("FW Version %02x.%02x.%02x\n", init->fw_version.major,
				init->fw_version.minor, init->fw_version.build);
		pr_debug("Build Type %x\n", init->fw_version.type);
		pr_debug(" Build date %s Time %s\n",
				init->build_info.date, init->build_info.time);
	}
	sst_wake_up_alloc_block(sst_drv_ctx, FW_DWNL_ID, retval, NULL);
	return retval;
}
/**
* sst_process_message_mfld - Processes message from SST
*
* @work:	Pointer to work structure
*
* This function is scheduled by ISR
* It take a msg from process_queue and does action based on msg
*/
void sst_process_message_mfld(struct work_struct *work)
{
	struct sst_ipc_msg_wq *msg, *tmp;
	int str_id;

	/* copy the message before enabling interrupts */
	tmp = container_of(work, struct sst_ipc_msg_wq, wq);
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (NULL == msg) {
		pr_err("%s:memory alloc failed. msg didn't processed\n", __func__);
		return;
	}
	memcpy(msg, tmp, sizeof(*msg));
	str_id = msg->header.part.str_id;

	intel_sst_clear_intr_mfld();
	pr_debug("IPC process for %x\n", msg->header.full);
	/* based on msg in list call respective handler */
	switch (msg->header.part.msg_id) {
	case IPC_SST_BUF_UNDER_RUN:
	case IPC_SST_BUF_OVER_RUN:
		if (sst_validate_strid(str_id)) {
			pr_err("stream id %d invalid\n", str_id);
			break;
		}
		pr_err("Buffer under/overrun for %d\n",
				msg->header.part.str_id);
		pr_err("Got Underrun & not to send data...ignore\n");
		break;

	case IPC_SST_FRAGMENT_ELPASED: {
		pr_debug("IPC_SST_FRAGMENT_ELPASED for %d", str_id);
		sst_cdev_fragment_elapsed(str_id);
		break;
	}

	case IPC_IA_PRINT_STRING:
		pr_debug("been asked to print something by fw\n");
		/* TBD */
		break;

	case IPC_IA_FW_INIT_CMPLT: {
		/* send next data to FW */
		process_fw_init(msg);
		break;
	}

	case IPC_SST_STREAM_PROCESS_FATAL_ERR:
		if (sst_validate_strid(str_id)) {
			pr_err("stream id %d invalid\n", str_id);
			break;
		}
		pr_err("codec fatal error %x stream %d...\n",
				msg->header.full, msg->header.part.str_id);
		pr_err("Dropping the stream\n");
		sst_drop_stream(msg->header.part.str_id);
		break;
	default:
		/* Illegal case */
		pr_err("Unhandled msg %x header %x\n",
		msg->header.part.msg_id, msg->header.full);
	}
	kfree(msg);
	return;
}

/**
* sst_process_message - Processes message from SST
*
* @work:	Pointer to work structure
*
* This function is scheduled by ISR
* It take a msg from process_queue and does action based on msg
*/

void sst_process_message_mrfld(struct work_struct *work)
{
	struct sst_ipc_msg_wq *msg, *tmp;
	int str_id;

	/* copy the message before enabling interrupts */
	tmp = container_of(work, struct sst_ipc_msg_wq, wq);
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (NULL == msg) {
		pr_err("%s:memory alloc failed. msg didn't processed\n", __func__);
		return;
	}
	memcpy(msg, tmp, sizeof(*msg));
	str_id = msg->mrfld_header.p.header_high.part.str_id;
	intel_sst_clear_intr_mrfld();

	pr_debug("ProcesMsg:%d\n", msg->mrfld_header.p.header_high.part.msg_id);
	switch (msg->mrfld_header.p.header_high.part.msg_id) {
	case IPC_SST_BUF_UNDER_RUN:
	case IPC_SST_BUF_OVER_RUN:
		if (sst_validate_strid(str_id)) {
			pr_err("stream id %d invalid\n", str_id);
			break;
		}
		pr_err("Buffer under/overrun for %d\n",
				msg->mrfld_header.p.header_high.part.str_id);
		pr_err("Got Underrun & not to send data...ignore\n");
		break;
	case IPC_IA_FW_INIT_CMPLT: {
		/* send next data to FW */
		process_fw_init(msg);
		break;
	}
	default:
		/* Illegal case */
		pr_err("Unhandled msg %x header %x\n",
		msg->header.part.msg_id, msg->header.full);
	}
	kfree(msg);
	return;
}

void sst_process_reply_mrfld(struct work_struct *work)
{
	struct sst_ipc_msg_wq *msg, *tmp;
	int str_id;
	struct stream_info *str_info;

	/* copy the message before enabling interrupts */
	tmp = container_of(work, struct sst_ipc_msg_wq, wq);
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (NULL == msg) {
		pr_err("%s:memory alloc failed. msg didn't processed\n", __func__);
		return;
	}
	memcpy(msg, tmp, sizeof(*msg));
	str_id = msg->mrfld_header.p.header_high.part.str_id;
	intel_sst_clear_intr_mrfld();

	pr_debug("Msg-reply:%d\n", msg->mrfld_header.p.header_high.part.msg_id);
	switch (msg->mrfld_header.p.header_high.part.msg_id) {
	case IPC_IA_ALLOC_STREAM: {
		/* map to stream, call play */
		if (msg->mrfld_header.p.header_high.part.result)
			pr_err("error alloc stream\n");
		sst_alloc_stream_response_mrfld(str_id);
		break;
	}
	case IPC_IA_START_STREAM:
		pr_debug("reply for START STREAM\n");
		break;
	case IPC_IA_FREE_STREAM:
		str_info = &sst_drv_ctx->streams[str_id];
		if (!msg->mrfld_header.p.header_high.part.result) {
			pr_debug("Stream %d freed\n", str_id);
		} else {
			pr_err("Free for %d ret error %x\n",
			str_id, msg->mrfld_header.p.header_high.part.result);
		}
		if (str_info->ctrl_blk.on == true) {
			str_info->ctrl_blk.on = false;
			str_info->ctrl_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;
	case IPC_IA_DROP_STREAM:
		pr_debug("Drop received...\n");
		if (sst_validate_strid(str_id)) {
			pr_err("str id %d invalid\n", str_id);
			break;
		}

	default:
		/* Illegal case */
		pr_err("process reply:default\n");
	}
	kfree(msg);
	return;
}


/**
* sst_process_reply - Processes reply message from SST
*
* @work:	Pointer to work structure
*
* This function is scheduled by ISR
* It take a reply msg from response_queue and
* does action based on msg
*/
void sst_process_reply_mfld(struct work_struct *work)
{
	struct sst_ipc_msg_wq *msg, *tmp;
	struct stream_info *str_info;
	int str_id;

	/* copy the message before enabling interrupts */
	tmp = container_of(work, struct sst_ipc_msg_wq, wq);
	msg = kzalloc(sizeof(*msg), GFP_KERNEL);
	if (NULL == msg) {
		pr_err("%s:memory alloc failed. reply didn't processed\n", __func__);
		return;
	}
	memcpy(msg, tmp, sizeof(*msg));
	str_id = msg->header.part.str_id;

	intel_sst_clear_intr_mfld();

	pr_debug("sst: IPC process reply for %x\n", msg->header.full);

	switch (msg->header.part.msg_id) {
	case IPC_IA_ALG_PARAMS: {
		pr_debug("IPC_ALG_PARAMS response %x\n", msg->header.full);
		pr_debug("data value %x\n", msg->header.part.data);
		pr_debug("large value %x\n", msg->header.part.large);

		if (!msg->header.part.large) {
			if (!msg->header.part.data) {
				pr_debug("alg set success\n");
				sst_drv_ctx->ppp_params_blk.ret_code = 0;
			} else {
				pr_debug("alg set failed\n");
				sst_drv_ctx->ppp_params_blk.ret_code =
							-msg->header.part.data;
			}

		} else if (msg->header.part.data) {
			struct snd_ppp_params *mailbox_params, *get_params;
			char *params;

			pr_debug("alg get success\n");
			mailbox_params = (struct snd_ppp_params *)msg->mailbox;
			get_params = kzalloc(sizeof(*get_params), GFP_KERNEL);
			if (!get_params) {
				pr_err("sst: mem alloc failed\n");
				break;
			}
			memcpy_fromio(get_params, mailbox_params,
							sizeof(*get_params));
			get_params->params = kzalloc(mailbox_params->size,
							GFP_KERNEL);
			if (!get_params->params) {
				pr_err("sst: mem alloc failed\n");
				kfree(get_params);
				break;
			}
			params = msg->mailbox;
			params = params + sizeof(*mailbox_params) - sizeof(u32);
			memcpy_fromio(get_params->params, params,
							get_params->size);
			sst_drv_ctx->ppp_params_blk.ret_code = 0;
			sst_drv_ctx->ppp_params_blk.data = get_params;
		}

		if (sst_drv_ctx->ppp_params_blk.on == true) {
			sst_drv_ctx->ppp_params_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;
	}

	case IPC_IA_TUNING_PARAMS:
	case IPC_IA_SET_RUNTIME_PARAMS: {
		pr_debug("IPC_TUNING_PARAMS resp: %x\n", msg->header.full);
		pr_debug("data value %x\n", msg->header.part.data);
		if (msg->header.part.large) {
			pr_debug("alg set failed\n");
			sst_drv_ctx->ppp_params_blk.ret_code =
							-msg->header.part.data;
		} else {
			pr_debug("alg set success\n");
			sst_drv_ctx->ppp_params_blk.ret_code = 0;
		}
		if (sst_drv_ctx->ppp_params_blk.on == true) {
			sst_drv_ctx->ppp_params_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;
	}

	case IPC_IA_GET_FW_INFO: {
		struct snd_sst_fw_info *fw_info =
			(struct snd_sst_fw_info *)msg->mailbox;
		if (msg->header.part.large) {
			int major = fw_info->fw_version.major;
			int minor = fw_info->fw_version.minor;
			int build = fw_info->fw_version.build;
			pr_debug("Msg succeeded %x\n",
				       msg->header.part.msg_id);
			pr_debug("INFO: ***FW*** = %02d.%02d.%02d\n",
					major, minor, build);
			memcpy_fromio(sst_drv_ctx->fw_info_blk.data,
				((struct snd_sst_fw_info *)(msg->mailbox)),
				sizeof(struct snd_sst_fw_info));
			sst_drv_ctx->fw_info_blk.ret_code = 0;
		} else {
			pr_err(" Msg %x reply error %x\n",
			msg->header.part.msg_id, msg->header.part.data);
			sst_drv_ctx->fw_info_blk.ret_code =
					-msg->header.part.data;
		}
		if (sst_drv_ctx->fw_info_blk.on == true) {
			pr_debug("Memcopy succeeded\n");
			sst_drv_ctx->fw_info_blk.on = false;
			sst_drv_ctx->fw_info_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;
	}

	case IPC_IA_GET_STREAM_PARAMS:
		str_info = get_stream_info(str_id);
		if (!str_info) {
			pr_err("stream id %d invalid\n", str_id);
			break;
		}
		if (msg->header.part.large) {
			pr_debug("Get stream large success\n");
			memcpy_fromio(str_info->ctrl_blk.data,
				((void *)(msg->mailbox)),
				sizeof(struct snd_sst_fw_get_stream_params));
			str_info->ctrl_blk.ret_code = 0;
		} else {
			pr_err("Msg %x reply error %x\n",
				msg->header.part.msg_id, msg->header.part.data);
			str_info->ctrl_blk.ret_code = -msg->header.part.data;
		}
		if (str_info->ctrl_blk.on == true) {
			str_info->ctrl_blk.on = false;
			str_info->ctrl_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;
	case IPC_IA_DRAIN_STREAM:
		str_info = get_stream_info(str_id);
		if (!str_info) {
			pr_err("stream id %d invalid\n", str_id);
			break;
		}
		if (!msg->header.part.data) {
			pr_debug("Msg succeeded %x\n",
					msg->header.part.msg_id);
			str_info->ctrl_blk.ret_code = 0;

		} else {
			pr_err(" Msg %x reply error %x\n",
				msg->header.part.msg_id, msg->header.part.data);
			str_info->ctrl_blk.ret_code = -msg->header.part.data;

		}
		str_info = &sst_drv_ctx->streams[str_id];
		if (str_info->data_blk.on == true) {
			str_info->data_blk.on = false;
			str_info->data_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;

	case IPC_IA_DROP_STREAM:
		str_info = get_stream_info(str_id);
		if (!str_info) {
			pr_err("str id %d invalid\n", str_id);
			break;
		}
		if (msg->header.part.large) {
			struct snd_sst_drop_response *drop_resp =
				(struct snd_sst_drop_response *)msg->mailbox;

			pr_debug("Drop ret bytes %x\n", drop_resp->bytes);
			if (!drop_resp->result)
				pr_debug("drop success for %d\n", str_id);
			else
				pr_err("drop for %d failed with err %d\n",
						str_id, drop_resp->result);
		} else {
			pr_err("fw sent small IPC for drop response!!\n");
		}

		break;
	case IPC_IA_PAUSE_STREAM:
	case IPC_IA_RESUME_STREAM:
	case IPC_IA_SET_STREAM_PARAMS:
		str_info = get_stream_info(str_id);
		if (!str_info) {
			pr_err(" stream id %d invalid\n", str_id);
			break;
		}
		if (!msg->header.part.data) {
			pr_debug("Msg succeeded %x\n",
					msg->header.part.msg_id);
			str_info->ctrl_blk.ret_code = 0;
		} else {
			pr_err(" Msg %x reply error %x\n",
					msg->header.part.msg_id,
					msg->header.part.data);
			str_info->ctrl_blk.ret_code = -msg->header.part.data;
		}

		if (str_info->ctrl_blk.on == true) {
			str_info->ctrl_blk.on = false;
			str_info->ctrl_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;

	case IPC_IA_FREE_STREAM:
		str_info = &sst_drv_ctx->streams[str_id];
		if (!msg->header.part.data) {
			pr_debug("Stream %d freed\n", str_id);
		} else {
			pr_err("Free for %d ret error %x\n",
				       str_id, msg->header.part.data);
		}
		if (str_info->ctrl_blk.on == true) {
			str_info->ctrl_blk.on = false;
			str_info->ctrl_blk.condition = true;
			wake_up(&sst_drv_ctx->wait_queue);
		}
		break;
	case IPC_IA_ALLOC_STREAM: {
		/* map to stream, call play */
		struct snd_sst_alloc_response *resp =
				(struct snd_sst_alloc_response *)msg->mailbox;
		if (resp->str_type.result)
			pr_err("error alloc stream = %x\n",
				       resp->str_type.result);
		sst_alloc_stream_response(str_id, resp);
		break;
	}

	case IPC_IA_PREP_LIB_DNLD: {
		struct snd_sst_str_type *str_type =
			(struct snd_sst_str_type *)msg->mailbox;
		pr_debug("Prep Lib download %x\n",
				msg->header.part.msg_id);
		if (str_type->result)
			pr_err("Prep lib download %x\n", str_type->result);
		else
			pr_debug("Can download codec now...\n");
		sst_wake_up_alloc_block(sst_drv_ctx, str_id,
				str_type->result, NULL);
		break;
	}

	case IPC_IA_LIB_DNLD_CMPLT: {
		struct snd_sst_lib_download_info *resp =
			(struct snd_sst_lib_download_info *)msg->mailbox;
		int retval = resp->result;

		pr_debug("Lib downloaded %x\n", msg->header.part.msg_id);
		if (resp->result) {
			pr_err("err in lib dload %x\n", resp->result);
		} else {
			pr_debug("Codec download complete...\n");
			pr_debug("codec Type %d Ver %d Built %s: %s\n",
				resp->dload_lib.lib_info.lib_type,
				resp->dload_lib.lib_info.lib_version,
				resp->dload_lib.lib_info.b_date,
				resp->dload_lib.lib_info.b_time);
		}
		sst_wake_up_alloc_block(sst_drv_ctx, str_id,
						retval, NULL);
		break;
	}
	case IPC_IA_SET_FW_CTXT: {
		int retval = msg->header.part.data;

		if (!msg->header.part.data) {
			pr_debug("Msg IPC_IA_SET_FW_CTXT succedded %x\n",
				       msg->header.part.msg_id);
		} else {
			pr_err("Msg %x reply error %x\n",
			msg->header.part.msg_id, msg->header.part.data);
			break;
		}
		sst_wake_up_alloc_block(sst_drv_ctx, FW_DWNL_ID, retval, NULL);
		break;
	}
	case IPC_IA_GET_FW_VERSION: {
		struct ipc_header_fw_init *version =
				(struct ipc_header_fw_init *)msg->mailbox;
		int major = version->fw_version.major;
		int minor = version->fw_version.minor;
		int build = version->fw_version.build;
		dev_info(&sst_drv_ctx->pci->dev,
			"INFO: ***LOADED SST FW VERSION*** = %02d.%02d.%02d\n",
		major, minor, build);
		break;
	}
	case IPC_IA_GET_FW_BUILD_INF: {
		struct sst_fw_build_info *build =
			(struct sst_fw_build_info *)msg->mailbox;
		pr_debug("Build date:%sTime:%s", build->date, build->time);
		break;
	}
	case IPC_IA_START_STREAM:
		pr_debug("reply for START STREAM %x\n", msg->header.full);
		break;

	case IPC_IA_GET_FW_CTXT:
		pr_debug("reply for get fw ctxt  %x\n", msg->header.full);
		if (msg->header.part.data)
			sst_drv_ctx->fw_cntx_size = 0;
		else
			sst_drv_ctx->fw_cntx_size = *sst_drv_ctx->fw_cntx;
		pr_debug("fw copied data %x\n", sst_drv_ctx->fw_cntx_size);
		sst_wake_up_alloc_block(
			sst_drv_ctx, str_id, msg->header.part.data, NULL);
		break;
	default:
		/* Illegal case */
		pr_err("process reply:default = %x\n", msg->header.full);
	}
	kfree(msg);
	return;
}
