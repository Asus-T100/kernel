/*
 *  intel_sst_stream.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corp
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
 *  This file contains the stream operations of SST driver
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/pci.h>
#include <linux/firmware.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

/*
 * sst_check_device_type - Check the medfield device type
 *
 * @device: Device to be checked
 * @num_ch: Number of channels queried
 * @pcm_slot: slot to be enabled for this device
 *
 * This checks the deivce against the map and calculates pcm_slot value
 */
static int sst_check_device_type(u32 device, u32 num_chan, u32 *pcm_slot)
{
	/* device id range starts from 1 */
	if (device <= 0 || device > MAX_NUM_STREAMS_MFLD) {
		pr_debug("sst: device type invalid %d\n", device);
		return -EINVAL;
	}
	if (sst_drv_ctx->streams[device].status == STREAM_UN_INIT) {
		if (device == SND_SST_DEVICE_VIBRA && num_chan == 1)
			*pcm_slot = 0x10;
		else if (device == SND_SST_DEVICE_HAPTIC && num_chan == 1)
			*pcm_slot = 0x20;
		else if (device == SND_SST_DEVICE_IHF && num_chan == 1)
			*pcm_slot = 0x04;
		else if (device == SND_SST_DEVICE_IHF && num_chan == 2)
			*pcm_slot = 0x0C;
		else if (device == SND_SST_DEVICE_HEADSET && num_chan == 1)
			*pcm_slot = 0x01;
		else if (device == SND_SST_DEVICE_HEADSET && num_chan == 2)
			*pcm_slot = 0x03;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 1)
			*pcm_slot = 0x01;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 2)
			*pcm_slot = 0x03;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 3)
			*pcm_slot = 0x07;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan == 4)
			*pcm_slot = 0x0F;
		else if (device == SND_SST_DEVICE_CAPTURE && num_chan > 4)
			*pcm_slot = 0x1F;
		else if (device == SND_SST_DEVICE_COMPRESS &&
				(num_chan == 2 || num_chan == 1))
			*pcm_slot = sst_drv_ctx->compressed_slot;
		else {
			pr_debug("No condition satisfied.. ret err\n");
			return -EINVAL;
		}
	} else {
		pr_debug("this stream state is not uni-init, is %d\n",
					sst_drv_ctx->streams[device].status);
		return -EBADRQC;
	}
	pr_debug("returning slot %x\n", *pcm_slot);
	return 0;
}

/**
 * sst_alloc_stream - Send msg for a new stream ID
 *
 * @params:	stream params
 * @stream_ops:	operation of stream PB/capture
 * @codec:	codec for stream
 * @device:	device stream to be allocated for
 *
 * This function is called by any function which wants to start
 * a new stream. This also check if a stream exists which is idle
 * it initializes idle stream id to this request
 */
int sst_alloc_stream_ctp(char *params, struct sst_block *block)
{
	struct ipc_post *msg = NULL;
	struct snd_sst_alloc_params alloc_param;
	unsigned int pcm_slot = 0x03, num_ch;
	int str_id;
	struct snd_sst_params *str_params;
	struct snd_sst_stream_params *sparams;
	struct snd_sst_alloc_params_ext *aparams;
	struct stream_info *str_info;
	unsigned int stream_ops, device;
	unsigned long irq_flags;
	u8 codec;

	pr_debug("In %s\n", __func__);

	BUG_ON(!params);
	str_params = (struct snd_sst_params *)params;
	stream_ops = str_params->ops;
	codec = str_params->codec;
	device = str_params->device_type;
	sparams = &str_params->sparams;
	aparams = &str_params->aparams;
	num_ch = sst_get_num_channel(str_params);

	pr_debug("period_size = %d\n", aparams->frag_size);
	pr_debug("ring_buf_addr = 0x%x\n", aparams->ring_buf_info[0].addr);
	pr_debug("ring_buf_size = %d\n", aparams->ring_buf_info[0].size);
	pr_debug("In alloc device_type=%d\n", str_params->device_type);
	pr_debug("In alloc sg_count =%d\n", aparams->sg_count);

	str_id = str_params->stream_id;
	if (str_id <= 0)
		return -EBUSY;

	/*allocate device type context*/
	sst_init_stream(&sst_drv_ctx->streams[str_id], codec,
			str_id, stream_ops, pcm_slot);
	/* send msg to FW to allocate a stream */
	if (sst_create_ipc_msg(&msg, true))
		return -ENOMEM;

	alloc_param.str_type.codec_type = codec;
	alloc_param.str_type.str_type = str_params->stream_type;
	alloc_param.str_type.operation = stream_ops;
	alloc_param.str_type.protected_str = 0; /* non drm */
	alloc_param.str_type.time_slots = pcm_slot;
	alloc_param.str_type.reserved = 0;
	alloc_param.str_type.result = 0;
	memcpy(&alloc_param.stream_params, sparams,
			sizeof(struct snd_sst_stream_params));
	memcpy(&alloc_param.alloc_params, aparams,
			sizeof(struct snd_sst_alloc_params_ext));
	block->drv_id = str_id;
	block->msg_id = IPC_IA_ALLOC_STREAM;
	sst_fill_header(&msg->header, IPC_IA_ALLOC_STREAM, 1, str_id);
	msg->header.part.data = sizeof(alloc_param) + sizeof(u32);
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), &alloc_param,
			sizeof(alloc_param));
	str_info = &sst_drv_ctx->streams[str_id];
	str_info->num_ch = num_ch;
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	return str_id;
}

int sst_alloc_stream_mrfld(char *params, struct sst_block *block)
{
	struct ipc_post *msg = NULL;
	struct snd_sst_alloc_mrfld alloc_param;
	struct ipc_dsp_hdr dsp_hdr;
	struct snd_sst_params *str_params;
	struct snd_sst_tstamp fw_tstamp;
	unsigned int str_id, pipe_id, pvt_id, task_id;
	u32 len = 0;
	struct stream_info *str_info;
	unsigned long irq_flags;

	pr_debug("In %s\n", __func__);
	BUG_ON(!params);

	str_params = (struct snd_sst_params *)params;
	memset(&alloc_param, 0, sizeof(alloc_param));
	alloc_param.operation = str_params->ops;
	alloc_param.codec_type = str_params->codec;
	alloc_param.sg_count = str_params->aparams.sg_count;
	alloc_param.ring_buf_info[0].addr = str_params->aparams.ring_buf_info[0].addr;
	alloc_param.ring_buf_info[0].size = str_params->aparams.ring_buf_info[0].size;
	alloc_param.frag_size = str_params->aparams.frag_size;

	/* FIXME: check if any channel_map changes required later */
	memcpy(&alloc_param.codec_params, &str_params->sparams,
			sizeof(struct snd_sst_stream_params));

	str_id = str_params->stream_id;
	pipe_id = str_params->device_type;
	task_id = str_params->task;
	sst_drv_ctx->streams[str_id].pipe_id = pipe_id;
	sst_drv_ctx->streams[str_id].task_id = task_id;
	sst_drv_ctx->streams[str_id].num_ch = sst_get_num_channel(str_params);

	pvt_id = sst_assign_pvt_id(sst_drv_ctx);
	alloc_param.ts = (sst_drv_ctx->mailbox_add +
			sst_drv_ctx->tstamp + (str_id * sizeof(fw_tstamp)));
	pr_debug("alloc tstamp location = 0x%p\n", alloc_param.ts);
	pr_debug("assigned pipe id 0x%x to task %d\n", pipe_id, task_id);

	/*allocate device type context*/
	sst_init_stream(&sst_drv_ctx->streams[str_id], alloc_param.codec_type,
			str_id, alloc_param.operation, 0);
	/* send msg to FW to allocate a stream */
	if (sst_create_ipc_msg(&msg, true))
		return -ENOMEM;

	block->drv_id = pvt_id;
	block->msg_id = IPC_CMD;

	sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
			      task_id, 1, pvt_id);
	pr_debug("header:%x\n", msg->mrfld_header.p.header_high);
	msg->mrfld_header.p.header_high.part.res_rqd = 1;

	len = msg->mrfld_header.p.header_low_payload = sizeof(alloc_param) + sizeof(dsp_hdr);
	sst_fill_header_dsp(&dsp_hdr, IPC_IA_ALLOC_STREAM_MRFLD, pipe_id, sizeof(alloc_param));
	memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));
	memcpy(msg->mailbox_data + sizeof(dsp_hdr), &alloc_param,
			sizeof(alloc_param));
	str_info = &sst_drv_ctx->streams[str_id];
	pr_debug("header:%x\n", msg->mrfld_header.p.header_high);
	pr_debug("response rqd: %x", msg->mrfld_header.p.header_high.part.res_rqd);
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	pr_debug("calling post_message\n");
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	return str_id;
}

int sst_alloc_stream_mfld(char *params, struct sst_block *block)
{
	struct ipc_post *msg = NULL;
	struct snd_sst_alloc_params_mfld alloc_param;
	struct snd_sst_params *str_params;
	struct stream_info *str_info;
	unsigned int stream_ops, device;
	struct snd_sst_stream_params_mfld *sparams;
	unsigned int pcm_slot = 0, num_ch, pcm_wd_sz, sfreq;
	int str_id;
	u8 codec;
	u32 rb_size, rb_addr, period_count;
	unsigned long irq_flags;

	pr_debug("In %s\n", __func__);

	BUG_ON(!params);
	str_params = (struct snd_sst_params *)params;
	stream_ops = str_params->ops;
	codec = str_params->codec;
	device = str_params->device_type;
	num_ch = sst_get_num_channel(str_params);
	sfreq = sst_get_sfreq(str_params);
	pcm_wd_sz = sst_get_wdsize(str_params);
	rb_size = str_params->aparams.ring_buf_info[0].size;
	rb_addr = str_params->aparams.ring_buf_info[0].addr;
	period_count = str_params->aparams.frag_size / 4;


	pr_debug("period_size = %d\n", period_count);
	pr_debug("ring_buf_addr = 0x%x\n", rb_addr);
	pr_debug("ring_buf_size = %d\n", rb_size);
	pr_debug("device_type=%d\n", device);
	pr_debug("sfreq =%d\n", sfreq);
	pr_debug("stream_ops%d codec%d device%d\n", stream_ops, codec, device);


	sparams = kzalloc(sizeof(*sparams), GFP_KERNEL);
	if (!sparams) {
		pr_err("Unable to allocate snd_sst_stream_params\n");
		return -ENOMEM;
	}


	sparams->uc.pcm_params.codec = codec;
	sparams->uc.pcm_params.num_chan = num_ch;
	sparams->uc.pcm_params.pcm_wd_sz = pcm_wd_sz;
	sparams->uc.pcm_params.reserved = 0;
	sparams->uc.pcm_params.sfreq = sfreq;
	sparams->uc.pcm_params.ring_buffer_size = rb_size;
	sparams->uc.pcm_params.period_count = period_count;
	sparams->uc.pcm_params.ring_buffer_addr = rb_addr;



	if (sst_check_device_type(device, num_ch, &pcm_slot)) {
		kfree(sparams);
		return -EINVAL;
	}
	mutex_lock(&sst_drv_ctx->stream_lock);
	str_id = device;
	mutex_unlock(&sst_drv_ctx->stream_lock);
	pr_debug("slot %x\n", pcm_slot);

	/*allocate device type context*/
	sst_init_stream(&sst_drv_ctx->streams[str_id], codec,
			str_id, stream_ops, pcm_slot);
	/* send msg to FW to allocate a stream */
	if (sst_create_ipc_msg(&msg, true)) {
		kfree(sparams);
		return -ENOMEM;
	}

	alloc_param.str_type.codec_type = codec;
	alloc_param.str_type.str_type = SST_STREAM_TYPE_MUSIC;
	alloc_param.str_type.operation = stream_ops;
	alloc_param.str_type.protected_str = 0; /* non drm */
	alloc_param.str_type.time_slots = pcm_slot;
	alloc_param.str_type.reserved = 0;
	alloc_param.str_type.result = 0;

	block->drv_id = str_id;
	block->msg_id = IPC_IA_ALLOC_STREAM;

	sst_fill_header(&msg->header, IPC_IA_ALLOC_STREAM, 1, str_id);
	msg->header.part.data = sizeof(alloc_param) + sizeof(u32);
	memcpy(&alloc_param.stream_params, sparams,
			sizeof(*sparams));
	kfree(sparams);

	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), &alloc_param,
			sizeof(alloc_param));
	str_info = &sst_drv_ctx->streams[str_id];
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	pr_debug("SST DBG:alloc stream done\n");
	return str_id;
}

/**
* sst_get_fw_info - Send msg to query for firmware configurations
* @info: out param that holds the firmare configurations
*
* This function is called when the firmware configurations are queiried for
*/
int sst_get_fw_info(struct snd_sst_fw_info *info)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	unsigned long irq_flags;
	struct sst_block *block = NULL;

	pr_debug("sst_get_fw_info called\n");

	retval = sst_create_block_and_ipc_msg(&msg, false, sst_drv_ctx, &block,
			IPC_IA_GET_FW_INFO, 0);
	if (retval)
		return retval;

	sst_fill_header(&msg->header, IPC_IA_GET_FW_INFO, 0, 0);
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	retval = sst_wait_timeout(sst_drv_ctx, block);
	if (retval) {
		pr_err("error in fw_info = %d\n", retval);
		retval = -EIO;
	}
	memcpy(info, block->data, sizeof(*info));
	pr_debug("INFO: ***FW*** = %02d.%02d.%02d\n",
			info->fw_version.major,
			info->fw_version.minor,
			info->fw_version.build);
	sst_free_block(sst_drv_ctx, block);
	return retval;
}


/**
* sst_stream_stream - Send msg for a pausing stream
* @str_id:	 stream ID
*
* This function is called by any function which wants to start
* a stream.
*/
int sst_start_stream(int str_id)
{
	int retval = 0, pvt_id;
	u32 len = 0;
	struct ipc_post *msg = NULL;
	struct ipc_dsp_hdr dsp_hdr;
	struct stream_info *str_info;

	pr_debug("sst_start_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	if (str_info->status != STREAM_RUNNING)
		return -EBADRQC;

	if (sst_create_ipc_msg(&msg, true))
		return -ENOMEM;

	if (!sst_drv_ctx->use_32bit_ops) {
		pr_debug("start mrfld");
		pvt_id = sst_assign_pvt_id(sst_drv_ctx);
		pr_debug("pvt_id = %d, pipe id = %d, task = %d\n",
			 pvt_id, str_info->pipe_id, str_info->task_id);
		sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
				      str_info->task_id, 1, pvt_id);

		len = sizeof(u16) + sizeof(dsp_hdr);
		msg->mrfld_header.p.header_low_payload = len;
		sst_fill_header_dsp(&dsp_hdr, IPC_IA_START_STREAM_MRFLD,
				str_info->pipe_id, sizeof(u16));
		memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));
		memset(msg->mailbox_data + sizeof(dsp_hdr), 0, sizeof(u16));
	} else {
		pr_debug("fill START_STREAM for MFLD/CTP\n");
		sst_fill_header(&msg->header, IPC_IA_START_STREAM, 1, str_id);
		msg->header.part.data =  sizeof(u32) + sizeof(u32);
		memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
		memset(msg->mailbox_data + sizeof(u32), 0, sizeof(u32));
	}
	sst_drv_ctx->ops->sync_post_message(msg);
	return retval;
}

int sst_send_byte_stream_mrfld(void *sbytes)
{
	struct ipc_post *msg = NULL;
	struct snd_sst_bytes *bytes = (struct snd_sst_bytes *) sbytes;
	unsigned long irq_flags;
	u32 length;
	int ret, pvt_id;
	struct sst_block *block;

	pr_debug("%s:\ntype:%d\nipc_msg:%x\nblock:%d\ntask_id:%x\npipe: %d\nlength:%d\n",
		__func__, bytes->type, bytes->ipc_msg,
		bytes->block, bytes->task_id,
		bytes->pipe_id, bytes->len);

	/* need some err check as this is user data, perhpas move this to the
	 * platform driver and pass the struct
	 */
	if (sst_create_ipc_msg(&msg, true))
		return -ENOMEM;

	pvt_id = sst_assign_pvt_id(sst_drv_ctx);
	sst_fill_header_mrfld(&msg->mrfld_header, bytes->ipc_msg, bytes->task_id,
			      1, pvt_id);
	msg->mrfld_header.p.header_high.part.res_rqd = bytes->block;
	length = bytes->len;
	msg->mrfld_header.p.header_low_payload = length;
	pr_debug("length is %d\n", length);
	memcpy(msg->mailbox_data, &bytes->bytes, bytes->len);
	if (bytes->block) {
		block = sst_create_block(sst_drv_ctx, bytes->ipc_msg, pvt_id);
		if (block == NULL) {
			kfree(msg);
			return -ENOMEM;
		}
	}
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node,
			&sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	pr_debug("msg->mrfld_header.p.header_low_payload:%d", msg->mrfld_header.p.header_low_payload);
	if (bytes->block) {
		ret = sst_wait_timeout(sst_drv_ctx, block);
		if (ret) {
			pr_err("%s: fw returned err %d\n", __func__, ret);
			sst_free_block(sst_drv_ctx, block);
			return ret;
		}
	}
	if (bytes->type == SND_SST_BYTES_GET) {
		/* copy the reply and send back
		 * we need to update only sz and payload
		 */
		if (bytes->block) {
			unsigned char *r = block->data;
			pr_debug("read back %d bytes", bytes->len);
			print_bytes(r, bytes->len, 16, 8);

			memcpy(bytes->bytes, r, bytes->len);
		}
	}
	if (bytes->block)
		sst_free_block(sst_drv_ctx, block);
	return 0;
}
int sst_send_byte_stream(void *sbytes)
{
	struct ipc_post *msg = NULL;
	struct snd_sst_bytes *bytes = (struct snd_sst_bytes *) sbytes;
	unsigned long irq_flags;
	u32 length;
	int ret, pvt_id;
	struct sst_block *block = NULL;

	pr_debug("%s:\ntype:%d\nipc_msg:%x\nblock:%d\ntask_id:%x\npipe: %d\nlength:%d\n",
		__func__, bytes->type, bytes->ipc_msg,
		bytes->block, bytes->task_id,
		bytes->pipe_id, bytes->len);

	/* need some err check as this is user data, perhpas move this to the
	 * platform driver and pass the struct
	 */
	if (sst_create_ipc_msg(&msg, true))
		return -ENOMEM;
	/* FIXME: we need pipe id here for str_id */

	pvt_id = sst_assign_pvt_id(sst_drv_ctx);
	sst_fill_header_mrfld_32(&msg->header.full, bytes->task_id,
			bytes->ipc_msg, pvt_id, bytes->block, 1);
	length = bytes->len;
	pr_debug("length is %d\n", length);
	memcpy(msg->mailbox_data, &length, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32),
		&bytes->bytes, bytes->len);

	if (bytes->block) {
		block = sst_create_block(sst_drv_ctx, 0, pvt_id);
		if (block == NULL) {
			kfree(msg);
			return -ENOMEM;
		}
	}
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node,
			&sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	sst_drv_ctx->ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	if (bytes->block) {
		ret = sst_wait_timeout(sst_drv_ctx, block);
		if (ret) {
			pr_err("%s: fw returned err %d\n", __func__, ret);
			sst_free_block(sst_drv_ctx, block);
			return ret;
		}
	}
	if (bytes->type == SND_SST_BYTES_GET) {
		/* copy the reply and send back
		 * we need to update only sz and payload
		 */
		struct snd_sst_bytes *r;
		if (bytes->block) {
			r = block->data;
			bytes->len = r->len;
			memcpy(bytes->bytes, r->bytes, bytes->len);
		}
	}
	if (bytes->block)
		sst_free_block(sst_drv_ctx, block);
	return 0;
}

int sst_send_probe_bytes(struct intel_sst_drv *sst)
{
	struct ipc_post *msg = NULL;
	struct sst_block *block;
	unsigned long irq_flags;
	int ret_val = 0;

	ret_val = sst_create_block_and_ipc_msg(&msg, true, sst,
			&block, IPC_IA_DBG_SET_PROBE_PARAMS, 0);
	if (ret_val) {
		pr_err("Can't allocate block/msg: Probe Byte Stream\n");
		return ret_val;
	}

	sst_fill_header(&msg->header, IPC_IA_DBG_SET_PROBE_PARAMS, 1, 0);

	msg->header.part.data = sizeof(u32) + sst->probe_bytes->len;
	memcpy(msg->mailbox_data, &msg->header.full, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32), sst->probe_bytes->bytes,
				sst->probe_bytes->len);

	spin_lock_irqsave(&sst->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst->ipc_spin_lock, irq_flags);

	sst->ops->post_message(msg);

	ret_val = sst_wait_timeout(sst, block);
	sst_free_block(sst, block);
	if (ret_val)
		pr_err("set probe stream param..timeout!\n");
	return ret_val;
}

/*
 * sst_pause_stream - Send msg for a pausing stream
 * @str_id:	 stream ID
 *
 * This function is called by any function which wants to pause
 * an already running stream.
 */
int sst_pause_stream(int str_id)
{
	int retval = 0, pvt_id, len;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;
	struct sst_block *block;
	struct ipc_dsp_hdr dsp_hdr;

	pr_debug("SST DBG:sst_pause_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;
	if (str_info->status == STREAM_PAUSED)
		return 0;
	if (str_info->status == STREAM_RUNNING ||
		str_info->status == STREAM_INIT) {
		if (str_info->prev == STREAM_UN_INIT)
			return -EBADRQC;
		if (!sst_drv_ctx->use_32bit_ops) {
			pvt_id = sst_assign_pvt_id(sst_drv_ctx);
			retval = sst_create_block_and_ipc_msg(&msg, true,
					sst_drv_ctx, &block, IPC_CMD, pvt_id);
			if (retval)
				return retval;
			sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
					str_info->task_id, 1, pvt_id);
			msg->mrfld_header.p.header_high.part.res_rqd = 1;
			len = sizeof(dsp_hdr);
			msg->mrfld_header.p.header_low_payload = len;
			sst_fill_header_dsp(&dsp_hdr, IPC_IA_PAUSE_STREAM_MRFLD,
						str_info->pipe_id, 0);
			memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));
		} else {
			retval = sst_create_block_and_ipc_msg(&msg, false,
					sst_drv_ctx, &block,
					IPC_IA_PAUSE_STREAM, str_id);
			if (retval)
				return retval;
			sst_fill_header(&msg->header, IPC_IA_PAUSE_STREAM,
								0, str_id);
		}
		spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		list_add_tail(&msg->node,
				&sst_drv_ctx->ipc_dispatch_list);

		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
		retval = sst_wait_timeout(sst_drv_ctx, block);
		sst_free_block(sst_drv_ctx, block);
		if (retval == 0) {
			str_info->prev = str_info->status;
			str_info->status = STREAM_PAUSED;
		} else if (retval == SST_ERR_INVALID_STREAM_ID) {
			retval = -EINVAL;
			mutex_lock(&sst_drv_ctx->stream_lock);
			sst_clean_stream(str_info);
			mutex_unlock(&sst_drv_ctx->stream_lock);
		}
	} else {
		retval = -EBADRQC;
		pr_debug("SST DBG:BADRQC for stream\n ");
	}

	return retval;
}

/**
 * sst_resume_stream - Send msg for resuming stream
 * @str_id:		stream ID
 *
 * This function is called by any function which wants to resume
 * an already paused stream.
 */
int sst_resume_stream(int str_id)
{
	int retval = 0;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;
	struct sst_block *block = NULL;
	int pvt_id, len;
	struct ipc_dsp_hdr dsp_hdr;

	pr_debug("SST DBG:sst_resume_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;
	if (str_info->status == STREAM_RUNNING)
			return 0;
	if (str_info->status == STREAM_PAUSED) {
		if (!sst_drv_ctx->use_32bit_ops) {
			pvt_id = sst_assign_pvt_id(sst_drv_ctx);
			retval = sst_create_block_and_ipc_msg(&msg, true,
					sst_drv_ctx, &block, IPC_CMD, pvt_id);
			if (retval)
				return retval;
			sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
					str_info->task_id, 1, pvt_id);
			msg->mrfld_header.p.header_high.part.res_rqd = 1;
			len = sizeof(dsp_hdr);
			msg->mrfld_header.p.header_low_payload = len;
			sst_fill_header_dsp(&dsp_hdr,
						IPC_IA_RESUME_STREAM_MRFLD,
						str_info->pipe_id, 0);
			memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));
		} else {
			retval = sst_create_block_and_ipc_msg(&msg, false,
					sst_drv_ctx, &block,
					IPC_IA_RESUME_STREAM, str_id);
			if (retval)
				return retval;
			sst_fill_header(&msg->header, IPC_IA_RESUME_STREAM,
								0, str_id);
		}
		spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		list_add_tail(&msg->node,
				&sst_drv_ctx->ipc_dispatch_list);
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
		retval = sst_wait_timeout(sst_drv_ctx, block);
		sst_free_block(sst_drv_ctx, block);
		if (!retval) {
			if (str_info->prev == STREAM_RUNNING)
				str_info->status = STREAM_RUNNING;
			else
				str_info->status = STREAM_INIT;
			str_info->prev = STREAM_PAUSED;
		} else if (retval == -SST_ERR_INVALID_STREAM_ID) {
			retval = -EINVAL;
			mutex_lock(&sst_drv_ctx->stream_lock);
			sst_clean_stream(str_info);
			mutex_unlock(&sst_drv_ctx->stream_lock);
		}
	} else {
		retval = -EBADRQC;
		pr_err("SST ERR: BADQRC for stream\n");
	}

	return retval;
}


/**
 * sst_drop_stream - Send msg for stopping stream
 * @str_id:		stream ID
 *
 * This function is called by any function which wants to stop
 * a stream.
 */
int sst_drop_stream(int str_id)
{
	int retval = 0, pvt_id;
	struct stream_info *str_info;
	struct ipc_post *msg = NULL;
	struct ipc_dsp_hdr dsp_hdr;
	u32 len = 0;

	pr_debug("SST DBG:sst_drop_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;

	if (str_info->status != STREAM_UN_INIT) {

		if ((sst_drv_ctx->pci_id != SST_MRFLD_PCI_ID) ||
				(sst_drv_ctx->use_32bit_ops == true)) {
			str_info->prev = STREAM_UN_INIT;
			str_info->status = STREAM_INIT;
			str_info->cumm_bytes = 0;
			sst_send_sync_msg(IPC_IA_DROP_STREAM, str_id);
		} else {
			if (sst_create_ipc_msg(&msg, true))
				return -ENOMEM;
			pvt_id = sst_assign_pvt_id(sst_drv_ctx);
			sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
					      str_info->task_id, 1, pvt_id);

			msg->mrfld_header.p.header_low_payload = sizeof(dsp_hdr);
			sst_fill_header_dsp(&dsp_hdr, IPC_IA_DROP_STREAM_MRFLD,
					str_info->pipe_id, 0);
			memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));
			sst_drv_ctx->ops->sync_post_message(msg);
		}
	} else {
		retval = -EBADRQC;
		pr_debug("BADQRC for stream, state %x\n", str_info->status);
	}
	return retval;
}

/**
* sst_drain_stream - Send msg for draining stream
* @str_id:		stream ID
*
* This function is called by any function which wants to drain
* a stream.
*/
int sst_drain_stream(int str_id, bool partial_drain)
{
	int retval = 0, pvt_id, len;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;
	struct sst_block *block = NULL;
	struct ipc_dsp_hdr dsp_hdr;

	pr_debug("SST DBG:sst_drain_stream for %d\n", str_id);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;
	if (str_info->status != STREAM_RUNNING &&
		str_info->status != STREAM_INIT &&
		str_info->status != STREAM_PAUSED) {
			pr_err("SST ERR: BADQRC for stream = %d\n",
				       str_info->status);
			return -EBADRQC;
	}

	if (!sst_drv_ctx->use_32bit_ops) {
		pvt_id = sst_assign_pvt_id(sst_drv_ctx);
		retval = sst_create_block_and_ipc_msg(&msg, true,
				sst_drv_ctx, &block, IPC_CMD, pvt_id);
		if (retval)
			return retval;
		sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
				str_info->task_id, 1, pvt_id);
		pr_debug("header:%x\n",
			(unsigned int)msg->mrfld_header.p.header_high.full);
		msg->mrfld_header.p.header_high.part.res_rqd = 1;

		len = sizeof(u8) + sizeof(dsp_hdr);
		msg->mrfld_header.p.header_low_payload = len;
		sst_fill_header_dsp(&dsp_hdr, IPC_IA_DRAIN_STREAM_MRFLD,
					str_info->pipe_id, sizeof(u8));
		memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));
		memcpy(msg->mailbox_data + sizeof(dsp_hdr),
				&partial_drain, sizeof(u8));
	} else {
		retval = sst_create_block_and_ipc_msg(&msg, false,
				sst_drv_ctx, &block,
				IPC_IA_DRAIN_STREAM, str_id);
		if (retval)
			return retval;
		sst_fill_header(&msg->header, IPC_IA_DRAIN_STREAM, 0, str_id);
		msg->header.part.data = partial_drain;
	}
	spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
	ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
	retval = sst_wait_interruptible(sst_drv_ctx, block);
	sst_free_block(sst_drv_ctx, block);
	return retval;
}

/**
 * sst_free_stream - Frees a stream
 * @str_id:		stream ID
 *
 * This function is called by any function which wants to free
 * a stream.
 */
int sst_free_stream(int str_id)
{
	int retval = 0;
	unsigned int pvt_id;
	struct ipc_post *msg = NULL;
	struct stream_info *str_info;
	struct intel_sst_ops *ops;
	unsigned long irq_flags;
	struct ipc_dsp_hdr dsp_hdr;
	struct sst_block *block;

	pr_debug("SST DBG:sst_free_stream for %d\n", str_id);

	mutex_lock(&sst_drv_ctx->sst_lock);
	if (sst_drv_ctx->sst_state == SST_UN_INIT) {
		mutex_unlock(&sst_drv_ctx->sst_lock);
		return -ENODEV;
	}
	mutex_unlock(&sst_drv_ctx->sst_lock);
	str_info = get_stream_info(str_id);
	if (!str_info)
		return -EINVAL;
	ops = sst_drv_ctx->ops;

	mutex_lock(&str_info->lock);
	if (str_info->status != STREAM_UN_INIT) {
		str_info->prev =  str_info->status;
		str_info->status = STREAM_UN_INIT;
		mutex_unlock(&str_info->lock);

		if (!sst_drv_ctx->use_32bit_ops) {
			pvt_id = sst_assign_pvt_id(sst_drv_ctx);
			retval = sst_create_block_and_ipc_msg(&msg, true,
					sst_drv_ctx, &block, IPC_CMD, pvt_id);
			if (retval)
				return retval;

			sst_fill_header_mrfld(&msg->mrfld_header, IPC_CMD,
					      str_info->task_id, 1, pvt_id);
			msg->mrfld_header.p.header_low_payload =
							sizeof(dsp_hdr);
			sst_fill_header_dsp(&dsp_hdr, IPC_IA_FREE_STREAM_MRFLD,
						str_info->pipe_id,  0);
			memcpy(msg->mailbox_data, &dsp_hdr, sizeof(dsp_hdr));
		} else {
			retval = sst_create_block_and_ipc_msg(&msg, false,
						sst_drv_ctx, &block,
						IPC_IA_FREE_STREAM, str_id);
			if (retval)
				return retval;
			sst_fill_header(&msg->header, IPC_IA_FREE_STREAM,
								 0, str_id);
		}
		spin_lock_irqsave(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
		spin_unlock_irqrestore(&sst_drv_ctx->ipc_spin_lock, irq_flags);
		if (!sst_drv_ctx->use_32bit_ops) {
			/*FIXME: do we need to wake up drain stream here,
			 * how to get the pvt_id and msg_id
			 */
		} else {
			sst_wake_up_block(sst_drv_ctx, 0, str_id,
				IPC_IA_DRAIN_STREAM, NULL, 0);
		}
		ops->post_message(&sst_drv_ctx->ipc_post_msg_wq);
		retval = sst_wait_timeout(sst_drv_ctx, block);
		pr_debug("sst: wait for free returned %d\n", retval);
		mutex_lock(&sst_drv_ctx->stream_lock);
		sst_clean_stream(str_info);
		mutex_unlock(&sst_drv_ctx->stream_lock);
		pr_debug("SST DBG:Stream freed\n");
		sst_free_block(sst_drv_ctx, block);
	} else {
		mutex_unlock(&str_info->lock);
		retval = -EBADRQC;
		pr_debug("SST DBG:BADQRC for stream\n");
	}

	return retval;
}
