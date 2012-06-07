/*
 *  intel_sst_interface.c - Intel SST Driver for audio engine
 *
 *  Copyright (C) 2008-10 Intel Corp
 *  Authors:	Vinod Koul <vinod.koul@intel.com>
 *		Harsha Priya <priya.harsha@intel.com>
 *		Dharageswari R <dharageswari.r@intel.com)
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
 *  This driver exposes the audio engine functionalities to the ALSA
 *	and middleware.
 *  Upper layer interfaces (MAD driver, MMF) to SST driver
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/fs.h>
#include <linux/firmware.h>
#include <linux/pm_runtime.h>
#include <sound/intel_sst_ioctl.h>
#include "../sst_platform.h"
#include "intel_sst_fw_ipc.h"
#include "intel_sst_common.h"

static void sst_restore_fw_context(void)
{
	struct snd_sst_ctxt_params fw_context;
	struct ipc_post *msg = NULL;
	int retval = 0;

	pr_debug("restore_fw_context\n");
	/*nothing to restore*/
	if (!sst_drv_ctx->fw_cntx_size)
		return;
	pr_debug("restoring context......\n");
	/*send msg to fw*/
	if (sst_create_large_msg(&msg))
		return;
	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_CTXT_RESTORE);
	sst_fill_header(&msg->header, IPC_IA_SET_FW_CTXT, 1, 0);
	sst_drv_ctx->alloc_block[0].sst_id = FW_DWNL_ID;
	sst_drv_ctx->alloc_block[0].ops_block.condition = false;
	sst_drv_ctx->alloc_block[0].ops_block.ret_code = 0;
	sst_drv_ctx->alloc_block[0].ops_block.on = true;

	msg->header.part.data = sizeof(fw_context) + sizeof(u32);
	fw_context.address = virt_to_phys((void *)sst_drv_ctx->fw_cntx);
	fw_context.size = sst_drv_ctx->fw_cntx_size;
	memcpy(msg->mailbox_data, &msg->header, sizeof(u32));
	memcpy(msg->mailbox_data + sizeof(u32),
				&fw_context, sizeof(fw_context));
	spin_lock(&sst_drv_ctx->list_spin_lock);
	list_add_tail(&msg->node, &sst_drv_ctx->ipc_dispatch_list);
	spin_unlock(&sst_drv_ctx->list_spin_lock);
	sst_post_message(&sst_drv_ctx->ipc_post_msg_wq);
	retval = sst_wait_timeout(sst_drv_ctx,
				&sst_drv_ctx->alloc_block[0].ops_block);
	sst_drv_ctx->alloc_block[0].sst_id = BLOCK_UNINIT;
	if (retval)
		pr_err("sst_restore_fw_context..timeout!\n");
	return;
}

/*
 * sst_download_fw - download the audio firmware to DSP
 *
 * This function is called when the FW needs to be downloaded to SST DSP engine
 */
int sst_download_fw(void)
{
	int retval;

	char name[20];

	if (sst_drv_ctx->sst_state != SST_START_INIT)
		return -EAGAIN;

	snprintf(name, sizeof(name), "%s%04x%s", "fw_sst_",
					sst_drv_ctx->pci_id, ".bin");

	if (!sst_drv_ctx->fw_in_mem) {
		retval = sst_request_fw();
		if (retval)
			return retval;
	}
	sst_drv_ctx->alloc_block[0].sst_id = FW_DWNL_ID;
	sst_drv_ctx->alloc_block[0].ops_block.condition = false;
	retval = sst_load_fw(sst_drv_ctx->fw_in_mem, NULL);
	if (retval)
		goto end_restore;

	retval = sst_wait_timeout(sst_drv_ctx,
				&sst_drv_ctx->alloc_block[0].ops_block);
	if (retval) {
		pr_err("fw download failed %d\n" , retval);
		/* assume FW d/l failed due to timeout*/
		retval = -EBUSY;
	}

end_restore:
	sst_drv_ctx->alloc_block[0].sst_id = BLOCK_UNINIT;
	if (retval)
		return retval;

	sst_restore_fw_context();
	sst_set_fw_state_locked(sst_drv_ctx, SST_FW_RUNNING);

	return retval;
}
/*
 * sst_stalled - this function checks if the lpe is in stalled state
 */
int sst_stalled(void)
{
	int retry = 1000;
	int retval = -1;

	while (retry) {
		if (!sst_drv_ctx->lpe_stalled)
			return 0;
		/*wait for time and re-check*/
		usleep_range(1000, 1500);

		retry--;
	}
	pr_debug("in Stalled State\n");
	return retval;
}

void free_stream_context(unsigned int str_id)
{
	struct stream_info *stream;
	stream = get_stream_info(str_id);
	if (stream) {
		/* str_id is valid, so stream is alloacted */
		if (sst_free_stream(str_id))
			sst_clean_stream(&sst_drv_ctx->streams[str_id]);
		if (stream->ops == STREAM_OPS_PLAYBACK ||
				stream->ops == STREAM_OPS_PLAYBACK_DRM)
			sst_drv_ctx->pb_streams--;
		else if (stream->ops == STREAM_OPS_CAPTURE)
			sst_drv_ctx->cp_streams--;
	}
}

void sst_send_lpe_mixer_algo_params(void)
{
	struct snd_ppp_params algo_param;
	struct snd_ppp_mixer_params mixer_param;
	unsigned int input_mixer, stream_device_id;

	mutex_lock(&sst_drv_ctx->mixer_ctrl_lock);
	input_mixer = (sst_drv_ctx->device_input_mixer)
				& SST_INPUT_STREAM_MIXED;
	stream_device_id = sst_drv_ctx->device_input_mixer - input_mixer;
	algo_param.algo_id = SST_CODEC_MIXER;
	algo_param.str_id = stream_device_id;
	algo_param.enable = 1;
	algo_param.reserved = 0;
	algo_param.size = sizeof(algo_param);
	mixer_param.type = SST_ALGO_PARAM_MIXER_STREAM_CFG;
	mixer_param.input_stream_bitmap = input_mixer;
	mixer_param.size = sizeof(input_mixer);
	algo_param.params = &mixer_param;
	mutex_unlock(&sst_drv_ctx->mixer_ctrl_lock);
	pr_err("setting pp param\n");
	pr_debug("Algo ID %d Str id %d Enable %d Size %d\n",
			algo_param.algo_id, algo_param.str_id,
			algo_param.enable, algo_param.size);
	 sst_send_algo_param(&algo_param);
}


/*
 * sst_get_stream_allocated - this function gets a stream allocated with
 * the given params
 *
 * @str_param : stream params
 * @lib_dnld : pointer to pointer of lib downlaod struct
 *
 * This creates new stream id for a stream, in case lib is to be downloaded to
 * DSP, it downloads that
 */
int sst_get_stream_allocated(struct sst_stream_params *str_param,
		struct snd_sst_lib_download **lib_dnld)
{
	int retval, str_id;
	struct stream_info *str_info;

	if (sst_drv_ctx->pci_id == SST_CLV_PCI_ID) {
		pr_debug("Sending LPE mixer algo Params\n");
		sst_send_lpe_mixer_algo_params();
	}
	retval = sst_alloc_stream((char *) &str_param->sparams, str_param->ops,
				str_param->codec, str_param->device_type);
	if (retval < 0) {
		pr_err("sst_alloc_stream failed %d\n", retval);
		return retval;
	}
	pr_debug("Stream allocated %d\n", retval);
	str_id = retval;
	str_info = &sst_drv_ctx->streams[str_id];
	/* Block the call for reply */
	retval = sst_wait_timeout(sst_drv_ctx, &str_info->ctrl_blk);
	if ((retval != 0) || (str_info->ctrl_blk.ret_code != 0)) {
		pr_err("sst: FW alloc failed retval %d, ret_code %d\n",\
				retval, str_info->ctrl_blk.ret_code);
		if (retval == SST_ERR_STREAM_IN_USE) {
			pr_err("sst:FW not in clean state, send free for:%d\n",
					str_param->device_type);
			sst_free_stream(str_param->device_type);
		}
		str_id = -str_info->ctrl_blk.ret_code; /*return error*/
		if (str_id == 0)
			str_id = retval; /*FW timed out*/
		*lib_dnld = str_info->ctrl_blk.data;
		sst_clean_stream(str_info);
	} else
		pr_debug("FW Stream allocated success\n");
	return str_id; /*will ret either error (in above if) or correct str id*/
}

/*
 * sst_get_sfreq - this function returns the frequency of the stream
 *
 * @str_param : stream params
 */
static int sst_get_sfreq(struct sst_stream_params *str_param)
{
	switch (str_param->codec) {
	case SST_CODEC_TYPE_PCM:
		return str_param->sparams.sfreq;
	case SST_CODEC_TYPE_MP3:
		return str_param->sparams.sfreq;
	case SST_CODEC_TYPE_AAC:
		return str_param->sparams.sfreq;
	case SST_CODEC_TYPE_WMA9:
		return str_param->sparams.sfreq;
	default:
		return 0;
	}
}

/*
 * sst_get_stream - this function prepares for stream allocation
 *
 * @str_param : stream param
 */
int sst_get_stream(struct sst_stream_params *str_param)
{
	int i, retval;
	struct stream_info *str_info;
	struct snd_sst_lib_download *lib_dnld;

	/* stream is not allocated, we are allocating */
	retval = sst_get_stream_allocated(str_param, &lib_dnld);
	if (retval == -(SST_LIB_ERR_LIB_DNLD_REQUIRED)) {
		/* codec download is required */
		struct snd_sst_alloc_response *response;

		pr_debug("Codec is required.... trying that\n");
		if (lib_dnld == NULL) {
			pr_err("lib download null!!! abort\n");
			return -EIO;
		}
		i = sst_get_block_stream(sst_drv_ctx);
		if (i < 0) {
			pr_err("invalid value for number of stream\n ");
			kfree(lib_dnld);
			return i;
		}
		response = sst_drv_ctx->alloc_block[i].ops_block.data;
		pr_debug("alloc block allocated = %d\n", i);

		retval = sst_load_library(lib_dnld, str_param->ops);
		kfree(lib_dnld);

		sst_drv_ctx->alloc_block[i].sst_id = BLOCK_UNINIT;
		if (!retval) {
			pr_debug("codec was downloaded successfully\n");

			retval = sst_get_stream_allocated(str_param, &lib_dnld);
			if (retval <= 0) {
				retval = -EIO;
				goto err;
			}

			pr_debug("Alloc done stream id %d\n", retval);
		} else {
			pr_debug("codec download failed\n");
			retval = -EIO;
			goto err;
		}
	} else if  (retval <= 0) {
		retval = -EIO;
		goto err;
	}
	/*else
		set_port_params(str_param, str_param->ops);*/

	/* store sampling freq */
	str_info = &sst_drv_ctx->streams[retval];
	str_info->sfreq = sst_get_sfreq(str_param);

	/* power on the analog, if reqd */
	if (str_param->ops == STREAM_OPS_PLAYBACK ||
			str_param->ops == STREAM_OPS_PLAYBACK_DRM) {
		/*Only if the playback is MP3 - Send a message*/
		sst_drv_ctx->pb_streams++;
	} else if (str_param->ops == STREAM_OPS_CAPTURE) {

		/*Send a messageif not sent already*/
		sst_drv_ctx->cp_streams++;
	}

err:
	return retval;
}

static int sst_prepare_fw(void)
{
	int retval = 0;

	mutex_lock(&sst_drv_ctx->sst_lock);
	if (sst_drv_ctx->sst_state == SST_UN_INIT) {
		sst_drv_ctx->sst_state = SST_START_INIT;
		mutex_unlock(&sst_drv_ctx->sst_lock);
		/* FW is not downloaded */
		pr_debug("DSP Downloading FW now...\n");
		retval = sst_download_fw();
		if (retval) {
			pr_err("FW download fail %x\n", retval);
			sst_set_fw_state_locked(sst_drv_ctx, SST_UN_INIT);
			return retval;
		}
	} else {
		mutex_unlock(&sst_drv_ctx->sst_lock);
	}
	return retval;
}

void sst_process_mad_ops(struct work_struct *work)
{

	struct mad_ops_wq *mad_ops =
			container_of(work, struct mad_ops_wq, wq);
	int retval = 0;

	switch (mad_ops->control_op) {
	case SST_SND_PAUSE:
		retval = sst_pause_stream(mad_ops->stream_id);
		break;
	case SST_SND_RESUME:
		retval = sst_resume_stream(mad_ops->stream_id);
		break;
	case SST_SND_DROP:
		pr_debug("in mad_ops drop stream\n");
		retval = sst_drop_stream(mad_ops->stream_id);
		break;
	case SST_SND_START:
		pr_debug("start stream\n");
		retval = sst_start_stream(mad_ops->stream_id);
		break;
	case SST_SND_STREAM_PROCESS:
		pr_debug("play/capt frames...\n");
		break;
	default:
		pr_err(" wrong control_ops reported\n");
	}
	if (retval)
		pr_err("%s(): op: %d, retval: %d\n",
				__func__, mad_ops->control_op, retval);
	kfree(mad_ops);
	return;
}

/*
 * sst_open_pcm_stream - Open PCM interface
 *
 * @str_param: parameters of pcm stream
 *
 * This function is called by MID sound card driver to open
 * a new pcm interface
 */
static int sst_open_pcm_stream(struct sst_stream_params *str_param)
{
	struct stream_info *str_info;
	int retval;

	if (!str_param)
		return -EINVAL;

	pr_debug("open_pcm, doing rtpm_get\n");
	pm_runtime_get_sync(&sst_drv_ctx->pci->dev);

	retval = sst_prepare_fw();
	if (retval) {
		pr_err("Unable to download FW\n");
		pm_runtime_put(&sst_drv_ctx->pci->dev);
		return retval;
	}

	mutex_lock(&sst_drv_ctx->sst_lock);
	if (sst_drv_ctx->sst_state != SST_FW_RUNNING) {
		mutex_unlock(&sst_drv_ctx->sst_lock);
		return -EAGAIN;
	}
	mutex_unlock(&sst_drv_ctx->sst_lock);

	retval = sst_get_stream(str_param);
	if (retval > 0) {
		sst_drv_ctx->stream_cnt++;
		str_info = &sst_drv_ctx->streams[retval];
		str_info->src = MAD_DRV;
	}

	return retval;
}

/*
 * sst_close_pcm_stream - Close PCM interface
 *
 * @str_id: stream id to be closed
 *
 * This function is called by MID sound card driver to close
 * an existing pcm interface
 */
static int sst_close_pcm_stream(unsigned int str_id)
{
	struct stream_info *stream;

	pr_debug("stream free called\n");
	stream = get_stream_info(str_id);
	if (!stream)
		return -EINVAL;
	free_stream_context(str_id);
	stream->pcm_substream = NULL;
	stream->status = STREAM_UN_INIT;
	stream->period_elapsed = NULL;
	sst_drv_ctx->stream_cnt--;
	pr_debug("will call runtime put now\n");
	pm_runtime_put(&sst_drv_ctx->pci->dev);
	return 0;
}

int sst_send_sync_msg(int ipc, int str_id)
{
	struct ipc_post *msg = NULL;

	if (sst_create_short_msg(&msg))
		return -ENOMEM;
	sst_fill_header(&msg->header, ipc, 0, str_id);
	return sst_sync_post_message(msg);
}

/*
 * sst_device_control - Set Control params
 *
 * @cmd: control cmd to be set
 * @arg: command argument
 *
 * This function is called by MID sound card driver to set
 * SST/Sound card controls for an opened stream.
 * This is registered with MID driver
 */
static int sst_device_control(int cmd, void *arg)
{
	int retval = 0, str_id = 0;

	switch (cmd) {
	case SST_SND_PAUSE:
	case SST_SND_RESUME: {
		struct mad_ops_wq *work = kzalloc(sizeof(*work), GFP_ATOMIC);
		if (!work)
			return -ENOMEM;
		INIT_WORK(&work->wq, sst_process_mad_ops);
		work->control_op = cmd;
		work->stream_id = *(int *)arg;
		queue_work(sst_drv_ctx->mad_wq, &work->wq);
		break;
	}
	case SST_SND_START: {
		struct stream_info *str_info;
		int ipc;
		str_id = *(int *)arg;
		str_info = get_stream_info(str_id);
		if (!str_info)
			return -EINVAL;
		ipc = IPC_IA_START_STREAM;
		str_info->prev = str_info->status;
		str_info->status = STREAM_RUNNING;
		retval = sst_send_sync_msg(ipc, str_id);
		break;
	}
	case SST_SND_DROP: {
		struct stream_info *str_info;
		int ipc;
		str_id = *(int *)arg;
		str_info = get_stream_info(str_id);
		if (!str_info)
			return -EINVAL;
		ipc = IPC_IA_DROP_STREAM;
		str_info->prev = STREAM_UN_INIT;
		str_info->status = STREAM_INIT;
		retval = sst_send_sync_msg(ipc, str_id);
		break;
	}
	case SST_SND_STREAM_INIT: {
		struct pcm_stream_info *str_info;
		struct stream_info *stream;

		pr_debug("stream init called\n");
		str_info = (struct pcm_stream_info *)arg;
		str_id = str_info->str_id;
		stream = get_stream_info(str_id);
		if (!stream) {
			retval = -EINVAL;
			break;
		}
		pr_debug("setting the period ptrs\n");
		stream->pcm_substream = str_info->mad_substream;
		stream->period_elapsed = str_info->period_elapsed;
		stream->sfreq = str_info->sfreq;
		stream->prev = stream->status;
		stream->status = STREAM_INIT;
		break;
	}

	case SST_SND_BUFFER_POINTER: {
		struct pcm_stream_info *stream_info;
		struct snd_sst_tstamp fw_tstamp = {0,};
		struct stream_info *stream;


		stream_info = (struct pcm_stream_info *)arg;
		str_id = stream_info->str_id;
		stream = get_stream_info(str_id);
		if (!stream) {
			retval = -EINVAL;
			break;
		}

		if (!stream->pcm_substream)
			break;
		memcpy_fromio(&fw_tstamp,
			((void *)(sst_drv_ctx->mailbox + SST_TIME_STAMP)
			+(str_id * sizeof(fw_tstamp))),
			sizeof(fw_tstamp));

		pr_debug("Pointer Query on strid = %d ops %d\n",
						str_id, stream->ops);

		if (stream->ops == STREAM_OPS_PLAYBACK)
			stream_info->buffer_ptr = fw_tstamp.samples_rendered;
		else
			stream_info->buffer_ptr = fw_tstamp.samples_processed;
		stream_info->pcm_delay = fw_tstamp.pcm_delay;
		pr_debug("Samples rendered = %llu, buffer ptr %llu\n",
			fw_tstamp.samples_rendered, stream_info->buffer_ptr);
		break;
	}
	default:
		/* Illegal case */
		pr_warn("illegal req\n");
		return -EINVAL;
	}

	return retval;
}

/*
 * sst_copy_runtime_param - copy runtime params from src to dst
 *				 structure.
 *
 *@dst: destination runtime structure
 *@src: source runtime structure
 *
 * This helper function is called to copy the runtime parameter
 * structure.
*/
static int sst_copy_runtime_param(struct snd_sst_runtime_params *dst,
			struct snd_sst_runtime_params *src)
{
	dst->type = src->type;
	dst->str_id = src->str_id;
	dst->size = src->size;
	if (dst->addr) {
		pr_err("mem allocated in prev setting, use the same memory\n");
		return -EINVAL;
	}
	dst->addr = kzalloc(dst->size, GFP_KERNEL);
	if (!dst->addr)
		return -ENOMEM;
	memcpy(dst->addr, src->addr, dst->size);
	return 0;
}
/*
 * sst_set_generic_params - Set generic params
 *
 * @cmd: control cmd to be set
 * @arg: command argument
 *
 * This function is called by MID sound card driver to configure
 * SST runtime params.
 */
static int sst_set_generic_params(enum sst_controls cmd, void *arg)
{
	int ret_val = 0;
	pr_debug("Enter:%s, cmd:%d\n", __func__, cmd);

	if (NULL == arg)
		return -EINVAL;

	switch (cmd) {
	case SST_SET_RUNTIME_PARAMS: {
		struct snd_sst_runtime_params *src;
		struct snd_sst_runtime_params *dst;

		src = (struct snd_sst_runtime_params *)arg;
		dst = &(sst_drv_ctx->runtime_param.param);
		ret_val = sst_copy_runtime_param(dst, src);
		break;
		}
	case SST_SET_ALGO_PARAMS: {
		unsigned int device_input_mixer = *((unsigned int *)arg);
		pr_debug("LPE mixer algo param set %x\n", device_input_mixer);
		mutex_lock(&sst_drv_ctx->mixer_ctrl_lock);
		sst_drv_ctx->device_input_mixer = device_input_mixer;
		mutex_unlock(&sst_drv_ctx->mixer_ctrl_lock);
		break;
	}
	default:
		pr_err("Invalid cmd request:%d\n", cmd);
		ret_val = -EINVAL;
	}
	return ret_val;
}

static struct sst_ops pcm_ops = {
	.open = sst_open_pcm_stream,
	.device_control = sst_device_control,
	.set_generic_params = sst_set_generic_params,
	.close = sst_close_pcm_stream,
};

static struct sst_device sst_dsp_device = {
	.name = "Intel(R) SST LPE",
	.dev = NULL,
	.ops = &pcm_ops,
};

/*
 * register_sst - function to register DSP
 *
 * This functions registers DSP with the platform driver
 */
int register_sst(struct device *dev)
{
	int ret_val;
	sst_dsp_device.dev = dev;
	ret_val = sst_register_dsp(&sst_dsp_device);
	if (ret_val)
		pr_err("Unable to register DSP with platform driver\n");

	return ret_val;
}

int unregister_sst(struct device *dev)
{
	return sst_unregister_dsp(&sst_dsp_device);
}
