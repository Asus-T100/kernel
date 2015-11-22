/*
 *  pcm.c - Intel MID Platform driver file implementing PCM functionality
 *
 *  Copyright (C) 2010-2013 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Author: Harsha Priya <priya.harsha@intel.com>
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
 *
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/intel_sst_ioctl.h>
#include <asm/platform_sst_audio.h>
#include <asm/intel_sst_mrfld.h>
#include <asm/intel-mid.h>
#include "platform_ipc_v2.h"
#include "sst_platform.h"
#include "sst_platform_pvt.h"

struct device *sst_pdev;
struct sst_device *sst_dsp;
extern struct snd_compr_ops sst_platform_compr_ops;
extern struct snd_effect_ops effects_ops;

/* module parameters */
static int dpcm_enable;
static int dfw_enable;

/* dpcm_enable should be =0 for mofd_v0 and =1 for mofd_v1 */
module_param(dpcm_enable, int, 0644);
MODULE_PARM_DESC(dpcm_enable, "DPCM module parameter");
/* dfw_enable should be =0 for mofd_v0 and =1 for mofd_v1 */
module_param(dfw_enable, int, 0644);
MODULE_PARM_DESC(dfw_enable, "DFW module parameter");

static DEFINE_MUTEX(sst_dsp_lock);

static struct snd_pcm_hardware sst_platform_pcm_hw = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_DOUBLE |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_MMAP|
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
			SNDRV_PCM_INFO_NO_PERIOD_WAKEUP |
			SNDRV_PCM_INFO_SYNC_START),
	.formats = (SNDRV_PCM_FMTBIT_S16 | SNDRV_PCM_FMTBIT_U16 |
			SNDRV_PCM_FMTBIT_S24 | SNDRV_PCM_FMTBIT_U24 |
			SNDRV_PCM_FMTBIT_S32 | SNDRV_PCM_FMTBIT_U32),
	.rates = (SNDRV_PCM_RATE_8000|
			SNDRV_PCM_RATE_16000 |
			SNDRV_PCM_RATE_44100 |
			SNDRV_PCM_RATE_48000),
	.rate_min = SST_MIN_RATE,
	.rate_max = SST_MAX_RATE,
	.channels_min =	SST_MIN_CHANNEL,
	.channels_max =	SST_MAX_CHANNEL,
	.buffer_bytes_max = SST_MAX_BUFFER,
	.period_bytes_min = SST_MIN_PERIOD_BYTES,
	.period_bytes_max = SST_MAX_PERIOD_BYTES,
	.periods_min = SST_MIN_PERIODS,
	.periods_max = SST_MAX_PERIODS,
	.fifo_size = SST_FIFO_SIZE,
};

static struct sst_dev_stream_map dpcm_strm_map[] = {
	{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, /* Reserved, not in use */
	{MERR_DPCM_AUDIO, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_MEDIA1_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_DB,    0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_MEDIA2_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_LL,    0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_LOW_PCM0_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_COMPR, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_MEDIA0_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_VOIP,  0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_VOIP_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 0, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE1_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 1, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE2_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 2, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE3_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 3, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE4_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 4, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE5_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 5, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE6_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 6, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE7_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 7, SNDRV_PCM_STREAM_PLAYBACK, PIPE_PROBE8_IN, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_AUDIO, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_MEDIA2_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_VOIP,  0, SNDRV_PCM_STREAM_CAPTURE, PIPE_VOIP_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_CAPTURE, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_MEDIA3_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 0, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE1_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 1, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE2_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 2, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE3_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 3, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE4_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 4, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE5_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 5, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE6_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 6, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE7_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	{MERR_DPCM_PROBE, 7, SNDRV_PCM_STREAM_CAPTURE, PIPE_PROBE8_OUT, SST_TASK_ID_MEDIA, SST_DEV_MAP_IN_USE},
	/* stream ID 25 used by Aware, but no device exposed to userspace */
};

static int sst_platform_ihf_set_tdm_slot(struct snd_soc_dai *dai,
			unsigned int tx_mask, unsigned int rx_mask,
			int slots, int slot_width) {
	struct snd_sst_runtime_params params_data;
	int channels = slots;

	/* registering with SST driver to get access to SST APIs to use */
	if (!sst_dsp) {
		pr_err("sst: DSP not registered\n");
		return -EIO;
	}
	params_data.type = SST_SET_CHANNEL_INFO;
	params_data.str_id = SND_SST_DEVICE_IHF;
	params_data.size = sizeof(channels);
	params_data.addr = &channels;
	return sst_dsp->ops->set_generic_params(SST_SET_RUNTIME_PARAMS,
							(void *)&params_data);
}

static int sst_media_digital_mute(struct snd_soc_dai *dai, int mute, int stream)
{

	pr_debug("%s: enter, mute=%d dai-name=%s dir=%d\n", __func__, mute, dai->name, stream);

	if (dpcm_enable == 1)
		sst_send_pipe_gains(dai, stream, mute);

	return 0;
}

/* helper functions */
void sst_set_stream_status(struct sst_runtime_stream *stream,
					int state)
{
	unsigned long flags;
	spin_lock_irqsave(&stream->status_lock, flags);
	stream->stream_status = state;
	spin_unlock_irqrestore(&stream->status_lock, flags);
}

static inline int sst_get_stream_status(struct sst_runtime_stream *stream)
{
	int state;
	unsigned long flags;

	spin_lock_irqsave(&stream->status_lock, flags);
	state = stream->stream_status;
	spin_unlock_irqrestore(&stream->status_lock, flags);
	return state;
}

static void sst_fill_alloc_params(struct snd_pcm_substream *substream,
				struct snd_sst_alloc_params_ext *alloc_param)
{
	unsigned int channels;
	snd_pcm_uframes_t period_size;
	ssize_t periodbytes;
	ssize_t buffer_bytes = snd_pcm_lib_buffer_bytes(substream);
	u32 buffer_addr = virt_to_phys(substream->dma_buffer.area);

	channels = substream->runtime->channels;
	period_size = substream->runtime->period_size;
	periodbytes = samples_to_bytes(substream->runtime, period_size);
	alloc_param->ring_buf_info[0].addr = buffer_addr;
	alloc_param->ring_buf_info[0].size = buffer_bytes;
	alloc_param->sg_count = 1;
	alloc_param->reserved = 0;
	alloc_param->frag_size = periodbytes * channels;

	pr_debug("period_size = %d\n", alloc_param->frag_size);
	pr_debug("ring_buf_addr = 0x%x\n", alloc_param->ring_buf_info[0].addr);
}
static void sst_fill_pcm_params(struct snd_pcm_substream *substream,
				struct snd_sst_stream_params *param)
{
	param->uc.pcm_params.num_chan = (u8) substream->runtime->channels;
	param->uc.pcm_params.pcm_wd_sz = substream->runtime->sample_bits;
	param->uc.pcm_params.sfreq = substream->runtime->rate;

	/* PCM stream via ALSA interface */
	param->uc.pcm_params.use_offload_path = 0;
	param->uc.pcm_params.reserved2 = 0;
	memset(param->uc.pcm_params.channel_map, 0, sizeof(u8));
	pr_debug("sfreq= %d, wd_sz = %d\n",
	param->uc.pcm_params.sfreq, param->uc.pcm_params.pcm_wd_sz);

}

#define ASSIGN_PIPE_ID(periodtime, lowlatency, deepbuffer) \
	((periodtime) <= (lowlatency) ? PIPE_LOW_PCM0_IN : \
	((periodtime) >= (deepbuffer) ? PIPE_MEDIA3_IN : PIPE_MEDIA1_IN))

static int sst_get_stream_mapping(int dev, int sdev, int dir,
	struct sst_dev_stream_map *map, int size, u8 pipe_id,
	const struct sst_lowlatency_deepbuff *ll_db)
{
	int index;
	unsigned long pt = 0, ll = 0, db = 0;

	if (map == NULL)
		return -EINVAL;

	pr_debug("dev %d sdev %d dir %d\n", dev, sdev, dir);

	/* index 0 is not used in stream map */
	for (index = 1; index < size; index++) {
		if ((map[index].dev_num == dev) &&
		    (map[index].subdev_num == sdev) &&
		    (map[index].direction == dir)) {
			/* device id for the probe is assigned dynamically */
			if (map[index].status == SST_DEV_MAP_IN_USE) {
				return index;
			} else if (map[index].status == SST_DEV_MAP_FREE) {
				map[index].status = SST_DEV_MAP_IN_USE;

				if (map[index].dev_num == MERR_SALTBAY_PROBE) {
					map[index].device_id = pipe_id;

				} else if (map[index].dev_num == MERR_SALTBAY_AUDIO) {
					if (!ll_db->low_latency || !ll_db->deep_buffer)
						return -EINVAL;

					pt = ll_db->period_time;
					ll = *(ll_db->low_latency);
					db = *(ll_db->deep_buffer);

					pr_debug("PT %lu LL %lu DB %lu\n", pt, ll, db);

					map[index].device_id = ASSIGN_PIPE_ID(pt,
								ll, db);
				}
				pr_debug("%s: pipe_id 0%x index %d", __func__,
						map[index].device_id, index);

				return index;
			}
		}
	}
	return 0;
}

int sst_fill_stream_params(void *substream,
	const struct sst_data *ctx, struct snd_sst_params *str_params, bool is_compress)
{
	int map_size;
	int index;
	struct sst_dev_stream_map *map;
	struct snd_pcm_substream *pstream = NULL;
	struct snd_compr_stream *cstream = NULL;

	map = ctx->pdata->pdev_strm_map;
	map_size = ctx->pdata->strm_map_size;

	if (is_compress == true) {
		cstream = (struct snd_compr_stream *)substream;
		str_params->no_irq = 0;
	} else {
		pstream = (struct snd_pcm_substream *)substream;
		str_params->no_irq = pstream->runtime->no_period_wakeup;
	}
	str_params->stream_type = SST_STREAM_TYPE_MUSIC;

	/* For pcm streams */
	if (pstream) {
		index = sst_get_stream_mapping(pstream->pcm->device,
					  pstream->number, pstream->stream,
					  map, map_size, ctx->pipe_id, &ctx->ll_db);
		if (index <= 0)
			return -EINVAL;

		str_params->stream_id = index;
		str_params->device_type = map[index].device_id;
		str_params->task = map[index].task_id;

		if (str_params->device_type == SST_PROBE_IN)
			str_params->stream_type = SST_STREAM_TYPE_PROBE;

		pr_debug("str_id = %d, device_type = 0x%x, task = %d",
			 str_params->stream_id, str_params->device_type,
			 str_params->task);

		str_params->ops = (u8)pstream->stream;
	}

	if (cstream) {
		/* FIXME: Add support for subdevice number in
		 * snd_compr_stream */
		index = sst_get_stream_mapping(cstream->device->device,
					       0, cstream->direction,
					       map, map_size, ctx->pipe_id, &ctx->ll_db);
		if (index <= 0)
			return -EINVAL;
		str_params->stream_id = index;
		str_params->device_type = map[index].device_id;
		str_params->task = map[index].task_id;
		pr_debug("compress str_id = %d, device_type = 0x%x, task = %d",
			 str_params->stream_id, str_params->device_type,
			 str_params->task);

		str_params->ops = (u8)cstream->direction;
	}
	return 0;
}

#define CALC_PERIODTIME(period_size, rate) (((period_size) * 1000) / (rate))

static int sst_platform_alloc_stream(struct snd_pcm_substream *substream,
		struct snd_soc_platform *platform)
{
	struct sst_runtime_stream *stream =
			substream->runtime->private_data;
	struct snd_sst_stream_params param = {{{0,},},};
	struct snd_sst_params str_params = {0};
	struct snd_sst_alloc_params_ext alloc_params = {0};
	int ret_val = 0;
	struct sst_data *ctx = snd_soc_platform_get_drvdata(platform);

	/* set codec params and inform SST driver the same */
	sst_fill_pcm_params(substream, &param);
	sst_fill_alloc_params(substream, &alloc_params);
	substream->runtime->dma_area = substream->dma_buffer.area;
	str_params.sparams = param;
	str_params.aparams = alloc_params;
	str_params.codec = SST_CODEC_TYPE_PCM;

	ctx->ll_db.period_time = CALC_PERIODTIME(substream->runtime->period_size,
					substream->runtime->rate);

	/* fill the device type and stream id to pass to SST driver */
	ret_val = sst_fill_stream_params(substream, ctx, &str_params, false);
	pr_debug("platform prepare: fill stream params ret_val = 0x%x\n", ret_val);
	if (ret_val < 0)
		return ret_val;

	stream->stream_info.str_id = str_params.stream_id;

	ret_val = stream->ops->open(&str_params);
	pr_debug("platform prepare: stream open ret_val = 0x%x\n", ret_val);
	if (ret_val <= 0)
		return ret_val;

	pr_debug("platform allocated strid:  %d\n", stream->stream_info.str_id);

	return ret_val;
}

static void sst_period_elapsed(void *mad_substream)
{
	struct snd_pcm_substream *substream = mad_substream;
	struct sst_runtime_stream *stream;
	int status;

	if (!substream || !substream->runtime) {
		pr_debug("In %s : Null Substream pointer\n", __func__);
		return;
	}
	stream = substream->runtime->private_data;
	if (!stream) {
		pr_debug("In %s : Null Stream pointer\n", __func__);
		return;
	}
	status = sst_get_stream_status(stream);
	if (status != SST_PLATFORM_RUNNING) {
		pr_debug("In %s : Stream Status=%d\n", __func__, status);
		return;
	}
	snd_pcm_period_elapsed(substream);
}

static int sst_platform_init_stream(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream =
			substream->runtime->private_data;
	int ret_val;

	pr_debug("setting buffer ptr param\n");
	sst_set_stream_status(stream, SST_PLATFORM_INIT);
	stream->stream_info.period_elapsed = sst_period_elapsed;
	stream->stream_info.mad_substream = substream;
	stream->stream_info.buffer_ptr = 0;
	stream->stream_info.sfreq = substream->runtime->rate;
	pr_debug("pcm_substream %p, period_elapsed %p\n",
			stream->stream_info.mad_substream, stream->stream_info.period_elapsed);
	ret_val = stream->ops->device_control(
			SST_SND_STREAM_INIT, &stream->stream_info);
	if (ret_val)
		pr_err("control_set ret error %d\n", ret_val);
	return ret_val;

}

static inline int power_up_sst(struct sst_runtime_stream *sst)
{
	return sst->ops->power(true);
}

static inline int power_down_sst(struct sst_runtime_stream *sst)
{
	return sst->ops->power(false);
}
/* end -- helper functions */

static int sst_media_open(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	int ret_val = 0;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sst_runtime_stream *stream;

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	spin_lock_init(&stream->status_lock);

	/* get the sst ops */
	mutex_lock(&sst_dsp_lock);
	if (!sst_dsp ||
	    !try_module_get(sst_dsp->dev->driver->owner)) {
		pr_err("no device available to run\n");
		ret_val = -ENODEV;
		goto out_ops;
	}
	stream->ops = sst_dsp->ops;
	mutex_unlock(&sst_dsp_lock);

	stream->stream_info.str_id = 0;
	sst_set_stream_status(stream, SST_PLATFORM_UNINIT);
	stream->stream_info.mad_substream = substream;
	runtime->private_data = stream;

	if (strstr(dai->name, "Power-cpu-dai"))
		return power_up_sst(stream);

	/* Make sure, that the period size is always even */
	snd_pcm_hw_constraint_step(substream->runtime, 0,
			   SNDRV_PCM_HW_PARAM_PERIODS, 2);

	pr_debug("buf_ptr %llu\n", stream->stream_info.buffer_ptr);
	return snd_pcm_hw_constraint_integer(runtime,
			 SNDRV_PCM_HW_PARAM_PERIODS);
out_ops:
	kfree(stream);
	mutex_unlock(&sst_dsp_lock);
	return ret_val;
}

static void sst_free_stream_in_use(struct sst_dev_stream_map *map, int str_id)
{
	if (dpcm_enable == 1)
		return;

	if ((map[str_id].dev_num == MERR_SALTBAY_AUDIO) ||
			(map[str_id].dev_num == MERR_SALTBAY_PROBE)) {
		/* Do nothing in capture for audio device */
		if ((map[str_id].dev_num == MERR_SALTBAY_AUDIO) &&
				(map[str_id].direction == SNDRV_PCM_STREAM_CAPTURE))
			return;
		if ((map[str_id].task_id == SST_TASK_ID_MEDIA) &&
				(map[str_id].status == SST_DEV_MAP_IN_USE)) {
			pr_debug("str_id %d device_id 0x%x\n", str_id, map[str_id].device_id);
			map[str_id].status = SST_DEV_MAP_FREE;
			map[str_id].device_id = PIPE_RSVD;
		}
	}
	return;
}

static void sst_media_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;
	struct sst_data *ctx = snd_soc_platform_get_drvdata(dai->platform);

	stream = substream->runtime->private_data;
	if (strstr(dai->name, "Power-cpu-dai"))
		ret_val = power_down_sst(stream);

	str_id = stream->stream_info.str_id;
	if (str_id)
		ret_val = stream->ops->close(str_id);
	sst_free_stream_in_use(ctx->pdata->pdev_strm_map, str_id);
	module_put(sst_dsp->dev->driver->owner);
	kfree(stream);
	pr_debug("%s: %d\n", __func__, ret_val);
}

static int sst_dpcm_probe_cmd(struct snd_soc_platform *platform,
		struct snd_pcm_substream *substream, u16 pipe_id, bool on)
{
	int ret = 0;
	if ((dpcm_enable == 1) && (substream->pcm->device == MERR_DPCM_PROBE))
			ret = sst_dpcm_probe_send(platform, pipe_id, substream->number,
					substream->stream, on);
	return ret;
}

static inline unsigned int get_current_pipe_id(struct snd_soc_platform *platform,
					       struct snd_pcm_substream *substream)
{
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);
	struct sst_dev_stream_map *map = sst->pdata->pdev_strm_map;
	struct sst_runtime_stream *stream =
			substream->runtime->private_data;
	u32 str_id = stream->stream_info.str_id;
	unsigned int pipe_id;
	pipe_id = map[str_id].device_id;

	pr_debug("%s: got pipe_id = %#x for str_id = %d\n",
		 __func__, pipe_id, str_id);
	return pipe_id;
}

static void sst_probe_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	u16 probe_pipe_id = get_current_pipe_id(dai->platform, substream);

	sst_dpcm_probe_cmd(dai->platform, substream, probe_pipe_id, false);
	sst_media_close(substream, dai);
}

static int sst_media_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;

	pr_debug("%s\n", __func__);

	stream = substream->runtime->private_data;
	str_id = stream->stream_info.str_id;
	if (stream->stream_info.str_id)
		return ret_val;

	ret_val = sst_platform_alloc_stream(substream, dai->platform);
	if (ret_val <= 0)
		return ret_val;

	ret_val = sst_platform_init_stream(substream);
	if (ret_val)
		return ret_val;
	substream->runtime->hw.info = SNDRV_PCM_INFO_BLOCK_TRANSFER;

	return ret_val;
}

static int sst_probe_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	u16 probe_pipe_id;

	sst_media_prepare(substream, dai);
	probe_pipe_id = get_current_pipe_id(dai->platform, substream);

	return sst_dpcm_probe_cmd(dai->platform, substream, probe_pipe_id, true);
}

static int sst_media_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	pr_debug("%s\n", __func__);

	snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	memset(substream->runtime->dma_area, 0, params_buffer_bytes(params));
	return 0;
}

static int sst_get_ssp_num(const char *id)
{
	if (!strncmp(id, "ssp0-port", 9))
		return SST_SSP_PORT0;
	else if (!strncmp(id, "ssp1-port", 9))
		return SST_SSP_PORT1;
	else if (!strncmp(id, "ssp2-port", 9))
		return SST_SSP_PORT2;
	else
		return -EINVAL;
}

static int sst_be_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	int ssp_num;
	struct snd_interval *rate =  hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);

	ssp_num = sst_get_ssp_num(dai->name);
	if (ssp_num < 0)
		return ssp_num;
	if (dai->active == 1)
		send_ssp_cmd(dai->platform, rate->max, ssp_num, true);
	return 0;

}

static int sst_media_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return snd_pcm_lib_free_pages(substream);
}

static int sst_send_timer_cmd(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	pr_debug("In %s :dai=%s pb=%d cp= %d dai_active=%d id=%d\n", __func__,
		dai->name, dai->playback_active, dai->capture_active, dai->active,  dai->id);
	if (!dai->active) {
		sst_handle_vb_timer(dai->platform, true);
	}
	return 0;
}

static void sst_disable_ssp(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	int ssp_num;

	pr_debug("In %s :dai=%s pb=%d cp= %d dai_active=%d id=%d\n", __func__,
		dai->name, dai->playback_active, dai->capture_active, dai->active, dai->id);
	if (!dai->active) {
		ssp_num = sst_get_ssp_num(dai->name);
		if (ssp_num < 0)
			return;
		else
			send_ssp_cmd(dai->platform, 0, ssp_num, false);
		sst_handle_vb_timer(dai->platform, false);
	}
}

static int sst_platform_set_ssp_slot(struct snd_soc_dai *dai,
			unsigned int tx_mask, unsigned int rx_mask,
			int slots, int slot_width) {
	int ssp_num, ret;
	struct sst_data *ctx = snd_soc_platform_get_drvdata(dai->platform);

	if (!dai->active)
		return 0;

	ssp_num = sst_get_ssp_num(dai->name);
	if (ssp_num < 0)
		return ssp_num;
	ret = sst_fill_ssp_slot(ctx, tx_mask, rx_mask, ssp_num, slots, slot_width);
	if (ret < 0) {
		pr_err("sst_fill_ssp_slot failed..\n");
		return ret;
	}
	return 0;
}

static int sst_set_format(struct snd_soc_dai *dai, unsigned int fmt)
{
	int ssp_num, ret;
	struct sst_data *ctx = snd_soc_platform_get_drvdata(dai->platform);

	if (!dai->active)
		return 0;
	ssp_num = sst_get_ssp_num(dai->name);
	if (ssp_num < 0)
		return ssp_num;

	ret = sst_fill_ssp_config(ctx, ssp_num, fmt, true);
	if (ret < 0) {
		pr_err("sst_set_format failed..\n");
		return ret;
	}
	return 0;
}

static struct snd_soc_dai_ops sst_media_dai_ops = {
	.startup = sst_media_open,
	.shutdown = sst_media_close,
	.prepare = sst_media_prepare,
	.hw_params = sst_media_hw_params,
	.hw_free = sst_media_hw_free,
	.set_tdm_slot = sst_platform_ihf_set_tdm_slot,
	.mute_stream = sst_media_digital_mute,
};

static struct snd_soc_dai_ops sst_probe_dai_ops = {
	.startup = sst_media_open,
	.hw_params = sst_media_hw_params,
	.hw_free = sst_media_hw_free,
	.shutdown = sst_probe_close,
	.prepare = sst_probe_prepare,
};

static struct snd_soc_dai_ops sst_loopback_dai_ops = {
	.startup = sst_media_open,
	.shutdown = sst_media_close,
	.prepare = sst_media_prepare,
};

static struct snd_soc_dai_ops sst_compr_dai_ops = {
	.mute_stream = sst_media_digital_mute,
};

static struct snd_soc_dai_ops sst_be_dai_ops = {
	.startup = sst_send_timer_cmd,
	.hw_params = sst_be_hw_params,
	.set_fmt = sst_set_format,
	.set_tdm_slot = sst_platform_set_ssp_slot,
	.shutdown = sst_disable_ssp,
};

static struct snd_soc_dai_driver sst_platform_dai[] = {
{
	.name = SST_HEADSET_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.stream_name = "Headset Playback",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Headset Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_DEEPBUFFER_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.stream_name = "Deepbuffer Playback",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_LOWLATENCY_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.stream_name = "Low Latency Playback",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_SPEAKER_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.stream_name = "Speaker Playback",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_VOICE_DAI,
	.playback = {
		.stream_name = "Voice Downlink",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Voice Uplink",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_COMPRESS_DAI,
	.compress_dai = 1,
	.ops = &sst_compr_dai_ops,
	.playback = {
		.stream_name = "Compress Playback",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_VIRTUAL_DAI,
	.playback = {
		.stream_name = "Virtual Playback",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_POWER_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.stream_name = "Dummy Power Stream",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = SST_PROBE_DAI,
	.ops = &sst_probe_dai_ops,
	.playback = {
		.stream_name = "Probe Playback",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name = "Probe Capture",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
},
{
	.name = SST_VOIP_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.stream_name = "VOIP Playback",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "VOIP Capture",
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_LOOPBACK_DAI,
	.ops = &sst_loopback_dai_ops,
	.capture = {
		.stream_name = "Loopback Capture",
		.channels_min = SST_MONO,
		.channels_max = SST_MONO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_CAPTURE_DAI,
	.ops = &sst_media_dai_ops,
	.capture = {
		.stream_name = "Audio Capture",
		.channels_min = 1,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},

/*BE CPU  Dais */
{
	.name = "ssp0-port",
	.ops = &sst_be_dai_ops,
	.playback = {
		.stream_name = "ssp0 Tx",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp0 Rx",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "ssp1-port",
	.ops = &sst_be_dai_ops,
	.playback = {
		.stream_name = "ssp1 Tx",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp1 Rx",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "ssp2-port",
	.ops = &sst_be_dai_ops,
	.playback = {
		.stream_name = "ssp2 Tx",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp2 Rx",
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
};

static int sst_platform_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("sst_platform_open called:%s\n", dai_link->cpu_dai_name);
	if (substream->pcm->internal)
		return 0;
	runtime = substream->runtime;
	runtime->hw = sst_platform_pcm_hw;
	return 0;
}

static int sst_platform_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	pr_debug("sst_platform_close called:%s\n", dai_link->cpu_dai_name);
	return 0;
}

static int sst_platform_pcm_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	int ret_val = 0, str_id;
	struct sst_runtime_stream *stream;
	int str_cmd, status, alsa_state;

	if (substream->pcm->internal)
		return 0;
	pr_debug("sst_platform_pcm_trigger called\n");
	stream = substream->runtime->private_data;
	str_id = stream->stream_info.str_id;
	alsa_state = substream->runtime->status->state;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("Trigger Start\n");
		str_cmd = SST_SND_START;
		status = SST_PLATFORM_RUNNING;
		stream->stream_info.mad_substream = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("Trigger stop\n");
		str_cmd = SST_SND_DROP;
		status = SST_PLATFORM_DROPPED;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pr_debug("Trigger pause\n");
		str_cmd = SST_SND_PAUSE;
		status = SST_PLATFORM_PAUSED;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_debug("Trigger pause release\n");
		str_cmd = SST_SND_RESUME;
		status = SST_PLATFORM_RUNNING;
		break;
	default:
		return -EINVAL;
	}
	ret_val = stream->ops->device_control(str_cmd, &str_id);
	if (!ret_val)
		sst_set_stream_status(stream, status);

	return ret_val;
}


static snd_pcm_uframes_t sst_platform_pcm_pointer
			(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream;
	int ret_val, status;
	struct pcm_stream_info *str_info;

	stream = substream->runtime->private_data;
	status = sst_get_stream_status(stream);
	if (status == SST_PLATFORM_INIT)
		return 0;
	str_info = &stream->stream_info;
	ret_val = stream->ops->device_control(
				SST_SND_BUFFER_POINTER, str_info);
	if (ret_val) {
		pr_err("sst: error code = %d\n", ret_val);
		return ret_val;
	}
	substream->runtime->soc_delay = str_info->pcm_delay;
	return str_info->buffer_ptr;
}

static struct snd_pcm_ops sst_platform_ops = {
	.open = sst_platform_open,
	.close = sst_platform_close,
	.ioctl = snd_pcm_lib_ioctl,
	.trigger = sst_platform_pcm_trigger,
	.pointer = sst_platform_pcm_pointer,
};

static void sst_pcm_free(struct snd_pcm *pcm)
{
	pr_debug("sst_pcm_free called\n");
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

static int sst_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
	int retval = 0;

	pr_debug("sst_pcm_new called\n");
	if (dai->driver->playback.channels_min ||
			dai->driver->capture.channels_min) {
		retval =  snd_pcm_lib_preallocate_pages_for_all(pcm,
			SNDRV_DMA_TYPE_CONTINUOUS,
			snd_dma_continuous_data(GFP_DMA),
			SST_MAX_BUFFER, SST_MAX_BUFFER);
		if (retval) {
			pr_err("dma buffer allocationf fail\n");
			return retval;
		}
	}
	return retval;
}

static int sst_soc_probe(struct snd_soc_platform *platform)
{
	struct sst_data *ctx = snd_soc_platform_get_drvdata(platform);
	struct soft_platform_id spid;
	int ret = 0;

	memcpy(&spid, ctx->pdata->spid, sizeof(spid));
	pr_debug("Enter:%s\n", __func__);
	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
	    INTEL_MID_BOARD(1, TABLET, CLVT) ||
	    INTEL_MID_BOARD(1, TABLET, BYT))
		return sst_platform_clv_init(platform);
	if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
			INTEL_MID_BOARD(1, TABLET, MRFL) ||
			INTEL_MID_BOARD(1, PHONE, MOFD) ||
			INTEL_MID_BOARD(1, TABLET, MOFD)) {
		if (dpcm_enable == 1 && dfw_enable == 1)
			ret = sst_dsp_init_v2_dpcm_dfw(platform);
		else if (dpcm_enable == 1)
			ret = sst_dsp_init_v2_dpcm(platform);
		else
			ret = sst_dsp_init(platform);
		if (ret)
			return ret;
		ret = snd_soc_register_effect(platform->card, &effects_ops);
	}
	if (INTEL_MID_BOARD(1, TABLET, CHT)) {
		if (dpcm_enable == 1 && dfw_enable == 1)
			ret = sst_dsp_init_v2_dpcm_dfw(platform);
		if (dpcm_enable == 1)
			ret = sst_dsp_init_v2_dpcm(platform);
		else
			ret = sst_dsp_init(platform);
		if (ret)
			pr_err("Dsp init failed: %d\n", ret);
	}
	return ret;
}

static int sst_soc_remove(struct snd_soc_platform *platform)
{
	pr_debug("%s called\n", __func__);
	return 0;
}

static struct snd_soc_platform_driver sst_soc_platform_drv  = {
	.probe		= sst_soc_probe,
	.remove		= sst_soc_remove,
	.ops		= &sst_platform_ops,
	.compr_ops	= &sst_platform_compr_ops,
	.pcm_new	= sst_pcm_new,
	.pcm_free	= sst_pcm_free,
	.read		= sst_soc_read,
	.write		= sst_soc_write,
};

static int sst_platform_async_cb(struct sst_platform_cb_params *params)
{
	int retval = 0;
	struct snd_soc_platform *soc_platform;
	struct snd_soc_card *card;
	struct snd_kcontrol *kcontrol;
	struct sst_data *sst;

	soc_platform = snd_soc_lookup_platform(sst_pdev);
	if (!soc_platform) {
		pr_err("Platform not found\n");
		return -EINVAL;
	}

	switch (params->event) {
	case SST_PLATFORM_VTSV_READ_EVENT: {
		u8 *vtsv_result = params->params;

		sst = snd_soc_platform_get_drvdata(soc_platform);
		card = soc_platform->card;
		kcontrol = snd_soc_card_get_kcontrol(card, "vtsv event");
		if (!kcontrol) {
			pr_err("SST VTSV POLL control not found\n");
			return -EINVAL;
		}
		mutex_lock(&sst->lock);
		/* 0th index of array contains size of array */
		memcpy(sst->vtsv_result.data, vtsv_result, vtsv_result[0]);
		mutex_unlock(&sst->lock);
		snd_ctl_notify(card->snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
					&kcontrol->id);
		break;
	}

	case SST_PLATFORM_TRIGGER_RECOVERY: {
		bool *dapm_param = params->params;

		card = soc_platform->card;
		snd_soc_dapm_state_set(card, *dapm_param);
		break;
	}

	default:
		pr_info("No event handler for event Id %d\n", params->event);
	}

	return retval;
}

static struct sst_platform_cb_ops cb_ops = {
	.async_cb = sst_platform_async_cb,
};

int sst_register_dsp(struct sst_device *sst_dev)
{
	if (!sst_dev)
		return -ENODEV;
	mutex_lock(&sst_dsp_lock);
	if (sst_dsp) {
		pr_err("we already have a device %s\n", sst_dsp->name);
		mutex_unlock(&sst_dsp_lock);
		return -EEXIST;
	}
	pr_debug("registering device %s\n", sst_dev->name);
	sst_dev->cb_ops = &cb_ops;
	sst_dsp = sst_dev;
	mutex_unlock(&sst_dsp_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(sst_register_dsp);

int sst_unregister_dsp(struct sst_device *dev)
{
	if (dev != sst_dsp)
		return -EINVAL;

	mutex_lock(&sst_dsp_lock);
	if (sst_dsp) {
		pr_debug("unregister %s\n", sst_dsp->name);
		mutex_unlock(&sst_dsp_lock);
		return -EIO;
	}

	sst_dsp = NULL;
	mutex_unlock(&sst_dsp_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(sst_unregister_dsp);

static const struct snd_soc_component_driver pcm_component = {
	.name           = "pcm",
};

static int sst_platform_probe(struct platform_device *pdev)
{
	struct sst_data *sst;
	int ret;
	struct sst_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("sst_platform_probe called\n");
	sst = devm_kzalloc(&pdev->dev, sizeof(*sst), GFP_KERNEL);
	if (sst == NULL) {
		pr_err("kzalloc failed\n");
		return -ENOMEM;
	}

	if (dpcm_enable == 1) {
		pr_info("dpcm enabled; overriding stream map\n");
		pdata->pdev_strm_map = dpcm_strm_map;
		pdata->strm_map_size = ARRAY_SIZE(dpcm_strm_map);
	}
	sst_pdev = &pdev->dev;
	sst->pdata = pdata;
	mutex_init(&sst->lock);
	dev_set_drvdata(&pdev->dev, sst);

	ret = snd_soc_register_platform(&pdev->dev,
					 &sst_soc_platform_drv);
	if (ret) {
		pr_err("registering soc platform failed\n");
		return ret;
	}
	ret = snd_soc_register_component(&pdev->dev, &pcm_component,
				sst_platform_dai, ARRAY_SIZE(sst_platform_dai));
	if (ret) {
		pr_err("registering cpu dais failed\n");
		snd_soc_unregister_platform(&pdev->dev);
	}

	return ret;
}

static int sst_platform_remove(struct platform_device *pdev)
{

	snd_soc_unregister_component(&pdev->dev);
	snd_soc_unregister_platform(&pdev->dev);
	pr_debug("sst_platform_remove success\n");
	return 0;
}

static struct platform_driver sst_platform_driver = {
	.driver		= {
		.name		= "sst-platform",
		.owner		= THIS_MODULE,
	},
	.probe		= sst_platform_probe,
	.remove		= sst_platform_remove,
};

module_platform_driver(sst_platform_driver);

MODULE_DESCRIPTION("ASoC Intel(R) MID Platform driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sst-platform");
