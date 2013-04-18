/*
 *  sst_platform.c - Intel MID Platform driver
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/intel_sst_ioctl.h>
#include <asm/platform_sst_audio.h>
#include <asm/intel-mid.h>
#include "sst_platform.h"
#include "sst_platform_pvt.h"
struct sst_device *sst_dsp;
struct device *sst_pdev;

static DEFINE_MUTEX(sst_dsp_lock);

static struct snd_pcm_hardware sst_platform_pcm_hw = {
	.info =	(SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_DOUBLE |
			SNDRV_PCM_INFO_PAUSE |
			SNDRV_PCM_INFO_RESUME |
			SNDRV_PCM_INFO_MMAP|
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_BLOCK_TRANSFER |
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


int sst_set_mixer_param(unsigned int device_input_mixer)
{
	if (!sst_dsp) {
		pr_err("sst: DSP not registered\n");
		return -ENODEV;
	}

	/*allocate memory for params*/
	return sst_dsp->ops->set_generic_params(SST_SET_ALGO_PARAMS,
						(void *)&device_input_mixer);
}

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

/* helper functions */
static inline void sst_set_stream_status(struct sst_runtime_stream *stream,
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

static int sst_get_stream_mapping(int dev, int sdev, int dir,
	struct sst_dev_stream_map *map, int size, u8 pipe_id)
{
	int index;

	if (map == NULL)
		return -EINVAL;

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
				map[index].device_id = pipe_id;
				pr_debug("%s: pipe_id %d index %d", __func__, pipe_id, index);

				return index;
			}
		}
	}
	return 0;
}

static int sst_fill_stream_params(void *substream,
	const struct sst_data *ctx, struct snd_sst_params *str_params, bool is_compress)
{
	bool use_strm_map;
	int map_size;
	int index;
	struct sst_dev_stream_map *map;
	struct snd_pcm_substream *pstream = NULL;
	struct snd_compr_stream *cstream = NULL;

	use_strm_map = ctx->pdata->use_strm_map;
	map = ctx->pdata->pdev_strm_map;
	map_size = ctx->pdata->strm_map_size;

	if (is_compress == true)
		cstream = (struct snd_compr_stream *)substream;
	else
		pstream = (struct snd_pcm_substream *)substream;

	str_params->stream_type = SST_STREAM_TYPE_MUSIC;

	/* For pcm streams */
	if (pstream) {
		if (use_strm_map) {
			index = sst_get_stream_mapping(pstream->pcm->device,
						  pstream->number, pstream->stream,
						  map, map_size, ctx->pipe_id);
			if (index <= 0)
				return -EINVAL;

			str_params->stream_id = index;
			str_params->device_type = map[index].device_id;
			str_params->task = map[index].task_id;

			if (str_params->device_type == SST_PROBE_IN)
				str_params->stream_type = SST_STREAM_TYPE_PROBE;

			pr_debug("str_id = %d, device_type = %d, task = %d",
				 str_params->stream_id, str_params->device_type,
				 str_params->task);
		} else {
			if (pstream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				str_params->device_type = pstream->pcm->device + 1;
				pr_debug("Playback stream, Device %d\n",
						pstream->pcm->device);
			} else {
				str_params->device_type = SND_SST_DEVICE_CAPTURE;
				pr_debug("Capture stream, Device %d\n",
						pstream->pcm->device);
			}
		}
		str_params->ops = (u8)pstream->stream;
	}

	if (cstream) {
		if (use_strm_map) {
			/* FIXME: Add support for subdevice number in
			 * snd_compr_stream */
			index = sst_get_stream_mapping(cstream->device->device,
						       0, cstream->direction,
						       map, map_size, ctx->pipe_id);
			if (index <= 0)
				return -EINVAL;
			str_params->stream_id = index;
			str_params->device_type = map[index].device_id;
			str_params->task = map[index].task_id;
			pr_debug("compress str_id = %d, device_type = %d, task = %d",
				 str_params->stream_id, str_params->device_type,
				 str_params->task);

		}
		str_params->ops = (u8)cstream->direction;
	}
	return 0;
}

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
	/* fill the device type and stream id to pass to SST driver */
	ret_val = sst_fill_stream_params(substream, ctx, &str_params, false);
	pr_debug("platform prepare: fill stream params ret_val = 0x%x\n", ret_val);
	if (ret_val < 0)
		return ret_val;

	ret_val = stream->ops->open(&str_params);
	pr_debug("platform prepare: stream open ret_val = 0x%x\n", ret_val);
	if (ret_val <= 0)
		return ret_val;
	stream->stream_info.str_id = ret_val;
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

static void sst_media_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;
	struct sst_data *ctx = snd_soc_platform_get_drvdata(dai->platform);
	struct sst_dev_stream_map *map = ctx->pdata->pdev_strm_map;

	stream = substream->runtime->private_data;
	if (strstr(dai->name, "Power-cpu-dai"))
		ret_val = power_down_sst(stream);

	str_id = stream->stream_info.str_id;
	if (str_id)
		ret_val = stream->ops->close(str_id);

	if (strstr(dai->name, SST_PROBE_DAI)) {
		if ((map[str_id].task_id == SST_TASK_ID_MEDIA) &&
			(map[str_id].status == SST_DEV_MAP_IN_USE)) {
				pr_debug("str_id %d deviced_id %d\n", str_id, map[str_id].device_id);
				map[str_id].status = SST_DEV_MAP_FREE;
				map[str_id].device_id = PIPE_RSVD;
		}
	}

	module_put(sst_dsp->dev->driver->owner);
	kfree(stream);
	pr_debug("%s: %d\n", __func__, ret_val);
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
	snprintf(substream->pcm->id, sizeof(substream->pcm->id),
			"%d", stream->stream_info.str_id);

	ret_val = sst_platform_init_stream(substream);
	if (ret_val)
		return ret_val;
	substream->runtime->hw.info = SNDRV_PCM_INFO_BLOCK_TRANSFER;
	return ret_val;
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

static int sst_media_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	return snd_pcm_lib_free_pages(substream);
}

static struct snd_soc_dai_ops sst_media_dai_ops = {
	.startup = sst_media_open,
	.shutdown = sst_media_close,
	.prepare = sst_media_prepare,
	.hw_params = sst_media_hw_params,
	.hw_free = sst_media_hw_free,
	.set_tdm_slot = sst_platform_ihf_set_tdm_slot,
};

static struct snd_soc_dai_ops sst_aware_dai_ops = {
	.startup = sst_media_open,
	.shutdown = sst_media_close,
	.prepare = sst_media_prepare,
};
static struct snd_soc_dai_driver sst_platform_dai[] = {
{
	.name = SST_HEADSET_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_SPEAKER_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_VOICE_DAI,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_COMPRESS_DAI,
	.compress_dai = 1,
	.playback = {
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_VIRTUAL_DAI,
	.playback = {
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
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_CONTINUOUS,
	},
},
{
	.name = SST_PROBE_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 |
				SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = SST_VOIP_DAI,
	.ops = &sst_media_dai_ops,
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = SST_AWARE_DAI,
	.ops = &sst_aware_dai_ops,
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_MONO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
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

/* compress stream operations */
static void sst_compr_fragment_elapsed(void *arg)
{
	struct snd_compr_stream *cstream = (struct snd_compr_stream *)arg;

	pr_debug("fragment elapsed by driver\n");
	if (cstream)
		snd_compr_fragment_elapsed(cstream);
}

static int sst_platform_compr_open(struct snd_compr_stream *cstream)
{

	int ret_val = 0;
	struct snd_compr_runtime *runtime = cstream->runtime;
	struct sst_runtime_stream *stream;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("%s called:%s\n", __func__, dai_link->cpu_dai_name);

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	spin_lock_init(&stream->status_lock);

	/* get the sst ops */
	if (!sst_dsp || !try_module_get(sst_dsp->dev->driver->owner)) {
		pr_err("no device available to run\n");
		ret_val = -ENODEV;
		goto out_ops;
	}
	stream->compr_ops = sst_dsp->compr_ops;

	stream->id = 0;
	sst_set_stream_status(stream, SST_PLATFORM_INIT);
	runtime->private_data = stream;
	return 0;
out_ops:
	kfree(stream);
	return ret_val;
}

static int sst_platform_compr_free(struct snd_compr_stream *cstream)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;

	stream = cstream->runtime->private_data;
	/*need to check*/
	str_id = stream->id;
	if (str_id)
		ret_val = stream->compr_ops->close(str_id);
	module_put(sst_dsp->dev->driver->owner);
	kfree(stream);
	pr_debug("%s: %d\n", __func__, ret_val);
	return 0;
}

static int sst_platform_compr_set_params(struct snd_compr_stream *cstream,
					struct snd_compr_params *params)
{
	struct sst_runtime_stream *stream;
	int retval = 0;
	struct snd_sst_params str_params;
	struct sst_compress_cb cb;
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_platform *platform = rtd->platform;
	struct sst_data *ctx = snd_soc_platform_get_drvdata(platform);

	pr_debug("In function %s\n", __func__);
	stream = cstream->runtime->private_data;
	/* construct fw structure for this*/
	memset(&str_params, 0, sizeof(str_params));

	/* fill the device type and stream id to pass to SST driver */
	retval = sst_fill_stream_params(cstream, ctx, &str_params, true);
	pr_debug("compr_set_params: fill stream params ret_val = 0x%x\n", retval);
	if (retval < 0)
		return retval;

	switch (params->codec.id) {
	case SND_AUDIOCODEC_MP3: {
		str_params.codec = SST_CODEC_TYPE_MP3;
		str_params.sparams.uc.mp3_params.num_chan = params->codec.ch_in;
		str_params.sparams.uc.mp3_params.pcm_wd_sz = 16;
		break;
	}

	case SND_AUDIOCODEC_AAC: {
		str_params.codec = SST_CODEC_TYPE_AAC;
		str_params.sparams.uc.aac_params.num_chan = params->codec.ch_in;
		str_params.sparams.uc.aac_params.pcm_wd_sz = 16;
		if (params->codec.format == SND_AUDIOSTREAMFORMAT_MP4ADTS)
			str_params.sparams.uc.aac_params.bs_format =
							AAC_BIT_STREAM_ADTS;
		else if (params->codec.format == SND_AUDIOSTREAMFORMAT_RAW)
			str_params.sparams.uc.aac_params.bs_format =
							AAC_BIT_STREAM_RAW;
		else {
			pr_err("Undefined format%d\n", params->codec.format);
			return -EINVAL;
		}
		str_params.sparams.uc.aac_params.externalsr =
						params->codec.sample_rate;
		break;
	}

	default:
		pr_err("codec not supported, id =%d\n", params->codec.id);
		return -EINVAL;
	}

	str_params.aparams.ring_buf_info[0].addr  =
					virt_to_phys(cstream->runtime->buffer);
	str_params.aparams.ring_buf_info[0].size =
					cstream->runtime->buffer_size;
	str_params.aparams.sg_count = 1;
	str_params.aparams.frag_size = cstream->runtime->fragment_size;

	cb.param = cstream;
	cb.compr_cb = sst_compr_fragment_elapsed;

	retval = stream->compr_ops->open(&str_params, &cb);
	if (retval < 0) {
		pr_err("stream allocation failed %d\n", retval);
		return retval;
	}

	stream->id = retval;
	return 0;
}

static int sst_platform_compr_trigger(struct snd_compr_stream *cstream, int cmd)
{
	struct sst_runtime_stream *stream =
		cstream->runtime->private_data;

	return stream->compr_ops->control(cmd, stream->id);
}

static int sst_platform_compr_pointer(struct snd_compr_stream *cstream,
					struct snd_compr_tstamp *tstamp)
{
	struct sst_runtime_stream *stream;

	stream  = cstream->runtime->private_data;
	stream->compr_ops->tstamp(stream->id, tstamp);
	tstamp->byte_offset = tstamp->copied_total %
				 (u32)cstream->runtime->buffer_size;
	pr_debug("calc bytes offset/copied bytes as %d\n", tstamp->byte_offset);
	return 0;
}

static int sst_platform_compr_ack(struct snd_compr_stream *cstream,
					size_t bytes)
{
	struct sst_runtime_stream *stream;

	stream  = cstream->runtime->private_data;
	stream->compr_ops->ack(stream->id, (unsigned long)bytes);
	stream->bytes_written += bytes;

	return 0;
}

static int sst_platform_compr_get_caps(struct snd_compr_stream *cstream,
					struct snd_compr_caps *caps)
{
	struct sst_runtime_stream *stream =
		cstream->runtime->private_data;

	return stream->compr_ops->get_caps(caps);
}

static int sst_platform_compr_get_codec_caps(struct snd_compr_stream *cstream,
					struct snd_compr_codec_caps *codec)
{
	struct sst_runtime_stream *stream =
		cstream->runtime->private_data;

	return stream->compr_ops->get_codec_caps(codec);
}

static int sst_platform_compr_set_metadata(struct snd_compr_stream *cstream,
					struct snd_compr_metadata *metadata)
{
	struct sst_runtime_stream *stream  =
		 cstream->runtime->private_data;

	return stream->compr_ops->set_metadata(stream->id, metadata);
}

static struct snd_compr_ops sst_platform_compr_ops = {

	.open = sst_platform_compr_open,
	.free = sst_platform_compr_free,
	.set_params = sst_platform_compr_set_params,
	.set_metadata = sst_platform_compr_set_metadata,
	.trigger = sst_platform_compr_trigger,
	.pointer = sst_platform_compr_pointer,
	.ack = sst_platform_compr_ack,
	.get_caps = sst_platform_compr_get_caps,
	.get_codec_caps = sst_platform_compr_get_codec_caps,
};

static int __devinit sst_soc_probe(struct snd_soc_platform *platform)
{
	struct sst_data *ctx = snd_soc_platform_get_drvdata(platform);
	struct soft_platform_id spid;

	memcpy(&spid, ctx->pdata->spid, sizeof(spid));
	pr_debug("Enter:%s\n", __func__);
#ifdef CONFIG_PRH_TEMP_WA_FOR_SPID
	return sst_platform_clv_init(platform);
#else
	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
	    INTEL_MID_BOARD(1, TABLET, CLVT) ||
	    INTEL_MID_BOARD(1, TABLET, BYT))
		return sst_platform_clv_init(platform);
	if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
	    INTEL_MID_BOARD(1, TABLET, MRFL))
		return sst_dsp_init(platform);
	return 0;
#endif
}

static int sst_soc_remove(struct snd_soc_platform *platform)
{
	pr_debug("%s called\n", __func__);
	return 0;
}

static struct snd_soc_platform_driver sst_soc_platform_drv  __devinitdata = {
	.probe		= sst_soc_probe,
	.remove		= sst_soc_remove,
	.ops		= &sst_platform_ops,
	.compr_ops	= &sst_platform_compr_ops,
	.pcm_new	= sst_pcm_new,
	.pcm_free	= sst_pcm_free,
	.read		= sst_soc_read,
	.write		= sst_soc_write,
};
int sst_fill_config_data(struct sst_data *sst)
{
	int len;
	char *platform_data;
	struct sst_platform_data *sst_pdata = sst->pdata;

	pr_debug("%s called\n", __func__);
	len = sizeof(*(sst_pdata->bdata)) + sizeof(*(sst_pdata->pdata));
	platform_data = devm_kzalloc(sst_pdev,
					(len + sizeof(u32)), GFP_KERNEL);
	if (platform_data == NULL) {
		pr_err("kzalloc failed\n");
		return -ENOMEM;
	}
	memcpy(platform_data, &len, sizeof(len));
	memcpy(platform_data + sizeof(int), sst_pdata->bdata,
					sizeof(*(sst_pdata->bdata)));
	memcpy(platform_data + sizeof(int) + sizeof(*(sst_pdata->bdata)),
					sst_pdata->pdata, sizeof(*(sst_pdata->pdata)));
	sst_dsp->ops->set_generic_params(SST_SET_SSP_CONFIG, platform_data);

	return 0;
}

int sst_register_dsp(struct sst_device *sst_dev)
{

	struct sst_data *sst;
	struct sst_platform_data *sst_pdata;

	if (!sst_pdev)
		return -ENODEV;
	sst =  dev_get_drvdata(sst_pdev);
	sst_pdata = sst->pdata;

	if (!sst_dev)
		return -ENODEV;
	mutex_lock(&sst_dsp_lock);
	if (sst_dsp) {
		pr_err("we already have a device %s\n", sst_dsp->name);
		mutex_unlock(&sst_dsp_lock);
		return -EEXIST;
	}
	pr_debug("registering device %s\n", sst_dev->name);

	sst_dsp = sst_dev;
	if (!(sst_pdata->bdata == NULL) && !(sst_pdata->pdata == NULL))
		sst_fill_config_data(sst);
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

static int __devinit sst_platform_probe(struct platform_device *pdev)
{
	struct sst_data *sst;
	int ret;
	struct sst_platform_data *pdata = pdev->dev.platform_data;

	pr_debug("sst_platform_probe called\n");
	sst = devm_kzalloc(&pdev->dev, sizeof(*sst), GFP_KERNEL);
	if (sst == NULL) {
		pr_err("kzalloc failed\n");
		return -ENOMEM;
	};
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
	ret = snd_soc_register_dais(&pdev->dev,
				sst_platform_dai, ARRAY_SIZE(sst_platform_dai));
	if (ret) {
		pr_err("registering cpu dais failed\n");
		snd_soc_unregister_platform(&pdev->dev);
	}

	return ret;
}

static int sst_platform_remove(struct platform_device *pdev)
{

	snd_soc_unregister_dais(&pdev->dev, ARRAY_SIZE(sst_platform_dai));
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

static int __init sst_soc_platform_init(void)
{
	pr_debug("sst_soc_platform_init called\n");
	return  platform_driver_register(&sst_platform_driver);
}
module_init(sst_soc_platform_init);

static void __exit sst_soc_platform_exit(void)
{
	platform_driver_unregister(&sst_platform_driver);
	pr_debug("sst_soc_platform_exit success\n");
}
module_exit(sst_soc_platform_exit);

MODULE_DESCRIPTION("ASoC Intel(R) MID Platform driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_AUTHOR("Harsha Priya <priya.harsha@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:sst-platform");
