/*
 *  sst_platform.c - Intel MID Platform driver
 *
 *  Copyright (C) 2010 Intel Corp
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

#include <linux/slab.h>
#include <linux/io.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/intel_sst_ioctl.h>
#include <sound/intel_sst.h>
#include "../codecs/sn95031.h"
#include "sst_platform.h"

#define SST_VOICE_DAI "Voice-cpu-dai"

static struct sst_platform_ctx *sst_cpu_ctx;

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

static int sst_platform_ihf_set_tdm_slot(struct snd_soc_dai *dai,
			unsigned int tx_mask, unsigned int rx_mask,
			int slots, int slot_width) {
	struct intel_sst_card_ops *sstdrv_ops;
	struct snd_sst_runtime_params params_data;
	int channels = slots;
	int ret_val;

	/* allocate memory for SST API set */
	sstdrv_ops = kzalloc(sizeof(*sstdrv_ops), GFP_KERNEL);
	if (!sstdrv_ops)
		return -ENOMEM;
	/* registering with SST driver to get access to SST APIs to use */
	ret_val = register_sst_card(sstdrv_ops);
	if (ret_val) {
		pr_err("sst: sst card registration failed\n");
		return -EIO;
	}
	params_data.type = SST_SET_CHANNEL_INFO;
	params_data.str_id = SND_SST_DEVICE_IHF;
	params_data.size = sizeof(channels);
	params_data.addr = &channels;
	ret_val = sstdrv_ops->pcm_control->set_generic_params(SST_SET_RUNTIME_PARAMS,
							(void *)&params_data);
	kfree(sstdrv_ops);
	return ret_val;
}

static const struct snd_soc_dai_ops sst_ihf_ops = {
	.set_tdm_slot = sst_platform_ihf_set_tdm_slot,
};

/* MFLD - MSIC */
static struct snd_soc_dai_driver sst_platform_dai[] = {
{
	.name = "Headset-cpu-dai",
	.playback = {
		.channels_min = SST_STEREO,
		.channels_max = SST_STEREO,
/**FIXME ***/
#if (defined(CONFIG_SND_MFLD_MACHINE) || (CONFIG_SND_MFLD_MACHINE_GI))
		.rates = SNDRV_PCM_RATE_44100,
#else
		.rates = SNDRV_PCM_RATE_48000,
#endif
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = 1,
		.channels_max = 2,
#if (defined(CONFIG_SND_MFLD_MACHINE) || (CONFIG_SND_MFLD_MACHINE_GI))
		.rates = SNDRV_PCM_RATE_44100,
#else
		.rates = SNDRV_PCM_RATE_48000,
#endif
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "Speaker-cpu-dai",
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
#if (defined(CONFIG_SND_MFLD_MACHINE) || (CONFIG_SND_MFLD_MACHINE_GI))
		.rates = SNDRV_PCM_RATE_44100,
#else
		.rates = SNDRV_PCM_RATE_48000,
#endif
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &sst_ihf_ops,
},
{
	.name = "Vibra1-cpu-dai",
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_MONO,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "Vibra2-cpu-dai",
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_MONO,
		.rates = SNDRV_PCM_RATE_44100,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "Voice-cpu-dai",
	.playback = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.channels_min = SST_MONO,
		.channels_max = SST_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
};

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

static void sst_fill_pcm_params(struct snd_pcm_substream *substream,
				struct snd_sst_stream_params *param)
{

	param->uc.pcm_params.codec = SST_CODEC_TYPE_PCM;
	param->uc.pcm_params.num_chan = (u8) substream->runtime->channels;
	param->uc.pcm_params.pcm_wd_sz = substream->runtime->sample_bits;
	param->uc.pcm_params.reserved = 0;
	param->uc.pcm_params.sfreq = substream->runtime->rate;
	param->uc.pcm_params.ring_buffer_size =
					snd_pcm_lib_buffer_bytes(substream);
	param->uc.pcm_params.period_count = substream->runtime->period_size;
	param->uc.pcm_params.ring_buffer_addr =
				virt_to_phys(substream->dma_buffer.area);
	pr_debug("period_cnt = %d\n", param->uc.pcm_params.period_count);
	pr_debug("sfreq= %d, wd_sz = %d\n",
		 param->uc.pcm_params.sfreq, param->uc.pcm_params.pcm_wd_sz);
}

static int sst_platform_alloc_stream(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream =
			substream->runtime->private_data;
	struct snd_sst_stream_params param = {{{0,},},};
	struct snd_sst_params str_params = {0};
	int ret_val;

	/* set codec params and inform SST driver the same */
	sst_fill_pcm_params(substream, &param);
	substream->runtime->dma_area = substream->dma_buffer.area;
	str_params.sparams = param;
	str_params.codec =  param.uc.pcm_params.codec;
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		str_params.ops = STREAM_OPS_PLAYBACK;
		str_params.device_type = substream->pcm->device + 1;
		pr_debug("Playbck stream,Device %d\n",
					substream->pcm->device);
	} else {
		str_params.ops = STREAM_OPS_CAPTURE;
		str_params.device_type = SND_SST_DEVICE_CAPTURE;
		pr_debug("Capture stream,Device %d\n",
					substream->pcm->device);
	}
	ret_val = stream->sstdrv_ops->pcm_control->open(&str_params);
	pr_debug("SST_SND_PLAY/CAPTURE ret_val = %x\n", ret_val);
	if (ret_val <= 0)
		return ret_val;

	stream->stream_info.str_id = ret_val;
	pr_debug("str id :  %d\n", stream->stream_info.str_id);

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
	ret_val = stream->sstdrv_ops->pcm_control->device_control(
			SST_SND_STREAM_INIT, &stream->stream_info);
	if (ret_val)
		pr_err("control_set ret error %d\n", ret_val);
	return ret_val;

}
/* end -- helper functions */

static int sst_platform_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct sst_runtime_stream *stream;
	int ret_val = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("sst_platform_open called:%s\n", dai_link->cpu_dai_name);

	runtime = substream->runtime;
	runtime->hw = sst_platform_pcm_hw;
	if (!strcmp(dai_link->cpu_dai_name, SST_VOICE_DAI)) {
		if (sst_cpu_ctx->active_nonvoice_cnt > 0) {
			pr_err("music/vibra dai is active, voice is not allowed\n");
			return -EBUSY;
		}
		sst_cpu_ctx->active_voice_cnt++;
/**FIXME in clean up patch***/
#if (defined(CONFIG_SND_MFLD_MACHINE) || (CONFIG_SND_MFLD_MACHINE_GI))

		ret_val = intel_sst_set_pll(true, SST_PLL_VOICE);
#endif
		if (!ret_val)
			return snd_pcm_hw_constraint_integer(runtime,
				SNDRV_PCM_HW_PARAM_PERIODS);
		else
			return ret_val;
	}

	if (sst_cpu_ctx->active_voice_cnt > 0) {
		pr_err("Voice dai is active, no other stream allowed\n");
		return -EBUSY;
	}

	stream = kzalloc(sizeof(*stream), GFP_KERNEL);
	if (!stream)
		return -ENOMEM;

	spin_lock_init(&stream->status_lock);
	stream->stream_info.str_id = 0;
	sst_set_stream_status(stream, SST_PLATFORM_INIT);
	stream->stream_info.mad_substream = substream;
	/* allocate memory for SST API set */
	stream->sstdrv_ops = kzalloc(sizeof(*stream->sstdrv_ops),
							GFP_KERNEL);
	if (!stream->sstdrv_ops) {
		pr_err("sst: mem allocation for ops fail\n");
		ret_val = -ENOMEM;
		goto out_ops;
	}
	stream->sstdrv_ops->vendor_id = MSIC_VENDOR_ID;
	/* registering with SST driver to get access to SST APIs to use */
	ret_val = register_sst_card(stream->sstdrv_ops);
	if (ret_val) {
		pr_err("sst: sst card registration failed\n");
		goto out_reg_sst;
	}
	runtime->private_data = stream;
	sst_cpu_ctx->active_nonvoice_cnt++;

	/* Make sure, that the period size is always even */
	snd_pcm_hw_constraint_step(substream->runtime, 0,
			   SNDRV_PCM_HW_PARAM_PERIODS, 2);

	return snd_pcm_hw_constraint_integer(runtime,
			 SNDRV_PCM_HW_PARAM_PERIODS);

out_reg_sst:
	kfree(stream->sstdrv_ops);
out_ops:
	kfree(stream);
	return ret_val;
}

static int sst_platform_close(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	pr_debug("sst_platform_close called:%s\n", dai_link->cpu_dai_name);

	if (!strcmp(dai_link->cpu_dai_name, SST_VOICE_DAI)) {
		sst_cpu_ctx->active_voice_cnt--;
#if (defined(CONFIG_SND_MFLD_MACHINE) || (CONFIG_SND_MFLD_MACHINE_GI))
		intel_sst_set_pll(false, SST_PLL_VOICE);
#endif
		goto func_exit;
	}
	sst_cpu_ctx->active_nonvoice_cnt--;
	stream = substream->runtime->private_data;
	str_id = stream->stream_info.str_id;
	if (str_id) {
		if (stream->stream_status == SST_PLATFORM_SUSPENDED) {
			pr_debug("device suspended resume for closure\n");
			ret_val = stream->sstdrv_ops->pcm_control->
				device_control(SST_SND_DEVICE_RESUME_SYNC,
				&str_id);
			if (!ret_val)
				sst_set_stream_status(stream,
					SST_PLATFORM_DROPPED);
		}
		ret_val = stream->sstdrv_ops->pcm_control->close(str_id);
	}
	unregister_sst_card(stream->sstdrv_ops);
	kfree(stream->sstdrv_ops);
	kfree(stream);
func_exit:
/**FIXME ***/
#if (defined(CONFIG_SND_MFLD_MACHINE) || (CONFIG_SND_MFLD_MACHINE_GI))
	if (!sst_cpu_ctx->active_nonvoice_cnt)
		snd_soc_dai_set_tristate(codec_dai, 1);
#endif
	return ret_val;
}

static int sst_platform_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct sst_runtime_stream *stream;
	int ret_val = 0, str_id;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	pr_debug("sst_platform_pcm_prepare called\n");

	if (!strcmp(dai_link->cpu_dai_name, SST_VOICE_DAI)) {
		pr_debug("pcm_preare for Voice, returning.\n");
		return ret_val;
	}
	stream = substream->runtime->private_data;
	str_id = stream->stream_info.str_id;
	if (stream->stream_info.str_id)
		return ret_val;

	ret_val = sst_platform_alloc_stream(substream);
	if (ret_val <= 0)
		goto out;
	snprintf(substream->pcm->id, sizeof(substream->pcm->id),
			"%d", stream->stream_info.str_id);

	ret_val = sst_platform_init_stream(substream);
	if (ret_val)
		return ret_val;
	substream->runtime->hw.info = SNDRV_PCM_INFO_BLOCK_TRANSFER;


out:
	/* we are suspending the sink/source here as we want to have
	 * device suspended in idle, before handling next request we will send
	 * an explict RESUME call  */
	pr_debug("Suspend NOW\n");
	stream->sstdrv_ops->pcm_control->device_control(
			SST_SND_DEVICE_SUSPEND, &stream->stream_info.str_id);
	sst_set_stream_status(stream, SST_PLATFORM_SUSPENDED);
	return ret_val;
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

	if (stream->stream_status == SST_PLATFORM_SUSPENDED) {
		pr_debug("in trigger, device is suspended, so resume it\n");
		ret_val = stream->sstdrv_ops->pcm_control->device_control(
					SST_SND_DEVICE_RESUME, &str_id);
		if (!ret_val)
			sst_set_stream_status(stream, SST_PLATFORM_DROPPED);
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		pr_debug("sst: Trigger Start\n");
		str_cmd = SST_SND_START;
		status = SST_PLATFORM_RUNNING;
		stream->stream_info.mad_substream = substream;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		pr_debug("sst: in stop\n");
		str_cmd = SST_SND_DROP;
		status = SST_PLATFORM_DROPPED;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		pr_debug("sst: in pause\n");
		str_cmd = SST_SND_PAUSE;
		status = SST_PLATFORM_PAUSED;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		pr_debug("sst: in pause release\n");
		str_cmd = SST_SND_RESUME;
		status = SST_PLATFORM_RUNNING;
		break;
	default:
		return -EINVAL;
	}
	ret_val = stream->sstdrv_ops->pcm_control->device_control(str_cmd,
								&str_id);
	if (!ret_val)
		sst_set_stream_status(stream, status);

	if (cmd == SNDRV_PCM_TRIGGER_STOP &&
				alsa_state == SNDRV_PCM_STATE_DRAINING) {
		ret_val = stream->sstdrv_ops->pcm_control->device_control(
					SST_SND_DEVICE_SUSPEND, &str_id);
		if (!ret_val)
			sst_set_stream_status(stream, SST_PLATFORM_SUSPENDED);
	}
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
	ret_val = stream->sstdrv_ops->pcm_control->device_control(
				SST_SND_BUFFER_POINTER, str_info);
	if (ret_val) {
		pr_err("sst: error code = %d\n", ret_val);
		return ret_val;
	}
	substream->runtime->delay = stream->stream_info.pcm_delay;
	return stream->stream_info.buffer_ptr;
}

static int sst_platform_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	int ret;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

/**FIXME -Need to move all codec related stuff to Machine driver ***/
#if (defined(CONFIG_SND_MFLD_MACHINE) || (CONFIG_SND_MFLD_MACHINE_GI))
	if (strcmp(rtd->dai_link->cpu_dai_name, SST_VOICE_DAI)) {
		/* Force the data width to 24 bit in MSIC. Post Processing
		algorithms in DSP enabled with 24 bit precision */
		ret = snd_soc_codec_set_params(codec, SNDRV_PCM_FORMAT_S24_LE);
		if (ret < 0) {
			pr_debug("codec set_params returned error %d\n", ret);
			return ret;
		}
	}
	/*last two parameters have to non-zero, otherwise pll gets disabled*/
	snd_soc_dai_set_pll(codec_dai, 0, SST_CLK_UNINIT, 1,
							params_rate(params));
#endif
	snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	memset(substream->runtime->dma_area, 0, params_buffer_bytes(params));

	return 0;
}

static int sst_platform_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_ops sst_platform_ops = {
	.open = sst_platform_open,
	.close = sst_platform_close,
	.ioctl = snd_pcm_lib_ioctl,
	.prepare = sst_platform_pcm_prepare,
	.trigger = sst_platform_pcm_trigger,
	.pointer = sst_platform_pcm_pointer,
	.hw_params = sst_platform_pcm_hw_params,
	.hw_free = sst_platform_pcm_hw_free,
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
			snd_dma_continuous_data(GFP_KERNEL),
			SST_MAX_BUFFER, SST_MAX_BUFFER);
		if (retval) {
			pr_err("dma buffer allocationf fail\n");
			return retval;
		}
	}
	return retval;
}
static struct snd_soc_platform_driver sst_soc_platform_drv = {
	.ops		= &sst_platform_ops,
	.pcm_new	= sst_pcm_new,
	.pcm_free	= sst_pcm_free,
};

static int sst_platform_probe(struct platform_device *pdev)
{
	int ret;

	pr_debug("sst_platform_probe called\n");
	ret = snd_soc_register_platform(&pdev->dev, &sst_soc_platform_drv);
	if (ret) {
		pr_err("registering soc platform failed\n");
		return ret;
	}

	sst_cpu_ctx = kzalloc(sizeof(*sst_cpu_ctx), GFP_KERNEL);
	if (!sst_cpu_ctx) {
		pr_err("memory alloc failed for cpu_dais context\n");
		snd_soc_unregister_platform(&pdev->dev);
		return -ENOMEM;
	}

	ret = snd_soc_register_dais(&pdev->dev,
				sst_platform_dai, ARRAY_SIZE(sst_platform_dai));
	if (ret) {
		pr_err("registering cpu dais failed\n");
		snd_soc_unregister_platform(&pdev->dev);
		kfree(sst_cpu_ctx);
	}
	return ret;
}

static int sst_platform_remove(struct platform_device *pdev)
{

	snd_soc_unregister_dais(&pdev->dev, ARRAY_SIZE(sst_platform_dai));
	snd_soc_unregister_platform(&pdev->dev);
	kfree(sst_cpu_ctx);
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
