/*
 *  ctp_vb_cs42l73.c - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2012-13 Intel Corp
 *  Author: KP Jeeja<jeeja.kp@intel.com>
 *  Author: Dharageswari.R<dharageswari.r@intel.com>
 *  Author: Subhransu Prusty S<subhranshu.s.prusty@intel.com>
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
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/intel_sst_ctp.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/cs42l73.h"
#include "ctp_common.h"

/* As per the codec spec the mic2_sdet debounce delay is 20ms.
 * But having 20ms delay doesn't work */
#define MIC2SDET_DEBOUNCE_DELAY 50 /* 50 ms */
#define MICBIAS_NAME	"MIC2 Bias"

/* CS42L73 widgets */
static const struct snd_soc_dapm_widget ctp_vb_dapm_widgets[] = {

	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", ctp_amp_event),
};

/* CS42L73 Audio Map */
static const struct snd_soc_dapm_route ctp_vb_audio_map[] = {
	{"MIC2", NULL, "Headset Mic"},
	/* Headphone (L+R)->  HPOUTA, HPOUTB */
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
	{"Ext Spk", NULL, "SPKLINEOUT"},
	{"Ext Spk", NULL, "SPKOUT"},
};

static int ctp_vb_cs42l73_startup(struct snd_pcm_substream *substream)
{
	unsigned int device = substream->pcm->device;
	pr_debug("%s - applying rate constraint\n", __func__);
	switch (device) {
	case CTP_VB_AUD_ASP_DEV:
	case CTP_VB_AUD_PROBE_DEV:
	case CTP_VB_COMMS_FM_DEV:
	case CTP_VB_AUD_VIRTUAL_ASP_DEV:
		snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &constraints_48000);
		break;
	case CTP_VB_AUD_VSP_DEV:
		snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_16000);
		ctp_config_voicecall_flag(substream, true);
		break;
	case CTP_VB_COMMS_BT_SCO_DEV:
		snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_8000_16000);
		break;
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
	return 0;
}

static int ctp_vb_cs42l73_shutdown(struct snd_pcm_substream *substream)
{
	unsigned int device = substream->pcm->device;
	switch (device) {
	case CTP_VB_AUD_VSP_DEV:
		ctp_config_voicecall_flag(substream, false);
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
	return 0;
}

static int ctp_vb_cs42l73_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int device = substream->pcm->device;
	struct ctp_clk_fmt clk_fmt;

	switch (device) {
	case CTP_VB_AUD_ASP_DEV:
	case CTP_VB_AUD_VSP_DEV:
	case CTP_VB_COMMS_FM_DEV:
	case CTP_VB_AUD_VIRTUAL_ASP_DEV:
		clk_fmt.clk_id = CS42L73_CLKID_MCLK1;
		clk_fmt.freq = DEFAULT_MCLK;
		clk_fmt.dir = SND_SOC_CLOCK_IN;
		/* Slave mode */
		clk_fmt.fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS;
		return ctp_set_clk_fmt(codec_dai, &clk_fmt);
	case CTP_VB_COMMS_BT_SCO_DEV:
		clk_fmt.clk_id = CS42L73_CLKID_MCLK1;
		clk_fmt.freq = DEFAULT_MCLK;
		clk_fmt.dir = SND_SOC_CLOCK_IN;
		/* Slave mode */
		clk_fmt.fmt = SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF |
					SND_SOC_DAIFMT_CBS_CFS;
		return ctp_set_clk_fmt(codec_dai, &clk_fmt);
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
}

static int ctp_vb_cs42l73_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int device = cstream->device->device;
	struct ctp_clk_fmt clk_fmt;

	switch (device) {
	case CTP_VB_AUD_COMP_ASP_DEV:
		clk_fmt.clk_id = CS42L73_CLKID_MCLK1;
		clk_fmt.freq = DEFAULT_MCLK;
		clk_fmt.dir = SND_SOC_CLOCK_IN;
		/* Slave mode */
		clk_fmt.fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS;
		return ctp_set_clk_fmt(codec_dai, &clk_fmt);
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
}

int ctp_vb_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;
	struct snd_soc_card *card = runtime->card;

	/* Set codec bias level */
	ctp_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* Add Jack specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, ctp_vb_dapm_widgets,
					ARRAY_SIZE(ctp_vb_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Jack specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, ctp_vb_audio_map,
					ARRAY_SIZE(ctp_vb_audio_map));

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "EAROUT");
	snd_soc_dapm_ignore_suspend(dapm, "Ext Spk");
	snd_soc_dapm_ignore_suspend(dapm, "Headphone");
	snd_soc_dapm_ignore_suspend(dapm, "Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "DMICA");
	snd_soc_dapm_ignore_suspend(dapm, "DMICB");

	snd_soc_dapm_disable_pin(dapm, "MIC2");
	snd_soc_dapm_disable_pin(dapm, "SPKLINEOUT");
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);

	return ret;
}


static struct snd_soc_ops ctp_vb_asp_ops = {
	.startup = ctp_vb_cs42l73_startup,
	.hw_params = ctp_vb_cs42l73_hw_params,
};

static struct snd_soc_compr_ops ctp_vb_asp_compr_ops = {
	.set_params = ctp_vb_cs42l73_set_params,
};

static struct snd_soc_ops ctp_vb_vsp_ops = {
	.startup = ctp_vb_cs42l73_startup,
	.hw_params = ctp_vb_cs42l73_hw_params,
	.shutdown = ctp_vb_cs42l73_shutdown,
};
static struct snd_soc_ops ctp_vb_bt_xsp_ops = {
	.startup = ctp_vb_cs42l73_startup,
	.hw_params = ctp_vb_cs42l73_hw_params,
};

static struct snd_soc_ops ctp_vb_fm_xsp_ops = {
	.startup = ctp_vb_cs42l73_startup,
	.hw_params = ctp_vb_cs42l73_hw_params,
};

static struct snd_soc_ops ctp_probe_ops = {
	.startup = ctp_vb_cs42l73_startup,
};

static struct snd_soc_dai_link ctp_vb_dailink[] = {
	[CTP_VB_AUD_ASP_DEV] = {
		.name = "Cloverview ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = snd_ctp_init,
		.ignore_suspend = 1,
		.ops = &ctp_vb_asp_ops,
		.playback_count = 2,
	},
	[CTP_VB_AUD_VSP_DEV] = {
		.name = "Cloverview VSP",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-vsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_vb_vsp_ops,
	},

	[CTP_VB_AUD_COMP_ASP_DEV] = {
		.name = "Cloverview Comp ASP",
		.stream_name = "Compress-Audio",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_vb_asp_compr_ops,
	},
	[CTP_VB_COMMS_BT_SCO_DEV] = {
		.name = "Cloverview BT XSP",
		.stream_name = "BT-Audio",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-xsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_vb_bt_xsp_ops,
	},
	[CTP_VB_COMMS_FM_DEV] = {
		.name = "Cloverview FM XSP",
		.stream_name = "FM-Audio",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-xsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_vb_fm_xsp_ops,
	},
	[CTP_VB_AUD_PROBE_DEV] = {
		.name = "Cloverview Probe",
		.stream_name = "CTP Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.init = NULL,
		.ops = &ctp_probe_ops,
	},
	[CTP_VB_AUD_VIRTUAL_ASP_DEV] = {
		.name = "Cloverview virtual-ASP",
		.stream_name = "virtual-stream",
		.cpu_dai_name = "Virtual-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &ctp_vb_asp_ops,
	},

};

int vb_hp_detection(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int enable)
{
	return cs42l73_hp_detection(codec, jack, enable);
}

int vb_bp_detection(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int enable)
{
	return cs42l73_bp_detection(codec, jack, enable);
}

void vb_mclk_switch(struct device *dev, bool mode)
{
	cs42l73_mclk_switch(dev, mode);
}

int vb_dai_link(struct snd_soc_card *card)
{
	card->dai_link = ctp_vb_dailink;
	card->num_links = ARRAY_SIZE(ctp_vb_dailink);
	return 0;
}

static void ctp_vb_card_name(struct snd_soc_card *card)
{
	card->name = "cloverview_audio";
}

struct snd_soc_machine_ops ctp_vb_cs42l73_ops = {
	.card_name = ctp_vb_card_name,
	.ctp_init = ctp_vb_init,
	.dai_link = vb_dai_link,
	.bp_detection = vb_bp_detection,
	.hp_detection = vb_hp_detection,
	.mclk_switch = vb_mclk_switch,
	.jack_support = true,
	.micsdet_debounce = MIC2SDET_DEBOUNCE_DELAY,
	.mic_bias = MICBIAS_NAME,
};

MODULE_DESCRIPTION("ASoC Intel(R) Cloverview MID Machine driver");
MODULE_AUTHOR("Jeeja KP<jeeja.kp@intel.com>");
MODULE_AUTHOR("Dharageswari R<dharageswari.r@intel.com>");
MODULE_AUTHOR("Subhransu Prusty S<subhranshu.s.prusty@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("ipc:ctpcs42l73-audio");
