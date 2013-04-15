/*
 *  merr_bb_cs42l73.c - ASoc Machine driver for Intel Merrfield Bodega Bay MID platform
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Omair M Abdullah <omair.m.abdullah@intel.com>
 * 	Ramesh Babu K V <Ramesh.Babu@intel.com>
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
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_sst_ctp.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "ctp_common.h"
#include "../codecs/cs42l73.h"

#define GPIO_AMP_ON		0x30
#define GPIO_AMP_OFF		0x31
#define GPIO2CTLO		0x80

/* As per the codec spec the mic2_sdet debounce delay is 20ms.
 * But having 20ms delay doesn't work */
#define MIC2SDET_DEBOUNCE_DELAY 50 /* 50 ms */
#define MICBIAS_NAME	"MIC2 Bias"

int merr_bb_amp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	int ret;

	if (SND_SOC_DAPM_EVENT_ON(event)) {
		/*Enable  IHFAMP_SD_N  GPIO */
		ret = intel_scu_ipc_iowrite8(GPIO2CTLO, GPIO_AMP_ON);
		if (ret)
			pr_err("write of  failed, err %d\n", ret);
	} else {
		/*Disable  IHFAMP_SD_N  GPIO */
		ret = intel_scu_ipc_iowrite8(GPIO2CTLO, GPIO_AMP_OFF);
		if (ret)
			pr_err("write of  failed, err %d\n", ret);
	}
	return 0;
}

static const struct snd_soc_dapm_widget merr_bb_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", merr_bb_amp_event),
};

static const struct snd_soc_dapm_route merr_bb_audio_map[] = {
	{"MIC1", NULL, "Headset Mic"},
	/* Headphone (LR)->  HPOUTA, HPOUTB */
	{"Headphone", NULL, "HPOUTA"},
	{"Headphone", NULL, "HPOUTB"},
	{"Ext Spk", NULL, "SPKLINEOUT"},
	{"Ext Spk", NULL, "SPKOUT"},
};

static int merr_bb_cs42l73_startup(struct snd_pcm_substream *substream)
{
	unsigned int device = substream->pcm->device;
	pr_debug("%s - applying rate constraint\n", __func__);
	switch (device) {
	case MERR_BB_AUD_ASP_DEV:
	case MERR_BB_AUD_PROBE_DEV:
	case MERR_BB_COMMS_FM_DEV:
	case MERR_BB_AUD_VIRTUAL_ASP_DEV:
		snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &constraints_48000);
		break;
	case MERR_BB_AUD_VSP_DEV:
		snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&constraints_16000);
		ctp_config_voicecall_flag(substream, true);
		break;
	case MERR_BB_COMMS_BT_SCO_DEV:
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

static int merr_bb_cs42l73_shutdown(struct snd_pcm_substream *substream)
{
	unsigned int device = substream->pcm->device;
	switch (device) {
	case MERR_BB_AUD_VSP_DEV:
		ctp_config_voicecall_flag(substream, false);
	default:
		pr_err("Invalid device\n");
		return -EINVAL;
	}
	return 0;
}

static int merr_bb_cs42l73_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int device = substream->pcm->device;
	struct ctp_clk_fmt clk_fmt;

	switch (device) {
	case MERR_BB_AUD_ASP_DEV:
	case MERR_BB_AUD_VSP_DEV:
	case MERR_BB_COMMS_BT_SCO_DEV:
	case MERR_BB_COMMS_FM_DEV:
	case MERR_BB_AUD_VIRTUAL_ASP_DEV:
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

static struct snd_soc_ops merr_bb_asp_ops = {
	.startup = merr_bb_cs42l73_startup,
	.hw_params = merr_bb_cs42l73_hw_params,
};

static struct snd_soc_ops merr_bb_vsp_ops = {
	.startup = merr_bb_cs42l73_startup,
	.hw_params = merr_bb_cs42l73_hw_params,
	.shutdown = merr_bb_cs42l73_shutdown,
};
static struct snd_soc_ops merr_bb_bt_xsp_ops = {
	.startup = merr_bb_cs42l73_startup,
	.hw_params = merr_bb_cs42l73_hw_params,
};

static struct snd_soc_ops merr_bb_fm_xsp_ops = {
	.startup = merr_bb_cs42l73_startup,
	.hw_params = merr_bb_cs42l73_hw_params,
};

static struct snd_soc_ops merr_bb_probe_ops = {
	.startup = merr_bb_cs42l73_startup,
};

struct snd_soc_dai_link merr_bb_dailink[] = {
	[MERR_BB_AUD_ASP_DEV] = {
		.name = "Merrifield BB ASP",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = snd_ctp_init,
		.ignore_suspend = 1,
		.ops = &merr_bb_asp_ops,
	},
	[MERR_BB_AUD_VSP_DEV] = {
		.name = "Merrifield BB VSP",
		.stream_name = "Voice",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-vsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &merr_bb_vsp_ops,
	},
	[MERR_BB_COMMS_BT_SCO_DEV] = {
		.name = "Merrifield BB BT XSP",
		.stream_name = "BT-Audio",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-xsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &merr_bb_bt_xsp_ops,
	},
	[MERR_BB_COMMS_FM_DEV] = {
		.name = "Merrifield BB FM XSP",
		.stream_name = "FM-Audio",
		.cpu_dai_name = "Voice-cpu-dai",
		.codec_dai_name = "cs42l73-xsp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &merr_bb_fm_xsp_ops,
	},
	[MERR_BB_AUD_PROBE_DEV] = {
		.name = "Merrifield BB Probe",
		.stream_name = "Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.init = NULL,
		.ops = &merr_bb_probe_ops,
	},
	[MERR_BB_AUD_VIRTUAL_ASP_DEV] = {
		.name = "Merrifield BB virtual-ASP",
		.stream_name = "virtual-stream",
		.cpu_dai_name = "Virtual-cpu-dai",
		.codec_dai_name = "cs42l73-asp",
		.codec_name = "cs42l73.1-004a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &merr_bb_asp_ops,
	},
};

static int merr_bb_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;

	pr_debug("%s:%d", __func__, __LINE__);
	/* Set codec bias level */
	ctp_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* Add Jack specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, merr_bb_dapm_widgets,
					ARRAY_SIZE(merr_bb_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Jack specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, merr_bb_audio_map,
					ARRAY_SIZE(merr_bb_audio_map));

	/* Keep the voice call paths active during
	suspend. Mark the end points ignore_suspend */
	snd_soc_dapm_ignore_suspend(dapm, "EAROUT");
	snd_soc_dapm_ignore_suspend(dapm, "Ext Spk");
	snd_soc_dapm_ignore_suspend(dapm, "Headphone");
	snd_soc_dapm_ignore_suspend(dapm, "Headset Mic");
	snd_soc_dapm_ignore_suspend(dapm, "DMICA");
	snd_soc_dapm_ignore_suspend(dapm, "DMICB");
	snd_soc_dapm_sync(dapm);
	return ret;
}

static int merr_bb_hp_detection(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int enable)
{
	return cs42l73_hp_detection(codec, jack, enable);
}

static int merr_bb_bp_detection(struct snd_soc_codec *codec,
			struct snd_soc_jack *jack, int enable)
{
	return cs42l73_bp_detection(codec, jack, enable);
}

void merr_bb_mclk_switch(struct device *dev, bool mode)
{
	cs42l73_mclk_switch(dev, mode);
}
static int merr_bb_dai_link(struct snd_soc_card *card)
{
	card->dai_link = merr_bb_dailink;
	card->num_links = ARRAY_SIZE(merr_bb_dailink);
	return 0;
}

static void merr_bb_card_name(struct snd_soc_card *card)
{
	card->name = "merr_prh_cs42l73";
}

struct snd_soc_machine_ops merr_bb_cs42l73_ops = {
	.card_name = merr_bb_card_name,
	.ctp_init = merr_bb_init,
	.dai_link = merr_bb_dai_link,
	.jack_support = true,
	.bp_detection = merr_bb_bp_detection,
	.hp_detection = merr_bb_hp_detection,
	.micsdet_debounce = MIC2SDET_DEBOUNCE_DELAY,
	.mclk_switch = merr_bb_mclk_switch,
	.mic_bias = MICBIAS_NAME,
};

MODULE_DESCRIPTION("ASoC Intel(R) Merrifield BB MID Machine driver");
MODULE_AUTHOR("Omair M Abdullah <omair.m.abdullah@intel.com>");
MODULE_AUTHOR("Ramesh Babu K V<Ramesh.Babu@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:merrbbcs42l73-audio");
