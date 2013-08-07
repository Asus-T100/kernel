/*
 *  merr_saltbay_wm8958.c - ASoc Machine driver for Intel Merrfield MID platform
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
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
#include <asm/intel_mid_rpmsg.h>
#include <asm/platform_mrfld_audio.h>
#include <asm/intel_sst_mrfld.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>

#include <linux/mfd/wm8994/core.h>
#include <linux/mfd/wm8994/registers.h>
#include <linux/mfd/wm8994/pdata.h>
#include "../../codecs/wm8994.h"

#define SYSCLK_RATE        24576000
#define DEFAULT_MCLK       19200000

static inline struct snd_soc_codec *mrfld_8958_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "wm8994-codec")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

static int mrfld_wm8958_set_clk_fmt(struct snd_soc_dai *codec_dai)
{
	unsigned int fmt;
	int ret;

	ret = snd_soc_dai_set_tdm_slot(codec_dai, 0, 0, 4, SNDRV_PCM_FORMAT_S24_LE);
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	/* WM8958 slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_pll(codec_dai,
				WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
				DEFAULT_MCLK , SYSCLK_RATE);
	if (ret < 0) {
		pr_err("can't set codec pll configuration %d\n", ret);
		return ret;
	}

	/* take input from 19.2MHz PLL, into MCLK1 to generate FLL1 */
	ret = snd_soc_dai_set_sysclk(codec_dai, WM8994_SYSCLK_FLL1,
				SYSCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		pr_err("can't set codec sysclk configuration %d\n", ret);
		return ret;
	}
	return 0;
}

static int mrfld_8958_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	return mrfld_wm8958_set_clk_fmt(codec_dai);
}

static int mrfld_wm8958_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	return mrfld_wm8958_set_clk_fmt(codec_dai);
}

struct mrfld_8958_mc_private {
	struct snd_soc_jack jack;
	int jack_retry;
};

static int mrfld_8958_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	int ret;

	if (dapm->dev != aif1_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (card->dapm.bias_level == SND_SOC_BIAS_STANDBY) {
			ret = snd_soc_dai_set_pll(aif1_dai,
				WM8994_FLL1, WM8994_FLL_SRC_MCLK1,
				DEFAULT_MCLK, SYSCLK_RATE);
			if (ret < 0) {
				pr_err("Failed to start FLL: %d\n", ret);
				return ret;
			}

			ret = snd_soc_dai_set_sysclk(aif1_dai, WM8994_SYSCLK_FLL1,
					SYSCLK_RATE, SND_SOC_CLOCK_IN);
			if (ret < 0) {
				pr_err("Failed to set codec sysclk configuration %d\n", ret);
				return ret;
			}
		}
		break;
	default:
		break;
	}
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);
	return 0;
}

static int mrfld_8958_set_bias_level_post(struct snd_soc_card *card,
		 struct snd_soc_dapm_context *dapm,
		 enum snd_soc_bias_level level)
{
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	int ret;

	if (dapm->dev != aif1_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_STANDBY:
		pr_debug("in %s turning OFF PLL ", __func__);
		ret = snd_soc_dai_set_sysclk(aif1_dai, WM8994_SYSCLK_MCLK2,
				32768, SND_SOC_CLOCK_IN);
		if (ret < 0) {
			pr_err("Failed to switch to OSC: %d", ret);
			return ret;
		}

		ret = snd_soc_dai_set_pll(aif1_dai, WM8994_FLL1, 0, 0, 0);
		if (ret < 0) {
			pr_err("Failed to stop the FLL: %d", ret);
			return ret;
		}
		break;
	default:
		break;
	}
	dapm->bias_level = level;
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);
	return 0;
}

static const struct snd_soc_dapm_widget widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC", NULL),
};

static const struct snd_soc_dapm_route map[] = {
	{ "Headphones", NULL, "HPOUT1L" },
	{ "Headphones", NULL, "HPOUT1R" },

	/* saltbay uses 2 DMICs, other configs may use more so change below
	 * accordingly
	 */
	{ "DMIC1DAT", NULL, "DMIC" },
	{ "DMIC2DAT", NULL, "DMIC" },
	/*{ "DMIC3DAT", NULL, "DMIC" },*/
	/*{ "DMIC4DAT", NULL, "DMIC" },*/

	/* MICBIAS2 is connected as Bias for both DMIC and AMIC so we link it
	 * here. Also AMIC wires up to IN1LP pin
	 */
	{ "DMIC", NULL, "MICBIAS1" },
	{ "AMIC", NULL, "MICBIAS2" },
	{ "IN1LP", NULL, "AMIC" },

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1DAC1L", "NULL", "Codec OUT0"  },
	{ "AIF1DAC1R", "NULL", "Codec OUT0"  },
	{ "AIF1DAC2L", "NULL", "Codec OUT1"  },
	{ "AIF1DAC2R", "NULL", "Codec OUT1"  },
	{ "Codec IN0", "NULL", "AIF1ADC1L" },
	{ "Codec IN0", "NULL", "AIF1ADC1R" },
	{ "Codec IN1", "NULL", "AIF1ADC2L" },
	{ "Codec IN1", "NULL", "AIF1ADC2R" },
};

static const struct wm8958_micd_rate micdet_rates[] = {
	{ 32768,       true,  1, 4 },
	{ 32768,       false, 1, 1 },
	{ 44100 * 256, true,  7, 10 },
	{ 44100 * 256, false, 7, 10 },
};

static void wm8958_custom_micd_set_rate(struct snd_soc_codec *codec)
{
	struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(codec);
	int best, i, sysclk, val;
	bool idle;
	const struct wm8958_micd_rate *rates;
	int num_rates;

	idle = !wm8994->jack_mic;

	sysclk = snd_soc_read(codec, WM8994_CLOCKING_1);
	if (sysclk & WM8994_SYSCLK_SRC)
		sysclk = wm8994->aifclk[1];
	else
		sysclk = wm8994->aifclk[0];

	if (wm8994->pdata && wm8994->pdata->micd_rates) {
		rates = wm8994->pdata->micd_rates;
		num_rates = wm8994->pdata->num_micd_rates;
	} else {
		rates = micdet_rates;
		num_rates = ARRAY_SIZE(micdet_rates);
	}

	best = 0;
	for (i = 0; i < num_rates; i++) {
		if (rates[i].idle != idle)
			continue;
		if (abs(rates[i].sysclk - sysclk) <
		    abs(rates[best].sysclk - sysclk))
			best = i;
		else if (rates[best].idle != idle)
			best = i;
	}

	val = rates[best].start << WM8958_MICD_BIAS_STARTTIME_SHIFT
		| rates[best].rate << WM8958_MICD_RATE_SHIFT;

	dev_dbg(codec->dev, "MICD rate %d,%d for %dHz %s\n",
		rates[best].start, rates[best].rate, sysclk,
		idle ? "idle" : "active");

	snd_soc_update_bits(codec, WM8958_MIC_DETECT_1,
			    WM8958_MICD_BIAS_STARTTIME_MASK |
			    WM8958_MICD_RATE_MASK, val);
}

static void wm8958_custom_mic_id(void *data, u16 status)
{
	struct snd_soc_codec *codec = data;
	struct wm8994_priv *wm8994 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "wm8958 custom mic id called with status %x\n",
		status);

	/* Either nothing present or just starting detection */
	if (!(status & WM8958_MICD_STS)) {
		/* If nothing present then clear our statuses */
		dev_dbg(codec->dev, "Detected open circuit\n");

		schedule_delayed_work(&wm8994->open_circuit_work,
				      msecs_to_jiffies(2500));
		return;
	}

	/* If the measurement is showing a high impedence we've got a
	 * microphone.
	 */
	if (status & 0x600) {
		dev_dbg(codec->dev, "Detected microphone\n");

		wm8994->mic_detecting = false;
		wm8994->jack_mic = true;

		wm8958_custom_micd_set_rate(codec);

		snd_soc_jack_report(wm8994->micdet[0].jack, SND_JACK_HEADSET,
				    SND_JACK_HEADSET);
	}


	if (status & 0xfc) {
		dev_dbg(codec->dev, "Detected headphone\n");

		/* Partial inserts of headsets with complete insert
		 * after an indeterminate amount of time require
		 * continouous micdetect enabled (until open circuit
		 * or headset is detected)
		 * */
		wm8994->mic_detecting = true;

		wm8958_custom_micd_set_rate(codec);

		snd_soc_jack_report(wm8994->micdet[0].jack, SND_JACK_HEADPHONE,
				    SND_JACK_HEADSET);
	}
}

static int mrfld_8958_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	unsigned int fmt;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dai *aif1_dai = card->rtd[0].codec_dai;
	struct mrfld_8958_mc_private *ctx = snd_soc_card_get_drvdata(card);

	pr_debug("Entry %s\n", __func__);

	ret = snd_soc_dai_set_tdm_slot(aif1_dai, 0, 0, 4, SNDRV_PCM_FORMAT_S24_LE);
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	/* WM8958 slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(aif1_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	mrfld_8958_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	/* mark pins as NC */


	snd_soc_dapm_sync(dapm);
	/* FIXME
	 * set all the nc_pins, set all the init control
	 * and add any machine controls here
	 */

	ctx->jack_retry = 0;
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE | SND_JACK_BTN_0,
			       &ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}

	snd_jack_set_key(ctx->jack.jack, SND_JACK_BTN_0, KEY_MEDIA);

	wm8958_mic_detect(codec, &ctx->jack, NULL, NULL,
			  wm8958_custom_mic_id, codec);

	snd_soc_update_bits(codec, WM8994_AIF1_DAC1_FILTERS_1, WM8994_AIF1DAC1_MUTE, 0);
	snd_soc_update_bits(codec, WM8994_AIF1_DAC2_FILTERS_1, WM8994_AIF1DAC2_MUTE, 0);
	return 0;
}

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int mrfld_8958_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops mrfld_8958_ops = {
	.startup = mrfld_8958_startup,
	.hw_params = mrfld_8958_hw_params,
};

static int mrfld_8958_voip_aware_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops mrfld_8958_voip_aware_ops = {
	.startup = mrfld_8958_voip_aware_startup,
	.hw_params = mrfld_8958_hw_params,
};

static struct snd_soc_compr_ops mrfld_compr_ops = {
	.set_params = mrfld_wm8958_compr_set_params,
};

struct snd_soc_dai_link mrfld_8958_msic_dailink[] = {
	[MERR_SALTBAY_AUDIO] = {
		.name = "Merrifield Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = mrfld_8958_init,
		.ignore_suspend = 1,
		.ops = &mrfld_8958_ops,
	},
	[MERR_SALTBAY_COMPR] = {
		.name = "Merrifield Compress Port",
		.stream_name = "Compress",
		.platform_name = "sst-platform",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.compr_ops = &mrfld_compr_ops,
	},
	[MERR_SALTBAY_VOIP] = {
		.name = "Merrifield VOIP Port",
		.stream_name = "Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8958_voip_aware_ops,
	},
	[MERR_SALTBAY_PROBE] = {
		.name = "Merrifield Probe Port",
		.stream_name = "Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.playback_count = 8,
		.capture_count = 8,
	},
	[MERR_SALTBAY_AWARE] = {
		.name = "Merrifield Aware Port",
		.stream_name = "Aware",
		.cpu_dai_name = "Aware-cpu-dai",
		.codec_dai_name = "wm8994-aif1",
		.codec_name = "wm8994-codec",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8958_voip_aware_ops,
	},
	[MERR_SALTBAY_VAD] = {
		.name = "Merrifield VAD Port",
		.stream_name = "Vad",
		.cpu_dai_name = "Virtual-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
	},
	[MERR_SALTBAY_POWER] = {
		.name = "Virtual Power Port",
		.stream_name = "Power",
		.cpu_dai_name = "Power-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_mrfld_8958_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}

static void snd_mrfld_8958_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return;
}

static int snd_mrfld_8958_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}
#else
#define snd_mrfld_8958_prepare NULL
#define snd_mrfld_8958_complete NULL
#define snd_mrfld_8958_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_mrfld = {
	.name = "wm8958-audio",
	.dai_link = mrfld_8958_msic_dailink,
	.num_links = ARRAY_SIZE(mrfld_8958_msic_dailink),
	.set_bias_level = mrfld_8958_set_bias_level,
	.set_bias_level_post = mrfld_8958_set_bias_level_post,
	.dapm_widgets = widgets,
	.num_dapm_widgets = ARRAY_SIZE(widgets),
	.dapm_routes = map,
	.num_dapm_routes = ARRAY_SIZE(map),
};

static int snd_mrfld_8958_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct mrfld_8958_mc_private *drv;
	struct mrfld_audio_platform_data *pdata;

	pr_debug("Entry %s\n", __func__);

	drv = kzalloc(sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	pdata = pdev->dev.platform_data;

	/* register the soc card */
	snd_soc_card_mrfld.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_mrfld, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_mrfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_mrfld);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	kfree(drv);
	return ret_val;
}

static int snd_mrfld_8958_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_8958_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	kfree(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

const struct dev_pm_ops snd_mrfld_8958_mc_pm_ops = {
	.prepare = snd_mrfld_8958_prepare,
	.complete = snd_mrfld_8958_complete,
	.poweroff = snd_mrfld_8958_poweroff,
};

static struct platform_driver snd_mrfld_8958_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mrfld_wm8958",
		.pm = &snd_mrfld_8958_mc_pm_ops,
	},
	.probe = snd_mrfld_8958_mc_probe,
	.remove = snd_mrfld_8958_mc_remove,
};

static int snd_mrfld_8958_driver_init(void)
{
	pr_info("Merrifield Machine Driver mrfld_wm8958 registerd\n");
	return platform_driver_register(&snd_mrfld_8958_mc_driver);
}

static void snd_mrfld_8958_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_mrfld_8958_mc_driver);
}

static int snd_mrfld_8958_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_mrfld wm8958 rpmsg device\n");

	ret = snd_mrfld_8958_driver_init();

out:
	return ret;
}

static void snd_mrfld_8958_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_mrfld_8958_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_mrfld wm8958 rpmsg device\n");
}

static void snd_mrfld_8958_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_mrfld_8958_rpmsg_id_table[] = {
	{ .name = "rpmsg_mrfld_wm8958_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_mrfld_8958_rpmsg_id_table);

static struct rpmsg_driver snd_mrfld_8958_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_mrfld_8958_rpmsg_id_table,
	.probe		= snd_mrfld_8958_rpmsg_probe,
	.callback	= snd_mrfld_8958_rpmsg_cb,
	.remove		= snd_mrfld_8958_rpmsg_remove,
};

static int __init snd_mrfld_8958_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_mrfld_8958_rpmsg);
}
late_initcall(snd_mrfld_8958_rpmsg_init);

static void __exit snd_mrfld_8958_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_mrfld_8958_rpmsg);
}
module_exit(snd_mrfld_8958_rpmsg_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Merrifield MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mrfld_wm8958");
