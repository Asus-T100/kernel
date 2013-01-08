/*
 *  mrfld_machine_lm49453.c - ASoc Machine driver for Intel Merrifield MID platform
 *  for the lm49453 codec
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Vinod Koul <vinod.koul@linux.intel.com>
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

#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/ipc_device.h>
#include <linux/async.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/platform_mrfld_audio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/lm49453.h"

static int mrfld_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt;
	int ret;

	pr_debug("Enter in %s\n", __func__);

	/* LM49453  slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}
	return 0;
}

struct mrfld_mc_private {
	struct snd_soc_jack jack;
};

static int mrfld_jack_gpio_detect(void);

enum gpios {
	MRFLD_HSDET,
};

static struct snd_soc_jack_gpio hs_gpio[] = {
	[MRFLD_HSDET] = {
		.name			= "mrfld-hsdet-gpio",
		.report			= SND_JACK_HEADSET | SND_JACK_HEADPHONE | SND_JACK_MICROPHONE,
		.debounce_time		= 50,
		.jack_status_check	= mrfld_jack_gpio_detect,
		.irq_flags		= IRQF_TRIGGER_RISING,
	},
};

#define LM49453_PD_I_IRQ			BIT(4)
#define LM49453_PD_R_IRQ			BIT(5)
#define LM49453_DETECT_REPORT_VALID_IRQ		BIT(6)
#define LM49453_DETECT_REPORT_INVALID_IRQ	BIT(7)

static int mrfld_jack_gpio_detect(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[MRFLD_HSDET];
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	unsigned int reg;
	int status = 0;

	/* read register clears interrupt */
	reg = snd_soc_read(codec, LM49453_P0_HSD_IRQ1_REG);
	pr_debug("interrupt received, val = 0x%x\n", (reg >> 4));
	if (reg & LM49453_DETECT_REPORT_INVALID_IRQ) {
		pr_debug("invalid headset type - restart HSD\n");
		snd_soc_update_bits(codec, LM49453_P0_PMC_SETUP_REG, LM49453_PMC_SETUP_CHIP_EN, 0);
		snd_soc_update_bits(codec, LM49453_P0_PMC_SETUP_REG,
				    LM49453_PMC_SETUP_CHIP_EN, LM49453_CHIP_EN_HSD_DETECT);
		/* we want to wait till the codec is settled, per LM49453 codec
		 * spec
		 */
		msleep(20);
	}
	/* if insertion bit also set,
	 * decide on removal based on type detected
	 */
	if (!(reg & LM49453_PD_I_IRQ) && (reg & LM49453_PD_R_IRQ)) {
		pr_debug("headset removed");
		status = 0;
	}
	if (reg & LM49453_DETECT_REPORT_VALID_IRQ)
		status = lm49453_get_jack_type(codec);

#ifdef CONFIG_SWITCH_MID
	if (status) {
		if (status == SND_JACK_HEADPHONE)
			mid_headset_report((1<<1));
		else if (status == SND_JACK_HEADSET)
			mid_headset_report(1);
	} else {
		mid_headset_report(0);
	}
#endif
	pr_debug("headset status: 0x%x\n", status);
	return status;
}

static int mrfld_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		card->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_OFF:
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);
	return 0;
}

static int mrfld_set_bias_level_post(struct snd_soc_card *card,
				     struct snd_soc_dapm_context *dapm,
				     enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;
	codec = list_entry(card->codec_dev_list.next,
			struct snd_soc_codec, card_list);
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		/*No action required*/
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
			break;
		card->dapm.bias_level = level;
		break;
	default:
		return -EINVAL;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return 0;
}

static const struct snd_soc_dapm_widget widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route map[] = {
	{"Headphones", NULL, "HPOUTR"},
	{"Headphones", NULL, "HPOUTL"},
	{"AMIC1", NULL, "Mic"},

	/* SWM map link the SWM outs to codec AIF */
	{ "PORT1_SDI", "NULL", "Codec OUT0"  },
	{ "PORT1_SDI", "NULL", "Codec OUT1"  },
	{ "Codec IN0", "NULL", "PORT1_SDO"  },
	{ "Codec IN1", "NULL", "PORT1_SDO"  },
};

static int mrfld_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct mrfld_mc_private *ctx = snd_soc_card_get_drvdata(card);

	pr_debug("Entry %s\n", __func__);
	mrfld_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);
	/* FIXME
	 * set all the nc_pins, set all the init control
	 * and add any machine controls here
	 */

	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE | SND_JACK_BTN_0,
			       &ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}
	ret = snd_soc_jack_add_gpios(&ctx->jack, ARRAY_SIZE(hs_gpio), hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}

	snd_soc_write(codec, LM49453_P0_HSDET_CLK_DIV_REG, 0x00);

	/* set some magic reg in page 2 for jack detection to work well */
	snd_soc_write(codec, LM49453_PAGE_REG, 2);
	snd_soc_write(codec, 0xFC, 0xE);
	snd_soc_write(codec, LM49453_PAGE_REG, 0);

	/* set default debounce time to 2s */
	/* TODO: create control for this in codec drv */
	snd_soc_write(codec, LM49453_P0_HSD_PD_DBNC_REG, 0xF);

	/* set HSD block to detect only CONFIG1 (LRGM) type headsets */
	snd_soc_write(codec, LM49453_P0_HSD_PIN3_4_CFG_REG, LM49453_JACK_CONFIG1);

	snd_soc_write(codec, LM49453_P0_HSD_IRQ_MASK1_REG, 0xF0);
	snd_soc_update_bits(codec, LM49453_P0_PMC_SETUP_REG,
			    LM49453_PMC_SETUP_CHIP_EN, LM49453_CHIP_EN_HSD_DETECT);

	return 0;
}

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int mrfld_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops mrfld_ops = {
	.startup = mrfld_startup,
	.hw_params = mrfld_hw_params,
};

struct snd_soc_dai_link mrfld_msic_dailink[] = {
	{
		.name = "Merrifield Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "LM49453 Headset",
		.codec_name = "lm49453.1-001a",
		.platform_name = "sst-platform",
		.init = mrfld_init,
		.ignore_suspend = 1,
		.ops = &mrfld_ops,
	},
	{
		.name = "Merrifield Speaker Port",
		.stream_name = "Speaker",
		.cpu_dai_name = "Speaker-cpu-dai",
		.codec_dai_name = "LM49453 Speaker",
		.codec_name = "lm49453.1-001a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_ops,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_mrfld_suspend(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}

static int snd_mrfld_resume(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
	return 0;
}

static int snd_mrfld_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}
#else
#define snd_mrfld_suspend NULL
#define snd_mrfld_resume NULL
#define snd_mrfld_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_mrfld = {
	.name = "lm49453-audio",
	.dai_link = mrfld_msic_dailink,
	.num_links = ARRAY_SIZE(mrfld_msic_dailink),
	.set_bias_level_post = mrfld_set_bias_level_post,
	.dapm_widgets = widgets,
	.num_dapm_widgets = ARRAY_SIZE(widgets),
	.dapm_routes = map,
	.num_dapm_routes = ARRAY_SIZE(map),
};

static int snd_mrfld_mc_probe(struct ipc_device *ipcdev)
{
	int ret_val = 0;
	struct mrfld_mc_private *drv;
	struct mrfld_audio_platform_data *pdata;

	pr_debug("Entry %s\n", __func__);

	drv = kzalloc(sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	pdata = ipcdev->dev.platform_data;
	hs_gpio[MRFLD_HSDET].gpio = pdata->codec_gpio;

	/* register the soc card */
	snd_soc_card_mrfld.dev = &ipcdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_mrfld, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_mrfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	ipc_set_drvdata(ipcdev, &snd_soc_card_mrfld);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	kfree(drv);
	return ret_val;
}

static int __devexit snd_mrfld_mc_remove(struct ipc_device *ipcdev)
{
	struct snd_soc_card *soc_card = ipc_get_drvdata(ipcdev);
	struct mrfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_soc_jack_free_gpios(&drv->jack, ARRAY_SIZE(hs_gpio), hs_gpio);
	kfree(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	ipc_set_drvdata(ipcdev, NULL);
	return 0;
}

const struct dev_pm_ops snd_mrfld_mc_pm_ops = {
	.suspend = snd_mrfld_suspend,
	.resume = snd_mrfld_resume,
	.poweroff = snd_mrfld_poweroff,
};

static struct ipc_driver snd_mrfld_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mrfld_lm49453",
		.pm = &snd_mrfld_mc_pm_ops,
	},
	.probe = snd_mrfld_mc_probe,
	.remove = __devexit_p(snd_mrfld_mc_remove),
};

static int __init snd_mrfld_driver_init(void)
{
	ipc_driver_register(&snd_mrfld_mc_driver);
	pr_info("Merrifield Machine Driver mrfld_lm49453 registerd\n");
	return 0;
}

static void __exit snd_mrfld_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	ipc_driver_unregister(&snd_mrfld_mc_driver);
}

late_initcall(snd_mrfld_driver_init);
module_exit(snd_mrfld_driver_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Merrifield MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msic-audio");
