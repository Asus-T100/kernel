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
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../codecs/lm49453.h"

#define LM49453_DEFAULT_MCLK     12288000

static int mrfld_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt;
	int ret;

	pr_debug("Enter in %s\n", __func__);

	pr_err("cpu dai name %s\n", cpu_dai->name);

	/* LM49453  slave Mode */
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
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
	struct platform_device	*socdev;
	void __iomem		*int_base;
	struct snd_soc_codec	*codec;
};

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
		pr_err("", __func__, card->name, card->dapm.bias_level);
		return -EINVAL;
		break;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return 0;
}

static int mrfld_init(struct snd_soc_pcm_runtime *runtime)
{
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;
	struct snd_soc_card *card = runtime->card;

	pr_debug("Entry %s\n", __func__);
	mrfld_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;
	if (ret)
		return ret;
	mutex_lock(&codec->mutex);
	snd_soc_dapm_sync(dapm);
	mutex_unlock(&codec->mutex);
	/* FIXME
	 * set all the nc_pins, set all the init control
	 * and add any machine controls here
	 */

	/* we want to check if anything is inserted at boot,
	 * so send a fake event to codec and it will read adc
	 * to find if anything is there or not */
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

#ifdef CONFIG_PM
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
};

static int snd_mrfld_mc_probe(struct ipc_device *ipcdev)
{
	int ret_val = 0;
	struct mrfld_mc_private *drv;

	pr_debug("Entry %s\n", __func__);

	drv = kzalloc(sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

	/* register the soc card */
	snd_soc_card_mrfld.dev = &ipcdev->dev;
	ret_val = snd_soc_register_card(&snd_soc_card_mrfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	ipc_set_drvdata(ipcdev, &snd_soc_card_mrfld);
	snd_soc_card_set_drvdata(&snd_soc_card_mrfld, drv);
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
		/* TODO: change this */
		.name = "mrfld_cs42l73",
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
