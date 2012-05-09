/*
 *  es305_codec.c - ALSA ASOC [dummy] Codec driver for Audience ES305
 *
 *  Copyright (C) 2011 Intel Corp
 *  Author:
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

#define FORMAT(fmt) "%s: " fmt, __func__
#define pr_fmt(fmt) KBUILD_MODNAME ": " FORMAT(fmt)

#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/intel_scu_ipc.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

/*
 *
 * DUMMY SOC CODEC DRIVER for es305
 *
 */

static struct snd_soc_dai_driver es305_dai[] = {
	{
		.name = "es305 Voice",
		.id = 0,
		.playback = {
				.stream_name = "es305_Downlink",
				.channels_min = 1,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_48000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
				.stream_name = "es305_Uplink",
				.channels_min = 1,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_48000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = NULL,
	},

};

static struct snd_soc_codec_driver soc_codec_es305 = {
	.probe = NULL,
	.remove = NULL,
};


static int __devinit es305_device_probe(struct platform_device *pdev)
{
	pr_info("es305: codec device probe called for %s\n",
			dev_name(&pdev->dev));
	return snd_soc_register_codec(&pdev->dev, &soc_codec_es305,
			es305_dai, ARRAY_SIZE(es305_dai));
}

static int __devexit es305_device_remove(struct platform_device *pdev)
{
	pr_info("es305: codec device remove called\n");
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver es305_codec_driver = {
	.driver		= {
		.name		= "es305_codec",
		.owner		= THIS_MODULE,
	},
	.probe		= es305_device_probe,
	.remove		= es305_device_remove,
};

static int __init es305_init(void)
{
	pr_info("called\n");
	return platform_driver_register(&es305_codec_driver);
}
module_init(es305_init);

static void __exit es305_exit(void)
{
	pr_info("called\n");
	platform_driver_unregister(&es305_codec_driver);
}
module_exit(es305_exit);

MODULE_DESCRIPTION("ASoC es305 codec driver");
MODULE_AUTHOR("Selma Bensaid <selma.bensaid@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:es305");

