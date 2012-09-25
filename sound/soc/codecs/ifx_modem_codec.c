/*
 *  ifx_modem.c - ALSA ASOC driver for IFX1130 in Combined Interface Mode
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


static struct snd_soc_dai_driver ifx_modem_dai[] = {
	{
		.name = "IFX_Modem_Mixing",
		.id = 0,
		.playback = {
				.stream_name = "IFX_Modem_Mix",
				.channels_min = 1,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_48000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
				.stream_name = "IFX_Modem_Record",
				.channels_min = 1,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_48000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = NULL,
	},

};

static struct snd_soc_codec_driver soc_codec_ifx_modem = {
	.probe = NULL,
	.remove = NULL,
};


static int __devinit ifx_modem_device_probe(struct platform_device *pdev)
{
	pr_info("codec device probe called for %s\n",
			dev_name(&pdev->dev));
	return snd_soc_register_codec(&pdev->dev, &soc_codec_ifx_modem,
			ifx_modem_dai, ARRAY_SIZE(ifx_modem_dai));
}

static int __devexit ifx_modem_device_remove(struct platform_device *pdev)
{
	pr_info("codec device remove called\n");
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver ifx_modem_codec_driver = {
	.driver		= {
		.name		= "ifx_modem",
		.owner		= THIS_MODULE,
	},
	.probe		= ifx_modem_device_probe,
	.remove		= ifx_modem_device_remove,
};

static int __init ifx_modem_init(void)
{
	pr_info("called\n");
	return platform_driver_register(&ifx_modem_codec_driver);
}
module_init(ifx_modem_init);

static void __exit ifx_modem_exit(void)
{
	pr_info("called\n");
	platform_driver_unregister(&ifx_modem_codec_driver);
}
module_exit(ifx_modem_exit);

MODULE_DESCRIPTION("ASoC IFX Modem codec driver");
MODULE_AUTHOR("Selma Bensaid <selma.bensaid@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:ifx-modem");

