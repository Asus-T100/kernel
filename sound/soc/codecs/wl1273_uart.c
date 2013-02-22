/*
 *  wl1273_uart.c - ALSA ASOC driver for WL1273 in Combined Interface Mode
 *
 *  Copyright (C) 2012 Intel Corp
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

#define WL1273_BT_CODEC_NAME "wl1273_BT"
#define WL1273_FM_CODEC_NAME "wl1273_FM"

static struct snd_soc_codec_driver soc_codec_wl1273_uart = {
		.probe = NULL,
		.remove = NULL,
};

static int wl1273_uart_startup(struct snd_pcm_substream *substream,
			  struct snd_soc_dai *codec_dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;

	/*
	 * The FM and BT stream cannot be open in //
	 */
	if (!strcmp(codec_dai->name, WL1273_FM_CODEC_NAME)) {
		if (codec->active) {
			WARN(1, "WL1273 UART: Open %s unsupported Config\n",
					codec_dai->name);
			return -EBUSY;
		}
	} else if (!strcmp(codec_dai->name, WL1273_BT_CODEC_NAME)) {
		if (codec->active &&
				(!codec_dai->playback_active) &&
				(!codec_dai->capture_active)) {
			WARN(1, "WL1273 UART: Open %s unsupported Config\n",
					codec_dai->name);
			return -EBUSY;
		}
	}
	return 0;
}

static struct snd_soc_dai_ops wl1273_uart_dai_ops = {
	.startup	= wl1273_uart_startup,
};

static struct snd_soc_dai_driver wl1273_uart_dai[] = {
	{
		.name = WL1273_BT_CODEC_NAME,
		.id = 0,
		.playback = {
				.stream_name = "BT_Playback",
				.channels_min = 1,
				.channels_max = 1,
				.rates = SNDRV_PCM_RATE_8000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
				.stream_name = "BT_Capture",
				.channels_min = 1,
				.channels_max = 1,
				.rates = SNDRV_PCM_RATE_8000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &wl1273_uart_dai_ops,
	},
	{
		.name = WL1273_FM_CODEC_NAME,
		.id = 1,
		.playback = {
				.stream_name = "FM_Playback",
				.channels_min = 0,
				.channels_max = 0,
				.rates = SNDRV_PCM_RATE_48000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.capture = {
				.stream_name = "FM_Capture",
				.channels_min = 1,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_48000,
				.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &wl1273_uart_dai_ops,
	},
};

static int __devinit wl1273_uart_device_probe(struct platform_device *pdev)
{
	pr_info("codec device probe called for %s\n",
			dev_name(&pdev->dev));

	return snd_soc_register_codec(&pdev->dev, &soc_codec_wl1273_uart,
			wl1273_uart_dai, ARRAY_SIZE(wl1273_uart_dai));
}

static int __devexit wl1273_uart_device_remove(struct platform_device *pdev)
{
	pr_info("codec device remove called\n");
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver wl1273_uart_codec_driver = {
	.driver		= {
		.name		= "wl1273_uart",
		.owner		= THIS_MODULE,
	},
	.probe		= wl1273_uart_device_probe,
	.remove		= wl1273_uart_device_remove,
};

static int __init wl1273_uart_init(void)
{
	pr_info("FCT %s called\n", __func__);
	return platform_driver_register(&wl1273_uart_codec_driver);
}
module_init(wl1273_uart_init);

static void __exit wl1273_uart_exit(void)
{
	pr_info("FCT %s called\n", __func__);
	platform_driver_unregister(&wl1273_uart_codec_driver);
}
module_exit(wl1273_uart_exit);

MODULE_DESCRIPTION("ASoC TI WL1273 codec driver");
MODULE_AUTHOR("Selma Bensaid <selma.bensaid@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:wl1273-uart");

