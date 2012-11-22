/*
 * platform_mrfld_audio.c: MRFLD audio platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Dharageswari R <dharageswari.r@intel.com>
 *	Vinod Koul <vinod.koul@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <asm/platform_sst_audio.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include <asm/platform_mrfld_audio.h>

static struct mrfld_audio_platform_data mrfld_audio_pdata = {
	.spid = &spid,
};

void *merfld_audio_platform_data(void *info)
{
	struct platform_device *pdev;
	int ret;

	pr_debug("in %s\n", __func__);

	ret = add_sst_platform_device();
	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return NULL;
	}

	pdev = platform_device_alloc("hdmi-audio", -1);
	if (!pdev) {
		pr_err("failed to allocate hdmi-audio platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add hdmi-audio platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	/* request the gpios for audio */
	mrfld_audio_pdata.codec_gpio = get_gpio_by_name("audiocodec_int");
	mrfld_audio_pdata.codec_rst = get_gpio_by_name("audiocodec_rst");

	return &mrfld_audio_pdata;
}
