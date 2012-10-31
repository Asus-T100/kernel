/*
 * platform_clvs_audio.c: CLVS audio platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
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
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include <asm/platform_clvs_audio.h>

static struct clvcs_audio_platform_data clvcs_audio_pdata = {
	.spid = &spid,
};

void *clvs_audio_platform_data(void *info)
{
	struct platform_device *pdev;
	int ret;

	ret = add_sst_platform_device();
	if (ret < 0)
		return NULL;

	pdev = platform_device_alloc("compress-sst", -1);
	if (!pdev) {
		pr_err("failed to allocate compress-sst platform device\n");
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add compress-sst platform device\n");
		platform_device_put(pdev);
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

	return &clvcs_audio_pdata;
}
