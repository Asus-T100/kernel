/*
 * platform_msic_audio.c: MSIC audio platform data initilization file
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
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_msic.h"
#include "platform_msic_audio.h"

void *msic_audio_platform_data(void *info)
{
	struct platform_device *pdev;

	pdev = platform_device_register_simple("sst-platform", -1, NULL, 0);

	if (IS_ERR(pdev)) {
		pr_err("failed to create audio platform device\n");
		return NULL;
	}

	register_rpmsg_service("rpmsg_msic_audio", RPROC_SCU, RP_MSIC_AUDIO);

	return msic_generic_platform_data(info, INTEL_MSIC_BLOCK_AUDIO);
}

