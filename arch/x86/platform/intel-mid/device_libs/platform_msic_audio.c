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
#include <sound/msic_audio_platform.h>
#include <sound/sn95031_platform.h>
#include "platform_msic.h"
#include "platform_msic_audio.h"

static struct resource msic_audio_resources[] __initdata = {
	{
		.name  = "IRQ",
		.flags = IORESOURCE_IRQ,
	},
	{
		.name  = "IRQ_BASE",
		.flags = IORESOURCE_MEM,
		.start = MSIC_IRQ_STATUS_OCAUDIO,
		.end   = MSIC_IRQ_STATUS_ACCDET,
	},
};

static struct sn95031_platform_data sn95031_pdata = {
	.spid = &spid,
};

static struct msic_audio_platform_data msic_audio_pdata = {
	.spid = &spid,
};

void *msic_audio_platform_data(void *info)
{
	void *pdata = NULL;
	struct platform_device *pdev = NULL;
	struct sfi_device_table_entry *entry = info;

	pdev = platform_device_alloc("sst-platform", -1);
	if (!pdev) {
		pr_err("failed to create sst-platform device\n");
		goto out;
	}
	if (platform_device_add(pdev)) {
		pr_err("failed to add sst-platform device\n");
		goto pdev_add_fail;
	}

	pdev = platform_device_alloc("hdmi-audio", -1);
	if (!pdev) {
		pr_err("failed to allocate hdmi-audio platform device\n");
		goto out;
	}
	if (platform_device_add(pdev)) {
		pr_err("failed to add hdmi-audio platform device\n");
		goto pdev_add_fail;
	}

	pdev = platform_device_alloc("sn95031", -1);
	if (!pdev) {
		pr_err("failed to allocate sn95031 platform device\n");
		goto out;
	}
	if (platform_device_add_data(pdev, &sn95031_pdata,
			sizeof(struct sn95031_platform_data))) {
		pr_err("failed to add sn95031 platform data\n");
		goto pdev_add_fail;
	}
	if (platform_device_add(pdev)) {
		pr_err("failed to add sn95031 platform device\n");
		goto pdev_add_fail;
	}

	pdev = platform_device_alloc(MSIC_AUDIO_DEVICE_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
			MSIC_AUDIO_DEVICE_NAME);
		goto out;
	}
	install_irq_resource(pdev, entry->irq);
	if (platform_device_add_data(pdev, &msic_audio_pdata,
			sizeof(struct msic_audio_platform_data))) {
		pr_err("failed to add audio platform data\n");
		goto pdev_add_fail;
	}
	if (platform_device_add_resources(pdev, &msic_audio_resources,
			ARRAY_SIZE(msic_audio_resources))) {
		pr_err("failed to add audio resources\n");
		goto pdev_add_fail;
	}
	if (platform_device_add(pdev)) {
		pr_err("failed to add SFI platform dev %s\n",
			MSIC_AUDIO_DEVICE_NAME);
		goto pdev_add_fail;
	}
	pdata = &msic_audio_pdata;
	register_rpmsg_service("rpmsg_msic_audio", RPROC_SCU, RP_MSIC_AUDIO);

pdev_add_fail:
	platform_device_put(pdev);
out:
	return pdata;
}
