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
#include <sound/msic_audio_platform.h>
#include <sound/sn95031_platform.h>
#include <asm/platform_sst_audio.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"
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

static struct intel_ipc_dev_res ipc_msic_audio_res[] __initdata = {
	{
		.name                   = "msic_audio",
		.num_resources          = ARRAY_SIZE(msic_audio_resources),
		.resources              = msic_audio_resources,
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
	int ret;
	struct platform_device *pdev;
	struct sfi_device_table_entry *entry = info;

	ret = add_sst_platform_device();
	if (ret < 0)
		return NULL;

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

	pdev = platform_device_alloc("sn95031", -1);
	if (!pdev) {
		pr_err("failed to allocate sn95031 platform device\n");
		return NULL;
	}

	ret = platform_device_add_data(pdev, &sn95031_pdata,
				sizeof(struct sn95031_platform_data));
	if (ret) {
		pr_err("failed to add sn95031 platform data\n");
		platform_device_put(pdev);
		return NULL;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add sn95031 platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	if (strncmp(entry->name, "msic_audio", 16) == 0)
		handle_ipc_irq_res(entry->irq, ipc_msic_audio_res);

	return &msic_audio_pdata;
}

