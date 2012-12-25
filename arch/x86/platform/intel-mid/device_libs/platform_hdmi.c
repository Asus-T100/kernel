/*
 * platform_hdmi.c: HDMI platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
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
#include <linux/ipc_device.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_hdmi.h"

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>


static struct resource hdmi_resources[] __initdata = {
	{
		.name  = "IRQ",
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_hdmi_res[] __initdata = {
	{
		.name                   = "hdmi",
		.num_resources          = ARRAY_SIZE(hdmi_resources),
		.resources              = hdmi_resources,
	},
};

void *hdmi_platform_data(void *info)
{
	return NULL;
}

/* hack to add hdmi device to platform driver */
static int hdmi_platform_dev_init(void)
{
	int ret = 0;
	struct platform_device *pdev;

	pdev = platform_device_alloc("hdmi", -1);

	if (!pdev) {
		return -ENOMEM;
	}
	ret = platform_device_add(pdev);
	if (ret) {
		platform_device_put(pdev);
		return ret;
	}
	return ret;
}

rootfs_initcall(hdmi_platform_dev_init);



