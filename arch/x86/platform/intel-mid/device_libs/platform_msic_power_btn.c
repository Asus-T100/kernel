/*
 * platform_msic_power_btn.c: MSIC power btn platform data initilization file
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
#include <linux/sfi.h>
#include <linux/init.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_msic.h"
#include "platform_msic_power_btn.h"

/* SRAM address for power button state */
#if defined(CONFIG_BOARD_CTP) && defined(CONFIG_POWER_BUTTON_CLVP)
#define MSIC_PB_STAT    0xffffefcb
#elif defined(CONFIG_BOARD_CTP)
#define MSIC_PB_STAT    0xffff7fcb
#else
#define MSIC_PB_STAT	0xffff7fd0
#endif
#define MSIC_PB_LEN	1

static struct resource mid_pb_resources[] __initdata = {
	{
		.name	= "IRQ",
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name  = "MSIC_PB_STAT",
		.flags = IORESOURCE_MEM,
		.start = MSIC_PB_STAT,
		.end   = MSIC_PB_STAT,
	},
};

void __init *msic_power_btn_platform_data(void *info)
{
	struct platform_device *pdev;
	struct sfi_device_table_entry *entry = info;
	struct resource *rc;
	void *pdata = NULL;
	void *retval = NULL;

	pdev = platform_device_alloc(POWERBTN_DEVICE_NAME, -1);

	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
			POWERBTN_DEVICE_NAME);
		goto out;
	}

	if (platform_device_add_resources(pdev, &mid_pb_resources,
			ARRAY_SIZE(mid_pb_resources))) {
		pr_err("failed to add resources\n");
		goto out;
	}

	rc = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "IRQ");
	if (!rc) {
		pr_err("failed to get resource by name for IRQ\n");
		goto out;
	}
	rc->start = entry->irq;

	pdev->dev.platform_data = pdata;

	if (platform_device_add(pdev)) {
		pr_err("failed to add powerbtn platform device\n");
		platform_device_put(pdev);
		goto out;
	}

	register_rpmsg_service("rpmsg_mid_powerbtn", RPROC_SCU,
				RP_MSIC_POWER_BTN);
out:
	return retval;
}
