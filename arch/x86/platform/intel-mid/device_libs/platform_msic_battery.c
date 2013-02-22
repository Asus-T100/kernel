/*
 * platform_msic_battery.c: MSIC battery platform data initilization file
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
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_msic.h"
#include "platform_msic_battery.h"

void __init *msic_battery_platform_data(void *info)
{
	struct platform_device *pdev = NULL;
	struct sfi_device_table_entry *entry = info;
	void *pdata = NULL;
	int ret = 0;

	pdev = platform_device_alloc(BATTERY_DEVICE_NAME, -1);

	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
					BATTERY_DEVICE_NAME);
		goto out;
	}

	pdev->dev.platform_data = pdata;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add battery platform device\n");
		platform_device_put(pdev);
		goto out;
	}

	install_irq_resource(pdev, entry->irq);

	register_rpmsg_service("rpmsg_msic_battery", RPROC_SCU,
				RP_MSIC_BATTERY);
out:
	return pdata;
}
