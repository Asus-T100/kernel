/*
 * platform_mrfl_thermal.c: Platform data initilization file for
 *			Intel Merrifield Platform thermal driver
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/mfd/intel_msic.h>
#include <linux/platform_device.h>

#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_mrfl_thermal.h"

void __init *mrfl_thermal_platform_data(void *info)
{
	struct platform_device *pdev;
	struct sfi_device_table_entry *entry = info;

	pr_err("inside mrfl_thermal_platform_data\n");

	pdev = platform_device_alloc(MRFL_THERM_DEV_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
			MRFL_THERM_DEV_NAME);
		return NULL;
	}

	if (platform_device_add(pdev)) {
		pr_err("failed to add thermal platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	install_irq_resource(pdev, entry->irq);
	register_rpmsg_service("rpmsg_mrfl_thermal", RPROC_SCU,
				RP_BCOVE_THERMAL);

	return 0;
}
