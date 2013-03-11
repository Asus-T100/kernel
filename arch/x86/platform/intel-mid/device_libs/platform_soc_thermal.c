/*
 * platform_soc_thermal.c: Platform data for SoC DTS driver
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#define pr_fmt(fmt)  "intel_soc_thermal: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include "platform_soc_thermal.h"

#include <asm/intel-mid.h>

static struct resource res = {
		.flags = IORESOURCE_IRQ,
};

void soc_thrm_device_handler(struct sfi_device_table_entry *pentry,
				struct devs_id *dev)
{
	int ret;
	struct platform_device *pdev;

	pr_info("IPC bus = %d, name = %16.16s, irq = 0x%2x\n",
		pentry->host_num, pentry->name, pentry->irq);

	res.start = pentry->irq;

	pdev = platform_device_register_simple(pentry->name, -1,
					(const struct resource *)&res, 1);
	if (IS_ERR(pdev)) {
		ret = PTR_ERR(pdev);
		pr_err("platform_soc_thermal:pdev_register failed: %d\n", ret);
	}
}

