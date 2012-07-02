/*
 * platform_msic_vdd.c: MSIC VDD platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Chaurasia, Avinash K <avinash.k.chaurasia@intel.com>
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
#include <linux/gpio.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_msic_vdd.h"

#define BCUIRQ 0x24

static struct resource msic_vdd_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_vdd_res[] __initdata = {
	{
		.name                   = MSIC_VDD_DEV_NAME,
		.num_resources          = ARRAY_SIZE(msic_vdd_resources),
		.resources              = msic_vdd_resources,
	},

};

void __init *msic_vdd_platform_data(void *info)
{
	static struct intel_msic_vdd_pdata msic_vdd_pdata;

	msic_vdd_pdata.msi = BCUIRQ;

	handle_ipc_irq_res(BCUIRQ, ipc_msic_vdd_res);

	return &msic_vdd_pdata;
}
