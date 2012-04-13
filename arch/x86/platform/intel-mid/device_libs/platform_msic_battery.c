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
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_msic_battery.h"

static struct resource msic_battery_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_battery_res[] __initdata = {
	{
		.name                   = "msic_battery",
		.num_resources          = ARRAY_SIZE(msic_battery_resources),
		.resources              = msic_battery_resources,
	},

};

void __init *msic_battery_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;

	handle_ipc_irq_res(entry->irq, ipc_msic_battery_res);

	return NULL;
}

