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
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_msic_power_btn.h"

static struct resource msic_power_btn_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_power_btn_res[] __initdata = {
	{
		.name                   = "msic_power_btn",
		.num_resources          = ARRAY_SIZE(msic_power_btn_resources),
		.resources              = msic_power_btn_resources,
	},

};

void __init *msic_power_btn_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;

	handle_ipc_irq_res(entry->irq, ipc_msic_power_btn_res);

	return NULL;
}

