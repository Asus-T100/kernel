/*
 * platform_msic_adc.c: MSIC ADC platform data initilization file
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
#include <asm/intel_mid_gpadc.h>
#include "platform_ipc.h"
#include "platform_msic_adc.h"

static struct resource msic_adc_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_adc_res[] __initdata = {
	{
		.name                   = "msic_adc",
		.num_resources          = ARRAY_SIZE(msic_adc_resources),
		.resources              = msic_adc_resources,
	},

};

void __init *msic_adc_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_mid_gpadc_platform_data msic_adc_pdata;
	msic_adc_pdata.intr = 0xffff7fc0;

	handle_ipc_irq_res(entry->irq, ipc_msic_adc_res);

	return &msic_adc_pdata;
}
