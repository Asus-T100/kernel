/*
 * platform_mrfl_adc.c: Platform data for Merrifield GPADC driver
 *
 * (C) Copyright 2012 Intel Corporation
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
#include <asm/intel_basincove_gpadc.h>

#include "platform_ipc.h"
#include "platform_mrfl_adc.h"

/* SRAM address where the GPADC interrupt register is cached */
#define GPADC_SRAM_INTR_ADDR	0xfffff615

static struct resource mrfl_adc_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_mrfl_adc_res[] __initdata = {
	{
		.name                   = MRFL_ADC_DEV_NAME,
		.num_resources          = ARRAY_SIZE(mrfl_adc_resources),
		.resources              = mrfl_adc_resources,
	},

};

void __init *mrfl_adc_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_basincove_gpadc_platform_data gpadc_pdata;

	gpadc_pdata.intr = GPADC_SRAM_INTR_ADDR;

	handle_ipc_irq_res(entry->irq, ipc_mrfl_adc_res);

	return &gpadc_pdata;
}
