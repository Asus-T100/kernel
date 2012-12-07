/*
 * platform_mrfl_pmic.c: Platform data for Merrifield PMIC driver
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
#include <asm/pmic_pdata.h>
#include <linux/power/bq24261_charger.h>

#include "platform_ipc.h"
#include "platform_mrfl_pmic.h"

static struct resource mrfl_pmic_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_mrfl_pmic_res[] __initdata = {
	{
		.name                   = MRFL_PMIC_DEV_NAME,
		.num_resources          = ARRAY_SIZE(mrfl_pmic_resources),
		.resources              = mrfl_pmic_resources,
	},

};

void __init *mrfl_pmic_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct pmic_platform_data pmic_pdata;

	handle_ipc_irq_res(entry->irq, ipc_mrfl_pmic_res);
#ifdef CONFIG_BQ24261_CHARGER
	pmic_pdata.cc_to_reg = bq24261_cc_to_reg;
	pmic_pdata.cv_to_reg = bq24261_cv_to_reg;
#endif

	return &pmic_pdata;
}
