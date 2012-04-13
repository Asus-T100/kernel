/*
 * platform_pmic_gpio.c: PMIC GPIO platform data initilization file
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
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/intel_pmic_gpio.h>
#include "platform_ipc.h"
#include "platform_pmic_gpio.h"

static struct resource pmic_gpio_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_pmic_gpio_res[] __initdata = {
	{
		.name                   = "pmic_gpio",
		.num_resources          = ARRAY_SIZE(pmic_gpio_resources),
		.resources              = pmic_gpio_resources,
	},
};

void __init *pmic_gpio_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_pmic_gpio_platform_data pmic_gpio_pdata;
	int gpio_base = get_gpio_by_name("pmic_gpio_base");

	if (gpio_base == -1)
		gpio_base = 64;
	pmic_gpio_pdata.gpio_base = gpio_base;
	pmic_gpio_pdata.irq_base = gpio_base + INTEL_MID_IRQ_OFFSET;
	pmic_gpio_pdata.gpiointr = 0xffffeff8;

	handle_ipc_irq_res(entry->irq, ipc_pmic_gpio_res);

	return &pmic_gpio_pdata;
}

