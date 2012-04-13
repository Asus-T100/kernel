/*
 * platform_msic_gpio.c: MSIC GPIO platform data initilization file
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
#include <linux/gpio.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_msic_gpio.h"

static struct resource msic_gpio_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_gpio_res[] __initdata = {
	{
		.name                   = "msic_gpio",
		.num_resources          = ARRAY_SIZE(msic_gpio_resources),
		.resources              = msic_gpio_resources,
	},

};

void __init *msic_gpio_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_msic_gpio_pdata msic_gpio_pdata;

	int gpio = get_gpio_by_name("msic_gpio_base");

	if (gpio < 0)
		return NULL;

	msic_gpio_pdata.gpio_base = gpio;
	handle_ipc_irq_res(entry->irq, ipc_msic_gpio_res);

	return &msic_gpio_pdata;
}

