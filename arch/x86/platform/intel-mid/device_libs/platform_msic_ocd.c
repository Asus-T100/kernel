/*
 * platform_msic_ocd.c: MSIC OCD platform data initilization file
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
#include "platform_msic_ocd.h"

static struct resource msic_ocd_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_ocd_res[] __initdata = {
	{
		.name                   = "msic_ocd",
		.num_resources          = ARRAY_SIZE(msic_ocd_resources),
		.resources              = msic_ocd_resources,
	},

};

void __init *msic_ocd_platform_data(void *info)
{
	static struct intel_msic_ocd_pdata msic_ocd_pdata;
	int gpio;

	gpio = get_gpio_by_name("ocd_gpio");

	if (gpio < 0)
		return NULL;

	msic_ocd_pdata.gpio = gpio;

	handle_ipc_irq_res(gpio + INTEL_MID_IRQ_OFFSET, ipc_msic_ocd_res);

	return &msic_ocd_pdata;
}

