/*
 * platform_ft5406.c: ft5406 platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/ft5x06_ts.h>
#include "platform_ft5406.h"

void *ft5406_platform_data(void *info)
{
	static struct ft5x0x_ts_platform_data ft_pdata;

	ft_pdata.irq = get_gpio_by_name("ts_int");
	ft_pdata.wake = get_gpio_by_name("ts_wake");
	ft_pdata.reset = get_gpio_by_name("ts_rst");
	ft_pdata.x_flip = 1;
	ft_pdata.y_flip = 1;

	return &ft_pdata;
}
