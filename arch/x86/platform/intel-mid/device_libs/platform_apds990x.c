/*
 * platform_apds990x.c: apds990x platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/i2c/apds990x.h>
#include <asm/intel-mid.h>
#include "platform_apds990x.h"

void *apds990x_platform_data(void *info)
{
	static struct apds990x_platform_data platform_data = {
		.pdrive = 0,
		.ppcount = 1,
	};
	platform_data.gpio_number = get_gpio_by_name("AL-intr");

	return &platform_data;
}
