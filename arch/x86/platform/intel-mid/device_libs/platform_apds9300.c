/*
 * platform_apds9300x.c: apds9300 platform data initilization file
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
#include <linux/i2c/apds9300.h>
#include <asm/intel-mid.h>
#include "platform_apds9300.h"

void *apds9300_platform_data(void *info)
{
	static struct apds9300_platform_data platform_data = {
		.cf = {
			.cf1    = 129,
			.irf1   = 243,
			.cf2    = 94,
			.irf2   = 119,
			.cf3    = 64,
			.irf3   = 74,
			.cf4    = 14,
			.irf4   = 11,
			.df     = 52,
			.ga     = 15728,
		},
	};

	platform_data.gpio_number = get_gpio_by_name("AL-intr");

	return &platform_data;
}
