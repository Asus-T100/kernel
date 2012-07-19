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
		.cf = {
			.cf1    = 4096,
			.irf1   = 9134,
			.cf2    = 2867,
			.irf2   = 5816,
			.df     = 52,
			.ga	= 1966 * 9 / 2,
		},
		.pdrive         = 0,
		.ppcount        = 1,
	};

	if (spid.product_line_id == INTEL_MFLDP_LEX_ENG) {
		if (spid.hardware_id < MFLDP_LEX_PR21)
			platform_data.cf.ga = 1966 / 2;
		else
			platform_data.cf.ga = 1966 * 4;
	}

	platform_data.gpio_number = get_gpio_by_name("AL-intr");

	return &platform_data;
}
