/*
 * platform_gps.c: gps platform data initialization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/intel_mid_gps.h>
#include <asm/intel-mid.h>
#include "platform_gps.h"

static struct intel_mid_gps_platform_data gps_data = {
	.gpio_reset  = -EINVAL,
	.gpio_enable = -EINVAL,
	.reset  = RESET_ON,
	.enable = ENABLE_OFF,
};

static struct platform_device intel_mid_gps_device = {
	.name			= "intel_mid_gps",
	.id			= -1,
	.dev			= {
		.platform_data	= &gps_data,
	},
};

static int __init intel_mid_gps_device_init(void)
{
	int ret = 0;

	gps_data.gpio_reset  = get_gpio_by_name(GPS_GPIO_RESET);
	gps_data.gpio_enable = get_gpio_by_name(GPS_GPIO_ENABLE);

	ret = platform_device_register(&intel_mid_gps_device);

	if (ret < 0)
		pr_err("platform_device_register failed for intel_mid_gps\n");

	return ret;
}

device_initcall(intel_mid_gps_device_init);
