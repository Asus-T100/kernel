/*
 * platform_mid_thermal_probe.c: thermal probe platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/platform_device.h>
#include <linux/intel_mid_thermal_probe.h>

#define THERMAL_ADC_CH	0x0b
#define DEVICE_NAME "mid_thermal_probe"

static struct thermal_probe_platform_data thermal_data = {
	.adc_ch = THERMAL_ADC_CH,
};

static struct platform_device thermal_device = {
	.name		= DEVICE_NAME,
	.id		= -1,
	.dev		= {
				.platform_data		= &thermal_data,
	},
};

static int __init mid_thermal_probe_init(void)
{
	int err;
	err = platform_device_register(&thermal_device);
	if (err < 0)
		pr_err("Fail to register mid thermal probe platform device.\n");

	return 0;
}

device_initcall(mid_thermal_probe_init);
