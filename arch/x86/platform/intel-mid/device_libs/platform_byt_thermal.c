/*
 * platform_byt_thermal.c: Platform data initilization file for
 *			Intel Baytrail Platform thermal driver
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#define pr_fmt(fmt)  "intel_mid_thermal: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_mid_pmic.h>

#include <asm/intel-mid.h>
#include <asm/intel_mid_thermal.h>

#define BYT_THERM_DEV_NAME	"crystal_cove_thermal"

enum {
	byt_thermal,
};

/* Correlation function to do Y=mX+C */
static int linear_correlation(void *info, long temp, long *res)
{
	struct intel_mid_thermal_sensor *sensor = info;

	if (!sensor)
		return -EINVAL;

	*res = ((temp * sensor->slope) / 1000) + sensor->intercept;

	return 0;
}

static struct intel_mid_thermal_sensor byt_sensors[] = {
	{
		.name = "SYSTHERM0",
		.index = 0,
		.slope = 1000,
		.intercept = 0,
		.temp_correlation = linear_correlation,
	},
	{
		.name = "SYSTHERM1",
		.index = 1,
		.slope = 1000,
		.intercept = 0,
		.temp_correlation = linear_correlation,
	},
	{
		.name = "SYSTHERM2",
		.index = 2,
		.slope = 1000,
		.intercept = 0,
		.temp_correlation = linear_correlation,
	},
	{
		.name = "PMICDIE",
		.index = 3,
		.slope = 1000,
		.intercept = 0,
		.direct = true,
		.temp_correlation = linear_correlation,
	},
};

static struct intel_mid_thermal_platform_data pdata[] = {
	[byt_thermal] = {
		.num_sensors = 4,
		.sensors = byt_sensors,
	},
};

static int __init byt_platform_thermal_init(void)
{
	int ret;

	pr_err("in byt_platform_thermal_init");
	ret = intel_mid_pmic_set_pdata(BYT_THERM_DEV_NAME,
				&pdata[byt_thermal],
				sizeof(pdata[byt_thermal]));
	if (ret)
		pr_err("Configuring platform data failed:%d\n", ret);
	return ret;
}

/*
 * This needs to be earlier than subsys_initcall; so
 * that we set the pdata for thermal subsystem, before
 * the crystal_cove driver registers a device for the same.
 */
arch_initcall(byt_platform_thermal_init);
