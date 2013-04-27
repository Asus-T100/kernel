/*
 * platform_extcon_mid.c: extcon_mid platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

static struct platform_device extcon_device = {
	.name		= "extcon-mid",
	.id		= -1,
};

static int __init extcon_mid_init(void)
{
	int err;
	err = platform_device_register(&extcon_device);
	if (err < 0)
		pr_err("Fail to register switch-mid platform device.\n");
	return 0;
}

device_initcall(extcon_mid_init);
