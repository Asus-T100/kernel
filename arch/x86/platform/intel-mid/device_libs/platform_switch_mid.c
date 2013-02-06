/*
 * platform_switch_mid.c: switch_mid platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_switch_mid.h"

static struct platform_device switch_device = {
	.name		= DEVICE_NAME,
	.id		= -1,
};

static int __init switch_mid_init(void)
{
	int err;
	err = platform_device_register(&switch_device);
	if (err < 0)
		pr_err("Fail to register switch-mid platform device.\n");
	return 0;
}

device_initcall(switch_mid_init);
