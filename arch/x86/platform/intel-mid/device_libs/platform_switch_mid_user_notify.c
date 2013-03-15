/*
 * platform_switch_mid_user_notify.c: switch_mid user notify platform data
 * initilization file
 *
 * (C) Copyright 2012 Intel Corporation
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

static struct platform_device switch_device = {
	.name		= "switch-mid_user_notify",
	.id		= -1,
};

static int __init switch_mid_user_notify_init(void)
{
	int err;
	err = platform_device_register(&switch_device);
	if (err < 0)
		pr_err("Fail to register switch-mid_user_notify platform device.\n");
	return 0;
}

device_initcall(switch_mid_user_notify_init);
