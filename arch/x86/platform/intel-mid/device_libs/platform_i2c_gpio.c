/*
 * platform_i2c_gpio.c: i2c_gpio platform data initilization file
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
#include <linux/i2c-gpio.h>
#include <linux/platform_device.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_i2c_gpio.h"

static int hdmi_i2c_workaround(void)
{
	int ret;
	struct platform_device *pdev;
	struct i2c_gpio_platform_data *pdata;

	/*
	 * Hard code a gpio controller platform device to take over
	 * the two gpio pins used to be controlled by i2c bus 3.
	 * This is to support HDMI EDID extension block read, which
	 * is not supported by the current i2c controller, so we use
	 * GPIO pin the simulate an i2c bus.
	 */
	pdev = platform_device_alloc(DEVICE_NAME, 8);
	if (!pdev) {
		pr_err("i2c-gpio: failed to alloc platform device\n");
		ret = -ENOMEM;
		goto out;
	}

	pdata = kzalloc(sizeof(struct i2c_gpio_platform_data), GFP_KERNEL);
	if (!pdata) {
		pr_err("i2c-gpio: failed to alloc platform data\n");
		kfree(pdev);
		ret = -ENOMEM;
		goto out;
	}
	pdata->scl_pin = 35 + 96;
	pdata->sda_pin = 36 + 96;
	pdata->sda_is_open_drain = 0;
	pdata->scl_is_open_drain = 0;
	pdev->dev.platform_data = pdata;

	platform_device_add(pdev);

	lnw_gpio_set_alt(pdata->sda_pin, LNW_GPIO);
	lnw_gpio_set_alt(pdata->scl_pin, LNW_GPIO);

out:
	return ret;
}
rootfs_initcall(hdmi_i2c_workaround);

