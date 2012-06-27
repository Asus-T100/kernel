/*
 * platform_kpd_led_gpio.c: kpd_led_gpio platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/lnw_gpio.h>
#include <asm/intel_kpd_gpio_led.h>
#include <asm/intel-mid.h>
#include "platform_kpd_led_gpio.h"

static int __init intel_kpd_gpio_led_init(void)
{
	int ret;
	struct platform_device *pdev;
	static struct intel_kpd_gpio_led_pdata pdata;

	pdev = platform_device_alloc(DEVICE_NAME, -1);
	if (!pdev) {
		pr_err("Failed to create platform device: intel_kpd_led\n");
		return -ENOMEM;
	}

	ret = get_gpio_by_name("intel_kpd_led");
	if (ret == -1) {
		pr_err("Failed to get KPD LED gpio pin from SFI table\n");
		platform_device_put(pdev);
		return ret;
	} else
		pdata.gpio = ret;

	pdev->dev.platform_data = &pdata;

	return platform_device_add(pdev);
}
device_initcall(intel_kpd_gpio_led_init);

