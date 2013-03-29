/*
 * platform_msic_gpio.c: MSIC GPIO platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_msic.h"
#include "platform_msic_gpio.h"

void __init *msic_gpio_platform_data(void *info)
{
	struct platform_device *pdev = NULL;
	struct sfi_device_table_entry *entry = info;
	static struct intel_msic_gpio_pdata msic_gpio_pdata;
	int ret;
	int gpio;

	pdev = platform_device_alloc(MSIC_GPIO_DEVICE_NAME, -1);

	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
					MSIC_GPIO_DEVICE_NAME);
		goto out;
	}

	gpio = get_gpio_by_name("msic_gpio_base");

	if (gpio < 0)
		return NULL;

	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
		INTEL_MID_BOARD(1, TABLET, CLVT)) {
		msic_gpio_pdata.ngpio_lv = 1;
		msic_gpio_pdata.ngpio_hv = 8;
	}

	msic_gpio_pdata.can_sleep = 1;
	msic_gpio_pdata.gpio_base = gpio;

	pdev->dev.platform_data = &msic_gpio_pdata;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add msic gpio platform device\n");
		platform_device_put(pdev);
		goto out;
	}

	install_irq_resource(pdev, entry->irq);

	register_rpmsg_service("rpmsg_msic_gpio", RPROC_SCU, RP_MSIC_GPIO);

out:
	return &msic_gpio_pdata;
}

