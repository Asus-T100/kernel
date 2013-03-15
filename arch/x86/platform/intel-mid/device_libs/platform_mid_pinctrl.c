/*
 * platform_mid_pinctrl.c: Intel MID pinctrl platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
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
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/intel_scu_flis.h>
#include <asm/intel_mid_pinctrl.h>
#include "platform_mid_pinctrl.h"

static struct intel_mid_pinctrl_platform_data ctp_pinctrl_pdata = {
	.name = "ctp_pinctrl",
	.pin_t = ctp_pin_table,
	.pin_num = CTP_PIN_NUM,
};

static void *get_pinctrl_platform_data(void)
{
	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
		INTEL_MID_BOARD(1, TABLET, CLVT)) {
		pr_info("%s, CTP platform detected\n", __func__);
		return &ctp_pinctrl_pdata;
	} else {
		return NULL;
	}
}

static int __init intel_mid_pinctrl_init(void)
{
	int ret;
	struct platform_device *pdev = NULL;

	pdev = platform_device_alloc(PINCTRL_DEVICE_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for platform dev %s\n",
					PINCTRL_DEVICE_NAME);
		ret = -EINVAL;
		goto out;
	}

	pdev->dev.platform_data = get_pinctrl_platform_data();

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add pinctrl platform device\n");
		platform_device_put(pdev);
		goto out;
	}

	pr_info("intel_mid_pinctrl platform device created\n");
out:
	return ret;
}
subsys_initcall(intel_mid_pinctrl_init);

