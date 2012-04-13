/*
 * platform_mid_pwm.c: mid_pwm platform data initilization file
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
#include <linux/ipc_device.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_pwm.h>
#include "platform_mid_pwm.h"

static int __init intel_mid_pwm_init(void)
{
	int ret, i;
	struct ipc_board_info board_info;
	static struct intel_mid_pwm_platform_data mid_pwm_pdata;

	for (i = 0; i < PWM_NUM; i++) {
		mid_pwm_pdata.reg_clkdiv0[i] = MSIC_REG_PWM0CLKDIV0 + i * 2;
		mid_pwm_pdata.reg_clkdiv1[i] = MSIC_REG_PWM0CLKDIV1 + i * 2;
		mid_pwm_pdata.reg_dutycyc[i] = MSIC_REG_PWM0DUTYCYCLE + i;
	}

	memset(&board_info, 0, sizeof(board_info));
	strncpy(board_info.name, DEVICE_NAME, 16);
	board_info.bus_id = IPC_SCU;
	board_info.id = -1;
	board_info.platform_data = &mid_pwm_pdata;

	ret = ipc_new_device(&board_info);
	if (ret) {
		pr_err("failed to create ipc device: intel_mid_pwm\n");
		return -1;
	}

	return 0;
}
fs_initcall(intel_mid_pwm_init);

