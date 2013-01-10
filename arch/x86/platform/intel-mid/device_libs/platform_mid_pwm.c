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

static struct intel_mid_pwm_device_data mfld_pwms[] = {
	[PWM_LED] = {
		.reg_clkdiv0 = 0x62,
		.reg_clkdiv1 = 0x61,
		.reg_dutycyc = 0x67,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
	[PWM_VIBRATOR] = {
		.reg_clkdiv0 = 0x64,
		.reg_clkdiv1 = 0x63,
		.reg_dutycyc = 0x68,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
	[PWM_LCD_BACKLIGHT] = {
		.reg_clkdiv0 = 0x66,
		.reg_clkdiv1 = 0x65,
		.reg_dutycyc = 0x69,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
};

static struct intel_mid_pwm_device_data ctp_pwms[] = {
	[PWM_LED] = {
		.reg_clkdiv0 = 0x62,
		.reg_clkdiv1 = 0x61,
		.reg_dutycyc = 0x67,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x63,
	},
	[PWM_VIBRATOR] = {
		.reg_clkdiv0 = 0x64,
		.reg_clkdiv1 = 0x63,
		.reg_dutycyc = 0x68,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
	[PWM_LCD_BACKLIGHT] = {
		.reg_clkdiv0 = 0x66,
		.reg_clkdiv1 = 0x65,
		.reg_dutycyc = 0x69,
		.val_clkdiv1 = 0x00,
		.val_clkdiv0 = 0x03,
	},
};

static struct intel_mid_pwm_platform_data pdata[] = {
	[mfld_pwm] = {
		.pwm_num = PWM_NUM,
		.ddata = mfld_pwms,
	},
	[ctp_pwm] = {
		.pwm_num = PWM_NUM,
		.ddata = ctp_pwms,
	},
};

static void *get_pwm_platform_data()
{
	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
		(INTEL_MID_BOARD(1, TABLET, CLVT))) {
		pr_info("%s, CLV board detected\n", __func__);
		return &pdata[ctp_pwm];
	} else {
		pr_info("%s, MFLD board detected\n", __func__);
		return &pdata[mfld_pwm];
	}
}

static int __init intel_mid_pwm_init(void)
{
	int ret, i;
	struct ipc_board_info board_info;

	memset(&board_info, 0, sizeof(board_info));
	strncpy(board_info.name, DEVICE_NAME, 16);
	board_info.bus_id = IPC_SCU;
	board_info.id = -1;
	board_info.platform_data = get_pwm_platform_data();

	ret = ipc_new_device(&board_info);
	if (ret) {
		pr_err("failed to create ipc device: intel_mid_pwm\n");
		return -1;
	}

	return 0;
}
fs_initcall(intel_mid_pwm_init);

