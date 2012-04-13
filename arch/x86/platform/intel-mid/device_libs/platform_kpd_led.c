/*
 * platform_kpd_led.c: kpd_led platform data initilization file
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
#include "platform_kpd_led.h"

static int __init intel_kpd_led_init(void)
{
	int ret;
	struct ipc_board_info board_info;

	memset(&board_info, 0, sizeof(board_info));
	strncpy(board_info.name, DEVICE_NAME, 16);
	board_info.bus_id = IPC_SCU;
	board_info.id = -1;

	ret = ipc_new_device(&board_info);
	if (ret) {
		pr_err("failed to create ipc device: intel_kpd_led\n");
		return -1;
	}

	return 0;
}
fs_initcall(intel_kpd_led_init);
