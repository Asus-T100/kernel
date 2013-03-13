/*
 * platform_s3400.c: s3400 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/sfi.h>
#include <linux/interrupt.h>
#include <linux/lnw_gpio.h>
#include <linux/synaptics_i2c_rmi4.h>
#include <asm/intel-mid.h>
#include "platform_s3400.h"

void *s3400_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = info;
	static struct rmi4_touch_calib calib;

	strncpy(calib.type, i2c_info->type, SFI_NAME_LEN);
	if (!strncmp(i2c_info->type, SFI_S3400_CGS, SFI_NAME_LEN)) {
		/* S3400_CGS */
		calib.swap_axes = true;
		calib.customer_id = 1342177280;
		calib.fw_name = "s3400_cgs.img";
		calib.key_dev_name = "rmi4_key";
	} else {
		/* S3400_IGZO */
		calib.swap_axes = true;
		calib.customer_id = 1342177280;
		calib.fw_name = "s3400_igzo.img";
		calib.key_dev_name = "rmi4_key";
	}

	static struct rmi4_platform_data s3400_platform_data = {
		.irq_type = IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT,
		.regulator_en = true,
		.regulator_name = "vemmc2",
		.calib = &calib,
	};

	s3400_platform_data.int_gpio_number = get_gpio_by_name("ts_int");
	s3400_platform_data.rst_gpio_number = get_gpio_by_name("ts_rst");

	return &s3400_platform_data;
}
