/*
 * platform_s3202.c: s3202 platform data initilization file
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
#include <linux/interrupt.h>
#include <linux/lnw_gpio.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
#include <linux/rmi_i2c.h>
#else
#include <linux/synaptics_i2c_rmi4.h>
#endif
#include <asm/intel-mid.h>
#include "platform_s3202.h"

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
static struct rmi_f11_functiondata synaptics_f11_data = {
	.swap_axes = true,
};

static unsigned char synaptic_keys[31] = {1, 2, 3, 4,};
			/* {KEY_BACK,KEY_MENU,KEY_HOME,KEY_SEARCH,} */

static struct rmi_button_map synaptics_button_map = {
	.nbuttons = 31,
	.map = synaptic_keys,
};
static struct rmi_f19_functiondata  synaptics_f19_data = {
	.button_map = &synaptics_button_map,
};

static struct rmi_functiondata synaptics_functiondata[] = {
	{
		.function_index = RMI_F11_INDEX,
		.data = &synaptics_f11_data,
	},
	{
		.function_index = RMI_F19_INDEX,
		.data = &synaptics_f19_data,
	},
};

static struct rmi_functiondata_list synaptics_perfunctiondata = {
	.count = ARRAY_SIZE(synaptics_functiondata),
	.functiondata = synaptics_functiondata,
};


static struct rmi_sensordata s3202_sensordata = {
	.perfunctiondata = &synaptics_perfunctiondata,
};
#endif

void *s3202_platform_data(void *info)
{
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
	struct i2c_board_info *i2c_info = info;
	static struct rmi_i2c_platformdata s3202_platform_data = {
		.delay_ms = 50,
		.sensordata = &s3202_sensordata,
	};

	s3202_platform_data.i2c_address = i2c_info->addr;
	s3202_sensordata.attn_gpio_number = get_gpio_by_name("ts_int");
	s3202_sensordata.rst_gpio_number  = get_gpio_by_name("ts_rst");
#else
	static const struct rmi4_touch_calib calibs[] = {
		/* TOUCH_TYPE_OGS */
		{
			.swap_axes = true,
			.customer_id = 20121109,
			.fw_name = "s3202_ogs.img",
			.key_dev_name = "rmi4_key",
		},
		/* TOUCH_TYPE_GFF */
		{
			.swap_axes = false,
			.customer_id = 20121109,
			.fw_name = "s3202_gff.img",
			.key_dev_name = "rmi4_key_gff",
		},
	};

	static struct rmi4_platform_data s3202_platform_data = {
		.irq_type = IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT,
		.regulator_en = true,
		.regulator_name = "vemmc2",
		.calibs = calibs,
	};

	s3202_platform_data.int_gpio_number = get_gpio_by_name("ts_int");
	s3202_platform_data.rst_gpio_number = get_gpio_by_name("ts_rst");

#endif
	return &s3202_platform_data;
}
