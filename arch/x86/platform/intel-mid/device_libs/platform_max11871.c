/*
 * platform_max11871.c: max11871 platform data initilization file
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
#include <linux/lnw_gpio.h>
#include <linux/max11871.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <asm/intel-mid.h>
#include "platform_max11871.h"


static struct max11871_platform_data max11871_pdata = {
	.version = 0x101,

	.abs_x_min = 0,
	.abs_x_max = 320,
	.abs_y_min = 4,
	.abs_y_max = 484,
	.abs_z_min = 0,
	.abs_z_max = 255,

	.irq_flags = IRQF_TRIGGER_FALLING,
};

static int max11871_power(int on)
{
	if (on) {
		gpio_set_value(max11871_pdata.gpio_rst, 1);
		msleep(40);
	} else {
		gpio_set_value(max11871_pdata.gpio_rst, 0);
		msleep(20);
	}

	return 0;
}

static int max11871_board_init(void)
{
	int ret = 0;
	int gpio = max11871_pdata.gpio_irq;

	ret = gpio_request(gpio, "max11871_irq");
	if (ret < 0) {
		pr_err("%s: failed to request GPIO %d\n", __func__, gpio);
		return ret;
	}
	return gpio_direction_input(gpio);
}

static ssize_t max11871_virtual_keys_show(struct kobject *obj,
				struct kobj_attribute *attr, char *buf)
{
	/*
	 * virtual key format: version:code:centerX:centerY:width:height
	 * version must be 0x01, coordinate uses display unit
	 * (GI display size is 320 * 480), not raw touch unit
	 */

	return sprintf(buf,
			"0x01:" __stringify(KEY_BACK) ":40:520:60:60:"
			"0x01:" __stringify(KEY_HOME) ":120:520:60:60:"
			"0x01:" __stringify(KEY_SEARCH) ":200:520:60:60:"
			"0x01:" __stringify(KEY_MENU) ":280:520:60:60\n");
}

static struct kobj_attribute max11871_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.max11871_touchscreen_0",
		.mode = S_IRUGO,
	},
	.show = max11871_virtual_keys_show,
};


void *max11871_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = info;

	intel_mid_create_property(&max11871_virtual_keys_attr.attr);

	max11871_pdata.platform_hw_init = max11871_board_init,
	max11871_pdata.power = max11871_power,
	max11871_pdata.i2c_addr = i2c_info->addr;
	max11871_pdata.gpio_irq = get_gpio_by_name("ts_int");
	max11871_pdata.gpio_rst = get_gpio_by_name("ts_rst");

	return &max11871_pdata;
}
