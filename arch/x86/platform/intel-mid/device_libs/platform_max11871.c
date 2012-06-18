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

#define MAX11871_GPIO_ERROR(ret, gpio, op)                                 \
	if (ret < 0) {                                                     \
		pr_err("%s: GPIO %d %s failed (%d)\n", __func__, gpio, op, \
			ret);                                              \
		return ret;                                                \
	}
static int max11871_init(struct max11871_pdata *pdata, int value)
{
	int  ret;

	if (value) {
		ret = gpio_request(pdata->gpio_tirq, "max11871_tirq");
		MAX11871_GPIO_ERROR(ret, pdata->gpio_tirq, "request");
		ret = gpio_direction_input(pdata->gpio_tirq);
		MAX11871_GPIO_ERROR(ret, pdata->gpio_tirq, "direction");

		ret = gpio_request(pdata->gpio_reset, "max11871_reset");
		MAX11871_GPIO_ERROR(ret, pdata->gpio_reset, "request");
		ret = gpio_direction_output(pdata->gpio_reset, 1);
		MAX11871_GPIO_ERROR(ret, pdata->gpio_reset, "direction");
	} else {
		gpio_free(pdata->gpio_tirq);
		gpio_free(pdata->gpio_reset);
	}

	return 0;
}

static int max11871_reset(struct max11871_pdata *pdata, int value)
{
	gpio_set_value(pdata->gpio_reset, !!value);
	return 0;
}

static int max11871_tirq(struct max11871_pdata *pdata)
{
	return gpio_get_value(pdata->gpio_tirq);
}

#if MAX11871_BOARD_CONFIG
struct max11871_config max11871_config = {
	.chip_configs = 3,
	.chip_config[0] = {.config_id = 0x0CFD, .fw_mappings = 1,
			.fw_mapping[0] = {.chip_id = 0x55, .fw_index = 0} },
	.chip_config[1] = {.config_id = 0x0002, .fw_mappings = 2,
			.fw_mapping[0] = {.chip_id = 0x55, .fw_index = 1},
			.fw_mapping[1] = {.chip_id = 0x57, .fw_index = 2} },
	.chip_config[2] = {.config_id = 0x0003, .fw_mappings = 2,
			.fw_mapping[0] = {.chip_id = 0x55, .fw_index = 3},
			.fw_mapping[1] = {.chip_id = 0x57, .fw_index = 4} },
	.fw_image[0] = {.file_name = "max11871_16x10_RevE.bin",
			.length = 0x8000, .config_boundary = 0x7A68},
	.fw_image[1] = {.file_name = "max11871_17x11_BYD_RevE.bin",
			.length = 0x8000, .config_boundary = 0x7A68},
	.fw_image[2] = {.file_name = "max11871_17x11_BYD_RevG.bin",
			.length = 0x8000, .config_boundary = 0x7A68},
	.fw_image[3] = {.file_name = "max11871_17x11_Truly_RevE.bin",
			.length = 0x8000, .config_boundary = 0x7A68},
	.fw_image[4] = {.file_name = "max11871_17x11_Truly_RevG.bin",
			.length = 0x8000, .config_boundary = 0x7A68},
	.default_chip_config = 0x0002,
	.default_chip_id = 0x57,
	.i2c_words = MAX_WORDS_REPORT,
	.max_touches = 10,
	.events_per_sec = 120,
	.coordinate_settings = MAX11871_REVERSE_Y | MAX11871_SWAP_XY,
	.coordinate_model = MAX11871_OPTIMAL,
	.coordinates[MAX11871_OPTIMAL] = {
		.panel_mx_l = 0,
		.panel_mx_h = 0,
		.panel_my_l = 0,
		.panel_my_h = 50,
		.lcd_x = 320,
		.lcd_y = 480,
		.button_xy[0] = {.x = 41, .y = 518, .size_x = 60,
				 .size_y = 60},
		.button_xy[1] = {.x = 120, .y = 518, .size_x = 60,
				 .size_y = 60},
		.button_xy[2] = {.x = 200, .y = 518, .size_x = 60,
				 .size_y = 60},
		.button_xy[3] = {.x = 275, .y = 518, .size_x = 60,
				 .size_y = 60} },
	.buttons_enabled = 0,
	.buttons_type = MAX11871_BUTTONS_XY,
	.buttons = 4,
	.button_code = {KEY_BACK, KEY_HOME, KEY_SEARCH, KEY_MENU},
	.input_protocol = MAX11871_PROTOCOL_A_TRACK
};

struct max11871_pdata max11871_pdata = {
	.config     = &max11871_config,
#else
struct max11871_pdata max11871_data = {
	.config     = NULL,
#endif
	.init       = max11871_init,
	.reset      = max11871_reset,
	.tirq       = max11871_tirq,
};

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
		.name = "virtualkeys." MAX11871_TOUCH,
		.mode = S_IRUGO,
	},
	.show = max11871_virtual_keys_show,
};

void *max11871_platform_data(void *info)
{
/* ideally need to do gpio_to_irq() here but currently it crashes; to be
   resolved later */
#if 0
	struct i2c_board_info *i2c_info = info;
#endif

	intel_mid_create_property(&max11871_virtual_keys_attr.attr);

	max11871_pdata.gpio_tirq = get_gpio_by_name("ts_int");
	max11871_pdata.gpio_reset = get_gpio_by_name("ts_rst");
#if 0
	i2c_info->irq = gpio_to_irq(max11871_pdata.gpio_tirq);
#endif

	return &max11871_pdata;
}

