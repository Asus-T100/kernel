/*
 * platform_ltr301als.c: ltr301als platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
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
#include <asm/intel-mid.h>
#include <linux/input/ltr301als.h>
#include "platform_ltr301als.h"

void *ltr301als_platform_data(void *info)
{
	static struct ltr301als_platform_data ltr301_pdata;
	struct i2c_board_info *i2c_info = info;
	int intr = get_gpio_by_name("AL-intr");

	/* store gpio number in i2c_board_info.irq */
	if (intr > 0)
		i2c_info->irq = intr;

	/* max opacity is 100, use 93% window loss */
	ltr301_pdata.window_opacity = 7;

	return &ltr301_pdata;
}


