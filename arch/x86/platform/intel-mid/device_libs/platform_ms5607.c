/*
 * platform_ms5607.c: ms5607 platform data initilization file
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
#include <asm/intel-mid.h>
#include <linux/ms5607.h>
#include "platform_ms5607.h"

void *ms5607_platform_data(void *info)
{
	static struct ms5607_platform_data ms5607_pdata;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;

	i2c_info->irq = 0xff;

	ms5607_pdata.poll_interval = 100;
	ms5607_pdata.min_interval  = 0;

	return &ms5607_pdata;
}
