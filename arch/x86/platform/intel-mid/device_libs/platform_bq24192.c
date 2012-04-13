/*
 * platform_bq24192.c: bq24192 platform data initilization file
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
#include <linux/power/bq24192_charger.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_bq24192.h"

void *bq24192_platform_data(void *info)
{
	static struct bq24192_platform_data platform_data;

	platform_data.slave_mode = 0;
	return &platform_data;
}
