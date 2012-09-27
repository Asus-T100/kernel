/*
 * platform_mxt224.c: mxt224 platform data initilization file
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
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/atmel_mxt224.h>
#include "platform_mxt224.h"

void *mxt224_platform_data(void *info)
{
	static struct mxt_platform_data mxt_pdata;

	mxt_pdata.numtouch       = 10;
	mxt_pdata.max_x          = 1023;
	mxt_pdata.max_y          = 975;
	mxt_pdata.orientation    = MXT_MSGB_T9_ORIENT_HORZ_FLIP;
	mxt_pdata.reset          = get_gpio_by_name("ts_rst");
	mxt_pdata.irq            = get_gpio_by_name("ts_int");

	return &mxt_pdata;
}
