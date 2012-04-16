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

static bool msic_battery_check(void)
{
	if (get_oem0_table() == NULL) {
		pr_info("Invalid battery detected\n");
		return false;
	} else {
		pr_info("Valid battery detected\n");
		return true;
	}
}
void *bq24192_platform_data(void *info)
{
	static struct bq24192_platform_data platform_data;

	if (msic_battery_check())
		platform_data.sfi_tabl_present = true;
	else
		platform_data.sfi_tabl_present = false;
	platform_data.vHigh = 4176;
	platform_data.vLow = 4076;
	platform_data.fc_dcp_curr = 1500;
	platform_data.fc_sdp_curr = 500;
	platform_data.maint_chrg_curr = 500;
	platform_data.sfi_tabl_present = false;
	platform_data.slave_mode = 0;
	return &platform_data;
}
