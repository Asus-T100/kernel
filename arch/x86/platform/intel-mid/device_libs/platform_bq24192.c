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

	/* Define the temperature ranges */
	platform_data.temp_mon_ranges = 4;
	platform_data.temp_mon_range[0].temp_up_lim = 60;
	platform_data.temp_mon_range[0].full_chrg_vol = 4100;
	platform_data.temp_mon_range[0].full_chrg_cur = 1500;
	platform_data.temp_mon_range[0].maint_chrg_vol_ll = 4000;
	platform_data.temp_mon_range[0].maint_chrg_vol_ul = 4050;
	platform_data.temp_mon_range[0].maint_chrg_cur = 950;

	platform_data.temp_mon_range[1].temp_up_lim = 45;
	platform_data.temp_mon_range[1].full_chrg_vol = 4200;
	platform_data.temp_mon_range[1].full_chrg_cur = 1500;
	platform_data.temp_mon_range[1].maint_chrg_vol_ll = 4126;
	platform_data.temp_mon_range[1].maint_chrg_vol_ul = 4200;
	platform_data.temp_mon_range[1].maint_chrg_cur = 950;

	platform_data.temp_mon_range[2].temp_up_lim = 10;
	platform_data.temp_mon_range[2].full_chrg_vol = 4200;
	platform_data.temp_mon_range[2].full_chrg_cur = 1500;
	platform_data.temp_mon_range[2].maint_chrg_vol_ll = 4126;
	platform_data.temp_mon_range[2].maint_chrg_vol_ul = 4200;
	platform_data.temp_mon_range[2].maint_chrg_cur = 950;

	platform_data.temp_mon_range[3].temp_up_lim = 0;
	platform_data.temp_mon_range[3].full_chrg_vol = 3950;
	platform_data.temp_mon_range[3].full_chrg_cur = 950;
	platform_data.temp_mon_range[3].maint_chrg_vol_ll = 3900;
	platform_data.temp_mon_range[3].maint_chrg_vol_ul = 3950;
	platform_data.temp_mon_range[3].maint_chrg_cur = 950;

	platform_data.temp_low_lim = -10;
	platform_data.slave_mode = 0;
	return &platform_data;
}
