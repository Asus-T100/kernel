/*
 * byt_ulpmc_battery.h - Battery driver for baytrail ULPMC chip
 *
 * Copyright (C) 2013 Intel Corporation
 * Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __BYT_ULPMC_BATTERY_H_
#define __BYT_ULPMC_BATTERY_H_

#define BATTID_LEN		8
#define EXTCON_NAME_LEN		16

enum ulpmc_chip_version {BYTULPMCFGV3, BYTULPMCFGV4};

struct ulpmc_platform_data {
	int gpio;
	char battid[BATTID_LEN];
	int volt_sh_min;
	int cc_lim0;	/* default charge current */
	int cc_lim1;	/* charge current limit 1 */
	int cc_lim2;	/* charge current limit 2 */
	int cc_lim3;	/* charge current limit 3 */
	enum ulpmc_chip_version version;
	char extcon_devname[EXTCON_NAME_LEN];
};

#ifdef CONFIG_BYT_ULPMC_BATTERY
extern void ulpmc_fwupdate_enter(void);
extern void ulpmc_fwupdate_exit(void);
extern struct i2c_client *ulpmc_get_i2c_client(void);
extern int byt_ulpmc_suspend_sdp_charging(void);
extern int byt_ulpmc_reset_charger(void);
#else
static void ulpmc_fwupdate_enter(void){ }
static void ulpmc_fwupdate_exit(void) { }
static struct i2c_client *ulpmc_get_i2c_client(void)
{
	return NULL;
}
int byt_ulpmc_suspend_sdp_charging(void)
{
	return 0;
}
int byt_ulpmc_reset_charger(void)
{
	return 0;
}
#endif

#endif /* __BYT_ULPMC_BATTERY_H_ */
