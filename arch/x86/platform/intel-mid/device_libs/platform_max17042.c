/*
 * platform_max17042.c: max17042 platform data initilization file
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
#include <linux/power_supply.h>
#include <linux/power/max17042_battery.h>
#include <linux/power/intel_mdf_battery.h>
#include <linux/power/smb347-charger.h>
#include <linux/power/bq24192_charger.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include <asm/intel_scu_ipc.h>
#include "platform_max17042.h"

void max17042_i2c_reset_workaround(void)
{
/* toggle clock pin of I2C-1 to recover devices from abnormal status.
 * currently, only max17042 on I2C-1 needs such workaround */
#define I2C_1_GPIO_PIN 27
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_GPIO);
	gpio_direction_output(I2C_1_GPIO_PIN, 0);
	gpio_set_value(I2C_1_GPIO_PIN, 1);
	udelay(10);
	gpio_set_value(I2C_1_GPIO_PIN, 0);
	udelay(10);
	lnw_gpio_set_alt(I2C_1_GPIO_PIN, LNW_ALT_1);
#undef I2C_1_GPIO_PIN
}
EXPORT_SYMBOL(max17042_i2c_reset_workaround);

static bool msic_battery_check(void)
{
	if (get_oem0_table() == NULL) {
		pr_info("invalid battery detected\n");
		return false;
	} else {
		pr_info("valid battery detected\n");
		return true;
	}
	return false;
}


#define UMIP_REF_FG_TBL			0x806	/* 2 bytes */
#define BATT_FG_TBL_BODY		14	/* 144 bytes */
/**
 * mfld_fg_restore_config_data - restore config data
 * @name : Power Supply name
 * @data : config data output pointer
 * @len : length of config data
 *
 */
int mfld_fg_restore_config_data(const char *name, void *data, int len)
{
	int mip_offset, ret;

	/* Read the fuel gauge config data from umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_read_mip((u8 *)data, len, mip_offset, 0);

	return ret;
}
EXPORT_SYMBOL(mfld_fg_restore_config_data);

/**
 * mfld_fg_save_config_data - save config data
 * @name : Power Supply name
 * @data : config data input pointer
 * @len : length of config data
 *
 */
int mfld_fg_save_config_data(const char *name, void *data, int len)
{
	int mip_offset, ret;

	/* write the fuel gauge config data to umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_write_umip((u8 *)data, len, mip_offset);

	return ret;
}
EXPORT_SYMBOL(mfld_fg_save_config_data);

static uint16_t ctp_cell_char_tbl[] = {
	/* Data to be written from 0x80h */
	0x9d70, 0xb720, 0xb940, 0xba50, 0xbba0, 0xbc70, 0xbce0, 0xbd40,
	0xbe60, 0xbf60, 0xc1e0, 0xc470, 0xc700, 0xc970, 0xcce0, 0xd070,

	/* Data to be written from 0x90h */
	0x00c0, 0x1730, 0x0690, 0x1260, 0x2a00, 0x3100, 0x3a30, 0x15a0,
	0x18f0, 0x0c80, 0x0c20, 0x0b10, 0x0b60, 0x07e0, 0x07a0, 0x07a0,

	/* Data to be written from 0xA0h */
	0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
	0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100, 0x0100,
};

static void ctp_fg_restore_config_data(const char *name, void *data, int len)
{
	struct max17042_config_data *fg_cfg_data =
				(struct max17042_config_data *)data;
	fg_cfg_data->cfg = 0x2210;
	fg_cfg_data->learn_cfg = 0x0076;
	fg_cfg_data->filter_cfg = 0x87a4;
	fg_cfg_data->relax_cfg = 0x506b;
	memcpy(&fg_cfg_data->cell_char_tbl, ctp_cell_char_tbl,
					sizeof(ctp_cell_char_tbl));
	fg_cfg_data->rcomp0 = 0x0047;
	fg_cfg_data->tempCo = 0x1920;
	fg_cfg_data->etc = 0x00e0;
	fg_cfg_data->kempty0 = 0x0100;
	fg_cfg_data->ichgt_term = 0x0240;
	fg_cfg_data->full_cap = 3400;
	fg_cfg_data->design_cap = 3400;
	fg_cfg_data->full_capnom = 3400;
	fg_cfg_data->soc_empty = 0x0060;
	fg_cfg_data->rsense = 1;
}
EXPORT_SYMBOL(ctp_fg_restore_config_data);

static int ctp_fg_save_config_data(const char *name, void *data, int len)
{
	return 0;
}
EXPORT_SYMBOL(ctp_fg_save_config_data);

void *max17042_platform_data(void *info)
{
	static struct max17042_platform_data platform_data;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
#ifdef CONFIG_BOARD_CTP /* TODO: get rid of this... */
	int intr = get_gpio_by_name("max17042");
#else
	int intr = get_gpio_by_name("max_fg_alert");
#endif

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;

	if (msic_battery_check()) {
		platform_data.enable_current_sense = true;
		platform_data.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	} else {
		platform_data.enable_current_sense = false;
		platform_data.technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	platform_data.is_init_done = 0;
	platform_data.reset_i2c_lines = max17042_i2c_reset_workaround;
#ifdef CONFIG_BATTERY_INTEL_MDF /* blackbay, not redridge, not ctp */
	platform_data.current_sense_enabled =
		intel_msic_is_current_sense_enabled;
	platform_data.battery_present = intel_msic_check_battery_present;
	platform_data.battery_health = intel_msic_check_battery_health;
	platform_data.battery_status = intel_msic_check_battery_status;
	platform_data.battery_pack_temp = intel_msic_get_battery_pack_temp;
	platform_data.save_config_data = intel_msic_save_config_data;
	platform_data.restore_config_data = intel_msic_restore_config_data;

	platform_data.is_cap_shutdown_enabled =
					intel_msic_is_capacity_shutdown_en;
	platform_data.is_volt_shutdown_enabled = intel_msic_is_volt_shutdown_en;
	platform_data.is_lowbatt_shutdown_enabled =
					intel_msic_is_lowbatt_shutdown_en;
	platform_data.get_vmin_threshold = intel_msic_get_vsys_min;
#endif
#ifdef CONFIG_BOARD_REDRIDGE /* TODO: get rid of this */
	platform_data.enable_current_sense = true;
	platform_data.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	platform_data.restore_config_data = mfld_fg_restore_config_data;
	platform_data.save_config_data = mfld_fg_save_config_data;
#endif
#ifdef CONFIG_BOARD_CTP
	platform_data.enable_current_sense = true;
	platform_data.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	platform_data.restore_config_data = ctp_fg_restore_config_data;
	platform_data.save_config_data = ctp_fg_save_config_data;
#endif
#ifdef CONFIG_CHARGER_SMB347 /* redridge dv10 */
	platform_data.battery_status = smb347_get_charging_status;
#endif
#ifdef CONFIG_CHARGER_BQ24192 /* clovertrail */
	platform_data.battery_status = bq24192_query_battery_status;
#endif

	return &platform_data;
}
