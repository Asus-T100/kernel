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
#include <linux/power/bq24261_charger.h>
#include <asm/pmic_pdata.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include <asm/intel_scu_ipc.h>
#include "platform_max17042.h"

void max17042_i2c_reset_workaround(void)
{
/* toggle clock pin of I2C to recover devices from abnormal status.
 * currently, only max17042 on I2C needs such workaround */
#if defined(CONFIG_BATTERY_INTEL_MDF)
#define I2C_GPIO_PIN 27
#elif defined(CONFIG_BOARD_CTP)
#define I2C_GPIO_PIN 29
#elif defined(CONFIG_X86_MRFLD)
#define I2C_GPIO_PIN 21
#else
#define I2C_GPIO_PIN 27
#endif
	lnw_gpio_set_alt(I2C_GPIO_PIN, LNW_GPIO);
	gpio_direction_output(I2C_GPIO_PIN, 0);
	gpio_set_value(I2C_GPIO_PIN, 1);
	udelay(10);
	gpio_set_value(I2C_GPIO_PIN, 0);
	udelay(10);
	lnw_gpio_set_alt(I2C_GPIO_PIN, LNW_ALT_1);
}
EXPORT_SYMBOL(max17042_i2c_reset_workaround);


static bool msic_battery_check(struct max17042_platform_data *pdata)
{
	struct sfi_table_simple *sb;
	char *mrfl_batt_str = "INTN0001";

	sb = (struct sfi_table_simple *)get_oem0_table();
	if (sb == NULL) {
		pr_info("invalid battery detected\n");
		snprintf(pdata->battid, BATTID_LEN + 1, "UNKNOWNB");
		snprintf(pdata->serial_num, SERIAL_NUM_LEN + 1, "000000");
		return false;
	} else {
		pr_info("valid battery detected\n");
		/* First entry in OEM0 table is the BATTID. Read battid
		 * if pentry is not NULL and header length is greater
		 * than BATTID length*/
		if (sb->pentry && sb->header.len >= BATTID_LEN) {
			if (!((INTEL_MID_BOARD(1, TABLET, MRFL)) ||
				(INTEL_MID_BOARD(1, PHONE, MRFL)))) {
				snprintf(pdata->battid, BATTID_LEN + 1, "%s",
						(char *)sb->pentry);
			} else {
				if (strncmp((char *)sb->pentry,
					"PG000001", (BATTID_LEN)) == 0) {
					snprintf(pdata->battid,
						(BATTID_LEN + 1),
							"%s", mrfl_batt_str);
				} else {
					snprintf(pdata->battid,
						(BATTID_LEN + 1),
						"%s", (char *)sb->pentry);
				}
			}

			/* First 2 bytes represent the model name
			 * and the remaining 6 bytes represent the
			 * serial number. */
			if (pdata->battid[0] == 'I' &&
				pdata->battid[1] >= '0'
					&& pdata->battid[1] <= '9') {
				unsigned char tmp[SERIAL_NUM_LEN + 2];
				int i, offset;
				snprintf(pdata->model_name,
					(MODEL_NAME_LEN) + 1,
						"%s", pdata->battid);
				memcpy(tmp, sb->pentry, BATTID_LEN);
				for (i = 0; i < SERIAL_NUM_LEN; i++) {
					sprintf(pdata->serial_num + i*2,
					"%02x", tmp[i + MODEL_NAME_LEN]);
				}
				pdata->serial_num[2 * SERIAL_NUM_LEN + 1]
									= '\0';

			} else {
				snprintf(pdata->model_name,
					(MODEL_NAME_LEN + 1),
						"%s", pdata->battid);
				snprintf(pdata->serial_num,
					(SERIAL_NUM_LEN + 1), "%s",
				pdata->battid + MODEL_NAME_LEN);
			}
		}
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
	0x0090, 0x1020, 0x04A0, 0x0D10, 0x1DC0, 0x22E0, 0x2940, 0x0F60,
	0x11B0, 0x08E0, 0x08A0, 0x07D0, 0x0820, 0x0590, 0x0570, 0x0570,

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
	fg_cfg_data->full_cap = 3408;
	fg_cfg_data->design_cap = 3408;
	fg_cfg_data->full_capnom = 3408;
	fg_cfg_data->soc_empty = 0x0060;
	fg_cfg_data->rsense = 1;
	fg_cfg_data->cycles = 0x160;
}
EXPORT_SYMBOL(ctp_fg_restore_config_data);

static int ctp_fg_save_config_data(const char *name, void *data, int len)
{
	return 0;
}
EXPORT_SYMBOL(ctp_fg_save_config_data);

int mrfl_get_bat_health(void)
{

	int pbat_health = -ENODEV;
	int bqbat_health = -ENODEV;
#ifdef CONFIG_BQ24261_CHARGER
	 bqbat_health = bq24261_get_bat_health();
#endif
#ifdef CONFIG_PMIC_CCSM
	pbat_health = pmic_get_health();
#endif

	/*Battery temperature exceptions are reported to PMIC. ALl other
	* exceptions are reported to bq24261 charger. Need to read the
	* battery health reported by both drivers, before reporting
	* the actual battery health
	*/

	/* FIXME: need to have a time stamp based implementation to
	* report battery health
	*/

	if (pbat_health < 0 && bqbat_health < 0)
		return pbat_health;
	if (pbat_health > 0 && pbat_health != POWER_SUPPLY_HEALTH_GOOD)
		return pbat_health;
	else
		return bqbat_health;
}

void *max17042_platform_data(void *info)
{
	static struct max17042_platform_data platform_data;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;
	int intr = get_gpio_by_name("max_fg_alert");

	i2c_info->irq = intr + INTEL_MID_IRQ_OFFSET;

	if (msic_battery_check(&platform_data)) {
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
	platform_data.temp_min_lim = 0;
	platform_data.temp_max_lim = 60;
	platform_data.volt_min_lim = 3200;
	platform_data.volt_max_lim = 4300;
	platform_data.restore_config_data = mfld_fg_restore_config_data;
	platform_data.save_config_data = mfld_fg_save_config_data;
#endif
#ifdef CONFIG_BOARD_CTP
	platform_data.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	platform_data.file_sys_storage_enabled = 1;
	platform_data.battery_health = ctp_get_battery_health;
	platform_data.is_volt_shutdown_enabled = ctp_is_volt_shutdown_enabled;
	platform_data.get_vmin_threshold = ctp_get_vsys_min;
	platform_data.soc_intr_mode_enabled = true;
#endif
#ifdef CONFIG_CHARGER_SMB347 /* redridge dv10 */
	/* smb347 charger driver needs to be ported to k3.4 by FT
	 * comment out this line to get salitpa compiled for now
	platform_data.battery_status = smb347_get_charging_status; */
#endif
#ifdef CONFIG_CHARGER_BQ24192 /* clovertrail */
	platform_data.battery_status = ctp_query_battery_status;
	platform_data.battery_pack_temp = ctp_get_battery_pack_temp;
#endif

#ifdef CONFIG_X86_MRFLD
	platform_data.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	platform_data.file_sys_storage_enabled = 1;
	platform_data.battery_health = mrfl_get_bat_health;
	platform_data.soc_intr_mode_enabled = true;
#endif
#ifdef CONFIG_PMIC_CCSM
	platform_data.battery_pack_temp = pmic_get_battery_pack_temp;
#endif
	return &platform_data;
}
