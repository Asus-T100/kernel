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
#include <linux/sfi.h>
#include <linux/power/bq24192_charger.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/battery_id.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_bq24192.h"
#include <linux/usb/otg.h>

static void *chgr_gpadc_handle;

static struct bq24192_platform_data platform_data;
static struct ps_pse_mod_prof *ps_pse_mod_prof;
static struct ps_batt_chg_prof *ps_batt_chrg_prof;
static struct power_supply_throttle bq24192_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = BQ24192_CHRG_CUR_NOLIMIT,

	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = BQ24192_CHRG_CUR_HIGH,

	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = BQ24192_CHRG_CUR_MEDIUM,

	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGING,
	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGER,
	},

};

char *bq24192_supplied_to[] = {
	"max170xx_battery",
	"max17042_battery",
	"max17047_battery",
};

static int const bptherm_curve_data[BPTHERM_CURVE_MAX_SAMPLES]
[BPTHERM_CURVE_MAX_VALUES] = {
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 977, 961},
	{-10, -15, 961, 941},
	{-5, -10, 941, 917},
	{0, -5, 917, 887},
	{5, 0, 887, 853},
	{10, 5, 853, 813},
	{15, 10, 813, 769},
	{20, 15, 769, 720},
	{25, 20, 720, 669},
	{30, 25, 669, 615},
	{35, 30, 615, 561},
	{40, 35, 561, 508},
	{45, 40, 508, 456},
	{50, 45, 456, 407},
	{55, 50, 407, 357},
	{60, 55, 357, 315},
	{65, 60, 315, 277},
	{70, 65, 277, 243},
	{75, 70, 243, 212},
	{80, 75, 212, 186},
	{85, 80, 186, 162},
	{90, 85, 162, 140},
	{100, 90, 140, 107},
};

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

void *platform_init_battery_adc(int num_sensors, int chan_number, int flag)
{
	pr_debug("%s\n", __func__);

	/* Allocate ADC Channels */
	return
		(chgr_gpadc_handle = intel_mid_gpadc_alloc(num_sensors,
				  chan_number | flag));
}

static void platform_get_sfi_batt_table(void *table)
{
	struct sfi_table_simple *sb =
			 (struct sfi_table_simple *)get_oem0_table();
	struct platform_batt_profile *batt_prof;

	int num_entries, i;

	pr_debug("%s\n", __func__);

	if (sb == NULL) {
		pr_debug("Invalid Battery detected\n");
		return;
	}

	num_entries = SFI_GET_NUM_ENTRIES(sb, struct platform_batt_profile);
	batt_prof = (struct platform_batt_profile *)table;

	pr_debug("%s: num_entries = %d\n", __func__, num_entries);

	if (num_entries) {
		memcpy(batt_prof, sb->pentry,
		num_entries * sizeof(struct platform_batt_profile));
	} else {
		batt_prof->temp_mon_ranges = 0;
		memcpy((void *)batt_prof->batt_id,
			(void *)"UNKNOWN", 8);
	}

	/* Allocate the memory for sharing battery profile */
	ps_pse_mod_prof = kzalloc(
				sizeof(*ps_pse_mod_prof),
				GFP_KERNEL);
	if (!ps_pse_mod_prof) {
		pr_debug("%s: Error in kzalloc\n", __func__);
		kfree(ps_pse_mod_prof);
		ps_pse_mod_prof = NULL;
		return;
	}

	ps_batt_chrg_prof = kzalloc(
				sizeof(*ps_batt_chrg_prof),
				GFP_KERNEL);
	if (!ps_batt_chrg_prof) {
		pr_debug("%s: Error in kzalloc\n", __func__);
		kfree(ps_batt_chrg_prof);
		kfree(ps_pse_mod_prof);
		ps_batt_chrg_prof = NULL;
		ps_pse_mod_prof = NULL;
		return;
	}

	memcpy(ps_pse_mod_prof->batt_id, batt_prof->batt_id, BATTID_STR_LEN);
	ps_pse_mod_prof->battery_type = batt_prof->battery_type;
	ps_pse_mod_prof->capacity = batt_prof->capacity;
	ps_pse_mod_prof->voltage_max = batt_prof->voltage_max;
	ps_pse_mod_prof->temp_mon_ranges = batt_prof->temp_mon_ranges;
	ps_pse_mod_prof->chrg_term_mA = 128;
	ps_pse_mod_prof->low_batt_mV = 3400;
	ps_pse_mod_prof->disch_tmp_ul = 60;
	ps_pse_mod_prof->disch_tmp_ll = 0;

	for (i = 0; i < batt_prof->temp_mon_ranges; i++) {
		ps_pse_mod_prof->temp_mon_range[i].temp_up_lim =
			batt_prof->temp_mon_range[i].temp_up_lim;
		ps_pse_mod_prof->temp_mon_range[i].full_chrg_vol =
			batt_prof->temp_mon_range[i].full_chrg_vol;
		ps_pse_mod_prof->temp_mon_range[i].full_chrg_cur =
			batt_prof->temp_mon_range[i].full_chrg_cur;
		ps_pse_mod_prof->temp_mon_range[i].maint_chrg_vol_ll =
			batt_prof->temp_mon_range[i].maint_chrg_vol_ll;
		ps_pse_mod_prof->temp_mon_range[i].maint_chrg_vol_ll =
			batt_prof->temp_mon_range[i].maint_chrg_vol_ll;
	}

	/* Change need in SMIP since temp zone 4 has all 0's */
	ps_pse_mod_prof->temp_mon_range[0].temp_up_lim = 60;

	ps_batt_chrg_prof->chrg_prof_type = PSE_MOD_CHRG_PROF;
	ps_batt_chrg_prof->batt_prof = ps_pse_mod_prof;

#ifdef CONFIG_POWER_SUPPLY_BATTID
	battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED,
				ps_batt_chrg_prof);
#endif

}

/* returns the max and min temp in which battery is suppose to operate */
static void platform_get_batt_temp_thresholds(
		struct platform_batt_profile *batt_profile,
		short int *temp_high, short int *temp_low)
{
	int i, max_range;
	struct platform_temp_mon_table *temp_mon_tabl = NULL;
	pr_debug("%s:%d:\n", __func__, __LINE__);

	if (temp_high == NULL)
		return;
	if (temp_low == NULL)
		return;

	*temp_high = *temp_low = 0;

	temp_mon_tabl = batt_profile->temp_mon_range;
	max_range = batt_profile->temp_mon_ranges;

	for (i = 0; i < max_range; i++) {
		if (*temp_high < temp_mon_tabl[i].temp_up_lim)
			*temp_high = temp_mon_tabl[i].temp_up_lim;
	}

	for (i = 0 ; i < max_range; i++) {
		if (*temp_low > temp_mon_tabl[i].temp_up_lim)
			*temp_low = temp_mon_tabl[i].temp_low_lim;
	}
}

/**
 * init_batt_thresholds - initialize battery thresholds
 * @chip: charger driver device context
 * Context: can sleep
 */
static void platform_init_battery_threshold(u8 total_temp_range,
	 struct platform_batt_safety_param *safe_param,
	 struct platform_batt_profile *batt_profile)
{
	int ret;
	u8 validate_smip_data[4];

	pr_debug("%s:%d:\n", __func__, __LINE__);

	platform_get_sfi_batt_table(batt_profile);

	safe_param->vbatt_sh_min = BATT_VMIN_THRESHOLD_DEF;
	safe_param->vbatt_crit = BATT_CRIT_CUTOFF_VOLT_DEF;
	safe_param->temp_high = BATT_TEMP_MAX_DEF;
	safe_param->temp_low = BATT_TEMP_MIN_DEF;

	/* Read the Signature verification data form SRAM */
	ret = intel_scu_ipc_read_mip((u8 *)validate_smip_data,
			4,
			SMIP_SRAM_OFFSET_ADDR, 1);
	if (ret) {
		pr_warn("EM:%s:%d:smip read failed\n", __func__, __LINE__);
		return;
	}

	pr_debug("EM:%s: SBCT_REV: 0x%x RSYS_OHMS 0x%x", __func__,
			validate_smip_data[0], validate_smip_data[3]);

	/* Check for Valid SMIP data */
	if ((validate_smip_data[0] == SBCT_REV) && (
				validate_smip_data[3] == RSYS_MOHMS)) {

		pr_debug("EM:%s: Valid SMIP data\n", __func__);
		/* Read the threshold values from SRAM */
		ret = intel_scu_ipc_read_mip((u8 *)safe_param,
				sizeof(struct platform_batt_safety_param),
				SMIP_SRAM_OFFSET_ADDR, 1);
		if (ret) {
			pr_warn("EM:%s:smip read failed\n", __func__);
			return;
		}

		pr_info("%s:sh_min - %x\n", __func__,
					safe_param->vbatt_sh_min);
		/* Read the battery properties from SRAM */
		/* BATTID_STR_LEN -> length of the battid
		 *  +1 -> number of temp_mon_ranges
		 *  SMIP_SRAM_BATT_PROP_OFFSET+4 -> ignore b1idmin and b1idmax
		 */
	}

	if (batt_profile->temp_mon_ranges == 0) {
		/* BELOW CODE WILL BE USED IN INVALID BATTERY
		   DO WE NEED TO USE CHARGE PROFILE FOR INVALID BATTERY? */
		batt_profile->temp_mon_ranges = total_temp_range;
		/* store the default parameters */
		/* Define the temperature ranges */
		if (batt_profile->temp_mon_ranges == 4) {
			batt_profile->temp_mon_range[0].temp_up_lim = 60;
			batt_profile->temp_mon_range[0].temp_low_lim = 45;
			batt_profile->temp_mon_range[0].full_chrg_vol = 4200;
			batt_profile->temp_mon_range[0].full_chrg_cur = 1216;
		batt_profile->temp_mon_range[0].maint_chrg_vol_ll = 4100;
		batt_profile->temp_mon_range[0].maint_chrg_vol_ul = 4200;
			batt_profile->temp_mon_range[0].maint_chrg_cur = 1216;

			batt_profile->temp_mon_range[1].temp_up_lim = 45;
			batt_profile->temp_mon_range[1].temp_low_lim = 0;
			batt_profile->temp_mon_range[1].full_chrg_vol = 4320;
			batt_profile->temp_mon_range[1].full_chrg_cur = 1216;
		batt_profile->temp_mon_range[1].maint_chrg_vol_ll = 4220;
		batt_profile->temp_mon_range[1].maint_chrg_vol_ul = 4320;
			batt_profile->temp_mon_range[1].maint_chrg_cur = 1216;
			batt_profile->temp_mon_range[2].temp_up_lim = 10;
			batt_profile->temp_mon_range[2].full_chrg_vol = 4320;
			batt_profile->temp_mon_range[2].full_chrg_cur = 1216;
		batt_profile->temp_mon_range[2].maint_chrg_vol_ll = 4220;
		batt_profile->temp_mon_range[2].maint_chrg_vol_ul = 4320;
			batt_profile->temp_mon_range[2].maint_chrg_cur = 1216;

			batt_profile->temp_mon_range[3].temp_up_lim = 0;
			batt_profile->temp_mon_range[3].temp_low_lim = -10;
			batt_profile->temp_mon_range[3].full_chrg_vol = 0;
			batt_profile->temp_mon_range[3].full_chrg_cur = 0;
		batt_profile->temp_mon_range[3].maint_chrg_vol_ll = 0;
		batt_profile->temp_mon_range[3].maint_chrg_vol_ul = 0;
			batt_profile->temp_mon_range[3].maint_chrg_cur = 0;
		}
	}

	platform_get_batt_temp_thresholds(batt_profile,
					&safe_param->temp_high,
					&safe_param->temp_low);

}

static int initialize_platform_data(void)
{

	pr_debug("%s:\n", __func__);

	chgr_gpadc_handle = platform_init_battery_adc(
			BATT_NUM_GPADC_SENSORS,	GPADC_BPTHERM_CHNUM,
			CH_NEED_VCALIB | CH_NEED_VREF);

	if (chgr_gpadc_handle == NULL) {
		pr_err("%s: unable to get the adc value\n", __func__);
		return -1;
	}

	platform_init_battery_threshold(TEMP_NR_RNG,
				&platform_data.safety_param,
				&platform_data.batt_profile);

	return 0;
}

static int platform_get_irq_number(void)
{
	int irq;
	pr_debug("%s:\n", __func__);
	irq = gpio_to_irq(CHGR_INT_N);
	pr_debug("%s:%d:irq = %d\n", __func__, __LINE__, irq);
	return irq;
}

static int platform_drive_vbus(bool onoff)
{
	int chrg_otg_gpio = BQ24192_CHRG_OTG_GPIO, ret = 0;
	static bool is_gpio_request;

	pr_debug("%s:%d:\n", __func__, __LINE__);

	if (!is_gpio_request) {
		ret = gpio_request(chrg_otg_gpio, "CHRG_OTG");
		if (ret) {
			pr_warn("%s:Failed to request gpio: error %d\n",
				__func__, ret);
			return ret;
		} else {
			pr_info("request gpio %d for CHRG_OTG pin\n",
							chrg_otg_gpio);
			is_gpio_request = true;
		}
	}

	/* assert the chrg_otg gpio now */
	if (onoff)
		gpio_direction_output(chrg_otg_gpio, 1);
	else {
		/* de-assert the chrg_otg gpio now */
		gpio_direction_output(chrg_otg_gpio, 0);
		gpio_direction_input(chrg_otg_gpio);
	}
	return ret;
}

/* Temperature conversion Macros */
static int platform_conv_adc_temp(int adc_val,
	int adc_max, int adc_diff, int temp_diff)
{
	int ret;

	pr_debug("%s\n", __func__);
	ret = (adc_max - adc_val) * temp_diff;
	return ret / adc_diff;
}

/**
 * ctp_adc_to_temp - convert ADC code to temperature
 * @adc_val : ADC sensor reading
 * @tmp : finally read temperature
 *
 * Returns 0 on success or -ERANGE in error case
 */
static int platform_adc_to_temp(uint16_t adc_val, int *tmp,
				const int bptherm_curve_data
			[BPTHERM_CURVE_MAX_SAMPLES][BPTHERM_CURVE_MAX_VALUES])
{
	int temp = 0;
	int i;

	pr_debug("%s\n", __func__);

	/*
	 * If the value returned as an ERANGE the battery icon shows an
	 * exclaimation mark in the COS.In order to fix the issue, if
	 * the ADC returns a value which is not in range specified, we
	 * update the value within the bound.
	 */
	if (adc_val > BPTHERM_ADC_MAX)
		adc_val = BPTHERM_ADC_MAX;
	else if (adc_val < BPTHERM_ADC_MIN)
		adc_val = BPTHERM_ADC_MIN;

	for (i = 0; i < BPTHERM_CURVE_MAX_SAMPLES; i++) {
		/* linear approximation for battery pack temperature */
		if (adc_val >= bptherm_curve_data[i][3] &&
			 adc_val <= bptherm_curve_data[i][2]) {
			temp = platform_conv_adc_temp(adc_val,
					bptherm_curve_data[i][2],
					 bptherm_curve_data[i][2] -
					 bptherm_curve_data[i][3],
					 bptherm_curve_data[i][0] -
					bptherm_curve_data[i][1]);

			temp += bptherm_curve_data[i][1];
			break;
		}
	}

	if (i >= BPTHERM_CURVE_MAX_SAMPLES) {
		pr_warn("EM: %s:Invalid temp adc range\n", __func__);
		pr_warn("EM: adc_val =%d\n", adc_val);
		return -EINVAL;
	}

	*tmp = temp;

	return 0;
}

/**
 * ctp_read_adc_temp - read ADC sensor to get the temperature
 * @tmp: op parameter where temperature get's read
 *
 * Returns 0 if success else -1 or -ERANGE
 */
static int platform_read_adc_temp(int *temp,
			const int bptherm_curve_data
			[BPTHERM_CURVE_MAX_SAMPLES][BPTHERM_CURVE_MAX_VALUES])
{
	int gpadc_sensor_val = 0;
	int ret;

	pr_debug("%s, handle = %x\n", __func__,
				(unsigned int)chgr_gpadc_handle);

	if (chgr_gpadc_handle == NULL) {
		ret = -ENODEV;
		goto read_adc_exit;
	}

	ret = intel_mid_gpadc_sample(chgr_gpadc_handle,
				GPADC_BPTHERM_SAMPLE_COUNT,
				&gpadc_sensor_val);
	if (ret) {
		pr_err("EM:%s:adc driver api returned error(%d)\n",
							__func__, ret);
		goto read_adc_exit;
	}

	ret = platform_adc_to_temp(gpadc_sensor_val, temp, bptherm_curve_data);
read_adc_exit:
	return ret;
}

/* returns the battery pack temperature read from adc */
static int platform_get_battery_pack_temp(int *temp)
{
	pr_debug("%s\n", __func__);
	return platform_read_adc_temp(temp, bptherm_curve_data);
}

static void platform_free_data(void)
{
	pr_debug("%s\n", __func__);
	if (chgr_gpadc_handle)
		intel_mid_gpadc_free(chgr_gpadc_handle);

	kfree(ps_batt_chrg_prof);
	kfree(ps_pse_mod_prof);
}

void *bq24192_platform_data(void *info)
{
	static struct bq24192_platform_data platform_data;

	pr_debug("%s:\n", __func__);
	if (msic_battery_check())
		platform_data.sfi_tabl_present = true;
	else
		platform_data.sfi_tabl_present = false;

	platform_data.throttle_states = bq24192_throttle_states;
	platform_data.supplied_to = bq24192_supplied_to;
	platform_data.num_throttle_states = ARRAY_SIZE(bq24192_throttle_states);
	platform_data.num_supplicants = ARRAY_SIZE(bq24192_supplied_to);
	platform_data.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB;
	platform_data.init_platform_data = initialize_platform_data;
	platform_data.get_irq_number = platform_get_irq_number;
	platform_data.drive_vbus = platform_drive_vbus;
	platform_data.get_battery_pack_temp = NULL;
	platform_data.query_otg = NULL;
	platform_data.free_platform_data = platform_free_data;
	platform_data.slave_mode = 0;

	register_rpmsg_service("rpmsg_bq24192", RPROC_SCU,
				RP_BQ24192);
	return &platform_data;
}
