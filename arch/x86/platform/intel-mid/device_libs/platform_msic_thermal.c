/*
 * platform_msic_thermal.c: msic_thermal platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/ipc_device.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_thermal.h>
#include <asm/intel_mid_gpadc.h>
#include "platform_msic_thermal.h"

static struct intel_mid_thermal_sensor ctp_sensors[];

/* ctp skin1 dependent sensor information */
static struct intel_mid_thermal_sensor *ctp_skin1_sensors[] = {
		&ctp_sensors[3]
};

/* ctp skin1 private info */
static struct skin1_private_info ctp_skin1_info = {
	.dependent = 1,
	.sensors = ctp_skin1_sensors,
};

/* ctp thermal sensor list */
static struct intel_mid_thermal_sensor ctp_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = 0,
		.slope = 446,
		.intercept = 14050,
		.adc_channel = 0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 1,
		.slope = 723,
		.intercept = 7084,
		.adc_channel = 0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
		.priv = &ctp_skin1_info,
		.temp_correlation = skin1_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 2,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},
	{
		.name = BPTHERM_NAME,
		.index = 3,
		.slope = 788,
		.intercept = 5065,
		.adc_channel = 0x09 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = bptherm_temp_correlation,
		.direct = false,
	},
	{
		.name = SYSTHERM1_NAME,
		.index = 4,
		.slope = 1000,
		.intercept = 0,
		.adc_channel = 0x05 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},

};

/* mfld thermal sensor list */
static struct intel_mid_thermal_sensor mfld_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = 0,
		.slope = 851,
		.intercept = 2800,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 1,
		.slope = 806,
		.intercept = 1800,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin1_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_SYS_NAME,
		.index = 2,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x0A | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 3,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},
	{
		.name = SYSTHERM1_NAME,
		.index = 4,
		.slope = 1000,
		.intercept = 0,
		.adc_channel = 0x09 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin1_temp_correlation,
		.direct = false,
	},

};

/* LEX thermal sensor list */
static struct intel_mid_thermal_sensor lex_sensors[] = {
	{
		.name = SKIN0_NAME,
		.index = 0,
		.slope = 820,
		.intercept = 3000,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin0_temp_correlation,
		.direct = false,
	},
	{
		.name = SKIN1_NAME,
		.index = 1,
		.slope = 850,
		.intercept = 3000,
		.adc_channel = 0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin1_temp_correlation,
		.direct = false,
	},
	{
		.name = MSIC_SYS_NAME,
		.index = 2,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x0A | CH_NEED_VREF | CH_NEED_VCALIB,
		.direct = false,
	},
	{
		.name = MSIC_DIE_NAME,
		.index = 3,
		.slope = 0,
		.intercept = 0,
		.adc_channel = 0x03 | CH_NEED_VCALIB,
		.direct = true,
	},
	{
		.name = SYSTHERM1_NAME,
		.index = 4,
		.slope = 1000,
		.intercept = 0,
		.adc_channel = 0x09 | CH_NEED_VREF | CH_NEED_VCALIB,
		.temp_correlation = skin1_temp_correlation,
		.direct = false,
	},

};

static struct intel_mid_thermal_platform_data pdata[] = {
	[mfld_thermal] = {
		.num_sensors = 5,
		.sensors = mfld_sensors,
		.soc_cooling = false,
	},
	[ctp_thermal] = {
		.num_sensors = 5,
		.sensors = ctp_sensors,
		.soc_cooling = true,
	},
	[lex_thermal] = {
		.num_sensors = 5,
		.sensors = lex_sensors,
		.soc_cooling = false,
	},

};

void *msic_thermal_platform_data()
{
	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
			(INTEL_MID_BOARD(1, TABLET, CLVT))) {
		pr_info("msic_thermal_platform_data: CLV board detected\n");
		return &pdata[ctp_thermal];
	} else if (INTEL_MID_BOARD(2, PHONE, MFLD, LEX, ENG) ||
			(INTEL_MID_BOARD(2, PHONE, MFLD, LEX, PRO))) {
		pr_info("msic_thermal_platform_data: LEX board detected\n");
		return &pdata[lex_thermal];
	} else {
		pr_info("msic_thermal_platform_data: MFLD board detected\n");
		return &pdata[mfld_thermal];
	}
}

static int __init intel_msic_thermal_init(void)
{
	int ret;
	struct ipc_board_info board_info;

	memset(&board_info, 0, sizeof(board_info));
	strncpy(board_info.name, MSIC_THERM_DEV_NAME, 16);
	board_info.bus_id = IPC_SCU;
	board_info.id = -1;
	board_info.platform_data = msic_thermal_platform_data();

	ret = ipc_new_device(&board_info);
	if (ret) {
		pr_err("failed to create ipc device: msic_thermal\n");
		return -1;
	}

	return 0;
}
fs_initcall(intel_msic_thermal_init);
