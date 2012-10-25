/*
 * board-merr_vv.c: Intel Merrifield VV board (SilverRidge)
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Bin Gao <bin.gao@intel.com>,
*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>

#include <asm/intel-mid.h>

#include "device_libs/platform_ipc.h"
#include "device_libs/platform_mrfl_adc.h"
#include "device_libs/platform_max3111.h"
#include "device_libs/platform_mrfl_pmic.h"
#include <asm/platform_mrfld_audio.h>
/* I2C Devices */
#include "device_libs/platform_max17042.h"
#include "device_libs/platform_bq24261.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_camera.h"
#include "device_libs/platform_imx175.h"
#include "device_libs/platform_ov9724.h"
#include "device_libs/platform_lm3559.h"

static void __init *no_platform_data(void *info)
{
	return NULL;
}

const struct intel_v4l2_subdev_id v4l2_ids[] = {
	{"imx175", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"ov9724", RAW_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{"lm3559", LED_FLASH, -1},
	{},
};

/*
 * WIFI devices
 */

#include "device_libs/platform_bcm43xx.h"

struct devs_id __initconst device_ids[] = {
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &mrfl_adc_platform_data, NULL},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"bcove_chrgr", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_vp_platform_data, NULL},

#ifdef CONFIG_BQ24261_CHARGER
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
#endif
#ifdef CONFIG_PMIC_CCSM
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_platform_data, NULL},
#endif
#ifdef CONFIG_I2C_PMIC
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
#endif
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	/*
	 * Camera Sensors and LED Flash.
	 * I2C devices for camera image subsystem which will not be load into
	 * I2C core while initialize
	 */
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_ignore_i2c_device_register},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_ignore_i2c_device_register},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_ignore_i2c_device_register},
	{},
};

/* not supported */
int penwell_otg_query_charging_cap(void *dummy)
{
	return -1;
}
