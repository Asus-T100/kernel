/*
 * board-blackbay.c: Intel Medfield based board (Blackbay)
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/sfi.h>
#include <linux/lnw_gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/platform_device.h>
#include <linux/i2c-gpio.h>

#include <asm/intel-mid.h>

/*
 * IPC devices
 */
#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_msic.h"
#include "device_libs/platform_msic_battery.h"
#include "device_libs/platform_msic_gpio.h"
#include "device_libs/platform_msic_audio.h"
#include "device_libs/platform_msic_power_btn.h"
#include "device_libs/platform_msic_ocd.h"
#include "device_libs/platform_msic_thermal.h"
#include "device_libs/platform_msic_adc.h"

/*
 * I2C devices
 */
#include "device_libs/platform_max7315.h"
#include "device_libs/platform_tca6416.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_emc1403.h"
#include "device_libs/platform_lis331.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_tc35876x.h"
#include "device_libs/platform_max17042.h"
#include "device_libs/platform_mxt224.h"
#include "device_libs/platform_camera.h"
#include "device_libs/platform_mt9e013.h"
#include "device_libs/platform_mt9m114.h"

/*
 * SPI devices
 */
#include "device_libs/platform_max3111.h"

/*
 * HSU devices
 */
#include "device_libs/platform_hsu.h"

static void __init *no_platform_data(void *info)
{
	return NULL;
}

const struct intel_v4l2_subdev_id v4l2_ids[] = {
	{"mt9e013", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"mt9m114", SOC_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{},
};

struct devs_id __initconst device_ids[] = {
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"i2c_accel", SFI_DEV_TYPE_I2C, 0, &lis331dl_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data, NULL},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"st_kim", SFI_DEV_TYPE_UART, 0, &hsu_dev_platform_data, NULL},
	{"mxt224", SFI_DEV_TYPE_I2C, 0, &mxt224_platform_data, NULL},

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},

	/*
	 * I2C devices for camera image subsystem which will not be load into
	 * I2C core while initialize
	 */
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_ignore_i2c_device_register},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_ignore_i2c_device_register},
	{},
};
