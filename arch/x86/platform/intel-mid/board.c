/*
 * board.c: Intel MID board file
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

#include "board.h"

/* IPC devices */
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
#include <asm/platform_ctp_audio.h>

/* I2C devices */
#include "device_libs/platform_apds990x.h"
#include "device_libs/platform_l3g4200d.h"
#include "device_libs/platform_max7315.h"
#include "device_libs/platform_tca6416.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_lsm303.h"
#include "device_libs/platform_ltr502als.h"
#include "device_libs/platform_emc1403.h"
#include "device_libs/platform_lis331.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_tc35876x.h"
#include "device_libs/platform_max17042.h"
#include "device_libs/platform_mxt224.h"
#include "device_libs/platform_camera.h"
#include "device_libs/platform_mt9e013.h"
#include "device_libs/platform_mt9m114.h"
#include "device_libs/platform_a1026.h"
#include "device_libs/platform_s3202.h"
#include "device_libs/platform_bq24192.h"
#include "device_libs/platform_ov8830.h"
#include "device_libs/platform_hmc5883.h"
#include "device_libs/platform_ms5607.h"
#include "device_libs/platform_lis3dh.h"

/* SPI devices */
#include "device_libs/platform_max3111.h"

/* HSI devices */
#include "device_libs/platform_hsi_modem.h"

/* HSU devices */
#include "device_libs/platform_hsu.h"

static void __init *no_platform_data(void *info)
{
	return NULL;
}

struct devs_id __initconst device_ids[] = {
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"st_kim", SFI_DEV_TYPE_UART, 0, &hsu_dev_platform_data, NULL},

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

	/* IPC devices */
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
						&ipc_device_handler},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&ipc_device_handler},
	{"a_gfreq",   SFI_DEV_TYPE_IPC, 0, &no_platform_data,
						&ipc_device_handler},
	{"clvcs_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices */
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"mxt224", SFI_DEV_TYPE_I2C, 0, &mxt224_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &s3202_platform_data},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"i2c_accel", SFI_DEV_TYPE_I2C, 0, &lis331dl_platform_data, NULL},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data, NULL},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"compass", SFI_DEV_TYPE_I2C, 0, &hmc5883_platform_data, NULL},
	{"baro", SFI_DEV_TYPE_I2C, 0, &ms5607_platform_data, NULL},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"accel", SFI_DEV_TYPE_I2C, 0, &lis3dh_platform_data, NULL},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"apds990x", SFI_DEV_TYPE_I2C, 0, &apds990x_platform_data},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"gyro", SFI_DEV_TYPE_I2C, 0, &gyro_platform_data, NULL},
	{"als", SFI_DEV_TYPE_I2C, 0, &ltr502als_platform_data, NULL},

	/* Modem */
#ifndef CONFIG_HSI_NO_MODEM
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
#endif

	{},
};

/*
 * Identifies the type of the board using SPID and returns
 * the respective device_id ptr.
 * @ returns NULL for invalid device.
 *
 * In case if you want to override the default device_id table,
 * Create a new device id table in board file and assign it to
 * device pointer.
 *
 * Example
 *	#ifdef CONFIG_BOARD_XXXX
 *		if (INTEL_MID_BOARD(1,PHONE,XXXX))
 *			dev_ptr = XXXX_device_ids;
 *	#endif
 */

struct devs_id *get_device_ptr(void)
{
	struct devs_id *dev_ptr = device_ids;

	return dev_ptr;
}

