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
#include "device_libs/platform_msic_vdd.h"
#include "device_libs/platform_mrfl_ocd.h"
#include "device_libs/platform_msic_thermal.h"
#include "device_libs/platform_soc_thermal.h"
#include "device_libs/platform_msic_adc.h"
#include <asm/platform_ctp_audio.h>
#include "device_libs/platform_bcove_adc.h"
#include <asm/platform_byt_audio.h>
#include "device_libs/platform_mrfl_pmic.h"
#include "device_libs/platform_mrfl_thermal.h"
#include "device_libs/platform_mrfl_pmic_i2c.h"
#include <asm/platform_mrfld_audio.h>
#include "device_libs/platform_vlv2_plat_clk.h"

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
#include "device_libs/platform_mt9d113.h"
#include "device_libs/platform_mt9m114.h"
#include "device_libs/platform_lm3554.h"
#include "device_libs/platform_mt9v113.h"
#include "device_libs/platform_ov5640.h"
#include "device_libs/platform_imx175.h"
#include "device_libs/platform_imx135.h"
#include "device_libs/platform_s5k8aay.h"
#include "device_libs/platform_ov9724.h"
#include "device_libs/platform_ov2722.h"
#include "device_libs/platform_lm3559.h"
#include "device_libs/platform_a1026.h"
#include "device_libs/platform_rmi4.h"
#include "device_libs/platform_bq24192.h"
#include "device_libs/platform_ov8830.h"
#include "device_libs/platform_hmc5883.h"
#include "device_libs/platform_ms5607.h"
#include "device_libs/platform_lis3dh.h"
#include "device_libs/platform_bq24261.h"
#include "device_libs/platform_r69001.h"
#include "device_libs/platform_wm5102.h"

/* SPI devices */
#include "device_libs/platform_max3111.h"

/* HSI devices */
#include "device_libs/platform_hsi_modem.h"
#include "device_libs/platform_ffl_modem.h"
#include "device_libs/platform_edlp_modem.h"
#include "device_libs/platform_edlp_fast.h"
#include "device_libs/platform_logical_modem.h"

/* Modem devices */
#include "device_libs/platform_modem_ctrl.h"

/* WIFI devices */
#include "device_libs/platform_wl12xx.h"
#include "device_libs/platform_bcm43xx.h"


static void __init *no_platform_data(void *info)
{
	return NULL;
}

struct devs_id __initconst device_ids[] = {

	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
#ifndef CONFIG_HSI_NO_MODEM
	{"logical_hsi", SFI_DEV_TYPE_HSI, 0, &logical_platform_data, NULL},
#endif

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
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},

	/* IPC devices */
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
						&ipc_device_handler},
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&ipc_device_handler},
	{"a_gfreq",   SFI_DEV_TYPE_IPC, 0, &no_platform_data,
						&ipc_device_handler},
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"merr_prh_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&soc_thrm_device_handler},
	{"vlv2_plat_clk", SFI_DEV_TYPE_IPC, 1,
		&vlv2_plat_clk_device_platform_data, &ipc_device_handler},
	{"byt_rt5642", SFI_DEV_TYPE_IPC, 1, &byt_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},
	{"rt5640", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"mxt224", SFI_DEV_TYPE_I2C, 0, &mxt224_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
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
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"gyro", SFI_DEV_TYPE_I2C, 0, &gyro_platform_data, NULL},
	{"als", SFI_DEV_TYPE_I2C, 0, &ltr502als_platform_data, NULL},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"dw9719", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	/* Modem */
#ifndef CONFIG_HSI_NO_MODEM
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
						NULL},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data, NULL},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_FFRD", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
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

struct devs_id __init *get_device_ptr(void)
{
	return device_ids;
}

