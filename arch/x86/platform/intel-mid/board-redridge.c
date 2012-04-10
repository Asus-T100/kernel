/*
 * board-redridge.c: Intel Medfield based board (Redridge)
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
#include <linux/ipc_device.h>
#include <linux/i2c-gpio.h>

#include <asm/intel-mid.h>
#include <asm/intel_mid_pwm.h>
#include <asm/intel_scu_ipc.h>

/*
 * IPC devices
 */

#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_msic_adc.h"
#include "device_libs/platform_msic_gpio.h"
#include "device_libs/platform_msic_audio.h"
#include "device_libs/platform_msic_power_btn.h"
#include "device_libs/platform_msic_ocd.h"

/*
 * I2C devices
 */

#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_lis331.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_max17042.h"
#include "device_libs/platform_smb347.h"
#include "device_libs/platform_camera.h"
#include "device_libs/platform_mt9e013.h"
#include "device_libs/platform_mt9m114.h"
#include "device_libs/platform_mxt_ts.h"
#include "device_libs/platform_a1026.h"
#include "device_libs/platform_lis3dh.h"
#include "device_libs/platform_hmc5883.h"
#include "device_libs/platform_ms5607.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_ltr502als.h"
#include "device_libs/platform_ltr301als.h"
#include "device_libs/platform_tc35876x.h"
#include "device_libs/platform_lm3554.h"

/*
 * SPI devices
 */

#include "device_libs/platform_max3111.h"

/*
 * HSI devices
 */

#include "device_libs/platform_hsi_modem.h"

/*
 * WIFI devices
 */

#include "device_libs/platform_wl12xx.h"

/*
 * Bluetooth devices
 */

#include "device_libs/platform_btwilink.h"

static void __init *no_platform_data(void *info)
{
	return NULL;
}

const struct intel_v4l2_subdev_id v4l2_ids[] = {
	{"mt9e013", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"mt9m114", SOC_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{"lm3554", LED_FLASH, -1},
	{},
};

struct devs_id __initconst device_ids[] = {
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data, NULL},
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
					&ipc_device_handler},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"smb347", SFI_DEV_TYPE_I2C, 1, &smb347_platform_data, NULL},
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data,
						NULL},
	/* MSIC subdevices */
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},

	/*
	 * I2C devices for camera image subsystem which will not be load into
	 * I2C core while initialize
	 */
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data,
					&intel_ignore_i2c_device_register},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_ignore_i2c_device_register},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_ignore_i2c_device_register},
	{"mxt1386", SFI_DEV_TYPE_I2C, 0, &mxt_ts_platform_data, NULL},
	{"mxt1386_wintek", SFI_DEV_TYPE_I2C, 0, &mxt_ts_platform_data, NULL},
	{"mxt1386_hanns", SFI_DEV_TYPE_I2C, 0, &mxt_ts_platform_data, NULL},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},
	{"accel", SFI_DEV_TYPE_I2C, 0, &lis3dh_platform_data, NULL},
	{"compass", SFI_DEV_TYPE_I2C, 0, &hmc5883_platform_data, NULL},
	{"gyro", SFI_DEV_TYPE_I2C, 0, &gyro_platform_data, NULL},
	{"baro", SFI_DEV_TYPE_I2C, 0, &ms5607_platform_data, NULL},
	{"als", SFI_DEV_TYPE_I2C, 0, &ltr502als_platform_data, NULL},
	{"ltr301", SFI_DEV_TYPE_I2C, 0, &ltr301als_platform_data, NULL},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data},

	{},
};

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static int board_id_proc_show(struct seq_file *m, void *v)
{
	char *bid;

	switch (board_id) {
	case MFLD_BID_CDK:
		bid = "cdk";        break;
	case MFLD_BID_AAVA:
		bid = "aava";       break;
	case MFLD_BID_PR2_PROTO:
	case MFLD_BID_PR2_PNP:
		bid = "pr2_proto";  break;
	case MFLD_BID_PR2_VOLUME:
		bid = "pr2_volume"; break;
	case MFLD_BID_PR3:
	case MFLD_BID_PR3_PNP:
		bid = "pr3";        break;
	default:
		bid = "unknown";    break;
	}
	seq_printf(m, "boardid=%s\n", bid);

	return 0;
}

static int board_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, board_id_proc_show, NULL);
}

static const struct file_operations board_id_proc_fops = {
	.open		= board_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

u32 mfld_board_id(void)
{
	return board_id;
}
EXPORT_SYMBOL_GPL(mfld_board_id);

int __init board_proc_init(void)
{
	proc_create("boardid", 0, NULL, &board_id_proc_fops);
	return 0;
}

early_initcall(board_proc_init);
