/*
 * board-ctp.c: Intel Clovertrail based boards(EV, VV, PRx)
 *
 * (C) Copyright 2012 Intel Corporation
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

/*
 * IPC devices
 */

#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_pmic_audio.h"
#include "device_libs/platform_msic_adc.h"
#include "device_libs/platform_msic_gpio.h"
#include <asm/platform_clvs_audio.h>
#include "device_libs/platform_msic_power_btn.h"
#include "device_libs/platform_msic_vdd.h"
/*
* I2C devices
*/

#include "device_libs/platform_lsm303.h"
#include "device_libs/platform_apds990x.h"
#include "device_libs/platform_l3g4200d.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_bq24192.h"
#include "device_libs/platform_camera.h"
#include "device_libs/platform_mt9m114.h"
#include "device_libs/platform_ov8830.h"
#include "device_libs/platform_mxt224.h"
#include "device_libs/platform_a1026.h"
#include "device_libs/platform_lis3dh.h"
#include "device_libs/platform_s3202.h"
#include "device_libs/platform_max17042.h"
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
	{"ov8830", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"mt9m114", SOC_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{"lm3554", LED_FLASH, -1},
	{},
};

struct devs_id __initconst device_ids[] = {
	{"a_gfreq",   SFI_DEV_TYPE_IPC, 0, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},

	/* Fuel Gauge and charger */
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},

	/* Audio */
	{"clvcs_audio", SFI_DEV_TYPE_IPC, 1, &clvs_audio_platform_data,
					     NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},

	/* Modem and Wifi */
#ifndef CONFIG_HSI_NO_MODEM
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
#endif
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data,
						NULL},
	/*
	 * Camera Sensors and LED Flash.
	 * I2C devices for camera image subsystem which will not be load into
	 * I2C core while initialize
	 */
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_ignore_i2c_device_register},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_ignore_i2c_device_register},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_ignore_i2c_device_register},

	/* Touch Screen */
	{"mxt224", SFI_DEV_TYPE_I2C, 0, &mxt224_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &s3202_platform_data},

	/* NFC */
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},

	/* Sensors */
	{"apds990x", SFI_DEV_TYPE_I2C, 0, &apds990x_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{},
};

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
static int board_id_proc_show(struct seq_file *m, void *v)
{
	char *bid;

	switch (board_id) {
	case CTP_BID_VV:
		bid = "ctp-vv";
		break;
	case CTP_BID_PR0:
		bid = "ctp-pr0";
		break;
	default:
		bid = "unknown";
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

u32 ctp_board_id(void)
{
	return board_id;
}
EXPORT_SYMBOL_GPL(ctp_board_id);

int __init board_proc_init(void)
{
	proc_create("boardid", 0, NULL, &board_id_proc_fops);
	return 0;
}

early_initcall(board_proc_init);
