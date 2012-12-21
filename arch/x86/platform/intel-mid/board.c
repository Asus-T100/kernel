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
#include <linux/ipc_device.h>
#include <linux/i2c-gpio.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <asm/intel-mid.h>
#include <asm/intel_mid_pwm.h>

#include "board.h"

/*
 * IPC devices
 */

#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_pmic_audio.h"
#include "device_libs/platform_msic_adc.h"
#include "device_libs/platform_msic_battery.h"
#include "device_libs/platform_msic_gpio.h"
#include "device_libs/platform_msic_audio.h"
#include "device_libs/platform_msic_power_btn.h"
#include "device_libs/platform_msic_ocd.h"
#include "device_libs/platform_msic_vdd.h"
#include "device_libs/platform_mrfl_adc.h"
#include "device_libs/platform_mrfl_pmic.h"
#include "device_libs/platform_mrfl_ocd.h"
#include "device_libs/platform_hdmi.h"
#include <asm/platform_ctp_audio.h>
#include <asm/platform_mrfld_audio.h>

/*
 * I2C devices
 */

#include "device_libs/platform_max7315.h"
#include "device_libs/platform_tca6416.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_lsm303.h"
#include "device_libs/platform_emc1403.h"
#include "device_libs/platform_lis331.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_cyttsp.h"
#include "device_libs/platform_max17042.h"
#include "device_libs/platform_camera.h"
#include "device_libs/platform_mt9e013.h"
#include "device_libs/platform_mt9d113.h"
#include "device_libs/platform_mt9m114.h"
#include "device_libs/platform_mt9v113.h"
#include "device_libs/platform_ov5640.h"
#include "device_libs/platform_mxt224.h"
#include "device_libs/platform_a1026.h"
#include "device_libs/platform_lis3dh.h"
#include "device_libs/platform_ms5607.h"
#include "device_libs/platform_mpu3050.h"
#include "device_libs/platform_ltr502als.h"
#include "device_libs/platform_hmc5883.h"
#include "device_libs/platform_max11871.h"
#include "device_libs/platform_apds990x.h"
#include "device_libs/platform_apds9300.h"
#include "device_libs/platform_lm3554.h"
#include "device_libs/platform_ft5406.h"
#include "device_libs/platform_tpm502i.h"
#include "device_libs/platform_l3g4200d.h"
#include "device_libs/platform_smb347.h"
#include "device_libs/platform_mxt_ts.h"
#include "device_libs/platform_ltr301als.h"
#include "device_libs/platform_tc35876x.h"
#include "device_libs/platform_ov8830.h"
#include "device_libs/platform_l3g4200d.h"
#include "device_libs/platform_bq24192.h"
#include "device_libs/platform_s3202.h"
#include "device_libs/platform_bq24261.h"
#include "device_libs/platform_imx175.h"
#include "device_libs/platform_imx135.h"
#include "device_libs/platform_ov9724.h"
#include "device_libs/platform_lm3559.h"

/*
 * SPI devices
 */

#include "device_libs/platform_max3111.h"
#include "device_libs/platform_ektf2136.h"
#include "device_libs/platform_ntrig.h"
#include "device_libs/platform_ntrig_g4.h"

/*
 * HSI devices
 */

#include "device_libs/platform_hsi_modem.h"
#include "device_libs/platform_ffl_modem.h"
#include "device_libs/platform_edlp_modem.h"

/*
 * WIFI devices
 */

#include "device_libs/platform_wl12xx.h"
#include "device_libs/platform_bcm43xx.h"

/*
 * Bluetooth devices
 */

#include "device_libs/platform_btwilink.h"


static void __init *no_platform_data(void *info)
{
	return NULL;
}

/*
 * Todo: Need to remove this hack once battery driver
 *       is refactored.
 */

#ifdef CONFIG_X86_MRFLD
/* not supported */
int penwell_otg_query_charging_cap(void *dummy)
{
	return -1;
}
#endif

struct devs_id __initconst device_ids[] = {
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
#ifdef CONFIG_X86_MRFLD
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_vp_platform_data, NULL},
#else
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
#endif
	{"ntrig_spi", SFI_DEV_TYPE_SPI, 1, &ntrig_platform_data, NULL},
	{"ntrig_g4_spi", SFI_DEV_TYPE_SPI, 1, &ntrig_g4_platform_data, NULL},
#ifndef CONFIG_HSI_NO_MODEM
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
						NULL},

#endif
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* MSIC subdevices */
	{"a_gfreq",   SFI_DEV_TYPE_IPC, 0, &no_platform_data, NULL},
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
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
					     NULL},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &mrfl_adc_platform_data, NULL},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data, NULL},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1,
		&msic_power_btn_platform_data, NULL},
	{"bcove_chrgr", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"mrfld_cs42l73", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						NULL},
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_platform_data, NULL},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"hdmi", SFI_DEV_TYPE_IPC, 1, &hdmi_platform_data, NULL},



	/*
	 * I2C devices for camera image subsystem which will not be load into
	 * I2C core while initialize
	 */
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
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"mxt224", SFI_DEV_TYPE_I2C, 0, &mxt224_platform_data, NULL},
	{"max11871", SFI_DEV_TYPE_I2C, 0, &max11871_platform_data, NULL},
	{"ft5406", SFI_DEV_TYPE_I2C, 0, &ft5406_platform_data, NULL},
	{"tpm_i2c", SFI_DEV_TYPE_I2C, 0, &tpm502i_platform_data, NULL},
	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},
	{"accel", SFI_DEV_TYPE_I2C, 0, &lis3dh_platform_data, NULL},
	{"compass", SFI_DEV_TYPE_I2C, 0, &hmc5883_platform_data, NULL},
	{"gyro", SFI_DEV_TYPE_I2C, 0, &gyro_platform_data, NULL},
	{"baro", SFI_DEV_TYPE_I2C, 0, &ms5607_platform_data, NULL},
	{"als", SFI_DEV_TYPE_I2C, 0, &ltr502als_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"apds990x", SFI_DEV_TYPE_I2C, 0, &apds990x_platform_data, NULL},
	{"apds9300", SFI_DEV_TYPE_I2C, 0, &apds9300_platform_data, NULL},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data,
						NULL},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"mpu3050", SFI_DEV_TYPE_I2C, 1, &mpu3050_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"smb347", SFI_DEV_TYPE_I2C, 1, &smb347_platform_data, NULL},
	{"mxt1386", SFI_DEV_TYPE_I2C, 0, &mxt_ts_platform_data, NULL},
	{"mxt1386_wintek", SFI_DEV_TYPE_I2C, 0, &mxt_ts_platform_data, NULL},
	{"mxt1386_hanns", SFI_DEV_TYPE_I2C, 0, &mxt_ts_platform_data, NULL},
	{"ltr301", SFI_DEV_TYPE_I2C, 0, &ltr301als_platform_data, NULL},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &s3202_platform_data, NULL},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"smb349", SFI_DEV_TYPE_I2C, 1, &smb347_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},

	{},
};

/*
 * Identifies the type of the board using SPID and returns
 * the respective device_id ptr.
 * @ returns NULL for invalid device.
 */
/*
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

static struct kobject *board_properties;

static int __init intel_mid_board_properties(void)
{
	int ret = 0;

	board_properties = kobject_create_and_add("board_properties", NULL);
	if (!board_properties) {
		pr_err("failed to create /sys/board_properties\n");
		ret = -EINVAL;
	}

	return ret;
}

int intel_mid_create_property(const struct attribute *attr)
{
	if (!board_properties)
		return -EINVAL;

	return sysfs_create_file(board_properties, attr);
}


/*
 * TODO: This code should be removed once userspace stops using board ID
 *       and start using SPID.
 *
 */

int board_id_proc_show(struct seq_file *m, void *v)
{
	char *bid;
	switch (board_id) {
	case MFLD_BID_CDK:
		bid = "cdk";
		break;
	case MFLD_BID_AAVA:
		bid = "aava";
		break;
	case MFLD_BID_PR2_PROTO:
	case MFLD_BID_PR2_PNP:
		if (INTEL_MID_BOARD(1, TABLET, MFLD))
			bid = "redridge_dv20";
		else if (((INTEL_MID_BOARD(1, TABLET, CLVT)) ||
				(INTEL_MID_BOARD(1, PHONE, CLVTP))))
			bid = "ctp-vv";
		else
			bid = "pr2_proto";
		break;
	case MFLD_BID_PR2_VOLUME:
		if (INTEL_MID_BOARD(1, TABLET, MFLD))
			bid = "redridge_dv10";
		else
			bid = "pr2_volume";
		break;
	case MFLD_BID_PR3:
	case MFLD_BID_PR3_PNP:
		bid = "pr3";
		break;
	case MFLD_BID_RR_DV21:
		bid = "redridge_dv21";
		break;
	case MFLD_BID_SALITPA_EV1:
		bid = "salitpa_ev10";
		break;
	case CTP_BID_PR0:
		bid = "ctp-pr0";
		break;
	default:
		bid = "unknown";
		break;
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

u32 get_board_id(void)
{
	return board_id;
}
EXPORT_SYMBOL_GPL(get_board_id);

int __init board_proc_init(void)
{
	intel_mid_board_properties();

	proc_create("boardid", 0, NULL, &board_id_proc_fops);
	return 0;
}
early_initcall(board_proc_init);
