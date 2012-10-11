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
#include <linux/i2c-gpio.h>

#include <asm/intel-mid.h>

/*
 * IPC devices
 */
#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_msic.h"
#include "device_libs/platform_msic_adc.h"
#include "device_libs/platform_msic_gpio.h"
#include "device_libs/platform_clvs_audio.h"
#include "device_libs/platform_msic_power_btn.h"

/*
 * I2C devices
 */
#include "device_libs/platform_mxt224.h"
#include "device_libs/platform_max17042.h"

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

struct devs_id __initconst device_ids[] = {
	{"a_gfreq",   SFI_DEV_TYPE_IPC, 0, &no_platform_data,
					&ipc_device_handler},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"st_kim", SFI_DEV_TYPE_UART, 0, &hsu_dev_platform_data, NULL},

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},

	/* Fuel Gauge and charger */
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},

	/* Audio */
	{"clvcs_audio", SFI_DEV_TYPE_IPC, 1, &clvs_audio_platform_data,
					     &ipc_device_handler},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},

	/* Touch Screen */
	{"mxt224", SFI_DEV_TYPE_I2C, 0, &mxt224_platform_data, NULL},

	{},
};
