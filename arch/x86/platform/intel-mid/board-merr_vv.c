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
#include <asm/platform_mrfld_audio.h>
/* I2C Devices */
#include "device_libs/platform_max17042.h"

static void __init *no_platform_data(void *info)
{
	return NULL;
}

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
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{},
};

/* not supported */
int penwell_otg_query_charging_cap(void *dummy)
{
	return -1;
}
