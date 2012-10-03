/*
 * board-sle.c: Intel Merrifield based board (Emulation Platform)
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Mark F. Brown <mark.f.brown@intel.com>,
 *         Bryan C. Morgan <bryan.c.morgan@intel.com>
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

/* I2C Devices */
#include "device_libs/platform_max17042.h"

static void __init *no_platform_data(void *info)
{
	return NULL;
}

struct devs_id __initconst device_ids[] = {
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &mrfl_adc_platform_data, NULL},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"bcove_chrgr", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_vp_platform_data, NULL},
	{},
};

/* not supported */
int penwell_otg_query_charging_cap(void *dummy)
{
	return -1;
}
