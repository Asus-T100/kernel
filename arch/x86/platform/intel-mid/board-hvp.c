/*
 * board-hvp.c: Intel Merrifield based board (Hybrid Virtual Platform)
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Mark F. Brown <mark.f.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sfi.h>
#include <asm/intel-mid.h>
#include "device_libs/platform_mrfld_audio.h"
#include "device_libs/platform_max3111.h"

/* not supported */
int penwell_otg_query_charging_cap(void *dummy)
{
	return -1;
}

struct devs_id __initconst device_ids[] = {
	{"mrfld_cs42l73", SFI_DEV_TYPE_IPC, 1, &mrfld_audio_platform_data},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_vp_platform_data, NULL},
	{},
};
