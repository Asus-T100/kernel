/*
 * platform_mrfl_ocd.c: Platform data for Merrifield Platform OCD  Driver
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_basincove_ocd.h>

#include "platform_ipc.h"

static int get_bcu_config(struct ocd_bcove_config_data *ocd_smip_data)
{

	/* intel_scu_ipc_read_mip function reads MIP data in terms of bytes */
	return (intel_scu_ipc_read_mip(ocd_smip_data, sizeof(*ocd_smip_data),
							BCU_SMIP_BASE, 1));
}

void *mrfl_ocd_platform_data(void)
{
	static struct ocd_platform_data ocd_platform_data;

	ocd_platform_data.bcu_config_data = get_bcu_config;

	return &ocd_platform_data;
}
