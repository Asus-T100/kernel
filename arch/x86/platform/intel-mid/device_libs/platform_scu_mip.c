/*
 * platform_scu_mip.c: SCU MIP platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Saranya Gopal <saranya.gopal@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <asm/intel_mip.h>

static struct smip_platform_prop mrfl_platform_prop[SMIP_NUM_CONFIG_PROPS] = {
	[USB_COMPLIANCE] = {
		.offset = 0x717,
		.len = 0x1,
		.is_bit_field = 1,
		.mask = 1 >> 6,
	},
	[CHARGE_TERMINATION] = {
		.offset = 0x3B3,
		.len = 0x1,
		.is_bit_field = 0,
		.mask = 0xff,
	},
	[SHUTDOWN_METHODOLOGY] = {
		.offset = 0x3B4,
		.len = 0x1,
		.is_bit_field = 0,
		.mask = 0xff,
	},
	[MOS_TRANS_CAPACITY] = {
		.offset = 0x3B5,
		.len = 0x1,
		.is_bit_field = 0,
		.mask = 0xff,
	},
	[NFC_RESV_CAPACITY] = {
		.offset = 0x3B6,
		.len = 0x1,
		.is_bit_field = 0,
		.mask = 0xff,
	},
	[TEMP_CRIT_SHUTDOWN] = {
		.offset = 0x3B7,
		.len = 0x1,
		.is_bit_field = 0,
		.mask = 0xff,
	},
};

static struct smip_platform_prop ctp_platform_prop[SMIP_NUM_CONFIG_PROPS] = {
	[USB_COMPLIANCE] = {
		.offset = 0x2E7,
		.len = 0x1,
		.is_bit_field = 1,
		.mask = 1 >> 6,
	},
};

static struct scu_mip_platform_data pdata;

void __init *scu_mip_platform_data(void)
{
	int i;
	struct platform_device *pdev;

	pdev = platform_device_alloc(SCU_MIP_DEV_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
					SCU_MIP_DEV_NAME);
		return NULL;
	}

	if (platform_device_add(pdev)) {
		pr_err("failed to add SCU MIP platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	if (INTEL_MID_BOARD(1, PHONE, CLVTP) ||
			(INTEL_MID_BOARD(1, TABLET, CLVT))) {
		for (i = 0; i < ARRAY_SIZE(ctp_platform_prop); i++)
			pdata.smip_prop[i] = ctp_platform_prop[i];
	} else if (INTEL_MID_BOARD(1, PHONE, MRFL) ||
			(INTEL_MID_BOARD(1, TABLET, MRFL))) {
		for (i = 0; i < ARRAY_SIZE(mrfl_platform_prop); i++)
			pdata.smip_prop[i] = mrfl_platform_prop[i];
	}

	pdev->dev.platform_data =  &pdata;

	return 0;
}
postcore_initcall(scu_mip_platform_data);
