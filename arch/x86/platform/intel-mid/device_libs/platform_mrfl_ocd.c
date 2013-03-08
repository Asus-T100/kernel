/*
 * platform_mrfl_ocd.c: Platform data for Merrifield Platform OCD  Driver
 *
 * (C) Copyright 2013 Intel Corporation
 *  Author: Saranya Gopal <saranya.gopal@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_basincove_ocd.h>

#include "platform_msic.h"
#include "platform_mrfl_ocd.h"

static int get_bcu_config(struct ocd_bcove_config_data *ocd_smip_data)
{

	/* intel_scu_ipc_read_mip function reads MIP data in terms of bytes */
	return (intel_scu_ipc_read_mip(ocd_smip_data, sizeof(*ocd_smip_data),
							BCU_SMIP_BASE, 1));
}

static struct ocd_platform_data ocd_data;

void __init *mrfl_ocd_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	struct platform_device *pdev;

	pdev = platform_device_alloc(MRFL_OCD_DEV_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
			MRFL_OCD_DEV_NAME);
		return NULL;
	}

	if (platform_device_add(pdev)) {
		pr_err("failed to add merrifield ocd platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	install_irq_resource(pdev, entry->irq);
	ocd_data.bcu_config_data = &get_bcu_config;
	pdev->dev.platform_data	= &ocd_data;
	register_rpmsg_service("rpmsg_mrfl_ocd", RPROC_SCU, RP_MRFL_OCD);

	return &ocd_data;
}
