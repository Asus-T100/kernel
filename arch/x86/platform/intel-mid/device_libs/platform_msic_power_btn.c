/*
 * platform_msic_power_btn.c: MSIC power btn platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/init.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_powerbtn.h>
#include <asm/intel_scu_ipc.h>
#include "platform_ipc.h"
#include "platform_msic_power_btn.h"

static struct resource msic_power_btn_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};

static struct intel_ipc_dev_res ipc_msic_power_btn_res[] __initdata = {
	{
		.name                   = "msic_power_btn",
		.num_resources          = ARRAY_SIZE(msic_power_btn_resources),
		.resources              = msic_power_btn_resources,
	},

};

static int mrfl_pb_irq_ack(struct intel_msic_power_btn_platform_data *pdata)
{
	intel_scu_ipc_update_register(pdata->pb_irq, 0, MSIC_PWRBTNM);
	intel_scu_ipc_update_register(pdata->pb_irq_mask, 0, MSIC_PWRBTNM);

	return 0;
}

void __init *msic_power_btn_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	static struct intel_msic_power_btn_platform_data msic_power_btn_pdata;

	if (INTEL_MID_BOARD(1, PHONE, MRFL)) {
		msic_power_btn_pdata.pbstat = 0xfffff61a;
		msic_power_btn_pdata.pb_level = (1 << 4);
		msic_power_btn_pdata.irq_lvl1_mask = 0x0c;
		msic_power_btn_pdata.pb_irq = 0x02;
		msic_power_btn_pdata.pb_irq_mask = 0x0d;
		msic_power_btn_pdata.irq_ack = mrfl_pb_irq_ack;
	} else if (INTEL_MID_BOARD(1, PHONE, CLVTP)) {
		msic_power_btn_pdata.pbstat = 0xffffefcb;
		msic_power_btn_pdata.pb_level = (1 << 3);
		msic_power_btn_pdata.irq_lvl1_mask = 0x21;
	} else if (INTEL_MID_BOARD(1, TABLET, CLVT)) {
		msic_power_btn_pdata.pbstat = 0xffff7fcb;
		msic_power_btn_pdata.pb_level = (1 << 3);
		msic_power_btn_pdata.irq_lvl1_mask = 0x21;
	} else {
		msic_power_btn_pdata.pbstat = 0xffff7fd0;
		msic_power_btn_pdata.pb_level = (1 << 3);
		msic_power_btn_pdata.irq_lvl1_mask = 0x21;
	}

	handle_ipc_irq_res(entry->irq, ipc_msic_power_btn_res);

	return &msic_power_btn_pdata;
}

