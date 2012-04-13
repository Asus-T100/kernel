/*
 * platform_pmic_audio.c: PMIC AUDIO platform data initilization file
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
#include <asm/intel-mid.h>
#include "platform_ipc.h"
#include "platform_pmic_audio.h"

static struct resource pmic_audio_resources[] __initdata = {
	{
		.flags = IORESOURCE_IRQ,
	},
};


static struct intel_ipc_dev_res ipc_pmic_audio_res[] __initdata = {
	{
		.name                   = "pmic_audio",
		.num_resources          = ARRAY_SIZE(pmic_audio_resources),
		.resources              = pmic_audio_resources,
	},
};

void __init *pmic_audio_platform_data(void *info)
{
	struct sfi_device_table_entry *entry = info;
	handle_ipc_irq_res(entry->irq, ipc_pmic_audio_res);
	return NULL;
}
