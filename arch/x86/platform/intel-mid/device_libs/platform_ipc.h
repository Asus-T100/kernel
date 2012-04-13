/*
 * platform_ipc.h: IPC platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_IPC_H_
#define _PLATFORM_IPC_H_

enum ipc_dev_type {
	IPC_DEV_PMIC_GPIO,
	IPC_DEV_PMIC_AUDIO,
	IPC_DEV_MSIC_ADC,
	IPC_DEV_MSIC_BATTERY,
	IPC_DEV_MSIC_GPIO,
	IPC_DEV_MSIC_AUDIO,
	IPC_DEV_MSIC_POWER_BTN,
	IPC_DEV_MSIC_OCD,

	IPC_DEV_NUM,
};

struct intel_ipc_dev_res {
	const char *name;
	int num_resources;
	struct resource *resources;
};

extern void ipc_device_handler(struct sfi_device_table_entry *pentry,
			struct devs_id *dev) __attribute__((weak));

extern void handle_ipc_irq_res(int irq,
		struct intel_ipc_dev_res *ipc_res) __attribute__((weak));
#endif
