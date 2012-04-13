/*
 * platform_ipc.c: IPC platform library file
 *
 * (C) Copyright 2008 Intel Corporation
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
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/ipc_device.h>
#include <asm/intel-mid.h>
#include "platform_ipc.h"

static struct intel_ipc_dev_res *ipc_dev_res;

static DEFINE_MUTEX(ipc_dev_lock);

/* this should be called with the holding of ipc_dev_lock */
void handle_ipc_irq_res(int irq, struct intel_ipc_dev_res *ipc_res)
{
	struct resource *res = ipc_res->resources;

	if (res->flags & IORESOURCE_IRQ)
		res->start = irq;

	ipc_dev_res = ipc_res;
}


void ipc_device_handler(struct sfi_device_table_entry *pentry,
				struct devs_id *dev)
{
	int res_num;
	struct resource *res;
	struct ipc_device *ipcdev;
	void *pdata = NULL;

	pr_info("IPC bus = %d, name = %16.16s, "
		"irq = 0x%2x\n", pentry->host_num, pentry->name, pentry->irq);

	mutex_lock(&ipc_dev_lock);

	pdata = dev->get_platform_data(pentry);

	ipcdev = ipc_device_alloc(pentry->name, -1);
	if (ipcdev == NULL) {
		pr_err("out of memory for SFI platform device '%s'.\n",
				pentry->name);
		return;
	}

	res = ipc_dev_res->resources;
	res_num = ipc_dev_res->num_resources;
	ipc_device_add_resources(ipcdev, res, res_num);

	ipcdev->dev.platform_data = pdata;
	ipc_device_add_to_list(ipcdev);

	mutex_unlock(&ipc_dev_lock);
}

