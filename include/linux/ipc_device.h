/**
 * ipc.c: Intel IPC bus interface definitions
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Shijie Zhang (shijie.zhang@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef _LINUX_IPC_DEVICE_H
#define _LINUX_IPC_DEVICE_H

#include <linux/device.h>
#include <linux/mod_devicetable.h>

enum {
	IPC_SCU = 0,
	IPC_PSH,
	IPC_BUS_NR,
};

struct ipc_board_info {
	char name[16];
	int id;
	int bus_id;
	u32 num_res;
	struct resource *res;
	void *platform_data;
};

struct ipc_device {
	const char *name;
	int id;
	int bus_id;
	struct device dev;
	u32 num_resources;
	struct list_head entry;
	struct resource *resource;
	const struct ipc_device_id *id_entry;
};

extern int ipc_get_irq(struct ipc_device *, int);
extern struct resource *ipc_get_resource_byname(struct ipc_device *,
		unsigned int, const char *);
extern struct device ipc_bus;
extern struct bus_type ipc_bus_type;
extern struct ipc_device *ipc_device_alloc(const char *name, int id);

extern int ipc_device_add_resources(struct ipc_device *,
		const struct resource *, unsigned int);
extern int ipc_device_add_data(struct ipc_device *, const void *, size_t);

struct ipc_driver {
	int (*probe)(struct ipc_device *);
	int (*remove)(struct ipc_device *);
	void (*shutdown)(struct ipc_device *);
	int (*suspend)(struct ipc_device *, pm_message_t state);
	int (*resume)(struct ipc_device *);
	struct device_driver driver;
	const struct ipc_device_id *id_table;
};

#define to_ipc_device(x) container_of((x), struct ipc_device, dev)
#define to_ipc_driver(x) container_of((x), struct ipc_driver, driver)

extern int ipc_driver_register(struct ipc_driver *);
extern void ipc_driver_unregister(struct ipc_driver *);

#define ipc_get_drvdata(_dev) dev_get_drvdata(&(_dev)->dev)
#define ipc_set_drvdata(_dev, data) dev_set_drvdata(&(_dev)->dev, (data))

extern int ipc_new_device(struct ipc_board_info *);
extern void ipc_device_add_to_list(struct ipc_device *);

extern void ipc_register_devices(int bus_id);
extern void ipc_remove_devices(int bus_id);

#ifdef CONFIG_PM_SLEEP
extern int ipc_pm_prepare(struct device *dev);
extern void ipc_pm_complete(struct device *dev);
#else
#define ipc_pm_prepare     NULL
#define ipc_pm_complete    NULL
#endif

#ifdef CONFIG_SUSPEND
extern int ipc_pm_suspend(struct device *dev);
extern int ipc_pm_suspend_noirq(struct device *dev);
extern int ipc_pm_resume(struct device *dev);
extern int ipc_pm_resume_noirq(struct device *dev);
#else
#define ipc_pm_suspend             NULL
#define ipc_pm_resume              NULL
#define ipc_pm_suspend_noirq       NULL
#define ipc_pm_resume_noirq        NULL
#endif

#ifdef CONFIG_HIBERNATE_CALLBACKS
extern int ipc_pm_freeze(struct device *dev);
extern int ipc_pm_freeze_noirq(struct device *dev);
extern int ipc_pm_thaw(struct device *dev);
extern int ipc_pm_thaw_noirq(struct device *dev);
extern int ipc_pm_poweroff(struct device *dev);
extern int ipc_pm_poweroff_noirq(struct device *dev);
extern int ipc_pm_restore(struct device *dev);
extern int ipc_pm_restore_noirq(struct device *dev);
#else
#define ipc_pm_freeze              NULL
#define ipc_pm_thaw                NULL
#define ipc_pm_poweroff            NULL
#define ipc_pm_restore             NULL
#define ipc_pm_freeze_noirq        NULL
#define ipc_pm_thaw_noirq          NULL
#define ipc_pm_poweroff_noirq      NULL
#define ipc_pm_restore_noirq       NULL
#endif

#ifdef CONFIG_PM_SLEEP
#define USE_IPC_PM_SLEEP_OPS \
	.prepare = ipc_pm_prepare, \
	.complete = ipc_pm_complete, \
	.suspend = ipc_pm_suspend, \
	.resume = ipc_pm_resume, \
	.freeze = ipc_pm_freeze, \
	.thaw = ipc_pm_thaw, \
	.poweroff = ipc_pm_poweroff, \
	.restore = ipc_pm_restore, \
	.suspend_noirq = ipc_pm_suspend_noirq, \
	.resume_noirq = ipc_pm_resume_noirq, \
	.freeze_noirq = ipc_pm_freeze_noirq, \
	.thaw_noirq = ipc_pm_thaw_noirq, \
	.poweroff_noirq = ipc_pm_poweroff_noirq, \
	.restore_noirq = ipc_pm_restore_noirq,
#else
#define USE_IPC_PM_SLEEP_OPS
#endif

#endif
