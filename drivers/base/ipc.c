/**
 * ipc.c: Intel IPC(Inter-Processor Communication) bus interface code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Shijie Zhang (shijie.zhang@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/ipc_device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>

static struct list_head ipc_device_list[IPC_BUS_NR];
static struct mutex ipc_bus_lock[IPC_BUS_NR];
static int bus_init_done[IPC_BUS_NR];

struct device ipc_bus = {
	.init_name = "ipc",
};
EXPORT_SYMBOL_GPL(ipc_bus);

struct ipc_object {
	struct ipc_device ipcdev;
	char name[1];
};

static struct resource *ipc_get_resource(struct ipc_device *ipcdev,
		unsigned int type, unsigned int num)

{
	int i;

	for (i = 0; i < ipcdev->num_resources; i++) {
		struct resource *r = &ipcdev->resource[i];
		if (type == resource_type(r) && num-- == 0)
			return r;
	}

	return NULL;
}

/**
 * ipc_get_irq - get irq resource from ipc device
 * @ipcdev: pointer to the ipc device
 * @num: resource num in ipc device's resource, 0 for ipc device
 */
int ipc_get_irq(struct ipc_device *ipcdev, int num)
{
	struct resource *r = ipc_get_resource(ipcdev, IORESOURCE_IRQ, num);

	return r ? r->start : -ENXIO;
}
EXPORT_SYMBOL_GPL(ipc_get_irq);

/**
 * ipc_get_resource_byname - get ipc resource by name
 * @dev: pointer to the ipc device
 * @type: resource type
 * @name: resource name
 */
struct resource *ipc_get_resource_byname(struct ipc_device *dev,
		unsigned int type, const char *name)
{
	int i;

	for (i = 0; i < dev->num_resources; i++) {
		struct resource *r = &dev->resource[i];

		if (type == resource_type(r) && !strcmp(r->name, name))
			return r;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(ipc_get_resource_byname);

static void ipc_device_release(struct device *dev)
{
	struct ipc_object *ipc = container_of(dev, struct ipc_object,
			ipcdev.dev);
	kfree(ipc->ipcdev.dev.platform_data);
	kfree(ipc->ipcdev.resource);
	kfree(ipc);
}

/**
 * ipc_device_alloc - allocate and initialize a ipc device
 * @name: name for ipc device
 * @id: instance number, -1 if only one
 */
struct ipc_device *ipc_device_alloc(const char *name, int id)
{
	struct ipc_object *ipc;

	ipc = kzalloc(sizeof(struct ipc_object) + strlen(name), GFP_KERNEL);
	if (ipc) {
		strcpy(ipc->name, name);
		ipc->ipcdev.name = ipc->name;
		ipc->ipcdev.id = id;
		device_initialize(&ipc->ipcdev.dev);
		ipc->ipcdev.dev.release = ipc_device_release;
	}

	return ipc ? &ipc->ipcdev : NULL;
}
EXPORT_SYMBOL_GPL(ipc_device_alloc);

/**
 * ipc_device_add_resources - add resources for ipc device
 * @ipcdev: pointer to ipc device
 * @res: pointer to the array of resource
 * @n: number of total resources
 */
int ipc_device_add_resources(struct ipc_device *ipcdev,
		const struct resource *res, unsigned int n)
{
	struct resource *r;

	r = kzalloc(sizeof(struct resource) * n, GFP_KERNEL);
	if (r) {
		memcpy(r, res, sizeof(struct resource) * n);
		ipcdev->resource = r;
		ipcdev->num_resources = n;
	}

	return r ? 0 : -ENOMEM;
}
EXPORT_SYMBOL_GPL(ipc_device_add_resources);

/**
 * ipc_device_add - add ipc device to the device tree
 * @ipcdev: pointer to the ipc device
 */
int ipc_device_add(struct ipc_device *ipcdev)
{
	int i, ret = 0;

	if (!ipcdev)
		return -EINVAL;

	if (!ipcdev->dev.parent)
		ipcdev->dev.parent = &ipc_bus;

	ipcdev->dev.bus = &ipc_bus_type;

	if (ipcdev->id != -1)
		dev_set_name(&ipcdev->dev, "%s.%d", ipcdev->name, ipcdev->id);
	else
		dev_set_name(&ipcdev->dev, "%s", ipcdev->name);

	for (i = 0; i < ipcdev->num_resources; i++) {
		struct resource *p, *r = &ipcdev->resource[i];

		if (r->name == NULL)
			r->name = dev_name(&ipcdev->dev);

		p = r->parent;

		if (!p) {
			if (resource_type(r) == IORESOURCE_MEM)
				p = &iomem_resource;
			else if (resource_type(r) == IORESOURCE_IO)
				p = &ioport_resource;
		}

		if (p && insert_resource(p, r)) {
			printk(KERN_ERR
				"%s: failed to claim resource %d\n",
				dev_name(&ipcdev->dev), i);
			ret = -EBUSY;
			goto failed;
		}
	}

	pr_debug("Registering ipc device '%s'. Parent at %s\n",
			dev_name(&ipcdev->dev), dev_name(ipcdev->dev.parent));

	ret = device_add(&ipcdev->dev);
	if (ret == 0)
		return ret;

failed:
	while (--i > 0) {
		struct resource *r = &ipcdev->resource[i];
		unsigned long type = resource_type(r);
		if (type == IORESOURCE_MEM || type == IORESOURCE_IO)
			release_resource(r);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ipc_device_add);

static ssize_t modalias_show(struct device *dev, struct device_attribute *a,
		char *buf)
{
	struct ipc_device  *ipcdev = to_ipc_device(dev);
	int len = snprintf(buf, PAGE_SIZE, "IPC:%s\n", ipcdev->name);

	return (len >= PAGE_SIZE) ? (PAGE_SIZE - 1) : len;
}

static struct device_attribute ipc_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};

static const struct ipc_device_id *ipc_match_id(const struct ipc_device_id *id,
			struct ipc_device *ipcdev)
{
	while (id->name[0]) {
		if (strcmp(ipcdev->name, id->name) == 0) {
			ipcdev->id_entry = id;
			return id;
		}
		id++;
	}

	return NULL;
}

static int ipc_match(struct device *_dev, struct device_driver *_drv)
{
	struct ipc_device *dev = to_ipc_device(_dev);
	struct ipc_driver *drv = to_ipc_driver(_drv);

	/* match against the id table first if exist */
	if (drv->id_table)
		return ipc_match_id(drv->id_table, dev) != NULL;

	/* fall-back to driver name match */
	return (strcmp(dev->name, _drv->name) == 0);
}

static int ipc_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct ipc_device  *ipcdev = to_ipc_device(dev);

	add_uevent_var(env, "MODALIAS=%s%s", IPC_MODULE_PREFIX,
		(ipcdev->id_entry) ? ipcdev->id_entry->name : ipcdev->name);
	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int ipc_legacy_suspend(struct device *dev, pm_message_t mesg)
{
	struct ipc_driver *ipcdrv = to_ipc_driver(dev->driver);
	struct ipc_device *ipcdev = to_ipc_device(dev);
	int ret = 0;

	if (dev->driver && ipcdrv->suspend)
		ret = ipcdrv->suspend(ipcdev, mesg);

	return ret;
}

static int ipc_legacy_resume(struct device *dev)
{
	struct ipc_driver *ipcdrv = to_ipc_driver(dev->driver);
	struct ipc_device *ipcdev = to_ipc_device(dev);
	int ret = 0;

	if (dev->driver && ipcdrv->resume)
		ret = ipcdrv->resume(ipcdev);

	return ret;
}

int ipc_pm_prepare(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (drv && drv->pm && drv->pm->prepare)
		ret = drv->pm->prepare(dev);

	return ret;
}

void ipc_pm_complete(struct device *dev)
{
	struct device_driver *drv = dev->driver;

	if (drv && drv->pm && drv->pm->complete)
		drv->pm->complete(dev);
}

#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_SUSPEND

int ipc_pm_suspend(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->suspend)
			ret = drv->pm->suspend(dev);
	} else {
		ret = ipc_legacy_suspend(dev, PMSG_SUSPEND);
	}

	return ret;
}

int ipc_pm_suspend_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->suspend_noirq)
			ret = drv->pm->suspend_noirq(dev);
	}

	return ret;
}

int ipc_pm_resume(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->resume)
			ret = drv->pm->resume(dev);
	} else {
		ret = ipc_legacy_resume(dev);
	}

	return ret;
}

int ipc_pm_resume_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->resume_noirq)
			ret = drv->pm->resume_noirq(dev);
	}

	return ret;
}

#endif /* CONFIG_SUSPEND */


#ifdef CONFIG_HIBERNATE_CALLBACKS

int ipc_pm_freeze(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->freeze)
			ret = drv->pm->freeze(dev);
	} else {
		ret = ipc_legacy_suspend(dev, PMSG_FREEZE);
	}

	return ret;
}

int ipc_pm_freeze_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->freeze_noirq)
			ret = drv->pm->freeze_noirq(dev);
	}

	return ret;
}

int ipc_pm_thaw(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->thaw)
			ret = drv->pm->thaw(dev);
	} else {
		ret = ipc_legacy_resume(dev);
	}

	return ret;
}

int ipc_pm_thaw_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->thaw_noirq)
			ret = drv->pm->thaw_noirq(dev);
	}

	return ret;
}

int ipc_pm_poweroff(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->poweroff)
			ret = drv->pm->poweroff(dev);
	} else {
		ret = ipc_legacy_suspend(dev, PMSG_HIBERNATE);
	}

	return ret;
}

int ipc_pm_poweroff_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->poweroff_noirq)
			ret = drv->pm->poweroff_noirq(dev);
	}

	return ret;
}

int ipc_pm_restore(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->restore)
			ret = drv->pm->restore(dev);
	} else {
		ret = ipc_legacy_resume(dev);
	}

	return ret;
}

int ipc_pm_restore_noirq(struct device *dev)
{
	struct device_driver *drv = dev->driver;
	int ret = 0;

	if (!drv)
		return 0;

	if (drv->pm) {
		if (drv->pm->restore_noirq)
			ret = drv->pm->restore_noirq(dev);
	}

	return ret;
}

#endif /* CONFIG_HIBERNATE_CALLBACKS */

static const struct dev_pm_ops ipc_dev_pm_ops = {
	.runtime_suspend = pm_generic_runtime_suspend,
	.runtime_resume = pm_generic_runtime_resume,
	.runtime_idle = pm_generic_runtime_idle,
	USE_IPC_PM_SLEEP_OPS
};

struct bus_type ipc_bus_type = {
	.name = "ipc",
	.dev_attrs = ipc_dev_attrs,
	.match = ipc_match,
	.uevent = ipc_uevent,
	.pm = &ipc_dev_pm_ops,
};
EXPORT_SYMBOL_GPL(ipc_bus_type);

int __init ipc_bus_init(void)
{
	int i, error;

	error = device_register(&ipc_bus);
	if (error)
		return error;

	error =  bus_register(&ipc_bus_type);
	if (error)
		device_unregister(&ipc_bus);

	for (i = 0; i < IPC_BUS_NR; i++) {
		INIT_LIST_HEAD(&ipc_device_list[i]);
		mutex_init(&ipc_bus_lock[i]);
	}

	return error;
}

static int ipc_drv_probe(struct device *_dev)
{
	struct ipc_driver *drv = to_ipc_driver(_dev->driver);
	struct ipc_device *dev = to_ipc_device(_dev);

	return drv->probe(dev);
}

static int ipc_drv_remove(struct device *_dev)
{
	struct ipc_driver *drv = to_ipc_driver(_dev->driver);
	struct ipc_device *dev = to_ipc_device(_dev);

	return drv->remove(dev);
}

static void ipc_drv_shutdown(struct device *_dev)
{
	struct ipc_driver *drv = to_ipc_driver(_dev->driver);
	struct ipc_device *dev = to_ipc_device(_dev);

	drv->shutdown(dev);
}

/**
 * ipc_driver_register - register ipc device driver
 * @drv: pointer to ipc device driver
 */
int ipc_driver_register(struct ipc_driver *drv)
{
	drv->driver.bus = &ipc_bus_type;

	if (drv->probe)
		drv->driver.probe = ipc_drv_probe;

	if (drv->remove)
		drv->driver.remove = ipc_drv_remove;

	if (drv->shutdown)
		drv->driver.shutdown = ipc_drv_shutdown;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(ipc_driver_register);

/**
 * ipc_driver_unregister - unregister ipc device driver
 * drv: pointer to ipc device driver
 */
void ipc_driver_unregister(struct ipc_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(ipc_driver_unregister);

/**
 * ipc_new_device - create the ipc device and add it to the IPC bus DT
 * @info: pointer to ipc board info
 */
int ipc_new_device(struct ipc_board_info *info)
{
	int ret;
	struct ipc_device *ipcdev;

	ipcdev = ipc_device_alloc(info->name, info->id);
	if (!ipcdev)
		return -ENOMEM;

	ipcdev->bus_id = info->bus_id;
	ipcdev->dev.platform_data = info->platform_data;

	if (info->num_res)
		ipc_device_add_resources(ipcdev, info->res, info->num_res);

	mutex_lock(&ipc_bus_lock[ipcdev->bus_id]);

	/**
	 * If the device driver is probed before IPC controller is initilized,
	 * just add it to the ipc_board_list, and process it after controller
	 * is initilized.
	 */
	if (bus_init_done[info->bus_id] == 0) {
		list_add_tail(&ipcdev->entry, &ipc_device_list[info->bus_id]);
		ret = 0;
	} else {
		ret = ipc_device_add(ipcdev);
		if (ret == 0)
			list_add_tail(&ipcdev->entry,
					&ipc_device_list[info->bus_id]);
	}

	mutex_unlock(&ipc_bus_lock[ipcdev->bus_id]);

	return ret;
}
EXPORT_SYMBOL_GPL(ipc_new_device);

void ipc_device_add_to_list(struct ipc_device *ipcdev)
{
	int bus_id = ipcdev->bus_id;

	mutex_lock(&ipc_bus_lock[bus_id]);
	list_add_tail(&ipcdev->entry, &ipc_device_list[bus_id]);
	mutex_unlock(&ipc_bus_lock[bus_id]);
}
EXPORT_SYMBOL_GPL(ipc_device_add_to_list);

/**
 * ipc_create_devices - scan IPC bus and create the IPC devices
 * @bus_id: IPC bus number
 */
void ipc_register_devices(int bus_id)
{
	struct ipc_device *ipcdev;

	mutex_lock(&ipc_bus_lock[bus_id]);

	bus_init_done[bus_id] = 1;

	list_for_each_entry(ipcdev, &ipc_device_list[bus_id], entry)
		if (bus_id == ipcdev->bus_id)
			ipc_device_add(ipcdev);

	mutex_unlock(&ipc_bus_lock[bus_id]);
}
EXPORT_SYMBOL_GPL(ipc_register_devices);

static void ipc_del_device(struct ipc_device *ipcdev)
{
	int i;

	if (ipcdev) {
		device_del(&ipcdev->dev);
		list_del(&ipcdev->entry);

		for (i = 0; i < ipcdev->num_resources; i++) {
			struct resource *r = &ipcdev->resource[i];
			unsigned long type = resource_type(r);

			if (type == IORESOURCE_MEM || type == IORESOURCE_IO)
				release_resource(r);
		}
	}
}

/**
 * ipc_remove_deviecs - remove the devices on the bus
 * @bus_id: IPC bus number
 */
void ipc_remove_devices(int bus_id)
{
	struct ipc_device *ipcdev;

	mutex_lock(&ipc_bus_lock[bus_id]);
	list_for_each_entry(ipcdev, &ipc_device_list[bus_id], entry)
		if (bus_id == ipcdev->bus_id)
			ipc_del_device(ipcdev);

	bus_init_done[bus_id] = 0;
	mutex_unlock(&ipc_bus_lock[bus_id]);
}
EXPORT_SYMBOL_GPL(ipc_remove_devices);

MODULE_AUTHOR("Shijie Zhang <shijie.zhang@intel.com>");
MODULE_DESCRIPTION("Intel IPC bus interface");
MODULE_LICENSE("GPL");
