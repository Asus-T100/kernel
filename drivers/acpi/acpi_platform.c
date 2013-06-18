/*
 * ACPI support for platform bus type.
 *
 * Copyright (C) 2012, Intel Corporation
 * Authors: Mika Westerberg <mika.westerberg@linux.intel.com>
 *          Mathias Nyman <mathias.nyman@linux.intel.com>
 *          Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "internal.h"

ACPI_MODULE_NAME("platform");

static int acpi_dev_gpio(struct acpi_resource *ares, void *data)
{
	int *count = data;

	if (ares->type == ACPI_RESOURCE_TYPE_GPIO)
		return (*count)++;

	return 1;
}

static int acpi_dev_resource_gpio(struct acpi_device *adev)
{
	int count = 0;
	int ret;
	struct list_head resource_list;

	if (!adev->handle)
		return count;

	INIT_LIST_HEAD(&resource_list);
	ret = acpi_dev_get_resources(adev, &resource_list, acpi_dev_gpio,
					&count);
	if (ret < 0)
		goto end;

	acpi_dev_free_resource_list(&resource_list);
end:
	return count;
}

/**
 * acpi_create_platform_device - Create platform device for ACPI device node
 * @adev: ACPI device node to create a platform device for.
 *
 * Check if the given @adev can be represented as a platform device and, if
 * that's the case, create and register a platform device, populate its common
 * resources and returns a pointer to it.  Otherwise, return %NULL.
 *
 * The platform device's name will be taken from the @adev's _HID and _UID.
 */
struct platform_device *acpi_create_platform_device(struct acpi_device *adev)
{
	struct platform_device *pdev = NULL;
	struct acpi_device *acpi_parent;
	struct platform_device_info pdevinfo;
	struct resource_list_entry *rentry;
	struct list_head resource_list;
	struct resource *resources = NULL;
	int count;

	/* If the ACPI node already has a physical device attached, skip it. */
	if (adev->physical_node_count)
		return NULL;

	INIT_LIST_HEAD(&resource_list);
	count = acpi_dev_get_resources(adev, &resource_list, NULL, NULL);
	if (count <= 0) {
		if (!acpi_dev_resource_gpio(adev))
			return NULL;
		else
			goto create_dev;
	}

	resources = kmalloc(count * sizeof(struct resource), GFP_KERNEL);
	if (!resources) {
		dev_err(&adev->dev, "No memory for resources\n");
		acpi_dev_free_resource_list(&resource_list);
		return NULL;
	}
	count = 0;
	list_for_each_entry(rentry, &resource_list, node)
		resources[count++] = rentry->res;

	acpi_dev_free_resource_list(&resource_list);

create_dev:
	memset(&pdevinfo, 0, sizeof(pdevinfo));
	/*
	 * If the ACPI node has a parent and that parent has a physical device
	 * attached to it, that physical device should be the parent of the
	 * platform device we are about to create.
	 */
	pdevinfo.parent = NULL;
	acpi_parent = adev->parent;
	if (acpi_parent) {
		struct acpi_device_physical_node *entry;
		struct list_head *list;

		mutex_lock(&acpi_parent->physical_node_lock);
		list = &acpi_parent->physical_node_list;
		if (!list_empty(list)) {
			entry = list_first_entry(list,
					struct acpi_device_physical_node,
					node);
			pdevinfo.parent = entry->dev;
		}
		mutex_unlock(&acpi_parent->physical_node_lock);
	}
	pdevinfo.name = dev_name(&adev->dev);
	pdevinfo.id = -1;
	pdevinfo.res = resources;
	pdevinfo.num_res = count;
	pdevinfo.acpi_node.handle = adev->handle;
	pdev = platform_device_register_full(&pdevinfo);
	if (IS_ERR(pdev)) {
		dev_err(&adev->dev, "platform device creation failed: %ld\n",
			PTR_ERR(pdev));
		pdev = NULL;
	} else {
		dev_dbg(&adev->dev, "created platform device %s\n",
			dev_name(&pdev->dev));
	}

	kfree(resources);
	return pdev;
}
