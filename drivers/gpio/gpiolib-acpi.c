/*
 * ACPI helpers for GPIO API
 *
 * Copyright (C) 2012, Intel Corporation
 * Authors: Mathias Nyman <mathias.nyman@linux.intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/export.h>
#include <linux/acpi_gpio.h>
#include <linux/acpi.h>

static int acpi_gpiochip_find(struct gpio_chip *gc, void *data)
{
	if (!gc->dev)
		return false;

	return ACPI_HANDLE(gc->dev) == data;
}

/**
 * acpi_get_gpio() - Translate ACPI GPIO pin to GPIO number usable with GPIO API
 * @path:	ACPI GPIO controller full path name, (e.g. "\\_SB.GPO1")
 * @pin:	ACPI GPIO pin number (0-based, controller-relative)
 *
 * Returns GPIO number to use with Linux generic GPIO API, or errno error value
 */

int acpi_get_gpio(char *path, int pin)
{
	struct gpio_chip *chip;
	acpi_handle handle;
	acpi_status status;

	status = acpi_get_handle(NULL, path, &handle);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	chip = gpiochip_find(handle, acpi_gpiochip_find);
	if (!chip)
		return -ENODEV;

	if (!gpio_is_valid(chip->base + pin))
		return -EINVAL;

	return chip->base + pin;
}
EXPORT_SYMBOL_GPL(acpi_get_gpio);

struct acpi_gpio_lookup {
	struct acpi_gpio_info info;
	int index;
	int gpio;
	int n;
};

static int acpi_find_gpio(struct acpi_resource *ares, void *data)
{
	struct acpi_gpio_lookup *lookup = data;

	if (ares->type != ACPI_RESOURCE_TYPE_GPIO)
		return 1;

	if (lookup->n++ == lookup->index && lookup->gpio < 0) {
		const struct acpi_resource_gpio *agpio = &ares->data.gpio;

		lookup->gpio = acpi_get_gpio(agpio->resource_source.string_ptr,
			agpio->pin_table[0]);
		lookup->info.gpioint =
			agpio->connection_type == ACPI_RESOURCE_GPIO_TYPE_INT;
	}

	return 1;
}

/**
 * acpi_get_gpio_by_index() - get a GPIO number from device resources
 * @dev: pointer to a device to get GPIO from
 * @index: index of GpioIo/GpioInt resource (starting from %0)
 * @info: info pointer to fill in (optional)
 *
 * Function goes through ACPI resources for @dev and based on @index looks
 * up a GpioIo/GpioInt resource, translates it to the Linux GPIO number,
 * and returns it. @index matches GpioIo/GpioInt resources only so if there
 * are total %3 GPIO resources, the index goes from %0 to %2.
 *
 * If the GPIO cannot be translated or there is an error, negative errno is
 * returned.
 *
 * Note: if the GPIO resource has multiple entries in the pin list, this
 * function only returns the first.
 */
int acpi_get_gpio_by_index(struct device *dev, int index,
		struct acpi_gpio_info *info)
{
	struct acpi_gpio_lookup lookup;
	struct list_head resource_list;
	struct acpi_device *adev;
	acpi_handle handle;
	int ret;

	if (!dev)
		return -EINVAL;

	handle = ACPI_HANDLE(dev);
	if (!handle || acpi_bus_get_device(handle, &adev))
		return -ENODEV;

	memset(&lookup, 0, sizeof(lookup));
	lookup.index = index;
	lookup.gpio = -ENODEV;

	INIT_LIST_HEAD(&resource_list);
	ret = acpi_dev_get_resources(adev, &resource_list, acpi_find_gpio,
					&lookup);
	if (ret < 0)
		return ret;

	acpi_dev_free_resource_list(&resource_list);

	if (lookup.gpio >= 0 && info)
		*info = lookup.info;

	return lookup.gpio;
}
EXPORT_SYMBOL_GPL(acpi_get_gpio_by_index);
