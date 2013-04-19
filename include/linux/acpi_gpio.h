#ifndef _LINUX_ACPI_GPIO_H_
#define _LINUX_ACPI_GPIO_H_

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>

/**
 * struct acpi_gpio_info - ACPI GPIO specific information
 * @gpioint: if %true this GPIO is of type GpioInt otherwise type is GpioIo
 */
struct acpi_gpio_info {
	bool gpioint;
};

#ifdef CONFIG_GPIO_ACPI

int acpi_get_gpio(char *path, int pin);
int acpi_get_gpio_by_index(struct device *dev, int index,
			struct acpi_gpio_info *info);

#else /* CONFIG_GPIO_ACPI */

static inline int acpi_get_gpio_by_index(struct device *dev, int index,
					struct acpi_gpio_info *info)
{
	return -ENODEV;
}

static inline int acpi_get_gpio(char *path, int pin)
{
	return -ENODEV;
}

#endif /* CONFIG_GPIO_ACPI */

#endif /* _LINUX_ACPI_GPIO_H_ */
