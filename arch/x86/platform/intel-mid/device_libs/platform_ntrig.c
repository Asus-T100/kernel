/*
 * platform_ntrig.c: ntrig touchscreen platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/spi/spi.h>
#include <linux/spi/ntrig_spi.h>
#include <asm/intel-mid.h>
#include "platform_ntrig.h"

void __init *ntrig_platform_data(void *info)
{
	static struct ntrig_spi_platform_data pdata;
	struct spi_board_info *spi_info = (struct spi_board_info *)info;

	spi_info->irq = get_gpio_by_name(NTRIG_SFI_GPIO_IRQ_NAME);
	if (spi_info->irq == -1) {
		pr_err("%s: Unable to find touch_int GPIO in the SFI table\n",
				__func__);
		return NULL;
	}

	pdata.oe_gpio = get_gpio_by_name(NTRIG_SFI_GPIO_ENABLE_NAME);
	if (pdata.oe_gpio == -1) {
		pr_err("%s: Unable to find touch_spi_en GPIO in the SFI table\n",
				__func__);
		return NULL;
	}
	pdata.oe_inverted = 0;
	pdata.pwr_gpio = NTRIG_SPI_PWR;
	pdata.irq_flags = 0; /* use default flags in the driver */

	return &pdata;
}
