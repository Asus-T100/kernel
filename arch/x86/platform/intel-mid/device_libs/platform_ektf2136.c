/*
 * platform_ektf2136.c: ektf2136 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_ektf2136.h"


void *ektf2136_platform_data(void *info)
{
	static int dummy;
	struct spi_board_info *spi_info = info;
	int intr = get_gpio_by_name("ts_int");

	if (intr == -1)
		return NULL;
	spi_info->irq = intr + INTEL_MID_IRQ_OFFSET;

	/* we return a dummy pdata */
	return &dummy;
}

