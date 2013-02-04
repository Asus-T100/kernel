/*
 * platform_max3111.c: max3111 platform data initilization file
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
#include <linux/spi/intel_mid_ssp_spi.h>
#include "platform_max3111.h"

void __init *mrfl_platform_data(void *info);

static struct intel_mid_ssp_spi_chip chip = {
		.burst_size = DFLT_FIFO_BURST_SIZE,
		.timeout = DFLT_TIMEOUT_VAL,
		/* UART DMA is not supported in VP */
		.dma_enabled = false,
};

void __init *max3111_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;
	int intr;

	if ((intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_VP) ||
		(intel_mrfl_identify_sim() == INTEL_MRFL_CPU_SIMULATION_SLE) ||
		(INTEL_MID_BOARD(1, PHONE, MRFL)))
		return mrfl_platform_data(info);

	spi_info->mode = SPI_MODE_0;
	intr = get_gpio_by_name("max3111_int");
	if (intr == -1)
		return NULL;
	spi_info->irq = intr + INTEL_MID_IRQ_OFFSET;
	return NULL;
}

void __init *mrfl_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;
	spi_info->mode = SPI_MODE_0;
	spi_info->controller_data = &chip;
	spi_info->bus_num = FORCE_SPI_BUS_NUM;
	spi_info->chip_select = FORCE_CHIP_SELECT;
	return NULL;
}
