/*
 * board-hvp.c: Intel Merrifield based board (Hybrid Virtual Platform)
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Mark F. Brown <mark.f.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/spi/intel_mid_ssp_spi.h>

/* SSP5 debug port settings */
#define SPI_MODALIAS		"spi_max3111"
#define SPI_MAX_SPEED_HZ	3125000
#define SPI_IRQ			0xFF

static struct intel_mid_ssp_spi_chip chip = {
		.burst_size = DFLT_FIFO_BURST_SIZE,
		.timeout = DFLT_TIMEOUT_VAL,
		.dma_enabled = 0,
};

static void __init ssp_uart_init(void)
{
	struct spi_board_info spi_info;

	memset(&spi_info, 0, sizeof(spi_info));
	strncpy(spi_info.modalias, SPI_MODALIAS, sizeof(spi_info.modalias)-1);
	spi_info.irq = SPI_IRQ;
	spi_info.bus_num = 0;
	spi_info.chip_select = 0;
	spi_info.max_speed_hz = SPI_MAX_SPEED_HZ;
	spi_info.controller_data = &chip;
	pr_info("info: SPI bus = %d, name = %16.16s, "
		"irq = 0x%2x, max_freq = %d, cs = %d\n",
		spi_info.bus_num,
		spi_info.modalias,
		spi_info.irq,
		spi_info.max_speed_hz,
		spi_info.chip_select);

	spi_info.mode = SPI_MODE_0;
	spi_info.platform_data = NULL;
	spi_register_board_info(&spi_info, 1);
}
/* not supported */
int penwell_otg_query_charging_cap(void *dummy)
{
	return -1;
}

static void __init vp_board_init(void)
{
	ssp_uart_init();
}

arch_initcall(vp_board_init);
