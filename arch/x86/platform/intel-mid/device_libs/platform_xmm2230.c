/*
 * platform_xmm2330.c: xmm2230 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
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
#include <linux/spi/ifx_modem.h>
#include <asm/intel-mid.h>
#include "platform_xmm2230.h"

#define XMM2330_SPI_SPEED_HZ 12500000

void __init *xmm2230_platform_data(void *info)
{
	static struct ifx_modem_platform_data xmm2230_pdata;

	xmm2230_pdata.srdy = get_gpio_by_name("xmm2230_srdy");
	xmm2230_pdata.mrdy = get_gpio_by_name("xmm2230_mrdy");
	xmm2230_pdata.max_hz = XMM2330_SPI_SPEED_HZ;
	xmm2230_pdata.use_dma = true;

	return &xmm2230_pdata;
}
