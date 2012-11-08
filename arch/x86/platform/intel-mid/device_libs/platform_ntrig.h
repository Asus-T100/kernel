/*
 * platform_ntrig.h: ntrig platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef _PLATFORM_NTRIG_H
#define _PLATFORM_NTRIG_H

#define NTRIG_SPI_PWR               176
#define NTRIG_SFI_GPIO_IRQ_NAME     "touch_int"
#define NTRIG_SFI_GPIO_ENABLE_NAME  "touch_spi_en"

extern void __init *ntrig_platform_data(void *info) __attribute__((weak));
#endif
