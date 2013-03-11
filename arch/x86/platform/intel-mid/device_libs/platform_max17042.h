/*
 * platform_max17042.h: max17042 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_MAX17042_H_
#define _PLATFORM_MAX17042_H_

#define	NTC_47K_TGAIN			0xE4E4
#define	NTC_47K_TOFF			0x2F1D
#define	NTC_10K_B3435K_TDK_TGAIN	0xE4E4
#define	NTC_10K_B3435K_TDK_TOFF		0x2218

extern void *max17042_platform_data(void *info) __attribute__((weak));
#endif
