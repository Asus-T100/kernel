/*
 * platform_s3202.h: s3202 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_S3202_H_
#define _PLATFORM_S3202_H_

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
#define RMI_F11_INDEX 0x11
#define RMI_F19_INDEX 0x19
#endif

extern void *s3202_platform_data(void *info) __attribute__((weak));
#endif
