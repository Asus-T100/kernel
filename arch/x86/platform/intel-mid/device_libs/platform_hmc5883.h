/*
 * platform_hmc5883.h: hmc5883 platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_HMC5883_H_
#define _PLATFORM_HMC5883_H_

extern void *hmc5883_platform_data(void *info) __attribute__((weak));
#endif
