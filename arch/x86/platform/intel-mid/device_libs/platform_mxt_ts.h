/*
 * platform_mxt_ts.h: mxt_ts platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_MXT_TS_H_
#define _PLATFORM_MXT_TS_H_

#define TOUCH_RESET_GPIO 129
#define TOUCH_IRQ_GPIO   62

extern void *mxt_ts_platform_data(void *info) __attribute__((weak));
#endif
