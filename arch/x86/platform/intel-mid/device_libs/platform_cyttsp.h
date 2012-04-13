/*
 * platform_cyttsp.h: cyttsp platform data header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CYTTSP_H_
#define _PLATFORM_CYTTSP_H_

#define CYTTSP_GPIO_PIN 0x3E
extern void *cyttsp_platform_data(void *info) __attribute__((weak));
#endif
