/*
 * platform_clv_regulator.h: clv regulator platform data header file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CLV_REGULATOR_H_
#define _PLATFORM_CLV_REGULATOR_H_

extern struct platform_device *get_regulator_dev(void);
extern void set_consumer_supply(struct platform_device *dev);

#endif
