/*
 * intel_mid_gps.h: Intel interface for gps devices
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __INTEL_MID_GPS_H__
#define __INTEL_MID_GPS_H__

#define RESET_ON	1
#define RESET_OFF	0
#define ENABLE_ON	1
#define ENABLE_OFF	0

/**
 * struct intel_mid_gps_platform_data - Intel MID GPS platform data
 * @gpio_reset:		GPS reset GPIO
 * @gpio_enable:	GPS enable GPIO
 * @reset:		GPS reset GPIO current value
 * @enable:		GPS enable GPIO current value
 */

struct intel_mid_gps_platform_data {
	int gpio_reset;
	int gpio_enable;
	unsigned int reset;
	unsigned int enable;
};

#endif
