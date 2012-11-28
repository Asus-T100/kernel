/*
 * This file is part of the APDS9300 sensor driver.
 * Chip is combined proximity and ambient light sensor.
 *
 * Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
 *
 * Contact: Samu Onkalo <samu.p.onkalo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __APDS9300_H__
#define __APDS9300_H__


#define APDS_IRLED_CURR_12mA	0x3
#define APDS_IRLED_CURR_25mA	0x2
#define APDS_IRLED_CURR_50mA	0x1
#define APDS_IRLED_CURR_100mA	0x0

/**
 * struct apds9300_platform_data - platform data for apsd9300.c driver
 * @cf: chip factor data
 * @pddrive: IR-led driving current
 * @ppcount: number of IR pulses used for proximity estimation
 * @setup_resources: interrupt line setup call back function
 * @release_resources: interrupt line release call back function
 *
 * Proximity detection result depends heavily on correct ppcount, pdrive
 * and cover window.
 *
 */

struct apds9300_platform_data {
	int    gpio_number;
	int    (*setup_resources)(void);
	int    (*release_resources)(void);
};

#endif
