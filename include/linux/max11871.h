/*
 * Copyright (C) 2012 Maxim Integrated Products, Inc.
 *
 * Driver Version: 2
 * Release Date: Mar 6, 2012
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MAX11871_H
#define _MAX11871_H

#define MAX11871_NAME  "max11871"


struct max11871_platform_data {
	u16 version;
	u8 i2c_addr;
	u16 abs_x_min;
	u16 abs_x_max;
	u16 abs_y_min;
	u16 abs_y_max;
	u16 abs_z_min;
	u16 abs_z_max;

	u16 abs_button_area_x_max;
	u16 abs_button_area_x_min;
	u16 abs_button_area_y_max;
	u16 abs_button_area_y_min;
	u16 abs_home_button_x;
	u16 abs_home_button_y;
	u16 abs_menu_button_x;
	u16 abs_menu_button_y;
	u16 abs_back_button_x;
	u16 abs_back_button_y;
	u16 abs_search_button_x;
	u16 abs_search_button_y;
	u16 abs_button_fuzz_x;
	u16 abs_button_fuzz_y;

	int gpio_irq;
	int gpio_rst;
	int irq_flags;
	int (*power)(int on);
	int (*platform_hw_init)(void);
	int (*get_report_ready)(void);
};

#endif /*_MAX11871_H*/
