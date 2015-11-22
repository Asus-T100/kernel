/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _SYNAPTICS_DSX_H_
#define _SYNAPTICS_DSX_H_

#define PLATFORM_DRIVER_NAME "synaptics_dsx"
#define I2C_DRIVER_NAME "synaptics_dsx_i2c"
#define SPI_DRIVER_NAME "synaptics_dsx_spi"

/*
 * struct synaptics_dsx_button_map - button map
 * @nbuttons: number of buttons
 * @map: pointer to array of button codes
 */
struct synaptics_dsx_button_map {
	unsigned char nbuttons;
	unsigned int *map;
};

/*
 * struct synaptics_dsx_board_data - DSX board data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @swap_axes: swap axes flag
 * @irq_gpio: attention interrupt GPIO
 * @irq_on_state: attention interrupt active state
 * @power_gpio: power switch GPIO
 * @power_on_state: power switch active state
 * @reset_gpio: reset GPIO
 * @reset_on_state: reset active state
 * @max_y_for_2d: maximum y value for 2D area when virtual buttons are present
 * @irq_flags: IRQ flags
 * @device_descriptor_addr: HID device descriptor address
 * @panel_x: x-axis resolution of display panel
 * @panel_y: y-axis resolution of display panel
 * @power_delay_ms: delay time to wait after powering up device
 * @reset_delay_ms: delay time to wait after resetting device
 * @reset_active_ms: reset active time
 * @byte_delay_us: delay time between two bytes of SPI data
 * @block_delay_us: delay time between two SPI transfers
 * @pwr_reg_name: pointer to name of regulator for power control
 * @bus_reg_name: pointer to name of regulator for bus pullup control
 * @cap_button_map: pointer to 0D button map
 * @vir_button_map: pointer to virtual button map
 */
struct synaptics_dsx_board_data {
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	int irq;
	int irq_gpio;
	int irq_on_state;
	int power_gpio;
	int power_on_state;
	int reset_gpio;
	int reset_on_state;
	int max_y_for_2d;
	unsigned long irq_flags;
	unsigned short device_descriptor_addr;
	unsigned int panel_x;
	unsigned int panel_y;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
	unsigned int byte_delay_us;
	unsigned int block_delay_us;
	const char *pwr_reg_name;
	const char *bus_reg_name;
	struct synaptics_dsx_button_map *cap_button_map;
	struct synaptics_dsx_button_map *vir_button_map;
};

#endif
