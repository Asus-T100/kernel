/**
 *
 * Synaptics Register Mapped Interface (RMI4) I2C Physical Layer Driver.
 * Copyright (c) 2007-2010, Synaptics Incorporated
 *
 * Author: Js HA <js.ha@stericsson.com> for ST-Ericsson
 * Author: Naveen Kumar G <naveen.gaddipati@stericsson.com> for ST-Ericsson
 * Copyright 2010 (c) ST-Ericsson AB
 */
/*
 * This file is licensed under the GPL2 license.
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 */

#ifndef _SYNAPTICS_RMI4_H_INCLUDED_
#define _SYNAPTICS_RMI4_H_INCLUDED_

#define TOUCH_TYPE_OGS	0
#define TOUCH_TYPE_GFF	1

struct rmi4_touch_calib {
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	u32 customer_id;
	char *fw_name;
	char *key_dev_name;
};

/**
 * struct synaptics_rmi4_platform_data - contains the rmi4 platform data
 * @int_gpio_number: interrupt gpio number
 * @rst_gpio_umber: reset gpio number
 * @irq_type: irq type
 * @x flip: x flip flag
 * @y flip: y flip flag
 * @swap_axes: x, y swap flag
 * @regulator_en: regulator enable flag
 *
 * This structure gives platform data for rmi4.
 */
struct rmi4_platform_data {
	int int_gpio_number;
	int rst_gpio_number;
	int irq_type;
	bool regulator_en;
	char *regulator_name;
	const struct rmi4_touch_calib *calibs;
};

#endif
