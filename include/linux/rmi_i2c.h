/**
 *
 * Synaptics RMI over I2C Physical Layer Driver Header File.
 * Copyright (c) 2007 - 2011, Synaptics Incorporated
 *
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

#if !defined(_RMI_I2C_H)
#define _RMI_I2C_H

#include <linux/rmi_platformdata.h>

/* In the future, we may change the device name.  If so, defining it here
 * makes life easier.
 */
#define RMI4_I2C_DRIVER_NAME "rmi4_ts"
#define RMI4_I2C_DEVICE_NAME "rmi4_ts"

/* Sensor-specific configuration data, to be included as the platform data
 * for the relevant i2c_board_info entry.
 *
 * This describes a single RMI4 sensor on an I2C bus, including:
 * its I2C address, IRQ (if any), the type of IRQ (if applicable), and an
 * optional list of any non-default settings (on a per function basis)
 * to be applied at start up.
 */
struct rmi_i2c_platformdata {
	/* The seven-bit i2c address of the sensor. */
	int i2c_address;

	/* If >0, the driver will delay this many milliseconds before attempting
	 * I2C communications.  This is necessary because some horribly broken
	 * development systems don't bring their I2C up very fast after system
	 * power on or reboot.  In most cases, you can safely ignore this.
	 */
	int delay_ms;

	/* Use this to specify platformdata that is not I2C specific. */
	struct rmi_sensordata *sensordata;
};

#endif
