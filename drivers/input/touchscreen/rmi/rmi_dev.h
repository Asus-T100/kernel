/*
 *
 * Synaptics Register Mapped Interface (RMI4) - RMI device Module Header.
 * Copyright (C) 2007 - 2011, Synaptics Incorporated
 *
 */
/*
 *
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
#if !defined(_RMI_DEV_H)
#define _RMI_DEV_H

#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/wait.h>


struct rmi_sensor_device;

/*
 *  rmi_char_dev: a char device to directly control RMI interface
 */

#define RMI_CHAR_DEV_TMPBUF_SZ 128
#define RMI_REG_ADDR_PAGE_SELECT 0xFF

struct rmi_char_dev {
	/* mutex for file operation*/
	struct mutex mutex_file_op;
    /* main char dev structure */
	struct cdev main_dev;

	/* register address for RMI protocol */
	/* filp->f_pos */

	/* pointer to the corresponding phys driver info for this sensor */
	/* The senor driver has the pointers to read, write, etc. */
	struct rmi_sensor_driver *sensor;
	/* reference count */
	int ref_count;
};

int rmi_char_dev_register(struct rmi_phys_driver *physical_driver,
		struct rmi_sensor_device *sensor_dev);
void rmi_char_dev_unregister(struct rmi_char_dev *char_dev,
		struct class *char_device_class);


#endif /*_RMI_DEV_H*/
