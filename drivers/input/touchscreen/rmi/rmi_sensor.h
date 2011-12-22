/**
 *
 * Synaptics Register Mapped Interface (RMI4) - RMI Sensor Module Header.
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

#if !defined(_RMI_SENSOR_H)
#define _RMI_SENSOR_H


#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/rmi_platformdata.h>
#include "rmi_dev.h"

struct rmi_sensor_driver {
	struct module *module;
	struct device_driver drv;
	struct rmi_sensor_device *sensor_device;

	/* Attention Function
	 *  This function is called by the low level isr in the physical
	 * driver. It merely schedules work to be done.
	 */
	void (*attention) (struct rmi_phys_driver *physdrvr);
	/* Probe Function
	 *  This function is called to give the sensor driver layer an
	 *  opportunity to claim an RMI device.  The sensor layer cannot
	 *  read RMI registers at this point since the rmi physical driver
	 *  has not been bound to it yet.  Defer that to the config
	 *  function call which occurs immediately after a successful probe.
	 */
	int (*probe) (struct rmi_sensor_driver *sensor);
	/* Config Function
	 *  This function is called after a successful probe.  It gives the
	 *  sensor driver an opportunity to query and/or configure an RMI
	 *  device before data starts flowing.
	 */
	void (*config) (struct rmi_sensor_driver *sensor);

	/* Functions can call this in order to dispatch IRQs. */
	void (*dispatchIRQs) (struct rmi_sensor_driver *sensor,
			      unsigned int irq_status);

	unsigned int interrupt_register_count;

	bool polling_required;

	/* pointer to the corresponding phys driver info for this sensor */
	/* The phys driver has the pointers to read, write, etc. */
	struct rmi_phys_driver *rpd;

	struct hrtimer timer;
	struct work_struct work;
	struct mutex work_lock;

	struct list_head functions;	/* List of rmi_function_infos */
		/* Per function initialization data. */
	struct rmi_functiondata_list *perfunctiondata;
		/* non-default operation for suspend/resume */
	struct rmi_sensor_suspend_custom_ops *custom_suspend_ops;
};

/* macro to get the pointer to the device_driver struct from the sensor */
#define to_rmi_sensor_driver(drv) \
	container_of(drv, struct rmi_sensor_driver, drv);

struct rmi_sensor_device {
	struct rmi_sensor_driver *driver;
	struct device dev;

	/* mutex for setting device_is_supended flag*/
	struct mutex setup_suspend_flag;
	int device_is_suspended;	/*it will be initialized to false(0) */
#ifdef CONFIG_HAS_EARLYSUSPEND
	/* handler to handle early_suspend and late_resume */
	struct early_suspend early_suspend_handler;
#endif
	/* pointer to data specific to a sensor implementation. */
	void *sensordata;

	struct list_head sensors;	/* link sensors into list */
#ifdef CONFIG_SYNA_RMI_DEV
	/* pointer to attention char device and char device */
	struct rmi_char_dev *char_dev;
	struct class *rmi_char_device_class;
#endif /*CONFIG_SYNA_RMI_DEV*/
};

int rmi_sensor_register_device(struct rmi_sensor_device *dev, int index);
int rmi_sensor_register_driver(struct rmi_sensor_driver *driver);
bool rmi_polling_required(struct rmi_sensor_driver *sensor);
int rmi_next_sensor_id(void);

void *rmi_sensor_get_functiondata(struct rmi_sensor_driver *driver,
				  unsigned char function_index);


/* Call this to instantiate a new sensor driver.
 */
struct rmi_sensor_driver *rmi_sensor_create_driver(
			struct rmi_sensor_device *sensor_device,
			struct rmi_phys_driver *physical_driver,
			struct rmi_sensordata *sensor_data);

/* Call this when you're done with the sensor driver.  This will clean up any
 * pending actions, cancel any running threads or works, and release all
 * storage.
 */
void rmi_sensor_destroy_driver(struct rmi_sensor_driver *driver);
#endif
