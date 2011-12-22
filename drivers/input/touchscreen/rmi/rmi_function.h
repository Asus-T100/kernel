/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function Device Header File.
 * Copyright (c) 2007 - 2011, Synaptics Incorporated
 *
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

#if !defined(_RMI_FUNCTION_H)
#define _RMI_FUNCTION_H

#include <linux/input.h>
#include <linux/device.h>
/* For each function present on the RMI device, we need to get the RMI4 Function
 * Descriptor info from the Page Descriptor Table. This will give us the
 * addresses for Query, Command, Control, Data and the Source Count (number
 * of sources for this function) and the function id.
 * This version contains the full addresses, needed in the case of multiple
 * pages with PDT's on them.
 */
struct rmi_function_descriptor_short {
	unsigned short query_base_addr;
	unsigned short command_base_addr;
	unsigned short control_base_addr;
	unsigned short data_base_addr;
	unsigned char interrupt_source_count;
	unsigned char function_number;
};
/* For each function present on the RMI device, there will be a corresponding
 * entry in the functions list of the rmi_sensor_driver structure.  This entry
 * gives information about the number of data sources and the number of data
 * registers associated with the function.
 */
struct rmi_function_info {
	/* The sensor this function belongs to.
	 */
	struct rmi_sensor_driver *sensor;

	/* A device associated with this function.
	 */
	struct rmi_function_device *function_device;

	unsigned char function_number;

	/* This is the number of data sources associated with the function. */
	unsigned char num_data_sources;

	/* This is the interrupt register and mask - needed for enabling the
	 *  interrupts and for checking what source had caused the attention
	 * line interrupt.
	 */
	unsigned char interrupt_register;
	unsigned char interrupt_mask;

	/* This is the RMI function descriptor associated with this function.
	 *  It contains the Base addresses for the functions query, command,
	 *  control, and data registers. It includes the page in the addresses.
	 */
	struct rmi_function_descriptor_short function_descriptor;

	/* pointer to data specific to a functions implementation. */
	void *fndata;

	/* A list of the function information.
	 *  This list uses the standard kernel linked list implementation.
	 *  Documentation on on how to use it can be found at
	 *  http://isis.poly.edu/kulesh/stuff/src/klist/.
	 */
	struct list_head link;
};

/* This struct is for creating a list of RMI4 functions that have data sources
associated with them. This is to facilitate adding new support for other
data sources besides 2D sensors.
To add a new data source support, the developer will create a new file
and add these 4 functions below with FN$## in front of the names - where
## is the hex number for the function taken from the RMI4 specification.

The function number will be associated with this and later will be used to
match the RMI4 function to the 4 functions for that RMI4 function number.
The user will also have to add code that adds the new rmi_functions item
to the global list of RMI4 functions and stores the pointers to the 4
functions in the function pointers.
 */
struct rmi_function_ops {
	unsigned char function_number;

	/* Pointers to function specific functions for interruptHandler,
	 * config, init, detect and attention. These ptrs. need to be filled
	 * in for every RMI4 function that has data source(s) associated with
	 * it - like fn $11 (2D sensors), fn $19 (buttons), etc. Each RMI4
	 * function that has data sources will be added into a list that is
	 * used to match the function number against the number stored here.
	 *
	 * The sensor implementation will call this whenever and IRQ is
	 * dispatched that this function is interested in.
	 */
	void (*inthandler) (struct rmi_function_info *rfi,
			    unsigned int asserted_IRQs);

	int (*config) (struct rmi_function_info *rmifninfo);
	int (*init) (struct rmi_function_device *function_device);
	int (*detect) (struct rmi_function_info *rmifninfo);
	/* If this is non-null, the sensor implementation will call this
	 * whenever the ATTN line is asserted.
	 */
	void (*attention) (struct rmi_function_info *rmifninfo);
	/**
	 *  suspend/resume provided from each function
	 */
	int (*suspend) (struct rmi_function_info *rmifninfo);
	void (*resume) (struct rmi_function_info *rmifninfo);
	/**
	 *  suspendable
	 *  return zero if the function cannot be suspended at the moment
	 *  nonzero if the function can be suspended
	 */
	int (*suspendable)(struct rmi_function_info *rmifninfo);
};

struct rmi_function_ops *rmi_find_function(int function_number);
int rmi_functions_init(struct input_dev *inputdev);

struct rmi_function_driver {
	struct module *module;
	struct device_driver drv;

	/* Probe Function
	 *  This function is called to give the function driver layer an
	 *  opportunity to claim an RMI function.
	 */
	int (*probe) (struct rmi_function_driver *function);
	/* Config Function
	 *  This function is called after a successful probe.  It gives the
	 *  function driver an opportunity to query and/or configure an RMI
	 *  function before data starts flowing.
	 */
	void (*config) (struct rmi_function_driver *function);

	unsigned short query_base_address;
	unsigned short control_base_address;
	unsigned short command_base_address;
	unsigned short data_base_address;
	/* offset from start of interrupt registers */
	unsigned int interrupt_register_offset;
	unsigned int interrupt_mask;

	/* Pointer to the corresponding phys driver info for this sensor.
	 * The phys driver has the pointers to read, write, etc.  Probably
	 * don't need it here - used down in bus driver and sensor driver. */
	struct rmi_phys_driver *rpd;

	struct list_head function_drivers;
};

struct rmi_function_device {
	/*TODO: function driver should be removed if/when we don't need it.*/
	/*struct rmi_function_driver *function;*/
	struct device dev;
	struct input_dev *input;
	/* need this to be bound to phys driver layer */
	struct rmi_sensor_driver *sensor;

	/* The function ptrs to the config, init, detect and
	 * report functions for this rmi function device. */
	struct rmi_function_ops *rmi_funcs;
	struct rmi_function_info *rfi;
	struct list_head functions;	/* link functions into list */
};

int rmi_function_register_device(struct rmi_function_device *dev,
				 int function_number);
#endif
