/**
 *
 * Synaptics Register Mapped Interface (RMI4) RMI Driver Header File.
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

#if !defined(_RMI_DRVR_H)
#define _RMI_DRVR_H

#include <linux/rmi_platformdata.h>
#include "rmi.h"

/*  RMI4 Protocol Support
 */

struct rmi_phys_driver {
	char *name;
	int (*write) (struct rmi_phys_driver *physdrvr, unsigned short address,
		      char data);
	int (*read) (struct rmi_phys_driver *physdrvr, unsigned short address,
		     char *buffer);
	int (*write_multiple) (struct rmi_phys_driver *physdrvr,
			       unsigned short address, char *buffer,
			       int length);
	int (*read_multiple) (struct rmi_phys_driver *physdrvr,
			      unsigned short address, char *buffer, int length);
	void (*attention) (struct rmi_phys_driver *physdrvr);
	bool polling_required;
	int irq;

	void (*set_attn_handler) (
		struct rmi_phys_driver *physdrvr,
		void (*attention) (struct rmi_phys_driver *physdrvr));
	int (*enable_device) (struct rmi_phys_driver *physdrvr);
	void (*disable_device) (struct rmi_phys_driver *physdrvr);

	/* Some quick data relating to the operation of the device. */
	char *proto_name;
	unsigned long tx_count;
	unsigned long tx_bytes;
	unsigned long tx_errors;
	unsigned long rx_count;
	unsigned long rx_bytes;
	unsigned long rx_errors;
	unsigned long attn_count;

	struct list_head drivers;
	struct rmi_sensor_driver *sensor;
	struct module *module;
};

int rmi_read(struct rmi_sensor_driver *sensor, unsigned short address,
	     char *dest);
int rmi_write(struct rmi_sensor_driver *sensor, unsigned short address,
	      unsigned char data);
int rmi_read_multiple(struct rmi_sensor_driver *sensor, unsigned short address,
		      char *dest, int length);
int rmi_write_multiple(struct rmi_sensor_driver *sensor, unsigned short address,
		       unsigned char *data, int length);
int rmi_register_sensor(struct rmi_phys_driver *physdrvr,
			struct rmi_sensordata *sensordata);
int rmi_unregister_sensors(struct rmi_phys_driver *physdrvr);

/* Utility routine to set bits in a register. */
int rmi_set_bits(struct rmi_sensor_driver *sensor, unsigned short address,
		 unsigned char bits);
/* Utility routine to clear bits in a register. */
int rmi_clear_bits(struct rmi_sensor_driver *sensor, unsigned short address,
		   unsigned char bits);
/* Utility routine to set the value of a bit field in a register. */
int rmi_set_bit_field(struct rmi_sensor_driver *sensor, unsigned short address,
		      unsigned char field_mask, unsigned char bits);

/* Utility routine to handle writes to read-only attributes.  Hopefully
 * this will never happen, but if the user does something stupid, we
 * don't want to accept it quietly.
 */
ssize_t rmi_store_error(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count);

/* Utility routine to handle reads to write-only attributes.  Hopefully
 * this will never happen, but if the user does something stupid, we
 * don't want to accept it quietly.
 */
ssize_t rmi_show_error(struct device *dev,
		       struct device_attribute *attr,
		       char *buf);

#endif
