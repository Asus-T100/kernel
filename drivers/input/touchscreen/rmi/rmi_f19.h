/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $11 header.
 * Copyright (c) 2007 - 2010, Synaptics Incorporated
 *
 * For every RMI4 function that has a data source - like 2D sensors,
 * buttons, LEDs, GPIOs, etc. - the user will create a new rmi_function_xx.c
 * file and add these functions to perform the config(), init(), report()
 * and detect() functionality. The function pointers are then srored under
 * the RMI function info and these functions will automatically be called by
 * the global config(), init(), report() and detect() functions that will
 * loop through all data sources and call the data sources functions using
 * these functions pointed to by the function ptrs.
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

#if !defined(_RMI_F19_H)
#define _RMI_F19_H

/* This is the data read from the F19 query registers.
 */
struct rmi_F19_query {
	bool has_hysteresis_threshold;
	bool has_sensitivity_adjust;
	bool configurable;
	unsigned char button_count;
};

struct rmi_F19_control {
	unsigned char button_usage;
	unsigned char filter_mode;
	unsigned char *interrupt_enable_registers;
	unsigned char *single_button_control;
	unsigned char *sensor_map;
	unsigned char *single_button_sensitivity;
	unsigned char global_sensitivity_adjustment;
	unsigned char global_hysteresis_threshold;
};


void FN_19_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs);
int FN_19_config(struct rmi_function_info *rmifninfo);
int FN_19_init(struct rmi_function_device *function_device);
int FN_19_detect(struct rmi_function_info *rmifninfo);
/* No attention function for Fn $19 */
#endif
