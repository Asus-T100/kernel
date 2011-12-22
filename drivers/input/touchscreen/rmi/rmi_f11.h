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

#if !defined(_RMI_F11_H)
#define _RMI_F11_H

/* This is the data read from the F11 query registers.
 */
struct rmi_F11_device_query {
	bool has_query_9;
	bool has_query_11;
	unsigned char number_of_sensors;
};

struct rmi_F11_sensor_query {
	bool configurable;
	bool has_sensitivity_adjust;
	bool has_gestures;
	bool has_absolute;
	bool has_relative;
	bool has_adj_hyst;
	bool has_dribble;
	bool has_pinch;
	bool has_flick;
	bool has_tap;
	bool has_tap_and_hold;
	bool has_double_tap;
	bool has_early_tap;
	bool has_press;

	bool has_palm_detect;
	bool has_rotate;
	bool has_touch_shapes;
	bool has_scroll_zones;
	bool has_individual_scroll_zones;
	bool has_multifinger_scroll;
	bool has_multifinger_scroll_edge_motion;
	bool has_multifinger_scroll_inertia;

	bool has_pen;
	bool has_proximity;
	bool has_palm_detect_sensitivity;
	bool has_suppress_on_palm_detect;

	bool has_Z_tuning;
	bool has_algorithm_selection;

	unsigned char number_of_touch_shapes;

	unsigned char number_of_fingers;
	unsigned char number_of_X_electrodes;
	unsigned char number_of_Y_electrodes;
	unsigned char maximum_electrodes;
	bool has_anchored_finger;
	unsigned char abs_data_size;
};

struct rmi_F11_control {
	bool relative_ballistics;
	bool relative_position_filter;
	bool absolute_position_filter;
	unsigned char reporting_mode;
	bool manually_tracked_finger;
	bool manually_tracked_finger_enable;
	unsigned char motion_sensitivity;
	unsigned char palm_detect_threshold;
	unsigned char delta_X_pos_threshold;
	unsigned char delta_Y_pos_threshold;
	unsigned char velocity;
	unsigned char acceleration;
	unsigned short sensor_max_X_pos;
	unsigned short sensor_max_Y_pos;
};

void FN_11_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs);
int FN_11_config(struct rmi_function_info *rmifninfo);
int FN_11_init(struct rmi_function_device *function_device);
int FN_11_detect(struct rmi_function_info *rmifninfo);
/* No attention function for Fn $11 */
#endif
