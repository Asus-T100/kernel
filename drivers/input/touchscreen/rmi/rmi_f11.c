/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $11 support for 2D.
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/rmi_platformdata.h>

#include "rmi.h"
#include "rmi_drvr.h"
#include "rmi_bus.h"
#include "rmi_sensor.h"
#include "rmi_function.h"
#include "rmi_f11.h"


/*
 * This will dump a ton of stuff to the console.  Don't turn it on.
 */
#define POS_DEBUG		0

/* By default, we'll support two fingers if we can't figure out how many we
 * really need to handle.
 */
#define DEFAULT_NR_OF_FINGERS 2

/*
 * Bitmasks and offsets for various things.  All of these are fixed in the
 * spec, and most are used exactly once in the code below.
 */
/* Per device queries. */
#define HAS_QUERY_9_MASK	0x08
#define HAS_QUERY_11_MASK	0x10
#define NR_SENSORS_MASK		0x07

#define DEVICE_QUERY_SIZE	1


/* Per sensor queries. */
#define CONFIGURABLE_OFFSET	0
#define CONFIGURABLE_MASK	0x80
#define HAS_SENSITIVITY_ADJUST_OFFSET 0
#define HAS_SENSITIVITY_ADJUST_MASK 0x40
#define HAS_GESTURES_OFFSET	0
#define HAS_GESTURES_MASK	0x20
#define HAS_ABS_OFFSET		0
#define HAS_ABS_MASK		0x10
#define HAS_REL_OFFSET		0
#define HAS_REL_MASK		0x08
#define NR_FINGERS_OFFSET	0
#define NR_FINGERS_MASK		0x07
#define NR_X_ELECTRODES_OFFSET	1
#define NR_Y_ELECTRODES_OFFSET	2
#define MAX_ELECTRODES_OFFSET	3
#define NR_ELECTRODES_MASK	0x7F
#define ABS_DATA_SIZE_OFFSET	4
#define ABS_DATA_SIZE_MASK	0x03
#define HAS_ANCHORED_FINGER_OFFSET 4
#define HAS_ANCHORED_FINGER_MASK 0x04
#define HAS_ADJ_HYST_OFFSET	4
#define HAS_ADJ_HYST_MASK	0x08
#define HAS_DRIBBLE_OFFSET	4
#define HAS_DRIBBLE_MASK	0x10

#define HAS_SINGLE_TAP_MASK	0x01
#define HAS_TAP_AND_HOLD_MASK	0x02
#define HAS_DOUBLE_TAP_MASK	0x04
#define HAS_EARLY_TAP_MASK	0x08
#define HAS_FLICK_MASK		0x10
#define HAS_PRESS_MASK		0x20
#define HAS_PINCH_MASK		0x40
#define ROTATE_FLICK_DATA_BYTES	2

#define HAS_PALM_DETECT_MASK	0x01
#define HAS_ROTATE_MASK		0x02
#define HAS_TOUCH_SHAPES_MASK	0x04
#define TOUCH_SHAPE_BITS_PER_BYTE	8
#define HAS_SCROLL_ZONES_MASK	0x08
#define HAS_INDIVIDUAL_SCROLL_ZONES_MASK	0x10
#define SCROLL_ZONE_DATA_BYTES	2
#define HAS_MULTIFINGER_SCROLL_MASK		0x20
#define HAS_MULTIFINGER_SCROLL_EDGE_MOTION_MASK	0x40
#define HAS_MULTIFINGER_SCROLL_INERTIA_MASK	0x80

#define HAS_PEN_MASK		0x01
#define HAS_PROXIMITY_MASK	0x02
#define HAS_PALM_DETECT_SENSITIVITY_MASK	0x04
#define HAS_SUPPRESS_ON_PALM_DETECT_MASK	0x08

#define NUMBER_OF_TOUCH_SHAPES_MASK	0x1F

#define HAS_Z_TUNING_MASK		0x01
#define HAS_ALGORITHM_SECTION_MASK	0x02

/* Data registers. */
#define X_HIGH_BITS_OFFSET		0
#define Y_HIGH_BITS_OFFSET		1
#define XY_LOW_BITS_OFFSET		2
#define XY_HIGH_BITS_SHIFT		4
#define XY_HIGH_BITS_MASK		0x0FF0
#define XY_LOW_BITS_MASK		0x0F
#define X_LOW_BITS_SHIFT		0
#define Y_LOW_BITS_SHIFT		4
#define W_XY_OFFSET			3
#define W_X_SHIFT			0
#define W_Y_SHIFT			4
#define W_XY_MASK			0x0F
#define Z_OFFSET			4
#define REL_X_OFFSET			0
#define REL_Y_OFFSET			1


/* The per-sensor query registers will never be larger than this. */
#define MAX_PER_SENSOR_QUERY_SIZE 11

/* If we can't figure out how many bytes of abs data there are per finger,
 * we'll use this and hope we get lucky.
 */
#define DEFAULT_ABS_BYTES_PER_FINGER 5

/* How many finger status values are packed into a byte?
 */
#define FINGER_STATES_PER_BYTE	4
#define BITS_PER_FINGER_STATE	2

#define REL_BYTES_PER_FINGER 2

struct f11_instance_data {
	struct rmi_F11_device_query *device_info;
	struct rmi_F11_sensor_query *sensor_info;
	struct rmi_F11_control *control_registers;
	unsigned char finger_data_buffer_size;
	unsigned char abs_data_offset;
	unsigned char abs_data_size;
	unsigned char rel_data_offset;
	unsigned char gesture_data_offset;
	unsigned char *finger_data_buffer;
	/* Last X & Y seen, needed at finger lift.  Was down indicates
	 * at least one finger was here. TODO: Eventually we'll need to
	 * track this info on a per finger basis. */
	bool wasdown;
	unsigned int old_X;
	unsigned int old_Y;
	/* Transformations to be applied to coordinates before reporting. */
	bool flip_X;
	bool flip_Y;
	int offset_X;
	int offset_Y;
	int clip_X_low;
	int clip_X_high;
	int clip_Y_low;
	int clip_Y_high;
	bool swap_axes;
	bool rel_report_enabled;

	unsigned int data8_offset;
	unsigned int data9_offset;
	unsigned int data10_offset;
	unsigned int data11_offset;
	unsigned int data13_offset;
	unsigned int data14_offset;
	unsigned int data16_offset;
};

enum finger_state {
	F11_NO_FINGER = 0,
	F11_PRESENT = 1,
	F11_INACCURATE = 2,
	F11_RESERVED = 3
};

#define FINGER_STATE_MASK 0x03

static ssize_t rmi_fn_11_flip_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_11_flip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t rmi_fn_11_clip_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_11_clip_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t rmi_fn_11_offset_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_11_offset_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count);

static ssize_t rmi_fn_11_swap_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_11_swap_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t rmi_fn_11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static ssize_t rmi_fn_11_maxPos_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static struct device_attribute attrs[] = {
	__ATTR(flip, 0666,
	       rmi_fn_11_flip_show, rmi_fn_11_flip_store),	/* RW attr */
	__ATTR(clip, 0666,
	       rmi_fn_11_clip_show, rmi_fn_11_clip_store),	/* RW attr */
	__ATTR(offset, 0666,
	       rmi_fn_11_offset_show, rmi_fn_11_offset_store),	/* RW attr */
	__ATTR(swap, 0666,
	       rmi_fn_11_swap_show, rmi_fn_11_swap_store),	/* RW attr */
	__ATTR(relreport, 0666,
	       rmi_fn_11_relreport_show, rmi_fn_11_relreport_store),	/* RW */
	__ATTR(maxPos, 0444,
	       rmi_fn_11_maxPos_show, rmi_store_error)	/* R0 attr */
};

static void handle_absolute_reports(struct rmi_function_info *rmifninfo);
static void handle_relative_report(struct rmi_function_info *rmifninfo);

/*
 * Read the device query register and extract interesting data.
 */
static int read_device_query(struct rmi_function_info *rmifninfo)
{
	int retval = 0;
	struct f11_instance_data *instance_data = rmifninfo->fndata;
	unsigned char device_query;

	retval = rmi_read(rmifninfo->sensor,
			rmifninfo->function_descriptor.query_base_addr,
			&device_query);
	if (retval) {
		pr_err("%s: Could not read F11 device query at 0x%04x\n",
		       __func__,
			rmifninfo->function_descriptor.query_base_addr);
		return retval;
	}

	/* Extract device data. */
	instance_data->device_info->has_query_9 =
		(device_query & HAS_QUERY_9_MASK) != 0;
	instance_data->device_info->has_query_11 =
		(device_query & HAS_QUERY_11_MASK) != 0;
	instance_data->device_info->number_of_sensors =
		(device_query & NR_SENSORS_MASK) + 1;
	pr_debug("%s: F11 device - %d sensors.  Query 9? %d.", __func__,
		instance_data->device_info->number_of_sensors,
		instance_data->device_info->has_query_9);
	if (instance_data->device_info->number_of_sensors > 1)
		pr_warning("%s: WARNING device has %d sensors, but RMI4 "
		"driver does not support multiple sensors yet.",
		__func__,
		instance_data->device_info->number_of_sensors);

	return retval;
}

/*
 * Read and parse the per sensor query information from the specified
 * address into the specified sensor_info.
 */
static int read_per_sensor_query(struct rmi_function_info *rmifninfo,
				 struct rmi_F11_sensor_query *sensor_info,
				 unsigned char address)
{
	int retval = 0;
	struct f11_instance_data *instance_data = rmifninfo->fndata;
	unsigned char query_buffer[MAX_PER_SENSOR_QUERY_SIZE];
	unsigned int nr_fingers;
	unsigned int query_offset;

	retval = rmi_read_multiple(rmifninfo->sensor, address,
				   query_buffer, sizeof(query_buffer));
	if (retval) {
		pr_err("%s: Could not read F11 device query at 0x%04x\n",
		       __func__, address);
		return retval;
	}

	/* 2D data sources have only 3 bits for the number of fingers
	 * supported - so the encoding is a bit wierd. */
	sensor_info->number_of_fingers = DEFAULT_NR_OF_FINGERS;
	nr_fingers = query_buffer[NR_FINGERS_OFFSET] & NR_FINGERS_MASK;
	switch (nr_fingers) {
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
		sensor_info->number_of_fingers = nr_fingers + 1;
		break;
	case 5:
		sensor_info->number_of_fingers = 10;
		break;
	default:
		pr_warning("%s: Invalid F11 nr fingers %d. Assuming %d.",
			__func__, nr_fingers, DEFAULT_NR_OF_FINGERS);
	}
	pr_debug("%s: Number of fingers: %d.", __func__,
		sensor_info->number_of_fingers);

	sensor_info->configurable =
		(query_buffer[CONFIGURABLE_OFFSET] & CONFIGURABLE_MASK) != 0;
	sensor_info->has_sensitivity_adjust =
		(query_buffer[HAS_SENSITIVITY_ADJUST_OFFSET] &
		HAS_SENSITIVITY_ADJUST_MASK) != 0;
	sensor_info->has_gestures =
		(query_buffer[HAS_GESTURES_OFFSET] & HAS_GESTURES_MASK) != 0;
	sensor_info->has_absolute =
		(query_buffer[HAS_ABS_OFFSET] & HAS_ABS_MASK) != 0;
	sensor_info->has_relative =
		(query_buffer[HAS_REL_OFFSET] & HAS_REL_MASK) != 0;

	sensor_info->abs_data_size =
		query_buffer[ABS_DATA_SIZE_OFFSET] & ABS_DATA_SIZE_MASK;
	sensor_info->has_anchored_finger =
		query_buffer[HAS_ANCHORED_FINGER_OFFSET] &
		HAS_ANCHORED_FINGER_MASK;
	sensor_info->has_adj_hyst =
		query_buffer[HAS_ADJ_HYST_OFFSET] & HAS_ADJ_HYST_MASK;
	sensor_info->has_dribble =
		query_buffer[HAS_DRIBBLE_OFFSET] & HAS_DRIBBLE_MASK;

	/* From here on, query offsets are determined by the presence or
	 * absence of various features.
	 */
	query_offset = ABS_DATA_SIZE_OFFSET + 1;

	/* This byte is here if we have relative reporting, but it
	 * doesn't contain anything.
	 */
	if (sensor_info->has_relative)
		query_offset++;

	/* The next two bytes are present if we have gestures. */
	if (sensor_info->has_gestures) {
		sensor_info->has_pinch =
			query_buffer[query_offset] & HAS_PINCH_MASK;
		sensor_info->has_flick =
			query_buffer[query_offset] & HAS_FLICK_MASK;
		sensor_info->has_tap =
			query_buffer[query_offset] & HAS_SINGLE_TAP_MASK;
		sensor_info->has_tap_and_hold =
			query_buffer[query_offset] & HAS_TAP_AND_HOLD_MASK;
		sensor_info->has_double_tap =
			query_buffer[query_offset] & HAS_DOUBLE_TAP_MASK;
		sensor_info->has_early_tap =
			query_buffer[query_offset] & HAS_EARLY_TAP_MASK;
		sensor_info->has_press =
			query_buffer[query_offset] & HAS_PRESS_MASK;
		query_offset++;

		sensor_info->has_palm_detect =
			query_buffer[query_offset] & HAS_PALM_DETECT_MASK;
		sensor_info->has_rotate =
			query_buffer[query_offset] & HAS_ROTATE_MASK;
		sensor_info->has_touch_shapes =
			query_buffer[query_offset] & HAS_TOUCH_SHAPES_MASK;
		sensor_info->has_scroll_zones =
			query_buffer[query_offset] & HAS_SCROLL_ZONES_MASK;
		sensor_info->has_individual_scroll_zones =
			query_buffer[query_offset] &
			HAS_INDIVIDUAL_SCROLL_ZONES_MASK;
		sensor_info->has_multifinger_scroll =
			query_buffer[query_offset] &
			HAS_MULTIFINGER_SCROLL_MASK;
		sensor_info->has_multifinger_scroll_edge_motion =
			query_buffer[query_offset] &
			HAS_MULTIFINGER_SCROLL_EDGE_MOTION_MASK;
		sensor_info->has_multifinger_scroll_inertia =
			query_buffer[query_offset] &
			HAS_MULTIFINGER_SCROLL_INERTIA_MASK;
		query_offset++;
	}

	if (instance_data->device_info->has_query_9) {
		sensor_info->has_pen =
			query_buffer[query_offset] & HAS_PEN_MASK;
		sensor_info->has_proximity =
			query_buffer[query_offset] & HAS_PROXIMITY_MASK;
		sensor_info->has_palm_detect_sensitivity =
			query_buffer[query_offset] &
			HAS_PALM_DETECT_SENSITIVITY_MASK;
		sensor_info->has_suppress_on_palm_detect =
			query_buffer[query_offset] &
			HAS_SUPPRESS_ON_PALM_DETECT_MASK;
		query_offset++;
	}

	if (sensor_info->has_touch_shapes) {
		sensor_info->number_of_touch_shapes =
			(query_buffer[query_offset] &
			NUMBER_OF_TOUCH_SHAPES_MASK) + 1;
		query_offset++;
	}

	if (instance_data->device_info->has_query_11) {
		sensor_info->has_Z_tuning =
			query_buffer[query_offset] & HAS_Z_TUNING_MASK;
		sensor_info->has_algorithm_selection =
			query_buffer[query_offset] & HAS_ALGORITHM_SECTION_MASK;
		query_offset++;
	}

	return 0;
}

/* Figure out just how much data we'll need to read. */
static void compute_finger_data_size(struct rmi_function_info *rmifninfo,
				     struct rmi_F11_sensor_query *sensor_info)
{
	struct f11_instance_data *instance_data = rmifninfo->fndata;
	unsigned int data_buffer_size = 0;

	/* Finger state comes first in the data registers.
	 */
	data_buffer_size =  (sensor_info->number_of_fingers +
		FINGER_STATES_PER_BYTE - 1) / FINGER_STATES_PER_BYTE;

	/* Next comes absolute data.
	 */
	if (sensor_info->has_absolute) {
		instance_data->abs_data_offset = data_buffer_size;
		switch (sensor_info->abs_data_size) {
		case 0:
			instance_data->abs_data_size = 5;
			break;
		default:
			instance_data->abs_data_size =
				DEFAULT_ABS_BYTES_PER_FINGER;
			pr_warning("%s: Unrecognized abs data size %d ignored.",
				__func__, sensor_info->abs_data_size);
		}
		data_buffer_size += sensor_info->number_of_fingers *
			instance_data->abs_data_size;
	}

	/* Then comes the relative data.  Once again, it's optional.
	 */
	if (sensor_info->has_relative) {
		instance_data->rel_data_offset = data_buffer_size;
		data_buffer_size +=
			sensor_info->number_of_fingers * REL_BYTES_PER_FINGER;
	}

	/* Gesture data is next, and it's very optional and very complicated.
	 */
	if (sensor_info->has_gestures) {
		instance_data->gesture_data_offset = data_buffer_size;
		if (sensor_info->has_pinch ||
				sensor_info->has_press ||
				sensor_info->has_flick ||
				sensor_info->has_early_tap ||
				sensor_info->has_double_tap ||
				sensor_info->has_tap ||
				sensor_info->has_tap_and_hold) {
			instance_data->data8_offset = data_buffer_size;
			data_buffer_size++;
			instance_data->data9_offset = data_buffer_size;
			data_buffer_size++;
		} else if (sensor_info->has_palm_detect ||
				sensor_info->has_rotate ||
				sensor_info->has_touch_shapes ||
				sensor_info->has_scroll_zones ||
				sensor_info->has_individual_scroll_zones ||
				sensor_info->has_multifinger_scroll ||
				sensor_info->has_multifinger_scroll_edge_motion
				||
				sensor_info->has_multifinger_scroll_inertia) {
			instance_data->data9_offset = data_buffer_size;
			data_buffer_size++;
		}
		if (sensor_info->has_pinch || sensor_info->has_flick) {
			instance_data->data10_offset = data_buffer_size;
			data_buffer_size++;
		}
		if (sensor_info->has_rotate || sensor_info->has_flick) {
			instance_data->data11_offset = data_buffer_size;
			data_buffer_size += ROTATE_FLICK_DATA_BYTES;
		}
		if (sensor_info->has_touch_shapes) {
			instance_data->data13_offset = data_buffer_size;
			data_buffer_size +=
				(sensor_info->number_of_touch_shapes +
				TOUCH_SHAPE_BITS_PER_BYTE - 1) /
				TOUCH_SHAPE_BITS_PER_BYTE;
		}
		if (sensor_info->has_scroll_zones) {
			instance_data->data14_offset = data_buffer_size;
			data_buffer_size += SCROLL_ZONE_DATA_BYTES;
		}
		if (sensor_info->has_individual_scroll_zones) {
			instance_data->data16_offset = data_buffer_size;
			data_buffer_size += SCROLL_ZONE_DATA_BYTES;
		}
	}

	instance_data->finger_data_buffer_size = data_buffer_size;
}

/* Reading and parsing the F11 query registers is a big hairy wad.  There's a
 * lot of stuff that is dependent on the presence or absence of other stuff,
 * and there's really no tidy way to deal with it.  We can break it down into
 * a few function calls, but some things (like computing finger data size)
 * are just not amenable to further break down.
 */
static int read_query_registers(struct rmi_function_info *rmifninfo)
{
	int retval = 0;
	struct f11_instance_data *instance_data = rmifninfo->fndata;
	unsigned char query_address =
		rmifninfo->function_descriptor.query_base_addr;

	retval = read_device_query(rmifninfo);

	if (retval)
		return retval;

	query_address = query_address + DEVICE_QUERY_SIZE;

	retval = read_per_sensor_query(rmifninfo, instance_data->sensor_info,
			query_address);

	if (retval)
		return retval;

	compute_finger_data_size(rmifninfo, instance_data->sensor_info);

	return 0;
}

static enum finger_state get_finger_state(unsigned char finger,
				     unsigned char *buffer)
{
	int finger_byte = finger / FINGER_STATES_PER_BYTE;
	int finger_shift =
		(finger % FINGER_STATES_PER_BYTE) * BITS_PER_FINGER_STATE;
	return (buffer[finger_byte] >> finger_shift) & FINGER_STATE_MASK;
}

/*
 * This reads in a sample and reports the function $11 source data to the
 * input subsystem. It is used for both polling and interrupt driven
 * operation. This is called a lot so don't put in any informational
 * printks since they will slow things way down!
 */
void FN_11_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs)
{
	/* number of touch points - fingers down in this case */
	int finger_down_count = 0;
	int finger;
	struct rmi_function_device *function_device =
			rmifninfo->function_device;
	struct f11_instance_data *instance_data = rmifninfo->fndata;
	int retval;

	/* get 2D sensor finger data */
	retval = rmi_read_multiple(rmifninfo->sensor,
			rmifninfo->function_descriptor.data_base_addr,
			instance_data->finger_data_buffer,
			instance_data->finger_data_buffer_size);
	if (retval) {
		pr_err("%s: Failed to read finger data registers, code=%d.\n",
		       __func__, retval);
		return;
	}

	/* First we need to count the fingers and generate some events
	 * related to that. */
	for (finger = 0; finger < instance_data->sensor_info->number_of_fingers;
	     finger++) {
		enum finger_state finger_status = get_finger_state(finger,
				instance_data->finger_data_buffer);
		if (finger_status == F11_PRESENT
		    || finger_status == F11_INACCURATE) {
			finger_down_count++;
			instance_data->wasdown = true;
		}
	}
	input_report_key(function_device->input, BTN_TOUCH, finger_down_count);

	for (finger = 0;
	     finger < (instance_data->sensor_info->number_of_fingers - 1);
	     finger++)
		input_report_key(function_device->input, BTN_2 + finger,
				 finger_down_count >= (finger + 2));

	if (instance_data->sensor_info->has_absolute)
		handle_absolute_reports(rmifninfo);

	if (instance_data->sensor_info->has_relative &&
			instance_data->rel_report_enabled)
		handle_relative_report(rmifninfo);

	input_sync(function_device->input); /* sync after groups of events */

}
EXPORT_SYMBOL(FN_11_inthandler);

static void handle_absolute_reports(struct rmi_function_info *rmifninfo)
{
	int finger;
	int finger_down_count = 0;
	struct rmi_function_device *function_device =
			rmifninfo->function_device;
	struct f11_instance_data *instance_data = rmifninfo->fndata;

	for (finger = 0; finger < instance_data->sensor_info->number_of_fingers;
	     finger++) {
		enum finger_state finger_status = get_finger_state(finger,
				instance_data->finger_data_buffer);
		int X = 0, Y = 0, Z = 0, Wy = 0, Wx = 0;

		/* if finger status indicates a finger is present then
		 *   extract the finger data and report it */
		if (finger_status == F11_PRESENT
				|| finger_status == F11_INACCURATE) {
			int max_X = instance_data->control_registers->
				sensor_max_X_pos;
			int max_Y = instance_data->control_registers->
				sensor_max_Y_pos;
			int reg = instance_data->abs_data_offset +
				(finger * instance_data->abs_data_size);

			finger_down_count++;

			X = ((instance_data->finger_data_buffer[reg +
					X_HIGH_BITS_OFFSET] <<
					XY_HIGH_BITS_SHIFT) & XY_HIGH_BITS_MASK)
				| ((instance_data->finger_data_buffer[reg +
					XY_LOW_BITS_OFFSET] >> X_LOW_BITS_SHIFT)
				& XY_LOW_BITS_MASK);
			Y = ((instance_data->finger_data_buffer[reg +
					Y_HIGH_BITS_OFFSET] <<
					XY_HIGH_BITS_SHIFT) & XY_HIGH_BITS_MASK)
				| ((instance_data->finger_data_buffer[reg +
					XY_LOW_BITS_OFFSET] >> Y_LOW_BITS_SHIFT)
					& XY_LOW_BITS_MASK);
			/* First thing to do is swap axes if needed. */
			if (instance_data->swap_axes) {
				int temp = X;
				X = Y;
				Y = temp;
				max_X = instance_data->control_registers->
					sensor_max_Y_pos;
				max_Y = instance_data->control_registers->
					sensor_max_X_pos;
			}
			if (instance_data->flip_X)
				X = max(max_X - X, 0);
			X = X - instance_data->offset_X;
			X = min(max(X, instance_data->clip_X_low),
				instance_data->clip_X_high);
			if (instance_data->flip_Y)
				Y = max(max_Y - Y, 0);
			Y = Y - instance_data->offset_Y;
			Y = min(max(Y, instance_data->clip_Y_low),
				instance_data->clip_Y_high);

			Wx = (instance_data->finger_data_buffer[reg +
				W_XY_OFFSET] >> W_Y_SHIFT) & W_XY_MASK;
			Wy = (instance_data->finger_data_buffer[reg +
				W_XY_OFFSET] >> W_Y_SHIFT) & W_XY_MASK;
			if (instance_data->swap_axes) {
				int temp = Wx;
				Wx = Wy;
				Wy = temp;
			}

			Z = instance_data->finger_data_buffer[reg + Z_OFFSET];
#if	POS_DEBUG
			pr_info("Finger %d - X:%d, Y:%d, Z:%d, Wx:%d, Wy:%d\n",
				 finger, X, Y, Z, Wx, Wy);
#endif

			/* if this is the first finger report normal
			* ABS_X, ABS_Y, PRESSURE, TOOL_WIDTH events for
			* non-MT apps. Apps that support Multi-touch
			* will ignore these events and use the MT
			* events. Apps that don't support Multi-touch
			* will still function.
			*/
			if (finger_down_count == 1) {
				instance_data->old_X = X;
				instance_data->old_Y = Y;
				input_report_abs(function_device->input,
						ABS_X, X);
				input_report_abs(function_device->input,
						ABS_Y, Y);
				input_report_abs(function_device->input,
						ABS_PRESSURE, Z);
				input_report_abs(function_device->input,
						ABS_TOOL_WIDTH, max(Wx, Wy));
			} else {
				/* TODO generate non MT events for
					* multifinger situation. */
			}
#if defined(CONFIG_SYNA_MULTI_TOUCH)
			/* Report Multi-Touch events for each finger */
#if defined(ABS_MT_PRESSURE)
			/* Finger pressure; not supported in all kernel
			 * versions. */
			input_report_abs(function_device->input,
					ABS_MT_PRESSURE, Z);
#endif
			/* major axis of touch area ellipse */
			input_report_abs(function_device->input,
					ABS_MT_TOUCH_MAJOR, max(Wx, Wy));
			/* minor axis of touch area ellipse */
			input_report_abs(function_device->input,
					ABS_MT_TOUCH_MINOR, min(Wx, Wy));
			/* Currently only 2 supported - 1 or 0 */
			input_report_abs(function_device->input,
					ABS_MT_ORIENTATION,
					(Wx > Wy ? 1 : 0));
			input_report_abs(function_device->input,
					ABS_MT_POSITION_X, X);
			input_report_abs(function_device->input,
					ABS_MT_POSITION_Y, Y);

			/* TODO: Tracking ID needs to be reported but
				* not used yet. Could be formed by keeping
				* an id per position and assiging a new id
				* when finger_status changes for that position.
				*/
			input_report_abs(function_device->input,
					ABS_MT_TRACKING_ID,
					finger + 1);

			/* MT sync between fingers */
			input_mt_sync(function_device->input);
#endif
		}
	}

	/* if we had a finger down before and now we don't have
	* any send a button up. */
	if ((finger_down_count == 0) && instance_data->wasdown) {
		instance_data->wasdown = false;

#if defined(CONFIG_SYNA_MULTI_TOUCH)
#if defined(ABS_MT_PRESSURE)
		input_report_abs(function_device->input, ABS_MT_PRESSURE, 0);
#endif
		input_report_abs(function_device->input, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(function_device->input, ABS_MT_TOUCH_MINOR, 0);
		input_report_abs(function_device->input, ABS_MT_POSITION_X,
				instance_data->old_X);
		input_report_abs(function_device->input, ABS_MT_POSITION_Y,
				instance_data->old_Y);
		input_report_abs(function_device->input, ABS_MT_TRACKING_ID, 1);
		input_mt_sync(function_device->input);
#endif

		input_report_abs(function_device->input, ABS_X,
				instance_data->old_X);
		input_report_abs(function_device->input, ABS_Y,
				instance_data->old_Y);
		instance_data->old_X = instance_data->old_Y = 0;
		pr_debug("%s: Finger up.", __func__);
	}
}

#define F11_MIN_RELATIVE -128
#define F11_MAX_RELATIVE 127

/* This function reads in relative data for first finger and
 * sends it to input system */
static void handle_relative_report(struct rmi_function_info *rmifninfo)
{
	struct f11_instance_data *instance_data = rmifninfo->fndata;
	struct rmi_function_device *function_device =
			rmifninfo->function_device;
	signed char X, Y;
	int reg = instance_data->rel_data_offset;


	X = instance_data->finger_data_buffer[reg + REL_X_OFFSET];
	Y = instance_data->finger_data_buffer[reg + REL_Y_OFFSET];
	if (instance_data->swap_axes) {
		signed char temp = X;
		X = Y;
		Y = temp;
	}
	if (instance_data->flip_X)
		X = -X;
	if (instance_data->flip_Y)
		Y = -Y;
	X = (signed char)min(F11_MAX_RELATIVE,
				max(F11_MIN_RELATIVE, (int)X));
	Y = (signed char)min(F11_MAX_RELATIVE,
				max(F11_MIN_RELATIVE, (int)Y));

	input_report_rel(function_device->input, REL_X, X);
	input_report_rel(function_device->input, REL_Y, Y);
}

/* This is a stub for now, and will be expanded as this implementation
 * evolves.
 */
int FN_11_config(struct rmi_function_info *rmifninfo)
{
	int retval = 0;

	pr_debug("%s: RMI4 function $11 config\n", __func__);

	return retval;
}
EXPORT_SYMBOL(FN_11_config);

/* This operation is done in a number of places, so we have a handy routine
 * for it.
 */
static void f11_set_abs_params(struct rmi_function_device *function_device)
{
	struct f11_instance_data *instance_data = function_device->rfi->fndata;
	/* Use the max X and max Y read from the device, or the clip values,
	 * whichever is stricter.
	 */
	int xMin = instance_data->clip_X_low;
	int xMax =
	    min((int)instance_data->control_registers->sensor_max_X_pos,
		instance_data->clip_X_high);
	int yMin = instance_data->clip_Y_low;
	int yMax =
	    min((int)instance_data->control_registers->sensor_max_Y_pos,
		instance_data->clip_Y_high);
	if (instance_data->swap_axes) {
		int temp = xMin;
		xMin = yMin;
		yMin = temp;
		temp = xMax;
		xMax = yMax;
		yMax = temp;
	}
	pr_debug("%s: Set ranges X=[%d..%d] Y=[%d..%d].", __func__, xMin, xMax,
		 yMin, yMax);
	input_set_abs_params(function_device->input, ABS_X, xMin, xMax, 0, 0);
	input_set_abs_params(function_device->input, ABS_Y, yMin, yMax, 0, 0);
	input_set_abs_params(function_device->input, ABS_PRESSURE, 0, 255, 0,
			     0);
	input_set_abs_params(function_device->input, ABS_TOOL_WIDTH, 0, 15, 0,
			     0);

#if defined(CONFIG_SYNA_MULTI_TOUCH)
#if defined(ABS_MT_PRESSURE)
	input_set_abs_params(function_device->input, ABS_MT_PRESSURE, 0, 255,
			     0, 0);
#endif
	input_set_abs_params(function_device->input, ABS_MT_TOUCH_MAJOR, 0, 15,
			     0, 0);
	input_set_abs_params(function_device->input, ABS_MT_TOUCH_MINOR, 0, 15,
			     0, 0);
	input_set_abs_params(function_device->input, ABS_MT_ORIENTATION, 0, 1,
			     0, 0);
	input_set_abs_params(function_device->input, ABS_MT_TRACKING_ID, 1, 10,
			     0, 0);
	input_set_abs_params(function_device->input, ABS_MT_POSITION_X, xMin,
			     xMax, 0, 0);
	input_set_abs_params(function_device->input, ABS_MT_POSITION_Y, yMin,
			     yMax, 0, 0);
#endif
}

/* Initialize any function $11 specific params and settings - input
 * settings, device settings, etc.
 */
int FN_11_init(struct rmi_function_device *function_device)
{
	struct f11_instance_data *instance_data = function_device->rfi->fndata;
	int retval = 0;
	int attr_count = 0;
	struct rmi_f11_functiondata *functiondata =
		rmi_sensor_get_functiondata(function_device->sensor,
			RMI_F11_INDEX);
	pr_debug("%s: RMI4 F11 init", __func__);

	/* TODO: Initialize these through some normal kernel mechanism.
	 */
	instance_data->flip_X = false;
	instance_data->flip_Y = false;
	instance_data->swap_axes = false;
	instance_data->rel_report_enabled = true;
	instance_data->offset_X = instance_data->offset_Y = 0;
	instance_data->clip_X_low = instance_data->clip_Y_low = 0;
	/* TODO: 65536 should actually be the largest valid RMI4
	 * position coordinate */
	instance_data->clip_X_high = instance_data->clip_Y_high = 65536;

	/* Load any overrides that were specified via platform data.
	 */
	if (functiondata) {
		pr_debug("%s: found F11 per function platformdata.", __func__);
		instance_data->flip_X = functiondata->flip_X;
		instance_data->flip_Y = functiondata->flip_Y;
		instance_data->swap_axes = functiondata->swap_axes;
		if (functiondata->offset) {
			instance_data->offset_X = functiondata->offset->x;
			instance_data->offset_Y = functiondata->offset->y;
		}
		if (functiondata->clip_X) {
			if (functiondata->clip_X->min >=
			    functiondata->clip_X->max) {
				pr_warning
				    ("%s: Clip X min (%d) >= X clip max (%d) "
				     "- ignored.",
				     __func__, functiondata->clip_X->min,
				     functiondata->clip_X->max);
			} else {
				instance_data->clip_X_low =
				    functiondata->clip_X->min;
				instance_data->clip_X_high =
				    functiondata->clip_X->max;
			}
		}
		if (functiondata->clip_Y) {
			if (functiondata->clip_Y->min >=
			    functiondata->clip_Y->max) {
				pr_warning
				    ("%s: Clip Y min (%d) >= Y clip max (%d) "
				     "- ignored.",
				     __func__, functiondata->clip_Y->min,
				     functiondata->clip_Y->max);
			} else {
				instance_data->clip_Y_low =
				    functiondata->clip_Y->min;
				instance_data->clip_Y_high =
				    functiondata->clip_Y->max;
			}
		}
	}

	/* need to init the input abs params for the 2D */
	set_bit(EV_ABS, function_device->input->evbit);
	set_bit(EV_SYN, function_device->input->evbit);
	set_bit(EV_KEY, function_device->input->evbit);
	set_bit(BTN_TOUCH, function_device->input->keybit);


	f11_set_abs_params(function_device);

	pr_debug("%s: Creating sysfs files.", __func__);
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&function_device->dev.kobj, &attrs[attr_count].attr) < 0) {
			pr_err
			    ("%s: Failed to create sysfs file for %s.",
			     __func__, attrs[attr_count].attr.name);
			retval = -ENODEV;
			goto error_exit;
		}
	}

	return 0;

error_exit:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&function_device->dev.kobj,
				  &attrs[attr_count].attr);
	/* If you alloc anything, free it here. */
	return retval;
}
EXPORT_SYMBOL(FN_11_init);

int FN_11_detect(struct rmi_function_info *rmifninfo)
{
	unsigned char control_buffer[12]; /* TODO: Compute size correctly. */
	int retval = 0;
	struct f11_instance_data *instance_data;

	pr_debug("%s: RMI4 F11 detect\n", __func__);

	if (rmifninfo->fndata) {
		/* detect routine should only ever be called once
		 * per rmifninfo. */
		pr_err("%s: WTF?!? F11 instance data is already present!",
		       __func__);
		return -EINVAL;
	}
	instance_data = kzalloc(sizeof(struct f11_instance_data), GFP_KERNEL);
	if (!instance_data) {
		pr_err("%s: Error allocating F11 instance data.\n", __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	rmifninfo->fndata = instance_data;

	instance_data->device_info =
	    kzalloc(sizeof(struct rmi_F11_device_query), GFP_KERNEL);
	if (!instance_data->device_info) {
		pr_err("%s: Error allocating F11 device query.\n", __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	instance_data->sensor_info =
	    kzalloc(sizeof(struct rmi_F11_sensor_query), GFP_KERNEL);
	if (!instance_data->sensor_info) {
		pr_err("%s: Error allocating F11 sensor query.\n", __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	retval = read_query_registers(rmifninfo);
	if (retval) {
		pr_err("%s: Failed to read sensor query registers.", __func__);
		goto error_exit;
	}

	instance_data->finger_data_buffer =
	    kcalloc(instance_data->finger_data_buffer_size,
		    sizeof(unsigned char), GFP_KERNEL);
	if (!instance_data->finger_data_buffer) {
		pr_err("%s: Failed to allocate finger data buffer.", __func__);
		retval = -ENOMEM;
		goto error_exit;
	}

	/* Grab a copy of the control registers. */
	instance_data->control_registers =
	    kzalloc(sizeof(struct rmi_F11_control), GFP_KERNEL);
	if (!instance_data->control_registers) {
		pr_err("%s: Error allocating F11 control registers.\n",
		       __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	retval = rmi_read_multiple(rmifninfo->sensor,
			      rmifninfo->function_descriptor.control_base_addr,
			      control_buffer, sizeof(control_buffer));
	if (retval) {
		pr_err("%s: Failed to read F11 control registers.", __func__);
		goto error_exit;
	}
	instance_data->control_registers->sensor_max_X_pos =
	    (((int)control_buffer[7] & 0x0F) << 8) + control_buffer[6];
	instance_data->control_registers->sensor_max_Y_pos =
	    (((int)control_buffer[9] & 0x0F) << 8) + control_buffer[8];
	pr_debug("%s: Max X %d Max Y %d", __func__,
		 instance_data->control_registers->sensor_max_X_pos,
		 instance_data->control_registers->sensor_max_Y_pos);
	return 0;

error_exit:
	if (instance_data) {
		kfree(instance_data->sensor_info);
		kfree(instance_data->device_info);
		kfree(instance_data->control_registers);
		kfree(instance_data->finger_data_buffer);
	}
	kfree(instance_data);
	rmifninfo->fndata = NULL;
	return retval;
}
EXPORT_SYMBOL(FN_11_detect);

static ssize_t rmi_fn_11_maxPos_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u %u\n",
		instance_data->control_registers->sensor_max_X_pos,
		instance_data->control_registers->sensor_max_Y_pos);
}

static ssize_t rmi_fn_11_flip_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u %u\n", instance_data->flip_X,
			instance_data->flip_Y);
}

static ssize_t rmi_fn_11_flip_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;
	unsigned int new_X, new_Y;

	if (sscanf(buf, "%u %u", &new_X, &new_Y) != 2)
		return -EINVAL;
	if (new_X < 0 || new_X > 1 || new_Y < 0 || new_Y > 1)
		return -EINVAL;
	instance_data->flip_X = new_X;
	instance_data->flip_Y = new_Y;

	return count;
}

static ssize_t rmi_fn_11_swap_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->swap_axes);
}

static ssize_t rmi_fn_11_swap_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;
	unsigned int newSwap;

	if (sscanf(buf, "%u", &newSwap) != 1)
		return -EINVAL;
	if (newSwap < 0 || newSwap > 1)
		return -EINVAL;
	instance_data->swap_axes = newSwap;

	f11_set_abs_params(fn);

	return count;
}

static ssize_t rmi_fn_11_relreport_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE,
			"%u\n", instance_data->rel_report_enabled);
}

static ssize_t rmi_fn_11_relreport_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;
	unsigned int new_value;

	if (sscanf(buf, "%u", &new_value) != 1)
		return -EINVAL;
	if (new_value < 0 || new_value > 1)
		return -EINVAL;
	instance_data->rel_report_enabled = new_value;

	return count;
}

static ssize_t rmi_fn_11_offset_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%d %d\n", instance_data->offset_X,
		instance_data->offset_Y);
}

static ssize_t rmi_fn_11_offset_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;
	int new_X, new_Y;

	if (sscanf(buf, "%d %d", &new_X, &new_Y) != 2)
		return -EINVAL;
	instance_data->offset_X = new_X;
	instance_data->offset_Y = new_Y;

	return count;
}

static ssize_t rmi_fn_11_clip_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u %u %u %u\n",
		       instance_data->clip_X_low, instance_data->clip_X_high,
		       instance_data->clip_Y_low, instance_data->clip_Y_high);
}

static ssize_t rmi_fn_11_clip_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f11_instance_data *instance_data = fn->rfi->fndata;
	unsigned int new_X_low, new_X_high, new_Y_low, new_Y_high;

	if (sscanf(buf, "%u %u %u %u",
			&new_X_low, &new_X_high, &new_Y_low, &new_Y_high) != 4)
		return -EINVAL;
	if (new_X_low < 0 || new_X_low >= new_X_high || new_Y_low < 0
			|| new_Y_low >= new_Y_high)
		return -EINVAL;
	instance_data->clip_X_low = new_X_low;
	instance_data->clip_X_high = new_X_high;
	instance_data->clip_Y_low = new_Y_low;
	instance_data->clip_Y_high = new_Y_high;

	f11_set_abs_params(fn);

	return count;
}
