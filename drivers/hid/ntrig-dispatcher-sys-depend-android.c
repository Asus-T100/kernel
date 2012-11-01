/*
 *  HID driver for N-Trig touchscreens
 *
 *  Copyright (c) 2011 N-TRIG
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/device.h>
#include "typedef-ntrig.h"
#include "ntrig-common.h"
#include "ntrig-dispatcher.h"
#include "ntrig-dispatcher-sysfs.h"
#include "ntrig-dispatcher-sys-depend.h"

void
config_multi_touch(struct ntrig_bus_device *dev, struct input_dev *input_device)
{
	int i;

	__set_bit(EV_ABS,		input_device->evbit);

	__set_bit(ABS_MT_POSITION_X,	input_device->absbit);
	__set_bit(ABS_MT_POSITION_Y,	input_device->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR,	input_device->absbit);
#ifdef MT_REPORT_TYPE_B
	__set_bit(ABS_MT_WIDTH_MINOR,	input_device->absbit);
	__set_bit(ABS_MT_PRESSURE,		input_device->absbit);
	__set_bit(ABS_MT_TOOL_TYPE,     input_device->absbit);
#else
	__set_bit(ABS_MT_TOUCH_MAJOR,	input_device->absbit);
#endif
	/**
	 *   [48/0x30] - ABS_MT_TOUCH_MAJOR  0 .. 40
	 *   [50/0x32] - ABS_MT_WIDTH_MAJOR  0 .. 8000
	 *   [53/0x35] - ABS_MT_POSITION_X   0 .. 1023
	 *   [54/0x36] - ABS_MT_POSITION_Y   0.. 599
	 *   ABS_MT_POSITION_Y =
	 */
	ntrig_dbg("inside %s before VIRTUAL_KEYS_SUPPORTED\n", __func__);

	if (virtual_keys_supported()) {
		ntrig_dbg("%s: VIRTUAL_KEYS_SUPPORTED1\n", __func__);

		__set_bit(EV_KEY, input_device->evbit);
		for (i = 0; i < get_virtual_keys_num(); i++)
			__set_bit(get_virt_keys_scan_code(i),
					input_device->keybit);

		input_set_abs_params(input_device, ABS_MT_POSITION_X,
			dev->logical_min_x + get_touch_screen_border_left(),
			dev->logical_max_x - get_touch_screen_border_right(),
			0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y,
			dev->logical_min_y+get_touch_screen_border_down(),
			dev->logical_max_y-get_touch_screen_border_up(),
			0, 0);
		ntrig_dbg("%s: VIRTUAL_KEYS_SUPPORTED2\n", __func__);
	} else {
		ntrig_dbg("%s: VIRTUAL_KEYS_SUPPORTED undefined\n", __func__);
		input_set_abs_params(input_device, ABS_MT_POSITION_X,
				dev->logical_min_x, dev->logical_max_x,	0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y,
				dev->logical_min_y, dev->logical_max_y,	0, 0);
	}

	input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0,
						ABS_MT_WIDTH_MAX, 0, 0);
	input_set_abs_params(input_device, ABS_MT_WIDTH_MINOR, 0,
						ABS_MT_WIDTH_MAX, 0, 0);
#ifdef MT_REPORT_TYPE_B
	input_set_abs_params(input_device, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_device, ABS_MT_TOOL_TYPE, 0, 1, 0, 0);
#else
	input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0,
						ABS_MT_TOUCH_MAJOR_MAX, 0, 0);
#endif
	/** ABS_MT_ORIENTATION: we use 0 or 1 values, we only detect
	 *  90 degree rotation (by looking at the maximum of dx and
	 *  dy reported by sensor */
	input_set_abs_params(input_device, ABS_MT_ORIENTATION, 0, 1, 0, 0);
	input_set_abs_params(input_device, ABS_MT_TRACKING_ID,
					0, ABS_MT_TRACKING_ID_MAX, 0, 0);

	ntrig_dbg("inside %s after VIRTUAL_KEYS_SUPPORTED\n", __func__);
}

void ntrig_simulate_single_touch(struct input_dev *input,
					struct device_finger_t *finger)
{

}
