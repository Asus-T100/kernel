/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#ifndef _lm3554_H_
#define _lm3554_H_

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/mutex.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/ioctl.h>

#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>

#define LEDFLASH_LM3554_NAME    "lm3554"
#define LEDFLASH_LM3554_ID      3554

#define	v4l2_queryctrl_entry_integer(_id, _name,\
		_minimum, _maximum, _step, \
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_INTEGER, \
		.name = _name, \
		.minimum = (_minimum), \
		.maximum = (_maximum), \
		.step = (_step), \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}
#define	v4l2_queryctrl_entry_boolean(_id, _name,\
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_BOOLEAN, \
		.name = _name, \
		.minimum = 0, \
		.maximum = 1, \
		.step = 1, \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}

#define	s_ctrl_id_entry_integer(_id, _name, \
		_minimum, _maximum, _step, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_integer(_id, _name,\
				_minimum, _maximum, _step,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

#define	s_ctrl_id_entry_boolean(_id, _name, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_boolean(_id, _name,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

/* Registers */
#define LM3554_TORCH_BRIGHTNESS_REG     0xA0
#define LM3554_FLASH_BRIGHTNESS_REG     0xB0
#define LM3554_FLASH_DURATION_REG       0xC0
#define LM3554_FLAGS_REG                0xD0
#define LM3554_CONFIG_REG_1             0xE0
#define LM3554_CONFIG_REG_2             0xF0

/* Value settings for Flash Time-out Duration*/
#define LM3554_DEFAULT_TIMEOUT          512U
#define LM3554_MIN_TIMEOUT              32U
#define LM3554_MAX_TIMEOUT              1024U
#define LM3554_TIMEOUT_STEPSIZE         32U

/* Flash modes */
#define LM3554_MODE_SHUTDOWN            0
#define LM3554_MODE_INDICATOR           1
#define LM3554_MODE_TORCH               2
#define LM3554_MODE_FLASH               3

/* timer delay time */
#define LM3554_TIMER_DELAY		5

/* Flags */
#define LM3554_FLAG_TIMEOUT                  (1<<0)
#define LM3554_FLAG_THERMAL_SHUTDOWN         (1<<1)
#define LM3554_FLAG_LED_FAULT                (1<<2)
#define LM3554_FLAG_TX1_INTERRUPT            (1<<3)
#define LM3554_FLAG_TX2_INTERRUPT            (1<<4)
#define LM3554_FLAG_LED_THERMAL_FAULT        (1<<5)
#define LM3554_FLAG_INPUT_VOLTAGE_LOW        (1<<7)

/* Percentage <-> value macros */
#define LM3554_MIN_PERCENT                   0U
#define LM3554_MAX_PERCENT                   100U
#define LM3554_CLAMP_PERCENTAGE(val) \
	clamp(val, LM3554_MIN_PERCENT, LM3554_MAX_PERCENT)
/* we add 1 to the value to end up in the range [min..100%]
 * rather than in the range [0..max] where max < 100 */
#define LM3554_VALUE_TO_PERCENT(v, step)     ((((v)+1) * (step)) / 100)
/* we subtract 1 from the percentage to make sure we round down into
 * the valid range of [0..max] and not [1..max+1] */
#define LM3554_PERCENT_TO_VALUE(p, step)     ((((p)-1) * 100) / (step))

/* Flash brightness, input is percentage, output is [0..15] */
#define LM3554_FLASH_STEP                    909
#define LM3554_FLASH_DEFAULT_BRIGHTNESS \
	LM3554_VALUE_TO_PERCENT(13, LM3554_FLASH_STEP)

/* Torch brightness, input is percentage, output is [0..7] */
#define LM3554_TORCH_STEP                    1250
#define LM3554_TORCH_DEFAULT_BRIGHTNESS \
	LM3554_VALUE_TO_PERCENT(2, LM3554_TORCH_STEP)

/* Indicator brightness, input is percentage, output is [0..3] */
#define LM3554_INDICATOR_STEP                2500
#define LM3554_INDICATOR_DEFAULT_BRIGHTNESS \
	LM3554_VALUE_TO_PERCENT(1, LM3554_INDICATOR_STEP)

#endif /* _LM3554_H_ */

