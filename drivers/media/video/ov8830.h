/*
 * Support for Omnivision OV8830 camera sensor.
 * Based on Aptina mt9e013 driver.
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

#ifndef __OV8830_H__
#define __OV8830_H__
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <linux/types.h>
#include <media/media-entity.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#define to_drv201_device(sd)	(&container_of(sd, struct ov8830_device, sd) \
				->drv201)

#define DRV201_I2C_ADDR				0x0E
#define DRV201_CONTROL				2
#define DRV201_VCM_CURRENT			3
#define DRV201_STATUS				5
#define DRV201_MODE				6
#define DRV201_VCM_FREQ				7

#define DRV201_MAX_FOCUS_POS			1023

/* drv201 device structure */
struct drv201_device {
	s32 focus;			/* Current focus value */
	struct timespec focus_time;	/* Time when focus was last time set */
	__u8 buffer[4];			/* Used for i2c transactions */
	const struct camera_af_platform_data *platform_data;
};

#define	OV8830_NAME	"ov8830"
#define	OV8830_ADDR	0x36
#define OV8830_ID	0x4b00

#define	LAST_REG_SETING		{0xffff, 0xff}
#define	is_last_reg_setting(item) ((item).reg == 0xffff)
#define I2C_MSG_LENGTH		0x2

#define OV8830_INVALID_CONFIG	0xffffffff

#define OV8830_INTG_UNIT_US	100
#define OV8830_MCLK		192

#define OV8830_REG_BITS	16
#define OV8830_REG_MASK	0xFFFF

/* This should be added into include/linux/videodev2.h */
#ifndef V4L2_IDENT_OV8830
#define V4L2_IDENT_OV8830	8245
#endif

/*
 * ov8830 System control registers
 */
#define OV8830_PLL_PLL10			0x3090
#define OV8830_PLL_PLL11			0x3091
#define OV8830_PLL_PLL12			0x3092
#define OV8830_PLL_PLL13			0x3093
#define	OV8830_TIMING_VTS			0x380E
#define OV8830_TIMING_HTS			0x380C

#define OV8830_SC_CMMN_CHIP_ID_H		0x0000
#define OV8830_SC_CMMN_CHIP_ID_L		0x0001

#define GROUPED_PARAMETER_UPDATE		0x0000
#define GROUPED_PARAMETER_HOLD			0x0100
#define OV8830_GROUPED_PARAMETER_HOLD		0x0104

#define OV8830_VT_PIX_CLK_DIV			0x0300
#define OV8830_VT_SYS_CLK_DIV			0x0302
#define OV8830_PRE_PLL_CLK_DIV			0x0304
#define OV8830_PLL_MULTIPLIER			0x0306
#define OV8830_OP_PIX_DIV			0x0308
#define OV8830_OP_SYS_DIV			0x030A
#define OV8830_FRAME_LENGTH_LINES		0x0340
#define OV8830_LINE_LENGTH_PCK			0x0342
#define OV8830_COARSE_INTG_TIME_MIN		0x1004
#define OV8830_COARSE_INTG_TIME_MAX		0x1006
#define OV8830_FINE_INTG_TIME_MIN		0x1008
#define OV8830_FINE_INTG_MIN_DEF		0x4FE
#define OV8830_FINE_INTG_TIME_MAX		0x100A
#define OV8830_FINE_INTG_MAX_DEF		0x3EE

#define OV8830_READ_MODE				0x3040
#define OV8830_LONG_EXPO			0x3500
#define OV8830_AGC_ADJ				0x350B
#define OV8830_TEST_PATTERN_MODE		0x3070

/* ov8830 SCCB */
#define OV8830_SCCB_CTRL			0x3100
#define OV8830_AEC_PK_EXPO_H			0x3500
#define OV8830_AEC_PK_EXPO_M			0x3501
#define OV8830_AEC_PK_EXPO_L			0x3502
#define OV8830_AEC_MANUAL_CTRL			0x3503
#define OV8830_AGC_ADJ_H			0x3508
#define OV8830_AGC_ADJ_L			0x3509

#define OV8830_FOCAL_LENGTH_NUM	439	/*4.39mm*/
#define OV8830_FOCAL_LENGTH_DEM	100
#define OV8830_F_NUMBER_DEFAULT_NUM	24
#define OV8830_F_NUMBER_DEM	10

#define OV8830_X_ADDR_MIN	0X1180
#define OV8830_Y_ADDR_MIN	0X1182
#define OV8830_X_ADDR_MAX	0X1184
#define OV8830_Y_ADDR_MAX	0X1186

#define OV8830_MIN_FRAME_LENGTH_LINES	0x1140
#define OV8830_MAX_FRAME_LENGTH_LINES	0x1142
#define OV8830_MIN_LINE_LENGTH_PCK	0x1144
#define OV8830_MAX_LINE_LENGTH_PCK	0x1146
#define OV8830_MIN_LINE_BLANKING_PCK	0x1148
#define OV8830_MIN_FRAME_BLANKING_LINES 0x114A
#define OV8830_X_OUTPUT_SIZE	0x034C
#define OV8830_Y_OUTPUT_SIZE	0x034E


/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV8830_FOCAL_LENGTH_DEFAULT 0x1B70064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define OV8830_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define OV8830_F_NUMBER_RANGE 0x180a180a
#define OTPM_ADD_START_1		0x1000
#define OTPM_DATA_LENGTH_1		0x0100
#define OTPM_COUNT 0x200

/* Defines for register writes and register array processing */
#define OV8830_BYTE_MAX	32
#define OV8830_SHORT_MAX	16
#define I2C_RETRY_COUNT		5
#define OV8830_TOK_MASK	0xfff0

#define	OV8830_STATUS_POWER_DOWN	0x0
#define	OV8830_STATUS_STANDBY		0x2
#define	OV8830_STATUS_ACTIVE		0x3
#define	OV8830_STATUS_VIEWFINDER	0x4

#define	v4l2_format_capture_type_entry(_width, _height, \
		_pixelformat, _bytesperline, _colorspace) \
	{\
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,\
		.fmt.pix.width = (_width),\
		.fmt.pix.height = (_height),\
		.fmt.pix.pixelformat = (_pixelformat),\
		.fmt.pix.bytesperline = (_bytesperline),\
		.fmt.pix.colorspace = (_colorspace),\
		.fmt.pix.sizeimage = (_height)*(_bytesperline),\
	}

#define	s_output_format_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps) \
	{\
		.v4l2_fmt = v4l2_format_capture_type_entry(_width, \
			_height, _pixelformat, _bytesperline, \
				_colorspace),\
		.fps = (_fps),\
	}

#define	s_output_format_reg_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps, _reg_setting) \
	{\
		.s_fmt = s_output_format_entry(_width, _height,\
				_pixelformat, _bytesperline, \
				_colorspace, _fps),\
		.reg_setting = (_reg_setting),\
	}

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

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


#define	macro_string_entry(VAL)	\
	{ \
		.val = VAL, \
		.string = #VAL, \
	}

enum ov8830_tok_type {
	OV8830_8BIT  = 0x0001,
	OV8830_16BIT = 0x0002,
	OV8830_RMW   = 0x0010,
	OV8830_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	OV8830_TOK_DELAY  = 0xfe00	/* delay token for reg list */
};

/*
 * If register address or register width is not 32 bit width,
 * user needs to convert it manually
 */

struct s_register_setting {
	u32 reg;
	u32 val;
};

struct s_output_format {
	struct v4l2_format v4l2_fmt;
	int fps;
};

/**
 * struct ov8830_fwreg - Firmware burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @val: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct ov8830_fwreg {
	enum ov8830_tok_type type; /* value, register or FW burst string */
	u16 addr;	/* target address */
	u32 val[8];
};

/**
 * struct ov8830_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct ov8830_reg {
	enum ov8830_tok_type type;
	union {
		u16 sreg;
		struct ov8830_fwreg *fwreg;
	} reg;
	u32 val;	/* @set value for read/mod/write, @mask */
	u32 val2;	/* optional: for rmw, OR mask */
};

/* Store macro values' debug names */
struct macro_string {
	u8 val;
	char *string;
};

static inline const char *
macro_to_string(const struct macro_string *array, int size, u8 val)
{
	int i;
	for (i = 0; i < size; i++) {
		if (array[i].val == val)
			return array[i].string;
	}
	return "Unknown VAL";
}

struct ov8830_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

struct ov8830_resolution {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	bool used;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	const struct ov8830_reg *regs;
};

struct ov8830_format {
	u8 *desc;
	u32 pixelformat;
	struct s_register_setting *regs;
};

/* ov8830 device structure */
struct ov8830_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int fmt_idx;
	int status;
	int streaming;
	int power;
	u8 res;
	u8 type;
	u16 sensor_id;
	u8 sensor_revision;
	int exposure;
	int gain;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	int run_mode;
	struct drv201_device drv201;
};

/*
 * The i2c adapter on Intel Medfield can transfer 32 bytes maximum
 * at a time. In burst mode we require that the buffer is transferred
 * in one shot, so limit the buffer size to 32 bytes minus a safety.
 */
#define OV8830_MAX_WRITE_BUF_SIZE	30
struct ov8830_write_buffer {
	u16 addr;
	u8 data[OV8830_MAX_WRITE_BUF_SIZE];
};

struct ov8830_write_ctrl {
	int index;
	struct ov8830_write_buffer buffer;
};

#define MAX_FMTS 1

#define OV8830_RES_WIDTH_MAX	3280
#define OV8830_RES_HEIGHT_MAX	2464

#endif
