/*
 * Support for Sony IMX camera sensor.
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

#ifndef __IMX_H__
#define __IMX_H__
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include "imx135.h"

#define I2C_MSG_LENGTH		0x2

#define IMX_MCLK		192

/* TODO - This should be added into include/linux/videodev2.h */
#ifndef V4L2_IDENT_IMX
#define V4L2_IDENT_IMX	8245
#endif

/*
 * imx System control registers
 */
#define IMX_MASK_5BIT	0x1F
#define IMX_MASK_4BIT	0xF
#define IMX_MASK_2BIT	0x3
#define IMX_MASK_11BIT	0x7FF
#define IMX_INTG_BUF_COUNT		2

#define IMX_FINE_INTG_TIME		0x1E8

#define IMX_VT_PIX_CLK_DIV			0x0301
#define IMX_VT_SYS_CLK_DIV			0x0303
#define IMX_PRE_PLL_CLK_DIV			0x0305
#define IMX_PLL_MULTIPLIER			0x030C
#define IMX_OP_PIX_DIV			0x0309
#define IMX_OP_SYS_DIV			0x030B
#define IMX_FRAME_LENGTH_LINES		0x0340
#define IMX_LINE_LENGTH_PIXELS		0x0342
#define IMX_COARSE_INTG_TIME_MIN	0x1004
#define IMX_COARSE_INTG_TIME_MAX	0x1006
#define IMX_BINNING_ENABLE		0x0390
#define IMX_BINNING_TYPE		0x0391

#define IMX_CROP_X_START		0x0344
#define IMX_CROP_Y_START		0x0346
#define IMX_CROP_X_END			0x0348
#define IMX_CROP_Y_END			0x034A
#define IMX_OUTPUT_WIDTH		0x034C
#define IMX_OUTPUT_HEIGHT		0x034E

#define IMX_READ_MODE			0x0390

#define IMX_COARSE_INTEGRATION_TIME		0x0202
#define IMX_TEST_PATTERN_MODE			0x0600
#define IMX_IMG_ORIENTATION			0x0101
#define IMX_VFLIP_BIT			1
#define IMX_GLOBAL_GAIN			0x0205
#define IMX_DGC_ADJ			0x020E

/* Defines for register writes and register array processing */
#define IMX_BYTE_MAX	30
#define IMX_SHORT_MAX	16
#define I2C_RETRY_COUNT		5
#define IMX_TOK_MASK	0xfff0

#define MAX_FMTS 1

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


struct imx_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

/* imx device structure */
struct imx_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct camera_sensor_platform_data *platform_data;
	struct mutex input_lock; /* serialize sensor's ioctl */
	int fmt_idx;
	int status;
	int streaming;
	int power;
	int run_mode;
	int vt_pix_clk_freq_mhz;
	u32 focus;
	u16 sensor_id;
	u16 coarse_itg;
	u16 fine_itg;
	u16 gain;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	u8 res;
	u8 type;
	u8 sensor_revision;
};

#define to_imx_sensor(x) container_of(x, struct imx_device, sd)

#define IMX_MAX_WRITE_BUF_SIZE	32
struct imx_write_buffer {
	u16 addr;
	u8 data[IMX_MAX_WRITE_BUF_SIZE];
};

struct imx_write_ctrl {
	int index;
	struct imx_write_buffer buffer;
};

struct sensor_mode_data {
	u32 coarse_integration_time_min;
	u32 coarse_integration_time_max_margin;
	u32 fine_integration_time_min;
	u32 fine_integration_time_max_margin;
	u32 fine_integration_time_def;
	u32 frame_length_lines;
	u32 line_length_pck;
	u32 read_mode;
	int vt_pix_clk_freq_mhz;
};

static const struct imx_reg imx_soft_standby[] = {
	{IMX_8BIT, 0x0100, 0x00},
	{IMX_TOK_TERM, 0, 0}
};

static const struct imx_reg imx_streaming[] = {
	{IMX_8BIT, 0x0100, 0x01},
	{IMX_TOK_TERM, 0, 0}
};

static const struct imx_reg imx_param_hold[] = {
	{IMX_8BIT, 0x0104, 0x01},	/* GROUPED_PARAMETER_HOLD */
	{IMX_TOK_TERM, 0, 0}
};

static const struct imx_reg imx_param_update[] = {
	{IMX_8BIT, 0x0104, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{IMX_TOK_TERM, 0, 0}
};

/* FIXME - to be removed when real OTP data is ready */
static const u8 otpdata[] = {
	2, 1, 3, 10, 0, 1, 233, 2, 92, 2, 207, 0, 211, 1, 70, 1,
	185, 0, 170, 1, 29, 1, 144, 2, 92, 2, 207, 3, 66, 2, 78, 169,
	94, 151, 9, 7, 5, 163, 121, 96, 77, 71, 78, 96, 123, 160, 132, 97,
	66, 47, 40, 47, 66, 97, 132, 117, 80, 49, 28, 21, 28, 49, 80, 119,
	113, 74, 43, 22, 15, 22, 42, 74, 115, 118, 81, 51, 31, 23, 30, 50,
	81, 119, 131, 98, 70, 51, 44, 51, 68, 98, 131, 181, 123, 100, 83, 76,
	83, 100, 123, 162, 74, 52, 40, 31, 28, 31, 40, 53, 72, 58, 40, 25,
	16, 13, 16, 25, 40, 57, 50, 32, 16, 6, 3, 7, 17, 31, 51, 48,
	29, 14, 3, 0, 4, 14, 29, 49, 50, 32, 17, 8, 4, 8, 17, 32,
	51, 58, 40, 27, 18, 15, 18, 26, 41, 58, 84, 53, 43, 34, 31, 34,
	42, 54, 75, 74, 52, 40, 30, 27, 31, 39, 52, 72, 59, 40, 25, 16,
	12, 16, 25, 40, 58, 51, 33, 17, 7, 3, 7, 17, 32, 52, 49, 30,
	14, 3, 0, 4, 14, 30, 50, 52, 33, 18, 8, 4, 8, 18, 33, 52,
	59, 41, 27, 18, 15, 18, 26, 41, 58, 85, 54, 43, 34, 31, 34, 42,
	54, 76, 123, 91, 74, 61, 57, 62, 74, 93, 123, 99, 73, 52, 39, 35,
	40, 53, 74, 100, 88, 61, 40, 26, 21, 27, 41, 62, 90, 85, 57, 36,
	21, 17, 22, 37, 58, 88, 89, 61, 41, 27, 22, 28, 41, 62, 91, 98,
	73, 53, 41, 37, 41, 53, 74, 100, 137, 91, 76, 64, 60, 64, 76, 93,
	127, 5, 114, 83, 64, 49, 45, 50, 64, 83, 110, 92, 65, 41, 26, 21,
	26, 41, 65, 91, 80, 52, 28, 11, 6, 11, 27, 52, 82, 77, 48, 23,
	7, 2, 7, 23, 47, 79, 81, 53, 29, 14, 8, 13, 29, 53, 82, 90,
	66, 44, 30, 24, 29, 43, 66, 91, 127, 83, 67, 54, 49, 54, 67, 84,
	113, 77, 54, 42, 32, 30, 33, 42, 56, 77, 60, 41, 26, 17, 14, 17,
	27, 42, 61, 52, 33, 17, 7, 3, 7, 18, 33, 54, 50, 30, 14, 3,
	0, 4, 15, 31, 53, 53, 33, 18, 8, 4, 8, 18, 34, 55, 60, 42,
	28, 19, 16, 19, 28, 43, 62, 88, 56, 45, 36, 33, 37, 45, 58, 81,
	79, 55, 42, 32, 29, 32, 42, 56, 77, 62, 43, 27, 17, 13, 17, 27,
	43, 62, 55, 35, 18, 7, 3, 7, 18, 35, 56, 53, 32, 15, 4, 0,
	4, 15, 32, 54, 55, 35, 19, 8, 4, 8, 19, 35, 57, 62, 43, 28,
	19, 15, 19, 28, 44, 63, 90, 57, 45, 36, 32, 35, 45, 58, 82, 184,
	136, 111, 93, 88, 96, 114, 141, 186, 147, 109, 80, 63, 57, 65, 83, 113,
	153, 131, 93, 63, 44, 38, 46, 66, 96, 138, 127, 87, 58, 37, 31, 39,
	60, 91, 135, 132, 93, 64, 46, 39, 47, 66, 97, 139, 147, 109, 82, 65,
	59, 66, 83, 113, 152, 203, 138, 114, 97, 91, 98, 116, 141, 192, 2, 8,
	2, 223, 2, 222, 1, 249, 2, 186, 2, 223, 2, 221, 1, 147
};

#define IMX_INVALID_CONFIG	0xffffffff
#define IMX_MAX_FOCUS_POS	1023
#define IMX_MAX_FOCUS_NEG	(-1023)
#define IMX_VCM_SLEW_STEP_MAX	0x3f
#define IMX_VCM_SLEW_TIME_MAX	0x1f

extern int imx_vcm_power_up(struct v4l2_subdev *sd);
extern int imx_vcm_power_down(struct v4l2_subdev *sd);
extern int imx_vcm_init(struct v4l2_subdev *sd);
extern int imx_t_focus_abs(struct v4l2_subdev *sd, s32 value);
extern int imx_t_focus_rel(struct v4l2_subdev *sd, s32 value);
extern int imx_q_focus_status(struct v4l2_subdev *sd, s32 *value);
extern int imx_q_focus_abs(struct v4l2_subdev *sd, s32 *value);

#endif

