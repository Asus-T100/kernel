/*
 * Support for Aptina MT9E013 camera sensor.
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

/*	NOT FOR INTEGRATION ! DO NOT ADD TO ANDORID GIT TREE */

#ifndef __MT9E013_H__
#define __MT9E013_H__
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

#define	MT9E013_NAME	"mt9e013"
#define	MT9E013_ADDR	0x36
#define MT9E013_ID	0x4b00

#define	LAST_REG_SETING		{0xffff, 0xff}
#define	is_last_reg_setting(item) ((item).reg == 0xffff)
#define I2C_MSG_LENGTH		0x2

#define MT9E013_INVALID_CONFIG	0xffffffff

#define MT9E013_MAX_FOCUS_POS	255
#define MT9E013_MAX_FOCUS_NEG	(-255)

#define MT9E013_INTG_UNIT_US	100
#define MT9E013_MCLK		192

#define MT9E013_REG_BITS	16
#define MT9E013_REG_MASK	0xFFFF

/* This should be added into include/linux/videodev2.h */
#ifndef V4L2_IDENT_MT9E013
#define V4L2_IDENT_MT9E013	8245
#endif

/*
 * mt9e013 System control registers
 */
#define MT9E013_SC_CMMN_CHIP_ID                 0x0000
#define MT9E013_SC_CMMN_REV_ID		        0x0002

#define GROUPED_PARAMETER_UPDATE		0x0000
#define GROUPED_PARAMETER_HOLD			0x0100
#define MT9E013_GROUPED_PARAMETER_HOLD		0x0104

#define MT9E013_VT_PIX_CLK_DIV			0x0300
#define MT9E013_VT_SYS_CLK_DIV			0x0302
#define MT9E013_PRE_PLL_CLK_DIV			0x0304
#define MT9E013_PLL_MULTIPLIER			0x0306
#define MT9E013_OP_PIX_DIV			0x0308
#define MT9E013_OP_SYS_DIV			0x030A
#define MT9E013_FRAME_LENGTH_LINES		0x0340
#define MT9E013_LINE_LENGTH_PCK			0x0342
#define MT9E013_COARSE_INTG_TIME_MIN		0x1004
#define MT9E013_COARSE_INTG_TIME_MAX		0x1006
#define MT9E013_FINE_INTG_TIME_MIN		0x1008
#define MT9E013_FINE_INTG_MIN_DEF		0x4FE
#define MT9E013_FINE_INTG_TIME_MAX		0x100A
#define MT9E013_FINE_INTG_MAX_DEF		0x3EE

#define MT9E013_READ_MODE				0x3040


#define MT9E013_COARSE_INTEGRATION_TIME		0x3012
#define MT9E013_FINE_INTEGRATION_TIME		0x3014
#define MT9E013_ROW_SPEED			0x3016
#define MT9E013_GLOBAL_GAIN			0x305e
#define MT9E013_GLOBAL_GAIN_WR			0x1000
#define MT9E013_TEST_PATTERN_MODE		0x3070
#define MT9E013_VCM_SLEW_STEP			0x30F0
#define MT9E013_VCM_SLEW_STEP_MAX		0x7
#define MT9E013_VCM_SLEW_STEP_MASK		0x7
#define MT9E013_VCM_CODE			0x30F2
#define MT9E013_VCM_SLEW_TIME			0x30F4
#define MT9E013_VCM_SLEW_TIME_MAX		0xffff
#define MT9E013_VCM_ENABLE			0x8000

/* mt9e013 SCCB */
#define MT9E013_SCCB_CTRL			0x3100
#define MT9E013_AEC_PK_EXPO_H			0x3500
#define MT9E013_AEC_PK_EXPO_M			0x3501
#define MT9E013_AEC_PK_EXPO_L			0x3502
#define MT9E013_AEC_MANUAL_CTRL			0x3503
#define MT9E013_AGC_ADJ_H			0x3508
#define MT9E013_AGC_ADJ_L			0x3509

#define MT9E013_FOCAL_LENGTH_NUM	439	/*4.39mm*/
#define MT9E013_FOCAL_LENGTH_DEM	100
#define MT9E013_F_NUMBER_DEFAULT_NUM	24
#define MT9E013_F_NUMBER_DEM	10

#define MT9E013_X_ADDR_MIN	0X1180
#define MT9E013_Y_ADDR_MIN	0X1182
#define MT9E013_X_ADDR_MAX	0X1184
#define MT9E013_Y_ADDR_MAX	0X1186

#define MT9E013_MIN_FRAME_LENGTH_LINES	0x1140
#define MT9E013_MAX_FRAME_LENGTH_LINES	0x1142
#define MT9E013_MIN_LINE_LENGTH_PCK	0x1144
#define MT9E013_MAX_LINE_LENGTH_PCK	0x1146
#define MT9E013_MIN_LINE_BLANKING_PCK	0x1148
#define MT9E013_MIN_FRAME_BLANKING_LINES 0x114A
#define MT9E013_X_OUTPUT_SIZE	0x034C
#define MT9E013_Y_OUTPUT_SIZE	0x034E

#define MT9E013_BIN_FACTOR_MAX			3

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define MT9E013_FOCAL_LENGTH_DEFAULT 0x1B70064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define MT9E013_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define MT9E013_F_NUMBER_RANGE 0x180a180a
#define OTPM_ADD_START_1		0x1000
#define OTPM_DATA_LENGTH_1		0x0100
#define OTPM_COUNT 0x200

/* Defines for register writes and register array processing */
#define MT9E013_BYTE_MAX	30
#define MT9E013_SHORT_MAX	16
#define I2C_RETRY_COUNT		5
#define MT9E013_TOK_MASK	0xfff0

#define	MT9E013_STATUS_POWER_DOWN	0x0
#define	MT9E013_STATUS_STANDBY		0x2
#define	MT9E013_STATUS_ACTIVE		0x3
#define	MT9E013_STATUS_VIEWFINDER	0x4

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

enum mt9e013_tok_type {
	MT9E013_8BIT  = 0x0001,
	MT9E013_16BIT = 0x0002,
	MT9E013_RMW   = 0x0010,
	MT9E013_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	MT9E013_TOK_DELAY  = 0xfe00	/* delay token for reg list */
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
 * struct mt9e013_fwreg - Firmware burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @val: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct mt9e013_fwreg {
	enum mt9e013_tok_type type; /* value, register or FW burst string */
	u16 addr;	/* target address */
	u32 val[8];
};

/**
 * struct mt9e013_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct mt9e013_reg {
	enum mt9e013_tok_type type;
	union {
		u16 sreg;
		struct mt9e013_fwreg *fwreg;
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

struct mt9e013_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

struct mt9e013_resolution {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	bool used;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	const struct mt9e013_reg *regs;
	u8 bin_factor_x;
	u8 bin_factor_y;
	unsigned short skip_frames;
};

struct mt9e013_format {
	u8 *desc;
	u32 pixelformat;
	struct s_register_setting *regs;
};

#define MT9E013_FUSEID_SIZE		8
#define MT9E013_FUSEID_START_ADDR	0x31f4

/* mt9e013 device structure */
struct mt9e013_device {
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
	u16 coarse_itg;
	u16 fine_itg;
	u16 gain;
	u32 focus;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	int run_mode;
	struct timespec timestamp_t_focus_abs;
	s16 number_of_steps;
	struct mutex input_lock; /* serialize sensor's ioctl */
	void *otp_data;
	void *fuseid;
	/* Older VCMs could not maintain the focus position in standby mode. */
	bool keeps_focus_pos;
};

#define MT9E013_MAX_WRITE_BUF_SIZE	32
struct mt9e013_write_buffer {
	u16 addr;
	u8 data[MT9E013_MAX_WRITE_BUF_SIZE];
};

struct mt9e013_write_ctrl {
	int index;
	struct mt9e013_write_buffer buffer;
};

#define MT9E013_OTP_START_ADDR		0x3800
#define MT9E013_OTP_DATA_SIZE		456
#define MT9E013_OTP_READY_REG		0x304a
#define MT9E013_OTP_READY_REG_DONE	(1 << 5)
#define MT9E013_OTP_READY_REG_OK	(1 << 6)

static const struct mt9e013_reg mt9e013_otp_type30[] = {
	{MT9E013_16BIT, {0x3134}, 0xcd95},
	{MT9E013_16BIT, {0x304c}, 0x3000},
	{MT9E013_16BIT, {0x304a}, 0x0010},
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_otp_type31[] = {
	{MT9E013_16BIT, {0x3134}, 0xcd95},
	{MT9E013_16BIT, {0x304c}, 0x3100},
	{MT9E013_16BIT, {0x304a}, 0x0010},
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_otp_type32[] = {
	{MT9E013_16BIT, {0x3134}, 0xcd95},
	{MT9E013_16BIT, {0x304c}, 0x3200},
	{MT9E013_16BIT, {0x304a}, 0x0010},
	{MT9E013_TOK_TERM, {0}, 0}
};

#define MT9E013_OTP_CHECKSUM		1
#define MT9E013_OTP_MOD_CHECKSUM	255

/*
 * Checksum entries in OTP data:
 * @start: start offset of checksum's input data
 * @end: end offset of checksum's input data
 * @checksum: offset where checksum is placed
 */
struct mt9e013_otp_checksum_format {
	u16 start;
	u16 end;
	u16 checksum;
};

static const struct mt9e013_otp_checksum_format
mt9e013_otp_checksum_list[] = {
	{0x0004, 0x00d7, 0x00e1},
	{0x00d8, 0x00df, 0x00e0},
	{0x00e4, 0x01b7, 0x01c1},
	{0x01b8, 0x01bf, 0x01c0},
	{0x0000, 0x01c3, 0x01c4},
};

#define MAX_FMTS 1

#define MT9E013_RES_WIDTH_MAX	3280
#define MT9E013_RES_HEIGHT_MAX	2464

/* Recommended Settings 29 Mar 2011*/
static const struct mt9e013_reg mt9e013_recommended_settings[] = {
	{MT9E013_16BIT, {0x3044}, 0x0590},
	{MT9E013_16BIT, {0x306E}, 0xFC80},
	{MT9E013_16BIT, {0x30B2}, 0xC000},
	{MT9E013_16BIT, {0x30D6}, 0x0800},
	{MT9E013_16BIT, {0x316C}, 0xB42F},
	{MT9E013_16BIT, {0x316E}, 0x869A},
	{MT9E013_16BIT, {0x3170}, 0x210E},
	{MT9E013_16BIT, {0x317A}, 0x010E},
	{MT9E013_16BIT, {0x31E0}, 0x1FB9},
	{MT9E013_16BIT, {0x31E6}, 0x07FC},
	{MT9E013_16BIT, {0x37C0}, 0x0000},
	{MT9E013_16BIT, {0x37C2}, 0x0000},
	{MT9E013_16BIT, {0x37C4}, 0x0000},
	{MT9E013_16BIT, {0x37C6}, 0x0000},
	{MT9E013_16BIT, {0x3E00}, 0x0011},
	{MT9E013_16BIT, {0x3E02}, 0x8801},
	{MT9E013_16BIT, {0x3E04}, 0x2801},
	{MT9E013_16BIT, {0x3E06}, 0x8449},
	{MT9E013_16BIT, {0x3E08}, 0x6841},
	{MT9E013_16BIT, {0x3E0A}, 0x400C},
	{MT9E013_16BIT, {0x3E0C}, 0x1001},
	{MT9E013_16BIT, {0x3E0E}, 0x2603},
	{MT9E013_16BIT, {0x3E10}, 0x4B41},
	{MT9E013_16BIT, {0x3E12}, 0x4B24},
	{MT9E013_16BIT, {0x3E14}, 0xA3CF},
	{MT9E013_16BIT, {0x3E16}, 0x8802},
	{MT9E013_16BIT, {0x3E18}, 0x84FF},
	{MT9E013_16BIT, {0x3E1A}, 0x8601},
	{MT9E013_16BIT, {0x3E1C}, 0x8401},
	{MT9E013_16BIT, {0x3E1E}, 0x840A},
	{MT9E013_16BIT, {0x3E20}, 0xFF00},
	{MT9E013_16BIT, {0x3E22}, 0x8401},
	{MT9E013_16BIT, {0x3E24}, 0x00FF},
	{MT9E013_16BIT, {0x3E26}, 0x0088},
	{MT9E013_16BIT, {0x3E28}, 0x2E8A},
	{MT9E013_16BIT, {0x3E30}, 0x0000},
	{MT9E013_16BIT, {0x3E32}, 0x8801},
	{MT9E013_16BIT, {0x3E34}, 0x4029},
	{MT9E013_16BIT, {0x3E36}, 0x00FF},
	{MT9E013_16BIT, {0x3E38}, 0x8469},
	{MT9E013_16BIT, {0x3E3A}, 0x00FF},
	{MT9E013_16BIT, {0x3E3C}, 0x2801},
	{MT9E013_16BIT, {0x3E3E}, 0x3E2A},
	{MT9E013_16BIT, {0x3E40}, 0x1C01},
	{MT9E013_16BIT, {0x3E42}, 0xFF84},
	{MT9E013_16BIT, {0x3E44}, 0x8401},
	{MT9E013_16BIT, {0x3E46}, 0x0C01},
	{MT9E013_16BIT, {0x3E48}, 0x8401},
	{MT9E013_16BIT, {0x3E4A}, 0x00FF},
	{MT9E013_16BIT, {0x3E4C}, 0x8402},
	{MT9E013_16BIT, {0x3E4E}, 0x8984},
	{MT9E013_16BIT, {0x3E50}, 0x6628},
	{MT9E013_16BIT, {0x3E52}, 0x8340},
	{MT9E013_16BIT, {0x3E54}, 0x00FF},
	{MT9E013_16BIT, {0x3E56}, 0x4A42},
	{MT9E013_16BIT, {0x3E58}, 0x2703},
	{MT9E013_16BIT, {0x3E5A}, 0x6752},
	{MT9E013_16BIT, {0x3E5C}, 0x3F2A},
	{MT9E013_16BIT, {0x3E5E}, 0x846A},
	{MT9E013_16BIT, {0x3E60}, 0x4C01},
	{MT9E013_16BIT, {0x3E62}, 0x8401},
	{MT9E013_16BIT, {0x3E66}, 0x3901},
	{MT9E013_16BIT, {0x3E90}, 0x2C01},
	{MT9E013_16BIT, {0x3E98}, 0x2B02},
	{MT9E013_16BIT, {0x3E92}, 0x2A04},
	{MT9E013_16BIT, {0x3E94}, 0x2509},
	{MT9E013_16BIT, {0x3E96}, 0x0000},
	{MT9E013_16BIT, {0x3E9A}, 0x2905},
	{MT9E013_16BIT, {0x3E9C}, 0x00FF},
	{MT9E013_16BIT, {0x3ECC}, 0x00EB},
	{MT9E013_16BIT, {0x3ED0}, 0x1E24},
	{MT9E013_16BIT, {0x3ED4}, 0xAFC4},
	{MT9E013_16BIT, {0x3ED6}, 0x909B},
	{MT9E013_16BIT, {0x3EE0}, 0x2424},
	{MT9E013_16BIT, {0x3EE2}, 0x9797},
	{MT9E013_16BIT, {0x3EE4}, 0xC100},
	{MT9E013_16BIT, {0x3EE6}, 0x0540},
	{MT9E013_16BIT, {0x3174}, 0x8000},
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_pll_timing[] = {
	/*			pixelrate into the isp =	153.600.000 Hz		 */
	{MT9E013_16BIT, {0x0300},	0x0004	}, /*	vt_pix_clk_div=	4	internal pixel clock freq =	192.000.000 Hz		 */
	{MT9E013_16BIT, {0x0302},	0x0001	}, /*	vt_sys_clk_div=	1				 */
	{MT9E013_16BIT, {0x0304},	0x0001	}, /*	pre_pll_clk_div=	1	PLL input clock freq =	19.200.000 Hz		 */
	{MT9E013_16BIT, {0x0306},	0x0028	}, /*	pll_multiplier=	40	mipi bus speed =	768.000.000 Hz		 */
	{MT9E013_16BIT, {0x0308},	0x000A	}, /*	op_pix_clk_div=	10	output pixel clock freq =	76.800.000 Hz		 */
	{MT9E013_16BIT, {0x030A},	0x0001	}, /*	op_sys_clk_div=	1				 */
	{MT9E013_16BIT, {0x3016},	0x111	}, /*	row_speed=	273				 */
	{MT9E013_TOK_DELAY, {0}, 1},
	{MT9E013_TOK_TERM, {0}, 0}
};


/*2-lane MIPI Interface Configuration*/
static const struct mt9e013_reg mt9e013_mipi_config[] = {
	{MT9E013_16BIT+MT9E013_RMW, {0x3064}, 0x0100, 0x0000},
	{MT9E013_16BIT, {0x31AE}, 0x0202},
	{MT9E013_16BIT, {0x31B8}, 0x03EF},
	/*{MT9E013_16BIT, {0x31B8}, 0x2FEF}, */
	{MT9E013_TOK_DELAY, {0}, 5},
	{MT9E013_TOK_TERM, {0}, 0}
};

/* MIPI Timing Settings */
static const struct mt9e013_reg mt9e013_mipi_timing[] = {
	{MT9E013_16BIT, {0x31B0}, 0x0083},
	{MT9E013_16BIT, {0x31B2}, 0x004D},
	{MT9E013_16BIT, {0x31B4}, 0x0E88},
	{MT9E013_16BIT, {0x31B6}, 0x0D24},
	{MT9E013_16BIT, {0x31B8}, 0x020E},
	{MT9E013_16BIT, {0x31BA}, 0x0710},
	{MT9E013_16BIT, {0x31BC}, 0x2A0D},
	{MT9E013_16BIT, {0x31BE}, 0xC007},
	{MT9E013_TOK_DELAY, {0}, 5},
	{MT9E013_TOK_TERM, {0}, 0}
};

/* Start Streaming
 * reset_register_restart_bad = 1
 * reset_register_mask_bad = 1
 * reset_register_lock_reg = 1
 * grouped_parameter_hold = 0
 * reset_register_stream = 1 */

static const struct mt9e013_reg mt9e013_start_streaming[] = {
	{MT9E013_16BIT+MT9E013_RMW, {0x301A}, 0x0200, 0x1},
	{MT9E013_16BIT+MT9E013_RMW, {0x301A}, 0x0400, 0x1},
	{MT9E013_16BIT+MT9E013_RMW, {0x301A}, 0x8, 0x1},
	{MT9E013_16BIT, {0x0104}, 0x0},
	{MT9E013_16BIT+MT9E013_RMW, {0x301A}, 0x4, 0x1},
	{MT9E013_TOK_TERM, {0}, 0}
};

#define GROUPED_PARAMETER_HOLD_ENABLE	{MT9E013_8BIT, {0x0104}, 0x1}

#define GROUPED_PARAMETER_HOLD_DISABLE	{MT9E013_8BIT, {0x0104}, 0x0}

#define INIT_VCM_CONTROL {MT9E013_16BIT, {0x30F0}, 0x800C} /* slew_rate[2:0] */
static const struct mt9e013_reg mt9e013_init_vcm[] = {
	INIT_VCM_CONTROL,				   /* VCM_CONTROL */
	{MT9E013_16BIT, {0x30F2}, 0x0000}, /* VCM_NEW_CODE */
	{MT9E013_16BIT, {0x30F4}, 0x0080}, /* VCM_STEP_TIME */
	{MT9E013_TOK_TERM, {0}, 0}
};

#define RESET_REGISTER	{MT9E013_16BIT, {0x301A}, 0x4A38}
static const struct mt9e013_reg mt9e013_reset_register[] = {
	RESET_REGISTER,
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_raw_10[] = {
	{MT9E013_16BIT, {0x0112}, 0x0A0A}, /* CCP_DATA_FORMAT, set to RAW10 mode */
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_scaler[] = {
	{MT9E013_16BIT, {0x0400}, 0x0000}, /* SCALE_MODE: 0:disable */
	{MT9E013_16BIT, {0x0404}, 0x0010}, /* SCALE_M = 16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

/*****************************still15ok*****************************/
static struct mt9e013_reg const mt9e013_STILL_8M_15fps[] = {
	/*	STILL 8M */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x0AA4	}, /*	FRAME_LENGTH_LINES	2724 */
	{MT9E013_16BIT, {0x0342},	0x1258	}, /*	LINE_LENGTH_PCK	4696 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x0000	}, /*	Y_ADDR_START	0 */
	{MT9E013_16BIT, {0x0348},	0x0CCF	}, /*	X_ADDR_END	3279 */
	{MT9E013_16BIT, {0x034A},	0x099F	}, /*	Y_ADDR_END	2463 */
	{MT9E013_16BIT, {0x034C},	0x0CD0	}, /*	X_OUTPUT_SIZE	3280 */
	{MT9E013_16BIT, {0x034E},	0x09A0	}, /*	Y_OUTPUT_SIZE	2464 */
	{MT9E013_16BIT, {0x3040},	0x0041	}, /*	READ_MODE	0 0 0 0 0 0 0 1 1 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x0}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0078	}, /*	FINE_CORRECTION	120 */
	{MT9E013_16BIT, {0X3012},	0x04CA	}, /*	COARSE_INTEGRATION_TIME	1226 */
	{MT9E013_16BIT, {0X3014},	0x03F6	}, /*	FINE_INTEGRATION_TIME	1014 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0000	}, /*	SCALE_MODE	0 */
	{MT9E013_16BIT, {0x0404},	0x0010	}, /*	SCALE_M	16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_STILL_6M_15fps[] = {
	/*	STILL 6M */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x0AA4	}, /*	FRAME_LENGTH_LINES	2724 */
	{MT9E013_16BIT, {0x0342},	0x1258	}, /*	LINE_LENGTH_PCK	4696 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x0134	}, /*	Y_ADDR_START	308 */
	{MT9E013_16BIT, {0x0348},	0x0CCF	}, /*	X_ADDR_END	3279 */
	{MT9E013_16BIT, {0x034A},	0x086D	}, /*	Y_ADDR_END	2157 */
	{MT9E013_16BIT, {0x034C},	0x0CD0	}, /*	X_OUTPUT_SIZE	3280 */
	{MT9E013_16BIT, {0x034E},	0x0738	}, /*	Y_OUTPUT_SIZE	1848 */
	{MT9E013_16BIT, {0x3040},	0x0041	}, /*	READ_MODE	0 0 0 0 0 0 0 1 1 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x0}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0078	}, /*	FINE_CORRECTION	120 */
	{MT9E013_16BIT, {0X3012},	0x04CA	}, /*	COARSE_INTEGRATION_TIME	1226 */
	{MT9E013_16BIT, {0X3014},	0x03F6	}, /*	FINE_INTEGRATION_TIME	1014 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0000	}, /*	SCALE_MODE	0 */
	{MT9E013_16BIT, {0x0404},	0x0010	}, /*	SCALE_M	16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_STILL_2M_15fps[] = {
	/*	STILL 2M */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x0C1C	}, /*	FRAME_LENGTH_LINES	3100 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x0000	}, /*	Y_ADDR_START	0 */
	{MT9E013_16BIT, {0x0348},	0x0CD1	}, /*	X_ADDR_END	3281 */
	{MT9E013_16BIT, {0x034A},	0x09A1	}, /*	Y_ADDR_END	2465 */
	{MT9E013_16BIT, {0x034C},	0x0668	}, /*	X_OUTPUT_SIZE	1640 */
	{MT9E013_16BIT, {0x034E},	0x04D0	}, /*	Y_OUTPUT_SIZE	1232 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0000	}, /*	SCALE_MODE	0 */
	{MT9E013_16BIT, {0x0404},	0x0010	}, /*	SCALE_M	16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

/*****************************preview30ok********************************/
static struct mt9e013_reg const mt9e013_PREVIEW_30fps[] = {
	/* PREVIEW */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x0000	}, /*	Y_ADDR_START	0 */
	{MT9E013_16BIT, {0x0348},	0x0CCF	}, /*	X_ADDR_END	3279 */
	{MT9E013_16BIT, {0x034A},	0x099F	}, /*	Y_ADDR_END	2463 */
	{MT9E013_16BIT, {0x034C},	0x0334	}, /*	X_OUTPUT_SIZE	820 */
	{MT9E013_16BIT, {0x034E},	0x0268	}, /*	Y_OUTPUT_SIZE	616 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0002	}, /*	SCALE_MODE	2 */
	{MT9E013_16BIT, {0x0404},	0x0020	}, /*	SCALE_M	32 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_WIDE_PREVIEW_30fps[] = {
	/* WIDE PREVIEW */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x0114	}, /*	Y_ADDR_START	276 */
	{MT9E013_16BIT, {0x0348},	0x0CCF	}, /*	X_ADDR_END	3279 */
	{MT9E013_16BIT, {0x034A},	0x088D	}, /*	Y_ADDR_END	2189 */
	{MT9E013_16BIT, {0x034C},	0x0668	}, /*	X_OUTPUT_SIZE	1640 */
	{MT9E013_16BIT, {0x034E},	0x03BC	}, /*	Y_OUTPUT_SIZE	956 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0000	}, /*	SCALE_MODE	0 */
	{MT9E013_16BIT, {0x0404},	0x0010	}, /*	SCALE_M	16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

/*****************************video************************/
static struct mt9e013_reg const mt9e013_1080p_strong_dvs_30fps[] = {
	/*	1080p strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x05AB	}, /*	FRAME_LENGTH_LINES	1451 */
	{MT9E013_16BIT, {0x0342},	0x113A	}, /*	LINE_LENGTH_PCK	4410 */
	{MT9E013_16BIT, {0x0344},	0x01D8	}, /*	X_ADDR_START	472 */
	{MT9E013_16BIT, {0x0346},	0x0242	}, /*	Y_ADDR_START	578 */
	{MT9E013_16BIT, {0x0348},	0x0AF7	}, /*	X_ADDR_END	2807 */
	{MT9E013_16BIT, {0x034A},	0x075D	}, /*	Y_ADDR_END	1885 */
	{MT9E013_16BIT, {0x034C},	0x0920	}, /*	X_OUTPUT_SIZE	2336 */
	{MT9E013_16BIT, {0x034E},	0x051C	}, /*	Y_OUTPUT_SIZE	1308 */
	{MT9E013_16BIT, {0x3040},	0x0041	}, /*	READ_MODE	0 0 0 0 0 0 0 1 1  */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x0}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0078	}, /*	FINE_CORRECTION	120 */
	{MT9E013_16BIT, {0X3012},	0x05AB	}, /*	COARSE_INTEGRATION_TIME	1451 */
	{MT9E013_16BIT, {0X3014},	0x0442	}, /*	FINE_INTEGRATION_TIME	1090 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0000	}, /*	SCALE_MODE	0 */
	{MT9E013_16BIT, {0x0404},	0x0010	}, /*	SCALE_M	16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_720p_strong_dvs_30fps[] = {
	/*	720p strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0048	}, /*	X_ADDR_START	72 */
	{MT9E013_16BIT, {0x0346},	0x0160	}, /*	Y_ADDR_START	352 */
	{MT9E013_16BIT, {0x0348},	0x0C89	}, /*	X_ADDR_END	3209 */
	{MT9E013_16BIT, {0x034A},	0x083F	}, /*	Y_ADDR_END	2111 */
	{MT9E013_16BIT, {0x034C},	0x0620	}, /*	X_OUTPUT_SIZE	1568 */
	{MT9E013_16BIT, {0x034E},	0x0370	}, /*	Y_OUTPUT_SIZE	880 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0002	}, /*	SCALE_MODE	2 */
	{MT9E013_16BIT, {0x0404},	0x0010	}, /*	SCALE_M	16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_WVGA_strong_dvs_30fps[] = {
	/*	WVGA strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x00D0	}, /*	Y_ADDR_START	208 */
	{MT9E013_16BIT, {0x0348},	0x0CCD	}, /*	X_ADDR_END	3277 */
	{MT9E013_16BIT, {0x034A},	0x08CD	}, /*	Y_ADDR_END	2253 */
	{MT9E013_16BIT, {0x034C},	0x0668	}, /*	X_OUTPUT_SIZE	1640 */
	{MT9E013_16BIT, {0x034E},	0x0400	}, /*	Y_OUTPUT_SIZE	1024 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0000	}, /*	SCALE_MODE	0 */
	{MT9E013_16BIT, {0x0404},	0x0010	}, /*	SCALE_M	16 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_D1PAL_strong_dvs_30fps[] = {
	/*	D1PAL strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0134	}, /*	X_ADDR_START	308 */
	{MT9E013_16BIT, {0x0346},	0x0024	}, /*	Y_ADDR_START	36 */
	{MT9E013_16BIT, {0x0348},	0x0B9B	}, /*	X_ADDR_END	2971 */
	{MT9E013_16BIT, {0x034A},	0x0957	}, /*	Y_ADDR_END	2391 */
	{MT9E013_16BIT, {0x034C},	0x0370	}, /*	X_OUTPUT_SIZE	880 */
	{MT9E013_16BIT, {0x034E},	0x02C4	}, /*	Y_OUTPUT_SIZE	708 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0002	}, /*	SCALE_MODE	2 */
	{MT9E013_16BIT, {0x0404},	0x001B	}, /*	SCALE_M	27 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_D1NTSC_strong_dvs_30fps[] = {
	/*	D1NTSC strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x008C	}, /*	Y_ADDR_START	140 */
	{MT9E013_16BIT, {0x0348},	0x0CCF	}, /*	X_ADDR_END	3279 */
	{MT9E013_16BIT, {0x034A},	0x0887	}, /*	Y_ADDR_END	 2183 */
	{MT9E013_16BIT, {0x034C},	0x0388	}, /*	X_OUTPUT_SIZE	904 */
	{MT9E013_16BIT, {0x034E},	0x025A	}, /*	Y_OUTPUT_SIZE	602 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0002	}, /*	SCALE_MODE	2 */
	{MT9E013_16BIT, {0x0404},	0x001D	}, /*	SCALE_M	29 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_VGA_strong_dvs_30fps[] = {
	/*	VGA strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0000	}, /*	X_ADDR_START	0 */
	{MT9E013_16BIT, {0x0346},	0x0000	}, /*	Y_ADDR_START	0 */
	{MT9E013_16BIT, {0x0348},	0x0CCF	}, /*	X_ADDR_END	3279 */
	{MT9E013_16BIT, {0x034A},	0x099F	}, /*	Y_ADDR_END	2463 */
	{MT9E013_16BIT, {0x034C},	0x0334	}, /*	X_OUTPUT_SIZE	820 */
	{MT9E013_16BIT, {0x034E},	0x0268	}, /*	Y_OUTPUT_SIZE	616 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0002	}, /*	SCALE_MODE	2 */
	{MT9E013_16BIT, {0x0404},	0x0020	}, /*	SCALE_M	32 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_QVGA_strong_dvs_30fps[] = {
	/*	QVGA strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0008	}, /*	X_ADDR_START	8 */
	{MT9E013_16BIT, {0x0346},	0x0000	}, /*	Y_ADDR_START	0 */
	{MT9E013_16BIT, {0x0348},	0x0CC7	}, /*	X_ADDR_END	3271 */
	{MT9E013_16BIT, {0x034A},	0x099F	}, /*	Y_ADDR_END	2463 */
	{MT9E013_16BIT, {0x034C},	0x0198	}, /*	X_OUTPUT_SIZE	408 */
	{MT9E013_16BIT, {0x034E},	0x0134	}, /*	Y_OUTPUT_SIZE	308 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0002	}, /*	SCALE_MODE	2 */
	{MT9E013_16BIT, {0x0404},	0x0040	}, /*	SCALE_M	64 */
	{MT9E013_TOK_TERM, {0}, 0}
};

static struct mt9e013_reg const mt9e013_QCIF_strong_dvs_30fps[] = {
	/* QCIF strong dvs */
	GROUPED_PARAMETER_HOLD_ENABLE,
	/* Frame size & Timing Configuration*/
	{MT9E013_16BIT, {0x0340},	0x060E	}, /*	FRAME_LENGTH_LINES	1550 */
	{MT9E013_16BIT, {0x0342},	0x1020	}, /*	LINE_LENGTH_PCK	4128 */
	{MT9E013_16BIT, {0x0344},	0x0080	}, /*	X_ADDR_START	128 */
	{MT9E013_16BIT, {0x0346},	0x0000	}, /*	Y_ADDR_START	0 */
	{MT9E013_16BIT, {0x0348},	0x0CB3	}, /*	X_ADDR_END	3251 */
	{MT9E013_16BIT, {0x034A},	0x099F	}, /*	Y_ADDR_END	2463 */
	{MT9E013_16BIT, {0x034C},	0x00D8	}, /*	X_OUTPUT_SIZE	216 */
	{MT9E013_16BIT, {0x034E},	0x00B0	}, /*	Y_OUTPUT_SIZE	176 */
	{MT9E013_16BIT, {0x3040},	0x04C3	}, /*	READ_MODE	0 0 0 0 0 1 0 3 3 */
	{MT9E013_16BIT | MT9E013_RMW, {0x306E}, 0x0010, 0x1}, /* DATAPATH_SELECT_TRUE_BAYER */
	/* Initial integration time */
	{MT9E013_16BIT, {0x3010},	0x0130	}, /*	FINE_CORRECTION	304 */
	{MT9E013_16BIT, {0X3012},	0x0573	}, /*	COARSE_INTEGRATION_TIME	1395 */
	{MT9E013_16BIT, {0X3014},	0x0846	}, /*	FINE_INTEGRATION_TIME	2118 */
	/* Scaler configuration */
	{MT9E013_16BIT, {0x0400},	0x0002	}, /*	SCALE_MODE	2 */
	{MT9E013_16BIT, {0x0404},	0x0070	}, /*	SCALE_M	112 */
	{MT9E013_TOK_TERM, {0}, 0}
};

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/

static const struct mt9e013_reg mt9e013_soft_standby[] = {
	{MT9E013_8BIT, {0x301C}, 0x00},
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_streaming[] = {
	{MT9E013_8BIT, {0x301C}, 0x01},
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_param_hold[] = {
	{MT9E013_8BIT, {0x0104}, 0x01},	/* GROUPED_PARAMETER_HOLD */
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_param_update[] = {
	{MT9E013_8BIT, {0x0104}, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{MT9E013_TOK_TERM, {0}, 0}
};

static const struct mt9e013_reg mt9e013_lens_shading[] = {
	{MT9E013_16BIT | MT9E013_RMW, {0x3780}, 0x8000, 0}, /* POLY_SC_ENABLE */
	{MT9E013_16BIT, {0x3600}, 0x0430},	/* P_GR_P0Q0 */
	{MT9E013_16BIT, {0x3602}, 0x1BEE},	/* P_GR_P0Q1 */
	{MT9E013_16BIT, {0x3604}, 0x39F0},	/* P_GR_P0Q2 */
	{MT9E013_16BIT, {0x3606}, 0xC7AD},	/* P_GR_P0Q3 */
	{MT9E013_16BIT, {0x3608}, 0xC390},	/* P_GR_P0Q4 */
	{MT9E013_16BIT, {0x360A}, 0x03D0},	/* P_RD_P0Q0 */
	{MT9E013_16BIT, {0x360C}, 0xA0CE},	/* P_RD_P0Q1 */
	{MT9E013_16BIT, {0x360E}, 0x2850},	/* P_RD_P0Q2 */
	{MT9E013_16BIT, {0x3610}, 0x6A0E},	/* P_RD_P0Q3 */
	{MT9E013_16BIT, {0x3612}, 0xAF30},	/* P_RD_P0Q4 */
	{MT9E013_16BIT, {0x3614}, 0x03D0},	/* P_BL_P0Q0 */
	{MT9E013_16BIT, {0x3616}, 0x36AE},	/* P_BL_P0Q1 */
	{MT9E013_16BIT, {0x3618}, 0x5E6F},	/* P_BL_P0Q2 */
	{MT9E013_16BIT, {0x361A}, 0xA22E},	/* P_BL_P0Q3 */
	{MT9E013_16BIT, {0x361C}, 0xF6EF},	/* P_BL_P0Q4 */
	{MT9E013_16BIT, {0x361E}, 0x02F0},	/* P_GB_P0Q0 */
	{MT9E013_16BIT, {0x3620}, 0xA00E},	/* P_GB_P0Q1 */
	{MT9E013_16BIT, {0x3622}, 0x3CD0},	/* P_GB_P0Q2 */
	{MT9E013_16BIT, {0x3624}, 0x530E},	/* P_GB_P0Q3 */
	{MT9E013_16BIT, {0x3626}, 0xCEF0},	/* P_GB_P0Q4 */
	{MT9E013_16BIT, {0x3640}, 0xAB2D},	/* P_GR_P1Q0 */
	{MT9E013_16BIT, {0x3642}, 0xB72E},	/* P_GR_P1Q1 */
	{MT9E013_16BIT, {0x3644}, 0x988D},	/* P_GR_P1Q2 */
	{MT9E013_16BIT, {0x3646}, 0x6E2E},	/* P_GR_P1Q3 */
	{MT9E013_16BIT, {0x3648}, 0x53EE},	/* P_GR_P1Q4 */
	{MT9E013_16BIT, {0x364A}, 0xDA2C},	/* P_RD_P1Q0 */
	{MT9E013_16BIT, {0x364C}, 0x3E8D},	/* P_RD_P1Q1 */
	{MT9E013_16BIT, {0x364E}, 0xAFAD},	/* P_RD_P1Q2 */
	{MT9E013_16BIT, {0x3650}, 0x874E},	/* P_RD_P1Q3 */
	{MT9E013_16BIT, {0x3652}, 0x5B4E},	/* P_RD_P1Q4 */
	{MT9E013_16BIT, {0x3654}, 0x740D},	/* P_BL_P1Q0 */
	{MT9E013_16BIT, {0x3656}, 0x310E},	/* P_BL_P1Q1 */
	{MT9E013_16BIT, {0x3658}, 0x280B},	/* P_BL_P1Q2 */
	{MT9E013_16BIT, {0x365A}, 0xE06E},	/* P_BL_P1Q3 */
	{MT9E013_16BIT, {0x365C}, 0xEA0D},	/* P_BL_P1Q4 */
	{MT9E013_16BIT, {0x365E}, 0x182D},	/* P_GB_P1Q0 */
	{MT9E013_16BIT, {0x3660}, 0xAD0E},	/* P_GB_P1Q1 */
	{MT9E013_16BIT, {0x3662}, 0x032E},	/* P_GB_P1Q2 */
	{MT9E013_16BIT, {0x3664}, 0x7EEE},	/* P_GB_P1Q3 */
	{MT9E013_16BIT, {0x3666}, 0xF34E},	/* P_GB_P1Q4 */
	{MT9E013_16BIT, {0x3680}, 0x0E31},	/* P_GR_P2Q0 */
	{MT9E013_16BIT, {0x3682}, 0x104F},	/* P_GR_P2Q1 */
	{MT9E013_16BIT, {0x3684}, 0x92D3},	/* P_GR_P2Q2 */
	{MT9E013_16BIT, {0x3686}, 0xA030},	/* P_GR_P2Q3 */
	{MT9E013_16BIT, {0x3688}, 0x3873},	/* P_GR_P2Q4 */
	{MT9E013_16BIT, {0x368A}, 0x1971},	/* P_RD_P2Q0 */
	{MT9E013_16BIT, {0x368C}, 0x750C},	/* P_RD_P2Q1 */
	{MT9E013_16BIT, {0x368E}, 0xFFF2},	/* P_RD_P2Q2 */
	{MT9E013_16BIT, {0x3690}, 0xEDAF},	/* P_RD_P2Q3 */
	{MT9E013_16BIT, {0x3692}, 0x1D73},	/* P_RD_P2Q4 */
	{MT9E013_16BIT, {0x3694}, 0x0031},	/* P_BL_P2Q0 */
	{MT9E013_16BIT, {0x3696}, 0x1A2F},	/* P_BL_P2Q1 */
	{MT9E013_16BIT, {0x3698}, 0xF792},	/* P_BL_P2Q2 */
	{MT9E013_16BIT, {0x369A}, 0x8530},	/* P_BL_P2Q3 */
	{MT9E013_16BIT, {0x369C}, 0x1F73},	/* P_BL_P2Q4 */
	{MT9E013_16BIT, {0x369E}, 0x08B1},	/* P_GB_P2Q0 */
	{MT9E013_16BIT, {0x36A0}, 0x11AE},	/* P_GB_P2Q1 */
	{MT9E013_16BIT, {0x36A2}, 0x9093},	/* P_GB_P2Q2 */
	{MT9E013_16BIT, {0x36A4}, 0x9030},	/* P_GB_P2Q3 */
	{MT9E013_16BIT, {0x36A6}, 0x36D3},	/* P_GB_P2Q4 */
	{MT9E013_16BIT, {0x36C0}, 0x5F2D},	/* P_GR_P3Q0 */
	{MT9E013_16BIT, {0x36C2}, 0x314F},	/* P_GR_P3Q1 */
	{MT9E013_16BIT, {0x36C4}, 0x684E},	/* P_GR_P3Q2 */
	{MT9E013_16BIT, {0x36C6}, 0x88B0},	/* P_GR_P3Q3 */
	{MT9E013_16BIT, {0x36C8}, 0xDAF0},	/* P_GR_P3Q4 */
	{MT9E013_16BIT, {0x36CA}, 0x636E},	/* P_RD_P3Q0 */
	{MT9E013_16BIT, {0x36CC}, 0xAD0C},	/* P_RD_P3Q1 */
	{MT9E013_16BIT, {0x36CE}, 0xEEEE},	/* P_RD_P3Q2 */
	{MT9E013_16BIT, {0x36D0}, 0x500E},	/* P_RD_P3Q3 */
	{MT9E013_16BIT, {0x36D2}, 0xDDCE},	/* P_RD_P3Q4 */
	{MT9E013_16BIT, {0x36D4}, 0xA3AC},	/* P_BL_P3Q0 */
	{MT9E013_16BIT, {0x36D6}, 0xC06E},	/* P_BL_P3Q1 */
	{MT9E013_16BIT, {0x36D8}, 0xC04F},	/* P_BL_P3Q2 */
	{MT9E013_16BIT, {0x36DA}, 0x49AF},	/* P_BL_P3Q3 */
	{MT9E013_16BIT, {0x36DC}, 0x4830},	/* P_BL_P3Q4 */
	{MT9E013_16BIT, {0x36DE}, 0x0F6B},	/* P_GB_P3Q0 */
	{MT9E013_16BIT, {0x36E0}, 0x1DEF},	/* P_GB_P3Q1 */
	{MT9E013_16BIT, {0x36E2}, 0x8730},	/* P_GB_P3Q2 */
	{MT9E013_16BIT, {0x36E4}, 0x9E50},	/* P_GB_P3Q3 */
	{MT9E013_16BIT, {0x36E6}, 0x7110},	/* P_GB_P3Q4 */
	{MT9E013_16BIT, {0x3700}, 0xF4F1},	/* P_GR_P4Q0 */
	{MT9E013_16BIT, {0x3702}, 0xF090},	/* P_GR_P4Q1 */
	{MT9E013_16BIT, {0x3704}, 0x6493},	/* P_GR_P4Q2 */
	{MT9E013_16BIT, {0x3706}, 0x5FB1},	/* P_GR_P4Q3 */
	{MT9E013_16BIT, {0x3708}, 0xADB3},	/* P_GR_P4Q4 */
	{MT9E013_16BIT, {0x370A}, 0xFEF1},	/* P_RD_P4Q0 */
	{MT9E013_16BIT, {0x370C}, 0x134B},	/* P_RD_P4Q1 */
	{MT9E013_16BIT, {0x370E}, 0x4D33},	/* P_RD_P4Q2 */
	{MT9E013_16BIT, {0x3710}, 0x9B8E},	/* P_RD_P4Q3 */
	{MT9E013_16BIT, {0x3712}, 0x88B3},	/* P_RD_P4Q4 */
	{MT9E013_16BIT, {0x3714}, 0xEBB1},	/* P_BL_P4Q0 */
	{MT9E013_16BIT, {0x3716}, 0x8131},	/* P_BL_P4Q1 */
	{MT9E013_16BIT, {0x3718}, 0x5AD3},	/* P_BL_P4Q2 */
	{MT9E013_16BIT, {0x371A}, 0x54F1},	/* P_BL_P4Q3 */
	{MT9E013_16BIT, {0x371C}, 0xB193},	/* P_BL_P4Q4 */
	{MT9E013_16BIT, {0x371E}, 0xE6D1},	/* P_GB_P4Q0 */
	{MT9E013_16BIT, {0x3720}, 0xE0EC},	/* P_GB_P4Q1 */
	{MT9E013_16BIT, {0x3722}, 0x6033},	/* P_GB_P4Q2 */
	{MT9E013_16BIT, {0x3724}, 0x9DCE},	/* P_GB_P4Q3 */
	{MT9E013_16BIT, {0x3726}, 0xA453},	/* P_GB_P4Q4 */
	{MT9E013_16BIT, {0x3782}, 0x0614},	/* POLY_ORIGIN_C */
	{MT9E013_16BIT, {0x3784}, 0x0494},	/* POLY_ORIGIN_R */
	{MT9E013_16BIT, {0x37C0}, 0xC40A},	/* P_GR_Q5 */
	{MT9E013_16BIT, {0x37C2}, 0xCE6A},	/* P_RD_Q5 */
	{MT9E013_16BIT, {0x37C4}, 0xDBAA},	/* P_BL_Q5 */
	{MT9E013_16BIT, {0x37C6}, 0xCCEA},	/* P_GB_Q5 */

	/*STATE= Lens Correction Falloff, 70 */
	{MT9E013_16BIT | MT9E013_RMW, {0x3780}, 0x8000, 1}, /* POLY_SC_ENABLE */
	{MT9E013_TOK_TERM, {0}, 0}
};

#endif
