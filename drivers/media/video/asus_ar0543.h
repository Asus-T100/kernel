/*
 * Support for Aptina AR0543 camera sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
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

#ifndef __AR0543_H__
#define __AR0543_H__
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

#define AR0543_RES_WIDTH_MAX	2592
#define AR0543_RES_HEIGHT_MAX	1944
#define MAX_FMTS 1

#define	AR0543_NAME	"ar0543"
#define	AR0543_ADDR	0x36
#define AR0543_ID	0x4800
#define AR0543_ID2	0x4b01

#define	LAST_REG_SETING		{0xffff, 0xff}
#define	is_last_reg_setting(item) ((item).reg == 0xffff)
#define I2C_MSG_LENGTH		0x2

#define AR0543_INVALID_CONFIG	0xffffffff

#define AR0543_MAX_FOCUS_POS	255
#define AR0543_MAX_FOCUS_NEG	(-255)

#define AR0543_INTG_UNIT_US	100
#define AR0543_MCLK		192

#define AR0543_REG_BITS	16
#define AR0543_REG_MASK	0xFFFF

/* This should be added into include/linux/videodev2.h */
#ifndef V4L2_IDENT_AR0543
#define V4L2_IDENT_AR0543	8245
#endif

/*
 * ar0543 System control registers
 */
#define AR0543_SC_CMMN_CHIP_ID                 0x0000
#define AR0543_SC_CMMN_REV_ID		        0x0002

#define GROUPED_PARAMETER_UPDATE		0x0000
#define GROUPED_PARAMETER_HOLD			0x0100
#define AR0543_GROUPED_PARAMETER_HOLD		0x0104

#define AR0543_VT_PIX_CLK_DIV			0x0300
#define AR0543_VT_SYS_CLK_DIV			0x0302
#define AR0543_PRE_PLL_CLK_DIV			0x0304
#define AR0543_PLL_MULTIPLIER			0x0306
#define AR0543_OP_PIX_DIV			0x0308
#define AR0543_OP_SYS_DIV			0x030A
#define AR0543_FRAME_LENGTH_LINES		0x0340
#define AR0543_LINE_LENGTH_PCK			0x0342
#define AR0543_COARSE_INTG_TIME_MIN		0x1004
#define AR0543_COARSE_INTG_TIME_MAX		0x1006
#define AR0543_FINE_INTG_TIME_MIN		0x1008
#define AR0543_FINE_INTG_MIN_DEF		0x4FE
#define AR0543_FINE_INTG_TIME_MAX		0x100A
#define AR0543_FINE_INTG_MAX_DEF		0x3EE

#define AR0543_READ_MODE				0x3040
#define AR0543_READ_MODE_X_ODD_INC		(BIT(6) | BIT(7) | BIT(8))
#define AR0543_READ_MODE_Y_ODD_INC		(BIT(0) | BIT(1) | BIT(2) |\
						BIT(3) | BIT(4) | BIT(5))

#define AR0543_HORIZONTAL_START_H		0x0344
#define AR0543_VERTICAL_START_H		0x0346
#define AR0543_HORIZONTAL_END_H		0x0348
#define AR0543_VERTICAL_END_H			0x034a
#define AR0543_HORIZONTAL_OUTPUT_SIZE_H	0x034c
#define AR0543_VERTICAL_OUTPUT_SIZE_H		0x034e

#define AR0543_COARSE_INTEGRATION_TIME		0x3012
#define AR0543_FINE_INTEGRATION_TIME		0x3014
#define AR0543_ROW_SPEED			0x3016
#define AR0543_GLOBAL_GAIN			0x305e
#define AR0543_GLOBAL_GAIN_WR			0x1000
#define AR0543_TEST_PATTERN_MODE		0x3070
#define AR0543_VCM_SLEW_STEP			0x30F0
#define AR0543_VCM_SLEW_STEP_MAX		0x7
#define AR0543_VCM_SLEW_STEP_MASK		0x7
#define AR0543_VCM_CODE			0x30F2
#define AR0543_VCM_SLEW_TIME			0x30F4
#define AR0543_VCM_SLEW_TIME_MAX		0xffff
#define AR0543_VCM_ENABLE			0x8000

/* ar0543 SCCB */
#define AR0543_SCCB_CTRL			0x3100
#define AR0543_AEC_PK_EXPO_H			0x3500
#define AR0543_AEC_PK_EXPO_M			0x3501
#define AR0543_AEC_PK_EXPO_L			0x3502
#define AR0543_AEC_MANUAL_CTRL			0x3503
#define AR0543_AGC_ADJ_H			0x3508
#define AR0543_AGC_ADJ_L			0x3509

#define AR0543_FOCAL_LENGTH_NUM	439	/*4.39mm*/
#define AR0543_FOCAL_LENGTH_DEM	100
#define AR0543_F_NUMBER_DEFAULT_NUM	24
#define AR0543_F_NUMBER_DEM	10

#define AR0543_X_ADDR_MIN	0X1180
#define AR0543_Y_ADDR_MIN	0X1182
#define AR0543_X_ADDR_MAX	0X1184
#define AR0543_Y_ADDR_MAX	0X1186

#define AR0543_MIN_FRAME_LENGTH_LINES	0x1140
#define AR0543_MAX_FRAME_LENGTH_LINES	0x1142
#define AR0543_MIN_LINE_LENGTH_PCK	0x1144
#define AR0543_MAX_LINE_LENGTH_PCK	0x1146
#define AR0543_MIN_LINE_BLANKING_PCK	0x1148
#define AR0543_MIN_FRAME_BLANKING_LINES 0x114A
#define AR0543_X_OUTPUT_SIZE	0x034C
#define AR0543_Y_OUTPUT_SIZE	0x034E

#define AR0543_BIN_FACTOR_MAX			3

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define AR0543_FOCAL_LENGTH_DEFAULT 0x1B70064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define AR0543_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define AR0543_F_NUMBER_RANGE 0x180a180a

/* Defines for register writes and register array processing */
#define AR0543_BYTE_MAX	30
#define AR0543_SHORT_MAX	16
#define I2C_RETRY_COUNT		5
#define AR0543_TOK_MASK	0xfff0

#define	AR0543_STATUS_POWER_DOWN	0x0
#define	AR0543_STATUS_STANDBY		0x2
#define	AR0543_STATUS_ACTIVE		0x3
#define	AR0543_STATUS_VIEWFINDER	0x4

//ASUS_BSP Wesley, for vcm test
#define VCM_ADDR           0x0c
#define VCM_CODE_MSB       0x03
#define VCM_CODE_LSB       0x04
#define VCM_MAX_FOCUS_POS  1023

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

enum ar0543_tok_type {
	AR0543_8BIT  = 0x0001,
	AR0543_16BIT = 0x0002,
	AR0543_RMW   = 0x0010,
	AR0543_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	AR0543_TOK_DELAY  = 0xfe00	/* delay token for reg list */
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
 * struct ar0543_fwreg - Firmware burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @val: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct ar0543_fwreg {
	enum ar0543_tok_type type; /* value, register or FW burst string */
	u16 addr;	/* target address */
	u32 val[8];
};

/**
 * struct ar0543_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct ar0543_reg {
	enum ar0543_tok_type type;
	union {
		u16 sreg;
		struct ar0543_fwreg *fwreg;
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

struct ar0543_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

struct ar0543_resolution {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	bool used;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	const struct ar0543_reg *regs;
	u8 bin_factor_x;
	u8 bin_factor_y;
	unsigned short skip_frames;
};

struct ar0543_format {
	u8 *desc;
	u32 pixelformat;
	struct s_register_setting *regs;
};

struct ar0543_af_data {
	u16 af_inf_pos;
	u16 af_1m_pos;
	u16 af_10cm_pos;
	u16 af_start_curr;
	u8 module_id;
	u8 vendor_id;
};

#define AR0543_FUSEID_SIZE		8
#define AR0543_FUSEID_NUM		4
#define AR0543_FUSEID_1			0x31f4
#define AR0543_FUSEID_2			0x31f6
#define AR0543_FUSEID_3			0x31f8
#define AR0543_FUSEID_4			0x31fa

/* ar0543 device structure */
struct ar0543_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int fmt_idx;
	int status;
	int streaming;
	int power;
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
	struct ar0543_af_data af_data;
	void *fuseid;
	/* Older VCMs could not maintain the focus position in standby mode. */
	bool keeps_focus_pos;
    struct attribute_group sensor_i2c_attribute; //Add for ATD read camera status+++
};

#define AR0543_MAX_WRITE_BUF_SIZE	32
struct ar0543_write_buffer {
	u16 addr;
	u8 data[AR0543_MAX_WRITE_BUF_SIZE];
};

struct ar0543_write_ctrl {
	int index;
	struct ar0543_write_buffer buffer;
};


#define AR0543_OTP_READ_EN               0x301A
#define AR0543_OTP_TIMING                0x3134
#define AR0543_OTP_RECORD_TYPE           0x304C
#define AR0543_OTP_AUTO_READ             0x304A
#define AR0543_OTP_READY_REG_DONE       (1 << 5)
#define AR0543_OTP_READY_REG_OK         (1 << 6)

#define AR0543_OTP_AF_INF_POS           0x3800
#define AR0543_OTP_AF_1M_POS            0x3802
#define AR0543_OTP_AF_10CM_POS          0x3804
#define AR0543_OTP_AF_START_CURR        0x3806
#define AR0543_OTP_AF_MODULE_ID         0x3808
#define AR0543_OTP_AF_VENDOR_ID         0x3809

#define AR0543_OTP_DATA_SIZE		10

#define GROUPED_PARAMETER_HOLD_ENABLE	{AR0543_8BIT, {0x0104}, 0x1}

#define GROUPED_PARAMETER_HOLD_DISABLE	{AR0543_8BIT, {0x0104}, 0x0}

#define INIT_VCM_CONTROL {AR0543_16BIT, {0x30F0}, 0x800C} /* slew_rate[2:0] */
static const struct ar0543_reg ar0543_init_vcm[] = {
	INIT_VCM_CONTROL,				   /* VCM_CONTROL */
	{AR0543_16BIT, {0x30F2}, 0x0000}, /* VCM_NEW_CODE */
	{AR0543_16BIT, {0x30F4}, 0x0080}, /* VCM_STEP_TIME */
	{AR0543_TOK_TERM, {0}, 0}
};

//#define RESET_REGISTER	{AR0543_16BIT, {0x301A}, 0x4A38}
static const struct ar0543_reg ar0543_reset_register[] = {
	{AR0543_8BIT,{0x0103},0x01},    //SOFTWARE_RESET (clears itself)
	{AR0543_TOK_DELAY, {0}, 50},     //DELAY=50       //Initialization Time

	//stop_streaming
	{AR0543_8BIT,{0x0100},0x00},// MODE_SELECT
	//dual_lane_MIPI_interface
	{AR0543_16BIT,{0x301A},0x0018},// RESET_REGISTER
	{AR0543_16BIT,{0x3064},0xB800},// SMIA_TEST
	{AR0543_16BIT,{0x31AE},0x0202},// SERIAL_FORMAT
	{AR0543_16BIT,{0x0112},0x0A0A},// 10bit raw output

	//REV1_recommended_settings
	{AR0543_16BIT,{0x316A},0x8400},// DAC_FBIAS
	{AR0543_16BIT,{0x316C},0x8400},// DAC_TXLO
	{AR0543_16BIT,{0x316E},0x8400},// DAC_ECL
	{AR0543_16BIT,{0x3EFA},0x1A1F},// DAC_LD_ECL
	{AR0543_16BIT,{0x3ED2},0xD965},// DAC_LD_6_7
	{AR0543_16BIT,{0x3ED8},0x7F1B},// DAC_LD_12_13
	{AR0543_16BIT,{0x3EDA},0x2F11},// DAC_LD_14_15
	{AR0543_16BIT,{0x3EE2},0x0060},// DAC_LD_22_23
	{AR0543_16BIT,{0x3EF2},0xD965},// DAC_LP_6_7
	{AR0543_16BIT,{0x3EF8},0x797F},// DAC_LD_TXHI
	{AR0543_16BIT,{0x3EFC},0x286F},// DAC_LD_FBIAS
	{AR0543_16BIT,{0x3EFE},0x2C01},// DAC_LD_TXLO

	//REV1_pixel_timing
	{AR0543_16BIT,{0x3E00},0x042F},// DYNAMIC_SEQRAM_00
	{AR0543_16BIT,{0x3E02},0xFFFF},// DYNAMIC_SEQRAM_02
	{AR0543_16BIT,{0x3E04},0xFFFF},// DYNAMIC_SEQRAM_04
	{AR0543_16BIT,{0x3E06},0xFFFF},// DYNAMIC_SEQRAM_06
	{AR0543_16BIT,{0x3E08},0x8071},// DYNAMIC_SEQRAM_08
	{AR0543_16BIT,{0x3E0A},0x7281},// DYNAMIC_SEQRAM_0A
	{AR0543_16BIT,{0x3E0C},0x4011},// DYNAMIC_SEQRAM_0C
	{AR0543_16BIT,{0x3E0E},0x8010},// DYNAMIC_SEQRAM_0E
	{AR0543_16BIT,{0x3E10},0x60A5},// DYNAMIC_SEQRAM_10
	{AR0543_16BIT,{0x3E12},0x4080},// DYNAMIC_SEQRAM_12
	{AR0543_16BIT,{0x3E14},0x4180},// DYNAMIC_SEQRAM_14
	{AR0543_16BIT,{0x3E16},0x0018},// DYNAMIC_SEQRAM_16
	{AR0543_16BIT,{0x3E18},0x46B7},// DYNAMIC_SEQRAM_18
	{AR0543_16BIT,{0x3E1A},0x4994},// DYNAMIC_SEQRAM_1A
	{AR0543_16BIT,{0x3E1C},0x4997},// DYNAMIC_SEQRAM_1C
	{AR0543_16BIT,{0x3E1E},0x4682},// DYNAMIC_SEQRAM_1E
	{AR0543_16BIT,{0x3E20},0x0018},// DYNAMIC_SEQRAM_20
	{AR0543_16BIT,{0x3E22},0x4241},// DYNAMIC_SEQRAM_22
	{AR0543_16BIT,{0x3E24},0x8000},// DYNAMIC_SEQRAM_24
	{AR0543_16BIT,{0x3E26},0x1880},// DYNAMIC_SEQRAM_26
	{AR0543_16BIT,{0x3E28},0x4785},// DYNAMIC_SEQRAM_28
	{AR0543_16BIT,{0x3E2A},0x4992},// DYNAMIC_SEQRAM_2A
	{AR0543_16BIT,{0x3E2C},0x4997},// DYNAMIC_SEQRAM_2C
	{AR0543_16BIT,{0x3E2E},0x4780},// DYNAMIC_SEQRAM_2E
	{AR0543_16BIT,{0x3E30},0x4D80},// DYNAMIC_SEQRAM_30
	{AR0543_16BIT,{0x3E32},0x100C},// DYNAMIC_SEQRAM_32
	{AR0543_16BIT,{0x3E34},0x8000},// DYNAMIC_SEQRAM_34
	{AR0543_16BIT,{0x3E36},0x184A},// DYNAMIC_SEQRAM_36
	{AR0543_16BIT,{0x3E38},0x8042},// DYNAMIC_SEQRAM_38
	{AR0543_16BIT,{0x3E3A},0x001A},// DYNAMIC_SEQRAM_3A
	{AR0543_16BIT,{0x3E3C},0x9610},// DYNAMIC_SEQRAM_3C
	{AR0543_16BIT,{0x3E3E},0x0C80},// DYNAMIC_SEQRAM_3E
	{AR0543_16BIT,{0x3E40},0x4DC6},// DYNAMIC_SEQRAM_40
	{AR0543_16BIT,{0x3E42},0x4A80},// DYNAMIC_SEQRAM_42
	{AR0543_16BIT,{0x3E44},0x0018},// DYNAMIC_SEQRAM_44
	{AR0543_16BIT,{0x3E46},0x8042},// DYNAMIC_SEQRAM_46
	{AR0543_16BIT,{0x3E48},0x8041},// DYNAMIC_SEQRAM_48
	{AR0543_16BIT,{0x3E4A},0x0018},// DYNAMIC_SEQRAM_4A
	{AR0543_16BIT,{0x3E4C},0x804B},// DYNAMIC_SEQRAM_4C
	{AR0543_16BIT,{0x3E4E},0xB74B},// DYNAMIC_SEQRAM_4E
	{AR0543_16BIT,{0x3E50},0x8010},// DYNAMIC_SEQRAM_50
	{AR0543_16BIT,{0x3E52},0x6056},// DYNAMIC_SEQRAM_52
	{AR0543_16BIT,{0x3E54},0x001C},// DYNAMIC_SEQRAM_54
	{AR0543_16BIT,{0x3E56},0x8211},// DYNAMIC_SEQRAM_56
	{AR0543_16BIT,{0x3E58},0x8056},// DYNAMIC_SEQRAM_58
	{AR0543_16BIT,{0x3E5A},0x827C},// DYNAMIC_SEQRAM_5A
	{AR0543_16BIT,{0x3E5C},0x0970},// DYNAMIC_SEQRAM_5C
	{AR0543_16BIT,{0x3E5E},0x8082},// DYNAMIC_SEQRAM_5E
	{AR0543_16BIT,{0x3E60},0x7281},// DYNAMIC_SEQRAM_60
	{AR0543_16BIT,{0x3E62},0x4C40},// DYNAMIC_SEQRAM_62
	{AR0543_16BIT,{0x3E64},0x8E4D},// DYNAMIC_SEQRAM_64
	{AR0543_16BIT,{0x3E66},0x8110},// DYNAMIC_SEQRAM_66
	{AR0543_16BIT,{0x3E68},0x0CAF},// DYNAMIC_SEQRAM_68
	{AR0543_16BIT,{0x3E6A},0x4D80},// DYNAMIC_SEQRAM_6A
	{AR0543_16BIT,{0x3E6C},0x100C},// DYNAMIC_SEQRAM_6C
	{AR0543_16BIT,{0x3E6E},0x8440},// DYNAMIC_SEQRAM_6E
	{AR0543_16BIT,{0x3E70},0x4C81},// DYNAMIC_SEQRAM_70
	{AR0543_16BIT,{0x3E72},0x7C5F},// DYNAMIC_SEQRAM_72
	{AR0543_16BIT,{0x3E74},0x7000},// DYNAMIC_SEQRAM_74
	{AR0543_16BIT,{0x3E76},0x0000},// DYNAMIC_SEQRAM_76
	{AR0543_16BIT,{0x3E78},0x0000},// DYNAMIC_SEQRAM_78
	{AR0543_16BIT,{0x3E7A},0x0000},// DYNAMIC_SEQRAM_7A
	{AR0543_16BIT,{0x3E7C},0x0000},// DYNAMIC_SEQRAM_7C
	{AR0543_16BIT,{0x3E7E},0x0000},// DYNAMIC_SEQRAM_7E
	{AR0543_16BIT,{0x3E80},0x0000},// DYNAMIC_SEQRAM_80
	{AR0543_16BIT,{0x3E82},0x0000},// DYNAMIC_SEQRAM_82
	{AR0543_16BIT,{0x3E84},0x0000},// DYNAMIC_SEQRAM_84
	{AR0543_16BIT,{0x3E86},0x0000},// DYNAMIC_SEQRAM_86
	{AR0543_16BIT,{0x3E88},0x0000},// DYNAMIC_SEQRAM_88
	{AR0543_16BIT,{0x3E8A},0x0000},// DYNAMIC_SEQRAM_8A
	{AR0543_16BIT,{0x3E8C},0x0000},// DYNAMIC_SEQRAM_8C
	{AR0543_16BIT,{0x3E8E},0x0000},// DYNAMIC_SEQRAM_8E
	{AR0543_16BIT,{0x3E90},0x0000},// DYNAMIC_SEQRAM_90
	{AR0543_16BIT,{0x3E92},0x0000},// DYNAMIC_SEQRAM_92
	{AR0543_16BIT,{0x3E94},0x0000},// DYNAMIC_SEQRAM_94
	{AR0543_16BIT,{0x3E96},0x0000},// DYNAMIC_SEQRAM_96
	{AR0543_16BIT,{0x3E98},0x0000},// DYNAMIC_SEQRAM_98
	{AR0543_16BIT,{0x3E9A},0x0000},// DYNAMIC_SEQRAM_9A
	{AR0543_16BIT,{0x3E9C},0x0000},// DYNAMIC_SEQRAM_9C
	{AR0543_16BIT,{0x3E9E},0x0000},// DYNAMIC_SEQRAM_9E
	{AR0543_16BIT,{0x3EA0},0x0000},// DYNAMIC_SEQRAM_A0
	{AR0543_16BIT,{0x3EA2},0x0000},// DYNAMIC_SEQRAM_A2
	{AR0543_16BIT,{0x3EA4},0x0000},// DYNAMIC_SEQRAM_A4
	{AR0543_16BIT,{0x3EA6},0x0000},// DYNAMIC_SEQRAM_A6
	{AR0543_16BIT,{0x3EA8},0x0000},// DYNAMIC_SEQRAM_A8
	{AR0543_16BIT,{0x3EAA},0x0000},// DYNAMIC_SEQRAM_AA
	{AR0543_16BIT,{0x3EAC},0x0000},// DYNAMIC_SEQRAM_AC
	{AR0543_16BIT,{0x3EAE},0x0000},// DYNAMIC_SEQRAM_AE
	{AR0543_16BIT,{0x3EB0},0x0000},// DYNAMIC_SEQRAM_B0
	{AR0543_16BIT,{0x3EB2},0x0000},// DYNAMIC_SEQRAM_B2
	{AR0543_16BIT,{0x3EB4},0x0000},// DYNAMIC_SEQRAM_B4
	{AR0543_16BIT,{0x3EB6},0x0000},// DYNAMIC_SEQRAM_B6
	{AR0543_16BIT,{0x3EB8},0x0000},// DYNAMIC_SEQRAM_B8
	{AR0543_16BIT,{0x3EBA},0x0000},// DYNAMIC_SEQRAM_BA
	{AR0543_16BIT,{0x3EBC},0x0000},// DYNAMIC_SEQRAM_BC
	{AR0543_16BIT,{0x3EBE},0x0000},// DYNAMIC_SEQRAM_BE
	{AR0543_16BIT,{0x3EC0},0x0000},// DYNAMIC_SEQRAM_C0
	{AR0543_16BIT,{0x3EC2},0x0000},// DYNAMIC_SEQRAM_C2
	{AR0543_16BIT,{0x3EC4},0x0000},// DYNAMIC_SEQRAM_C4
	{AR0543_16BIT,{0x3EC6},0x0000},// DYNAMIC_SEQRAM_C6
	{AR0543_16BIT,{0x3EC8},0x0000},// DYNAMIC_SEQRAM_C8
	{AR0543_16BIT,{0x3ECA},0x0000},// DYNAMIC_SEQRAM_CA
	{AR0543_16BIT,{0x3170},0x2150},// ANALOG_CONTROL
	{AR0543_16BIT,{0x317A},0x0150},// ANALOG_CONTROL6
	{AR0543_16BIT,{0x3ECC},0x2200},// DAC_LD_0_1
	{AR0543_16BIT,{0x3174},0x0000},// ANALOG_CONTROL3
	{AR0543_16BIT,{0x3176},0x0000},// ANALOG_CONTROL4
	{AR0543_16BIT,{0x30BC},0x0384},// CALIB_GLOBAL
	{AR0543_16BIT,{0x30C0},0x1220},// CALIB_CONTROL
	{AR0543_16BIT,{0x30D4},0x9200},// COLUMN_CORRECTION
	{AR0543_16BIT,{0x30B2},0xC000},// CALIB_TIED_OFFSET

	{AR0543_16BIT,{0x31B0},0x00C4},// FRAME_PREAMBLE
	{AR0543_16BIT,{0x31B2},0x0064},// LINE_PREAMBLE


	{AR0543_16BIT,{0x31B4},0x0E77},// MIPI_TIMING_0
	{AR0543_16BIT,{0x31B6},0x0D24},// MIPI_TIMING_1
	{AR0543_16BIT,{0x31B8},0x020E},// MIPI_TIMING_2
	{AR0543_16BIT,{0x31BA},0x0710},// MIPI_TIMING_3
	{AR0543_16BIT,{0x31BC},0x2A0D},// MIPI_TIMING_4
	{AR0543_16BIT,{0x31BE},0xC003},// MIPI_CONFIG_STATUS

	//updated June 2013--ADACD and 2DDC settings
	//ADACD: low gain
	{AR0543_16BIT,{0x3100},0x0002},// ADACD_CONTROL
	{AR0543_16BIT,{0x3102},0x0064},// ADACD_NOISE_MODEL1
	{AR0543_16BIT,{0x3104},0x0B6D},// ADACD_NOISE_MODEL2
	{AR0543_16BIT,{0x3106},0x0201},// ADACD_NOISE_FLOOR1
	{AR0543_16BIT,{0x3108},0x0905},// ADACD_NOISE_FLOOR2
	{AR0543_16BIT,{0x310A},0x002A},// ADACD_PEDESTAL
	//2DDC: low gain
	{AR0543_16BIT,{0x31E0},0x1F01},// PIX_DEF_ID
	{AR0543_16BIT,{0x3F02},0x0001},// PIX_DEF_2D_DDC_THRESH_HI3
	{AR0543_16BIT,{0x3F04},0x0032},// PIX_DEF_2D_DDC_THRESH_LO3
	{AR0543_16BIT,{0x3F06},0x015E},// PIX_DEF_2D_DDC_THRESH_HI4
	{AR0543_16BIT,{0x3F08},0x0190},// PIX_DEF_2D_DDC_THRESH_LO4

	{AR0543_16BIT,{0x305E},0x1127},// GLOBAL_GAIN

	{AR0543_16BIT,{0x3ECE},0x000A},// DAC_LD_2_3
	{AR0543_16BIT,{0x0400},0x0000},// SCALING_MODE
	{AR0543_16BIT,{0x0404},0x0010},// SCALE_M

	//PLL_Configuration
	{AR0543_16BIT,{0x0300},0x06},//vt_pix_clk_div = 0x6
	{AR0543_16BIT,{0x0302},0x01},//vt_sys_clk_div = 0x1
	{AR0543_16BIT,{0x0304},0x02},//pre_pll_clk_div = 0x2
	{AR0543_16BIT,{0x0306},0x46},//pll_multiplier = 0x46
	{AR0543_16BIT,{0x0308},0x0A},//op_pix_clk_div = 0xA
	{AR0543_16BIT,{0x030A},0x01},//op_sys_clk_div = 0x1
	{AR0543_TOK_DELAY, {0}, 5},//DELAY=5	

	//2592*1944 @15FPS
	{AR0543_16BIT,{0x0400}, 0x0000},	//scaling_mode
	{AR0543_16BIT,{0x0404}, 0x0010},	//scale_m
	{AR0543_16BIT,{0x034C}, 0x0A20},	//Output Width
	{AR0543_16BIT,{0x034E}, 0x0798},	//Output Height
	{AR0543_16BIT,{0x0344}, 0x0008},	//Column Start
	{AR0543_16BIT,{0x0346}, 0x0008},	//Row Start
	{AR0543_16BIT,{0x0348}, 0x0A27},	//Column End
	{AR0543_16BIT,{0x034A}, 0x079F},	//Row End
	{AR0543_16BIT,{0x3040}, 0x0041},	//Read Mode
	{AR0543_16BIT,{0x3010}, 0x00A0},	//Fine Correction
	{AR0543_16BIT,{0x3012}, 0x07E4},	//Coarse Integration Time
	{AR0543_16BIT,{0x3014}, 0x02CE},	//Fine Integration Time
	{AR0543_16BIT,{0x0340}, 0x07E5},	//Frame Lines
	{AR0543_16BIT,{0x0342}, 0x0E6E},	//Line Length

	{AR0543_8BIT,{0x0104}, 0x00 },	// GROUPED_PARAMETER_HOLD

	//=== End of Initial Setting ===
	{AR0543_TOK_TERM, {0}, 0}
};

static const struct ar0543_reg ar0543_soft_standby[] = {
	{AR0543_8BIT, {0x0100}, 0x00},
	{AR0543_TOK_TERM, {0}, 0}
};

static const struct ar0543_reg ar0543_streaming[] = {
	{AR0543_8BIT, {0x0100}, 0x01},
	{AR0543_TOK_TERM, {0}, 0}
};

static const struct ar0543_reg ar0543_param_hold[] = {
	{AR0543_8BIT, {0x0104}, 0x01},	/* GROUPED_PARAMETER_HOLD */
	{AR0543_TOK_TERM, {0}, 0}
};

static const struct ar0543_reg ar0543_param_update[] = {
	{AR0543_8BIT, {0x0104}, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{AR0543_TOK_TERM, {0}, 0}
};


static struct ar0543_reg const ar0543_2592x1944_15fps[] = {
	//[2592*1944 @15FPS]
	{AR0543_8BIT,{0x0104}, 0x01},   // GROUPED_PARAMETER_HOLD = 0x1

	// Timing Settings
	{AR0543_16BIT,{0x0400}, 0x0000},   // scaling_mode
	{AR0543_16BIT,{0x0404}, 0x0010},   // scale_m
	{AR0543_16BIT,{0x034C}, 0x0A20},   // Output Width
	{AR0543_16BIT,{0x034E}, 0x0798},   // Output Height
	{AR0543_16BIT,{0x0344}, 0x0008},   // Column Start
	{AR0543_16BIT,{0x0346}, 0x0008},   // Row Start
	{AR0543_16BIT,{0x0348}, 0x0A27},   // Column End
	{AR0543_16BIT,{0x034A}, 0x079F},   // Row End
	{AR0543_16BIT,{0x3040}, 0x0041},   // Read Mode
	{AR0543_16BIT,{0x3010}, 0x00A0},   // Fine Correction
	{AR0543_16BIT,{0x3012}, 0x07E4},   // Coarse Integration Time
	{AR0543_16BIT,{0x3014}, 0x02CE},   // Fine Integration Time
	{AR0543_16BIT,{0x0340}, 0x07E5},   // Frame Lines
	{AR0543_16BIT,{0x0342}, 0x0E6E},   // Line Length

	{AR0543_8BIT,{0x0104}, 0x00},   // GROUPED_PARAMETER_HOLD
	{AR0543_TOK_DELAY, {0}, 5},//DELAY=5
	{AR0543_TOK_TERM, {0}, 0}
};

static struct ar0543_reg const ar0543_1080p_30fps[] = {
	//[1936*1096 @30FPS]
	{AR0543_8BIT,{0x0104}, 0x01},   // GROUPED_PARAMETER_HOLD = 0x1

	// Timing Settings
	{AR0543_16BIT,{0x0400}, 0x0000},   // scaling_mode
	{AR0543_16BIT,{0x0404}, 0x0010},   // scale_m
	{AR0543_16BIT,{0x034C}, 0x0790},   // Output Width
	{AR0543_16BIT,{0x034E}, 0x0448},   // Output Height
	{AR0543_16BIT,{0x0344}, 0x0008},   // Column Start
	{AR0543_16BIT,{0x0346}, 0x0008},   // Row Start
	{AR0543_16BIT,{0x0348}, 0x0797},   // Column End
	{AR0543_16BIT,{0x034A}, 0x044F},   // Row End
	{AR0543_16BIT,{0x3040}, 0x0041},   // Read Mode
	{AR0543_16BIT,{0x3010}, 0x00A0},   // Fine Correction
	{AR0543_16BIT,{0x3012}, 0x0494},   // Coarse Integration Time
	{AR0543_16BIT,{0x3014}, 0x02CE},   // Fine Integration Time
	{AR0543_16BIT,{0x0340}, 0x0495},   // Frame Lines
	{AR0543_16BIT,{0x0342}, 0x0C6E},   // Line Length

	{AR0543_8BIT,{0x0104}, 0x00},   // GROUPED_PARAMETER_HOLD
	{AR0543_TOK_DELAY, {0}, 5},//DELAY=5
	{AR0543_TOK_TERM, {0}, 0}
};

static struct ar0543_reg const ar0543_960p_30fps[] = {
	//[1296*976 @30FPS]
	{AR0543_8BIT,{0x0104}, 0x01},   // GROUPED_PARAMETER_HOLD = 0x1

	// Timing Settings
	{AR0543_16BIT,{0x0400}, 0x0000},   // scaling_mode
	{AR0543_16BIT,{0x0404}, 0x0010},   // scale_m
	{AR0543_16BIT,{0x034C}, 0x0510},   // Output Width
	{AR0543_16BIT,{0x034E}, 0x03D0},   // Output Height
	{AR0543_16BIT,{0x0344}, 0x0008},   // Column Start
	{AR0543_16BIT,{0x0346}, 0x0008},   // Row Start
	{AR0543_16BIT,{0x0348}, 0x0A25},   // Column End
	{AR0543_16BIT,{0x034A}, 0x07A5},   // Row End
	{AR0543_16BIT,{0x3040}, 0x04C3},   // Read Mode
	{AR0543_16BIT,{0x3010}, 0x0184},   // Fine Correction
	{AR0543_16BIT,{0x3012}, 0x0418},   // Coarse Integration Time
	{AR0543_16BIT,{0x3014}, 0x05F8},   // Fine Integration Time
	{AR0543_16BIT,{0x0340}, 0x0419},   // Frame Lines
	{AR0543_16BIT,{0x0342}, 0x0DE6},   // Line Length

	{AR0543_8BIT,{0x0104}, 0x00},   // GROUPED_PARAMETER_HOLD
	{AR0543_TOK_DELAY, {0}, 5},//DELAY=5
	{AR0543_TOK_TERM, {0}, 0}
};


static struct ar0543_reg const ar0543_720p_30fps[] = {
	//[1296*736 @30FPS]
	{AR0543_8BIT,{0x0104}, 0x01},   // GROUPED_PARAMETER_HOLD = 0x1

	// Timing Settings
	{AR0543_16BIT,{0x0400}, 0x0000},   // scaling_mode
	{AR0543_16BIT,{0x0404}, 0x0010},   // scale_m
	{AR0543_16BIT,{0x034C}, 0x0510},   // Output Width
	{AR0543_16BIT,{0x034E}, 0x02E0},   // Output Height
	{AR0543_16BIT,{0x0344}, 0x0008},   // Column Start
	{AR0543_16BIT,{0x0346}, 0x0008},   // Row Start
	{AR0543_16BIT,{0x0348}, 0x0A25},   // Column End
	{AR0543_16BIT,{0x034A}, 0x05C5},   // Row End
	{AR0543_16BIT,{0x3040}, 0x04C3},   // Read Mode
	{AR0543_16BIT,{0x3010}, 0x0184},   // Fine Correction
	{AR0543_16BIT,{0x3012}, 0x0328},   // Coarse Integration Time
	{AR0543_16BIT,{0x3014}, 0x05F8},   // Fine Integration Time
	{AR0543_16BIT,{0x0340}, 0x0329},   // Frame Lines
	{AR0543_16BIT,{0x0342}, 0x1206},   // Line Length

	{AR0543_8BIT,{0x0104}, 0x00},   // GROUPED_PARAMETER_HOLD
	{AR0543_TOK_DELAY, {0}, 5},//DELAY=5
	{AR0543_TOK_TERM, {0}, 0}
};


static struct ar0543_reg const ar0543_VGA_30fps[] = {
	//[656*496 @30FPS] },   // 668*500 that bining from 2592*1944 and scale from 1296*972
	{AR0543_8BIT,{0x0104}, 0x01},   // GROUPED_PARAMETER_HOLD = 0x1

	// Timing Settings
	{AR0543_16BIT,{0x0400}, 0x0002},   // scaling_mode
	{AR0543_16BIT,{0x0404}, 0x001F},   // scale_m
	{AR0543_16BIT,{0x034C}, 0x029C},   // Output Width
	{AR0543_16BIT,{0x034E}, 0x01F4},   // Output Height
	{AR0543_16BIT,{0x0344}, 0x0008},   // Column Start
	{AR0543_16BIT,{0x0346}, 0x0008},   // Row Start
	{AR0543_16BIT,{0x0348}, 0x0A25},   // Column End
	{AR0543_16BIT,{0x034A}, 0x079D},   // Row End
	{AR0543_16BIT,{0x3040}, 0x04C3},   // Read Mode
	{AR0543_16BIT,{0x3010}, 0x0184},   // Fine Correction
	{AR0543_16BIT,{0x3012}, 0x0414},   // Coarse Integration Time
	{AR0543_16BIT,{0x3014}, 0x05F8},   // Fine Integration Time
	{AR0543_16BIT,{0x0340}, 0x0415},   // Frame Lines
	{AR0543_16BIT,{0x0342}, 0x0DF4},   // Line Length

	{AR0543_8BIT,{0x0104}, 0x00},   // GROUPED_PARAMETER_HOLD
	{AR0543_TOK_DELAY, {0}, 5},//DELAY=5
	{AR0543_TOK_TERM, {0}, 0}
};

static struct ar0543_reg const ar0543_CIF_30fps[] = {
	//[368*304  @30FPS]},   // 432*324 that bining from 2592*1944 and scale from 1296*972
	{AR0543_8BIT,{0x0104}, 0x01},   // GROUPED_PARAMETER_HOLD = 0x1

	// Timing Settings
	{AR0543_16BIT,{0x0400}, 0x0002},   // scaling_mode
	{AR0543_16BIT,{0x0404}, 0x0030},   // scale_m
	{AR0543_16BIT,{0x034C}, 0x01B0},   // Output Width
	{AR0543_16BIT,{0x034E}, 0x0144},   // Output Height
	{AR0543_16BIT,{0x0344}, 0x0008},   // Column Start
	{AR0543_16BIT,{0x0346}, 0x0008},   // Row Start
	{AR0543_16BIT,{0x0348}, 0x0A25},   // Column End
	{AR0543_16BIT,{0x034A}, 0x079D},   // Row End
	{AR0543_16BIT,{0x3040}, 0x04C3},   // Read Mode
	{AR0543_16BIT,{0x3010}, 0x0184},   // Fine Correction
	{AR0543_16BIT,{0x3012}, 0x0414},   // Coarse Integration Time
	{AR0543_16BIT,{0x3014}, 0x05F8},   // Fine Integration Time
	{AR0543_16BIT,{0x0340}, 0x0415},   // Frame Lines
	{AR0543_16BIT,{0x0342}, 0x0DF4},   // Line Length

	{AR0543_8BIT,{0x0104}, 0x00},   // GROUPED_PARAMETER_HOLD
	{AR0543_TOK_DELAY, {0}, 5},//DELAY=5
	{AR0543_TOK_TERM, {0}, 0}
};

static struct ar0543_resolution ar0543_res_preview[] = {
	{
		 .desc =	"PREVIEW_960p_30fps"	,
		 .width =	1296	,
		 .height =	976	,
		 .fps =		30	,
		 .used =	0	,
		 .pixels_per_line = 0x0DE6, /* consistent with regs arrays */
		 .lines_per_frame = 0x0419, /* consistent with regs arrays */
		 .regs =	ar0543_960p_30fps	,
		 .bin_factor_x =	1,
		 .bin_factor_y =	1,
		 .skip_frames = 1, /*change skip num from 1 to 0 after 3A init
				    param invalid issue fixed*/
	},
	{
		 // For 2560x1920 output
		 .desc =	"PREVIEW_2592x1944_15fps"	,
		 .width =	2592	,
		 .height =	1944	,
		 .fps =		15	,
		 .used =	0	,
		 .pixels_per_line = 0x0E6E, /* consistent with regs arrays */
		 .lines_per_frame = 0x07E5, /* consistent with regs arrays */
		 .regs =	ar0543_2592x1944_15fps,
		 .bin_factor_x =	0,
		 .bin_factor_y =	0,
		 .skip_frames = 1,
	},
};

#define N_RES_PREVIEW (ARRAY_SIZE(ar0543_res_preview))

static struct ar0543_resolution ar0543_res_still[] = {
	{
		 // For 2560x1920 output
		 .desc =	"STILL_2592x1944_15fps"	,
		 .width =	2592	,
		 .height =	1944	,
		 .fps =		15	,
		 .used =	0	,
		 .pixels_per_line = 0x0E6E, /* consistent with regs arrays */
		 .lines_per_frame = 0x07E5, /* consistent with regs arrays */
		 .regs =	ar0543_2592x1944_15fps,
		 .bin_factor_x =	0,
		 .bin_factor_y =	0,
		 .skip_frames = 1,
	},
};

#define N_RES_STILL (ARRAY_SIZE(ar0543_res_still))

static struct ar0543_resolution ar0543_res_video[] = {
	{
		 .desc =	 "VIDEO_CIF_30fps"  ,
		 .width =	 368 ,
		 .height =  304 ,
		 .fps =	 30  ,
		 .used =	 0	 ,
		 .pixels_per_line = 0x0DF4, /* consistent with regs arrays */
		 .lines_per_frame = 0x0415, /* consistent with regs arrays */
		 .regs =	 ar0543_CIF_30fps	 ,
		 .bin_factor_x =	 2,
		 .bin_factor_y =	 2,
		 .skip_frames = 1,
	},
	{
		 .desc =	 "VIDEO_VGA_30fps"	 ,
		 .width =	 656 ,
		 .height =  496 ,
		 .fps =	 30  ,
		 .used =	 0	 ,
		 .pixels_per_line = 0x0DF4, /* consistent with regs arrays */
		 .lines_per_frame = 0x0415, /* consistent with regs arrays */
		 .regs =	 ar0543_VGA_30fps	 ,
		 .bin_factor_x =	 2,
		 .bin_factor_y =	 2,
		 .skip_frames = 1,
	},
	{
		 .desc =	 "VIDEO_720p_30fps"  ,
		 .width =	 1296	 ,
		 .height =  736 ,
		 .fps =	 30  ,
		 .used =	 0	 ,
		 .pixels_per_line = 0x1206, /* consistent with regs arrays */
		 .lines_per_frame = 0x0329, /* consistent with regs arrays */
		 .regs =	 ar0543_720p_30fps	 ,
		 .bin_factor_x =	 1,
		 .bin_factor_y =	 1,
		 .skip_frames = 1,
	},
	{
		 .desc =	  "VIDEO_960p_30fps"  ,
		 .width =   1296	  ,
		 .height =  976 ,
		 .fps =	  30  ,
		 .used =	  0   ,
		 .pixels_per_line = 0x0DE6, /* consistent with regs arrays */
		 .lines_per_frame = 0x0419, /* consistent with regs arrays */
		 .regs =	  ar0543_960p_30fps   ,
		 .bin_factor_x =	  1,
		 .bin_factor_y =	  1,
		 .skip_frames = 1,
	},
	{
		 .desc =	  "VIDEO_1080p_30fps" ,
		 .width =   1936	  ,
		 .height =  1096	  ,
		 .fps =	  30  ,
		 .used =	  0   ,
		 .pixels_per_line = 0x0C6E, /* consistent with regs arrays */
		 .lines_per_frame = 0x0495, /* consistent with regs arrays */
		 .regs =	  ar0543_1080p_30fps  ,
		 .bin_factor_x =	  1,
		 .bin_factor_y =	  1,
		 .skip_frames = 1,
	},
};

#define N_RES_VIDEO (ARRAY_SIZE(ar0543_res_video))

static struct ar0543_resolution *ar0543_res = ar0543_res_preview;
static int N_RES = N_RES_PREVIEW;

#endif
