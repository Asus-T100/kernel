/*
 * Support for s5k8aay Camera Sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#ifndef __S5K8AAY_H__
#define __S5K8AAY_H__

#include <linux/atomisp.h>
#include <linux/atomisp_platform.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/media-entity.h>

#define V4L2_IDENT_S5K8AAY 8245

#define MT9P111_REV3
#define FULLINISUPPORT

/* #defines for register writes and register array processing */
#define MISENSOR_8BIT		1
#define MISENSOR_16BIT		2
#define MISENSOR_32BIT		4

#define MISENSOR_FWBURST0	0x80
#define MISENSOR_FWBURST1	0x81
#define MISENSOR_FWBURST4	0x84
#define MISENSOR_FWBURST	0x88

#define MISENSOR_TOK_TERM	0xf000	/* terminating token for reg list */
#define MISENSOR_TOK_DELAY	0xfe00	/* delay token for reg list */
#define MISENSOR_TOK_FWLOAD	0xfd00	/* token indicating load FW */
#define MISENSOR_TOK_POLL	0xfc00	/* token indicating poll instruction */
#define MISENSOR_TOK_MASK	0xfff0
#define MISENSOR_AWB_STEADY	(1<<0)	/* awb steady */
#define MISENSOR_AE_READY	(1<<3)	/* ae status ready */

/* mask to set sensor read_mode via misensor_rmw_reg */
#define MISENSOR_R_MODE_MASK	0x0330
/* mask to set sensor vert_flip and horz_mirror */
#define MISENSOR_VFLIP_MASK	0x0002
#define MISENSOR_HFLIP_MASK	0x0001
#define MISENSOR_FLIP_EN	1
#define MISENSOR_FLIP_DIS	0

/* bits set to set sensor read_mode via misensor_rmw_reg */
#define MISENSOR_SKIPPING_SET	0x0011
#define MISENSOR_SUMMING_SET	0x0033
#define MISENSOR_NORMAL_SET	0x0000

/* sensor register that control sensor read-mode and mirror */
#define MISENSOR_READ_MODE	0xC834
/* sensor ae-track status register */
#define MISENSOR_AE_TRACK_STATUS	0xA800
/* sensor awb status register */
#define MISENSOR_AWB_STATUS	0xAC00

#define SENSOR_DETECTED		1
#define SENSOR_NOT_DETECTED	0

#define I2C_RETRY_COUNT		5
#define MSG_LEN_OFFSET		2

#ifndef MIPI_CONTROL
#define MIPI_CONTROL		0x3400	/* MIPI_Control */
#endif

/* GPIO pin on Moorestown */
#define GPIO_SCLK_25		44
#define GPIO_STB_PIN		47

#define GPIO_STDBY_PIN		49   /* ab:new */
#define GPIO_RESET_PIN		50

#define S5K8AAY_CHIP_ID		0x00000040	/* Should be 0x00A0 or 0x00B0 */
#define S5K8AAY_CHIP_ID_VAL	0x08AA
#define S5K8AAY_ROM_REVISION	0x00000042

/* ulBPat; */

#define S5K8AAY_BPAT_RGRGGBGB	(1 << 0)
#define S5K8AAY_BPAT_GRGRBGBG	(1 << 1)
#define S5K8AAY_BPAT_GBGBRGRG	(1 << 2)
#define S5K8AAY_BPAT_BGBGGRGR	(1 << 3)

#define S5K8AAY_WAIT_STAT_TIMEOUT	100
#define AHB_MSB_ADDR_PTR		0xfcfc
#define GEN_REG_OFFSH			0xd000
#define GEN_ROM_REG_OFFSH		0x0000
#define REG_CMDWR_ADDRH			0x0028
#define REG_CMDWR_ADDRL			0x002a
#define REG_CMDRD_ADDRH			0x002c
#define REG_CMDRD_ADDRL			0x002e
#define REG_CMDBUF0_ADDR		0x0f12

#define S5K8AAY_FOCAL_LENGTH_NUM	208	/*2.08mm*/
#define S5K8AAY_FOCAL_LENGTH_DEM	100
#define S5K8AAY_F_NUMBER_DEFAULT_NUM	24
#define S5K8AAY_F_NUMBER_DEM	10
#define S5K8AAY_WAIT_STAT_TIMEOUT	100
#define S5K8AAY_FLICKER_MODE_50HZ	1
#define S5K8AAY_FLICKER_MODE_60HZ	2
/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define S5K8AAY_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define S5K8AAY_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define S5K8AAY_F_NUMBER_RANGE 0x180a180a

/* Supported resolutions */
enum {
	S5K8AAY_RES_QCIF,
	S5K8AAY_RES_QVGA,
	S5K8AAY_RES_VGA,
	S5K8AAY_RES_480P,
	S5K8AAY_RES_720P,
	S5K8AAY_RES_960P,
};
#define S5K8AAY_RES_960P_SIZE_H		1280
#define S5K8AAY_RES_960P_SIZE_V		960
#define S5K8AAY_RES_720P_SIZE_H		1280
#define S5K8AAY_RES_720P_SIZE_V		720
#define S5K8AAY_RES_480P_SIZE_H		768
#define S5K8AAY_RES_480P_SIZE_V		480
#define S5K8AAY_RES_VGA_SIZE_H		640
#define S5K8AAY_RES_VGA_SIZE_V		480
#define S5K8AAY_RES_QVGA_SIZE_H		320
#define S5K8AAY_RES_QVGA_SIZE_V		240
#define S5K8AAY_RES_QCIF_SIZE_H		176
#define S5K8AAY_RES_QCIF_SIZE_V		144

/* completion status polling requirements, usage based on Aptina .INI Rev2 */
enum poll_reg {
	NO_POLLING,
	PRE_POLLING,
	POST_POLLING,
};
/*
 * struct misensor_reg - MI sensor  register format
 * @length: length of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 * Define a structure for sensor register initialization values
 */
struct misensor_reg {
	u32 length;
	u32 reg;
	u32 val;	/* value or for read/mod/write, AND mask */
	u32 val2;	/* optional; for rmw, OR mask */
};

/*
 * struct misensor_fwreg - Firmware burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @valx: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct misensor_fwreg {
	u32	type;	/* type of value, register or FW burst string */
	u32	addr;	/* target address */
	u32	val0;
	u32	val1;
	u32	val2;
	u32	val3;
	u32	val4;
	u32	val5;
	u32	val6;
	u32	val7;
};

struct regval_list {
	u16 reg_num;
	u8 value;
};

struct s5k8aay_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int real_model_id;
	int nctx;
	int power;

	unsigned int bus_width;
	unsigned int mode;
	unsigned int field_inv;
	unsigned int field_sel;
	unsigned int ycseq;
	unsigned int conv422;
	unsigned int bpat;
	unsigned int hpol;
	unsigned int vpol;
	unsigned int edge;
	unsigned int bls;
	unsigned int gamma;
	unsigned int cconv;
	unsigned int res;
	unsigned int dwn_sz;
	unsigned int blc;
	unsigned int agc;
	unsigned int awb;
	unsigned int aec;
	/* extention SENSOR version 2 */
	unsigned int cie_profile;

	/* extention SENSOR version 3 */
	unsigned int flicker_freq;

	/* extension SENSOR version 4 */
	unsigned int smia_mode;
	unsigned int mipi_mode;

	/* Add name here to load shared library */
	unsigned int type;

	/*Number of MIPI lanes*/
	unsigned int mipi_lanes;
	char name[32];

	u8 lightfreq;
};

struct s5k8aay_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct s5k8aay_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	struct regval_list *regs;
};

struct s5k8aay_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/* 2 bytes used for address: 256 bytes total */
#define S5K8AAY_MAX_WRITE_BUF_SIZE	2
struct s5k8aay_write_buffer {
	u16 addr;
	u8 data[S5K8AAY_MAX_WRITE_BUF_SIZE];
};

struct s5k8aay_write_ctrl {
	int index;
	struct s5k8aay_write_buffer buffer;
};

/*
 * Modes supported by the s5k8aay driver.
 * Please, keep them in ascending order.
 */
static struct s5k8aay_res_struct s5k8aay_res[] = {
	{
	.desc	= "720p",
	.res	= S5K8AAY_RES_720P,
	.width	= 1280,
	.height	= 720,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
};

static const struct i2c_device_id s5k8aay_id[] = {
	{"s5k8aay", 0},
	{}
};

#endif
