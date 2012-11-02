/*
 * Support for mt9v113 Camera Sensor.
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

#ifndef __MT9V113_H__
#define __MT9V113_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>


#define V4L2_IDENT_MT9V113 0x2280

#define MT9V113_REG_CHIPID	0x0
#define MT9V113_REG_PLL_DIV	0x0010
#define MT9V113_REG_PLL_P	0x0012
#define MT9V113_REG_PLL_CTRL	0x0014
#define MT9V113_REG_STBY_CTRL	0x0018
#define MT9V113_REG_MISC_CTRL	0x001a

#define MT9V113_REG_OFIFO_CTRL	0x321c
#define MT9V113_REG_MIPI_CTRL	0x3400
#define MT9V113_REG_MIPI_STAT	0x3402

#define MIPI_STAT_BIT_MIPI_STBY_STAT	0
#define MIPI_STAT_MASK_MIPI_STBY_STAT	(1 << MIPI_STAT_BIT_MIPI_STBY_STAT)

#define MIPI_CTRL_BIT_MIPI_STBY_REQ	1
#define MIPI_CTRL_MASK_MIPI_STBY_REQ	(1 << MIPI_CTRL_BIT_MIPI_STBY_REQ)

#define MIPI_CTRL_BIT_MIPI_EOF_REQ	4
#define MIPI_CTRL_MASK_MIPI_EOF_REQ	(1 << MIPI_CTRL_BIT_MIPI_EOF_REQ)

#define STBY_CTRL_BIT_STBY_STAT	14
#define STBY_CTRL_MASK_STBY_STAT	(1 << STBY_CTRL_BIT_STBY_STAT)

#define STBY_CTRL_BIT_STBY_REQ	0
#define STBY_CTRL_MASK_STBY_REQ	(1 << STBY_CTRL_BIT_STBY_REQ)

#define PLL_CTRL_BIT_PLL_STAT	15
#define PLL_CTRL_MASK_PLL_STAT	(1 << PLL_CTRL_BIT_PLL_STAT)

#define PLL_CTRL_BIT_INIT_PLL	0
#define PLL_CTRL_MASK_INIT_PLL	(1 << PLL_CTRL_BIT_INIT_PLL)

#define PLL_CTRL_BIT_EN_PLL	1
#define PLL_CTRL_MASK_EN_PLL	(1 << PLL_CTRL_BIT_EN_PLL)

#define MIPI_CTRL_BIT_EN_MIPI	9
#define MIPI_CTRL_MASK_EN_MIPI	(1 << MIPI_CTRL_BIT_EN_MIPI)

#define OFIFO_CTRL_BIT_SENS_OUT	7
#define OFIFO_CTRL_MASK_SENS_OUT	(1 << OFIFO_CTRL_BIT_SENS_OUT)

#define MT9V113_MCU_VAR_ADDR	0x098c
#define MT9V113_MCU_VAR_DATA0	0x0990
#define MT9V113_MCU_VAR_DATA1	0x0992
#define MT9V113_MCU_VAR_DATA2	0x0994
#define MT9V113_MCU_VAR_DATA3	0x0996
#define MT9V113_MCU_VAR_DATA4	0x0998
#define MT9V113_MCU_VAR_DATA5	0x099a
#define MT9V113_MCU_VAR_DATA6	0x099c
#define MT9V113_MCU_VAR_DATA7	0x099e

#define MT9V113_VAR_SEQ_CMD	0xa103
#define SEQ_CMD_RUN		0x0
#define SEQ_CMD_REFRESH_MODE	0x0006
#define SEQ_CMD_REFRESH		0x0005
#define MT9V113_VAR_SEQ_STATE	0xa104

/* #defines for register writes and register array processing */
#define MISENSOR_8BIT		1
#define MISENSOR_16BIT		2
#define MISENSOR_32BIT		4

#define MISENSOR_TOK_TERM	0xf000	/* terminating token for reg list */
#define MISENSOR_TOK_DELAY	0xfe00	/* delay token for reg list */
#define MISENSOR_TOK_FWLOAD	0xfd00	/* token indicating load FW */
#define MISENSOR_TOK_POLL	0xfc00	/* token indicating poll instruction */
#define MISENSOR_TOK_RMW	0x0010  /* RMW operation */
#define MISENSOR_TOK_MASK	0xfff0
#define MISENSOR_AWB_STEADY	(1<<0)	/* awb steady */
#define MISENSOR_AE_READY	(1<<3)	/* ae status ready */

#define I2C_RETRY_COUNT		5
#define MSG_LEN_OFFSET		2
#define MAX_FMTS		1

/* Supported resolutions */
enum {
	MT9V113_RES_QCIF,
	MT9V113_RES_QVGA,
	MT9V113_RES_VGA,
};
#define MT9V113_RES_VGA_SIZE_H		640
#define MT9V113_RES_VGA_SIZE_V		480
#define MT9V113_RES_QVGA_SIZE_H		320
#define MT9V113_RES_QVGA_SIZE_V		240
#define MT9V113_RES_QCIF_SIZE_H		176
#define MT9V113_RES_QCIF_SIZE_V		144

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

struct regval_list {
	u16 reg_num;
	u8 value;
};

struct mt9v113_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int real_model_id;

	unsigned int res;
};

struct mt9v113_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct mt9v113_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	struct regval_list *regs;
};

struct mt9v113_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

#define MT9V113_MAX_WRITE_BUF_SIZE	32
struct mt9v113_write_buffer {
	u16 addr;
	u8 data[MT9V113_MAX_WRITE_BUF_SIZE];
};

struct mt9v113_write_ctrl {
	int index;
	struct mt9v113_write_buffer buffer;
};

/*
 * Modes supported by the mt9v113 driver.
 * Please, keep them in ascending order.
 */
static struct mt9v113_res_struct mt9v113_res[] = {
	{
	.desc	= "QCIF",
	.res	= MT9V113_RES_QCIF,
	.width	= 176,
	.height	= 144,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
	{
	.desc	= "QVGA",
	.res	= MT9V113_RES_QVGA,
	.width	= 320,
	.height	= 240,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
	{
	.desc	= "VGA",
	.res	= MT9V113_RES_VGA,
	.width	= 640,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 1,
	},
};
#define N_RES (ARRAY_SIZE(mt9v113_res))

static const struct i2c_device_id mt9v113_id[] = {
	{"mt9v113", 0},
	{}
};


/*
 * TBD
 * Optimize for two context config
 * Sensor pixel clock: 13.2 MHZ
 * hblank time: (852-184) * ( 1 / 13.2E6) = 50.6uS
 * vblank time: (516 - 152) * (852 / 13.2E6 ) = 23.4mS
 * frame time: 516 * (852 / 13.2E6) = 33.3mS

 */
static struct misensor_reg const mt9v113_qcif_init[] = {
	{MISENSOR_16BIT, 0x98c, 0x2703},/*Output Width (a)*/
	{MISENSOR_16BIT, 0x990, 0x00b0},/*      = 176*/
	{MISENSOR_16BIT, 0x98c, 0x2705},/*Output Height (a)*/
	{MISENSOR_16BIT, 0x990, 0x0090},/*      = 144*/
	{MISENSOR_16BIT, 0x98c, 0x2707},/*Output Width (b)*/
	{MISENSOR_16BIT, 0x990, 0x0280},/*      = 640*/
	{MISENSOR_16BIT, 0x98c, 0x2709},/*Output Height (b)*/
	{MISENSOR_16BIT, 0x990, 0x01e0},/*      = 480*/
	{MISENSOR_16BIT, 0x98c, 0x270d},/*Row Start (a)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x270f},/*column Start (a)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2711},/*Row end (a)*/
	{MISENSOR_16BIT, 0x990, 0x1e7},/*      = 487*/
	{MISENSOR_16BIT, 0x98c, 0x2713},/*column end (a)*/
	{MISENSOR_16BIT, 0x990, 0x287},/*      = 647*/
	{MISENSOR_16BIT, 0x98c, 0x2715},/*Row Speed (a)*/
	{MISENSOR_16BIT, 0x990, 0x0001},/*      = 1*/
	{MISENSOR_16BIT, 0x98c, 0x2717},/*Read Mode (a)*/
	{MISENSOR_16BIT, 0x990, 0x0026},/*      = 38*/
	{MISENSOR_16BIT, 0x98c, 0x2719},/*sensor_fine_correction (a)*/
	{MISENSOR_16BIT, 0x990, 0x001a},/*      = 26*/
	{MISENSOR_16BIT, 0x98c, 0x271b},/*sensor_fine_IT_min (a)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x271d},/*sensor_fine_IT_max_margin (a)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x271f},/*frame Lines (a)*/
	{MISENSOR_16BIT, 0x990, 0x0204},/*      = 516*/
	{MISENSOR_16BIT, 0x98c, 0x2721},/*Line Length (a)*/
	{MISENSOR_16BIT, 0x990, 0x0354},/*      = 852*/
	{MISENSOR_16BIT, 0x98c, 0x2723},/*Row Start (b)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2725},/*column Start (b)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2727},/*Row end (b)*/
	{MISENSOR_16BIT, 0x990, 0x1e7},/*      = 487*/
	{MISENSOR_16BIT, 0x98c, 0x2729},/*column end (b)*/
	{MISENSOR_16BIT, 0x990, 0x287},/*      = 647*/
	{MISENSOR_16BIT, 0x98c, 0x272b},/*Row Speed (b)*/
	{MISENSOR_16BIT, 0x990, 0x0001},/*      = 1*/
	{MISENSOR_16BIT, 0x98c, 0x272d},/*Read Mode (b)*/
	{MISENSOR_16BIT, 0x990, 0x0026},/*      = 38*/
	{MISENSOR_16BIT, 0x98c, 0x272f},/*sensor_fine_correction (b)*/
	{MISENSOR_16BIT, 0x990, 0x001a},/*      = 26*/
	{MISENSOR_16BIT, 0x98c, 0x2731},/*sensor_fine_IT_min (b)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x2733},/*sensor_fine_IT_max_margin (b)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x2735},/*frame Lines (b)*/
	{MISENSOR_16BIT, 0x990, 0x0204},/*      = 516*/
	{MISENSOR_16BIT, 0x98c, 0x2737},/*Line Length (b)*/
	{MISENSOR_16BIT, 0x990, 0x0354},/*      = 852*/
	{MISENSOR_16BIT, 0x98c, 0x2739},/*crop_X0 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x273b},/*crop_X1 (a)*/
	{MISENSOR_16BIT, 0x990, 0x027f},/*      = 639*/
	{MISENSOR_16BIT, 0x98c, 0x273d},/*crop_Y0 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x273f},/*crop_Y1 (a)*/
	{MISENSOR_16BIT, 0x990, 0x01df},/*      = 479*/
	{MISENSOR_16BIT, 0x98c, 0x2747},/*crop_X0 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2749},/*crop_X1 (b)*/
	{MISENSOR_16BIT, 0x990, 0x027f},/*      = 639*/
	{MISENSOR_16BIT, 0x98c, 0x274b},/*crop_Y0 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x274d},/*crop_Y1 (b)*/
	{MISENSOR_16BIT, 0x990, 0x01df},/*      = 479*/
	{MISENSOR_16BIT, 0x98c, 0x222d},/*R9 Step*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0xa408},/*search_f1_50*/
	{MISENSOR_16BIT, 0x990, 0x1f},/*      = 31*/
	{MISENSOR_16BIT, 0x98c, 0xa409},/*search_f2_50*/
	{MISENSOR_16BIT, 0x990, 0x21},/*      = 33*/
	{MISENSOR_16BIT, 0x98c, 0xa40a},/*search_f1_60*/
	{MISENSOR_16BIT, 0x990, 0x25},/*      = 37*/
	{MISENSOR_16BIT, 0x98c, 0xa40b},/*search_f2_60*/
	{MISENSOR_16BIT, 0x990, 0x27},/*      = 39*/
	{MISENSOR_16BIT, 0x98c, 0x2411},/*R9_Step_60 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0x2413},/*R9_Step_50 (a)*/
	{MISENSOR_16BIT, 0x990, 0x009b},/*      = 155*/
	{MISENSOR_16BIT, 0x98c, 0x2415},/*R9_Step_60 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0x2417},/*R9_Step_50 (b)*/
	{MISENSOR_16BIT, 0x990, 0x009b},/*      = 155*/
	{MISENSOR_16BIT, 0x98c, 0xa404},/*fd Mode*/
	{MISENSOR_16BIT, 0x990, 0x10},/*      = 16*/
	{MISENSOR_16BIT, 0x98c, 0xa40d},/*Stat_min*/
	{MISENSOR_16BIT, 0x990, 0x02},/*      = 2*/
	{MISENSOR_16BIT, 0x98c, 0xa40e},/*Stat_max*/
	{MISENSOR_16BIT, 0x990, 0x03},/*      = 3*/
	{MISENSOR_16BIT, 0x98c, 0xa410},/*Min_amplitude*/
	{MISENSOR_16BIT, 0x990, 0x0a},/*      = 10*/
	{MISENSOR_TOK_TERM, 0, 0}
};

/*
 * TBD
 * Optimize for two context config
 * Sensor pixel clock: 13.2 MHZ
 * hblank time: (852-328) * ( 1 / 13.2E6) = 39.6uS
 * vblank time: (516 - 248) * (852 / 13.2E6 ) = 17mS
 * frame time: 516 * (852 / 13.2E6) = 33.3mS
 */
static struct misensor_reg const mt9v113_qvga_init[] = {
	{MISENSOR_16BIT, 0x98c, 0x2703},/*Output Width (a)*/
	{MISENSOR_16BIT, 0x990, 0x0140},/*      = 320*/
	{MISENSOR_16BIT, 0x98c, 0x2705},/*Output Height (a)*/
	{MISENSOR_16BIT, 0x990, 0x00f0},/*      = 240*/
	{MISENSOR_16BIT, 0x98c, 0x2707},/*Output Width (b)*/
	{MISENSOR_16BIT, 0x990, 0x0280},/*      = 640*/
	{MISENSOR_16BIT, 0x98c, 0x2709},/*Output Height (b)*/
	{MISENSOR_16BIT, 0x990, 0x01e0},/*      = 480*/
	{MISENSOR_16BIT, 0x98c, 0x270d},/*Row Start (a)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x270f},/*column Start (a)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2711},/*Row end (a)*/
	{MISENSOR_16BIT, 0x990, 0x1e7},/*      = 487*/
	{MISENSOR_16BIT, 0x98c, 0x2713},/*column end (a)*/
	{MISENSOR_16BIT, 0x990, 0x287},/*      = 647*/
	{MISENSOR_16BIT, 0x98c, 0x2715},/*Row Speed (a)*/
	{MISENSOR_16BIT, 0x990, 0x0001},/*      = 1*/
	{MISENSOR_16BIT, 0x98c, 0x2717},/*Read Mode (a)*/
	{MISENSOR_16BIT, 0x990, 0x0026},/*      = 38*/
	{MISENSOR_16BIT, 0x98c, 0x2719},/*sensor_fine_correction (a)*/
	{MISENSOR_16BIT, 0x990, 0x001a},/*      = 26*/
	{MISENSOR_16BIT, 0x98c, 0x271b},/*sensor_fine_IT_min (a)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x271d},/*sensor_fine_IT_max_margin (a)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x271f},/*frame Lines (a)*/
	{MISENSOR_16BIT, 0x990, 0x0204},/*      = 516*/
	{MISENSOR_16BIT, 0x98c, 0x2721},/*Line Length (a)*/
	{MISENSOR_16BIT, 0x990, 0x0354},/*      = 852*/
	{MISENSOR_16BIT, 0x98c, 0x2723},/*Row Start (b)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2725},/*column Start (b)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2727},/*Row end (b)*/
	{MISENSOR_16BIT, 0x990, 0x1e7},/*      = 487*/
	{MISENSOR_16BIT, 0x98c, 0x2729},/*column end (b)*/
	{MISENSOR_16BIT, 0x990, 0x287},/*      = 647*/
	{MISENSOR_16BIT, 0x98c, 0x272b},/*Row Speed (b)*/
	{MISENSOR_16BIT, 0x990, 0x0001},/*      = 1*/
	{MISENSOR_16BIT, 0x98c, 0x272d},/*Read Mode (b)*/
	{MISENSOR_16BIT, 0x990, 0x0026},/*      = 38*/
	{MISENSOR_16BIT, 0x98c, 0x272f},/*sensor_fine_correction (b)*/
	{MISENSOR_16BIT, 0x990, 0x001a},/*      = 26*/
	{MISENSOR_16BIT, 0x98c, 0x2731},/*sensor_fine_IT_min (b)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x2733},/*sensor_fine_IT_max_margin (b)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x2735},/*frame Lines (b)*/
	{MISENSOR_16BIT, 0x990, 0x0204},/*      = 516*/
	{MISENSOR_16BIT, 0x98c, 0x2737},/*Line Length (b)*/
	{MISENSOR_16BIT, 0x990, 0x0354},/*      = 852*/
	{MISENSOR_16BIT, 0x98c, 0x2739},/*crop_X0 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x273b},/*crop_X1 (a)*/
	{MISENSOR_16BIT, 0x990, 0x027f},/*      = 639*/
	{MISENSOR_16BIT, 0x98c, 0x273d},/*crop_Y0 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x273f},/*crop_Y1 (a)*/
	{MISENSOR_16BIT, 0x990, 0x01df},/*      = 479*/
	{MISENSOR_16BIT, 0x98c, 0x2747},/*crop_X0 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2749},/*crop_X1 (b)*/
	{MISENSOR_16BIT, 0x990, 0x027f},/*      = 639*/
	{MISENSOR_16BIT, 0x98c, 0x274b},/*crop_Y0 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x274d},/*crop_Y1 (b)*/
	{MISENSOR_16BIT, 0x990, 0x01df},/*      = 479*/
	{MISENSOR_16BIT, 0x98c, 0x222d},/*R9 Step*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0xa408},/*search_f1_50*/
	{MISENSOR_16BIT, 0x990, 0x1f},/*      = 31*/
	{MISENSOR_16BIT, 0x98c, 0xa409},/*search_f2_50*/
	{MISENSOR_16BIT, 0x990, 0x21},/*      = 33*/
	{MISENSOR_16BIT, 0x98c, 0xa40a},/*search_f1_60*/
	{MISENSOR_16BIT, 0x990, 0x25},/*      = 37*/
	{MISENSOR_16BIT, 0x98c, 0xa40b},/*search_f2_60*/
	{MISENSOR_16BIT, 0x990, 0x27},/*      = 39*/
	{MISENSOR_16BIT, 0x98c, 0x2411},/*R9_Step_60 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0x2413},/*R9_Step_50 (a)*/
	{MISENSOR_16BIT, 0x990, 0x009b},/*      = 155*/
	{MISENSOR_16BIT, 0x98c, 0x2415},/*R9_Step_60 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0x2417},/*R9_Step_50 (b)*/
	{MISENSOR_16BIT, 0x990, 0x009b},/*      = 155*/
	{MISENSOR_16BIT, 0x98c, 0xa404},/*fd Mode*/
	{MISENSOR_16BIT, 0x990, 0x10},/*      = 16*/
	{MISENSOR_16BIT, 0x98c, 0xa40d},/*Stat_min*/
	{MISENSOR_16BIT, 0x990, 0x02},/*      = 2*/
	{MISENSOR_16BIT, 0x98c, 0xa40e},/*Stat_max*/
	{MISENSOR_16BIT, 0x990, 0x03},/*      = 3*/
	{MISENSOR_16BIT, 0x98c, 0xa410},/*Min_amplitude*/
	{MISENSOR_16BIT, 0x990, 0x0a},/*      = 10*/
	{MISENSOR_TOK_TERM, 0, 0}
};

/*
 * TBD
 * Optimize for two context config
 *
 * Sensor pixel clock: 13.2 MHZ
 * hblank time: (852-648) * ( 1 / 13.2E6) = 15.4uS
 * vblank time: (516 - 488) * (852 / 13.2E6 ) = 1.8mS
 * frame time: 516 * (852 / 13.2E6) = 33.3mS
 */
static struct misensor_reg const mt9v113_vga_init[] = {
	{MISENSOR_16BIT, 0x98c, 0x2703},/*Output Width (a)*/
	{MISENSOR_16BIT, 0x990, 0x0280},/*      = 640*/
	{MISENSOR_16BIT, 0x98c, 0x2705},/*Output Height (a)*/
	{MISENSOR_16BIT, 0x990, 0x01e0},/*      = 480*/
	{MISENSOR_16BIT, 0x98c, 0x2707},/*Output Width (b)*/
	{MISENSOR_16BIT, 0x990, 0x0280},/*      = 640*/
	{MISENSOR_16BIT, 0x98c, 0x2709},/*Output Height (b)*/
	{MISENSOR_16BIT, 0x990, 0x01e0},/*      = 480*/
	{MISENSOR_16BIT, 0x98c, 0x270d},/*Row Start (a)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x270f},/*column Start (a)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2711},/*Row end (a)*/
	{MISENSOR_16BIT, 0x990, 0x1e7},/*      = 487*/
	{MISENSOR_16BIT, 0x98c, 0x2713},/*column end (a)*/
	{MISENSOR_16BIT, 0x990, 0x287},/*      = 647*/
	{MISENSOR_16BIT, 0x98c, 0x2715},/*Row Speed (a)*/
	{MISENSOR_16BIT, 0x990, 0x0001},/*      = 1*/
	{MISENSOR_16BIT, 0x98c, 0x2717},/*Read Mode (a)*/
	{MISENSOR_16BIT, 0x990, 0x0026},/*      = 38*/
	{MISENSOR_16BIT, 0x98c, 0x2719},/*sensor_fine_correction (a)*/
	{MISENSOR_16BIT, 0x990, 0x001a},/*      = 26*/
	{MISENSOR_16BIT, 0x98c, 0x271b},/*sensor_fine_IT_min (a)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x271d},/*sensor_fine_IT_max_margin (a)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x271f},/*frame Lines (a)*/
	{MISENSOR_16BIT, 0x990, 0x0204},/*      = 516*/
	{MISENSOR_16BIT, 0x98c, 0x2721},/*Line Length (a)*/
	{MISENSOR_16BIT, 0x990, 0x0354},/*      = 852*/
	{MISENSOR_16BIT, 0x98c, 0x2723},/*Row Start (b)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2725},/*column Start (b)*/
	{MISENSOR_16BIT, 0x990, 0x000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2727},/*Row end (b)*/
	{MISENSOR_16BIT, 0x990, 0x1e7},/*      = 487*/
	{MISENSOR_16BIT, 0x98c, 0x2729},/*column end (b)*/
	{MISENSOR_16BIT, 0x990, 0x287},/*      = 647*/
	{MISENSOR_16BIT, 0x98c, 0x272b},/*Row Speed (b)*/
	{MISENSOR_16BIT, 0x990, 0x0001},/*      = 1*/
	{MISENSOR_16BIT, 0x98c, 0x272d},/*Read Mode (b)*/
	{MISENSOR_16BIT, 0x990, 0x0026},/*      = 38*/
	{MISENSOR_16BIT, 0x98c, 0x272f},/*sensor_fine_correction (b)*/
	{MISENSOR_16BIT, 0x990, 0x001a},/*      = 26*/
	{MISENSOR_16BIT, 0x98c, 0x2731},/*sensor_fine_IT_min (b)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x2733},/*sensor_fine_IT_max_margin (b)*/
	{MISENSOR_16BIT, 0x990, 0x006b},/*      = 107*/
	{MISENSOR_16BIT, 0x98c, 0x2735},/*frame Lines (b)*/
	{MISENSOR_16BIT, 0x990, 0x0204},/*      = 516*/
	{MISENSOR_16BIT, 0x98c, 0x2737},/*Line Length (b)*/
	{MISENSOR_16BIT, 0x990, 0x0354},/*      = 852*/
	{MISENSOR_16BIT, 0x98c, 0x2739},/*crop_X0 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x273b},/*crop_X1 (a)*/
	{MISENSOR_16BIT, 0x990, 0x027f},/*      = 639*/
	{MISENSOR_16BIT, 0x98c, 0x273d},/*crop_Y0 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x273f},/*crop_Y1 (a)*/
	{MISENSOR_16BIT, 0x990, 0x01df},/*      = 479*/
	{MISENSOR_16BIT, 0x98c, 0x2747},/*crop_X0 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x2749},/*crop_X1 (b)*/
	{MISENSOR_16BIT, 0x990, 0x027f},/*      = 639*/
	{MISENSOR_16BIT, 0x98c, 0x274b},/*crop_Y0 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0000},/*      = 0*/
	{MISENSOR_16BIT, 0x98c, 0x274d},/*crop_Y1 (b)*/
	{MISENSOR_16BIT, 0x990, 0x01df},/*      = 479*/
	{MISENSOR_16BIT, 0x98c, 0x222d},/*R9 Step*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0xa408},/*search_f1_50*/
	{MISENSOR_16BIT, 0x990, 0x1f},/*      = 31*/
	{MISENSOR_16BIT, 0x98c, 0xa409},/*search_f2_50*/
	{MISENSOR_16BIT, 0x990, 0x21},/*      = 33*/
	{MISENSOR_16BIT, 0x98c, 0xa40a},/*search_f1_60*/
	{MISENSOR_16BIT, 0x990, 0x25},/*      = 37*/
	{MISENSOR_16BIT, 0x98c, 0xa40b},/*search_f2_60*/
	{MISENSOR_16BIT, 0x990, 0x27},/*      = 39*/
	{MISENSOR_16BIT, 0x98c, 0x2411},/*R9_Step_60 (a)*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0x2413},/*R9_Step_50 (a)*/
	{MISENSOR_16BIT, 0x990, 0x009b},/*      = 155*/
	{MISENSOR_16BIT, 0x98c, 0x2415},/*R9_Step_60 (b)*/
	{MISENSOR_16BIT, 0x990, 0x0081},/*      = 129*/
	{MISENSOR_16BIT, 0x98c, 0x2417},/*R9_Step_50 (b)*/
	{MISENSOR_16BIT, 0x990, 0x009b},/*      = 155*/
	{MISENSOR_16BIT, 0x98c, 0xa404},/*fd Mode*/
	{MISENSOR_16BIT, 0x990, 0x10},/*      = 16*/
	{MISENSOR_16BIT, 0x98c, 0xa40d},/*Stat_min*/
	{MISENSOR_16BIT, 0x990, 0x02},/*      = 2*/
	{MISENSOR_16BIT, 0x98c, 0xa40e},/*Stat_max*/
	{MISENSOR_16BIT, 0x990, 0x03},/*      = 3*/
	{MISENSOR_16BIT, 0x98c, 0xa410},/*Min_amplitude*/
	{MISENSOR_16BIT, 0x990, 0x0a},/*      = 10*/
	{MISENSOR_TOK_TERM, 0, 0}
};
/*
 * Soft Reset
 * 1: Set SYSCTL 0x001A[1:0] to 0x3 to initiate internal reset cycle.
 * 2: Wait 6000 EXTCLK cycles.
 * 3: Reset SYSCTL 0x001A[1:0] to 0x0 for normal operation.
 *
 * SYSCTL
 * 1: bit9=0: Parallel output port is disabled.
 * 2: bit8=0: Output is enabled (gpio ?)
 * 3: bit4=1: GPIO not remained power on in standby
 * 4: bit3=1: Reserved in DS, MIPI enabled ?
 */
static struct misensor_reg const mt9v113_reset[] = {
	{MISENSOR_16BIT, 0x001a, 0x0011},
	{MISENSOR_TOK_DELAY, 0, 1},
	{MISENSOR_16BIT, 0x001a, 0x0010},
	{MISENSOR_TOK_DELAY, 0, 1}, /* wait for normal operation */
	{MISENSOR_TOK_TERM, 0, 0}
};

/*
 * Reduce IO Current
 */
static struct misensor_reg const mt9v113_reduce_current[] = {
	{MISENSOR_16BIT, 0x098c, 0x02f0}, /* MCU_ADDRESS*/
	{MISENSOR_16BIT, 0x0990, 0x0000}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x02f2}, /* MCU_ADDRESS*/
	{MISENSOR_16BIT, 0x0990, 0x0210}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x02f4}, /* MCU_ADDRESS*/
	{MISENSOR_16BIT, 0x0990, 0x001a}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2145}, /* [SEQ_ADVSEQ_CALLLIST_5]*/
	{MISENSOR_16BIT, 0x0990, 0x02f4}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2134}, /* [SEQ_ADVSEQ_STACKOPTIONS]*/
	{MISENSOR_16BIT, 0x0990, 0x0001}, /* MCU_DATA_0*/

	{MISENSOR_TOK_TERM, 0, 0}
};

/* AWB_CCM initialization */
static struct misensor_reg const mt9v113_awb_ccm[] = {
	{MISENSOR_16BIT, 0x098c, 0x2306}, /* MCU_ADDRESS [aWb_ccM_L_0]*/
	{MISENSOR_16BIT, 0x0990, 0x0315}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2308}, /* MCU_ADDRESS [aWb_ccM_L_1]*/
	{MISENSOR_16BIT, 0x0990, 0xfddc}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x230a}, /* MCU_ADDRESS [aWb_ccM_L_2]*/
	{MISENSOR_16BIT, 0x0990, 0x003a}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x230c}, /* MCU_ADDRESS [aWb_ccM_L_3]*/
	{MISENSOR_16BIT, 0x0990, 0xff58}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x230e}, /* MCU_ADDRESS [aWb_ccM_L_4]*/
	{MISENSOR_16BIT, 0x0990, 0x02b7}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2310}, /* MCU_ADDRESS [aWb_ccM_L_5]*/
	{MISENSOR_16BIT, 0x0990, 0xff31}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2312}, /* MCU_ADDRESS [aWb_ccM_L_6]*/
	{MISENSOR_16BIT, 0x0990, 0xff4c}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2314}, /* MCU_ADDRESS [aWb_ccM_L_7]*/
	{MISENSOR_16BIT, 0x0990, 0xfe4c}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2316}, /* MCU_ADDRESS [aWb_ccM_L_8]*/
	{MISENSOR_16BIT, 0x0990, 0x039e}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2318}, /* MCU_ADDRESS [aWb_ccM_L_9]*/
	{MISENSOR_16BIT, 0x0990, 0x001c}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x231a}, /* MCU_ADDRESS [aWb_ccM_L_10]*/
	{MISENSOR_16BIT, 0x0990, 0x0039}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x231c}, /* MCU_ADDRESS [aWb_ccM_RL_0]*/
	{MISENSOR_16BIT, 0x0990, 0x007f}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x231e}, /* MCU_ADDRESS [aWb_ccM_RL_1]*/
	{MISENSOR_16BIT, 0x0990, 0xff77}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2320}, /* MCU_ADDRESS [aWb_ccM_RL_2]*/
	{MISENSOR_16BIT, 0x0990, 0x000a}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2322}, /* MCU_ADDRESS [aWb_ccM_RL_3]*/
	{MISENSOR_16BIT, 0x0990, 0x0020}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2324}, /* MCU_ADDRESS [aWb_ccM_RL_4]*/
	{MISENSOR_16BIT, 0x0990, 0x001b}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2326}, /* MCU_ADDRESS [aWb_ccM_RL_5]*/
	{MISENSOR_16BIT, 0x0990, 0xffc6}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2328}, /* MCU_ADDRESS [aWb_ccM_RL_6]*/
	{MISENSOR_16BIT, 0x0990, 0x0086}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x232a}, /* MCU_ADDRESS [aWb_ccM_RL_7]*/
	{MISENSOR_16BIT, 0x0990, 0x00b5}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x232c}, /* MCU_ADDRESS [aWb_ccM_RL_8]*/
	{MISENSOR_16BIT, 0x0990, 0xfec3}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x232e}, /* MCU_ADDRESS [aWb_ccM_RL_9]*/
	{MISENSOR_16BIT, 0x0990, 0x0001}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2330}, /* MCU_ADDRESS [aWb_ccM_RL_10]*/
	{MISENSOR_16BIT, 0x0990, 0xffef}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa348}, /* [aWb_GaIN_bUffeR_SPeed]*/
	{MISENSOR_16BIT, 0x0990, 0x0008}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa349}, /* MCU_ADDRESS [aWb_JUMP_dIVISOR]*/
	{MISENSOR_16BIT, 0x0990, 0x0002}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa34a}, /* MCU_ADDRESS [aWb_GaIN_MIN]*/
	{MISENSOR_16BIT, 0x0990, 0x0090}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa34b}, /* MCU_ADDRESS [aWb_GaIN_MaX]*/
	{MISENSOR_16BIT, 0x0990, 0x00ff}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa34c}, /* MCU_ADDRESS [aWb_GaINMIN_b]*/
	{MISENSOR_16BIT, 0x0990, 0x0075}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa34d}, /* MCU_ADDRESS [aWb_GaINMaX_b]*/
	{MISENSOR_16BIT, 0x0990, 0x00ef}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa351}, /* [aWb_ccM_POSITION_MIN]*/
	{MISENSOR_16BIT, 0x0990, 0x0000}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa352}, /* [aWb_ccM_POSITION_MaX]*/
	{MISENSOR_16BIT, 0x0990, 0x007f}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa354}, /* MCU_ADDRESS [aWb_SaTURaTION]*/
	{MISENSOR_16BIT, 0x0990, 0x0043}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa355}, /* MCU_ADDRESS [aWb_MOde]*/
	{MISENSOR_16BIT, 0x0990, 0x0001}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa35d}, /* [aWb_STeadY_bGaIN_OUT_MIN]*/
	{MISENSOR_16BIT, 0x0990, 0x0078}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa35e}, /* [aWb_STeadY_bGaIN_OUT_MaX]*/
	{MISENSOR_16BIT, 0x0990, 0x0086}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa35f}, /* [aWb_STeadY_bGaIN_IN_MIN]*/
	{MISENSOR_16BIT, 0x0990, 0x007e}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa360}, /* [aWb_STeadY_bGaIN_IN_MaX]*/
	{MISENSOR_16BIT, 0x0990, 0x0082}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0x2361}, /* MCU_ADDRESS [aWb_cNT_PXL_TH]*/
	{MISENSOR_16BIT, 0x0990, 0x0040}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa363}, /* MCU_ADDRESS [aWb_TG_MIN0]*/
	{MISENSOR_16BIT, 0x0990, 0x00d2}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa364}, /* MCU_ADDRESS [aWb_TG_MaX0]*/
	{MISENSOR_16BIT, 0x0990, 0x00f6}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa302}, /* MCU_ADDRESS [aWb_WINdOW_POS]*/
	{MISENSOR_16BIT, 0x0990, 0x0000}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa303}, /* MCU_ADDRESS [aWb_WINdOW_SIZe]*/
	{MISENSOR_16BIT, 0x0990, 0x00ef}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xab20}, /* MCU_ADDRESS [HG_LL_SaT1]*/
	{MISENSOR_16BIT, 0x0990, 0x0024}, /* MCU_DATA_0*/

/*force aWb Setting for fW bootup*/
	{MISENSOR_16BIT, 0x098c, 0xa353}, /* MCU_ADDRESS [aWb_ccM_POSITION]*/
	{MISENSOR_16BIT, 0x0990, 0x0020}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa34e}, /* MCU_ADDRESS [aWb_GaIN_R]*/
	{MISENSOR_16BIT, 0x0990, 0x009a}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa34f}, /* MCU_ADDRESS [aWb_GaIN_G]*/
	{MISENSOR_16BIT, 0x0990, 0x0080}, /* MCU_DATA_0*/
	{MISENSOR_16BIT, 0x098c, 0xa350}, /* MCU_ADDRESS [aWb_GaIN_b]*/
	{MISENSOR_16BIT, 0x0990, 0x0082}, /* MCU_DATA_0*/

	{MISENSOR_TOK_TERM, 0, 0}
};

/* CPIPE_Calibration */
static struct misensor_reg const mt9v113_cpipe_calibration[] = {
	{MISENSOR_16BIT, 0x098c, 0x274f}, /*MOde_dec_cTRL_b*/
	{MISENSOR_16BIT, 0x0990, 0x0004},
	{MISENSOR_16BIT, 0x098c, 0x2741}, /*MOde_dec_cTRL_a*/
	{MISENSOR_16BIT, 0x0990, 0x0004},
	{MISENSOR_16BIT, 0x098c, 0x275f}, /*MOde_cOMMONMOde*/
	{MISENSOR_16BIT, 0x0990, 0x0594},
	{MISENSOR_16BIT, 0x098c, 0x2761}, /*MOde_cOMMONMOde*/
	{MISENSOR_16BIT, 0x0990, 0x0094},
	{MISENSOR_TOK_TERM, 0, 0},
};

static struct misensor_reg const mt9v113_cpipe_perference[] = {
	{MISENSOR_16BIT, 0x098c, 0x2b1f}, /*HG_LLMOde*/
	{MISENSOR_16BIT, 0x0990, 0x00c7},
	{MISENSOR_16BIT, 0x098c, 0x2b31}, /*HG_NR_STOP_G*/
	{MISENSOR_16BIT, 0x0990, 0x001e},
	{MISENSOR_16BIT, 0x098c, 0x2b20}, /*HG_LL_SaT1*/
	{MISENSOR_16BIT, 0x0990, 0x0054},
	{MISENSOR_16BIT, 0x098c, 0x2b21}, /*HG_LL_INTeRPTHReSH1*/
	{MISENSOR_16BIT, 0x0990, 0x0016},
	{MISENSOR_16BIT, 0x098c, 0x2b22}, /*HG_LL_aPcORR1*/
	{MISENSOR_16BIT, 0x0990, 0x0002},
	{MISENSOR_16BIT, 0x098c, 0x2b24}, /*HG_LL_STaT2*/
	{MISENSOR_16BIT, 0x0990, 0x0005},
	{MISENSOR_16BIT, 0x098c, 0x2b25}, /*HG_LL_INTeRPTHReSH2*/
	{MISENSOR_16BIT, 0x0990, 0x0034},
	{MISENSOR_16BIT, 0x098c, 0x2b28}, /*HG_LL_bRIGHTNeSSSTaRT*/
	{MISENSOR_16BIT, 0x0990, 0x170c},
	{MISENSOR_16BIT, 0x098c, 0x2b2a}, /*HG_LL_bRIGHTNeSSSSTOP*/
	{MISENSOR_16BIT, 0x0990, 0x3e80},

	{MISENSOR_TOK_TERM, 0, 0},
};

#endif
