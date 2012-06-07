/* Core Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) touchscreen drivers.
 * For use with Cypress Txx2xx and Txx3xx parts.
 * Supported parts include:
 * CY8CTST241
 * CY8CTMG240
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com (kev@cypress.com)
 *
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/byteorder/generic.h>
#include <linux/bitops.h>
#include <linux/cyttsp.h>
#include <linux/ctype.h>
#include <linux/earlysuspend.h>
#include "cyttsp_core.h"

/* rely on kernel input.h to define Multi-Touch capability */
#ifndef ABS_MT_TRACKING_ID
/* define only if not defined already by system; */
/* value based on linux kernel 2.6.30.10 */
#define ABS_MT_TRACKING_ID (ABS_MT_BLOB_ID + 1)
#endif /* ABS_MT_TRACKING_ID */

#define        TOUCHSCREEN_TIMEOUT (msecs_to_jiffies(28))
/* Bootloader File 0 offset */
#define CY_BL_FILE0       0x00
/* Bootloader command directive */
#define CY_BL_CMD         0xFF
/* Bootloader Enter Loader mode */
#define CY_BL_ENTER       0x38
/* Bootloader Write a Block */
#define CY_BL_WRITE_BLK   0x39
/* Bootloader Terminate Loader mode */
#define CY_BL_TERMINATE   0x3B
/* Bootloader Exit and Verify Checksum command */
#define CY_BL_EXIT        0xA5
/* Bootloader default keys */
#define CY_BL_KEY0 0
#define CY_BL_KEY1 1
#define CY_BL_KEY2 2
#define CY_BL_KEY3 3
#define CY_BL_KEY4 4
#define CY_BL_KEY5 5
#define CY_BL_KEY6 6
#define CY_BL_KEY7 7

#define CY_DIFF(m, n)               ((m) != (n))
#define GET_NUM_TOUCHES(x)          ((x) & 0x0F)
#define GET_TOUCH1_ID(x)            (((x) & 0xF0) >> 4)
#define GET_TOUCH2_ID(x)            ((x) & 0x0F)
#define GET_TOUCH3_ID(x)            (((x) & 0xF0) >> 4)
#define GET_TOUCH4_ID(x)            ((x) & 0x0F)
#define IS_LARGE_AREA(x)            (((x) & 0x10) >> 4)
#define IS_BAD_PKT(x)               ((x) & 0x20)
#define FLIP_DATA_FLAG              0x01
#define REVERSE_X_FLAG              0x02
#define REVERSE_Y_FLAG              0x04
#define FLIP_DATA(flags)            ((flags) & FLIP_DATA_FLAG)
#define REVERSE_X(flags)            ((flags) & REVERSE_X_FLAG)
#define REVERSE_Y(flags)            ((flags) & REVERSE_Y_FLAG)
#define FLIP_XY(x, y)      {typeof(x) tmp; tmp = (x); (x) = (y); (y) = tmp; }
#define INVERT_X(x, xmax)           ((xmax) - (x))
#define INVERT_Y(y, ymax)           ((ymax) - (y))
#define SET_HSTMODE(reg, mode)      ((reg) & (mode))
#define GET_HSTMODE(reg)            ((reg & 0x70) >> 4)
#define GET_BOOTLOADERMODE(reg)     ((reg & 0x10) >> 4)

/* maximum number of concurrent ST track IDs */
#define CY_NUM_ST_TCH_ID            2
/* maximum number of concurrent MT track IDs */
#define CY_NUM_MT_TCH_ID            4
/* maximum number of track IDs */
#define CY_NUM_TRK_ID               16

#define CY_NTCH                     0 /* lift off */
#define CY_TCH                      1 /* touch down */
#define CY_ST_FNGR1_IDX             0
#define CY_ST_FNGR2_IDX             1
#define CY_MT_TCH1_IDX              0
#define CY_MT_TCH2_IDX              1
#define CY_MT_TCH3_IDX              2
#define CY_MT_TCH4_IDX              3
#define CY_XPOS                     0
#define CY_YPOS                     1
#define CY_IGNR_TCH               (-1)
#define CY_SMALL_TOOL_WIDTH         10
#define CY_LARGE_TOOL_WIDTH         255
#define CY_REG_BASE                 0x00
#define CY_REG_GEST_SET             0x1E
#define CY_REG_ACT_INTRVL           0x1D
#define CY_REG_TCH_TMOUT            (CY_REG_ACT_INTRVL+1)
#define CY_REG_LP_INTRVL            (CY_REG_TCH_TMOUT+1)
#define CY_SOFT_RESET               (1 << 0)
#define CY_DEEP_SLEEP               (1 << 1)
#define CY_LOW_POWER                (1 << 2)
#define CY_MAXZ                     255
#define CY_INIT                     1
#define CY_DELAY_DFLT               10 /* ms */
#define CY_DELAY_SYSINFO            20 /* ms */
#define CY_DELAY_BL                 300
#define CY_DELAY_DNLOAD             100 /* ms */
#define CY_HNDSHK_BIT               0x80
#define CY_NUM_RETRY                4 /* max number of retries for read ops */
/* device mode bits */
#define CY_OPERATE_MODE             0x00
#define CY_SYSINFO_MODE             0x10
/* power mode select bits */
#define CY_SOFT_RESET_MODE          0x01 /* return to Bootloader mode */
#define CY_DEEP_SLEEP_MODE          0x02
#define CY_LOW_POWER_MODE           0x04
#define CY_NUM_KEY                  8

/* TrueTouch Standard Product Gen3 (Txx3xx) interface definition */
struct cyttsp_xydata {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	u16 x1 __attribute__ ((packed));
	u16 y1 __attribute__ ((packed));
	u8 z1;
	u8 touch12_id;
	u16 x2 __attribute__ ((packed));
	u16 y2 __attribute__ ((packed));
	u8 z2;
	u8 gest_cnt;
	u8 gest_id;
	u16 x3 __attribute__ ((packed));
	u16 y3 __attribute__ ((packed));
	u8 z3;
	u8 touch34_id;
	u16 x4 __attribute__ ((packed));
	u16 y4 __attribute__ ((packed));
	u8 z4;
	u8 tt_undef[3];
	u8 gest_set;
	u8 tt_reserved;
};

struct cyttsp_xydata_gen2 {
	u8 hst_mode;
	u8 tt_mode;
	u8 tt_stat;
	u16 x1 __attribute__ ((packed));
	u16 y1 __attribute__ ((packed));
	u8 z1;
	u8 evnt_idx;
	u16 x2 __attribute__ ((packed));
	u16 y2 __attribute__ ((packed));
	u8 tt_undef1;
	u8 gest_cnt;
	u8 gest_id;
	u8 tt_undef[14];
	u8 gest_set;
	u8 tt_reserved;
};

/* TrueTouch Standard Product Gen2 (Txx2xx) interface definition */
enum cyttsp_gen2_std {
	CY_GEN2_NOTOUCH = 0x03, /* Both touches removed */
	CY_GEN2_GHOST =   0x02, /* ghost */
	CY_GEN2_2TOUCH =  0x03, /* 2 touch; no ghost */
	CY_GEN2_1TOUCH =  0x01, /* 1 touch only */
	CY_GEN2_TOUCH2 =  0x01, /* 1st touch removed; 2nd touch remains */
};

/* TTSP System Information interface definition */
struct cyttsp_sysinfo_data {
	u8 hst_mode;
	u8 mfg_cmd;
	u8 mfg_stat;
	u8 cid[3];
	u8 tt_undef1;
	u8 uid[8];
	u8 bl_verh;
	u8 bl_verl;
	u8 tts_verh;
	u8 tts_verl;
	u8 app_idh;
	u8 app_idl;
	u8 app_verh;
	u8 app_verl;
	u8 tt_undef[6];
	u8 act_intrvl;
	u8 tch_tmout;
	u8 lp_intrvl;
};

/* TTSP Bootloader Register Map interface definition */
#define CY_BL_CHKSUM_OK 0x01
struct cyttsp_bootloader_data {
	u8 bl_file;
	u8 bl_status;
	u8 bl_error;
	u8 blver_hi;
	u8 blver_lo;
	u8 bld_blver_hi;
	u8 bld_blver_lo;
	u8 ttspver_hi;
	u8 ttspver_lo;
	u8 appid_hi;
	u8 appid_lo;
	u8 appver_hi;
	u8 appver_lo;
	u8 cid_0;
	u8 cid_1;
	u8 cid_2;
};

#define cyttsp_wake_data cyttsp_xydata

struct cyttsp {
	struct device *pdev;
	void *bus_priv;
	int irq;
	struct input_dev *input;
	struct work_struct work;
	struct timer_list timer;
	struct mutex mutex;
	char phys[32];
	struct cyttsp_platform_data *platform_data;
	struct cyttsp_bootloader_data bl_data;
	struct cyttsp_sysinfo_data sysinfo_data;
	u8 num_prv_st_tch;
	u16 act_trk[CY_NUM_TRK_ID];
	u16 prv_mt_tch[CY_NUM_MT_TCH_ID];
	u16 prv_st_tch[CY_NUM_ST_TCH_ID];
	u16 prv_mt_pos[CY_NUM_TRK_ID][2];
	struct cyttsp_bus_ops *bus_ops;
	unsigned fw_loader_mode:1;
	unsigned suspended:1;

	struct early_suspend early_suspend;
};

struct cyttsp_track_data {
	u8 prv_tch;
	u8 cur_tch;
	u16 cur_trk[CY_NUM_TRK_ID];
	u16 cur_st_tch[CY_NUM_ST_TCH_ID];
	u16 cur_mt_tch[CY_NUM_MT_TCH_ID];
	/* if NOT CY_USE_TRACKING_ID then only */
	/* uses CY_NUM_MT_TCH_ID positions */
	u16 cur_mt_pos[CY_NUM_TRK_ID][2];
	/* if NOT CY_USE_TRACKING_ID then only */
	/* uses CY_NUM_MT_TCH_ID positions */
	u8 cur_mt_z[CY_NUM_TRK_ID];
	u8 tool_width;
	u16 st_x1;
	u16 st_y1;
	u8 st_z1;
	u16 st_x2;
	u16 st_y2;
	u8 st_z2;
};

static const u8 bl_cmd[] = {
	CY_BL_FILE0, CY_BL_CMD, CY_BL_EXIT,
	CY_BL_KEY0, CY_BL_KEY1, CY_BL_KEY2,
	CY_BL_KEY3, CY_BL_KEY4, CY_BL_KEY5,
	CY_BL_KEY6, CY_BL_KEY7
};

void *cyttsp_get_bus_priv(struct device *dev)
{
	struct cyttsp *ts = dev_get_drvdata(dev);

	return ts->bus_priv;
}
EXPORT_SYMBOL(cyttsp_get_bus_priv);

static u8 ttsp_convert_gen2(u8 cur_tch, struct cyttsp_xydata *pxy_data)
{
	struct cyttsp_xydata_gen2 *pxy_data_gen2;
	pxy_data_gen2 = (struct cyttsp_xydata_gen2 *)(pxy_data);

	if (pxy_data_gen2->evnt_idx == CY_GEN2_NOTOUCH) {
		cur_tch = 0;
	} else if (cur_tch == CY_GEN2_GHOST) {
		cur_tch = 0;
	} else if (cur_tch == CY_GEN2_2TOUCH) {
		/* stuff artificial track ID1 and ID2 */
		pxy_data->touch12_id = 0x12;
		pxy_data->z1 = CY_MAXZ;
		pxy_data->z2 = CY_MAXZ;
		cur_tch--; /* 2 touches */
	} else if (cur_tch == CY_GEN2_1TOUCH) {
		/* stuff artificial track ID1 and ID2 */
		pxy_data->touch12_id = 0x12;
		pxy_data->z1 = CY_MAXZ;
		pxy_data->z2 = CY_NTCH;
		if (pxy_data_gen2->evnt_idx == CY_GEN2_TOUCH2) {
			/* push touch 2 data into touch1
			 * (first finger up; second finger down) */
			/* stuff artificial track ID1 for touch2 info */
			pxy_data->touch12_id = 0x20;
			/* stuff touch 1 with touch 2 coordinate data */
			pxy_data->x1 = pxy_data->x2;
			pxy_data->y1 = pxy_data->y2;
		}
	} else {
		cur_tch = 0;
	}
	return cur_tch;
}

static int ttsp_read_block_data(struct cyttsp *ts, u8 command,
		u8 length, void *buf)
{
	int rc;
	int tries;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	if (!buf || !length) {
		dev_err(ts->pdev, "%s: error buf:%s len:%u\n",
			__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}

	for (tries = 0, rc = -1; tries < CY_NUM_RETRY && rc < 0; tries++)
		rc = ts->bus_ops->read(ts->bus_priv, command, length, buf);

	if (rc < 0)
		dev_err(ts->pdev, "error read block data %d\n", rc);
	return rc;
}

static int ttsp_write_block_data(struct cyttsp *ts, u8 command,
		u8 length, void *buf)
{
	int rc;
	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	if (!buf || !length) {
		dev_err(ts->pdev, "%s: error buf:%s len:%u\n",
			__func__, !buf ? "NULL" : "OK", length);
		return -EIO;
	}

	rc = ts->bus_ops->write(ts->bus_priv, command, length, buf);
	if (rc < 0)
		dev_err(ts->pdev, "error write block data %d\n", rc);
	return rc;
}

static int ttsp_tch_ext(struct cyttsp *ts, void *buf)
{
	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	return ts->bus_ops->ext(ts->bus_priv, buf);
}

static bool cyttsp_inlist(struct cyttsp *ts, u16 prev_track[], u8 cur_trk_id,
		u8 *prev_loc, u8 num_touches)
{
	u8 id = 0;

	dev_dbg(ts->pdev, "IN p[%d]=%d c=%d n=%d loc=%d\n",
		id, prev_track[id], cur_trk_id, num_touches, *prev_loc);

	for (*prev_loc = CY_IGNR_TCH; id < num_touches; id++) {
		dev_dbg(ts->pdev, "p[%d]=%d c=%d n=%d loc=%d\n",
			id, prev_track[id], cur_trk_id, num_touches, *prev_loc);
		if (prev_track[id] == cur_trk_id) {
			*prev_loc = id;
			break;
		}
	}
	dev_dbg(ts->pdev, "OUT p[%d]=%d c=%d n=%d loc=%d\n",
		id, prev_track[id], cur_trk_id, num_touches, *prev_loc);

	return *prev_loc < CY_NUM_TRK_ID;
}

static bool cyttsp_next_avail_inlist(struct cyttsp *ts, u16 cur_trk[],
		u8 *new_loc, u8 num_touches)
{
	u8 id = 0;
	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	for (*new_loc = CY_IGNR_TCH; id < num_touches; id++) {
		if (cur_trk[id] > CY_NUM_TRK_ID) {
			*new_loc = id;
			break;
		}
	}
	return *new_loc < CY_NUM_TRK_ID;
}

/* ************************************************************************
 * The cyttsp_xy_worker function reads the XY coordinates and sends them to
 * the input layer.  It is scheduled from the interrupt (or timer).
 * *************************************************************************/
void handle_single_touch(struct cyttsp_xydata *xy, struct cyttsp_track_data *t,
		struct cyttsp *ts)
{
	u8 id;
	u8 use_trk_id = ts->platform_data->use_trk_id;

	dev_dbg(ts->pdev, "ST STEP 0 - ST1 ID=%d  ST2 ID=%d\n",
		t->cur_st_tch[CY_ST_FNGR1_IDX], t->cur_st_tch[CY_ST_FNGR2_IDX]);

	if (t->cur_st_tch[CY_ST_FNGR1_IDX] > CY_NUM_TRK_ID) {
		/* reassign finger 1 and 2 positions to new tracks */
		if (t->cur_tch > 0) {
			/* reassign st finger1 */
			if (use_trk_id) {
				id = CY_MT_TCH1_IDX;
				t->cur_st_tch[CY_ST_FNGR1_IDX] =
					t->cur_mt_tch[id];
			} else {
				id = GET_TOUCH1_ID(xy->touch12_id);
				t->cur_st_tch[CY_ST_FNGR1_IDX] = id;
			}
			t->st_x1 = t->cur_mt_pos[id][CY_XPOS];
			t->st_y1 = t->cur_mt_pos[id][CY_YPOS];
			t->st_z1 = t->cur_mt_z[id];

			dev_dbg(ts->pdev, "ST STEP 1 - ST1 ID=%3d\n",
				t->cur_st_tch[CY_ST_FNGR1_IDX]);

			if ((t->cur_tch > 1) &&
			    (t->cur_st_tch[CY_ST_FNGR2_IDX] > CY_NUM_TRK_ID)) {
				/* reassign st finger2 */
				if (use_trk_id) {
					id = CY_MT_TCH2_IDX;
					t->cur_st_tch[CY_ST_FNGR2_IDX] =
						t->cur_mt_tch[id];
				} else {
					id = GET_TOUCH2_ID(xy->touch12_id);
					t->cur_st_tch[CY_ST_FNGR2_IDX] = id;
				}
				t->st_x2 = t->cur_mt_pos[id][CY_XPOS];
				t->st_y2 = t->cur_mt_pos[id][CY_YPOS];
				t->st_z2 = t->cur_mt_z[id];

				dev_dbg(ts->pdev, "ST STEP 2 - ST2 ID=%3d\n",
					t->cur_st_tch[CY_ST_FNGR2_IDX]);
			}
		}
	} else if (t->cur_st_tch[CY_ST_FNGR2_IDX] > CY_NUM_TRK_ID) {
		if (t->cur_tch > 1) {
			/* reassign st finger2 */
			if (use_trk_id) {
				/* reassign st finger2 */
				id = CY_MT_TCH2_IDX;
				t->cur_st_tch[CY_ST_FNGR2_IDX] =
					t->cur_mt_tch[id];
			} else {
				/* reassign st finger2 */
				id = GET_TOUCH2_ID(xy->touch12_id);
				t->cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
			t->st_x2 = t->cur_mt_pos[id][CY_XPOS];
			t->st_y2 = t->cur_mt_pos[id][CY_YPOS];
			t->st_z2 = t->cur_mt_z[id];

			dev_dbg(ts->pdev, "ST STEP 3 - ST2 ID=%3d\n",
				t->cur_st_tch[CY_ST_FNGR2_IDX]);
		}
	}
	/* if the 1st touch is missing and there is a 2nd touch,
	 * then set the 1st touch to 2nd touch and terminate 2nd touch
	 */
	if ((t->cur_st_tch[CY_ST_FNGR1_IDX] > CY_NUM_TRK_ID) &&
	    (t->cur_st_tch[CY_ST_FNGR2_IDX] < CY_NUM_TRK_ID)) {
		t->st_x1 = t->st_x2;
		t->st_y1 = t->st_y2;
		t->st_z1 = t->st_z2;
		t->cur_st_tch[CY_ST_FNGR1_IDX] = t->cur_st_tch[CY_ST_FNGR2_IDX];
		t->cur_st_tch[CY_ST_FNGR2_IDX] = CY_IGNR_TCH;
	}
	/* if the 2nd touch ends up equal to the 1st touch,
	 * then just report a single touch */
	if (t->cur_st_tch[CY_ST_FNGR1_IDX] == t->cur_st_tch[CY_ST_FNGR2_IDX])
		t->cur_st_tch[CY_ST_FNGR2_IDX] = CY_IGNR_TCH;

	/* set Single Touch current event signals */
	if (t->cur_st_tch[CY_ST_FNGR1_IDX] < CY_NUM_TRK_ID) {
		input_report_abs(ts->input, ABS_X, t->st_x1);
		input_report_abs(ts->input, ABS_Y, t->st_y1);
		input_report_abs(ts->input, ABS_PRESSURE, t->st_z1);

		/* only report button down if no previous touch */
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] >= CY_NUM_TRK_ID)
			input_report_key(ts->input, BTN_TOUCH, CY_TCH);
		input_report_abs(ts->input, ABS_TOOL_WIDTH, t->tool_width);

		dev_dbg(ts->pdev, "ST->F1:%3d X:%3d Y:%3d Z:%3d\n",
			t->cur_st_tch[CY_ST_FNGR1_IDX],
			t->st_x1, t->st_y1, t->st_z1);

		if (t->cur_st_tch[CY_ST_FNGR2_IDX] < CY_NUM_TRK_ID) {
			input_report_key(ts->input, BTN_2, CY_TCH);
			input_report_abs(ts->input, ABS_HAT0X, t->st_x2);
			input_report_abs(ts->input, ABS_HAT0Y, t->st_y2);

			dev_dbg(ts->pdev, "ST->F2:%3d X:%3d Y:%3d Z:%3d\n",
				t->cur_st_tch[CY_ST_FNGR2_IDX],
				t->st_x2, t->st_y2, t->st_z2);
		} else {
			input_report_key(ts->input, BTN_2, CY_NTCH);
		}
	} else {
		input_report_abs(ts->input, ABS_PRESSURE, CY_NTCH);
		input_report_key(ts->input, BTN_TOUCH, CY_NTCH);
		input_report_key(ts->input, BTN_2, CY_NTCH);
	}
	/* update platform data for the current single touch info */
	ts->prv_st_tch[CY_ST_FNGR1_IDX] = t->cur_st_tch[CY_ST_FNGR1_IDX];
	ts->prv_st_tch[CY_ST_FNGR2_IDX] = t->cur_st_tch[CY_ST_FNGR2_IDX];

}

void handle_multi_touch(struct cyttsp_track_data *t, struct cyttsp *ts)
{
	u8 id, trk_id;
	u8 num_touch, loc;
	void (*mt_sync_func)(struct input_dev *) = ts->platform_data->mt_sync;
	u16 tmp_trk[CY_NUM_MT_TCH_ID];
	u16 snd_trk[CY_NUM_MT_TCH_ID];

	if (!ts->platform_data->use_trk_id)
		goto no_track_id;

	/* set Multi-Touch current event signals */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		trk_id = t->cur_mt_tch[id];

		if (trk_id >= CY_NUM_TRK_ID)
			continue;

		input_report_abs(ts->input, ABS_MT_TRACKING_ID, trk_id);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
				t->cur_mt_z[id]);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
				t->tool_width);
		input_report_abs(ts->input, ABS_MT_POSITION_X,
				t->cur_mt_pos[id][CY_XPOS]);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,
				t->cur_mt_pos[id][CY_YPOS]);
		if (mt_sync_func)
			mt_sync_func(ts->input);

		ts->act_trk[trk_id] = CY_TCH;
		ts->prv_mt_pos[trk_id][CY_XPOS] = t->cur_mt_pos[id][CY_XPOS];
		ts->prv_mt_pos[trk_id][CY_YPOS] = t->cur_mt_pos[id][CY_YPOS];
		dev_dbg(ts->pdev, "id: %d, tracking: %d X: %d Y: %d\n",
			id, trk_id, t->cur_mt_pos[id][CY_XPOS],
			t->cur_mt_pos[id][CY_YPOS]);
	}
	return;

no_track_id:
	/* set temporary track array elements to voids */
	memset(tmp_trk, CY_IGNR_TCH, sizeof(tmp_trk));
	memset(snd_trk, CY_IGNR_TCH, sizeof(snd_trk));

	/* get what is currently active */
	num_touch = 0;
	for (id = 0; id < CY_NUM_TRK_ID && num_touch < CY_NUM_MT_TCH_ID; id++) {
		if (t->cur_trk[id] == CY_TCH) {
			/* only incr counter if track found */
			tmp_trk[num_touch++] = id;
		}
	}
	dev_dbg(ts->pdev, "T1: t0=%d, t1=%d, t2=%d, t3=%d\n",
		tmp_trk[0], tmp_trk[1], tmp_trk[2], tmp_trk[3]);
	dev_dbg(ts->pdev, "T1: p0=%d, p1=%d, p2=%d, p3=%d\n",
		ts->prv_mt_tch[0], ts->prv_mt_tch[1],
		ts->prv_mt_tch[2], ts->prv_mt_tch[3]);

	/* pack in still active previous touches */
	for (id = t->prv_tch = 0; id < num_touch; id++) {
		if (cyttsp_inlist(ts, ts->prv_mt_tch, tmp_trk[id],
					&loc, CY_NUM_MT_TCH_ID)) {
			loc &= CY_NUM_MT_TCH_ID - 1;
			snd_trk[loc] = tmp_trk[id];
			t->prv_tch++;
			dev_dbg(ts->pdev,
				"in list s[%d]=%d t[%d]=%d,loc=%d p=%d\n",
				loc, snd_trk[loc], id, tmp_trk[id],
				loc, t->prv_tch);
		} else {
			dev_dbg(ts->pdev,
				"is not in list s[%d]=%d t[%d]=%d loc=%d\n",
				id, snd_trk[id], id, tmp_trk[id], loc);
		}
	}
	dev_dbg(ts->pdev, "S1: s0=%d, s1=%d, s2=%d, s3=%d p=%d\n",
		snd_trk[0], snd_trk[1], snd_trk[2], snd_trk[3], t->prv_tch);

	/* pack in new touches */
	for (id = 0; id < num_touch; id++) {
		if (!cyttsp_inlist(ts, snd_trk, tmp_trk[id],
					&loc, CY_NUM_MT_TCH_ID)) {

			dev_dbg(ts->pdev, "not in list t[%d]=%d, loc=%d\n",
				id, tmp_trk[id], loc);

			if (cyttsp_next_avail_inlist(ts, snd_trk, &loc,
						CY_NUM_MT_TCH_ID)) {
				loc &= CY_NUM_MT_TCH_ID - 1;
				snd_trk[loc] = tmp_trk[id];
				dev_dbg(ts->pdev,
					"put in list s[%d]=%d t[%d]=%d\n",
					loc, snd_trk[loc], id, tmp_trk[id]);
			}
		} else {
			dev_dbg(ts->pdev,
				"is in list s[%d]=%d t[%d]=%d loc=%d\n",
				id, snd_trk[id], id, tmp_trk[id], loc);
		}
	}
	dev_dbg(ts->pdev, "S2: s0=%d, s1=%d, s2=%d, s3=%d\n",
		snd_trk[0], snd_trk[1], snd_trk[2], snd_trk[3]);

	/* sync motion event signals for each current touch */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		/* z will either be 0 (NOTOUCH) or
		 * some pressure (TOUCH)
		 */
		dev_dbg(ts->pdev, "MT0 prev[%d]=%d temp[%d]=%d send[%d]=%d\n",
			id, ts->prv_mt_tch[id], id, tmp_trk[id],
			id, snd_trk[id]);

		if (snd_trk[id] < CY_NUM_TRK_ID) {
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					t->cur_mt_z[snd_trk[id]]);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					t->tool_width);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
					t->cur_mt_pos[snd_trk[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
					t->cur_mt_pos[snd_trk[id]][CY_YPOS]);
			if (mt_sync_func)
				mt_sync_func(ts->input);

			dev_dbg(ts->pdev, "MT1 -> TID:%3d X:%3d Y:%3d Z:%3d\n",
				snd_trk[id],
				t->cur_mt_pos[snd_trk[id]][CY_XPOS],
				t->cur_mt_pos[snd_trk[id]][CY_YPOS],
				t->cur_mt_z[snd_trk[id]]);

		} else if (ts->prv_mt_tch[id] < CY_NUM_TRK_ID) {
			/* void out this touch */
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					CY_NTCH);
			input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,
					t->tool_width);
			input_report_abs(ts->input, ABS_MT_POSITION_X,
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_XPOS]);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_YPOS]);

			if (mt_sync_func)
				mt_sync_func(ts->input);

			dev_dbg(ts->pdev,
				"MT2->TID:%2d X:%3d Y:%3d Z:%3d liftoff-sent\n",
				ts->prv_mt_tch[id],
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_XPOS],
				ts->prv_mt_pos[ts->prv_mt_tch[id]][CY_YPOS],
				CY_NTCH);
		} else {
			/* do not stuff any signals for this
			 * previously and currently void touches
			 */
			dev_dbg(ts->pdev,
				"MT3->send[%d]=%d - No touch - NOT sent\n",
				id, snd_trk[id]);
		}
	}

	/* save current posted tracks to
	 * previous track memory */
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		ts->prv_mt_tch[id] = snd_trk[id];
		if (snd_trk[id] < CY_NUM_TRK_ID) {
			ts->prv_mt_pos[snd_trk[id]][CY_XPOS] =
				t->cur_mt_pos[snd_trk[id]][CY_XPOS];
			ts->prv_mt_pos[snd_trk[id]][CY_YPOS] =
				t->cur_mt_pos[snd_trk[id]][CY_YPOS];
			dev_dbg(ts->pdev,
				"MT4->TID:%2d X:%3d Y:%3d Z:%3d save for prv\n",
				snd_trk[id],
				ts->prv_mt_pos[snd_trk[id]][CY_XPOS],
				ts->prv_mt_pos[snd_trk[id]][CY_YPOS],
				CY_NTCH);
		}
	}
	memset(ts->act_trk, CY_NTCH, sizeof(ts->act_trk));
	for (id = 0; id < CY_NUM_MT_TCH_ID; id++) {
		if (snd_trk[id] < CY_NUM_TRK_ID)
			ts->act_trk[snd_trk[id]] = CY_TCH;
	}
}

void cyttsp_xy_worker(struct cyttsp *ts)
{
	struct cyttsp_xydata xy_data;
	u8 id, tilt, rev_x, rev_y;
	struct cyttsp_track_data trc;
	s32 retval;

	dev_dbg(ts->pdev, "%s: Enter\n", __func__);

	/* get event data from CYTTSP device */
	retval = ttsp_read_block_data(ts, CY_REG_BASE,
			sizeof(xy_data), &xy_data);
	if (retval < 0) {
		dev_dbg(ts->pdev, "failed to read device on i2c bus\n");
		goto exit_xy_worker;
	}

	/* touch extension handling */
	retval = ttsp_tch_ext(ts, &xy_data);

	if (retval < 0) {
		dev_err(ts->pdev, "touch extension handling error\n");
		goto exit_xy_worker;
	} else if (retval > 0) {
		dev_dbg(ts->pdev, "touch extension handled\n");
		goto exit_xy_worker;
	}

	/* provide flow control handshake */
	if (ts->irq) {
		if (ts->platform_data->use_hndshk) {
			u8 cmd;
			cmd = xy_data.hst_mode & CY_HNDSHK_BIT ?
				xy_data.hst_mode & ~CY_HNDSHK_BIT :
				xy_data.hst_mode | CY_HNDSHK_BIT;
			retval = ttsp_write_block_data(ts, CY_REG_BASE,
					sizeof(cmd), (u8 *)&cmd);
		}
	}
	trc.cur_tch = GET_NUM_TOUCHES(xy_data.tt_stat);
	if (GET_HSTMODE(xy_data.hst_mode) != CY_OPERATE_MODE) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		dev_dbg(ts->pdev, "invalide mode detected\n");
	} else if (IS_LARGE_AREA(xy_data.tt_stat) == 1) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		dev_dbg(ts->pdev, "large area detected\n");
	} else if (trc.cur_tch > CY_NUM_MT_TCH_ID) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		dev_dbg(ts->pdev, "num touch error detected\n");
	} else if (IS_BAD_PKT(xy_data.tt_mode)) {
		/* terminate all active tracks */
		trc.cur_tch = CY_NTCH;
		dev_dbg(ts->pdev, "invalide buffer detected\n");
	}

	/* set tool size */
	trc.tool_width = CY_SMALL_TOOL_WIDTH;

	if (ts->platform_data->gen == CY_GEN2) {
		/* translate Gen2 interface data into comparable Gen3 data */
		trc.cur_tch = ttsp_convert_gen2(trc.cur_tch, &xy_data);
	}

	/* clear current active track ID array and count previous touches */
	for (id = 0, trc.prv_tch = CY_NTCH; id < CY_NUM_TRK_ID; id++) {
		trc.cur_trk[id] = CY_NTCH;
		trc.prv_tch += ts->act_trk[id];
	}

	/* send no events if there were no previous touches */
	/* and no new touches */
	if ((trc.prv_tch == CY_NTCH) &&
	    ((trc.cur_tch == CY_NTCH) || (trc.cur_tch > CY_NUM_MT_TCH_ID)))
		goto exit_xy_worker;

	dev_dbg(ts->pdev, "prev=%d, curr=%d\n", trc.prv_tch, trc.cur_tch);

	/* clear current single-touch array */
	memset(trc.cur_st_tch, CY_IGNR_TCH, sizeof(trc.cur_st_tch));

	/* clear single touch positions */
	trc.st_x1 = trc.st_y1 = trc.st_z1 =
		trc.st_x2 = trc.st_y2 = trc.st_z2 = CY_NTCH;

	/* clear current multi-touch arrays */
	memset(trc.cur_mt_tch, CY_IGNR_TCH, sizeof(trc.cur_mt_tch));
	memset(trc.cur_mt_pos, CY_NTCH, sizeof(trc.cur_mt_pos));
	memset(trc.cur_mt_z, CY_NTCH, sizeof(trc.cur_mt_z));

	/* Determine if display is tilted */
	tilt = !!FLIP_DATA(ts->platform_data->flags);
	/* Check for switch in origin */
	rev_x = !!REVERSE_X(ts->platform_data->flags);
	rev_y = !!REVERSE_Y(ts->platform_data->flags);

	/* process the touches */
	switch (trc.cur_tch) {
	case 4:
		xy_data.x4 = be16_to_cpu(xy_data.x4);
		xy_data.y4 = be16_to_cpu(xy_data.y4);
		if (tilt)
			FLIP_XY(xy_data.x4, xy_data.y4);

		if (rev_x)
			xy_data.x4 = INVERT_X(xy_data.x4,
					      ts->platform_data->maxx);
		if (rev_y)
			xy_data.y4 = INVERT_X(xy_data.y4,
					      ts->platform_data->maxy);

		/*
		 * Shift the tracking id to 0-based for all touches,
		 * required by Android for onFling event generation.
		 */
		id = GET_TOUCH4_ID(xy_data.touch34_id) - 1;
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_XPOS] = xy_data.x4;
			trc.cur_mt_pos[CY_MT_TCH4_IDX][CY_YPOS] = xy_data.y4;
			trc.cur_mt_z[CY_MT_TCH4_IDX] = xy_data.z4;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x4;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y4;
			trc.cur_mt_z[id] = xy_data.z4;
		}
		trc.cur_mt_tch[CY_MT_TCH4_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] < CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x4;
				trc.st_y1 = xy_data.y4;
				trc.st_z1 = xy_data.z4;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x4;
				trc.st_y2 = xy_data.y4;
				trc.st_z2 = xy_data.z4;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		dev_dbg(ts->pdev, "4th XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
			xy_data.x4, xy_data.y4, xy_data.z4,
			(xy_data.touch34_id & 0x0F));

		/* do not break */
	case 3:
		xy_data.x3 = be16_to_cpu(xy_data.x3);
		xy_data.y3 = be16_to_cpu(xy_data.y3);
		if (tilt)
			FLIP_XY(xy_data.x3, xy_data.y3);

		if (rev_x)
			xy_data.x3 = INVERT_X(xy_data.x3,
					      ts->platform_data->maxx);
		if (rev_y)
			xy_data.y3 = INVERT_X(xy_data.y3,
					      ts->platform_data->maxy);

		id = GET_TOUCH3_ID(xy_data.touch34_id) - 1;
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_XPOS] = xy_data.x3;
			trc.cur_mt_pos[CY_MT_TCH3_IDX][CY_YPOS] = xy_data.y3;
			trc.cur_mt_z[CY_MT_TCH3_IDX] = xy_data.z3;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x3;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y3;
			trc.cur_mt_z[id] = xy_data.z3;
		}
		trc.cur_mt_tch[CY_MT_TCH3_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] < CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x3;
				trc.st_y1 = xy_data.y3;
				trc.st_z1 = xy_data.z3;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x3;
				trc.st_y2 = xy_data.y3;
				trc.st_z2 = xy_data.z3;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		dev_dbg(ts->pdev, "3rd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
			xy_data.x3, xy_data.y3, xy_data.z3,
			((xy_data.touch34_id >> 4) & 0x0F));

		/* do not break */
	case 2:
		xy_data.x2 = be16_to_cpu(xy_data.x2);
		xy_data.y2 = be16_to_cpu(xy_data.y2);
		if (tilt)
			FLIP_XY(xy_data.x2, xy_data.y2);

		if (rev_x)
			xy_data.x2 = INVERT_X(xy_data.x2,
					      ts->platform_data->maxx);
		if (rev_y)
			xy_data.y2 = INVERT_X(xy_data.y2,
					      ts->platform_data->maxy);
		id = GET_TOUCH2_ID(xy_data.touch12_id) - 1;
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_XPOS] = xy_data.x2;
			trc.cur_mt_pos[CY_MT_TCH2_IDX][CY_YPOS] = xy_data.y2;
			trc.cur_mt_z[CY_MT_TCH2_IDX] = xy_data.z2;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x2;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y2;
			trc.cur_mt_z[id] = xy_data.z2;
		}
		trc.cur_mt_tch[CY_MT_TCH2_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] < CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x2;
				trc.st_y1 = xy_data.y2;
				trc.st_z1 = xy_data.z2;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x2;
				trc.st_y2 = xy_data.y2;
				trc.st_z2 = xy_data.z2;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		dev_dbg(ts->pdev, "2nd XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
			xy_data.x2, xy_data.y2, xy_data.z2,
			(xy_data.touch12_id & 0x0F));

		/* do not break */
	case 1:
		xy_data.x1 = be16_to_cpu(xy_data.x1);
		xy_data.y1 = be16_to_cpu(xy_data.y1);
		if (tilt)
			FLIP_XY(xy_data.x1, xy_data.y1);

		if (rev_x)
			xy_data.x1 = INVERT_X(xy_data.x1,
					      ts->platform_data->maxx);
		if (rev_y)
			xy_data.y1 = INVERT_X(xy_data.y1,
					      ts->platform_data->maxy);

		id = GET_TOUCH1_ID(xy_data.touch12_id) - 1;
		if (ts->platform_data->use_trk_id) {
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_XPOS] = xy_data.x1;
			trc.cur_mt_pos[CY_MT_TCH1_IDX][CY_YPOS] = xy_data.y1;
			trc.cur_mt_z[CY_MT_TCH1_IDX] = xy_data.z1;
		} else {
			trc.cur_mt_pos[id][CY_XPOS] = xy_data.x1;
			trc.cur_mt_pos[id][CY_YPOS] = xy_data.y1;
			trc.cur_mt_z[id] = xy_data.z1;
		}
		trc.cur_mt_tch[CY_MT_TCH1_IDX] = id;
		trc.cur_trk[id] = CY_TCH;
		if (ts->prv_st_tch[CY_ST_FNGR1_IDX] < CY_NUM_TRK_ID) {
			if (ts->prv_st_tch[CY_ST_FNGR1_IDX] == id) {
				trc.st_x1 = xy_data.x1;
				trc.st_y1 = xy_data.y1;
				trc.st_z1 = xy_data.z1;
				trc.cur_st_tch[CY_ST_FNGR1_IDX] = id;
			} else if (ts->prv_st_tch[CY_ST_FNGR2_IDX] == id) {
				trc.st_x2 = xy_data.x1;
				trc.st_y2 = xy_data.y1;
				trc.st_z2 = xy_data.z1;
				trc.cur_st_tch[CY_ST_FNGR2_IDX] = id;
			}
		}
		dev_dbg(ts->pdev, "1st XYZ:% 3d,% 3d,% 3d  ID:% 2d\n",
			xy_data.x1, xy_data.y1, xy_data.z1,
			((xy_data.touch12_id >> 4) & 0x0F));

		break;
	case 0:
	default:
			break;
	}

	if (ts->platform_data->use_st)
		handle_single_touch(&xy_data, &trc, ts);

	if (ts->platform_data->use_mt) {
		if (trc.cur_tch)
			handle_multi_touch(&trc, ts);
		else
			ts->platform_data->mt_sync(ts->input);
	}

	/* handle gestures */
	if (ts->platform_data->use_gestures && xy_data.gest_id) {
		input_report_key(ts->input, BTN_3, CY_TCH);
		input_report_abs(ts->input, ABS_HAT1X, xy_data.gest_id);
		input_report_abs(ts->input, ABS_HAT2Y, xy_data.gest_cnt);
	}

	/* signal the view motion event */
	input_sync(ts->input);

exit_xy_worker:
	dev_dbg(ts->pdev, "%s: finished.\n", __func__);
	return;
}

/* ************************************************************************
 * ISR function. This function is general, initialized in drivers init
 * function and disables further IRQs until this IRQ is processed in worker.
 * *************************************************************************/
static irqreturn_t cyttsp_irq(int irq, void *handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	dev_dbg(ts->pdev, "Got IRQ!\n");
	cyttsp_xy_worker(ts);
	return IRQ_HANDLED;
}

/* schedulable worker entry for timer polling method */
void cyttsp_xy_worker_(struct work_struct *work)
{
	struct cyttsp *ts = container_of(work, struct cyttsp, work);
	cyttsp_xy_worker(ts);
}

/* Timer function used as touch interrupt generator for polling method */
static void cyttsp_timer(unsigned long handle)
{
	struct cyttsp *ts = (struct cyttsp *)handle;

	dev_dbg(ts->pdev, "TTSP timer event!\n");
	/* schedule motion signal handling */
	if (!work_pending(&ts->work))
		schedule_work(&ts->work);
	mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	return;
}


/* ************************************************************************
 * Probe initialization functions
 * ************************************************************************ */
static int cyttsp_load_bl_regs(struct cyttsp *ts)
{
	int retval = 0;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	retval = ttsp_read_block_data(ts, CY_REG_BASE,
				sizeof(ts->bl_data), &(ts->bl_data));

	if (retval < 0) {
		dev_err(ts->pdev, "%s: Failed reading block data, err:%d\n",
			__func__, retval);
	}
	return retval;
}

static int cyttsp_exit_bl_mode(struct cyttsp *ts)
{
	int retval;
	int tries = 0;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(bl_cmd),
			(void *)bl_cmd);
	if (retval < 0) {
		dev_err(ts->pdev, "%s: failed writing block data, err:%d\n",
			__func__, retval);
		goto fail;
	}
	do {
		msleep(500);
		cyttsp_load_bl_regs(ts);
	} while (GET_BOOTLOADERMODE(ts->bl_data.bl_status) && tries++ < 10);
	return 0;
fail:
	return retval;
}

static int cyttsp_set_sysinfo_mode(struct cyttsp *ts)
{
	int retval;
	u8 cmd = CY_SYSINFO_MODE;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	/* wait for TTSP Device to complete switch to SysInfo mode */
	msleep(500);
	if (retval < 0) {
		dev_err(ts->pdev, "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		goto write_block_data_fail;
	}
	retval = ttsp_read_block_data(ts, CY_REG_BASE, sizeof(ts->sysinfo_data),
				&(ts->sysinfo_data));

	if (retval < 0) {
		dev_err(ts->pdev, "%s: Failed reading block data, err:%d\n",
			__func__, retval);
		goto read_block_data_fail;
	}

	dev_dbg(ts->pdev,
		"SI2: hst_mode=0x%02X mfg_cmd=0x%02X mfg_stat=0x%02X\n",
		ts->sysinfo_data.hst_mode, ts->sysinfo_data.mfg_cmd,
		ts->sysinfo_data.mfg_stat);

	dev_dbg(ts->pdev, "SI2: bl_ver=0x%02X%02X\n",
		ts->sysinfo_data.bl_verh, ts->sysinfo_data.bl_verl);

	dev_dbg(ts->pdev,
		"SI2: act_intrvl=0x%02X tch_tmout=0x%02X lp_intrvl=0x%02X\n",
		ts->sysinfo_data.act_intrvl,
		ts->sysinfo_data.tch_tmout,
		ts->sysinfo_data.lp_intrvl);

	dev_info(ts->pdev, "SI2: tts_ver=0x%02X%02X app_id=0x%02X%02X "
		"app_ver=0x%02X%02X c_id=0x%02X%02X%02X\n",
		ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
		ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
		ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
		ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
		ts->sysinfo_data.cid[2]);
	return 0;

write_block_data_fail:
read_block_data_fail:
	return retval;
}

static int cyttsp_set_operational_mode(struct cyttsp *ts)
{
	int retval;
	u8 cmd = CY_OPERATE_MODE;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0) {
		dev_err(ts->pdev, "%s: Failed writing block data, err:%d\n",
			__func__, retval);
		return retval;
	}
	/* wait for TTSP Device to complete switch to Operational mode */
	msleep(500);
	return 0;
}

static int cyttsp_soft_reset(struct cyttsp *ts)
{
	int retval;
	int tries;
	u8 cmd = CY_SOFT_RESET_MODE;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);
	/* reset TTSP Device back to bootloader mode */
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	/* wait for TTSP Device to complete reset back to bootloader */
	tries = 0;
	msleep(200);
	do {
		msleep(100);
		cyttsp_load_bl_regs(ts);
	} while (ts->bl_data.bl_status != 0x10 &&
		 ts->bl_data.bl_status != 0x11 &&
		 tries++ < 100);
	if (tries >= 100)
		dev_err(ts->pdev, "%s: bootlodader not ready, status 0x%02x\n",
			__func__, ts->bl_data.bl_status);
	return retval;
}

static int cyttsp_power_on(struct cyttsp *ts)
{
	int retval = 0;
	u8 cmd;
	int tries = 0;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);
	/* check if the TTSP device has a bootloader installed */
	retval = cyttsp_soft_reset(ts);
	if (retval < 0)
		goto bypass;

	do {
		msleep(500);
		retval = cyttsp_load_bl_regs(ts);
	} while (!GET_BOOTLOADERMODE(ts->bl_data.bl_status) &&
		!(GET_HSTMODE(ts->bl_data.bl_file) == CY_OPERATE_MODE) &&
		tries++ < 10);

	/* is bootloader missing? */
	if (!GET_BOOTLOADERMODE(ts->bl_data.bl_status)) {
		/* skip all bootloader and sys info and */
		/* go straight to operational mode */
		if (!(retval < 0)) {
			cyttsp_set_operational_mode(ts);
			goto bypass;
		}
	}
	if (retval < 0)
		goto bypass;

	retval = cyttsp_exit_bl_mode(ts);

	if (retval < 0)
		goto bypass;

	/* switch to System Information mode to read */
	/* versions and set interval registers */
	retval = cyttsp_set_sysinfo_mode(ts);
	if (retval < 0)
		goto bypass;
	if ((CY_DIFF(ts->platform_data->act_intrvl, CY_ACT_INTRVL_DFLT) ||
	     CY_DIFF(ts->platform_data->tch_tmout, CY_TCH_TMOUT_DFLT) ||
	     CY_DIFF(ts->platform_data->lp_intrvl, CY_LP_INTRVL_DFLT))) {

		u8 intrvl_ray[3];

		intrvl_ray[0] = ts->platform_data->act_intrvl;
		intrvl_ray[1] = ts->platform_data->tch_tmout;
		intrvl_ray[2] = ts->platform_data->lp_intrvl;

		dev_dbg(ts->pdev,
			"act_intrvl=0x%02X tch_tmout=0x%02X lp_intrvl=0x%02X\n",
			ts->platform_data->act_intrvl,
			ts->platform_data->tch_tmout,
			ts->platform_data->lp_intrvl);

		/* set intrvl registers */
		retval = ttsp_write_block_data(ts, CY_REG_ACT_INTRVL,
					sizeof(intrvl_ray), intrvl_ray);

		msleep(CY_DELAY_SYSINFO);
	}
	/* switch back to Operational mode */
	dev_dbg(ts->pdev, "switch back to operational mode\n");
	if (retval < 0)
		goto bypass;

	cmd = CY_OPERATE_MODE;
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	/* wait for TTSP Device to complete switch to */
	/* Operational mode */
	msleep(1000);
	/* init gesture setup */
	if (retval < 0)
		goto bypass;

	/* enable low power mode, let firmware do auto power management */
	cmd = CY_LOW_POWER_MODE;
	retval = ttsp_write_block_data(ts, CY_REG_BASE, sizeof(cmd), &cmd);
	if (retval < 0)
		goto bypass;

	if (ts->platform_data->use_gestures) {
		u8 gesture_setup;

		dev_dbg(ts->pdev, "Init gesture setup\n");

		gesture_setup = ts->platform_data->gest_set;
		retval = ttsp_write_block_data(ts, CY_REG_GEST_SET,
					sizeof(gesture_setup), &gesture_setup);
		msleep(CY_DELAY_DFLT);
	}

bypass:
	if (retval < 0)
		ts->platform_data->power_state = CY_IDLE_STATE;
	else
		ts->platform_data->power_state = CY_ACTIVE_STATE;

	dev_dbg(ts->pdev, "Power state is %s\n",
		(ts->platform_data->power_state == CY_ACTIVE_STATE) ?
		"ACTIVE" : "IDLE");
	return retval;
}

#ifdef CONFIG_PM
int cyttsp_resume(void *handle)
{
	int retval = 0;
	struct cyttsp *ts = handle;
	struct cyttsp_xydata xydata;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);
	mutex_lock(&ts->mutex);

	if (!ts->fw_loader_mode && ts->suspended) {
		ts->suspended = 0;
		if (ts->platform_data->use_sleep &&
		    (ts->platform_data->power_state != CY_ACTIVE_STATE)) {
			if (ts->platform_data->wakeup)
				retval = ts->platform_data->wakeup();

			if (retval < 0)
				dev_err(ts->pdev, "wakeup failed!\n");
			else {
				if (ts->platform_data->use_timer)
					mod_timer(&ts->timer,
						jiffies + TOUCHSCREEN_TIMEOUT);
				else
					enable_irq(ts->irq);
				retval = ttsp_read_block_data(ts, CY_REG_BASE,
						sizeof(xydata), &xydata);
				if (!(retval < 0) &&
				    (GET_HSTMODE(xydata.hst_mode) ==
							CY_OPERATE_MODE)) {
					ts->platform_data->power_state =
							CY_ACTIVE_STATE;
					retval = 0;
				}
			}
		}
	}

	mutex_unlock(&ts->mutex);
	dev_dbg(ts->pdev, "Wake Up %s\n", (retval < 0) ? "FAIL" : "PASS");

	return retval;
}
EXPORT_SYMBOL(cyttsp_resume);

int cyttsp_suspend(void *handle)
{
	struct cyttsp *ts = handle;
	u8 sleep_mode = 0;
	int retval = 0;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);
	mutex_lock(&ts->mutex);

	if (!ts->fw_loader_mode && !ts->suspended) {
		if (ts->platform_data->use_timer) {
			del_timer(&ts->timer);
			cancel_work_sync(&ts->work);
		} else {
			/* this call waits for any pending IRQ handlers */
			disable_irq(ts->irq);
		}
		ts->suspended = 1;
		if (ts->platform_data->use_sleep &&
		    (ts->platform_data->power_state == CY_ACTIVE_STATE)) {
			sleep_mode = CY_DEEP_SLEEP_MODE | CY_LOW_POWER_MODE;
			retval = ttsp_write_block_data(ts, CY_REG_BASE,
					sizeof(sleep_mode), &sleep_mode);
			if (!(retval < 0)) {
				ts->platform_data->power_state = CY_SLEEP_STATE;
				retval = 0;
			}
		}
		dev_dbg(ts->pdev, "Sleep Power state is %s\n",
			(ts->platform_data->power_state == CY_ACTIVE_STATE) ?
			"ACTIVE" :
			((ts->platform_data->power_state == CY_SLEEP_STATE) ?
			 "SLEEP" : "LOW POWER"));
	}

	mutex_unlock(&ts->mutex);
	return retval;
}
EXPORT_SYMBOL(cyttsp_suspend);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp_early_suspend(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	cyttsp_suspend(ts);
}

static void cyttsp_late_resume(struct early_suspend *h)
{
	struct cyttsp *ts = container_of(h, struct cyttsp, early_suspend);

	cyttsp_resume(ts);
}
#endif

static ssize_t firmware_write(struct file *file, struct kobject *kobj,
		struct bin_attribute *bin_attr,
		char *buf, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->mutex);
	ttsp_write_block_data(ts, CY_REG_BASE, size, buf);
	mutex_unlock(&ts->mutex);
	return size;
}

static ssize_t firmware_read(struct file *file, struct kobject *kobj,
		struct bin_attribute *ba,
		char *buf, loff_t pos, size_t size)
{
	int count = 0;
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cyttsp *ts = dev_get_drvdata(dev);

	mutex_lock(&ts->mutex);
	if (!ts->fw_loader_mode)
		goto exit;
	if (!cyttsp_load_bl_regs(ts)) {
		*(unsigned short *)buf = ts->bl_data.bl_status << 8 |
					 ts->bl_data.bl_error;
		count = sizeof(unsigned short);
	} else {
		dev_err(ts->pdev, "error reading status\n");
		count = 0;
	}
exit:
	mutex_unlock(&ts->mutex);
	return count;
}

static struct bin_attribute cyttsp_firmware = {
	.attr = {
		.name = "firmware",
		.mode = 0644,
	},
	.size = 128,
	.read = firmware_read,
	.write = firmware_write,
};

static ssize_t attr_fwloader_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cyttsp *ts = dev_get_drvdata(dev);
	return sprintf(buf, "0x%02X%02X 0x%02X%02X 0x%02X%02X 0x%02X%02X%02X\n",
			ts->sysinfo_data.tts_verh, ts->sysinfo_data.tts_verl,
			ts->sysinfo_data.app_idh, ts->sysinfo_data.app_idl,
			ts->sysinfo_data.app_verh, ts->sysinfo_data.app_verl,
			ts->sysinfo_data.cid[0], ts->sysinfo_data.cid[1],
			ts->sysinfo_data.cid[2]);
}

static ssize_t attr_fwloader_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *p;
	int ret;
	struct cyttsp *ts = dev_get_drvdata(dev);
	unsigned val = simple_strtoul(buf, &p, 10);

	ret = p - buf;
	if (*p && isspace(*p))
		ret++;

	dev_dbg(ts->pdev, "%s: %u\n", __func__, val);

	mutex_lock(&ts->mutex);
	if (val && !ts->fw_loader_mode) {
		ts->fw_loader_mode = 1;
#ifdef CONFIG_PM
		if (ts->suspended)
			cyttsp_resume(ts);
#endif
		if (ts->platform_data->use_timer)
			del_timer(&ts->timer);
		else
			disable_irq_nosync(ts->irq);
		cancel_work_sync(&ts->work);

		ts->suspended = 0;
		if (sysfs_create_bin_file(&dev->kobj, &cyttsp_firmware))
			dev_err(ts->pdev, "unable to create file\n");
		cyttsp_soft_reset(ts);
		dev_info(ts->pdev, "FW loader started.\n");
	} else if (!val && ts->fw_loader_mode) {
		sysfs_remove_bin_file(&dev->kobj, &cyttsp_firmware);
		cyttsp_soft_reset(ts);
		cyttsp_exit_bl_mode(ts);
		cyttsp_set_sysinfo_mode(ts);
		cyttsp_set_operational_mode(ts);

		if (ts->platform_data->use_timer)
			mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
		else
			enable_irq(ts->irq);
		ts->fw_loader_mode = 0;
		dev_info(ts->pdev, "FW loader finished.\n");
	}
	mutex_unlock(&ts->mutex);
	return  ret == size ? ret : -EINVAL;
}

static struct device_attribute fwloader =
	__ATTR(fwloader, 0644, attr_fwloader_show, attr_fwloader_store);

int cyttsp_core_init(void *bus_priv, struct cyttsp_bus_ops *bus_ops,
		struct device *pdev)
{
	struct input_dev *input_device;
	struct cyttsp *ts;
	int retval = 0;

	dev_dbg(pdev, "%s: enter\n", __func__);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		dev_err(pdev, "%s: error kzalloc\n", __func__);
		retval = -ENOMEM;
		goto error_alloc_data_failed;
	}
	mutex_init(&ts->mutex);
	ts->pdev = pdev;
	ts->platform_data = pdev->platform_data;
	ts->bus_ops = bus_ops;
	ts->bus_priv = bus_priv;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = cyttsp_early_suspend;
	ts->early_suspend.resume = cyttsp_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	if (ts->platform_data->init)
		retval = ts->platform_data->init(1);
	if (retval) {
		dev_err(pdev, "platform init failed!\n");
		goto error_init;
	}

	if (ts->platform_data->use_timer)
		ts->irq = -1;
	else
		ts->irq = gpio_to_irq(ts->platform_data->irq_gpio);

	retval = cyttsp_power_on(ts);
	if (retval < 0) {
		dev_err(pdev, "power on failed!\n");
		goto error_power_on;
	}

	/* Create the input device and register it. */
	input_device = input_allocate_device();
	if (!input_device) {
		dev_err(pdev, "failed to allocate input device\n");
		retval = -ENOMEM;
		goto error_input_allocate_device;
	}

	ts->input = input_device;
	input_device->name = ts->platform_data->name;
	input_device->phys = ts->phys;
	input_device->dev.parent = ts->pdev;
	/* init the touch structures */
	ts->num_prv_st_tch = CY_NTCH;
	memset(ts->act_trk, CY_NTCH, sizeof(ts->act_trk));
	memset(ts->prv_mt_pos, CY_NTCH, sizeof(ts->prv_mt_pos));
	memset(ts->prv_mt_tch, CY_IGNR_TCH, sizeof(ts->prv_mt_tch));
	memset(ts->prv_st_tch, CY_IGNR_TCH, sizeof(ts->prv_st_tch));

	__set_bit(EV_SYN, input_device->evbit);
	__set_bit(EV_KEY, input_device->evbit);
	__set_bit(EV_ABS, input_device->evbit);
	__set_bit(BTN_TOUCH, input_device->keybit);
	__set_bit(BTN_2, input_device->keybit);
	if (ts->platform_data->use_gestures)
		__set_bit(BTN_3, input_device->keybit);

	input_set_abs_params(input_device, ABS_X, 0,
			     ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_Y, 0,
			     ts->platform_data->maxy, 0, 0);
	input_set_abs_params(input_device, ABS_TOOL_WIDTH, 0,
			     CY_LARGE_TOOL_WIDTH, 0, 0);
	input_set_abs_params(input_device, ABS_PRESSURE, 0, CY_MAXZ, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0X, 0,
			     ts->platform_data->maxx, 0, 0);
	input_set_abs_params(input_device, ABS_HAT0Y, 0,
			     ts->platform_data->maxy, 0, 0);
	if (ts->platform_data->use_gestures) {
		input_set_abs_params(input_device, ABS_HAT1X, 0,
				     CY_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_HAT1Y, 0,
				     CY_MAXZ, 0, 0);
	}
	if (ts->platform_data->use_mt) {
		input_set_abs_params(input_device, ABS_MT_POSITION_X, 0,
				     ts->platform_data->maxx, 0, 0);
		input_set_abs_params(input_device, ABS_MT_POSITION_Y, 0,
				     ts->platform_data->maxy, 0, 0);
		input_set_abs_params(input_device, ABS_MT_TOUCH_MAJOR, 0,
				     CY_MAXZ, 0, 0);
		input_set_abs_params(input_device, ABS_MT_WIDTH_MAJOR, 0,
				     CY_LARGE_TOOL_WIDTH, 0, 0);
		if (ts->platform_data->use_trk_id)
			input_set_abs_params(input_device, ABS_MT_TRACKING_ID,
					     0, CY_NUM_TRK_ID, 0, 0);
	}

	if (ts->platform_data->use_virtual_keys)
		input_set_capability(input_device, EV_KEY, KEY_PROG1);

	retval = input_register_device(input_device);
	if (retval) {
		dev_err(pdev, "failed to register input device\n");
		goto error_input_register_device;
	}

	dev_dbg(pdev, "registered input device\n");

	/* Prepare our worker structure prior to setting up the timer ISR */
	INIT_WORK(&ts->work, cyttsp_xy_worker_);

	/* Timer or Interrupt setup */
	if (ts->platform_data->use_timer) {
		dev_dbg(pdev, "setup timer\n");
		setup_timer(&ts->timer, cyttsp_timer, (unsigned long) ts);
		mod_timer(&ts->timer, jiffies + TOUCHSCREEN_TIMEOUT);
	} else {
		dev_dbg(pdev, "setup interrupt\n");
		retval = request_threaded_irq(ts->irq, NULL, cyttsp_irq,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					input_device->name, ts);

		if (retval) {
			dev_err(pdev, "failed to request irq\n");
			goto error_free_irq;
		}
		dev_dbg(pdev, "interrupt=%d\n", ts->irq);
	}
	retval = device_create_file(pdev, &fwloader);
	if (retval) {
		dev_err(pdev, "could not create attribute\n");
		goto device_create_error;
	}
	dev_set_drvdata(pdev, ts);
	return 0;

device_create_error:
error_free_irq:
	if (ts->irq >= 0)
		free_irq(ts->irq, ts);
	input_unregister_device(input_device);
error_input_register_device:
	input_free_device(input_device);
error_input_allocate_device:
error_power_on:
	if (ts->platform_data->init)
		ts->platform_data->init(0);
error_init:
	kfree(ts);
error_alloc_data_failed:
	return retval;
}
EXPORT_SYMBOL(cyttsp_core_init);

/* registered in driver struct */
void cyttsp_core_release(void *handle)
{
	struct cyttsp *ts = handle;

	dev_dbg(ts->pdev, "%s: enter\n", __func__);

	cancel_work_sync(&ts->work);
	if (ts->platform_data->use_timer)
		del_timer_sync(&ts->timer);
	else
		free_irq(ts->irq, ts);
	input_unregister_device(ts->input);
	input_free_device(ts->input);
	if (ts->platform_data->init)
		ts->platform_data->init(0);
	kfree(ts);
}
EXPORT_SYMBOL(cyttsp_core_release);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard touchscreen driver core");
MODULE_AUTHOR("Cypress");

