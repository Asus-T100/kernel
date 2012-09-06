/**************************************************************************
 * Copyright (c) 2007, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Authors: Thomas Hellstrom <thomas-at-tungstengraphics.com>
 **************************************************************************/
#ifndef _PSB_SCHEDULE_H_
#define _PSB_SCHEDULE_H_

#include <drm/drmP.h>

struct psb_context;

enum psb_task_type {
	psb_flip_task
};

struct drm_psb_private;

/*
struct psb_scheduler_seq {
	uint32_t sequence;
	int reported;
};
*/

struct psb_scheduler {
	struct drm_device *dev;
	struct mutex topaz_power_mutex;
	struct mutex msvdx_power_mutex;
	struct mutex vsp_power_mutex;

	struct delayed_work topaz_suspend_wq;
	struct delayed_work msvdx_suspend_wq;
	struct delayed_work vsp_suspend_wq;
};

/*
#define PSB_RF_FIRE_TA       (1 << 0)
#define PSB_RF_OOM           (1 << 1)
#define PSB_RF_OOM_REPLY     (1 << 2)
#define PSB_RF_TERMINATE     (1 << 3)
#define PSB_RF_TA_DONE       (1 << 4)
#define PSB_RF_FIRE_RASTER   (1 << 5)
#define PSB_RF_RASTER_DONE   (1 << 6)
#define PSB_RF_DEALLOC       (1 << 7)
*/

extern int psb_scheduler_init(struct drm_device *dev,
			      struct psb_scheduler *scheduler);

#endif
