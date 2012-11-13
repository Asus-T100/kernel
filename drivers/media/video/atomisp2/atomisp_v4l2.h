/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
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

#ifndef ATOMISP_V4L2_H_
#define ATOMISP_V4L2_H_

#include <linux/videodev2.h>

/* SH header files */

#include <sh_css_types.h>

#define ATOMISP_MAJOR		0
#define ATOMISP_MINOR		5
#define ATOMISP_PATCHLEVEL	1

#define DRIVER_VERSION_STR	__stringify(ATOMISP_MAJOR) \
	"." __stringify(ATOMISP_MINOR) "." __stringify(ATOMISP_PATCHLEVEL)
#define DRIVER_VERSION		KERNEL_VERSION(ATOMISP_MAJOR, \
	ATOMISP_MINOR, ATOMISP_PATCHLEVEL)

/*ISP binary running mode*/
#define CI_MODE_PREVIEW		0x8000
#define CI_MODE_VIDEO		0x4000
#define CI_MODE_STILL_CAPTURE	0x2000
#define CI_MODE_NONE		0x0000

#define ATOM_ISP_STEP_WIDTH	2
#define ATOM_ISP_STEP_HEIGHT	2

#define ATOM_ISP_MIN_WIDTH	256
#define ATOM_ISP_MIN_HEIGHT	2
#define ATOM_ISP_MAX_WIDTH	4352
#define ATOM_ISP_MAX_HEIGHT	3264

#define ATOM_ISP_MAX_WIDTH_TMP	1280
#define ATOM_ISP_MAX_HEIGHT_TMP	720

#endif /* ATOMISP_V4L2_H_ */
