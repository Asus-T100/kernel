/*
 * Support for Clovertrail PNW Camera Imaging ISP subsystem.
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

#ifndef __ATOMISP_COMPAT_H__
#define __ATOMISP_COMPAT_H__

#ifdef CONFIG_VIDEO_ATOMISP_CSS20
#include "atomisp_compat_css20.h"
#else
#include "atomisp_compat_css15.h"
#endif

#include <linux/firmware.h>

struct atomisp_device;

void atomisp_set_css_env(const struct firmware *isp,
			struct atomisp_css_env *atomisp_env);

int atomisp_css_init(struct atomisp_device *isp,
			struct atomisp_css_env *atomisp_env);

void atomisp_css_init_struct(struct atomisp_device *isp);

#endif
