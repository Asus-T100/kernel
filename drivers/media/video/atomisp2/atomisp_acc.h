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

#include <linux/atomisp.h>
#include "sh_css_types.h"
#include "atomisp_internal.h"

int atomisp_acc_load(struct atomisp_device *isp,
		     struct atomisp_acc_fw_load *fw);

int atomisp_acc_unload(struct atomisp_device *isp,
		       unsigned int *handler);

void atomisp_acc_unload_all(struct atomisp_device *isp);

int atomisp_acc_start(struct atomisp_device *isp,
		      unsigned int *handler);

int atomisp_acc_wait(struct atomisp_device *isp,
		     unsigned int *handler);

int atomisp_acc_map(struct atomisp_device *isp,
		    struct atomisp_acc_map *map);

int atomisp_acc_unmap(struct atomisp_device *isp,
		      struct atomisp_acc_map *map);

int atomisp_acc_s_mapped_arg(struct atomisp_device *isp,
			     struct atomisp_acc_s_mapped_arg *arg);
