/*
* Support for Medfield PNW Camera Imaging ISP subsystem.
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

#ifndef _SH_CSS_PARAMS_DVS_H_
#define _SH_CSS_PARAMS_DVS_H_

#include "ia_css.h"


struct ia_css_dvs_6axis_config *
generate_dvs_6axis_table(const struct ia_css_resolution	*frame_res, const struct ia_css_resolution *dvs_offset);

void
free_dvs_6axis_table(struct ia_css_dvs_6axis_config  **dvs_6axis_config);

void 
copy_dvs_6axis_table(struct ia_css_dvs_6axis_config *dvs_config_dst,
			 const struct ia_css_dvs_6axis_config *dvs_config_src);


#endif
