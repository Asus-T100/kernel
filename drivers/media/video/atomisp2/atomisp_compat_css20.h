/*
 * Support for Clovertrail PNW Camera Imaging ISP subsystem.
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

#ifndef __ATOMISP_COMPAT_CSS20_H__
#define __ATOMISP_COMPAT_CSS20_H__

#include "ia_css.h"
#include "ia_css_types.h"

#define ATOMISP_CSS2_PIPE_MAX	2

#define atomisp_css_pipe_id ia_css_pipe_id
#define atomisp_css_buffer_type ia_css_buffer_type
#define atomisp_css_dis_data ia_css_isp_dvs_statistics
typedef struct ia_css_isp_3a_statistics atomisp_css_3a_data;

#define CSS_PIPE_ID_PREVIEW	IA_CSS_PIPE_ID_PREVIEW
#define CSS_PIPE_ID_COPY	IA_CSS_PIPE_ID_COPY
#define CSS_PIPE_ID_VIDEO	IA_CSS_PIPE_ID_VIDEO
#define CSS_PIPE_ID_CAPTURE	IA_CSS_PIPE_ID_CAPTURE
#define CSS_PIPE_ID_ACC		IA_CSS_PIPE_ID_ACC
#define CSS_PIPE_ID_NUM		IA_CSS_PIPE_ID_NUM

struct atomisp_css_env {
	struct ia_css_env isp_css_env;
	struct ia_css_fw isp_css_fw;
	struct ia_css_stream *stream;
	struct ia_css_stream_config stream_config;
	struct ia_css_pipe *pipes[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe *multi_pipes[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe_config pipe_configs[IA_CSS_PIPE_ID_NUM];
	struct ia_css_pipe_extra_config pipe_extra_configs[IA_CSS_PIPE_ID_NUM];
	bool update_pipe[IA_CSS_PIPE_ID_NUM];
	unsigned int curr_pipe;
	enum atomisp_css2_stream_state stream_state;
};

struct atomisp_s3a_buf {
	atomisp_css_3a_data *s3a_data;
	struct list_head list;
};

struct atomisp_dis_buf {
	struct atomisp_css_dis_data *dis_data;
	struct list_head list;
};

#endif
