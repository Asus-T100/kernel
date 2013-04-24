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

#include <media/v4l2-mediabus.h>

#include "ia_css.h"
#include "ia_css_types.h"

#define ATOMISP_CSS2_PIPE_MAX	2

#define atomisp_css_pipe_id ia_css_pipe_id
#define atomisp_css_buffer_type ia_css_buffer_type
#define atomisp_css_dis_data ia_css_isp_dvs_statistics
#define atomisp_css_irq_info  ia_css_irq_info
#define atomisp_css_isp_config ia_css_isp_config
#define atomisp_css_bayer_order ia_css_bayer_order
#define atomisp_css_stream_format ia_css_stream_format
#define atomisp_css_capture_mode ia_css_capture_mode
#define atomisp_css_input_mode ia_css_input_mode
typedef struct ia_css_isp_3a_statistics atomisp_css_3a_data;

#define CSS_PIPE_ID_PREVIEW	IA_CSS_PIPE_ID_PREVIEW
#define CSS_PIPE_ID_COPY	IA_CSS_PIPE_ID_COPY
#define CSS_PIPE_ID_VIDEO	IA_CSS_PIPE_ID_VIDEO
#define CSS_PIPE_ID_CAPTURE	IA_CSS_PIPE_ID_CAPTURE
#define CSS_PIPE_ID_ACC		IA_CSS_PIPE_ID_ACC
#define CSS_PIPE_ID_NUM		IA_CSS_PIPE_ID_NUM

#define CSS_INPUT_MODE_SENSOR	IA_CSS_INPUT_MODE_SENSOR
#define CSS_INPUT_MODE_FIFO	IA_CSS_INPUT_MODE_FIFO
#define CSS_INPUT_MODE_TPG	IA_CSS_INPUT_MODE_TPG
#define CSS_INPUT_MODE_PRBS	IA_CSS_INPUT_MODE_PRBS
#define CSS_INPUT_MODE_MEMORY	IA_CSS_INPUT_MODE_MEMORY
#define CSS_INPUT_MODE_BUFFERED_SENSOR	IA_CSS_INPUT_MODE_BUFFERED_SENSOR

#define CSS_IRQ_INFO_CSS_RECEIVER_ERROR	IA_CSS_IRQ_INFO_CSS_RECEIVER_ERROR
#define CSS_IRQ_INFO_EVENTS_READY	IA_CSS_IRQ_INFO_EVENTS_READY
#define CSS_IRQ_INFO_INPUT_SYSTEM_ERROR \
	IA_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR
#define CSS_IRQ_INFO_IF_ERROR	IA_CSS_IRQ_INFO_IF_ERROR

#define CSS_BUFFER_TYPE_NUM	IA_CSS_BUFFER_TYPE_NUM

#define CSS_FRAME_FLASH_STATE_NONE	IA_CSS_FRAME_FLASH_STATE_NONE
#define CSS_FRAME_FLASH_STATE_PARTIAL	IA_CSS_FRAME_FLASH_STATE_PARTIAL
#define CSS_FRAME_FLASH_STATE_FULL	IA_CSS_FRAME_FLASH_STATE_FULL

#define CSS_BAYER_ORDER_GRBG	IA_CSS_BAYER_ORDER_GRBG
#define CSS_BAYER_ORDER_RGGB	IA_CSS_BAYER_ORDER_RGGB
#define CSS_BAYER_ORDER_BGGR	IA_CSS_BAYER_ORDER_BGGR
#define CSS_BAYER_ORDER_GBRG	IA_CSS_BAYER_ORDER_GBRG

/*
 * Hide IA_ naming difference in otherwise common CSS macros.
 */
#define CSS_ID(val)	(IA_ ## val)
#define CSS_EVENT(val)	(IA_CSS_EVENT_TYPE_ ## val)
#define CSS_FORMAT(val)	(IA_CSS_STREAM_FORMAT_ ## val)

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

/*
 * These are used to indicate the css stream state, corresponding
 * stream handling can be done via judging the different state.
 */
enum atomisp_css_stream_state {
	CSS_STREAM_UNINIT,
	CSS_STREAM_CREATED,
	CSS_STREAM_STARTED,
	CSS_STREAM_STOPPED,
};

struct atomisp_css_buffer {
	struct ia_css_buffer css_buffer;
};

struct atomisp_css_event {
	enum atomisp_css_pipe_id pipe;
	struct ia_css_event event;
};

#endif
