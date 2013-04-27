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

#ifndef __ATOMISP_COMPAT_CSS15_H__
#define __ATOMISP_COMPAT_CSS15_H__

#include "sh_css.h"
#include "sh_css_sp.h"

#define atomisp_css_pipe_id sh_css_pipe_id
#define atomisp_css_buffer_type sh_css_buffer_type
#define atomisp_css_dis_data sh_css_dis_data
#define atomisp_css_irq_info sh_css_interrupt_info
typedef union sh_css_s3a_data atomisp_css_3a_data;

#define CSS_PIPE_ID_PREVIEW	SH_CSS_PREVIEW_PIPELINE
#define CSS_PIPE_ID_COPY	SH_CSS_COPY_PIPELINE
#define CSS_PIPE_ID_VIDEO	SH_CSS_VIDEO_PIPELINE
#define CSS_PIPE_ID_CAPTURE	SH_CSS_CAPTURE_PIPELINE
#define CSS_PIPE_ID_ACC		SH_CSS_ACC_PIPELINE
#define CSS_PIPE_ID_NUM		SH_CSS_NR_OF_PIPELINES

#define CSS_IRQ_INFO_EVENTS_READY	SH_CSS_IRQ_INFO_BUFFER_DONE
#define CSS_IRQ_INFO_INVALID_FIRST_FRAME \
	SH_CSS_IRQ_INFO_INVALID_FIRST_FRAME

#if defined(HAS_IRQ_MAP_VERSION_2)

#define CSS_IRQ_INFO_INPUT_SYSTEM_ERROR	SH_CSS_IRQ_INFO_INPUT_SYSTEM_ERROR
#define CSS_IRQ_INFO_IF_ERROR	SH_CSS_IRQ_INFO_IF_ERROR

#elif defined(HAS_IRQ_MAP_VERSION_1) || defined(HAS_IRQ_MAP_VERSION_1_DEMO)

#define CSS_IRQ_INFO_CSS_RECEIVER_ERROR	SH_CSS_IRQ_INFO_CSS_RECEIVER_ERROR

#endif

#define CSS_BUFFER_TYPE_NUM	SH_CSS_BUFFER_TYPE_NR_OF_TYPES

#define CSS_FRAME_FLASH_STATE_NONE	SH_CSS_FRAME_NO_FLASH
#define CSS_FRAME_FLASH_STATE_PARTIAL	SH_CSS_FRAME_PARTIAL_FLASH
#define CSS_FRAME_FLASH_STATE_FULL	SH_CSS_FRAME_FULLY_FLASH

/*
 * Hide SH_ naming difference in otherwise common CSS macros.
 */
#define CSS_ID(val)	(SH_ ## val)
#define CSS_EVENT(val)	(SH_CSS_EVENT_ ## val)

struct atomisp_css_env {
	struct sh_css_env isp_css_env;
};

struct atomisp_s3a_buf {
	atomisp_css_3a_data s3a_data;
	struct list_head list;
};

struct atomisp_dis_buf {
	struct atomisp_css_dis_data dis_data;
	struct list_head list;
};

struct atomisp_css_buffer {
	struct sh_css_buffer css_buffer;
	struct atomisp_css_dis_data *dis_data;
	atomisp_css_3a_data *s3a_data;
};

struct atomisp_css_event {
	enum atomisp_css_pipe_id pipe;
	enum sh_css_event_type event;
};

void atomisp_css_mmu_invalidate_tlb(void);

#endif
