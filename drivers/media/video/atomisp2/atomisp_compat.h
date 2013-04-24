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

#include <media/videobuf-vmalloc.h>

#define CSS_RX_IRQ_INFO_BUFFER_OVERRUN \
	CSS_ID(CSS_RX_IRQ_INFO_BUFFER_OVERRUN)
#define CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE \
	CSS_ID(CSS_RX_IRQ_INFO_ENTER_SLEEP_MODE)
#define CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE \
	CSS_ID(CSS_RX_IRQ_INFO_EXIT_SLEEP_MODE)
#define CSS_RX_IRQ_INFO_ECC_CORRECTED \
	CSS_ID(CSS_RX_IRQ_INFO_ECC_CORRECTED)
#define CSS_RX_IRQ_INFO_ERR_SOT \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_SOT)
#define CSS_RX_IRQ_INFO_ERR_SOT_SYNC \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_SOT_SYNC)
#define CSS_RX_IRQ_INFO_ERR_CONTROL \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_CONTROL)
#define CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_ECC_DOUBLE)
#define CSS_RX_IRQ_INFO_ERR_CRC \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_CRC)
#define CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_UNKNOWN_ID)
#define CSS_RX_IRQ_INFO_ERR_FRAME_SYNC \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_FRAME_SYNC)
#define CSS_RX_IRQ_INFO_ERR_FRAME_DATA \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_FRAME_DATA)
#define CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_DATA_TIMEOUT)
#define CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_UNKNOWN_ESC)
#define CSS_RX_IRQ_INFO_ERR_LINE_SYNC \
	CSS_ID(CSS_RX_IRQ_INFO_ERR_LINE_SYNC)
#define CSS_RX_IRQ_INFO_INIT_TIMEOUT \
	CSS_ID(CSS_RX_IRQ_INFO_INIT_TIMEOUT)

#define CSS_IRQ_INFO_CSS_RECEIVER_SOF	CSS_ID(CSS_IRQ_INFO_CSS_RECEIVER_SOF)
#define CSS_IRQ_INFO_CSS_RECEIVER_EOF	CSS_ID(CSS_IRQ_INFO_CSS_RECEIVER_EOF)

#define CSS_EVENT_OUTPUT_FRAME_DONE	CSS_EVENT(OUTPUT_FRAME_DONE)
#define CSS_EVENT_VF_OUTPUT_FRAME_DONE	CSS_EVENT(VF_OUTPUT_FRAME_DONE)
#define CSS_EVENT_3A_STATISTICS_DONE	CSS_EVENT(3A_STATISTICS_DONE)
#define CSS_EVENT_DIS_STATISTICS_DONE	CSS_EVENT(DIS_STATISTICS_DONE)
#define CSS_EVENT_PIPELINE_DONE		CSS_EVENT(PIPELINE_DONE)

#define CSS_BUFFER_TYPE_3A_STATISTICS	CSS_ID(CSS_BUFFER_TYPE_3A_STATISTICS)
#define CSS_BUFFER_TYPE_DIS_STATISTICS	CSS_ID(CSS_BUFFER_TYPE_DIS_STATISTICS)
#define CSS_BUFFER_TYPE_INPUT_FRAME	CSS_ID(CSS_BUFFER_TYPE_INPUT_FRAME)
#define CSS_BUFFER_TYPE_OUTPUT_FRAME	CSS_ID(CSS_BUFFER_TYPE_OUTPUT_FRAME)
#define CSS_BUFFER_TYPE_VF_OUTPUT_FRAME	CSS_ID(CSS_BUFFER_TYPE_VF_OUTPUT_FRAME)
#define CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME \
	CSS_ID(CSS_BUFFER_TYPE_RAW_OUTPUT_FRAME)

#define CSS_FORMAT_RAW_8	CSS_FORMAT(RAW_8)
#define CSS_FORMAT_RAW_10	CSS_FORMAT(RAW_10)
#define CSS_FORMAT_RAW_12	CSS_FORMAT(RAW_12)

#define CSS_CAPTURE_MODE_RAW		CSS_ID(CSS_CAPTURE_MODE_RAW)
#define CSS_CAPTURE_MODE_BAYER		CSS_ID(CSS_CAPTURE_MODE_BAYER)
#define CSS_CAPTURE_MODE_PRIMARY	CSS_ID(CSS_CAPTURE_MODE_PRIMARY)
#define CSS_CAPTURE_MODE_ADVANCED	CSS_ID(CSS_CAPTURE_MODE_ADVANCED)
#define CSS_CAPTURE_MODE_LOW_LIGHT	CSS_ID(CSS_CAPTURE_MODE_LOW_LIGHT)

struct atomisp_device;

void atomisp_set_css_env(struct atomisp_device *isp);

int atomisp_css_init(struct atomisp_device *isp);

void atomisp_css_uninit(struct atomisp_device *isp);

void atomisp_css_suspend(void);

int atomisp_css_resume(struct atomisp_device *isp);

void atomisp_css_init_struct(struct atomisp_device *isp);

int atomisp_css_irq_translate(struct atomisp_device *isp,
			      unsigned int *infos);

void atomisp_css_rx_get_irq_info(unsigned int *infos);

void atomisp_css_rx_clear_irq_info(unsigned int infos);

int atomisp_css_irq_enable(struct atomisp_device *isp,
			   enum atomisp_css_irq_info info, bool enable);

int atomisp_q_video_buffer_to_css(struct atomisp_device *isp,
			struct videobuf_vmalloc_memory *vm_mem,
			enum atomisp_css_buffer_type css_buf_type,
			enum atomisp_css_pipe_id css_pipe_id);

int atomisp_q_s3a_buffer_to_css(struct atomisp_device *isp,
			struct atomisp_s3a_buf *s3a_buf,
			enum atomisp_css_pipe_id css_pipe_id);

int atomisp_q_dis_buffer_to_css(struct atomisp_device *isp,
			struct atomisp_dis_buf *dis_buf,
			enum atomisp_css_pipe_id css_pipe_id);

void atomisp_css_mmu_invalidate_cache(void);

int atomisp_css_start(struct atomisp_device *isp,
		      enum atomisp_css_pipe_id pipe_id, bool in_reset);

void atomisp_css_update_isp_params(struct atomisp_device *isp);

int atomisp_css_queue_buffer(struct atomisp_device *isp,
			     enum atomisp_css_pipe_id pipe_id,
			     enum atomisp_css_buffer_type buf_type,
			     struct atomisp_css_buffer *isp_css_buffer);

int atomisp_css_dequeue_buffer(struct atomisp_device *isp,
				enum atomisp_css_pipe_id pipe_id,
				enum atomisp_css_buffer_type buf_type,
				struct atomisp_css_buffer *isp_css_buffer);

int atomisp_css_get_3a_statistics(struct atomisp_device *isp,
				  struct atomisp_css_buffer *isp_css_buffer);

void atomisp_css_get_dis_statistics(struct atomisp_device *isp,
				    struct atomisp_css_buffer *isp_css_buffer);

int atomisp_css_dequeue_event(struct atomisp_css_event *current_event);

int atomisp_css_input_set_resolution(struct atomisp_device *isp,
					struct v4l2_mbus_framefmt *ffmt);

void atomisp_css_input_set_binning_factor(struct atomisp_device *isp,
						unsigned int bin_factor);

void atomisp_css_input_set_bayer_order(struct atomisp_device *isp,
				enum atomisp_css_bayer_order bayer_order);

void atomisp_css_input_set_format(struct atomisp_device *isp,
					enum atomisp_css_stream_format format);

int atomisp_css_input_set_effective_resolution(struct atomisp_device *isp,
				unsigned int width, unsigned int height);

void atomisp_css_video_set_dis_envelope(struct atomisp_device *isp,
					unsigned int dvs_w, unsigned int dvs_h);

void atomisp_css_input_set_two_pixels_per_clock(struct atomisp_device *isp,
						bool two_ppc);

void atomisp_css_enable_raw_binning(struct atomisp_device *isp,
					bool enable);

void atomisp_css_capture_set_mode(struct atomisp_device *isp,
				enum atomisp_css_capture_mode mode);

void atomisp_css_input_set_mode(struct atomisp_device *isp,
				enum atomisp_css_input_mode mode);
#endif
