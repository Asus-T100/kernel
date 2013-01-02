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

#ifndef	__ATOMISP_FOPS_H__
#define	__ATOMISP_FOPS_H__

int atomisp_q_video_buffers_to_css(struct atomisp_device *isp,
			     struct atomisp_video_pipe *pipe,
			     enum sh_css_buffer_type css_buf_type,
			     enum sh_css_pipe_id css_pipe_id);

int atomisp_q_s3a_buffers_to_css(struct atomisp_device *isp,
			   enum sh_css_pipe_id css_pipe_id);

int atomisp_q_dis_buffers_to_css(struct atomisp_device *isp,
			   enum sh_css_pipe_id css_pipe_id);

/*
 * Videobuf ops
 */
int atomisp_buf_setup(struct videobuf_queue *vq,
			unsigned int *count,
			unsigned int *size);

int atomisp_buf_prepare(struct videobuf_queue *vq,
			  struct videobuf_buffer *vb,
			  enum v4l2_field field);

void atomisp_buf_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb);

void atomisp_buf_release(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb);

int atomisp_init_struct(struct atomisp_device *isp);

unsigned int atomisp_users(struct atomisp_device *isp);

/*
 * Memory help functions for image frame and private parameters
 */

int atomisp_videobuf_mmap_mapper(struct videobuf_queue *q,
				     struct vm_area_struct *vma);

int atomisp_file_mmap(struct file *file, struct vm_area_struct *vma);

int atomisp_qbuf_to_css(struct atomisp_device *isp,
			struct atomisp_video_pipe *pipe,
			struct videobuf_buffer *vb);

int atomisp_qbuffers_to_css(struct atomisp_device *isp);

extern const struct v4l2_file_operations atomisp_fops;

#endif /* __ATOMISP_FOPS_H__ */
