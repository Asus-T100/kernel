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

#include "ia_css.h"
#include "sh_css_sp.h"

#ifdef ATOMISP_CSS2
#define ia_css_sp_has_booted() ia_css_sp_has_initialized()

//static inline enum sh_css_err sh_css_allocate_continuous_frames(bool enable)
//{
	//return sh_css_err_unsupported_configuration;
//}
enum ia_css_err ia_css_preview_configure_output(
	struct atomisp_device *isp,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_preview_get_output_frame_info(
	struct atomisp_device *isp,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_capture_configure_output(
	struct atomisp_device *isp,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_capture_get_output_frame_info(
	struct atomisp_device *isp,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_capture_configure_viewfinder(
	struct atomisp_device *isp,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_capture_get_viewfinder_frame_info(
	struct atomisp_device *isp,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_video_configure_output(
	struct atomisp_device *isp,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_video_get_output_frame_info(
	struct atomisp_device *isp,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_video_configure_viewfinder(
	struct atomisp_device *isp,
	unsigned int width,
	unsigned int height,
	enum ia_css_frame_format format);

enum ia_css_err ia_css_video_get_viewfinder_frame_info(
	struct atomisp_device *isp,
	struct ia_css_frame_info *info);

enum ia_css_err ia_css_preview_configure_pp_input(
	struct atomisp_device *isp,
	unsigned int width,
	unsigned int height);

enum ia_css_err ia_css_capture_configure_pp_input(
	struct atomisp_device *isp,
	unsigned int width,
	unsigned int height);
void
ia_css_capture_set_mode(struct atomisp_device *isp,
			enum ia_css_capture_mode mode);

void
ia_css_capture_enable_online(struct atomisp_device *isp,
			     bool enable);

void
ia_css_enable_raw_binning(struct atomisp_device *isp,
			     bool enable);
void
ia_css_input_set_two_pixels_per_clock(struct atomisp_device *isp,
					   bool enable);

void ia_css_preview_enable_online(struct atomisp_device *isp,
				  bool enable);
void ia_css_enable_continuous(struct atomisp_device *isp,
				  bool enable);
enum ia_css_err ia_css_capture_get_output_raw_frame_info(
	struct atomisp_device *isp,
	struct ia_css_frame_info *info);

void atomisp_sh_css_mmu_set_page_table_base_index(unsigned int base_index);
enum ia_css_err ia_css_stop(struct atomisp_device *isp, bool need_reset);
enum ia_css_err ia_css_start(struct atomisp_device *isp, bool need_reset);
void ia_css_disable_vf_pp(struct atomisp_device *isp, bool disable);
void ia_css_video_set_dis_envelope(struct atomisp_device *isp,
			      unsigned int dvs_w,
			      unsigned int dvs_h);
void ia_css_input_set_effective_resolution(struct atomisp_device *isp,
			unsigned int width, unsigned int height);
void ia_css_input_set_resolution(struct atomisp_device *isp,
			unsigned int width, unsigned int height);
void ia_css_input_set_binning_factor(struct atomisp_device *isp, int binning);
void ia_css_input_set_bayer_order(struct atomisp_device *isp,
			     enum ia_css_bayer_order order);
void ia_css_input_set_format(struct atomisp_device *isp,
			     enum ia_css_stream_format format);
void
ia_css_input_configure_port(struct atomisp_device *isp,
			    const mipi_port_ID_t port,
			    const unsigned int	 num_lanes,
			    const unsigned int	 timeout);
void
ia_css_input_set_mode(struct atomisp_device *isp,
		      enum ia_css_input_mode mode);

void
ia_css_enbale_dz(struct atomisp_device *isp, bool enable);

#endif
