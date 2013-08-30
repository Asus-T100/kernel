/* Release Version: ci_master_byt_20130820_2200 */
/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2013 Intel Corporation. All Rights Reserved.
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

#ifndef _IA_CSS_STREAM_H_
#define _IA_CSS_STREAM_H_

#include <input_system.h>
#include "ia_css.h"
#include "sh_css_params.h"
/**
 * structure to hold all internal stream related information
 */
struct ia_css_stream {
	struct ia_css_stream_config    config;
	struct ia_css_stream_info      info;
	rx_cfg_t                       csi_rx_config;
	bool                           reconfigure_css_rx;
	struct ia_css_pipe            *last_pipe;
	int                            num_pipes;
	struct ia_css_pipe           **pipes;
	struct sh_css_pipe            *continuous_pipe;
	struct ia_css_isp_parameters  *isp_params_configs;
	bool                           continuous;
	bool                           cont_capt;
};

struct sh_css_binary *
ia_css_stream_get_dvs_binary(const struct ia_css_stream *stream);

struct sh_css_binary *
ia_css_stream_get_3a_binary(const struct ia_css_stream *stream);

unsigned int 
ia_css_stream_input_format_bits_per_pixel(struct ia_css_stream *stream);

bool
sh_css_params_set_binning_factor(struct ia_css_stream *stream, unsigned int sensor_binning);

enum ia_css_err
sh_css_params_write_to_ddr(struct ia_css_stream *stream,
			   const struct sh_css_binary *binary_info);

void
sh_css_param_update_isp_params(struct ia_css_stream *stream, bool commit);

void
sh_css_invalidate_params(struct ia_css_stream *stream);

/* The following functions are used for testing purposes only */
const struct ia_css_fpn_table *
ia_css_get_fpn_table(struct ia_css_stream *stream);

struct ia_css_shading_table *
ia_css_get_shading_table(struct ia_css_stream *stream);

const struct sh_css_isp_params *
ia_css_get_isp_params(struct ia_css_stream *stream);

void
ia_css_get_isp_dis_coefficients(struct ia_css_stream *stream,
				short *horizontal_coefficients,
				short *vertical_coefficients);

void
ia_css_get_isp_dvs2_coefficients(struct ia_css_stream *stream,
	short *hor_coefs_odd_real,
	short *hor_coefs_odd_imag,
	short *hor_coefs_even_real,
	short *hor_coefs_even_imag,
	short *ver_coefs_odd_real,
	short *ver_coefs_odd_imag,
	short *ver_coefs_even_real,
	short *ver_coefs_even_imag);

enum ia_css_err
ia_css_stream_isp_parameters_init(struct ia_css_stream *stream);

void
ia_css_stream_isp_parameters_uninit(struct ia_css_stream *stream);

#endif /*_IA_CSS_STREAM_H_*/
