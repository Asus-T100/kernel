/* Release Version: ci_master_byt_20130905_2200 */
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

#ifndef _SH_CSS_HRT_H_
#define _SH_CSS_HRT_H_

#include <sp.h>
#include <isp.h>

#include "ia_css.h"	/* enum ia_css_stream_format */

/* SP access */
void sh_css_hrt_sp_start_si(void);

void sh_css_hrt_sp_start_copy_frame(void);

void sh_css_hrt_sp_start_isp(void);

enum ia_css_err sh_css_hrt_sp_wait(void);

bool sh_css_hrt_system_is_idle(void);

void sh_css_hrt_send_input_frame(
	const unsigned short	*data,
	unsigned int	width,
	unsigned int	height,
	unsigned int	ch_id,
	enum ia_css_stream_format	input_format,
	bool			two_ppc);

void sh_css_hrt_streaming_to_mipi_start_frame(
	unsigned int	ch_id,
	enum ia_css_stream_format	input_format,
	bool			two_ppc);

void sh_css_hrt_streaming_to_mipi_send_line(
	unsigned int	ch_id,
	const unsigned short	*data,
	unsigned int	width,
	const unsigned short	*data2,
	unsigned int	width2);

void sh_css_hrt_streaming_to_mipi_end_frame(
	unsigned int	ch_id);

#endif /* _SH_CSS_HRT_H_ */
