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

#ifndef _SH_CSS_RX_H_
#define _SH_CSS_RX_H_

#include "input_system.h"

/* CSS Receiver */

void sh_css_rx_configure(
	const rx_cfg_t	*config,
	const enum ia_css_input_mode input_mode);

void sh_css_rx_disable(void);

void sh_css_rx_enable_all_interrupts(void);

unsigned int sh_css_rx_get_interrupt_reg(void);

/** @brief Translate format and compression to format type.
 *
 * @param[in]	input_format	The input format.
 * @param[in]	compression	The compression scheme.
 * @param[out]	fmt_type	Pointer to the resulting format type.
 * @return			Error code.
 *
 * Translate an input format and mipi compression pair to the fmt_type.
 * This is normally done by the sensor, but when using the input fifo, this
 * format type must be sumitted correctly by the application.
 */
enum ia_css_err sh_css_input_format_type(
	enum ia_css_stream_format input_format,
	mipi_predictor_t	compression,
	unsigned int		*fmt_type);

#endif /* _SH_CSS_RX_H_ */
