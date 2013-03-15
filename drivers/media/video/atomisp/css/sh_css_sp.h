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

#ifndef _SH_CSS_SP_H_
#define _SH_CSS_SP_H_

#include "sh_css_binary.h"
#include "sh_css_internal.h"

struct sh_css_sp_frame_data {
	const char *sp_group_addr;	/* Address of sp_group in ddr */
	unsigned read_sp_group_from_ddr;
	const char *bin_copy_out;
	unsigned bin_copy_bytes_available;
};

/* Function to initialize the data and bss section descr of the binary */
void
sh_css_sp_store_init_dmem(const struct sh_css_sp_fw *fw);

enum sh_css_err
sh_css_sp_init(void);

void
sh_css_sp_uninit(void);

void
sh_css_sp_start_histogram(struct sh_css_histogram *histogram,
			  const struct sh_css_frame *frame);

/* Start binary (jpeg) copy on the SP */
void
sh_css_sp_start_binary_copy(struct sh_css_frame *out_frame,
			    unsigned two_ppc);

/* Start raw copy on the SP */
void
sh_css_sp_start_raw_copy(struct sh_css_binary *binary,
			 struct sh_css_frame *out_frame,
			 unsigned two_ppc,
			 bool start);

unsigned int
sh_css_sp_get_binary_copy_size(void);

/* Return the value of a SW interrupt */
unsigned int
sh_css_sp_get_sw_interrupt_value(void);

enum sh_css_err
sh_css_sp_start_isp(struct sh_css_binary *binary,
		    const struct sh_css_binary_args *args,
		    bool preview_mode,
		    bool low_light);

void
sh_css_sp_get_debug_state(struct sh_css_sp_debug_state *state);

void
sh_css_sp_set_overlay(const struct sh_css_overlay *overlay);

void
sh_css_sp_set_if_configs(const struct sh_css_if_config *config_a,
			 const struct sh_css_if_config *config_b);

void
sh_css_sp_program_input_circuit(int fmt_type,
				int ch_id,
				enum sh_css_input_mode input_mode);

void
sh_css_sp_configure_sync_gen(int width,
			     int height,
			     int hblank_cycles,
			     int vblank_cycles);

void
sh_css_sp_configure_tpg(int x_mask,
			int y_mask,
			int x_delta,
			int y_delta,
			int xy_mask);

void
sh_css_sp_configure_prbs(int seed);

void
sh_css_store_sp_per_frame_data(void);

extern struct sh_css_sp_group sh_css_sp_group;
#endif /* _SH_CSS_SP_H_ */
