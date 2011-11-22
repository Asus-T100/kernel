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

/* This module provides access to the CSS hardware. This includes access to
   local memories, registers and starting processors. */

#ifndef _SH_CSS_HRT_H_
#define _SH_CSS_HRT_H_

#include <hmm/hmm.h>

/* here we include some system header files that provide constants and such
   about the system and the ISP processor. */
#include <hive_isp_css_host_ids_hrt.h>
#include <hive_isp_css_defs.h>
#include <isp2300_medfield_params.h>
#include <scalar_processor_params.h>
#include <gdc_defs.h>

/* The following files provide access to types and memory related functions
   that contain system specific behavior. */
#include <hive_isp_css_irq_types_hrt.h>
#include <hive_isp_css_mm_hrt.h>

#include "sh_css_types.h"
#include "sh_css_rx.h"
#include "sh_css_sp_start.h"

enum sh_css_fifo_channel {
	/* SP <-> ISP */
	sh_css_hrt_fifo_isp_to_sp,
	sh_css_hrt_fifo_sp_to_isp,
	/* ISP <-> PRIM IF A */
	sh_css_hrt_fifo_isp_to_if_prim_a,
	sh_css_hrt_fifo_if_prim_a_to_isp,
	/* ISP <-> PRIM IF B (only ISP side) */
	sh_css_hrt_fifo_isp_to_if_prim_b,
	sh_css_hrt_fifo_if_prim_b_to_isp,
	/* ISP <-> DMA */
	sh_css_hrt_fifo_isp_to_dma,
	sh_css_hrt_fifo_dma_to_isp,
	/* SP <-> DMA */
	sh_css_hrt_fifo_sp_to_dma,
	sh_css_hrt_fifo_dma_to_sp,
	/* ISP <-> GDC */
	sh_css_hrt_fifo_isp_to_gdc,
	sh_css_hrt_fifo_gdc_to_isp,
};

enum sh_css_dma_ctrl_state {
	sh_css_dma_ctrl_state_idle,
	sh_css_dma_ctrl_state_req_rcv,
	sh_css_dma_ctrl_state_rcv,
	sh_css_dma_ctrl_state_rcv_req,
	sh_css_dma_ctrl_state_init
};

enum sh_css_dma_command {
	sh_css_dma_command_read,
	sh_css_dma_command_write,
	sh_css_dma_command_set_channel,
	sh_css_dma_command_set_param,
	sh_css_dma_command_read_spec,
	sh_css_dma_command_write_spec,
	sh_css_dma_command_init,
	sh_css_dma_command_init_spec,
	sh_css_dma_command_reset
};

enum sh_css_dma_rw_state {
	sh_css_dma_rw_state_idle,
	sh_css_dma_rw_state_req,
	sh_css_dma_rw_state_next_line,
	sh_css_dma_rw_state_unlock_channel
};

enum sh_css_dma_fifo_state {
	sh_css_dma_fifo_state_will_be_full,
	sh_css_dma_fifo_state_full,
	sh_css_dma_fifo_state_empty
};

struct sh_css_dma_port_state {
	bool                       req_cs;
	bool                       req_we_n;
	bool                       req_run;
	bool                       req_ack;
	bool                       send_cs;
	bool                       send_we_n;
	bool                       send_run;
	bool                       send_ack;
	enum sh_css_dma_fifo_state fifo_state;
	int                        fifo_counter;
};

struct sh_css_dma_channel_state {
	int  connection;
	bool sign_extend;
	bool reverse_elem_order;
	int  height;
	int  stride_a;
	int  elems_a;
	int  cropping_a;
	int  width_a;
	int  stride_b;
	int  elems_b;
	int  cropping_b;
	int  width_b;
};

struct sh_css_dma_state {
	bool                       fsm_command_idle;
	bool                       fsm_command_run;
	bool                       fsm_command_stalling;
	bool                       fsm_command_error;
	enum sh_css_dma_command    last_command;
	int                        last_command_channel;
	int                        last_command_param;
	enum sh_css_dma_command    current_command;
	int                        current_addr_a;
	int                        current_addr_b;
	bool                       fsm_ctrl_idle;
	bool                       fsm_ctrl_run;
	bool                       fsm_ctrl_stalling;
	bool                       fsm_ctrl_error;
	enum sh_css_dma_ctrl_state fsm_ctrl_state;
	int                        fsm_ctrl_source_dev;
	int                        fsm_ctrl_source_addr;
	int                        fsm_ctrl_source_stride;
	int                        fsm_ctrl_source_width;
	int                        fsm_ctrl_source_height;
	int                        fsm_ctrl_pack_source_dev;
	int                        fsm_ctrl_pack_dest_dev;
	int                        fsm_ctrl_dest_addr;
	int                        fsm_ctrl_dest_stride;
	int                        fsm_ctrl_pack_source_width;
	int                        fsm_ctrl_pack_dest_height;
	int                        fsm_ctrl_pack_dest_width;
	int                        fsm_ctrl_pack_source_elems;
	int                        fsm_ctrl_pack_dest_elems;
	int                        fsm_ctrl_pack_extension;
	int			   pack_idle;
	int			   pack_run;
	int			   pack_stalling;
	int			   pack_error;
	int                        pack_cnt_height;
	int                        pack_src_cnt_width;
	int                        pack_dest_cnt_width;
	enum sh_css_dma_rw_state   read_state;
	int                        read_cnt_height;
	int                        read_cnt_width;
	enum sh_css_dma_rw_state   write_state;
	int                        write_height;
	int                        write_width;
	struct sh_css_dma_port_state    port_states[3];
	struct sh_css_dma_channel_state channel_states[8];
};

struct sh_css_if_state {
	int reset,
	    start_line,
	    start_column,
	    cropped_height,
	    cropped_width,
	    ver_decimation,
	    hor_decimation,
	    deinterleaving,
	    left_padding,
	    eol_offset,
	    vmem_start_address,
	    vmem_end_address,
	    vmem_increment,
	    yuv420,
	    vsync_active_low,
	    hsync_active_low,
	    allow_fifo_overflow,
	    fsm_sync_status,
	    fsm_sync_counter,
	    fsm_crop_status,
	    fsm_crop_line_counter,
	    fsm_crop_pixel_counter,
	    fsm_deinterleaving_index,
	    fsm_dec_h_counter,
	    fsm_dec_v_counter,
	    fsm_dec_block_v_counter,
	    fsm_padding_status,
	    fsm_padding_elem_counter,
	    fsm_vector_support_error,
	    fsm_vector_buffer_full,
	    vector_support,
	    sensor_data_lost;
};

struct sh_css_cell_state {
	int    pc;
	int    status_register;
	bool is_broken;
	bool is_idle;
	bool is_sleeping;
	bool is_stalling;
};

struct sh_css_sp_stall_state {
	bool fifo0;
	bool fifo1;
	bool fifo2;
	bool fifo3;
	bool fifo4;
	bool fifo5;
	bool fifo6;
	bool fifo7;
	bool dmem;
	bool control_master;
	bool icache_master;
};

struct sh_css_isp_stall_state {
	bool fifo0;
	bool fifo1;
	bool fifo2;
	bool fifo3;
	bool fifo4;
	bool fifo5;
	bool stat_ctrl;
	bool dmem;
	bool vmem;
	bool vamem1;
	bool vamem2;
};

struct sh_css_fifo_channel_state {
	bool src_valid;
	bool fifo_accept;
	bool fifo_valid;
	bool sink_accept;
};

void
sh_css_sp_ctrl_store(unsigned int reg, unsigned int value);

void
sh_css_sp_ctrl_set_bits(unsigned int reg, unsigned long bits);

/* SP access */
void
sh_css_hrt_sp_start_si(void);

void
sh_css_hrt_sp_start_histogram(void);

void
sh_css_hrt_sp_start_copy_frame(void);

void
sh_css_hrt_sp_start_copy_binary_data(void);

void
sh_css_hrt_sp_start_copy_raw_data(void);

void
sh_css_hrt_sp_start_isp(void);

enum sh_css_err
sh_css_hrt_sp_wait(void);

bool
sh_css_hrt_sp_is_idle(void);

bool
sh_css_hrt_sp_is_stalling(void);

unsigned int
sh_css_hrt_sp_current_pc(void);

unsigned int
sh_css_hrt_sp_current_msink(void);

void
sh_css_hrt_sp_get_state(struct sh_css_cell_state *state,
			struct sh_css_sp_stall_state *stall_state);

/* ISP access */
bool
sh_css_hrt_isp_is_idle(void);

bool
sh_css_hrt_isp_is_stalling(void);

void
sh_css_hrt_isp_get_state(struct sh_css_cell_state *state,
			 struct sh_css_isp_stall_state *stall_state);

void
sh_css_hrt_fifo_channel_get_state(enum sh_css_fifo_channel,
				  struct sh_css_fifo_channel_state *state);

unsigned int
sh_css_hrt_isp_current_pc(void);

void
sh_css_isp_ctrl_store(unsigned int reg, unsigned int value);

unsigned int
sh_css_hrt_isp_current_msink(void);

unsigned int
sh_css_hrt_isp_current_sc(void);

/* Input formatters */
unsigned int
sh_css_hrt_if_prim_vec_align(void);

void
sh_css_hrt_if_prim_a_get_state(struct sh_css_if_state *state);

void
sh_css_hrt_if_prim_b_get_state(struct sh_css_if_state *state);

void
sh_css_hrt_if_reset(void);

void
sh_css_hrt_if_set_block_fifo_no_reqs(bool enable_prim, bool enable_prim_b);

bool
sh_css_hrt_system_is_idle(void);

void
sh_css_hrt_dma_get_state(struct sh_css_dma_state *state);

/* Interrupts */
void
sh_css_hrt_irq_enable(enum hrt_isp_css_irq irq,
		      bool rising_edge_in,
		      bool edge_not_pulse_out);

void
sh_css_hrt_irq_enable_sp(bool enable);

void
sh_css_hrt_irq_disable(enum hrt_isp_css_irq irq);

void
sh_css_hrt_irq_clear_all(void);

enum hrt_isp_css_irq_status
sh_css_hrt_irq_get_id(enum hrt_isp_css_irq *irq);

void
sh_css_hrt_irq_clear_sp(void);

/* GDC */
void
sh_css_hrt_gdc_set_lut(const int data[4][HRT_GDC_N]);

/* MMU */
void
sh_css_hrt_mmu_set_page_table_base_address(void *base_address);

void *
sh_css_hrt_mmu_get_page_table_base_address(void);

void
sh_css_hrt_mmu_invalidate_cache(void);

void
sh_css_hrt_send_input_frame(unsigned short *data,
			    unsigned int width,
			    unsigned int height,
			    unsigned int ch_id,
			    enum sh_css_input_format input_format,
			    bool two_ppc);

void
sh_css_hrt_streaming_to_mipi_start_frame(unsigned int ch_id,
				enum sh_css_input_format input_format,
				bool two_ppc);

void
sh_css_hrt_streaming_to_mipi_send_line(unsigned int ch_id,
						unsigned short *data,
						unsigned int width,
						unsigned short *data2,
						unsigned int width2);

void
sh_css_hrt_streaming_to_mipi_end_frame(unsigned int ch_id);


#endif /* _SH_CSS_HRT_H_ */
