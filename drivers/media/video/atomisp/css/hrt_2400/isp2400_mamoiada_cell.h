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

#ifndef _isp2400_mamoiada_cell_h
#define _isp2400_mamoiada_cell_h

//#include <hrt/api.h>

#define isp2400_mamoiada_prog_mem_slave_port  isp2400_mamoiada_sl_ippmem_cell_port
#define isp2400_mamoiada_prog_mem_width       640
#define isp2400_mamoiada_prog_mem_base        0x0
#define isp2400_mamoiada_stat_ctrl_slave_port isp2400_mamoiada_sl_ipstatctrl_cell_port
#define isp2400_mamoiada_stat_ctrl_base       0x0
#define isp2400_mamoiada_view_table_base      0x0
#define isp2400_mamoiada_num_views            1
#define isp2400_mamoiada_default_mem          isp2400_mamoiada_base_dmem
#define isp2400_mamoiada_int_size             4
#define isp2400_mamoiada_char_bits            8
#define isp2400_mamoiada_uchar_bits           isp2400_mamoiada_char_bits
#define isp2400_mamoiada_short_bits           16
#define isp2400_mamoiada_ushort_bits           isp2400_mamoiada_short_bits
#define isp2400_mamoiada_int_bits             32
#define isp2400_mamoiada_uint_bits            isp2400_mamoiada_int_bits
#define isp2400_mamoiada_long_bits            32
#define isp2400_mamoiada_ulong_bits           isp2400_mamoiada_long_bits
#define isp2400_mamoiada_ptr_bits             32

#define isp2400_mamoiada_start_address_register          0x1
#define isp2400_mamoiada_break_address_register          0x2
#define isp2400_mamoiada_debug_pc_register               0x7
#define isp2400_mamoiada_reset_flag_register             0x0
#define isp2400_mamoiada_reset_flag_bit                  0x0
#define isp2400_mamoiada_start_flag_register             0x0
#define isp2400_mamoiada_start_flag_bit                  0x1
#define isp2400_mamoiada_break_flag_register             0x0
#define isp2400_mamoiada_break_flag_bit                  0x2
#define isp2400_mamoiada_run_flag_register               0x0
#define isp2400_mamoiada_run_flag_bit                    0x3
#define isp2400_mamoiada_broken_flag_register            0x0
#define isp2400_mamoiada_broken_flag_bit                 0x4
#define isp2400_mamoiada_ready_flag_register             0x0
#define isp2400_mamoiada_ready_flag_bit                  0x5
#define isp2400_mamoiada_sleeping_flag_register          0x0
#define isp2400_mamoiada_sleeping_flag_bit               0x6
#define isp2400_mamoiada_stalling_flag_register          0x0
#define isp2400_mamoiada_stalling_flag_bit               0x7
#define isp2400_mamoiada_irq_clr_flag_register           0x0
#define isp2400_mamoiada_irq_clr_flag_bit                0x8
#define isp2400_mamoiada_broken_irq_mask_flag_register   0x0
#define isp2400_mamoiada_broken_irq_mask_flag_bit        0x9
#define isp2400_mamoiada_ready_irq_mask_flag_register    0x0
#define isp2400_mamoiada_ready_irq_mask_flag_bit         0xA
#define isp2400_mamoiada_sleeping_irq_mask_flag_register 0x0
#define isp2400_mamoiada_sleeping_irq_mask_flag_bit      0xB
#define isp2400_mamoiada_debug_step_flag_register        0x6
#define isp2400_mamoiada_debug_step_flag_bit             0x0
#define isp2400_mamoiada_loop_cache_invalidate_flag_register    0x9
#define isp2400_mamoiada_loop_cache_invalidate_flag_bit         0x0
#define isp2400_mamoiada_program_filename_register       isp2400_mamoiada_start_flag_register
#define isp2400_mamoiada_base_config_mem_iam_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_base_config_mem_iam_op0_stalling_flag_bit 0x0
#define isp2400_mamoiada_base_config_mem_iam_op1_stalling_flag_register 0x8
#define isp2400_mamoiada_base_config_mem_iam_op1_stalling_flag_bit 0x1
#define isp2400_mamoiada_base_dmem_loc_mt_am_inst_0_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_base_dmem_loc_mt_am_inst_0_op0_stalling_flag_bit 0x2
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op0_stalling_flag_bit 0x3
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op1_stalling_flag_register 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op1_stalling_flag_bit 0x4
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op2_stalling_flag_register 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op2_stalling_flag_bit 0x5
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op3_stalling_flag_register 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op3_stalling_flag_bit 0x6
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op4_stalling_flag_register 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op4_stalling_flag_bit 0x7
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op5_stalling_flag_register 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op5_stalling_flag_bit 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op6_stalling_flag_register 0x8
#define isp2400_mamoiada_base_fifo_loc_mt_am_inst_1_op6_stalling_flag_bit 0x9
#define isp2400_mamoiada_simd_vmem_loc_mt_am_inst_2_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_simd_vmem_loc_mt_am_inst_2_op0_stalling_flag_bit 0xA
#define isp2400_mamoiada_simd_vamem1_loc_mt_am_inst_3_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_simd_vamem1_loc_mt_am_inst_3_op0_stalling_flag_bit 0xB
#define isp2400_mamoiada_simd_vamem2_loc_mt_am_inst_4_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_simd_vamem2_loc_mt_am_inst_4_op0_stalling_flag_bit 0xC
#define isp2400_mamoiada_simd_vamem3_loc_mt_am_inst_5_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_simd_vamem3_loc_mt_am_inst_5_op0_stalling_flag_bit 0xD
#define isp2400_mamoiada_simd_histogram_loc_mt_am_inst_6_op0_stalling_flag_register 0x8
#define isp2400_mamoiada_simd_histogram_loc_mt_am_inst_6_op0_stalling_flag_bit 0xE

#define isp2400_mamoiada_icache base_config_mem_icache
#define isp2400_mamoiada_icache_segment_size 1048576
#define isp2400_mamoiada_base_config_mem_master_base_address_lowest_register 0x4
#define isp2400_mamoiada_base_config_mem_master_burst_size_register          0x5
#define isp2400_mamoiada_base_config_mem_master_burst_size_lsb               0x0
#define isp2400_mamoiada_base_config_mem_master_burst_size_num_bits          0x2
#define isp2400_mamoiada_base_config_mem_master_burst_timeout_register       0x5
#define isp2400_mamoiada_base_config_mem_master_burst_timeout_lsb            0x2
#define isp2400_mamoiada_base_config_mem_master_burst_timeout_num_bits       0x4
#define isp2400_mamoiada_icache_master_interface isp2400_mamoiada_base_config_mem_master

#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_period_register 0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_period_lsb      0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_period_num_bits 0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1_bandwidth_register 0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1_bandwidth_lsb      0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1_bandwidth_num_bits 0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0_bandwidth_register 0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0_bandwidth_lsb      0x0
#define isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0_bandwidth_num_bits 0x0
#define _hrt_ctl_set_arbiter_isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp(cell, period, bandwidth, length) \
  { \
    hrt_ctl_set_arbiter_period(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp, period); \
    if (0<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1, bandwidth[0]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1, 0);\
    if (1<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0, bandwidth[1]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0, 0);\
  }

#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_period_register 0x0
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_period_lsb      0xE
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_period_num_bits 0x1
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1_bandwidth_register 0x0
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1_bandwidth_lsb      0x10
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1_bandwidth_num_bits 0x1
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0_bandwidth_register 0x0
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0_bandwidth_lsb      0xF
#define isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0_bandwidth_num_bits 0x1
#define _hrt_ctl_set_arbiter_isp2400_mamoiada_base_dmem_Arb_data_mem_wp0(cell, period, bandwidth, length) \
  { \
    hrt_ctl_set_arbiter_period(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0, period); \
    if (0<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1, bandwidth[0]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1, 0);\
    if (1<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0, bandwidth[1]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0, 0);\
  }


#define isp2400_mamoiada_arbiter_mode_lsb 0xE
#define isp2400_mamoiada_arbiter_mode_register 0x0
#define isp2400_mamoiada_base_config_mem_icache_invalidate_flag_register      0x0
#define isp2400_mamoiada_base_config_mem_icache_invalidate_flag_bit           0xC
#define isp2400_mamoiada_base_config_mem_icache_prefetch_enable_flag_register 0x0
#define isp2400_mamoiada_base_config_mem_icache_prefetch_enable_flag_bit      0xD

#define isp2400_mamoiada_base_dmem_size 0x4000
#define isp2400_mamoiada_base_dmem_physical_size 0x4000
#define isp2400_mamoiada_base_dmem_first_slave_port isp2400_mamoiada_sl_ipdmem_cell_port
#define isp2400_mamoiada_base_dmem_sl_ipdmem_next_cell_port isp2400_mamoiada_unreachable_cell_port
#define isp2400_mamoiada_sl_ipdmem_isp2400_mamoiada_base_dmem_address 0x0
#define _hrt_mem_load_8_isp2400_mamoiada_base_dmem(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_uload_8_isp2400_mamoiada_base_dmem(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_load_16_isp2400_mamoiada_base_dmem(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_uload_16_isp2400_mamoiada_base_dmem(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_load_32_isp2400_mamoiada_base_dmem(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_uload_32_isp2400_mamoiada_base_dmem(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_store_8_isp2400_mamoiada_base_dmem(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_store_16_isp2400_mamoiada_base_dmem(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_store_32_isp2400_mamoiada_base_dmem(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_load_isp2400_mamoiada_base_dmem(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_store_isp2400_mamoiada_base_dmem(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define _hrt_mem_set_isp2400_mamoiada_base_dmem(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_base_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_base_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_base_dmem))
#define isp2400_mamoiada_simd_vmem_size 0x2000000
#define isp2400_mamoiada_simd_vmem_physical_size 0x1FFFF80
#define isp2400_mamoiada_simd_vmem_first_slave_port isp2400_mamoiada_simd_ipdma_cell_port
#define isp2400_mamoiada_simd_vmem_simd_ipdma_next_cell_port isp2400_mamoiada_unreachable_cell_port
#define isp2400_mamoiada_simd_ipdma_isp2400_mamoiada_simd_vmem_address 0x0
#define _hrt_mem_load_8_isp2400_mamoiada_simd_vmem(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_uload_8_isp2400_mamoiada_simd_vmem(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_load_16_isp2400_mamoiada_simd_vmem(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_uload_16_isp2400_mamoiada_simd_vmem(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_load_32_isp2400_mamoiada_simd_vmem(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_uload_32_isp2400_mamoiada_simd_vmem(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_store_8_isp2400_mamoiada_simd_vmem(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_store_16_isp2400_mamoiada_simd_vmem(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_store_32_isp2400_mamoiada_simd_vmem(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_load_isp2400_mamoiada_simd_vmem(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_store_isp2400_mamoiada_simd_vmem(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define _hrt_mem_set_isp2400_mamoiada_simd_vmem(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vmem))
#define isp2400_mamoiada_simd_vamem1_size 0x1000
#define isp2400_mamoiada_simd_vamem1_physical_size 0x1000
#define isp2400_mamoiada_simd_vamem1_first_slave_port isp2400_mamoiada_sl_ipvamem_cell_port
#define isp2400_mamoiada_simd_vamem1_sl_ipvamem_next_cell_port isp2400_mamoiada_unreachable_cell_port
#define isp2400_mamoiada_sl_ipvamem_isp2400_mamoiada_simd_vamem1_address 0x0
#define _hrt_mem_load_8_isp2400_mamoiada_simd_vamem1(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_uload_8_isp2400_mamoiada_simd_vamem1(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_load_16_isp2400_mamoiada_simd_vamem1(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_uload_16_isp2400_mamoiada_simd_vamem1(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_load_32_isp2400_mamoiada_simd_vamem1(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_uload_32_isp2400_mamoiada_simd_vamem1(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_store_8_isp2400_mamoiada_simd_vamem1(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_store_16_isp2400_mamoiada_simd_vamem1(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_store_32_isp2400_mamoiada_simd_vamem1(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_load_isp2400_mamoiada_simd_vamem1(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_store_isp2400_mamoiada_simd_vamem1(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define _hrt_mem_set_isp2400_mamoiada_simd_vamem1(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem1), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem1, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem1))
#define isp2400_mamoiada_simd_vamem2_size 0x1000
#define isp2400_mamoiada_simd_vamem2_physical_size 0x1000
#define isp2400_mamoiada_simd_vamem2_first_slave_port isp2400_mamoiada_sl_ipvamem_cell_port
#define isp2400_mamoiada_simd_vamem2_sl_ipvamem_next_cell_port isp2400_mamoiada_unreachable_cell_port
#define isp2400_mamoiada_sl_ipvamem_isp2400_mamoiada_simd_vamem2_address 0x10000
#define _hrt_mem_load_8_isp2400_mamoiada_simd_vamem2(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_uload_8_isp2400_mamoiada_simd_vamem2(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_load_16_isp2400_mamoiada_simd_vamem2(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_uload_16_isp2400_mamoiada_simd_vamem2(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_load_32_isp2400_mamoiada_simd_vamem2(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_uload_32_isp2400_mamoiada_simd_vamem2(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_store_8_isp2400_mamoiada_simd_vamem2(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_store_16_isp2400_mamoiada_simd_vamem2(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_store_32_isp2400_mamoiada_simd_vamem2(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_load_isp2400_mamoiada_simd_vamem2(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_store_isp2400_mamoiada_simd_vamem2(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define _hrt_mem_set_isp2400_mamoiada_simd_vamem2(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem2), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem2, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem2))
#define isp2400_mamoiada_simd_vamem3_size 0x1000
#define isp2400_mamoiada_simd_vamem3_physical_size 0x1000
#define isp2400_mamoiada_simd_vamem3_first_slave_port isp2400_mamoiada_sl_ipvamem_cell_port
#define isp2400_mamoiada_simd_vamem3_sl_ipvamem_next_cell_port isp2400_mamoiada_unreachable_cell_port
#define isp2400_mamoiada_sl_ipvamem_isp2400_mamoiada_simd_vamem3_address 0x20000
#define _hrt_mem_load_8_isp2400_mamoiada_simd_vamem3(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_uload_8_isp2400_mamoiada_simd_vamem3(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_load_16_isp2400_mamoiada_simd_vamem3(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_uload_16_isp2400_mamoiada_simd_vamem3(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_load_32_isp2400_mamoiada_simd_vamem3(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_uload_32_isp2400_mamoiada_simd_vamem3(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_store_8_isp2400_mamoiada_simd_vamem3(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_store_16_isp2400_mamoiada_simd_vamem3(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_store_32_isp2400_mamoiada_simd_vamem3(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_load_isp2400_mamoiada_simd_vamem3(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_store_isp2400_mamoiada_simd_vamem3(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define _hrt_mem_set_isp2400_mamoiada_simd_vamem3(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_vamem3), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_vamem3, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_vamem3))
#define isp2400_mamoiada_simd_histogram_size 0x1000
#define isp2400_mamoiada_simd_histogram_physical_size 0x1000
#define isp2400_mamoiada_simd_histogram_first_slave_port isp2400_mamoiada_sl_iphist_cell_port
#define isp2400_mamoiada_simd_histogram_sl_iphist_next_cell_port isp2400_mamoiada_unreachable_cell_port
#define isp2400_mamoiada_sl_iphist_isp2400_mamoiada_simd_histogram_address 0x0
#define _hrt_mem_load_8_isp2400_mamoiada_simd_histogram(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_uload_8_isp2400_mamoiada_simd_histogram(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_load_16_isp2400_mamoiada_simd_histogram(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_uload_16_isp2400_mamoiada_simd_histogram(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_load_32_isp2400_mamoiada_simd_histogram(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_uload_32_isp2400_mamoiada_simd_histogram(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_store_8_isp2400_mamoiada_simd_histogram(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_store_16_isp2400_mamoiada_simd_histogram(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_store_32_isp2400_mamoiada_simd_histogram(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_load_isp2400_mamoiada_simd_histogram(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_store_isp2400_mamoiada_simd_histogram(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_set_isp2400_mamoiada_simd_histogram(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2400_mamoiada_simd_histogram), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2400_mamoiada_simd_histogram, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2400_mamoiada_simd_histogram))
#define _hrt_mem_load_isp2400_mamoiada_char(cell, mem, addr)   HRTCAT(hrt_mem_load_, isp2400_mamoiada_char_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_uchar(cell, mem, addr)  HRTCAT(hrt_mem_uload_, isp2400_mamoiada_uchar_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_short(cell, mem, addr)  HRTCAT(hrt_mem_load_, isp2400_mamoiada_short_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_ushort(cell, mem, addr) HRTCAT(hrt_mem_uload_, isp2400_mamoiada_ushort_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_int(cell, mem, addr)    HRTCAT(hrt_mem_load_, isp2400_mamoiada_int_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_uint(cell, mem, addr)   HRTCAT(hrt_mem_uload_, isp2400_mamoiada_uint_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_long(cell, mem, addr)   HRTCAT(hrt_mem_load_, isp2400_mamoiada_long_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_ulong(cell, mem, addr)  HRTCAT(hrt_mem_uload_, isp2400_mamoiada_ulong_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2400_mamoiada_ptr(cell, mem, addr)    HRTCAT(hrt_mem_uload_, isp2400_mamoiada_ptr_bits)(cell, mem, addr)
#define _hrtx_memid_load_CASE_isp2400_mamoiada(cell, mem, addr, data, bytes) \
  switch(mem) { \
    case isp2400_mamoiada_base_dmem: \
      _hrt_mem_load_isp2400_mamoiada_base_dmem(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vmem: \
      _hrt_mem_load_isp2400_mamoiada_simd_vmem(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem1: \
      _hrt_mem_load_isp2400_mamoiada_simd_vamem1(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem2: \
      _hrt_mem_load_isp2400_mamoiada_simd_vamem2(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem3: \
      _hrt_mem_load_isp2400_mamoiada_simd_vamem3(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_histogram: \
      _hrt_mem_load_isp2400_mamoiada_simd_histogram(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_memid_store_CASE_isp2400_mamoiada(cell, mem, addr, data, bytes) \
  switch(mem) {\
    case isp2400_mamoiada_base_dmem: \
      _hrt_mem_store_isp2400_mamoiada_base_dmem(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vmem: \
      _hrt_mem_store_isp2400_mamoiada_simd_vmem(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem1: \
      _hrt_mem_store_isp2400_mamoiada_simd_vamem1(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem2: \
      _hrt_mem_store_isp2400_mamoiada_simd_vamem2(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem3: \
      _hrt_mem_store_isp2400_mamoiada_simd_vamem3(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_histogram: \
      _hrt_mem_store_isp2400_mamoiada_simd_histogram(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_memid_set_CASE_isp2400_mamoiada(cell, mem, addr, data, bytes) \
  switch(mem) {\
    case isp2400_mamoiada_base_dmem: \
      _hrt_mem_set_isp2400_mamoiada_base_dmem(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vmem: \
      _hrt_mem_set_isp2400_mamoiada_simd_vmem(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem1: \
      _hrt_mem_set_isp2400_mamoiada_simd_vamem1(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem2: \
      _hrt_mem_set_isp2400_mamoiada_simd_vamem2(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_vamem3: \
      _hrt_mem_set_isp2400_mamoiada_simd_vamem3(cell, addr, data, bytes); \
      break;\
    case isp2400_mamoiada_simd_histogram: \
      _hrt_mem_set_isp2400_mamoiada_simd_histogram(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_cellid_mem_size_CASE_isp2400_mamoiada(cell, mem) \
  switch(mem) {\
    case isp2400_mamoiada_base_dmem: \
      return _hrt_cell_mem_size(cell, isp2400_mamoiada_base_dmem); \
    case isp2400_mamoiada_simd_vmem: \
      return _hrt_cell_mem_size(cell, isp2400_mamoiada_simd_vmem); \
    case isp2400_mamoiada_simd_vamem1: \
      return _hrt_cell_mem_size(cell, isp2400_mamoiada_simd_vamem1); \
    case isp2400_mamoiada_simd_vamem2: \
      return _hrt_cell_mem_size(cell, isp2400_mamoiada_simd_vamem2); \
    case isp2400_mamoiada_simd_vamem3: \
      return _hrt_cell_mem_size(cell, isp2400_mamoiada_simd_vamem3); \
    case isp2400_mamoiada_simd_histogram: \
      return _hrt_cell_mem_size(cell, isp2400_mamoiada_simd_histogram); \
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_cellid_mem_physical_size_CASE_isp2400_mamoiada(cell, mem) \
  switch(mem) {\
    case isp2400_mamoiada_base_dmem: \
      return _hrt_cell_mem_physical_size(cell, isp2400_mamoiada_base_dmem); \
    case isp2400_mamoiada_simd_vmem: \
      return _hrt_cell_mem_physical_size(cell, isp2400_mamoiada_simd_vmem); \
    case isp2400_mamoiada_simd_vamem1: \
      return _hrt_cell_mem_physical_size(cell, isp2400_mamoiada_simd_vamem1); \
    case isp2400_mamoiada_simd_vamem2: \
      return _hrt_cell_mem_physical_size(cell, isp2400_mamoiada_simd_vamem2); \
    case isp2400_mamoiada_simd_vamem3: \
      return _hrt_cell_mem_physical_size(cell, isp2400_mamoiada_simd_vamem3); \
    case isp2400_mamoiada_simd_histogram: \
      return _hrt_cell_mem_physical_size(cell, isp2400_mamoiada_simd_histogram); \
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_ctlid_set_arbiter_period_CASE_isp2400_mamoiada(cell, arbiter, period) \
  switch(arbiter) {\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp: \
      hrt_ctl_set_arbiter_period(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp, period); \
      break;\
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0: \
      hrt_ctl_set_arbiter_period(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0, period); \
      break;\
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_get_arbiter_period_CASE_isp2400_mamoiada(cell, arbiter) \
  switch(arbiter) {\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp: \
      return hrtx_ctl_get_arbiter_period(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp); \
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0: \
      return hrtx_ctl_get_arbiter_period(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0); \
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_get_arbiter_period_num_bits_CASE_isp2400_mamoiada(cell, arbiter) \
  switch(arbiter) {\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp: \
      return _hrt_cell_arbiter_period_num_bits(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp); \
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0: \
      return _hrt_cell_arbiter_period_num_bits(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0); \
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_set_arbiter_CASE_isp2400_mamoiada(cell, arbiter, period, bandwidth, length) \
  switch(arbiter) {\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp: \
      hrt_ctl_set_arbiter(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp, period, bandwidth, length); \
      break;\
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0: \
      hrt_ctl_set_arbiter(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0, period, bandwidth, length); \
      break;\
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_set_arbiter_contender_bandwidth_CASE_isp2400_mamoiada(cell, contender, bandwidth) \
  switch(contender) {\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1, bandwidth); \
      break;\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0, bandwidth); \
      break;\
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1, bandwidth); \
      break;\
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0, bandwidth); \
      break;\
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_get_arbiter_contender_bandwidth_CASE_isp2400_mamoiada(cell, contender) \
  switch(contender) {\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1); \
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0); \
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1); \
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0); \
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_get_arbiter_contender_bandwidth_num_bits_CASE_isp2400_mamoiada(cell, contender) \
  switch(contender) {\
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1); \
    case isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0); \
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1); \
    case isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0); \
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_set_burst_length_CASE_isp2400_mamoiada(cell, mt_int, length) \
  switch(mt_int) {\
    case isp2400_mamoiada_base_config_mem_master: \
      hrt_ctl_set_burst_length(cell, isp2400_mamoiada_base_config_mem_master, length); \
      break;\
    default:\
      length = length; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_burst_length_CASE_isp2400_mamoiada(cell, mt_int) \
  switch(mt_int) {\
    case isp2400_mamoiada_base_config_mem_master: \
      return hrtx_ctl_get_burst_length(cell, isp2400_mamoiada_base_config_mem_master); \
    default:\
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_set_burst_interval_CASE_isp2400_mamoiada(cell, mt_int, interval) \
  switch(mt_int) {\
    case isp2400_mamoiada_base_config_mem_master: \
      hrt_ctl_set_burst_interval(cell, isp2400_mamoiada_base_config_mem_master, interval); \
      break;\
    default:\
      interval = interval; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_burst_interval_CASE_isp2400_mamoiada(cell, mt_int) \
  switch(mt_int) {\
    case isp2400_mamoiada_base_config_mem_master: \
      return hrtx_ctl_get_burst_interval(cell, isp2400_mamoiada_base_config_mem_master); \
    default:\
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_ext_base_address_CASE_isp2400_mamoiada(cell, mt_int, segment) \
  switch(mt_int) {\
    case isp2400_mamoiada_base_config_mem_master: \
      return hrtx_ctl_get_ext_base_address(cell, isp2400_mamoiada_base_config_mem_master, segment); \
    default:\
      segment = segment; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_set_ext_base_address_CASE_isp2400_mamoiada(cell, mt_int, segment, addr) \
  switch(mt_int) {\
    case isp2400_mamoiada_base_config_mem_master: \
      hrt_ctl_set_ext_base_address(cell, isp2400_mamoiada_base_config_mem_master, segment, addr); \
      break;\
    default:\
      segment = segment; \
      addr = addr; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }

#define _jtag_isp2400_mamoiada_base_config_mem_master_width 640
#define _jtag_isp2400_mamoiada_base_config_mem_master_capacity 6144
#define _jtag_isp2400_mamoiada_base_config_mem_master_type 1
#define _jtag_isp2400_mamoiada_base_dmem_data_mem_width 32
#define _jtag_isp2400_mamoiada_base_dmem_data_mem_capacity 4096
#define _jtag_isp2400_mamoiada_base_dmem_data_mem_type 1
#define _jtag_isp2400_mamoiada_base_fifo_fifo_if_width 32
#define _jtag_isp2400_mamoiada_base_fifo_fifo_if_capacity 0
#define _jtag_isp2400_mamoiada_base_fifo_fifo_if_type 1
#define _jtag_isp2400_mamoiada_base_fifo_fifo_if_b_width 32
#define _jtag_isp2400_mamoiada_base_fifo_fifo_if_b_capacity 0
#define _jtag_isp2400_mamoiada_base_fifo_fifo_if_b_type 1
#define _jtag_isp2400_mamoiada_base_fifo_fifo_dma_width 32
#define _jtag_isp2400_mamoiada_base_fifo_fifo_dma_capacity 0
#define _jtag_isp2400_mamoiada_base_fifo_fifo_dma_type 1
#define _jtag_isp2400_mamoiada_base_fifo_fifo_gdc_width 32
#define _jtag_isp2400_mamoiada_base_fifo_fifo_gdc_capacity 0
#define _jtag_isp2400_mamoiada_base_fifo_fifo_gdc_type 1
#define _jtag_isp2400_mamoiada_base_fifo_fifo_scl_width 32
#define _jtag_isp2400_mamoiada_base_fifo_fifo_scl_capacity 0
#define _jtag_isp2400_mamoiada_base_fifo_fifo_scl_type 1
#define _jtag_isp2400_mamoiada_base_fifo_fifo_gpfifo_width 32
#define _jtag_isp2400_mamoiada_base_fifo_fifo_gpfifo_capacity 0
#define _jtag_isp2400_mamoiada_base_fifo_fifo_gpfifo_type 1
#define _jtag_isp2400_mamoiada_base_fifo_fifo_sp_width 32
#define _jtag_isp2400_mamoiada_base_fifo_fifo_sp_capacity 0
#define _jtag_isp2400_mamoiada_base_fifo_fifo_sp_type 1
#define _jtag_isp2400_mamoiada_simd_vmem_bamem_width 896
#define _jtag_isp2400_mamoiada_simd_vmem_bamem_capacity 262143
#define _jtag_isp2400_mamoiada_simd_vmem_bamem_type 1
#define _jtag_isp2400_mamoiada_simd_vamem1_asp_lut_width 12
#define _jtag_isp2400_mamoiada_simd_vamem1_asp_lut_capacity 2048
#define _jtag_isp2400_mamoiada_simd_vamem1_asp_lut_type 1
#define _jtag_isp2400_mamoiada_simd_vamem2_asp_lut_width 12
#define _jtag_isp2400_mamoiada_simd_vamem2_asp_lut_capacity 2048
#define _jtag_isp2400_mamoiada_simd_vamem2_asp_lut_type 1
#define _jtag_isp2400_mamoiada_simd_vamem3_asp_lut_width 12
#define _jtag_isp2400_mamoiada_simd_vamem3_asp_lut_capacity 2048
#define _jtag_isp2400_mamoiada_simd_vamem3_asp_lut_type 1
#define _jtag_isp2400_mamoiada_simd_histogram_asp_histogram_width 24
#define _jtag_isp2400_mamoiada_simd_histogram_asp_histogram_capacity 1024
#define _jtag_isp2400_mamoiada_simd_histogram_asp_histogram_type 1
#define _jtag_isp2400_mamoiada_base_rf1_width 32
#define _jtag_isp2400_mamoiada_base_rf1_capacity 64
#define _jtag_isp2400_mamoiada_base_rf1_type 2
#define _jtag_isp2400_mamoiada_base_rf2_width 32
#define _jtag_isp2400_mamoiada_base_rf2_capacity 16
#define _jtag_isp2400_mamoiada_base_rf2_type 2
#define _jtag_isp2400_mamoiada_base_rf3_width 32
#define _jtag_isp2400_mamoiada_base_rf3_capacity 64
#define _jtag_isp2400_mamoiada_base_rf3_type 2
#define _jtag_isp2400_mamoiada_base_frf1_width 64
#define _jtag_isp2400_mamoiada_base_frf1_capacity 16
#define _jtag_isp2400_mamoiada_base_frf1_type 2
#define _jtag_isp2400_mamoiada_simd_rf1_width 32
#define _jtag_isp2400_mamoiada_simd_rf1_capacity 64
#define _jtag_isp2400_mamoiada_simd_rf1_type 2
#define _jtag_isp2400_mamoiada_simd_rf2_width 16
#define _jtag_isp2400_mamoiada_simd_rf2_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf2_type 2
#define _jtag_isp2400_mamoiada_simd_rf3_width 16
#define _jtag_isp2400_mamoiada_simd_rf3_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf3_type 2
#define _jtag_isp2400_mamoiada_simd_rf4_width 16
#define _jtag_isp2400_mamoiada_simd_rf4_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf4_type 2
#define _jtag_isp2400_mamoiada_simd_rf4_vadd_width 16
#define _jtag_isp2400_mamoiada_simd_rf4_vadd_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf4_vadd_type 2
#define _jtag_isp2400_mamoiada_simd_rf5_width 16
#define _jtag_isp2400_mamoiada_simd_rf5_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf5_type 2
#define _jtag_isp2400_mamoiada_simd_rf6_width 16
#define _jtag_isp2400_mamoiada_simd_rf6_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf6_type 2
#define _jtag_isp2400_mamoiada_simd_rf7_width 16
#define _jtag_isp2400_mamoiada_simd_rf7_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf7_type 2
#define _jtag_isp2400_mamoiada_simd_rf8_width 16
#define _jtag_isp2400_mamoiada_simd_rf8_capacity 16
#define _jtag_isp2400_mamoiada_simd_rf8_type 2
#define _jtag_isp2400_mamoiada_simd_vrf1_width 896
#define _jtag_isp2400_mamoiada_simd_vrf1_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf1_type 2
#define _jtag_isp2400_mamoiada_simd_vrf2_width 896
#define _jtag_isp2400_mamoiada_simd_vrf2_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf2_type 2
#define _jtag_isp2400_mamoiada_simd_vrf3_width 896
#define _jtag_isp2400_mamoiada_simd_vrf3_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf3_type 2
#define _jtag_isp2400_mamoiada_simd_vrf4_width 896
#define _jtag_isp2400_mamoiada_simd_vrf4_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf4_type 2
#define _jtag_isp2400_mamoiada_simd_vrf4_vadd_width 896
#define _jtag_isp2400_mamoiada_simd_vrf4_vadd_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf4_vadd_type 2
#define _jtag_isp2400_mamoiada_simd_vrf5_width 896
#define _jtag_isp2400_mamoiada_simd_vrf5_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf5_type 2
#define _jtag_isp2400_mamoiada_simd_vrf6_width 896
#define _jtag_isp2400_mamoiada_simd_vrf6_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf6_type 2
#define _jtag_isp2400_mamoiada_simd_vrf7_width 896
#define _jtag_isp2400_mamoiada_simd_vrf7_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf7_type 2
#define _jtag_isp2400_mamoiada_simd_vrf8_width 896
#define _jtag_isp2400_mamoiada_simd_vrf8_capacity 24
#define _jtag_isp2400_mamoiada_simd_vrf8_type 2
#define _jtag_isp2400_mamoiada_simd_srf1_width 56
#define _jtag_isp2400_mamoiada_simd_srf1_capacity 4
#define _jtag_isp2400_mamoiada_simd_srf1_type 2
#define _jtag_isp2400_mamoiada_simd_srf2_width 56
#define _jtag_isp2400_mamoiada_simd_srf2_capacity 64
#define _jtag_isp2400_mamoiada_simd_srf2_type 2
#define _jtag_isp2400_mamoiada_simd_srf3_width 56
#define _jtag_isp2400_mamoiada_simd_srf3_capacity 64
#define _jtag_isp2400_mamoiada_simd_srf3_type 2
#define _jtag_isp2400_mamoiada_simd_srf4_width 56
#define _jtag_isp2400_mamoiada_simd_srf4_capacity 32
#define _jtag_isp2400_mamoiada_simd_srf4_type 2
#define _jtag_isp2400_mamoiada_simd_srf5_width 56
#define _jtag_isp2400_mamoiada_simd_srf5_capacity 64
#define _jtag_isp2400_mamoiada_simd_srf5_type 2
#define _jtag_isp2400_mamoiada_simd_frf1_width 64
#define _jtag_isp2400_mamoiada_simd_frf1_capacity 4
#define _jtag_isp2400_mamoiada_simd_frf1_type 2
#define _jtag_isp2400_mamoiada_simd_frf2_width 64
#define _jtag_isp2400_mamoiada_simd_frf2_capacity 16
#define _jtag_isp2400_mamoiada_simd_frf2_type 2
#define _jtag_isp2400_mamoiada_simd_frf3_width 64
#define _jtag_isp2400_mamoiada_simd_frf3_capacity 4
#define _jtag_isp2400_mamoiada_simd_frf3_type 2
#define _jtag_isp2400_mamoiada_simd_frf4_width 64
#define _jtag_isp2400_mamoiada_simd_frf4_capacity 4
#define _jtag_isp2400_mamoiada_simd_frf4_type 2
#define _jtag_isp2400_mamoiada_simd_frf5_width 64
#define _jtag_isp2400_mamoiada_simd_frf5_capacity 8
#define _jtag_isp2400_mamoiada_simd_frf5_type 2
#define _jtag_isp2400_mamoiada_simd_frf6_width 64
#define _jtag_isp2400_mamoiada_simd_frf6_capacity 4
#define _jtag_isp2400_mamoiada_simd_frf6_type 2
#define _jtag_isp2400_mamoiada_base_PC_width 13
#define _jtag_isp2400_mamoiada_base_PC_capacity 1
#define _jtag_isp2400_mamoiada_base_PC_type 2
#define _jtag_isp2400_mamoiada_base_SR_width 9
#define _jtag_isp2400_mamoiada_base_SR_capacity 1
#define _jtag_isp2400_mamoiada_base_SR_type 2

#endif /* _isp2400_mamoiada_cell_h */
