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

#ifndef _scalar_processor_2400_cell_h
#define _scalar_processor_2400_cell_h

//#include <hrt/api.h>

#define scalar_processor_2400_prog_mem_slave_port  scalar_processor_2400_unreachable_cell_port
#define scalar_processor_2400_prog_mem_width       0
#define scalar_processor_2400_prog_mem_base        0x0
#define scalar_processor_2400_stat_ctrl_slave_port scalar_processor_2400_sl_ip0_cell_port
#define scalar_processor_2400_stat_ctrl_base       0x0
#define scalar_processor_2400_view_table_base      0x0
#define scalar_processor_2400_num_views            1
#define scalar_processor_2400_default_mem          scalar_processor_2400_dmem
#define scalar_processor_2400_int_size             4
#define scalar_processor_2400_char_bits            8
#define scalar_processor_2400_uchar_bits           scalar_processor_2400_char_bits
#define scalar_processor_2400_short_bits           16
#define scalar_processor_2400_ushort_bits           scalar_processor_2400_short_bits
#define scalar_processor_2400_int_bits             32
#define scalar_processor_2400_uint_bits            scalar_processor_2400_int_bits
#define scalar_processor_2400_long_bits            32
#define scalar_processor_2400_ulong_bits           scalar_processor_2400_long_bits
#define scalar_processor_2400_ptr_bits             32

#define scalar_processor_2400_start_address_register          0x1
#define scalar_processor_2400_break_address_register          0x2
#define scalar_processor_2400_debug_pc_register               0x9
#define scalar_processor_2400_reset_flag_register             0x0
#define scalar_processor_2400_reset_flag_bit                  0x0
#define scalar_processor_2400_start_flag_register             0x0
#define scalar_processor_2400_start_flag_bit                  0x1
#define scalar_processor_2400_break_flag_register             0x0
#define scalar_processor_2400_break_flag_bit                  0x2
#define scalar_processor_2400_run_flag_register               0x0
#define scalar_processor_2400_run_flag_bit                    0x3
#define scalar_processor_2400_broken_flag_register            0x0
#define scalar_processor_2400_broken_flag_bit                 0x4
#define scalar_processor_2400_ready_flag_register             0x0
#define scalar_processor_2400_ready_flag_bit                  0x5
#define scalar_processor_2400_sleeping_flag_register          0x0
#define scalar_processor_2400_sleeping_flag_bit               0x6
#define scalar_processor_2400_stalling_flag_register          0x0
#define scalar_processor_2400_stalling_flag_bit               0x7
#define scalar_processor_2400_irq_clr_flag_register           0x0
#define scalar_processor_2400_irq_clr_flag_bit                0x8
#define scalar_processor_2400_broken_irq_mask_flag_register   0x0
#define scalar_processor_2400_broken_irq_mask_flag_bit        0x9
#define scalar_processor_2400_ready_irq_mask_flag_register    0x0
#define scalar_processor_2400_ready_irq_mask_flag_bit         0xA
#define scalar_processor_2400_sleeping_irq_mask_flag_register 0x0
#define scalar_processor_2400_sleeping_irq_mask_flag_bit      0xB
#define scalar_processor_2400_debug_step_flag_register        0x8
#define scalar_processor_2400_debug_step_flag_bit             0x0
#define scalar_processor_2400_loop_cache_invalidate_flag_register    0xB
#define scalar_processor_2400_loop_cache_invalidate_flag_bit         0x0
#define scalar_processor_2400_program_filename_register       scalar_processor_2400_start_flag_register
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op0_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op0_stalling_flag_bit 0x0
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op1_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op1_stalling_flag_bit 0x1
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op2_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op2_stalling_flag_bit 0x2
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op3_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op3_stalling_flag_bit 0x3
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op4_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op4_stalling_flag_bit 0x4
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op5_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op5_stalling_flag_bit 0x5
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op6_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op6_stalling_flag_bit 0x6
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op7_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op7_stalling_flag_bit 0x7
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op8_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op8_stalling_flag_bit 0x8
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op9_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op9_stalling_flag_bit 0x9
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op10_stalling_flag_register 0xA
#define scalar_processor_2400_fifo_loc_mt_am_inst_0_op10_stalling_flag_bit 0xA
#define scalar_processor_2400_dmem_loc_mt_am_inst_1_op0_stalling_flag_register 0xA
#define scalar_processor_2400_dmem_loc_mt_am_inst_1_op0_stalling_flag_bit 0xB
#define scalar_processor_2400_xmem_loc_mt_am_inst_2_op0_stalling_flag_register 0xA
#define scalar_processor_2400_xmem_loc_mt_am_inst_2_op0_stalling_flag_bit 0xC
#define scalar_processor_2400_config_icache_conf_icache_loc_mt_am_inst_3_op0_stalling_flag_register 0xA
#define scalar_processor_2400_config_icache_conf_icache_loc_mt_am_inst_3_op0_stalling_flag_bit 0xD

#define scalar_processor_2400_icache config_icache_conf_icache_icache
#define scalar_processor_2400_icache_segment_size 1048576
#define scalar_processor_2400_xmem_master_int_base_address_lowest_register 0x4
#define scalar_processor_2400_xmem_master_int_burst_size_register          0x0
#define scalar_processor_2400_xmem_master_int_burst_size_lsb               0x0
#define scalar_processor_2400_xmem_master_int_burst_size_num_bits          0x0
#define scalar_processor_2400_xmem_master_int_burst_timeout_register       0x6
#define scalar_processor_2400_xmem_master_int_burst_timeout_lsb            0x0
#define scalar_processor_2400_xmem_master_int_burst_timeout_num_bits       0x4
#define scalar_processor_2400_config_icache_conf_icache_master_base_address_lowest_register 0x5
#define scalar_processor_2400_config_icache_conf_icache_master_burst_size_register          0x7
#define scalar_processor_2400_config_icache_conf_icache_master_burst_size_lsb               0x0
#define scalar_processor_2400_config_icache_conf_icache_master_burst_size_num_bits          0x4
#define scalar_processor_2400_config_icache_conf_icache_master_burst_timeout_register       0x7
#define scalar_processor_2400_config_icache_conf_icache_master_burst_timeout_lsb            0x4
#define scalar_processor_2400_config_icache_conf_icache_master_burst_timeout_num_bits       0x4
#define scalar_processor_2400_icache_master_interface scalar_processor_2400_config_icache_conf_icache_master

#define scalar_processor_2400_dmem_Arb_mem_wp_period_register 0x0
#define scalar_processor_2400_dmem_Arb_mem_wp_period_lsb      0xE
#define scalar_processor_2400_dmem_Arb_mem_wp_period_num_bits 0x5
#define scalar_processor_2400_dmem_Arb_mem_wp_source1_bandwidth_register 0x0
#define scalar_processor_2400_dmem_Arb_mem_wp_source1_bandwidth_lsb      0x18
#define scalar_processor_2400_dmem_Arb_mem_wp_source1_bandwidth_num_bits 0x5
#define scalar_processor_2400_dmem_Arb_mem_wp_source0_bandwidth_register 0x0
#define scalar_processor_2400_dmem_Arb_mem_wp_source0_bandwidth_lsb      0x13
#define scalar_processor_2400_dmem_Arb_mem_wp_source0_bandwidth_num_bits 0x5
#define _hrt_ctl_set_arbiter_scalar_processor_2400_dmem_Arb_mem_wp(cell, period, bandwidth, length) \
  { \
    hrt_ctl_set_arbiter_period(cell, scalar_processor_2400_dmem_Arb_mem_wp, period); \
    if (0<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source1, bandwidth[0]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source1, 0);\
    if (1<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source0, bandwidth[1]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source0, 0);\
  }


#define scalar_processor_2400_arbiter_mode_lsb 0xE
#define scalar_processor_2400_arbiter_mode_register 0x0
#define scalar_processor_2400_config_icache_conf_icache_icache_invalidate_flag_register      0x0
#define scalar_processor_2400_config_icache_conf_icache_icache_invalidate_flag_bit           0xC
#define scalar_processor_2400_config_icache_conf_icache_icache_prefetch_enable_flag_register 0x0
#define scalar_processor_2400_config_icache_conf_icache_icache_prefetch_enable_flag_bit      0xD

#define scalar_processor_2400_dmem_size 0x4000
#define scalar_processor_2400_dmem_physical_size 0x4000
#define scalar_processor_2400_dmem_first_slave_port scalar_processor_2400_sl_ip1_cell_port
#define scalar_processor_2400_dmem_sl_ip1_next_cell_port scalar_processor_2400_unreachable_cell_port
#define scalar_processor_2400_sl_ip1_scalar_processor_2400_dmem_address 0x0
#define _hrt_mem_load_8_scalar_processor_2400_dmem(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_uload_8_scalar_processor_2400_dmem(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_load_16_scalar_processor_2400_dmem(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_uload_16_scalar_processor_2400_dmem(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_load_32_scalar_processor_2400_dmem(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_uload_32_scalar_processor_2400_dmem(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_store_8_scalar_processor_2400_dmem(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_store_16_scalar_processor_2400_dmem(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_store_32_scalar_processor_2400_dmem(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_load_scalar_processor_2400_dmem(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_store_scalar_processor_2400_dmem(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_set_scalar_processor_2400_dmem(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, scalar_processor_2400_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, scalar_processor_2400_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, scalar_processor_2400_dmem))
#define _hrt_mem_load_scalar_processor_2400_char(cell, mem, addr)   HRTCAT(hrt_mem_load_, scalar_processor_2400_char_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_uchar(cell, mem, addr)  HRTCAT(hrt_mem_uload_, scalar_processor_2400_uchar_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_short(cell, mem, addr)  HRTCAT(hrt_mem_load_, scalar_processor_2400_short_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_ushort(cell, mem, addr) HRTCAT(hrt_mem_uload_, scalar_processor_2400_ushort_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_int(cell, mem, addr)    HRTCAT(hrt_mem_load_, scalar_processor_2400_int_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_uint(cell, mem, addr)   HRTCAT(hrt_mem_uload_, scalar_processor_2400_uint_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_long(cell, mem, addr)   HRTCAT(hrt_mem_load_, scalar_processor_2400_long_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_ulong(cell, mem, addr)  HRTCAT(hrt_mem_uload_, scalar_processor_2400_ulong_bits)(cell, mem, addr)
#define _hrt_mem_load_scalar_processor_2400_ptr(cell, mem, addr)    HRTCAT(hrt_mem_uload_, scalar_processor_2400_ptr_bits)(cell, mem, addr)
#define _hrtx_memid_load_CASE_scalar_processor_2400(cell, mem, addr, data, bytes) \
  switch(mem) { \
    case scalar_processor_2400_dmem: \
      _hrt_mem_load_scalar_processor_2400_dmem(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_memid_store_CASE_scalar_processor_2400(cell, mem, addr, data, bytes) \
  switch(mem) {\
    case scalar_processor_2400_dmem: \
      _hrt_mem_store_scalar_processor_2400_dmem(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_memid_set_CASE_scalar_processor_2400(cell, mem, addr, data, bytes) \
  switch(mem) {\
    case scalar_processor_2400_dmem: \
      _hrt_mem_set_scalar_processor_2400_dmem(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_cellid_mem_size_CASE_scalar_processor_2400(cell, mem) \
  switch(mem) {\
    case scalar_processor_2400_dmem: \
      return _hrt_cell_mem_size(cell, scalar_processor_2400_dmem); \
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_cellid_mem_physical_size_CASE_scalar_processor_2400(cell, mem) \
  switch(mem) {\
    case scalar_processor_2400_dmem: \
      return _hrt_cell_mem_physical_size(cell, scalar_processor_2400_dmem); \
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_ctlid_set_arbiter_period_CASE_scalar_processor_2400(cell, arbiter, period) \
  switch(arbiter) {\
    case scalar_processor_2400_dmem_Arb_mem_wp: \
      hrt_ctl_set_arbiter_period(cell, scalar_processor_2400_dmem_Arb_mem_wp, period); \
      break;\
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_get_arbiter_period_CASE_scalar_processor_2400(cell, arbiter) \
  switch(arbiter) {\
    case scalar_processor_2400_dmem_Arb_mem_wp: \
      return hrtx_ctl_get_arbiter_period(cell, scalar_processor_2400_dmem_Arb_mem_wp); \
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_get_arbiter_period_num_bits_CASE_scalar_processor_2400(cell, arbiter) \
  switch(arbiter) {\
    case scalar_processor_2400_dmem_Arb_mem_wp: \
      return _hrt_cell_arbiter_period_num_bits(cell, scalar_processor_2400_dmem_Arb_mem_wp); \
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_set_arbiter_CASE_scalar_processor_2400(cell, arbiter, period, bandwidth, length) \
  switch(arbiter) {\
    case scalar_processor_2400_dmem_Arb_mem_wp: \
      hrt_ctl_set_arbiter(cell, scalar_processor_2400_dmem_Arb_mem_wp, period, bandwidth, length); \
      break;\
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_set_arbiter_contender_bandwidth_CASE_scalar_processor_2400(cell, contender, bandwidth) \
  switch(contender) {\
    case scalar_processor_2400_dmem_Arb_mem_wp_source1: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source1, bandwidth); \
      break;\
    case scalar_processor_2400_dmem_Arb_mem_wp_source0: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source0, bandwidth); \
      break;\
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_get_arbiter_contender_bandwidth_CASE_scalar_processor_2400(cell, contender) \
  switch(contender) {\
    case scalar_processor_2400_dmem_Arb_mem_wp_source1: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source1); \
    case scalar_processor_2400_dmem_Arb_mem_wp_source0: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, scalar_processor_2400_dmem_Arb_mem_wp_source0); \
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_get_arbiter_contender_bandwidth_num_bits_CASE_scalar_processor_2400(cell, contender) \
  switch(contender) {\
    case scalar_processor_2400_dmem_Arb_mem_wp_source1: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, scalar_processor_2400_dmem_Arb_mem_wp_source1); \
    case scalar_processor_2400_dmem_Arb_mem_wp_source0: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, scalar_processor_2400_dmem_Arb_mem_wp_source0); \
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_set_burst_length_CASE_scalar_processor_2400(cell, mt_int, length) \
  switch(mt_int) {\
    case scalar_processor_2400_xmem_master_int: \
      hrt_ctl_set_burst_length(cell, scalar_processor_2400_xmem_master_int, length); \
      break;\
    case scalar_processor_2400_config_icache_conf_icache_master: \
      hrt_ctl_set_burst_length(cell, scalar_processor_2400_config_icache_conf_icache_master, length); \
      break;\
    default:\
      length = length; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_burst_length_CASE_scalar_processor_2400(cell, mt_int) \
  switch(mt_int) {\
    case scalar_processor_2400_xmem_master_int: \
      return hrtx_ctl_get_burst_length(cell, scalar_processor_2400_xmem_master_int); \
    case scalar_processor_2400_config_icache_conf_icache_master: \
      return hrtx_ctl_get_burst_length(cell, scalar_processor_2400_config_icache_conf_icache_master); \
    default:\
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_set_burst_interval_CASE_scalar_processor_2400(cell, mt_int, interval) \
  switch(mt_int) {\
    case scalar_processor_2400_xmem_master_int: \
      hrt_ctl_set_burst_interval(cell, scalar_processor_2400_xmem_master_int, interval); \
      break;\
    case scalar_processor_2400_config_icache_conf_icache_master: \
      hrt_ctl_set_burst_interval(cell, scalar_processor_2400_config_icache_conf_icache_master, interval); \
      break;\
    default:\
      interval = interval; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_burst_interval_CASE_scalar_processor_2400(cell, mt_int) \
  switch(mt_int) {\
    case scalar_processor_2400_xmem_master_int: \
      return hrtx_ctl_get_burst_interval(cell, scalar_processor_2400_xmem_master_int); \
    case scalar_processor_2400_config_icache_conf_icache_master: \
      return hrtx_ctl_get_burst_interval(cell, scalar_processor_2400_config_icache_conf_icache_master); \
    default:\
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_ext_base_address_CASE_scalar_processor_2400(cell, mt_int, segment) \
  switch(mt_int) {\
    case scalar_processor_2400_xmem_master_int: \
      return hrtx_ctl_get_ext_base_address(cell, scalar_processor_2400_xmem_master_int, segment); \
    case scalar_processor_2400_config_icache_conf_icache_master: \
      return hrtx_ctl_get_ext_base_address(cell, scalar_processor_2400_config_icache_conf_icache_master, segment); \
    default:\
      segment = segment; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_set_ext_base_address_CASE_scalar_processor_2400(cell, mt_int, segment, addr) \
  switch(mt_int) {\
    case scalar_processor_2400_xmem_master_int: \
      hrt_ctl_set_ext_base_address(cell, scalar_processor_2400_xmem_master_int, segment, addr); \
      break;\
    case scalar_processor_2400_config_icache_conf_icache_master: \
      hrt_ctl_set_ext_base_address(cell, scalar_processor_2400_config_icache_conf_icache_master, segment, addr); \
      break;\
    default:\
      segment = segment; \
      addr = addr; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }

#define _jtag_scalar_processor_2400_fifo_fifo0_width 32
#define _jtag_scalar_processor_2400_fifo_fifo0_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo0_type 1
#define _jtag_scalar_processor_2400_fifo_fifo1_width 32
#define _jtag_scalar_processor_2400_fifo_fifo1_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo1_type 1
#define _jtag_scalar_processor_2400_fifo_fifo2_width 32
#define _jtag_scalar_processor_2400_fifo_fifo2_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo2_type 1
#define _jtag_scalar_processor_2400_fifo_fifo3_width 32
#define _jtag_scalar_processor_2400_fifo_fifo3_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo3_type 1
#define _jtag_scalar_processor_2400_fifo_fifo4_width 32
#define _jtag_scalar_processor_2400_fifo_fifo4_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo4_type 1
#define _jtag_scalar_processor_2400_fifo_fifo5_width 32
#define _jtag_scalar_processor_2400_fifo_fifo5_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo5_type 1
#define _jtag_scalar_processor_2400_fifo_fifo6_width 32
#define _jtag_scalar_processor_2400_fifo_fifo6_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo6_type 1
#define _jtag_scalar_processor_2400_fifo_fifo7_width 32
#define _jtag_scalar_processor_2400_fifo_fifo7_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo7_type 1
#define _jtag_scalar_processor_2400_fifo_fifo8_width 32
#define _jtag_scalar_processor_2400_fifo_fifo8_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo8_type 1
#define _jtag_scalar_processor_2400_fifo_fifo9_width 32
#define _jtag_scalar_processor_2400_fifo_fifo9_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo9_type 1
#define _jtag_scalar_processor_2400_fifo_fifo10_width 32
#define _jtag_scalar_processor_2400_fifo_fifo10_capacity 0
#define _jtag_scalar_processor_2400_fifo_fifo10_type 1
#define _jtag_scalar_processor_2400_dmem_mem_width 32
#define _jtag_scalar_processor_2400_dmem_mem_capacity 4096
#define _jtag_scalar_processor_2400_dmem_mem_type 1
#define _jtag_scalar_processor_2400_xmem_master_int_width 32
#define _jtag_scalar_processor_2400_xmem_master_int_capacity 1073741824
#define _jtag_scalar_processor_2400_xmem_master_int_type 1
#define _jtag_scalar_processor_2400_config_icache_conf_icache_master_width 64
#define _jtag_scalar_processor_2400_config_icache_conf_icache_master_capacity 131072
#define _jtag_scalar_processor_2400_config_icache_conf_icache_master_type 1
#define _jtag_scalar_processor_2400_rf1_width 32
#define _jtag_scalar_processor_2400_rf1_capacity 32
#define _jtag_scalar_processor_2400_rf1_type 2
#define _jtag_scalar_processor_2400_rf2_width 32
#define _jtag_scalar_processor_2400_rf2_capacity 16
#define _jtag_scalar_processor_2400_rf2_type 2
#define _jtag_scalar_processor_2400_pc_width 17
#define _jtag_scalar_processor_2400_pc_capacity 1
#define _jtag_scalar_processor_2400_pc_type 2
#define _jtag_scalar_processor_2400_sr_width 9
#define _jtag_scalar_processor_2400_sr_capacity 1
#define _jtag_scalar_processor_2400_sr_type 2

#endif /* _scalar_processor_2400_cell_h */
