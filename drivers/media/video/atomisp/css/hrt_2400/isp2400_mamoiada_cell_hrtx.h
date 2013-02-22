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

#ifndef _isp2400_mamoiada_cell_hrtx_h
#define _isp2400_mamoiada_cell_hrtx_h

/* Identifiers to be used as arguments for hrt functions*/
enum isp2400_mamoiada_memories {
  isp2400_mamoiada_base_config_mem,
  isp2400_mamoiada_base_dmem,
  isp2400_mamoiada_base_fifo,
  isp2400_mamoiada_simd_vmem,
  isp2400_mamoiada_simd_vamem1,
  isp2400_mamoiada_simd_vamem2,
  isp2400_mamoiada_simd_vamem3,
  isp2400_mamoiada_simd_histogram,
  isp2400_mamoiada_num_memories
};

enum isp2400_mamoiada_slave_ports {
  isp2400_mamoiada_sl_ipstatctrl_cell_port,
  isp2400_mamoiada_sl_ipdmem_cell_port,
  isp2400_mamoiada_sl_ippmem_cell_port,
  isp2400_mamoiada_sl_ipvamem_cell_port,
  isp2400_mamoiada_sl_iphist_cell_port,
  isp2400_mamoiada_simd_ipdma_cell_port,
  isp2400_mamoiada_num_slave_ports,
  isp2400_mamoiada_unreachable_cell_port
};

enum isp2400_mamoiada_master_interfaces {
  isp2400_mamoiada_base_config_mem_master,
  isp2400_mamoiada_num_master_interfaces
};

enum isp2400_mamoiada_arbiters {
  isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp,
  isp2400_mamoiada_base_dmem_Arb_data_mem_wp0,
  isp2400_mamoiada_num_arbiters
};

enum isp2400_mamoiada_arbiter_contenders {
  isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source1,
  isp2400_mamoiada_base_config_mem_Arb_prg_mem_wp_source0,
  isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source1,
  isp2400_mamoiada_base_dmem_Arb_data_mem_wp0_source0,
  isp2400_mamoiada_num_arbiter_contenders
};

#endif /* _isp2400_mamoiada_cell_hrtx_h */
