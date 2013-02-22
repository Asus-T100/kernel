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

#ifndef _scalar_processor_2400_cell_hrtx_h
#define _scalar_processor_2400_cell_hrtx_h

/* Identifiers to be used as arguments for hrt functions*/
enum scalar_processor_2400_memories {
  scalar_processor_2400_fifo,
  scalar_processor_2400_dmem,
  scalar_processor_2400_xmem,
  scalar_processor_2400_config_icache_conf_icache,
  scalar_processor_2400_num_memories
};

enum scalar_processor_2400_slave_ports {
  scalar_processor_2400_sl_ip0_cell_port,
  scalar_processor_2400_sl_ip1_cell_port,
  scalar_processor_2400_num_slave_ports,
  scalar_processor_2400_unreachable_cell_port
};

enum scalar_processor_2400_master_interfaces {
  scalar_processor_2400_xmem_master_int,
  scalar_processor_2400_config_icache_conf_icache_master,
  scalar_processor_2400_num_master_interfaces
};

enum scalar_processor_2400_arbiters {
  scalar_processor_2400_dmem_Arb_mem_wp,
  scalar_processor_2400_num_arbiters
};

enum scalar_processor_2400_arbiter_contenders {
  scalar_processor_2400_dmem_Arb_mem_wp_source1,
  scalar_processor_2400_dmem_Arb_mem_wp_source0,
  scalar_processor_2400_num_arbiter_contenders
};

#endif /* _scalar_processor_2400_cell_hrtx_h */
