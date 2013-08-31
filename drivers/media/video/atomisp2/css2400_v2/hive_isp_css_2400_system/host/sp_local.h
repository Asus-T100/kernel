/* Release Version: ci_master_byt_20130823_2200 */
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

#ifndef __SP_LOCAL_H_INCLUDED__
#define __SP_LOCAL_H_INCLUDED__

#ifndef __KERNEL__
#include <stdbool.h>
#endif

#include "sp_global.h"

struct sp_state_s {
	int		pc;
	int		status_register;
	bool	is_broken;
	bool	is_idle;
	bool	is_sleeping;
	bool	is_stalling;
};

struct sp_stall_s {
	bool	fifo0;
	bool	fifo1;
	bool	fifo2;
	bool	fifo3;
	bool	fifo4;
	bool	fifo5;
	bool	fifo6;
	bool	fifo7;
	bool	fifo8;
	bool	fifo9;
	bool	fifoa;
	bool	dmem;
	bool	control_master;
	bool	icache_master;
};

#if !defined(HRT_RTL) && !defined(HRT_HW) && !defined(HRT_FPGA) && !defined(HRT_PD)
#include "hive_isp_css_sp_hrt.h"
#define sp_address_of(var)	_sp_var_addr(var)
#else
#define sp_address_of(var)	(HIVE_ADDR_ ## var)
#endif

/*
 * deprecated
 */
#define store_sp_int(var, value) \
	sp_dmem_store_uint32(SP0_ID, (unsigned)sp_address_of(var), \
		(uint32_t)(value))

#define store_sp_ptr(var, value) \
	sp_dmem_store_uint32(SP0_ID, (unsigned)sp_address_of(var), \
		(uint32_t)(value))

#define load_sp_uint(var) \
	sp_dmem_load_uint32(SP0_ID, (unsigned)sp_address_of(var))

#define load_sp_array_uint8(array_name, index) \
	sp_dmem_load_uint8(SP0_ID, (unsigned)sp_address_of(array_name) + \
		(index)*sizeof(uint8_t))

#define load_sp_array_uint16(array_name, index) \
	sp_dmem_load_uint16(SP0_ID, (unsigned)sp_address_of(array_name) + \
		(index)*sizeof(uint16_t))

#define load_sp_array_uint(array_name, index) \
	sp_dmem_load_uint32(SP0_ID, (unsigned)sp_address_of(array_name) + \
		(index)*sizeof(uint32_t))

#define store_sp_var(var, data, bytes) \
	sp_dmem_store(SP0_ID, (unsigned)sp_address_of(var), data, bytes)

#define store_sp_array_uint8(array_name, index, value) \
	sp_dmem_store_uint8(SP0_ID, (unsigned)sp_address_of(array_name) + \
		(index)*sizeof(uint8_t), value)

#define store_sp_array_uint16(array_name, index, value) \
	sp_dmem_store_uint16(SP0_ID, (unsigned)sp_address_of(array_name) + \
		(index)*sizeof(uint16_t), value)

#define store_sp_array_uint(array_name, index, value) \
	sp_dmem_store_uint32(SP0_ID, (unsigned)sp_address_of(array_name) + \
		(index)*sizeof(uint32_t), value)

#define store_sp_var_with_offset(var, offset, data, bytes) \
	sp_dmem_store(SP0_ID, (unsigned)sp_address_of(var) + \
		offset, data, bytes)

#define load_sp_var(var, data, bytes) \
	sp_dmem_load(SP0_ID, (unsigned)sp_address_of(var), data, bytes)

#define load_sp_var_with_offset(var, offset, data, bytes) \
	sp_dmem_load(SP0_ID, (unsigned)sp_address_of(var) + offset, \
		data, bytes)

#endif /* __SP_LOCAL_H_INCLUDED__ */
