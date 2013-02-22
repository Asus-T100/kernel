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

#ifndef _HRT_MASTER_PORT_H_
#define _HRT_MASTER_PORT_H_

/* This file contains the end of the HRT.
 * Here we split between the hardware implementation (memcpy / assignments)
 * and the software backends (_hrt_master_port_load / _hrt_master_port_store)
 */

/* If HRT_USE_VIR_ADDRS is defined, the OS needs to implement the following
   functions for us:
   _hrt_master_port_store_8(addr,data)
   _hrt_master_port_store_16(addr,data)
   _hrt_master_port_store_32(addr,data)
   _hrt_master_port_load_8(addr)
   _hrt_master_port_uload_8(addr)
   _hrt_master_port_load_16(addr)
   _hrt_master_port_uload_16(addr)
   _hrt_master_port_load_32(addr)
   _hrt_master_port_uload_32(addr)
   _hrt_master_port_store_8_volatile(addr,data)
   _hrt_master_port_load_8_volatile(addr)
   _hrt_master_port_uload_8_volatile(addr)
   _hrt_master_port_store_16_volatile(addr,data)
   _hrt_master_port_load_16_volatile(addr)
   _hrt_master_port_uload_16_volatile(addr)
   _hrt_master_port_store_32_volatile(addr,data)
   _hrt_master_port_load_32_volatile(addr)
   _hrt_master_port_uload_32_volatile(addr)
   _hrt_mem_store(addr,data,size)
   _hrt_mem_load(addr,data,size)
   _hrt_mem_set(addr,val,size)
*/

#define hrt_master_port_store_8(addr, data) \
	_hrt_master_port_store_8(addr, data)
#define hrt_master_port_store_16(addr, data) \
	_hrt_master_port_store_16(addr, data)
#define hrt_master_port_store_32(addr, data) \
	_hrt_master_port_store_32(addr, data)
#define hrt_master_port_load_8(addr) \
	_hrt_master_port_load_8(addr)
#define hrt_master_port_load_16(addr) \
	_hrt_master_port_load_16(addr)
#define hrt_master_port_load_32(addr) \
	_hrt_master_port_load_32(addr)
#define hrt_master_port_uload_8(addr) \
	_hrt_master_port_uload_8(addr)
#define hrt_master_port_uload_16(addr) \
	_hrt_master_port_uload_16(addr)
#define hrt_master_port_uload_32(addr) \
	_hrt_master_port_uload_32(addr)

#define hrt_master_port_store(addr, data, bytes) \
	_hrt_master_port_unaligned_store((void *)(addr), \
					(const void *)(data), bytes)
#define hrt_master_port_load(addr, data, bytes) \
	_hrt_master_port_unaligned_load((const void *)(addr), \
					(void *)(data), bytes)
#define hrt_master_port_set(addr, data, bytes) \
	_hrt_master_port_unaligned_set((void *)(addr), \
					(int)(data), bytes)

#ifdef HRT_HW
/* on real hardware, we cannot print messages, so we get rid of them here. */
#define _hrt_master_port_unaligned_store_msg(a, d, s, m) \
	_hrt_master_port_unaligned_store(a, d, s)
#define _hrt_master_port_unaligned_load_msg(a, d, s, m) \
	_hrt_master_port_unaligned_load(a, d, s)
#define _hrt_master_port_unaligned_set_msg(a, d, s, m) \
	_hrt_master_port_unaligned_set(a, d, s)
#define _hrt_master_port_store_8_msg(a, d, m) \
	_hrt_master_port_store_8(a, d)
#define _hrt_master_port_store_16_msg(a, d, m) \
	_hrt_master_port_store_16(a, d)
#define _hrt_master_port_store_32_msg(a, d, m) \
	_hrt_master_port_store_32(a, d)
#define _hrt_master_port_load_8_msg(a, m) \
	_hrt_master_port_load_8(a)
#define _hrt_master_port_load_16_msg(a, m) \
	_hrt_master_port_load_16(a)
#define _hrt_master_port_load_32_msg(a, m) \
	_hrt_master_port_load_32(a)
#define _hrt_master_port_uload_8_msg(a, m) \
	_hrt_master_port_uload_8(a)
#define _hrt_master_port_uload_16_msg(a, m) \
	_hrt_master_port_uload_16(a)
#define _hrt_master_port_uload_32_msg(a, m) \
	_hrt_master_port_uload_32(a)
#define _hrt_master_port_store_8_volatile_msg(a, d, m) \
	_hrt_master_port_store_8_volatile(a, d)
#define _hrt_master_port_load_8_volatile_msg(a, m) \
	_hrt_master_port_load_8_volatile(a)
#define _hrt_master_port_uload_8_volatile_msg(a, m) \
	_hrt_master_port_uload_8_volatile(a)
#define _hrt_master_port_store_16_volatile_msg(a, d, m) \
	_hrt_master_port_store_16_volatile(a, d)
#define _hrt_master_port_load_16_volatile_msg(a, m) \
	_hrt_master_port_load_16_volatile(a)
#define _hrt_master_port_uload_16_volatile_msg(a, m) \
	_hrt_master_port_uload_16_volatile(a)
#define _hrt_master_port_store_32_volatile_msg(a, d, m) \
	_hrt_master_port_store_32_volatile(a, d)
#define _hrt_master_port_load_32_volatile_msg(a, m) \
	_hrt_master_port_load_32_volatile(a)
#define _hrt_master_port_uload_32_volatile_msg(a, m) \
	_hrt_master_port_uload_32_volatile(a)
/* reduce number of functions */
#define _hrt_master_port_unaligned_store(address, data, size) \
	_hrt_mem_store(address, data, size)
#define _hrt_master_port_unaligned_load(address, data, size) \
	_hrt_mem_load(address, data, size)
#define _hrt_master_port_unaligned_set(address, data, size) \
	_hrt_mem_set(address, data, size)
#endif /* HRT_HW */

#if defined(__HIVECC)
#include "master_port_hivecc.h"
#elif defined(HRT_USE_VIR_ADDRS)
/* do nothing, hrt backend is already included */
#elif defined(HRT_HW)
#include "master_port_hw.h"
#else
#include "master_port_sim.h"
#endif

#endif /* _HRT_MASTER_PORT_H_ */
