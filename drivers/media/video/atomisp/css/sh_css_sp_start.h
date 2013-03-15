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

#ifndef _SH_CSS_SP_START_H_
#define _SH_CSS_SP_START_H_

#ifdef CONFIG_X86_MRFLD
#define SYSTEM_hive_isp_css_2400_system
#endif

#if defined(SYSTEM_hive_isp_css_2400_system)
#include "hrt_2400/cell.h"
#include "hrt_2400/host.h" 
#endif

#include "sh_css_firmware.h"

#if defined(SYSTEM_hive_isp_css_2400_system)

#ifdef PRINT_CSIM_STATS
#include "sh_css_statistics.h"
#define stat_new_binary() sh_css_statistics_frame(true)
#else
#define stat_new_binary() 
#endif

#define sh_css_sp_start_function(func) \
	do { \
		/* sh_css_print_statistics();*/ \
		stat_new_binary(); \
		hrt_cell_start_function(SP, func); \
		/* wait for input network to be ready */ \
		while (!sh_css_isp_has_started()) { \
			hrt_sleep(); \
		} \
	} while (0)
#else
#define sh_css_sp_start_function(func) \
	sh_css_sp_start(HIVE_ADDR_ ## func ## _entry)
#endif

/* int on SP is 4 bytes */
#define SP_INT_BYTES 4

#define store_sp_int(var, value) \
	sh_css_sp_dmem_store_32((unsigned) HIVE_ADDR_ ## var, \
				(unsigned int)(value))

#define store_sp_ptr(var, value) \
	sh_css_sp_dmem_store_32((unsigned) HIVE_ADDR_ ## var, \
				(unsigned int)(value))

#define load_sp_uint(var) \
	sh_css_sp_dmem_load_32((unsigned) HIVE_ADDR_ ## var)

#define load_sp_array_uint(array, index) \
	sh_css_sp_dmem_load_32((unsigned) HIVE_ADDR_ ## array + \
			       (index)*SP_INT_BYTES)

#define store_sp_array_uint(array, index, value) \
	sh_css_sp_dmem_store_32((unsigned) HIVE_ADDR_ ## array + \
				(index)*SP_INT_BYTES, value)

#define store_sp_var(var, data, bytes) \
	sh_css_sp_dmem_store((unsigned) HIVE_ADDR_ ## var, data, bytes)

#define SH_CSS_PREVENT_UNINIT_READS 0

unsigned int
sh_css_sp_dmem_load_32(unsigned int address);

void
sh_css_sp_dmem_store_32(unsigned int address, unsigned int value);

void
sh_css_sp_dmem_load(unsigned int address, void *data,
		    unsigned int bytes);

void
sh_css_sp_dmem_store(unsigned int address, const void *data,
		     unsigned int bytes);

void
sh_css_sp_start(unsigned int start_address);

void *
sh_css_sp_load_program(const struct sh_css_sp_fw *fw, const char *sp_prog,
		       void *code_addr, bool standard);

void
sh_css_sp_activate_program(const struct sh_css_sp_fw *fw,
			   void *code_addr,
			   const char *sp_prog);

void
sh_css_sp_invalidate_mmu(void);

#endif /* _SH_CSS_SP_START_H_ */
