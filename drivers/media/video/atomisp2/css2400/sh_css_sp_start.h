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

#define SH_CSS_PREVENT_UNINIT_READS 0

#define __INLINE_SP__
#include "sp.h"		/* hrt_vaddress */

#include "sh_css_firmware.h"

#if defined(FIST_CTRL)
#define sh_css_sp_start_function(func) \
	do { \
		sh_css_sp_start(HIVE_ADDR_ ## func ## _entry)
		/* wait for input network to be ready */ \
		while (!sh_css_isp_has_started()) { \
			hrt_sleep(); \
		} \
	} while (0)
#else
#define sh_css_sp_start_function(func) \
	sh_css_sp_start(HIVE_ADDR_ ## func ## _entry)
#endif

void
sh_css_sp_start(unsigned int start_address);

hrt_vaddress
sh_css_sp_load_program(const struct sh_css_fw_info *fw, const char *sp_prog,
		       hrt_vaddress code_addr);

void
sh_css_sp_activate_program(const struct sh_css_fw_info *fw,
			   hrt_vaddress code_addr,
			   const char *sp_prog);

void
sh_css_sp_invalidate_mmu(void);

#endif /* _SH_CSS_SP_START_H_ */
