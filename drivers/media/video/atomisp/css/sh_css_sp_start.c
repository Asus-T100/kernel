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
#ifdef CONFIG_X86_MRFLD
#define SYSTEM_hive_isp_css_2400_system
#endif

#include "sh_css_sp_start.h"
#include "sh_css_sp.h"
#include "sh_css_hw.h"
#include "sh_css_hrt.h"
#include "sh_css_firmware.h"

#ifdef SYSTEM_hive_isp_css_2400_system
#include "hrt_2400/cell.h"
#include "hrt_2400/sp.map.h"
#else
#include "hrt/sp.map.h"
#endif
static unsigned char *sp_dmem_base_address = SP_DMEM_BASE;
static bool invalidate_mmu;

void
sh_css_sp_invalidate_mmu(void)
{
	invalidate_mmu = true;
}

void
sh_css_sp_start(unsigned int start_address)
{
	if (invalidate_mmu) {
		sh_css_hrt_mmu_invalidate_cache();
		invalidate_mmu = false;
	}
	/* set the start address */
	sh_css_sp_ctrl_store(SP_START_ADDR_REG, start_address);
	/* set the run bit and the start bit with a read-modify-write */
	sh_css_sp_ctrl_set_bits(SP_SC_REG,
			1UL<<SP_START_BIT | 1UL<<SP_RUN_BIT);
}


void *
sh_css_sp_load_program(const struct sh_css_sp_fw *fw, const char *sp_prog,
		       void *code_addr, bool standard)
{
	if (!code_addr) {
		/* store code (text section) to DDR */
#if defined(SYSTEM_hive_isp_css_2400_system)
		code_addr = (void *)hrt_isp_css_mm_alloc(1);
	}
#else
		code_addr = hrt_isp_css_mm_alloc(fw->text_size);
		if (!code_addr)
			return NULL;
		hrt_isp_css_mm_store(code_addr, fw->text, fw->text_size);
	}
	if (standard) {
		struct sh_css_sp_fw *f = (struct sh_css_sp_fw *)fw;
		f->dmem_init_data = (void *)HIVE_ADDR_sp_init_dmem_data;
	}
#endif
	/* Set the correct start address for the SP program */
	sh_css_sp_activate_program(fw, code_addr, sp_prog);

	return code_addr;
}
#ifdef SYSTEM_hive_isp_css_2400_system
void
sh_css_sp_activate_program(const struct sh_css_sp_fw *fw,
			   void *code_addr,
			   const char *sp_prog)
{
	(void)fw; /* not used on csim, only on hw */
	hrt_cell_set_icache_base_address(SP, (unsigned long)code_addr);
	hrt_cell_invalidate_icache(SP);
	hrt_cell_load_program(SP, sp_prog);
}
#else
void
sh_css_sp_activate_program(const struct sh_css_sp_fw *fw,
				void *code_addr,
				const char *sp_prog)
{
	(void)sp_prog; /* not used on hardware, only for simulation */

	/* now we program the base address into the icache and
	 * invalidate the cache.
	 */
	sh_css_sp_ctrl_store(SP_ICACHE_ADDR_REG, (unsigned long)code_addr);
	sh_css_sp_ctrl_set_bits(SP_ICACHE_INV_REG, 1UL<<SP_ICACHE_INV_BIT);

	/* Set descr in the SP to initialize the SP DMEM */
	sh_css_sp_store_init_dmem(fw);
}
#endif

unsigned int
sh_css_sp_dmem_load_32(unsigned int address)
{
	unsigned long addr = (unsigned long)(sp_dmem_base_address + address);
	return hrt_master_port_uload_32(addr);
}

void
sh_css_sp_dmem_store_32(unsigned int address, unsigned int value)
{
	unsigned long addr = (unsigned long)(sp_dmem_base_address + address);
	hrt_master_port_store_32(addr, value);
}

void
sh_css_sp_dmem_load(unsigned int address, void *data, unsigned int bytes)
{
	unsigned long addr = (unsigned long)(sp_dmem_base_address + address);
	hrt_master_port_load(addr, data, bytes);
}

void
sh_css_sp_dmem_store(unsigned int address, const void *data, unsigned int bytes)
{
	unsigned long addr = (unsigned long)(sp_dmem_base_address + address);
	hrt_master_port_store(addr, data, bytes);
}
