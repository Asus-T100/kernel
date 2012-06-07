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

#include "sh_css_sp_start.h"
#include "sh_css_sp.h"
#include "sh_css_hw.h"
#include "sh_css_hrt.h"
#include "sp.map.h"
#include "sh_css_firmware.h"

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
		       void *code_addr)
{
	if (!code_addr) {
		/* store code (text section) to DDR */
		code_addr = hrt_isp_css_mm_alloc(fw->text_size);
		if (!code_addr)
			return NULL;
		hrt_isp_css_mm_store(code_addr, fw->text, fw->text_size);
	}

	/* Set the correct start address for the SP program */
	sh_css_sp_activate_program(fw, code_addr, sp_prog);

	return code_addr;
}

	void
sh_css_sp_activate_program(const struct sh_css_sp_fw *fw,
				void *code_addr,
				const char *sp_prog)
{
	struct sh_css_sp_init_dmem_cfg init_dmem_cfg;
	void *data_addr;
	(void)sp_prog; /* not used on hardware, only for simulation */

	/* now we program the base address into the icache and
	 * invalidate the cache.
	 */
	sh_css_sp_ctrl_store(SP_ICACHE_ADDR_REG, (unsigned long)code_addr);
	sh_css_sp_ctrl_set_bits(SP_ICACHE_INV_REG, 1UL<<SP_ICACHE_INV_BIT);

	/* store data section to DDR */
	data_addr = hrt_isp_css_mm_alloc(fw->data_size);
	hrt_isp_css_mm_store(data_addr, fw->data, fw->data_size);

	/* Configure the data structure to initialize dmem */
	init_dmem_cfg.ddr_data_addr  = data_addr;
	init_dmem_cfg.dmem_data_addr = (void *) fw->data_target;
	init_dmem_cfg.data_size      = fw->data_size;
	init_dmem_cfg.dmem_bss_addr  = (void *) fw->bss_target;
	init_dmem_cfg.bss_size       = fw->bss_size;

	/* Start function on the SP to initialize the SP DMEM */
	sh_css_sp_start_init_dmem(&init_dmem_cfg);

	hrt_isp_css_mm_free(data_addr);
}


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
