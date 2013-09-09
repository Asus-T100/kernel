/* Release Version: ci_master_byt_20130905_2200 */
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

#include "sh_css_sp_start.h"
#include "sh_css_sp.h"
#include "sh_css_firmware.h"

#define __INLINE_SP__
#include "sp.h"

#include "mmu_device.h"

#include "memory_access.h"

#include "assert_support.h"

static bool invalidate_mmu;
static struct ia_css_sp_init_dmem_cfg init_dmem_cfg;
static uint32_t init_dmem_data;

void
sh_css_sp_invalidate_mmu(void)
{
	invalidate_mmu = true;
}

void sh_css_sp_start(
	unsigned int start_address)
{
	assert(sizeof(unsigned int) <= sizeof(hrt_data));

	if (invalidate_mmu) {
		mmu_ID_t	mmu_id;
		for (mmu_id = (mmu_ID_t)0;mmu_id < N_MMU_ID; mmu_id++) {
			mmu_invalidate_cache(mmu_id);
		}
		invalidate_mmu = false;
	}
	sp_dmem_store(SP0_ID, init_dmem_data, &init_dmem_cfg, sizeof(init_dmem_cfg));
	/* set the start address */
	sp_ctrl_store(SP0_ID, SP_START_ADDR_REG, (hrt_data)start_address);
	sp_ctrl_setbit(SP0_ID, SP_SC_REG, SP_RUN_BIT);
	sp_ctrl_setbit(SP0_ID, SP_SC_REG, SP_START_BIT);
}

hrt_vaddress sh_css_sp_load_program(
	const struct ia_css_fw_info *fw,
	const char *sp_prog,
	hrt_vaddress code_addr)
{
	if (code_addr == mmgr_NULL) {
		/* store code (text + icache) and data to DDR
		 *
		 * Data used to be stored separately, because of access alignment constraints,
		 * fix the FW generation instead
		 */
		code_addr = mmgr_malloc(fw->blob.size);
		if (code_addr == mmgr_NULL)
			return code_addr;
		mmgr_store(code_addr, fw->blob.code, fw->blob.size);

		assert((fw->blob.icache_source % HIVE_ISP_DDR_WORD_BYTES) == 0);
		assert((fw->blob.text_source % HIVE_ISP_DDR_WORD_BYTES) == 0);
		assert((fw->blob.data_source % HIVE_ISP_DDR_WORD_BYTES) == 0);
	}

	/* Set the correct start address for the SP program */
	sh_css_sp_activate_program(fw, code_addr, sp_prog);

	return code_addr;
}

void sh_css_sp_activate_program(
	const struct ia_css_fw_info *fw,
	hrt_vaddress code_addr,
	const char *sp_prog)
{
	(void)sp_prog; /* not used on hardware, only for simulation */

	assert(sizeof(hrt_vaddress) <= sizeof(hrt_data));

	/* now we program the base address into the icache and
	 * invalidate the cache.
	 */
	sp_ctrl_store(SP0_ID, SP_ICACHE_ADDR_REG, (hrt_data)code_addr);
	sp_ctrl_setbit(SP0_ID, SP_ICACHE_INV_REG, SP_ICACHE_INV_BIT);

	/* Set descr in the SP to initialize the SP DMEM */
	/*
	 * The FW stores user-space pointers to the FW, the ISP pointer
	 * is only available here
	 *
	 */

	/* Configure the data structure to initialize dmem */
	init_dmem_cfg.done	     = false;
	init_dmem_cfg.ddr_code_addr  = code_addr;
	init_dmem_cfg.ddr_data_addr  = code_addr + fw->blob.data_source;
	init_dmem_cfg.pmem_text_addr = 0; /* fw->blob.text_target; */
	init_dmem_cfg.dmem_data_addr = fw->blob.data_target;
	init_dmem_cfg.dmem_bss_addr  = fw->blob.bss_target;
	init_dmem_cfg.text_size      = fw->blob.text_size;
	init_dmem_cfg.data_size      = fw->blob.data_size;
	init_dmem_cfg.bss_size       = fw->blob.bss_size;

	init_dmem_data =  fw->info.sp.init_dmem_data;
}
