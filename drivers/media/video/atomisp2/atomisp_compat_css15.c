/*
 * Support for Clovertrail PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#include "sh_css_debug.h"
#include "host/mmu_local.h"
#include "device_access/device_access.h"
#include "memory_access/memory_access.h"

#include "atomisp_compat.h"
#include "atomisp_internal.h"
#include "atomisp_cmd.h"

#define CSS_DTRACE_VERBOSITY_LEVEL	5	/* Controls trace verbosity */

int atomisp_css_init(struct atomisp_device *isp,
			struct atomisp_css_env *atomisp_css_env)
{
	device_set_base_address(0);

	/*
	 * if the driver gets closed and reopened, the HMM is not reinitialized
	 * This means we need to put the L1 page table base address back into
	 * the ISP
	 */
	if (isp->mmu_l1_base)
		/*
		 * according to sh_css.c sh_css_mmu_set_page_table_base_index
		 * is deprecated and mmgr_set_base_address should be used
		 * instead. But just for now (with CSS "alpha") replacing
		 * all sh_cssh_mmu_set_page_table_base_index() -calls
		 * with mmgr_set_base_address() is not working.
		 */
		sh_css_mmu_set_page_table_base_index(
				HOST_ADDRESS(isp->mmu_l1_base));

	/* With CSS "alpha" it is mandatory to set base address always */
	mmgr_set_base_address(HOST_ADDRESS(isp->mmu_l1_base));

	/* Init ISP */
	if (sh_css_init(&atomisp_css_env->isp_css_env,
			isp->firmware->data, isp->firmware->size)) {
		dev_err(isp->dev, "css init failed --- bad firmware?\n");
		return -EINVAL;
	}

	/* CSS has default zoom factor of 61x61, we want no zoom
	   because the zoom binary for capture is broken (XNR). */
	if (IS_ISP2400)
		sh_css_set_zoom_factor(MRFLD_MAX_ZOOM_FACTOR,
					MRFLD_MAX_ZOOM_FACTOR);
	else
		sh_css_set_zoom_factor(MFLD_MAX_ZOOM_FACTOR,
					MFLD_MAX_ZOOM_FACTOR);

	/* Initialize the CSS debug trace verbosity level. To change
	 * the verbosity level, change the definition of this macro
	 * up in the file
	 */
	sh_css_set_dtrace_level(CSS_DTRACE_VERBOSITY_LEVEL);

	dev_dbg(isp->dev, "sh_css_init success\n");

	return 0;
}

void atomisp_set_css_env(const struct firmware *firmware,
			struct atomisp_css_env *atomisp_css_env)
{
	atomisp_css_env->isp_css_env = sh_css_default_env();
	atomisp_css_env->isp_css_env.sh_env.alloc = atomisp_kernel_zalloc;
	atomisp_css_env->isp_css_env.sh_env.free = atomisp_kernel_free;
}
