/**************************************************************************
 * Copyright (c) 2012, Intel Corporation.
 * All Rights Reserved.

 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Hitesh K. Patel <hitesh.k.patel@intel.com>
 */

#include <linux/kernel.h>
#include <linux/printk.h>
#include <linux/delay.h>
#include <asm/intel-mid.h>

#include "psb_drv.h"
#include "gfx_ospm.h"
#include "pmu_tng.h"
#include "tng_wa.h"

#define	USE_GFX_PM_FUNC			0

/* WRAPPER Offset 0x160024 */
#define GFX_STATUS_OFFSET		0x24

/* define which island to power down */
#define GFX_ALL		(GFX_SLC_LDO_SSC | \
					GFX_SLC_SSC | \
					GFX_SDKCK_SSC | \
					GFX_RSCD_SSC)

#define GFX_POWER_UP(x) \
	pmu_nc_set_power_state(x, OSPM_ISLAND_UP, GFX_SS_PM0)

#define GFX_POWER_DOWN(x) \
	pmu_nc_set_power_state(x, OSPM_ISLAND_DOWN, GFX_SS_PM0)

static u32 gfx_island_selected = GFX_SLC_LDO_SSC | GFX_SLC_SSC |
	GFX_SDKCK_SSC | GFX_RSCD_SSC;

enum GFX_ISLAND_STATUS {
	POWER_ON = 0,		/* No gating (clk or power) */
	CLOCK_GATED,		/* Clock Gating */
	SOFT_RESET,		/* Soft Reset */
	POWER_OFF,		/* Powered off or Power gated.*/
};

static int pm_cmd_freq_wait(u32 reg_freq)
{
	int tcount;
	u32 freq_val;

	for (tcount = 0; ; tcount++) {
		freq_val = intel_mid_msgbus_read32(PUNIT_PORT, reg_freq);
		if ((freq_val & IP_FREQ_VALID) == 0)
			break;
		if (tcount > 500) {
			WARN(1, "%s: P-Unit freq request wait timeout",
				__func__);
			return -EBUSY;
		}
		udelay(1);
	}

	return 0;
}

static int pm_cmd_freq_set(u32 reg_freq, u32 freq_code)
{
	u32 freq_val;
	int rva;

	pm_cmd_freq_wait(reg_freq);

	freq_val = IP_FREQ_VALID | freq_code;
	intel_mid_msgbus_write32(PUNIT_PORT, reg_freq, freq_val);

	rva = pm_cmd_freq_wait(reg_freq);

	return rva;
}

int set_gpu_freq(u32 freq_code)
{
	return pm_cmd_freq_set(GFX_SS_PM1, freq_code);
}

/***********************************************************
 * All Graphics Island
 ***********************************************************/
static bool first_boot = true;
/**
 * ospm_gfx_power_up
 *
 * Power up graphics islands
 * Sequence & flow from SAS
 */
static bool ospm_gfx_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	u32 gfx_all = gfx_island_selected;

	OSPM_DPF("Pre-power-up status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	if (first_boot) {
		gfx_all = GFX_ALL;
		first_boot = false;
	}

	if (gfx_all & GFX_SLC_LDO_SSC)
		ret = GFX_POWER_UP(PMU_LDO);

	if (gfx_all & GFX_SLC_SSC)
		ret = GFX_POWER_UP(PMU_SLC);

#if A0_WORKAROUNDS
	/**
	 * If turning some power on, and the power to be on includes SLC,
	 * and SLC was not previously on, then setup some registers.
	 */
	if (gfx_all & GFX_SLC_SSC)
		apply_A0_workarounds(OSPM_GRAPHICS_ISLAND, 1);
#endif

	if (gfx_all & GFX_SDKCK_SSC)
		ret = GFX_POWER_UP(PMU_SDKCK);

	if (gfx_all & GFX_RSCD_SSC)
		ret = GFX_POWER_UP(PMU_RSCD);

	OSPM_DPF("Post-power-up status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	return !ret;
}

/**
 * ospm_gfx_power_down
 *
 * Power down Graphics islands
 * Sequence & flow from SAS
 */
static bool ospm_gfx_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;

	OSPM_DPF("Pre-power-off Status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	/* power down every thing */
	if (gfx_island_selected & GFX_RSCD_SSC)
		ret = GFX_POWER_DOWN(PMU_RSCD);

	if (gfx_island_selected & GFX_SDKCK_SSC)
		ret = GFX_POWER_DOWN(PMU_SDKCK);

	if (gfx_island_selected & GFX_SLC_SSC)
		ret = GFX_POWER_DOWN(PMU_SLC);

	if (gfx_island_selected & GFX_SLC_LDO_SSC)
		ret = GFX_POWER_DOWN(PMU_LDO);

	OSPM_DPF("Post-power-off Status = 0x%08lX\n",
		intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS));

	return !ret;
}

/**
 * ospm_gfx_init
 *
 * Graphics power island init
 */
void ospm_gfx_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	OSPM_DPF("%s\n", __func__);
	p_island->p_funcs->power_up = ospm_gfx_power_up;
	p_island->p_funcs->power_down = ospm_gfx_power_down;
	p_island->p_dependency = NULL;
}
