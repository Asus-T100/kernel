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

#include "tng_wa.h"

/* psb_intel_reg.h - for BIT* definitions */
#include "psb_intel_reg.h"

/* A0 Workarounds */

static void apply_HSD_4568161_4582997(struct drm_device *dev)
{
	/* HSD - 4568161: Local idle gating on SLC core clock
	   causes SLC to drop video mem request if clock is
	   heavily Throttling
	   Workaround: SW should set GFX_CG_DIS_0[8] at offset
	   0x160000 whenever powering up GFX */
	/* HSD 4582997: Trunk and idle clock gating must be disabled
	   Workaround: The driver should set GFX_CG_DIS_0[1:0] = 2'b11
	   at MMADR offset 0x160000 */

	uint32_t GFX_CG_DIS_0_OFFSET = 0x160000 - GFX_WRAPPER_OFFSET;
	uint32_t GFX_CG_DIS_0_DATA = WRAPPER_REG_READ(GFX_CG_DIS_0_OFFSET);

	GFX_CG_DIS_0_DATA |= (BIT0 | BIT1 | BIT8);

	WRAPPER_REG_WRITE(GFX_CG_DIS_0_OFFSET, GFX_CG_DIS_0_DATA);
}

static void apply_HSD_3940227_4568479(struct drm_device *dev)
{
	/* HSD - 3940227: SLC wrapper must support breaking 4K
	   crossing VSP brusts on SLC path for IMG sepc compliancy
	   Workaround: Read-Modify-Write to set GBYPASSENABLE[2]
	   and GBYPASSENABLE[8] on same write at MMADR offset
	   0x2850 before powering up VSP */
	/**
	 * HSD - 4568479: VED issues write fence cache hint incorrectly
	 * Workaround: Read-Modify-Write to set GBYPASSENABLE[0] and
	 * GBYPASSENABLE[8] on same write at MMADR offset 0x2850
	 * before powering up VSP
	 */
	uint32_t GBYPASSENABLE_OFFSET = 0x2850 - PSB_VDC_OFFSET;
	uint32_t GBYPASSENABLE_DATA = REG_READ(GBYPASSENABLE_OFFSET);

	GBYPASSENABLE_DATA |= (BIT0 | BIT2 | BIT8);

	REG_WRITE(GBYPASSENABLE_OFFSET, GBYPASSENABLE_DATA);
}

#if HSD_4568152
/* Not needed if apply 3940227 and 4568479 */
static void apply_HSD_4568152(struct drm_device *dev)
{
	/* HSD - 4568152: VP8 test fail with SLC due to bad cache policy
	   Workaround: SW must set RGX_CR_SLC_CTRL_BYPASS[6] at
	   MMADR offset 0x103828 before powering up VED for VP8 decode */
	uint32_t RGX_CR_SLC_CTRL_BYPASS_OFFSET = 0x103828 - RGX_OFFSET;
	uint32_t RGX_CR_SLC_CTRL_BYPASS_DATA = RGX_REG_READ(
					RGX_CR_SLC_CTRL_BYPASS_OFFSET);

	RGX_CR_SLC_CTRL_BYPASS_DATA |= BIT6;

	RGX_REG_WRITE(RGX_CR_SLC_CTRL_BYPASS_OFFSET,
			RGX_CR_SLC_CTRL_BYPASS_DATA);
}
#endif

static void apply_HSD_4568473(struct drm_device *dev)
{
	/* HSD 4568473: PFI credits
	   Workaround: GCLIP_CONTROL[19:16] should be programmed to
	   4'b1110 and GCILP_CONTROL[23] should be programmed to
	   1'b1 on same write (MMADDR offset 0x160020) */

	int GCLIP_CONTROL_OFFSET = 0x160020 - GFX_WRAPPER_OFFSET;
	int GCLIP_CONTROL_DATA = WRAPPER_REG_READ(GCLIP_CONTROL_OFFSET);

	GCLIP_CONTROL_DATA &= ~(BIT16);
	GCLIP_CONTROL_DATA |= (BIT17 | BIT18 | BIT19 | BIT23);

	WRAPPER_REG_WRITE(GCLIP_CONTROL_OFFSET, GCLIP_CONTROL_DATA);
}

static void apply_HSD_4582616(struct drm_device *dev)
{
	/* HSD 45682616: czclk remains gated for ~5us after pipeA
	   framestart in S0i1-display mode, could lead to underflows
	   Workaround: offset 70404[5] must be set to '1' when
	   Display is powered on */

	uint32_t DISPLAY_OFFSET = 0x70404 - PSB_VDC_OFFSET;
	uint32_t DISPLAY_DATA = REG_READ(DISPLAY_OFFSET);

	DISPLAY_DATA |= BIT5;

	REG_WRITE(DISPLAY_OFFSET, DISPLAY_DATA);
}

#if NO_HSD_WORKAROUND
/* No need as this is default setting of HW */
static void apply_NO_HSD_Workaround(struct drm_device *dev)
{
	/* HSD (Not specified): SLC brust disable
	   Workaround: RGX_CR_SLC_CTRL_MISC[0] at MMADR
	   offset 0x103800 must be programmed to 1'b1 */
	uint32_t RGX_CR_SLC_CTRL_MISC_OFFSET = 0x103800 - RGX_OFFSET;
	uint32_t RGX_CR_SLC_CTRL_MISC_DATA = RGX_REG_READ(
					RGX_CR_SLC_CTRL_MISC_OFFSET);

	RGX_CR_SLC_CTRL_MISC_DATA |= BIT0;

	RGX_REG_WRITE(RGX_CR_SLC_CTRL_MISC_OFFSET,
			RGX_CR_SLC_CTRL_MISC_DATA);
}
#endif

/* Apply the A0 Workaround */
void apply_A0_workarounds(int islands, int pre_po)
{
	struct drm_device *dev = gpDrmDevice;

	if (!dev)
		return;

	/* Only apply workaround on power up. */

	switch (islands) {
	case OSPM_DISPLAY_ISLAND:
		/*  When display is powered-on. */
		if (!pre_po)
			apply_HSD_4582616(dev);
		break;

	case OSPM_GRAPHICS_ISLAND:
		/* Before GFX is powered-on. */
		if (pre_po) {
			apply_HSD_4568161_4582997(dev);
			apply_HSD_4568473(dev);
		}
		break;

	case OSPM_VIDEO_ENC_ISLAND:
	case OSPM_VIDEO_VPP_ISLAND:
	case OSPM_VIDEO_DEC_ISLAND:
		/*  Before powering up VED for VP8 decode */
		if (pre_po)
			apply_HSD_3940227_4568479(dev);
		break;

	default:
		break;
	}
}
