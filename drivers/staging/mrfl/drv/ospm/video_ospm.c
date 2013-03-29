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
#include "psb_drv.h"
#include "pmu_tng.h"
#include "video_ospm.h"
#include "vsp.h"

/***********************************************************
 * vsp islands
 ***********************************************************/
/**
 * vsp_power_up
 *
 * Power up island
 */
static bool vsp_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int pm_ret = 0;

#if A0_WORKAROUNDS
	apply_A0_workarounds(OSPM_VIDEO_VPP_ISLAND, 1);
#endif

	pm_ret = pmu_set_power_state_tng(VSP_SS_PM0, VSP_SSC,
					 TNG_COMPOSITE_I0);
	if (pm_ret) {
		PSB_DEBUG_PM("VSP: pmu_set_power_state_tng ON failed!\n");
		return false;
	}
	/* Add the count to make sure the VSP don't be shut down
	 * when the count be decrease to 0.
	 */
	atomic_inc(&p_island->ref_count);

	PSB_DEBUG_PM("Power ON VSP!\n");
	return ret;
}

/**
 * vsp_power_down
 *
 * Power down island
 */
static bool vsp_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;
	int pm_ret = 0;

	/* whether the VSP is idle */
	if (psb_check_vsp_idle(dev)) {
		PSB_DEBUG_PM("The VSP isn't in idle!\n");
		return false;
	}

	pm_ret = pmu_set_power_state_tng(VSP_SS_PM0, VSP_SSC,
					 TNG_COMPOSITE_D3);
	if (pm_ret) {
		PSB_DEBUG_PM("VSP: pmu_set_power_state_tng OFF failed!\n");
		return false;
	}

	PSB_DEBUG_PM("Power OFF VSP!\n");
	return ret;
}

/**
 * ospm_vsp_init
 *
 * initilize
 */
void ospm_vsp_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	PSB_DEBUG_ENTRY("\n");

	p_island->p_funcs->power_up = vsp_power_up;
	p_island->p_funcs->power_down = vsp_power_down;
	p_island->p_dependency = NULL;
}

/***********************************************************
 * ved islands
 ***********************************************************/
/**
 * ved_power_up
 *
 * Power up island
 */
static bool ved_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	int pm_ret = 0;

	PSB_DEBUG_PM("powering up ved\n");
	pm_ret = pmu_set_power_state_tng(VED_SS_PM0, VED_SSC,
					 TNG_COMPOSITE_I0);
	if (pm_ret) {
		PSB_DEBUG_PM("power up ved failed\n");
		return false;
	}
	/* Add the count to make sure the VED don't be shut down
	 * when the count be decrease to 0.
	*/
	atomic_inc(&p_island->ref_count);

	return ret;
}

/**
 * ved_power_down
 *
 * Power down island
 */
static bool ved_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	int pm_ret = 0;

	PSB_DEBUG_PM("powering down ved\n");

	pm_ret = pmu_set_power_state_tng(VED_SS_PM0, VED_SSC,
					 TNG_COMPOSITE_D3);
	if (pm_ret) {
		PSB_DEBUG_PM("power down ved failed\n");
		return false;
	}

	return ret;
}

/**
 * ospm_ved_init
 *
 * initilize
 */
void ospm_ved_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	PSB_DEBUG_ENTRY("\n");

	p_island->p_funcs->power_up = ved_power_up;
	p_island->p_funcs->power_down = ved_power_down;
	p_island->p_dependency = NULL;
}

/***********************************************************
 * vec islands
 ***********************************************************/
/**
 * vec_power_up
 *
 * Power up island
 */
static bool vec_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	int pm_ret = 0;

	PSB_DEBUG_PM("powering up vec\n");
	pm_ret = pmu_set_power_state_tng(VEC_SS_PM0, VEC_SSC,
					 TNG_COMPOSITE_I0);
	if (pm_ret) {
		PSB_DEBUG_PM("power up vec failed\n");
		return false;
	}
	/* Add the count to make sure the VEC don't be shut down
	 * when the count be decrease to 0.
	 */
	atomic_inc(&p_island->ref_count);

	return ret;
}

/**
 * vec_power_down
 *
 * Power down island
 */
static bool vec_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret = true;
	int pm_ret = 0;

	PSB_DEBUG_PM("powering down vec\n");

	pm_ret = pmu_set_power_state_tng(VEC_SS_PM0, VEC_SSC,
					 TNG_COMPOSITE_D3);
	if (pm_ret) {
		PSB_DEBUG_PM("power down ved failed\n");
		return false;
	}

	return ret;
}

/**
 * ospm_vec_init
 *
 * initilize
 */
void ospm_vec_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	PSB_DEBUG_ENTRY("\n");

	p_island->p_funcs->power_up = vec_power_up;
	p_island->p_funcs->power_down = vec_power_down;
	p_island->p_dependency = NULL;
}
