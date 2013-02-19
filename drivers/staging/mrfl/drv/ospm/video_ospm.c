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

	PSB_DEBUG_ENTRY("\n");

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

	PSB_DEBUG_ENTRY("\n");

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

	PSB_DEBUG_ENTRY("\n");

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

	PSB_DEBUG_ENTRY("\n");

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
	bool ret;

	PSB_DEBUG_ENTRY("\n");

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
	bool ret;

	PSB_DEBUG_ENTRY("\n");

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
