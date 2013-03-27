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
#include "dc_ospm.h"
#include "pmu_tng.h"

/***********************************************************
 * display A Island implementation
 ***********************************************************/

/**
 * disp_a_power_up
 *
 * Power up island : return true if success
 */
static bool disp_a_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_DISP_A, OSPM_ISLAND_UP, DSP_SS_PM);

#if A0_WORKAROUNDS
	if (!ret)
		apply_A0_workarounds(OSPM_DISPLAY_ISLAND, 0);
#endif

	OSPM_DPF("Power on island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * disp_a_power_down
 *
 * Power down island : return true if success
 */
static bool disp_a_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_DISP_A, OSPM_ISLAND_DOWN, DSP_SS_PM);

	OSPM_DPF("Power off island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * ospm_disp_a_init
 *
 * initilize
 */
void ospm_disp_a_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	p_island->p_funcs->power_up = disp_a_power_up;
	p_island->p_funcs->power_down = disp_a_power_down;
	p_island->p_dependency = NULL;
}

/***********************************************************
 * display B Island implementation
 ***********************************************************/
/**
 * disp_b_power_up
 *
 * Power up island : return true if success
 */
static bool disp_b_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_DISP_B, OSPM_ISLAND_UP, DSP_SS_PM);

	OSPM_DPF("Power on island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * disp_b_power_down
 *
 * Power down island : return true if success
 */
static bool disp_b_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_DISP_B, OSPM_ISLAND_DOWN, DSP_SS_PM);

	OSPM_DPF("Power off island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * ospm_disp_b_init
 *
 * initilize
 */
void ospm_disp_b_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	p_island->p_funcs->power_up = disp_b_power_up;
	p_island->p_funcs->power_down = disp_b_power_down;
	p_island->p_dependency = NULL;
}

/***********************************************************
 * display C Island implementation
 ***********************************************************/
/**
 * disp_c_power_up
 *
 * Power up island : return true if success
 */
static bool disp_c_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_DISP_C, OSPM_ISLAND_UP, DSP_SS_PM);

	OSPM_DPF("Power on island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * disp_c_power_down
 *
 * Power down island : return true if success
 */
static bool disp_c_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_DISP_C, OSPM_ISLAND_DOWN, DSP_SS_PM);

	OSPM_DPF("Power off island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * ospm_disp_c_init
 *
 * initilize
 */
void ospm_disp_c_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	p_island->p_funcs->power_up = disp_c_power_up;
	p_island->p_funcs->power_down = disp_c_power_down;
	p_island->p_dependency = NULL;
}

/***********************************************************
 * display MIO Island implementation
 ***********************************************************/
/**
 * mio_power_up
 *
 * Power up island : return true if success
 */
static bool mio_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_MIO, OSPM_ISLAND_UP, MIO_SS_PM);

	OSPM_DPF("Power on island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * mio_power_down
 *
 * Power down island : return true if success
 */
static bool mio_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_MIO, OSPM_ISLAND_DOWN, MIO_SS_PM);

	OSPM_DPF("Power off island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * ospm_mio_init
 *
 * initilize
 */
void ospm_mio_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	p_island->p_funcs->power_up = mio_power_up;
	p_island->p_funcs->power_down = mio_power_down;
	p_island->p_dependency = NULL;
}

/***********************************************************
 * display HDMI Island implementation
 ***********************************************************/
/**
 * hdmi_power_up
 *
 * Power up island : return true if success
 */
static bool hdmi_power_up(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_HDMI, OSPM_ISLAND_UP, HDMIO_SS_PM);

	OSPM_DPF("Power on island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * hdmi_power_down
 *
 * Power down island : return true if success
 */
static bool hdmi_power_down(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	bool ret;

	ret = pmu_nc_set_power_state(PMU_HDMI, OSPM_ISLAND_DOWN, HDMIO_SS_PM);

	OSPM_DPF("Power off island %x, returned %d\n", p_island->island, ret);

	return !ret;
}

/**
 * ospm_hdmi_init
 *
 * initilize
 */
void ospm_hdmi_init(struct drm_device *dev,
			struct ospm_power_island *p_island)
{
	p_island->p_funcs->power_up = hdmi_power_up;
	p_island->p_funcs->power_down = hdmi_power_down;
	p_island->p_dependency = NULL;
}
