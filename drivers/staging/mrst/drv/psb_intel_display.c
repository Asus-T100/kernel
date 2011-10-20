/*
 * Copyright Â© 2006-2007 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Authors:
 *	Eric Anholt <eric@anholt.net>
 */

#include <linux/i2c.h>
#include <linux/pm_runtime.h>

#include <drm/drmP.h>
#include "psb_fb.h"
#include "psb_drv.h"
#include "psb_intel_drv.h"
#include "psb_intel_reg.h"
#include "psb_intel_display.h"
#include "psb_powermgmt.h"

/*MDFLD defines */
static int mdfld_crtc_mode_set(struct drm_crtc *crtc,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode,
			      int x, int y,
			      struct drm_framebuffer *old_fb);
static void mdfld_crtc_dpms(struct drm_crtc *crtc, int mode);
/*MDFLD defines end */

struct psb_intel_clock_t {
	/* given values */
	int n;
	int m1, m2;
	int p1, p2;
	/* derived values */
	int dot;
	int vco;
	int m;
	int p;
};

struct psb_intel_range_t {
	int min, max;
};

struct psb_intel_p2_t {
	int dot_limit;
	int p2_slow, p2_fast;
};

#define INTEL_P2_NUM		      2

struct psb_intel_limit_t {
	struct psb_intel_range_t dot, vco, n, m, m1, m2, p, p1;
	struct psb_intel_p2_t p2;
};

#define I8XX_DOT_MIN		  25000
#define I8XX_DOT_MAX		 350000
#define I8XX_VCO_MIN		 930000
#define I8XX_VCO_MAX		1400000
#define I8XX_N_MIN		      3
#define I8XX_N_MAX		     16
#define I8XX_M_MIN		     96
#define I8XX_M_MAX		    140
#define I8XX_M1_MIN		     18
#define I8XX_M1_MAX		     26
#define I8XX_M2_MIN		      6
#define I8XX_M2_MAX		     16
#define I8XX_P_MIN		      4
#define I8XX_P_MAX		    128
#define I8XX_P1_MIN		      2
#define I8XX_P1_MAX		     33
#define I8XX_P1_LVDS_MIN	      1
#define I8XX_P1_LVDS_MAX	      6
#define I8XX_P2_SLOW		      4
#define I8XX_P2_FAST		      2
#define I8XX_P2_LVDS_SLOW	      14
#define I8XX_P2_LVDS_FAST	      14	/* No fast option */
#define I8XX_P2_SLOW_LIMIT	 165000

#define I9XX_DOT_MIN		  20000
#define I9XX_DOT_MAX		 400000
#define I9XX_VCO_MIN		1400000
#define I9XX_VCO_MAX		2800000
#define I9XX_N_MIN		      3
#define I9XX_N_MAX		      8
#define I9XX_M_MIN		     70
#define I9XX_M_MAX		    120
#define I9XX_M1_MIN		     10
#define I9XX_M1_MAX		     20
#define I9XX_M2_MIN		      5
#define I9XX_M2_MAX		      9
#define I9XX_P_SDVO_DAC_MIN	      5
#define I9XX_P_SDVO_DAC_MAX	     80
#define I9XX_P_LVDS_MIN		      7
#define I9XX_P_LVDS_MAX		     98
#define I9XX_P1_MIN		      1
#define I9XX_P1_MAX		      8
#define I9XX_P2_SDVO_DAC_SLOW		     10
#define I9XX_P2_SDVO_DAC_FAST		      5
#define I9XX_P2_SDVO_DAC_SLOW_LIMIT	 200000
#define I9XX_P2_LVDS_SLOW		     14
#define I9XX_P2_LVDS_FAST		      7
#define I9XX_P2_LVDS_SLOW_LIMIT		 112000

#define INTEL_LIMIT_I8XX_DVO_DAC    0
#define INTEL_LIMIT_I8XX_LVDS	    1
#define INTEL_LIMIT_I9XX_SDVO_DAC   2
#define INTEL_LIMIT_I9XX_LVDS	    3

static const struct psb_intel_limit_t psb_intel_limits[] = {
	{			/* INTEL_LIMIT_I8XX_DVO_DAC */
	 .dot = {.min = I8XX_DOT_MIN, .max = I8XX_DOT_MAX},
	 .vco = {.min = I8XX_VCO_MIN, .max = I8XX_VCO_MAX},
	 .n = {.min = I8XX_N_MIN, .max = I8XX_N_MAX},
	 .m = {.min = I8XX_M_MIN, .max = I8XX_M_MAX},
	 .m1 = {.min = I8XX_M1_MIN, .max = I8XX_M1_MAX},
	 .m2 = {.min = I8XX_M2_MIN, .max = I8XX_M2_MAX},
	 .p = {.min = I8XX_P_MIN, .max = I8XX_P_MAX},
	 .p1 = {.min = I8XX_P1_MIN, .max = I8XX_P1_MAX},
	 .p2 = {.dot_limit = I8XX_P2_SLOW_LIMIT,
		.p2_slow = I8XX_P2_SLOW, .p2_fast = I8XX_P2_FAST},
	 },
	{			/* INTEL_LIMIT_I8XX_LVDS */
	 .dot = {.min = I8XX_DOT_MIN, .max = I8XX_DOT_MAX},
	 .vco = {.min = I8XX_VCO_MIN, .max = I8XX_VCO_MAX},
	 .n = {.min = I8XX_N_MIN, .max = I8XX_N_MAX},
	 .m = {.min = I8XX_M_MIN, .max = I8XX_M_MAX},
	 .m1 = {.min = I8XX_M1_MIN, .max = I8XX_M1_MAX},
	 .m2 = {.min = I8XX_M2_MIN, .max = I8XX_M2_MAX},
	 .p = {.min = I8XX_P_MIN, .max = I8XX_P_MAX},
	 .p1 = {.min = I8XX_P1_LVDS_MIN, .max = I8XX_P1_LVDS_MAX},
	 .p2 = {.dot_limit = I8XX_P2_SLOW_LIMIT,
		.p2_slow = I8XX_P2_LVDS_SLOW, .p2_fast = I8XX_P2_LVDS_FAST},
	 },
	{			/* INTEL_LIMIT_I9XX_SDVO_DAC */
	 .dot = {.min = I9XX_DOT_MIN, .max = I9XX_DOT_MAX},
	 .vco = {.min = I9XX_VCO_MIN, .max = I9XX_VCO_MAX},
	 .n = {.min = I9XX_N_MIN, .max = I9XX_N_MAX},
	 .m = {.min = I9XX_M_MIN, .max = I9XX_M_MAX},
	 .m1 = {.min = I9XX_M1_MIN, .max = I9XX_M1_MAX},
	 .m2 = {.min = I9XX_M2_MIN, .max = I9XX_M2_MAX},
	 .p = {.min = I9XX_P_SDVO_DAC_MIN, .max = I9XX_P_SDVO_DAC_MAX},
	 .p1 = {.min = I9XX_P1_MIN, .max = I9XX_P1_MAX},
	 .p2 = {.dot_limit = I9XX_P2_SDVO_DAC_SLOW_LIMIT,
		.p2_slow = I9XX_P2_SDVO_DAC_SLOW, .p2_fast =
		I9XX_P2_SDVO_DAC_FAST},
	 },
	{			/* INTEL_LIMIT_I9XX_LVDS */
	 .dot = {.min = I9XX_DOT_MIN, .max = I9XX_DOT_MAX},
	 .vco = {.min = I9XX_VCO_MIN, .max = I9XX_VCO_MAX},
	 .n = {.min = I9XX_N_MIN, .max = I9XX_N_MAX},
	 .m = {.min = I9XX_M_MIN, .max = I9XX_M_MAX},
	 .m1 = {.min = I9XX_M1_MIN, .max = I9XX_M1_MAX},
	 .m2 = {.min = I9XX_M2_MIN, .max = I9XX_M2_MAX},
	 .p = {.min = I9XX_P_LVDS_MIN, .max = I9XX_P_LVDS_MAX},
	 .p1 = {.min = I9XX_P1_MIN, .max = I9XX_P1_MAX},
	 /* The single-channel range is 25-112Mhz, and dual-channel
	  * is 80-224Mhz.  Prefer single channel as much as possible.
	  */
	 .p2 = {.dot_limit = I9XX_P2_LVDS_SLOW_LIMIT,
		.p2_slow = I9XX_P2_LVDS_SLOW, .p2_fast = I9XX_P2_LVDS_FAST},
	 },
};

static const struct psb_intel_limit_t *psb_intel_limit(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	const struct psb_intel_limit_t *limit;

	if (IS_I9XX(dev)) {
		if (psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS))
			limit = &psb_intel_limits[INTEL_LIMIT_I9XX_LVDS];
		else
			limit = &psb_intel_limits[INTEL_LIMIT_I9XX_SDVO_DAC];
	} else {
		if (psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS))
			limit = &psb_intel_limits[INTEL_LIMIT_I8XX_LVDS];
		else
			limit = &psb_intel_limits[INTEL_LIMIT_I8XX_DVO_DAC];
	}
	return limit;
}

/** Derive the pixel clock for the given refclk and divisors for 8xx chips. */

static void i8xx_clock(int refclk, struct psb_intel_clock_t *clock)
{
	clock->m = 5 * (clock->m1 + 2) + (clock->m2 + 2);
	clock->p = clock->p1 * clock->p2;
	clock->vco = refclk * clock->m / (clock->n + 2);
	clock->dot = clock->vco / clock->p;
}

/** Derive the pixel clock for the given refclk and divisors for 9xx chips. */

static void i9xx_clock(int refclk, struct psb_intel_clock_t *clock)
{
	clock->m = 5 * (clock->m1 + 2) + (clock->m2 + 2);
	clock->p = clock->p1 * clock->p2;
	clock->vco = refclk * clock->m / (clock->n + 2);
	clock->dot = clock->vco / clock->p;
}

static void psb_intel_clock(struct drm_device *dev, int refclk,
			struct psb_intel_clock_t *clock)
{
	if (IS_I9XX(dev))
		return i9xx_clock(refclk, clock);
	else
		return i8xx_clock(refclk, clock);
}

/**
 * Returns whether any output on the specified pipe is of the specified type
 */
bool psb_intel_pipe_has_type(struct drm_crtc *crtc, int type)
{
	struct drm_device *dev = crtc->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_connector *l_entry;

	list_for_each_entry(l_entry, &mode_config->connector_list, head) {
		if (l_entry->encoder && l_entry->encoder->crtc == crtc) {
			struct psb_intel_output *psb_intel_output =
			    to_psb_intel_output(l_entry);
			if (psb_intel_output->type == type)
				return true;
		}
	}
	return false;
}

#define INTELPllInvalid(s)   { /* ErrorF (s) */; return false; }
/**
 * Returns whether the given set of divisors are valid for a given refclk with
 * the given connectors.
 */

static bool psb_intel_PLL_is_valid(struct drm_crtc *crtc,
			       struct psb_intel_clock_t *clock)
{
	const struct psb_intel_limit_t *limit = psb_intel_limit(crtc);

	if (clock->p1 < limit->p1.min || limit->p1.max < clock->p1)
		INTELPllInvalid("p1 out of range\n");
	if (clock->p < limit->p.min || limit->p.max < clock->p)
		INTELPllInvalid("p out of range\n");
	if (clock->m2 < limit->m2.min || limit->m2.max < clock->m2)
		INTELPllInvalid("m2 out of range\n");
	if (clock->m1 < limit->m1.min || limit->m1.max < clock->m1)
		INTELPllInvalid("m1 out of range\n");
	if (clock->m1 <= clock->m2)
		INTELPllInvalid("m1 <= m2\n");
	if (clock->m < limit->m.min || limit->m.max < clock->m)
		INTELPllInvalid("m out of range\n");
	if (clock->n < limit->n.min || limit->n.max < clock->n)
		INTELPllInvalid("n out of range\n");
	if (clock->vco < limit->vco.min || limit->vco.max < clock->vco)
		INTELPllInvalid("vco out of range\n");
	/* XXX: We may need to be checking "Dot clock"
	 * depending on the multiplier, connector, etc.,
	 * rather than just a single range.
	 */
	if (clock->dot < limit->dot.min || limit->dot.max < clock->dot)
		INTELPllInvalid("dot out of range\n");

	return true;
}

/**
 * Returns a set of divisors for the desired target clock with the given
 * refclk, or FALSE.  The returned values represent the clock equation:
 * reflck * (5 * (m1 + 2) + (m2 + 2)) / (n + 2) / p1 / p2.
 */
static bool psb_intel_find_best_PLL(struct drm_crtc *crtc, int target,
				int refclk,
				struct psb_intel_clock_t *best_clock)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_clock_t clock;
	const struct psb_intel_limit_t *limit = psb_intel_limit(crtc);
	int err = target;

	if (IS_I9XX(dev) && psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS) &&
	    (REG_READ(LVDS) & LVDS_PORT_EN) != 0) {
		/*
		 * For LVDS, if the panel is on, just rely on its current
		 * settings for dual-channel.  We haven't figured out how to
		 * reliably set up different single/dual channel state, if we
		 * even can.
		 */
		if ((REG_READ(LVDS) & LVDS_CLKB_POWER_MASK) ==
		    LVDS_CLKB_POWER_UP)
			clock.p2 = limit->p2.p2_fast;
		else
			clock.p2 = limit->p2.p2_slow;
	} else {
		if (target < limit->p2.dot_limit)
			clock.p2 = limit->p2.p2_slow;
		else
			clock.p2 = limit->p2.p2_fast;
	}

	memset(best_clock, 0, sizeof(*best_clock));

	for (clock.m1 = limit->m1.min; clock.m1 <= limit->m1.max;
	     clock.m1++) {
		for (clock.m2 = limit->m2.min;
		     clock.m2 < clock.m1 && clock.m2 <= limit->m2.max;
		     clock.m2++) {
			for (clock.n = limit->n.min;
			     clock.n <= limit->n.max; clock.n++) {
				for (clock.p1 = limit->p1.min;
				     clock.p1 <= limit->p1.max;
				     clock.p1++) {
					int this_err;

					psb_intel_clock(dev, refclk, &clock);

					if (!psb_intel_PLL_is_valid
					    (crtc, &clock))
						continue;

					this_err = abs(clock.dot - target);
					if (this_err < err) {
						*best_clock = clock;
						err = this_err;
					}
				}
			}
		}
	}

	return err != target;
}

void psb_intel_wait_for_vblank(struct drm_device *dev)
{
	/* Wait for 20ms, i.e. one cycle at 50hz. */
	udelay(20000);
}

int psb_intel_pipe_set_base(struct drm_crtc *crtc,
			    int x, int y, struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	/* struct drm_i915_master_private *master_priv; */
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct psb_framebuffer *psbfb = to_psb_fb(crtc->fb);
	struct psb_intel_mode_device *mode_dev = psb_intel_crtc->mode_dev;
	int pipe = psb_intel_crtc->pipe;
	unsigned long Start, Offset;
	int dspbase = (pipe == 0 ? DSPABASE : DSPBBASE);
	int dspsurf = (pipe == 0 ? DSPASURF : DSPBSURF);
	int dspstride = (pipe == 0) ? DSPASTRIDE : DSPBSTRIDE;
	int dspcntr_reg = (pipe == 0) ? DSPACNTR : DSPBCNTR;
	u32 dspcntr;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	/* no fb bound */
	if (!crtc->fb) {
		DRM_DEBUG("No FB bound\n");
		return 0;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return 0;

	if (IS_MRST(dev) && (pipe == 0))
		dspbase = DSPALINOFF;

	Start = mode_dev->bo_offset(dev, psbfb);
	Offset = y * crtc->fb->pitch + x * (crtc->fb->bits_per_pixel / 8);

	REG_WRITE(dspstride, crtc->fb->pitch);

	dspcntr = REG_READ(dspcntr_reg);
	dspcntr &= ~DISPPLANE_PIXFORMAT_MASK;

	switch (crtc->fb->bits_per_pixel) {
	case 8:
		dspcntr |= DISPPLANE_8BPP;
		break;
	case 16:
		if (crtc->fb->depth == 15)
			dspcntr |= DISPPLANE_15_16BPP;
		else
			dspcntr |= DISPPLANE_16BPP;
		break;
	case 24:
	case 32:
		dspcntr |= DISPPLANE_32BPP_NO_ALPHA;
		break;
	default:
		DRM_ERROR("Unknown color depth\n");
		ret = -EINVAL;
		goto psb_intel_pipe_set_base_exit;
	}
	REG_WRITE(dspcntr_reg, dspcntr);

	DRM_DEBUG("Writing base %08lX %08lX %d %d\n", Start, Offset, x, y);
	if (IS_I965G(dev) || IS_MRST(dev)) {
		REG_WRITE(dspbase, Offset);
		REG_READ(dspbase);
		REG_WRITE(dspsurf, Start);
		REG_READ(dspsurf);
	} else {
		REG_WRITE(dspbase, Start + Offset);
		REG_READ(dspbase);
	}

psb_intel_pipe_set_base_exit:

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return ret;
}

/**
 * Sets the power management mode of the pipe and plane.
 *
 * This code should probably grow support for turning the cursor off and back
 * on appropriately at the same time as we're turning the pipe off/on.
 */
static void psb_intel_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct drm_device *dev = crtc->dev;
	/* struct drm_i915_master_private *master_priv; */
	/* struct drm_i915_private *dev_priv = dev->dev_private; */
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	int dpll_reg = (pipe == 0) ? DPLL_A : DPLL_B;
	int dspcntr_reg = (pipe == 0) ? DSPACNTR : DSPBCNTR;
	int dspbase_reg = (pipe == 0) ? DSPABASE : DSPBBASE;
	int pipeconf_reg = (pipe == 0) ? PIPEACONF : PIPEBCONF;
	u32 temp;
	bool enabled;

	/* XXX: When our outputs are all unaware of DPMS modes other than off
	 * and on, we should map those modes to DRM_MODE_DPMS_OFF in the CRTC.
	 */
	switch (mode) {
	case DRM_MODE_DPMS_ON:
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
		/* Enable the DPLL */
		temp = REG_READ(dpll_reg);
		if ((temp & DPLL_VCO_ENABLE) == 0) {
			REG_WRITE(dpll_reg, temp);
			REG_READ(dpll_reg);
			/* Wait for the clocks to stabilize. */
			udelay(150);
			REG_WRITE(dpll_reg, temp | DPLL_VCO_ENABLE);
			REG_READ(dpll_reg);
			/* Wait for the clocks to stabilize. */
			udelay(150);
			REG_WRITE(dpll_reg, temp | DPLL_VCO_ENABLE);
			REG_READ(dpll_reg);
			/* Wait for the clocks to stabilize. */
			udelay(150);
		}

		/* Enable the pipe */
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEACONF_ENABLE) == 0)
			REG_WRITE(pipeconf_reg, temp | PIPEACONF_ENABLE);

		/* Enable the plane */
		temp = REG_READ(dspcntr_reg);
		if ((temp & DISPLAY_PLANE_ENABLE) == 0) {
			REG_WRITE(dspcntr_reg,
				  temp | DISPLAY_PLANE_ENABLE);
			/* Flush the plane changes */
			REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
		}

		psb_intel_crtc_load_lut(crtc);

		/* Give the overlay scaler a chance to enable
		 * if it's on this pipe */
		/* psb_intel_crtc_dpms_video(crtc, true); TODO */
		break;
	case DRM_MODE_DPMS_OFF:
		/* Give the overlay scaler a chance to disable
		 * if it's on this pipe */
		/* psb_intel_crtc_dpms_video(crtc, FALSE); TODO */

		/* Disable the VGA plane that we never use */
		REG_WRITE(VGACNTRL, VGA_DISP_DISABLE);

		/* Disable display plane */
		temp = REG_READ(dspcntr_reg);
		if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
			REG_WRITE(dspcntr_reg,
				  temp & ~DISPLAY_PLANE_ENABLE);
			/* Flush the plane changes */
			REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
			REG_READ(dspbase_reg);
		}

		if (!IS_I9XX(dev)) {
			/* Wait for vblank for the disable to take effect */
			psb_intel_wait_for_vblank(dev);
		}

		/* Next, disable display pipes */
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEACONF_ENABLE) != 0) {
			REG_WRITE(pipeconf_reg, temp & ~PIPEACONF_ENABLE);
			REG_READ(pipeconf_reg);
		}

		/* Wait for vblank for the disable to take effect. */
		psb_intel_wait_for_vblank(dev);

		temp = REG_READ(dpll_reg);
		if ((temp & DPLL_VCO_ENABLE) != 0) {
			REG_WRITE(dpll_reg, temp & ~DPLL_VCO_ENABLE);
			REG_READ(dpll_reg);
		}

		/* Wait for the clocks to turn off. */
		udelay(150);
		break;
	}

	enabled = crtc->enabled && mode != DRM_MODE_DPMS_OFF;

#if 0				/* JB: Add vblank support later */
	if (enabled)
		dev_priv->vblank_pipe |= (1 << pipe);
	else
		dev_priv->vblank_pipe &= ~(1 << pipe);
#endif

#if 0				/* JB: Add sarea support later */
	if (!dev->primary->master)
		return 0;

	master_priv = dev->primary->master->driver_priv;
	if (!master_priv->sarea_priv)
		return 0;

	switch (pipe) {
	case 0:
		master_priv->sarea_priv->planeA_w =
		    enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->planeA_h =
		    enabled ? crtc->mode.vdisplay : 0;
		break;
	case 1:
		master_priv->sarea_priv->planeB_w =
		    enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->planeB_h =
		    enabled ? crtc->mode.vdisplay : 0;
		break;
	default:
		DRM_ERROR("Can't update pipe %d in SAREA\n", pipe);
		break;
	}
#endif

	/*Set FIFO Watermarks*/
	REG_WRITE(DSPARB, 0x3F3E);
}

static void psb_intel_crtc_prepare(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void psb_intel_crtc_commit(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_ON);
}

void psb_intel_encoder_prepare(struct drm_encoder *encoder)
{
	struct drm_encoder_helper_funcs *encoder_funcs =
	    encoder->helper_private;
	/* lvds has its own version of prepare see psb_intel_lvds_prepare */
	encoder_funcs->dpms(encoder, DRM_MODE_DPMS_OFF);
}

void psb_intel_encoder_commit(struct drm_encoder *encoder)
{
	struct drm_encoder_helper_funcs *encoder_funcs =
	    encoder->helper_private;
	/* lvds has its own version of commit see psb_intel_lvds_commit */
	encoder_funcs->dpms(encoder, DRM_MODE_DPMS_ON);
}

static bool psb_intel_crtc_mode_fixup(struct drm_crtc *crtc,
				  struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}


/**
 * Return the pipe currently connected to the panel fitter,
 * or -1 if the panel fitter is not present or not in use
 */
static int psb_intel_panel_fitter_pipe(struct drm_device *dev)
{
	u32 pfit_control;

	/* i830 doesn't have a panel fitter */
	if (IS_I830(dev))
		return -1;

	pfit_control = REG_READ(PFIT_CONTROL);

	/* See if the panel fitter is in use */
	if ((pfit_control & PFIT_ENABLE) == 0)
		return -1;

	/* 965 can place panel fitter on either pipe */
	if (IS_I965G(dev) || IS_MID(dev))
		return (pfit_control >> 29) & 0x3;

	/* older chips can only use pipe 1 */
	return 1;
}

static int psb_intel_crtc_mode_set(struct drm_crtc *crtc,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode,
			       int x, int y,
			       struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	int fp_reg = (pipe == 0) ? FPA0 : FPB0;
	int dpll_reg = (pipe == 0) ? DPLL_A : DPLL_B;
	int dpll_md_reg = (psb_intel_crtc->pipe == 0) ? DPLL_A_MD : DPLL_B_MD;
	int dspcntr_reg = (pipe == 0) ? DSPACNTR : DSPBCNTR;
	int pipeconf_reg = (pipe == 0) ? PIPEACONF : PIPEBCONF;
	int htot_reg = (pipe == 0) ? HTOTAL_A : HTOTAL_B;
	int hblank_reg = (pipe == 0) ? HBLANK_A : HBLANK_B;
	int hsync_reg = (pipe == 0) ? HSYNC_A : HSYNC_B;
	int vtot_reg = (pipe == 0) ? VTOTAL_A : VTOTAL_B;
	int vblank_reg = (pipe == 0) ? VBLANK_A : VBLANK_B;
	int vsync_reg = (pipe == 0) ? VSYNC_A : VSYNC_B;
	int dspsize_reg = (pipe == 0) ? DSPASIZE : DSPBSIZE;
	int dsppos_reg = (pipe == 0) ? DSPAPOS : DSPBPOS;
	int pipesrc_reg = (pipe == 0) ? PIPEASRC : PIPEBSRC;
	int refclk;
	struct psb_intel_clock_t clock;
	u32 dpll = 0, fp = 0, dspcntr, pipeconf;
	bool ok, is_sdvo = false, is_dvo = false;
	bool is_crt = false, is_lvds = false, is_tv = false;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_connector *connector;

	list_for_each_entry(connector, &mode_config->connector_list, head) {
		struct psb_intel_output *psb_intel_output =
		    to_psb_intel_output(connector);

		if (!connector->encoder
		    || connector->encoder->crtc != crtc)
			continue;

		switch (psb_intel_output->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		case INTEL_OUTPUT_SDVO:
			is_sdvo = true;
			break;
		case INTEL_OUTPUT_DVO:
			is_dvo = true;
			break;
		case INTEL_OUTPUT_TVOUT:
			is_tv = true;
			break;
		case INTEL_OUTPUT_ANALOG:
			is_crt = true;
			break;
		}
	}

	if (IS_I9XX(dev))
		refclk = 96000;
	else
		refclk = 48000;

	ok = psb_intel_find_best_PLL(crtc, adjusted_mode->clock, refclk,
				 &clock);
	if (!ok) {
		DRM_ERROR("Couldn't find PLL settings for mode!\n");
		return 0;
	}

	fp = clock.n << 16 | clock.m1 << 8 | clock.m2;

	dpll = DPLL_VGA_MODE_DIS;
	if (IS_I9XX(dev)) {
		if (is_lvds) {
			dpll |= DPLLB_MODE_LVDS;
			if (IS_POULSBO(dev))
				dpll |= DPLL_DVO_HIGH_SPEED;
		} else
			dpll |= DPLLB_MODE_DAC_SERIAL;
		if (is_sdvo) {
			dpll |= DPLL_DVO_HIGH_SPEED;
			if (IS_I945G(dev) ||
			    IS_I945GM(dev) ||
			    IS_POULSBO(dev)) {
				int sdvo_pixel_multiply =
				    adjusted_mode->clock / mode->clock;
				dpll |=
				    (sdvo_pixel_multiply -
				     1) << SDVO_MULTIPLIER_SHIFT_HIRES;
			}
		}

		/* compute bitmask from p1 value */
		dpll |= (1 << (clock.p1 - 1)) << 16;
		switch (clock.p2) {
		case 5:
			dpll |= DPLL_DAC_SERIAL_P2_CLOCK_DIV_5;
			break;
		case 7:
			dpll |= DPLLB_LVDS_P2_CLOCK_DIV_7;
			break;
		case 10:
			dpll |= DPLL_DAC_SERIAL_P2_CLOCK_DIV_10;
			break;
		case 14:
			dpll |= DPLLB_LVDS_P2_CLOCK_DIV_14;
			break;
		}
		if (IS_I965G(dev))
			dpll |= (6 << PLL_LOAD_PULSE_PHASE_SHIFT);
	} else {
		if (is_lvds) {
			dpll |=
			    (1 << (clock.p1 - 1)) <<
			    DPLL_FPA01_P1_POST_DIV_SHIFT;
		} else {
			if (clock.p1 == 2)
				dpll |= PLL_P1_DIVIDE_BY_TWO;
			else
				dpll |=
				    (clock.p1 -
				     2) << DPLL_FPA01_P1_POST_DIV_SHIFT;
			if (clock.p2 == 4)
				dpll |= PLL_P2_DIVIDE_BY_4;
		}
	}

	if (is_tv) {
		dpll |= 3;
	}
	else
		dpll |= PLL_REF_INPUT_DREFCLK;

	/* setup pipeconf */
	pipeconf = REG_READ(pipeconf_reg);

	/* Set up the display plane register */
	dspcntr = DISPPLANE_GAMMA_ENABLE;

	if (pipe == 0)
		dspcntr |= DISPPLANE_SEL_PIPE_A;
	else
		dspcntr |= DISPPLANE_SEL_PIPE_B;

	dspcntr |= DISPLAY_PLANE_ENABLE;
	pipeconf |= PIPEACONF_ENABLE;
	dpll |= DPLL_VCO_ENABLE;


	/* Disable the panel fitter if it was on our pipe */
	if (psb_intel_panel_fitter_pipe(dev) == pipe)
		REG_WRITE(PFIT_CONTROL, 0);

	DRM_DEBUG("Mode for pipe %c:\n", pipe == 0 ? 'A' : 'B');
	drm_mode_debug_printmodeline(mode);

	if (dpll & DPLL_VCO_ENABLE) {
		REG_WRITE(fp_reg, fp);
		REG_WRITE(dpll_reg, dpll & ~DPLL_VCO_ENABLE);
		REG_READ(dpll_reg);
		udelay(150);
	}

	/* The LVDS pin pair needs to be on before the DPLLs are enabled.
	 * This is an exception to the general rule that mode_set doesn't turn
	 * things on.
	 */
	if (is_lvds) {
		u32 lvds = REG_READ(LVDS);

		lvds |=
		    LVDS_PORT_EN | LVDS_A0A2_CLKA_POWER_UP |
		    LVDS_PIPEB_SELECT;
		/* Set the B0-B3 data pairs corresponding to
		 * whether we're going to
		 * set the DPLLs for dual-channel mode or not.
		 */
		if (clock.p2 == 7)
			lvds |= LVDS_B0B3_POWER_UP | LVDS_CLKB_POWER_UP;
		else
			lvds &= ~(LVDS_B0B3_POWER_UP | LVDS_CLKB_POWER_UP);

		/* It would be nice to set 24 vs 18-bit mode (LVDS_A3_POWER_UP)
		 * appropriately here, but we need to look more
		 * thoroughly into how panels behave in the two modes.
		 */

		REG_WRITE(LVDS, lvds);
		REG_READ(LVDS);
	}

	REG_WRITE(fp_reg, fp);
	REG_WRITE(dpll_reg, dpll);
	REG_READ(dpll_reg);
	/* Wait for the clocks to stabilize. */
	udelay(150);

	if (IS_I965G(dev)) {
		int sdvo_pixel_multiply =
		    adjusted_mode->clock / mode->clock;
		REG_WRITE(dpll_md_reg,
			  (0 << DPLL_MD_UDI_DIVIDER_SHIFT) |
			  ((sdvo_pixel_multiply -
			    1) << DPLL_MD_UDI_MULTIPLIER_SHIFT));
	} else {
		/* write it again -- the BIOS does, after all */
		REG_WRITE(dpll_reg, dpll);
	}
	REG_READ(dpll_reg);
	/* Wait for the clocks to stabilize. */
	udelay(150);

	REG_WRITE(htot_reg, (adjusted_mode->crtc_hdisplay - 1) |
		  ((adjusted_mode->crtc_htotal - 1) << 16));
	REG_WRITE(hblank_reg, (adjusted_mode->crtc_hblank_start - 1) |
		  ((adjusted_mode->crtc_hblank_end - 1) << 16));
	REG_WRITE(hsync_reg, (adjusted_mode->crtc_hsync_start - 1) |
		  ((adjusted_mode->crtc_hsync_end - 1) << 16));
	REG_WRITE(vtot_reg, (adjusted_mode->crtc_vdisplay - 1) |
		  ((adjusted_mode->crtc_vtotal - 1) << 16));
	REG_WRITE(vblank_reg, (adjusted_mode->crtc_vblank_start - 1) |
		  ((adjusted_mode->crtc_vblank_end - 1) << 16));
	REG_WRITE(vsync_reg, (adjusted_mode->crtc_vsync_start - 1) |
		  ((adjusted_mode->crtc_vsync_end - 1) << 16));
	/* pipesrc and dspsize control the size that is scaled from,
	 * which should always be the user's requested size.
	 */
	REG_WRITE(dspsize_reg,
		  ((mode->vdisplay - 1) << 16) | (mode->hdisplay - 1));
	REG_WRITE(dsppos_reg, 0);
	REG_WRITE(pipesrc_reg,
		  ((mode->hdisplay - 1) << 16) | (mode->vdisplay - 1));
	REG_WRITE(pipeconf_reg, pipeconf);
	REG_READ(pipeconf_reg);

	psb_intel_wait_for_vblank(dev);

	REG_WRITE(dspcntr_reg, dspcntr);

	/* Flush the plane changes */
	{
		struct drm_crtc_helper_funcs *crtc_funcs =
		    crtc->helper_private;
		crtc_funcs->mode_set_base(crtc, x, y, old_fb);
	}

	psb_intel_wait_for_vblank(dev);

	return 0;
}

/** Loads the palette/gamma unit for the CRTC with the prepared values */
void psb_intel_crtc_load_lut(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_psb_private *dev_priv =
				(struct drm_psb_private *)dev->dev_private;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int palreg = PALETTE_A;
	int i;

	/* The clocks have to be on to load the palette. */
	if (!crtc->enabled)
		return;

	switch (psb_intel_crtc->pipe) {
	case 0:
		break;
	case 1:
		palreg = PALETTE_B;
		break;
	case 2:
		palreg = PALETTE_C;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number. \n");
		return;
	}
	
	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_ONLY_IF_ON)) {
		for (i = 0; i < 256; i++) {
			REG_WRITE(palreg + 4 * i,
				  ((psb_intel_crtc->lut_r[i] +
				  psb_intel_crtc->lut_adj[i]) << 16) |
				  ((psb_intel_crtc->lut_g[i] +
				  psb_intel_crtc->lut_adj[i]) << 8) |
				  (psb_intel_crtc->lut_b[i] +
				  psb_intel_crtc->lut_adj[i]));
		}
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		for (i = 0; i < 256; i++) {
			dev_priv->save_palette_a[i] =
				  ((psb_intel_crtc->lut_r[i] +
				  psb_intel_crtc->lut_adj[i]) << 16) |
				  ((psb_intel_crtc->lut_g[i] +
				  psb_intel_crtc->lut_adj[i]) << 8) |
				  (psb_intel_crtc->lut_b[i] +
				  psb_intel_crtc->lut_adj[i]);
		}

	}
}

#ifndef CONFIG_X86_MRST
/**
 * Save HW states of giving crtc
 */
static void psb_intel_crtc_save(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	/* struct drm_psb_private *dev_priv =
			(struct drm_psb_private *)dev->dev_private; */
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct psb_intel_crtc_state *crtc_state = psb_intel_crtc->crtc_state;
	int pipeA = (psb_intel_crtc->pipe == 0);
	uint32_t paletteReg;
	int i;

	DRM_DEBUG("\n");

	if (!crtc_state) {
		DRM_DEBUG("No CRTC state found\n");
		return;
	}

	crtc_state->saveDSPCNTR = REG_READ(pipeA ? DSPACNTR : DSPBCNTR);
	crtc_state->savePIPECONF = REG_READ(pipeA ? PIPEACONF : PIPEBCONF);
	crtc_state->savePIPESRC = REG_READ(pipeA ? PIPEASRC : PIPEBSRC);
	crtc_state->saveFP0 = REG_READ(pipeA ? FPA0 : FPB0);
	crtc_state->saveFP1 = REG_READ(pipeA ? FPA1 : FPB1);
	crtc_state->saveDPLL = REG_READ(pipeA ? DPLL_A : DPLL_B);
	crtc_state->saveHTOTAL = REG_READ(pipeA ? HTOTAL_A : HTOTAL_B);
	crtc_state->saveHBLANK = REG_READ(pipeA ? HBLANK_A : HBLANK_B);
	crtc_state->saveHSYNC = REG_READ(pipeA ? HSYNC_A : HSYNC_B);
	crtc_state->saveVTOTAL = REG_READ(pipeA ? VTOTAL_A : VTOTAL_B);
	crtc_state->saveVBLANK = REG_READ(pipeA ? VBLANK_A : VBLANK_B);
	crtc_state->saveVSYNC = REG_READ(pipeA ? VSYNC_A : VSYNC_B);
	crtc_state->saveDSPSTRIDE = REG_READ(pipeA ? DSPASTRIDE : DSPBSTRIDE);

	/*NOTE: DSPSIZE DSPPOS only for psb*/
	crtc_state->saveDSPSIZE = REG_READ(pipeA ? DSPASIZE : DSPBSIZE);
	crtc_state->saveDSPPOS = REG_READ(pipeA ? DSPAPOS : DSPBPOS);

	crtc_state->saveDSPBASE = REG_READ(pipeA ? DSPABASE : DSPBBASE);

	DRM_DEBUG("(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)\n",
			crtc_state->saveDSPCNTR,
			crtc_state->savePIPECONF,
			crtc_state->savePIPESRC,
			crtc_state->saveFP0,
			crtc_state->saveFP1,
			crtc_state->saveDPLL,
			crtc_state->saveHTOTAL,
			crtc_state->saveHBLANK,
			crtc_state->saveHSYNC,
			crtc_state->saveVTOTAL,
			crtc_state->saveVBLANK,
			crtc_state->saveVSYNC,
			crtc_state->saveDSPSTRIDE,
			crtc_state->saveDSPSIZE,
			crtc_state->saveDSPPOS,
			crtc_state->saveDSPBASE
		);

	paletteReg = pipeA ? PALETTE_A : PALETTE_B;
	for (i = 0; i < 256; ++i)
		crtc_state->savePalette[i] = REG_READ(paletteReg + (i << 2));
}

/**
 * Restore HW states of giving crtc
 */
static void psb_intel_crtc_restore(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	/* struct drm_psb_private * dev_priv =
				(struct drm_psb_private *)dev->dev_private; */
	struct psb_intel_crtc *psb_intel_crtc =  to_psb_intel_crtc(crtc);
	struct psb_intel_crtc_state *crtc_state = psb_intel_crtc->crtc_state;
	/* struct drm_crtc_helper_funcs * crtc_funcs = crtc->helper_private; */
	int pipeA = (psb_intel_crtc->pipe == 0);
	uint32_t paletteReg;
	int i;

	DRM_DEBUG("\n");

	if (!crtc_state) {
		DRM_DEBUG("No crtc state\n");
		return;
	}

	DRM_DEBUG(
		"current:(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)\n",
		REG_READ(pipeA ? DSPACNTR : DSPBCNTR),
		REG_READ(pipeA ? PIPEACONF : PIPEBCONF),
		REG_READ(pipeA ? PIPEASRC : PIPEBSRC),
		REG_READ(pipeA ? FPA0 : FPB0),
		REG_READ(pipeA ? FPA1 : FPB1),
		REG_READ(pipeA ? DPLL_A : DPLL_B),
		REG_READ(pipeA ? HTOTAL_A : HTOTAL_B),
		REG_READ(pipeA ? HBLANK_A : HBLANK_B),
		REG_READ(pipeA ? HSYNC_A : HSYNC_B),
		REG_READ(pipeA ? VTOTAL_A : VTOTAL_B),
		REG_READ(pipeA ? VBLANK_A : VBLANK_B),
		REG_READ(pipeA ? VSYNC_A : VSYNC_B),
		REG_READ(pipeA ? DSPASTRIDE : DSPBSTRIDE),
		REG_READ(pipeA ? DSPASIZE : DSPBSIZE),
		REG_READ(pipeA ? DSPAPOS : DSPBPOS),
		REG_READ(pipeA ? DSPABASE : DSPBBASE)
		);

	DRM_DEBUG(
		"saved: (%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)\n",
		crtc_state->saveDSPCNTR,
		crtc_state->savePIPECONF,
		crtc_state->savePIPESRC,
		crtc_state->saveFP0,
		crtc_state->saveFP1,
		crtc_state->saveDPLL,
		crtc_state->saveHTOTAL,
		crtc_state->saveHBLANK,
		crtc_state->saveHSYNC,
		crtc_state->saveVTOTAL,
		crtc_state->saveVBLANK,
		crtc_state->saveVSYNC,
		crtc_state->saveDSPSTRIDE,
		crtc_state->saveDSPSIZE,
		crtc_state->saveDSPPOS,
		crtc_state->saveDSPBASE
		);

	if (crtc_state->saveDPLL & DPLL_VCO_ENABLE) {
		REG_WRITE(pipeA ? DPLL_A : DPLL_B,
			crtc_state->saveDPLL & ~DPLL_VCO_ENABLE);
		REG_READ(pipeA ? DPLL_A : DPLL_B);
		DRM_DEBUG("write dpll: %x\n",
				REG_READ(pipeA ? DPLL_A : DPLL_B));
		udelay(150);
	}

	REG_WRITE(pipeA ? FPA0 : FPB0, crtc_state->saveFP0);
	REG_READ(pipeA ? FPA0 : FPB0);

	REG_WRITE(pipeA ? FPA1 : FPB1, crtc_state->saveFP1);
	REG_READ(pipeA ? FPA1 : FPB1);

	REG_WRITE(pipeA ? DPLL_A : DPLL_B, crtc_state->saveDPLL);
	REG_READ(pipeA ? DPLL_A : DPLL_B);
	udelay(150);

	REG_WRITE(pipeA ? HTOTAL_A : HTOTAL_B, crtc_state->saveHTOTAL);
	REG_WRITE(pipeA ? HBLANK_A : HBLANK_B, crtc_state->saveHBLANK);
	REG_WRITE(pipeA ? HSYNC_A : HSYNC_B, crtc_state->saveHSYNC);
	REG_WRITE(pipeA ? VTOTAL_A : VTOTAL_B, crtc_state->saveVTOTAL);
	REG_WRITE(pipeA ? VBLANK_A : VBLANK_B, crtc_state->saveVBLANK);
	REG_WRITE(pipeA ? VSYNC_A : VSYNC_B, crtc_state->saveVSYNC);
	REG_WRITE(pipeA ? DSPASTRIDE : DSPBSTRIDE, crtc_state->saveDSPSTRIDE);

	REG_WRITE(pipeA ? DSPASIZE : DSPBSIZE, crtc_state->saveDSPSIZE);
	REG_WRITE(pipeA ? DSPAPOS : DSPBPOS, crtc_state->saveDSPPOS);

	REG_WRITE(pipeA ? PIPEASRC : PIPEBSRC, crtc_state->savePIPESRC);
	REG_WRITE(pipeA ? DSPABASE : DSPBBASE, crtc_state->saveDSPBASE);
	REG_WRITE(pipeA ? PIPEACONF : PIPEBCONF, crtc_state->savePIPECONF);

	psb_intel_wait_for_vblank(dev);

	REG_WRITE(pipeA ? DSPACNTR : DSPBCNTR, crtc_state->saveDSPCNTR);
	REG_WRITE(pipeA ? DSPABASE : DSPBBASE, crtc_state->saveDSPBASE);

	psb_intel_wait_for_vblank(dev);

	paletteReg = pipeA ? PALETTE_A : PALETTE_B;
	for (i = 0; i < 256; ++i)
		REG_WRITE(paletteReg + (i << 2), crtc_state->savePalette[i]);
}
#endif

static int psb_intel_crtc_cursor_set(struct drm_crtc *crtc,
				 struct drm_file *file_priv,
				 uint32_t handle,
				 uint32_t width, uint32_t height)
{
	struct drm_device *dev = crtc->dev;
	struct drm_psb_private *dev_priv =
				(struct drm_psb_private *)dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct psb_intel_mode_device *mode_dev = psb_intel_crtc->mode_dev;
	int pipe = psb_intel_crtc->pipe;
	uint32_t control = (pipe == 0) ? CURACNTR : CURBCNTR;
	uint32_t base = (pipe == 0) ? CURABASE : CURBBASE;
	uint32_t temp;
	size_t addr = 0;
	uint32_t page_offset;
	size_t size;
	void *bo;
	int ret;

	DRM_DEBUG("\n");

	/* if we want to turn of the cursor ignore width and height */
	if (!handle) {
		DRM_DEBUG("cursor off\n");
		/* turn off the cursor */
		temp = 0;
		temp |= CURSOR_MODE_DISABLE;

		if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					      OSPM_UHB_ONLY_IF_ON)) {
			REG_WRITE(control, temp);
			REG_WRITE(base, 0);
			ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		}

		/* unpin the old bo */
		if (psb_intel_crtc->cursor_bo) {
			mode_dev->bo_unpin_for_scanout(dev,
						       psb_intel_crtc->
						       cursor_bo);
			psb_intel_crtc->cursor_bo = NULL;
		}

		return 0;
	}

	/* Currently we only support 64x64 cursors */
	if (width != 64 || height != 64) {
		DRM_ERROR("we currently only support 64x64 cursors\n");
		return -EINVAL;
	}

	bo = mode_dev->bo_from_handle(dev, file_priv, handle);
	if (!bo)
		return -ENOENT;

	ret = mode_dev->bo_pin_for_scanout(dev, bo);
	if (ret)
		return ret;
	size = mode_dev->bo_size(dev, bo);
	if (size < width * height * 4) {
		DRM_ERROR("buffer is to small\n");
		return -ENOMEM;
	}

	/*insert this bo into gtt*/
	DRM_DEBUG("%s: map meminfo for hw cursor. handle %x\n",
						__func__, handle);

	ret = psb_gtt_map_meminfo(dev, (IMG_HANDLE)handle, 0, &page_offset);
	if (ret) {
		DRM_ERROR("Can not map meminfo to GTT. handle 0x%x\n", handle);
		return ret;
	}

	addr = page_offset << PAGE_SHIFT;

	if (IS_POULSBO(dev))
		addr += pg->stolen_base;

	psb_intel_crtc->cursor_addr = addr;

	temp = 0;
	/* set the pipe for the cursor */
	temp |= (pipe << 28);
	temp |= CURSOR_MODE_64_ARGB_AX | MCURSOR_GAMMA_ENABLE;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_ONLY_IF_ON)) {
		REG_WRITE(control, temp);
		REG_WRITE(base, addr);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}

	/* unpin the old bo */
	if (psb_intel_crtc->cursor_bo && psb_intel_crtc->cursor_bo != bo) {
		mode_dev->bo_unpin_for_scanout(dev, psb_intel_crtc->cursor_bo);
		psb_intel_crtc->cursor_bo = bo;
	}

	return 0;
}

static int psb_intel_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	uint32_t temp = 0;
	uint32_t adder;


	if (x < 0) {
		temp |= (CURSOR_POS_SIGN << CURSOR_X_SHIFT);
		x = -x;
	}
	if (y < 0) {
		temp |= (CURSOR_POS_SIGN << CURSOR_Y_SHIFT);
		y = -y;
	}

	temp |= ((x & CURSOR_POS_MASK) << CURSOR_X_SHIFT);
	temp |= ((y & CURSOR_POS_MASK) << CURSOR_Y_SHIFT);

	adder = psb_intel_crtc->cursor_addr;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_ONLY_IF_ON)) {
		REG_WRITE((pipe == 0) ? CURAPOS : CURBPOS, temp);
		REG_WRITE((pipe == 0) ? CURABASE : CURBBASE, adder);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}
	return 0;
}

static void psb_intel_crtc_gamma_set(struct drm_crtc *crtc, u16 *red,
				 u16 *green, u16 *blue, uint32_t size)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int i;

	if (size != 256)
		return;

	for (i = 0; i < 256; i++) {
		psb_intel_crtc->lut_r[i] = red[i] >> 8;
		psb_intel_crtc->lut_g[i] = green[i] >> 8;
		psb_intel_crtc->lut_b[i] = blue[i] >> 8;
	}

	psb_intel_crtc_load_lut(crtc);
}

static int mrst_crtc_set_config(struct drm_mode_set *set)
{
	int ret = 0;
	struct drm_device * dev = set->crtc->dev;
	struct drm_psb_private * dev_priv = dev->dev_private;

	if(!dev_priv->rpm_enabled)
		return drm_crtc_helper_set_config(set);

	pm_runtime_forbid(&dev->pdev->dev);

	ret = drm_crtc_helper_set_config(set);

	pm_runtime_allow(&dev->pdev->dev);

	return ret;
}

/* Returns the clock of the currently programmed mode of the given pipe. */
static int psb_intel_crtc_clock_get(struct drm_device *dev,
				struct drm_crtc *crtc)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	u32 dpll;
	u32 fp;
	struct psb_intel_clock_t clock;
	bool is_lvds;
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_ONLY_IF_ON)) {
		dpll = REG_READ((pipe == 0) ? DPLL_A : DPLL_B);
		if ((dpll & DISPLAY_RATE_SELECT_FPA1) == 0)
			fp = REG_READ((pipe == 0) ? FPA0 : FPB0);
		else
			fp = REG_READ((pipe == 0) ? FPA1 : FPB1);
		is_lvds = (pipe == 1) && (REG_READ(LVDS) & LVDS_PORT_EN);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		dpll = (pipe == 0) ?
			dev_priv->saveDPLL_A : dev_priv->saveDPLL_B;

		if ((dpll & DISPLAY_RATE_SELECT_FPA1) == 0)
			fp = (pipe == 0) ?
				dev_priv->saveFPA0 :
				dev_priv->saveFPB0;
		else
			fp = (pipe == 0) ?
				dev_priv->saveFPA1 :
				dev_priv->saveFPB1;

		is_lvds = (pipe == 1) && (dev_priv->saveLVDS & LVDS_PORT_EN);
	}

	clock.m1 = (fp & FP_M1_DIV_MASK) >> FP_M1_DIV_SHIFT;
	clock.m2 = (fp & FP_M2_DIV_MASK) >> FP_M2_DIV_SHIFT;
	clock.n = (fp & FP_N_DIV_MASK) >> FP_N_DIV_SHIFT;

	if (is_lvds) {
		clock.p1 =
		    ffs((dpll &
			 DPLL_FPA01_P1_POST_DIV_MASK_I830_LVDS) >>
			DPLL_FPA01_P1_POST_DIV_SHIFT);
		clock.p2 = 14;

		if ((dpll & PLL_REF_INPUT_MASK) ==
		    PLLB_REF_INPUT_SPREADSPECTRUMIN) {
			/* XXX: might not be 66MHz */
			i8xx_clock(66000, &clock);
		} else
			i8xx_clock(48000, &clock);
	} else {
		if (dpll & PLL_P1_DIVIDE_BY_TWO)
			clock.p1 = 2;
		else {
			clock.p1 =
			    ((dpll &
			      DPLL_FPA01_P1_POST_DIV_MASK_I830) >>
			     DPLL_FPA01_P1_POST_DIV_SHIFT) + 2;
		}
		if (dpll & PLL_P2_DIVIDE_BY_4)
			clock.p2 = 4;
		else
			clock.p2 = 2;

		i8xx_clock(48000, &clock);
	}

	/* XXX: It would be nice to validate the clocks, but we can't reuse
	 * i830PllIsValid() because it relies on the xf86_config connector
	 * configuration being accurate, which it isn't necessarily.
	 */

	return clock.dot;
}

/** Returns the currently programmed mode of the given pipe. */
struct drm_display_mode *psb_intel_crtc_mode_get(struct drm_device *dev,
					     struct drm_crtc *crtc)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	struct drm_display_mode *mode;
	int htot;
	int hsync;
	int vtot;
	int vsync;
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_ONLY_IF_ON)) {
		htot = REG_READ((pipe == 0) ? HTOTAL_A : HTOTAL_B);
		hsync = REG_READ((pipe == 0) ? HSYNC_A : HSYNC_B);
		vtot = REG_READ((pipe == 0) ? VTOTAL_A : VTOTAL_B);
		vsync = REG_READ((pipe == 0) ? VSYNC_A : VSYNC_B);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		htot = (pipe == 0) ?
			dev_priv->saveHTOTAL_A : dev_priv->saveHTOTAL_B;
		hsync = (pipe == 0) ?
			dev_priv->saveHSYNC_A : dev_priv->saveHSYNC_B;
		vtot = (pipe == 0) ?
			dev_priv->saveVTOTAL_A : dev_priv->saveVTOTAL_B;
		vsync = (pipe == 0) ?
			dev_priv->saveVSYNC_A : dev_priv->saveVSYNC_B;
	}

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->clock = psb_intel_crtc_clock_get(dev, crtc);
	mode->hdisplay = (htot & 0xffff) + 1;
	mode->htotal = ((htot & 0xffff0000) >> 16) + 1;
	mode->hsync_start = (hsync & 0xffff) + 1;
	mode->hsync_end = ((hsync & 0xffff0000) >> 16) + 1;
	mode->vdisplay = (vtot & 0xffff) + 1;
	mode->vtotal = ((vtot & 0xffff0000) >> 16) + 1;
	mode->vsync_start = (vsync & 0xffff) + 1;
	mode->vsync_end = ((vsync & 0xffff0000) >> 16) + 1;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void psb_intel_crtc_destroy(struct drm_crtc *crtc)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);

#ifndef CONFIG_X86_MRST
	kfree(psb_intel_crtc->crtc_state);
#endif
	drm_crtc_cleanup(crtc);
	kfree(psb_intel_crtc);
}

static const struct drm_crtc_helper_funcs psb_intel_helper_funcs = {
	.dpms = psb_intel_crtc_dpms,
	.mode_fixup = psb_intel_crtc_mode_fixup,
	.mode_set = psb_intel_crtc_mode_set,
	.mode_set_base = psb_intel_pipe_set_base,
	.prepare = psb_intel_crtc_prepare,
	.commit = psb_intel_crtc_commit,
};

static const struct drm_crtc_helper_funcs mrst_helper_funcs;
static const struct drm_crtc_helper_funcs mdfld_helper_funcs;
const struct drm_crtc_funcs mdfld_intel_crtc_funcs;

const struct drm_crtc_funcs psb_intel_crtc_funcs = {
#ifndef CONFIG_X86_MRST
	.save = psb_intel_crtc_save,
	.restore = psb_intel_crtc_restore,
#endif
	.cursor_set = psb_intel_crtc_cursor_set,
	.cursor_move = psb_intel_crtc_cursor_move,
	.gamma_set = psb_intel_crtc_gamma_set,
	.set_config = mrst_crtc_set_config,
	.destroy = psb_intel_crtc_destroy,
};

void psb_intel_crtc_init(struct drm_device *dev, int pipe,
		     struct psb_intel_mode_device *mode_dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_intel_crtc *psb_intel_crtc;
	int i;
	uint16_t *r_base, *g_base, *b_base;

	PSB_DEBUG_ENTRY("\n");

	/* We allocate a extra array of drm_connector pointers
	 * for fbdev after the crtc */
	psb_intel_crtc =
	    kzalloc(sizeof(struct psb_intel_crtc) +
		    (INTELFB_CONN_LIMIT * sizeof(struct drm_connector *)),
		    GFP_KERNEL);
	if (psb_intel_crtc == NULL)
		return;

#ifndef CONFIG_X86_MRST
	psb_intel_crtc->crtc_state =
		kzalloc(sizeof(struct psb_intel_crtc_state), GFP_KERNEL);
	if (!psb_intel_crtc->crtc_state) {
		DRM_INFO("Crtc state error: No memory\n");
		kfree(psb_intel_crtc);
		return;
	}
#endif

	if (IS_MDFLD(dev)) {
		drm_crtc_init(dev, &psb_intel_crtc->base, &mdfld_intel_crtc_funcs);
	} else {
		drm_crtc_init(dev, &psb_intel_crtc->base, &psb_intel_crtc_funcs);
	}

	drm_mode_crtc_set_gamma_size(&psb_intel_crtc->base, 256);
	psb_intel_crtc->pipe = pipe;
	psb_intel_crtc->plane = pipe;

	r_base = psb_intel_crtc->base.gamma_store;
	g_base = r_base + 256;
	b_base = g_base + 256;
	for (i = 0; i < 256; i++) {
		psb_intel_crtc->lut_r[i] = i;
		psb_intel_crtc->lut_g[i] = i;
		psb_intel_crtc->lut_b[i] = i;
		r_base[i] = i << 8;
		g_base[i] = i << 8;
		b_base[i] = i << 8;

		psb_intel_crtc->lut_adj[i] = 0;
	}

	psb_intel_crtc->mode_dev = mode_dev;
	psb_intel_crtc->cursor_addr = 0;

	if (IS_MRST(dev)) {
		drm_crtc_helper_add(&psb_intel_crtc->base, &mrst_helper_funcs);
	} else if (IS_MDFLD(dev)) {
		drm_crtc_helper_add(&psb_intel_crtc->base, &mdfld_helper_funcs);
	} else {
		drm_crtc_helper_add(&psb_intel_crtc->base,
				    &psb_intel_helper_funcs);
	}

	/* Setup the array of drm_connector pointer array */
	psb_intel_crtc->mode_set.crtc = &psb_intel_crtc->base;
	BUG_ON(pipe >= ARRAY_SIZE(dev_priv->plane_to_crtc_mapping) ||
	       dev_priv->plane_to_crtc_mapping[psb_intel_crtc->plane] != NULL);
	dev_priv->plane_to_crtc_mapping[psb_intel_crtc->plane] = &psb_intel_crtc->base;
	dev_priv->pipe_to_crtc_mapping[psb_intel_crtc->pipe] = &psb_intel_crtc->base;
	psb_intel_crtc->mode_set.connectors =
	    (struct drm_connector **) (psb_intel_crtc + 1);
	psb_intel_crtc->mode_set.num_connectors = 0;
}

int psb_intel_get_pipe_from_crtc_id(struct drm_device *dev, void *data,
				struct drm_file *file_priv)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct drm_psb_get_pipe_from_crtc_id_arg *pipe_from_crtc_id = data;
	struct drm_mode_object *drmmode_obj;
	struct psb_intel_crtc *crtc;

	if (!dev_priv) {
		DRM_ERROR("called with no initialization\n");
		return -EINVAL;
	}

	drmmode_obj = drm_mode_object_find(dev, pipe_from_crtc_id->crtc_id,
			DRM_MODE_OBJECT_CRTC);

	if (!drmmode_obj) {
		DRM_ERROR("no such CRTC id\n");
		return -EINVAL;
	}

	crtc = to_psb_intel_crtc(obj_to_crtc(drmmode_obj));
	pipe_from_crtc_id->pipe = crtc->pipe;

	return 0;
}

struct drm_crtc *psb_intel_get_crtc_from_pipe(struct drm_device *dev, int pipe)
{
	struct drm_crtc *crtc = NULL;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
		if (psb_intel_crtc->pipe == pipe)
			break;
	}
	return crtc;
}

int psb_intel_connector_clones(struct drm_device *dev, int type_mask)
{
	int index_mask = 0;
	struct drm_connector *connector;
	int entry = 0;

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		struct psb_intel_output *psb_intel_output =
		    to_psb_intel_output(connector);
		if (type_mask & (1 << psb_intel_output->type))
			index_mask |= (1 << entry);
		entry++;
	}
	return index_mask;
}

#if 0				/* JB: Should be per device */
static void psb_intel_setup_outputs(struct drm_device *dev)
{
	struct drm_connector *connector;

	psb_intel_crt_init(dev);

	/* Set up integrated LVDS */
	if (IS_MOBILE(dev) && !IS_I830(dev))
		psb_intel_lvds_init(dev);

	if (IS_I9XX(dev)) {
		psb_intel_sdvo_init(dev, SDVOB);
		psb_intel_sdvo_init(dev, SDVOC);
	} else
		psb_intel_dvo_init(dev);

	if (IS_I9XX(dev) && !IS_I915G(dev))
		psb_intel_tv_init(dev);

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		struct psb_intel_output *psb_intel_output =
		    to_psb_intel_output(connector);
		struct drm_encoder *encoder = &psb_intel_output->enc;
		int crtc_mask = 0, clone_mask = 0;

		/* valid crtcs */
		switch (psb_intel_output->type) {
		case INTEL_OUTPUT_DVO:
		case INTEL_OUTPUT_SDVO:
			crtc_mask = ((1 << 0) | (1 << 1));
			clone_mask = ((1 << INTEL_OUTPUT_ANALOG) |
				      (1 << INTEL_OUTPUT_DVO) |
				      (1 << INTEL_OUTPUT_SDVO));
			break;
		case INTEL_OUTPUT_ANALOG:
			crtc_mask = ((1 << 0) | (1 << 1));
			clone_mask = ((1 << INTEL_OUTPUT_ANALOG) |
				      (1 << INTEL_OUTPUT_DVO) |
				      (1 << INTEL_OUTPUT_SDVO));
			break;
		case INTEL_OUTPUT_LVDS:
			crtc_mask = (1 << 1);
			clone_mask = (1 << INTEL_OUTPUT_LVDS);
			break;
		case INTEL_OUTPUT_TVOUT:
			crtc_mask = ((1 << 0) | (1 << 1));
			clone_mask = (1 << INTEL_OUTPUT_TVOUT);
			break;
		}
		encoder->possible_crtcs = crtc_mask;
		encoder->possible_clones =
		    psb_intel_connector_clones(dev, clone_mask);
	}
}
#endif

#if 0	/* JB: Rework framebuffer code into something none device specific */
static void psb_intel_user_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct psb_intel_framebuffer *psb_intel_fb =
					to_psb_intel_framebuffer(fb);
	struct drm_device *dev = fb->dev;

	if (fb->fbdev)
		intelfb_remove(dev, fb);

	drm_framebuffer_cleanup(fb);
	drm_gem_object_unreference(fb->mm_private);

	kfree(psb_intel_fb);
}

static int psb_intel_user_framebuffer_create_handle(struct drm_framebuffer *fb,
						struct drm_file *file_priv,
						unsigned int *handle)
{
	struct drm_gem_object *object = fb->mm_private;

	return drm_gem_handle_create(file_priv, object, handle);
}

static const struct drm_framebuffer_funcs psb_intel_fb_funcs = {
	.destroy = psb_intel_user_framebuffer_destroy,
	.create_handle = psb_intel_user_framebuffer_create_handle,
};

struct drm_framebuffer *psb_intel_framebuffer_create(struct drm_device *dev,
						 struct drm_mode_fb_cmd
						 *mode_cmd,
						 void *mm_private)
{
	struct psb_intel_framebuffer *psb_intel_fb;

	psb_intel_fb = kzalloc(sizeof(*psb_intel_fb), GFP_KERNEL);
	if (!psb_intel_fb)
		return NULL;

	if (!drm_framebuffer_init(dev,
				  &psb_intel_fb->base,
				  &psb_intel_fb_funcs))
		return NULL;

	drm_helper_mode_fill_fb_struct(&psb_intel_fb->base, mode_cmd);

	return &psb_intel_fb->base;
}


static struct drm_framebuffer *psb_intel_user_framebuffer_create(struct
							     drm_device
							     *dev,
							     struct
							     drm_file
							     *filp,
							     struct
							     drm_mode_fb_cmd
							     *mode_cmd)
{
	struct drm_gem_object *obj;

	obj = drm_gem_object_lookup(dev, filp, mode_cmd->handle);
	if (!obj)
		return NULL;

	return psb_intel_framebuffer_create(dev, mode_cmd, obj);
}

static int psb_intel_insert_new_fb(struct drm_device *dev,
			       struct drm_file *file_priv,
			       struct drm_framebuffer *fb,
			       struct drm_mode_fb_cmd *mode_cmd)
{
	struct psb_intel_framebuffer *psb_intel_fb;
	struct drm_gem_object *obj;
	struct drm_crtc *crtc;

	psb_intel_fb = to_psb_intel_framebuffer(fb);

	mutex_lock(&dev->struct_mutex);
	obj = drm_gem_object_lookup(dev, file_priv, mode_cmd->handle);

	if (!obj) {
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}
	drm_gem_object_unreference(psb_intel_fb->base.mm_private);
	drm_helper_mode_fill_fb_struct(fb, mode_cmd, obj);
	mutex_unlock(&dev->struct_mutex);

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (crtc->fb == fb) {
			struct drm_crtc_helper_funcs *crtc_funcs =
			    crtc->helper_private;
			crtc_funcs->mode_set_base(crtc, crtc->x, crtc->y);
		}
	}
	return 0;
}

static const struct drm_mode_config_funcs psb_intel_mode_funcs = {
	.resize_fb = psb_intel_insert_new_fb,
	.fb_create = psb_intel_user_framebuffer_create,
	.fb_changed = intelfb_probe,
};
#endif

#if 0				/* Should be per device */
void psb_intel_modeset_init(struct drm_device *dev)
{
	int num_pipe;
	int i;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.funcs = (void *) &psb_intel_mode_funcs;

	if (IS_I965G(dev)) {
		dev->mode_config.max_width = 8192;
		dev->mode_config.max_height = 8192;
	} else {
		dev->mode_config.max_width = 2048;
		dev->mode_config.max_height = 2048;
	}

	/* set memory base */
	/* MRST and PSB should use BAR 2*/
	dev->mode_config.fb_base =
	    pci_resource_start(dev->pdev, 2);

	if (IS_MOBILE(dev) || IS_I9XX(dev))
		num_pipe = 2;
	else
		num_pipe = 1;
	DRM_DEBUG("%d display pipe%s available.\n",
		  num_pipe, num_pipe > 1 ? "s" : "");

	for (i = 0; i < num_pipe; i++)
		psb_intel_crtc_init(dev, i);

	psb_intel_setup_outputs(dev);

	/* setup fbs */
	/* drm_initial_config(dev); */
}
#endif

void psb_intel_modeset_cleanup(struct drm_device *dev)
{
	drm_mode_config_cleanup(dev);
}


/* current intel driver doesn't take advantage of encoders
   always give back the encoder for the connector
*/
struct drm_encoder *psb_intel_best_encoder(struct drm_connector *connector)
{
	struct psb_intel_output *psb_intel_output =
					to_psb_intel_output(connector);

	return &psb_intel_output->enc;
}

/* MRST_PLATFORM start */
struct mrst_limit_t {
	struct psb_intel_range_t dot, m, p1;
};

struct mrst_clock_t {
	/* derived values */
	int dot;
	int m;
	int p1;
};

#define MRST_LIMIT_LVDS_100L	    0
#define MRST_LIMIT_LVDS_83	    1
#define MRST_LIMIT_LVDS_100	    2

#define MRST_DOT_MIN		  19750
#define MRST_DOT_MAX		  120000
#define MRST_M_MIN_100L		    20
#define MRST_M_MIN_100		    10
#define MRST_M_MIN_83		    12
#define MRST_M_MAX_100L		    34
#define MRST_M_MAX_100		    17
#define MRST_M_MAX_83		    20
#define MRST_P1_MIN		    2
#define MRST_P1_MAX_0		    7
#define MRST_P1_MAX_1		    8

static const struct mrst_limit_t mrst_limits[] = {
	{			/* MRST_LIMIT_LVDS_100L */
	 .dot = {.min = MRST_DOT_MIN, .max = MRST_DOT_MAX},
	 .m = {.min = MRST_M_MIN_100L, .max = MRST_M_MAX_100L},
	 .p1 = {.min = MRST_P1_MIN, .max = MRST_P1_MAX_1},
	 },
	{			/* MRST_LIMIT_LVDS_83L */
	 .dot = {.min = MRST_DOT_MIN, .max = MRST_DOT_MAX},
	 .m = {.min = MRST_M_MIN_83, .max = MRST_M_MAX_83},
	 .p1 = {.min = MRST_P1_MIN, .max = MRST_P1_MAX_0},
	 },
	{			/* MRST_LIMIT_LVDS_100 */
	 .dot = {.min = MRST_DOT_MIN, .max = MRST_DOT_MAX},
	 .m = {.min = MRST_M_MIN_100, .max = MRST_M_MAX_100},
	 .p1 = {.min = MRST_P1_MIN, .max = MRST_P1_MAX_1},
	 },
};

#define MRST_M_MIN	    10
static const u32 mrst_m_converts[] = {
	0x2B, 0x15, 0x2A, 0x35, 0x1A, 0x0D, 0x26, 0x33, 0x19, 0x2C,
	0x36, 0x3B, 0x1D, 0x2E, 0x37, 0x1B, 0x2D, 0x16, 0x0B, 0x25,
	0x12, 0x09, 0x24, 0x32, 0x39, 0x1c,
};

#define COUNT_MAX 1000
void mrstWaitForPipeDisable(struct drm_device *dev)
{
	int count, temp;

	/* FIXME it should be removed after validation. */
	if (IS_MRST(dev)) {
		psb_intel_wait_for_vblank(dev);
		return;
	}

	/* Wait for for the pipe disable to take effect. */
	for (count = 0; count < COUNT_MAX; count++) {
		temp = REG_READ(PIPEACONF);
		if ((temp & PIPEACONF_PIPE_STATE) == 0)
			break;
	}

	PSB_DEBUG_ENTRY("cout = %d. \n", count);
}

void mrstWaitForPipeEnable(struct drm_device *dev)
{
	int count, temp;

	/* FIXME it should be removed after validation. */
	if (IS_MRST(dev)) {
		psb_intel_wait_for_vblank(dev);
		return;
	}

	/* Wait for for the pipe enable to take effect. */
	for (count = 0; count < COUNT_MAX; count++) {
		temp = REG_READ(PIPEACONF);
		if ((temp & PIPEACONF_PIPE_STATE) == 1)
			break;
	}

	PSB_DEBUG_ENTRY("cout = %d. \n", count);
}

static const struct mrst_limit_t *mrst_limit(struct drm_crtc *crtc)
{
	const struct mrst_limit_t *limit = NULL;
	struct drm_device *dev = crtc->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	if (psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_LVDS)
	    || psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_MIPI)) {
		switch (dev_priv->core_freq) {
		case 100:
			limit = &mrst_limits[MRST_LIMIT_LVDS_100L];
			break;
		case 166:
			limit = &mrst_limits[MRST_LIMIT_LVDS_83];
			break;
		case 200:
			limit = &mrst_limits[MRST_LIMIT_LVDS_100];
			break;
		}
	} else {
		limit = NULL;
		PSB_DEBUG_ENTRY("mrst_limit Wrong display type. \n");
	}

	return limit;
}

/** Derive the pixel clock for the given refclk and divisors for 8xx chips. */
static void mrst_clock(int refclk, struct mrst_clock_t *clock)
{
	clock->dot = (refclk * clock->m) / (14 * clock->p1);
}

void mrstPrintPll(char *prefix, struct mrst_clock_t *clock)
{
	PSB_DEBUG_ENTRY
	    ("%s: dotclock = %d,  m = %d, p1 = %d. \n",
	     prefix, clock->dot, clock->m, clock->p1);
}

/**
 * Returns a set of divisors for the desired target clock with the given refclk,
 * or FALSE.  Divisor values are the actual divisors for
 */
static bool
mrstFindBestPLL(struct drm_crtc *crtc, int target, int refclk,
		struct mrst_clock_t *best_clock)
{
	struct mrst_clock_t clock;
	const struct mrst_limit_t *limit = mrst_limit(crtc);
	int err = target;

	memset(best_clock, 0, sizeof(*best_clock));

	for (clock.m = limit->m.min; clock.m <= limit->m.max; clock.m++) {
		for (clock.p1 = limit->p1.min; clock.p1 <= limit->p1.max;
		     clock.p1++) {
			int this_err;

			mrst_clock(refclk, &clock);

			this_err = abs(clock.dot - target);
			if (this_err < err) {
				*best_clock = clock;
				err = this_err;
			}
		}
	}
	DRM_DEBUG("mrstFindBestPLL err = %d.\n", err);

	return err != target;
}

/**
 * Sets the power management mode of the pipe and plane.
 *
 * This code should probably grow support for turning the cursor off and back
 * on appropriately at the same time as we're turning the pipe off/on.
 */
static void mrst_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct drm_psb_private *dev_priv = dev->dev_private;
	int pipe = psb_intel_crtc->pipe;
	int dpll_reg = (pipe == 0) ? MRST_DPLL_A : DPLL_B;
	int dspcntr_reg = (pipe == 0) ? DSPACNTR : DSPBCNTR;
	int dspbase_reg = (pipe == 0) ? MRST_DSPABASE : DSPBBASE;
	int pipeconf_reg = (pipe == 0) ? PIPEACONF : PIPEBCONF;
	u32 temp;
	bool enabled;

	PSB_DEBUG_ENTRY("mode = %d, pipe = %d \n", mode, pipe);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	/* XXX: When our outputs are all unaware of DPMS modes other than off
	 * and on, we should map those modes to DRM_MODE_DPMS_OFF in the CRTC.
	 */
	switch (mode) {
	case DRM_MODE_DPMS_ON:
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
		/* Enable the DPLL */
		temp = REG_READ(dpll_reg);
		if ((temp & DPLL_VCO_ENABLE) == 0) {
			REG_WRITE(dpll_reg, temp);
			REG_READ(dpll_reg);
			/* Wait for the clocks to stabilize. */
			udelay(150);
			REG_WRITE(dpll_reg, temp | DPLL_VCO_ENABLE);
			REG_READ(dpll_reg);
			/* Wait for the clocks to stabilize. */
			udelay(150);
			REG_WRITE(dpll_reg, temp | DPLL_VCO_ENABLE);
			REG_READ(dpll_reg);
			/* Wait for the clocks to stabilize. */
			udelay(150);
		}
		if (dev_priv->iLVDS_enable || \
			!dev_priv->dsi_plane_pipe_control) {
			/* Enable the pipe */
			temp = REG_READ(pipeconf_reg);
			if ((temp & PIPEACONF_ENABLE) == 0)
				REG_WRITE(pipeconf_reg, temp | PIPEACONF_ENABLE);

			/* Enable the plane */
			temp = REG_READ(dspcntr_reg);
			if ((temp & DISPLAY_PLANE_ENABLE) == 0) {
				REG_WRITE(dspcntr_reg,
					  temp | DISPLAY_PLANE_ENABLE);
				/* Flush the plane changes */
				REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
			}
		}

		psb_intel_crtc_load_lut(crtc);

		/* Give the overlay scaler a chance to enable
		   if it's on this pipe */
		/* psb_intel_crtc_dpms_video(crtc, true); TODO */
		break;
	case DRM_MODE_DPMS_OFF:
		/* Give the overlay scaler a chance to disable
		 * if it's on this pipe */
		/* psb_intel_crtc_dpms_video(crtc, FALSE); TODO */

		/* Disable the VGA plane that we never use */
		REG_WRITE(VGACNTRL, VGA_DISP_DISABLE);
		if (dev_priv->iLVDS_enable || \
			!dev_priv->dsi_plane_pipe_control) {
			/* Disable display plane */
			temp = REG_READ(dspcntr_reg);
			if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
				REG_WRITE(dspcntr_reg,
					  temp & ~DISPLAY_PLANE_ENABLE);
				/* Flush the plane changes */
				REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
				REG_READ(dspbase_reg);
			}

			if (!IS_I9XX(dev)) {
				/* Wait for vblank for the disable to take effect */
				psb_intel_wait_for_vblank(dev);
			}

			/* Next, disable display pipes */
			temp = REG_READ(pipeconf_reg);
			if ((temp & PIPEACONF_ENABLE) != 0) {
				REG_WRITE(pipeconf_reg, temp & ~PIPEACONF_ENABLE);
				REG_READ(pipeconf_reg);
			}

			/* Wait for for the pipe disable to take effect. */
			mrstWaitForPipeDisable(dev);
		}

		temp = REG_READ(dpll_reg);
		if ((temp & DPLL_VCO_ENABLE) != 0) {
			REG_WRITE(dpll_reg, temp & ~DPLL_VCO_ENABLE);
			REG_READ(dpll_reg);
		}

		/* Wait for the clocks to turn off. */
		udelay(150);

		break;
	}

	enabled = crtc->enabled && mode != DRM_MODE_DPMS_OFF;

#if 0				/* JB: Add vblank support later */
	if (enabled)
		dev_priv->vblank_pipe |= (1 << pipe);
	else
		dev_priv->vblank_pipe &= ~(1 << pipe);
#endif

#if 0				/* JB: Add sarea support later */
	if (!dev->primary->master)
		return;

	master_priv = dev->primary->master->driver_priv;
	if (!master_priv->sarea_priv)
		return;

	switch (pipe) {
	case 0:
		master_priv->sarea_priv->planeA_w =
		    enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->planeA_h =
		    enabled ? crtc->mode.vdisplay : 0;
		break;
	case 1:
		master_priv->sarea_priv->planeB_w =
		    enabled ? crtc->mode.hdisplay : 0;
		master_priv->sarea_priv->planeB_h =
		    enabled ? crtc->mode.vdisplay : 0;
		break;
	default:
		DRM_ERROR("Can't update pipe %d in SAREA\n", pipe);
		break;
	}
#endif

	/*Set FIFO Watermarks*/
	REG_WRITE(DSPARB, 0x3FFF);
	REG_WRITE(DSPFW1, 0x3F88080A);
	REG_WRITE(DSPFW2, 0x0b060808);
	REG_WRITE(DSPFW3, 0x0);
	REG_WRITE(DSPFW4, 0x08030404);
	REG_WRITE(DSPFW5, 0x04040404);
	REG_WRITE(DSPFW6, 0x78);
	REG_WRITE(0x70400, REG_READ(0x70400) | 0x4000);
	/* Must write Bit 14 of the Chicken Bit Register */

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static int mrst_crtc_mode_set(struct drm_crtc *crtc,
			      struct drm_display_mode *mode,
			      struct drm_display_mode *adjusted_mode,
			      int x, int y,
			      struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	int pipe = psb_intel_crtc->pipe;
	int fp_reg = (pipe == 0) ? MRST_FPA0 : FPB0;
	int dpll_reg = (pipe == 0) ? MRST_DPLL_A : DPLL_B;
	int dspcntr_reg = (pipe == 0) ? DSPACNTR : DSPBCNTR;
	int pipeconf_reg = (pipe == 0) ? PIPEACONF : PIPEBCONF;
	int htot_reg = (pipe == 0) ? HTOTAL_A : HTOTAL_B;
	int hblank_reg = (pipe == 0) ? HBLANK_A : HBLANK_B;
	int hsync_reg = (pipe == 0) ? HSYNC_A : HSYNC_B;
	int vtot_reg = (pipe == 0) ? VTOTAL_A : VTOTAL_B;
	int vblank_reg = (pipe == 0) ? VBLANK_A : VBLANK_B;
	int vsync_reg = (pipe == 0) ? VSYNC_A : VSYNC_B;
	int pipesrc_reg = (pipe == 0) ? PIPEASRC : PIPEBSRC;
	int refclk = 0;
	struct mrst_clock_t clock;
	u32 dpll = 0, fp = 0, dspcntr, pipeconf, lvdsport;
	bool ok, is_sdvo = false;
	bool is_crt = false, is_lvds = false, is_tv = false;
	bool is_mipi = false;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct psb_intel_output *psb_intel_output = NULL;
	uint64_t scalingType = DRM_MODE_SCALE_CENTER;
	struct drm_encoder *encoder;

	PSB_DEBUG_ENTRY("pipe = 0x%x \n", pipe);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return 0;

	memcpy(&psb_intel_crtc->saved_mode,
		mode,
		sizeof(struct drm_display_mode));
	memcpy(&psb_intel_crtc->saved_adjusted_mode,
		adjusted_mode,
		sizeof(struct drm_display_mode));

	list_for_each_entry(encoder, &mode_config->encoder_list, head) {

		if (encoder->crtc != crtc)
			continue;

		psb_intel_output = enc_to_psb_intel_output(encoder);
		switch (psb_intel_output->type) {
		case INTEL_OUTPUT_LVDS:
			is_lvds = true;
			break;
		case INTEL_OUTPUT_SDVO:
			is_sdvo = true;
			break;
		case INTEL_OUTPUT_TVOUT:
			is_tv = true;
			break;
		case INTEL_OUTPUT_ANALOG:
			is_crt = true;
			break;
		case INTEL_OUTPUT_MIPI:
			is_mipi = true;
			break;
		}
	}

	/* Disable the VGA plane that we never use */
	REG_WRITE(VGACNTRL, VGA_DISP_DISABLE);

	/* Disable the panel fitter if it was on our pipe */
	if (psb_intel_panel_fitter_pipe(dev) == pipe)
		REG_WRITE(PFIT_CONTROL, 0);

	REG_WRITE(pipesrc_reg,
		  ((mode->crtc_hdisplay - 1) << 16) |
		  (mode->crtc_vdisplay - 1));

	if (psb_intel_output)
		drm_connector_property_get_value(&psb_intel_output->base,
			dev->mode_config.scaling_mode_property, &scalingType);

	if (scalingType == DRM_MODE_SCALE_NO_SCALE) {
		/* Moorestown doesn't have register support for centering so
		 * we need to mess with the h/vblank and h/vsync start and
		 * ends to get centering */
		int offsetX = 0, offsetY = 0;

		offsetX = (adjusted_mode->crtc_hdisplay -
			   mode->crtc_hdisplay) / 2;
		offsetY = (adjusted_mode->crtc_vdisplay -
			   mode->crtc_vdisplay) / 2;

		REG_WRITE(htot_reg, (mode->crtc_hdisplay - 1) |
			((adjusted_mode->crtc_htotal - 1) << 16));
		REG_WRITE(vtot_reg, (mode->crtc_vdisplay - 1) |
			((adjusted_mode->crtc_vtotal - 1) << 16));
		REG_WRITE(hblank_reg,
			(adjusted_mode->crtc_hblank_start - offsetX - 1) |
			((adjusted_mode->crtc_hblank_end - offsetX - 1) << 16));
		REG_WRITE(hsync_reg,
			(adjusted_mode->crtc_hsync_start - offsetX - 1) |
			((adjusted_mode->crtc_hsync_end - offsetX - 1) << 16));
		REG_WRITE(vblank_reg,
			(adjusted_mode->crtc_vblank_start - offsetY - 1) |
			((adjusted_mode->crtc_vblank_end - offsetY - 1) << 16));
		REG_WRITE(vsync_reg,
			(adjusted_mode->crtc_vsync_start - offsetY - 1) |
			((adjusted_mode->crtc_vsync_end - offsetY - 1) << 16));
	} else {
		REG_WRITE(htot_reg, (adjusted_mode->crtc_hdisplay - 1) |
			((adjusted_mode->crtc_htotal - 1) << 16));
		REG_WRITE(vtot_reg, (adjusted_mode->crtc_vdisplay - 1) |
			((adjusted_mode->crtc_vtotal - 1) << 16));
		REG_WRITE(hblank_reg, (adjusted_mode->crtc_hblank_start - 1) |
			((adjusted_mode->crtc_hblank_end - 1) << 16));
		REG_WRITE(hsync_reg, (adjusted_mode->crtc_hsync_start - 1) |
			((adjusted_mode->crtc_hsync_end - 1) << 16));
		REG_WRITE(vblank_reg, (adjusted_mode->crtc_vblank_start - 1) |
			((adjusted_mode->crtc_vblank_end - 1) << 16));
		REG_WRITE(vsync_reg, (adjusted_mode->crtc_vsync_start - 1) |
			((adjusted_mode->crtc_vsync_end - 1) << 16));
	}

	/* Flush the plane changes */
	{
		struct drm_crtc_helper_funcs *crtc_funcs =
		    crtc->helper_private;
		crtc_funcs->mode_set_base(crtc, x, y, old_fb);
	}

	/* setup pipeconf */
	pipeconf = REG_READ(pipeconf_reg);

	/* Set up the display plane register */
	dspcntr = REG_READ(dspcntr_reg);
	dspcntr |= DISPPLANE_GAMMA_ENABLE;

	if (pipe == 0)
		dspcntr |= DISPPLANE_SEL_PIPE_A;
	else
		dspcntr |= DISPPLANE_SEL_PIPE_B;

	dev_priv->dspcntr = dspcntr |= DISPLAY_PLANE_ENABLE;
	dev_priv->pipeconf = pipeconf |= PIPEACONF_ENABLE;

	if (is_mipi)
		goto mrst_crtc_mode_set_exit;

	refclk = dev_priv->core_freq * 1000;

	dpll = 0;		/*BIT16 = 0 for 100MHz reference */

	ok = mrstFindBestPLL(crtc, adjusted_mode->clock, refclk, &clock);

	if (!ok) {
		PSB_DEBUG_ENTRY("mrstFindBestPLL fail in mrst_crtc_mode_set. \n");
	} else {
		PSB_DEBUG_ENTRY("mrst_crtc_mode_set pixel clock = %d,"
			 "m = %x, p1 = %x. \n", clock.dot, clock.m,
			 clock.p1);
	}

	fp = mrst_m_converts[(clock.m - MRST_M_MIN)] << 8;

	dpll |= DPLL_VGA_MODE_DIS;


	dpll |= DPLL_VCO_ENABLE;

	if (is_lvds)
		dpll |= DPLLA_MODE_LVDS;
	else
		dpll |= DPLLB_MODE_DAC_SERIAL;

	if (is_sdvo) {
		int sdvo_pixel_multiply =
		    adjusted_mode->clock / mode->clock;

		dpll |= DPLL_DVO_HIGH_SPEED;
		dpll |=
		    (sdvo_pixel_multiply -
		     1) << SDVO_MULTIPLIER_SHIFT_HIRES;
	}


	/* compute bitmask from p1 value */
	dpll |= (1 << (clock.p1 - 2)) << 17;

	dpll |= DPLL_VCO_ENABLE;

	mrstPrintPll("chosen", &clock);

	if (dpll & DPLL_VCO_ENABLE) {
		REG_WRITE(fp_reg, fp);
		REG_WRITE(dpll_reg, dpll & ~DPLL_VCO_ENABLE);
		REG_READ(dpll_reg);
/* FIXME check the DPLLA lock bit PIPEACONF[29] */
		udelay(150);
	}

	/* The LVDS pin pair needs to be on before the DPLLs are enabled.
	 * This is an exception to the general rule that mode_set doesn't turn
	 * things on.
	 */
	if (is_lvds) {

		/*lvdsport = 0x803003c0;*/
		/*lvdsport = 0x813003c0;*/
		lvdsport = dev_priv->gct_data.Panel_Port_Control;

		/*REG_WRITE(LVDS, lvdsport); */
	}

	REG_WRITE(fp_reg, fp);
	REG_WRITE(dpll_reg, dpll);
	REG_READ(dpll_reg);
	/* Wait for the clocks to stabilize. */
	udelay(150);

	/* write it again -- the BIOS does, after all */
	REG_WRITE(dpll_reg, dpll);
	REG_READ(dpll_reg);
	/* Wait for the clocks to stabilize. */
	udelay(150);

	REG_WRITE(pipeconf_reg, pipeconf);
	REG_READ(pipeconf_reg);

	/* Wait for for the pipe enable to take effect. */
	mrstWaitForPipeEnable(dev);

	REG_WRITE(dspcntr_reg, dspcntr);
	psb_intel_wait_for_vblank(dev);

mrst_crtc_mode_set_exit:

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

static const struct drm_crtc_helper_funcs mrst_helper_funcs = {
	.dpms = mrst_crtc_dpms,
	.mode_fixup = psb_intel_crtc_mode_fixup,
	.mode_set = mrst_crtc_mode_set,
	.mode_set_base = psb_intel_pipe_set_base,
	.prepare = psb_intel_crtc_prepare,
	.commit = psb_intel_crtc_commit,
};

static const struct drm_crtc_helper_funcs mdfld_helper_funcs = {
	.dpms = mdfld_crtc_dpms,
	.mode_fixup = psb_intel_crtc_mode_fixup,
	.mode_set = mdfld_crtc_mode_set,
	.mode_set_base = mdfld__intel_pipe_set_base,
	.prepare = psb_intel_crtc_prepare,
	.commit = psb_intel_crtc_commit,
};

/* MRST_PLATFORM end */

#include "psb_intel_display2.c"
