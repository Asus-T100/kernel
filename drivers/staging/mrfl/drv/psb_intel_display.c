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
#include "pwr_mgmt.h"
#include "mrfld_clock.h"
#include "mrfld_s3d.h"

/* FIXME may delete after MRFLD PO */
#include "mrfld_display.h"

#define DRM_OUTPUT_POLL_PERIOD (10 * HZ)

/*MRFLD defines */
static int mrfld_crtc_mode_set(struct drm_crtc *crtc,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode,
			       int x, int y, struct drm_framebuffer *old_fb);
static void mrfld_crtc_dpms(struct drm_crtc *crtc, int mode);
/*MRFLD defines end */

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

struct psb_intel_p2_t {
	int dot_limit;
	int p2_slow, p2_fast;
};

#define INTEL_P2_NUM		      2

struct psb_intel_limit_t {
	struct psb_intel_range_t dot, vco, n, m, m1, m2, p, p1;
	struct psb_intel_p2_t p2;
};

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

void psb_intel_wait_for_vblank(struct drm_device *dev)
{
	/* Wait for 20ms, i.e. one cycle at 50hz. */

	/*
	 * Between kernel 3.0 and 3.3, udelay was made to complain at compile
	 * time for argument == 20000 or more.
	 * Therefore, reduce it from 20000 to 19999.
	 */
	udelay(19999);
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
	int dspstride = (pipe == 0) ? DSPASTRIDE : DSPBSTRIDE;
	int dspcntr_reg = (pipe == 0) ? DSPACNTR : DSPBCNTR;
	u32 dspcntr;
	u32 power_island = 0;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	/* no fb bound */
	if (!crtc->fb) {
		DRM_DEBUG("No FB bound\n");
		return 0;
	}

	power_island = pipe_to_island(pipe);

	if (!power_island_get(power_island))
		return 0;

	Start = mode_dev->bo_offset(dev, psbfb);
	Offset = y * crtc->fb->pitches[0] + x * (crtc->fb->bits_per_pixel / 8);

	REG_WRITE(dspstride, crtc->fb->pitches[0]);

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
	REG_WRITE(dspbase, Start + Offset);
	REG_READ(dspbase);

 psb_intel_pipe_set_base_exit:

	power_island_put(power_island);

	return ret;
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
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	return true;
}

/**
 * Return the pipe currently connected to the panel fitter,
 * or -1 if the panel fitter is not present or not in use
 */
int psb_intel_panel_fitter_pipe(struct drm_device *dev)
{
	u32 pfit_control;

	pfit_control = REG_READ(PFIT_CONTROL);

	/* See if the panel fitter is in use */
	if ((pfit_control & PFIT_ENABLE) == 0)
		return -1;

	/* 965 can place panel fitter on either pipe */
	if (IS_MID(dev))
		return (pfit_control >> 29) & 0x3;

	/* older chips can only use pipe 1 */
	return 1;
}

/** Loads the palette/gamma unit for the CRTC with the prepared values */
void psb_intel_crtc_load_lut(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int palreg = PALETTE_A;
	u32 power_island = 0;
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

	power_island = pipe_to_island(psb_intel_crtc->pipe);

	if (power_island_get(power_island)) {
		for (i = 0; i < 256; i++) {
			REG_WRITE(palreg + 4 * i,
				  ((psb_intel_crtc->lut_r[i] +
				    psb_intel_crtc->lut_adj[i]) << 16) |
				  ((psb_intel_crtc->lut_g[i] +
				    psb_intel_crtc->lut_adj[i]) << 8) |
				  (psb_intel_crtc->lut_b[i] +
				   psb_intel_crtc->lut_adj[i]));
		}
		power_island_put(power_island);
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

	/*NOTE: DSPSIZE DSPPOS only for psb */
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
		  crtc_state->saveDSPPOS, crtc_state->saveDSPBASE);

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
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
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

	DRM_DEBUG("current:(%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)\n",
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

	DRM_DEBUG("saved: (%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x)\n",
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
		  crtc_state->saveDSPPOS, crtc_state->saveDSPBASE);

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

static void psb_intel_crtc_gamma_set(struct drm_crtc *crtc, u16 * red,
					u16 * green, u16 * blue,
					uint32_t start, uint32_t size)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int i;
	int brk = (start + size > 256) ? 256 : start + size;

	for (i = start; i < brk; i++) {
		psb_intel_crtc->lut_r[i] = red[i] >> 8;
		psb_intel_crtc->lut_g[i] = green[i] >> 8;
		psb_intel_crtc->lut_b[i] = blue[i] >> 8;
	}

	psb_intel_crtc_load_lut(crtc);
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

static const struct drm_crtc_helper_funcs mrfld_helper_funcs;
const struct drm_crtc_funcs mdfld_intel_crtc_funcs;

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

	drm_crtc_init(dev, &psb_intel_crtc->base, &mdfld_intel_crtc_funcs);

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

	drm_crtc_helper_add(&psb_intel_crtc->base, &mrfld_helper_funcs);

	/* Setup the array of drm_connector pointer array */
	psb_intel_crtc->mode_set.crtc = &psb_intel_crtc->base;
	BUG_ON(pipe >= ARRAY_SIZE(dev_priv->plane_to_crtc_mapping) ||
	       dev_priv->plane_to_crtc_mapping[psb_intel_crtc->plane] != NULL);
	dev_priv->plane_to_crtc_mapping[psb_intel_crtc->plane] =
	    &psb_intel_crtc->base;
	dev_priv->pipe_to_crtc_mapping[psb_intel_crtc->pipe] =
	    &psb_intel_crtc->base;
	psb_intel_crtc->mode_set.connectors =
	    (struct drm_connector **)(psb_intel_crtc + 1);
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

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		struct psb_intel_output *psb_intel_output =
		    to_psb_intel_output(connector);
		if (type_mask & (1 << psb_intel_output->type))
			index_mask |= (1 << entry);
		entry++;
	}
	return index_mask;
}

#if 0				/* JB: Rework framebuffer code into something none device specific */
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

struct drm_framebuffer *psb_intel_framebuffer_create(struct drm_device *dev, struct drm_mode_fb_cmd2
						     *mode_cmd,
						     void *mm_private)
{
	struct psb_intel_framebuffer *psb_intel_fb;

	psb_intel_fb = kzalloc(sizeof(*psb_intel_fb), GFP_KERNEL);
	if (!psb_intel_fb)
		return NULL;

	if (!drm_framebuffer_init(dev,
				  &psb_intel_fb->base, &psb_intel_fb_funcs))
		return NULL;

	drm_helper_mode_fill_fb_struct(&psb_intel_fb->base, mode_cmd);

	return &psb_intel_fb->base;
}

static struct drm_framebuffer *psb_intel_user_framebuffer_create(struct
								 drm_device
								 *dev, struct
								 drm_file
								 *filp, struct
								 drm_mode_fb_cmd2
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
				   struct drm_mode_fb_cmd2 *mode_cmd)
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

	dev->mode_config.funcs = (void *)&psb_intel_mode_funcs;

	if (IS_I965G(dev)) {
		dev->mode_config.max_width = 8192;
		dev->mode_config.max_height = 8192;
	} else {
		dev->mode_config.max_width = 2048;
		dev->mode_config.max_height = 2048;
	}

	/* set memory base */
	/* MRST and PSB should use BAR 2 */
	dev->mode_config.fb_base = pci_resource_start(dev->pdev, 2);

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

#define COUNT_MAX 1000

static const struct drm_crtc_helper_funcs mrfld_helper_funcs = {
	.dpms = mrfld_crtc_dpms,
	.mode_fixup = psb_intel_crtc_mode_fixup,
	.mode_set = mrfld_crtc_mode_set,
	.mode_set_base = mdfld__intel_pipe_set_base,
	.prepare = psb_intel_crtc_prepare,
	.commit = psb_intel_crtc_commit,
};

static void intel_output_poll_execute(struct work_struct *work)
{
	struct delayed_work *delayed_work = to_delayed_work(work);
	struct drm_device *dev = container_of(delayed_work, struct drm_device,
			mode_config.output_poll_work);
	struct drm_connector *connector;
	enum drm_connector_status old_status;
	bool repoll = false, changed = false;

	mutex_lock(&dev->mode_config.mutex);
	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {

		/* if this is HPD or polled don't check it -
		   TV out for instance */
		if (!connector->polled)
			continue;

		else if (connector->polled & (DRM_CONNECTOR_POLL_CONNECT |
					DRM_CONNECTOR_POLL_DISCONNECT))
			repoll = true;

		old_status = connector->status;
		/* if we are connected and don't want to poll for disconnect
		   skip it */
		if (old_status == connector_status_connected &&
				!(connector->polled &
					DRM_CONNECTOR_POLL_DISCONNECT) &&
				!(connector->polled & DRM_CONNECTOR_POLL_HPD))
			continue;

		connector->status = connector->funcs->detect(connector, false);
		DRM_DEBUG_KMS("[CONNECTOR:%d:%s] status updated from" \
				"%d to %d\n",
				connector->base.id,
				drm_get_connector_name(connector),
				old_status, connector->status);
		if (old_status != connector->status)
			changed = true;
	}

	mutex_unlock(&dev->mode_config.mutex);

	if (changed) {
		/* call fbdev then send a uevent */
		if (dev->mode_config.funcs->output_poll_changed)
			dev->mode_config.funcs->output_poll_changed(dev);

		drm_sysfs_hotplug_event(dev);
	}

	if (repoll)
		queue_delayed_work(system_nrt_wq, delayed_work,
				DRM_OUTPUT_POLL_PERIOD);
}

void intel_drm_kms_helper_poll_init(struct drm_device *dev)
{
	INIT_DELAYED_WORK(&dev->mode_config.output_poll_work,
			intel_output_poll_execute);
	dev->mode_config.poll_enabled = true;

	drm_kms_helper_poll_enable(dev);
}

/* MRST_PLATFORM end */

#include "psb_intel_display2.c"
#include "mrfld_display.c"
