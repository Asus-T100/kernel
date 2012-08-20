/*
 * Copyright © 2010 Intel Corporation
 *
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * jim liu <jim.liu@intel.com>
 * Jackie Li<yaodong.li@intel.com>
 */

#ifndef __MDFLD_DSI_DPI_H__
#define __MDFLD_DSI_DPI_H__

#include "mdfld_dsi_output.h"
#include "mdfld_output.h"

struct mdfld_dsi_dpi_timing {
	u16 hsync_count;
	u16 hbp_count;
	u16 hfp_count;
	u16 hactive_count;
	u16 vsync_count;
	u16 vbp_count;
	u16 vfp_count;
};

struct mdfld_dsi_dpi_output {
	struct mdfld_dsi_encoder base;
	struct drm_device *dev;
	
	int panel_on;
	int first_boot;

	struct panel_funcs *p_funcs;
};

#define MDFLD_DSI_DPI_OUTPUT(dsi_encoder) \
	container_of(dsi_encoder, struct mdfld_dsi_dpi_output, base)

/*export functions*/
void dsi_set_bridge_reset_state(int state);
void mdfld_dsi_dpi_turn_on(struct mdfld_dsi_dpi_output *output,
		int pipe);
void mdfld_init_TOSHIBA_MIPI(struct drm_device *dev);
void mdfld_deinit_TOSHIBA_MIPI(struct drm_device *dev);

extern int mdfld_dsi_dpi_timing_calculation(struct drm_display_mode *mode,
				struct mdfld_dsi_dpi_timing *dpi_timing,
				int num_lane, int bpp);
extern struct mdfld_dsi_encoder *mdfld_dsi_dpi_init(struct drm_device *dev,
				struct mdfld_dsi_connector *dsi_connector,
				struct panel_funcs *p_funcs);

/*MDFLD DPI helper functions*/
extern void mdfld_dsi_dpi_dpms(struct drm_encoder *encoder, int mode);
extern bool mdfld_dsi_dpi_mode_fixup(struct drm_encoder *encoder,
				     struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode);
extern void mdfld_dsi_dpi_prepare(struct drm_encoder *encoder);
extern void mdfld_dsi_dpi_commit(struct drm_encoder *encoder);
extern void mdfld_dsi_dpi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode);
extern void mdfld_dsi_dpi_save(struct drm_encoder *encoder);
extern void mdfld_dsi_dpi_restore(struct drm_encoder *encoder);
extern void mdfld_dsi_dpi_turn_on(struct mdfld_dsi_dpi_output *output,
								int pipe);
extern void mdfld_dsi_dpi_controller_init(struct mdfld_dsi_config *dsi_config,
					int pipe);
extern void mid_enable_pipe_event(struct drm_psb_private *dev_priv, int pipe);
extern void psb_enable_pipestat(struct drm_psb_private *dev_priv, int pipe,
				u32 mask);

#endif /*__MDFLD_DSI_DPI_H__*/
