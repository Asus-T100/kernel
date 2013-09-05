/*
 * Copyright © 2008 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 * Shashank Sharma <shashank.sharma@intel.com>
 * Uma Shankar <uma.shankar@intel.com>
 * Shobhit Kumar <skumar40@intel.com>
 */

#ifndef _I915_CLR_MNGR_H_
#define _I915_CLR_MNGR_H_

struct ContBrightlut {
	short sprite_no;
	u32 val;
};

struct HueSaturationlut {
	short sprite_no;
	u32 val;
};

extern u32 CSCSoftlut[CSC_MAX_COEFF_COUNT];
extern u32 gammaSoftlut[GAMMA_CORRECT_MAX_COUNT];
extern u32 gammaSpriteSoftlut[GAMMA_SP_MAX_COUNT];

extern int parse_clrmgr_input(uint *dest, char *src, int max, int read);
extern int do_intel_enable_CSC(struct drm_device *dev, void *data,
						struct drm_crtc *crtc);
extern bool intel_pipe_has_type(struct drm_crtc *crtc, int type);
extern void do_intel_disable_CSC(struct drm_device *dev,
						struct drm_crtc *crtc);
extern int intel_crtc_enable_gamma(struct drm_crtc *crtc, u32 identifier);
extern int intel_crtc_disable_gamma(struct drm_crtc *crtc, u32 identifier);
extern int intel_sprite_cb_adjust(drm_i915_private_t *dev_priv,
		struct ContBrightlut *cb_ptr);
extern int intel_sprite_hs_adjust(drm_i915_private_t *dev_priv,
		struct HueSaturationlut *hs_ptr);
#endif
