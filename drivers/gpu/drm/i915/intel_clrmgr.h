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


/* CSC correction */
#define CSC_MAX_COEFF_COUNT		6
#define CLR_MGR_PARSE_MAX		128
#define PIPECONF_GAMMA			(1<<24)
#define GAMMA_CORRECT_MAX_COUNT 256
#define GAMMA_SP_MAX_COUNT		6
/* Gamma correction defines */
#define GAMMA_MAX_VAL			1024
#define SHIFTBY6(val) (val<<6)
#define PIPEA_GAMMA_MAX_RED		0x70010
#define PIPEA_GAMMA_MAX_GREEN	0x70014
#define PIPEA_GAMMA_MAX_BLUE		0x70018
/* Sprite gamma correction regs */
#define GAMMA_SPA_GAMC0			0x721F4
#define GAMMA_SPA_GAMC1			0x721F0
#define GAMMA_SPA_GAMC2			0x721EC
#define GAMMA_SPA_GAMC3			0x721E8
#define GAMMA_SPA_GAMC4			0x721E4
#define GAMMA_SPA_GAMC5			0x721E0

#define GAMMA_SPB_GAMC0			0x721F4
#define GAMMA_SPB_GAMC1			0x721F0
#define GAMMA_SPB_GAMC2			0x721EC
#define GAMMA_SPB_GAMC3			0x721E8
#define GAMMA_SPB_GAMC4			0x721E4
#define GAMMA_SPB_GAMC5			0x721E0

#define GAMMA_SPA_CNTRL			0x72180
#define GAMMA_SPB_CNTRL			0x72280
#define GAMMA_ENABLE_SPR			(1<<30)
#define GAMMA_SP_MAX_COUNT			6
#define NO_SPRITE_REG				4


/* Color manager features */
enum ClrMgrFeatures {
	ClrMgrCsc = 1,
	ClrMgrGamma,
	ClrMgrContrBright,
	ClrMgrHueSat,
};

/* Required for sysfs entry calls */
extern u32 CSCSoftlut[CSC_MAX_COEFF_COUNT];
extern u32 gammaSoftlut[GAMMA_CORRECT_MAX_COUNT];
extern u32 gammaSpriteSoftlut[GAMMA_SP_MAX_COUNT];

/* Prototypes */
int parse_clrmgr_input(uint *dest, char *src, int max, int read);
int do_intel_enable_CSC(struct drm_device *dev, void *data,
						struct drm_crtc *crtc);
bool intel_pipe_has_type(struct drm_crtc *crtc, int type);
void do_intel_disable_CSC(struct drm_device *dev,
						struct drm_crtc *crtc);
int intel_crtc_enable_gamma(struct drm_crtc *crtc, u32 identifier);
int intel_crtc_disable_gamma(struct drm_crtc *crtc, u32 identifier);
int intel_sprite_cb_adjust(drm_i915_private_t *dev_priv,
		struct ContBrightlut *cb_ptr);
int intel_sprite_hs_adjust(drm_i915_private_t *dev_priv,
		struct HueSaturationlut *hs_ptr);
void intel_save_clr_mgr_status(struct drm_device *dev);
bool intel_restore_clr_mgr_status(struct drm_device *dev);

#endif
