/*****************************************************************************
 *
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
 ******************************************************************************/
#include <linux/console.h>

#include "psb_drv.h"
#include "psb_fb.h"
#include "psb_intel_reg.h"
#include "displayclass_interface.h"
#include "mdfld_dsi_output.h"
#include "pwr_mgmt.h"

#define KEEP_UNUSED_CODE 0

#if KEEP_UNUSED_CODE
static int FindCurPipe(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (drm_helper_crtc_in_use(crtc)) {
			struct psb_intel_crtc *psb_intel_crtc =
			    to_psb_intel_crtc(crtc);
			return psb_intel_crtc->pipe;
		}
	}

	return 0;
}
#endif /* if KEEP_UNUSED_CODE */

static void user_mode_start(struct drm_psb_private *dev_priv)
{
	if (!dev_priv->um_start) {
		dev_priv->um_start = true;
		dev_priv->b_async_flip_enable = true;
		if (dev_priv->b_dsr_enable_config)
			dev_priv->b_dsr_enable = true;
	}
}

static void DCWriteReg(struct drm_device *dev, unsigned long ulOffset,
		       unsigned long ulValue)
{
	struct drm_psb_private *dev_priv;
	void *pvRegAddr;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	pvRegAddr = (void *)(dev_priv->vdc_reg + ulOffset);
	mb();
	iowrite32(ulValue, pvRegAddr);
}

void DCCBGetFramebuffer(struct drm_device *dev, struct psb_framebuffer **ppsb)
{
	struct drm_psb_private *dev_priv;
	struct psb_fbdev *fbdev;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	fbdev = dev_priv->fbdev;
	if (fbdev != NULL)
		*ppsb = fbdev->pfb;
}

int DCChangeFrameBuffer(struct drm_device *dev,
			struct psb_framebuffer *psbfb)
{
	return 0;
}

int DCCBEnableVSyncInterrupt(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv;
	int ret = 0;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	if (drm_vblank_get(dev, pipe)) {
		DRM_DEBUG("Couldn't enable vsync interrupt\n");
		ret = -EINVAL;
	}

	return ret;
}

void DCCBDisableVSyncInterrupt(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	drm_vblank_put(dev, pipe);
}

void DCCBInstallVSyncISR(struct drm_device *dev,
			 pfn_vsync_handler pVsyncHandler)
{
	struct drm_psb_private *dev_priv;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	dev_priv->psb_vsync_handler = pVsyncHandler;
}

void DCCBUninstallVSyncISR(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	dev_priv->psb_vsync_handler = NULL;
}

void DCCBFlipToSurface(struct drm_device *dev, unsigned long uiAddr,
				unsigned long uiFormat, unsigned long uiStride,
		       unsigned int pipeflag)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	u32 dspsurf = (dev_priv->cur_pipe == 0 ? DSPASURF : DSPBSURF);
	u32 dspcntr;
	u32 dspstride;
	u32 reg_offset;
	u32 val = 0;
	u32 power_island = 0;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx;

	DRM_DEBUG("%s %s %d, uiAddr = 0x%lx\n", __FILE__, __func__,
			  __LINE__, uiAddr);

	user_mode_start(dev_priv);

	if (pipeflag == 0) {
		dsi_config = dev_priv->dsi_configs[0];
		reg_offset = 0;
	} else if (pipeflag == 2) {
		dsi_config = dev_priv->dsi_configs[1];
		reg_offset = 0x2000;
	} else if (pipeflag == 1) {
		dsi_config = NULL;
		reg_offset = 0x1000;
	} else {
		DRM_ERROR("%s: invalid pipe %u\n", __func__, pipeflag);
		return;
	}

	/*update format*/
	val = (0x80000000 | uiFormat);

	if (dsi_config) {
		dsi_ctx = &dsi_config->dsi_hw_context;
		dsi_ctx->dspstride = uiStride;
		dsi_ctx->dspcntr = val;
		dsi_ctx->dspsurf = uiAddr;
	}

	dspsurf = DSPASURF + reg_offset;
	dspcntr = DSPACNTR + reg_offset;
	dspstride = DSPASTRIDE + reg_offset;

	DCWriteReg(dev, dspcntr, val);
	/*update stride*/
	DCWriteReg(dev, dspstride, uiStride);
	/*update surface address*/
	DCWriteReg(dev, dspsurf, uiAddr);
}

void DCCBFlipOverlay(struct drm_device *dev,
			struct intel_dc_overlay_ctx *ctx)
{
	struct drm_psb_private *dev_priv;
	u32 ovadd_reg = OV_OVADD;

	if (!dev || !ctx)
		return;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	user_mode_start(dev_priv);

	if (ctx->index == 1)
		ovadd_reg = OVC_OVADD;

	ctx->ovadd |= ctx->pipe;
	ctx->ovadd |= 1;

	PSB_WVDC32(ctx->ovadd, ovadd_reg);
}

void DCCBFlipSprite(struct drm_device *dev,
			struct intel_dc_sprite_ctx *ctx)
{
	struct drm_psb_private *dev_priv;
	u32 reg_offset = 0x3000;

	if (!dev || !ctx)
		return;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	user_mode_start(dev_priv);

	if (ctx->index != 0) {
		DRM_ERROR("%s: invalid index %d\n", __func__, ctx->index);
		return;
	}

	/* asign sprite to pipe */
	if (ctx->pipe)
		ctx->cntr |= (0x1 << 24);
	else
		ctx->cntr &= ~(0x3 << 24);

	if ((ctx->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(ctx->pos, DSPAPOS + reg_offset);

	if ((ctx->update_mask & SPRITE_UPDATE_SIZE)) {
		PSB_WVDC32(ctx->size, DSPASIZE + reg_offset);
		PSB_WVDC32(ctx->stride, DSPASTRIDE + reg_offset);
	}

	if ((ctx->update_mask & SPRITE_UPDATE_CONTROL))
		PSB_WVDC32(ctx->cntr, DSPACNTR + reg_offset);

	if ((ctx->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(ctx->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(ctx->surf, DSPASURF + reg_offset);
	}
}

void DCCBFlipPrimary(struct drm_device *dev,
			struct intel_dc_primary_ctx *ctx)
{
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *dsi_ctx;
	u32 reg_offset;
	int pipe;

	if (!dev || !ctx)
		return;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	user_mode_start(dev_priv);

	if (ctx->index == 0) {
		reg_offset = 0;
		dsi_config = dev_priv->dsi_configs[0];
		pipe = 0;
	} else if (ctx->index == 1) {
		reg_offset = 0x1000;
		pipe = 1;
	} else if (ctx->index == 2) {
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		pipe = 2;
	} else
		return;

	if (dsi_config) {
		dsi_ctx = &dsi_config->dsi_hw_context;
		dsi_ctx->dsppos = ctx->pos;
		dsi_ctx->dspsize = ctx->size;
		dsi_ctx->dspstride = ctx->stride;
		dsi_ctx->dspcntr = ctx->cntr;
		dsi_ctx->dsplinoff = ctx->linoff;
		dsi_ctx->dspsurf = ctx->surf;
	}

	if ((ctx->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(ctx->pos, DSPAPOS + reg_offset);

	if ((ctx->update_mask & SPRITE_UPDATE_SIZE)) {
		PSB_WVDC32(ctx->size, DSPASIZE + reg_offset);
		PSB_WVDC32(ctx->stride, DSPASTRIDE + reg_offset);
	}

	if ((ctx->update_mask & SPRITE_UPDATE_CONTROL))
		PSB_WVDC32(ctx->cntr, DSPACNTR + reg_offset);

	if ((ctx->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(ctx->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(ctx->surf, DSPASURF + reg_offset);
	}
}

static void _OverlayWaitFlip(struct drm_device *dev, u32 ovstat_reg)
{
	int retry;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	/**
	 * make sure overlay command buffer
	 * was copied before updating the system
	 * overlay command buffer.
	 */
	retry = 3000;
	while (--retry) {
		if (BIT31 & PSB_RVDC32(ovstat_reg))
			break;
		udelay(10);
	}

	if (!retry)
		DRM_ERROR("OVADD flip timeout!\n");
}
int DCCBOverlayEnable(struct drm_device *dev, u32 ctx,
			int index, int enabled)
{
	u32 ovadd_reg = OV_OVADD;
	u32 ovstat_reg = OV_DOVASTA;
	u32 power_islands = OSPM_DISPLAY_A;
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (index != 0 && index != 1) {
		DRM_ERROR("Invalid overlay index %d\n", index);
		return -EINVAL;
	}

	if (index) {
		ovadd_reg = OVC_OVADD;
		ovstat_reg = OVC_DOVCSTA;
		power_islands |= OSPM_DISPLAY_C;
	}

	if (power_island_get(power_islands)) {
		PSB_WVDC32(ctx, ovadd_reg);
		/*wait for overlay flipped*/
		_OverlayWaitFlip(dev, ovstat_reg);
		power_island_put(power_islands);
	}

	return 0;
}

int DCCBSpriteEnable(struct drm_device *dev, u32 ctx,
			int index, int enabled)
{
	u32 power_islands = (OSPM_DISPLAY_A | OSPM_DISPLAY_C);
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (index != 0) {
		DRM_ERROR("Invalid overlay index %d\n", index);
		return -EINVAL;
	}

	if (power_island_get(power_islands)) {
		PSB_WVDC32((PSB_RVDC32(DSPDCNTR) & ~BIT31), DSPDCNTR);
		PSB_WVDC32((PSB_RVDC32(DSPDSURF)), DSPDSURF);
		power_island_put(power_islands);
	}

	return 0;
}

void DCCBUpdateDbiPanel(struct drm_device *dev)
{
	mdfld_dbi_update_panel(dev, 0);
}

void DCCBUnblankDisplay(struct drm_device *dev)
{
	int res;
	struct psb_framebuffer *psb_fb = NULL;

	DCCBGetFramebuffer(dev, &psb_fb);

	if (!psb_fb)
		return;

	console_lock();
	res = fb_blank(psb_fb->fbdev, 0);
	console_unlock();
	if (res != 0) {
		DRM_ERROR("fb_blank failed (%d)", res);
	}
}

void DCCBFlipDSRCb(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (!dev_priv->um_start) {
		dev_priv->um_start = true;

		if (dev_priv->b_dsr_enable_config)
			dev_priv->b_dsr_enable = true;
	}

	if (dev_priv->b_dsr_enable && dev_priv->b_is_in_idle) {
		dev_priv->exit_idle(dev, MDFLD_DSR_2D_3D, NULL, true);
	}
}

u32 DCCBGetPipeCount(void)
{
	/* FIXME */
	return 3;
}

bool DCCBIsSuspended(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	bool ret = false;

	if (!dev_priv)
		return false;

	mutex_lock(&dev->mode_config.mutex);
	ret = dev_priv->early_suspended;
	mutex_unlock(&dev->mode_config.mutex);

	return ret;
}

int DCCBIsPipeActive(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	u32 pipeconf_reg;
	int active = 0;

	if (pipe == 0)
		pipeconf_reg = PIPEACONF;
	else if (pipe == 1)
		pipeconf_reg = PIPEBCONF;
	else {
		DRM_ERROR("%s: unsupported pipe %d\n", __func__, pipe);
		return 0;
	}

	/* get display a for register reading */
	if (power_island_get(OSPM_DISPLAY_A)) {
		active = (PSB_RVDC32(pipeconf_reg) & BIT31) ? 1 : 0 ;
		power_island_put(OSPM_DISPLAY_A);
	}

	return active;
}
