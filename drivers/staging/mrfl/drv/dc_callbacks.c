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

void DCCBEnableVSyncInterrupt(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	dev_priv->cur_pipe = FindCurPipe(dev);
	if (drm_vblank_get(dev, dev_priv->cur_pipe)) {
		DRM_ERROR("Couldn't enable vsync interrupt");
	}
}

void DCCBDisableVSyncInterrupt(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv;

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	dev_priv->cur_pipe = FindCurPipe(dev);
	drm_vblank_put(dev, dev_priv->cur_pipe);
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

/* Note: revisit to add supports for displaying L-frame only S3D display. */
/* Note: for upper layer driver or application to figure out which pipe to set s3d mode. */
void MrfldFlipToSurface(struct drm_device *dev,
			struct mrfld_s3d_flip *ps3d_flip, unsigned int pipeflag)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (!dev_priv->um_start) {
		dev_priv->um_start = true;
		if (dev_priv->b_dsr_enable_config)
			dev_priv->b_dsr_enable = true;
	}

	if (dev_priv->cur_s3d_state == (ps3d_flip->s3d_state & S3D_STATE_MASK)) {
		if (pipeflag & PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
			mrfld_s3d_flip_surf_addr(dev, 0, ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
		if (pipeflag & PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
			mrfld_s3d_flip_surf_addr(dev, 2, ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
		if (pipeflag & PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
			mrfld_s3d_flip_surf_addr(dev, 1, ps3d_flip);
#endif

	} else if ((dev_priv->cur_s3d_state & S3D_STATE_ENALBLED) !=
		   (ps3d_flip->s3d_state & S3D_STATE_ENALBLED)) {
		dev_priv->cur_s3d_state = ps3d_flip->s3d_state & S3D_STATE_MASK;

		if (ps3d_flip->s3d_state & S3D_STATE_ENALBLED) {
			/* Set s3d mode */
			switch (ps3d_flip->s3d_format) {
			case S3D_FRAME_PACKING:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_frame_packing(dev, 0,
								   ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_frame_packing(dev, 2,
								   ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_frame_packing(dev, 1,
								   ps3d_flip);
#endif
				break;
			case S3D_LINE_ALTERNATIVE:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_line_interleave(dev, 0,
								     ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_line_interleave(dev, 2,
								     ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_line_interleave(dev, 1,
								     ps3d_flip);
#endif
				break;
			case S3D_SIDE_BY_SIDE_FULL:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_full_side_by_side(dev, 0,
								       ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_full_side_by_side(dev, 2,
								       ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_full_side_by_side(dev, 1,
								       ps3d_flip);
#endif
				break;
			case S3D_TOP_AND_BOTTOM:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_top_and_bottom(dev, 0,
								    ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_top_and_bottom(dev, 2,
								    ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_top_and_bottom(dev, 1,
								    ps3d_flip);
#endif
				break;
			case S3D_SIDE_BY_SIDE_HALF:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_half_side_by_side(dev, 0,
								       ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_half_side_by_side(dev, 2,
								       ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_half_side_by_side(dev, 1,
								       ps3d_flip);
#endif
				break;
			case S3D_LINE_ALTERNATIVE_HALF:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_line_interleave_half(dev,
									  0,
									  ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_line_interleave_half(dev,
									  2,
									  ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_line_interleave_half(dev,
									  1,
									  ps3d_flip);
#endif
				break;
			case S3D_PIXEL_INTERLEAVING_HALF:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_pixel_interleaving_half
					    (dev, 0, ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_pixel_interleaving_half
					    (dev, 2, ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_pixel_interleaving_half
					    (dev, 1, ps3d_flip);
#endif
				break;
			case S3D_PIXEL_INTERLEAVING:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_to_pixel_interleaving_full
					    (dev, 0, ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_to_pixel_interleaving_full
					    (dev, 2, ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_to_pixel_interleaving_full
					    (dev, 1, ps3d_flip);
#endif
				break;
			default:
				DRM_ERROR("Invalid S3D format 0x(%x).",
					  ps3d_flip->s3d_format);

			}

		} else {
			/* Set back to 2d mode */
			switch (ps3d_flip->s3d_format) {
			case S3D_FRAME_PACKING:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_frame_packing(dev, 0,
								     ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_frame_packing(dev, 2,
								     ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_frame_packing(dev, 1,
								     ps3d_flip);
#endif
				break;
			case S3D_LINE_ALTERNATIVE:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_line_interleave(dev, 0,
								       ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_line_interleave(dev, 2,
								       ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_line_interleave(dev, 1,
								       ps3d_flip);
#endif
				break;
			case S3D_SIDE_BY_SIDE_FULL:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_full_side_by_side(dev, 0,
									 ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_full_side_by_side(dev, 2,
									 ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_full_side_by_side(dev, 1,
									 ps3d_flip);
#endif
				break;
			case S3D_TOP_AND_BOTTOM:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_top_and_bottom(dev, 0,
								      ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_top_and_bottom(dev, 2,
								      ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_top_and_bottom(dev, 1,
								      ps3d_flip);
#endif
				break;
			case S3D_SIDE_BY_SIDE_HALF:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_half_side_by_side(dev, 0,
									 ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_half_side_by_side(dev, 2,
									 ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_half_side_by_side(dev, 1,
									 ps3d_flip);
#endif
				break;
			case S3D_LINE_ALTERNATIVE_HALF:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_line_interleave_half(dev,
									    0,
									    ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_line_interleave_half(dev,
									    2,
									    ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_line_interleave_half(dev,
									    1,
									    ps3d_flip);
#endif
				break;
			case S3D_PIXEL_INTERLEAVING_HALF:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_pixel_interleaving_half
					    (dev, 0, ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_pixel_interleaving_half
					    (dev, 2, ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_pixel_interleaving_half
					    (dev, 1, ps3d_flip);
#endif
				break;
			case S3D_PIXEL_INTERLEAVING:
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A)
					mrfld_s3d_from_pixel_interleaving_full
					    (dev, 0, ps3d_flip);
#if defined(CONFIG_MID_DUAL_MIPI)
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C)
					mrfld_s3d_from_pixel_interleaving_full
					    (dev, 2, ps3d_flip);
#endif
#ifdef CONFIG_MID_HDMI
				if (pipeflag &
				    PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B)
					mrfld_s3d_from_pixel_interleaving_full
					    (dev, 1, ps3d_flip);
#endif
				break;
			default:
				DRM_ERROR("Invalid S3D format 0x(%x).",
					  ps3d_flip->s3d_format);

			}

		}

	}
}

void DCCBFlipToSurface(struct drm_device *dev, unsigned long uiAddr,
		       unsigned int pipeflag)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	int dspbase = (dev_priv->cur_pipe == 0 ? DSPABASE : DSPBBASE);
	int dspsurf = (dev_priv->cur_pipe == 0 ? DSPASURF : DSPBSURF);

	DRM_ERROR("%s %s %d\n", __FILE__, __func__, __LINE__);
#ifdef FIXME
	if (IS_MRFLD(dev))
		MrfldFlipToSurface(dev, uiAddr, pipeflag);
#endif
	DRM_ERROR("%s %s %d, uiAddr = 0x%x\n", __FILE__, __func__, __LINE__,
		  uiAddr);
	dspsurf = DSPASURF;
	DCWriteReg(dev, dspsurf, uiAddr);

#if FIXME
	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, false)) {
		if (IS_FLDS(dev)) {
			if (!dev_priv->um_start) {
				dev_priv->um_start = true;
				if (dev_priv->b_dsr_enable_config)
					dev_priv->b_dsr_enable = true;
			}

			if (pipeflag & PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A) {
				dspsurf = DSPASURF;
				DRM_ERROR("%s %s %d\n", __FILE__, __func__,
					  __LINE__);
				DCWriteReg(dev, dspsurf, uiAddr);
			}
#if defined(CONFIG_MID_DUAL_MIPI)
			if (pipeflag & PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C) {
				dspsurf = DSPCSURF;
				DRM_ERROR("%s %s %d\n", __FILE__, __func__,
					  __LINE__);
				DCWriteReg(dev, dspsurf, uiAddr);
			}
#endif
#ifdef CONFIG_MID_HDMI
			/* To avoid Plane B still fetches data from original frame
			 * buffer. */
			if (pipeflag & PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B) {
				dspsurf = DSPBSURF;
				DRM_ERROR("%s %s %d\n", __FILE__, __func__,
					  __LINE__);
				DCWriteReg(dev, dspsurf, uiAddr);
			}
#endif
		} else {
			DRM_ERROR("%s %s %d\n", __FILE__, __func__, __LINE__);
			DCWriteReg(dev, dspbase, uiAddr);
		}
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}
#endif
}

void DCCBUnblankDisplay(struct drm_device *dev)
{
	int res;
	struct psb_framebuffer *psb_fb;

	DCCBGetFramebuffer(dev, &psb_fb);

	console_trylock();
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
