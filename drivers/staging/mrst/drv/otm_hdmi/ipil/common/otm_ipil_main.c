/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2011 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:

  Intel Corporation
  2200 Mission College Blvd.
  Santa Clara, CA  95054

  BSD LICENSE

  Copyright(c) 2011 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <asm/io.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/delay.h>
#include "ipil_internal.h"

#include "otm_hdmi.h"
#include "otm_hdmi_types.h"

#include "ips_hdmi.h"
#include "ipil_hdmi.h"

static hdmi_device_t *hdmi_dev;

otm_hdmi_ret_t ipil_hdmi_set_hdmi_dev(hdmi_device_t *dev)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	if (!dev)
		rc = OTM_HDMI_ERR_NULL_ARG;
	else
		hdmi_dev = dev;
	return rc;
}

uint32_t hdmi_read32(uint32_t reg)
{
	if (hdmi_dev)
		return readl((const void *)(hdmi_dev->io_address + reg));

	return 0;
}

void hdmi_write32(uint32_t reg, uint32_t val)
{
	if (hdmi_dev)
		writel(val, (void *)(hdmi_dev->io_address + reg));
}

otm_hdmi_ret_t ipil_hdmi_decide_I2C_HW(hdmi_context_t *ctx)
{
	return ips_hdmi_decide_I2C_HW(ctx);
}

otm_hdmi_ret_t ipil_hdmi_general_5V_enable(hdmi_device_t *dev)
{
	return ips_hdmi_general_5V_enable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_5V_disable(hdmi_device_t *dev)
{
	return ips_hdmi_general_5V_disable(dev);
}

otm_hdmi_ret_t ipil_hdmi_set_program_clocks(hdmi_context_t *ctx,
					    unsigned int dclk)
{
	return ips_hdmi_set_program_clocks(ctx, dclk);
}

otm_hdmi_ret_t ipil_hdmi_audio_init(hdmi_context_t *ctx)
{
	return ips_hdmi_audio_init(ctx);
}

otm_hdmi_ret_t ipil_hdmi_audio_deinit(hdmi_context_t *ctx)
{
	return ips_hdmi_audio_deinit(ctx);
}

otm_hdmi_ret_t ipil_hdmi_general_unit_enable(hdmi_device_t *dev)
{
	return ips_hdmi_general_unit_enable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_unit_disable(hdmi_device_t *dev)
{
	return ips_hdmi_general_unit_disable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_hdcp_clock_enable(hdmi_device_t *dev)
{
	return ips_hdmi_general_hdcp_clock_enable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_hdcp_clock_disable(hdmi_device_t *dev)
{
	return ips_hdmi_general_hdcp_clock_disable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_audio_clock_enable(hdmi_device_t *dev)
{
	return ips_hdmi_general_audio_clock_enable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_audio_clock_disable(hdmi_device_t *dev)
{
	return ips_hdmi_general_audio_clock_disable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_pixel_clock_enable(hdmi_device_t *dev)
{
	return ips_hdmi_general_pixel_clock_enable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_pixel_clock_disable(hdmi_device_t *dev)
{
	return ips_hdmi_general_pixel_clock_disable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_tdms_clock_enable(hdmi_device_t *dev)
{
	return ips_hdmi_general_tdms_clock_enable(dev);
}

otm_hdmi_ret_t ipil_hdmi_general_tdms_clock_disable(hdmi_device_t *dev)
{
	return ips_hdmi_general_tdms_clock_disable(dev);
}

otm_hdmi_ret_t ipil_hdmi_i2c_disable(hdmi_device_t *dev)
{
	return ips_hdmi_i2c_disable(dev);
}

/*
 * Description: enable infoframes
 *
 * @dev:        hdmi_device_t
 * @type:	type of infoframe packet
 * @pkt:	infoframe packet data
 * @freq:	number of times packet needs to be sent
 *
 * Returns:     OTM_HDMI_ERR_NULL_ARG on NULL parameters
 *		OTM_HDMI_ERR_INVAL on invalid packet type
 *		OTM_HDMI_SUCCESS on success
 */
otm_hdmi_ret_t ipil_hdmi_enable_infoframe(hdmi_device_t *dev,
		unsigned int type, otm_hdmi_packet_t *pkt, unsigned int freq)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;

	if (!dev || !pkt)
		return OTM_HDMI_ERR_NULL_ARG;

	switch (type) {
	case HDMI_PACKET_AVI:
	case HDMI_PACKET_VS:
	case HDMI_PACKET_SPD:
		rc = ips_hdmi_enable_vid_infoframe(dev, type, pkt, freq);
		break;
	default:/* TODO: Revisit for Other Infoframes */
		rc = OTM_HDMI_ERR_INVAL;
		break;
	}

	return rc;
}

/*
 * Description: disable particular infoframe
 *
 * @dev:        hdmi_device_t
 * @type:	type of infoframe packet
 *
 * Returns:     OTM_HDMI_ERR_NULL_ARG on NULL parameters
 *		OTM_HDMI_ERR_INVAL on invalid packet type
 *		OTM_HDMI_SUCCESS on success
*/
otm_hdmi_ret_t ipil_hdmi_disable_infoframe(hdmi_device_t *dev,
					unsigned int type)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;

	if (!dev)
		return OTM_HDMI_ERR_NULL_ARG;

	switch (type) {
	case HDMI_PACKET_AVI:
	case HDMI_PACKET_VS:
	case HDMI_PACKET_SPD:
		rc = ips_hdmi_disable_vid_infoframe(dev, type);
		break;
	default:/* TODO: Revisit for Other Infoframes */
		rc = OTM_HDMI_ERR_INVAL;
		break;
	}

	return rc;
}

/*
 * Description: disable all infoframes
 *
 * @dev:        hdmi_device_t
 *
 * Returns:     OTM_HDMI_ERR_NULL_ARG on NULL parameters
 *		OTM_HDMI_SUCCESS on success
*/
otm_hdmi_ret_t ipil_hdmi_disable_all_infoframes(hdmi_device_t *dev)
{
	if (!dev)
		return OTM_HDMI_ERR_NULL_ARG;

	return ips_hdmi_disable_all_infoframes(dev);
}

static void pfit_landscape(int hsrc_sz, int vsrc_sz,
			int hdst_sz, int vdst_sz)
{
	int hmsb, vmsb, hratio, vratio;

	hdmi_write32(IPIL_PFIT_CONTROL,
			IPIL_PFIT_ENABLE |
			IPIL_PFIT_PIPE_SELECT_B |
			IPIL_PFIT_SCALING_PROGRAM);

	/* handling scaling up and down */
	if (hsrc_sz >= hdst_sz) {
		/* scaling down: msb = 1 */
		hratio = IPIL_PFIT_FRACTIONAL_VALUE * (hsrc_sz - hdst_sz) /
							(hdst_sz + 1);
		hmsb = 1;
	} else {
		/* scaling up: msb = 0 */
		hratio = IPIL_PFIT_FRACTIONAL_VALUE * (hsrc_sz + 1) /
							(hdst_sz + 1);
		hmsb = 0;
	}
	if (vsrc_sz >= vdst_sz) {
		/* scaling down: msb = 1 */
		vratio = IPIL_PFIT_FRACTIONAL_VALUE * (vsrc_sz - vdst_sz) /
							(vdst_sz + 1);
		vmsb = 1;
	} else {
		/* scaling up: msb = 0 */
		vratio = IPIL_PFIT_FRACTIONAL_VALUE * (vsrc_sz + 1) /
							(vdst_sz + 1);
		vmsb = 0;
	}

	pr_debug("\nhdisp = %d, vdisp = %d\n", hdst_sz, vdst_sz);
	pr_debug("\nhratio = %d, vratio = %d\n", hratio, vratio);
	hdmi_write32(IPIL_PFIT_PGM_RATIOS,
		vmsb << IPIL_PFIT_VERT_MSB_SHIFT |
		hmsb << IPIL_PFIT_HORIZ_MSB_SHIFT |
		vratio << IPIL_PFIT_VERT_SCALE_SHIFT |
		hratio << IPIL_PFIT_HORIZ_SCALE_SHIFT);
}

/*
 * Description: programs hdmi pipe src and size of the input.
 *
 * @dev:		hdmi_device_t
 * @scalingtype:	scaling type (FULL_SCREEN, CENTER, NO_SCALE etc.)
 * @mode:		mode requested
 * @adjusted_mode:	adjusted mode
 * @fb_width, fb_height:allocated frame buffer dimensions
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_INVAL on NULL input arguments
 */
otm_hdmi_ret_t ipil_hdmi_crtc_mode_set_program_dspregs(hdmi_device_t *dev,
					int scalingtype,
					ipil_timings_t *mode,
					ipil_timings_t *adjusted_mode,
					int fb_width, int fb_height)
{
	int sprite_pos_x = 0, sprite_pos_y = 0;
	int sprite_width = 0, sprite_height = 0;
	int src_image_hor = 0, src_image_vert = 0;

	/* NULL checks */
	if (dev == NULL || mode == NULL || adjusted_mode == NULL) {
		pr_debug("\ninvalid argument\n");
		return OTM_HDMI_ERR_INVAL;
	}

	/*
	 * Frame buffer size may beyond active region in case of
	 * panning mode.
	 */
	sprite_width = min_t(int, fb_width, adjusted_mode->width);
	sprite_height = min_t(int, fb_height, adjusted_mode->height);

	switch (scalingtype) {
	case OTM_HDMI_SCALE_NONE:
	case OTM_HDMI_SCALE_CENTER:
		/*
		 * This mode is used to support centering the screen
		 * by setting reg in DISPLAY controller
		 */
		src_image_hor = adjusted_mode->width;
		src_image_vert = adjusted_mode->height;
		sprite_pos_x = (src_image_hor - sprite_width) / 2;
		sprite_pos_y = (src_image_vert - sprite_height) / 2;

		hdmi_write32(IPIL_PFIT_CONTROL,
				hdmi_read32(IPIL_PFIT_CONTROL) &
						~IPIL_PFIT_ENABLE);
		break;

	case OTM_HDMI_SCALE_FULLSCREEN:
		src_image_hor = sprite_width;
		src_image_vert = sprite_height;
		sprite_pos_x = 0;
		sprite_pos_y = 0;

		if ((adjusted_mode->width > sprite_width) ||
			(adjusted_mode->height > sprite_height))
			hdmi_write32(IPIL_PFIT_CONTROL,
					IPIL_PFIT_ENABLE |
					IPIL_PFIT_PIPE_SELECT_B |
					IPIL_PFIT_SCALING_AUTO);
		break;

	case OTM_HDMI_SCALE_ASPECT:
		sprite_pos_x = 0;
		sprite_pos_y = 0;
		sprite_height = fb_height;
		sprite_width = fb_width;
		src_image_hor = fb_width;
		src_image_vert = fb_height;

		/* Use panel fitting when the display does not match
		 * with the framebuffer size */
		if ((adjusted_mode->width != fb_width) ||
		    (adjusted_mode->height != fb_height)) {
			if (fb_width > fb_height) {
				pr_debug("[hdmi]: Landscape mode...\n");
				/* Landscape mode: program ratios is
				 * used because 480p does not work with
				 * auto */
				if (adjusted_mode->height == 480)
					pfit_landscape(sprite_width,
						sprite_height,
						adjusted_mode->width,
						adjusted_mode->height);
				else
					hdmi_write32(IPIL_PFIT_CONTROL,
						IPIL_PFIT_ENABLE |
						IPIL_PFIT_PIPE_SELECT_B |
						IPIL_PFIT_SCALING_AUTO);
			} else {
				/* Portrait mode */
				pr_debug("[hdmi]: Portrait mode...\n");
				if (adjusted_mode->height == 768 &&
					adjusted_mode->width == 1024) {
					src_image_hor = adjusted_mode->width *
							fb_height /
							adjusted_mode->height;
					src_image_vert = fb_height;
					sprite_pos_x = (src_image_hor -
								fb_width) / 2;
					hdmi_write32(IPIL_PFIT_CONTROL,
						IPIL_PFIT_ENABLE |
						IPIL_PFIT_PIPE_SELECT_B |
						IPIL_PFIT_SCALING_AUTO);
				}
				else {
					/* Panel fitter HW has some limitations/bugs
					 * which forces us to tweak the way we use
					 * PILLARBOX mode.
					 */
#ifdef MFLD_HDMI_DV1
					src_image_hor = max_t(int, fb_width,
							      adjusted_mode->width) + 1;
					src_image_vert = max_t(int, fb_height,
							      adjusted_mode->height);
					sprite_pos_x = (src_image_hor - fb_width) / 2;
#endif
					hdmi_write32(IPIL_PFIT_CONTROL,
						     IPIL_PFIT_ENABLE |
						     IPIL_PFIT_PIPE_SELECT_B |
						     IPIL_PFIT_SCALING_PILLARBOX);
				}
			}
		} else {
			/* TODO: setting pfit_control to 0 is resulting in
			 * tearing. Hence disabling this code. Need to
			 * revisit later.
			 */
			/* hdmi_write32(IPIL_PFIT_CONTROL, 0); */
			hdmi_write32(IPIL_PFIT_CONTROL,
					IPIL_PFIT_ENABLE |
					IPIL_PFIT_PIPE_SELECT_B |
					IPIL_PFIT_SCALING_AUTO);
		}

		break;

	default:
		/* Android will not change mode, however ,we have tools
		to change HDMI timing so there is some cases frame
		buffer no change ,but timing changed mode setting, in
		this case. mode information for source size is not
		right, so here use fb information for source/sprite
		size*/

		/* The defined sprite rectangle must always be
		completely contained within the displayable area of the
		screen image (frame buffer). */
		sprite_pos_x = 0;
		sprite_pos_y = 0;
		sprite_height = fb_height;
		sprite_width = fb_width;
		src_image_hor = fb_width;
		src_image_vert = fb_height;
		if ((adjusted_mode->width != fb_width) ||
				(adjusted_mode->height != fb_height))
			hdmi_write32(IPIL_PFIT_CONTROL,
					IPIL_PFIT_ENABLE |
					IPIL_PFIT_PIPE_SELECT_B);

		break;
	}

	pr_debug("Sprite position: (%d, %d)\n", sprite_pos_x,
			sprite_pos_y);
	pr_debug("Sprite size: %d x %d\n", sprite_width,
			sprite_height);
	pr_debug("Pipe source image size: %d x %d\n",
			src_image_hor, src_image_vert);

	hdmi_write32(IPIL_DSPBPOS, (sprite_pos_y << 16) | sprite_pos_x);
	hdmi_write32(IPIL_DSPBSIZE, ((sprite_height - 1) << 16) |
				 (sprite_width - 1));
	hdmi_write32(IPIL_PIPEBSRC, ((src_image_hor - 1) << 16) |
				(src_image_vert - 1));

	return OTM_HDMI_SUCCESS;
}

/*
 * Description: this is pre-modeset configuration. This can be
 *		resetting HDMI unit, disabling/enabling dpll etc
 *		on the need basis.
 *
 * @dev:	hdmi_device_t
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_INVAL on NULL input arguments
 */
otm_hdmi_ret_t ipil_hdmi_crtc_mode_set_prepare(hdmi_device_t *dev)
{
	/* NULL checks */
	if (dev == NULL) {
		pr_debug("\ninvalid argument\n");
		return OTM_HDMI_ERR_INVAL;
	}

	/* Nothing needed as of now for medfield */

	return OTM_HDMI_SUCCESS;
}

/*
 * Description: programs all the timing registers based on scaling type.
 *
 * @dev:		hdmi_device_t
 * @scalingtype:	scaling type (FULL_SCREEN, CENTER, NO_SCALE etc.)
 * @mode:		mode requested
 * @adjusted_mode:	adjusted mode
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_INVAL on NULL input arguments
 */
otm_hdmi_ret_t ipil_hdmi_crtc_mode_set_program_timings(hdmi_device_t *dev,
					int scalingtype,
					otm_hdmi_timing_t *mode,
					otm_hdmi_timing_t *adjusted_mode)
{
	/* NULL checks */
	if (dev == NULL || mode == NULL || adjusted_mode == NULL) {
		pr_debug("\ninvalid argument\n");
		return OTM_HDMI_ERR_INVAL;
	}

	if (scalingtype == OTM_HDMI_SCALE_NONE) {
		/* Moorestown doesn't have register support for centering so we
		 * need to  mess with the h/vblank and h/vsync start and ends
		 * to get centering
		 */
		int offsetX = 0, offsetY = 0;

		offsetX = (adjusted_mode->width - mode->width) / 2;
		offsetY = (adjusted_mode->height - mode->height) / 2;

		hdmi_write32(IPIL_HTOTAL_B, (mode->width - 1) |
				((adjusted_mode->htotal - 1) << 16));

		hdmi_write32(IPIL_VTOTAL_B, (mode->height - 1) |
				((adjusted_mode->vtotal - 1) << 16));

		hdmi_write32(IPIL_HBLANK_B,
			(adjusted_mode->hblank_start - offsetX - 1) |
			((adjusted_mode->hblank_end - offsetX - 1) << 16));

		hdmi_write32(IPIL_HSYNC_B,
			(adjusted_mode->hsync_start - offsetX - 1) |
			((adjusted_mode->hsync_end - offsetX - 1) << 16));

		hdmi_write32(IPIL_VBLANK_B,
			(adjusted_mode->vblank_start - offsetY - 1) |
			((adjusted_mode->vblank_end - offsetY - 1) << 16));

		hdmi_write32(IPIL_VSYNC_B,
			(adjusted_mode->vsync_start - offsetY - 1) |
			((adjusted_mode->vsync_end - offsetY - 1) << 16));
	} else {
		hdmi_write32(IPIL_HTOTAL_B,
				(adjusted_mode->width - 1) |
				((adjusted_mode->htotal - 1) << 16));

		hdmi_write32(IPIL_VTOTAL_B,
				(adjusted_mode->height - 1) |
				((adjusted_mode->vtotal - 1) << 16));

		hdmi_write32(IPIL_HBLANK_B,
				(adjusted_mode->hblank_start - 1) |
				((adjusted_mode->hblank_end - 1) << 16));

		hdmi_write32(IPIL_HSYNC_B,
				(adjusted_mode->hsync_start - 1) |
				((adjusted_mode->hsync_end - 1) << 16));

		hdmi_write32(IPIL_VBLANK_B,
				(adjusted_mode->vblank_start - 1) |
				((adjusted_mode->vblank_end - 1) << 16));

		hdmi_write32(IPIL_VSYNC_B,
				(adjusted_mode->vsync_start - 1) |
				((adjusted_mode->vsync_end - 1) << 16));
	}

	return OTM_HDMI_SUCCESS;
}

/*
 * Description: programs dpll clocks, enables dpll and waits
 *		till it locks with DSI PLL
 *
 * @dev:	hdmi_device_t
 * @dclk:	refresh rate dot clock in kHz of current mode
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_INVAL on NULL input arguments
 */
otm_hdmi_ret_t	ipil_hdmi_crtc_mode_set_program_dpll(hdmi_device_t *dev,
							unsigned long dclk)
{
	otm_hdmi_ret_t rc = OTM_HDMI_SUCCESS;
	u32 dpll_adj, fp;
	u32 dpll;
	int timeout = 0;

	/* NULL checks */
	if (dev == NULL) {
		pr_debug("\ninvalid argument\n");
		return OTM_HDMI_ERR_INVAL;
	}

	/* get the adjusted clock value */
	rc = ips_hdmi_get_adjusted_clk(dclk, &dpll_adj, &fp, &dev->clock_khz);
	if (rc != OTM_HDMI_SUCCESS) {
		pr_debug("\nfailed to calculate adjusted clock\n");
		return rc;
	}

	dpll = hdmi_read32(IPIL_DPLL_B);
	if (dpll & IPIL_DPLL_VCO_ENABLE) {
		dpll &= ~IPIL_DPLL_VCO_ENABLE;
		hdmi_write32(IPIL_DPLL_B, dpll);
		hdmi_read32(IPIL_DPLL_B);

		/* reset M1, N1 & P1 */
		hdmi_write32(IPIL_DPLL_DIV0, 0);
		dpll &= ~IPIL_P1_MASK;
		hdmi_write32(IPIL_DPLL_B, dpll);
	}

	/*
	 * When ungating power of DPLL, needs to wait 0.5us
	 * before enable the VCO
	 */
	if (dpll & IPIL_PWR_GATE_EN) {
		dpll &= ~IPIL_PWR_GATE_EN;
		hdmi_write32(IPIL_DPLL_B, dpll);
		udelay(1);
	}

	dpll = dpll_adj;
	hdmi_write32(IPIL_DPLL_DIV0, fp);
	hdmi_write32(IPIL_DPLL_B, dpll);
	udelay(1);

	dpll |= IPIL_DPLL_VCO_ENABLE;
	hdmi_write32(IPIL_DPLL_B, dpll);
	hdmi_read32(IPIL_DPLL_B);

	/* wait for DSI PLL to lock */
	while ((timeout < 20000) && !(hdmi_read32(IPIL_PIPEBCONF) &
					IPIL_PIPECONF_PLL_LOCK)) {
		udelay(150);
		timeout++;
	}

	return OTM_HDMI_SUCCESS;
}

/*
 * Description: configures the display plane register and enables
 *		pipeconf.
 *
 * @dev: hdmi_device_t
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_INVAL on NULL input arguments
 */
otm_hdmi_ret_t ipil_hdmi_crtc_mode_set_program_pipeconf(hdmi_device_t *dev)
{
	u32 dspcntr;
	u32 pipeconf;

	/* NULL checks */
	if (dev == NULL) {
		pr_debug("\ninvalid argument\n");
		return OTM_HDMI_ERR_INVAL;
	}

	/* Set up the display plane register */
	dspcntr = hdmi_read32(IPIL_DSPBCNTR);
	dspcntr |= 1 << IPIL_DSP_PLANE_PIPE_POS;
	dspcntr |= IPIL_DSP_PLANE_ENABLE;

	/* setup pipeconf */
	pipeconf = IPIL_PIPEACONF_ENABLE;


	hdmi_write32(IPIL_PIPEBCONF, pipeconf);
	hdmi_read32(IPIL_PIPEBCONF);

	hdmi_write32(IPIL_DSPBCNTR, dspcntr);

	return OTM_HDMI_SUCCESS;
}

/*
 * Description: encoder mode set function for hdmi. enables phy.
 *		set correct polarity for the current mode, sets
 *		correct panel fitting.
 *
 *
 * @dev:		hdmi_device_t
 * @mode:		mode requested
 * @adjusted_mode:	adjusted mode
 * @is_monitor_hdmi:	is monitor type is hdmi or not
 *
 * Returns:	OTM_HDMI_SUCCESS on success
 *		OTM_HDMI_ERR_INVAL on NULL input arguments
 */
otm_hdmi_ret_t ipil_hdmi_enc_mode_set(hdmi_device_t *dev,
					otm_hdmi_timing_t *mode,
					otm_hdmi_timing_t *adjusted_mode,
					bool is_monitor_hdmi)
{
	u32 hdmib, hdmi_phy_misc;
	bool phsync;
	bool pvsync;

	/* NULL checks */
	if (dev == NULL || mode == NULL || adjusted_mode == NULL) {
		pr_debug("\ninvalid argument\n");
		return OTM_HDMI_ERR_INVAL;
	}

	if (is_monitor_hdmi) {
		hdmib = hdmi_read32(IPIL_HDMIB_CONTROL)
						| IPIL_HDMIB_PIPE_B_SELECT
						| IPIL_HDMIB_NULL_PACKET
						| IPIL_HDMIB_AUDIO_ENABLE;
	} else {
		hdmib = hdmi_read32(IPIL_HDMIB_CONTROL)
						| IPIL_HDMIB_PIPE_B_SELECT;
		hdmib &= ~IPIL_HDMIB_NULL_PACKET;
		hdmib &= ~IPIL_HDMIB_AUDIO_ENABLE;
	}

	/* disable HDMI port since the DPMS will take care of the enabling */
	hdmib &= ~IPIL_HDMIB_PORT_EN;

	/* set output polarity */
	phsync = adjusted_mode->mode_info_flags & IPIL_TIMING_FLAG_PHSYNC;
	pvsync = adjusted_mode->mode_info_flags & IPIL_TIMING_FLAG_PVSYNC;
	pr_debug("enc_mode_set %dx%d (%c,%c)\n", adjusted_mode->width,
						adjusted_mode->height,
						phsync ? '+' : '-',
						pvsync ? '+' : '-');
	if (phsync)
		hdmib = SET_BITS(hdmib, IPIL_HSYNC_POLARITY_MASK);
	else
		hdmib = CLEAR_BITS(hdmib, IPIL_HSYNC_POLARITY_MASK);
	if (pvsync)
		hdmib = SET_BITS(hdmib, IPIL_VSYNC_POLARITY_MASK);
	else
		hdmib = CLEAR_BITS(hdmib, IPIL_VSYNC_POLARITY_MASK);

	hdmi_phy_misc = hdmi_read32(IPIL_HDMIPHYMISCCTL) &
					~IPIL_HDMI_PHY_POWER_DOWN;

	hdmi_write32(IPIL_HDMIPHYMISCCTL, hdmi_phy_misc);
	hdmi_write32(IPIL_HDMIB_CONTROL, hdmib);
	hdmi_read32(IPIL_HDMIB_CONTROL);

	return OTM_HDMI_SUCCESS;
}

/*
 * Description: save HDMI display registers
 *
 * @dev:		hdmi_device_t
 *
 * Returns: none
 */
void ipil_hdmi_save_display_registers(hdmi_device_t *dev)
{
	if (NULL != dev)
		ips_hdmi_save_display_registers(dev);
}

/*
 * Description: save HDMI data island packets
 *
 * @dev:		hdmi_device_t
 *
 * Returns: none
 */
void ipil_hdmi_save_data_island(hdmi_device_t *dev)
{
	if (NULL != dev)
		ips_hdmi_save_data_island(dev);
}

/*
 * Description: destroys any saved HDMI data
 *
 * @dev:	hdmi_device_t
 *
 * Returns: none
 */
void ipil_hdmi_destroy_saved_data(hdmi_device_t *dev)
{
	if (NULL != dev)
		ips_hdmi_destroy_saved_data(dev);
}

/*
 * Description: disable HDMI display
 *
 * @dev:		hdmi_device_t
 *
 * Returns: none
 */
void ipil_disable_hdmi(hdmi_device_t *dev)
{
	if (NULL != dev)
		ips_disable_hdmi(dev);
}

/*
 * Description: restore HDMI display registers and enable display
 *
 * @dev:		hdmi_device_t
 *
 * Returns: none
 */
void ipil_hdmi_restore_and_enable_display(hdmi_device_t *dev)
{
	if (NULL != dev)
		ips_hdmi_restore_and_enable_display(dev);
}

/*
 * Description: restore HDMI data island packets
 *
 * @dev:		hdmi_device_t
 *
 * Returns: none
 */
void ipil_hdmi_restore_data_island(hdmi_device_t *dev)
{
	if (NULL != dev)
		ips_hdmi_restore_data_island(dev);
}
