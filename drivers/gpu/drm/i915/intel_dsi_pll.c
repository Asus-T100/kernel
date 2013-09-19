/*
 * Copyright © 2013-2013 Intel Corporation
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
 *	Shobhit Kumar <shobhit.kumar@intel.com>
 *	Yogesh Mohan Marimuthu <yogesh.mohan.marimuthu@intel.com>
 */

#include <linux/kernel.h>
#include "intel_drv.h"
#include "i915_drv.h"
#include "intel_dsi.h"

struct dsi_clock_table {
	u32 freq;
	u8 m;
	u8 p;
};

struct dsi_mnp {
	u32 dsi_pll_ctrl;
	u32 dsi_pll_div;
};

u32 lfsr_converts[] = {
		426, 469, 234, 373, 442, 221, 110, 311, 411,	/* 62 - 70 */
	461, 486, 243, 377, 188, 350, 175, 343, 427, 213,	/* 71 - 80 */
	106, 53, 282, 397, 354, 227, 113, 56, 284, 142,		/* 81 - 90 */
	71, 35							/* 91 - 92 */
};

struct dsi_clock_table dsi_clk_tbl[] = {
		{300, 72, 6}, {313, 75, 6}, {323, 78, 6}, {333, 80, 6},
		{343, 82, 6}, {353, 85, 6}, {363, 87, 6}, {373, 90, 6},
		{383, 92, 6}, {390, 78, 5}, {393, 79, 5}, {400, 80, 5},
		{401, 80, 5}, {402, 80, 5}, {403, 81, 5}, {404, 81, 5},
		{405, 81, 5}, {406, 81, 5}, {407, 81, 5}, {408, 82, 5},
		{409, 82, 5}, {410, 82, 5}, {411, 82, 5}, {412, 82, 5},
		{413, 83, 5}, {414, 83, 5}, {415, 83, 5}, {416, 83, 5},
		{417, 83, 5}, {418, 84, 5}, {419, 84, 5}, {420, 84, 5},
		{430, 86, 5}, {440, 88, 5}, {450, 90, 5}, {460, 92, 5},
		{470, 75, 4}, {480, 77, 4}, {490, 78, 4}, {500, 80, 4},
		{510, 82, 4}, {520, 83, 4}, {530, 85, 4}, {540, 86, 4},
		{550, 88, 4}, {560, 90, 4}, {570, 91, 4}, {580, 70, 3},
		{590, 71, 3}, {600, 72, 3}, {610, 73, 3}, {620, 74, 3},
		{630, 76, 3}, {640, 77, 3}, {650, 78, 3}, {660, 79, 3},
		{670, 80, 3}, {680, 82, 3}, {690, 83, 3}, {700, 84, 3},
		{710, 85, 3}, {720, 86, 3}, {730, 88, 3}, {740, 89, 3},
		{750, 90, 3}, {760, 91, 3}, {770, 92, 3}, {780, 62, 2},
		{790, 63, 2}, {800, 64, 2}, {880, 70, 2}, {900, 72, 2},
		{1000, 80, 2},		/* dsi clock frequency in Mhz*/
};

/* Get DSI clock from pixel clock */
int dsi_clk_from_pclk(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *dsi_clk) {
	u32 dsi_bit_clock_hz;
	u32 pkt_pixel_size;		/* in bits */
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (intel_dsi->dsi_packet_format == DSI_24BPP_PACKED)
		pkt_pixel_size = 24;
	else if (intel_dsi->dsi_packet_format == DSI_18BPP_LOOSELY_PACKED)
		pkt_pixel_size = 24;
	else if (intel_dsi->dsi_packet_format == DSI_18BPP_PACKED)
		pkt_pixel_size = 18;
	else if (intel_dsi->dsi_packet_format == DSI_16BPP_PACKED)
		pkt_pixel_size = 16;
	else
		return -ECHRNG;

	/* For Acer AUO B080XAT panel, use a fixed DSI data rate of 513 Mbps */
	if (dev_priv->mipi.panel_id == MIPI_DSI_AUO_B080XAT_PANEL_ID) {
		*dsi_clk = 513;
		return 0;
	}

	/* DSI data rate = pixel clock * bits per pixel / lane count
	   pixel clock is converted from KHz to Hz */
	dsi_bit_clock_hz = (((mode->clock * 1000) * pkt_pixel_size) \
				/ intel_dsi->dev.lane_count);

	/* return DSI data rate as Mbps */
	*dsi_clk = dsi_bit_clock_hz / (1024 * 1024);
	return 0;
}

int dsi_rr_formula(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *dsi_clk) {
	u32 hactive, vactive, hfp, hsync, hbp, vfp, vsync, vbp;
	u32 pkt_pixel_size;		/* in bits */
	u32 hsync_bytes;
	u32 hbp_bytes;
	u32 hactive_bytes;
	u32 hfp_bytes;
	u32 bytes_per_line;
	u32 bytes_per_frame;
	u32 num_frames;
	u32 bytes_per_x_frames;
	u32 bytes_per_x_frames_x_lanes;
	u32 dsi_byte_clock_hz;
	u32 dsi_bit_clock_hz;

	if (intel_dsi->dsi_packet_format == DSI_24BPP_PACKED)
		pkt_pixel_size = 24;
	else if (intel_dsi->dsi_packet_format == DSI_18BPP_LOOSELY_PACKED)
		pkt_pixel_size = 24;
	else if (intel_dsi->dsi_packet_format == DSI_18BPP_PACKED)
		pkt_pixel_size = 18;
	else if (intel_dsi->dsi_packet_format == DSI_16BPP_PACKED)
		pkt_pixel_size = 16;
	else
		return -ECHRNG;

	hactive = mode->hdisplay;
	vactive = mode->vdisplay;
	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;

	hsync_bytes = ((hsync * pkt_pixel_size) / 8) +
			(((hsync * pkt_pixel_size) % 8) && 1);
	hbp_bytes = ((hbp * pkt_pixel_size) / 8) +
			(((hbp * pkt_pixel_size) % 8) && 1);
	hactive_bytes = ((hactive * pkt_pixel_size) / 8) +
			(((hactive * pkt_pixel_size) % 8) && 1);
	hfp_bytes = ((hfp * pkt_pixel_size) / 8) +
			(((hfp * pkt_pixel_size) % 8) && 1);

	DRM_DEBUG_KMS("### hactive = %0d\n", hactive);
	DRM_DEBUG_KMS("### hfp = %0d\n", hfp);
	DRM_DEBUG_KMS("### hsync = %0d\n", hsync);
	DRM_DEBUG_KMS("### hbp = %0d\n", hbp);
	DRM_DEBUG_KMS("### vfp = %0d\n", vfp);
	DRM_DEBUG_KMS("### vsync = %0d\n", vsync);
	DRM_DEBUG_KMS("### vbp = %0d\n", vbp);
	DRM_DEBUG_KMS("### hsync_bytes = %0d\n", hsync_bytes);
	DRM_DEBUG_KMS("### hbp_bytes = %0d\n", hbp_bytes);
	DRM_DEBUG_KMS("### hactive_bytes = %0d\n", hactive_bytes);
	DRM_DEBUG_KMS("### hfp_bytes = %0d\n", hfp_bytes);

	bytes_per_line = DSI_HSS_PACKET_SIZE + hsync_bytes + \
		DSI_HSA_PACKET_EXTRA_SIZE + DSI_HSE_PACKET_SIZE + \
		hbp_bytes + DSI_HBP_PACKET_EXTRA_SIZE + \
		hactive_bytes + DSI_HACTIVE_PACKET_EXTRA_SIZE + \
		hfp_bytes + DSI_HFP_PACKET_EXTRA_SIZE;

	if ((intel_dsi->dev.eotp_pkt == 1) && \
			(intel_dsi->dev.operation_mode == DSI_VIDEO_MODE) && \
			(intel_dsi->dev.video_mode_type == DSI_VIDEO_BURST)) {
		bytes_per_line = bytes_per_line + DSI_EOTP_PACKET_SIZE;
		/* Need to accurately calculate LP to HS transition
		 * timeout and add it to bytes_per_line*/
	}

	bytes_per_frame = (vsync * bytes_per_line) + (vbp * bytes_per_line) + \
			(vactive * bytes_per_line) + (vfp * bytes_per_line);

	if ((intel_dsi->dev.eotp_pkt == 1) &&
			(intel_dsi->dev.operation_mode == DSI_VIDEO_MODE)) {
		if ((intel_dsi->dev.video_mode_type ==
				DSI_VIDEO_NBURST_SPULSE) ||
				(intel_dsi->dev.video_mode_type ==
				DSI_VIDEO_NBURST_SEVENT))
			bytes_per_frame = bytes_per_frame +
					DSI_EOTP_PACKET_SIZE;
			/* Need to accurately calculate LP to HS
			 * transition timeout and add it to bytes_per_frame*/
	}

	num_frames = (mode->clock * 1000) / (mode->htotal * mode->vtotal);
	bytes_per_x_frames = num_frames * bytes_per_frame;
	bytes_per_x_frames_x_lanes = bytes_per_x_frames /
			intel_dsi->dev.lane_count;

	/* the dsi clock is divided by 2 in the hardware to get dsi ddr clock */
	dsi_byte_clock_hz = bytes_per_x_frames_x_lanes;
	dsi_bit_clock_hz = dsi_byte_clock_hz * 8;
	*dsi_clk = dsi_bit_clock_hz / (1000 * 1000);

	if ((intel_dsi->dev.eotp_pkt == 1) &&
			(intel_dsi->dev.operation_mode == DSI_VIDEO_MODE) &&
			(intel_dsi->dev.video_mode_type == DSI_VIDEO_BURST)) {
		*dsi_clk = *dsi_clk * 2;
	}

	DRM_DEBUG_KMS("### bytes_per_line = %0d\n", bytes_per_line);
	DRM_DEBUG_KMS("### bytes_per_frame = %0d\n", bytes_per_frame);
	DRM_DEBUG_KMS("### num_frames = %0d\n", num_frames);
	DRM_DEBUG_KMS("### bytes_per_x_frames = %0d\n", bytes_per_x_frames);
	DRM_DEBUG_KMS("### bytes_per_x_frames_x_lanes = %0d\n",
			bytes_per_x_frames_x_lanes);
	DRM_DEBUG_KMS("### dsi_byte_clock_hz = %0d\n", dsi_byte_clock_hz);
	DRM_DEBUG_KMS("### dsi_bit_clock_hz = %0d\n", dsi_bit_clock_hz);
	DRM_DEBUG_KMS("### dsi_clk = %0d\n", *dsi_clk);
	return 0;
}

int dsi_15percent_formula(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *dsi_clk)
{
	u32 bpp;
	u32 dsi_pixel_clk;

	if (intel_dsi->dsi_packet_format == dsi_24Bpp_packed)
		bpp = 24;
	else if (intel_dsi->dsi_packet_format == dsi_18Bpp_loosely_packed)
		bpp = 24;
	else
		bpp = 18;

	dsi_pixel_clk = (mode->clock * bpp) /
			(intel_dsi->dev.lane_count * 1000);
	*dsi_clk = /*((dsi_pixel_clk * 15) / 100) + */dsi_pixel_clk;

	return 0;
}

int get_dsi_clk(struct intel_dsi *intel_dsi, struct drm_display_mode *mode, \
		u32 *dsi_clk)
{

	return dsi_clk_from_pclk(intel_dsi, mode, dsi_clk);
	/*return dsi_15percent_formula(intel_dsi, mode, dsi_clk);*/
}

int mnp_from_clk_table(u32 dsi_clk, struct dsi_mnp *dsi_mnp)
{
	unsigned int i;
	u8 m = 0;
	u8 n = 0;
	u8 p = 0;
	u32 m_seed;

	if (dsi_clk < 300 || dsi_clk > 1000)
		return -ECHRNG;

	for (i = 0; i < sizeof(dsi_clk_tbl)/sizeof(struct dsi_clock_table);
			i++) {
		if (dsi_clk_tbl[i].freq > dsi_clk)
			break;
	}

	if (i == sizeof(dsi_clk_tbl)/sizeof(struct dsi_clock_table))
		return -ECHRNG;

	m = dsi_clk_tbl[i].m;
	p = dsi_clk_tbl[i].p;

	m_seed = lfsr_converts[m - 62];
	n = 1;
	dsi_mnp->dsi_pll_ctrl = (1 << (17 + p - 2)) | (1 << 8);
	dsi_mnp->dsi_pll_div = ((n - 1) << 16) | m_seed;

	return 0;
}

int dsi_calc_mnp(u32 dsi_clk, struct dsi_mnp *dsi_mnp)
{
	u32 m, n, p;
	u32 ref_clk;
	u32 error;
	u32 tmp_error;
	u32 target_dsi_clk;
	u32 calc_dsi_clk;
	u32 calc_m;
	u32 calc_p;
	u32 m_seed;

	if (dsi_clk < 300 || dsi_clk > 1150) {
		DRM_ERROR("DSI CLK Out of Range\n");
		return -ECHRNG;
	}

	ref_clk = 25000;
	target_dsi_clk = dsi_clk * 1000;
	error = 0xFFFFFFFF;
	tmp_error = 0xFFFFFFFF;
	calc_m = 0;
	calc_p = 0;

	for (m = 62; m <= 92; m++) {
		for (p = 2; p <= 6; p++) {
			/* Find the optimal m and p divisors
			with minimal error +/- the required clock */
			calc_dsi_clk = (m * ref_clk) / p;
			if (calc_dsi_clk == target_dsi_clk) {
				calc_m = m;
				calc_p = p;
				error = 0;
				break;
			} else if (calc_dsi_clk > target_dsi_clk)
				tmp_error = calc_dsi_clk - target_dsi_clk;
			else
				tmp_error = target_dsi_clk - calc_dsi_clk;

			if (tmp_error < error) {
					error = tmp_error;
					calc_m = m;
					calc_p = p;
			}
		}

		if (error == 0)
			break;
	}

	m_seed = lfsr_converts[calc_m - 62];
	n = 1;
	dsi_mnp->dsi_pll_ctrl = (1 << (17 + calc_p - 2)) | (1 << 8);
	dsi_mnp->dsi_pll_div = ((n - 1) << 16) | m_seed;

	return 0;
}

int intel_configure_dsi_pll(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode)
{
	struct drm_i915_private *dev_priv =
			intel_dsi->base.base.dev->dev_private;
	int ret;
	struct dsi_mnp dsi_mnp;
	u32 dsi_clk;


	get_dsi_clk(intel_dsi, mode, &dsi_clk);
	ret = dsi_calc_mnp(dsi_clk, &dsi_mnp);
	/*ret = mnp_from_clk_table(dsi_clk, &dsi_mnp);*/

	if (ret != 0)
		return ret;

	intel_cck_write32(dev_priv, 0x48, 0x00000000);
	intel_cck_write32(dev_priv, 0x4C, dsi_mnp.dsi_pll_div);
	intel_cck_write32(dev_priv, 0x48, dsi_mnp.dsi_pll_ctrl);

	return 0;
}

int intel_enable_dsi_pll(struct intel_dsi *intel_dsi)
{
	struct drm_i915_private *dev_priv =
			intel_dsi->base.base.dev->dev_private;

	/* bandgap reset */
	intel_flisdsi_write32(dev_priv, 0x08, 0x0001);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0005);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0025);
	udelay(150);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0000);
	intel_flisdsi_write32(dev_priv, 0x08, 0x0000);

	/* enable DPLL ref clock */
	I915_WRITE_BITS(_DPLL_A, DPLL_REFA_CLK_ENABLE_VLV,
						DPLL_REFA_CLK_ENABLE_VLV);

	udelay(1000);	/*wait 0.5us after ungating before enabling again */
	intel_cck_write32_bits(dev_priv, 0x48, 1 << 31,  1 << 31);

	if (wait_for(((I915_READ(PIPECONF(intel_dsi->pipe)) & (1 << 29)) ==
			(1 << 29)), 20)) {
		DRM_ERROR("DSI PLL lock failed\n");
		return -EIO;
	}

	DRM_DEBUG_KMS("DSI PLL locked\n");
	return 0;
}

int intel_disable_dsi_pll(struct intel_dsi *intel_dsi)
{
	struct drm_i915_private *dev_priv =
			intel_dsi->base.base.dev->dev_private;

	intel_cck_write32_bits(dev_priv, 0x48, 0x00000000, 0x80000000);
	udelay(500);

	if ((I915_READ(PIPECONF(PIPE_A)) & PIPECONF_ENABLE == 0) &&
		(I915_READ(PIPECONF(PIPE_B)) & PIPECONF_ENABLE == 0)
		)
		I915_WRITE_BITS(_DPLL_A, 0x00000000, DPLL_REFA_CLK_ENABLE_VLV);

	return 0;
}
