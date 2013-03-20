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
 *	Jesse Barnes <jesse.barnes@intel.com>
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

u32 lfsr_converts[] = {
		426, 469, 234, 373, 442, 221, 110, 311, 411,	/* 62 - 70 */
	461, 486, 243, 377, 188, 350, 175, 343, 427, 213,	/* 71 - 80 */
	106, 53, 282, 397, 354, 227, 113, 56, 284, 142,		/* 81 - 90 */
	71, 35							/* 91 - 92 */
};

struct dsi_clock_table dsi_clk_tbl[] = {
		{33300, 80, 6}, {32300, 78, 6}, {31300, 75, 6}, {30000, 72, 6},
		{37300, 90, 6}, {36300, 87, 6}, {35300, 85, 6}, {34300, 82, 6},
		{40000, 80, 5}, {39300, 79, 5}, {39000, 78, 5}, {38300, 92, 6},
		{40400, 81, 5}, {40300, 81, 5}, {40200, 80, 5}, {40100, 80, 5},
		{40800, 82, 5}, {40700, 81, 5}, {40600, 81, 5}, {40500, 81, 5},
		{41200, 82, 5}, {41100, 82, 5}, {41000, 82, 5}, {40900, 82, 5},
		{42000, 84, 5}, {41900, 84, 5}, {41800, 84, 5}, {41700, 83, 5},
		{41600, 83, 5}, {41500, 83, 5}, {41400, 83, 5}, {41300, 83, 5},
		{46000, 92, 5}, {45000, 90, 5}, {44000, 88, 5}, {43000, 86, 5},
		{50000, 80, 4}, {49000, 78, 4}, {48000, 77, 4}, {47000, 75, 4},
		{54000, 86, 4}, {53000, 85, 4}, {52000, 83, 4}, {51000, 82, 4},
		{58000, 70, 3}, {57000, 91, 4}, {56000, 90, 4}, {55000, 88, 4},
		{62000, 74, 3}, {61000, 73, 3}, {60000, 72, 3}, {59000, 71, 3},
		{66000, 79, 3}, {65000, 78, 3}, {64000, 77, 3}, {63000, 76, 3},
		{70000, 84, 3}, {69000, 83, 3}, {68000, 82, 3}, {67000, 80, 3},
		{74000, 89, 3}, {73000, 88, 3}, {72000, 86, 3}, {71000, 85, 3},
		{78000, 62, 2}, {77000, 92, 3}, {76000, 91, 3}, {75000, 90, 3},
		{90000, 72, 2}, {88000, 70, 2}, {80000, 64, 2}, {79000, 63, 2},
		{100000, 80, 2},	/* dsi clock frequency in 10Khz*/
};



int intel_configure_dsi_pll(struct intel_dsi *intel_dsi, u32 pixel_clock)
{
	struct drm_i915_private *dev_priv =
			intel_dsi->base.base.dev->dev_private;
	u32 target_dsi_clk = 0;
	u32 ddr_clk_pixel;
	u32 ddr_clk;
	u8 lane_count;
	unsigned int i;
	u8 m;
	u8 n;
	u8 p;
	u32 m_seed;
	u32 dsi_pll_ctrl;
	u32 dsi_pll_div;


	lane_count = intel_dsi->lane_count;

	/* DDR Clock For Pixels In Mbps
	 * clk = (PixelClock In Khz * ColorDepth in Bpp)
	 *		/ ( 2 * NumberOfLanes * 1000)
	 * DDR Clock = 15% of DdrClockPixelBit + DdrClockPixelBit */
	if (intel_dsi->dsi_packet_format == dsi_24Bpp_packed) {
		ddr_clk_pixel = (pixel_clock * 24) / (lane_count * 1000);
		ddr_clk = ((ddr_clk_pixel * 15) / 100) + ddr_clk_pixel;
		target_dsi_clk  = ddr_clk * 100;
	} else if (intel_dsi->dsi_packet_format == dsi_18Bpp_loosely_packed) {
		ddr_clk_pixel = (pixel_clock * 24) / (lane_count * 1000);
		ddr_clk = ((ddr_clk_pixel * 15) / 100) + ddr_clk_pixel;
		target_dsi_clk  = ddr_clk * 100;
	} else {
		ddr_clk_pixel = (pixel_clock * 18) / (lane_count * 1000);
		ddr_clk = ((ddr_clk_pixel * 15) / 100) + ddr_clk_pixel;
		target_dsi_clk  = ddr_clk * 100;
	}

	if (target_dsi_clk < 30000 || target_dsi_clk > 100000)
		return -ECHRNG;

	for (i = 0; i <= sizeof(dsi_clk_tbl)/sizeof(struct dsi_clock_table);
			i++) {
		if (dsi_clk_tbl[i].freq > target_dsi_clk) {
			if (i == 0)
				break;
			if ((dsi_clk_tbl[i].freq - target_dsi_clk) <
				(target_dsi_clk - dsi_clk_tbl[i - 1].freq)) {
				break;
			} else {
				i = i - 1;
				break;
			}
		}
	}

	m = dsi_clk_tbl[i].m;
	p = dsi_clk_tbl[i].p;
	m_seed = lfsr_converts[m - 62];
	n = 1;
	dsi_pll_ctrl = (1 << (17 + p - 2)) | (1 << 8);
	dsi_pll_div = ((n - 1) << 16) | m_seed;

	intel_cck_write32(dev_priv, 0x48, 0x00000000);
	intel_cck_write32(dev_priv, 0x4C, dsi_pll_div);
	intel_cck_write32(dev_priv, 0x48, dsi_pll_ctrl);

	return 0;
}

int intel_enable_dsi_pll(struct intel_dsi *intel_dsi)
{
	struct drm_i915_private *dev_priv =
			intel_dsi->base.base.dev->dev_private;
	u32 tmp;

	udelay(1000);
	intel_cck_write32_bits(dev_priv, 0x48, 1 << 31,  1 << 31);
	intel_punit_write32_bits(dev_priv, 0x39, 0x00000003, 0x00000003);
	udelay(1000);

	intel_punit_read32(dev_priv, 0x39, &tmp);

	if ((tmp & 0x03) == 0) {
		intel_punit_write32_bits(dev_priv, 0x39,
				0x00000003, 0x00000003);
		udelay(1000);
	}

	if ((tmp & 0x03) == 3) {
		intel_punit_write32_bits(dev_priv, 0x39,
				0x00000000, 0x00000003);
		udelay(1000);
	}

	if (wait_for(((I915_READ(PIPECONF(intel_dsi->pipe)) & (1 << 29)) ==
			(1 << 29)), 20)) {
		DRM_ERROR("DSI PLL lock failed\n");
		return -EIO;
	}

	return 0;
}
