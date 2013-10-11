/*
 * Copyright © 2013 Intel Corporation
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
 * Author: Jani Nikula <jani.nikula@intel.com>
 *	   Shobhit Kumar <shobhit.kumar@intel.com>
 *
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include <asm/intel-mid.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_vbt_generic.h"

struct gpio_table gtable[] = {
	{ GPI0_NC_0_HV_DDI0_HPD, GPIO_NC_0_HV_DDI0_PAD, 0 },
	{ GPIO_NC_1_HV_DDI0_DDC_SDA, GPIO_NC_1_HV_DDI0_DDC_SDA_PAD, 0 },
	{ GPIO_NC_2_HV_DDI0_DDC_SCL, GPIO_NC_2_HV_DDI0_DDC_SCL_PAD, 0 },
	{ GPIO_NC_3_PANEL0_VDDEN, GPIO_NC_3_PANEL0_VDDEN_PAD, 0 },
	{ GPIO_NC_4_PANEL0_BLKEN, GPIO_NC_4_PANEL0_BLKEN_PAD, 0 },
	{ GPIO_NC_5_PANEL0_BLKCTL, GPIO_NC_5_PANEL0_BLKCTL_PAD, 0 }
};

static u8 *mipi_exec_send_packet(struct intel_dsi *intel_dsi, u8 *data)
{
	u8 type, byte, mode, vc, port;
	u16 len;

	DRM_DEBUG_DRIVER("MIPI: Executing sene packet element\n");

	byte = *data++;
	mode = (byte >> MIPI_TRANSFER_MODE_SHIFT) & 0x1;
	vc = (byte >> MIPI_VIRTUAL_CHANNEL_SHIFT) & 0x3;
	port = (byte >> MIPI_PORT_SHIFT) & 0x3;

	/* get packet type and increment the pointer */
	type = *data++;

	len = *data;
	data += 2;

	switch (type) {
	case MIPI_GENERIC_SHORT_WRITE_0_PARAM:
		dsi_vc_generic_write_0(intel_dsi, vc);
		break;
	case MIPI_GENERIC_SHORT_WRITE_1_PARAM:
		dsi_vc_generic_write_1(intel_dsi, vc, *data);
		data++;
		break;
	case MIPI_GENERIC_SHORT_WRITE_2_PARAM:
		dsi_vc_generic_write_2(intel_dsi, vc, *data, *(data + 1));
		data += 2;
		break;
	case MIPI_GENERIC_READ_0_PARAM:
	case MIPI_GENERIC_READ_1_PARAM:
	case MIPI_GENERIC_READ_2_PARAM:
		DRM_DEBUG_DRIVER("Generic Read not yet implemented or used\n");
		break;
	case MIPI_GENERIC_LONG_WRITE:
		dsi_vc_generic_write(intel_dsi, vc, data, len);
		data += len;
		break;
	case MIPI_MAN_DCS_SHORT_WRITE_0_PARAM:
		dsi_vc_dcs_write_0(intel_dsi, vc, *data);
		data++;
		break;
	case MIPI_MAN_DCS_SHORT_WRITE_1_PARAM:
		dsi_vc_dcs_write_1(intel_dsi, vc, *data, *(data + 1));
		data += 2;
		break;
	case MIPI_MAN_DCS_READ_0_PARAM:
		DRM_DEBUG_DRIVER("DCS Read not yet implemented or used\n");
		break;
	case MIPI_MAN_DCS_LONG_WRITE:
		dsi_vc_dcs_write(intel_dsi, vc, data, len);
		data += len;
		break;
	};

	return data;
}

static u8 *mipi_exec_delay(struct intel_dsi *intel_dsi, u8 *data)
{
	u32 delay = *data;

	DRM_DEBUG_DRIVER("MIPI: executing delay element\n");
	usleep_range(delay, delay + 10);
	data += 4;

	return data;
}

static u8 *mipi_exec_gpio(struct intel_dsi *intel_dsi, u8 *data)
{
	u8 gpio, action;
	u16 function, pad;
	u32 val;
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_DRIVER("MIPI: executing gpio element\n");
	gpio = *data++;

	/* pull up/down */
	action = *data++;

	function = gtable[gpio].function_reg;
	pad = gtable[gpio].pad_reg;

	if (!gtable[gpio].init) {
		/* program the function */
		intel_gpio_nc_write32(dev_priv, function, 0x2000CC00);
		gtable[gpio].init = 1;
	}

	val = 0x4 | action;

	/* pull up/down */
	intel_gpio_nc_write32(dev_priv, pad, val);

	return data;
}

FN_MIPI_ELEM_EXEC exec_elem[] = {
	NULL, /* reserved */
	mipi_exec_send_packet,
	mipi_exec_delay,
	mipi_exec_gpio,
	NULL, /* status read; later */
};

/*
 * MIPI Sequence from VBT #53 parsing logic
 * We have already separated each seqence during bios parsing
 * Following is generic execution function for any sequence
 */
static void generic_exec_sequence(struct intel_dsi *intel_dsi, char *sequence)
{
	u8 *data = sequence;
	FN_MIPI_ELEM_EXEC mipi_elem_exec;
	int index;

	DRM_DEBUG_DRIVER("Starting MIPI sequence - %d\n", *data);

	/* go to the first element of the sequence */
	data++;

	/* parse each byte till we reach end of sequence byte - 0x00 */
	while (1) {
		index = *data;
		mipi_elem_exec = exec_elem[index];

		/* goto element payload */
		data++;

		/* execute the element specifc rotines */
		data = mipi_elem_exec(intel_dsi, data);

		/*
		 * After processing the element, data should point to
		 * next element or end of sequence
		 * check if have we reached end of sequence
		 */
		if (*data == 0x00)
			break;
	}
}

static void generic_get_panel_info(int pipe, struct drm_connector *connector)
{
	DRM_DEBUG_KMS("\n");
	if (!connector)
		return;

	if (pipe == 0) {
		/* FIXME: Fill from VBT */
		connector->display_info.width_mm = 0;
		connector->display_info.height_mm = 0;
	}

	return;
}

bool generic_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct _mipi_config *mipi_config = dev_priv->mipi.config;
	unsigned long long pixel_clock;
	u32 bits_per_pixel = 24;
	u32 tlpx_ns, extra_byte_count, bitrate, tlpx_ui;
	struct drm_display_mode *mode = dev_priv->lfp_lvds_vbt_mode;
	u32 ui_num, ui_den;

	DRM_DEBUG_KMS("\n");

	dsi->eotp_pkt = mipi_config->eot_disabled ? 0 : 1;
	dsi->lane_count = mipi_config->lane_cnt + 1;
	dsi->pixel_format = mipi_config->videomode_color_format << 7;

	/*
	 * If DSI DDR clock frequency is not present in VBT,
	 * calculate and initialize dsi_clock_freq
	 */
	if (mipi_config->dsi_ddr_clk) {
		dsi->dsi_clock_freq = mipi_config->dsi_ddr_clk / 1000;
	} else {
		pixel_clock = mode->clock;
		if (dsi->pixel_format == VID_MODE_FORMAT_RGB666)
			bits_per_pixel = 18;
		else if (dsi->pixel_format == VID_MODE_FORMAT_RGB565)
			bits_per_pixel = 16;

		/* why this is data rate in Mbps !!! */
		dsi->dsi_clock_freq = (pixel_clock * 1000 * bits_per_pixel)
							/ (1024 * 1024);
		dsi->dsi_clock_freq = dsi->dsi_clock_freq / dsi->lane_count;
	}

	/* in Kbps */
	bitrate = dsi->dsi_clock_freq * 1024;
	ui_num = bitrate;
	ui_den = NS_MHZ_RATIO;

	dsi->operation_mode = mipi_config->cmd_mode;
	dsi->video_mode_type = mipi_config->vtm;
	dsi->escape_clk_div = mipi_config->byte_clk_sel;
	dsi->lp_rx_timeout = mipi_config->lp_rx_timeout;
	dsi->turn_arnd_val = mipi_config->turn_around_timeout;
	dsi->rst_timer_val = mipi_config->device_reset_timer;
	dsi->init_count = mipi_config->master_init_timer;
	dsi->hs_to_lp_count = mipi_config->hl_switch_cnt;
	dsi->bw_timer = mipi_config->dbi_bw_timer;
	dsi->video_frmt_cfg_bits = mipi_config->bta ? DISABLE_VIDEO_BTA : 0;

	/*
	 * DPHY parameters are calculated, if not present in VBT.
	 * ui(s) = 1/f [f in hz]
	 * ui(ns) = 10^9/f*10^6 [f in Mhz] -> 10^3/f(Mhz)
	 *
	 * LP byte clock = TLPX/8ui
	 *
	 * DPHY param register value is calculated if one or more
	 * parameter values are not set in VBT.
	 *
	 * As per DPHY spec, THS-PREPARE min = 40ns+4UI, max = 85ns+6UI
	 * THS-PREPARE count is calculated as the average between min
	 * and max DPHY spec specified values.
	 * TCLK-PREPARE+TCLK-ZERO = 300ns
	 * TCLK-ZERO = 300 - TCLK-PREPARE
	 * TCLK-TRAIL = 60ns
	 * THS-EXIT = 100ns
	 *
	 * Since txddrclkhs_i is 2xUI, the count values programmed in
	 * DPHY param register are divided by 2
	 *
	 */
	switch (dsi->escape_clk_div) {
	case 0:
		tlpx_ns = 50;
		break;
	case 1:
		tlpx_ns = 100;
		break;

	case 2:
		tlpx_ns = 200;
		break;
	default:
		tlpx_ns = 50;
		break;
	}

	switch (dsi->lane_count) {
	case 1:
	case 2:
		extra_byte_count = 2;
		break;
	case 3:
		extra_byte_count = 4;
		break;
	case 4:
	default:
		extra_byte_count = 3;
		break;
	}

	/* B060 */
	dsi->lp_byte_clk = ceil_div(tlpx_ns * ui_num, 8 * ui_den);

	if (!mipi_config->prepare_cnt ||
			!mipi_config->clk_zero_cnt ||
			!mipi_config->trail_cnt ||
			!mipi_config->exit_zero_cnt) {

		/* prepare count */
		mipi_config->prepare_cnt = ceil_div(
					mipi_config->ths_prepare * ui_num,
					ui_den * 2
					);

		/* exit zero count */
		mipi_config->exit_zero_cnt = ceil_div(
			(mipi_config->ths_prepare_hszero -
					mipi_config->ths_prepare) * ui_num,
					ui_den * 2
					);

		if (mipi_config->exit_zero_cnt < (55 * ui_num / ui_den)) {
			int mod = (55 * ui_num) % ui_den;
			if (mod)
				mipi_config->exit_zero_cnt += 1;
		}

		/* clk zero count */
		mipi_config->clk_zero_cnt = ceil_div(
				(mipi_config->tclk_prepare_clkzero -
					mipi_config->ths_prepare) * ui_num,
					2 * ui_den
					);

		/* trail count */
		mipi_config->trail_cnt = ceil_div(
				mipi_config->tclk_trail * ui_num,
				2 * ui_den);
	}

	/* B080 */
	dsi->dphy_reg = mipi_config->exit_zero_cnt << 24
			| mipi_config->trail_cnt << 16
			| mipi_config->clk_zero_cnt << 8
			| mipi_config->prepare_cnt;

	DRM_DEBUG_DRIVER("dsi->dphy_reg = 0x%x\n",  dsi->dphy_reg);

	/*
	 * Calculate switch count if the values are not set in VBT.
	 * LP to HS switch count = 4TLPX + PREP_COUNT + EXIT_ZERO_COUNT
	 *				+ 10UI + Extra Byte Count
	 *
	 * HS to LP switch count = THS-TRAIL + 2TLPX + Extra Byte Count
	 * Extra Byte Count is calculated according to number of lanes.
	 * High Low Switch Count is the Max of LP to HS and
	 * HS to LP switch count
	 *
	 */
	tlpx_ui = ceil_div(tlpx_ns * ui_num, ui_den);

	/* B044 */
	dsi->hs_to_lp_count =
		ceil_div(
			4 * tlpx_ui + mipi_config->prepare_cnt * 2 +
			mipi_config->clk_zero_cnt + 10,
			8);

	dsi->hs_to_lp_count += extra_byte_count;

	/* B088 */
	if (mipi_config->clk_lane_switch_cnt) {
		dsi->clk_lp_to_hs_count =
			(mipi_config->clk_lane_switch_cnt & 0xffff0000)
			>> 16;
		dsi->clk_hs_to_lp_count =
			mipi_config->clk_lane_switch_cnt & 0xffff;
	} else {
		dsi->clk_lp_to_hs_count =
			ceil_div(
				4 * tlpx_ui + mipi_config->prepare_cnt * 2 +
				mipi_config->clk_zero_cnt * 2,
				8);

		dsi->clk_lp_to_hs_count += extra_byte_count;

		dsi->clk_hs_to_lp_count =
			ceil_div(2 * tlpx_ui + mipi_config->trail_cnt * 2 + 8,
				8);
		dsi->clk_hs_to_lp_count += extra_byte_count;
	}

	DRM_DEBUG_DRIVER("B044 = 0x%x, B060 = 0x%x, B080 = 0x%x, B088 = 0x%x\n",
			dsi->hs_to_lp_count, dsi->lp_byte_clk, dsi->dphy_reg,
			dsi->clk_lp_to_hs_count << 16 |
			dsi->clk_hs_to_lp_count);

	DRM_DEBUG_KMS("EOT %s\n", dsi->eotp_pkt ? "ENABLED" : "DISABLED");
	DRM_DEBUG_KMS("DSI Frequency %d\n", dsi->dsi_clock_freq);
	DRM_DEBUG_KMS("Mode %s\n", dsi->operation_mode ? "COMMAND" : "VIDEO");
	DRM_DEBUG_KMS("Pixel Format %d\n", dsi->pixel_format);
	DRM_DEBUG_KMS("TLPX %d\n", dsi->escape_clk_div);
	DRM_DEBUG_KMS("LP RX Timeout 0x%x\n", dsi->lp_rx_timeout);
	DRM_DEBUG_KMS("Turnaround Timeout 0x%x\n", dsi->turn_arnd_val);
	DRM_DEBUG_KMS("Init Count 0x%x\n", dsi->init_count);
	DRM_DEBUG_KMS("HS to LP Count 0x%x\n", dsi->hs_to_lp_count);
	DRM_DEBUG_KMS("LP Byte Clock %d\n", dsi->lp_byte_clk);
	DRM_DEBUG_KMS("DBI BW Timer 0x%x\n", dsi->bw_timer);
	DRM_DEBUG_KMS("LP to HS Clock Count 0x%x\n", dsi->clk_lp_to_hs_count);
	DRM_DEBUG_KMS("HS to LP Clock Count 0x%x\n", dsi->clk_hs_to_lp_count);
	DRM_DEBUG_KMS("BTA %s\n",
			dsi->video_frmt_cfg_bits & DISABLE_VIDEO_BTA ?
			"DISABLED" : "ENABLED");
	DRM_DEBUG_KMS("DPHY 0x%x\n", dsi->dphy_reg);

	dsi->backlight_off_delay = 20;
	dsi->send_shutdown = false;
	dsi->shutdown_pkt_delay = 20;

	return true;
}

void generic_create_resources(struct intel_dsi_device *dsi) { }

void generic_dpms(struct intel_dsi_device *dsi, bool enable)
{
	DRM_DEBUG_KMS("\n");
}

int generic_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool generic_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void generic_panel_reset(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->mipi.sequence[MIPI_SEQ_ASSERT_RESET];
	generic_exec_sequence(intel_dsi, sequence);
}

void generic_disable_panel_power(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->mipi.sequence[MIPI_SEQ_DEASSERT_RESET];
	generic_exec_sequence(intel_dsi, sequence);
}

void generic_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->mipi.sequence[MIPI_SEQ_DISPLAY_OFF];
	generic_exec_sequence(intel_dsi, sequence);
}

enum drm_connector_status generic_detect(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->lfp_type = INTEL_OUTPUT_DSI;

	return connector_status_connected;
}

bool generic_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *generic_get_modes(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->lfp_lvds_vbt_mode->type |= DRM_MODE_TYPE_PREFERRED;
	return dev_priv->lfp_lvds_vbt_mode;
}

void generic_dump_regs(struct intel_dsi_device *dsi) { }

void generic_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops vbt_generic_dsi_display_ops = {
	.init = generic_init,
	.get_info = generic_get_panel_info,
	.create_resources = generic_create_resources,
	.dpms = generic_dpms,
	.mode_valid = generic_mode_valid,
	.mode_fixup = generic_mode_fixup,
	.panel_reset = generic_panel_reset,
	.disable_panel_power = generic_disable_panel_power,
	.disable = generic_disable,
	.detect = generic_detect,
	.get_hw_state = generic_get_hw_state,
	.get_modes = generic_get_modes,
	.destroy = generic_destroy,
	.dump_regs = generic_dump_regs
};
