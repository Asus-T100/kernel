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
	/* create private data, slam to dsi->dev_priv. could support many panels
	 * based on dsi->name. This panal supports both command and video mode,
	 * so check the type. */

	/* where to get all the board info style stuff:
	 *
	 * - gpio numbers, if any (external te, reset)
	 * - pin config, mipi lanes
	 * - dsi backlight? (->create another bl device if needed)
	 * - esd interval, ulps timeout
	 *
	 */

	DRM_DEBUG_KMS("\n");

	dsi->eotp_pkt = 1;
	dsi->operation_mode = DSI_VIDEO_MODE;
	dsi->video_mode_type = DSI_VIDEO_BURST | IP_TG_CONFIG;
	dsi->pixel_format = VID_MODE_FORMAT_RGB666_LOOSE;
	dsi->port_bits = BANDGAP_LNC_CIRCUIT | LP_OUTPUT_HOLD;
	dsi->turn_arnd_val = 0x3f;
	dsi->rst_timer_val = 0xff;
	dsi->hs_to_lp_count = 0x1b;
	dsi->lp_byte_clk = 4;
	dsi->bw_timer = 0;
	dsi->clk_lp_to_hs_count = 0x1b;
	dsi->clk_hs_to_lp_count = 0x0c;
	dsi->video_frmt_cfg_bits = IP_TG_CONFIG;
	dsi->dphy_reg = 0x1B104315;

	return true;
}

void generic_create_resources(struct intel_dsi_device *dsi) { }

void generic_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	if (enable) {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);

	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
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

enum drm_connector_status generic_detect(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->is_mipi = true;

	return connector_status_connected;
}

bool generic_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *generic_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 768;
	mode->hsync_start = 828;
	mode->hsync_end = 892;
	mode->htotal = 948;

	mode->vdisplay = 1024;
	mode->vsync_start = 1160;
	mode->vsync_end = 1210;
	mode->vtotal = 1240;

	mode->vrefresh = 60;

	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
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
	.detect = generic_detect,
	.get_hw_state = generic_get_hw_state,
	.get_modes = generic_get_modes,
	.destroy = generic_destroy,
	.dump_regs = generic_dump_regs,
};
