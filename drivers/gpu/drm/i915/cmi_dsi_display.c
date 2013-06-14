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
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"

/*
 * This would be a panel driver
 *
 * It gets the callbacks in the ops (which need to be amended) and uses the
 * provided interfaces to do dsi communications and stuff.
 *
 * In an ideal world this should work nicely. However there are setups where the
 * panel driver (which could be e.g. a DSI-LVDS bridge driver with an attached
 * panel) might need more control over the sequence of events. More flexibility
 * than the rigid ordering of callbacks provide.
 *
 * Currently we call dsi command functions directly, but with the future common
 * display framework they could be through function pointers, to make this panel
 * driver generic across SoC platforms.
 *
 * This panel supports both video and command mode. It should check the passed
 * struct intel_dsi_device pointer, see the type, and act accordingly.
 *
 */

/* Manufacturer specific commands for panel initialization. */
enum {
	MCS_SET_PASSWORD		= 0xb9,
	MCS_CABC_BL			= 0x51,
	MCS_CABC_ON			= 0x53,
	MCS_CABC_MODE			= 0x55,
	MCS_IC_BIAS_CURRENT		= 0xbf,
	MCS_SET_POWER			= 0xb1,
	MCS_SET_DISPLAY_REGISTER	= 0xb2,
	MCS_SET_COMMAND_CYC		= 0xb4,
	MCS_SET_MIPI_CONTROL_LANES	= 0xba,
	MCS_SET_MODE			= 0xc2,
	MCS_SET_BLANKING_AREA_OPTION	= 0xc7,
	MCS_SET_PANEL			= 0xcc,
	MCS_SET_LTPS_EQ_FUNCTION	= 0xd4,
	MCS_SET_LTPS_CONTROL_OUTPUT	= 0xd5,
	MCS_SET_VEDIO_CYC		= 0xd8,
	MCS_SET_R_GAMMA			= 0xe0,
	MCS_SET_G_GAMMA			= 0xe1,
	MCS_SET_B_GAMMA			= 0xe2,
};

static u8 mcs_set_password[] = {
	MCS_SET_PASSWORD, 0xff, 0x83, 0x92,
};

static u8 mcs_cabc_bl[] = {
	MCS_CABC_BL, 0xff,
};

static u8 mcs_cabc_on[] = {
	MCS_CABC_ON, 0x24,
};

static u8 mcs_cabc_mode[] = {
	MCS_CABC_MODE, 0x00,
};

static u8 mcs_ic_bias_current[] = {
	MCS_IC_BIAS_CURRENT, 0x05, 0x60, 0x82, 0x00, };

static u8 mcs_set_power[] = {
	MCS_SET_POWER,
	0x7c, 0x00, 0x44, 0x24, 0x00, 0x0d, 0x0d, 0x12,
	0x1a, 0x3f, 0x3f, 0x42, 0x72,
};

static u8 mcs_set_display_register[] = {
	MCS_SET_DISPLAY_REGISTER,
	0x0f, 0xc8, 0x05, 0x0f, 0x08, 0x84, 0x00, 0xff,
	0x05, 0x0f, 0x04, 0x20,
};

static u8 mcs_set_command_cyc[] = {
	MCS_SET_COMMAND_CYC,
	0x00, 0x00, 0x05, 0x00, 0xa0, 0x05, 0x16, 0x9d,
	0x30, 0x03, 0x16, 0x00, 0x03, 0x03, 0x00, 0x1b,
	0x06, 0x07, 0x07, 0x00,
};

static u8 mcs_set_mipi_control_lanes[] = {
	MCS_SET_MIPI_CONTROL_LANES, 0x12, 0x83, /* 3 lanes */
};

static u8 mcs_set_video_mode[] = {
	MCS_SET_MODE, 0x03, /* video mode */
};

static u8 mcs_set_command_mode[] = {
	MCS_SET_MODE, 0x08, /* command mode */
};

static u8 mcs_set_blanking_area_option[] = {
	MCS_SET_BLANKING_AREA_OPTION, 0x00, 0x40,
};

static u8 mcs_set_panel[] = {
	MCS_SET_PANEL, 0x08,
};

static u8 mcs_set_ltps_eq_function[] = {
	MCS_SET_LTPS_EQ_FUNCTION, 0x0c,
};

static u8 mcs_set_ltps_control_output[] = {
	MCS_SET_LTPS_CONTROL_OUTPUT,
	0x00, 0x08, 0x08, 0x00, 0x44, 0x55, 0x66, 0x77,
	0xcc, 0xcc, 0xcc, 0xcc, 0x00, 0x77, 0x66, 0x55,
	0x44, 0xcc, 0xcc, 0xcc, 0xcc,
};

static u8 mcs_set_vedio_cyc[] = {
	MCS_SET_VEDIO_CYC,
	0x00, 0x00, 0x04, 0x00, 0xa0, 0x04, 0x16, 0x9d,
	0x30, 0x03, 0x16, 0x00, 0x03, 0x03, 0x00, 0x1b,
	0x06, 0x07, 0x07, 0x00,
};

static u8 mcs_set_r_gamma[] = {
	MCS_SET_R_GAMMA,
	0x3a, 0x3e, 0x3c, 0x2f, 0x31, 0x32, 0x33, 0x46,
	0x04, 0x08, 0x0c, 0x0d, 0x10, 0x0f, 0x11, 0x10,
	0x17, 0x3a, 0x3e, 0x3c, 0x2f, 0x31, 0x32, 0x33,
	0x46, 0x04, 0x08, 0x0c, 0x0d, 0x10, 0x0f, 0x11,
	0x10, 0x17,
};

static u8 mcs_set_g_gamma[] = {
	MCS_SET_G_GAMMA,
	0x3b, 0x3e, 0x3d, 0x31, 0x31, 0x32, 0x33, 0x46,
	0x03, 0x07, 0x0b, 0x0d, 0x10, 0x0e, 0x11, 0x10,
	0x17, 0x3b, 0x3e, 0x3d, 0x31, 0x31, 0x32, 0x33,
	0x46, 0x03, 0x07, 0x0b, 0x0d, 0x10, 0x0e, 0x11,
	0x10, 0x17,
};

static u8 mcs_set_b_gamma[] = {
	MCS_SET_B_GAMMA,
	0x01, 0x06, 0x07, 0x2d, 0x2a, 0x32, 0x1f, 0x40,
	0x05, 0x0c, 0x0e, 0x11, 0x14, 0x12, 0x13, 0x0f,
	0x18, 0x01, 0x06, 0x07, 0x2d, 0x2a, 0x32, 0x1f,
	0x40, 0x05, 0x0c, 0x0e, 0x11, 0x14, 0x12, 0x13,
	0x0f, 0x18
};

#define ELEM(x) { x, sizeof(x) }
static struct {
	const u8 *data;
	size_t len;
} mcs_init_commands[] = {
	ELEM(mcs_cabc_bl),
	ELEM(mcs_cabc_on),
	ELEM(mcs_cabc_mode),
	ELEM(mcs_ic_bias_current),
	ELEM(mcs_set_power),
	ELEM(mcs_set_display_register),
	ELEM(mcs_set_command_cyc),
	ELEM(mcs_set_mipi_control_lanes),
	ELEM(mcs_set_video_mode),
	ELEM(mcs_set_blanking_area_option),
	ELEM(mcs_set_panel),
	ELEM(mcs_set_ltps_eq_function),
	ELEM(mcs_set_ltps_control_output),
	ELEM(mcs_set_vedio_cyc),
	ELEM(mcs_set_r_gamma),
	ELEM(mcs_set_g_gamma),
	ELEM(mcs_set_b_gamma),
};
#undef ELEM

bool cmi_init(struct intel_dsi_device *dsi)
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
	return true;
}

void cmi_create_resources(struct intel_dsi_device *dsi) { }

void cmi_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	if (enable) {
		int i;

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_generic_write(intel_dsi, 0, mcs_set_password,
				     sizeof(mcs_set_password));

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		for (i = 0; i < ARRAY_SIZE(mcs_init_commands); i++) {
			dsi_vc_generic_write(intel_dsi, 0,
					     mcs_init_commands[i].data,
					     mcs_init_commands[i].len);
		}

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
}

int cmi_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return 1;
}

bool cmi_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void cmi_prepare(struct intel_dsi_device *dsi) { }

void cmi_commit(struct intel_dsi_device *dsi) { }

void cmi_mode_set(struct intel_dsi_device *dsi,
		  struct drm_display_mode *mode,
		  struct drm_display_mode *adjusted_mode) { }

enum drm_connector_status cmi_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

bool cmi_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *cmi_get_modes(struct intel_dsi_device *dsi)
{
	return NULL;
}

void cmi_dump_regs(struct intel_dsi_device *dsi) { }

void cmi_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */ struct intel_dsi_dev_ops
cmi_dsi_display_ops = {
	.init = cmi_init,
	.create_resources = cmi_create_resources,
	.dpms = cmi_dpms,
	.mode_valid = cmi_mode_valid,
	.mode_fixup = cmi_mode_fixup,
	.prepare = cmi_prepare,
	.commit = cmi_commit,
	.mode_set = cmi_mode_set,
	.detect = cmi_detect,
	.get_hw_state = cmi_get_hw_state,
	.get_modes = cmi_get_modes,
	.destroy = cmi_destroy,
	.dump_regs = cmi_dump_regs,
};
