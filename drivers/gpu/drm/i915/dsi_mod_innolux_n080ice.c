/* 
 *opyright © 2013 Intel Corporation
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
 * Author: Ben Kao <ben.kao@intel.com>
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
#include "dsi_mod_innolux_n080ice.h"
#include <linux/mfd/intel_mid_pmic.h>


static void  n080ice_get_panel_info(int pipe, struct drm_connector *connector)
{
	if (!connector) {
		DRM_DEBUG_KMS("Innolux: Invalid input to get_info\n");
		return;
	}

	if (pipe == 0) {
		connector->display_info.width_mm = 108;
		connector->display_info.height_mm = 172;
	}

	return;
}

static struct drm_display_mode *n080ice_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode = NULL;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("Innolux: No memory\n");
		return NULL;
	}

	mode->hdisplay = 800;
	mode->hsync_start = mode->hdisplay + 40;
	mode->hsync_end = mode->hsync_start + 4;
	mode->htotal = mode->hsync_end + 40;
	mode->vdisplay = 1280;
	mode->vsync_start = mode->vdisplay + 10;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 12;
	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}


static bool n080ice_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

static enum drm_connector_status n080ice_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

static bool n080ice_mode_fixup(struct intel_dsi_device *dsi,
			       const struct drm_display_mode *mode,
			       struct drm_display_mode *adjusted_mode) {
	return true;
}

static int n080ice_mode_valid(struct intel_dsi_device *dsi,
			      struct drm_display_mode *mode)
{
	return MODE_OK;
}

static void n080ice_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	if (enable) {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
}

void n080ice_panel_reset(struct intel_dsi_device *dsi)
{
	intel_mid_pmic_writeb(0x52, 1);
	msleep(20);	//<asus-Bruce 20131129+>
}

void n080ice_send_otp_cmds(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	msleep(40);
	intel_gpio_nc_write32(dev_priv, 0x4160, 0x2000CC00);
	intel_gpio_nc_write32(dev_priv, 0x4168, 0x00000005);
	usleep_range(3500, 4500);
	intel_gpio_nc_write32(dev_priv, 0x4168, 0x00000004);
	usleep_range(3500, 4500);
	intel_gpio_nc_write32(dev_priv, 0x4168, 0x00000005);
	msleep(21);

	intel_dsi->hs = 0;
	{
		u8 buf[] = {0xFF, 0xAA, 0x55, 0xA5, 0x80};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	{
		u8 buf[] = {0x6F, 0x11, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xF7, 0x20, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF7, 0xA0);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x19);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF7, 0x12);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x08);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xFA, 0x40);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x11);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF3, 0x01);
	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC8, 0x80);
	{
		u8 buf[] = {0xB1, 0x6C, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB6, 0x08);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x02);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB8, 0x08);
	{
		u8 buf[] = {0xBB, 0x74, 0x44};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBC, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBD, 0x02, 0xB0, 0x0C, 0x0A, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xB0, 0x05, 0x05};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB1, 0x05, 0x05};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBC, 0x90, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBD, 0x90, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xCA, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC0, 0x04);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xBE, 0x29);
	{
		u8 buf[] = {0xB3, 0x37, 0x37};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB4, 0x19, 0x19};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB9, 0x44, 0x44};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBA, 0x24, 0x24};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xEE, 0x01);
	{
		u8 buf[] = {0xEF, 0x09, 0x06, 0x15, 0x18};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	{
		u8 buf[] = {0xB0, 0x00, 0x00, 0x00, 0x25, 0x00, 0x43};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 7);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);
	{
		u8 buf[] = {0xB0, 0x00, 0x54, 0x00, 0x68, 0x00, 0xA0};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 7);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x0C);
	{
		u8 buf[] = {0xB0, 0x00, 0xC0, 0x01, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	{
		u8 buf[] = {0xB1, 0x01, 0x30, 0x01, 0x78, 0x01, 0xAE};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 7);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);
	{
		u8 buf[] = {0xB1, 0x02, 0x08, 0x02, 0x50, 0x02, 0x52};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 7);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x0C);
	{
		u8 buf[] = {0xB1, 0x02, 0x96, 0x02, 0xDC};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	{
		u8 buf[] = {0xB2, 0x03, 0x08, 0x03, 0x49, 0x03, 0x77};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 7);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);
	{
		u8 buf[] = {0xB2, 0x03, 0xA3, 0x03, 0xAC, 0x03, 0xC2};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 7);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x0C);
	{
		u8 buf[] = {0xB2, 0x03, 0xC9, 0x03, 0xE3};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	{
		u8 buf[] = {0xB3, 0x03, 0xFC, 0x03, 0xFF};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xB0, 0x00, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB1, 0x12, 0x14};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB2, 0x16, 0x18};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB3, 0x1A, 0x29};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB4, 0x2A, 0x08};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB5, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB6, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB7, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB8, 0x31, 0x0A};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB9, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBA, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBB, 0x0B, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBC, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBD, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBE, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xBF, 0x09, 0x2A};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC0, 0x29, 0x1B};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC1, 0x19, 0x17};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC2, 0x15, 0x13};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC3, 0x11, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xE5, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC4, 0x09, 0x1B};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC5, 0x19, 0x17};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC6, 0x15, 0x13};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC7, 0x11, 0x29};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC8, 0x2A, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC9, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xCA, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xCB, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xCC, 0x31, 0x0B};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xCD, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xCE, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xCF, 0x0A, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD0, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD1, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD2, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD3, 0x00, 0x2A};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD4, 0x29, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD5, 0x12, 0x14};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD6, 0x16, 0x18};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD7, 0x1A, 0x08};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xE6, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xD8, 0x00, 0x00, 0x00, 0x54, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xD9, 0x00, 0x15, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE7, 0x00);
	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xB0, 0x20, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB1, 0x20, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB2, 0x05, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xB6, 0x05, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xB7, 0x05, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xBA, 0x57, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xBB, 0x57, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xC0, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	{
		u8 buf[] = {0xC1, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 5);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC4, 0x60);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC5, 0x40);
	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xBD, 0x03, 0x01, 0x03, 0x03, 0x03};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xB0, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB1, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB2, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB3, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB4, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xB5, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB8, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB9, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xBA, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xBB, 0x02);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xBC, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC4, 0x80);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC5, 0xA4);
	{
		u8 buf[] = {0xC8, 0x05, 0x30};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xC9, 0x01, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 3);
	}
	{
		u8 buf[] = {0xCC, 0x00, 0x00, 0x3C};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 4);
	}
	{
		u8 buf[] = {0xCD, 0x00, 0x00, 0x3C};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 4);
	}
	{
		u8 buf[] = {0xD1, 0x00, 0x04, 0xFD, 0x07, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	{
		u8 buf[] = {0xD2, 0x00, 0x05, 0x02, 0x07, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, buf, 6);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE5, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE6, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE7, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE8, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE9, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xEA, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xED, 0x30);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x11);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF3, 0x01);
	dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON);
	dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);
	dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
}

void n080ice_enable(struct intel_dsi_device *dsi)
{
}

void n080ice_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
	dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
}

void n080ice_disable_panel_power(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	intel_gpio_nc_write32(dev_priv, 0x4160, 0x2000CC00);
	intel_gpio_nc_write32(dev_priv, 0x4168, 0x00000004);
	intel_mid_pmic_writeb(0x52, 0);
}

bool n080ice_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!dsi) {
		DRM_DEBUG_KMS("Init: Invalid input to n080ice_init\n");
		return false;
	}

	dsi->eotp_pkt = CLOCKSTOP | EOT_DISABLE;
	dsi->operation_mode = DSI_VIDEO_MODE;
	dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	dsi->port_bits = 0;
	dsi->turn_arnd_val = 0x3F;
	dsi->rst_timer_val = 0xFFFF;
	dsi->init_count = 0x7D0;
	dsi->hs_to_lp_count = 0x46;
	dsi->lp_byte_clk = 0x03;
	dsi->bw_timer = 0;
	dsi->clk_lp_to_hs_count = 0x1E;
	dsi->clk_hs_to_lp_count = 0x0D;
	dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
	dsi->dphy_reg = 0x340F370B;

	dsi->backlight_on_delay = 100;	//<asus-Bruce 20131129+>
	dsi->backlight_off_delay = 134;
	dev_priv->mipi.panel_bpp = 24;

	return true;
}

struct intel_dsi_dev_ops innolux_n080ice_dsi_display_ops = {
	.init = n080ice_init,
	.get_info = n080ice_get_panel_info,
	.dpms = n080ice_dpms,
	.mode_valid = n080ice_mode_valid,
	.mode_fixup = n080ice_mode_fixup,
	.detect = n080ice_detect,
	.get_hw_state = n080ice_get_hw_state,
	.get_modes = n080ice_get_modes,
	.send_otp_cmds = n080ice_send_otp_cmds,
	.panel_reset = n080ice_panel_reset,
	.enable = n080ice_enable,
	.disable = n080ice_disable,
	.disable_panel_power = n080ice_disable_panel_power,
};
