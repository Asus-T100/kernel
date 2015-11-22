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
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_innolux_m181.h"

#include <linux/lnw_gpio.h>
#include <linux/acpi_gpio.h>
#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/mfd/intel_mid_pmic.h>

#define DEBUG 1
#if DEBUG
	#define sean_debug(x...) printk(x)
#else
	#define sean_debug(x...) do {} while(0)
#endif

static void ivo_m181_get_panel_info(int pipe, struct drm_connector *connector)
{
	sean_debug("%s:----sean test----m181_ivo_panel_get_info----\n", __func__);
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = M181_10x7_PANEL_WIDTH;
		connector->display_info.height_mm = M181_10x7_PANEL_HEIGHT;
	}

	return;
}

bool ivo_m181_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	sean_debug("%s:----sean test----m181_ivo_panel_init----\n", __func__);
//	intel_dsi->hs = true;
	intel_dsi->hs = 1;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
//	intel_dsi->eot_disable = 1;
	intel_dsi->eotp_pkt = 0;
	intel_dsi->port_bits = 0;
//	intel_dsi->dsi_clock_freq = 513;
	intel_dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
//	intel_dsi->escape_clk_div = ESCAPE_CLOCK_DIVIDER_1;
//	intel_dsi->lp_rx_timeout = 0xffff;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xff;
//	intel_dsi->init_count = 0x7d0;
	intel_dsi->hs_to_lp_count = 0x19; //sean test ivo
	intel_dsi->lp_byte_clk = 0x3; //sean test ivo
	intel_dsi->bw_timer = 0x0;
	intel_dsi->clk_lp_to_hs_count = 0x1E; //sean test ivo
	intel_dsi->clk_hs_to_lp_count = 0x0D; //sean test ivo
	intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
	intel_dsi->dphy_reg = 0x1B0D340B; //sean test ivo
	intel_dsi->pclk = 68428;
	intel_dsi->burst_mode_ratio = 100; //liwei add

	intel_dsi->backlight_off_delay = 68;	//sean test t5 >= 68
	intel_dsi->send_shutdown = true;
	intel_dsi->backlight_on_delay = 210;	//sean test t5 >= 210
	intel_dsi->shutdown_pkt_delay = 134;	//sean test

	intel_dsi->clock_stop = true;	//seantest no continue more

	return true;
}

void ivo_m181_create_resources(struct intel_dsi_device *dsi) { }

void ivo_m181_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	sean_debug("%s:----sean test----m181_ivo_dpms----\n", __func__);
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

int ivo_m181_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool ivo_m181_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void ivo_m181_panel_reset(struct intel_dsi_device *dsi)
{

	int err;
	sean_debug("%s:----sean test----m181_panel_reset----\n", __func__);

	err = gpio_request(69, "sd_pwr_en");
	if (err){
		printk("%s----sean test----gpio_requset_fail----\n",__func__);
	}
	//msleep(20); //20ms

	gpio_direction_output(69, 1);	// sean 2 low pulse
	//gpio_set_value(69, 1);
	//usleep_range(5000,10000);	//
	gpio_set_value(69, 0);
	usleep_range(5000,10000); 	// > 10E-2 ms
	gpio_set_value(69, 1);
	usleep_range(1000,5000); 	// < 5ms
	gpio_free(69);
	msleep(30);

}

void ivo_m181_disable_panel_power(struct intel_dsi_device *dsi)
{
	int err;
	err = gpio_request(69, "sd_pwr_en1");
	if (err){
		printk("%s:----sean test----m181_ivo_panel_request fail----\n",__func__);
	}

	gpio_direction_output(69, 0);
	gpio_set_value(69, 0);	//RESX low

	//msleep(50);

	intel_mid_pmic_writeb(0x52,0x00);//PANEL_EN 3.3v
	usleep_range(9000,10000);	//sean test t6 <= 10
	intel_mid_pmic_writeb(0x3C,0x24);//GPIOxxxCTLO GPIO1P1 1.8v
	sean_debug("%s:----sean test----panel 1.8V set low----%d\n", __func__,__LINE__);

	err =  intel_mid_pmic_readb(0x52);
	sean_debug("%s:----sean test----ivo_m181_disable_panel_power----%d,3.3v:%d\n", __func__,__LINE__,err);

	gpio_free(69);

}

void ivo_m181_send_otp_cmds(struct intel_dsi_device *dsi)
{

	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	DRM_DEBUG_KMS("\n");

	sean_debug("%s:----sean test----m181_ivo_send_otp_cmds----%d\n", __func__,__LINE__);
	intel_dsi->hs = 0 ;

	msleep(20);

	//========== Internal setting ==========
	sean_debug("%s:----sean test----m181_ivo_send_otp_cmds----%d\n", __func__,__LINE__);
	{
		unsigned char data[] = {0xF0, 0x5A, 0x5A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	{
		unsigned char data[] = {0xF1, 0x5A, 0x5A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	{
		unsigned char data[] = {0xFC, 0x5A, 0x5A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB1, 0x10);

	{
		unsigned char data[] = {0xB2, 0x14, 0x22, 0X2F, 0X04};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB5, 0x30);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB6, 0x11);

	{
		unsigned char data[] = {0xBC, 0x01, 0x4E, 0X0A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);
	}

	{
		unsigned char data[] = {0xC3, 0x40, 0X00, 0X20};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);
	}

	{
		unsigned char data[] = {0xF2, 0x0A, 0X08, 0X08, 0x40, 0x10, 0X0F};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	{
		unsigned char data[] = {0xF3, 0x01, 0xD7, 0XE2, 0X62, 0xF4, 0xF7, 0X77, 0x30, 0x26, 0XFF};
		dsi_vc_dcs_write(intel_dsi, 0, data, 11);
	}

	{
		unsigned char data[] = {0xF4, 0x00, 0x02, 0x03, 0x26, 0x03, 0x02, 0x09, 0x00,
								0x07, 0x16, 0x16, 0x03, 0x00, 0x08, 0x08, 0x03, 0x18,
								0x1A, 0x12, 0x1C, 0x1D, 0x1E, 0x18, 0x09, 0x01, 0x04,
								0x05, 0x81, 0x77, 0x78, 0x72, 0x83, 0x80, 0x80, 0xF0,
								0x00, 0x01, 0x01, 0x28, 0x04, 0x03, 0x28, 0x01, 0xD1, 0x32};
		dsi_vc_dcs_write(intel_dsi, 0, data, 46);
	}

	{
		unsigned char data[] = {0xF5, 0x7C, 0x3C, 0x3C, 0x5F, 0xAB, 0x98, 0x52, 0x0F,
								0x33, 0x43, 0x04, 0x59, 0x54, 0x52, 0x05, 0x40, 0x60,
								0x58, 0x60, 0x40, 0x27, 0x26, 0x52, 0x25, 0x6D, 0x18};
		dsi_vc_dcs_write(intel_dsi, 0, data, 27);
	}

	{
		unsigned char data[] = {0xEE, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x1F, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 9);
	}

	{
		unsigned char data[] = {0xEF, 0x34, 0x12, 0x32, 0x54, 0x30, 0x80, 0x04, 0x80,
								0x00, 0x00, 0x21, 0x03, 0x03, 0x40, 0x80, 0x82, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 18);
	}

	{
		unsigned char data[] = {0xF7, 0x02, 0x02, 0x13, 0x11, 0x0F, 0x0F, 0x0B, 0x0B,
								0x0E, 0x0E, 0x0A, 0x0A, 0x05, 0x07, 0x1F, 0x1F, 0x02,
								0x02, 0x12, 0x10, 0x0D, 0x0D, 0x09 ,0x09, 0x0C, 0x0C,
								0x08, 0x08, 0x04, 0x06, 0x1F, 0x1F};
		dsi_vc_dcs_write(intel_dsi, 0, data, 33);
	}

	{
		unsigned char data[] = {0xF6, 0x60, 0x25, 0x86, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	{
		unsigned char data[] = {0xFA, 0x00, 0x2D, 0x07, 0x04, 0x0B, 0x00, 0x04, 0x07,
								0x0A, 0x15, 0x20, 0x29, 0x32, 0x33, 0x2C, 0x29, 0x33};
		dsi_vc_dcs_write(intel_dsi, 0, data, 18);
	}

	{
		unsigned char data[] = {0xFB, 0x0B, 0x39, 0x07, 0x0D, 0x0C, 0x00, 0x04, 0x08,
								0x0C, 0x16, 0x20, 0x28, 0x32, 0x33, 0x2C, 0x2F, 0x33};
		dsi_vc_dcs_write(intel_dsi, 0, data, 18);
	}

	{
		unsigned char data[] = {0xFD, 0x16, 0x10, 0x11, 0x23, 0x09};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xFE, 0x00, 0x02, 0x03, 0x21, 0x00, 0x78};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	dsi_vc_dcs_write_0(intel_dsi, 0, 0x35);

	//========== power on setting ==========
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);			//sean test /sleep out
	msleep(10);

	{
		unsigned char data[] = {0xC3, 0x40, 0X00, 0X28};//sean test /enable power IC
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);
	}
	msleep(10);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);			//sean test /display on
	msleep(10);
	//==============END======================

	sean_debug("%s:----sean test----m181_ivo_send_otp_cmds----%d\n", __func__,__LINE__);
	msleep(10);
}

void ivo_m181_enable(struct intel_dsi_device *dsi)
{
	sean_debug("%s:----sean test----m181_ivo_enable----\n", __func__);
}

void ivo_m181_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
	sean_debug("%s:----sean test----m181_ivo_panel_disable----\n", __func__);

	//========== power off setting ==========
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);			//sean test /display off
	msleep(10);

	{
		unsigned char data[] = {0xC3, 0x40, 0x00,0x20};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);	//sean test /disable power IC
	}
	msleep(10);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);			//sean test /sleep in
	msleep(10);
	//================= END ==================
}

enum drm_connector_status ivo_m181_detect(struct intel_dsi_device *dsi)
{
	sean_debug("%s:----sean test----m181_ivo_detect----\n", __func__);
	return connector_status_connected;
}

bool ivo_m181_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *ivo_m181_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode;
	sean_debug("%s:----sean test----m181_ivo_get_modes----\n", __func__);
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 800;			//sean test
	mode->hsync_start = 816;		//sean test	HFP = 16
	mode->hsync_end = 832;			//sean test	HW = 16
	mode->htotal = 880;				//sean test	HBP = 48


	mode->vdisplay = 1280;			//sean test
	mode->vsync_start = 1288;		//sean test	VFP	= 8
	mode->vsync_end = 1292;			//sean test	VW	= 4
	mode->vtotal = 1296;			//sean test	VBP	= 4

	mode->vrefresh = 60;

	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

void ivo_m181_dump_regs(struct intel_dsi_device *dsi) { }

void ivo_m181_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops ivo_m181_dsi_display_ops = {
	.init = ivo_m181_init,
	.get_info = ivo_m181_get_panel_info,
	.create_resources = ivo_m181_create_resources,
	.dpms = ivo_m181_dpms,
	.mode_valid = ivo_m181_mode_valid,
	.mode_fixup = ivo_m181_mode_fixup,
	.panel_reset = ivo_m181_panel_reset,
	.disable_panel_power = ivo_m181_disable_panel_power,
	.send_otp_cmds = ivo_m181_send_otp_cmds,
	.enable = ivo_m181_enable,
	.disable = ivo_m181_disable,
	.detect = ivo_m181_detect,
	.get_hw_state = ivo_m181_get_hw_state,
	.get_modes = ivo_m181_get_modes,
	.destroy = ivo_m181_destroy,
	.dump_regs = ivo_m181_dump_regs,
};
