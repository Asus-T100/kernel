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
#include "dsi_mod_auo_m181.h"

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

static void auo_m181_get_panel_info(int pipe, struct drm_connector *connector)
{
	DRM_DEBUG_KMS("\n");
	sean_debug("%s:----sean test----m181_auo_panel_get_info----\n", __func__);
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = M181_10x7_PANEL_WIDTH;
		connector->display_info.height_mm = M181_10x7_PANEL_HEIGHT;
	}

	return;
}

bool auo_m181_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
	sean_debug("%s:----sean test----m181_auo_panel_init----\n", __func__);

//	intel_dsi->hs = true;
	intel_dsi->hs = 1;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
//	intel_dsi->eot_disable = 1;
	intel_dsi->eotp_pkt = 0;
	intel_dsi->port_bits = 0;
	//intel_dsi->dsi_clock_freq = 500;
	intel_dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
//	intel_dsi->escape_clk_div = ESCAPE_CLOCK_DIVIDER_1;
//	intel_dsi->lp_rx_timeout = 0xffff;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xff;
//	intel_dsi->init_count = 0x7d0;
	intel_dsi->hs_to_lp_count = 0x46; //sean test
	intel_dsi->lp_byte_clk = 0x3; //sean test
	intel_dsi->bw_timer = 0x0;
	intel_dsi->clk_lp_to_hs_count = 0x22; //sean test
	intel_dsi->clk_hs_to_lp_count = 0x0F; //sean test
	intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
	intel_dsi->dphy_reg = 0x3B113E0C; //sean test
	intel_dsi->pclk = 78312;
	intel_dsi->burst_mode_ratio = 100; //liwei add

	intel_dsi->backlight_off_delay = 50;
	intel_dsi->send_shutdown = true;
	intel_dsi->backlight_on_delay = 50;	//sean test t9 >= 50
	intel_dsi->shutdown_pkt_delay = 50;	//sean test

	intel_dsi->clock_stop = true;	//seantest no continue more

	return true;
}

void auo_m181_create_resources(struct intel_dsi_device *dsi) { }

void auo_m181_dpms(struct intel_dsi_device *dsi, bool enable)
{
	/*
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
	sean_debug("----sean test----m181_auo_dpms----\n");
	if (enable) {

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);
		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);


	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
	*/
}

int auo_m181_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool auo_m181_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void auo_m181_panel_reset(struct intel_dsi_device *dsi)
{

	int err;
	sean_debug("----sean test----auo_m181_panel_reset----\n");

	err = gpio_request(69, "sd_pwr_en1");
	if (err){
		printk("%s:----sean test----m181_panel_reset----\n",__func__);
	}

	gpio_direction_output(69, 0);
	gpio_set_value(69, 1);
	usleep_range(1000,2000);	//sean test t5 >= 5
	gpio_set_value(69, 0);
	usleep_range(1000,2000);	//sean test t6 >= 5
	gpio_set_value(69, 1);
	gpio_free(69);

	msleep(10);

}

void auo_m181_disable_panel_power(struct intel_dsi_device *dsi)
{
	int err;
	err = gpio_request(69, "sd_pwr_en1");
	if (err){
		printk("%s:----sean test----m181_auo_panel_request fail----\n",__func__);
	}

	err =  intel_mid_pmic_readb(0x52);
	sean_debug("%s:----sean test----auo_m181_disable_panel_power----%d,3.3v:%d\n", __func__,__LINE__,err);

	msleep(100);	//sean test t12 >= 50

	gpio_direction_output(69, 0);
	gpio_set_value(69, 0);	//RESX low
	//sean_debug("%s:----sean test----auo_m181_disable_panel_power----%d\n", __func__,__LINE__);
	//gpio_free(69);

	//msleep(5);	//sean test t13 =< 50

	//sean_debug("%s:----sean test----auo_m181_disable_panel_power----%d\n", __func__,__LINE__);
	intel_mid_pmic_writeb(0x52,0x00);//PANEL_EN 3.3v
	usleep_range(16000,17000); //sean test t15 <= 10
	intel_mid_pmic_writeb(0x3C,0x24);//GPIOxxxCTLO GPIO1P1 1.8v
	sean_debug("%s:----sean test----panel 1.8V set low----%d\n", __func__,__LINE__);

	err =  intel_mid_pmic_readb(0x52);
	sean_debug("%s:----sean test----auo_m181_disable_panel_power----%d,3.3v:%d\n", __func__,__LINE__,err);

	gpio_free(69);

	msleep(500);	//sean test t17 >= 500
}

void auo_m181_send_otp_cmds(struct intel_dsi_device *dsi)
{

	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	/*
	int err;
	DRM_DEBUG_KMS("\n");
	sean_debug("%s:----sean test----m181_send_otp_cmds----%d\n", __func__,__LINE__);

	err = gpio_request(69, "sd_pwr_en1");
	if (err){
		printk("%s:----sean test----m181_panel_reset----\n",__func__);
	}

	gpio_direction_output(69, 0);
	gpio_set_value(69, 1);
	usleep_range(1000,2000);	//sean test t5 >= 5
	gpio_set_value(69, 0);
	usleep_range(1000,2000);	//sean test t6 >= 5
	gpio_set_value(69, 1);
	gpio_free(69);
	msleep(10);*/

	sean_debug("%s:----sean test----m181_send_otp_cmds----%d\n", __func__,__LINE__);
	intel_dsi->hs = 0 ;
	msleep(10);	//sean test t7 >= 5

	//========== Internal setting ==========
	sean_debug("%s:----sean test----m181_send_otp_cmds----%d\n", __func__,__LINE__);

	{
		unsigned char data[] = {0xF0, 0x5A, 0x5A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);	//sean test /password
	}

	msleep(5);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);			//sean test /sleep out
	msleep(5);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);			//sean test /display on
	msleep(5);

	{
		unsigned char data[] = {0xC3, 0x40, 0x00,0x28};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);	//sean test /enable power IC
	}
	//================= END ================
	msleep(190);//send initial code to chip delay min:190
}

void auo_m181_enable(struct intel_dsi_device *dsi)
{

	sean_debug("----sean test----m181_auo_panel_enable----\n");
/*
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	//DRM_DEBUG_KMS("\n");
	sean_debug("----sean test----m181_auo_panel_enable----\n");

	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);			//sean test /sleep out
	//msleep(120);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);			//sean test /display on
*/
}

void auo_m181_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	sean_debug("----sean test----m181_auo_panel_disable----\n");
	//========== power off setting ==========
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);			//sean test /display off
	msleep(10);

	{
		unsigned char data[] = {0xC3, 0x40, 0x00,0x20};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);	//sean test /disable power IC
	}
	//msleep(100);	//sean test t11 >= 100
	msleep(10);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);			//sean test /sleep in
	msleep(10);
	//================= END ==================
}

enum drm_connector_status auo_m181_detect(struct intel_dsi_device *dsi)
{
	sean_debug("%s:----sean test----auo_m181_detect----\n", __func__);
	return connector_status_connected;
}

bool auo_m181_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *auo_m181_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode;
	sean_debug("%s:----sean test----auo_m181_get_modes----\n", __func__);
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 800;			//sean test
	mode->hsync_start = 824;		//sean test	HFP = 24
	mode->hsync_end = 828;			//sean test	HW = 4
	mode->htotal = 1004;			//sean test	HBP = 176


	mode->vdisplay = 1280;			//sean test
	mode->vsync_start = 1288;		//sean test	VFP	= 8
	mode->vsync_end = 1292;			//sean test	VW	= 4
	mode->vtotal = 1300;			//sean test	VBP	= 8

	mode->vrefresh = 60;

	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

void auo_m181_dump_regs(struct intel_dsi_device *dsi) { }

void auo_m181_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops auo_m181_dsi_display_ops = {
	.init = auo_m181_init,
	.get_info = auo_m181_get_panel_info,
	.create_resources = auo_m181_create_resources,
	.dpms = auo_m181_dpms,
	.mode_valid = auo_m181_mode_valid,
	.mode_fixup = auo_m181_mode_fixup,
	.panel_reset = auo_m181_panel_reset,
	.disable_panel_power = auo_m181_disable_panel_power,
	.send_otp_cmds = auo_m181_send_otp_cmds,
	.enable = auo_m181_enable,
	.disable = auo_m181_disable,
	.detect = auo_m181_detect,
	.get_hw_state = auo_m181_get_hw_state,
	.get_modes = auo_m181_get_modes,
	.destroy = auo_m181_destroy,
	.dump_regs = auo_m181_dump_regs,
};
