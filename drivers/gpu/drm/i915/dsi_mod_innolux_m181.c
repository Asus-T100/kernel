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

static void m181_get_panel_info(int pipe, struct drm_connector *connector)
{
	sean_debug("%s:----sean test----m181_innolux_panel_get_info----\n", __func__);
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = M181_10x7_PANEL_WIDTH;
		connector->display_info.height_mm = M181_10x7_PANEL_HEIGHT;
	}

	return;
}

bool m181_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	sean_debug("%s:----sean test----m181_innolux_panel_init----\n", __func__);
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
	intel_dsi->hs_to_lp_count = 0x20; //sean test
	intel_dsi->lp_byte_clk = 0x3; //sean test
	intel_dsi->bw_timer = 0x0;
	intel_dsi->clk_lp_to_hs_count = 0x1F; //sean test
	intel_dsi->clk_hs_to_lp_count = 0x0E; //sean test
	intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
	intel_dsi->dphy_reg = 0x3710390B; //sean test
	intel_dsi->pclk = 72680;
	intel_dsi->burst_mode_ratio = 100; //liwei add


	intel_dsi->backlight_off_delay = 20;	//keep data more than 8 frames in 60HZ
	intel_dsi->send_shutdown = true;
	intel_dsi->backlight_on_delay = 134;	//sean test
	intel_dsi->shutdown_pkt_delay = 134;	//sean test

	intel_dsi->clock_stop = true;	//seantest no continue more

	return true;
}

void m181_create_resources(struct intel_dsi_device *dsi) { }

void m181_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	sean_debug("%s:----sean test----m181_innolux_dpms----\n", __func__);
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

int m181_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool m181_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void m181_panel_reset(struct intel_dsi_device *dsi)
{
	/*
	int err;
	sean_debug("%s:----sean test----m181_panel_reset----\n", __func__);

	err = gpio_request(69, "sd_pwr_en");
	if (err){
		printk("%s----sean test----gpio_requset_fail----\n",__func__);
	}

	gpio_direction_output(69, 1);
	usleep_range(10000,15000);
	gpio_set_value(69, 0);
	usleep_range(10000,15000);
	gpio_set_value(69, 1);

	gpio_free(69);
	msleep(300);
	*/

}

void m181_disable_panel_power(struct intel_dsi_device *dsi)
{
	int err;
	err = gpio_request(69, "sd_pwr_en1");
	if (err){
		printk("%s:----sean test----m181_panel_request fail----\n",__func__);
	}

	msleep(100);	//sean test t12 >= 50

	gpio_direction_output(69, 0);
	gpio_set_value(69, 0);	//RESX low
	//sean_debug("%s:----sean test----ivo_m181_disable_panel_power----%d\n", __func__,__LINE__);
	//gpio_free(69);

	//msleep(5);	//sean test t13 =< 50

	//sean_debug("%s:----sean test----ivo_m181_disable_panel_power----%d\n", __func__,__LINE__);
	intel_mid_pmic_writeb(0x52,0x00);//PANEL_EN 3.3v
	usleep_range(16000,17000); //sean test t15 <= 10
	intel_mid_pmic_writeb(0x3C,0x24);//GPIOxxxCTLO GPIO1P1 1.8v
	sean_debug("%s:----sean test----panel 1.8V set low----%d\n", __func__,__LINE__);

	err =  intel_mid_pmic_readb(0x52);
	sean_debug("%s:----sean test----ivo_m181_disable_panel_power----%d,3.3v:%d\n", __func__,__LINE__,err);

	gpio_free(69);
	msleep(160);	//sean test t9 >= 150
}

void m181_send_otp_cmds(struct intel_dsi_device *dsi)
{

	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	int err;
	DRM_DEBUG_KMS("\n");
	sean_debug("%s:----sean test----m181_innolux_send_otp_cmds----%d\n", __func__,__LINE__);

	err = gpio_request(69, "sd_pwr_en1");
	if (err){
		printk("%s----sean test----gpio_requset_fail----\n",__func__);
	}

	gpio_direction_output(69, 0);	// sean 2 low pulse
	gpio_set_value(69, 1);
	usleep_range(1000,5000);
	gpio_set_value(69, 0);
	usleep_range(1000,5000); //sean test t7 >= 10
	gpio_set_value(69, 1);
	gpio_free(69);
	msleep(30);	//sean test t5 >= 20

	sean_debug("%s:----sean test----m181_innolux_send_otp_cmds----%d\n", __func__,__LINE__);
	intel_dsi->hs = 0 ;

	//========== Internal setting ==========
	sean_debug("%s:----sean test----m181_innolux_send_otp_cmds----%d\n", __func__,__LINE__);
	{
		unsigned char data[] = {0xFF, 0xAA, 0x55, 0xA5, 0x80};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	{
		unsigned char data[] = {0x6F, 0x11, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	{
		unsigned char data[] = {0xF7, 0x20, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF7, 0xA0);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x19);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF7, 0x12);

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF4, 0x03);//modified by Novatek

	//=============Internal setting END ================
	//========== page0 relative ==========
	sean_debug("%s:----sean test----m181_innolux_send_otp_cmds----%d\n", __func__,__LINE__);
	{
		unsigned char data[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC8, 0x00);

	{
		unsigned char data[] = {0xB1, 0x6C, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB6, 0x08);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x02);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB8, 0x08);

	{
		unsigned char data[] = {0xBB, 0x74, 0x44};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	{
		unsigned char data[] = {0xBC, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	{
		unsigned char data[] = {0xBD, 0x02, 0xB0, 0x0C, 0x0A, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}
	//=================page0 relative END ================
	//========== page1 relative ==========
	{
		unsigned char data[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}
	{
		unsigned char data[] = {0xB0, 0x05, 0x05};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB1, 0x05, 0x05};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBC, 0x90, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBD, 0x90, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xCA, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC0, 0x04);

	{
		unsigned char data[] = {0xB3, 0x37, 0x37};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB4, 0x19, 0x19};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB9, 0x44, 0x44};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBA, 0x24, 0x24};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	//=================page1 relative END ================
	//========== page2 relative ==========
	{
		unsigned char data[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xEE, 0x01);

	{
		unsigned char data[] = {0xEF, 0x09, 0x06, 0x15, 0x18};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	{
		unsigned char data[] = {0xB0, 0x00, 0x00, 0x00, 0x25, 0x00 ,0x43};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);

	{
		unsigned char data[] = {0xB0, 0x00, 0x54, 0x00, 0x68, 0x00 ,0xA0};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x0C);

	{
		unsigned char data[] = {0xB0, 0x00, 0xC0, 0x01, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	{
		unsigned char data[] = {0xB1, 0x01, 0x30, 0x01, 0x78, 0x01 ,0xAE};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);

	{
		unsigned char data[] = {0xB1, 0x02, 0x08, 0x02, 0x50, 0x02 ,0x52};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x0C);

	{
		unsigned char data[] = {0xB1, 0x02, 0x96, 0x02, 0xDC};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	{
		unsigned char data[] = {0xB2, 0x03, 0x08, 0x03, 0x49, 0x03 ,0x77};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x06);

	{
		unsigned char data[] = {0xB2, 0x03, 0xA3, 0x03, 0xAC, 0x03 ,0xC2};
		dsi_vc_dcs_write(intel_dsi, 0, data, 7);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x0C);

	{
		unsigned char data[] = {0xB2, 0x03, 0xC9, 0x03, 0xE3};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	{
		unsigned char data[] = {0xB3, 0x03, 0xFC, 0x03, 0xFF};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	//=================page2 relative END ================
	//========== GOA relative ==========
	//========== page6 relative:GOUT Mapping,VGLO select ==========
	{
		unsigned char data[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}
	{
		unsigned char data[] = {0xB0, 0x00, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB1, 0x12, 0x14};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB2, 0x16, 0x18};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB3, 0x1A, 0x29};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB4, 0x2A, 0x08};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB5, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB6, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB7, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB8, 0x31, 0x0A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB9, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBA, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBB, 0x0B, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBC, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBD, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBE, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xBF, 0x09, 0x2A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC0, 0x29, 0x1B};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC1, 0x19, 0x17};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC2, 0x15, 0x13};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC3, 0x11, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xE5, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC4, 0x09, 0x1B};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC5, 0x19, 0x17};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC6, 0x15, 0x13};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC7, 0x11, 0x29};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC8, 0x2A, 0x01};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC9, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xCA, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xCB, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xCC, 0x31, 0x0B};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xCD, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xCE, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xCF, 0x0A, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD0, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD1, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD2, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD3, 0x00, 0x2A};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD4, 0x29, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD5, 0x12, 0x14};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD6, 0x16, 0x18};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD7, 0x1A, 0x08};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xE6, 0x31, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xD8, 0x00, 0x00, 0x00, 0x54, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}
	{
		unsigned char data[] = {0xD9, 0x00, 0x15, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE7, 0x00);
	//========== GOA relative END ==========
	//========== PAGE3 relative ==========
	{
		unsigned char data[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xB0, 0x20, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	{
		unsigned char data[] = {0xB1, 0x20, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	{
		unsigned char data[] = {0xB2, 0x05, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xB6, 0x05, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xB7, 0x05, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xBA, 0x57, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xBB, 0x57, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xC0, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	{
		unsigned char data[] = {0xC1, 0x00, 0x00, 0x00, 0x00};
		dsi_vc_dcs_write(intel_dsi, 0, data, 5);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC4, 0x60);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC5, 0x40);
	//========== PAGE3 relative END ==========
	//========== PAGE5 relative ==========
	{
		unsigned char data[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xBD, 0x03, 0x01, 0x03, 0x03, 0x03};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	{
		unsigned char data[] = {0xB0, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB1, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB2, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB3, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB4, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xB5, 0x17, 0x06};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB8, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xB9, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xBA, 0x00);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xBB, 0x02);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xBC, 0x00);

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC0, 0x07);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC4, 0x80);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xC5, 0xA4);

	{
		unsigned char data[] = {0xC8, 0x05, 0x30};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xC9, 0x01, 0x31};
		dsi_vc_dcs_write(intel_dsi, 0, data, 3);
	}
	{
		unsigned char data[] = {0xCC, 0x00, 0x00, 0x3C};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);
	}
	{
		unsigned char data[] = {0xCD, 0x00, 0x00, 0x3C};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);
	}

	{
		unsigned char data[] = {0xD1, 0x00, 0x04, 0xFD, 0x07, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}
	{
		unsigned char data[] = {0xD2, 0x00, 0x05, 0x02, 0x07, 0x10};
		dsi_vc_dcs_write(intel_dsi, 0, data, 6);
	}

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE5, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE6, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE7, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE8, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xE9, 0x06);
	dsi_vc_dcs_write_1(intel_dsi, 0, 0xEA, 0x06);

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xED, 0x30);
	//========== PAGE5 relative END==========
	//=============reload setting:reload related setting to internal circuit
	dsi_vc_dcs_write_1(intel_dsi, 0, 0x6F, 0x11);

	dsi_vc_dcs_write_1(intel_dsi, 0, 0xF3, 0x01);
	//=======================END========================
	//====================Normal Display================
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x35);			//sean test 
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);			//sean test /sleep out
	msleep(120);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);			//sean test /display on
	//=======================END========================
	sean_debug("%s:----sean test----m181_innolux_send_otp_cmds----%d\n", __func__,__LINE__);
	msleep(10);
}

void m181_enable(struct intel_dsi_device *dsi)
{
	//struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	sean_debug("%s:----sean test----m181_enable----\n", __func__);

	//DRM_DEBUG_KMS("\n");

	//dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);			//sean test /sleep out
	//msleep(140);									//keep data more than 8 frames in 60HZ:134
	//dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);			//sean test /display on
}

void m181_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
	sean_debug("%s:----sean test----m181_innolux_panel_disable----\n", __func__);
	//========== power off setting ==========
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);			//sean test /display off
	msleep(10);

	{
		unsigned char data[] = {0xC3, 0x40, 0x00,0x20};
		dsi_vc_dcs_write(intel_dsi, 0, data, 4);	//sean test /disable power IC
	}
	msleep(10);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);			//sean test /sleep in
	msleep(120);
	//================= END ==================
}

enum drm_connector_status m181_detect(struct intel_dsi_device *dsi)
{
	sean_debug("%s:----sean test----m181_innolux_detect----\n", __func__);
	return connector_status_connected;
}

bool m181_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *m181_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode;
	sean_debug("%s:----sean test----m181_get_modes----\n", __func__);
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 800;			//sean test
	mode->hsync_start = 850;		//sean test	HFP = 50
	mode->hsync_end = 852;			//sean test	HW = 2
	mode->htotal = 900;				//sean test	HBP = 48


	mode->vdisplay = 1280;			//sean test
	mode->vsync_start = 1312;		//sean test	VFP	= 32
	mode->vsync_end = 1314;			//sean test	VW	= 2
	mode->vtotal = 1346;			//sean test	VBP	= 32

	mode->vrefresh = 60;

	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

void m181_dump_regs(struct intel_dsi_device *dsi) { }

void m181_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops innolux_m181_dsi_display_ops = {
	.init = m181_init,
	.get_info = m181_get_panel_info,
	.create_resources = m181_create_resources,
	.dpms = m181_dpms,
	.mode_valid = m181_mode_valid,
	.mode_fixup = m181_mode_fixup,
	.panel_reset = m181_panel_reset,
	.disable_panel_power = m181_disable_panel_power,
	.send_otp_cmds = m181_send_otp_cmds,
	.enable = m181_enable,
	.disable = m181_disable,
	.detect = m181_detect,
	.get_hw_state = m181_get_hw_state,
	.get_modes = m181_get_modes,
	.destroy = m181_destroy,
	.dump_regs = m181_dump_regs,
};
