/*
 * Copyright © 2010 Intel Corporation
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
 * Faxing Lu
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_esd.h"
#include <asm/intel_scu_pmic.h>

#include "displays/jdi_cmd.h"

static int mipi_reset_gpio;
static int bias_en_gpio;

static u8 jdi_mcs_clumn_addr[] = {
			0x2a, 0x00, 0x00, 0x02, 0xcf};
static u8 jdi_mcs_page_addr[] = {
			0x2b, 0x00, 0x00, 0x04, 0xff};

static
int jdi_cmd_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}
	err = mdfld_dsi_send_mcs_short_hs(sender,
			exit_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Exit Sleep Mode\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	msleep(130);
	err = mdfld_dsi_send_mcs_short_hs(sender,
			write_display_brightness, 0xff, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Brightness\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			write_ctrl_display, 0x24, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Write Control Display\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			write_ctrl_cabc, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Write Control CABC\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_tear_on, 0x00, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Tear On\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	err = mdfld_dsi_send_mcs_long_hs(sender,
			jdi_mcs_clumn_addr,
			5, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Clumn Address\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	err = mdfld_dsi_send_mcs_long_hs(sender,
			jdi_mcs_page_addr,
			5, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Page Address\n",
		__func__, __LINE__);
		goto ic_init_err;
	}

	return 0;

ic_init_err:
	err = -EIO;
	return err;
}

static
void jdi_cmd_controller_init(
		struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_hw_context *hw_ctx =
				&dsi_config->dsi_hw_context;
#ifdef ENABLE_CSC_GAMMA /*FIXME*/
	struct drm_device *dev = dsi_config->dev;

	struct csc_setting csc = {
			.pipe = 0,
			.type = CSC_REG_SETTING,
			.enable_state = true,
			.data_len = CSC_REG_COUNT,
			.data.csc_reg_data = {
			0xFFB0424, 0xFDF, 0x4320FF1,
			0xFDC, 0xFF50FF5, 0x415}
		 };
	struct gamma_setting gamma = {
		.pipe = 0,
		.type = GAMMA_REG_SETTING,
		.enable_state = true,
		.data_len = GAMMA_10_BIT_TABLE_COUNT,
		.gamma_tableX100 = {
			0x000000, 0x030303, 0x050505, 0x070707,
			0x090909, 0x0C0C0C, 0x0E0E0E, 0x101010,
			0x121212, 0x141414, 0x171717, 0x191919,
			0x1B1B1B, 0x1D1D1D, 0x1F1F1F, 0x212121,
			0x232323, 0x252525, 0x282828, 0x2A2A2A,
			0x2C2C2C, 0x2E2E2E, 0x303030, 0x323232,
			0x343434, 0x363636, 0x383838, 0x3A3A3A,
			0x3C3C3C, 0x3E3E3E, 0x404040, 0x424242,
			0x444444, 0x464646, 0x484848, 0x4A4A4A,
			0x4C4C4C, 0x4E4E4E, 0x505050, 0x525252,
			0x545454, 0x565656, 0x585858, 0x5A5A5A,
			0x5C5C5C, 0x5E5E5E, 0x606060, 0x626262,
			0x646464, 0x666666, 0x686868, 0x6A6A6A,
			0x6C6C6C, 0x6E6E6E, 0x707070, 0x727272,
			0x747474, 0x767676, 0x787878, 0x7A7A7A,
			0x7C7C7C, 0x7E7E7E, 0x808080, 0x828282,
			0x848484, 0x868686, 0x888888, 0x8A8A8A,
			0x8C8C8C, 0x8E8E8E, 0x909090, 0x929292,
			0x949494, 0x969696, 0x989898, 0x999999,
			0x9B9B9B, 0x9D9D9D, 0x9F9F9F, 0xA1A1A1,
			0xA3A3A3, 0xA5A5A5, 0xA7A7A7, 0xA9A9A9,
			0xABABAB, 0xADADAD, 0xAFAFAF, 0xB1B1B1,
			0xB3B3B3, 0xB5B5B5, 0xB6B6B6, 0xB8B8B8,
			0xBABABA, 0xBCBCBC, 0xBEBEBE, 0xC0C0C0,
			0xC2C2C2, 0xC4C4C4, 0xC6C6C6, 0xC8C8C8,
			0xCACACA, 0xCCCCCC, 0xCECECE, 0xCFCFCF,
			0xD1D1D1, 0xD3D3D3, 0xD5D5D5, 0xD7D7D7,
			0xD9D9D9, 0xDBDBDB, 0xDDDDDD, 0xDFDFDF,
			0xE1E1E1, 0xE3E3E3, 0xE4E4E4, 0xE6E6E6,
			0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
			0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
			0xF7F7F7, 0xF9F9F9, 0xFBFBFB, 0xFDFDFD}
	 };
#endif

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 3;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	/* FIXME: enable CSC and GAMMA */
	/*dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;*/
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x0;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->turn_around_timeout = 0x1a;
	hw_ctx->high_low_switch_count = 0x24;
	hw_ctx->clk_lane_switch_time_cnt = 0x240010;
	hw_ctx->lp_byteclk = 0x5;
	hw_ctx->dphy_param = 0x25155b1e;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0xf0;
	hw_ctx->dbi_bw_ctrl = 1390;
	hw_ctx->hs_ls_dbi_enable = 0x0;
	hw_ctx->dsi_func_prg = ((DBI_DATA_WIDTH_OPT2 << 13) |
				dsi_config->lane_count);
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE |
			BANDGAP_CHICKEN_BIT |
			TE_TRIGGER_GPIO_PIN;
	hw_ctx->video_mode_format = 0xf;

#ifdef ENABLE_CSC_GAMMA /*FIXME*/
	if (dsi_config->enable_gamma_csc & ENABLE_CSC) {
		/* setting the tuned csc setting */
		drm_psb_enable_color_conversion = 1;
		mdfld_intel_crtc_set_color_conversion(dev, &csc);
	}

	if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
		/* setting the tuned gamma setting */
		drm_psb_enable_gamma = 1;
		mdfld_intel_crtc_set_gamma(dev, &gamma);
	}
#endif

}
static
int jdi_cmd_panel_connection_detect(
	struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		DRM_INFO("%s: do NOT support dual panel\n",
		__func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int jdi_cmd_power_on(
	struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_address_mode, 0x0, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Address Mode\n",
		__func__, __LINE__);
		goto power_err;
	}
	usleep_range(20000, 20100);

	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_pixel_format, 0x77, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Pixel format\n",
		__func__, __LINE__);
		goto power_err;
	}

	/*turn on display*/
	err = mdfld_dsi_send_dcs(sender,
		 set_display_on,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("faild to set_display_on mode\n");
		goto power_err;
	}
	usleep_range(20000, 20100);

power_err:
	return err;
}

static void __vpro2_power_ctrl(bool on)
{
	u8 addr, value;
	addr = 0xad;
	if (intel_scu_ipc_ioread8(addr, &value))
		DRM_ERROR("%s: %d: failed to read vPro2\n",
		__func__, __LINE__);

	/* Control vPROG2 power rail with 2.85v. */
	if (on)
		value |= 0x1;
	else
		value &= ~0x1;

	if (intel_scu_ipc_iowrite8(addr, value))
		DRM_ERROR("%s: %d: failed to write vPro2\n",
				__func__, __LINE__);
}

static int jdi_cmd_power_off(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_display_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display Off\n",
		__func__, __LINE__);
		goto power_off_err;
	}
	usleep_range(20000, 20100);

	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_tear_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Tear Off\n",
		__func__, __LINE__);
		goto power_off_err;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			enter_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Enter Sleep Mode\n",
		__func__, __LINE__);
		goto power_off_err;
	}

	msleep(60);

	err = mdfld_dsi_send_gen_short_hs(sender,
		access_protect, 4, 2,
		MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Access Protect\n",
		__func__, __LINE__);
		goto power_off_err;
	}

	err = mdfld_dsi_send_gen_short_hs(sender,
		low_power_mode, 1, 2,
		MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Low Power Mode\n",
		__func__, __LINE__);
		goto power_off_err;
	}
	if (bias_en_gpio)
		gpio_set_value_cansleep(bias_en_gpio, 0);
	usleep_range(1000, 1500);
	return 0;
power_off_err:
	err = -EIO;
	return err;
}

static
int jdi_cmd_set_brightness(
		struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 duty_val = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (0xFF * level) / 100;
	mdfld_dsi_send_mcs_short_hs(sender,
			write_display_brightness, duty_val, 1,
			MDFLD_DSI_SEND_PACKAGE);
	return 0;
}

static
void _get_panel_reset_gpio(void)
{
	int ret = 0;
	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("disp0_rst");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio, " \
				  "use default reset pin\n");
			return;
		}
		mipi_reset_gpio = ret;
		ret = gpio_request(mipi_reset_gpio, "mipi_display");
		if (ret) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return;
		}
		gpio_direction_output(mipi_reset_gpio, 0);
	}
}
static
int jdi_cmd_panel_reset(
		struct mdfld_dsi_config *dsi_config)
{
	int ret = 0;
	u8 *vaddr = NULL, *vaddr1 = NULL;
	int reg_value_scl = 0;

	PSB_DEBUG_ENTRY("\n");

	/* Because when reset touchscreen panel, touchscreen will pull i2c bus
	 * to low, sometime this operation will cause i2c bus enter into wrong
	 * status, so before reset, switch i2c scl pin */
	vaddr1 = ioremap(0xff0c1d30, 4);
	reg_value_scl = ioread32(vaddr1);
	reg_value_scl &= ~0x1000;
	iowrite32(reg_value_scl, vaddr1);

	__vpro2_power_ctrl(true);
	usleep_range(2000, 2500);

	if (bias_en_gpio == 0) {
		bias_en_gpio = 189;
		ret = gpio_request(bias_en_gpio, "bias_enable");
		if (ret) {
			DRM_ERROR("Faild to request bias_enable gpio\n");
			return -EINVAL;
		}
		gpio_direction_output(bias_en_gpio, 0);
	}

	_get_panel_reset_gpio();

	gpio_direction_output(bias_en_gpio, 0);
	gpio_direction_output(mipi_reset_gpio, 0);
	gpio_set_value_cansleep(bias_en_gpio, 0);
	gpio_set_value_cansleep(mipi_reset_gpio, 0);
	usleep_range(2000, 2500);
	gpio_set_value_cansleep(bias_en_gpio, 1);
	usleep_range(2000, 2500);
	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	usleep_range(3000, 3500);
	vaddr = ioremap(0xff0c2d00, 0x60);
	iowrite32(0x3221, vaddr + 0x1c);
	usleep_range(2000, 2500);
	iounmap(vaddr);

	/* switch i2c scl pin back */
	reg_value_scl |= 0x1000;
	iowrite32(reg_value_scl, vaddr1);
	iounmap(vaddr1);
	return 0;
}

static
int jdi_cmd_exit_deep_standby(
		struct mdfld_dsi_config *dsi_config)
{
	PSB_DEBUG_ENTRY("\n");

	if (bias_en_gpio)
		gpio_set_value_cansleep(bias_en_gpio, 1);
	_get_panel_reset_gpio();
	gpio_direction_output(mipi_reset_gpio, 0);

	gpio_set_value_cansleep(mipi_reset_gpio, 0);
	usleep_range(1000, 1500);
	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	usleep_range(3000, 3500);
	return 0;
}

static
struct drm_display_mode *jdi_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 720;
	mode->hsync_start = 816;
	mode->hsync_end = 818;
	mode->htotal = 920;

	mode->vdisplay = 1280;
	mode->vsync_start = 1288;
	mode->vsync_end = 1296;
	mode->vtotal = 1304;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void jdi_cmd_get_panel_info(int pipe,
		struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = PANEL_4DOT3_WIDTH;
		pi->height_mm = PANEL_4DOT3_HEIGHT;
	}
}

void jdi_cmd_init(struct drm_device *dev,
		struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");
	p_funcs->reset = jdi_cmd_panel_reset;
	p_funcs->power_on = jdi_cmd_power_on;
	p_funcs->power_off = jdi_cmd_power_off;
	p_funcs->drv_ic_init = jdi_cmd_drv_ic_init;
	p_funcs->get_config_mode = jdi_cmd_get_config_mode;
	p_funcs->get_panel_info = jdi_cmd_get_panel_info;
	p_funcs->dsi_controller_init =
			jdi_cmd_controller_init;
	p_funcs->detect =
			jdi_cmd_panel_connection_detect;
	p_funcs->set_brightness =
			jdi_cmd_set_brightness;
	p_funcs->exit_deep_standby =
			jdi_cmd_exit_deep_standby;

}
