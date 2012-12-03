/*
 * Copyright (c)  2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
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
 * Geng Xiujun <xiujun.geng@intel.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"

#define GPIO_MIPI_PANEL_RESET 128

/* Renesas display command */
static u8 gi_er61529_mcs_protect_on[]	= {0xb0, 0x04};
static u8 gi_er61529_mcs_protect_off[]	= {0xb0, 0x03};
static u8 gi_er61529_mcs_lp_mode_cntr[] = {0xb1, 0x1};
static u8 gi_er61529_mcs_mem_access[]	= {0xb3, 0x02, 0x00, 0x00, 0x00};
static u8 gi_er61529_mcs_panel_driving[] = {
	0xc0, 0x06, 0xdf, 0x40,
	0x13, 0x00, 0x01, 0x00,
	0x55
};
static u8 gi_er61529_disp_timming[] = {
	0xc1, 0x07, 0x28, 0x08,
	0x08, 0x00
};
static u8 gi_er61529_src_timming[] = {
	0xc4, 0x57, 0x00, 0x03,
	0x03
};
static u8 gi_er61529_polarity_setting[] = {0xc6, 0x04};
static u8 gi_er61529_gamma_a[] = {
	0xc8, 0x01, 0x0f, 0x16,
	0x1f, 0x2d, 0x45, 0x40,
	0x2d, 0x1f, 0x17, 0x0d,
	0x08, 0x01, 0x0f, 0x16,
	0x1f, 0x2d, 0x45, 0x40,
	0x2d, 0x1f, 0x17, 0x0d,
	0x08
};
static u8 gi_er61529_gamma_b[] = {
	0xc9, 0x01, 0x0f, 0x16,
	0x1f, 0x2d, 0x45, 0x40,
	0x2d, 0x1f, 0x17, 0x0d,
	0x08, 0x01, 0x0f, 0x16,
	0x1f, 0x2d, 0x45, 0x40,
	0x2d, 0x1f, 0x17, 0x0d,
	0x08
};
static u8 gi_er61529_gamma_c[] = {
	0xca, 0x01, 0x0f, 0x16,
	0x1f, 0x2d, 0x45, 0x40,
	0x2d, 0x1f, 0x17, 0x0d,
	0x08, 0x01, 0x0f, 0x16,
	0x1f, 0x2d, 0x45, 0x40,
	0x2d, 0x1f, 0x17, 0x0d,
	0x08
};
static u8 gi_er61529_power_setting[] = {
	0xd0, 0xa5, 0x06, 0x08,
	0x20, 0x2c, 0x04, 0x01,
	0x00, 0x08, 0x01, 0x00,
	0x06, 0x01, 0x00, 0x00,
	0x20
};
static u8 gi_er61529_vcom_setting[] = {
	0xd1, 0x02, 0x2c, 0x2c,
	0x6f
};
static u8 gi_er61529_set_pixel_format[] = {0x3a, 0x77};
static u8 gi_er61529_set_column_address[] = {
	0x2a, 0x00, 0x00, 0x01,
	0x3f
};
static u8 gi_er61529_set_page_address[] = {
	0x2b, 0x00, 0x00, 0x01,
	0xdf
};
static u8 gi_er61529_set_address_mode[] = {0x36, 0x00};
static u8 gi_er61529_set_ddb_write_cntr[] = {
	0xe1, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00
};
static u8 gi_er61529_nvm_load_cntr[] = {0xe2, 0x80};
static u8 gi_er61529_dcs_exit_sleep_mode[] = {0x11};
static u8 gi_er61529_dcs_set_display_on[] = {0x29};
static u8 gi_dbc_control_on_data[] = {
	0xb8, 0x00, 0x02, 0x02,
	0xf0, 0xf0, 0xc0, 0xc0,
	0x04, 0x1f, 0x90, 0x90,
	0x19, 0x35, 0x62, 0xa3,
	0x00, 0x00, 0x00, 0x00,
	0x00
};
static u8 gi_er61529_enter_sleep_mode[] = {0x10, 0x00, 0x00, 0x00};
static u8 gi_er61529_exit_sleep_mode[]	= {0x11, 0x00, 0x00, 0x00};
static u8 gi_er61529_backlight_cntr[]	= {0xb9, 0x01, 0x00, 0xff, 0x18};
static u8 gi_er61529_set_backlight[]	= {0xb9, 0x01, 0x00, 0xff, 0x18};
static u8 gi_er61529_set_te_scanline[]	= {0x44, 0x00, 0x01};
static u8 gi_er61529_set_tear_on[]	= {0x35, 0x00};
static u8 gi_er61529_backlight_cntr_1[] = {
	0xb8, 0x01, 0x0f, 0x00,
	0xf0, 0x00, 0xc8, 0x00,
	0x08, 0x14, 0x10, 0x00,
	0x37, 0x5a, 0x87, 0xbe,
	0xdc, 0x00, 0x00, 0x00,
	0x00};

static int gi_renesas_dbi_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_on, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_mem_access, 5, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_panel_driving, 9, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_disp_timming, 6, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_src_timming, 5, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_polarity_setting, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_gamma_a, 25, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_gamma_b, 25, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_gamma_c, 25, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_power_setting, 17, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_vcom_setting, 5, 9);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_set_pixel_format, 2, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_set_column_address, 5, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_set_page_address, 5, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_set_address_mode, 2, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_set_te_scanline, 3, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_set_tear_on, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_off, 2, 0);

	return 0;
}

static void
gi_renesas_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	dsi_config->lane_count = 1;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	hw_ctx->pll_bypass_mode = 1;
	hw_ctx->cck_div = 1;
	hw_ctx->mipi_control = 0x00;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x28;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x0;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150a600f;
	hw_ctx->dbi_bw_ctrl = 0x820;
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | TE_TRIGGER_GPIO_PIN;
	hw_ctx->mipi |= dsi_config->lane_config;
	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0xa000 | dsi_config->lane_count);
}

static
struct drm_display_mode *gi_renesas_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 320;
	mode->vdisplay = 480;
	/* HFP = 10, HSYNC = 10, HBP = 20 */
	mode->hsync_start = mode->hdisplay + 10;
	mode->hsync_end = mode->hsync_start + 10;
	mode->htotal = mode->hsync_end + 20;
	/* VFP = 10, VSYNC = 2, VBP = 20 */
	mode->vsync_start = mode->vdisplay + 10;
	mode->vsync_end = mode->vsync_start + 2;
	mode->vtotal = mode->vsync_end + 10;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static
int __gi_renesas_dsi_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_hw_registers *regs =
		&dsi_config->regs;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (drm_psb_enable_cabc) {
		/* enable cabc */
		gi_er61529_backlight_cntr_1[1] = 0x01;
		mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_on, 2, 0);
		mdfld_dsi_send_gen_long_hs(sender, gi_er61529_backlight_cntr_1, 21, 0);
		mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_off, 2, 0);
	}

	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_on, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_backlight_cntr, 5, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_off, 2, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_exit_sleep_mode, 1, 0);
	mdelay(120);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_set_tear_on, 2, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_dcs_set_display_on, 1, 0);

	return err;
}

static
int __gi_renesas_dsi_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* turn off display */
	err = mdfld_dsi_send_dcs(sender,
		 set_display_off,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent set_display_off faild\n", __func__);
		goto power_err;
	}
	mdelay(70);

	/* set tear off display */
	err = mdfld_dsi_send_dcs(sender,
		 set_tear_off,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent set_tear_off faild\n", __func__);
		goto power_err;
	}

	/* disable CABC */
	gi_er61529_backlight_cntr_1[1] = 0x00;
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_on, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_backlight_cntr_1, 21, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_off, 2, 0);

	err =
	mdfld_dsi_send_mcs_long_hs(sender, gi_er61529_enter_sleep_mode, 1, 0);
	if (err) {
		DRM_ERROR("Enter sleep mode error\n");
		goto power_err;
	}
	mdelay(120);

	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_on, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_lp_mode_cntr, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_off, 2, 0);

power_err:
	return err;
}

static
void gi_renesas_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	if (pipe == 0) {
		pi->width_mm = PANEL_3DOT47_WIDTH;
		pi->height_mm = PANEL_3DOT47_HEIGHT;
	}
}

static
int gi_renesas_dsi_cmd_detect(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	int status;
	int pipe = dsi_config->pipe;
	uint32_t dpll_val, device_ready_val;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
				(dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			status = MDFLD_DSI_PANEL_CONNECTED;
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			status = MDFLD_DSI_PANEL_DISCONNECTED;
			DRM_INFO("%s: do NOT support dual panel\n", __func__);
		}

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
		dsi_config->dsi_hw_context.panel_on = 0;
	}

	return 0;
}

static
int gi_renesas_dsi_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 backlight_val;

	PSB_DEBUG_ENTRY("Set brightness level %d...\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	backlight_val = level * 255 / 100;
	gi_er61529_set_backlight[2] = backlight_val;

	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_on, 2, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_set_backlight, 5, 0);
	mdfld_dsi_send_gen_long_hs(sender, gi_er61529_mcs_protect_off, 2, 0);

	return 0;
}

static
int gi_renesas_dsi_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	static int mipi_reset_gpio;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("mipi-reset");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio, " \
				  "use default reset pin\n");
			ret = 128;
		}

		mipi_reset_gpio = ret;

		ret = gpio_request(mipi_reset_gpio, "mipi_display");
		if (ret) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return -EINVAL;
		}

		gpio_direction_output(mipi_reset_gpio, 0);
	}

	gpio_set_value_cansleep(mipi_reset_gpio, 0);
	mdelay(11);

	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	mdelay(20);

	return 0;
}


void gi_renesas_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	p_funcs->get_config_mode = gi_renesas_cmd_get_config_mode;
	p_funcs->get_panel_info = gi_renesas_cmd_get_panel_info;
	p_funcs->reset = gi_renesas_dsi_panel_reset;
	p_funcs->drv_ic_init = gi_renesas_dbi_ic_init;
	p_funcs->dsi_controller_init = gi_renesas_dsi_controller_init;
	p_funcs->detect = gi_renesas_dsi_cmd_detect;
	p_funcs->set_brightness = gi_renesas_dsi_cmd_set_brightness;
	p_funcs->power_on = __gi_renesas_dsi_power_on;
	p_funcs->power_off = __gi_renesas_dsi_power_off;
}
