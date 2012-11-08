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
 * Jim Liu <jim.liu@intel.com>
 * Jackie Li <yaodong.li@intel.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_pmic.h>
#include <linux/regulator/machine.h>

struct h8c7_regulator_factory {
	bool h8c7_mmc2_on;
	struct regulator *regulator;
};

static struct h8c7_regulator_factory h8c7_regulator_status;

static u8 h8c7_exit_sleep_mode[]     = {0x11};
static u8 h8c7_soft_reset[]          = {0x01};
static u8 h8c7_set_tear_on[]         = {0x35, 0x00};
static u8 h8c7_set_brightness[]      = {0x51, 0x00};
static u8 h8c7_turn_on_backlight[]   = {0x53, 0x24};
static u8 h8c7_turn_off_backlight[]  = {0x53, 0x00};
static u8 h8c7_disable_cabc[]        = {0x55, 0x00};
static u8 h8c7_set_mipi_ctrl[]       = {0xba, 0x12};
static u8 h8c7_command_mode[]        = {0xc2, 0x08};
static u8 h8c7_set_panel[]           = {0xcc, 0x08};
static u8 h8c7_set_eq_func_ltps[]    = {0xd4, 0x0c};
static u8 h8c7_set_address_mode[]    = {0x36, 0x00};
static u8 h8c7_set_te_scanline[]     = {0x44, 0x00, 0x00, 0x00};
static u8 h8c7_set_pixel_format[]    = {0x3a, 0x77};
static u8 h8c7_mcs_protect_off[]     = {0xb9, 0xff, 0x83, 0x92};
static u8 h8c7_mcs_protect_on[]      = {0xb9, 0x00, 0x00, 0x00};
static u8 h8c7_set_blanking_opt_2[]  = {0xc7, 0x00, 0x40, 0x00};
static u8 h8c7_mcs_clumn_addr[]      = {0x2a, 0x00, 0x00, 0x02, 0xcf};
static u8 h8c7_mcs_page_addr[]       = {0x2b, 0x00, 0x00, 0x04, 0xff};
static u8 h8c7_ic_bias_current[] = {
	0xbf, 0x05, 0x60, 0x82,
	0x00, 0x00, 0x00, 0x00};
static u8 h8c7_set_power[] = {
	0xb1, 0x7c, 0x00, 0x44,
	0xa5, 0x00, 0x0d, 0x0d,
	0x12, 0x1a, 0x3f, 0x3f,
	0x42, 0x72};
static u8 h8c7_set_power_dstb[] = {
	0xb1, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 h8c7_set_disp_reg[] = {
	0xb2, 0x0f, 0xc8, 0x05,
	0x0f, 0x04, 0x84, 0x00,
	0xff, 0x05, 0x0f, 0x04,
	0x20};
static u8 h8c7_set_command_cyc[] = {
	0xb4, 0x00, 0x00, 0x05,
	0x00, 0xa0, 0x05, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x06, 0x07, 0x07,
	0x00};
static u8 h8c7_set_ltps_ctrl_output[] = {
	0xd5, 0x00, 0x08, 0x08,
	0x00, 0x44, 0x55, 0x66,
	0x77, 0xcc, 0xcc, 0xcc,
	0xcc, 0x00, 0x77, 0x66,
	0x55, 0x44, 0xcc, 0xcc,
	0xcc, 0xcc, 0x00, 0x00};
static u8 h8c7_set_video_cyc[] = {
	0xd8, 0x00, 0x00, 0x04,
	0x00, 0xa0, 0x04, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x06, 0x07, 0x07,
	0x00};
static u8 h8c7_gamma_r[] = {
	0xe0, 0x3a, 0x3e, 0x3c,
	0x2f, 0x31, 0x32, 0x33,
	0x46, 0x04, 0x08, 0x0c,
	0x0d, 0x10, 0x0f, 0x11,
	0x10, 0x17, 0x3a, 0x3e,
	0x3c, 0x2f, 0x31, 0x32,
	0x33, 0x46, 0x04, 0x08,
	0x0c, 0x0d, 0x10, 0x0f,
	0x11, 0x10, 0x17};
static u8 h8c7_gamma_g[] = {
	0xe1, 0x3b, 0x3e, 0x3d,
	0x31, 0x31, 0x32, 0x33,
	0x46, 0x03, 0x07, 0x0b,
	0x0d, 0x10, 0x0e, 0x11,
	0x10, 0x17, 0x3b, 0x3e,
	0x3d, 0x31, 0x31, 0x32,
	0x33, 0x46, 0x03, 0x07,
	0x0b, 0x0d, 0x10, 0x0e,
	0x11, 0x10, 0x17};
static u8 h8c7_gamma_b[] = {
	0xe2, 0x01, 0x06, 0x07,
	0x2d, 0x2a, 0x32, 0x1f,
	0x40, 0x05, 0x0c, 0x0e,
	0x11, 0x14, 0x12, 0x13,
	0x0f, 0x18, 0x01, 0x06,
	0x07, 0x2d, 0x2a, 0x32,
	0x1f, 0x40, 0x05, 0x0c,
	0x0e, 0x11, 0x14, 0x12,
	0x13, 0x0f, 0x18};
static u8 h8c7_enter_set_cabc[] = {
	0xc9, 0x1f, 0x00, 0x1e,
	0x1e, 0x00, 0x20, 0x00,
	0x01, 0xe3};

/*cabc 20% gain setting*/
static u8 h8c7_set_cabc_gain[] = {
	0xca, 0x28, 0x27, 0x26,
	0x25, 0x24, 0x23, 0x22,
	0x21, 0x20};

#define MIPI_RESET_GPIO_DEFAULT 128

static
int mdfld_h8c7_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender)
		return -EINVAL;

	PSB_DEBUG_ENTRY("\n");
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	/**
	 * soft reset will let panel exit from deep standby mode and
	 * keep at standy mode.
	 */
	mdfld_dsi_send_gen_long_hs(sender, h8c7_soft_reset, 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/*wait for 5ms*/
	mdelay(5);

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_exit_sleep_mode, 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
	mdelay(150);

	/* set password*/
	mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set backlight on*/
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_turn_on_backlight, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* disalble CABC*/
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_ic_bias_current, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_power, 14, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_disp_reg, 13, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_command_cyc, 21, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_mipi_ctrl, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_command_mode, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_blanking_opt_2, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_panel, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_eq_func_ltps, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_ltps_ctrl_output, 24, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_video_cyc, 21, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_gamma_r, 35, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_gamma_g, 35, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_gamma_b, 35, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_enter_set_cabc, 10, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_pixel_format, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_clumn_addr, 5, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_page_addr, 5, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_address_mode, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_te_scanline, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_set_tear_on, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_on, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	return 0;
}

static
void mdfld_h8c7_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 3;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_3_1;
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x00;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x28;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	/* HW team suggested 1390 for bandwidth setting */
	hw_ctx->dbi_bw_ctrl = 1390;
	hw_ctx->dphy_param = 0x20124E1A;
	hw_ctx->dsi_func_prg = (0xa000 | dsi_config->lane_count);
	hw_ctx->mipi = TE_TRIGGER_GPIO_PIN;
	hw_ctx->mipi |= dsi_config->lane_config;
}

static
struct drm_display_mode *h8c7_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->htotal = 920;
	mode->hdisplay = 720;
	mode->hsync_start = 816;
	mode->hsync_end = 824;
	mode->vtotal = 1300;
	mode->vdisplay = 1280;
	mode->vsync_start = 1294;
	mode->vsync_end = 1296;
	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
	PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
	PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
	PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
	PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
	PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
	PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
	PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
	PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
int mdfld_dsi_h8c7_cmd_power_on(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];
	int err = 0;
	int enable_err, enabled = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (!IS_ERR(h8c7_regulator_status.regulator)) {

		if (!h8c7_regulator_status.h8c7_mmc2_on) {
			PSB_DEBUG_ENTRY("Before power on, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
			PSB_DEBUG_ENTRY("Begin to power on\n");
			h8c7_regulator_status.h8c7_mmc2_on = true;
		} else {
			DRM_ERROR("power on several times without off\n");
		}

		enabled = regulator_is_enabled(h8c7_regulator_status.regulator);
		enable_err = regulator_enable(h8c7_regulator_status.regulator);
		if (enable_err < 0) {
			regulator_put(h8c7_regulator_status.regulator);
			DRM_ERROR("FATAL:enable h8c7 regulator error\n");
		}

		/* vemmc2 need 50ms delay due to stability
		** If already enabled, no need to wait for this delay.
		** This code isn't race proof but since in addition to
		** this panel driver only touch driver is enabling this
		** regulator and does it after this function has been
		** finished, this code works well enough for now.
		*/
		if (!enabled)
			msleep(50);
		PSB_DEBUG_ENTRY("After power on, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
	}

	/*clean on-panel FB*/
	/*re-initizlize the te_seq count & set to one*/
	atomic64_set(&sender->te_seq, 1);
	err = mdfld_dsi_send_dcs(sender,
			write_mem_start,
			NULL,
			0,
			CMD_DATA_SRC_PIPE,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("sent write_mem_start faild\n");
		goto power_err;
	}
	/*wait for above write_mem_start happen
	*fifo check will be at the begining of next command
	*/
	mdelay(16);
	/*sleep out*/
	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
			exit_sleep_mode,
			param,
			1,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);

	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", exit_sleep_mode);
		goto power_err;
	}

	/**
	 * must wait 120ms before entering sleep mode.
	 * and wait 5ms before sending next command
	 */
	mdelay(120);

	/*turn on display*/
	param[0] = 0x00;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
		 set_display_on,
		 param,
		 3,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("faild to exit sleep mode\n");
		goto power_err;
	}
	if (drm_psb_enable_cabc) {
		/* turn on cabc */
		h8c7_disable_cabc[1] = 0x3;
		mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc, 4, 0);
		DRM_INFO("%s enable h8c7 cabc\n", __func__);
	}
power_err:
	return err;
}

static int mdfld_dsi_h8c7_cmd_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* turn off cabc */
	h8c7_disable_cabc[1] = 0x0;
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc, 4, 0);

	/*turn off backlight*/
	err = mdfld_dsi_send_mcs_long_hs(sender, h8c7_turn_off_backlight, 4, 0);
	if (err) {
		DRM_ERROR("%s: failed to turn off backlight\n", __func__);
		goto out;
	}
	mdelay(1);

	/*turn off display */
	err = mdfld_dsi_send_dcs(sender,
		 set_display_off,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("sent set_display_off faild\n");
		goto out;
	}

	/*Enter sleep mode */
	err = mdfld_dsi_send_dcs(sender,
			enter_sleep_mode,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);

	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", enter_sleep_mode);
		goto out;
	}

	/**
	 * MIPI spec shows it must wait 5ms
	 * before sneding next command
	 */
	mdelay(5);

	/*enter deep standby mode*/
	err = mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
	if (err) {
		DRM_ERROR("Failed to turn off protection\n");
		goto out;
	}

	err = mdfld_dsi_send_gen_long_hs(sender, h8c7_set_power_dstb, 14, 0);
	if (err)
		DRM_ERROR("Failed to enter DSTB\n");
	mdelay(5);
	mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_on, 4, 0);

out:
	if (!IS_ERR(h8c7_regulator_status.regulator)) {
		if (h8c7_regulator_status.h8c7_mmc2_on) {
			h8c7_regulator_status.h8c7_mmc2_on = false;
			PSB_DEBUG_GENERAL("Begin to power off\n");
		} else
			DRM_ERROR("power off several times without on\n");
		regulator_disable(h8c7_regulator_status.regulator);
		PSB_DEBUG_GENERAL("After power off, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
	}

	return err;
}

static
void h8c7_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = PANEL_4DOT3_WIDTH;
		pi->height_mm = PANEL_4DOT3_HEIGHT;
	}
}

static
int mdfld_dsi_h8c7_cmd_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
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
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int mdfld_dsi_h8c7_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (drm_psb_enable_cabc) {
		if (level < 50) {
			h8c7_disable_cabc[1] = 0x0;
		} else if (level < 66) {
			/* Case 10% */
			h8c7_disable_cabc[1] = 0x3;
			h8c7_set_cabc_gain[1] = 0x23;
			h8c7_set_cabc_gain[2] = 0x23;
			h8c7_set_cabc_gain[3] = 0x23;
			h8c7_set_cabc_gain[4] = 0x22;
			h8c7_set_cabc_gain[5] = 0x22;
			h8c7_set_cabc_gain[6] = 0x22;
			h8c7_set_cabc_gain[7] = 0x21;
			h8c7_set_cabc_gain[8] = 0x21;
			h8c7_set_cabc_gain[9] = 0x20;
		} else if (level < 82) {
			/* Case 20% */
			h8c7_disable_cabc[1] = 0x3;
			h8c7_set_cabc_gain[1] = 0x28;
			h8c7_set_cabc_gain[2] = 0x27;
			h8c7_set_cabc_gain[3] = 0x26;
			h8c7_set_cabc_gain[4] = 0x25;
			h8c7_set_cabc_gain[5] = 0x24;
			h8c7_set_cabc_gain[6] = 0x23;
			h8c7_set_cabc_gain[7] = 0x22;
			h8c7_set_cabc_gain[8] = 0x21;
			h8c7_set_cabc_gain[9] = 0x20;
		} else {
			/* Case 30% */
			h8c7_disable_cabc[1] = 0x3;
			h8c7_set_cabc_gain[1] = 0x2d;
			h8c7_set_cabc_gain[2] = 0x2c;
			h8c7_set_cabc_gain[3] = 0x2b;
			h8c7_set_cabc_gain[4] = 0x29;
			h8c7_set_cabc_gain[5] = 0x27;
			h8c7_set_cabc_gain[6] = 0x25;
			h8c7_set_cabc_gain[7] = 0x23;
			h8c7_set_cabc_gain[8] = 0x21;
			h8c7_set_cabc_gain[9] = 0x20;
		}

		mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc, 4, 0);
		mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_cabc_gain, 10, 0);
	}

	duty_val = (255 * level) / 100;
	h8c7_set_brightness[1] = duty_val;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_brightness, 2, 0);

	return 0;
}

static
int mdfld_dsi_h8c7_cmd_panel_reset(struct mdfld_dsi_config *dsi_config)
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
	mdelay(5);

	return 0;
}

void h8c7_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ena_err;

	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = h8c7_cmd_get_config_mode;
	p_funcs->get_panel_info = h8c7_cmd_get_panel_info;

	/**
	 * because CMI will reset panel in enter_sleep_mode /exit_sleep_mode
	 * so hw reset is not necessary, otherwise, it will conflict
	 */
	p_funcs->reset = NULL;
	p_funcs->drv_ic_init = mdfld_h8c7_drv_ic_init;
	p_funcs->dsi_controller_init = mdfld_h8c7_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_h8c7_cmd_detect;
	p_funcs->power_on = mdfld_dsi_h8c7_cmd_power_on;
	p_funcs->power_off = mdfld_dsi_h8c7_cmd_power_off;
	p_funcs->set_brightness = mdfld_dsi_h8c7_cmd_set_brightness;

	/* Please check the file pmic_avp.c for the correct regulator name */
	h8c7_regulator_status.regulator = regulator_get(NULL, "vemmc2");
	if (IS_ERR(h8c7_regulator_status.regulator)) {
		DRM_ERROR("H8C7 device failed to get mmc regulator\n");
		return ;
	}

	h8c7_regulator_status.h8c7_mmc2_on =
		regulator_is_enabled(h8c7_regulator_status.regulator);

	if (h8c7_regulator_status.h8c7_mmc2_on) {
		ena_err = regulator_enable(h8c7_regulator_status.regulator);
		if (ena_err < 0) {
			regulator_put(h8c7_regulator_status.regulator);
			DRM_ERROR("FATAL:enable h8c7 regulator error\n");
		}
	}
	/*Temperory Work Around to keep VEMMC2 rail ON as current display
	panel doesnt support this */
	ena_err = regulator_enable(h8c7_regulator_status.regulator);
	if (ena_err < 0) {
		regulator_put(h8c7_regulator_status.regulator);
		DRM_ERROR("FATAL:enable h8c7 regulator error in WA\n");
	}
}
