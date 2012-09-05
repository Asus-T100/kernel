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
 * Gideon Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
 * Austin Hu <austin.hu@intel.com>
 */

#include "displays/gi_sony_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"

#define GPIO_MIPI_PANEL_RESET 128

/* ************************************************************************* *\
 * FUNCTION: mdfld_dsi_gi_sony_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
/* [SC1] Setting */
static u32 gi_sony_enter_sleep_mode[] = {0x00000010};
static u32 gi_sony_exit_sleep_mode[] = {0x00000011};
static u32 gi_sony_set_brightness[] = {0x00000051};
static u32 gi_sony_select_CABC_mode[] = {0x00000355};
static u32 gi_sony_enable_CABC_bl_off[] = {0x00002853};
static u32 gi_sony_enable_CABC_bl_on[] = {0x00002C53};
static u32 gi_sony_set_display_on[] = {0x00000029};
static u32 gi_sony_set_display_off[] = {0x00000028};

/* ************************************************************************* *\
 * FUNCTION: mdfld_gi_l5f3_dbi_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */

static u32 gi_l5f3_set_column_add[] = {0x0100002a, 0x0000003f};
static u32 gi_l5f3_set_row_add[] = {0x0100002b, 0x000000df};
static u32 gi_l5f3_set_address_mode[] = {0x00000036};
static u32 gi_l5f3_set_pixel_format[] = {0x0000773a};
static u32 gi_l5f3_set_te_scanline[] = {0x00000044};
static u32 gi_l5f3_set_tear_on[] = {0x00000035};
static u32 gi_l5f3_passwd1_on[] = {0x005a5af0};
static u32 gi_l5f3_passwd2_on[] = {0x005a5af1};
static u32 gi_l5f3_dstb_on[] = {0x000001df};
static u32 gi_l5f3_set_disctl[] = {0x0f4a3bf2, 0x08081004, 0x00080800,
	0x4c000000, 0x20201004};
static u32 gi_l5f3_set_pwrctl[] = {0x000007f4, 0x00000000, 0x05440000,
	0x00054400};
static u32 gi_l5f3_set_vcmctl[] = {0x171500f5, 0x00020000, 0x15000000,
	0x00000017};
static u32 gi_l5f3_set_srcctl[] = {0x080001f6, 0x01000103, 0x00000000};
static u32 gi_l5f3_set_ifctl[] = {0x108198f7, 0x00000003};
/* static u32 gi_l5f3_set_ifctl[] = {0x108148f7, 0x00000003}; */
static u32 gi_l5f3_set_panelctl[] = {0x000055f8};
static u32 gi_l5f3_set_gammasel[] = {0x000027f9};
static u32 gi_l5f3_set_pgammactl[] = {0x06040cfa, 0x24211f1e, 0x2e2f2d21,
	0x00000f2e, 0x00000000};
static u32 gi_l5f3_set_ngammactl[] = {0x0f040cfb, 0x2d2f2e2e, 0x1f212421,
	0x0000061e, 0x00000000};
static u32 gi_l5f3_set_miectl1[] = {0x108080c0};
static u32 gi_l5f3_set_bcmode[] = {0x000013c1};
static u32 gi_l5f3_set_wrmiectl2[] = {0x000008c2, 0x0000df01, 0x00003f01};
static u32 gi_l5f3_set_wrblctl[] = {0x201000c3};
static u32 gi_l5f3_passwd1_off[] = {0x00a5a5f0};

static u32 gi_l5f3_set_full_brightness[] = {0x0000ff51};
static u32 gi_l5f3_turn_on_backlight[] = {0x00002453};
static u32 gi_l5f3_disable_cabc[] = {0x00000055};
static u32 gi_l5f3_exit_sleep_mode[] = {0x00000011};

/* FIXME Optimize the delay time after PO.  */
static int mdfld_gi_l5f3_dpi_ic_init(struct mdfld_dsi_config *dsi_config,
		int pipe)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_column_add, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_row_add, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_address_mode, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_pixel_format, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_te_scanline, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_tear_on, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_passwd1_on, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_disctl, 20, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_pwrctl, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_vcmctl, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_srcctl, 12, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_ifctl, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_panelctl, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_gammasel, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_pgammactl, 20, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_ngammactl, 20, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_miectl1, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_bcmode, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_wrmiectl2, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_wrblctl, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_passwd1_off, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_turn_on_backlight, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_disable_cabc, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_exit_sleep_mode, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	return 0;
}

static void
mdfld_dsi_gi_sony_dsi_controller_init(struct mdfld_dsi_config *dsi_config,
				int pipe, int update)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	int lane_count = dsi_config->lane_count;
	u32 mipi_control_reg = dsi_config->regs.mipi_control_reg;
	u32 intr_en_reg = dsi_config->regs.intr_en_reg;
	u32 hs_tx_timeout_reg = dsi_config->regs.hs_tx_timeout_reg;
	u32 lp_rx_timeout_reg = dsi_config->regs.lp_rx_timeout_reg;
	u32 turn_around_timeout_reg =
		dsi_config->regs.turn_around_timeout_reg;
	u32 device_reset_timer_reg =
		dsi_config->regs.device_reset_timer_reg;
	u32 high_low_switch_count_reg =
		dsi_config->regs.high_low_switch_count_reg;
	u32 init_count_reg = dsi_config->regs.init_count_reg;
	u32 eot_disable_reg = dsi_config->regs.eot_disable_reg;
	u32 lp_byteclk_reg = dsi_config->regs.lp_byteclk_reg;
	u32 clk_lane_switch_time_cnt_reg =
		dsi_config->regs.clk_lane_switch_time_cnt_reg;
	u32 video_mode_format_reg =
		dsi_config->regs.video_mode_format_reg;
	u32 dphy_param_reg = dsi_config->regs.dphy_param_reg;
	u32 dsi_func_prg_reg = dsi_config->regs.dsi_func_prg_reg;

	PSB_DEBUG_ENTRY("%s: initializing dsi controller on pipe %d\n",
			__func__, pipe);

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xff;
	hw_ctx->high_low_switch_count = 0x25;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150a600f;

	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | lane_count);
}

static int mdfld_dsi_gi_sony_detect(struct mdfld_dsi_config *dsi_config,
				int pipe)
{
	int status;

	printk(KERN_ALERT"%s\n", __func__);

	if (pipe == 0) {
		/*reconfig lane configuration*/
		/* [SC1] in bb2 is 3 */
		dsi_config->lane_count = 1;
		/* [SC1] in bb2 is MDFLD_DSI_DATA_LANE_3_1 */
		/*
		 * FIXME: JLIU7 dsi_config->lane_config =
		 * MDFLD_DSI_DATA_LANE_4_0;
		 */
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
		dsi_config->dsi_hw_context.pll_bypass_mode = 0;
		/* This is for 400 mhz.  Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int mdfld_dsi_gi_sony_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*change power state*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_exit_sleep_mode, 4, 0);

	msleep(120);

	/*enable CABC with backlight off*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_select_CABC_mode, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_off, 4, 0);

	/*set display on*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_set_display_on, 4, 0);

	msleep(21);

	/*enable BLON , CABC*/
	if (1) {
		mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_on,
				4, 0);
		printk(KERN_ALERT "enable SC1 cabc\n");
	}

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	return 0;
}

static int mdfld_dsi_gi_sony_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send SHUT_DOWN packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	/*change power state here*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_set_display_off, 4, 0);

	/*disable BLCON, disable CABC*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_off, 4, 0);
	printk(KERN_ALERT "disable SC1 cabc\n");

	msleep(21);

	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enter_sleep_mode, 4, 0);

	msleep(120);

	/* DSTB, deep standby sequenc */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_passwd2_on, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_dstb_on, 4, 0);
	PSB_DEBUG_ENTRY("putting panel into deep sleep standby\n");

	msleep(50);

	return 0;
}

static int mdfld_dsi_gi_sony_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;
	PSB_DEBUG_ENTRY("Set brightness level %d...\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (255 * level) / 100;

	/*update duty value*/
	gi_sony_set_brightness[0] =  (0x00000051 | (duty_val << 8));
	/* [SC1] change backlight control- brightness */
	mdfld_dsi_send_gen_long_hs(sender, gi_sony_set_brightness, 4, 0);

	return 0;
}

static int mdfld_dsi_gi_sony_panel_reset(struct mdfld_dsi_config *dsi_config,
		int reset_from)
{
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	int ret = 0;
	static bool b_gpio_required[PSB_NUM_PIPE] = {0};
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;

	if (reset_from == RESET_FROM_BOOT_UP) {
		b_gpio_required[dsi_config->pipe] = false;
		if (dsi_config->pipe) {
			PSB_DEBUG_ENTRY(
				"PR2 GPIO reset for MIPIC is skipped!\n");
			goto fun_exit;
		}
		ret = gpio_request(GPIO_MIPI_PANEL_RESET, "gfx");
		if (ret) {
			DRM_ERROR(
			"Failed to request gpio %d\n", GPIO_MIPI_PANEL_RESET);
			goto err;
		}
		b_gpio_required[dsi_config->pipe] = true;

		/* for get date from panel side is not easy,
		so here use display side setting to judge
		wheather panel have enabled or not by FW */
		if ((REG_READ(regs->dpll_reg) & BIT31) &&
			(REG_READ(regs->pipeconf_reg) & BIT30) &&
			(REG_READ(regs->mipi_reg) & BIT31)) {
			PSB_DEBUG_ENTRY(
				"FW has initialized the panel, skip reset during boot up\n.");
			psb_enable_vblank(dev, dsi_config->pipe);

			mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_on, 4, 0);
			mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_miectl1, 4, 0);
			mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_bcmode, 4, 0);
			mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrmiectl2, 4, 0);
			mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrblctl, 4, 0);
			mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_off, 4, 0);
			mdfld_dsi_send_mcs_long_hs(sender, gi_sony_select_CABC_mode, 4, 0);
			mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_off, 4, 0);
			/*set display on*/
			mdfld_dsi_send_mcs_long_hs(sender, gi_sony_set_display_on, 4, 0);
			msleep(21);
			mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_on, 4, 0);

			goto fun_exit;
		}
	}
	if (b_gpio_required[dsi_config->pipe]) {
		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 0);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 0);

		/*reset low level width 11ms*/
		mdelay(10);
		gpio_direction_output(GPIO_MIPI_PANEL_RESET, 1);
		gpio_set_value_cansleep(GPIO_MIPI_PANEL_RESET, 1);

		/*reset time 20ms*/
		mdelay(20);
	} else {
		PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	}
fun_exit:
	if (b_gpio_required[dsi_config->pipe])
		PSB_DEBUG_ENTRY("pr2 panel reset successfull.");
	return 0;
err:
	gpio_free(GPIO_MIPI_PANEL_RESET);
	PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	return 0;
}

struct drm_display_mode *gi_sony_vid_get_config_mode(struct drm_device *dev)
{
	struct drm_display_mode *mode;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	bool use_gct = false; /*Disable GCT for now*/
	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	if (use_gct) {
		PSB_DEBUG_ENTRY("gct find MIPI panel.\n");
		mode->hdisplay = (ti->hactive_hi << 8) | ti->hactive_lo;
		mode->vdisplay = (ti->vactive_hi << 8) | ti->vactive_lo;
		mode->hsync_start = mode->hdisplay + \
				((ti->hsync_offset_hi << 8) | \
				ti->hsync_offset_lo);
		mode->hsync_end = mode->hsync_start + \
				((ti->hsync_pulse_width_hi << 8) | \
				ti->hsync_pulse_width_lo);
		mode->htotal = mode->hdisplay + ((ti->hblank_hi << 8) | \
								ti->hblank_lo);
		mode->vsync_start = \
			mode->vdisplay + ((ti->vsync_offset_hi << 8) | \
						ti->vsync_offset_lo);
		mode->vsync_end = \
			mode->vsync_start + ((ti->vsync_pulse_width_hi << 8) | \
						ti->vsync_pulse_width_lo);
		mode->vtotal = mode->vdisplay + \
				((ti->vblank_hi << 8) | ti->vblank_lo);
		mode->clock = ti->pixel_clock * 10;

		PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
		PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
		PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
		PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
		PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
		PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
		PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
		PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
		PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);
	} else {
		mode->hdisplay = 320;
		mode->vdisplay = 480;
#if 0
		/* HFP = 20, HSYNC = 8, HBP = 20 */
		mode->hsync_start = mode->hdisplay + 20;
		mode->hsync_end = mode->hsync_start + 8;
		mode->htotal = mode->hsync_end + 20;
		/* VFP = 16, VSYNC = 8, VBP = 4 */
		mode->vsync_start = mode->vdisplay + 16;
		mode->vsync_end = mode->vsync_start + 8;
		mode->vtotal = mode->vsync_end + 4;
		mode->clock = 16500;
#else
		/* HFP = 2, HSYNC = 8, HBP = 20 */
		mode->hsync_start = mode->hdisplay + 20;
		mode->hsync_end = mode->hsync_start + 8;
		mode->htotal = mode->hsync_end + 20;
		/* VFP = 4, VSYNC = 4, VBP = 12 */
		mode->vsync_start = mode->vdisplay + 4;
		mode->vsync_end = mode->vsync_start + 4;
		mode->vtotal = mode->vsync_end + 12;
		mode->clock = (mode->htotal * mode->vtotal) * 60 / 1000;
#endif
	}

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static int gi_sony_vid_get_panel_info(struct drm_device *dev,
				int pipe,
				struct panel_info *pi)
{
	if (!dev || !pi)
		return -EINVAL;
	pi->width_mm = PANEL_3DOT47_WIDTH;
	pi->height_mm = PANEL_3DOT47_HEIGHT;

	return 0;
}

void gi_sony_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = &gi_sony_vid_get_config_mode;
	p_funcs->update_fb = NULL;
	p_funcs->get_panel_info = gi_sony_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_gi_sony_panel_reset;
	p_funcs->drv_ic_init = mdfld_gi_l5f3_dpi_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_gi_sony_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_gi_sony_detect;
	p_funcs->power_on = mdfld_dsi_gi_sony_power_on;
	p_funcs->power_off = mdfld_dsi_gi_sony_power_off;
	p_funcs->set_brightness = mdfld_dsi_gi_sony_set_brightness;
}
