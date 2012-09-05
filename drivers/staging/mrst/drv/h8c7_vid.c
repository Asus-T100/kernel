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

#include "displays/h8c7_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include <linux/sfi.h>

/* ************************************************************************* *\
 * FUNCTION: mdfld_h8c7_dpi_ic_init
 *
 * DESCRIPTION:  This function is called only by mrst_dsi_mode_set and
 *               restore_display_registers.  since this function does not
 *               acquire the mutex, it is important that the calling function
 *               does!
\* ************************************************************************* */
static u32 h8c7_exit_sleep_mode[] = {0x00000011};
static u32 h8c7_mcs_protect_off[] = {0x9283ffb9};
static u32 h8c7_set_tear_on[] = {0x00000035};
static u32 h8c7_set_brightness[] = {0x00000051};
static u32 h8c7_set_full_brightness[] = {0x0000ff51};
static u32 h8c7_turn_on_backlight[] = {0x00002453};
static u32 h8c7_disable_cabc[] = {0x00000055};
static u32 h8c7_ic_bias_current[] = {0x826005bf, 0x00000000};
static u32 h8c7_set_power[] = {0x44007cb1, 0x0d0d0024, 0x3f3f1a12, 0x00007242};
static u32 h8c7_set_disp_reg[] = {0x05c80fb2, 0x0084080f, 0x040f05ff, 0x00000020};
static u32 h8c7_set_command_cyc[] = {0x050000b4, 0x1605a000, 0x1603309d,
	0x00030300, 0x0707061b, 0x00000000};
static u32 h8c7_set_mipi_ctrl[] = {0x008312ba};
static u32 h8c7_video_mode[] = {0x000003c2};
static u32 h8c7_set_blanking_opt_2[] = {0x004000c7};
static u32 h8c7_set_panel[] = {0x000008cc};
static u32 h8c7_set_eq_func_ltps[] = {0x00000cd4};
static u32 h8c7_set_ltps_ctrl_output[] = {0x080800d5, 0x66554400, 0xcccccc77,
	0x667700cc, 0xcccc4455, 0x0000cccc};
static u32 h8c7_set_video_cyc[] = {0x040000d8, 0x1604a000, 0x1603309d,
	0x00030300, 0x0707061b, 0x00000000};
static u32 h8c7_gamma_r[] = {0x3c3e3ae0, 0x3332312f, 0x0c080446, 0x110f100d,
	0x3e3a1710, 0x32312f3c, 0x08044633, 0x0f100d0c, 0x00171011};
static u32 h8c7_gamma_g[] = {0x3d3e3be1, 0x33323131, 0x0b070346, 0x110e100d,
	0x3e3b1710, 0x3231313d, 0x07034633, 0x0e100d0b, 0x00171011};
static u32 h8c7_gamma_b[] = {0x070601e2, 0x1f322a2d, 0x0e0c0540, 0x13121411,
	0x0601180f, 0x322a2d07, 0x0c05401f, 0x1214110e, 0x00180f13};
static u32 h8c7_enter_set_cabc[] = {0x1e001fc9, 0x0000001e, 0x00003e01};

static u32 h8c7_mcs_protect_on[] = {0x000000b9};
static u32 h8c7_set_address_mode[] = {0x00000036};
static u32 h8c7_set_pixel_format[] = {0x0000703a};
static u32 h8c7_set_display_on[] = {0x00000029};
static u32 h8c7_set_display_off[] = {0x00000028};
static u32 h8c7_enter_sleep_mode[] = {0x00000010};

#define MIN_BRIGHTNESS_LEVEL 54
#define MAX_BRIGHTNESS_LEVEL 100

static int mdfld_h8c7_dpi_ic_init(struct mdfld_dsi_config *dsi_config, int pipe)
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

	/*wait for 5ms*/
	wait_timeout = jiffies + (HZ / 200);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_exit_sleep_mode, 4, 0);
	wait_timeout = jiffies + (3 * HZ / 20);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set password and wait for 10ms. */
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_off, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set TE on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_tear_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_full_brightness, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set backlight on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_turn_on_backlight, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* disalble CABC and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_disable_cabc, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_ic_bias_current, 8, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_power, 16, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_disp_reg, 16, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_command_cyc, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_mipi_ctrl, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_video_mode, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_blanking_opt_2, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_panel, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_eq_func_ltps, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_ltps_ctrl_output, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_video_cyc, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_r, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_g, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_b, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_enter_set_cabc, 10, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* disable password and wait for 10ms. */
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_address_mode, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_pixel_format, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	return 0;
}

static void
mdfld_h8c7_dpi_controller_init(struct mdfld_dsi_config *dsi_config, int pipe)
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

int mdfld_dsi_h8c7_detect(struct mdfld_dsi_config *dsi_config,
				int pipe)
{
	int status;
	struct drm_device *dev = dsi_config->dev;

	printk(KERN_ALERT"%s\n", __func__);

	if (pipe == 0) {
		/*reconfig lane configuration*/
		dsi_config->lane_count = 3;
		dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
		dsi_config->dsi_hw_context.pll_bypass_mode = 1;
		/* This is for 400 mhz.  Set it to 0 for 800mhz */
		dsi_config->dsi_hw_context.cck_div = 1;

		if (IS_CTP(dev))
			dsi_config->dsi_hw_context.pll_bypass_mode = 0;

		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		PSB_DEBUG_ENTRY("Only support single panel\n");
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int mdfld_dsi_h8c7_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;
	int err;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_exit_sleep_mode, 4, 0);
	wait_timeout = jiffies + (3 * HZ / 20);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/*set display on*/
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_display_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* FIXME Enable CABC later*/

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	return 0;
}

static int mdfld_dsi_h8c7_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;
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

	/* FIXME disable CABC later*/

	/*set display off*/
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_display_off, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* sleep in and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_enter_sleep_mode, 4, 0);
	wait_timeout = jiffies + (3 * HZ / 20);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	return 0;
}

int mdfld_dsi_h8c7_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (255 * level) / 100;
	h8c7_set_brightness[0] = (0x00000051 | (duty_val << 8));

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_brightness, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	return 0;
}

/* MIPI display panel reset GPIO */
static int mdfld_mipi_panel_gpio_reset = -EINVAL;

static int mdfld_mipi_panel_gpio_parse(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb = (struct sfi_table_simple *)table;
	struct sfi_gpio_table_entry *entry;
	int i, num;

	num = SFI_GET_NUM_ENTRIES(sb, struct sfi_gpio_table_entry);
	entry = (struct sfi_gpio_table_entry *)sb->pentry;

	for (i = 0; i < num; i++, entry++) {
		if (!strncmp(entry->pin_name, "mipi-reset", SFI_NAME_LEN))
			mdfld_mipi_panel_gpio_reset = entry->pin_no;
	}

	return 0;
}

int mdfld_dsi_h8c7_panel_reset(struct mdfld_dsi_config *dsi_config,
		int reset_from)
{
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	int ret = 0;
	bool b_gpio_required[PSB_NUM_PIPE] = {0};
	unsigned gpio_mipi_panel_reset = 128;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;

	if (IS_CTP(dev)) {
		sfi_table_parse(SFI_SIG_GPIO, NULL, NULL, mdfld_mipi_panel_gpio_parse);
		gpio_mipi_panel_reset = mdfld_mipi_panel_gpio_reset;
	}

	if (reset_from == RESET_FROM_BOOT_UP) {
		b_gpio_required[dsi_config->pipe] = false;
		if (dsi_config->pipe) {
			PSB_DEBUG_ENTRY(
				"GPIO reset for MIPIC is skipped!\n");
			goto fun_exit;
		}
		ret = gpio_request(gpio_mipi_panel_reset, "gfx");
		if (ret) {
			DRM_ERROR(
			"Failed to request gpio %d\n", gpio_mipi_panel_reset);
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
			goto fun_exit;
		}
	}
	if (b_gpio_required[dsi_config->pipe]) {
		gpio_direction_output(gpio_mipi_panel_reset, 0);
		gpio_set_value_cansleep(gpio_mipi_panel_reset, 0);

		/*reset low level width 11ms*/
		mdelay(10);

		gpio_direction_output(gpio_mipi_panel_reset, 1);
		gpio_set_value_cansleep(gpio_mipi_panel_reset, 1);

		/*reset time 5ms*/
		mdelay(5);
	} else {
		PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	}
fun_exit:
	if (b_gpio_required[dsi_config->pipe])
		PSB_DEBUG_ENTRY("pr2 panel reset successfull.");
	return 0;
err:
	gpio_free(gpio_mipi_panel_reset);
	PSB_DEBUG_ENTRY("pr2 panel reset fail.!");
	return 0;
}

struct drm_display_mode *h8c7_get_config_mode(struct drm_device *dev)
{
	struct drm_display_mode *mode;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
	struct mrst_timing_info *ti = &dev_priv->gct_data.DTD;
	bool use_gct = false; /*Disable GCT for now*/
	if (IS_CTP(dev))
		use_gct = true;

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
		mode->hdisplay = 720;
		mode->vdisplay = 1280;
		mode->hsync_start = 816;
		mode->hsync_end = 824;
		mode->htotal = 920;
		mode->vsync_start = 1284;
		mode->vsync_end = 1286;
		mode->vtotal = 1300;
		mode->vrefresh = 60;
		mode->clock =  mode->vrefresh * mode->vtotal *
				mode->htotal / 1000;
	}

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

int h8c7_get_panel_info(struct drm_device *dev,
				int pipe,
				struct panel_info *pi)
{
	if (!dev || !pi)
		return -EINVAL;

	pi->width_mm = PANEL_4DOT3_WIDTH;
	pi->height_mm = PANEL_4DOT3_HEIGHT;

	return 0;
}

void h8c7_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = &h8c7_get_config_mode;
	p_funcs->update_fb = NULL;
	p_funcs->get_panel_info = h8c7_get_panel_info;
	p_funcs->reset = mdfld_dsi_h8c7_panel_reset;
	p_funcs->drv_ic_init = mdfld_h8c7_dpi_ic_init;
	p_funcs->dsi_controller_init = mdfld_h8c7_dpi_controller_init;
	p_funcs->detect = mdfld_dsi_h8c7_detect;
	p_funcs->power_on = mdfld_dsi_h8c7_power_on;
	p_funcs->power_off = mdfld_dsi_h8c7_power_off;
	p_funcs->set_brightness = mdfld_dsi_h8c7_set_brightness;
}
