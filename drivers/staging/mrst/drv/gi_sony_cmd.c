/*
 * Copyright (c)  2010 Intel Corporation
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
 * Jim Liu <jim.liu@intel.com>
 * Jackie Li <yaodong.li@intel.com>
 * Gideon Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
 * Austin Hu <austin.hu@intel.com>
*/

#include "displays/gi_sony_cmd.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"

#define GPIO_MIPI_PANEL_RESET 128

enum delay_type {
	MSLEEP,
	JIFFIES_TIMEOUT,
	MDELAY
};


/* delay */
static void mdfld_ms_delay(enum delay_type d_type, int delay)
{
	unsigned long wait_timeout;

	switch (d_type) {
	case MSLEEP:
		msleep(delay);
		break;
	case JIFFIES_TIMEOUT:
		wait_timeout = jiffies + (HZ * delay / 1000);
		while (time_before_eq(jiffies, wait_timeout))
			cpu_relax();
		break;
	case MDELAY:
		mdelay(delay);
		break;
	}
}

static u8 gi_l5f3_set_column_add[] = {
	0x2a, 0x00, 0x00, 0x01,
	0x3f, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_row_add[] = {0x2b, 0x00, 0x00, 0x01,
	0xdf, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_address_mode[] = {0x36, 0xd0, 0x00, 0x00};
static u8 gi_l5f3_set_pixel_format[] = {0x3a, 0x77, 0x00, 0x00};
static u8 gi_l5f3_set_te_scanline[] = {0x44, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_tear_on[] = {0x35, 0x00, 0x00, 0x00};
static u8 gi_l5f3_passwd1_on[] = {0xf0, 0x5a, 0x5a, 0x00};
static u8 gi_l5f3_passwd2_on[] = {0xf1, 0x5a, 0x5a, 0x00};
static u8 gi_l5f3_dstb_on[] = {0xdf, 0x01, 0x00, 0x00};
static u8 gi_l5f3_set_disctl[] = {
	0xf2, 0x3b, 0x55, 0x0f,
	0x04, 0x02, 0x08, 0x08,
	0x00, 0x08, 0x08, 0x00,
	0x00, 0x00, 0x00, 0x55,
	0x04, 0x02, 0x04, 0x02};
static u8 gi_l5f3_set_pwrctl[] = {
	0xf4, 0x07, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x44, 0x05,
	0x00, 0x44, 0x05, 0x00};
static u8 gi_l5f3_set_vcmctl[] = {
	0xf5, 0x00, 0x15, 0x17,
	0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x15,
	0x17, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_srcctl[] = {
	0xf6, 0x01, 0x00, 0x08,
	0x03, 0x01, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_ifctl[] = {
	0xf7, 0x48, 0x80, 0x10,
	0x03, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_panelctl[] = {0xf8, 0x55, 0x00, 0x00};
static u8 gi_l5f3_set_gammasel[] = {0xf9, 0x27, 0x00, 0x00};
static u8 gi_l5f3_set_pgammactl[] = {
	0xfa, 0x0c, 0x04, 0x06,
	0x1e, 0x1f, 0x21, 0x24,
	0x21, 0x2d, 0x2f, 0x2e,
	0x2e, 0x0f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_ngammactl[] = {
	0xfb, 0x0c, 0x04, 0x0f,
	0x2e, 0x2e, 0x2f, 0x2d,
	0x21, 0x24, 0x21, 0x1f,
	0x1e, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_miectl1[] = {0xc0, 0x80, 0x80, 0x10};
static u8 gi_l5f3_set_bcmode[] = {0xc1, 0x13, 0x00, 0x00};
static u8 gi_l5f3_set_wrmiectl2[] = {
	0xc2, 0x08, 0x00, 0x00,
	0x01, 0xdf, 0x00, 0x00,
	0x01, 0x3f, 0x00, 0x00};
static u8 gi_l5f3_set_wrblctl[] = {0xc3, 0x00, 0x10, 0x20};
static u8 gi_l5f3_passwd1_off[] = {0xf0, 0xa5, 0xa5, 0x00};
static u8 gi_l5f3_turn_on_backlight[] = {0x53, 0x24, 0x00, 0x00};
static u8 gi_l5f3_disable_cabc[] = {0x55, 0x00, 0x00, 0x00};
static u8 gi_l5f3_exit_sleep_mode[] = {0x11, 0x00, 0x00, 0x00};

/* FIXME Optimize the delay time after PO.  */
static
int mdfld_gi_l5f3_dbi_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	/*wait for 5ms*/
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_column_add, 8, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_row_add, 8, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_address_mode, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_pixel_format, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set TE scanline and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_set_te_scanline, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set password on and wait for 10ms. */
	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_on, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_disctl, 20, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_pwrctl, 16, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_vcmctl, 16, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_srcctl, 12, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_ifctl, 8, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_panelctl, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_gammasel, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_pgammactl, 20, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_ngammactl, 20, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_miectl1, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_bcmode, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrmiectl2, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_set_wrblctl, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	mdfld_dsi_send_gen_long_hs(sender, gi_l5f3_passwd1_off, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* set backlight on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_turn_on_backlight, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	/* disalble CABC and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_disable_cabc, 4, 0);
	mdfld_ms_delay(MSLEEP, 5);

	dsi_config->drv_ic_inited = 1;

	return 0;
}

static void
mdfld_gi_sony_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/* reconfig lane configuration */
	/* [SC1] in bb2 is 3 */
	dsi_config->lane_count = 1;
	/* [SC1] in bb2 is MDFLD_DSI_DATA_LANE_3_1 */
	/*
	 * FIXME: JLIU7 dsi_config->lane_config =
	 * MDFLD_DSI_DATA_LANE_4_0;
	 */
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	hw_ctx->pll_bypass_mode = 1;
	/* This is for 400 mhz. Set it to 0 for 800mhz */
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
struct drm_display_mode *gi_sony_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 320;
	mode->vdisplay = 480;
	/* HFP = 20, HSYNC = 8, HBP = 20 */
	mode->hsync_start = mode->hdisplay + 20;
	mode->hsync_end = mode->hsync_start + 8;
	mode->htotal = mode->hsync_end + 20;
	/* VFP = 32, VSYNC = 8, VBP = 32 */
	mode->vsync_start = mode->vdisplay + 32;
	mode->vsync_end = mode->vsync_start + 8;
	mode->vtotal = mode->vsync_end + 32;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static
int __mdfld_gi_sony_dsi_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (drm_psb_enable_cabc) {

		param[0] = 0x03;
		err = mdfld_dsi_send_dcs(sender,
			 write_ctrl_cabc,
			 param,
			 1,
			 CMD_DATA_SRC_SYSTEM_MEM,
			 MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s - write_ctrl_cabc faild\n", __func__);
			goto power_err;
		}

		param[0] = 0x28;
		err = mdfld_dsi_send_dcs(sender,
			 write_ctrl_display,
			 param,
			 1,
			 CMD_DATA_SRC_SYSTEM_MEM,
			 MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s - write_ctrl_display faild\n", __func__);
			goto power_err;
		}

		param[0] = 0x2c;
		err = mdfld_dsi_send_dcs(sender,
			 write_ctrl_display,
			 param,
			 1,
			 CMD_DATA_SRC_SYSTEM_MEM,
			 MDFLD_DSI_SEND_PACKAGE);
		if (err) {
			DRM_ERROR("%s - write_ctrl_display faild\n", __func__);
			goto power_err;
		}
		DRM_INFO("%s enable lxt cabc\n", __func__);
	}

	err = mdfld_dsi_send_dcs(sender,
		 exit_sleep_mode,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", exit_sleep_mode);
		goto power_err;
	}

	msleep(150);

	err = mdfld_dsi_send_dcs(sender,
		 set_tear_on,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", set_tear_on);
		goto power_err;
	}

	err = mdfld_dsi_send_dcs(sender,
		 set_display_on,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent set_display_on faild\n", __func__);
		goto power_err;
	}
	msleep(21);

power_err:
	return err;
}

static
int __mdfld_gi_sony_dsi_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

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

	/*set tear off*/
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

	/* disable BLCON, disable CABC */
	param[0] = 0x28;
	err = mdfld_dsi_send_dcs(sender,
		 write_ctrl_display,
		 param,
		 1,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s - sent write_ctrl_display faild\n", __func__);
		goto power_err;
	}

	/* Enter sleep mode */
	err = mdfld_dsi_send_dcs(sender,
		enter_sleep_mode,
		NULL,
		0,
		CMD_DATA_SRC_SYSTEM_MEM,
		MDFLD_DSI_SEND_PACKAGE);

	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", enter_sleep_mode);
		goto power_err;
	}
	msleep(120);

	/* DSTB, deep standby sequenc */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_passwd2_on, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_dstb_on, 4, 0);
	PSB_DEBUG_ENTRY("putting panel into deep sleep standby\n");

	msleep(50);

power_err:
	return err;
}

static
void gi_sony_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	if (pipe == 0) {
		pi->width_mm = PANEL_3DOT47_WIDTH;
		pi->height_mm = PANEL_3DOT47_HEIGHT;
	}
}

static
int mdfld_gi_sony_dsi_cmd_detect(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	int status;
	int pipe = dsi_config->pipe;
	uint32_t dpll_val, device_ready_val;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

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
			mdfld_dsi_send_gen_long_hs(sender,
						 gi_l5f3_passwd1_on, 4, 0);
			mdfld_dsi_send_gen_long_hs(sender,
						 gi_l5f3_set_disctl, 20, 0);
			mdfld_dsi_send_gen_long_hs(sender,
						 gi_l5f3_passwd1_off, 4, 0);
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
int mdfld_gi_sony_dsi_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 backlight_value;
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	backlight_value = ((level * 0xff) / 100) & 0xff;

	param[0] = backlight_value;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
				 write_display_brightness,
				 param,
				 3,
				 CMD_DATA_SRC_SYSTEM_MEM,
				 MDFLD_DSI_SEND_PACKAGE);

	if (err)
		DRM_ERROR("DCS 0x%x sent failed\n", exit_sleep_mode);

	return 0;
}

static
int mdfld_gi_sony_dsi_panel_reset(struct mdfld_dsi_config *dsi_config)
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


void gi_sony_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	p_funcs->get_config_mode = gi_sony_cmd_get_config_mode;
	p_funcs->get_panel_info = gi_sony_cmd_get_panel_info;
	p_funcs->reset = mdfld_gi_sony_dsi_panel_reset;
	p_funcs->drv_ic_init = mdfld_gi_l5f3_dbi_ic_init;
	p_funcs->dsi_controller_init = mdfld_gi_sony_dsi_controller_init;
	p_funcs->detect = mdfld_gi_sony_dsi_cmd_detect;
	p_funcs->set_brightness = mdfld_gi_sony_dsi_cmd_set_brightness;
	p_funcs->power_on = __mdfld_gi_sony_dsi_power_on;
	p_funcs->power_off = __mdfld_gi_sony_dsi_power_off;
}
