/*
 * Copyright (C) 2010 Intel Corporation
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
 * Austin Hu <austin.hu@intel.com>
 */

#include <asm/intel_scu_pmic.h>

#include "displays/jdi_vid.h"

static u8 jdi_set_address_mode[] = {0x36, 0xc0, 0x00, 0x00};
static u8 jdi_write_display_brightness[] = {0x51, 0x0f, 0xff, 0x00};

int mdfld_dsi_jdi_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	/* Set Address Mode */
	err = mdfld_dsi_send_mcs_long_hs(sender, jdi_set_address_mode,
			4,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Address Mode\n", __func__, __LINE__);
		goto ic_init_err;
	}

	/* Set Pixel format */
	err = mdfld_dsi_send_mcs_short_hs(sender, set_pixel_format, 0x70, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Pixel format\n", __func__, __LINE__);
		goto ic_init_err;
	}

	/* change "ff0f" according to the brightness desired. */
	err = mdfld_dsi_send_mcs_long_hs(sender, jdi_write_display_brightness,
			4, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Brightness\n", __func__, __LINE__);
		goto ic_init_err;
	}

	/* Write control display */
	err = mdfld_dsi_send_mcs_short_hs(sender, write_ctrl_display, 0x24, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Write Control Display\n", __func__,
				__LINE__);
		goto ic_init_err;
	}

	/* Write control CABC */
	err = mdfld_dsi_send_mcs_short_hs(sender, write_ctrl_cabc, STILL_IMAGE,
			1, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Write Control CABC\n", __func__, __LINE__);
		goto ic_init_err;
	}

	return 0;

ic_init_err:
	err = -EIO;
	return err;
}

static
void mdfld_dsi_jdi_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
	/* Virtual channel number */
	int mipi_vc = 0;
	int mipi_pixel_format = 0x4;
	/* BURST_MODE */
	int mipi_mode = 0x3;
	/* IP_TG_CONFIG */
	int ip_tg_config = 0x4;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 3;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	hw_ctx->pll_bypass_mode = 0;
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->turn_around_timeout = 0xFFFF;
	hw_ctx->device_reset_timer = 0xFF;
	hw_ctx->high_low_switch_count = 0x20;
	hw_ctx->clk_lane_switch_time_cnt = 0x0020000E;
	hw_ctx->dbi_bw_ctrl = 0x0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->init_count = 0x7D0;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->dphy_param = 0x1B0F4115;

	/*setup video mode format*/
	hw_ctx->video_mode_format = mipi_mode | ip_tg_config;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = ((mipi_pixel_format << 7) | (mipi_vc << 3) |
			dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		BANDGAP_CHICKEN_BIT | dsi_config->lane_config;
}

static int mdfld_dsi_jdi_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;
	u32 power_island = 0;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		power_island = pipe_to_island(pipe);

		if (!power_island_get(power_island)) {
			DRM_ERROR("Failed to turn on power island\n");
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

		power_island_put(power_island);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int mdfld_dsi_jdi_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* Sleep Out */
	err = mdfld_dsi_send_mcs_short_hs(sender, exit_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Exit Sleep Mode\n", __func__, __LINE__);
		goto power_on_err;
	}
	/* Wait for 6 frames after exit_sleep_mode. */
	msleep(100);

	/* Set Display on */
	err = mdfld_dsi_send_mcs_short_hs(sender, set_display_on, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
		goto power_on_err;
	}
	/* Wait for 1 frame after set_display_on. */
	msleep(20);

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender, MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		goto power_on_err;
	}

	return 0;

power_on_err:
	err = -EIO;
	return err;
}

static void __vpro2_power_ctrl(bool on)
{
	u8 addr, value;
	addr = 0xad;
	if (intel_scu_ipc_ioread8(addr, &value))
		DRM_ERROR("%s: %d: failed to read vPro2\n", __func__, __LINE__);

	/* Control vPROG2 power rail with 2.85v. */
	if (on)
		value |= 0x1;
	else
		value &= ~0x1;

	if (intel_scu_ipc_iowrite8(addr, value))
		DRM_ERROR("%s: %d: failed to write vPro2\n",
				__func__, __LINE__);
}

static int mdfld_dsi_jdi_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		goto power_off_err;
	}
	/* According HW DSI spec, need to wait for 100ms. */
	msleep(100);

	/* Set Display off */
	err = mdfld_dsi_send_mcs_short_hs(sender, set_display_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
		goto power_off_err;
	}
	/* Wait for 1 frame after set_display_on. */
	msleep(20);

	/* Sleep In */
	err = mdfld_dsi_send_mcs_short_hs(sender, enter_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Exit Sleep Mode\n", __func__, __LINE__);
		goto power_off_err;
	}
	/* Wait for 3 frames after enter_sleep_mode. */
	msleep(51);

	/* Can not poweroff VPROG2, because many other module related to
	 * this power supply, such as PSH sensor. */
	/*__vpro2_power_ctrl(false);*/

	return 0;

power_off_err:
	err = -EIO;
	return err;
}

static int mdfld_dsi_jdi_set_brightness(struct mdfld_dsi_config *dsi_config,
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

	duty_val = (0xFFF * level) / 100;

	/*
	 * Note: the parameters of write_display_brightness in JDI R69001 spec
	 * map DBV[7:4] as MSB.
	 */
	jdi_write_display_brightness[0] = 0x51;
	jdi_write_display_brightness[1] = duty_val & 0xF;
	jdi_write_display_brightness[2] = ((duty_val & 0xFF0) >> 4);
	jdi_write_display_brightness[3] = 0x0;
	mdfld_dsi_send_mcs_long_hs(sender, jdi_write_display_brightness, 4, 0);

	return 0;
}

static int mdfld_dsi_jdi_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	u8 *vaddr = NULL, *vaddr1 = NULL;
	int reg_value = 0, reg_value_scl = 0;
	int j = 0;

	PSB_DEBUG_ENTRY("\n");

	/* Because when reset touchscreen panel, touchscreen will pull i2c bus
	 * to low, sometime this operation will cause i2c bus enter into wrong
	 * status, so before reset, switch i2c scl pin */
	vaddr1 = ioremap(0xff0c1d30, 4);
	reg_value_scl = ioread32(vaddr1);
	reg_value_scl &= ~0x1000;
	iowrite32(reg_value_scl, vaddr1);

	__vpro2_power_ctrl(true);

	/* For meeting tRW1 panel spec */
	usleep_range(2000, 2500);

	vaddr = ioremap(0xff008000, 0x60);

	reg_value = ioread32(vaddr);
	if (reg_value == 1) {
		while (j++ < 0x1000)
			DRM_ERROR("GPDR is locked and can't be set.");
	} else {
		/* GPIO as an output */
		reg_value = ioread32(vaddr + 0x30);
		reg_value |= 0x60000000;
		iowrite32(reg_value, vaddr + 0x30);

		/* Clear the pin */
		iowrite32(0x60000000, vaddr + 0x60);

		/*
		 * For meeting tRW1 panel spec in case RST pin starts from high
		 */
		usleep_range(2000, 2500);

		/* set the pin */
		iowrite32(0x40000000, vaddr + 0x48);

		/* For meeting tsVSP panel spec */
		usleep_range(2000, 2500);

		/* set the pin */
		iowrite32(0x20000000, vaddr + 0x48);

		/* Additional delay for VSP and VSN to settle */
		usleep_range(2000, 2500);
	}

	iounmap(vaddr);

	/* switch i2c scl pin back */
	reg_value_scl |= 0x1000;
	iowrite32(reg_value_scl, vaddr1);
	iounmap(vaddr1);

	return 0;
}

static struct drm_display_mode *jdi_vid_get_config_mode(void)
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
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static void jdi_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi)
		return;

	if (pipe == 0) {
		pi->width_mm = JDI_PANEL_WIDTH;
		pi->height_mm = JDI_PANEL_HEIGHT;
	}

	return;
}

void jdi_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = jdi_vid_get_config_mode;
	p_funcs->get_panel_info = jdi_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_jdi_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_jdi_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_jdi_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_jdi_detect;
	p_funcs->power_on = mdfld_dsi_jdi_power_on;
	p_funcs->power_off = mdfld_dsi_jdi_power_off;
	p_funcs->set_brightness = mdfld_dsi_jdi_set_brightness;
}
