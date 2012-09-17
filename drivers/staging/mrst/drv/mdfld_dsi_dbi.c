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
 *  jim liu <jim.liu@intel.com>
 *  Jackie Li<yaodong.li@intel.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include "psb_powermgmt.h"
#include "mdfld_dsi_dbi_dsr.h"

/**
 * Enter DSR 
 */
void mdfld_dsi_dbi_enter_dsr (struct mdfld_dsi_dbi_output * dbi_output, int pipe)
{
	return;
}

#ifndef CONFIG_MDFLD_DSI_DPU
static void mdfld_dbi_output_exit_dsr(struct mdfld_dsi_dbi_output *dbi_output,
		int pipe,
		void *p_surfaceAddr,
		bool check_hw_on_only)
{
	return;
}

int mdfld_dsi_dbi_async_check_fifo_empty(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output **dbi_outputs = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_pkg_sender *sender = NULL;
	int err = 0;

	dbi_outputs = dsr_info->dbi_outputs;
	dbi_output = 0 ? dbi_outputs[1] : dbi_outputs[0];
	if (!dbi_output)
		return 0;

	sender = mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);

	err = mdfld_dsi_check_fifo_empty(sender);
	return err;
}

/*
 * use hw te to update fb
 */
int mdfld_dsi_dbi_async_flip_fb_update(struct drm_device *dev, int pipe)
{
	return 0;
}

/**
 * Exit from DSR
 */
void mdfld_dsi_dbi_exit_dsr(struct drm_device *dev,
		u32 update_src,
		void *p_surfaceAddr,
		bool check_hw_on_only)
{
}

static bool mdfld_dbi_is_in_dsr(struct drm_device * dev)
{
	if(REG_READ(MRST_DPLL_A) & DPLL_VCO_ENABLE)
		return false;
	if((REG_READ(PIPEACONF) & PIPEACONF_ENABLE) ||
	   (REG_READ(PIPECCONF) & PIPEACONF_ENABLE))
		return false;
	if((REG_READ(DSPACNTR) & DISPLAY_PLANE_ENABLE) ||
	   (REG_READ(DSPCCNTR) & DISPLAY_PLANE_ENABLE))
		return false;

	return true;
}

/* Perodically update dbi panel */
void mdfld_dbi_update_panel(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output **dbi_outputs;
	struct mdfld_dsi_dbi_output *dbi_output;
	struct mdfld_dsi_config *dsi_config;
	struct mdfld_dsi_hw_context *ctx;
	int i;

	if (!dsr_info)
		return;

	dbi_outputs = dsr_info->dbi_outputs;
	dbi_output = pipe ? dbi_outputs[1] : dbi_outputs[0];
	dsi_config = pipe ? dev_priv->dsi_configs[1] : dev_priv->dsi_configs[0];

	if (!dbi_output || !dsi_config || (pipe == 1))
		return;

	ctx = &dsi_config->dsi_hw_context;

	/*lock dsi config*/
	mutex_lock(&dsi_config->context_lock);

	/*if FB is damaged and panel is on update on-panel FB*/
	if (!ctx->panel_on)
		goto update_out;

	if (dbi_output->p_funcs && dbi_output->p_funcs->update_fb)
		dbi_output->p_funcs->update_fb(dbi_output, pipe);
update_out:
	mutex_unlock(&dsi_config->context_lock);
}

int mdfld_dbi_dsr_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;

	PSB_DEBUG_ENTRY("\n");

	if (!dsr_info || IS_ERR(dsr_info)) {
		dsr_info = kzalloc(sizeof(struct mdfld_dbi_dsr_info),
				   GFP_KERNEL);
		if (!dsr_info) {
			DRM_ERROR("No memory\n");
			return -ENOMEM;
		}

		dev_priv->dbi_dsr_info = dsr_info;
	}

	return 0;
}
#endif

static int __dbi_enter_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	if (ctx->device_ready & DSI_POWER_STATE_ULPS_MASK) {
		DRM_ERROR("Broken ULPS states\n");
		return -EINVAL;
	}

	/*wait for all FIFOs empty*/
	mdfld_dsi_wait_for_fifos_empty(sender);

	/*inform DSI host is to be put on ULPS*/
	ctx->device_ready |= DSI_POWER_STATE_ULPS_ENTER;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	DRM_INFO("%s: entered ULPS state\n", __func__);
	return 0;
}

static int __dbi_exit_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	/*enter ULPS EXIT state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	ctx->device_ready |= DSI_POWER_STATE_ULPS_EXIT;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	/*wait for 1ms as spec suggests*/
	mdelay(1);

	/*clear ULPS state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	DRM_INFO("%s: exited ULPS state\n", __func__);
	return 0;
}

/**
 * Power on sequence for command mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dbi_panel_power_on(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int retry;
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	mdfld_dsi_dsr_forbid_locked(dsi_config);

	/*after entering dstb mode, need reset*/
	if (p_funcs && p_funcs->reset)
		p_funcs->reset(dsi_config, RESET_FROM_OSPM_RESUME);

	/*Enable DSI PLL*/
	if (!(REG_READ(regs->dpll_reg) & BIT31)) {
		if (ctx->pll_bypass_mode) {
			uint32_t dpll = 0;

			REG_WRITE(regs->dpll_reg, dpll);
			if (ctx->cck_div)
				dpll = dpll | BIT11;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT12;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT13;
			REG_WRITE(regs->dpll_reg, dpll);
			dpll = dpll | BIT31;
			REG_WRITE(regs->dpll_reg, dpll);
		} else {
			REG_WRITE(regs->dpll_reg, 0x0);
			REG_WRITE(regs->fp_reg, 0x0);
			REG_WRITE(regs->fp_reg, ctx->fp);
			REG_WRITE(regs->dpll_reg, ((ctx->dpll) & ~BIT30));
			udelay(2);
			val = REG_READ(regs->dpll_reg);
			REG_WRITE(regs->dpll_reg, (val | BIT31));

			/*wait for PLL lock on pipe*/
			retry = 10000;
			while (--retry && !(REG_READ(PIPEACONF) & BIT29))
				udelay(3);
			if (!retry) {
				DRM_ERROR("PLL failed to lock on pipe\n");
				err = -EAGAIN;
				goto power_on_err;
			}
		}
	}

	/*D-PHY parameter*/
	REG_WRITE(regs->dphy_param_reg, ctx->dphy_param);

	/*Configure DSI controller*/
	REG_WRITE(regs->mipi_control_reg, ctx->mipi_control);
	REG_WRITE(regs->intr_en_reg, ctx->intr_en);
	REG_WRITE(regs->hs_tx_timeout_reg, ctx->hs_tx_timeout);
	REG_WRITE(regs->lp_rx_timeout_reg, ctx->lp_rx_timeout);
	REG_WRITE(regs->turn_around_timeout_reg,
		ctx->turn_around_timeout);
	REG_WRITE(regs->device_reset_timer_reg,
		ctx->device_reset_timer);
	REG_WRITE(regs->high_low_switch_count_reg,
		ctx->high_low_switch_count);
	REG_WRITE(regs->init_count_reg, ctx->init_count);
	REG_WRITE(regs->eot_disable_reg, ctx->eot_disable);
	REG_WRITE(regs->lp_byteclk_reg, ctx->lp_byteclk);
	REG_WRITE(regs->clk_lane_switch_time_cnt_reg,
		ctx->clk_lane_switch_time_cnt);
	REG_WRITE(regs->dsi_func_prg_reg, ctx->dsi_func_prg);

	/*DBI bw ctrl*/
	REG_WRITE(regs->dbi_bw_ctrl_reg, ctx->dbi_bw_ctrl);

	/*Setup pipe timing*/
	REG_WRITE(regs->htotal_reg, ctx->htotal);
	REG_WRITE(regs->hblank_reg, ctx->hblank);
	REG_WRITE(regs->hsync_reg, ctx->hsync);
	REG_WRITE(regs->vtotal_reg, ctx->vtotal);
	REG_WRITE(regs->vblank_reg, ctx->vblank);
	REG_WRITE(regs->vsync_reg, ctx->vsync);
	REG_WRITE(regs->pipesrc_reg, ctx->pipesrc);
	REG_WRITE(regs->dsppos_reg, ctx->dsppos);
	REG_WRITE(regs->dspstride_reg, ctx->dspstride);

	/*Setup plane*/
	REG_WRITE(regs->dspsize_reg, ctx->dspsize);
	REG_WRITE(regs->dspsurf_reg, ctx->dspsurf);
	REG_WRITE(regs->dsplinoff_reg, ctx->dsplinoff);
	REG_WRITE(regs->vgacntr_reg, ctx->vgacntr);

	/*exit ULPS*/
	if (__dbi_exit_ulps_locked(dsi_config)) {
		DRM_ERROR("Failed to exit ULPS\n");
		goto power_on_err;
	}
	REG_WRITE(regs->device_ready_reg, (ctx->device_ready | BIT0));

	/*Enable pipe*/
	val = ctx->pipeconf;
	val &= ~0x000c0000 | BIT31 | PIPEACONF_DSR;
	REG_WRITE(regs->pipeconf_reg, val);

	/*Wait for pipe enabling,when timing generator is working */
	retry = 10000;
	while (--retry && !(REG_READ(regs->pipeconf_reg) & BIT30))
		udelay(3);

	if (!retry) {
		DRM_ERROR("Failed to enable pipe\n");
		err = -EAGAIN;
		goto power_on_err;
	}

	/*enable plane*/
	val = ctx->dspcntr | BIT31;
	REG_WRITE(regs->dspcntr_reg, val);

	/*update MIPI port config*/
	REG_WRITE(regs->mipi_reg, ctx->mipi);

	/*set low power output hold*/
	REG_WRITE(regs->mipi_reg, (ctx->mipi | BIT16));

	/*enable TE, will need it in panel power on*/
	mdfld_enable_te(dev, dsi_config->pipe);

	/**
	 * Different panel may have different ways to have
	 * drvIC initialized. Support it!
	 */
	if (p_funcs && p_funcs->drv_ic_init) {
		if (p_funcs->drv_ic_init(dsi_config, 0)) {
			DRM_ERROR("Failed to init dsi controller, reset it!\n");
			err = -EAGAIN;
			goto power_on_err;
		}
	}

	/**
	 * Different panel may have different ways to have
	 * panel turned on. Support it!
	 */
	if (p_funcs && p_funcs->power_on)
		if (p_funcs->power_on(dsi_config)) {
			DRM_ERROR("Failed to power on panel\n");
			err = -EAGAIN;
			goto power_on_err;
		}

	/*Notify PVR module that screen is on*/
	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 1);

	if (p_funcs && p_funcs->set_brightness)
		if (p_funcs->set_brightness(dsi_config,
					ctx->lastbrightnesslevel))
			DRM_ERROR("Failed to set panel brightness\n");

power_on_err:
	mdfld_dsi_dsr_allow_locked(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Power off sequence for command mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dbi_panel_power_off(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	int pipe0_enabled;
	int pipe2_enabled;
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	PSB_DEBUG_ENTRY("\n");

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs && p_funcs->set_brightness)
		if (p_funcs->set_brightness(dsi_config, 0))
			DRM_ERROR("Failed to set panel brightness\n");

	/*Notify PVR module that screen is off*/
	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);

	mdfld_dsi_dsr_forbid_locked(dsi_config);

	/*Disable TE, don't need it anymore*/
	mdfld_disable_te(dev, dsi_config->pipe);

	/**
	 * Different panel may have different ways to have
	 * panel turned off. Support it!
	 */
	if (p_funcs && p_funcs->power_off) {
		if (p_funcs->power_off(dsi_config)) {
			DRM_ERROR("Failed to power off panel\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}

	/*save the plane informaton, for it will updated*/
	ctx->dspsurf = dev_priv->init_screen_start;
	ctx->dsplinoff = dev_priv->init_screen_offset;
	ctx->pipestat = REG_READ(regs->pipestat_reg);
	ctx->dspcntr = REG_READ(regs->dspcntr_reg);
	ctx->dspstride = REG_READ(regs->dspstride_reg);
	ctx->pipeconf = REG_READ(regs->pipeconf_reg);

	/*Disable plane*/
	val = ctx->dspcntr;
	REG_WRITE(regs->dspcntr_reg, (val & ~BIT31));

	/*Disable pipe and overlay & cursor panel assigned to this pipe*/
	val = ctx->pipeconf;
	REG_WRITE(regs->pipeconf_reg, ((val | 0x000c0000) & ~BIT31));

	/*Disable DSI controller*/
	val = ctx->device_ready;
	REG_WRITE(regs->device_ready_reg, (val & ~BIT0));

	/*enter ulps*/
	if (__dbi_enter_ulps_locked(dsi_config)) {
		DRM_ERROR("Failed to enter ULPS\n");
		goto power_off_err;
	}

	/*Disable DSI PLL*/
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled) {
		REG_WRITE(regs->dpll_reg , 0x0);
		/*power gate pll*/
		REG_WRITE(regs->dpll_reg, BIT30);
	}
power_off_err:
	mdfld_dsi_dsr_allow_locked(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/* generic dbi function */
static
int mdfld_generic_dsi_dbi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_dbi_output *dbi_output;
	struct mdfld_dsi_connector *dsi_connector;
	struct mdfld_dsi_config *dsi_config;
	struct panel_funcs *p_funcs;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;

	if (!encoder) {
		DRM_ERROR("Invalid encoder\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("%s\n", (on ? "on" : "off"));

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dbi_output = MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dsi_connector = mdfld_dsi_encoder_get_connector(dsi_encoder);
	p_funcs = dbi_output->p_funcs;
	dev = encoder->dev;
	dev_priv = dev->dev_private;

	mutex_lock(&dsi_config->context_lock);

	if (dsi_connector->status != connector_status_connected)
		goto set_power_err;

	if (dbi_output->first_boot &&
	    dsi_config->dsi_hw_context.panel_on) {
		DRM_INFO("skip panle power setting for first boot! " \
			 "panel is already powered on\n");
		goto fun_exit;
	}

	switch (on) {
	case true:
		/* panel is already on */
		if (dsi_config->dsi_hw_context.panel_on)
			goto fun_exit;

		if (__dbi_panel_power_on(dsi_config, p_funcs)) {
			DRM_ERROR("Faild to turn on panel\n");
			goto set_power_err;
		}
		mdfld_dsi_error_detector_wakeup(dsi_connector);

		dsi_config->dsi_hw_context.panel_on = 1;
		dbi_output->dbi_panel_on = 1;
		break;
	case false:
		if (!dsi_config->dsi_hw_context.panel_on &&
		    !dbi_output->first_boot)
			goto fun_exit;

		if (__dbi_panel_power_off(dsi_config, p_funcs)) {
			DRM_ERROR("Faild to turn off panel\n");
			goto set_power_err;
		}

		dsi_config->dsi_hw_context.panel_on = 0;
		dbi_output->dbi_panel_on = 0;
		break;
	default:
		break;
	}

fun_exit:
	mutex_unlock(&dsi_config->context_lock);
	PSB_DEBUG_ENTRY("successfully\n");
	return 0;

set_power_err:
	mutex_unlock(&dsi_config->context_lock);
	PSB_DEBUG_ENTRY("unsuccessfully!\n");
	return -EAGAIN;
}

static
void mdfld_generic_dsi_dbi_mode_set(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	return;
}

static
void mdfld_generic_dsi_dbi_prepare(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);

	PSB_DEBUG_ENTRY("\n");

	dbi_output->mode_flags |= MODE_SETTING_IN_ENCODER;
	dbi_output->mode_flags &= ~MODE_SETTING_ENCODER_DONE;

	mdfld_generic_dsi_dbi_set_power(encoder, false);
	gbdispstatus = false;
}

static
void mdfld_generic_dsi_dbi_commit(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder =
		MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct drm_device *dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("\n");

	mdfld_generic_dsi_dbi_set_power(encoder, true);
	gbdispstatus = true;

	dbi_output->mode_flags &= ~MODE_SETTING_IN_ENCODER;
	if (dbi_output->channel_num == 1)
		dev_priv->dsr_fb_update |= MDFLD_DSR_2D_3D_2;
	else
		dev_priv->dsr_fb_update |= MDFLD_DSR_2D_3D_0;
	dbi_output->mode_flags |= MODE_SETTING_ENCODER_DONE;

	dbi_output->first_boot = false;
}

static
void mdfld_generic_dsi_dbi_dpms(struct drm_encoder *encoder, int mode)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_dbi_output *dbi_output;
	struct drm_device *dev;
	struct mdfld_dsi_config *dsi_config;
	struct drm_psb_private *dev_priv;

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dbi_output = MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("%s\n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));

	if (!gbdispstatus) {
		DRM_INFO("panel in suspend status, " \
			 "skip turn on/off from DMPS");
		return;
	}

	mutex_lock(&dev_priv->dpms_mutex);

	if (mode == DRM_MODE_DPMS_ON)
		mdfld_generic_dsi_dbi_set_power(encoder, true);
	else
		mdfld_generic_dsi_dbi_set_power(encoder, false);

	mutex_unlock(&dev_priv->dpms_mutex);
}

static
void mdfld_generic_dsi_dbi_save(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	mdfld_generic_dsi_dbi_set_power(encoder, false);
}

static
void mdfld_generic_dsi_dbi_restore(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	mdfld_generic_dsi_dbi_set_power(encoder, true);
}

static
bool mdfld_generic_dsi_dbi_mode_fixup(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct drm_display_mode *fixed_mode = dbi_output->panel_fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	if (fixed_mode) {
		adjusted_mode->hdisplay = fixed_mode->hdisplay;
		adjusted_mode->hsync_start = fixed_mode->hsync_start;
		adjusted_mode->hsync_end = fixed_mode->hsync_end;
		adjusted_mode->htotal = fixed_mode->htotal;
		adjusted_mode->vdisplay = fixed_mode->vdisplay;
		adjusted_mode->vsync_start = fixed_mode->vsync_start;
		adjusted_mode->vsync_end = fixed_mode->vsync_end;
		adjusted_mode->vtotal = fixed_mode->vtotal;
		adjusted_mode->clock = fixed_mode->clock;
		drm_mode_set_crtcinfo(adjusted_mode, CRTC_INTERLACE_HALVE_V);
		kfree(fixed_mode);
	}

	return true;
}

static
struct drm_encoder_funcs dsi_dbi_generic_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static
struct drm_encoder_helper_funcs dsi_dbi_generic_encoder_helper_funcs = {
	.save = mdfld_generic_dsi_dbi_save,
	.restore = mdfld_generic_dsi_dbi_restore,
	.dpms = mdfld_generic_dsi_dbi_dpms,
	.mode_fixup = mdfld_generic_dsi_dbi_mode_fixup,
	.prepare = mdfld_generic_dsi_dbi_prepare,
	.mode_set = mdfld_generic_dsi_dbi_mode_set,
	.commit = mdfld_generic_dsi_dbi_commit,
};

/*
 * Init DSI DBI encoder.
 * Allocate an mdfld_dsi_encoder and attach it to given @dsi_connector
 * return pointer of newly allocated DBI encoder, NULL on error
 */
struct mdfld_dsi_encoder *mdfld_dsi_dbi_init(struct drm_device *dev,
		struct mdfld_dsi_connector *dsi_connector,
		struct panel_funcs *p_funcs)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_config *dsi_config;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL;
	struct drm_display_mode *fixed_mode = NULL;
	struct psb_gtt *pg = dev_priv ? (dev_priv->pg) : NULL;

#ifdef CONFIG_MDFLD_DSI_DPU
	struct mdfld_dbi_dpu_info *dpu_info =
		dev_priv ? (dev_priv->dbi_dpu_info) : NULL;
#else
	struct mdfld_dbi_dsr_info *dsr_info =
		dev_priv ? (dev_priv->dbi_dsr_info) : NULL;
#endif
	int pipe;
	int ret;

	PSB_DEBUG_ENTRY("\n");

	if (!pg || !dsi_connector || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return NULL;
	}

	dsi_config = mdfld_dsi_get_config(dsi_connector);
	pipe = dsi_connector->pipe;

	/*detect panel connection stauts*/
	if (p_funcs->detect) {
		ret = p_funcs->detect(dsi_config, pipe);
		if (ret) {
			PSB_DEBUG_ENTRY("Fail to detect Panel %d\n",
					pipe);
			dsi_connector->status =
				connector_status_disconnected;
		} else {
			PSB_DEBUG_ENTRY("Panel %d is connected\n",
					pipe);
			dsi_connector->status =
				connector_status_connected;
		}
	} else {
		/*use the default config*/
		if (pipe == 0)
			dsi_connector->status =
				connector_status_connected;
		else
			dsi_connector->status =
				connector_status_disconnected;
	}

	/*init DSI controller*/
	if (p_funcs->dsi_controller_init)
		p_funcs->dsi_controller_init(dsi_config, pipe);

	if (dsi_connector->status == connector_status_connected) {
		if (pipe == 0)
			dev_priv->panel_desc |= DISPLAY_A;
		if (pipe == 2)
			dev_priv->panel_desc |= DISPLAY_C;
	}

	/* TODO: get panel info from DDB */
	dbi_output = kzalloc(sizeof(struct mdfld_dsi_dbi_output), GFP_KERNEL);
	if (!dbi_output) {
		DRM_ERROR("No memory\n");
		return NULL;
	}

	if (dsi_connector->pipe == 0) {
		dbi_output->channel_num = 0;
		dev_priv->dbi_output = dbi_output;
	} else if (dsi_connector->pipe == 2) {
		dbi_output->channel_num = 1;
		dev_priv->dbi_output2 = dbi_output;
	} else {
		DRM_ERROR("only support 2 DSI outputs\n");
		goto out_err1;
	}

	dbi_output->dev = dev;
	dbi_output->p_funcs = p_funcs;

	/*get fixed mode*/
	fixed_mode = dsi_config->fixed_mode;

	dbi_output->panel_fixed_mode = fixed_mode;

	/*create drm encoder object*/
	connector = &dsi_connector->base.base;
	encoder = &dbi_output->base.base;
	drm_encoder_init(dev,
			encoder,
			&dsi_dbi_generic_encoder_funcs,
			DRM_MODE_ENCODER_MIPI);
	drm_encoder_helper_add(encoder,
			&dsi_dbi_generic_encoder_helper_funcs);

	/*attach to given connector*/
	drm_mode_connector_attach_encoder(connector, encoder);
	connector->encoder = encoder;

	/*set possible crtcs and clones*/
	if (dsi_connector->pipe) {
		encoder->possible_crtcs = (1 << 2);
		encoder->possible_clones = (1 << 1);
	} else {
		encoder->possible_crtcs = (1 << 0);
		encoder->possible_clones = (1 << 0);
	}

	dev_priv->dsr_fb_update = 0;
	dev_priv->b_dsr_enable = false;
	dev_priv->b_async_flip_enable = false;
	dev_priv->exit_idle = mdfld_dsi_dbi_exit_dsr;
	dev_priv->async_flip_update_fb = mdfld_dsi_dbi_async_flip_fb_update;
	dev_priv->async_check_fifo_empty = mdfld_dsi_dbi_async_check_fifo_empty;

#if defined(CONFIG_MDFLD_DSI_DPU) || defined(CONFIG_MDFLD_DSI_DSR)
	dev_priv->b_dsr_enable_config = true;
#endif /*CONFIG_MDFLD_DSI_DSR*/

	dbi_output->first_boot = true;
	dbi_output->mode_flags = MODE_SETTING_IN_ENCODER;

#ifdef CONFIG_MDFLD_DSI_DPU
	/*add this output to dpu_info*/

	if (dsi_connector->status == connector_status_connected) {
		if (dsi_connector->pipe == 0)
			dpu_info->dbi_outputs[0] = dbi_output;
		else
			dpu_info->dbi_outputs[1] = dbi_output;

		dpu_info->dbi_output_num++;
	}

#else /*CONFIG_MDFLD_DSI_DPU*/
	if (dsi_connector->status == connector_status_connected) {
		/*add this output to dsr_info*/
		if (dsi_connector->pipe == 0)
			dsr_info->dbi_outputs[0] = dbi_output;
		else
			dsr_info->dbi_outputs[1] = dbi_output;

		dsr_info->dbi_output_num++;
	}
#endif

	PSB_DEBUG_ENTRY("successfully\n");

	return &dbi_output->base;

out_err1:
	kfree(dbi_output);

	return NULL;
}

void mdfld_reset_panel_handler_work(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
		container_of(work, struct drm_psb_private, reset_panel_work);
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct panel_funcs *p_funcs  = NULL;

	dbi_output = dev_priv->dbi_output;
	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config || !dbi_output)
		return;

	/*disable ESD when HDMI connected*/
	if (hdmi_state)
		return;

	PSB_DEBUG_ENTRY("\n");

	p_funcs = dbi_output->p_funcs;
	if (p_funcs) {
		mutex_lock(&dsi_config->context_lock);

		if (__dbi_panel_power_off(dsi_config, p_funcs)) {
			mutex_unlock(&dsi_config->context_lock);
			return;
		}

		acquire_ospm_lock();
		ospm_power_island_down(OSPM_DISPLAY_A_ISLAND);
		ospm_power_island_up(OSPM_DISPLAY_A_ISLAND);
		release_ospm_lock();

		if (__dbi_panel_power_on(dsi_config, p_funcs)) {
			mutex_unlock(&dsi_config->context_lock);
			return;
		}

		mutex_unlock(&dsi_config->context_lock);

		DRM_INFO("%s: End panel reset\n", __func__);
	} else {
		DRM_INFO("%s invalid panel init\n", __func__);
	}
}
