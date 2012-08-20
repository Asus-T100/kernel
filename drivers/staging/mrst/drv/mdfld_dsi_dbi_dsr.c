/*
 * Copyright © 2012 Intel Corporation
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
 * Jackie Li<yaodong.li@intel.com>
 */
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dsr.h"
#include "mdfld_dsi_pkg_sender.h"

#define DSR_COUNT 2

static int exit_dsr_locked(struct mdfld_dsi_config *dsi_config)
{
	u32 val = 0;
	struct mdfld_dsi_pkg_sender *sender;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int retry;
	int err;

	PSB_DEBUG_ENTRY("mdfld_dsi_dsr: exit dsr\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	sender = mdfld_dsi_get_pkg_sender(dsi_config);

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
		OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("Failed power on display island\n");
		return -EINVAL;
	}

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

	REG_WRITE(regs->device_ready_reg, ctx->device_ready | BIT0);

	/*Enable pipe*/
	val = ctx->pipeconf;
	val &= ~0x000c0000;
	val |= BIT31;
	val |= PIPEACONF_DSR;

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

	REG_WRITE(regs->ovaadd_reg, ctx->ovaadd);

	/*enable TE, will need it in panel power on*/
	mdfld_enable_te(dev, dsi_config->pipe);

power_on_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return 0;
}

static int enter_dsr_locked(struct mdfld_dsi_config *dsi_config, int level)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	struct mdfld_dsi_pkg_sender *sender;
	int err;

	int pipe0_enabled;
	int pipe2_enabled;

	PSB_DEBUG_ENTRY("mdfld_dsi_dsr: enter dsr\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	sender = mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("Failed to get dsi sender\n");
		return -EINVAL;
	}

	if (level < DSR_EXITED) {
		DRM_ERROR("Why to do this?");
		return -EINVAL;
	}

	if (level > DSR_ENTERED_LEVEL0) {
		/**
		 * TODO: require OSPM interfaces to tell OSPM module that
		 * display controller is ready to be power gated.
		 * OSPM module needs to response this request ASAP.
		 * NOTE: it makes no sense to have display controller islands
		 * & pci power gated here directly. OSPM module is the only one
		 * who can power gate/ungate power islands.
		 * FIXME: since there's no ospm interfaces for acquiring
		 * suspending DSI related power islands, we have to call OSPM
		 * interfaces to power gate display islands and pci right now,
		 * which should NOT happen in this way!!!
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("Failed power on display island\n");
			return -EINVAL;
		}

		PSB_DEBUG_ENTRY("mdfld_dsi_dsr: entering DSR level 1\n");

		/*Disable TE, don't need it anymore*/
		mdfld_disable_te(dev, dsi_config->pipe);
		err = mdfld_dsi_wait_for_fifos_empty(sender);
		if (err) {
			DRM_ERROR("mdfld_dsi_dsr: FIFO not empty\n");
			return err;
		}
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

		/*suspend display*/
		ospm_suspend_display(dev);

		/*
		 *suspend pci
		 *FIXME: should I do it here?
		 *how about decoder/encoder is working??
		 *OSPM should check the refcout of each islands before
		 *actually power off PCI!!!
		 *need invoke this in the same context, we need deal with
		 *DSR lock later for suspend PCI may go to sleep!!!
		 */
		/*ospm_suspend_pci(dev->pdev);*/

		PSB_DEBUG_ENTRY("mdfld_dsi_dsr: entered\n");
		return 0;
	}

	/*
	 * if DSR_EXITED < level < DSR_ENTERED_LEVEL1, we only have the display
	 * controller components turned off instead of power gate them.
	 * this is useful for HDMI & WIDI.
	 */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
		OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("Failed power on display island\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("mdfld_dsi_dsr: entering DSR level 0\n");

	/*Disable TE, don't need it anymore*/
	mdfld_disable_te(dev, dsi_config->pipe);

	/*Disable plane*/
	val = ctx->dspcntr;
	REG_WRITE(regs->dspcntr_reg, (val & ~BIT31));

	val = REG_READ(regs->pipeconf_reg);
	/*Disable overlay & cursor panel assigned to this pipe*/
	REG_WRITE(regs->pipeconf_reg, (val | (0x000c0000)));

	/*Disable pipe*/
	val = REG_READ(regs->pipeconf_reg);
	val &= ~BIT31;
	REG_WRITE(regs->pipeconf_reg, val);

	mdfld_dsi_wait_for_fifos_empty(sender);

	/*Disable DSI PLL*/
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled)
		REG_WRITE(regs->dpll_reg , 0x0);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	PSB_DEBUG_ENTRY("entered\n");
	return 0;
}

static void dsr_power_off_work(struct work_struct *work)
{
	DRM_INFO("mdfld_dsi_dsr: power off work\n");
}

static void dsr_power_on_work(struct work_struct *work)
{
	DRM_INFO("mdfld_dsi_dsr: power on work\n");
}

int mdfld_dsi_dsr_update_panel_fb(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	struct mdfld_dsi_pkg_sender *sender;
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	/*ignore it if there are pending fb updates*/
	if (dsr->pending_fb_updates)
		goto update_fb_out;

	/*no pending fb updates, go ahead to send out write_mem_start*/
	PSB_DEBUG_ENTRY("send out write_mem_start\n");
	sender = mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("No sender\n");
		err = -EINVAL;
		goto update_fb_out;
	}

	err = mdfld_dsi_send_dcs(sender, write_mem_start,
				NULL, 0, CMD_DATA_SRC_PIPE,
				MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to send write_mem_start");
		err = -EINVAL;
		goto update_fb_out;
	}

	/*increase pending fb updates*/
	dsr->pending_fb_updates++;
	/*clear free count*/
	dsr->free_count = 0;
update_fb_out:
	return err;
}

int mdfld_dsi_dsr_report_te(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int err = 0;
	int dsr_level;

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	/*
	 * TODO: check HDMI & WIDI connection state here, then setup
	 * dsr_level accordingly.
	 */
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	/*
	 * FIXME: when hdmi connected with no audio output, we still can
	 * power gate DSI related islands, how to check whether HDMI audio
	 * is active or not.
	 * Currently, we simply enter DSR LEVEL0 when HDMI is connected
	 */
	if (dev_priv->bhdmiconnected)
		dsr_level = DSR_ENTERED_LEVEL0;
	else
		dsr_level = DSR_ENTERED_LEVEL1;

	mutex_lock(&dsi_config->context_lock);

	if (!dsr->dsr_enabled)
		goto report_te_out;

	/*if panel is off, then forget it*/
	if (!dsi_config->dsi_hw_context.panel_on)
		goto report_te_out;

	if (dsr_level <= dsr->dsr_state)
		goto report_te_out;
	else if (++dsr->free_count > DSR_COUNT && !dsr->ref_count) {
		/*reset free count*/
		dsr->free_count = 0;
		/*enter dsr*/
		err = enter_dsr_locked(dsi_config, dsr_level);
		if (err) {
			DRM_ERROR("Failed to enter DSR\n");
			goto report_te_out;
		}
		dsr->dsr_state = dsr_level;
	}
report_te_out:
	/*clear pending fb updates*/
	dsr->pending_fb_updates = 0;
	mutex_unlock(&dsi_config->context_lock);
	return err;
}

int mdfld_dsi_dsr_forbid_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	/*exit dsr if necessary*/

	if (!dsr->dsr_enabled)
		goto forbid_out;

	/*if reference count is not 0, it means dsr was forbidden*/
	if (dsr->ref_count) {
		dsr->ref_count++;
		goto forbid_out;
	}

	/*exited dsr if current dsr state is DSR_ENTERED*/
	if (dsr->dsr_state > DSR_EXITED) {
		err = exit_dsr_locked(dsi_config);
		if (err) {
			DRM_ERROR("Failed to exit DSR\n");
			goto forbid_out;
		}
		dsr->dsr_state = DSR_EXITED;
	}
	dsr->ref_count++;
forbid_out:
	return err;
}

int mdfld_dsi_dsr_forbid(struct mdfld_dsi_config *dsi_config)
{
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	mutex_lock(&dsi_config->context_lock);

	err = mdfld_dsi_dsr_forbid_locked(dsi_config);

	mutex_unlock(&dsi_config->context_lock);

	return err;
}

int mdfld_dsi_dsr_allow_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	if (!dsr->dsr_enabled)
		goto allow_out;

	dsr->ref_count--;
allow_out:
	return 0;
}

int mdfld_dsi_dsr_allow(struct mdfld_dsi_config *dsi_config)
{
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	mutex_lock(&dsi_config->context_lock);

	err = mdfld_dsi_dsr_allow_locked(dsi_config);

	mutex_unlock(&dsi_config->context_lock);

	return err;
}

void mdfld_dsi_dsr_enable(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return;

	/*lock dsr*/
	mutex_lock(&dsi_config->context_lock);

	dsr->dsr_enabled = 1;
	dsr->dsr_state = DSR_EXITED;

	mutex_unlock(&dsi_config->context_lock);
}

int mdfld_dsi_dsr_in_dsr(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	int in_dsr = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		goto get_state_out;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		goto get_state_out;

	/*lock dsr*/
	mutex_lock(&dsi_config->context_lock);

	if (dsr->dsr_state > DSR_EXITED)
		in_dsr = 1;

	mutex_unlock(&dsi_config->context_lock);
get_state_out:
	return in_dsr;
}

/**
 * init dsr structure
 */
int mdfld_dsi_dsr_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	/*check panel type*/
	if (dsi_config->type == MDFLD_DSI_ENCODER_DPI) {
		DRM_INFO("%s: Video mode panel, disabling DSR\n", __func__);
		return 0;
	}

	dsr = kzalloc(sizeof(struct mdfld_dsi_dsr), GFP_KERNEL);
	if (!dsr) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	/*init reference count*/
	dsr->ref_count = 0;

	/*init free count*/
	dsr->free_count = 0;

	/*init pending fb updates*/
	dsr->pending_fb_updates = 0;

	/*init dsr enabled*/
	dsr->dsr_enabled = 0;

	/*set dsr state*/
	dsr->dsr_state = DSR_INIT;

	/*init power on/off works*/
	INIT_WORK(&dsr->power_off_work, dsr_power_off_work);
	INIT_WORK(&dsr->power_on_work, dsr_power_on_work);

	/*init dsi config*/
	dsr->dsi_config = dsi_config;

	dsi_config->dsr = dsr;

	PSB_DEBUG_ENTRY("successfully\n");

	return 0;
}

/**
 * destroy dsr structure
 */
void mdfld_dsi_dsr_destroy(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	PSB_DEBUG_ENTRY("\n");

	dsr = dsi_config->dsr;

	if (!dsr)
		kfree(dsr);

	dsi_config->dsr = 0;
}
