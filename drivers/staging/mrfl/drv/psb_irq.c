/**************************************************************************
 * Copyright (c) 2007, Intel Corporation.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Intel funded Tungsten Graphics (http://www.tungstengraphics.com) to
 * develop this driver.
 *
 **************************************************************************/
/*
 */

#include <drm/drmP.h>
#include "psb_drv.h"
#include "psb_reg.h"
#include "psb_msvdx.h"
#include "mdfld_dsi_dbi_dsr.h"

#ifdef MEDFIELD
#include "pnw_topaz.h"
#endif
#ifdef MERRIFIELD
#include "tng_topaz.h"
#endif

#ifdef SUPPORT_VSP
#include "vsp.h"
#endif

#include "psb_intel_reg.h"
#include "pwr_mgmt.h"

#include "mdfld_dsi_dbi_dpu.h"

#include "psb_irq.h"
#include "psb_intel_hdmi.h"

#define KEEP_UNUSED_CODE 0


extern int drm_psb_smart_vsync;
/*
 * inline functions
 */

static inline u32 psb_pipestat(int pipe)
{
	if (pipe == 0)
		return PIPEASTAT;
	if (pipe == 1)
		return PIPEBSTAT;
	if (pipe == 2)
		return PIPECSTAT;
	BUG();
}

static inline u32 mid_pipe_event(int pipe)
{
	if (pipe == 0)
		return _PSB_PIPEA_EVENT_FLAG;
	if (pipe == 1)
		return _MDFLD_PIPEB_EVENT_FLAG;
	if (pipe == 2)
		return _MDFLD_PIPEC_EVENT_FLAG;
	BUG();
}

static inline u32 mid_pipe_vsync(int pipe)
{
	if (pipe == 0)
		return _PSB_VSYNC_PIPEA_FLAG;
	if (pipe == 1)
		return _PSB_VSYNC_PIPEB_FLAG;
	if (pipe == 2)
		return _MDFLD_PIPEC_VBLANK_FLAG;
	BUG();
}

static inline u32 mid_pipeconf(int pipe)
{
	if (pipe == 0)
		return PIPEACONF;
	if (pipe == 1)
		return PIPEBCONF;
	if (pipe == 2)
		return PIPECCONF;
	BUG();
}

void psb_enable_pipestat(struct drm_psb_private *dev_priv, int pipe, u32 mask)
{
	u32 power_island = pipe_to_island(pipe);

	if ((dev_priv->pipestat[pipe] & mask) != mask) {
		u32 reg = psb_pipestat(pipe);
		dev_priv->pipestat[pipe] |= mask;
		/* Enable the interrupt, clear any pending status */
		if (power_island_get(power_island)) {
			u32 writeVal = PSB_RVDC32(reg);
			writeVal |= (mask | (mask >> 16));
			PSB_WVDC32(writeVal, reg);
			(void)PSB_RVDC32(reg);
			power_island_put(power_island);
		}
	}
}

void psb_disable_pipestat(struct drm_psb_private *dev_priv, int pipe, u32 mask)
{
	u32 power_island = pipe_to_island(pipe);

	if ((dev_priv->pipestat[pipe] & mask) != 0) {
		u32 reg = psb_pipestat(pipe);
		dev_priv->pipestat[pipe] &= ~mask;
		if (power_island_get(power_island)) {
			if ((mask == PIPE_VBLANK_INTERRUPT_ENABLE) ||
					(mask == PIPE_TE_ENABLE)) {
				atomic_inc(&dev_priv->vblank_count[pipe]);
				wake_up_interruptible(&dev_priv->vsync_queue);
			}

			u32 writeVal = PSB_RVDC32(reg);
			writeVal &= ~mask;
			PSB_WVDC32(writeVal, reg);
			(void)PSB_RVDC32(reg);
			power_island_put(power_island);
		}
	}
}

void mid_enable_pipe_event(struct drm_psb_private *dev_priv, int pipe)
{
	u32 power_island = pipe_to_island(pipe);

	if (power_island_get(power_island)) {
		u32 pipe_event = mid_pipe_event(pipe);
		dev_priv->vdc_irq_mask |= pipe_event;
		PSB_WVDC32(~dev_priv->vdc_irq_mask, PSB_INT_MASK_R);
		PSB_WVDC32(dev_priv->vdc_irq_mask, PSB_INT_ENABLE_R);
		power_island_put(power_island);
	}
}

void mid_disable_pipe_event(struct drm_psb_private *dev_priv, int pipe)
{
	u32 power_island = pipe_to_island(pipe);

	if (dev_priv->pipestat[pipe] == 0) {
		if (power_island_get(power_island)) {
			u32 pipe_event = mid_pipe_event(pipe);
			dev_priv->vdc_irq_mask &= ~pipe_event;
			PSB_WVDC32(~dev_priv->vdc_irq_mask, PSB_INT_MASK_R);
			PSB_WVDC32(dev_priv->vdc_irq_mask, PSB_INT_ENABLE_R);
			power_island_put(power_island);
		}
	}
}

#if KEEP_UNUSED_CODE
/**
 * Check if we can disable vblank for video MIPI display
 *
 */
static void mid_check_vblank(struct drm_device *dev, uint32_t pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;
	static unsigned long cnt = 0;

	if (drm_psb_smart_vsync == 0) {
		if ((cnt++) % 600 == 0) {
			PSB_DEBUG_ENTRY("[vsync irq] 600 times !\n");
		}
		return;
	}

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
	if (dev_priv->dsr_idle_count > 50)
		dev_priv->b_is_in_idle = true;
	else
		dev_priv->dsr_idle_count++;
	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
}
#endif /* if KEEP_UNUSED_CODE */

u32 intel_vblank_count(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;

	return atomic_read(&dev_priv->vblank_count[pipe]);
}

/**
 *  Display controller interrupt handler for vsync/vblank.
 *
 *  Modified to handle the midi to hdmi clone 7/13/2012
 *      williamx.f.schmidt@intel.com
 */
static void mid_vblank_handler(struct drm_device *dev, uint32_t pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	drm_handle_vblank(dev, pipe);

	if (dev_priv->psb_vsync_handler)
		(*dev_priv->psb_vsync_handler)(dev, pipe);
}

/**
 * Display controller interrupt handler for pipe hdmi audio underrun.
 *
 */
static void mdfld_pipe_hdmi_audio_underrun(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	void *had_pvt_data = dev_priv->had_pvt_data;
	enum had_event_type event_type = HAD_EVENT_AUDIO_BUFFER_UNDERRUN;

	if (dev_priv->mdfld_had_event_callbacks)
		(*dev_priv->mdfld_had_event_callbacks) (event_type,
							had_pvt_data);
}

/**
 * Display controller interrupt handler for pipe hdmi audio buffer done.
 *
 */
static void mdfld_pipe_hdmi_audio_buffer_done(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (dev_priv->mdfld_had_event_callbacks)
		(*dev_priv->mdfld_had_event_callbacks)
		    (HAD_EVENT_AUDIO_BUFFER_DONE, dev_priv->had_pvt_data);
}

void psb_te_timer_func(unsigned long data)
{
	/*
	   struct drm_psb_private * dev_priv = (struct drm_psb_private *)data;
	   struct drm_device *dev = (struct drm_device *)dev_priv->dev;
	   uint32_t pipe = dev_priv->cur_pipe;
	   drm_handle_vblank(dev, pipe);
	   if( dev_priv->psb_vsync_handler != NULL)
	   (*dev_priv->psb_vsync_handler)(dev,pipe);
	 */
}

void mdfld_vsync_event_work(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
		container_of(work, struct drm_psb_private, vsync_event_work);
	int pipe = dev_priv->vsync_pipe;
	struct drm_device *dev = dev_priv->dev;

	mid_vblank_handler(dev, pipe);

	/* TODO: to report vsync event to HWC. */
	/*report vsync event*/
	/* mdfld_vsync_event(dev, pipe); */
	wake_up_interruptible(&dev_priv->vsync_queue);
}

void mdfld_te_handler_work(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
	    container_of(work, struct drm_psb_private, te_work);
	int pipe = dev_priv->te_pipe;
	struct drm_device *dev = dev_priv->dev;
	if (dev_priv->b_async_flip_enable) {
		/*
		* will sync with HDMI later
		*
		*	if (mipi_te_hdmi_vsync_check(dev, pipe)) {
		*/
			if (dev_priv->psb_vsync_handler != NULL)
				(*dev_priv->psb_vsync_handler)(dev, pipe);
		/*
		*	}
		*/
		mdfld_dsi_dsr_report_te(dev_priv->dsi_configs[0]);
	} else {
		#ifdef CONFIG_MID_DSI_DPU
			mdfld_dpu_update_panel(dev);
		#else
			mdfld_dbi_update_panel(dev, pipe);
		#endif
		drm_handle_vblank(dev, pipe);

		if (dev_priv->psb_vsync_handler != NULL)
			(*dev_priv->psb_vsync_handler) (dev, pipe);
	}
}

/**
 * Display controller interrupt handler for pipe event.
 *
 */
#define WAIT_STATUS_CLEAR_LOOP_COUNT 0xffff
static void mid_pipe_event_handler(struct drm_device *dev, uint32_t pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	uint32_t pipe_stat_val = 0;
	uint32_t pipe_stat_reg = psb_pipestat(pipe);
	uint32_t pipe_enable = dev_priv->pipestat[pipe];
	uint32_t pipe_status = dev_priv->pipestat[pipe] >> 16;
	uint32_t i = 0;
	unsigned long irq_flags;

	spin_lock_irqsave(&dev_priv->irqmask_lock, irq_flags);

	pipe_stat_val = PSB_RVDC32(pipe_stat_reg);
	pipe_stat_val &= pipe_enable | pipe_status;
	pipe_stat_val &= pipe_stat_val >> 16;

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irq_flags);

	/* clear the 2nd level interrupt status bits */
	/**
	* FIXME: shouldn't use while loop here. However, the interrupt
	* status 'sticky' bits cannot be cleared by setting '1' to that
	* bit once...
	*/
	for (i = 0; i < WAIT_STATUS_CLEAR_LOOP_COUNT; i++) {
		PSB_WVDC32(PSB_RVDC32(pipe_stat_reg), pipe_stat_reg);
		(void)PSB_RVDC32(pipe_stat_reg);

		if ((PSB_RVDC32(pipe_stat_reg) & pipe_status) == 0)
			break;
	}

	if (i == WAIT_STATUS_CLEAR_LOOP_COUNT)
		DRM_ERROR
		    ("%s, can't clear the status bits in pipe_stat_reg, its value = 0x%x. \n",
		     __FUNCTION__, PSB_RVDC32(pipe_stat_reg));

	if ((pipe_stat_val & PIPE_DPST_EVENT_STATUS) &&
	    (dev_priv->psb_dpst_state != NULL)) {
		uint32_t pwm_reg = 0;
		uint32_t hist_reg = 0;
		u32 irqCtrl = 0;
		struct dpst_guardband guardband_reg;
		struct dpst_ie_histogram_control ie_hist_cont_reg;

		hist_reg = PSB_RVDC32(HISTOGRAM_INT_CONTROL);

		/* Determine if this is histogram or pwm interrupt */
		if (hist_reg & HISTOGRAM_INT_CTRL_CLEAR) {
			/* Notify UM of histogram interrupt */
			psb_dpst_notify_change_um(DPST_EVENT_HIST_INTERRUPT,
						  dev_priv->psb_dpst_state);

			/* disable dpst interrupts */
			guardband_reg.data = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
			guardband_reg.interrupt_enable = 0;
			guardband_reg.interrupt_status = 1;
			PSB_WVDC32(guardband_reg.data, HISTOGRAM_INT_CONTROL);

			ie_hist_cont_reg.data =
			    PSB_RVDC32(HISTOGRAM_LOGIC_CONTROL);
			ie_hist_cont_reg.ie_histogram_enable = 0;
			PSB_WVDC32(ie_hist_cont_reg.data,
				   HISTOGRAM_LOGIC_CONTROL);

			irqCtrl = PSB_RVDC32(PIPEASTAT);
			irqCtrl &= ~PIPE_DPST_EVENT_ENABLE;
			PSB_WVDC32(irqCtrl, PIPEASTAT);
		}
		pwm_reg = PSB_RVDC32(PWM_CONTROL_LOGIC);
		if ((pwm_reg & PWM_PHASEIN_INT_ENABLE) &&
		    !(pwm_reg & PWM_PHASEIN_ENABLE)) {
			/* Notify UM of the phase complete */
			psb_dpst_notify_change_um(DPST_EVENT_PHASE_COMPLETE,
						  dev_priv->psb_dpst_state);

			/* Temporarily get phase mngr ready to generate
			 * another interrupt until this can be moved to
			 * user mode */
			/* PSB_WVDC32(pwm_reg | 0x80010100 | PWM_PHASEIN_ENABLE,
			   PWM_CONTROL_LOGIC); */
		}
	}

	if (pipe_stat_val & PIPE_VBLANK_STATUS) {
		dev_priv->vsync_pipe = pipe;
		atomic_inc(&dev_priv->vblank_count[pipe]);
		schedule_work(&dev_priv->vsync_event_work);
	}

	if (pipe_stat_val & PIPE_TE_STATUS) {
		dev_priv->te_pipe = pipe;
		atomic_inc(&dev_priv->vblank_count[pipe]);
		schedule_work(&dev_priv->te_work);
	}

	if (pipe_stat_val & PIPE_HDMI_AUDIO_UNDERRUN_STATUS) {
		mdfld_pipe_hdmi_audio_underrun(dev);
	}

	if (pipe_stat_val & PIPE_HDMI_AUDIO_BUFFER_DONE_STATUS) {
		mdfld_pipe_hdmi_audio_buffer_done(dev);
	}
}

/**
 * Display controller interrupt handler.
 */
static void psb_vdc_interrupt(struct drm_device *dev, uint32_t vdc_stat)
{

	if (vdc_stat & _PSB_PIPEA_EVENT_FLAG) {
		mid_pipe_event_handler(dev, 0);
	}

	if (vdc_stat & _MDFLD_PIPEB_EVENT_FLAG) {
		mid_pipe_event_handler(dev, 1);
	}

	if (vdc_stat & _MDFLD_PIPEC_EVENT_FLAG) {
		mid_pipe_event_handler(dev, 2);
	}
}

irqreturn_t psb_irq_handler(DRM_IRQ_ARGS)
{
	struct drm_device *dev = (struct drm_device *)arg;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	uint32_t vdc_stat, dsp_int = 0, sgx_int = 0, msvdx_int = 0;
	uint32_t topaz_int = 0, vsp_int = 0;
	int handled = 0;
	unsigned long irq_flags;

	/*      PSB_DEBUG_ENTRY("\n"); */

	spin_lock_irqsave(&dev_priv->irqmask_lock, irq_flags);

	vdc_stat = PSB_RVDC32(PSB_INT_IDENTITY_R);

	if (vdc_stat & _MDFLD_DISP_ALL_IRQ_FLAG) {
		PSB_DEBUG_IRQ("Got DISP interrupt\n");
		dsp_int = 1;
	}

	if (vdc_stat & _PSB_IRQ_SGX_FLAG) {
		PSB_DEBUG_IRQ("Got SGX interrupt\n");
		sgx_int = 1;
	}
	if (vdc_stat & _PSB_IRQ_MSVDX_FLAG) {
		PSB_DEBUG_IRQ("Got MSVDX interrupt\n");
		msvdx_int = 1;
	}

	if (vdc_stat & _LNC_IRQ_TOPAZ_FLAG) {
		PSB_DEBUG_IRQ("Got TOPAX interrupt\n");
		topaz_int = 1;
	}

	if (vdc_stat & _TNG_IRQ_VSP_FLAG) {
		PSB_DEBUG_IRQ("Got VSP interrupt\n");
		vsp_int = 1;
	}

	vdc_stat &= dev_priv->vdc_irq_mask;
	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irq_flags);

	if (dsp_int && ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
		psb_vdc_interrupt(dev, vdc_stat);
		handled = 1;
	}

	if (msvdx_int && (IS_FLDS(dev)
			  || ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND))) {
		psb_msvdx_interrupt(dev);
		handled = 1;
	}
#ifdef MEDFIELD
	if ((IS_MDFLD(dev) && topaz_int)) {
		pnw_topaz_interrupt(dev);
		handled = 1;
	}
#endif
#ifdef MERRIFIELD
	if ((IS_MRFLD(dev) && topaz_int)) {
		tng_topaz_interrupt(dev);
		handled = 1;
	}
#endif
#ifdef SUPPORT_VSP
	if (vsp_int) {
		vsp_interrupt(dev);
		handled = 1;
	}
#endif

	if (sgx_int) {
		if (PVRSRVInterrupt(dev) != 0)
			handled = 1;
	}

	PSB_WVDC32(vdc_stat, PSB_INT_IDENTITY_R);
	(void)PSB_RVDC32(PSB_INT_IDENTITY_R);
	DRM_READMEMORYBARRIER();

	if (!handled)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

void psb_irq_preinstall(struct drm_device *dev)
{
	psb_irq_preinstall_islands(dev, OSPM_ALL_ISLANDS);
}

/**
 * FIXME: should I remove display irq enable here??
 */
void psb_irq_preinstall_islands(struct drm_device *dev, int hw_islands)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;

	PSB_DEBUG_IRQ("\n");

	if (dev_priv->b_dsr_enable)
		dev_priv->exit_idle(dev, MDFLD_DSR_2D_3D, 0, true);

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	if (hw_islands & OSPM_DISPLAY_ISLAND) {
		if (ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
			if (dev->vblank_enabled[0])
				dev_priv->vdc_irq_mask |= _PSB_PIPEA_EVENT_FLAG;
			if (dev->vblank_enabled[1])
				dev_priv->vdc_irq_mask |=
				    _MDFLD_PIPEB_EVENT_FLAG;
			if (dev->vblank_enabled[2])
				dev_priv->vdc_irq_mask |=
				    _MDFLD_PIPEC_EVENT_FLAG;
		}
	}
	if (hw_islands & OSPM_GRAPHICS_ISLAND) {
		dev_priv->vdc_irq_mask |= _PSB_IRQ_SGX_FLAG;
	}

	if (hw_islands & OSPM_VIDEO_DEC_ISLAND)
		if (IS_MID(dev) && ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND))
			dev_priv->vdc_irq_mask |= _PSB_IRQ_MSVDX_FLAG;

	if (hw_islands & OSPM_VIDEO_ENC_ISLAND)
		if (IS_MID(dev) && ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND))
			dev_priv->vdc_irq_mask |= _LNC_IRQ_TOPAZ_FLAG;

	if (hw_islands & OSPM_VIDEO_VPP_ISLAND)
		if (IS_MID(dev))
			dev_priv->vdc_irq_mask |= _TNG_IRQ_VSP_FLAG;

	/*This register is safe even if display island is off*/
	PSB_WVDC32(~dev_priv->vdc_irq_mask, PSB_INT_MASK_R);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
}

int psb_irq_postinstall(struct drm_device *dev)
{
	return psb_irq_postinstall_islands(dev, OSPM_ALL_ISLANDS);
}

int psb_irq_postinstall_islands(struct drm_device *dev, int hw_islands)
{

	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;

	PSB_DEBUG_IRQ("\n");

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	/*This register is safe even if display island is off */
	PSB_WVDC32(dev_priv->vdc_irq_mask, PSB_INT_ENABLE_R);

	if (IS_MID(dev) && !dev_priv->topaz_disabled)
		if (hw_islands & OSPM_VIDEO_ENC_ISLAND)
			if (ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND)) {
#ifdef MEDFIELD
				if (IS_MDFLD(dev))
					pnw_topaz_enableirq(dev);
#endif
#ifdef MERRIFIELD
				if (IS_MRFLD(dev))
					tng_topaz_enableirq(dev);
#endif

			}

	if (hw_islands & OSPM_VIDEO_DEC_ISLAND)
		psb_msvdx_enableirq(dev);

#ifdef SUPPORT_VSP
	if (hw_islands & OSPM_VIDEO_VPP_ISLAND)
		vsp_enableirq(dev);
#endif

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);

	return 0;
}

void psb_irq_uninstall(struct drm_device *dev)
{
	psb_irq_uninstall_islands(dev, OSPM_ALL_ISLANDS);
}

void psb_irq_uninstall_islands(struct drm_device *dev, int hw_islands)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;

	PSB_DEBUG_IRQ("\n");

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	if (hw_islands & OSPM_DISPLAY_ISLAND)
		dev_priv->vdc_irq_mask &= _PSB_IRQ_SGX_FLAG |
					  _PSB_IRQ_MSVDX_FLAG |
					  _LNC_IRQ_TOPAZ_FLAG |
					  _TNG_IRQ_VSP_FLAG;

	/*TODO: remove follwoing code */
	if (hw_islands & OSPM_GRAPHICS_ISLAND) {
		dev_priv->vdc_irq_mask &= ~_PSB_IRQ_SGX_FLAG;
	}

	if ((hw_islands & OSPM_VIDEO_DEC_ISLAND) && IS_MID(dev))
		dev_priv->vdc_irq_mask &= ~_PSB_IRQ_MSVDX_FLAG;

	if ((hw_islands & OSPM_VIDEO_ENC_ISLAND) && IS_MID(dev))
		dev_priv->vdc_irq_mask &= ~_LNC_IRQ_TOPAZ_FLAG;

	if ((hw_islands & OSPM_VIDEO_VPP_ISLAND) && IS_MID(dev))
		dev_priv->vdc_irq_mask &= ~_TNG_IRQ_VSP_FLAG;

	/*These two registers are safe even if display island is off*/
	PSB_WVDC32(~dev_priv->vdc_irq_mask, PSB_INT_MASK_R);
	PSB_WVDC32(dev_priv->vdc_irq_mask, PSB_INT_ENABLE_R);

	wmb();

	/*This register is safe even if display island is off */
	PSB_WVDC32(PSB_RVDC32(PSB_INT_IDENTITY_R), PSB_INT_IDENTITY_R);

	if (IS_MID(dev) && !dev_priv->topaz_disabled)
		if (hw_islands & OSPM_VIDEO_ENC_ISLAND)
			if (ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND)) {
#ifdef MEDFIELD
				if (IS_MDFLD(dev))
					pnw_topaz_disableirq(dev);
#endif
#ifdef MERRIFIELD
				if (IS_MRFLD(dev))
					tng_topaz_disableirq(dev);
#endif
			}

	if (hw_islands & OSPM_VIDEO_DEC_ISLAND)
		if (ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND))
			psb_msvdx_disableirq(dev);

#ifdef SUPPORT_VSP
	if (hw_islands & OSPM_VIDEO_VPP_ISLAND)
		if (ospm_power_is_hw_on(OSPM_VIDEO_VPP_ISLAND))
			vsp_disableirq(dev);
#endif
	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
}

void psb_irq_turn_on_dpst(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	u32 hist_reg;
	u32 pwm_reg;

	/* FIXME: revisit the power island when touching the DPST feature. */
	if (power_island_get(OSPM_DISPLAY_A)) {
		PSB_WVDC32(BIT31, HISTOGRAM_LOGIC_CONTROL);
		hist_reg = PSB_RVDC32(HISTOGRAM_LOGIC_CONTROL);
		PSB_WVDC32(BIT31, HISTOGRAM_INT_CONTROL);
		hist_reg = PSB_RVDC32(HISTOGRAM_INT_CONTROL);

		PSB_WVDC32(0x80010100, PWM_CONTROL_LOGIC);
		pwm_reg = PSB_RVDC32(PWM_CONTROL_LOGIC);
		PSB_WVDC32(pwm_reg | PWM_PHASEIN_ENABLE |
			   PWM_PHASEIN_INT_ENABLE, PWM_CONTROL_LOGIC);
		pwm_reg = PSB_RVDC32(PWM_CONTROL_LOGIC);

		psb_enable_pipestat(dev_priv, 0, PIPE_DPST_EVENT_ENABLE);

		hist_reg = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
		PSB_WVDC32(hist_reg | HISTOGRAM_INT_CTRL_CLEAR,
			   HISTOGRAM_INT_CONTROL);
		pwm_reg = PSB_RVDC32(PWM_CONTROL_LOGIC);
		PSB_WVDC32(pwm_reg | 0x80010100 | PWM_PHASEIN_ENABLE,
			   PWM_CONTROL_LOGIC);

		power_island_put(OSPM_DISPLAY_A);
	}
}

int psb_irq_enable_dpst(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;

	PSB_DEBUG_ENTRY("\n");

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	/* enable DPST */
	mid_enable_pipe_event(dev_priv, 0);
	psb_irq_turn_on_dpst(dev);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	return 0;
}

void psb_irq_turn_off_dpst(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	u32 hist_reg;
	u32 pwm_reg;

	/* FIXME: revisit the power island when touching the DPST feature. */
	if (power_island_get(OSPM_DISPLAY_A)) {
		PSB_WVDC32(0x00000000, HISTOGRAM_INT_CONTROL);
		hist_reg = PSB_RVDC32(HISTOGRAM_INT_CONTROL);

		psb_disable_pipestat(dev_priv, 0, PIPE_DPST_EVENT_ENABLE);

		pwm_reg = PSB_RVDC32(PWM_CONTROL_LOGIC);
		PSB_WVDC32(pwm_reg & !(PWM_PHASEIN_INT_ENABLE),
			   PWM_CONTROL_LOGIC);
		pwm_reg = PSB_RVDC32(PWM_CONTROL_LOGIC);

		power_island_put(OSPM_DISPLAY_A);
	}
}

int psb_irq_disable_dpst(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;

	PSB_DEBUG_ENTRY("\n");

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	mid_disable_pipe_event(dev_priv, 0);
	psb_irq_turn_off_dpst(dev);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);

	return 0;
}

#ifdef PSB_FIXME
static int psb_vblank_do_wait(struct drm_device *dev,
			      unsigned int *sequence, atomic_t * counter)
{
	unsigned int cur_vblank;
	int ret = 0;
	DRM_WAIT_ON(ret, dev->vbl_queue, 3 * DRM_HZ,
		    (((cur_vblank = atomic_read(counter))
		      - *sequence) <= (1 << 23)));
	*sequence = cur_vblank;

	return ret;
}
#endif

/*
 * It is used to enable VBLANK interrupt
 */
int psb_enable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;
	uint32_t reg_val = 0;
	uint32_t pipeconf_reg = mid_pipeconf(pipe);
	mdfld_dsi_encoder_t encoder_type;
	u32 power_island = pipe_to_island(pipe);

	PSB_DEBUG_ENTRY("\n");

	encoder_type = is_panel_vid_or_cmd(dev);
	if (IS_MRFLD(dev) &&
		(encoder_type == MDFLD_DSI_ENCODER_DBI) &&
			(pipe == 0))
				return 0;

	if (power_island_get(power_island)) {
		reg_val = REG_READ(pipeconf_reg);
		power_island_put(power_island);
	}

	if (!(reg_val & PIPEACONF_ENABLE))
		return -EINVAL;

	dev_priv->b_vblank_enable = true;

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	drm_psb_disable_vsync = 0;
	mid_enable_pipe_event(dev_priv, pipe);
	psb_enable_pipestat(dev_priv, pipe, PIPE_VBLANK_INTERRUPT_ENABLE);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	PSB_DEBUG_ENTRY("%s: Enabled VBlank for pipe %d\n", __func__, pipe);

	return 0;
}

/*
 * It is used to disable VBLANK interrupt
 */
void psb_disable_vblank(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;
	mdfld_dsi_encoder_t encoder_type;

	PSB_DEBUG_ENTRY("\n");

	encoder_type = is_panel_vid_or_cmd(dev);
#if 0
	if (IS_MRFLD(dev) && (encoder_type == MDFLD_DSI_ENCODER_DBI))
		mdfld_disable_te(dev, pipe);
#endif
	dev_priv->b_vblank_enable = false;

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	drm_psb_disable_vsync = 1;
	mid_disable_pipe_event(dev_priv, pipe);
	psb_disable_pipestat(dev_priv, pipe, PIPE_VBLANK_INTERRUPT_ENABLE);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	PSB_DEBUG_ENTRY("%s: Disabled VBlank for pipe %d\n", __func__, pipe);
}

/* Called from drm generic code, passed a 'crtc', which
 * we use as a pipe index
 */
u32 psb_get_vblank_counter(struct drm_device *dev, int pipe)
{
	uint32_t high_frame = PIPEAFRAMEHIGH;
	uint32_t low_frame = PIPEAFRAMEPIXEL;
	uint32_t pipeconf_reg = PIPEACONF;
	uint32_t reg_val = 0;
	uint32_t high1 = 0, high2 = 0, low = 0, count = 0;
	u32 power_island = pipe_to_island(pipe);

	switch (pipe) {
	case 0:
		break;
	case 1:
		high_frame = PIPEBFRAMEHIGH;
		low_frame = PIPEBFRAMEPIXEL;
		pipeconf_reg = PIPEBCONF;
		break;
	case 2:
		high_frame = PIPECFRAMEHIGH;
		low_frame = PIPECFRAMEPIXEL;
		pipeconf_reg = PIPECCONF;
		break;
	default:
		DRM_ERROR("%s, invalded pipe.\n", __func__);
		return 0;
	}

	if (!power_island_get(power_island))
		return 0;

	reg_val = REG_READ(pipeconf_reg);

	if (!(reg_val & PIPEACONF_ENABLE)) {
		DRM_DEBUG("trying to get vblank count for disabled pipe %d\n",
			  pipe);
		goto psb_get_vblank_counter_exit;
	}

	/*
	 * High & low register fields aren't synchronized, so make sure
	 * we get a low value that's stable across two reads of the high
	 * register.
	 */
	do {
		high1 = ((REG_READ(high_frame) & PIPE_FRAME_HIGH_MASK) >>
			 PIPE_FRAME_HIGH_SHIFT);
		low = ((REG_READ(low_frame) & PIPE_FRAME_LOW_MASK) >>
		       PIPE_FRAME_LOW_SHIFT);
		high2 = ((REG_READ(high_frame) & PIPE_FRAME_HIGH_MASK) >>
			 PIPE_FRAME_HIGH_SHIFT);
	} while (high1 != high2);

	count = (high1 << 8) | low;

 psb_get_vblank_counter_exit:

	power_island_put(power_island);

	return count;
}

/*
 * It is used to enable TE interrupt
 */
int mdfld_enable_te(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;
	uint32_t reg_val = 0;
	uint32_t pipeconf_reg = mid_pipeconf(pipe);
	u32 power_island = pipe_to_island(pipe);

	if (power_island_get(power_island)) {
		reg_val = REG_READ(pipeconf_reg);
		power_island_put(power_island);
	}

	if (!(reg_val & PIPEACONF_ENABLE))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	mid_enable_pipe_event(dev_priv, pipe);
	psb_enable_pipestat(dev_priv, pipe, PIPE_TE_ENABLE);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	PSB_DEBUG_ENTRY("%s: Enabled TE for pipe %d\n", __func__, pipe);

	return 0;
}

/*
 * It is used to disable TE interrupt
 */
void mdfld_disable_te(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	mid_disable_pipe_event(dev_priv, pipe);
	psb_disable_pipestat(dev_priv, pipe, PIPE_TE_ENABLE);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	PSB_DEBUG_ENTRY("%s: Disabled TE for pipe %d\n", __func__, pipe);
}

int mid_irq_enable_hdmi_audio(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;
	u32 reg_val = 0, mask = 0;

	if (power_island_get(OSPM_DISPLAY_B)) {
		reg_val = REG_READ(PIPEBCONF);
		power_island_put(OSPM_DISPLAY_B);
	}

	if (!(reg_val & PIPEACONF_ENABLE))
		return -EINVAL;

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	/* enable HDMI audio interrupt */
	mid_enable_pipe_event(dev_priv, 1);
	dev_priv->pipestat[1] &= ~PIPE_HDMI_AUDIO_INT_MASK;
	mask = dev_priv->hdmi_audio_interrupt_mask;
	psb_enable_pipestat(dev_priv, 1, mask);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	return 0;
}

int mid_irq_disable_hdmi_audio(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;
	unsigned long irqflags;

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);

	mid_disable_pipe_event(dev_priv, 1);
	psb_disable_pipestat(dev_priv, 1, PIPE_HDMI_AUDIO_INT_MASK);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	return 0;
}
