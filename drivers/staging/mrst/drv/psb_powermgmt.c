/**************************************************************************
 * Copyright (c) 2009, Intel Corporation.
 * All Rights Reserved.

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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *    Benjamin Defnet <benjamin.r.defnet@intel.com>
 *    Rajesh Poornachandran <rajesh.poornachandran@intel.com>
 *
 */
#include "psb_powermgmt.h"
#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_msvdx.h"
#include "lnc_topaz.h"
#include "pnw_topaz.h"
#include "mdfld_gl3.h"
#include <linux/mutex.h>
#include "lnc_topaz_hw_reg.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include "psb_intel_hdmi.h"
#include "mdfld_ti_tpd.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_lvds_bridge.h"
#ifdef CONFIG_GFX_RTPM
#include <linux/pm_runtime.h>
#endif
#include <linux/atomic.h>

#define SUPPORT_EARLY_SUSPEND 1

#if SUPPORT_EARLY_SUSPEND
#include <linux/earlysuspend.h>
#endif /* if SUPPORT_EARLY_SUSPEND */

#undef OSPM_GFX_DPK
#define SCU_CMD_VPROG2  0xe3

struct drm_device *gpDrmDevice = NULL;
static struct mutex g_ospm_mutex;
static bool gbSuspendInProgress = false;
static bool gbResumeInProgress = false;
static int g_hw_power_status_mask;
static atomic_t g_display_access_count;
static atomic_t g_graphics_access_count;
static atomic_t g_videoenc_access_count;
static atomic_t g_videodec_access_count;
extern u32 DISP_PLANEB_STATUS;

static bool gbSuspended = false;
bool gbgfxsuspended = false;

void acquire_ospm_lock(void)
{
	mutex_lock(&g_ospm_mutex);
}

void release_ospm_lock(void)
{
	mutex_unlock(&g_ospm_mutex);
}

#if SUPPORT_EARLY_SUSPEND
/*
 * gfx_early_suspend
 *
 */
static void gfx_early_suspend(struct early_suspend *h);
static void gfx_late_resume(struct early_suspend *h);

static struct early_suspend gfx_early_suspend_desc = {
        .level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
        .suspend = gfx_early_suspend,
        .resume = gfx_late_resume,
};
#endif /* if SUPPORT_EARLY_SUSPEND */

static int ospm_runtime_pm_topaz_suspend(struct drm_device *dev)
{
	int ret = 0;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	struct pnw_topaz_private *pnw_topaz_priv = dev_priv->topaz_private;
	struct psb_video_ctx *pos, *n;
	int encode_ctx = 0, encode_running = 0;


	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		int entrypoint = pos->ctx_type & 0xff;
		if (entrypoint == VAEntrypointEncSlice ||
		    entrypoint == VAEntrypointEncPicture) {
			encode_ctx = 1;
			break;
		}
	}

	/* have encode context, but not started, or is just closed */
	if (encode_ctx && dev_priv->topaz_ctx)
		encode_running = 1;

	if (!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND))
		goto out;

	if (atomic_read(&g_videoenc_access_count)) {
		ret = -1;
		goto out;
	}

#ifdef CONFIG_MDFD_VIDEO_DECODE
	if (IS_MRST(dev)) {
		if (lnc_check_topaz_idle(dev)) {
			ret = -2;
			goto out;
		}
	}

	if (IS_MDFLD(dev)) {
		if (pnw_check_topaz_idle(dev)) {
			ret = -2;
			goto out;
		}
	}
	psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
	if (IS_MRST(dev)) {
		if (encode_running)
			lnc_topaz_save_mtx_state(gpDrmDevice);
		TOPAZ_NEW_PMSTATE(dev, topaz_priv, PSB_PMSTATE_POWERDOWN);
	}

	if (IS_MDFLD(dev)) {
		if (encode_running)
			pnw_topaz_save_mtx_state(gpDrmDevice);
		PNW_TOPAZ_NEW_PMSTATE(dev, pnw_topaz_priv,
				PSB_PMSTATE_POWERDOWN);
	}
#endif
	ospm_power_island_down(OSPM_VIDEO_ENC_ISLAND);
out:
	return ret;
}


static int ospm_runtime_pm_msvdx_resume(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	/*printk(KERN_ALERT "ospm_runtime_pm_msvdx_resume\n");*/

#ifdef CONFIG_MDFD_VIDEO_DECODE
	MSVDX_NEW_PMSTATE(dev, msvdx_priv, PSB_PMSTATE_POWERUP);

	psb_msvdx_restore_context(dev);
#endif

	return 0;
}

static int ospm_runtime_pm_topaz_resume(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	struct pnw_topaz_private *pnw_topaz_priv = dev_priv->topaz_private;
	struct psb_video_ctx *pos, *n;
	int encode_ctx = 0, encode_running = 0;

	/*printk(KERN_ALERT "ospm_runtime_pm_topaz_resume\n");*/

	list_for_each_entry_safe(pos, n, &dev_priv->video_ctx, head) {
		int entrypoint = pos->ctx_type & 0xff;
		if (entrypoint == VAEntrypointEncSlice ||
		    entrypoint == VAEntrypointEncPicture) {
			encode_ctx = 1;
			break;
		}
	}

	/* have encode context, but not started, or is just closed */
	if (encode_ctx && dev_priv->topaz_ctx)
		encode_running = 1;

	if (encode_ctx)
		PSB_DEBUG_PM("Topaz: has encode context, running=%d\n",
			     encode_running);
	else
		PSB_DEBUG_PM("Topaz: no encode running\n");

#ifdef CONFIG_MDFD_VIDEO_DECODE
	if (IS_MRST(dev)) {
		if (encode_running) { /* has encode session running */
			psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			lnc_topaz_restore_mtx_state(gpDrmDevice);
		}
		TOPAZ_NEW_PMSTATE(dev, topaz_priv, PSB_PMSTATE_POWERUP);
	}

	if (IS_MDFLD(dev)) {
		if (encode_running) { /* has encode session running */
			psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			pnw_topaz_restore_mtx_state(gpDrmDevice);
		}
		PNW_TOPAZ_NEW_PMSTATE(dev, pnw_topaz_priv, PSB_PMSTATE_POWERUP);
	}
#endif
	return 0;
}

#ifdef FIX_OSPM_POWER_DOWN
void ospm_apm_power_down_msvdx(struct drm_device *dev)
{
	return;
	mutex_lock(&g_ospm_mutex);

	if (atomic_read(&g_videodec_access_count))
		goto out;
	if (psb_check_msvdx_idle(dev))
		goto out;

	gbSuspendInProgress = true;
	psb_msvdx_save_context(dev);
#ifdef FIXME_MRST_VIDEO_DEC
	ospm_power_island_down(OSPM_VIDEO_DEC_ISLAND);
#endif
	gbSuspendInProgress = false;
out:
	mutex_unlock(&g_ospm_mutex);
	return;
}

void ospm_apm_power_down_topaz(struct drm_device *dev)
{
	return; /* todo for OSPM */

	mutex_lock(&g_ospm_mutex);

	if (atomic_read(&g_videoenc_access_count))
		goto out;
	if (lnc_check_topaz_idle(dev))
		goto out;

	gbSuspendInProgress = true;
	lnc_topaz_save_mtx_state(dev);
	ospm_power_island_down(OSPM_VIDEO_ENC_ISLAND);
	gbSuspendInProgress = false;
out:
	mutex_unlock(&g_ospm_mutex);
	return;
}
#else
void ospm_apm_power_down_msvdx(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	mutex_lock(&g_ospm_mutex);
	if (!ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND))
		goto out;

	if (atomic_read(&g_videodec_access_count))
		goto out;
#ifdef CONFIG_MDFD_VIDEO_DECODE
	if (psb_check_msvdx_idle(dev))
		goto out;

	gbSuspendInProgress = true;
	psb_msvdx_save_context(dev);
#endif
	ospm_power_island_down(OSPM_VIDEO_DEC_ISLAND);
#ifdef CONFIG_MDFD_GL3
	/* Power off GL3 */
	ospm_power_island_down(OSPM_GL3_CACHE_ISLAND);
#endif
	gbSuspendInProgress = false;
	MSVDX_NEW_PMSTATE(dev, msvdx_priv, PSB_PMSTATE_POWERDOWN);
out:
	mutex_unlock(&g_ospm_mutex);
	return;
}

void ospm_apm_power_down_topaz(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	struct pnw_topaz_private *pnw_topaz_priv = dev_priv->topaz_private;

	mutex_lock(&g_ospm_mutex);

	if (!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND))
		goto out;
	if (atomic_read(&g_videoenc_access_count))
		goto out;
#ifdef CONFIG_MDFD_VIDEO_DECODE
	if (IS_MRST(dev))
		if (lnc_check_topaz_idle(dev))
			goto out;
	if (IS_MDFLD(dev))
		if (pnw_check_topaz_idle(dev))
			goto out;
	gbSuspendInProgress = true;
	if (IS_MRST(dev)) {
		psb_irq_uninstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
        	lnc_topaz_save_mtx_state(dev);
		TOPAZ_NEW_PMSTATE(dev, topaz_priv, PSB_PMSTATE_POWERDOWN);
	}
	if (IS_MDFLD(dev)) {
        	psb_irq_uninstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
		pnw_topaz_save_mtx_state(gpDrmDevice);
		PNW_TOPAZ_NEW_PMSTATE(dev, pnw_topaz_priv, PSB_PMSTATE_POWERDOWN);
	}
	ospm_power_island_down(OSPM_VIDEO_ENC_ISLAND);
#endif

#ifdef CONFIG_MDFD_GL3
	/* Power off GL3 */
	if (IS_MDFLD(dev))
		ospm_power_island_down(OSPM_GL3_CACHE_ISLAND);
#endif

	gbSuspendInProgress = false;
out:
	mutex_unlock(&g_ospm_mutex);
	return;
}
#endif
/*
 * ospm_power_init
 *
 * Description: Initialize this ospm power management module
 */
void ospm_power_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = (struct drm_psb_private *)dev->dev_private;
	unsigned long flags;

	gpDrmDevice = dev;

	mutex_init(&g_ospm_mutex);
	spin_lock_init(&dev_priv->ospm_lock);

	spin_lock_irqsave(&dev_priv->ospm_lock, flags);
	g_hw_power_status_mask = OSPM_ALL_ISLANDS;
	spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);

	atomic_set(&g_display_access_count, 0);
	atomic_set(&g_graphics_access_count, 0);
	atomic_set(&g_videoenc_access_count, 0);
	atomic_set(&g_videodec_access_count, 0);

#if SUPPORT_EARLY_SUSPEND
	register_early_suspend(&gfx_early_suspend_desc);
#endif /* if SUPPORT_EARLY_SUSPEND */

#ifdef OSPM_STAT
	dev_priv->graphics_state = PSB_PWR_STATE_ON;
	dev_priv->gfx_last_mode_change = jiffies;
	dev_priv->gfx_on_time = 0;
	dev_priv->gfx_off_time = 0;
#endif


}

/*
 * ospm_power_uninit
 *
 * Description: Uninitialize this ospm power management module
 */
void ospm_power_uninit(void)
{
#if SUPPORT_EARLY_SUSPEND
    unregister_early_suspend(&gfx_early_suspend_desc);
#endif /* if SUPPORT_EARLY_SUSPEND */

    mutex_destroy(&g_ospm_mutex);
#ifdef CONFIG_GFX_RTPM
	pm_runtime_get_noresume(&gpDrmDevice->pdev->dev);
#endif
}
/*
* ospm_post_init
*
* Description: Power gate unused GFX & Display islands.
*/
void ospm_post_init(struct drm_device *dev)
{
	u32 dc_islands  = 0;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;

#ifndef CONFIG_MDFD_GL3
	ospm_power_island_down(OSPM_GL3_CACHE_ISLAND);
#endif
	/*Save & Power gate un-used display islands.*/
	mdfld_save_display(dev);

	if (!(dev_priv->panel_desc & DISPLAY_A))
		dc_islands |= OSPM_DISPLAY_A_ISLAND;

	if (!(dev_priv->panel_desc & DISPLAY_B))
		dc_islands |= OSPM_DISPLAY_B_ISLAND;

	if (!(dev_priv->panel_desc & DISPLAY_C))
		dc_islands |= OSPM_DISPLAY_C_ISLAND;

	if (!(dev_priv->panel_desc))
		dc_islands |= OSPM_MIPI_ISLAND;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT
		"%s dc_islands: %x to be powered OFF\n",
		__func__, dc_islands);
#endif
	/*
	If pmu_nc_set_power_state fails then accessing HW
	reg would result in a crash - IERR/Fabric error.
	*/
	if (pmu_nc_set_power_state(dc_islands,
		OSPM_ISLAND_DOWN, OSPM_REG_TYPE))
		BUG();

/* if HDMI is disabled in the kernel .config, then we want to
disable these MSIC power rails permanently.  */
#ifndef CONFIG_MDFD_HDMI
	if (IS_MDFLD_OLD(dev)) {
		/* turn off HDMI power rails */
		intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_OFF);
		intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_OFF);
	}
#endif

}

/*
 * save_display_registers
 *
 * Description: We are going to suspend so save current display
 * register state.
 */
static int save_display_registers(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct drm_crtc * crtc;
	struct drm_connector * connector;
	int i;

	/* Display arbitration control + watermarks */
	dev_priv->saveDSPARB = PSB_RVDC32(DSPARB);
	dev_priv->saveDSPFW1 = PSB_RVDC32(DSPFW1);
	dev_priv->saveDSPFW2 = PSB_RVDC32(DSPFW2);
	dev_priv->saveDSPFW3 = PSB_RVDC32(DSPFW3);
	dev_priv->saveDSPFW4 = PSB_RVDC32(DSPFW4);
	dev_priv->saveDSPFW5 = PSB_RVDC32(DSPFW5);
	dev_priv->saveDSPFW6 = PSB_RVDC32(DSPFW6);
	dev_priv->saveCHICKENBIT = PSB_RVDC32(DSPCHICKENBIT);

	if (IS_MRST(dev)) {
		/* Pipe & plane A info */
		dev_priv->savePIPEACONF = PSB_RVDC32(PIPEACONF);
		dev_priv->savePIPEASRC = PSB_RVDC32(PIPEASRC);
		dev_priv->saveFPA0 = PSB_RVDC32(MRST_FPA0);
		dev_priv->saveFPA1 = PSB_RVDC32(MRST_FPA1);
		dev_priv->saveDPLL_A = PSB_RVDC32(MRST_DPLL_A);
		dev_priv->saveHTOTAL_A = PSB_RVDC32(HTOTAL_A);
		dev_priv->saveHBLANK_A = PSB_RVDC32(HBLANK_A);
		dev_priv->saveHSYNC_A = PSB_RVDC32(HSYNC_A);
		dev_priv->saveVTOTAL_A = PSB_RVDC32(VTOTAL_A);
		dev_priv->saveVBLANK_A = PSB_RVDC32(VBLANK_A);
		dev_priv->saveVSYNC_A = PSB_RVDC32(VSYNC_A);
		dev_priv->saveBCLRPAT_A = PSB_RVDC32(BCLRPAT_A);
		dev_priv->saveDSPACNTR = PSB_RVDC32(DSPACNTR);
		dev_priv->saveDSPASTRIDE = PSB_RVDC32(DSPASTRIDE);
		dev_priv->saveDSPAADDR = PSB_RVDC32(DSPABASE);
		dev_priv->saveDSPASURF = PSB_RVDC32(DSPASURF);
		dev_priv->saveDSPALINOFF = PSB_RVDC32(DSPALINOFF);
		dev_priv->saveDSPATILEOFF = PSB_RVDC32(DSPATILEOFF);

		/*save cursor regs*/
		dev_priv->saveDSPACURSOR_CTRL = PSB_RVDC32(CURACNTR);
		dev_priv->saveDSPACURSOR_BASE = PSB_RVDC32(CURABASE);
		dev_priv->saveDSPACURSOR_POS = PSB_RVDC32(CURAPOS);

		/*save palette (gamma) */
		for (i = 0; i < 256; i++)
			dev_priv->save_palette_a[i] = PSB_RVDC32(PALETTE_A + (i<<2));

		/*save performance state*/
		dev_priv->savePERF_MODE = PSB_RVDC32(MRST_PERF_MODE);

		/* LVDS state */
		dev_priv->savePP_CONTROL = PSB_RVDC32(PP_CONTROL);
		dev_priv->savePFIT_PGM_RATIOS = PSB_RVDC32(PFIT_PGM_RATIOS);
		dev_priv->savePFIT_AUTO_RATIOS = PSB_RVDC32(PFIT_AUTO_RATIOS);
		dev_priv->saveBLC_PWM_CTL = PSB_RVDC32(BLC_PWM_CTL);
		dev_priv->saveBLC_PWM_CTL2 = PSB_RVDC32(BLC_PWM_CTL2);
		dev_priv->saveLVDS = PSB_RVDC32(LVDS);
		dev_priv->savePFIT_CONTROL = PSB_RVDC32(PFIT_CONTROL);
		dev_priv->savePP_ON_DELAYS = PSB_RVDC32(LVDSPP_ON);
		dev_priv->savePP_OFF_DELAYS = PSB_RVDC32(LVDSPP_OFF);
		dev_priv->savePP_DIVISOR = PSB_RVDC32(PP_CYCLE);

		/* HW overlay */
		dev_priv->saveOV_OVADD = PSB_RVDC32(OV_OVADD);
		dev_priv->saveOV_OGAMC0 = PSB_RVDC32(OV_OGAMC0);
		dev_priv->saveOV_OGAMC1 = PSB_RVDC32(OV_OGAMC1);
		dev_priv->saveOV_OGAMC2 = PSB_RVDC32(OV_OGAMC2);
		dev_priv->saveOV_OGAMC3 = PSB_RVDC32(OV_OGAMC3);
		dev_priv->saveOV_OGAMC4 = PSB_RVDC32(OV_OGAMC4);
		dev_priv->saveOV_OGAMC5 = PSB_RVDC32(OV_OGAMC5);

		/* MIPI DSI */
                dev_priv->saveMIPI = PSB_RVDC32(MIPI);
                dev_priv->saveDEVICE_READY_REG = PSB_RVDC32(DEVICE_READY_REG);
                dev_priv->saveINTR_EN_REG  = PSB_RVDC32(INTR_EN_REG);
                dev_priv->saveDSI_FUNC_PRG_REG  = PSB_RVDC32(DSI_FUNC_PRG_REG);
                dev_priv->saveHS_TX_TIMEOUT_REG = PSB_RVDC32(HS_TX_TIMEOUT_REG);
                dev_priv->saveLP_RX_TIMEOUT_REG = PSB_RVDC32(LP_RX_TIMEOUT_REG);
                dev_priv->saveTURN_AROUND_TIMEOUT_REG =
                        PSB_RVDC32(TURN_AROUND_TIMEOUT_REG);
                dev_priv->saveDEVICE_RESET_REG = PSB_RVDC32(DEVICE_RESET_REG);
                dev_priv->saveDPI_RESOLUTION_REG =
                        PSB_RVDC32(DPI_RESOLUTION_REG);
                dev_priv->saveHORIZ_SYNC_PAD_COUNT_REG =
                        PSB_RVDC32(HORIZ_SYNC_PAD_COUNT_REG);
                dev_priv->saveHORIZ_BACK_PORCH_COUNT_REG =
                        PSB_RVDC32(HORIZ_BACK_PORCH_COUNT_REG);
                dev_priv->saveHORIZ_FRONT_PORCH_COUNT_REG =
                        PSB_RVDC32(HORIZ_FRONT_PORCH_COUNT_REG);
                dev_priv->saveHORIZ_ACTIVE_AREA_COUNT_REG =
                        PSB_RVDC32(HORIZ_ACTIVE_AREA_COUNT_REG);
                dev_priv->saveVERT_SYNC_PAD_COUNT_REG =
                        PSB_RVDC32(VERT_SYNC_PAD_COUNT_REG);
                dev_priv->saveVERT_BACK_PORCH_COUNT_REG =
                        PSB_RVDC32(VERT_BACK_PORCH_COUNT_REG);
                dev_priv->saveVERT_FRONT_PORCH_COUNT_REG =
                        PSB_RVDC32(VERT_FRONT_PORCH_COUNT_REG);
                dev_priv->saveHIGH_LOW_SWITCH_COUNT_REG =
                        PSB_RVDC32(HIGH_LOW_SWITCH_COUNT_REG);
                dev_priv->saveINIT_COUNT_REG = PSB_RVDC32(INIT_COUNT_REG);
                dev_priv->saveMAX_RET_PAK_REG = PSB_RVDC32(MAX_RET_PAK_REG);
                dev_priv->saveVIDEO_FMT_REG = PSB_RVDC32(VIDEO_FMT_REG);
                dev_priv->saveEOT_DISABLE_REG = PSB_RVDC32(EOT_DISABLE_REG);
                dev_priv->saveLP_BYTECLK_REG = PSB_RVDC32(LP_BYTECLK_REG);
                dev_priv->saveHS_LS_DBI_ENABLE_REG =
                        PSB_RVDC32(HS_LS_DBI_ENABLE_REG);
                dev_priv->saveTXCLKESC_REG = PSB_RVDC32(TXCLKESC_REG);
                dev_priv->saveDPHY_PARAM_REG  = PSB_RVDC32(DPHY_PARAM_REG);
                dev_priv->saveMIPI_CONTROL_REG = PSB_RVDC32(MIPI_CONTROL_REG);

		/* DPST registers */
                dev_priv->saveHISTOGRAM_INT_CONTROL_REG = PSB_RVDC32(HISTOGRAM_INT_CONTROL);
                dev_priv->saveHISTOGRAM_LOGIC_CONTROL_REG = PSB_RVDC32(HISTOGRAM_LOGIC_CONTROL);
		dev_priv->savePWM_CONTROL_LOGIC = PSB_RVDC32(PWM_CONTROL_LOGIC);


	} else { /*PSB*/
		/*save crtc and output state*/
		mutex_lock(&dev->mode_config.mutex);
		list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
			if(drm_helper_crtc_in_use(crtc)) {
				crtc->funcs->save(crtc);
			}
		}

		list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
			connector->funcs->save(connector);
		}
		mutex_unlock(&dev->mode_config.mutex);
	}

	/* Interrupt state */
	/*
	 * Handled in psb_irq.c
	 */

	return 0;
}

/*
 * restore_display_registers
 *
 * Description: We are going to resume so restore display register state.
 */
static int restore_display_registers(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct drm_crtc * crtc;
	struct drm_connector * connector;
	unsigned long i, pp_stat;

	/* Display arbitration + watermarks */
	PSB_WVDC32(dev_priv->saveDSPARB, DSPARB);
	PSB_WVDC32(dev_priv->saveDSPFW1, DSPFW1);
	PSB_WVDC32(dev_priv->saveDSPFW2, DSPFW2);
	PSB_WVDC32(dev_priv->saveDSPFW3, DSPFW3);
	PSB_WVDC32(dev_priv->saveDSPFW4, DSPFW4);
	PSB_WVDC32(dev_priv->saveDSPFW5, DSPFW5);
	PSB_WVDC32(dev_priv->saveDSPFW6, DSPFW6);
	PSB_WVDC32(dev_priv->saveCHICKENBIT, DSPCHICKENBIT);

	/*make sure VGA plane is off. it initializes to on after reset!*/
	PSB_WVDC32(0x80000000, VGACNTRL);

	if (IS_MRST(dev)) {
		/* set the plls */
		PSB_WVDC32(dev_priv->saveFPA0, MRST_FPA0);
		PSB_WVDC32(dev_priv->saveFPA1, MRST_FPA1);
		/* Actually enable it */
		PSB_WVDC32(dev_priv->saveDPLL_A, MRST_DPLL_A);
		DRM_UDELAY(150);

		/* Restore mode */
		PSB_WVDC32(dev_priv->saveHTOTAL_A, HTOTAL_A);
		PSB_WVDC32(dev_priv->saveHBLANK_A, HBLANK_A);
		PSB_WVDC32(dev_priv->saveHSYNC_A, HSYNC_A);
		PSB_WVDC32(dev_priv->saveVTOTAL_A, VTOTAL_A);
		PSB_WVDC32(dev_priv->saveVBLANK_A, VBLANK_A);
		PSB_WVDC32(dev_priv->saveVSYNC_A, VSYNC_A);
		PSB_WVDC32(dev_priv->savePIPEASRC, PIPEASRC);
		PSB_WVDC32(dev_priv->saveBCLRPAT_A, BCLRPAT_A);

		/*restore performance mode*/
		PSB_WVDC32(dev_priv->savePERF_MODE, MRST_PERF_MODE);

		/*enable the pipe*/
		if (dev_priv->iLVDS_enable)
			PSB_WVDC32(dev_priv->savePIPEACONF, PIPEACONF);

		/* Setup MIPI */
                PSB_WVDC32(dev_priv->saveINTR_EN_REG, INTR_EN_REG);
                PSB_WVDC32(dev_priv->saveDSI_FUNC_PRG_REG, DSI_FUNC_PRG_REG);
                PSB_WVDC32(dev_priv->saveHS_TX_TIMEOUT_REG, HS_TX_TIMEOUT_REG);
                PSB_WVDC32(dev_priv->saveLP_RX_TIMEOUT_REG, LP_RX_TIMEOUT_REG);
                PSB_WVDC32(dev_priv->saveTURN_AROUND_TIMEOUT_REG,
			TURN_AROUND_TIMEOUT_REG);
                PSB_WVDC32(dev_priv->saveDEVICE_RESET_REG, DEVICE_RESET_REG);
                PSB_WVDC32(dev_priv->saveDPI_RESOLUTION_REG,
			DPI_RESOLUTION_REG);
                PSB_WVDC32(dev_priv->saveHORIZ_SYNC_PAD_COUNT_REG,
			HORIZ_SYNC_PAD_COUNT_REG);
                PSB_WVDC32(dev_priv->saveHORIZ_BACK_PORCH_COUNT_REG,
			HORIZ_BACK_PORCH_COUNT_REG);
                PSB_WVDC32(dev_priv->saveHORIZ_FRONT_PORCH_COUNT_REG,
			HORIZ_FRONT_PORCH_COUNT_REG);
                PSB_WVDC32(dev_priv->saveHORIZ_ACTIVE_AREA_COUNT_REG,
			HORIZ_ACTIVE_AREA_COUNT_REG);
                PSB_WVDC32(dev_priv->saveVERT_SYNC_PAD_COUNT_REG,
			VERT_SYNC_PAD_COUNT_REG);
                PSB_WVDC32(dev_priv->saveVERT_BACK_PORCH_COUNT_REG,
			VERT_BACK_PORCH_COUNT_REG);
                PSB_WVDC32(dev_priv->saveVERT_FRONT_PORCH_COUNT_REG,
			VERT_FRONT_PORCH_COUNT_REG);
                PSB_WVDC32(dev_priv->saveHIGH_LOW_SWITCH_COUNT_REG,
			HIGH_LOW_SWITCH_COUNT_REG);
                PSB_WVDC32(dev_priv->saveINIT_COUNT_REG, INIT_COUNT_REG);
                PSB_WVDC32(dev_priv->saveMAX_RET_PAK_REG, MAX_RET_PAK_REG);
                PSB_WVDC32(dev_priv->saveVIDEO_FMT_REG, VIDEO_FMT_REG);
                PSB_WVDC32(dev_priv->saveEOT_DISABLE_REG, EOT_DISABLE_REG);
                PSB_WVDC32(dev_priv->saveLP_BYTECLK_REG, LP_BYTECLK_REG);
                PSB_WVDC32(dev_priv->saveHS_LS_DBI_ENABLE_REG,
			HS_LS_DBI_ENABLE_REG);
                PSB_WVDC32(dev_priv->saveTXCLKESC_REG, TXCLKESC_REG);
                PSB_WVDC32(dev_priv->saveDPHY_PARAM_REG, DPHY_PARAM_REG);
                PSB_WVDC32(dev_priv->saveMIPI_CONTROL_REG, MIPI_CONTROL_REG);

		/*set up the plane*/
		PSB_WVDC32(dev_priv->saveDSPALINOFF, DSPALINOFF);
		PSB_WVDC32(dev_priv->saveDSPASTRIDE, DSPASTRIDE);
		PSB_WVDC32(dev_priv->saveDSPATILEOFF, DSPATILEOFF);

		/* Enable the plane */
		PSB_WVDC32(dev_priv->saveDSPACNTR, DSPACNTR);
		PSB_WVDC32(dev_priv->saveDSPASURF, DSPASURF);

		/*Enable Cursor A*/
		PSB_WVDC32(dev_priv->saveDSPACURSOR_CTRL, CURACNTR);
		PSB_WVDC32(dev_priv->saveDSPACURSOR_POS, CURAPOS);
		PSB_WVDC32(dev_priv->saveDSPACURSOR_BASE, CURABASE);

		/* restore palette (gamma) */
		/*DRM_UDELAY(50000); */
		for (i = 0; i < 256; i++)
			PSB_WVDC32(dev_priv->save_palette_a[i], PALETTE_A + (i<<2));

		if (dev_priv->iLVDS_enable) {
			PSB_WVDC32(dev_priv->saveBLC_PWM_CTL2, BLC_PWM_CTL2);
			PSB_WVDC32(dev_priv->saveLVDS, LVDS); /*port 61180h*/
			PSB_WVDC32(dev_priv->savePFIT_CONTROL, PFIT_CONTROL);
			PSB_WVDC32(dev_priv->savePFIT_PGM_RATIOS, PFIT_PGM_RATIOS);
			PSB_WVDC32(dev_priv->savePFIT_AUTO_RATIOS, PFIT_AUTO_RATIOS);
			PSB_WVDC32(dev_priv->saveBLC_PWM_CTL, BLC_PWM_CTL);
			PSB_WVDC32(dev_priv->savePP_ON_DELAYS, LVDSPP_ON);
			PSB_WVDC32(dev_priv->savePP_OFF_DELAYS, LVDSPP_OFF);
			PSB_WVDC32(dev_priv->savePP_DIVISOR, PP_CYCLE);
			PSB_WVDC32(dev_priv->savePP_CONTROL, PP_CONTROL);
		} else {
                        PSB_WVDC32(MIPI_PORT_EN | MIPI_BORDER_EN, MIPI); /*force on port*/
                        PSB_WVDC32(1, DEVICE_READY_REG);/* force on to re-program */

                        if (dev_priv->saveDEVICE_READY_REG) {
                                if ((REG_READ(INTR_STAT_REG) & SPL_PKT_SENT)) {
                                        REG_WRITE(INTR_STAT_REG, SPL_PKT_SENT);
                                }

                                /*send turn on packet*/
                                PSB_WVDC32(DPI_TURN_ON, DPI_CONTROL_REG);

                                /*wait for special packet sent interrupt*/
                                mrst_wait_for_INTR_PKT_SENT(dev);

                                msleep(100);
                        }

                        if(dev_priv->init_drvIC)
                                dev_priv->init_drvIC(dev);
                        PSB_WVDC32(dev_priv->saveMIPI, MIPI); /*port 61190h*/
                        PSB_WVDC32(dev_priv->saveDEVICE_READY_REG, DEVICE_READY_REG);
                        PSB_WVDC32(dev_priv->savePIPEACONF, PIPEACONF);
                        PSB_WVDC32(dev_priv->saveBLC_PWM_CTL2, BLC_PWM_CTL2);
                        PSB_WVDC32(dev_priv->saveBLC_PWM_CTL, BLC_PWM_CTL);
                }


		/*wait for cycle delay*/
		do {
			pp_stat = PSB_RVDC32(PP_STATUS);
		} while (pp_stat & 0x08000000);

		DRM_UDELAY(999);
		/*wait for panel power up*/
		do {
			pp_stat = PSB_RVDC32(PP_STATUS);
		} while (pp_stat & 0x10000000);

		/* restore HW overlay */
		PSB_WVDC32(dev_priv->saveOV_OVADD, OV_OVADD);
		PSB_WVDC32(dev_priv->saveOV_OGAMC0, OV_OGAMC0);
		PSB_WVDC32(dev_priv->saveOV_OGAMC1, OV_OGAMC1);
		PSB_WVDC32(dev_priv->saveOV_OGAMC2, OV_OGAMC2);
		PSB_WVDC32(dev_priv->saveOV_OGAMC3, OV_OGAMC3);
		PSB_WVDC32(dev_priv->saveOV_OGAMC4, OV_OGAMC4);
		PSB_WVDC32(dev_priv->saveOV_OGAMC5, OV_OGAMC5);

		/* DPST registers */
                PSB_WVDC32(dev_priv->saveHISTOGRAM_INT_CONTROL_REG, HISTOGRAM_INT_CONTROL);
                PSB_WVDC32(dev_priv->saveHISTOGRAM_LOGIC_CONTROL_REG, HISTOGRAM_LOGIC_CONTROL);
		PSB_WVDC32(dev_priv->savePWM_CONTROL_LOGIC, PWM_CONTROL_LOGIC);


	} else { /*PSB*/
		mutex_lock(&dev->mode_config.mutex);
		list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
			if(drm_helper_crtc_in_use(crtc))
				crtc->funcs->restore(crtc);
		}

		list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
			connector->funcs->restore(connector);
		}
		mutex_unlock(&dev->mode_config.mutex);
	}


	/*Interrupt state*/
	/*
	 * Handled in psb_irq.c
	 */

	return 0;
}
/*
 * mdfld_save_display_registers
 *
 * Description: We are going to suspend so save current display
 * register state.
 *
 */
static int mdfld_save_display_registers (struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int i;

	/* regester */
	u32 dpll_reg = MRST_DPLL_A;
	u32 fp_reg = MRST_FPA0;
	u32 pipeconf_reg = PIPEACONF;
	u32 htot_reg = HTOTAL_A;
	u32 hblank_reg = HBLANK_A;
	u32 hsync_reg = HSYNC_A;
	u32 vtot_reg = VTOTAL_A;
	u32 vblank_reg = VBLANK_A;
	u32 vsync_reg = VSYNC_A;
	u32 pipesrc_reg = PIPEASRC;
	u32 dspstride_reg = DSPASTRIDE;
	u32 dsplinoff_reg = DSPALINOFF;
	u32 dsptileoff_reg = DSPATILEOFF;
	u32 dspsize_reg = DSPASIZE;
	u32 dsppos_reg = DSPAPOS;
	u32 dspsurf_reg = DSPASURF;
	u32 mipi_reg = MIPI;
	u32 dspcntr_reg = DSPACNTR;
	u32 dspstatus_reg = PIPEASTAT;
	u32 palette_reg = PALETTE_A;
	u32 color_coef_reg = PIPEA_COLOR_COEF0;

	/* pointer to values */
	u32 *dpll_val = &dev_priv->saveDPLL_A;
	u32 *fp_val = &dev_priv->saveFPA0;
	u32 *pipeconf_val = &dev_priv->savePIPEACONF;
	u32 *htot_val = &dev_priv->saveHTOTAL_A;
	u32 *hblank_val = &dev_priv->saveHBLANK_A;
	u32 *hsync_val = &dev_priv->saveHSYNC_A;
	u32 *vtot_val = &dev_priv->saveVTOTAL_A;
	u32 *vblank_val = &dev_priv->saveVBLANK_A;
	u32 *vsync_val = &dev_priv->saveVSYNC_A;
	u32 *pipesrc_val = &dev_priv->savePIPEASRC;
	u32 *dspstride_val = &dev_priv->saveDSPASTRIDE;
	u32 *dsplinoff_val = &dev_priv->saveDSPALINOFF;
	u32 *dsptileoff_val = &dev_priv->saveDSPATILEOFF;
	u32 *dspsize_val = &dev_priv->saveDSPASIZE;
	u32 *dsppos_val = &dev_priv->saveDSPAPOS;
	u32 *dspsurf_val = &dev_priv->saveDSPASURF;
	u32 *mipi_val = &dev_priv->saveMIPI;
	u32 *dspcntr_val = &dev_priv->saveDSPACNTR;
	u32 *dspstatus_val = &dev_priv->saveDSPASTATUS;
	u32 *palette_val = dev_priv->save_palette_a;
	u32 *color_coef = dev_priv->save_color_coef_a;
	PSB_DEBUG_ENTRY("\n");

	/**
	 * For MIPI panels, all plane/pipe/port/DSI controller values
	 * were already saved in dsi_hw_context, no need to save/restore
	 * for these registers.
	 * NOTE: only support TMD panel now, add support for other MIPI
	 * panels later
	 */
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
	if (pipe != 1 && ((get_panel_type(dev, pipe) == TMD_VID) ||
		(get_panel_type(dev, pipe) == TMD_6X10_VID) ||
		(get_panel_type(dev, pipe) == H8C7_VID) ||
		(get_panel_type(dev, pipe) == GI_SONY_VID) ||
		/* SC1 setting */
		(get_panel_type(dev, pipe) == AUO_SC1_VID)))
		return 0;
#endif
#endif

	switch (pipe) {
	case 0:
		break;
	case 1:
		/* regester */
		dpll_reg = MDFLD_DPLL_B;
		fp_reg = MDFLD_DPLL_DIV0;
		pipeconf_reg = PIPEBCONF;
		htot_reg = HTOTAL_B;
		hblank_reg = HBLANK_B;
		hsync_reg = HSYNC_B;
		vtot_reg = VTOTAL_B;
		vblank_reg = VBLANK_B;
		vsync_reg = VSYNC_B;
		pipesrc_reg = PIPEBSRC;
		dspstride_reg = DSPBSTRIDE;
		dsplinoff_reg = DSPBLINOFF;
		dsptileoff_reg = DSPBTILEOFF;
		dspsize_reg = DSPBSIZE;
		dsppos_reg = DSPBPOS;
		dspsurf_reg = DSPBSURF;
		dspcntr_reg = DSPBCNTR;
		dspstatus_reg = PIPEBSTAT;
		palette_reg = PALETTE_B;
		color_coef_reg = PIPEB_COLOR_COEF0;

		/* values */
		dpll_val = &dev_priv->saveDPLL_B;
		fp_val = &dev_priv->saveFPB0;
		pipeconf_val = &dev_priv->savePIPEBCONF;
		htot_val = &dev_priv->saveHTOTAL_B;
		hblank_val = &dev_priv->saveHBLANK_B;
		hsync_val = &dev_priv->saveHSYNC_B;
		vtot_val = &dev_priv->saveVTOTAL_B;
		vblank_val = &dev_priv->saveVBLANK_B;
		vsync_val = &dev_priv->saveVSYNC_B;
		pipesrc_val = &dev_priv->savePIPEBSRC;
		dspstride_val = &dev_priv->saveDSPBSTRIDE;
		dsplinoff_val = &dev_priv->saveDSPBLINOFF;
		dsptileoff_val = &dev_priv->saveDSPBTILEOFF;
		dspsize_val = &dev_priv->saveDSPBSIZE;
		dsppos_val = &dev_priv->saveDSPBPOS;
		dspsurf_val = &dev_priv->saveDSPBSURF;
		dspcntr_val = &dev_priv->saveDSPBCNTR;
		dspstatus_val = &dev_priv->saveDSPBSTATUS;
		palette_val = dev_priv->save_palette_b;
		color_coef = dev_priv->save_color_coef_b;
		break;
	case 2:
		/* regester */
		pipeconf_reg = PIPECCONF;
		htot_reg = HTOTAL_C;
		hblank_reg = HBLANK_C;
		hsync_reg = HSYNC_C;
		vtot_reg = VTOTAL_C;
		vblank_reg = VBLANK_C;
		vsync_reg = VSYNC_C;
		pipesrc_reg = PIPECSRC;
		dspstride_reg = DSPCSTRIDE;
		dsplinoff_reg = DSPCLINOFF;
		dsptileoff_reg = DSPCTILEOFF;
		dspsize_reg = DSPCSIZE;
		dsppos_reg = DSPCPOS;
		dspsurf_reg = DSPCSURF;
		mipi_reg = MIPI_C;
		dspcntr_reg = DSPCCNTR;
		dspstatus_reg = PIPECSTAT;
		palette_reg = PALETTE_C;
		color_coef_reg = PIPEC_COLOR_COEF0;

		/* pointer to values */
		pipeconf_val = &dev_priv->savePIPECCONF;
		htot_val = &dev_priv->saveHTOTAL_C;
		hblank_val = &dev_priv->saveHBLANK_C;
		hsync_val = &dev_priv->saveHSYNC_C;
		vtot_val = &dev_priv->saveVTOTAL_C;
		vblank_val = &dev_priv->saveVBLANK_C;
		vsync_val = &dev_priv->saveVSYNC_C;
		pipesrc_val = &dev_priv->savePIPECSRC;
		dspstride_val = &dev_priv->saveDSPCSTRIDE;
		dsplinoff_val = &dev_priv->saveDSPCLINOFF;
		dsptileoff_val = &dev_priv->saveDSPCTILEOFF;
		dspsize_val = &dev_priv->saveDSPCSIZE;
		dsppos_val = &dev_priv->saveDSPCPOS;
		dspsurf_val = &dev_priv->saveDSPCSURF;
		mipi_val = &dev_priv->saveMIPI_C;
		dspcntr_val = &dev_priv->saveDSPCCNTR;
		dspstatus_val = &dev_priv->saveDSPCSTATUS;
		palette_val = dev_priv->save_palette_c;
		color_coef = dev_priv->save_color_coef_c;
		break;
	default:
		DRM_ERROR("%s, invalid pipe number.\n", __func__);
       		return -EINVAL;
	}

	/* Pipe & plane A info */
	*dpll_val = PSB_RVDC32(dpll_reg);
	*fp_val = PSB_RVDC32(fp_reg);
	*pipeconf_val = PSB_RVDC32(pipeconf_reg);
	*htot_val = PSB_RVDC32(htot_reg);
	*hblank_val = PSB_RVDC32(hblank_reg);
	*hsync_val = PSB_RVDC32(hsync_reg);
	*vtot_val = PSB_RVDC32(vtot_reg);
	*vblank_val = PSB_RVDC32(vblank_reg);
	*vsync_val = PSB_RVDC32(vsync_reg);
	*pipesrc_val = PSB_RVDC32(pipesrc_reg);
	*dspstride_val = PSB_RVDC32(dspstride_reg);
	*dsplinoff_val = PSB_RVDC32(dsplinoff_reg);
	*dsptileoff_val = PSB_RVDC32(dsptileoff_reg);
	*dspsize_val = PSB_RVDC32(dspsize_reg);
	*dsppos_val = PSB_RVDC32(dsppos_reg);
	*dspsurf_val = PSB_RVDC32(dspsurf_reg);
	*dspcntr_val = PSB_RVDC32(dspcntr_reg);
	*dspstatus_val = PSB_RVDC32(dspstatus_reg);

	/*save palette (gamma) */
	for (i = 0; i < 256; i++)
		palette_val[i] = PSB_RVDC32(palette_reg + (i<<2));

	/*save color_coef (chrome) */
	for (i = 0; i < 6; i++) {
		color_coef[i] = PSB_RVDC32(color_coef_reg + (i<<2));
		printk("0x%x ", color_coef[i]);
	}


	if (pipe == 1) {
		dev_priv->savePFIT_CONTROL = PSB_RVDC32(PFIT_CONTROL);
		dev_priv->savePFIT_PGM_RATIOS = PSB_RVDC32(PFIT_PGM_RATIOS);

		dev_priv->saveHDMIPHYMISCCTL = PSB_RVDC32(HDMIPHYMISCCTL);
		dev_priv->saveHDMIB_CONTROL = PSB_RVDC32(HDMIB_CONTROL);
		return 0;
	}

	*mipi_val = PSB_RVDC32(mipi_reg);
	return 0;
}
/*
 * mdfld_save_cursor_overlay_registers
 *
 * Description: We are going to suspend so save current cursor and overlay display
 * register state.
 */
static int mdfld_save_cursor_overlay_registers(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	/*save cursor regs*/
	dev_priv->saveDSPACURSOR_CTRL = PSB_RVDC32(CURACNTR);
	dev_priv->saveDSPACURSOR_BASE = PSB_RVDC32(CURABASE);
	dev_priv->saveDSPACURSOR_POS = PSB_RVDC32(CURAPOS);

	dev_priv->saveDSPBCURSOR_CTRL = PSB_RVDC32(CURBCNTR);
	dev_priv->saveDSPBCURSOR_BASE = PSB_RVDC32(CURBBASE);
	dev_priv->saveDSPBCURSOR_POS = PSB_RVDC32(CURBPOS);

	dev_priv->saveDSPCCURSOR_CTRL = PSB_RVDC32(CURCCNTR);
	dev_priv->saveDSPCCURSOR_BASE = PSB_RVDC32(CURCBASE);
	dev_priv->saveDSPCCURSOR_POS = PSB_RVDC32(CURCPOS);

	/* HW overlay */
	dev_priv->saveOV_OVADD = PSB_RVDC32(OV_OVADD);
	dev_priv->saveOV_OGAMC0 = PSB_RVDC32(OV_OGAMC0);
	dev_priv->saveOV_OGAMC1 = PSB_RVDC32(OV_OGAMC1);
	dev_priv->saveOV_OGAMC2 = PSB_RVDC32(OV_OGAMC2);
	dev_priv->saveOV_OGAMC3 = PSB_RVDC32(OV_OGAMC3);
	dev_priv->saveOV_OGAMC4 = PSB_RVDC32(OV_OGAMC4);
	dev_priv->saveOV_OGAMC5 = PSB_RVDC32(OV_OGAMC5);

	dev_priv->saveOV_OVADD_C = PSB_RVDC32(OV_OVADD + OV_C_OFFSET);
	dev_priv->saveOV_OGAMC0_C = PSB_RVDC32(OV_OGAMC0 + OV_C_OFFSET);
	dev_priv->saveOV_OGAMC1_C = PSB_RVDC32(OV_OGAMC1 + OV_C_OFFSET);
	dev_priv->saveOV_OGAMC2_C = PSB_RVDC32(OV_OGAMC2 + OV_C_OFFSET);
	dev_priv->saveOV_OGAMC3_C = PSB_RVDC32(OV_OGAMC3 + OV_C_OFFSET);
	dev_priv->saveOV_OGAMC4_C = PSB_RVDC32(OV_OGAMC4 + OV_C_OFFSET);
	dev_priv->saveOV_OGAMC5_C = PSB_RVDC32(OV_OGAMC5 + OV_C_OFFSET);

	return 0;
}
/*
 * mdfld_restore_display_registers
 *
 * Description: We are going to resume so restore display register state.
 *
 */
static int mdfld_restore_display_registers(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_config * dsi_config = NULL;
	u32 i = 0;
	u32 dpll = 0;
	u32 timeout = 0;
	u32 reg_offset = 0;

	/* regester */
	u32 dpll_reg = MRST_DPLL_A;
	u32 fp_reg = MRST_FPA0;
	u32 pipeconf_reg = PIPEACONF;
	u32 htot_reg = HTOTAL_A;
	u32 hblank_reg = HBLANK_A;
	u32 hsync_reg = HSYNC_A;
	u32 vtot_reg = VTOTAL_A;
	u32 vblank_reg = VBLANK_A;
	u32 vsync_reg = VSYNC_A;
	u32 pipesrc_reg = PIPEASRC;
	u32 dspstride_reg = DSPASTRIDE;
	u32 dsplinoff_reg = DSPALINOFF;
	u32 dsptileoff_reg = DSPATILEOFF;
	u32 dspsize_reg = DSPASIZE;
	u32 dsppos_reg = DSPAPOS;
	u32 dspsurf_reg = DSPASURF;
	u32 dspstatus_reg = PIPEASTAT;
	u32 mipi_reg = MIPI;
	u32 dspcntr_reg = DSPACNTR;
	u32 palette_reg = PALETTE_A;
	u32 color_coef_reg = PIPEA_COLOR_COEF0;

	/* values */
	u32 dpll_val = dev_priv->saveDPLL_A & ~DPLL_VCO_ENABLE;
	u32 fp_val = dev_priv->saveFPA0;
	u32 pipeconf_val = dev_priv->savePIPEACONF;
	u32 htot_val = dev_priv->saveHTOTAL_A;
	u32 hblank_val = dev_priv->saveHBLANK_A;
	u32 hsync_val = dev_priv->saveHSYNC_A;
	u32 vtot_val = dev_priv->saveVTOTAL_A;
	u32 vblank_val = dev_priv->saveVBLANK_A;
	u32 vsync_val = dev_priv->saveVSYNC_A;
	u32 pipesrc_val = dev_priv->savePIPEASRC;
	u32 dspstride_val = dev_priv->saveDSPASTRIDE;
	u32 dsplinoff_val = dev_priv->saveDSPALINOFF;
	u32 dsptileoff_val = dev_priv->saveDSPATILEOFF;
	u32 dspsize_val = dev_priv->saveDSPASIZE;
	u32 dsppos_val = dev_priv->saveDSPAPOS;
	u32 dspsurf_val = dev_priv->saveDSPASURF;
	u32 dspstatus_val = dev_priv->saveDSPASTATUS;
	u32 mipi_val = dev_priv->saveMIPI;
	u32 dspcntr_val = dev_priv->saveDSPACNTR;
	u32 *palette_val = dev_priv->save_palette_a;
	u32 *color_coef = dev_priv->save_color_coef_a;
	PSB_DEBUG_ENTRY("\n");

	/**
	 * For MIPI panels, all plane/pipe/port/DSI controller values
	 * were already saved in dsi_hw_context, no need to save/restore
	 * for these registers.
	 * NOTE: only support TMD panel now, add support for other MIPI
	 * panels later
	 */
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
	if (pipe != 1 && ((get_panel_type(dev, pipe) == TMD_VID) ||
		(get_panel_type(dev, pipe) == TMD_6X10_VID) ||
		(get_panel_type(dev, pipe) == H8C7_VID) ||
		(get_panel_type(dev, pipe) == GI_SONY_VID) ||
		/* SC1 setting */
		(get_panel_type(dev, pipe) == AUO_SC1_VID)))
		return 0;
#endif
#endif

	switch (pipe) {
	case 0:
		dsi_config = dev_priv->dsi_configs[0];
		break;
	case 1:
		/* regester */
		dpll_reg = MDFLD_DPLL_B;
		fp_reg = MDFLD_DPLL_DIV0;
		pipeconf_reg = PIPEBCONF;
		htot_reg = HTOTAL_B;
		hblank_reg = HBLANK_B;
		hsync_reg = HSYNC_B;
		vtot_reg = VTOTAL_B;
		vblank_reg = VBLANK_B;
		vsync_reg = VSYNC_B;
		pipesrc_reg = PIPEBSRC;
		dspstride_reg = DSPBSTRIDE;
		dsplinoff_reg = DSPBLINOFF;
		dsptileoff_reg = DSPBTILEOFF;
		dspsize_reg = DSPBSIZE;
		dsppos_reg = DSPBPOS;
		dspsurf_reg = DSPBSURF;
		dspcntr_reg = DSPBCNTR;
		palette_reg = PALETTE_B;
		dspstatus_reg = PIPEBSTAT;
		color_coef_reg = PIPEB_COLOR_COEF0;

		/* values */
		dpll_val = dev_priv->saveDPLL_B & ~DPLL_VCO_ENABLE;
		fp_val = dev_priv->saveFPB0;
		pipeconf_val = dev_priv->savePIPEBCONF;
		htot_val = dev_priv->saveHTOTAL_B;
		hblank_val = dev_priv->saveHBLANK_B;
		hsync_val = dev_priv->saveHSYNC_B;
		vtot_val = dev_priv->saveVTOTAL_B;
		vblank_val = dev_priv->saveVBLANK_B;
		vsync_val = dev_priv->saveVSYNC_B;
		pipesrc_val = dev_priv->savePIPEBSRC;
		dspstride_val = dev_priv->saveDSPBSTRIDE;
		dsplinoff_val = dev_priv->saveDSPBLINOFF;
		dsptileoff_val = dev_priv->saveDSPBTILEOFF;
		dspsize_val = dev_priv->saveDSPBSIZE;
		dsppos_val = dev_priv->saveDSPBPOS;
		dspsurf_val = dev_priv->saveDSPBSURF;
		dspcntr_val = dev_priv->saveDSPBCNTR & ~DISPLAY_PLANE_ENABLE;
		dspstatus_val = dev_priv->saveDSPBSTATUS;
		palette_val = dev_priv->save_palette_b;
		color_coef = dev_priv->save_color_coef_b;
		break;
	case 2:
		reg_offset = MIPIC_REG_OFFSET;

		/* regester */
		pipeconf_reg = PIPECCONF;
		htot_reg = HTOTAL_C;
		hblank_reg = HBLANK_C;
		hsync_reg = HSYNC_C;
		vtot_reg = VTOTAL_C;
		vblank_reg = VBLANK_C;
		vsync_reg = VSYNC_C;
		pipesrc_reg = PIPECSRC;
		dspstride_reg = DSPCSTRIDE;
		dsplinoff_reg = DSPCLINOFF;
		dsptileoff_reg = DSPCTILEOFF;
		dspsize_reg = DSPCSIZE;
		dsppos_reg = DSPCPOS;
		dspsurf_reg = DSPCSURF;
		mipi_reg = MIPI_C;
		dspcntr_reg = DSPCCNTR;
		palette_reg = PALETTE_C;
		dspstatus_reg = PIPECSTAT;
		color_coef_reg = PIPEC_COLOR_COEF0;

		/* values */
		pipeconf_val = dev_priv->savePIPECCONF;
		htot_val = dev_priv->saveHTOTAL_C;
		hblank_val = dev_priv->saveHBLANK_C;
		hsync_val = dev_priv->saveHSYNC_C;
		vtot_val = dev_priv->saveVTOTAL_C;
		vblank_val = dev_priv->saveVBLANK_C;
		vsync_val = dev_priv->saveVSYNC_C;
		pipesrc_val = dev_priv->savePIPECSRC;
		dspstride_val = dev_priv->saveDSPCSTRIDE;
		dsplinoff_val = dev_priv->saveDSPCLINOFF;
		dsptileoff_val = dev_priv->saveDSPCTILEOFF;
		dspsize_val = dev_priv->saveDSPCSIZE;
		dsppos_val = dev_priv->saveDSPCPOS;
		dspsurf_val = dev_priv->saveDSPCSURF;
		dspstatus_val = dev_priv->saveDSPCSTATUS;
		mipi_val = dev_priv->saveMIPI_C;
		dspcntr_val = dev_priv->saveDSPCCNTR;
		palette_val = dev_priv->save_palette_c;
		color_coef = dev_priv->save_color_coef_c;

		dsi_config = dev_priv->dsi_configs[1];
		break;
	default:
		DRM_ERROR("%s, invalid pipe number.\n", __func__);
		return -EINVAL;
	}

	/*make sure VGA plane is off. it initializes to on after reset!*/
	PSB_WVDC32(0x80000000, VGACNTRL);

	dpll = PSB_RVDC32(dpll_reg);

	if (!(dpll & DPLL_VCO_ENABLE)) {
		/**
		 * When ungating power of DPLL, needs to wait 0.5us
		 * before enable the VCO
		 */
		if (dpll & MDFLD_PWR_GATE_EN) {
			dpll &= ~MDFLD_PWR_GATE_EN;
			PSB_WVDC32(dpll, dpll_reg);
			/* FIXME_MDFLD PO - change 500 to 1 after PO */
			udelay(500);
		}

		PSB_WVDC32(fp_val, fp_reg);
		PSB_WVDC32(dpll_val, dpll_reg);
		/* FIXME_MDFLD PO - change 500 to 1 after PO */
		udelay(500);

		dpll_val |= DPLL_VCO_ENABLE;
		PSB_WVDC32(dpll_val, dpll_reg);
		PSB_RVDC32(dpll_reg);

		/* wait for DSI PLL to lock */
		/**
		 * FIXME: HDMI PLL cannot be locked on pipe 1,
		 * replace pipe == 0 with pipe != 2 when HDMI
		 * restore is ready
		 */
		while ((pipe == 0) &&
			(timeout < 20000) &&
			!(PSB_RVDC32(pipeconf_reg) & PIPECONF_DSIPLL_LOCK)) {
			udelay(150);
			timeout++;
		}

		if (timeout == 20000) {
			DRM_ERROR("%s, can't lock DSIPLL.\n", __func__);
			return -EINVAL;
		}
	}

	/* Restore mode */
	PSB_WVDC32(htot_val, htot_reg);
	PSB_WVDC32(hblank_val, hblank_reg);
	PSB_WVDC32(hsync_val, hsync_reg);
	PSB_WVDC32(vtot_val, vtot_reg);
	PSB_WVDC32(vblank_val, vblank_reg);
	PSB_WVDC32(vsync_val, vsync_reg);
	PSB_WVDC32(pipesrc_val, pipesrc_reg);
	PSB_WVDC32(dspstatus_val,dspstatus_reg);

	/*set up the plane*/
	PSB_WVDC32(dspstride_val, dspstride_reg);
	PSB_WVDC32(dsplinoff_val, dsplinoff_reg);
	PSB_WVDC32(dsptileoff_val, dsptileoff_reg);
	PSB_WVDC32(dspsize_val, dspsize_reg);
	PSB_WVDC32(dsppos_val, dsppos_reg);
	PSB_WVDC32(dspsurf_val, dspsurf_reg);

	if (pipe == 1) {
		PSB_WVDC32(dev_priv->savePFIT_CONTROL, PFIT_CONTROL);
		PSB_WVDC32(dev_priv->savePFIT_PGM_RATIOS, PFIT_PGM_RATIOS);

		PSB_WVDC32(dev_priv->saveHDMIPHYMISCCTL, HDMIPHYMISCCTL);
		PSB_WVDC32(dev_priv->saveHDMIB_CONTROL, HDMIB_CONTROL);

	} else {
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
		/*set up pipe related registers*/
		PSB_WVDC32(mipi_val, mipi_reg);

		if(in_atomic() || in_interrupt())
			mdelay(20);
		else
			msleep(20);

#endif
		/*TODO: remove MIPI restore code later*/
		/*dsi_config->dvr_ic_inited = 0;*/
		/*mdfld_dsi_tmd_drv_ic_init(dsi_config, pipe);*/
	}

	/*save color_coef (chrome) */
	for (i = 0; i < 6; i++) {
		PSB_WVDC32(color_coef[i], color_coef_reg + (i<<2));
		printk("0x%x ", color_coef[i]);
	}

	/* restore palette (gamma) */
	/*DRM_UDELAY(50000); */
	for (i = 0; i < 256; i++)
		PSB_WVDC32(palette_val[i], palette_reg + (i<<2));

	/* disable gamma if needed */
	if (pipe == 0 && drm_psb_enable_gamma == 0)
		 dspcntr_val &= ~(PIPEACONF_GAMMA);

	/* disable csc if needed */
	if (pipe == 0 && drm_psb_enable_color_conversion == 0)
		 pipeconf_val &= ~(PIPEACONF_COLOR_MATRIX_ENABLE);

	/*enable the plane*/
	PSB_WVDC32(dspcntr_val, dspcntr_reg);

    if(in_atomic() || in_interrupt())
        mdelay(20);
    else
	msleep(20);

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
	{
		u32 temp = 0;
		u32 device_ready_reg = 0;

		device_ready_reg = DEVICE_READY_REG + reg_offset;
		/* LP Hold Release */
		temp = REG_READ(mipi_reg);
		temp |= LP_OUTPUT_HOLD_RELEASE;
		REG_WRITE(mipi_reg, temp);
		mdelay(1);


		/* Set DSI host to exit from Utra Low Power State */
		temp = REG_READ(device_ready_reg);
		temp &= ~ULPS_MASK;
		temp |= 0x3;
		temp |= EXIT_ULPS_DEV_READY;
		REG_WRITE(device_ready_reg, temp);
		mdelay(1);

		temp = REG_READ(device_ready_reg);
		temp &= ~ULPS_MASK;
		temp |= EXITING_ULPS;
		REG_WRITE(device_ready_reg, temp);
		mdelay(1);
	}
#endif

	/*enable the pipe*/
	PSB_WVDC32(pipeconf_val, pipeconf_reg);

	if (pipe == 1)
		return 0;

	if ((get_panel_type(dev, pipe) == GI_SONY_CMD)||(get_panel_type(dev, pipe) == H8C7_CMD))
		psb_enable_vblank(dev, pipe);
	else if (IS_MDFLD(dev) && (dev_priv->platform_rev_id != MDFLD_PNW_A0) &&
			!is_panel_vid_or_cmd(dev))
		mdfld_enable_te(dev, pipe);

	return 0;
}

/*
 * mdfld_restore_cursor_overlay_registers
 *
 * Description: We are going to resume so restore cursor and overlay register state.
 */
static int mdfld_restore_cursor_overlay_registers(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	/*Enable Cursor A*/
	PSB_WVDC32(dev_priv->saveDSPACURSOR_CTRL, CURACNTR);
	PSB_WVDC32(dev_priv->saveDSPACURSOR_POS, CURAPOS);
	PSB_WVDC32(dev_priv->saveDSPACURSOR_BASE, CURABASE);

	PSB_WVDC32(dev_priv->saveDSPBCURSOR_CTRL, CURBCNTR);
	PSB_WVDC32(dev_priv->saveDSPBCURSOR_POS, CURBPOS);
	PSB_WVDC32(dev_priv->saveDSPBCURSOR_BASE, CURBBASE);

	PSB_WVDC32(dev_priv->saveDSPCCURSOR_CTRL, CURCCNTR);
	PSB_WVDC32(dev_priv->saveDSPCCURSOR_POS, CURCPOS);
	PSB_WVDC32(dev_priv->saveDSPCCURSOR_BASE, CURCBASE);

	/* restore HW overlay */
	PSB_WVDC32(dev_priv->saveOV_OVADD, OV_OVADD);
	PSB_WVDC32(dev_priv->saveOV_OGAMC0, OV_OGAMC0);
	PSB_WVDC32(dev_priv->saveOV_OGAMC1, OV_OGAMC1);
	PSB_WVDC32(dev_priv->saveOV_OGAMC2, OV_OGAMC2);
	PSB_WVDC32(dev_priv->saveOV_OGAMC3, OV_OGAMC3);
	PSB_WVDC32(dev_priv->saveOV_OGAMC4, OV_OGAMC4);
	PSB_WVDC32(dev_priv->saveOV_OGAMC5, OV_OGAMC5);

	PSB_WVDC32(dev_priv->saveOV_OVADD_C, OV_OVADD + OV_C_OFFSET);
	PSB_WVDC32(dev_priv->saveOV_OGAMC0_C, OV_OGAMC0 + OV_C_OFFSET);
	PSB_WVDC32(dev_priv->saveOV_OGAMC1_C, OV_OGAMC1 + OV_C_OFFSET);
	PSB_WVDC32(dev_priv->saveOV_OGAMC2_C, OV_OGAMC2 + OV_C_OFFSET);
	PSB_WVDC32(dev_priv->saveOV_OGAMC3_C, OV_OGAMC3 + OV_C_OFFSET);
	PSB_WVDC32(dev_priv->saveOV_OGAMC4_C, OV_OGAMC4 + OV_C_OFFSET);
	PSB_WVDC32(dev_priv->saveOV_OGAMC5_C, OV_OGAMC5 + OV_C_OFFSET);

	return 0;
}

/*
 *  mdfld_save_display
 *
 * Description: Save display status before DPMS OFF for RuntimePM
 */
void mdfld_save_display(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "ospm_save_display\n");
#endif
	mdfld_save_cursor_overlay_registers(dev);

	if (dev_priv->panel_desc & DISPLAY_A) mdfld_save_display_registers(dev, 0);
	if (dev_priv->panel_desc & DISPLAY_C) mdfld_save_display_registers(dev, 2);  //h8c7_cmd
}

/*
 * powermgmt_suspend_display
 *
 * Description: Suspend the display hardware saving state and disabling
 * as necessary.
 */
void ospm_suspend_display(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int pp_stat, ret=0;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif

	if (!ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s - IGNORING!!!!\n", __func__);
#endif
		return;
	}

	if (IS_MDFLD(dev)) {
                mdfld_save_cursor_overlay_registers(dev);

		if (dev_priv->panel_desc & DISPLAY_A)
			mdfld_save_display_registers(dev, 0);
		if (dev_priv->panel_desc & DISPLAY_B)
			mdfld_save_display_registers(dev, 1);
		if (dev_priv->panel_desc & DISPLAY_C)
			mdfld_save_display_registers(dev, 2);

		if (dev_priv->panel_desc & DISPLAY_A)
			mdfld_disable_crtc(dev, 0);
		if (dev_priv->panel_desc & DISPLAY_B)
			mdfld_disable_crtc(dev, 1);
		if (dev_priv->panel_desc & DISPLAY_C)
			mdfld_disable_crtc(dev, 2);

		/*save performance state*/
		dev_priv->savePERF_MODE = PSB_RVDC32(MRST_PERF_MODE);
		dev_priv->saveVED_CG_DIS = PSB_RVDC32(PSB_MSVDX_CLOCKGATING);
#ifdef CONFIG_MDFD_GL3
		dev_priv->saveGL3_CTL = PSB_RVDC32(MDFLD_GL3_CONTROL);
		dev_priv->saveGL3_USE_WRT_INVAL = PSB_RVDC32(MDFLD_GL3_USE_WRT_INVAL);
#endif

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
		{
			u32 temp = 0;
			u32 device_ready_reg = 0;
			u32 mipi_reg = 0;
			u32 reg_offset = 0;

			device_ready_reg = DEVICE_READY_REG + reg_offset;
			mipi_reg = MIPI;
			/* Put the panel in ULPS mode for S0ix. */
			temp = REG_READ(device_ready_reg);
			temp &= ~ULPS_MASK;
			temp |= ENTERING_ULPS;
			REG_WRITE(device_ready_reg, temp);

			/* LP Hold */
			temp = REG_READ(mipi_reg);
			temp &= ~LP_OUTPUT_HOLD;
			REG_WRITE(mipi_reg, temp);
			mdelay(1);
		}
#endif

	} else {
		save_display_registers(dev);

		if (dev_priv->iLVDS_enable) {
			/*shutdown the panel*/
			PSB_WVDC32(0, PP_CONTROL);

			do {
				pp_stat = PSB_RVDC32(PP_STATUS);
			} while (pp_stat & 0x80000000);

			/*turn off the plane*/
			PSB_WVDC32(0x58000000, DSPACNTR);
			PSB_WVDC32(0, DSPASURF);/*trigger the plane disable*/
			/*wait ~4 ticks*/
			msleep(4);

			/*turn off pipe*/
			PSB_WVDC32(0x0, PIPEACONF);
			/*wait ~8 ticks*/
			msleep(8);

			/*turn off PLLs*/
			PSB_WVDC32(0, MRST_DPLL_A);
		} else {
			PSB_WVDC32(DPI_SHUT_DOWN, DPI_CONTROL_REG);
			PSB_WVDC32(0x0, PIPEACONF);
			PSB_WVDC32(0x2faf0000, BLC_PWM_CTL);
			while (REG_READ(0x70008) & 0x40000000);
			while ((PSB_RVDC32(GEN_FIFO_STAT_REG) & DPI_FIFO_EMPTY)
				!= DPI_FIFO_EMPTY);
			PSB_WVDC32(0, DEVICE_READY_REG);

			/* turn off panel power */
			ret = 0;
#ifdef CONFIG_X86_MDFLD
			ret = intel_scu_ipc_simple_command(IPC_MSG_PANEL_ON_OFF, IPC_CMD_PANEL_OFF);
			if (ret)
				printk(KERN_WARNING "IPC 0xE9 failed to turn off pnl pwr. Error is: %x\n", ret);
#endif
		}
	}

	ospm_power_island_down(OSPM_DISPLAY_ISLAND);
}

#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
/*
 * is_hdmi_plugged_out
 *
 * Description: to check whether hdmi is plugged out in S3 suspend
 *
 */
static bool is_hdmi_plugged_out(struct drm_device *dev)
{
	u8 data = 0;
	bool hdmi_plugged_out = true;

	if (IS_MDFLD_OLD(dev)) {
		intel_scu_ipc_ioread8(MSIC_HDMI_STATUS, &data);

		if (data & HPD_SIGNAL_STATUS)
			hdmi_plugged_out = false;
		else
			hdmi_plugged_out = true;
	} else if (IS_CTP(dev)) {
		if (gpio_get_value(CLV_TI_HPD_GPIO_PIN) == 0)
			hdmi_plugged_out = true;
		else
			hdmi_plugged_out = false;
	}

	return hdmi_plugged_out;
}
#endif

/*
 * ospm_resume_display
 *
 * Description: Resume the display hardware restoring state and enabling
 * as necessary.
 */
void ospm_resume_display(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
	if (ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s - IGNORING!!!!\n", __func__);
#endif
		printk(KERN_ALERT "[DISPLAY] Exit %s because hw on\n",
				__func__);
		return;
	}

	if (IS_MDFLD(dev)) {
		/*restore performance mode*/
		PSB_WVDC32(dev_priv->savePERF_MODE, MRST_PERF_MODE);
		PSB_WVDC32(dev_priv->saveVED_CG_DIS, PSB_MSVDX_CLOCKGATING);
#ifdef CONFIG_MDFD_GL3
		PSB_WVDC32(dev_priv->saveGL3_CTL, MDFLD_GL3_CONTROL);
		PSB_WVDC32(dev_priv->saveGL3_USE_WRT_INVAL, MDFLD_GL3_USE_WRT_INVAL);
#endif
    }

	/* turn on the display power island */
	ospm_power_island_up(OSPM_DISPLAY_ISLAND);

	PSB_WVDC32(pg->pge_ctl | _PSB_PGETBL_ENABLED, PSB_PGETBL_CTL);
	pci_write_config_word(pdev, PSB_GMCH_CTRL,
			pg->gmch_ctrl | _PSB_GMCH_ENABLED);

	/* Don't reinitialize the GTT as it is unnecessary.  The gtt is
	 * stored in memory so it will automatically be restored.  All
	 * we need to do is restore the PGETBL_CTL which we already do
	 * above.
	 */
	/*psb_gtt_init(dev_priv->pg, 1);*/

	if (IS_MDFLD(dev)) {
		if (dev_priv->panel_desc & DISPLAY_C)
			mdfld_restore_display_registers(dev, 2);
		if (dev_priv->panel_desc & DISPLAY_A)
			mdfld_restore_display_registers(dev, 0);

		/*
		 * Don't restore Display B registers during resuming, if HDMI
		 * isn't turned on before suspending.
		 */
		if (dev_priv->panel_desc & DISPLAY_B) {
			mdfld_restore_display_registers(dev, 1);
			/*devices connect status will be changed
			 when system suspend,re-detect once here*/
#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
			if (!is_hdmi_plugged_out(dev)) {
				PSB_DEBUG_ENTRY("resume hdmi_state %d", hdmi_state);
				if (dev_priv->had_pvt_data && hdmi_state)
					dev_priv->had_interface->
						resume(dev_priv->had_pvt_data);
			}
#endif
		}
		mdfld_restore_cursor_overlay_registers(dev);

	}else{
		if (!dev_priv->iLVDS_enable) {
#ifdef CONFIG_X86_MDFLD
        	        int ret=0;
			ret = intel_scu_ipc_simple_command(IPC_MSG_PANEL_ON_OFF, IPC_CMD_PANEL_ON);
                	if (ret)
                        	printk(KERN_WARNING "IPC 0xE9 failed to turn on pnl pwr.  Error is: %x\n", ret);
	                msleep(2000); /* wait 2 seconds */
#endif
        	}

		restore_display_registers(dev);
	}
}

#if 1 /* for debugging ospm */
/*
 * ospm_suspend_pci
 *
 * Description: Suspend the pci device saving state and disabling
 * as necessary.
 */
static void ospm_suspend_pci(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct drm_psb_private *dev_priv = dev->dev_private;
	int bsm, vbt;

	if (gbSuspended)
		return;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "ospm_suspend_pci\n");
#endif

	pci_save_state(pdev);
	pci_read_config_dword(pdev, 0x5C, &bsm);
	dev_priv->saveBSM = bsm;
	pci_read_config_dword(pdev, 0xFC, &vbt);
	dev_priv->saveVBT = vbt;
	pci_read_config_dword(pdev, PSB_PCIx_MSI_ADDR_LOC, &dev_priv->msi_addr);
	pci_read_config_dword(pdev, PSB_PCIx_MSI_DATA_LOC, &dev_priv->msi_data);

	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
	/*
	Disabling the IPC call to SCU
	to turn off Vprog2 as SCU doesn't
	support this IPC.
	This will be enabled once SCU support
	is available.
	*/

#if 0
	/*Notify SCU on GFX suspend for VProg2.*/
	{
		unsigned int gfx_off = 0;
		unsigned int  ret = 0;
		ret = intel_scu_ipc_command(SCU_CMD_VPROG2, 0, &gfx_off, 1, NULL, 0);
		if (ret)
			printk(KERN_WARNING
			"%s IPC 0xE3 failed; error is: %x\n", __func__, ret);
	}
#endif

	gbSuspended = true;
	gbgfxsuspended = true;
}

/*
 * ospm_resume_pci
 *
 * Description: Resume the pci device restoring state and enabling
 * as necessary.
 */
static bool ospm_resume_pci(struct pci_dev *pdev)
{
	struct drm_device *dev = pci_get_drvdata(pdev);
	struct drm_psb_private *dev_priv = dev->dev_private;
	int ret = 0;

	if (!gbSuspended)
		return true;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "ospm_resume_pci\n");
#endif
	/*
	Disabling the IPC call to SCU
	to turn on Vprog2 as SCU doesn't
	support this IPC.
	This will be enabled once SCU support
	is available.
	*/
#if 0
	/*Notify SCU on GFX resume for VProg2.*/
	{
		unsigned int gfx_on = 1;
		unsigned int ret = 0;
		ret = intel_scu_ipc_command(SCU_CMD_VPROG2, 0, &gfx_on, 1, NULL, 0);
		if (ret)
			printk(KERN_WARNING
			"%s IPC 0xE3 failed; error is: %x\n", __func__, ret);
	}
#endif

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_write_config_dword(pdev, 0x5c, dev_priv->saveBSM);
	pci_write_config_dword(pdev, 0xFC, dev_priv->saveVBT);
	/* retoring MSI address and data in PCIx space */
	pci_write_config_dword(pdev, PSB_PCIx_MSI_ADDR_LOC, dev_priv->msi_addr);
	pci_write_config_dword(pdev, PSB_PCIx_MSI_DATA_LOC, dev_priv->msi_data);
	ret = pci_enable_device(pdev);

	if (ret != 0)
		printk(KERN_ALERT "ospm_resume_pci: pci_enable_device failed: %d\n", ret);
	else
		gbSuspended = false;

	return !gbSuspended;
}
#endif

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
static void dsi_lvds_panel_get_hdmi_audio_status(void)
{
#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
	struct snd_intel_had_interface *had_interface = dev_priv->had_interface;
	pm_event_t hdmi_audio_event;

	if (dev_priv->had_pvt_data && hdmi_state) {
		hdmi_audio_event.event = 0;
		dev_priv->hdmi_audio_busy =
			had_interface->suspend(dev_priv->had_pvt_data,
					hdmi_audio_event);
	}
#endif
}

static void gfx_redridge_early_suspend(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev ? dev->dev_private : NULL;
	struct drm_encoder *encoder = NULL;
	struct drm_crtc *crtc = NULL;
	struct drm_encoder_helper_funcs *enc_funcs;

	if (!dev || !dev_priv) {
		pr_err("%s: dev pointor is NULL\n", __func__);
		return;
	}

	if (dev_priv->encoder0 && (dev_priv->panel_desc & DISPLAY_A)) {
		if (dev_priv->bhdmiconnected)
			dsi_lvds_panel_get_hdmi_audio_status();
		if (dev_priv->hdmi_audio_busy) {
			pr_debug("%s: hdmi audio busy\n", __func__);
			dsi_lvds_toshiba_bridge_panel_off();
			dsi_set_pipe_plane_enable_state(dev, 0, 0);
		} else {
			mdfld_dsi_dpi_set_power(&dev_priv->encoder0->base,
					false);
		}
	}

	if (dev_priv->encoder2 && (dev_priv->panel_desc & DISPLAY_C))
		mdfld_dsi_dpi_set_power(&dev_priv->encoder2->base, false);

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		enc_funcs = encoder->helper_private;
		if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS
				&& drm_helper_encoder_in_use(encoder)) {
			if (enc_funcs && enc_funcs->save)
				enc_funcs->save(encoder);
		}
	}
}

static void gfx_redridge_late_resume(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev ? dev->dev_private : NULL;
	struct drm_encoder *encoder = NULL;
	struct drm_crtc *crtc = NULL;
	struct drm_encoder_helper_funcs *enc_funcs;

	if (!dev || !dev_priv) {
		pr_err("%s: dev pointor is NULL\n", __func__);
		return;
	}

	if (dev_priv->encoder0 && (dev_priv->panel_desc & DISPLAY_A)) {
		if (dev_priv->hdmi_audio_busy) {
			pr_debug("%s: hdmi audio busy\n", __func__);
			dsi_lvds_toshiba_bridge_panel_on(dev);
			dsi_set_pipe_plane_enable_state(dev, 1, 0);
			dev_priv->hdmi_audio_busy = 0;
		} else {
			encoder = &dev_priv->encoder0->base;
			crtc = encoder->crtc;
			if (crtc)
				mdfld_dsi_dpi_mode_set(
						encoder,
						&crtc->mode,
						&crtc->hwmode);
			mdfld_dsi_dpi_set_power(encoder, true);
		}
	}
	if (dev_priv->encoder2 && (dev_priv->panel_desc & DISPLAY_C)) {
		encoder = &dev_priv->encoder2->base;
		crtc = encoder->crtc;
		if (crtc)
			mdfld_dsi_dpi_mode_set(
					encoder,
					&crtc->mode,
					&crtc->hwmode);
		mdfld_dsi_dpi_set_power(encoder, true);
	}
	list_for_each_entry(encoder,
			&dev->mode_config.encoder_list,
			head) {
		enc_funcs = encoder->helper_private;
		if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS
				&& drm_helper_encoder_in_use(encoder)) {
			if (enc_funcs && enc_funcs->restore)
				enc_funcs->restore(encoder);
		}
	}
}
#endif

#if SUPPORT_EARLY_SUSPEND
static void gfx_early_suspend(struct early_suspend *h)
{
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
	struct drm_device *dev = dev_priv->dev;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *enc_funcs;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "\n   gfx_early_suspend\n");
#endif

	if (h) {
		while (dev_priv->is_in_panel_reset) {
			mdelay(100);
			printk(KERN_ALERT "===power key wait\n");
		}
	}
	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);
	/*Display off*/
	if (IS_MDFLD(gpDrmDevice)) {
		if ((dev_priv->panel_id == TMD_VID) ||
			(dev_priv->panel_id == H8C7_VID) ||
			(dev_priv->panel_id == TMD_6X10_VID) ||
			(dev_priv->panel_id == GI_SONY_VID) ||
			(dev_priv->panel_id == GI_SONY_CMD) ||
			(dev_priv->panel_id == H8C7_CMD) ||
			(dev_priv->panel_id == AUO_SC1_VID) ||
			/* SC1 setting */
			(dev_priv->panel_id == AUO_SC1_CMD)) {
#if defined(CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY) || \
			defined(CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE)
			gfx_redridge_early_suspend(dev);
#else
			list_for_each_entry(encoder,
					&dev->mode_config.encoder_list,
					head) {
				enc_funcs = encoder->helper_private;
				if (!drm_helper_encoder_in_use(encoder))
					continue;
				if (enc_funcs && enc_funcs->save)
					enc_funcs->save(encoder);
			}
#endif
		} else if (dev_priv->panel_id == TPO_CMD) {
			if (dev_priv->encoder0 &&
				(dev_priv->panel_desc & DISPLAY_A))
				mdfld_dsi_dbi_set_power(
					&dev_priv->encoder0->base, false);
			if (dev_priv->encoder2 &&
				(dev_priv->panel_desc & DISPLAY_C))
				mdfld_dsi_dbi_set_power(
					&dev_priv->encoder2->base, false);
		} else
			printk(KERN_ALERT "panel type is not support currently\n");
	}

	gbdispstatus = false;

#ifdef CONFIG_GFX_RTPM
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT " allow GFX runtime_pm\n");
#endif
	pm_runtime_allow(&gpDrmDevice->pdev->dev);
#endif

}
#endif /* if SUPPORT_EARLY_SUSPEND */

static void resume_data_back(void)
{
	pm_runtime_forbid(&gpDrmDevice->pdev->dev);
	mutex_lock(&g_ospm_mutex);
	ospm_resume_pci(gpDrmDevice->pdev);
	ospm_resume_display(gpDrmDevice->pdev);
	psb_irq_preinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
	psb_irq_postinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
	mutex_unlock(&g_ospm_mutex);
}
static void restore_panel_controll_back(struct drm_psb_private *dev_priv)
{
	struct drm_device *dev = dev_priv->dev;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *enc_funcs;
	u32 dspcntr_val;

	if (IS_MDFLD(gpDrmDevice)) {
		if ((dev_priv->panel_id == TMD_VID) ||
			(dev_priv->panel_id == H8C7_VID) ||
			(dev_priv->panel_id == TMD_6X10_VID) ||
			(dev_priv->panel_id == GI_SONY_VID) ||
			(dev_priv->panel_id == GI_SONY_CMD) ||
			(dev_priv->panel_id == H8C7_CMD) ||
			(dev_priv->panel_id == AUO_SC1_VID) ||
			/* SC1 setting */
			(dev_priv->panel_id == AUO_SC1_CMD)) {
#if defined(CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY) || \
			defined(CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE)
			gfx_redridge_late_resume(dev);
#else
			list_for_each_entry(encoder,
					&dev->mode_config.encoder_list,
					head) {
				enc_funcs = encoder->helper_private;
				if (!drm_helper_encoder_in_use(encoder))
					continue;
				if (enc_funcs && enc_funcs->restore)
					enc_funcs->restore(encoder);
			}
#endif
		} else if (dev_priv->panel_id == TPO_CMD) {
			if (dev_priv->encoder0 &&
				(dev_priv->panel_desc & DISPLAY_A))
				mdfld_dsi_dbi_set_power(
					&dev_priv->encoder0->base, true);
			if (dev_priv->encoder2 &&
				(dev_priv->panel_desc & DISPLAY_C))
				mdfld_dsi_dbi_set_power(
					&dev_priv->encoder2->base, true);
		} else {
			printk(KERN_ALERT "%s invalid panel\n",
				__func__);
		}

		if (dev_priv->panel_desc & DISPLAY_B) {
			dspcntr_val = PSB_RVDC32(DSPBCNTR);
			/* comply the status with HDMI DPMS */
			if (DISP_PLANEB_STATUS == DISPLAY_PLANE_DISABLE)
				PSB_WVDC32(dspcntr_val
					& ~DISPLAY_PLANE_ENABLE, DSPBCNTR);
			else
				PSB_WVDC32(dspcntr_val
					| DISPLAY_PLANE_ENABLE, DSPBCNTR);
		}
		if (dev_priv->pvr_screen_event_handler)
			dev_priv->pvr_screen_event_handler(dev, 1);
		gbdispstatus = true;

		if (lastFailedBrightness > 0)
			psb_set_brightness(NULL);
	}

}

#if SUPPORT_EARLY_SUSPEND
static void gfx_late_resume(struct early_suspend *h)
{
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "\ngfx_late_resume\n");
#endif

	if (IS_MDFLD(gpDrmDevice)) {
#ifdef CONFIG_GFX_RTPM
		resume_data_back();
#endif
	}
	restore_panel_controll_back(dev_priv);
}
#endif /* if SUPPORT_EARLY_SUSPEND */

/*
 * ospm_power_suspend
 *
 * Description: OSPM is telling our driver to suspend so save state
 * and power down all hardware.
 */
int ospm_power_suspend(struct pci_dev *pdev, pm_message_t state)
{
	int ret = 0;
	int graphics_access_count;
	int videoenc_access_count;
	int videodec_access_count;
	int display_access_count;

#if ((defined CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE) \
	|| (defined CONFIG_SND_INTELMID_HDMI_AUDIO) \
	|| (defined CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
#endif

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
	if (dev_priv->hdmi_audio_busy)
		return -EBUSY;
#endif

#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
	struct snd_intel_had_interface *had_interface = dev_priv->had_interface;
	int hdmi_audio_busy = 0;
	pm_event_t hdmi_audio_event;
#endif
	if(gbSuspendInProgress || gbResumeInProgress)
        {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s system BUSY\n", __func__);
#endif
                return  -EBUSY;
        }
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
        mutex_lock(&g_ospm_mutex);
        if (!gbSuspended) {
                graphics_access_count = atomic_read(&g_graphics_access_count);
                videoenc_access_count = atomic_read(&g_videoenc_access_count);
                videodec_access_count = atomic_read(&g_videodec_access_count);
                display_access_count = atomic_read(&g_display_access_count);
#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
		if (dev_priv->had_pvt_data && hdmi_state) {
			hdmi_audio_event.event = 0;
			hdmi_audio_busy =
				had_interface->suspend(dev_priv->had_pvt_data,
							hdmi_audio_event);
		}
#endif

                if (graphics_access_count
			|| videoenc_access_count
			|| videodec_access_count
			|| display_access_count
#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
			|| hdmi_audio_busy
#endif
		)
                        ret = -EBUSY;
                if (!ret) {
                        gbSuspendInProgress = true;

                        //! may cause runtime resume during early suspend
                        //psb_irq_uninstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
                        ospm_suspend_display(gpDrmDevice);
#if 0   /* FIXME: video driver support for Linux Runtime PM */
			/* FIXME: video driver support for Linux Runtime PM */
                        if (ospm_runtime_pm_msvdx_suspend(gpDrmDevice) != 0) {
				suspend_pci = false;
                        }

#endif
                        if (ospm_runtime_pm_topaz_suspend(gpDrmDevice) != 0)
				ret = -EBUSY;

			if (!ret)
				ospm_suspend_pci(pdev);
                        gbSuspendInProgress = false;
                } else {
                        printk(KERN_ALERT "ospm_power_suspend: device busy: graphics %d videoenc %d videodec %d display %d\n", graphics_access_count, videoenc_access_count, videodec_access_count, display_access_count);
                }
        }


        mutex_unlock(&g_ospm_mutex);
        return ret;
}

/*
 * ospm_power_island_up
 *
 * Description: Restore power to the specified island(s) (powergating)
 */
void ospm_power_island_up(int hw_islands)
{
	u32 dc_islands  = 0;
	u32 gfx_islands = hw_islands;
	unsigned long flags;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) gpDrmDevice->dev_private;

#ifdef OSPM_GFX_DPK
	 printk(KERN_ALERT "%s hw_islands: %x\n",
		 __func__, hw_islands);
#endif
	if (hw_islands & OSPM_DISPLAY_ISLAND) {
		/*Power-up required islands only*/
		if (dev_priv->panel_desc & DISPLAY_A)
			dc_islands |= OSPM_DISPLAY_A_ISLAND;

		if (dev_priv->panel_desc & DISPLAY_B)
			dc_islands |= OSPM_DISPLAY_B_ISLAND;

		if (dev_priv->panel_desc & DISPLAY_C)
			dc_islands |= OSPM_DISPLAY_C_ISLAND;

		if (dev_priv->panel_desc)
			dc_islands |= OSPM_MIPI_ISLAND;

		/*
		If pmu_nc_set_power_state fails then accessing HW
		reg would result in a crash - IERR/Fabric error.
		*/
		spin_lock_irqsave(&dev_priv->ospm_lock, flags);
		if (pmu_nc_set_power_state(dc_islands,
			OSPM_ISLAND_UP, OSPM_REG_TYPE))
			BUG();
		g_hw_power_status_mask |= OSPM_DISPLAY_ISLAND;
		spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_DISPLAY_ISLAND;
	}

	if (gfx_islands) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s other hw_islands: %x\n",
			 __func__, gfx_islands);
#endif
		/*
		If pmu_nc_set_power_state fails then accessing HW
		reg would result in a crash - IERR/Fabric error.
		*/
#ifdef CONFIG_MDFD_GL3
		if (IS_D0(gpDrmDevice)) {
			/*
			 * GL3 power island needs to be on for MSVDX working.
			 * We found this during enabling new MSVDX firmware
			 * uploading mechanism(by PUNIT) for Penwell D0.
			 */
			if ((gfx_islands & OSPM_VIDEO_DEC_ISLAND) &&
					!ospm_power_is_hw_on(OSPM_GL3_CACHE_ISLAND))
				gfx_islands |= OSPM_GL3_CACHE_ISLAND;
		}
#endif
		spin_lock_irqsave(&dev_priv->ospm_lock, flags);
		if (pmu_nc_set_power_state(gfx_islands,
					   OSPM_ISLAND_UP, APM_REG_TYPE))
			BUG();
		g_hw_power_status_mask |= gfx_islands;
		spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);
	}
}
/*
 * ospm_power_resume
 */
int ospm_power_resume(struct pci_dev *pdev)
{
	if(gbSuspendInProgress || gbResumeInProgress)
        {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s: Suspend/ResumeInProgress\n",
			__func__);
#endif
                return 0;
        }

        mutex_lock(&g_ospm_mutex);

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "OSPM_GFX_DPK: ospm_power_resume\n");
#endif

  	gbResumeInProgress = true;

        ospm_resume_pci(pdev);

	ospm_resume_display(gpDrmDevice->pdev);
        psb_irq_preinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
        psb_irq_postinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);

	gbResumeInProgress = false;

        mutex_unlock(&g_ospm_mutex);

	return 0;
}


/*
 * ospm_power_island_down
 *
 * Description: Cut power to the specified island(s) (powergating)
 */
void ospm_power_island_down(int hw_islands)
{
	u32 dc_islands = 0;
	u32 gfx_islands = hw_islands;
	unsigned long flags;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) gpDrmDevice->dev_private;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s hw_islands: %x\n",
		__func__, hw_islands);
#endif

	if (hw_islands & OSPM_DISPLAY_ISLAND) {
		/*Power gate all display islands.*/
		dc_islands |= (OSPM_DISPLAY_A_ISLAND |
				OSPM_DISPLAY_B_ISLAND |
				OSPM_DISPLAY_C_ISLAND |
				OSPM_MIPI_ISLAND);

		/*
		If pmu_nc_set_power_state fails then accessing HW
		reg would result in a crash - IERR/Fabric error.
		*/
		spin_lock_irqsave(&dev_priv->ospm_lock, flags);
		g_hw_power_status_mask &= ~OSPM_DISPLAY_ISLAND;
		if (pmu_nc_set_power_state(dc_islands,
					   OSPM_ISLAND_DOWN, OSPM_REG_TYPE))
			BUG();
		spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_DISPLAY_ISLAND;
	}

	if (gfx_islands) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s other hw_islands: %x\n",
			 __func__, gfx_islands);
#endif
		spin_lock_irqsave(&dev_priv->ospm_lock, flags);
		if (gfx_islands & OSPM_GL3_CACHE_ISLAND) {
#ifdef CONFIG_MDFD_GL3
			/*
			Make sure both GFX & Video aren't
			using GL3
			*/
			if (atomic_read(&g_graphics_access_count) ||
					(g_hw_power_status_mask &
					(OSPM_VIDEO_DEC_ISLAND |
					OSPM_VIDEO_ENC_ISLAND |
					OSPM_GRAPHICS_ISLAND)) ||
					(drm_psb_gl3_enable == 0)) {
#ifdef OSPM_GFX_DPK
				printk(KERN_ALERT
				"%s GL3 in use - can't turn OFF\n",
				__func__);
#endif
				gfx_islands &=  ~OSPM_GL3_CACHE_ISLAND;
				if (!gfx_islands)
					goto out;
			}
#endif
		}

		/*
		If pmu_nc_set_power_state fails then accessing HW
		reg would result in a crash - IERR/Fabric error.
		*/
		g_hw_power_status_mask &= ~gfx_islands;
		if (pmu_nc_set_power_state(gfx_islands,
			OSPM_ISLAND_DOWN, APM_REG_TYPE))
			BUG();
out:
		spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);
	}
}

/*
 * ospm_power_is_hw_on
 *
 * Description: do an instantaneous check for if the specified islands
 * are on.  Only use this in cases where you know the g_state_change_mutex
 * is already held such as in irq install/uninstall.  Otherwise, use
 * ospm_power_using_hw_begin().
 */
bool ospm_power_is_hw_on(int hw_islands)
{
	unsigned long flags;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) gpDrmDevice->dev_private;
	bool ret = false;
	spin_lock_irqsave(&dev_priv->ospm_lock, flags);
	ret = ((g_hw_power_status_mask & hw_islands)
			== hw_islands) ? true : false;
	spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);
	return ret;
}

/*
 * ospm_power_using_hw_begin
 *
 * Description: Notify PowerMgmt module that you will be accessing the
 * specified island's hw so don't power it off.  If force_on is true,
 * this will power on the specified island if it is off.
 * Otherwise, this will return false and the caller is expected to not
 * access the hw.
 *
 * NOTE *** If this is called from and interrupt handler or other atomic
 * context, then it will return false if we are in the middle of a
 * power state transition and the caller will be expected to handle that
 * even if force_on is set to true.
 */
bool ospm_power_using_hw_begin(int hw_island, UHBUsage usage)
{
	bool ret = true;
	bool island_is_off = false;
	bool b_atomic = (in_interrupt() || in_atomic());
	bool locked = true;
	struct pci_dev *pdev = gpDrmDevice->pdev;
	IMG_UINT32 deviceID = 0;
	bool force_on = usage ? true: false;

#ifdef CONFIG_GFX_RTPM
	if(gbSuspendInProgress) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s Suspend In Progress, call pm_runtime_get_noresume\n", __func__);
#endif
		pm_runtime_get_noresume(&pdev->dev);
	} else {
		pm_runtime_get(&pdev->dev);
	}
#endif
	/*quick path, not 100% race safe, but should be enough comapre to current other code in this file */
	if (!force_on) {
		if (!ospm_power_is_hw_on(hw_island)) {
#ifdef CONFIG_GFX_RTPM
			pm_runtime_put(&pdev->dev);
#endif
			return false;
        }
		else {
			locked = false;
			goto increase_count;
		}
	}
	if (!b_atomic)
		mutex_lock(&g_ospm_mutex);

	island_is_off = !ospm_power_is_hw_on(hw_island);

	if (b_atomic && (gbSuspendInProgress || gbResumeInProgress || gbSuspended) && force_on && island_is_off)
		ret = false;

	if (ret && island_is_off && !force_on)
		ret = false;

	if (ret && island_is_off && force_on) {
		gbResumeInProgress = true;

		ret = ospm_resume_pci(pdev);

		if (ret) {
			switch(hw_island)
			{
			case OSPM_DISPLAY_ISLAND:
				deviceID = gui32MRSTDisplayDeviceID;
				ospm_resume_display(pdev);
				psb_irq_preinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
				break;
			case OSPM_GRAPHICS_ISLAND:
				deviceID = gui32SGXDeviceID;
#ifdef CONFIG_MDFD_GL3
				ospm_power_island_up(OSPM_GRAPHICS_ISLAND |
						OSPM_GL3_CACHE_ISLAND);
#else
				ospm_power_island_up(OSPM_GRAPHICS_ISLAND);
#endif
				psb_irq_preinstall_islands(gpDrmDevice, OSPM_GRAPHICS_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice, OSPM_GRAPHICS_ISLAND);
				break;
#if 1 /* for debugging video driver ospm */
			case OSPM_VIDEO_DEC_ISLAND:
				if(!ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
					/* printk(KERN_ALERT "%s power on display for video decode use\n", __func__); */
					deviceID = gui32MRSTDisplayDeviceID;
					ospm_resume_display(pdev);
					psb_irq_preinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
					psb_irq_postinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
				}
				else{
					/* printk(KERN_ALERT "%s display is already on for video decode use\n", __func__);*/
				}

				if(!ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND)) {
					/* printk(KERN_ALERT "%s power on video decode\n", __func__); */
					deviceID = gui32MRSTMSVDXDeviceID;
#ifdef CONFIG_MDFD_GL3
					ospm_power_island_up(OSPM_GL3_CACHE_ISLAND | OSPM_VIDEO_DEC_ISLAND);
					if (IS_D0(gpDrmDevice)) {
						struct drm_psb_private *dev_priv =
							(struct drm_psb_private *) gpDrmDevice->dev_private;
						int ret;

						ret = psb_wait_for_register(dev_priv, MSVDX_COMMS_SIGNATURE,
									    MSVDX_COMMS_SIGNATURE_VALUE,
									    0xffffffff);
						if (ret)
							DRM_ERROR("MSVDX: firmware fails to initialize.\n");
					}
#else
					ospm_power_island_up(OSPM_VIDEO_DEC_ISLAND);
#endif
					ospm_runtime_pm_msvdx_resume(gpDrmDevice);
					psb_irq_preinstall_islands(gpDrmDevice, OSPM_VIDEO_DEC_ISLAND);
					psb_irq_postinstall_islands(gpDrmDevice, OSPM_VIDEO_DEC_ISLAND);
				}
				else{
					/* printk(KERN_ALERT "%s video decode is already on\n", __func__); */
				}

				break;
			case OSPM_VIDEO_ENC_ISLAND:
				if (IS_MRST(gpDrmDevice) &&
						(!ospm_power_is_hw_on(
							OSPM_DISPLAY_ISLAND))) {
					deviceID = gui32MRSTDisplayDeviceID;
					ospm_resume_display(pdev);
					psb_irq_preinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
					psb_irq_postinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
				}

				if(!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND)) {
					/* printk(KERN_ALERT "%s power on video encode\n", __func__); */
					deviceID = gui32MRSTTOPAZDeviceID;
#ifdef CONFIG_MDFD_GL3
					ospm_power_island_up(OSPM_VIDEO_ENC_ISLAND);
					ospm_power_island_up(OSPM_GL3_CACHE_ISLAND);
#else
					ospm_power_island_up(OSPM_VIDEO_ENC_ISLAND);
#endif
					ospm_runtime_pm_topaz_resume(gpDrmDevice);
					psb_irq_preinstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
					psb_irq_postinstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
				}
				else{
					/* printk(KERN_ALERT "%s video decode is already on\n", __func__); */
				}
#endif
				break;
			default:
				printk(KERN_ALERT "%s unknown island !!!!\n",
						__func__);
				break;
			}

		}

		if (!ret)
			printk(KERN_ALERT "%s: %d failed\n",
					__func__, hw_island);

		gbResumeInProgress = false;
	}
increase_count:
	if (ret) {
		switch(hw_island)
		{
		case OSPM_GRAPHICS_ISLAND:
			atomic_inc(&g_graphics_access_count);
			break;
		case OSPM_VIDEO_ENC_ISLAND:
			atomic_inc(&g_videoenc_access_count);
			break;
		case OSPM_VIDEO_DEC_ISLAND:
			atomic_inc(&g_videodec_access_count);
			break;
		case OSPM_DISPLAY_ISLAND:
			atomic_inc(&g_display_access_count);
			break;
		}

	}
#ifdef CONFIG_GFX_RTPM
	else{
		pm_runtime_put(&pdev->dev);
	}
#endif

	if (!b_atomic && locked)
		mutex_unlock(&g_ospm_mutex);

	return ret;
}
EXPORT_SYMBOL(ospm_power_using_hw_begin);


/*
 * ospm_power_using_hw_end
 *
 * Description: Notify PowerMgmt module that you are done accessing the
 * specified island's hw so feel free to power it off.  Note that this
 * function doesn't actually power off the islands.
 */
void ospm_power_using_hw_end(int hw_island)
{
	switch(hw_island)
	{
	case OSPM_GRAPHICS_ISLAND:
		atomic_dec(&g_graphics_access_count);
		break;
	case OSPM_VIDEO_ENC_ISLAND:
		atomic_dec(&g_videoenc_access_count);
		break;
	case OSPM_VIDEO_DEC_ISLAND:
		atomic_dec(&g_videodec_access_count);
		break;
	case OSPM_DISPLAY_ISLAND:
		atomic_dec(&g_display_access_count);
		break;
	}

#ifdef CONFIG_GFX_RTPM
	/* decrement runtime pm ref count */
	pm_runtime_put(&gpDrmDevice->pdev->dev);
#endif

	WARN_ON(atomic_read(&g_graphics_access_count) < 0);
	WARN_ON(atomic_read(&g_videoenc_access_count) < 0);
	WARN_ON(atomic_read(&g_videodec_access_count) < 0);
	WARN_ON(atomic_read(&g_display_access_count) < 0);
}
EXPORT_SYMBOL(ospm_power_using_hw_end);

int ospm_runtime_pm_allow(struct drm_device * dev)
{
	struct drm_psb_private * dev_priv = dev->dev_private;
	bool panel_on, panel_on2;

	PSB_DEBUG_ENTRY("%s\n", __func__);

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
	if(dev_priv->rpm_enabled)
		return 0;

	if(is_panel_vid_or_cmd(dev)) {
		/*DPI panel*/
		panel_on = dev_priv->dpi_panel_on;
		panel_on2 = dev_priv->dpi_panel_on2;
	} else {
		/*DBI panel*/
		panel_on = dev_priv->dbi_panel_on;
		panel_on2 = dev_priv->dbi_panel_on2;
	}

#ifdef CONFIG_GFX_RTPM
	if(panel_on && panel_on2) {
		pm_runtime_allow(&dev->pdev->dev);
		dev_priv->rpm_enabled = 1;
		DRM_INFO("Runtime PM enabled\n");
	}
#endif

	return 0;
}

void ospm_runtime_pm_forbid(struct drm_device * dev)
{
	struct drm_psb_private * dev_priv = dev->dev_private;

	DRM_INFO("%s\n", __func__);

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif

#ifdef CONFIG_GFX_RTPM
	pm_runtime_forbid(&dev->pdev->dev);
#endif
	dev_priv->rpm_enabled = 0;
}

int psb_runtime_suspend(struct device *dev)
{
	pm_message_t state;
	int ret = 0;
	state.event = 0;
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
        if (atomic_read(&g_graphics_access_count) || atomic_read(&g_videoenc_access_count)
		|| (gbdispstatus == true)
		|| atomic_read(&g_videodec_access_count) || atomic_read(&g_display_access_count)){
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "GFX:%d VEC:%d VED:%d DC:%d DSR:%d\n",
			atomic_read(&g_graphics_access_count),
			atomic_read(&g_videoenc_access_count),
			atomic_read(&g_videodec_access_count),
			atomic_read(&g_display_access_count), gbdispstatus);
#endif
                return -EBUSY;
        }
        else
		ret = ospm_power_suspend(gpDrmDevice->pdev, state);

	return ret;
}

int psb_runtime_resume(struct device *dev)
{
	/* Notify HDMI Audio sub-system about the resume. */

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
	/* Nop for GFX */
	return 0;
}

int psb_runtime_idle(struct device *dev)
{
#if ((defined CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE) \
	|| (defined CONFIG_SND_INTELMID_HDMI_AUDIO) \
	|| (defined CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
#endif

#if (defined(CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE))
	if (dev_priv->hdmi_audio_busy)
		return -EBUSY;
#endif

#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
		defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
	struct snd_intel_had_interface *had_interface = dev_priv->had_interface;
	int hdmi_audio_busy = 0;
	pm_event_t hdmi_audio_event;

	if (dev_priv->had_pvt_data && hdmi_state) {
		hdmi_audio_event.event = 0;
		hdmi_audio_busy =
			had_interface->suspend(dev_priv->had_pvt_data,
					hdmi_audio_event);
	}
#endif

	if (atomic_read(&g_graphics_access_count) || atomic_read(&g_videoenc_access_count)
		|| atomic_read(&g_videodec_access_count) || atomic_read(&g_display_access_count)
		|| (gbdispstatus == true)
#if (defined(CONFIG_SND_INTELMID_HDMI_AUDIO) || \
	defined(CONFIG_SND_INTELMID_HDMI_AUDIO_MODULE))
		|| hdmi_audio_busy
#endif
#if 0   /* FIXME: video driver support for Linux Runtime PM */
		|| (msvdx_hw_busy == 1)
		|| (topaz_hw_busy == 1))
#else
		)
#endif
			return -EBUSY;
		else
			return 0;
}

#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE
DEFINE_MUTEX(vadd_mutex);
static int i2c_access_count;

/* use access count to mark status of i2c bus 2, and make sure avdd is turned on
 * when accessing this i2c. when accaccess count reaches 1, then turn on lvds
 * panel's avdd
 */
void vlcm_vadd_get()
{
	mutex_lock(&vadd_mutex);
	++i2c_access_count;
	if (i2c_access_count == 1) {
		if (gpio_direction_output(GPIO_MIPI_LCD_VADD, 1)) {
			pr_err("%s: faild to pull high VADD\n", __func__);
			goto unlock;
		}
		msleep(260);
	}
unlock:
	mutex_unlock(&vadd_mutex);
}

/* decrease reference count, and turn vadd off when count reaches 0
 */
void vlcm_vadd_put()
{
	mutex_lock(&vadd_mutex);
	if (i2c_access_count == 0) {
		pr_warn("%s: i2c_access_count is 0\n", __func__);
		goto unlock;
	}

	--i2c_access_count;
	if (i2c_access_count > 0)
		goto unlock;
	/* i2c_access_count == 0 */
	if (gpio_direction_output(GPIO_MIPI_LCD_VADD, 0)) {
		pr_err("%s: faild to pull low VADD\n",
				__func__);
	}
unlock:
	mutex_unlock(&vadd_mutex);
}
#endif
void mdfld_reset_panel_handler_work(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
		container_of(work, struct drm_psb_private, reset_panel_work);
	int mipi_pipe = dev_priv->cur_pipe;
	struct drm_device *dev = dev_priv->dev;
	struct mdfld_dsi_config *dsi_config = NULL;

	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct panel_funcs *p_funcs  = NULL;
	int delay_between_dispaly_island_off_on = 20;
	int delay_after_reset_gpio_toggle = 20;

	/*if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
	*				OSPM_UHB_FORCE_POWER_ON))
	*	return -EAGAIN;
	*/
	dev_priv->is_in_panel_reset = true;
	if (!dev_priv->is_mipi_on)
		goto reset_error;

	if (mipi_pipe == 0) {
		dbi_output = dev_priv->dbi_output;
		dsi_config = dev_priv->dsi_configs[0];
	} else {
		dbi_output = dev_priv->dbi_output2;
		dsi_config = dev_priv->dsi_configs[1];
	}
	if (!dbi_output) {
		printk(KERN_ALERT "%s invalid dbi_output\n",
						__func__);
		goto reset_error;
	}
	p_funcs = dbi_output->p_funcs;
	if (p_funcs) {
#if SUPPORT_EARLY_SUSPEND
		gfx_early_suspend(NULL);
#endif /* if SUPPORT_EARLY_SUSPEND */
		if (p_funcs->get_reset_delay_time)
			p_funcs->get_reset_delay_time(
					&delay_between_dispaly_island_off_on,
					&delay_after_reset_gpio_toggle);
		mdelay(delay_between_dispaly_island_off_on);

		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
						OSPM_UHB_FORCE_POWER_ON))
			goto reset_error;

		if (p_funcs->reset)
			p_funcs->reset(dsi_config, RESET_FROM_OSPM_RESUME);

		mdelay(delay_after_reset_gpio_toggle);
		if (p_funcs->disp_control_init)
			p_funcs->disp_control_init(dev);

		if (IS_MDFLD(gpDrmDevice))
			resume_data_back();

		if (p_funcs->drv_ic_init)
			p_funcs->drv_ic_init(dsi_config, mipi_pipe);

		restore_panel_controll_back(dev_priv);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		printk(KERN_ALERT"End panel reset!!!\n");
	} else {
		printk(KERN_ALERT "%s invalid panel init\n",
								__func__);
	}
reset_error:
	dev_priv->is_in_panel_reset = false;
	/*ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);*/
}

