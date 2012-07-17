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
#include "pnw_topaz.h"
#include "vsp.h"
#include <linux/mutex.h>
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include <asm/intel_scu_ipc.h>
#include "psb_intel_hdmi.h"
#ifdef CONFIG_GFX_RTPM
#include <linux/pm_runtime.h>
#endif

#include <linux/earlysuspend.h>
#include <asm/intel-mid.h>

#undef OSPM_GFX_DPK
#define SCU_CMD_VPROG2  0xe3

/*extern int drm_psb_dsr; */
struct drm_device *gpDrmDevice;
static struct mutex g_ospm_mutex;
static bool gbSuspendInProgress;
static bool gbResumeInProgress;
static int g_hw_power_status_mask;
static atomic_t g_display_access_count;
static atomic_t g_graphics_access_count;
static atomic_t g_videoenc_access_count;
static atomic_t g_videodec_access_count;
static atomic_t g_videovsp_access_count;

static bool gbSuspended;
bool gbgfxsuspended;

static void psb_runtimepm_wq_handler(struct work_struct *work);
DECLARE_DELAYED_WORK(rtpm_work, psb_runtimepm_wq_handler);

void psb_runtimepm_wq_handler(struct work_struct *work)
{
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;

#ifdef CONFIG_GFX_RTPM
	if (gbdispstatus == false)
		pm_runtime_allow(&gpDrmDevice->pdev->dev);
#endif
}

/*
 * gfx_early_suspend
 *
 */
static void gfx_early_suspend(struct early_suspend *h);
static void gfx_late_resume(struct early_suspend *h);

static struct early_suspend gfx_early_suspend_desc = {
	/* .level = EARLY_SUSPEND_LEVEL_STOP_DRAWING, */
	/* .suspend = gfx_early_suspend, */
	/* .resume = gfx_late_resume, */
};

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

	if (IS_FLDS(dev)) {
		if (pnw_check_topaz_idle(dev)) {
			ret = -2;
			goto out;
		}
	}

	psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);

	if (IS_FLDS(dev)) {
		if (encode_running)	/* has encode session running */
			pnw_topaz_save_mtx_state(gpDrmDevice);
		PNW_TOPAZ_NEW_PMSTATE(dev, pnw_topaz_priv,
				      PSB_PMSTATE_POWERDOWN);
	}
	ospm_power_island_down(OSPM_VIDEO_ENC_ISLAND);
 out:
	return ret;
}

static int ospm_runtime_pm_msvdx_resume(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;

	/*printk(KERN_ALERT "ospm_runtime_pm_msvdx_resume\n"); */

	MSVDX_NEW_PMSTATE(dev, msvdx_priv, PSB_PMSTATE_POWERUP);

	psb_msvdx_restore_context(dev);

	return 0;
}

static int ospm_runtime_pm_topaz_resume(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct topaz_private *topaz_priv = dev_priv->topaz_private;
	struct pnw_topaz_private *pnw_topaz_priv = dev_priv->topaz_private;
	struct psb_video_ctx *pos, *n;
	int encode_ctx = 0, encode_running = 0;

	/*printk(KERN_ALERT "ospm_runtime_pm_topaz_resume\n"); */

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

	if (IS_FLDS(dev)) {
		if (encode_running) {	/* has encode session running */
			psb_irq_uninstall_islands(gpDrmDevice,
						  OSPM_VIDEO_ENC_ISLAND);
			pnw_topaz_restore_mtx_state(gpDrmDevice);
		}
		PNW_TOPAZ_NEW_PMSTATE(dev, pnw_topaz_priv, PSB_PMSTATE_POWERUP);
	}

	return 0;
}

static int ospm_runtime_pm_vsp_suspend(struct drm_device *dev)
{
	int ret = 0;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	/* whether the HW has been powered off */
	if (!ospm_power_is_hw_on(OSPM_VIDEO_VPP_ISLAND))
		goto out;

	/* whether the HW is used */
	if (atomic_read(&g_videovsp_access_count)) {
		ret = -1;
		goto out;
	}

	/* whether the HW is idle */
	if (psb_check_vsp_idle(dev)) {
		ret = -2;
		goto out;
	}

	psb_vsp_save_context(dev);

	ospm_power_island_down(OSPM_VIDEO_VPP_ISLAND);

	/* update the PM state */
	VSP_NEW_PMSTATE(dev, vsp_priv, PSB_PMSTATE_POWERDOWN);
out:
	return ret;
}

static int ospm_runtime_pm_vsp_resume(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	/*printk(KERN_ALERT "%s\n", __func__);*/

	psb_vsp_restore_context(dev);

	VSP_NEW_PMSTATE(dev, vsp_priv, PSB_PMSTATE_POWERUP);

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
	return;			/* todo for OSPM */

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
	if (psb_check_msvdx_idle(dev))
		goto out;

	gbSuspendInProgress = true;
	psb_msvdx_save_context(dev);
	ospm_power_island_down(OSPM_VIDEO_DEC_ISLAND);
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
	if (IS_FLDS(dev))
		if (pnw_check_topaz_idle(dev))
			goto out;

	gbSuspendInProgress = true;
	if (IS_FLDS(dev)) {
		psb_irq_uninstall_islands(dev, OSPM_VIDEO_ENC_ISLAND);
		pnw_topaz_save_mtx_state(gpDrmDevice);
		PNW_TOPAZ_NEW_PMSTATE(dev, pnw_topaz_priv,
				      PSB_PMSTATE_POWERDOWN);
	}
	ospm_power_island_down(OSPM_VIDEO_ENC_ISLAND);

	gbSuspendInProgress = false;
 out:
	mutex_unlock(&g_ospm_mutex);
	return;
}

void ospm_apm_power_down_vsp(struct drm_device *dev)
{

	struct drm_psb_private *dev_priv = dev->dev_private;
	struct vsp_private *vsp_priv = dev_priv->vsp_private;

	mutex_lock(&g_ospm_mutex);

	/* whether the HW has been powered off */
	if (!ospm_power_is_hw_on(OSPM_VIDEO_VPP_ISLAND))
		goto out;

	/* whether the HW is used */
	if (atomic_read(&g_videovsp_access_count))
		goto out;

	/* whether the HW is idle */
	if (psb_check_vsp_idle(dev))
		goto out;

	gbSuspendInProgress = true;

	/* The function should be implemented in vsp.c */
	psb_vsp_save_context(dev);

	ospm_power_island_down(OSPM_VIDEO_VPP_ISLAND);

	/* update the PM state */
	VSP_NEW_PMSTATE(dev, vsp_priv, PSB_PMSTATE_POWERDOWN);

	gbSuspendInProgress = false;

out:
	mutex_unlock(&g_ospm_mutex);
}
#endif
/*
 * ospm_power_init
 *
 * Description: Initialize this ospm power management module
 */
void ospm_power_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	gpDrmDevice = dev;

	mutex_init(&g_ospm_mutex);
	spin_lock_init(&dev_priv->ospm_lock);
	g_hw_power_status_mask = OSPM_ALL_ISLANDS;
	atomic_set(&g_display_access_count, 0);
	atomic_set(&g_graphics_access_count, 0);
	atomic_set(&g_videoenc_access_count, 0);
	atomic_set(&g_videodec_access_count, 0);
	atomic_set(&g_videovsp_access_count, 0);

	register_early_suspend(&gfx_early_suspend_desc);

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
	unregister_early_suspend(&gfx_early_suspend_desc);
	mutex_destroy(&g_ospm_mutex);
#ifdef CONFIG_GFX_RTPM
	pm_runtime_disable(&gpDrmDevice->pdev->dev);
	pm_runtime_set_suspended(&gpDrmDevice->pdev->dev);
#endif
}

/*
* ospm_post_init
*
* Description: Power gate unused GFX & Display islands.
*/
void ospm_post_init(struct drm_device *dev)
{
	u32 dc_islands = 0;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (IS_MDFLD(dev)) {
		/*Save & Power gate un-used display islands. */
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

#ifdef CONFIG_X86_MRST
		/* if HDMI is disabled in the kernel .config, then we want to
		   disable these MSIC power rails permanently.  */
#ifndef CONFIG_MDFD_HDMI
		/* turn off HDMI power rails */
		intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_OFF);
		intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_OFF);
#endif
#endif
	} else if (IS_MRFLD(dev)) {
		if (!(dev_priv->panel_desc & DISPLAY_A))
			dc_islands |= MRFLD_GFX_DSPA;

		if (!(dev_priv->panel_desc & DISPLAY_B)) {
			dc_islands |= MRFLD_GFX_DSPB;
			dc_islands |= MRFLD_GFX_HDMIO;
		}

		if (!(dev_priv->panel_desc & DISPLAY_C))
			dc_islands |= MRFLD_GFX_DSPC;

		if (!(dev_priv->panel_desc))
			dc_islands |= MRFLD_GFX_MIO;

		mrfld_set_power_state(OSPM_DISPLAY_ISLAND,
					dc_islands,
					POWER_ISLAND_DOWN);
	}
}

/*
 * mdfld_save_display_registers
 *
 * Description: We are going to suspend so save current display
 * register state.
 *
 */
static int mdfld_save_display_registers(struct drm_device *dev, int pipe)
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
	PSB_DEBUG_ENTRY("\n");

	/**
	 * For MIPI panels, all plane/pipe/port/DSI controller values
	 * were already saved in dsi_hw_context, no need to save/restore
	 * for these registers.
	 * NOTE: only support TMD panel now, add support for other MIPI
	 * panels later
	 */
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	if (pipe != 1 && ((get_panel_type(dev, pipe) == TMD_VID) ||
			  (get_panel_type(dev, pipe) == TMD_6X10_VID)))
		return 0;
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
		palette_val[i] = PSB_RVDC32(palette_reg + (i << 2));

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
 * Description: We are going to suspend so save current cursor
 * and overlay display register state.
 */
static int mdfld_save_cursor_overlay_registers(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	/*save cursor regs */
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
	struct mdfld_dsi_config *dsi_config = NULL;
	u32 i = 0;
	u32 dpll = 0;
	u32 timeout = 0;
	u32 reg_offset = 0;
	u32 temp = 0;
	u32 device_ready_reg = DEVICE_READY_REG;

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
	PSB_DEBUG_ENTRY("\n");

	/**
	 * For MIPI panels, all plane/pipe/port/DSI controller values
	 * were already saved in dsi_hw_context, no need to save/restore
	 * for these registers.
	 * NOTE: only support TMD panel now, add support for other MIPI
	 * panels later
	 */
#ifndef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
	if (pipe != 1 && ((get_panel_type(dev, pipe) == TMD_VID) ||
			  (get_panel_type(dev, pipe) == TMD_6X10_VID)))
		return 0;
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
		dspcntr_val = dev_priv->saveDSPBCNTR;
		dspstatus_val = dev_priv->saveDSPBSTATUS;
		palette_val = dev_priv->save_palette_b;
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

		dsi_config = dev_priv->dsi_configs[1];
		break;
	default:
		DRM_ERROR("%s, invalid pipe number.\n", __func__);
		return -EINVAL;
	}

	/*make sure VGA plane is off. it initializes to on after reset! */
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
	PSB_WVDC32(dspstatus_val, dspstatus_reg);

	/*set up the plane */
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
		/*set up pipe related registers */
		PSB_WVDC32(mipi_val, mipi_reg);

		/*setup MIPI adapter + MIPI IP registers */
		mdfld_dsi_controller_init(dsi_config, pipe);

		/* in_atomic() is forbid in checkpath.pl */
		/*if (in_atomic() || in_interrupt())*/
		if (in_interrupt())
			mdelay(20);
		else
			msleep(20);

		/*TODO: remove MIPI restore code later */
		/*dsi_config->dvr_ic_inited = 0; */
		/*mdfld_dsi_tmd_drv_ic_init(dsi_config, pipe); */
	}

	/* enable the plane */
	PSB_WVDC32(dspcntr_val, dspcntr_reg);

	/* in_atomic() is forbid in checkpath.pl */
	/*if (in_atomic() || in_interrupt()) */
	if (in_interrupt())
		mdelay(20);
	else
		msleep(20);

	if (drm_psb_dsr) {
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
	/*enable the pipe */
	PSB_WVDC32(pipeconf_val, pipeconf_reg);

	/* restore palette (gamma) */
	/*DRM_UDELAY(50000); */
	for (i = 0; i < 256; i++)
		PSB_WVDC32(palette_val[i], palette_reg + (i << 2));

	if (pipe == 1)
		return 0;

	if (IS_MRFLD(dev) && !is_panel_vid_or_cmd(dev))
		mdfld_enable_te(dev, pipe);

	return 0;
}

/*
 * mdfld_restore_cursor_overlay_registers
 *
 * Description: We are going to resume so restore cursor and overlay
 * register state.
 */
static int mdfld_restore_cursor_overlay_registers(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	/*Enable Cursor A */
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
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "ospm_save_display\n");
#endif
	mdfld_save_cursor_overlay_registers(dev);

	mdfld_save_display_registers(dev, 0);

	mdfld_save_display_registers(dev, 2);
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
	int pp_stat, ret = 0;
	u32 temp = 0;
	u32 device_ready_reg = DEVICE_READY_REG;
	u32 mipi_reg = MIPI;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
	if (!(g_hw_power_status_mask & OSPM_DISPLAY_ISLAND)) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s - IGNORING!!!!\n", __func__);
#endif
		return;
	}

	if (IS_FLDS(dev)) {
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

		if (drm_psb_dsr) {
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

	}

	ospm_power_island_down(OSPM_DISPLAY_ISLAND);
	if (drm_psb_dsr) {
		gbdispstatus = false;
		schedule_delayed_work(&rtpm_work, 0);
	}
}

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

	if (drm_psb_dsr) {
		gbdispstatus = true;
#ifdef CONFIG_GFX_RTPM
		pm_runtime_forbid(&gpDrmDevice->pdev->dev);
#endif
	}
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
	if (g_hw_power_status_mask & OSPM_DISPLAY_ISLAND) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s - IGNORING!!!!\n", __func__);
#endif
		return;
	}

	/* turn on the display power island */
	ospm_power_island_up(OSPM_DISPLAY_ISLAND);

	if (!IS_MRFLD(dev))
		PSB_WVDC32(pg->pge_ctl | _PSB_PGETBL_ENABLED, PSB_PGETBL_CTL);

	/* Don't reinitialize the GTT as it is unnecessary.  The gtt is
	 * stored in memory so it will automatically be restored.  All
	 * we need to do is restore the PGETBL_CTL which we already do
	 * above.
	 */
	/*psb_gtt_init(dev_priv->pg, 1); */

	if (IS_FLDS(dev)) {
		if (dev_priv->panel_desc & DISPLAY_C)
			mdfld_restore_display_registers(dev, 2);
		if (dev_priv->panel_desc & DISPLAY_A)
			mdfld_restore_display_registers(dev, 0);
		/*
		 * Don't restore Display B registers during resuming, if HDMI
		 * isn't turned on before suspending.
		 */
		if ((dev_priv->panel_desc & DISPLAY_B) &&
		    (dev_priv->saveHDMIB_CONTROL & HDMIB_PORT_EN))
			mdfld_restore_display_registers(dev, 1);
		mdfld_restore_cursor_overlay_registers(dev);
	}
}

#if 1				/* for debugging ospm */
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
	/*Notify SCU on GFX suspend for VProg2. */
	{
		unsigned int gfx_off = 0;
		unsigned int ret = 0;
		ret =
		    intel_scu_ipc_command(SCU_CMD_VPROG2, 0, &gfx_off, 1, NULL,
					  0);
		if (ret)
			printk(KERN_WARNING
			       "%s IPC 0xE3 failed; error is: %x\n", __func__,
			       ret);
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
	/*Notify SCU on GFX resume for VProg2. */
	{
		unsigned int gfx_on = 1;
		unsigned int ret = 0;
		ret =
		    intel_scu_ipc_command(SCU_CMD_VPROG2, 0, &gfx_on, 1, NULL,
					  0);
		if (ret)
			printk(KERN_WARNING
			       "%s IPC 0xE3 failed; error is: %x\n", __func__,
			       ret);
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
		printk(KERN_ALERT
		       "ospm_resume_pci: pci_enable_device failed: %d\n", ret);
	else
		gbSuspended = false;

	return !gbSuspended;
}
#endif

static void gfx_early_suspend(struct early_suspend *h)
{
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
	struct drm_device *dev = dev_priv->dev;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *enc_funcs;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "\n   gfx_early_suspend\n");
#endif

	if (drm_psb_dsr) {
		if (gbSuspended)
			return;
	}

	/*Display off */
	if (IS_FLDS(gpDrmDevice)) {
		if ((dev_priv->panel_id == TMD_VID) ||
		    (dev_priv->panel_id == TMD_6X10_VID)) {
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
			if (dev_priv->encoder0 &&
			    (dev_priv->panel_desc & DISPLAY_A))
				mdfld_dsi_dpi_set_power(dev_priv->encoder0,
							false);
			if (dev_priv->encoder2
			    && (dev_priv->panel_desc & DISPLAY_C))
				mdfld_dsi_dpi_set_power(dev_priv->encoder2,
							false);
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
				mdfld_dsi_dbi_set_power(&dev_priv->
							encoder0->base, false);
			if (dev_priv->encoder2
			    && (dev_priv->panel_desc & DISPLAY_C))
				mdfld_dsi_dbi_set_power(&dev_priv->
							encoder2->base, false);
		} else
			printk(KERN_ALERT
			       "panel type is not support currently\n");
	}

	gbdispstatus = false;

#ifdef CONFIG_GFX_RTPM
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT " allow GFX runtime_pm\n");
#endif
	schedule_delayed_work(&rtpm_work, 0);
#endif

}

static void gfx_late_resume(struct early_suspend *h)
{
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
	struct drm_device *dev = dev_priv->dev;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *enc_funcs;
#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "\ngfx_late_resume\n");
#endif

	if (drm_psb_dsr) {
		if (!gbSuspended)
			return;
	}

	if (IS_FLDS(gpDrmDevice)) {

#ifdef CONFIG_GFX_RTPM
		pm_runtime_forbid(&gpDrmDevice->pdev->dev);
		ospm_resume_pci(gpDrmDevice->pdev);
		ospm_resume_display(gpDrmDevice->pdev);
		psb_irq_preinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);
		psb_irq_postinstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND);

#endif
		if (IS_FLDS(gpDrmDevice)) {
			if ((dev_priv->panel_id == TMD_VID) ||
			    (dev_priv->panel_id == TMD_6X10_VID)) {
#ifdef CONFIG_SUPPORT_TOSHIBA_MIPI_DISPLAY
				if (dev_priv->encoder0 &&
				    (dev_priv->panel_desc & DISPLAY_A))
					mdfld_dsi_dpi_set_power
					    (dev_priv->encoder0, true);
				if (dev_priv->encoder2
				    && (dev_priv->panel_desc & DISPLAY_C))
					mdfld_dsi_dpi_set_power
					    (dev_priv->encoder2, true);
#else
				list_for_each_entry(encoder,
						    &dev->
						    mode_config.encoder_list,
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
					mdfld_dsi_dbi_set_power
					    (&dev_priv->encoder0->base, true);
				if (dev_priv->encoder2
				    && (dev_priv->panel_desc & DISPLAY_C))
					mdfld_dsi_dbi_set_power
					    (&dev_priv->encoder2->base, true);
			} else {
				printk(KERN_ALERT "%s invalid panel\n",
				       __func__);
			}
		}

		gbdispstatus = true;

		if (lastFailedBrightness > 0)
			psb_set_brightness(NULL);
	}
}

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
	int videovsp_access_count;
	bool suspend_pci = true;

	if (gbSuspendInProgress || gbResumeInProgress) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s system BUSY\n", __func__);
#endif
		return -EBUSY;
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
		videovsp_access_count = atomic_read(&g_videovsp_access_count);

		if (graphics_access_count ||
			videoenc_access_count ||
			videodec_access_count ||
			videovsp_access_count ||
			display_access_count)
			ret = -EBUSY;

		if (!ret) {
			gbSuspendInProgress = true;

	/* ! may cause runtime resume during early suspend
	 * psb_irq_uninstall_islands(gpDrmDevice, OSPM_DISPLAY_ISLAND); */
			ospm_suspend_display(gpDrmDevice);
#if 0			/* FIXME: video driver support for Linux Runtime PM */
			/* FIXME: video driver support for Linux Runtime PM */
			if (ospm_runtime_pm_msvdx_suspend(gpDrmDevice) != 0)
				suspend_pci = false;
#endif
			if (ospm_runtime_pm_topaz_suspend(gpDrmDevice) != 0)
				suspend_pci = false;

			if (suspend_pci == true)
				ospm_suspend_pci(pdev);

			gbSuspendInProgress = false;
		} else {
			printk(KERN_ALERT
			       "ospm_power_suspend: device busy: graphics %d "
				"videoenc %d videodec %d display %d\n",
			       graphics_access_count, videoenc_access_count,
			       videodec_access_count, display_access_count);
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
	u32 dc_islands = 0;
	u32 gfx_islands = hw_islands;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)gpDrmDevice->dev_private;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s hw_islands: %x\n", __func__, hw_islands);
#endif
	if (hw_islands & OSPM_DISPLAY_ISLAND) {

#ifdef CONFIG_MDFD_HDMI
		/* Always turn on MSIC VCC330 and VHDMI when display is on. */
		intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_ON);
		/* MSIC documentation requires that there be a 500us delay
		   after enabling VCC330 before you can enable VHDMI */
		usleep_range(500, 1000);
		/* turn on HDMI power rails */
		intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_ON | VHDMI_DB_30MS);
#endif
		/*Power-up required islands only */
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
		if (pmu_nc_set_power_state(dc_islands,
					   OSPM_ISLAND_UP, OSPM_REG_TYPE))
			BUG();

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_DISPLAY_ISLAND;
	}

	if (hw_islands & OSPM_VIDEO_VPP_ISLAND) {
		if (mrfld_set_power_state(
				OSPM_VIDEO_VPP_ISLAND,
				0,
				OSPM_ISLAND_UP))
			BUG();

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_VIDEO_VPP_ISLAND;
	}

	if (hw_islands & OSPM_VIDEO_DEC_ISLAND) {
		if (mrfld_set_power_state(
				OSPM_VIDEO_DEC_ISLAND,
				0,
				OSPM_ISLAND_UP))
			BUG();

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_VIDEO_DEC_ISLAND;
	}

	if (hw_islands & OSPM_VIDEO_ENC_ISLAND) {
		if (mrfld_set_power_state(
				OSPM_VIDEO_ENC_ISLAND,
				0,
				OSPM_ISLAND_UP))
			BUG();

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_VIDEO_ENC_ISLAND;
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
		if (pmu_nc_set_power_state(gfx_islands,
					   OSPM_ISLAND_UP, APM_REG_TYPE))
			BUG();
	}

	g_hw_power_status_mask |= hw_islands;
}

/*
 * ospm_power_resume
 */
int ospm_power_resume(struct pci_dev *pdev)
{
	if (gbSuspendInProgress || gbResumeInProgress) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "%s: Suspend/ResumeInProgress\n", __func__);
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

static int wait_for_pm_cmd_complete(int verify_mask, int state_type,
				    int reg_type)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)gpDrmDevice->dev_private;
	u32 pwr_sts = 0;
	u32 pwr_mask = 0;
	u32 pwr_cnt = 0;
	int count = 0;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s: verify_mask: 0x%x reg_type: 0x%x\n", __func__,
	       verify_mask, reg_type);
#endif

	if (reg_type < 0x30 && reg_type > 0x3f) {
		printk(KERN_ALERT "ERROR: %s invalid reg_type\n", __func__);
		return -EINVAL;
	}
	/* Request power state change */
	pwr_cnt = intel_mid_msgbus_read32(PUNIT_PORT, reg_type);

	pwr_mask = pwr_cnt;

	if (state_type == POWER_ISLAND_DOWN)
		/* pwr_mask |= verify_mask; */
		pwr_mask &= ~verify_mask;
	else if (state_type == POWER_ISLAND_UP)
		/* pwr_mask &= ~verify_mask; */
		pwr_mask |= verify_mask;
	else {
		printk(KERN_ALERT "ERROR: %s invalid power state: %d\n",
		       __func__, state_type);
		return -EINVAL;
	}

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s: current_value (reg_type:0x%x) :0x%x\n", __func__,
	       reg_type, pwr_cnt);
	printk(KERN_ALERT "%s: new value to be written: 0x%x\n", __func__,
	       pwr_mask);
#endif

	if (pwr_mask != pwr_cnt)
		intel_mid_msgbus_write32(PUNIT_PORT, reg_type, pwr_mask);
	else
		return 0;

	/* Poll for new power state */
	while (true) {
		pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg_type);
		if (state_type == POWER_ISLAND_DOWN) {
			if ((pwr_sts & pwr_mask) == pwr_mask)
				break;
			else
				udelay(10);
		} else if (state_type == POWER_ISLAND_UP) {
			if (pwr_sts == pwr_mask)
				break;
			else
				udelay(10);
		}
		count++;
		if (WARN_ONCE(count > 500000, "Timed out waiting for P-Unit"))
			return -EBUSY;
	}
	return 0;
}

int mrfld_set_power_state(int islands, int sub_islands, int state_type)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)gpDrmDevice->dev_private;
	unsigned long flags;
	int ret = 0;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s islands: 0x%x, sub_islands:0x%x,state_type:%s\n",
			__func__, islands, sub_island,
			state_type ? "on" : "off");
#endif

	spin_lock_irqsave(&dev_priv->ospm_lock, flags);

	switch (islands) {
	case OSPM_DISPLAY_ISLAND: {
		u32 pwr_cnt = 0;
		u32 pwr_mask = 0;
		u32 dsp_mask = 0;
		u32 gfx_mask = 0;
		u32 mio_mask = 0;
		u32 hdmio_mask = 0;

		switch (sub_islands) {
		case MRFLD_GFX_DSPA:
			dsp_mask |= DSPASSC;
			break;
		case MRFLD_GFX_DSPB:
			dsp_mask |= DSPBSSC;
			break;
		case MRFLD_GFX_DSPC:
			dsp_mask |= DSPCSSC;
			break;
		case MRFLD_GFX_MIO:
			mio_mask |= MIOSSC;
			break;
		case MRFLD_GFX_HDMIO:
			hdmio_mask |= HDMIOSSC;
			break;
		case MRFLD_GFX_SLC:
			gfx_mask |= GFX_SLC_SSC;
			break;
		case MRFLD_GFX_SDKCK:
			gfx_mask |= GFX_SDKCK_SSC;
			break;
		case MRFLD_GFX_RSCD:
			gfx_mask |= GFX_RSCD_SSC;
			break;
		case MRFLD_GFX_SLC_LDO:
			gfx_mask |= GFX_SLC_LDO_SSC;
			break;
		default:
			spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);
			printk(KERN_ALERT "ERROR: %s invalid sub_islands: 0x%x\n",
				__func__, sub_islands);

			return -EINVAL;
		}

		if (!ret && dsp_mask) {
			ret = wait_for_pm_cmd_complete(
						dsp_mask,
						state_type,
						DSP_SS_PM);
			if (ret)
				goto unlock;
		}

		if (!ret && mio_mask) {
			ret = wait_for_pm_cmd_complete(
						mio_mask,
						state_type,
						MIO_SS_PM);
			if (ret)
				goto unlock;
		}
		if (!ret && hdmio_mask) {
			ret = wait_for_pm_cmd_complete(
						hdmio_mask,
						state_type,
						HDMIO_SS_PM);
			if (ret)
				goto unlock;
		}
		if (!ret && gfx_mask) {
			ret = wait_for_pm_cmd_complete(
						gfx_mask,
						state_type,
						GFX_SS_PM0);
			if (ret)
				goto unlock;
		}
	}
		break;
	case OSPM_VIDEO_VPP_ISLAND:
		ret = wait_for_pm_cmd_complete(VSP_SSC, state_type, VSP_SS_PM0);
		if (ret)
			goto unlock;
		break;
	case OSPM_VIDEO_DEC_ISLAND:
		ret = wait_for_pm_cmd_complete(VED_SSC, state_type, VED_SS_PM0);
		if (ret)
			goto unlock;
		break;
	case OSPM_VIDEO_ENC_ISLAND:
		ret = wait_for_pm_cmd_complete(VEC_SSC, state_type, VEC_SS_PM0);
		if (ret)
			goto unlock;
		break;
	default:
		spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);
		printk(KERN_ALERT "Could NOT support island: %x\n", islands);
		return -EINVAL;
	}

unlock:
	spin_unlock_irqrestore(&dev_priv->ospm_lock, flags);
	if (ret)
		printk(KERN_ALERT "ERROR: wait_for_pm_cmd_complete FAILED!"
			"ret=0x%x, island=%d, sub-island=%d, state=%d\n",
			ret, islands, sub_islands, state_type);
	return ret;
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

	g_hw_power_status_mask &= ~hw_islands;

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s hw_islands: %x\n", __func__, hw_islands);
#endif

	if (hw_islands & OSPM_DISPLAY_ISLAND) {
		/*Power gate all display islands. */
		dc_islands |= (OSPM_DISPLAY_A_ISLAND |
			       OSPM_DISPLAY_B_ISLAND |
			       OSPM_DISPLAY_C_ISLAND | OSPM_MIPI_ISLAND);

		/*
		   If pmu_nc_set_power_state fails then accessing HW
		   reg would result in a crash - IERR/Fabric error.
		 */
		if (pmu_nc_set_power_state(dc_islands,
					   OSPM_ISLAND_DOWN, OSPM_REG_TYPE))
			BUG();
#ifdef CONFIG_MDFD_HDMI
		/* Turn off MSIC VCC330 and VHDMI if HDMI is disconnected. */
		if (!hdmi_state) {
			/* turn off HDMI power rails */
			intel_scu_ipc_iowrite8(MSIC_VHDMICNT, VHDMI_OFF);
			intel_scu_ipc_iowrite8(MSIC_VCC330CNT, VCC330_OFF);
		}
#endif
		g_hw_power_status_mask &= ~OSPM_DISPLAY_ISLAND;
		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_DISPLAY_ISLAND;
	}

	if (hw_islands & OSPM_VIDEO_VPP_ISLAND) {
		if (mrfld_set_power_state(
				OSPM_VIDEO_VPP_ISLAND,
				0,
				OSPM_ISLAND_DOWN))
			BUG();

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_VIDEO_VPP_ISLAND;
	}

	if (hw_islands & OSPM_VIDEO_DEC_ISLAND) {
		if (mrfld_set_power_state(
				OSPM_VIDEO_DEC_ISLAND,
				0,
				OSPM_ISLAND_DOWN))
			BUG();

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_VIDEO_DEC_ISLAND;
	}

	if (hw_islands & OSPM_VIDEO_ENC_ISLAND) {
		if (mrfld_set_power_state(
				OSPM_VIDEO_ENC_ISLAND,
				0,
				OSPM_ISLAND_DOWN))
			BUG();

		/* handle other islands */
		gfx_islands = hw_islands & ~OSPM_VIDEO_ENC_ISLAND;
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
		if (pmu_nc_set_power_state(gfx_islands,
					   OSPM_ISLAND_DOWN, APM_REG_TYPE))
			BUG();
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
	return ((g_hw_power_status_mask & hw_islands) ==
		hw_islands) ? true : false;
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
bool ospm_power_using_hw_begin(int hw_island, enum UHBUsage usage)
{
	bool ret = true;
	bool island_is_off = false;
	/* in_atomic() is forbid in checkpath.pl */
	/* bool b_atomic = (in_interrupt() || in_atomic());*/
	bool b_atomic = in_interrupt();
	bool locked = true;
	struct pci_dev *pdev = gpDrmDevice->pdev;
	bool force_on = usage ? true : false;

#ifdef CONFIG_GFX_RTPM
	if (gbSuspendInProgress) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT
		       "%s Suspend In Progress, call pm_runtime_get_noresume\n",
		       __func__);
#endif
		pm_runtime_get_noresume(&pdev->dev);
	} else {
		pm_runtime_get(&pdev->dev);
	}
#endif
	/* quick path, not 100% race safe, but should be enough
	 * comapre to current other code in this file */
	if (!force_on) {
		if (hw_island & (OSPM_ALL_ISLANDS & ~g_hw_power_status_mask)) {
#ifdef CONFIG_GFX_RTPM
			pm_runtime_put(&pdev->dev);
#endif
			return false;
		} else {
			locked = false;
			goto increase_count;
		}
	}

	if (!b_atomic)
		mutex_lock(&g_ospm_mutex);

	island_is_off =
	    hw_island & (OSPM_ALL_ISLANDS & ~g_hw_power_status_mask);

	if (b_atomic
	    && (gbSuspendInProgress || gbResumeInProgress || gbSuspended)
	    && force_on && island_is_off)
		ret = false;

	if (ret && island_is_off && !force_on)
		ret = false;

	if (ret && island_is_off && force_on) {
		gbResumeInProgress = true;

		ret = ospm_resume_pci(pdev);
		if (!ret) {
			printk(KERN_ALERT "%s: %d failed\n",
				__func__, hw_island);
			gbResumeInProgress = false;
			goto increase_count;
		}

		switch (hw_island) {
		case OSPM_DISPLAY_ISLAND:
			ospm_resume_display(pdev);
			psb_irq_preinstall_islands(gpDrmDevice,
						OSPM_DISPLAY_ISLAND);
			psb_irq_postinstall_islands(gpDrmDevice,
						OSPM_DISPLAY_ISLAND);
			break;
		case OSPM_GRAPHICS_ISLAND:
			ospm_power_island_up(OSPM_GRAPHICS_ISLAND);
			psb_irq_preinstall_islands(gpDrmDevice,
						OSPM_GRAPHICS_ISLAND);
			psb_irq_postinstall_islands(gpDrmDevice,
						OSPM_GRAPHICS_ISLAND);
			break;
		case OSPM_VIDEO_DEC_ISLAND:
			if (!ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
				ospm_resume_display(pdev);
				psb_irq_preinstall_islands(gpDrmDevice,
							OSPM_DISPLAY_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice,
							OSPM_DISPLAY_ISLAND);
			}

			if (!ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND)) {
				ospm_power_island_up(OSPM_VIDEO_DEC_ISLAND);
				ospm_runtime_pm_msvdx_resume(gpDrmDevice);
				psb_irq_preinstall_islands(gpDrmDevice,
							OSPM_VIDEO_DEC_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice,
							OSPM_VIDEO_DEC_ISLAND);
			}

			break;
		case OSPM_VIDEO_ENC_ISLAND:
			if (!ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
				ospm_resume_display(pdev);
				psb_irq_preinstall_islands(gpDrmDevice,
							OSPM_DISPLAY_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice,
							OSPM_DISPLAY_ISLAND);
			}

			if (!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND)) {
				ospm_power_island_up(OSPM_VIDEO_ENC_ISLAND);
				ospm_runtime_pm_topaz_resume(gpDrmDevice);
				psb_irq_preinstall_islands(gpDrmDevice,
							OSPM_VIDEO_ENC_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice,
							OSPM_VIDEO_ENC_ISLAND);
			}

			break;
		case OSPM_VIDEO_VPP_ISLAND:
			if (!ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND)) {
				ospm_resume_display(pdev);
				psb_irq_preinstall_islands(gpDrmDevice,
						OSPM_DISPLAY_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice,
						OSPM_DISPLAY_ISLAND);
			}

			if (!ospm_power_is_hw_on(OSPM_VIDEO_VPP_ISLAND)) {
				ospm_power_island_up(OSPM_VIDEO_VPP_ISLAND);
				ospm_runtime_pm_vsp_resume(gpDrmDevice);
				psb_irq_preinstall_islands(gpDrmDevice,
						OSPM_VIDEO_VPP_ISLAND);
				psb_irq_postinstall_islands(gpDrmDevice,
						OSPM_VIDEO_VPP_ISLAND);
			}

			break;

		default:
			printk(KERN_ALERT "%s unknown island !!!!\n",
			       __func__);
			break;
		}

		gbResumeInProgress = false;
	}
 increase_count:
	if (ret) {
		switch (hw_island) {
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
		case OSPM_VIDEO_VPP_ISLAND:
			atomic_inc(&g_videovsp_access_count);
			break;
		}

	}
#ifdef CONFIG_GFX_RTPM
	else
		pm_runtime_put(&pdev->dev);
#endif

	if (!b_atomic && locked)
		mutex_unlock(&g_ospm_mutex);

	return ret;
}

/*
 * ospm_power_using_hw_end
 *
 * Description: Notify PowerMgmt module that you are done accessing the
 * specified island's hw so feel free to power it off.  Note that this
 * function doesn't actually power off the islands.
 */
void ospm_power_using_hw_end(int hw_island)
{
	switch (hw_island) {
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
	case OSPM_VIDEO_VPP_ISLAND:
		atomic_dec(&g_videovsp_access_count);
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
	WARN_ON(atomic_read(&g_videovsp_access_count) < 0);
}

int ospm_runtime_pm_allow(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	bool panel_on, panel_on2;

	PSB_DEBUG_ENTRY("%s\n", __func__);

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
	if (dev_priv->rpm_enabled)
		return 0;

	if (is_panel_vid_or_cmd(dev)) {
		/*DPI panel */
		panel_on = dev_priv->dpi_panel_on;
		panel_on2 = dev_priv->dpi_panel_on2;
	} else {
		/*DBI panel */
		panel_on = dev_priv->dbi_panel_on;
		panel_on2 = dev_priv->dbi_panel_on2;
	}

#ifdef CONFIG_GFX_RTPM
	if (panel_on && panel_on2) {
		pm_runtime_allow(&dev->pdev->dev);
		dev_priv->rpm_enabled = 1;
		DRM_INFO("Runtime PM enabled\n");
	}
#endif

	return 0;
}

void ospm_runtime_pm_forbid(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

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
	if (atomic_read(&g_graphics_access_count)
	    || atomic_read(&g_videoenc_access_count)
	    || (gbdispstatus == true)
	    || atomic_read(&g_videovsp_access_count)
	    || atomic_read(&g_videodec_access_count)
	    || atomic_read(&g_display_access_count)) {
#ifdef OSPM_GFX_DPK
		printk(KERN_ALERT "GFX:%d VEC:%d VED:%d VSP:%d DC:%d DSR:%d\n",
			atomic_read(&g_graphics_access_count),
			atomic_read(&g_videoenc_access_count),
			atomic_read(&g_videodec_access_count),
			atomic_read(&g_videovsp_access_count),
			atomic_read(&g_display_access_count), gbdispstatus);
#endif
		return -EBUSY;
	} else
		ret = ospm_power_suspend(gpDrmDevice->pdev, state);

	return ret;
}

int psb_runtime_resume(struct device *dev)
{
	/* Notify HDMI Audio sub-system about the resume. */
#ifdef CONFIG_SND_INTELMID_HDMI_AUDIO
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;

	if (dev_priv->had_pvt_data)
		dev_priv->had_interface->resume(dev_priv->had_pvt_data);
#endif

#ifdef OSPM_GFX_DPK
	printk(KERN_ALERT "%s\n", __func__);
#endif
	/* Nop for GFX */
	return 0;
}

int psb_runtime_idle(struct device *dev)
{
#ifdef CONFIG_SND_INTELMID_HDMI_AUDIO
	struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
	int hdmi_audio_busy = 0;
	pm_event_t hdmi_audio_event;
#endif

#ifdef CONFIG_SND_INTELMID_HDMI_AUDIO
	if (dev_priv->had_pvt_data) {
		hdmi_audio_event.event = 0;
		hdmi_audio_busy =
		    dev_priv->had_interface->suspend(dev_priv->had_pvt_data,
						     hdmi_audio_event);
	}
#endif
	if (atomic_read(&g_graphics_access_count)
	    || atomic_read(&g_videoenc_access_count)
	    || atomic_read(&g_videodec_access_count)
	    || atomic_read(&g_display_access_count)
	    || atomic_read(&g_videovsp_access_count)
	    || (gbdispstatus == true)
#ifdef CONFIG_SND_INTELMID_HDMI_AUDIO
	    || hdmi_audio_busy
#endif
	)
		return -EBUSY;
	else
		return 0;
}
