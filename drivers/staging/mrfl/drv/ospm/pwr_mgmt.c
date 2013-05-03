/**************************************************************************
 * Copyright (c) 2012, Intel Corporation.
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
 *    Hitesh K. Patel <hitesh.k.patel@intel.com>
 */


#include <linux/earlysuspend.h>
#include <linux/spinlock.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>

#ifdef CONFIG_GFX_RTPM
#include <linux/pm_runtime.h>
#endif

#include "psb_drv.h"
#include "pmu_tng.h"
#include "tng_wa.h"
#include "pwr_mgmt.h"
#include "gfx_rtpm.h"
#include "gfx_ospm.h"
#include "dc_ospm.h"
#include "video_ospm.h"
#include "early_suspend.h"


static struct _ospm_data_ *g_ospm_data;
struct drm_device *gpDrmDevice;

/* island, state, ref_count, init_func, power_func */
static struct ospm_power_island island_list[] = {
	{OSPM_DISPLAY_A, OSPM_POWER_OFF, {0}, ospm_disp_a_init, NULL},
	{OSPM_DISPLAY_B, OSPM_POWER_OFF, {0}, ospm_disp_b_init, NULL},
	{OSPM_DISPLAY_C, OSPM_POWER_OFF, {0}, ospm_disp_c_init, NULL},
	{OSPM_DISPLAY_MIO, OSPM_POWER_OFF, {0}, ospm_mio_init, NULL},
	{OSPM_DISPLAY_HDMI, OSPM_POWER_OFF, {0}, ospm_hdmi_init, NULL},
	{OSPM_GRAPHICS_ISLAND, OSPM_POWER_OFF, {0}, ospm_gfx_init, NULL},
	{OSPM_VIDEO_VPP_ISLAND, OSPM_POWER_OFF, {0}, ospm_vsp_init, NULL},
	{OSPM_VIDEO_DEC_ISLAND, OSPM_POWER_OFF, {0}, ospm_ved_init, NULL},
	{OSPM_VIDEO_ENC_ISLAND, OSPM_POWER_OFF, {0}, ospm_vec_init, NULL},
};

/**
 * in_atomic_or_interrupt() - Return non-zero if in atomic context.
 * Problems with this code:
 * - Function in_atomic is not guaranteed to detect the atomic state entered
 *   by acquisition of a spinlock (and indeed does so only if CONFIG_PREEMPT).
 *   For a discussion on the use of in_atomic and why is it considered (in
 *   general) problematic, see: http://lwn.net/Articles/274695/
 * - Therefore, scripts/checkpatch.pl will complain about use of function
 *   in_atomic in non-core kernel.  For this reason, the several uses of
 *   in_atomic in this file were centralized here (so only one warning).
 *
 * Note: The test herein was originally:
 *   in_atomic() || in_interrupt()
 * but the test for in_interrupt() is redundant with the in_atomic test.
 */
#if !defined CONFIG_PREEMPT
#error Function in_atomic (in general) requires CONFIG_PREEMPT
#endif

#ifdef OSPM_DEBUG_INFO
const char *get_island_name(u32 hw_island)
{
	const char *pstr;

	switch (hw_island) {
	case OSPM_DISPLAY_A:
		pstr = "DISP A ";
		break;
	case OSPM_DISPLAY_B:
		pstr = "DISP B ";
		break;
	case OSPM_DISPLAY_C:
		pstr = "DISP C ";
		break;
	case OSPM_DISPLAY_MIO:
		pstr = "MIO    ";
		break;
	case OSPM_DISPLAY_HDMI:
		pstr = "HDMI   ";
		break;
	case OSPM_VIDEO_VPP_ISLAND:
		pstr = "VSP    ";
		break;
	case OSPM_VIDEO_DEC_ISLAND:
		pstr = "VED    ";
		break;
	case OSPM_VIDEO_ENC_ISLAND:
		pstr = "VEC    ";
		break;
	case OSPM_GRAPHICS_ISLAND:
		pstr = "GFX    ";
		break;
	default:
		pstr = "(unknown hw_island)";
		break;
	}

	return pstr;
}

static void dump_ref_count(u32 hw_island)
{
	int i = 0;
	int ref_value = 0;
	struct ospm_power_island *p_island = NULL;

	OSPM_DPF("*** power island refrence count. ***\n");

	for (i = 0; i < ARRAY_SIZE(island_list); i++) {
		if (hw_island & island_list[i].island) {
			p_island = &island_list[i];
			ref_value = atomic_read(&p_island->ref_count);
			printk(KERN_ALERT
				"*** %s: %d\n",
				get_island_name(island_list[i].island),
				ref_value);
		}
	}

	OSPM_DPF("%s: ************************************\n");
}
#endif	/* OSPM_DEBUG_INFO */

/**
 * ospm_suspend_pci
 *
 * Description: Suspend the pci device saving state and disabling
 * as necessary.
 */
static void ospm_suspend_pci(struct drm_device *dev)
{
	struct pci_dev *pdev = dev->pdev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int bsm, vbt, bgsm;

	OSPM_DPF("%s\n", __func__);

	pci_save_state(pdev);
	pci_read_config_dword(pdev, 0x5C, &bsm);
	dev_priv->saveBSM = bsm;
	pci_read_config_dword(pdev, 0xFC, &vbt);
	dev_priv->saveVBT = vbt;
	pci_read_config_dword(pdev, 0x70, &bgsm);
	dev_priv->saveBGSM = bgsm;
	pci_read_config_dword(pdev, PSB_PCIx_MSI_ADDR_LOC, &dev_priv->msi_addr);
	pci_read_config_dword(pdev, PSB_PCIx_MSI_DATA_LOC, &dev_priv->msi_data);

	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
}

/**
 * ospm_resume_pci
 *
 * Description: Resume the pci device restoring state and enabling
 * as necessary.
 */
static void ospm_resume_pci(struct drm_device *dev)
{
	struct pci_dev *pdev = dev->pdev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int ret = 0;

	OSPM_DPF("%s\n", __func__);

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	pci_write_config_dword(pdev, 0x70, dev_priv->saveBGSM);
	pci_write_config_dword(pdev, 0x5c, dev_priv->saveBSM);
	pci_write_config_dword(pdev, 0xFC, dev_priv->saveVBT);

	/* retoring MSI address and data in PCIx space */
	pci_write_config_dword(pdev, PSB_PCIx_MSI_ADDR_LOC, dev_priv->msi_addr);
	pci_write_config_dword(pdev, PSB_PCIx_MSI_DATA_LOC, dev_priv->msi_data);
	ret = pci_enable_device(pdev);

	if (ret != 0)
		OSPM_DPF("pci_enable_device failed: %d\n", ret);
}

/**
 * get_island_ptr
 *
 * get pointer to the island
 * use it to get array item for setting dependency
 *
 * Although island values are defined as bit mask values,
 * this function only supports having a single bit set
 * in this parameter.
 */
struct ospm_power_island *get_island_ptr(u32 hw_island)
{
	struct ospm_power_island *p_island = NULL;
	int i = 0;

	/* got through islands array to find the island */
	while ((i < ARRAY_SIZE(island_list) && (!p_island))) {
		/* do we have the island? */
		if (hw_island & island_list[i].island) {
			/* Found it */
			p_island = &island_list[i];
			break;
		}

		i++;
	}

	if (i == ARRAY_SIZE(island_list))
		OSPM_DPF("island %x not found\n", hw_island);

	return p_island;
}

/**
 * power_up_island
 *
 * Description: Power up the island and all of it's dependent islands
 */
static bool power_up_island(struct ospm_power_island *p_island)
{
	bool ret = true;

	/* handle the dependency first */
	if (p_island->p_dependency) {
		/* Power up dependent island */
		ret = power_up_island(p_island->p_dependency);
		if (!ret)
			return ret;
	}

	/* if successfully handled dependency */
	if (!atomic_read(&p_island->ref_count)) {
		/* power on the island */
		ret = p_island->p_funcs->power_up(g_ospm_data->dev, p_island);
		if (ret) {
			p_island->island_state = OSPM_POWER_ON;

			/*
			 * FIXME: revisit to check whether the 1st level
			 * interrupts of VED/VEC/VSP need to be turned on here.
			 */
			/* Video irq need to be set */
			if (p_island->island & OSPM_VIDEO_ISLAND) {
				psb_irq_preinstall_islands(
						g_ospm_data->dev,
						p_island->island);
				psb_irq_postinstall_islands(
						g_ospm_data->dev,
						p_island->island);
			}
		} else
			return ret;
	}

	/* increment the ref count */
	atomic_inc(&p_island->ref_count);

	return ret;
}

/**
 * power_down_island
 *
 * Description: Power down the island and all of it's dependent islands
 */
static bool power_down_island(struct ospm_power_island *p_island)
{
	bool ret = true;

	if (atomic_dec_return(&p_island->ref_count) < 0) {
		OSPM_DPF("Island %x, UnExpect RefCount %d\n",
				p_island->island,
				p_island->ref_count);
		goto power_down_err;
	}

	/* check to see if island is turned off */
	if (!atomic_read(&p_island->ref_count)) {
		/* power on the island */
		ret = p_island->p_funcs->power_down(
				g_ospm_data->dev,
				p_island);

		/* set the island state */
		if (ret)
			p_island->island_state = OSPM_POWER_OFF;
		else
			goto power_down_err;
	}

	/* handle the dependency later */
	if (p_island->p_dependency) {
		/* Power down dependent island */
		ret = power_down_island(p_island->p_dependency);
		if (!ret)
			goto power_down_err;
	}

	return ret;

power_down_err:
	atomic_inc(&p_island->ref_count);
	ret = false;
	return ret;
}

/**
 * power_island_get
 *
 * Description: Notify PowerMgmt module that you will be accessing the
 * specified island's hw so don't power it off.  If the island is not
 * powered up, it will power it on.
 *
 */
bool power_island_get(u32 hw_island)
{
	u32 i = 0;
	bool ret = true;
	struct ospm_power_island *p_island;
	struct drm_psb_private *dev_priv = g_ospm_data->dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);

	for (i = 0; i < ARRAY_SIZE(island_list); i++) {
		if (hw_island & island_list[i].island) {
			p_island = &island_list[i];
			ret = power_up_island(p_island);
			if (!ret) {
				OSPM_DPF("power up failed %x\n",
					island_list[i].island);
				goto out_err;
			}
		}
	}

out_err:
	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);

	return ret;
}

/**
 * power_island_put
 *
 * Description: Notify PowerMgmt module that you are done accessing the
 * specified island's hw so feel free to power it off.  Note that this
 * function doesn't actually power off the islands.
 */
bool power_island_put(u32 hw_island)
{
	bool ret = true;
	u32 i = 0;
	u32 ref_count = 0;
	struct ospm_power_island *p_island;
	struct drm_psb_private *dev_priv = g_ospm_data->dev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);

	for (i = 0; i < ARRAY_SIZE(island_list); i++) {
		if (hw_island & island_list[i].island) {
			/* Power down the island if needed */
			p_island = &island_list[i];
			ret = power_down_island(p_island);
			if (!ret) {
				OSPM_DPF("power down failed %x\n",
					island_list[i].island);
				goto out_err;
			}
		}
	}

out_err:
	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);

	return ret;
}

/**
 * is_island_on
 *
 * Description: checks to see if the island is up
 * returns true if hw_island is ON
 * returns false if hw_island is OFF
 */
bool is_island_on(u32 hw_island)
{
	/* get the power island */
	struct ospm_power_island *p_island = get_island_ptr(hw_island);
	unsigned long flags;
	bool island_on = false;

	/* TODO: add lock here. */
	island_on = (p_island->island_state == OSPM_POWER_ON) ? true : false;

	return island_on;
}

u32 pipe_to_island(u32 pipe)
{
	u32 power_island = 0;

	switch (pipe) {
	case 0:
		power_island = OSPM_DISPLAY_A;
		break;
	case 1:
		power_island = OSPM_DISPLAY_B;
		break;
	case 2:
		power_island = OSPM_DISPLAY_C;
		break;
	default:
		DRM_ERROR("%s: invalid pipe %u\n", __func__, pipe);
		return 0;
	}

	return power_island;
}

/**
 * ospm_power_init
 *
 * Description: Initialize this ospm power management module
 */
void ospm_power_init(struct drm_device *dev)
{
	u32 i = 0;
	u32 nc_pwr_sts;

	/* allocate ospm data */
	g_ospm_data = kmalloc(sizeof(struct _ospm_data_), GFP_KERNEL);
	if (!g_ospm_data)
		goto out_err;

	spin_lock_init(&g_ospm_data->ospm_lock);
	g_ospm_data->dev = dev;
	gpDrmDevice = dev;

	/* initilize individual islands */
	for (i = 0; i < ARRAY_SIZE(island_list); i++) {
		island_list[i].p_funcs = kmalloc(sizeof(struct power_ops),
						GFP_KERNEL);
		if ((island_list[i].p_funcs) && (island_list[i].init_func)) {
			island_list[i].init_func(dev, &island_list[i]);
			atomic_set(&island_list[i].ref_count, 0);
		}
	}

	/* register early_suspend runtime pm */
	intel_media_early_suspend_init(dev);

out_err:
	return;
}

/**
 * ospm_power_uninit
 *
 * Description: Uninitialize this ospm power management module
 */
void ospm_power_uninit(void)
{
	int i;
	OSPM_DPF("%s\n", __func__);

	/* un-init early suspend */
	intel_media_early_suspend_uninit();

	/* Do we need to turn off all islands? */
	power_island_put(OSPM_ALL_ISLANDS);

	for (i = 0; i < ARRAY_SIZE(island_list); i++)
		kfree(island_list[i].p_funcs);

	kfree(g_ospm_data);
}

/**
 * ospm_power_suspend
 *
 * Description: suspend all islands
 */
bool ospm_power_suspend(void)
{
	int i;
	struct ospm_power_island *p_island = NULL;
	unsigned long flags;

	OSPM_DPF("%s\n", __func__);

	/* Asking RGX to power off */
	if (!PVRSRVRGXSetPowerState(g_ospm_data->dev, OSPM_POWER_OFF))
		return false;

	/* FIXME: try to turn off PCI in power_island_put(). */
	ospm_suspend_pci(g_ospm_data->dev);

	return true;
}

/**
 * ospm_power_resume
 *
 * Description: resume previously suspended islands.
 */
void ospm_power_resume(void)
{
	int i;
	struct ospm_power_island *p_island = NULL;
	unsigned long flags;

	OSPM_DPF("%s\n", __func__);

	ospm_resume_pci(g_ospm_data->dev);

	OSPM_DPF("pci resumed.\n");

	/* restore Graphics State */
	PVRSRVRGXSetPowerState(g_ospm_data->dev, OSPM_POWER_ON);
	OSPM_DPF("Graphics state restored.\n");

	OSPM_DPF("display resumed.\n");
}

/* FIXME: hkpatel */
/*** LEGACY SUPPORT ****/
/*** REMOVE ONCE CONVERTED ALL FUNCTIONS TO NEW ARCH */

/* Legacy Function for support */
bool ospm_power_using_hw_begin(int hw_island, u32 usage)
{
	bool ret = true;

	/*
	 * FIXME: make ospm_power_using_hw_begin used for Display islands only
	 * take effect for DSPB/HDMIO islands, becaused it's called by the OTM
	 * HDMI codes and not to impact CTP/MDFLD. But eventually need to
	 * replace hw_begin() with power_island_get() in OTM HDMI.
	 */
	if (hw_island == OSPM_DISPLAY_ISLAND)
		hw_island = OSPM_DISPLAY_B | OSPM_DISPLAY_HDMI;

	ret = power_island_get(hw_island);

	return ret;
}

bool ospm_power_is_hw_on(u32 hw_island)
{
	return is_island_on(hw_island);
}

void ospm_power_using_hw_end(int hw_island)
{
	/*
	 * FIXME: make ospm_power_using_hw_end used for Display islands only
	 * take effect for DSPB/HDMIO islands, becaused it's called by the OTM
	 * HDMI codes and not to impact CTP/MDFLD. But eventually need to
	 * replace hw_end() with power_island_put() in OTM HDMI.
	 */
	if (hw_island == OSPM_DISPLAY_ISLAND)
		hw_island = OSPM_DISPLAY_B | OSPM_DISPLAY_HDMI;

	power_island_put(hw_island);
}

void ospm_power_using_video_end(int hw_island)
{
	ospm_power_using_hw_end(hw_island);
}

bool ospm_power_using_video_begin(int hw_island)
{
	ospm_power_using_hw_begin(hw_island, 0);
}

void ospm_apm_power_down_msvdx(struct drm_device *dev, int force_off)
{
	struct ospm_power_island *p_island;
	int ret;
	unsigned long flags;
	PSB_DEBUG_PM("MSVDX: work queue is scheduled to power off msvdx.\n");
	p_island = get_island_ptr(OSPM_VIDEO_DEC_ISLAND);

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);

	if (force_off)
		goto power_off;
	if (!ospm_power_is_hw_on(OSPM_VIDEO_DEC_ISLAND)) {
		PSB_DEBUG_PM("g_hw_power_status_mask: msvdx in power off.\n");
		goto out;
	}

	if (atomic_read(&p_island->ref_count)) {
		PSB_DEBUG_PM("ved ref_count has been set.\n");
		goto out;
	}

power_off:
	ret = p_island->p_funcs->power_down(
			g_ospm_data->dev,
			p_island);

	/* set the island state */
	if (ret)
		p_island->island_state = OSPM_POWER_OFF;

	/* MSVDX_NEW_PMSTATE(dev, msvdx_priv, PSB_PMSTATE_POWERDOWN); */
out:
	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);
	return;
}
void ospm_apm_power_down_topaz(struct drm_device *dev)
{
	int ret;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct ospm_power_island *p_island;
	unsigned long flags;

	PSB_DEBUG_PM("Power down VEC...\n");
	p_island = get_island_ptr(OSPM_VIDEO_ENC_ISLAND);

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);

	if (!ospm_power_is_hw_on(OSPM_VIDEO_ENC_ISLAND))
		goto out;

	if (atomic_read(&p_island->ref_count)) {
		PSB_DEBUG_PM("vec ref_count has been set(%d), bypass\n",
			     atomic_read(&p_island->ref_count));
		goto out;
	}

power_off:
	ret = p_island->p_funcs->power_down(
			g_ospm_data->dev,
			p_island);

	/* set the island state */
	if (ret)
		p_island->island_state = OSPM_POWER_OFF;

	PSB_DEBUG_PM("Power down VEC done\n");
out:
	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);
	return;
}

int ospm_apm_power_down_vsp(struct drm_device *dev)
{
	struct ospm_power_island *vsp_island;
	int ret = 0;
	bool pm_ret;
	unsigned long flags;

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);

	vsp_island = get_island_ptr(OSPM_VIDEO_VPP_ISLAND);
	if (!vsp_island) {
		PSB_DEBUG_PM("Couldn't get VSP island!\n");
		goto out;
	}

	if (!ospm_power_is_hw_on(OSPM_VIDEO_VPP_ISLAND)) {
		PSB_DEBUG_PM("VSP have been power off!\n");
		goto out;
	}

	if (atomic_read(&vsp_island->ref_count)) {
		PSB_DEBUG_PM("The VSP ref_count is NOT 0\n");
		ret = -EBUSY;
		goto out;
	}

	pm_ret = vsp_island->p_funcs->power_down(
				g_ospm_data->dev,
				vsp_island);
	if (pm_ret == false) {
		PSB_DEBUG_PM("Power OFF VSP island failed!\n");
		ret = -EBUSY;
		goto out;
	}

	vsp_island->island_state = OSPM_POWER_OFF;
	PSB_DEBUG_PM("VSP island is powered off!\n");

out:
	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);
	return ret;
}

int ospm_runtime_pm_allow(struct drm_device *dev)
{
	return 0;
}

void ospm_runtime_pm_forbid(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

#ifdef CONFIG_GFX_RTPM
	pm_runtime_forbid(&dev->pdev->dev);
#endif
	dev_priv->rpm_enabled = 0;
}
