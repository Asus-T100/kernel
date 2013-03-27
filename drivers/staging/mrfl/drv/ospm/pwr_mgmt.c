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
#include <linux/mutex.h>
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
	int bsm, vbt;

	OSPM_DPF("%s\n", __func__);

	if (g_ospm_data->b_suspended) {
		/* Warning to catch logic error */
		OSPM_DPF("ospm_suspend_pci: already suspended.\n");
		return;
	}

	pci_save_state(pdev);
	pci_read_config_dword(pdev, 0x5C, &bsm);
	dev_priv->saveBSM = bsm;
	pci_read_config_dword(pdev, 0xFC, &vbt);
	dev_priv->saveVBT = vbt;
	pci_read_config_dword(pdev, PSB_PCIx_MSI_ADDR_LOC, &dev_priv->msi_addr);
	pci_read_config_dword(pdev, PSB_PCIx_MSI_DATA_LOC, &dev_priv->msi_data);

	pci_disable_device(pdev);
	/* FIXME: vcheeram : Check we can suspend and resume PCI*/
	/*pci_set_power_state(pdev, PCI_D3hot);*/

	g_ospm_data->b_suspended = true;
}

/**
 * ospm_resume_pci
 *
 * Description: Resume the pci device restoring state and enabling
 * as necessary.
 */
static bool ospm_resume_pci(struct drm_device *dev)
{
	struct pci_dev *pdev = dev->pdev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	int ret = 0;

	OSPM_DPF("%s\n", __func__);

	if (!g_ospm_data->b_suspended) {
		/* Warning to catch logic error */
		OSPM_DPF("not suspended.\n");
		return true;
	}

	/* FIXME: vcheeram : Check we can suspend and resume PCI*/
	/*pci_set_power_state(pdev, PCI_D0);*/
	pci_restore_state(pdev);
	pci_write_config_dword(pdev, 0x5c, dev_priv->saveBSM);
	pci_write_config_dword(pdev, 0xFC, dev_priv->saveVBT);
	/* retoring MSI address and data in PCIx space */
	pci_write_config_dword(pdev, PSB_PCIx_MSI_ADDR_LOC, dev_priv->msi_addr);
	pci_write_config_dword(pdev, PSB_PCIx_MSI_DATA_LOC, dev_priv->msi_data);
	ret = pci_enable_device(pdev);

	if (ret != 0)
		OSPM_DPF("pci_enable_device failed: %d\n", ret);
	else
		g_ospm_data->b_suspended = false;

	return !g_ospm_data->b_suspended;
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
		if (p_island->p_dependency->island_state == OSPM_POWER_OFF) {
			/* Power up dependent island */
			ret = power_up_island(p_island->p_dependency);
			atomic_inc(&p_island->p_dependency->ref_count);
		}
	}

	/* if successfully handled dependency */
	if (ret) {
		if (!atomic_read(&p_island->ref_count)) {
			/* power on the island */
			ret = p_island->p_funcs->power_up(
						g_ospm_data->dev,
						p_island);
			if (ret) {
				p_island->island_state = OSPM_POWER_ON;
				/* Video irq need to be set */
				if (p_island->island & OSPM_VIDEO_ISLAND) {
					psb_irq_preinstall_islands(
							g_ospm_data->dev,
							p_island->island);
					psb_irq_postinstall_islands(
							g_ospm_data->dev,
							p_island->island);
				}
			}
		}

		/* increment the ref count */
		atomic_inc(&p_island->ref_count);
	}

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

	/* handle the dependency first */
	if (p_island->p_dependency) {
		if (p_island->p_dependency->island_state == OSPM_POWER_ON) {
			/* decrement the ref count */
			atomic_dec(&p_island->p_dependency->ref_count);
			/* Power down dependent island */
			ret = power_down_island(p_island->p_dependency);
		}
	}

	/* if successfully handled dependency */
	if (ret) {
		/* decrement the ref count */
		if (atomic_dec_return(&p_island->ref_count) < 0) {
			OSPM_DPF("Island %x, UnExpect RefCount %d\n",
				p_island->island,
				p_island->ref_count);
			dump_stack();
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
		}

		/*WARN_ON(atomic_read(&g_graphics_access_count) < 0);*/
	}

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

	if (dev_priv->early_suspended) {
		OSPM_DPF("power_island_get: System suspended.\n");
		return false;
	}

	/* mutex lock */
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
	/* mutex unlock */
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

	if (dev_priv->early_suspended) {
		OSPM_DPF("power_island_put: System suspended.\n");
		return false;
	}

	/* mutex lock */
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
	/* mutex unlock */
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
	return (p_island->island_state == OSPM_POWER_ON);
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
	g_ospm_data->b_suspended = false;
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

	/* get the current power state of the island */
	nc_pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS);

	OSPM_DPF("default island state = 0x%08lX\n", nc_pwr_sts);

	nc_pwr_sts = ~nc_pwr_sts;

	/* get the islands that are powered on by default */
	/* this are powered on at boot by FW */
	for (i = 0; i < ARRAY_SIZE(island_list); i++) {
		if (nc_pwr_sts & island_list[i].island) {
			OSPM_DPF("Island powered by FW : %x\n",
				island_list[i].island);
			power_island_get(island_list[i].island);
		}
	}

	/* register early_suspend runtime pm */
	intel_media_early_suspend_init(dev);

out_err:
	return;
}

/**
* ospm_post_init
*
* Description: Power gate unused GFX & Display islands.
*/
void ospm_post_init(struct drm_device *dev)
{
	u32 nc_pwr_sts;
	u32 dc_islands = 0;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *)dev->dev_private;

	if (dev_priv->panel_desc & DISPLAY_A)
		dc_islands |= OSPM_DISPLAY_A;

	if (dev_priv->panel_desc & DISPLAY_B) {
		dc_islands |= OSPM_DISPLAY_B;
		dc_islands |= OSPM_DISPLAY_HDMI;
	}

	if (dev_priv->panel_desc & DISPLAY_C)
		dc_islands |= OSPM_DISPLAY_C;

	if (dev_priv->panel_desc)
		dc_islands |= OSPM_DISPLAY_MIO;

	/* dc_islands now contains islands that needs to be enabled */
	/* disable islands that were enabled by FW at boot */
	OSPM_DPF("DC island to be turned ON : %x\n",
			dc_islands & OSPM_DISPLAY_ISLAND);
	dc_islands = (~dc_islands) & OSPM_DISPLAY_ISLAND;
	OSPM_DPF("DC island to be turned off : %x\n", dc_islands);
	/* get the current power state of the island */
	nc_pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, NC_PM_SSS);
	/* turn off the islands that are turned on by firmware but the driver
	 * decides that it does not need */
	power_island_put(dc_islands & (~nc_pwr_sts));

	OSPM_DPF("nc island state = 0x%08lX\n", nc_pwr_sts);
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

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);

	/* power down all individual islands */
	for (i = 0; i < ARRAY_SIZE(island_list); i++) {
		p_island = &island_list[i];

		if (p_island->island & OSPM_DISPLAY_ISLAND) {
			if (atomic_read(&p_island->ref_count)) {
				p_island->p_funcs->power_down(
						g_ospm_data->dev,
						p_island);
			}
		}
	}

	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);

	/* Save Graphics State */
	if (!PVRSRVRGXSetPowerState(g_ospm_data->dev, OSPM_POWER_OFF))
		return false;

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);
	ospm_suspend_pci(g_ospm_data->dev);
	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);

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

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);
	ospm_resume_pci(g_ospm_data->dev);
	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);

	OSPM_DPF("pci resumed.\n");

	/* restore Graphics State */
	PVRSRVRGXSetPowerState(g_ospm_data->dev, OSPM_POWER_ON);
	OSPM_DPF("Graphics state restored.\n");

	spin_lock_irqsave(&g_ospm_data->ospm_lock, flags);

	/* power down all individual islands */
	for (i = 0; i < ARRAY_SIZE(island_list); i++) {
		p_island = &island_list[i];

		if (p_island->island & OSPM_DISPLAY_ISLAND) {
			if (atomic_read(&p_island->ref_count)) {
				p_island->p_funcs->power_up(
							g_ospm_data->dev,
							p_island);

				psb_irq_preinstall_islands(g_ospm_data->dev,
							p_island->island);
				psb_irq_postinstall_islands(g_ospm_data->dev,
							p_island->island);
			}
		}
	}

	spin_unlock_irqrestore(&g_ospm_data->ospm_lock, flags);

	OSPM_DPF("display resumed.\n");
}

/* FIXME: hkpatel */
/*** LEGACY SUPPORT ****/
/*** REMOVE ONCE CONVERTED ALL FUNCTIONS TO NEW ARCH */

/* Legacy Function for support */
bool ospm_power_using_hw_begin(int hw_island, u32 usage)
{
	bool ret = true;

	ret = power_island_get(hw_island);

	return ret;
}
bool ospm_power_is_hw_on(u32 hw_island)
{
	return is_island_on(hw_island);
}

void ospm_power_using_hw_end(int hw_island)
{
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

void ospm_apm_power_down_msvdx(struct drm_device *dev, bool on)
{
}
void ospm_apm_power_down_topaz(struct drm_device *dev)
{
}
int ospm_apm_power_down_vsp(struct drm_device *dev)
{
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
