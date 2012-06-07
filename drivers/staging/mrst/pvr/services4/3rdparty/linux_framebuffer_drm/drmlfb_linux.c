/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/version.h>

#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>

#include <drm/drmP.h>

#include <asm/io.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "pvrmodule.h"
#include "pvr_drm.h"
#include "drmlfb.h"


#include "psb_drv.h"

#if !defined(SUPPORT_DRI_DRM)
#error "SUPPORT_DRI_DRM must be set"
#endif

#define	MAKESTRING(x) # x

#if !defined(DISPLAY_CONTROLLER)
#define DISPLAY_CONTROLLER pvrlfb
#endif


#define unref__ __attribute__ ((unused))


extern int fb_idx;

void *MRSTLFBAllocKernelMem(unsigned long ulSize)
{
	return kmalloc(ulSize, GFP_KERNEL);
}

void MRSTLFBFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}


MRST_ERROR MRSTLFBGetLibFuncAddr (char *szFunctionName, PFN_DC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetDisplayClassJTable", szFunctionName) != 0)
	{
		return (MRST_ERROR_INVALID_PARAMS);
	}

	
	*ppfnFuncTable = PVRGetDisplayClassJTable;

	return (MRST_OK);
}

static void MRSTLFBVSyncWriteReg(MRSTLFB_DEVINFO *psDevInfo, unsigned long ulOffset, unsigned long ulValue)
{

	void *pvRegAddr = (void *)(psDevInfo->pvRegs + ulOffset);
	mb();
	iowrite32(ulValue, pvRegAddr);
}

void MRSTLFBEnableVSyncInterrupt(MRSTLFB_DEVINFO * psDevinfo)
{
#if defined(MRST_USING_INTERRUPTS)	
	if( drm_vblank_get( psDevinfo->psDrmDevice , psDevinfo->ui32MainPipe ) )
	{
		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX	"Couldn't get vsync enabled"));
	}
#endif
}

void MRSTLFBDisableVSyncInterrupt(MRSTLFB_DEVINFO * psDevinfo)
{
#if defined(MRST_USING_INTERRUPTS)	
	drm_vblank_put( psDevinfo->psDrmDevice,  psDevinfo->ui32MainPipe );    
#endif
}

#if defined(MRST_USING_INTERRUPTS)
MRST_ERROR MRSTLFBInstallVSyncISR(MRSTLFB_DEVINFO *psDevInfo, MRSTLFB_VSYNC_ISR_PFN pVsyncHandler)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *) psDevInfo->psDrmDevice->dev_private;
	dev_priv->psb_vsync_handler = pVsyncHandler;
	return (MRST_OK);
}


MRST_ERROR MRSTLFBUninstallVSyncISR(MRSTLFB_DEVINFO	*psDevInfo)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *) psDevInfo->psDrmDevice->dev_private;
	dev_priv->psb_vsync_handler = NULL;
	return (MRST_OK);
}
#endif 


void MRSTLFBFlipToSurface(MRSTLFB_DEVINFO *psDevInfo,  unsigned long uiAddr)
{
    int dspbase = (psDevInfo->ui32MainPipe == 0 ? DSPABASE : DSPBBASE);
    int dspsurf = (psDevInfo->ui32MainPipe == 0 ? DSPASURF : DSPBSURF);
    struct drm_device * dev = psDevInfo->psDrmDevice;
    struct drm_psb_private *dev_priv =
        (struct drm_psb_private *) psDevInfo->psDrmDevice->dev_private;
    MRSTLFB_SWAPCHAIN *psCurrentSwapChain = psDevInfo->psCurrentSwapChain;

    if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, false))
    {   
        if (IS_MRST(dev)) {
            MRSTLFBVSyncWriteReg(psDevInfo, dspsurf, uiAddr);
        } else if (IS_MDFLD(dev)) {
		if (!dev_priv->um_start) {
			dev_priv->um_start = true;
			if (dev_priv->b_dsr_enable_config)
				dev_priv->b_dsr_enable = true;
		}
		if (psCurrentSwapChain != NULL) {
			if (psCurrentSwapChain->ui32SwapChainPropertyFlag
					& PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A) {
				dspsurf = DSPASURF;
				MRSTLFBVSyncWriteReg(psDevInfo, dspsurf, uiAddr);
			}
#if defined(CONFIG_MDFD_DUAL_MIPI)
			if (psCurrentSwapChain->ui32SwapChainPropertyFlag
					& PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C) {
				dspsurf = DSPCSURF;
				MRSTLFBVSyncWriteReg(psDevInfo, dspsurf, uiAddr);
			}
#endif
#ifdef CONFIG_MDFD_HDMI
			/* To avoid Plane B still fetches data from original frame
			 * buffer. */
			if (psCurrentSwapChain->ui32SwapChainPropertyFlag
					& PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B) {
				dspsurf = DSPBSURF;
				MRSTLFBVSyncWriteReg(psDevInfo, dspsurf, uiAddr);
			}
#endif
		} else {
			printk(KERN_WARNING "Current Swapchain is null, no attached plane info\
				 available, omit address update\n");
			if (drm_psb_debug & PSB_D_GENERAL)
				dump_stack();
		}
        } else {
            MRSTLFBVSyncWriteReg(psDevInfo, dspbase, uiAddr);
        }   
        ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
    }   
}

int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Init)(struct drm_device unref__ *dev)
{
	if(MRSTLFBInit(dev) != MRST_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": %s: MRSTLFBInit failed\n", __FUNCTION__);
		return -ENODEV;
	}

	return 0;
}

void PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Cleanup)(struct drm_device unref__ *dev)
{    
	if(MRSTLFBDeinit() != MRST_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": %s: can't deinit device\n", __FUNCTION__);
	}
}

int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Suspend)(struct drm_device unref__ *dev)
{
	MRSTLFBSuspend();

	return 0;
}

int PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Resume)(struct drm_device unref__ *dev)
{
	MRSTLFBResume();

	return 0;
}

