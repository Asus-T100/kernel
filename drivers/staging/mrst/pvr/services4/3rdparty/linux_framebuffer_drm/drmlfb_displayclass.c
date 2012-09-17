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

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/fb.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "drmlfb.h"

#include "psb_drv.h"
#include "psb_fb.h"

#include "mdfld_dsi_dbi_dsr.h"

#include <portdefs.h>

#if !defined(SUPPORT_DRI_DRM)
#error "SUPPORT_DRI_DRM must be set"
#endif

static void *gpvAnchor;
extern int drm_psb_3D_vblank;

#define MRSTLFB_COMMAND_COUNT		1

#define FLIP_TIMEOUT (HZ/4)

static PFN_DC_GET_PVRJTABLE pfnGetPVRJTable = 0;
static int FirstCleanFlag = 1;

static MRSTLFB_DEVINFO * GetAnchorPtr(void)
{
	return (MRSTLFB_DEVINFO *)gpvAnchor;
}

static void SetAnchorPtr(MRSTLFB_DEVINFO *psDevInfo)
{
	gpvAnchor = (void*)psDevInfo;
}

static IMG_BOOL MRSTLFBFlip(MRSTLFB_DEVINFO *psDevInfo,
				 MRSTLFB_BUFFER *psBuffer)
{
	unsigned long ulAddr = (unsigned long)psBuffer->sDevVAddr.uiAddr;
	struct fb_info *psLINFBInfo;

	if (!psDevInfo->bSuspended && !psDevInfo->bLeaveVT)
	{
		if (MRSTLFBFlipToSurface(psDevInfo, ulAddr) == IMG_FALSE) {
			DRM_INFO("%s: returning false\n", __func__);
			return IMG_FALSE;
		}
	}

	psDevInfo->ulLastFlipAddr = ulAddr;
	psDevInfo->bLastFlipAddrValid = MRST_TRUE;

	psLINFBInfo = psDevInfo->psLINFBInfo;
	psLINFBInfo->screen_base = psBuffer->sCPUVAddr;

	if (FirstCleanFlag == 1) {
		memset(psDevInfo->sSystemBuffer.sCPUVAddr, 0,
				psDevInfo->sSystemBuffer.ui32BufferSize);
		FirstCleanFlag = 0;
	}
	return IMG_TRUE;
}

static inline void MRSTFBFlipComplete(MRSTLFB_SWAPCHAIN *psSwapChain, MRSTLFB_VSYNC_FLIP_ITEM* psFlipItem, MRST_BOOL bSchedule)
{
	MRSTLFB_VSYNC_FLIP_ITEM *psLastItem;
	SYS_DATA				*psSysData;
	MRST_BOOL bMISRScheduled = MRST_FALSE;
	SysAcquireData(&psSysData);
	if (psSwapChain) {
		psLastItem = &(psSwapChain->sLastItem);
		if (psLastItem->bValid && psLastItem->bFlipped && psLastItem->bCmdCompleted == MRST_FALSE)
		{
			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete((IMG_HANDLE)psLastItem->hCmdComplete, MRST_TRUE);
			psLastItem->bCmdCompleted = MRST_TRUE;
			bMISRScheduled = MRST_TRUE;
		}
		if (psFlipItem)
			psSwapChain->sLastItem = *psFlipItem;
	}
	if (bSchedule && !bMISRScheduled)
		OSScheduleMISR(psSysData);
}


static void MRSTLFBFlipOverlay(MRSTLFB_DEVINFO *psDevInfo,
			struct intel_overlay_context *psContext)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	u32 ovadd_reg = OV_OVADD;

	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	DRM_INFO("%s: flip 0x%x, index %d, pipe 0x%x\n", __func__,
		psContext->ovadd, psContext->index, psContext->pipe);

	if (psContext->index == 1)
		ovadd_reg = OVC_OVADD;

	psContext->ovadd |= psContext->pipe;
	psContext->ovadd |= 1;

	PSB_WVDC32(psContext->ovadd, ovadd_reg);
}

static void MRSTLFBFlipSprite(MRSTLFB_DEVINFO *psDevInfo,
			struct intel_sprite_context *psContext)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config;
	struct mdfld_dsi_hw_context *ctx;
	u32 reg_offset;
	int pipe;

	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	if (psContext->index == 0) {
		reg_offset = 0;
		pipe = 0;
		dsi_config = dev_priv->dsi_configs[0];
	} else if (psContext->index == 1) {
		reg_offset = 0x1000;
		pipe = 1;
	} else if (psContext->index == 2) {
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		pipe = 2;
	} else
		return;

	if ((psContext->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(psContext->pos, DSPAPOS + reg_offset);
	if ((psContext->update_mask & SPRITE_UPDATE_SIZE)) {
		PSB_WVDC32(psContext->size, DSPASIZE + reg_offset);
		PSB_WVDC32(psContext->stride, DSPASTRIDE + reg_offset);
	}

	if ((psContext->update_mask & SPRITE_UPDATE_CONTROL))
		PSB_WVDC32(psContext->cntr, DSPACNTR + reg_offset);

	if ((psContext->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(psContext->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(psContext->surf, DSPASURF + reg_offset);
	}

	if (dsi_config) {
		ctx = &dsi_config->dsi_hw_context;
		ctx->dsppos = psContext->pos;
		ctx->dspsize = psContext->size;
		ctx->dspstride = psContext->stride;
		ctx->dspcntr = psContext->cntr;
		ctx->dsplinoff = psContext->linoff;
		ctx->dspsurf = psContext->surf;
	}
}

static void MRSTLFBFlipPrimary(MRSTLFB_DEVINFO *psDevInfo,
			struct intel_sprite_context *psContext)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config = 0;
	struct mdfld_dsi_hw_context *ctx = 0;
	u32 reg_offset;
	int pipe;

	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	if (psContext->index == 0) {
		reg_offset = 0;
		pipe = 0;
		dsi_config = dev_priv->dsi_configs[0];
	} else if (psContext->index == 1) {
		reg_offset = 0x1000;
		pipe = 1;
	} else if (psContext->index == 2) {
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		pipe = 2;
	} else
		return;

	/*for HDMI only flip the surface address*/
	if (pipe == 1)
		psContext->update_mask &= SPRITE_UPDATE_SURFACE;

	if ((psContext->update_mask & SPRITE_UPDATE_POSITION))
		PSB_WVDC32(psContext->pos, DSPAPOS + reg_offset);
	if ((psContext->update_mask & SPRITE_UPDATE_SIZE)) {
		PSB_WVDC32(psContext->size, DSPASIZE + reg_offset);
		PSB_WVDC32(psContext->stride, DSPASTRIDE + reg_offset);
	}

	if ((psContext->update_mask & SPRITE_UPDATE_CONTROL))
		PSB_WVDC32(psContext->cntr, DSPACNTR + reg_offset);

	if ((psContext->update_mask & SPRITE_UPDATE_SURFACE)) {
		PSB_WVDC32(psContext->linoff, DSPALINOFF + reg_offset);
		PSB_WVDC32(psContext->surf, DSPASURF + reg_offset);
	}

	if (dsi_config) {
		ctx = &dsi_config->dsi_hw_context;
		ctx->dsppos = psContext->pos;
		ctx->dspsize = psContext->size;
		ctx->dspstride = psContext->stride;
		ctx->dspcntr = psContext->cntr;
		ctx->dsplinoff = psContext->linoff;
		ctx->dspsurf = psContext->surf;
	}
}

static IMG_BOOL MRSTLFBFlipContexts(MRSTLFB_DEVINFO *psDevInfo,
			struct mdfld_plane_contexts *psContexts)
{
	struct intel_sprite_context *psPrimaryContext;
	struct intel_sprite_context *psSpriteContext;
	struct intel_overlay_context *psOverlayContext;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	IMG_BOOL ret = IMG_TRUE;
	int i;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, MRST_TRUE)) {
		DRM_ERROR("mdfld_dsi_dsr: failed to hw_begin\n");
		return IMG_FALSE;
	}

	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	/*flip all active primary planes*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (psContexts->active_primaries & (1 << i)) {
			psPrimaryContext = &psContexts->primary_contexts[i];
			MRSTLFBFlipPrimary(psDevInfo, psPrimaryContext);
		}
	}

	/*flip all active sprite planes*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (psContexts->active_sprites & (1 << i)) {
			psSpriteContext = &psContexts->sprite_contexts[i];
			MRSTLFBFlipSprite(psDevInfo, psSpriteContext);
		}
	}

	/*flip all active overlay planes*/
	for (i = 0; i < INTEL_OVERLAY_PLANE_NUM; i++) {
		if (psContexts->active_overlays & (1 << i)) {
			psOverlayContext = &psContexts->overlay_contexts[i];
			MRSTLFBFlipOverlay(psDevInfo, psOverlayContext);
		}
	}

	if (mdfld_dsi_dsr_update_panel_fb(dev_priv->dsi_configs[0])) {
		DRM_ERROR("mdfld_dsi_dsr: failed to update panel fb\n");
		ret = IMG_FALSE;
	}

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return ret;
}

static void MRSTLFBRestoreLastFlip(MRSTLFB_DEVINFO *psDevInfo)
{
	if (!psDevInfo->bSuspended && !psDevInfo->bLeaveVT)
	{
		if (psDevInfo->bLastFlipAddrValid)
		{
			MRSTLFBFlipToSurface(psDevInfo, psDevInfo->ulLastFlipAddr);
		}
	}
}

static void MRSTLFBClearSavedFlip(MRSTLFB_DEVINFO *psDevInfo)
{
	psDevInfo->bLastFlipAddrValid = MRST_FALSE;
}


static IMG_BOOL FlushInternalVSyncQueue(MRSTLFB_SWAPCHAIN *psSwapChain,
			 MRST_BOOL bFlip)
{
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
	unsigned long            ulMaxIndex;
	unsigned long            i;
	IMG_BOOL                 ret = IMG_TRUE;

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulSwapChainLength - 1;

	MRSTFBFlipComplete(psSwapChain, NULL, MRST_TRUE);
	for(i = 0; i < psSwapChain->ulSwapChainLength; i++)
	{
		if (psFlipItem->bValid == MRST_FALSE)
		{
			continue;
		}

		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": FlushInternalVSyncQueue: Flushing swap buffer (index %lu)\n", psSwapChain->ulRemoveIndex));

		if (psFlipItem->bFlipped == MRST_FALSE && bFlip)
		{
			if (psFlipItem->psBuffer)
				ret = MRSTLFBFlip(
					psSwapChain->psDevInfo,
					psFlipItem->psBuffer);
			else
				ret = MRSTLFBFlipContexts(
					psSwapChain->psDevInfo,
					&psFlipItem->sPlaneContexts);
			if (ret == IMG_FALSE) {
				DRM_INFO("%s: returning %d from DRMLFBFlipBuffer2", __func__, ret);
				return ret;
			}

		}

		if(psFlipItem->bCmdCompleted == MRST_FALSE)
		{
			DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": FlushInternalVSyncQueue: Calling command complete for swap buffer (index %lu)\n", psSwapChain->ulRemoveIndex));

			psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete((IMG_HANDLE)psFlipItem->hCmdComplete, MRST_TRUE);
		}


		psSwapChain->ulRemoveIndex++;

		if(psSwapChain->ulRemoveIndex > ulMaxIndex)
		{
			psSwapChain->ulRemoveIndex = 0;
		}


		psFlipItem->bFlipped = MRST_FALSE;
		psFlipItem->bCmdCompleted = MRST_FALSE;
		psFlipItem->bValid = MRST_FALSE;


		psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	}

	psSwapChain->ulInsertIndex = 0;
	psSwapChain->ulRemoveIndex = 0;

	return IMG_TRUE;
}

static int DRMLFBFifoEmpty(MRSTLFB_DEVINFO *psDevInfo)
{
	struct drm_device *dev = psDevInfo->psDrmDevice;
	struct drm_psb_private *dev_priv =
	(struct drm_psb_private *) psDevInfo->psDrmDevice->dev_private;

	return dev_priv->async_check_fifo_empty(dev);
}

static IMG_BOOL DRMLFBFlipBuffer(MRSTLFB_DEVINFO *psDevInfo,
		 MRSTLFB_SWAPCHAIN *psSwapChain,
		 MRSTLFB_BUFFER *psBuffer)
{
	IMG_BOOL ret = IMG_TRUE;
	if(psSwapChain != NULL)
	{
		if(psDevInfo->psCurrentSwapChain != NULL)
		{
			if(psDevInfo->psCurrentSwapChain != psSwapChain)
				ret = FlushInternalVSyncQueue(
				psDevInfo->psCurrentSwapChain, MRST_FALSE);
			if (ret == IMG_FALSE) {
				DRM_INFO("%s: returning %d from FlushInternalVSyncQueue\n", __func__, ret);
				return ret;
			}
		}
		psDevInfo->psCurrentSwapChain = psSwapChain;
		psDevInfo->psCurrentBuffer = psBuffer;
	}

	ret = MRSTLFBFlip(psDevInfo, psBuffer);
	if (ret != IMG_TRUE)
		DRM_INFO("%s: returning %d from MRSTLFBFlip", __func__, ret);
	return ret;
}

static IMG_BOOL DRMLFBFlipBuffer2(MRSTLFB_DEVINFO *psDevInfo,
			MRSTLFB_SWAPCHAIN *psSwapChain,
			struct mdfld_plane_contexts *psContexts)
{
	IMG_BOOL ret = IMG_TRUE;

	if (!psSwapChain)
		goto flip_out;

	if (!psDevInfo->psCurrentSwapChain) {
		psDevInfo->psCurrentSwapChain = psSwapChain;
		psDevInfo->psCurrentBuffer = NULL;
		goto flip_out;
	}

	if (psDevInfo->psCurrentSwapChain != psSwapChain) {
		ret = FlushInternalVSyncQueue(psDevInfo->psCurrentSwapChain,
					MRST_FALSE);
		if (ret == IMG_FALSE) {
			DRM_INFO("%s: returning %d from FlushInternalVSyncQueue\n", __func__, ret);
			return ret;
		}
                psDevInfo->psCurrentSwapChain = psSwapChain;
                psDevInfo->psCurrentBuffer = NULL;
	}

flip_out:
	ret = MRSTLFBFlipContexts(psDevInfo, psContexts);
	if (ret != IMG_TRUE)
		DRM_INFO("%s: returning %d from MRSTLFBFlipContexts\n", __func__, ret);
	return ret;
}

static void SetFlushStateNoLock(MRSTLFB_DEVINFO* psDevInfo,
                                        MRST_BOOL bFlushState)
{
	if (bFlushState)
	{
		if (psDevInfo->ulSetFlushStateRefCount == 0)
		{
			psDevInfo->bFlushCommands = MRST_TRUE;
			if (psDevInfo->psCurrentSwapChain != NULL)
			{
				FlushInternalVSyncQueue(psDevInfo->psCurrentSwapChain, MRST_TRUE);
			}
		}
		psDevInfo->ulSetFlushStateRefCount++;
	}
	else
	{
		if (psDevInfo->ulSetFlushStateRefCount != 0)
		{
			psDevInfo->ulSetFlushStateRefCount--;
			if (psDevInfo->ulSetFlushStateRefCount == 0)
			{
				psDevInfo->bFlushCommands = MRST_FALSE;
			}
		}
	}
}

static IMG_VOID SetFlushState(MRSTLFB_DEVINFO* psDevInfo,
                                      MRST_BOOL bFlushState)
{
	unsigned long ulLockFlags;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	SetFlushStateNoLock(psDevInfo, bFlushState);

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
}

static IMG_VOID SetDCState(IMG_HANDLE hDevice, IMG_UINT32 ui32State)
{
	MRSTLFB_DEVINFO *psDevInfo = (MRSTLFB_DEVINFO *)hDevice;

	switch (ui32State)
	{
		case DC_STATE_FLUSH_COMMANDS:
			SetFlushState(psDevInfo, MRST_TRUE);
			break;
		case DC_STATE_NO_FLUSH_COMMANDS:
			SetFlushState(psDevInfo, MRST_FALSE);
			break;
		default:
			break;
	}

	return;
}

static int FrameBufferEvents(struct notifier_block *psNotif,
                             unsigned long event, void *data)
{
	MRSTLFB_DEVINFO *psDevInfo;
	struct fb_event *psFBEvent = (struct fb_event *)data;
	MRST_BOOL bBlanked;


	if (event != FB_EVENT_BLANK)
	{
		return 0;
	}

	psDevInfo = GetAnchorPtr();

	bBlanked = (*(IMG_INT *)psFBEvent->data != 0) ? MRST_TRUE: MRST_FALSE;

	if (bBlanked != psDevInfo->bBlanked)
	{
		psDevInfo->bBlanked = bBlanked;

		SetFlushState(psDevInfo, bBlanked);
	}

	return 0;
}


static MRST_ERROR UnblankDisplay(MRSTLFB_DEVINFO *psDevInfo)
{
	int res;

	console_lock();
	res = fb_blank(psDevInfo->psLINFBInfo, 0);
	console_unlock();
	if (res != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_blank failed (%d)", res);
		return (MRST_ERROR_GENERIC);
	}

	return (MRST_OK);
}

static MRST_ERROR EnableLFBEventNotification(MRSTLFB_DEVINFO *psDevInfo)
{
	int                res;
	MRST_ERROR         eError;


	memset(&psDevInfo->sLINNotifBlock, 0, sizeof(psDevInfo->sLINNotifBlock));

	psDevInfo->sLINNotifBlock.notifier_call = FrameBufferEvents;
	psDevInfo->bBlanked = MRST_FALSE;

	res = fb_register_client(&psDevInfo->sLINNotifBlock);
	if (res != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_register_client failed (%d)", res);

		return (MRST_ERROR_GENERIC);
	}

	eError = UnblankDisplay(psDevInfo);
	if (eError != MRST_OK)
	{
		DEBUG_PRINTK((KERN_WARNING DRIVER_PREFIX
			": UnblankDisplay failed (%d)", eError));
		return eError;
	}

	return (MRST_OK);
}

static MRST_ERROR DisableLFBEventNotification(MRSTLFB_DEVINFO *psDevInfo)
{
	int res;


	res = fb_unregister_client(&psDevInfo->sLINNotifBlock);
	if (res != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_unregister_client failed (%d)", res);
		return (MRST_ERROR_GENERIC);
	}

	return (MRST_OK);
}

static PVRSRV_ERROR OpenDCDevice(IMG_UINT32 ui32DeviceID,
                                 IMG_HANDLE *phDevice,
                                 PVRSRV_SYNC_DATA* psSystemBufferSyncData)
{
	MRSTLFB_DEVINFO *psDevInfo;
	MRST_ERROR eError;

	UNREFERENCED_PARAMETER(ui32DeviceID);

	psDevInfo = GetAnchorPtr();


	psDevInfo->sSystemBuffer.psSyncData = psSystemBufferSyncData;

	psDevInfo->ulSetFlushStateRefCount = 0;
	psDevInfo->bFlushCommands = MRST_FALSE;

	eError = EnableLFBEventNotification(psDevInfo);
	if (eError != MRST_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": Couldn't enable framebuffer event notification\n");
		return PVRSRV_ERROR_UNABLE_TO_OPEN_DC_DEVICE;
	}


	*phDevice = (IMG_HANDLE)psDevInfo;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR CloseDCDevice(IMG_HANDLE hDevice)
{
	MRSTLFB_DEVINFO *psDevInfo = (MRSTLFB_DEVINFO *)hDevice;
	MRST_ERROR eError;

	eError = DisableLFBEventNotification(psDevInfo);
	if (eError != MRST_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": Couldn't disable framebuffer event notification\n");
		return PVRSRV_ERROR_UNABLE_TO_REMOVE_DEVICE;
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR EnumDCFormats(IMG_HANDLE hDevice,
                                  IMG_UINT32 *pui32NumFormats,
                                  DISPLAY_FORMAT *psFormat)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !pui32NumFormats)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*pui32NumFormats = 1;

	if(psFormat)
	{
		psFormat[0] = psDevInfo->sDisplayFormat;
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR EnumDCDims(IMG_HANDLE hDevice,
                               DISPLAY_FORMAT *psFormat,
                               IMG_UINT32 *pui32NumDims,
                               DISPLAY_DIMS *psDim)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !psFormat || !pui32NumDims)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*pui32NumDims = 1;


	if(psDim)
	{
		psDim[0] = psDevInfo->sDisplayDim;
	}

	return (PVRSRV_OK);
}


static PVRSRV_ERROR GetDCSystemBuffer(IMG_HANDLE hDevice, IMG_HANDLE *phBuffer)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;



	*phBuffer = (IMG_HANDLE)&psDevInfo->sSystemBuffer;

	return (PVRSRV_OK);
}


static PVRSRV_ERROR GetDCInfo(IMG_HANDLE hDevice, DISPLAY_INFO *psDCInfo)
{
	MRSTLFB_DEVINFO	*psDevInfo;

	if(!hDevice || !psDCInfo)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*psDCInfo = psDevInfo->sDisplayInfo;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetDCBufferAddr(IMG_HANDLE        hDevice,
                                    IMG_HANDLE        hBuffer,
                                    IMG_SYS_PHYADDR   **ppsSysAddr,
                                    IMG_SIZE_T        *pui32ByteSize,
                                    IMG_VOID          **ppvCpuVAddr,
                                    IMG_HANDLE        *phOSMapInfo,
                                    IMG_BOOL          *pbIsContiguous,
	                            IMG_UINT32	      *pui32TilingStride)
{
	MRSTLFB_DEVINFO	*psDevInfo;
	MRSTLFB_BUFFER *psSystemBuffer;

	UNREFERENCED_PARAMETER(pui32TilingStride);

	if(!hDevice)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	if(!hBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	psSystemBuffer = (MRSTLFB_BUFFER *)hBuffer;

	if (!ppsSysAddr)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	if( psSystemBuffer->bIsContiguous )
		*ppsSysAddr = &psSystemBuffer->uSysAddr.sCont;
	else
		*ppsSysAddr = psSystemBuffer->uSysAddr.psNonCont;

	if (!pui32ByteSize)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	*pui32ByteSize = psSystemBuffer->ui32BufferSize;

	if (ppvCpuVAddr)
	{
		*ppvCpuVAddr = psSystemBuffer->sCPUVAddr;
	}

	if (phOSMapInfo)
	{
		*phOSMapInfo = (IMG_HANDLE)0;
	}

	if (pbIsContiguous)
	{
		*pbIsContiguous = psSystemBuffer->bIsContiguous;
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR CreateDCSwapChain(IMG_HANDLE hDevice,
                                      IMG_UINT32 ui32Flags,
                                      DISPLAY_SURF_ATTRIBUTES *psDstSurfAttrib,
                                      DISPLAY_SURF_ATTRIBUTES *psSrcSurfAttrib,
                                      IMG_UINT32 ui32BufferCount,
                                      PVRSRV_SYNC_DATA **ppsSyncData,
                                      IMG_UINT32 ui32OEMFlags,
                                      IMG_HANDLE *phSwapChain,
                                      IMG_UINT32 *pui32SwapChainID)
{
	MRSTLFB_DEVINFO	*psDevInfo;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	MRSTLFB_BUFFER **ppsBuffer;
	MRSTLFB_VSYNC_FLIP_ITEM *psVSyncFlips;
	IMG_UINT32 i;
	IMG_UINT32 iSCId = MAX_SWAPCHAINS;
	PVRSRV_ERROR eError = PVRSRV_ERROR_NOT_SUPPORTED;
	unsigned long ulLockFlags;
	struct drm_device* psDrmDev;
	unsigned long ulSwapChainLength;
	struct drm_psb_private *dev_priv = NULL;
	struct psb_framebuffer *psbfb = NULL;
	struct psb_fbdev *fbdev = NULL;

	UNREFERENCED_PARAMETER(ui32OEMFlags);


	if(!hDevice
	|| !psDstSurfAttrib
	|| !psSrcSurfAttrib
	|| !ppsSyncData
	|| !phSwapChain)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;


	if(ui32BufferCount > psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers)
	{
		return (PVRSRV_ERROR_TOOMANYBUFFERS);
	}


	ulSwapChainLength = ui32BufferCount + 6;


	if(psDstSurfAttrib->pixelformat != psDevInfo->sDisplayFormat.pixelformat
	|| psDstSurfAttrib->sDims.ui32ByteStride != psDevInfo->sDisplayDim.ui32ByteStride
	|| psDstSurfAttrib->sDims.ui32Width != psDevInfo->sDisplayDim.ui32Width
	|| psDstSurfAttrib->sDims.ui32Height != psDevInfo->sDisplayDim.ui32Height)
	{

		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	if(psDstSurfAttrib->pixelformat != psSrcSurfAttrib->pixelformat
	|| psDstSurfAttrib->sDims.ui32ByteStride != psSrcSurfAttrib->sDims.ui32ByteStride
	|| psDstSurfAttrib->sDims.ui32Width != psSrcSurfAttrib->sDims.ui32Width
	|| psDstSurfAttrib->sDims.ui32Height != psSrcSurfAttrib->sDims.ui32Height)
	{

		return (PVRSRV_ERROR_INVALID_PARAMS);
	}


	UNREFERENCED_PARAMETER(ui32Flags);


	psSwapChain = (MRSTLFB_SWAPCHAIN*)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_SWAPCHAIN));
	if(!psSwapChain)
	{
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}

	for(iSCId = 0;iSCId < MAX_SWAPCHAINS;++iSCId)
	{
		if( psDevInfo->apsSwapChains[iSCId] == NULL )
		{
			psDevInfo->apsSwapChains[iSCId] = psSwapChain;
			break;
		}
	}

	if(iSCId == MAX_SWAPCHAINS)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeSwapChain;
	}
	ppsBuffer = (MRSTLFB_BUFFER**)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_BUFFER*) * ui32BufferCount);
	if(!ppsBuffer)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeSwapChain;
	}
	for (i = 0; i < ui32BufferCount; i++) ppsBuffer[i] = NULL;


	psVSyncFlips = (MRSTLFB_VSYNC_FLIP_ITEM *)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_VSYNC_FLIP_ITEM) * ulSwapChainLength);
	if (!psVSyncFlips)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorFreeBuffers;
	}

	psSwapChain->ulSwapChainLength = ulSwapChainLength;
	psSwapChain->ulBufferCount = (unsigned long)ui32BufferCount;
	psSwapChain->ppsBuffer = ppsBuffer;
	psSwapChain->psVSyncFlips = psVSyncFlips;
	psSwapChain->ulInsertIndex = 0;
	psSwapChain->ulRemoveIndex = 0;
	psSwapChain->psPVRJTable = &psDevInfo->sPVRJTable;
	psSwapChain->sLastItem.bValid = MRST_FALSE;


	for (i = 0; i < ui32BufferCount; i++)
	{
		unsigned long bufSize = psDevInfo->sDisplayDim.ui32ByteStride * psDevInfo->sDisplayDim.ui32Height;
		if(MRSTLFBAllocBuffer(psDevInfo, bufSize, &ppsBuffer[i] ) != MRST_OK ) {
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto ErrorFreeAllocatedBuffes;
		}
		ppsBuffer[i]->psSyncData = ppsSyncData[i];
	}


	for (i = 0; i < ulSwapChainLength; i++)
	{
		psVSyncFlips[i].bValid = MRST_FALSE;
		psVSyncFlips[i].bFlipped = MRST_FALSE;
		psVSyncFlips[i].bCmdCompleted = MRST_FALSE;
	}

	psDrmDev = psDevInfo->psDrmDevice;

	psSwapChain->psDevInfo = psDevInfo;
	psSwapChain->psDrmDev = psDrmDev;
	psSwapChain->psDrmDriver = psDrmDev->driver;

	dev_priv = (struct drm_psb_private *)psDrmDev->dev_private;
	fbdev = dev_priv->fbdev;
	if (fbdev != NULL)
		psbfb = fbdev->pfb;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	psSwapChain->ui32SwapChainID = *pui32SwapChainID = iSCId+1;
	psSwapChain->ui32SwapChainPropertyFlag =
		PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A
		| PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B
		| PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C;

	psSwapChain->ulSwapChainGTTOffset =
		(psbfb != NULL) ? psbfb->offset : 0;

	if(psDevInfo->psCurrentSwapChain == NULL)
		psDevInfo->psCurrentSwapChain = psSwapChain;

	psDevInfo->ui32SwapChainNum++;
	if(psDevInfo->ui32SwapChainNum == 1)
	{
		MRSTLFBEnableVSyncInterrupt(psDevInfo);
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);


	*phSwapChain = (IMG_HANDLE)psSwapChain;

	return (PVRSRV_OK);

ErrorFreeAllocatedBuffes:
	for (i = 0; i < ui32BufferCount; i++)
	{
		if(ppsBuffer[i] != NULL)
			MRSTLFBFreeBuffer( psDevInfo, &ppsBuffer[i] );
	}
	MRSTLFBFreeKernelMem(psVSyncFlips);
ErrorFreeBuffers:
	MRSTLFBFreeKernelMem(ppsBuffer);
ErrorFreeSwapChain:
	if(iSCId != MAX_SWAPCHAINS && psDevInfo->apsSwapChains[iSCId] == psSwapChain )
		psDevInfo->apsSwapChains[iSCId] = NULL;
	MRSTLFBFreeKernelMem(psSwapChain);

	return eError;
}

static PVRSRV_ERROR DestroyDCSwapChain(IMG_HANDLE hDevice,
	IMG_HANDLE hSwapChain)
{
	MRSTLFB_DEVINFO	*psDevInfo;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	unsigned long ulLockFlags;
	int i;
	IMG_UINT32 taskid;

	if(!hDevice || !hSwapChain)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}
	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;
	psSwapChain = (MRSTLFB_SWAPCHAIN*)hSwapChain;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	psDevInfo->ui32SwapChainNum--;

	if(psDevInfo->ui32SwapChainNum == 0)
	{
		MRSTLFBDisableVSyncInterrupt(psDevInfo);
	}

	psDevInfo->apsSwapChains[ psSwapChain->ui32SwapChainID -1] = NULL;

	/*
	 * Release the spin lock here, as FlushInternalVSyncQueue() may invoke
	 * mutex_lock when freeing buffer.
	 */
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	FlushInternalVSyncQueue(psSwapChain, psDevInfo->ui32SwapChainNum == 0);

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (psDevInfo->ui32SwapChainNum == 0)
	{

		DRMLFBFlipBuffer(psDevInfo, NULL, &psDevInfo->sSystemBuffer);
		MRSTLFBClearSavedFlip(psDevInfo);
	}

	if (psDevInfo->psCurrentSwapChain == psSwapChain ||
		psDevInfo->ui32SwapChainNum == 0)
		psDevInfo->psCurrentSwapChain = NULL;

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (psSwapChain->ulBufferCount)
		taskid = (psSwapChain->ppsBuffer[0])->ui32OwnerTaskID;

	for (i = 0; i < psSwapChain->ulBufferCount; i++)
	{
		MRSTLFBFreeBuffer(psDevInfo, &psSwapChain->ppsBuffer[i] );
	}

	if (psSwapChain->ulBufferCount)
		psb_gtt_free_ht_for_tgid(psDevInfo->psDrmDevice,
			taskid);

	MRSTLFBFreeKernelMem(psSwapChain->psVSyncFlips);
	MRSTLFBFreeKernelMem(psSwapChain->ppsBuffer);
	MRSTLFBFreeKernelMem(psSwapChain);

	return (PVRSRV_OK);
}

static PVRSRV_ERROR SetDCDstRect(IMG_HANDLE hDevice,
	IMG_HANDLE hSwapChain,
	IMG_RECT *psRect)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(psRect);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR SetDCSrcRect(IMG_HANDLE hDevice,
                                 IMG_HANDLE hSwapChain,
                                 IMG_RECT *psRect)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(psRect);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR SetDCDstColourKey(IMG_HANDLE hDevice,
                                      IMG_HANDLE hSwapChain,
                                      IMG_UINT32 ui32CKColour)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(ui32CKColour);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR SetDCSrcColourKey(IMG_HANDLE hDevice,
                                      IMG_HANDLE hSwapChain,
                                      IMG_UINT32 ui32CKColour)
{
	UNREFERENCED_PARAMETER(hDevice);
	UNREFERENCED_PARAMETER(hSwapChain);
	UNREFERENCED_PARAMETER(ui32CKColour);



	return (PVRSRV_ERROR_NOT_SUPPORTED);
}

static PVRSRV_ERROR GetDCBuffers(IMG_HANDLE hDevice,
                                 IMG_HANDLE hSwapChain,
                                 IMG_UINT32 *pui32BufferCount,
                                 IMG_HANDLE *phBuffer)
{
	MRSTLFB_DEVINFO   *psDevInfo;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	unsigned long      i;


	if(!hDevice
	|| !hSwapChain
	|| !pui32BufferCount
	|| !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;
	psSwapChain = (MRSTLFB_SWAPCHAIN*)hSwapChain;


	*pui32BufferCount = (IMG_UINT32)psSwapChain->ulBufferCount;


	for (i = 0; i < psSwapChain->ulBufferCount; i++)
	{
		phBuffer[i] = (IMG_HANDLE)psSwapChain->ppsBuffer[i];
	}

	return (PVRSRV_OK);
}

static PVRSRV_ERROR GetDCFrontBuffer(IMG_HANDLE hDevice,
                                 IMG_UINT32 *pui32SwapChainID,
                                 IMG_HANDLE *phBuffer)
{
	MRSTLFB_DEVINFO   *psDevInfo;

	if(!hDevice
	|| !pui32SwapChainID
	|| !phBuffer)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;

	*pui32SwapChainID = psDevInfo->psCurrentSwapChain ?
		psDevInfo->psCurrentSwapChain->ui32SwapChainID : 0;

	*phBuffer = (IMG_HANDLE)psDevInfo->psCurrentBuffer;

	return (PVRSRV_OK);
}

static PVRSRV_ERROR SwapToDCBuffer(IMG_HANDLE hDevice,
                                   IMG_HANDLE hBuffer,
                                   IMG_UINT32 ui32SwapInterval,
                                   IMG_HANDLE hPrivateTag,
                                   IMG_UINT32 ui32ClipRectCount,
                                   IMG_RECT *psClipRect)
{
	MRSTLFB_DEVINFO *psDevInfo;

	UNREFERENCED_PARAMETER(ui32SwapInterval);
	UNREFERENCED_PARAMETER(hPrivateTag);
	UNREFERENCED_PARAMETER(psClipRect);

	if(!hDevice
	|| !hBuffer
	|| (ui32ClipRectCount != 0))
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo = (MRSTLFB_DEVINFO*)hDevice;


	return (PVRSRV_OK);
}

void MRSTLFBFlipTimerFn(unsigned long arg)
{
	MRSTLFB_DEVINFO *psDevInfo = (MRSTLFB_DEVINFO *)arg;
	unsigned long ulLockFlags;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	MRSTLFB_VSYNC_FLIP_ITEM *psLastItem;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	psSwapChain = psDevInfo->psCurrentSwapChain;
	if (psSwapChain == NULL)
	{
		printk(KERN_WARNING "MRSTLFBFlipTimerFn: Swapchain is null\n");
		goto ExitUnlock;
	}
	if (psSwapChain->ulRemoveIndex == psSwapChain->ulInsertIndex)
	{
		psLastItem = &(psSwapChain->sLastItem);
		if (psLastItem->bValid && psLastItem->bFlipped && psLastItem->bCmdCompleted == MRST_FALSE)
		{
			MRSTFBFlipComplete(psSwapChain, NULL, MRST_TRUE);
			goto ExitUnlock;
		}
	}
	printk(KERN_WARNING "MRSTLFBFlipTimerFn: swapchain is not empty, flush queue\n");
	FlushInternalVSyncQueue(psSwapChain, MRST_TRUE);
ExitUnlock:
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
}


static PVRSRV_ERROR SwapToDCSystem(IMG_HANDLE hDevice,
                                   IMG_HANDLE hSwapChain)
{
	if(!hDevice || !hSwapChain)
	{
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}


	return (PVRSRV_OK);
}


static MRST_BOOL MRSTLFBVSyncIHandler(MRSTLFB_DEVINFO *psDevInfo, int iPipe)
{
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
	MRST_BOOL bStatus = MRST_TRUE;
	unsigned long ulMaxIndex;
	unsigned long ulLockFlags;
	MRSTLFB_SWAPCHAIN *psSwapChain;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);


	psSwapChain = psDevInfo->psCurrentSwapChain;
	if (psSwapChain == NULL)
		goto ExitUnlock;

	if (psDevInfo->bFlushCommands || psDevInfo->bSuspended || psDevInfo->bLeaveVT)
		goto ExitUnlock;

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulSwapChainLength - 1;

	while(psFlipItem->bValid)
	{
		if(psFlipItem->bFlipped)
		{
			if(!psFlipItem->bCmdCompleted)
			{
				MRST_BOOL bScheduleMISR;

				bScheduleMISR = MRST_TRUE;
				MRSTFBFlipComplete(psSwapChain, psFlipItem, MRST_TRUE);
				psFlipItem->bCmdCompleted = MRST_TRUE;
			}

			psFlipItem->ulSwapInterval--;

			if(psFlipItem->ulSwapInterval == 0)
			{
				psSwapChain->ulRemoveIndex++;

				if(psSwapChain->ulRemoveIndex > ulMaxIndex)
					psSwapChain->ulRemoveIndex = 0;

				psFlipItem->bCmdCompleted = MRST_FALSE;
				psFlipItem->bFlipped = MRST_FALSE;

				psFlipItem->bValid = MRST_FALSE;
			}
			else
			{

				break;
			}
		}
		else
		{
			if (psFlipItem->psBuffer)
				DRMLFBFlipBuffer(psDevInfo, psSwapChain,
						psFlipItem->psBuffer);
			else
				DRMLFBFlipBuffer2(psDevInfo, psSwapChain,
						&psFlipItem->sPlaneContexts);
			psFlipItem->bFlipped = MRST_TRUE;

			break;
		}


		psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	}
	if (psSwapChain->ulRemoveIndex != psSwapChain->ulInsertIndex) {
		mod_timer(&psDevInfo->sFlipTimer, FLIP_TIMEOUT + jiffies);
		bStatus = MRST_FALSE;
	}
ExitUnlock:
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	return bStatus;
}

#if defined(MRST_USING_INTERRUPTS)
static int
MRSTLFBVSyncISR(struct drm_device *psDrmDevice, int iPipe)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();

	return MRSTLFBVSyncIHandler(psDevInfo, iPipe);
}
#endif

static IMG_BOOL updatePlaneContexts(MRSTLFB_SWAPCHAIN *psSwapChain,
				DISPLAYCLASS_FLIP_COMMAND2 *psFlipCmd,
				struct mdfld_plane_contexts *psPlaneContexts)
{
	int i;
	PDC_MEM_INFO psMemInfo, psCurrentMemInfo;
	MRSTLFB_BUFFER *psBuffer, *psCurrentBuffer;
	struct fb_info *psLINFBInfo;

	if (!psSwapChain || !psFlipCmd || !psPlaneContexts) {
		DRM_ERROR("Invalid parameters\n");
		return IMG_FALSE;
	}

	if (!psPlaneContexts->active_primaries)
		return IMG_TRUE;

	/* if active_primaries, update plane surface address*/
	psCurrentMemInfo = 0;

	for (i = 0; i < psFlipCmd->ui32NumMemInfos; i++) {
		psMemInfo = psFlipCmd->ppsMemInfos[i];

		if (!psMemInfo)
			continue;

		if (psMemInfo->memType == PVRSRV_MEMTYPE_DEVICECLASS) {
			psCurrentMemInfo = psMemInfo;
			break;
		}
	}

	if (!psCurrentMemInfo) {
		DRM_ERROR("Failed to get FB MemInfo\n");
		return IMG_FALSE;
	}

	/*get mrstlfb_buffer*/
	psCurrentBuffer = 0;
	for (i = 0; i < psSwapChain->ulBufferCount; i++) {
		psBuffer = psSwapChain->ppsBuffer[i];

		if (!psBuffer)
			continue;

		if (psBuffer->sCPUVAddr == psMemInfo->pvLinAddrKM) {
			psCurrentBuffer = psBuffer;
			psLINFBInfo = psSwapChain->psDevInfo->psLINFBInfo;
			psLINFBInfo->screen_base = psBuffer->sCPUVAddr;
			break;
		}
	}

	if (!psCurrentBuffer) {
		DRM_ERROR("Failed to get FB Buffer\n");
		return IMG_FALSE;
	}

	/*update primary context with fb surface address*/
	for (i = 0; i < INTEL_SPRITE_PLANE_NUM; i++) {
		if (!(psSwapChain->ui32SwapChainPropertyFlag & (1 << i))) {
			psPlaneContexts->active_primaries &= ~(1 << i);
			continue;
		}

		if (psPlaneContexts->active_primaries & (1 << i)) {
			psPlaneContexts->primary_contexts[i].surf =
				psCurrentBuffer->sDevVAddr.uiAddr;
		}
	}

	return IMG_TRUE;
}

static IMG_BOOL ProcessFlip2(IMG_HANDLE hCmdCookie,
			IMG_UINT32 ui32DataSize,
			IMG_VOID *pvData)
{
	DISPLAYCLASS_FLIP_COMMAND2 *psFlipCmd;
	MRSTLFB_SWAPCHAIN *psSwapChain;
#if defined(MRST_USING_INTERRUPTS)
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
#endif
	unsigned long ulLockFlags;
	unsigned long irqflags;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	MRSTLFB_DEVINFO *psDevInfo;
	struct mdfld_plane_contexts *psPlaneContexts;
	struct mdfld_dsi_config *dsi_config;

	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND2 *)pvData;
	psDevInfo = (MRSTLFB_DEVINFO *)psFlipCmd->hExtDevice;
	psSwapChain = (MRSTLFB_SWAPCHAIN *)psFlipCmd->hExtSwapChain;
	dev = psDevInfo->psDrmDevice;
	dev_priv =
		(struct drm_psb_private *)psDevInfo->psDrmDevice->dev_private;

	/*verify private data*/
	if (!psFlipCmd->pvPrivData ||
	psFlipCmd->ui32PrivDataLength != sizeof(struct mdfld_plane_contexts)) {
		DRM_ERROR("%s: Invalid private data\n", __func__);
		return IMG_FALSE;
	}

	if (FirstCleanFlag == 1) {
		memset(psDevInfo->sSystemBuffer.sCPUVAddr, 0,
				psDevInfo->sSystemBuffer.ui32BufferSize);
		FirstCleanFlag = 0;
	}

	dsi_config = dev_priv->dsi_configs[0];

	psPlaneContexts = (struct mdfld_plane_contexts *)psFlipCmd->pvPrivData;

	mutex_lock(&dsi_config->context_lock);
	mdfld_dsi_dsr_forbid_locked(dsi_config);

	if (dev_priv->exit_idle && (dsi_config->type == MDFLD_DSI_ENCODER_DPI))
		dev_priv->exit_idle(dev, MDFLD_DSR_2D_3D, NULL, true);

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	/*update context*/
	updatePlaneContexts(psSwapChain, psFlipCmd, psPlaneContexts);

#if defined(MRST_USING_INTERRUPTS)
	if (!drm_psb_3D_vblank || psFlipCmd->ui32SwapInterval == 0 ||
		psDevInfo->bFlushCommands) {
#endif
		/* update sprite plane context*/
		if (DRMLFBFlipBuffer2(
			psDevInfo,
			 psSwapChain, psPlaneContexts) == IMG_FALSE) {
			DRM_INFO("%s: DRMLFBFlipBuffer2 failed\n", __func__);
			goto ExitErrorUnlock;
		}
		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
								IMG_TRUE);
#if defined(MRST_USING_INTERRUPTS)
		goto ExitTrueUnlock;
	}

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulInsertIndex];

	/*start Flip watch dog*/
	mod_timer(&psDevInfo->sFlipTimer, FLIP_TIMEOUT + jiffies);

	if (hdmi_state) {
		/*
		 * Enable HDMI vblank interrupt, otherwise page flip would stuck
		 * if both MIPI A and C are off.
		 */
		spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
		mid_enable_pipe_event(dev_priv, 1);
		psb_enable_pipestat(dev_priv, 1, PIPE_VBLANK_INTERRUPT_ENABLE);
		spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	}

	if (psFlipItem->bValid == MRST_FALSE) {
		unsigned long ulMaxIndex = psSwapChain->ulSwapChainLength - 1;
		if (psSwapChain->ulInsertIndex == psSwapChain->ulRemoveIndex) {
			/*update sprite plane context*/
			if (DRMLFBFlipBuffer2(
				psDevInfo,
				 psSwapChain, psPlaneContexts) == IMG_FALSE) {
				DRM_INFO("%s: DRMLFBFlipBuffer2 failed\n", __func__);
				psFlipItem->bFlipped = MRST_FALSE;
				goto ExitErrorUnlock;
			}
			psFlipItem->bFlipped = MRST_TRUE;
		} else {
			psFlipItem->bFlipped = MRST_FALSE;
		}

		psFlipItem->hCmdComplete = (MRST_HANDLE)hCmdCookie;
		psFlipItem->ulSwapInterval =
			(unsigned long)psFlipCmd->ui32SwapInterval;
		psFlipItem->psBuffer = NULL;
		/*copy plane contexts to this flip item*/
		memcpy(&psFlipItem->sPlaneContexts, psPlaneContexts,
			sizeof(struct mdfld_plane_contexts));
		psFlipItem->bValid = MRST_TRUE;

		psSwapChain->ulInsertIndex++;
		if (psSwapChain->ulInsertIndex > ulMaxIndex)
			psSwapChain->ulInsertIndex = 0;

		goto ExitTrueUnlock;
	}
ExitErrorUnlock:
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
	mdfld_dsi_dsr_allow_locked(dsi_config);
	mutex_unlock(&dsi_config->context_lock);
	return IMG_FALSE;
ExitTrueUnlock:
#endif
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
	mdfld_dsi_dsr_allow_locked(dsi_config);
	mutex_unlock(&dsi_config->context_lock);
	return IMG_TRUE;
}

static IMG_BOOL ProcessFlip(IMG_HANDLE  hCmdCookie,
                            IMG_UINT32  ui32DataSize,
                            IMG_VOID   *pvData, IMG_BOOL bFlush)
{
	DISPLAYCLASS_FLIP_COMMAND *psFlipCmd;
	MRSTLFB_DEVINFO *psDevInfo;
	MRSTLFB_BUFFER *psBuffer;
	MRSTLFB_SWAPCHAIN *psSwapChain;
#if defined(MRST_USING_INTERRUPTS)
	MRSTLFB_VSYNC_FLIP_ITEM* psFlipItem;
#endif
	unsigned long ulLockFlags;
	unsigned long irqflags;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config *dsi_config;

	if(!hCmdCookie || !pvData)
		return IMG_FALSE;


	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND*)pvData;

	if (psFlipCmd == IMG_NULL)
		return IMG_FALSE;


	psDevInfo = (MRSTLFB_DEVINFO*)psFlipCmd->hExtDevice;
	dev = psDevInfo->psDrmDevice;
	dev_priv = (struct drm_psb_private *)
		psDevInfo->psDrmDevice->dev_private;
	psBuffer = (MRSTLFB_BUFFER*)psFlipCmd->hExtBuffer;
	psSwapChain = (MRSTLFB_SWAPCHAIN*) psFlipCmd->hExtSwapChain;

	/* bFlush == true means hw recovery */
	if (bFlush) {
		spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);
		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie, IMG_TRUE);
		spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
		return IMG_TRUE;
	}

	if (!dev_priv->um_start) {
		dev_priv->um_start = true;
		dev_priv->b_async_flip_enable = true;
		if (dev_priv->b_dsr_enable_config)
			dev_priv->b_dsr_enable = true;
	}

	/*Screen is off, no need to send data*/
	if (psDevInfo->bScreenState) {
		spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie,
				IMG_TRUE);

		spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

		return IMG_TRUE;
	}

	if (!psBuffer)
		return ProcessFlip2(hCmdCookie, ui32DataSize, pvData);

	dsi_config = dev_priv->dsi_configs[0];

	mutex_lock(&dsi_config->context_lock);
	mdfld_dsi_dsr_forbid_locked(dsi_config);

	if (dev_priv->exit_idle && (dsi_config->type == MDFLD_DSI_ENCODER_DPI))
		dev_priv->exit_idle(dev, MDFLD_DSR_2D_3D, NULL, true);

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

#if defined(MRST_USING_INTERRUPTS)

    if(!drm_psb_3D_vblank || psFlipCmd->ui32SwapInterval == 0 || psDevInfo->bFlushCommands)
	{
#endif
		if (DRMLFBFlipBuffer(
			psDevInfo, psSwapChain, psBuffer) == IMG_FALSE) {
			DRM_INFO("%s: DRMLFBFlipBuffer failed\n", __func__);
			goto ExitErrorUnlock;
		}

		MRSTFBFlipComplete(psSwapChain, NULL, MRST_FALSE);
		psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie, IMG_TRUE);

#if defined(MRST_USING_INTERRUPTS)
		goto ExitTrueUnlock;
	}

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulInsertIndex];

	/*start Flip watch dog*/
	mod_timer(&psDevInfo->sFlipTimer, FLIP_TIMEOUT + jiffies);

	if (hdmi_state) {
		/*
		 * Enable HDMI vblank interrupt, otherwise page flip would stuck
		 * if both MIPI A and C are off.
		 */
		spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
		mid_enable_pipe_event(dev_priv, 1);
		psb_enable_pipestat(dev_priv, 1, PIPE_VBLANK_INTERRUPT_ENABLE);
		spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	}

	if(psFlipItem->bValid == MRST_FALSE)
	{
		unsigned long ulMaxIndex = psSwapChain->ulSwapChainLength - 1;
		if(psSwapChain->ulInsertIndex == psSwapChain->ulRemoveIndex)
		{
			if (DRMLFBFlipBuffer(psDevInfo,
				 psSwapChain, psBuffer) == IMG_FALSE) {
				DRM_INFO("%s: DRMLFBFlipBuffer failed\n", __func__);
				psFlipItem->bFlipped = MRST_FALSE;
				goto ExitErrorUnlock;
			}

			psFlipItem->bFlipped = MRST_TRUE;
		}
		else
		{
			psFlipItem->bFlipped = MRST_FALSE;
		}

		psFlipItem->hCmdComplete = (MRST_HANDLE)hCmdCookie;
		psFlipItem->ulSwapInterval = (unsigned long)psFlipCmd->ui32SwapInterval;
		psFlipItem->psBuffer = psBuffer;
		psFlipItem->bValid = MRST_TRUE;

		psSwapChain->ulInsertIndex++;
		if(psSwapChain->ulInsertIndex > ulMaxIndex)
		{
			psSwapChain->ulInsertIndex = 0;
		}

		goto ExitTrueUnlock;
	}
ExitErrorUnlock:
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
	mdfld_dsi_dsr_allow_locked(dsi_config);
	mutex_unlock(&dsi_config->context_lock);
	return IMG_FALSE;

ExitTrueUnlock:
#endif
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
	mdfld_dsi_dsr_allow_locked(dsi_config);
	mutex_unlock(&dsi_config->context_lock);
	return IMG_TRUE;
}


#if defined(PVR_MRST_FB_SET_PAR_ON_INIT)
static void MRSTFBSetPar(struct fb_info *psLINFBInfo)
{
	acquire_console_sem();

	if (psLINFBInfo->fbops->fb_set_par != NULL)
	{
		int res;

		res = psLINFBInfo->fbops->fb_set_par(psLINFBInfo);
		if (res != 0)
		{
			printk(KERN_WARNING DRIVER_PREFIX
				": fb_set_par failed: %d\n", res);

		}
	}
	else
	{
		printk(KERN_WARNING DRIVER_PREFIX
			": fb_set_par not set - HW cursor may not work\n");
	}

	release_console_sem();
}
#endif

void MRSTLFBSuspend(void)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	unsigned long ulLockFlags;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (!psDevInfo->bSuspended)
	{
#if !defined(PVR_MRST_STYLE_PM)
		if(psDevInfo->ui32SwapChainNum != 0)
		{
			MRSTLFBDisableVSyncInterrupt(psDevInfo);
		}
#endif
		psDevInfo->bSuspended = MRST_TRUE;
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
}

void MRSTLFBResume(void)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	unsigned long ulLockFlags;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (psDevInfo->bSuspended)
	{
#if !defined(PVR_MRST_STYLE_PM)
		if(psDevInfo->ui32SwapChainNum != 0)
		{
			MRSTLFBEnableVSyncInterrupt(psDevInfo);
		}
#endif
		psDevInfo->bSuspended = MRST_FALSE;

		MRSTLFBRestoreLastFlip(psDevInfo);
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

#if !defined(PVR_MRST_STYLE_PM)
	(void) UnblankDisplay(psDevInfo);
#endif
}

#ifdef DRM_PVR_USE_INTEL_FB
#include "mm.h"
int MRSTLFBHandleChangeFB(struct drm_device* dev, struct psb_framebuffer *psbfb)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	int i;
	struct drm_psb_private * dev_priv;
	struct psb_gtt * pg;

	if( !psDevInfo->sSystemBuffer.bIsContiguous )
		MRSTLFBFreeKernelMem( psDevInfo->sSystemBuffer.uSysAddr.psNonCont );

	dev_priv = (struct drm_psb_private *)dev->dev_private;
	pg = dev_priv->pg;

	psDevInfo->sDisplayDim.ui32ByteStride = psbfb->base.MEMBER_PITCH;
	psDevInfo->sDisplayDim.ui32Width = psbfb->base.width;
	psDevInfo->sDisplayDim.ui32Height = psbfb->base.height;

	psDevInfo->sSystemBuffer.ui32BufferSize = psbfb->size;

	psDevInfo->sSystemBuffer.sCPUVAddr = pg->vram_addr;

	psDevInfo->sSystemBuffer.sDevVAddr.uiAddr = 0;
	psDevInfo->sSystemBuffer.bIsAllocated = IMG_FALSE;

	if(psbfb->bo )
	{

		psDevInfo->sSystemBuffer.bIsContiguous = IMG_FALSE;
		psDevInfo->sSystemBuffer.uSysAddr.psNonCont = MRSTLFBAllocKernelMem( sizeof( IMG_SYS_PHYADDR ) * psbfb->bo->ttm->num_pages);
		for(i = 0;i < psbfb->bo->ttm->num_pages;++i)
		{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0))
			struct page *p = ttm_tt_get_page( psbfb->bo->ttm, i);
#else
			struct page *p = psbfb->bo->ttm->pages[i];
#endif
			psDevInfo->sSystemBuffer.uSysAddr.psNonCont[i].uiAddr = page_to_pfn(p) << PAGE_SHIFT;

		}
	}
	else
	{





		psDevInfo->sSystemBuffer.bIsContiguous = IMG_TRUE;
		psDevInfo->sSystemBuffer.uSysAddr.sCont.uiAddr = pg->stolen_base;
	}

	return 0;
}
#else

int MRSTLFBHandleChangeFB(struct drm_device* dev, struct psb_framebuffer *psbfb)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	int i;


	if( !psDevInfo->sSystemBuffer.bIsContiguous )
		MRSTLFBFreeKernelMem( psDevInfo->sSystemBuffer.uSysAddr.psNonCont );


	psDevInfo->sDisplayDim.ui32ByteStride = psbfb->base.MEMBER_PITCH;
	psDevInfo->sDisplayDim.ui32Width = psbfb->base.width;
	psDevInfo->sDisplayDim.ui32Height = psbfb->base.height;

	psDevInfo->sSystemBuffer.ui32BufferSize = psbfb->buf.size;
	psDevInfo->sSystemBuffer.sCPUVAddr = psbfb->buf.kMapping;
	psDevInfo->sSystemBuffer.sDevVAddr.uiAddr = psbfb->buf.offsetGTT;
	psDevInfo->sSystemBuffer.bIsAllocated = MRST_FALSE;

	if ( psbfb->buf.type == PSB_BUFFER_VRAM )
	{

		struct drm_device * psDrmDevice = psDevInfo->psDrmDevice;
		struct drm_psb_private * dev_priv = (struct drm_psb_private *)psDrmDevice->dev_private;
		struct psb_gtt * pg = dev_priv->pg;

		psDevInfo->sSystemBuffer.bIsContiguous = MRST_TRUE;
		psDevInfo->sSystemBuffer.uSysAddr.sCont.uiAddr = pg->stolen_base;
	} else {

		psDevInfo->sSystemBuffer.bIsContiguous = MRST_FALSE;
		psDevInfo->sSystemBuffer.uSysAddr.psNonCont = MRSTLFBAllocKernelMem( sizeof( IMG_SYS_PHYADDR ) * (psbfb->buf.pagesNum));
		for (i = 0; i < psbfb->buf.pagesNum; i++)
		{
			psDevInfo->sSystemBuffer.uSysAddr.psNonCont[i].uiAddr = psbfb_get_buffer_pfn( psDevInfo->psDrmDevice, &psbfb->buf, i) << PAGE_SHIFT;
		}
	}

	return 0;
}
#endif

MRST_ERROR MRSTLFBChangeSwapChainProperty(unsigned long *psSwapChainGTTOffset,
		unsigned long ulSwapChainGTTSize, IMG_INT32 i32Pipe)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
	struct drm_device *psDrmDevice = NULL;
	struct drm_psb_private *dev_priv = NULL;
	struct psb_gtt *pg = NULL;
	uint32_t tt_pages = 0;
	uint32_t max_gtt_offset = 0;

	int iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_NONE;
	IMG_UINT32 ui32SwapChainID = 0;
	unsigned long ulLockFlags;
	unsigned long ulCurrentSwapChainGTTOffset = 0;
	MRST_ERROR eError = MRST_ERROR_GENERIC;

	if (psDevInfo == IMG_NULL) {
		DRM_DEBUG("MRSTLFB hasn't been initialized\n");
		/* Won't attach/de-attach the plane in case of no swap chain
		 * created. */
		eError = MRST_ERROR_INIT_FAILURE;
		return eError;
	}

	psDrmDevice = psDevInfo->psDrmDevice;
	dev_priv = (struct drm_psb_private *)psDrmDevice->dev_private;
	pg = dev_priv->pg;
	if (pg == NULL) {
		DRM_ERROR("Invalid GTT data.\n");
		return eError;
	}

	tt_pages = (pg->gatt_pages < PSB_TT_PRIV0_PLIMIT) ?
		(pg->gatt_pages) : PSB_TT_PRIV0_PLIMIT;

	/* Another half of GTT is managed by TTM. */
	tt_pages /= 2;
	max_gtt_offset = tt_pages << PAGE_SHIFT;

	if ((psSwapChainGTTOffset == IMG_NULL) ||
			(ulSwapChainGTTSize == 0) ||
			((*psSwapChainGTTOffset + ulSwapChainGTTSize) >
			 max_gtt_offset)) {
		DRM_ERROR("Invalid GTT offset.\n");
		return eError;
	}

	switch (i32Pipe) {
	case 0:
		iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A;
		break;
	case 1:
		iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B;
		break;
	case 2:
		iSwapChainAttachedPlane = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");
		return eError;
	}

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (psDevInfo->apsSwapChains == IMG_NULL) {
		DRM_ERROR("No swap chain.\n");
		spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
		return eError;
	}

	ulCurrentSwapChainGTTOffset =
		psDevInfo->apsSwapChains[ui32SwapChainID]->ulSwapChainGTTOffset;

	for (ui32SwapChainID = 0; ui32SwapChainID < psDevInfo->ui32SwapChainNum;
			ui32SwapChainID++) {
		if (psDevInfo->apsSwapChains[ui32SwapChainID] == IMG_NULL)
			continue;

		if (*psSwapChainGTTOffset == ulCurrentSwapChainGTTOffset) {
			psDevInfo->apsSwapChains[ui32SwapChainID]->ui32SwapChainPropertyFlag |= iSwapChainAttachedPlane;

			/*
			 * Trigger the display plane to flip to the swap
			 * chain's last flip surface, to avoid that it still
			 * displays with original GTT offset after mode setting
			 * and attached to the specific swap chain.
			 */
			if (psDevInfo->bLastFlipAddrValid)
				*psSwapChainGTTOffset =
					psDevInfo->ulLastFlipAddr;
		}
		else
			psDevInfo->apsSwapChains[ui32SwapChainID]->ui32SwapChainPropertyFlag &= ~iSwapChainAttachedPlane;
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	eError = MRST_OK;
	return eError;
}

static int MRSTLFBFindMainPipe(struct drm_device *dev)
{
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head)
	{
		if ( drm_helper_crtc_in_use(crtc) )
		{
			struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
			return psb_intel_crtc->pipe;
		}
	}

	return 0;
}

#ifndef DRM_PVR_USE_INTEL_FB
static int DRMLFBLeaveVTHandler(struct drm_device *dev)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
  	unsigned long ulLockFlags;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (!psDevInfo->bLeaveVT)
	{
		if(psDevInfo->psCurrentSwapChain != NULL)
		{
			FlushInternalVSyncQueue(psDevInfo->psCurrentSwapChain, MRST_TRUE);
			SetFlushStateNoLock(psDevInfo, MRST_TRUE);
		}

		DRMLFBFlipBuffer(psDevInfo, NULL, &psDevInfo->sSystemBuffer);

		psDevInfo->bLeaveVT = MRST_TRUE;
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	return 0;
}

static int DRMLFBEnterVTHandler(struct drm_device *dev)
{
	MRSTLFB_DEVINFO *psDevInfo = GetAnchorPtr();
  	unsigned long ulLockFlags;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (psDevInfo->bLeaveVT)
	{
		if(psDevInfo->psCurrentSwapChain != NULL)
		{
			SetFlushStateNoLock(psDevInfo, MRST_FALSE);
		}

		psDevInfo->bLeaveVT = MRST_FALSE;

		MRSTLFBRestoreLastFlip(psDevInfo);
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	return 0;
}
#endif
static
int MRSTLFBScreenEventHandler(struct drm_device* psDrmDevice, int state)
{
	MRSTLFB_DEVINFO *psDevInfo;
	MRST_BOOL bScreenOFF;

	psDevInfo = GetAnchorPtr();

	bScreenOFF = (state == 0) ? MRST_TRUE : MRST_FALSE;

	if (bScreenOFF != psDevInfo->bScreenState) {
		DRM_INFO("Screen event:%d\n", bScreenOFF);
		del_timer(&psDevInfo->sFlipTimer);
		psDevInfo->bScreenState = bScreenOFF;
		SetFlushState(psDevInfo, bScreenOFF);
	}

	return 0;
}

static MRST_ERROR MRSTLFBInstallScreenEvents(MRSTLFB_DEVINFO *psDevInfo, MRSTLFB_SCREEN_EVENT_PFN pScreenEventHandler)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *) psDevInfo->psDrmDevice->dev_private;
	dev_priv->pvr_screen_event_handler = pScreenEventHandler;
	return (MRST_OK);
}

static MRST_ERROR InitDev(MRSTLFB_DEVINFO *psDevInfo)
{
	MRST_ERROR eError = MRST_ERROR_GENERIC;
	struct fb_info *psLINFBInfo;
	struct drm_device * psDrmDevice = psDevInfo->psDrmDevice;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct drm_psb_private * psDrmPrivate = (struct drm_psb_private *)psDrmDevice->dev_private;
	struct psb_fbdev * psPsbFBDev = (struct psb_fbdev *)psDrmPrivate->fbdev;
#endif
	struct drm_framebuffer * psDrmFB;
	struct psb_framebuffer *psbfb;


	int hdisplay;
	int vdisplay;
	int i;
	unsigned long FBSize;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	psDrmFB = psPsbFBDev->psb_fb_helper.fb;
#else
	psDrmFB = list_first_entry(&psDrmDevice->mode_config.fb_kernel_list,
				   struct drm_framebuffer,
				   filp_head);
#endif
	if(!psDrmFB) {
		printk(KERN_INFO"%s: Cannot find drm FB",__FUNCTION__);
		return eError;
	}
	psbfb = to_psb_fb(psDrmFB);

	hdisplay = psDrmFB->width;
	vdisplay = psDrmFB->height;
	FBSize = psDrmFB->MEMBER_PITCH * psDrmFB->height;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	psLINFBInfo = (struct fb_info*)psPsbFBDev->psb_fb_helper.fbdev;
#else
	psLINFBInfo = (struct fb_info*)psDrmFB->fbdev;
#endif

#if defined(PVR_MRST_FB_SET_PAR_ON_INIT)
	MRSTFBSetPar(psLINFBInfo);
#endif


	psDevInfo->sSystemBuffer.bIsContiguous = MRST_TRUE;
	psDevInfo->sSystemBuffer.bIsAllocated = MRST_FALSE;

	MRSTLFBHandleChangeFB(psDrmDevice, psbfb);

	switch( psDrmFB->depth )
	{
	case 32:
	case 24:
		{
			psDevInfo->sDisplayFormat.pixelformat = PVRSRV_PIXEL_FORMAT_ARGB8888;
			break;
		}
	case 16:
		{
			psDevInfo->sDisplayFormat.pixelformat = PVRSRV_PIXEL_FORMAT_RGB565;
			break;
		}
	default:
		{
			printk(KERN_ERR"%s: Unknown bit depth %d\n",__FUNCTION__,psDrmFB->depth);
		}
	}
	psDevInfo->psLINFBInfo = psLINFBInfo;

	psDevInfo->ui32MainPipe = MRSTLFBFindMainPipe(psDevInfo->psDrmDevice);

	for(i = 0;i < MAX_SWAPCHAINS;++i)
	{
		psDevInfo->apsSwapChains[i] = NULL;
	}




	psDevInfo->pvRegs = psbfb_vdc_reg(psDevInfo->psDrmDevice);

	if (psDevInfo->pvRegs == NULL)
	{
		eError = PVRSRV_ERROR_BAD_MAPPING;
		printk(KERN_WARNING DRIVER_PREFIX ": Couldn't map registers needed for flipping\n");
		return eError;
	}

	return MRST_OK;
}

IMG_VOID MRSTQuerySwapCommand(IMG_HANDLE hDev, IMG_HANDLE hSwap, IMG_HANDLE hBuffer, IMG_HANDLE hTag, IMG_UINT16* ID, IMG_BOOL* bAddRef)
{
	UNREFERENCED_PARAMETER(hDev);
	UNREFERENCED_PARAMETER(hSwap);
	UNREFERENCED_PARAMETER(hBuffer);
	UNREFERENCED_PARAMETER(hTag);
	UNREFERENCED_PARAMETER(ID);
	*bAddRef = IMG_FALSE;
}


MRST_ERROR MRSTLFBInit(struct drm_device * dev)
{

	MRSTLFB_DEVINFO		*psDevInfo;
#ifndef DRM_PVR_USE_INTEL_FB
	struct drm_psb_private *psDrmPriv = (struct drm_psb_private *)dev->dev_private;
#endif

	psDevInfo = GetAnchorPtr();

	if (psDevInfo == NULL)
	{
		PFN_CMD_PROC	 		pfnCmdProcList[MRSTLFB_COMMAND_COUNT];
		IMG_UINT32				aui32SyncCountList[MRSTLFB_COMMAND_COUNT][2];

		psDevInfo = (MRSTLFB_DEVINFO *)MRSTLFBAllocKernelMem(sizeof(MRSTLFB_DEVINFO));

		if(!psDevInfo)
		{
			return (MRST_ERROR_OUT_OF_MEMORY);
		}


		memset(psDevInfo, 0, sizeof(MRSTLFB_DEVINFO));


		SetAnchorPtr((void*)psDevInfo);

		psDevInfo->psDrmDevice = dev;
		psDevInfo->ulRefCount = 0;

		if(InitDev(psDevInfo) != MRST_OK)
		{
			return (MRST_ERROR_INIT_FAILURE);
		}

		if(MRSTLFBGetLibFuncAddr ("PVRGetDisplayClassJTable", &pfnGetPVRJTable) != MRST_OK)
		{
			return (MRST_ERROR_INIT_FAILURE);
		}


		if(!(*pfnGetPVRJTable)(&psDevInfo->sPVRJTable))
		{
			return (MRST_ERROR_INIT_FAILURE);
		}


		spin_lock_init(&psDevInfo->sSwapChainLock);

		psDevInfo->psCurrentSwapChain = NULL;
		psDevInfo->bFlushCommands = MRST_FALSE;

		psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers = 4;
		psDevInfo->sDisplayInfo.ui32MaxSwapChains = MAX_SWAPCHAINS;
		psDevInfo->sDisplayInfo.ui32MaxSwapInterval = 3;
		psDevInfo->sDisplayInfo.ui32MinSwapInterval = 0;

		strncpy(psDevInfo->sDisplayInfo.szDisplayName, DISPLAY_DEVICE_NAME, MAX_DISPLAY_NAME_SIZE);




		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX
			": Maximum number of swap chain buffers: %u\n",
			psDevInfo->sDisplayInfo.ui32MaxSwapChainBuffers));





		psDevInfo->sDCJTable.ui32TableSize = sizeof(PVRSRV_DC_SRV2DISP_KMJTABLE);
		psDevInfo->sDCJTable.pfnOpenDCDevice = OpenDCDevice;
		psDevInfo->sDCJTable.pfnCloseDCDevice = CloseDCDevice;
		psDevInfo->sDCJTable.pfnEnumDCFormats = EnumDCFormats;
		psDevInfo->sDCJTable.pfnEnumDCDims = EnumDCDims;
		psDevInfo->sDCJTable.pfnGetDCSystemBuffer = GetDCSystemBuffer;
		psDevInfo->sDCJTable.pfnGetDCInfo = GetDCInfo;
		psDevInfo->sDCJTable.pfnGetBufferAddr = GetDCBufferAddr;
		psDevInfo->sDCJTable.pfnCreateDCSwapChain = CreateDCSwapChain;
		psDevInfo->sDCJTable.pfnDestroyDCSwapChain = DestroyDCSwapChain;
		psDevInfo->sDCJTable.pfnSetDCDstRect = SetDCDstRect;
		psDevInfo->sDCJTable.pfnSetDCSrcRect = SetDCSrcRect;
		psDevInfo->sDCJTable.pfnSetDCDstColourKey = SetDCDstColourKey;
		psDevInfo->sDCJTable.pfnSetDCSrcColourKey = SetDCSrcColourKey;
		psDevInfo->sDCJTable.pfnGetDCBuffers = GetDCBuffers;
		psDevInfo->sDCJTable.pfnSwapToDCBuffer = SwapToDCBuffer;
		psDevInfo->sDCJTable.pfnSetDCState = SetDCState;
		psDevInfo->sDCJTable.pfnGetDCFrontBuffer = GetDCFrontBuffer;
		psDevInfo->sDCJTable.pfnQuerySwapCommandID = MRSTQuerySwapCommand;


		if(psDevInfo->sPVRJTable.pfnPVRSRVRegisterDCDevice (
			&psDevInfo->sDCJTable,
			&psDevInfo->uiDeviceID ) != PVRSRV_OK)
		{
			return (MRST_ERROR_DEVICE_REGISTER_FAILED);
		}

		printk("Device ID: %d\n", (int)psDevInfo->uiDeviceID);











#if defined (MRST_USING_INTERRUPTS)

	if(MRSTLFBInstallVSyncISR(psDevInfo,MRSTLFBVSyncISR) != MRST_OK)
	{
		DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX	"ISR Installation failed\n"));
		return (MRST_ERROR_INIT_FAILURE);
	}
#endif


	pfnCmdProcList[DC_FLIP_COMMAND] = ProcessFlip;


	aui32SyncCountList[DC_FLIP_COMMAND][0] = 0;
	aui32SyncCountList[DC_FLIP_COMMAND][1] = 10;





	if (psDevInfo->sPVRJTable.pfnPVRSRVRegisterCmdProcList (psDevInfo->uiDeviceID,
								&pfnCmdProcList[0],
								aui32SyncCountList,
								MRSTLFB_COMMAND_COUNT) != PVRSRV_OK)
	  {
	    printk(KERN_WARNING DRIVER_PREFIX ": Can't register callback\n");
	    return (MRST_ERROR_CANT_REGISTER_CALLBACK);
	  }


	}


#ifndef DRM_PVR_USE_INTEL_FB
	psDrmPriv->psb_change_fb_handler = MRSTLFBHandleChangeFB;

	psDrmPriv->psb_leave_vt_handler = DRMLFBLeaveVTHandler;
	psDrmPriv->psb_enter_vt_handler = DRMLFBEnterVTHandler;
#endif
    MRSTLFBInstallScreenEvents(psDevInfo, MRSTLFBScreenEventHandler);

	psDevInfo->ulRefCount++;

	psDevInfo->sFlipTimer.data = (unsigned long)psDevInfo;
	psDevInfo->sFlipTimer.function = MRSTLFBFlipTimerFn;
	init_timer(&psDevInfo->sFlipTimer);
	return (MRST_OK);
}

MRST_ERROR MRSTLFBDeinit(void)
{
	MRSTLFB_DEVINFO *psDevInfo, *psDevFirst;

	psDevFirst = GetAnchorPtr();
	psDevInfo = psDevFirst;


	if (psDevInfo == NULL)
	{
		return (MRST_ERROR_GENERIC);
	}


	psDevInfo->ulRefCount--;

	if (psDevInfo->ulRefCount == 0)
	{

		PVRSRV_DC_DISP2SRV_KMJTABLE	*psJTable = &psDevInfo->sPVRJTable;

		if (psDevInfo->sPVRJTable.pfnPVRSRVRemoveCmdProcList (psDevInfo->uiDeviceID, MRSTLFB_COMMAND_COUNT) != PVRSRV_OK)
		{
			return (MRST_ERROR_GENERIC);
		}

#if defined (MRST_USING_INTERRUPTS)

		if(MRSTLFBUninstallVSyncISR(psDevInfo) != MRST_OK)
		{
			return (MRST_ERROR_GENERIC);
		}
#endif


		if (psJTable->pfnPVRSRVRemoveDCDevice(psDevInfo->uiDeviceID) != PVRSRV_OK)
		{
			return (MRST_ERROR_GENERIC);
		}


		MRSTLFBFreeKernelMem(psDevInfo);
	}


	SetAnchorPtr(NULL);


	return (MRST_OK);
}



MRST_ERROR MRSTLFBAllocBuffer(struct MRSTLFB_DEVINFO_TAG *psDevInfo, IMG_UINT32 ui32Size, MRSTLFB_BUFFER **ppBuffer)
{
	IMG_VOID *pvBuf;
	IMG_UINT32 ulPagesNumber;
	IMG_UINT32 ulCounter;
	int i;

	pvBuf = __vmalloc( ui32Size, GFP_KERNEL | __GFP_HIGHMEM, __pgprot((pgprot_val(PAGE_KERNEL ) & ~_PAGE_CACHE_MASK) | _PAGE_CACHE_WC) );
	if( pvBuf == NULL )
	{
		return MRST_ERROR_OUT_OF_MEMORY;
	}

	ulPagesNumber = (ui32Size + PAGE_SIZE -1) / PAGE_SIZE;

	*ppBuffer = MRSTLFBAllocKernelMem( sizeof( MRSTLFB_BUFFER ) );
	(*ppBuffer)->sCPUVAddr = pvBuf;
	(*ppBuffer)->ui32BufferSize = ui32Size;
	(*ppBuffer)->uSysAddr.psNonCont = MRSTLFBAllocKernelMem( sizeof( IMG_SYS_PHYADDR ) * ulPagesNumber);
	(*ppBuffer)->bIsAllocated = MRST_TRUE;
	(*ppBuffer)->bIsContiguous = MRST_FALSE;
	(*ppBuffer)->ui32OwnerTaskID = task_tgid_nr(current);

	i = 0;
	for (ulCounter = 0; ulCounter < ui32Size; ulCounter += PAGE_SIZE)
	{
		(*ppBuffer)->uSysAddr.psNonCont[i++].uiAddr = vmalloc_to_pfn( pvBuf + ulCounter ) << PAGE_SHIFT;
	}

	psb_gtt_map_pvr_memory( psDevInfo->psDrmDevice,
							(unsigned int)*ppBuffer,
							(*ppBuffer)->ui32OwnerTaskID,
							(IMG_CPU_PHYADDR*) (*ppBuffer)->uSysAddr.psNonCont,
							ulPagesNumber,
							&(*ppBuffer)->sDevVAddr.uiAddr );

	(*ppBuffer)->sDevVAddr.uiAddr <<= PAGE_SHIFT;

   	return MRST_OK;
}

MRST_ERROR MRSTLFBFreeBuffer(struct MRSTLFB_DEVINFO_TAG *psDevInfo, MRSTLFB_BUFFER **ppBuffer)
{
	if( !(*ppBuffer)->bIsAllocated )
		return MRST_ERROR_INVALID_PARAMS;

#ifndef DRM_PVR_USE_INTEL_FB
	psb_gtt_unmap_memory( psDevInfo->psDrmDevice,
#else
	psb_gtt_unmap_pvr_memory( psDevInfo->psDrmDevice,
#endif
							  (unsigned int)*ppBuffer,
							  (*ppBuffer)->ui32OwnerTaskID);

	vfree( (*ppBuffer)->sCPUVAddr );

	MRSTLFBFreeKernelMem( (*ppBuffer)->uSysAddr.psNonCont );

	MRSTLFBFreeKernelMem( *ppBuffer);

	*ppBuffer = NULL;

	return MRST_OK;
}




