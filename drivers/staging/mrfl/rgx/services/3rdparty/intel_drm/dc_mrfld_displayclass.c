/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
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
#include "img_defs.h"
#include "servicesext.h"
#include "allocmem.h"
#include "kerneldisplay.h"
#include "dc_mrfld.h"
#include "mm.h"
#include "pvrsrv_error.h"
#include "displayclass_interface.h"
#include "display_callbacks.h"
#include "dc_server.h"

#if !defined(SUPPORT_DRI_DRM)
#error "SUPPORT_DRI_DRM must be set"
#endif

/*
	DSR
*/
extern int drm_psb_dsr;
extern struct drm_device *gpDrmDevice;
extern void ospm_suspend_display(struct drm_device *dev);
extern void ospm_resume_display(struct pci_dev *pdev);

static void dsr_wq_handler(struct work_struct *work);
DECLARE_DELAYED_WORK(dsr_work, dsr_wq_handler);

void dsr_wq_handler(struct work_struct *work)
{
	ospm_suspend_display(gpDrmDevice);
}

static void dsrexit_wq_handler(struct work_struct *work);
DECLARE_DELAYED_WORK(dsr_exitwork, dsrexit_wq_handler);

void dsrexit_wq_handler(struct work_struct *work)
{
	ospm_resume_display(gpDrmDevice->pdev);
}

static IMG_HANDLE gpvAnchor = NULL;

#define MRSTLFB_COMMAND_COUNT		1

static MRFLD_DEVINFO *GetAnchorPtr(void)
{
	return (MRFLD_DEVINFO *) gpvAnchor;
}

static void MRSTLFBFlip(MRFLD_DEVINFO * psDevInfo, MRFLD_BUFFER * psBuffer)
{
	unsigned long ulAddr = (unsigned long)psBuffer->sDevVAddr.uiAddr;
	MRSTLFB_SWAPCHAIN *psCurrentSwapChain = psDevInfo->psCurrentSwapChain;
	unsigned int pipeflag = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_NONE;
	struct psb_framebuffer *psbfb;

	DCCBGetFramebuffer(psDevInfo->psDrmDevice, &psbfb);

	if (psCurrentSwapChain)
		pipeflag = psCurrentSwapChain->ui32SwapChainPropertyFlag;

	if (!psDevInfo->bSuspended) {
		DCCBFlipToSurface(psDevInfo->psDrmDevice, ulAddr, pipeflag);
	}

	psDevInfo->ulLastFlipAddr = ulAddr;
	psDevInfo->bLastFlipAddrValid = IMG_TRUE;

	psbfb->fbdev->screen_base = psBuffer->sCPUVAddr;
}

static void FlushInternalVSyncQueue(MRSTLFB_SWAPCHAIN * psSwapChain,
				    IMG_BOOL bFlip)
{
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
	unsigned long ulMaxIndex;
	unsigned long i;

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulSwapChainLength - 1;

	for (i = 0; i < psSwapChain->ulSwapChainLength; i++) {
		if (psFlipItem->bValid == IMG_FALSE) {
			continue;
		}

		DRM_DEBUG
		    ("FlushInternalVSyncQueue: Flushing swap buffer (index %lu)\n",
		     psSwapChain->ulRemoveIndex);

		if (psFlipItem->bFlipped == IMG_FALSE) {
			if (bFlip) {

				MRSTLFBFlip(psSwapChain->psDevInfo,
					    psFlipItem->psBuffer);
			}
		}

		if (psFlipItem->bCmdCompleted == IMG_FALSE) {
			DRM_DEBUG
			    ("FlushInternalVSyncQueue: Calling command complete for swap buffer (index %lu)\n",
			     psSwapChain->ulRemoveIndex);

//                      psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete((IMG_HANDLE)psFlipItem->hCmdComplete, IMG_FALSE, IMG_TRUE);
		}

		psSwapChain->ulRemoveIndex++;

		if (psSwapChain->ulRemoveIndex > ulMaxIndex) {
			psSwapChain->ulRemoveIndex = 0;
		}

		psFlipItem->bFlipped = IMG_FALSE;
		psFlipItem->bCmdCompleted = IMG_FALSE;
		psFlipItem->bValid = IMG_FALSE;

		psFlipItem =
		    &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	}

	psSwapChain->ulInsertIndex = 0;
	psSwapChain->ulRemoveIndex = 0;
}

static void DRMLFBFlipBuffer(MRFLD_DEVINFO * psDevInfo,
			     MRSTLFB_SWAPCHAIN * psSwapChain,
			     MRFLD_BUFFER * psBuffer)
{
	if (psSwapChain != NULL) {
		if (psDevInfo->psCurrentSwapChain != NULL) {

			if (psDevInfo->psCurrentSwapChain != psSwapChain)
				FlushInternalVSyncQueue(psDevInfo->
							psCurrentSwapChain,
							IMG_FALSE);
		}
		psDevInfo->psCurrentSwapChain = psSwapChain;
	}

	MRSTLFBFlip(psDevInfo, psBuffer);
}

static void SetFlushStateNoLock(MRFLD_DEVINFO * psDevInfo, IMG_BOOL bFlushState)
{
	if (bFlushState) {
		if (psDevInfo->ulSetFlushStateRefCount == 0) {
			psDevInfo->bFlushCommands = IMG_TRUE;
			if (psDevInfo->psCurrentSwapChain != NULL) {
				FlushInternalVSyncQueue(psDevInfo->
							psCurrentSwapChain,
							IMG_TRUE);
			}
		}
		psDevInfo->ulSetFlushStateRefCount++;
	} else {
		if (psDevInfo->ulSetFlushStateRefCount != 0) {
			psDevInfo->ulSetFlushStateRefCount--;
			if (psDevInfo->ulSetFlushStateRefCount == 0) {
				psDevInfo->bFlushCommands = IMG_FALSE;
			}
		}
	}
}

static IMG_VOID SetFlushState(MRFLD_DEVINFO * psDevInfo, IMG_BOOL bFlushState)
{
	unsigned long ulLockFlags;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	SetFlushStateNoLock(psDevInfo, bFlushState);

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
}

static int FrameBufferEvents(struct notifier_block *psNotif,
			     unsigned long event, void *data)
{
	MRFLD_DEVINFO *psDevInfo;
	struct fb_event *psFBEvent = (struct fb_event *)data;
	IMG_BOOL bBlanked;

	if (event != FB_EVENT_BLANK) {
		return 0;
	}

	psDevInfo = GetAnchorPtr();

	bBlanked = (*(IMG_INT *) psFBEvent->data != 0) ? IMG_TRUE : IMG_FALSE;

	if (bBlanked != psDevInfo->bBlanked) {
		psDevInfo->bBlanked = bBlanked;

		SetFlushState(psDevInfo, bBlanked);
	}

	return 0;
}

static PVRSRV_ERROR EnableLFBEventNotification(MRFLD_DEVINFO * psDevInfo)
{
	int res;

	memset(&psDevInfo->sLINNotifBlock, 0,
	       sizeof(psDevInfo->sLINNotifBlock));

	psDevInfo->sLINNotifBlock.notifier_call = FrameBufferEvents;
	psDevInfo->bBlanked = IMG_FALSE;

	res = fb_register_client(&psDevInfo->sLINNotifBlock);
	if (res != 0) {
		DRM_ERROR("fb_register_client failed (%d)", res);

		return (PVRSRV_ERROR_UNABLE_TO_OPEN_DC_DEVICE);
	}

	DCCBUnblankDisplay(psDevInfo->psDrmDevice);

	return (PVRSRV_OK);
}

static PVRSRV_ERROR DisableLFBEventNotification(MRFLD_DEVINFO * psDevInfo)
{
	int res;

	res = fb_unregister_client(&psDevInfo->sLINNotifBlock);
	if (res != 0) {
		DRM_ERROR("fb_unregister_client failed (%d)", res);
		return (PVRSRV_ERROR_UNABLE_TO_UNREGISTER_DEVICE);
	}

	return (PVRSRV_OK);
}

static IMG_VOID DCMrfldGetDCInfo(IMG_HANDLE hDevice, DC_DISPLAY_INFO * psDCInfo)
{
	if (hDevice && psDCInfo) {
		MRFLD_DEVINFO *psDevInfo = (MRFLD_DEVINFO *) hDevice;

		*psDCInfo = psDevInfo->sDisplayInfo;
	}
}

static PVRSRV_ERROR DCMrfldPanelQueryCount(IMG_HANDLE hDevice,
					   IMG_UINT32 * pui32NumPannels)
{
	if (!hDevice || !pui32NumPannels) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/*
	   If you know the panel count at compile time just hardcode it, if it's
	   dynamic you should probe it here
	 */
	*pui32NumPannels = 1;

	return PVRSRV_OK;
}

static PVRSRV_ERROR DCMrfldPanelQuery(IMG_HANDLE hDeviceData,
				      IMG_UINT32 ui32PanelsArraySize,
				      IMG_UINT32 * pui32NumPannels,
				      PVRSRV_SURFACE_INFO * psSurfInfo)
{
	MRFLD_DEVINFO *psDevInfo;

	if (!hDeviceData || !pui32NumPannels || !psSurfInfo) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDeviceData;

	/*
	   If we have hotplug displays then there is a chance a display could
	   have been removed so return the number of panels we have queryed
	 */
	*pui32NumPannels = 1;

	/*
	   Either hard code the values here or probe each panel here. If a new
	   panel has been hotpluged then ignore it as we've not been given
	   room to store it's data
	 */
	psSurfInfo[0] = psDevInfo->sPrimInfo;

	return PVRSRV_OK;
}

static PVRSRV_ERROR DCMrfldFormatQuery(IMG_HANDLE hDeviceData,
				       IMG_UINT32 ui32NumFormats,
				       PVRSRV_SURFACE_FORMAT * pasFormat,
				       IMG_UINT32 * pui32Supported)
{
	IMG_UINT32 i;

	PVR_UNREFERENCED_PARAMETER(hDeviceData);

	for (i = 0; i < ui32NumFormats; i++) {
		pui32Supported[i] = 0;

		/*
		   If the display controller has multiple display pipes (DMA engines)
		   each one should be checked to see if it supports the specified
		   format.
		 */
		if (pasFormat[i].ePixFormat == IMG_PIXFMT_B8G8R8A8_UNORM) {
			pui32Supported[i] = 1;
		} else if (pasFormat[i].ePixFormat == IMG_PIXFMT_B5G6R5_UNORM) {
			pui32Supported[i] = 1;
		}
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR DCMrfldDimQuery(IMG_HANDLE hDeviceData,
				    IMG_UINT32 ui32NumDims,
				    PVRSRV_SURFACE_DIMS * psDim,
				    IMG_UINT32 * pui32Supported)
{
	IMG_UINT32 i;
	MRFLD_DEVINFO *psDevInfo;

	if (!hDeviceData || !psDim || !pui32Supported) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDeviceData;

	PVR_UNREFERENCED_PARAMETER(hDeviceData);

	for (i = 0; i < ui32NumDims; i++) {
		pui32Supported[i] = 0;

		/*
		   If the display controller has multiple display pipes (DMA engines)
		   each one should be checked to see if it supports the specified
		   dimension.
		 */
		if ((psDim[i].ui32Width == psDevInfo->sPrimInfo.sDims.ui32Width)
		    && (psDim[i].ui32Height ==
			psDevInfo->sPrimInfo.sDims.ui32Height)) {
			pui32Supported[i] = 1;
		}
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR DCMrfldContextCreate(IMG_HANDLE hDeviceData,
					 IMG_HANDLE * hDisplayContext)
{
	MRFLD_DEVINFO *psDevInfo;

	if (!hDeviceData || !hDisplayContext) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDeviceData;

	/*
	   The creation of a display context is a software concept and
	   it's an "agreement" between the services client and the DC driver
	   as to what this means (if anything)
	 */
	if (gpvAnchor == hDeviceData) {
		*hDisplayContext = hDeviceData;

		psDevInfo->ulRefCount++;
		return PVRSRV_OK;
	}

	DRM_ERROR("DC resource unavailable");
	return PVRSRV_ERROR_RESOURCE_UNAVAILIBLE;
}

static IMG_VOID DCMrfldContextDestroy(IMG_HANDLE hDisplayContext)
{
	MRFLD_DEVINFO *psDevInfo;

	if (!hDisplayContext) {
		return;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDisplayContext;

	/*
	   Counter part to ContextCreate. Any buffers created/imported
	   on this display context will have been freed before this call
	   so all the display driver needs to do is release any resources
	   allocated at ContextCreate time.
	 */
	psDevInfo->ulRefCount--;
}

static IMG_VOID DCMrfldContextConfigure(IMG_HANDLE hDisplayContext,
					IMG_UINT32 ui32PipeCount,
					PVRSRV_SURFACE_CONFIG_INFO *
					pasSurfAttrib, IMG_HANDLE * ahBuffers,
					IMG_UINT32 ui32DisplayPeriod,
					IMG_HANDLE hConfigData)
{
#if 1

	MRFLD_DEVINFO *psDevInfo = (MRFLD_DEVINFO *) hDisplayContext;

	MRFLD_BUFFER *psBuffer = (MRFLD_BUFFER *) (ahBuffers[0]);

	MRSTLFBFlip(psDevInfo, psBuffer);

	return;
#endif
#if 0
	unsigned long ulAddr = (unsigned long)psBuffer->sDevVAddr.uiAddr;

	DRM_ERROR("%s %s %d\n", __FILE__, __func__, __LINE__);

	unsigned int pipeflag = PVRSRV_SWAPCHAIN_ATTACHED_PLANE_NONE;

	struct psb_framebuffer *psbfb;

	if (!psDevInfo->bSuspended) {
		DRM_ERROR("%s %s %d\n", __FILE__, __func__, __LINE__);
		DCCBFlipToSurface(psDevInfo->psDrmDevice, ulAddr, pipeflag);
	}

	DRM_ERROR("%s %s %d\n", __FILE__, __func__, __LINE__);

	psDevInfo->ulLastFlipAddr = ulAddr;
	psDevInfo->bLastFlipAddrValid = IMG_TRUE;

	psbfb->fbdev->screen_base = psBuffer->sCPUVAddr;

	return;
#endif
}

static PVRSRV_ERROR DCMrfldContextConfigureCheck(IMG_HANDLE hDisplayContext,
						 IMG_UINT32 ui32PipeCount,
						 PVRSRV_SURFACE_CONFIG_INFO *
						 pasSurfAttrib,
						 IMG_HANDLE * ahBuffers)
{
	IMG_UINT32 i;
	IMG_BOOL bFail = IMG_FALSE;
	MRFLD_DEVINFO *psDevInfo;

	if (!hDisplayContext) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDisplayContext;

	/*
	   This optional function allows the display driver to check if the display
	   configuration passed in is valid.
	   It's possible that due to HW contraints that although the client has
	   honoured the DimQuery and FormatQuery results the configuration it
	   has requested is still not possible (e.g. there isn't enough space in
	   the display controllers's MMU, or due restrictions on display pipes.
	 */

	for (i = 0; i < ui32PipeCount; i++) {
		PVRSRV_SURFACE_INFO *psSurfInfo = &pasSurfAttrib[i].sSurface;

		if (psSurfInfo->sFormat.ePixFormat != IMG_PIXFMT_B8G8R8A8_UNORM
		    && psSurfInfo->sFormat.ePixFormat !=
		    IMG_PIXFMT_B5G6R5_UNORM) {
			bFail = IMG_TRUE;
		}
		if (psSurfInfo->sDims.ui32Width !=
		    psDevInfo->sPrimInfo.sDims.ui32Width) {
			bFail = IMG_TRUE;
		}
		if (psSurfInfo->sDims.ui32Height !=
		    psDevInfo->sPrimInfo.sDims.ui32Height) {
			bFail = IMG_TRUE;
		}
		if (pasSurfAttrib[i].ui32XOffset != 0) {
			bFail = IMG_TRUE;
		}
		if (pasSurfAttrib[i].ui32YOffset != 0) {
			bFail = IMG_TRUE;
		}

		if (bFail) {
			break;
		}
	}

	if (bFail) {
		return PVRSRV_ERROR_INVALID_CONFIG;
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR DCMrfldBufferAlloc(IMG_HANDLE hDisplayContext,
				       DC_BUFFER_CREATE_INFO * psCreateInfo,
				       IMG_DEVMEM_LOG2ALIGN_T * puiLog2PageSize,
				       IMG_UINT32 * pui32PageCount,
				       IMG_UINT32 * pui32PhysHeapID,
				       IMG_UINT32 * pui32Stride,
				       IMG_HANDLE * phBuffer)
{
	MRFLD_BUFFER *psBuffer = NULL;
	PVRSRV_SURFACE_INFO *psSurfInfo = &psCreateInfo->sSurface;
	IMG_UINT32 ulPagesNumber, i, ulCounter;
	MRFLD_DEVINFO *psDevInfo;

	if (!hDisplayContext) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDisplayContext;

	/*
	   Allocate the buffer control structure
	 */
	psBuffer = OSAllocMem(sizeof(MRFLD_BUFFER));
	if (psBuffer == NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	/*
	   As we're been asked to allocate this buffer we decide what it's
	   stride should be.
	 */
	psBuffer->hDisplayContext = psDevInfo;
	psBuffer->ui32ByteStride =
	    psSurfInfo->sDims.ui32Width * psCreateInfo->ui32BPP;
	psBuffer->ui32Width = psSurfInfo->sDims.ui32Width;
	psBuffer->ui32Height = psSurfInfo->sDims.ui32Height;
	psBuffer->ui32BufferSize =
	    psBuffer->ui32Height * psBuffer->ui32ByteStride;
	psBuffer->ePixFormat = psSurfInfo->sFormat.ePixFormat;

	/*
	   Allocate display adressable memory. We only need physcial addresses
	   at this stage.

	   Note: This could be defered till the 1st map or acquire call.
	   IMG uses pgprot_noncached(PAGE_KERNEL)
	 */
	psBuffer->sCPUVAddr = __vmalloc(psBuffer->ui32BufferSize,
					GFP_KERNEL | __GFP_HIGHMEM,
					__pgprot((pgprot_val(PAGE_KERNEL) &
						  ~_PAGE_CACHE_MASK) |
						 _PAGE_CACHE_WC));
	if (psBuffer->sCPUVAddr == NULL) {
		OSFreeMem(psBuffer);
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	ulPagesNumber = psBuffer->ui32BufferSize >> PAGE_SHIFT;
	psBuffer->psSysAddr =
	    OSAllocMem(sizeof(IMG_SYS_PHYADDR) * ulPagesNumber);
	if (psBuffer->psSysAddr == NULL) {
		vfree(psBuffer->sCPUVAddr);
		OSFreeMem(psBuffer);
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	i = 0;
	for (ulCounter = 0; ulCounter < psBuffer->ui32BufferSize;
	     ulCounter += PAGE_SIZE) {
		psBuffer->psSysAddr[i++].uiAddr =
		    vmalloc_to_pfn(psBuffer->sCPUVAddr +
				   ulCounter) << PAGE_SHIFT;
	}

	psBuffer->bIsAllocated = IMG_TRUE;
	psBuffer->bIsContiguous = IMG_FALSE;
	psBuffer->ui32OwnerTaskID = task_tgid_nr(current);

	DCCBgttMapMemory(psDevInfo->psDrmDevice,
			 (unsigned int)psBuffer,
			 psBuffer->ui32OwnerTaskID,
			 (uintptr_t *) psBuffer->psSysAddr,
			 ulPagesNumber,
			 (unsigned int *)&psBuffer->sDevVAddr.uiAddr);

	psBuffer->sDevVAddr.uiAddr <<= PAGE_SHIFT;

	*pui32Stride = psBuffer->ui32ByteStride;
	*puiLog2PageSize = PAGE_SHIFT;
	*pui32PageCount = ulPagesNumber;
	*pui32PhysHeapID = 0;
	*phBuffer = psBuffer;

	DRM_INFO("Allocate buffer (%p)\n", psBuffer);
	return PVRSRV_OK;
}

static PVRSRV_ERROR DCMrfldBufferAcquire(IMG_HANDLE hBuffer,
					 IMG_DEV_PHYADDR * pasDevPAddr,
					 IMG_PVOID * ppvLinAddr)
{
	MRFLD_BUFFER *psBuffer = hBuffer;
	IMG_UINT32 ulPages =
	    (psBuffer->ui32BufferSize + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
	IMG_UINT32 i;

	/*
	   If we didn't allocate the display memory at buffer alloc time
	   we would have to do it here.
	 */

	/*
	   Fill in the array of addresses we where passed
	 */
	for (i = 0; i < ulPages; i++) {
		pasDevPAddr[i].uiAddr = psBuffer->psSysAddr[i].uiAddr;
	}
	*ppvLinAddr = psBuffer->sCPUVAddr;

	DRM_INFO("Acquire buffer (%p), ph=%x, pc=%d\n", psBuffer,
		 psBuffer->psSysAddr[0].uiAddr, ulPages);
	return PVRSRV_OK;
}

static IMG_VOID DCMrfldBufferRelease(IMG_HANDLE hBuffer)
{
	MRFLD_BUFFER *psBuffer = hBuffer;
	/*
	   We could release the display memory here (assuming it wasn't
	   still mapped into the display controller).

	   As the buffer hasn't been freed the contents must be preserved, i.e.
	   in the next call to Acquire different physcial pages can be returned,
	   but they must have the same contents as the old pages had at Release
	   time.
	 */
	DRM_INFO("Release buffer (%p) memory\n", psBuffer);
}

static IMG_VOID DCMrfldBufferFree(IMG_HANDLE hBuffer)
{
	MRFLD_BUFFER *psBuffer = hBuffer;
	MRFLD_DEVINFO *psDevInfo;

	if (!hBuffer) {
		return;
	}

	DRM_DEBUG("Free buffer (%p)\n", psBuffer);

	if (!psBuffer->bIsAllocated)
		return;

	psDevInfo = (MRFLD_DEVINFO *) psBuffer->hDisplayContext;

	DCCBgttUnmapMemory(psDevInfo->psDrmDevice,
			   (unsigned int)psBuffer, psBuffer->ui32OwnerTaskID);

	vfree(psBuffer->sCPUVAddr);

	OSFreeMem(psBuffer->psSysAddr);

	OSFreeMem(psBuffer);
}

static PVRSRV_ERROR DCMrfldBufferImport(IMG_HANDLE hDisplayContext,
					IMG_UINT32 ui32NumPlanes,
					IMG_HANDLE ** paphImport,
					DC_BUFFER_IMPORT_INFO * psSurfAttrib,
					IMG_HANDLE * phBuffer)
{
	/*
	   This it optional and should only be provided if the display controller
	   can access "general" memory (e.g. the memory doesn't have to contiguesus)
	 */
	MRFLD_BUFFER *psBuffer = NULL;
	MRFLD_DEVINFO *psDevInfo;

	if (!hDisplayContext) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDisplayContext;

	/*
	   Check to see if our display hardware supports this buffer
	 */
	if ((psSurfAttrib->ePixFormat != IMG_PIXFMT_B8G8R8A8_UNORM &&
	     psSurfAttrib->ePixFormat != IMG_PIXFMT_B5G6R5_UNORM) ||
	    (psSurfAttrib->ui32Width[0] != psDevInfo->sPrimInfo.sDims.ui32Width)
	    || (psSurfAttrib->ui32Height[0] !=
		psDevInfo->sPrimInfo.sDims.ui32Height)) {
		return PVRSRV_ERROR_UNSUPPORTED_PIXEL_FORMAT;
	}

	psBuffer = OSAllocMem(sizeof(MRFLD_BUFFER));
	if (psBuffer == NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	psBuffer->ui32Width = psSurfAttrib->ui32Width[0];
	psBuffer->ePixFormat = psSurfAttrib->ePixFormat;
	psBuffer->ui32ByteStride = psSurfAttrib->ui32Stride[0];
	psBuffer->ui32Width = psSurfAttrib->ui32Width[0];
	psBuffer->ui32Height = psSurfAttrib->ui32Height[0];

	/*
	   If the display controller supports mapping "general" memory, but has
	   limitations (e.g. if it doesn't have full range addressing) these
	   should be checked here by calling DCImportBufferAcquire. In this case
	   it lock down the physcial address of the buffer at this stange rather
	   then being able to defer it to map time.
	 */
	psBuffer->hImport = paphImport[0];

	*phBuffer = psBuffer;

	DRM_DEBUG("Import buffer (%p)\n", psBuffer);
	return PVRSRV_OK;
}

static PVRSRV_ERROR DCMrfldBufferMap(IMG_HANDLE hBuffer)
{
	DRM_ERROR("To be implemented: %s", __func__);
	return PVRSRV_OK;
}

static IMG_VOID DCMrfldBufferUnmap(IMG_HANDLE hBuffer)
{
	DRM_ERROR("To be implemented: %s", __func__);
}

static PVRSRV_ERROR DCMrfldBufferSystemAcquire(IMG_HANDLE hDeviceData,
					       IMG_DEVMEM_LOG2ALIGN_T *
					       puiLog2PageSize,
					       IMG_UINT32 * pui32PageCount,
					       IMG_UINT32 * pui32PhysHeapID,
					       IMG_UINT32 * pui32Stride,
					       IMG_HANDLE * phSystemBuffer)
{
	MRFLD_DEVINFO *psDevInfo;

	if (!hDeviceData) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psDevInfo = (MRFLD_DEVINFO *) hDeviceData;

	/*
	   This function is optionial. It provides a method for services
	   to acquire a display buffer which it didn't setup but was created
	   by the OS (e.g. Linux frame buffer).
	   If the OS should trigger a mode change then it's not allowed to free
	   the previous buffer until services has released it via BufferSystemRelease
	 */

	/*
	   Take a reference to the system buffer 1st to make sure it isn't freed
	 */
	psDevInfo->sSystemBuffer.ui32RefCount++;

	*puiLog2PageSize = PAGE_SHIFT;
	*pui32PageCount = psDevInfo->sSystemBuffer.ui32BufferSize >> PAGE_SHIFT;
	*pui32PhysHeapID = 0;
	*pui32Stride = psDevInfo->sSystemBuffer.ui32ByteStride;
	*phSystemBuffer = &psDevInfo->sSystemBuffer;

	DRM_INFO("%s: pc=%d, st=%d, bf=%p", __func__, *pui32PageCount,
		 *pui32Stride, *phSystemBuffer);

	return PVRSRV_OK;
}

static IMG_VOID DCMrfldBufferSystemRelease(IMG_HANDLE hSystemBuffer)
{
	MRFLD_BUFFER *psBuffer = hSystemBuffer;

	psBuffer->ui32RefCount--;

	/*
	   If the system buffer has changed and we've just dropped the last
	   refcount then free the buffer
	 */
	if (psBuffer->ui32RefCount == 0) {
		/* Free the buffer and it's memory (if the memory was allocated) */
	}

	DRM_INFO("%s: rc=%d, bf=%p", __func__, psBuffer->ui32RefCount,
		 psBuffer);
}

static int MrfldVSyncISR(struct drm_device *psDrmDevice, int iPipe)
{
	MRFLD_DEVINFO *psDevInfo = GetAnchorPtr();
	IMG_BOOL bStatus = IMG_FALSE;
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
	unsigned long ulMaxIndex;
	unsigned long ulLockFlags;
	MRSTLFB_SWAPCHAIN *psSwapChain;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	psSwapChain = psDevInfo->psCurrentSwapChain;
	if (psSwapChain == NULL) {
		goto ExitUnlock;
	}

	if (psDevInfo->bFlushCommands || psDevInfo->bSuspended) {
		goto ExitUnlock;
	}

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	ulMaxIndex = psSwapChain->ulSwapChainLength - 1;

	while (psFlipItem->bValid) {

		if (psFlipItem->bFlipped) {

			if (!psFlipItem->bCmdCompleted) {

				IMG_BOOL bScheduleMISR;

				bScheduleMISR = IMG_TRUE;

//                              psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete((IMG_HANDLE)psFlipItem->hCmdComplete, IMG_FALSE, bScheduleMISR);

				psFlipItem->bCmdCompleted = IMG_TRUE;
			}

			psFlipItem->ulSwapInterval--;

			if (psFlipItem->ulSwapInterval == 0) {

				psSwapChain->ulRemoveIndex++;

				if (psSwapChain->ulRemoveIndex > ulMaxIndex) {
					psSwapChain->ulRemoveIndex = 0;
				}

				psFlipItem->bCmdCompleted = IMG_FALSE;
				psFlipItem->bFlipped = IMG_FALSE;

				psFlipItem->bValid = IMG_FALSE;
			} else {

				break;
			}
		} else {

			DRMLFBFlipBuffer(psDevInfo, psSwapChain,
					 psFlipItem->psBuffer);

			psFlipItem->bFlipped = IMG_TRUE;

			break;
		}

		psFlipItem =
		    &psSwapChain->psVSyncFlips[psSwapChain->ulRemoveIndex];
	}
	if (drm_psb_dsr)
		schedule_delayed_work(&dsr_work, 10);

	if (psSwapChain->ulRemoveIndex == psSwapChain->ulInsertIndex)
		bStatus = IMG_TRUE;
 ExitUnlock:
	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	return bStatus;
}

#if 0
static IMG_BOOL ProcessFlip(IMG_HANDLE hCmdCookie,
			    IMG_UINT32 ui32DataSize, IMG_VOID * pvData)
{
	DISPLAYCLASS_FLIP_COMMAND *psFlipCmd;
	MRFLD_DEVINFO *psDevInfo;
	MRFLD_BUFFER *psBuffer;
	MRSTLFB_SWAPCHAIN *psSwapChain;
	MRSTLFB_VSYNC_FLIP_ITEM *psFlipItem;
	unsigned long ulLockFlags;

	if (!hCmdCookie || !pvData) {
		return IMG_FALSE;
	}

	psFlipCmd = (DISPLAYCLASS_FLIP_COMMAND *) pvData;

	if (psFlipCmd == IMG_NULL
	    || sizeof(DISPLAYCLASS_FLIP_COMMAND) != ui32DataSize) {
		return IMG_FALSE;
	}

	if (drm_psb_dsr)
		schedule_delayed_work(&dsr_exitwork, 0);

	psDevInfo = (MRFLD_DEVINFO *) psFlipCmd->hExtDevice;
	psBuffer = (MRFLD_BUFFER *) psFlipCmd->hExtBuffer;
	psSwapChain = (MRSTLFB_SWAPCHAIN *) psFlipCmd->hExtSwapChain;

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	if (!drm_psb_3D_vblank || psFlipCmd->ui32SwapInterval == 0
	    || psDevInfo->bFlushCommands) {
		DRMLFBFlipBuffer(psDevInfo, psSwapChain, psBuffer);

//              psSwapChain->psPVRJTable->pfnPVRSRVCmdComplete(hCmdCookie, IMG_FALSE, IMG_TRUE);

		goto ExitTrueUnlock;
	}

	psFlipItem = &psSwapChain->psVSyncFlips[psSwapChain->ulInsertIndex];

	DCCBFlipDSRCb(psDevInfo->psDrmDevice);

	if (psFlipItem->bValid == IMG_FALSE) {
		unsigned long ulMaxIndex = psSwapChain->ulSwapChainLength - 1;

		if (psSwapChain->ulInsertIndex == psSwapChain->ulRemoveIndex) {

			DRMLFBFlipBuffer(psDevInfo, psSwapChain, psBuffer);

			psFlipItem->bFlipped = IMG_TRUE;
		} else {
			psFlipItem->bFlipped = IMG_FALSE;
		}

		psFlipItem->hCmdComplete = hCmdCookie;
		psFlipItem->ulSwapInterval =
		    (unsigned long)psFlipCmd->ui32SwapInterval;
		psFlipItem->psBuffer = psBuffer;
		psFlipItem->bValid = IMG_TRUE;

		psSwapChain->ulInsertIndex++;
		if (psSwapChain->ulInsertIndex > ulMaxIndex) {
			psSwapChain->ulInsertIndex = 0;
		}

		goto ExitTrueUnlock;
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
	return IMG_FALSE;

 ExitTrueUnlock:

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);
	return IMG_TRUE;
}
#endif

int DCChangeFrameBuffer(struct drm_device *dev, struct psb_framebuffer *psbfb)
{
	MRFLD_DEVINFO *psDevInfo = GetAnchorPtr();
	int i;

	OSFreeMem(psDevInfo->sSystemBuffer.psSysAddr);

	psDevInfo->sPrimInfo.sDims.ui32Width = psbfb->base.width;
	psDevInfo->sPrimInfo.sDims.ui32Height = psbfb->base.height;

	psDevInfo->sSystemBuffer.ui32BufferSize = psbfb->size;
	psDevInfo->sSystemBuffer.ui32ByteStride = psbfb->base.pitch;
	psDevInfo->sSystemBuffer.sCPUVAddr = psbfb->vram_addr;

	psDevInfo->sSystemBuffer.sDevVAddr.uiAddr = 0;
	psDevInfo->sSystemBuffer.bIsAllocated = IMG_FALSE;
	psDevInfo->sSystemBuffer.ui32RefCount = 1;

	if (psbfb->bo) {
		psDevInfo->sSystemBuffer.bIsContiguous = IMG_FALSE;
		psDevInfo->sSystemBuffer.psSysAddr =
		    OSAllocMem(sizeof(IMG_SYS_PHYADDR) *
			       psbfb->bo->ttm->num_pages);
		for (i = 0; i < psbfb->bo->ttm->num_pages; ++i) {
			struct page *p = ttm_tt_get_page(psbfb->bo->ttm, i);
			psDevInfo->sSystemBuffer.psSysAddr[i].uiAddr =
			    page_to_pfn(p) << PAGE_SHIFT;
		}
	} else {
		psDevInfo->sSystemBuffer.bIsContiguous = IMG_FALSE;
		psDevInfo->sSystemBuffer.psSysAddr =
		    OSAllocMem(sizeof(IMG_SYS_PHYADDR) * psbfb->size /
			       PAGE_SIZE);
		for (i = 0; i < psbfb->size / PAGE_SIZE; ++i) {
			psDevInfo->sSystemBuffer.psSysAddr[i].uiAddr =
			    psbfb->stolen_base + i * PAGE_SIZE;
		}
	}

	return 0;
}

bool DCChangeSwapChainProperty(unsigned long *psSwapChainGTTOffset,
			       int iSwapChainAttachedPlane)
{
	MRFLD_DEVINFO *psDevInfo = GetAnchorPtr();
	struct drm_device *psDrmDevice = NULL;
	struct psb_framebuffer *psbfb = NULL;
	uint32_t tt_pages = 0;
	uint32_t max_gtt_offset = 0;

	IMG_UINT32 ui32SwapChainID = 0;
	unsigned long ulLockFlags;
	unsigned long ulCurrentSwapChainGTTOffset = 0;

	if (psDevInfo == IMG_NULL) {
		DRM_ERROR("MRSTLFB hasn't been initialized\n");
		/* Won't attach/de-attach the plane in case of no swap chain
		 * created. */
		return false;
	}

	if (psDevInfo->apsSwapChains == IMG_NULL) {
		DRM_ERROR("No swap chain.\n");
		return false;
	}

	psDrmDevice = psDevInfo->psDrmDevice;
	DCCBGetFramebuffer(psDrmDevice, &psbfb);
	if (psbfb == NULL) {
		DRM_ERROR("Invalid psbfb data.\n");
		return false;
	}

	tt_pages = psbfb->tt_pages;

	/* Another half of GTT is managed by TTM. */
	tt_pages /= 2;
	max_gtt_offset = (tt_pages << PAGE_SHIFT) / 4;

	if ((psSwapChainGTTOffset == IMG_NULL) ||
	    (*psSwapChainGTTOffset > max_gtt_offset)) {
		DRM_ERROR("Invalid GTT offset.\n");
		return false;
	}

	switch (iSwapChainAttachedPlane) {
	case PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A:
	case PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B:
	case PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C:
		break;
	default:
		DRM_ERROR("Illegal Pipe Number %d.\n", iSwapChainAttachedPlane);
		return false;
	}

	spin_lock_irqsave(&psDevInfo->sSwapChainLock, ulLockFlags);

	ulCurrentSwapChainGTTOffset =
	    psDevInfo->apsSwapChains[ui32SwapChainID]->ulSwapChainGTTOffset;

	for (ui32SwapChainID = 0; ui32SwapChainID < psDevInfo->ui32SwapChainNum;
	     ui32SwapChainID++) {
		if (psDevInfo->apsSwapChains[ui32SwapChainID] == IMG_NULL)
			continue;

		if (*psSwapChainGTTOffset == ulCurrentSwapChainGTTOffset) {
			psDevInfo->apsSwapChains[ui32SwapChainID]->
			    ui32SwapChainPropertyFlag |=
			    iSwapChainAttachedPlane;

			/*
			 * Trigger the display plane to flip to the swap
			 * chain's last flip surface, to avoid that it still
			 * displays with original GTT offset after mode setting
			 * and attached to the specific swap chain.
			 */
			if (psDevInfo->bLastFlipAddrValid)
				*psSwapChainGTTOffset
				    = psDevInfo->ulLastFlipAddr;
		} else
			psDevInfo->apsSwapChains[ui32SwapChainID]->
			    ui32SwapChainPropertyFlag &=
			    ~iSwapChainAttachedPlane;
	}

	spin_unlock_irqrestore(&psDevInfo->sSwapChainLock, ulLockFlags);

	return true;
}

static PVRSRV_ERROR InitDev(MRFLD_DEVINFO * psDevInfo)
{
	struct drm_device *psDrmDevice = psDevInfo->psDrmDevice;
	struct psb_framebuffer *psbfb;
	int i;

	DCCBGetFramebuffer(psDrmDevice, &psbfb);

	DCChangeFrameBuffer(psDrmDevice, psbfb);

	switch (psbfb->depth) {
	case 32:
	case 24:
		psDevInfo->sPrimInfo.sFormat.ePixFormat =
		    IMG_PIXFMT_B8G8R8A8_UNORM;
		break;
	case 16:
		psDevInfo->sPrimInfo.sFormat.ePixFormat =
		    IMG_PIXFMT_B5G6R5_UNORM;
		break;
	default:
		DRM_ERROR("%s: Unknown bit depth %d\n", __FUNCTION__,
			  psbfb->depth);
		break;
	}

	for (i = 0; i < MAX_SWAPCHAINS; ++i) {
		psDevInfo->apsSwapChains[i] = NULL;
	}

	return PVRSRV_OK;
}

PVRSRV_ERROR MerrifieldDCInit(struct drm_device * dev)
{
	MRFLD_DEVINFO *psDevInfo;
/*
	PFN_CMD_PROC		pfnCmdProcList[MRSTLFB_COMMAND_COUNT];
	IMG_UINT32			aui32SyncCountList[MRSTLFB_COMMAND_COUNT][2];
*/
	psDevInfo = GetAnchorPtr();

	if (psDevInfo) {
		psDevInfo->ulRefCount++;
		return (PVRSRV_OK);
	}

	DRM_DEBUG("Initialize DC %s\n", DISPLAY_DEVICE_NAME);

	psDevInfo = (MRFLD_DEVINFO *) OSAllocMem(sizeof(MRFLD_DEVINFO));

	if (!psDevInfo) {
		return (PVRSRV_ERROR_OUT_OF_MEMORY);
	}

	memset(psDevInfo, 0, sizeof(MRFLD_DEVINFO));

	gpvAnchor = (void *)psDevInfo;

	psDevInfo->psDrmDevice = dev;
	psDevInfo->ulRefCount = 0;

	if (InitDev(psDevInfo) != PVRSRV_OK) {
		OSFreeMem(psDevInfo);
		return (PVRSRV_ERROR_INIT_FAILURE);
	}

	psDevInfo->ulSetFlushStateRefCount = 0;
	psDevInfo->bFlushCommands = IMG_FALSE;

	if (EnableLFBEventNotification(psDevInfo) != PVRSRV_OK) {
		DRM_ERROR("Couldn't enable framebuffer event notification\n");
		OSFreeMem(psDevInfo);
		return PVRSRV_ERROR_UNABLE_TO_OPEN_DC_DEVICE;
	}

	spin_lock_init(&psDevInfo->sSwapChainLock);

	psDevInfo->psCurrentSwapChain = NULL;
	psDevInfo->bFlushCommands = IMG_FALSE;

	strncpy(psDevInfo->sDisplayInfo.szDisplayName, DISPLAY_DEVICE_NAME, 50);

	/*
	   Report what our minimum and maximum display period is.
	 */
	psDevInfo->sDisplayInfo.ui32MinDisplayPeriod = 0;
	psDevInfo->sDisplayInfo.ui32MaxDisplayPeriod = 1;

	psDevInfo->sDCJTable.pfnGetInfo = DCMrfldGetDCInfo;
	psDevInfo->sDCJTable.pfnPanelQueryCount = DCMrfldPanelQueryCount;
	psDevInfo->sDCJTable.pfnPanelQuery = DCMrfldPanelQuery;
	psDevInfo->sDCJTable.pfnFormatQuery = DCMrfldFormatQuery;
	psDevInfo->sDCJTable.pfnDimQuery = DCMrfldDimQuery;
	psDevInfo->sDCJTable.pfnContextCreate = DCMrfldContextCreate;
	psDevInfo->sDCJTable.pfnContextDestroy = DCMrfldContextDestroy;
	psDevInfo->sDCJTable.pfnContextConfigure = DCMrfldContextConfigure;
	psDevInfo->sDCJTable.pfnContextConfigureCheck =
	    DCMrfldContextConfigureCheck;
	psDevInfo->sDCJTable.pfnBufferAlloc = DCMrfldBufferAlloc;
	psDevInfo->sDCJTable.pfnBufferAcquire = DCMrfldBufferAcquire;
	psDevInfo->sDCJTable.pfnBufferRelease = DCMrfldBufferRelease;
	psDevInfo->sDCJTable.pfnBufferFree = DCMrfldBufferFree;
	psDevInfo->sDCJTable.pfnBufferImport = DCMrfldBufferImport;
	psDevInfo->sDCJTable.pfnBufferMap = DCMrfldBufferMap;
	psDevInfo->sDCJTable.pfnBufferUnmap = DCMrfldBufferUnmap;
	psDevInfo->sDCJTable.pfnBufferSystemAcquire =
	    DCMrfldBufferSystemAcquire;
	psDevInfo->sDCJTable.pfnBufferSystemRelease =
	    DCMrfldBufferSystemRelease;

	/*
	   Register our DC driver with services
	 */
	if (DCRegisterDevice(&psDevInfo->sDCJTable,
			     2,
			     psDevInfo, &psDevInfo->hSrvHandle) != PVRSRV_OK) {
		OSFreeMem(psDevInfo);
		return (PVRSRV_ERROR_DEVICE_REGISTER_FAILED);
	}

	DRM_DEBUG("Service Handle: %p\n", psDevInfo->hSrvHandle);

	DCCBInstallVSyncISR(dev, MrfldVSyncISR);
/*
	pfnCmdProcList[DC_FLIP_COMMAND] = ProcessFlip;

	aui32SyncCountList[DC_FLIP_COMMAND][0] = 0;
	aui32SyncCountList[DC_FLIP_COMMAND][1] = 2;

	if (psDevInfo->sPVRJTable.pfnPVRSRVRegisterCmdProcList (psDevInfo->uiDeviceID,
							&pfnCmdProcList[0],
							aui32SyncCountList,
							MRSTLFB_COMMAND_COUNT) != PVRSRV_OK)
	{
		DRM_ERROR("Can't register callback\n");
		return (PVRSRV_ERROR_CANT_REGISTER_CALLBACK);
	}
*/

	DRM_DEBUG("Initialize DC success");
	return (PVRSRV_OK);
}

PVRSRV_ERROR MerrifieldDCDeinit(void)
{
	MRFLD_DEVINFO *psDevInfo, *psDevFirst;

	psDevFirst = GetAnchorPtr();
	psDevInfo = psDevFirst;

	if (psDevInfo == NULL) {
		return (PVRSRV_ERROR_INVALID_PARAMS);
	}

	psDevInfo->ulRefCount--;

	if (psDevInfo->ulRefCount == 0) {
		DCCBUninstallVSyncISR(psDevInfo->psDrmDevice);

		DCUnregisterDevice(psDevInfo->hSrvHandle);

		/* Free system buffer? */
		if (psDevInfo->sSystemBuffer.psSysAddr) {
			OSFreeMem(psDevInfo->sSystemBuffer.psSysAddr);
		}

		if (DisableLFBEventNotification(psDevInfo) != PVRSRV_OK) {
			DRM_ERROR
			    ("Couldn't disable framebuffer event notification\n");
		}

		OSFreeMem(psDevInfo);
	}

	gpvAnchor = NULL;

	return (PVRSRV_OK);
}
