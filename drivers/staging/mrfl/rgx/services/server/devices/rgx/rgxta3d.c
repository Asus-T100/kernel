/*************************************************************************/ /*!
@File
@Title          RGX TA/3D routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX TA/3D routines
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/
/* for the offsetof macro */
#include <stddef.h> 

#include "pdump_km.h"
#include "pvr_debug.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgxta3d.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "osfunc.h"
#include "pvrsrv.h"


#include "rgxdefs_km.h"
#include "rgx_fwif_km.h"
#include "physmem.h"

static PVRSRV_ERROR _UpdateFwFreelistSize(RGX_FREELIST *psFreeList)
{
	PVRSRV_ERROR			eError;
	RGXFWIF_KCCB_CMD		sGPCCBCmd;

	sGPCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_FREELIST_SIZE_UPDATE;
	sGPCCBCmd.uCmdData.sFreelistSizeData.ui32FwFreeListAddr = psFreeList->sFreeListFWDevVAddr.ui32Addr;
	sGPCCBCmd.uCmdData.sFreelistSizeData.ui32NewNumPages = psFreeList->ui32CurrentFLPages;

	PVR_DPF((PVR_DBG_MESSAGE, "Send FW update: freelist [FWAddr=0x%08x] has 0x%08x pages",
								psFreeList->sFreeListFWDevVAddr.ui32Addr,
								psFreeList->ui32CurrentFLPages));

	/* Submit command to the firmware.  */
	eError = RGXScheduleCommand(psFreeList->psDeviceNode->pvDevice,
								RGXFWIF_DM_GP,
								&sGPCCBCmd,
								sizeof(sGPCCBCmd),
								IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "_UpdateFwFreelistSize: failed to update FW freelist size. (error = %u)", eError));
		return eError;
	}

	return PVRSRV_OK;
}

PVRSRV_ERROR RGXGrowFreeList(RGX_FREELIST *psFreeList,
							IMG_UINT32 ui32NumPages,
							PDLLIST_NODE pListHeader)
{
	RGX_PMR_NODE	*psPMRNode;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_BOOL bMappingTable = IMG_TRUE;
	IMG_DEVMEM_OFFSET_T uiOffset;
	IMG_DEVMEM_SIZE_T uiLength;
	IMG_DEVMEM_SIZE_T uistartPage;
	PVRSRV_ERROR eError;

	/* Are we allowed to grow ? */
	if ((psFreeList->ui32MaxFLPages - psFreeList->ui32CurrentFLPages) < ui32NumPages)
	{
		PVR_DPF((PVR_DBG_WARNING,"Freelist [0x%p]: grow by %u pages denied. Max PB size reached (current pages %u/%u)",
				psFreeList,
				ui32NumPages,
				psFreeList->ui32CurrentFLPages,
				psFreeList->ui32MaxFLPages));
		return PVRSRV_ERROR_PBSIZE_ALREADY_MAX;
	}

	/* Allocate kernel memory block structure */
	psPMRNode = OSAllocMem(sizeof(*psPMRNode));
	if (psPMRNode == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXGrowFreeList: failed to allocated host data structure\n"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorAllocHost;
	}

	/*
	 * Lock protects simultaneous manipulation of:
	 * - the memory block list
	 * - the freelist's ui32CurrentFLPages
	 */
	OSLockAcquire(psFreeList->psDeviceNode->hLockFreeList);


	psPMRNode->ui32NumPages = ui32NumPages;
	psPMRNode->psFreeList = psFreeList;

	/* Allocate Memory Block */
	PDUMPCOMMENT("Allocate PB Block (Pages %08X)", ui32NumPages);
	uiSize = ui32NumPages * 4096;  /* FIXME: get correct page size (mi) */
	eError = PhysmemNewRamBackedPMR(psFreeList->psDeviceNode,
									uiSize,
									uiSize,
									1,
									1,
									&bMappingTable,
									12,		/* FIXME: get from HW defines (mi) */
									PVRSRV_MEMALLOCFLAG_GPU_READABLE,
									&psPMRNode->psPMR);
	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "RGXGrowFreeList: Failed to allocate PB block of size: 0x%016llX",
				 (IMG_UINT64)uiSize));
		goto ErrorBlockAlloc;
	}

	uiLength = psPMRNode->ui32NumPages * sizeof(IMG_UINT32);
	uistartPage = (psFreeList->ui32MaxFLPages - psFreeList->ui32CurrentFLPages - psPMRNode->ui32NumPages);
	uiOffset = psFreeList->uiFreeListPMROffset + (uistartPage * sizeof(IMG_UINT32));

   	/* FIXME: some sanity checks wrt staying within the allocation (mi) */
//    PVR_ASSERT(uiOffset + uiLength <= psFreeList->psFreeListPMR->uiLogicalSize);

	/* write Freelist with Memory Block physical addresses */
	eError = PMRWritePMPageList(
						/* Target PMR, offset, and length */
						psFreeList->psFreeListPMR,
						uiOffset,
						uiLength,
						/* Referenced PMR, and "page" granularity */
						psPMRNode->psPMR,
						12, 				/* FIXME: get correct size (mi) */
						&psPMRNode->psPageList);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "RGXGrowFreeList: Failed to write pages of Node %p",
				 psPMRNode));
		goto ErrorPopulateFreelist;
	}

	/* We add It must be added to the tail, otherwise the freelist population won't work */
	dllist_add_to_head(pListHeader, &psPMRNode->sMemoryBlock);

	/* Update number of available pages */
	psFreeList->ui32CurrentFLPages += ui32NumPages;

	/* Update statistics */
	if (psFreeList->ui32NumHighPages < psFreeList->ui32CurrentFLPages)
	{
		psFreeList->ui32NumHighPages = psFreeList->ui32CurrentFLPages;
	}

	OSLockRelease(psFreeList->psDeviceNode->hLockFreeList);

	PVR_DPF((PVR_DBG_MESSAGE,"Freelist [%p]: grow by %u pages (current pages %u/%u)",
			psFreeList,
			ui32NumPages,
			psFreeList->ui32CurrentFLPages,
			psFreeList->ui32MaxFLPages));

	return PVRSRV_OK;

	/* Error handling */
ErrorPopulateFreelist:
	PMRUnrefPMR(psPMRNode->psPMR);

ErrorBlockAlloc:
	OSFreeMem(psPMRNode);

ErrorAllocHost:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;

}

static PVRSRV_ERROR RGXShrinkFreeList(PDLLIST_NODE pListHeader,
										RGX_FREELIST *psFreeList)
{
	DLLIST_NODE *psNode;
	RGX_PMR_NODE *psPMRNode;
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32OldValue;

	/*
	 * Lock protects simultaneous manipulation of:
	 * - the memory block list
	 * - the freelist's ui32CurrentFLPages value
	 */
	PVR_ASSERT(pListHeader);
	PVR_ASSERT(psFreeList);
	PVR_ASSERT(psFreeList->psDeviceNode);
	PVR_ASSERT(psFreeList->psDeviceNode->hLockFreeList);

	OSLockAcquire(psFreeList->psDeviceNode->hLockFreeList);

	/* Get node from head of list and remove it */
	psNode = dllist_get_next_node(pListHeader);
	if (psNode)
	{
		dllist_remove_node(psNode);

		psPMRNode = IMG_CONTAINER_OF(psNode, RGX_PMR_NODE, sMemoryBlock);
		PVR_ASSERT(psPMRNode);
		PVR_ASSERT(psPMRNode->psPMR);
		PVR_ASSERT(psPMRNode->psFreeList);

		/* remove block from freelist list */

		/* Unwrite Freelist with Memory Block physical addresses */
		eError = PMRUnwritePMPageList(psPMRNode->psPageList);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					 "RGXRemoveBlockFromFreeListKM: Failed to unwrite pages of Node %p",
					 psPMRNode));
			PVR_ASSERT(IMG_FALSE);
		}

		/* Free PMR (We should be the only one that holds a ref on the PMR) */
		eError = PMRUnrefPMR(psPMRNode->psPMR);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					 "RGXRemoveBlockFromFreeListKM: Failed to free PB block %p (error %u)",
					 psPMRNode->psPMR,
					 eError));
			PVR_ASSERT(IMG_FALSE);
		}

		/* update available pages in freelist */
		ui32OldValue = psFreeList->ui32CurrentFLPages;
		psFreeList->ui32CurrentFLPages -= psPMRNode->ui32NumPages;

		/* check underflow */
		PVR_ASSERT(ui32OldValue > psFreeList->ui32CurrentFLPages);

		PVR_DPF((PVR_DBG_MESSAGE, "Freelist [%p]: shrink by %u pages (current pages %u/%u)",
								psFreeList,
								psPMRNode->ui32NumPages,
								psFreeList->ui32CurrentFLPages,
								psFreeList->ui32MaxFLPages));

		OSFreeMem(psPMRNode);
	}
	else
	{
		PVR_DPF((PVR_DBG_WARNING,"Freelist [0x%p]: shrink denied. PB already at initial PB size (%u pages)",
								psFreeList,
								psFreeList->ui32InitFLPages));
		return PVRSRV_ERROR_PBSIZE_ALREADY_MIN;
	}

	OSLockRelease(psFreeList->psDeviceNode->hLockFreeList);

	return PVRSRV_OK;
}


/* Create HWRTDataSet */
IMG_EXPORT
PVRSRV_ERROR RGXCreateHWRTData(PVRSRV_DEVICE_NODE	*psDeviceNode,
							   IMG_UINT32			psRenderTarget, /* FIXME this should not be IMG_UINT32 */
							   IMG_DEV_VIRTADDR		psPMMListDevVAddr,
							   IMG_DEV_VIRTADDR		psVFPPageTableAddr,
							   IMG_UINT32			apsFreeLists[RGXFW_MAX_FREELISTS],
							   RGX_RTDATA_CLEANUP_DATA	**ppsCleanupData,
							   DEVMEM_MEMDESC		**ppsRTACtlMemDesc,
							   IMG_UINT16			ui16MaxRTs,
							   DEVMEM_MEMDESC		**ppsMemDesc,
							   IMG_UINT32			*puiHWRTData)
{
	PVRSRV_ERROR eError;
	PVRSRV_RGXDEV_INFO *psDevInfo;
	RGXFWIF_DEV_VIRTADDR pFirmwareAddr;
	RGXFWIF_HWRTDATA *psHWRTData;
	RGXFWIF_RTA_CTL *psRTACtl;
	IMG_UINT32 ui32Loop;
	RGX_RTDATA_CLEANUP_DATA *psTmpCleanup;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto AllocError;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/* Allocate cleanup sync */
	eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext,
						   &psTmpCleanup->psCleanupSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateHWRTData: Failed to allocate cleanup sync (0x%x)",
				eError));
		goto SyncAlloc;
	}

	psDevInfo = psDeviceNode->pvDevice;

	/*
	 * This FW RT-Data is only mapped into kernel for initialisation.
	 * Otherwise this allocation is only used by the FW.
	 * Therefore the GPU cache doesn't need coherency,
	 * and write-combine is suffice on the CPU side (WC buffer will be flushed at the first TA-kick)
	 */
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(RGXFWIF_HWRTDATA),
							PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
							PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
							PVRSRV_MEMALLOCFLAG_GPU_READABLE |
							PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
							PVRSRV_MEMALLOCFLAG_CPU_READABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
							PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
							ppsMemDesc);
	if (eError != PVRSRV_OK) 
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXCreateHWRTData: DevmemAllocate for RGX_FWIF_HWRTDATA failed\n"));
		goto FWRTDataAllocateError;
	}

	psTmpCleanup->psDeviceNode = psDeviceNode;
	psTmpCleanup->psFWHWRTDataMemDesc = *ppsMemDesc;

	RGXSetFirmwareAddress(&pFirmwareAddr, *ppsMemDesc, 0, RFW_FWADDR_FLAG_NONE);

	*puiHWRTData = pFirmwareAddr.ui32Addr;

	DevmemAcquireCpuVirtAddr(*ppsMemDesc, (IMG_VOID **)&psHWRTData);

	/* FIXME: MList is something that that PM writes physical addresses to,
	 * so ideally its best allocated in kernel */
	psHWRTData->psPMMListDevVAddr = psPMMListDevVAddr;
	psHWRTData->psParentRenderTarget.ui32Addr = psRenderTarget;
	#if defined(SUPPORT_VFP)
	psHWRTData->sVFPPageTableAddr = psVFPPageTableAddr;
	#endif
	for (ui32Loop = 0; ui32Loop < RGXFW_MAX_FREELISTS; ui32Loop++)
	{
		psHWRTData->apsFreeLists[ui32Loop] = *((PRGXFWIF_FREELIST *)&(apsFreeLists[ui32Loop])); /* FIXME: Fix pointer type casting */
	}
	
	PDUMPCOMMENT("Allocate RGXFW RTA control");
	eError = DevmemFwAllocate(psDevInfo,
										sizeof(RGXFWIF_RTA_CTL),
										PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
										PVRSRV_MEMALLOCFLAG_GPU_READABLE |
										PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
										PVRSRV_MEMALLOCFLAG_UNCACHED |
										PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC,
										ppsRTACtlMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateHWRTData: Failed to allocate RGX RTA control (%u)",
				eError));
		goto FWRTAAllocateError;
	}
	psTmpCleanup->psRTACtlMemDesc = *ppsRTACtlMemDesc;
	RGXSetFirmwareAddress(&psHWRTData->psRTACtl,
								   *ppsRTACtlMemDesc,
								   0, RFW_FWADDR_FLAG_NONE);
	
	DevmemAcquireCpuVirtAddr(*ppsRTACtlMemDesc, (IMG_VOID **)&psRTACtl);
	psRTACtl->ui32RenderTargetIndex = 0;
	psRTACtl->ui32ActiveRenderTargets = 0;

	if (ui16MaxRTs > 1)
	{
		/* Allocate memory for the checks */
		PDUMPCOMMENT("Allocate memory for shadow render target cache");
		eError = DevmemFwAllocate(psDevInfo,
								ui16MaxRTs,
								PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
								PVRSRV_MEMALLOCFLAG_GPU_READABLE |
								PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
								PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
								PVRSRV_MEMALLOCFLAG_UNCACHED|
								PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC, 
								&psTmpCleanup->psRTArrayMemDesc);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXCreateHWRTData: Failed to allocate %d bytes for render target array (%u)",
						ui16MaxRTs,
						eError));
			goto FWAllocateRTArryError;
		}

		RGXSetFirmwareAddress(&psRTACtl->paui32ValidRenderTargets,
										psTmpCleanup->psRTArrayMemDesc,
										0, RFW_FWADDR_FLAG_NONE);
	}
	else
	{
		psRTACtl->paui32ValidRenderTargets.ui32Addr = 0;
	}
		
	PDUMPCOMMENT("Dump HWRTData 0x%08X", *puiHWRTData);
	DevmemPDumpLoadMem(*ppsMemDesc, 0, sizeof(*psHWRTData), PDUMP_FLAGS_CONTINUOUS);
	PDUMPCOMMENT("Dump RTACtl");
	DevmemPDumpLoadMem(*ppsRTACtlMemDesc, 0, sizeof(*psRTACtl), PDUMP_FLAGS_CONTINUOUS);

	DevmemReleaseCpuVirtAddr(*ppsMemDesc);
	DevmemReleaseCpuVirtAddr(*ppsRTACtlMemDesc);
	return PVRSRV_OK;

	DevmemFwFree(psTmpCleanup->psRTArrayMemDesc);
FWAllocateRTArryError:
	RGXUnsetFirmwareAddress(*ppsRTACtlMemDesc);
	DevmemFwFree(*ppsRTACtlMemDesc);
FWRTAAllocateError:
	RGXUnsetFirmwareAddress(*ppsMemDesc);
	DevmemFwFree(*ppsMemDesc);
FWRTDataAllocateError:
	SyncPrimFree(psTmpCleanup->psCleanupSync);
SyncAlloc:
	OSFreeMem(psTmpCleanup);

AllocError:
	return eError;
}

/* Destroy HWRTDataSet */
IMG_EXPORT
PVRSRV_ERROR RGXDestroyHWRTData(RGX_RTDATA_CLEANUP_DATA *psCleanupData)
{
	PVRSRV_ERROR eError;
	PRGXFWIF_HWRTDATA psHWRTData;

	RGXSetFirmwareAddress(&psHWRTData, psCleanupData->psFWHWRTDataMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	/* Cleanup HWRTData in TA */
	eError = RGXFWRequestHWRTDataCleanUp(psCleanupData->psDeviceNode,
										 psHWRTData,
										 psCleanupData->psCleanupSync,
										 RGXFWIF_DM_TA);
	if (eError == PVRSRV_ERROR_RETRY)
	{
		return eError;
	}

	/* Cleanup HWRTData in 3D */
	eError = RGXFWRequestHWRTDataCleanUp(psCleanupData->psDeviceNode,
										 psHWRTData,
										 psCleanupData->psCleanupSync,
										 RGXFWIF_DM_3D);
	if (eError == PVRSRV_ERROR_RETRY)
	{
		return eError;
	}

	/* If we got here then TA and 3D operations on this RTData have finished */
	if(psCleanupData->psRTACtlMemDesc)
	{
		RGXUnsetFirmwareAddress(psCleanupData->psRTACtlMemDesc);
		DevmemFwFree(psCleanupData->psRTACtlMemDesc);
	}
	
	RGXUnsetFirmwareAddress(psCleanupData->psFWHWRTDataMemDesc);
	DevmemFwFree(psCleanupData->psFWHWRTDataMemDesc);
	
	if(psCleanupData->psRTArrayMemDesc)
	{
		RGXUnsetFirmwareAddress(psCleanupData->psRTArrayMemDesc);
		DevmemFwFree(psCleanupData->psRTArrayMemDesc);
	}

	SyncPrimFree(psCleanupData->psCleanupSync);

	OSFreeMem(psCleanupData);

	return PVRSRV_OK;
}

IMG_EXPORT
PVRSRV_ERROR RGXCreateFreeList(PVRSRV_DEVICE_NODE	*psDeviceNode, 
							   IMG_UINT32			ui32MaxFLPages,
							   IMG_UINT32			ui32InitFLPages,
							   IMG_UINT32			ui32GrowFLPages,
							   IMG_DEV_VIRTADDR		sFreeListDevVAddr,
							   PMR					*psFreeListPMR,
							   IMG_DEVMEM_OFFSET_T	uiFreeListPMROffset,
							   IMG_UINT32			*sFreeListFWDevVAddr,
							   RGX_FREELIST			**ppsFreeList)
{
	PVRSRV_ERROR				eError;
	RGXFWIF_FREELIST			*psFWFreeList;
	DEVMEM_MEMDESC				*psFWFreelistMemDesc;
	RGX_FREELIST				*psFreeList;
	PVRSRV_RGXDEV_INFO			*psDevInfo = psDeviceNode->pvDevice;

	/* Allocate kernel freelist struct */
	psFreeList = OSAllocMem(sizeof(*psFreeList));
	if (psFreeList == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXCreateFreeList: failed to allocated host data structure\n"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorAllocHost;
	}
	OSMemSet(psFreeList, 0, sizeof(*psFreeList));

	/* Allocate cleanup sync */
	eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext,
						   &psFreeList->psCleanupSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateHWRTData: Failed to allocate cleanup sync (0x%x)",
				eError));
		goto SyncAlloc;
	}

	/*
	 * This FW FreeList context is only mapped into kernel for initialisation.
	 * Otherwise this allocation is only used by the FW.
	 * Therefore the GPU cache doesn't need coherency,
	 * and write-combine is suffice on the CPU side (WC buffer will be flushed at the first TA-kick)
	 */
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(*psFWFreeList),
							PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
							PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
							PVRSRV_MEMALLOCFLAG_GPU_READABLE |
							PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
							PVRSRV_MEMALLOCFLAG_CPU_READABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
							PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
							&psFWFreelistMemDesc);
	if (eError != PVRSRV_OK) 
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXCreateFreeList: DevmemAllocate for RGXFWIF_FREELIST failed\n"));
		goto FWFreeListAlloc;
	}

	/* Initialise host data structures */
	psFreeList->psDeviceNode = psDeviceNode;
	psFreeList->psFreeListPMR = psFreeListPMR;
	psFreeList->uiFreeListPMROffset = uiFreeListPMROffset;
	psFreeList->psFWFreelistMemDesc = psFWFreelistMemDesc;
	RGXSetFirmwareAddress(&psFreeList->sFreeListFWDevVAddr, psFWFreelistMemDesc, 0, RFW_FWADDR_FLAG_NONE);
	psFreeList->ui32FreelistID = psDeviceNode->ui32FreelistCurrID++;
	psFreeList->ui32MaxFLPages = ui32MaxFLPages;
	psFreeList->ui32InitFLPages = ui32InitFLPages;
	psFreeList->ui32GrowFLPages = ui32GrowFLPages;
	psFreeList->ui32CurrentFLPages = 0;
	dllist_init(&psFreeList->sMemoryBlockHead);
	dllist_init(&psFreeList->sMemoryBlockInitHead);


	/* Add to list of freelists */
	OSLockAcquire(psDeviceNode->hLockFreeList);
	dllist_add_to_tail(&psDeviceNode->sFreeListHead, &psFreeList->sNode);
	OSLockRelease(psDeviceNode->hLockFreeList);


	/* Initialise FW data structure */
	DevmemAcquireCpuVirtAddr(psFreeList->psFWFreelistMemDesc, (IMG_VOID **)&psFWFreeList);
	psFWFreeList->ui32MaxPages = ui32MaxFLPages;
	psFWFreeList->ui32CurrentPages = ui32InitFLPages;
	psFWFreeList->ui32GrowPages = ui32GrowFLPages;
	psFWFreeList->ui64CurrentStackTop = ui32InitFLPages - 1;
	psFWFreeList->psFreeListDevVAddr = sFreeListDevVAddr;
	psFWFreeList->ui64CurrentDevVAddr = sFreeListDevVAddr.uiAddr + ((ui32MaxFLPages - ui32InitFLPages) * sizeof(IMG_UINT32));
	psFWFreeList->ui32FreeListID = psFreeList->ui32FreelistID;

	PVR_DPF((PVR_DBG_MESSAGE,"Freelist %p created: Max pages 0x%08x, Init pages 0x%08x, Max FL base address 0x%016llx, Init FL base address 0x%016llx",
			psFreeList,
			ui32MaxFLPages,
			ui32InitFLPages,
			sFreeListDevVAddr.uiAddr,
			psFWFreeList->psFreeListDevVAddr.uiAddr));

	PDUMPCOMMENT("Dump FW FreeList");
	DevmemPDumpLoadMem(psFreeList->psFWFreelistMemDesc, 0, sizeof(*psFWFreeList), PDUMP_FLAGS_CONTINUOUS);

	/*
	 * Separate dump of the Freelist's number of Pages and stack pointer.
	 * This allows to easily modify the PB size in the out2.txt files.
	 */
	PDUMPCOMMENT("FreeList TotalPages");
	DevmemPDumpLoadMemValue64(psFreeList->psFWFreelistMemDesc,
							offsetof(RGXFWIF_FREELIST, ui32CurrentPages),
							psFWFreeList->ui32CurrentPages,
							PDUMP_FLAGS_CONTINUOUS);
	PDUMPCOMMENT("FreeList StackPointer");
	DevmemPDumpLoadMemValue64(psFreeList->psFWFreelistMemDesc,
							offsetof(RGXFWIF_FREELIST, ui64CurrentStackTop),
							psFWFreeList->ui64CurrentStackTop,
							PDUMP_FLAGS_CONTINUOUS);

	DevmemReleaseCpuVirtAddr(psFreeList->psFWFreelistMemDesc);


	/* Add initial PB block */
	eError = RGXGrowFreeList(psFreeList,
								ui32InitFLPages,
								&psFreeList->sMemoryBlockInitHead);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"RGXCreateFreeList: failed to allocate initial memory block for free list 0x%016llx (error = %u)\n",
				sFreeListDevVAddr.uiAddr,
				eError));
		goto ErrorAllocBlock;
	}

	/* return values */
	*sFreeListFWDevVAddr = psFreeList->sFreeListFWDevVAddr.ui32Addr;
	*ppsFreeList = psFreeList;

	return PVRSRV_OK;

	/* Error handling */

ErrorAllocBlock:
	RGXUnsetFirmwareAddress(psFWFreelistMemDesc);
	DevmemFwFree(psFWFreelistMemDesc);

FWFreeListAlloc:
	SyncPrimFree(psFreeList->psCleanupSync);

SyncAlloc:
	OSFreeMem(psFreeList);

ErrorAllocHost:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}


/*
	RGXDestroyFreeList
*/
IMG_EXPORT
PVRSRV_ERROR RGXDestroyFreeList(RGX_FREELIST *psFreeList)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psFreeList);

	eError = RGXFWRequestFreeListCleanUp(psFreeList->psDeviceNode,
										 psFreeList->sFreeListFWDevVAddr,
										 psFreeList->psCleanupSync);
	if (eError != PVRSRV_ERROR_RETRY)
	{
		/* print some statistics */
		if ((psFreeList->ui32NumGrowReqByApp > 0) || (psFreeList->ui32NumGrowReqByFW > 0))
		{
			PVR_LOG(("F/L destroyed [0x%p]: On-Demand Grows = %u, Push-Grows = %u, Number of Pages (init) = %u, Number of Pages (watermark) = %u",
					psFreeList,
					psFreeList->ui32NumGrowReqByFW,
					psFreeList->ui32NumGrowReqByApp,
					psFreeList->ui32InitFLPages,
					psFreeList->ui32NumHighPages));
		}

		/* Destroy FW structures */
		RGXUnsetFirmwareAddress(psFreeList->psFWFreelistMemDesc);
		DevmemFwFree(psFreeList->psFWFreelistMemDesc);

		/* Remove grow shrink blocks */
		while (!dllist_is_empty(&psFreeList->sMemoryBlockHead))
		{
			RGXShrinkFreeList(&psFreeList->sMemoryBlockHead,
							psFreeList);
		}

		/* Remove initial PB block */
		eError = RGXShrinkFreeList(&psFreeList->sMemoryBlockInitHead,
						psFreeList);

		/* consistency checks */
		PVR_ASSERT(eError == PVRSRV_OK);
		PVR_ASSERT(dllist_is_empty(&psFreeList->sMemoryBlockInitHead));
		PVR_ASSERT(psFreeList->ui32CurrentFLPages == 0);

		/* Remove FreeList from list */
		OSLockAcquire(psFreeList->psDeviceNode->hLockFreeList);
		dllist_remove_node(&psFreeList->sNode);
		OSLockRelease(psFreeList->psDeviceNode->hLockFreeList);

		SyncPrimFree(psFreeList->psCleanupSync);

		/* free Freelist */
		OSFreeMem(psFreeList);
	}

	return eError;
}



/*
	RGXAddBlockToFreeListKM
*/

IMG_EXPORT
PVRSRV_ERROR RGXAddBlockToFreeListKM(RGX_FREELIST *psFreeList,
										IMG_UINT32 ui32NumPages)
{
	PVRSRV_ERROR eError;

	/* Check if we have reference to freelist's PMR */
	if (psFreeList->psFreeListPMR == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,	"Freelist is not configured for grow"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* grow freelist */
	eError = RGXGrowFreeList(psFreeList,
							ui32NumPages,
							&psFreeList->sMemoryBlockHead);
	if(eError == PVRSRV_OK)
	{
		/* update freelist data in firmware */
		_UpdateFwFreelistSize(psFreeList);

		psFreeList->ui32NumGrowReqByApp++;
	}

	return eError;
}

/*
	RGXRemoveBlockFromFreeListKM
*/

IMG_EXPORT
PVRSRV_ERROR RGXRemoveBlockFromFreeListKM(RGX_FREELIST *psFreeList)
{
	PVRSRV_ERROR eError;

	/* TODO: send command to firmware to move the freelists devvirtaddr (1 block) (mi)*/

	/* TODO: re-arrange pages in freelist (mi) */

	eError = RGXShrinkFreeList(&psFreeList->sMemoryBlockHead,
								psFreeList);

	return eError;
}


/*
	RGXCreateRenderTarget
*/
IMG_EXPORT
PVRSRV_ERROR RGXCreateRenderTarget(PVRSRV_DEVICE_NODE	*psDeviceNode, 
								   IMG_DEV_VIRTADDR		psVHeapTableDevVAddr,
								   RGX_RT_CLEANUP_DATA 	**ppsCleanupData,
								   IMG_UINT32			*sRenderTargetFWDevVAddr)
{
	PVRSRV_ERROR			eError;
	RGXFWIF_RENDER_TARGET	*psRenderTarget;
	RGXFWIF_DEV_VIRTADDR	pFirmwareAddr;
	PVRSRV_RGXDEV_INFO 		*psDevInfo = psDeviceNode->pvDevice;
	RGX_RT_CLEANUP_DATA		*psCleanupData;

	psCleanupData = OSAllocMem(sizeof(*psCleanupData));
	if (psCleanupData == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	OSMemSet(psCleanupData, 0, sizeof(*psCleanupData));
	psCleanupData->psDeviceNode = psDeviceNode;
	/*
	 * This FW render target context is only mapped into kernel for initialisation.
	 * Otherwise this allocation is only used by the FW.
	 * Therefore the GPU cache doesn't need coherency,
	 * and write-combine is suffice on the CPU side (WC buffer will be flushed at the first TA-kick)
	 */
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(*psRenderTarget),
							PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
							PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
							PVRSRV_MEMALLOCFLAG_GPU_READABLE |
							PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
							PVRSRV_MEMALLOCFLAG_CPU_READABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
							PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
							&psCleanupData->psRenderTargetMemDesc);
	if (eError != PVRSRV_OK) 
	{
		PVR_DPF((PVR_DBG_ERROR, "RGXCreateRenderTarget: DevmemAllocate for Render Target failed\n"));
		return eError;
	}
	RGXSetFirmwareAddress(&pFirmwareAddr, psCleanupData->psRenderTargetMemDesc, 0, RFW_FWADDR_FLAG_NONE);
	*sRenderTargetFWDevVAddr = pFirmwareAddr.ui32Addr;

	DevmemAcquireCpuVirtAddr(psCleanupData->psRenderTargetMemDesc, (IMG_VOID **)&psRenderTarget);
	psRenderTarget->psVHeapTableDevVAddr = psVHeapTableDevVAddr;
	PDUMPCOMMENT("Dump RenderTarget");
	DevmemPDumpLoadMem(psCleanupData->psRenderTargetMemDesc, 0, sizeof(*psRenderTarget), PDUMP_FLAGS_CONTINUOUS);
	DevmemReleaseCpuVirtAddr(psCleanupData->psRenderTargetMemDesc);

	*ppsCleanupData = psCleanupData;

	return PVRSRV_OK;
}


/*
	RGXDestroyRenderTarget
*/
IMG_EXPORT
PVRSRV_ERROR RGXDestroyRenderTarget(RGX_RT_CLEANUP_DATA *psCleanupData)
{
	RGXUnsetFirmwareAddress(psCleanupData->psRenderTargetMemDesc);

	/*
		Note:
		When we get RT cleanup in the FW call that instead
	*/
	/* Flush the the SLC before freeing */
	{
		RGXFWIF_KCCB_CMD sFlushInvalCmd;
		PVRSRV_ERROR eError;
		PVRSRV_DEVICE_NODE *psDeviceNode = psCleanupData->psDeviceNode;
	
		/* Schedule the SLC flush command ... */
	#if defined(PDUMP)
		PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Submit SLC flush and invalidate");
	#endif
		sFlushInvalCmd.eCmdType = RGXFWIF_KCCB_CMD_SLCFLUSHINVAL;
		sFlushInvalCmd.uCmdData.sSLCFlushInvalData.bInval = IMG_TRUE;
		sFlushInvalCmd.uCmdData.sSLCFlushInvalData.eDM = RGXFWIF_DM_2D; //Covers all of Sidekick
		sFlushInvalCmd.uCmdData.sSLCFlushInvalData.psContext.ui32Addr = 0;
		
		eError = RGXSendCommandWithPowLock(psDeviceNode->pvDevice,
											RGXFWIF_DM_GP,
											&sFlushInvalCmd,
											sizeof(sFlushInvalCmd),
											IMG_TRUE);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXFreeUFOBlock: Failed to schedule SLC flush command with error (%u)", eError));
		}
		else
		{
			/* Wait for the SLC flush to complete */
			eError = RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, psDeviceNode->psSyncPrim, IMG_TRUE);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"RGXFreeUFOBlock: SLC flush and invalidate aborted with error (%u)", eError));
			}
		}
	}

	DevmemFwFree(psCleanupData->psRenderTargetMemDesc);
	OSFreeMem(psCleanupData);
	return PVRSRV_OK;
}

/*
	RGXCreateZSBuffer
*/
IMG_EXPORT
PVRSRV_ERROR RGXCreateZSBufferKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
								DEVMEMINT_RESERVATION 	*psReservation,
								PMR 					*psPMR,
								PVRSRV_MEMALLOCFLAGS_T 	uiMapFlags,
								RGX_ZSBUFFER_DATA **ppsZSBuffer,
								IMG_UINT32 *pui32ZSBufferFWDevVAddr)
{
	PVRSRV_ERROR				eError;
	PVRSRV_RGXDEV_INFO 			*psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_FWZSBUFFER			*psFWZSBuffer;
	RGX_ZSBUFFER_DATA			*psZSBuffer;
	DEVMEM_MEMDESC				*psFWZSBufferMemDesc;
	IMG_BOOL					bOnDemand = ((uiMapFlags & PVRSRV_MEMALLOCFLAG_NO_OSPAGES_ON_ALLOC) > 0);

	/* Allocate host data structure */
	psZSBuffer = OSAllocMem(sizeof(*psZSBuffer));
	if (psZSBuffer == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateZSBufferKM: Failed to allocate cleanup data structure for ZS-Buffer"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto ErrorAllocCleanup;
	}
	OSMemSet(psZSBuffer, 0, sizeof(*psZSBuffer));

	eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext,
						   &psZSBuffer->psCleanupSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateZSBufferKM: Failed to allocate cleanup sync (0x%x)",
				eError));
		goto ErrorSyncAlloc;
	}

	/* Populate Host data */
	psZSBuffer->psDeviceNode = psDeviceNode;
	psZSBuffer->psReservation = psReservation;
	psZSBuffer->psPMR = psPMR;
	psZSBuffer->uiMapFlags = uiMapFlags;
	psZSBuffer->ui32RefCount = 0;
	psZSBuffer->bOnDemand = bOnDemand;
    if (bOnDemand)
    {
    	psZSBuffer->ui32DeferredAllocID = psDeviceNode->ui32DeferredAllocCurrID++;
    	psZSBuffer->psMapping = IMG_NULL;

		OSLockAcquire(psDeviceNode->hLockZSBuffer);
    	dllist_add_to_tail(&psDeviceNode->sDeferredAllocHead, &psZSBuffer->sNode);
		OSLockRelease(psDeviceNode->hLockZSBuffer);
    }

	/* Allocate firmware memory for ZS-Buffer. */
	PDUMPCOMMENT("Allocate firmware ZS-Buffer data structure");
	eError = DevmemFwAllocate(psDevInfo,
							sizeof(*psFWZSBuffer),
							PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) |
							PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
							PVRSRV_MEMALLOCFLAG_GPU_READABLE |
							PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
							PVRSRV_MEMALLOCFLAG_CPU_READABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
							PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
							PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
							&psFWZSBufferMemDesc);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateZSBufferKM: Failed to allocate firmware ZS-Buffer (%u)", eError));
		goto ErrorAllocFWZSBuffer;
	}
	psZSBuffer->psZSBufferMemDesc = psFWZSBufferMemDesc;

	/* Temporarily map the firmware render context to the kernel. */
	eError = DevmemAcquireCpuVirtAddr(psFWZSBufferMemDesc,
                                      (IMG_VOID **)&psFWZSBuffer);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateZSBufferKM: Failed to map firmware ZS-Buffer (%u)", eError));
		goto ErrorAcquireFWZSBuffer;
	}

	/* Populate FW ZS-Buffer data structure */
	psFWZSBuffer->bOnDemand = bOnDemand;
	psFWZSBuffer->bMapped = !bOnDemand;
	psFWZSBuffer->ui32MappingID = psZSBuffer->ui32DeferredAllocID;

	/* Get firmware address of ZS-Buffer. */
	RGXSetFirmwareAddress(&psZSBuffer->sZSBufferFWDevVAddr, psFWZSBufferMemDesc, 0, RFW_FWADDR_FLAG_NONE);

	/* Dump the ZS-Buffer and the memory content */
	PDUMPCOMMENT("Dump firmware ZS-Buffer");
	DevmemPDumpLoadMem(psFWZSBufferMemDesc, 0, sizeof(*psFWZSBuffer), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(psFWZSBufferMemDesc);


	/* define return value */
	*ppsZSBuffer = psZSBuffer;
	*pui32ZSBufferFWDevVAddr = psZSBuffer->sZSBufferFWDevVAddr.ui32Addr;

	PVR_DPF((PVR_DBG_MESSAGE, "ZS-Buffer [%p] created (%s)",
							psZSBuffer,
							(bOnDemand) ? "On-Demand": "Up-front"));

	return PVRSRV_OK;

	/* error handling */

ErrorAcquireFWZSBuffer:
	DevmemFwFree(psFWZSBufferMemDesc);

ErrorAllocFWZSBuffer:
	SyncPrimFree(psZSBuffer->psCleanupSync);

ErrorSyncAlloc:
	OSFreeMem(psZSBuffer);

ErrorAllocCleanup:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}


/*
	RGXDestroyZSBuffer
*/
IMG_EXPORT
PVRSRV_ERROR RGXDestroyZSBufferKM(RGX_ZSBUFFER_DATA *psZSBuffer)
{
	POS_LOCK hLockZSBuffer;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psZSBuffer);
	hLockZSBuffer = psZSBuffer->psDeviceNode->hLockZSBuffer;

	/* Request ZS Buffer cleanup */
	eError = RGXFWRequestZSBufferCleanUp(psZSBuffer->psDeviceNode,
										psZSBuffer->sZSBufferFWDevVAddr,
										psZSBuffer->psCleanupSync);
	if (eError != PVRSRV_ERROR_RETRY)
	{
		/* print some statistics */
		if ((psZSBuffer->bOnDemand) &&
				((psZSBuffer->ui32NumReqByApp > 0) || (psZSBuffer->ui32NumReqByFW > 0)))
		{
				PVR_LOG(("Z/S destroyed [0x%p]: Backing requests by App = %u, Backing requests by Firmware = %u",
							psZSBuffer,
							psZSBuffer->ui32NumReqByApp,
							psZSBuffer->ui32NumReqByFW));
		}

		/* Free the firmware render context. */
    	RGXUnsetFirmwareAddress(psZSBuffer->psZSBufferMemDesc);
		DevmemFwFree(psZSBuffer->psZSBufferMemDesc);

	    /* Remove Deferred Allocation from list */
		if (psZSBuffer->bOnDemand)
		{
			OSLockAcquire(hLockZSBuffer);
			PVR_ASSERT(dllist_node_is_in_list(&psZSBuffer->sNode));
			dllist_remove_node(&psZSBuffer->sNode);
			OSLockRelease(hLockZSBuffer);
		}

		SyncPrimFree(psZSBuffer->psCleanupSync);

		PVR_ASSERT(psZSBuffer->ui32RefCount == 0);

		PVR_DPF((PVR_DBG_MESSAGE,"ZS-Buffer [%p] destroyed",psZSBuffer));

		/* Free ZS-Buffer host data structure */
		OSFreeMem(psZSBuffer);

	}

	return eError;
}

PVRSRV_ERROR
RGXBackingZSBuffer(RGX_ZSBUFFER_DATA *psZSBuffer)
{
	POS_LOCK hLockZSBuffer;
	PVRSRV_ERROR eError;

	if (!psZSBuffer)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if ((psZSBuffer->uiMapFlags & PVRSRV_MEMALLOCFLAG_NO_OSPAGES_ON_ALLOC) == 0)
	{
		/* Only deferred allocations can be populated */
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	PVR_DPF((PVR_DBG_MESSAGE,"ZS Buffer [%p, ID=0x%08x]: Physical backing requested",
								psZSBuffer,
								psZSBuffer->ui32DeferredAllocID));
	hLockZSBuffer = psZSBuffer->psDeviceNode->hLockZSBuffer;

	OSLockAcquire(hLockZSBuffer);

	if (psZSBuffer->ui32RefCount == 0)
	{
		if (psZSBuffer->bOnDemand)
		{
			IMG_HANDLE hDevmemHeap;

			PVR_ASSERT(psZSBuffer->psMapping == IMG_NULL);

			/* Get Heap */
			eError = DevmemServerGetHeapHandle(psZSBuffer->psReservation, &hDevmemHeap);
			PVR_ASSERT(psZSBuffer->psMapping == IMG_NULL);

			eError = DevmemIntMapPMR(hDevmemHeap,
									psZSBuffer->psReservation,
									psZSBuffer->psPMR,
									psZSBuffer->uiMapFlags,
									&psZSBuffer->psMapping);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"Unable populate ZS Buffer [%p, ID=0x%08x] with error %u",
										psZSBuffer,
										psZSBuffer->ui32DeferredAllocID,
										eError));
				OSLockRelease(hLockZSBuffer);
				return eError;

			}
			PVR_DPF((PVR_DBG_MESSAGE, "ZS Buffer [%p, ID=0x%08x]: Physical backing acquired",
										psZSBuffer,
										psZSBuffer->ui32DeferredAllocID));
		}
	}

	/* Increase refcount*/
	psZSBuffer->ui32RefCount++;

	OSLockRelease(hLockZSBuffer);

	return PVRSRV_OK;
}


PVRSRV_ERROR
RGXPopulateZSBufferKM(RGX_ZSBUFFER_DATA *psZSBuffer,
					RGX_POPULATION **ppsPopulation)
{
	RGX_POPULATION *psPopulation;
	PVRSRV_ERROR eError;

	psZSBuffer->ui32NumReqByApp++;

	/* Do the backing */
	eError = RGXBackingZSBuffer(psZSBuffer);
	if (eError != PVRSRV_OK)
	{
		goto OnErrorBacking;
	}

	/* Create the handle to the backing */
	psPopulation = OSAllocMem(sizeof(*psPopulation));
	if (psPopulation == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto OnErrorAlloc;
	}

	psPopulation->psZSBuffer = psZSBuffer;

	/* return value */
	*ppsPopulation = psPopulation;

	return PVRSRV_OK;

OnErrorAlloc:
	RGXUnbackingZSBuffer(psZSBuffer);

OnErrorBacking:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR
RGXUnbackingZSBuffer(RGX_ZSBUFFER_DATA *psZSBuffer)
{
	POS_LOCK hLockZSBuffer;
	PVRSRV_ERROR eError;

	if (!psZSBuffer)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	PVR_ASSERT(psZSBuffer->ui32RefCount);

	PVR_DPF((PVR_DBG_MESSAGE,"ZS Buffer [%p, ID=0x%08x]: Physical backing removal requested",
								psZSBuffer,
								psZSBuffer->ui32DeferredAllocID));

	hLockZSBuffer = psZSBuffer->psDeviceNode->hLockZSBuffer;

	OSLockAcquire(hLockZSBuffer);

	if (psZSBuffer->bOnDemand)
	{
		if (psZSBuffer->ui32RefCount == 1)
		{
			PVR_ASSERT(psZSBuffer->psMapping);

			eError = DevmemIntUnmapPMR(psZSBuffer->psMapping);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"Unable to unpopulate ZS Buffer [%p, ID=0x%08x] with error %u",
										psZSBuffer,
										psZSBuffer->ui32DeferredAllocID,
										eError));
				OSLockRelease(hLockZSBuffer);
				return eError;
			}

			PVR_DPF((PVR_DBG_MESSAGE, "ZS Buffer [%p, ID=0x%08x]: Physical backing removed",
										psZSBuffer,
										psZSBuffer->ui32DeferredAllocID));
		}
	}

	/* Decrease refcount*/
	psZSBuffer->ui32RefCount--;

	OSLockRelease(hLockZSBuffer);

	return PVRSRV_OK;
}

PVRSRV_ERROR
RGXUnpopulateZSBufferKM(RGX_POPULATION *psPopulation)
{
	PVRSRV_ERROR eError;

	if (!psPopulation)
	{
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eError = RGXUnbackingZSBuffer(psPopulation->psZSBuffer);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	OSFreeMem(psPopulation);

	return PVRSRV_OK;
}


/*
 * PVRSRVRGXCreateRenderContextKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateRenderContextKM(PVRSRV_DEVICE_NODE		*psDeviceNode,
											DEVMEM_MEMDESC 			*psTACCBMemDesc,
											DEVMEM_MEMDESC 			*psTACCBCtlMemDesc,
											DEVMEM_MEMDESC 			*ps3DCCBMemDesc,
											DEVMEM_MEMDESC 			*ps3DCCBCtlMemDesc,
											RGX_RC_CLEANUP_DATA		**ppsCleanupData,
											DEVMEM_MEMDESC 			**ppsFWRenderContextMemDesc,
											DEVMEM_MEMDESC 			**ppsFWRenderContextStateMemDesc,
											IMG_UINT32				ui32Priority,
											IMG_DEV_VIRTADDR		sMCUFenceAddr,
											IMG_DEV_VIRTADDR		sVDMCallStackAddr,
											IMG_UINT32				ui32FrameworkRegisterSize,
											IMG_PBYTE				pbyFrameworkRegisters,
											IMG_HANDLE				hMemCtxPrivData)
{
	PVRSRV_ERROR			eError;
	PVRSRV_RGXDEV_INFO 		*psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_FWRENDERCONTEXT	*psFWRenderContext;
	RGX_RC_CLEANUP_DATA		*psTmpCleanup;
	RGXFWIF_TACTX_STATE		*psContextState;
	DEVMEM_MEMDESC			*psFWFrameworkMemDesc;

	/* Prepare cleanup structure */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/* Allocate cleanup sync */
	eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext,
						   &psTmpCleanup->psCleanupSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateComputeContextKM: Failed to allocate cleanup sync (0x%x)",
				eError));
		goto fail_syncalloc;
	}

	/*
		Allocate device memory for the firmware render context.
	*/
	PDUMPCOMMENT("Allocate RGX firmware render context");

	eError = DevmemFwAllocate(psDevInfo,
							sizeof(*psFWRenderContext),
							RGX_FWCOMCTX_ALLOCFLAGS,
							ppsFWRenderContextMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to allocate firmware render context (%u)",
				eError));
		goto fail_contextalloc;
	}
	psTmpCleanup->psFWRenderContextMemDesc = *ppsFWRenderContextMemDesc;
	psTmpCleanup->psDeviceNode = psDeviceNode;

	/*
		Temporarily map the firmware render context to the kernel.
	*/
	eError = DevmemAcquireCpuVirtAddr(*ppsFWRenderContextMemDesc,
                                      (IMG_VOID **)&psFWRenderContext);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context (%u)",
				eError));
		goto fail_cpuvirtacquire;
	}

	/*
		Allocate device memory for the firmware GPU context suspend state.
		Note: the FW reads/writes the state to memory by accessing the GPU register interface.
	*/
	PDUMPCOMMENT("Allocate RGX firmware 3D context suspend state");

	eError = DevmemFwAllocate(psDevInfo,
							sizeof(RGXFWIF_CTX_STATE),
							RGX_FWCOMCTX_ALLOCFLAGS,
							ppsFWRenderContextStateMemDesc);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to allocate firmware GPU context suspend state (%u)",
				eError));
		goto fail_contextsuspendalloc;
	}
	psTmpCleanup->psFWRenderContextStateMemDesc = *ppsFWRenderContextStateMemDesc;

	/* 
	 * Create the FW framework buffer
	 */
	eError = PVRSRVRGXFrameworkCreateKM(psDeviceNode, & psFWFrameworkMemDesc, ui32FrameworkRegisterSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to allocate firmware GPU framework state (%u)",
				eError));
		goto fail_frameworkcreate;
	}
	
	psTmpCleanup->psFWFrameworkMemDesc = psFWFrameworkMemDesc;

	/* Copy the Framework client data into the framework buffer */
	eError = PVRSRVRGXFrameworkCopyRegisters(psFWFrameworkMemDesc, pbyFrameworkRegisters, ui32FrameworkRegisterSize);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to populate the framework buffer (%u)",
				eError));
		goto fail_frameworkcopy;
	}

	/* Init TA FW common context */
	eError = RGXInitFWCommonContext(&psFWRenderContext->sTAContext,
									psTACCBMemDesc,
									psTACCBCtlMemDesc,
									hMemCtxPrivData,
									psFWFrameworkMemDesc,
									ui32Priority,
									&sMCUFenceAddr,
									&psTmpCleanup->sFWTAContextCleanup);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to init TA fw common context (%u)",
				eError));
		goto fail_tacontextinit;
	}

	/* Init 3D FW common context */
	eError = RGXInitFWCommonContext(&psFWRenderContext->s3DContext,
									ps3DCCBMemDesc,
									ps3DCCBCtlMemDesc,
									hMemCtxPrivData,
									psFWFrameworkMemDesc,
									ui32Priority,
									&sMCUFenceAddr,
									&psTmpCleanup->sFW3DContextCleanup);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to init 3D fw common context (%u)",
				eError));
		goto fail_3dcontextinit;
	}
	
	/*
	 * Set the firmware GPU context state buffer. Note the offset allows the same buffer
	 * to be used by the TA and 3D separately by specifying custom offset. The
	 * buffer itself is allocated/deallocated with the granularity of the parent
	 * structure.
	 * 
	 * The common context stores a dword pointer (FW) so we can cast the generic buffer to
	 * the correct TA/3D state structure type in the FW.
	 */
	RGXSetFirmwareAddress(&psFWRenderContext->sTAContext.psContextState,
						  *ppsFWRenderContextStateMemDesc,
						  offsetof(RGXFWIF_CTX_STATE, sTAContextState),	/* TA state */
						  RFW_FWADDR_FLAG_NONE);

	eError = DevmemAcquireCpuVirtAddr(*ppsFWRenderContextStateMemDesc,
                                      (IMG_VOID **)&psContextState);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context state (%u)",
				eError));
		goto fail_suspendcpuvirtacquire;
	}
	psContextState->uTAReg_VDM_CALL_STACK_POINTER = sVDMCallStackAddr.uiAddr;

	RGXSetFirmwareAddress(&psFWRenderContext->s3DContext.psContextState,
						  *ppsFWRenderContextStateMemDesc,
						  offsetof(RGXFWIF_CTX_STATE, s3DContextState),	/* 3D state */
						  RFW_FWADDR_FLAG_NONE);

	/*
	 * Dump the Render and the memory contexts
	 */
	PDUMPCOMMENT("Dump FWRenderContext");
	DevmemPDumpLoadMem(*ppsFWRenderContextMemDesc, 0, sizeof(*psFWRenderContext), PDUMP_FLAGS_CONTINUOUS);

	/*
	 * Dump the FW TA/3D context suspend state buffer
	 */
	PDUMPCOMMENT("Dump FWRenderContextState");
	DevmemPDumpLoadMem(*ppsFWRenderContextStateMemDesc, 0, sizeof(RGXFWIF_CTX_STATE), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(*ppsFWRenderContextMemDesc);
	DevmemReleaseCpuVirtAddr(*ppsFWRenderContextStateMemDesc);

	return PVRSRV_OK;

fail_suspendcpuvirtacquire:
	RGXUnsetFirmwareAddress(*ppsFWRenderContextStateMemDesc);	/* TA state */
	RGXDeinitFWCommonContext(&psTmpCleanup->sFW3DContextCleanup);
fail_3dcontextinit:
	RGXDeinitFWCommonContext(&psTmpCleanup->sFWTAContextCleanup);
fail_tacontextinit:
fail_frameworkcopy:
	DevmemFwFree(psFWFrameworkMemDesc);
fail_frameworkcreate:
	DevmemFwFree(*ppsFWRenderContextStateMemDesc);
fail_contextsuspendalloc:
	DevmemReleaseCpuVirtAddr(*ppsFWRenderContextMemDesc);
fail_cpuvirtacquire:
	DevmemFwFree(*ppsFWRenderContextMemDesc);
fail_contextalloc:
	SyncPrimFree(psTmpCleanup->psCleanupSync);
fail_syncalloc:
	OSFreeMem(psTmpCleanup);

	return eError;
}


/*
 * PVRSRVRGXDestroyRenderContextKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXDestroyRenderContextKM(RGX_RC_CLEANUP_DATA *psCleanupData)
{
	PVRSRV_ERROR				eError;
	PRGXFWIF_FWCOMMONCONTEXT	psCommonContext;
	RGXFWIF_FWRENDERCONTEXT	*psFWRenderContext;

	/* Cleanup the TA if we haven't already */
	if ((psCleanupData->ui32CleanupStatus & RC_CLEANUP_TA_COMPLETE) == 0)
	{
		/* Request a flush out and cleanup for TA */
		RGXSetFirmwareAddress(&psCommonContext,
								psCleanupData->psFWRenderContextMemDesc,
								offsetof(RGXFWIF_FWRENDERCONTEXT, sTAContext),
								RFW_FWADDR_NOREF_FLAG);

		eError = RGXFWRequestCommonContextCleanUp(psCleanupData->psDeviceNode,
												  psCommonContext,
												  psCleanupData->psCleanupSync,
												  RGXFWIF_DM_TA);
		if (eError != PVRSRV_ERROR_RETRY)
		{
			eError = RGXDeinitFWCommonContext(&psCleanupData->sFWTAContextCleanup);

			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXDestroyRenderContextKM : failed to deinit TA fw common ctx. Error:%u", eError));
				goto e0;
			}

#if defined(DEBUG)
			/* Log the number of TA context stores which occurred */
			{
				RGXFWIF_CTX_STATE	*psFWState;

				eError = DevmemAcquireCpuVirtAddr(psCleanupData->psFWRenderContextStateMemDesc,
												  (IMG_VOID**)&psFWState);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context state (%u)",
							eError));
				}
				else
				{
					PVR_DPF((PVR_DBG_WARNING,"Number of context stores on FW TA context 0x%010x: %u",
							psCommonContext.ui32Addr,
							psFWState->sTAContextState.ui32NumStores));
	
					/* Release the CPU virt addr */
					DevmemReleaseCpuVirtAddr(psCleanupData->psFWRenderContextStateMemDesc);
				}
			}
#endif

			RGXUnsetFirmwareAddress(psCleanupData->psFWRenderContextStateMemDesc);	/* TA state */
			psCleanupData->ui32CleanupStatus |= RC_CLEANUP_TA_COMPLETE;
		}
		else
		{
			goto e0;
		}
	}

	/* Cleanup the 3D if we haven't already */
	if ((psCleanupData->ui32CleanupStatus & RC_CLEANUP_3D_COMPLETE) == 0)
	{
		/* Request a flush out and cleanup for 3D */
		RGXSetFirmwareAddress(&psCommonContext,
								psCleanupData->psFWRenderContextMemDesc,
								offsetof(RGXFWIF_FWRENDERCONTEXT, s3DContext),
								RFW_FWADDR_NOREF_FLAG);

		eError = RGXFWRequestCommonContextCleanUp(psCleanupData->psDeviceNode,
												  psCommonContext,
												  psCleanupData->psCleanupSync,
												  RGXFWIF_DM_3D);
		if (eError != PVRSRV_ERROR_RETRY)
		{
			eError = RGXDeinitFWCommonContext(&psCleanupData->sFW3DContextCleanup);
	
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXDestroyRenderContextKM : failed to deinit 3D fw common ctx. Error:%u", eError));
				goto e0;
			}
	
#if defined(DEBUG)
			/* Log the number of 3D context stores which occurred */
			{
				RGXFWIF_CTX_STATE	*psFWState;
				
				eError = DevmemAcquireCpuVirtAddr(psCleanupData->psFWRenderContextStateMemDesc,
												  (IMG_VOID**)&psFWState);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context state (%u)",
							eError));
				}
				else
				{
					PVR_DPF((PVR_DBG_WARNING,"Number of context stores on FW 3D context 0x%010x: %u",
							psCommonContext.ui32Addr,
							psFWState->s3DContextState.ui32NumStores));

					/* Release the CPU virt addr */
					DevmemReleaseCpuVirtAddr(psCleanupData->psFWRenderContextStateMemDesc);
				}
			}
#endif

			/* Unmap the TA/3D context state buffer pointers */
			RGXUnsetFirmwareAddress(psCleanupData->psFWRenderContextStateMemDesc);	/* 3D state */
			psCleanupData->ui32CleanupStatus |= RC_CLEANUP_3D_COMPLETE;
		}
		else
		{
			goto e0;
		}
	}

	/*
		Only if both TA and 3D contexts have been cleaned up can we
		free the shared resources
	*/
	if (psCleanupData->ui32CleanupStatus == (RC_CLEANUP_3D_COMPLETE | RC_CLEANUP_TA_COMPLETE))
	{
		/* Print some SPM statistics */
		eError = DevmemAcquireCpuVirtAddr(psCleanupData->psFWRenderContextMemDesc,
	                                      (IMG_VOID **)&psFWRenderContext);
		if (eError == PVRSRV_OK)
		{
			if ((psFWRenderContext->ui32TotalNumPartialRenders > 0) ||
				(psFWRenderContext->ui32TotalNumOutOfMemory > 0))
			{
				PVR_LOG(("RC destroyed [0x%p]: Number of OOMs = %u, Number of partial renders = %u",
						psFWRenderContext,
						psFWRenderContext->ui32TotalNumOutOfMemory,
						psFWRenderContext->ui32TotalNumPartialRenders));
			}

			DevmemReleaseCpuVirtAddr(psCleanupData->psFWRenderContextMemDesc);
		}
		else
		{
			PVR_DPF((PVR_DBG_ERROR,"PVRSRVRGXDestroyRenderContextKM: Failed to map firmware render context (%u)",
					eError));
		}

		/* Free the framework buffer */
		DevmemFwFree(psCleanupData->psFWFrameworkMemDesc);
	
		/* Free the firmware TA/3D context state buffer */
		DevmemFwFree(psCleanupData->psFWRenderContextStateMemDesc);
	
		/* Free the firmware render context */
		DevmemFwFree(psCleanupData->psFWRenderContextMemDesc);

		/* Free the cleanup sync */
		SyncPrimFree(psCleanupData->psCleanupSync);

		OSFreeMem(psCleanupData);
	}

	return PVRSRV_OK;

e0:
	return eError;
}

/*
 * PVRSRVRGXKickTA3DKM
 */
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXKickTA3DKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
								 DEVMEM_MEMDESC 	*psFWRenderContextMemDesc,
								 IMG_BOOL			bLastTAInScene,
								 IMG_BOOL			bKickTA,
								 IMG_BOOL			bKick3D,
								 IMG_UINT32			ui32TAcCCBWoffUpdate,
								 IMG_UINT32			ui323DcCCBWoffUpdate,
								 IMG_BOOL			bbPDumpContinuous)
{
	PVRSRV_ERROR			eError = 0;
	RGXFWIF_KCCB_CMD		sTACCBCmd;
	RGXFWIF_KCCB_CMD		s3DCCBCmd;

	if(bKickTA)
	{
		/*
		 * Construct the kernel TA CCB command.
		 * (Safe to release reference to render context virtual address because
		 * render context destruction must flush the firmware).
		 */
		sTACCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
		RGXSetFirmwareAddress(&sTACCBCmd.uCmdData.sCmdKickData.psContext, psFWRenderContextMemDesc,
						  offsetof(RGXFWIF_FWRENDERCONTEXT, sTAContext), RFW_FWADDR_NOREF_FLAG);
		sTACCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = ui32TAcCCBWoffUpdate;

		/*
		 * Submit the TA command to the firmware.
		 */
		eError = RGXScheduleCommand(psDeviceNode->pvDevice,
									RGXFWIF_DM_TA,
									&sTACCBCmd,
									sizeof(sTACCBCmd),
									bbPDumpContinuous);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXKickTA3DKM failed to schedule kernel TA command. Error:%u", eError));
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
	}

	if (bKick3D)
	{
		/*
		 * Construct the kernel 3D CCB command.
		 * (Safe to release reference to render context virtual address because
		 * render context destruction must flush the firmware).
		 */
		s3DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
		RGXSetFirmwareAddress(&s3DCCBCmd.uCmdData.sCmdKickData.psContext,
							  psFWRenderContextMemDesc,
							  offsetof(RGXFWIF_FWRENDERCONTEXT, s3DContext),
							  RFW_FWADDR_NOREF_FLAG);
		s3DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = ui323DcCCBWoffUpdate;

		/*
		 * Submit the 3D command to the firmware.
		 */
		eError = RGXScheduleCommand(psDeviceNode->pvDevice,
									RGXFWIF_DM_3D,
									&s3DCCBCmd,
									sizeof(s3DCCBCmd),
									bbPDumpContinuous);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXKickTA3DKM failed to schedule kernel 3D command. Error:%u", eError));
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
	}


PVRSRVRGXKickTA3DKM_Exit:
	return eError;
}


/******************************************************************************
 End of file (rgxta3d.c)
******************************************************************************/
