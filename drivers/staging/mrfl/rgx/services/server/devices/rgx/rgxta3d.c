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
#include "rgxccb.h"

#include "rgxdefs_km.h"
#include "rgx_fwif_km.h"
#include "physmem.h"
#include "sync_server.h"
#include "sync_internal.h"

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
#include "pvr_sync.h"
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

typedef struct _DEVMEM_REF_LOOKUP_
{
	IMG_UINT32 ui32ZSBufferID;
	RGX_ZSBUFFER_DATA *psZSBuffer;
} DEVMEM_REF_LOOKUP;

typedef struct _DEVMEM_FREELIST_LOOKUP_
{
	IMG_UINT32 ui32FreeListID;
	RGX_FREELIST *psFreeList;
} DEVMEM_FREELIST_LOOKUP;

static IMG_BOOL _RGXDumpPMRPageList(PDLLIST_NODE psNode, IMG_PVOID pvCallbackData)
{
	RGX_PMR_NODE *psPMRNode = IMG_CONTAINER_OF(psNode, RGX_PMR_NODE, sMemoryBlock);
	PVRSRV_ERROR			eError;

	eError = PMRDumpPageList(psPMRNode->psPMR,
							RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"Error (%u) printing pmr %p", eError, psPMRNode->psPMR));
	}

	return IMG_TRUE;
}

IMG_BOOL RGXDumpFreeListPageList(RGX_FREELIST *psFreeList)
{
	PVR_LOG(("Freelist FWAddr 0x%08x, ID = %d, CheckSum 0x%016llx",
				psFreeList->sFreeListFWDevVAddr.ui32Addr,
				psFreeList->ui32FreelistID,
				psFreeList->ui64FreelistChecksum));

	/* Dump Init FreeList page list */
	PVR_LOG(("  Initial Memory block"));
	dllist_foreach_node(&psFreeList->sMemoryBlockInitHead,
					_RGXDumpPMRPageList,
					IMG_NULL);

	/* Dump Grow FreeList page list */
	PVR_LOG(("  Grow Memory blocks"));
	dllist_foreach_node(&psFreeList->sMemoryBlockHead,
					_RGXDumpPMRPageList,
					IMG_NULL);

	return IMG_TRUE;
}

static PVRSRV_ERROR _UpdateFwFreelistSize(RGX_FREELIST *psFreeList,
										IMG_BOOL bGrow,
										IMG_UINT32 ui32DeltaSize)
{
	PVRSRV_ERROR			eError;
	RGXFWIF_KCCB_CMD		sGPCCBCmd;

	sGPCCBCmd.eCmdType = (bGrow) ? RGXFWIF_KCCB_CMD_FREELIST_GROW_UPDATE : RGXFWIF_KCCB_CMD_FREELIST_SHRINK_UPDATE;
	sGPCCBCmd.uCmdData.sFreeListGSData.psFreeListFWDevVAddr = psFreeList->sFreeListFWDevVAddr.ui32Addr;
	sGPCCBCmd.uCmdData.sFreeListGSData.ui32DeltaSize = ui32DeltaSize;
	sGPCCBCmd.uCmdData.sFreeListGSData.ui32NewSize = psFreeList->ui32CurrentFLPages;

	PVR_DPF((PVR_DBG_MESSAGE, "Send FW update: freelist [FWAddr=0x%08x] has 0x%08x pages",
								psFreeList->sFreeListFWDevVAddr.ui32Addr,
								psFreeList->ui32CurrentFLPages));

	/* Submit command to the firmware.  */
	eError = RGXScheduleCommand(psFreeList->psDevInfo,
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

static PVRSRV_ERROR _FreeListCheckSum(RGX_FREELIST *psFreeList,
                   	   	   	   	   	   IMG_UINT64 *pui64CheckSum)
{
#if defined(NO_HARDWARE)
	/* No checksum needed as we have all information in the pdumps */
	PVR_UNREFERENCED_PARAMETER(psFreeList);
	*pui64CheckSum = 0;
	return PVRSRV_OK;
#else
	PVRSRV_ERROR eError;
	IMG_SIZE_T uiNumBytes;
    IMG_UINT8* pui8Buffer;
    IMG_UINT32* pui32Buffer;
    IMG_UINT32 ui32CheckSumAdd = 0;
    IMG_UINT32 ui32CheckSumXor = 0;
    IMG_UINT32 ui32Entry;
    IMG_UINT32 ui32Entry2;

	/* Allocate Buffer of the size of the freelist */
	pui8Buffer = OSAllocMem(psFreeList->ui32CurrentFLPages * sizeof(IMG_UINT32));
    if (pui8Buffer == IMG_NULL)
    {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
        goto _OSAllocMem_Exit;
    }

    /* Copy freelist content into Buffer */
    eError = PMR_ReadBytes(psFreeList->psFreeListPMR,
    				psFreeList->uiFreeListPMROffset + (psFreeList->ui32MaxFLPages - psFreeList->ui32CurrentFLPages) * sizeof(IMG_UINT32),
    				pui8Buffer,
    				psFreeList->ui32CurrentFLPages * sizeof(IMG_UINT32),
            		&uiNumBytes);
    if (eError != PVRSRV_OK)
    {
    	goto _PMR_ReadBytes_Exit;
    }

    PVR_ASSERT(uiNumBytes == psFreeList->ui32CurrentFLPages * sizeof(IMG_UINT32));

    /* Generate checksum */
    pui32Buffer = (IMG_UINT32 *)pui8Buffer;
    for(ui32Entry = 0; ui32Entry < psFreeList->ui32CurrentFLPages; ui32Entry++)
    {
    	ui32CheckSumAdd += pui32Buffer[ui32Entry];
    	ui32CheckSumXor ^= pui32Buffer[ui32Entry];

    	/* Check for double entries */
    	for (ui32Entry2 = 0; ui32Entry2 < psFreeList->ui32CurrentFLPages; ui32Entry2++)
    	{
			if ((ui32Entry != ui32Entry2) &&
				(pui32Buffer[ui32Entry] == pui32Buffer[ui32Entry2]))
			{
				PVR_DPF((PVR_DBG_ERROR, "Freelist consistency failure: FW addr: 0x%08X, Double entry found 0x%08x on idx: %d and %d",
											psFreeList->sFreeListFWDevVAddr.ui32Addr,
											pui32Buffer[ui32Entry2],
											ui32Entry,
											ui32Entry2));
				while(1)
				{
					OSSleepms(1000);
				}
//				PVR_ASSERT(0);
			}
    	}
    }

    OSFreeMem(pui8Buffer);

    /* Set return value */
    *pui64CheckSum = ((IMG_UINT64)ui32CheckSumXor << 32) | ui32CheckSumAdd;
    PVR_ASSERT(eError == PVRSRV_OK);
    return PVRSRV_OK;

    /*
      error exit paths follow
    */

_PMR_ReadBytes_Exit:
	OSFreeMem(pui8Buffer);

_OSAllocMem_Exit:
    PVR_ASSERT(eError != PVRSRV_OK);
    return eError;
#endif
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
	IMG_UINT64 ui64CheckSum;
	IMG_UINT32 ui32CheckSumXor;
	IMG_UINT32 ui32CheckSumAdd;

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
	OSLockAcquire(psFreeList->psDevInfo->hLockFreeList);


	psPMRNode->ui32NumPages = ui32NumPages;
	psPMRNode->psFreeList = psFreeList;

	/* Allocate Memory Block */
	PDUMPCOMMENT("Allocate PB Block (Pages %08X)", ui32NumPages);
	uiSize = ui32NumPages * RGX_BIF_PM_PHYSICAL_PAGE_SIZE;
	eError = PhysmemNewRamBackedPMR(psFreeList->psDevInfo->psDeviceNode,
									uiSize,
									uiSize,
									1,
									1,
									&bMappingTable,
									RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT,
									PVRSRV_MEMALLOCFLAG_GPU_READABLE,
									&psPMRNode->psPMR);
	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
				 "RGXGrowFreeList: Failed to allocate PB block of size: 0x%016llX",
				 (IMG_UINT64)uiSize));
		goto ErrorBlockAlloc;
	}

	/* Zeroing physical pages pointed by the PMR */
	if (psFreeList->psDevInfo->ui32DeviceFlags & RGXKM_DEVICE_STATE_ZERO_FREELIST)
	{
		eError = PMRZeroingPMR(psPMRNode->psPMR, RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"RGXGrowFreeList: Failed to zero PMR %p of freelist %p with Error %d",
									psPMRNode->psPMR,
									psFreeList,
									eError));
			PVR_ASSERT(0);
		}
	}

	uiLength = psPMRNode->ui32NumPages * sizeof(IMG_UINT32);
	uistartPage = (psFreeList->ui32MaxFLPages - psFreeList->ui32CurrentFLPages - psPMRNode->ui32NumPages);
	uiOffset = psFreeList->uiFreeListPMROffset + (uistartPage * sizeof(IMG_UINT32));

   	/* write Freelist with Memory Block physical addresses */
	eError = PMRWritePMPageList(
						/* Target PMR, offset, and length */
						psFreeList->psFreeListPMR,
						uiOffset,
						uiLength,
						/* Referenced PMR, and "page" granularity */
						psPMRNode->psPMR,
						RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT,
						&psPMRNode->psPageList,
						&ui64CheckSum);
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

	if (psFreeList->bCheckFreelist)
	{
		/* Update checksum */
		ui32CheckSumAdd = (IMG_UINT32)(psFreeList->ui64FreelistChecksum + ui64CheckSum);
		ui32CheckSumXor = (IMG_UINT32)((psFreeList->ui64FreelistChecksum  ^ ui64CheckSum) >> 32);
		psFreeList->ui64FreelistChecksum = ((IMG_UINT64)ui32CheckSumXor << 32) | ui32CheckSumAdd;
		/* Note: We can't do a freelist check here, because the freelist is probably empty (OOM) */
	}

	OSLockRelease(psFreeList->psDevInfo->hLockFreeList);

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
	OSLockRelease(psFreeList->psDevInfo->hLockFreeList);

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
	PVR_ASSERT(psFreeList->psDevInfo);
	PVR_ASSERT(psFreeList->psDevInfo->hLockFreeList);

	OSLockAcquire(psFreeList->psDevInfo->hLockFreeList);

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

	OSLockRelease(psFreeList->psDevInfo->hLockFreeList);

	return PVRSRV_OK;
}

static IMG_BOOL _FindFreeList(PDLLIST_NODE psNode, IMG_PVOID pvCallbackData)
{
	DEVMEM_FREELIST_LOOKUP *psRefLookUp = (DEVMEM_FREELIST_LOOKUP *)pvCallbackData;
	RGX_FREELIST *psFreeList;

	psFreeList = IMG_CONTAINER_OF(psNode, RGX_FREELIST, sNode);

	if (psFreeList->ui32FreelistID == psRefLookUp->ui32FreeListID)
	{
		psRefLookUp->psFreeList = psFreeList;
		return IMG_FALSE;
	}
	else
	{
		return IMG_TRUE;
	}
}

IMG_VOID RGXProcessRequestGrow(PVRSRV_RGXDEV_INFO *psDevInfo,
								IMG_UINT32 ui32FreelistID)
{
	DEVMEM_FREELIST_LOOKUP sLookUp;
	RGXFWIF_KCCB_CMD s3DCCBCmd;
	IMG_UINT32 ui32GrowValue;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psDevInfo);

	/* find the freelist with the corresponding ID */
	sLookUp.ui32FreeListID = ui32FreelistID;
	sLookUp.psFreeList = IMG_NULL;

	OSLockAcquire(psDevInfo->hLockFreeList);
	dllist_foreach_node(&psDevInfo->sFreeListHead, _FindFreeList, (IMG_PVOID)&sLookUp);
	OSLockRelease(psDevInfo->hLockFreeList);

	if (sLookUp.psFreeList)
	{
		RGX_FREELIST *psFreeList = sLookUp.psFreeList;

		/* Try to grow the freelist */
		eError = RGXGrowFreeList(psFreeList,
								psFreeList->ui32GrowFLPages,
								&psFreeList->sMemoryBlockHead);
		if (eError == PVRSRV_OK)
		{
			/* Grow successful, return size of grow size */
			ui32GrowValue = psFreeList->ui32GrowFLPages;

			psFreeList->ui32NumGrowReqByFW++;
		}
		else
		{
			/* Grow failed */
			ui32GrowValue = 0;
			PVR_DPF((PVR_DBG_ERROR,"Grow for FreeList %p failed (error %u)",
									psFreeList,
									eError));
		}

		/* send feedback */
		s3DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_FREELIST_GROW_UPDATE;
		s3DCCBCmd.uCmdData.sFreeListGSData.psFreeListFWDevVAddr = sLookUp.psFreeList->sFreeListFWDevVAddr.ui32Addr;
		s3DCCBCmd.uCmdData.sFreeListGSData.ui32DeltaSize = ui32GrowValue;
		s3DCCBCmd.uCmdData.sFreeListGSData.ui32NewSize = psFreeList->ui32CurrentFLPages;
		eError = RGXScheduleCommand(psDevInfo,
											RGXFWIF_DM_3D,
											&s3DCCBCmd,
											sizeof(s3DCCBCmd),
											IMG_FALSE);

		/* Kernel CCB should never fill up, as the FW is processing them right away  */
		PVR_ASSERT(eError == PVRSRV_OK);
	}
	else
	{
		/* Should never happen */
		PVR_DPF((PVR_DBG_ERROR,"FreeList Lookup for FreeList ID 0x%08x failed (Populate)", sLookUp.ui32FreeListID));
		PVR_ASSERT(IMG_FALSE);
	}
}

static IMG_BOOL _RGXCheckFreeListReconstruction(PDLLIST_NODE psNode, IMG_PVOID pvCallbackData)
{

	PVRSRV_RGXDEV_INFO 		*psDevInfo;
	RGX_FREELIST			*psFreeList;
	RGX_PMR_NODE			*psPMRNode;
	PVRSRV_ERROR			eError;
	IMG_UINT32				uiOffset;
	IMG_UINT32				uiLength;
	IMG_UINT32				uistartPage;
	IMG_UINT64				ui64CheckSum;

	psPMRNode = IMG_CONTAINER_OF(psNode, RGX_PMR_NODE, sMemoryBlock);
	psFreeList = psPMRNode->psFreeList;
	PVR_ASSERT(psFreeList);
	psDevInfo = psFreeList->psDevInfo;
	PVR_ASSERT(psDevInfo);

	uiLength = psPMRNode->ui32NumPages * sizeof(IMG_UINT32);
	uistartPage = (psFreeList->ui32MaxFLPages - psFreeList->ui32CurrentFLPages - psPMRNode->ui32NumPages);
	uiOffset = psFreeList->uiFreeListPMROffset + (uistartPage * sizeof(IMG_UINT32));

	PMRUnwritePMPageList(psPMRNode->psPageList);
	psPMRNode->psPageList = IMG_NULL;
	eError = PMRWritePMPageList(
						/* Target PMR, offset, and length */
						psFreeList->psFreeListPMR,
						uiOffset,
						uiLength,
						/* Referenced PMR, and "page" granularity */
						psPMRNode->psPMR,
						RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT,
						&psPMRNode->psPageList,
						&ui64CheckSum);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"Error (%u) writing FL 0x%08x", eError, (IMG_UINT32)psFreeList->ui32FreelistID));
	}

	/* Zeroing physical pages pointed by the reconstructed freelist */
	if (psDevInfo->ui32DeviceFlags & RGXKM_DEVICE_STATE_ZERO_FREELIST)
	{
		eError = PMRZeroingPMR(psPMRNode->psPMR, RGX_BIF_PM_PHYSICAL_PAGE_ALIGNSHIFT);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"_RGXCheckFreeListReconstruction: Failed to zero PMR %p of freelist %p with Error %d",
									psPMRNode->psPMR,
									psFreeList,
									eError));
			PVR_ASSERT(0);
		}
	}

	psFreeList->ui32CurrentFLPages += psPMRNode->ui32NumPages;

	return IMG_TRUE;
}

IMG_VOID RGXProcessRequestFreelistsReconstruction(PVRSRV_RGXDEV_INFO *psDevInfo,
								RGXFWIF_DM eDM,
								IMG_UINT32 ui32FreelistsCount,
								IMG_UINT32 *paui32Freelists)
{
	PVRSRV_ERROR eError;
	DEVMEM_FREELIST_LOOKUP sLookUp;
	IMG_UINT32 ui32Loop, ui32Loop2;
	RGXFWIF_KCCB_CMD s3DCCBCmd;
	IMG_UINT64 ui64CheckSum;
	
	PVR_ASSERT(psDevInfo);

	//PVR_DPF((PVR_DBG_ERROR,"FreeList RECONSTRUCTION: Reconstructing %u freelist(s)", ui32FreelistsCount));
	
	for (ui32Loop = 0; ui32Loop < ui32FreelistsCount; ui32Loop++)
	{
		/* check if there is more than one occurence of FL on the list */	
		for (ui32Loop2 = ui32Loop + 1; ui32Loop2 < ui32FreelistsCount; ui32Loop2++)
		{
			if (paui32Freelists[ui32Loop] == paui32Freelists[ui32Loop2])
			{
				/* There is a duplicate on a list, skip current Freelist */
				break;
			}
		}

		if (ui32Loop2 < ui32FreelistsCount)
		{
			/* There is a duplicate on the list, skip current Freelist */
			continue;
		}

		/* find the freelist with the corresponding ID */
		sLookUp.ui32FreeListID = paui32Freelists[ui32Loop];
		sLookUp.psFreeList = IMG_NULL;
	
		//PVR_DPF((PVR_DBG_ERROR,"FreeList RECONSTRUCTION: Looking for freelist %08X", (IMG_UINT32)sLookUp.ui32FreeListID));
		OSLockAcquire(psDevInfo->hLockFreeList);
		//PVR_DPF((PVR_DBG_ERROR,"FreeList RECONSTRUCTION: Freelist head %08X", (IMG_UINT32)&psDevInfo->sFreeListHead));
		dllist_foreach_node(&psDevInfo->sFreeListHead, _FindFreeList, (IMG_PVOID)&sLookUp);
		OSLockRelease(psDevInfo->hLockFreeList);

		if (sLookUp.psFreeList)
		{
			RGX_FREELIST *psFreeList = sLookUp.psFreeList;

			//PVR_DPF((PVR_DBG_ERROR,"FreeList RECONSTRUCTION: Reconstructing freelist %08X", (IMG_UINT32)psFreeList));
		
			/* Do the FreeList Reconstruction */
				
			psFreeList->ui32CurrentFLPages = 0;

			/* Reconstructing Init FreeList pages */
			dllist_foreach_node(&psFreeList->sMemoryBlockInitHead,
							_RGXCheckFreeListReconstruction, 
							IMG_NULL);

			/* Reconstructing Grow FreeList pages */
			dllist_foreach_node(&psFreeList->sMemoryBlockHead,
							_RGXCheckFreeListReconstruction, 
							IMG_NULL);

			if (psFreeList->bCheckFreelist)
			{
				/* Get Freelist checksum (as the list is fully populated) */
				eError = _FreeListCheckSum(psFreeList,
											&ui64CheckSum);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_ERROR,
							 "RGXProcessRequestFreelistsReconstruction: Failed to get freelist checksum Node %p",
							 psFreeList));
					while(1)
					{
						OSSleepms(1000);
					}
//					PVR_ASSERT(0);
				}

				/* Verify checksum with previous value */
				if (psFreeList->ui64FreelistChecksum != ui64CheckSum)
				{
					PVR_DPF((PVR_DBG_ERROR, "RGXProcessRequestFreelistsReconstruction: Freelist [%p] checksum failed: before reconstruction = 0x%016llx, after reconstruction = 0x%016llx",
											psFreeList,
											psFreeList->ui64FreelistChecksum,
											ui64CheckSum));
					while(1)
					{
						OSSleepms(1000);
					}
					//PVR_ASSERT(0);
				}
			}

			eError = PVRSRV_OK;

			if (eError == PVRSRV_OK)
			{
				/* Freelist reconstruction successful */
				s3DCCBCmd.uCmdData.sFreeListsReconstructionData.aui32FreelistIDs[ui32Loop] = 
													paui32Freelists[ui32Loop];
			}
			else
			{
				/* Freelist reconstruction failed */
				s3DCCBCmd.uCmdData.sFreeListsReconstructionData.aui32FreelistIDs[ui32Loop] = 
													paui32Freelists[ui32Loop] | RGXFWIF_FREELISTS_RECONSTRUCTION_FAILED_FLAG;
				
				PVR_DPF((PVR_DBG_ERROR,"Reconstructing of FreeList %p failed (error %u)",
										psFreeList,
										eError));
			}
		}
		else
		{
			/* Should never happen */
			PVR_DPF((PVR_DBG_ERROR,"FreeList Lookup for FreeList ID 0x%08x failed (Freelist reconstruction)", sLookUp.ui32FreeListID));
			PVR_ASSERT(IMG_FALSE);
		}
	}

	/* send feedback */
	s3DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_FREELISTS_RECONSTRUCTION_UPDATE;
	s3DCCBCmd.uCmdData.sFreeListsReconstructionData.ui32FreelistsCount = ui32FreelistsCount;
	eError = RGXScheduleCommand(psDevInfo,
										eDM,
										&s3DCCBCmd,
										sizeof(s3DCCBCmd),
										IMG_FALSE);

	/* Kernel CCB should never fill up, as the FW is processing them right away  */
	PVR_ASSERT(eError == PVRSRV_OK);
}

/* Create HWRTDataSet */
IMG_EXPORT
PVRSRV_ERROR RGXCreateHWRTData(PVRSRV_DEVICE_NODE	*psDeviceNode,
							   IMG_UINT32			psRenderTarget, /* FIXME this should not be IMG_UINT32 */
							   IMG_DEV_VIRTADDR		psPMMListDevVAddr,
							   IMG_DEV_VIRTADDR		psVFPPageTableAddr,
							   RGX_FREELIST			*apsFreeLists[RGXFW_MAX_FREELISTS],
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


	OSLockAcquire(psDevInfo->hLockFreeList);
	for (ui32Loop = 0; ui32Loop < RGXFW_MAX_FREELISTS; ui32Loop++)
	{
		psTmpCleanup->apsFreeLists[ui32Loop] = apsFreeLists[ui32Loop];
		psTmpCleanup->apsFreeLists[ui32Loop]->ui32RefCount++;
		psHWRTData->apsFreeLists[ui32Loop] = *((PRGXFWIF_FREELIST *)&(psTmpCleanup->apsFreeLists[ui32Loop]->sFreeListFWDevVAddr.ui32Addr)); /* FIXME: Fix pointer type casting */
		/* invalid initial snapshot value, in case first TA using this freelist fails */
		psHWRTData->aui32FreeListHWRSnapshot[ui32Loop] = IMG_UINT32_MAX;
	}
	OSLockRelease(psDevInfo->hLockFreeList);
	
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
	OSLockAcquire(psDevInfo->hLockFreeList);
	for (ui32Loop = 0; ui32Loop < RGXFW_MAX_FREELISTS; ui32Loop++)
	{
		psTmpCleanup->apsFreeLists[ui32Loop]->ui32RefCount--;
	}
	OSLockRelease(psDevInfo->hLockFreeList);
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
	PVRSRV_RGXDEV_INFO *psDevInfo;
	PVRSRV_ERROR eError;
	PRGXFWIF_HWRTDATA psHWRTData;
	IMG_UINT32 ui32Loop;

	PVR_ASSERT(psCleanupData);

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

	psDevInfo = psCleanupData->psDeviceNode->pvDevice;

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

	/* decrease freelist refcount */
	OSLockAcquire(psDevInfo->hLockFreeList);
	for (ui32Loop = 0; ui32Loop < RGXFW_MAX_FREELISTS; ui32Loop++)
	{
		psCleanupData->apsFreeLists[ui32Loop]->ui32RefCount--;
	}
	OSLockRelease(psDevInfo->hLockFreeList);

	OSFreeMem(psCleanupData);

	return PVRSRV_OK;
}

IMG_EXPORT
PVRSRV_ERROR RGXCreateFreeList(PVRSRV_DEVICE_NODE	*psDeviceNode, 
							   IMG_UINT32			ui32MaxFLPages,
							   IMG_UINT32			ui32InitFLPages,
							   IMG_UINT32			ui32GrowFLPages,
							   IMG_BOOL				bCheckFreelist,
							   IMG_DEV_VIRTADDR		sFreeListDevVAddr,
							   PMR					*psFreeListPMR,
							   IMG_DEVMEM_OFFSET_T	uiFreeListPMROffset,
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
		PVR_DPF((PVR_DBG_ERROR,"RGXCreateFreeList: Failed to allocate cleanup sync (0x%x)",
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
	psFreeList->psDevInfo = psDevInfo;
	psFreeList->psFreeListPMR = psFreeListPMR;
	psFreeList->uiFreeListPMROffset = uiFreeListPMROffset;
	psFreeList->psFWFreelistMemDesc = psFWFreelistMemDesc;
	RGXSetFirmwareAddress(&psFreeList->sFreeListFWDevVAddr, psFWFreelistMemDesc, 0, RFW_FWADDR_FLAG_NONE);
	psFreeList->ui32FreelistID = psDevInfo->ui32FreelistCurrID++;
	psFreeList->ui32MaxFLPages = ui32MaxFLPages;
	psFreeList->ui32InitFLPages = ui32InitFLPages;
	psFreeList->ui32GrowFLPages = ui32GrowFLPages;
	psFreeList->ui32CurrentFLPages = 0;
	psFreeList->ui64FreelistChecksum = 0;
	psFreeList->ui32RefCount = 0;
	psFreeList->bCheckFreelist = bCheckFreelist;
	dllist_init(&psFreeList->sMemoryBlockHead);
	dllist_init(&psFreeList->sMemoryBlockInitHead);


	/* Add to list of freelists */
	OSLockAcquire(psDevInfo->hLockFreeList);
	dllist_add_to_tail(&psDevInfo->sFreeListHead, &psFreeList->sNode);
	OSLockRelease(psDevInfo->hLockFreeList);


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
	IMG_UINT64 ui64CheckSum;

	PVR_ASSERT(psFreeList);

	if (psFreeList->ui32RefCount != 0)
	{
		/* Freelist still busy */
		return PVRSRV_ERROR_RETRY;
	}

	/* Freelist is not in use => start firmware cleanup */
	eError = RGXFWRequestFreeListCleanUp(psFreeList->psDevInfo,
										 psFreeList->sFreeListFWDevVAddr,
										 psFreeList->psCleanupSync);
	if(eError != PVRSRV_OK)
	{
		/* Can happen if the firmware took too long to handle the cleanup request,
		 * or if SLC-flushes didn't went through (due to some GPU lockup) */
		return eError;
	}

	if (psFreeList->bCheckFreelist)
	{
		/* Do consistency tests (as the list is fully populated) */
		eError = _FreeListCheckSum(psFreeList,
									&ui64CheckSum);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,
					 "RGXDestroyFreeList: Failed to get freelist checksum Node %p",
					 psFreeList));
			while(1)
			{
				OSSleepms(1000);
			}
//				PVR_ASSERT(0);
		}

		if (psFreeList->ui64FreelistChecksum != ui64CheckSum)
		{
			PVR_DPF((PVR_DBG_ERROR,
					 "RGXDestroyFreeList: Checksum mismatch [%p]! stored 0x%016llx, verified 0x%016llx %p",
					 psFreeList,
					 psFreeList->ui64FreelistChecksum,
					 ui64CheckSum,
					 psFreeList));
			while(1)
			{
				OSSleepms(1000);
			}
//			PVR_ASSERT(0);
		}
	}

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
		RGXShrinkFreeList(&psFreeList->sMemoryBlockHead, psFreeList);
		PVR_ASSERT(eError == PVRSRV_OK);
	}

	/* Remove initial PB block */
	eError = RGXShrinkFreeList(&psFreeList->sMemoryBlockInitHead, psFreeList);
	PVR_ASSERT(eError == PVRSRV_OK);

	/* consistency checks */
	PVR_ASSERT(dllist_is_empty(&psFreeList->sMemoryBlockInitHead));
	PVR_ASSERT(psFreeList->ui32CurrentFLPages == 0);

	/* Remove FreeList from list */
	OSLockAcquire(psFreeList->psDevInfo->hLockFreeList);
	dllist_remove_node(&psFreeList->sNode);
	OSLockRelease(psFreeList->psDevInfo->hLockFreeList);

	SyncPrimFree(psFreeList->psCleanupSync);

	/* free Freelist */
	OSFreeMem(psFreeList);

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
		_UpdateFwFreelistSize(psFreeList, IMG_TRUE, ui32NumPages);

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

	/* TODO:
	 * Make sure the pages part of the memory block are not in use anymore.
	 * Instruct the firmware to update the freelist pointers accordingly.
	 */

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
	psRenderTarget->bZeroTACaches = IMG_FALSE;
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
	psZSBuffer->psDevInfo = psDevInfo;
	psZSBuffer->psReservation = psReservation;
	psZSBuffer->psPMR = psPMR;
	psZSBuffer->uiMapFlags = uiMapFlags;
	psZSBuffer->ui32RefCount = 0;
	psZSBuffer->bOnDemand = bOnDemand;
    if (bOnDemand)
    {
    	psZSBuffer->ui32ZSBufferID = psDevInfo->ui32ZSBufferCurrID++;
    	psZSBuffer->psMapping = IMG_NULL;

		OSLockAcquire(psDevInfo->hLockZSBuffer);
    	dllist_add_to_tail(&psDevInfo->sZSBufferHead, &psZSBuffer->sNode);
		OSLockRelease(psDevInfo->hLockZSBuffer);
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
	psFWZSBuffer->eState = (bOnDemand) ? RGXFWIF_ZSBUFFER_UNBACKED : RGXFWIF_ZSBUFFER_BACKED;
	psFWZSBuffer->ui32ZSBufferID = psZSBuffer->ui32ZSBufferID;

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
	hLockZSBuffer = psZSBuffer->psDevInfo->hLockZSBuffer;

	/* Request ZS Buffer cleanup */
	eError = RGXFWRequestZSBufferCleanUp(psZSBuffer->psDevInfo,
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
								psZSBuffer->ui32ZSBufferID));
	hLockZSBuffer = psZSBuffer->psDevInfo->hLockZSBuffer;

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
										psZSBuffer->ui32ZSBufferID,
										eError));
				OSLockRelease(hLockZSBuffer);
				return eError;

			}
			PVR_DPF((PVR_DBG_MESSAGE, "ZS Buffer [%p, ID=0x%08x]: Physical backing acquired",
										psZSBuffer,
										psZSBuffer->ui32ZSBufferID));
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
								psZSBuffer->ui32ZSBufferID));

	hLockZSBuffer = psZSBuffer->psDevInfo->hLockZSBuffer;

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
										psZSBuffer->ui32ZSBufferID,
										eError));
				OSLockRelease(hLockZSBuffer);
				return eError;
			}

			PVR_DPF((PVR_DBG_MESSAGE, "ZS Buffer [%p, ID=0x%08x]: Physical backing removed",
										psZSBuffer,
										psZSBuffer->ui32ZSBufferID));
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

static IMG_BOOL _FindZSBuffer(PDLLIST_NODE psNode, IMG_PVOID pvCallbackData)
{
	DEVMEM_REF_LOOKUP *psRefLookUp = (DEVMEM_REF_LOOKUP *)pvCallbackData;
	RGX_ZSBUFFER_DATA *psZSBuffer;

	psZSBuffer = IMG_CONTAINER_OF(psNode, RGX_ZSBUFFER_DATA, sNode);

	if (psZSBuffer->ui32ZSBufferID == psRefLookUp->ui32ZSBufferID)
	{
		psRefLookUp->psZSBuffer = psZSBuffer;
		return IMG_FALSE;
	}
	else
	{
		return IMG_TRUE;
	}
}

IMG_VOID RGXProcessRequestZSBufferBacking(PVRSRV_RGXDEV_INFO *psDevInfo,
											IMG_UINT32 ui32ZSBufferID)
{
	DEVMEM_REF_LOOKUP sLookUp;
	RGXFWIF_KCCB_CMD sTACCBCmd;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psDevInfo);

	/* scan all deferred allocations */
	sLookUp.ui32ZSBufferID = ui32ZSBufferID;
	sLookUp.psZSBuffer = IMG_NULL;

	OSLockAcquire(psDevInfo->hLockZSBuffer);
	dllist_foreach_node(&psDevInfo->sZSBufferHead, _FindZSBuffer, (IMG_PVOID)&sLookUp);
	OSLockRelease(psDevInfo->hLockZSBuffer);

	if (sLookUp.psZSBuffer)
	{
		IMG_BOOL bBackingDone = IMG_TRUE;

		/* Populate ZLS */
		eError = RGXBackingZSBuffer(sLookUp.psZSBuffer);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"Populating ZS-Buffer failed failed with error %u (ID = 0x%08x)", eError, ui32ZSBufferID));
			bBackingDone = IMG_FALSE;
		}

		/* send confirmation */
		sTACCBCmd.eCmdType = RGXFWIF_KCCB_CMD_ZSBUFFER_BACKING_UPDATE;
		sTACCBCmd.uCmdData.sZSBufferBackingData.psZSBufferFWDevVAddr = sLookUp.psZSBuffer->sZSBufferFWDevVAddr.ui32Addr;
		sTACCBCmd.uCmdData.sZSBufferBackingData.bDone = bBackingDone;
		eError = RGXScheduleCommand(psDevInfo,
											RGXFWIF_DM_TA,
											&sTACCBCmd,
											sizeof(sTACCBCmd),
											IMG_FALSE);

		/* Kernel CCB should never fill up, as the FW is processing them right away  */
		PVR_ASSERT(eError == PVRSRV_OK);

		sLookUp.psZSBuffer->ui32NumReqByFW++;
	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR,"ZS Buffer Lookup for ZS Buffer ID 0x%08x failed (Populate)", sLookUp.ui32ZSBufferID));
	}
}

IMG_VOID RGXProcessRequestZSBufferUnbacking(PVRSRV_RGXDEV_INFO *psDevInfo,
											IMG_UINT32 ui32ZSBufferID)
{
	DEVMEM_REF_LOOKUP sLookUp;
	RGXFWIF_KCCB_CMD sTACCBCmd;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psDevInfo);

	/* scan all deferred allocations */
	sLookUp.ui32ZSBufferID = ui32ZSBufferID;
	sLookUp.psZSBuffer = IMG_NULL;

	OSLockAcquire(psDevInfo->hLockZSBuffer);
	dllist_foreach_node(&psDevInfo->sZSBufferHead, _FindZSBuffer, (IMG_PVOID)&sLookUp);
	OSLockRelease(psDevInfo->hLockZSBuffer);

	if (sLookUp.psZSBuffer)
	{
		/* Unpopulate ZLS */
		eError = RGXUnbackingZSBuffer(sLookUp.psZSBuffer);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"UnPopulating ZS-Buffer failed failed with error %u (ID = 0x%08x)", eError, ui32ZSBufferID));
			PVR_ASSERT(IMG_FALSE);
		}

		/* send confirmation */
		sTACCBCmd.eCmdType = RGXFWIF_KCCB_CMD_ZSBUFFER_UNBACKING_UPDATE;
		sTACCBCmd.uCmdData.sZSBufferBackingData.psZSBufferFWDevVAddr = sLookUp.psZSBuffer->sZSBufferFWDevVAddr.ui32Addr;
		sTACCBCmd.uCmdData.sZSBufferBackingData.bDone = IMG_TRUE;
		eError = RGXScheduleCommand(psDevInfo,
											RGXFWIF_DM_TA,
											&sTACCBCmd,
											sizeof(sTACCBCmd),
											IMG_FALSE);

		/* Kernel CCB should never fill up, as the FW is processing them right away  */
		PVR_ASSERT(eError == PVRSRV_OK);

	}
	else
	{
		PVR_DPF((PVR_DBG_ERROR,"ZS Buffer Lookup for ZS Buffer ID 0x%08x failed (UnPopulate)", sLookUp.ui32ZSBufferID));
	}
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
	psTmpCleanup->bDumpedTACCBCtlAlready = IMG_FALSE;
	psTmpCleanup->bDumped3DCCBCtlAlready = IMG_FALSE;

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
	psContextState->uTAReg_VDM_CALL_STACK_POINTER_Init = sVDMCallStackAddr.uiAddr;

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
PVRSRV_ERROR PVRSRVRGXKickTA3DKM(CONNECTION_DATA	*psConnection,
								 PVRSRV_DEVICE_NODE	*psDeviceNode,
								 DEVMEM_MEMDESC 	*psFWRenderContextMemDesc,
								 IMG_BOOL			bLastTAInScene,
								 IMG_BOOL			bKickTA,
								 IMG_BOOL			bKick3D,
								 IMG_UINT32			*pui32TAcCCBWoffUpdate,
								 IMG_UINT32			*pui323DcCCBWoffUpdate,
								 DEVMEM_MEMDESC 	*psTAcCCBMemDesc,
								 DEVMEM_MEMDESC 	*psTACCBCtlMemDesc,
								 DEVMEM_MEMDESC 	*ps3DcCCBMemDesc,
								 DEVMEM_MEMDESC 	*ps3DCCBCtlMemDesc,
								 IMG_UINT32			ui32TAServerSyncPrims,
								 PVRSRV_CLIENT_SYNC_PRIM_OP**	pasTASyncOp,
								 SERVER_SYNC_PRIMITIVE **pasTAServerSyncs,
								 IMG_UINT32			ui323DServerSyncPrims,
								 PVRSRV_CLIENT_SYNC_PRIM_OP**	pas3DSyncOp,
								 SERVER_SYNC_PRIMITIVE **pas3DServerSyncs,
								 IMG_UINT32			ui32TACmdSize,
								 IMG_PBYTE			pui8TACmd,
								 IMG_UINT32			ui32TAFenceEnd,
								 IMG_UINT32			ui32TAUpdateEnd,
								 IMG_UINT32			ui323DCmdSize,
								 IMG_PBYTE			pui83DCmd,
								 IMG_UINT32			ui323DFenceEnd,
								 IMG_UINT32			ui323DUpdateEnd,
								 IMG_UINT32         ui32NumFenceFds,
								 IMG_INT32          *ai32FenceFds,
								 IMG_BOOL			bPDumpContinuous,
								 RGX_RC_CLEANUP_DATA *psCleanupData)
{
	PVRSRV_ERROR			eError = 0;
	RGXFWIF_KCCB_CMD		sTACCBCmd;
	RGXFWIF_KCCB_CMD		s3DCCBCmd;
	volatile RGXFWIF_CCCB_CTL	*psTACCBCtl;
	volatile RGXFWIF_CCCB_CTL	*ps3DCCBCtl;
	IMG_UINT8					*pui8TACmdPtr, *pui83DCmdPtr;
	IMG_UINT32 i = 0;
	RGXFWIF_UFO					*psUFOPtr;
	IMG_UINT8                   *pui8FencePtr; 
	IMG_UINT8                   *pui8UpdatePtr;
	IMG_UINT32                  ui32FDFenceCmdSize = 0;
	IMG_UINT32                  ui32FDUpdateCmdSize = 0;
#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
	IMG_UINT32 ui32NumFenceSyncs = 0;
	IMG_UINT32 *pui32FenceFWAddrs;
	IMG_UINT32 *pui32FenceValues;
	IMG_UINT32 ui32NumUpdateSyncs = 0;
	IMG_UINT32 *pui32UpdateFWAddrs;
	IMG_UINT32 *pui32UpdateValues;
	RGXFWIF_CCB_CMD_HEADER *psFDFenceHdr  = IMG_NULL;
	RGXFWIF_CCB_CMD_HEADER *psFDUpdateHdr = IMG_NULL;
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

	if(bKickTA)
	{
		pui8FencePtr= pui8TACmd + ui32TAFenceEnd;
		pui8UpdatePtr= pui8TACmd + ui32TAUpdateEnd;
	
		for (i = 0; i < ui32TAServerSyncPrims; i++)
		{
			IMG_BOOL bUpdate;
			PVRSRV_CLIENT_SYNC_PRIM_OP *psSyncOp = pasTASyncOp[i];
						
			PVR_ASSERT(psSyncOp->ui32Flags != 0);
			if (psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE)
			{
				bUpdate = IMG_TRUE;
			}
			else
			{
				bUpdate = IMG_FALSE;
			}

			eError = PVRSRVServerSyncQueueHWOpKM(pasTAServerSyncs[i],
												  bUpdate,
												  &psSyncOp->ui32FenceValue,
												  &psSyncOp->ui32UpdateValue);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVServerSyncQueueHWOpKM: Failed (0x%x)", eError));
				goto PVRSRVRGXKickTA3DKM_Exit;
			}
			
			if(psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8FencePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32FenceValue;
				PDUMPCOMMENT("TA client server fence - 0x%x <- 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8FencePtr += sizeof(*psUFOPtr);
			}
			if(psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8UpdatePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32UpdateValue;
				PDUMPCOMMENT("TA client server update - 0x%x -> 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8UpdatePtr += sizeof(*psUFOPtr);
			}
		}
		
		eError = DevmemAcquireCpuVirtAddr(psTACCBCtlMemDesc,
										  (IMG_VOID **)&psTACCBCtl);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"CreateCCB: Failed to map client CCB control (0x%x)", eError));
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
		/*
		 * Acquire space in the TA CCB for the command.
		 */
		eError = RGXAcquireCCB(psTACCBCtl,
							   pui32TAcCCBWoffUpdate,
							   psTAcCCBMemDesc,
							   ui32TACmdSize,
							   (IMG_PVOID *)&pui8TACmdPtr,
							   psCleanupData->bDumpedTACCBCtlAlready,
							   bPDumpContinuous);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXKickTA: Failed to acquire %d bytes in client CCB", ui32TACmdSize));
			DevmemReleaseCpuVirtAddr(psTACCBCtlMemDesc);
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
		
		OSMemCopy(pui8TACmdPtr, pui8TACmd, ui32TACmdSize);
		
		/*
		 * Release the TA CCB for the command.
		 */

		eError = RGXReleaseCCB(psDeviceNode, 
							   psTACCBCtl,
							   psTAcCCBMemDesc, 
							   psTACCBCtlMemDesc,
							   &psCleanupData->bDumpedTACCBCtlAlready,
							   pui32TAcCCBWoffUpdate,
							   ui32TACmdSize, 
							   bPDumpContinuous,
							   psConnection->psSyncConnectionData);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXKickTA: Failed to release space in TA CCB"));
			DevmemReleaseCpuVirtAddr(psTACCBCtlMemDesc);
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
		
		/*
		 * Construct the kernel TA CCB command.
		 * (Safe to release reference to render context virtual address because
		 * render context destruction must flush the firmware).
		 */
		sTACCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
		RGXSetFirmwareAddress(&sTACCBCmd.uCmdData.sCmdKickData.psContext, psFWRenderContextMemDesc,
						  offsetof(RGXFWIF_FWRENDERCONTEXT, sTAContext), RFW_FWADDR_NOREF_FLAG);
		sTACCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = *pui32TAcCCBWoffUpdate;

		/*
		 * Submit the TA command to the firmware.
		 */
		eError = RGXScheduleCommand(psDeviceNode->pvDevice,
									RGXFWIF_DM_TA,
									&sTACCBCmd,
									sizeof(sTACCBCmd),
									bPDumpContinuous);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXKickTA3DKM failed to schedule kernel TA command. Error:%u", eError));
			DevmemReleaseCpuVirtAddr(psTACCBCtlMemDesc);
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
		
		DevmemReleaseCpuVirtAddr(psTACCBCtlMemDesc);
	}

	if (bKick3D)
	{
		pui8FencePtr= pui83DCmd + ui323DFenceEnd;
		pui8UpdatePtr= pui83DCmd + ui323DUpdateEnd;
		for (i = 0; i < ui323DServerSyncPrims; i++)
		{
			IMG_BOOL bUpdate;
			PVRSRV_CLIENT_SYNC_PRIM_OP *psSyncOp = pas3DSyncOp[i];
								
			PVR_ASSERT(psSyncOp->ui32Flags != 0);
			if ((psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE) && ui323DUpdateEnd)
			{
				bUpdate = IMG_TRUE;
			}
			else
			{
				bUpdate = IMG_FALSE;
			}
			
			eError = PVRSRVServerSyncQueueHWOpKM(pas3DServerSyncs[i],
												  bUpdate,
												  &psSyncOp->ui32FenceValue,
												  &psSyncOp->ui32UpdateValue);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVServerSyncQueueHWOpKM: Failed (0x%x)", eError));
				goto PVRSRVRGXKickTA3DKM_Exit;
			}
						
			if((psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK) && ui323DFenceEnd)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8FencePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32FenceValue;
				PDUMPCOMMENT("3D client server fence - 0x%x <- 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8FencePtr += sizeof(*psUFOPtr);
			}
			if((psSyncOp->ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE) && ui323DUpdateEnd)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8UpdatePtr;
				psUFOPtr->puiAddrUFO.ui32Addr =  SyncPrimGetFirmwareAddr(psSyncOp->psSync);
				psUFOPtr->ui32Value = psSyncOp->ui32UpdateValue;
				PDUMPCOMMENT("3D client server update - 0x%x -> 0x%x",
								   psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8UpdatePtr += sizeof(*psUFOPtr);
			}
		}
		
		eError = DevmemAcquireCpuVirtAddr(ps3DCCBCtlMemDesc,
										  (IMG_VOID **)&ps3DCCBCtl);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"CreateCCB: Failed to map client CCB control (0x%x)", eError));
			goto PVRSRVRGXKickTA3DKM_Exit;
		}

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
		eError = PVRFDSyncQueryFencesKM(ui32NumFenceFds,
										ai32FenceFds,
										IMG_FALSE,
										&ui32NumFenceSyncs,
										&pui32FenceFWAddrs,
										&pui32FenceValues,
										&ui32NumUpdateSyncs,
										&pui32UpdateFWAddrs,
										&pui32UpdateValues);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR,"PVRFDSyncQueryFencesKM: Failed (0x%x)", eError));
			DevmemReleaseCpuVirtAddr(ps3DCCBCtlMemDesc);
			goto PVRSRVRGXKickTA3DKM_Exit;
		}			

		if (ui32NumFenceSyncs)
		{
			ui32FDFenceCmdSize = RGX_CCB_FWALLOC_ALIGN(ui32NumFenceSyncs * sizeof(RGXFWIF_UFO) + sizeof(RGXFWIF_CCB_CMD_HEADER));
		}
		if (ui32NumUpdateSyncs)
		{
			ui32FDUpdateCmdSize = RGX_CCB_FWALLOC_ALIGN(ui32NumUpdateSyncs * sizeof(RGXFWIF_UFO) + sizeof(RGXFWIF_CCB_CMD_HEADER));
		}

		ui323DCmdSize += ui32FDFenceCmdSize + ui32FDUpdateCmdSize;
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

		/*
		 * Acquire space in the 3D CCB for the command.
		 */
		eError = RGXAcquireCCB(ps3DCCBCtl,
							   pui323DcCCBWoffUpdate,
							   ps3DcCCBMemDesc,
							   ui323DCmdSize,
							   (IMG_PVOID *)&pui83DCmdPtr,
							   psCleanupData->bDumped3DCCBCtlAlready,
							   bPDumpContinuous);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXKickTA: Failed to acquire %d bytes in client CCB", ui32TACmdSize));
			DevmemReleaseCpuVirtAddr(ps3DCCBCtlMemDesc);
			PVR_ASSERT(0);
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
		
		OSMemCopy(&pui83DCmdPtr[ui32FDFenceCmdSize], pui83DCmd, ui323DCmdSize - ui32FDFenceCmdSize - ui32FDUpdateCmdSize);

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC)
		if (ui32FDFenceCmdSize)
		{
			/* Fill the fence header */
			psFDFenceHdr = (RGXFWIF_CCB_CMD_HEADER *) pui83DCmdPtr;
			psFDFenceHdr->eCmdType = RGXFWIF_CCB_CMD_TYPE_FENCE;
			psFDFenceHdr->ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_UFO) * ui32NumFenceSyncs);;
			/* Fill in the actual fence commands */
			pui8FencePtr = (IMG_UINT8 *) &pui83DCmdPtr[sizeof(RGXFWIF_CCB_CMD_HEADER)];
			for (i = 0; i < ui32NumFenceSyncs; i++)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8FencePtr;
				psUFOPtr->puiAddrUFO.ui32Addr = pui32FenceFWAddrs[i];
				psUFOPtr->ui32Value = pui32FenceValues[i];
				PDUMPCOMMENT("3D client server fence - 0x%x <- 0x%x (Android native fence)",
							 psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8FencePtr += sizeof(*psUFOPtr);
			}
			OSFreeMem(pui32FenceFWAddrs);
			OSFreeMem(pui32FenceValues);
		}
		if (ui32FDUpdateCmdSize)
		{
			/* Fill the update header */
			psFDUpdateHdr = (RGXFWIF_CCB_CMD_HEADER *) &pui83DCmdPtr[ui323DCmdSize - ui32FDUpdateCmdSize];
			psFDUpdateHdr->eCmdType = RGXFWIF_CCB_CMD_TYPE_UPDATE;
			psFDUpdateHdr->ui32CmdSize = RGX_CCB_FWALLOC_ALIGN(sizeof(RGXFWIF_UFO) * ui32NumUpdateSyncs);
			/* Fill in the actual update commands */
			pui8UpdatePtr = (IMG_UINT8 *) &pui83DCmdPtr[ui323DCmdSize - ui32FDUpdateCmdSize + sizeof(RGXFWIF_CCB_CMD_HEADER)];
			for (i = 0; i < ui32NumUpdateSyncs; i++)
			{
				psUFOPtr = (RGXFWIF_UFO *) pui8UpdatePtr;
				psUFOPtr->puiAddrUFO.ui32Addr = pui32UpdateFWAddrs[i];
				psUFOPtr->ui32Value = pui32UpdateValues[i];
				PDUMPCOMMENT("3D client server update - 0x%x -> 0x%x (Android native fence)",
							 psUFOPtr->puiAddrUFO.ui32Addr, psUFOPtr->ui32Value);
				pui8UpdatePtr += sizeof(*psUFOPtr);
			}
			OSFreeMem(pui32UpdateFWAddrs);
			OSFreeMem(pui32UpdateValues);
		}
#endif /* defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) */

		/*
		 * Release the 3D CCB for the command.
		 */

		eError = RGXReleaseCCB(psDeviceNode, 
							   ps3DCCBCtl,
							   ps3DcCCBMemDesc, 
							   ps3DCCBCtlMemDesc,
							   &psCleanupData->bDumped3DCCBCtlAlready,
							   pui323DcCCBWoffUpdate,
							   ui323DCmdSize,
							   bPDumpContinuous,
							   psConnection->psSyncConnectionData);

		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "RGXKickTA: Failed to release space in 3D CCB"));
			DevmemReleaseCpuVirtAddr(ps3DCCBCtlMemDesc);
			PVR_ASSERT(0);
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
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
		s3DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = *pui323DcCCBWoffUpdate;

		/*
		 * Submit the 3D command to the firmware.
		 */
		eError = RGXScheduleCommand(psDeviceNode->pvDevice,
									RGXFWIF_DM_3D,
									&s3DCCBCmd,
									sizeof(s3DCCBCmd),
									bPDumpContinuous);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "PVRSRVRGXKickTA3DKM failed to schedule kernel 3D command. Error:%u", eError));
			DevmemReleaseCpuVirtAddr(ps3DCCBCtlMemDesc);
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
		
		DevmemReleaseCpuVirtAddr(ps3DCCBCtlMemDesc);
	}

#if defined(PVR_ANDROID_NATIVE_WINDOW_HAS_SYNC) && defined(NO_HARDWARE)
	for (i = 0; i < ui32NumFenceFds; i++)
	{
		if (PVRFDSyncNoHwUpdateFenceKM(ai32FenceFds[i]) != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed nohw update on fence fd=%d",
					 __func__, ai32FenceFds[i]));
		}
	}
#endif

PVRSRVRGXKickTA3DKM_Exit:
	return eError;
}


/******************************************************************************
 End of file (rgxta3d.c)
******************************************************************************/
