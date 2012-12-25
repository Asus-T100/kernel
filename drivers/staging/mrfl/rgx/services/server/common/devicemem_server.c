/*************************************************************************/ /*!
@File
@Title          Device Memory Management
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Server-side component of the Device Memory Management.
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
/* our exported API */
#include "devicemem_server.h"
#include "devicemem_utils.h"

#include "device.h" /* For device node */
#include "img_types.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"

#include "mmu_common.h"
#include "pdump_km.h" /* FIXME: check hierarchy... */
#include "pmr.h"

#include "allocmem.h"

struct _DEVMEMINT_CTX_
{
    PVRSRV_DEVICE_NODE *psDevNode;

    /* MMU common code needs to have a context.  There's a one-to-one
       correspondence between device memory context and MMU context,
       but we have the abstraction here so that we don't need to care
       what the MMU does with its context, and the MMU code need not
       know about us at all. */
    MMU_CONTEXT *psMMUContext;

    IMG_UINT32 ui32RefCount;

    /* This handle is for devices that require notification when a new
       memory context is created and they need to store private data that
       is associated with the context. */
    IMG_HANDLE hPrivData;
};

struct _DEVMEMINT_HEAP_
{
    struct _DEVMEMINT_CTX_ *psDevmemCtx;
    IMG_UINT32 ui32RefCount;
};

struct _DEVMEMINT_RESERVATION_
{
    struct _DEVMEMINT_HEAP_ *psDevmemHeap;
    IMG_DEV_VIRTADDR sBase;
    IMG_DEVMEM_SIZE_T uiLength;
};

struct _DEVMEMINT_MAPPING_
{
    struct _DEVMEMINT_RESERVATION_ *psReservation;
    PMR *psPMR;
    IMG_UINT32 uiNumPages;
    IMG_UINT32 uiLog2PageSize;
};

/*************************************************************************/ /*!
@Function       DevmemServerGetImportHandle
@Description    For given exportable memory descriptor returns PMR handle.
@Return         Memory is exportable - Success
                PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemServerGetImportHandle(DEVMEM_MEMDESC *psMemDesc,
						   IMG_HANDLE *phImport)
{
	PVRSRV_ERROR eError;

	if (psMemDesc->psImport->bExportable == IMG_FALSE)
	{
        eError = PVRSRV_ERROR_DEVICEMEM_CANT_EXPORT_SUBALLOCATION;
        goto e0;
	}

	*phImport = psMemDesc->psImport->hPMR;
	return PVRSRV_OK;

e0:
	return eError;
}

/*************************************************************************/ /*!
@Function       DevmemServerGetHeapHandle
@Description    For given reservation returns the Heap handle.
@Return         PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemServerGetHeapHandle(DEVMEMINT_RESERVATION *psReservation,
						   IMG_HANDLE *phHeap)
{
	*phHeap = psReservation->psDevmemHeap;
	return PVRSRV_OK;
}



/*************************************************************************/ /*!
@Function       DevmemIntCtxCreate
@Description    Creates and initialises a device memory context.
@Return         valid Device Memory context handle - Success
                PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemIntCtxCreate(
                   PVRSRV_DEVICE_NODE *psDeviceNode,
                   DEVMEMINT_CTX **ppsDevmemCtxPtr,
                   IMG_HANDLE *hPrivData
                   )
{
    PVRSRV_ERROR eError;
    DEVMEMINT_CTX *psDevmemCtx;
    IMG_HANDLE hPrivDataInt = IMG_NULL;

	PVR_DPF((PVR_DBG_MESSAGE, "DevmemIntCtx_Create"));

	/* allocate a Devmem context */
    psDevmemCtx = OSAllocMem(sizeof *psDevmemCtx);
    if (psDevmemCtx == IMG_NULL)
	{
        eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		PVR_DPF ((PVR_DBG_ERROR, "DevmemIntCtx_Create: Alloc failed"));
        goto e0;
	}

    psDevmemCtx->psDevNode = psDeviceNode;

    /* FIXME:  take ref-count on devnode? how! */

    /* Call down to MMU context creation */

    eError = MMU_ContextCreate(psDeviceNode,
                               &psDevmemCtx->psMMUContext);
	if (eError != PVRSRV_OK)
	{
		goto e1;
	}


	if (psDeviceNode->pfnRegisterMemoryContext)
	{
		eError = psDeviceNode->pfnRegisterMemoryContext(psDeviceNode, psDevmemCtx->psMMUContext, &hPrivDataInt);
		if (eError != PVRSRV_OK)
		{
			goto e2;
		}
	}

	/* Store the private data as it is required to unregister the memory context */
	psDevmemCtx->hPrivData = hPrivDataInt;
	*hPrivData = hPrivDataInt;

    if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "DevmemIntCtx_Create: MMU_ContextCreate failed"));
		goto e1;
	}

	psDevmemCtx->ui32RefCount++;

    *ppsDevmemCtxPtr = psDevmemCtx;

	return PVRSRV_OK;

 e2:
    MMU_ContextDestroy(psDevmemCtx->psMMUContext);

 e1:
    OSFreeMem(psDevmemCtx);

 e0:
    PVR_ASSERT(eError != PVRSRV_OK);
    return eError;
}

/*************************************************************************/ /*!
@Function       DevmemIntCtxCreate
@Description    Creates and initialises a device memory context.
@Return         valid Device Memory context handle - Success
                PVRSRV_ERROR failure code
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemIntHeapCreate(
                    DEVMEMINT_CTX *psDevmemCtx,
                    IMG_DEV_VIRTADDR sHeapBaseAddr,
                    IMG_DEVMEM_SIZE_T uiHeapLength,
                    IMG_UINT32 uiLog2DataPageSize,
                    DEVMEMINT_HEAP **ppsDevmemHeapPtr
                    )
{
    PVRSRV_ERROR eError;
    DEVMEMINT_HEAP *psDevmemHeap;

	PVR_DPF((PVR_DBG_MESSAGE, "DevmemIntHeap_Create"));

	/* allocate a Devmem context */
	psDevmemHeap = OSAllocMem(sizeof *psDevmemHeap);
    if (psDevmemHeap == IMG_NULL)
	{
        eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		PVR_DPF ((PVR_DBG_ERROR, "DevmemIntHeap_Create: Alloc failed"));
        goto e0;
	}

    psDevmemHeap->psDevmemCtx = psDevmemCtx;

    psDevmemHeap->psDevmemCtx->ui32RefCount ++;

	psDevmemHeap->ui32RefCount = 1;

    *ppsDevmemHeapPtr = psDevmemHeap;

	return PVRSRV_OK;


 e0:
    return eError;
}

PVRSRV_ERROR
DevmemIntMapPMR(DEVMEMINT_HEAP *psDevmemHeap,
                DEVMEMINT_RESERVATION *psReservation,
                PMR *psPMR,
                PVRSRV_MEMALLOCFLAGS_T uiMapFlags,
                DEVMEMINT_MAPPING **ppsMappingPtr)
{
    PVRSRV_ERROR eError;
    DEVMEMINT_MAPPING *psMapping;
    /* page-size in device MMU */
    IMG_UINT32 uiLog2DevPageSize;
    /* number of pages (device pages) that allocation spans */
    IMG_UINT32 ui32NumDevPages;
    /* device virtual address of start of allocation */
    IMG_DEV_VIRTADDR sAllocationDevVAddr;
    /* and its length */
    IMG_DEVMEM_SIZE_T uiAllocationSize;

	/* allocate memory to record the mapping info */
	psMapping = OSAllocMem(sizeof *psMapping);
    if (psMapping == IMG_NULL)
	{
        eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		PVR_DPF ((PVR_DBG_ERROR, "DevmemIntMapPMR: Alloc failed"));
        goto e0;
	}

    uiAllocationSize = psReservation->uiLength;

    /* FIXME: Get page size (which? host, device, or lowest common denominator) */
    uiLog2DevPageSize = 12;
    ui32NumDevPages = 0xffffffffU & (((uiAllocationSize - 1)
                                      >> uiLog2DevPageSize) + 1);
    PVR_ASSERT(ui32NumDevPages << uiLog2DevPageSize == uiAllocationSize);

    /* FIXME: perhaps MapPMR ought to be the one calling LockSysPhysAddresses()? */

    eError = PMRLockSysPhysAddresses(psPMR,
                                     uiLog2DevPageSize);
    if (eError != PVRSRV_OK)
	{
        goto e2;
	}

    sAllocationDevVAddr = psReservation->sBase;

    /*  N.B.  We pass mapping permission flags to MMU_MapPMR and let
       it reject the mapping if the permissions on the PMR are not compatible.
       FIXME: we ought to validate the permissions here and just have the
       MMU_MapPMR be a dumb function that does as it's told. */

    eError = MMU_MapPMR (psDevmemHeap->psDevmemCtx->psMMUContext,
                         sAllocationDevVAddr,
                         psPMR,
                         ui32NumDevPages << uiLog2DevPageSize,
                         /* FIXME: where page size? */
                         uiMapFlags);
    PVR_ASSERT(eError == PVRSRV_OK);

    psMapping->psReservation = psReservation;
    psMapping->uiNumPages = ui32NumDevPages;
    psMapping->uiLog2PageSize = uiLog2DevPageSize;
    psMapping->psPMR = psPMR;
    /* Don't bother with refcount on reservation, as a reservation
       only ever holds one mapping, so we directly increment the
       refcount on the heap instead */
    psMapping->psReservation->psDevmemHeap->ui32RefCount ++;

    *ppsMappingPtr = psMapping;

    return PVRSRV_OK;

 e2:
	OSFreeMem(psMapping);

 e0:
    PVR_ASSERT (eError != PVRSRV_OK);
    return eError;
}


PVRSRV_ERROR
DevmemIntUnmapPMR(DEVMEMINT_MAPPING *psMapping)
{
    PVRSRV_ERROR eError;
    DEVMEMINT_HEAP *psDevmemHeap;
    /* device virtual address of start of allocation */
    IMG_DEV_VIRTADDR sAllocationDevVAddr;
    /* number of pages (device pages) that allocation spans */
    IMG_UINT32 ui32NumDevPages;

    psDevmemHeap = psMapping->psReservation->psDevmemHeap;

    ui32NumDevPages = psMapping->uiNumPages;
    sAllocationDevVAddr = psMapping->psReservation->sBase;

    /* FIXME: Do we want an error code */
    MMU_UnmapPages (psDevmemHeap->psDevmemCtx->psMMUContext,
                    sAllocationDevVAddr,
                    ui32NumDevPages);

    /* FIXME: shouldn't UnmapPMR call this? */
    eError = PMRUnlockSysPhysAddresses(psMapping->psPMR);
    PVR_ASSERT(eError == PVRSRV_OK);

    /* Don't bother with refcount on reservation, as a reservation
       only ever holds one mapping, so we directly decrement the
       refcount on the heap instead */
    psDevmemHeap->ui32RefCount --;

	OSFreeMem(psMapping);

    return PVRSRV_OK;
}


PVRSRV_ERROR
DevmemIntReserveRange(DEVMEMINT_HEAP *psDevmemHeap,
                      IMG_DEV_VIRTADDR sAllocationDevVAddr,
                      IMG_DEVMEM_SIZE_T uiAllocationSize,
                      DEVMEMINT_RESERVATION **ppsReservationPtr)
{
    PVRSRV_ERROR eError;
    DEVMEMINT_RESERVATION *psReservation;

	/* allocate memory to record the reservation info */
	psReservation = OSAllocMem(sizeof *psReservation);
    if (psReservation == IMG_NULL)
	{
        eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		PVR_DPF ((PVR_DBG_ERROR, "DevmemIntReserveRange: Alloc failed"));
        goto e0;
	}

    psReservation->sBase = sAllocationDevVAddr;
    psReservation->uiLength = uiAllocationSize;

    /* FIXME: what about protection flags on the range reservation?
       These could affect Catalogue and Directory entries... */
    eError = MMU_Alloc (psDevmemHeap->psDevmemCtx->psMMUContext,
                        uiAllocationSize,
                        &uiAllocationSize,
                        0, /* IMG_UINT32 uiProtFlags */
                        0, /* alignment is n/a since we supply devvaddr */
                        &sAllocationDevVAddr);
    if (eError != PVRSRV_OK)
    {
        goto e1;
    }

    /* since we supplied the virt addr, MMU_Alloc shouldn't have
       chosen a new one for us */
    PVR_ASSERT(sAllocationDevVAddr.uiAddr == psReservation->sBase.uiAddr);

    psReservation->psDevmemHeap = psDevmemHeap;
    *ppsReservationPtr = psReservation;

    return PVRSRV_OK;

    /*
      error exit paths follow
    */

 e1:
	OSFreeMem(psReservation);

 e0:
    PVR_ASSERT(eError != PVRSRV_OK);
    return eError;
}

PVRSRV_ERROR
DevmemIntUnreserveRange(DEVMEMINT_RESERVATION *psReservation)
{
    /* FIXME: No error from mmu_free? */
    MMU_Free (psReservation->psDevmemHeap->psDevmemCtx->psMMUContext,
              psReservation->sBase,
              psReservation->uiLength);

	OSFreeMem(psReservation);

    return PVRSRV_OK;
}

PVRSRV_ERROR
DevmemIntHeapDestroy(
                     DEVMEMINT_HEAP *psDevmemHeap
                     )
{

	PVR_DPF((PVR_DBG_MESSAGE, "DevmemIntHeapDestroy"));

    if (psDevmemHeap->ui32RefCount != 1)
    {
        PVR_DPF((PVR_DBG_ERROR, "BUG!  DevmemIntHeapDestroy called but has too many references (%d) "
                 "which probably means allocations have been made from the heap and not freed",
                 psDevmemHeap->ui32RefCount));
        return PVRSRV_ERROR_DEVICEMEM_ALLOCATIONS_REMAIN_IN_HEAP;
    }

    PVR_ASSERT(psDevmemHeap->ui32RefCount == 1);


    psDevmemHeap->psDevmemCtx->ui32RefCount --;

	OSFreeMem(psDevmemHeap);
	return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       DevmemIntCtxDestroy
@Description    Destroy that created by DevmemIntCtxCreate
@Input          psDevmemCtx   Device Memory context
@Return         cannot fail.
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemIntCtxDestroy(
                    DEVMEMINT_CTX *psDevmemCtx
                    )
{
	PVRSRV_DEVICE_NODE *psDevNode = psDevmemCtx->psDevNode;

	PVR_DPF((PVR_DBG_MESSAGE, "DevmemCtx_Destroy"));

	if (psDevNode->pfnUnregisterMemoryContext)
	{
		psDevNode->pfnUnregisterMemoryContext(psDevmemCtx->hPrivData);
	}
    MMU_ContextDestroy(psDevmemCtx->psMMUContext);

    /* FIXME:  relinquish ref-count on devnode? how! */

	OSFreeMem(psDevmemCtx);
	return PVRSRV_OK;
}

/*************************************************************************/ /*!
@Function       DevmemSLCFlushInvalRequest
@Description    Requests a SLC Flush and Invalidate
@Input          psDeviceNode    Device node
@Input          psPmr           PMR
@Return         PVRSRV_OK
*/ /**************************************************************************/
PVRSRV_ERROR
DevmemSLCFlushInvalRequest(PVRSRV_DEVICE_NODE *psDeviceNode,
							PMR *psPmr)
{

	/* invoke SLC flush and invalidate request */
	psDeviceNode->pfnSLCCacheInvalidateRequest(psDeviceNode, psPmr);

	return PVRSRV_OK;
}

#if defined (PDUMP)
IMG_UINT32 DevmemIntMMUContextID(DEVMEMINT_CTX *psDevMemContext)
{
	IMG_UINT32 ui32MMUContextID;
	MMU_AcquirePDumpMMUContext(psDevMemContext->psMMUContext, &ui32MMUContextID);
	return ui32MMUContextID;
}

PVRSRV_ERROR
DevmemIntPDumpSaveToFileVirtual(DEVMEMINT_CTX *psDevmemCtx,
                                IMG_DEV_VIRTADDR sDevAddrStart,
                                IMG_DEVMEM_SIZE_T uiSize,
                                IMG_UINT32 ui32ArraySize,
                                const IMG_CHAR *pszFilename,
								IMG_UINT32 ui32FileOffset,
								IMG_UINT32 ui32PDumpFlags)
{
    PVRSRV_ERROR eError;
    IMG_UINT32 uiPDumpMMUCtx;

    PVR_UNREFERENCED_PARAMETER(ui32ArraySize);

	eError = MMU_AcquirePDumpMMUContext(psDevmemCtx->psMMUContext,
										&uiPDumpMMUCtx);

    PVR_ASSERT(eError == PVRSRV_OK);

    /*
      The following SYSMEM refers to the 'MMU Context', hence it
      should be the MMU context, not the PMR, that says what the PDump
      MemSpace tag is?
      From a PDump P.O.V. it doesn't matter which name space we use as long
      as that MemSpace is used on the 'MMU Context' we're dumping from
    */
    eError = PDumpMMUSAB(psDevmemCtx->psDevNode->sDevId.pszPDumpDevName,
                            uiPDumpMMUCtx,
                            sDevAddrStart,
                            uiSize,
                            pszFilename,
                            ui32FileOffset,
							ui32PDumpFlags);
    PVR_ASSERT(eError == PVRSRV_OK);

	MMU_ReleasePDumpMMUContext(psDevmemCtx->psMMUContext);
    return PVRSRV_OK;
}


PVRSRV_ERROR
DevmemIntPDumpBitmap(PVRSRV_DEVICE_NODE *psDeviceNode,
						IMG_CHAR *pszFileName,
						IMG_UINT32 ui32FileOffset,
						IMG_UINT32 ui32Width,
						IMG_UINT32 ui32Height,
						IMG_UINT32 ui32StrideInBytes,
						IMG_DEV_VIRTADDR sDevBaseAddr,
						DEVMEMINT_CTX *psDevMemContext,
						IMG_UINT32 ui32Size,
						PDUMP_PIXEL_FORMAT ePixelFormat,
						IMG_UINT32 ui32AddrMode,
						IMG_UINT32 ui32PDumpFlags)
{
	IMG_UINT32 ui32ContextID;
	PVRSRV_ERROR eError;

	eError = MMU_AcquirePDumpMMUContext(psDevMemContext->psMMUContext, &ui32ContextID);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "DevmemIntPDumpBitmap: Failed to acquire MMU context"));
		return PVRSRV_ERROR_FAILED_TO_ALLOC_MMUCONTEXT_ID;
	}

	eError = PDumpBitmapKM(psDeviceNode,
							pszFileName,
							ui32FileOffset,
							ui32Width,
							ui32Height,
							ui32StrideInBytes,
							sDevBaseAddr,
							ui32ContextID,
							ui32Size,
							ePixelFormat,
							ui32AddrMode,
							ui32PDumpFlags);

	/* Don't care about return value */
	MMU_ReleasePDumpMMUContext(psDevMemContext->psMMUContext);

	return eError;
}
#endif
