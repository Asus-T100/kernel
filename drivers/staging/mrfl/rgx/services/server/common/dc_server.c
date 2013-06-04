/**************************************************************************/ /*!
@File
@Title          Server side Display Class functions
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side functions of the Display Class
                interface.
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
*/ /***************************************************************************/

#include "allocmem.h"
#include "lock.h"
#include "osfunc.h"
#include "img_types.h"
#include "scp.h"
#include "dc_server.h"
#include "kerneldisplay.h"
#include "pvr_debug.h"
#include "pmr.h"
#include "pdump_physmem.h"
#include "sync_server.h"
#include "pvrsrv.h"
#include "debug_request_ids.h"

#include <linux/kernel.h>

struct _DC_DISPLAY_CONTEXT_
{
	DC_DEVICE		*psDevice;
	SCP_CONTEXT		*psSCPContext;
	IMG_HANDLE		hDisplayContext;
	IMG_UINT32		ui32ConfigsInFlight;
	IMG_UINT32		ui32RefCount;
	POS_LOCK		hLock;
	IMG_UINT32		ui32TokenOut;
	IMG_UINT32		ui32TokenIn;

	IMG_HANDLE		hCmdCompNotify;

	IMG_BOOL		bIssuedNullFlip;
	IMG_HANDLE		hMISR;
	IMG_HANDLE		hDebugNotify;
};

struct _DC_DEVICE_
{
	const DC_DEVICE_FUNCTIONS	*psFuncTable;
	IMG_UINT32					ui32MaxConfigsInFlight;
	IMG_HANDLE					hDeviceData;
	IMG_UINT32					ui32RefCount;
	POS_LOCK					hLock;
	IMG_UINT32					ui32Index;
	IMG_HANDLE					psEventList;
	IMG_HANDLE					hSystemBuffer;
	PMR							*psSystemBufferPMR;
	DC_DISPLAY_CONTEXT			sSystemContext;
	DC_DEVICE					*psNext;
};

typedef enum _DC_BUFFER_TYPE_
{
	DC_BUFFER_TYPE_UNKNOWN = 0,
	DC_BUFFER_TYPE_ALLOC,
	DC_BUFFER_TYPE_IMPORT,
	DC_BUFFER_TYPE_SYSTEM,
} DC_BUFFER_TYPE;

typedef struct _DC_BUFFER_ALLOC_DATA_
{
	PMR	*psPMR;
} DC_BUFFER_ALLOC_DATA;

typedef struct _DC_BUFFER_IMPORT_DATA_
{
/*
	Required as the DC doesn't need to map the PMR during the import call we
	need to make sure that the PMR doesn't get freed before the DC maps it
	by taking an ref on the PMR during the import and drop it on the unimport.
*/
	IMG_UINT32	ui32NumPlanes;
	PMR			*apsImport[3];
} DC_BUFFER_IMPORT_DATA;

struct _DC_BUFFER_
{
	DC_DISPLAY_CONTEXT	*psDisplayContext;
	DC_BUFFER_TYPE		eType;
	union {
		DC_BUFFER_ALLOC_DATA	sAllocData;
		DC_BUFFER_IMPORT_DATA	sImportData;
	} uBufferData;
	IMG_HANDLE			hBuffer;
	IMG_UINT32			ui32MapCount;
	IMG_UINT32			ui32RefCount;
	POS_LOCK			hLock;
	POS_LOCK			hMapLock;
};

typedef struct _DC_CMD_RDY_DATA_
{
	DC_DISPLAY_CONTEXT			*psDisplayContext;
	IMG_UINT32					ui32BufferCount;
	PVRSRV_SURFACE_CONFIG_INFO	*pasSurfAttrib;
	IMG_HANDLE					*pahBuffer;
	IMG_UINT32					ui32DisplayPeriod;
} DC_CMD_RDY_DATA;

typedef struct _DC_CMD_COMP_DATA_
{
	DC_DISPLAY_CONTEXT	*psDisplayContext;
	IMG_UINT32			ui32BufferCount;
	DC_BUFFER			**apsBuffer;
	IMG_UINT32			ui32Token;
} DC_CMD_COMP_DATA;

typedef struct _DC_BUFFER_PMR_DATA_
{
	DC_BUFFER				*psBuffer;			/*!< The buffer this PMR private data refers to */
	IMG_DEVMEM_LOG2ALIGN_T	uiLog2PageSize;		/*!< Log 2 of the buffers pagesize */
	IMG_UINT32				ui32PageCount;		/*!< Number of pages in this buffer */
	PHYS_HEAP				*psPhysHeap;		/*!< The physical heap the memory resides on */
	IMG_DEV_PHYADDR			*pasDevPAddr;		/*!< Pointer to an array of device physcial addresses */
	IMG_PVOID				pvLinAddr;			/*!< CPU virtual pointer or NULL if the DC driver didn't have one */

	IMG_HANDLE				hPDumpAllocInfo;	/*!< Handle to PDump alloc data */
	IMG_BOOL				bPDumpMalloced;		/*!< Did we get as far as PDump alloc? */
} DC_BUFFER_PMR_DATA;

POS_LOCK g_hDCListLock;

DC_DEVICE *g_psDCDeviceList;
IMG_UINT32 g_ui32DCDeviceCount;
IMG_UINT32 g_ui32DCNextIndex;

#if defined(DC_DEBUG) && defined(REFCOUNT_DEBUG)
#define DC_REFCOUNT_PRINT(fmt, ...)		\
	PVRSRVDebugPrintf(PVR_DBG_WARNING,	\
			  __FILE__,		\
			  __LINE__,		\
			  fmt,			\
			  __VA_ARGS__)
#else
#define DC_REFCOUNT_PRINT(fmt, ...)
#endif

#if defined(DC_DEBUG)
#define DC_DEBUG_PRINT(fmt, ...)			\
	PVRSRVDebugPrintf(PVR_DBG_WARNING,		\
			  __FILE__,			\
			  __LINE__,			\
			  fmt,				\
			  __VA_ARGS__)
#else
#define DC_DEBUG_PRINT(fmt, ...)
#endif

/*****************************************************************************
 *                             Private functions                             *
 *****************************************************************************/

static IMG_VOID _DCDeviceAcquireRef(DC_DEVICE *psDevice)
{
	OSLockAcquire(psDevice->hLock);
	psDevice->ui32RefCount++;
	DC_REFCOUNT_PRINT("%s: DC device %p, refcount = %d",
					  __FUNCTION__, psDevice, psDevice->ui32RefCount);
	OSLockRelease(psDevice->hLock);
}

static IMG_VOID _DCDeviceReleaseRef(DC_DEVICE *psDevice)
{
	IMG_UINT32 ui32RefCount;

	OSLockAcquire(psDevice->hLock);
	ui32RefCount = --psDevice->ui32RefCount;
	OSLockRelease(psDevice->hLock);

	if (ui32RefCount == 0)
	{
		OSLockAcquire(g_hDCListLock);
		if (psDevice == g_psDCDeviceList)
		{
			g_psDCDeviceList = psDevice->psNext;
		}
		else
		{
			DC_DEVICE *psTmp = g_psDCDeviceList;
	
			while (psTmp->psNext != psDevice)
			{
				psTmp = psTmp->psNext;
			}
			psTmp->psNext = g_psDCDeviceList->psNext;
		}
	
		g_ui32DCDeviceCount--;
		OSLockRelease(g_hDCListLock);
	}
	else
	{
		/* Signal this devices event list as the unload might be blocked on it */
		OSEventObjectSignal(psDevice->psEventList);
	}
	DC_REFCOUNT_PRINT("%s: DC device %p, refcount = %d",
					  __FUNCTION__, psDevice, ui32RefCount);
}

static IMG_VOID _DCDisplayContextAcquireRef(DC_DISPLAY_CONTEXT *psDisplayContext)
{
	OSLockAcquire(psDisplayContext->hLock);
	psDisplayContext->ui32RefCount++;
	DC_REFCOUNT_PRINT("%s: DC display context %p, refcount = %d",
					  __FUNCTION__, psDisplayContext, psDisplayContext->ui32RefCount);
	OSLockRelease(psDisplayContext->hLock);
}

static IMG_VOID _DCDisplayContextReleaseRef(DC_DISPLAY_CONTEXT *psDisplayContext)
{
	IMG_UINT32 ui32RefCount;

	OSLockAcquire(psDisplayContext->hLock);
	ui32RefCount = --psDisplayContext->ui32RefCount;
	OSLockRelease(psDisplayContext->hLock);

	if (ui32RefCount == 0)
	{
		DC_DEVICE *psDevice = psDisplayContext->psDevice;

		PVRSRVUnregisterDbgRequestNotify(psDisplayContext->hDebugNotify);

		/* unregister the device from cmd complete notifications */
		PVRSRVUnregisterCmdCompleteNotify(psDisplayContext->hCmdCompNotify);
		psDisplayContext->hCmdCompNotify = IMG_NULL;

		OSUninstallMISR(psDisplayContext->hMISR);
		SCPDestroy(psDisplayContext->psSCPContext);
		psDevice->psFuncTable->pfnContextDestroy(psDisplayContext->hDisplayContext);
		_DCDeviceReleaseRef(psDevice);
		OSLockDestroy(psDisplayContext->hLock);
		OSFreeMem(psDisplayContext);
	}

	DC_REFCOUNT_PRINT("%s: DC display context %p, refcount = %d",
					  __FUNCTION__, psDisplayContext, ui32RefCount);
}

static IMG_VOID _DCBufferAcquireRef(DC_BUFFER *psBuffer)
{
	OSLockAcquire(psBuffer->hLock);
	psBuffer->ui32RefCount++;
	DC_REFCOUNT_PRINT("%s: DC buffer %p, refcount = %d",
					  __FUNCTION__, psBuffer, psBuffer->ui32RefCount);
	OSLockRelease(psBuffer->hLock);
}


static IMG_VOID _DCFreeAllocedBuffer(DC_BUFFER *psBuffer)
{
	DC_DISPLAY_CONTEXT *psDisplayContext = psBuffer->psDisplayContext;
	DC_DEVICE *psDevice = psDisplayContext->psDevice;

	psDevice->psFuncTable->pfnBufferFree(psBuffer->hBuffer);
	_DCDisplayContextReleaseRef(psDisplayContext);
}

static IMG_VOID _DCFreeImportedBuffer(DC_BUFFER *psBuffer)
{
	DC_DISPLAY_CONTEXT *psDisplayContext = psBuffer->psDisplayContext;
	DC_DEVICE *psDevice = psDisplayContext->psDevice;
	IMG_UINT32 i;

	for (i=0;i<psBuffer->uBufferData.sImportData.ui32NumPlanes;i++)
	{
		PMRUnrefPMR(psBuffer->uBufferData.sImportData.apsImport[i]);
	}
	psDevice->psFuncTable->pfnBufferFree(psBuffer->hBuffer);
	_DCDisplayContextReleaseRef(psDisplayContext);
}

static IMG_VOID _DCFreeSystemBuffer(DC_BUFFER *psBuffer)
{
	DC_DISPLAY_CONTEXT *psDisplayContext = psBuffer->psDisplayContext;
	DC_DEVICE *psDevice = psDisplayContext->psDevice;

	psDevice->psFuncTable->pfnBufferSystemRelease(psBuffer->hBuffer);
	_DCDeviceReleaseRef(psDevice);
}

/*
	Drop a reference on the buffer. Last person gets to free it
*/
static IMG_VOID _DCBufferReleaseRef(DC_BUFFER *psBuffer)
{
	IMG_UINT32 ui32RefCount;

	OSLockAcquire(psBuffer->hLock);
	ui32RefCount = --psBuffer->ui32RefCount;
	OSLockRelease(psBuffer->hLock);

	if (ui32RefCount == 0)
	{
		switch (psBuffer->eType)
		{
			case DC_BUFFER_TYPE_ALLOC:
					_DCFreeAllocedBuffer(psBuffer);
					break;
			case DC_BUFFER_TYPE_IMPORT:
					_DCFreeImportedBuffer(psBuffer);
					break;
			case DC_BUFFER_TYPE_SYSTEM:
					_DCFreeSystemBuffer(psBuffer);
					break;
			default:
					PVR_ASSERT(IMG_FALSE);
		}
		OSLockDestroy(psBuffer->hMapLock);
		OSLockDestroy(psBuffer->hLock);
		OSFreeMem(psBuffer);
	}
	DC_REFCOUNT_PRINT("%s: DC buffer %p, refcount = %d",
					  __FUNCTION__, psBuffer, ui32RefCount);
}

static PVRSRV_ERROR _DCBufferMap(DC_BUFFER *psBuffer)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	OSLockAcquire(psBuffer->hMapLock);
	if (psBuffer->ui32MapCount++ == 0)
	{
		DC_DEVICE *psDevice = psBuffer->psDisplayContext->psDevice;

		if(psDevice->psFuncTable->pfnBufferMap)
		{
			eError = psDevice->psFuncTable->pfnBufferMap(psBuffer->hBuffer);
			if (eError != PVRSRV_OK)
			{
				goto out_unlock;
			}
		}

		_DCBufferAcquireRef(psBuffer);
	}

	DC_REFCOUNT_PRINT("%s: DC buffer %p, MapCount = %d",
					  __FUNCTION__, psBuffer, psBuffer->ui32MapCount);

out_unlock:
	OSLockRelease(psBuffer->hMapLock);
	return eError;
}

static IMG_VOID _DCBufferUnmap(DC_BUFFER *psBuffer)
{
	DC_DEVICE *psDevice = psBuffer->psDisplayContext->psDevice;
	IMG_UINT32 ui32MapCount;

	OSLockAcquire(psBuffer->hMapLock);
	ui32MapCount = --psBuffer->ui32MapCount;
	OSLockRelease(psBuffer->hMapLock);

	if (ui32MapCount == 0)
	{
		if(psDevice->psFuncTable->pfnBufferUnmap)
		{
			psDevice->psFuncTable->pfnBufferUnmap(psBuffer->hBuffer);
		}

		_DCBufferReleaseRef(psBuffer);
	}
	DC_REFCOUNT_PRINT("%s: DC Buffer %p, MapCount = %d",
					  __FUNCTION__, psBuffer, ui32MapCount);
}

static PVRSRV_ERROR _DCDeviceBufferArrayCreate(IMG_UINT32 ui32BufferCount,
											   DC_BUFFER **papsBuffers,
											   IMG_HANDLE **pahDeviceBuffers)
{
	IMG_HANDLE *ahDeviceBuffers;
	IMG_UINT32 i;

	/* Create an array of the DC's private Buffer handles */
	ahDeviceBuffers = OSAllocMem(sizeof(IMG_HANDLE) * ui32BufferCount);
	if (ahDeviceBuffers == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	OSMemSet(ahDeviceBuffers, 0, sizeof(IMG_HANDLE) * ui32BufferCount);

	for (i=0;i<ui32BufferCount;i++)
	{
		ahDeviceBuffers[i] = papsBuffers[i]->hBuffer;
	}	

	*pahDeviceBuffers = ahDeviceBuffers;

	return PVRSRV_OK;
}

static IMG_VOID _DCDeviceBufferArrayDestroy(IMG_HANDLE ahDeviceBuffers)
{
	OSFreeMem(ahDeviceBuffers);
}

static IMG_BOOL _DCDisplayContextReady(IMG_PVOID hReadyData)
{
	DC_CMD_RDY_DATA *psReadyData = (DC_CMD_RDY_DATA *) hReadyData;
	DC_DISPLAY_CONTEXT *psDisplayContext = psReadyData->psDisplayContext;
	DC_DEVICE *psDevice = psDisplayContext->psDevice;

	if (psDisplayContext->ui32ConfigsInFlight >= psDevice->ui32MaxConfigsInFlight)
	{
		/*
			We're at the DC's max commands in-flight so don't take this command
			off the queue
		*/
		return IMG_FALSE;
	}

	return IMG_TRUE;
}

static IMG_VOID _DCDisplayContextConfigure(IMG_PVOID hReadyData,
										   IMG_PVOID hCompleteData)
{
	DC_CMD_RDY_DATA *psReadyData = (DC_CMD_RDY_DATA *) hReadyData;
	DC_DISPLAY_CONTEXT *psDisplayContext = psReadyData->psDisplayContext;
	DC_DEVICE *psDevice = psDisplayContext->psDevice;

	OSLockAcquire(psDisplayContext->hLock);
	psDisplayContext->ui32ConfigsInFlight++;
	OSLockRelease(psDisplayContext->hLock);

#if defined(DC_DEBUG)
	{
		DC_DEBUG_PRINT("_DCDisplayContextConfigure: Send command (%d) out", 
				((DC_CMD_COMP_DATA*) hCompleteData)->ui32Token);
	}
#endif /* DC_DEBUG */

	/*
		Note: We've already done all the acquire refs at
		      DCDisplayContextConfigure time.
	*/
	psDevice->psFuncTable->pfnContextConfigure(psDisplayContext->hDisplayContext,
											   psReadyData->ui32BufferCount,
											   psReadyData->pasSurfAttrib,
											   psReadyData->pahBuffer,
											   psReadyData->ui32DisplayPeriod,
											   hCompleteData);
}

/*
	_DCDisplayContextRun

	Kick the MISR which will check for any commands which can be processed
*/
static INLINE IMG_VOID _DCDisplayContextRun(DC_DISPLAY_CONTEXT *psDisplayContext)
{
	OSScheduleMISR(psDisplayContext->hMISR);
}

/*
	_DCDisplayContextMISR

	This gets called when this MISR is fired
*/
static IMG_VOID _DCDisplayContextMISR(IMG_VOID *pvData)
{
	DC_DISPLAY_CONTEXT *psDisplayContext = pvData;

	SCPRun(psDisplayContext->psSCPContext);
}

/*
 * PMR related functions and structures
 */

/*
	Callback function for locking the system physical page addresses.
	As we acquire the display memory at PMR create time there is nothing
	to do here.
*/
static PVRSRV_ERROR _DCPMRLockPhysAddresses(PMR_IMPL_PRIVDATA pvPriv,
											IMG_UINT32 uiLog2DevPageSize)
{
	DC_BUFFER_PMR_DATA *psPMRPriv = pvPriv;
	DC_BUFFER *psBuffer = psPMRPriv->psBuffer;
	DC_DEVICE *psDevice = psBuffer->psDisplayContext->psDevice;
	PVRSRV_ERROR eError;

	if (uiLog2DevPageSize < psPMRPriv->uiLog2PageSize)
	{
		eError = PVRSRV_ERROR_PMR_INCOMPATIBLE_CONTIGUITY;
		goto fail_contigcheck;
	}

	psPMRPriv->pasDevPAddr = OSAllocMem(sizeof(IMG_DEV_PHYADDR) *
							 psPMRPriv->ui32PageCount);
	if (psPMRPriv->pasDevPAddr == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_alloc;
	}

	OSMemSet(psPMRPriv->pasDevPAddr,
			 0,
			 sizeof(IMG_DEV_PHYADDR) * psPMRPriv->ui32PageCount);

	eError = psDevice->psFuncTable->pfnBufferAcquire(psBuffer->hBuffer,
													 psPMRPriv->pasDevPAddr,
													 &psPMRPriv->pvLinAddr);
	if (eError != PVRSRV_OK)
	{
		goto fail_query;
	}

	return PVRSRV_OK;

fail_query:
	OSFreeMem(psPMRPriv->pasDevPAddr);
fail_alloc:
fail_contigcheck:
	return eError;
}

static PVRSRV_ERROR _DCPMRUnlockPhysAddresses(PMR_IMPL_PRIVDATA pvPriv)
{
	DC_BUFFER_PMR_DATA *psPMRPriv = pvPriv;
	DC_BUFFER *psBuffer = psPMRPriv->psBuffer;
	DC_DEVICE *psDevice = psBuffer->psDisplayContext->psDevice;

	psDevice->psFuncTable->pfnBufferRelease(psBuffer->hBuffer);
	OSFreeMem(psPMRPriv->pasDevPAddr);

	return PVRSRV_OK;
}

static PVRSRV_ERROR _DCPMRDevPhysAddr(PMR_IMPL_PRIVDATA pvPriv,
									  IMG_DEVMEM_OFFSET_T uiOffset,
									  IMG_DEV_PHYADDR *psDevAddrPtr)
{
	DC_BUFFER_PMR_DATA *psPMRPriv = pvPriv;
    IMG_UINT32 uiNumPages;
    IMG_UINT32 uiLog2PageSize;
    IMG_UINT32 uiPageSize;
    IMG_UINT32 uiPageIndex;
    IMG_UINT32 uiInPageOffset;
    IMG_DEV_PHYADDR sDevAddr;

    uiLog2PageSize = psPMRPriv->uiLog2PageSize;
    uiNumPages = psPMRPriv->ui32PageCount;

    uiPageSize = 1ULL << uiLog2PageSize;

    uiPageIndex = (IMG_UINT32)(uiOffset >> uiLog2PageSize);
    /* verify the cast */
    /* N.B.  Strictly... this could be triggered by an illegal
       uiOffset arg too. */
    PVR_ASSERT(uiPageIndex << uiLog2PageSize == uiOffset);

    uiInPageOffset = (IMG_UINT32)(uiOffset - (uiPageIndex << uiLog2PageSize));
    PVR_ASSERT(uiOffset == (uiPageIndex << uiLog2PageSize) + uiInPageOffset);

    PVR_ASSERT(uiPageIndex < uiNumPages);
    PVR_ASSERT(uiInPageOffset < uiPageSize);

	/* FIXME: Remove sysaddr */
    sDevAddr.uiAddr = psPMRPriv->pasDevPAddr[uiPageIndex].uiAddr;
	PVR_ASSERT((sDevAddr.uiAddr & (uiPageSize - 1)) == 0);

    *psDevAddrPtr = sDevAddr;
    psDevAddrPtr->uiAddr += uiInPageOffset;

    return PVRSRV_OK;
}

#if defined(PDUMP)
static IMG_VOID _PDumpPMRMalloc(const PMR *psPMR,
								IMG_DEVMEM_SIZE_T uiSize,
								IMG_DEVMEM_ALIGN_T uiBlockSize,
								IMG_HANDLE *phPDumpAllocInfoPtr)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hPDumpAllocInfo;
	IMG_CHAR aszMemspaceName[30];
	IMG_CHAR aszSymbolicName[30];
	IMG_DEVMEM_OFFSET_T uiOffset;
	IMG_DEVMEM_OFFSET_T uiNextSymName;

	uiOffset = 0;
	eError = PMR_PDumpSymbolicAddr(psPMR,
								   uiOffset,
								   sizeof(aszMemspaceName),
								   &aszMemspaceName[0],
								   sizeof(aszSymbolicName),
								   &aszSymbolicName[0],
								   &uiOffset,
								   &uiNextSymName);
	PVR_ASSERT(eError == PVRSRV_OK);
	PVR_ASSERT(uiOffset == 0);
	PVR_ASSERT((uiOffset + uiSize) <= uiNextSymName);

	PDumpPMRMalloc(aszMemspaceName,
				   aszSymbolicName,
				   uiSize,
				   uiBlockSize,
				   IMG_TRUE,
				   &hPDumpAllocInfo);

	*phPDumpAllocInfoPtr = hPDumpAllocInfo;
}
#else	/* PDUMP */
static IMG_VOID
_PDumpPMRMalloc(const PMR *psPMR,
                IMG_DEVMEM_SIZE_T uiSize,
                IMG_DEVMEM_ALIGN_T uiBlockSize,
                IMG_HANDLE *phPDumpAllocInfoPtr)
{
	PVR_UNREFERENCED_PARAMETER(psPMR);
	PVR_UNREFERENCED_PARAMETER(uiSize);
	PVR_UNREFERENCED_PARAMETER(uiBlockSize);
	PVR_UNREFERENCED_PARAMETER(phPDumpAllocInfoPtr);
}
#endif	/* PDUMP */

static PVRSRV_ERROR _DCPMRFinalize(PMR_IMPL_PRIVDATA pvPriv)
{
	DC_BUFFER_PMR_DATA *psPMRPriv = pvPriv;

	/* Conditionally do the PDump free, because if CreatePMR failed we
	   won't have done the PDump MALLOC.  */
	if (psPMRPriv->bPDumpMalloced)
	{
		PDumpPMRFree(psPMRPriv->hPDumpAllocInfo);
	}

	PhysHeapRelease(psPMRPriv->psPhysHeap);
	_DCBufferReleaseRef(psPMRPriv->psBuffer);
	OSFreeMem(psPMRPriv);

	return PVRSRV_OK;
}

static PVRSRV_ERROR _DCPMRReadBytes(PMR_IMPL_PRIVDATA pvPriv,
									IMG_DEVMEM_OFFSET_T uiOffset,
									IMG_UINT8 *pcBuffer,
									IMG_SIZE_T uiBufSz,
									IMG_SIZE_T *puiNumBytes)
{
    DC_BUFFER_PMR_DATA *psPMRPriv = pvPriv;
    IMG_CPU_PHYADDR sCpuPAddr;
    IMG_SIZE_T uiBytesCopied = 0;
    IMG_SIZE_T uiBytesToCopy = uiBufSz;
    IMG_SIZE_T uiBytesCopyableFromPage;
    IMG_VOID *pvMapping;
    IMG_UINT8 *pcKernelPointer;
    IMG_UINT32 uiBufferOffset = 0;
    IMG_UINT32 uiPageIndex;
    IMG_UINT32 uiInPageOffset;

	/* If we already have a CPU mapping just us it */
	if (psPMRPriv->pvLinAddr)
	{
		pcKernelPointer = psPMRPriv->pvLinAddr;
		OSMemCopy(pcBuffer, &pcKernelPointer[uiOffset], uiBufSz);
		*puiNumBytes = uiBufSz;
		return PVRSRV_OK;
	}

	/* Copy the data page by page */
    while (uiBytesToCopy > 0)
    {
        /* we have to kmap one page in at a time */
        uiPageIndex = uiOffset >> psPMRPriv->uiLog2PageSize;

        uiInPageOffset = uiOffset - (uiPageIndex << psPMRPriv->uiLog2PageSize);
        uiBytesCopyableFromPage = uiBytesToCopy;
        if (uiBytesCopyableFromPage + uiInPageOffset > (1<<psPMRPriv->uiLog2PageSize))
        {
            uiBytesCopyableFromPage = (1 << psPMRPriv->uiLog2PageSize)-uiInPageOffset;
        }

		PhysHeapDevPAddrToCpuPAddr(psPMRPriv->psPhysHeap, &sCpuPAddr, &psPMRPriv->pasDevPAddr[uiPageIndex]);

        pvMapping = OSMapPhysToLin(sCpuPAddr,
								   1 << psPMRPriv->uiLog2PageSize,
								   0);
        PVR_ASSERT(pvMapping != IMG_NULL);
        pcKernelPointer = pvMapping;
        OSMemCopy(&pcBuffer[uiBufferOffset], &pcKernelPointer[uiInPageOffset], uiBytesCopyableFromPage);
        OSUnMapPhysToLin(pvMapping, 1 << psPMRPriv->uiLog2PageSize, 0);

        uiBufferOffset += uiBytesCopyableFromPage;
        uiBytesToCopy -= uiBytesCopyableFromPage;
        uiOffset += uiBytesCopyableFromPage;
        uiBytesCopied += uiBytesCopyableFromPage;
    }

    *puiNumBytes = uiBytesCopied;
    return PVRSRV_OK;
}

static PMR_IMPL_FUNCTAB sDCPMRFuncTab = {
	.pfnLockPhysAddresses			= _DCPMRLockPhysAddresses,
	.pfnUnlockPhysAddresses			= _DCPMRUnlockPhysAddresses,
	.pfnDevPhysAddr					= _DCPMRDevPhysAddr,
	.pfnReadBytes					= _DCPMRReadBytes,
	.pfnFinalize					= _DCPMRFinalize,
};

static PVRSRV_ERROR _DCCreatePMR(IMG_DEVMEM_LOG2ALIGN_T uiLog2PageSize,
								 IMG_UINT32 ui32PageCount,
								 IMG_UINT32 ui32PhysHeapID,
								 DC_BUFFER *psBuffer,
								 PMR **ppsPMR)
{
	DC_BUFFER_PMR_DATA *psPMRPriv;
	PHYS_HEAP *psPhysHeap;
	IMG_DEVMEM_SIZE_T uiBufferSize;
	IMG_HANDLE hPDumpAllocInfo;
	PVRSRV_ERROR eError;
	IMG_BOOL bMappingTable = IMG_TRUE;

	/*
		Create the PMR for this buffer.

		Note: At this stage we don't need to know the physcial pages just
		the page size and the size of the PMR. The 1st call that needs the
		physcial pages will cause a request into the DC driver (pfnBufferQuery)
	*/
	psPMRPriv = OSAllocMem(sizeof(DC_BUFFER_PMR_DATA));
	if (psPMRPriv == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_privalloc;
	}

	OSMemSet(psPMRPriv, 0, sizeof(DC_BUFFER_PMR_DATA));

	/* Acquire the physical heap the memory is on */
	eError = PhysHeapAcquire(ui32PhysHeapID, &psPhysHeap);
	if (eError != PVRSRV_OK)
	{
		goto fail_physheap;
	}

	/* Take a reference on the buffer (for the copy in the PMR) */
	_DCBufferAcquireRef(psBuffer);

	/* Fill in the private data for the PMR */
	psPMRPriv->uiLog2PageSize = uiLog2PageSize;
	psPMRPriv->ui32PageCount = ui32PageCount;
	psPMRPriv->psPhysHeap = psPhysHeap;
	psPMRPriv->pasDevPAddr = IMG_NULL;
	psPMRPriv->psBuffer = psBuffer;

	uiBufferSize = (1 << uiLog2PageSize) * ui32PageCount;

	/* Create the PMR for the MM layer */
	eError = PMRCreatePMR(psPhysHeap,
						  uiBufferSize,
						  uiBufferSize,
						  1,
						  1,
						  &bMappingTable,
						  uiLog2PageSize,
						  PVRSRV_MEMALLOCFLAG_UNCACHED,
						  "DISPLAY",
						  &sDCPMRFuncTab,
						  psPMRPriv,
						  ppsPMR);

	if (eError != PVRSRV_OK)
	{
		goto fail_pmrcreate;
	}

	_PDumpPMRMalloc(*ppsPMR,
					uiBufferSize,
					1U<<uiLog2PageSize,
					&hPDumpAllocInfo);
#if defined(PDUMP)
	psPMRPriv->hPDumpAllocInfo = hPDumpAllocInfo;
	psPMRPriv->bPDumpMalloced = IMG_TRUE;
#endif

	return PVRSRV_OK;

fail_pmrcreate:
	PhysHeapRelease(psPhysHeap);
fail_physheap:
	OSFreeMem(psPMRPriv);
fail_privalloc:
	return eError;
}

static IMG_VOID _DCDisplayContextNotify(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle)
{
	DC_DISPLAY_CONTEXT	*psDisplayContext = (DC_DISPLAY_CONTEXT*) hCmdCompHandle;

	_DCDisplayContextRun(psDisplayContext);
}

static IMG_VOID _DCDebugRequest(PVRSRV_DBGREQ_HANDLE hDebugRequestHandle, IMG_UINT32 ui32VerbLevel)
{
	DC_DISPLAY_CONTEXT	*psDisplayContext = (DC_DISPLAY_CONTEXT*) hDebugRequestHandle;

	switch(ui32VerbLevel)
	{
		case DEBUG_REQUEST_VERBOSITY_LOW:
			PVR_LOG(("Configs in-flight = %d", psDisplayContext->ui32ConfigsInFlight));
			break;

		case DEBUG_REQUEST_VERBOSITY_MEDIUM:
			PVR_LOG(("Display context SCP status"));
			SCPDumpStatus(psDisplayContext->psSCPContext);
			break;
			
		default:
			break;
	}
}

/*****************************************************************************
 * Public interface functions exposed through the bridge to services client  *
 *****************************************************************************/

PVRSRV_ERROR DCDevicesQueryCount(IMG_UINT32 *pui32DeviceCount)
{
	*pui32DeviceCount = g_ui32DCDeviceCount;
	return PVRSRV_OK;
}

PVRSRV_ERROR DCDevicesEnumerate(IMG_UINT32 ui32DeviceArraySize,
								IMG_UINT32 *pui32DeviceCount,
								IMG_UINT32 *paui32DeviceIndex)
{
	IMG_UINT32 i;
	IMG_UINT32 ui32LoopCount;
	DC_DEVICE *psTmp = g_psDCDeviceList;

	OSLockAcquire(g_hDCListLock);

	if (g_ui32DCDeviceCount > ui32DeviceArraySize)
	{
		ui32LoopCount = ui32DeviceArraySize;
	}
	else
	{
		ui32LoopCount = g_ui32DCDeviceCount;
	}
	
	for (i=0;i<ui32LoopCount;i++)
	{
		PVR_ASSERT(psTmp != IMG_NULL);
		paui32DeviceIndex[i] = psTmp->ui32Index;
		psTmp = psTmp->psNext;
	}

	*pui32DeviceCount = ui32LoopCount;
	OSLockRelease(g_hDCListLock);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCDeviceAcquire(IMG_UINT32 ui32DeviceIndex,
							 DC_DEVICE **ppsDevice)
{
	DC_DEVICE *psDevice = g_psDCDeviceList;

	if (psDevice == IMG_NULL)
	{
		return PVRSRV_ERROR_NO_DC_DEVICES_FOUND;
	}

	while(psDevice->ui32Index != ui32DeviceIndex)
	{
		psDevice = psDevice->psNext;
		if (psDevice == IMG_NULL)
		{
			return PVRSRV_ERROR_NO_DC_DEVICES_FOUND;
		}
	}

	_DCDeviceAcquireRef(psDevice);
	*ppsDevice = psDevice;
	return PVRSRV_OK;
}

PVRSRV_ERROR DCDeviceRelease(DC_DEVICE *psDevice)
{
	_DCDeviceReleaseRef(psDevice);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCGetInfo(DC_DEVICE *psDevice,
					   DC_DISPLAY_INFO *psDisplayInfo)
{
	psDevice->psFuncTable->pfnGetInfo(psDevice->hDeviceData,
									  psDisplayInfo);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCPanelQueryCount(DC_DEVICE *psDevice,
								IMG_UINT32 *pui32NumPanels)
{
	psDevice->psFuncTable->pfnPanelQueryCount(psDevice->hDeviceData,
											  pui32NumPanels);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCPanelQuery(DC_DEVICE *psDevice,
						   IMG_UINT32 ui32PanelsArraySize,
						   IMG_UINT32 *pui32NumPanels,
						   PVRSRV_PANEL_INFO *pasPanelInfo)
{
	psDevice->psFuncTable->pfnPanelQuery(psDevice->hDeviceData,
										 ui32PanelsArraySize,
										 pui32NumPanels,
										 pasPanelInfo);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCFormatQuery(DC_DEVICE *psDevice,
						   IMG_UINT32 ui32FormatArraySize,
						   PVRSRV_SURFACE_FORMAT *pasFormat,
						   IMG_UINT32 *pui32Supported)
{
	psDevice->psFuncTable->pfnFormatQuery(psDevice->hDeviceData,
									   ui32FormatArraySize,
									   pasFormat,
									   pui32Supported);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCDimQuery(DC_DEVICE *psDevice,
						IMG_UINT32 ui32DimSize,
						PVRSRV_SURFACE_DIMS *pasDim,
						IMG_UINT32 *pui32Supported)
{
	psDevice->psFuncTable->pfnDimQuery(psDevice->hDeviceData,
										  ui32DimSize,
										  pasDim,
										  pui32Supported);

	return PVRSRV_OK;
}

/*
	FIXME:
	The system buffer breaks the rule of only calling DC callbacks on first
	ref and last deref. For the pfnBufferSystemAcquire this is expected
	as each call could get back a different buffer, but calls to
	pfnBufferAcquire and pfnBufferRelease could happen multiple times
	for the same buffer
*/
PVRSRV_ERROR DCSystemBufferAcquire(DC_DEVICE *psDevice,
								   IMG_UINT32 *pui32ByteStride,
								   DC_BUFFER **ppsBuffer)
{
	DC_BUFFER *psNew;
	PMR *psPMR;
	PVRSRV_ERROR eError;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2PageSize;
	IMG_UINT32 ui32PageCount;
	IMG_UINT32 ui32PhysHeapID;

	if (psDevice->psFuncTable->pfnBufferSystemAcquire == IMG_NULL)
	{
		eError = PVRSRV_ERROR_NO_SYSTEM_BUFFER;
		goto fail_nopfn;
	}

	psNew = OSAllocMem(sizeof(DC_BUFFER));
	if (psNew == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_alloc;
	}

	OSMemSet(psNew, 0, sizeof(DC_BUFFER));

	eError = OSLockCreate(&psNew->hLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		goto fail_lock;
	}

	eError = OSLockCreate(&psNew->hMapLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		goto fail_maplock;
	}

	eError = psDevice->psFuncTable->pfnBufferSystemAcquire(psDevice->hDeviceData,
														   &uiLog2PageSize,
														   &ui32PageCount,
														   &ui32PhysHeapID,
														   pui32ByteStride,
														   &psNew->hBuffer);
	if (eError != PVRSRV_OK)
	{
		goto fail_bufferacquire;
	}

	psNew->psDisplayContext = &psDevice->sSystemContext;
	psNew->eType = DC_BUFFER_TYPE_SYSTEM;
	psNew->ui32MapCount = 0;
	psNew->ui32RefCount = 1;

	/*
		Creating the PMR for the system buffer is a bit tricky as there is no
		"create" call for it.
		We should only ever have one PMR for the same buffer and so we can't
		just create one every call to this function. We also have to deal with
		the system buffer changing (mode change) so we can't just create the PMR
		once at DC driver register time.
		So what we do is cache the DC's handle to the system buffer and check if
		this call the handle has changed (indicating a mode change) and create
		a new PMR in this case.
	*/
	if (psNew->hBuffer != psDevice->hSystemBuffer)
	{
		if (psDevice->psSystemBufferPMR)
		{
			/*
				Mode change:
				We've already got a system buffer but the DC has given us a new
				one so we need to drop the 2nd reference we took on it as a
				different system buffer will be freed as DC unregister time
			*/
			PMRUnrefPMR(psDevice->psSystemBufferPMR);
		}

		eError = _DCCreatePMR(uiLog2PageSize,
							  ui32PageCount,
							  ui32PhysHeapID,
							  psNew,
							  &psPMR);
		if (eError != PVRSRV_OK)
		{
			goto fail_createpmr;
		}

		psNew->uBufferData.sAllocData.psPMR = psPMR;
		psDevice->hSystemBuffer = psNew->hBuffer;
		psDevice->psSystemBufferPMR = psPMR;

		/*
			Take a 2nd reference on the PMR as we always drop a reference
			in the release call but we don't want the PMR to be freed until
			either a new system buffer as been acquired or the DC device gets
			unregistered
		*/
		PMRRefPMR(psDevice->psSystemBufferPMR);
	}
	else
	{
		/*
			A PMR for the system buffer as already been created so just
			take a reference to the PMR to make sure it doesn't go away
		*/
		PMRRefPMR(psDevice->psSystemBufferPMR);
		psNew->uBufferData.sAllocData.psPMR = psDevice->psSystemBufferPMR;
	}

	/*
		The system buffer is tied to the device unlike all other buffers
		which are tied to a display context.
	*/
	_DCDeviceAcquireRef(psDevice);

	*ppsBuffer = psNew;

	return PVRSRV_OK;

fail_createpmr:
fail_bufferacquire:
	OSLockDestroy(psNew->hMapLock);
fail_maplock:
	OSLockDestroy(psNew->hLock);
fail_lock:
	OSFreeMem(psNew);
fail_alloc:
fail_nopfn:
	return eError;
}

PVRSRV_ERROR DCSystemBufferRelease(DC_BUFFER *psBuffer)
{
	PMRUnrefPMR(psBuffer->uBufferData.sAllocData.psPMR);
	_DCBufferReleaseRef(psBuffer);
	return PVRSRV_OK;
}

PVRSRV_ERROR DCDisplayContextCreate(DC_DEVICE *psDevice,
									DC_DISPLAY_CONTEXT **ppsDisplayContext)
{
	DC_DISPLAY_CONTEXT *psDisplayContext;
	PVRSRV_ERROR eError;

	psDisplayContext = OSAllocMem(sizeof(DC_DISPLAY_CONTEXT));
	if (psDisplayContext == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	psDisplayContext->psDevice = psDevice;
	psDisplayContext->hDisplayContext = IMG_NULL;
	psDisplayContext->ui32TokenOut = 0;
	psDisplayContext->ui32TokenIn = 0;
	psDisplayContext->ui32RefCount = 1;
	psDisplayContext->ui32ConfigsInFlight = 0;
	psDisplayContext->bIssuedNullFlip = IMG_FALSE;

	eError = OSLockCreate(&psDisplayContext->hLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		goto FailLock;
	}

	/* Create a Software Command Processor with 4K CCB size. 
	 * With the HWC it might be possible to reach the limit off the buffer.
	 * This could be bad when the buffers currently on the screen can't be
	 * flipped to the new one, cause the command for them doesn't fit into the
	 * queue (Deadlock). This situation should properly detected to make at
	 * least the debugging easier. */
	eError = SCPCreate(12, &psDisplayContext->psSCPContext);
	if (eError != PVRSRV_OK)
	{
		goto FailSCP;
	}

	eError = psDevice->psFuncTable->pfnContextCreate(psDevice->hDeviceData,
													 &psDisplayContext->hDisplayContext);

	if (eError != PVRSRV_OK)
	{
		goto FailDCDeviceContext;
	}

	_DCDeviceAcquireRef(psDevice);

	/* Create an MISR for our display context */
	eError = OSInstallMISR(&psDisplayContext->hMISR,
						   _DCDisplayContextMISR,
						   psDisplayContext);
	if (eError != PVRSRV_OK)
	{
		goto FailMISR;
	}
	/*
		Register for the command complete callback.

		Note:
		After calling this function our MISR can be called at any point.
	*/
	eError = PVRSRVRegisterCmdCompleteNotify(&psDisplayContext->hCmdCompNotify, _DCDisplayContextNotify, psDisplayContext);
	if (eError != PVRSRV_OK)
	{
		goto FailRegisterCmdComplete;
	}

	/* Register our debug request notify callback */
	eError = PVRSRVRegisterDbgRequestNotify(&psDisplayContext->hDebugNotify,
											_DCDebugRequest,
											DEBUG_REQUEST_DC,
											psDisplayContext);
	if (eError != PVRSRV_OK)
	{
		goto FailRegisterDbgRequest;
	}

	*ppsDisplayContext = psDisplayContext;

	return PVRSRV_OK;

FailRegisterDbgRequest:
	PVRSRVUnregisterCmdCompleteNotify(psDisplayContext->hCmdCompNotify);
FailRegisterCmdComplete:
	OSUninstallMISR(psDisplayContext->hMISR);
FailMISR:
	_DCDeviceReleaseRef(psDevice);
FailDCDeviceContext:
	SCPDestroy(psDisplayContext->psSCPContext);
FailSCP:
	OSLockDestroy(psDisplayContext->hLock);
FailLock:
	OSFreeMem(psDisplayContext);
	return eError;
}

PVRSRV_ERROR DCDisplayContextConfigureCheck(DC_DISPLAY_CONTEXT *psDisplayContext,
											IMG_UINT32 ui32PipeCount,
											PVRSRV_SURFACE_CONFIG_INFO *pasSurfAttrib,
											DC_BUFFER **papsBuffers)
{
	DC_DEVICE *psDevice = psDisplayContext->psDevice;
	PVRSRV_ERROR eError;
	IMG_HANDLE *ahBuffers;
	
	_DCDisplayContextAcquireRef(psDisplayContext);

	/* Create an array of private device specific buffer handles */
	eError = _DCDeviceBufferArrayCreate(ui32PipeCount,
										papsBuffers,
										&ahBuffers);
	if (eError != PVRSRV_OK)
	{
		goto FailBufferArrayCreate;
	}

	/* Do we need to check if this is valid config? */
	if (psDevice->psFuncTable->pfnContextConfigureCheck)
	{

		eError = psDevice->psFuncTable->pfnContextConfigureCheck(psDisplayContext->hDisplayContext,
																ui32PipeCount,
																pasSurfAttrib,
																ahBuffers);
		if (eError != PVRSRV_OK)
		{
			goto FailConfigCheck;
		}
	}

	_DCDeviceBufferArrayDestroy(ahBuffers);
	_DCDisplayContextReleaseRef(psDisplayContext);
	return PVRSRV_OK;

FailConfigCheck:
	_DCDeviceBufferArrayDestroy(ahBuffers);
FailBufferArrayCreate:
	_DCDisplayContextReleaseRef(psDisplayContext);

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR DCDisplayContextConfigure(DC_DISPLAY_CONTEXT *psDisplayContext,
									   IMG_UINT32 ui32PipeCount,
									   PVRSRV_SURFACE_CONFIG_INFO *pasSurfAttrib,
									   DC_BUFFER **papsBuffers,
									   IMG_UINT32 ui32SyncOpCount,
									   SERVER_SYNC_PRIMITIVE **papsSync,
									   IMG_BOOL *pabUpdate,
									   IMG_UINT32 ui32DisplayPeriod,
									   IMG_UINT32 ui32MaxDepth,
									   IMG_INT32 i32AcquireFenceFd,
									   IMG_INT32 *pi32ReleaseFenceFd)
{
	DC_DEVICE *psDevice = psDisplayContext->psDevice;
	PVRSRV_ERROR eError;
	IMG_HANDLE *ahBuffers;
	IMG_UINT32 ui32BuffersMapped = 0;
	IMG_UINT32 i;
	IMG_UINT32 ui32CmdRdySize;
	IMG_UINT32 ui32CmdCompSize;
	IMG_UINT32 ui32CopySize;
	IMG_PVOID pvReadyData;
	IMG_PVOID pvCompleteData;
	DC_CMD_RDY_DATA *psReadyData;
	DC_CMD_COMP_DATA *psCompleteData;
	PVRSRV_DATA *psData = PVRSRVGetPVRSRVData();

	_DCDisplayContextAcquireRef(psDisplayContext);

	if (ui32MaxDepth == 1)
	{
		eError = PVRSRV_ERROR_DC_INVALID_MAXDEPTH;
		goto FailMaxDepth;
	}
	else if (ui32MaxDepth > 0)
	{
		/* ui32TokenOut/In wrap-around case takes care of itself. */
		if (psDisplayContext->ui32TokenOut - psDisplayContext->ui32TokenIn >= ui32MaxDepth)
		{
			eError = PVRSRV_ERROR_RETRY;
			goto FailMaxDepth;
		}
	}

	/* Reset the release fd */
	if (pi32ReleaseFenceFd)
		*pi32ReleaseFenceFd = -1;

	/* If we get sent a NULL flip then we don't need to do the check or map */
	if (ui32PipeCount != 0)
	{
		/* Create an array of private device specific buffer handles */
		eError = _DCDeviceBufferArrayCreate(ui32PipeCount,
											papsBuffers,
											&ahBuffers);
		if (eError != PVRSRV_OK)
		{
			goto FailBufferArrayCreate;
		}
	
		/* Do we need to check if this is valid config? */
		if (psDevice->psFuncTable->pfnContextConfigureCheck)
		{
	
			eError = psDevice->psFuncTable->pfnContextConfigureCheck(psDisplayContext->hDisplayContext,
																	ui32PipeCount,
																	pasSurfAttrib,
																	ahBuffers);
			if (eError != PVRSRV_OK)
			{
				goto FailConfigCheck;
			}
		}
	
		/* Map all the buffers that are going to be used */
		for (i=0;i<ui32PipeCount;i++)
		{
			eError = _DCBufferMap(papsBuffers[i]);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "DCDisplayContextConfigure: Failed to map buffer"));
				goto FailMapBuffer;
			}
			ui32BuffersMapped++;
		}
	}

	ui32CmdRdySize = sizeof(DC_CMD_RDY_DATA) +  
					 ((sizeof(IMG_HANDLE) + sizeof(PVRSRV_SURFACE_CONFIG_INFO))
					 * ui32PipeCount);
	ui32CmdCompSize = sizeof(DC_CMD_COMP_DATA) + 
					  (sizeof(DC_BUFFER *) * ui32PipeCount);

	/* Allocate a command */
	eError = SCPAllocCommand(psDisplayContext->psSCPContext,
							 ui32SyncOpCount,
							 papsSync,
							 pabUpdate,
							 i32AcquireFenceFd,
							 _DCDisplayContextReady,
							 _DCDisplayContextConfigure,
							 ui32CmdRdySize,
							 ui32CmdCompSize,
							 &pvReadyData,
							 &pvCompleteData,
							 pi32ReleaseFenceFd);

	if (eError != PVRSRV_OK)
	{
		goto FailCommandAlloc;
	}

	/*
		Set up command ready data
	*/
	psReadyData = pvReadyData;
	pvReadyData += sizeof(DC_CMD_RDY_DATA);

	psReadyData->ui32DisplayPeriod = ui32DisplayPeriod;
	psReadyData->psDisplayContext = psDisplayContext;
	psReadyData->ui32BufferCount = ui32PipeCount;

	/* Copy over surface atrribute array */
	if (ui32PipeCount != 0)
	{
		psReadyData->pasSurfAttrib = pvReadyData;
		ui32CopySize = sizeof(PVRSRV_SURFACE_CONFIG_INFO) * ui32PipeCount;
		OSMemCopy(psReadyData->pasSurfAttrib, pasSurfAttrib, ui32CopySize);
		pvReadyData += ui32CopySize;
	}
	else
	{
		psReadyData->pasSurfAttrib = IMG_NULL;
	}

	/* Copy over device buffer handle buffer array */
	if (ui32PipeCount != 0)
	{
		psReadyData->pahBuffer = pvReadyData;
		ui32CopySize = sizeof(IMG_HANDLE) * ui32PipeCount;
		OSMemCopy(psReadyData->pahBuffer, ahBuffers, ui32CopySize);
	}
	else
	{
		psReadyData->pahBuffer = IMG_NULL;
	}

	/*
		Set up command complete data
	*/
	psCompleteData = pvCompleteData;
	pvCompleteData += sizeof(DC_CMD_COMP_DATA);

	psCompleteData->psDisplayContext = psDisplayContext;
	psCompleteData->ui32Token = psDisplayContext->ui32TokenOut++;
	psCompleteData->ui32BufferCount = ui32PipeCount;

	if (ui32PipeCount != 0)
	{
		/* Copy the buffer pointers */
		psCompleteData->apsBuffer = pvCompleteData;
		for (i=0;i<ui32PipeCount;i++)
		{
			psCompleteData->apsBuffer[i] = papsBuffers[i];
		}
	}

	/* Check if we need to do any CPU cache operations before sending the config */
	OSCPUOperation(psData->uiCacheOp);
	psData->uiCacheOp = PVRSRV_CACHE_OP_NONE;

	/* Submit the command */
	eError = SCPSubmitCommand(psDisplayContext->psSCPContext);

	/* Check for new work on this display context */
	_DCDisplayContextRun(psDisplayContext);

	/* The only way this submit can fail is if there is a bug in this module */
	PVR_ASSERT(eError == PVRSRV_OK);

	if (ui32PipeCount != 0)
	{
		_DCDeviceBufferArrayDestroy(ahBuffers);
	}

	return PVRSRV_OK;

FailCommandAlloc:
FailMapBuffer:
	if (ui32PipeCount != 0)
	{
		for (i=0;i<ui32BuffersMapped;i++)
		{
			_DCBufferUnmap(papsBuffers[i]);
		}
	}
FailConfigCheck:
	if (ui32PipeCount != 0)
	{
		_DCDeviceBufferArrayDestroy(ahBuffers);
	}
FailBufferArrayCreate:
FailMaxDepth:
	_DCDisplayContextReleaseRef(psDisplayContext);

	return eError;
}

PVRSRV_ERROR DCDisplayContextDestroy(DC_DISPLAY_CONTEXT *psDisplayContext)
{
	PVRSRV_ERROR eError;

	/*
		On the first cleanup request try to issue the NULL flip.
		If we fail then we should get retry which we pass back to
		the caller who will try again later.
	*/
	if (!psDisplayContext->bIssuedNullFlip)
	{
		eError = DCDisplayContextConfigure(psDisplayContext,
										   0,
										   IMG_NULL,
										   IMG_NULL,
										   0,
										   IMG_NULL,
										   IMG_NULL,
										   0,
										   0,
										   -1,
										   IMG_NULL);

		if (eError != PVRSRV_OK)
		{
			return eError;
		}
		psDisplayContext->bIssuedNullFlip = IMG_TRUE;
	}

	/*
		Flush out everything from SCP
		
		This will ensure that the MISR isn't dropping the last reference
		which would cause a deadlock during cleanup
	*/
	eError = SCPFlush(psDisplayContext->psSCPContext);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	_DCDisplayContextReleaseRef(psDisplayContext);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCBufferAlloc(DC_DISPLAY_CONTEXT *psDisplayContext,
						   DC_BUFFER_CREATE_INFO *psSurfInfo,
						   IMG_UINT32 *pui32ByteStride,
						   DC_BUFFER **ppsBuffer)
{
	DC_DEVICE *psDevice = psDisplayContext->psDevice;
	DC_BUFFER *psNew;
	PMR *psPMR;
	PVRSRV_ERROR eError;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2PageSize;
	IMG_UINT32 ui32PageCount;
	IMG_UINT32 ui32PhysHeapID;

	psNew = OSAllocMem(sizeof(DC_BUFFER));
	if (psNew == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	OSMemSet(psNew, 0, sizeof(DC_BUFFER));

	eError = OSLockCreate(&psNew->hLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		goto fail_lock;
	}

	eError = OSLockCreate(&psNew->hMapLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		goto fail_maplock;
	}

	eError = psDevice->psFuncTable->pfnBufferAlloc(psDisplayContext->hDisplayContext,
												  psSurfInfo,
												  &uiLog2PageSize,
												  &ui32PageCount,
												  &ui32PhysHeapID,
												  pui32ByteStride,
												  &psNew->hBuffer);
	if (eError != PVRSRV_OK)
	{
		goto fail_bufferalloc;
	}

	/*
		Fill in the basic info for our buffer
		(must be before _DCCreatePMR)
	*/
	psNew->psDisplayContext = psDisplayContext;
	psNew->eType = DC_BUFFER_TYPE_ALLOC;
	psNew->ui32MapCount = 0;
	psNew->ui32RefCount = 1;

	eError = _DCCreatePMR(uiLog2PageSize,
						  ui32PageCount,
						  ui32PhysHeapID,
						  psNew,
						  &psPMR);
	if (eError != PVRSRV_OK)
	{
		goto fail_createpmr;
	}

	psNew->uBufferData.sAllocData.psPMR = psPMR;
	_DCDisplayContextAcquireRef(psDisplayContext);

	*ppsBuffer = psNew;

	return PVRSRV_OK;

fail_createpmr:
	psDevice->psFuncTable->pfnBufferFree(psNew->hBuffer);
fail_bufferalloc:
	OSLockDestroy(psNew->hMapLock);
fail_maplock:
	OSLockDestroy(psNew->hLock);
fail_lock:
	OSFreeMem(psNew);
	return eError;
}

PVRSRV_ERROR DCBufferFree(DC_BUFFER *psBuffer)
{
	/*
		Only drop the reference on the PMR if this is a DC allocated
		buffer. In the case of imported buffers the 3rd party DC
		driver manages the PMR's "directly"
	*/
	if (psBuffer->eType == DC_BUFFER_TYPE_ALLOC)
	{
		PMRUnrefPMR(psBuffer->uBufferData.sAllocData.psPMR);
	}
	_DCBufferReleaseRef(psBuffer);

	return PVRSRV_OK;
}

PVRSRV_ERROR DCBufferImport(DC_DISPLAY_CONTEXT *psDisplayContext,
							IMG_UINT32 ui32NumPlanes,
							PMR **papsImport,
						    DC_BUFFER_IMPORT_INFO *psSurfAttrib,
						    DC_BUFFER **ppsBuffer)
{
	DC_DEVICE *psDevice = psDisplayContext->psDevice;
	DC_BUFFER *psNew;
	PVRSRV_ERROR eError;
	IMG_UINT32 i;

	if(psDevice->psFuncTable->pfnBufferImport == IMG_NULL)
	{
		eError = PVRSRV_ERROR_NOT_SUPPORTED;
		goto FailEarlyError;
	}

	psNew = OSAllocMem(sizeof(DC_BUFFER));
	if (psNew == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto FailEarlyError;
	}
	OSMemSet(psNew, 0, sizeof(DC_BUFFER));

	eError = OSLockCreate(&psNew->hLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		goto FailLock;
	}

	eError = OSLockCreate(&psNew->hMapLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		goto FailMapLock;
	}

	eError = psDevice->psFuncTable->pfnBufferImport(psDisplayContext->hDisplayContext,
													ui32NumPlanes,
													(IMG_HANDLE **)papsImport,
													psSurfAttrib,
													&psNew->hBuffer);
	if (eError != PVRSRV_OK)
	{
		goto FailBufferImport;
	}

	/*
		Take a reference on the PMR to make sure it can't be released before
		we've finished with it
	*/
	for (i=0;i<ui32NumPlanes;i++)
	{
		PMRRefPMR(papsImport[i]);
		psNew->uBufferData.sImportData.apsImport[i] = papsImport[i];
	}

	_DCDisplayContextAcquireRef(psDisplayContext);
	psNew->psDisplayContext = psDisplayContext;
	psNew->eType = DC_BUFFER_TYPE_IMPORT;
	psNew->uBufferData.sImportData.ui32NumPlanes = ui32NumPlanes;
	psNew->ui32MapCount = 0;
	psNew->ui32RefCount = 1;

	*ppsBuffer = psNew;

	return PVRSRV_OK;

FailBufferImport:
	OSLockDestroy(psNew->hMapLock);
FailMapLock:
	OSLockDestroy(psNew->hLock);
FailLock:
	OSFreeMem(psNew);

FailEarlyError:
	return eError;
}

PVRSRV_ERROR DCBufferUnimport(DC_BUFFER *psBuffer)
{
	_DCBufferReleaseRef(psBuffer);
	return PVRSRV_OK;
}


PVRSRV_ERROR DCBufferAcquire(DC_BUFFER *psBuffer, PMR **ppsPMR)
{
	PMR *psPMR = psBuffer->uBufferData.sAllocData.psPMR;
	PVRSRV_ERROR eError;

	if (psBuffer->eType == DC_BUFFER_TYPE_IMPORT)
	{
		PVR_DPF((PVR_DBG_ERROR, "DCBufferAcquire: Invalid request, DC buffer is an import"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto fail_typecheck;
	}
	PMRRefPMR(psPMR);

	*ppsPMR = psPMR;
	return PVRSRV_OK;
	
fail_typecheck:
	return eError;
}

PVRSRV_ERROR DCBufferRelease(PMR *psPMR)
{
	/*
		Drop our reference on the PMR. If we're the last one then the PMR
		will be freed and are _DCPMRFinalize function will be called where
		we drop our reference on the buffer
	*/
	PMRUnrefPMR(psPMR);
	return PVRSRV_OK;
}

PVRSRV_ERROR DCBufferPin(DC_BUFFER *psBuffer, DC_PIN_HANDLE *phPin)
{
	*phPin = psBuffer;
	return _DCBufferMap(psBuffer);
}

PVRSRV_ERROR DCBufferUnpin(DC_PIN_HANDLE hPin)
{
	DC_BUFFER *psBuffer = hPin;

	_DCBufferUnmap(psBuffer);
	return PVRSRV_OK;
}

/*****************************************************************************
 *     Public interface functions for 3rd party display class devices        *
 *****************************************************************************/

PVRSRV_ERROR DCRegisterDevice(DC_DEVICE_FUNCTIONS *psFuncTable,
							  IMG_UINT32 ui32MaxConfigsInFlight,
							  IMG_HANDLE hDeviceData,
							  IMG_HANDLE *phSrvHandle)
{
	DC_DEVICE *psNew;
	PVRSRV_ERROR eError;

	psNew = OSAllocMem(sizeof(DC_DEVICE));
	if (psNew == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	eError = OSLockCreate(&psNew->hLock, LOCK_TYPE_NONE);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	psNew->psFuncTable = psFuncTable;
	psNew->ui32MaxConfigsInFlight = ui32MaxConfigsInFlight;
	psNew->hDeviceData = hDeviceData;
	psNew->ui32RefCount = 1;
	psNew->hSystemBuffer = IMG_NULL;
	psNew->ui32Index = g_ui32DCNextIndex++;
	eError = OSEventObjectCreate(IMG_NULL, &psNew->psEventList);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	/* Init state required for system surface */
	psNew->hSystemBuffer = IMG_NULL;
	psNew->psSystemBufferPMR = IMG_NULL;
	psNew->sSystemContext.psDevice = psNew;
	psNew->sSystemContext.hDisplayContext = hDeviceData;	/* FIXME: Is this the correct thing to do? */

	OSLockAcquire(g_hDCListLock);
	psNew->psNext = g_psDCDeviceList;
	
	g_psDCDeviceList = psNew;
	g_ui32DCDeviceCount++;
	OSLockRelease(g_hDCListLock);

	*phSrvHandle = (IMG_HANDLE) psNew;

	return PVRSRV_OK;
}

IMG_VOID DCUnregisterDevice(IMG_HANDLE hSrvHandle)
{
	DC_DEVICE *psDevice = (DC_DEVICE *) hSrvHandle;
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_ERROR eError;

	/*
		If the system buffer was acquired and a PMR created for it, release
		it before releasing the device as the PMR will have a reference to
		the device
	*/
	if (psDevice->psSystemBufferPMR)
	{
		PMRUnrefPMR(psDevice->psSystemBufferPMR);
	}

	/*
	 * At this stage the DC driver wants to unload, if other things have
	 * reference to the DC device we need to block here until they have
	 * been release as when this function returns the DC driver code could
	 * be unloaded.
	 */

	/* If the driver is in a bad state we just free resources regardless */
	if (psPVRSRVData->eServicesState == PVRSRV_SERVICES_STATE_OK)
	{
		/* Skip the wait if we're the last reference holder */
		if (psDevice->ui32RefCount != 1)
		{
			IMG_HANDLE hEvent;
			
			eError = OSEventObjectOpen(psDevice->psEventList, &hEvent);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,
						 "%s: Failed to open event object (%d), will busy wait",
						 __FUNCTION__, eError));
				hEvent = IMG_NULL;
			}
			
			while(psDevice->ui32RefCount != 1)
			{
				if (hEvent != IMG_NULL)
				{
					OSEventObjectWait(hEvent);
				}
			}
			if (hEvent != IMG_NULL)
			{
				OSEventObjectClose(hEvent);
			}
		}
	}
	else
	{
		/* We're in a bad state, force the refcount */
		psDevice->ui32RefCount = 1;
	}

	_DCDeviceReleaseRef(psDevice);

	PVR_ASSERT(psDevice->ui32RefCount == 0);
	OSEventObjectDestroy(psDevice->psEventList);
	OSLockDestroy(psDevice->hLock);
	OSFreeMem(psDevice);
}

IMG_VOID DCDisplayConfigurationRetired(IMG_HANDLE hConfigData)
{
	DC_CMD_COMP_DATA *psData = hConfigData;
	DC_DISPLAY_CONTEXT *psDisplayContext = psData->psDisplayContext;
	IMG_UINT32 i;

	DC_DEBUG_PRINT("DCDisplayConfigurationRetired: Command (%d) received", psData->ui32Token);
	/* Sanity check */
	if (psData->ui32Token != psDisplayContext->ui32TokenIn)
	{
		PVR_DPF((PVR_DBG_ERROR,
				"Display config retired in unexpected order (was %d, expecting %d)",
				psData->ui32Token, psDisplayContext->ui32TokenIn));
		PVR_ASSERT(IMG_FALSE);
	}

	OSLockAcquire(psDisplayContext->hLock);
	psDisplayContext->ui32TokenIn++;

	psDisplayContext->ui32ConfigsInFlight--;
	OSLockRelease(psDisplayContext->hLock);

	for (i = 0; i < psData->ui32BufferCount; i++)
	{
		_DCBufferUnmap(psData->apsBuffer[i]);
	}

	_DCDisplayContextReleaseRef(psDisplayContext);

	/*
		Note:

		We must call SCPCommandComplete here and not before as we need
		to ensure that we're not the last to hold the reference as
		we can't destroy the display context from the MISR which we
		can be called from.
	*/
	SCPCommandComplete(psDisplayContext->psSCPContext);

	/* Notify devices (including ourself) in case some item has been unblocked */
	PVRSRVCheckStatus(IMG_NULL);
}

PVRSRV_ERROR DCImportBufferAcquire(IMG_HANDLE hImport,
								   IMG_DEVMEM_LOG2ALIGN_T uiLog2PageSize,
								   IMG_UINT32 *pui32PageCount,
								   IMG_DEV_PHYADDR **ppasDevPAddr)
{
	PMR *psPMR = hImport;
	IMG_DEV_PHYADDR *pasDevPAddr;
	IMG_DEVMEM_SIZE_T uiLogicalSize;
	IMG_UINT32 ui32PageCount;
	IMG_DEVMEM_OFFSET_T uiOffset = 0;
	IMG_UINT32 i;
	PVRSRV_ERROR eError;

	eError = PMR_LogicalSize(psPMR, &uiLogicalSize);
	if (eError != PVRSRV_OK)
	{
		goto fail_getsize;
	}

	ui32PageCount = uiLogicalSize >> uiLog2PageSize;

	pasDevPAddr = OSAllocMem(sizeof(IMG_DEV_PHYADDR) * ui32PageCount);
	if (pasDevPAddr == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_alloc;
	}

	OSMemSet(pasDevPAddr, 0, sizeof(IMG_DEV_PHYADDR) * ui32PageCount);

	/* Lock the pages */
	eError = PMRLockSysPhysAddresses(psPMR, uiLog2PageSize);
	if (eError != PVRSRV_OK)
	{
		goto fail_lock;
	}

	/* Get each page one by one */
	for (i=0;i<ui32PageCount;i++)
	{
		IMG_BOOL bValid;

		eError = PMR_DevPhysAddr(psPMR, uiOffset, &pasDevPAddr[i], &bValid);
		if (eError != PVRSRV_OK)
		{
			goto fail_getphysaddr;
		}

		/* The DC import function doesn't support sparse allocations */
		PVR_ASSERT(bValid);
		uiOffset += 1 << uiLog2PageSize;
	}

	*pui32PageCount = ui32PageCount;
	*ppasDevPAddr = pasDevPAddr;
	return PVRSRV_OK;

fail_getphysaddr:
	PMRUnlockSysPhysAddresses(psPMR);
fail_lock:
	OSFreeMem(pasDevPAddr);
fail_alloc:
fail_getsize:
	return eError;
}

IMG_VOID DCImportBufferRelease(IMG_HANDLE hImport,
							   IMG_DEV_PHYADDR *pasDevPAddr)
{
	PMR *psPMR = hImport;

	/* Unlock the pages */
	PMRUnlockSysPhysAddresses(psPMR);
	OSFreeMem(pasDevPAddr);
}

/*****************************************************************************
 *                Public interface functions for services                    *
 *****************************************************************************/
PVRSRV_ERROR DCInit()
{
	g_psDCDeviceList = IMG_NULL;
	g_ui32DCNextIndex = 0;
	return OSLockCreate(&g_hDCListLock, LOCK_TYPE_NONE);
}

PVRSRV_ERROR DCDeInit()
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	if (psPVRSRVData->eServicesState == PVRSRV_SERVICES_STATE_OK)
	{
		/* FIXME: Should this be a blocking call? */
		PVR_ASSERT(g_psDCDeviceList == IMG_NULL);
	}

	OSLockDestroy(g_hDCListLock);

	return PVRSRV_OK;
}
