/*************************************************************************/ /*!
@File
@Title          Device Memory Management internal utility functions
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Utility functions used internally by device memory management
                code.
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

#include "allocmem.h"
#include "img_types.h"
#include "pvrsrv_error.h"
#include "ra.h"
#include "devicemem_utils.h"
#include "client_mm_bridge.h"

IMG_INTERNAL
PVRSRV_ERROR _DevmemValidateParams(IMG_DEVMEM_SIZE_T uiSize,
								   IMG_DEVMEM_ALIGN_T uiAlign,
								   DEVMEM_FLAGS_T uiFlags)
{
    if (!(uiFlags & PVRSRV_MEMALLOCFLAG_GPU_READABLE))
    {
        /* Don't support memory not GPU readable currently */
        return PVRSRV_ERROR_INVALID_PARAMS;
    }

    if ((uiFlags & PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC) &&
        (uiFlags & PVRSRV_MEMALLOCFLAG_POISON_ON_ALLOC))
    {
        /* Zero on Alloc and Poison on Alloc are mutually exclusive */
        return PVRSRV_ERROR_INVALID_PARAMS;
    }

    if (uiAlign & (uiAlign-1))
    {
        return PVRSRV_ERROR_INVALID_PARAMS;
    }

    /* Verify that size is a positive integer multiple of alignment */
#if 0 // FIXME
    if (uiSize & (uiAlign-1))
    {
        /* Size not a multiple of alignment */
        return PVRSRV_ERROR_INVALID_PARAMS;
    }
#endif
    if (uiSize == 0)
    {
        return PVRSRV_ERROR_INVALID_PARAMS;
    }

    return PVRSRV_OK;
}

/*
	Allocate and init an import structure
*/
IMG_INTERNAL
PVRSRV_ERROR _DevmemImportStructAlloc(IMG_HANDLE hBridge,
									  IMG_BOOL bExportable,
									  DEVMEM_IMPORT **ppsImport)
{
	DEVMEM_IMPORT *psImport;
	PVRSRV_ERROR eError;

    psImport = OSAllocMem(sizeof *psImport);
    if (psImport == IMG_NULL)
    {
        return PVRSRV_ERROR_OUT_OF_MEMORY;
    }

	/* Setup some known bad values for things we don't have yet */
	psImport->sDeviceImport.hReservation = LACK_OF_RESERVATION_POISON;
    psImport->sDeviceImport.hMapping = LACK_OF_MAPPING_POISON;
    psImport->sDeviceImport.psHeap = IMG_NULL;
    psImport->sDeviceImport.bMapped = IMG_FALSE;

	eError = OSLockCreate(&psImport->sDeviceImport.hLock, LOCK_TYPE_PASSIVE);
	if (eError != PVRSRV_OK)
	{
		goto failDIOSLockCreate;
	}

	psImport->sCPUImport.hOSMMapData = IMG_NULL;
	psImport->sCPUImport.pvCPUVAddr = IMG_NULL;

	eError = OSLockCreate(&psImport->sCPUImport.hLock, LOCK_TYPE_PASSIVE);
	if (eError != PVRSRV_OK)
	{
		goto failCIOSLockCreate;
	}

	/* Set up common elements */
    psImport->hBridge = hBridge;
    psImport->bExportable = bExportable;

	/* Setup refcounts */
    psImport->sDeviceImport.ui32RefCount = 0;
    psImport->sCPUImport.ui32RefCount = 0;
    psImport->ui32RefCount = 0;

	/* Create the lock */
	eError = OSLockCreate(&psImport->hLock, LOCK_TYPE_PASSIVE);
	if (eError != PVRSRV_OK)
	{
		goto failILockAlloc;
	}

    *ppsImport = psImport;
    
    return PVRSRV_OK;

failILockAlloc:
	OSLockDestroy(psImport->sCPUImport.hLock);
failCIOSLockCreate:
	OSLockDestroy(psImport->sDeviceImport.hLock);
failDIOSLockCreate:
	OSFreeMem(psImport);
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

/*
	Initialise the import structure
*/
IMG_INTERNAL
IMG_VOID _DevmemImportStructInit(DEVMEM_IMPORT *psImport,
								 IMG_DEVMEM_SIZE_T uiSize,
								 IMG_DEVMEM_ALIGN_T uiAlign,
								 DEVMEM_FLAGS_T uiFlags,
								 IMG_HANDLE hPMR)
{
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
					__FUNCTION__,
					psImport,
					0,
					1);

	psImport->uiSize = uiSize;
	psImport->uiAlign = uiAlign;
	psImport->uiFlags = uiFlags;
	psImport->hPMR = hPMR;
	psImport->ui32RefCount = 1;
}

/*
	Map an import to the device
*/
IMG_INTERNAL
PVRSRV_ERROR _DevmemImportStructDevMap(DEVMEM_HEAP *psHeap,
									   IMG_BOOL bMap,
									   DEVMEM_IMPORT *psImport)
{
	DEVMEM_DEVICE_IMPORT *psDeviceImport;
	IMG_BOOL bStatus;
    RA_BASE_T uiAllocatedAddr;
    RA_LENGTH_T uiAllocatedSize;
    IMG_DEV_VIRTADDR sBase;
    IMG_HANDLE hReservation;
    PVRSRV_ERROR eError;

	psDeviceImport = &psImport->sDeviceImport;

	OSLockAcquire(psDeviceImport->hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
					__FUNCTION__,
					psImport,
					psDeviceImport->ui32RefCount,
					psDeviceImport->ui32RefCount+1);

	if (psDeviceImport->ui32RefCount++ == 0)
	{
		_DevmemImportStructAcquire(psImport);

		OSLockAcquire(psHeap->hLock);
		psHeap->uiImportCount++;
		OSLockRelease(psHeap->hLock);

		if (psHeap->psCtx->hBridge != psImport->hBridge)
		{
			/*
				The import was done with a different connection then the
				memory context which means they are not compatible.
			*/
			eError = PVRSRV_ERROR_INVALID_PARAMS;
			goto failCheck;
		}

		/* Allocate space in the VM */
	    bStatus = RA_Alloc(psHeap->psQuantizedVMRA,
	                       psImport->uiSize,
	                       0, /* flags: this RA doesn't use flags*/
	                       psImport->uiAlign,
	                       &uiAllocatedAddr,
	                       &uiAllocatedSize,
	                       IMG_NULL /* don't care about per-import priv data */
	                       );
	    if (!bStatus)
	    {
	        eError = PVRSRV_ERROR_DEVICEMEM_OUT_OF_DEVICE_VM;
	        goto failVMRAAlloc;
	    }
	
	    /* No reason for the allocated virtual size to be different from
	       the PMR's size */
	    PVR_ASSERT(uiAllocatedSize == psImport->uiSize);
	
	    sBase.uiAddr = uiAllocatedAddr;
	
		/* Setup page tables for the allocated VM space */
	    eError = BridgeDevmemIntReserveRange(psHeap->psCtx->hBridge,
											 psHeap->hDevMemServerHeap,
											 sBase,
											 uiAllocatedSize,
											 &hReservation);
	    if (eError != PVRSRV_OK)
	    {
	        goto failReserve;
	    }

		if (bMap)
		{
			DEVMEM_FLAGS_T uiMapFlags;
			
			uiMapFlags = psImport->uiFlags & PVRSRV_MEMALLOCFLAGS_PERMAPPINGFLAGSMASK;

			/* Actually map the PMR to allocated VM space */
			eError = BridgeDevmemIntMapPMR(psHeap->psCtx->hBridge,
										   psHeap->hDevMemServerHeap,
										   hReservation,
										   psImport->hPMR,
										   uiMapFlags,
										   &psDeviceImport->hMapping);
			if (eError != PVRSRV_OK)
			{
				goto failMap;
			}
			psDeviceImport->bMapped = IMG_TRUE;
		}

		/* Setup device mapping specific parts of the mapping info */
	    psDeviceImport->hReservation = hReservation;
		psDeviceImport->sDevVAddr.uiAddr = uiAllocatedAddr;
		psDeviceImport->psHeap = psHeap;
	}
	else
	{
		/*
			Check that we've been asked to map it into the
			same heap 2nd time around
		*/
		if (psHeap != psDeviceImport->psHeap)
		{
			eError = PVRSRV_ERROR_INVALID_HEAP;
			goto failParams;
		}
	}
	OSLockRelease(psDeviceImport->hLock);

	return PVRSRV_OK;

failMap:
	BridgeDevmemIntUnreserveRange(psHeap->psCtx->hBridge,
								  hReservation);
failReserve:
	RA_Free(psHeap->psQuantizedVMRA,
            uiAllocatedAddr);
failVMRAAlloc:
failCheck:
	_DevmemImportStructRelease(psImport);
	psHeap->uiImportCount--;
failParams:
	OSLockRelease(psDeviceImport->hLock);
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*
	Unmap an import from the Device
*/
IMG_INTERNAL
IMG_VOID _DevmemImportStructDevUnmap(DEVMEM_IMPORT *psImport)
{
	PVRSRV_ERROR eError;
	DEVMEM_DEVICE_IMPORT *psDeviceImport;

	psDeviceImport = &psImport->sDeviceImport;

	OSLockAcquire(psDeviceImport->hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
					__FUNCTION__,
					psImport,
					psDeviceImport->ui32RefCount,
					psDeviceImport->ui32RefCount-1);

	if (--psDeviceImport->ui32RefCount == 0)
	{
		DEVMEM_HEAP *psHeap = psDeviceImport->psHeap;

		OSLockRelease(psDeviceImport->hLock);

		if (psDeviceImport->bMapped)
		{
			eError = BridgeDevmemIntUnmapPMR(psImport->hBridge,
											psDeviceImport->hMapping);
			PVR_ASSERT(eError == PVRSRV_OK);
		}
	
	    eError = BridgeDevmemIntUnreserveRange(psImport->hBridge,
	                                        psDeviceImport->hReservation);
	    PVR_ASSERT(eError == PVRSRV_OK);
	
	    RA_Free(psHeap->psQuantizedVMRA,
	            psDeviceImport->sDevVAddr.uiAddr);

		_DevmemImportStructRelease(psImport);

		OSLockAcquire(psHeap->hLock);
		psHeap->uiImportCount--;
		OSLockRelease(psHeap->hLock);
	}
	else
	{
		OSLockRelease(psDeviceImport->hLock);
	}
}

/*
	Map an import into the CPU
*/
IMG_INTERNAL
PVRSRV_ERROR _DevmemImportStructCPUMap(DEVMEM_IMPORT *psImport)
{
	PVRSRV_ERROR eError;
	DEVMEM_CPU_IMPORT *psCPUImport;
	IMG_SIZE_T uiMappingLength;

	psCPUImport = &psImport->sCPUImport;

	OSLockAcquire(psCPUImport->hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
					__FUNCTION__,
					psImport,
					psCPUImport->ui32RefCount,
					psCPUImport->ui32RefCount+1);

	if (psCPUImport->ui32RefCount++ == 0)
	{
		_DevmemImportStructAcquire(psImport);
		eError = OSMMapPMR(psImport->hBridge,
						   psImport->hPMR,
						   psImport->uiSize,
						   &psCPUImport->hOSMMapData,
						   &psCPUImport->pvCPUVAddr,
						   &uiMappingLength);
		if (eError != PVRSRV_OK)
		{
			goto failMap;
		}

		/* There is no reason the mapping length is different to the size */
		PVR_ASSERT(uiMappingLength == psImport->uiSize);
	}
	OSLockRelease(psCPUImport->hLock);

	return PVRSRV_OK;

failMap:
	psCPUImport->ui32RefCount--;
	_DevmemImportStructRelease(psImport);
	OSLockRelease(psCPUImport->hLock);
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*
	Unmap an import from the CPU
*/
IMG_INTERNAL
IMG_VOID _DevmemImportStructCPUUnmap(DEVMEM_IMPORT *psImport)
{
	DEVMEM_CPU_IMPORT *psCPUImport;

	psCPUImport = &psImport->sCPUImport;

	OSLockAcquire(psCPUImport->hLock);
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
					__FUNCTION__,
					psImport,
					psCPUImport->ui32RefCount,
					psCPUImport->ui32RefCount-1);

	if (--psCPUImport->ui32RefCount == 0)
	{
		OSLockRelease(psCPUImport->hLock);
		OSMUnmapPMR(psImport->hBridge,
					psImport->hPMR,
					psCPUImport->hOSMMapData,
					psCPUImport->pvCPUVAddr,
					psImport->uiSize);
		_DevmemImportStructRelease(psImport);
	}
	else
	{
		OSLockRelease(psCPUImport->hLock);
	}
}


