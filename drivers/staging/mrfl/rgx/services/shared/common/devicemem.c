/******************************************************************************
 * Name         : devicemem.c
 * Title        : Device Memory Management
 * Author(s)    : Imagination Technologies
 * Created      :
 *
 * Copyright    : 2010 by Imagination Technologies Limited.
 *                All rights reserved. No part of this software, either
 *                material or conceptual may be copied or distributed,
 *                transmitted, transcribed, stored in a retrieval system or
 *                translated into any human or computer language in any form
 *                by any means, electronic, mechanical, manual or otherwise,
 *                or disclosed to third parties without the express written
 *                permission of Imagination Technologies Limited,
 *                Home Park Estate, Kings Langley, Hertfordshire,
 *                WD4 8LZ, U.K.
 *
 * Description : Front End (nominally Client side part, but now invokable
 *               from server too) of device memory management
 *
 * Platform     : ALL
 *
 *****************************************************************************/

#include "devicemem.h"
#include "img_types.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"
#include "allocmem.h"
#include "ra.h"
#include "osfunc.h"
#include "devicemem_mmap.h"
#include "devicemem_utils.h"
#include "client_mm_bridge.h"

/* FIXME */
#if defined(__KERNEL__)
#include "pvrsrv.h"
#endif

/*****************************************************************************
 *                    Unsecure export internals                              *
 *****************************************************************************/

static PVRSRV_ERROR
_Mapping_Export(DEVMEM_IMPORT * psImport,
		DEVMEM_EXPORTHANDLE * phPMRExportHandlePtr,
		DEVMEM_EXPORTKEY * puiExportKeyPtr,
		DEVMEM_SIZE_T * puiSize, DEVMEM_LOG2ALIGN_T * puiLog2Contig)
{
	/* Gets an export handle and key for the PMR used for this mapping */
	/* Can only be done if there are no suballocations for this mapping */

	PVRSRV_ERROR eError;
	DEVMEM_EXPORTHANDLE hPMRExportHandle;
	DEVMEM_EXPORTKEY uiExportKey;
	IMG_DEVMEM_SIZE_T uiSize;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2Contig;

	if (psImport == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	if (!psImport->bExportable) {
		eError = PVRSRV_ERROR_DEVICEMEM_CANT_EXPORT_SUBALLOCATION;
		goto failParams;
	}

	eError = BridgePMRExportPMR(psImport->hBridge,
				    psImport->hPMR,
				    &hPMRExportHandle,
				    &uiSize, &uiLog2Contig, &uiExportKey);
	if (eError != PVRSRV_OK) {
		goto failExport;
	}

	PVR_ASSERT(uiSize == psImport->uiSize);

	*phPMRExportHandlePtr = hPMRExportHandle;
	*puiExportKeyPtr = uiExportKey;
	*puiSize = uiSize;
	*puiLog2Contig = uiLog2Contig;

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 failExport:
 failParams:

	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;

}

static IMG_VOID
_Mapping_Unexport(DEVMEM_IMPORT * psImport,
		  DEVMEM_EXPORTHANDLE hPMRExportHandle)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psImport != IMG_NULL);

	eError = BridgePMRUnexportPMR(psImport->hBridge, hPMRExportHandle);
	PVR_ASSERT(eError == PVRSRV_OK);
}

/*****************************************************************************
 *                    Sub allocation internals                               *
 *****************************************************************************/

static PVRSRV_ERROR
_AllocateDeviceMemory(IMG_HANDLE hBridge,
		      IMG_HANDLE hDeviceNode,
		      IMG_UINT32 uiLog2Quantum,
		      IMG_DEVMEM_SIZE_T uiSize,
		      IMG_DEVMEM_ALIGN_T uiAlign,
		      DEVMEM_FLAGS_T uiFlags,
		      IMG_BOOL bExportable, DEVMEM_IMPORT ** ppsImport)
{
	DEVMEM_IMPORT *psImport;
	DEVMEM_FLAGS_T uiPMRFlags;
	IMG_HANDLE hPMR;
	PVRSRV_ERROR eError;

	eError = _DevmemImportStructAlloc(hBridge, bExportable, &psImport);
	if (eError != PVRSRV_OK) {
		goto failAlloc;
	}

	/* Check the size is a multiple of the quantum */
	PVR_ASSERT((uiSize & ((1ULL << uiLog2Quantum) - 1)) == 0);

	/* Pass only the PMR flags down */
	uiPMRFlags = uiFlags & PVRSRV_MEMALLOCFLAGS_PMRFLAGSMASK;

	eError = BridgePhysmemNewRamBackedPMR(hBridge,
					      hDeviceNode,
					      uiSize,
					      uiLog2Quantum, uiPMRFlags, &hPMR);
	if (eError != PVRSRV_OK) {
		/* Our check above should have ensured this the "not page
		   multiple" error never happens */
		PVR_ASSERT(eError != PVRSRV_ERROR_PMR_NOT_PAGE_MULTIPLE);

		goto failPMR;
	}

	_DevmemImportStructInit(psImport, uiSize, uiAlign, uiFlags, hPMR);

	*ppsImport = psImport;
	return PVRSRV_OK;

 failPMR:
	_DevmemImportStructRelease(psImport);
 failAlloc:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

static IMG_VOID _FreeDeviceMemory(DEVMEM_IMPORT * psImport)
{
	_DevmemImportStructRelease(psImport);
}

static IMG_BOOL
_SubAllocImportAlloc(RA_PERARENA_HANDLE hArena,
		     RA_LENGTH_T uiSize, RA_FLAGS_T _flags,
		     /* returned data */
		     RA_BASE_T * puiBase,
		     RA_LENGTH_T * puiActualSize, RA_PERISPAN_HANDLE * phImport)
{
	/* When suballocations need a new lump of memory, the RA calls
	   back here.  Later, in the kernel, we must construct a new PMR
	   and a pairing between the new lump of virtual memory and the
	   PMR (whether or not such PMR is backed by physical memory) */
	DEVMEM_HEAP *psHeap;
	DEVMEM_IMPORT *psImport;
	IMG_DEVMEM_ALIGN_T uiAlign;
	DEVMEM_FLAGS_T uiFlags;
	PVRSRV_ERROR eError;

	uiFlags = (DEVMEM_FLAGS_T) _flags;

	/* Per-arena private handle is, for us, the heap */
	psHeap = hArena;

	/* align to the l.s.b. of the size...  e.g. 96kiB aligned to
	   32kiB. NB: There is an argument to say that the RA should never
	   ask us for Non-power-of-2 size anyway, but I don't want to make
	   that restriction arbitrarily now */
	uiAlign = uiSize & ~(uiSize - 1);

	/* The RA should not have invoked us with a size that is not a
	   multiple of the quantum anyway */
	PVR_ASSERT((uiSize & ((1ULL << psHeap->uiLog2Quantum) - 1)) == 0);

	eError = _AllocateDeviceMemory(psHeap->psCtx->hBridge,
				       psHeap->psCtx->hDeviceNode,
				       psHeap->uiLog2Quantum,
				       uiSize,
				       uiAlign, uiFlags, IMG_FALSE, &psImport);
	if (eError != PVRSRV_OK) {
		goto failAlloc;
	}

	/*
	   Suballocations always get mapped into the device was we need to
	   key the RA off something and as we can't export suballocations
	   there is no valid reason to request an allocation an not map it
	 */
	eError = _DevmemImportStructDevMap(psHeap, IMG_TRUE, psImport);
	if (eError != PVRSRV_OK) {
		goto failMap;
	}

	*puiBase = psImport->sDeviceImport.sDevVAddr.uiAddr;
	*puiActualSize = uiSize;
	*phImport = psImport;

	return IMG_TRUE;

	/*
	   error exit paths follow
	 */
 failMap:
	_FreeDeviceMemory(psImport);
 failAlloc:

	return IMG_FALSE;
}

static IMG_VOID
_SubAllocImportFree(RA_PERARENA_HANDLE hArena,
		    RA_BASE_T uiBase, RA_PERISPAN_HANDLE hImport)
{
	DEVMEM_IMPORT *psImport = hImport;

	PVR_ASSERT(psImport != IMG_NULL);
	PVR_ASSERT(hArena == psImport->sDeviceImport.psHeap);
	PVR_ASSERT(uiBase == psImport->sDeviceImport.sDevVAddr.uiAddr);

	_DevmemImportStructDevUnmap(psImport);
	_DevmemImportStructRelease(psImport);
}

/*****************************************************************************
 *                    Devmem context internals                               *
 *****************************************************************************/

static PVRSRV_ERROR
_PopulateContextFromBlueprint(struct _DEVMEM_CONTEXT_ *psCtx,
			      IMG_HANDLE hDeviceNode,
			      DEVMEM_HEAPCFGID uiHeapBlueprintID)
{
	PVRSRV_ERROR eError;
	PVRSRV_ERROR eError2;
	struct _DEVMEM_HEAP_ **ppsHeapArray;
	IMG_UINT32 uiNumHeaps;
	IMG_UINT32 uiHeapsToUnwindOnError;
	IMG_UINT32 uiHeapIndex;
	IMG_DEV_VIRTADDR sDevVAddrBase;
	IMG_CHAR aszHeapName[DEVMEM_HEAPNAME_MAXLENGTH];
	IMG_DEVMEM_SIZE_T uiHeapLength;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2DataPageSize;

	eError = DevmemHeapCount(psCtx->hBridge,
				 hDeviceNode, uiHeapBlueprintID, &uiNumHeaps);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	/* TODO: use new host mem alloc api */
	if (uiNumHeaps == 0) {
		ppsHeapArray = IMG_NULL;
	} else {
		ppsHeapArray = OSAllocMem(sizeof(*ppsHeapArray) * uiNumHeaps);
		if (ppsHeapArray == IMG_NULL) {
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto e0;
		}
	}

	uiHeapsToUnwindOnError = 0;

	for (uiHeapIndex = 0; uiHeapIndex < uiNumHeaps; uiHeapIndex++) {
		eError = DevmemHeapDetails(psCtx->hBridge,
					   hDeviceNode,
					   uiHeapBlueprintID,
					   uiHeapIndex,
					   &aszHeapName[0],
					   sizeof(aszHeapName),
					   &sDevVAddrBase,
					   &uiHeapLength, &uiLog2DataPageSize);
		if (eError != PVRSRV_OK) {
			PVR_ASSERT(uiHeapIndex == 0);
			goto e1;
		}

		eError = DevmemCreateHeap(psCtx,
					  sDevVAddrBase,
					  uiHeapLength,
					  uiLog2DataPageSize,
					  aszHeapName,
					  &ppsHeapArray[uiHeapIndex]);
		if (eError != PVRSRV_OK) {
			PVR_ASSERT(uiHeapIndex == 0);
			goto e1;
		}

		uiHeapsToUnwindOnError = uiHeapIndex + 1;
	}

	psCtx->uiAutoHeapCount = uiNumHeaps;
	psCtx->ppsAutoHeapArray = ppsHeapArray;

	PVR_ASSERT(psCtx->uiNumHeaps >= psCtx->uiAutoHeapCount);
	PVR_ASSERT(psCtx->uiAutoHeapCount == uiNumHeaps);

	return PVRSRV_OK;

	/*
	   error exit paths
	 */
 e1:
	for (uiHeapIndex = 0; uiHeapIndex < uiHeapsToUnwindOnError;
	     uiHeapIndex++) {
		eError2 = DevmemDestroyHeap(ppsHeapArray[uiHeapIndex]);
		PVR_ASSERT(eError2 == PVRSRV_OK);
	}

	if (uiNumHeaps != 0) {
		OSFreeMem(ppsHeapArray);
	}

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static IMG_VOID _UnpopulateContextFromBlueprint(struct _DEVMEM_CONTEXT_ *psCtx)
{
	PVRSRV_ERROR eError2;
	IMG_UINT32 uiHeapIndex;
	IMG_BOOL bDoCheck = IMG_TRUE;
#if defined(__KERNEL__)
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	if (psPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK) {
		bDoCheck = IMG_FALSE;
	}
#endif

	PVR_ASSERT(psCtx->uiNumHeaps >= psCtx->uiAutoHeapCount);

	for (uiHeapIndex = 0; uiHeapIndex < psCtx->uiAutoHeapCount;
	     uiHeapIndex++) {
		eError2 =
		    DevmemDestroyHeap(psCtx->ppsAutoHeapArray[uiHeapIndex]);
		if (bDoCheck) {
			PVR_ASSERT(eError2 == PVRSRV_OK);
		}
	}

	if (psCtx->uiAutoHeapCount != 0) {
		OSFreeMem(psCtx->ppsAutoHeapArray);
		psCtx->ppsAutoHeapArray = IMG_NULL;
	}
	psCtx->uiAutoHeapCount = 0;

	PVR_ASSERT(psCtx->uiAutoHeapCount == 0);
	PVR_ASSERT(psCtx->ppsAutoHeapArray == IMG_NULL);
}

/*****************************************************************************
 *                    Devmem context functions                               *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemCreateContext(DEVMEM_BRIDGE_HANDLE hBridge,
		    IMG_HANDLE hDeviceNode,
		    DEVMEM_HEAPCFGID uiHeapBlueprintID,
		    DEVMEM_CONTEXT ** ppsCtxPtr)
{
	PVRSRV_ERROR eError;
	DEVMEM_CONTEXT *psCtx;
	/* handle to the server-side counterpart of the device memory
	   context (specifically, for handling mapping to device MMU) */
	IMG_HANDLE hDevMemServerContext;
	IMG_HANDLE hPrivData;

	if (ppsCtxPtr == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	psCtx = OSAllocMem(sizeof *psCtx);
	if (psCtx == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	psCtx->uiNumHeaps = 0;

	psCtx->hBridge = hBridge;

	/* Create (server-side) Device Memory context */
	eError = BridgeDevmemIntCtxCreate(psCtx->hBridge,
					  hDeviceNode,
					  &hDevMemServerContext, &hPrivData);
	if (eError != PVRSRV_OK) {
		goto e1;
	}
	psCtx->hDeviceNode = hDeviceNode;
	psCtx->hDevMemServerContext = hDevMemServerContext;
	psCtx->hPrivData = hPrivData;

	/* automagic heap creation */
	psCtx->uiAutoHeapCount = 0;

	eError =
	    _PopulateContextFromBlueprint(psCtx, hDeviceNode,
					  uiHeapBlueprintID);
	if (eError != PVRSRV_OK) {
		goto e2;
	}

	*ppsCtxPtr = psCtx;

	PVR_ASSERT(psCtx->uiNumHeaps == psCtx->uiAutoHeapCount);
	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 e2:
	PVR_ASSERT(psCtx->uiAutoHeapCount == 0);
	PVR_ASSERT(psCtx->uiNumHeaps == 0);
	BridgeDevmemIntCtxDestroy(psCtx->hBridge, hDevMemServerContext);

 e1:
	OSFreeMem(psCtx);

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAcquireDevPrivData(DEVMEM_CONTEXT * psCtx, IMG_HANDLE * hPrivData)
{
	PVRSRV_ERROR eError;

	if ((psCtx == IMG_NULL) || (hPrivData == IMG_NULL)) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	*hPrivData = psCtx->hPrivData;
	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR DevmemReleaseDevPrivData(DEVMEM_CONTEXT * psCtx)
{
	PVRSRV_ERROR eError;

	if (psCtx == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}
	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemFindHeapByName(const struct _DEVMEM_CONTEXT_ * psCtx,
		     const IMG_CHAR * pszHeapName,
		     struct _DEVMEM_HEAP_ ** ppsHeapRet)
{
	IMG_UINT32 uiHeapIndex;

	/* N.B.  This func is only useful for finding "automagic" heaps by name */
	for (uiHeapIndex = 0;
	     uiHeapIndex < psCtx->uiAutoHeapCount; uiHeapIndex++) {
		if (!OSStringCompare
		    (psCtx->ppsAutoHeapArray[uiHeapIndex]->pszName,
		     pszHeapName)) {
			*ppsHeapRet = psCtx->ppsAutoHeapArray[uiHeapIndex];
			return PVRSRV_OK;
		}
	}

	return PVRSRV_ERROR_DEVICEMEM_INVALID_HEAP_INDEX;
}

IMG_INTERNAL PVRSRV_ERROR DevmemDestroyContext(DEVMEM_CONTEXT * psCtx)
{
	PVRSRV_ERROR eError;
	IMG_BOOL bDoCheck = IMG_TRUE;

#if defined(__KERNEL__)
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	if (psPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK) {
		bDoCheck = IMG_FALSE;
	}
#endif

	if (psCtx == IMG_NULL) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* should be only the automagically instantiated heaps left */
	if (psCtx->uiNumHeaps != psCtx->uiAutoHeapCount) {
		/* TODO: not really the right error message */
		return PVRSRV_ERROR_DEVICEMEM_ALLOCATIONS_REMAIN_IN_HEAP;
	}

	_UnpopulateContextFromBlueprint(psCtx);

	if (bDoCheck) {
		PVR_ASSERT(psCtx->uiAutoHeapCount == 0);
		PVR_ASSERT(psCtx->uiNumHeaps == 0);
	}
	eError = BridgeDevmemIntCtxDestroy(psCtx->hBridge,
					   psCtx->hDevMemServerContext);
	if (bDoCheck) {
		PVR_ASSERT(eError == PVRSRV_OK);
	}

	OSFreeMem(psCtx);

	return PVRSRV_OK;
}

/*****************************************************************************
 *                 Devmem heap query functions                               *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapConfigCount(DEVMEM_BRIDGE_HANDLE hBridge,
		      IMG_HANDLE hDeviceNode, IMG_UINT32 * puiNumHeapConfigsOut)
{
	PVRSRV_ERROR eError;

	eError = BridgeHeapCfgHeapConfigCount(hBridge,
					      hDeviceNode,
					      puiNumHeapConfigsOut);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapCount(DEVMEM_BRIDGE_HANDLE hBridge,
		IMG_HANDLE hDeviceNode,
		IMG_UINT32 uiHeapConfigIndex, IMG_UINT32 * puiNumHeapsOut)
{
	PVRSRV_ERROR eError;

	eError = BridgeHeapCfgHeapCount(hBridge,
					hDeviceNode,
					uiHeapConfigIndex, puiNumHeapsOut);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapConfigName(DEVMEM_BRIDGE_HANDLE hBridge,
		     IMG_HANDLE hDeviceNode,
		     IMG_UINT32 uiHeapConfigIndex,
		     IMG_CHAR * pszConfigNameOut, IMG_UINT32 uiConfigNameBufSz)
{
	PVRSRV_ERROR eError;

	eError = BridgeHeapCfgHeapConfigName(hBridge,
					     hDeviceNode,
					     uiHeapConfigIndex,
					     uiConfigNameBufSz,
					     pszConfigNameOut);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemHeapDetails(DEVMEM_BRIDGE_HANDLE hBridge,
		  IMG_HANDLE hDeviceNode,
		  IMG_UINT32 uiHeapConfigIndex,
		  IMG_UINT32 uiHeapIndex,
		  IMG_CHAR * pszHeapNameOut,
		  IMG_UINT32 uiHeapNameBufSz,
		  IMG_DEV_VIRTADDR * psDevVAddrBaseOut,
		  IMG_DEVMEM_SIZE_T * puiHeapLengthOut,
		  IMG_UINT32 * puiLog2DataPageSizeOut)
{
	PVRSRV_ERROR eError;

	eError = BridgeHeapCfgHeapDetails(hBridge,
					  hDeviceNode,
					  uiHeapConfigIndex,
					  uiHeapIndex,
					  uiHeapNameBufSz,
					  pszHeapNameOut,
					  psDevVAddrBaseOut,
					  puiHeapLengthOut,
					  puiLog2DataPageSizeOut);

	return eError;
}

/*****************************************************************************
 *                    Devmem heap functions                                  *
 *****************************************************************************/

/* See devicemem.h for important notes regarding the arguments
   to this function */
IMG_INTERNAL PVRSRV_ERROR
DevmemCreateHeap(DEVMEM_CONTEXT * psCtx,
		 IMG_DEV_VIRTADDR sBaseAddress,
		 IMG_DEVMEM_SIZE_T uiLength,
		 IMG_UINT32 ui32Log2Quantum,
		 const IMG_CHAR * pszName, DEVMEM_HEAP ** ppsHeapPtr)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	DEVMEM_HEAP *psHeap;
	/* handle to the server-side counterpart of the device memory
	   heap (specifically, for handling mapping to device MMU */
	IMG_HANDLE hDevMemServerHeap;

	IMG_CHAR aszBuf[100];
	IMG_CHAR *pszStr;

	if (ppsHeapPtr == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	psHeap = OSAllocMem(sizeof *psHeap);
	if (psHeap == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	/* Need to keep local copy of heap name, so caller may free
	   theirs */
	pszStr = OSAllocMem(OSStringLength(pszName) + 1);
	if (pszStr == IMG_NULL) {
		goto e1;
	}
	OSStringCopy(pszStr, pszName);
	psHeap->pszName = pszStr;

	psHeap->sBaseAddress = sBaseAddress;
	psHeap->uiImportCount = 0;

	OSSNPrintf(aszBuf, sizeof(aszBuf),
		   "NDM heap '%s' (suballocs) ctx:%p", pszName, psCtx);
	pszStr = OSAllocMem(OSStringLength(aszBuf) + 1);
	if (pszStr == IMG_NULL) {
		goto e2;
	}
	OSStringCopy(pszStr, aszBuf);
	psHeap->pszSubAllocRAName = pszStr;

	psHeap->psSubAllocRA = RA_Create(psHeap->pszSubAllocRAName,
					 /* Initial import: */
					 (RA_BASE_T) 0, (RA_LENGTH_T) 0, (RA_FLAGS_T) 0, IMG_NULL,	/* per ispan handle */
					 /* Subsequent imports: */
					 ui32Log2Quantum,
					 _SubAllocImportAlloc,
					 _SubAllocImportFree,
					 (RA_PERARENA_HANDLE) psHeap);
	if (psHeap->psSubAllocRA == IMG_NULL) {
		eError = PVRSRV_ERROR_DEVICEMEM_UNABLE_TO_CREATE_ARENA;
		goto e3;
	}

	psHeap->uiLog2Quantum = ui32Log2Quantum;

	OSSNPrintf(aszBuf, sizeof(aszBuf),
		   "NDM heap '%s' (QVM) ctx:%p", pszName, psCtx);
	pszStr = OSAllocMem(OSStringLength(aszBuf) + 1);
	if (pszStr == IMG_NULL) {
		goto e4;
	}
	OSStringCopy(pszStr, aszBuf);
	psHeap->pszQuantizedVMRAName = pszStr;

	psHeap->psQuantizedVMRA = RA_Create(psHeap->pszQuantizedVMRAName,
					    /* Initial import: */
					    (RA_BASE_T) sBaseAddress.uiAddr, (RA_LENGTH_T) uiLength, (RA_FLAGS_T) 0,	/* This RA doesn't use or need flags */
					    IMG_NULL,	/* per ispan handle */
					    /* Subsequent import: */
					    0, IMG_NULL, IMG_NULL,
					    (RA_PERARENA_HANDLE) psHeap);
	if (psHeap->psQuantizedVMRA == IMG_NULL) {
		eError = PVRSRV_ERROR_DEVICEMEM_UNABLE_TO_CREATE_ARENA;
		goto e5;
	}

	psHeap->psCtx = psCtx;

	/* Create server-side counterpart of Device Memory heap */
	eError = BridgeDevmemIntHeapCreate(psCtx->hBridge, psCtx->hDevMemServerContext, sBaseAddress, uiLength, 12,	/* TODO: Fix harcoded value, need to get log2 page-size */
					   &hDevMemServerHeap);
	if (eError != PVRSRV_OK) {
		goto e6;
	}
	psHeap->hDevMemServerHeap = hDevMemServerHeap;

	psHeap->psCtx->uiNumHeaps++;
	*ppsHeapPtr = psHeap;

#if defined PVRSRV_NEWDEVMEM_SUPPORT_MEM_TRACKING
	psHeap->psMemDescList = IMG_NULL;
#endif				/* PVRSRV_NEWDEVMEM_SUPPORT_MEM_TRACKING */

	return PVRSRV_OK;

	/*
	   error exit paths
	 */

 e6:
	RA_Delete(psHeap->psQuantizedVMRA);
 e5:
	OSFreeMem(psHeap->pszQuantizedVMRAName);
 e4:
	RA_Delete(psHeap->psSubAllocRA);
 e3:
	OSFreeMem(psHeap->pszSubAllocRAName);
 e2:
	OSFreeMem(psHeap->pszName);
 e1:
	OSFreeMem(psHeap);
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemGetHeapBaseDevVAddr(struct _DEVMEM_HEAP_ * psHeap,
			  IMG_DEV_VIRTADDR * pDevVAddr)
{
	if (psHeap == IMG_NULL) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	*pDevVAddr = psHeap->sBaseAddress;

	return PVRSRV_OK;
}

IMG_INTERNAL IMG_VOID
DevmemExportalignAdjustSizeAndAlign(DEVMEM_HEAP * psHeap,
				    IMG_DEVMEM_SIZE_T * puiSize,
				    IMG_DEVMEM_ALIGN_T * puiAlign)
{
	IMG_DEVMEM_SIZE_T uiSize = *puiSize;
	IMG_DEVMEM_ALIGN_T uiAlign = *puiAlign;
	IMG_UINT32 uiLog2Quantum;

	if (psHeap) {
		uiLog2Quantum = psHeap->uiLog2Quantum;
	} else {
		uiLog2Quantum = 12;	/* FIXME: Call OS function */
	}

	if ((1ULL << uiLog2Quantum) > uiAlign) {
		uiAlign = 1ULL << uiLog2Quantum;
	}
	uiSize = (uiSize + uiAlign - 1) & ~(uiAlign - 1);

	*puiSize = uiSize;
	*puiAlign = uiAlign;
}

IMG_INTERNAL PVRSRV_ERROR DevmemDestroyHeap(DEVMEM_HEAP * psHeap)
{
	PVRSRV_ERROR eError;

	if (psHeap == IMG_NULL) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	if (psHeap->uiImportCount > 0) {
		PVR_DPF((PVR_DBG_ERROR, "%d(%s) leaks remain",
			 psHeap->uiImportCount, psHeap->pszName));
		return PVRSRV_ERROR_DEVICEMEM_ALLOCATIONS_REMAIN_IN_HEAP;
	}

	PVR_ASSERT(psHeap->psCtx->uiNumHeaps > 0);
	psHeap->psCtx->uiNumHeaps--;

	eError = BridgeDevmemIntHeapDestroy(psHeap->psCtx->hBridge,
					    psHeap->hDevMemServerHeap);
	PVR_ASSERT(eError == PVRSRV_OK);

	RA_Delete(psHeap->psQuantizedVMRA);
	OSFreeMem(psHeap->pszQuantizedVMRAName);

	RA_Delete(psHeap->psSubAllocRA);
	OSFreeMem(psHeap->pszSubAllocRAName);

	OSFreeMem(psHeap->pszName);

	OSFreeMem(psHeap);

	return PVRSRV_OK;
}

/*****************************************************************************
 *                Devmem allocation/free functions                           *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemAllocate(DEVMEM_HEAP * psHeap,
	       IMG_DEVMEM_SIZE_T uiSize,
	       IMG_DEVMEM_ALIGN_T uiAlign,
	       DEVMEM_FLAGS_T uiFlags, DEVMEM_MEMDESC ** ppsMemDescPtr)
{
	IMG_BOOL bStatus;	/* eError for RA */
	RA_BASE_T uiAllocatedAddr;
	RA_LENGTH_T uiAllocatedSize;
	RA_PERISPAN_HANDLE hImport;	/* the "import" from which this sub-allocation came */
	RA_FLAGS_T uiFlagsForRA;
	PVRSRV_ERROR eError;
	DEVMEM_MEMDESC *psMemDesc;
	IMG_DEVMEM_OFFSET_T uiOffset = 0;
	DEVMEM_IMPORT *psImport;

	if (psHeap == IMG_NULL || ppsMemDescPtr == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	eError = _DevmemValidateParams(uiSize, uiAlign, uiFlags);
	if (eError != PVRSRV_OK) {
		goto failParams;
	}

	psMemDesc = OSAllocMem(sizeof *psMemDesc);
	if (psMemDesc == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto failMemDescAlloc;
	}

	/*
	   No request for exportable memory so use the RA
	 */
	uiFlagsForRA =
	    (RA_FLAGS_T) (uiFlags &
			  PVRSRV_MEMALLOCFLAGS_RA_DIFFERENTIATION_MASK);
	/* Check that the cast didn't lose any flags due to differint integer widths */
	PVR_ASSERT(uiFlagsForRA ==
		   (uiFlags & PVRSRV_MEMALLOCFLAGS_RA_DIFFERENTIATION_MASK));

	/* 
	   When the RA suballocates memory from a Span it does not zero it. It only zeroes the
	   memory if it allocates a new Span; but we don't know what is going to happen for this
	   RA_Alloc call. Therefore, we zero the mem after the allocation below.
	 */
	uiFlagsForRA &= ~PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC;

	/*
	   FIXME?

	   The RA uses flags for its internal differentiation for
	   determining whether allocations can share imports.  It also
	   passes the flags to the import function.  This is two separate
	   things, and IMO, the infomation to be passed to the import
	   ought to be private to the user of RA, and could be richer than
	   just some flags, e.g. it could be a tuple of numbers
	   (e.g. priority etc.) so this is currently broken as the PMR
	   will get the same flags (and only such flags) that we pass to
	   the RA.  This will be fine until we need to pass more data than
	   fits in the flags.  This needs to be fixed.

	   Ditto for the mapping flags.  At the moment the mapping can
	   only have the same flags as we pass here into the RA.
	 */
	bStatus = RA_Alloc(psHeap->psSubAllocRA,
			   uiSize,
			   uiFlagsForRA,
			   uiAlign,
			   &uiAllocatedAddr, &uiAllocatedSize, &hImport);
	if (!bStatus) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto failDeviceMemAlloc;
	}

	psImport = hImport;
	uiOffset = uiAllocatedAddr - psImport->sDeviceImport.sDevVAddr.uiAddr;

	_DevmemMemDescInit(psMemDesc, uiOffset, psImport);

	/* zero the memory */
	if (uiFlags & PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC) {
		IMG_VOID *pvAddr;
		PVRSRV_ERROR eError;

		eError = DevmemAcquireCpuVirtAddr(psMemDesc, &pvAddr);
		if (eError != PVRSRV_OK) {
			goto failDeviceMemAlloc;
		}

		OSMemSet(pvAddr, 0x0, uiSize);

		DevmemReleaseCpuVirtAddr(psMemDesc);
	}

	*ppsMemDescPtr = psMemDesc;

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 failDeviceMemAlloc:
	OSFreeMem(psMemDesc);
 failMemDescAlloc:
 failParams:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAllocateExportable(IMG_HANDLE hBridge,
			 IMG_HANDLE hDeviceNode,
			 IMG_DEVMEM_SIZE_T uiSize,
			 IMG_DEVMEM_ALIGN_T uiAlign,
			 DEVMEM_FLAGS_T uiFlags,
			 DEVMEM_MEMDESC ** ppsMemDescPtr)
{
	PVRSRV_ERROR eError;
	DEVMEM_MEMDESC *psMemDesc;
	DEVMEM_IMPORT *psImport;
	IMG_UINT32 uiLog2Quantum = 12;	/* FIXME: Call OS function */

	DevmemExportalignAdjustSizeAndAlign(IMG_NULL, &uiSize, &uiAlign);

	eError = _DevmemValidateParams(uiSize, uiAlign, uiFlags);
	if (eError != PVRSRV_OK) {
		goto failParams;
	}

	psMemDesc = OSAllocMem(sizeof *psMemDesc);
	if (psMemDesc == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto failMemDescAlloc;
	}

	/*
	   Note:
	   In the case of exportable memory we have no heap to
	   query the pagesize from, so we assume host pagesize.
	 */
	eError = _AllocateDeviceMemory(hBridge,
				       hDeviceNode,
				       uiLog2Quantum,
				       uiSize,
				       uiAlign, uiFlags, IMG_TRUE, &psImport);
	if (eError != PVRSRV_OK) {
		goto failDeviceMemAlloc;
	}

	_DevmemMemDescInit(psMemDesc, 0, psImport);

	*ppsMemDescPtr = psMemDesc;

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 failDeviceMemAlloc:
	OSFreeMem(psMemDesc);

 failMemDescAlloc:
 failParams:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

/*****************************************************************************
 *                Devmem unsecure export functions                           *
 *****************************************************************************/

IMG_INTERNAL PVRSRV_ERROR
DevmemExport(DEVMEM_MEMDESC * psMemDesc, DEVMEM_EXPORTCOOKIE * psExportCookie)
{
	/* Caller to provide storage for export cookie struct */
	PVRSRV_ERROR eError;
	IMG_HANDLE /* TODO: SID! */ hPMRExportHandle = 0;
	IMG_UINT64 uiPMRExportPassword = 0;	/* TODO: better type than this? */
	IMG_DEVMEM_SIZE_T uiSize = 0;
	IMG_DEVMEM_LOG2ALIGN_T uiLog2Contig = 0;

	if (psMemDesc == IMG_NULL || psExportCookie == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	eError = _Mapping_Export(psMemDesc->psImport,
				 &hPMRExportHandle,
				 &uiPMRExportPassword, &uiSize, &uiLog2Contig);
	if (eError != PVRSRV_OK) {
		psExportCookie->uiSize = 0;
		goto e0;
	}

	psExportCookie->hPMRExportHandle = hPMRExportHandle;
	psExportCookie->uiPMRExportPassword = uiPMRExportPassword;
	psExportCookie->uiSize = uiSize;
	psExportCookie->uiLog2ContiguityGuarantee = uiLog2Contig;

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemMakeServerExportClientExport(DEVMEM_BRIDGE_HANDLE hBridge,
				   DEVMEM_SERVER_EXPORTCOOKIE
				   hServerExportCookie,
				   DEVMEM_EXPORTCOOKIE * psExportCookie)
{
	DEVMEM_EXPORTHANDLE hPMRExportHandle;
	IMG_DEVMEM_SIZE_T uiPMRSize;
	IMG_DEVMEM_LOG2ALIGN_T uiPMRLog2Contig;
	DEVMEM_EXPORTKEY uiPMRExportKey;
	PVRSRV_ERROR eError = PVRSRV_OK;

	eError = BridgePMRMakeServerExportClientExport(hBridge,
						       hServerExportCookie,
						       &hPMRExportHandle,
						       &uiPMRSize,
						       &uiPMRLog2Contig,
						       &uiPMRExportKey);

	if (eError == PVRSRV_OK) {
		psExportCookie->hPMRExportHandle = hPMRExportHandle;
		psExportCookie->uiPMRExportPassword = uiPMRExportKey;
		psExportCookie->uiSize = uiPMRSize;
		psExportCookie->uiLog2ContiguityGuarantee = uiPMRLog2Contig;
	}

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemUnmakeServerExportClientExport(DEVMEM_BRIDGE_HANDLE hBridge,
				     DEVMEM_EXPORTCOOKIE * psExportCookie)
{
	return BridgePMRUnmakeServerExportClientExport(hBridge,
						       psExportCookie->
						       hPMRExportHandle);
}

IMG_INTERNAL IMG_BOOL
DevmemIsValidExportCookie(DEVMEM_EXPORTCOOKIE * psExportCookie)
{
	/* Assume that if the size is set, the export cookie is used */
	return (psExportCookie->uiSize != 0x0);
}

IMG_INTERNAL IMG_VOID
DevmemUnexport(DEVMEM_MEMDESC * psMemDesc, DEVMEM_EXPORTCOOKIE * psExportCookie)
{
	_Mapping_Unexport(psMemDesc->psImport,
			  psExportCookie->hPMRExportHandle);

	psExportCookie->uiSize = 0;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemImport(IMG_HANDLE hBridge,
	     DEVMEM_EXPORTCOOKIE * psCookie,
	     DEVMEM_FLAGS_T uiFlags, DEVMEM_MEMDESC ** ppsMemDescPtr)
{
	DEVMEM_MEMDESC *psMemDesc;
	DEVMEM_IMPORT *psImport;
	IMG_HANDLE hPMR;
	PVRSRV_ERROR eError;

	if (ppsMemDescPtr == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	psMemDesc = OSAllocMem(sizeof *psMemDesc);
	if (psMemDesc == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto failMemDescAlloc;
	}

	eError = _DevmemImportStructAlloc(hBridge, IMG_TRUE, &psImport);
	if (eError != PVRSRV_OK) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto failImportAlloc;
	}

	/* Get a handle to the PMR (inc refcount, checks authorization) */
	eError = BridgePMRImportPMR(hBridge, psCookie->hPMRExportHandle, psCookie->uiPMRExportPassword, psCookie->uiSize,	/* not trusted - just for sanity checks */
				    psCookie->uiLog2ContiguityGuarantee,	/* not trusted - just for sanity checks */
				    &hPMR);
	if (eError != PVRSRV_OK) {
		goto failImport;
	}

	_DevmemImportStructInit(psImport,
				psCookie->uiSize,
				1ULL << psCookie->uiLog2ContiguityGuarantee,
				uiFlags, hPMR);

	_DevmemMemDescInit(psMemDesc, 0, psImport);

	*ppsMemDescPtr = psMemDesc;

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 failImport:
	_DevmemImportStructRelease(psImport);
 failImportAlloc:
	OSFreeMem(psMemDesc);
 failMemDescAlloc:
 failParams:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

/*****************************************************************************
 *                   Common MemDesc functions                                *
 *****************************************************************************/

/*
	This function is called for freeing any class of memory
*/
IMG_INTERNAL IMG_VOID DevmemFree(DEVMEM_MEMDESC * psMemDesc)
{
	_DevmemMemDescRelease(psMemDesc);
}

IMG_INTERNAL PVRSRV_ERROR
DevmemMapToDevice(DEVMEM_MEMDESC * psMemDesc,
		  DEVMEM_HEAP * psHeap, IMG_DEV_VIRTADDR * psDevVirtAddr)
{
	DEVMEM_IMPORT *psImport;
	IMG_DEV_VIRTADDR sDevVAddr;
	PVRSRV_ERROR eError;

	if (psHeap == IMG_NULL) {
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto failParams;
	}

	if (psMemDesc->sDeviceMemDesc.ui32RefCount != 0) {
		eError = PVRSRV_ERROR_DEVICEMEM_ALREADY_MAPPED;
		goto failCheck;
	}

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psMemDesc,
			      psMemDesc->sDeviceMemDesc.ui32RefCount,
			      psMemDesc->sDeviceMemDesc.ui32RefCount + 1);

	psImport = psMemDesc->psImport;
	_DevmemMemDescAcquire(psMemDesc);

	eError = _DevmemImportStructDevMap(psHeap, IMG_TRUE, psImport);
	if (eError != PVRSRV_OK) {
		goto failMap;
	}

	sDevVAddr.uiAddr = psImport->sDeviceImport.sDevVAddr.uiAddr;
	sDevVAddr.uiAddr += psMemDesc->uiOffset;
	psMemDesc->sDeviceMemDesc.sDevVAddr = sDevVAddr;
	psMemDesc->sDeviceMemDesc.ui32RefCount++;

	*psDevVirtAddr = psMemDesc->sDeviceMemDesc.sDevVAddr;

	return PVRSRV_OK;

 failMap:
	_DevmemMemDescRelease(psMemDesc);
 failCheck:
 failParams:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAcquireDevVirtAddr(DEVMEM_MEMDESC * psMemDesc,
			 IMG_DEV_VIRTADDR * psDevVirtAddr)
{
	PVRSRV_ERROR eError;

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psMemDesc,
			      psMemDesc->sDeviceMemDesc.ui32RefCount,
			      psMemDesc->sDeviceMemDesc.ui32RefCount + 1);

	if (psMemDesc->sDeviceMemDesc.ui32RefCount == 0) {
		eError = PVRSRV_ERROR_DEVICEMEM_NO_MAPPING;
		goto failCheck;
	}
	psMemDesc->sDeviceMemDesc.ui32RefCount++;

	*psDevVirtAddr = psMemDesc->sDeviceMemDesc.sDevVAddr;

	return PVRSRV_OK;

 failCheck:
	PVR_ASSERT(eError != PVRSRV_OK);

	return eError;
}

IMG_INTERNAL IMG_VOID DevmemReleaseDevVirtAddr(DEVMEM_MEMDESC * psMemDesc)
{
	PVR_ASSERT(psMemDesc != IMG_NULL);
	PVR_ASSERT(psMemDesc->sDeviceMemDesc.ui32RefCount != 0);

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psMemDesc,
			      psMemDesc->sDeviceMemDesc.ui32RefCount,
			      psMemDesc->sDeviceMemDesc.ui32RefCount - 1);

	if (--psMemDesc->sDeviceMemDesc.ui32RefCount == 0) {
		DEVMEM_IMPORT *psImport;
		PVRSRV_ERROR eError;

		psImport = psMemDesc->psImport;

		/*
		   request an SLC flush and invalidate

		   We do it here as not all memory allocations/imports are
		   mapped into the device.
		 */
		eError = BridgeDevmemSLCFlushInvalRequest(psImport->hBridge,
							  psImport->
							  sDeviceImport.psHeap->
							  psCtx->hDeviceNode,
							  psImport->hPMR);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "%s: SLC Flush and Invalidate request failed with error %d",
				 __FUNCTION__, eError));
		}

		_DevmemImportStructDevUnmap(psImport);
		_DevmemMemDescRelease(psMemDesc);
	}
}

IMG_INTERNAL PVRSRV_ERROR
DevmemAcquireCpuVirtAddr(DEVMEM_MEMDESC * psMemDesc, IMG_VOID ** ppvCpuVirtAddr)
{
	PVRSRV_ERROR eError;

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psMemDesc,
			      psMemDesc->sCPUMemDesc.ui32RefCount,
			      psMemDesc->sCPUMemDesc.ui32RefCount + 1);

	if (psMemDesc->sCPUMemDesc.ui32RefCount++ == 0) {
		DEVMEM_IMPORT *psImport = psMemDesc->psImport;
		IMG_UINT8 *pui8CPUVAddr;

		_DevmemMemDescAcquire(psMemDesc);
		eError = _DevmemImportStructCPUMap(psImport);
		if (eError != PVRSRV_OK) {
			goto failMap;
		}

		pui8CPUVAddr = psImport->sCPUImport.pvCPUVAddr;
		pui8CPUVAddr += psMemDesc->uiOffset;
		psMemDesc->sCPUMemDesc.pvCPUVAddr = pui8CPUVAddr;
	}
	*ppvCpuVirtAddr = psMemDesc->sCPUMemDesc.pvCPUVAddr;

	return PVRSRV_OK;

 failMap:
	PVR_ASSERT(eError != PVRSRV_OK);
	_DevmemMemDescRelease(psMemDesc);

	return eError;
}

IMG_INTERNAL IMG_VOID DevmemReleaseCpuVirtAddr(DEVMEM_MEMDESC * psMemDesc)
{
	PVR_ASSERT(psMemDesc != IMG_NULL);
	PVR_ASSERT(psMemDesc->sCPUMemDesc.ui32RefCount != 0);

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psMemDesc,
			      psMemDesc->sCPUMemDesc.ui32RefCount,
			      psMemDesc->sCPUMemDesc.ui32RefCount - 1);

	if (--psMemDesc->sCPUMemDesc.ui32RefCount == 0) {
		_DevmemImportStructCPUUnmap(psMemDesc->psImport);
		_DevmemMemDescRelease(psMemDesc);
	}
}

IMG_INTERNAL PVRSRV_ERROR
DevmemGetImportUID(DEVMEM_MEMDESC * psMemDesc, IMG_UINT64 * pui64UID)
{
	DEVMEM_IMPORT *psImport = psMemDesc->psImport;
	PVRSRV_ERROR eError;

	eError = BridgePMRGetUID(psImport->hBridge, psImport->hPMR, pui64UID);

	return eError;
}
