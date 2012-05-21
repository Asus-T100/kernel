									    /*************************************************************************//*!
									       @File           physmem_lma.c
									       @Title          Local card memory allocator
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Part of the memory management. This module is responsible for
									       implementing the function callbacks for local card memory.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "img_types.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"
#include "pvrsrv_memallocflags.h"

#include "allocmem.h"
#include "osfunc.h"
#include "pvrsrv.h"
#include "physmem_lma.h"
#include "pdump_physmem.h"
#include "pmr.h"
#include "pmr_impl.h"

typedef struct _PMR_LMALLOCARRAY_DATA_ {
	PVRSRV_DEVICE_NODE *psDevNode;
	IMG_UINT32 uiNumAllocs;
	IMG_UINT32 uiLog2AllocSize;
	IMG_UINT32 uiAllocSize;
	IMG_DEV_PHYADDR *pasDevPAddr;

	/*
	   for pdump...
	 */
	IMG_BOOL bPDumpMalloced;
	IMG_HANDLE hPDumpAllocInfo;

	/*
	   record at alloc time whether poisoning will be required when the
	   PMR is freed.
	 */
	IMG_BOOL bPoisonOnFree;
} PMR_LMALLOCARRAY_DATA;

static PVRSRV_ERROR _MapAlloc(PVRSRV_DEVICE_NODE * psDevNode,
			      IMG_DEV_PHYADDR * psDevPAddr, IMG_SIZE_T uiSize,
			      IMG_VOID ** pvPtr)
{
	IMG_CPU_PHYADDR sCpuPAddr;

	PhysHeapDevPAddrToCpuPAddr(psDevNode->psPhysHeap, &sCpuPAddr,
				   psDevPAddr);
	*pvPtr = OSMapPhysToLin(sCpuPAddr, uiSize, 0);

	if (*pvPtr == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	} else {
		return PVRSRV_OK;
	}
}

static IMG_VOID _UnMapAlloc(PVRSRV_DEVICE_NODE * psDevNode, IMG_SIZE_T uiSize,
			    IMG_VOID * pvPtr)
{
	OSUnMapPhysToLin(pvPtr, uiSize, 0);
}

static PVRSRV_ERROR
_PoisonAlloc(PVRSRV_DEVICE_NODE * psDevNode,
	     IMG_DEV_PHYADDR * psDevPAddr,
	     IMG_UINT32 uiAllocSize,
	     const IMG_CHAR * pacPoisonData, IMG_SIZE_T uiPoisonSize)
{
	IMG_UINT32 uiSrcByteIndex;
	IMG_UINT32 uiDestByteIndex;
	IMG_VOID *pvKernLin = IMG_NULL;
	IMG_CHAR *pcDest = IMG_NULL;

	PVRSRV_ERROR eError;

	eError = _MapAlloc(psDevNode, psDevPAddr, uiAllocSize, &pvKernLin);
	if (eError != PVRSRV_OK) {
		goto map_failed;
	}
	pcDest = pvKernLin;

	uiSrcByteIndex = 0;
	for (uiDestByteIndex = 0; uiDestByteIndex < uiAllocSize;
	     uiDestByteIndex++) {
		pcDest[uiDestByteIndex] = pacPoisonData[uiSrcByteIndex];
		uiSrcByteIndex++;
		if (uiSrcByteIndex == uiPoisonSize) {
			uiSrcByteIndex = 0;
		}
	}

	_UnMapAlloc(psDevNode, uiAllocSize, pvKernLin);
	return PVRSRV_OK;

 map_failed:
	PVR_DPF((PVR_DBG_ERROR, "Failed to poison allocation"));
	return eError;
}

static PVRSRV_ERROR
_ZeroAlloc(PVRSRV_DEVICE_NODE * psDevNode,
	   IMG_DEV_PHYADDR * psDevPAddr, IMG_UINT32 uiAllocSize)
{
	IMG_VOID *pvKernLin = IMG_NULL;
	PVRSRV_ERROR eError;

	eError = _MapAlloc(psDevNode, psDevPAddr, uiAllocSize, &pvKernLin);
	if (eError != PVRSRV_OK) {
		goto map_failed;
	}

	OSMemSet(pvKernLin, 0, uiAllocSize);

	_UnMapAlloc(psDevNode, uiAllocSize, pvKernLin);
	return PVRSRV_OK;

 map_failed:
	PVR_DPF((PVR_DBG_ERROR, "Failed to zero allocation"));
	return eError;
}

static const IMG_CHAR _AllocPoison[] = "^PoIsOn";
static const IMG_UINT32 _AllocPoisonSize = 7;
static const IMG_CHAR _FreePoison[] = "<DEAD-BEEF>";
static const IMG_UINT32 _FreePoisonSize = 11;

static PVRSRV_ERROR
_AllocLMPages(PVRSRV_DEVICE_NODE * psDevNode,
	      PMR_SIZE_T uiSize,
	      IMG_UINT32 uiLog2PageSize,
	      IMG_BOOL bZero,
	      IMG_BOOL bPoisonOnAlloc,
	      IMG_BOOL bPoisonOnFree,
	      IMG_BOOL bContig, PMR_LMALLOCARRAY_DATA ** ppsPageArrayDataPtr)
{
	PMR_LMALLOCARRAY_DATA *psPageArrayData = IMG_NULL;
	PVRSRV_ERROR eError;
	IMG_BOOL bAllocResult;
	RA_BASE_T uiCardAddr;
	RA_LENGTH_T uiActualSize;
	IMG_UINT32 i;

	PVR_ASSERT(!bZero || !bPoisonOnAlloc);

	if (uiSize >= 0x1000000000ULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "physmem_lma.c: Do you really want 64GB of physical memory in one go?  This is likely a bug"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	PVR_ASSERT(OSGetPageShift() <= uiLog2PageSize);

	if ((uiSize & ((1ULL << uiLog2PageSize) - 1)) != 0) {
		eError = PVRSRV_ERROR_PMR_NOT_PAGE_MULTIPLE;
		goto e0;
	}

	psPageArrayData = OSAllocMem(sizeof(PMR_LMALLOCARRAY_DATA));
	if (psPageArrayData == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}
	OSMemSet(psPageArrayData, 0, sizeof(PMR_LMALLOCARRAY_DATA));

	if (bContig) {
		/*
		   Some allocations require kernel mappings in which case in order
		   to be virtually contiguous we also have to be physically contiguous.
		 */
		psPageArrayData->uiNumAllocs = 1;
		psPageArrayData->uiAllocSize = uiSize;
		psPageArrayData->uiLog2AllocSize = uiLog2PageSize;
	} else {
		IMG_UINT32 uiNumPages;

		/* Use of cast below is justified by the assertion that follows to
		   prove that no significant bits have been truncated */
		uiNumPages =
		    (IMG_UINT32) (((uiSize - 1) >> uiLog2PageSize) + 1);
		PVR_ASSERT((uiNumPages << uiLog2PageSize) == uiSize);

		psPageArrayData->uiNumAllocs = uiNumPages;
		psPageArrayData->uiAllocSize = 1 << uiLog2PageSize;
		psPageArrayData->uiLog2AllocSize = uiLog2PageSize;
	}

	psPageArrayData->pasDevPAddr = OSAllocMem(sizeof(IMG_DEV_PHYADDR) *
						  psPageArrayData->uiNumAllocs);
	if (psPageArrayData->pasDevPAddr == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e1;
	}
	OSMemSet(psPageArrayData->pasDevPAddr, 0, sizeof(IMG_DEV_PHYADDR) *
		 psPageArrayData->uiNumAllocs);

	/* N.B.  We have a window of opportunity where a failure in
	   createPMR the finalize function can be called before the PMR
	   MALLOC and thus the hPDumpAllocInfo won't be set.  So we have
	   to conditionally call the PDumpFree function. */
	psPageArrayData->bPDumpMalloced = IMG_FALSE;

	psPageArrayData->bPoisonOnFree = bPoisonOnFree;

	for (i = 0; i < psPageArrayData->uiNumAllocs; i++) {
		bAllocResult = RA_Alloc(psDevNode->psLocalDevMemArena, psPageArrayData->uiAllocSize, 0,	/* No flags */
					1 << uiLog2PageSize, &uiCardAddr, &uiActualSize, IMG_NULL);	/* No private handle */

		if (!bAllocResult) {
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto e2;
		}

		psPageArrayData->pasDevPAddr[i].uiAddr = uiCardAddr;

		if (bPoisonOnAlloc) {
			eError = _PoisonAlloc(psDevNode,
					      &psPageArrayData->pasDevPAddr[i],
					      psPageArrayData->uiAllocSize,
					      _AllocPoison, _AllocPoisonSize);
			if (eError != PVRSRV_OK) {
				goto e2;
			}
		}

		if (bZero) {
			eError = _ZeroAlloc(psDevNode,
					    &psPageArrayData->pasDevPAddr[i],
					    psPageArrayData->uiAllocSize);
			if (eError != PVRSRV_OK) {
				goto e2;
			}
		}
	}

	*ppsPageArrayDataPtr = psPageArrayData;

	return PVRSRV_OK;

	/*
	   error exit paths follow:
	 */
 e2:
	for (i = i; i == 0; i--) {
		RA_Free(psDevNode->psLocalDevMemArena,
			psPageArrayData->pasDevPAddr[i].uiAddr);
	}
	OSFreeMem(psPageArrayData->pasDevPAddr);
 e1:
	OSFreeMem(psPageArrayData);
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static PVRSRV_ERROR _FreeLMPages(PMR_LMALLOCARRAY_DATA * psPageArrayData)
{
	IMG_UINT32 uiAllocSize;
	IMG_UINT32 i;

	uiAllocSize = psPageArrayData->uiAllocSize;

	for (i = 0; i < psPageArrayData->uiNumAllocs; i++) {
		if (psPageArrayData->bPoisonOnFree) {
			_PoisonAlloc(psPageArrayData->psDevNode,
				     &psPageArrayData->pasDevPAddr[i],
				     uiAllocSize, _FreePoison, _FreePoisonSize);
		}
		RA_Free(psPageArrayData->psDevNode->psLocalDevMemArena,
			psPageArrayData->pasDevPAddr[i].uiAddr);
	}

	OSFreeMem(psPageArrayData->pasDevPAddr);
	OSFreeMem(psPageArrayData);

	PVR_DPF((PVR_DBG_MESSAGE,
		 "physmem_lma.c: freed local memory for PMR @0x%p",
		 psPageArrayData));

	return PVRSRV_OK;
}

#if defined(PDUMP)
static IMG_VOID
_PDumpPMRMalloc(const PMR * psPMR,
		IMG_DEVMEM_SIZE_T uiSize,
		IMG_DEVMEM_ALIGN_T uiBlockSize,
		IMG_HANDLE * phPDumpAllocInfoPtr)
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
				       &uiOffset, &uiNextSymName);
	PVR_ASSERT(eError == PVRSRV_OK);
	PVR_ASSERT(uiOffset == 0);
	PVR_ASSERT((uiOffset + uiSize) <= uiNextSymName);

	PDumpPMRMalloc(aszMemspaceName,
		       aszSymbolicName,
		       uiSize, uiBlockSize, IMG_FALSE, &hPDumpAllocInfo);

	*phPDumpAllocInfoPtr = hPDumpAllocInfo;
}
#else				/* PDUMP */
static IMG_VOID
_PDumpPMRMalloc(const PMR * psPMR,
		IMG_DEVMEM_SIZE_T uiSize,
		IMG_DEVMEM_ALIGN_T uiBlockSize,
		IMG_HANDLE * phPDumpAllocInfoPtr)
{
	PVR_UNREFERENCED_PARAMETER(psPMR);
	PVR_UNREFERENCED_PARAMETER(uiSize);
	PVR_UNREFERENCED_PARAMETER(uiBlockSize);
	PVR_UNREFERENCED_PARAMETER(phPDumpAllocInfoPtr);
}
#endif				/* PDUMP */

/*
 *
 * Implementation of callback functions
 *
 */

/* destructor func is called after last reference disappears, but
   before PMR itself is freed. */
static PVRSRV_ERROR PMRFinalizeLocalMem(PMR_IMPL_PRIVDATA pvPriv)
{
	PVRSRV_ERROR eError;
	PMR_LMALLOCARRAY_DATA *psLMAllocArrayData = IMG_NULL;

	psLMAllocArrayData = pvPriv;

	/* Conditionally do the PDump free, because if CreatePMR failed we
	   won't have done the PDump MALLOC.  */
	if (psLMAllocArrayData->bPDumpMalloced) {
		PDumpPMRFree(psLMAllocArrayData->hPDumpAllocInfo);
	}

	/* This is unfortunately a little unbalanced.  We can't do the
	   "un-alloc pages" until now. */

	eError = _FreeLMPages(psLMAllocArrayData);
	PVR_ASSERT(eError == PVRSRV_OK);	/* can we do better? */

	return PVRSRV_OK;
}

/* callback function for locking the system physical page addresses.
   As we are LMA there is nothing to do as we control physical memory. */
static PVRSRV_ERROR
PMRLockSysPhysAddressesLocalMem(PMR_IMPL_PRIVDATA pvPriv,
				IMG_UINT32 uiLog2DevPageSize)
{
	PVR_UNREFERENCED_PARAMETER(pvPriv);
	PVR_UNREFERENCED_PARAMETER(uiLog2DevPageSize);

	return PVRSRV_OK;

}

static PVRSRV_ERROR PMRUnlockSysPhysAddressesLocalMem(PMR_IMPL_PRIVDATA pvPriv)
{
	PVR_UNREFERENCED_PARAMETER(pvPriv);

	return PVRSRV_OK;
}

/* N.B.  It is assumed that PMRLockSysPhysAddressesLocalMem() is called _before_ this function! */
static PVRSRV_ERROR
PMRSysPhysAddrLocalMem(PMR_IMPL_PRIVDATA pvPriv,
		       IMG_DEVMEM_OFFSET_T uiOffset,
		       IMG_DEV_PHYADDR * psDevPAddr)
{
	IMG_UINT32 uiLog2AllocSize;
	IMG_UINT32 uiNumAllocs;
	IMG_UINT32 uiAllocIndex;
	IMG_UINT32 uiInAllocOffset;
	PMR_LMALLOCARRAY_DATA *psLMAllocArrayData = IMG_NULL;

	psLMAllocArrayData = pvPriv;

	uiNumAllocs = psLMAllocArrayData->uiNumAllocs;
	if (uiNumAllocs > 1) {
		PVR_ASSERT(psLMAllocArrayData->uiLog2AllocSize != 0);
		uiLog2AllocSize = psLMAllocArrayData->uiLog2AllocSize;

		uiAllocIndex = uiOffset >> uiLog2AllocSize;
		uiInAllocOffset = uiOffset - (uiAllocIndex << uiLog2AllocSize);
		PVR_ASSERT(uiAllocIndex < uiNumAllocs);
		PVR_ASSERT(uiInAllocOffset < (1 << uiLog2AllocSize));

		psDevPAddr->uiAddr =
		    psLMAllocArrayData->pasDevPAddr[uiAllocIndex].uiAddr +
		    uiInAllocOffset;
	} else {
		psDevPAddr->uiAddr =
		    psLMAllocArrayData->pasDevPAddr[0].uiAddr + uiOffset;
	}

	return PVRSRV_OK;
}

static PVRSRV_ERROR
PMRAcquireKernelMappingDataLocalMem(PMR_IMPL_PRIVDATA pvPriv,
				    IMG_SIZE_T uiOffset,
				    IMG_SIZE_T uiSize,
				    IMG_VOID ** ppvKernelAddressOut,
				    IMG_HANDLE * phHandleOut,
				    PMR_FLAGS_T ulFlags)
{
	PVRSRV_ERROR eError;
	PMR_LMALLOCARRAY_DATA *psLMAllocArrayData = IMG_NULL;
	IMG_VOID *pvKernLinAddr = IMG_NULL;
	IMG_UINT32 ui32PageIndex = 0;

	PVR_UNREFERENCED_PARAMETER(ulFlags);

	psLMAllocArrayData = pvPriv;

	/* Check that we can map this in contiguously */
	if (psLMAllocArrayData->uiNumAllocs != 1) {
		IMG_SIZE_T uiStart = uiOffset;
		IMG_SIZE_T uiEnd = uiOffset + uiSize - 1;
		IMG_SIZE_T uiPageMask =
		    ~((1 << psLMAllocArrayData->uiLog2AllocSize) - 1);

		/* We can still map if only one page is required */
		if ((uiStart & uiPageMask) != (uiEnd & uiPageMask)) {
			eError = PVRSRV_ERROR_PMR_INCOMPATIBLE_CONTIGUITY;
			goto e0;
		}

		/* Locate the desired physical page to map in */
		ui32PageIndex = uiOffset >> psLMAllocArrayData->uiLog2AllocSize;
	}

	PVR_ASSERT(ui32PageIndex < psLMAllocArrayData->uiNumAllocs);

	eError = _MapAlloc(psLMAllocArrayData->psDevNode,
			   &psLMAllocArrayData->pasDevPAddr[ui32PageIndex],
			   psLMAllocArrayData->uiAllocSize, &pvKernLinAddr);

	*ppvKernelAddressOut =
	    ((IMG_CHAR *) pvKernLinAddr) +
	    (uiOffset & ((1U << psLMAllocArrayData->uiLog2AllocSize) - 1));
	*phHandleOut = pvKernLinAddr;

	return eError;

	/*
	   error exit paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static IMG_VOID PMRReleaseKernelMappingDataLocalMem(PMR_IMPL_PRIVDATA pvPriv,
						    IMG_HANDLE hHandle)
{
	PMR_LMALLOCARRAY_DATA *psLMAllocArrayData = IMG_NULL;
	IMG_VOID *pvKernLinAddr = IMG_NULL;

	psLMAllocArrayData = (PMR_LMALLOCARRAY_DATA *) pvPriv;
	pvKernLinAddr = (IMG_VOID *) hHandle;

	_UnMapAlloc(psLMAllocArrayData->psDevNode,
		    psLMAllocArrayData->uiAllocSize, pvKernLinAddr);
}

static PVRSRV_ERROR
PMRReadBytesLocalMem(PMR_IMPL_PRIVDATA pvPriv,
		     IMG_DEVMEM_OFFSET_T uiOffset,
		     IMG_UINT8 * pcBuffer,
		     IMG_SIZE_T uiBufSz, IMG_SIZE_T * puiNumBytes)
{
	PMR_LMALLOCARRAY_DATA *psLMAllocArrayData = IMG_NULL;
	IMG_SIZE_T uiBytesCopied;
	IMG_SIZE_T uiBytesToCopy;
	IMG_SIZE_T uiBytesCopyableFromAlloc;
	IMG_VOID *pvMapping = IMG_NULL;
	IMG_UINT8 *pcKernelPointer = IMG_NULL;
	IMG_UINT32 uiBufferOffset;
	IMG_UINT32 uiAllocIndex;
	IMG_UINT32 uiInAllocOffset;
	PVRSRV_ERROR eError;

	psLMAllocArrayData = pvPriv;

	uiBytesCopied = 0;
	uiBytesToCopy = uiBufSz;
	uiBufferOffset = 0;

	if (psLMAllocArrayData->uiNumAllocs > 1) {
		while (uiBytesToCopy > 0) {
			/* we have to map one alloc in at a time */
			PVR_ASSERT(psLMAllocArrayData->uiLog2AllocSize != 0);
			uiAllocIndex =
			    uiOffset >> psLMAllocArrayData->uiLog2AllocSize;
			uiInAllocOffset =
			    uiOffset -
			    (uiAllocIndex << psLMAllocArrayData->
			     uiLog2AllocSize);
			uiBytesCopyableFromAlloc = uiBytesToCopy;
			if (uiBytesCopyableFromAlloc + uiInAllocOffset >
			    (1 << psLMAllocArrayData->uiLog2AllocSize)) {
				uiBytesCopyableFromAlloc =
				    (1 << psLMAllocArrayData->uiLog2AllocSize) -
				    uiInAllocOffset;
			}

			PVR_ASSERT(uiBytesCopyableFromAlloc != 0);
			PVR_ASSERT(uiAllocIndex <
				   psLMAllocArrayData->uiNumAllocs);
			PVR_ASSERT(uiInAllocOffset <
				   (1 << psLMAllocArrayData->uiLog2AllocSize));

			eError = _MapAlloc(psLMAllocArrayData->psDevNode,
					   &psLMAllocArrayData->
					   pasDevPAddr[uiAllocIndex],
					   psLMAllocArrayData->uiAllocSize,
					   &pvMapping);
			if (eError != PVRSRV_OK) {
				goto e0;
			}
			pcKernelPointer = pvMapping;
			OSMemCopy(&pcBuffer[uiBufferOffset],
				  &pcKernelPointer[uiInAllocOffset],
				  uiBytesCopyableFromAlloc);
			_UnMapAlloc(psLMAllocArrayData->psDevNode,
				    psLMAllocArrayData->uiAllocSize, pvMapping);
			uiBufferOffset += uiBytesCopyableFromAlloc;
			uiBytesToCopy -= uiBytesCopyableFromAlloc;
			uiOffset += uiBytesCopyableFromAlloc;
			uiBytesCopied += uiBytesCopyableFromAlloc;
		}
	} else {
		PVR_ASSERT((uiOffset + uiBufSz) <=
			   psLMAllocArrayData->uiAllocSize);
		PVR_ASSERT(psLMAllocArrayData->uiAllocSize != 0);
		eError = _MapAlloc(psLMAllocArrayData->psDevNode,
				   &psLMAllocArrayData->pasDevPAddr[0],
				   psLMAllocArrayData->uiAllocSize, &pvMapping);
		if (eError != PVRSRV_OK) {
			goto e0;
		}
		pcKernelPointer = pvMapping;
		OSMemCopy(pcBuffer, &pcKernelPointer[uiOffset], uiBufSz);
		_UnMapAlloc(psLMAllocArrayData->psDevNode,
			    psLMAllocArrayData->uiAllocSize, pvMapping);
		uiBytesCopied = uiBufSz;
	}
	*puiNumBytes = uiBytesCopied;
	return PVRSRV_OK;
 e0:
	*puiNumBytes = uiBytesCopied;
	return eError;
}

static PMR_IMPL_FUNCTAB _sPMRLMAFuncTab = {
	/* pfnLockPhysAddresses */
	&PMRLockSysPhysAddressesLocalMem,
	/* pfnUnlockPhysAddresses */
	&PMRUnlockSysPhysAddressesLocalMem,
	/* pfnDevPhysAddr */
	&PMRSysPhysAddrLocalMem,
	/* pfnPDumpSymbolicAddr */
	IMG_NULL,
	/* pfnAcquireKernelMappingData */
	&PMRAcquireKernelMappingDataLocalMem,
	/* pfnReleaseKernelMappingData */
	&PMRReleaseKernelMappingDataLocalMem,
	/* pfnReadBytes */
	&PMRReadBytesLocalMem,
	/* pfnFinalize */
	&PMRFinalizeLocalMem
};

PVRSRV_ERROR
PhysmemNewLocalRamBackedPMR(PVRSRV_DEVICE_NODE * psDevNode,
			    IMG_DEVMEM_SIZE_T uiSize,
			    IMG_UINT32 uiLog2PageSize,
			    PVRSRV_MEMALLOCFLAGS_T uiFlags, PMR ** ppsPMRPtr)
{
	PVRSRV_ERROR eError;
	PVRSRV_ERROR eError2;
	PMR *psPMR = IMG_NULL;
	PMR_LMALLOCARRAY_DATA *psPrivData = IMG_NULL;
	IMG_HANDLE hPDumpAllocInfo = IMG_NULL;
	PMR_FLAGS_T uiPMRFlags;
	IMG_BOOL bZero;
	IMG_BOOL bPoisonOnAlloc;
	IMG_BOOL bPoisonOnFree;
	IMG_BOOL bContig;

	if (uiFlags & PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC) {
		bZero = IMG_TRUE;
	} else {
		bZero = IMG_FALSE;
	}

	if (uiFlags & PVRSRV_MEMALLOCFLAG_POISON_ON_ALLOC) {
		bPoisonOnAlloc = IMG_TRUE;
	} else {
		bPoisonOnAlloc = IMG_FALSE;
	}

	if (uiFlags & PVRSRV_MEMALLOCFLAG_POISON_ON_FREE) {
		bPoisonOnFree = IMG_TRUE;
	} else {
		bPoisonOnFree = IMG_FALSE;
	}

	if (uiFlags & PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE) {
		bContig = IMG_TRUE;
	} else {
		bContig = IMG_FALSE;
	}

	if ((uiFlags & PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC) &&
	    (uiFlags & PVRSRV_MEMALLOCFLAG_POISON_ON_ALLOC)) {
		/* Zero on Alloc and Poison on Alloc are mutually exclusive */
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	/* Silently round up alignment/pagesize if request was less that
	   PAGE_SHIFT, because it would never be harmful for memory to be
	   _more_ contiguous that was desired */

	uiLog2PageSize = OSGetPageShift() > uiLog2PageSize ? OSGetPageShift()
	    : uiLog2PageSize;

	eError = _AllocLMPages(psDevNode,
			       uiSize,
			       uiLog2PageSize,
			       bZero,
			       bPoisonOnAlloc,
			       bPoisonOnFree, bContig, &psPrivData);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	/* In this instance, we simply pass flags straight through.

	   Generically, uiFlags can include things that control the PMR
	   factory, but we don't need any such thing (at the time of
	   writing!), and our caller specifies all PMR flags so we don't
	   need to meddle with what was given to us.
	 */
	uiPMRFlags =
	    (PMR_FLAGS_T) (uiFlags & PVRSRV_MEMALLOCFLAGS_PMRFLAGSMASK);
	/* check no significant bits were lost in cast due to different
	   bit widths for flags */
	PVR_ASSERT(uiPMRFlags == (uiFlags & PVRSRV_MEMALLOCFLAGS_PMRFLAGSMASK));

	eError = PMRCreatePMR(psDevNode->psPhysHeap,
			      uiSize,
			      uiLog2PageSize,
			      uiPMRFlags,
			      "PMRLMA", &_sPMRLMAFuncTab, psPrivData, &psPMR);
	if (eError != PVRSRV_OK) {
		goto e1;
	}

	_PDumpPMRMalloc(psPMR, uiSize,
			/* alignment is alignment of start of buffer _and_
			   minimum contiguity - i.e. smallest allowable
			   page-size.  FIXME: review this decision. */
			1U << uiLog2PageSize, &hPDumpAllocInfo);
	psPrivData->hPDumpAllocInfo = hPDumpAllocInfo;
	psPrivData->bPDumpMalloced = IMG_TRUE;
	psPrivData->psDevNode = psDevNode;

	*ppsPMRPtr = psPMR;
	return PVRSRV_OK;

 e1:
	eError2 = _FreeLMPages(psPrivData);
	PVR_ASSERT(eError2 == PVRSRV_OK);

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}
