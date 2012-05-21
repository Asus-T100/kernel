/*!****************************************************************************
@File		physmem_osmem.c

@Title		Implementation of PMR functions for OS managed memory

@Author		Imagination Technologies

@Copyright	Copyright 2010 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either
                material or conceptual may be copied or distributed,
                transmitted, transcribed, stored in a retrieval system
                or translated into any human or computer language in any
                form by any means, electronic, mechanical, manual or
                other-wise, or disclosed to third parties without the
                express written permission of Imagination Technologies
                Limited, Unit 8, HomePark Industrial Estate,
                King's Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform	generic

@Description	Part of the memory management.  This module is responsible for
                implementing the function callbacks for physical memory borrowed
                from that normally managed by the operating system.

@DoxygenVer

******************************************************************************/

/* include5/ */
#include "img_types.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"
#include "pvrsrv_memallocflags.h"

/* services/server/include/ */
#include "osfunc.h"
#include "pdump_physmem.h"
#include "pmr.h"
#include "pmr_impl.h"

/* ourselves */
#include "physmem_osmem.h"

#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/mm_types.h>
#include <linux/vmalloc.h>
#include "linux/gfp.h"
#include "asm/io.h"
#if defined(CONFIG_X86)
#include <asm/cacheflush.h>
#endif

struct _PMR_OSPAGEARRAY_DATA_ {
	/*
	   uiNumPages:

	   number of "pages" (a.k.a. macro pages, compound pages, higher
	   order pages, etc...)
	 */
	IMG_UINT32 uiNumPages;

	/*
	   uiLog2PageSize;

	   size of each "page" -- this would normally be the same as
	   PAGE_SHIFT, but we support the idea that we may allocate pages
	   in larger chunks for better contiguity, using order>0 in the
	   call to alloc_pages()
	 */
	IMG_UINT32 uiLog2PageSize;

	/*
	   the pages thusly allocated...  N.B.. One entry per compound page,
	   where compound pages are used.
	 */
	struct page **pagearray;

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

	/*
	   The cache mode of the PMR (required at free time)
	 */
	IMG_BOOL bUnsetMemoryType;
};

static IMG_VOID
_PoisonPages(struct page *page,
	     IMG_UINT32 uiOrder,
	     const IMG_CHAR * pacPoisonData, IMG_SIZE_T uiPoisonSize)
{
	void *kvaddr;
	IMG_UINT32 uiSrcByteIndex;
	IMG_UINT32 uiDestByteIndex;
	IMG_UINT32 uiSubPageIndex;
	IMG_CHAR *pcDest;

	uiSrcByteIndex = 0;
	for (uiSubPageIndex = 0; uiSubPageIndex < (1U << uiOrder);
	     uiSubPageIndex++) {
		kvaddr = kmap(page + uiSubPageIndex);

		pcDest = kvaddr;

		for (uiDestByteIndex = 0; uiDestByteIndex < PAGE_SIZE;
		     uiDestByteIndex++) {
			pcDest[uiDestByteIndex] = pacPoisonData[uiSrcByteIndex];
			uiSrcByteIndex++;
			if (uiSrcByteIndex == uiPoisonSize) {
				uiSrcByteIndex = 0;
			}
		}
		kunmap(page + uiSubPageIndex);
	}
}

static const IMG_CHAR _AllocPoison[] = "^PoIsOn";
static const IMG_UINT32 _AllocPoisonSize = 7;
static const IMG_CHAR _FreePoison[] = "<DEAD-BEEF>";
static const IMG_UINT32 _FreePoisonSize = 11;

static PVRSRV_ERROR
_AllocOSPages(PMR_SIZE_T uiSize,
	      IMG_UINT32 uiLog2PageSize,
	      IMG_BOOL bZero,
	      IMG_BOOL bPoisonOnAlloc,
	      IMG_BOOL bPoisonOnFree,
	      IMG_UINT32 ui32CacheMode,
	      struct _PMR_OSPAGEARRAY_DATA_ **ppsPageArrayDataPtr)
{
	/* Allocate a bunch of physical memory.  Must be whole number of
	   pages worth */

	PVRSRV_ERROR eError;
	IMG_VOID *pvData;
	IMG_UINT32 uiNumPages;
	IMG_UINT32 uiOrder;
	IMG_UINT32 uiPageIndex;

	struct page **ppsPageArray;
	struct _PMR_OSPAGEARRAY_DATA_ *psPageArrayData;

	unsigned int gfp_flags;
#if defined (CONFIG_X86)
	IMG_PVOID pvPageVAddr;
#endif
	if (uiSize >= 0x1000000000ULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "physmem_osmem_linux.c: Do you really want 64GB of physical memory in one go?  This is likely a bug"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e_freed_pvdata;
	}

	PVR_ASSERT(PAGE_SHIFT <= uiLog2PageSize);

	if ((uiSize & ((1ULL << uiLog2PageSize) - 1)) != 0) {
		eError = PVRSRV_ERROR_PMR_NOT_PAGE_MULTIPLE;
		goto e_freed_pvdata;
	}

	uiOrder = uiLog2PageSize - PAGE_SHIFT;
	/* Use of cast below is justified by the assertion that follows to
	   prove that no significant bits have been truncated */
	uiNumPages = (IMG_UINT32) (((uiSize - 1) >> uiLog2PageSize) + 1);
	PVR_ASSERT((uiNumPages << uiLog2PageSize) == uiSize);

	/* todo: get rid of o/s specific hacks here */
	pvData = kmalloc(sizeof(struct _PMR_OSPAGEARRAY_DATA_) +
			 sizeof(struct page *) * uiNumPages, GFP_KERNEL);
	if (pvData == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "physmem_osmem_linux.c: OS refused the memory allocation for the table of pages.  Did you ask for too much?"));
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e_freed_pvdata;
	}
	PVR_ASSERT(pvData != IMG_NULL);

	psPageArrayData = pvData;
	ppsPageArray = pvData + sizeof(struct _PMR_OSPAGEARRAY_DATA_);
	psPageArrayData->pagearray = ppsPageArray;
	psPageArrayData->uiLog2PageSize = uiLog2PageSize;
	psPageArrayData->uiNumPages = uiNumPages;

	/* N.B.  We have a window of opportunity where a failure in
	   createPMR the finalize function can be called before the PMR
	   MALLOC and thus the hPDumpAllocInfo won't be set.  So we have
	   to conditionally call the PDumpFree function. */
	psPageArrayData->bPDumpMalloced = IMG_FALSE;

	psPageArrayData->bPoisonOnFree = bPoisonOnFree;
	psPageArrayData->bUnsetMemoryType = IMG_FALSE;

	gfp_flags = GFP_KERNEL;

	if (bZero) {
		gfp_flags |= __GFP_ZERO;
	}

	/* Allocate pages one at a time.  Note that the _device_ memory
	   page size may be different from the _host_ cpu page size - we
	   have a concept of a minimum contiguity requirement, which must
	   be sufficient to meet the requirement of both device and host
	   page size (and possibly other devices or other external
	   constraints).  We are allocating ONE "minimum contiguity unit"
	   (in practice, generally a _device_ page, but not necessarily)
	   at a time, by asking the OS for 2**uiOrder _host_ pages at a
	   time. */
	for (uiPageIndex = 0; uiPageIndex < uiNumPages; uiPageIndex++) {
		/* TODO: Compound page stuff hasn't had a
		   great deal of testing.  Remove this check once we know it
		   works, or if you're feeling brave */
		PVR_ASSERT(uiOrder == 0);

		ppsPageArray[uiPageIndex] = alloc_pages(gfp_flags, uiOrder);
#if defined (CONFIG_X86)
		if (ppsPageArray[uiPageIndex] != IMG_NULL) {
			/*
			   On X86 if we already have a mapping we need to change the mode of 
			   current mapping before we map it ourselves
			 */
			pvPageVAddr = page_address(ppsPageArray[uiPageIndex]);
			if (pvPageVAddr != NULL) {
				int ret;
				switch (ui32CacheMode &
					PVRSRV_MEMALLOCFLAG_CPU_CACHE_MODE_MASK)
				{
					/* FIXME: What do we do for cache coherent? For now make uncached */
				case PVRSRV_MEMALLOCFLAG_CPU_CACHE_COHERENT:
				case PVRSRV_MEMALLOCFLAG_CPU_UNCACHED:
					ret =
					    set_memory_uc((unsigned long)
							  pvPageVAddr, 1);
					if (ret) {
						eError =
						    PVRSRV_ERROR_UNABLE_TO_SET_CACHE_MODE;
						__free_pages(ppsPageArray
							     [uiPageIndex],
							     uiOrder);
						ppsPageArray[uiPageIndex] =
						    IMG_NULL;
					}
					psPageArrayData->bUnsetMemoryType =
					    IMG_TRUE;
					break;

				case PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE:
					ret =
					    set_memory_wc((unsigned long)
							  pvPageVAddr, 1);
					if (ret) {
						eError =
						    PVRSRV_ERROR_UNABLE_TO_SET_CACHE_MODE;
						__free_pages(ppsPageArray
							     [uiPageIndex],
							     uiOrder);
						ppsPageArray[uiPageIndex] =
						    IMG_NULL;
					}
					psPageArrayData->bUnsetMemoryType =
					    IMG_TRUE;
					break;

				case PVRSRV_MEMALLOCFLAG_CPU_CACHE_INCOHERENT:
					break;

				default:
					break;
				}
			}
		}
#endif
		if (ppsPageArray[uiPageIndex] == IMG_NULL) {
			PVR_DPF((PVR_DBG_ERROR,
				 "physmem_osmem_linux.c: alloc_pages failed to honour request at %d of %d",
				 uiPageIndex, uiNumPages));
			for (--uiPageIndex; uiPageIndex < uiNumPages;
			     --uiPageIndex) {
#if defined(CONFIG_X86)
				pvPageVAddr =
				    page_address(ppsPageArray[uiPageIndex]);
				if (psPageArrayData->bUnsetMemoryType ==
				    IMG_TRUE) {
					int ret;

					ret =
					    set_memory_wb((unsigned long)
							  pvPageVAddr, 1);
					if (ret) {
						PVR_DPF((PVR_DBG_ERROR,
							 "%s: Failed to reset page attribute",
							 __FUNCTION__));
					}
				}
#endif
				__free_pages(ppsPageArray[uiPageIndex],
					     uiOrder);
			}
			eError = PVRSRV_ERROR_PMR_FAILED_TO_ALLOC_PAGES;
			goto e_freed_pages;
		}

		/* Can't ask us to zero it and poison it */
		PVR_ASSERT(!bZero || !bPoisonOnAlloc);

		if (bPoisonOnAlloc) {
			_PoisonPages(ppsPageArray[uiPageIndex],
				     uiOrder, _AllocPoison, _AllocPoisonSize);
		}
	}

	PVR_DPF((PVR_DBG_MESSAGE,
		 "physmem_osmem_linux.c: allocated OS memory for PMR @0x%p",
		 psPageArrayData));

	*ppsPageArrayDataPtr = psPageArrayData;

	return PVRSRV_OK;

	/*
	   error exit paths follow:
	 */
 e_freed_pages:
	kfree(pvData);

 e_freed_pvdata:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static PVRSRV_ERROR _FreeOSPages(struct _PMR_OSPAGEARRAY_DATA_ *psPageArrayData)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 uiNumPages;
	IMG_UINT32 uiOrder;
	IMG_UINT32 uiPageIndex;
	struct page **ppsPageArray;
#if defined (CONFIG_X86)
	IMG_PVOID pvPageVAddr;
#endif

	ppsPageArray = psPageArrayData->pagearray;

	uiNumPages = psPageArrayData->uiNumPages;

	uiOrder = psPageArrayData->uiLog2PageSize - PAGE_SHIFT;

	for (uiPageIndex = 0; uiPageIndex < uiNumPages; uiPageIndex++) {
		if (psPageArrayData->bPoisonOnFree) {
			_PoisonPages(ppsPageArray[uiPageIndex],
				     uiOrder, _FreePoison, _FreePoisonSize);
		}
#if defined(CONFIG_X86)
		if (psPageArrayData->bUnsetMemoryType == IMG_TRUE) {
			int ret;

			pvPageVAddr = page_address(ppsPageArray[uiPageIndex]);
			ret = set_memory_wb((unsigned long)pvPageVAddr, 1);
			if (ret) {
				PVR_DPF((PVR_DBG_ERROR,
					 "%s: Failed to reset page attribute",
					 __FUNCTION__));
			}
		}
#endif
		__free_pages(ppsPageArray[uiPageIndex], uiOrder);
	}

	kfree(psPageArrayData);

	PVR_DPF((PVR_DBG_MESSAGE,
		 "physmem_osmem_linux.c: freed OS memory for PMR @0x%p",
		 psPageArrayData));

	eError = PVRSRV_OK;

	return eError;
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
static PVRSRV_ERROR PMRFinalizeOSMem(PMR_IMPL_PRIVDATA pvPriv
				     //struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData
    )
{
	PVRSRV_ERROR eError;
	struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData;

	psOSPageArrayData = pvPriv;

	/* Conditionally do the PDump free, because if CreatePMR failed we
	   won't have done the PDump MALLOC.  */
	if (psOSPageArrayData->bPDumpMalloced) {
		PDumpPMRFree(psOSPageArrayData->hPDumpAllocInfo);
	}

	/* This is unfortunately a little unbalanced.  We can't do the
	   "un-alloc pages" until now. */

	eError = _FreeOSPages(psOSPageArrayData);
	PVR_ASSERT(eError == PVRSRV_OK);	/* can we do better? */

	return PVRSRV_OK;
}

/* callback function for locking the system physical page addresses.
   This function must be called before the lookup address func. */
static PVRSRV_ERROR PMRLockSysPhysAddressesOSMem(PMR_IMPL_PRIVDATA pvPriv,
						 // struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData,
						 IMG_UINT32 uiLog2DevPageSize)
{
	PVRSRV_ERROR eError;
	struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData;

	psOSPageArrayData = pvPriv;

	/* Physical page addresses are already locked down in this
	   implementation, so there is no need to acquire physical
	   addresses.  We do need to verify that the physical contiguity
	   requested by the caller (i.e. page size of the device they
	   intend to map this memory into) is compatible with (i.e. not of
	   coarser granularity than) our already known physicial
	   contiguity of the pages */
	if (uiLog2DevPageSize > psOSPageArrayData->uiLog2PageSize) {
		/* or NOT_MAPPABLE_TO_THIS_PAGE_SIZE ? */
		eError = PVRSRV_ERROR_PMR_INCOMPATIBLE_CONTIGUITY;
		return eError;
	}

	eError = PVRSRV_OK;
	return eError;

}

static PVRSRV_ERROR PMRUnlockSysPhysAddressesOSMem(PMR_IMPL_PRIVDATA pvPriv
						   //struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData
    )
{
	/* Just drops the refcount. */

	PVRSRV_ERROR eError;
	struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData;

	psOSPageArrayData = pvPriv;
	eError = PVRSRV_OK;

	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

/* N.B.  It is assumed that PMRLockSysPhysAddressesOSMem() is called _before_ this function! */
static PVRSRV_ERROR PMRSysPhysAddrOSMem(PMR_IMPL_PRIVDATA pvPriv,
					//const struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData,
					IMG_DEVMEM_OFFSET_T uiOffset,
					IMG_DEV_PHYADDR * psDevPAddr)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 uiPageSize;
	IMG_UINT32 uiNumPages;
	IMG_UINT32 uiPageIndex;
	IMG_UINT32 uiInPageOffset;
	struct page **ppsPageArray;
	const struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData;

	psOSPageArrayData = pvPriv;
	ppsPageArray = psOSPageArrayData->pagearray;

	uiNumPages = psOSPageArrayData->uiNumPages;
	uiPageSize = 1U << psOSPageArrayData->uiLog2PageSize;

	uiPageIndex = uiOffset >> psOSPageArrayData->uiLog2PageSize;
	uiInPageOffset =
	    uiOffset - (uiPageIndex << psOSPageArrayData->uiLog2PageSize);
	PVR_ASSERT(uiPageIndex < uiNumPages);
	PVR_ASSERT(uiInPageOffset < uiPageSize);

	psDevPAddr->uiAddr =
	    page_to_phys(ppsPageArray[uiPageIndex]) + uiInPageOffset;

	eError = PVRSRV_OK;

	return eError;
}

static PVRSRV_ERROR
PMRAcquireKernelMappingDataOSMem(PMR_IMPL_PRIVDATA pvPriv,
				 IMG_SIZE_T uiOffset,
				 IMG_SIZE_T uiSize,
				 IMG_VOID ** ppvKernelAddressOut,
				 IMG_HANDLE * phHandleOut, PMR_FLAGS_T ulFlags)
{
	PVRSRV_ERROR eError;
	struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData;
	IMG_VOID *pvAddress;
	pgprot_t prot = PAGE_KERNEL;

	psOSPageArrayData = pvPriv;

	if (psOSPageArrayData->uiLog2PageSize != PAGE_SHIFT) {
		/* we only know how to use vmap on allocations comprising
		   individual pages.  Higher-order "pages" are not supported
		   with this. */
		eError = PVRSRV_ERROR_PMR_INCOMPATIBLE_CONTIGUITY;
		goto e0;
	}

	switch (ulFlags & PVRSRV_MEMALLOCFLAG_CPU_CACHE_MODE_MASK) {
		/* FIXME: What do we do for cache coherent? For now make uncached */
	case PVRSRV_MEMALLOCFLAG_CPU_CACHE_COHERENT:
	case PVRSRV_MEMALLOCFLAG_CPU_UNCACHED:
		prot = PAGE_KERNEL_UC_MINUS;
		break;

	case PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE:
		prot = PAGE_KERNEL_WC;
		break;

	case PVRSRV_MEMALLOCFLAG_CPU_CACHE_INCOHERENT:
		break;

	default:
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}
	pvAddress = vm_map_ram(psOSPageArrayData->pagearray,
			       psOSPageArrayData->uiNumPages, -1, prot);

	*ppvKernelAddressOut = pvAddress + uiOffset;
	*phHandleOut = pvAddress;

	return PVRSRV_OK;

	/*
	   error exit paths follow
	 */

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

static IMG_VOID PMRReleaseKernelMappingDataOSMem(PMR_IMPL_PRIVDATA pvPriv,
						 IMG_HANDLE hHandle)
{
	struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData;

	psOSPageArrayData = pvPriv;
	vm_unmap_ram(hHandle, psOSPageArrayData->uiNumPages);
}

static PVRSRV_ERROR
PMRReadBytesOSMem(PMR_IMPL_PRIVDATA pvPriv,
		  IMG_DEVMEM_OFFSET_T uiOffset,
		  IMG_UINT8 * pcBuffer,
		  IMG_SIZE_T uiBufSz, IMG_SIZE_T * puiNumBytes)
{
	struct _PMR_OSPAGEARRAY_DATA_ *psOSPageArrayData;
	IMG_SIZE_T uiBytesCopied;
	IMG_SIZE_T uiBytesToCopy;
	IMG_SIZE_T uiBytesCopyableFromPage;
	IMG_VOID *pvMapping;
	const IMG_UINT8 *pcKernelPointer;
	IMG_UINT32 uiBufferOffset;
	IMG_UINT32 uiPageIndex;
	IMG_UINT32 uiInPageOffset;

	psOSPageArrayData = pvPriv;

	uiBytesCopied = 0;
	uiBytesToCopy = uiBufSz;
	uiBufferOffset = 0;

	while (uiBytesToCopy > 0) {
		/* we have to kmap one page in at a time */
		uiPageIndex = uiOffset >> psOSPageArrayData->uiLog2PageSize;

		/* this code is untested (i.e. doesn't work!) with "macro pages" (or whatever they're called) */
		PVR_ASSERT(psOSPageArrayData->uiLog2PageSize == PAGE_SHIFT);

		uiInPageOffset =
		    uiOffset -
		    (uiPageIndex << psOSPageArrayData->uiLog2PageSize);
		uiBytesCopyableFromPage = uiBytesToCopy;
		if (uiBytesCopyableFromPage + uiInPageOffset >
		    (1 << psOSPageArrayData->uiLog2PageSize)) {
			uiBytesCopyableFromPage =
			    (1 << psOSPageArrayData->uiLog2PageSize) -
			    uiInPageOffset;
		}
		pvMapping = kmap(psOSPageArrayData->pagearray[uiPageIndex]);
		PVR_ASSERT(pvMapping != IMG_NULL);
		pcKernelPointer = pvMapping;
		memcpy(&pcBuffer[uiBufferOffset],
		       &pcKernelPointer[uiInPageOffset],
		       uiBytesCopyableFromPage);
		kunmap(psOSPageArrayData->pagearray[uiPageIndex]);

		uiBufferOffset += uiBytesCopyableFromPage;
		uiBytesToCopy -= uiBytesCopyableFromPage;
		uiOffset += uiBytesCopyableFromPage;
		uiBytesCopied += uiBytesCopyableFromPage;
	}

	*puiNumBytes = uiBytesCopied;
	return PVRSRV_OK;
}

static PMR_IMPL_FUNCTAB _sPMROSPFuncTab = {
	.pfnLockPhysAddresses = &PMRLockSysPhysAddressesOSMem,
	.pfnUnlockPhysAddresses = &PMRUnlockSysPhysAddressesOSMem,
	.pfnDevPhysAddr = &PMRSysPhysAddrOSMem,
	.pfnAcquireKernelMappingData = &PMRAcquireKernelMappingDataOSMem,
	.pfnReleaseKernelMappingData = &PMRReleaseKernelMappingDataOSMem,
	.pfnReadBytes = &PMRReadBytesOSMem,
	.pfnFinalize = &PMRFinalizeOSMem
};

static PVRSRV_ERROR
_NewOSAllocPagesPMR(PVRSRV_DEVICE_NODE * psDevNode,
		    IMG_DEVMEM_SIZE_T uiSize,
		    IMG_UINT32 uiLog2PageSize,
		    PVRSRV_MEMALLOCFLAGS_T uiFlags, PMR ** ppsPMRPtr)
{
	PVRSRV_ERROR eError;
	PVRSRV_ERROR eError2;
	PMR *psPMR;
	struct _PMR_OSPAGEARRAY_DATA_ *psPrivData;
	IMG_HANDLE hPDumpAllocInfo = IMG_NULL;
	PMR_FLAGS_T uiPMRFlags;
	IMG_BOOL bZero;
	IMG_BOOL bPoisonOnAlloc;
	IMG_BOOL bPoisonOnFree;
	IMG_UINT32 ui32CacheMode =
	    (IMG_UINT32) (uiFlags & PVRSRV_MEMALLOCFLAG_CPU_CACHE_MODE_MASK);

	PVR_ASSERT(ui32CacheMode ==
		   (uiFlags & PVRSRV_MEMALLOCFLAG_CPU_CACHE_MODE_MASK));

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

	if ((uiFlags & PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC) &&
	    (uiFlags & PVRSRV_MEMALLOCFLAG_POISON_ON_ALLOC)) {
		/* Zero on Alloc and Poison on Alloc are mutually exclusive */
		eError = PVRSRV_ERROR_INVALID_PARAMS;
		goto e0;
	}

	/* Silently round up alignment/pagesize if request was less that
	   PAGE_SHIFT, because it would never be harmful for memory to be
	   _more_ contiguous that was desired */

	uiLog2PageSize = PAGE_SHIFT > uiLog2PageSize
	    ? PAGE_SHIFT : uiLog2PageSize;

	eError = _AllocOSPages(uiSize,
			       uiLog2PageSize,
			       bZero,
			       bPoisonOnAlloc,
			       bPoisonOnFree, ui32CacheMode, &psPrivData);
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
			      "PMROSAP", &_sPMROSPFuncTab, psPrivData, &psPMR);
	if (eError != PVRSRV_OK) {
		goto e1;
	}

	_PDumpPMRMalloc(psPMR, uiSize,
			/* alignment is alignment of start of buffer _and_
			   minimum contiguity - i.e. smallest allowable
			   page-size.  TODO: review this decision. */
			1U << uiLog2PageSize, &hPDumpAllocInfo);
	psPrivData->hPDumpAllocInfo = hPDumpAllocInfo;
	psPrivData->bPDumpMalloced = IMG_TRUE;

	*ppsPMRPtr = psPMR;
	return PVRSRV_OK;

 e1:
	eError2 = _FreeOSPages(psPrivData);
	PVR_ASSERT(eError2 == PVRSRV_OK);

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR
PhysmemNewOSRamBackedPMR(PVRSRV_DEVICE_NODE * psDevNode,
			 IMG_DEVMEM_SIZE_T uiSize,
			 IMG_UINT32 uiLog2PageSize,
			 PVRSRV_MEMALLOCFLAGS_T uiFlags, PMR ** ppsPMRPtr)
{
	return _NewOSAllocPagesPMR(psDevNode,
				   uiSize, uiLog2PageSize, uiFlags, ppsPMRPtr);
}
