/******************************************************************************
 * Name         : pdump_mmu.h
 * Title        : common MMU Management
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
 * Description  : Implements basic low level control of MMU.
 *
 * Platform     : ALL
 *
 *****************************************************************************/

#ifndef SRVKM_PDUMP_MMU_H
#define SRVKM_PDUMP_MMU_H

/* services/server/include/ */
#include "pdump_symbolicaddr.h"
/* include/ */
#include "img_types.h"
#include "pvrsrv_error.h"

/*
	PDUMP MMU atttributes
*/
typedef struct _PDUMP_MMU_ATTRIB_DEVICE_ {
	/* Per-Device Pdump attribs */

	/*!< Pdump memory bank name */
	IMG_CHAR *pszPDumpMemDevName;

	/*!< Pdump register bank name */
	IMG_CHAR *pszPDumpRegDevName;

	//??        IMG_CHAR        *pszPDRegRegion;
} PDUMP_MMU_ATTRIB_DEVICE;

typedef struct _PDUMP_MMU_ATTRIB_CONTEXT_ {
	IMG_UINT32 ui32Dummy;
} PDUMP_MMU_ATTRIB_CONTEXT;

typedef struct _PDUMP_MMU_ATTRIB_HEAP_ {
	/* data page info */
	IMG_UINT32 ui32DataPageMask;

/* 	/\* page table info *\/ */
/* 	IMG_UINT32 ui32PTEValid; */
/* 	IMG_UINT32 ui32PTSize; */
/* 	IMG_UINT32 ui32PTEAlignShift; */

/* 	/\* page directory info *\/ */
/* 	IMG_UINT32 ui32PDEMask; */
/* 	IMG_UINT32 ui32PDEAlignShift; */
} PDUMP_MMU_ATTRIB_HEAP;

typedef struct _PDUMP_MMU_ATTRIB_ {
	/* TODO: would these be better as pointers rather than copies? */
	struct _PDUMP_MMU_ATTRIB_DEVICE_ sDevice;
	struct _PDUMP_MMU_ATTRIB_CONTEXT_ sContext;
	struct _PDUMP_MMU_ATTRIB_HEAP_ sHeap;
} PDUMP_MMU_ATTRIB;

#if defined(PDUMP)
extern PVRSRV_ERROR PDumpMMUMalloc(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
				   IMG_DEV_PHYADDR * psDevPAddr,
				   IMG_UINT32 ui32Size, IMG_UINT32 ui32Align);

extern PVRSRV_ERROR PDumpMMUFree(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
				 IMG_DEV_PHYADDR * psDevPAddr);

extern PVRSRV_ERROR PDumpMMUMalloc2(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
				    const IMG_CHAR * pszSymbolicAddr,
				    IMG_UINT32 ui32Size, IMG_UINT32 ui32Align);

extern PVRSRV_ERROR PDumpMMUFree2(const IMG_CHAR * pszPDumpDevName, const IMG_CHAR * pszTableType,	/* PAGE_CATALOGUE, PAGE_DIRECTORY, PAGE_TABLE */
				  const IMG_CHAR * pszSymbolicAddr);

extern PVRSRV_ERROR PDumpMMUDumpPCEntries(const IMG_CHAR * pszPDumpDevName,
					  IMG_VOID * pvPCMem,
					  IMG_DEV_PHYADDR sPCDevPAddr,
					  IMG_UINT32 uiFirstEntry,
					  IMG_UINT32 uiNumEntries,
					  IMG_UINT32 uiBytesPerEntry,
					  IMG_UINT32 uiPDAddrAlignShift,
					  IMG_UINT32 uiPDAddrShift,
					  IMG_UINT64 uiPDAddrMask,
					  IMG_UINT64 uiPCEProtMask,
					  IMG_UINT32 ui32Flags);

extern PVRSRV_ERROR PDumpMMUDumpPDEntries(const IMG_CHAR * pszPDumpDevName,
					  IMG_VOID * pvPDMem,
					  IMG_DEV_PHYADDR sPDDevPAddr,
					  IMG_UINT32 uiFirstEntry,
					  IMG_UINT32 uiNumEntries,
					  IMG_UINT32 uiBytesPerEntry,
					  IMG_UINT32 uiPTAddrAlignShift,
					  IMG_UINT32 uiPTAddrShift,
					  IMG_UINT64 uiPTAddrMask,
					  IMG_UINT64 uiPDEProtMask,
					  IMG_UINT32 ui32Flags);

extern PVRSRV_ERROR PDumpMMUDumpPTEntries(const IMG_CHAR * pszPDumpDevName,
					  IMG_VOID * pvPTMem,
					  IMG_DEV_PHYADDR sPTDevPAddr,
					  IMG_UINT32 uiFirstEntry,
					  IMG_UINT32 uiNumEntries,
					  const IMG_CHAR * pszMemspaceName,
					  const IMG_CHAR * pszSymbolicAddr,
					  IMG_UINT64 uiSymbolicAddrOffset,
					  IMG_UINT32 uiBytesPerEntry,
					  IMG_UINT32 uiDPAddrAlignShift,
					  IMG_UINT32 uiDPAddrShift,
					  IMG_UINT64 uiDPAddrMask,
					  IMG_UINT64 uiPTEProtMask,
					  IMG_UINT32 ui32Flags);

extern PVRSRV_ERROR PDumpMMUAllocMMUContext(const IMG_CHAR *
					    pszPDumpMemSpaceName,
					    IMG_DEV_PHYADDR sPCDevPAddr,
					    IMG_UINT32 * pui32MMUContextID);

extern PVRSRV_ERROR PDumpMMUFreeMMUContext(const IMG_CHAR *
					   pszPDumpMemSpaceName,
					   IMG_UINT32 ui32MMUContextID);

extern PVRSRV_ERROR PDumpMMUActivateCatalog(const IMG_CHAR *
					    pszPDumpRegSpaceName,
					    const IMG_CHAR * pszPDumpRegName,
					    IMG_UINT32 uiRegAddr,
					    const IMG_CHAR *
					    pszPDumpPCSymbolicName);

/* TODO: split to separate file... (debatable whether this is anything to do with MMU) */
extern PVRSRV_ERROR
PDumpMMUSAB(const IMG_CHAR * pszPDumpMemNamespace,
	    IMG_UINT32 uiPDumpMMUCtx,
	    IMG_DEV_VIRTADDR sDevAddrStart,
	    IMG_DEVMEM_SIZE_T uiSize,
	    const IMG_CHAR * pszFilename,
	    IMG_UINT32 uiFileOffset, IMG_UINT32 ui32PDumpFlags);

#define PDUMP_MMU_MALLOC_PC(pszPDumpMemDevName, pszDevPAddr, ui32Size, ui32Align) \
        PDumpMMUMalloc(pszPDumpMemDevName, "PAGE_CATALOGUE", pszDevPAddr, ui32Size, ui32Align)
#define PDUMP_MMU_DUMP_PC_ENTRIES(pszPDumpMemDevName, pvPCMem, sPCDevPAddr, uiFirstEntry, uiNumEntries, uiBytesPerEntry, uiPDAddrAlignShift, uiPDAddrShift, uiPDAddrMask, uiPCEProtMask, ui32Flags) \
        PDumpMMUDumpPCEntries(pszPDumpMemDevName, pvPCMem, sPCDevPAddr, uiFirstEntry, uiNumEntries, uiBytesPerEntry, uiPDAddrAlignShift, uiPDAddrShift, uiPDAddrMask, uiPCEProtMask, ui32Flags)
#define PDUMP_MMU_FREE_PC(pszPDumpMemDevName, psDevPAddr) \
        PDumpMMUFree(pszPDumpMemDevName, "PAGE_CATALOGUE", psDevPAddr)
#define PDUMP_MMU_MALLOC_PD(pszPDumpMemDevName, pszDevPAddr, ui32Size, ui32Align) \
        PDumpMMUMalloc(pszPDumpMemDevName, "PAGE_DIRECTORY", pszDevPAddr, ui32Size, ui32Align)
#define PDUMP_MMU_DUMP_PD_ENTRIES(pszPDumpMemDevName, pvPDMem, sPDDevPAddr, uiFirstEntry, uiNumEntries, uiBytesPerEntry, uiPTAddrAlignShift, uiPTAddrShift, uiPTAddrMask, uiPDEProtMask, ui32Flags) \
        PDumpMMUDumpPDEntries(pszPDumpMemDevName, pvPDMem, sPDDevPAddr, uiFirstEntry, uiNumEntries, uiBytesPerEntry, uiPTAddrAlignShift, uiPTAddrShift, uiPTAddrMask, uiPDEProtMask, ui32Flags)
#define PDUMP_MMU_FREE_PD(pszPDumpMemDevName, psDevPAddr) \
        PDumpMMUFree(pszPDumpMemDevName, "PAGE_DIRECTORY", psDevPAddr)
#define PDUMP_MMU_MALLOC_PT(pszPDumpMemDevName, pszDevPAddr, ui32Size, ui32Align) \
        PDumpMMUMalloc(pszPDumpMemDevName, "PAGE_TABLE", pszDevPAddr, ui32Size, ui32Align)
#define PDUMP_MMU_DUMP_PT_ENTRIES(pszPDumpMemDevName, pvPTMem, sPTDevPAddr, uiFirstEntry, uiNumEntries, pszDPMemspaceName, pszDPSymbolicAddr, uiDPSymbolicAddrOffset, uiBytesPerEntry, uiDPAddrAlignShift, uiDPAddrShift, uiDPAddrMask, uiPTEProtMask, ui32Flags) \
        PDumpMMUDumpPTEntries(pszPDumpMemDevName, pvPTMem, sPTDevPAddr, uiFirstEntry, uiNumEntries, pszDPMemspaceName, pszDPSymbolicAddr, uiDPSymbolicAddrOffset, uiBytesPerEntry, uiDPAddrAlignShift, uiDPAddrShift, uiDPAddrMask, uiPTEProtMask, ui32Flags)
#define PDUMP_MMU_FREE_PT(pszPDumpMemDevName, psDevPAddr) \
        PDumpMMUFree(pszPDumpMemDevName, "PAGE_TABLE", psDevPAddr)
#define PDUMP_MMU_MALLOC_DP(pszPDumpMemDevName, aszSymbolicAddr, ui32Size, ui32Align) \
        PDumpMMUMalloc2(pszPDumpMemDevName, "DATA_PAGE", aszSymbolicAddr, ui32Size, ui32Align)
#define PDUMP_MMU_FREE_DP(pszPDumpMemDevName, aszSymbolicAddr) \
        PDumpMMUFree2(pszPDumpMemDevName, "DATA_PAGE", aszSymbolicAddr)

#define PDUMP_MMU_ALLOC_MMUCONTEXT(pszPDumpMemDevName, sPCDevPAddr, puiPDumpCtxID) \
        PDumpMMUAllocMMUContext(pszPDumpMemDevName,                     \
                                sPCDevPAddr,                            \
                                puiPDumpCtxID)

#define PDUMP_MMU_FREE_MMUCONTEXT(pszPDumpMemDevName, uiPDumpCtxID) \
        PDumpMMUFreeMMUContext(pszPDumpMemDevName, uiPDumpCtxID)
#else
#define PDUMP_MMU_MALLOC_PC(pszPDumpMemDevName, pszDevPAddr, ui32Size, ui32Align) \
        ((void)0)
#define PDUMP_MMU_DUMP_PC_ENTRIES(pszPDumpMemDevName, pvPCMem, sPCDevPAddr, uiFirstEntry, uiNumEntries, uiBytesPerEntry, uiPDAddrAlignShift, uiPDAddrShift, uiPDAddrMask, uiPCEProtMask, ui32Flags) \
        ((void)0)
#define PDUMP_MMU_FREE_PC(pszPDumpMemDevName, psDevPAddr) \
        ((void)0)
#define PDUMP_MMU_MALLOC_PD(pszPDumpMemDevName, pszDevPAddr, ui32Size, ui32Align) \
        ((void)0)
#define PDUMP_MMU_DUMP_PD_ENTRIES(pszPDumpMemDevName, pvPDMem, sPDDevPAddr, uiFirstEntry, uiNumEntries, uiBytesPerEntry, uiPTAddrAlignShift, uiPTAddrShift, uiPTAddrMask, uiPDEProtMask, ui32Flags) \
        ((void)0)
#define PDUMP_MMU_FREE_PD(pszPDumpMemDevName, psDevPAddr) \
        ((void)0)
#define PDUMP_MMU_MALLOC_PT(pszPDumpMemDevName, pszDevPAddr, ui32Size, ui32Align) \
        ((void)0)
#define PDUMP_MMU_DUMP_PT_ENTRIES(pszPDumpMemDevName, pvPCMem, sPCDevPAddr, uiFirstEntry, uiNumEntries, pszDPMemspaceName, pszDPSymbolicAddr, uiDPSymbolicAddrOffset, uiBytesPerEntry, uiDPAddrAlignShift, uiDPAddrShift, uiDPAddrMask, uiPCEProtMask, ui32Flags) \
        ((void)0)
#define PDUMP_MMU_FREE_PT(pszPDumpMemDevName, psDevPAddr) \
        ((void)0)
#define PDUMP_MMU_MALLOC_DP(pszPDumpMemDevName, pszDevPAddr, ui32Size, ui32Align) \
        ((void)0)
#define PDUMP_MMU_FREE_DP(pszPDumpMemDevName, psDevPAddr) \
        ((void)0)

#define PDUMP_MMU_ALLOC_MMUCONTEXT(pszPDumpMemDevName, sPCDevPAddr, puiPDumpCtxID) \
        ((void)0)
#define PDUMP_MMU_FREE_MMUCONTEXT(pszPDumpMemDevName, uiPDumpCtxID) \
        ((void)0)
#endif				// defined(PDUMP)

#endif
