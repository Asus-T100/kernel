/******************************************************************************
 * Name         : mmu_common.h
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

#ifndef MMU_COMMON_H
#define MMU_COMMON_H

/*
	The Memory Management Unit (MMU) performs device virtual to physical translation.

	Terminology:
	 - page catalogue, PC	(optional, 3 tier MMU)
	 - page directory, PD
	 - page table, PT (can be variable sized)
	 - data page, DP (can be variable sized)
    Note: PD and PC are fixed size and can't be larger than 
           the native physical (CPU) page size
	Shifts and AlignShift variables:
	 - 'xxxShift' represent the number of bits a bitfield is shifted left from bit0 
	 - 'xxxAlignShift' is used to convert a bitfield (based at bit0) into byte units 
	 	by applying a bit shift left by 'xxxAlignShift' bits
*/

/*
	Device Virtual Address Config:

	Incoming Device Virtual Address is deconstructed into up to 4
	fields, where the virtual address is up to 64bits:
	MSB-----------------------------------------------LSB
	| PC Index:   | PD Index:  | PT Index: | DP offset: |
	| d bits      | c bits     | b-v bits  |  a+v bits  |
	-----------------------------------------------------
	where v is the variable page table modifier, e.g.
			v == 0 -> 4KB DP
			v == 2 -> 16KB DP
			v == 4 -> 64KB DP
			v == 6 -> 256KB DP
			v == 8 -> 1MB DP
			v == 10 -> 4MB DP
*/

/* services/server/include/ */
#include "pdump_mmu.h"
#include "pmr.h"

/* include/ */
#include "img_types.h"
#include "pvrsrv_error.h"
#include "servicesext.h"

/*!
	The level of the MMU
*/
typedef enum {
	MMU_LEVEL_0 = 0,	/* Level 0 = Page */

	MMU_LEVEL_1,
	MMU_LEVEL_2,
	MMU_LEVEL_3,
} MMU_LEVEL;

#define MMU_MAX_LEVEL 3

struct _MMU_DEVVADDR_CONFIG_;

/*!
	MMU device attributes. This structure is the interface between the generic
	MMU code and the device specific MMU code.
*/
typedef struct _MMU_DEVICEATTRIBS_ {
	/*! The type of the top level object */
	MMU_LEVEL eTopLevel;

	/*! Alignment requirement of the base object */
	IMG_UINT32 ui32BaseAlign;

	/*! HW config of the base object */
	struct _MMU_PxE_CONFIG_ *psBaseConfig;

	/*! Address split for the base object */
	const struct _MMU_DEVVADDR_CONFIG_ *psTopLevelDevVAddrConfig;

	/*! Callback for creating protection bits for the page catalogue entry with 8 bye entry */
	 IMG_UINT64(*pfnDerivePCEProt8) (IMG_UINT32);
	/*! Callback for creating protection bits for the page catalogue entry with 4 bye entry */
	 IMG_UINT32(*pfnDerivePCEProt4) (IMG_UINT32);
	/*! Callback for creating protection bits for the page directory entry with 8 bye entry */
	 IMG_UINT64(*pfnDerivePDEProt8) (IMG_UINT32);
	/*! Callback for creating protection bits for the page directory entry with 4 bye entry */
	 IMG_UINT32(*pfnDerivePDEProt4) (IMG_UINT32);
	/*! Callback for creating protection bits for the page table entry with 8 bye entry */
	 IMG_UINT64(*pfnDerivePTEProt8) (IMG_UINT32);
	/*! Callback for creating protection bits for the page table entry with 4 bye entry */
	 IMG_UINT32(*pfnDerivePTEProt4) (IMG_UINT32);

	/*! Callback for getting the MMU configuration based on the specified page size */
	 PVRSRV_ERROR(*pfnGetPageSizeConfiguration) (IMG_UINT32
						     ui32DataPageSize,
						     const struct
						     _MMU_PxE_CONFIG_ **
						     ppsMMUPDEConfig,
						     const struct
						     _MMU_PxE_CONFIG_ **
						     ppsMMUPTEConfig,
						     const struct
						     _MMU_DEVVADDR_CONFIG_ **
						     ppsMMUDevVAddrConfig,
						     IMG_HANDLE * phPriv2);
	/*! Callback for putting the MMU configuration obtained from pfnGetPageSizeConfiguration */
	 PVRSRV_ERROR(*pfnPutPageSizeConfiguration) (IMG_HANDLE hPriv);

	/*! Private data handle */
	IMG_HANDLE hGetPageSizeFnPriv;
} MMU_DEVICEATTRIBS;

/*!
	MMU virtual address split
*/
typedef struct _MMU_DEVVADDR_CONFIG_ {
	/*! Page catalogue index mask */
	IMG_UINT64 uiPCIndexMask;
	/*! Page catalogue index shift */
	IMG_UINT8 uiPCIndexShift;
	/*! Page directory mask */
	IMG_UINT64 uiPDIndexMask;
	/*! Page directory shift */
	IMG_UINT8 uiPDIndexShift;
	/*! Page table mask */
	IMG_UINT64 uiPTIndexMask;
	/*! Page index shift */
	IMG_UINT8 uiPTIndexShift;
	/*! Page offset mask */
	IMG_UINT64 uiPageOffsetMask;
	/*! Page offset shift */
	IMG_UINT8 uiPageOffsetShift;
} MMU_DEVVADDR_CONFIG;

/*
	P(C/D/T) Entry Config:

	MSB-----------------------------------------------LSB
	| PT Addr:   | variable PT ctrl | protection flags: |
	| bits c+v   | b bits           | a bits            |
	-----------------------------------------------------
	where v is the variable page table modifier and is optional
*/
/*!
	Generic MMU page * entry description. This is used to describe PC, PD and PT
*/
typedef struct _MMU_PxE_CONFIG_ {
	/*! Size of an entry in bytes */
	IMG_UINT8 uiBytesPerEntry;

	/*! Physical address mask */
	IMG_UINT64 uiAddrMask;
	/*! Physical address shift */
	IMG_UINT8 uiAddrShift;
	/*! Log 2 alignment */
	IMG_UINT8 uiLog2Align;

	/*! Variable control mask */
	IMG_UINT64 uiVarCtrlMask;
	/*! Variable control shift */
	IMG_UINT8 uiVarCtrlShift;

	/*! Protection flags mask */
	IMG_UINT64 uiProtMask;
	/*! Protection flags shift */
	IMG_UINT8 uiProtShift;
} MMU_PxE_CONFIG;

/* MMU Protection flags */

/* These are specified generically and in a h/w independent way, and
   are interpreted at each level (PC/PD/PT) separately. */

/* The following flags are for internal use only, and should not
   traverse the API */
/* FIXME: should they even be exposed in this file
   then? */
#define MMU_PROTFLAGS_INVALID 0x80000000U

/* 
   TODO: distinguish between "MMU_MAPFLAGS_T" -- specified by caller
   to MMU_MapPMR -- and "MMU_PROTFLAGS_T" used internally in
   mmu_common?  I think it's fine to have just one type here... */
typedef IMG_UINT32 MMU_PROTFLAGS_T;

/* The following flags should be supplied by the caller: */
#define MMU_PROTFLAGS_READABLE	   				(1U<<0)
#define MMU_PROTFLAGS_WRITEABLE		   		    (1U<<1)
#define MMU_PROTFLAGS_CACHE_COHERENT			(1U<<2)
#define MMU_PROTFLAGS_CACHED					(1U<<3)

/* Device specific flags*/
#define MMU_PROTFLAGS_DEVICE_OFFSET		16
#define MMU_PROTFLAGS_DEVICE_MASK		0x000f0000UL
#define MMU_PROTFLAGS_DEVICE(n)	\
			(((n) << MMU_PROTFLAGS_DEVICE_OFFSET) & \
			MMU_PROTFLAGS_DEVICE_MASK)

typedef struct _MMU_CONTEXT_ MMU_CONTEXT;

struct _PVRSRV_DEVICE_NODE_;

									    /*************************************************************************//*!
									       @Function       MMU_ContextCreate

									       @Description    Create a new MMU context

									       @Input          psDevNode               Device node of the device to create the
									       MMU context for

									       @Output         ppsMMUContext           The created MMU context

									       @Return         PVRSRV_OK if the MMU context was successfully created
									     */
/*****************************************************************************/
extern PVRSRV_ERROR
MMU_ContextCreate(struct _PVRSRV_DEVICE_NODE_ *psDevNode,
		  MMU_CONTEXT ** ppsMMUContext);

									    /*************************************************************************//*!
									       @Function       MMU_ContextDestroy

									       @Description    Destroy a MMU context

									       @Input          ppsMMUContext           MMU context to destroy

									       @Return         None
									     */
/*****************************************************************************/
extern IMG_VOID MMU_ContextDestroy(MMU_CONTEXT * psMMUContext);

									    /*************************************************************************//*!
									       @Function       MMU_Alloc

									       @Description    Allocate the page tables required for the specified virtual range

									       @Input          psMMUContext            MMU context to operate on

									       @Input          uSize                   The size of the allocation

									       @Output         puActualSize            Actual size of allocation

									       @Input          uiProtFlags             Generic MMU protection flags

									       @Input          uDevVAddrAlignment      Alignment requirement of the virtual
									       allocation

									       @Input          psDevVAddr              Virtual address to start the allocation
									       from

									       @Return         PVRSRV_OK if the allocation of the page tables was successful
									     */
/*****************************************************************************/
extern PVRSRV_ERROR
MMU_Alloc(MMU_CONTEXT * psMMUContext,
	  IMG_DEVMEM_SIZE_T uSize,
	  IMG_DEVMEM_SIZE_T * puActualSize,
	  IMG_UINT32 uiProtFlags,
	  IMG_DEVMEM_SIZE_T uDevVAddrAlignment, IMG_DEV_VIRTADDR * psDevVAddr);

									    /*************************************************************************//*!
									       @Function       MMU_Free

									       @Description    Free the page tables of the specified virtual range

									       @Input          psMMUContext            MMU context to operate on

									       @Input          psDevVAddr              Virtual address to start the free
									       from

									       @Input          uSize                   The size of the allocation

									       @Return         None
									     */
/*****************************************************************************/
extern IMG_VOID
MMU_Free(MMU_CONTEXT * psMMUContext, IMG_DEV_VIRTADDR sDevVAddr,
	 IMG_DEVMEM_SIZE_T uiSize);

									    /*************************************************************************//*!
									       @Function       MMU_UnmapPages

									       @Description    Unmap pages from the MMU.

									       @Input          psMMUContext            MMU context to operate on

									       @Input          psDevVAddr              Device virtual address of the 1st page

									       @Input          ui32PageCount           Number of pages to unmap

									       @Return         None
									     */
/*****************************************************************************/
extern IMG_VOID
MMU_UnmapPages(MMU_CONTEXT * psMMUContext,
	       IMG_DEV_VIRTADDR sDevVAddr, IMG_UINT32 ui32PageCount);

									    /*************************************************************************//*!
									       @Function       MMU_MapPMR

									       @Description    Map a PMR into the MMU.

									       @Input          psMMUContext            MMU context to operate on

									       @Input          sDevVAddr               Device virtual address to map the PMR
									       into

									       @Input          psPMR                   PMR to map

									       @Input          uiSizeBytes             Size in bytes to map

									       @Input          uiMappingFlags          Memalloc flags for the mapping

									       @Return         PVRSRV_OK if the PMR was successfully mapped
									     */
/*****************************************************************************/
extern PVRSRV_ERROR
MMU_MapPMR(MMU_CONTEXT * psMMUContext,
	   IMG_DEV_VIRTADDR sDevVAddr,
	   const PMR * psPMR,
	   IMG_DEVMEM_SIZE_T uiSizeBytes,
	   PVRSRV_MEMALLOCFLAGS_T uiMappingFlags);

									    /*************************************************************************//*!
									       @Function       MMU_AcquireBaseAddr

									       @Description    Acquire the device physcial address of the base level MMU object

									       @Input          psMMUContext            MMU context to operate on

									       @Output         psPhysAddr              Device physical address of the base level
									       MMU object

									       @Return         PVRSRV_OK if successful
									     */
/*****************************************************************************/
PVRSRV_ERROR
MMU_AcquireBaseAddr(MMU_CONTEXT * psMMUContext, IMG_DEV_PHYADDR * psPhysAddr);

									    /*************************************************************************//*!
									       @Function       MMU_ReleaseBaseAddr

									       @Description    Release the device physcial address of the base level MMU object

									       @Input          psMMUContext            MMU context to operate on

									       @Return         PVRSRV_OK if successful
									     */
/*****************************************************************************/
IMG_VOID MMU_ReleaseBaseAddr(MMU_CONTEXT * psMMUContext);

									    /*************************************************************************//*!
									       @Function       MMU_SetDeviceData

									       @Description    Set the device specific callback data

									       @Input          psMMUContext            MMU context to store the data on

									       @Input          hDevData                Device data

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID MMU_SetDeviceData(MMU_CONTEXT * psMMUContext, IMG_HANDLE hDevData);

#if defined(PDUMP)
									    /*************************************************************************//*!
									       @Function       MMU_ContextDerivePCPDumpSymAddr

									       @Description    Derives a PDump Symbolic address for the top level MMU object

									       @Input          psMMUContext                    MMU context to operate on

									       @Input          pszPDumpSymbolicNameBuffer      Buffer to write the PDump symbolic
									       address to

									       @Input          uiPDumpSymbolicNameBufferSize   Size of the buffer

									       @Return         PVRSRV_OK if successful
									     */
/*****************************************************************************/
extern PVRSRV_ERROR MMU_ContextDerivePCPDumpSymAddr(MMU_CONTEXT * psMMUContext,
						    IMG_CHAR *
						    pszPDumpSymbolicNameBuffer,
						    IMG_SIZE_T
						    uiPDumpSymbolicNameBufferSize);

									    /*************************************************************************//*!
									       @Function       MMU_PDumpWritePageCatBase

									       @Description    PDump write of the top level MMU object to a device register

									       @Input          psMMUContext        MMU context to operate on

									       @Input          pszSpaceName             PDump name of the mem/reg space

									       @Input          uiOffset                 Offset to write the address to

									       @Return         PVRSRV_OK if successful
									     */
/*****************************************************************************/
PVRSRV_ERROR MMU_PDumpWritePageCatBase(MMU_CONTEXT * psMMUContext,
				       const IMG_CHAR * pszSpaceName,
				       IMG_DEVMEM_OFFSET_T uiOffset);

									    /*************************************************************************//*!
									       @Function       MMU_AcquirePDumpMMUContext

									       @Description    Accquire a reference to the PDump MMU context for this MMU
									       context

									       @Input          psMMUContext            MMU context to operate on

									       @Input          pszRegSpaceName         PDump name of the register space

									       @Output         pui32PDumpMMUContextID  PDump MMU context ID

									       @Return         PVRSRV_OK if successful
									     */
/*****************************************************************************/
PVRSRV_ERROR MMU_AcquirePDumpMMUContext(MMU_CONTEXT * psMMUContext,
					IMG_UINT32 * pui32PDumpMMUContextID);

									    /*************************************************************************//*!
									       @Function       MMU_ReleasePDumpMMUContext

									       @Description    Release a reference to the PDump MMU context for this MMU context

									       @Input          psMMUContext            MMU context to operate on

									       @Input          pszRegSpaceName         PDump name of the register space

									       @Output         pui32PDumpMMUContextID  PDump MMU context ID

									       @Return         PVRSRV_OK if successful
									     */
/*****************************************************************************/
PVRSRV_ERROR MMU_ReleasePDumpMMUContext(MMU_CONTEXT * psMMUContext);
#else				/* PDUMP */

#ifdef INLINE_IS_PRAGMA
#pragma inline(MMU_PDumpWritePageCatBase)
#endif
static INLINE IMG_VOID
MMU_PDumpWritePageCatBase(MMU_CONTEXT * psMMUContext,
			  const IMG_CHAR * pszSpaceName,
			  IMG_DEVMEM_OFFSET_T uiOffset)
{
	PVR_UNREFERENCED_PARAMETER(psMMUContext);
	PVR_UNREFERENCED_PARAMETER(pszSpaceName);
	PVR_UNREFERENCED_PARAMETER(uiOffset);
}
#endif				/* PDUMP */

#endif				/* #ifdef MMU_COMMON_H */
