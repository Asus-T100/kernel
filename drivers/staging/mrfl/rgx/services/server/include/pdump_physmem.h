/******************************************************************************
 * Name         : pdump_physmem.h
 * Title        : pdump functions to assist with physmem allocations
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

#ifndef SRVSRV_PDUMP_PHYSMEM_H
#define SRVSRV_PDUMP_PHYSMEM_H

#include "img_types.h"
#include "pdumpdefs.h"
#include "pvrsrv_error.h"

#include "pdump.h"

typedef struct _PDUMP_PHYSMEM_INFO_T_ PDUMP_PHYSMEM_INFO_T;

#if defined(PDUMP)
/* TODO: this isn't really PMR specific any more
   - perhaps this should go to a more common place? */
extern PVRSRV_ERROR
PDumpPMRMalloc(const IMG_CHAR * pszDevSpace,
	       const IMG_CHAR * pszSymbolicAddress, IMG_UINT64 ui64Size,
	       /* alignment is alignment of start of buffer _and_
	          minimum contiguity - i.e. smallest allowable
	          page-size.  TODO: review this decision. */
	       IMG_UINT32 ui32Align,
	       IMG_BOOL bForcePersistent, IMG_HANDLE * phHandlePtr);
extern
PVRSRV_ERROR PDumpPMRFree(IMG_HANDLE hPDumpAllocationInfoHandle);
#else				/* PDUMP */

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSyncPrimPDumpPolKM)
#endif
static INLINE PVRSRV_ERROR
PDumpPMRMalloc(const IMG_CHAR * pszDevSpace,
	       const IMG_CHAR * pszSymbolicAddress,
	       IMG_UINT64 ui64Size,
	       IMG_UINT32 ui32Align,
	       IMG_BOOL bForcePersistent, IMG_HANDLE * phHandlePtr)
{
	PVR_UNREFERENCED_PARAMETER(pszDevSpace);
	PVR_UNREFERENCED_PARAMETER(pszSymbolicAddress);
	PVR_UNREFERENCED_PARAMETER(ui64Size);
	PVR_UNREFERENCED_PARAMETER(ui32Align);
	PVR_UNREFERENCED_PARAMETER(bForcePersistent);
	PVR_UNREFERENCED_PARAMETER(phHandlePtr);
	PVR_UNREFERENCED_PARAMETER(bForcePersistent);
	return PVRSRV_OK;
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSyncPrimPDumpPolKM)
#endif
static INLINE PVRSRV_ERROR PDumpPMRFree(IMG_HANDLE hPDumpAllocationInfoHandle)
{
	PVR_UNREFERENCED_PARAMETER(hPDumpAllocationInfoHandle);
	return PVRSRV_OK;
}
#endif				/* PDUMP */

#define PMR_DEFAULT_PREFIX "PMR"
#define PMR_DEFAULT_MEMSPACE_NAME "SYSMEM"
#define PMR_SYMBOLICADDR_FMTSPEC "%s%llu"

#if defined(PDUMP)
#define PDUMP_PHYSMEM_MALLOC_OSPAGES(pszPDumpMemDevName, ui32SerialNum, ui32Size, ui32Align, phHandlePtr) \
    PDumpPMRMalloc(pszPDumpMemDevName, PMR_OSALLOCPAGES_PREFIX, ui32SerialNum, ui32Size, ui32Align, phHandlePtr)
#define PDUMP_PHYSMEM_FREE_OSPAGES(hHandle) \
    PDumpPMRFree(hHandle)
#else
#define PDUMP_PHYSMEM_MALLOC_OSPAGES(pszPDumpMemDevName, ui32SerialNum, ui32Size, ui32Align, phHandlePtr) \
    ((void)(*phHandlePtr=IMG_NULL))
#define PDUMP_PHYSMEM_FREE_OSPAGES(hHandle) \
    ((void)(0))
#endif				// defined(PDUMP)

extern PVRSRV_ERROR
PDumpPMRWRW(const IMG_CHAR * pszDevSpace,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiOffset,
	    IMG_UINT32 ui32Value, PDUMP_FLAGS_T uiPDumpFlags);

extern PVRSRV_ERROR
PDumpPMRLDB(const IMG_CHAR * pszDevSpace,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiOffset,
	    IMG_DEVMEM_SIZE_T uiSize,
	    const IMG_CHAR * pszFilename,
	    IMG_UINT32 uiFileOffset, PDUMP_FLAGS_T uiPDumpFlags);

extern PVRSRV_ERROR
PDumpPMRSAB(const IMG_CHAR * pszDevSpace,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiOffset,
	    IMG_DEVMEM_SIZE_T uiSize,
	    const IMG_CHAR * pszFileName, IMG_UINT32 uiFileOffset);

/*
  PDumpPMRPOL()

  emits a POL to the PDUMP.
*/
/* TODO: move to pdump_common, no longer PMR specific */
extern PVRSRV_ERROR
PDumpPMRPOL(const IMG_CHAR * pszMempaceName,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiOffset,
	    IMG_UINT32 ui32Value,
	    IMG_UINT32 ui32Mask,
	    PDUMP_POLL_OPERATOR eOperator,
	    IMG_UINT32 uiCount, IMG_UINT32 uiDelay, PDUMP_FLAGS_T uiPDumpFlags);

extern PVRSRV_ERROR
PDumpPMRCBP(const IMG_CHAR * pszMemspaceName,
	    const IMG_CHAR * pszSymbolicName,
	    IMG_DEVMEM_OFFSET_T uiReadOffset,
	    IMG_DEVMEM_OFFSET_T uiWriteOffset,
	    IMG_DEVMEM_SIZE_T uiPacketSize, IMG_DEVMEM_SIZE_T uiBufferSize);

/*
 * PDumpWriteBuffer()
 *
 * writes a binary blob to the pdump param stream containing the
 * current contents of the memory, and returns the filename and offset
 * of where that blob is located (for use in a subsequent LDB, for
 * example)
 *
 * Caller to provide buffer to receive filename, and declare the size
 * of that buffer
 */
extern PVRSRV_ERROR PDumpWriteBuffer( /* const */ IMG_UINT8 * pcBuffer,
				     /* FIXME:
				        pcBuffer above ought to be :
				        const IMG_UINT8 *pcBuffer
				        but, PDumpOSWriteString takes pointer to non-const data.
				      */
				     IMG_SIZE_T uiNumBytes,
				     PDUMP_FLAGS_T uiPDumpFlags,
				     IMG_CHAR * pszFilenameOut,
				     IMG_SIZE_T uiFilenameBufSz,
				     PDUMP_FILEOFFSET_T * puiOffsetOut);

#endif				/* #ifndef SRVSRV_PDUMP_PHYSMEM_H */
