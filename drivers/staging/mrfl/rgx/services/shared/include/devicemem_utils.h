									     /**************************************************************************//*!
									        @File           devicemem_utils.h
									        @Title          Device Memory Management internal utitlity functions
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Utility functions used internaly by device memory manamagement
									        code.
    *//***************************************************************************/

#ifndef _DEVICEMEM_UTILS_H_
#define _DEVICEMEM_UTILS_H_

#include "devicemem.h"
#include "img_types.h"
#include "pvrsrv_error.h"
#include "pvr_debug.h"
#include "allocmem.h"
#include "ra.h"
#include "osfunc.h"
#include "devicemem_mmap.h"
#include "devicemem_utils.h"
#include "client_mm_bridge.h"

#define DEVMEM_HEAPNAME_MAXLENGTH 160

#if defined(DEVMEM_DEBUG) && defined(REFCOUNT_DEBUG)
#define DEVMEM_REFCOUNT_PRINT_PRINT(fmt, ...) PVRSRVDebugPrintf(PVR_DBG_ERROR, fmt, __VA_ARGS__)
#else
#define DEVMEM_REFCOUNT_PRINT(fmt, ...)
#endif

/* If we need a "hMapping" but we don't have a server-side mapping, we
   poison the entry with this value so that it's easily recognised in
   the debugger.  Note that this is potentially a valid handle, but
   then so is IMG_NULL, which is no better, indeed worse, as it's not
   obvious in the debugger.  The value doesn't matter.  We _never_ use
   it (and because it's valid, we never assert it isn't this) but it's
   nice to have a value in the source code that we can grep for when
   things go wrong. */
#define LACK_OF_MAPPING_POISON ((IMG_HANDLE)0x6116dead)
#define LACK_OF_RESERVATION_POISON ((IMG_HANDLE)0x7117dead)

struct _DEVMEM_CONTEXT_ {
	/* Cookie of the device on which this memory context resides */
	IMG_HANDLE hDeviceNode;

	/* Number of heaps that have been created in this context
	   (regardless of whether they have allocations) */
	IMG_UINT32 uiNumHeaps;

	/* Sometimes we need to talk to Kernel Services.  In order to do
	   so, we need the connection handle */
	DEVMEM_BRIDGE_HANDLE hBridge;

	/*
	   Each "DEVMEM_CONTEXT" has a counterpart in the server,
	   which is responsible for handling the mapping into device MMU.
	   We have a handle to that here.
	 */
	IMG_HANDLE hDevMemServerContext;

	/* Number of automagically created heaps in this context,
	   i.e. those that are born at context creation time from the
	   chosen "heap config" or "blueprint" */
	IMG_UINT32 uiAutoHeapCount;

	/* pointer to array of such heaps */
	struct _DEVMEM_HEAP_ **ppsAutoHeapArray;

	/* Private data handle for device specific data */
	IMG_HANDLE hPrivData;
};

struct _DEVMEM_HEAP_ {
	/* Name of heap - for debug and lookup purposes. */
	IMG_CHAR *pszName;

	/* Number of live imports in the heap */
	IMG_UINT32 uiImportCount;

	/*
	 * Base address of heap, required by clients due to some requesters
	 * not being full range 
	 */
	IMG_DEV_VIRTADDR sBaseAddress;

	/* This RA is for managing sub-allocations in virtual space.  Two
	   more RA's will be used under the hood for managing the coarser
	   allocation of virtual space from the heap, and also for
	   managing the physical backing storage. */
	RA_ARENA *psSubAllocRA;
	IMG_CHAR *pszSubAllocRAName;
	/*
	   This RA is for the coarse allocation of virtual space from the heap
	 */
	RA_ARENA *psQuantizedVMRA;
	IMG_CHAR *pszQuantizedVMRAName;

	/* We also need to store a copy of the quantum size in order to
	   feed this down to the server */
	IMG_UINT32 uiLog2Quantum;

	/* The parent memory context for this heap */
	struct _DEVMEM_CONTEXT_ *psCtx;

	/*
	   Each "DEVMEM_HEAP" has a counterpart in the server,
	   which is responsible for handling the mapping into device MMU.
	   We have a handle to that here.
	 */
	IMG_HANDLE hDevMemServerHeap;
};

typedef struct _DEVMEM_DEVICE_IMPORT_ {
	DEVMEM_HEAP *psHeap;	/*!< Heap this import is bound to */
	IMG_DEV_VIRTADDR sDevVAddr;	/*!< Device virtual address of the import */
	IMG_UINT32 ui32RefCount;	/*!< Refcount of the device virtual address */
	IMG_HANDLE hReservation;	/*!< Device memory reservation handle */
	IMG_HANDLE hMapping;	/*!< Device mapping handle */
	IMG_BOOL bMapped;	/*!< This is import mapped? */
} DEVMEM_DEVICE_IMPORT;

typedef struct _DEVMEM_CPU_IMPORT_ {
	IMG_PVOID pvCPUVAddr;	/*!< CPU virtual address of the import */
	IMG_UINT32 ui32RefCount;	/*!< Refcount of the CPU virtual address */
	IMG_HANDLE hOSMMapData;	/*!< CPU mapping handle */
} DEVMEM_CPU_IMPORT;

typedef struct _DEVMEM_IMPORT_ {
	DEVMEM_BRIDGE_HANDLE hBridge;	/*!< Bridge connection for the server */
	IMG_DEVMEM_ALIGN_T uiAlign;	/*!< Alignment requirement */
	DEVMEM_SIZE_T uiSize;	/*!< Size of import */
	IMG_UINT32 ui32RefCount;	/*!< Refcount for this import */
	IMG_BOOL bExportable;	/*!< Is this import exportable? */
	IMG_HANDLE hPMR;	/*!< Handle to the PMR */
	DEVMEM_FLAGS_T uiFlags;	/*!< Flags for this import */

	DEVMEM_DEVICE_IMPORT sDeviceImport;	/*!< Device specifics of the import */
	DEVMEM_CPU_IMPORT sCPUImport;	/*!< CPU specifics of the import */
} DEVMEM_IMPORT;

typedef struct _DEVMEM_DEVICE_MEMDESC_ {
	IMG_DEV_VIRTADDR sDevVAddr;	/*!< Device virtual address of the allocation */
	IMG_UINT32 ui32RefCount;	/*!< Refcount of the device virtual address */
} DEVMEM_DEVICE_MEMDESC;

typedef struct _DEVMMEM_CPU_MEMDESC_ {
	IMG_PVOID pvCPUVAddr;	/*!< CPU virtual address of the import */
	IMG_UINT32 ui32RefCount;	/*!< Refcount of the device CPU address */
} DEVMMEM_CPU_MEMDESC;

struct _DEVMEM_MEMDESC_ {
	DEVMEM_IMPORT *psImport;	/*!< Import this memdesc is on */
	IMG_DEVMEM_OFFSET_T uiOffset;	/*!< Offset into import where our allocation starts */
	IMG_UINT32 ui32RefCount;	/*!< Refcount of the memdesc */

	DEVMEM_DEVICE_MEMDESC sDeviceMemDesc;	/*!< Device specifics of the memdesc */
	DEVMMEM_CPU_MEMDESC sCPUMemDesc;	/*!< CPU specifics of the memdesc */
};

PVRSRV_ERROR _DevmemValidateParams(IMG_DEVMEM_SIZE_T uiSize,
				   IMG_DEVMEM_ALIGN_T uiAlign,
				   DEVMEM_FLAGS_T uiFlags);

PVRSRV_ERROR _DevmemImportStructAlloc(IMG_HANDLE hBridge,
				      IMG_BOOL bExportable,
				      DEVMEM_IMPORT ** ppsImport);

IMG_VOID _DevmemImportStructFree(DEVMEM_IMPORT * psImport);

IMG_VOID _DevmemImportStructInit(DEVMEM_IMPORT * psImport,
				 IMG_DEVMEM_SIZE_T uiSize,
				 IMG_DEVMEM_ALIGN_T uiAlign,
				 PVRSRV_MEMALLOCFLAGS_T uiMapFlags,
				 IMG_HANDLE hPMR);

PVRSRV_ERROR _DevmemImportStructDevMap(DEVMEM_HEAP * psHeap,
				       IMG_BOOL bMap, DEVMEM_IMPORT * psImport);

IMG_VOID _DevmemImportStructDevUnmap(DEVMEM_IMPORT * psImport);

PVRSRV_ERROR _DevmemImportStructCPUMap(DEVMEM_IMPORT * psImport);

IMG_VOID _DevmemImportStructCPUUnmap(DEVMEM_IMPORT * psImport);

PVRSRV_ERROR _DevmemValidateParams(IMG_DEVMEM_SIZE_T uiSize,
				   IMG_DEVMEM_ALIGN_T uiAlign,
				   DEVMEM_FLAGS_T uiFlags);

/*
	The Devmem import structure is the structure we use
	to manage memory that is "imported" (which is page
	granular) from the server into our process, this
	includes allocations.

	This allows memory to be imported without requireing
	any CPU or device mapping. Memory can then be mapped
	into the device or CPU on demand, but neather is
	required.
*/

static INLINE IMG_VOID _DevmemImportStructAcquire(DEVMEM_IMPORT * psImport)
{
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psImport,
			      psImport->ui32RefCount,
			      psImport->ui32RefCount + 1);

	psImport->ui32RefCount++;
}

static INLINE IMG_VOID _DevmemImportStructRelease(DEVMEM_IMPORT * psImport)
{
	PVR_ASSERT(psImport->ui32RefCount != 0);

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psImport,
			      psImport->ui32RefCount,
			      psImport->ui32RefCount - 1);

	if (--psImport->ui32RefCount == 0) {
		BridgePMRUnrefPMR(psImport->hBridge, psImport->hPMR);
		OSFreeMem(psImport);
	}
}

/*
	Discard a created, but unitilised import structure.
	This must only be called before _DevmemImportStructInit
	after which _DevmemImportStructRelease must be used to
	"free" the import structure
*/
static INLINE IMG_VOID _DevmemImportDiscard(DEVMEM_IMPORT * psImport)
{
	PVR_ASSERT(psImport->ui32RefCount == 0);
	OSFreeMem(psImport);
}

/*
	Init the MemDesc structure
*/
static INLINE IMG_VOID _DevmemMemDescInit(DEVMEM_MEMDESC * psMemDesc,
					  IMG_DEVMEM_OFFSET_T uiOffset,
					  DEVMEM_IMPORT * psImport)
{
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d", __FUNCTION__, psMemDesc, 0, 1);

	psMemDesc->psImport = psImport;
	psMemDesc->uiOffset = uiOffset;

	psMemDesc->sDeviceMemDesc.ui32RefCount = 0;
	psMemDesc->sCPUMemDesc.ui32RefCount = 0;
	psMemDesc->ui32RefCount = 1;
}

static INLINE IMG_VOID _DevmemMemDescAcquire(DEVMEM_MEMDESC * psMemDesc)
{
	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psMemDesc,
			      psMemDesc->ui32RefCount,
			      psMemDesc->ui32RefCount + 1);

	psMemDesc->ui32RefCount++;
}

static INLINE IMG_VOID _DevmemMemDescRelease(DEVMEM_MEMDESC * psMemDesc)
{
	PVR_ASSERT(psMemDesc->ui32RefCount != 0);

	DEVMEM_REFCOUNT_PRINT("%s (%p) %d->%d",
			      __FUNCTION__,
			      psMemDesc,
			      psMemDesc->ui32RefCount,
			      psMemDesc->ui32RefCount - 1);

	if (--psMemDesc->ui32RefCount == 0) {
		if (!psMemDesc->psImport->bExportable) {
			RA_Free(psMemDesc->psImport->sDeviceImport.psHeap->
				psSubAllocRA,
				psMemDesc->sDeviceMemDesc.sDevVAddr.uiAddr);
		} else {
			_DevmemImportStructRelease(psMemDesc->psImport);
		}
		OSFreeMem(psMemDesc);
	}
}
#endif				/* _DEVICEMEM_UTILS_H_ */
