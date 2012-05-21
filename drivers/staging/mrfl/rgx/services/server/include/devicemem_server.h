/*!****************************************************************************
@File           devicemem_server.h

@Title          Device Memory Management

@Author         Imagination Technologies

@Copyright      Copyright 2010 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either material
                or conceptual may be copied or distributed, transmitted,
                transcribed, stored in a retrieval system or translated into
                any human or computer language in any form by any means,
                electronic, mechanical, manual or otherwise, or disclosed
                to third parties without the express written permission of
                Imagination Technologies Limited, Home Park Estate,
                Kings Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform       Generic

@Description    Header file for server side component of device memory management

@DoxygenVer

******************************************************************************/

#ifndef __DEVICEMEM_SERVER_H__
#define __DEVICEMEM_SERVER_H__

#include "device.h"		/* For device node */
#include "img_types.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"

#include "pmr.h"

typedef struct _DEVMEMINT_CTX_ DEVMEMINT_CTX;
typedef struct _DEVMEMINT_HEAP_ DEVMEMINT_HEAP;
/* TODO: can we unify RESERVATION and MAPPING to save data structures? */
typedef struct _DEVMEMINT_RESERVATION_ DEVMEMINT_RESERVATION;
typedef struct _DEVMEMINT_MAPPING_ DEVMEMINT_MAPPING;

/*
 * DevmemIntCtxCreate()
 *
 * Create a Server-side Device Memory Context.  This is usually the
 * counterpart of the client side memory context, and indeed is
 * usually created at the same time.
 *
 * You must have one of these before creating any heaps.
 *
 * All heaps must have been destroyed before calling
 * DevmemIntCtxDestroy()
 *
 * If you call DevmemIntCtxCreate() (and it succeeds) you are promising
 * to later call DevmemIntCtxDestory()
 *
 * Note that this call will cause the device MMU code to do some work
 * for creating the device memory context, but it does not guarantee
 * that a page catalogue will have been created, as this may be
 * deferred until first allocation.
 *
 * Caller to provide storage for a pointer to the DEVMEM_CTX object
 * that will be created by this call.
 */
extern PVRSRV_ERROR DevmemIntCtxCreate(PVRSRV_DEVICE_NODE * psDeviceNode,
				       /* devnode / perproc / resman context etc */
				       DEVMEMINT_CTX ** ppsDevmemCtxPtr,
				       IMG_HANDLE * hPrivData);
/*
 * DevmemIntCtxDestroy()
 *
 * Undoes a prior DevmemIntCtxCreate.
 */
extern PVRSRV_ERROR DevmemIntCtxDestroy(DEVMEMINT_CTX * psDevmemCtx);

/*
 * DevmemIntHeapCreate()
 *
 * Creates a new heap in this device memory context.  This will cause
 * a call into the MMU code to allocate various data structures for
 * managing this heap.  It will not necessarily cause any page tables
 * to be set up, as this can be deferred until first allocation.
 * (i.e. we shouldn't care - it's up to the MMU code)
 *
 * Note that the data page size must be specified (as log 2).  The
 * data page size as specified here will be communicated to the mmu
 * module, and thus may determine the page size configured in page
 * directory entries for subsequent allocations from this heap.  It is
 * essential that the page size here is less than or equal to the
 * "minimum contiguity guarantee" of any PMR that you subsequently
 * attempt to map to this heap.
 *
 * If you call DevmemIntHeapCreate() (and the call succeeds) you are
 * promising that you shall subsequently call DevmemIntHeapDestroy()
 *
 * Caller to provide storage for a pointer to the DEVMEM_HEAP object
 * that will be created by this call.
 */
extern PVRSRV_ERROR
DevmemIntHeapCreate(DEVMEMINT_CTX * psDevmemCtx,
		    IMG_DEV_VIRTADDR sHeapBaseAddr,
		    IMG_DEVMEM_SIZE_T uiHeapLength,
		    IMG_UINT32 uiLog2DataPageSize,
		    DEVMEMINT_HEAP ** ppsDevmemHeapPtr);
/*
 * DevmemIntHeapDestroy()
 *
 * Destroys a heap previously created with DevmemIntHeapCreate()
 *
 * All allocations from his heap must have been freed before this
 * call.
 */
extern PVRSRV_ERROR DevmemIntHeapDestroy(DEVMEMINT_HEAP * psDevmemHeap);

/*
 * DevmemIntMapPMR()
 *
 * Maps the given PMR to the virtual range previously allocated with
 * DevmemIntReserveRange()
 *
 * If appropriate, the PMR must have had its physical backing
 * committed, as this call will call into the MMU code to set up the
 * page tables for this allocation, which shall in turn request the
 * physical addresses from the PMR.  Alternatively, the PMR
 * implementation can choose to do so off the back of the "lock"
 * callback, which it will receive as a result (indirectly) of this
 * call.
 *
 * This function makes no promise w.r.t. the circumstances that it can
 * be called, and these would be "inherited" from the implementation
 * of the PMR.  For example if the PMR "lock" callback causes pages to
 * be pinned at that time (which may cause scheduling or disk I/O
 * etc.) then it would not be legal to "Map" the PMR in a context
 * where scheduling events are disallowed.
 *
 * If you call DevmemIntMapPMR() (and the call succeeds) then you are
 * promising that you shall later call DevmemIntUnmapPMR()
 */
extern PVRSRV_ERROR
DevmemIntMapPMR(DEVMEMINT_HEAP * psDevmemHeap,
		DEVMEMINT_RESERVATION * psReservation,
		PMR * psPMR,
		PVRSRV_MEMALLOCFLAGS_T uiMapFlags,
		DEVMEMINT_MAPPING ** ppsMappingPtr);
/*
 * DevmemIntUnmapPMR()
 *
 * Reverses the mapping caused by DevmemIntMapPMR()
 */
extern PVRSRV_ERROR DevmemIntUnmapPMR(DEVMEMINT_MAPPING * psMapping);

/*
 * DevmemIntReserveRange()
 *
 * Indicates that the specified range should be reserved from the
 * given heap.
 *
 * In turn causes the page tables to be allocated to cover the
 * specified range.
 *
 * If you call DevmemIntReserveRange() (and the call succeeds) then you
 * are promising that you shall later call DevmemIntUnreserveRange()
 */
extern PVRSRV_ERROR
DevmemIntReserveRange(DEVMEMINT_HEAP * psDevmemHeap,
		      IMG_DEV_VIRTADDR sAllocationDevVAddr,
		      IMG_DEVMEM_SIZE_T uiAllocationSize,
		      DEVMEMINT_RESERVATION ** ppsReservationPtr);
/*
 * DevmemIntUnreserveRange()
 *
 * Undoes the state change caused by DevmemIntReserveRage()
 */
extern PVRSRV_ERROR
DevmemIntUnreserveRange(DEVMEMINT_RESERVATION * psDevmemReservation);

/*
 * SLCFlushInvalRequest()
 *
 * Schedules an SLC Flush & Invalidate on the firmware if required.
 * If the request is performed depends on the caching attributes
 * of the allocation and hence depends on the underlying PMR
  */
extern PVRSRV_ERROR
DevmemSLCFlushInvalRequest(PVRSRV_DEVICE_NODE * psDeviceNode, PMR * psPmr);

#if defined(PDUMP)
/*
 * DevmemIntPDumpSaveToFileVirtual()
 *
 * Writes out PDump "SAB" commands with the data found in memory at
 * the given virtual address.
 */
/* FIXME: uiArraySize shouldn't be here, and is an
   artefact of the bridging */
extern PVRSRV_ERROR
DevmemIntPDumpSaveToFileVirtual(DEVMEMINT_CTX * psDevmemCtx,
				IMG_DEV_VIRTADDR sDevAddrStart,
				IMG_DEVMEM_SIZE_T uiSize,
				IMG_UINT32 uiArraySize,
				const IMG_CHAR * pszFilename,
				IMG_UINT32 ui32FileOffset,
				IMG_UINT32 ui32PDumpFlags);

extern PVRSRV_ERROR
DevmemIntPDumpBitmap(PVRSRV_DEVICE_NODE * psDeviceNode,
		     IMG_CHAR * pszFileName,
		     IMG_UINT32 ui32FileOffset,
		     IMG_UINT32 ui32Width,
		     IMG_UINT32 ui32Height,
		     IMG_UINT32 ui32StrideInBytes,
		     IMG_DEV_VIRTADDR sDevBaseAddr,
		     DEVMEMINT_CTX * psDevMemContext,
		     IMG_UINT32 ui32Size,
		     PDUMP_PIXEL_FORMAT ePixelFormat,
		     PDUMP_MEM_FORMAT eMemFormat, IMG_UINT32 ui32PDumpFlags);
#else				/* PDUMP */

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSyncPrimPDumpPolKM)
#endif
static INLINE PVRSRV_ERROR
DevmemIntPDumpSaveToFileVirtual(DEVMEMINT_CTX * psDevmemCtx,
				IMG_DEV_VIRTADDR sDevAddrStart,
				IMG_DEVMEM_SIZE_T uiSize,
				IMG_UINT32 uiArraySize,
				const IMG_CHAR * pszFilename,
				IMG_UINT32 ui32FileOffset,
				IMG_UINT32 ui32PDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psDevmemCtx);
	PVR_UNREFERENCED_PARAMETER(sDevAddrStart);
	PVR_UNREFERENCED_PARAMETER(uiSize);
	PVR_UNREFERENCED_PARAMETER(uiArraySize);
	PVR_UNREFERENCED_PARAMETER(pszFilename);
	PVR_UNREFERENCED_PARAMETER(ui32FileOffset);
	PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);
	return PVRSRV_OK;
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSyncPrimPDumpPolKM)
#endif
static INLINE PVRSRV_ERROR
DevmemIntPDumpBitmap(PVRSRV_DEVICE_NODE * psDeviceNode,
		     IMG_CHAR * pszFileName,
		     IMG_UINT32 ui32FileOffset,
		     IMG_UINT32 ui32Width,
		     IMG_UINT32 ui32Height,
		     IMG_UINT32 ui32StrideInBytes,
		     IMG_DEV_VIRTADDR sDevBaseAddr,
		     DEVMEMINT_CTX * psDevMemContext,
		     IMG_UINT32 ui32Size,
		     PDUMP_PIXEL_FORMAT ePixelFormat,
		     PDUMP_MEM_FORMAT eMemFormat, IMG_UINT32 ui32PDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psDeviceNode);
	PVR_UNREFERENCED_PARAMETER(pszFileName);
	PVR_UNREFERENCED_PARAMETER(ui32FileOffset);
	PVR_UNREFERENCED_PARAMETER(ui32Width);
	PVR_UNREFERENCED_PARAMETER(ui32Height);
	PVR_UNREFERENCED_PARAMETER(ui32StrideInBytes);
	PVR_UNREFERENCED_PARAMETER(sDevBaseAddr);
	PVR_UNREFERENCED_PARAMETER(psDevMemContext);
	PVR_UNREFERENCED_PARAMETER(ui32Size);
	PVR_UNREFERENCED_PARAMETER(ePixelFormat);
	PVR_UNREFERENCED_PARAMETER(eMemFormat);
	PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);
	return PVRSRV_OK;
}
#endif				/* PDUMP */
#endif				/* ifndef __DEVICEMEM_SERVER_H__ */
