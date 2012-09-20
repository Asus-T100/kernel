/*!****************************************************************************
@File			handle.h

@Title			Handle Manager API

@Author			Imagination Technologies

@date   		14/05/07
 
@Copyright     	Copyright 2007 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either
                material or conceptual may be copied or distributed,
                transmitted, transcribed, stored in a retrieval system
                or translated into any human or computer language in any
                form by any means, electronic, mechanical, manual or
                other-wise, or disclosed to third parties without the
                express written permission of Imagination Technologies
                Limited, Unit 8, HomePark Industrial Estate,
                King's Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform		generic

@Description	Provide handle management

@DoxygenVer		

******************************************************************************/

#ifndef __HANDLE_H__
#define __HANDLE_H__

/*
 * Handle API
 * ----------
 * The handle API is intended to provide handles for kernel resources,
 * which can then be passed back to user space processes.
 *
 * The following functions comprise the API.  Each function takes a
 * pointer to a PVRSRV_HANDLE_BASE strcture, one of which is allocated
 * for each process, and stored in the per-process data area.  Use
 * KERNEL_HANDLE_BASE for handles not allocated for a particular process,
 * or for handles that need to be allocated before the PVRSRV_HANDLE_BASE
 * structure for the process is available.
 *
 * PVRSRV_ERROR PVRSRVAllocHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_HANDLE *phHandle, IMG_VOID *pvData, PVRSRV_HANDLE_TYPE eType,
 * 	PVRSRV_HANDLE_ALLOC_FLAG eFlag);
 *
 * Allocate a handle phHandle, for the resource of type eType pointed to by
 * pvData.
 *
 * For handles that have a definite lifetime, where the corresponding
 * resource is explicitly created and destroyed, eFlag should be zero.
 *
 * If the resource is not explicitly created and destroyed, eFlag should be
 * set to PVRSRV_HANDLE_ALLOC_FLAG_SHARED.  For a given process, the same
 * handle will be returned each time a handle for the resource is allocated
 * with the PVRSRV_HANDLE_ALLOC_FLAG_SHARED flag.
 *
 * If a particular resource may be referenced multiple times by a
 * given process, setting eFlag to PVRSRV_HANDLE_ALLOC_FLAG_MULTI
 * will allow multiple handles to be allocated for the resource.
 * Such handles cannot be found with PVRSRVFindHandle.
 *
 * PVRSRV_ERROR PVRSRVAllocSubHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_HANDLE *phHandle, IMG_VOID *pvData, PVRSRV_HANDLE_TYPE eType,
 * 	PVRSRV_HANDLE_ALLOC_FLAG eFlag, IMG_HANDLE hParent);
 *
 * This function is similar to PVRSRVAllocHandle, except that the allocated
 * handles are associated with a parent handle, hParent, that has been
 * allocated previously.  Subhandles are automatically deallocated when their
 * parent handle is dealloacted.
 * Subhandles can be treated as ordinary handles.  For example, they may
 * have subhandles of their own, and may be explicity deallocated using
 * PVRSRVReleaseHandle (see below).
 *
 * PVRSRV_ERROR PVRSRVFindHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_HANDLE *phHandle, IMG_VOID *pvData, PVRSRV_HANDLE_TYPE eType);
 *
 * Find the handle previously allocated for the resource pointed to by
 * pvData, of type eType.  Handles allocated with the flag
 * PVRSRV_HANDLE_ALLOC_FLAG_MULTI cannot be found using this
 * function.
 *
 * PVRSRV_ERROR PVRSRVLookupHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_PVOID *ppvData, IMG_HANDLE hHandle, PVRSRV_HANDLE_TYPE eType);
 *
 * Given a handle for a resource of type eType, return the pointer to the
 * resource.
 *
 * PVRSRV_ERROR PVRSRVLookuSubHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_PVOID *ppvData, IMG_HANDLE hHandle, PVRSRV_HANDLE_TYPE eType,
 * 	IMH_HANDLE hAncestor);
 *
 * Similar to PVRSRVLookupHandle, but checks the handle is a descendent
 * of hAncestor.
 *
 * PVRSRV_ERROR PVRSRVLookupHandleAnyType(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_PVOID *ppvData, PVRSRV_HANDLE_TYPE *peType, IMG_HANDLE hHandle);
 *
 * This function returns the resource pointer corresponding to the
 * given handle, and the resource type in peType.  This function is
 * intended for situations where a handle may be one of several types,
 * but the type isn't known beforehand.
 *
 * PVRSRV_ERROR PVRSRVReleaseHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_HANDLE hHandle, PVRSRV_HANDLE_TYPE eType);
 *
 * Deallocate a handle of given type.
 *
 * PVRSRV_ERROR PVRSRVLookupAndReleaseHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_PVOID *ppvData, IMG_HANDLE hHandle, PVRSRV_HANDLE_TYPE eType);
 *
 * This function combines the functionality of PVRSRVLookupHandle and
 * PVRSRVReleaseHandle, deallocating the handle after looking it up.
 *
 * PVRSRV_ERROR PVRSRVGetParentHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_PVOID *phParent, IMG_HANDLE hHandle, PVRSRV_HANDLE_TYPE eType);
 *
 * Return the parent of a handle in *phParent, or IMG_NULL if the handle has
 * no parent.
 *
 * PVRSRV_ERROR PVRSRVNewHandleBatch(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_UINT32 ui32BatchSize)
 *
 * Allocate a new handle batch.  This preallocates ui32BatchSize handles.
 * Batch mode simplifies the handling of handle allocation failures.
 * The handle API is unchanged in batch mode, except that handles freed
 * in batch mode will not be available for reallocation until the batch
 * is committed or released (see below).
 *
 * PVRSRV_ERROR PVRSRVCommitHandleBatch(PVRSRV_HANDLE_BASE *psBase)
 * void PVRSRVReleaseHandleBatch(PVRSRV_HANDLE_BASE *psBase)
 *
 * When handle allocation from a handle batch is complete, the
 * batch must be committed by calling PVRSRVCommitHandleBatch.  If
 * an error occurred, and none of the handles in the batch are no
 * longer needed, PVRSRVReleaseHandleBatch must be called.
 * The macros PVRSRVAllocHandleNR, and PVRSRVAllocSubHandleNR
 * are defined for use in batch mode.  These work the same way
 * as PVRSRVAllocHandle and PVRSRVAllocSubHandle, except that
 * they don't return a value, relying on the fact that
 * PVRSRVCommitHandleBatch will not commit any of the handles
 * in a batch if there was an error allocating one of the
 * handles in the batch.
 *
 * PVRSRV_ERROR PVRSRVSetMaxHandle(PVRSRV_HANDLE_BASE *psBase,
 * 	IMG_UINT32 ui32MaxHandle)
 * Set the maximum handle number.  This is intended to restrict the
 * handle range so that it will fit within a given field width.  For
 * example, setting the maximum handle number to 0x7fffffff, would
 * ensure the handles would fit within a 31 bit width field.  This
 * facility should be used with caution, as it restricts the number of
 * handles that can be allocated.
 *
 * IMG_UINT32 PVRSRVGetMaxHandle(PVRSRV_HANDLE_BASE *psBase)
 * Return the maximum handle number, or 0 if the setting of a limit
 * is not supported.
 *
 * PVRSRV_ERROR PVRSRVEnableHandlePurging(PVRSRV_HANDLE_BASE *psBase)
 * Allows unused handle space to be reclaimed, by calling
 * PVRSRVPurgeHandles.  Note that allocating handles may have a
 * higher overhead if purging is enabled.
 *
 * PVRSRV_ERROR PVRSRVPurgeHandles((PVRSRV_HANDLE_BASE *psBase)
 * Purge handles for a handle base that has purging enabled.
 */

#if defined (__cplusplus)
extern "C" {
#endif

#include "img_types.h"
#include "hash.h"
#include "resman.h"

	typedef enum {
		PVRSRV_HANDLE_TYPE_NONE = 0,
		PVRSRV_HANDLE_TYPE_PERPROC_DATA,
		PVRSRV_HANDLE_TYPE_DEV_NODE,
		PVRSRV_HANDLE_TYPE_DEV_MEM_CONTEXT,
		PVRSRV_HANDLE_TYPE_DEV_MEM_HEAP,
		PVRSRV_HANDLE_TYPE_MEM_INFO,
		PVRSRV_HANDLE_TYPE_SYNC_INFO,
		PVRSRV_HANDLE_TYPE_DISP_INFO,
		PVRSRV_HANDLE_TYPE_DISP_SWAP_CHAIN,
		PVRSRV_HANDLE_TYPE_BUF_INFO,
		PVRSRV_HANDLE_TYPE_DEVICE_BUFFER,
		PVRSRV_HANDLE_TYPE_RGX_HW_TRANSFER_CONTEXT,
		PVRSRV_HANDLE_TYPE_RGX_HW_2D_CONTEXT,
		PVRSRV_HANDLE_TYPE_SHARED_PB_DESC,
		PVRSRV_HANDLE_TYPE_MEM_INFO_REF,
		PVRSRV_HANDLE_TYPE_SHARED_SYS_MEM_INFO,
		PVRSRV_HANDLE_TYPE_SHARED_EVENT_OBJECT,
		PVRSRV_HANDLE_TYPE_EVENT_OBJECT_CONNECT,
		PVRSRV_HANDLE_TYPE_MMAP_INFO,
		PVRSRV_HANDLE_TYPE_SOC_TIMER,
		PVRSRV_HANDLE_TYPE_SYNC_INFO_MOD_OBJ,
		PVRSRV_HANDLE_TYPE_PHYSMEM_CTX,
		PVRSRV_HANDLE_TYPE_PHYSMEM_PMR,
		PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_EXPORT,
		PVRSRV_HANDLE_TYPE_PHYSMEM_MMAP_DATA,
		PVRSRV_HANDLE_TYPE_DEVMEMINT_CTX,
		PVRSRV_HANDLE_TYPE_DEVMEMINT_HEAP,
		PVRSRV_HANDLE_TYPE_DEVMEMINT_RESERVATION,
		PVRSRV_HANDLE_TYPE_DEVMEMINT_MAPPING,
		PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
		PVRSRV_HANDLE_TYPE_RGX_RC_CLEANUP,
		PVRSRV_HANDLE_TYPE_RGX_RTDATA_CLEANUP,
		PVRSRV_HANDLE_TYPE_RGX_TQ3D_CLEANUP,
		PVRSRV_HANDLE_TYPE_RGX_TQ2D_CLEANUP,
		PVRSRV_HANDLE_TYPE_RGX_CC_CLEANUP,
		PVRSRV_HANDLE_TYPE_RGX_CCB_CLEANUP,
		PVRSRV_HANDLE_TYPE_SERVER_EXPORTCOOKIE,
		PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE,
		PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK,
		PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE,
		PVRSRV_HANDLE_TYPE_SERVER_SYNC_EXPORT,
		PVRSRV_HANDLE_TYPE_SERVER_OP_COOKIE,
		PVRSRV_HANDLE_TYPE_RGX_FWIF_HWRTDATA,
		PVRSRV_HANDLE_TYPE_RGX_FWIF_RENDERTARGET,
		PVRSRV_HANDLE_TYPE_RGX_FWIF_FREELIST,
		PVRSRV_HANDLE_TYPE_DC_DEVICE,
		PVRSRV_HANDLE_TYPE_DC_DISPLAY_CONTEXT,
		PVRSRV_HANDLE_TYPE_DC_BUFFER,
		PVRSRV_HANDLE_TYPE_DC_PIN_HANDLE,
		PVRSRV_HANDLE_TYPE_DEVMEM_MEM_IMPORT,
		PVRSRV_HANDLE_TYPE_DEVMEM_MEM_EXPORT,
		PVRSRV_HANDLE_TYPE_PHYSMEM_PMR_PAGELIST,
	} PVRSRV_HANDLE_TYPE;

	typedef enum {
		/* No flags */
		PVRSRV_HANDLE_ALLOC_FLAG_NONE = 0,
		/* Share a handle that already exists for a given data pointer */
		PVRSRV_HANDLE_ALLOC_FLAG_SHARED = 0x01,
		/* Muliple handles can point at the given data pointer */
		PVRSRV_HANDLE_ALLOC_FLAG_MULTI = 0x02,
		/* Subhandles are allocated in a private handle space */
		PVRSRV_HANDLE_ALLOC_FLAG_PRIVATE = 0x04
	} PVRSRV_HANDLE_ALLOC_FLAG;

	struct _PVRSRV_HANDLE_BASE_;
	typedef struct _PVRSRV_HANDLE_BASE_ PVRSRV_HANDLE_BASE;

#ifdef	PVR_SECURE_HANDLES
	extern PVRSRV_HANDLE_BASE *gpsKernelHandleBase;

#define	KERNEL_HANDLE_BASE (gpsKernelHandleBase)

	PVRSRV_ERROR PVRSRVAllocHandle(PVRSRV_HANDLE_BASE * psBase,
				       IMG_HANDLE * phHandle, IMG_VOID * pvData,
				       PVRSRV_HANDLE_TYPE eType,
				       PVRSRV_HANDLE_ALLOC_FLAG eFlag);

	PVRSRV_ERROR PVRSRVAllocSubHandle(PVRSRV_HANDLE_BASE * psBase,
					  IMG_HANDLE * phHandle,
					  IMG_VOID * pvData,
					  PVRSRV_HANDLE_TYPE eType,
					  PVRSRV_HANDLE_ALLOC_FLAG eFlag,
					  IMG_HANDLE hParent);

	PVRSRV_ERROR PVRSRVFindHandle(PVRSRV_HANDLE_BASE * psBase,
				      IMG_HANDLE * phHandle, IMG_VOID * pvData,
				      PVRSRV_HANDLE_TYPE eType);

	PVRSRV_ERROR PVRSRVLookupHandleAnyType(PVRSRV_HANDLE_BASE * psBase,
					       IMG_PVOID * ppvData,
					       PVRSRV_HANDLE_TYPE * peType,
					       IMG_HANDLE hHandle);

	PVRSRV_ERROR PVRSRVLookupHandle(PVRSRV_HANDLE_BASE * psBase,
					IMG_PVOID * ppvData, IMG_HANDLE hHandle,
					PVRSRV_HANDLE_TYPE eType);

	PVRSRV_ERROR PVRSRVLookupSubHandle(PVRSRV_HANDLE_BASE * psBase,
					   IMG_PVOID * ppvData,
					   IMG_HANDLE hHandle,
					   PVRSRV_HANDLE_TYPE eType,
					   IMG_HANDLE hAncestor);

	PVRSRV_ERROR PVRSRVGetParentHandle(PVRSRV_HANDLE_BASE * psBase,
					   IMG_PVOID * phParent,
					   IMG_HANDLE hHandle,
					   PVRSRV_HANDLE_TYPE eType);

	PVRSRV_ERROR PVRSRVLookupAndReleaseHandle(PVRSRV_HANDLE_BASE * psBase,
						  IMG_PVOID * ppvData,
						  IMG_HANDLE hHandle,
						  PVRSRV_HANDLE_TYPE eType);

	PVRSRV_ERROR PVRSRVReleaseHandle(PVRSRV_HANDLE_BASE * psBase,
					 IMG_HANDLE hHandle,
					 PVRSRV_HANDLE_TYPE eType);

	PVRSRV_ERROR PVRSRVNewHandleBatch(PVRSRV_HANDLE_BASE * psBase,
					  IMG_UINT32 ui32BatchSize);

	PVRSRV_ERROR PVRSRVCommitHandleBatch(PVRSRV_HANDLE_BASE * psBase);

	IMG_VOID PVRSRVReleaseHandleBatch(PVRSRV_HANDLE_BASE * psBase);

	PVRSRV_ERROR PVRSRVSetMaxHandle(PVRSRV_HANDLE_BASE * psBase,
					IMG_UINT32 ui32MaxHandle);

	IMG_UINT32 PVRSRVGetMaxHandle(PVRSRV_HANDLE_BASE * psBase);

	PVRSRV_ERROR PVRSRVEnableHandlePurging(PVRSRV_HANDLE_BASE * psBase);

	PVRSRV_ERROR PVRSRVPurgeHandles(PVRSRV_HANDLE_BASE * psBase);

	PVRSRV_ERROR PVRSRVAllocHandleBase(PVRSRV_HANDLE_BASE ** ppsBase);

	PVRSRV_ERROR PVRSRVFreeHandleBase(PVRSRV_HANDLE_BASE * psBase);

	PVRSRV_ERROR PVRSRVHandleInit(IMG_VOID);

	PVRSRV_ERROR PVRSRVHandleDeInit(IMG_VOID);

#else				/* #ifdef  PVR_SECURE_HANDLES */

#define KERNEL_HANDLE_BASE IMG_NULL

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVAllocHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVAllocHandle(PVRSRV_HANDLE_BASE * psBase,
					   IMG_HANDLE * phHandle,
					   IMG_VOID * pvData,
					   PVRSRV_HANDLE_TYPE eType,
					   PVRSRV_HANDLE_ALLOC_FLAG eFlag) {
		PVR_UNREFERENCED_PARAMETER(eType);
		PVR_UNREFERENCED_PARAMETER(eFlag);
		PVR_UNREFERENCED_PARAMETER(psBase);

		*phHandle = pvData;
		return PVRSRV_OK;
	}
#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVAllocSubHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVAllocSubHandle(PVRSRV_HANDLE_BASE * psBase,
					      IMG_HANDLE * phHandle,
					      IMG_VOID * pvData,
					      PVRSRV_HANDLE_TYPE eType,
					      PVRSRV_HANDLE_ALLOC_FLAG eFlag,
					      IMG_HANDLE hParent) {
		PVR_UNREFERENCED_PARAMETER(eType);
		PVR_UNREFERENCED_PARAMETER(eFlag);
		PVR_UNREFERENCED_PARAMETER(hParent);
		PVR_UNREFERENCED_PARAMETER(psBase);

		*phHandle = pvData;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVFindHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVFindHandle(PVRSRV_HANDLE_BASE * psBase,
					  IMG_HANDLE * phHandle,
					  IMG_VOID * pvData,
					  PVRSRV_HANDLE_TYPE eType) {
		PVR_UNREFERENCED_PARAMETER(eType);
		PVR_UNREFERENCED_PARAMETER(psBase);

		*phHandle = pvData;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVLookupHandleAnyType)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVLookupHandleAnyType(PVRSRV_HANDLE_BASE * psBase,
						   IMG_PVOID * ppvData,
						   PVRSRV_HANDLE_TYPE * peType,
						   IMG_HANDLE hHandle) {
		PVR_UNREFERENCED_PARAMETER(psBase);
		/*
		 * Unlike the other functions here, the returned results will need
		 * to be handled differently for the secure and non-secure cases.
		 */
		*peType = PVRSRV_HANDLE_TYPE_NONE;

		*ppvData = hHandle;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVLookupHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVLookupHandle(PVRSRV_HANDLE_BASE * psBase,
					    IMG_PVOID * ppvData,
					    IMG_HANDLE hHandle,
					    PVRSRV_HANDLE_TYPE eType) {
		PVR_UNREFERENCED_PARAMETER(psBase);
		PVR_UNREFERENCED_PARAMETER(eType);

		*ppvData = hHandle;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVLookupSubHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVLookupSubHandle(PVRSRV_HANDLE_BASE * psBase,
					       IMG_PVOID * ppvData,
					       IMG_HANDLE hHandle,
					       PVRSRV_HANDLE_TYPE eType,
					       IMG_HANDLE hAncestor) {
		PVR_UNREFERENCED_PARAMETER(psBase);
		PVR_UNREFERENCED_PARAMETER(eType);
		PVR_UNREFERENCED_PARAMETER(hAncestor);

		*ppvData = hHandle;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVGetParentHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVGetParentHandle(PVRSRV_HANDLE_BASE * psBase,
					       IMG_PVOID * phParent,
					       IMG_HANDLE hHandle,
					       PVRSRV_HANDLE_TYPE eType) {
		PVR_UNREFERENCED_PARAMETER(psBase);
		PVR_UNREFERENCED_PARAMETER(eType);
		PVR_UNREFERENCED_PARAMETER(hHandle);

		*phParent = IMG_NULL;

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVLookupAndReleaseHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVLookupAndReleaseHandle(PVRSRV_HANDLE_BASE *
						      psBase,
						      IMG_PVOID * ppvData,
						      IMG_HANDLE hHandle,
						      PVRSRV_HANDLE_TYPE eType)
	{
		PVR_UNREFERENCED_PARAMETER(eType);
		PVR_UNREFERENCED_PARAMETER(psBase);

		*ppvData = hHandle;
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVReleaseHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVReleaseHandle(PVRSRV_HANDLE_BASE * psBase,
					     IMG_HANDLE hHandle,
					     PVRSRV_HANDLE_TYPE eType) {
		PVR_UNREFERENCED_PARAMETER(hHandle);
		PVR_UNREFERENCED_PARAMETER(eType);
		PVR_UNREFERENCED_PARAMETER(psBase);

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVNewHandleBatch)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVNewHandleBatch(PVRSRV_HANDLE_BASE * psBase,
					      IMG_UINT32 ui32BatchSize) {
		PVR_UNREFERENCED_PARAMETER(psBase);
		PVR_UNREFERENCED_PARAMETER(ui32BatchSize);

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVCommitHandleBatch)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVCommitHandleBatch(PVRSRV_HANDLE_BASE * psBase) {
		PVR_UNREFERENCED_PARAMETER(psBase);

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVReleaseHandleBatch)
#endif
	static INLINE
	    IMG_VOID PVRSRVReleaseHandleBatch(PVRSRV_HANDLE_BASE * psBase) {
		PVR_UNREFERENCED_PARAMETER(psBase);
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSetMaxHandle)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVSetMaxHandle(PVRSRV_HANDLE_BASE * psBase,
					    IMG_UINT32 ui32MaxHandle) {
		PVR_UNREFERENCED_PARAMETER(psBase);
		PVR_UNREFERENCED_PARAMETER(ui32MaxHandle);

		return PVRSRV_ERROR_NOT_SUPPORTED;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVGetMaxHandle)
#endif
	static INLINE IMG_UINT32 PVRSRVGetMaxHandle(PVRSRV_HANDLE_BASE * psBase) {
		PVR_UNREFERENCED_PARAMETER(psBase);

		return 0;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVEnableHandlePurging)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVEnableHandlePurging(PVRSRV_HANDLE_BASE * psBase)
	{
		PVR_UNREFERENCED_PARAMETER(psBase);

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPurgeHandles)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVPurgeHandles(PVRSRV_HANDLE_BASE * psBase) {
		PVR_UNREFERENCED_PARAMETER(psBase);

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVAllocHandleBase)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVAllocHandleBase(PVRSRV_HANDLE_BASE ** ppsBase) {
		*ppsBase = IMG_NULL;

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVFreeHandleBase)
#endif
	static INLINE
	    PVRSRV_ERROR PVRSRVFreeHandleBase(PVRSRV_HANDLE_BASE * psBase) {
		PVR_UNREFERENCED_PARAMETER(psBase);

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVHandleInit)
#endif
	static INLINE PVRSRV_ERROR PVRSRVHandleInit(IMG_VOID) {
		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVHandleDeInit)
#endif
	static INLINE PVRSRV_ERROR PVRSRVHandleDeInit(IMG_VOID) {
		return PVRSRV_OK;
	}

#endif				/* #ifdef  PVR_SECURE_HANDLES */

/*
 * Versions of PVRSRVAllocHandle and PVRSRVAllocSubHandle with no return
 * values.  Intended for use with batched handle allocation, relying on
 * CommitHandleBatch to detect handle allocation errors.
 */
#define PVRSRVAllocHandleNR(psBase, phHandle, pvData, eType, eFlag) \
	(IMG_VOID)PVRSRVAllocHandle(psBase, phHandle, pvData, eType, eFlag)

#define PVRSRVAllocSubHandleNR(psBase, phHandle, pvData, eType, eFlag, hParent) \
	(IMG_VOID)PVRSRVAllocSubHandle(psBase, phHandle, pvData, eType, eFlag, hParent)

#if defined (__cplusplus)
}
#endif

#endif				/* __HANDLE_H__ */

/******************************************************************************
 End of file (handle.h)
******************************************************************************/
