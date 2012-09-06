/*!****************************************************************************
@File           device.h

@Title          Common Device header

@Author         Imagination Technologies

@Date           02 / 06 /04

@Copyright      Copyright 2004-2008 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either material
                or conceptual may be copied or distributed, transmitted,
                transcribed, stored in a retrieval system or translated into
                any human or computer language in any form by any means,
                electronic, mechanical, manual or otherwise, or disclosed
                to third parties without the express written permission of
                Imagination Technologies Limited, Home Park Estate,
                Kings Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform       Generic

@Description    Device related function templates and defines

@DoxygenVer		

******************************************************************************/

#ifndef __DEVICE_H__
#define __DEVICE_H__

#if defined(__cplusplus)
extern "C" {
#endif

#include "devicemem_heapcfg.h"
#include "mmu_common.h"
#include "ra.h"			/* RA_ARENA */
#include "resman.h"		/* PRESMAN_ITEM */
#include "pvrsrv_device.h"
#include "srvkm.h"
#include "devicemem.h"
#include "physheap.h"
#include "sync.h"
#include "dllist.h"
#include "cache_external.h"

/* BM context forward reference */
	typedef struct _BM_CONTEXT_ BM_CONTEXT;

	/*********************************************************************//*!
	   @Function      AllocUFOCallback
	   @Description   Device specific callback for allocation of an UFO

	   @Input         psDeviceNode    Pointer to device node to allocate
	   the UFO for.
	   @Output        ppsMemDesc      Pointer to pointer for the memdesc of
	   the allocation

	   @Return        PVRSRV_OK if allocation was successfull
	 */
/*********************************************************************/
	typedef PVRSRV_ERROR(*AllocUFOBlockCallback) (struct
						      _PVRSRV_DEVICE_NODE_ *
						      psDeviceNode,
						      DEVMEM_MEMDESC **
						      ppsMemDesc,
						      IMG_UINT32 *
						      pui32SyncAddr,
						      IMG_UINT32 *
						      puiSyncPrimBlockSize);

	/*********************************************************************//*!
	   @Function      FreeUFOCallback
	   @Description   Device specific callback for freeing of an UFO

	   @Input         psDeviceNode    Pointer to device node to allocate
	   the UFO for.
	   @Input         psMemDesc       Pointer to pointer for the memdesc of
	   the allocation
	 */
/*********************************************************************/
	typedef IMG_VOID(*FreeUFOBlockCallback) (DEVMEM_MEMDESC * psMemDesc);

	/*********************************************************************//*!
	   @Function      CPUCacheOperation
	   @Description   Request a cpu cache operation to happen before the next
	   operation on that device is queued.

	   @Input         psDeviceNode    Pointer to device node to request
	   the flush on.
	 */
/*********************************************************************/
	typedef IMG_VOID(*CPUCacheOperation) (struct _PVRSRV_DEVICE_NODE_ *
					      psDeviceNode,
					      PVRSRV_CACHE_OP eCacheOp);

	typedef struct _DEVICE_MEMORY_INFO_ {
		/* size of address space, as log2 */
		IMG_UINT32 ui32AddressSpaceSizeLog2;

		/* 
		   flags, includes physical memory resource types available to the system.  
		   Allows for validation at heap creation, define PVRSRV_BACKINGSTORE_XXX 
		 */
		IMG_UINT32 ui32Flags;

		/* heap count.  Doesn't include additional heaps from PVRSRVCreateDeviceMemHeap */
		IMG_UINT32 ui32HeapCount;

		/* BM kernel context for the device */
		BM_CONTEXT *pBMKernelContext;

		/* BM context list for the device */
		BM_CONTEXT *pBMContext;

		/* Blueprints for creating new device memory contexts */
		IMG_UINT32 uiNumHeapConfigs;
		DEVMEM_HEAP_CONFIG *psDeviceMemoryHeapConfigArray;
		/* naughty naughty - the following ought not the be exposed */
		DEVMEM_HEAP_BLUEPRINT *psDeviceMemoryHeap;
	} DEVICE_MEMORY_INFO;

	typedef struct _Px_HANDLE_ {
		union {
			IMG_VOID *pvHandle;
			IMG_UINT64 ui64Handle;
		} u;
	} Px_HANDLE;

	typedef enum _PVRSVR_DEVICE_STATE_ {
		PVRSVR_DEVICE_STATE_UNDEFINED = 0,
		PVRSVR_DEVICE_STATE_INIT,
		PVRSVR_DEVICE_STATE_ACTIVE,
		PVRSVR_DEVICE_STATE_DEINIT,
	} PVRSVR_DEVICE_STATE;

#define PRVSRV_DEVICE_FLAGS_LMA		(1 << 0)

	typedef struct _PVRSRV_DEVICE_NODE_ {
		PVRSRV_DEVICE_IDENTIFIER sDevId;
		IMG_UINT32 ui32RefCount;

		PVRSVR_DEVICE_STATE eDevState;

		/* device specific MMU attributes */
		MMU_DEVICEATTRIBS *psMMUDevAttrs;

		/*
		   callbacks the device must support:
		 */

		 PVRSRV_ERROR(*pfnCreateRamBackedPMR) (struct
						       _PVRSRV_DEVICE_NODE_ *
						       psDevNode,
						       IMG_DEVMEM_SIZE_T uiSize,
						       IMG_UINT32
						       uiLog2PageSize,
						       PVRSRV_MEMALLOCFLAGS_T
						       uiFlags,
						       PMR ** ppsPMRPtr);

		 PVRSRV_ERROR(*pfnMMUPxAlloc) (struct _PVRSRV_DEVICE_NODE_ *
					       psDevNode, IMG_SIZE_T uiSize,
					       Px_HANDLE * psMemHandle,
					       IMG_DEV_PHYADDR * psDevPAddr);

		 IMG_VOID(*pfnMMUPxFree) (struct _PVRSRV_DEVICE_NODE_ *
					  psDevNode, Px_HANDLE * psMemHandle);

		 PVRSRV_ERROR(*pfnMMUPxMap) (struct _PVRSRV_DEVICE_NODE_ *
					     psDevNode,
					     Px_HANDLE * pshMemHandle,
					     IMG_SIZE_T uiSize,
					     IMG_DEV_PHYADDR * psDevPAddr,
					     IMG_VOID ** pvPtr);

		 IMG_VOID(*pfnMMUPxUnmap) (struct _PVRSRV_DEVICE_NODE_ *
					   psDevNode, Px_HANDLE * psMemHandle,
					   IMG_VOID * pvPtr);
		IMG_UINT32 uiMMUPxLog2AllocGran;
		IMG_CHAR *pszMMUPxPDumpMemSpaceName;

		 IMG_VOID(*pfnMMUCacheInvalidate) (struct _PVRSRV_DEVICE_NODE_ *
						   psDevNode,
						   IMG_HANDLE hDeviceData,
						   MMU_LEVEL eLevel,
						   IMG_BOOL bUnmap);

		 PVRSRV_ERROR(*pfnSLCCacheInvalidateRequest) (struct
							      _PVRSRV_DEVICE_NODE_
							      * psDevNode,
							      PMR * psPmr);

		 IMG_VOID(*pfnDumpDebugInfo) (struct _PVRSRV_DEVICE_NODE_ *
					      psDevNode);

		PVRSRV_DEVICE_CONFIG *psDevConfig;

		/* device post-finalise compatibility check */
		 PVRSRV_ERROR(*pfnInitDeviceCompatCheck) (struct
							  _PVRSRV_DEVICE_NODE_
							  *);

		/* Flag indicating that command complete callback needs to be reprocessed */
		IMG_BOOL bReProcessDeviceCommandComplete;

		/* information about the device's address space and heaps */
		DEVICE_MEMORY_INFO sDevMemoryInfo;

		/* private device information */
		IMG_VOID *pvDevice;
		IMG_UINT32 ui32pvDeviceSize;	/* required by GetClassDeviceInfo API */

		/* Resource Manager Context */
		PRESMAN_CONTEXT hResManContext;

		IMG_UINT32 ui32Flags;

		IMG_CHAR szRAName[50];

		RA_ARENA *psLocalDevMemArena;

		PHYS_HEAP *psPhysHeap;

		struct _PVRSRV_DEVICE_NODE_ *psNext;
		struct _PVRSRV_DEVICE_NODE_ **ppsThis;

		/* Functions for notification about memory contexts */
		 PVRSRV_ERROR(*pfnRegisterMemoryContext) (struct
							  _PVRSRV_DEVICE_NODE_ *
							  psDeviceNode,
							  MMU_CONTEXT *
							  psMMUContext,
							  IMG_HANDLE *
							  hPrivData);
		 IMG_VOID(*pfnUnregisterMemoryContext) (IMG_HANDLE hPrivData);

		/* Funtions for allocation/freeing of UFOs */
		AllocUFOBlockCallback pfnAllocUFOBlock;	/*!< Callback for allocation of a block of UFO memory */
		FreeUFOBlockCallback pfnFreeUFOBlock;	/*!< Callback for freeing of a block of UFO memory */

		PSYNC_PRIM_CONTEXT hSyncPrimContext;

		CPUCacheOperation pfnCPUCacheOperation;

#if defined(PDUMP)
		/*      device-level callback which is called when pdump.exe starts.
		 *      Should be implemented in device-specific init code, e.g. rgxinit.c
		 */
		 PVRSRV_ERROR(*pfnPDumpInitDevice) (struct _PVRSRV_DEVICE_NODE_
						    * psDeviceNode);
		/* device-level callback to return pdump ID associated to a memory context */
		 IMG_UINT32(*pfnMMUGetContextID) (IMG_HANDLE hDevMemContext);
#endif
	} PVRSRV_DEVICE_NODE;

	PVRSRV_ERROR IMG_CALLCONV PVRSRVFinaliseSystem(IMG_BOOL bInitSuccesful);

	PVRSRV_ERROR IMG_CALLCONV PVRSRVDevInitCompatCheck(PVRSRV_DEVICE_NODE *
							   psDeviceNode);

#if defined(__cplusplus)
}
#endif
#endif				/* __DEVICE_H__ */
/******************************************************************************
 End of file (device.h)
******************************************************************************/
