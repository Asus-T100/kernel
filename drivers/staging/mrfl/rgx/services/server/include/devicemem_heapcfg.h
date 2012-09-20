/*!****************************************************************************
@File           devicemem2.h

@Title          Temporary Device Memory 2 stuff

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

@Description    Device memory management

@DoxygenVer		

******************************************************************************/

#ifndef __DEVICEMEMHEAPCFG_H__
#define __DEVICEMEMHEAPCFG_H__

#include "img_types.h"
#include "pvrsrv_error.h"

/* FIXME: Find a better way of defining _PVRSRV_DEVICE_NODE_ */
struct _PVRSRV_DEVICE_NODE_;

/*
  A "heap config" is a blueprint to be used for initial setting up of
  heaps when a device memory context is created.

  We define a data structure to define this, but it's really down to
  the caller to populate it.  This is all expected to be in-kernel.
  We provide an API that client code can use to enquire about the
  blueprint, such that it may do the heap setup during the context
  creation call on behalf of the user */

/* blueprint for a single heap */
typedef struct _DEVMEM_HEAP_BLUEPRINT_ {
	/* Name of this heap - for debug purposes, and perhaps for lookup
	   by name? */
	const IMG_CHAR *pszName;

	/* Virtual address of the beginning of the heap.  This _must_ be a
	   multiple of the data page size for the heap.  It is
	   _recommended_ that it be coarser than that - especially, it
	   should begin on a boundary appropriate to the MMU for the
	   device.  For Rogue, this is a Page Directory boundary, or 1GB
	   (virtual address a multiple of 0x0040000000). */
	IMG_DEV_VIRTADDR sHeapBaseAddr;

	/* Length of the heap.  Given that the END address of the heap has
	   a similar restriction to that of the _beginning_ of the heap.
	   That is the heap length _must_ be a whole number of data pages.
	   Again, the recommendation is that it ends on a 1GB boundary.
	   Again, this is not essential, but we do know that (at the time
	   of writing) the current implementation of mmu_common.c is such
	   that no two heaps may share a page directory, thus the
	   remaining virtual space would be wasted if the length were not
	   a multiple of 1GB */
	IMG_DEVMEM_SIZE_T uiHeapLength;

	/* Data page size.  This is the page size that is going to get
	   programmed into the MMU, so it needs to be a valid one for the
	   device.  Importantly, the start address and length _must_ be
	   multiples of this page size.  Note that the page size is
	   specified as the log 2 relative to 1 byte (e.g. 12 indicates
	   4kB) */
	IMG_UINT32 uiLog2DataPageSize;
} DEVMEM_HEAP_BLUEPRINT;

/* entire named heap config */
typedef struct _DEVMEM_HEAP_CONFIG_ {
	/* Name of this heap config - for debug and maybe lookup */
	const IMG_CHAR *pszName;

	/* Number of heaps in this config */
	IMG_UINT32 uiNumHeaps;

	/* Array of individual heap blueprints as defined above */
	DEVMEM_HEAP_BLUEPRINT *psHeapBlueprintArray;
} DEVMEM_HEAP_CONFIG;

extern PVRSRV_ERROR
HeapCfgHeapConfigCount(const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
		       IMG_UINT32 * puiNumHeapConfigsOut);

extern PVRSRV_ERROR
HeapCfgHeapCount(const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
		 IMG_UINT32 uiHeapConfigIndex, IMG_UINT32 * puiNumHeapsOut);

extern PVRSRV_ERROR
HeapCfgHeapConfigName(const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
		      IMG_UINT32 uiHeapConfigIndex,
		      IMG_UINT32 uiHeapConfigNameBufSz,
		      IMG_CHAR * pszHeapConfigNameOut);

extern PVRSRV_ERROR
HeapCfgHeapDetails(const struct _PVRSRV_DEVICE_NODE_ *psDeviceNode,
		   IMG_UINT32 uiHeapConfigIndex,
		   IMG_UINT32 uiHeapIndex,
		   IMG_UINT32 uiHeapNameBufSz,
		   IMG_CHAR * pszHeapNameOut,
		   IMG_DEV_VIRTADDR * psDevVAddrBaseOut,
		   IMG_DEVMEM_SIZE_T * puiHeapLengthOut,
		   IMG_UINT32 * puiLog2DataPageSizeOut);

#endif
