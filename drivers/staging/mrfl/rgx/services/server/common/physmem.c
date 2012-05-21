									    /*************************************************************************//*!
									       @File           physmem.c
									       @Title          Physmem
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Common entry point for creation of RAM backed PMR's
									       @License        Strictly Confidential.
    *//***************************************************************************/
#include "img_types.h"
#include "pvrsrv_error.h"
#include "pvrsrv_memallocflags.h"
#include "device.h"
#include "physmem.h"
#include "pvrsrv.h"

PVRSRV_ERROR
PhysmemNewRamBackedPMR(PVRSRV_DEVICE_NODE * psDevNode,
		       IMG_DEVMEM_SIZE_T uiSize,
		       IMG_UINT32 uiLog2PageSize,
		       PVRSRV_MEMALLOCFLAGS_T uiFlags, PMR ** ppsPMRPtr)
{
	/********************************
	 * Sanity check the cache flags *
	 ********************************/
	/* Check if we can honour cached cache-coherent allocations */
	if ((CPU_CACHE_MODE(uiFlags) ==
	     PVRSRV_MEMALLOCFLAG_CPU_CACHED_CACHE_COHERENT)
	    && (!PVRSRVSystemHasCacheSnooping())) {
		return PVRSRV_ERROR_UNSUPPORTED_CACHE_MODE;
	}

	/* Both or neither have to be cache-coherent */
	if ((CPU_CACHE_MODE(uiFlags) ==
	     PVRSRV_MEMALLOCFLAG_CPU_CACHE_COHERENT) ^ (GPU_CACHE_MODE(uiFlags)
							==
							PVRSRV_MEMALLOCFLAG_GPU_CACHE_COHERENT))
	{
		return PVRSRV_ERROR_UNSUPPORTED_CACHE_MODE;
	}

	if ((CPU_CACHE_MODE(uiFlags) ==
	     PVRSRV_MEMALLOCFLAG_CPU_CACHED_CACHE_COHERENT) ^
	    (GPU_CACHE_MODE(uiFlags) ==
	     PVRSRV_MEMALLOCFLAG_GPU_CACHED_CACHE_COHERENT)) {
		return PVRSRV_ERROR_UNSUPPORTED_CACHE_MODE;
	}

	/*
	   FIXME:

	   Systems might have their own restrictions which we should also check.
	 */

	return psDevNode->pfnCreateRamBackedPMR(psDevNode,
						uiSize,
						uiLog2PageSize,
						uiFlags, ppsPMRPtr);
}
