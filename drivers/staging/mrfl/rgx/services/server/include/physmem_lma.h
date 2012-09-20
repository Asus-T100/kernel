									     /**************************************************************************//*!
									        @File           physmem_lma.h
									        @Title          Header for local card memory allocator
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Part of the memory management. This module is responsible for
									        implementing the function callbacks for local card memory.
    *//***************************************************************************/

#ifndef _SRVSRV_PHYSMEM_LMA_H_
#define _SRVSRV_PHYSMEM_LMA_H_

/* include/ */
#include "img_types.h"
#include "pvrsrv_error.h"
#include "pvrsrv_memallocflags.h"

/* services/server/include/ */
#include "pmr.h"
#include "pmr_impl.h"

/*
 * PhysmemNewLocalRamBackedPMR
 *
 * This function will create a PMR using the local card memory and is OS
 * agnostic.
 */
PVRSRV_ERROR
PhysmemNewLocalRamBackedPMR(PVRSRV_DEVICE_NODE * psDevNode,
			    IMG_DEVMEM_SIZE_T uiSize,
			    IMG_UINT32 uiLog2PageSize,
			    PVRSRV_MEMALLOCFLAGS_T uiFlags, PMR ** ppsPMRPtr);

#endif				/* #ifndef _SRVSRV_PHYSMEM_LMA_H_ */
