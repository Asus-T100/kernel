									     /**************************************************************************//*!
									        @File           physmem.h
									        @Title          Physmem header
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Header for common entry point for creation of RAM backed PMR's
    *//***************************************************************************/

#ifndef _SRVSRV_PHYSMEM_H_
#define _SRVSRV_PHYSMEM_H_

/* include/ */
#include "img_types.h"
#include "pvrsrv_error.h"
#include "pvrsrv_memallocflags.h"

/* services/server/include/ */
#include "pmr.h"
#include "pmr_impl.h"

/*
 * PhysmemNewRamBackedPMR
 *
 * This function will create a RAM backed PMR using the device specific
 * callback, this allows control at a per-devicenode level to select the
 * memory source thus supporting mixed UMA/LMA systems.
 *
 * The size must be a multiple of page size.  The page size is
 * specified in log2.  It should be regarded as a minimum contiguity
 * of which the that the resulting memory must be a multiple.  It may
 * be that this should be a fixed number.  It may be that the
 * allocation size needs to be a multiple of some coarser "page size"
 * than that specified in the page size argument.  For example, take
 * an OS whose page granularity is a fixed 16kB, but the caller
 * requests memory in page sizes of 4kB.  The request can be satisfied
 * if and only if the SIZE requested is a multiple of 16kB.  If the
 * arguments supplied are such that this OS cannot grant the request,
 * PVRSRV_ERROR_INVALID_PARAMS will be returned.
 *
 * The caller should supply storage of a pointer.  Upon successful
 * return a PMR object will have been created and a pointer to it
 * returned in the PMROut argument.
 *
 * A PMR thusly created should be destroyed with PhysmemUnrefPMR.
 *
 * Note that this function may cause memory allocations and on some
 * OSes this may cause scheduling events, so it is important that this
 * function be called with interrupts enabled and in a context where
 * scheduling events and memory allocations are permitted.
 *
 * The flags may be used by the implementation to change its behaviour
 * if required.  The flags will also be stored in the PMR as immutable
 * metadata and returned to mmu_common when it asks for it.
 *
 */
extern PVRSRV_ERROR
PhysmemNewRamBackedPMR(PVRSRV_DEVICE_NODE * psDevNode,
		       IMG_DEVMEM_SIZE_T uiSize,
		       IMG_UINT32 uiLog2PageSize,
		       PVRSRV_MEMALLOCFLAGS_T uiFlags, PMR ** ppsPMROut);

#endif				/* _SRVSRV_PHYSMEM_H_ */
