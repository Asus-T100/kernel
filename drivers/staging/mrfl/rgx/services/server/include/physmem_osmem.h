/*!****************************************************************************
@File		physmem_osmem.h

@Title		PMR implementation of OS derived physical memory

@Author		Imagination Technologies

@Copyright     	Copyright 2010 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either
                material or conceptual may be copied or distributed,
                transmitted, transcribed, stored in a retrieval system
                or translated into any human or computer language in any
                form by any means, electronic, mechanical, manual or
                other-wise, or disclosed to third parties without the
                express written permission of Imagination Technologies
                Limited, Unit 8, HomePark Industrial Estate,
                King's Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform	generic

@Description	Part of the memory management.  This module is
                responsible for the an implementation of the "PMR"
                abstraction.  This interface is for the
                PhysmemNewOSRamBackedPMR() "PMR Factory" which is
                responsible for claiming chunks of memory (in
                particular physically contiguous quanta) from the
                Operating System.

                As such, this interface will be implemented on a
                Per-OS basis, in the "env" directory for that system.
                A dummy implementation is available in
                physmem_osmem_dummy.c for operating systems that
                cannot, or do not wish to, offer this functionality.

@DoxygenVer

******************************************************************************/

#ifndef _SRVSRV_PHYSMEM_OSMEM_H_
#define _SRVSRV_PHYSMEM_OSMEM_H_

/* include/ */
#include "img_types.h"
#include "pvrsrv_error.h"
#include "pvrsrv_memallocflags.h"

/* services/server/include/ */
#include "pmr.h"
#include "pmr_impl.h"

/*
 * PhysmemNewOSRamBackedPMR
 *
 * To be overridden on a per-OS basis.
 *
 * This function will create a PMR using the default "OS supplied" physical pages
 * method, assuming such is available on a particular operating system.  (If not,
 * PVRSRV_ERROR_NOT_SUPPORTED should be returned)
 */
extern PVRSRV_ERROR
PhysmemNewOSRamBackedPMR(PVRSRV_DEVICE_NODE * psDevNode,
			 IMG_DEVMEM_SIZE_T uiSize,
			 IMG_UINT32 uiLog2PageSize,
			 PVRSRV_MEMALLOCFLAGS_T uiFlags, PMR ** ppsPMROut);

#endif				/* #ifndef _SRVSRV_PHYSMEM_OSMEM_H_ */
