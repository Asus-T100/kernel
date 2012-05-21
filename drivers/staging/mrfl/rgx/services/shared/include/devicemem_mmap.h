/******************************************************************************
 * Name         : devicemem_mmap.c
 * Title        : Device Memory Management
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
 * Description  : OS abstraction for the mmap2 interface for mapping PMRs into
 *                User Mode memory
 *
 * Platform     : ALL
 *
 *****************************************************************************/

#ifndef _DEVICEMEM_MMAP_H_
#define _DEVICEMEM_MMAP_H_

#include "img_types.h"
#include "pvrsrv_error.h"

/*
 *
 * OSMMapPMR
 *
 * Causes this PMR to be mapped into CPU memory that the user process
 * may access.
 *
 * Whether the memory is mapped readonly, readwrite, or not at all, is
 * dependent on the PMR itself.
 *
 * The PMR handle is opaque to the user, and lower levels of this
 * stack ensure that the handle is private to this process, such that
 * this API cannot be abused to gain access to other people's PMRs.
 *
 * The OS implementation of this function should return the virtual
 * address and length for the User to use.  The "PrivData" is to be
 * stored opaquely by the caller (N.B. he should make no assumptions,
 * in particular, IMG_NULL is a valid handle) and given back to the
 * call to OSMunmapPMR.
 *
 * The OS implementation is free to use the PrivData handle for any
 * purpose it sees fit.
 */

extern PVRSRV_ERROR
OSMMapPMR(IMG_HANDLE hBridge,
	  IMG_HANDLE hPMR,
	  IMG_DEVMEM_SIZE_T uiPMRLength,
	  IMG_HANDLE * phOSMMapPrivDataOut,
	  IMG_VOID ** ppvMappingAddressOut, IMG_SIZE_T * puiMappingLengthOut);

/*
 *
 * OSMUnmapPMR
 *
 * The reverse of OSMMapPMR
 *
 * The caller is required to pass the PMR handle back in along with
 * the same 3-tuple of information as was returned by the call to
 * OSMMapPMR
 *
 */
/* 
   FIXME:
   perhaps this function should take _only_ the hOSMMapPrivData arg,
   and the implementation is required to store any of the other data
   items that it requires to do the unmap?
*/
extern IMG_VOID
OSMUnmapPMR(IMG_HANDLE hBridge,
	    IMG_HANDLE hPMR,
	    IMG_HANDLE hOSMMapPrivData,
	    IMG_VOID * pvMappingAddress, IMG_SIZE_T uiMappingLength);

#endif
