									    /*************************************************************************//*!
									       @File
									       @Title          memory allocation header
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Memory-Allocation API definitions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __ALOCMEM_H__
#define __ALLOCMEM_H__

#include "img_types.h"

#if defined (__cplusplus)
extern "C" {
#endif

	IMG_PVOID OSAllocMem(IMG_UINT32 ui32Size);

	IMG_VOID OSFreeMem(IMG_PVOID pvCpuVAddr);

#if defined (__cplusplus)
}
#endif
#endif				/* __ALLOCMEM_H__ */
/******************************************************************************
 End of file (allocmem.h)
******************************************************************************/
