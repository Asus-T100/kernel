/**************************************************************************
 *
 * Name         : mm.c
 *
 * Copyright    : 2007 by Imagination Technologies Limited. 
 *                All rights reserved.
 *                No part of this software, either material or conceptual 
 *                may be copied or distributed, transmitted, transcribed,
 *                stored in a retrieval system or translated into any 
 *                human or computer language in any form by any means,
 *                electronic, mechanical, manual or other-wise, or 
 *                disclosed to third parties without the express written
 *                permission of:
 *                Imagination Technologies Limited, 
 *                HomePark Industrial Estate, 
 *                Kings Langley, 
 *                Hertfordshire,
 *                WD4 8LZ, 
 *                UK
 *
 * Description  : Misc memory management utility functions for Linux
 *
 **************************************************************************/

#include <asm/io.h>

#include "img_defs.h"
#include "mutils.h"
#include "pvr_debug.h"
#include "mm.h"

IMG_VOID *_IORemapWrapper(IMG_CPU_PHYADDR BasePAddr,
			  IMG_UINT32 ui32Bytes,
			  IMG_UINT32 ui32MappingFlags,
			  IMG_CHAR * pszFileName, IMG_UINT32 ui32Line)
{
	IMG_VOID *pvIORemapCookie;

#if defined(FIXME)
	/* FIXME: We should not be using PVRSRV_HAP_*, heap flags should have no meaning here */
	switch (ui32MappingFlags & PVRSRV_HAP_CACHETYPE_MASK) {
	case PVRSRV_HAP_CACHED:
		pvIORemapCookie =
		    (IMG_VOID *) IOREMAP(BasePAddr.uiAddr, ui32Bytes);
		break;
	case PVRSRV_HAP_WRITECOMBINE:
		pvIORemapCookie =
		    (IMG_VOID *) IOREMAP_WC(BasePAddr.uiAddr, ui32Bytes);
		break;
	case PVRSRV_HAP_UNCACHED:
		pvIORemapCookie =
		    (IMG_VOID *) IOREMAP_UC(BasePAddr.uiAddr, ui32Bytes);
		break;
	default:
		PVR_DPF((PVR_DBG_ERROR,
			 "IORemapWrapper: unknown mapping flags"));
		return NULL;
	}
#else
	PVR_UNREFERENCED_PARAMETER(ui32MappingFlags);

	pvIORemapCookie = (IMG_VOID *) IOREMAP_UC(BasePAddr.uiAddr, ui32Bytes);
#endif

	PVR_UNREFERENCED_PARAMETER(pszFileName);
	PVR_UNREFERENCED_PARAMETER(ui32Line);

	return pvIORemapCookie;
}

IMG_VOID
_IOUnmapWrapper(IMG_VOID * pvIORemapCookie, IMG_CHAR * pszFileName,
		IMG_UINT32 ui32Line)
{
	PVR_UNREFERENCED_PARAMETER(pszFileName);
	PVR_UNREFERENCED_PARAMETER(ui32Line);
	iounmap(pvIORemapCookie);
}
