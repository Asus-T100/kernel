									    /*************************************************************************//*!
									       @File
									       @Title          Services cache management header
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Defines for cache management which are visible internally
									       and externally
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _CACHE_EXTERNAL_H_
#define _CACHE_EXTERNAL_H_
#include "img_types.h"

typedef IMG_UINT32 PVRSRV_CACHE_OP;

#define PVRSRV_CACHE_OP_NONE		0x0	/*!< No operation */
#define PVRSRV_CACHE_OP_CLEAN		0x1	/*!< Flush w/o invalidate */
#define PVRSRV_CACHE_OP_INVALIDATE	0x2	/*!< Invalidate w/o flush */
#define PVRSRV_CACHE_OP_FLUSH		0x3	/*!< Flush w/ invalidate */

/*
	If we get multiple cache operations before the operation which will
	trigger the oepration to happen then we need to make sure we do
	the right thing.
	
	Note: PVRSRV_CACHE_OP_INVALIDATE should never be passed into here
*/
#ifdef INLINE_IS_PRAGMA
#pragma inline(SetCacheOp)
#endif
static INLINE PVRSRV_CACHE_OP SetCacheOp(PVRSRV_CACHE_OP uiCurrent,
					 PVRSRV_CACHE_OP uiNew)
{
	PVRSRV_CACHE_OP uiRet;

	uiRet = uiCurrent | uiNew;
	return uiRet;
}

#endif				/* _CACHE_EXTERNAL_H_ */
