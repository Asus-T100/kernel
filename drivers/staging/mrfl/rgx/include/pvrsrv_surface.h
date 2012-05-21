									    /*************************************************************************//*!
									       @File
									       @Title          Device class external
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Defines DC specific structures which are externally visible
									       (i.e. visible to clients of services), but are also required
									       within services.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _PVRSRV_SURFACE_H_
#define _PVRSRV_SURFACE_H_

#include "img_types.h"

typedef struct _PVRSRV_SURFACE_FORMAT_ {
	IMG_UINT32 ePixFormat;
} PVRSRV_SURFACE_FORMAT;

typedef struct _PVRSRV_SURFACE_DIMS_ {
	IMG_UINT32 ui32Width;
	IMG_UINT32 ui32Height;
} PVRSRV_SURFACE_DIMS;

typedef struct _PVRSRV_SURFACE_INFO_ {
	PVRSRV_SURFACE_DIMS sDims;
	PVRSRV_SURFACE_FORMAT sFormat;
} PVRSRV_SURFACE_INFO;

typedef struct _PVRSRV_SURFACE_CONFIG_INFO_ {
	PVRSRV_SURFACE_INFO sSurface;
	IMG_UINT32 ui32XOffset;
	IMG_UINT32 ui32YOffset;
} PVRSRV_SURFACE_CONFIG_INFO;

#endif				/* _PVRSRV_SURFACE_H_ */
