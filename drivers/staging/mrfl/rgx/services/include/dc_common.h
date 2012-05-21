									    /*************************************************************************//*!
									       @File
									       @Title          Common Display Class header
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Defines DC specific structures which are shared within services
									       only
									       @License        Strictly Confidential.
    *//**************************************************************************/
#include "img_types.h"
#include "services.h"

#ifndef _DC_COMMON_H_
#define _DC_COMMON_H_
typedef struct _DC_BUFFER_CREATE_INFO_ {
	PVRSRV_SURFACE_INFO sSurface;	/*!< Surface properies, specificed by user */
	IMG_UINT32 ui32BPP;	/*!< Bits per pixel */
} DC_BUFFER_CREATE_INFO;

#endif				/* _DC_COMMON_H_ */
