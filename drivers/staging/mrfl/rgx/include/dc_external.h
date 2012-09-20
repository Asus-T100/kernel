									    /*************************************************************************//*!
									       @File
									       @Title          Device class external
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Defines DC specific structures which are externally visible
									       (i.e. visible to clients of services), but are also required
									       within services.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _DC_EXTERNAL_H_
#define _DC_EXTERNAL_H_

#include "img_types.h"

#define DC_NAME_SIZE	50
typedef struct _DC_DISPLAY_INFO_ {
	IMG_CHAR szDisplayName[DC_NAME_SIZE];
	IMG_UINT32 ui32MinDisplayPeriod;
	IMG_UINT32 ui32MaxDisplayPeriod;
} DC_DISPLAY_INFO;

typedef struct _DC_BUFFER_IMPORT_INFO_ {
	IMG_UINT32 ePixFormat;
	IMG_UINT32 ui32ColourChannels;
	IMG_UINT32 ui32Width[3];
	IMG_UINT32 ui32Height[3];
	IMG_UINT32 ui32Stride[3];
} DC_BUFFER_IMPORT_INFO;

#endif				/* _DC_EXTERNAL_H_ */
