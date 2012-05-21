									    /*************************************************************************//*!
									       @File
									       @Title          YUV defines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(_IMGYUV_H_)
#define _IMGYUV_H_

typedef enum {
	IMG_COLORSPACE_BT601_CONFORMANT_RANGE = 1,
	IMG_COLORSPACE_BT601_FULL_RANGE = 2,
	IMG_COLORSPACE_BT709_CONFORMANT_RANGE = 3,
	IMG_COLORSPACE_BT709_FULL_RANGE = 4,
} IMG_YUV_COLORSPACE;

typedef enum {
	IMG_CHROMA_INTERP_ZERO = 1,
	IMG_CHROMA_INTERP_QUARTER = 2,
	IMG_CHROMA_INTERP_HALF = 3,
	IMG_CHROMA_INTERP_THREEQUARTERS = 4,
} IMG_YUV_CHROMA_INTERP;

#endif				/* _IMGYUV_H_ */
