									    /*************************************************************************//*!
									       @File
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/
#ifndef _SERVICES_PDUMP_H_
#define _SERVICES_PDUMP_H_

#include "img_types.h"

typedef IMG_UINT32 PDUMP_FLAGS_T;

#define PDUMP_FLAGS_NEVER			0x08000000UL
#define PDUMP_FLAGS_LASTFRAME		0x10000000UL
#define PDUMP_FLAGS_RESETLFBUFFER	0x20000000UL
#define PDUMP_FLAGS_CONTINUOUS		0x40000000UL
#define PDUMP_FLAGS_PERSISTENT		0x80000000UL

#define PDUMP_FILEOFFSET_FMTSPEC "0x%08X"
typedef IMG_UINT32 PDUMP_FILEOFFSET_T;

#endif				/* _SERVICES_PDUMP_H_ */
