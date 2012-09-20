									    /*************************************************************************//*!
									       @File
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _CACHE_DEFINES_H_

#define CACHEFLUSH_GENERIC	1
#define CACHEFLUSH_X86		2

#if CACHEFLUSH_TYPE == 0
#error Unknown cache flush type, please addd to cache_defines.h
#endif

#endif				/* _CACHE_DEFINES_H_ */
