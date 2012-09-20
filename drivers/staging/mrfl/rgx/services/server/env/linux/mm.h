									    /*************************************************************************//*!
									       @File
									       @Title          Declares various memory management utility functions for Linux.
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/
#ifndef __IMG_LINUX_MM_H__
#define __IMG_LINUX_MM_H__

#include <asm/io.h>
/*!
 *******************************************************************************
 * @brief Reserve physical IO memory and create a CPU virtual mapping for it
 *
 * @param BasePAddr 
 * @param ui32Bytes  
 * @param ui32MappingFlags  
 *
 * @return 
 ******************************************************************************/
#if defined(DEBUG_LINUX_MEMORY_ALLOCATIONS)
#define IORemapWrapper(BasePAddr, ui32Bytes, ui32MappingFlags) \
    _IORemapWrapper(BasePAddr, ui32Bytes, ui32MappingFlags, __FILE__, __LINE__)
#else
#define IORemapWrapper(BasePAddr, ui32Bytes, ui32MappingFlags) \
    _IORemapWrapper(BasePAddr, ui32Bytes, ui32MappingFlags, NULL, 0)
#endif
IMG_VOID *_IORemapWrapper(IMG_CPU_PHYADDR BasePAddr,
			  IMG_UINT32 ui32Bytes,
			  IMG_UINT32 ui32MappingFlags,
			  IMG_CHAR * pszFileName, IMG_UINT32 ui32Line);

/*!
 ******************************************************************************
 * @brief Unmaps an IO memory mapping created using IORemap
 *
 * @param pvIORemapCookie  
 *
 * @return 
 ******************************************************************************/
#if defined(DEBUG_LINUX_MEMORY_ALLOCATIONS)
#define IOUnmapWrapper(pvIORemapCookie) \
    _IOUnmapWrapper(pvIORemapCookie, __FILE__, __LINE__)
#else
#define IOUnmapWrapper(pvIORemapCookie) \
    _IOUnmapWrapper(pvIORemapCookie, NULL, 0)
#endif
IMG_VOID _IOUnmapWrapper(IMG_VOID * pvIORemapCookie, IMG_CHAR * pszFileName,
			 IMG_UINT32 ui32Line);

#endif				/* __IMG_LINUX_MM_H__ */
