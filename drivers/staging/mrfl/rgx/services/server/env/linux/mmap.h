									    /*************************************************************************//*!
									       @File
									       @Title          Linux mmap interface declaration
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__MMAP_H__)
#define __MMAP_H__

#include <linux/mm.h>

/*!
 *******************************************************************************
 * @brief Mmap initialisation code
 *
 * @param void  
 *
 * @return 
 ******************************************************************************/
IMG_VOID PVRMMapInit(IMG_VOID);

/*!
 *******************************************************************************
 * @brief Mmap de-initialisation code
 *
 * @param void  
 *
 * @return 
 ******************************************************************************/
IMG_VOID PVRMMapCleanup(IMG_VOID);

/*!
 *******************************************************************************
 * @brief driver mmap entry point
 * 
 * @param pFile : user file structure
 *
 * @param ps_vma : vm area structure
 * 
 * @return 0 for success, -errno for failure.
 ******************************************************************************/
int MMapPMR(struct file *pFile, struct vm_area_struct *ps_vma);

#endif				/* __MMAP_H__ */
