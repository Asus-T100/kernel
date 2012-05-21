									    /*************************************************************************//*!
									       @File
									       @Title          RGX memory context management
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for RGX memory context management
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXMEM_H__)
#define __RGXMEM_H__

#include "pvrsrv_error.h"
#include "device.h"
#include "mmu_common.h"
#include "rgxdevice.h"

/* FIXME: SyncPrim should be stored on the memory context */
IMG_VOID RGXMMUSyncPrimAlloc(PVRSRV_DEVICE_NODE * psDeviceNode);
IMG_VOID RGXMMUSyncPrimFree(IMG_VOID);

IMG_VOID RGXMMUCacheInvalidate(PVRSRV_DEVICE_NODE * psDeviceNode,
			       IMG_HANDLE hDeviceData,
			       MMU_LEVEL eMMULevel, IMG_BOOL bUnmap);

PVRSRV_ERROR RGXSLCCacheInvalidateRequest(PVRSRV_DEVICE_NODE * psDeviceNode,
					  PMR * psPmr);

PVRSRV_ERROR RGXPreKickCacheCommand(PVRSRV_RGXDEV_INFO * psDevInfo);

IMG_VOID RGXUnregisterMemoryContext(IMG_HANDLE hPrivData);
PVRSRV_ERROR RGXRegisterMemoryContext(PVRSRV_DEVICE_NODE * psDeviceNode,
				      MMU_CONTEXT * psMMUContext,
				      IMG_HANDLE * hPrivData);

#endif				/* __RGXMEM_H__ */
