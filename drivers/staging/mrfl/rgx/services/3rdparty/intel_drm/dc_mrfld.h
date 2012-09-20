/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#ifndef __MRSTLFB_H__
#define __MRSTLFB_H__

#include <drm/drmP.h>
#include "kerneldisplay.h"

#define MAX_SWAPCHAINS			  10

#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_NONE (0 << 0)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A    (1 << 0)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B    (1 << 1)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C    (1 << 2)

typedef int (*MRSTLFB_VSYNC_ISR_PFN) (struct drm_device * psDrmDevice,
				      int iPipe);

typedef struct MRFLD_BUFFER_TAG {
	IMG_HANDLE hDisplayContext;
	IMG_PIXFMT ePixFormat;
	IMG_UINT32 ui32BufferSize;
	IMG_UINT32 ui32ByteStride;
	IMG_UINT32 ui32Width;
	IMG_UINT32 ui32Height;
	IMG_SYS_PHYADDR *psSysAddr;
	IMG_DEV_VIRTADDR sDevVAddr;
	IMG_CPU_VIRTADDR sCPUVAddr;
	IMG_BOOL bIsContiguous;
	IMG_BOOL bIsAllocated;
	IMG_UINT32 ui32OwnerTaskID;
	IMG_HANDLE hImport;
	IMG_UINT32 ui32RefCount;
} MRFLD_BUFFER;

typedef struct MRSTLFB_VSYNC_FLIP_ITEM_TAG {
	IMG_HANDLE hCmdComplete;
	IMG_UINT32 ulSwapInterval;
	IMG_BOOL bValid;
	IMG_BOOL bFlipped;
	IMG_BOOL bCmdCompleted;
	MRFLD_BUFFER *psBuffer;
} MRSTLFB_VSYNC_FLIP_ITEM;

typedef struct MRSTLFB_SWAPCHAIN_TAG {
	IMG_UINT32 ulBufferCount;
	IMG_UINT32 ui32SwapChainID;
	IMG_UINT32 ui32SwapChainPropertyFlag;
	IMG_UINT32 ulSwapChainGTTOffset;
	MRFLD_BUFFER **ppsBuffer;
	IMG_UINT32 ulSwapChainLength;
	MRSTLFB_VSYNC_FLIP_ITEM *psVSyncFlips;
	IMG_UINT32 ulInsertIndex;
	IMG_UINT32 ulRemoveIndex;

	struct drm_driver *psDrmDriver;
	struct drm_device *psDrmDev;
	struct MRFLD_DEVINFO_TAG *psDevInfo;

	struct MRSTLFB_SWAPCHAIN_TAG *psNext;
} MRSTLFB_SWAPCHAIN;

typedef struct MRFLD_DEVINFO_TAG {
	struct drm_device *psDrmDevice;
	IMG_HANDLE hSrvHandle;
	IMG_UINT32 ulRefCount;

	MRFLD_BUFFER sSystemBuffer;

	struct _DC_DEVICE_FUNCTIONS_ sDCJTable;

	MRSTLFB_SWAPCHAIN *psCurrentSwapChain;
	MRSTLFB_SWAPCHAIN *apsSwapChains[MAX_SWAPCHAINS];
	IMG_UINT32 ui32SwapChainNum;
	spinlock_t sSwapChainLock;
	unsigned long ulLastFlipAddr;
	IMG_BOOL bLastFlipAddrValid;

	IMG_UINT32 ulSetFlushStateRefCount;

	IMG_BOOL bFlushCommands;
	IMG_BOOL bBlanked;

	struct notifier_block sLINNotifBlock;

	IMG_DEV_VIRTADDR sDisplayDevVAddr;
	DC_DISPLAY_INFO sDisplayInfo;
	PVRSRV_SURFACE_INFO sPrimInfo;

	IMG_BOOL bSuspended;
} MRFLD_DEVINFO;

#define DISPLAY_DEVICE_NAME "Merrifield DRM"
#define	DRVNAME	"Merrifield-DRM"

#ifndef UNREFERENCED_PARAMETER
#define	UNREFERENCED_PARAMETER(param) (param) = (param)
#endif

PVRSRV_ERROR MerrifieldDCInit(struct drm_device *dev);
PVRSRV_ERROR MerrifieldDCDeinit(void);

#endif
