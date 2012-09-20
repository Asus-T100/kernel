									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for pdumpcmm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for pdumpcmm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_PDUMPCMM_BRIDGE_H
#define COMMON_PDUMPCMM_BRIDGE_H

#include "devicemem_typedefs.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_PDUMPCMM_CMD_FIRST			(PVRSRV_BRIDGE_PDUMPCMM_START)
#define PVRSRV_BRIDGE_PDUMPCMM_DEVMEMPDUMPBITMAP			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMPCMM_CMD_FIRST+0)
#define PVRSRV_BRIDGE_PDUMPCMM_CMD_LAST			(PVRSRV_BRIDGE_PDUMPCMM_CMD_FIRST+0)

/*******************************************
            DevmemPDumpBitmap          
 *******************************************/

/* Bridge in structure for DevmemPDumpBitmap */
typedef struct PVRSRV_BRIDGE_IN_DEVMEMPDUMPBITMAP_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_CHAR *puiFileName;
	IMG_UINT32 ui32FileOffset;
	IMG_UINT32 ui32Width;
	IMG_UINT32 ui32Height;
	IMG_UINT32 ui32StrideInBytes;
	IMG_DEV_VIRTADDR sDevBaseAddr;
	IMG_HANDLE hDevmemCtx;
	IMG_UINT32 ui32Size;
	PDUMP_PIXEL_FORMAT ePixelFormat;
	PDUMP_MEM_FORMAT eMemFormat;
	IMG_UINT32 ui32PDumpFlags;
} PVRSRV_BRIDGE_IN_DEVMEMPDUMPBITMAP;

/* Bridge out structure for DevmemPDumpBitmap */
typedef struct PVRSRV_BRIDGE_OUT_DEVMEMPDUMPBITMAP_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEVMEMPDUMPBITMAP;

#endif				/* COMMON_PDUMPCMM_BRIDGE_H */
