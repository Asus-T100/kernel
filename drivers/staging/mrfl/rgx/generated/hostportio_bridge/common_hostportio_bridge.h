									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for hostportio
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for hostportio
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_HOSTPORTIO_BRIDGE_H
#define COMMON_HOSTPORTIO_BRIDGE_H

#include "rgx_bridge.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_HOSTPORTIO_CMD_FIRST			(PVRSRV_BRIDGE_HOSTPORTIO_START)
#define PVRSRV_BRIDGE_HOSTPORTIO_HOSTPORTREAD			PVRSRV_IOWR(PVRSRV_BRIDGE_HOSTPORTIO_CMD_FIRST+0)
#define PVRSRV_BRIDGE_HOSTPORTIO_HOSTPORTWRITE			PVRSRV_IOWR(PVRSRV_BRIDGE_HOSTPORTIO_CMD_FIRST+1)
#define PVRSRV_BRIDGE_HOSTPORTIO_CMD_LAST			(PVRSRV_BRIDGE_HOSTPORTIO_CMD_FIRST+1)

/*******************************************
            HostPortRead          
 *******************************************/

/* Bridge in structure for HostPortRead */
typedef struct PVRSRV_BRIDGE_IN_HOSTPORTREAD_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hMemCtxPrivData;
	IMG_UINT32 ui32CRHostIFVal;
	IMG_UINT32 ui32ReadOffset;
	IMG_DEVMEM_SIZE_T uiDstBufLen;
} PVRSRV_BRIDGE_IN_HOSTPORTREAD;

/* Bridge out structure for HostPortRead */
typedef struct PVRSRV_BRIDGE_OUT_HOSTPORTREAD_TAG {
	IMG_CHAR *puiDstBuffer;
	IMG_DEVMEM_SIZE_T uiNumBytesRead;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_HOSTPORTREAD;

/*******************************************
            HostPortWrite          
 *******************************************/

/* Bridge in structure for HostPortWrite */
typedef struct PVRSRV_BRIDGE_IN_HOSTPORTWRITE_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hMemCtxPrivData;
	IMG_UINT32 ui32CRHostIFVal;
	IMG_UINT32 ui32WriteOffset;
	IMG_DEVMEM_SIZE_T uiSrcBufLen;
	IMG_CHAR *puiSrcBuffer;
} PVRSRV_BRIDGE_IN_HOSTPORTWRITE;

/* Bridge out structure for HostPortWrite */
typedef struct PVRSRV_BRIDGE_OUT_HOSTPORTWRITE_TAG {
	IMG_DEVMEM_SIZE_T uiNumBytesWritten;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_HOSTPORTWRITE;

#endif				/* COMMON_HOSTPORTIO_BRIDGE_H */
