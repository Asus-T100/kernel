									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for rgxccb
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for rgxccb
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_RGXCCB_BRIDGE_H
#define COMMON_RGXCCB_BRIDGE_H

#include "rgx_bridge.h"
#include "devicemem_typedefs.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_RGXCCB_CMD_FIRST			(PVRSRV_BRIDGE_RGXCCB_START)
#define PVRSRV_BRIDGE_RGXCCB_RGXCREATECCB			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXCCB_CMD_FIRST+0)
#define PVRSRV_BRIDGE_RGXCCB_RGXDESTROYCCB			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXCCB_CMD_FIRST+1)
#define PVRSRV_BRIDGE_RGXCCB_CMD_LAST			(PVRSRV_BRIDGE_RGXCCB_CMD_FIRST+1)

/*******************************************
            RGXCreateCCB          
 *******************************************/

/* Bridge in structure for RGXCreateCCB */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATECCB_TAG {
	IMG_HANDLE hDevNode;
	IMG_UINT32 ui32AllocSize;
	IMG_UINT32 ui32AllocAlignment;
} PVRSRV_BRIDGE_IN_RGXCREATECCB;

/* Bridge out structure for RGXCreateCCB */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATECCB_TAG {
	IMG_HANDLE hCleanupCookie;
	IMG_HANDLE hClientCCBMemDesc;
	IMG_HANDLE hClientCCBCtlMemDesc;
	DEVMEM_SERVER_EXPORTCOOKIE hClientCCBExportCookie;
	DEVMEM_SERVER_EXPORTCOOKIE hClientCCBCtlExportCookie;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATECCB;

/*******************************************
            RGXDestroyCCB          
 *******************************************/

/* Bridge in structure for RGXDestroyCCB */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYCCB_TAG {
	IMG_HANDLE hCleanupCookie;
} PVRSRV_BRIDGE_IN_RGXDESTROYCCB;

/* Bridge out structure for RGXDestroyCCB */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYCCB_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYCCB;

#endif				/* COMMON_RGXCCB_BRIDGE_H */
