									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for syncexport
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for syncexport
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_SYNCEXPORT_BRIDGE_H
#define COMMON_SYNCEXPORT_BRIDGE_H

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_SYNCEXPORT_CMD_FIRST			(PVRSRV_BRIDGE_SYNCEXPORT_START)
#define PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVEREXPORT			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNCEXPORT_CMD_FIRST+0)
#define PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVERUNEXPORT			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNCEXPORT_CMD_FIRST+1)
#define PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVERIMPORT			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNCEXPORT_CMD_FIRST+2)
#define PVRSRV_BRIDGE_SYNCEXPORT_CMD_LAST			(PVRSRV_BRIDGE_SYNCEXPORT_CMD_FIRST+2)

/*******************************************
            SyncPrimServerExport          
 *******************************************/

/* Bridge in structure for SyncPrimServerExport */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMSERVEREXPORT_TAG {
	IMG_HANDLE hSyncHandle;
} PVRSRV_BRIDGE_IN_SYNCPRIMSERVEREXPORT;

/* Bridge out structure for SyncPrimServerExport */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMSERVEREXPORT_TAG {
	IMG_HANDLE hExport;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMSERVEREXPORT;

/*******************************************
            SyncPrimServerUnexport          
 *******************************************/

/* Bridge in structure for SyncPrimServerUnexport */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMSERVERUNEXPORT_TAG {
	IMG_HANDLE hExport;
} PVRSRV_BRIDGE_IN_SYNCPRIMSERVERUNEXPORT;

/* Bridge out structure for SyncPrimServerUnexport */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMSERVERUNEXPORT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMSERVERUNEXPORT;

/*******************************************
            SyncPrimServerImport          
 *******************************************/

/* Bridge in structure for SyncPrimServerImport */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMSERVERIMPORT_TAG {
	IMG_HANDLE hImport;
} PVRSRV_BRIDGE_IN_SYNCPRIMSERVERIMPORT;

/* Bridge out structure for SyncPrimServerImport */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMSERVERIMPORT_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32SyncPrimVAddr;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMSERVERIMPORT;

#endif				/* COMMON_SYNCEXPORT_BRIDGE_H */
