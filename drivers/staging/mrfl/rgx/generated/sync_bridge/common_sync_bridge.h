									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for sync
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for sync
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_SYNC_BRIDGE_H
#define COMMON_SYNC_BRIDGE_H

#include "pdump.h"
#include "pdumpdefs.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_SYNC_CMD_FIRST			(PVRSRV_BRIDGE_SYNC_START)
#define PVRSRV_BRIDGE_SYNC_ALLOCSYNCPRIMITIVEBLOCK			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+0)
#define PVRSRV_BRIDGE_SYNC_FREESYNCPRIMITIVEBLOCK			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+1)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMSET			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+2)
#define PVRSRV_BRIDGE_SYNC_SERVERSYNCPRIMSET			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+3)
#define PVRSRV_BRIDGE_SYNC_SERVERSYNCALLOC			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+4)
#define PVRSRV_BRIDGE_SYNC_SERVERSYNCFREE			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+5)
#define PVRSRV_BRIDGE_SYNC_SERVERSYNCQUEUEHWOP			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+6)
#define PVRSRV_BRIDGE_SYNC_SERVERSYNCGETSTATUS			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+7)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMOPCREATE			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+8)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMOPTAKE			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+9)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMOPREADY			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+10)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMOPCOMPLETE			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+11)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMOPDESTROY			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+12)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMP			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+13)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMPPOL			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+14)
#define PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMPCBP			PVRSRV_IOWR(PVRSRV_BRIDGE_SYNC_CMD_FIRST+15)
#define PVRSRV_BRIDGE_SYNC_CMD_LAST			(PVRSRV_BRIDGE_SYNC_CMD_FIRST+15)

/*******************************************
            AllocSyncPrimitiveBlock          
 *******************************************/

/* Bridge in structure for AllocSyncPrimitiveBlock */
typedef struct PVRSRV_BRIDGE_IN_ALLOCSYNCPRIMITIVEBLOCK_TAG {
	IMG_HANDLE hDevNode;
} PVRSRV_BRIDGE_IN_ALLOCSYNCPRIMITIVEBLOCK;

/* Bridge out structure for AllocSyncPrimitiveBlock */
typedef struct PVRSRV_BRIDGE_OUT_ALLOCSYNCPRIMITIVEBLOCK_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32SyncPrimVAddr;
	IMG_UINT32 ui32SyncPrimBlockSize;
	DEVMEM_SERVER_EXPORTCOOKIE hExportCookie;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_ALLOCSYNCPRIMITIVEBLOCK;

/*******************************************
            FreeSyncPrimitiveBlock          
 *******************************************/

/* Bridge in structure for FreeSyncPrimitiveBlock */
typedef struct PVRSRV_BRIDGE_IN_FREESYNCPRIMITIVEBLOCK_TAG {
	IMG_HANDLE hSyncHandle;
} PVRSRV_BRIDGE_IN_FREESYNCPRIMITIVEBLOCK;

/* Bridge out structure for FreeSyncPrimitiveBlock */
typedef struct PVRSRV_BRIDGE_OUT_FREESYNCPRIMITIVEBLOCK_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_FREESYNCPRIMITIVEBLOCK;

/*******************************************
            SyncPrimSet          
 *******************************************/

/* Bridge in structure for SyncPrimSet */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMSET_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32Index;
	IMG_UINT32 ui32Value;
} PVRSRV_BRIDGE_IN_SYNCPRIMSET;

/* Bridge out structure for SyncPrimSet */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMSET_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMSET;

/*******************************************
            ServerSyncPrimSet          
 *******************************************/

/* Bridge in structure for ServerSyncPrimSet */
typedef struct PVRSRV_BRIDGE_IN_SERVERSYNCPRIMSET_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32Value;
} PVRSRV_BRIDGE_IN_SERVERSYNCPRIMSET;

/* Bridge out structure for ServerSyncPrimSet */
typedef struct PVRSRV_BRIDGE_OUT_SERVERSYNCPRIMSET_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SERVERSYNCPRIMSET;

/*******************************************
            ServerSyncAlloc          
 *******************************************/

/* Bridge in structure for ServerSyncAlloc */
typedef struct PVRSRV_BRIDGE_IN_SERVERSYNCALLOC_TAG {
	IMG_HANDLE hDevNode;
} PVRSRV_BRIDGE_IN_SERVERSYNCALLOC;

/* Bridge out structure for ServerSyncAlloc */
typedef struct PVRSRV_BRIDGE_OUT_SERVERSYNCALLOC_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32SyncPrimVAddr;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SERVERSYNCALLOC;

/*******************************************
            ServerSyncFree          
 *******************************************/

/* Bridge in structure for ServerSyncFree */
typedef struct PVRSRV_BRIDGE_IN_SERVERSYNCFREE_TAG {
	IMG_HANDLE hSyncHandle;
} PVRSRV_BRIDGE_IN_SERVERSYNCFREE;

/* Bridge out structure for ServerSyncFree */
typedef struct PVRSRV_BRIDGE_OUT_SERVERSYNCFREE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SERVERSYNCFREE;

/*******************************************
            ServerSyncQueueHWOp          
 *******************************************/

/* Bridge in structure for ServerSyncQueueHWOp */
typedef struct PVRSRV_BRIDGE_IN_SERVERSYNCQUEUEHWOP_TAG {
	IMG_HANDLE hSyncHandle;
} PVRSRV_BRIDGE_IN_SERVERSYNCQUEUEHWOP;

/* Bridge out structure for ServerSyncQueueHWOp */
typedef struct PVRSRV_BRIDGE_OUT_SERVERSYNCQUEUEHWOP_TAG {
	IMG_UINT32 ui32FenceValue;
	IMG_UINT32 ui32UpdateValue;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SERVERSYNCQUEUEHWOP;

/*******************************************
            ServerSyncGetStatus          
 *******************************************/

/* Bridge in structure for ServerSyncGetStatus */
typedef struct PVRSRV_BRIDGE_IN_SERVERSYNCGETSTATUS_TAG {
	IMG_UINT32 ui32SyncCount;
	IMG_HANDLE *phSyncHandle;
} PVRSRV_BRIDGE_IN_SERVERSYNCGETSTATUS;

/* Bridge out structure for ServerSyncGetStatus */
typedef struct PVRSRV_BRIDGE_OUT_SERVERSYNCGETSTATUS_TAG {
	IMG_UINT32 *pui32UID;
	IMG_UINT32 *pui32FWAddr;
	IMG_UINT32 *pui32CurrentOp;
	IMG_UINT32 *pui32NextOp;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SERVERSYNCGETSTATUS;

/*******************************************
            SyncPrimOpCreate          
 *******************************************/

/* Bridge in structure for SyncPrimOpCreate */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMOPCREATE_TAG {
	IMG_UINT32 ui32SyncBlockCount;
	IMG_HANDLE *phBlockList;
	IMG_UINT32 ui32ClientSyncCount;
	IMG_UINT32 *pui32SyncBlockIndex;
	IMG_UINT32 *pui32Index;
	IMG_UINT32 ui32ServerSyncCount;
	IMG_HANDLE *phServerSync;
} PVRSRV_BRIDGE_IN_SYNCPRIMOPCREATE;

/* Bridge out structure for SyncPrimOpCreate */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMOPCREATE_TAG {
	IMG_HANDLE hServerCookie;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMOPCREATE;

/*******************************************
            SyncPrimOpTake          
 *******************************************/

/* Bridge in structure for SyncPrimOpTake */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMOPTAKE_TAG {
	IMG_HANDLE hServerCookie;
	IMG_UINT32 ui32ClientSyncCount;
	IMG_UINT32 *pui32Flags;
	IMG_UINT32 *pui32FenceValue;
	IMG_UINT32 *pui32UpdateValue;
	IMG_UINT32 ui32ServerSyncCount;
} PVRSRV_BRIDGE_IN_SYNCPRIMOPTAKE;

/* Bridge out structure for SyncPrimOpTake */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMOPTAKE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMOPTAKE;

/*******************************************
            SyncPrimOpReady          
 *******************************************/

/* Bridge in structure for SyncPrimOpReady */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMOPREADY_TAG {
	IMG_HANDLE hServerCookie;
} PVRSRV_BRIDGE_IN_SYNCPRIMOPREADY;

/* Bridge out structure for SyncPrimOpReady */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMOPREADY_TAG {
	IMG_BOOL bReady;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMOPREADY;

/*******************************************
            SyncPrimOpComplete          
 *******************************************/

/* Bridge in structure for SyncPrimOpComplete */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMOPCOMPLETE_TAG {
	IMG_HANDLE hServerCookie;
} PVRSRV_BRIDGE_IN_SYNCPRIMOPCOMPLETE;

/* Bridge out structure for SyncPrimOpComplete */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMOPCOMPLETE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMOPCOMPLETE;

/*******************************************
            SyncPrimOpDestroy          
 *******************************************/

/* Bridge in structure for SyncPrimOpDestroy */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMOPDESTROY_TAG {
	IMG_HANDLE hServerCookie;
} PVRSRV_BRIDGE_IN_SYNCPRIMOPDESTROY;

/* Bridge out structure for SyncPrimOpDestroy */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMOPDESTROY_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMOPDESTROY;

/*******************************************
            SyncPrimPDump          
 *******************************************/

/* Bridge in structure for SyncPrimPDump */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMPDUMP_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32Offset;
} PVRSRV_BRIDGE_IN_SYNCPRIMPDUMP;

/* Bridge out structure for SyncPrimPDump */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMP_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMP;

/*******************************************
            SyncPrimPDumpPol          
 *******************************************/

/* Bridge in structure for SyncPrimPDumpPol */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMPDUMPPOL_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32Offset;
	IMG_UINT32 ui32Value;
	IMG_UINT32 ui32Mask;
	PDUMP_POLL_OPERATOR eOperator;
	PDUMP_FLAGS_T uiPDumpFlags;
} PVRSRV_BRIDGE_IN_SYNCPRIMPDUMPPOL;

/* Bridge out structure for SyncPrimPDumpPol */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMPPOL_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMPPOL;

/*******************************************
            SyncPrimPDumpCBP          
 *******************************************/

/* Bridge in structure for SyncPrimPDumpCBP */
typedef struct PVRSRV_BRIDGE_IN_SYNCPRIMPDUMPCBP_TAG {
	IMG_HANDLE hSyncHandle;
	IMG_UINT32 ui32Offset;
	IMG_UINT32 ui32WriteOffset;
	IMG_UINT32 ui32PacketSize;
	IMG_UINT32 ui32BufferSize;
} PVRSRV_BRIDGE_IN_SYNCPRIMPDUMPCBP;

/* Bridge out structure for SyncPrimPDumpCBP */
typedef struct PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMPCBP_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMPCBP;

#endif				/* COMMON_SYNC_BRIDGE_H */
