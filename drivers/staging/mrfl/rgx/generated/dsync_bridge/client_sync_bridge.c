									    /*************************************************************************//*!
									       @Title          Direct client bridge for sync
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "client_sync_bridge.h"
#include "img_defs.h"
#include "pvr_debug.h"

/* Module specific includes */
#include "pdump.h"
#include "pdumpdefs.h"

#include "sync_server.h"
#include "pdump.h"

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeAllocSyncPrimitiveBlock(IMG_HANDLE
								     hBridge,
								     IMG_HANDLE
								     hDevNode,
								     IMG_HANDLE
								     *
								     phSyncHandle,
								     IMG_UINT32
								     *
								     pui32SyncPrimVAddr,
								     IMG_UINT32
								     *
								     pui32SyncPrimBlockSize,
								     DEVMEM_SERVER_EXPORTCOOKIE
								     *
								     phExportCookie)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDevNodeInt;
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;
	DEVMEM_EXPORTCOOKIE *psExportCookieInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDevNodeInt = (IMG_HANDLE) hDevNode;

	eError =
	    PVRSRVAllocSyncPrimitiveBlockKM(hDevNodeInt,
					    &psSyncHandleInt,
					    pui32SyncPrimVAddr,
					    pui32SyncPrimBlockSize,
					    &psExportCookieInt);

	*phSyncHandle = psSyncHandleInt;
	*phExportCookie = psExportCookieInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeFreeSyncPrimitiveBlock(IMG_HANDLE
								    hBridge,
								    IMG_HANDLE
								    hSyncHandle)
{
	PVRSRV_ERROR eError;
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SYNC_PRIMITIVE_BLOCK *) hSyncHandle;

	eError = PVRSRVFreeSyncPrimitiveBlockKM(psSyncHandleInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimSet(IMG_HANDLE hBridge,
							 IMG_HANDLE hSyncHandle,
							 IMG_UINT32 ui32Index,
							 IMG_UINT32 ui32Value)
{
	PVRSRV_ERROR eError;
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SYNC_PRIMITIVE_BLOCK *) hSyncHandle;

	eError = PVRSRVSyncPrimSetKM(psSyncHandleInt, ui32Index, ui32Value);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeServerSyncPrimSet(IMG_HANDLE
							       hBridge,
							       IMG_HANDLE
							       hSyncHandle,
							       IMG_UINT32
							       ui32Value)
{
	PVRSRV_ERROR eError;
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SERVER_SYNC_PRIMITIVE *) hSyncHandle;

	eError = PVRSRVServerSyncPrimSetKM(psSyncHandleInt, ui32Value);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeServerSyncAlloc(IMG_HANDLE hBridge,
							     IMG_HANDLE
							     hDevNode,
							     IMG_HANDLE *
							     phSyncHandle,
							     IMG_UINT32 *
							     pui32SyncPrimVAddr)
{
	PVRSRV_ERROR eError;
	IMG_HANDLE hDevNodeInt;
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	hDevNodeInt = (IMG_HANDLE) hDevNode;

	eError =
	    PVRSRVServerSyncAllocKM(hDevNodeInt,
				    &psSyncHandleInt, pui32SyncPrimVAddr);

	*phSyncHandle = psSyncHandleInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeServerSyncFree(IMG_HANDLE hBridge,
							    IMG_HANDLE
							    hSyncHandle)
{
	PVRSRV_ERROR eError;
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SERVER_SYNC_PRIMITIVE *) hSyncHandle;

	eError = PVRSRVServerSyncFreeKM(psSyncHandleInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeServerSyncQueueHWOp(IMG_HANDLE
								 hBridge,
								 IMG_HANDLE
								 hSyncHandle,
								 IMG_UINT32 *
								 pui32FenceValue,
								 IMG_UINT32 *
								 pui32UpdateValue)
{
	PVRSRV_ERROR eError;
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SERVER_SYNC_PRIMITIVE *) hSyncHandle;

	eError =
	    PVRSRVServerSyncQueueHWOpKM(psSyncHandleInt,
					pui32FenceValue, pui32UpdateValue);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeServerSyncGetStatus(IMG_HANDLE
								 hBridge,
								 IMG_UINT32
								 ui32SyncCount,
								 IMG_HANDLE *
								 phSyncHandle,
								 IMG_UINT32 *
								 pui32UID,
								 IMG_UINT32 *
								 pui32FWAddr,
								 IMG_UINT32 *
								 pui32CurrentOp,
								 IMG_UINT32 *
								 pui32NextOp)
{
	PVRSRV_ERROR eError;
	SERVER_SYNC_PRIMITIVE **psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SERVER_SYNC_PRIMITIVE **) phSyncHandle;

	eError =
	    PVRSRVServerSyncGetStatusKM(ui32SyncCount,
					psSyncHandleInt,
					pui32UID,
					pui32FWAddr,
					pui32CurrentOp, pui32NextOp);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimOpCreate(IMG_HANDLE
							      hBridge,
							      IMG_UINT32
							      ui32SyncBlockCount,
							      IMG_HANDLE *
							      phBlockList,
							      IMG_UINT32
							      ui32ClientSyncCount,
							      IMG_UINT32 *
							      pui32SyncBlockIndex,
							      IMG_UINT32 *
							      pui32Index,
							      IMG_UINT32
							      ui32ServerSyncCount,
							      IMG_HANDLE *
							      phServerSync,
							      IMG_HANDLE *
							      phServerCookie)
{
	PVRSRV_ERROR eError;
	SYNC_PRIMITIVE_BLOCK **psBlockListInt;
	SERVER_SYNC_PRIMITIVE **psServerSyncInt;
	SERVER_OP_COOKIE *psServerCookieInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psBlockListInt = (SYNC_PRIMITIVE_BLOCK **) phBlockList;
	psServerSyncInt = (SERVER_SYNC_PRIMITIVE **) phServerSync;

	eError =
	    PVRSRVSyncPrimOpCreateKM(ui32SyncBlockCount,
				     psBlockListInt,
				     ui32ClientSyncCount,
				     pui32SyncBlockIndex,
				     pui32Index,
				     ui32ServerSyncCount,
				     psServerSyncInt, &psServerCookieInt);

	*phServerCookie = psServerCookieInt;

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimOpTake(IMG_HANDLE hBridge,
							    IMG_HANDLE
							    hServerCookie,
							    IMG_UINT32
							    ui32ClientSyncCount,
							    IMG_UINT32 *
							    pui32Flags,
							    IMG_UINT32 *
							    pui32FenceValue,
							    IMG_UINT32 *
							    pui32UpdateValue,
							    IMG_UINT32
							    ui32ServerSyncCount)
{
	PVRSRV_ERROR eError;
	SERVER_OP_COOKIE *psServerCookieInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psServerCookieInt = (SERVER_OP_COOKIE *) hServerCookie;

	eError =
	    PVRSRVSyncPrimOpTakeKM(psServerCookieInt,
				   ui32ClientSyncCount,
				   pui32Flags,
				   pui32FenceValue,
				   pui32UpdateValue, ui32ServerSyncCount);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimOpReady(IMG_HANDLE hBridge,
							     IMG_HANDLE
							     hServerCookie,
							     IMG_BOOL * pbReady)
{
	PVRSRV_ERROR eError;
	SERVER_OP_COOKIE *psServerCookieInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psServerCookieInt = (SERVER_OP_COOKIE *) hServerCookie;

	eError = PVRSRVSyncPrimOpReadyKM(psServerCookieInt, pbReady);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimOpComplete(IMG_HANDLE
								hBridge,
								IMG_HANDLE
								hServerCookie)
{
	PVRSRV_ERROR eError;
	SERVER_OP_COOKIE *psServerCookieInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psServerCookieInt = (SERVER_OP_COOKIE *) hServerCookie;

	eError = PVRSRVSyncPrimOpCompleteKM(psServerCookieInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimOpDestroy(IMG_HANDLE
							       hBridge,
							       IMG_HANDLE
							       hServerCookie)
{
	PVRSRV_ERROR eError;
	SERVER_OP_COOKIE *psServerCookieInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psServerCookieInt = (SERVER_OP_COOKIE *) hServerCookie;

	eError = PVRSRVSyncPrimOpDestroyKM(psServerCookieInt);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimPDump(IMG_HANDLE hBridge,
							   IMG_HANDLE
							   hSyncHandle,
							   IMG_UINT32
							   ui32Offset)
{
	PVRSRV_ERROR eError;
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SYNC_PRIMITIVE_BLOCK *) hSyncHandle;

	eError = PVRSRVSyncPrimPDumpKM(psSyncHandleInt, ui32Offset);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimPDumpPol(IMG_HANDLE
							      hBridge,
							      IMG_HANDLE
							      hSyncHandle,
							      IMG_UINT32
							      ui32Offset,
							      IMG_UINT32
							      ui32Value,
							      IMG_UINT32
							      ui32Mask,
							      PDUMP_POLL_OPERATOR
							      eOperator,
							      PDUMP_FLAGS_T
							      uiPDumpFlags)
{
	PVRSRV_ERROR eError;
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SYNC_PRIMITIVE_BLOCK *) hSyncHandle;

	eError =
	    PVRSRVSyncPrimPDumpPolKM(psSyncHandleInt,
				     ui32Offset,
				     ui32Value,
				     ui32Mask, eOperator, uiPDumpFlags);

	return eError;
}

IMG_INTERNAL PVRSRV_ERROR IMG_CALLCONV BridgeSyncPrimPDumpCBP(IMG_HANDLE
							      hBridge,
							      IMG_HANDLE
							      hSyncHandle,
							      IMG_UINT32
							      ui32Offset,
							      IMG_UINT32
							      ui32WriteOffset,
							      IMG_UINT32
							      ui32PacketSize,
							      IMG_UINT32
							      ui32BufferSize)
{
	PVRSRV_ERROR eError;
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;

	PVR_UNREFERENCED_PARAMETER(hBridge);

	psSyncHandleInt = (SYNC_PRIMITIVE_BLOCK *) hSyncHandle;

	eError =
	    PVRSRVSyncPrimPDumpCBPKM(psSyncHandleInt,
				     ui32Offset,
				     ui32WriteOffset,
				     ui32PacketSize, ui32BufferSize);

	return eError;
}
