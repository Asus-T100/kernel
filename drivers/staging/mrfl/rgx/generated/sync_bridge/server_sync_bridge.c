									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for sync
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for sync
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "sync_server.h"
#include "pdump.h"

#include "common_sync_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR FreeSyncPrimitiveBlockResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR ServerSyncFreeResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR SyncPrimOpDestroyResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static IMG_INT
PVRSRVBridgeAllocSyncPrimitiveBlock(IMG_UINT32 ui32BridgeID,
				    PVRSRV_BRIDGE_IN_ALLOCSYNCPRIMITIVEBLOCK *
				    psAllocSyncPrimitiveBlockIN,
				    PVRSRV_BRIDGE_OUT_ALLOCSYNCPRIMITIVEBLOCK *
				    psAllocSyncPrimitiveBlockOUT,
				    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;
	DEVMEM_EXPORTCOOKIE *psExportCookieInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_ALLOCSYNCPRIMITIVEBLOCK);

	NEW_HANDLE_BATCH_OR_ERROR(psAllocSyncPrimitiveBlockOUT->eError,
				  psConnection, 2)

	    /* Look up the address from the handle */
	    psAllocSyncPrimitiveBlockOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psAllocSyncPrimitiveBlockIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psAllocSyncPrimitiveBlockOUT->eError != PVRSRV_OK) {
		goto AllocSyncPrimitiveBlock_exit;
	}

	psAllocSyncPrimitiveBlockOUT->eError =
	    PVRSRVAllocSyncPrimitiveBlockKM(hDevNodeInt,
					    &psSyncHandleInt,
					    &psAllocSyncPrimitiveBlockOUT->
					    ui32SyncPrimVAddr,
					    &psAllocSyncPrimitiveBlockOUT->
					    ui32SyncPrimBlockSize,
					    &psExportCookieInt);
	/* Exit early if bridged call fails */
	if (psAllocSyncPrimitiveBlockOUT->eError != PVRSRV_OK) {
		goto AllocSyncPrimitiveBlock_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hSyncHandleInt2 = ResManRegisterRes(psConnection->hResManContext,
					    RESMAN_TYPE_SYNC_PRIMITIVE_BLOCK,
					    psSyncHandleInt, 0,
					    /* FIXME: how can we avoid this cast? */
					    (RESMAN_FREE_FN) &
					    PVRSRVFreeSyncPrimitiveBlockKM);
	if (hSyncHandleInt2 == IMG_NULL) {
		psAllocSyncPrimitiveBlockOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto AllocSyncPrimitiveBlock_exit;
	}
	psAllocSyncPrimitiveBlockOUT->eError =
	    PVRSRVAllocHandle(psConnection->psHandleBase,
			      &psAllocSyncPrimitiveBlockOUT->hSyncHandle,
			      (IMG_HANDLE) hSyncHandleInt2,
			      PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psAllocSyncPrimitiveBlockOUT->eError != PVRSRV_OK) {
		goto AllocSyncPrimitiveBlock_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psAllocSyncPrimitiveBlockOUT->hExportCookie,
			       (IMG_HANDLE) psExportCookieInt,
			       PVRSRV_HANDLE_TYPE_SERVER_EXPORTCOOKIE,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psAllocSyncPrimitiveBlockOUT->hSyncHandle);
	COMMIT_HANDLE_BATCH_OR_ERROR(psAllocSyncPrimitiveBlockOUT->eError,
				     psConnection);

 AllocSyncPrimitiveBlock_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeFreeSyncPrimitiveBlock(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_FREESYNCPRIMITIVEBLOCK *
				   psFreeSyncPrimitiveBlockIN,
				   PVRSRV_BRIDGE_OUT_FREESYNCPRIMITIVEBLOCK *
				   psFreeSyncPrimitiveBlockOUT,
				   CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_FREESYNCPRIMITIVEBLOCK);

	/* Look up the address from the handle */
	psFreeSyncPrimitiveBlockOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psFreeSyncPrimitiveBlockIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
	if (psFreeSyncPrimitiveBlockOUT->eError != PVRSRV_OK) {
		goto FreeSyncPrimitiveBlock_exit;
	}

	psFreeSyncPrimitiveBlockOUT->eError =
	    FreeSyncPrimitiveBlockResManProxy(hSyncHandleInt2);
	/* Exit early if bridged call fails */
	if (psFreeSyncPrimitiveBlockOUT->eError != PVRSRV_OK) {
		goto FreeSyncPrimitiveBlock_exit;
	}

	psFreeSyncPrimitiveBlockOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psFreeSyncPrimitiveBlockIN->
				hSyncHandle,
				PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);

 FreeSyncPrimitiveBlock_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimSet(IMG_UINT32 ui32BridgeID,
			PVRSRV_BRIDGE_IN_SYNCPRIMSET * psSyncPrimSetIN,
			PVRSRV_BRIDGE_OUT_SYNCPRIMSET * psSyncPrimSetOUT,
			CONNECTION_DATA * psConnection)
{
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SYNC_SYNCPRIMSET);

	/* Look up the address from the handle */
	psSyncPrimSetOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psSyncPrimSetIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
	if (psSyncPrimSetOUT->eError != PVRSRV_OK) {
		goto SyncPrimSet_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimSetOUT->eError =
	    ResManFindPrivateDataByPtr(hSyncHandleInt2,
				       (IMG_VOID **) & psSyncHandleInt,
				       IMG_NULL);
	if (psSyncPrimSetOUT->eError != PVRSRV_OK) {
		goto SyncPrimSet_exit;
	}

	psSyncPrimSetOUT->eError =
	    PVRSRVSyncPrimSetKM(psSyncHandleInt,
				psSyncPrimSetIN->ui32Index,
				psSyncPrimSetIN->ui32Value);

 SyncPrimSet_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeServerSyncPrimSet(IMG_UINT32 ui32BridgeID,
			      PVRSRV_BRIDGE_IN_SERVERSYNCPRIMSET *
			      psServerSyncPrimSetIN,
			      PVRSRV_BRIDGE_OUT_SERVERSYNCPRIMSET *
			      psServerSyncPrimSetOUT,
			      CONNECTION_DATA * psConnection)
{
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SERVERSYNCPRIMSET);

	/* Look up the address from the handle */
	psServerSyncPrimSetOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psServerSyncPrimSetIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
	if (psServerSyncPrimSetOUT->eError != PVRSRV_OK) {
		goto ServerSyncPrimSet_exit;
	}

	/* Look up the data from the resman address */
	psServerSyncPrimSetOUT->eError =
	    ResManFindPrivateDataByPtr(hSyncHandleInt2,
				       (IMG_VOID **) & psSyncHandleInt,
				       IMG_NULL);
	if (psServerSyncPrimSetOUT->eError != PVRSRV_OK) {
		goto ServerSyncPrimSet_exit;
	}

	psServerSyncPrimSetOUT->eError =
	    PVRSRVServerSyncPrimSetKM(psSyncHandleInt,
				      psServerSyncPrimSetIN->ui32Value);

 ServerSyncPrimSet_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeServerSyncAlloc(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_SERVERSYNCALLOC *
			    psServerSyncAllocIN,
			    PVRSRV_BRIDGE_OUT_SERVERSYNCALLOC *
			    psServerSyncAllocOUT,
			    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SERVERSYNCALLOC);

	NEW_HANDLE_BATCH_OR_ERROR(psServerSyncAllocOUT->eError, psConnection, 1)

	    /* Look up the address from the handle */
	    psServerSyncAllocOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psServerSyncAllocIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psServerSyncAllocOUT->eError != PVRSRV_OK) {
		goto ServerSyncAlloc_exit;
	}

	psServerSyncAllocOUT->eError =
	    PVRSRVServerSyncAllocKM(hDevNodeInt,
				    &psSyncHandleInt,
				    &psServerSyncAllocOUT->ui32SyncPrimVAddr);
	/* Exit early if bridged call fails */
	if (psServerSyncAllocOUT->eError != PVRSRV_OK) {
		goto ServerSyncAlloc_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hSyncHandleInt2 = ResManRegisterRes(psConnection->hResManContext,
					    RESMAN_TYPE_SERVER_SYNC_PRIMITIVE,
					    psSyncHandleInt, 0,
					    /* FIXME: how can we avoid this cast? */
					    (RESMAN_FREE_FN) &
					    PVRSRVServerSyncFreeKM);
	if (hSyncHandleInt2 == IMG_NULL) {
		psServerSyncAllocOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto ServerSyncAlloc_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psServerSyncAllocOUT->hSyncHandle,
			    (IMG_HANDLE) hSyncHandleInt2,
			    PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psServerSyncAllocOUT->eError,
				     psConnection);

 ServerSyncAlloc_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeServerSyncFree(IMG_UINT32 ui32BridgeID,
			   PVRSRV_BRIDGE_IN_SERVERSYNCFREE * psServerSyncFreeIN,
			   PVRSRV_BRIDGE_OUT_SERVERSYNCFREE *
			   psServerSyncFreeOUT, CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SERVERSYNCFREE);

	/* Look up the address from the handle */
	psServerSyncFreeOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psServerSyncFreeIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
	if (psServerSyncFreeOUT->eError != PVRSRV_OK) {
		goto ServerSyncFree_exit;
	}

	psServerSyncFreeOUT->eError =
	    ServerSyncFreeResManProxy(hSyncHandleInt2);
	/* Exit early if bridged call fails */
	if (psServerSyncFreeOUT->eError != PVRSRV_OK) {
		goto ServerSyncFree_exit;
	}

	psServerSyncFreeOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psServerSyncFreeIN->hSyncHandle,
				PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);

 ServerSyncFree_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeServerSyncQueueHWOp(IMG_UINT32 ui32BridgeID,
				PVRSRV_BRIDGE_IN_SERVERSYNCQUEUEHWOP *
				psServerSyncQueueHWOpIN,
				PVRSRV_BRIDGE_OUT_SERVERSYNCQUEUEHWOP *
				psServerSyncQueueHWOpOUT,
				CONNECTION_DATA * psConnection)
{
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SERVERSYNCQUEUEHWOP);

	/* Look up the address from the handle */
	psServerSyncQueueHWOpOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psServerSyncQueueHWOpIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
	if (psServerSyncQueueHWOpOUT->eError != PVRSRV_OK) {
		goto ServerSyncQueueHWOp_exit;
	}

	/* Look up the data from the resman address */
	psServerSyncQueueHWOpOUT->eError =
	    ResManFindPrivateDataByPtr(hSyncHandleInt2,
				       (IMG_VOID **) & psSyncHandleInt,
				       IMG_NULL);
	if (psServerSyncQueueHWOpOUT->eError != PVRSRV_OK) {
		goto ServerSyncQueueHWOp_exit;
	}

	psServerSyncQueueHWOpOUT->eError =
	    PVRSRVServerSyncQueueHWOpKM(psSyncHandleInt,
					&psServerSyncQueueHWOpOUT->
					ui32FenceValue,
					&psServerSyncQueueHWOpOUT->
					ui32UpdateValue);

 ServerSyncQueueHWOp_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeServerSyncGetStatus(IMG_UINT32 ui32BridgeID,
				PVRSRV_BRIDGE_IN_SERVERSYNCGETSTATUS *
				psServerSyncGetStatusIN,
				PVRSRV_BRIDGE_OUT_SERVERSYNCGETSTATUS *
				psServerSyncGetStatusOUT,
				CONNECTION_DATA * psConnection)
{
	SERVER_SYNC_PRIMITIVE **psSyncHandleInt = IMG_NULL;
	IMG_HANDLE *hSyncHandleInt2 = IMG_NULL;
	IMG_UINT32 *pui32UIDInt = IMG_NULL;
	IMG_UINT32 *pui32FWAddrInt = IMG_NULL;
	IMG_UINT32 *pui32CurrentOpInt = IMG_NULL;
	IMG_UINT32 *pui32NextOpInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SERVERSYNCGETSTATUS);

	psSyncHandleInt =
	    kmalloc(psServerSyncGetStatusIN->ui32SyncCount *
		    sizeof(SERVER_SYNC_PRIMITIVE *), GFP_KERNEL);
	if (!psSyncHandleInt) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto ServerSyncGetStatus_exit;
	}

	hSyncHandleInt2 =
	    kmalloc(psServerSyncGetStatusIN->ui32SyncCount * sizeof(IMG_HANDLE),
		    GFP_KERNEL);
	if (!hSyncHandleInt2) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto ServerSyncGetStatus_exit;
	}

	if (copy_from_user
	    (hSyncHandleInt2, psServerSyncGetStatusIN->phSyncHandle,
	     psServerSyncGetStatusIN->ui32SyncCount * sizeof(IMG_HANDLE)) !=
	    0) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto ServerSyncGetStatus_exit;
	}

	pui32UIDInt =
	    kmalloc(psServerSyncGetStatusIN->ui32SyncCount * sizeof(IMG_UINT32),
		    GFP_KERNEL);
	if (!pui32UIDInt) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto ServerSyncGetStatus_exit;
	}

	pui32FWAddrInt =
	    kmalloc(psServerSyncGetStatusIN->ui32SyncCount * sizeof(IMG_UINT32),
		    GFP_KERNEL);
	if (!pui32FWAddrInt) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto ServerSyncGetStatus_exit;
	}

	pui32CurrentOpInt =
	    kmalloc(psServerSyncGetStatusIN->ui32SyncCount * sizeof(IMG_UINT32),
		    GFP_KERNEL);
	if (!pui32CurrentOpInt) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto ServerSyncGetStatus_exit;
	}

	pui32NextOpInt =
	    kmalloc(psServerSyncGetStatusIN->ui32SyncCount * sizeof(IMG_UINT32),
		    GFP_KERNEL);
	if (!pui32NextOpInt) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto ServerSyncGetStatus_exit;
	}

	{
		IMG_UINT32 i;

		for (i = 0; i < psServerSyncGetStatusIN->ui32SyncCount; i++) {
			/* Look up the address from the handle */
			psServerSyncGetStatusOUT->eError =
			    PVRSRVLookupHandle(psConnection->psHandleBase,
					       (IMG_HANDLE *) &
					       hSyncHandleInt2[i],
					       hSyncHandleInt2[i],
					       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
			if (psServerSyncGetStatusOUT->eError != PVRSRV_OK) {
				goto ServerSyncGetStatus_exit;
			}

			/* Look up the data from the resman address */
			psServerSyncGetStatusOUT->eError =
			    ResManFindPrivateDataByPtr(hSyncHandleInt2[i],
						       (IMG_VOID **) &
						       psSyncHandleInt[i],
						       IMG_NULL);
			if (psServerSyncGetStatusOUT->eError != PVRSRV_OK) {
				goto ServerSyncGetStatus_exit;
			}
		}
	}

	psServerSyncGetStatusOUT->eError =
	    PVRSRVServerSyncGetStatusKM(psServerSyncGetStatusIN->ui32SyncCount,
					psSyncHandleInt,
					pui32UIDInt,
					pui32FWAddrInt,
					pui32CurrentOpInt, pui32NextOpInt);

	if (copy_to_user(psServerSyncGetStatusOUT->pui32UID, pui32UIDInt,
			 (psServerSyncGetStatusIN->ui32SyncCount *
			  sizeof(IMG_UINT32))) != 0) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto ServerSyncGetStatus_exit;
	}

	if (copy_to_user(psServerSyncGetStatusOUT->pui32FWAddr, pui32FWAddrInt,
			 (psServerSyncGetStatusIN->ui32SyncCount *
			  sizeof(IMG_UINT32))) != 0) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto ServerSyncGetStatus_exit;
	}

	if (copy_to_user
	    (psServerSyncGetStatusOUT->pui32CurrentOp, pui32CurrentOpInt,
	     (psServerSyncGetStatusIN->ui32SyncCount * sizeof(IMG_UINT32))) !=
	    0) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto ServerSyncGetStatus_exit;
	}

	if (copy_to_user(psServerSyncGetStatusOUT->pui32NextOp, pui32NextOpInt,
			 (psServerSyncGetStatusIN->ui32SyncCount *
			  sizeof(IMG_UINT32))) != 0) {
		psServerSyncGetStatusOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto ServerSyncGetStatus_exit;
	}

 ServerSyncGetStatus_exit:
	if (psSyncHandleInt)
		kfree(psSyncHandleInt);
	if (hSyncHandleInt2)
		kfree(hSyncHandleInt2);
	if (pui32UIDInt)
		kfree(pui32UIDInt);
	if (pui32FWAddrInt)
		kfree(pui32FWAddrInt);
	if (pui32CurrentOpInt)
		kfree(pui32CurrentOpInt);
	if (pui32NextOpInt)
		kfree(pui32NextOpInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimOpCreate(IMG_UINT32 ui32BridgeID,
			     PVRSRV_BRIDGE_IN_SYNCPRIMOPCREATE *
			     psSyncPrimOpCreateIN,
			     PVRSRV_BRIDGE_OUT_SYNCPRIMOPCREATE *
			     psSyncPrimOpCreateOUT,
			     CONNECTION_DATA * psConnection)
{
	SYNC_PRIMITIVE_BLOCK **psBlockListInt = IMG_NULL;
	IMG_HANDLE *hBlockListInt2 = IMG_NULL;
	IMG_UINT32 *ui32SyncBlockIndexInt = IMG_NULL;
	IMG_UINT32 *ui32IndexInt = IMG_NULL;
	SERVER_SYNC_PRIMITIVE **psServerSyncInt = IMG_NULL;
	IMG_HANDLE *hServerSyncInt2 = IMG_NULL;
	SERVER_OP_COOKIE *psServerCookieInt;
	IMG_HANDLE hServerCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMOPCREATE);

	psBlockListInt =
	    kmalloc(psSyncPrimOpCreateIN->ui32SyncBlockCount *
		    sizeof(SYNC_PRIMITIVE_BLOCK *), GFP_KERNEL);
	if (!psBlockListInt) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpCreate_exit;
	}

	hBlockListInt2 =
	    kmalloc(psSyncPrimOpCreateIN->ui32SyncBlockCount *
		    sizeof(IMG_HANDLE), GFP_KERNEL);
	if (!hBlockListInt2) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpCreate_exit;
	}

	if (copy_from_user(hBlockListInt2, psSyncPrimOpCreateIN->phBlockList,
			   psSyncPrimOpCreateIN->ui32SyncBlockCount *
			   sizeof(IMG_HANDLE)) != 0) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto SyncPrimOpCreate_exit;
	}

	ui32SyncBlockIndexInt =
	    kmalloc(psSyncPrimOpCreateIN->ui32ClientSyncCount *
		    sizeof(IMG_UINT32), GFP_KERNEL);
	if (!ui32SyncBlockIndexInt) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpCreate_exit;
	}

	if (copy_from_user
	    (ui32SyncBlockIndexInt, psSyncPrimOpCreateIN->pui32SyncBlockIndex,
	     psSyncPrimOpCreateIN->ui32ClientSyncCount * sizeof(IMG_UINT32)) !=
	    0) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto SyncPrimOpCreate_exit;
	}

	ui32IndexInt =
	    kmalloc(psSyncPrimOpCreateIN->ui32ClientSyncCount *
		    sizeof(IMG_UINT32), GFP_KERNEL);
	if (!ui32IndexInt) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpCreate_exit;
	}

	if (copy_from_user(ui32IndexInt, psSyncPrimOpCreateIN->pui32Index,
			   psSyncPrimOpCreateIN->ui32ClientSyncCount *
			   sizeof(IMG_UINT32)) != 0) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto SyncPrimOpCreate_exit;
	}

	psServerSyncInt =
	    kmalloc(psSyncPrimOpCreateIN->ui32ServerSyncCount *
		    sizeof(SERVER_SYNC_PRIMITIVE *), GFP_KERNEL);
	if (!psServerSyncInt) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpCreate_exit;
	}

	hServerSyncInt2 =
	    kmalloc(psSyncPrimOpCreateIN->ui32ServerSyncCount *
		    sizeof(IMG_HANDLE), GFP_KERNEL);
	if (!hServerSyncInt2) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpCreate_exit;
	}

	if (copy_from_user(hServerSyncInt2, psSyncPrimOpCreateIN->phServerSync,
			   psSyncPrimOpCreateIN->ui32ServerSyncCount *
			   sizeof(IMG_HANDLE)) != 0) {
		psSyncPrimOpCreateOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto SyncPrimOpCreate_exit;
	}

	NEW_HANDLE_BATCH_OR_ERROR(psSyncPrimOpCreateOUT->eError, psConnection,
				  1)
	{
		IMG_UINT32 i;

		for (i = 0; i < psSyncPrimOpCreateIN->ui32SyncBlockCount; i++) {
			/* Look up the address from the handle */
			psSyncPrimOpCreateOUT->eError =
			    PVRSRVLookupHandle(psConnection->psHandleBase,
					       (IMG_HANDLE *) &
					       hBlockListInt2[i],
					       hBlockListInt2[i],
					       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
			if (psSyncPrimOpCreateOUT->eError != PVRSRV_OK) {
				goto SyncPrimOpCreate_exit;
			}

			/* Look up the data from the resman address */
			psSyncPrimOpCreateOUT->eError =
			    ResManFindPrivateDataByPtr(hBlockListInt2[i],
						       (IMG_VOID **) &
						       psBlockListInt[i],
						       IMG_NULL);
			if (psSyncPrimOpCreateOUT->eError != PVRSRV_OK) {
				goto SyncPrimOpCreate_exit;
			}
		}
	}
	{
		IMG_UINT32 i;

		for (i = 0; i < psSyncPrimOpCreateIN->ui32ServerSyncCount; i++) {
			/* Look up the address from the handle */
			psSyncPrimOpCreateOUT->eError =
			    PVRSRVLookupHandle(psConnection->psHandleBase,
					       (IMG_HANDLE *) &
					       hServerSyncInt2[i],
					       hServerSyncInt2[i],
					       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
			if (psSyncPrimOpCreateOUT->eError != PVRSRV_OK) {
				goto SyncPrimOpCreate_exit;
			}

			/* Look up the data from the resman address */
			psSyncPrimOpCreateOUT->eError =
			    ResManFindPrivateDataByPtr(hServerSyncInt2[i],
						       (IMG_VOID **) &
						       psServerSyncInt[i],
						       IMG_NULL);
			if (psSyncPrimOpCreateOUT->eError != PVRSRV_OK) {
				goto SyncPrimOpCreate_exit;
			}
		}
	}

	psSyncPrimOpCreateOUT->eError =
	    PVRSRVSyncPrimOpCreateKM(psSyncPrimOpCreateIN->ui32SyncBlockCount,
				     psBlockListInt,
				     psSyncPrimOpCreateIN->ui32ClientSyncCount,
				     ui32SyncBlockIndexInt,
				     ui32IndexInt,
				     psSyncPrimOpCreateIN->ui32ServerSyncCount,
				     psServerSyncInt, &psServerCookieInt);
	/* Exit early if bridged call fails */
	if (psSyncPrimOpCreateOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpCreate_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hServerCookieInt2 = ResManRegisterRes(psConnection->hResManContext,
					      RESMAN_TYPE_SERVER_OP_COOKIE,
					      psServerCookieInt, 0,
					      /* FIXME: how can we avoid this cast? */
					      (RESMAN_FREE_FN) &
					      PVRSRVSyncPrimOpDestroyKM);
	if (hServerCookieInt2 == IMG_NULL) {
		psSyncPrimOpCreateOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto SyncPrimOpCreate_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psSyncPrimOpCreateOUT->hServerCookie,
			    (IMG_HANDLE) hServerCookieInt2,
			    PVRSRV_HANDLE_TYPE_SERVER_OP_COOKIE,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psSyncPrimOpCreateOUT->eError,
				     psConnection);

 SyncPrimOpCreate_exit:
	if (psBlockListInt)
		kfree(psBlockListInt);
	if (hBlockListInt2)
		kfree(hBlockListInt2);
	if (ui32SyncBlockIndexInt)
		kfree(ui32SyncBlockIndexInt);
	if (ui32IndexInt)
		kfree(ui32IndexInt);
	if (psServerSyncInt)
		kfree(psServerSyncInt);
	if (hServerSyncInt2)
		kfree(hServerSyncInt2);

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimOpTake(IMG_UINT32 ui32BridgeID,
			   PVRSRV_BRIDGE_IN_SYNCPRIMOPTAKE * psSyncPrimOpTakeIN,
			   PVRSRV_BRIDGE_OUT_SYNCPRIMOPTAKE *
			   psSyncPrimOpTakeOUT, CONNECTION_DATA * psConnection)
{
	SERVER_OP_COOKIE *psServerCookieInt;
	IMG_HANDLE hServerCookieInt2;
	IMG_UINT32 *ui32FlagsInt = IMG_NULL;
	IMG_UINT32 *ui32FenceValueInt = IMG_NULL;
	IMG_UINT32 *ui32UpdateValueInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMOPTAKE);

	ui32FlagsInt =
	    kmalloc(psSyncPrimOpTakeIN->ui32ClientSyncCount *
		    sizeof(IMG_UINT32), GFP_KERNEL);
	if (!ui32FlagsInt) {
		psSyncPrimOpTakeOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpTake_exit;
	}

	if (copy_from_user(ui32FlagsInt, psSyncPrimOpTakeIN->pui32Flags,
			   psSyncPrimOpTakeIN->ui32ClientSyncCount *
			   sizeof(IMG_UINT32)) != 0) {
		psSyncPrimOpTakeOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto SyncPrimOpTake_exit;
	}

	ui32FenceValueInt =
	    kmalloc(psSyncPrimOpTakeIN->ui32ClientSyncCount *
		    sizeof(IMG_UINT32), GFP_KERNEL);
	if (!ui32FenceValueInt) {
		psSyncPrimOpTakeOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpTake_exit;
	}

	if (copy_from_user
	    (ui32FenceValueInt, psSyncPrimOpTakeIN->pui32FenceValue,
	     psSyncPrimOpTakeIN->ui32ClientSyncCount * sizeof(IMG_UINT32)) !=
	    0) {
		psSyncPrimOpTakeOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto SyncPrimOpTake_exit;
	}

	ui32UpdateValueInt =
	    kmalloc(psSyncPrimOpTakeIN->ui32ClientSyncCount *
		    sizeof(IMG_UINT32), GFP_KERNEL);
	if (!ui32UpdateValueInt) {
		psSyncPrimOpTakeOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto SyncPrimOpTake_exit;
	}

	if (copy_from_user
	    (ui32UpdateValueInt, psSyncPrimOpTakeIN->pui32UpdateValue,
	     psSyncPrimOpTakeIN->ui32ClientSyncCount * sizeof(IMG_UINT32)) !=
	    0) {
		psSyncPrimOpTakeOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto SyncPrimOpTake_exit;
	}

	/* Look up the address from the handle */
	psSyncPrimOpTakeOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hServerCookieInt2,
			       psSyncPrimOpTakeIN->hServerCookie,
			       PVRSRV_HANDLE_TYPE_SERVER_OP_COOKIE);
	if (psSyncPrimOpTakeOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpTake_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimOpTakeOUT->eError =
	    ResManFindPrivateDataByPtr(hServerCookieInt2,
				       (IMG_VOID **) & psServerCookieInt,
				       IMG_NULL);
	if (psSyncPrimOpTakeOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpTake_exit;
	}

	psSyncPrimOpTakeOUT->eError =
	    PVRSRVSyncPrimOpTakeKM(psServerCookieInt,
				   psSyncPrimOpTakeIN->ui32ClientSyncCount,
				   ui32FlagsInt,
				   ui32FenceValueInt,
				   ui32UpdateValueInt,
				   psSyncPrimOpTakeIN->ui32ServerSyncCount);

 SyncPrimOpTake_exit:
	if (ui32FlagsInt)
		kfree(ui32FlagsInt);
	if (ui32FenceValueInt)
		kfree(ui32FenceValueInt);
	if (ui32UpdateValueInt)
		kfree(ui32UpdateValueInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimOpReady(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_SYNCPRIMOPREADY *
			    psSyncPrimOpReadyIN,
			    PVRSRV_BRIDGE_OUT_SYNCPRIMOPREADY *
			    psSyncPrimOpReadyOUT,
			    CONNECTION_DATA * psConnection)
{
	SERVER_OP_COOKIE *psServerCookieInt;
	IMG_HANDLE hServerCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMOPREADY);

	/* Look up the address from the handle */
	psSyncPrimOpReadyOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hServerCookieInt2,
			       psSyncPrimOpReadyIN->hServerCookie,
			       PVRSRV_HANDLE_TYPE_SERVER_OP_COOKIE);
	if (psSyncPrimOpReadyOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpReady_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimOpReadyOUT->eError =
	    ResManFindPrivateDataByPtr(hServerCookieInt2,
				       (IMG_VOID **) & psServerCookieInt,
				       IMG_NULL);
	if (psSyncPrimOpReadyOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpReady_exit;
	}

	psSyncPrimOpReadyOUT->eError =
	    PVRSRVSyncPrimOpReadyKM(psServerCookieInt,
				    &psSyncPrimOpReadyOUT->bReady);

 SyncPrimOpReady_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimOpComplete(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_SYNCPRIMOPCOMPLETE *
			       psSyncPrimOpCompleteIN,
			       PVRSRV_BRIDGE_OUT_SYNCPRIMOPCOMPLETE *
			       psSyncPrimOpCompleteOUT,
			       CONNECTION_DATA * psConnection)
{
	SERVER_OP_COOKIE *psServerCookieInt;
	IMG_HANDLE hServerCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMOPCOMPLETE);

	/* Look up the address from the handle */
	psSyncPrimOpCompleteOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hServerCookieInt2,
			       psSyncPrimOpCompleteIN->hServerCookie,
			       PVRSRV_HANDLE_TYPE_SERVER_OP_COOKIE);
	if (psSyncPrimOpCompleteOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpComplete_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimOpCompleteOUT->eError =
	    ResManFindPrivateDataByPtr(hServerCookieInt2,
				       (IMG_VOID **) & psServerCookieInt,
				       IMG_NULL);
	if (psSyncPrimOpCompleteOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpComplete_exit;
	}

	psSyncPrimOpCompleteOUT->eError =
	    PVRSRVSyncPrimOpCompleteKM(psServerCookieInt);

 SyncPrimOpComplete_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimOpDestroy(IMG_UINT32 ui32BridgeID,
			      PVRSRV_BRIDGE_IN_SYNCPRIMOPDESTROY *
			      psSyncPrimOpDestroyIN,
			      PVRSRV_BRIDGE_OUT_SYNCPRIMOPDESTROY *
			      psSyncPrimOpDestroyOUT,
			      CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hServerCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMOPDESTROY);

	/* Look up the address from the handle */
	psSyncPrimOpDestroyOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hServerCookieInt2,
			       psSyncPrimOpDestroyIN->hServerCookie,
			       PVRSRV_HANDLE_TYPE_SERVER_OP_COOKIE);
	if (psSyncPrimOpDestroyOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpDestroy_exit;
	}

	psSyncPrimOpDestroyOUT->eError =
	    SyncPrimOpDestroyResManProxy(hServerCookieInt2);
	/* Exit early if bridged call fails */
	if (psSyncPrimOpDestroyOUT->eError != PVRSRV_OK) {
		goto SyncPrimOpDestroy_exit;
	}

	psSyncPrimOpDestroyOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psSyncPrimOpDestroyIN->
				hServerCookie,
				PVRSRV_HANDLE_TYPE_SERVER_OP_COOKIE);

 SyncPrimOpDestroy_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimPDump(IMG_UINT32 ui32BridgeID,
			  PVRSRV_BRIDGE_IN_SYNCPRIMPDUMP * psSyncPrimPDumpIN,
			  PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMP * psSyncPrimPDumpOUT,
			  CONNECTION_DATA * psConnection)
{
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMP);

	/* Look up the address from the handle */
	psSyncPrimPDumpOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psSyncPrimPDumpIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
	if (psSyncPrimPDumpOUT->eError != PVRSRV_OK) {
		goto SyncPrimPDump_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimPDumpOUT->eError =
	    ResManFindPrivateDataByPtr(hSyncHandleInt2,
				       (IMG_VOID **) & psSyncHandleInt,
				       IMG_NULL);
	if (psSyncPrimPDumpOUT->eError != PVRSRV_OK) {
		goto SyncPrimPDump_exit;
	}

	psSyncPrimPDumpOUT->eError =
	    PVRSRVSyncPrimPDumpKM(psSyncHandleInt,
				  psSyncPrimPDumpIN->ui32Offset);

 SyncPrimPDump_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimPDumpPol(IMG_UINT32 ui32BridgeID,
			     PVRSRV_BRIDGE_IN_SYNCPRIMPDUMPPOL *
			     psSyncPrimPDumpPolIN,
			     PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMPPOL *
			     psSyncPrimPDumpPolOUT,
			     CONNECTION_DATA * psConnection)
{
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMPPOL);

	/* Look up the address from the handle */
	psSyncPrimPDumpPolOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psSyncPrimPDumpPolIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
	if (psSyncPrimPDumpPolOUT->eError != PVRSRV_OK) {
		goto SyncPrimPDumpPol_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimPDumpPolOUT->eError =
	    ResManFindPrivateDataByPtr(hSyncHandleInt2,
				       (IMG_VOID **) & psSyncHandleInt,
				       IMG_NULL);
	if (psSyncPrimPDumpPolOUT->eError != PVRSRV_OK) {
		goto SyncPrimPDumpPol_exit;
	}

	psSyncPrimPDumpPolOUT->eError =
	    PVRSRVSyncPrimPDumpPolKM(psSyncHandleInt,
				     psSyncPrimPDumpPolIN->ui32Offset,
				     psSyncPrimPDumpPolIN->ui32Value,
				     psSyncPrimPDumpPolIN->ui32Mask,
				     psSyncPrimPDumpPolIN->eOperator,
				     psSyncPrimPDumpPolIN->uiPDumpFlags);

 SyncPrimPDumpPol_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimPDumpCBP(IMG_UINT32 ui32BridgeID,
			     PVRSRV_BRIDGE_IN_SYNCPRIMPDUMPCBP *
			     psSyncPrimPDumpCBPIN,
			     PVRSRV_BRIDGE_OUT_SYNCPRIMPDUMPCBP *
			     psSyncPrimPDumpCBPOUT,
			     CONNECTION_DATA * psConnection)
{
	SYNC_PRIMITIVE_BLOCK *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMPCBP);

	/* Look up the address from the handle */
	psSyncPrimPDumpCBPOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psSyncPrimPDumpCBPIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SYNC_PRIMITIVE_BLOCK);
	if (psSyncPrimPDumpCBPOUT->eError != PVRSRV_OK) {
		goto SyncPrimPDumpCBP_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimPDumpCBPOUT->eError =
	    ResManFindPrivateDataByPtr(hSyncHandleInt2,
				       (IMG_VOID **) & psSyncHandleInt,
				       IMG_NULL);
	if (psSyncPrimPDumpCBPOUT->eError != PVRSRV_OK) {
		goto SyncPrimPDumpCBP_exit;
	}

	psSyncPrimPDumpCBPOUT->eError =
	    PVRSRVSyncPrimPDumpCBPKM(psSyncHandleInt,
				     psSyncPrimPDumpCBPIN->ui32Offset,
				     psSyncPrimPDumpCBPIN->ui32WriteOffset,
				     psSyncPrimPDumpCBPIN->ui32PacketSize,
				     psSyncPrimPDumpCBPIN->ui32BufferSize);

 SyncPrimPDumpCBP_exit:

	return 0;
}

PVRSRV_ERROR RegisterSYNCFunctions(IMG_VOID);
IMG_VOID UnregisterSYNCFunctions(IMG_VOID);

/*
 * Register all SYNC functions with services
 */
PVRSRV_ERROR RegisterSYNCFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_ALLOCSYNCPRIMITIVEBLOCK,
			      PVRSRVBridgeAllocSyncPrimitiveBlock);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_FREESYNCPRIMITIVEBLOCK,
			      PVRSRVBridgeFreeSyncPrimitiveBlock);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMSET,
			      PVRSRVBridgeSyncPrimSet);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SERVERSYNCPRIMSET,
			      PVRSRVBridgeServerSyncPrimSet);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SERVERSYNCALLOC,
			      PVRSRVBridgeServerSyncAlloc);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SERVERSYNCFREE,
			      PVRSRVBridgeServerSyncFree);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SERVERSYNCQUEUEHWOP,
			      PVRSRVBridgeServerSyncQueueHWOp);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SERVERSYNCGETSTATUS,
			      PVRSRVBridgeServerSyncGetStatus);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMOPCREATE,
			      PVRSRVBridgeSyncPrimOpCreate);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMOPTAKE,
			      PVRSRVBridgeSyncPrimOpTake);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMOPREADY,
			      PVRSRVBridgeSyncPrimOpReady);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMOPCOMPLETE,
			      PVRSRVBridgeSyncPrimOpComplete);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMOPDESTROY,
			      PVRSRVBridgeSyncPrimOpDestroy);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMP,
			      PVRSRVBridgeSyncPrimPDump);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMPPOL,
			      PVRSRVBridgeSyncPrimPDumpPol);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNC_SYNCPRIMPDUMPCBP,
			      PVRSRVBridgeSyncPrimPDumpCBP);

	return PVRSRV_OK;
}

/*
 * Unregister all sync functions with services
 */
IMG_VOID UnregisterSYNCFunctions(IMG_VOID)
{
}
