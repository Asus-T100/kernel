									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for syncexport
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for syncexport
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "sync_server.h"

#include "common_syncexport_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR SyncPrimServerUnexportResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static IMG_INT
PVRSRVBridgeSyncPrimServerExport(IMG_UINT32 ui32BridgeID,
				 PVRSRV_BRIDGE_IN_SYNCPRIMSERVEREXPORT *
				 psSyncPrimServerExportIN,
				 PVRSRV_BRIDGE_OUT_SYNCPRIMSERVEREXPORT *
				 psSyncPrimServerExportOUT,
				 CONNECTION_DATA * psConnection)
{
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;
	SERVER_SYNC_EXPORT *psExportInt;
	IMG_HANDLE hExportInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVEREXPORT);

	/* Look up the address from the handle */
	psSyncPrimServerExportOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hSyncHandleInt2,
			       psSyncPrimServerExportIN->hSyncHandle,
			       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
	if (psSyncPrimServerExportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerExport_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimServerExportOUT->eError =
	    ResManFindPrivateDataByPtr(hSyncHandleInt2,
				       (IMG_VOID **) & psSyncHandleInt,
				       IMG_NULL);
	if (psSyncPrimServerExportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerExport_exit;
	}

	psSyncPrimServerExportOUT->eError =
	    PVRSRVSyncPrimServerExportKM(psSyncHandleInt, &psExportInt);
	/* Exit early if bridged call fails */
	if (psSyncPrimServerExportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerExport_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hExportInt2 = ResManRegisterRes(psConnection->hResManContext,
					RESMAN_TYPE_SERVER_SYNC_EXPORT,
					psExportInt, 0,
					/* FIXME: how can we avoid this cast? */
					(RESMAN_FREE_FN) &
					PVRSRVSyncPrimServerUnexportKM);
	if (hExportInt2 == IMG_NULL) {
		psSyncPrimServerExportOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto SyncPrimServerExport_exit;
	}
	/* see if it's already exported */
	/* FIXME: This code should go when we implement serverImport, serverUnimport */
	psSyncPrimServerExportOUT->eError =
	    PVRSRVFindHandle(KERNEL_HANDLE_BASE,
			     &psSyncPrimServerExportOUT->hExport,
			     (IMG_HANDLE) hExportInt2,
			     PVRSRV_HANDLE_TYPE_SERVER_SYNC_EXPORT);
	if (psSyncPrimServerExportOUT->eError == PVRSRV_OK) {
		/* it's already exported */
		return 0;
	}

	psSyncPrimServerExportOUT->eError =
	    PVRSRVAllocHandle(KERNEL_HANDLE_BASE,
			      &psSyncPrimServerExportOUT->hExport,
			      (IMG_HANDLE) hExportInt2,
			      PVRSRV_HANDLE_TYPE_SERVER_SYNC_EXPORT,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psSyncPrimServerExportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerExport_exit;
	}

 SyncPrimServerExport_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimServerUnexport(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_SYNCPRIMSERVERUNEXPORT *
				   psSyncPrimServerUnexportIN,
				   PVRSRV_BRIDGE_OUT_SYNCPRIMSERVERUNEXPORT *
				   psSyncPrimServerUnexportOUT,
				   CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hExportInt2;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVERUNEXPORT);

	/* Look up the address from the handle */
	psSyncPrimServerUnexportOUT->eError =
	    PVRSRVLookupHandle(KERNEL_HANDLE_BASE,
			       (IMG_HANDLE *) & hExportInt2,
			       psSyncPrimServerUnexportIN->hExport,
			       PVRSRV_HANDLE_TYPE_SERVER_SYNC_EXPORT);
	if (psSyncPrimServerUnexportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerUnexport_exit;
	}

	psSyncPrimServerUnexportOUT->eError =
	    SyncPrimServerUnexportResManProxy(hExportInt2);
	/* Exit early if bridged call fails */
	if (psSyncPrimServerUnexportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerUnexport_exit;
	}

	psSyncPrimServerUnexportOUT->eError =
	    PVRSRVReleaseHandle(KERNEL_HANDLE_BASE,
				(IMG_HANDLE) psSyncPrimServerUnexportIN->
				hExport, PVRSRV_HANDLE_TYPE_SERVER_SYNC_EXPORT);

 SyncPrimServerUnexport_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSyncPrimServerImport(IMG_UINT32 ui32BridgeID,
				 PVRSRV_BRIDGE_IN_SYNCPRIMSERVERIMPORT *
				 psSyncPrimServerImportIN,
				 PVRSRV_BRIDGE_OUT_SYNCPRIMSERVERIMPORT *
				 psSyncPrimServerImportOUT,
				 CONNECTION_DATA * psConnection)
{
	SERVER_SYNC_EXPORT *psImportInt;
	IMG_HANDLE hImportInt2;
	SERVER_SYNC_PRIMITIVE *psSyncHandleInt;
	IMG_HANDLE hSyncHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVERIMPORT);

	/* Look up the address from the handle */
	psSyncPrimServerImportOUT->eError =
	    PVRSRVLookupHandle(KERNEL_HANDLE_BASE,
			       (IMG_HANDLE *) & hImportInt2,
			       psSyncPrimServerImportIN->hImport,
			       PVRSRV_HANDLE_TYPE_SERVER_SYNC_EXPORT);
	if (psSyncPrimServerImportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerImport_exit;
	}

	/* Look up the data from the resman address */
	psSyncPrimServerImportOUT->eError =
	    ResManFindPrivateDataByPtr(hImportInt2, (IMG_VOID **) & psImportInt,
				       IMG_NULL);
	if (psSyncPrimServerImportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerImport_exit;
	}

	psSyncPrimServerImportOUT->eError =
	    PVRSRVSyncPrimServerImportKM(psImportInt,
					 &psSyncHandleInt,
					 &psSyncPrimServerImportOUT->
					 ui32SyncPrimVAddr);
	/* Exit early if bridged call fails */
	if (psSyncPrimServerImportOUT->eError != PVRSRV_OK) {
		goto SyncPrimServerImport_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hSyncHandleInt2 = ResManRegisterRes(psConnection->hResManContext,
					    RESMAN_TYPE_SERVER_SYNC_PRIMITIVE,
					    psSyncHandleInt, 0,
					    /* FIXME: how can we avoid this cast? */
					    (RESMAN_FREE_FN) &
					    PVRSRVServerSyncFreeKM);
	if (hSyncHandleInt2 == IMG_NULL) {
		psSyncPrimServerImportOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto SyncPrimServerImport_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psSyncPrimServerImportOUT->hSyncHandle,
			    (IMG_HANDLE) hSyncHandleInt2,
			    PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);

 SyncPrimServerImport_exit:

	return 0;
}

PVRSRV_ERROR RegisterSYNCEXPORTFunctions(IMG_VOID);
IMG_VOID UnregisterSYNCEXPORTFunctions(IMG_VOID);

/*
 * Register all SYNCEXPORT functions with services
 */
PVRSRV_ERROR RegisterSYNCEXPORTFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVEREXPORT,
			      PVRSRVBridgeSyncPrimServerExport);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVERUNEXPORT,
			      PVRSRVBridgeSyncPrimServerUnexport);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCEXPORT_SYNCPRIMSERVERIMPORT,
			      PVRSRVBridgeSyncPrimServerImport);

	return PVRSRV_OK;
}

/*
 * Unregister all syncexport functions with services
 */
IMG_VOID UnregisterSYNCEXPORTFunctions(IMG_VOID)
{
}
