									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for rgxccb
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for rgxccb
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "rgxccb.h"

#include "common_rgxccb_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR RGXDestroyCCBResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static IMG_INT
PVRSRVBridgeRGXCreateCCB(IMG_UINT32 ui32BridgeID,
			 PVRSRV_BRIDGE_IN_RGXCREATECCB * psRGXCreateCCBIN,
			 PVRSRV_BRIDGE_OUT_RGXCREATECCB * psRGXCreateCCBOUT,
			 CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	RGX_CCB_CLEANUP_DATA *psCleanupCookieInt;
	IMG_HANDLE hCleanupCookieInt2;
	DEVMEM_MEMDESC *psClientCCBMemDescInt;
	DEVMEM_MEMDESC *psClientCCBCtlMemDescInt;
	DEVMEM_EXPORTCOOKIE *psClientCCBExportCookieInt;
	DEVMEM_EXPORTCOOKIE *psClientCCBCtlExportCookieInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXCCB_RGXCREATECCB);

	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateCCBOUT->eError, psConnection, 5)

	    /* Look up the address from the handle */
	    psRGXCreateCCBOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXCreateCCBIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXCreateCCBOUT->eError != PVRSRV_OK) {
		goto RGXCreateCCB_exit;
	}

	psRGXCreateCCBOUT->eError =
	    PVRSRVRGXCreateCCBKM(hDevNodeInt,
				 psRGXCreateCCBIN->ui32AllocSize,
				 psRGXCreateCCBIN->ui32AllocAlignment,
				 &psCleanupCookieInt,
				 &psClientCCBMemDescInt,
				 &psClientCCBCtlMemDescInt,
				 &psClientCCBExportCookieInt,
				 &psClientCCBCtlExportCookieInt);
	/* Exit early if bridged call fails */
	if (psRGXCreateCCBOUT->eError != PVRSRV_OK) {
		goto RGXCreateCCB_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hCleanupCookieInt2 = ResManRegisterRes(psConnection->hResManContext,
					       RESMAN_TYPE_RGX_CCB,
					       psCleanupCookieInt, 0,
					       /* FIXME: how can we avoid this cast? */
					       (RESMAN_FREE_FN) &
					       PVRSRVRGXDestroyCCBKM);
	if (hCleanupCookieInt2 == IMG_NULL) {
		psRGXCreateCCBOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateCCB_exit;
	}
	psRGXCreateCCBOUT->eError =
	    PVRSRVAllocHandle(psConnection->psHandleBase,
			      &psRGXCreateCCBOUT->hCleanupCookie,
			      (IMG_HANDLE) hCleanupCookieInt2,
			      PVRSRV_HANDLE_TYPE_RGX_CCB_CLEANUP,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psRGXCreateCCBOUT->eError != PVRSRV_OK) {
		goto RGXCreateCCB_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psRGXCreateCCBOUT->hClientCCBMemDesc,
			       (IMG_HANDLE) psClientCCBMemDescInt,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psRGXCreateCCBOUT->hCleanupCookie);
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psRGXCreateCCBOUT->hClientCCBCtlMemDesc,
			       (IMG_HANDLE) psClientCCBCtlMemDescInt,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psRGXCreateCCBOUT->hCleanupCookie);
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psRGXCreateCCBOUT->hClientCCBExportCookie,
			       (IMG_HANDLE) psClientCCBExportCookieInt,
			       PVRSRV_HANDLE_TYPE_SERVER_EXPORTCOOKIE,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psRGXCreateCCBOUT->hCleanupCookie);
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psRGXCreateCCBOUT->hClientCCBCtlExportCookie,
			       (IMG_HANDLE) psClientCCBCtlExportCookieInt,
			       PVRSRV_HANDLE_TYPE_SERVER_EXPORTCOOKIE,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psRGXCreateCCBOUT->hCleanupCookie);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateCCBOUT->eError, psConnection);

 RGXCreateCCB_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyCCB(IMG_UINT32 ui32BridgeID,
			  PVRSRV_BRIDGE_IN_RGXDESTROYCCB * psRGXDestroyCCBIN,
			  PVRSRV_BRIDGE_OUT_RGXDESTROYCCB * psRGXDestroyCCBOUT,
			  CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hCleanupCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXCCB_RGXDESTROYCCB);

	/* Look up the address from the handle */
	psRGXDestroyCCBOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hCleanupCookieInt2,
			       psRGXDestroyCCBIN->hCleanupCookie,
			       PVRSRV_HANDLE_TYPE_RGX_CCB_CLEANUP);
	if (psRGXDestroyCCBOUT->eError != PVRSRV_OK) {
		goto RGXDestroyCCB_exit;
	}

	psRGXDestroyCCBOUT->eError =
	    RGXDestroyCCBResManProxy(hCleanupCookieInt2);
	/* Exit early if bridged call fails */
	if (psRGXDestroyCCBOUT->eError != PVRSRV_OK) {
		goto RGXDestroyCCB_exit;
	}

	psRGXDestroyCCBOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psRGXDestroyCCBIN->hCleanupCookie,
				PVRSRV_HANDLE_TYPE_RGX_CCB_CLEANUP);

 RGXDestroyCCB_exit:

	return 0;
}

PVRSRV_ERROR RegisterRGXCCBFunctions(IMG_VOID);
IMG_VOID UnregisterRGXCCBFunctions(IMG_VOID);

/*
 * Register all RGXCCB functions with services
 */
PVRSRV_ERROR RegisterRGXCCBFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCCB_RGXCREATECCB,
			      PVRSRVBridgeRGXCreateCCB);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCCB_RGXDESTROYCCB,
			      PVRSRVBridgeRGXDestroyCCB);

	return PVRSRV_OK;
}

/*
 * Unregister all rgxccb functions with services
 */
IMG_VOID UnregisterRGXCCBFunctions(IMG_VOID)
{
}
