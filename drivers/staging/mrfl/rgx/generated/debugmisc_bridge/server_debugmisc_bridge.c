									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for debugmisc
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for debugmisc
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "debugmisc_server.h"

#include "common_debugmisc_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static IMG_INT
PVRSRVBridgeDebugMiscTilingSetState(IMG_UINT32 ui32BridgeID,
				    PVRSRV_BRIDGE_IN_DEBUGMISCTILINGSETSTATE *
				    psDebugMiscTilingSetStateIN,
				    PVRSRV_BRIDGE_OUT_DEBUGMISCTILINGSETSTATE *
				    psDebugMiscTilingSetStateOUT,
				    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	IMG_HANDLE hMemCtxPrivDataInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DEBUGMISC_DEBUGMISCTILINGSETSTATE);

	/* Look up the address from the handle */
	psDebugMiscTilingSetStateOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psDebugMiscTilingSetStateIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psDebugMiscTilingSetStateOUT->eError != PVRSRV_OK) {
		goto DebugMiscTilingSetState_exit;
	}
	/* Look up the address from the handle */
	psDebugMiscTilingSetStateOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hMemCtxPrivDataInt,
			       psDebugMiscTilingSetStateIN->hMemCtxPrivData,
			       PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	if (psDebugMiscTilingSetStateOUT->eError != PVRSRV_OK) {
		goto DebugMiscTilingSetState_exit;
	}

	psDebugMiscTilingSetStateOUT->eError =
	    PVRSRVDebugMiscTilingSetStateKM(hDevNodeInt,
					    hMemCtxPrivDataInt,
					    psDebugMiscTilingSetStateIN->
					    bbEnabled);

 DebugMiscTilingSetState_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDebugMiscSLCSetBypassState(IMG_UINT32 ui32BridgeID,
				       PVRSRV_BRIDGE_IN_DEBUGMISCSLCSETBYPASSSTATE
				       * psDebugMiscSLCSetBypassStateIN,
				       PVRSRV_BRIDGE_OUT_DEBUGMISCSLCSETBYPASSSTATE
				       * psDebugMiscSLCSetBypassStateOUT,
				       CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DEBUGMISC_DEBUGMISCSLCSETBYPASSSTATE);

	/* Look up the address from the handle */
	psDebugMiscSLCSetBypassStateOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psDebugMiscSLCSetBypassStateIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psDebugMiscSLCSetBypassStateOUT->eError != PVRSRV_OK) {
		goto DebugMiscSLCSetBypassState_exit;
	}

	psDebugMiscSLCSetBypassStateOUT->eError =
	    PVRSRVDebugMiscSLCSetBypassStateKM(hDevNodeInt,
					       psDebugMiscSLCSetBypassStateIN->
					       ui32Flags,
					       psDebugMiscSLCSetBypassStateIN->
					       bIsBypassed);

 DebugMiscSLCSetBypassState_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDebugMiscSetFWLog(IMG_UINT32 ui32BridgeID,
				 PVRSRV_BRIDGE_IN_RGXDEBUGMISCSETFWLOG *
				 psRGXDebugMiscSetFWLogIN,
				 PVRSRV_BRIDGE_OUT_RGXDEBUGMISCSETFWLOG *
				 psRGXDebugMiscSetFWLogOUT,
				 CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DEBUGMISC_RGXDEBUGMISCSETFWLOG);

	/* Look up the address from the handle */
	psRGXDebugMiscSetFWLogOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXDebugMiscSetFWLogIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXDebugMiscSetFWLogOUT->eError != PVRSRV_OK) {
		goto RGXDebugMiscSetFWLog_exit;
	}

	psRGXDebugMiscSetFWLogOUT->eError =
	    PVRSRVRGXDebugMiscSetFWLogKM(hDevNodeInt,
					 psRGXDebugMiscSetFWLogIN->
					 ui32RGXFWLogType);

 RGXDebugMiscSetFWLog_exit:

	return 0;
}

PVRSRV_ERROR RegisterDEBUGMISCFunctions(IMG_VOID);
IMG_VOID UnregisterDEBUGMISCFunctions(IMG_VOID);

/*
 * Register all DEBUGMISC functions with services
 */
PVRSRV_ERROR RegisterDEBUGMISCFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_DEBUGMISC_DEBUGMISCTILINGSETSTATE,
			      PVRSRVBridgeDebugMiscTilingSetState);
	SetDispatchTableEntry
	    (PVRSRV_BRIDGE_DEBUGMISC_DEBUGMISCSLCSETBYPASSSTATE,
	     PVRSRVBridgeDebugMiscSLCSetBypassState);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DEBUGMISC_RGXDEBUGMISCSETFWLOG,
			      PVRSRVBridgeRGXDebugMiscSetFWLog);

	return PVRSRV_OK;
}

/*
 * Unregister all debugmisc functions with services
 */
IMG_VOID UnregisterDEBUGMISCFunctions(IMG_VOID)
{
}
