									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for rgxinit
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for rgxinit
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "rgxinit.h"

#include "common_rgxinit_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static IMG_INT
PVRSRVBridgeRGXInitFirmware(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_RGXINITFIRMWARE *
			    psRGXInitFirmwareIN,
			    PVRSRV_BRIDGE_OUT_RGXINITFIRMWARE *
			    psRGXInitFirmwareOUT,
			    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_EXPORTCOOKIE *psFWMemAllocServerExportCookieInt;
	IMG_UINT32 *ui32RGXFWAlignChecksInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXINIT_RGXINITFIRMWARE);

	ui32RGXFWAlignChecksInt =
	    kmalloc(psRGXInitFirmwareIN->ui32RGXFWAlignChecksSize *
		    sizeof(IMG_UINT32), GFP_KERNEL);
	if (!ui32RGXFWAlignChecksInt) {
		psRGXInitFirmwareOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXInitFirmware_exit;
	}

	if (copy_from_user
	    (ui32RGXFWAlignChecksInt,
	     psRGXInitFirmwareIN->pui32RGXFWAlignChecks,
	     psRGXInitFirmwareIN->ui32RGXFWAlignChecksSize *
	     sizeof(IMG_UINT32)) != 0) {
		psRGXInitFirmwareOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXInitFirmware_exit;
	}

	NEW_HANDLE_BATCH_OR_ERROR(psRGXInitFirmwareOUT->eError, psConnection, 1)

	    /* Look up the address from the handle */
	    psRGXInitFirmwareOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXInitFirmwareIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXInitFirmwareOUT->eError != PVRSRV_OK) {
		goto RGXInitFirmware_exit;
	}

	psRGXInitFirmwareOUT->eError =
	    PVRSRVRGXInitFirmwareKM(hDevNodeInt,
				    psRGXInitFirmwareIN->uiFWMemAllocSize,
				    &psFWMemAllocServerExportCookieInt,
				    &psRGXInitFirmwareOUT->sFWMemDevVAddrBase,
				    &psRGXInitFirmwareOUT->ui32FWHeapBase,
				    &psRGXInitFirmwareOUT->spsRGXFwInit,
				    psRGXInitFirmwareIN->bEnableSignatureChecks,
				    psRGXInitFirmwareIN->
				    ui32SignatureChecksBufSize,
				    psRGXInitFirmwareIN->
				    ui32RGXFWAlignChecksSize,
				    ui32RGXFWAlignChecksInt,
				    psRGXInitFirmwareIN->ui32ui32ConfigFlags);
	/* Exit early if bridged call fails */
	if (psRGXInitFirmwareOUT->eError != PVRSRV_OK) {
		goto RGXInitFirmware_exit;
	}

	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psRGXInitFirmwareOUT->
			    hFWMemAllocServerExportCookie,
			    (IMG_HANDLE) psFWMemAllocServerExportCookieInt,
			    PVRSRV_HANDLE_TYPE_SERVER_EXPORTCOOKIE,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXInitFirmwareOUT->eError,
				     psConnection);

 RGXInitFirmware_exit:
	if (ui32RGXFWAlignChecksInt)
		kfree(ui32RGXFWAlignChecksInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXInitDevPart2(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_RGXINITDEVPART2 *
			    psRGXInitDevPart2IN,
			    PVRSRV_BRIDGE_OUT_RGXINITDEVPART2 *
			    psRGXInitDevPart2OUT,
			    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	RGX_INIT_COMMAND *psInitScriptInt = IMG_NULL;
	RGX_INIT_COMMAND *psDbgScriptInt = IMG_NULL;
	RGX_INIT_COMMAND *psDeinitScriptInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXINIT_RGXINITDEVPART2);

	psInitScriptInt =
	    kmalloc(RGX_MAX_INIT_COMMANDS * sizeof(RGX_INIT_COMMAND),
		    GFP_KERNEL);
	if (!psInitScriptInt) {
		psRGXInitDevPart2OUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXInitDevPart2_exit;
	}

	if (copy_from_user(psInitScriptInt, psRGXInitDevPart2IN->psInitScript,
			   RGX_MAX_INIT_COMMANDS * sizeof(RGX_INIT_COMMAND)) !=
	    0) {
		psRGXInitDevPart2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXInitDevPart2_exit;
	}

	psDbgScriptInt =
	    kmalloc(RGX_MAX_INIT_COMMANDS * sizeof(RGX_INIT_COMMAND),
		    GFP_KERNEL);
	if (!psDbgScriptInt) {
		psRGXInitDevPart2OUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXInitDevPart2_exit;
	}

	if (copy_from_user(psDbgScriptInt, psRGXInitDevPart2IN->psDbgScript,
			   RGX_MAX_INIT_COMMANDS * sizeof(RGX_INIT_COMMAND)) !=
	    0) {
		psRGXInitDevPart2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXInitDevPart2_exit;
	}

	psDeinitScriptInt =
	    kmalloc(RGX_MAX_DEINIT_COMMANDS * sizeof(RGX_INIT_COMMAND),
		    GFP_KERNEL);
	if (!psDeinitScriptInt) {
		psRGXInitDevPart2OUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXInitDevPart2_exit;
	}

	if (copy_from_user
	    (psDeinitScriptInt, psRGXInitDevPart2IN->psDeinitScript,
	     RGX_MAX_DEINIT_COMMANDS * sizeof(RGX_INIT_COMMAND)) != 0) {
		psRGXInitDevPart2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXInitDevPart2_exit;
	}

	/* Look up the address from the handle */
	psRGXInitDevPart2OUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXInitDevPart2IN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXInitDevPart2OUT->eError != PVRSRV_OK) {
		goto RGXInitDevPart2_exit;
	}

	psRGXInitDevPart2OUT->eError =
	    PVRSRVRGXInitDevPart2KM(hDevNodeInt,
				    psInitScriptInt,
				    psDbgScriptInt,
				    psDeinitScriptInt,
				    psRGXInitDevPart2IN->ui32KernelCatBase);

 RGXInitDevPart2_exit:
	if (psInitScriptInt)
		kfree(psInitScriptInt);
	if (psDbgScriptInt)
		kfree(psDbgScriptInt);
	if (psDeinitScriptInt)
		kfree(psDeinitScriptInt);

	return 0;
}

PVRSRV_ERROR RegisterRGXINITFunctions(IMG_VOID);
IMG_VOID UnregisterRGXINITFunctions(IMG_VOID);

/*
 * Register all RGXINIT functions with services
 */
PVRSRV_ERROR RegisterRGXINITFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXINIT_RGXINITFIRMWARE,
			      PVRSRVBridgeRGXInitFirmware);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXINIT_RGXINITDEVPART2,
			      PVRSRVBridgeRGXInitDevPart2);

	return PVRSRV_OK;
}

/*
 * Unregister all rgxinit functions with services
 */
IMG_VOID UnregisterRGXINITFunctions(IMG_VOID)
{
}
