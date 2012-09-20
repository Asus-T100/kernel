									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for hostportio
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for hostportio
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "hostportio_server.h"

#include "common_hostportio_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static IMG_INT
PVRSRVBridgeHostPortRead(IMG_UINT32 ui32BridgeID,
			 PVRSRV_BRIDGE_IN_HOSTPORTREAD * psHostPortReadIN,
			 PVRSRV_BRIDGE_OUT_HOSTPORTREAD * psHostPortReadOUT,
			 CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	IMG_HANDLE hMemCtxPrivDataInt;
	IMG_CHAR *puiDstBufferInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_HOSTPORTIO_HOSTPORTREAD);

	puiDstBufferInt =
	    kmalloc(psHostPortReadIN->uiDstBufLen * sizeof(IMG_CHAR),
		    GFP_KERNEL);
	if (!puiDstBufferInt) {
		psHostPortReadOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto HostPortRead_exit;
	}

	/* Look up the address from the handle */
	psHostPortReadOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psHostPortReadIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psHostPortReadOUT->eError != PVRSRV_OK) {
		goto HostPortRead_exit;
	}
	/* Look up the address from the handle */
	psHostPortReadOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hMemCtxPrivDataInt,
			       psHostPortReadIN->hMemCtxPrivData,
			       PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	if (psHostPortReadOUT->eError != PVRSRV_OK) {
		goto HostPortRead_exit;
	}

	psHostPortReadOUT->eError =
	    PVRSRVHostPortReadKM(hDevNodeInt,
				 hMemCtxPrivDataInt,
				 psHostPortReadIN->ui32CRHostIFVal,
				 psHostPortReadIN->ui32ReadOffset,
				 psHostPortReadIN->uiDstBufLen,
				 puiDstBufferInt,
				 &psHostPortReadOUT->uiNumBytesRead);

	if (copy_to_user(psHostPortReadOUT->puiDstBuffer, puiDstBufferInt,
			 (psHostPortReadIN->uiDstBufLen * sizeof(IMG_CHAR))) !=
	    0) {
		psHostPortReadOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto HostPortRead_exit;
	}

 HostPortRead_exit:
	if (puiDstBufferInt)
		kfree(puiDstBufferInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeHostPortWrite(IMG_UINT32 ui32BridgeID,
			  PVRSRV_BRIDGE_IN_HOSTPORTWRITE * psHostPortWriteIN,
			  PVRSRV_BRIDGE_OUT_HOSTPORTWRITE * psHostPortWriteOUT,
			  CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	IMG_HANDLE hMemCtxPrivDataInt;
	IMG_CHAR *uiSrcBufferInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_HOSTPORTIO_HOSTPORTWRITE);

	uiSrcBufferInt =
	    kmalloc(psHostPortWriteIN->uiSrcBufLen * sizeof(IMG_CHAR),
		    GFP_KERNEL);
	if (!uiSrcBufferInt) {
		psHostPortWriteOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto HostPortWrite_exit;
	}

	if (copy_from_user(uiSrcBufferInt, psHostPortWriteIN->puiSrcBuffer,
			   psHostPortWriteIN->uiSrcBufLen * sizeof(IMG_CHAR)) !=
	    0) {
		psHostPortWriteOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto HostPortWrite_exit;
	}

	/* Look up the address from the handle */
	psHostPortWriteOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psHostPortWriteIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psHostPortWriteOUT->eError != PVRSRV_OK) {
		goto HostPortWrite_exit;
	}
	/* Look up the address from the handle */
	psHostPortWriteOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hMemCtxPrivDataInt,
			       psHostPortWriteIN->hMemCtxPrivData,
			       PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	if (psHostPortWriteOUT->eError != PVRSRV_OK) {
		goto HostPortWrite_exit;
	}

	psHostPortWriteOUT->eError =
	    PVRSRVHostPortWriteKM(hDevNodeInt,
				  hMemCtxPrivDataInt,
				  psHostPortWriteIN->ui32CRHostIFVal,
				  psHostPortWriteIN->ui32WriteOffset,
				  psHostPortWriteIN->uiSrcBufLen,
				  uiSrcBufferInt,
				  &psHostPortWriteOUT->uiNumBytesWritten);

 HostPortWrite_exit:
	if (uiSrcBufferInt)
		kfree(uiSrcBufferInt);

	return 0;
}

PVRSRV_ERROR RegisterHOSTPORTIOFunctions(IMG_VOID);
IMG_VOID UnregisterHOSTPORTIOFunctions(IMG_VOID);

/*
 * Register all HOSTPORTIO functions with services
 */
PVRSRV_ERROR RegisterHOSTPORTIOFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_HOSTPORTIO_HOSTPORTREAD,
			      PVRSRVBridgeHostPortRead);
	SetDispatchTableEntry(PVRSRV_BRIDGE_HOSTPORTIO_HOSTPORTWRITE,
			      PVRSRVBridgeHostPortWrite);

	return PVRSRV_OK;
}

/*
 * Unregister all hostportio functions with services
 */
IMG_VOID UnregisterHOSTPORTIOFunctions(IMG_VOID)
{
}
