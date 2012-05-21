									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for rgxpdump
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for rgxpdump
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "rgxpdump.h"

#include "common_rgxpdump_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static IMG_INT
PVRSRVBridgePDumpTraceBuffer(IMG_UINT32 ui32BridgeID,
			     PVRSRV_BRIDGE_IN_PDUMPTRACEBUFFER *
			     psPDumpTraceBufferIN,
			     PVRSRV_BRIDGE_OUT_PDUMPTRACEBUFFER *
			     psPDumpTraceBufferOUT,
			     CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXPDUMP_PDUMPTRACEBUFFER);

	/* Look up the address from the handle */
	psPDumpTraceBufferOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psPDumpTraceBufferIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psPDumpTraceBufferOUT->eError != PVRSRV_OK) {
		goto PDumpTraceBuffer_exit;
	}

	psPDumpTraceBufferOUT->eError =
	    PVRSRVPDumpTraceBufferKM(hDeviceNodeInt,
				     psPDumpTraceBufferIN->ui32PDumpFlags);

 PDumpTraceBuffer_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgePDumpSignatureBuffer(IMG_UINT32 ui32BridgeID,
				 PVRSRV_BRIDGE_IN_PDUMPSIGNATUREBUFFER *
				 psPDumpSignatureBufferIN,
				 PVRSRV_BRIDGE_OUT_PDUMPSIGNATUREBUFFER *
				 psPDumpSignatureBufferOUT,
				 CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXPDUMP_PDUMPSIGNATUREBUFFER);

	/* Look up the address from the handle */
	psPDumpSignatureBufferOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psPDumpSignatureBufferIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psPDumpSignatureBufferOUT->eError != PVRSRV_OK) {
		goto PDumpSignatureBuffer_exit;
	}

	psPDumpSignatureBufferOUT->eError =
	    PVRSRVPDumpSignatureBufferKM(hDeviceNodeInt,
					 psPDumpSignatureBufferIN->
					 ui32PDumpFlags);

 PDumpSignatureBuffer_exit:

	return 0;
}

PVRSRV_ERROR RegisterRGXPDUMPFunctions(IMG_VOID);
IMG_VOID UnregisterRGXPDUMPFunctions(IMG_VOID);

/*
 * Register all RGXPDUMP functions with services
 */
PVRSRV_ERROR RegisterRGXPDUMPFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXPDUMP_PDUMPTRACEBUFFER,
			      PVRSRVBridgePDumpTraceBuffer);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXPDUMP_PDUMPSIGNATUREBUFFER,
			      PVRSRVBridgePDumpSignatureBuffer);

	return PVRSRV_OK;
}

/*
 * Unregister all rgxpdump functions with services
 */
IMG_VOID UnregisterRGXPDUMPFunctions(IMG_VOID)
{
}
