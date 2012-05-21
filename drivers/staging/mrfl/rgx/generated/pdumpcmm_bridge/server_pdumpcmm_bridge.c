									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for pdumpcmm
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for pdumpcmm
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "devicemem_server.h"

#include "common_pdumpcmm_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static IMG_INT
PVRSRVBridgeDevmemPDumpBitmap(IMG_UINT32 ui32BridgeID,
			      PVRSRV_BRIDGE_IN_DEVMEMPDUMPBITMAP *
			      psDevmemPDumpBitmapIN,
			      PVRSRV_BRIDGE_OUT_DEVMEMPDUMPBITMAP *
			      psDevmemPDumpBitmapOUT,
			      CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceNodeInt;
	IMG_CHAR *uiFileNameInt = IMG_NULL;
	DEVMEMINT_CTX *psDevmemCtxInt;
	IMG_HANDLE hDevmemCtxInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMPCMM_DEVMEMPDUMPBITMAP);

	uiFileNameInt =
	    kmalloc(PVRSRV_PDUMP_MAX_FILENAME_SIZE * sizeof(IMG_CHAR),
		    GFP_KERNEL);
	if (!uiFileNameInt) {
		psDevmemPDumpBitmapOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DevmemPDumpBitmap_exit;
	}

	if (copy_from_user(uiFileNameInt, psDevmemPDumpBitmapIN->puiFileName,
			   PVRSRV_PDUMP_MAX_FILENAME_SIZE * sizeof(IMG_CHAR)) !=
	    0) {
		psDevmemPDumpBitmapOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DevmemPDumpBitmap_exit;
	}

	/* Look up the address from the handle */
	psDevmemPDumpBitmapOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceNodeInt,
			       psDevmemPDumpBitmapIN->hDeviceNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psDevmemPDumpBitmapOUT->eError != PVRSRV_OK) {
		goto DevmemPDumpBitmap_exit;
	}
	/* Look up the address from the handle */
	psDevmemPDumpBitmapOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevmemCtxInt2,
			       psDevmemPDumpBitmapIN->hDevmemCtx,
			       PVRSRV_HANDLE_TYPE_DEVMEMINT_CTX);
	if (psDevmemPDumpBitmapOUT->eError != PVRSRV_OK) {
		goto DevmemPDumpBitmap_exit;
	}

	/* Look up the data from the resman address */
	psDevmemPDumpBitmapOUT->eError =
	    ResManFindPrivateDataByPtr(hDevmemCtxInt2,
				       (IMG_VOID **) & psDevmemCtxInt,
				       IMG_NULL);
	if (psDevmemPDumpBitmapOUT->eError != PVRSRV_OK) {
		goto DevmemPDumpBitmap_exit;
	}

	psDevmemPDumpBitmapOUT->eError =
	    DevmemIntPDumpBitmap(hDeviceNodeInt,
				 uiFileNameInt,
				 psDevmemPDumpBitmapIN->ui32FileOffset,
				 psDevmemPDumpBitmapIN->ui32Width,
				 psDevmemPDumpBitmapIN->ui32Height,
				 psDevmemPDumpBitmapIN->ui32StrideInBytes,
				 psDevmemPDumpBitmapIN->sDevBaseAddr,
				 psDevmemCtxInt,
				 psDevmemPDumpBitmapIN->ui32Size,
				 psDevmemPDumpBitmapIN->ePixelFormat,
				 psDevmemPDumpBitmapIN->eMemFormat,
				 psDevmemPDumpBitmapIN->ui32PDumpFlags);

 DevmemPDumpBitmap_exit:
	if (uiFileNameInt)
		kfree(uiFileNameInt);

	return 0;
}

PVRSRV_ERROR RegisterPDUMPCMMFunctions(IMG_VOID);
IMG_VOID UnregisterPDUMPCMMFunctions(IMG_VOID);

/*
 * Register all PDUMPCMM functions with services
 */
PVRSRV_ERROR RegisterPDUMPCMMFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMPCMM_DEVMEMPDUMPBITMAP,
			      PVRSRVBridgeDevmemPDumpBitmap);

	return PVRSRV_OK;
}

/*
 * Unregister all pdumpcmm functions with services
 */
IMG_VOID UnregisterPDUMPCMMFunctions(IMG_VOID)
{
}
