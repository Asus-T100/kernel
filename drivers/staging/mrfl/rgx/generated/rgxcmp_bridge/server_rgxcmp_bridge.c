/*************************************************************************/ /*!
@File
@Title          Server bridge for rgxcmp
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for rgxcmp
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "rgxcompute.h"


#include "common_rgxcmp_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR
RGXDestroyComputeContextResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

	return eError;
}


static IMG_INT
PVRSRVBridgeRGXCreateComputeContext(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXCREATECOMPUTECONTEXT *psRGXCreateComputeContextIN,
					 PVRSRV_BRIDGE_OUT_RGXCREATECOMPUTECONTEXT *psRGXCreateComputeContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC * psCmpCCBMemDescInt;
	DEVMEM_MEMDESC * psCmpCCBCtlMemDescInt;
	RGX_CC_CLEANUP_DATA * psCleanupCookieInt;
	IMG_HANDLE hCleanupCookieInt2;
	DEVMEM_MEMDESC * psFWComputeContextInt;
	IMG_BYTE *psFrameworkCmdInt = IMG_NULL;
	IMG_HANDLE hPrivDataInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXCMP_RGXCREATECOMPUTECONTEXT);

	psFrameworkCmdInt = kmalloc(RGXFWIF_RF_CMD_SIZE * sizeof(IMG_BYTE), GFP_KERNEL);
	if (!psFrameworkCmdInt)
	{
		psRGXCreateComputeContextOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXCreateComputeContext_exit;
	}


	if (copy_from_user(psFrameworkCmdInt, psRGXCreateComputeContextIN->psFrameworkCmd,
		RGXFWIF_RF_CMD_SIZE * sizeof(IMG_BYTE)) != 0)
	{
		psRGXCreateComputeContextOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXCreateComputeContext_exit;
	}


	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateComputeContextOUT->eError, psConnection, 2);

	/* Look up the address from the handle */
	psRGXCreateComputeContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hDevNodeInt,
						   psRGXCreateComputeContextIN->hDevNode,
						   PVRSRV_HANDLE_TYPE_DEV_NODE);
	if(psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateComputeContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateComputeContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psCmpCCBMemDescInt,
						   psRGXCreateComputeContextIN->hCmpCCBMemDesc,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateComputeContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateComputeContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psCmpCCBCtlMemDescInt,
						   psRGXCreateComputeContextIN->hCmpCCBCtlMemDesc,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateComputeContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateComputeContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hPrivDataInt,
						   psRGXCreateComputeContextIN->hPrivData,
						   PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	if(psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateComputeContext_exit;
	}

	psRGXCreateComputeContextOUT->eError =
		PVRSRVRGXCreateComputeContextKM(
					hDevNodeInt,
					psCmpCCBMemDescInt,
					psCmpCCBCtlMemDescInt,
					&psCleanupCookieInt,
					&psFWComputeContextInt,
					psRGXCreateComputeContextIN->ui32Priority,
					psRGXCreateComputeContextIN->sMCUFenceAddr,
					psRGXCreateComputeContextIN->ui32FrameworkCmdize,
					psFrameworkCmdInt,
					hPrivDataInt);
	/* Exit early if bridged call fails */
	if(psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateComputeContext_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hCleanupCookieInt2 = ResManRegisterRes(psConnection->hResManContext,
												RESMAN_TYPE_RGX_COMPUTE_CONTEXT,
												psCleanupCookieInt,
												/* FIXME: how can we avoid this cast? */
												(RESMAN_FREE_FN)&PVRSRVRGXDestroyComputeContextKM);
	if (hCleanupCookieInt2 == IMG_NULL)
	{
		psRGXCreateComputeContextOUT->eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateComputeContext_exit;
	}
	psRGXCreateComputeContextOUT->eError = PVRSRVAllocHandle(psConnection->psHandleBase,
					  &psRGXCreateComputeContextOUT->hCleanupCookie,
					  (IMG_HANDLE) hCleanupCookieInt2,
					  PVRSRV_HANDLE_TYPE_RGX_CC_CLEANUP,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  );
	if (psRGXCreateComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateComputeContext_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
					  &psRGXCreateComputeContextOUT->hFWComputeContext,
					  (IMG_HANDLE) psFWComputeContextInt,
					  PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  ,psRGXCreateComputeContextOUT->hCleanupCookie);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateComputeContextOUT->eError, psConnection);



RGXCreateComputeContext_exit:
	if (psFrameworkCmdInt)
		kfree(psFrameworkCmdInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyComputeContext(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXDESTROYCOMPUTECONTEXT *psRGXDestroyComputeContextIN,
					 PVRSRV_BRIDGE_OUT_RGXDESTROYCOMPUTECONTEXT *psRGXDestroyComputeContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hCleanupCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXCMP_RGXDESTROYCOMPUTECONTEXT);


	/* Look up the address from the handle */
	psRGXDestroyComputeContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hCleanupCookieInt2,
						   psRGXDestroyComputeContextIN->hCleanupCookie,
						   PVRSRV_HANDLE_TYPE_RGX_CC_CLEANUP);
	if(psRGXDestroyComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXDestroyComputeContext_exit;
	}

	psRGXDestroyComputeContextOUT->eError = RGXDestroyComputeContextResManProxy(hCleanupCookieInt2);
	/* Exit early if bridged call fails */
	if(psRGXDestroyComputeContextOUT->eError != PVRSRV_OK)
	{
		goto RGXDestroyComputeContext_exit;
	}

	psRGXDestroyComputeContextOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psRGXDestroyComputeContextIN->hCleanupCookie,
					PVRSRV_HANDLE_TYPE_RGX_CC_CLEANUP);


RGXDestroyComputeContext_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXKickCDM(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXKICKCDM *psRGXKickCDMIN,
					 PVRSRV_BRIDGE_OUT_RGXKICKCDM *psRGXKickCDMOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC * psFWComputeContextInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM);


	/* Look up the address from the handle */
	psRGXKickCDMOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hDevNodeInt,
						   psRGXKickCDMIN->hDevNode,
						   PVRSRV_HANDLE_TYPE_DEV_NODE);
	if(psRGXKickCDMOUT->eError != PVRSRV_OK)
	{
		goto RGXKickCDM_exit;
	}
	/* Look up the address from the handle */
	psRGXKickCDMOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psFWComputeContextInt,
						   psRGXKickCDMIN->hFWComputeContext,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXKickCDMOUT->eError != PVRSRV_OK)
	{
		goto RGXKickCDM_exit;
	}

	psRGXKickCDMOUT->eError =
		PVRSRVRGXKickCDMKM(
					hDevNodeInt,
					psFWComputeContextInt,
					psRGXKickCDMIN->ui32cCCBWoffUpdate,
					psRGXKickCDMIN->bbPDumpContinuous);



RGXKickCDM_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXFlushComputeData(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXFLUSHCOMPUTEDATA *psRGXFlushComputeDataIN,
					 PVRSRV_BRIDGE_OUT_RGXFLUSHCOMPUTEDATA *psRGXFlushComputeDataOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC * psFWComputeContextInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXCMP_RGXFLUSHCOMPUTEDATA);


	/* Look up the address from the handle */
	psRGXFlushComputeDataOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hDevNodeInt,
						   psRGXFlushComputeDataIN->hDevNode,
						   PVRSRV_HANDLE_TYPE_DEV_NODE);
	if(psRGXFlushComputeDataOUT->eError != PVRSRV_OK)
	{
		goto RGXFlushComputeData_exit;
	}
	/* Look up the address from the handle */
	psRGXFlushComputeDataOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psFWComputeContextInt,
						   psRGXFlushComputeDataIN->hFWComputeContext,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXFlushComputeDataOUT->eError != PVRSRV_OK)
	{
		goto RGXFlushComputeData_exit;
	}

	psRGXFlushComputeDataOUT->eError =
		PVRSRVRGXFlushComputeDataKM(
					hDevNodeInt,
					psFWComputeContextInt);



RGXFlushComputeData_exit:

	return 0;
}


PVRSRV_ERROR RegisterRGXCMPFunctions(IMG_VOID);
IMG_VOID UnregisterRGXCMPFunctions(IMG_VOID);

/*
 * Register all RGXCMP functions with services
 */
PVRSRV_ERROR RegisterRGXCMPFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP_RGXCREATECOMPUTECONTEXT, PVRSRVBridgeRGXCreateComputeContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP_RGXDESTROYCOMPUTECONTEXT, PVRSRVBridgeRGXDestroyComputeContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM, PVRSRVBridgeRGXKickCDM);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXCMP_RGXFLUSHCOMPUTEDATA, PVRSRVBridgeRGXFlushComputeData);

	return PVRSRV_OK;
}

/*
 * Unregister all rgxcmp functions with services
 */
IMG_VOID UnregisterRGXCMPFunctions(IMG_VOID)
{
}
