/*************************************************************************/ /*!
@File
@Title          Server bridge for rgxtq
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for rgxtq
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

#include "rgxtransfer.h"


#include "common_rgxtq_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR
RGXDestroyTQ3DContextResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

	return eError;
}

static PVRSRV_ERROR
RGXDestroyTQ2DContextResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

	return eError;
}


static IMG_INT
PVRSRVBridgeRGXCreateTQ3DContext(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXCREATETQ3DCONTEXT *psRGXCreateTQ3DContextIN,
					 PVRSRV_BRIDGE_OUT_RGXCREATETQ3DCONTEXT *psRGXCreateTQ3DContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC * psTQ3DCCBMemDescInt;
	DEVMEM_MEMDESC * psTQ3DCCBCtlMemDescInt;
	RGX_TQ3D_CLEANUP_DATA * psCleanupCookieInt;
	IMG_HANDLE hCleanupCookieInt2;
	DEVMEM_MEMDESC * psFWTQ3DContextInt;
	DEVMEM_MEMDESC * psFWTQ3DContextStateInt;
	IMG_BYTE *psFrameworkCmdInt = IMG_NULL;
	IMG_HANDLE hPrivDataInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXTQ_RGXCREATETQ3DCONTEXT);

	psFrameworkCmdInt = kmalloc(RGXFWIF_RF_CMD_SIZE * sizeof(IMG_BYTE), GFP_KERNEL);
	if (!psFrameworkCmdInt)
	{
		psRGXCreateTQ3DContextOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXCreateTQ3DContext_exit;
	}


	if (copy_from_user(psFrameworkCmdInt, psRGXCreateTQ3DContextIN->psFrameworkCmd,
		RGXFWIF_RF_CMD_SIZE * sizeof(IMG_BYTE)) != 0)
	{
		psRGXCreateTQ3DContextOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXCreateTQ3DContext_exit;
	}


	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateTQ3DContextOUT->eError, psConnection, 3);

	/* Look up the address from the handle */
	psRGXCreateTQ3DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hDevNodeInt,
						   psRGXCreateTQ3DContextIN->hDevNode,
						   PVRSRV_HANDLE_TYPE_DEV_NODE);
	if(psRGXCreateTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ3DContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateTQ3DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psTQ3DCCBMemDescInt,
						   psRGXCreateTQ3DContextIN->hTQ3DCCBMemDesc,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXCreateTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ3DContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateTQ3DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psTQ3DCCBCtlMemDescInt,
						   psRGXCreateTQ3DContextIN->hTQ3DCCBCtlMemDesc,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXCreateTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ3DContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateTQ3DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hPrivDataInt,
						   psRGXCreateTQ3DContextIN->hPrivData,
						   PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	if(psRGXCreateTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ3DContext_exit;
	}

	psRGXCreateTQ3DContextOUT->eError =
		PVRSRVRGXCreateTQ3DContextKM(
					hDevNodeInt,
					psTQ3DCCBMemDescInt,
					psTQ3DCCBCtlMemDescInt,
					&psCleanupCookieInt,
					&psFWTQ3DContextInt,
					&psFWTQ3DContextStateInt,
					psRGXCreateTQ3DContextIN->ui32Priority,
					psRGXCreateTQ3DContextIN->sMCUFenceAddr,
					psRGXCreateTQ3DContextIN->ui32FrameworkCmdize,
					psFrameworkCmdInt,
					hPrivDataInt);
	/* Exit early if bridged call fails */
	if(psRGXCreateTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ3DContext_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hCleanupCookieInt2 = ResManRegisterRes(psConnection->hResManContext,
												RESMAN_TYPE_RGX_TQ3D_CONTEXT,
												psCleanupCookieInt,
												/* FIXME: how can we avoid this cast? */
												(RESMAN_FREE_FN)&PVRSRVRGXDestroyTQ3DContextKM);
	if (hCleanupCookieInt2 == IMG_NULL)
	{
		psRGXCreateTQ3DContextOUT->eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateTQ3DContext_exit;
	}
	psRGXCreateTQ3DContextOUT->eError = PVRSRVAllocHandle(psConnection->psHandleBase,
					  &psRGXCreateTQ3DContextOUT->hCleanupCookie,
					  (IMG_HANDLE) hCleanupCookieInt2,
					  PVRSRV_HANDLE_TYPE_RGX_TQ3D_CLEANUP,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  );
	if (psRGXCreateTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ3DContext_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
					  &psRGXCreateTQ3DContextOUT->hFWTQ3DContext,
					  (IMG_HANDLE) psFWTQ3DContextInt,
					  PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  ,psRGXCreateTQ3DContextOUT->hCleanupCookie);
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
					  &psRGXCreateTQ3DContextOUT->hFWTQ3DContextState,
					  (IMG_HANDLE) psFWTQ3DContextStateInt,
					  PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  ,psRGXCreateTQ3DContextOUT->hCleanupCookie);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateTQ3DContextOUT->eError, psConnection);



RGXCreateTQ3DContext_exit:
	if (psFrameworkCmdInt)
		kfree(psFrameworkCmdInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyTQ3DContext(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXDESTROYTQ3DCONTEXT *psRGXDestroyTQ3DContextIN,
					 PVRSRV_BRIDGE_OUT_RGXDESTROYTQ3DCONTEXT *psRGXDestroyTQ3DContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hCleanupCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXTQ_RGXDESTROYTQ3DCONTEXT);


	/* Look up the address from the handle */
	psRGXDestroyTQ3DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hCleanupCookieInt2,
						   psRGXDestroyTQ3DContextIN->hCleanupCookie,
						   PVRSRV_HANDLE_TYPE_RGX_TQ3D_CLEANUP);
	if(psRGXDestroyTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXDestroyTQ3DContext_exit;
	}

	psRGXDestroyTQ3DContextOUT->eError = RGXDestroyTQ3DContextResManProxy(hCleanupCookieInt2);
	/* Exit early if bridged call fails */
	if(psRGXDestroyTQ3DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXDestroyTQ3DContext_exit;
	}

	psRGXDestroyTQ3DContextOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psRGXDestroyTQ3DContextIN->hCleanupCookie,
					PVRSRV_HANDLE_TYPE_RGX_TQ3D_CLEANUP);


RGXDestroyTQ3DContext_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSubmitTQ3DKick(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_SUBMITTQ3DKICK *psSubmitTQ3DKickIN,
					 PVRSRV_BRIDGE_OUT_SUBMITTQ3DKICK *psSubmitTQ3DKickOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC * psFWTQ3DContextInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXTQ_SUBMITTQ3DKICK);


	/* Look up the address from the handle */
	psSubmitTQ3DKickOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hDevNodeInt,
						   psSubmitTQ3DKickIN->hDevNode,
						   PVRSRV_HANDLE_TYPE_DEV_NODE);
	if(psSubmitTQ3DKickOUT->eError != PVRSRV_OK)
	{
		goto SubmitTQ3DKick_exit;
	}
	/* Look up the address from the handle */
	psSubmitTQ3DKickOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psFWTQ3DContextInt,
						   psSubmitTQ3DKickIN->hFWTQ3DContext,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psSubmitTQ3DKickOUT->eError != PVRSRV_OK)
	{
		goto SubmitTQ3DKick_exit;
	}

	psSubmitTQ3DKickOUT->eError =
		PVRSRVSubmitTQ3DKickKM(
					hDevNodeInt,
					psFWTQ3DContextInt,
					psSubmitTQ3DKickIN->ui32ui32TQ3DcCCBWoffUpdate,
					psSubmitTQ3DKickIN->bbPDumpContinuous);



SubmitTQ3DKick_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXCreateTQ2DContext(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXCREATETQ2DCONTEXT *psRGXCreateTQ2DContextIN,
					 PVRSRV_BRIDGE_OUT_RGXCREATETQ2DCONTEXT *psRGXCreateTQ2DContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC * psTQ2DCCBMemDescInt;
	DEVMEM_MEMDESC * psTQ2DCCBCtlMemDescInt;
	RGX_TQ2D_CLEANUP_DATA * psCleanupCookieInt;
	IMG_HANDLE hCleanupCookieInt2;
	DEVMEM_MEMDESC * psFWTQ2DContextInt;
	IMG_BYTE *psFrameworkCmdInt = IMG_NULL;
	IMG_HANDLE hPrivDataInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXTQ_RGXCREATETQ2DCONTEXT);

	psFrameworkCmdInt = kmalloc(RGXFWIF_RF_CMD_SIZE * sizeof(IMG_BYTE), GFP_KERNEL);
	if (!psFrameworkCmdInt)
	{
		psRGXCreateTQ2DContextOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXCreateTQ2DContext_exit;
	}


	if (copy_from_user(psFrameworkCmdInt, psRGXCreateTQ2DContextIN->psFrameworkCmd,
		RGXFWIF_RF_CMD_SIZE * sizeof(IMG_BYTE)) != 0)
	{
		psRGXCreateTQ2DContextOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXCreateTQ2DContext_exit;
	}


	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateTQ2DContextOUT->eError, psConnection, 2);

	/* Look up the address from the handle */
	psRGXCreateTQ2DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hDevNodeInt,
						   psRGXCreateTQ2DContextIN->hDevNode,
						   PVRSRV_HANDLE_TYPE_DEV_NODE);
	if(psRGXCreateTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ2DContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateTQ2DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psTQ2DCCBMemDescInt,
						   psRGXCreateTQ2DContextIN->hTQ2DCCBMemDesc,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXCreateTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ2DContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateTQ2DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psTQ2DCCBCtlMemDescInt,
						   psRGXCreateTQ2DContextIN->hTQ2DCCBCtlMemDesc,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psRGXCreateTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ2DContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateTQ2DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hPrivDataInt,
						   psRGXCreateTQ2DContextIN->hPrivData,
						   PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	if(psRGXCreateTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ2DContext_exit;
	}

	psRGXCreateTQ2DContextOUT->eError =
		PVRSRVRGXCreateTQ2DContextKM(
					hDevNodeInt,
					psTQ2DCCBMemDescInt,
					psTQ2DCCBCtlMemDescInt,
					&psCleanupCookieInt,
					&psFWTQ2DContextInt,
					psRGXCreateTQ2DContextIN->ui32Priority,
					psRGXCreateTQ2DContextIN->ui32FrameworkCmdize,
					psFrameworkCmdInt,
					hPrivDataInt);
	/* Exit early if bridged call fails */
	if(psRGXCreateTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ2DContext_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hCleanupCookieInt2 = ResManRegisterRes(psConnection->hResManContext,
												RESMAN_TYPE_RGX_TQ2D_CONTEXT,
												psCleanupCookieInt,
												/* FIXME: how can we avoid this cast? */
												(RESMAN_FREE_FN)&PVRSRVRGXDestroyTQ2DContextKM);
	if (hCleanupCookieInt2 == IMG_NULL)
	{
		psRGXCreateTQ2DContextOUT->eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateTQ2DContext_exit;
	}
	psRGXCreateTQ2DContextOUT->eError = PVRSRVAllocHandle(psConnection->psHandleBase,
					  &psRGXCreateTQ2DContextOUT->hCleanupCookie,
					  (IMG_HANDLE) hCleanupCookieInt2,
					  PVRSRV_HANDLE_TYPE_RGX_TQ2D_CLEANUP,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  );
	if (psRGXCreateTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXCreateTQ2DContext_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
					  &psRGXCreateTQ2DContextOUT->hFWTQ2DContext,
					  (IMG_HANDLE) psFWTQ2DContextInt,
					  PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
					  PVRSRV_HANDLE_ALLOC_FLAG_NONE
					  ,psRGXCreateTQ2DContextOUT->hCleanupCookie);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateTQ2DContextOUT->eError, psConnection);



RGXCreateTQ2DContext_exit:
	if (psFrameworkCmdInt)
		kfree(psFrameworkCmdInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyTQ2DContext(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RGXDESTROYTQ2DCONTEXT *psRGXDestroyTQ2DContextIN,
					 PVRSRV_BRIDGE_OUT_RGXDESTROYTQ2DCONTEXT *psRGXDestroyTQ2DContextOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hCleanupCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXTQ_RGXDESTROYTQ2DCONTEXT);


	/* Look up the address from the handle */
	psRGXDestroyTQ2DContextOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hCleanupCookieInt2,
						   psRGXDestroyTQ2DContextIN->hCleanupCookie,
						   PVRSRV_HANDLE_TYPE_RGX_TQ2D_CLEANUP);
	if(psRGXDestroyTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXDestroyTQ2DContext_exit;
	}

	psRGXDestroyTQ2DContextOUT->eError = RGXDestroyTQ2DContextResManProxy(hCleanupCookieInt2);
	/* Exit early if bridged call fails */
	if(psRGXDestroyTQ2DContextOUT->eError != PVRSRV_OK)
	{
		goto RGXDestroyTQ2DContext_exit;
	}

	psRGXDestroyTQ2DContextOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psRGXDestroyTQ2DContextIN->hCleanupCookie,
					PVRSRV_HANDLE_TYPE_RGX_TQ2D_CLEANUP);


RGXDestroyTQ2DContext_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeSubmitTQ2DKick(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_SUBMITTQ2DKICK *psSubmitTQ2DKickIN,
					 PVRSRV_BRIDGE_OUT_SUBMITTQ2DKICK *psSubmitTQ2DKickOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC * psFWTQ2DContextInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_RGXTQ_SUBMITTQ2DKICK);


	/* Look up the address from the handle */
	psSubmitTQ2DKickOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hDevNodeInt,
						   psSubmitTQ2DKickIN->hDevNode,
						   PVRSRV_HANDLE_TYPE_DEV_NODE);
	if(psSubmitTQ2DKickOUT->eError != PVRSRV_OK)
	{
		goto SubmitTQ2DKick_exit;
	}
	/* Look up the address from the handle */
	psSubmitTQ2DKickOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &psFWTQ2DContextInt,
						   psSubmitTQ2DKickIN->hFWTQ2DContext,
						   PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if(psSubmitTQ2DKickOUT->eError != PVRSRV_OK)
	{
		goto SubmitTQ2DKick_exit;
	}

	psSubmitTQ2DKickOUT->eError =
		PVRSRVSubmitTQ2DKickKM(
					hDevNodeInt,
					psFWTQ2DContextInt,
					psSubmitTQ2DKickIN->ui32ui32TQ2DcCCBWoffUpdate,
					psSubmitTQ2DKickIN->bbPDumpContinuous);



SubmitTQ2DKick_exit:

	return 0;
}


PVRSRV_ERROR RegisterRGXTQFunctions(IMG_VOID);
IMG_VOID UnregisterRGXTQFunctions(IMG_VOID);

/*
 * Register all RGXTQ functions with services
 */
PVRSRV_ERROR RegisterRGXTQFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ_RGXCREATETQ3DCONTEXT, PVRSRVBridgeRGXCreateTQ3DContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ_RGXDESTROYTQ3DCONTEXT, PVRSRVBridgeRGXDestroyTQ3DContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ_SUBMITTQ3DKICK, PVRSRVBridgeSubmitTQ3DKick);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ_RGXCREATETQ2DCONTEXT, PVRSRVBridgeRGXCreateTQ2DContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ_RGXDESTROYTQ2DCONTEXT, PVRSRVBridgeRGXDestroyTQ2DContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTQ_SUBMITTQ2DKICK, PVRSRVBridgeSubmitTQ2DKick);

	return PVRSRV_OK;
}

/*
 * Unregister all rgxtq functions with services
 */
IMG_VOID UnregisterRGXTQFunctions(IMG_VOID)
{
}
