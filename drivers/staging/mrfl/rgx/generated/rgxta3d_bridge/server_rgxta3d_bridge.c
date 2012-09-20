									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for rgxta3d
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for rgxta3d
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "rgxta3d.h"

#include "common_rgxta3d_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR RGXDestroyHWRTDataResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR RGXDestroyRenderTargetResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR RGXDestroyFreeListResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR RGXDestroyRenderContextResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static IMG_INT
PVRSRVBridgeRGXCreateHWRTData(IMG_UINT32 ui32BridgeID,
			      PVRSRV_BRIDGE_IN_RGXCREATEHWRTDATA *
			      psRGXCreateHWRTDataIN,
			      PVRSRV_BRIDGE_OUT_RGXCREATEHWRTDATA *
			      psRGXCreateHWRTDataOUT,
			      CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	IMG_UINT32 *ui32apsFreeListsInt = IMG_NULL;
	RGX_RTDATA_CLEANUP_DATA *psCleanupCookieInt;
	IMG_HANDLE hCleanupCookieInt2;
	DEVMEM_MEMDESC *pssHWRTDataMemDescInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXCREATEHWRTDATA);

	ui32apsFreeListsInt =
	    kmalloc(RGX_NUM_FREELIST_TYPES * sizeof(IMG_UINT32), GFP_KERNEL);
	if (!ui32apsFreeListsInt) {
		psRGXCreateHWRTDataOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto RGXCreateHWRTData_exit;
	}

	if (copy_from_user
	    (ui32apsFreeListsInt, psRGXCreateHWRTDataIN->pui32apsFreeLists,
	     RGX_NUM_FREELIST_TYPES * sizeof(IMG_UINT32)) != 0) {
		psRGXCreateHWRTDataOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto RGXCreateHWRTData_exit;
	}

	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateHWRTDataOUT->eError, psConnection,
				  2)

	    /* Look up the address from the handle */
	    psRGXCreateHWRTDataOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXCreateHWRTDataIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXCreateHWRTDataOUT->eError != PVRSRV_OK) {
		goto RGXCreateHWRTData_exit;
	}

	psRGXCreateHWRTDataOUT->eError =
	    RGXCreateHWRTData(hDevNodeInt,
			      psRGXCreateHWRTDataIN->ui32RenderTarget,
			      psRGXCreateHWRTDataIN->sPMMlistDevVAddr,
			      ui32apsFreeListsInt,
			      &psCleanupCookieInt,
			      &pssHWRTDataMemDescInt,
			      &psRGXCreateHWRTDataOUT->ui32FWHWRTData);
	/* Exit early if bridged call fails */
	if (psRGXCreateHWRTDataOUT->eError != PVRSRV_OK) {
		goto RGXCreateHWRTData_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hCleanupCookieInt2 = ResManRegisterRes(psConnection->hResManContext,
					       RESMAN_TYPE_RGX_FWIF_HWRTDATA,
					       psCleanupCookieInt, 0,
					       /* FIXME: how can we avoid this cast? */
					       (RESMAN_FREE_FN) &
					       RGXDestroyHWRTData);
	if (hCleanupCookieInt2 == IMG_NULL) {
		psRGXCreateHWRTDataOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateHWRTData_exit;
	}
	psRGXCreateHWRTDataOUT->eError =
	    PVRSRVAllocHandle(psConnection->psHandleBase,
			      &psRGXCreateHWRTDataOUT->hCleanupCookie,
			      (IMG_HANDLE) hCleanupCookieInt2,
			      PVRSRV_HANDLE_TYPE_RGX_RTDATA_CLEANUP,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psRGXCreateHWRTDataOUT->eError != PVRSRV_OK) {
		goto RGXCreateHWRTData_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psRGXCreateHWRTDataOUT->hsHWRTDataMemDesc,
			       (IMG_HANDLE) pssHWRTDataMemDescInt,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psRGXCreateHWRTDataOUT->hCleanupCookie);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateHWRTDataOUT->eError,
				     psConnection);

 RGXCreateHWRTData_exit:
	if (ui32apsFreeListsInt)
		kfree(ui32apsFreeListsInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyHWRTData(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_RGXDESTROYHWRTDATA *
			       psRGXDestroyHWRTDataIN,
			       PVRSRV_BRIDGE_OUT_RGXDESTROYHWRTDATA *
			       psRGXDestroyHWRTDataOUT,
			       CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hCleanupCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYHWRTDATA);

	/* Look up the address from the handle */
	psRGXDestroyHWRTDataOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hCleanupCookieInt2,
			       psRGXDestroyHWRTDataIN->hCleanupCookie,
			       PVRSRV_HANDLE_TYPE_RGX_RTDATA_CLEANUP);
	if (psRGXDestroyHWRTDataOUT->eError != PVRSRV_OK) {
		goto RGXDestroyHWRTData_exit;
	}

	psRGXDestroyHWRTDataOUT->eError =
	    RGXDestroyHWRTDataResManProxy(hCleanupCookieInt2);
	/* Exit early if bridged call fails */
	if (psRGXDestroyHWRTDataOUT->eError != PVRSRV_OK) {
		goto RGXDestroyHWRTData_exit;
	}

	psRGXDestroyHWRTDataOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psRGXDestroyHWRTDataIN->
				hCleanupCookie,
				PVRSRV_HANDLE_TYPE_RGX_RTDATA_CLEANUP);

 RGXDestroyHWRTData_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXCreateRenderTarget(IMG_UINT32 ui32BridgeID,
				  PVRSRV_BRIDGE_IN_RGXCREATERENDERTARGET *
				  psRGXCreateRenderTargetIN,
				  PVRSRV_BRIDGE_OUT_RGXCREATERENDERTARGET *
				  psRGXCreateRenderTargetOUT,
				  CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC *pssRenderTargetMemDescInt;
	IMG_HANDLE hsRenderTargetMemDescInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXCREATERENDERTARGET);

	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateRenderTargetOUT->eError,
				  psConnection, 1)

	    /* Look up the address from the handle */
	    psRGXCreateRenderTargetOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXCreateRenderTargetIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXCreateRenderTargetOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderTarget_exit;
	}

	psRGXCreateRenderTargetOUT->eError =
	    RGXCreateRenderTarget(hDevNodeInt,
				  psRGXCreateRenderTargetIN->
				  spsVHeapTableDevVAddr,
				  &pssRenderTargetMemDescInt,
				  &psRGXCreateRenderTargetOUT->
				  ui32sRenderTargetFWDevVAddr);
	/* Exit early if bridged call fails */
	if (psRGXCreateRenderTargetOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderTarget_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hsRenderTargetMemDescInt2 =
	    ResManRegisterRes(psConnection->hResManContext,
			      RESMAN_TYPE_RGX_FWIF_RENDERTARGET,
			      pssRenderTargetMemDescInt, 0,
			      /* FIXME: how can we avoid this cast? */
			      (RESMAN_FREE_FN) & RGXDestroyRenderTarget);
	if (hsRenderTargetMemDescInt2 == IMG_NULL) {
		psRGXCreateRenderTargetOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateRenderTarget_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psRGXCreateRenderTargetOUT->hsRenderTargetMemDesc,
			    (IMG_HANDLE) hsRenderTargetMemDescInt2,
			    PVRSRV_HANDLE_TYPE_RGX_FWIF_RENDERTARGET,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateRenderTargetOUT->eError,
				     psConnection);

 RGXCreateRenderTarget_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyRenderTarget(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_RGXDESTROYRENDERTARGET *
				   psRGXDestroyRenderTargetIN,
				   PVRSRV_BRIDGE_OUT_RGXDESTROYRENDERTARGET *
				   psRGXDestroyRenderTargetOUT,
				   CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hsRenderTargetMemDescInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYRENDERTARGET);

	/* Look up the address from the handle */
	psRGXDestroyRenderTargetOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hsRenderTargetMemDescInt2,
			       psRGXDestroyRenderTargetIN->
			       hsRenderTargetMemDesc,
			       PVRSRV_HANDLE_TYPE_RGX_FWIF_RENDERTARGET);
	if (psRGXDestroyRenderTargetOUT->eError != PVRSRV_OK) {
		goto RGXDestroyRenderTarget_exit;
	}

	psRGXDestroyRenderTargetOUT->eError =
	    RGXDestroyRenderTargetResManProxy(hsRenderTargetMemDescInt2);
	/* Exit early if bridged call fails */
	if (psRGXDestroyRenderTargetOUT->eError != PVRSRV_OK) {
		goto RGXDestroyRenderTarget_exit;
	}

	psRGXDestroyRenderTargetOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psRGXDestroyRenderTargetIN->
				hsRenderTargetMemDesc,
				PVRSRV_HANDLE_TYPE_RGX_FWIF_RENDERTARGET);

 RGXDestroyRenderTarget_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXCreateFreeList(IMG_UINT32 ui32BridgeID,
			      PVRSRV_BRIDGE_IN_RGXCREATEFREELIST *
			      psRGXCreateFreeListIN,
			      PVRSRV_BRIDGE_OUT_RGXCREATEFREELIST *
			      psRGXCreateFreeListOUT,
			      CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC *pssFWFreeListMemDescInt;
	IMG_HANDLE hsFWFreeListMemDescInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXCREATEFREELIST);

	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateFreeListOUT->eError, psConnection,
				  1)

	    /* Look up the address from the handle */
	    psRGXCreateFreeListOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXCreateFreeListIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXCreateFreeListOUT->eError != PVRSRV_OK) {
		goto RGXCreateFreeList_exit;
	}

	psRGXCreateFreeListOUT->eError =
	    RGXCreateFreeList(hDevNodeInt,
			      psRGXCreateFreeListIN->ui32ui32TotalPMPages,
			      psRGXCreateFreeListIN->spsFreeListDevVAddr,
			      &pssFWFreeListMemDescInt,
			      &psRGXCreateFreeListOUT->ui32sFreeListFWDevVAddr);
	/* Exit early if bridged call fails */
	if (psRGXCreateFreeListOUT->eError != PVRSRV_OK) {
		goto RGXCreateFreeList_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hsFWFreeListMemDescInt2 =
	    ResManRegisterRes(psConnection->hResManContext,
			      RESMAN_TYPE_RGX_FWIF_FREELIST,
			      pssFWFreeListMemDescInt, 0,
			      /* FIXME: how can we avoid this cast? */
			      (RESMAN_FREE_FN) & RGXDestroyFreeList);
	if (hsFWFreeListMemDescInt2 == IMG_NULL) {
		psRGXCreateFreeListOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateFreeList_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psRGXCreateFreeListOUT->hsFWFreeListMemDesc,
			    (IMG_HANDLE) hsFWFreeListMemDescInt2,
			    PVRSRV_HANDLE_TYPE_RGX_FWIF_FREELIST,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateFreeListOUT->eError,
				     psConnection);

 RGXCreateFreeList_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyFreeList(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_RGXDESTROYFREELIST *
			       psRGXDestroyFreeListIN,
			       PVRSRV_BRIDGE_OUT_RGXDESTROYFREELIST *
			       psRGXDestroyFreeListOUT,
			       CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hsFWFreeListMemDescInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYFREELIST);

	/* Look up the address from the handle */
	psRGXDestroyFreeListOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hsFWFreeListMemDescInt2,
			       psRGXDestroyFreeListIN->hsFWFreeListMemDesc,
			       PVRSRV_HANDLE_TYPE_RGX_FWIF_FREELIST);
	if (psRGXDestroyFreeListOUT->eError != PVRSRV_OK) {
		goto RGXDestroyFreeList_exit;
	}

	psRGXDestroyFreeListOUT->eError =
	    RGXDestroyFreeListResManProxy(hsFWFreeListMemDescInt2);
	/* Exit early if bridged call fails */
	if (psRGXDestroyFreeListOUT->eError != PVRSRV_OK) {
		goto RGXDestroyFreeList_exit;
	}

	psRGXDestroyFreeListOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psRGXDestroyFreeListIN->
				hsFWFreeListMemDesc,
				PVRSRV_HANDLE_TYPE_RGX_FWIF_FREELIST);

 RGXDestroyFreeList_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXCreateRenderContext(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_RGXCREATERENDERCONTEXT *
				   psRGXCreateRenderContextIN,
				   PVRSRV_BRIDGE_OUT_RGXCREATERENDERCONTEXT *
				   psRGXCreateRenderContextOUT,
				   CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC *psTACCBMemDescInt;
	DEVMEM_MEMDESC *psTACCBCtlMemDescInt;
	DEVMEM_MEMDESC *ps3DCCBMemDescInt;
	DEVMEM_MEMDESC *ps3DCCBCtlMemDescInt;
	RGX_RC_CLEANUP_DATA *psCleanupCookieInt;
	IMG_HANDLE hCleanupCookieInt2;
	DEVMEM_MEMDESC *psFWRenderContextInt;
	DEVMEM_MEMDESC *psFW3DContextStateInt;
	IMG_HANDLE hPrivDataInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXCREATERENDERCONTEXT);

	NEW_HANDLE_BATCH_OR_ERROR(psRGXCreateRenderContextOUT->eError,
				  psConnection, 3)

	    /* Look up the address from the handle */
	    psRGXCreateRenderContextOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXCreateRenderContextIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateRenderContextOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & psTACCBMemDescInt,
			       psRGXCreateRenderContextIN->hTACCBMemDesc,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateRenderContextOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & psTACCBCtlMemDescInt,
			       psRGXCreateRenderContextIN->hTACCBCtlMemDesc,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateRenderContextOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & ps3DCCBMemDescInt,
			       psRGXCreateRenderContextIN->h3DCCBMemDesc,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateRenderContextOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & ps3DCCBCtlMemDescInt,
			       psRGXCreateRenderContextIN->h3DCCBCtlMemDesc,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}
	/* Look up the address from the handle */
	psRGXCreateRenderContextOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPrivDataInt,
			       psRGXCreateRenderContextIN->hPrivData,
			       PVRSRV_HANDLE_TYPE_DEV_PRIV_DATA);
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}

	psRGXCreateRenderContextOUT->eError =
	    PVRSRVRGXCreateRenderContextKM(hDevNodeInt,
					   psTACCBMemDescInt,
					   psTACCBCtlMemDescInt,
					   ps3DCCBMemDescInt,
					   ps3DCCBCtlMemDescInt,
					   &psCleanupCookieInt,
					   &psFWRenderContextInt,
					   &psFW3DContextStateInt,
					   psRGXCreateRenderContextIN->
					   ui32Priority,
					   psRGXCreateRenderContextIN->
					   sMCUFenceAddr,
					   psRGXCreateRenderContextIN->
					   sVDMCallStackAddr, hPrivDataInt);
	/* Exit early if bridged call fails */
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hCleanupCookieInt2 = ResManRegisterRes(psConnection->hResManContext,
					       RESMAN_TYPE_RGX_RENDER_CONTEXT,
					       psCleanupCookieInt, 0,
					       /* FIXME: how can we avoid this cast? */
					       (RESMAN_FREE_FN) &
					       PVRSRVRGXDestroyRenderContextKM);
	if (hCleanupCookieInt2 == IMG_NULL) {
		psRGXCreateRenderContextOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto RGXCreateRenderContext_exit;
	}
	psRGXCreateRenderContextOUT->eError =
	    PVRSRVAllocHandle(psConnection->psHandleBase,
			      &psRGXCreateRenderContextOUT->hCleanupCookie,
			      (IMG_HANDLE) hCleanupCookieInt2,
			      PVRSRV_HANDLE_TYPE_RGX_RC_CLEANUP,
			      PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	if (psRGXCreateRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXCreateRenderContext_exit;
	}
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psRGXCreateRenderContextOUT->hFWRenderContext,
			       (IMG_HANDLE) psFWRenderContextInt,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psRGXCreateRenderContextOUT->hCleanupCookie);
	PVRSRVAllocSubHandleNR(psConnection->psHandleBase,
			       &psRGXCreateRenderContextOUT->hFW3DContextState,
			       (IMG_HANDLE) psFW3DContextStateInt,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC,
			       PVRSRV_HANDLE_ALLOC_FLAG_NONE,
			       psRGXCreateRenderContextOUT->hCleanupCookie);
	COMMIT_HANDLE_BATCH_OR_ERROR(psRGXCreateRenderContextOUT->eError,
				     psConnection);

 RGXCreateRenderContext_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXDestroyRenderContext(IMG_UINT32 ui32BridgeID,
				    PVRSRV_BRIDGE_IN_RGXDESTROYRENDERCONTEXT *
				    psRGXDestroyRenderContextIN,
				    PVRSRV_BRIDGE_OUT_RGXDESTROYRENDERCONTEXT *
				    psRGXDestroyRenderContextOUT,
				    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hCleanupCookieInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYRENDERCONTEXT);

	/* Look up the address from the handle */
	psRGXDestroyRenderContextOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hCleanupCookieInt2,
			       psRGXDestroyRenderContextIN->hCleanupCookie,
			       PVRSRV_HANDLE_TYPE_RGX_RC_CLEANUP);
	if (psRGXDestroyRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXDestroyRenderContext_exit;
	}

	psRGXDestroyRenderContextOUT->eError =
	    RGXDestroyRenderContextResManProxy(hCleanupCookieInt2);
	/* Exit early if bridged call fails */
	if (psRGXDestroyRenderContextOUT->eError != PVRSRV_OK) {
		goto RGXDestroyRenderContext_exit;
	}

	psRGXDestroyRenderContextOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psRGXDestroyRenderContextIN->
				hCleanupCookie,
				PVRSRV_HANDLE_TYPE_RGX_RC_CLEANUP);

 RGXDestroyRenderContext_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeRGXKickTA3D(IMG_UINT32 ui32BridgeID,
			PVRSRV_BRIDGE_IN_RGXKICKTA3D * psRGXKickTA3DIN,
			PVRSRV_BRIDGE_OUT_RGXKICKTA3D * psRGXKickTA3DOUT,
			CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDevNodeInt;
	DEVMEM_MEMDESC *psFWRenderContextInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_RGXTA3D_RGXKICKTA3D);

	/* Look up the address from the handle */
	psRGXKickTA3DOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDevNodeInt,
			       psRGXKickTA3DIN->hDevNode,
			       PVRSRV_HANDLE_TYPE_DEV_NODE);
	if (psRGXKickTA3DOUT->eError != PVRSRV_OK) {
		goto RGXKickTA3D_exit;
	}
	/* Look up the address from the handle */
	psRGXKickTA3DOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & psFWRenderContextInt,
			       psRGXKickTA3DIN->hFWRenderContext,
			       PVRSRV_HANDLE_TYPE_RGX_FW_MEMDESC);
	if (psRGXKickTA3DOUT->eError != PVRSRV_OK) {
		goto RGXKickTA3D_exit;
	}

	psRGXKickTA3DOUT->eError =
	    PVRSRVRGXKickTA3DKM(hDevNodeInt,
				psFWRenderContextInt,
				psRGXKickTA3DIN->bbLastTAInScene,
				psRGXKickTA3DIN->bbKickTA,
				psRGXKickTA3DIN->bbKick3D,
				psRGXKickTA3DIN->ui32TAcCCBWoffUpdate,
				psRGXKickTA3DIN->ui323DcCCBWoffUpdate);

 RGXKickTA3D_exit:

	return 0;
}

PVRSRV_ERROR RegisterRGXTA3DFunctions(IMG_VOID);
IMG_VOID UnregisterRGXTA3DFunctions(IMG_VOID);

/*
 * Register all RGXTA3D functions with services
 */
PVRSRV_ERROR RegisterRGXTA3DFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXCREATEHWRTDATA,
			      PVRSRVBridgeRGXCreateHWRTData);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYHWRTDATA,
			      PVRSRVBridgeRGXDestroyHWRTData);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXCREATERENDERTARGET,
			      PVRSRVBridgeRGXCreateRenderTarget);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYRENDERTARGET,
			      PVRSRVBridgeRGXDestroyRenderTarget);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXCREATEFREELIST,
			      PVRSRVBridgeRGXCreateFreeList);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYFREELIST,
			      PVRSRVBridgeRGXDestroyFreeList);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXCREATERENDERCONTEXT,
			      PVRSRVBridgeRGXCreateRenderContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYRENDERCONTEXT,
			      PVRSRVBridgeRGXDestroyRenderContext);
	SetDispatchTableEntry(PVRSRV_BRIDGE_RGXTA3D_RGXKICKTA3D,
			      PVRSRVBridgeRGXKickTA3D);

	return PVRSRV_OK;
}

/*
 * Unregister all rgxta3d functions with services
 */
IMG_VOID UnregisterRGXTA3DFunctions(IMG_VOID)
{
}
