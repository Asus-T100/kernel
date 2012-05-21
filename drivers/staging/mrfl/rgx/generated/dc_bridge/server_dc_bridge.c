									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for dc
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for dc
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "dc_server.h"

#include "common_dc_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR DCDeviceReleaseResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DCSystemBufferReleaseResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DCDisplayContextDestroyResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DCBufferFreeResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DCBufferUnimportResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DCBufferUnpinResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static PVRSRV_ERROR DCBufferReleaseResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT(eError == PVRSRV_OK);

	return eError;
}

static IMG_INT
PVRSRVBridgeDCDevicesQueryCount(IMG_UINT32 ui32BridgeID,
				PVRSRV_BRIDGE_IN_DCDEVICESQUERYCOUNT *
				psDCDevicesQueryCountIN,
				PVRSRV_BRIDGE_OUT_DCDEVICESQUERYCOUNT *
				psDCDevicesQueryCountOUT,
				CONNECTION_DATA * psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psDCDevicesQueryCountIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCDEVICESQUERYCOUNT);

	psDCDevicesQueryCountOUT->eError =
	    DCDevicesQueryCount(&psDCDevicesQueryCountOUT->ui32DeviceCount);

	return 0;
}

static IMG_INT
PVRSRVBridgeDCDevicesEnumerate(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_DCDEVICESENUMERATE *
			       psDCDevicesEnumerateIN,
			       PVRSRV_BRIDGE_OUT_DCDEVICESENUMERATE *
			       psDCDevicesEnumerateOUT,
			       CONNECTION_DATA * psConnection)
{
	IMG_UINT32 *pui32DeviceIndexInt = IMG_NULL;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCDEVICESENUMERATE);

	pui32DeviceIndexInt =
	    kmalloc(psDCDevicesEnumerateIN->ui32DeviceArraySize *
		    sizeof(IMG_UINT32), GFP_KERNEL);
	if (!pui32DeviceIndexInt) {
		psDCDevicesEnumerateOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDevicesEnumerate_exit;
	}

	psDCDevicesEnumerateOUT->eError =
	    DCDevicesEnumerate(psDCDevicesEnumerateIN->ui32DeviceArraySize,
			       &psDCDevicesEnumerateOUT->ui32DeviceCount,
			       pui32DeviceIndexInt);

	if (copy_to_user
	    (psDCDevicesEnumerateOUT->pui32DeviceIndex, pui32DeviceIndexInt,
	     (psDCDevicesEnumerateOUT->ui32DeviceCount * sizeof(IMG_UINT32))) !=
	    0) {
		psDCDevicesEnumerateOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DCDevicesEnumerate_exit;
	}

 DCDevicesEnumerate_exit:
	if (pui32DeviceIndexInt)
		kfree(pui32DeviceIndexInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeDCDeviceAcquire(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_DCDEVICEACQUIRE *
			    psDCDeviceAcquireIN,
			    PVRSRV_BRIDGE_OUT_DCDEVICEACQUIRE *
			    psDCDeviceAcquireOUT,
			    CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCDEVICEACQUIRE);

	NEW_HANDLE_BATCH_OR_ERROR(psDCDeviceAcquireOUT->eError, psConnection, 1)

	    psDCDeviceAcquireOUT->eError =
	    DCDeviceAcquire(psDCDeviceAcquireIN->ui32DeviceIndex, &psDeviceInt);
	/* Exit early if bridged call fails */
	if (psDCDeviceAcquireOUT->eError != PVRSRV_OK) {
		goto DCDeviceAcquire_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hDeviceInt2 = ResManRegisterRes(psConnection->hResManContext,
					RESMAN_TYPE_DC_DEVICE, psDeviceInt, 0,
					/* FIXME: how can we avoid this cast? */
					(RESMAN_FREE_FN) & DCDeviceRelease);
	if (hDeviceInt2 == IMG_NULL) {
		psDCDeviceAcquireOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DCDeviceAcquire_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDCDeviceAcquireOUT->hDevice,
			    (IMG_HANDLE) hDeviceInt2,
			    PVRSRV_HANDLE_TYPE_DC_DEVICE,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDCDeviceAcquireOUT->eError,
				     psConnection);

 DCDeviceAcquire_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCDeviceRelease(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_DCDEVICERELEASE *
			    psDCDeviceReleaseIN,
			    PVRSRV_BRIDGE_OUT_DCDEVICERELEASE *
			    psDCDeviceReleaseOUT,
			    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDeviceInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCDEVICERELEASE);

	/* Look up the address from the handle */
	psDCDeviceReleaseOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCDeviceReleaseIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCDeviceReleaseOUT->eError != PVRSRV_OK) {
		goto DCDeviceRelease_exit;
	}

	psDCDeviceReleaseOUT->eError = DCDeviceReleaseResManProxy(hDeviceInt2);
	/* Exit early if bridged call fails */
	if (psDCDeviceReleaseOUT->eError != PVRSRV_OK) {
		goto DCDeviceRelease_exit;
	}

	psDCDeviceReleaseOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDCDeviceReleaseIN->hDevice,
				PVRSRV_HANDLE_TYPE_DC_DEVICE);

 DCDeviceRelease_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCGetInfo(IMG_UINT32 ui32BridgeID,
		      PVRSRV_BRIDGE_IN_DCGETINFO * psDCGetInfoIN,
		      PVRSRV_BRIDGE_OUT_DCGETINFO * psDCGetInfoOUT,
		      CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCGETINFO);

	/* Look up the address from the handle */
	psDCGetInfoOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCGetInfoIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCGetInfoOUT->eError != PVRSRV_OK) {
		goto DCGetInfo_exit;
	}

	/* Look up the data from the resman address */
	psDCGetInfoOUT->eError =
	    ResManFindPrivateDataByPtr(hDeviceInt2, (IMG_VOID **) & psDeviceInt,
				       IMG_NULL);
	if (psDCGetInfoOUT->eError != PVRSRV_OK) {
		goto DCGetInfo_exit;
	}

	psDCGetInfoOUT->eError =
	    DCGetInfo(psDeviceInt, &psDCGetInfoOUT->sDisplayInfo);

 DCGetInfo_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCPanelQueryCount(IMG_UINT32 ui32BridgeID,
			      PVRSRV_BRIDGE_IN_DCPANELQUERYCOUNT *
			      psDCPanelQueryCountIN,
			      PVRSRV_BRIDGE_OUT_DCPANELQUERYCOUNT *
			      psDCPanelQueryCountOUT,
			      CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCPANELQUERYCOUNT);

	/* Look up the address from the handle */
	psDCPanelQueryCountOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCPanelQueryCountIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCPanelQueryCountOUT->eError != PVRSRV_OK) {
		goto DCPanelQueryCount_exit;
	}

	/* Look up the data from the resman address */
	psDCPanelQueryCountOUT->eError =
	    ResManFindPrivateDataByPtr(hDeviceInt2, (IMG_VOID **) & psDeviceInt,
				       IMG_NULL);
	if (psDCPanelQueryCountOUT->eError != PVRSRV_OK) {
		goto DCPanelQueryCount_exit;
	}

	psDCPanelQueryCountOUT->eError =
	    DCPanelQueryCount(psDeviceInt,
			      &psDCPanelQueryCountOUT->ui32NumPanels);

 DCPanelQueryCount_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCPanelQuery(IMG_UINT32 ui32BridgeID,
			 PVRSRV_BRIDGE_IN_DCPANELQUERY * psDCPanelQueryIN,
			 PVRSRV_BRIDGE_OUT_DCPANELQUERY * psDCPanelQueryOUT,
			 CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;
	PVRSRV_SURFACE_INFO *psSurfInfoInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCPANELQUERY);

	psSurfInfoInt =
	    kmalloc(psDCPanelQueryIN->ui32PanelsArraySize *
		    sizeof(PVRSRV_SURFACE_INFO), GFP_KERNEL);
	if (!psSurfInfoInt) {
		psDCPanelQueryOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCPanelQuery_exit;
	}

	/* Look up the address from the handle */
	psDCPanelQueryOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCPanelQueryIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCPanelQueryOUT->eError != PVRSRV_OK) {
		goto DCPanelQuery_exit;
	}

	/* Look up the data from the resman address */
	psDCPanelQueryOUT->eError =
	    ResManFindPrivateDataByPtr(hDeviceInt2, (IMG_VOID **) & psDeviceInt,
				       IMG_NULL);
	if (psDCPanelQueryOUT->eError != PVRSRV_OK) {
		goto DCPanelQuery_exit;
	}

	psDCPanelQueryOUT->eError =
	    DCPanelQuery(psDeviceInt,
			 psDCPanelQueryIN->ui32PanelsArraySize,
			 &psDCPanelQueryOUT->ui32NumPanels, psSurfInfoInt);

	if (copy_to_user(psDCPanelQueryOUT->psSurfInfo, psSurfInfoInt,
			 (psDCPanelQueryOUT->ui32NumPanels *
			  sizeof(PVRSRV_SURFACE_INFO))) != 0) {
		psDCPanelQueryOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DCPanelQuery_exit;
	}

 DCPanelQuery_exit:
	if (psSurfInfoInt)
		kfree(psSurfInfoInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeDCFormatQuery(IMG_UINT32 ui32BridgeID,
			  PVRSRV_BRIDGE_IN_DCFORMATQUERY * psDCFormatQueryIN,
			  PVRSRV_BRIDGE_OUT_DCFORMATQUERY * psDCFormatQueryOUT,
			  CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;
	PVRSRV_SURFACE_FORMAT *psFormatInt = IMG_NULL;
	IMG_UINT32 *pui32SupportedInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCFORMATQUERY);

	psFormatInt =
	    kmalloc(psDCFormatQueryIN->ui32NumFormats *
		    sizeof(PVRSRV_SURFACE_FORMAT), GFP_KERNEL);
	if (!psFormatInt) {
		psDCFormatQueryOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCFormatQuery_exit;
	}

	if (copy_from_user(psFormatInt, psDCFormatQueryIN->psFormat,
			   psDCFormatQueryIN->ui32NumFormats *
			   sizeof(PVRSRV_SURFACE_FORMAT)) != 0) {
		psDCFormatQueryOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DCFormatQuery_exit;
	}

	pui32SupportedInt =
	    kmalloc(psDCFormatQueryIN->ui32NumFormats * sizeof(IMG_UINT32),
		    GFP_KERNEL);
	if (!pui32SupportedInt) {
		psDCFormatQueryOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCFormatQuery_exit;
	}

	/* Look up the address from the handle */
	psDCFormatQueryOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCFormatQueryIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCFormatQueryOUT->eError != PVRSRV_OK) {
		goto DCFormatQuery_exit;
	}

	/* Look up the data from the resman address */
	psDCFormatQueryOUT->eError =
	    ResManFindPrivateDataByPtr(hDeviceInt2, (IMG_VOID **) & psDeviceInt,
				       IMG_NULL);
	if (psDCFormatQueryOUT->eError != PVRSRV_OK) {
		goto DCFormatQuery_exit;
	}

	psDCFormatQueryOUT->eError =
	    DCFormatQuery(psDeviceInt,
			  psDCFormatQueryIN->ui32NumFormats,
			  psFormatInt, pui32SupportedInt);

	if (copy_to_user(psDCFormatQueryOUT->pui32Supported, pui32SupportedInt,
			 (psDCFormatQueryIN->ui32NumFormats *
			  sizeof(IMG_UINT32))) != 0) {
		psDCFormatQueryOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DCFormatQuery_exit;
	}

 DCFormatQuery_exit:
	if (psFormatInt)
		kfree(psFormatInt);
	if (pui32SupportedInt)
		kfree(pui32SupportedInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeDCDimQuery(IMG_UINT32 ui32BridgeID,
		       PVRSRV_BRIDGE_IN_DCDIMQUERY * psDCDimQueryIN,
		       PVRSRV_BRIDGE_OUT_DCDIMQUERY * psDCDimQueryOUT,
		       CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;
	PVRSRV_SURFACE_DIMS *psDimInt = IMG_NULL;
	IMG_UINT32 *pui32SupportedInt = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCDIMQUERY);

	psDimInt =
	    kmalloc(psDCDimQueryIN->ui32NumDims * sizeof(PVRSRV_SURFACE_DIMS),
		    GFP_KERNEL);
	if (!psDimInt) {
		psDCDimQueryOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDimQuery_exit;
	}

	if (copy_from_user(psDimInt, psDCDimQueryIN->psDim,
			   psDCDimQueryIN->ui32NumDims *
			   sizeof(PVRSRV_SURFACE_DIMS)) != 0) {
		psDCDimQueryOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DCDimQuery_exit;
	}

	pui32SupportedInt =
	    kmalloc(psDCDimQueryIN->ui32NumDims * sizeof(IMG_UINT32),
		    GFP_KERNEL);
	if (!pui32SupportedInt) {
		psDCDimQueryOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDimQuery_exit;
	}

	/* Look up the address from the handle */
	psDCDimQueryOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCDimQueryIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCDimQueryOUT->eError != PVRSRV_OK) {
		goto DCDimQuery_exit;
	}

	/* Look up the data from the resman address */
	psDCDimQueryOUT->eError =
	    ResManFindPrivateDataByPtr(hDeviceInt2, (IMG_VOID **) & psDeviceInt,
				       IMG_NULL);
	if (psDCDimQueryOUT->eError != PVRSRV_OK) {
		goto DCDimQuery_exit;
	}

	psDCDimQueryOUT->eError =
	    DCDimQuery(psDeviceInt,
		       psDCDimQueryIN->ui32NumDims,
		       psDimInt, pui32SupportedInt);

	if (copy_to_user(psDCDimQueryOUT->pui32Supported, pui32SupportedInt,
			 (psDCDimQueryIN->ui32NumDims * sizeof(IMG_UINT32))) !=
	    0) {
		psDCDimQueryOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DCDimQuery_exit;
	}

 DCDimQuery_exit:
	if (psDimInt)
		kfree(psDimInt);
	if (pui32SupportedInt)
		kfree(pui32SupportedInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeDCSystemBufferAcquire(IMG_UINT32 ui32BridgeID,
				  PVRSRV_BRIDGE_IN_DCSYSTEMBUFFERACQUIRE *
				  psDCSystemBufferAcquireIN,
				  PVRSRV_BRIDGE_OUT_DCSYSTEMBUFFERACQUIRE *
				  psDCSystemBufferAcquireOUT,
				  CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;
	DC_BUFFER *psBufferInt;
	IMG_HANDLE hBufferInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCSYSTEMBUFFERACQUIRE);

	NEW_HANDLE_BATCH_OR_ERROR(psDCSystemBufferAcquireOUT->eError,
				  psConnection, 1)

	    /* Look up the address from the handle */
	    psDCSystemBufferAcquireOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCSystemBufferAcquireIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCSystemBufferAcquireOUT->eError != PVRSRV_OK) {
		goto DCSystemBufferAcquire_exit;
	}

	/* Look up the data from the resman address */
	psDCSystemBufferAcquireOUT->eError =
	    ResManFindPrivateDataByPtr(hDeviceInt2, (IMG_VOID **) & psDeviceInt,
				       IMG_NULL);
	if (psDCSystemBufferAcquireOUT->eError != PVRSRV_OK) {
		goto DCSystemBufferAcquire_exit;
	}

	psDCSystemBufferAcquireOUT->eError =
	    DCSystemBufferAcquire(psDeviceInt,
				  &psDCSystemBufferAcquireOUT->ui32Stride,
				  &psBufferInt);
	/* Exit early if bridged call fails */
	if (psDCSystemBufferAcquireOUT->eError != PVRSRV_OK) {
		goto DCSystemBufferAcquire_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hBufferInt2 = ResManRegisterRes(psConnection->hResManContext,
					RESMAN_TYPE_DC_BUFFER, psBufferInt, 0,
					/* FIXME: how can we avoid this cast? */
					(RESMAN_FREE_FN) &
					DCSystemBufferRelease);
	if (hBufferInt2 == IMG_NULL) {
		psDCSystemBufferAcquireOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DCSystemBufferAcquire_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDCSystemBufferAcquireOUT->hBuffer,
			    (IMG_HANDLE) hBufferInt2,
			    PVRSRV_HANDLE_TYPE_DC_BUFFER,
			    PVRSRV_HANDLE_ALLOC_FLAG_SHARED);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDCSystemBufferAcquireOUT->eError,
				     psConnection);

 DCSystemBufferAcquire_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCSystemBufferRelease(IMG_UINT32 ui32BridgeID,
				  PVRSRV_BRIDGE_IN_DCSYSTEMBUFFERRELEASE *
				  psDCSystemBufferReleaseIN,
				  PVRSRV_BRIDGE_OUT_DCSYSTEMBUFFERRELEASE *
				  psDCSystemBufferReleaseOUT,
				  CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hBufferInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCSYSTEMBUFFERRELEASE);

	/* Look up the address from the handle */
	psDCSystemBufferReleaseOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hBufferInt2,
			       psDCSystemBufferReleaseIN->hBuffer,
			       PVRSRV_HANDLE_TYPE_DC_BUFFER);
	if (psDCSystemBufferReleaseOUT->eError != PVRSRV_OK) {
		goto DCSystemBufferRelease_exit;
	}

	psDCSystemBufferReleaseOUT->eError =
	    DCSystemBufferReleaseResManProxy(hBufferInt2);
	/* Exit early if bridged call fails */
	if (psDCSystemBufferReleaseOUT->eError != PVRSRV_OK) {
		goto DCSystemBufferRelease_exit;
	}

	psDCSystemBufferReleaseOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDCSystemBufferReleaseIN->hBuffer,
				PVRSRV_HANDLE_TYPE_DC_BUFFER);

 DCSystemBufferRelease_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCDisplayContextCreate(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTCREATE *
				   psDCDisplayContextCreateIN,
				   PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTCREATE *
				   psDCDisplayContextCreateOUT,
				   CONNECTION_DATA * psConnection)
{
	DC_DEVICE *psDeviceInt;
	IMG_HANDLE hDeviceInt2;
	DC_DISPLAY_CONTEXT *psDisplayContextInt;
	IMG_HANDLE hDisplayContextInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTCREATE);

	NEW_HANDLE_BATCH_OR_ERROR(psDCDisplayContextCreateOUT->eError,
				  psConnection, 1)

	    /* Look up the address from the handle */
	    psDCDisplayContextCreateOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDeviceInt2,
			       psDCDisplayContextCreateIN->hDevice,
			       PVRSRV_HANDLE_TYPE_DC_DEVICE);
	if (psDCDisplayContextCreateOUT->eError != PVRSRV_OK) {
		goto DCDisplayContextCreate_exit;
	}

	/* Look up the data from the resman address */
	psDCDisplayContextCreateOUT->eError =
	    ResManFindPrivateDataByPtr(hDeviceInt2, (IMG_VOID **) & psDeviceInt,
				       IMG_NULL);
	if (psDCDisplayContextCreateOUT->eError != PVRSRV_OK) {
		goto DCDisplayContextCreate_exit;
	}

	psDCDisplayContextCreateOUT->eError =
	    DCDisplayContextCreate(psDeviceInt, &psDisplayContextInt);
	/* Exit early if bridged call fails */
	if (psDCDisplayContextCreateOUT->eError != PVRSRV_OK) {
		goto DCDisplayContextCreate_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hDisplayContextInt2 = ResManRegisterRes(psConnection->hResManContext,
						RESMAN_TYPE_DC_DISPLAY_CONTEXT,
						psDisplayContextInt, 0,
						/* FIXME: how can we avoid this cast? */
						(RESMAN_FREE_FN) &
						DCDisplayContextDestroy);
	if (hDisplayContextInt2 == IMG_NULL) {
		psDCDisplayContextCreateOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DCDisplayContextCreate_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDCDisplayContextCreateOUT->hDisplayContext,
			    (IMG_HANDLE) hDisplayContextInt2,
			    PVRSRV_HANDLE_TYPE_DC_DISPLAY_CONTEXT,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDCDisplayContextCreateOUT->eError,
				     psConnection);

 DCDisplayContextCreate_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCDisplayContextConfigure(IMG_UINT32 ui32BridgeID,
				      PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTCONFIGURE
				      * psDCDisplayContextConfigureIN,
				      PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTCONFIGURE
				      * psDCDisplayContextConfigureOUT,
				      CONNECTION_DATA * psConnection)
{
	DC_DISPLAY_CONTEXT *psDisplayContextInt;
	IMG_HANDLE hDisplayContextInt2;
	PVRSRV_SURFACE_CONFIG_INFO *psSurfInfoInt = IMG_NULL;
	DC_BUFFER **psBuffersInt = IMG_NULL;
	IMG_HANDLE *hBuffersInt2 = IMG_NULL;
	SERVER_SYNC_PRIMITIVE **psSyncInt = IMG_NULL;
	IMG_HANDLE *hSyncInt2 = IMG_NULL;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTCONFIGURE);

	psSurfInfoInt =
	    kmalloc(psDCDisplayContextConfigureIN->ui32PipeCount *
		    sizeof(PVRSRV_SURFACE_CONFIG_INFO), GFP_KERNEL);
	if (!psSurfInfoInt) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDisplayContextConfigure_exit;
	}

	if (copy_from_user
	    (psSurfInfoInt, psDCDisplayContextConfigureIN->psSurfInfo,
	     psDCDisplayContextConfigureIN->ui32PipeCount *
	     sizeof(PVRSRV_SURFACE_CONFIG_INFO)) != 0) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_INVALID_PARAMS;

		goto DCDisplayContextConfigure_exit;
	}

	psBuffersInt =
	    kmalloc(psDCDisplayContextConfigureIN->ui32PipeCount *
		    sizeof(DC_BUFFER *), GFP_KERNEL);
	if (!psBuffersInt) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDisplayContextConfigure_exit;
	}

	hBuffersInt2 =
	    kmalloc(psDCDisplayContextConfigureIN->ui32PipeCount *
		    sizeof(IMG_HANDLE), GFP_KERNEL);
	if (!hBuffersInt2) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDisplayContextConfigure_exit;
	}

	if (copy_from_user
	    (hBuffersInt2, psDCDisplayContextConfigureIN->phBuffers,
	     psDCDisplayContextConfigureIN->ui32PipeCount *
	     sizeof(IMG_HANDLE)) != 0) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_INVALID_PARAMS;

		goto DCDisplayContextConfigure_exit;
	}

	psSyncInt =
	    kmalloc(psDCDisplayContextConfigureIN->ui32SyncCount *
		    sizeof(SERVER_SYNC_PRIMITIVE *), GFP_KERNEL);
	if (!psSyncInt) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDisplayContextConfigure_exit;
	}

	hSyncInt2 =
	    kmalloc(psDCDisplayContextConfigureIN->ui32SyncCount *
		    sizeof(IMG_HANDLE), GFP_KERNEL);
	if (!hSyncInt2) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCDisplayContextConfigure_exit;
	}

	if (copy_from_user(hSyncInt2, psDCDisplayContextConfigureIN->phSync,
			   psDCDisplayContextConfigureIN->ui32SyncCount *
			   sizeof(IMG_HANDLE)) != 0) {
		psDCDisplayContextConfigureOUT->eError =
		    PVRSRV_ERROR_INVALID_PARAMS;

		goto DCDisplayContextConfigure_exit;
	}

	/* Look up the address from the handle */
	psDCDisplayContextConfigureOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDisplayContextInt2,
			       psDCDisplayContextConfigureIN->hDisplayContext,
			       PVRSRV_HANDLE_TYPE_DC_DISPLAY_CONTEXT);
	if (psDCDisplayContextConfigureOUT->eError != PVRSRV_OK) {
		goto DCDisplayContextConfigure_exit;
	}

	/* Look up the data from the resman address */
	psDCDisplayContextConfigureOUT->eError =
	    ResManFindPrivateDataByPtr(hDisplayContextInt2,
				       (IMG_VOID **) & psDisplayContextInt,
				       IMG_NULL);
	if (psDCDisplayContextConfigureOUT->eError != PVRSRV_OK) {
		goto DCDisplayContextConfigure_exit;
	}
	{
		IMG_UINT32 i;

		for (i = 0; i < psDCDisplayContextConfigureIN->ui32PipeCount;
		     i++) {
			/* Look up the address from the handle */
			psDCDisplayContextConfigureOUT->eError =
			    PVRSRVLookupHandle(psConnection->psHandleBase,
					       (IMG_HANDLE *) & hBuffersInt2[i],
					       hBuffersInt2[i],
					       PVRSRV_HANDLE_TYPE_DC_BUFFER);
			if (psDCDisplayContextConfigureOUT->eError != PVRSRV_OK) {
				goto DCDisplayContextConfigure_exit;
			}

			/* Look up the data from the resman address */
			psDCDisplayContextConfigureOUT->eError =
			    ResManFindPrivateDataByPtr(hBuffersInt2[i],
						       (IMG_VOID **) &
						       psBuffersInt[i],
						       IMG_NULL);
			if (psDCDisplayContextConfigureOUT->eError != PVRSRV_OK) {
				goto DCDisplayContextConfigure_exit;
			}
		}
	}
	{
		IMG_UINT32 i;

		for (i = 0; i < psDCDisplayContextConfigureIN->ui32SyncCount;
		     i++) {
			/* Look up the address from the handle */
			psDCDisplayContextConfigureOUT->eError =
			    PVRSRVLookupHandle(psConnection->psHandleBase,
					       (IMG_HANDLE *) & hSyncInt2[i],
					       hSyncInt2[i],
					       PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE);
			if (psDCDisplayContextConfigureOUT->eError != PVRSRV_OK) {
				goto DCDisplayContextConfigure_exit;
			}

			/* Look up the data from the resman address */
			psDCDisplayContextConfigureOUT->eError =
			    ResManFindPrivateDataByPtr(hSyncInt2[i],
						       (IMG_VOID **) &
						       psSyncInt[i], IMG_NULL);
			if (psDCDisplayContextConfigureOUT->eError != PVRSRV_OK) {
				goto DCDisplayContextConfigure_exit;
			}
		}
	}

	psDCDisplayContextConfigureOUT->eError =
	    DCDisplayContextConfigure(psDisplayContextInt,
				      psDCDisplayContextConfigureIN->
				      ui32PipeCount, psSurfInfoInt,
				      psBuffersInt,
				      psDCDisplayContextConfigureIN->
				      ui32SyncCount, psSyncInt,
				      psDCDisplayContextConfigureIN->
				      ui32DisplayPeriod);

 DCDisplayContextConfigure_exit:
	if (psSurfInfoInt)
		kfree(psSurfInfoInt);
	if (psBuffersInt)
		kfree(psBuffersInt);
	if (hBuffersInt2)
		kfree(hBuffersInt2);
	if (psSyncInt)
		kfree(psSyncInt);
	if (hSyncInt2)
		kfree(hSyncInt2);

	return 0;
}

static IMG_INT
PVRSRVBridgeDCDisplayContextDestroy(IMG_UINT32 ui32BridgeID,
				    PVRSRV_BRIDGE_IN_DCDISPLAYCONTEXTDESTROY *
				    psDCDisplayContextDestroyIN,
				    PVRSRV_BRIDGE_OUT_DCDISPLAYCONTEXTDESTROY *
				    psDCDisplayContextDestroyOUT,
				    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hDisplayContextInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTDESTROY);

	/* Look up the address from the handle */
	psDCDisplayContextDestroyOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDisplayContextInt2,
			       psDCDisplayContextDestroyIN->hDisplayContext,
			       PVRSRV_HANDLE_TYPE_DC_DISPLAY_CONTEXT);
	if (psDCDisplayContextDestroyOUT->eError != PVRSRV_OK) {
		goto DCDisplayContextDestroy_exit;
	}

	psDCDisplayContextDestroyOUT->eError =
	    DCDisplayContextDestroyResManProxy(hDisplayContextInt2);
	/* Exit early if bridged call fails */
	if (psDCDisplayContextDestroyOUT->eError != PVRSRV_OK) {
		goto DCDisplayContextDestroy_exit;
	}

	psDCDisplayContextDestroyOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDCDisplayContextDestroyIN->
				hDisplayContext,
				PVRSRV_HANDLE_TYPE_DC_DISPLAY_CONTEXT);

 DCDisplayContextDestroy_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferAlloc(IMG_UINT32 ui32BridgeID,
			  PVRSRV_BRIDGE_IN_DCBUFFERALLOC * psDCBufferAllocIN,
			  PVRSRV_BRIDGE_OUT_DCBUFFERALLOC * psDCBufferAllocOUT,
			  CONNECTION_DATA * psConnection)
{
	DC_DISPLAY_CONTEXT *psDisplayContextInt;
	IMG_HANDLE hDisplayContextInt2;
	DC_BUFFER *psBufferInt;
	IMG_HANDLE hBufferInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCBUFFERALLOC);

	NEW_HANDLE_BATCH_OR_ERROR(psDCBufferAllocOUT->eError, psConnection, 1)

	    /* Look up the address from the handle */
	    psDCBufferAllocOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDisplayContextInt2,
			       psDCBufferAllocIN->hDisplayContext,
			       PVRSRV_HANDLE_TYPE_DC_DISPLAY_CONTEXT);
	if (psDCBufferAllocOUT->eError != PVRSRV_OK) {
		goto DCBufferAlloc_exit;
	}

	/* Look up the data from the resman address */
	psDCBufferAllocOUT->eError =
	    ResManFindPrivateDataByPtr(hDisplayContextInt2,
				       (IMG_VOID **) & psDisplayContextInt,
				       IMG_NULL);
	if (psDCBufferAllocOUT->eError != PVRSRV_OK) {
		goto DCBufferAlloc_exit;
	}

	psDCBufferAllocOUT->eError =
	    DCBufferAlloc(psDisplayContextInt,
			  &psDCBufferAllocIN->sSurfInfo,
			  &psDCBufferAllocOUT->ui32Stride, &psBufferInt);
	/* Exit early if bridged call fails */
	if (psDCBufferAllocOUT->eError != PVRSRV_OK) {
		goto DCBufferAlloc_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hBufferInt2 = ResManRegisterRes(psConnection->hResManContext,
					RESMAN_TYPE_DC_BUFFER, psBufferInt, 0,
					/* FIXME: how can we avoid this cast? */
					(RESMAN_FREE_FN) & DCBufferFree);
	if (hBufferInt2 == IMG_NULL) {
		psDCBufferAllocOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DCBufferAlloc_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDCBufferAllocOUT->hBuffer,
			    (IMG_HANDLE) hBufferInt2,
			    PVRSRV_HANDLE_TYPE_DC_BUFFER,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDCBufferAllocOUT->eError, psConnection);

 DCBufferAlloc_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferImport(IMG_UINT32 ui32BridgeID,
			   PVRSRV_BRIDGE_IN_DCBUFFERIMPORT * psDCBufferImportIN,
			   PVRSRV_BRIDGE_OUT_DCBUFFERIMPORT *
			   psDCBufferImportOUT, CONNECTION_DATA * psConnection)
{
	DC_DISPLAY_CONTEXT *psDisplayContextInt;
	IMG_HANDLE hDisplayContextInt2;
	PMR **psImportInt = IMG_NULL;
	DC_BUFFER *psBufferInt;
	IMG_HANDLE hBufferInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCBUFFERIMPORT);

	psImportInt =
	    kmalloc(psDCBufferImportIN->ui32NumPlanes * sizeof(PMR *),
		    GFP_KERNEL);
	if (!psImportInt) {
		psDCBufferImportOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto DCBufferImport_exit;
	}

	if (copy_from_user(psImportInt, psDCBufferImportIN->phImport,
			   psDCBufferImportIN->ui32NumPlanes *
			   sizeof(IMG_HANDLE)) != 0) {
		psDCBufferImportOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto DCBufferImport_exit;
	}

	NEW_HANDLE_BATCH_OR_ERROR(psDCBufferImportOUT->eError, psConnection, 1)

	    /* Look up the address from the handle */
	    psDCBufferImportOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hDisplayContextInt2,
			       psDCBufferImportIN->hDisplayContext,
			       PVRSRV_HANDLE_TYPE_DC_DISPLAY_CONTEXT);
	if (psDCBufferImportOUT->eError != PVRSRV_OK) {
		goto DCBufferImport_exit;
	}

	/* Look up the data from the resman address */
	psDCBufferImportOUT->eError =
	    ResManFindPrivateDataByPtr(hDisplayContextInt2,
				       (IMG_VOID **) & psDisplayContextInt,
				       IMG_NULL);
	if (psDCBufferImportOUT->eError != PVRSRV_OK) {
		goto DCBufferImport_exit;
	}
	{
		IMG_UINT32 i;

		for (i = 0; i < psDCBufferImportIN->ui32NumPlanes; i++) {
			/* Look up the address from the handle */
			psDCBufferImportOUT->eError =
			    PVRSRVLookupHandle(psConnection->psHandleBase,
					       (IMG_HANDLE *) & psImportInt[i],
					       psImportInt[i],
					       PVRSRV_HANDLE_TYPE_DEVMEM_MEM_EXPORT);
			if (psDCBufferImportOUT->eError != PVRSRV_OK) {
				goto DCBufferImport_exit;
			}
		}
	}

	psDCBufferImportOUT->eError =
	    DCBufferImport(psDisplayContextInt,
			   psDCBufferImportIN->ui32NumPlanes,
			   psImportInt,
			   &psDCBufferImportIN->sSurfAttrib, &psBufferInt);
	/* Exit early if bridged call fails */
	if (psDCBufferImportOUT->eError != PVRSRV_OK) {
		goto DCBufferImport_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hBufferInt2 = ResManRegisterRes(psConnection->hResManContext,
					RESMAN_TYPE_DC_BUFFER, psBufferInt, 0,
					/* FIXME: how can we avoid this cast? */
					(RESMAN_FREE_FN) & DCBufferFree);
	if (hBufferInt2 == IMG_NULL) {
		psDCBufferImportOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DCBufferImport_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDCBufferImportOUT->hBuffer,
			    (IMG_HANDLE) hBufferInt2,
			    PVRSRV_HANDLE_TYPE_DC_BUFFER,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDCBufferImportOUT->eError, psConnection);

 DCBufferImport_exit:
	if (psImportInt)
		kfree(psImportInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferFree(IMG_UINT32 ui32BridgeID,
			 PVRSRV_BRIDGE_IN_DCBUFFERFREE * psDCBufferFreeIN,
			 PVRSRV_BRIDGE_OUT_DCBUFFERFREE * psDCBufferFreeOUT,
			 CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hBufferInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCBUFFERFREE);

	/* Look up the address from the handle */
	psDCBufferFreeOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hBufferInt2,
			       psDCBufferFreeIN->hBuffer,
			       PVRSRV_HANDLE_TYPE_DC_BUFFER);
	if (psDCBufferFreeOUT->eError != PVRSRV_OK) {
		goto DCBufferFree_exit;
	}

	psDCBufferFreeOUT->eError = DCBufferFreeResManProxy(hBufferInt2);
	/* Exit early if bridged call fails */
	if (psDCBufferFreeOUT->eError != PVRSRV_OK) {
		goto DCBufferFree_exit;
	}

	psDCBufferFreeOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDCBufferFreeIN->hBuffer,
				PVRSRV_HANDLE_TYPE_DC_BUFFER);

 DCBufferFree_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferUnimport(IMG_UINT32 ui32BridgeID,
			     PVRSRV_BRIDGE_IN_DCBUFFERUNIMPORT *
			     psDCBufferUnimportIN,
			     PVRSRV_BRIDGE_OUT_DCBUFFERUNIMPORT *
			     psDCBufferUnimportOUT,
			     CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hBufferInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCBUFFERUNIMPORT);

	/* Look up the address from the handle */
	psDCBufferUnimportOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hBufferInt2,
			       psDCBufferUnimportIN->hBuffer,
			       PVRSRV_HANDLE_TYPE_DC_BUFFER);
	if (psDCBufferUnimportOUT->eError != PVRSRV_OK) {
		goto DCBufferUnimport_exit;
	}

	psDCBufferUnimportOUT->eError =
	    DCBufferUnimportResManProxy(hBufferInt2);
	/* Exit early if bridged call fails */
	if (psDCBufferUnimportOUT->eError != PVRSRV_OK) {
		goto DCBufferUnimport_exit;
	}

	psDCBufferUnimportOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDCBufferUnimportIN->hBuffer,
				PVRSRV_HANDLE_TYPE_DC_BUFFER);

 DCBufferUnimport_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferPin(IMG_UINT32 ui32BridgeID,
			PVRSRV_BRIDGE_IN_DCBUFFERPIN * psDCBufferPinIN,
			PVRSRV_BRIDGE_OUT_DCBUFFERPIN * psDCBufferPinOUT,
			CONNECTION_DATA * psConnection)
{
	DC_BUFFER *psBufferInt;
	IMG_HANDLE hBufferInt2;
	DC_PIN_HANDLE hPinHandleInt;
	IMG_HANDLE hPinHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCBUFFERPIN);

	NEW_HANDLE_BATCH_OR_ERROR(psDCBufferPinOUT->eError, psConnection, 1)

	    /* Look up the address from the handle */
	    psDCBufferPinOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hBufferInt2,
			       psDCBufferPinIN->hBuffer,
			       PVRSRV_HANDLE_TYPE_DC_BUFFER);
	if (psDCBufferPinOUT->eError != PVRSRV_OK) {
		goto DCBufferPin_exit;
	}

	/* Look up the data from the resman address */
	psDCBufferPinOUT->eError =
	    ResManFindPrivateDataByPtr(hBufferInt2, (IMG_VOID **) & psBufferInt,
				       IMG_NULL);
	if (psDCBufferPinOUT->eError != PVRSRV_OK) {
		goto DCBufferPin_exit;
	}

	psDCBufferPinOUT->eError = DCBufferPin(psBufferInt, &hPinHandleInt);
	/* Exit early if bridged call fails */
	if (psDCBufferPinOUT->eError != PVRSRV_OK) {
		goto DCBufferPin_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hPinHandleInt2 = ResManRegisterRes(psConnection->hResManContext,
					   RESMAN_TYPE_DC_PIN_HANDLE,
					   hPinHandleInt, 0,
					   /* FIXME: how can we avoid this cast? */
					   (RESMAN_FREE_FN) & DCBufferUnpin);
	if (hPinHandleInt2 == IMG_NULL) {
		psDCBufferPinOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DCBufferPin_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDCBufferPinOUT->hPinHandle,
			    (IMG_HANDLE) hPinHandleInt2,
			    PVRSRV_HANDLE_TYPE_DC_PIN_HANDLE,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDCBufferPinOUT->eError, psConnection);

 DCBufferPin_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferUnpin(IMG_UINT32 ui32BridgeID,
			  PVRSRV_BRIDGE_IN_DCBUFFERUNPIN * psDCBufferUnpinIN,
			  PVRSRV_BRIDGE_OUT_DCBUFFERUNPIN * psDCBufferUnpinOUT,
			  CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hPinHandleInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_DC_DCBUFFERUNPIN);

	/* Look up the address from the handle */
	psDCBufferUnpinOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hPinHandleInt2,
			       psDCBufferUnpinIN->hPinHandle,
			       PVRSRV_HANDLE_TYPE_DC_PIN_HANDLE);
	if (psDCBufferUnpinOUT->eError != PVRSRV_OK) {
		goto DCBufferUnpin_exit;
	}

	psDCBufferUnpinOUT->eError = DCBufferUnpinResManProxy(hPinHandleInt2);
	/* Exit early if bridged call fails */
	if (psDCBufferUnpinOUT->eError != PVRSRV_OK) {
		goto DCBufferUnpin_exit;
	}

	psDCBufferUnpinOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDCBufferUnpinIN->hPinHandle,
				PVRSRV_HANDLE_TYPE_DC_PIN_HANDLE);

 DCBufferUnpin_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferAcquire(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_DCBUFFERACQUIRE *
			    psDCBufferAcquireIN,
			    PVRSRV_BRIDGE_OUT_DCBUFFERACQUIRE *
			    psDCBufferAcquireOUT,
			    CONNECTION_DATA * psConnection)
{
	DC_BUFFER *psBufferInt;
	IMG_HANDLE hBufferInt2;
	PMR *psExtMemInt;
	IMG_HANDLE hExtMemInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCBUFFERACQUIRE);

	NEW_HANDLE_BATCH_OR_ERROR(psDCBufferAcquireOUT->eError, psConnection, 1)

	    /* Look up the address from the handle */
	    psDCBufferAcquireOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hBufferInt2,
			       psDCBufferAcquireIN->hBuffer,
			       PVRSRV_HANDLE_TYPE_DC_BUFFER);
	if (psDCBufferAcquireOUT->eError != PVRSRV_OK) {
		goto DCBufferAcquire_exit;
	}

	/* Look up the data from the resman address */
	psDCBufferAcquireOUT->eError =
	    ResManFindPrivateDataByPtr(hBufferInt2, (IMG_VOID **) & psBufferInt,
				       IMG_NULL);
	if (psDCBufferAcquireOUT->eError != PVRSRV_OK) {
		goto DCBufferAcquire_exit;
	}

	psDCBufferAcquireOUT->eError =
	    DCBufferAcquire(psBufferInt, &psExtMemInt);
	/* Exit early if bridged call fails */
	if (psDCBufferAcquireOUT->eError != PVRSRV_OK) {
		goto DCBufferAcquire_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hExtMemInt2 = ResManRegisterRes(psConnection->hResManContext,
					RESMAN_TYPE_PMR, psExtMemInt, 0,
					/* FIXME: how can we avoid this cast? */
					(RESMAN_FREE_FN) & DCBufferRelease);
	if (hExtMemInt2 == IMG_NULL) {
		psDCBufferAcquireOUT->eError =
		    PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto DCBufferAcquire_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
			    &psDCBufferAcquireOUT->hExtMem,
			    (IMG_HANDLE) hExtMemInt2,
			    PVRSRV_HANDLE_TYPE_DEVMEM_MEM_IMPORT,
			    PVRSRV_HANDLE_ALLOC_FLAG_NONE);
	COMMIT_HANDLE_BATCH_OR_ERROR(psDCBufferAcquireOUT->eError,
				     psConnection);

 DCBufferAcquire_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDCBufferRelease(IMG_UINT32 ui32BridgeID,
			    PVRSRV_BRIDGE_IN_DCBUFFERRELEASE *
			    psDCBufferReleaseIN,
			    PVRSRV_BRIDGE_OUT_DCBUFFERRELEASE *
			    psDCBufferReleaseOUT,
			    CONNECTION_DATA * psConnection)
{
	IMG_HANDLE hExtMemInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_DC_DCBUFFERRELEASE);

	/* Look up the address from the handle */
	psDCBufferReleaseOUT->eError =
	    PVRSRVLookupHandle(psConnection->psHandleBase,
			       (IMG_HANDLE *) & hExtMemInt2,
			       psDCBufferReleaseIN->hExtMem,
			       PVRSRV_HANDLE_TYPE_DEVMEM_MEM_IMPORT);
	if (psDCBufferReleaseOUT->eError != PVRSRV_OK) {
		goto DCBufferRelease_exit;
	}

	psDCBufferReleaseOUT->eError = DCBufferReleaseResManProxy(hExtMemInt2);
	/* Exit early if bridged call fails */
	if (psDCBufferReleaseOUT->eError != PVRSRV_OK) {
		goto DCBufferRelease_exit;
	}

	psDCBufferReleaseOUT->eError =
	    PVRSRVReleaseHandle(psConnection->psHandleBase,
				(IMG_HANDLE) psDCBufferReleaseIN->hExtMem,
				PVRSRV_HANDLE_TYPE_DEVMEM_MEM_IMPORT);

 DCBufferRelease_exit:

	return 0;
}

PVRSRV_ERROR RegisterDCFunctions(IMG_VOID);
IMG_VOID UnregisterDCFunctions(IMG_VOID);

/*
 * Register all DC functions with services
 */
PVRSRV_ERROR RegisterDCFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDEVICESQUERYCOUNT,
			      PVRSRVBridgeDCDevicesQueryCount);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDEVICESENUMERATE,
			      PVRSRVBridgeDCDevicesEnumerate);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDEVICEACQUIRE,
			      PVRSRVBridgeDCDeviceAcquire);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDEVICERELEASE,
			      PVRSRVBridgeDCDeviceRelease);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCGETINFO,
			      PVRSRVBridgeDCGetInfo);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCPANELQUERYCOUNT,
			      PVRSRVBridgeDCPanelQueryCount);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCPANELQUERY,
			      PVRSRVBridgeDCPanelQuery);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCFORMATQUERY,
			      PVRSRVBridgeDCFormatQuery);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDIMQUERY,
			      PVRSRVBridgeDCDimQuery);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCSYSTEMBUFFERACQUIRE,
			      PVRSRVBridgeDCSystemBufferAcquire);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCSYSTEMBUFFERRELEASE,
			      PVRSRVBridgeDCSystemBufferRelease);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTCREATE,
			      PVRSRVBridgeDCDisplayContextCreate);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTCONFIGURE,
			      PVRSRVBridgeDCDisplayContextConfigure);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCDISPLAYCONTEXTDESTROY,
			      PVRSRVBridgeDCDisplayContextDestroy);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERALLOC,
			      PVRSRVBridgeDCBufferAlloc);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERIMPORT,
			      PVRSRVBridgeDCBufferImport);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERFREE,
			      PVRSRVBridgeDCBufferFree);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERUNIMPORT,
			      PVRSRVBridgeDCBufferUnimport);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERPIN,
			      PVRSRVBridgeDCBufferPin);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERUNPIN,
			      PVRSRVBridgeDCBufferUnpin);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERACQUIRE,
			      PVRSRVBridgeDCBufferAcquire);
	SetDispatchTableEntry(PVRSRV_BRIDGE_DC_DCBUFFERRELEASE,
			      PVRSRVBridgeDCBufferRelease);

	return PVRSRV_OK;
}

/*
 * Unregister all dc functions with services
 */
IMG_VOID UnregisterDCFunctions(IMG_VOID)
{
}
