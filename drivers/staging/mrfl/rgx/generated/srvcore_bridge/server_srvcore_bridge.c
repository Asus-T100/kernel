/*************************************************************************/ /*!
@File
@Title          Server bridge for srvcore
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for srvcore
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

#include "srvcore.h"
#include "pvrsrv.h"


#include "common_srvcore_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static PVRSRV_ERROR
ReleaseGlobalEventObjectResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

	return eError;
}

static PVRSRV_ERROR
EventObjectCloseResManProxy(IMG_HANDLE hResmanItem)
{
	PVRSRV_ERROR eError;

	eError = ResManFreeResByPtr(hResmanItem);

	/* Freeing a resource should never fail... */
	PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

	return eError;
}


static IMG_INT
PVRSRVBridgeConnect(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_CONNECT *psConnectIN,
					 PVRSRV_BRIDGE_OUT_CONNECT *psConnectOUT,
					 CONNECTION_DATA *psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_CONNECT);



	psConnectOUT->eError =
		PVRSRVConnectKM(
					psConnectIN->ui32Flags,
					psConnectIN->ui32ClientBuildOptions,
					psConnectIN->ui32ClientDDKVersion,
					psConnectIN->ui32ClientDDKBuild);




	return 0;
}

static IMG_INT
PVRSRVBridgeDisconnect(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_DISCONNECT *psDisconnectIN,
					 PVRSRV_BRIDGE_OUT_DISCONNECT *psDisconnectOUT,
					 CONNECTION_DATA *psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psDisconnectIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_DISCONNECT);



	psDisconnectOUT->eError =
		PVRSRVDisconnectKM(
					);




	return 0;
}

static IMG_INT
PVRSRVBridgeEnumerateDevices(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_ENUMERATEDEVICES *psEnumerateDevicesIN,
					 PVRSRV_BRIDGE_OUT_ENUMERATEDEVICES *psEnumerateDevicesOUT,
					 CONNECTION_DATA *psConnection)
{
	PVRSRV_DEVICE_IDENTIFIER *psDeviceIdentifierInt = IMG_NULL;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psEnumerateDevicesIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_ENUMERATEDEVICES);

	psDeviceIdentifierInt = kmalloc(PVRSRV_MAX_DEVICES * sizeof(PVRSRV_DEVICE_IDENTIFIER), GFP_KERNEL);
	if (!psDeviceIdentifierInt)
	{
		psEnumerateDevicesOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto EnumerateDevices_exit;
	}




	psEnumerateDevicesOUT->eError =
		PVRSRVEnumerateDevicesKM(
					&psEnumerateDevicesOUT->ui32NumDevices,
					psDeviceIdentifierInt);


	if (copy_to_user(psEnumerateDevicesOUT->psDeviceIdentifier, psDeviceIdentifierInt,
		(PVRSRV_MAX_DEVICES * sizeof(PVRSRV_DEVICE_IDENTIFIER)))  != 0)
	{
		psEnumerateDevicesOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto EnumerateDevices_exit;
	}


EnumerateDevices_exit:
	if (psDeviceIdentifierInt)
		kfree(psDeviceIdentifierInt);

	return 0;
}

static IMG_INT
PVRSRVBridgeAcquireDeviceData(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_ACQUIREDEVICEDATA *psAcquireDeviceDataIN,
					 PVRSRV_BRIDGE_OUT_ACQUIREDEVICEDATA *psAcquireDeviceDataOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hDevCookieInt;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_ACQUIREDEVICEDATA);


	NEW_HANDLE_BATCH_OR_ERROR(psAcquireDeviceDataOUT->eError, psConnection, 1);


	psAcquireDeviceDataOUT->eError =
		PVRSRVAcquireDeviceDataKM(
					psAcquireDeviceDataIN->ui32DevIndex,
					psAcquireDeviceDataIN->eDeviceType,
					&hDevCookieInt);
	/* Exit early if bridged call fails */
	if(psAcquireDeviceDataOUT->eError != PVRSRV_OK)
	{
		goto AcquireDeviceData_exit;
	}

	PVRSRVAllocHandleNR(psConnection->psHandleBase,
					  &psAcquireDeviceDataOUT->hDevCookie,
					  (IMG_HANDLE) hDevCookieInt,
					  PVRSRV_HANDLE_TYPE_DEV_NODE,
					  PVRSRV_HANDLE_ALLOC_FLAG_SHARED
					  );
	COMMIT_HANDLE_BATCH_OR_ERROR(psAcquireDeviceDataOUT->eError, psConnection);



AcquireDeviceData_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeInitSrvConnect(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_INITSRVCONNECT *psInitSrvConnectIN,
					 PVRSRV_BRIDGE_OUT_INITSRVCONNECT *psInitSrvConnectOUT,
					 CONNECTION_DATA *psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psInitSrvConnectIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_INITSRVCONNECT);



	psInitSrvConnectOUT->eError =
		PVRSRVInitSrvConnectKM(psConnection
					);




	return 0;
}

static IMG_INT
PVRSRVBridgeInitSrvDisconnect(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_INITSRVDISCONNECT *psInitSrvDisconnectIN,
					 PVRSRV_BRIDGE_OUT_INITSRVDISCONNECT *psInitSrvDisconnectOUT,
					 CONNECTION_DATA *psConnection)
{

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_INITSRVDISCONNECT);



	psInitSrvDisconnectOUT->eError =
		PVRSRVInitSrvDisconnectKM(psConnection,
					psInitSrvDisconnectIN->bInitSuccesful,
					psInitSrvDisconnectIN->ui32ClientBuildOptions);




	return 0;
}

static IMG_INT
PVRSRVBridgeAcquireGlobalEventObject(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_ACQUIREGLOBALEVENTOBJECT *psAcquireGlobalEventObjectIN,
					 PVRSRV_BRIDGE_OUT_ACQUIREGLOBALEVENTOBJECT *psAcquireGlobalEventObjectOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hGlobalEventObjectInt;
	IMG_HANDLE hGlobalEventObjectInt2;

	PVR_UNREFERENCED_PARAMETER(psAcquireGlobalEventObjectIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_ACQUIREGLOBALEVENTOBJECT);


	NEW_HANDLE_BATCH_OR_ERROR(psAcquireGlobalEventObjectOUT->eError, psConnection, 1);


	psAcquireGlobalEventObjectOUT->eError =
		AcquireGlobalEventObjectServer(
					&hGlobalEventObjectInt);
	/* Exit early if bridged call fails */
	if(psAcquireGlobalEventObjectOUT->eError != PVRSRV_OK)
	{
		goto AcquireGlobalEventObject_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hGlobalEventObjectInt2 = ResManRegisterRes(psConnection->hResManContext,
												RESMAN_TYPE_SHARED_EVENT_OBJECT,
												hGlobalEventObjectInt,
												/* FIXME: how can we avoid this cast? */
												(RESMAN_FREE_FN)&ReleaseGlobalEventObjectServer);
	if (hGlobalEventObjectInt2 == IMG_NULL)
	{
		psAcquireGlobalEventObjectOUT->eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto AcquireGlobalEventObject_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
					  &psAcquireGlobalEventObjectOUT->hGlobalEventObject,
					  (IMG_HANDLE) hGlobalEventObjectInt2,
					  PVRSRV_HANDLE_TYPE_SHARED_EVENT_OBJECT,
					  PVRSRV_HANDLE_ALLOC_FLAG_SHARED
					  );
	COMMIT_HANDLE_BATCH_OR_ERROR(psAcquireGlobalEventObjectOUT->eError, psConnection);



AcquireGlobalEventObject_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeReleaseGlobalEventObject(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_RELEASEGLOBALEVENTOBJECT *psReleaseGlobalEventObjectIN,
					 PVRSRV_BRIDGE_OUT_RELEASEGLOBALEVENTOBJECT *psReleaseGlobalEventObjectOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hGlobalEventObjectInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_RELEASEGLOBALEVENTOBJECT);


	/* Look up the address from the handle */
	psReleaseGlobalEventObjectOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hGlobalEventObjectInt2,
						   psReleaseGlobalEventObjectIN->hGlobalEventObject,
						   PVRSRV_HANDLE_TYPE_SHARED_EVENT_OBJECT);
	if(psReleaseGlobalEventObjectOUT->eError != PVRSRV_OK)
	{
		goto ReleaseGlobalEventObject_exit;
	}

	psReleaseGlobalEventObjectOUT->eError = ReleaseGlobalEventObjectResManProxy(hGlobalEventObjectInt2);
	/* Exit early if bridged call fails */
	if(psReleaseGlobalEventObjectOUT->eError != PVRSRV_OK)
	{
		goto ReleaseGlobalEventObject_exit;
	}

	psReleaseGlobalEventObjectOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psReleaseGlobalEventObjectIN->hGlobalEventObject,
					PVRSRV_HANDLE_TYPE_SHARED_EVENT_OBJECT);


ReleaseGlobalEventObject_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeEventObjectOpen(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_EVENTOBJECTOPEN *psEventObjectOpenIN,
					 PVRSRV_BRIDGE_OUT_EVENTOBJECTOPEN *psEventObjectOpenOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hEventObjectInt;
	IMG_HANDLE hEventObjectInt2;
	IMG_HANDLE hOSEventInt;
	IMG_HANDLE hOSEventInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTOPEN);


	NEW_HANDLE_BATCH_OR_ERROR(psEventObjectOpenOUT->eError, psConnection, 1);

	/* Look up the address from the handle */
	psEventObjectOpenOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hEventObjectInt2,
						   psEventObjectOpenIN->hEventObject,
						   PVRSRV_HANDLE_TYPE_SHARED_EVENT_OBJECT);
	if(psEventObjectOpenOUT->eError != PVRSRV_OK)
	{
		goto EventObjectOpen_exit;
	}

	/* Look up the data from the resman address */
	psEventObjectOpenOUT->eError = ResManFindPrivateDataByPtr(hEventObjectInt2, (IMG_VOID **) &hEventObjectInt);
	if(psEventObjectOpenOUT->eError != PVRSRV_OK)
	{
		goto EventObjectOpen_exit;
	}

	psEventObjectOpenOUT->eError =
		OSEventObjectOpen(
					hEventObjectInt,
					&hOSEventInt);
	/* Exit early if bridged call fails */
	if(psEventObjectOpenOUT->eError != PVRSRV_OK)
	{
		goto EventObjectOpen_exit;
	}

	/* Create a resman item and overwrite the handle with it */
	hOSEventInt2 = ResManRegisterRes(psConnection->hResManContext,
												RESMAN_TYPE_EVENT_OBJECT,
												hOSEventInt,
												/* FIXME: how can we avoid this cast? */
												(RESMAN_FREE_FN)&OSEventObjectClose);
	if (hOSEventInt2 == IMG_NULL)
	{
		psEventObjectOpenOUT->eError = PVRSRV_ERROR_UNABLE_TO_REGISTER_RESOURCE;
		goto EventObjectOpen_exit;
	}
	PVRSRVAllocHandleNR(psConnection->psHandleBase,
					  &psEventObjectOpenOUT->hOSEvent,
					  (IMG_HANDLE) hOSEventInt2,
					  PVRSRV_HANDLE_TYPE_EVENT_OBJECT_CONNECT,
					  PVRSRV_HANDLE_ALLOC_FLAG_MULTI
					  );
	COMMIT_HANDLE_BATCH_OR_ERROR(psEventObjectOpenOUT->eError, psConnection);



EventObjectOpen_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeEventObjectWait(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_EVENTOBJECTWAIT *psEventObjectWaitIN,
					 PVRSRV_BRIDGE_OUT_EVENTOBJECTWAIT *psEventObjectWaitOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hOSEventKMInt;
	IMG_HANDLE hOSEventKMInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTWAIT);


	/* Look up the address from the handle */
	psEventObjectWaitOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hOSEventKMInt2,
						   psEventObjectWaitIN->hOSEventKM,
						   PVRSRV_HANDLE_TYPE_EVENT_OBJECT_CONNECT);
	if(psEventObjectWaitOUT->eError != PVRSRV_OK)
	{
		goto EventObjectWait_exit;
	}

	/* Look up the data from the resman address */
	psEventObjectWaitOUT->eError = ResManFindPrivateDataByPtr(hOSEventKMInt2, (IMG_VOID **) &hOSEventKMInt);
	if(psEventObjectWaitOUT->eError != PVRSRV_OK)
	{
		goto EventObjectWait_exit;
	}

	psEventObjectWaitOUT->eError =
		OSEventObjectWait(
					hOSEventKMInt);



EventObjectWait_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeEventObjectClose(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_EVENTOBJECTCLOSE *psEventObjectCloseIN,
					 PVRSRV_BRIDGE_OUT_EVENTOBJECTCLOSE *psEventObjectCloseOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hOSEventKMInt2;

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTCLOSE);


	/* Look up the address from the handle */
	psEventObjectCloseOUT->eError =
		PVRSRVLookupHandle(psConnection->psHandleBase,
						   (IMG_HANDLE *) &hOSEventKMInt2,
						   psEventObjectCloseIN->hOSEventKM,
						   PVRSRV_HANDLE_TYPE_EVENT_OBJECT_CONNECT);
	if(psEventObjectCloseOUT->eError != PVRSRV_OK)
	{
		goto EventObjectClose_exit;
	}

	psEventObjectCloseOUT->eError = EventObjectCloseResManProxy(hOSEventKMInt2);
	/* Exit early if bridged call fails */
	if(psEventObjectCloseOUT->eError != PVRSRV_OK)
	{
		goto EventObjectClose_exit;
	}

	psEventObjectCloseOUT->eError =
		PVRSRVReleaseHandle(psConnection->psHandleBase,
					(IMG_HANDLE) psEventObjectCloseIN->hOSEventKM,
					PVRSRV_HANDLE_TYPE_EVENT_OBJECT_CONNECT);


EventObjectClose_exit:

	return 0;
}

static IMG_INT
PVRSRVBridgeDumpDebugInfo(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_DUMPDEBUGINFO *psDumpDebugInfoIN,
					 PVRSRV_BRIDGE_OUT_DUMPDEBUGINFO *psDumpDebugInfoOUT,
					 CONNECTION_DATA *psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psDumpDebugInfoIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_DUMPDEBUGINFO);



	psDumpDebugInfoOUT->eError =
		PVRSRVDumpDebugInfoKM(
					);




	return 0;
}

static IMG_INT
PVRSRVBridgeHWOpTimeout(IMG_UINT32 ui32BridgeID,
					 PVRSRV_BRIDGE_IN_HWOPTIMEOUT *psHWOpTimeoutIN,
					 PVRSRV_BRIDGE_OUT_HWOPTIMEOUT *psHWOpTimeoutOUT,
					 CONNECTION_DATA *psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psHWOpTimeoutIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID, PVRSRV_BRIDGE_SRVCORE_HWOPTIMEOUT);



	psHWOpTimeoutOUT->eError =
		PVRSRVHWOpTimeoutKM(
					);




	return 0;
}


PVRSRV_ERROR RegisterSRVCOREFunctions(IMG_VOID);
IMG_VOID UnregisterSRVCOREFunctions(IMG_VOID);

/*
 * Register all SRVCORE functions with services
 */
PVRSRV_ERROR RegisterSRVCOREFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_CONNECT, PVRSRVBridgeConnect);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_DISCONNECT, PVRSRVBridgeDisconnect);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_ENUMERATEDEVICES, PVRSRVBridgeEnumerateDevices);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_ACQUIREDEVICEDATA, PVRSRVBridgeAcquireDeviceData);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_INITSRVCONNECT, PVRSRVBridgeInitSrvConnect);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_INITSRVDISCONNECT, PVRSRVBridgeInitSrvDisconnect);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_ACQUIREGLOBALEVENTOBJECT, PVRSRVBridgeAcquireGlobalEventObject);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_RELEASEGLOBALEVENTOBJECT, PVRSRVBridgeReleaseGlobalEventObject);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTOPEN, PVRSRVBridgeEventObjectOpen);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTWAIT, PVRSRVBridgeEventObjectWait);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_EVENTOBJECTCLOSE, PVRSRVBridgeEventObjectClose);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_DUMPDEBUGINFO, PVRSRVBridgeDumpDebugInfo);
	SetDispatchTableEntry(PVRSRV_BRIDGE_SRVCORE_HWOPTIMEOUT, PVRSRVBridgeHWOpTimeout);

	return PVRSRV_OK;
}

/*
 * Unregister all srvcore functions with services
 */
IMG_VOID UnregisterSRVCOREFunctions(IMG_VOID)
{
}
