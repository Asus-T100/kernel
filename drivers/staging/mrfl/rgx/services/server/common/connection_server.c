									    /*************************************************************************//*!
									       @File
									       @Title          Server side connection management
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Handles connections coming from the client and the management
									       connection based information
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "resman.h"
#include "handle.h"
#include "connection_server.h"
#include "osconnection_server.h"
#include "allocmem.h"
#include "pvr_debug.h"

/*!
******************************************************************************

 @Function	FreeConnectionData

 @Description	Free a connection data area

 @Input		psConnection - pointer to connection data area

 @Return	Error code, or PVRSRV_OK

******************************************************************************/
static PVRSRV_ERROR FreeConnectionData(CONNECTION_DATA * psConnection)
{
	PVRSRV_ERROR eError;

	PVR_ASSERT(psConnection != IMG_NULL);

	if (psConnection == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "FreeConnectionData: invalid parameter"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* Free handle base for this connection */
	if (psConnection->psHandleBase != IMG_NULL) {
		eError = PVRSRVFreeHandleBase(psConnection->psHandleBase);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "FreeConnectionData: Couldn't free handle base for connection (%d)",
				 eError));
			return eError;
		}
	}

	/* Call environment specific per process deinit function */
	eError = OSConnectionPrivateDataDeInit(psConnection->hOsPrivateData);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "FreeConnectionData: OSConnectionPrivateDataDeInit failed (%d)",
			 eError));
		return eError;
	}

	OSFreeMem(psConnection);

	return PVRSRV_OK;
}

/* PVRSRVConnectionConnect*/
PVRSRV_ERROR PVRSRVConnectionConnect(IMG_PVOID * ppvPrivData,
				     IMG_PVOID pvOSData)
{
	CONNECTION_DATA *psConnection;
	PVRSRV_ERROR eError = PVRSRV_OK;

	/* Allocate per-process data area */
	psConnection = OSAllocMem(sizeof(*psConnection));
	if (psConnection == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVConnectionConnect: Couldn't allocate per-process data (%d)",
			 eError));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	OSMemSet(psConnection, 0, sizeof(*psConnection));

	/* Call environment specific per process init function */
	eError =
	    OSConnectionPrivateDataInit(&psConnection->hOsPrivateData,
					pvOSData);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVConnectionConnect: OSConnectionPrivateDataInit failed (%d)",
			 eError));
		goto failure;
	}

	/* Allocate handle base for this process */
	eError = PVRSRVAllocHandleBase(&psConnection->psHandleBase);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVConnectionConnect: Couldn't allocate handle base for process (%d)",
			 eError));
		goto failure;
	}

	/* Create a resource manager context for the process */
	eError = PVRSRVResManConnect(&psConnection->hResManContext);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVConnectionConnect: Couldn't register with the resource manager"));
		goto failure;
	}

	*ppvPrivData = psConnection;

	return eError;

 failure:
	(IMG_VOID) FreeConnectionData(psConnection);
	return eError;
}

/* PVRSRVConnectionDisconnect */
IMG_VOID PVRSRVConnectionDisconnect(IMG_PVOID pvDataPtr)
{
	PVRSRV_ERROR eError;
	CONNECTION_DATA *psConnection = pvDataPtr;

	/* Close the Resource Manager connection */
	PVRSRVResManDisconnect(psConnection->hResManContext, IMG_FALSE);

	/* Free the connection data */
	eError = FreeConnectionData(psConnection);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVConnectionDisconnect: Error freeing per-process data"));
	}

	/* FIXME: Is this still required? */
	eError = PVRSRVPurgeHandles(KERNEL_HANDLE_BASE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVConnectionDisconnect: Purge of global handle pool failed (%d)",
			 eError));
	}
}

/* PVRSRVConnectionInit */
PVRSRV_ERROR PVRSRVConnectionInit(IMG_VOID)
{
	return PVRSRV_OK;
}

/* PVRSRVConnectionDeInit */
PVRSRV_ERROR PVRSRVConnectionDeInit(IMG_VOID)
{
	return PVRSRV_OK;
}
