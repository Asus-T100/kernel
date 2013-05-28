/*************************************************************************/ /*!
@File
@Title          core services functions
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Main APIs for core services functions
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

#include "handle.h"
#include "connection_server.h"
#include "pdump_km.h"
#include "ra.h"
#include "allocmem.h"
#include "pmr.h"
#include "dc_server.h"
#include "pvrsrv.h"
#include "pvrsrv_device.h"
#include "pvr_debug.h"
#include "sync.h"

#include "pvrversion.h"

#include "lists.h"
#include "dllist.h"
#include "syscommon.h"

#include "physmem_lma.h"
#include "physmem_osmem.h"

#include "tlintern.h"

#if defined (SUPPORT_RGX)
#include "rgxinit.h"
#endif

static PVRSRV_DATA	*gpsPVRSRVData = IMG_NULL;

static PVRSRV_SYSTEM_CONFIG *gpsSysConfig = IMG_NULL;

typedef PVRSRV_ERROR (*PFN_REGISTER_DEVICE)(PVRSRV_DEVICE_NODE *psDeviceNode);
typedef PVRSRV_ERROR (*PFN_UNREGISTER_DEVICE)(PVRSRV_DEVICE_NODE *psDeviceNode);

static PFN_REGISTER_DEVICE sRegisterDevice[PVRSRV_DEVICE_TYPE_LAST + 1];
static PFN_UNREGISTER_DEVICE sUnregisterDevice[PVRSRV_DEVICE_TYPE_LAST + 1];

static PVRSRV_ERROR IMG_CALLCONV PVRSRVRegisterDevice(PVRSRV_DEVICE_CONFIG *psDevConfig);
static PVRSRV_ERROR IMG_CALLCONV PVRSRVUnregisterDevice(PVRSRV_DEVICE_NODE *psDeviceNode);

IMG_UINT32	g_ui32InitFlags;

/* mark which parts of Services were initialised */
#define		INIT_DATA_ENABLE_PDUMPINIT	0x1U
#define		INIT_GLOBAL_RESMAN 0x2U

/* Head of the list of callbacks called when Cmd complete happens */
static DLLIST_NODE sCmdCompNotifyHead;
static POSWR_LOCK hNotifyLock = IMG_NULL;

/*!
******************************************************************************

 @Function	AllocateDeviceID

 @Description

 allocates a device id from the pool of valid ids

 @input psPVRSRVData :	Services private data

 @input pui32DevID : device id to return

 @Return device id

******************************************************************************/
static PVRSRV_ERROR AllocateDeviceID(PVRSRV_DATA *psPVRSRVData, IMG_UINT32 *pui32DevID)
{
	SYS_DEVICE_ID* psDeviceWalker;
	SYS_DEVICE_ID* psDeviceEnd;

	psDeviceWalker = &psPVRSRVData->sDeviceID[0];
	psDeviceEnd = psDeviceWalker + SYS_DEVICE_COUNT;

	/* find a free ID */
	while (psDeviceWalker < psDeviceEnd)
	{
		if (!psDeviceWalker->bInUse)
		{
			psDeviceWalker->bInUse = IMG_TRUE;
			*pui32DevID = psDeviceWalker->uiID;

			return PVRSRV_OK;
		}
		psDeviceWalker++;
	}

	PVR_DPF((PVR_DBG_ERROR,"AllocateDeviceID: No free and valid device IDs available!"));

	/* Should never get here: sDeviceID[] may have been setup too small */
	PVR_ASSERT(psDeviceWalker < psDeviceEnd);

	return PVRSRV_ERROR_NO_FREE_DEVICEIDS_AVAILABLE;
}


/*!
******************************************************************************

 @Function	FreeDeviceID

 @Description

 frees a device id from the pool of valid ids

 @input psPVRSRVData :	Services private data

 @input ui32DevID : device id to free

 @Return device id

******************************************************************************/
static PVRSRV_ERROR FreeDeviceID(PVRSRV_DATA *psPVRSRVData, IMG_UINT32 ui32DevID)
{
	SYS_DEVICE_ID* psDeviceWalker;
	SYS_DEVICE_ID* psDeviceEnd;

	psDeviceWalker = &psPVRSRVData->sDeviceID[0];
	psDeviceEnd = psDeviceWalker + SYS_DEVICE_COUNT;

	/* find the ID to free */
	while (psDeviceWalker < psDeviceEnd)
	{
		/* if matching id and in use, free */
		if	(
				(psDeviceWalker->uiID == ui32DevID) &&
				(psDeviceWalker->bInUse)
			)
		{
			psDeviceWalker->bInUse = IMG_FALSE;
			return PVRSRV_OK;
		}
		psDeviceWalker++;
	}

	PVR_DPF((PVR_DBG_ERROR,"FreeDeviceID: no matching dev ID that is in use!"));

	/* should never get here */
	PVR_ASSERT(psDeviceWalker < psDeviceEnd);

	return PVRSRV_ERROR_INVALID_DEVICEID;
}


/*!
******************************************************************************
 @Function	PVRSRVEnumerateDCKM_ForEachVaCb

 @Description

 Enumerates the device node (if is of the same class as given).

 @Input psDeviceNode	- The device node to be enumerated
 		va				- variable arguments list, with:
							pui32DevCount	- The device count pointer (to be increased)
							ppui32DevID		- The pointer to the device IDs pointer (to be updated and increased)
******************************************************************************/
static IMG_VOID PVRSRVEnumerateDevicesKM_ForEachVaCb(PVRSRV_DEVICE_NODE *psDeviceNode, va_list va)
{
	IMG_UINT *pui32DevCount;
	PVRSRV_DEVICE_IDENTIFIER **ppsDevIdList;

	pui32DevCount = va_arg(va, IMG_UINT*);
	ppsDevIdList = va_arg(va, PVRSRV_DEVICE_IDENTIFIER**);

	if (psDeviceNode->sDevId.eDeviceType != PVRSRV_DEVICE_TYPE_EXT)
	{
		*(*ppsDevIdList) = psDeviceNode->sDevId;
		(*ppsDevIdList)++;
		(*pui32DevCount)++;
	}
}



/*!
******************************************************************************

 @Function PVRSRVEnumerateDevicesKM

 @Description
 This function will enumerate all the devices supported by the
 PowerVR services within the target system.
 The function returns a list of the device ID strcutres stored either in
 the services or constructed in the user mode glue component in certain
 environments. The number of devices in the list is also returned.

 In a binary layered component which does not support dynamic runtime selection,
 the glue code should compile to return the supported devices statically,
 e.g. multiple instances of the same device if multiple devices are supported,
 or the target combination of Rogue and display device.

 In the case of an environment (for instance) where one Rogue may connect to two
 display devices this code would enumerate all three devices and even
 non-dynamic Rogue selection code should retain the facility to parse the list
 to find the index of the Rogue device

 @output pui32NumDevices :	On success, contains the number of devices present
 							in the system

 @output psDevIdList	 :	Pointer to called supplied buffer to receive the
 							list of PVRSRV_DEVICE_IDENTIFIER

 @return PVRSRV_ERROR  :	PVRSRV_NO_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVEnumerateDevicesKM(IMG_UINT32 *pui32NumDevices,
											 	   PVRSRV_DEVICE_IDENTIFIER *psDevIdList)
{
	PVRSRV_DATA			*psPVRSRVData = PVRSRVGetPVRSRVData();
	IMG_UINT32 			i;

	if (!pui32NumDevices || !psDevIdList)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVEnumerateDevicesKM: Invalid params"));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/*
		setup input buffer to be `empty'
	*/
	for (i=0; i<PVRSRV_MAX_DEVICES; i++)
	{
		psDevIdList[i].eDeviceType = PVRSRV_DEVICE_TYPE_UNKNOWN;
	}

	/* and zero device count */
	*pui32NumDevices = 0;

	/*
		Search through the device list for services managed devices
		return id info for each device and the number of devices
		available
	*/
	List_PVRSRV_DEVICE_NODE_ForEach_va(psPVRSRVData->psDeviceNodeList,
									   &PVRSRVEnumerateDevicesKM_ForEachVaCb,
									   pui32NumDevices,
									   &psDevIdList);


	return PVRSRV_OK;
}

static IMG_VOID CleanupThread(IMG_PVOID pvData)
{
	PVRSRV_DATA *psPVRSRVData = pvData;
	IMG_HANDLE hOSEvent;

	OSAcquireBridgeLock();

	/*
		While the driver is in a good state and is not being unloaded
		try to free any deferred items when an event happens
	*/
	while ((psPVRSRVData->eServicesState == PVRSRV_SERVICES_STATE_OK) && 
			(!psPVRSRVData->bUnload))
	{
		/* Open the global event object so we can listen on it */
		OSEventObjectOpen(psPVRSRVData->hGlobalEventObject, &hOSEvent);

		/* We don't want to hold the lock while waiting */
		OSSetReleasePVRLock();

		/* Only call into resman if something has happened */
		OSEventObjectWait(hOSEvent);

		/*
			We have to close the event object so we don't get the event from
			the firmware that was issued by the cleanup request
		*/
		OSEventObjectClose(hOSEvent);
		PVRSRVResManFlushDeferContext(psPVRSRVData->hResManDeferContext);
	}

	/* Release our hold of the lock */
	OSReleaseBridgeLock();
}

PVRSRV_DATA *PVRSRVGetPVRSRVData()
{
	return gpsPVRSRVData;
}


PVRSRV_ERROR IMG_CALLCONV PVRSRVInit(IMG_VOID)
{
	PVRSRV_ERROR	eError;
	PVRSRV_SYSTEM_CONFIG *psSysConfig;
	IMG_UINT32 i;

#if defined (SUPPORT_RGX)
	/* FIXME find a way to do this without device-specific code here */
	sRegisterDevice[PVRSRV_DEVICE_TYPE_RGX] = RGXRegisterDevice;
#endif

	eError = PhysHeapInit();
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}

	/* Get the system config */
	eError = SysCreateConfigData(&psSysConfig);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	/* Save to global pointer for later */
	gpsSysConfig = psSysConfig;

    /*
     * Allocate the device-independent data
     */
    gpsPVRSRVData = OSAllocMem(sizeof(*gpsPVRSRVData));
    if (gpsPVRSRVData == IMG_NULL)
    {
        return PVRSRV_ERROR_OUT_OF_MEMORY;
    }
    OSMemSet(gpsPVRSRVData, 0, sizeof(*gpsPVRSRVData));
	gpsPVRSRVData->ui32NumDevices = psSysConfig->uiDeviceCount;

	for (i=0;i<SYS_DEVICE_COUNT;i++)
	{
		gpsPVRSRVData->sDeviceID[i].uiID = i;
		gpsPVRSRVData->sDeviceID[i].bInUse = IMG_FALSE;
	}

	/*
	 * Register the physical memory heaps
	 */
	PVR_ASSERT(psSysConfig->ui32PhysHeapCount <= SYS_PHYS_HEAP_COUNT);
	for (i=0;i<psSysConfig->ui32PhysHeapCount;i++)
	{
		eError = PhysHeapRegister(&psSysConfig->pasPhysHeaps[i],
								  &gpsPVRSRVData->apsRegisteredPhysHeaps[i]);
		if (eError != PVRSRV_OK)
		{
			goto Error;
		}
		gpsPVRSRVData->ui32RegisteredPhysHeaps++;
	}

	/* Init any OS specific's */
	eError = OSInitEnvData();
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}

	/* Initialise Resource Manager */
	eError = ResManInit();
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}

	eError = PVRSRVConnectionInit();
	if(eError != PVRSRV_OK)
	{
		goto Error;
	}

    eError = PMRInit();
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}

#if !defined(UNDER_WDDM)
	eError = DCInit();
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}
#endif

	/* Initialise handles */
	eError = PVRSRVHandleInit();
	if(eError != PVRSRV_OK)
	{
		goto Error;
	}

	/* Initialise Power Manager Lock */
	eError = OSLockCreate(&gpsPVRSRVData->hPowerLock, LOCK_TYPE_PASSIVE);
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}

	/* Initialise system power state */
	gpsPVRSRVData->eCurrentPowerState = PVRSRV_SYS_POWER_STATE_ON;
	gpsPVRSRVData->eFailedPowerState = PVRSRV_SYS_POWER_STATE_Unspecified;

	/* Initialise overall system state */
	gpsPVRSRVData->eServicesState = PVRSRV_SERVICES_STATE_OK;

	/* Create an event object */
	eError = OSEventObjectCreate("PVRSRV_GLOBAL_EVENTOBJECT", &gpsPVRSRVData->hGlobalEventObject);
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}

	/* initialise list of command complete notifiers */
	dllist_init(&sCmdCompNotifyHead);

	/* Create a lock of the list notifiers */
	eError = OSWRLockCreate(&hNotifyLock);
	if (eError != PVRSRV_OK)
	{
		goto Error;
	}

	/* Initialise pdump */
	eError = PDUMPINIT();
	if(eError != PVRSRV_OK)
	{
		goto Error;
	}

	g_ui32InitFlags |= INIT_DATA_ENABLE_PDUMPINIT;

	/* Register all the system devices */
	for (i=0;i<psSysConfig->uiDeviceCount;i++)
	{
		if (PVRSRVRegisterDevice(&psSysConfig->pasDevices[i]) != PVRSRV_OK)
		{
			/* FIXME: We should unregister devices if we fail */
			return eError;
		}
	}

	eError = PVRSRVResManCreateDeferContext(&gpsPVRSRVData->hResManDeferContext);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVInit: Failed PVRSRVResManConnect call"));
		goto Error;
	}
	g_ui32InitFlags |= INIT_GLOBAL_RESMAN;

	/* Create a thread which is used to do the deferred cleanup */
	eError = OSThreadCreate(&gpsPVRSRVData->hCleanupThread,
							"pvr_defer_free",
							CleanupThread,
							gpsPVRSRVData);

	return eError;

Error:
	PVRSRVDeInit();
	return eError;
}


IMG_VOID IMG_CALLCONV PVRSRVDeInit(IMG_VOID)
{
	PVRSRV_DATA		*psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_ERROR	eError;
	IMG_UINT32		i;

	if (gpsPVRSRVData == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeInit failed - invalid gpsPVRSRVData"));
		return;
	}

#if defined (SUPPORT_RGX)
	sUnregisterDevice[PVRSRV_DEVICE_TYPE_RGX] = DevDeInitRGX;
#endif

	/* Destroy the cleanup thread */
	psPVRSRVData->bUnload = IMG_TRUE;
	OSEventObjectSignal(psPVRSRVData->hGlobalEventObject);
	OSThreadDestory(gpsPVRSRVData->hCleanupThread);

	if (g_ui32InitFlags & INIT_GLOBAL_RESMAN)
	{
		PVRSRVResManDestroyDeferContext(gpsPVRSRVData->hResManDeferContext);
	}

	/* Unregister all the system devices */
	for (i=0;i<psPVRSRVData->ui32RegisteredDevices;i++)
	{
		PVRSRV_DEVICE_NODE *psDeviceNode = psPVRSRVData->apsRegisteredDevNodes[i];

		/* set device state */
		psDeviceNode->eDevState = PVRSVR_DEVICE_STATE_DEINIT;

		/* Counter part to what gets done in PVRSRVFinaliseSystem */
		if (psDeviceNode->hSyncPrimContext != IMG_NULL)
		{
			if (psDeviceNode->psSyncPrim != IMG_NULL)
			{
				/* Free general pupose sync primitive */
				SyncPrimFree(psDeviceNode->psSyncPrim);
				psDeviceNode->psSyncPrim = IMG_NULL;
			}
			if (psDeviceNode->psSyncPrimPreKick != IMG_NULL)
			{
				/* Free PreKick sync primitive */
				SyncPrimFree(psDeviceNode->psSyncPrimPreKick);
				psDeviceNode->psSyncPrimPreKick = IMG_NULL;
			}
			if (psDeviceNode->psZSBufferPopulateSyncPrim != IMG_NULL)
			{
				/* Free ZS Buffer populate sync primitive */
				SyncPrimFree(psDeviceNode->psZSBufferPopulateSyncPrim);
				psDeviceNode->psZSBufferPopulateSyncPrim = IMG_NULL;
			}
			if (psDeviceNode->psZSBufferUnPopulateSyncPrim != IMG_NULL)
			{
				/* Free ZS Buffer unpopulate sync primitive */
				SyncPrimFree(psDeviceNode->psZSBufferUnPopulateSyncPrim);
				psDeviceNode->psZSBufferUnPopulateSyncPrim = IMG_NULL;
			}
			if (psDeviceNode->psGrowSyncPrim != IMG_NULL)
			{
				/* Free ZS Buffer populate sync primitive */
				SyncPrimFree(psDeviceNode->psGrowSyncPrim);
				psDeviceNode->psGrowSyncPrim = IMG_NULL;
			}
			if (psDeviceNode->psShrinkSyncPrim != IMG_NULL)
			{
				/* Free ZS Buffer unpopulate sync primitive */
				SyncPrimFree(psDeviceNode->psShrinkSyncPrim);
				psDeviceNode->psShrinkSyncPrim = IMG_NULL;
			}

			OSLockDestroy(psDeviceNode->hLockZSBuffer);
			OSLockDestroy(psDeviceNode->hLockFreeList);

			SyncPrimContextDestroy(psDeviceNode->hSyncPrimContext);
			psDeviceNode->hSyncPrimContext = IMG_NULL;
		}

		PVRSRVUnregisterDevice(psDeviceNode);
		psPVRSRVData->apsRegisteredDevNodes[i] = IMG_NULL;
	}
	SysDestroyConfigData(gpsSysConfig);

	if (hNotifyLock)
	{
		OSWRLockDestroy(hNotifyLock);
	}

	/* deinitialise pdump */
	if ((g_ui32InitFlags & INIT_DATA_ENABLE_PDUMPINIT) > 0)
	{
		PDUMPDEINIT();
	}
	
	/* destroy event object */
	if (gpsPVRSRVData->hGlobalEventObject)
	{
		OSEventObjectDestroy(gpsPVRSRVData->hGlobalEventObject);
		gpsPVRSRVData->hGlobalEventObject = IMG_NULL;
	}

	/* Check there is no notify function */
	if (!dllist_is_empty(&sCmdCompNotifyHead))
	{
		PDLLIST_NODE psNode = dllist_get_next_node(&sCmdCompNotifyHead);

		/* some device did not unregistered properly */
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeInit: Notify list for cmd complete is not empty!!"));

		/* clean the nodes anyway */
		while (psNode != IMG_NULL)
		{
			PVRSRV_CMDCOMP_NOTIFY	*psNotify;

			dllist_remove_node(psNode);
			
			psNotify = IMG_CONTAINER_OF(psNode, PVRSRV_CMDCOMP_NOTIFY, sListNode);
			OSFreeMem(psNotify);

			psNode = dllist_get_next_node(&sCmdCompNotifyHead);
		}
	}

	OSLockDestroy(gpsPVRSRVData->hPowerLock);

	eError = PVRSRVHandleDeInit();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeInit: PVRSRVHandleDeInit failed"));
	}

#if !defined(UNDER_WDDM)
	eError = DCDeInit();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeInit: DCInit() failed"));
	}
#endif


    eError = PMRDeInit();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeInit: PMRDeInit() failed"));
	}

	eError = PVRSRVConnectionDeInit();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeInit: PVRSRVConnectionDataDeInit failed"));
	}

	ResManDeInit();

	for (i=0;i<gpsPVRSRVData->ui32RegisteredPhysHeaps;i++)
	{
		PhysHeapUnregister(gpsPVRSRVData->apsRegisteredPhysHeaps[i]);
	}
	eError = PhysHeapDeinit();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeInit: PhysHeapDeinit() failed"));
	}

	OSFreeMem(gpsPVRSRVData);
	gpsPVRSRVData = IMG_NULL;
}

PVRSRV_ERROR LMA_MMUPxAlloc(PVRSRV_DEVICE_NODE *psDevNode, IMG_SIZE_T uiSize,
							Px_HANDLE *psMemHandle, IMG_DEV_PHYADDR *psDevPAddr)
{
	IMG_BOOL bSuccess;
	RA_BASE_T uiCardAddr;
	RA_LENGTH_T uiActualSize;

	PVR_ASSERT((uiSize & OSGetPageMask()) == 0);

	bSuccess = RA_Alloc(psDevNode->psLocalDevMemArena, 
						uiSize,
						0,					/* No flags */
						OSGetPageSize(),
						&uiCardAddr,
						&uiActualSize,
						IMG_NULL);			/* No private handle */

	PVR_ASSERT(uiSize == uiActualSize);

	psMemHandle->u.ui64Handle = uiCardAddr;
	psDevPAddr->uiAddr = (IMG_UINT64) uiCardAddr;

	if (bSuccess)
		return PVRSRV_OK;
	else
		return PVRSRV_ERROR_OUT_OF_MEMORY;
}

IMG_VOID LMA_MMUPxFree(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle)
{
	RA_BASE_T uiCardAddr = (RA_BASE_T) psMemHandle->u.ui64Handle;

	RA_Free(psDevNode->psLocalDevMemArena, uiCardAddr);
}

PVRSRV_ERROR LMA_MMUPxMap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle,
							IMG_SIZE_T uiSize, IMG_DEV_PHYADDR *psDevPAddr,
							IMG_VOID **pvPtr)
{
	IMG_CPU_PHYADDR sCpuPAddr;
	PVR_UNREFERENCED_PARAMETER(psMemHandle);
	PVR_UNREFERENCED_PARAMETER(uiSize);

	PhysHeapDevPAddrToCpuPAddr(psDevNode->psPhysHeap, &sCpuPAddr, psDevPAddr);
	*pvPtr = OSMapPhysToLin(sCpuPAddr,
							OSGetPageSize(),
							0);
	if (pvPtr == IMG_NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	else
	{
		return PVRSRV_OK;
	}
}

IMG_VOID LMA_MMUPxUnmap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle,
						IMG_VOID *pvPtr)
{
	PVR_UNREFERENCED_PARAMETER(psMemHandle);
	PVR_UNREFERENCED_PARAMETER(psDevNode);

	OSUnMapPhysToLin(pvPtr, OSGetPageSize(), 0);
}

/*!
******************************************************************************

 @Function	PVRSRVRegisterDevice

 @Description

 registers a device with the system

 @Input	   psDevConfig			: Device configuration structure

 @Return   PVRSRV_ERROR  :

******************************************************************************/
static PVRSRV_ERROR IMG_CALLCONV PVRSRVRegisterDevice(PVRSRV_DEVICE_CONFIG *psDevConfig)
{
	PVRSRV_DATA			*psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_ERROR		eError;
	PVRSRV_DEVICE_NODE	*psDeviceNode;

	/* Allocate device node */
	psDeviceNode = OSAllocMem(sizeof(PVRSRV_DEVICE_NODE));
	if (psDeviceNode == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRegisterDevice : Failed to alloc memory for psDeviceNode"));
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}
	OSMemSet(psDeviceNode, 0, sizeof(PVRSRV_DEVICE_NODE));

	/* set device state */
	psDeviceNode->eDevState = PVRSVR_DEVICE_STATE_INIT;

	eError = PhysHeapAcquire(psDevConfig->ui32PhysHeapID, &psDeviceNode->psPhysHeap);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRegisterDevice : Failed to acquire physcial memory heap"));
		goto e1;
	}

	/* Do we have card memory? If so create an RA to manage it */
	if (PhysHeapGetType(psDeviceNode->psPhysHeap) == PHYS_HEAP_TYPE_LMA)
	{
		RA_LENGTH_T uSize;
		IMG_CPU_PHYADDR sCpuPAddr;
		IMG_UINT64 ui64Size;

		eError = PhysHeapGetAddress(psDeviceNode->psPhysHeap, &sCpuPAddr);
		if (eError != PVRSRV_OK)
		{
			/* We can only get here if there is a bug in this module */
			PVR_ASSERT(IMG_FALSE);
			return eError;
		}

		eError = PhysHeapGetSize(psDeviceNode->psPhysHeap, &ui64Size);
		if (eError != PVRSRV_OK)
		{
			/* We can only get here if there is a bug in this module */
			PVR_ASSERT(IMG_FALSE);
			return eError;
		}


		PVR_DPF((PVR_DBG_MESSAGE, "Creating RA for card memory 0x%016llx-0x%016llx",
				 (IMG_UINT64) sCpuPAddr.uiAddr, sCpuPAddr.uiAddr + ui64Size));

		OSSNPrintf(psDeviceNode->szRAName, sizeof(psDeviceNode->szRAName), 
											"%s card mem",
											psDevConfig->pszName);

		uSize = (RA_LENGTH_T) ui64Size;
		PVR_ASSERT(uSize == ui64Size);

		psDeviceNode->psLocalDevMemArena =
			RA_Create(psDeviceNode->szRAName,
						0,
						uSize,
						0,					/* No flags */
						IMG_NULL,			/* No private data */
						OSGetPageSize(),	/* Use host page size, keeps things simple */
						IMG_NULL,			/* No Import */
						IMG_NULL,			/* No free import */
						IMG_NULL);			/* No import handle */

		if (psDeviceNode->psLocalDevMemArena == IMG_NULL)
		{
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto e2;
		}
		psDeviceNode->pfnMMUPxAlloc = LMA_MMUPxAlloc;
		psDeviceNode->pfnMMUPxFree = LMA_MMUPxFree;
		psDeviceNode->pfnMMUPxMap = LMA_MMUPxMap;
		psDeviceNode->pfnMMUPxUnmap = LMA_MMUPxUnmap;
		psDeviceNode->uiMMUPxLog2AllocGran = OSGetPageShift();
		psDeviceNode->pfnCreateRamBackedPMR = PhysmemNewLocalRamBackedPMR;
		psDeviceNode->ui32Flags = PRVSRV_DEVICE_FLAGS_LMA;

		/*
			FIXME: We might want PT memory to come from a different heap so it
			would make sense to specify the HeapID for it, but need to think
			if/how this would affect how we do the CPU <> Dev physical address
			translation.
		*/
		psDeviceNode->pszMMUPxPDumpMemSpaceName = PhysHeapPDumpMemspaceName(psDeviceNode->psPhysHeap);
	}
	else
	{
		PVR_DPF((PVR_DBG_MESSAGE, "===== OS System memory, no local card memory"));

		psDeviceNode->pfnMMUPxAlloc = OSMMUPxAlloc;
		psDeviceNode->pfnMMUPxFree = OSMMUPxFree;
		psDeviceNode->pfnMMUPxMap = OSMMUPxMap;
		psDeviceNode->pfnMMUPxUnmap = OSMMUPxUnmap;
		psDeviceNode->uiMMUPxLog2AllocGran = OSGetPageShift();
		psDeviceNode->pfnCreateRamBackedPMR = PhysmemNewOSRamBackedPMR;

		/* See above FIXME */
		psDeviceNode->pszMMUPxPDumpMemSpaceName = PhysHeapPDumpMemspaceName(psDeviceNode->psPhysHeap);
	}

	/* Add the devnode to our list so we can unregister it later */
	psPVRSRVData->apsRegisteredDevNodes[psPVRSRVData->ui32RegisteredDevices++] = psDeviceNode;

	psDeviceNode->ui32RefCount = 1;
	psDeviceNode->psDevConfig = psDevConfig;

	/* all devices need a unique identifier */
	AllocateDeviceID(psPVRSRVData, &psDeviceNode->sDevId.ui32DeviceIndex);

	/* Device type and class will be setup during this callback */
	eError = sRegisterDevice[psDevConfig->eDeviceType](psDeviceNode);
	if (eError != PVRSRV_OK)
	{
		OSFreeMem(psDeviceNode);
		/*not nulling pointer, out of scope*/
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRegisterDevice : Failed to register device"));
		eError = PVRSRV_ERROR_DEVICE_REGISTER_FAILED;
		goto e3;
	}

	/* Need to remember the RGX device node for use in the Transport Layer
	 * when allocating stream buffers that are shared with clients.
	 */
	if ( PVRSRV_DEVICE_TYPE_RGX == psDeviceNode->psDevConfig->eDeviceType ) 
	{
		TLSetGlobalRgxDevice(psDeviceNode);
	}

	PVR_DPF((PVR_DBG_MESSAGE, "Registered device %d of type %d", psDeviceNode->sDevId.ui32DeviceIndex, psDeviceNode->sDevId.eDeviceType));
	PVR_DPF((PVR_DBG_MESSAGE, "Register bank address = 0x%08lx", (unsigned long)psDevConfig->sRegsCpuPBase.uiAddr));
	PVR_DPF((PVR_DBG_MESSAGE, "IRQ = %d", psDevConfig->ui32IRQ));
	
	/* and finally insert the device into the dev-list */
	List_PVRSRV_DEVICE_NODE_Insert(&psPVRSRVData->psDeviceNodeList, psDeviceNode);

	/* set device state */
	psDeviceNode->eDevState = PVRSVR_DEVICE_STATE_ACTIVE;

	return PVRSRV_OK;
e3:
	if (psDeviceNode->psLocalDevMemArena)
	{
		RA_Delete(psDeviceNode->psLocalDevMemArena);
	}
e2:
	PhysHeapRelease(psDeviceNode->psPhysHeap);
e1:
	OSFreeMem(psDeviceNode);
e0:
	return eError;
}

PVRSRV_ERROR IMG_CALLCONV PVRSRVSysPrePowerState(PVRSRV_SYS_POWER_STATE eNewPowerState, IMG_BOOL bForced)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (gpsSysConfig->pfnSysPrePowerState)
	{
		eError = gpsSysConfig->pfnSysPrePowerState(eNewPowerState);
	}
	return eError;
}

PVRSRV_ERROR IMG_CALLCONV PVRSRVSysPostPowerState(PVRSRV_SYS_POWER_STATE eNewPowerState, IMG_BOOL bForced)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (gpsSysConfig->pfnSysPostPowerState)
	{
		eError = gpsSysConfig->pfnSysPostPowerState(eNewPowerState);
	}
	return eError;
}

PVRSRV_ERROR IMG_CALLCONV PVRSRVRegisterExtDevice(PVRSRV_DEVICE_NODE *psDeviceNode,
													IMG_UINT32 *pui32DeviceIndex,
													IMG_UINT32 ui32PhysHeapID)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_ERROR eError;

	psDeviceNode->ui32RefCount = 1;

	eError = PhysHeapAcquire(ui32PhysHeapID, &psDeviceNode->psPhysHeap);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRegisterExtDevice: Failed to acquire physcial memory heap"));
		goto e0;
	}		
	/* allocate a unique device id */
	eError = AllocateDeviceID(psPVRSRVData, &psDeviceNode->sDevId.ui32DeviceIndex);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVRegisterExtDevice: Failed to allocate Device ID"));
		goto e1;
	}

	if (pui32DeviceIndex)
	{
		*pui32DeviceIndex = psDeviceNode->sDevId.ui32DeviceIndex;
	}

	List_PVRSRV_DEVICE_NODE_Insert(&psPVRSRVData->psDeviceNodeList, psDeviceNode);

	return PVRSRV_OK;
e1:
	PhysHeapRelease(psDeviceNode->psPhysHeap);
e0:
	return eError;
}

IMG_VOID IMG_CALLCONV PVRSRVUnregisterExtDevice(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	List_PVRSRV_DEVICE_NODE_Remove(psDeviceNode);
	(IMG_VOID)FreeDeviceID(psPVRSRVData, psDeviceNode->sDevId.ui32DeviceIndex);
	PhysHeapRelease(psDeviceNode->psPhysHeap);
}

static PVRSRV_ERROR PVRSRVFinaliseSystem_SetPowerState_AnyCb(PVRSRV_DEVICE_NODE *psDeviceNode, va_list va)
{
	PVRSRV_ERROR eError;
	PVRSRV_DEV_POWER_STATE ePowState;

	ePowState = va_arg(va, PVRSRV_DEV_POWER_STATE);

	/* We don't want to free the lock while waiting */
	OSSetKeepPVRLock();

	eError = PVRSRVPowerLock();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem_SetPowerState_AnyCb: Failed to acquire power lock"));
		return eError;
	}

	eError = PVRSRVSetDevicePowerStateKM(psDeviceNode->sDevId.ui32DeviceIndex,
										 ePowState,
										 KERNEL_ID, IMG_FALSE, IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed PVRSRVSetDevicePowerStateKM call (%s, device index: %d)", 
						PVRSRVGetErrorStringKM(eError),
						psDeviceNode->sDevId.ui32DeviceIndex));
	}
	
	PVRSRVPowerUnlock();

	return eError;
}

/*wraps the PVRSRVDevInitCompatCheck call and prints a debugging message if failed*/
static PVRSRV_ERROR PVRSRVFinaliseSystem_CompatCheck_Any_va(PVRSRV_DEVICE_NODE *psDeviceNode, va_list va)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 *pui32ClientBuildOptions;

	pui32ClientBuildOptions = va_arg(va, IMG_UINT32*);

	eError = PVRSRVDevInitCompatCheck(psDeviceNode, *pui32ClientBuildOptions);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed PVRSRVDevInitCompatCheck call (device index: %d)", psDeviceNode->sDevId.ui32DeviceIndex));
	}
	return eError;
}


/*!
******************************************************************************

 @Function	PVRSRVFinaliseSystem

 @Description

 Final part of system initialisation.

 @Input	   ui32DevIndex : Index to the required device

 @Return   PVRSRV_ERROR  :

******************************************************************************/
PVRSRV_ERROR IMG_CALLCONV PVRSRVFinaliseSystem(IMG_BOOL bInitSuccessful, IMG_UINT32 ui32ClientBuildOptions)
{
	PVRSRV_DATA			*psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_ERROR		eError;
	IMG_UINT32			i;

	PVR_DPF((PVR_DBG_MESSAGE, "PVRSRVFinaliseSystem"));

	if (bInitSuccessful)
	{
		/* Place all devices into their default power state. */
		eError = List_PVRSRV_DEVICE_NODE_PVRSRV_ERROR_Any_va(psPVRSRVData->psDeviceNodeList,
														&PVRSRVFinaliseSystem_SetPowerState_AnyCb,
														PVRSRV_DEV_POWER_STATE_DEFAULT);
		if (eError != PVRSRV_OK)
		{
			return eError;
		}

		for (i=0;i<psPVRSRVData->ui32RegisteredDevices;i++)
		{
			PVRSRV_DEVICE_NODE *psDeviceNode = psPVRSRVData->apsRegisteredDevNodes[i];

			SyncPrimContextCreate(IMG_NULL,
								  psDeviceNode,
								  IMG_NULL,
								  &psDeviceNode->hSyncPrimContext);

			/* Allocate general purpose sync primitive */
			eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext, &psDeviceNode->psSyncPrim);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed to allocate sync primitive with error (%u)", eError));
				return eError;
			}
			/* Allocate PreKick sync primitive */
			eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext, &psDeviceNode->psSyncPrimPreKick);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed to allocate PreKick sync primitive with error (%u)", eError));
				return eError;
			}
			/* Allocate sync primitive for deferred ZS Buffer population */
			eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext, &psDeviceNode->psZSBufferPopulateSyncPrim);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed to allocate ZS buffer population sync primitive with error (%u)", eError));
				return eError;
			}
			SyncPrimSet(psDeviceNode->psZSBufferPopulateSyncPrim,0);

			/* Allocate sync primitive for deferred ZS Buffer unpopulation */
			eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext, &psDeviceNode->psZSBufferUnPopulateSyncPrim);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed to allocate ZS buffer unpopulation sync primitive with error (%u)", eError));
				return eError;
			}
			SyncPrimSet(psDeviceNode->psZSBufferUnPopulateSyncPrim,0);

			/* Allocate sync primitive for grow */
			eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext, &psDeviceNode->psGrowSyncPrim);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed to allocate grow sync primitive with error (%u)", eError));
				return eError;
			}
			SyncPrimSet(psDeviceNode->psGrowSyncPrim,0);

			/* Allocate sync primitive for shrink */
			eError = SyncPrimAlloc(psDeviceNode->hSyncPrimContext, &psDeviceNode->psShrinkSyncPrim);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed to allocate shrink sync primitive with error (%u)", eError));
				return eError;
			}
			SyncPrimSet(psDeviceNode->psShrinkSyncPrim,0);

			/* Register ZLS Sync prim */
			eError = psDeviceNode->pfnZSBufferSyncPrimUpdate(psDeviceNode,
															psDeviceNode->psZSBufferPopulateSyncPrim,
															psDeviceNode->psZSBufferUnPopulateSyncPrim,
															psDeviceNode->psGrowSyncPrim,
															psDeviceNode->psShrinkSyncPrim);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR,"PVRSRVFinaliseSystem: Failed to submit ZLS sync primitive to firmware with error (%u)", eError));
				return eError;
			}

			/* Init lists of deferred Allocations */
			eError = OSLockCreate(&psDeviceNode->hLockZSBuffer,LOCK_TYPE_PASSIVE);
			PVR_ASSERT(eError == PVRSRV_OK);
			dllist_init(&psDeviceNode->sDeferredAllocHead);
			eError = OSLockCreate(&psDeviceNode->hLockFreeList,LOCK_TYPE_PASSIVE);
			PVR_ASSERT(eError == PVRSRV_OK);
			dllist_init(&psDeviceNode->sFreeListHead);
			psDeviceNode->ui32DeferredAllocCurrID = 1;
			psDeviceNode->ui32FreelistCurrID = 1;
		}

		/* Verify firmware compatibility for devices */
		eError = List_PVRSRV_DEVICE_NODE_PVRSRV_ERROR_Any_va(psPVRSRVData->psDeviceNodeList,
													&PVRSRVFinaliseSystem_CompatCheck_Any_va,
													&ui32ClientBuildOptions);
		if (eError != PVRSRV_OK)
		{
			return eError;
		}
		

	}

	eError = PDumpStopInitPhaseKM(IMG_SRV_INIT);

	if(eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "Failed to stop PDump init phase"));
		return eError;
	}

	return PVRSRV_OK;
}


PVRSRV_ERROR PVRSRVDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode,IMG_UINT32 ui32ClientBuildOptions)
{
	/* Only check devices which specify a compatibility check callback */
	if (psDeviceNode->pfnInitDeviceCompatCheck)
		return psDeviceNode->pfnInitDeviceCompatCheck(psDeviceNode, ui32ClientBuildOptions);
	else
		return PVRSRV_OK;
}

/*!
******************************************************************************

 @Function	PVRSRVAcquireDeviceDataKM

 @Description

 Matchs a device given a device type and a device index.

 @input	psDeviceNode :The device node to be matched.

 @Input	   va : Variable argument list with:
			eDeviceType : Required device type. If type is unknown use ui32DevIndex
						 to locate device data

 			ui32DevIndex : Index to the required device obtained from the
						PVRSRVEnumerateDevice function

 @Return   PVRSRV_ERROR  :

******************************************************************************/
static IMG_VOID * PVRSRVAcquireDeviceDataKM_Match_AnyVaCb(PVRSRV_DEVICE_NODE *psDeviceNode, va_list va)
{
	PVRSRV_DEVICE_TYPE eDeviceType;
	IMG_UINT32 ui32DevIndex;

	eDeviceType = va_arg(va, PVRSRV_DEVICE_TYPE);
	ui32DevIndex = va_arg(va, IMG_UINT32);

	if ((eDeviceType != PVRSRV_DEVICE_TYPE_UNKNOWN &&
		psDeviceNode->sDevId.eDeviceType == eDeviceType) ||
		(eDeviceType == PVRSRV_DEVICE_TYPE_UNKNOWN &&
		 psDeviceNode->sDevId.ui32DeviceIndex == ui32DevIndex))
	{
		return psDeviceNode;
	}
	else
	{
		return IMG_NULL;
	}
}

/*!
******************************************************************************

 @Function	PVRSRVAcquireDeviceDataKM

 @Description

 Returns device information

 @Input	   ui32DevIndex : Index to the required device obtained from the
						PVRSRVEnumerateDevice function

 @Input	   eDeviceType : Required device type. If type is unknown use ui32DevIndex
						 to locate device data

 @Output  *phDevCookie : Dev Cookie


 @Return   PVRSRV_ERROR  :

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVAcquireDeviceDataKM (IMG_UINT32			ui32DevIndex,
													 PVRSRV_DEVICE_TYPE	eDeviceType,
													 IMG_HANDLE			*phDevCookie)
{
	PVRSRV_DATA			*psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_DEVICE_NODE	*psDeviceNode;

	PVR_DPF((PVR_DBG_MESSAGE, "PVRSRVAcquireDeviceDataKM"));

	/* Find device in the list */
	psDeviceNode = List_PVRSRV_DEVICE_NODE_Any_va(psPVRSRVData->psDeviceNodeList,
												&PVRSRVAcquireDeviceDataKM_Match_AnyVaCb,
												eDeviceType,
												ui32DevIndex);


	if (!psDeviceNode)
	{
		/* device can't be found in the list so it isn't in the system */
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVAcquireDeviceDataKM: requested device is not present"));
		return PVRSRV_ERROR_INIT_FAILURE;
	}

/*FoundDevice:*/

	PVR_ASSERT (psDeviceNode->ui32RefCount > 0);

	/* return the dev cookie? */
	if (phDevCookie)
	{
		*phDevCookie = (IMG_HANDLE)psDeviceNode;
	}

	return PVRSRV_OK;
}


/*!
******************************************************************************

 @Function	PVRSRVDeinitialiseDevice

 @Description

 This De-inits device

 @Input	   ui32DevIndex : Index to the required device

 @Return   PVRSRV_ERROR  :

******************************************************************************/
static PVRSRV_ERROR IMG_CALLCONV PVRSRVUnregisterDevice(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	PVRSRV_DATA			*psPVRSRVData = PVRSRVGetPVRSRVData();
	PVRSRV_ERROR		eError;

	/* We don't want to free the lock while waiting */
	OSSetKeepPVRLock();

	eError = PVRSRVPowerLock();
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeinitialiseDevice: Failed to acquire power lock"));
		return eError;
	}

	/*
		Power down the device if necessary.
	 */
	eError = PVRSRVSetDevicePowerStateKM(psDeviceNode->sDevId.ui32DeviceIndex,
										 PVRSRV_DEV_POWER_STATE_OFF,
										 KERNEL_ID,
										 IMG_FALSE,
										 IMG_TRUE);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVDeinitialiseDevice: Failed PVRSRVSetDevicePowerStateKM call (%s)", PVRSRVGetErrorStringKM(eError)));

		/* If the driver is okay then return the error, otherwise we can ignore this error. */
		if (PVRSRVGetPVRSRVData()->eServicesState == PVRSRV_SERVICES_STATE_OK)
		{
			PVRSRVPowerUnlock();
			return eError;
		}
		else
		{
			PVR_DPF((PVR_DBG_MESSAGE,"PVRSRVDeinitialiseDevice: Will continue to unregister as driver status is not OK"));
		}
	}

	PVRSRVPowerUnlock();

	/*
		De-init the device.
	*/
	sUnregisterDevice[psDeviceNode->sDevId.eDeviceType](psDeviceNode);

	/* Remove RA for local card memory */
	if (psDeviceNode->psLocalDevMemArena)
	{
		RA_Delete(psDeviceNode->psLocalDevMemArena);
	}

	/* remove node from list */
	List_PVRSRV_DEVICE_NODE_Remove(psDeviceNode);

	/* deallocate id and memory */
	(IMG_VOID)FreeDeviceID(psPVRSRVData, psDeviceNode->sDevId.ui32DeviceIndex);

	PhysHeapRelease(psDeviceNode->psPhysHeap);

	OSFreeMem(psDeviceNode);
	/*not nulling pointer, out of scope*/

	return (PVRSRV_OK);
}


/*
	PollForValueKM
*/
static
PVRSRV_ERROR IMG_CALLCONV PollForValueKM (volatile IMG_UINT32*	pui32LinMemAddr,
										  IMG_UINT32			ui32Value,
										  IMG_UINT32			ui32Mask,
										  IMG_UINT32			ui32Timeoutus,
										  IMG_UINT32			ui32PollPeriodus,
										  IMG_BOOL				bAllowPreemption)
{
#if defined(NO_HARDWARE)
	PVR_UNREFERENCED_PARAMETER(pui32LinMemAddr);
	PVR_UNREFERENCED_PARAMETER(ui32Value);
	PVR_UNREFERENCED_PARAMETER(ui32Mask);
	PVR_UNREFERENCED_PARAMETER(ui32Timeoutus);
	PVR_UNREFERENCED_PARAMETER(ui32PollPeriodus);
	PVR_UNREFERENCED_PARAMETER(bAllowPreemption);
	return PVRSRV_OK;
#else
	IMG_UINT32	ui32ActualValue = 0xFFFFFFFFU; /* Initialiser only required to prevent incorrect warning */

	if (bAllowPreemption)
	{
		PVR_ASSERT(ui32PollPeriodus >= 1000);
	}

	LOOP_UNTIL_TIMEOUT(ui32Timeoutus)
	{
		ui32ActualValue = (*pui32LinMemAddr & ui32Mask);
		if(ui32ActualValue == ui32Value)
		{
			return PVRSRV_OK;
		}

		if (gpsPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK)
		{
			return PVRSRV_ERROR_TIMEOUT;
		}

		if (bAllowPreemption)
		{
			OSSleepms(ui32PollPeriodus / 1000);
		}
		else
		{
			OSWaitus(ui32PollPeriodus);
		}
	} END_LOOP_UNTIL_TIMEOUT();

	PVR_DPF((PVR_DBG_ERROR,"PollForValueKM: Timeout. Expected 0x%x but found 0x%x (mask 0x%x).",
			ui32Value, ui32ActualValue, ui32Mask));
	
	return PVRSRV_ERROR_TIMEOUT;
#endif /* NO_HARDWARE */
}


/*
	PVRSRVPollForValueKM
*/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVPollForValueKM (volatile IMG_UINT32	*pui32LinMemAddr,
												IMG_UINT32			ui32Value,
												IMG_UINT32			ui32Mask)
{
	return PollForValueKM(pui32LinMemAddr, ui32Value, ui32Mask,
						  MAX_HW_TIME_US,
						  MAX_HW_TIME_US/WAIT_TRY_COUNT,
						  IMG_FALSE);
}

/*
	PVRSRVWaitForValueKM
*/
IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVWaitForValueKM (volatile IMG_UINT32	*pui32LinMemAddr,
												IMG_UINT32			ui32Value,
												IMG_UINT32			ui32Mask)
{
#if defined(NO_HARDWARE)
	PVR_UNREFERENCED_PARAMETER(pui32LinMemAddr);
	PVR_UNREFERENCED_PARAMETER(ui32Value);
	PVR_UNREFERENCED_PARAMETER(ui32Mask);
	return PVRSRV_OK;
#else

	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	IMG_HANDLE hOSEvent;
	PVRSRV_ERROR eError;
	PVRSRV_ERROR eErrorWait;
	IMG_UINT32 ui32ActualValue;

	eError = OSEventObjectOpen(psPVRSRVData->hGlobalEventObject, &hOSEvent);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,"PVRSRVWaitForValueKM: Failed to setup EventObject with error (%d)", eError));
		goto EventObjectOpenError;
	}

	eError = PVRSRV_ERROR_TIMEOUT;
	
	LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
	{
		ui32ActualValue = (*pui32LinMemAddr & ui32Mask);

		/* Expected value has been found */
		if (ui32ActualValue == ui32Value)
		{
			eError = PVRSRV_OK;
			break;
		}
		/* Services in bad state, dont wait any more */
		else if (psPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK)
		{
			eError = PVRSRV_ERROR_NOT_READY;
			break;
		}
		else
		{
			/* wait for event and retry */
			eErrorWait = OSEventObjectWait(hOSEvent);
			if (eErrorWait != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_WARNING,"PVRSRVWaitForValueKM: Waiting for value failed with error %d. Expected 0x%x but found 0x%x (Mask 0x%08x). Retrying",
							eErrorWait,
							ui32Value,
							ui32ActualValue,
							ui32Mask));
			}
		}
	} END_LOOP_UNTIL_TIMEOUT();

	OSEventObjectClose(hOSEvent);

EventObjectOpenError:

	return eError;

#endif /* NO_HARDWARE */
}

#if !defined(NO_HARDWARE)
static IMG_BOOL _CheckStatus(PDLLIST_NODE psNode, IMG_PVOID pvCallbackData)
{
	PVRSRV_CMDCOMP_HANDLE	hCmdCompCallerHandle = (PVRSRV_CMDCOMP_HANDLE) pvCallbackData;
	PVRSRV_CMDCOMP_NOTIFY	*psNotify;

	psNotify = IMG_CONTAINER_OF(psNode, PVRSRV_CMDCOMP_NOTIFY, sListNode);

	/* A device has finished some processing, check if that unblocks other devices */
	if (hCmdCompCallerHandle != psNotify->hCmdCompHandle)
	{
		psNotify->pfnCmdCompleteNotify(psNotify->hCmdCompHandle);
	}

	/* keep processing until the end of the list */
	return IMG_TRUE;
}
#endif

IMG_VOID IMG_CALLCONV PVRSRVCheckStatus(PVRSRV_CMDCOMP_HANDLE hCmdCompCallerHandle)
{
	PVRSRV_DATA			*psPVRSRVData = PVRSRVGetPVRSRVData();

	/* notify any registered device to check if block work items can now proceed */
#if !defined(NO_HARDWARE)
	OSWRLockAcquireRead(hNotifyLock);
	dllist_foreach_node(&sCmdCompNotifyHead, _CheckStatus, hCmdCompCallerHandle);
	OSWRLockReleaseRead(hNotifyLock);
#endif

	/* signal global event object */
	if (psPVRSRVData->hGlobalEventObject)
	{
		IMG_HANDLE hOSEventKM = psPVRSRVData->hGlobalEventObject;
		if(hOSEventKM)
		{
			OSEventObjectSignal(hOSEventKM);
		}
	}
}

/*!
 ******************************************************************************

 @Function		PVRSRVGetErrorStringKM

 @Description	Returns a text string relating to the PVRSRV_ERROR enum.

 @Note		case statement used rather than an indexed arrary to ensure text is
 			synchronised with the correct enum

 @Input		eError : PVRSRV_ERROR enum

 @Return	const IMG_CHAR * : Text string

 @Note		Must be kept in sync with servicesext.h

******************************************************************************/

IMG_EXPORT
const IMG_CHAR *PVRSRVGetErrorStringKM(PVRSRV_ERROR eError)
{
	switch(eError)
	{
		case PVRSRV_OK:
			return "PVRSRV_OK";
#define PVRE(x) \
		case x: \
			return #x;
#include "pvrsrv_errors.h"
#undef PVRE
		default:
			return "Unknown PVRSRV error number";
	}
}

/*
	PVRSRVSystemDebugInfo
 */
PVRSRV_ERROR PVRSRVSystemDebugInfo(IMG_VOID)
{
	return SysDebugInfo(gpsSysConfig);
}

/*
	PVRSRVGetSystemName
*/
const IMG_CHAR *PVRSRVGetSystemName(IMG_VOID)
{
	return gpsSysConfig->pszSystemName;
}

/*
	PVRSRVSystemHasCacheSnooping
*/
IMG_BOOL PVRSRVSystemHasCacheSnooping(IMG_VOID)
{
	return gpsSysConfig->bHasCacheSnooping;
}

/*
	PVRSRVSystemWaitCycles
*/
IMG_VOID PVRSRVSystemWaitCycles(PVRSRV_DEVICE_CONFIG *psDevConfig, IMG_UINT32 ui32Cycles)
{
	/* Delay in us */
	IMG_UINT32 ui32Delayus = 1;

	/* obtain the device freq */
	if (psDevConfig->pfnClockFreqGet != IMG_NULL)
	{
		IMG_UINT32 ui32DeviceFreq;

		ui32DeviceFreq = psDevConfig->pfnClockFreqGet(psDevConfig->hSysData);

		ui32Delayus = (ui32Cycles*1000000)/ui32DeviceFreq;

		if (ui32Delayus == 0)
		{
			ui32Delayus = 1;
		}
	}

	OSWaitus(ui32Delayus);
}

/*
	PVRSRVRegisterCmdCompleteNotify
*/
PVRSRV_ERROR PVRSRVRegisterCmdCompleteNotify(IMG_HANDLE *phNotify, PFN_CMDCOMP_NOTIFY pfnCmdCompleteNotify, PVRSRV_CMDCOMP_HANDLE hCmdCompHandle)
{
	PVRSRV_CMDCOMP_NOTIFY *psNotify;

	if ((phNotify == IMG_NULL) || (pfnCmdCompleteNotify == IMG_NULL) || (hCmdCompHandle == IMG_NULL))
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Bad arguments (%p, %p, %p)", __FUNCTION__, phNotify, pfnCmdCompleteNotify, hCmdCompHandle));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	psNotify = OSAllocMem(sizeof(*psNotify));
	if (psNotify == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Not enough memory to allocate CmdCompleteNotify function", __FUNCTION__));
		return PVRSRV_ERROR_OUT_OF_MEMORY;		
	}

	/* Set-up the notify data */
	psNotify->hCmdCompHandle = hCmdCompHandle;
	psNotify->pfnCmdCompleteNotify = pfnCmdCompleteNotify;

	/* Add it to the list of Notify functions */
	OSWRLockAcquireWrite(hNotifyLock);
	dllist_add_to_tail(&sCmdCompNotifyHead, &psNotify->sListNode);
	OSWRLockReleaseWrite(hNotifyLock);

	*phNotify = psNotify;

	return PVRSRV_OK;
}

/*
	PVRSRVUnregisterCmdCompleteNotify
*/
PVRSRV_ERROR PVRSRVUnregisterCmdCompleteNotify(IMG_HANDLE hNotify)
{
	PVRSRV_CMDCOMP_NOTIFY *psNotify = (PVRSRV_CMDCOMP_NOTIFY*) hNotify;

	if (psNotify == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR,"%s: Bad arguments (%p)", __FUNCTION__, hNotify));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/* remove the node from the list */
	OSWRLockAcquireWrite(hNotifyLock);
	dllist_remove_node(&psNotify->sListNode);
	OSWRLockReleaseWrite(hNotifyLock);

	/* free the notify structure that holds the node */
	OSFreeMem(psNotify);

	return PVRSRV_OK;

}

PVRSRV_ERROR AcquireGlobalEventObjectServer(IMG_HANDLE *phGlobalEventObject)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	*phGlobalEventObject = psPVRSRVData->hGlobalEventObject;

	return PVRSRV_OK;
}

PVRSRV_ERROR ReleaseGlobalEventObjectServer(IMG_HANDLE hGlobalEventObject)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	PVR_ASSERT(psPVRSRVData->hGlobalEventObject == hGlobalEventObject);

	return PVRSRV_OK;
}

/*****************************************************************************
 End of file (pvrsrv.c)
*****************************************************************************/
