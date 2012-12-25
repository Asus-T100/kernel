/**************************************************************************/ /*!
@File
@Title          PowerVR services server header file
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
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
*/ /***************************************************************************/

#ifndef PVRSRV_H
#define PVRSRV_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "device.h"
#include "resman.h"
#include "power.h"
#include "sysinfo.h"
#include "physheap.h"


typedef struct _SYS_DEVICE_ID_TAG
{
	IMG_UINT32	uiID;
	IMG_BOOL	bInUse;

} SYS_DEVICE_ID;

/* FIXME: Anything that uses SYS_DEVICE_COUNT should be changed */
typedef struct PVRSRV_DATA_TAG
{
    IMG_UINT32                  ui32NumDevices;      	   	/*!< number of devices in system */
	SYS_DEVICE_ID				sDeviceID[SYS_DEVICE_COUNT];
	PVRSRV_DEVICE_NODE			*apsRegisteredDevNodes[SYS_DEVICE_COUNT];
	IMG_UINT32					ui32RegisteredDevices;
	IMG_UINT32		 			ui32CurrentOSPowerState;	/*!< current OS specific power state */
	PVRSRV_DEVICE_NODE			*psDeviceNodeList;			/*!< List head of device nodes */
	struct _DEVICE_COMMAND_DATA_ *apsDeviceCommandData[SYS_DEVICE_COUNT];

	IMG_UINT32					ui32RegisteredPhysHeaps;
	PHYS_HEAP					*apsRegisteredPhysHeaps[SYS_PHYS_HEAP_COUNT];

    PVRSRV_POWER_DEV			*psPowerDeviceList;			/*!< list of devices registered with the power manager */
	POS_LOCK					hPowerLock;					/*!< lock for power state transitions */
   	PVRSRV_SYS_POWER_STATE		eCurrentPowerState;			/*!< current Kernel services power state */
   	PVRSRV_SYS_POWER_STATE		eFailedPowerState;			/*!< Kernel services power state (Failed to transition to) */

   	PVRSRV_SERVICES_STATE		eServicesState;				/*!< global driver state */

	IMG_HANDLE					hGlobalEventObject;			/*!< OS Global Event Object */
	
	PVRSRV_CACHE_OP				uiCacheOp;					/*!< Pending cache operations in the system */
	PRESMAN_DEFER_CONTEXT		hResManDeferContext;		/*!< Device driver global defer resman context */

	IMG_HANDLE					hCleanupThread;				/*!< Cleanup thread */
	IMG_BOOL					bUnload;					/*!< Driver unload is in progress */
} PVRSRV_DATA;


typedef IMG_HANDLE PVRSRV_CMDCOMP_HANDLE;
typedef IMG_VOID (*PFN_CMDCOMP_NOTIFY) (PVRSRV_CMDCOMP_HANDLE hCmdCompHandle);

typedef struct PVRSRV_CMDCOMP_NOTIFY_TAG
{
	PVRSRV_CMDCOMP_HANDLE	hCmdCompHandle;
	PFN_CMDCOMP_NOTIFY		pfnCmdCompleteNotify;

	DLLIST_NODE					sListNode;
} PVRSRV_CMDCOMP_NOTIFY;


/*!
******************************************************************************

 @Function	PVRSRVGetPVRSRVData

 @Description	Get a pointer to the global data

 @Return   PVRSRV_DATA *

******************************************************************************/
PVRSRV_DATA *PVRSRVGetPVRSRVData(IMG_VOID);

IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVEnumerateDevicesKM(IMG_UINT32 *pui32NumDevices,
											 	   PVRSRV_DEVICE_IDENTIFIER *psDevIdList);

IMG_EXPORT
PVRSRV_ERROR IMG_CALLCONV PVRSRVAcquireDeviceDataKM (IMG_UINT32			ui32DevIndex,
													 PVRSRV_DEVICE_TYPE	eDeviceType,
													 IMG_HANDLE			*phDevCookie);

PVRSRV_ERROR IMG_CALLCONV PVRSRVRegisterExtDevice(PVRSRV_DEVICE_NODE *psDeviceNode,
													IMG_UINT32 *pui32DeviceIndex,
													IMG_UINT32 ui32PhysHeapID);

IMG_VOID IMG_CALLCONV PVRSRVUnregisterExtDevice(PVRSRV_DEVICE_NODE *psDeviceNode);

PVRSRV_ERROR IMG_CALLCONV PVRSRVSysPrePowerState(PVRSRV_SYS_POWER_STATE eNewPowerState, IMG_BOOL bForced);

PVRSRV_ERROR IMG_CALLCONV PVRSRVSysPostPowerState(PVRSRV_SYS_POWER_STATE eNewPowerState, IMG_BOOL bForced);

PVRSRV_ERROR LMA_MMUPxAlloc(PVRSRV_DEVICE_NODE *psDevNode, IMG_SIZE_T uiSize,
							Px_HANDLE *psMemHandle, IMG_DEV_PHYADDR *psDevPAddr);

IMG_VOID LMA_MMUPxFree(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle);

PVRSRV_ERROR LMA_MMUPxMap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle,
							IMG_SIZE_T uiSize, IMG_DEV_PHYADDR *psDevPAddr,
							IMG_VOID **pvPtr);

IMG_VOID LMA_MMUPxUnmap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle,
						IMG_VOID *pvPtr);
										

/*!
******************************************************************************
 @Function	PVRSRVPollForValueKM

 @Description
 Polls for a value to match a masked read

 @Input pui32LinMemAddr : CPU linear address to poll
 @Input ui32Value : required value
 @Input ui32Mask : Mask

 @Return   PVRSRV_ERROR :
******************************************************************************/
IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV PVRSRVPollForValueKM(volatile IMG_UINT32	*pui32LinMemAddr,
														  IMG_UINT32			ui32Value,
														  IMG_UINT32			ui32Mask);

/*!
******************************************************************************
 @Function	PVRSRVWaitForValueKM

 @Description
 Waits (using EventObjects) for a value to match a masked read

 @Input pui32LinMemAddr			: CPU linear address to poll
 @Input ui32Value				: required value
 @Input ui32Mask				: Mask

 @Return   PVRSRV_ERROR :
******************************************************************************/
IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV PVRSRVWaitForValueKM(volatile IMG_UINT32	*pui32LinMemAddr,
														IMG_UINT32			ui32Value,
														IMG_UINT32			ui32Mask);

/*!
*****************************************************************************
 @Function	: PVRSRVSystemDebugInfo

 @Description	: Dump the system debug info
*****************************************************************************/
PVRSRV_ERROR PVRSRVSystemDebugInfo(IMG_VOID);

/*!
*****************************************************************************
 @Function	: PVRSRVGetSystemName

 @Description	: Gets the system name string

 @Return : The system name
*****************************************************************************/
const IMG_CHAR *PVRSRVGetSystemName(IMG_VOID);

/*!
*****************************************************************************
 @Function	: PVRSRVSystemHasCacheSnooping

 @Description	: Returns whether the system has cache snooping

 @Return : IMG_TRUE if the system has cache snooping
*****************************************************************************/
IMG_BOOL PVRSRVSystemHasCacheSnooping(IMG_VOID);

/*!
*****************************************************************************
 @Function	: PVRSRVSystemWaitCycles

 @Description	: Waits for at least ui32Cycles of the Device clk.

*****************************************************************************/
IMG_VOID PVRSRVSystemWaitCycles(PVRSRV_DEVICE_CONFIG *psDevConfig, IMG_UINT32 ui32Cycles);



/*!
*****************************************************************************
 @Function	: PVRSRVCheckStatus

 @Description	: Notify any registered cmd complete function (except if its
				  hPrivData matches the hCmdCompHandle handler) and raise the global 
				  event object. 

 @Input hCmdCompHandle	: Identify the caller by the handler used when 
						  registering for cmd complete. IMG_NULL calls all
						  the notify functions.

*****************************************************************************/
IMG_VOID IMG_CALLCONV PVRSRVCheckStatus(PVRSRV_CMDCOMP_HANDLE hCmdCompCallerHandle);

/*!
*****************************************************************************
 @Function	: PVRSRVRegisterCmdCompleteNotify

 @Description	: Register a notify function which is called when some device
				  finishes some work (that is, when someone calls to PVRSRVCheckStatus).

 @Input phNotify : Pointer to the Cmd complete notify handler

 @Input pfnCmdCompleteNotify : Notify function

 @Input hPrivData : Handler to data passed to the Notify function when called

*****************************************************************************/
PVRSRV_ERROR PVRSRVRegisterCmdCompleteNotify(IMG_HANDLE *phNotify, PFN_CMDCOMP_NOTIFY pfnCmdCompleteNotify, PVRSRV_CMDCOMP_HANDLE hPrivData);

/*!
*****************************************************************************
 @Function	: PVRSRVUnregisterCmdCompleteNotify

 @Description	: Unregister a previously registered notify func.

 @Input hNotify : Cmd complete notify handler registered previously

*****************************************************************************/
PVRSRV_ERROR PVRSRVUnregisterCmdCompleteNotify(IMG_HANDLE hNotify);

/*!
*****************************************************************************
 @Function	: AcquireGlobalEventObjectServer

 @Description	: Acquire the global event object.

 @Output phGlobalEventObject : Handle to the global event object

*****************************************************************************/
PVRSRV_ERROR AcquireGlobalEventObjectServer(IMG_HANDLE *phGlobalEventObject);

/*!
*****************************************************************************
 @Function	: ReleaseGlobalEventObjectServer

 @Description	: Release the global event object.

 @Input hGlobalEventObject : Handle to the global event object

*****************************************************************************/
PVRSRV_ERROR ReleaseGlobalEventObjectServer(IMG_HANDLE hGlobalEventObject);
#endif /* PVRSRV_H */
