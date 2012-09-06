/*!****************************************************************************
@File			pvrsrv.h

@Author			Imagination Technologies

@date   		2011/Feb/08
 
@Copyright     	Copyright 2011 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either
                material or conceptual may be copied or distributed,
                transmitted, transcribed, stored in a retrieval system
                or translated into any human or computer language in any
                form by any means, electronic, mechanical, manual or
                other-wise, or disclosed to third parties without the
                express written permission of Imagination Technologies
                Limited, Unit 8, HomePark Industrial Estate,
                King's Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform		generic

@Description	PowerVR services server header file

@DoxygenVer		

******************************************************************************/

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

	typedef struct _SYS_DEVICE_ID_TAG {
		IMG_UINT32 uiID;
		IMG_BOOL bInUse;

	} SYS_DEVICE_ID;

/* FIXME: Anything that uses SYS_DEVICE_COUNT should be changed */
	typedef struct PVRSRV_DATA_TAG {
		IMG_UINT32 ui32NumDevices;	/*!< number of devices in system */
		SYS_DEVICE_ID sDeviceID[SYS_DEVICE_COUNT];
		PVRSRV_DEVICE_NODE *apsRegisteredDevNodes[SYS_DEVICE_COUNT];
		IMG_UINT32 ui32RegisteredDevices;
		IMG_UINT32 ui32CurrentOSPowerState;	/*!< current OS specific power state */
		PVRSRV_DEVICE_NODE *psDeviceNodeList;	/*!< List head of device nodes */
		struct _DEVICE_COMMAND_DATA_
		    *apsDeviceCommandData[SYS_DEVICE_COUNT];

		IMG_UINT32 ui32RegisteredPhysHeaps;
		PHYS_HEAP *apsRegisteredPhysHeaps[SYS_PHYS_HEAP_COUNT];

		PVRSRV_POWER_DEV *psPowerDeviceList;	/*!< list of devices registered with the power manager */
		PVRSRV_RESOURCE sPowerStateChangeResource;	/*!< lock for power state transitions */
		PVRSRV_SYS_POWER_STATE eCurrentPowerState;	/*!< current Kernel services power state */
		PVRSRV_SYS_POWER_STATE eFailedPowerState;	/*!< Kernel services power state (Failed to transition to) */

		PVRSRV_SERVICES_STATE eServicesState;	/*!< global driver state */

		IMG_HANDLE hGlobalEventObject;	/*!< OS Global Event Object */
	} PVRSRV_DATA;

	typedef IMG_HANDLE PVRSRV_CMDCOMP_HANDLE;
	typedef IMG_VOID(*PFN_CMDCOMP_NOTIFY) (PVRSRV_CMDCOMP_HANDLE
					       hCmdCompHandle);

	typedef struct PVRSRV_CMDCOMP_NOTIFY_TAG {
		PVRSRV_CMDCOMP_HANDLE hCmdCompHandle;
		PFN_CMDCOMP_NOTIFY pfnCmdCompleteNotify;

		DLLIST_NODE sListNode;
	} PVRSRV_CMDCOMP_NOTIFY;

/*!
******************************************************************************

 @Function	PVRSRVGetPVRSRVData

 @Description	Get a pointer to the global data

 @Return   PVRSRV_DATA *

******************************************************************************/
	PVRSRV_DATA *PVRSRVGetPVRSRVData(IMG_VOID);

/* FIXME: Should this definition live here? */
	 IMG_EXPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVGetMiscInfoKM(IMG_UINT32
							  ui32StateRequest,
							  IMG_UINT32 *
							  pui32StatePresent,
							  IMG_UINT32
							  ui32MemoryStrLen,
							  IMG_CHAR *
							  pszMemoryStr,
							  IMG_HANDLE *
							  hGlobalEventObject,
							  IMG_UINT32 *
							  paui32DDKVersion);

	 IMG_EXPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVEnumerateDevicesKM(IMG_UINT32 *
							       pui32NumDevices,
							       PVRSRV_DEVICE_IDENTIFIER
							       * psDevIdList);

	 IMG_EXPORT
	    PVRSRV_ERROR IMG_CALLCONV PVRSRVAcquireDeviceDataKM(IMG_UINT32
								ui32DevIndex,
								PVRSRV_DEVICE_TYPE
								eDeviceType,
								IMG_HANDLE *
								phDevCookie);

	PVRSRV_ERROR IMG_CALLCONV PVRSRVRegisterExtDevice(PVRSRV_DEVICE_NODE *
							  psDeviceNode,
							  IMG_UINT32 *
							  pui32DeviceIndex,
							  IMG_UINT32
							  ui32PhysHeapID);

	IMG_VOID IMG_CALLCONV PVRSRVUnregisterExtDevice(PVRSRV_DEVICE_NODE *
							psDeviceNode);

	PVRSRV_ERROR IMG_CALLCONV PVRSRVSysPrePowerState(PVRSRV_SYS_POWER_STATE
							 eNewPowerState);

	PVRSRV_ERROR IMG_CALLCONV PVRSRVSysPostPowerState(PVRSRV_SYS_POWER_STATE
							  eNewPowerState);

	PVRSRV_ERROR LMA_MMUPxAlloc(PVRSRV_DEVICE_NODE * psDevNode,
				    IMG_SIZE_T uiSize, Px_HANDLE * psMemHandle,
				    IMG_DEV_PHYADDR * psDevPAddr);

	IMG_VOID LMA_MMUPxFree(PVRSRV_DEVICE_NODE * psDevNode,
			       Px_HANDLE * psMemHandle);

	PVRSRV_ERROR LMA_MMUPxMap(PVRSRV_DEVICE_NODE * psDevNode,
				  Px_HANDLE * psMemHandle, IMG_SIZE_T uiSize,
				  IMG_DEV_PHYADDR * psDevPAddr,
				  IMG_VOID ** pvPtr);

	IMG_VOID LMA_MMUPxUnmap(PVRSRV_DEVICE_NODE * psDevNode,
				Px_HANDLE * psMemHandle, IMG_VOID * pvPtr);

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
	IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV PVRSRVPollForValueKM(volatile
								  IMG_UINT32 *
								  pui32LinMemAddr,
								  IMG_UINT32
								  ui32Value,
								  IMG_UINT32
								  ui32Mask);

/*!
******************************************************************************
 @Function	PVRSRVWaitForValueKM

 @Description
 Waits (using EventObjects) for a value to match a masked read

 @Input pui32LinMemAddr			: CPU linear address to poll
 @Input ui32Value				: required value
 @Input ui32Mask				: Mask
 @Input ui32MaxCount			: Number of retries

 @Return   PVRSRV_ERROR :
******************************************************************************/
	IMG_IMPORT PVRSRV_ERROR IMG_CALLCONV PVRSRVWaitForValueKM(volatile
								  IMG_UINT32 *
								  pui32LinMemAddr,
								  IMG_UINT32
								  ui32Value,
								  IMG_UINT32
								  ui32Mask,
								  IMG_UINT32
								  ui32MaxCount);

/*!
*****************************************************************************
 @Function	: PVRSRVGetSystemName

 @Description	: Gets the system name string

 @Return : The system name
*****************************************************************************/
	IMG_CONST IMG_CHAR *PVRSRVGetSystemName(IMG_VOID);

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
	IMG_VOID PVRSRVSystemWaitCycles(PVRSRV_DEVICE_CONFIG * psDevConfig,
					IMG_UINT32 ui32Cycles);

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
	IMG_VOID IMG_CALLCONV PVRSRVCheckStatus(PVRSRV_CMDCOMP_HANDLE
						hCmdCompCallerHandle);

/*!
*****************************************************************************
 @Function	: PVRSRVRegisterCmdCompleteNotify

 @Description	: Register a notify function which is called when some device
				  finishes some work (that is, when someone calls to PVRSRVCheckStatus).

 @Input pfnCmdCompleteNotify : Notify function

 @Input hPrivData : Handler to data passed to the Notify function when called

*****************************************************************************/
	PVRSRV_ERROR PVRSRVRegisterCmdCompleteNotify(PFN_CMDCOMP_NOTIFY
						     pfnCmdCompleteNotify,
						     PVRSRV_CMDCOMP_HANDLE
						     hPrivData);

/*!
*****************************************************************************
 @Function	: PVRSRVUnregisterCmdCompleteNotify

 @Description	: Unregister a previosuly registered notify func.

 @Input pfnCmdCompleteNotify : Notify function to unregister

*****************************************************************************/
	IMG_VOID PVRSRVUnregisterCmdCompleteNotify(PFN_CMDCOMP_NOTIFY
						   pfnCmdCompleteNotify);
#endif				/* PVRSRV_H */
