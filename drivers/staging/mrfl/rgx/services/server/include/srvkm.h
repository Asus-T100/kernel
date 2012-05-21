/*!****************************************************************************
@File			srvkm.h

@Author			Imagination Technologies

@date   		20/07/2007
 
@Copyright     	Copyright 2007 by Imagination Technologies Limited.
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

@Description	Services kernel module internal header file

@DoxygenVer		

******************************************************************************/

#ifndef SRVKM_H
#define SRVKM_H

#include "servicesext.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**	Use PVR_DPF() unless message is necessary in release build
 */
#ifdef PVR_DISABLE_LOGGING
#define PVR_LOG(X)
#else
	/* PRQA S 3410 1 *//* this macro requires no brackets in order to work */
#define PVR_LOG(X)			PVRSRVReleasePrintf X;
#endif

	IMG_IMPORT IMG_VOID IMG_CALLCONV PVRSRVReleasePrintf(const IMG_CHAR *
							     pszFormat,
							     ...)
	    IMG_FORMAT_PRINTF(1, 2);

/*!
******************************************************************************

 @Function	PVRSRVInit

 @Description	Initialise services

 @Input	   psSysData	: sysdata structure

 @Return   PVRSRV_ERROR	:

******************************************************************************/
	PVRSRV_ERROR IMG_CALLCONV PVRSRVInit(IMG_VOID);

/*!
******************************************************************************

 @Function	PVRSRVDeInit

 @Description	De-Initialise services

 @Input	   psSysData	: sysdata structure

 @Return   PVRSRV_ERROR	:

******************************************************************************/
	IMG_VOID IMG_CALLCONV PVRSRVDeInit(IMG_VOID);

/*!
******************************************************************************

 @Function	PVRSRVScheduleDevicesKM

 @Description	Schedules all Services-Managed Devices to check their pending
 				command queues. The intention is that ScheduleDevices be called by the
				3rd party BC driver after it has finished writing new data to its output
				texture.

 @Input		bInLISR

 @Return	IMG_VOID

******************************************************************************/
	IMG_IMPORT IMG_VOID PVRSRVScheduleDevicesKM(IMG_BOOL bInLISR);

	IMG_VOID IMG_CALLCONV PVRSRVSetDCState(IMG_UINT32 ui32State);

	PVRSRV_ERROR IMG_CALLCONV PVRSRVSaveRestoreLiveSegments(IMG_HANDLE
								hArena,
								IMG_PBYTE
								pbyBuffer,
								IMG_SIZE_T *
								puiBufSize,
								IMG_BOOL bSave);

/*!
******************************************************************************

 @Function	PVRSRVScheduleDeviceCallbacks

 @Description	Schedule all device callbacks

 @Input		ui32CallerID

 @Return	IMG_VOID

******************************************************************************/
	IMG_VOID PVRSRVScheduleDeviceCallbacks(IMG_UINT32 ui32CallerID);

#if defined (__cplusplus)
}
#endif
/******************
HIGHER LEVEL MACROS
*******************//*----------------------------------------------------------------------------
Repeats the body of the loop for a certain minimum time, or until the body
exits by its own means (break, return, goto, etc.)

Example of usage:

LOOP_UNTIL_TIMEOUT(MAX_HW_TIME_US)
{
	if(psQueueInfo->ui32ReadOffset == psQueueInfo->ui32WriteOffset)
	{
		bTimeout = IMG_FALSE;
		break;
	}
	
	OSWaitus(MAX_HW_TIME_US/WAIT_TRY_COUNT);
} END_LOOP_UNTIL_TIMEOUT();

-----------------------------------------------------------------------------*//*	uiNotLastLoop will remain at 1 until the timeout has expired, at which time		
 * 	it will be decremented and the loop executed one final time. This is necessary
 *	when preemption is enabled. 
			   *//* PRQA S 3411,3431 12 *//* critical format, leave alone */
#define LOOP_UNTIL_TIMEOUT(TIMEOUT) \
{\
	IMG_UINT32 uiOffset, uiStart, uiCurrent; \
	IMG_INT32 iNotLastLoop;					 \
	for(uiOffset = 0, uiStart = OSClockus(), uiCurrent = uiStart + 1, iNotLastLoop = 1;\
		((uiCurrent - uiStart + uiOffset) < (TIMEOUT)) || iNotLastLoop--;				\
		uiCurrent = OSClockus(),													\
		uiOffset = uiCurrent < uiStart ? IMG_UINT32_MAX - uiStart : uiOffset,		\
		uiStart = uiCurrent < uiStart ? 0 : uiStart)
#define END_LOOP_UNTIL_TIMEOUT() \
}
/*!
 ******************************************************************************

 @Function		PVRSRVGetErrorStringKM

 @Description	Returns a text string relating to the PVRSRV_ERROR enum.

 ******************************************************************************/ IMG_IMPORT
const IMG_CHAR *PVRSRVGetErrorStringKM(PVRSRV_ERROR eError);

/*
	FIXME: This should be defined elsewhere when server sync are implemented
*/
typedef struct _SERVER_SYNC_PRIM_ {
	/* Placeholder until structure is properly implemented */
	IMG_UINT32 ui32Placeholder;
} SERVER_SYNC_PRIM;

#endif				/* SRVKM_H */
