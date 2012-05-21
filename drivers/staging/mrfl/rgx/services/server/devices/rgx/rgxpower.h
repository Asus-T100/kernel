									    /*************************************************************************//*!
									       @File
									       @Title          RGX power header file
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX power
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXPOWER_H__)
#define __RGXPOWER_H__

#include "pvrsrv_error.h"
#include "img_types.h"
#include "servicesext.h"

#if defined (__cplusplus)
extern "C" {
#endif

/*!
******************************************************************************

 @Function	RGXPrePowerState

 @Description

 does necessary preparation before power state transition

 @Input	   hDevHandle : RGX Device Node
 @Input	   eNewPowerState : New power state
 @Input	   eCurrentPowerState : Current power state

 @Return   PVRSRV_ERROR :

******************************************************************************/
	PVRSRV_ERROR RGXPrePowerState(IMG_HANDLE hDevHandle,
				      PVRSRV_DEV_POWER_STATE eNewPowerState,
				      PVRSRV_DEV_POWER_STATE
				      eCurrentPowerState);

/*!
******************************************************************************

 @Function	RGXPostPowerState

 @Description

 does necessary preparation after power state transition

 @Input	   hDevHandle : RGX Device Node
 @Input	   eNewPowerState : New power state
 @Input	   eCurrentPowerState : Current power state

 @Return   PVRSRV_ERROR :

******************************************************************************/
	PVRSRV_ERROR RGXPostPowerState(IMG_HANDLE hDevHandle,
				       PVRSRV_DEV_POWER_STATE eNewPowerState,
				       PVRSRV_DEV_POWER_STATE
				       eCurrentPowerState);

/*!
******************************************************************************

 @Function	RGXPreClockSpeedChange

 @Description

	Does processing required before an RGX clock speed change.

 @Input	   hDevHandle : RGX Device Node
 @Input	   bIdleDevice : Whether the firmware needs to be idled
 @Input	   eCurrentPowerState : Power state of the device

 @Return   PVRSRV_ERROR :

******************************************************************************/
	PVRSRV_ERROR RGXPreClockSpeedChange(IMG_HANDLE hDevHandle,
					    IMG_BOOL bIdleDevice,
					    PVRSRV_DEV_POWER_STATE
					    eCurrentPowerState);

/*!
******************************************************************************

 @Function	RGXPostClockSpeedChange

 @Description

	Does processing required after an RGX clock speed change.

 @Input	   hDevHandle : RGX Device Node
 @Input	   bIdleDevice : Whether the firmware had been idled previously
 @Input	   eCurrentPowerState : Power state of the device

 @Return   PVRSRV_ERROR :

******************************************************************************/
	PVRSRV_ERROR RGXPostClockSpeedChange(IMG_HANDLE hDevHandle,
					     IMG_BOOL bIdleDevice,
					     PVRSRV_DEV_POWER_STATE
					     eCurrentPowerState);

#endif				/* __RGXPOWER_H__ */
