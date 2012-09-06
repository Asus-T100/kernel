									    /*************************************************************************//*!
									       @File
									       @Title          Device specific utility routines declarations
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Inline functions/structures specific to RGX
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "device.h"
#include "rgxdevice.h"
#include "pvrsrv.h"

/*!
******************************************************************************

 @Function	RGXScheduleProcessQueuesKM

 @Description - Software command complete handler

 @Input psDeviceNode - RGX device node

******************************************************************************/
IMG_IMPORT
    IMG_VOID RGXScheduleProcessQueuesKM(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle);

/*!
******************************************************************************

 @Function	RGXIsDevicePowered

 @Description

	Whether the device is powered, for the purposes of lockup detection.

 @Input psDeviceNode - pointer to device node

 @Return   IMG_BOOL  : Whether device is powered

******************************************************************************/
IMG_IMPORT IMG_BOOL RGXIsDevicePowered(PVRSRV_DEVICE_NODE * psDeviceNode);

/*!
******************************************************************************

 @Function	RGXRunScript

 @Description Execuute the commands in the script

 @Input 

 @Return   PVRSRV_ERROR

******************************************************************************/
PVRSRV_ERROR RGXRunScript(PVRSRV_RGXDEV_INFO * psDevInfo,
			  RGX_INIT_COMMAND * psScript,
			  IMG_UINT32 ui32NumCommands,
			  IMG_UINT32 ui32PdumpFlags);

/*!
******************************************************************************

 @Function	RGXRequestMemoryContextCleanUp

 @Description Schedules a FW memory context cleanup and wait until its completion

 @Input psDeviceNode - pointer to device node

 @Input psFWContext - firmware address of the context to be cleaned up

 @Input eDM - Data master, to which the cleanup command should be send

******************************************************************************/
PVRSRV_ERROR RGXRequestMemoryContextCleanUp(PVRSRV_DEVICE_NODE * psDeviceNode,
					    PRGXFWIF_FWCOMMONCONTEXT
					    psFWContext, RGXFWIF_DM eDM);

/*!
******************************************************************************

 @Function	RGXRequestHWRTDataCleanUp

 @Description Schedules a FW HWRTData memory cleanup and wait until its completion

 @Input psDeviceNode - pointer to device node

 @Input psHWRTData - firmware address of the HWRTData to be cleaned up

 @Input eDM - Data master, to which the cleanup command should be send

 ******************************************************************************/
PVRSRV_ERROR RGXRequestHWRTDataCleanUp(PVRSRV_DEVICE_NODE * psDeviceNode,
				       PRGXFWIF_HWRTDATA psHWRTData,
				       RGXFWIF_DM eDM);

/******************************************************************************
 End of file (rgxutils.h)
******************************************************************************/
