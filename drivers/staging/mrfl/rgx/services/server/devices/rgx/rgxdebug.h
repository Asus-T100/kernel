									    /*************************************************************************//*!
									       @File
									       @Title          RGX debug header file
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX debugging functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXDEBUG_H__)
#define __RGXDEBUG_H__

#include "pvrsrv_error.h"
#include "img_types.h"
#include "device.h"
#include "rgxdevice.h"

/*!
*******************************************************************************

 @Function	RGXPanic

 @Description

 Called when an unrecoverable situation is detected. Dumps RGX debug
 information and tells the OS to panic.

 @Input psDevInfo - RGX device info

 @Return IMG_VOID

******************************************************************************/
IMG_VOID RGXPanic(PVRSRV_RGXDEV_INFO * psDevInfo);

/*!
*******************************************************************************

 @Function	RGXDumpDebugInfo

 @Description

 Dump useful debugging info

 @Input psDevInfo	 - RGX device info
 @Input bDumpRGXRegs - Whether to dump RGX debug registers. Must not be done
 						when RGX is not powered.

 @Return   IMG_VOID

******************************************************************************/
IMG_VOID RGXDumpDebugInfo(PVRSRV_RGXDEV_INFO * psDevInfo,
			  IMG_BOOL bDumpRGXRegs);

#endif				/* __RGXDEBUG_H__ */
