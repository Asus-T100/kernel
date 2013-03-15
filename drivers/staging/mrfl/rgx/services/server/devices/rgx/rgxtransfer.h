/*************************************************************************/ /*!
@File
@Title          RGX Transfer queue Functionality
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the RGX Transfer queue Functionality
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

#if !defined(__RGXTRANSFER_H__)
#define __RGXTRANSFER_H__

#include "devicemem.h"
#include "device.h"
#include "rgxdevice.h"
#include "rgxfwutils.h"
#include "rgx_fwif_resetframework.h"


#if defined (__cplusplus)
extern "C" {
#endif

typedef struct {
	PVRSRV_DEVICE_NODE		*psDeviceNode;
	DEVMEM_MEMDESC			*psFWTQ2DContextMemDesc;
	DEVMEM_MEMDESC			*psFWFrameworkMemDesc;
	RGX_FWCOMCTX_CLEANUP	sFWComContextCleanup;
	PVRSRV_CLIENT_SYNC_PRIM	*psCleanupSync;
} RGX_TQ2D_CLEANUP_DATA;


/*!
*******************************************************************************

 @Function	PVRSRVRGXCreateTQ2DContextKM

 @Description
	Server-side implementation of RGXCreateTQ2DContext

 @Input pvDeviceNode - device node
 
FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateTQ2DContextKM(PVRSRV_DEVICE_NODE		*psDeviceNode,
										  DEVMEM_MEMDESC 			*psTQ2DCCBMemDesc,
										  DEVMEM_MEMDESC 			*psTQ2DCCBCtlMemDesc,
										  RGX_TQ2D_CLEANUP_DATA		**ppsCleanupData,
										  DEVMEM_MEMDESC 			**ppsFWTQ2DContextMemDesc,
										  IMG_UINT32				ui32Priority,
										  IMG_UINT32				ui32FrameworkRegisterSize,
										  IMG_PBYTE					pbyFrameworkRegisters,
										  IMG_HANDLE				hMemCtxPrivData);


/*!
*******************************************************************************

 @Function	PVRSRVRGXDestroyTQ2DContextKM

 @Description
	Server-side implementation of RGXDestroyTQ2DContext

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXDestroyTQ2DContextKM(RGX_TQ2D_CLEANUP_DATA *psCleanupData);


typedef struct {
	PVRSRV_DEVICE_NODE 		*psDeviceNode;
	DEVMEM_MEMDESC			*psFWTQ3DContextMemDesc;
	DEVMEM_MEMDESC			*psFWTQ3DContextStateMemDesc;
	DEVMEM_MEMDESC			*psFWFrameworkMemDesc;
	RGX_FWCOMCTX_CLEANUP	sFWComContextCleanup;
	PVRSRV_CLIENT_SYNC_PRIM	*psCleanupSync;
} RGX_TQ3D_CLEANUP_DATA;


/*!
*******************************************************************************

 @Function	PVRSRVRGXCreateTQ3DContextKM

 @Description
	Server-side implementation of RGXCreateTQ3DContext

 @Input pvDeviceNode - device node
 
FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateTQ3DContextKM(PVRSRV_DEVICE_NODE		*psDeviceNode,
										  DEVMEM_MEMDESC 			*psTQ3DCCBMemDesc,
										  DEVMEM_MEMDESC 			*psTQ3DCCBCtlMemDesc,
										  RGX_TQ3D_CLEANUP_DATA		**ppsCleanupData,
										  DEVMEM_MEMDESC 			**ppsFWTQ3DContextMemDesc,
										  DEVMEM_MEMDESC 			**ppsFWTQ3DContextStateMemDesc,
										  IMG_UINT32				ui32Priority,
										  IMG_DEV_VIRTADDR			sMCUFenceAddr,
										  IMG_UINT32				ui32FrameworkRegisterSize,
										  IMG_PBYTE					pbyFrameworkRegisters,
										  IMG_HANDLE				hMemCtxPrivData);


/*!
*******************************************************************************

 @Function	PVRSRVRGXDestroyTQ3DContextKM

 @Description
	Server-side implementation of RGXDestroyTQ3DContext

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXDestroyTQ3DContextKM(RGX_TQ3D_CLEANUP_DATA *psCleanupData);


/*!
*******************************************************************************

 @Function	PVRSRVSubmit2DKickKM

 @Description
	Schedules a 2D HW command on the firmware

 @Input pvDeviceNode - device node
 @Input psFWTQ2DContextMemDesc - memdesc for the TQ 2D render context
 @Input ui32TQ2DcCCBWoffUpdate - New fw Woff for the client TQ 2D CCB


 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVSubmitTQ2DKickKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
									DEVMEM_MEMDESC 		*psFWTQ2DContextMemDesc,
									IMG_UINT32			ui32TQ2DcCCBWoffUpdate,
									IMG_BOOL			bPDumpContinuous);


/*!
*******************************************************************************

 @Function	PVRSRVSubmitTQ3DKickKM

 @Description
	Schedules a TQ 3D HW command on the firmware

 @Input pvDeviceNode - device node
 @Input psFWTQ3DContextMemDesc - memdesc for the TQ 3D render context
 @Input ui32TQ3DcCCBWoffUpdate - New fw Woff for the client TQ 3D CCB

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVSubmitTQ3DKickKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
									DEVMEM_MEMDESC 		*psFWTQ3DContextMemDesc,
									IMG_UINT32			ui32TQ3DcCCBWoffUpdate,
									IMG_BOOL			bPDumpContinuous);


#endif /* __RGXTRANSFER_H__ */
