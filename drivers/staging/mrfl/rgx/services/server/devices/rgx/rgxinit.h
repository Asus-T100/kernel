/*************************************************************************/ /*!
@File
@Title          RGX initialisation header file
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the RGX initialisation
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

#if !defined(__RGXINIT_H__)
#define __RGXINIT_H__

#include "pvrsrv_error.h"
#include "img_types.h"
#include "rgxscript.h"
#include "device.h"
#include "rgxdevice.h"

#if defined (__cplusplus)
extern "C" {
#endif


/*!
*******************************************************************************

 @Function	PVRSRVRGXInitDevPart2KM

 @Description

 Second part of server-side RGX initialisation

 @Input pvDeviceNode - device node

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_IMPORT
PVRSRV_ERROR PVRSRVRGXInitDevPart2KM (PVRSRV_DEVICE_NODE	*psDeviceNode,
									  RGX_INIT_COMMAND		*psInitScript,
									  RGX_INIT_COMMAND		*psDbgScript,
									  RGX_INIT_COMMAND		*psDeinitScript,
									  IMG_UINT32			ui32KernelCatBase,
									  RGX_ACTIVEPM_CONF		eActivePMConf);


/*!
*******************************************************************************

 @Function	PVRSRVRGXInitFirmwareKM

 @Description

 Server-side RGX firmware initialisation

 @Input pvDeviceNode - device node

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_IMPORT
PVRSRV_ERROR PVRSRVRGXInitFirmwareKM(PVRSRV_DEVICE_NODE			*psDeviceNode, 
									 IMG_DEVMEM_SIZE_T 			ui32FWMemAllocSize,
									 DEVMEM_EXPORTCOOKIE		**ppsFWMemAllocServerExportCookie,
									 IMG_DEV_VIRTADDR			*psFWMemDevVAddrBase,
									 IMG_UINT64					*pui64FWHeapBase,
									 RGXFWIF_DEV_VIRTADDR		*psRGXFwInit,
									 IMG_BOOL					bEnableSignatureChecks,
									 IMG_UINT32					ui32SignatureChecksBufSize,
									 IMG_UINT32					ui32RGXFWAlignChecksSize,
									 IMG_UINT32					*pui32RGXFWAlignChecks,
									 IMG_UINT32					ui32ConfigFlags,
									 IMG_UINT32					ui32LogType,
									 RGXFWIF_COMPCHECKS_BVNC	*psClientBVNC);

/*!
*******************************************************************************

 @Function	RGXRegisterDevice

 @Description

 Registers the device with the system

 @Input: 	psDeviceNode - device node

 @Return   PVRSRV_ERROR :

******************************************************************************/
PVRSRV_ERROR RGXRegisterDevice(PVRSRV_DEVICE_NODE *psDeviceNode);


/*!
*******************************************************************************

 @Function	DevDeInitRGX

 @Description

 Reset and deinitialise Chip

 @Input pvDeviceNode - device info. structure

 @Return   PVRSRV_ERROR

******************************************************************************/
PVRSRV_ERROR DevDeInitRGX(PVRSRV_DEVICE_NODE *psDeviceNode);


/* FIXME move RGXPanic and RGXDumpDebugInfo somewhere more appropriate */
IMG_VOID RGXPanic(PVRSRV_RGXDEV_INFO	*psDevInfo);

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
IMG_VOID RGXDumpDebugInfo(PVRSRV_RGXDEV_INFO	*psDevInfo,
						  IMG_BOOL				bDumpRGXRegs);

#endif /* __RGXINIT_H__ */
