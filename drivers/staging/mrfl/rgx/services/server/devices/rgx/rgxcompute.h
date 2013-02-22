/*************************************************************************/ /*!
@File
@Title          RGX compute functionality
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the RGX compute functionality
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

#if !defined(__RGXCOMPUTE_H__)
#define __RGXCOMPUTE_H__

#include "devicemem.h"
#include "device.h"
#include "rgxfwutils.h"
#include "rgx_fwif_resetframework.h"

#if defined (__cplusplus)
extern "C" {
#endif

typedef struct {
	PVRSRV_DEVICE_NODE 		*psDeviceNode;
	DEVMEM_MEMDESC			*psFWComputeContextMemDesc;
	DEVMEM_MEMDESC			*psFWFrameworkMemDesc;
	RGX_FWCOMCTX_CLEANUP 	sFWComContextCleanup;
	PVRSRV_CLIENT_SYNC_PRIM	*psCleanupSync;
} RGX_CC_CLEANUP_DATA;


/*!
*******************************************************************************
 @Function	PVRSRVRGXCreateComputeContextKM

 @Description
	

 @Input pvDeviceNode 
 @Input psCmpCCBMemDesc - 
 @Input psCmpCCBCtlMemDesc - 
 @Output ppsFWComputeContextMemDesc - 

 @Return   PVRSRV_ERROR
******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateComputeContextKM(PVRSRV_DEVICE_NODE			*psDeviceNode,
											 DEVMEM_MEMDESC				*psCmpCCBMemDesc,
											 DEVMEM_MEMDESC				*psCmpCCBCtlMemDesc,
											 RGX_CC_CLEANUP_DATA		**ppsCleanupData,
											 DEVMEM_MEMDESC				**ppsFWComputeContextMemDesc,
											 IMG_UINT32					ui32Priority,
											 IMG_DEV_VIRTADDR			sMCUFenceAddr,
											 IMG_UINT32					ui32FrameworkRegisterSize,
											 IMG_PBYTE					pbyFrameworkRegisters,
											 IMG_HANDLE					hMemCtxPrivData);

/*! 
*******************************************************************************
 @Function	PVRSRVRGXDestroyComputeContextKM

 @Description
	Server-side implementation of RGXDestroyComputeContext

 @Input psCleanupData - 

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXDestroyComputeContextKM(RGX_CC_CLEANUP_DATA *psCleanupData);


/*!
*******************************************************************************
 @Function	PVRSRVRGXKickCDMKM

 @Description
	Server-side implementation of RGXKickCDM

 @Input psDeviceNode - RGX Device node
 @Input psFWComputeContextMemDesc - Mem desc for firmware compute context
 @Input ui32cCCBWoffUpdate - New fw Woff for the client CDM CCB

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXKickCDMKM(PVRSRV_DEVICE_NODE	*psDeviceNode,
								DEVMEM_MEMDESC 		*psFWComputeContextMemDesc,
								IMG_UINT32			ui32cCCBWoffUpdate,
								IMG_BOOL			bbPDumpContinuous);
								
/*!
*******************************************************************************
 @Function	PVRSRVRGXFlushComputeDataKM

 @Description
	Server-side implementation of RGXFlushComputeData

 @Input psDeviceNode - RGX Device node

 @Return   PVRSRV_ERROR
******************************************************************************/
PVRSRV_ERROR PVRSRVRGXFlushComputeDataKM(PVRSRV_DEVICE_NODE *psDeviceNode,  DEVMEM_MEMDESC *psFWContextMemDesc);

#endif /* __RGXCOMPUTE_H__ */
