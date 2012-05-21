									    /*************************************************************************//*!
									       @File
									       @Title          RGX initialisation header file
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX initialisation
									       @License        Strictly Confidential.
    *//**************************************************************************/

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
	    PVRSRV_ERROR PVRSRVRGXInitDevPart2KM(PVRSRV_DEVICE_NODE *
						 psDeviceNode,
						 RGX_INIT_COMMAND *
						 psInitScript,
						 RGX_INIT_COMMAND * psDbgScript,
						 RGX_INIT_COMMAND *
						 psDeinitScript,
						 IMG_UINT32 ui32KernelCatBase);

/*!
*******************************************************************************

 @Function	PVRSRVRGXInitFirmwareKM

 @Description

 Server-side RGX firmware initialisation

 @Input pvDeviceNode - device node

 @Return   PVRSRV_ERROR

******************************************************************************/
	IMG_IMPORT
	    PVRSRV_ERROR PVRSRVRGXInitFirmwareKM(PVRSRV_DEVICE_NODE *
						 psDeviceNode,
						 IMG_DEVMEM_SIZE_T
						 ui32FWMemAllocSize,
						 DEVMEM_EXPORTCOOKIE **
						 ppsFWMemAllocServerExportCookie,
						 IMG_DEV_VIRTADDR *
						 psFWMemDevVAddrBase,
						 IMG_UINT64 * pui64FWHeapBase,
						 RGXFWIF_DEV_VIRTADDR *
						 psRGXFwInit,
						 IMG_BOOL
						 bEnableSignatureChecks,
						 IMG_UINT32
						 ui32SignatureChecksBufSize,
						 IMG_UINT32
						 ui32RGXFWAlignChecksSize,
						 IMG_UINT32 *
						 pui32RGXFWAlignChecks,
						 IMG_UINT32 ui32ConfigFlags);

/*!
*******************************************************************************

 @Function	RGXRegisterDevice

 @Description

 Registers the device with the system

 @Input: 	psDeviceNode - device node

 @Return   PVRSRV_ERROR :

******************************************************************************/
	PVRSRV_ERROR RGXRegisterDevice(PVRSRV_DEVICE_NODE * psDeviceNode);

/*!
*******************************************************************************

 @Function	DevDeInitRGX

 @Description

 Reset and deinitialise Chip

 @Input pvDeviceNode - device info. structure

 @Return   PVRSRV_ERROR

******************************************************************************/
	PVRSRV_ERROR DevDeInitRGX(PVRSRV_DEVICE_NODE * psDeviceNode);

/* FIXME move RGXPanic and RGXDumpDebugInfo somewhere more appropriate */
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

#endif				/* __RGXINIT_H__ */
