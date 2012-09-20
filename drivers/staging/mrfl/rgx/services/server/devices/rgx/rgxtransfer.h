									    /*************************************************************************//*!
									       @File
									       @Title          RGX Transfer queue Functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX Transfer queue Functionality
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXTRANSFER_H__)
#define __RGXTRANSFER_H__

#include "devicemem.h"
#include "device.h"
#include "rgxdevice.h"
#include "rgxfwutils.h"

#if defined (__cplusplus)
extern "C" {
#endif

	typedef struct {
		PVRSRV_DEVICE_NODE *psDeviceNode;
		DEVMEM_MEMDESC *psFWTQ2DContextMemDesc;
		RGX_FWCOMCTX_CLEANUP sFWComContextCleanup;
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
	    PVRSRV_ERROR PVRSRVRGXCreateTQ2DContextKM(PVRSRV_DEVICE_NODE *
						      psDeviceNode,
						      DEVMEM_MEMDESC *
						      psTQ2DCCBMemDesc,
						      DEVMEM_MEMDESC *
						      psTQ2DCCBCtlMemDesc,
						      RGX_TQ2D_CLEANUP_DATA **
						      ppsCleanupData,
						      DEVMEM_MEMDESC **
						      ppsFWTQ2DContextMemDesc,
						      IMG_UINT32 ui32Priority,
						      IMG_HANDLE
						      hMemCtxPrivData);

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
	    PVRSRV_ERROR PVRSRVRGXDestroyTQ2DContextKM(RGX_TQ2D_CLEANUP_DATA *
						       psCleanupData);

	typedef struct {
		PVRSRV_DEVICE_NODE *psDeviceNode;
		DEVMEM_MEMDESC *psFWTQ3DContextMemDesc;
		DEVMEM_MEMDESC *psFWTQ3DContextStateMemDesc;
		RGX_FWCOMCTX_CLEANUP sFWComContextCleanup;
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
	    PVRSRV_ERROR PVRSRVRGXCreateTQ3DContextKM(PVRSRV_DEVICE_NODE *
						      psDeviceNode,
						      DEVMEM_MEMDESC *
						      psTQ3DCCBMemDesc,
						      DEVMEM_MEMDESC *
						      psTQ3DCCBCtlMemDesc,
						      RGX_TQ3D_CLEANUP_DATA **
						      ppsCleanupData,
						      DEVMEM_MEMDESC **
						      ppsFWTQ3DContextMemDesc,
						      DEVMEM_MEMDESC **
						      ppsFWTQ3DContextStateMemDesc,
						      IMG_UINT32 ui32Priority,
						      IMG_DEV_VIRTADDR
						      sMCUFenceAddr,
						      IMG_HANDLE
						      hMemCtxPrivData);

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
	    PVRSRV_ERROR PVRSRVRGXDestroyTQ3DContextKM(RGX_TQ3D_CLEANUP_DATA *
						       psCleanupData);

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
	    PVRSRV_ERROR PVRSRVSubmitTQ2DKickKM(PVRSRV_DEVICE_NODE *
						psDeviceNode,
						DEVMEM_MEMDESC *
						psFWTQ2DContextMemDesc,
						IMG_UINT32
						ui32TQ2DcCCBWoffUpdate,
						IMG_BOOL bPDumpContinuous);

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
	    PVRSRV_ERROR PVRSRVSubmitTQ3DKickKM(PVRSRV_DEVICE_NODE *
						psDeviceNode,
						DEVMEM_MEMDESC *
						psFWTQ3DContextMemDesc,
						IMG_UINT32
						ui32TQ3DcCCBWoffUpdate,
						IMG_BOOL bPDumpContinuous);

#endif				/* __RGXTRANSFER_H__ */
