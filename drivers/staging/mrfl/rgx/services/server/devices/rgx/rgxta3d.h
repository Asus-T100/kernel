									    /*************************************************************************//*!
									       @File
									       @Title          RGX TA and 3D Functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX TA and 3D Functionality
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXTA3D_H__)
#define __RGXTA3D_H__

#include "devicemem.h"
#include "device.h"
#include "rgxdevice.h"
#include "rgx_fwif_shared.h"
#include "rgxfwutils.h"

#if defined (__cplusplus)
extern "C" {
#endif

	typedef struct {
		PVRSRV_DEVICE_NODE *psDeviceNode;
		DEVMEM_MEMDESC *psFWRenderContextMemDesc;
		DEVMEM_MEMDESC *psFWRenderContextStateMemDesc;	/*!< TA/3D context suspend state */
		RGX_FWCOMCTX_CLEANUP sFWTAContextCleanup;
		RGX_FWCOMCTX_CLEANUP sFW3DContextCleanup;
	} RGX_RC_CLEANUP_DATA;

	typedef struct {
		PVRSRV_DEVICE_NODE *psDeviceNode;
		DEVMEM_MEMDESC *psFWHWRTDataMemDesc;
	} RGX_RTDATA_CLEANUP_DATA;

/* Create HWRTDataSet */
	 IMG_EXPORT
	    PVRSRV_ERROR RGXCreateHWRTData(PVRSRV_DEVICE_NODE * psDeviceNode,
					   IMG_UINT32 psRenderTarget,
					   IMG_DEV_VIRTADDR psPMMListDevVAddr,
					   IMG_UINT32
					   apsFreeLists[RGX_NUM_FREELIST_TYPES],
					   RGX_RTDATA_CLEANUP_DATA **
					   ppsCleanupData,
					   DEVMEM_MEMDESC ** psMemDesc,
					   IMG_UINT32 * puiHWRTData);

/* Destroy HWRTData */
	 IMG_EXPORT
	    PVRSRV_ERROR RGXDestroyHWRTData(RGX_RTDATA_CLEANUP_DATA *
					    psCleanupData);

/* Create Render Target */
	 IMG_EXPORT
	    PVRSRV_ERROR RGXCreateRenderTarget(PVRSRV_DEVICE_NODE *
					       psDeviceNode,
					       IMG_DEV_VIRTADDR
					       psVHeapTableDevVAddr,
					       DEVMEM_MEMDESC **
					       psRenderTargetMemDesc,
					       IMG_UINT32 *
					       sRenderTargetFWDevVAddr);

/* Destroy render target */
	 IMG_EXPORT
	    PVRSRV_ERROR RGXDestroyRenderTarget(DEVMEM_MEMDESC * psMemDesc);

/* Create free list */
	 IMG_EXPORT
	    PVRSRV_ERROR RGXCreateFreeList(PVRSRV_DEVICE_NODE * psDeviceNode,
					   IMG_UINT32 ui32TotalPMPages,
					   IMG_DEV_VIRTADDR psFreeListDevVAddr,
					   DEVMEM_MEMDESC **
					   psFWFreeListMemDesc,
					   IMG_UINT32 * sFreeListFWDevVAddr);

/* Destroy free list */
	 IMG_EXPORT PVRSRV_ERROR RGXDestroyFreeList(DEVMEM_MEMDESC * psMemDesc);
/*!
*******************************************************************************

 @Function	PVRSRVRGXCreateRenderContextKM

 @Description
	Server-side implementation of RGXCreateRenderContext

 @Input pvDeviceNode - device node
 
FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
	 IMG_EXPORT
	    PVRSRV_ERROR PVRSRVRGXCreateRenderContextKM(PVRSRV_DEVICE_NODE *
							psDeviceNode,
							DEVMEM_MEMDESC *
							psTACCBMemDesc,
							DEVMEM_MEMDESC *
							psTACCBCtlMemDesc,
							DEVMEM_MEMDESC *
							ps3DCCBMemDesc,
							DEVMEM_MEMDESC *
							ps3DCCBCtlMemDesc,
							RGX_RC_CLEANUP_DATA **
							ppsCleanupData,
							DEVMEM_MEMDESC **
							ppsFWRenderContextMemDesc,
							DEVMEM_MEMDESC **
							ppsFWContextStateMemDesc,
							IMG_UINT32 ui32Priority,
							IMG_DEV_VIRTADDR
							sMCUFenceAddr,
							IMG_DEV_VIRTADDR
							psVDMStackPointer,
							IMG_HANDLE
							hMemCtxPrivData);

/*!
*******************************************************************************

 @Function	PVRSRVRGXDestroyRenderContextKM

 @Description
	Server-side implementation of RGXDestroyRenderContext

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
	 IMG_EXPORT
	    PVRSRV_ERROR PVRSRVRGXDestroyRenderContextKM(RGX_RC_CLEANUP_DATA *
							 psCleanupData);

/*!
*******************************************************************************

 @Function	PVRSRVRGXKickTA3DKM

 @Description
	Server-side implementation of RGXKickTA3D

 @Input pvDeviceNode - device node
 @Input psFWRenderContextMemDesc - memdesc for the firmware render context
 @Input bLastTAInScene - IMG_TRUE if last TA in scene (terminate or abort)
 @Input bKickTA - IMG_TRUE to kick the TA
 @Input bKick3D - IMG_TRUE to kick the 3D
 @Input ui32TAcCCBWoffUpdate - New fw Woff for the client TA CCB
 @Input ui323DcCCBWoffUpdate - New fw Woff for the client 3D CCB

 @Return   PVRSRV_ERROR

******************************************************************************/
	 IMG_EXPORT
	    PVRSRV_ERROR PVRSRVRGXKickTA3DKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					     DEVMEM_MEMDESC *
					     psFWRenderContextMemDesc,
					     IMG_BOOL bLastTAInScene,
					     IMG_BOOL bKickTA, IMG_BOOL bKick3D,
					     IMG_UINT32 ui32TAcCCBWoffUpdate,
					     IMG_UINT32 ui323DcCCBWoffUpdate);

#endif				/* __RGXTA3D_H__ */
