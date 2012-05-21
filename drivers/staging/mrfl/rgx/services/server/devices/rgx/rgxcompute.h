									    /*************************************************************************//*!
									       @File
									       @Title          RGX compute functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX compute functionality
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXCOMPUTE_H__)
#define __RGXCOMPUTE_H__

#include "devicemem.h"
#include "device.h"
#include "rgxfwutils.h"

#if defined (__cplusplus)
extern "C" {
#endif

	typedef struct {
		PVRSRV_DEVICE_NODE *psDeviceNode;
		DEVMEM_MEMDESC *psFWComputeContextMemDesc;
		RGX_FWCOMCTX_CLEANUP sFWComContextCleanup;
	} RGX_CC_CLEANUP_DATA;

/*!
*******************************************************************************
 @Function	PVRSRVRGXCreateComputeContextKM

 @Description
	Server-side implementation of RGXCreateComputeContext

 @Input pvDeviceNode - device node
 @Input psCmpCCBMemDesc - Mem desc for compute client CCB 
 @Input psCmpCCBCtlMemDesc - Mem desc for compute client CCB control
 @Output ppsFWComputeContextMemDesc - Mem desc for firmware compute context

 @Return   PVRSRV_ERROR
******************************************************************************/
	 IMG_EXPORT
	    PVRSRV_ERROR PVRSRVRGXCreateComputeContextKM(PVRSRV_DEVICE_NODE *
							 psDeviceNode,
							 DEVMEM_MEMDESC *
							 psCmpCCBMemDesc,
							 DEVMEM_MEMDESC *
							 psCmpCCBCtlMemDesc,
							 RGX_CC_CLEANUP_DATA **
							 ppsCleanupData,
							 DEVMEM_MEMDESC **
							 ppsFWComputeContextMemDesc,
							 IMG_UINT32
							 ui32Priority,
							 IMG_DEV_VIRTADDR
							 sMCUFenceAddr,
							 IMG_HANDLE
							 hMemCtxPrivData);

/*! 
*******************************************************************************
 @Function	PVRSRVRGXDestroyComputeContextKM

 @Description
	Server-side implementation of RGXDestroyComputeContext

 @Input psCleanupData - 

 @Return   PVRSRV_ERROR
******************************************************************************/
	PVRSRV_ERROR PVRSRVRGXDestroyComputeContextKM(RGX_CC_CLEANUP_DATA *
						      psCleanupData);

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
	PVRSRV_ERROR PVRSRVRGXKickCDMKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					DEVMEM_MEMDESC *
					psFWComputeContextMemDesc,
					IMG_UINT32 ui32cCCBWoffUpdate);

/*!
*******************************************************************************
 @Function	PVRSRVRGXFlushComputeDataKM

 @Description
	Server-side implementation of RGXFlushComputeData

 @Input psDeviceNode - RGX Device node

 @Return   PVRSRV_ERROR
******************************************************************************/
	PVRSRV_ERROR PVRSRVRGXFlushComputeDataKM(PVRSRV_DEVICE_NODE *
						 psDeviceNode,
						 DEVMEM_MEMDESC *
						 psFWContextMemDesc);

#endif				/* __RGXCOMPUTE_H__ */
