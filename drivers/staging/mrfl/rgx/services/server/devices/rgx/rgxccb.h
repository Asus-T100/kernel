									    /*************************************************************************//*!
									       @File
									       @Title          RGX Circular Command Buffer functionality.
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX Circular Command Buffer functionality.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXCCB_H__)
#define __RGXCCB_H__

#include "devicemem.h"
#include "device.h"

#if defined (__cplusplus)
extern "C" {
#endif

	typedef struct {
		DEVMEM_MEMDESC *psClientCCBMemDesc;
		DEVMEM_MEMDESC *psClientCCBCtlMemDesc;
		DEVMEM_EXPORTCOOKIE sClientCCBExportCookie;
		DEVMEM_EXPORTCOOKIE sClientCCBCtlExportCookie;
	} RGX_CCB_CLEANUP_DATA;

/*!
*******************************************************************************

 @Function	PVRSRVRGXCreateCCBKM

 @Description
	Server-side implementation of RGXCreateCCB

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR PVRSRVRGXCreateCCBKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					      IMG_UINT32 ui32AllocSize,
					      IMG_UINT32 ui32AllocAlignment,
					      RGX_CCB_CLEANUP_DATA **
					      ppsCleanupData,
					      DEVMEM_MEMDESC **
					      ppsClientCCBMemDesc,
					      DEVMEM_MEMDESC **
					      ppsClientCCBCtlMemDesc,
					      DEVMEM_EXPORTCOOKIE **
					      psClientCCBExportCookie,
					      DEVMEM_EXPORTCOOKIE **
					      psClientCCBCtlExportCookie);

/*!
*******************************************************************************

 @Function	PVRSRVRGXDestroyCCBKM

 @Description
	Server-side implementation of RGXDestroyCCB

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
	 IMG_IMPORT
	    PVRSRV_ERROR PVRSRVRGXDestroyCCBKM(RGX_CCB_CLEANUP_DATA *
					       psCleanupData);

#endif				/* __RGXCCB_H__ */
