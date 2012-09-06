									    /*************************************************************************//*!
									       @File
									       @Title          RGX pdump Functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX pdump functionality
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "img_defs.h"
#include "rgxdefs.h"
#include "pvrsrv_error.h"
#include "rgxdevice.h"
#include "device.h"
#include "devicemem.h"
#include "pdump_km.h"
#include "pvr_debug.h"

#if defined(PDUMP)
/*!
******************************************************************************

 @Function	PVRSRVPDumpSignatureBufferKM

 @Description

 Dumps TA and 3D signature and checksum buffers

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
    PVRSRV_ERROR PVRSRVPDumpSignatureBufferKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					      IMG_UINT32 ui32PDumpFlags);

/*!
******************************************************************************

 @Function	PVRSRVPDumpTraceBufferKM

 @Description

 Dumps TA and 3D signature and checksum buffers

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
    PVRSRV_ERROR PVRSRVPDumpTraceBufferKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					  IMG_UINT32 ui32PDumpFlags);
#else				/* PDUMP */

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpSignatureBufferKM)
#endif
static INLINE PVRSRV_ERROR
PVRSRVPDumpSignatureBufferKM(PVRSRV_DEVICE_NODE * psDeviceNode,
			     IMG_UINT32 ui32PDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psDeviceNode);
	PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);
	return PVRSRV_OK;
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVPDumpTraceBufferKM)
#endif
static INLINE PVRSRV_ERROR
PVRSRVPDumpTraceBufferKM(PVRSRV_DEVICE_NODE * psDeviceNode,
			 IMG_UINT32 ui32PDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psDeviceNode);
	PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);
	return PVRSRV_OK;
}
#endif				/* PDUMP */
/******************************************************************************
 End of file (rgxpdump.h)
******************************************************************************/
