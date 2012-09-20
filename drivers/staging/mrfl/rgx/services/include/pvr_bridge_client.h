									    /*************************************************************************//*!
									       @File
									       @Title          PVR Bridge Functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the PVR Bridge code
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __PVR_BRIDGE_U_H__
#define __PVR_BRIDGE_U_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include "img_types.h"
#include "pvrsrv_error.h"

/******************************************************************************
 * Function prototypes 
 *****************************************************************************/

	PVRSRV_ERROR OpenServices(IMG_HANDLE * phServices,
				  IMG_UINT32 ui32SrvFlags);
	PVRSRV_ERROR CloseServices(IMG_HANDLE hServices);
	IMG_RESULT PVRSRVBridgeCall(IMG_HANDLE hServices,
				    IMG_UINT32 ui32FunctionID,
				    IMG_VOID * pvParamIn,
				    IMG_UINT32 ui32InBufferSize,
				    IMG_VOID * pvParamOut,
				    IMG_UINT32 ui32OutBufferSize);

#if defined (__cplusplus)
}
#endif
#endif				/* __PVR_BRIDGE_U_H__ */
/******************************************************************************
 End of file (pvr_bridge_u.h)
******************************************************************************/
