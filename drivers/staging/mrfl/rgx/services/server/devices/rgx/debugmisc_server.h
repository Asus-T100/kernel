									    /*************************************************************************//*!
									       @File
									       @Title          Debugging and miscellaneous functions server interface
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Kernel services functions for debugging and other
									       miscellaneous functionality.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if ! defined(DEBUGMISC_SERVER_H)
#define DEBUGMISC_SERVER_H

#include <img_defs.h>
#include <pvrsrv_error.h>
#include <device.h>

IMG_EXPORT PVRSRV_ERROR
PVRSRVDebugMiscTilingSetStateKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				IMG_HANDLE hMemCtxPrivData, IMG_BOOL bEnabled);

IMG_EXPORT PVRSRV_ERROR
PVRSRVDebugMiscSLCSetBypassStateKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				   IMG_UINT32 uiFlags, IMG_BOOL bSetBypassed);

IMG_EXPORT PVRSRV_ERROR
PVRSRVRGXDebugMiscSetFWLogKM(PVRSRV_DEVICE_NODE * psDeviceNode,
			     IMG_UINT32 ui32RGXFWLogType);
#endif
