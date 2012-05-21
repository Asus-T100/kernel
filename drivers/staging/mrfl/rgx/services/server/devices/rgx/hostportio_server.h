									    /*************************************************************************//*!
									       @File
									       @Title          Hostport services functions interface
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Kernel services functions for hostport I/O:
									       Reading from and Writing to the hostport from the host.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if ! defined(HOSTPORTIO_KM_H)
#define HOSTPORTIO_KM_H

#include <img_defs.h>
#include <pvrsrv_error.h>
#include <device.h>

IMG_EXPORT PVRSRV_ERROR
PVRSRVHostPortReadKM(PVRSRV_DEVICE_NODE * psDeviceNode,
		     IMG_HANDLE hMemCtxPrivData,
		     IMG_UINT32 ui32CRHostIFVal,
		     IMG_UINT32 ui32ReadOffset,
		     IMG_DEVMEM_SIZE_T uiDstBufLen,
		     IMG_CHAR * pDstBuffer,
		     IMG_DEVMEM_SIZE_T * puiNumBytesRead);

IMG_EXPORT PVRSRV_ERROR
PVRSRVHostPortWriteKM(PVRSRV_DEVICE_NODE * psDeviceNode,
		      IMG_HANDLE hMemCtxPrivData,
		      IMG_UINT32 ui32CRHostIFVal,
		      IMG_UINT32 ui32WriteOffset,
		      IMG_DEVMEM_SIZE_T uiSrcBufLen,
		      IMG_CHAR * pSrcBuffer,
		      IMG_DEVMEM_SIZE_T * puiNumBytesWritten);

#endif
