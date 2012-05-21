									    /*************************************************************************//*!
									       @File           devicemem_heapcfg.c
									       @Title          Temporary Device Memory 2 stuff
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Device memory management
									       @License        Strictly Confidential.
    *//***************************************************************************/

/* our exported API */
#include "devicemem_heapcfg.h"

#include "device.h"		/*FIXME: - circular */
#include "img_types.h"
#include "pvr_debug.h"
#include "pvrsrv_error.h"
#include "osfunc.h"

PVRSRV_ERROR
HeapCfgHeapConfigCount(const PVRSRV_DEVICE_NODE * psDeviceNode,
		       IMG_UINT32 * puiNumHeapConfigsOut)
{

	*puiNumHeapConfigsOut = psDeviceNode->sDevMemoryInfo.uiNumHeapConfigs;

	return PVRSRV_OK;
}

PVRSRV_ERROR
HeapCfgHeapCount(const PVRSRV_DEVICE_NODE * psDeviceNode,
		 IMG_UINT32 uiHeapConfigIndex, IMG_UINT32 * puiNumHeapsOut)
{
	if (uiHeapConfigIndex >= psDeviceNode->sDevMemoryInfo.uiNumHeapConfigs) {
		return PVRSRV_ERROR_DEVICEMEM_INVALID_HEAP_CONFIG_INDEX;
	}

	*puiNumHeapsOut =
	    psDeviceNode->sDevMemoryInfo.
	    psDeviceMemoryHeapConfigArray[uiHeapConfigIndex].uiNumHeaps;

	return PVRSRV_OK;
}

PVRSRV_ERROR
HeapCfgHeapConfigName(const PVRSRV_DEVICE_NODE * psDeviceNode,
		      IMG_UINT32 uiHeapConfigIndex,
		      IMG_UINT32 uiHeapConfigNameBufSz,
		      IMG_CHAR * pszHeapConfigNameOut)
{
	if (uiHeapConfigIndex >= psDeviceNode->sDevMemoryInfo.uiNumHeapConfigs) {
		return PVRSRV_ERROR_DEVICEMEM_INVALID_HEAP_CONFIG_INDEX;
	}

	OSSNPrintf(pszHeapConfigNameOut, uiHeapConfigNameBufSz, "%s",
		   psDeviceNode->sDevMemoryInfo.
		   psDeviceMemoryHeapConfigArray[uiHeapConfigIndex].pszName);

	return PVRSRV_OK;
}

PVRSRV_ERROR
HeapCfgHeapDetails(const PVRSRV_DEVICE_NODE * psDeviceNode,
		   IMG_UINT32 uiHeapConfigIndex,
		   IMG_UINT32 uiHeapIndex,
		   IMG_UINT32 uiHeapNameBufSz,
		   IMG_CHAR * pszHeapNameOut,
		   IMG_DEV_VIRTADDR * psDevVAddrBaseOut,
		   IMG_DEVMEM_SIZE_T * puiHeapLengthOut,
		   IMG_UINT32 * puiLog2DataPageSizeOut)
{
	DEVMEM_HEAP_BLUEPRINT *psHeapBlueprint;

	if (uiHeapConfigIndex >= psDeviceNode->sDevMemoryInfo.uiNumHeapConfigs) {
		return PVRSRV_ERROR_DEVICEMEM_INVALID_HEAP_CONFIG_INDEX;
	}

	if (uiHeapIndex >=
	    psDeviceNode->sDevMemoryInfo.
	    psDeviceMemoryHeapConfigArray[uiHeapConfigIndex].uiNumHeaps) {
		return PVRSRV_ERROR_DEVICEMEM_INVALID_HEAP_INDEX;
	}

	psHeapBlueprint =
	    &psDeviceNode->sDevMemoryInfo.
	    psDeviceMemoryHeapConfigArray[uiHeapConfigIndex].
	    psHeapBlueprintArray[uiHeapIndex];

	OSSNPrintf(pszHeapNameOut, uiHeapNameBufSz, "%s",
		   psHeapBlueprint->pszName);
	*psDevVAddrBaseOut = psHeapBlueprint->sHeapBaseAddr;
	*puiHeapLengthOut = psHeapBlueprint->uiHeapLength;
	*puiLog2DataPageSizeOut = psHeapBlueprint->uiLog2DataPageSize;

	return PVRSRV_OK;
}
