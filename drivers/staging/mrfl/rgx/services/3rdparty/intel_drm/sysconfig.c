									     /**************************************************************************//*!
									        @File           sysconfig.c
									        @Title          Sysconfig layer for Emu
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Implements the system layer for the emulator
    *//***************************************************************************/

#include <drm/drmP.h>
#include "img_types.h"
#include "pvrsrv_device.h"
#include "syscommon.h"
#include "sysconfig.h"
#include "allocmem.h"
#include "pvr_debug.h"
#include "osfunc.h"

#include "pci_support.h"

typedef struct _PLAT_DATA_ {
	IMG_HANDLE hRGXPCI;

	struct drm_device *psDRMDev;
} PLAT_DATA;

PLAT_DATA *gpsPlatData = IMG_NULL;
extern struct drm_device *gpsPVRDRMDev;

/*
	PCIInitDev
*/
static PVRSRV_ERROR PCIInitDev(PLAT_DATA * psPlatData)
{
	PVRSRV_DEVICE_CONFIG *psDevice = &sSysConfig.pasDevices[0];
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32MaxOffset;
	IMG_UINT32 ui32BaseAddr = 0;

	psPlatData->psDRMDev = gpsPVRDRMDev;
	if (!psPlatData->psDRMDev) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: DRM device not initialized"));
		return PVRSRV_ERROR_NOT_SUPPORTED;
	}

	if (!IS_MRFLD(psPlatData->psDRMDev)) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device 0x%08x not supported",
			 psPlatData->psDRMDev->pci_device));
		return PVRSRV_ERROR_NOT_SUPPORTED;
	}

	psPlatData->hRGXPCI =
	    OSPCISetDev((IMG_VOID *) psPlatData->psDRMDev->pdev, 0);
	if (!psPlatData->hRGXPCI) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Failed to acquire PCI device"));
		return PVRSRV_ERROR_PCI_DEVICE_NOT_FOUND;
	}

	ui32MaxOffset = OSPCIAddrRangeLen(psPlatData->hRGXPCI, 0);
	if (ui32MaxOffset < (RGX_REG_OFFSET + RGX_REG_SIZE)) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region 0x%08x isn't big enough",
			 ui32MaxOffset));
		return PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
	}
	PVR_DPF((PVR_DBG_WARNING, "PCIInitDev: Device memory region len 0x%08x",
		 ui32MaxOffset));

	/* Reserve the address range */
	if (OSPCIRequestAddrRange(psPlatData->hRGXPCI, 0) != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region not available"));
		return PVRSRV_ERROR_PCI_REGION_UNAVAILABLE;

	}

	ui32BaseAddr = OSPCIAddrRangeStart(psPlatData->hRGXPCI, 0);

	if (OSPCIIRQ(psPlatData->hRGXPCI, &psDevice->ui32IRQ) != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "PCIInitDev: Couldn't get IRQ"));
		eError = PVRSRV_ERROR_INVALID_DEVICE;
		goto e4;
	}
	PVR_DPF((PVR_DBG_WARNING,
		 "PCIInitDev: BaseAddr 0x%08x, EndAddr 0x%08x, IRQ %d",
		 ui32BaseAddr, OSPCIAddrRangeEnd(psPlatData->hRGXPCI, 0),
		 psDevice->ui32IRQ));

	psDevice->sRegsCpuPBase.uiAddr = ui32BaseAddr + RGX_REG_OFFSET;
	psDevice->ui32RegsSize = RGX_REG_SIZE;
	PVR_DPF((PVR_DBG_WARNING, "PCIInitDev: sRegsCpuPBase 0x%x, size 0x%x",
		 psDevice->sRegsCpuPBase.uiAddr, psDevice->ui32RegsSize));

	return PVRSRV_OK;

 e4:
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI, 0);
	OSPCIReleaseDev(psPlatData->hRGXPCI);

	return eError;
}

/*!
******************************************************************************

 @Function		PCIDeInitDev

 @Description

 Uninitialise the PCI device when it is no loger required

 @Input		psSysData :	System data

 @Return	none

******************************************************************************/
static IMG_VOID PCIDeInitDev(PLAT_DATA * psPlatData)
{
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI, 0);
	OSPCIReleaseDev(psPlatData->hRGXPCI);
}

PVRSRV_ERROR SysCreateConfigData(PVRSRV_SYSTEM_CONFIG ** ppsSysConfig)
{
	PLAT_DATA *psPlatData;
	PVRSRV_ERROR eError;

	psPlatData = OSAllocMem(sizeof(PLAT_DATA));
	OSMemSet(psPlatData, 0, sizeof(PLAT_DATA));

	/* Query the Emu for reg and IRQ information */
	eError = PCIInitDev(psPlatData);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	/* Save data for this device */
	sSysConfig.pasDevices[0].hSysData = (IMG_HANDLE) psPlatData;

	/* Save private data for the physical memory heap */
	gsPhysHeapConfig.hPrivData = (IMG_HANDLE) psPlatData;

	*ppsSysConfig = &sSysConfig;

	gpsPlatData = psPlatData;
	return PVRSRV_OK;
 e0:
	return eError;
}

IMG_VOID SysDestroyConfigData(PVRSRV_SYSTEM_CONFIG * psSysConfig)
{
	PLAT_DATA *psPlatData = gpsPlatData;

	PVR_UNREFERENCED_PARAMETER(psSysConfig);
	PCIDeInitDev(psPlatData);
	OSFreeMem(psPlatData);
}

static IMG_VOID SysCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
				      IMG_DEV_PHYADDR * psDevPAddr,
				      IMG_CPU_PHYADDR * psCpuPAddr)
{
	PLAT_DATA *psPlatData = (PLAT_DATA *) hPrivData;

	psDevPAddr->uiAddr = psCpuPAddr->uiAddr;
}

static IMG_VOID SysDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
				      IMG_CPU_PHYADDR * psCpuPAddr,
				      IMG_DEV_PHYADDR * psDevPAddr)
{
	PLAT_DATA *psPlatData = (PLAT_DATA *) hPrivData;

	psCpuPAddr->uiAddr = psDevPAddr->uiAddr;
}

/******************************************************************************
 End of file (sysconfig.c)
******************************************************************************/
