									     /**************************************************************************//*!
									        @File           sysconfig.c
									        @Title          Sysconfig layer for Emu
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Implements the system layer for the emulator
    *//***************************************************************************/

#include "img_types.h"
#include "pvrsrv_device.h"
#include "sysinfo.h"
#include "syscommon.h"
#include "sysconfig.h"
#include "emu.h"
#include "allocmem.h"
#include "pvr_debug.h"
#include "osfunc.h"

#include "pci_support.h"

#include "emu_cr_defs.h"

typedef struct _PLAT_DATA_ {
	IMG_HANDLE hRGXPCI;
	IMG_UINT32 uiAtlasRegPAddr;
	IMG_UINT32 uiAtlasRegSize;
#if defined (LMA)
	IMG_UINT64 uiLMACpuPAddr;
	IMG_UINT64 uiLMASize;
#endif
} PLAT_DATA;

PLAT_DATA *gpsPlatData = IMG_NULL;

/*
	EmuReset
*/
static PVRSRV_ERROR EmuReset(IMG_CPU_PHYADDR sRegsCpuPBase)
{
	IMG_CPU_PHYADDR sWrapperRegsCpuPBase;
	IMG_VOID *pvWrapperRegs;

	sWrapperRegsCpuPBase.uiAddr =
	    sRegsCpuPBase.uiAddr + EMULATOR_RGX_REG_WRAPPER_OFFSET;

	/*
	   Create a temporary mapping of the wrapper registers in order to reset
	   the emulator design.
	 */
	pvWrapperRegs = OSMapPhysToLin(sWrapperRegsCpuPBase,
				       EMULATOR_RGX_REG_WRAPPER_SIZE, 0);
	if (pvWrapperRegs == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR,
			 "EmuReset: Failed to create wrapper register mapping\n"));
		return PVRSRV_ERROR_BAD_MAPPING;
	}

	/*
	   Emu reset.
	 */
	OSWriteHWReg32(pvWrapperRegs, EMU_CR_SOFT_RESET,
		       EMU_CR_SOFT_RESET_SYS_EN | EMU_CR_SOFT_RESET_MEM_EN |
		       EMU_CR_SOFT_RESET_CORE_EN);
	/* Flush register write */
	(void)OSReadHWReg32(pvWrapperRegs, EMU_CR_SOFT_RESET);
	OSWaitus(10);

	OSWriteHWReg32(pvWrapperRegs, EMU_CR_SOFT_RESET, 0x0);
	/* Flush register write */
	(void)OSReadHWReg32(pvWrapperRegs, EMU_CR_SOFT_RESET);
	OSWaitus(10);

#if !defined(LMA)
	/* If we're UMA then enable bus mastering */
	OSWriteHWReg32(pvWrapperRegs, EMU_CR_PCI_MASTER,
		       EMU_CR_PCI_MASTER_MODE_EN);

	/* Flush register write */
	(void)OSReadHWReg32(pvWrapperRegs, EMU_CR_PCI_MASTER);
#endif

	/*
	   Remove the temporary register mapping.
	 */
	OSUnMapPhysToLin(pvWrapperRegs, EMULATOR_RGX_REG_WRAPPER_SIZE, 0);

	return PVRSRV_OK;
}

/*
	PCIInitDev
*/
static PVRSRV_ERROR PCIInitDev(PLAT_DATA * psPlatData)
{
	PVRSRV_DEVICE_CONFIG *psDevice = &sSysConfig.pasDevices[0];
	PVRSRV_ERROR eError;
#if defined (LMA)
	IMG_UINT32 uiLMACpuPAddr;
	IMG_UINT32 uiLMASize;
#endif

	psPlatData->hRGXPCI = OSPCIAcquireDev(SYS_RGX_DEV_VENDOR_ID,
					      SYS_RGX_DEV_DEVICE_ID,
					      HOST_PCI_INIT_FLAG_BUS_MASTER |
					      HOST_PCI_INIT_FLAG_MSI);

	if (!psPlatData->hRGXPCI) {
		psPlatData->hRGXPCI = OSPCIAcquireDev(SYS_RGX_DEV_VENDOR_ID,
						      SYS_RGX_DEV1_DEVICE_ID,
						      HOST_PCI_INIT_FLAG_BUS_MASTER
						      | HOST_PCI_INIT_FLAG_MSI);
	}

	if (!psPlatData->hRGXPCI) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Failed to acquire PCI device"));
		eError = PVRSRV_ERROR_PCI_DEVICE_NOT_FOUND;
		goto e0;
	}

	psPlatData->uiAtlasRegPAddr =
	    OSPCIAddrRangeStart(psPlatData->hRGXPCI,
				EMULATOR_ATLAS_REG_PCI_BASENUM);
	psPlatData->uiAtlasRegSize =
	    OSPCIAddrRangeLen(psPlatData->hRGXPCI,
			      EMULATOR_ATLAS_REG_PCI_BASENUM);

	/* Check the address range is large enough. */
	if (psPlatData->uiAtlasRegSize < EMULATOR_ATLAS_REG_SIZE) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region isn't big enough (Emu reg) (was 0x%08x, required 0x%08x)",
			 psPlatData->uiAtlasRegSize, EMULATOR_ATLAS_REG_SIZE));
		eError = PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
		goto e1;
	}

	/* Reserve the address range */
	if (OSPCIRequestAddrRange
	    (psPlatData->hRGXPCI, EMULATOR_ATLAS_REG_PCI_BASENUM)
	    != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region not available (Emu reg)"));
		eError = PVRSRV_ERROR_PCI_REGION_UNAVAILABLE;
		goto e1;
	}

	psDevice->sRegsCpuPBase.uiAddr =
	    OSPCIAddrRangeStart(psPlatData->hRGXPCI,
				EMULATOR_RGX_REG_PCI_BASENUM);
	psDevice->ui32RegsSize =
	    OSPCIAddrRangeLen(psPlatData->hRGXPCI,
			      EMULATOR_RGX_REG_PCI_BASENUM);

	/* Check the address range is large enough. */
	if (psDevice->ui32RegsSize < EMULATOR_RGX_REG_SIZE) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region isn't big enough (RGX reg) (was 0x%08x, required 0x%08x)",
			 psDevice->ui32RegsSize, EMULATOR_RGX_REG_SIZE));
		eError = PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
		goto e2;
	}

	/* Reserve the address range */
	if (OSPCIRequestAddrRange
	    (psPlatData->hRGXPCI, EMULATOR_RGX_REG_PCI_BASENUM) != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region not available (RGX reg) "));
		eError = PVRSRV_ERROR_PCI_REGION_UNAVAILABLE;
		goto e2;
	}
#if defined (LMA)
	uiLMACpuPAddr =
	    OSPCIAddrRangeStart(psPlatData->hRGXPCI,
				EMULATOR_RGX_MEM_PCI_BASENUM);
	uiLMASize =
	    OSPCIAddrRangeLen(psPlatData->hRGXPCI,
			      EMULATOR_RGX_MEM_PCI_BASENUM);

	/* Setup the RGX heap */
	gsPhysHeapConfig[0].sStartAddr.uiAddr = uiLMACpuPAddr;
	gsPhysHeapConfig[0].uiSize = uiLMASize - RESERVE_DC_MEM_SIZE;

	/* Setup the DC heap */
	gsPhysHeapConfig[1].sStartAddr.uiAddr =
	    uiLMACpuPAddr + gsPhysHeapConfig[0].uiSize;
	gsPhysHeapConfig[1].uiSize = RESERVE_DC_MEM_SIZE;

	/* Check the address range is large enough. */
	if (uiLMASize < EMULATOR_RGX_MEM_REGION_SIZE) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region isn't big enough (was 0x%08x, required 0x%08x)",
			 uiLMASize, EMULATOR_RGX_MEM_REGION_SIZE));
		eError = PVRSRV_ERROR_PCI_REGION_TOO_SMALL;
		goto e3;
	}

	/* Reserve the address range */
	if (OSPCIRequestAddrRange
	    (psPlatData->hRGXPCI, EMULATOR_RGX_MEM_PCI_BASENUM) != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PCIInitDev: Device memory region not available"));
		eError = PVRSRV_ERROR_PCI_REGION_UNAVAILABLE;
		goto e3;
	}
#endif

	if (OSPCIIRQ(psPlatData->hRGXPCI, &psDevice->ui32IRQ) != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "PCIInitDev: Couldn't get IRQ"));
		eError = PVRSRV_ERROR_INVALID_DEVICE;
		goto e4;
	}
#if defined (LMA)
	/* Save this info for later use */
	psPlatData->uiLMACpuPAddr = uiLMACpuPAddr;
	psPlatData->uiLMASize = uiLMASize;
#endif

	/*
	   Reset the emulator design
	 */
	eError = EmuReset(psDevice->sRegsCpuPBase);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "PCIInitDev: Couldn't reset emulator"));
		goto e4;
	}

	return PVRSRV_OK;

 e4:
#if defined (LMA)
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI,
			      EMULATOR_RGX_MEM_PCI_BASENUM);
 e3:
#endif
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI,
			      EMULATOR_RGX_REG_PCI_BASENUM);
 e2:
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI,
			      EMULATOR_ATLAS_REG_PCI_BASENUM);
 e1:
	OSPCIReleaseDev(psPlatData->hRGXPCI);
 e0:
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
#if defined (LMA)
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI,
			      EMULATOR_RGX_MEM_PCI_BASENUM);
#endif
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI,
			      EMULATOR_RGX_REG_PCI_BASENUM);
	OSPCIReleaseAddrRange(psPlatData->hRGXPCI,
			      EMULATOR_ATLAS_REG_PCI_BASENUM);
	OSPCIReleaseDev(psPlatData->hRGXPCI);
}

static IMG_VOID EmuCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
				      IMG_DEV_PHYADDR * psDevPAddr,
				      IMG_CPU_PHYADDR * psCpuPAddr)
{
	PLAT_DATA *psPlatData = (PLAT_DATA *) hPrivData;

#if defined(LMA)
	psDevPAddr->uiAddr = psCpuPAddr->uiAddr - psPlatData->uiLMACpuPAddr;
#else
	PVR_UNREFERENCED_PARAMETER(psPlatData);
	psDevPAddr->uiAddr = psCpuPAddr->uiAddr;
#endif
}

static IMG_VOID EmuDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
				      IMG_CPU_PHYADDR * psCpuPAddr,
				      IMG_DEV_PHYADDR * psDevPAddr)
{
	PLAT_DATA *psPlatData = (PLAT_DATA *) hPrivData;

#if defined(LMA)
	psCpuPAddr->uiAddr = psDevPAddr->uiAddr + psPlatData->uiLMACpuPAddr;
#else
	PVR_UNREFERENCED_PARAMETER(psPlatData);
	psCpuPAddr->uiAddr = psDevPAddr->uiAddr;
#endif
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

#if defined (LMA)
	/* Save private data for the physical memory heaps */
	gsPhysHeapConfig[0].hPrivData = (IMG_HANDLE) psPlatData;
	gsPhysHeapConfig[1].hPrivData = (IMG_HANDLE) psPlatData;
#endif
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

static IMG_UINT32 EmuRGXClockFreq(IMG_HANDLE hSysData)
{
	return EMU_RGX_CLOCK_FREQ;
}

/******************************************************************************
 End of file (sysconfig.c)
******************************************************************************/
