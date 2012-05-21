/*!****************************************************************************
@File			sysconfig.h
@Title			System Description Header
@Author			Copyright (C) Imagination Technologies Limited.
				All rights reserved. Strictly Confidential.
@Description	Emulator system-specific declarations and macros
******************************************************************************/

#include "pvrsrv_device.h"
#include "rgxdevice.h"

#if !defined(__SYSCCONFIG_H__)
#define __SYSCCONFIG_H__

static IMG_VOID SysCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
				      IMG_DEV_PHYADDR * psDevPAddr,
				      IMG_CPU_PHYADDR * psCpuPAddr);

static IMG_VOID SysDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
				      IMG_CPU_PHYADDR * psCpuPAddr,
				      IMG_DEV_PHYADDR * psDevPAddr);

static RGX_TIMING_INFORMATION sRGXTimingInfo = {
	.ui32CoreClockSpeed = 100000000,
	.bEnableActivePM = IMG_FALSE,
	.ui32ActivePowManLatencyms = 0,
};

static RGX_DATA sRGXData = {
	.psRGXTimingInfo = &sRGXTimingInfo,
};

static PVRSRV_DEVICE_CONFIG sDevices[] = {
	/* RGX device */
	{
	 .eDeviceType = PVRSRV_DEVICE_TYPE_RGX,
	 .pszName = "RGX",

	 /* Device setup information */
	 .sRegsCpuPBase.uiAddr = 0,
	 .ui32RegsSize = 0,
	 .ui32IRQ = 0,
	 .bIRQIsShared = IMG_TRUE,

	 /* No power management on no HW system */
	 .pfnPrePowerState = IMG_NULL,
	 .pfnPostPowerState = IMG_NULL,

	 .hDevData = &sRGXData,
	 .hSysData = IMG_NULL,
	 }
};

static PHYS_HEAP_FUNCTIONS gsPhysHeapFuncs = {
	.pfnCpuPAddrToDevPAddr = SysCpuPAddrToDevPAddr,
	.pfnDevPAddrToCpuPAddr = SysDevPAddrToCpuPAddr,
};

static PHYS_HEAP_CONFIG gsPhysHeapConfig = {
	.ui32PhysHeapID = 0,
	.eType = PHYS_HEAP_TYPE_UMA,
	.pszPDumpMemspaceName = "SYSMEM",
	.psMemFuncs = &gsPhysHeapFuncs,
	.hPrivData = IMG_NULL,
};

static PVRSRV_SYSTEM_CONFIG sSysConfig = {
	.pszSystemName = "Merrifield VP with Rogue",
	.uiDeviceCount = sizeof(sDevices) / sizeof(PVRSRV_DEVICE_CONFIG),
	.pasDevices = &sDevices[0],

	/* Physcial memory heaps */
	.ui32PhysHeapCount =
	    sizeof(gsPhysHeapConfig) / sizeof(PHYS_HEAP_CONFIG),
	.pasPhysHeaps = &gsPhysHeapConfig,

	/* No power management on no HW system */
	.pfnSysPrePowerState = IMG_NULL,
	.pfnSysPostPowerState = IMG_NULL,

	/* no cache snooping */
	.bHasCacheSnooping = IMG_FALSE,
};

#define VENDOR_ID_MERRIFIELD        0x8086
#define DEVICE_ID_MERRIFIELD        0x1180

#define RGX_REG_OFFSET              0x100000
#define RGX_REG_SIZE                0x10000

#define IS_MRFLD(dev) (((dev)->pci_device & 0xFFF8) == DEVICE_ID_MERRIFIELD)

/*****************************************************************************
 * system specific data structures
 *****************************************************************************/

#endif				/* __SYSCCONFIG_H__ */
