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

#define EMU_RGX_CLOCK_FREQ		(400000000)
#define RESERVE_DC_MEM_SIZE		(16 * 1024 * 1024)

static IMG_UINT32 EmuRGXClockFreq(IMG_HANDLE hSysData);

static IMG_VOID EmuCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
				      IMG_DEV_PHYADDR * psDevPAddr,
				      IMG_CPU_PHYADDR * psCpuPAddr);

static IMG_VOID EmuDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
				      IMG_CPU_PHYADDR * psCpuPAddr,
				      IMG_DEV_PHYADDR * psDevPAddr);

static RGX_TIMING_INFORMATION sRGXTimingInfo = {
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
	 .ui32PhysHeapID = 0,	/* Use physcial heap 0 */

	 /* No power management on no HW system */
	 .pfnPrePowerState = IMG_NULL,
	 .pfnPostPowerState = IMG_NULL,

	 .pfnClockFreqGet = EmuRGXClockFreq,

	 .hDevData = &sRGXData,
	 .hSysData = IMG_NULL,
	 }
};

static PHYS_HEAP_FUNCTIONS gsPhysHeapFuncs = {
	.pfnCpuPAddrToDevPAddr = EmuCpuPAddrToDevPAddr,
	.pfnDevPAddrToCpuPAddr = EmuDevPAddrToCpuPAddr,
};

#if defined(LMA)
static PHYS_HEAP_CONFIG gsPhysHeapConfig[] = {
	{			/* Heap 0 is used for RGX */
	 .ui32PhysHeapID = 0,
	 .eType = PHYS_HEAP_TYPE_LMA,
	 .pszPDumpMemspaceName = "LMA",
	 .psMemFuncs = &gsPhysHeapFuncs,
	 .hPrivData = IMG_NULL,
	 },
	{			/* Heap 1 is used for DC memory */
	 .ui32PhysHeapID = 1,
	 .eType = PHYS_HEAP_TYPE_LMA,
	 .pszPDumpMemspaceName = "LMA",
	 .psMemFuncs = &gsPhysHeapFuncs,
	 .hPrivData = IMG_NULL,
	 }
};
#else
static PHYS_HEAP_CONFIG gsPhysHeapConfig = {
	.ui32PhysHeapID = 0,
	.eType = PHYS_HEAP_TYPE_UMA,
	.pszPDumpMemspaceName = "SYSMEM",
	.psMemFuncs = &gsPhysHeapFuncs,
	.hPrivData = IMG_NULL,
};
#endif
static PVRSRV_SYSTEM_CONFIG sSysConfig = {
	.pszSystemName = "Emu with Rogue",
	.uiDeviceCount = sizeof(sDevices) / sizeof(PVRSRV_DEVICE_CONFIG),
	.pasDevices = &sDevices[0],

	/* Physcial memory heaps */
	.ui32PhysHeapCount =
	    sizeof(gsPhysHeapConfig) / sizeof(PHYS_HEAP_CONFIG),
#if defined(LMA)
	.pasPhysHeaps = &gsPhysHeapConfig[0],
#else
	.pasPhysHeaps = &gsPhysHeapConfig,
#endif
	/* No power management on no HW system */
	.pfnSysPrePowerState = IMG_NULL,
	.pfnSysPostPowerState = IMG_NULL,

	/* no cache snooping */
	.bHasCacheSnooping = IMG_FALSE,
};

/*****************************************************************************
 * system specific data structures
 *****************************************************************************/

#endif				/* __SYSCCONFIG_H__ */
