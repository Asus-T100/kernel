/*!****************************************************************************
@File           sysconfig.c

@Title          System Configuration

@Author         Imagination Technologies

@Date           29/09/2003

@Copyright      Copyright 2003-2008 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either material
                or conceptual may be copied or distributed, transmitted,
                transcribed, stored in a retrieval system or translated into
                any human or computer language in any form by any means,
                electronic, mechanical, manual or otherwise, or disclosed
                to third parties without the express written permission of
                Imagination Technologies Limited, Home Park Estate,
                Kings Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform       Generic

@Description    System Configuration functions

@DoxygenVer

******************************************************************************/

#include "pvrsrv_device.h"
#include "syscommon.h"
#include "sysconfig.h"
#include "physheap.h"

static RGX_TIMING_INFORMATION gsRGXTimingInfo;
static RGX_DATA gsRGXData;
static PVRSRV_DEVICE_CONFIG gsDevices[1];
static PVRSRV_SYSTEM_CONFIG gsSysConfig;

static PHYS_HEAP_FUNCTIONS gsPhysHeapFuncs;
static PHYS_HEAP_CONFIG gsPhysHeapConfig;

/*
	CPU to Device physcial address translation
*/
static
IMG_VOID UMAPhysHeapCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
				       IMG_DEV_PHYADDR * psDevPAddr,
				       IMG_CPU_PHYADDR * psCpuPAddr)
{
	psDevPAddr->uiAddr = psCpuPAddr->uiAddr;
}

/*
	Device to CPU physcial address translation
*/
static
IMG_VOID UMAPhysHeapDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
				       IMG_CPU_PHYADDR * psCpuPAddr,
				       IMG_DEV_PHYADDR * psDevPAddr)
{
	psCpuPAddr->uiAddr = psDevPAddr->uiAddr;
}

/*
	SysCreateConfigData
*/
PVRSRV_ERROR SysCreateConfigData(PVRSRV_SYSTEM_CONFIG ** ppsSysConfig)
{
	/*
	 * Setup information about physcial memory heap(s) we have
	 */
	gsPhysHeapFuncs.pfnCpuPAddrToDevPAddr = UMAPhysHeapCpuPAddrToDevPAddr;
	gsPhysHeapFuncs.pfnDevPAddrToCpuPAddr = UMAPhysHeapDevPAddrToCpuPAddr;

	gsPhysHeapConfig.ui32PhysHeapID = 0;
	gsPhysHeapConfig.pszPDumpMemspaceName = "SYSMEM";
	gsPhysHeapConfig.eType = PHYS_HEAP_TYPE_UMA;
	gsPhysHeapConfig.psMemFuncs = &gsPhysHeapFuncs;
	gsPhysHeapConfig.hPrivData = IMG_NULL;

	gsSysConfig.pasPhysHeaps = &gsPhysHeapConfig;
	gsSysConfig.ui32PhysHeapCount =
	    sizeof(gsPhysHeapConfig) / sizeof(PHYS_HEAP_CONFIG);

	/*
	 * Setup RGX specific timing data
	 */
	gsRGXTimingInfo.ui32CoreClockSpeed = RGX_NOHW_CORE_CLOCK_SPEED;
	gsRGXTimingInfo.bEnableActivePM = IMG_FALSE;
	gsRGXTimingInfo.ui32ActivePowManLatencyms = 0;

	/*
	 *Setup RGX specific data
	 */
	gsRGXData.psRGXTimingInfo = &gsRGXTimingInfo;

	/*
	 * Setup RGX device
	 */
	gsDevices[0].eDeviceType = PVRSRV_DEVICE_TYPE_RGX;
	gsDevices[0].pszName = "RGX";

	/* Device setup information */
	gsDevices[0].sRegsCpuPBase.uiAddr = 0x00f00baa;
	gsDevices[0].ui32RegsSize = 0x4000;
	gsDevices[0].ui32IRQ = 0x00000bad;
	gsDevices[0].bIRQIsShared = IMG_FALSE;

	/* No power management on no HW system */
	gsDevices[0].pfnPrePowerState = IMG_NULL;
	gsDevices[0].pfnPostPowerState = IMG_NULL;

	/* No clock frequency either */
	gsDevices[0].pfnClockFreqGet = IMG_NULL;

	gsDevices[0].hDevData = &gsRGXData;

	/* Hostport */
	gsDevices[0].sHPApertureBasePAddr.uiAddr = 0;
	gsDevices[0].ui32HPApertureSizeBytes = 64 * 1024 * 1024;

	/*
	 * Setup system config
	 */
	gsSysConfig.pszSystemName = RGX_NOHW_SYSTEM_NAME;
	gsSysConfig.uiDeviceCount = sizeof(gsDevices) / sizeof(gsDevices[0]);
	gsSysConfig.pasDevices = &gsDevices[0];

	/* No power management on no HW system */
	gsSysConfig.pfnSysPrePowerState = IMG_NULL;
	gsSysConfig.pfnSysPostPowerState = IMG_NULL;

	/* no cache snooping */
	gsSysConfig.bHasCacheSnooping = IMG_FALSE;

	*ppsSysConfig = &gsSysConfig;
	return PVRSRV_OK;
}

/*
	SysDestroyConfigData
*/
IMG_VOID SysDestroyConfigData(PVRSRV_SYSTEM_CONFIG * psSysConfig)
{
	PVR_UNREFERENCED_PARAMETER(psSysConfig);
}

/******************************************************************************
 End of file (sysconfig.c)
******************************************************************************/
