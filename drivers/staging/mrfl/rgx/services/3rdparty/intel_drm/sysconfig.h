/*************************************************************************/ /*!
@File
@Title          System Description Header
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    This header provides system-specific declarations and macros
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include "pvrsrv_device.h"
#include "rgxdevice.h"

#if !defined(__SYSCCONFIG_H__)
#define __SYSCCONFIG_H__

static IMG_VOID SysCpuPAddrToDevPAddr(IMG_HANDLE hPrivData,
										IMG_DEV_PHYADDR *psDevPAddr,
										IMG_CPU_PHYADDR *psCpuPAddr);

static IMG_VOID SysDevPAddrToCpuPAddr(IMG_HANDLE hPrivData,
										IMG_CPU_PHYADDR *psCpuPAddr,
										IMG_DEV_PHYADDR *psDevPAddr);

static PVRSRV_ERROR SysDevicePostPowerState(
		PVRSRV_DEV_POWER_STATE eNewPowerState,
		PVRSRV_DEV_POWER_STATE eCurrentPowerState,
		IMG_BOOL bForced);

static PVRSRV_ERROR SysDevicePrePowerState(
		PVRSRV_DEV_POWER_STATE eNewPowerState,
		PVRSRV_DEV_POWER_STATE eCurrentPowerState,
		IMG_BOOL bForced);


static RGX_TIMING_INFORMATION sRGXTimingInfo =
{
	.ui32CoreClockSpeed        = 400000000, /* changed from 100000000, */
	.bEnableActivePM           = IMG_FALSE,
	.bEnableRDPowIsland	       = IMG_FALSE,
};

static RGX_DATA sRGXData =
{
	.psRGXTimingInfo = &sRGXTimingInfo,
};

static PVRSRV_DEVICE_CONFIG sDevices[] =
{
	/* RGX device */
	{
		.eDeviceType            = PVRSRV_DEVICE_TYPE_RGX,
		.pszName                = "RGX",

		/* Device setup information */
		.sRegsCpuPBase.uiAddr   = 0,
		.ui32RegsSize           = 0,
		.ui32IRQ                = 0,
		.bIRQIsShared           = IMG_TRUE,

		/* No power management on no HW system */
		.pfnPrePowerState       = SysDevicePrePowerState,
		.pfnPostPowerState      = SysDevicePostPowerState,

		.hDevData               = &sRGXData,
		.hSysData               = IMG_NULL,
	}
};

static PHYS_HEAP_FUNCTIONS gsPhysHeapFuncs = {
	.pfnCpuPAddrToDevPAddr	= SysCpuPAddrToDevPAddr,
	.pfnDevPAddrToCpuPAddr	= SysDevPAddrToCpuPAddr,
};

static PHYS_HEAP_CONFIG	gsPhysHeapConfig = {
	.ui32PhysHeapID			= 0,
	.eType					= PHYS_HEAP_TYPE_UMA,
	.pszPDumpMemspaceName	= "SYSMEM",
	.psMemFuncs				= &gsPhysHeapFuncs,
	.hPrivData				= IMG_NULL,
};


static PVRSRV_SYSTEM_CONFIG sSysConfig = {
	.pszSystemName = "Merrifield with Rogue",
	.uiDeviceCount = sizeof(sDevices)/sizeof(PVRSRV_DEVICE_CONFIG),
	.pasDevices = &sDevices[0],

	/* Physcial memory heaps */
	.ui32PhysHeapCount = sizeof(gsPhysHeapConfig) / sizeof(PHYS_HEAP_CONFIG),
	.pasPhysHeaps = &gsPhysHeapConfig,

	/* No power management on no HW system */
	.pfnSysPrePowerState = NULL,
	.pfnSysPostPowerState = NULL,

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
 
#endif	/* __SYSCCONFIG_H__ */
