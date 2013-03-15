/**************************************************************************/ /*!
@File
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
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
*/ /***************************************************************************/

#ifndef __PVRSRV_DEVICE_H__
#define __PVRSRV_DEVICE_H__

#include "servicesext.h"
#include "pvrsrv_device_types.h"
#include "img_types.h"
#include "ra.h"
#include "physheap.h"
#include "rgx_fwif_km.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef struct _PVRSRV_DEVICE_CONFIG_ PVRSRV_DEVICE_CONFIG;


typedef IMG_VOID (*PFN_MISR)(IMG_VOID *pvData);

typedef IMG_BOOL (*PFN_LISR)(IMG_VOID *pvData);

typedef IMG_UINT32 (*PFN_SYS_DEV_CLK_FREQ_GET)(IMG_HANDLE hSysData);

typedef PVRSRV_ERROR (*PFN_SYS_DEV_PRE_POWER)(PVRSRV_DEV_POWER_STATE eNewPowerState,
                                              PVRSRV_DEV_POWER_STATE eCurrentPowerState,
											  IMG_BOOL bForced);


typedef PVRSRV_ERROR (*PFN_SYS_DEV_POST_POWER)(PVRSRV_DEV_POWER_STATE eNewPowerState,
                                               PVRSRV_DEV_POWER_STATE eCurrentPowerState,
											   IMG_BOOL bForced);

typedef IMG_VOID (*PFN_SYS_DEV_INTERRUPT_HANDLED)(PVRSRV_DEVICE_CONFIG *psDevConfig);

struct _PVRSRV_DEVICE_CONFIG_
{
	/*! Configuration flags */
	IMG_UINT32			uiFlags;

	/*! Name of the device (used when registering the IRQ) */
	IMG_CHAR			*pszName;

	/*! Type of device this is */
	PVRSRV_DEVICE_TYPE		eDeviceType;

	/*! Register bank address */
	IMG_CPU_PHYADDR			sRegsCpuPBase;
	/*! Register bank size */
	IMG_UINT32			ui32RegsSize;
	/*! Device interrupt number */
	IMG_UINT32			ui32IRQ;

	/*! The device interrupt is shared */
	IMG_BOOL			bIRQIsShared;
	/*! Device specific data handle */
	IMG_HANDLE			hDevData;

	/*! System specific data. This gets passed into system callback functions */
	IMG_HANDLE			hSysData;

	/*! ID of the Physcial memory heap to use */
	IMG_UINT32			ui32PhysHeapID;

	/*! Callback to inform the device we about to change power state */
	PFN_SYS_DEV_PRE_POWER		pfnPrePowerState;

	/*! Callback to inform the device we have finished the power state change */
	PFN_SYS_DEV_POST_POWER		pfnPostPowerState;

	/*! Callback to obtain the clock frequency from the device */
	PFN_SYS_DEV_CLK_FREQ_GET	pfnClockFreqGet;

	/*! Callback to inform the device that an interrupt has been handled */
	PFN_SYS_DEV_INTERRUPT_HANDLED	pfnInterruptHandled;

	/*! Current breakpoint data master */
	RGXFWIF_DM			eBPDM;
	/*! A Breakpoint has been set */
	IMG_BOOL			bBPSet;	
};

typedef PVRSRV_ERROR (*PFN_SYSTEM_PRE_POWER_STATE)(PVRSRV_SYS_POWER_STATE eNewPowerState);
typedef PVRSRV_ERROR (*PFN_SYSTEM_POST_POWER_STATE)(PVRSRV_SYS_POWER_STATE eNewPowerState);

typedef struct _PVRSRV_SYSTEM_CONFIG_
{
	IMG_UINT32				uiSysFlags;
	IMG_CHAR				*pszSystemName;
	IMG_UINT32				uiDeviceCount;
	PVRSRV_DEVICE_CONFIG	*pasDevices;
	PFN_SYSTEM_PRE_POWER_STATE pfnSysPrePowerState;
	PFN_SYSTEM_POST_POWER_STATE pfnSysPostPowerState;
	IMG_BOOL				bHasCacheSnooping;

	PHYS_HEAP_CONFIG		*pasPhysHeaps;
	IMG_UINT32				ui32PhysHeapCount;
} PVRSRV_SYSTEM_CONFIG;

#if defined(__cplusplus)
}
#endif

#endif /* __PVRSRV_DEVICE_H__*/
