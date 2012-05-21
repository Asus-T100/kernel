									    /*************************************************************************//*!
									       @File                    pvrsrv_device.h

									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved

									       @License        Strictly Confidential.
    *//**************************************************************************/

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

	typedef IMG_VOID(*PFN_MISR) (IMG_VOID * pvData);

	typedef IMG_BOOL(*PFN_LISR) (IMG_VOID * pvData);

	typedef IMG_UINT32(*PFN_SYS_DEV_CLK_FREQ_GET) (IMG_HANDLE hSysData);

	typedef PVRSRV_ERROR(*PFN_SYS_DEV_PRE_POWER) (PVRSRV_DEV_POWER_STATE
						      eNewPowerState,
						      PVRSRV_DEV_POWER_STATE
						      eCurrentPowerState);

	typedef PVRSRV_ERROR(*PFN_SYS_DEV_POST_POWER) (PVRSRV_DEV_POWER_STATE
						       eNewPowerState,
						       PVRSRV_DEV_POWER_STATE
						       eCurrentPowerState);

	typedef struct _PVRSRV_DEVICE_CONFIG_ {
		/*! Configuration flags */
		IMG_UINT32 uiFlags;

		/*! Name of the device (used when registering the IRQ) */
		IMG_CHAR *pszName;

		/*! Type of device this is */
		PVRSRV_DEVICE_TYPE eDeviceType;

		/*! Register bank address */
		IMG_CPU_PHYADDR sRegsCpuPBase;
		/*! Register bank size */
		IMG_UINT32 ui32RegsSize;
		/*! Device interrupt number */
		IMG_UINT32 ui32IRQ;

		/*! The device interrupt is shared */
		IMG_BOOL bIRQIsShared;
		/*! Device specific data handle */
		IMG_HANDLE hDevData;

		/*! System specific data. This gets passed into system callback functions */
		IMG_HANDLE hSysData;

		/*! ID of the Physcial memory heap to use */
		IMG_UINT32 ui32PhysHeapID;

		/*! Hostport aperture base CPU physical address */
		IMG_CPU_PHYADDR sHPApertureBasePAddr;
		/*! Hostport aperture size */
		IMG_UINT32 ui32HPApertureSizeBytes;

		/*! Callback to inform the device we about to change power state */
		PFN_SYS_DEV_PRE_POWER pfnPrePowerState;

		/*! Callback to inform the device we have finished the power state change */
		PFN_SYS_DEV_POST_POWER pfnPostPowerState;

		/*! Callback to obtain the clock frequency from the device */
		PFN_SYS_DEV_CLK_FREQ_GET pfnClockFreqGet;

		/*! Current breakpoint data master */
		RGXFWIF_DM eBPDM;
		/*! A Breakpoint has been set */
		IMG_BOOL bBPSet;
	} PVRSRV_DEVICE_CONFIG;

	typedef
	    PVRSRV_ERROR(*PFN_SYSTEM_PRE_POWER_STATE) (PVRSRV_SYS_POWER_STATE
						       eNewPowerState);
	typedef
	    PVRSRV_ERROR(*PFN_SYSTEM_POST_POWER_STATE) (PVRSRV_SYS_POWER_STATE
							eNewPowerState);

	typedef struct _PVRSRV_SYSTEM_CONFIG_ {
		IMG_UINT32 uiSysFlags;
		IMG_CHAR *pszSystemName;
		IMG_UINT32 uiDeviceCount;
		PVRSRV_DEVICE_CONFIG *pasDevices;
		PFN_SYSTEM_PRE_POWER_STATE pfnSysPrePowerState;
		PFN_SYSTEM_POST_POWER_STATE pfnSysPostPowerState;
		IMG_BOOL bHasCacheSnooping;

		PHYS_HEAP_CONFIG *pasPhysHeaps;
		IMG_UINT32 ui32PhysHeapCount;
	} PVRSRV_SYSTEM_CONFIG;

#if defined(__cplusplus)
}
#endif
#endif				/* __PVRSRV_DEVICE_H__ */
