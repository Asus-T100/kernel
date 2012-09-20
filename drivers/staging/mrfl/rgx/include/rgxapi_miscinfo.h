									    /*************************************************************************//*!
									       @File
									       @Title          RGX Misc Info API
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXAPI_MISCINFO_H__)
#define __RGXAPI_MISCINFO_H__

/*!
	List of possible requests/commands to RGXGetMiscInfo()
*/
/* FIXME: Shouldn't be in here. */
typedef enum _RGX_MISC_INFO_REQUEST_ {
	RGX_MISC_INFO_REQUEST_CLOCKSPEED = 0,	/*!< Clock speed Misc Info Request */
	RGX_MISC_INFO_REQUEST_RGXREV,	/*!< RGX Revision Misc Info Request */
	RGX_MISC_INFO_REQUEST_DRIVER_RGXREV,	/*!< Driver RGX Revision Misc Info Request */
	RGX_MISC_INFO_DUMP_DEBUG_INFO,	/*!< Dump Debug Info Misc Info Request */
	RGX_MISC_INFO_PANIC,	/*!< Panic Misc Info Request */
	RGX_MISC_INFO_REQUEST_SPM,	/*!< SPM Misc Info Request */
	RGX_MISC_INFO_REQUEST_ACTIVEPOWER,	/*!< Active Power Misc Info Request */
	RGX_MISC_INFO_REQUEST_FORCE_I16 = 0x7fff	/*!< Force enum to 16 bits */
} RGX_MISC_INFO_REQUEST;

									    /*************************************************************************//*!
									     * Struct for passing RGX core rev/features from ukernel to driver.
									     * This is accessed from the kernel part of the driver and microkernel; it is
									     * only accessed in user space during buffer allocation in srvinit.
     *//**************************************************************************/
typedef struct _PVRSRV_RGX_MISCINFO_FEATURES {
	IMG_UINT32 ui32CoreRev;	/*!< RGX Core revision from HW register */
	IMG_UINT32 ui32CoreID;	/*!< RGX Core ID from HW register */
	IMG_UINT32 ui32DDKVersion;	/*!< software DDK version */
	IMG_UINT32 ui32DDKBuild;	/*!< software DDK build no. */
	IMG_UINT32 ui32CoreIdSW;	/*!< software core version (ID) */
	IMG_UINT32 ui32CoreRevSW;	/*!< software core revision */
	IMG_UINT32 ui32BuildOptions;	/*!< build options bit-field */
} PVRSRV_RGX_MISCINFO_FEATURES;

									    /*************************************************************************//*!
									     * Struct for getting lock-up stats from the kernel driver
    *//***************************************************************************/
typedef struct _PVRSRV_RGX_MISCINFO_LOCKUPS {
	IMG_UINT32 ui32HostDetectedLockups;	/*!< Host timer detected lockups */
	IMG_UINT32 ui32uKernelDetectedLockups;	/*!< Microkernel detected lockups */
} PVRSRV_RGX_MISCINFO_LOCKUPS;

									    /*************************************************************************//*!
									     * Struct for getting lock-up stats from the kernel driver
     *//**************************************************************************/
typedef struct _PVRSRV_RGX_MISCINFO_ACTIVEPOWER {
	IMG_UINT32 ui32NumActivePowerEvents;	/*!< active power events */
} PVRSRV_RGX_MISCINFO_ACTIVEPOWER;

									    /*************************************************************************//*!
									     * Struct for getting SPM stats fro the kernel driver
     *//**************************************************************************/
typedef struct _PVRSRV_RGX_MISCINFO_SPM {
	IMG_HANDLE hRTDataSet;	/*!< render target data set handle returned from RGXAddRenderTarget */
	IMG_UINT32 ui32NumOutOfMemSignals;	/*!< Number of Out of Mem Signals */
	IMG_UINT32 ui32NumSPMRenders;	/*!< Number of SPM renders */
} PVRSRV_RGX_MISCINFO_SPM;

/*!
 ******************************************************************************
 * Structure for misc RGX commands in services
 *****************************************************************************/
typedef struct _RGX_MISC_INFO_ {
	RGX_MISC_INFO_REQUEST eRequest;	/*!< Command request to RGXGetMiscInfo() */
	union {
		IMG_UINT32 reserved;	/*!< Unused: ensures valid code in the case everything else is compiled out */
		PVRSRV_RGX_MISCINFO_FEATURES sRGXFeatures;	/*!< RGX Features Misc Info */
		IMG_UINT32 ui32RGXClockSpeed;	/*!< RGX Clock Speed */
		PVRSRV_RGX_MISCINFO_ACTIVEPOWER sActivePower;	/*!< Active Power Misc Info */
		PVRSRV_RGX_MISCINFO_LOCKUPS sLockups;	/*!< Lockups Misc Info */
		PVRSRV_RGX_MISCINFO_SPM sSPM;	/*!< SPM Misc Info */
	} uData;
} RGX_MISC_INFO;

#endif				/* __RGXAPI_MISCINFO_H__ */
