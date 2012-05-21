									    /*************************************************************************//*!
									       @File
									       @Title          RGX device node header file
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the RGX device node
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXDEVICE_H__)
#define __RGXDEVICE_H__

#include "img_types.h"
#include "pvrsrv_device_types.h"
#include "mmu_common.h"
#include "rgx_fwif_km.h"
#include "rgx_fwif.h"
#include "rgxscript.h"
#include "cache_external.h"
#include "device.h"

typedef struct _PVRSRV_STUB_PBDESC_ PVRSRV_STUB_PBDESC;

typedef struct _PVRSRV_RGXDEV_INFO_ {
	PVRSRV_DEVICE_TYPE eDeviceType;
	PVRSRV_DEVICE_CLASS eDeviceClass;
	PVRSRV_DEVICE_NODE *psDeviceNode;

	IMG_UINT8 ui8VersionMajor;
	IMG_UINT8 ui8VersionMinor;
	IMG_UINT32 ui32CoreConfig;
	IMG_UINT32 ui32CoreFlags;

	/* Kernel mode linear address of device registers */
	IMG_PVOID pvRegsBaseKM;

	/* FIXME: The alloc for this should go through OSAllocMem in future */
	IMG_HANDLE hRegMapping;

	/* System physical address of device registers */
	IMG_CPU_PHYADDR sRegsPhysBase;
	/*  Register region size in bytes */
	IMG_UINT32 ui32RegSize;

	/*  RGX clock speed */
	IMG_UINT32 ui32CoreClockSpeed;

	PVRSRV_STUB_PBDESC *psStubPBDescListKM;

	/* Firmware memory context info */
	DEVMEM_CONTEXT *psKernelDevmemCtx;
	DEVMEM_HEAP *psFirmwareHeap;
	MMU_CONTEXT *psKernelMMUCtx;
	IMG_UINT32 ui32KernelCatBase;

	IMG_VOID *pvDeviceMemoryHeap;

	DEVMEM_MEMDESC *apsKernelCCBCtlMemDesc[RGXFWIF_DM_MAX];	/*!< memdesc for kernel CCB control */
	RGXFWIF_KCCB_CTL *apsKernelCCBCtl[RGXFWIF_DM_MAX];	/*!< kernel CCB control kernel mapping */
	DEVMEM_MEMDESC *apsKernelCCBMemDesc[RGXFWIF_DM_MAX];	/*!< memdesc for kernel CCB */
	IMG_UINT8 *apsKernelCCB[RGXFWIF_DM_MAX];	/*!< kernel CCB kernel mapping */

	PVRSRV_CACHE_OP uiCacheOp;

	/* client-side build options */
	IMG_UINT32 ui32ClientBuildOptions;

	/*
	   if we don't preallocate the pagetables we must 
	   insert newly allocated page tables dynamically 
	 */
	IMG_VOID *pvMMUContextList;

	IMG_UINT32 ui32ClkGateStatusReg;
	IMG_UINT32 ui32ClkGateStatusMask;
	RGX_SCRIPTS sScripts;

	DEVMEM_MEMDESC *psRGXFWMemDesc;
	DEVMEM_EXPORTCOOKIE sRGXFWExportCookie;

	DEVMEM_MEMDESC *psRGXFWIfTraceBufCtlMemDesc;
	RGXFWIF_TRACEBUF *psRGXFWIfTraceBuf;

	DEVMEM_MEMDESC *psRGXFWIfInitMemDesc;

#if defined(RGXFW_ALIGNCHECKS)
	DEVMEM_MEMDESC *psRGXFWAlignChecksMemDesc;
#endif

#if defined(SUPPORT_RGXFW_UNITTESTS)
	DEVMEM_MEMDESC *psRGXFWUnittestsMemDesc;
#endif

	DEVMEM_MEMDESC *psRGXFWSigTAChecksMemDesc;
	IMG_UINT32 ui32SigTAChecksSize;

	DEVMEM_MEMDESC *psRGXFWSig3DChecksMemDesc;
	IMG_UINT32 ui32Sig3DChecksSize;

	IMG_VOID *pvLISRData;
	IMG_VOID *pvMISRData;

#if defined (PDUMP)
	IMG_BOOL abDumpedKCCBCtlAlready[RGXFWIF_DM_MAX];

#endif

} PVRSRV_RGXDEV_INFO;

typedef struct _RGX_TIMING_INFORMATION_ {
	IMG_UINT32 ui32CoreClockSpeed;
	IMG_BOOL bEnableActivePM;
	IMG_UINT32 ui32ActivePowManLatencyms;
} RGX_TIMING_INFORMATION;

typedef struct _RGX_DATA_ {
	/*! Timing information */
	RGX_TIMING_INFORMATION *psRGXTimingInfo;
} RGX_DATA;

/*
	RGX PDUMP register bank name (prefix)
*/
#define RGX_PDUMPREG_NAME		"RGXREG"

#endif				/* __RGXDEVICE_H__ */
