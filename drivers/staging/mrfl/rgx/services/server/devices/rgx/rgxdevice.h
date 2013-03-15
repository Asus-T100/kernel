/*************************************************************************/ /*!
@File
@Title          RGX device node header file
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the RGX device node
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

typedef struct _PVRSRV_RGXDEV_INFO_
{
	PVRSRV_DEVICE_TYPE		eDeviceType;
	PVRSRV_DEVICE_CLASS		eDeviceClass;
	PVRSRV_DEVICE_NODE		*psDeviceNode;

	IMG_UINT8				ui8VersionMajor;
	IMG_UINT8				ui8VersionMinor;
	IMG_UINT32				ui32CoreConfig;
	IMG_UINT32				ui32CoreFlags;

	/* Kernel mode linear address of device registers */
	IMG_PVOID				pvRegsBaseKM;

	/* FIXME: The alloc for this should go through OSAllocMem in future */
	IMG_HANDLE				hRegMapping;

	/* System physical address of device registers*/
	IMG_CPU_PHYADDR			sRegsPhysBase;
	/*  Register region size in bytes */
	IMG_UINT32				ui32RegSize;

	/*  RGX clock speed */
	IMG_UINT32				ui32CoreClockSpeed;

	PVRSRV_STUB_PBDESC		*psStubPBDescListKM;


	/* Firmware memory context info */
	DEVMEM_CONTEXT			*psKernelDevmemCtx;
	DEVMEM_HEAP				*psFirmwareHeap;
	MMU_CONTEXT				*psKernelMMUCtx;
	IMG_UINT32				ui32KernelCatBase;

	IMG_VOID				*pvDeviceMemoryHeap;
	
	DEVMEM_MEMDESC			*apsKernelCCBCtlMemDesc[RGXFWIF_DM_MAX];	/*!< memdesc for kernel CCB control */
	RGXFWIF_KCCB_CTL		*apsKernelCCBCtl[RGXFWIF_DM_MAX];			/*!< kernel CCB control kernel mapping */
	DEVMEM_MEMDESC			*apsKernelCCBMemDesc[RGXFWIF_DM_MAX];		/*!< memdesc for kernel CCB */
	IMG_UINT8				*apsKernelCCB[RGXFWIF_DM_MAX];				/*!< kernel CCB kernel mapping */

	/*
		if we don't preallocate the pagetables we must 
		insert newly allocated page tables dynamically 
	*/
	IMG_VOID				*pvMMUContextList;

	IMG_UINT32				ui32ClkGateStatusReg;
	IMG_UINT32				ui32ClkGateStatusMask;
	RGX_SCRIPTS				sScripts;

	DEVMEM_MEMDESC			*psRGXFWMemDesc;
	DEVMEM_EXPORTCOOKIE		sRGXFWExportCookie;

	DEVMEM_MEMDESC			*psRGXFWIfTraceBufCtlMemDesc;
	RGXFWIF_TRACEBUF		*psRGXFWIfTraceBuf;

	DEVMEM_MEMDESC			*psRGXFWIfInitMemDesc;

#if defined(RGXFW_ALIGNCHECKS)
	DEVMEM_MEMDESC			*psRGXFWAlignChecksMemDesc;	
#endif

	DEVMEM_MEMDESC			*psRGXFWSigTAChecksMemDesc;	
	IMG_UINT32				ui32SigTAChecksSize;

	DEVMEM_MEMDESC			*psRGXFWSig3DChecksMemDesc;	
	IMG_UINT32				ui32Sig3DChecksSize;

	IMG_VOID				*pvLISRData;
	IMG_VOID				*pvMISRData;
	
	DEVMEM_MEMDESC			*psRGXBIFFaultAddressMemDesc;

#if defined(FIX_HW_BRN_37200)
	DEVMEM_MEMDESC			*psRGXFWHWBRN37200MemDesc;
#endif

#if defined (PDUMP)
	IMG_BOOL				abDumpedKCCBCtlAlready[RGXFWIF_DM_MAX];
	
#endif	

	/*! Handle on transport layer stream used to export HWPerf data to user
	 * side clients. Set during initialisation if the app hint turns bit 7
	 * 'Enable HWPerf' on in the ConfigFlags sent to the FW. FW stores this
	 * bit in the RGXFW_CTL.ui32StateFlags member. It may also get
	 * set by the API RGXCtrlHWPerf(). Thus this member may be 0 if HWPerf is
	 * not enabled as this member is created and destroyed on demand.
	 */
	IMG_HANDLE				hHWPerfStream;

	PSYNC_PRIM_CONTEXT		hSyncPrimContext;
	PVRSRV_CLIENT_SYNC_PRIM *psPowSyncPrim;

	IMG_VOID (*pfnActivePowerCheck) (PVRSRV_DEVICE_NODE *psDeviceNode);

} PVRSRV_RGXDEV_INFO;



typedef struct _RGX_TIMING_INFORMATION_
{
	IMG_UINT32			ui32CoreClockSpeed;
	IMG_BOOL			bEnableActivePM;
	IMG_BOOL			bEnableRDPowIsland;
} RGX_TIMING_INFORMATION;

typedef struct _RGX_DATA_
{
	/*! Timing information */
	RGX_TIMING_INFORMATION	*psRGXTimingInfo;
} RGX_DATA;


/*
	RGX PDUMP register bank name (prefix)
*/
#define RGX_PDUMPREG_NAME		"RGXREG"

#endif /* __RGXDEVICE_H__ */
