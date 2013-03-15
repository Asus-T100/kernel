/*************************************************************************/ /*!
@File
@Title          RGX firmware utility routines
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    RGX firmware utility routines
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

#if !defined(__RGXFWUTILS_H__)
#define __RGXFWUTILS_H__

#include "rgxdevice.h"
#include "devicemem.h"
#include "device.h"
#include "pvrsrv.h"

typedef struct
{
	DEVMEM_MEMDESC		*psCCBMemDesc;
	DEVMEM_MEMDESC		*psCCBCtlMemDesc;
	DEVMEM_MEMDESC		*psFWMemContextMemDesc;
	DEVMEM_MEMDESC		*psFWFrameworkMemDesc;
} RGX_FWCOMCTX_CLEANUP;

/*
 * Firmware-only allocation (which are initialised by the host) must be aligned to the SLC cache line size.
 * This is because firmware-only allocations are GPU_CACHE_INCOHERENT and this causes problems
 * if two allocations share the same cache line; e.g. the initialisation of the second allocation won't
 * make it into the SLC cache because it has been already loaded when accessing the content of the first allocation.
 */
static INLINE PVRSRV_ERROR DevmemFwAllocate(PVRSRV_RGXDEV_INFO *psDevInfo,
											IMG_DEVMEM_SIZE_T uiSize,
											DEVMEM_FLAGS_T uiFlags,
											DEVMEM_MEMDESC **ppsMemDescPtr)
{
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	PVRSRV_ERROR eError;

	eError = DevmemAllocate(psDevInfo->psFirmwareHeap,
							uiSize,
							ROGUE_CACHE_LINE_SIZE,
							uiFlags,
							ppsMemDescPtr);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	/*
		We need to map it so the heap for this allocation
		is set
	*/
	eError = DevmemMapToDevice(*ppsMemDescPtr,
							   psDevInfo->psFirmwareHeap,
							   &sTmpDevVAddr);
	return eError;
}

static INLINE PVRSRV_ERROR DevmemFwAllocateExportable(PVRSRV_DEVICE_NODE *psDeviceNode,
													  IMG_DEVMEM_SIZE_T uiSize,
													  DEVMEM_FLAGS_T uiFlags,
													  DEVMEM_MEMDESC **ppsMemDescPtr)
{
	PVRSRV_RGXDEV_INFO *psDevInfo = (PVRSRV_RGXDEV_INFO *) psDeviceNode->pvDevice;
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	PVRSRV_ERROR eError;

	eError = DevmemAllocateExportable(IMG_NULL,
									  (IMG_HANDLE) psDeviceNode,
									  uiSize,
									  64,
									  uiFlags,
									  ppsMemDescPtr);
	if (eError != PVRSRV_OK)
	{
		return eError;
	}

	/*
		We need to map it so the heap for this allocation
		is set
	*/
	eError = DevmemMapToDevice(*ppsMemDescPtr,
							   psDevInfo->psFirmwareHeap,
							   &sTmpDevVAddr);
	return eError;
}

static INLINE IMG_VOID DevmemFwFree(DEVMEM_MEMDESC *psMemDesc)
{
	DevmemReleaseDevVirtAddr(psMemDesc);
	DevmemFree(psMemDesc);
}

/*
 * This FW Common Context is only mapped into kernel for initialisation and cleanup purposes.
 * Otherwise this allocation is only used by the FW.
 * Therefore the GPU cache doesn't need coherency,
 * and write-combine is suffice on the CPU side (WC buffer will be flushed at the first kick)
 */
#define RGX_FWCOMCTX_ALLOCFLAGS	(PVRSRV_MEMALLOCFLAG_DEVICE_FLAG(PMMETA_PROTECT) | \
								 PVRSRV_MEMALLOCFLAG_GPU_READABLE | \
								 PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE | \
								 PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT | \
								 PVRSRV_MEMALLOCFLAG_CPU_READABLE | \
								 PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE | \
								 PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE | \
								 PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE | \
								 PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC)

/******************************************************************************
 * RGXSetFirmwareAddress Flags
 *****************************************************************************/
#define RFW_FWADDR_FLAG_NONE		(0)			/*!< Void flag */
#define RFW_FWADDR_NOREF_FLAG		(1U << 0)	/*!< It is safe to immediately release the reference to the pointer, 
												  otherwise RGXUnsetFirmwareAddress() must be call when finished. */
#define RFW_FWADDR_METACACHED_FLAG	(1U << 1)	/*!< The addr returned uses the meta dcache */


PVRSRV_ERROR RGXSetupFirmware(PVRSRV_DEVICE_NODE	*psDeviceNode, 
							  IMG_DEVMEM_SIZE_T		ui32FWMemAllocSize,
							  DEVMEM_EXPORTCOOKIE	**ppsFWMemAllocServerExportCookie,
							  IMG_DEV_VIRTADDR		*psFWMemDevVAddrBase,
							  IMG_BOOL				bEnableSignatureChecks,
							  IMG_UINT32			ui32SignatureChecksBufSize,
							  IMG_UINT32			ui32RGXFWAlignChecksSize,
							  IMG_UINT32			*pui32RGXFWAlignChecks,
							  IMG_UINT32			ui32ConfigFlags,
							  IMG_UINT32			ui32LogType,
							  RGXFWIF_DEV_VIRTADDR	*psRGXFwInit);

IMG_VOID RGXFreeFirmware(PVRSRV_RGXDEV_INFO 	*psDevInfo);

/*************************************************************************/ /*!
@Function       RGXSetFirmwareAddress

@Description    Sets a pointer in a firmware data structure.

@Input          ppDest		 Address of the pointer to set
@Input          psSrc		 MemDesc describing the pointer
@Input          ui32Flags	 Any combination of  RFW_FWADDR_*_FLAG

@Return			IMG_VOID
*/ /**************************************************************************/
IMG_VOID RGXSetFirmwareAddress(RGXFWIF_DEV_VIRTADDR	*ppDest,
							   DEVMEM_MEMDESC		*psSrc,
							   IMG_UINT32			uiOffset,
							   IMG_UINT32			ui32Flags);

/*************************************************************************/ /*!
@Function       RGXUnsetFirmwareAddress

@Description    Unsets a pointer in a firmware data structure

@Input          psSrc		 MemDesc describing the pointer

@Return			IMG_VOID
*/ /**************************************************************************/
IMG_VOID RGXUnsetFirmwareAddress(DEVMEM_MEMDESC			*psSrc);

PVRSRV_ERROR RGXStartFirmware(PVRSRV_RGXDEV_INFO 	*psDevInfo);

/*!
******************************************************************************

 @Function	RGXScheduleProcessQueuesKM

 @Description - Software command complete handler
				(sends uncounted kicks for all the DMs)

 @Input hCmdCompHandle - RGX device node

******************************************************************************/
IMG_IMPORT
IMG_VOID RGXScheduleProcessQueuesKM(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle);

/*************************************************************************/ /*!
@Function       RGXSendCommandWithPowLock

@Description    Sends a command to a particular DM without honouring
				pending cache operations but taking the power lock.

@Input          psDevInfo			Device Info
@Input          eDM					To which DM the cmd is sent.
@Input          psKCCBCmd			The cmd to send.
@Input          ui32CmdSize			The cmd size.
@Input          bPDumpContinuous

@Return			PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXSendCommandWithPowLock(PVRSRV_RGXDEV_INFO 	*psDevInfo,
										RGXFWIF_DM			eKCCBType,
									 	RGXFWIF_KCCB_CMD	*psKCCBCmd,
									 	IMG_UINT32			ui32CmdSize,
									 	IMG_BOOL			bPDumpContinuous);

/*************************************************************************/ /*!
@Function       RGXSendCommandRaw

@Description    Sends a command to a particular DM without honouring
				pending cache operations or the power lock.

@Input          psDevInfo			Device Info
@Input          eDM					To which DM the cmd is sent.
@Input          psKCCBCmd			The cmd to send.
@Input          ui32CmdSize			The cmd size.
@Input          bPDumpContinuous

@Return			PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXSendCommandRaw(PVRSRV_RGXDEV_INFO 	*psDevInfo,
								 RGXFWIF_DM			eKCCBType,
								 RGXFWIF_KCCB_CMD	*psKCCBCmd,
								 IMG_UINT32			ui32CmdSize,
								 PDUMP_FLAGS_T		uiPdumpFlags);


/*************************************************************************/ /*!
@Function       RGXScheduleCommand

@Description    Sends a command to a particular DM

@Input          psDevInfo			Device Info
@Input          eDM					To which DM the cmd is sent.
@Input          psKCCBCmd			The cmd to send.
@Input          ui32CmdSize			The cmd size.
@Input          bPDumpContinuous

@Return			PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXScheduleCommand(PVRSRV_RGXDEV_INFO 	*psDevInfo,
								RGXFWIF_DM			eKCCBType,
								RGXFWIF_KCCB_CMD	*psKCCBCmd,
								IMG_UINT32			ui32CmdSize,
								IMG_BOOL			bPDumpContinuous);

/*************************************************************************/ /*!
@Function       RGXScheduleCommandAndWait

@Description    Schedules the command with RGXScheduleCommand and then waits 
				for the FW to update a sync. The sync must be piggy backed on
				the cmd, either by passing a sync cmd or a cmd that contains the
				sync which the FW will eventually update. The sync is created in
				the function, therefore the function provides a FWAddr and 
				UpdateValue for that cmd.

@Input          psDevInfo			Device Info
@Input          eDM					To which DM the cmd is sent.
@Input          psKCCBCmd			The cmd to send.
@Input          ui32CmdSize			The cmd size.
@Input          puiSyncObjFWAddr	Pointer to the location with the FWAddr of 
									the sync.
@Input          puiUpdateValue		Pointer to the location with the update 
									value of the sync.
@Input          bPDumpContinuous

@Return			PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR RGXScheduleCommandAndWait(PVRSRV_RGXDEV_INFO 	*psDevInfo,
									   RGXFWIF_DM			eDM,
									   RGXFWIF_KCCB_CMD		*psKCCBCmd,
									   IMG_UINT32			ui32CmdSize,
									   IMG_UINT32			*puiSyncObjDevVAddr,
									   IMG_UINT32			*puiUpdateValue,
									   PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
									   IMG_BOOL				bPDumpContinuous);

PVRSRV_ERROR RGXFirmwareUnittests(PVRSRV_RGXDEV_INFO *psDevInfo);


/*! ***********************************************************************//**
@brief          Copy framework state into FW addressable buffer

@param          psFWFrameworkMemDesc
@param          pbyGPUFRegisterList
@param          ui32FrameworkRegisterSize

@returns        PVRSRV_ERROR 
*/ /**************************************************************************/
PVRSRV_ERROR PVRSRVRGXFrameworkCopyRegisters(DEVMEM_MEMDESC * psFWFrameworkMemDesc,
		IMG_PBYTE                                           pbyGPUFRegisterList,
		IMG_UINT32                                          ui32FrameworkRegisterSize);


/*! ***********************************************************************//**
@brief          Create FW addressable buffer for framework

@param          psDeviceNode
@param          ppsFWFrameworkMemDesc
@param          ui32FrameworkRegisterSize

@returns        PVRSRV_ERROR 
*/ /**************************************************************************/
PVRSRV_ERROR PVRSRVRGXFrameworkCreateKM(PVRSRV_DEVICE_NODE * psDeviceNode,
										DEVMEM_MEMDESC     ** ppsFWFrameworkMemDesc,
										IMG_UINT32         ui32FrameworkRegisterSize);

PVRSRV_ERROR RGXInitFWCommonContext(RGXFWIF_FWCOMMONCONTEXT		*psFWComContext,
									  DEVMEM_MEMDESC 			*psCCBMemDesc,
									  DEVMEM_MEMDESC 			*psCCBCtlMemDesc,
									  DEVMEM_MEMDESC			*psFWMemContextMemDesc,
									  DEVMEM_MEMDESC			*psFWFrameworkMemDesc,
									  IMG_UINT32				ui32Priority,
									  IMG_DEV_VIRTADDR			*psMCUFenceAddr,
									  RGX_FWCOMCTX_CLEANUP		*psCleanupData);

PVRSRV_ERROR RGXDeinitFWCommonContext(RGX_FWCOMCTX_CLEANUP *psCleanupData);


/*************************************************************************/ /*!
@Function       RGXWaitForFWOp

@Description    Send a sync command and wait to be signaled.

@Input          psDevInfo			Device Info
@Input          eDM					To which DM the cmd is sent.
@Input          bPDumpContinuous	

@Return			IMG_VOID
*/ /**************************************************************************/
PVRSRV_ERROR RGXWaitForFWOp(PVRSRV_RGXDEV_INFO	*psDevInfo,
									RGXFWIF_DM	eDM,
									PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
									IMG_BOOL	bPDumpContinuous);

/*!
******************************************************************************

 @Function	RGXFWRequestCommonContextCleanUp

 @Description Schedules a FW common context cleanup. The firmware will doesn't
              block waiting for the resource to become idle but rather notifies
              the host that the resources is busy.

 @Input psDeviceNode - pointer to device node

 @Input psFWContext - firmware address of the context to be cleaned up

 @Input eDM - Data master, to which the cleanup command should be send

******************************************************************************/
PVRSRV_ERROR RGXFWRequestCommonContextCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
											  PRGXFWIF_FWCOMMONCONTEXT psFWContext,
											  PVRSRV_CLIENT_SYNC_PRIM *psSyncPrim,
											  RGXFWIF_DM eDM);

/*!
******************************************************************************

 @Function	RGXFWRequestHWRTDataCleanUp

 @Description Schedules a FW HWRTData memory cleanup. The firmware will doesn't
              block waiting for the resource to become idle but rather notifies
              the host that the resources is busy.

 @Input psDeviceNode - pointer to device node

 @Input psHWRTData - firmware address of the HWRTData to be cleaned up

 @Input eDM - Data master, to which the cleanup command should be send

 ******************************************************************************/
PVRSRV_ERROR RGXFWRequestHWRTDataCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
										 PRGXFWIF_HWRTDATA psHWRTData,
										 PVRSRV_CLIENT_SYNC_PRIM *psSync,
										 RGXFWIF_DM eDM);

/*!
******************************************************************************

 @Function	RGXFWRequestFreeListCleanUp

 @Description Schedules a FW FreeList cleanup. The firmware will doesn't block
              waiting for the resource to become idle but rather notifies the
              host that the resources is busy.

 @Input psDeviceNode - pointer to device node

 @Input psHWRTData - firmware address of the HWRTData to be cleaned up

 @Input eDM - Data master, to which the cleanup command should be send

 ******************************************************************************/
PVRSRV_ERROR RGXFWRequestFreeListCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
										 PRGXFWIF_FREELIST psFWFreeList,
										 PVRSRV_CLIENT_SYNC_PRIM *psSync);

/*!
******************************************************************************

 @Function	RGXFWRequestZSBufferCleanUp

 @Description Schedules a FW ZS Buffer cleanup. The firmware will doesn't block
              waiting for the resource to become idle but rather notifies the
              host that the resources is busy.

 @Input psDeviceNode - pointer to device node

 @Input psFWZSBuffer - firmware address of the ZS Buffer to be cleaned up

 @Input eDM - Data master, to which the cleanup command should be send

 ******************************************************************************/

PVRSRV_ERROR RGXFWRequestZSBufferCleanUp(PVRSRV_DEVICE_NODE *psDeviceNode,
										 PRGXFWIF_ZSBUFFER psFWZSBuffer,
										 PVRSRV_CLIENT_SYNC_PRIM *psSync);

#endif /* __RGXFWUTILS_H__ */
/******************************************************************************
 End of file (rgxfwutils.h)
******************************************************************************/
