									    /*************************************************************************//*!
									       @File
									       @Title          RGX firmware utility routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX firmware utility routines
									       @License        Strictly Confidential.
    *//**************************************************************************/

#if !defined(__RGXFWUTILS_H__)
#define __RGXFWUTILS_H__

#include "rgxdevice.h"
#include "devicemem.h"
#include "device.h"

typedef struct {
	DEVMEM_MEMDESC *psCCBMemDesc;
	DEVMEM_MEMDESC *psCCBCtlMemDesc;
	DEVMEM_MEMDESC *psFWMemContextMemDesc;
} RGX_FWCOMCTX_CLEANUP;

/*
 * Firmware-only allocation (which are initialised by the host) must be aligned to the SLC cache line size.
 * This is because firmware-only allocations are GPU_CACHE_INCOHERENT and this causes problems
 * if two allocations share the same cache line; e.g. the initialisation of the second allocation won't
 * make it into the SLC cache because it has been already loaded when accessing the content of the first allocation.
 */
/* FIXME: 64 is the size of the SLC cache line. This value should probably be taken from the rgx_cr_defs.h file once available */
static INLINE PVRSRV_ERROR DevmemFwAllocate(PVRSRV_RGXDEV_INFO * psDevInfo,
					    IMG_DEVMEM_SIZE_T uiSize,
					    DEVMEM_FLAGS_T uiFlags,
					    DEVMEM_MEMDESC ** ppsMemDescPtr)
{
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	PVRSRV_ERROR eError;

	eError = DevmemAllocate(psDevInfo->psFirmwareHeap,
				uiSize, 64, uiFlags, ppsMemDescPtr);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	/*
	   We need to map it so the heap for this allocation
	   is set
	 */
	eError = DevmemMapToDevice(*ppsMemDescPtr,
				   psDevInfo->psFirmwareHeap, &sTmpDevVAddr);
	return eError;
}

static INLINE PVRSRV_ERROR DevmemFwAllocateExportable(PVRSRV_DEVICE_NODE *
						      psDeviceNode,
						      IMG_DEVMEM_SIZE_T uiSize,
						      DEVMEM_FLAGS_T uiFlags,
						      DEVMEM_MEMDESC **
						      ppsMemDescPtr)
{
	PVRSRV_RGXDEV_INFO *psDevInfo =
	    (PVRSRV_RGXDEV_INFO *) psDeviceNode->pvDevice;
	IMG_DEV_VIRTADDR sTmpDevVAddr;
	PVRSRV_ERROR eError;

	eError = DevmemAllocateExportable(IMG_NULL,
					  (IMG_HANDLE) psDeviceNode,
					  uiSize, 64, uiFlags, ppsMemDescPtr);
	if (eError != PVRSRV_OK) {
		return eError;
	}

	/*
	   We need to map it so the heap for this allocation
	   is set
	 */
	eError = DevmemMapToDevice(*ppsMemDescPtr,
				   psDevInfo->psFirmwareHeap, &sTmpDevVAddr);
	return eError;
}

static INLINE IMG_VOID DevmemFwFree(DEVMEM_MEMDESC * psMemDesc)
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
#define RFW_FWADDR_FLAG_NONE		(0)	/*!< Void flag */
#define RFW_FWADDR_NOREF_FLAG		(1U << 0)	/*!< It is safe to immediately release the reference to the pointer, 
							   otherwise RGXUnsetFirmwareAddress() must be call when finished. */
#define RFW_FWADDR_METACACHED_FLAG	(1U << 1)	/*!< The addr returned uses the meta dcache */

PVRSRV_ERROR RGXSetupFirmware(PVRSRV_DEVICE_NODE * psDeviceNode,
			      IMG_DEVMEM_SIZE_T ui32FWMemAllocSize,
			      DEVMEM_EXPORTCOOKIE **
			      ppsFWMemAllocServerExportCookie,
			      IMG_DEV_VIRTADDR * psFWMemDevVAddrBase,
			      IMG_BOOL bEnableSignatureChecks,
			      IMG_UINT32 ui32SignatureChecksBufSize,
			      IMG_UINT32 ui32RGXFWAlignChecksSize,
			      IMG_UINT32 * pui32RGXFWAlignChecks,
			      IMG_UINT32 ui32ConfigFlags,
			      RGXFWIF_DEV_VIRTADDR * psRGXFwInit);

IMG_VOID RGXFreeFirmware(PVRSRV_RGXDEV_INFO * psDevInfo);

									    /*************************************************************************//*!
									       @Function       RGXSetFirmwareAddress

									       @Description    Sets a pointer in a firmware data structure.

									       @Input          ppDest            Address of the pointer to set
									       @Input          psSrc             MemDesc describing the pointer
									       @Input          ui32Flags         Any combination of  RFW_FWADDR_*_FLAG

									       @Return                  IMG_VOID
    *//**************************************************************************/
IMG_VOID RGXSetFirmwareAddress(RGXFWIF_DEV_VIRTADDR * ppDest,
			       DEVMEM_MEMDESC * psSrc,
			       IMG_UINT32 uiOffset, IMG_UINT32 ui32Flags);

									    /*************************************************************************//*!
									       @Function       RGXUnsetFirmwareAddress

									       @Description    Unsets a pointer in a firmware data structure

									       @Input          psSrc             MemDesc describing the pointer

									       @Return                  IMG_VOID
    *//**************************************************************************/
IMG_VOID RGXUnsetFirmwareAddress(DEVMEM_MEMDESC * psSrc);

PVRSRV_ERROR RGXStartFirmware(PVRSRV_RGXDEV_INFO * psDevInfo);

PVRSRV_ERROR _RGXScheduleCommand(PVRSRV_RGXDEV_INFO * psDevInfo,
				 RGXFWIF_DM eKCCBType,
				 RGXFWIF_KCCB_CMD * psKCCBCmd,
				 IMG_UINT32 ui32CmdSize,
				 IMG_BOOL bPDumpContinuous);

									    /*************************************************************************//*!
									       @Function       RGXScheduleCommand

									       @Description    Sends the command to a particular DM

									       @Input          psDevInfo                        Device Info
									       @Input          eDM                                      To which DM the cmd is sent.
									       @Input          psKCCBCmd                        The cmd to send.
									       @Input          ui32CmdSize                      The cmd size.
									       @Input          bPDumpContinuous

									       @Return                  PVRSRV_ERROR
    *//**************************************************************************/
PVRSRV_ERROR RGXScheduleCommand(PVRSRV_RGXDEV_INFO * psDevInfo,
				RGXFWIF_DM eKCCBType,
				RGXFWIF_KCCB_CMD * psKCCBCmd,
				IMG_UINT32 ui32CmdSize,
				IMG_BOOL bPDumpContinuous);

									    /*************************************************************************//*!
									       @Function       RGXScheduleCommandAndWait

									       @Description    Schedules the command with RGXScheduleCommand and then waits 
									       for the FW to update a sync. The sync must be piggy backed on
									       the cmd, either by passyng a sync cmd or a cmd that cotains the
									       sync which the FW will eventually update. The sync is created in
									       the function, therefore the function provides a FWAddr and 
									       UpdateValue for that cmd.

									       @Input          psDevInfo                        Device Info
									       @Input          eDM                                      To which DM the cmd is sent.
									       @Input          psKCCBCmd                        The cmd to send.
									       @Input          ui32CmdSize                      The cmd size.
									       @Input          puiSyncObjFWAddr Pointer to the location with the FWAddr of 
									       the sync.
									       @Input          puiUpdateValue           Pointer to the location with the update 
									       value of the sync.
									       @Input          bPDumpContinuous

									       @Return                  PVRSRV_ERROR
    *//**************************************************************************/
PVRSRV_ERROR RGXScheduleCommandAndWait(PVRSRV_RGXDEV_INFO * psDevInfo,
				       RGXFWIF_DM eDM,
				       RGXFWIF_KCCB_CMD * psKCCBCmd,
				       IMG_UINT32 ui32CmdSize,
				       IMG_UINT32 * puiSyncObjDevVAddr,
				       IMG_UINT32 * puiUpdateValue,
				       IMG_BOOL bPDumpContinuous);

PVRSRV_ERROR RGXFirmwareUnittests(PVRSRV_RGXDEV_INFO * psDevInfo);

PVRSRV_ERROR RGXInitFWCommonContext(RGXFWIF_FWCOMMONCONTEXT * psFWComContext,
				    DEVMEM_MEMDESC * psCCBMemDesc,
				    DEVMEM_MEMDESC * psCCBCtlMemDesc,
				    DEVMEM_MEMDESC * psFWMemContextMemDesc,
				    IMG_UINT32 ui32Priority,
				    IMG_DEV_VIRTADDR * psMCUFenceAddr,
				    RGX_FWCOMCTX_CLEANUP * psCleanupData);

PVRSRV_ERROR RGXDeinitFWCommonContext(RGX_FWCOMCTX_CLEANUP * psCleanupData);

									    /*************************************************************************//*!
									       @Function       RGXWaitForFWOp

									       @Description    Send a sync command and wait to be signaled.

									       @Input          psDevInfo                        Device Info
									       @Input          eDM                                      To which DM the cmd is sent.
									       @Input          bPDumpContinuous 

									       @Return                  IMG_VOID
    *//**************************************************************************/
PVRSRV_ERROR RGXWaitForFWOp(PVRSRV_RGXDEV_INFO * psDevInfo,
			    RGXFWIF_DM eDM, IMG_BOOL bPDumpContinuous);

#endif				/* __RGXFWUTILS_H__ */
/******************************************************************************
 End of file (rgxfwutils.h)
******************************************************************************/
