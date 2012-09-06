									    /*************************************************************************//*!
									       @File
									       @Title          RGX TA/3D routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX TA/3D routines
									       @License        Strictly Confidential.
    *//**************************************************************************/
/* for the offsetof macro */
#include <stddef.h>

#include "pdump_km.h"
#include "pvr_debug.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgxta3d.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "osfunc.h"
#include "pvrsrv.h"

#include "rgx_cr_defs_km.h"
#include "rgx_fwif_km.h"

/* Create HWRTDataSet */
IMG_EXPORT PVRSRV_ERROR RGXCreateHWRTData(PVRSRV_DEVICE_NODE * psDeviceNode, IMG_UINT32 psRenderTarget,	/* FIXME this should not be IMG_UINT32 */
					  IMG_DEV_VIRTADDR psPMMListDevVAddr,
					  IMG_UINT32
					  apsFreeLists[RGX_NUM_FREELIST_TYPES],
					  RGX_RTDATA_CLEANUP_DATA **
					  ppsCleanupData,
					  DEVMEM_MEMDESC ** ppsMemDesc,
					  IMG_UINT32 * puiHWRTData)
{
	PVRSRV_ERROR eError;
	PVRSRV_RGXDEV_INFO *psDevInfo;
	RGXFWIF_DEV_VIRTADDR pFirmwareAddr;
	RGXFWIF_HWRTDATA *psHWRTData;
	IMG_UINT32 ui32Loop;
	RGX_RTDATA_CLEANUP_DATA *psTmpCleanup;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto AllocError;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	psDevInfo = psDeviceNode->pvDevice;

	/*
	 * This FW RT-Data is only mapped into kernel for initialisation.
	 * Otherwise this allocation is only used by the FW.
	 * Therefore the GPU cache doesn't need coherency,
	 * and write-combine is suffice on the CPU side (WC buffer will be flushed at the first TA-kick)
	 */
	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(RGXFWIF_HWRTDATA),
				  PVRSRV_MEMALLOCFLAG_DEVICE_FLAG
				  (PMMETA_PROTECT) |
				  PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
				  PVRSRV_MEMALLOCFLAG_GPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
				  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
				  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
				  ppsMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXCreateHWRTData: DevmemAllocate for RGX_FWIF_HWRTDATA failed\n"));
		goto FWAllocateError;
	}

	psTmpCleanup->psDeviceNode = psDeviceNode;
	psTmpCleanup->psFWHWRTDataMemDesc = *ppsMemDesc;

	RGXSetFirmwareAddress(&pFirmwareAddr, *ppsMemDesc, 0,
			      RFW_FWADDR_FLAG_NONE);

	*puiHWRTData = pFirmwareAddr.ui32Addr;

	DevmemAcquireCpuVirtAddr(*ppsMemDesc, (IMG_VOID **) & psHWRTData);

	/* FIXME: MList is something that that PM writes physical addresses to,
	 * so ideally its best allocated in kernel */
	psHWRTData->s3DCleanupState.eCleanupState = RGXFWIF_CLEANUP_NONE;
	psHWRTData->sTACleanupState.eCleanupState = RGXFWIF_CLEANUP_NONE;
	psHWRTData->psPMMListDevVAddr = psPMMListDevVAddr;
	psHWRTData->psParentRenderTarget.ui32Addr = psRenderTarget;
	for (ui32Loop = 0; ui32Loop < RGX_NUM_FREELIST_TYPES; ui32Loop++) {
		psHWRTData->apsFreeLists[ui32Loop] = *((PRGXFWIF_FREELIST *) & (apsFreeLists[ui32Loop]));	/* FIXME: Fix pointer type casting */
	}
	PDUMPCOMMENT("Dump HWRTData 0x%08X", *puiHWRTData);
	DevmemPDumpLoadMem(*ppsMemDesc, 0, sizeof(*psHWRTData),
			   PDUMP_FLAGS_CONTINUOUS);
	DevmemReleaseCpuVirtAddr(*ppsMemDesc);
	return PVRSRV_OK;

 FWAllocateError:
	OSFreeMem(psTmpCleanup);

 AllocError:
	return eError;
}

/* Destroy HWRTDataSet */
IMG_EXPORT
    PVRSRV_ERROR RGXDestroyHWRTData(RGX_RTDATA_CLEANUP_DATA * psCleanupData)
{

	PRGXFWIF_HWRTDATA psHWRTData;

	RGXSetFirmwareAddress(&psHWRTData, psCleanupData->psFWHWRTDataMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	/* Cleanup HWRTData in TA */
	RGXRequestHWRTDataCleanUp(psCleanupData->psDeviceNode,
				  psHWRTData, RGXFWIF_DM_TA);

	/* Cleanup HWRTData in 3D */
	RGXRequestHWRTDataCleanUp(psCleanupData->psDeviceNode,
				  psHWRTData, RGXFWIF_DM_3D);

	RGXUnsetFirmwareAddress(psCleanupData->psFWHWRTDataMemDesc);
	DevmemFwFree(psCleanupData->psFWHWRTDataMemDesc);

	OSFreeMem(psCleanupData);

	return PVRSRV_OK;
}

IMG_EXPORT
    PVRSRV_ERROR RGXCreateFreeList(PVRSRV_DEVICE_NODE * psDeviceNode,
				   IMG_UINT32 ui32TotalPMPages,
				   IMG_DEV_VIRTADDR psFreeListDevVAddr,
				   DEVMEM_MEMDESC ** psFWFreeListMemDesc,
				   IMG_UINT32 * sFreeListFWDevVAddr)
{
	PVRSRV_ERROR eError;
	RGXFWIF_FREELIST *psFWFreeList;
	RGXFWIF_DEV_VIRTADDR pFirmwareAddr;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	/*
	 * This FW FreeList context is only mapped into kernel for initialisation.
	 * Otherwise this allocation is only used by the FW.
	 * Therefore the GPU cache doesn't need coherency,
	 * and write-combine is suffice on the CPU side (WC buffer will be flushed at the first TA-kick)
	 */
	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(*psFWFreeList),
				  PVRSRV_MEMALLOCFLAG_DEVICE_FLAG
				  (PMMETA_PROTECT) |
				  PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
				  PVRSRV_MEMALLOCFLAG_GPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
				  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
				  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
				  psFWFreeListMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXCreateFreeList: DevmemAllocate for RGXFWIF_FREELIST failed\n"));
		return eError;
	}
	RGXSetFirmwareAddress(&pFirmwareAddr, *psFWFreeListMemDesc, 0,
			      RFW_FWADDR_FLAG_NONE);
	*sFreeListFWDevVAddr = pFirmwareAddr.ui32Addr;

	DevmemAcquireCpuVirtAddr(*psFWFreeListMemDesc,
				 (IMG_VOID **) & psFWFreeList);
	psFWFreeList->ui32TotalPMPages = ui32TotalPMPages;
	psFWFreeList->ui64CurrentStackTop = ui32TotalPMPages - 1;
	psFWFreeList->psFreeListDevVAddr = psFreeListDevVAddr;
	PDUMPCOMMENT("Dump FW FreeList");
	DevmemPDumpLoadMem(*psFWFreeListMemDesc, 0, sizeof(*psFWFreeList),
			   PDUMP_FLAGS_CONTINUOUS);
	DevmemReleaseCpuVirtAddr(*psFWFreeListMemDesc);
	return PVRSRV_OK;
}

/*
	RGXDestroyFreeList
*/
IMG_EXPORT PVRSRV_ERROR RGXDestroyFreeList(DEVMEM_MEMDESC * psMemDesc)
{
	RGXUnsetFirmwareAddress(psMemDesc);
	DevmemFwFree(psMemDesc);
	return PVRSRV_OK;
}

/*
	RGXCreateRenderTarget
*/
IMG_EXPORT
    PVRSRV_ERROR RGXCreateRenderTarget(PVRSRV_DEVICE_NODE * psDeviceNode,
				       IMG_DEV_VIRTADDR psVHeapTableDevVAddr,
				       DEVMEM_MEMDESC ** psRenderTargetMemDesc,
				       IMG_UINT32 * sRenderTargetFWDevVAddr)
{
	PVRSRV_ERROR eError;
	RGXFWIF_RENDER_TARGET *psRenderTarget;
	RGXFWIF_DEV_VIRTADDR pFirmwareAddr;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	/*
	 * This FW render target context is only mapped into kernel for initialisation.
	 * Otherwise this allocation is only used by the FW.
	 * Therefore the GPU cache doesn't need coherency,
	 * and write-combine is suffice on the CPU side (WC buffer will be flushed at the first TA-kick)
	 */
	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(*psRenderTarget),
				  PVRSRV_MEMALLOCFLAG_DEVICE_FLAG
				  (PMMETA_PROTECT) |
				  PVRSRV_MEMALLOCFLAG_ZERO_ON_ALLOC |
				  PVRSRV_MEMALLOCFLAG_GPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_GPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_GPU_CACHE_INCOHERENT |
				  PVRSRV_MEMALLOCFLAG_CPU_READABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE |
				  PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE |
				  PVRSRV_MEMALLOCFLAG_KERNEL_CPU_MAPPABLE,
				  psRenderTargetMemDesc);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXCreateFreeList: DevmemAllocate for RGXFWIF_FREELIST failed\n"));
		return eError;
	}
	RGXSetFirmwareAddress(&pFirmwareAddr, *psRenderTargetMemDesc, 0,
			      RFW_FWADDR_FLAG_NONE);
	*sRenderTargetFWDevVAddr = pFirmwareAddr.ui32Addr;

	DevmemAcquireCpuVirtAddr(*psRenderTargetMemDesc,
				 (IMG_VOID **) & psRenderTarget);
	psRenderTarget->psVHeapTableDevVAddr = psVHeapTableDevVAddr;
	PDUMPCOMMENT("Dump RenderTarget");
	DevmemPDumpLoadMem(*psRenderTargetMemDesc, 0, sizeof(*psRenderTarget),
			   PDUMP_FLAGS_CONTINUOUS);
	DevmemReleaseCpuVirtAddr(*psRenderTargetMemDesc);
	return PVRSRV_OK;
}

/*
	RGXDestroyRenderTarget
*/
IMG_EXPORT PVRSRV_ERROR RGXDestroyRenderTarget(DEVMEM_MEMDESC * psMemDesc)
{
	RGXUnsetFirmwareAddress(psMemDesc);
	DevmemFwFree(psMemDesc);
	return PVRSRV_OK;
}

/*
 * PVRSRVRGXCreateRenderContextKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXCreateRenderContextKM(PVRSRV_DEVICE_NODE *
						psDeviceNode,
						DEVMEM_MEMDESC * psTACCBMemDesc,
						DEVMEM_MEMDESC *
						psTACCBCtlMemDesc,
						DEVMEM_MEMDESC * ps3DCCBMemDesc,
						DEVMEM_MEMDESC *
						ps3DCCBCtlMemDesc,
						RGX_RC_CLEANUP_DATA **
						ppsCleanupData,
						DEVMEM_MEMDESC **
						ppsFWRenderContextMemDesc,
						DEVMEM_MEMDESC **
						ppsFWRenderContextStateMemDesc,
						IMG_UINT32 ui32Priority,
						IMG_DEV_VIRTADDR sMCUFenceAddr,
						IMG_DEV_VIRTADDR
						sVDMCallStackAddr,
						IMG_HANDLE hMemCtxPrivData)
{
	PVRSRV_ERROR eError;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_FWRENDERCONTEXT *psFWRenderContext;
	RGX_RC_CLEANUP_DATA *psTmpCleanup;
	RGXFWIF_TACTX_STATE *psContextState;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/*
	   Allocate device memory for the firmware render context.
	 */
	PDUMPCOMMENT("Allocate RGX firmware render context");

	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(*psFWRenderContext),
				  RGX_FWCOMCTX_ALLOCFLAGS,
				  ppsFWRenderContextMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateRenderContextKM: Failed to allocate firmware render context (%u)",
			 eError));
		goto e0;
	}
	psTmpCleanup->psFWRenderContextMemDesc = *ppsFWRenderContextMemDesc;
	psTmpCleanup->psDeviceNode = psDeviceNode;

	/*
	   Temporarily map the firmware render context to the kernel.
	 */
	eError = DevmemAcquireCpuVirtAddr(*ppsFWRenderContextMemDesc,
					  (IMG_VOID **) & psFWRenderContext);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context (%u)",
			 eError));
		goto e1;
	}

	/*
	   Allocate device memory for the firmware GPU context suspend state.
	   Note: the FW reads/writes the state to memory by accessing the GPU register interface.
	 */
	PDUMPCOMMENT("Allocate RGX firmware 3D context suspend state");

	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(RGXFWIF_CTX_STATE),
				  RGX_FWCOMCTX_ALLOCFLAGS,
				  ppsFWRenderContextStateMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateRenderContextKM: Failed to allocate firmware GPU context suspend state (%u)",
			 eError));
		goto e2;
	}
	psTmpCleanup->psFWRenderContextStateMemDesc =
	    *ppsFWRenderContextStateMemDesc;

	/* Init TA FW common context */
	eError = RGXInitFWCommonContext(&psFWRenderContext->sTAContext,
					psTACCBMemDesc,
					psTACCBCtlMemDesc,
					hMemCtxPrivData,
					ui32Priority,
					&sMCUFenceAddr,
					&psTmpCleanup->sFWTAContextCleanup);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateRenderContextKM: Failed to init TA fw common context (%u)",
			 eError));
		goto e3;
	}

	/* Init 3D FW common context */
	eError = RGXInitFWCommonContext(&psFWRenderContext->s3DContext,
					ps3DCCBMemDesc,
					ps3DCCBCtlMemDesc,
					hMemCtxPrivData,
					ui32Priority,
					&sMCUFenceAddr,
					&psTmpCleanup->sFW3DContextCleanup);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateRenderContextKM: Failed to init 3D fw common context (%u)",
			 eError));
		goto e3;
	}

	/*
	 * Set the firmware GPU context state buffer. Note the offset allows the same buffer
	 * to be used by the TA and 3D separately by specifying custom offset. The
	 * buffer itself is allocated/deallocated with the granularity of the parent
	 * structure.
	 * 
	 * The common context stores a dword pointer (FW) so we can cast the generic buffer to
	 * the correct TA/3D state structure type in the FW.
	 */
	RGXSetFirmwareAddress(&psFWRenderContext->sTAContext.psContextState, *ppsFWRenderContextStateMemDesc, offsetof(RGXFWIF_CTX_STATE, sTAContextState),	/* TA state */
			      RFW_FWADDR_FLAG_NONE);

	eError = DevmemAcquireCpuVirtAddr(*ppsFWRenderContextStateMemDesc,
					  (IMG_VOID **) & psContextState);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateRenderContextKM: Failed to map firmware render context state (%u)",
			 eError));
		goto e3;
	}
	psContextState->uTAReg_VDM_CALL_STACK_POINTER =
	    sVDMCallStackAddr.uiAddr;

	RGXSetFirmwareAddress(&psFWRenderContext->s3DContext.psContextState, *ppsFWRenderContextStateMemDesc, offsetof(RGXFWIF_CTX_STATE, s3DContextState),	/* 3D state */
			      RFW_FWADDR_FLAG_NONE);

	/*
	 * Dump the Render and the memory contexts
	 */
	PDUMPCOMMENT("Dump FWRenderContext");
	DevmemPDumpLoadMem(*ppsFWRenderContextMemDesc, 0,
			   sizeof(*psFWRenderContext), PDUMP_FLAGS_CONTINUOUS);

	/*
	 * Dump the FW TA/3D context suspend state buffer
	 */
	PDUMPCOMMENT("Dump FWRenderContextState");
	DevmemPDumpLoadMem(*ppsFWRenderContextStateMemDesc, 0,
			   sizeof(RGXFWIF_CTX_STATE), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(*ppsFWRenderContextMemDesc);
	DevmemReleaseCpuVirtAddr(*ppsFWRenderContextStateMemDesc);

	return PVRSRV_OK;

 e3:
	DevmemFwFree(*ppsFWRenderContextStateMemDesc);
 e2:
	DevmemReleaseCpuVirtAddr(*ppsFWRenderContextMemDesc);
 e1:
	DevmemFwFree(*ppsFWRenderContextMemDesc);
 e0:
	OSFreeMem(psTmpCleanup);
	return eError;
}

/*
 * PVRSRVRGXDestroyRenderContextKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXDestroyRenderContextKM(RGX_RC_CLEANUP_DATA *
						 psCleanupData)
{
	PVRSRV_ERROR eError;
	PRGXFWIF_FWCOMMONCONTEXT psCommonContext;

	/* Request a flush out and cleanup for TA */
	RGXSetFirmwareAddress(&psCommonContext,
			      psCleanupData->psFWRenderContextMemDesc,
			      offsetof(RGXFWIF_FWRENDERCONTEXT, sTAContext),
			      RFW_FWADDR_NOREF_FLAG);
	RGXRequestMemoryContextCleanUp(psCleanupData->psDeviceNode,
				       psCommonContext, RGXFWIF_DM_TA);

	eError = RGXDeinitFWCommonContext(&psCleanupData->sFWTAContextCleanup);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXDestroyRenderContextKM : failed to deinit TA fw common ctx. Error:%u",
			 eError));
		goto e0;
	}

	/* Request a flush out and cleanup for 3D */
	RGXSetFirmwareAddress(&psCommonContext,
			      psCleanupData->psFWRenderContextMemDesc,
			      offsetof(RGXFWIF_FWRENDERCONTEXT, s3DContext),
			      RFW_FWADDR_NOREF_FLAG);
	RGXRequestMemoryContextCleanUp(psCleanupData->psDeviceNode,
				       psCommonContext, RGXFWIF_DM_3D);

	eError = RGXDeinitFWCommonContext(&psCleanupData->sFW3DContextCleanup);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXDestroyRenderContextKM : failed to deinit 3D fw common ctx. Error:%u",
			 eError));
		goto e0;
	}

	/*
	 * Unmap the TA/3D context state buffer pointers
	 */
	RGXUnsetFirmwareAddress(psCleanupData->psFWRenderContextStateMemDesc);	/* TA state */
	RGXUnsetFirmwareAddress(psCleanupData->psFWRenderContextStateMemDesc);	/* 3D state */

	/*
	 * Free the firmware TA/3D context state buffer
	 * (this is shared between the TA and 3D so only one free)
	 */
	DevmemFwFree(psCleanupData->psFWRenderContextStateMemDesc);

	/*
	 * Free the firmware render context.
	 */
	DevmemFwFree(psCleanupData->psFWRenderContextMemDesc);

	OSFreeMem(psCleanupData);

 e0:
	return eError;
}

/*
 * PVRSRVRGXKickTA3DKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXKickTA3DKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				     DEVMEM_MEMDESC * psFWRenderContextMemDesc,
				     IMG_BOOL bLastTAInScene,
				     IMG_BOOL bKickTA,
				     IMG_BOOL bKick3D,
				     IMG_UINT32 ui32TAcCCBWoffUpdate,
				     IMG_UINT32 ui323DcCCBWoffUpdate)
{
	PVRSRV_ERROR eError;
	RGXFWIF_KCCB_CMD sTACCBCmd;
	RGXFWIF_KCCB_CMD s3DCCBCmd;

	if (bKickTA) {
		/*
		 * Construct the kernel TA CCB command.
		 * (Safe to release reference to render context virtual address because
		 * render context destruction must flush the firmware).
		 */
		sTACCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
		RGXSetFirmwareAddress(&sTACCBCmd.uCmdData.sCmdKickData.
				      psContext, psFWRenderContextMemDesc,
				      offsetof(RGXFWIF_FWRENDERCONTEXT,
					       sTAContext),
				      RFW_FWADDR_NOREF_FLAG);
		sTACCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate =
		    ui32TAcCCBWoffUpdate;

		/*
		 * Submit the TA command to the firmware.
		 */
		eError = RGXScheduleCommand(psDeviceNode->pvDevice,
					    RGXFWIF_DM_TA,
					    &sTACCBCmd,
					    sizeof(sTACCBCmd), IMG_FALSE);

		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "PVRSRVRGXKickTA3DKM failed to schedule kernel TA command. Error:%u",
				 eError));
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
	}

	if (bKick3D) {
		/*
		 * Construct the kernel 3D CCB command.
		 * (Safe to release reference to render context virtual address because
		 * render context destruction must flush the firmware).
		 */
		s3DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
		RGXSetFirmwareAddress(&s3DCCBCmd.uCmdData.sCmdKickData.
				      psContext, psFWRenderContextMemDesc,
				      offsetof(RGXFWIF_FWRENDERCONTEXT,
					       s3DContext),
				      RFW_FWADDR_NOREF_FLAG);
		s3DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate =
		    ui323DcCCBWoffUpdate;

		/*
		 * Submit the 3D command to the firmware.
		 */
		eError = RGXScheduleCommand(psDeviceNode->pvDevice,
					    RGXFWIF_DM_3D,
					    &s3DCCBCmd,
					    sizeof(s3DCCBCmd), IMG_FALSE);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "PVRSRVRGXKickTA3DKM failed to schedule kernel 3D command. Error:%u",
				 eError));
			goto PVRSRVRGXKickTA3DKM_Exit;
		}
	}

 PVRSRVRGXKickTA3DKM_Exit:
	return eError;
}

/******************************************************************************
 End of file (rgxta3d.c)
******************************************************************************/
