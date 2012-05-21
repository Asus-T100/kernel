									    /*************************************************************************//*!
									       @File
									       @Title          RGX Compute routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX Compute routines
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "pdump_km.h"
#include "pvr_debug.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgxcompute.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "osfunc.h"
#include "sync_internal.h"

IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXCreateComputeContextKM(PVRSRV_DEVICE_NODE *
						 psDeviceNode,
						 DEVMEM_MEMDESC *
						 psCmpCCBMemDesc,
						 DEVMEM_MEMDESC *
						 psCmpCCBCtlMemDesc,
						 RGX_CC_CLEANUP_DATA **
						 ppsCleanupData,
						 DEVMEM_MEMDESC **
						 ppsFWComputeContextMemDesc,
						 IMG_UINT32 ui32Priority,
						 IMG_DEV_VIRTADDR sMCUFenceAddr,
						 IMG_HANDLE hMemCtxPrivData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_FWCOMMONCONTEXT *psFWComputeContext;
	RGX_CC_CLEANUP_DATA *psTmpCleanup;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/*
	   Allocate device memory for the firmware compute context.
	 */
	PDUMPCOMMENT("Allocate RGX firmware compute context");

	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(*psFWComputeContext),
				  RGX_FWCOMCTX_ALLOCFLAGS,
				  ppsFWComputeContextMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateComputeContextKM: Failed to allocate firmware compute context (0x%x)",
			 eError));
		goto e0;
	}
	psTmpCleanup->psFWComputeContextMemDesc = *ppsFWComputeContextMemDesc;
	psTmpCleanup->psDeviceNode = psDeviceNode;

	/*
	   Temporarily map the firmware compute context to the kernel.
	 */
	eError = DevmemAcquireCpuVirtAddr(*ppsFWComputeContextMemDesc,
					  (IMG_VOID **) & psFWComputeContext);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateComputeContextKM: Failed to map firmware compute context (0x%x)",
			 eError));
		goto e1;
	}

	eError = RGXInitFWCommonContext(psFWComputeContext,
					psCmpCCBMemDesc,
					psCmpCCBCtlMemDesc,
					hMemCtxPrivData,
					ui32Priority,
					&sMCUFenceAddr,
					&psTmpCleanup->sFWComContextCleanup);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateComputeContextKM: Failed to init firmware common context (%u)",
			 eError));
		goto e2;
	}

	/*
	 * Dump the Compute and the memory contexts
	 */
	PDUMPCOMMENT("Dump FWComputeContext");
	DevmemPDumpLoadMem(*ppsFWComputeContextMemDesc, 0,
			   sizeof(*psFWComputeContext), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(*ppsFWComputeContextMemDesc);

	return PVRSRV_OK;
 e2:
	DevmemReleaseCpuVirtAddr(*ppsFWComputeContextMemDesc);
 e1:
	DevmemFwFree(*ppsFWComputeContextMemDesc);
 e0:
	OSFreeMem(psTmpCleanup);
	return eError;
}

IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXDestroyComputeContextKM(RGX_CC_CLEANUP_DATA *
						  psCleanupData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PRGXFWIF_FWCOMMONCONTEXT psFWComContextFWAddr;

	RGXSetFirmwareAddress(&psFWComContextFWAddr,
			      psCleanupData->psFWComputeContextMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	RGXRequestMemoryContextCleanUp(psCleanupData->psDeviceNode,
				       psFWComContextFWAddr, RGXFWIF_DM_CDM);

	eError = RGXDeinitFWCommonContext(&psCleanupData->sFWComContextCleanup);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXDestroyComputeContextKM : failed to deinit fw common ctx. Error:%u",
			 eError));
		goto e0;
	}

	/*
	 * Free the firmware common context.
	 */
	DevmemFwFree(psCleanupData->psFWComputeContextMemDesc);

	OSFreeMem(psCleanupData);

 e0:
	return eError;
}

IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXKickCDMKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				    DEVMEM_MEMDESC * psFWComputeContextMemDesc,
				    IMG_UINT32 ui32cCCBWoffUpdate)
{
	PVRSRV_ERROR eError;
	RGXFWIF_KCCB_CMD sCmpKCCBCmd;

	/*
	 * Construct the kernel compute CCB command.
	 * (Safe to release reference to compute context virtual address because
	 * render context destruction must flush the firmware).
	 */
	sCmpKCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	RGXSetFirmwareAddress(&sCmpKCCBCmd.uCmdData.sCmdKickData.psContext,
			      psFWComputeContextMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);
	sCmpKCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate = ui32cCCBWoffUpdate;

	/*
	 * Submit the compute command to the firmware.
	 */
	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    RGXFWIF_DM_CDM,
				    &sCmpKCCBCmd,
				    sizeof(sCmpKCCBCmd), IMG_FALSE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXKickCDMKM failed to schedule kernel CCB command. (0x%x)",
			 eError));
		goto PVRSRVRGXKickCDMKM_Exit;
	}

 PVRSRVRGXKickCDMKM_Exit:
	return eError;
}

IMG_EXPORT PVRSRV_ERROR PVRSRVRGXFlushComputeDataKM(PVRSRV_DEVICE_NODE *
						    psDeviceNode,
						    DEVMEM_MEMDESC *
						    psFWContextMemDesc)
{
	RGXFWIF_KCCB_CMD sFlushCmd;
	PVRSRV_ERROR eError = PVRSRV_OK;

#if defined(PDUMP)
	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Submit Compute flush");
#endif
	sFlushCmd.eCmdType = RGXFWIF_KCCB_CMD_SLCFLUSHINVAL;
	sFlushCmd.uCmdData.sSLCFlushInvalData.bInval = IMG_FALSE;
	sFlushCmd.uCmdData.sSLCFlushInvalData.eDM = RGXFWIF_DM_CDM;

	RGXSetFirmwareAddress(&sFlushCmd.uCmdData.sSLCFlushInvalData.psContext,
			      psFWContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	eError = _RGXScheduleCommand(psDeviceNode->pvDevice,
				     RGXFWIF_DM_GP,
				     &sFlushCmd, sizeof(sFlushCmd), IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXFlushComputeDataKM: Failed to schedule SLC flush command with error (%u)",
			 eError));
	} else {
		/* Wait for the SLC flush to complete */
		eError =
		    RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP,
				   IMG_TRUE);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "PVRSRVRGXFlushComputeDataKM: Compute flush aborted with error (%u)",
				 eError));
		}
	}
	return eError;
}

/******************************************************************************
 End of file (rgxcompute.c)
******************************************************************************/
