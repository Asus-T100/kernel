									    /*************************************************************************//*!
									       @File
									       @Title          Device specific transfer queue routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Device specific functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "pdump_km.h"
#include "rgxdevice.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "rgxtransfer.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "osfunc.h"
#include "pvr_debug.h"
#include "pvrsrv.h"

IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXCreateTQ2DContextKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					      DEVMEM_MEMDESC * psTQ2DCCBMemDesc,
					      DEVMEM_MEMDESC *
					      psTQ2DCCBCtlMemDesc,
					      RGX_TQ2D_CLEANUP_DATA **
					      ppsCleanupData,
					      DEVMEM_MEMDESC **
					      ppsFWTQ2DContextMemDesc,
					      IMG_UINT32 ui32Priority,
					      IMG_HANDLE hMemCtxPrivData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_FWCOMMONCONTEXT *psFWTQ2DContext;
	RGX_TQ2D_CLEANUP_DATA *psTmpCleanup;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/*
	   Allocate device memory for the firmware TQ 2D context.
	 */
	PDUMPCOMMENT("Allocate RGX firmware TQ 2D context");

	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(*psFWTQ2DContext),
				  RGX_FWCOMCTX_ALLOCFLAGS,
				  ppsFWTQ2DContextMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateTQ2DContextKM: Failed to allocate firmware TQ 2D context (%u)",
			 eError));
		goto e0;
	}
	psTmpCleanup->psFWTQ2DContextMemDesc = *ppsFWTQ2DContextMemDesc;
	psTmpCleanup->psDeviceNode = psDeviceNode;

	/*
	   Temporarily map the firmware TQ 2D context to the kernel.
	 */
	eError = DevmemAcquireCpuVirtAddr(*ppsFWTQ2DContextMemDesc,
					  (IMG_VOID **) & psFWTQ2DContext);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateTQ2DContextKM: Failed to map firmware TQ 2D context (%u)",
			 eError));
		goto e1;
	}

	eError = RGXInitFWCommonContext(psFWTQ2DContext,
					psTQ2DCCBMemDesc,
					psTQ2DCCBCtlMemDesc,
					hMemCtxPrivData,
					ui32Priority,
					IMG_NULL,
					&psTmpCleanup->sFWComContextCleanup);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateTQ2DContextKM: Failed to init firmware common context (%u)",
			 eError));
		goto e2;
	}

	/*
	 * Dump the TQ2D and the memory contexts
	 */
	PDUMPCOMMENT("Dump FWTQ2DContext");
	DevmemPDumpLoadMem(*ppsFWTQ2DContextMemDesc, 0,
			   sizeof(*psFWTQ2DContext), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(*ppsFWTQ2DContextMemDesc);

	return PVRSRV_OK;
 e2:
	DevmemReleaseCpuVirtAddr(*ppsFWTQ2DContextMemDesc);
 e1:
	DevmemFwFree(*ppsFWTQ2DContextMemDesc);
 e0:
	OSFreeMem(psTmpCleanup);
	return eError;
}

/*
 * PVRSRVRGXDestroyTQ2DContextKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXDestroyTQ2DContextKM(RGX_TQ2D_CLEANUP_DATA *
					       psCleanupData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PRGXFWIF_FWCOMMONCONTEXT psFWComContextFWAddr;

	RGXSetFirmwareAddress(&psFWComContextFWAddr,
			      psCleanupData->psFWTQ2DContextMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	RGXRequestMemoryContextCleanUp(psCleanupData->psDeviceNode,
				       psFWComContextFWAddr, RGXFWIF_DM_2D);

	eError = RGXDeinitFWCommonContext(&psCleanupData->sFWComContextCleanup);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXDestroyTQ2DContextKM : failed to deinit fw common ctx. Error:%u",
			 eError));
		goto e0;
	}

	/*
	 * Free the firmware common context.
	 */
	DevmemFwFree(psCleanupData->psFWTQ2DContextMemDesc);

	OSFreeMem(psCleanupData);

 e0:
	return eError;
}

/*
 * PVRSRVRGXCreateTQ3DContextKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXCreateTQ3DContextKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					      DEVMEM_MEMDESC * psTQ3DCCBMemDesc,
					      DEVMEM_MEMDESC *
					      psTQ3DCCBCtlMemDesc,
					      RGX_TQ3D_CLEANUP_DATA **
					      ppsCleanupData,
					      DEVMEM_MEMDESC **
					      ppsFWTQ3DContextMemDesc,
					      DEVMEM_MEMDESC **
					      ppsFWTQ3DContextStateMemDesc,
					      IMG_UINT32 ui32Priority,
					      IMG_DEV_VIRTADDR sMCUFenceAddr,
					      IMG_HANDLE hMemCtxPrivData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;
	RGXFWIF_FWCOMMONCONTEXT *psFWTQ3DContext;
	RGX_TQ3D_CLEANUP_DATA *psTmpCleanup;

	/* Prepare cleanup struct */
	psTmpCleanup = OSAllocMem(sizeof(*psTmpCleanup));
	if (psTmpCleanup == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	OSMemSet(psTmpCleanup, 0, sizeof(*psTmpCleanup));
	*ppsCleanupData = psTmpCleanup;

	/*
	   Allocate device memory for the firmware TQ 3D context.
	 */
	PDUMPCOMMENT("Allocate RGX firmware TQ 3D context");

	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(*psFWTQ3DContext),
				  RGX_FWCOMCTX_ALLOCFLAGS,
				  ppsFWTQ3DContextMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateTQ3DContextKM: Failed to allocate firmware TQ 3D context (%u)",
			 eError));
		goto e0;
	}
	psTmpCleanup->psFWTQ3DContextMemDesc = *ppsFWTQ3DContextMemDesc;
	psTmpCleanup->psDeviceNode = psDeviceNode;

	/*
	   Temporarily map the firmware TQ 3D context to the kernel.
	 */
	eError = DevmemAcquireCpuVirtAddr(*ppsFWTQ3DContextMemDesc,
					  (IMG_VOID **) & psFWTQ3DContext);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateTQ3DContextKM: Failed to map firmware TQ 3D context (%u)",
			 eError));
		goto e1;
	}

	/*
	   Allocate device memory for the firmware GPU context suspend state.
	   Note: the FW reads/writes the state to memory by accessing the GPU register interface.
	 */
	PDUMPCOMMENT("Allocate RGX firmware TQ/3D context suspend state");

	eError = DevmemFwAllocate(psDevInfo,
				  sizeof(RGXFWIF_3DCTX_STATE),
				  RGX_FWCOMCTX_ALLOCFLAGS,
				  ppsFWTQ3DContextStateMemDesc);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateTQ3DContextKM: Failed to allocate firmware GPU context suspend state (%u)",
			 eError));
		goto e2;
	}
	psTmpCleanup->psFWTQ3DContextStateMemDesc =
	    *ppsFWTQ3DContextStateMemDesc;

	/* Init TQ/3D FW common context */
	eError = RGXInitFWCommonContext(psFWTQ3DContext,
					psTQ3DCCBMemDesc,
					psTQ3DCCBCtlMemDesc,
					hMemCtxPrivData,
					ui32Priority,
					&sMCUFenceAddr,
					&psTmpCleanup->sFWComContextCleanup);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXCreateTQ3DContextKM: Failed to init firmware common context (%u)",
			 eError));
		goto e3;
	}

	/*
	 * Set the firmware GPU context state buffer.
	 * 
	 * The common context stores a dword pointer (FW) so we can cast the generic buffer to
	 * the correct 3D (3D/TQ = normal 3D) state structure type in the FW.
	 */
	RGXSetFirmwareAddress(&psFWTQ3DContext->psContextState,
			      *ppsFWTQ3DContextStateMemDesc,
			      0, RFW_FWADDR_FLAG_NONE);

	/*
	 * Dump the TQ3D and the memory contexts
	 */
	PDUMPCOMMENT("Dump FWTQ3DContext");
	DevmemPDumpLoadMem(*ppsFWTQ3DContextMemDesc, 0,
			   sizeof(*psFWTQ3DContext), PDUMP_FLAGS_CONTINUOUS);

	/*
	 * Dump the FW TQ/3D context suspend state buffer
	 */
	PDUMPCOMMENT("Dump FWTQ3DContextState");
	DevmemPDumpLoadMem(*ppsFWTQ3DContextStateMemDesc, 0,
			   sizeof(RGXFWIF_3DCTX_STATE), PDUMP_FLAGS_CONTINUOUS);

	/* Release address acquired above. */
	DevmemReleaseCpuVirtAddr(*ppsFWTQ3DContextMemDesc);

	return PVRSRV_OK;

 e3:
	DevmemFwFree(*ppsFWTQ3DContextStateMemDesc);
 e2:
	DevmemReleaseCpuVirtAddr(*ppsFWTQ3DContextMemDesc);
 e1:
	DevmemFwFree(*ppsFWTQ3DContextMemDesc);
 e0:
	OSFreeMem(psTmpCleanup);
	return eError;
}

/*
 * PVRSRVRGXDestroyTQ3DContextKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVRGXDestroyTQ3DContextKM(RGX_TQ3D_CLEANUP_DATA *
					       psCleanupData)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	PRGXFWIF_FWCOMMONCONTEXT psFWComContextFWAddr;

	RGXSetFirmwareAddress(&psFWComContextFWAddr,
			      psCleanupData->psFWTQ3DContextMemDesc,
			      0, RFW_FWADDR_NOREF_FLAG);

	RGXRequestMemoryContextCleanUp(psCleanupData->psDeviceNode,
				       psFWComContextFWAddr, RGXFWIF_DM_3D);

	eError = RGXDeinitFWCommonContext(&psCleanupData->sFWComContextCleanup);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXDestroyTQ3DContextKM : failed to deinit fw common ctx. Error:%u",
			 eError));
		goto e0;
	}

	/*
	 * Unmap the TA/3D context state buffer pointers
	 */
	RGXUnsetFirmwareAddress(psCleanupData->psFWTQ3DContextStateMemDesc);

	/*
	 * Free the firmware TQ/3D context state buffer
	 */
	DevmemFwFree(psCleanupData->psFWTQ3DContextStateMemDesc);

	/*
	 * Free the firmware common context.
	 */
	DevmemFwFree(psCleanupData->psFWTQ3DContextMemDesc);

	OSFreeMem(psCleanupData);

 e0:
	return eError;
}

/*
 * PVRSRVSubmit2DKickKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVSubmitTQ2DKickKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					DEVMEM_MEMDESC * psFWTQ2DContextMemDesc,
					IMG_UINT32 ui32TQ2DcCCBWoffUpdate,
					IMG_BOOL bPDumpContinuous)
{
	RGXFWIF_KCCB_CMD s2DCCBCmd;
	PVRSRV_ERROR eError;

	/*
	 * Construct the kernel 2D CCB command.
	 */
	s2DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	RGXSetFirmwareAddress(&s2DCCBCmd.uCmdData.sCmdKickData.psContext,
			      psFWTQ2DContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);
	s2DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate =
	    ui32TQ2DcCCBWoffUpdate;

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    RGXFWIF_DM_2D,
				    &s2DCCBCmd,
				    sizeof(s2DCCBCmd), bPDumpContinuous);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVSubmitTQ2DKickKM : failed to schedule kernel 2D command. Error:%u",
			 eError));
		return eError;
	}
	return PVRSRV_OK;
}

/*
 * PVRSRVSubmitTQ3DKickKM
 */
IMG_EXPORT
    PVRSRV_ERROR PVRSRVSubmitTQ3DKickKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					DEVMEM_MEMDESC * psFWTQ3DContextMemDesc,
					IMG_UINT32 ui32TQ3DcCCBWoffUpdate,
					IMG_BOOL bPDumpContinuous)
{
	RGXFWIF_KCCB_CMD s3DCCBCmd;
	PVRSRV_ERROR eError;

	/*
	 * Construct the kernel 3D CCB command.
	 */
	s3DCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_KICK;
	RGXSetFirmwareAddress(&s3DCCBCmd.uCmdData.sCmdKickData.psContext,
			      psFWTQ3DContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);
	s3DCCBCmd.uCmdData.sCmdKickData.ui32CWoffUpdate =
	    ui32TQ3DcCCBWoffUpdate;

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    RGXFWIF_DM_3D,
				    &s3DCCBCmd,
				    sizeof(s3DCCBCmd), bPDumpContinuous);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVSubmitTQ3DKickKM : failed to schedule kernel TQ command. Error:%u",
			 eError));
		return eError;
	}
	return PVRSRV_OK;
}

									    /**************************************************************************//**
 End of file (rgxtransfer.c)
******************************************************************************/
