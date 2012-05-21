									    /*************************************************************************//*!
									       @File
									       @Title          RGX Breakpoint routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    RGX Breakpoint routines
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "rgxbreakpoint.h"
#include "pvr_debug.h"
#include "rgxutils.h"
#include "rgxfwutils.h"
#include "device.h"
#include "sync_internal.h"
#include "pdump_km.h"
#include "pvrsrv.h"

PVRSRV_ERROR PVRSRVRGXSetBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				      IMG_HANDLE hMemCtxPrivData,
				      RGXFWIF_DM eFWDataMaster,
				      IMG_UINT32 ui32BPAddr,
				      IMG_UINT32 ui32HandlerAddr,
				      IMG_UINT32 ui32DataMaster)
{
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hMemCtxPrivData;
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sBPCmd;

	if (psDeviceNode->psDevConfig->bBPSet == IMG_TRUE)
		return PVRSRV_ERROR_BP_ALREADY_SET;

	sBPCmd.eCmdType = RGXFWIF_KCCB_CMD_BP;
	sBPCmd.uCmdData.sBPData.ui32BPAddr = ui32BPAddr;
	sBPCmd.uCmdData.sBPData.ui32HandlerAddr = ui32HandlerAddr;
	sBPCmd.uCmdData.sBPData.ui32BPDM = ui32DataMaster;
	sBPCmd.uCmdData.sBPData.bEnable = IMG_TRUE;
	sBPCmd.uCmdData.sBPData.ui32Flags = RGXFWIF_BPDATA_FLAGS_WRITE;

	RGXSetFirmwareAddress(&sBPCmd.uCmdData.sBPData.psFWMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    eFWDataMaster,
				    &sBPCmd, sizeof(sBPCmd), IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXSetBreakpointKM: RGXScheduleCommand failed. Error:%u",
			 eError));
		return eError;
	}

	/* Wait for FW to complete */
	eError =
	    RGXWaitForFWOp(psDeviceNode->pvDevice, eFWDataMaster, IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXSetBreakpointKM: Wait for completion aborted with error (%u)",
			 eError));
		return eError;
	}

	psDeviceNode->psDevConfig->eBPDM = eFWDataMaster;
	psDeviceNode->psDevConfig->bBPSet = IMG_TRUE;

	return eError;
}

PVRSRV_ERROR PVRSRVRGXClearBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					IMG_HANDLE hMemCtxPrivData)
{
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hMemCtxPrivData;
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sBPCmd;
	RGXFWIF_DM eDataMaster = psDeviceNode->psDevConfig->eBPDM;

	sBPCmd.eCmdType = RGXFWIF_KCCB_CMD_BP;
	sBPCmd.uCmdData.sBPData.ui32BPAddr = 0;
	sBPCmd.uCmdData.sBPData.ui32HandlerAddr = 0;
	sBPCmd.uCmdData.sBPData.bEnable = IMG_FALSE;
	sBPCmd.uCmdData.sBPData.ui32Flags =
	    RGXFWIF_BPDATA_FLAGS_WRITE | RGXFWIF_BPDATA_FLAGS_CTL;

	RGXSetFirmwareAddress(&sBPCmd.uCmdData.sBPData.psFWMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    eDataMaster,
				    &sBPCmd, sizeof(sBPCmd), IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXClearBreakpointKM: RGXScheduleCommand failed. Error:%u",
			 eError));
		return eError;
	}

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, eDataMaster, IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXClearBreakpointKM: Wait for completion aborted with error (%u)",
			 eError));
		return eError;
	}

	psDeviceNode->psDevConfig->bBPSet = IMG_FALSE;

	return eError;
}

PVRSRV_ERROR PVRSRVRGXEnableBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					 IMG_HANDLE hMemCtxPrivData)
{
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hMemCtxPrivData;
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sBPCmd;
	RGXFWIF_DM eDataMaster = psDeviceNode->psDevConfig->eBPDM;

	if (psDeviceNode->psDevConfig->bBPSet == IMG_FALSE)
		return PVRSRV_ERROR_BP_NOT_SET;

	sBPCmd.eCmdType = RGXFWIF_KCCB_CMD_BP;
	sBPCmd.uCmdData.sBPData.bEnable = IMG_TRUE;
	sBPCmd.uCmdData.sBPData.ui32Flags = RGXFWIF_BPDATA_FLAGS_CTL;

	RGXSetFirmwareAddress(&sBPCmd.uCmdData.sBPData.psFWMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    eDataMaster,
				    &sBPCmd, sizeof(sBPCmd), IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXEnableBreakpointKM: RGXScheduleCommand failed. Error:%u",
			 eError));
		return eError;
	}

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, eDataMaster, IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXEnableBreakpointKM: Wait for completion aborted with error (%u)",
			 eError));
		return eError;
	}

	return eError;
}

PVRSRV_ERROR PVRSRVRGXDisableBreakpointKM(PVRSRV_DEVICE_NODE * psDeviceNode,
					  IMG_HANDLE hMemCtxPrivData)
{
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hMemCtxPrivData;
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sBPCmd;
	RGXFWIF_DM eDataMaster = psDeviceNode->psDevConfig->eBPDM;

	if (psDeviceNode->psDevConfig->bBPSet == IMG_FALSE)
		return PVRSRV_ERROR_BP_NOT_SET;

	sBPCmd.eCmdType = RGXFWIF_KCCB_CMD_BP;
	sBPCmd.uCmdData.sBPData.bEnable = IMG_FALSE;
	sBPCmd.uCmdData.sBPData.ui32Flags = RGXFWIF_BPDATA_FLAGS_CTL;

	RGXSetFirmwareAddress(&sBPCmd.uCmdData.sBPData.psFWMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    eDataMaster,
				    &sBPCmd, sizeof(sBPCmd), IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXDisableBreakpointKM: RGXScheduleCommand failed. Error:%u",
			 eError));
		return eError;
	}

	/* Wait for FW to complete */
	eError = RGXWaitForFWOp(psDeviceNode->pvDevice, eDataMaster, IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXDisableBreakpointKM: Wait for completion aborted with error (%u)",
			 eError));
		return eError;
	}

	return eError;
}

PVRSRV_ERROR PVRSRVRGXOverallocateBPRegistersKM(PVRSRV_DEVICE_NODE *
						psDeviceNode,
						IMG_UINT32 ui32TempRegs,
						IMG_UINT32 ui32SharedRegs)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sBPCmd;

	sBPCmd.eCmdType = RGXFWIF_KCCB_CMD_BP;
	sBPCmd.uCmdData.sBPData.ui32Flags = RGXFWIF_BPDATA_FLAGS_REGS;
	sBPCmd.uCmdData.sBPData.ui32TempRegs = ui32TempRegs;
	sBPCmd.uCmdData.sBPData.ui32SharedRegs = ui32SharedRegs;

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    RGXFWIF_DM_GP,
				    &sBPCmd, sizeof(sBPCmd), IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXOverallocateBPRegistersKM: RGXScheduleCommand failed. Error:%u",
			 eError));
		return eError;
	}

	/* Wait for FW to complete */
	eError =
	    RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP, IMG_TRUE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVRGXOverallocateBPRegistersKM: Wait for completion aborted with error (%u)",
			 eError));
		return eError;
	}

	return eError;
}

/******************************************************************************
 End of file (rgxbreakpoint.c)
******************************************************************************/
