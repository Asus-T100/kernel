									    /*************************************************************************//*!
									       @File
									       @Title          Device specific utility routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Device specific functions
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>

#include "rgxdefs.h"
#include "rgx_fwif_km.h"
#include "pdump_km.h"
#include "osfunc.h"
#include "allocmem.h"
#include "pvr_debug.h"
#include "rgxutils.h"
#include "power.h"
#include "pvrsrv.h"
#include "sync_internal.h"
#include "rgxfwutils.h"

IMG_VOID RGXScheduleProcessQueuesKM(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle)
{
	/* FIXME implement this function */
	PVRSRV_DEVICE_NODE *psDeviceNode =
	    (PVRSRV_DEVICE_NODE *) hCmdCompHandle;

	PVR_UNREFERENCED_PARAMETER(psDeviceNode);
/* 	PVRSRV_ERROR 		eError; */
/* 	PVRSRV_RGXDEV_INFO 	*psDevInfo = psDeviceNode->pvDevice; */
/* 	RGXMKIF_HOST_CTL	*psHostCtl = psDevInfo->psKernelRGXHostCtlMemInfo->pvLinAddrKM; */
/* 	IMG_UINT32		ui32PowerStatus; */
/* 	RGXMKIF_COMMAND		sCommand = {0}; */

/* 	ui32PowerStatus = psHostCtl->ui32PowerStatus; */
/* 	if ((ui32PowerStatus & PVRSRV_USSE_EDM_POWMAN_NO_WORK) != 0) */
/* 	{ */
/* 		/\* The firwmare has no work to do so don't waste power. *\/ */
/* 		return; */
/* 	} */

/* 	eError = RGXScheduleCCBCommandKM(psDeviceNode, RGXMKIF_CMD_PROCESS_QUEUES, &sCommand, ISR_ID, 0); */
/* 	if (eError != PVRSRV_OK) */
/* 	{ */
/* 		PVR_DPF((PVR_DBG_ERROR,"RGXScheduleProcessQueuesKM failed to schedule CCB command: %u", eError)); */
/* 	} */
}

IMG_BOOL RGXIsDevicePowered(PVRSRV_DEVICE_NODE * psDeviceNode)
{
	return PVRSRVIsDevicePowered(psDeviceNode->sDevId.ui32DeviceIndex);
}

/*
 * RGXRunScript
 */
PVRSRV_ERROR RGXRunScript(PVRSRV_RGXDEV_INFO * psDevInfo,
			  RGX_INIT_COMMAND * psScript,
			  IMG_UINT32 ui32NumCommands, IMG_UINT32 ui32PdumpFlags)
{
	IMG_UINT32 ui32PC;
	RGX_INIT_COMMAND *psComm;
#if !defined(NO_HARDWARE)
	IMG_UINT32 ui32LastLoopPoint = 0xFFFFFFFF;
#endif				/* NO_HARDWARE */

	for (ui32PC = 0, psComm = psScript;
	     ui32PC < ui32NumCommands; ui32PC++, psComm++) {
		switch (psComm->eOp) {
		case RGX_INIT_OP_DBG_READ32_HW_REG:
			{
				IMG_UINT32 ui32RegVal;
				ui32RegVal =
				    OSReadHWReg32(psDevInfo->pvRegsBaseKM,
						  psComm->sDBGReadHWReg.
						  ui32Offset);
				PVR_LOG(("%s: 0x%08X",
					 psComm->sDBGReadHWReg.aszName,
					 ui32RegVal));
				break;
			}
		case RGX_INIT_OP_DBG_READ64_HW_REG:
			{
				IMG_UINT64 ui64RegVal;
				ui64RegVal =
				    OSReadHWReg64(psDevInfo->pvRegsBaseKM,
						  psComm->sDBGReadHWReg.
						  ui32Offset);
				PVR_LOG(("%s: 0x%016llX",
					 psComm->sDBGReadHWReg.aszName,
					 ui64RegVal));
				break;
			}
		case RGX_INIT_OP_WRITE_HW_REG:
			{
				OSWriteHWReg32(psDevInfo->pvRegsBaseKM,
					       psComm->sWriteHWReg.ui32Offset,
					       psComm->sWriteHWReg.ui32Value);
				PDUMPCOMMENT
				    ("RGXRunScript: Write HW reg operation");
				PDUMPREG32(RGX_PDUMPREG_NAME,
					   psComm->sWriteHWReg.ui32Offset,
					   psComm->sWriteHWReg.ui32Value,
					   ui32PdumpFlags);
				break;
			}
#if defined(PDUMP)
		case RGX_INIT_OP_PDUMP_HW_REG:
			{
				PDUMPCOMMENT
				    ("RGXRunScript: Dump HW reg operation");
				PDUMPREG32(RGX_PDUMPREG_NAME,
					   psComm->sPDumpHWReg.ui32Offset,
					   psComm->sPDumpHWReg.ui32Value,
					   ui32PdumpFlags);
				break;
			}
#endif
		case RGX_INIT_OP_POLL_HW_REG:
			{
				if (PVRSRVPollForValueKM
				    ((IMG_UINT32 *) ((IMG_UINT8 *) psDevInfo->
						     pvRegsBaseKM +
						     psComm->sPollHWReg.
						     ui32Offset),
				     psComm->sPollHWReg.ui32Value,
				     psComm->sPollHWReg.ui32Mask) !=
				    PVRSRV_OK) {
					PVR_DPF((PVR_DBG_ERROR,
						 "RGXRunScript: Poll for Reg (0x%x) failed -> Cancel script.",
						 psComm->sPollHWReg.
						 ui32Offset));
					return PVRSRV_ERROR_TIMEOUT;
				}

				PDUMPREGPOL(RGX_PDUMPREG_NAME,
					    psComm->sPollHWReg.ui32Offset,
					    psComm->sPollHWReg.ui32Value,
					    psComm->sPollHWReg.ui32Mask,
					    ui32PdumpFlags,
					    PDUMP_POLL_OPERATOR_EQUAL);

				break;
			}

		case RGX_INIT_OP_LOOP_POINT:
			{
#if !defined(NO_HARDWARE)
				ui32LastLoopPoint = ui32PC;
#endif				/* NO_HARDWARE */
				break;
			}

		case RGX_INIT_OP_COND_BRANCH:
			{
#if !defined(NO_HARDWARE)
				IMG_UINT32 ui32RegVal =
				    OSReadHWReg32(psDevInfo->pvRegsBaseKM,
						  psComm->
						  sConditionalBranchPoint.
						  ui32Offset);

				if ((ui32RegVal & psComm->
				     sConditionalBranchPoint.ui32Mask) !=
				    psComm->sConditionalBranchPoint.ui32Value) {
					ui32PC = ui32LastLoopPoint - 1;
				}
#endif				/* NO_HARDWARE */

				PDUMPIDLWITHFLAGS(30, ui32PdumpFlags);
				break;
			}

		case RGX_INIT_OP_HALT:
			{
				return PVRSRV_OK;
			}
		case RGX_INIT_OP_ILLEGAL:
			/* FALLTHROUGH */
		default:
			{
				PVR_DPF((PVR_DBG_ERROR,
					 "RGXRunScript: PC %d: Illegal command: %d",
					 ui32PC, psComm->eOp));
				return PVRSRV_ERROR_UNKNOWN_SCRIPT_OPERATION;
			}
		}

	}

	return PVRSRV_ERROR_UNKNOWN_SCRIPT_OPERATION;
}

/*
 * RGXRequestMemoryContextCleanUp
 */

PVRSRV_ERROR RGXRequestMemoryContextCleanUp(PVRSRV_DEVICE_NODE * psDeviceNode,
					    PRGXFWIF_FWCOMMONCONTEXT
					    psFWComContextFWAddr,
					    RGXFWIF_DM eDM)
{
	RGXFWIF_KCCB_CMD sRCCleanUpCmd;
	PVRSRV_ERROR eError;

	PDUMPCOMMENT("Mem ctx cleanup Request DM%d [context = 0x%08x]", eDM,
		     psFWComContextFWAddr.ui32Addr);

	sRCCleanUpCmd.eCmdType = RGXFWIF_KCCB_CMD_MC_CLEANUP;
	sRCCleanUpCmd.uCmdData.sMCCleanupData.psContext = psFWComContextFWAddr;
	eError = RGXScheduleCommandAndWait(psDeviceNode->pvDevice,
					   eDM,
					   &sRCCleanUpCmd,
					   sizeof(RGXFWIF_KCCB_CMD),
					   &sRCCleanUpCmd.uCmdData.
					   sMCCleanupData.uiSyncObjDevVAddr,
					   &sRCCleanUpCmd.uCmdData.
					   sMCCleanupData.uiUpdateVal,
					   IMG_FALSE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXScheduleMemoryContextCleanUp: Failed to schedule a memory context cleanup with error (%u)",
			 eError));
	}

	return eError;
}

/*
 * RGXRequestHWRTDataCleanUp
 */

PVRSRV_ERROR RGXRequestHWRTDataCleanUp(PVRSRV_DEVICE_NODE * psDeviceNode,
				       PRGXFWIF_HWRTDATA psHWRTData,
				       RGXFWIF_DM eDM)
{
	RGXFWIF_KCCB_CMD sRCCleanUpCmd;
	PVRSRV_ERROR eError;

	PDUMPCOMMENT("HWRTData cleanup Request DM%d [psHWRTData = 0x%08x]", eDM,
		     psHWRTData.ui32Addr);

	sRCCleanUpCmd.eCmdType = RGXFWIF_KCCB_CMD_HWRTDATA_CLEANUP;
	sRCCleanUpCmd.uCmdData.sHWRTDataCleanupData.psHWRTData = psHWRTData;
	eError = RGXScheduleCommandAndWait(psDeviceNode->pvDevice,
					   eDM,
					   &sRCCleanUpCmd,
					   sizeof(RGXFWIF_KCCB_CMD),
					   &sRCCleanUpCmd.uCmdData.
					   sHWRTDataCleanupData.
					   uiSyncObjDevVAddr,
					   &sRCCleanUpCmd.uCmdData.
					   sHWRTDataCleanupData.uiUpdateVal,
					   IMG_FALSE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "RGXRequestHWRTDataCleanUp: Failed to schedule a HWRTData cleanup with error (%u)",
			 eError));
	}

	return eError;
}

/******************************************************************************
 End of file (rgxutils.c)
******************************************************************************/
