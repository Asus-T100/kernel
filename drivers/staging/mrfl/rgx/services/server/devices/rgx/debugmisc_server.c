									    /*************************************************************************//*!
									       @File
									       @Title          Debugging and miscellaneous functions server implementation
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Kernel services functions for debugging and other
									       miscellaneous functionality.
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "pvrsrv.h"
#include "pvr_debug.h"
#include "debugmisc_server.h"
#include "rgxfwutils.h"
#include "mm.h"
#include "pdump_km.h"

IMG_EXPORT PVRSRV_ERROR
PVRSRVDebugMiscTilingSetStateKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				IMG_HANDLE hMemCtxPrivData, IMG_BOOL bEnabled)
{
	PVRSRV_ERROR eError = PVRSRV_OK;
	RGXFWIF_KCCB_CMD sSTSCCBCmd;
	DEVMEM_MEMDESC *psFWMemContextMemDesc = hMemCtxPrivData;

	PVR_DPF((PVR_DBG_ERROR, "PVRSRVDebugMiscTilingSetStateKM: enter."));

	sSTSCCBCmd.eCmdType = RGXFWIF_KCCB_CMD_TILINGCTL;

	RGXSetFirmwareAddress(&sSTSCCBCmd.uCmdData.sTileData.psMemContext,
			      psFWMemContextMemDesc, 0, RFW_FWADDR_NOREF_FLAG);
	sSTSCCBCmd.uCmdData.sTileData.bEnabled = bEnabled;

	PVR_DPF((PVR_DBG_ERROR,
		 "PVRSRVDebugMiscTilingSetStateKM: scheduling CCB command."));

	eError = RGXScheduleCommand(psDeviceNode->pvDevice,
				    RGXFWIF_DM_GP,
				    &sSTSCCBCmd, sizeof(sSTSCCBCmd), IMG_FALSE);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVDebugMiscTilingSetStateKM: RGXScheduleCommand failed."
			 " Error:%u", eError));
	} else {
		/* Wait for the command to complete */
		eError =
		    RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP,
				   IMG_FALSE);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "PVRSRVDebugMiscSLCSetEnableStateKM: Waiting for value aborted with error (%u)",
				 eError));
		}
	}

	PVR_DPF((PVR_DBG_ERROR, "PVRSRVDebugMiscTilingSetStateKM: exit."));
	return eError;
}

IMG_EXPORT PVRSRV_ERROR
PVRSRVDebugMiscSLCSetBypassStateKM(PVRSRV_DEVICE_NODE * psDeviceNode,
				   IMG_UINT32 uiFlags, IMG_BOOL bSetBypassed)
{
	RGXFWIF_KCCB_CMD sSLCBPCtlCmd;
	PVRSRV_ERROR eError = PVRSRV_OK;

	sSLCBPCtlCmd.eCmdType = RGXFWIF_KCCB_CMD_SLCBPCTL;
	sSLCBPCtlCmd.uCmdData.sSLCBPCtlData.bSetBypassed = bSetBypassed;
	sSLCBPCtlCmd.uCmdData.sSLCBPCtlData.uiFlags = uiFlags;

	if ((eError = RGXScheduleCommand(psDeviceNode->pvDevice,
					 RGXFWIF_DM_GP,
					 &sSLCBPCtlCmd,
					 sizeof(sSLCBPCtlCmd), IMG_FALSE))) {
		PVR_DPF((PVR_DBG_ERROR,
			 "PVRSRVDebugMiscSLCSetEnableStateKM: RGXScheduleCommandfailed. Error:%u",
			 eError));
	} else {
		/* Wait for the SLC flush to complete */
		eError =
		    RGXWaitForFWOp(psDeviceNode->pvDevice, RGXFWIF_DM_GP,
				   IMG_FALSE);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "PVRSRVDebugMiscSLCSetEnableStateKM: Waiting for value aborted with error (%u)",
				 eError));
		}
	}

	return PVRSRV_OK;
}

IMG_EXPORT PVRSRV_ERROR
PVRSRVRGXDebugMiscSetFWLogKM(PVRSRV_DEVICE_NODE * psDeviceNode,
			     IMG_UINT32 ui32RGXFWLogType)
{
	RGXFWIF_LOG_TYPE eLogType;
	PVRSRV_RGXDEV_INFO *psDevInfo = psDeviceNode->pvDevice;

	/* check log type is valid */
	if (ui32RGXFWLogType >= RGXFWIF_LOG_TYPE_NUM) {
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	eLogType = (RGXFWIF_LOG_TYPE) ui32RGXFWLogType;

	/* set the new log type */
	psDevInfo->psRGXFWIfTraceBuf->eLogType = eLogType;

	return PVRSRV_OK;

}
