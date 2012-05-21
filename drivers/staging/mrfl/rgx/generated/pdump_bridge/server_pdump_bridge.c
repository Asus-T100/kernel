									    /*************************************************************************//*!
									       @File
									       @Title          Server bridge for pdump
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side of the bridge for pdump
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>
#include <asm/uaccess.h>

#include "img_defs.h"

#include "pdump_km.h"

#include "common_pdump_bridge.h"

#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#include "rgx_bridge.h"
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>

static IMG_INT
PVRSRVBridgePVRSRVPDumpIsCapturing(IMG_UINT32 ui32BridgeID,
				   PVRSRV_BRIDGE_IN_PVRSRVPDUMPISCAPTURING *
				   psPVRSRVPDumpIsCapturingIN,
				   PVRSRV_BRIDGE_OUT_PVRSRVPDUMPISCAPTURING *
				   psPVRSRVPDumpIsCapturingOUT,
				   CONNECTION_DATA * psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psPVRSRVPDumpIsCapturingIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPISCAPTURING);

	psPVRSRVPDumpIsCapturingOUT->eError =
	    PDumpIsCaptureFrameKM(&psPVRSRVPDumpIsCapturingOUT->bIsCapturing);

	return 0;
}

static IMG_INT
PVRSRVBridgePVRSRVPDumpComment(IMG_UINT32 ui32BridgeID,
			       PVRSRV_BRIDGE_IN_PVRSRVPDUMPCOMMENT *
			       psPVRSRVPDumpCommentIN,
			       PVRSRV_BRIDGE_OUT_PVRSRVPDUMPCOMMENT *
			       psPVRSRVPDumpCommentOUT,
			       CONNECTION_DATA * psConnection)
{
	IMG_CHAR *uiCommentInt = IMG_NULL;

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPCOMMENT);

	uiCommentInt =
	    kmalloc(PVRSRV_PDUMP_MAX_COMMENT_SIZE * sizeof(IMG_CHAR),
		    GFP_KERNEL);
	if (!uiCommentInt) {
		psPVRSRVPDumpCommentOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;

		goto PVRSRVPDumpComment_exit;
	}

	if (copy_from_user(uiCommentInt, psPVRSRVPDumpCommentIN->puiComment,
			   PVRSRV_PDUMP_MAX_COMMENT_SIZE * sizeof(IMG_CHAR)) !=
	    0) {
		psPVRSRVPDumpCommentOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

		goto PVRSRVPDumpComment_exit;
	}

	psPVRSRVPDumpCommentOUT->eError =
	    PDumpCommentKM(uiCommentInt, psPVRSRVPDumpCommentIN->ui32Flags);

 PVRSRVPDumpComment_exit:
	if (uiCommentInt)
		kfree(uiCommentInt);

	return 0;
}

static IMG_INT
PVRSRVBridgePVRSRVPDumpSetFrame(IMG_UINT32 ui32BridgeID,
				PVRSRV_BRIDGE_IN_PVRSRVPDUMPSETFRAME *
				psPVRSRVPDumpSetFrameIN,
				PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSETFRAME *
				psPVRSRVPDumpSetFrameOUT,
				CONNECTION_DATA * psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSETFRAME);

	psPVRSRVPDumpSetFrameOUT->eError =
	    PDumpSetFrameKM(psPVRSRVPDumpSetFrameIN->ui32Frame);

	return 0;
}

static IMG_INT
PVRSRVBridgePVRSRVPDumpIsLastCaptureFrame(IMG_UINT32 ui32BridgeID,
					  PVRSRV_BRIDGE_IN_PVRSRVPDUMPISLASTCAPTUREFRAME
					  * psPVRSRVPDumpIsLastCaptureFrameIN,
					  PVRSRV_BRIDGE_OUT_PVRSRVPDUMPISLASTCAPTUREFRAME
					  * psPVRSRVPDumpIsLastCaptureFrameOUT,
					  CONNECTION_DATA * psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psPVRSRVPDumpIsLastCaptureFrameIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPISLASTCAPTUREFRAME);

	psPVRSRVPDumpIsLastCaptureFrameOUT->eError =
	    PDumpIsLastCaptureFrameKM();

	return 0;
}

static IMG_INT
PVRSRVBridgePVRSRVPDumpStartInitPhase(IMG_UINT32 ui32BridgeID,
				      PVRSRV_BRIDGE_IN_PVRSRVPDUMPSTARTINITPHASE
				      * psPVRSRVPDumpStartInitPhaseIN,
				      PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSTARTINITPHASE
				      * psPVRSRVPDumpStartInitPhaseOUT,
				      CONNECTION_DATA * psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVR_UNREFERENCED_PARAMETER(psPVRSRVPDumpStartInitPhaseIN);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSTARTINITPHASE);

	psPVRSRVPDumpStartInitPhaseOUT->eError = PDumpStartInitPhaseKM();

	return 0;
}

static IMG_INT
PVRSRVBridgePVRSRVPDumpStopInitPhase(IMG_UINT32 ui32BridgeID,
				     PVRSRV_BRIDGE_IN_PVRSRVPDUMPSTOPINITPHASE *
				     psPVRSRVPDumpStopInitPhaseIN,
				     PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSTOPINITPHASE
				     * psPVRSRVPDumpStopInitPhaseOUT,
				     CONNECTION_DATA * psConnection)
{

	PVR_UNREFERENCED_PARAMETER(psConnection);

	PVRSRV_BRIDGE_ASSERT_CMD(ui32BridgeID,
				 PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSTOPINITPHASE);

	psPVRSRVPDumpStopInitPhaseOUT->eError =
	    PDumpStopInitPhaseKM(psPVRSRVPDumpStopInitPhaseIN->eModuleID);

	return 0;
}

PVRSRV_ERROR RegisterPDUMPFunctions(IMG_VOID);
IMG_VOID UnregisterPDUMPFunctions(IMG_VOID);

/*
 * Register all PDUMP functions with services
 */
PVRSRV_ERROR RegisterPDUMPFunctions(IMG_VOID)
{
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPISCAPTURING,
			      PVRSRVBridgePVRSRVPDumpIsCapturing);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPCOMMENT,
			      PVRSRVBridgePVRSRVPDumpComment);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSETFRAME,
			      PVRSRVBridgePVRSRVPDumpSetFrame);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPISLASTCAPTUREFRAME,
			      PVRSRVBridgePVRSRVPDumpIsLastCaptureFrame);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSTARTINITPHASE,
			      PVRSRVBridgePVRSRVPDumpStartInitPhase);
	SetDispatchTableEntry(PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSTOPINITPHASE,
			      PVRSRVBridgePVRSRVPDumpStopInitPhase);

	return PVRSRV_OK;
}

/*
 * Unregister all pdump functions with services
 */
IMG_VOID UnregisterPDUMPFunctions(IMG_VOID)
{
}
