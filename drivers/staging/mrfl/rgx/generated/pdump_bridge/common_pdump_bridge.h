									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for pdump
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for pdump
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_PDUMP_BRIDGE_H
#define COMMON_PDUMP_BRIDGE_H

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_PDUMP_CMD_FIRST			(PVRSRV_BRIDGE_PDUMP_START)
#define PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPISCAPTURING			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMP_CMD_FIRST+0)
#define PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPCOMMENT			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMP_CMD_FIRST+1)
#define PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSETFRAME			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMP_CMD_FIRST+2)
#define PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPISLASTCAPTUREFRAME			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMP_CMD_FIRST+3)
#define PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSTARTINITPHASE			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMP_CMD_FIRST+4)
#define PVRSRV_BRIDGE_PDUMP_PVRSRVPDUMPSTOPINITPHASE			PVRSRV_IOWR(PVRSRV_BRIDGE_PDUMP_CMD_FIRST+5)
#define PVRSRV_BRIDGE_PDUMP_CMD_LAST			(PVRSRV_BRIDGE_PDUMP_CMD_FIRST+5)

/*******************************************
            PVRSRVPDumpIsCapturing          
 *******************************************/

/* Bridge in structure for PVRSRVPDumpIsCapturing */
typedef struct PVRSRV_BRIDGE_IN_PVRSRVPDUMPISCAPTURING_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_PVRSRVPDUMPISCAPTURING;

/* Bridge out structure for PVRSRVPDumpIsCapturing */
typedef struct PVRSRV_BRIDGE_OUT_PVRSRVPDUMPISCAPTURING_TAG {
	IMG_BOOL bIsCapturing;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PVRSRVPDUMPISCAPTURING;

/*******************************************
            PVRSRVPDumpComment          
 *******************************************/

/* Bridge in structure for PVRSRVPDumpComment */
typedef struct PVRSRV_BRIDGE_IN_PVRSRVPDUMPCOMMENT_TAG {
	IMG_CHAR *puiComment;
	IMG_UINT32 ui32Flags;
} PVRSRV_BRIDGE_IN_PVRSRVPDUMPCOMMENT;

/* Bridge out structure for PVRSRVPDumpComment */
typedef struct PVRSRV_BRIDGE_OUT_PVRSRVPDUMPCOMMENT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PVRSRVPDUMPCOMMENT;

/*******************************************
            PVRSRVPDumpSetFrame          
 *******************************************/

/* Bridge in structure for PVRSRVPDumpSetFrame */
typedef struct PVRSRV_BRIDGE_IN_PVRSRVPDUMPSETFRAME_TAG {
	IMG_UINT32 ui32Frame;
} PVRSRV_BRIDGE_IN_PVRSRVPDUMPSETFRAME;

/* Bridge out structure for PVRSRVPDumpSetFrame */
typedef struct PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSETFRAME_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSETFRAME;

/*******************************************
            PVRSRVPDumpIsLastCaptureFrame          
 *******************************************/

/* Bridge in structure for PVRSRVPDumpIsLastCaptureFrame */
typedef struct PVRSRV_BRIDGE_IN_PVRSRVPDUMPISLASTCAPTUREFRAME_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_PVRSRVPDUMPISLASTCAPTUREFRAME;

/* Bridge out structure for PVRSRVPDumpIsLastCaptureFrame */
typedef struct PVRSRV_BRIDGE_OUT_PVRSRVPDUMPISLASTCAPTUREFRAME_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PVRSRVPDUMPISLASTCAPTUREFRAME;

/*******************************************
            PVRSRVPDumpStartInitPhase          
 *******************************************/

/* Bridge in structure for PVRSRVPDumpStartInitPhase */
typedef struct PVRSRV_BRIDGE_IN_PVRSRVPDUMPSTARTINITPHASE_TAG {
	IMG_UINT32 ui32EmptyStructPlaceholder;
} PVRSRV_BRIDGE_IN_PVRSRVPDUMPSTARTINITPHASE;

/* Bridge out structure for PVRSRVPDumpStartInitPhase */
typedef struct PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSTARTINITPHASE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSTARTINITPHASE;

/*******************************************
            PVRSRVPDumpStopInitPhase          
 *******************************************/

/* Bridge in structure for PVRSRVPDumpStopInitPhase */
typedef struct PVRSRV_BRIDGE_IN_PVRSRVPDUMPSTOPINITPHASE_TAG {
	IMG_MODULE_ID eModuleID;
} PVRSRV_BRIDGE_IN_PVRSRVPDUMPSTOPINITPHASE;

/* Bridge out structure for PVRSRVPDumpStopInitPhase */
typedef struct PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSTOPINITPHASE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PVRSRVPDUMPSTOPINITPHASE;

#endif				/* COMMON_PDUMP_BRIDGE_H */
