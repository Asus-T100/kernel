									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for rgxpdump
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for rgxpdump
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_RGXPDUMP_BRIDGE_H
#define COMMON_RGXPDUMP_BRIDGE_H

#include "rgx_bridge.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_RGXPDUMP_CMD_FIRST			(PVRSRV_BRIDGE_RGXPDUMP_START)
#define PVRSRV_BRIDGE_RGXPDUMP_PDUMPTRACEBUFFER			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXPDUMP_CMD_FIRST+0)
#define PVRSRV_BRIDGE_RGXPDUMP_PDUMPSIGNATUREBUFFER			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXPDUMP_CMD_FIRST+1)
#define PVRSRV_BRIDGE_RGXPDUMP_CMD_LAST			(PVRSRV_BRIDGE_RGXPDUMP_CMD_FIRST+1)

/*******************************************
            PDumpTraceBuffer          
 *******************************************/

/* Bridge in structure for PDumpTraceBuffer */
typedef struct PVRSRV_BRIDGE_IN_PDUMPTRACEBUFFER_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_UINT32 ui32PDumpFlags;
} PVRSRV_BRIDGE_IN_PDUMPTRACEBUFFER;

/* Bridge out structure for PDumpTraceBuffer */
typedef struct PVRSRV_BRIDGE_OUT_PDUMPTRACEBUFFER_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PDUMPTRACEBUFFER;

/*******************************************
            PDumpSignatureBuffer          
 *******************************************/

/* Bridge in structure for PDumpSignatureBuffer */
typedef struct PVRSRV_BRIDGE_IN_PDUMPSIGNATUREBUFFER_TAG {
	IMG_HANDLE hDeviceNode;
	IMG_UINT32 ui32PDumpFlags;
} PVRSRV_BRIDGE_IN_PDUMPSIGNATUREBUFFER;

/* Bridge out structure for PDumpSignatureBuffer */
typedef struct PVRSRV_BRIDGE_OUT_PDUMPSIGNATUREBUFFER_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_PDUMPSIGNATUREBUFFER;

#endif				/* COMMON_RGXPDUMP_BRIDGE_H */
