									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for debugmisc
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for debugmisc
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_DEBUGMISC_BRIDGE_H
#define COMMON_DEBUGMISC_BRIDGE_H

#include "rgx_bridge.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_DEBUGMISC_CMD_FIRST			(PVRSRV_BRIDGE_DEBUGMISC_START)
#define PVRSRV_BRIDGE_DEBUGMISC_DEBUGMISCTILINGSETSTATE			PVRSRV_IOWR(PVRSRV_BRIDGE_DEBUGMISC_CMD_FIRST+0)
#define PVRSRV_BRIDGE_DEBUGMISC_DEBUGMISCSLCSETBYPASSSTATE			PVRSRV_IOWR(PVRSRV_BRIDGE_DEBUGMISC_CMD_FIRST+1)
#define PVRSRV_BRIDGE_DEBUGMISC_RGXDEBUGMISCSETFWLOG			PVRSRV_IOWR(PVRSRV_BRIDGE_DEBUGMISC_CMD_FIRST+2)
#define PVRSRV_BRIDGE_DEBUGMISC_CMD_LAST			(PVRSRV_BRIDGE_DEBUGMISC_CMD_FIRST+2)

/*******************************************
            DebugMiscTilingSetState          
 *******************************************/

/* Bridge in structure for DebugMiscTilingSetState */
typedef struct PVRSRV_BRIDGE_IN_DEBUGMISCTILINGSETSTATE_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hMemCtxPrivData;
	IMG_BOOL bbEnabled;
} PVRSRV_BRIDGE_IN_DEBUGMISCTILINGSETSTATE;

/* Bridge out structure for DebugMiscTilingSetState */
typedef struct PVRSRV_BRIDGE_OUT_DEBUGMISCTILINGSETSTATE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEBUGMISCTILINGSETSTATE;

/*******************************************
            DebugMiscSLCSetBypassState          
 *******************************************/

/* Bridge in structure for DebugMiscSLCSetBypassState */
typedef struct PVRSRV_BRIDGE_IN_DEBUGMISCSLCSETBYPASSSTATE_TAG {
	IMG_HANDLE hDevNode;
	IMG_UINT32 ui32Flags;
	IMG_BOOL bIsBypassed;
} PVRSRV_BRIDGE_IN_DEBUGMISCSLCSETBYPASSSTATE;

/* Bridge out structure for DebugMiscSLCSetBypassState */
typedef struct PVRSRV_BRIDGE_OUT_DEBUGMISCSLCSETBYPASSSTATE_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_DEBUGMISCSLCSETBYPASSSTATE;

/*******************************************
            RGXDebugMiscSetFWLog          
 *******************************************/

/* Bridge in structure for RGXDebugMiscSetFWLog */
typedef struct PVRSRV_BRIDGE_IN_RGXDEBUGMISCSETFWLOG_TAG {
	IMG_HANDLE hDevNode;
	IMG_UINT32 ui32RGXFWLogType;
} PVRSRV_BRIDGE_IN_RGXDEBUGMISCSETFWLOG;

/* Bridge out structure for RGXDebugMiscSetFWLog */
typedef struct PVRSRV_BRIDGE_OUT_RGXDEBUGMISCSETFWLOG_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDEBUGMISCSETFWLOG;

#endif				/* COMMON_DEBUGMISC_BRIDGE_H */
