									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for breakpoint
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for breakpoint
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_BREAKPOINT_BRIDGE_H
#define COMMON_BREAKPOINT_BRIDGE_H

#include "rgx_bridge.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_BREAKPOINT_CMD_FIRST			(PVRSRV_BRIDGE_BREAKPOINT_START)
#define PVRSRV_BRIDGE_BREAKPOINT_RGXSETBREAKPOINT			PVRSRV_IOWR(PVRSRV_BRIDGE_BREAKPOINT_CMD_FIRST+0)
#define PVRSRV_BRIDGE_BREAKPOINT_RGXCLEARBREAKPOINT			PVRSRV_IOWR(PVRSRV_BRIDGE_BREAKPOINT_CMD_FIRST+1)
#define PVRSRV_BRIDGE_BREAKPOINT_RGXENABLEBREAKPOINT			PVRSRV_IOWR(PVRSRV_BRIDGE_BREAKPOINT_CMD_FIRST+2)
#define PVRSRV_BRIDGE_BREAKPOINT_RGXDISABLEBREAKPOINT			PVRSRV_IOWR(PVRSRV_BRIDGE_BREAKPOINT_CMD_FIRST+3)
#define PVRSRV_BRIDGE_BREAKPOINT_RGXOVERALLOCATEBPREGISTERS			PVRSRV_IOWR(PVRSRV_BRIDGE_BREAKPOINT_CMD_FIRST+4)
#define PVRSRV_BRIDGE_BREAKPOINT_CMD_LAST			(PVRSRV_BRIDGE_BREAKPOINT_CMD_FIRST+4)

/*******************************************
            RGXSetBreakpoint          
 *******************************************/

/* Bridge in structure for RGXSetBreakpoint */
typedef struct PVRSRV_BRIDGE_IN_RGXSETBREAKPOINT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hPrivData;
	IMG_UINT32 eFWDataMaster;
	IMG_UINT32 ui32BreakpointAddr;
	IMG_UINT32 ui32HandlerAddr;
	IMG_UINT32 ui32DM;
} PVRSRV_BRIDGE_IN_RGXSETBREAKPOINT;

/* Bridge out structure for RGXSetBreakpoint */
typedef struct PVRSRV_BRIDGE_OUT_RGXSETBREAKPOINT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXSETBREAKPOINT;

/*******************************************
            RGXClearBreakpoint          
 *******************************************/

/* Bridge in structure for RGXClearBreakpoint */
typedef struct PVRSRV_BRIDGE_IN_RGXCLEARBREAKPOINT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hPrivData;
} PVRSRV_BRIDGE_IN_RGXCLEARBREAKPOINT;

/* Bridge out structure for RGXClearBreakpoint */
typedef struct PVRSRV_BRIDGE_OUT_RGXCLEARBREAKPOINT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCLEARBREAKPOINT;

/*******************************************
            RGXEnableBreakpoint          
 *******************************************/

/* Bridge in structure for RGXEnableBreakpoint */
typedef struct PVRSRV_BRIDGE_IN_RGXENABLEBREAKPOINT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hPrivData;
} PVRSRV_BRIDGE_IN_RGXENABLEBREAKPOINT;

/* Bridge out structure for RGXEnableBreakpoint */
typedef struct PVRSRV_BRIDGE_OUT_RGXENABLEBREAKPOINT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXENABLEBREAKPOINT;

/*******************************************
            RGXDisableBreakpoint          
 *******************************************/

/* Bridge in structure for RGXDisableBreakpoint */
typedef struct PVRSRV_BRIDGE_IN_RGXDISABLEBREAKPOINT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hPrivData;
} PVRSRV_BRIDGE_IN_RGXDISABLEBREAKPOINT;

/* Bridge out structure for RGXDisableBreakpoint */
typedef struct PVRSRV_BRIDGE_OUT_RGXDISABLEBREAKPOINT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDISABLEBREAKPOINT;

/*******************************************
            RGXOverallocateBPRegisters          
 *******************************************/

/* Bridge in structure for RGXOverallocateBPRegisters */
typedef struct PVRSRV_BRIDGE_IN_RGXOVERALLOCATEBPREGISTERS_TAG {
	IMG_HANDLE hDevNode;
	IMG_UINT32 ui32TempRegs;
	IMG_UINT32 ui32SharedRegs;
} PVRSRV_BRIDGE_IN_RGXOVERALLOCATEBPREGISTERS;

/* Bridge out structure for RGXOverallocateBPRegisters */
typedef struct PVRSRV_BRIDGE_OUT_RGXOVERALLOCATEBPREGISTERS_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXOVERALLOCATEBPREGISTERS;

#endif				/* COMMON_BREAKPOINT_BRIDGE_H */
