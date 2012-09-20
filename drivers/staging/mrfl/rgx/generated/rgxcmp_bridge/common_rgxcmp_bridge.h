									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for rgxcmp
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for rgxcmp
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_RGXCMP_BRIDGE_H
#define COMMON_RGXCMP_BRIDGE_H

#include "rgx_bridge.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_RGXCMP_CMD_FIRST			(PVRSRV_BRIDGE_RGXCMP_START)
#define PVRSRV_BRIDGE_RGXCMP_RGXCREATECOMPUTECONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXCMP_CMD_FIRST+0)
#define PVRSRV_BRIDGE_RGXCMP_RGXDESTROYCOMPUTECONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXCMP_CMD_FIRST+1)
#define PVRSRV_BRIDGE_RGXCMP_RGXKICKCDM			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXCMP_CMD_FIRST+2)
#define PVRSRV_BRIDGE_RGXCMP_RGXFLUSHCOMPUTEDATA			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXCMP_CMD_FIRST+3)
#define PVRSRV_BRIDGE_RGXCMP_CMD_LAST			(PVRSRV_BRIDGE_RGXCMP_CMD_FIRST+3)

/*******************************************
            RGXCreateComputeContext          
 *******************************************/

/* Bridge in structure for RGXCreateComputeContext */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATECOMPUTECONTEXT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hCmpCCBMemDesc;
	IMG_HANDLE hCmpCCBCtlMemDesc;
	IMG_UINT32 ui32Priority;
	IMG_DEV_VIRTADDR sMCUFenceAddr;
	IMG_HANDLE hPrivData;
} PVRSRV_BRIDGE_IN_RGXCREATECOMPUTECONTEXT;

/* Bridge out structure for RGXCreateComputeContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATECOMPUTECONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
	IMG_HANDLE hFWComputeContext;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATECOMPUTECONTEXT;

/*******************************************
            RGXDestroyComputeContext          
 *******************************************/

/* Bridge in structure for RGXDestroyComputeContext */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYCOMPUTECONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
} PVRSRV_BRIDGE_IN_RGXDESTROYCOMPUTECONTEXT;

/* Bridge out structure for RGXDestroyComputeContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYCOMPUTECONTEXT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYCOMPUTECONTEXT;

/*******************************************
            RGXKickCDM          
 *******************************************/

/* Bridge in structure for RGXKickCDM */
typedef struct PVRSRV_BRIDGE_IN_RGXKICKCDM_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hFWComputeContext;
	IMG_UINT32 ui32cCCBWoffUpdate;
} PVRSRV_BRIDGE_IN_RGXKICKCDM;

/* Bridge out structure for RGXKickCDM */
typedef struct PVRSRV_BRIDGE_OUT_RGXKICKCDM_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXKICKCDM;

/*******************************************
            RGXFlushComputeData          
 *******************************************/

/* Bridge in structure for RGXFlushComputeData */
typedef struct PVRSRV_BRIDGE_IN_RGXFLUSHCOMPUTEDATA_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hFWComputeContext;
} PVRSRV_BRIDGE_IN_RGXFLUSHCOMPUTEDATA;

/* Bridge out structure for RGXFlushComputeData */
typedef struct PVRSRV_BRIDGE_OUT_RGXFLUSHCOMPUTEDATA_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXFLUSHCOMPUTEDATA;

#endif				/* COMMON_RGXCMP_BRIDGE_H */
