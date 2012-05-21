									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for rgxtq
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for rgxtq
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_RGXTQ_BRIDGE_H
#define COMMON_RGXTQ_BRIDGE_H

#include "rgx_bridge.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_RGXTQ_CMD_FIRST			(PVRSRV_BRIDGE_RGXTQ_START)
#define PVRSRV_BRIDGE_RGXTQ_RGXCREATETQ3DCONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTQ_CMD_FIRST+0)
#define PVRSRV_BRIDGE_RGXTQ_RGXDESTROYTQ3DCONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTQ_CMD_FIRST+1)
#define PVRSRV_BRIDGE_RGXTQ_SUBMITTQ3DKICK			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTQ_CMD_FIRST+2)
#define PVRSRV_BRIDGE_RGXTQ_RGXCREATETQ2DCONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTQ_CMD_FIRST+3)
#define PVRSRV_BRIDGE_RGXTQ_RGXDESTROYTQ2DCONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTQ_CMD_FIRST+4)
#define PVRSRV_BRIDGE_RGXTQ_SUBMITTQ2DKICK			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTQ_CMD_FIRST+5)
#define PVRSRV_BRIDGE_RGXTQ_CMD_LAST			(PVRSRV_BRIDGE_RGXTQ_CMD_FIRST+5)

/*******************************************
            RGXCreateTQ3DContext          
 *******************************************/

/* Bridge in structure for RGXCreateTQ3DContext */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATETQ3DCONTEXT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hTQ3DCCBMemDesc;
	IMG_HANDLE hTQ3DCCBCtlMemDesc;
	IMG_UINT32 ui32Priority;
	IMG_DEV_VIRTADDR sMCUFenceAddr;
	IMG_HANDLE hPrivData;
} PVRSRV_BRIDGE_IN_RGXCREATETQ3DCONTEXT;

/* Bridge out structure for RGXCreateTQ3DContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATETQ3DCONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
	IMG_HANDLE hFWTQ3DContext;
	IMG_HANDLE hFWTQ3DContextState;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATETQ3DCONTEXT;

/*******************************************
            RGXDestroyTQ3DContext          
 *******************************************/

/* Bridge in structure for RGXDestroyTQ3DContext */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYTQ3DCONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
} PVRSRV_BRIDGE_IN_RGXDESTROYTQ3DCONTEXT;

/* Bridge out structure for RGXDestroyTQ3DContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYTQ3DCONTEXT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYTQ3DCONTEXT;

/*******************************************
            SubmitTQ3DKick          
 *******************************************/

/* Bridge in structure for SubmitTQ3DKick */
typedef struct PVRSRV_BRIDGE_IN_SUBMITTQ3DKICK_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hFWTQ3DContext;
	IMG_UINT32 ui32ui32TQ3DcCCBWoffUpdate;
	IMG_BOOL bbPDumpContinuous;
} PVRSRV_BRIDGE_IN_SUBMITTQ3DKICK;

/* Bridge out structure for SubmitTQ3DKick */
typedef struct PVRSRV_BRIDGE_OUT_SUBMITTQ3DKICK_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SUBMITTQ3DKICK;

/*******************************************
            RGXCreateTQ2DContext          
 *******************************************/

/* Bridge in structure for RGXCreateTQ2DContext */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATETQ2DCONTEXT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hTQ2DCCBMemDesc;
	IMG_HANDLE hTQ2DCCBCtlMemDesc;
	IMG_UINT32 ui32Priority;
	IMG_HANDLE hPrivData;
} PVRSRV_BRIDGE_IN_RGXCREATETQ2DCONTEXT;

/* Bridge out structure for RGXCreateTQ2DContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATETQ2DCONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
	IMG_HANDLE hFWTQ2DContext;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATETQ2DCONTEXT;

/*******************************************
            RGXDestroyTQ2DContext          
 *******************************************/

/* Bridge in structure for RGXDestroyTQ2DContext */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYTQ2DCONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
} PVRSRV_BRIDGE_IN_RGXDESTROYTQ2DCONTEXT;

/* Bridge out structure for RGXDestroyTQ2DContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYTQ2DCONTEXT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYTQ2DCONTEXT;

/*******************************************
            SubmitTQ2DKick          
 *******************************************/

/* Bridge in structure for SubmitTQ2DKick */
typedef struct PVRSRV_BRIDGE_IN_SUBMITTQ2DKICK_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hFWTQ2DContext;
	IMG_UINT32 ui32ui32TQ2DcCCBWoffUpdate;
	IMG_BOOL bbPDumpContinuous;
} PVRSRV_BRIDGE_IN_SUBMITTQ2DKICK;

/* Bridge out structure for SubmitTQ2DKick */
typedef struct PVRSRV_BRIDGE_OUT_SUBMITTQ2DKICK_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_SUBMITTQ2DKICK;

#endif				/* COMMON_RGXTQ_BRIDGE_H */
