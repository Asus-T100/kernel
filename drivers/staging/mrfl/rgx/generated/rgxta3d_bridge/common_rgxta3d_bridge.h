									    /*************************************************************************//*!
									       @File
									       @Title          Common bridge header for rgxta3d
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Declares common defines and structures that are used by both
									       the client and sever side of the bridge for rgxta3d
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef COMMON_RGXTA3D_BRIDGE_H
#define COMMON_RGXTA3D_BRIDGE_H

#include "rgx_bridge.h"

/* FIXME: need to create pvrbridge_common.h" */
#include "pvr_bridge.h"

#define PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST			(PVRSRV_BRIDGE_RGXTA3D_START)
#define PVRSRV_BRIDGE_RGXTA3D_RGXCREATEHWRTDATA			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+0)
#define PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYHWRTDATA			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+1)
#define PVRSRV_BRIDGE_RGXTA3D_RGXCREATERENDERTARGET			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+2)
#define PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYRENDERTARGET			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+3)
#define PVRSRV_BRIDGE_RGXTA3D_RGXCREATEFREELIST			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+4)
#define PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYFREELIST			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+5)
#define PVRSRV_BRIDGE_RGXTA3D_RGXCREATERENDERCONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+6)
#define PVRSRV_BRIDGE_RGXTA3D_RGXDESTROYRENDERCONTEXT			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+7)
#define PVRSRV_BRIDGE_RGXTA3D_RGXKICKTA3D			PVRSRV_IOWR(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+8)
#define PVRSRV_BRIDGE_RGXTA3D_CMD_LAST			(PVRSRV_BRIDGE_RGXTA3D_CMD_FIRST+8)

/*******************************************
            RGXCreateHWRTData          
 *******************************************/

/* Bridge in structure for RGXCreateHWRTData */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATEHWRTDATA_TAG {
	IMG_HANDLE hDevNode;
	IMG_UINT32 ui32RenderTarget;
	IMG_DEV_VIRTADDR sPMMlistDevVAddr;
	IMG_UINT32 *pui32apsFreeLists;
} PVRSRV_BRIDGE_IN_RGXCREATEHWRTDATA;

/* Bridge out structure for RGXCreateHWRTData */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATEHWRTDATA_TAG {
	IMG_HANDLE hCleanupCookie;
	IMG_HANDLE hsHWRTDataMemDesc;
	IMG_UINT32 ui32FWHWRTData;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATEHWRTDATA;

/*******************************************
            RGXDestroyHWRTData          
 *******************************************/

/* Bridge in structure for RGXDestroyHWRTData */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYHWRTDATA_TAG {
	IMG_HANDLE hCleanupCookie;
} PVRSRV_BRIDGE_IN_RGXDESTROYHWRTDATA;

/* Bridge out structure for RGXDestroyHWRTData */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYHWRTDATA_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYHWRTDATA;

/*******************************************
            RGXCreateRenderTarget          
 *******************************************/

/* Bridge in structure for RGXCreateRenderTarget */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATERENDERTARGET_TAG {
	IMG_HANDLE hDevNode;
	IMG_DEV_VIRTADDR spsVHeapTableDevVAddr;
} PVRSRV_BRIDGE_IN_RGXCREATERENDERTARGET;

/* Bridge out structure for RGXCreateRenderTarget */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATERENDERTARGET_TAG {
	IMG_HANDLE hsRenderTargetMemDesc;
	IMG_UINT32 ui32sRenderTargetFWDevVAddr;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATERENDERTARGET;

/*******************************************
            RGXDestroyRenderTarget          
 *******************************************/

/* Bridge in structure for RGXDestroyRenderTarget */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYRENDERTARGET_TAG {
	IMG_HANDLE hsRenderTargetMemDesc;
} PVRSRV_BRIDGE_IN_RGXDESTROYRENDERTARGET;

/* Bridge out structure for RGXDestroyRenderTarget */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYRENDERTARGET_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYRENDERTARGET;

/*******************************************
            RGXCreateFreeList          
 *******************************************/

/* Bridge in structure for RGXCreateFreeList */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATEFREELIST_TAG {
	IMG_HANDLE hDevNode;
	IMG_UINT32 ui32ui32TotalPMPages;
	IMG_DEV_VIRTADDR spsFreeListDevVAddr;
} PVRSRV_BRIDGE_IN_RGXCREATEFREELIST;

/* Bridge out structure for RGXCreateFreeList */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATEFREELIST_TAG {
	IMG_HANDLE hsFWFreeListMemDesc;
	IMG_UINT32 ui32sFreeListFWDevVAddr;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATEFREELIST;

/*******************************************
            RGXDestroyFreeList          
 *******************************************/

/* Bridge in structure for RGXDestroyFreeList */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYFREELIST_TAG {
	IMG_HANDLE hsFWFreeListMemDesc;
} PVRSRV_BRIDGE_IN_RGXDESTROYFREELIST;

/* Bridge out structure for RGXDestroyFreeList */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYFREELIST_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYFREELIST;

/*******************************************
            RGXCreateRenderContext          
 *******************************************/

/* Bridge in structure for RGXCreateRenderContext */
typedef struct PVRSRV_BRIDGE_IN_RGXCREATERENDERCONTEXT_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hTACCBMemDesc;
	IMG_HANDLE hTACCBCtlMemDesc;
	IMG_HANDLE h3DCCBMemDesc;
	IMG_HANDLE h3DCCBCtlMemDesc;
	IMG_UINT32 ui32Priority;
	IMG_DEV_VIRTADDR sMCUFenceAddr;
	IMG_DEV_VIRTADDR sVDMCallStackAddr;
	IMG_HANDLE hPrivData;
} PVRSRV_BRIDGE_IN_RGXCREATERENDERCONTEXT;

/* Bridge out structure for RGXCreateRenderContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXCREATERENDERCONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
	IMG_HANDLE hFWRenderContext;
	IMG_HANDLE hFW3DContextState;
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXCREATERENDERCONTEXT;

/*******************************************
            RGXDestroyRenderContext          
 *******************************************/

/* Bridge in structure for RGXDestroyRenderContext */
typedef struct PVRSRV_BRIDGE_IN_RGXDESTROYRENDERCONTEXT_TAG {
	IMG_HANDLE hCleanupCookie;
} PVRSRV_BRIDGE_IN_RGXDESTROYRENDERCONTEXT;

/* Bridge out structure for RGXDestroyRenderContext */
typedef struct PVRSRV_BRIDGE_OUT_RGXDESTROYRENDERCONTEXT_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXDESTROYRENDERCONTEXT;

/*******************************************
            RGXKickTA3D          
 *******************************************/

/* Bridge in structure for RGXKickTA3D */
typedef struct PVRSRV_BRIDGE_IN_RGXKICKTA3D_TAG {
	IMG_HANDLE hDevNode;
	IMG_HANDLE hFWRenderContext;
	IMG_BOOL bbLastTAInScene;
	IMG_BOOL bbKickTA;
	IMG_BOOL bbKick3D;
	IMG_UINT32 ui32TAcCCBWoffUpdate;
	IMG_UINT32 ui323DcCCBWoffUpdate;
} PVRSRV_BRIDGE_IN_RGXKICKTA3D;

/* Bridge out structure for RGXKickTA3D */
typedef struct PVRSRV_BRIDGE_OUT_RGXKICKTA3D_TAG {
	PVRSRV_ERROR eError;
} PVRSRV_BRIDGE_OUT_RGXKICKTA3D;

#endif				/* COMMON_RGXTA3D_BRIDGE_H */
