/*************************************************************************/ /*!
@File
@Title          RGX TA and 3D Functionality
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the RGX TA and 3D Functionality
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#if !defined(__RGXTA3D_H__)
#define __RGXTA3D_H__

#include "devicemem.h"
#include "devicemem_server.h"
#include "device.h"
#include "rgxdevice.h"
#include "rgx_fwif_shared.h"
#include "rgx_fwif_resetframework.h"
#include "rgxfwutils.h"
#include "sync_server.h"
#include "connection_server.h"

#if defined (__cplusplus)
extern "C" {
#endif

typedef struct _RGX_FREELIST_ RGX_FREELIST;
typedef struct _RGX_PMR_NODE_ RGX_PMR_NODE;

typedef struct {
	PVRSRV_DEVICE_NODE		*psDeviceNode;
	DEVMEM_MEMDESC			*psFWRenderContextMemDesc;
	DEVMEM_MEMDESC			*psFWRenderContextStateMemDesc;	/*!< TA/3D context suspend state */
	DEVMEM_MEMDESC			*psFWFrameworkMemDesc;			/*!< Framework state for FW */
	RGX_FWCOMCTX_CLEANUP	sFWTAContextCleanup;
	RGX_FWCOMCTX_CLEANUP	sFW3DContextCleanup;
	IMG_UINT32				ui32CleanupStatus;
#define RC_CLEANUP_TA_COMPLETE		(1 << 0)
#define RC_CLEANUP_3D_COMPLETE		(1 << 1)
	PVRSRV_CLIENT_SYNC_PRIM	*psCleanupSync;
	IMG_BOOL				bDumpedTACCBCtlAlready;
	IMG_BOOL				bDumped3DCCBCtlAlready;
} RGX_RC_CLEANUP_DATA;

typedef struct {
	PVRSRV_DEVICE_NODE		*psDeviceNode;
	DEVMEM_MEMDESC			*psFWHWRTDataMemDesc;
	DEVMEM_MEMDESC			*psRTACtlMemDesc;
	DEVMEM_MEMDESC			*psRTArrayMemDesc;
	RGX_FREELIST 			*apsFreeLists[RGXFW_MAX_FREELISTS];
	PVRSRV_CLIENT_SYNC_PRIM	*psCleanupSync;
} RGX_RTDATA_CLEANUP_DATA;

struct _RGX_FREELIST_ {
	PVRSRV_RGXDEV_INFO 		*psDevInfo;

	/* Free list PMR */
	PMR						*psFreeListPMR;
	IMG_DEVMEM_OFFSET_T		uiFreeListPMROffset;

	/* Freelist config */
	IMG_UINT32				ui32MaxFLPages;
	IMG_UINT32				ui32InitFLPages;
	IMG_UINT32				ui32CurrentFLPages;
	IMG_UINT32				ui32GrowFLPages;
	IMG_UINT32				ui32FreelistID;
	IMG_UINT64				ui64FreelistChecksum;	/* checksum over freelist content */
	IMG_BOOL				bCheckFreelist;			/* freelist check enabled */
	IMG_UINT32				ui32RefCount;			/* freelist reference counting */

	IMG_UINT32				ui32NumGrowReqByApp;	/* Total number of grow requests by Application*/
	IMG_UINT32				ui32NumGrowReqByFW;		/* Total Number of grow requests by Firmware */
	IMG_UINT32				ui32NumHighPages;		/* High Mark of pages in the freelist */

	/* Memory Blocks */
	DLLIST_NODE				sMemoryBlockHead;
	DLLIST_NODE				sMemoryBlockInitHead;
	DLLIST_NODE				sNode;

	/* FW data structures */
	DEVMEM_MEMDESC			*psFWFreelistMemDesc;
	RGXFWIF_DEV_VIRTADDR	sFreeListFWDevVAddr;

	PVRSRV_CLIENT_SYNC_PRIM	*psCleanupSync;
} ;

struct _RGX_PMR_NODE_ {
	RGX_FREELIST			*psFreeList;
	PMR						*psPMR;
	PMR_PAGELIST 			*psPageList;
	DLLIST_NODE				sMemoryBlock;
	IMG_UINT32				ui32NumPages;
	IMG_BOOL				bInternal;
} ;

typedef struct {
	PVRSRV_DEVICE_NODE		*psDeviceNode;
	DEVMEM_MEMDESC			*psRenderTargetMemDesc;
} RGX_RT_CLEANUP_DATA;

typedef struct {
	PVRSRV_RGXDEV_INFO		*psDevInfo;
	DEVMEM_MEMDESC			*psZSBufferMemDesc;
	RGXFWIF_DEV_VIRTADDR	sZSBufferFWDevVAddr;

	DEVMEMINT_RESERVATION 	*psReservation;
	PMR 					*psPMR;
	DEVMEMINT_MAPPING 		*psMapping;
	PVRSRV_MEMALLOCFLAGS_T 	uiMapFlags;
	IMG_UINT32 				ui32ZSBufferID;
	IMG_UINT32 				ui32RefCount;
	IMG_BOOL				bOnDemand;

	IMG_BOOL				ui32NumReqByApp;		/* Number of Backing Requests from  Application */
	IMG_BOOL				ui32NumReqByFW;			/* Number of Backing Requests from Firmware */

	DLLIST_NODE	sNode;

	PVRSRV_CLIENT_SYNC_PRIM	*psCleanupSync;
}RGX_ZSBUFFER_DATA;

typedef struct {
	RGX_ZSBUFFER_DATA		*psZSBuffer;
} RGX_POPULATION;

/* Dump the physical pages of a freelist */
IMG_BOOL RGXDumpFreeListPageList(RGX_FREELIST *psFreeList);


/* Create HWRTDataSet */
IMG_EXPORT
PVRSRV_ERROR RGXCreateHWRTData(PVRSRV_DEVICE_NODE	*psDeviceNode, 
							   IMG_UINT32			psRenderTarget,
							   IMG_DEV_VIRTADDR		psPMMListDevVAddr,
							   IMG_DEV_VIRTADDR		psVFPPageTableAddr,
							   RGX_FREELIST			*apsFreeLists[RGXFW_MAX_FREELISTS],
							   RGX_RTDATA_CLEANUP_DATA	**ppsCleanupData,
							   DEVMEM_MEMDESC			**ppsRTACtlMemDesc,
							   IMG_UINT16			ui16MaxRTs,
							   DEVMEM_MEMDESC		**psMemDesc,
							   IMG_UINT32			*puiHWRTData);

/* Destroy HWRTData */
IMG_EXPORT
PVRSRV_ERROR RGXDestroyHWRTData(RGX_RTDATA_CLEANUP_DATA *psCleanupData);

/* Create Render Target */
IMG_EXPORT
PVRSRV_ERROR RGXCreateRenderTarget(PVRSRV_DEVICE_NODE	*psDeviceNode,
								   IMG_DEV_VIRTADDR		psVHeapTableDevVAddr,
								   RGX_RT_CLEANUP_DATA	**ppsCleanupData,
								   IMG_UINT32			*sRenderTargetFWDevVAddr);

/* Destroy render target */
IMG_EXPORT
PVRSRV_ERROR RGXDestroyRenderTarget(RGX_RT_CLEANUP_DATA *psCleanupData);


/*
	RGXCreateZSBuffer
*/
IMG_EXPORT
PVRSRV_ERROR RGXCreateZSBufferKM(PVRSRV_DEVICE_NODE				*psDeviceNode,
								DEVMEMINT_RESERVATION 	*psReservation,
								PMR 					*psPMR,
								PVRSRV_MEMALLOCFLAGS_T 	uiMapFlags,
								RGX_ZSBUFFER_DATA		 	**ppsZSBuffer,
								IMG_UINT32					*sRenderTargetFWDevVAddr);

/*
	RGXDestroyZSBuffer
*/
IMG_EXPORT
PVRSRV_ERROR RGXDestroyZSBufferKM(RGX_ZSBUFFER_DATA *psZSBuffer);


/*
 * RGXBackingZSBuffer()
 *
 * Backs ZS-Buffer with physical pages
 */
PVRSRV_ERROR
RGXBackingZSBuffer(RGX_ZSBUFFER_DATA *psZSBuffer);

/*
 * RGXPopulateZSBufferKM()
 *
 * Backs ZS-Buffer with physical pages (called by Bridge calls)
 */
IMG_EXPORT
PVRSRV_ERROR RGXPopulateZSBufferKM(RGX_ZSBUFFER_DATA *psZSBuffer,
									RGX_POPULATION **ppsPopulation);

/*
 * RGXUnbackingZSBuffer()
 *
 * Frees ZS-Buffer's physical pages
 */
IMG_EXPORT
PVRSRV_ERROR RGXUnbackingZSBuffer(RGX_ZSBUFFER_DATA *psZSBuffer);

/*
 * RGXUnpopulateZSBufferKM()
 *
 * Frees ZS-Buffer's physical pages (called by Bridge calls )
 */
IMG_EXPORT
PVRSRV_ERROR RGXUnpopulateZSBufferKM(RGX_POPULATION *psPopulation);

/*
	RGXProcessRequestZSBufferBacking
*/
IMG_EXPORT
IMG_VOID RGXProcessRequestZSBufferBacking(PVRSRV_RGXDEV_INFO *psDevInfo,
										IMG_UINT32 ui32ZSBufferID);

/*
	RGXProcessRequestZSBufferUnbacking
*/
IMG_EXPORT
IMG_VOID RGXProcessRequestZSBufferUnbacking(PVRSRV_RGXDEV_INFO *psDevInfo,
										IMG_UINT32 ui32ZSBufferID);

/*
	RGXGrowFreeList
*/
IMG_INTERNAL
PVRSRV_ERROR RGXGrowFreeList(RGX_FREELIST *psFreeList,
									IMG_UINT32 ui32NumPages,
									PDLLIST_NODE pListHeader);

/* Create free list */
IMG_EXPORT
PVRSRV_ERROR RGXCreateFreeList(PVRSRV_DEVICE_NODE	*psDeviceNode, 
							   IMG_UINT32			ui32MaxFLPages,
							   IMG_UINT32			ui32InitFLPages,
							   IMG_UINT32			ui32GrowFLPages,
							   IMG_BOOL				bCheckFreelist,
							   IMG_DEV_VIRTADDR		sFreeListDevVAddr,
							   PMR					*psFreeListPMR,
							   IMG_DEVMEM_OFFSET_T	uiFreeListPMROffset,
							   RGX_FREELIST			**ppsFreeList);

/* Destroy free list */
IMG_EXPORT
PVRSRV_ERROR RGXDestroyFreeList(RGX_FREELIST *psFreeList);

/*
	RGXProcessRequestGrow
*/
IMG_EXPORT
IMG_VOID RGXProcessRequestGrow(PVRSRV_RGXDEV_INFO *psDevInfo,
								IMG_UINT32 ui32FreelistID);


/* Grow free list */
IMG_EXPORT
PVRSRV_ERROR RGXAddBlockToFreeListKM(RGX_FREELIST *psFreeList,
										IMG_UINT32 ui32NumPages);

/* Shrink free list */
IMG_EXPORT
PVRSRV_ERROR RGXRemoveBlockFromFreeListKM(RGX_FREELIST *psFreeList);


/* Reconstruct free list after Hardware Recovery */
IMG_VOID RGXProcessRequestFreelistsReconstruction(PVRSRV_RGXDEV_INFO *psDevInfo,
								RGXFWIF_DM eDM,
								IMG_UINT32 ui32FreelistsCount,
								IMG_UINT32 *paui32Freelists);

/*!
*******************************************************************************

 @Function	PVRSRVRGXCreateRenderContextKM

 @Description
	Server-side implementation of RGXCreateRenderContext

 @Input pvDeviceNode - device node
 
FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXCreateRenderContextKM(PVRSRV_DEVICE_NODE		*psDeviceNode,
											DEVMEM_MEMDESC 			*psTACCBMemDesc,
											DEVMEM_MEMDESC 			*psTACCBCtlMemDesc,
											DEVMEM_MEMDESC 			*ps3DCCBMemDesc,
											DEVMEM_MEMDESC 			*ps3DCCBCtlMemDesc,
											RGX_RC_CLEANUP_DATA		**ppsCleanupData,
											DEVMEM_MEMDESC 			**ppsFWRenderContextMemDesc,
											DEVMEM_MEMDESC 			**ppsFWContextStateMemDesc,
											IMG_UINT32				ui32Priority,
											IMG_DEV_VIRTADDR		sMCUFenceAddr,
											IMG_DEV_VIRTADDR		psVDMStackPointer,
											IMG_UINT32				ui32FrameworkRegisterSize,
											IMG_PBYTE				pbyFrameworkRegisters,
											IMG_HANDLE				hMemCtxPrivData);


/*!
*******************************************************************************

 @Function	PVRSRVRGXDestroyRenderContextKM

 @Description
	Server-side implementation of RGXDestroyRenderContext

 @Input pvDeviceNode - device node

FIXME fill this in

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXDestroyRenderContextKM(RGX_RC_CLEANUP_DATA *psCleanupData);


/*!
*******************************************************************************

 @Function	PVRSRVRGXKickTA3DKM

 @Description
	Server-side implementation of RGXKickTA3D

 @Input pvDeviceNode - device node
 @Input psFWRenderContextMemDesc - memdesc for the firmware render context
 @Input bLastTAInScene - IMG_TRUE if last TA in scene (terminate or abort)
 @Input bKickTA - IMG_TRUE to kick the TA
 @Input bKick3D - IMG_TRUE to kick the 3D
 @Input ui32TAcCCBWoffUpdate - New fw Woff for the client TA CCB
 @Input ui323DcCCBWoffUpdate - New fw Woff for the client 3D CCB

 @Return   PVRSRV_ERROR

******************************************************************************/
IMG_EXPORT
PVRSRV_ERROR PVRSRVRGXKickTA3DKM(CONNECTION_DATA	*psConnection,
								 PVRSRV_DEVICE_NODE	*psDeviceNode,
								 DEVMEM_MEMDESC 	*psFWRenderContextMemDesc,
								 IMG_BOOL			bLastTAInScene,
								 IMG_BOOL			bKickTA,
								 IMG_BOOL			bKick3D,
								 IMG_UINT32			*pui32TAcCCBWoffUpdate,
								 IMG_UINT32			*pui323DcCCBWoffUpdate,
								 DEVMEM_MEMDESC 	*psTAcCCBMemDesc,
								 DEVMEM_MEMDESC 	*psTACCBCtlMemDesc,
								 DEVMEM_MEMDESC 	*ps3DcCCBMemDesc,
								 DEVMEM_MEMDESC 	*ps3DCCBCtlMemDesc,
								 IMG_UINT32			ui32TAServerSyncPrims,
								 PVRSRV_CLIENT_SYNC_PRIM_OP**	pasTASyncOp,
								 SERVER_SYNC_PRIMITIVE **pasTAServerSyncs,
								 IMG_UINT32			ui323DServerSyncPrims,
								 PVRSRV_CLIENT_SYNC_PRIM_OP**	pas3DSyncOp,
								 SERVER_SYNC_PRIMITIVE **pas3DServerSyncs,
								 IMG_UINT32			ui32TACmdSize,
								 IMG_PBYTE			pui8TACmd,
								 IMG_UINT32			ui32TAFenceEnd,
								 IMG_UINT32			ui32TAUpdateEnd,
								 IMG_UINT32			ui323DCmdSize,
								 IMG_PBYTE			pui83DCmd,
								 IMG_UINT32			ui323DFenceEnd,
								 IMG_UINT32			ui323DUpdateEnd,
								 IMG_UINT32         ui32NumFenceFds,
								 IMG_INT32          *ai32FenceFds,
								 IMG_BOOL			bPDumpContinuous,
								 RGX_RC_CLEANUP_DATA *psCleanupData);

#endif /* __RGXTA3D_H__ */
