									     /**************************************************************************//*!
									        @File           sync.h
									        @Title          Synchronisation interface header
									        @Date
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Defines the client side interface for synchronisation
    *//***************************************************************************/

#include "img_types.h"
#include "pvrsrv_error.h"
#include "sync_external.h"
#include "pdumpdefs.h"

#ifndef _SYNC_
#define _SYNC_

									    /*************************************************************************//*!
									       @Function       SyncPrimContextCreate

									       @Description    Create a new synchronisation context

									       @Input          hBridge                 Bridge handle

									       @Input          hDeviceNode             Device node handle

									       @Output         hSyncPrimContext        Handle to the created synchronisation
									       primitive context

									       @Return         PVRSRV_OK if the synchronisation primitive context was
									       successfully created
									     */
/*****************************************************************************/
PVRSRV_ERROR
SyncPrimContextCreate(SYNC_BRIDGE_HANDLE hBridge,
		      IMG_HANDLE hDeviceNode,
		      PSYNC_PRIM_CONTEXT * hSyncPrimContext);

									    /*************************************************************************//*!
									       @Function       SyncPrimContextDestroy

									       @Description    Destroy a synchronisation context

									       @Input          hSyncPrimContext        Handle to the synchronisation
									       primitive context to destroy

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID SyncPrimContextDestroy(PSYNC_PRIM_CONTEXT hSyncPrimContext);

									    /*************************************************************************//*!
									       @Function       SyncPrimAlloc

									       @Description    Allocate a new synchronisation primitive on the specified
									       synchronisation context

									       @Input          hSyncPrimContext        Handle to the synchronisation
									       primitive context

									       @Output         ppsSync                 Created synchronisation primitive

									       @Return         PVRSRV_OK if the synchronisation primitive was
									       successfully created
									     */
/*****************************************************************************/
PVRSRV_ERROR
SyncPrimAlloc(PSYNC_PRIM_CONTEXT hSyncPrimContext,
	      PVRSRV_CLIENT_SYNC_PRIM ** ppsSync);

									    /*************************************************************************//*!
									       @Function       SyncPrimFree

									       @Description    Free a synchronisation primitive

									       @Input          psSync                  The synchronisation primitive to free

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID SyncPrimFree(PVRSRV_CLIENT_SYNC_PRIM * psSync);

									    /*************************************************************************//*!
									       @Function       SyncPrimSet

									       @Description    Set the synchronisation primitive to a value

									       @Input          psSync                  The synchronisation primitive to set

									       @Input          ui32Value               Value to set it to

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID SyncPrimSet(PVRSRV_CLIENT_SYNC_PRIM * psSync, IMG_UINT32 ui32Value);

#if defined(NO_HARDWARE)

									    /*************************************************************************//*!
									       @Function       SyncPrimNoHwUpdate

									       @Description    Updates the synchronisation primitive value (in NoHardware drivers)

									       @Input          psSync                  The synchronisation primitive to update

									       @Input          ui32Value               Value to update it to

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID
SyncPrimNoHwUpdate(PVRSRV_CLIENT_SYNC_PRIM * psSync, IMG_UINT32 ui32Value);
#endif

#if !defined(__KERNEL__)
PVRSRV_ERROR
SyncPrimExport(PVRSRV_CLIENT_SYNC_PRIM * psSync,
	       PVRSRV_CLIENT_SYNC_PRIM_HANDLE * phExport);

#if !defined(SUPPORT_SECURE_EXPORT)
IMG_VOID
SyncPrimUnexport(IMG_HANDLE hBridge, PVRSRV_CLIENT_SYNC_PRIM_HANDLE hExport);
#endif

PVRSRV_ERROR
SyncPrimImport(SYNC_BRIDGE_HANDLE hBridge,
	       PVRSRV_CLIENT_SYNC_PRIM_HANDLE hImport,
	       PVRSRV_CLIENT_SYNC_PRIM ** ppsSync);
#endif

PVRSRV_ERROR
SyncPrimServerAlloc(SYNC_BRIDGE_HANDLE hBridge,
		    IMG_HANDLE hDeviceNode, PVRSRV_CLIENT_SYNC_PRIM ** ppsSync);

PVRSRV_ERROR
SyncPrimServerGetStatus(IMG_UINT32 ui32SyncCount,
			PVRSRV_CLIENT_SYNC_PRIM ** papsSync,
			IMG_UINT32 * pui32UID,
			IMG_UINT32 * pui32FWAddr,
			IMG_UINT32 * pui32CurrentOp, IMG_UINT32 * pui32NextOp);

PVRSRV_ERROR SyncPrimServerQueueOp(PVRSRV_CLIENT_SYNC_PRIM_OP * psSyncOp);

IMG_BOOL SyncPrimIsServerSync(PVRSRV_CLIENT_SYNC_PRIM * psSync);

IMG_HANDLE SyncPrimGetServerHandle(PVRSRV_CLIENT_SYNC_PRIM * psSync);

PVRSRV_ERROR
SyncPrimOpCreate(IMG_UINT32 ui32SyncCount,
		 PVRSRV_CLIENT_SYNC_PRIM ** papsSyncPrim,
		 PSYNC_OP_COOKIE * ppsCookie);

PVRSRV_ERROR
SyncPrimOpTake(PSYNC_OP_COOKIE psCookie,
	       IMG_UINT32 ui32SyncCount,
	       PVRSRV_CLIENT_SYNC_PRIM_OP * pasSyncOp);

PVRSRV_ERROR SyncPrimOpReady(PSYNC_OP_COOKIE psCookie, IMG_BOOL * pbReady);

PVRSRV_ERROR SyncPrimOpComplete(PSYNC_OP_COOKIE psCookie);

IMG_VOID SyncPrimOpDestroy(PSYNC_OP_COOKIE psCookie);

#if defined(PDUMP)
									    /*************************************************************************//*!
									       @Function       SyncPrimPDump

									       @Description    PDump the current value of the synchronisation primitive

									       @Input          psSync                  The synchronisation primitive to PDump

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID SyncPrimPDump(PVRSRV_CLIENT_SYNC_PRIM * psSync);

									    /*************************************************************************//*!
									       @Function       SyncPrimPDumpPol

									       @Description    Do a PDump poll of the synchronisation primitive

									       @Input          psSync                  The synchronisation primitive to PDump

									       @Input          ui32Value               Value to use

									       @Input          ui32Mask                PDump mask operator

									       @Input          ui32PDumpFlags          PDump flags

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID
SyncPrimPDumpPol(PVRSRV_CLIENT_SYNC_PRIM * psSync,
		 IMG_UINT32 ui32Value,
		 IMG_UINT32 ui32Mask,
		 PDUMP_POLL_OPERATOR eOperator, IMG_UINT32 ui32PDumpFlags);

									    /*************************************************************************//*!
									       @Function       SyncPrimPDumpCBP

									       @Description    Do a PDump CB poll using the synchronisation primitive

									       @Input          psSync                  The synchronisation primitive to PDump

									       @Input          uiWriteOffset           Current write offset of buffer

									       @Input          uiPacketSize            Size of the packet to write into CB

									       @Input          uiBufferSize            Size of the CB

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID
SyncPrimPDumpCBP(PVRSRV_CLIENT_SYNC_PRIM * psSync,
		 IMG_UINT32 uiWriteOffset,
		 IMG_UINT32 uiPacketSize, IMG_UINT32 uiBufferSize);

									    /*************************************************************************//*!
									       @Function       SyncPrimPDumpClientContexts

									       @Description    Pdump all the synchronization memory for this client process

									       @Return         None
									     */
/*****************************************************************************/
IMG_VOID SyncPrimPDumpClientContexts(IMG_VOID);
#else

#ifdef INLINE_IS_PRAGMA
#pragma inline(SyncPrimPDump)
#endif
static INLINE IMG_VOID SyncPrimPDump(PVRSRV_CLIENT_SYNC_PRIM * psSync)
{
	PVR_UNREFERENCED_PARAMETER(psSync);
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(SyncPrimPDumpPol)
#endif
static INLINE IMG_VOID
SyncPrimPDumpPol(PVRSRV_CLIENT_SYNC_PRIM * psSync,
		 IMG_UINT32 ui32Value,
		 IMG_UINT32 ui32Mask,
		 PDUMP_POLL_OPERATOR eOperator, IMG_UINT32 ui32PDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psSync);
	PVR_UNREFERENCED_PARAMETER(ui32Value);
	PVR_UNREFERENCED_PARAMETER(ui32Mask);
	PVR_UNREFERENCED_PARAMETER(eOperator);
	PVR_UNREFERENCED_PARAMETER(ui32PDumpFlags);
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(SyncPrimPDumpCBP)
#endif
static INLINE IMG_VOID
SyncPrimPDumpCBP(PVRSRV_CLIENT_SYNC_PRIM * psSync,
		 IMG_UINT32 uiWriteOffset,
		 IMG_UINT32 uiPacketSize, IMG_UINT32 uiBufferSize)
{
	PVR_UNREFERENCED_PARAMETER(psSync);
	PVR_UNREFERENCED_PARAMETER(uiWriteOffset);
	PVR_UNREFERENCED_PARAMETER(uiPacketSize);
	PVR_UNREFERENCED_PARAMETER(uiBufferSize);
}

static INLINE IMG_VOID SyncPrimPDumpClientContexts(IMG_VOID)
{
}
#endif				/* PDUMP */
#endif				/* _PVRSRV_SYNC_ */
