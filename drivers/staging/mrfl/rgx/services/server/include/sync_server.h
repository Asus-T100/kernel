									     /**************************************************************************//*!
									        @File           sync_server.h
									        @Title          Server side synchronisation interface
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Describes the server side synchronisation functions
    *//***************************************************************************/

#include "img_types.h"
#include "device.h"
#include "devicemem.h"
#include "pdump.h"
#include "pvrsrv_error.h"

#if defined(SUPPORT_SECURE_EXPORT)
#include "connection_server.h"
#endif

#ifndef _SYNC_SERVER_H_
#define _SYNC_SERVER_H_

typedef struct _SERVER_OP_COOKIE_ SERVER_OP_COOKIE;
typedef struct _SERVER_SYNC_PRIMITIVE_ SERVER_SYNC_PRIMITIVE;
typedef struct _SYNC_PRIMITIVE_BLOCK_ SYNC_PRIMITIVE_BLOCK;
typedef struct _SERVER_SYNC_EXPORT_ SERVER_SYNC_EXPORT;

PVRSRV_ERROR
PVRSRVAllocSyncPrimitiveBlockKM(PVRSRV_DEVICE_NODE * psDevNode,
				SYNC_PRIMITIVE_BLOCK ** ppsSyncBlk,
				IMG_UINT32 * puiSyncPrimVAddr,
				IMG_UINT32 * puiSyncPrimBlockSize,
				DEVMEM_EXPORTCOOKIE ** psExportCookie);

PVRSRV_ERROR
PVRSRVExportSyncPrimitiveBlockKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk,
				 DEVMEM_EXPORTCOOKIE ** psExportCookie);

PVRSRV_ERROR
PVRSRVUnexportSyncPrimitiveBlockKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk);

PVRSRV_ERROR PVRSRVFreeSyncPrimitiveBlockKM(SYNC_PRIMITIVE_BLOCK * ppsSyncBlk);

PVRSRV_ERROR
PVRSRVSyncPrimSetKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk, IMG_UINT32 ui32Index,
		    IMG_UINT32 ui32Value);

PVRSRV_ERROR
PVRSRVServerSyncPrimSetKM(SERVER_SYNC_PRIMITIVE * psServerSync,
			  IMG_UINT32 ui32Value);

#if !defined(SUPPORT_SECURE_EXPORT)
PVRSRV_ERROR
PVRSRVSyncPrimServerExportKM(SERVER_SYNC_PRIMITIVE * psSync,
			     SERVER_SYNC_EXPORT ** ppsExport);

PVRSRV_ERROR PVRSRVSyncPrimServerUnexportKM(SERVER_SYNC_EXPORT * psExport);

PVRSRV_ERROR
PVRSRVSyncPrimServerImportKM(SERVER_SYNC_EXPORT * psExport,
			     SERVER_SYNC_PRIMITIVE ** ppsSync,
			     IMG_UINT32 * pui32SyncPrimVAddr);
#else
PVRSRV_ERROR
PVRSRVSyncPrimServerExportKM(CONNECTION_DATA * psConnection,
			     SERVER_SYNC_PRIMITIVE * psSync,
			     IMG_SECURE_TYPE * phSecure,
			     SERVER_SYNC_EXPORT ** ppsExport,
			     CONNECTION_DATA ** ppsSecureConnection);

PVRSRV_ERROR PVRSRVSyncPrimServerUnexportKM(SERVER_SYNC_EXPORT * psExport);

PVRSRV_ERROR
PVRSRVSyncPrimServerImportKM(IMG_SECURE_TYPE hSecure,
			     SERVER_SYNC_PRIMITIVE ** ppsSync,
			     IMG_UINT32 * pui32SyncPrimVAddr);
#endif

PVRSRV_ERROR
PVRSRVServerSyncAllocKM(PVRSRV_DEVICE_NODE * psDevNode,
			SERVER_SYNC_PRIMITIVE ** ppsSync,
			IMG_UINT32 * pui32SyncPrimVAddr);
PVRSRV_ERROR PVRSRVServerSyncFreeKM(SERVER_SYNC_PRIMITIVE * psSync);

PVRSRV_ERROR
PVRSRVServerSyncGetStatusKM(IMG_UINT32 ui32SyncCount,
			    SERVER_SYNC_PRIMITIVE ** papsSyncs,
			    IMG_UINT32 * pui32UID,
			    IMG_UINT32 * pui32FWAddr,
			    IMG_UINT32 * pui32CurrentOp,
			    IMG_UINT32 * pui32NextOp);

PVRSRV_ERROR
PVRSRVServerSyncQueueSWOpKM(SERVER_SYNC_PRIMITIVE * psSync,
			    IMG_UINT32 * pui32FenceValue,
			    IMG_UINT32 * pui32UpdateValue);

PVRSRV_ERROR
PVRSRVServerSyncQueueHWOpKM(SERVER_SYNC_PRIMITIVE * psSync,
			    IMG_UINT32 * pui32FenceValue,
			    IMG_UINT32 * pui32UpdateValue);

IMG_BOOL
ServerSyncFenceIsMeet(SERVER_SYNC_PRIMITIVE * psSync,
		      IMG_UINT32 ui32FenceValue);

IMG_VOID
ServerSyncCompleteOp(SERVER_SYNC_PRIMITIVE * psSync,
		     IMG_UINT32 ui32UpdateValue);

PVRSRV_ERROR
PVRSRVSyncPrimOpCreateKM(IMG_UINT32 ui32SyncBlockCount,
			 SYNC_PRIMITIVE_BLOCK ** papsSyncPrimBlock,
			 IMG_UINT32 ui32ClientSyncCount,
			 IMG_UINT32 * paui32SyncBlockIndex,
			 IMG_UINT32 * paui32Index,
			 IMG_UINT32 ui32ServerSyncCount,
			 SERVER_SYNC_PRIMITIVE ** papsServerSync,
			 SERVER_OP_COOKIE ** ppsServerCookie);

PVRSRV_ERROR
PVRSRVSyncPrimOpTakeKM(SERVER_OP_COOKIE * psServerCookie,
		       IMG_UINT32 ui32ClientSyncCount,
		       IMG_UINT32 * paui32Flags,
		       IMG_UINT32 * paui32FenceValue,
		       IMG_UINT32 * paui32UpdateValue,
		       IMG_UINT32 ui32ServerSyncCount);

PVRSRV_ERROR
PVRSRVSyncPrimOpReadyKM(SERVER_OP_COOKIE * psServerCookie, IMG_BOOL * pbReady);

PVRSRV_ERROR PVRSRVSyncPrimOpCompleteKM(SERVER_OP_COOKIE * psServerCookie);

PVRSRV_ERROR PVRSRVSyncPrimOpDestroyKM(SERVER_OP_COOKIE * psServerCookie);

#if defined(PDUMP)
PVRSRV_ERROR
PVRSRVSyncPrimPDumpKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk, IMG_UINT32 ui32Offset);

PVRSRV_ERROR
PVRSRVSyncPrimPDumpPolKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk,
			 IMG_UINT32 ui32Offset, IMG_UINT32 ui32Value,
			 IMG_UINT32 ui32Mask, PDUMP_POLL_OPERATOR eOperator,
			 PDUMP_FLAGS_T uiDumpFlags);

PVRSRV_ERROR
PVRSRVSyncPrimPDumpCBPKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk,
			 IMG_UINT32 ui32Offset, IMG_UINT32 uiWriteOffset,
			 IMG_UINT32 uiPacketSize, IMG_UINT32 uiBufferSize);

#else				/* PDUMP */

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSyncPrimPDumpKM)
#endif
static INLINE PVRSRV_ERROR
PVRSRVSyncPrimPDumpKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk, IMG_UINT32 ui32Offset)
{
	PVR_UNREFERENCED_PARAMETER(psSyncBlk);
	PVR_UNREFERENCED_PARAMETER(ui32Offset);
	return PVRSRV_OK;
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSyncPrimPDumpPolKM)
#endif
static INLINE PVRSRV_ERROR
PVRSRVSyncPrimPDumpPolKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk,
			 IMG_UINT32 ui32Offset, IMG_UINT32 ui32Value,
			 IMG_UINT32 ui32Mask, PDUMP_POLL_OPERATOR eOperator,
			 PDUMP_FLAGS_T uiDumpFlags)
{
	PVR_UNREFERENCED_PARAMETER(psSyncBlk);
	PVR_UNREFERENCED_PARAMETER(ui32Offset);
	PVR_UNREFERENCED_PARAMETER(ui32Value);
	PVR_UNREFERENCED_PARAMETER(ui32Mask);
	PVR_UNREFERENCED_PARAMETER(eOperator);
	PVR_UNREFERENCED_PARAMETER(uiDumpFlags);
	return PVRSRV_OK;
}

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVSyncPrimPDumpCBPKM)
#endif
static INLINE PVRSRV_ERROR
PVRSRVSyncPrimPDumpCBPKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk,
			 IMG_UINT32 ui32Offset, IMG_UINT32 uiWriteOffset,
			 IMG_UINT32 uiPacketSize, IMG_UINT32 uiBufferSize)
{
	PVR_UNREFERENCED_PARAMETER(psSyncBlk);
	PVR_UNREFERENCED_PARAMETER(ui32Offset);
	PVR_UNREFERENCED_PARAMETER(uiWriteOffset);
	PVR_UNREFERENCED_PARAMETER(uiPacketSize);
	PVR_UNREFERENCED_PARAMETER(uiBufferSize);
	return PVRSRV_OK;
}
#endif				/* PDUMP */
#endif	/*_SYNC_SERVER_H_ */
