									    /*************************************************************************//*!
									       @File           sync_server.c
									       @Title          Server side synchronisation functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements the server side functions that for synchronisation
									       @License        Strictly Confidential.
    *//**************************************************************************/
#include "img_types.h"
#include "sync_server.h"
#include "allocmem.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "osfunc.h"
#include "pdump.h"
#include "pvr_debug.h"
#include "pdump_km.h"
#include "sync.h"
#include "sync_internal.h"

#if defined(SUPPORT_SECURE_EXPORT)
#include "ossecure_export.h"
#endif

struct _SYNC_PRIMITIVE_BLOCK_ {
	PVRSRV_DEVICE_NODE *psDevNode;
	DEVMEM_MEMDESC *psMemDesc;
	DEVMEM_EXPORTCOOKIE sExportCookie;
	IMG_UINT32 *pui32LinAddr;
	IMG_UINT32 ui32BlockSize;	/*!< Size of the Sync Primitive Block */
	IMG_UINT32 ui32RefCount;
};

struct _SERVER_SYNC_PRIMITIVE_ {
	PVRSRV_CLIENT_SYNC_PRIM *psSync;
	IMG_UINT32 ui32NextOp;
	IMG_UINT32 ui32RefCount;
	IMG_UINT32 ui32UID;
	/* PDump only data */
	IMG_BOOL bSWOperation;
	IMG_BOOL bSWOpStartedInCaptRange;
	IMG_UINT32 ui32LastHWUpdate;
	IMG_BOOL bPDumped;
};

struct _SERVER_SYNC_EXPORT_ {
	SERVER_SYNC_PRIMITIVE *psSync;
};

struct _SERVER_OP_COOKIE_ {
	IMG_BOOL bActive;
	/*
	   Client syncblock(s) info.
	   If this changes update the calculation of ui32BlockAllocSize
	 */
	IMG_UINT32 ui32SyncBlockCount;
	SYNC_PRIMITIVE_BLOCK **papsSyncPrimBlock;

	/*
	   Client sync(s) info.
	   If this changes update the calculation of ui32ClientAllocSize
	 */
	IMG_UINT32 ui32ClientSyncCount;
	IMG_UINT32 *paui32SyncBlockIndex;
	IMG_UINT32 *paui32Index;
	IMG_UINT32 *paui32Flags;
	IMG_UINT32 *paui32FenceValue;
	IMG_UINT32 *paui32UpdateValue;

	/*
	   Server sync(s) info
	   If this changes update the calculation of ui32ServerAllocSize
	 */
	IMG_UINT32 ui32ServerSyncCount;
	SERVER_SYNC_PRIMITIVE **papsServerSync;
	IMG_UINT32 *paui32ServerFenceValue;
	IMG_UINT32 *paui32ServerUpdateValue;

};

static IMG_UINT32 g_ServerSyncUID = 0;

#if defined(SYNC_DEBUG) || defined(REFCOUNT_DEBUG)
#define SYNC_REFCOUNT_PRINT(fmt, ...) PVRSRVDebugPrintf(PVR_DBG_WARNING, __FILE__, __LINE__, fmt, __VA_ARGS__)
#else
#define SYNC_REFCOUNT_PRINT(fmt, ...)
#endif

#if defined(SYNC_DEBUG)
#define SYNC_UPDATES_PRINT(fmt, ...) PVRSRVDebugPrintf(PVR_DBG_WARNING, __FILE__, __LINE__, fmt, __VA_ARGS__)
#else
#define SYNC_UPDATES_PRINT(fmt, ...)
#endif

static
IMG_VOID _SyncPrimitiveBlockRef(SYNC_PRIMITIVE_BLOCK * psSyncBlk)
{
	psSyncBlk->ui32RefCount++;
	SYNC_REFCOUNT_PRINT("%s: Sync block %p, refcount = %d",
			    __FUNCTION__, psSyncBlk, psSyncBlk->ui32RefCount);
}

static
IMG_VOID _SyncPrimitiveBlockUnref(SYNC_PRIMITIVE_BLOCK * psSyncBlk)
{
	if (--psSyncBlk->ui32RefCount == 0) {
		PVRSRV_DEVICE_NODE *psDevNode = psSyncBlk->psDevNode;

		SYNC_REFCOUNT_PRINT("%s: Sync block %p, refcount = %d (remove)",
				    __FUNCTION__, psSyncBlk,
				    psSyncBlk->ui32RefCount);

		DevmemUnexport(psSyncBlk->psMemDesc, &psSyncBlk->sExportCookie);
		DevmemReleaseCpuVirtAddr(psSyncBlk->psMemDesc);
		psDevNode->pfnFreeUFOBlock(psSyncBlk->psMemDesc);
		OSFreeMem(psSyncBlk);
	} else {
		SYNC_REFCOUNT_PRINT("%s: Sync block %p, refcount = %d",
				    __FUNCTION__, psSyncBlk,
				    psSyncBlk->ui32RefCount);
	}
}

PVRSRV_ERROR
PVRSRVAllocSyncPrimitiveBlockKM(PVRSRV_DEVICE_NODE * psDevNode,
				SYNC_PRIMITIVE_BLOCK ** ppsSyncBlk,
				IMG_UINT32 * puiSyncPrimVAddr,
				IMG_UINT32 * puiSyncPrimBlockSize,
				DEVMEM_EXPORTCOOKIE ** psExportCookie)
{
	SYNC_PRIMITIVE_BLOCK *psNewSyncBlk;
	PVRSRV_ERROR eError;

	psNewSyncBlk = OSAllocMem(sizeof(SYNC_PRIMITIVE_BLOCK));
	if (psNewSyncBlk == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}
	psNewSyncBlk->psDevNode = psDevNode;

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS, "Allocate UFO block");

	eError = psDevNode->pfnAllocUFOBlock(psDevNode,
					     &psNewSyncBlk->psMemDesc,
					     puiSyncPrimVAddr,
					     &psNewSyncBlk->ui32BlockSize);
	if (eError != PVRSRV_OK) {
		goto e1;
	}

	eError = DevmemAcquireCpuVirtAddr(psNewSyncBlk->psMemDesc,
					  (IMG_PVOID *) & psNewSyncBlk->
					  pui32LinAddr);
	if (eError != PVRSRV_OK) {
		goto e2;
	}

	eError =
	    DevmemExport(psNewSyncBlk->psMemDesc, &psNewSyncBlk->sExportCookie);
	if (eError != PVRSRV_OK) {
		goto e3;
	}

	psNewSyncBlk->ui32RefCount = 0;
	_SyncPrimitiveBlockRef(psNewSyncBlk);

	*psExportCookie = &psNewSyncBlk->sExportCookie;
	*ppsSyncBlk = psNewSyncBlk;
	*puiSyncPrimBlockSize = psNewSyncBlk->ui32BlockSize;

	PDUMPCOMMENTWITHFLAGS(PDUMP_FLAGS_CONTINUOUS,
			      "Allocated UFO block (FirmwareVAddr = 0x%08x)",
			      *puiSyncPrimVAddr);

	return PVRSRV_OK;
 e3:
	psDevNode->pfnFreeUFOBlock(psNewSyncBlk->psMemDesc);
 e2:
	DevmemReleaseCpuVirtAddr(psNewSyncBlk->psMemDesc);
 e1:
	OSFreeMem(psNewSyncBlk);
 e0:
	return eError;
}

PVRSRV_ERROR PVRSRVFreeSyncPrimitiveBlockKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk)
{
	_SyncPrimitiveBlockUnref(psSyncBlk);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVSyncPrimSetKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk, IMG_UINT32 ui32Index,
		    IMG_UINT32 ui32Value)
{
	psSyncBlk->pui32LinAddr[ui32Index] = ui32Value;

	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVServerSyncPrimSetKM(SERVER_SYNC_PRIMITIVE * psServerSync,
			  IMG_UINT32 ui32Value)
{
	*psServerSync->psSync->pui32LinAddr = ui32Value;

	return PVRSRV_OK;
}

static
IMG_VOID _ServerSyncRef(SERVER_SYNC_PRIMITIVE * psSync)
{
	psSync->ui32RefCount++;
	SYNC_REFCOUNT_PRINT("%s: Server sync %p, refcount = %d",
			    __FUNCTION__, psSync, psSync->ui32RefCount);
}

PVRSRV_ERROR
PVRSRVServerSyncAllocKM(PVRSRV_DEVICE_NODE * psDevNode,
			SERVER_SYNC_PRIMITIVE ** ppsSync,
			IMG_UINT32 * pui32SyncPrimVAddr)
{
	SERVER_SYNC_PRIMITIVE *psNewSync;
	PVRSRV_ERROR eError;

	psNewSync = OSAllocMem(sizeof(SERVER_SYNC_PRIMITIVE));
	if (psNewSync == IMG_NULL) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	eError = SyncPrimAlloc(psDevNode->hSyncPrimContext, &psNewSync->psSync);
	if (eError != PVRSRV_OK) {
		goto fail_sync_alloc;
	}

	SyncPrimSet(psNewSync->psSync, 0);

	psNewSync->ui32NextOp = 0;
	psNewSync->ui32RefCount = 0;
	psNewSync->ui32UID = g_ServerSyncUID++;
	psNewSync->bSWOperation = IMG_FALSE;
	psNewSync->ui32LastHWUpdate = 0x0bad592c;
	psNewSync->bPDumped = IMG_FALSE;
	_ServerSyncRef(psNewSync);
	*pui32SyncPrimVAddr = SyncPrimGetFirmwareAddr(psNewSync->psSync);
	SYNC_UPDATES_PRINT("%s: sync: %p, fwaddr: %8.8X", __FUNCTION__,
			   psNewSync, *pui32SyncPrimVAddr);
	*ppsSync = psNewSync;
	return PVRSRV_OK;

 fail_sync_alloc:
	OSFreeMem(psNewSync);
	return eError;
}

static
IMG_VOID _ServerSyncUnref(SERVER_SYNC_PRIMITIVE * psSync)
{
	if (--psSync->ui32RefCount == 0) {
		SYNC_REFCOUNT_PRINT("%s: Server sync %p, refcount = %d",
				    __FUNCTION__, psSync, psSync->ui32RefCount);
		SyncPrimFree(psSync->psSync);
		OSFreeMem(psSync);
	} else {
		SYNC_REFCOUNT_PRINT("%s: Server sync %p, refcount = %d",
				    __FUNCTION__, psSync, psSync->ui32RefCount);
	}
}

PVRSRV_ERROR PVRSRVServerSyncFreeKM(SERVER_SYNC_PRIMITIVE * psSync)
{
	_ServerSyncUnref(psSync);
	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVServerSyncGetStatusKM(IMG_UINT32 ui32SyncCount,
			    SERVER_SYNC_PRIMITIVE ** papsSyncs,
			    IMG_UINT32 * pui32UID,
			    IMG_UINT32 * pui32FWAddr,
			    IMG_UINT32 * pui32CurrentOp,
			    IMG_UINT32 * pui32NextOp)
{
	IMG_UINT32 i;

	for (i = 0; i < ui32SyncCount; i++) {
		PVRSRV_CLIENT_SYNC_PRIM *psClientSync = papsSyncs[i]->psSync;

		pui32UID[i] = papsSyncs[i]->ui32UID;
		pui32FWAddr[i] = SyncPrimGetFirmwareAddr(psClientSync);
		pui32CurrentOp[i] = *psClientSync->pui32LinAddr;
		pui32NextOp[i] = papsSyncs[i]->ui32NextOp;
	}
	return PVRSRV_OK;
}

static PVRSRV_ERROR
_PVRSRVSyncPrimServerExportKM(SERVER_SYNC_PRIMITIVE * psSync,
			      SERVER_SYNC_EXPORT ** ppsExport)
{
	SERVER_SYNC_EXPORT *psNewExport;
	PVRSRV_ERROR eError;

	psNewExport = OSAllocMem(sizeof(SERVER_SYNC_EXPORT));
	if (!psNewExport) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	/* FIXME: Lock */
	_ServerSyncRef(psSync);
	/* FIXME: Unlock */
	psNewExport->psSync = psSync;
	*ppsExport = psNewExport;

	return PVRSRV_OK;
 e0:
	return eError;
}

static PVRSRV_ERROR
_PVRSRVSyncPrimServerUnexportKM(SERVER_SYNC_EXPORT * psExport)
{
	/* FIXME: Lock */
	_ServerSyncUnref(psExport->psSync);
	/* FIXME: Unlock */

	OSFreeMem(psExport);

	return PVRSRV_OK;
}

static IMG_VOID
_PVRSRVSyncPrimServerImportKM(SERVER_SYNC_EXPORT * psExport,
			      SERVER_SYNC_PRIMITIVE ** ppsSync,
			      IMG_UINT32 * pui32SyncPrimVAddr)
{
	/* FIXME: Lock */
	_ServerSyncRef(psExport->psSync);
	/* FIXME: Unlock */

	*ppsSync = psExport->psSync;
	*pui32SyncPrimVAddr = SyncPrimGetFirmwareAddr(psExport->psSync->psSync);
}

#if !defined(SUPPORT_SECURE_EXPORT)
PVRSRV_ERROR
PVRSRVSyncPrimServerExportKM(SERVER_SYNC_PRIMITIVE * psSync,
			     SERVER_SYNC_EXPORT ** ppsExport)
{
	return _PVRSRVSyncPrimServerExportKM(psSync, ppsExport);
}

PVRSRV_ERROR PVRSRVSyncPrimServerUnexportKM(SERVER_SYNC_EXPORT * psExport)
{
	return _PVRSRVSyncPrimServerUnexportKM(psExport);
}

PVRSRV_ERROR
PVRSRVSyncPrimServerImportKM(SERVER_SYNC_EXPORT * psExport,
			     SERVER_SYNC_PRIMITIVE ** ppsSync,
			     IMG_UINT32 * pui32SyncPrimVAddr)
{
	_PVRSRVSyncPrimServerImportKM(psExport, ppsSync, pui32SyncPrimVAddr);

	return PVRSRV_OK;
}

#else				/* SUPPORT_SECURE_EXPORT */
PVRSRV_ERROR
PVRSRVSyncPrimServerExportKM(CONNECTION_DATA * psConnection,
			     SERVER_SYNC_PRIMITIVE * psSync,
			     IMG_SECURE_TYPE * phSecure,
			     SERVER_SYNC_EXPORT ** ppsExport,
			     CONNECTION_DATA ** ppsSecureConnection)
{
	SERVER_SYNC_EXPORT *psNewExport;
	PVRSRV_ERROR eError;

	/* Create an export server sync */
	eError = _PVRSRVSyncPrimServerExportKM(psSync, &psNewExport);

	if (eError != PVRSRV_OK) {
		goto e0;
	}

	/* Transform it into a secure export */
	eError = OSSecureExport(psConnection,
				(IMG_PVOID) psNewExport,
				phSecure, ppsSecureConnection);
	if (eError != PVRSRV_OK) {
		goto e1;
	}

	*ppsExport = psNewExport;
	return PVRSRV_OK;
 e1:
	_PVRSRVSyncPrimServerUnexportKM(psNewExport);
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR PVRSRVSyncPrimServerUnexportKM(SERVER_SYNC_EXPORT * psExport)
{
	_PVRSRVSyncPrimServerUnexportKM(psExport);
	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVSyncPrimServerImportKM(IMG_SECURE_TYPE hSecure,
			     SERVER_SYNC_PRIMITIVE ** ppsSync,
			     IMG_UINT32 * pui32SyncPrimVAddr)
{
	PVRSRV_ERROR eError;
	SERVER_SYNC_EXPORT *psImport;

	/* Retrieve the data from the secure import */
	eError = OSSecureImport(hSecure, (IMG_PVOID *) & psImport);
	if (eError != PVRSRV_OK) {
		goto e0;
	}

	_PVRSRVSyncPrimServerImportKM(psImport, ppsSync, pui32SyncPrimVAddr);
	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}
#endif				/* SUPPORT_SECURE_EXPORT */

static IMG_VOID
_ServerSyncTakeOperation(SERVER_SYNC_PRIMITIVE * psSync,
			 IMG_UINT32 * pui32FenceValue,
			 IMG_UINT32 * pui32UpdateValue)
{
	IMG_BOOL bInCaptureRange;

	*pui32FenceValue = psSync->ui32NextOp++;
	*pui32UpdateValue = psSync->ui32NextOp;

	PDumpIsCaptureFrameKM(&bInCaptureRange);
	/*
	   If this is the 1st operation (in this capture range) then PDump
	   this sync
	 */
	if (!psSync->bPDumped && bInCaptureRange) {
		IMG_CHAR azTmp[100];
		OSSNPrintf(azTmp,
			   sizeof(azTmp),
			   "Dump initial sync state (%p, FW VAddr = 0x%08x) = 0x%08x",
			   psSync,
			   SyncPrimGetFirmwareAddr(psSync->psSync),
			   *psSync->psSync->pui32LinAddr);
		PDumpCommentKM(azTmp, 0);

		SyncPrimPDump(psSync->psSync);
		psSync->bPDumped = IMG_TRUE;
	}

	/*
	   When exiting capture range clear down bPDumped as we might re-enter
	   capture range and thus need to PDump this sync again
	 */
	if (!bInCaptureRange) {
		psSync->bPDumped = IMG_FALSE;
	}
}

PVRSRV_ERROR
PVRSRVServerSyncQueueSWOpKM(SERVER_SYNC_PRIMITIVE * psSync,
			    IMG_UINT32 * pui32FenceValue,
			    IMG_UINT32 * pui32UpdateValue)
{
	/* FIXME: Lock */
	_ServerSyncRef(psSync);
	_ServerSyncTakeOperation(psSync, pui32FenceValue, pui32UpdateValue);

	/*
	   If we're transitioning from a HW operation to a SW operation we
	   need to save the last update the HW will do so that when we PDump
	   we can issue a POL for it before the next HW operation and then
	   LDB in the last SW fence update
	 */
	if (psSync->bSWOperation == IMG_FALSE) {
		psSync->bSWOperation = IMG_TRUE;
		psSync->ui32LastHWUpdate = *pui32FenceValue;
		PDumpIsCaptureFrameKM(&psSync->bSWOpStartedInCaptRange);
	}
	SYNC_UPDATES_PRINT("%s: sync: %p, fence: %d, value: %d", __FUNCTION__,
			   psSync, *pui32FenceValue, *pui32UpdateValue);
	/* FIXME: Unlock */

	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVServerSyncQueueHWOpKM(SERVER_SYNC_PRIMITIVE * psSync,
			    IMG_UINT32 * pui32FenceValue,
			    IMG_UINT32 * pui32UpdateValue)
{
	/* FIXME: Lock */
	_ServerSyncTakeOperation(psSync, pui32FenceValue, pui32UpdateValue);
	/* FIXME: Unlock */

	if (psSync->bSWOperation) {
		IMG_CHAR azTmp[100];
		OSSNPrintf(azTmp,
			   sizeof(azTmp),
			   "Dummy update for SW operations (%p, FW VAddr = 0x%08x)",
			   psSync, SyncPrimGetFirmwareAddr(psSync->psSync));
		PDumpCommentKM(azTmp, 0);

		if (psSync->bSWOpStartedInCaptRange) {
			/* Dump a POL for the previous HW operation */
			SyncPrimPDumpPol(psSync->psSync,
					 psSync->ui32LastHWUpdate,
					 0xffffffff,
					 PDUMP_POLL_OPERATOR_EQUAL, 0);
		}

		/* Dump the current value (i.e. the value after all the SW operations) */
		SyncPrimPDump(psSync->psSync);

		/* Reset the state as we've just done a HW operation */
		psSync->bSWOperation = IMG_FALSE;
	}

	SYNC_UPDATES_PRINT("%s: sync: %p, fence: %d, value: %d", __FUNCTION__,
			   psSync, *pui32FenceValue, *pui32UpdateValue);

	return PVRSRV_OK;
}

IMG_BOOL ServerSyncFenceIsMeet(SERVER_SYNC_PRIMITIVE * psSync,
			       IMG_UINT32 ui32FenceValue)
{
	SYNC_UPDATES_PRINT("%s: sync: %p, value(%d) == fence(%d)?",
			   __FUNCTION__, psSync, *psSync->psSync->pui32LinAddr,
			   ui32FenceValue);
	return (*psSync->psSync->pui32LinAddr == ui32FenceValue);
}

IMG_VOID
ServerSyncCompleteOp(SERVER_SYNC_PRIMITIVE * psSync, IMG_UINT32 ui32UpdateValue)
{
	SYNC_UPDATES_PRINT("%s: sync: %p (%d) = %d", __FUNCTION__, psSync,
			   *psSync->psSync->pui32LinAddr, ui32UpdateValue);
	*psSync->psSync->pui32LinAddr = ui32UpdateValue;
	_ServerSyncUnref(psSync);
}

PVRSRV_ERROR
PVRSRVSyncPrimOpCreateKM(IMG_UINT32 ui32SyncBlockCount,
			 SYNC_PRIMITIVE_BLOCK ** papsSyncPrimBlock,
			 IMG_UINT32 ui32ClientSyncCount,
			 IMG_UINT32 * paui32SyncBlockIndex,
			 IMG_UINT32 * paui32Index,
			 IMG_UINT32 ui32ServerSyncCount,
			 SERVER_SYNC_PRIMITIVE ** papsServerSync,
			 SERVER_OP_COOKIE ** ppsServerCookie)
{
	SERVER_OP_COOKIE *psNewCookie;
	IMG_UINT32 ui32BlockAllocSize;
	IMG_UINT32 ui32ServerAllocSize;
	IMG_UINT32 ui32ClientAllocSize;
	IMG_UINT32 ui32TotalAllocSize;
	IMG_UINT32 i;
	IMG_CHAR *pcPtr;
	PVRSRV_ERROR eError;

	/* Allocate space for all the sync block list */
	ui32BlockAllocSize =
	    ui32SyncBlockCount * (sizeof(SYNC_PRIMITIVE_BLOCK *));

	/* Allocate space for all the client sync size elements */
	ui32ClientAllocSize = ui32ClientSyncCount * (5 * sizeof(IMG_UINT32));

	/* Allocate space for all the server sync size elements */
	ui32ServerAllocSize =
	    ui32ServerSyncCount * (sizeof(SERVER_SYNC_PRIMITIVE *)
				   + (2 * sizeof(IMG_UINT32)));

	ui32TotalAllocSize = sizeof(SERVER_OP_COOKIE) +
	    ui32BlockAllocSize + ui32ServerAllocSize + ui32ClientAllocSize;

	psNewCookie = OSAllocMem(ui32TotalAllocSize);
	pcPtr = (IMG_CHAR *) psNewCookie;

	if (!psNewCookie) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}
	OSMemSet(psNewCookie, 0, ui32TotalAllocSize);

	/* Setup the pointers */
	pcPtr += sizeof(SERVER_OP_COOKIE);
	psNewCookie->papsSyncPrimBlock = (SYNC_PRIMITIVE_BLOCK **) pcPtr;

	pcPtr += sizeof(SYNC_PRIMITIVE_BLOCK *) * ui32SyncBlockCount;
	psNewCookie->paui32SyncBlockIndex = (IMG_UINT32 *) pcPtr;

	pcPtr += sizeof(IMG_UINT32) * ui32ClientSyncCount;
	psNewCookie->paui32Index = (IMG_UINT32 *) pcPtr;

	pcPtr += sizeof(IMG_UINT32) * ui32ClientSyncCount;
	psNewCookie->paui32Flags = (IMG_UINT32 *) pcPtr;

	pcPtr += sizeof(IMG_UINT32) * ui32ClientSyncCount;
	psNewCookie->paui32FenceValue = (IMG_UINT32 *) pcPtr;

	pcPtr += sizeof(IMG_UINT32) * ui32ClientSyncCount;
	psNewCookie->paui32UpdateValue = (IMG_UINT32 *) pcPtr;

	pcPtr += sizeof(IMG_UINT32) * ui32ClientSyncCount;
	psNewCookie->papsServerSync = (SERVER_SYNC_PRIMITIVE **) pcPtr;

	pcPtr += sizeof(SERVER_SYNC_PRIMITIVE *) * ui32ServerSyncCount;
	psNewCookie->paui32ServerFenceValue = (IMG_UINT32 *) pcPtr;

	pcPtr += sizeof(IMG_UINT32) * ui32ServerSyncCount;
	psNewCookie->paui32ServerUpdateValue = (IMG_UINT32 *) pcPtr;

	pcPtr += sizeof(IMG_UINT32) * ui32ServerSyncCount;

	/* Check the pointer setup went ok */
	PVR_ASSERT(pcPtr == (((IMG_CHAR *) psNewCookie) + ui32TotalAllocSize));

	psNewCookie->ui32SyncBlockCount = ui32SyncBlockCount;
	psNewCookie->ui32ServerSyncCount = ui32ServerSyncCount;
	psNewCookie->ui32ClientSyncCount = ui32ClientSyncCount;
	psNewCookie->bActive = IMG_FALSE;

	/* Copy all the data into our server coockie */
	OSMemCopy(psNewCookie->papsSyncPrimBlock,
		  papsSyncPrimBlock,
		  sizeof(SYNC_PRIMITIVE_BLOCK *) * ui32SyncBlockCount);

	OSMemCopy(psNewCookie->paui32SyncBlockIndex,
		  paui32SyncBlockIndex,
		  sizeof(IMG_UINT32) * ui32ClientSyncCount);
	OSMemCopy(psNewCookie->paui32Index,
		  paui32Index, sizeof(IMG_UINT32) * ui32ClientSyncCount);

	OSMemCopy(psNewCookie->papsServerSync,
		  papsServerSync,
		  sizeof(SERVER_SYNC_PRIMITIVE *) * ui32ServerSyncCount);

	/*
	   Take a reference on all the sync blocks and server syncs so they can't
	   be freed while we're using them
	 */
	for (i = 0; i < ui32SyncBlockCount; i++) {
		_SyncPrimitiveBlockRef(psNewCookie->papsSyncPrimBlock[i]);
	}

	for (i = 0; i < ui32ServerSyncCount; i++) {
		_ServerSyncRef(psNewCookie->papsServerSync[i]);
	}

	*ppsServerCookie = psNewCookie;
	return PVRSRV_OK;

 e0:
	return eError;
}

PVRSRV_ERROR
PVRSRVSyncPrimOpTakeKM(SERVER_OP_COOKIE * psServerCookie,
		       IMG_UINT32 ui32ClientSyncCount,
		       IMG_UINT32 * paui32Flags,
		       IMG_UINT32 * paui32FenceValue,
		       IMG_UINT32 * paui32UpdateValue,
		       IMG_UINT32 ui32ServerSyncCount)
{
	IMG_UINT32 i;

	if ((ui32ClientSyncCount != psServerCookie->ui32ClientSyncCount) ||
	    (ui32ServerSyncCount != psServerCookie->ui32ServerSyncCount)) {
		/* The bridge layer should have stopped us getting here but check incase */
		PVR_DPF((PVR_DBG_ERROR, "%s: Invalid sync counts",
			 __FUNCTION__));
		return PVRSRV_ERROR_INVALID_PARAMS;
	}

	/*
	   For client syncs all we need to do is save the values
	   that we've been passed
	 */
	OSMemCopy(psServerCookie->paui32Flags,
		  paui32Flags, sizeof(IMG_UINT32) * ui32ClientSyncCount);
	OSMemCopy(psServerCookie->paui32FenceValue,
		  paui32FenceValue, sizeof(IMG_UINT32) * ui32ClientSyncCount);
	OSMemCopy(psServerCookie->paui32UpdateValue,
		  paui32UpdateValue, sizeof(IMG_UINT32) * ui32ClientSyncCount);

	/*
	   For server syncs we just take an operation
	 */
	for (i = 0; i < ui32ServerSyncCount; i++) {
		PVRSRVServerSyncQueueSWOpKM(psServerCookie->papsServerSync[i],
					    &psServerCookie->
					    paui32ServerFenceValue[i],
					    &psServerCookie->
					    paui32ServerUpdateValue[i]);
	}

	psServerCookie->bActive = IMG_TRUE;
	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVSyncPrimOpReadyKM(SERVER_OP_COOKIE * psServerCookie, IMG_BOOL * pbReady)
{
	IMG_UINT32 i;
	IMG_BOOL bReady = IMG_TRUE;

	/* Check the client syncs */
	for (i = 0; i < psServerCookie->ui32ClientSyncCount; i++) {
		if (psServerCookie->
		    paui32Flags[i] & PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK) {
			IMG_UINT32 ui32BlockIndex =
			    psServerCookie->paui32SyncBlockIndex[i];
			IMG_UINT32 ui32Index = psServerCookie->paui32Index[i];
			SYNC_PRIMITIVE_BLOCK *psSyncBlock =
			    psServerCookie->papsSyncPrimBlock[ui32BlockIndex];

			if (psSyncBlock->pui32LinAddr[ui32Index] !=
			    psServerCookie->paui32FenceValue[i]) {
				bReady = IMG_FALSE;
				goto e0;
			}
		}
	}

	for (i = 0; i < psServerCookie->ui32ServerSyncCount; i++) {
		bReady =
		    ServerSyncFenceIsMeet(psServerCookie->papsServerSync[i],
					  psServerCookie->
					  paui32ServerFenceValue[i]);
		if (!bReady) {
			break;
		}
	}

 e0:
	*pbReady = bReady;
	return PVRSRV_OK;
}

static
PVRSRV_ERROR _SyncPrimOpComplete(SERVER_OP_COOKIE * psServerCookie)
{
	IMG_UINT32 i;

	for (i = 0; i < psServerCookie->ui32ClientSyncCount; i++) {
		if (psServerCookie->
		    paui32Flags[i] & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE) {
			IMG_UINT32 ui32BlockIndex =
			    psServerCookie->paui32SyncBlockIndex[i];
			IMG_UINT32 ui32Index = psServerCookie->paui32Index[i];
			SYNC_PRIMITIVE_BLOCK *psSyncBlock =
			    psServerCookie->papsSyncPrimBlock[ui32BlockIndex];

			psSyncBlock->pui32LinAddr[ui32Index] =
			    psServerCookie->paui32UpdateValue[i];
		}
	}

	for (i = 0; i < psServerCookie->ui32ServerSyncCount; i++) {
		ServerSyncCompleteOp(psServerCookie->papsServerSync[i],
				     psServerCookie->
				     paui32ServerUpdateValue[i]);
	}

	psServerCookie->bActive = IMG_FALSE;
	return PVRSRV_OK;
}

PVRSRV_ERROR PVRSRVSyncPrimOpCompleteKM(SERVER_OP_COOKIE * psServerCookie)
{
	IMG_BOOL bReady;

	PVRSRVSyncPrimOpReadyKM(psServerCookie, &bReady);

	/* Check the client is playing ball */
	if (!bReady) {
		PVR_DPF((PVR_DBG_ERROR, "%s: sync op still not ready",
			 __FUNCTION__));

		return PVRSRV_ERROR_BAD_SYNC_STATE;
	}

	return _SyncPrimOpComplete(psServerCookie);
}

PVRSRV_ERROR PVRSRVSyncPrimOpDestroyKM(SERVER_OP_COOKIE * psServerCookie)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 i;

	if (psServerCookie->bActive) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: Fatal error, shouldn't free cookie while it's active",
			 __FUNCTION__));

		/* FIXME: Wait for the operation to complete instead of compliting it here */
		_SyncPrimOpComplete(psServerCookie);
	}

	/* Drop our references on the sync blocks and server syncs */
	for (i = 0; i < psServerCookie->ui32SyncBlockCount; i++) {
		_SyncPrimitiveBlockUnref(psServerCookie->papsSyncPrimBlock[i]);
	}

	for (i = 0; i < psServerCookie->ui32ServerSyncCount; i++) {
		_ServerSyncUnref(psServerCookie->papsServerSync[i]);
	}

	OSFreeMem(psServerCookie);
	return PVRSRV_OK;

	return eError;
}

#if defined(PDUMP)
PVRSRV_ERROR
PVRSRVSyncPrimPDumpKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk, IMG_UINT32 ui32Offset)
{
	/*
	   We might be ask to PDump sync state outside of capture range
	   (e.g. texture uploads) so make this continuous.
	 */
	DevmemPDumpLoadMem(psSyncBlk->psMemDesc,
			   ui32Offset,
			   sizeof(IMG_UINT32), PDUMP_FLAGS_CONTINUOUS);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVSyncPrimPDumpPolKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk,
			 IMG_UINT32 ui32Offset, IMG_UINT32 ui32Value,
			 IMG_UINT32 ui32Mask, PDUMP_POLL_OPERATOR eOperator,
			 PDUMP_FLAGS_T ui32PDumpFlags)
{
	DevmemPDumpDevmemPol32(psSyncBlk->psMemDesc,
			       ui32Offset,
			       ui32Value, ui32Mask, eOperator, ui32PDumpFlags);

	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVSyncPrimPDumpCBPKM(SYNC_PRIMITIVE_BLOCK * psSyncBlk,
			 IMG_UINT32 ui32Offset, IMG_UINT32 uiWriteOffset,
			 IMG_UINT32 uiPacketSize, IMG_UINT32 uiBufferSize)
{
	DevmemPDumpCBP(psSyncBlk->psMemDesc,
		       ui32Offset, uiWriteOffset, uiPacketSize, uiBufferSize);
	return PVRSRV_OK;
}
#endif
