									     /**************************************************************************//*!
									        @File           sync.c
									        @Title          Services synchronisation interface
									        @Date
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Implements client side code for services synchronisation
									        interface
    *//***************************************************************************/

#include "img_types.h"
#include "client_sync_bridge.h"
#if !defined(__KERNEL__)	/* FIXME: split in client only and shared */
#if !defined(SUPPORT_SECURE_EXPORT)
#include "client_syncexport_bridge.h"
#else
#include "client_syncsexport_bridge.h"
#endif
#endif
#include "allocmem.h"
#include "osfunc.h"
#include "devicemem.h"
#include "devicemem_pdump.h"
#include "ra.h"
#include "pvr_debug.h"
#include "dllist.h"
#include "sync.h"
#include "sync_internal.h"
/* FIXME */
#if defined(__KERNEL__)
#include "pvrsrv.h"
#endif

/*
	Private structure's
*/
#define SYNC_PRIM_NAME_SIZE		50
typedef struct _SYNC_PRIM_CONTEXT_ {
	SYNC_BRIDGE_HANDLE hBridge;	/*!< Bridge handle */
	IMG_HANDLE hDeviceNode;	/*!< The device we're operating on */
	IMG_CHAR azName[SYNC_PRIM_NAME_SIZE];	/*!< Name of the RA */
	RA_ARENA *psRA;		/*!< RA context */
	struct _SYNC_PRIM_BLOCK_ *psSyncBlock;	/*!< Block allocation */
	IMG_UINT32 ui32RefCount;	/*!< Refcount for this context */
	DLLIST_NODE sListNode;	/*!< Listnode for per-process context list */
} SYNC_PRIM_CONTEXT;

typedef struct _SYNC_PRIM_BLOCK_ {
	SYNC_PRIM_CONTEXT *psContext;	/*!< Our copy of the services connection */
	IMG_HANDLE hServerSyncPrimBlock;	/*!< Server handle for this block */
	IMG_UINT32 ui32SyncBlockSize;	/*!< Size of the sync prim block */
	IMG_UINT32 ui32FirmwareAddr;	/*!< Firmware address */
	DEVMEM_MEMDESC *hMemDesc;	/*!< Host mapping handle */
	IMG_UINT32 *pui32LinAddr;	/*!< User CPU mapping */
} SYNC_PRIM_BLOCK;

typedef enum _SYNC_PRIM_TYPE_ {
	SYNC_PRIM_TYPE_UNKNOWN = 0,
	SYNC_PRIM_TYPE_LOCAL,
	SYNC_PRIM_TYPE_SERVER,
} SYNC_PRIM_TYPE;

typedef struct _SYNC_PRIM_LOCAL_ {
	SYNC_PRIM_BLOCK *psSyncBlock;	/*!< Synchronisation block this primitive is allocated on */
	IMG_UINT32 ui32Index;	/*!< Index into Synchronisation block */
} SYNC_PRIM_LOCAL;

typedef struct _SYNC_PRIM_SERVER_ {
	SYNC_BRIDGE_HANDLE hBridge;	/*!< Bridge handle */
	IMG_HANDLE hServerSync;	/*!< Handle to the server sync */
	IMG_UINT32 ui32FirmwareAddr;	/*!< Firmware address of the sync */
} SYNC_PRIM_SERVER;

typedef struct _SYNC_PRIM_ {
	PVRSRV_CLIENT_SYNC_PRIM sCommon;	/*!< Client visible part of the sync prim */
	SYNC_PRIM_TYPE eType;	/*!< Sync primative type */
	union {
		SYNC_PRIM_LOCAL sLocal;	/*!< Local sync primative data */
		SYNC_PRIM_SERVER sServer;	/*!< Server sync primative data */
	} u;
} SYNC_PRIM;

#define SYNC_BLOCK_LIST_CHUNCK_SIZE	10

typedef struct _SYNC_BLOCK_LIST_ {
	IMG_UINT32 ui32BlockCount;	/*!< Number of contexts in the list */
	IMG_UINT32 ui32BlockListSize;	/*!< Size of the array contexts */
	SYNC_PRIM_BLOCK **papsSyncPrimBlock;	/*!< Array of syncprim blocks */
} SYNC_BLOCK_LIST;

typedef struct _SYNC_OP_COOKIE_ {
	IMG_UINT32 ui32SyncCount;
	IMG_UINT32 ui32ClientSyncCount;
	IMG_UINT32 ui32ServerSyncCount;
	IMG_BOOL bHaveServerSync;
	IMG_HANDLE hBridge;
	IMG_HANDLE hServerCookie;

	SYNC_BLOCK_LIST *psSyncBlockList;
	PVRSRV_CLIENT_SYNC_PRIM **papsSyncPrim;
	/*
	   Client sync(s) info.
	   If this changes update the calculation of ui32ClientAllocSize
	 */
	IMG_UINT32 *paui32SyncBlockIndex;
	IMG_UINT32 *paui32Index;
	IMG_UINT32 *paui32Flags;
	IMG_UINT32 *paui32FenceValue;
	IMG_UINT32 *paui32UpdateValue;

	/*
	   Server sync(s) info
	   If this changes update the calculation of ui32ServerAllocSize
	 */
	IMG_HANDLE *pahServerSync;
} SYNC_OP_COOKIE;

static DECLARE_DLLIST(g_sContextList);

/*
	Internal interfaces for management of synchronisation block memory
*/
static PVRSRV_ERROR
AllocSyncPrimitiveBlock(SYNC_PRIM_CONTEXT * psContext,
			SYNC_PRIM_BLOCK ** ppsSyncBlock)
{
	SYNC_PRIM_BLOCK *psSyncBlk;
	DEVMEM_SERVER_EXPORTCOOKIE hServerExportCookie;
	DEVMEM_EXPORTCOOKIE sExportCookie;
	PVRSRV_ERROR eError;

	psSyncBlk = OSAllocMem(sizeof(SYNC_PRIM_BLOCK));
	if (psSyncBlk == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}
	psSyncBlk->psContext = psContext;

	/* Allocate sync prim block */
	eError = BridgeAllocSyncPrimitiveBlock(psContext->hBridge,
					       psContext->hDeviceNode,
					       &psSyncBlk->hServerSyncPrimBlock,
					       &psSyncBlk->ui32FirmwareAddr,
					       &psSyncBlk->ui32SyncBlockSize,
					       &hServerExportCookie);
	if (eError != PVRSRV_OK) {
		goto e1;
	}

	/* Make it mappabled by the client */
	eError = DevmemMakeServerExportClientExport(psContext->hBridge,
						    hServerExportCookie,
						    &sExportCookie);
	if (eError != PVRSRV_OK) {
		goto e2;
	}

	/* Get CPU mapping of the memory block */
	eError = DevmemImport(psContext->hBridge,
			      &sExportCookie,
			      PVRSRV_MEMALLOCFLAG_CPU_READABLE,
			      &psSyncBlk->hMemDesc);
	if (eError != PVRSRV_OK) {
		goto e2;
	}

	eError = DevmemAcquireCpuVirtAddr(psSyncBlk->hMemDesc,
					  (IMG_PVOID *) & psSyncBlk->
					  pui32LinAddr);
	if (eError != PVRSRV_OK) {
		goto e3;
	}

	*ppsSyncBlock = psSyncBlk;
	return PVRSRV_OK;

 e3:
	DevmemFree(psSyncBlk->hMemDesc);
 e2:
	BridgeFreeSyncPrimitiveBlock(psContext->hBridge,
				     psSyncBlk->hServerSyncPrimBlock);
 e1:
	OSFreeMem(psSyncBlk);
 e0:
	return eError;
}

static IMG_VOID FreeSyncPrimitiveBlock(SYNC_PRIM_BLOCK * psSyncBlk)
{
	SYNC_PRIM_CONTEXT *psContext = psSyncBlk->psContext;

	if (psSyncBlk == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR, "FreeSyncPrimitiveBlock: NULL handle"));
		return;
	}

	DevmemReleaseCpuVirtAddr(psSyncBlk->hMemDesc);
	DevmemFree(psSyncBlk->hMemDesc);
	BridgeFreeSyncPrimitiveBlock(psContext->hBridge,
				     psSyncBlk->hServerSyncPrimBlock);
}

static IMG_VOID SyncPrimGetCPULinAddr(SYNC_PRIM * psSync)
{
	SYNC_PRIM_BLOCK *psSyncBlock = psSync->u.sLocal.psSyncBlock;

	psSync->sCommon.pui32LinAddr = psSyncBlock->pui32LinAddr +
	    psSync->u.sLocal.ui32Index;
}

static IMG_VOID SyncPrimLocalFree(SYNC_PRIM * psSyncInt)
{
	SYNC_PRIM_BLOCK *psSyncBlock;
	SYNC_PRIM_CONTEXT *psContext;

	PVR_ASSERT(psSyncInt->eType == SYNC_PRIM_TYPE_LOCAL);
	psSyncBlock = psSyncInt->u.sLocal.psSyncBlock;
	psContext = psSyncBlock->psContext;

	RA_Free(psContext->psRA,
		psSyncInt->u.sLocal.ui32Index * sizeof(IMG_UINT32));
	OSFreeMem(psSyncInt);
	psContext->ui32RefCount--;
}

static IMG_VOID SyncPrimServerFree(SYNC_PRIM * psSyncInt)
{
	PVRSRV_ERROR eError;

	eError = BridgeServerSyncFree(psSyncInt->u.sServer.hBridge,
				      psSyncInt->u.sServer.hServerSync);
	if (eError != PVRSRV_OK) {
		/* Doesn't matter if the free fails as resman will cleanup */
		PVR_DPF((PVR_DBG_ERROR, "SyncPrimServerFree failed"));
	}
}

static IMG_UINT32 SyncPrimGetFirmwareAddrLocal(SYNC_PRIM * psSyncInt)
{
	SYNC_PRIM_BLOCK *psSyncBlock;

	psSyncBlock = psSyncInt->u.sLocal.psSyncBlock;
	return psSyncBlock->ui32FirmwareAddr +
	    (psSyncInt->u.sLocal.ui32Index * sizeof(IMG_UINT32));
}

static IMG_UINT32 SyncPrimGetFirmwareAddrServer(SYNC_PRIM * psSyncInt)
{
	return psSyncInt->u.sServer.ui32FirmwareAddr;
}

IMG_INTERNAL IMG_UINT32 SyncPrimGetFirmwareAddr(PVRSRV_CLIENT_SYNC_PRIM *
						psSync)
{
	SYNC_PRIM *psSyncInt;
	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	if (psSyncInt->eType == SYNC_PRIM_TYPE_LOCAL) {
		return SyncPrimGetFirmwareAddrLocal(psSyncInt);
	} else if (psSyncInt->eType == SYNC_PRIM_TYPE_SERVER) {
		return SyncPrimGetFirmwareAddrServer(psSyncInt);
	} else {
		PVR_DPF((PVR_DBG_ERROR,
			 "SyncPrimGetFirmwareAddr: Invalid sync type"));
		/*
		   Either the client has given us a bad pointer or there is an
		   error in this module
		 */
		PVR_ASSERT(IMG_FALSE);
		return 0;
	}
}

#if !defined(__KERNEL__)
static SYNC_BRIDGE_HANDLE _SyncPrimGetBridgeHandleLocal(SYNC_PRIM * psSyncInt)
{
	return psSyncInt->u.sLocal.psSyncBlock->psContext->hBridge;
}

static SYNC_BRIDGE_HANDLE _SyncPrimGetBridgeHandleServer(SYNC_PRIM * psSyncInt)
{
	return psSyncInt->u.sServer.hBridge;
}

static SYNC_BRIDGE_HANDLE _SyncPrimGetBridgeHandle(PVRSRV_CLIENT_SYNC_PRIM *
						   psSync)
{
	SYNC_PRIM *psSyncInt;
	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	if (psSyncInt->eType == SYNC_PRIM_TYPE_LOCAL) {
		return _SyncPrimGetBridgeHandleLocal(psSyncInt);
	} else if (psSyncInt->eType == SYNC_PRIM_TYPE_SERVER) {
		return _SyncPrimGetBridgeHandleServer(psSyncInt);
	} else {
		PVR_DPF((PVR_DBG_ERROR,
			 "_SyncPrimGetBridgeHandle: Invalid sync type"));
		/*
		   Either the client has given us a bad pointer or there is an
		   error in this module
		 */
		PVR_ASSERT(IMG_FALSE);
		return 0;
	}
}
#endif

/*
	Internal interfaces for management of syncprim block lists
*/
static SYNC_BLOCK_LIST *_SyncPrimBlockListCreate(IMG_VOID)
{
	SYNC_BLOCK_LIST *psBlockList;

	psBlockList = OSAllocMem(sizeof(SYNC_BLOCK_LIST) +
				 (sizeof(SYNC_PRIM_BLOCK *)
				  * SYNC_BLOCK_LIST_CHUNCK_SIZE));
	if (!psBlockList) {
		return IMG_NULL;
	}

	psBlockList->ui32BlockCount = 0;
	psBlockList->ui32BlockListSize = SYNC_BLOCK_LIST_CHUNCK_SIZE;

	psBlockList->papsSyncPrimBlock = OSAllocMem(sizeof(SYNC_PRIM_BLOCK *)
						    *
						    SYNC_BLOCK_LIST_CHUNCK_SIZE);
	if (!psBlockList->papsSyncPrimBlock) {
		OSFreeMem(psBlockList);
		return IMG_NULL;
	}

	OSMemSet(psBlockList->papsSyncPrimBlock,
		 0, sizeof(SYNC_PRIM_BLOCK *) * psBlockList->ui32BlockListSize);

	return psBlockList;
}

static PVRSRV_ERROR _SyncPrimBlockListAdd(SYNC_BLOCK_LIST * psBlockList,
					  SYNC_PRIM_BLOCK * psSyncPrimBlock)
{
	IMG_UINT32 i;

	/* Check the context isn't already on the list */
	for (i = 0; i < psBlockList->ui32BlockCount; i++) {
		if (psBlockList->papsSyncPrimBlock[i] == psSyncPrimBlock) {
			return PVRSRV_OK;
		}
	}

	/* Check we have space for a new item */
	if (psBlockList->ui32BlockCount == psBlockList->ui32BlockListSize) {
		SYNC_PRIM_BLOCK **papsNewSyncPrimBlock;

		papsNewSyncPrimBlock = OSAllocMem(sizeof(SYNC_PRIM_BLOCK *) *
						  (psBlockList->ui32BlockCount +
						   SYNC_BLOCK_LIST_CHUNCK_SIZE));
		if (!papsNewSyncPrimBlock) {
			return PVRSRV_ERROR_OUT_OF_MEMORY;
		}

		OSMemSet(psBlockList->papsSyncPrimBlock,
			 0,
			 sizeof(SYNC_PRIM_BLOCK *) *
			 psBlockList->ui32BlockListSize);
		OSMemCopy(papsNewSyncPrimBlock, psBlockList->papsSyncPrimBlock,
			  sizeof(SYNC_PRIM_CONTEXT *) *
			  psBlockList->ui32BlockCount);

		OSFreeMem(psBlockList->papsSyncPrimBlock);

		psBlockList->papsSyncPrimBlock = papsNewSyncPrimBlock;
		psBlockList->ui32BlockListSize += SYNC_BLOCK_LIST_CHUNCK_SIZE;
	}

	/* Add the context to the list */
	psBlockList->papsSyncPrimBlock[psBlockList->ui32BlockCount++] =
	    psSyncPrimBlock;
	return PVRSRV_OK;
}

static PVRSRV_ERROR _SyncPrimBlockListContextToIndex(SYNC_BLOCK_LIST *
						     psBlockList,
						     SYNC_PRIM_BLOCK *
						     psSyncPrimBlock,
						     IMG_UINT32 * pui32Index)
{
	IMG_UINT32 i;

	for (i = 0; i < psBlockList->ui32BlockCount; i++) {
		if (psBlockList->papsSyncPrimBlock[i] == psSyncPrimBlock) {
			*pui32Index = i;
			return PVRSRV_OK;
		}
	}

	return PVRSRV_ERROR_INVALID_PARAMS;
}

static PVRSRV_ERROR _SyncPrimBlockListHandleArrayCreate(SYNC_BLOCK_LIST *
							psBlockList,
							IMG_UINT32 *
							pui32BlockHandleCount,
							IMG_HANDLE **
							ppahHandleList)
{
	IMG_HANDLE *pahHandleList;
	IMG_UINT32 i;

	pahHandleList = OSAllocMem(sizeof(IMG_HANDLE *) *
				   psBlockList->ui32BlockCount);
	if (!pahHandleList) {
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	for (i = 0; i < psBlockList->ui32BlockCount; i++) {
		pahHandleList[i] =
		    psBlockList->papsSyncPrimBlock[i]->hServerSyncPrimBlock;
	}

	*ppahHandleList = pahHandleList;
	*pui32BlockHandleCount = psBlockList->ui32BlockCount;

	return PVRSRV_OK;
}

static IMG_VOID _SyncPrimBlockListHandleArrayDestroy(IMG_HANDLE * pahHandleList)
{
	OSFreeMem(pahHandleList);
}

static IMG_UINT32 _SyncPrimBlockListGetClientValue(SYNC_BLOCK_LIST *
						   psBlockList,
						   IMG_UINT32 ui32BlockIndex,
						   IMG_UINT32 ui32Index)
{
	return psBlockList->papsSyncPrimBlock[ui32BlockIndex]->
	    pui32LinAddr[ui32Index];
}

static IMG_VOID _SyncPrimBlockListDestroy(SYNC_BLOCK_LIST * psBlockList)
{
	OSFreeMem(psBlockList->papsSyncPrimBlock);
	OSFreeMem(psBlockList);
}

#if defined(PDUMP)
/*
	Internal interfaces for management of PDump functions
 */

static IMG_BOOL _PDumpSyncContext(PDLLIST_NODE psNode, IMG_PVOID pvData)
{
	SYNC_PRIM_CONTEXT *psContext =
	    IMG_CONTAINER_OF(psNode, SYNC_PRIM_CONTEXT, sListNode);
	SYNC_PRIM_BLOCK *psSyncBlock = psContext->psSyncBlock;
	PVR_UNREFERENCED_PARAMETER(pvData);

	/*
	   We might be ask to PDump sync state outside of capture range
	   (e.g. texture uploads) so make this continuous.
	 */
	DevmemPDumpLoadMem(psSyncBlock->hMemDesc,
			   0,
			   psSyncBlock->ui32SyncBlockSize,
			   PDUMP_FLAGS_CONTINUOUS);
	return IMG_TRUE;
}
#endif
/*
	External interfaces
*/

IMG_INTERNAL PVRSRV_ERROR
SyncPrimContextCreate(SYNC_BRIDGE_HANDLE hBridge,
		      IMG_HANDLE hDeviceNode,
		      PSYNC_PRIM_CONTEXT * phSyncPrimContext)
{
	SYNC_PRIM_CONTEXT *psContext;
	PVRSRV_ERROR eError;

	psContext = OSAllocMem(sizeof(SYNC_PRIM_CONTEXT));
	if (psContext == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	psContext->hBridge = hBridge;
	psContext->hDeviceNode = hDeviceNode;

	/* Allocate a page size block of sync memory */
	eError = AllocSyncPrimitiveBlock(psContext, &psContext->psSyncBlock);
	if (eError != PVRSRV_OK) {
		goto e1;
	}
	OSSNPrintf(psContext->azName, SYNC_PRIM_NAME_SIZE, "Sync Prim RA-%p",
		   psContext);

	psContext->psRA = RA_Create(	/* Params for initial import */
					   psContext->azName, 0, psContext->psSyncBlock->ui32SyncBlockSize, 0,	/* No flags */
					   IMG_NULL,	/* No private data */
					   /* Params for imports */
					   0, IMG_NULL, IMG_NULL, IMG_NULL);
	if (psContext->psRA == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e2;
	}
	psContext->ui32RefCount = 1;

	/* Link new context to the list */
	dllist_add_to_tail(&g_sContextList, &psContext->sListNode);

	*phSyncPrimContext = psContext;
	return PVRSRV_OK;
 e2:
	FreeSyncPrimitiveBlock(psContext->psSyncBlock);
 e1:
	OSFreeMem(psContext);
 e0:
	return eError;
}

IMG_INTERNAL IMG_VOID SyncPrimContextDestroy(PSYNC_PRIM_CONTEXT
					     hSyncPrimContext)
{
	SYNC_PRIM_CONTEXT *psContext = hSyncPrimContext;
	IMG_BOOL bDoRefCheck = IMG_TRUE;

/* FIXME */
#if defined(__KERNEL__)
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();
	if (psPVRSRVData->eServicesState != PVRSRV_SERVICES_STATE_OK) {
		bDoRefCheck = IMG_FALSE;
	}
#endif

	if (--psContext->ui32RefCount != 0) {
		PVR_DPF((PVR_DBG_ERROR,
			 "SyncPrimContextDestroy: Refcount non-zero"));

		if (bDoRefCheck) {
			PVR_ASSERT(0);
		}
		return;
	}

	/* Link new context to the list */
	dllist_remove_node(&psContext->sListNode);
	RA_Delete(psContext->psRA);
	FreeSyncPrimitiveBlock(psContext->psSyncBlock);
	OSFreeMem(psContext);
}

IMG_INTERNAL PVRSRV_ERROR SyncPrimAlloc(PSYNC_PRIM_CONTEXT hSyncPrimContext,
					PVRSRV_CLIENT_SYNC_PRIM ** ppsSync)
{
	SYNC_PRIM_CONTEXT *psContext = hSyncPrimContext;
	SYNC_PRIM *psNewSync;
	PVRSRV_ERROR eError;
	RA_BASE_T pBase;

	psNewSync = OSAllocMem(sizeof(SYNC_PRIM));
	if (psNewSync == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	if (!RA_Alloc(psContext->psRA,
		      sizeof(IMG_UINT32),
		      0, sizeof(IMG_UINT32), &pBase, IMG_NULL, IMG_NULL)) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e1;
	}
	psNewSync->eType = SYNC_PRIM_TYPE_LOCAL;
	psNewSync->u.sLocal.ui32Index = pBase / sizeof(IMG_UINT32);
	psNewSync->u.sLocal.psSyncBlock = psContext->psSyncBlock;
	SyncPrimGetCPULinAddr(psNewSync);
	psContext->ui32RefCount++;
	*ppsSync = &psNewSync->sCommon;

	return PVRSRV_OK;
 e1:
	OSFreeMem(psNewSync);
 e0:
	return eError;
}

IMG_INTERNAL IMG_VOID SyncPrimFree(PVRSRV_CLIENT_SYNC_PRIM * psSync)
{
	SYNC_PRIM *psSyncInt;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	if (psSyncInt->eType == SYNC_PRIM_TYPE_LOCAL) {
		SyncPrimLocalFree(psSyncInt);
	} else if (psSyncInt->eType == SYNC_PRIM_TYPE_SERVER) {
		SyncPrimServerFree(psSyncInt);
	} else {
		PVR_DPF((PVR_DBG_ERROR, "SyncPrimFree: Invalid sync type"));
		/*
		   Either the client has given us a bad pointer or there is an
		   error in this module
		 */
		PVR_ASSERT(IMG_FALSE);
		return;
	}
}

static IMG_VOID _SyncPrimSetValue(SYNC_PRIM * psSyncInt, IMG_UINT32 ui32Value)
{
	PVRSRV_ERROR eError;

	if (psSyncInt->eType == SYNC_PRIM_TYPE_LOCAL) {
		SYNC_PRIM_BLOCK *psSyncBlock;
		SYNC_PRIM_CONTEXT *psContext;

		psSyncBlock = psSyncInt->u.sLocal.psSyncBlock;
		psContext = psSyncBlock->psContext;

		eError = BridgeSyncPrimSet(psContext->hBridge,
					   psSyncBlock->hServerSyncPrimBlock,
					   psSyncInt->u.sLocal.ui32Index,
					   ui32Value);
		PVR_ASSERT(eError == PVRSRV_OK);
	} else {
		eError = BridgeServerSyncPrimSet(psSyncInt->u.sServer.hBridge,
						 psSyncInt->u.sServer.
						 hServerSync, ui32Value);
		PVR_ASSERT(eError == PVRSRV_OK);

	}
}

#if defined(NO_HARDWARE)
IMG_INTERNAL IMG_VOID
SyncPrimNoHwUpdate(PVRSRV_CLIENT_SYNC_PRIM * psSync, IMG_UINT32 ui32Value)
{
	SYNC_PRIM *psSyncInt;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	/* There is no check for the psSyncInt to be LOCAL as this call
	   substitutes the Firmware updating a sync and that sync could
	   be a server one */

	_SyncPrimSetValue(psSyncInt, ui32Value);
}
#endif

IMG_INTERNAL IMG_VOID
SyncPrimSet(PVRSRV_CLIENT_SYNC_PRIM * psSync, IMG_UINT32 ui32Value)
{
	SYNC_PRIM *psSyncInt;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	if (psSyncInt->eType != SYNC_PRIM_TYPE_LOCAL) {
		PVR_DPF((PVR_DBG_ERROR, "SyncPrimSet: Invalid sync type"));
		/*PVR_ASSERT(IMG_FALSE); */
		return;
	}

	_SyncPrimSetValue(psSyncInt, ui32Value);

#if defined(PDUMP)
	SyncPrimPDump(psSync);
#endif

}

#if !defined(__KERNEL__)

IMG_INTERNAL
    PVRSRV_ERROR SyncPrimExport(PVRSRV_CLIENT_SYNC_PRIM * psSync,
				PVRSRV_CLIENT_SYNC_PRIM_HANDLE * phExport)
{
	SYNC_PRIM *psSyncInt = (SYNC_PRIM *) psSync;
	PVRSRV_ERROR eError;

	if (!SyncPrimIsServerSync(psSync)) {
		/* For now we only export server syncs */
		eError = PVRSRV_ERROR_INVALID_SYNC_PRIM;
		goto e0;
	}

	/* Create a new, cross-process handle */
	eError = BridgeSyncPrimServerExport(psSyncInt->u.sServer.hBridge,
					    psSyncInt->u.sServer.hServerSync,
					    phExport);

	if (eError != PVRSRV_OK) {
		goto e0;
	}

	return PVRSRV_OK;

 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

#if !defined(SUPPORT_SECURE_EXPORT)
IMG_INTERNAL
    IMG_VOID SyncPrimUnexport(IMG_HANDLE hBridge,
			      PVRSRV_CLIENT_SYNC_PRIM_HANDLE hExport)
{
	PVRSRV_ERROR eError;

	eError = BridgeSyncPrimServerUnexport(hBridge, hExport);
	PVR_ASSERT(eError == PVRSRV_OK);
}
#endif

IMG_INTERNAL
    PVRSRV_ERROR SyncPrimImport(SYNC_BRIDGE_HANDLE hBridge,
				PVRSRV_CLIENT_SYNC_PRIM_HANDLE hImport,
				PVRSRV_CLIENT_SYNC_PRIM ** ppsSync)
{
	SYNC_PRIM *psNewSync;
	PVRSRV_ERROR eError;

	psNewSync = OSAllocMem(sizeof(SYNC_PRIM));
	if (psNewSync == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}
	OSMemSet(psNewSync, 0, sizeof(SYNC_PRIM));

	eError = BridgeSyncPrimServerImport(hBridge,
					    hImport,
					    &psNewSync->u.sServer.hServerSync,
					    &psNewSync->u.sServer.
					    ui32FirmwareAddr);

	if (eError != PVRSRV_OK) {
		goto e1;
	}

	psNewSync->eType = SYNC_PRIM_TYPE_SERVER;
	psNewSync->u.sServer.hBridge = hBridge;
	*ppsSync = &psNewSync->sCommon;

	return PVRSRV_OK;
 e1:
	OSFreeMem(psNewSync);
 e0:
	return eError;
}
#endif

IMG_INTERNAL
    PVRSRV_ERROR SyncPrimOpCreate(IMG_UINT32 ui32SyncCount,
				  PVRSRV_CLIENT_SYNC_PRIM ** papsSyncPrim,
				  PSYNC_OP_COOKIE * ppsCookie)
{
	SYNC_OP_COOKIE *psNewCookie;
	SYNC_BLOCK_LIST *psSyncBlockList;
	IMG_UINT32 ui32ServerSyncCount = 0;
	IMG_UINT32 ui32ClientSyncCount = 0;
	IMG_UINT32 ui32ServerAllocSize;
	IMG_UINT32 ui32ClientAllocSize;
	IMG_UINT32 ui32TotalAllocSize;
	IMG_UINT32 ui32ServerIndex = 0;
	IMG_UINT32 ui32ClientIndex = 0;
	IMG_UINT32 i;
	IMG_UINT32 ui32SyncBlockCount;
	IMG_HANDLE hBridge;
	IMG_HANDLE *pahHandleList;
	IMG_CHAR *pcPtr;
	PVRSRV_ERROR eError;

	psSyncBlockList = _SyncPrimBlockListCreate();

	if (!psSyncBlockList) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	for (i = 0; i < ui32SyncCount; i++) {
		if (SyncPrimIsServerSync(papsSyncPrim[i])) {
			ui32ServerSyncCount++;
		} else {
			SYNC_PRIM *psSync = (SYNC_PRIM *) papsSyncPrim[i];

			ui32ClientSyncCount++;
			eError =
			    _SyncPrimBlockListAdd(psSyncBlockList,
						  psSync->u.sLocal.psSyncBlock);
			if (eError != PVRSRV_OK) {
				goto e1;
			}
		}
	}

	ui32ServerAllocSize = ui32ServerSyncCount * (sizeof(IMG_HANDLE));
	ui32ClientAllocSize = ui32ClientSyncCount * (5 * sizeof(IMG_UINT32));
	ui32TotalAllocSize = sizeof(SYNC_OP_COOKIE) +
	    (sizeof(PVRSRV_CLIENT_SYNC_PRIM *) * ui32SyncCount) +
	    ui32ServerAllocSize + ui32ClientAllocSize;

	psNewCookie = OSAllocMem(ui32TotalAllocSize);
	pcPtr = (IMG_CHAR *) psNewCookie;

	if (!psNewCookie) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e1;
	}

	/* Setup the pointers */
	pcPtr += sizeof(SYNC_OP_COOKIE);
	psNewCookie->papsSyncPrim = (PVRSRV_CLIENT_SYNC_PRIM **) pcPtr;

	pcPtr += sizeof(PVRSRV_CLIENT_SYNC_PRIM *) * ui32SyncCount;
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
	psNewCookie->pahServerSync = (IMG_HANDLE *) pcPtr;
	pcPtr += sizeof(IMG_HANDLE) * ui32ServerSyncCount;

	/* Check the pointer setup went ok */
	PVR_ASSERT(pcPtr == (((IMG_CHAR *) psNewCookie) + ui32TotalAllocSize));

	psNewCookie->ui32SyncCount = ui32SyncCount;
	psNewCookie->ui32ServerSyncCount = ui32ServerSyncCount;
	psNewCookie->ui32ClientSyncCount = ui32ClientSyncCount;
	psNewCookie->psSyncBlockList = psSyncBlockList;

	/*
	   Get the bridge handle from the 1st sync.

	   Note: We assume the all syncs have been created with the same
	   services connection.
	 */
	if (SyncPrimIsServerSync(papsSyncPrim[0])) {
		SYNC_PRIM *psSync = (SYNC_PRIM *) papsSyncPrim[0];

		hBridge = psSync->u.sServer.hBridge;
	} else {
		SYNC_PRIM *psSync = (SYNC_PRIM *) papsSyncPrim[0];

		hBridge = psSync->u.sLocal.psSyncBlock->psContext->hBridge;
	}

	psNewCookie->hBridge = hBridge;

	if (ui32ServerSyncCount) {
		psNewCookie->bHaveServerSync = IMG_TRUE;
	} else {
		psNewCookie->bHaveServerSync = IMG_FALSE;
	}

	/* Fill in the server and client sync data */
	for (i = 0; i < ui32SyncCount; i++) {
		SYNC_PRIM *psSync = (SYNC_PRIM *) papsSyncPrim[i];

		if (SyncPrimIsServerSync(papsSyncPrim[i])) {
			psNewCookie->pahServerSync[ui32ServerIndex] =
			    psSync->u.sServer.hServerSync;

			ui32ServerIndex++;
		} else {
			/* Location of sync */
			eError =
			    _SyncPrimBlockListContextToIndex(psSyncBlockList,
							     psSync->u.sLocal.
							     psSyncBlock,
							     &psNewCookie->
							     paui32SyncBlockIndex
							     [ui32ClientIndex]);
			if (eError != PVRSRV_OK) {
				goto e2;
			}

			psNewCookie->paui32Index[ui32ClientIndex] =
			    psSync->u.sLocal.ui32Index;

			ui32ClientIndex++;
		}

		psNewCookie->papsSyncPrim[i] = papsSyncPrim[i];
	}

	eError = _SyncPrimBlockListHandleArrayCreate(psSyncBlockList,
						     &ui32SyncBlockCount,
						     &pahHandleList);
	if (eError != PVRSRV_OK) {
		goto e2;
	}

	/*
	   Create the server side cookie. Here we pass in all the unchanging
	   data so we only need to pass in the minimum at takeop time
	 */
	eError = BridgeSyncPrimOpCreate(hBridge,
					ui32SyncBlockCount,
					pahHandleList,
					psNewCookie->ui32ClientSyncCount,
					psNewCookie->paui32SyncBlockIndex,
					psNewCookie->paui32Index,
					psNewCookie->ui32ServerSyncCount,
					psNewCookie->pahServerSync,
					&psNewCookie->hServerCookie);

	/* Free the handle list regardless of error */
	_SyncPrimBlockListHandleArrayDestroy(pahHandleList);

	if (eError != PVRSRV_OK) {
		goto e2;
	}

	*ppsCookie = psNewCookie;
	return PVRSRV_OK;

 e2:
	OSFreeMem(psNewCookie);
 e1:
	_SyncPrimBlockListDestroy(psSyncBlockList);
 e0:
	return eError;
}

IMG_INTERNAL
    PVRSRV_ERROR SyncPrimOpTake(PSYNC_OP_COOKIE psCookie,
				IMG_UINT32 ui32SyncCount,
				PVRSRV_CLIENT_SYNC_PRIM_OP * pasSyncOp)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 ui32ClientIndex = 0;
	IMG_UINT32 i;

	/* Copy client sync operations */
	for (i = 0; i < ui32SyncCount; i++) {
		/*
		   Sanity check the client passes in the same syncs as the
		   ones we got at create time
		 */
		if (psCookie->papsSyncPrim[i] != pasSyncOp[i].psSync) {
			return PVRSRV_ERROR_INVALID_PARAMS;
		}

		if (!SyncPrimIsServerSync(pasSyncOp[i].psSync)) {
			/* Client operation information */
			psCookie->paui32Flags[ui32ClientIndex] =
			    pasSyncOp[i].ui32Flags;
			psCookie->paui32FenceValue[ui32ClientIndex] =
			    pasSyncOp[i].ui32FenceValue;
			psCookie->paui32UpdateValue[ui32ClientIndex] =
			    pasSyncOp[i].ui32UpdateValue;

			ui32ClientIndex++;
		}
	}

	eError = BridgeSyncPrimOpTake(psCookie->hBridge,
				      psCookie->hServerCookie,
				      psCookie->ui32ClientSyncCount,
				      psCookie->paui32Flags,
				      psCookie->paui32FenceValue,
				      psCookie->paui32UpdateValue,
				      psCookie->ui32ServerSyncCount);

	return eError;
}

IMG_INTERNAL
    PVRSRV_ERROR SyncPrimOpReady(PSYNC_OP_COOKIE psCookie, IMG_BOOL * pbReady)
{
	PVRSRV_ERROR eError;
	PVR_ASSERT(psCookie != IMG_NULL);

	/*
	   If we have a server sync we have no choice
	   but to do the check in the server
	 */
	if (psCookie->bHaveServerSync) {
		eError = BridgeSyncPrimOpReady(psCookie->hBridge,
					       psCookie->hServerCookie,
					       pbReady);
		if (eError != PVRSRV_OK) {
			PVR_DPF((PVR_DBG_ERROR,
				 "%s: Failed to do sync check in server (Error = %d)",
				 __FUNCTION__, eError));
			goto e0;
		}
	} else {
		IMG_UINT32 i;
		IMG_UINT32 ui32SnapShot;
		IMG_BOOL bReady = IMG_TRUE;

		for (i = 0; i < psCookie->ui32ClientSyncCount; i++) {
			if ((psCookie->
			     paui32Flags[i] & PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK)
			    == 0) {
				continue;
			}

			ui32SnapShot =
			    _SyncPrimBlockListGetClientValue(psCookie->
							     psSyncBlockList,
							     psCookie->
							     paui32SyncBlockIndex
							     [i],
							     psCookie->
							     paui32Index[i]);
			if (ui32SnapShot != psCookie->paui32FenceValue[i]) {
				break;
			}
		}

		*pbReady = bReady;
	}

	return PVRSRV_OK;
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

IMG_INTERNAL PVRSRV_ERROR SyncPrimOpComplete(PSYNC_OP_COOKIE psCookie)
{
	PVRSRV_ERROR eError;

	eError = BridgeSyncPrimOpComplete(psCookie->hBridge,
					  psCookie->hServerCookie);

	return eError;
}

IMG_INTERNAL IMG_VOID SyncPrimOpDestroy(PSYNC_OP_COOKIE psCookie)
{
	PVRSRV_ERROR eError;

	eError = BridgeSyncPrimOpDestroy(psCookie->hBridge,
					 psCookie->hServerCookie);
	PVR_ASSERT(eError == PVRSRV_OK);

	_SyncPrimBlockListDestroy(psCookie->psSyncBlockList);
	OSFreeMem(psCookie);
}

#if !defined(__KERNEL__)
IMG_INTERNAL
    PVRSRV_ERROR SyncPrimServerAlloc(SYNC_BRIDGE_HANDLE hBridge,
				     IMG_HANDLE hDeviceNode,
				     PVRSRV_CLIENT_SYNC_PRIM ** ppsSync)
{
	SYNC_PRIM *psNewSync;
	PVRSRV_ERROR eError;

	psNewSync = OSAllocMem(sizeof(SYNC_PRIM));
	if (psNewSync == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}
	OSMemSet(psNewSync, 0, sizeof(SYNC_PRIM));

	eError = BridgeServerSyncAlloc(hBridge,
				       hDeviceNode,
				       &psNewSync->u.sServer.hServerSync,
				       &psNewSync->u.sServer.ui32FirmwareAddr);

	if (eError != PVRSRV_OK) {
		goto e1;
	}

	psNewSync->eType = SYNC_PRIM_TYPE_SERVER;
	psNewSync->u.sServer.hBridge = hBridge;
	*ppsSync = &psNewSync->sCommon;

	return PVRSRV_OK;
 e1:
	OSFreeMem(psNewSync);
 e0:
	return eError;
}

IMG_INTERNAL
    PVRSRV_ERROR SyncPrimServerGetStatus(IMG_UINT32 ui32SyncCount,
					 PVRSRV_CLIENT_SYNC_PRIM ** papsSync,
					 IMG_UINT32 * pui32UID,
					 IMG_UINT32 * pui32FWAddr,
					 IMG_UINT32 * pui32CurrentOp,
					 IMG_UINT32 * pui32NextOp)
{
	PVRSRV_ERROR eError;
	IMG_UINT32 i;
	SYNC_BRIDGE_HANDLE hBridge = _SyncPrimGetBridgeHandle(papsSync[0]);
	IMG_HANDLE *pahServerHandle;

	pahServerHandle = OSAllocMem(sizeof(IMG_HANDLE) * ui32SyncCount);
	if (pahServerHandle == IMG_NULL) {
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto e0;
	}

	/*
	   Check that all the sync we've been passed are server syncs
	   and that they all are on the same connection.
	 */
	for (i = 0; i < ui32SyncCount; i++) {
		SYNC_PRIM *psIntSync =
		    IMG_CONTAINER_OF(papsSync[i], SYNC_PRIM, sCommon);

		if (!SyncPrimIsServerSync(papsSync[i])) {
			eError = PVRSRV_ERROR_INVALID_SYNC_PRIM;
			goto e1;
		}

		if (hBridge != _SyncPrimGetBridgeHandle(papsSync[i])) {
			PVR_DPF((PVR_DBG_ERROR,
				 "SyncServerGetStatus: Sync connection is different\n"));
			eError = PVRSRV_ERROR_INVALID_SYNC_PRIM;
			goto e1;
		}

		pahServerHandle[i] = psIntSync->u.sServer.hServerSync;
	}

	eError = BridgeServerSyncGetStatus(hBridge,
					   ui32SyncCount,
					   pahServerHandle,
					   pui32UID,
					   pui32FWAddr,
					   pui32CurrentOp, pui32NextOp);
	if (eError != PVRSRV_OK) {
		goto e1;
	}
	return PVRSRV_OK;

 e1:
	OSFreeMem(pahServerHandle);
 e0:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

#endif

IMG_INTERNAL IMG_BOOL SyncPrimIsServerSync(PVRSRV_CLIENT_SYNC_PRIM * psSync)
{
	SYNC_PRIM *psSyncInt;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);
	if (psSyncInt->eType == SYNC_PRIM_TYPE_SERVER) {
		return IMG_TRUE;
	}

	return IMG_FALSE;
}

IMG_INTERNAL
    IMG_HANDLE SyncPrimGetServerHandle(PVRSRV_CLIENT_SYNC_PRIM * psSync)
{
	SYNC_PRIM *psSyncInt;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);
	PVR_ASSERT(psSyncInt->eType == SYNC_PRIM_TYPE_SERVER);

	return psSyncInt->u.sServer.hServerSync;
}

IMG_INTERNAL
    PVRSRV_ERROR SyncPrimServerQueueOp(PVRSRV_CLIENT_SYNC_PRIM_OP * psSyncOp)
{
	SYNC_PRIM *psSyncInt;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psSyncOp != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSyncOp->psSync, SYNC_PRIM, sCommon);
	if (psSyncInt->eType != SYNC_PRIM_TYPE_SERVER) {
		return PVRSRV_ERROR_INVALID_SYNC_PRIM;
	}

	psSyncOp->ui32Flags = PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK |
	    PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE;

	eError = BridgeServerSyncQueueHWOp(psSyncInt->u.sServer.hBridge,
					   psSyncInt->u.sServer.hServerSync,
					   &psSyncOp->ui32FenceValue,
					   &psSyncOp->ui32UpdateValue);
	return eError;
}

#if defined(PDUMP)
IMG_INTERNAL IMG_VOID SyncPrimPDump(PVRSRV_CLIENT_SYNC_PRIM * psSync)
{
	SYNC_PRIM *psSyncInt;
	SYNC_PRIM_BLOCK *psSyncBlock;
	SYNC_PRIM_CONTEXT *psContext;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	if (psSyncInt->eType != SYNC_PRIM_TYPE_LOCAL) {
		PVR_DPF((PVR_DBG_ERROR, "SyncPrimPDump: Invalid sync type"));
		PVR_ASSERT(IMG_FALSE);
		return;
	}

	psSyncBlock = psSyncInt->u.sLocal.psSyncBlock;
	psContext = psSyncBlock->psContext;

	eError = BridgeSyncPrimPDump(psContext->hBridge,
				     psSyncBlock->hServerSyncPrimBlock,
				     psSyncInt->u.sLocal.ui32Index *
				     sizeof(IMG_UINT32));

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);
}

IMG_INTERNAL IMG_VOID SyncPrimPDumpPol(PVRSRV_CLIENT_SYNC_PRIM * psSync,
				       IMG_UINT32 ui32Value,
				       IMG_UINT32 ui32Mask,
				       PDUMP_POLL_OPERATOR eOperator,
				       IMG_UINT32 ui32PDumpFlags)
{
	SYNC_PRIM *psSyncInt;
	SYNC_PRIM_BLOCK *psSyncBlock;
	SYNC_PRIM_CONTEXT *psContext;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	if (psSyncInt->eType != SYNC_PRIM_TYPE_LOCAL) {
		PVR_DPF((PVR_DBG_ERROR, "SyncPrimPDumpPol: Invalid sync type"));
		PVR_ASSERT(IMG_FALSE);
		return;
	}

	psSyncBlock = psSyncInt->u.sLocal.psSyncBlock;
	psContext = psSyncBlock->psContext;

	eError = BridgeSyncPrimPDumpPol(psContext->hBridge,
					psSyncBlock->hServerSyncPrimBlock,
					psSyncInt->u.sLocal.ui32Index *
					sizeof(IMG_UINT32), ui32Value, ui32Mask,
					eOperator, ui32PDumpFlags);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);
}

IMG_INTERNAL IMG_VOID SyncPrimPDumpCBP(PVRSRV_CLIENT_SYNC_PRIM * psSync,
				       IMG_UINT32 uiWriteOffset,
				       IMG_UINT32 uiPacketSize,
				       IMG_UINT32 uiBufferSize)
{
	SYNC_PRIM *psSyncInt;
	SYNC_PRIM_BLOCK *psSyncBlock;
	SYNC_PRIM_CONTEXT *psContext;
	PVRSRV_ERROR eError;

	PVR_ASSERT(psSync != IMG_NULL);
	psSyncInt = IMG_CONTAINER_OF(psSync, SYNC_PRIM, sCommon);

	if (psSyncInt->eType != SYNC_PRIM_TYPE_LOCAL) {
		PVR_DPF((PVR_DBG_ERROR, "SyncPrimPDumpCBP: Invalid sync type"));
		PVR_ASSERT(IMG_FALSE);
		return;
	}

	psSyncBlock = psSyncInt->u.sLocal.psSyncBlock;
	psContext = psSyncBlock->psContext;

	eError = BridgeSyncPrimPDumpCBP(psContext->hBridge,
					psSyncBlock->hServerSyncPrimBlock,
					psSyncInt->u.sLocal.ui32Index *
					sizeof(IMG_UINT32), uiWriteOffset,
					uiPacketSize, uiBufferSize);

	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed with error %d", __FUNCTION__, eError));
	}
	PVR_ASSERT(eError == PVRSRV_OK);
}

IMG_INTERNAL IMG_VOID SyncPrimPDumpClientContexts(IMG_VOID)
{
	dllist_foreach_node(&g_sContextList, _PDumpSyncContext, IMG_NULL);
}
#endif
