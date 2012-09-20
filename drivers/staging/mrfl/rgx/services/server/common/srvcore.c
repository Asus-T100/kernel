									    /*************************************************************************//*!
									       @File
									       @Title          PVR Common Bridge Module (kernel side)
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Implements core PVRSRV API, server side
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include <stddef.h>

#include "img_defs.h"
#include "pvr_debug.h"
#include "ra.h"
#include "pvr_bridge.h"
#include "connection_server.h"
#include "device.h"

#include "pdump_km.h"

#include "env_data.h"		/* FIXME when this is removed, -Werror should be re-enabled for this file */

#include "srvkm.h"
#include "allocmem.h"

#include "srvcore.h"
#include "pvrsrv.h"
#include "power.h"
#include "lists.h"

/* For the purpose of maintainability, it is intended that this file should not
 * contain any OS specific #ifdefs. Please find a way to add e.g.
 * an osfunc.c abstraction or override the entire function in question within
 * env,*,pvr_bridge_k.c
 */

PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY
    g_BridgeDispatchTable[BRIDGE_DISPATCH_TABLE_ENTRY_COUNT];

#if defined(DEBUG_BRIDGE_KM)
PVRSRV_BRIDGE_GLOBAL_STATS g_BridgeGlobalStats;
#endif

/* FIXME: Why do we need psConnection? It's not used here or in OSCopyX */
#if defined(DEBUG_BRIDGE_KM)
PVRSRV_ERROR
CopyFromUserWrapper(CONNECTION_DATA * psConnection,
		    IMG_UINT32 ui32BridgeID,
		    IMG_VOID * pvDest, IMG_VOID * pvSrc, IMG_UINT32 ui32Size)
{
	g_BridgeDispatchTable[ui32BridgeID].ui32CopyFromUserTotalBytes +=
	    ui32Size;
	g_BridgeGlobalStats.ui32TotalCopyFromUserBytes += ui32Size;
	return OSCopyFromUser(psConnection, pvDest, pvSrc, ui32Size);
}

PVRSRV_ERROR
CopyToUserWrapper(CONNECTION_DATA * psConnection,
		  IMG_UINT32 ui32BridgeID,
		  IMG_VOID * pvDest, IMG_VOID * pvSrc, IMG_UINT32 ui32Size)
{
	g_BridgeDispatchTable[ui32BridgeID].ui32CopyToUserTotalBytes +=
	    ui32Size;
	g_BridgeGlobalStats.ui32TotalCopyToUserBytes += ui32Size;
	return OSCopyToUser(psConnection, pvDest, pvSrc, ui32Size);
}
#endif

PVRSRV_ERROR PVRSRVConnectKM(IMG_UINT32 ui32Flags)
{
	PVRSRV_ERROR eError = PVRSRV_OK;

#if defined(PDUMP) && defined(FIXME)
	/* Store the per process connection info.
	 * FIXME: the Xserver initially connects via PVR2D which sets the persistent flag.
	 * But, later the Xserver may connect via SGL which doesn't carry the flag (in
	 * general SGL clients aren't persistent). So we OR in the flag so if it was set
	 * before it remains set.
	 */
	if (ui32Flags & SRV_FLAGS_PERSIST) {
		eError = PDumpAddPersistantProcess();
	}
#else
	PVR_UNREFERENCED_PARAMETER(ui32Flags);
#endif

	return eError;
}

PVRSRV_ERROR PVRSRVDisconnectKM(IMG_VOID)
{
	/* just return OK, per-process data is cleaned up by resmgr */

	return PVRSRV_OK;
}

/*!
******************************************************************************
 @Function	PVRSRVDumpDebugInfo_ForEachVaCb

 @Description

 Callback function for List_PVRSRV_DEVICE_NODE_ForEach_va(). Dump debug info
 for the device.

 @Input psDeviceNode	- The device node
 		va				- variable arguments list, not used
******************************************************************************/
static IMG_VOID PVRSRVDumpDebugInfo_ForEachVaCb(PVRSRV_DEVICE_NODE *
						psDeviceNode, va_list va)
{
	PVR_UNREFERENCED_PARAMETER(va);

	if (psDeviceNode->pfnDumpDebugInfo != IMG_NULL) {
		psDeviceNode->pfnDumpDebugInfo(psDeviceNode);
	}
}

/*
	PVRSRVDumpDebugInfoKM
*/
PVRSRV_ERROR PVRSRVDumpDebugInfoKM(IMG_VOID)
{
	PVRSRV_DATA *psPVRSRVData = PVRSRVGetPVRSRVData();

	PVR_LOG(("User requested PVR debug info"));

	/*
	   Search through the device list for services managed devices, dumping
	   debug info for each.
	 */
	List_PVRSRV_DEVICE_NODE_ForEach_va(psPVRSRVData->psDeviceNodeList,
					   &PVRSRVDumpDebugInfo_ForEachVaCb);

	return PVRSRV_OK;
}

IMG_INT
DummyBW(IMG_UINT32 ui32BridgeID,
	IMG_VOID * psBridgeIn,
	IMG_VOID * psBridgeOut, CONNECTION_DATA * psConnection)
{
#if !defined(DEBUG)
	PVR_UNREFERENCED_PARAMETER(ui32BridgeID);
#endif
	PVR_UNREFERENCED_PARAMETER(psBridgeIn);
	PVR_UNREFERENCED_PARAMETER(psBridgeOut);
	PVR_UNREFERENCED_PARAMETER(psConnection);

#if defined(DEBUG_BRIDGE_KM)
	PVR_DPF((PVR_DBG_ERROR, "%s: BRIDGE ERROR: BridgeID %u (%s) mapped to "
		 "Dummy Wrapper (probably not what you want!)",
		 __FUNCTION__, ui32BridgeID,
		 g_BridgeDispatchTable[ui32BridgeID].pszIOCName));
#else
	PVR_DPF((PVR_DBG_ERROR, "%s: BRIDGE ERROR: BridgeID %u mapped to "
		 "Dummy Wrapper (probably not what you want!)",
		 __FUNCTION__, ui32BridgeID));
#endif
	return -ENOTTY;
}

/*!
 * *****************************************************************************
 * @brief A wrapper for filling in the g_BridgeDispatchTable array that does
 * 		  error checking.
 *
 * @param ui32Index
 * @param pszIOCName
 * @param pfFunction
 * @param pszFunctionName
 *
 * @return
 ********************************************************************************/
IMG_VOID
_SetDispatchTableEntry(IMG_UINT32 ui32Index,
		       const IMG_CHAR * pszIOCName,
		       BridgeWrapperFunction pfFunction,
		       const IMG_CHAR * pszFunctionName)
{
	static IMG_UINT32 ui32PrevIndex = ~0UL;	/* -1 */
#if !defined(DEBUG)
	PVR_UNREFERENCED_PARAMETER(pszIOCName);
#endif
#if !defined(DEBUG_BRIDGE_KM_DISPATCH_TABLE) && !defined(DEBUG_BRIDGE_KM)
	PVR_UNREFERENCED_PARAMETER(pszFunctionName);
#endif

#if defined(DEBUG_BRIDGE_KM_DISPATCH_TABLE)
	/* Enable this to dump out the dispatch table entries */
	PVR_DPF((PVR_DBG_WARNING, "%s: %d %s %s", __FUNCTION__, ui32Index,
		 pszIOCName, pszFunctionName));
#endif

	/* If we are over-writing a previous entry that is a BUG!
	 * NOTE: This shouldn't be debug only since switching from debug->release
	 * etc is likly to modify the available ioctls and thus be a point where
	 * mistakes are exposed. This isn't run at at a performance critical time.
	 */
	if (g_BridgeDispatchTable[ui32Index].pfFunction) {
#if defined(DEBUG_BRIDGE_KM)
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: BUG!: Adding dispatch table entry for %s clobbers an existing entry for %s",
			 __FUNCTION__, pszIOCName,
			 g_BridgeDispatchTable[ui32Index].pszIOCName));
#else
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: BUG!: Adding dispatch table entry for %s clobbers an existing entry (index=%u)",
			 __FUNCTION__, pszIOCName, ui32Index));
#endif
		PVR_DPF((PVR_DBG_ERROR,
			 "NOTE: Enabling DEBUG_BRIDGE_KM_DISPATCH_TABLE may help debug this issue."));
		OSPanic();
	}

	/* Any gaps are sub-optimal in-terms of memory usage, but we are mainly
	 * interested in spotting any large gap of wasted memory that could be
	 * accidentally introduced.
	 *
	 * This will currently flag up any gaps > 5 entries.
	 *
	 * NOTE: This shouldn't be debug only since switching from debug->release
	 * etc is likly to modify the available ioctls and thus be a point where
	 * mistakes are exposed. This isn't run at at a performance critical time.
	 */
//      if((ui32PrevIndex != (IMG_UINT32)-1) &&
	if ((ui32PrevIndex != ~0UL) &&
	    ((ui32Index >= ui32PrevIndex + DISPATCH_TABLE_GAP_THRESHOLD) ||
	     (ui32Index <= ui32PrevIndex))) {
#if defined(DEBUG_BRIDGE_KM)
		PVR_DPF((PVR_DBG_WARNING,
			 "%s: There is a gap in the dispatch table between indices %u (%s) and %u (%s)",
			 __FUNCTION__, ui32PrevIndex,
			 g_BridgeDispatchTable[ui32PrevIndex].pszIOCName,
			 ui32Index, pszIOCName));
#else
		PVR_DPF((PVR_DBG_WARNING,
			 "%s: There is a gap in the dispatch table between indices %u and %u (%s)",
			 __FUNCTION__, (IMG_UINT) ui32PrevIndex,
			 (IMG_UINT) ui32Index, pszIOCName));
#endif
		PVR_DPF((PVR_DBG_ERROR,
			 "NOTE: Enabling DEBUG_BRIDGE_KM_DISPATCH_TABLE may help debug this issue."));
		OSPanic();
	}

	g_BridgeDispatchTable[ui32Index].pfFunction = pfFunction;
#if defined(DEBUG_BRIDGE_KM)
	g_BridgeDispatchTable[ui32Index].pszIOCName = pszIOCName;
	g_BridgeDispatchTable[ui32Index].pszFunctionName = pszFunctionName;
	g_BridgeDispatchTable[ui32Index].ui32CallCount = 0;
	g_BridgeDispatchTable[ui32Index].ui32CopyFromUserTotalBytes = 0;
#endif

	ui32PrevIndex = ui32Index;
}

PVRSRV_ERROR PVRSRVInitSrvConnectKM(CONNECTION_DATA * psConnection)
{
	/* PRQA S 3415 1 *//* side effects needed - if any step fails */
	if ((OSProcHasPrivSrvInit() == IMG_FALSE)
	    || PVRSRVGetInitServerState(PVRSRV_INIT_SERVER_RUNNING)
	    || PVRSRVGetInitServerState(PVRSRV_INIT_SERVER_RAN)) {
		return PVRSRV_ERROR_SRV_CONNECT_FAILED;
	}
#if defined (__linux__)
	PVRSRVSetInitServerState(PVRSRV_INIT_SERVER_RUNNING, IMG_TRUE);
#endif
	psConnection->bInitProcess = IMG_TRUE;

	return PVRSRV_OK;
}

PVRSRV_ERROR
PVRSRVInitSrvDisconnectKM(CONNECTION_DATA * psConnection,
			  IMG_BOOL bInitSuccesful)
{
	PVRSRV_ERROR eError;

	if (!psConnection->bInitProcess) {
		return PVRSRV_ERROR_SRV_DISCONNECT_FAILED;
	}

	psConnection->bInitProcess = IMG_FALSE;

	PVRSRVSetInitServerState(PVRSRV_INIT_SERVER_RUNNING, IMG_FALSE);
	PVRSRVSetInitServerState(PVRSRV_INIT_SERVER_RAN, IMG_TRUE);

	eError = PVRSRVFinaliseSystem(bInitSuccesful);

	PVRSRVSetInitServerState(PVRSRV_INIT_SERVER_SUCCESSFUL,
				 ((eError == PVRSRV_OK) && (bInitSuccesful))
				 ? IMG_TRUE : IMG_FALSE);

	return eError;
}

IMG_INT BridgedDispatchKM(CONNECTION_DATA * psConnection,
			  PVRSRV_BRIDGE_PACKAGE * psBridgePackageKM)
{

	IMG_VOID *psBridgeIn;
	IMG_VOID *psBridgeOut;
	BridgeWrapperFunction pfBridgeHandler;
	IMG_UINT32 ui32BridgeID = psBridgePackageKM->ui32BridgeID;
	IMG_INT err = -EFAULT;

#if defined(DEBUG_TRACE_BRIDGE_KM)
	PVR_DPF((PVR_DBG_CALLTRACE, "%s: %s",
		 __FUNCTION__, g_BridgeDispatchTable[ui32BridgeID].pszIOCName));
#endif

#if defined(DEBUG_BRIDGE_KM)
	g_BridgeDispatchTable[ui32BridgeID].ui32CallCount++;
	g_BridgeGlobalStats.ui32IOCTLCount++;
#endif

	if (!psConnection->bInitProcess) {
		if (PVRSRVGetInitServerState(PVRSRV_INIT_SERVER_RAN)) {
			if (!PVRSRVGetInitServerState
			    (PVRSRV_INIT_SERVER_SUCCESSFUL)) {
				PVR_DPF((PVR_DBG_ERROR,
					 "%s: Initialisation failed.  Driver unusable.",
					 __FUNCTION__));
				goto return_fault;
			}
		} else {
			if (PVRSRVGetInitServerState
			    (PVRSRV_INIT_SERVER_RUNNING)) {
				PVR_DPF((PVR_DBG_ERROR,
					 "%s: Initialisation is in progress",
					 __FUNCTION__));
				goto return_fault;
			} else {
				/* Only certain operations are allowed */
				switch (ui32BridgeID) {
					/* FIXME: Bridge defines should never be used outside auto generated code,
					   we need to rethink this */
				case PVRSRV_GET_BRIDGE_ID(PVRSRV_BRIDGE_SRVCORE_CONNECT):
				case PVRSRV_GET_BRIDGE_ID(PVRSRV_BRIDGE_SRVCORE_DISCONNECT):
				case PVRSRV_GET_BRIDGE_ID(PVRSRV_BRIDGE_SRVCORE_INITSRVCONNECT):
				case PVRSRV_GET_BRIDGE_ID(PVRSRV_BRIDGE_SRVCORE_INITSRVDISCONNECT):
					break;
				default:
					PVR_DPF((PVR_DBG_ERROR,
						 "%s: Driver initialisation not completed yet.",
						 __FUNCTION__));
					goto return_fault;
				}
			}
		}
	}

#if defined(__linux__)
	{
		/* This should be moved into the linux specific code */
		ENV_DATA *psEnvData = OSGetEnvData();

		/* We have already set up some static buffers to store our ioctl data... */
		psBridgeIn = psEnvData->pvBridgeData;
		psBridgeOut =
		    (IMG_PVOID) ((IMG_PBYTE) psBridgeIn +
				 PVRSRV_MAX_BRIDGE_IN_SIZE);

		/* check we are not using a bigger bridge than allocated */
#if defined(DEBUG)
		PVR_ASSERT(psBridgePackageKM->ui32InBufferSize <
			   PVRSRV_MAX_BRIDGE_IN_SIZE);
		PVR_ASSERT(psBridgePackageKM->ui32OutBufferSize <
			   PVRSRV_MAX_BRIDGE_OUT_SIZE);
#endif

		if (psBridgePackageKM->ui32InBufferSize > 0) {
			if (!OSAccessOK(PVR_VERIFY_READ,
					psBridgePackageKM->pvParamIn,
					psBridgePackageKM->ui32InBufferSize)) {
				PVR_DPF((PVR_DBG_ERROR,
					 "%s: Invalid pvParamIn pointer",
					 __FUNCTION__));
			}

			if (CopyFromUserWrapper(psConnection,
						ui32BridgeID,
						psBridgeIn,
						psBridgePackageKM->pvParamIn,
						psBridgePackageKM->
						ui32InBufferSize)
			    != PVRSRV_OK) {
				goto return_fault;
			}
		}

		if (psBridgePackageKM->ui32OutBufferSize > 0) {
			/*
			 * Copy the output structure as it might contain
			 * pointers which from our point of view are inputs
			 */
			if (CopyFromUserWrapper(psConnection,
						ui32BridgeID,
						psBridgeOut,
						psBridgePackageKM->pvParamOut,
						psBridgePackageKM->
						ui32OutBufferSize)
			    != PVRSRV_OK) {
				goto return_fault;
			}
		}
	}
#else
	psBridgeIn = psBridgePackageKM->pvParamIn;
	psBridgeOut = psBridgePackageKM->pvParamOut;
#endif

	if (ui32BridgeID >= (BRIDGE_DISPATCH_TABLE_ENTRY_COUNT)) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: ui32BridgeID = %d is out if range!", __FUNCTION__,
			 ui32BridgeID));
		goto return_fault;
	}
	pfBridgeHandler =
	    (BridgeWrapperFunction) g_BridgeDispatchTable[ui32BridgeID].
	    pfFunction;
	err =
	    pfBridgeHandler(ui32BridgeID, psBridgeIn, psBridgeOut,
			    psConnection);
	if (err < 0) {
		goto return_fault;
	}

#if defined(__linux__)
	/* 
	   This should always be true as a.t.m. all bridge calls have to
	   return an error message, but this could change so we do this
	   check to be safe.
	 */
	if (psBridgePackageKM->ui32OutBufferSize > 0) {
		/* This should be moved into the linux specific code */
		if (CopyToUserWrapper(psConnection,
				      ui32BridgeID,
				      psBridgePackageKM->pvParamOut,
				      psBridgeOut,
				      psBridgePackageKM->ui32OutBufferSize)
		    != PVRSRV_OK) {
			goto return_fault;
		}
	}
#endif

	err = 0;
 return_fault:
	ReleaseHandleBatch(psConnection);
	return err;
}

/******************************************************************************
 End of file (bridged_pvr_bridge.c)
******************************************************************************/
