/*!****************************************************************************
@File           bridged_pvr_bridge.h

@Title          PVR Bridge Functionality

@Author         Imagination Technologies

@Date           18th Jan 2008

@Copyright      Copyright 2008 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either material
                or conceptual may be copied or distributed, transmitted,
                transcribed, stored in a retrieval system or translated into
                any human or computer language in any form by any means,
                electronic, mechanical, manual or otherwise, or disclosed
                to third parties without the express written permission of
                Imagination Technologies Limited, Home Park Estate,
                Kings Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform       Generic

@Description    Header for the PVR Bridge code

@DoxygenVer

******************************************************************************/

#ifndef __BRIDGED_PVR_BRIDGE_H__
#define __BRIDGED_PVR_BRIDGE_H__

#include "connection_server.h"
#include "pvr_debug.h"

#include "pvr_bridge.h"
#if defined(SUPPORT_RGX)
#include "rgx_bridge.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__linux__)
#define PVRSRV_GET_BRIDGE_ID(X)	_IOC_NR(X)
#else
#define PVRSRV_GET_BRIDGE_ID(X)	((X) - PVRSRV_IOWR(PVRSRV_BRIDGE_CORE_CMD_FIRST))
#endif

#ifndef ENOMEM
#define ENOMEM	12
#endif
#ifndef EFAULT
#define EFAULT	14
#endif
#ifndef ENOTTY
#define ENOTTY	25
#endif

#if defined(DEBUG_BRIDGE_KM)
	PVRSRV_ERROR
	    CopyFromUserWrapper(CONNECTION_DATA * psConnection,
				IMG_UINT32 ui32BridgeID,
				IMG_VOID * pvDest,
				IMG_VOID * pvSrc, IMG_UINT32 ui32Size);
	PVRSRV_ERROR
	    CopyToUserWrapper(CONNECTION_DATA * psConnection,
			      IMG_UINT32 ui32BridgeID,
			      IMG_VOID * pvDest,
			      IMG_VOID * pvSrc, IMG_UINT32 ui32Size);
#else
#define CopyFromUserWrapper(psConnection, ui32BridgeID, pvDest, pvSrc, ui32Size) \
	OSCopyFromUser(psConnection, pvDest, pvSrc, ui32Size)
#define CopyToUserWrapper(psConnection, ui32BridgeID, pvDest, pvSrc, ui32Size) \
	OSCopyToUser(psConnection, pvDest, pvSrc, ui32Size)
#endif

#define ASSIGN_AND_RETURN_ON_ERROR(error, src, res)		\
	do							\
	{							\
		(error) = (src);				\
		if ((error) != PVRSRV_OK) 			\
		{						\
			return (res);				\
		}						\
	} while ((error) != PVRSRV_OK);

#define ASSIGN_AND_EXIT_ON_ERROR(error, src)		\
	ASSIGN_AND_RETURN_ON_ERROR(error, src, 0)

#if defined (PVR_SECURE_HANDLES)
#ifdef INLINE_IS_PRAGMA
#pragma inline(NewHandleBatch)
#endif
	static INLINE PVRSRV_ERROR
	    NewHandleBatch(CONNECTION_DATA * psConnection,
			   IMG_UINT32 ui32BatchSize) {
		PVRSRV_ERROR eError;

		 PVR_ASSERT(!psConnection->bHandlesBatched);

		 eError =
		    PVRSRVNewHandleBatch(psConnection->psHandleBase,
					 ui32BatchSize);

		if (eError == PVRSRV_OK) {
			psConnection->bHandlesBatched = IMG_TRUE;
		}

		return eError;
	}

#define NEW_HANDLE_BATCH_OR_ERROR(error, psConnection, ui32BatchSize)	\
	ASSIGN_AND_EXIT_ON_ERROR(error, NewHandleBatch(psConnection, ui32BatchSize))

#ifdef INLINE_IS_PRAGMA
#pragma inline(CommitHandleBatch)
#endif
	static INLINE PVRSRV_ERROR
	    CommitHandleBatch(CONNECTION_DATA * psConnection) {
		PVR_ASSERT(psConnection->bHandlesBatched);

		psConnection->bHandlesBatched = IMG_FALSE;

		return PVRSRVCommitHandleBatch(psConnection->psHandleBase);
	}

#define COMMIT_HANDLE_BATCH_OR_ERROR(error, psConnection) 			\
	ASSIGN_AND_EXIT_ON_ERROR(error, CommitHandleBatch(psConnection))

#ifdef INLINE_IS_PRAGMA
#pragma inline(ReleaseHandleBatch)
#endif
	static INLINE IMG_VOID
	    ReleaseHandleBatch(CONNECTION_DATA * psConnection) {
		if (psConnection->bHandlesBatched) {
			psConnection->bHandlesBatched = IMG_FALSE;

			PVRSRVReleaseHandleBatch(psConnection->psHandleBase);
		}
	}
#else				/* defined(PVR_SECURE_HANDLES) */
#define NEW_HANDLE_BATCH_OR_ERROR(error, psConnection, ui32BatchSize)
#define COMMIT_HANDLE_BATCH_OR_ERROR(error, psConnection)
#define ReleaseHandleBatch(psConnection)
#endif				/* defined(PVR_SECURE_HANDLES) */

	IMG_INT
	    DummyBW(IMG_UINT32 ui32BridgeID,
		    IMG_VOID * psBridgeIn,
		    IMG_VOID * psBridgeOut, CONNECTION_DATA * psConnection);

	typedef IMG_INT(*BridgeWrapperFunction) (IMG_UINT32 ui32BridgeID,
						 IMG_VOID * psBridgeIn,
						 IMG_VOID * psBridgeOut,
						 CONNECTION_DATA *
						 psConnection);

	typedef struct _PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY {
		BridgeWrapperFunction pfFunction;	/*!< The wrapper function that validates the ioctl
							   arguments before calling into srvkm proper */
#if defined(DEBUG_BRIDGE_KM)
		const IMG_CHAR *pszIOCName;	/*!< Name of the ioctl: e.g. "PVRSRV_BRIDGE_CONNECT_SERVICES" */
		const IMG_CHAR *pszFunctionName;	/*!< Name of the wrapper function: e.g. "PVRSRVConnectBW" */
		IMG_UINT32 ui32CallCount;	/*!< The total number of times the ioctl has been called */
		IMG_UINT32 ui32CopyFromUserTotalBytes;	/*!< The total number of bytes copied from
							   userspace within this ioctl */
		IMG_UINT32 ui32CopyToUserTotalBytes;	/*!< The total number of bytes copied from
							   userspace within this ioctl */
#endif
	} PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY;

#if defined(SUPPORT_VGX) || defined(SUPPORT_MSVDX)
#if defined(SUPPORT_VGX)
#define BRIDGE_DISPATCH_TABLE_ENTRY_COUNT (PVRSRV_BRIDGE_LAST_VGX_CMD+1)
#define PVRSRV_BRIDGE_LAST_DEVICE_CMD	   PVRSRV_BRIDGE_LAST_VGX_CMD
#else
#define BRIDGE_DISPATCH_TABLE_ENTRY_COUNT (PVRSRV_BRIDGE_LAST_MSVDX_CMD+1)
#define PVRSRV_BRIDGE_LAST_DEVICE_CMD	   PVRSRV_BRIDGE_LAST_MSVDX_CMD
#endif
#else
#if defined(SUPPORT_RGX)
#define BRIDGE_DISPATCH_TABLE_ENTRY_COUNT (PVRSRV_BRIDGE_LAST_RGX_CMD+1)
#define PVRSRV_BRIDGE_LAST_DEVICE_CMD	   PVRSRV_BRIDGE_LAST_RGX_CMD
#else
#define BRIDGE_DISPATCH_TABLE_ENTRY_COUNT (PVRSRV_BRIDGE_LAST_NON_DEVICE_CMD+1)
#define PVRSRV_BRIDGE_LAST_DEVICE_CMD	   PVRSRV_BRIDGE_LAST_NON_DEVICE_CMD
#endif
#endif

	extern PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY
	    g_BridgeDispatchTable[BRIDGE_DISPATCH_TABLE_ENTRY_COUNT];

	 IMG_VOID
	    _SetDispatchTableEntry(IMG_UINT32 ui32Index,
				   const IMG_CHAR * pszIOCName,
				   BridgeWrapperFunction pfFunction,
				   const IMG_CHAR * pszFunctionName);

	/* PRQA S 0884,3410 2*//* macro relies on the lack of brackets */
#define SetDispatchTableEntry(ui32Index, pfFunction) \
	_SetDispatchTableEntry(PVRSRV_GET_BRIDGE_ID(ui32Index), #ui32Index, (BridgeWrapperFunction)pfFunction, #pfFunction)

#define DISPATCH_TABLE_GAP_THRESHOLD 5

#if defined(DEBUG)
#define PVRSRV_BRIDGE_ASSERT_CMD(X, Y) PVR_ASSERT(X == PVRSRV_GET_BRIDGE_ID(Y))
#else
#define PVRSRV_BRIDGE_ASSERT_CMD(X, Y) PVR_UNREFERENCED_PARAMETER(X)
#endif

#if defined(DEBUG_BRIDGE_KM)
	typedef struct _PVRSRV_BRIDGE_GLOBAL_STATS {
		IMG_UINT32 ui32IOCTLCount;
		IMG_UINT32 ui32TotalCopyFromUserBytes;
		IMG_UINT32 ui32TotalCopyToUserBytes;
	} PVRSRV_BRIDGE_GLOBAL_STATS;

/* OS specific code way want to report the stats held here and within the
 * BRIDGE_DISPATCH_TABLE_ENTRYs (E.g. on Linux we report these via a
 * proc entry /proc/pvr/bridge_stats. Ref printLinuxBridgeStats()) */
	extern PVRSRV_BRIDGE_GLOBAL_STATS g_BridgeGlobalStats;
#endif

	PVRSRV_ERROR CommonBridgeInit(IMG_VOID);

	IMG_INT BridgedDispatchKM(CONNECTION_DATA * psConnection,
				  PVRSRV_BRIDGE_PACKAGE * psBridgePackageKM);

	 PVRSRV_ERROR PVRSRVConnectKM(IMG_UINT32 ui32Flags);

	 PVRSRV_ERROR PVRSRVDisconnectKM(IMG_VOID);

	 PVRSRV_ERROR PVRSRVInitSrvConnectKM(CONNECTION_DATA * psConnection);

	 PVRSRV_ERROR
	    PVRSRVInitSrvDisconnectKM(CONNECTION_DATA * psConnection,
				      IMG_BOOL bInitSuccesful);

	 PVRSRV_ERROR PVRSRVDumpDebugInfoKM(IMG_VOID);

#if defined (__cplusplus)
}
#endif
#endif				/* __BRIDGED_PVR_BRIDGE_H__ */
/******************************************************************************
 End of file (bridged_pvr_bridge.h)
******************************************************************************/
