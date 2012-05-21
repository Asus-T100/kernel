/*******************************************************************************
* Name         : pvr_bridge_k.c
* Title        : PVR Bridge Module (kernel side)
* Author       : Imagination Technologies
* Created      : 8 September 2003
*
* Copyright    : 2003-2008 by Imagination Technologies Limited.
*                All rights reserved.  No part of this software, either
*                material or conceptual may be copied or distributed,
*                transmitted, transcribed, stored in a retrieval system
*                or translated into any human or computer language in any
*                form by any means, electronic, mechanical, manual or
*                other-wise, or disclosed to third parties without the
*                express written permission of Imagination Technologies
*                Limited, Unit 8, HomePark Industrial Estate,
*                King's Langley, Hertfordshire, WD4 8LZ, U.K.
*
* Description  : Receives calls from the user portion of services and
*                despatches them to functions in the kernel portion.
*
* Platform     : linux
*
********************************************************************************/

#include "img_defs.h"
#include "pvr_bridge.h"
#include "connection_server.h"
#include "mutex.h"
#include "syscommon.h"
#include "pvr_debug.h"
#include "proc.h"
#include "private_data.h"
#include "linkage.h"

#if defined(SUPPORT_DRI_DRM)
#include <drm/drmP.h>
#include "pvr_drm.h"
#if defined(PVR_SECURE_DRM_AUTH_EXPORT)
#include "env_connection.h"
#endif
#endif

/* RGX: */
#if defined(SUPPORT_RGX)
#include "rgx_bridge.h"
#endif

#include "srvcore.h"
#include "common_srvcore_bridge.h"
#include "cache_defines.h"

#ifdef MODULE_TEST
/************************************************************************/
// additional includes for services testing
/************************************************************************/
#include "pvr_test_bridge.h"
#include "kern_test.h"
/************************************************************************/
// end of additional includes
/************************************************************************/
#endif

#if defined(SUPPORT_DRI_DRM)
#define	PRIVATE_DATA(pFile) ((pFile)->driver_priv)
#else
#define	PRIVATE_DATA(pFile) ((pFile)->private_data)
#endif

#if defined(DEBUG_BRIDGE_KM)

static struct proc_dir_entry *g_ProcBridgeStats = 0;
static void *ProcSeqNextBridgeStats(struct seq_file *sfile, void *el,
				    loff_t off);
static void ProcSeqShowBridgeStats(struct seq_file *sfile, void *el);
static void *ProcSeqOff2ElementBridgeStats(struct seq_file *sfile, loff_t off);
static void ProcSeqStartstopBridgeStats(struct seq_file *sfile, IMG_BOOL start);

#endif

extern PVRSRV_LINUX_MUTEX gPVRSRVLock;

PVRSRV_ERROR RegisterPDUMPFunctions(IMG_VOID);
PVRSRV_ERROR RegisterDCFunctions(IMG_VOID);
PVRSRV_ERROR RegisterMMFunctions(IMG_VOID);
PVRSRV_ERROR RegisterCMMFunctions(IMG_VOID);
PVRSRV_ERROR RegisterPDUMPMMFunctions(IMG_VOID);
PVRSRV_ERROR RegisterPDUMPCMMFunctions(IMG_VOID);
PVRSRV_ERROR RegisterSRVCOREFunctions(IMG_VOID);
PVRSRV_ERROR RegisterSYNCFunctions(IMG_VOID);
#if !defined(SUPPORT_SECURE_EXPORT)
PVRSRV_ERROR RegisterSYNCEXPORTFunctions(IMG_VOID);
#else
PVRSRV_ERROR RegisterSYNCSEXPORTFunctions(IMG_VOID);
#endif
#if defined (SUPPORT_RGX)
PVRSRV_ERROR RegisterRGXINITFunctions(IMG_VOID);
PVRSRV_ERROR RegisterRGXTA3DFunctions(IMG_VOID);
PVRSRV_ERROR RegisterRGXTQFunctions(IMG_VOID);
PVRSRV_ERROR RegisterRGXCMPFunctions(IMG_VOID);
PVRSRV_ERROR RegisterRGXCCBFunctions(IMG_VOID);
PVRSRV_ERROR RegisterHOSTPORTIOFunctions(IMG_VOID);
PVRSRV_ERROR RegisterBREAKPOINTFunctions(IMG_VOID);
PVRSRV_ERROR RegisterDEBUGMISCFunctions(IMG_VOID);
PVRSRV_ERROR RegisterRGXPDUMPFunctions(IMG_VOID);
#endif				/* SUPPORT_RGX */
#if (CACHEFLUSH_TYPE == CACHEFLUSH_GENERIC)
PVRSRV_ERROR RegisterCACHEGENERICFunctions(IMG_VOID);
#endif
#if defined(SUPPORT_SECURE_EXPORT)
PVRSRV_ERROR RegisterSMMFunctions(IMG_VOID);
#endif
#if defined(SUPPORT_PMMIF)
PVRSRV_ERROR RegisterPMMIFFunctions(IMG_VOID);
#endif

/* FIXME: These and their friends above will go when full bridge gen comes in */
PVRSRV_ERROR LinuxBridgeInit(IMG_VOID);
IMG_VOID LinuxBridgeDeInit(IMG_VOID);

PVRSRV_ERROR LinuxBridgeInit(IMG_VOID)
{
	PVRSRV_ERROR eError;

#if defined(DEBUG_BRIDGE_KM)
	{
		g_ProcBridgeStats = CreateProcReadEntrySeq("bridge_stats",
							   NULL,
							   ProcSeqNextBridgeStats,
							   ProcSeqShowBridgeStats,
							   ProcSeqOff2ElementBridgeStats,
							   ProcSeqStartstopBridgeStats);
		if (!g_ProcBridgeStats) {
			return PVRSRV_ERROR_OUT_OF_MEMORY;
		}
	}
#endif
	eError = RegisterSRVCOREFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterSYNCFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#if !defined(SUPPORT_SECURE_EXPORT)
	eError = RegisterSYNCEXPORTFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#else
	eError = RegisterSYNCSEXPORTFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#endif

	eError = RegisterPDUMPFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
	eError = RegisterMMFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
	eError = RegisterCMMFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
	eError = RegisterPDUMPMMFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
	eError = RegisterPDUMPCMMFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#if defined(SUPPORT_PMMIF)
	eError = RegisterPMMIFFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#endif

	eError = RegisterDCFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#if (CACHEFLUSH_TYPE == CACHEFLUSH_GENERIC)
	eError = RegisterCACHEGENERICFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#endif

#if defined(SUPPORT_SECURE_EXPORT)
	eError = RegisterSMMFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
#endif

#if defined (SUPPORT_RGX)
	eError = RegisterRGXTQFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}
	eError = RegisterRGXCCBFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterRGXCMPFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterRGXINITFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterRGXTA3DFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterHOSTPORTIOFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterBREAKPOINTFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterDEBUGMISCFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

	eError = RegisterRGXPDUMPFunctions();
	if (eError != PVRSRV_OK) {
		return eError;
	}

#endif				/* SUPPORT_RGX */

	return eError;
}

IMG_VOID LinuxBridgeDeInit(IMG_VOID)
{
#if defined(DEBUG_BRIDGE_KM)
	RemoveProcEntrySeq(g_ProcBridgeStats);
#endif
}

#if defined(DEBUG_BRIDGE_KM)

/*
 * Lock MMap regions list (called on page start/stop while reading /proc/mmap)
 *
 * sfile : seq_file that handles /proc file
 * start : TRUE if it's start, FALSE if it's stop
 *  
 */
static void ProcSeqStartstopBridgeStats(struct seq_file *sfile, IMG_BOOL start)
{
	PVR_UNREFERENCED_PARAMETER(sfile);

	if (start) {
		LinuxLockMutex(&gPVRSRVLock);
	} else {
		LinuxUnLockMutex(&gPVRSRVLock);
	}
}

/*
 * Convert offset (index from KVOffsetTable) to element 
 * (called when reading /proc/mmap file)

 * sfile : seq_file that handles /proc file
 * off : index into the KVOffsetTable from which to print
 *  
 * returns void* : Pointer to element that will be dumped
 *  
*/
static void *ProcSeqOff2ElementBridgeStats(struct seq_file *sfile, loff_t off)
{
	PVR_UNREFERENCED_PARAMETER(sfile);

	if (!off) {
		return PVR_PROC_SEQ_START_TOKEN;
	}

	if (off > BRIDGE_DISPATCH_TABLE_ENTRY_COUNT) {
		return (void *)0;
	}

	return (void *)&g_BridgeDispatchTable[off - 1];
}

/*
 * Gets next MMap element to show. (called when reading /proc/mmap file)

 * sfile : seq_file that handles /proc file
 * el : actual element
 * off : index into the KVOffsetTable from which to print
 *  
 * returns void* : Pointer to element to show (0 ends iteration)
*/
static void *ProcSeqNextBridgeStats(struct seq_file *sfile, void *el,
				    loff_t off)
{
	PVR_UNREFERENCED_PARAMETER(el);

	return ProcSeqOff2ElementBridgeStats(sfile, off);
}

/*
 * Show MMap element (called when reading /proc/mmap file)

 * sfile : seq_file that handles /proc file
 * el : actual element
 *  
*/
static void ProcSeqShowBridgeStats(struct seq_file *sfile, void *el)
{
	PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *psEntry =
	    (PVRSRV_BRIDGE_DISPATCH_TABLE_ENTRY *) el;

	if (el == PVR_PROC_SEQ_START_TOKEN) {
		seq_printf(sfile,
			   "Total ioctl call count = %u\n"
			   "Total number of bytes copied via copy_from_user = %u\n"
			   "Total number of bytes copied via copy_to_user = %u\n"
			   "Total number of bytes copied via copy_*_user = %u\n\n"
			   "%-45s | %-40s | %10s | %20s | %10s\n",
			   g_BridgeGlobalStats.ui32IOCTLCount,
			   g_BridgeGlobalStats.ui32TotalCopyFromUserBytes,
			   g_BridgeGlobalStats.ui32TotalCopyToUserBytes,
			   g_BridgeGlobalStats.ui32TotalCopyFromUserBytes +
			   g_BridgeGlobalStats.ui32TotalCopyToUserBytes,
			   "Bridge Name", "Wrapper Function", "Call Count",
			   "copy_from_user Bytes", "copy_to_user Bytes");
		return;
	}

	seq_printf(sfile,
		   "%-45s   %-40s   %-10u   %-20u   %-10u\n",
		   psEntry->pszIOCName,
		   psEntry->pszFunctionName,
		   psEntry->ui32CallCount,
		   psEntry->ui32CopyFromUserTotalBytes,
		   psEntry->ui32CopyToUserTotalBytes);
}

#endif				/* DEBUG_BRIDGE_KM */

#if defined(SUPPORT_DRI_DRM)
int
PVRSRV_BridgeDispatchKM(struct drm_device *dev, void *arg,
			struct drm_file *pFile)
#else
long
PVRSRV_BridgeDispatchKM(struct file *pFile, unsigned int unref__ ioctlCmd,
			unsigned long arg)
#endif
{
#if !defined(SUPPORT_DRI_DRM)
	PVRSRV_BRIDGE_PACKAGE *psBridgePackageUM =
	    (PVRSRV_BRIDGE_PACKAGE *) arg;
	PVRSRV_BRIDGE_PACKAGE sBridgePackageKM;
#endif
	PVRSRV_BRIDGE_PACKAGE *psBridgePackageKM;
	CONNECTION_DATA *psConnection = LinuxConnectionFromFile(pFile);
	IMG_INT err = -EFAULT;

	LinuxLockMutex(&gPVRSRVLock);

#if defined(SUPPORT_DRI_DRM)
	PVR_UNREFERENCED_PARAMETER(dev);

	psBridgePackageKM = (PVRSRV_BRIDGE_PACKAGE *) arg;
	PVR_ASSERT(psBridgePackageKM != IMG_NULL);
#else
	PVR_UNREFERENCED_PARAMETER(ioctlCmd);

	psBridgePackageKM = &sBridgePackageKM;

	if (!OSAccessOK(PVR_VERIFY_WRITE,
			psBridgePackageUM, sizeof(PVRSRV_BRIDGE_PACKAGE))) {
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: Received invalid pointer to function arguments",
			 __FUNCTION__));

		goto unlock_and_return;
	}

	/* FIXME - Currently the CopyFromUserWrapper which collects stats about
	 * how much data is shifted to/from userspace isn't available to us
	 * here. */
	if (OSCopyFromUser(IMG_NULL,
			   psBridgePackageKM,
			   psBridgePackageUM, sizeof(PVRSRV_BRIDGE_PACKAGE))
	    != PVRSRV_OK) {
		goto unlock_and_return;
	}
#endif

	psBridgePackageKM->ui32BridgeID =
	    PVRSRV_GET_BRIDGE_ID(psBridgePackageKM->ui32BridgeID);

	err = BridgedDispatchKM(psConnection, psBridgePackageKM);

#if !defined(SUPPORT_DRI_DRM)
 unlock_and_return:
#endif
	LinuxUnLockMutex(&gPVRSRVLock);
	return err;
}
