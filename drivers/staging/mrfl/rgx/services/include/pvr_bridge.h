									    /*************************************************************************//*!
									       @File
									       @Title          PVR Bridge Functionality
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Header for the PVR Bridge code
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __PVR_BRIDGE_H__
#define __PVR_BRIDGE_H__

#if defined (__cplusplus)
extern "C" {
#endif

#include "cache_defines.h"

#include "common_dc_bridge.h"
#include "common_mm_bridge.h"
#include "common_cmm_bridge.h"
#include "common_pdumpmm_bridge.h"
#include "common_pdumpcmm_bridge.h"
#if defined(SUPPORT_PMMIF)
#include "common_pmmif_bridge.h"
#endif
#include "common_pdump_bridge.h"
#include "common_srvcore_bridge.h"
#include "common_sync_bridge.h"
#if !defined(SUPPORT_SECURE_EXPORT)
#include "common_syncexport_bridge.h"
#else
#include "common_syncsexport_bridge.h"
#endif
#if (CACHEFLUSH_TYPE == CACHEFLUSH_GENERIC)
#include "common_cachegeneric_bridge.h"
#endif
#if defined(SUPPORT_SECURE_EXPORT)
#include "common_smm_bridge.h"
#endif

/* 
 * Bridge Cmd Ids
 */

/* FIXME remove anything OS-specific from this file */

#ifdef __linux__

#include <linux/ioctl.h>
	/*!< Nov 2006: according to ioctl-number.txt 'g' wasn't in use. */
#define PVRSRV_IOC_GID      'g'
#define PVRSRV_IO(INDEX)    _IO(PVRSRV_IOC_GID, INDEX, PVRSRV_BRIDGE_PACKAGE)
#define PVRSRV_IOW(INDEX)   _IOW(PVRSRV_IOC_GID, INDEX, PVRSRV_BRIDGE_PACKAGE)
#define PVRSRV_IOR(INDEX)   _IOR(PVRSRV_IOC_GID, INDEX, PVRSRV_BRIDGE_PACKAGE)
#define PVRSRV_IOWR(INDEX)  _IOWR(PVRSRV_IOC_GID, INDEX, PVRSRV_BRIDGE_PACKAGE)

#else				/* __linux__ */

#if defined(UNDER_VISTA)
#define PVRSRV_IOC_GID          (0x800UL)	/*!< (see ioctldef.h for details) */
#else
#error Unknown platform: Cannot define ioctls
#endif

#define PVRSRV_IO(INDEX)    (PVRSRV_IOC_GID + (INDEX))
#define PVRSRV_IOW(INDEX)   (PVRSRV_IOC_GID + (INDEX))
#define PVRSRV_IOR(INDEX)   (PVRSRV_IOC_GID + (INDEX))
#define PVRSRV_IOWR(INDEX)  (PVRSRV_IOC_GID + (INDEX))

#define PVRSRV_BRIDGE_BASE                  PVRSRV_IOC_GID
#endif				/* __linux__ */

/* 
 * Note *REMEMBER* to update PVRSRV_BRIDGE_LAST_CMD (below) if you add any new
 * bridge commands!
 */

#define PVRSRV_BRIDGE_CORE_CMD_FIRST			0UL

/* Core functions */
#define PVRSRV_BRIDGE_SRVCORE_START				(PVRSRV_BRIDGE_CORE_CMD_FIRST)

/* Sync functions */
#define PVRSRV_BRIDGE_SYNC_START				(PVRSRV_BRIDGE_SRVCORE_CMD_LAST + 1)

#if !defined(SUPPORT_SECURE_EXPORT)
#define PVRSRV_BRIDGE_SYNCEXPORT_START			(PVRSRV_BRIDGE_SYNC_CMD_LAST + 1)
#define SYNCEXPORT_LAST							PVRSRV_BRIDGE_SYNCEXPORT_CMD_LAST
#else
#define PVRSRV_BRIDGE_SYNCSEXPORT_START			(PVRSRV_BRIDGE_SYNC_CMD_LAST + 1)
#define SYNCEXPORT_LAST							PVRSRV_BRIDGE_SYNCSEXPORT_CMD_LAST
#endif

/* PDUMP */
#define PVRSRV_BRIDGE_PDUMP_START				(SYNCEXPORT_LAST + 1)

/* Memory Management */
#define PVRSRV_BRIDGE_MM_START      		(PVRSRV_BRIDGE_PDUMP_CMD_LAST + 1)
#define PVRSRV_BRIDGE_CMM_START      		(PVRSRV_BRIDGE_MM_CMD_LAST + 1)
#define PVRSRV_BRIDGE_PDUMPMM_START      	(PVRSRV_BRIDGE_CMM_CMD_LAST + 1)
#define PVRSRV_BRIDGE_PDUMPCMM_START      	(PVRSRV_BRIDGE_PDUMPMM_CMD_LAST + 1)
#define PVRSRV_BRIDGE_PMMIF_START			(PVRSRV_BRIDGE_PDUMPCMM_CMD_LAST + 1)
#if !defined(SUPPORT_PMMIF)
#define PVRSRV_BRIDGE_PMMIF_CMD_LAST		(PVRSRV_BRIDGE_PMMIF_START - 1)
#endif

/* Display Class */
#define PVRSRV_BRIDGE_DC_START				(PVRSRV_BRIDGE_PMMIF_CMD_LAST + 1)

/* Generic cache interface */
#define PVRSRV_BRIDGE_CACHEGENERIC_START	(PVRSRV_BRIDGE_DC_CMD_LAST + 1)
#if (CACHEFLUSH_TYPE != CACHEFLUSH_GENERIC)
#define PVRSRV_BRIDGE_CACHEGENERIC_CMD_LAST		(PVRSRV_BRIDGE_CACHEGENERIC_START - 1)
#endif

#define PVRSRV_BRIDGE_SMM_START				(PVRSRV_BRIDGE_CACHEGENERIC_CMD_LAST + 1)
#if !defined(SUPPORT_SECURE_EXPORT)
#define PVRSRV_BRIDGE_SMM_CMD_LAST			(PVRSRV_BRIDGE_SMM_START - 1)
#endif

/* For sgx_bridge.h (msvdx_bridge.h should probably use these defines too) */
/* "last" below means last+1 (first beyond last) */
#define PVRSRV_BRIDGE_LAST_NON_DEVICE_CMD       (PVRSRV_BRIDGE_SMM_CMD_LAST)

/******************************************************************************
 * Generic bridge structures 
 *****************************************************************************/

/******************************************************************************
 *	bridge packaging structure
 *****************************************************************************/
	typedef struct PVRSRV_BRIDGE_PACKAGE_TAG {
		IMG_UINT32 ui32BridgeID;	/*!< ioctl/drvesc index */
		IMG_UINT32 ui32Size;	/*!< size of structure */
		IMG_VOID *pvParamIn;	/*!< input data buffer */
		IMG_UINT32 ui32InBufferSize;	/*!< size of input data buffer */
		IMG_VOID *pvParamOut;	/*!< output data buffer */
		IMG_UINT32 ui32OutBufferSize;	/*!< size of output data buffer */
	} PVRSRV_BRIDGE_PACKAGE;

#if defined (__cplusplus)
}
#endif
#endif				/* __PVR_BRIDGE_H__ */
/******************************************************************************
 End of file (pvr_bridge.h)
******************************************************************************/
