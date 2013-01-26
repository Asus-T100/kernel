/*************************************************************************/ /*!
@File
@Title          PVR Bridge Functionality
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Header for the PVR Bridge code
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
#if defined(SUPPORT_ION)
#include "common_ion_bridge.h"
#endif
#include "common_pdump_bridge.h"
#include "common_srvcore_bridge.h"
#include "common_sync_bridge.h"
#if defined(SUPPORT_INSECURE_EXPORT)
#include "common_syncexport_bridge.h"
#endif
#if defined(SUPPORT_SECURE_EXPORT)
#include "common_syncsexport_bridge.h"
#endif
#if (CACHEFLUSH_TYPE == CACHEFLUSH_GENERIC)
#include "common_cachegeneric_bridge.h"
#endif
#if defined(SUPPORT_SECURE_EXPORT)
#include "common_smm_bridge.h"
#endif
#include "common_pvrtl_bridge.h"

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

#else /* __linux__ */

	#if defined(UNDER_WDDM)
		#define PVRSRV_IOC_GID          (0x800UL)			/*!< (see ioctldef.h for details) */
	#else
		#error Unknown platform: Cannot define ioctls
	#endif

	#define PVRSRV_IO(INDEX)    (PVRSRV_IOC_GID + (INDEX))
	#define PVRSRV_IOW(INDEX)   (PVRSRV_IOC_GID + (INDEX))
	#define PVRSRV_IOR(INDEX)   (PVRSRV_IOC_GID + (INDEX))
	#define PVRSRV_IOWR(INDEX)  (PVRSRV_IOC_GID + (INDEX))

	#define PVRSRV_BRIDGE_BASE                  PVRSRV_IOC_GID
#endif /* __linux__ */


/* Note: The pattern
 *   #if !defined(SUPPORT_FEATURE)
 *   #define PVRSRV_BRIDGE_FEATURE_CMD_LAST	(PVRSRV_BRIDGE_FEATURE_START - 1)
 *   #endif
 * is used in the macro definitions below to make PVRSRV_BRIDGE_FEATURE_*
 * take up no command numbers if SUPPORT_FEATURE is disabled. If
 * SUPPORT_FEATURE is enabled, the last command number will be provided by
 * its header (common_feature_bridge.h).
 */

#define PVRSRV_BRIDGE_CORE_CMD_FIRST			0UL

/* Core functions */
#define PVRSRV_BRIDGE_SRVCORE_START				(PVRSRV_BRIDGE_CORE_CMD_FIRST)

/* Sync functions */
#define PVRSRV_BRIDGE_SYNC_START				(PVRSRV_BRIDGE_SRVCORE_CMD_LAST + 1)

#define PVRSRV_BRIDGE_SYNCEXPORT_START			(PVRSRV_BRIDGE_SYNC_CMD_LAST + 1)
#if !defined(SUPPORT_INSECURE_EXPORT)
#define PVRSRV_BRIDGE_SYNCEXPORT_CMD_LAST		(PVRSRV_BRIDGE_SYNCEXPORT_START - 1)
#endif
#define PVRSRV_BRIDGE_SYNCSEXPORT_START			(PVRSRV_BRIDGE_SYNCEXPORT_CMD_LAST + 1)
#if !defined(SUPPORT_SECURE_EXPORT)
#define PVRSRV_BRIDGE_SYNCSEXPORT_CMD_LAST		(PVRSRV_BRIDGE_SYNCSEXPORT_START - 1)
#endif

/* PDUMP */
#define PVRSRV_BRIDGE_PDUMP_START				(PVRSRV_BRIDGE_SYNCSEXPORT_CMD_LAST + 1)

/* Memory Management */
#define PVRSRV_BRIDGE_MM_START      		(PVRSRV_BRIDGE_PDUMP_CMD_LAST + 1)
#define PVRSRV_BRIDGE_CMM_START      		(PVRSRV_BRIDGE_MM_CMD_LAST + 1)
#define PVRSRV_BRIDGE_PDUMPMM_START      	(PVRSRV_BRIDGE_CMM_CMD_LAST + 1)
#define PVRSRV_BRIDGE_PDUMPCMM_START      	(PVRSRV_BRIDGE_PDUMPMM_CMD_LAST + 1)
#define PVRSRV_BRIDGE_PMMIF_START			(PVRSRV_BRIDGE_PDUMPCMM_CMD_LAST + 1)
#if !defined(SUPPORT_PMMIF)
#define PVRSRV_BRIDGE_PMMIF_CMD_LAST		(PVRSRV_BRIDGE_PMMIF_START - 1)
#endif
#define PVRSRV_BRIDGE_ION_START				(PVRSRV_BRIDGE_PMMIF_CMD_LAST + 1)
#if !defined(SUPPORT_ION)
#define PVRSRV_BRIDGE_ION_CMD_LAST			(PVRSRV_BRIDGE_ION_START - 1)
#endif

/* Display Class */
#define PVRSRV_BRIDGE_DC_START				(PVRSRV_BRIDGE_ION_CMD_LAST + 1)

/* Generic cache interface */
#define PVRSRV_BRIDGE_CACHEGENERIC_START	(PVRSRV_BRIDGE_DC_CMD_LAST + 1)
#if (CACHEFLUSH_TYPE != CACHEFLUSH_GENERIC)
#define PVRSRV_BRIDGE_CACHEGENERIC_CMD_LAST		(PVRSRV_BRIDGE_CACHEGENERIC_START - 1)
#endif

#define PVRSRV_BRIDGE_SMM_START				(PVRSRV_BRIDGE_CACHEGENERIC_CMD_LAST + 1)
#if !defined(SUPPORT_SECURE_EXPORT)
#define PVRSRV_BRIDGE_SMM_CMD_LAST			(PVRSRV_BRIDGE_SMM_START - 1)
#endif


/* Transport Layer interface */
#define PVRSRV_BRIDGE_PVRTL_START				(PVRSRV_BRIDGE_SMM_CMD_LAST + 1)


/* For rgx_bridge.h. "last" below means last+1 (first beyond last) */
#define PVRSRV_BRIDGE_LAST_NON_DEVICE_CMD       (PVRSRV_BRIDGE_PVRTL_CMD_LAST)

/******************************************************************************
 * Generic bridge structures 
 *****************************************************************************/


/******************************************************************************
 *	bridge packaging structure
 *****************************************************************************/
typedef struct PVRSRV_BRIDGE_PACKAGE_TAG
{
	IMG_UINT32				ui32BridgeID;			/*!< ioctl/drvesc index */
	IMG_UINT32				ui32Size;				/*!< size of structure */
	IMG_VOID				*pvParamIn;				/*!< input data buffer */ 
	IMG_UINT32				ui32InBufferSize;		/*!< size of input data buffer */
	IMG_VOID				*pvParamOut;			/*!< output data buffer */
	IMG_UINT32				ui32OutBufferSize;		/*!< size of output data buffer */
}PVRSRV_BRIDGE_PACKAGE;


#if defined (__cplusplus)
}
#endif

#endif /* __PVR_BRIDGE_H__ */

/******************************************************************************
 End of file (pvr_bridge.h)
******************************************************************************/

