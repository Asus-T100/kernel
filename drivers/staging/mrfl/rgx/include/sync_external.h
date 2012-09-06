									    /*************************************************************************//*!
									       @File
									       @Title          Services external synchronisation interface header
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Defines synchronisation structures that are visible internally
									       and externally
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "img_types.h"

#ifndef _SYNC_EXTERNAL_
#define _SYNC_EXTERNAL_

typedef IMG_HANDLE SYNC_BRIDGE_HANDLE;
typedef struct _SYNC_PRIM_CONTEXT_ *PSYNC_PRIM_CONTEXT;
typedef struct _SYNC_OP_COOKIE_ *PSYNC_OP_COOKIE;

typedef struct _PVRSRV_CLIENT_SYNC_PRIM_ {
	volatile IMG_UINT32 *pui32LinAddr;	/*!< User pointer to the primitive */
} PVRSRV_CLIENT_SYNC_PRIM;

#if defined(SUPPORT_SECURE_EXPORT)
typedef IMG_SECURE_TYPE PVRSRV_CLIENT_SYNC_PRIM_HANDLE;
#else
typedef IMG_HANDLE PVRSRV_CLIENT_SYNC_PRIM_HANDLE;
#endif

typedef struct _PVRSRV_CLIENT_SYNC_PRIM_OP_ {
	IMG_UINT32 ui32Flags;	/*!< Operation flags */
#define PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK	(1 << 0)
#define PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE	(1 << 1)
	PVRSRV_CLIENT_SYNC_PRIM *psSync;	/*!< Pointer to the client sync */
	IMG_UINT32 ui32FenceValue;	/*!< The Fence value (only used if PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK is set) */
	IMG_UINT32 ui32UpdateValue;	/*!< The Update value (only used if PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE is set) */
} PVRSRV_CLIENT_SYNC_PRIM_OP;

#endif				/* _SYNC_EXTERNAL_ */
