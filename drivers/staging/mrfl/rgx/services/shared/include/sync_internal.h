									     /**************************************************************************//*!
									        @File           sync_internal.h
									        @Title          Services internal synchronisation interface header
									        @Date
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Defines the internal client side interface for services
									        synchronisation
    *//***************************************************************************/

#include "img_types.h"
#include "sync_external.h"

#ifndef _SYNC_INTERNAL_
#define _SYNC_INTERNAL_

/* FIXME this must return a correctly typed pointer */
IMG_INTERNAL IMG_UINT32 SyncPrimGetFirmwareAddr(PVRSRV_CLIENT_SYNC_PRIM *
						psSync);

#endif				/* _SYNC_INTERNAL_ */
