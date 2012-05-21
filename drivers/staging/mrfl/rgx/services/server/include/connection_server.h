									     /**************************************************************************//*!
									        @File           connection_server.h
									        @Title          Server side connection management
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    API for server side connection management
    *//***************************************************************************/

#ifndef _CONNECTION_SERVER_H_
#define _CONNECTION_SERVER_H_

#if defined (__cplusplus)
extern "C" {
#endif

#include "img_types.h"
#include "resman.h"

#include "handle.h"

	typedef struct _CONNECTION_DATA_ {
		PRESMAN_CONTEXT hResManContext;
		PVRSRV_HANDLE_BASE *psHandleBase;
#if defined (PVR_SECURE_HANDLES)
		/* Handles are being allocated in batches */
		IMG_BOOL bHandlesBatched;
#endif				/* PVR_SECURE_HANDLES */

		/* True if the process is the initialisation server. */
		IMG_BOOL bInitProcess;

		/*
		 * OS specific data can be stored via this handle.
		 * See osconnection_server.h for a generic mechanism
		 * for initialising this field.
		 */
		IMG_HANDLE hOsPrivateData;

		IMG_PVOID hSecureData;
	} CONNECTION_DATA;

	PVRSRV_ERROR PVRSRVConnectionConnect(IMG_PVOID * ppvPrivData,
					     IMG_PVOID pvOSData);
	IMG_VOID PVRSRVConnectionDisconnect(IMG_PVOID pvPrivData);

	PVRSRV_ERROR PVRSRVConnectionInit(IMG_VOID);
	PVRSRV_ERROR PVRSRVConnectionDeInit(IMG_VOID);

#ifdef INLINE_IS_PRAGMA
#pragma inline(PVRSRVConnectionPrivateData)
#endif
	static INLINE
	    IMG_HANDLE PVRSRVConnectionPrivateData(CONNECTION_DATA *
						   psConnection) {
		return (psConnection !=
			IMG_NULL) ? psConnection->hOsPrivateData : IMG_NULL;
	}
#if defined (__cplusplus)
}
#endif
#endif				/* _CONNECTION_SERVER_H_ */
