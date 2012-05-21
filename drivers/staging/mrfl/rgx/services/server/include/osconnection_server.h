									     /**************************************************************************//*!
									        @File           osconnection_server.h
									        @Title          Server side connection management
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    API for OS specific callbacks from server side connection
									        management
    *//***************************************************************************/
#ifndef _OSCONNECTION_SERVER_H_
#define _OSCONNECTION_SERVER_H_

#include "handle.h"

#if defined (__cplusplus)
extern "C" {
#endif

#if defined(__linux__)
	PVRSRV_ERROR OSConnectionPrivateDataInit(IMG_HANDLE * phOsPrivateData,
						 IMG_PVOID pvOSData);
	PVRSRV_ERROR OSConnectionPrivateDataDeInit(IMG_HANDLE hOsPrivateData);

	PVRSRV_ERROR OSConnectionSetHandleOptions(PVRSRV_HANDLE_BASE *
						  psHandleBase);
#else				/* defined(__linux__) */
#ifdef INLINE_IS_PRAGMA
#pragma inline(OSConnectionPrivateDataInit)
#endif
	static INLINE PVRSRV_ERROR OSConnectionPrivateDataInit(IMG_HANDLE *
							       phOsPrivateData,
							       IMG_PVOID
							       pvOSData) {
		PVR_UNREFERENCED_PARAMETER(phOsPrivateData);
		PVR_UNREFERENCED_PARAMETER(pvOSData);

		return PVRSRV_OK;
	}
#ifdef INLINE_IS_PRAGMA
#pragma inline(OSConnectionPrivateDataDeInit)
#endif
	static INLINE PVRSRV_ERROR OSConnectionPrivateDataDeInit(IMG_HANDLE
								 hOsPrivateData)
	{
		PVR_UNREFERENCED_PARAMETER(hOsPrivateData);

		return PVRSRV_OK;
	}

#ifdef INLINE_IS_PRAGMA
#pragma inline(OSPerProcessSetHandleOptions)
#endif
	static INLINE PVRSRV_ERROR
	    OSConnectionSetHandleOptions(PVRSRV_HANDLE_BASE * psHandleBase) {
		PVR_UNREFERENCED_PARAMETER(psHandleBase);

		return PVRSRV_OK;
	}
#endif				/* defined(__linux__) */

#if defined (__cplusplus)
}
#endif

#endif				/* _OSCONNECTION_SERVER_H_ */
