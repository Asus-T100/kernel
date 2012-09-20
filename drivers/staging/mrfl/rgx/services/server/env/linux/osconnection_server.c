									    /*************************************************************************//*!
									       @File
									       @Title          Linux specific per process data functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#include "connection_server.h"
#include "osconnection_server.h"

#include "env_connection.h"
#include "proc.h"
#include "allocmem.h"
#include "pvr_debug.h"

PVRSRV_ERROR OSConnectionPrivateDataInit(IMG_HANDLE * phOsPrivateData,
					 IMG_PVOID pvOSData)
{
	ENV_CONNECTION_DATA *psEnvConnection;

	*phOsPrivateData = OSAllocMem(sizeof(ENV_CONNECTION_DATA));

	if (*phOsPrivateData == IMG_NULL) {
		PVR_DPF((PVR_DBG_ERROR, "%s: OSAllocMem failed", __FUNCTION__));
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psEnvConnection = (ENV_CONNECTION_DATA *) * phOsPrivateData;
	OSMemSet(psEnvConnection, 0, sizeof(*psEnvConnection));

	/* Save the pointer to our struct file */
	psEnvConnection->psFile = pvOSData;

#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
	/* Linked list of PVRSRV_FILE_PRIVATE_DATA structures */
	INIT_LIST_HEAD(&psEnvConnection->sDRMAuthListHead);
#endif

	return PVRSRV_OK;
}

PVRSRV_ERROR OSConnectionPrivateDataDeInit(IMG_HANDLE hOsPrivateData)
{
	/* ENV_CONNECTION_DATA *psEnvConnection; */

	if (hOsPrivateData == IMG_NULL) {
		return PVRSRV_OK;
	}

	/* psEnvConnection = hOsPrivateData; */

	/* Remove per process /proc entries */
	/* FIXME: How does connection stuff map into /proc? */
	/*RemoveConnectionProcDir(psEnvConnection); */

	OSFreeMem(hOsPrivateData);
	/*not nulling pointer, copy on stack */

	return PVRSRV_OK;
}
