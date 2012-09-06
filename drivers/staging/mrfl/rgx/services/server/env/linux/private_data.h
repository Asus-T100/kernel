									    /*************************************************************************//*!
									       @File
									       @Title          Linux private data structure
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __INCLUDED_PRIVATE_DATA_H_
#define __INCLUDED_PRIVATE_DATA_H_

#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
#include <linux/list.h>
#include <drm/drmP.h>
#endif

#include <linux/fs.h>

#include "connection_server.h"
#include "env_connection.h"

/* This structure is required in the rare case that a process creates
 * a connection to services, but before closing the file descriptor,
 * does a fork(). This fork() will duplicate the file descriptor in the
 * child process. If the parent process dies before the child, this can
 * cause the PVRSRVRelease() method to be called in a different process
 * context than the original PVRSRVOpen(). This is bad because we need
 * to update the per-process data reference count and/or free the
 * per-process data. So we must keep a record of which PID's per-process
 * data to inspect during ->release().
 */

typedef struct {
	IMG_PVOID pvConnectionData;

#if defined(PVR_SECURE_FD_EXPORT)
	/* Global kernel MemInfo handle */
	IMG_HANDLE hKernelMemInfo;
#endif				/* defined(PVR_SECURE_FD_EXPORT) */

#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
	/* The private data is on a list in the per-process data structure */
	struct list_head sDRMAuthListItem;

	struct drm_file *psDRMFile;
#endif

#if defined(SUPPORT_DRI_DRM_EXT)
	IMG_PVOID pPriv;	/*private data for extending this struct */
#endif
} PVRSRV_FILE_PRIVATE_DATA;

#if defined(SUPPORT_DRI_DRM)
CONNECTION_DATA *LinuxConnectionFromFile(struct drm_file *pFile);
#else
CONNECTION_DATA *LinuxConnectionFromFile(struct file *pFile);
#endif

struct file *LinuxFileFromEnvConnection(ENV_CONNECTION_DATA * psEnvConnection);

#endif				/* __INCLUDED_PRIVATE_DATA_H_ */
