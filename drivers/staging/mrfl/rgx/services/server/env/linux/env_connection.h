									     /**************************************************************************//*!
									        @File           env_connection.h
									        @Title          Server side connection management
									        @Author         Copyright (C) Imagination Technologies Limited.
									        All rights reserved. Strictly Confidential.
									        @Description    Linux specific server side connection management
    *//***************************************************************************/

#ifndef _ENV_CONNECTION_H_
#define _ENV_CONNECTION_H_

#include <linux/list.h>
#include <linux/proc_fs.h>

#include "handle.h"

typedef struct _ENV_CONNECTION_DATA_ {
	struct file *psFile;

	struct proc_dir_entry *psProcDir;
#if defined(SUPPORT_DRI_DRM) && defined(PVR_SECURE_DRM_AUTH_EXPORT)
	struct list_head sDRMAuthListHead;
#endif
} ENV_CONNECTION_DATA;

#endif				/* _ENV_CONNECTION_H_ */
