									    /*************************************************************************//*!
									       @File
									       @Title          Environmental Data header file
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Linux-specific part of system data.
									       @License        Strictly Confidential.
    *//**************************************************************************/
#ifndef _ENV_DATA_
#define _ENV_DATA_

#include <linux/interrupt.h>
#include <linux/pci.h>

#if defined(PVR_LINUX_MISR_USING_WORKQUEUE) || defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE)
#include <linux/workqueue.h>
#endif

/* 
 *	Env data specific to linux - convenient place to put this
 */

/* Fairly arbitrary sizes - hopefully enough for all bridge calls */
#define PVRSRV_MAX_BRIDGE_IN_SIZE	0x2000
#define PVRSRV_MAX_BRIDGE_OUT_SIZE	0x1000

typedef struct _ENV_DATA_TAG {
	IMG_VOID *pvBridgeData;
	struct pm_dev *psPowerDevice;
} ENV_DATA;

ENV_DATA *OSGetEnvData(IMG_VOID);

#endif				/* _ENV_DATA_ */
/*****************************************************************************
 End of file (env_data.h)
*****************************************************************************/
