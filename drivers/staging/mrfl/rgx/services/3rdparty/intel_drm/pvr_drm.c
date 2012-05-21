/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#include <drm/drmP.h>
#include <drm/drm.h>

#include "img_defs.h"
#include "mutex.h"
#include "lock.h"
#include "pvr_drm.h"
#include "pvrsrv_interface.h"
#include "pvr_bridge.h"
#include "srvkm.h"
#include "mmap.h"
#include "dc_mrfld.h"
#include "drm_shared.h"
#include "linkage.h"

#if defined(PDUMP)
#include "linuxsrv.h"
#endif

#define PVR_DRM_SRVKM_CMD       DRM_PVR_RESERVED1
#define PVR_DRM_IS_MASTER_CMD   DRM_PVR_RESERVED4
#define PVR_DRM_DBGDRV_CMD      DRM_PVR_RESERVED6

#define PVR_DRM_SRVKM_IOCTL \
	DRM_IOW(DRM_COMMAND_BASE + PVR_DRM_SRVKM_CMD, PVRSRV_BRIDGE_PACKAGE)

#define PVR_DRM_IS_MASTER_IOCTL \
	DRM_IO(DRM_COMMAND_BASE + PVR_DRM_IS_MASTER_CMD)

#if defined(PDUMP)
#define	PVR_DRM_DBGDRV_IOCTL \
	DRM_IOW(DRM_COMMAND_BASE + PVR_DRM_DBGDRV_CMD, IOCTL_PACKAGE)
#endif

static int
PVRDRMIsMaster(struct drm_device *dev, void *arg, struct drm_file *pFile)
{
	return 0;
}

static struct drm_ioctl_desc pvr_ioctls[] = {
	{PVR_DRM_SRVKM_IOCTL, 0, PVRSRV_BridgeDispatchKM},
	{PVR_DRM_IS_MASTER_IOCTL, DRM_MASTER, PVRDRMIsMaster},
#if defined(PDUMP)
	{PVR_DRM_DBGDRV_IOCTL, 0, dbgdrv_ioctl}
#endif
};

DECLARE_WAIT_QUEUE_HEAD(sWaitForInit);

static bool bInitComplete;
static bool bInitFailed;

#if !defined(PVR_DRI_DRM_NOT_PCI)
struct pci_dev *gpsPVRLDMDev;
#endif

struct drm_device *gpsPVRDRMDev;

#define PVR_DRM_FILE struct drm_file *

int PVRSRVDrmLoad(struct drm_device *dev, unsigned long flags)
{
	int iRes = 0;

	/* Init the mutex lock gsDebugMutexNonIRQ */
	PVRDPFInit();

	DRM_DEBUG("PVRSRVDrmLoad");

	gpsPVRDRMDev = dev;
#if !defined(PVR_DRI_DRM_NOT_PCI)
	gpsPVRLDMDev = dev->pdev;
#endif

#if defined(PDUMP)
	iRes = dbgdrv_init();
	if (iRes != 0) {
		goto exit;
	}
#endif

	iRes = PVRCore_Init();
	if (iRes != 0) {
		goto exit_dbgdrv_cleanup;
	}

	if (MerrifieldDCInit(dev) != PVRSRV_OK) {
		DRM_ERROR("%s: display class init failed\n", __FUNCTION__);
		goto exit_pvrcore_cleanup;
	}

	goto exit;

 exit_pvrcore_cleanup:
	PVRCore_Cleanup();

 exit_dbgdrv_cleanup:
#if defined(PDUMP)
	dbgdrv_cleanup();
#endif
 exit:
	if (iRes != 0) {
		bInitFailed = true;
	}
	bInitComplete = true;

	wake_up_interruptible(&sWaitForInit);

	return iRes;
}

int PVRSRVDrmUnload(struct drm_device *dev)
{
	DRM_DEBUG("PVRSRVDrmUnload");

	if (MerrifieldDCDeinit() != PVRSRV_OK) {
		DRM_ERROR("%s: can't deinit display class\n", __FUNCTION__);
	}

	PVRCore_Cleanup();

#if defined(PDUMP)
	dbgdrv_cleanup();
#endif

	return 0;
}

int PVRSRVDrmOpen(struct drm_device *dev, struct drm_file *file)
{
	while (!bInitComplete) {
		DEFINE_WAIT(sWait);

		prepare_to_wait(&sWaitForInit, &sWait, TASK_INTERRUPTIBLE);

		if (!bInitComplete) {
			DRM_DEBUG
			    ("%s: Waiting for module initialisation to complete",
			     __FUNCTION__);

			schedule();
		}

		finish_wait(&sWaitForInit, &sWait);

		if (signal_pending(current)) {
			return -ERESTARTSYS;
		}
	}

	if (bInitFailed) {
		DRM_DEBUG("%s: Module initialisation failed", __FUNCTION__);
		return -EINVAL;
	}

	return PVRSRVOpen(dev, file);
}

#if defined(SUPPORT_DRI_DRM_EXT) && !defined(PVR_LINUX_USING_WORKQUEUES)
void PVRSRVDrmPostClose(struct drm_device *dev, struct drm_file *file)
{
	PVRSRVRelease(file);

	file->driver_priv = NULL;
}
#else
int PVRSRVDrmRelease(struct inode *inode, struct file *filp)
{
	struct drm_file *file_priv = filp->private_data;
	void *psDriverPriv = file_priv->driver_priv;
	int ret;

	ret = drm_release(inode, filp);

	if (ret != 0) {

		DRM_DEBUG("%s : drm_release failed: %d", __FUNCTION__, ret);
	}

	PVRSRVRelease(filp);

	return 0;
}
#endif

void PVRSRVQueryIoctls(struct drm_ioctl_desc *ioctls)
{
	int i;

	for (i = 0; i < DRM_ARRAY_SIZE(pvr_ioctls); i++) {
		unsigned int slot =
		    DRM_IOCTL_NR(pvr_ioctls[i].cmd) - DRM_COMMAND_BASE;
		ioctls[slot] = pvr_ioctls[i];
	}
}

/* FIXME: ALEX. This func might need rework. */
unsigned int PVRSRVGetMeminfoSize(void *hMemHandle)
{
	return 0;
}

/* FIXME: ALEX. This func might need rework. */
void *PVRSRVGetMeminfoCPUAddr(void *hMemHandle)
{
	return 0;
}

/* FIXME: ALEX. To be implemented. */
int PVRSRVGetMeminfoPages(void *hMemHandle, struct page ***pages)
{
	return 0;
}

/* FIXME: ALEX. To be implemented. */
int PVRSRVInterrupt(struct drm_device *dev)
{
	return 1;
}

int PVRSRVMMap(struct file *pFile, struct vm_area_struct *ps_vma)
{
	return MMapPMR(pFile, ps_vma);
}
