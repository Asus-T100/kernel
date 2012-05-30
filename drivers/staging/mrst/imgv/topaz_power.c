/*
 * Copyright (c) 2009, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Author:binglin.chen@intel.com
 */

#include "topaz_power.h"
#include "lnc_topaz.h"
#include "pnw_topaz.h"
#include "psb_drv.h"
#include "services_headers.h"
#include "sysconfig.h"

static PVRSRV_ERROR DevInitTOPAZPart1(IMG_VOID *pvDeviceNode)
{
#if 1
	PVRSRV_DEVICE_NODE *psDeviceNode = (PVRSRV_DEVICE_NODE *)pvDeviceNode;
	PVRSRV_ERROR eError;
	PVRSRV_DEV_POWER_STATE eDefaultPowerState;

	/* register power operation function */
	/* FIXME: this should be in part2 init function, but
	 * currently here only OSPM needs IMG device... */
	eDefaultPowerState = PVRSRV_DEV_POWER_STATE_OFF;
	eError = PVRSRVRegisterPowerDevice(psDeviceNode->sDevId.ui32DeviceIndex,
					   TOPAZPrePowerState,
					   TOPAZPostPowerState,
					   TOPAZPreClockSpeedChange,
					   TOPAZPostClockSpeedChange,
					   (IMG_HANDLE)psDeviceNode,
					   PVRSRV_DEV_POWER_STATE_ON,
					   eDefaultPowerState);
	if (eError != PVRSRV_OK) {
		PVR_DPF((PVR_DBG_ERROR, "DevInitTOPAZPart1: failed to "
			 "register device with power manager"));
		return eError;
	}
#endif
	return PVRSRV_OK;
}

static PVRSRV_ERROR DevDeInitTOPAZ(IMG_VOID *pvDeviceNode)
{
#if 1
	PVRSRV_DEVICE_NODE *psDeviceNode = (PVRSRV_DEVICE_NODE *)pvDeviceNode;
	PVRSRV_ERROR eError;

	/* should deinit all resource */

	eError = PVRSRVRemovePowerDevice(psDeviceNode->sDevId.ui32DeviceIndex);
	if (eError != PVRSRV_OK)
		return eError;
#endif
	return PVRSRV_OK;
}

PVRSRV_ERROR TOPAZDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	/* version check */

	return PVRSRV_OK;
}

PVRSRV_ERROR TOPAZRegisterDevice(PVRSRV_DEVICE_NODE *psDeviceNode)
{
	psDeviceNode->sDevId.eDeviceType	= PVRSRV_DEVICE_TYPE_TOPAZ;
	psDeviceNode->sDevId.eDeviceClass	= PVRSRV_DEVICE_CLASS_VIDEO;

	psDeviceNode->pfnInitDevice		= DevInitTOPAZPart1;
	psDeviceNode->pfnDeInitDevice		= DevDeInitTOPAZ;

	psDeviceNode->pfnInitDeviceCompatCheck	= TOPAZDevInitCompatCheck;

	if (IS_MRST(gpDrmDevice))
		psDeviceNode->pfnDeviceISR = lnc_topaz_interrupt;
	else
		psDeviceNode->pfnDeviceISR = pnw_topaz_interrupt;
	psDeviceNode->pvISRData = (IMG_VOID *)gpDrmDevice;
	return PVRSRV_OK;
}

PVRSRV_ERROR TOPAZPrePowerState(IMG_HANDLE		hDevHandle,
				PVRSRV_DEV_POWER_STATE	eNewPowerState,
				PVRSRV_DEV_POWER_STATE	eCurrentPowerState)
{
#if 1
	/* ask for a change not power on*/
	if ((eNewPowerState != eCurrentPowerState) &&
	    (eNewPowerState != PVRSRV_DEV_POWER_STATE_ON)) {
		if (IS_MRST(gpDrmDevice)) {
			struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
			struct topaz_private *topaz_priv = dev_priv->topaz_private;
			TOPAZ_NEW_PMSTATE(gpDrmDevice, topaz_priv, PSB_PMSTATE_POWERDOWN);

			/* context save */
			/* context save require irq disable first */
			psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			lnc_topaz_save_mtx_state(gpDrmDevice);

			/* internally close the device */

			/* ask for power off */
			if (eNewPowerState == PVRSRV_DEV_POWER_STATE_OFF) {
				/* here will deinitialize the driver if needed */
				lnc_unmap_topaz_reg(gpDrmDevice);
			} else {
				PVR_DPF((PVR_DBG_MESSAGE,
					 "%s no action for transform from %d to %d",
					 __func__,
					 eCurrentPowerState,
					 eNewPowerState));
			}
		} else {
			struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
			struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;
			PNW_TOPAZ_NEW_PMSTATE(gpDrmDevice, topaz_priv, PSB_PMSTATE_POWERDOWN);

			/* context save */
			/* context save require irq disable first */
			psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			pnw_topaz_save_mtx_state(gpDrmDevice);

			/* internally close the device */

			/* ask for power off */
			if (eNewPowerState == PVRSRV_DEV_POWER_STATE_OFF) {
				/* here will deinitialize the driver if needed */
				/* FIXME: according to latest topaz runtime_pm suspend/resume
				 *        6f78a5b61be55d222fa3880008d2198efa7713bf by Austin, Yuan
				 *        1) Remove map_topaz_reg/unmap_topaz_reg for suspend/resume
				 *        no map/umap topaz register is needed */
				//pnw_unmap_topaz_reg(gpDrmDevice);
			} else {
				PVR_DPF((PVR_DBG_MESSAGE,
					 "%s no action for transform from %d to %d",
					 __func__,
					 eCurrentPowerState,
					 eNewPowerState));
			}
		}

	}
#endif
	return PVRSRV_OK;
}

PVRSRV_ERROR TOPAZPostPowerState(IMG_HANDLE		hDevHandle,
				 PVRSRV_DEV_POWER_STATE	eNewPowerState,
				 PVRSRV_DEV_POWER_STATE	eCurrentPowerState)
{
#if 1
	/* if ask for change & current status is not on */
	if ((eNewPowerState != eCurrentPowerState) &&
	    (eCurrentPowerState != PVRSRV_DEV_POWER_STATE_ON)) {
		if (IS_MRST(gpDrmDevice)) {
			/* internally open device */
			struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
			struct topaz_private *topaz_priv = dev_priv->topaz_private;
			TOPAZ_NEW_PMSTATE(gpDrmDevice, topaz_priv, PSB_PMSTATE_POWERUP);

			if (eCurrentPowerState == PVRSRV_DEV_POWER_STATE_OFF) {
				/* here will initialize the driver if needed */
				lnc_map_topaz_reg(gpDrmDevice);
			} else {
				PVR_DPF((PVR_DBG_MESSAGE,
					 "%s no action for transform from %d to %d",
					 __func__,
					 eCurrentPowerState,
					 eNewPowerState));
			}

			/* context restore */
			psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			lnc_topaz_restore_mtx_state(gpDrmDevice);
			psb_irq_preinstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			psb_irq_postinstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
		} else {
			/* internally open device */
			struct drm_psb_private *dev_priv = gpDrmDevice->dev_private;
			struct pnw_topaz_private *topaz_priv = dev_priv->topaz_private;
			PNW_TOPAZ_NEW_PMSTATE(gpDrmDevice, topaz_priv, PSB_PMSTATE_POWERUP);

			if (eCurrentPowerState == PVRSRV_DEV_POWER_STATE_OFF) {
				/* here will initialize the driver if needed */
				/* FIXME: according to latest topaz runtime_pm suspend/resume
				 *        6f78a5b61be55d222fa3880008d2198efa7713bf by Austin, Yuan
				 *        1) Remove map_topaz_reg/unmap_topaz_reg for suspend/resume
				 *        no map/umap topaz register is needed */
				//pnw_map_topaz_reg(gpDrmDevice);
			} else {
				PVR_DPF((PVR_DBG_MESSAGE,
					 "%s no action for transform from %d to %d",
					 __func__,
					 eCurrentPowerState,
					 eNewPowerState));
			}

			/* context restore */
			psb_irq_uninstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			pnw_topaz_restore_mtx_state(gpDrmDevice);
			psb_irq_preinstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
			psb_irq_postinstall_islands(gpDrmDevice, OSPM_VIDEO_ENC_ISLAND);
		}

	}
#endif
	return PVRSRV_OK;
}

PVRSRV_ERROR TOPAZPreClockSpeedChange(IMG_HANDLE	hDevHandle,
				      IMG_BOOL		bIdleDevice,
				      PVRSRV_DEV_POWER_STATE	eCurrentPowerState)
{
	return PVRSRV_OK;
}

PVRSRV_ERROR TOPAZPostClockSpeedChange(IMG_HANDLE	hDevHandle,
				       IMG_BOOL		bIdleDevice,
				       PVRSRV_DEV_POWER_STATE	eCurrentPowerState)
{
	return PVRSRV_OK;
}
