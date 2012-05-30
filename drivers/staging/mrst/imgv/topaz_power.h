/*
** topaz_power.h
** Login : <binglin.chen@intel.com>
** Started on  Mon Nov 16 13:31:42 2009 brady
**
** Copyright (C) 2009 brady
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#ifndef TOPAZ_POWER_H_
#define TOPAZ_POWER_H_

#include "services_headers.h"
#include "sysconfig.h"

extern struct drm_device *gpDrmDevice;

/* function define */
PVRSRV_ERROR TOPAZRegisterDevice(PVRSRV_DEVICE_NODE *psDeviceNode);
PVRSRV_ERROR TOPAZDevInitCompatCheck(PVRSRV_DEVICE_NODE *psDeviceNode);

/* power function define */
PVRSRV_ERROR TOPAZPrePowerState(
	IMG_HANDLE		hDevHandle,
	PVRSRV_DEV_POWER_STATE	eNewPowerState,
	PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR TOPAZPostPowerState(
	IMG_HANDLE		hDevHandle,
	PVRSRV_DEV_POWER_STATE	eNewPowerState,
	PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR TOPAZPreClockSpeedChange(
	IMG_HANDLE		hDevHandle,
	IMG_BOOL		bIdleDevice,
	PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR TOPAZPostClockSpeedChange(
	IMG_HANDLE		hDevHandle,
	IMG_BOOL		bIdleDevice,
	PVRSRV_DEV_POWER_STATE	eCurrentPowerState);
PVRSRV_ERROR TOPAZInitOSPM(PVRSRV_DEVICE_NODE *psDeviceNode);

#endif /* !TOPAZ_POWER_H_ */
