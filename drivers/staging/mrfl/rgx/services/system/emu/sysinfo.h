/*!****************************************************************************
@File			sysconfig.h
@Title			System Description Header
@Author			Copyright (C) Imagination Technologies Limited.
				All rights reserved. Strictly Confidential.
@Description	Emulator system-specific declarations and macros
******************************************************************************/

#if !defined(__SYSINFO_H__)
#define __SYSINFO_H__

/*!< System specific poll/timeout details */
#define MAX_HW_TIME_US				(5000000)
#define WAIT_TRY_COUNT				(10000)

#define SYS_DEVICE_COUNT 3	/* RGX, DISPLAY (external), BUFFER (external) */
#define SYS_PHYS_HEAP_COUNT 2

#define SYS_RGX_DEV_VENDOR_ID	0x1010
/* PCI emulator board */
#define	SYS_RGX_DEV_DEVICE_ID	0x1CE0
/* PCI Express emulator board */
#define	SYS_RGX_DEV1_DEVICE_ID	0x1CE3

#endif				/* __SYSINFO_H__ */
