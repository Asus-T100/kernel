/*!****************************************************************************
@File			sysinfo.h

@Title			System Description Header

@Author			Imagination Technologies

@date   		6 August 2004
 
@Copyright     	Copyright 2003-2004 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either
                material or conceptual may be copied or distributed,
                transmitted, transcribed, stored in a retrieval system
                or translated into any human or computer language in any
                form by any means, electronic, mechanical, manual or
                other-wise, or disclosed to third parties without the
                express written permission of Imagination Technologies
                Limited, Unit 8, HomePark Industrial Estate,
                King's Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform		generic

@Description	This header provides system-specific declarations and macros

@DoxygenVer		

******************************************************************************/

#if !defined(__SYSINFO_H__)
#define __SYSINFO_H__

/*!< System specific poll/timeout details */
#define MAX_HW_TIME_US				(500000)
#define WAIT_TRY_COUNT				(10000)

#define SYS_DEVICE_COUNT 3	/* RGX, DISPLAY (external), BUFFER (external) */

#define SYS_PHYS_HEAP_COUNT		1

#if defined(__linux__)
#define SYS_RGX_DEV_NAME    "rgxnohw"
#if defined(SUPPORT_DRI_DRM)
/*
 * Use the static bus ID for the platform DRM device.
 */
#if defined(PVR_DRI_DRM_DEV_BUS_ID)
#define	SYS_RGX_DEV_DRM_BUS_ID	PVR_DRI_DRM_DEV_BUS_ID
#else
#define SYS_RGX_DEV_DRM_BUS_ID	"platform:rgxnohw"
#endif				/* defined(PVR_DRI_DRM_DEV_BUS_ID) */
#endif				/* defined(SUPPORT_DRI_DRM) */
#endif

#endif				/* __SYSINFO_H__ */
