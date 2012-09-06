/*!****************************************************************************
@File			syscommon.h

@Title			Common System APIs and structures

@Author			Imagination Technologies

@date   		20 October 2003

@Copyright     	Copyright 2003-2005 by Imagination Technologies Limited.
                All rights reserved. No part of this software, either material
                or conceptual may be copied or distributed, transmitted,
                transcribed, stored in a retrieval system or translated into
                any human or computer language in any form by any means,
                electronic, mechanical, manual or other-wise, or disclosed to
                third parties without the express written permission of
                Imagination Technologies Limited, Home Park Estate,
                Kings Langley, Hertfordshire, WD4 8LZ, U.K.

@Platform		generic

@Description	This header provides common system-specific declarations and macros
                that are supported by all system's

@DoxygenVer

******************************************************************************/

#ifndef _SYSCOMMON_H
#define _SYSCOMMON_H

#include "osfunc.h"

#if defined(NO_HARDWARE) && defined(__linux__) && defined(__KERNEL__)
#include <asm/io.h>
#endif

#if defined (__cplusplus)
extern "C" {
#endif

	PVRSRV_ERROR SysCreateConfigData(PVRSRV_SYSTEM_CONFIG ** ppsSysConfig);
	IMG_VOID SysDestroyConfigData(PVRSRV_SYSTEM_CONFIG * psSysConfig);

/*
 * SysReadHWReg and SysWriteHWReg differ from OSReadHWReg and OSWriteHWReg
 * in that they are always intended for use with real hardware, even on
 * NO_HARDWARE systems.
 */
#if !(defined(NO_HARDWARE) && defined(__linux__) && defined(__KERNEL__))
#define	SysReadHWReg(p, o) OSReadHWReg(p, o)
#define SysWriteHWReg(p, o, v) OSWriteHWReg(p, o, v)
#else				/* !(defined(NO_HARDWARE) && defined(__linux__)) */
/*!
******************************************************************************

 @Function	SysReadHWReg

 @Description

 register read function

 @input pvLinRegBaseAddr :	lin addr of register block base

 @input ui32Offset :

 @Return   register value

******************************************************************************/
	static inline IMG_UINT32 SysReadHWReg(IMG_PVOID pvLinRegBaseAddr,
					      IMG_UINT32 ui32Offset) {
		return (IMG_UINT32) readl(pvLinRegBaseAddr + ui32Offset);
	}
/*!
******************************************************************************

 @Function	SysWriteHWReg

 @Description

 register write function

 @input pvLinRegBaseAddr :	lin addr of register block base

 @input ui32Offset :

 @input ui32Value :

 @Return   none

******************************************************************************/
	    static inline IMG_VOID SysWriteHWReg(IMG_PVOID
							 pvLinRegBaseAddr,
							 IMG_UINT32
							 ui32Offset,
							 IMG_UINT32 ui32Value) {
		writel(ui32Value, pvLinRegBaseAddr + ui32Offset);
	}
#endif				/* !(defined(NO_HARDWARE) && defined(__linux__)) */

#if defined(__cplusplus)
}
#endif

#endif

/*****************************************************************************
 End of file (syscommon.h)
*****************************************************************************/
