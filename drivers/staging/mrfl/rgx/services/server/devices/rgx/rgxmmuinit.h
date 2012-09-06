									    /*************************************************************************//*!
									       @File
									       @Title          Device specific initialisation routines
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Device specific MMU initialisation
									       @License        Strictly Confidential.
    *//**************************************************************************/

/* NB: this file is not to be included arbitrarily.  It exists solely
   for the linkage between rgxinit.c and rgxmmuinit.c, the former
   being otherwise cluttered by the contents of the latter */

#ifndef _SRVKM_RGXMMUINIT_H_
#define _SRVKM_RGXMMUINIT_H_

/* services5/srvkm/include/ */
#include "device.h"
#include "img_types.h"
#include "mmu_common.h"
/* include5/ */
#include "img_defs.h"

IMG_EXPORT PVRSRV_ERROR RGXMMUInit_Register(PVRSRV_DEVICE_NODE * psDeviceNode);
IMG_EXPORT PVRSRV_ERROR RGXMMUInit_Unregister(PVRSRV_DEVICE_NODE *
					      psDeviceNode);

#endif				/* #ifndef _SRVKM_RGXMMUINIT_H_ */
