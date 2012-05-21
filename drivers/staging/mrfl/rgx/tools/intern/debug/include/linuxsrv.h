/**************************************************************************
 * Name         : $RCSfile: linuxsrv.h $
 * Title        : linuxsrv.h
 * Author       : Jim Page
 * Created      : 23/11/2000
 *
 * Copyright    : 2000-2006 by Imagination Technologies Ltd. All rights reserved.
 *              : No part of this software, either material or conceptual 
 *              : may be copied or distributed, transmitted, transcribed,
 *              : stored in a retrieval system or translated into any 
 *              : human or computer language in any form by any means,
 *              : electronic, mechanical, manual or other-wise, or 
 *              : disclosed to third parties without the express written
 *              : permission of Imagination Technologies Limited, Unit 8, HomePark
 *              : Industrial Estate, King's Langley, Hertfordshire,
 *              : WD4 8LZ, U.K.
 *
 * Description  : module defs for pvr core drivers
 *
 * Platform     : linux
 *
 * Modifications:-
 *
 **************************************************************************/

#ifndef _LINUXSRV_H__
#define _LINUXSRV_H__

typedef struct tagIOCTL_PACKAGE {
	IMG_UINT32 ui32Cmd;	// ioctl command
	IMG_UINT32 ui32Size;	// needs to be correctly set
	IMG_VOID *pInBuffer;	// input data buffer
	IMG_UINT32 ui32InBufferSize;	// size of input data buffer
	IMG_VOID *pOutBuffer;	// output data buffer
	IMG_UINT32 ui32OutBufferSize;	// size of output data buffer
} IOCTL_PACKAGE;

IMG_UINT32 DeviceIoControl(IMG_UINT32 hDevice,
			   IMG_UINT32 ui32ControlCode,
			   IMG_VOID * pInBuffer,
			   IMG_UINT32 ui32InBufferSize,
			   IMG_VOID * pOutBuffer,
			   IMG_UINT32 ui32OutBufferSize,
			   IMG_UINT32 * pui32BytesReturned);

#endif				/* _LINUXSRV_H__ */
