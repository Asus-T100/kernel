									    /*************************************************************************//*!
									       @File
									       @Title          Parameter dump internal common functions
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __PDUMP_INT_H__
#define __PDUMP_INT_H__

#if defined (__cplusplus)
extern "C" {
#endif

/*
 *	This file contains internal pdump utility functions which may be accessed
 *	from OS-specific code. The header should not be included outside of srvkm
 *	pdump files.
 */
#include "dbgdrvif.h"

/* Callbacks which are registered with the debug driver. */
	IMG_EXPORT IMG_VOID PDumpConnectionNotify(IMG_VOID);

	typedef enum __PDUMP_DDWMODE {
		/* Continuous writes are always captured in the dbgdrv; the buffer will
		 * expand if no client/sink process is running.
		 */
		PDUMP_WRITE_MODE_CONTINUOUS = 0,
		/* Last frame capture */
		PDUMP_WRITE_MODE_LASTFRAME,
		/* Capture frame, binary data */
		PDUMP_WRITE_MODE_BINCM,
		/* Persistent capture, append data to init phase */
		PDUMP_WRITE_MODE_PERSISTENT
	} PDUMP_DDWMODE;

	IMG_UINT32 DbgWrite(PDBG_STREAM psStream, IMG_UINT8 * pui8Data,
			    IMG_UINT32 ui32BCount, IMG_UINT32 ui32Flags);

	IMG_UINT32 PDumpOSDebugDriverWrite(PDBG_STREAM psStream,
					   PDUMP_DDWMODE eDbgDrvWriteMode,
					   IMG_UINT8 * pui8Data,
					   IMG_UINT32 ui32BCount,
					   IMG_UINT32 ui32Level,
					   IMG_UINT32 ui32DbgDrvFlags);

#if defined (__cplusplus)
}
#endif
#endif				/* __PDUMP_INT_H__ */
/******************************************************************************
 End of file (pdump_int.h)
******************************************************************************/
