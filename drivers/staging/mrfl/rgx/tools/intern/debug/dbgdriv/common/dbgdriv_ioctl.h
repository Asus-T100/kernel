									    /*************************************************************************//*!
									       @File
									       @Title          32 Bit Highlander kernel manager VxD Services
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef _IOCTL_
#define _IOCTL_

/*****************************************************************************
 Global vars
*****************************************************************************/

#define MAX_DBGVXD_W32_API 25

extern IMG_UINT32(*g_DBGDrivProc[MAX_DBGVXD_W32_API]) (IMG_VOID *, IMG_VOID *);

#endif

/*****************************************************************************
 End of file (IOCTL.H)
*****************************************************************************/
