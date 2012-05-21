									    /*************************************************************************//*!
									       @File
									       @Title          Linux specific Services code internal interfaces
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    Interfaces between various parts of the Linux specific
									       Services code, that don't have any other obvious
									       header file to go into.
									       @License        Strictly Confidential.
    *//**************************************************************************/
#ifndef __LINKAGE_H__
#define __LINKAGE_H__

#if !defined(SUPPORT_DRI_DRM)
long PVRSRV_BridgeDispatchKM(struct file *file, unsigned int cmd,
			     unsigned long arg);
#endif

IMG_VOID PVRDPFInit(IMG_VOID);
PVRSRV_ERROR PVROSFuncInit(IMG_VOID);
IMG_VOID PVROSFuncDeInit(IMG_VOID);

#ifdef DEBUG

IMG_INT PVRDebugProcSetLevel(struct file *file, const IMG_CHAR * buffer,
			     IMG_UINT32 count, IMG_VOID * data);
void ProcSeqShowDebugLevel(struct seq_file *sfile, void *el);

#ifdef PVR_MANUAL_POWER_CONTROL
IMG_INT PVRProcSetPowerLevel(struct file *file, const IMG_CHAR * buffer,
			     IMG_UINT32 count, IMG_VOID * data);

void ProcSeqShowPowerLevel(struct seq_file *sfile, void *el);

#endif				/* PVR_MANUAL_POWER_CONTROL */

#endif				/* DEBUG */

#endif				/* __LINKAGE_H__ */
/*****************************************************************************
 End of file (linkage.h)
*****************************************************************************/
