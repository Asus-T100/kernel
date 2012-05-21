									    /*************************************************************************//*!
									       @File
									       @Title          Main driver lock
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @Description    The main driver lock, held in most places in
									       the driver.
									       @License        Strictly Confidential.
    *//**************************************************************************/
#ifndef __LOCK_H__
#define __LOCK_H__

/*
 * Main driver lock, used to ensure driver code is single threaded.
 * There are some places where this lock must not be taken, such as
 * in the mmap related deriver entry points.
 */
extern PVRSRV_LINUX_MUTEX gPVRSRVLock;

#endif				/* __LOCK_H__ */
/*****************************************************************************
 End of file (lock.h)
*****************************************************************************/
