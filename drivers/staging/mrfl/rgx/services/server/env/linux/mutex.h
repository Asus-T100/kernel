									    /*************************************************************************//*!
									       @File
									       @Title          Linux mutex interface
									       @Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
									       @License        Strictly Confidential.
    *//**************************************************************************/

#ifndef __INCLUDED_LINUX_MUTEX_H_
#define __INCLUDED_LINUX_MUTEX_H_

#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15))
#include <linux/mutex.h>
#else
#include <asm/semaphore.h>
#endif

#include "pvrsrv_error.h"

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15))

typedef struct mutex PVRSRV_LINUX_MUTEX;

#else				/* (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)) */

typedef struct {
	struct semaphore sSemaphore;
	/* since Linux's struct semaphore is intended to be
	 * opaque we don't poke inside for the count and
	 * instead we track it outselves. (So we can implement
	 * LinuxIsLockedMutex)
	 */
	atomic_t Count;
} PVRSRV_LINUX_MUTEX;

#endif

extern IMG_VOID LinuxInitMutex(PVRSRV_LINUX_MUTEX * psPVRSRVMutex);

extern IMG_VOID LinuxLockMutex(PVRSRV_LINUX_MUTEX * psPVRSRVMutex);

extern PVRSRV_ERROR LinuxLockMutexInterruptible(PVRSRV_LINUX_MUTEX *
						psPVRSRVMutex);

extern IMG_INT32 LinuxTryLockMutex(PVRSRV_LINUX_MUTEX * psPVRSRVMutex);

extern IMG_VOID LinuxUnLockMutex(PVRSRV_LINUX_MUTEX * psPVRSRVMutex);

extern IMG_BOOL LinuxIsLockedMutex(PVRSRV_LINUX_MUTEX * psPVRSRVMutex);

#endif				/* __INCLUDED_LINUX_MUTEX_H_ */
