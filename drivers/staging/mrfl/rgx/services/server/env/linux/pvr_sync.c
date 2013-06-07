/*************************************************************************/ /*!
@File           pvr_sync.c
@Title          Kernel driver for Android's sync mechanism
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include "pvr_sync.h"

#include <linux/sync.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "img_types.h"
#include "allocmem.h"
#include "pvr_debug.h"
#include "pvrsrv.h"
#include "sync_server.h"
#include "rgxfwutils.h"

#include "pvr_fd_sync_user.h"

/* FIXME:
 * - fix the fence usage on all places in wsegl (DeleteBuffer and stuff)
 */

/*#define DEBUG_OUTPUT 1*/

#ifdef DEBUG_OUTPUT
#define DPF(fmt, ...) PVR_DPF((PVR_DBG_BUFFERED, fmt, __VA_ARGS__))
#else
#define DPF(fmt, ...) do {} while(0)
#endif

/* This is the IMG extension of a sync_timeline */
struct PVR_SYNC_TIMELINE
{
	struct sync_timeline obj;

	/* Global timeline list support */
    struct list_head sTlList;

	IMG_UINT32 ui32Id;
	atomic_t sValue;
};

struct PVR_SYNC_KERNEL_SYNC_PRIM
{
	/* Base services sync prim structure */
	SERVER_SYNC_PRIMITIVE *psSync;

	/* FWAddr used by the server sync */
	IMG_UINT32            ui32SyncPrimVAddr;

	/* Internal sync update value. Currently always '1'.
	 * This might change when/if we change to local syncs. */
	IMG_UINT32            ui32SyncValue;

	/* Cleanup sync prim structure. 
	 * If the base sync prim is used for "checking" only within a gl stream,
	 * there is no way of knowing when this has happened. So use a second sync
	 * prim which just gets updated and check the update count when freeing
	 * this struct. */
	SERVER_SYNC_PRIMITIVE *psCleanUpSync;

	/* FWAddr used by the cleanup server sync */
	IMG_UINT32            ui32CleanUpVAddr;

	/* Last used update value for the cleanup server sync */
	IMG_UINT32            ui32CleanUpValue;

	/* Sync points can go away when there are deferred hardware
	 * operations still outstanding. We must not free the SERVER_SYNC_PRIMITIVE
	 * until the hardware is finished, so we add it to a defer list
	 * which is processed periodically ("defer-free").
	 *
	 * Note that the defer-free list is global, not per-timeline.
	 */
	struct list_head      sHead;
};

struct PVR_SYNC_DATA
{
	/* Every sync point has a services sync object. This object is used
	 * by the hardware to enforce ordering -- it is attached as a source
	 * dependency to various commands.
	 */
	struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel;

	/* Every sync data will get some unique id */
	IMG_UINT32 ui32Id;

	/* The value when the this fence was taken according to the timeline this
	 * fence belongs to. Defines somehow the age of the fence point. */
	IMG_UINT32 ui32FenceValue;

	/* This refcount is incremented at create and dup time, and decremented
	 * at free time. It ensures the object doesn't start the defer-free
	 * process until it is no longer referenced.
	 */
	atomic_t sRefCount;
};

/* This is the IMG extension of a sync_pt */
struct PVR_SYNC_PT
{
	/* Private data */
	struct sync_pt pt;

	struct PVR_SYNC_DATA *psSyncData;

	IMG_BOOL bSignaled;

	IMG_BOOL bUpdated;
};

/* Any sync point from a foreign (non-PVR) timeline needs to have a "shadow"
 * sync prim. This is modelled as a software operation. The foreign driver
 * completes the operation by calling a callback we registered with it. */
struct PVR_SYNC_FENCE_WAITER
{
    /* Base sync driver waiter structure */
    struct sync_fence_waiter    waiter;

    /* "Shadow" sync prim backing the foreign driver's sync_pt */
	struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel;
};

/* Global data for the sync driver */
static struct
{
	/* Services connection */
	IMG_HANDLE hDevCookie;
	/* Unique requester id for taking sw sync ops. */
	IMG_UINT32 ui32SyncRequesterId;
	/* Complete notify handle */
	IMG_HANDLE hCmdCompHandle;
	/* Multi-purpose workqueue. Various functions in the Google sync driver
	 * may call down to us in atomic context. However, sometimes we may need
	 * to lock a mutex. To work around this conflict, use the workqueue to
	 * defer whatever the operation was. */
	struct workqueue_struct *psWorkQueue;
	/* Linux work struct for workqueue. */
	struct work_struct sWork;
	/* Unique id counter for the timelines. */
	atomic_t sTimelineId;
}gsPVRSync;

static LIST_HEAD(gTlList);
static DEFINE_MUTEX(gTlListLock);

/* The "defer-free" object list. Driver global. */
static LIST_HEAD(gSyncPrimFreeList);
static DEFINE_SPINLOCK(gSyncPrimFreeListLock);

/* Linux debugfs handle */
static struct dentry *gpsDebugfsDentry;

#ifdef DEBUG_OUTPUT
static char* _debugInfoTl(struct sync_timeline *tl)
{
	static char szInfo[256];
	struct PVR_SYNC_TIMELINE* psPVRTl = (struct PVR_SYNC_TIMELINE*)tl;

	szInfo[0] = '\0';

	snprintf(szInfo, sizeof(szInfo), "id=%u n='%s' tv=%d",
			 psPVRTl->ui32Id,
			 tl->name,
			 atomic_read(&psPVRTl->sValue));

	return szInfo;
}
static char* _debugInfoPt(struct sync_pt *pt)
{
	static char szInfo[256];
	static char szInfo1[256];
	struct PVR_SYNC_PT* psPVRPt = (struct PVR_SYNC_PT*)pt;

	szInfo[0] = '\0';
	szInfo1[0] = '\0';

	if (psPVRPt->psSyncData->psSyncKernel->psCleanUpSync)
	{
		snprintf(szInfo1, sizeof(szInfo1), " # cleanup: fw=%08x v=%u",
				 psPVRPt->psSyncData->psSyncKernel->ui32CleanUpVAddr,
				 psPVRPt->psSyncData->psSyncKernel->ui32CleanUpValue);
	}

	snprintf(szInfo, sizeof(szInfo), "sync: id=%u%s tv=%u # fw=%08x v=%u r=%d%s p: %s",
			 psPVRPt->psSyncData->ui32Id,
			 psPVRPt->bSignaled ? "* " : " ",
			 psPVRPt->psSyncData->ui32FenceValue,
			 psPVRPt->psSyncData->psSyncKernel->ui32SyncPrimVAddr,
			 psPVRPt->psSyncData->psSyncKernel->ui32SyncValue,
			 atomic_read(&psPVRPt->psSyncData->sRefCount),
			 szInfo1,
			 _debugInfoTl(pt->parent));

	return szInfo;
}
#endif /* DEBUG_OUTPUT */

static struct sync_pt *PVRSyncDup(struct sync_pt *sync_pt)
{
	struct PVR_SYNC_PT *psPVRPtOne = (struct PVR_SYNC_PT *)sync_pt;
	struct PVR_SYNC_PT *psPVRPtTwo = IMG_NULL;

	DPF("%s: # %s", __func__,
		_debugInfoPt(sync_pt));

	psPVRPtTwo = (struct PVR_SYNC_PT *)
		sync_pt_create(psPVRPtOne->pt.parent, sizeof(struct PVR_SYNC_PT));
	if (!psPVRPtTwo)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to dup sync pt",
				 __func__));
		goto err_out;
	}

	psPVRPtTwo->psSyncData = psPVRPtOne->psSyncData;
	psPVRPtTwo->bSignaled  = psPVRPtOne->bSignaled;
	psPVRPtTwo->bUpdated   = psPVRPtOne->bUpdated;

	atomic_inc(&psPVRPtTwo->psSyncData->sRefCount);

err_out:
	return (struct sync_pt*)psPVRPtTwo;
}

static int PVRSyncHasSignaled(struct sync_pt *sync_pt)
{
    struct PVR_SYNC_PT *psPVRPt = (struct PVR_SYNC_PT *)sync_pt;

	/* Instantly complete any syncs that have had the timeline destroyed
	 * beneath them. */
	if (sync_pt->parent->destroyed)
	{
		psPVRPt->bSignaled = IMG_TRUE;
	}

	if (ServerSyncFenceIsMeet(psPVRPt->psSyncData->psSyncKernel->psSync, psPVRPt->psSyncData->psSyncKernel->ui32SyncValue))
	{
		psPVRPt->bSignaled = IMG_TRUE;
	}

	DPF("%s: r: %d # %s", __func__,
		psPVRPt->bSignaled, _debugInfoPt(sync_pt));

	return psPVRPt->bSignaled == IMG_TRUE ? 1 : 0;
}

static int PVRSyncCompare(struct sync_pt *a, struct sync_pt *b)
{
	DPF("%s: a # %s", __func__,
				 _debugInfoPt(a));
	DPF("%s: b # %s", __func__,
		_debugInfoPt(b));

	return
		((struct PVR_SYNC_PT*)a)->psSyncData->ui32FenceValue == ((struct PVR_SYNC_PT*)b)->psSyncData->ui32FenceValue ? 0 :
		((struct PVR_SYNC_PT*)a)->psSyncData->ui32FenceValue >  ((struct PVR_SYNC_PT*)b)->psSyncData->ui32FenceValue ? 1 : -1;
}

static void PVRSyncReleaseTimeline(struct sync_timeline *psObj)
{
	struct PVR_SYNC_TIMELINE *psPVRTl = (struct PVR_SYNC_TIMELINE *)psObj;

	DPF("%s: # %s", __func__,
		_debugInfoTl(psObj));

    mutex_lock(&gTlListLock);
    list_del(&psPVRTl->sTlList);
    mutex_unlock(&gTlListLock);

	/* FIXME: do we need to wait for all sync points to finish first? */
}

static void PVRSyncPrintTimeline(struct seq_file *s,
								 struct sync_timeline *psObj)
{
	struct PVR_SYNC_TIMELINE *psPVRTl = (struct PVR_SYNC_TIMELINE *)psObj;

	seq_printf(s, "id=%u tv=%d",
			   psPVRTl->ui32Id, atomic_read(&psPVRTl->sValue));
}

static void PVRSyncPrint(struct seq_file *s, struct sync_pt *psPt)
{
	struct PVR_SYNC_PT *psPVRPt = (struct PVR_SYNC_PT *)psPt;

	seq_printf(s, "sync: id=%u tv=%u # fw=0x%08x v=%u",
			   psPVRPt->psSyncData->ui32Id,
			   psPVRPt->psSyncData->ui32FenceValue,
			   psPVRPt->psSyncData->psSyncKernel->ui32SyncPrimVAddr,
			   psPVRPt->psSyncData->psSyncKernel->ui32SyncValue);
	if (psPVRPt->psSyncData->psSyncKernel->psCleanUpSync)
	{
		seq_printf(s, " # cleanup: fw=0x%08x v=%u ",
				   psPVRPt->psSyncData->psSyncKernel->ui32CleanUpVAddr,
				   psPVRPt->psSyncData->psSyncKernel->ui32CleanUpValue);
	}
}

static struct PVR_SYNC_PT *
PVRSyncCreateSync(struct PVR_SYNC_TIMELINE *psPVRTl)
{
	struct PVR_SYNC_PT *psPVRPt = IMG_NULL;
	IMG_UINT32 ui32Dummy;
	IMG_UINT32 ui32FWAddr;
	IMG_UINT32 ui32CurrOp;
	IMG_UINT32 ui32NextOp;
	PVRSRV_ERROR eError;

	psPVRPt = (struct PVR_SYNC_PT *)
		sync_pt_create(&psPVRTl->obj, sizeof(struct PVR_SYNC_PT));
	if (!psPVRPt)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to create sync pt",
				 __func__));
		goto err_out;
	}

	psPVRPt->bSignaled = IMG_FALSE;
	psPVRPt->bUpdated = IMG_FALSE;

	psPVRPt->psSyncData = OSAllocMem(sizeof(struct PVR_SYNC_DATA));
	if (!psPVRPt->psSyncData)
	{
		goto err_free_pt;
	}

	psPVRPt->psSyncData->psSyncKernel = OSAllocMem(sizeof(struct PVR_SYNC_KERNEL_SYNC_PRIM));
	if (!psPVRPt->psSyncData->psSyncKernel)
	{
		goto err_free_data;
	}

	atomic_set(&psPVRPt->psSyncData->sRefCount, 1);
	psPVRPt->psSyncData->ui32FenceValue = atomic_inc_return(&psPVRTl->sValue);

	eError = PVRSRVServerSyncAllocKM(gsPVRSync.hDevCookie,
									 &psPVRPt->psSyncData->psSyncKernel->psSync,
									 &psPVRPt->psSyncData->psSyncKernel->ui32SyncPrimVAddr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate prim server sync (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_free_data;
	}

	eError = PVRSRVServerSyncGetStatusKM(1, &psPVRPt->psSyncData->psSyncKernel->psSync,
										 &psPVRPt->psSyncData->ui32Id,
										 &ui32FWAddr,
										 &ui32CurrOp, &ui32NextOp);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to get status of prim server sync (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_free_data;
	}

	eError = PVRSRVServerSyncQueueHWOpKM(psPVRPt->psSyncData->psSyncKernel->psSync,
										 IMG_TRUE,
										 &ui32Dummy,
										 &psPVRPt->psSyncData->psSyncKernel->ui32SyncValue);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to queue prim server sync hw operation (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_free_data;
	}

	psPVRPt->psSyncData->psSyncKernel->psCleanUpSync = IMG_NULL;
	psPVRPt->psSyncData->psSyncKernel->ui32CleanUpVAddr = 0;
	psPVRPt->psSyncData->psSyncKernel->ui32CleanUpValue = 0;

err_out:
	return psPVRPt;

err_free_data:
	OSFreeMem(psPVRPt->psSyncData);

err_free_pt:
	sync_pt_free((struct sync_pt *)psPVRPt);
	psPVRPt = IMG_NULL;
	goto err_out;
}

static void 
PVRSyncAddToDeferFreeList(struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel)
{
	unsigned long flags;
	spin_lock_irqsave(&gSyncPrimFreeListLock, flags);
	list_add_tail(&psSyncKernel->sHead, &gSyncPrimFreeList);
	spin_unlock_irqrestore(&gSyncPrimFreeListLock, flags);
}

/* Releases a sync prim - freeing it if there are no outstanding
 * operations, else adding it to a deferred list to be freed later.
 * Returns IMG_TRUE if the free was deferred, IMG_FALSE otherwise.
 */
static IMG_BOOL
PVRSyncReleaseSyncPrim(struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel)
{
	PVRSRV_ERROR eError;

	/* Freeing the sync needs us to be in non atomic context,
	 * but this function may be called from the sync driver in
	 * interrupt context (for example a sw_sync user incs a timeline).
	 * In such a case we must defer processing to the WQ.
	 */
	if(in_atomic() || in_interrupt())
	{
		PVRSyncAddToDeferFreeList(psSyncKernel);
		return IMG_TRUE;
	}

	OSAcquireBridgeLock();

	if (   !ServerSyncFenceIsMeet(psSyncKernel->psSync, psSyncKernel->ui32SyncValue)
		|| (psSyncKernel->psCleanUpSync && !ServerSyncFenceIsMeet(psSyncKernel->psCleanUpSync, psSyncKernel->ui32CleanUpValue)))
	{
		OSReleaseBridgeLock();
		PVRSyncAddToDeferFreeList(psSyncKernel);
		return IMG_TRUE;
	}

	eError = PVRSRVServerSyncFreeKM(psSyncKernel->psSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to free prim server sync (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		/* Fall-thru */
	}
	if (psSyncKernel->psCleanUpSync)
	{
		eError = PVRSRVServerSyncFreeKM(psSyncKernel->psCleanUpSync);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to free prim server sync (%s)",
					 __func__, PVRSRVGetErrorStringKM(eError)));
			/* Fall-thru */
		}
	}
	OSFreeMem(psSyncKernel);
	OSReleaseBridgeLock();
	return IMG_FALSE;
}

static void PVRSyncFreeSync(struct sync_pt *psPt)
{
	struct PVR_SYNC_PT *psPVRPt = (struct PVR_SYNC_PT *)psPt;

	DPF("%s: # %s", __func__,
		_debugInfoPt(psPt));

    /* Only free on the last reference */
    if (atomic_dec_return(&psPVRPt->psSyncData->sRefCount) != 0)
        return;

	if(PVRSyncReleaseSyncPrim(psPVRPt->psSyncData->psSyncKernel))
		queue_work(gsPVRSync.psWorkQueue, &gsPVRSync.sWork);
	OSFreeMem(psPVRPt->psSyncData);
}

static struct sync_timeline_ops gsPVR_SYNC_TIMELINE_ops =
{
	.driver_name        = PVRSYNC_MODNAME,
	.dup                = PVRSyncDup,
	.has_signaled       = PVRSyncHasSignaled,
	.compare            = PVRSyncCompare,
	.free_pt            = PVRSyncFreeSync,
	.release_obj        = PVRSyncReleaseTimeline,
	.print_obj          = PVRSyncPrintTimeline,
	.print_pt           = PVRSyncPrint,
};

/* foreign sync handling */

static void
PVRSyncForeignSyncPtSignaled(struct sync_fence *fence,
							 struct sync_fence_waiter *waiter)
{
    struct PVR_SYNC_FENCE_WAITER *psWaiter =
        (struct PVR_SYNC_FENCE_WAITER *)waiter;

	/* Complete the SW operation and free the sync if we can. If we can't,
	 * it will be checked by a later workqueue kick. */
	ServerSyncCompleteOp(psWaiter->psSyncKernel->psSync, psWaiter->psSyncKernel->ui32SyncValue);

	/* Can ignore retval because we queue_work anyway */
	PVRSyncReleaseSyncPrim(psWaiter->psSyncKernel);

	/* This complete may unblock the GPU. */
	queue_work(gsPVRSync.psWorkQueue, &gsPVRSync.sWork);

	OSFreeMem(psWaiter);
	sync_fence_put(fence);
}

static struct PVR_SYNC_KERNEL_SYNC_PRIM *
PVRSyncCreateWaiterForForeignSync(int iFenceFd)
{
	struct PVR_SYNC_FENCE_WAITER *psWaiter;
	struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel = IMG_NULL;
    struct sync_fence *psFence;
	IMG_UINT32 ui32Dummy;
    PVRSRV_ERROR eError;
    int err;

	psFence = sync_fence_fdget(iFenceFd);
    if(!psFence)
    {
        PVR_DPF((PVR_DBG_ERROR, "%s: Failed to take reference on fence",
                                __func__));
        goto err_out;
    }

	psSyncKernel = OSAllocMem(sizeof(struct PVR_SYNC_KERNEL_SYNC_PRIM));
    if(!psSyncKernel)
    {
        PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate sync kernel", __func__));
        goto err_put_fence;
    }

	eError = PVRSRVServerSyncAllocKM(gsPVRSync.hDevCookie,
									 &psSyncKernel->psSync,
									 &psSyncKernel->ui32SyncPrimVAddr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate prim server sync (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_free_kernel;
	}

	eError = PVRSRVServerSyncQueueSWOpKM(psSyncKernel->psSync,
										 &ui32Dummy,
										 &psSyncKernel->ui32SyncValue,
										 gsPVRSync.ui32SyncRequesterId,
										 IMG_TRUE,
										 IMG_NULL);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to queue prim server sync sw operation (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_free_sync;
	}

	eError = PVRSRVServerSyncAllocKM(gsPVRSync.hDevCookie,
									 &psSyncKernel->psCleanUpSync,
									 &psSyncKernel->ui32CleanUpVAddr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate cleanup prim server sync (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_complete_sync;
	}

	eError = PVRSRVServerSyncQueueHWOpKM(psSyncKernel->psCleanUpSync,
										 IMG_TRUE,
										 &ui32Dummy,
										 &psSyncKernel->ui32CleanUpValue);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to queue cleanup prim server sync hw operation (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_free_cleanup_sync;
	}

	/* The custom waiter structure is freed in the waiter callback */
	psWaiter = OSAllocMem(sizeof(struct PVR_SYNC_FENCE_WAITER));
    if(!psWaiter)
    {
        PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate waiter", __func__));
        goto err_complete_cleanup_sync;
    }
	psWaiter->psSyncKernel = psSyncKernel;

	sync_fence_waiter_init(&psWaiter->waiter, PVRSyncForeignSyncPtSignaled);

    err = sync_fence_wait_async(psFence, &psWaiter->waiter);
    if(err)
    {
        if(err < 0)
        {
            PVR_DPF((PVR_DBG_ERROR, "%s: Fence was in error state", __func__));
            /* Fall-thru */
        }

        /* -1 means the fence was broken, 1 means the fence already
         * signalled. In either case, roll back what we've done and
         * skip using this sync_pt for synchronization.
         */
        goto err_free_waiter;
    }

err_out:
	return psSyncKernel;

err_free_waiter:
	OSFreeMem(psWaiter);

err_complete_cleanup_sync:
	PVRSRVServerSyncPrimSetKM(psSyncKernel->psCleanUpSync, psSyncKernel->ui32CleanUpValue);

err_free_cleanup_sync:
	eError = PVRSRVServerSyncFreeKM(psSyncKernel->psCleanUpSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to free cleanup prim server sync (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		/* Fall-thru */
	}

err_complete_sync:
	ServerSyncCompleteOp(psSyncKernel->psSync, psSyncKernel->ui32SyncValue);

err_free_sync:
	eError = PVRSRVServerSyncFreeKM(psSyncKernel->psSync);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to free prim server sync (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		/* Fall-thru */
	}

err_free_kernel:
	OSFreeMem(psSyncKernel);
	psSyncKernel = IMG_NULL;

err_put_fence:
	sync_fence_put(psFence);
	goto err_out;
}

static PVRSRV_ERROR
PVRSyncDebugFenceKM(IMG_INT32 i32FDFence,
					IMG_CHAR *pszName,
					IMG_INT32 *pi32Status,
					IMG_UINT32 ui32MaxNumSyncs,
					IMG_UINT32 *pui32NumSyncs,
					PVR_SYNC_DEBUG_SYNC_DATA *aPts)
{
	struct list_head *psEntry;
	struct sync_fence *psFence = sync_fence_fdget(i32FDFence);
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (!psFence)
		return PVRSRV_ERROR_HANDLE_NOT_FOUND;

	if (!pui32NumSyncs || !pi32Status)
		return PVRSRV_ERROR_INVALID_PARAMS;

	*pui32NumSyncs = 0;

	strncpy(pszName, psFence->name, sizeof(psFence->name));
	*pi32Status = psFence->status;

	list_for_each(psEntry, &psFence->pt_list_head)
	{
		struct sync_pt *psPt =
			container_of(psEntry, struct sync_pt, pt_list);
		if (*pui32NumSyncs == ui32MaxNumSyncs)
		{
			PVR_DPF((PVR_DBG_WARNING, "%s: To less space on fence query for all "
					 "the sync points in this fence", __func__));
			goto err_put;
		}

		/* Clear the entry */
		memset(&aPts[*pui32NumSyncs], 0, sizeof(aPts[*pui32NumSyncs]));

		/* Save this within the sync point. */
		strncpy(aPts[*pui32NumSyncs].szParentName, psPt->parent->name,
				sizeof(aPts[*pui32NumSyncs].szParentName));

		/* Only fill this for our sync points. Foreign syncs will get empty
		 * fields. */
		if(psPt->parent->ops == &gsPVR_SYNC_TIMELINE_ops)
		{
			struct PVR_SYNC_PT *psPVRPt = (struct PVR_SYNC_PT *)psPt;
			IMG_UINT32 ui32UID;
			IMG_UINT32 ui32FWAddr;
			IMG_UINT32 ui32CurrOp;
			IMG_UINT32 ui32NextOp;

			eError = PVRSRVServerSyncGetStatusKM(1, &psPVRPt->psSyncData->psSyncKernel->psSync,
												 &ui32UID, &ui32FWAddr,
												 &ui32CurrOp, &ui32NextOp);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: Failed to get status of prim server "
						 "sync (%s)", __func__, PVRSRVGetErrorStringKM(eError)));
				goto err_put;
			}

			aPts[*pui32NumSyncs].ui32Id                = ui32UID;
			aPts[*pui32NumSyncs].ui32CurrOp            = ui32CurrOp;
			aPts[*pui32NumSyncs].ui32NextOp            = ui32NextOp;
			aPts[*pui32NumSyncs].sData.ui32FWAddr      = psPVRPt->psSyncData->psSyncKernel->ui32SyncPrimVAddr;
			aPts[*pui32NumSyncs].sData.ui32FenceValue  = psPVRPt->psSyncData->ui32FenceValue;
			aPts[*pui32NumSyncs].sData.ui32UpdateValue = psPVRPt->psSyncData->ui32FenceValue;
		}

		++*pui32NumSyncs;
	}

err_put:
	sync_fence_put(psFence);
	return eError;
}

/* ioctl and fops handling */

static int PVRSyncOpen(struct inode *inode, struct file *file)
{
	IMG_CHAR task_comm[TASK_COMM_LEN + 1];
	struct PVR_SYNC_TIMELINE *psPVRTl;
	int err = -ENOMEM;

	get_task_comm(task_comm, current);

    psPVRTl = (struct PVR_SYNC_TIMELINE *)
		sync_timeline_create(&gsPVR_SYNC_TIMELINE_ops,
							 sizeof(struct PVR_SYNC_TIMELINE), task_comm);
    if (!psPVRTl)
    {
        PVR_DPF((PVR_DBG_ERROR, "%s: sync_timeline_create failed", __func__));
        goto err_out;
    }

	psPVRTl->ui32Id = atomic_inc_return(&gsPVRSync.sTimelineId);
	atomic_set(&psPVRTl->sValue, 0);

	DPF("%s: # %s", __func__,
		_debugInfoTl((struct sync_timeline*)psPVRTl));

	mutex_lock(&gTlListLock);
    list_add_tail(&psPVRTl->sTlList, &gTlList);
    mutex_unlock(&gTlListLock);

	file->private_data = psPVRTl;

	err = 0;

err_out:
    return err;
}

static int PVRSyncRelease(struct inode *inode, struct file *file)
{
	struct PVR_SYNC_TIMELINE *psPVRTl = file->private_data;

	DPF("%s: # %s", __func__,
		_debugInfoTl((struct sync_timeline*)psPVRTl));

	sync_timeline_destroy(&psPVRTl->obj);

	return 0;
}

static long
PVRSyncIOCTLCreateFence(struct PVR_SYNC_TIMELINE *psPVRTl, void __user *pvData)
{
	struct PVR_SYNC_CREATE_FENCE_IOCTL_DATA sData;
	int err = -EFAULT, iFd = get_unused_fd();
	struct sync_fence *psFence;
	struct sync_pt *psPt;

	if (iFd < 0)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to find unused fd (%d)",
								__func__, iFd));
		goto err_put_fd;
	}

	if (!access_ok(VERIFY_READ, pvData, sizeof(sData)))
	{
		goto err_put_fd;
	}

	if (copy_from_user(&sData, pvData, sizeof(sData)))
	{
		goto err_put_fd;
	}

	psPt = (struct sync_pt *)
		PVRSyncCreateSync(psPVRTl);
	if (!psPt)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to create a sync point (%d)",
								__func__, iFd));
		err = -ENOMEM;
		goto err_put_fd;
	}

	sData.szName[sizeof(sData.szName) - 1] = '\0';

	DPF("%s: %d('%s') # %s", __func__,
		iFd, sData.szName,
		_debugInfoTl((struct sync_timeline*)psPVRTl));

	psFence = sync_fence_create(sData.szName, psPt);
	if (!psFence)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to create a fence (%d)",
								__func__, iFd));
		err = -ENOMEM;
		goto err_sync_free;
	}

	sData.iFenceFd = iFd;

	if (!access_ok(VERIFY_WRITE, pvData, sizeof(sData)))
	{
		goto err_put_fence;
	}

	if (copy_to_user(pvData, &sData, sizeof(sData)))
	{
		goto err_put_fence;
	}

	sync_fence_install(psFence, iFd);

	err = 0;

err_out:
	return err;

err_put_fence:
	sync_fence_put(psFence);
err_sync_free:
	sync_pt_free(psPt);
err_put_fd:
	put_unused_fd(iFd);
	goto err_out;
}

static long
PVRSyncIOCTLDebugFence(struct PVR_SYNC_TIMELINE *psPVRTl, void __user *pvData)
{
	struct PVR_SYNC_DEBUG_FENCE_IOCTL_DATA sData;
	PVRSRV_ERROR eError;
	int err = -EFAULT;

	if (!access_ok(VERIFY_READ, pvData, sizeof(sData)))
	{
		goto err_out;
	}

	if (copy_from_user(&sData, pvData, sizeof(sData)))
	{
		goto err_out;
	}

	eError = PVRSyncDebugFenceKM(sData.iFenceFd,
								 sData.szName,
								 &sData.i32Status,
								 PVR_SYNC_MAX_QUERY_FENCE_POINTS,
								 &sData.ui32NumSyncs,
								 sData.aPts);
	if (eError != PVRSRV_OK)
	{
		goto err_out;
	}

	if (!access_ok(VERIFY_WRITE, pvData, sizeof(sData)))
	{
		goto err_out;
	}

	if (copy_to_user(pvData, &sData, sizeof(sData)))
	{
		goto err_out;
	}

	err = 0;

err_out:
	return err;

	return 0;
}

static long
PVRSyncIOCTL(struct file *file, unsigned int cmd, unsigned long __user arg)
{
	struct PVR_SYNC_TIMELINE *psPVRTl = file->private_data;
	void __user *pvData = (void __user *)arg;
	long err = -ENOTTY;

	OSAcquireBridgeLock();

	switch (cmd)
	{
		case PVR_SYNC_IOC_CREATE_FENCE:
            err = PVRSyncIOCTLCreateFence(psPVRTl, pvData);
			break;
		case PVR_SYNC_IOC_DEBUG_FENCE:
            err = PVRSyncIOCTLDebugFence(psPVRTl, pvData);
			break;
		default:
			break;
	}

	OSReleaseBridgeLock();

	return err;
}

static void
PVRSyncWorkQueueFunction(struct work_struct *data)
{
	struct list_head sFreeList, *psEntry, *n;
	unsigned long flags;
	PVRSRV_ERROR eError;

	/* A completed SW operation may un-block the GPU */
	PVRSRVCheckStatus(gsPVRSync.hDevCookie);

	/* We can't call PVRSRVServerSyncFreeKM directly in this loop because
	 * that will take the mmap mutex. We can't take mutexes while we have
	 * this list locked with a spinlock. So move all the items we want to
	 * free to another, local list (no locking required) and process it
	 * in a second loop.
	 */

	OSAcquireBridgeLock();

	INIT_LIST_HEAD(&sFreeList);
	spin_lock_irqsave(&gSyncPrimFreeListLock, flags);
	list_for_each_safe(psEntry, n, &gSyncPrimFreeList)
	{
		struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel =
            container_of(psEntry, struct PVR_SYNC_KERNEL_SYNC_PRIM, sHead);

		/* Check if this sync is not used anymore. */
		if (   !ServerSyncFenceIsMeet(psSyncKernel->psSync, psSyncKernel->ui32SyncValue)
			|| (psSyncKernel->psCleanUpSync && !ServerSyncFenceIsMeet(psSyncKernel->psCleanUpSync, psSyncKernel->ui32CleanUpValue)))
				continue;

		/* Remove the entry from the free list. */
		list_move_tail(psEntry, &sFreeList);
    }
    spin_unlock_irqrestore(&gSyncPrimFreeListLock, flags);

	list_for_each_safe(psEntry, n, &sFreeList)
	{
		struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel =
			container_of(psEntry, struct PVR_SYNC_KERNEL_SYNC_PRIM, sHead);

		list_del(psEntry);

		eError = PVRSRVServerSyncFreeKM(psSyncKernel->psSync);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to free prim server sync (%s)",
					 __func__, PVRSRVGetErrorStringKM(eError)));
			/* Fall-thru */
		}
		if (psSyncKernel->psCleanUpSync)
		{
			eError = PVRSRVServerSyncFreeKM(psSyncKernel->psCleanUpSync);
			if (eError != PVRSRV_OK)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: Failed to free cleanup prim server sync (%s)",
						 __func__, PVRSRVGetErrorStringKM(eError)));
				/* Fall-thru */
			}
		}
		OSFreeMem(psSyncKernel);
	}

	OSReleaseBridgeLock();
}

static const struct file_operations gsPVRSyncFOps =
{
	.owner          = THIS_MODULE,
	.open           = PVRSyncOpen,
	.release        = PVRSyncRelease,
	.unlocked_ioctl = PVRSyncIOCTL,
};

static struct miscdevice sPVRSyncDev =
{
	.minor          = MISC_DYNAMIC_MINOR,
	.name           = PVRSYNC_MODNAME,
	.fops           = &gsPVRSyncFOps,
};

static int PVRSyncDebugfsShow(struct seq_file *s, void *unused)
{
/*	struct list_head *pos;*/

	seq_printf(s, "Timelines:\n----------\n");

#ifdef DEBUG_OUTPUT
	PVRSRVDebugPrintfDumpCCB();
#endif

	//mutex_lock(&gTimelineListLock);
	//list_for_each(pos, &gTimelineList)
	//{
	//	unsigned long flags;
	//	struct list_head *activePoint;
	//	struct sync_timeline *psTimeline = (struct sync_timeline*)
	//		container_of(pos, struct PVR_SYNC_TIMELINE, sTimelineList);

	//	seq_printf(s, "%s :", psTimeline->name);
	//	PVRSyncPrintTimeline(s, psTimeline);
	//	seq_printf(s, "\n ");

	//	spin_lock_irqsave(&psTimeline->active_list_lock, flags);
	//	list_for_each(activePoint, &psTimeline->active_list_head)
	//	{
	//		struct sync_pt *psPt =
	//			container_of(activePoint, struct sync_pt, active_list);
	//		PVRSyncPrint(s, psPt);
	//		seq_printf(s, "\n ");
	//	}
	//	spin_unlock_irqrestore(&psTimeline->active_list_lock, flags);
	//	seq_printf(s, "\n");
	//}
	//mutex_unlock(&gTimelineListLock);
	return 0;
}

static int PVRSyncDebugfsOpen(struct inode *inode, struct file *file)
{
	return single_open(file, PVRSyncDebugfsShow, inode->i_private);
}

static const struct file_operations gsDebugfsFOps =
{
	.open			= PVRSyncDebugfsOpen,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release
};

static
IMG_VOID PVRSyncUpdateAllTimelines(PVRSRV_CMDCOMP_HANDLE hCmdCompHandle)
{
	IMG_BOOL bSignal;
	struct PVR_SYNC_TIMELINE *psPVRTl;
	struct list_head *psTlEntry, *psPtEntry;
	unsigned long flags;

	PVR_UNREFERENCED_PARAMETER(hCmdCompHandle);

	mutex_lock(&gTlListLock);
	list_for_each(psTlEntry, &gTlList)
	{
		bSignal = IMG_FALSE;
		psPVRTl =
			container_of(psTlEntry, struct PVR_SYNC_TIMELINE, sTlList);

		spin_lock_irqsave(&psPVRTl->obj.active_list_lock, flags);
		list_for_each(psPtEntry, &psPVRTl->obj.active_list_head)
		{
			struct sync_pt *psPt =
				container_of(psPtEntry, struct sync_pt, active_list);

			if(psPt->parent->ops != &gsPVR_SYNC_TIMELINE_ops)
				continue;

			DPF("%s: check # %s", __func__,
				_debugInfoPt(psPt));

			/* Check for any points which weren't signaled before, but
			 * are now. If so, signal the timeline and stop processing
			 * this timeline. */
			if (   ((struct PVR_SYNC_PT *)psPt)->bSignaled == IMG_FALSE
				&& PVRSyncHasSignaled(psPt))
			{
				DPF("%s: signal # %s", __func__,
					_debugInfoPt(psPt));
				/* Signal outside the spin lock. */
				bSignal = IMG_TRUE;
				break;
			}
		}
		spin_unlock_irqrestore(&psPVRTl->obj.active_list_lock, flags);

		if (bSignal)
			sync_timeline_signal((struct sync_timeline *)psPVRTl);
	}
	mutex_unlock(&gTlListLock);
}

IMG_INTERNAL
PVRSRV_ERROR PVRFDSyncDeviceInitKM(void)
{
	PVRSRV_ERROR eError;
	int err;

	DPF("%s", __func__);
	
	OSAcquireBridgeLock();

	/* TODO: multiple devices support? */
	eError = PVRSRVAcquireDeviceDataKM(0, PVRSRV_DEVICE_TYPE_RGX, &gsPVRSync.hDevCookie);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to initialise services (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_out;
	}

	eError = PVRSRVRegisterCmdCompleteNotify(&gsPVRSync.hCmdCompHandle,
											 &PVRSyncUpdateAllTimelines,
											 &gsPVRSync.hDevCookie);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to register MISR notification (%s)",
				 __func__, PVRSRVGetErrorStringKM(eError)));
		goto err_out;
	}

	gsPVRSync.psWorkQueue = create_freezable_workqueue("pvr_sync_workqueue");
	if (!gsPVRSync.psWorkQueue)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to create pvr_sync workqueue",
				 __func__));
		goto err_out;
	}

	INIT_WORK(&gsPVRSync.sWork, PVRSyncWorkQueueFunction);

	err = misc_register(&sPVRSyncDev);
	if (err)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to register pvr_sync device "
				 "(%d)", __func__, err));
		eError = PVRSRV_ERROR_RESOURCE_UNAVAILABLE;
		goto err_out;
	}

	eError = PVRSRVServerSyncRequesterRegisterKM(&gsPVRSync.ui32SyncRequesterId);
    if (eError != PVRSRV_OK)
    {
		PVR_DPF((PVR_DBG_ERROR, "%s: Failed to register sync requester "
				 "(%d)", __func__, err));
        goto err_out;
    }

	atomic_set(&gsPVRSync.sTimelineId, 0);

	/* Optional */
	gpsDebugfsDentry =
		debugfs_create_file("pvr_sync", S_IRUGO, NULL, NULL, &gsDebugfsFOps);
	if(!gpsDebugfsDentry)
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: Failed to create pvr_sync debugfs "
				 "entry", __func__));
		/* Fall-thru */
	}
	
err_out:
	OSReleaseBridgeLock();

	return eError;
}

IMG_INTERNAL
void PVRFDSyncDeviceDeInitKM(void)
{
	DPF("%s", __func__);

	OSAcquireBridgeLock();

	PVRSRVServerSyncRequesterUnregisterKM(gsPVRSync.ui32SyncRequesterId);

	PVRSRVUnregisterCmdCompleteNotify(gsPVRSync.hCmdCompHandle);

	destroy_workqueue(gsPVRSync.psWorkQueue);

	if(gpsDebugfsDentry)
		debugfs_remove(gpsDebugfsDentry);

	misc_deregister(&sPVRSyncDev);

	OSReleaseBridgeLock();
}

static
PVRSRV_ERROR PVRFDSyncQueryFenceKM(IMG_INT32 i32FDFence,
								   IMG_BOOL bUpdate,
								   IMG_UINT32 ui32MaxNumSyncs,
								   IMG_UINT32 *pui32NumSyncs,
								   PVR_SYNC_POINT_DATA *aPts)
{
	struct list_head *psEntry;
    struct sync_fence *psFence = sync_fence_fdget(i32FDFence);
	struct PVR_SYNC_TIMELINE *psPVRTl;
	struct PVR_SYNC_PT *psPVRPt = IMG_NULL;
	IMG_UINT32 ui32Dummy;
	PVRSRV_ERROR eError = PVRSRV_OK;
	IMG_BOOL bHaveForeignSync = IMG_FALSE;

	DPF("%s: fence %d ('%s')",
		__func__,
		i32FDFence, psFence->name);

	if (!psFence)
		return PVRSRV_ERROR_HANDLE_NOT_FOUND;

	*pui32NumSyncs = 0;

	/* TODO: Optimization: If the fence itself or a single sync point is
	 * already signaled, than don't return the entry. It makes no real sense to
	 * send CHECK commands down to the firmware, when we already know they are
	 * fulfilled. */
	list_for_each(psEntry, &psFence->pt_list_head)
	{
		struct sync_pt *psPt =
			container_of(psEntry, struct sync_pt, pt_list);

		if (*pui32NumSyncs == ui32MaxNumSyncs)
		{
			PVR_DPF((PVR_DBG_WARNING, "%s: To less space on fence query for all "
					 "the sync points in this fence", __func__));
			goto err_put;
		}

		if(psPt->parent->ops != &gsPVR_SYNC_TIMELINE_ops)
		{
			/* If there are foreign sync points in this fence we will add a
			 * shadow sync prim for them. */
			bHaveForeignSync = IMG_TRUE;
		}
		else
		{
			psPVRTl = (struct PVR_SYNC_TIMELINE *)psPt->parent;
			psPVRPt = (struct PVR_SYNC_PT *)psPt;

			DPF("%s: %d # %s", __func__,
				*pui32NumSyncs,
				_debugInfoPt(psPt));

			if (   bUpdate
				&& psPVRPt->bUpdated)
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: sync point used for update more than once!",
						 __func__));
			}

			/* Save this within the sync point. */
			aPts[*pui32NumSyncs].ui32FWAddr      = psPVRPt->psSyncData->psSyncKernel->ui32SyncPrimVAddr;
			aPts[*pui32NumSyncs].ui32Flags       = (bUpdate ? PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE :
															  PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK);
												   //| PVRSRV_CLIENT_SYNC_PRIM_OP_SYNC_FD;
			aPts[*pui32NumSyncs].ui32FenceValue  = psPVRPt->psSyncData->psSyncKernel->ui32SyncValue;
			aPts[*pui32NumSyncs].ui32UpdateValue = psPVRPt->psSyncData->psSyncKernel->ui32SyncValue;
			++*pui32NumSyncs;

			/* We will use the above sync for "check" only. In this case also
			 * insert a "cleanup" update command into the opengl stream. This
			 * can later be used for checking if the sync prim could be freed. */
			if (!bUpdate)
			{
				if (*pui32NumSyncs == ui32MaxNumSyncs)
				{
					PVR_DPF((PVR_DBG_WARNING, "%s: To less space on fence query for all "
							 "the sync points in this fence", __func__));
					goto err_put;
				}

				/* We returning for "check" only. Create the clean up sync on
				 * demand and queue an update operation. */
				if (!psPVRPt->psSyncData->psSyncKernel->psCleanUpSync)
				{
					eError = PVRSRVServerSyncAllocKM(gsPVRSync.hDevCookie,
													 &psPVRPt->psSyncData->psSyncKernel->psCleanUpSync,
													 &psPVRPt->psSyncData->psSyncKernel->ui32CleanUpVAddr);
					if (eError != PVRSRV_OK)
					{
						PVR_DPF((PVR_DBG_ERROR, "%s: Failed to allocate cleanup prim server sync (%s)",
								 __func__, PVRSRVGetErrorStringKM(eError)));
						goto err_put;
					}
				}
				
				eError = PVRSRVServerSyncQueueHWOpKM(psPVRPt->psSyncData->psSyncKernel->psCleanUpSync,
													 IMG_TRUE,
													 &ui32Dummy,
													 &psPVRPt->psSyncData->psSyncKernel->ui32CleanUpValue);
				if (eError != PVRSRV_OK)
				{
					PVR_DPF((PVR_DBG_ERROR, "%s: Failed to queue cleanup prim server sync hw operation (%s)",
							 __func__, PVRSRVGetErrorStringKM(eError)));
					goto err_put;
				}
				/* Save this within the sync point. */
				aPts[*pui32NumSyncs].ui32FWAddr      = psPVRPt->psSyncData->psSyncKernel->ui32CleanUpVAddr;
				aPts[*pui32NumSyncs].ui32Flags       = PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE;// | PVRSRV_CLIENT_SYNC_PRIM_OP_SYNC_FD;
				aPts[*pui32NumSyncs].ui32FenceValue  = psPVRPt->psSyncData->psSyncKernel->ui32CleanUpValue;
				aPts[*pui32NumSyncs].ui32UpdateValue = psPVRPt->psSyncData->psSyncKernel->ui32CleanUpValue;
				++*pui32NumSyncs;

			}

			if (bUpdate)
				psPVRPt->bUpdated = IMG_TRUE;
		}
	}

	/* Add one shadow sync prim for "all" foreign sync points. We are only
	 * interested in a signaled fence not individual signaled sync points. */
	if (bHaveForeignSync)
	{
		struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel;

		/* Create a shadow sync prim for the foreign sync point. */
		psSyncKernel = PVRSyncCreateWaiterForForeignSync(i32FDFence);

		/* This could be zero when the sync has signaled already. */
		if (psSyncKernel)
		{
			if (*pui32NumSyncs == ui32MaxNumSyncs - 1)
			{
				PVR_DPF((PVR_DBG_WARNING, "%s: To less space on fence query for all "
						 "the sync points in this fence", __func__));
				goto err_put;
			}

			aPts[*pui32NumSyncs].ui32FWAddr      = psSyncKernel->ui32SyncPrimVAddr;
			aPts[*pui32NumSyncs].ui32Flags       = PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK; // | PVRSRV_CLIENT_SYNC_PRIM_OP_SYNC_FD;
			aPts[*pui32NumSyncs].ui32FenceValue  = psSyncKernel->ui32SyncValue;
			aPts[*pui32NumSyncs].ui32UpdateValue = psSyncKernel->ui32SyncValue;
			++*pui32NumSyncs;

			aPts[*pui32NumSyncs].ui32FWAddr      = psSyncKernel->ui32CleanUpVAddr;
			aPts[*pui32NumSyncs].ui32Flags       = PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE; // | PVRSRV_CLIENT_SYNC_PRIM_OP_SYNC_FD;
			aPts[*pui32NumSyncs].ui32FenceValue  = psSyncKernel->ui32CleanUpValue;
			aPts[*pui32NumSyncs].ui32UpdateValue = psSyncKernel->ui32CleanUpValue;
			++*pui32NumSyncs;
		}
	}

err_put:
	sync_fence_put(psFence);
	return eError;
}

IMG_INTERNAL
PVRSRV_ERROR PVRFDSyncQueryFencesKM(IMG_UINT32 ui32NumFDFences,
									IMG_INT32 *ai32FDFences,
									IMG_BOOL bUpdate,
									IMG_UINT32 *pui32NumFenceSyncs,
									IMG_UINT32 **ppui32FenceFWAddrs,
									IMG_UINT32 **ppui32FenceValues,
									IMG_UINT32 *pui32NumUpdateSyncs,
									IMG_UINT32 **ppui32UpdateFWAddrs,
									IMG_UINT32 **ppui32UpdateValues)
{
	PVR_SYNC_POINT_DATA aPts[PVR_SYNC_MAX_QUERY_FENCE_POINTS];
	IMG_UINT32 ui32NumSyncs;
	IMG_UINT32 aui32FenceFWAddrsTmp[PVR_SYNC_MAX_QUERY_FENCE_POINTS * ui32NumFDFences];
	IMG_UINT32 aui32FenceValuesTmp[PVR_SYNC_MAX_QUERY_FENCE_POINTS * ui32NumFDFences];
	IMG_UINT32 aui32UpdateFWAddrsTmp[PVR_SYNC_MAX_QUERY_FENCE_POINTS * ui32NumFDFences];
	IMG_UINT32 aui32UpdateValuesTmp[PVR_SYNC_MAX_QUERY_FENCE_POINTS * ui32NumFDFences];
	IMG_UINT32 i, a, f = 0, u = 0;
	PVRSRV_ERROR eError = PVRSRV_OK;

	*ppui32FenceFWAddrs = IMG_NULL;
	*ppui32FenceValues = IMG_NULL;
	*ppui32UpdateFWAddrs = IMG_NULL;
	*ppui32UpdateValues = IMG_NULL;
	*pui32NumFenceSyncs = 0;
	*pui32NumUpdateSyncs = 0;

	for (i = 0; i < ui32NumFDFences; i++)
	{
		eError = PVRFDSyncQueryFenceKM(ai32FDFences[i],
									   bUpdate,
									   PVR_SYNC_MAX_QUERY_FENCE_POINTS,
									   &ui32NumSyncs,
									   aPts);
		if (eError != PVRSRV_OK)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: query fence %d failed (%s)", 
					 __func__, ai32FDFences[i], PVRSRVGetErrorStringKM(eError)));
			goto err_out;
		}			
		for (a = 0; a < ui32NumSyncs; a++)
		{
			if (aPts[a].ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_CHECK)
			{
				aui32FenceFWAddrsTmp[f] = aPts[a].ui32FWAddr;
				aui32FenceValuesTmp[f] = aPts[a].ui32FenceValue;
				if (++f == (PVR_SYNC_MAX_QUERY_FENCE_POINTS * ui32NumFDFences))
				{
					PVR_DPF((PVR_DBG_WARNING, "%s: To less space on fence query for all "
							 "the sync points in this fence", __func__));
					goto err_copy;
				}
			}
			if (aPts[a].ui32Flags & PVRSRV_CLIENT_SYNC_PRIM_OP_UPDATE)
			{
				aui32UpdateFWAddrsTmp[u] = aPts[a].ui32FWAddr;
				aui32UpdateValuesTmp[u] = aPts[a].ui32UpdateValue;
				if (++u == (PVR_SYNC_MAX_QUERY_FENCE_POINTS * ui32NumFDFences))
				{
					PVR_DPF((PVR_DBG_WARNING, "%s: To less space on fence query for all "
							 "the sync points in this fence", __func__));
					goto err_copy;
				}
			}
		}
	}

err_copy:
	if (f)
	{
		*ppui32FenceFWAddrs = OSAllocMem(sizeof(IMG_UINT32) * f);
		if (!*ppui32FenceFWAddrs)
		{
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto err_out;
		}
		*ppui32FenceValues = OSAllocMem(sizeof(IMG_UINT32) * f);
		if (!*ppui32FenceValues)
		{
			OSFreeMem(*ppui32FenceFWAddrs);
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto err_out;
		}
		OSMemCopy(*ppui32FenceFWAddrs, aui32FenceFWAddrsTmp, sizeof(IMG_UINT32) * f);
		OSMemCopy(*ppui32FenceValues, aui32FenceValuesTmp, sizeof(IMG_UINT32) * f);
		*pui32NumFenceSyncs = f;
	}
	if (u)
	{
		*ppui32UpdateFWAddrs = OSAllocMem(sizeof(IMG_UINT32) * u);
		if (!*ppui32UpdateFWAddrs)
		{
			OSFreeMem(*ppui32FenceFWAddrs);
			OSFreeMem(*ppui32FenceValues);
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto err_out;
		}
		*ppui32UpdateValues = OSAllocMem(sizeof(IMG_UINT32) * u);
		if (!*ppui32UpdateValues)
		{
			OSFreeMem(*ppui32FenceFWAddrs);
			OSFreeMem(*ppui32FenceValues);
			OSFreeMem(*ppui32UpdateFWAddrs);
			eError = PVRSRV_ERROR_OUT_OF_MEMORY;
			goto err_out;
		}
		OSMemCopy(*ppui32UpdateFWAddrs, aui32UpdateFWAddrsTmp, sizeof(IMG_UINT32) * u);
		OSMemCopy(*ppui32UpdateValues, aui32UpdateValuesTmp, sizeof(IMG_UINT32) * u);
		*pui32NumUpdateSyncs = u;
	}

err_out:
	return eError;
}

IMG_INTERNAL
PVRSRV_ERROR PVRFDSyncNoHwUpdateFenceKM(IMG_INT32 i32FDFence)
{
	struct list_head *psEntry;
	struct sync_fence *psFence = sync_fence_fdget(i32FDFence);
	PVRSRV_ERROR eError = PVRSRV_OK;

	if (!psFence)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: fence for fd=%d not found",
				 __func__, i32FDFence));
		return PVRSRV_ERROR_HANDLE_NOT_FOUND;
	}

	list_for_each(psEntry, &psFence->pt_list_head)
	{
		struct sync_pt *psPt =
			container_of(psEntry, struct sync_pt, pt_list);

		if (psPt->parent->ops == &gsPVR_SYNC_TIMELINE_ops)
		{
			struct PVR_SYNC_PT *psPVRPt = (struct PVR_SYNC_PT *)psPt;
			struct PVR_SYNC_KERNEL_SYNC_PRIM *psSyncKernel =
				psPVRPt->psSyncData->psSyncKernel;

			if (PVRSRVServerSyncPrimSetKM(psSyncKernel->psSync,
										  psSyncKernel->ui32SyncValue) == PVRSRV_OK)
			{
				sync_timeline_signal(psPt->parent);
			}
			else
			{
				PVR_DPF((PVR_DBG_ERROR, "%s: Failed to update backing sync prim. "
						 "This might cause lockups", __func__));
			}
		}
	}

	sync_fence_put(psFence);
	return eError;
}
