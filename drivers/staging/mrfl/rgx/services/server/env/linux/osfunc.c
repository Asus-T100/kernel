/*************************************************************************/ /*!
@File
@Title          Environment related functions
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

#include <linux/version.h>
#include <asm/io.h>
#include <asm/page.h>
#include <asm/div64.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/hugetlb.h> 
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>

#include <linux/string.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/hardirq.h>
#include <linux/timer.h>
#include <linux/capability.h>
#include <asm/uaccess.h>
#include <linux/spinlock.h>
#if defined(PVR_LINUX_MISR_USING_WORKQUEUE) || \
	defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE) || \
	defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || \
	defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE) || \
	defined(PVR_LINUX_USING_WORKQUEUES)
#include <linux/workqueue.h>
#endif
#include <linux/kthread.h>

#include "osfunc.h"
#include "img_types.h"
#include "mm.h"
#include "allocmem.h"
#include "mmap.h"
#include "env_data.h"
#include "proc.h"
#include "mutex.h"
#include "event.h"
#include "linkage.h"
#include "pvr_uaccess.h"
#include "pvr_debug.h"
#include "driverlock.h"

#if defined(EMULATOR)
#define EVENT_OBJECT_TIMEOUT_MS		(2000)
#else
#define EVENT_OBJECT_TIMEOUT_MS		(100)
#endif /* EMULATOR */

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,24))

static inline int is_vmalloc_addr(const void *pvCpuVAddr)
{
	unsigned long lAddr = (unsigned long)pvCpuVAddr;
	return lAddr >= VMALLOC_START && lAddr < VMALLOC_END;
}

#endif /* (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,24)) */

ENV_DATA *gpsEnvData = IMG_NULL;

PVRSRV_ERROR OSMMUPxAlloc(PVRSRV_DEVICE_NODE *psDevNode, IMG_SIZE_T uiSize,
							Px_HANDLE *psMemHandle, IMG_DEV_PHYADDR *psDevPAddr)
{
	IMG_CPU_PHYADDR sCpuPAddr;
	struct page *psPage;

	/* 
		Check that we're not doing multiple pages worth of
		import as it's not supported a.t.m.
	*/
	PVR_ASSERT(uiSize == PAGE_SIZE);

	psPage = alloc_page(GFP_KERNEL);
	if (psPage == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

#if defined (CONFIG_X86)
	{
		IMG_PVOID pvPageVAddr = page_address(psPage);
		int ret;
		ret = set_memory_wc((unsigned long)pvPageVAddr, 1);

		if (ret)
		{
			__free_page(psPage);
			return PVRSRV_ERROR_UNABLE_TO_SET_CACHE_MODE;
		}
	}
#endif
#if defined(__arm__)
	{
		IMG_CPU_PHYADDR sCPUPhysAddrStart, sCPUPhysAddrEnd;
		IMG_PVOID pvPageVAddr = kmap(psPage);

		sCPUPhysAddrStart.uiAddr = page_to_phys(psPage);
		sCPUPhysAddrEnd.uiAddr = sCPUPhysAddrStart.uiAddr + PAGE_SIZE;

		OSInvalidateCPUCacheRangeKM(pvPageVAddr,
									pvPageVAddr + PAGE_SIZE,
									sCPUPhysAddrStart,
									sCPUPhysAddrEnd);
	}
#endif

	psMemHandle->u.pvHandle = psPage;
	sCpuPAddr.uiAddr = page_to_phys(psPage);

	PhysHeapCpuPAddrToDevPAddr(psDevNode->psPhysHeap, psDevPAddr, &sCpuPAddr);

	return PVRSRV_OK;
}

IMG_VOID OSMMUPxFree(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle)
{
	struct page *psPage = (struct page*) psMemHandle->u.pvHandle;

#if defined (CONFIG_X86)
	{
		IMG_PVOID pvPageVAddr;
		int ret;
	
		pvPageVAddr = page_address(psPage);
		ret = set_memory_wb((unsigned long) pvPageVAddr, 1);
		if (ret)
		{
			PVR_DPF((PVR_DBG_ERROR, "%s: Failed to reset page attribute", __FUNCTION__));
		}
	}
#endif
	__free_page(psPage);
}

PVRSRV_ERROR OSMMUPxMap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle,
						IMG_SIZE_T uiSize, IMG_DEV_PHYADDR *psDevPAddr,
						IMG_VOID **pvPtr)
{
	struct page *psPage = (struct page*) psMemHandle->u.pvHandle;
	IMG_UINTPTR_T uiCPUVAddr;
	pgprot_t prot = PAGE_KERNEL;
	PVR_UNREFERENCED_PARAMETER(psDevNode);

	prot = pgprot_writecombine(prot);
	uiCPUVAddr = (IMG_UINTPTR_T) vm_map_ram(&psPage,
											1,
											-1,
											prot);

	*pvPtr = (IMG_VOID *) ((uiCPUVAddr & (~OSGetPageMask())) |
							((IMG_UINTPTR_T) (psDevPAddr->uiAddr & OSGetPageMask())));
	return PVRSRV_OK;
}

IMG_VOID OSMMUPxUnmap(PVRSRV_DEVICE_NODE *psDevNode, Px_HANDLE *psMemHandle, IMG_VOID *pvPtr)
{
	PVR_UNREFERENCED_PARAMETER(psDevNode);
	PVR_UNREFERENCED_PARAMETER(psMemHandle);

	vm_unmap_ram(pvPtr, 1);
}

/*************************************************************************/ /*!
@Function       OSMemCopy
@Description    Copies memory around
@Input          pvDst    Pointer to dst
@Output         pvSrc    Pointer to src
@Input          ui32Size Bytes to copy
*/ /**************************************************************************/
IMG_VOID OSMemCopy(IMG_VOID *pvDst, IMG_VOID *pvSrc, IMG_SIZE_T ui32Size)
{
#if defined(USE_UNOPTIMISED_MEMCPY)
    IMG_UINT8 *Src,*Dst;
    IMG_INT i;

    Src=(IMG_UINT8 *)pvSrc;
    Dst=(IMG_UINT8 *)pvDst;
    for(i=0;i<ui32Size;i++)
    {
        Dst[i]=Src[i];
    }
#else
    memcpy(pvDst, pvSrc, ui32Size);
#endif
}


/*************************************************************************/ /*!
@Function       OSMemSet
@Description    Function that does the same as the C memset() functions
@Modified      *pvDest     Pointer to start of buffer to be set
@Input          ui8Value   Value to set each byte to
@Input          ui32Size   Number of bytes to set
*/ /**************************************************************************/
IMG_VOID OSMemSet(IMG_VOID *pvDest, IMG_UINT8 ui8Value, IMG_SIZE_T ui32Size)
{
#if defined(USE_UNOPTIMISED_MEMSET)
    IMG_UINT8 *Buff;
    IMG_INT i;

    Buff=(IMG_UINT8 *)pvDest;
    for(i=0;i<ui32Size;i++)
    {
        Buff[i]=ui8Value;
    }
#else
    PVR_ASSERT(pvDest != IMG_NULL);

    memset(pvDest, (char) ui8Value, (size_t) ui32Size);
#endif
}


/*************************************************************************/ /*!
@Function       OSStringCopy
@Description    strcpy
*/ /**************************************************************************/
IMG_CHAR *OSStringCopy(IMG_CHAR *pszDest, const IMG_CHAR *pszSrc)
{
    return (strcpy(pszDest, pszSrc));
}

/*************************************************************************/ /*!
@Function       OSSNPrintf
@Description    snprintf
@Return         the chars written or -1 on error
*/ /**************************************************************************/
IMG_INT32 OSSNPrintf(IMG_CHAR *pStr, IMG_SIZE_T ui32Size, const IMG_CHAR *pszFormat, ...)
{
    va_list argList;
    IMG_INT32 iCount;

    va_start(argList, pszFormat);
    iCount = vsnprintf(pStr, (size_t)ui32Size, pszFormat, argList);
    va_end(argList);

    return iCount;
}

IMG_SIZE_T OSStringLength(const IMG_CHAR *pStr)
{
	return strlen(pStr);
}

IMG_INT32 OSStringCompare(const IMG_CHAR *pStr1, const IMG_CHAR *pStr2)
{
	return strcmp(pStr1, pStr2);
}

/*************************************************************************/ /*!
@Function       OSInitEnvData
@Description    Allocates space for env specific data
@Input          ppvEnvSpecificData   Pointer to pointer in which to return
                                     allocated data.
@Input          ui32MMUMode          MMU mode.
@Return         PVRSRV_OK
*/ /**************************************************************************/
PVRSRV_ERROR OSInitEnvData(IMG_VOID)
{
    /* allocate env specific data */
    gpsEnvData = OSAllocMem(sizeof(ENV_DATA));
    if (gpsEnvData == IMG_NULL)
    {
        return PVRSRV_ERROR_OUT_OF_MEMORY;
    }

    gpsEnvData->pvBridgeData = OSAllocMem(PVRSRV_MAX_BRIDGE_IN_SIZE + PVRSRV_MAX_BRIDGE_OUT_SIZE);
    if (gpsEnvData->pvBridgeData == IMG_NULL)
    {
        OSFreeMem(gpsEnvData);
		/*not nulling pointer, out of scope*/
        return PVRSRV_ERROR_OUT_OF_MEMORY;
    }

    return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSDeInitEnvData
@Description    frees env specific data memory
@Input          pvEnvSpecificData   Pointer to private structure
@Return         PVRSRV_OK on success else PVRSRV_ERROR_OUT_OF_MEMORY
*/ /**************************************************************************/
IMG_VOID OSDeInitEnvData(IMG_VOID)
{
    ENV_DATA *psEnvData = gpsEnvData;

    OSFreeMem(psEnvData->pvBridgeData);
    psEnvData->pvBridgeData = IMG_NULL;

    OSFreeMem(psEnvData);
}

ENV_DATA *OSGetEnvData(IMG_VOID)
{
	return gpsEnvData;
}

/*************************************************************************/ /*!
@Function       OSReleaseThreadQuanta
@Description    Releases thread quanta
*/ /**************************************************************************/ 
IMG_VOID OSReleaseThreadQuanta(IMG_VOID)
{
    schedule();
}


/*************************************************************************/ /*!
@Function       OSClockus
@Description    This function returns the clock in microseconds
@Return         clock (us)
*/ /**************************************************************************/ 
IMG_UINT32 OSClockus(IMG_VOID)
{
    IMG_UINT32 time, j = jiffies;

    time = j * (1000000 / HZ);

    return time;
}


/*
	OSWaitus
*/
IMG_VOID OSWaitus(IMG_UINT32 ui32Timeus)
{
    udelay(ui32Timeus);
}


/*
	OSWaitus
*/
IMG_VOID OSSleepms(IMG_UINT32 ui32Timems)
{
    msleep(ui32Timems);
}


/*************************************************************************/ /*!
@Function       OSGetCurrentProcessIDKM
@Description    Returns handle for current process
@Return         ID of current process
*****************************************************************************/
IMG_PID OSGetCurrentProcessIDKM(IMG_VOID)
{
    if (in_interrupt())
    {
        return KERNEL_ID;
    }

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0))
    return (IMG_PID)current->pgrp;
#else
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24))
    return (IMG_PID)task_tgid_nr(current);
#else
    return (IMG_PID)current->tgid;
#endif
#endif
}


/*************************************************************************/ /*!
@Function       OSGetPageSize
@Description    gets page size
@Return         page size
*/ /**************************************************************************/
IMG_SIZE_T OSGetPageSize(IMG_VOID)
{
    return PAGE_SIZE;
}

/*************************************************************************/ /*!
@Function       OSGetPageShift
@Description    gets page size
@Return         page size
*/ /**************************************************************************/
IMG_SIZE_T OSGetPageShift(IMG_VOID)
{
    return PAGE_SHIFT;
}

/*************************************************************************/ /*!
@Function       OSGetPageMask
@Description    gets page mask
@Return         page size
*/ /**************************************************************************/
IMG_SIZE_T OSGetPageMask(IMG_VOID)
{
    return (OSGetPageSize()-1);
}

typedef struct _LISR_DATA_ {
	PFN_LISR pfnLISR;
	IMG_VOID *pvData;
	IMG_UINT32 ui32IRQ;
} LISR_DATA;

/*
	DeviceISRWrapper
*/
static irqreturn_t DeviceISRWrapper(int irq, void *dev_id)
{
	LISR_DATA *psLISRData = (LISR_DATA *) dev_id;
    IMG_BOOL bStatus = IMG_FALSE;

    PVR_UNREFERENCED_PARAMETER(irq);

    bStatus = psLISRData->pfnLISR(psLISRData->pvData);

    return bStatus ? IRQ_HANDLED : IRQ_NONE;
}

/*
	OSInstallDeviceLISR
*/
PVRSRV_ERROR OSInstallDeviceLISR(PVRSRV_DEVICE_CONFIG *psDevConfig,
									IMG_HANDLE *hLISRData, PFN_LISR pfnLISR,
									IMG_VOID *pvData)
{
	LISR_DATA *psLISRData;
	unsigned long flags = 0;

	psLISRData = kmalloc(sizeof(LISR_DATA), GFP_KERNEL);

	if (psDevConfig->bIRQIsShared)
	{
		flags = IRQF_SHARED;
	}

	psLISRData->pfnLISR = pfnLISR;
	psLISRData->pvData = pvData;
	psLISRData->ui32IRQ = psDevConfig->ui32IRQ;

    PVR_TRACE(("Installing device LISR %s on IRQ %d with cookie %p", psDevConfig->pszName, psDevConfig->ui32IRQ, pvData));

    if(request_irq(psDevConfig->ui32IRQ, DeviceISRWrapper,
        flags, psDevConfig->pszName, psLISRData))
    {
        PVR_DPF((PVR_DBG_ERROR,"OSInstallDeviceLISR: Couldn't install device LISR on IRQ %d", psDevConfig->ui32IRQ));

        return PVRSRV_ERROR_UNABLE_TO_INSTALL_ISR;
    }

	*hLISRData = (IMG_HANDLE) psLISRData;

    return PVRSRV_OK;
}

/*
	OSUninstallDeviceLISR
*/
PVRSRV_ERROR OSUninstallDeviceLISR(IMG_HANDLE hLISRData)
{
	LISR_DATA *psLISRData = (LISR_DATA *) hLISRData;

    PVR_TRACE(("Uninstalling device LISR on IRQ %d with cookie %p", psLISRData->ui32IRQ,  psLISRData->pvData));

    free_irq(psLISRData->ui32IRQ, psLISRData);
	kfree(psLISRData);

    return PVRSRV_OK;
}

#if defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE)
typedef struct  _MISR_DATA_ {
	struct workqueue_struct *psWorkQueue;
	struct work_struct sMISRWork;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

/*
	MISRWrapper
*/
static void MISRWrapper(struct work_struct *data)
{
	MISR_DATA *psMISRData = container_of(data, MISR_DATA, sMISRWork);

	psMISRData->pfnMISR(psMISRData->hData);
}

/*
	OSInstallMISR
*/
PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR,
							IMG_VOID *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = kmalloc(sizeof(MISR_DATA), GFP_KERNEL);
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	psMISRData->psWorkQueue = create_singlethread_workqueue("pvr_workqueue");

	if (psMISRData->psWorkQueue == IMG_NULL)
	{
		PVR_DPF((PVR_DBG_ERROR, "OSInstallMISR: create_singlethreaded_workqueue failed"));
		kfree(psMISRData);
		return PVRSRV_ERROR_UNABLE_TO_CREATE_THREAD;
	}

	INIT_WORK(&psMISRData->sMISRWork, MISRWrapper);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}

/*
	OSUninstallMISR
*/
PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

	PVR_TRACE(("Uninstalling MISR"));

	destroy_workqueue(psMISRData->psWorkQueue);
	kfree(psMISRData);

	return PVRSRV_OK;
}

/*
	OSScheduleMISR
*/
PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

	/*
		Note:

		In the case of NO_HARDWARE we want the driver to be synchronous so
		that we don't have to worry about waiting for previous operations
		to complete
	*/
#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	queue_work(psMISRData->psWorkQueue, &psMISRData->sMISRWork);
#endif
	return PVRSRV_OK;
}
#else	/* defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE) */
#if defined(PVR_LINUX_MISR_USING_WORKQUEUE)
typedef struct  _MISR_DATA_ {
	struct work_struct sMISRWork;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

/*
	MISRWrapper
*/
static void MISRWrapper(struct work_struct *data)
{
	MISR_DATA *psMISRData = container_of(data, MISR_DATA, sMISRWork);

	psMISRData->pfnMISR(psMISRData->hData);
}

/*
	OSInstallMISR
*/
PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR, IMG_VOID *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = kmalloc(sizeof(MISR_DATA), GFP_KERNEL);
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

	PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

	INIT_WORK(&psMISRData->sMISRWork, MISRWrapper);

	*hMISRData = (IMG_HANDLE) psMISRData;

	return PVRSRV_OK;
}


/*
	OSUninstallMISR
*/
PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	PVR_TRACE(("Uninstalling MISR"));

	flush_scheduled_work();

	kfree(hMISRData);

	return PVRSRV_OK;
}

/*
	OSScheduleMISR
*/
PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = hMISRData;
#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	schedule_work(&psMISRData->sMISRWork);
#endif
	return PVRSRV_OK;
}

#else	/* #if defined(PVR_LINUX_MISR_USING_WORKQUEUE) */
typedef struct _MISR_DATA_ {
	struct tasklet_struct sMISRTasklet;
	PFN_MISR pfnMISR;
	void *hData;
} MISR_DATA;

/*
	MISRWrapper
*/
static void MISRWrapper(unsigned long data)
{
    MISR_DATA *psMISRData = (MISR_DATA *) data;

    psMISRData->pfnMISR(psMISRData->hData);
}

/*
	OSInstallMISR
*/
PVRSRV_ERROR OSInstallMISR(IMG_HANDLE *hMISRData, PFN_MISR pfnMISR, IMG_VOID *hData)
{
	MISR_DATA *psMISRData;

	psMISRData = kmalloc(sizeof(MISR_DATA), GFP_KERNEL);
	if (psMISRData == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psMISRData->hData = hData;
	psMISRData->pfnMISR = pfnMISR;

    PVR_TRACE(("Installing MISR with cookie %p", psMISRData));

    tasklet_init(&psMISRData->sMISRTasklet, MISRWrapper, (unsigned long)psMISRData);

	*hMISRData = (IMG_HANDLE) psMISRData;

    return PVRSRV_OK;
}

/*
	OSUninstallMISR
*/
PVRSRV_ERROR OSUninstallMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

    PVR_TRACE(("Uninstalling MISR"));

    tasklet_kill(&psMISRData->sMISRTasklet);

    return PVRSRV_OK;
}

/*
	OSScheduleMISR
*/
PVRSRV_ERROR OSScheduleMISR(IMG_HANDLE hMISRData)
{
	MISR_DATA *psMISRData = (MISR_DATA *) hMISRData;

#if defined(NO_HARDWARE)
	psMISRData->pfnMISR(psMISRData->hData);
#else
	tasklet_schedule(&psMISRData->sMISRTasklet);
#endif
	return PVRSRV_OK;
}

#endif /* #if defined(PVR_LINUX_MISR_USING_WORKQUEUE) */
#endif /* #if defined(PVR_LINUX_MISR_USING_PRIVATE_WORKQUEUE) */

typedef struct {
	struct task_struct *kthread;
	PFN_THREAD pfnThread;
	IMG_VOID *hData;
} OSThreadData;

static int OSThreadRun(void *data)
{
	OSThreadData *psOSThreadData = data;

	/* Call the client's kernel thread with the client's data pointer */
	psOSThreadData->pfnThread(psOSThreadData->hData);

	/* Wait for OSThreadDestroy() to call kthread_stop() */
	while (!kthread_should_stop())
	{
		 schedule();
	}

	return 0;
}

PVRSRV_ERROR OSThreadCreate(IMG_HANDLE *phThread,
							IMG_CHAR *pszThreadName,
							PFN_THREAD pfnThread,
							IMG_VOID *hData)
{
	OSThreadData *psOSThreadData;
	PVRSRV_ERROR eError;

	psOSThreadData = kmalloc(sizeof(OSThreadData), GFP_KERNEL);
	if (psOSThreadData == IMG_NULL)
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_alloc;
	}

	psOSThreadData->pfnThread = pfnThread;
	psOSThreadData->hData = hData;
	psOSThreadData->kthread = kthread_run(OSThreadRun, psOSThreadData, pszThreadName);

	if (IS_ERR(psOSThreadData->kthread))
	{
		eError = PVRSRV_ERROR_OUT_OF_MEMORY;
		goto fail_kthread;
	}

	*phThread = psOSThreadData;

	return PVRSRV_OK;

fail_kthread:
	kfree(psOSThreadData);
fail_alloc:
	PVR_ASSERT(eError != PVRSRV_OK);
	return eError;
}

PVRSRV_ERROR OSThreadDestory(IMG_HANDLE hThread)
{
	OSThreadData *psOSThreadData = hThread;
	int ret;

	/* Let the thread know we are ready for it to end and wait for it. */
	ret = kthread_stop(psOSThreadData->kthread);
	PVR_ASSERT(ret == 0);
	kfree(psOSThreadData);

	return PVRSRV_OK;
}

IMG_VOID OSPanic(IMG_VOID)
{
	BUG();
}

/*************************************************************************/ /*!
@Function       OSMapPhysToLin
@Description    Maps the physical memory into linear addr range
@Input          BasePAddr       Physical cpu address
@Input          ui32Bytes       Bytes to map
@Input          ui32CacheType   Cache type
@Return         Linear addr of mapping on success, else NULL
 */ /**************************************************************************/
IMG_VOID *
OSMapPhysToLin(IMG_CPU_PHYADDR BasePAddr,
               IMG_SIZE_T ui32Bytes,
               IMG_UINT32 ui32MappingFlags)
{
	IMG_VOID *pvIORemapCookie;
	
	pvIORemapCookie = IORemapWrapper(BasePAddr, ui32Bytes, ui32MappingFlags);
	if(pvIORemapCookie == IMG_NULL)
	{
		PVR_ASSERT(0);
	    return IMG_NULL;
	}
	
	return pvIORemapCookie;
}


/*************************************************************************/ /*!
@Function       OSUnMapPhysToLin
@Description    Unmaps memory that was mapped with OSMapPhysToLin
@Input          pvLinAddr
@Input          ui32Bytes
@Return         TRUE on success, else FALSE
*/ /**************************************************************************/
IMG_BOOL
OSUnMapPhysToLin(IMG_VOID *pvLinAddr, IMG_SIZE_T ui32Bytes, IMG_UINT32 ui32MappingFlags)
{
    PVR_UNREFERENCED_PARAMETER(ui32Bytes);	

	IOUnmapWrapper(pvLinAddr);

	return IMG_TRUE;
}

/*
	OSReadHWReg8
*/
IMG_UINT8 OSReadHWReg8(IMG_PVOID	pvLinRegBaseAddr,
					 	IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
    return (IMG_UINT8) readb((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x4e;	/* FIXME: OSReadHWReg should not exist in no hardware builds */
#endif
}

/*
	OSReadHWReg16
*/
IMG_UINT16 OSReadHWReg16(IMG_PVOID	pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
    return (IMG_UINT16) readw((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x3a4e;	/* FIXME: OSReadHWReg should not exist in no hardware builds */
#endif
}

/*
	OSReadHWReg32
*/
IMG_UINT32 OSReadHWReg32(IMG_PVOID	pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
#if !defined(NO_HARDWARE)
    return (IMG_UINT32) readl((IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#else
	return 0x30f73a4e;	/* FIXME: OSReadHWReg should not exist in no hardware builds */
#endif
}


/*
	OSReadHWReg64
*/
IMG_UINT64 OSReadHWReg64(IMG_PVOID	pvLinRegBaseAddr,
						 IMG_UINT32	ui32Offset)
{
	IMG_UINT64	ui64Result;
	
	ui64Result = OSReadHWReg32(pvLinRegBaseAddr, ui32Offset + 4);
	ui64Result <<= 32;
	ui64Result |= OSReadHWReg32(pvLinRegBaseAddr, ui32Offset);
	
	return ui64Result;
}

/*
	OSReadHWRegBank
*/
IMG_DEVMEM_SIZE_T OSReadHWRegBank(IMG_PVOID pvLinRegBaseAddr,
                                  IMG_UINT32 ui32Offset,
                                  IMG_UINT8 *pui8DstBuf,
                                  IMG_DEVMEM_SIZE_T uiDstBufLen)
{
#if !defined(NO_HARDWARE)
	IMG_DEVMEM_SIZE_T uiCounter;

	/* FIXME: optimize this */

	for(uiCounter = 0; uiCounter < uiDstBufLen; uiCounter++) {
		*(pui8DstBuf + uiCounter) =
		  readb(pvLinRegBaseAddr + ui32Offset + uiCounter);
	}

	return uiCounter;
#else
	return uiDstBufLen;
#endif
}

/*
	OSWriteHWReg8
*/
IMG_VOID OSWriteHWReg8(IMG_PVOID	pvLinRegBaseAddr,
					   IMG_UINT32	ui32Offset,
					   IMG_UINT8	ui8Value)
{
#if !defined(NO_HARDWARE)
    writeb(ui8Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}

/*
	OSWriteHWReg16
*/
IMG_VOID OSWriteHWReg16(IMG_PVOID	pvLinRegBaseAddr,
						IMG_UINT32	ui32Offset,
						IMG_UINT16	ui16Value)
{
#if !defined(NO_HARDWARE)
    writew(ui16Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}

/*
	OSWriteHWReg32
*/
IMG_VOID OSWriteHWReg32(IMG_PVOID	pvLinRegBaseAddr,
						IMG_UINT32	ui32Offset,
						IMG_UINT32	ui32Value)
{
#if !defined(NO_HARDWARE)
    writel(ui32Value, (IMG_PBYTE)pvLinRegBaseAddr+ui32Offset);
#endif
}


/*
	OSWriteHWReg64
*/
IMG_VOID OSWriteHWReg64(IMG_PVOID	pvLinRegBaseAddr,
						IMG_UINT32	ui32Offset,
						IMG_UINT64	ui64Value)
{
#if !defined(NO_HARDWARE)
	IMG_UINT32 ui32ValueLow, ui32ValueHigh;

	ui32ValueLow = ui64Value & 0xffffffff;
	ui32ValueHigh = ((IMG_UINT64) (ui64Value >> 32)) & 0xffffffff;

	writel(ui32ValueLow, pvLinRegBaseAddr + ui32Offset);
	writel(ui32ValueHigh, pvLinRegBaseAddr + ui32Offset + 4);
#endif
}

IMG_DEVMEM_SIZE_T OSWriteHWRegBank(IMG_PVOID pvLinRegBaseAddr,
							       IMG_UINT32 ui32Offset,
							       IMG_UINT8 *pui8SrcBuf,
							       IMG_DEVMEM_SIZE_T uiSrcBufLen)
{
#if !defined(NO_HARDWARE)
	IMG_DEVMEM_SIZE_T uiCounter;

	/* FIXME: optimize this */

	for(uiCounter = 0; uiCounter < uiSrcBufLen; uiCounter++) {
		writeb(*(pui8SrcBuf + uiCounter),
		       pvLinRegBaseAddr + ui32Offset + uiCounter);
	}

	return uiCounter;
#else
	return uiSrcBufLen;
#endif
}

#define	OS_MAX_TIMERS	8

/* Timer callback strucure used by OSAddTimer */
typedef struct TIMER_CALLBACK_DATA_TAG
{
    IMG_BOOL			bInUse;
    PFN_TIMER_FUNC		pfnTimerFunc;
    IMG_VOID 			*pvData;	
    struct timer_list		sTimer;
    IMG_UINT32			ui32Delay;
    IMG_BOOL			bActive;
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
    struct work_struct		sWork;
#endif
}TIMER_CALLBACK_DATA;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
static struct workqueue_struct	*psTimerWorkQueue;
#endif

static TIMER_CALLBACK_DATA sTimers[OS_MAX_TIMERS];

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
DEFINE_MUTEX(sTimerStructLock);
#else
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
/* The lock is used to control access to sTimers */
/* PRQA S 0671,0685 1 */ /* C99 macro not understood by QAC */
static spinlock_t sTimerStructLock = SPIN_LOCK_UNLOCKED;
#else
static DEFINE_SPINLOCK(sTimerStructLock);
#endif
#endif

static void OSTimerCallbackBody(TIMER_CALLBACK_DATA *psTimerCBData)
{
    if (!psTimerCBData->bActive)
        return;

    /* call timer callback */
    psTimerCBData->pfnTimerFunc(psTimerCBData->pvData);
    
    /* reset timer */
    mod_timer(&psTimerCBData->sTimer, psTimerCBData->ui32Delay + jiffies);
}


/*************************************************************************/ /*!
@Function       OSTimerCallbackWrapper
@Description    OS specific timer callback wrapper function
@Input          ui32Data    Timer callback data
*/ /**************************************************************************/
static IMG_VOID OSTimerCallbackWrapper(IMG_UINT32 ui32Data)
{
    TIMER_CALLBACK_DATA	*psTimerCBData = (TIMER_CALLBACK_DATA*)ui32Data;
    
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
    int res;

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
    res = queue_work(psTimerWorkQueue, &psTimerCBData->sWork);
#else
    res = schedule_work(&psTimerCBData->sWork);
#endif
    if (res == 0)
    {
        PVR_DPF((PVR_DBG_WARNING, "OSTimerCallbackWrapper: work already queued"));		
    }
#else
    OSTimerCallbackBody(psTimerCBData);
#endif
}


#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
static void OSTimerWorkQueueCallBack(struct work_struct *psWork)
{
    TIMER_CALLBACK_DATA *psTimerCBData = container_of(psWork, TIMER_CALLBACK_DATA, sWork);

    OSTimerCallbackBody(psTimerCBData);
}
#endif

/*************************************************************************/ /*!
@Function       OSAddTimer
@Description    OS specific function to install a timer callback
@Input          pfnTimerFunc    Timer callback
@Input         *pvData          Callback data
@Input          ui32MsTimeout   Callback period 
@Return         Valid handle success, NULL failure
*/ /**************************************************************************/
IMG_HANDLE OSAddTimer(PFN_TIMER_FUNC pfnTimerFunc, IMG_VOID *pvData, IMG_UINT32 ui32MsTimeout)
{
    TIMER_CALLBACK_DATA	*psTimerCBData;
    IMG_UINT32		ui32i;
#if !(defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE))
    unsigned long		ulLockFlags;
#endif

    /* check callback */
    if(!pfnTimerFunc)
    {
        PVR_DPF((PVR_DBG_ERROR, "OSAddTimer: passed invalid callback"));		
        return IMG_NULL;		
    }
    
    /* Allocate timer callback data structure */
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
    mutex_lock(&sTimerStructLock);
#else
    spin_lock_irqsave(&sTimerStructLock, ulLockFlags);
#endif
    for (ui32i = 0; ui32i < OS_MAX_TIMERS; ui32i++)
    {
        psTimerCBData = &sTimers[ui32i];
        if (!psTimerCBData->bInUse)
        {
            psTimerCBData->bInUse = IMG_TRUE;
            break;
        }
    }
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
    mutex_unlock(&sTimerStructLock);
#else
    spin_unlock_irqrestore(&sTimerStructLock, ulLockFlags);
#endif
    if (ui32i >= OS_MAX_TIMERS)
    {
        PVR_DPF((PVR_DBG_ERROR, "OSAddTimer: all timers are in use"));		
        return IMG_NULL;	
    }

    psTimerCBData->pfnTimerFunc = pfnTimerFunc;
    psTimerCBData->pvData = pvData;
    psTimerCBData->bActive = IMG_FALSE;
    
    /*
        HZ = ticks per second
        ui32MsTimeout = required ms delay
        ticks = (Hz * ui32MsTimeout) / 1000	
    */
    psTimerCBData->ui32Delay = ((HZ * ui32MsTimeout) < 1000)
                                ?	1
                                :	((HZ * ui32MsTimeout) / 1000);
    /* initialise object */
    init_timer(&psTimerCBData->sTimer);
    
    /* setup timer object */
    /* PRQA S 0307,0563 1 */ /* ignore warning about inconpartible ptr casting */
    psTimerCBData->sTimer.function = (IMG_VOID *)OSTimerCallbackWrapper;
    psTimerCBData->sTimer.data = (IMG_UINT32)psTimerCBData;
    
    return (IMG_HANDLE)(ui32i + 1);
}


static inline TIMER_CALLBACK_DATA *GetTimerStructure(IMG_HANDLE hTimer)
{
    IMG_UINT32 ui32i = ((IMG_UINT32)hTimer) - 1;

    PVR_ASSERT(ui32i < OS_MAX_TIMERS);

    return &sTimers[ui32i];
}

/*************************************************************************/ /*!
@Function       OSRemoveTimer
@Description    OS specific function to remove a timer callback
@Input          hTimer : timer handle
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSRemoveTimer (IMG_HANDLE hTimer)
{
    TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

    PVR_ASSERT(psTimerCBData->bInUse);
    PVR_ASSERT(!psTimerCBData->bActive);

    /* free timer callback data struct */
    psTimerCBData->bInUse = IMG_FALSE;
    
    return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSEnableTimer
@Description    OS specific function to enable a timer callback
@Input          hTimer    Timer handle
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSEnableTimer (IMG_HANDLE hTimer)
{
    TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

    PVR_ASSERT(psTimerCBData->bInUse);
    PVR_ASSERT(!psTimerCBData->bActive);

    /* Start timer arming */
    psTimerCBData->bActive = IMG_TRUE;

    /* set the expire time */
    psTimerCBData->sTimer.expires = psTimerCBData->ui32Delay + jiffies;

    /* Add the timer to the list */
    add_timer(&psTimerCBData->sTimer);
    
    return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSDisableTimer
@Description    OS specific function to disable a timer callback
@Input          hTimer    Timer handle
@Return         PVRSRV_ERROR
*/ /**************************************************************************/
PVRSRV_ERROR OSDisableTimer (IMG_HANDLE hTimer)
{
    TIMER_CALLBACK_DATA *psTimerCBData = GetTimerStructure(hTimer);

    PVR_ASSERT(psTimerCBData->bInUse);
    PVR_ASSERT(psTimerCBData->bActive);

    /* Stop timer from arming */
    psTimerCBData->bActive = IMG_FALSE;
    smp_mb();

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
    flush_workqueue(psTimerWorkQueue);
#endif
#if defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
    flush_scheduled_work();
#endif

    /* remove timer */
    del_timer_sync(&psTimerCBData->sTimer);	
    
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
    /*
     * This second flush is to catch the case where the timer ran
     * before we managed to delete it, in which case, it will have
     * queued more work for the workqueue.  Since the bActive flag
     * has been cleared, this second flush won't result in the
     * timer being rearmed.
     */
    flush_workqueue(psTimerWorkQueue);
#endif
#if defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
    flush_scheduled_work();
#endif

    return PVRSRV_OK;
}


/*************************************************************************/ /*!
@Function       OSEventObjectCreate
@Description    OS specific function to create an event object
@Input          pszName      Globally unique event object name (if null name must be autogenerated)
@Output         hEventObject OS event object info structure
@Return         PVRSRV_ERROR  
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectCreate(const IMG_CHAR *pszName, IMG_HANDLE *hEventObject)
{

    PVRSRV_ERROR eError = PVRSRV_OK;
    PVR_UNREFERENCED_PARAMETER(pszName);

    if(hEventObject)
    {
        if(LinuxEventObjectListCreate(hEventObject) != PVRSRV_OK)
        {
             eError = PVRSRV_ERROR_OUT_OF_MEMORY;
        }

    }
    else
    {
        PVR_DPF((PVR_DBG_ERROR, "OSEventObjectCreate: hEventObject is not a valid pointer"));
        eError = PVRSRV_ERROR_UNABLE_TO_CREATE_EVENT;
    }
    
    return eError;

}


/*************************************************************************/ /*!
@Function       OSEventObjectDestroy
@Description    OS specific function to destroy an event object
@Input          hEventObject   OS event object info structure
@Return         PVRSRV_ERROR   
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectDestroy(IMG_HANDLE hEventObject)
{
    PVRSRV_ERROR eError = PVRSRV_OK;

    if(hEventObject)
    {
        LinuxEventObjectListDestroy(hEventObject);
    }
    else
    {
        PVR_DPF((PVR_DBG_ERROR, "OSEventObjectDestroy: hEventObject is not a valid pointer"));
        eError = PVRSRV_ERROR_INVALID_PARAMS;
    }
    
    return eError;
}

/*************************************************************************/ /*!
@Function       OSEventObjectWaitTimeout
@Description    Wait for an event with timeout as supplied. Called from client
@Input          hOSEventKM    OS and kernel specific handle to event object
@Input          uiTimeoutMs   Non zero time period in milliseconds to wait
@Return         PVRSRV_ERROR_TIMEOUT : Wait reached wait limit and timed out
@Return         PVRSRV_ERROR         : any other system error code
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectWaitTimeout(IMG_HANDLE hOSEventKM, IMG_UINT32 uiTimeoutMs)
{
    PVRSRV_ERROR eError;

    if(hOSEventKM && uiTimeoutMs > 0)
    {
        eError = LinuxEventObjectWait(hOSEventKM, uiTimeoutMs);
    }
    else
    {
        PVR_DPF((PVR_DBG_ERROR, "OSEventObjectWait: invalid arguments %p, %d", hOSEventKM, uiTimeoutMs ));
        eError = PVRSRV_ERROR_INVALID_PARAMS;
    }

    return eError;
}

/*************************************************************************/ /*!
@Function       OSEventObjectWait
@Description    OS specific function to wait for an event object. Called
				from client. Uses a default wait with 100ms timeout.
@Input          hOSEventKM    OS and kernel specific handle to event object
@Return         PVRSRV_ERROR_TIMEOUT  : Reached wait limit and timed out
@Return         PVRSRV_ERROR  : any other system error code
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectWait(IMG_HANDLE hOSEventKM)
{
    return OSEventObjectWaitTimeout(hOSEventKM, EVENT_OBJECT_TIMEOUT_MS);
}

/*************************************************************************/ /*!
@Function       OSEventObjectOpen
@Description    OS specific function to open an event object.  Called from client
@Input          hEventObject  Pointer to an event object
@Output         phOSEvent     OS and kernel specific handle to event object
@Return         PVRSRV_ERROR  
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectOpen(IMG_HANDLE hEventObject,
                                            IMG_HANDLE *phOSEvent)
{
    PVRSRV_ERROR eError = PVRSRV_OK;
    
    if(hEventObject)
    {
        if(LinuxEventObjectAdd(hEventObject, phOSEvent) != PVRSRV_OK)
        {
            PVR_DPF((PVR_DBG_ERROR, "LinuxEventObjectAdd: failed"));
            eError = PVRSRV_ERROR_INVALID_PARAMS;
        }
    }
    else
    {
        PVR_DPF((PVR_DBG_ERROR, "OSEventObjectOpen: hEventObject is not a valid pointer"));
        eError = PVRSRV_ERROR_INVALID_PARAMS;
    }
    
    return eError;
}

/*************************************************************************/ /*!
@Function       OSEventObjectClose
@Description    OS specific function to close an event object.  Called from client
@Input          hOSEventKM    OS and kernel specific handle to event object
@Return         PVRSRV_ERROR  : 
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectClose(IMG_HANDLE hOSEventKM)
{
    PVRSRV_ERROR eError = PVRSRV_OK;

    if(hOSEventKM)
    {
        if(LinuxEventObjectDelete(hOSEventKM) != PVRSRV_OK)
        {
            PVR_DPF((PVR_DBG_ERROR, "LinuxEventObjectDelete: failed"));
            eError = PVRSRV_ERROR_INVALID_PARAMS;
        }

    }
    else
    {
        PVR_DPF((PVR_DBG_ERROR, "OSEventObjectDestroy: hEventObject is not a valid pointer"));
        eError = PVRSRV_ERROR_INVALID_PARAMS;
    }
    
    return eError;
    
}

/*************************************************************************/ /*!
@Function       OSEventObjectSignal
@Description    OS specific function to 'signal' an event object.  Called from L/MISR
@Input          hOSEventKM   OS and kernel specific handle to event object
@Return         PVRSRV_ERROR  
*/ /**************************************************************************/
PVRSRV_ERROR OSEventObjectSignal(IMG_HANDLE hEventObject)
{
    PVRSRV_ERROR eError;
    
    if(hEventObject)
    {
        eError = LinuxEventObjectSignal(hEventObject);
    }
    else
    {
        PVR_DPF((PVR_DBG_ERROR, "OSEventObjectSignal: hOSEventKM is not a valid handle"));
        eError = PVRSRV_ERROR_INVALID_PARAMS;
    }
    
    return eError;
}

/*************************************************************************/ /*!
@Function       OSProcHasPrivSrvInit
@Description    Does the process have sufficient privileges to initialise services?
@Return         IMG_BOOL 
*/ /**************************************************************************/
IMG_BOOL OSProcHasPrivSrvInit(IMG_VOID)
{
    return capable(CAP_SYS_MODULE) != 0;
}

/*************************************************************************/ /*!
@Function       OSCopyToUser
@Description    Copy a block of data into user space
@Input          pvSrc
@Output         pvDest
@Input 	        ui32Bytes 
@Return   PVRSRV_ERROR  : 
*/ /**************************************************************************/
PVRSRV_ERROR OSCopyToUser(IMG_PVOID pvProcess, 
                          IMG_VOID *pvDest, 
                          IMG_VOID *pvSrc, 
                          IMG_SIZE_T ui32Bytes)
{
    PVR_UNREFERENCED_PARAMETER(pvProcess);

    if(pvr_copy_to_user(pvDest, pvSrc, ui32Bytes)==0)
        return PVRSRV_OK;
    else
        return PVRSRV_ERROR_FAILED_TO_COPY_VIRT_MEMORY;
}

/*************************************************************************/ /*!
@Function       OSCopyFromUser
@Description    Copy a block of data from the user space
@Output         pvDest
@Input          pvSrc
@Input 	        ui32Bytes 
@Return         PVRSRV_ERROR  : 
*/ /**************************************************************************/
PVRSRV_ERROR OSCopyFromUser( IMG_PVOID pvProcess, 
                             IMG_VOID *pvDest, 
                             IMG_VOID *pvSrc, 
                             IMG_SIZE_T ui32Bytes)
{
    PVR_UNREFERENCED_PARAMETER(pvProcess);

    if(pvr_copy_from_user(pvDest, pvSrc, ui32Bytes)==0)
        return PVRSRV_OK;
    else
        return PVRSRV_ERROR_FAILED_TO_COPY_VIRT_MEMORY;
}

/*************************************************************************/ /*!
@Function       OSAccessOK
@Description    Checks if a user space pointer is valide
@Input          eVerification
@Input          pvUserPtr
@Input 	        ui32Bytes 
@Return         IMG_BOOL :
*/ /**************************************************************************/
IMG_BOOL OSAccessOK(IMG_VERIFY_TEST eVerification, IMG_VOID *pvUserPtr, IMG_SIZE_T ui32Bytes)
{
    IMG_INT linuxType;

    if (eVerification == PVR_VERIFY_READ)
    {
        linuxType = VERIFY_READ;
    }
    else
    {
        PVR_ASSERT(eVerification == PVR_VERIFY_WRITE);
        linuxType = VERIFY_WRITE;
    }

    return access_ok(linuxType, pvUserPtr, ui32Bytes);
}


IMG_VOID OSWriteMemoryBarrier(IMG_VOID)
{
	wmb();
}


IMG_VOID OSMemoryBarrier(IMG_VOID)
{
	mb();
}

struct _OSWR_LOCK_
{
	struct rw_semaphore sRWLock;
};

PVRSRV_ERROR OSWRLockCreate(POSWR_LOCK *ppsLock)
{
	POSWR_LOCK psLock;

	psLock = kmalloc(sizeof(*psLock), GFP_KERNEL);
	if (psLock == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}
	init_rwsem(&psLock->sRWLock);

	*ppsLock = psLock;

	return PVRSRV_OK;
}

IMG_VOID OSWRLockDestroy(POSWR_LOCK psLock)
{
	kfree(psLock);
}

IMG_VOID OSWRLockAcquireRead(POSWR_LOCK psLock)
{
	down_read(&psLock->sRWLock);
}

IMG_VOID OSWRLockReleaseRead(POSWR_LOCK psLock)
{
	up_read(&psLock->sRWLock);
}

IMG_VOID OSWRLockAcquireWrite(POSWR_LOCK psLock)
{
	down_write(&psLock->sRWLock);
}

IMG_VOID OSWRLockReleaseWrite(POSWR_LOCK psLock)
{
	up_write(&psLock->sRWLock);
}

IMG_UINT32 OSDivide64(IMG_UINT64 ui64Divident, IMG_UINT32 ui32Divisor, IMG_UINT32 *pui32Remainder)
{
	*pui32Remainder = do_div(ui64Divident, ui32Divisor);

	return (IMG_UINT32) ui64Divident;
}

/* One time osfunc initialisation */
PVRSRV_ERROR PVROSFuncInit(IMG_VOID)
{
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
    {
        psTimerWorkQueue = create_workqueue("pvr_timer");
        if (psTimerWorkQueue == NULL)
        {
	    PVR_DPF((PVR_DBG_ERROR, "%s: couldn't create timer workqueue", __FUNCTION__));		
	    return PVRSRV_ERROR_UNABLE_TO_CREATE_THREAD;

        }
    }
#endif

#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES) || defined(PVR_LINUX_TIMERS_USING_SHARED_WORKQUEUE)
    {
	IMG_UINT32 ui32i;

        for (ui32i = 0; ui32i < OS_MAX_TIMERS; ui32i++)
        {
            TIMER_CALLBACK_DATA *psTimerCBData = &sTimers[ui32i];

	    INIT_WORK(&psTimerCBData->sWork, OSTimerWorkQueueCallBack);
        }
    }
#endif
    return PVRSRV_OK;
}

/*
 * Osfunc deinitialisation.
 * Note that PVROSFuncInit may not have been called
 */
IMG_VOID PVROSFuncDeInit(IMG_VOID)
{
#if defined(PVR_LINUX_TIMERS_USING_WORKQUEUES)
    if (psTimerWorkQueue != NULL)
    {
	destroy_workqueue(psTimerWorkQueue);
    }
#endif
}

/* FIXME: need to re-think pvrlock for server syncs */
static IMG_BOOL gbDoRelease = IMG_TRUE;
IMG_VOID OSSetReleasePVRLock(IMG_VOID){ gbDoRelease = IMG_TRUE; }
IMG_VOID OSSetKeepPVRLock(IMG_VOID) { gbDoRelease = IMG_FALSE;}
IMG_BOOL OSGetReleasePVRLock(IMG_VOID){ return gbDoRelease;}
/* end FIXME */

IMG_VOID OSDumpStack(IMG_VOID)
{
	dump_stack();
}

IMG_BOOL OSTryAcquireBridgeLock(IMG_VOID)
{
	return LinuxTryLockMutex(&gPVRSRVLock);
}

IMG_VOID OSAcquireBridgeLock(IMG_VOID)
{
	LinuxLockMutex(&gPVRSRVLock);
}

IMG_VOID OSReleaseBridgeLock(IMG_VOID)
{
	LinuxUnLockMutex(&gPVRSRVLock);
}
