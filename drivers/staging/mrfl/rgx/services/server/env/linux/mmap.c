/*************************************************************************/ /*!
@File
@Title          Linux mmap interface
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
#include <asm/io.h>
#include <linux/mm.h>
#include <asm/page.h>

#include "img_defs.h"
#include "mutils.h"
#include "mmap.h"
#include "pvr_debug.h"
#include "mutex.h"
#include "handle.h"
#include "pvrsrv.h"
#include "connection_server.h"

#include "private_data.h"
#include "driverlock.h"

#if defined(SUPPORT_DRM)
#include "pvr_drm.h"
#endif

#if !defined(PVR_SECURE_HANDLES)
#error "The mmap code requires PVR_SECURE_HANDLES"
#endif

/* WARNING!
 * The mmap code has its own mutex, to prevent a possible deadlock,
 * when using gPVRSRVLock.
 * The Linux kernel takes the mm->mmap_sem before calling the mmap
 * entry points (PVRMMap, MMapVOpen, MMapVClose), but the ioctl
 * entry point may take mm->mmap_sem during fault handling, or 
 * before calling get_user_pages.  If gPVRSRVLock was used in the
 * mmap entry points, a deadlock could result, due to the ioctl
 * and mmap code taking the two locks in different orders.
 * As a corollary to this, the mmap entry points must not call
 * any driver code that relies on gPVRSRVLock is held.
 */
static PVRSRV_LINUX_MUTEX g_sMMapMutex;

#include "pmr.h"

static void MMapPMROpen(struct vm_area_struct* ps_vma)
{
	/* FIXME
    PMR *psPMR;

    psPMR = ps_vma->vm_private_data;
    */

    /* FIXME: up refcount */
    PVR_ASSERT(0);
}

static void MMapPMRClose(struct vm_area_struct *ps_vma)
{
    PMR *psPMR;

    psPMR = ps_vma->vm_private_data;

    PMRUnlockSysPhysAddresses(psPMR);
    PMRUnrefPMR(psPMR);
}

/*
 * This vma operation is used to read data from mmap regions. It is called
 * by access_process_vm, which is called to handle PTRACE_PEEKDATA ptrace
 * requests and reads from /proc/<pid>/mem.
 */
static int MMapVAccess(struct vm_area_struct *ps_vma, unsigned long addr,
					   void *buf, int len, int write)
{
    PMR *psPMR;
    unsigned long ulOffset;
    IMG_SIZE_T uiBytesCopied;
    PVRSRV_ERROR eError;
    int iRetVal = -EINVAL;

    psPMR = ps_vma->vm_private_data;

    ulOffset = addr - ps_vma->vm_start;

	eError = PMR_ReadBytes(psPMR,
						   (IMG_DEVMEM_OFFSET_T) ulOffset,
						   buf,
						   len,
						   &uiBytesCopied);

	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR, "%s: Error from PMR_ReadBytes (%d)",
				 __FUNCTION__, eError));
	}
	else
	{
		iRetVal = uiBytesCopied;
	}

	return iRetVal;
}

static struct vm_operations_struct gsMMapOps =
{
	.open=&MMapPMROpen,
	.close=&MMapPMRClose,
	.access=MMapVAccess,
};

int MMapPMR(struct file* pFile, struct vm_area_struct* ps_vma)
{
    PVRSRV_ERROR eError;
    IMG_HANDLE hSecurePMRHandle;
    IMG_SIZE_T uiLength;
    IMG_DEVMEM_OFFSET_T uiOffset;
    unsigned long uiPFN;
    IMG_HANDLE hPMRResmanHandle;
    PMR *psPMR;
    PMR_FLAGS_T ulPMRFlags;
    unsigned long ulNewFlags = 0;
    pgprot_t sPageProt;

#if defined(SUPPORT_DRM)
    CONNECTION_DATA *psConnection = LinuxConnectionFromFile(pFile->private_data);
#else
    CONNECTION_DATA *psConnection = LinuxConnectionFromFile(pFile);
#endif
	/* FIXME:
	 * Take the bridge mutex to make this code thread/multiprocess safe.
	 * This is not a permanent solution (see comment regarding the
	 * bridge mutex at the top of this file), but is needed to
	 * prevent assertion failures in ResManFindPriavateDataByPtr
	 * when X windows are being frequently resized (e.g. quickly dragging
	 * the corner of a window around when running the metacity window
	 * manager).
	 */
	LinuxLockMutex(&gPVRSRVLock);
	LinuxLockMutex(&g_sMMapMutex);

	hSecurePMRHandle=(IMG_HANDLE)((IMG_UINTPTR_T)ps_vma->vm_pgoff);

	/* FIXME: Only auto generated code should be doing handle lookup */
	eError = PVRSRVLookupHandle(psConnection->psHandleBase,
                                (IMG_HANDLE *) &hPMRResmanHandle,
                                hSecurePMRHandle,
                                PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (eError != PVRSRV_OK)
	{
        goto e0;
	}

    eError = ResManFindPrivateDataByPtr(hPMRResmanHandle,
                                        (IMG_VOID **)&psPMR);
	if (eError != PVRSRV_OK)
	{
        goto e0;
	}

	/*
		Take a reference on the PMR, make's sure that it can't be freed
		while it's mapped into the user process
	*/
	PMRRefPMR(psPMR);

    eError = PMRLockSysPhysAddresses(psPMR, PAGE_SHIFT);
	if (eError != PVRSRV_OK)
	{
        goto e1;
	}

    if (((ps_vma->vm_flags & VM_WRITE) != 0) &&
	((ps_vma->vm_flags & VM_SHARED) == 0))
    {
	eError = PVRSRV_ERROR_INVALID_PARAMS;
	goto e1;
    }

    /*
      FIXME:

      we ought to call PMR_Flags() here to check the permissions
      against the requested mode, and possibly to set up the cache
      control protflags
    */
	eError = PMR_Flags(psPMR, &ulPMRFlags);
	if (eError != PVRSRV_OK)
	{
        goto e1;
	}

	ulNewFlags = ps_vma->vm_flags;
#if 0
	/* Discard user read/write request, we will pull these flags from the PMR */
	ulNewFlags &= ~(VM_READ | VM_WRITE);

	if (ulPMRFlags & PVRSRV_MEMALLOCFLAG_CPU_READABLE)
	{
		ulNewFlags |= VM_READ;
	}
	if (ulPMRFlags & PVRSRV_MEMALLOCFLAG_CPU_WRITEABLE)
	{
		ulNewFlags |= VM_WRITE;
	}
#endif
	/* FIXME: We should check if this is IO memory and mark it as such if so */

	ps_vma->vm_flags = ulNewFlags;

#if defined(__arm__)
	sPageProt = __pgprot_modify(ps_vma->vm_page_prot, L_PTE_MT_MASK, vm_get_page_prot(ulNewFlags));
#elif defined(__i386__)
	sPageProt = pgprot_modify(ps_vma->vm_page_prot,
							   vm_get_page_prot(ulNewFlags));
#else
#error Please add pgprot_modify equivalent for your system
#endif

	switch (ulPMRFlags & PVRSRV_MEMALLOCFLAG_CPU_CACHE_MODE_MASK)
	{
		/* FIXME: What do we do for cache coherent? For now make uncached*/
		case PVRSRV_MEMALLOCFLAG_CPU_CACHE_COHERENT:
		case PVRSRV_MEMALLOCFLAG_CPU_UNCACHED:
				sPageProt = pgprot_noncached(sPageProt);
				break;

		case PVRSRV_MEMALLOCFLAG_CPU_WRITE_COMBINE:
				sPageProt = pgprot_writecombine(sPageProt);
				break;

		case PVRSRV_MEMALLOCFLAG_CPU_CACHE_INCOHERENT:
				break;

		default:
				eError = PVRSRV_ERROR_INVALID_PARAMS;
				goto e1;
	}
	ps_vma->vm_page_prot = sPageProt;

    uiLength = ps_vma->vm_end - ps_vma->vm_start;

    for (uiOffset = 0; uiOffset < uiLength; uiOffset += 1ULL<<PAGE_SHIFT)
    {
        IMG_SIZE_T uiNumContiguousBytes;
        int remap_pfn_range(struct vm_area_struct *vma, unsigned long from,
                            unsigned long pfn, unsigned long size,
                            pgprot_t prot);
        
        IMG_INT32 iStatus;
        IMG_CPU_PHYADDR sCpuPAddr;
        IMG_BOOL bValid;
	struct page *psPage = NULL;


        uiNumContiguousBytes = 1ULL<<PAGE_SHIFT;
        eError = PMR_CpuPhysAddr(psPMR,
                                 uiOffset,
                                 &sCpuPAddr,
                                 &bValid);
        PVR_ASSERT(eError == PVRSRV_OK); // FIXME: how handle error part way through?
        if (eError)
        {
            goto e2;
        }

		/*
			Only map in pages that are valid, any that aren't will be picked up
			by the nopage handler which will return a zeroed page for us
		*/
		if (bValid)
		{
		uiPFN = sCpuPAddr.uiAddr >> PAGE_SHIFT;
		PVR_ASSERT(uiPFN << PAGE_SHIFT == sCpuPAddr.uiAddr);

		PVR_ASSERT(pfn_valid(uiPFN));
		psPage = pfn_to_page(uiPFN);
		iStatus = vm_insert_page(ps_vma,
					ps_vma->vm_start + uiOffset,
					psPage);

	        PVR_ASSERT(iStatus == 0);
	        if(iStatus)
	        {
	            // N.B. not the right error code, but, it doesn't get propagated anyway... :(
	            eError = PVRSRV_ERROR_OUT_OF_MEMORY;
	
	            goto e2;
	        }
		}
        (void)pFile;
    }

    ps_vma->vm_flags |= (VM_IO | VM_RESERVED);

    /*
     * Disable mremap because our nopage handler assumes all
     * page requests have already been validated.
     */
    ps_vma->vm_flags |= VM_DONTEXPAND;
    
    /* Don't allow mapping to be inherited across a process fork */
    ps_vma->vm_flags |= VM_DONTCOPY;

    /* let us see the PMR so we can unlock it later */
    ps_vma->vm_private_data = psPMR;

    /* Install open and close handlers for ref-counting */
    ps_vma->vm_ops = &gsMMapOps;


	LinuxUnLockMutex(&g_sMMapMutex);
	LinuxUnLockMutex(&gPVRSRVLock);
    return 0;

    /*
      error exit paths follow
    */
 e2:
    PVR_DPF((PVR_DBG_ERROR, "don't know how to handle this error.  Abort!"));
    PMRUnlockSysPhysAddresses(psPMR);
 e1:
	PMRUnrefPMR(psPMR);
 e0:
    PVR_ASSERT(eError != PVRSRV_OK);
    PVR_DPF((PVR_DBG_ERROR, "unable to translate error %d", eError));
	LinuxUnLockMutex(&g_sMMapMutex);
	LinuxUnLockMutex(&gPVRSRVLock);
    return -ENOENT; // -EAGAIN // or what?
}

/*!
 *******************************************************************************

 @Function  PVRMMapInit

 @Description

 MMap initialisation code

 ******************************************************************************/
IMG_VOID
PVRMMapInit(IMG_VOID)
{
    LinuxInitMutex(&g_sMMapMutex);
    return;
}

/*!
 *******************************************************************************

 @Function  PVRMMapCleanup

 @Description

 Mmap deinitialisation code

 ******************************************************************************/
IMG_VOID
PVRMMapCleanup(IMG_VOID)
{
}
