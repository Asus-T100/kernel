/****************************************************************************
 *
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 ******************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#if defined(LMA)
#include <linux/pci.h>
#else
#include <linux/dma-mapping.h>
#endif

#include <drm/drmP.h>
#include <drm/drm.h>
#include "psb_drv.h"
#include "ttm/ttm_bo_api.h"
#include "ttm/ttm_placement.h"
#include "ttm/ttm_object.h"

#include "bufferclass_video.h"
#include "bufferclass_video_linux.h"
#include "pvrmodule.h"
#include "sys_pvr_drm_export.h"

#define DEVNAME    "bc_video"
#define    DRVNAME    DEVNAME

#if defined(BCE_USE_SET_MEMORY)
#undef BCE_USE_SET_MEMORY
#endif

#if defined(__i386__) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)) && defined(SUPPORT_LINUX_X86_PAT) && defined(SUPPORT_LINUX_X86_WRITECOMBINE)
#include <asm/cacheflush.h>
#define    BCE_USE_SET_MEMORY
#endif

unsigned int bc_video_id_usage[BC_VIDEO_DEVICE_MAX_ID];

MODULE_SUPPORTED_DEVICE(DEVNAME);

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
static struct class *psPvrClass;
#endif

static int AssignedMajorNumber;

#define unref__ __attribute__ ((unused))

#if defined(LMA)
#define PVR_BUFFERCLASS_MEMOFFSET (220 * 1024 * 1024)
#define PVR_BUFFERCLASS_MEMSIZE      (4 * 1024 * 1024)

unsigned long g_ulMemBase = 0;
unsigned long g_ulMemCurrent = 0;

#define VENDOR_ID_PVR               0x1010
#define DEVICE_ID_PVR               0x1CF1

#define PVR_MEM_PCI_BASENUM         2
#endif

#define file_to_id(file)  (iminor(file->f_path.dentry->d_inode))

int
BC_Video_ModInit(void)
{
	int i, j;
	/*LDM_PCI is defined, while LDM_PLATFORM and LMA are not defined.*/
#if defined(LDM_PLATFORM) || defined(LDM_PCI)
	struct device *psDev;
#endif

#if defined(LMA)
	struct pci_dev *psPCIDev;
	int error;
#endif

#if defined(LMA)
	psPCIDev = pci_get_device(VENDOR_ID_PVR, DEVICE_ID_PVR, NULL);
	if (psPCIDev == NULL) {
		printk(KERN_ERR DRVNAME
		       ": BC_Video_ModInit:  pci_get_device failed\n");
		goto ExitError;
	}

	if ((error = pci_enable_device(psPCIDev)) != 0) {
		printk(KERN_ERR DRVNAME
		       ": BC_Video_ModInit: pci_enable_device failed (%d)\n", error);
		goto ExitError;
	}
#endif

#if defined(DEBUG)
	printk(KERN_ERR DRVNAME ": BC_Video_ModInit: major device %d\n",
	       AssignedMajorNumber);
#endif

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
	psPvrClass = class_create(THIS_MODULE, "bc_video");
	if (IS_ERR(psPvrClass)) {
		printk(KERN_ERR DRVNAME
		       ": BC_Video_ModInit: unable to create class (%ld)",
		       PTR_ERR(psPvrClass));
		goto ExitUnregister;
	}

	psDev = device_create(psPvrClass, NULL, MKDEV(AssignedMajorNumber, 0),
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26))
			      NULL,
#endif
			      DEVNAME);
	if (IS_ERR(psDev)) {
		printk(KERN_ERR DRVNAME
		       ": BC_Video_ModInit: unable to create device (%ld)",
		       PTR_ERR(psDev));
		goto ExitDestroyClass;
	}
#endif

#if defined(LMA)
	g_ulMemBase =
		pci_resource_start(psPCIDev,
				   PVR_MEM_PCI_BASENUM) + PVR_BUFFERCLASS_MEMOFFSET;
#endif

	for (i = 0; i < BC_VIDEO_DEVICE_MAX_ID; i++) {
		bc_video_id_usage[i] = 0;
		if (BC_Video_Init(i) != BCE_OK) {
			printk(KERN_ERR DRVNAME
			       ": BC_Video_ModInit: can't init video bc device %d.\n", i);
			for (j = i; j >= 0; j--) {
				BC_Video_Deinit(j);
			}
			goto ExitUnregister;
		}
	}

	if (BC_Video_Init(BC_CAMERA_DEVICEID) != BCE_OK) {
		for (i = BC_VIDEO_DEVICE_MAX_ID - 1; i >= 0; i--) {
			BC_Video_Deinit(i);
		}
		BC_Video_Deinit(BC_CAMERA_DEVICEID);
		printk(KERN_ERR DRVNAME ": BC_Camera_ModInit: can't init device\n");
		goto ExitUnregister;
	}

#if defined(LMA)
	pci_disable_device(psPCIDev);
#endif

	return 0;

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
ExitDestroyClass:
	class_destroy(psPvrClass);
#endif
ExitUnregister:
	unregister_chrdev(AssignedMajorNumber, DEVNAME);
	//ExitDisable:
#if defined(LMA)
	pci_disable_device(psPCIDev);
ExitError:
#endif
	return -EBUSY;
}

int
BC_Video_ModCleanup(void)
{
	int i;
#if defined(LDM_PLATFORM) || defined(LDM_PCI)
	device_destroy(psPvrClass, MKDEV(AssignedMajorNumber, 0));
	class_destroy(psPvrClass);
#endif

	for (i = 0; i < BC_VIDEO_DEVICE_MAX_ID; i++) {
		if (BC_Video_Deinit(i) != BCE_OK) {
			printk(KERN_ERR DRVNAME
			       ": BC_Video_ModCleanup: can't deinit video device %d.\n",
			       i);
			return -1;
		}
	}

	if (BC_Video_Deinit(BC_CAMERA_DEVICEID) != BCE_OK) {
		printk(KERN_ERR DRVNAME
		       ": BC_Camera_ModCleanup: can't deinit device\n");
		return -1;
	}

	return 0;
}

void *
BCAllocKernelMem(unsigned long ulSize)
{
	return kmalloc(ulSize, GFP_KERNEL);
}

void
BCFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}

#define RANGE_TO_PAGES(range) (((range) + (PAGE_SIZE - 1)) >> PAGE_SHIFT)
#define    VMALLOC_TO_PAGE_PHYS(vAddr) page_to_phys(vmalloc_to_page(vAddr))

BCE_ERROR
BCAllocDiscontigMemory(unsigned long ulSize,
		       BCE_HANDLE unref__ * phMemHandle,
		       IMG_CPU_VIRTADDR * pLinAddr,
		       IMG_SYS_PHYADDR ** ppPhysAddr)
{
	unsigned long ulPages = RANGE_TO_PAGES(ulSize);
	IMG_SYS_PHYADDR *pPhysAddr;
	unsigned long ulPage;
	IMG_CPU_VIRTADDR LinAddr;

	LinAddr =
		__vmalloc(ulSize, GFP_KERNEL | __GFP_HIGHMEM,
			  pgprot_noncached(PAGE_KERNEL));
	if (!LinAddr) {
		return BCE_ERROR_OUT_OF_MEMORY;
	}

	pPhysAddr = kmalloc(ulPages * sizeof(IMG_SYS_PHYADDR), GFP_KERNEL);
	if (!pPhysAddr) {
		vfree(LinAddr);
		return BCE_ERROR_OUT_OF_MEMORY;
	}

	*pLinAddr = LinAddr;

	for (ulPage = 0; ulPage < ulPages; ulPage++) {
		pPhysAddr[ulPage].uiAddr = VMALLOC_TO_PAGE_PHYS(LinAddr);

		LinAddr += PAGE_SIZE;
	}

	*ppPhysAddr = pPhysAddr;

	return BCE_OK;
}

void
BCFreeDiscontigMemory(unsigned long ulSize,
		      BCE_HANDLE unref__ hMemHandle,
		      IMG_CPU_VIRTADDR LinAddr, IMG_SYS_PHYADDR * pPhysAddr)
{
	kfree(pPhysAddr);

	vfree(LinAddr);
}

BCE_ERROR
BCAllocContigMemory(unsigned long ulSize,
		    BCE_HANDLE unref__ * phMemHandle,
		    IMG_CPU_VIRTADDR * pLinAddr, IMG_CPU_PHYADDR * pPhysAddr)
{
#if defined(LMA)
	void *pvLinAddr;


	if (g_ulMemCurrent + ulSize >= PVR_BUFFERCLASS_MEMSIZE) {
		return (BCE_ERROR_OUT_OF_MEMORY);
	}

	pvLinAddr = ioremap(g_ulMemBase + g_ulMemCurrent, ulSize);

	if (pvLinAddr) {
		pPhysAddr->uiAddr = g_ulMemBase + g_ulMemCurrent;
		*pLinAddr = pvLinAddr;

		g_ulMemCurrent += ulSize;
		return (BCE_OK);
	}
	return (BCE_ERROR_OUT_OF_MEMORY);
#else
#if defined(BCE_USE_SET_MEMORY)
	void *pvLinAddr;
	unsigned long ulAlignedSize = PAGE_ALIGN(ulSize);
	int iPages = (int)(ulAlignedSize >> PAGE_SHIFT);
	int iError;

	pvLinAddr = kmalloc(ulAlignedSize, GFP_KERNEL);
	BUG_ON(((unsigned long) pvLinAddr) & ~PAGE_MASK);

	iError = set_memory_wc((unsigned long) pvLinAddr, iPages);
	if (iError != 0) {
		printk(KERN_ERR DRVNAME
		       ": BCAllocContigMemory:  set_memory_wc failed (%d)\n", iError);
		return (BCE_ERROR_OUT_OF_MEMORY);
	}

	pPhysAddr->uiAddr = virt_to_phys(pvLinAddr);
	*pLinAddr = pvLinAddr;

	return (BCE_OK);
#else
	dma_addr_t dma;
	void *pvLinAddr;

	pvLinAddr = dma_alloc_coherent(NULL, ulSize, &dma, GFP_KERNEL);
	if (pvLinAddr == NULL) {
		return (BCE_ERROR_OUT_OF_MEMORY);
	}

	pPhysAddr->uiAddr = dma;
	*pLinAddr = pvLinAddr;

	return (BCE_OK);
#endif
#endif
}

void
BCFreeContigMemory(unsigned long ulSize,
		   BCE_HANDLE unref__ hMemHandle,
		   IMG_CPU_VIRTADDR LinAddr, IMG_CPU_PHYADDR PhysAddr)
{
#if defined(LMA)
	g_ulMemCurrent -= ulSize;
	iounmap(LinAddr);
#else
#if defined(BCE_USE_SET_MEMORY)
	unsigned long ulAlignedSize = PAGE_ALIGN(ulSize);
	int iError;
	int iPages = (int)(ulAlignedSize >> PAGE_SHIFT);

	iError = set_memory_wb((unsigned long) LinAddr, iPages);
	if (iError != 0) {
		printk(KERN_ERR DRVNAME
		       ": BCFreeContigMemory:  set_memory_wb failed (%d)\n", iError);
	}
	kfree(LinAddr);
#else
	dma_free_coherent(NULL, ulSize, LinAddr, (dma_addr_t) PhysAddr.uiAddr);
#endif
#endif
}

IMG_SYS_PHYADDR
CpuPAddrToSysPAddrBC(IMG_CPU_PHYADDR cpu_paddr)
{
	IMG_SYS_PHYADDR sys_paddr;
	sys_paddr.uiAddr = cpu_paddr.uiAddr;
	return sys_paddr;
}

IMG_CPU_PHYADDR
SysPAddrToCpuPAddrBC(IMG_SYS_PHYADDR sys_paddr)
{
	IMG_CPU_PHYADDR cpu_paddr;
	cpu_paddr.uiAddr = sys_paddr.uiAddr;
	return cpu_paddr;
}

BCE_ERROR
BCOpenPVRServices(BCE_HANDLE * phPVRServices)
{
	*phPVRServices = 0;
	return (BCE_OK);
}


BCE_ERROR
BCClosePVRServices(BCE_HANDLE unref__ hPVRServices)
{
	return (BCE_OK);
}

BCE_ERROR
BCGetLibFuncAddr(BCE_HANDLE unref__ hExtDrv, char *szFunctionName,
		 PFN_BC_GET_PVRJTABLE * ppfnFuncTable)
{
	if (strcmp("PVRGetBufferClassJTable", szFunctionName) != 0) {
		return (BCE_ERROR_INVALID_PARAMS);
	}

	*ppfnFuncTable = PVRGetBufferClassJTable;

	return (BCE_OK);
}

int
BC_CreateBuffers(int id, bc_buf_params_t * p, IMG_BOOL is_conti_addr)
{
	BC_VIDEO_DEVINFO *psDevInfo;
	IMG_UINT32 i, stride, size;
	PVRSRV_PIXEL_FORMAT pixel_fmt;

	if (p->count <= 0)
		return -EINVAL;

	if (p->width <= 1 || p->height <= 1)
		return -EINVAL;
	printk(KERN_ERR "foucc: %x \n", p->fourcc);
	switch (p->fourcc) {
	case BC_PIX_FMT_NV12:
		pixel_fmt = PVRSRV_PIXEL_FORMAT_NV12;
		break;
	case BC_PIX_FMT_UYVY:
		pixel_fmt = PVRSRV_PIXEL_FORMAT_FOURCC_ORG_UYVY;
		break;
	case BC_PIX_FMT_RGB565:
		pixel_fmt = PVRSRV_PIXEL_FORMAT_RGB565;
		p->stride = p->stride << 1;    /* stride for RGB from user space is uncorrect */
		break;
	case BC_PIX_FMT_YUYV:
		pixel_fmt = PVRSRV_PIXEL_FORMAT_FOURCC_ORG_YUYV;
		break;
	case BC_PIX_FMT_YV12:
		pixel_fmt = PVRSRV_PIXEL_FORMAT_YV12;
		break;
	default:
		return -EINVAL;
		break;
	}

	stride = p->stride;

	if (p->type != BC_MEMORY_MMAP && p->type != BC_MEMORY_USERPTR)
		return -EINVAL;

	if ((psDevInfo = GetAnchorPtr(id)) == IMG_NULL)
		return -ENODEV;

	if (psDevInfo->ulNumBuffers)
		BC_DestroyBuffers(id);

	psDevInfo->buf_type = p->type;
	psDevInfo->psSystemBuffer =
		BCAllocKernelMem(sizeof(BC_VIDEO_BUFFER) * p->count);

	if (!psDevInfo->psSystemBuffer)
		return -ENOMEM;

	memset(psDevInfo->psSystemBuffer, 0, sizeof(BC_VIDEO_BUFFER) * p->count);
	size = p->height * stride;
	if (pixel_fmt == PVRSRV_PIXEL_FORMAT_NV12 ||
		pixel_fmt == PVRSRV_PIXEL_FORMAT_YV12)
		size += (stride >> 1) * (p->height >> 1) << 1;

	for (i = 0; i < p->count; i++) {
		IMG_SYS_PHYADDR *pPhysAddr;
		psDevInfo->ulNumBuffers++;
		psDevInfo->psSystemBuffer[i].ulSize = size;
		psDevInfo->psSystemBuffer[i].psSyncData = IMG_NULL;

		/*for discontig buffers, allocate memory for pPhysAddr */
		psDevInfo->psSystemBuffer[i].is_conti_addr = is_conti_addr;
		if (is_conti_addr) {
			pPhysAddr = BCAllocKernelMem(1 * sizeof(IMG_SYS_PHYADDR));
			if (!pPhysAddr) {
				return BCE_ERROR_OUT_OF_MEMORY;
			}
			memset(pPhysAddr, 0, 1 * sizeof(IMG_SYS_PHYADDR));
		} else {
			unsigned long ulPages = RANGE_TO_PAGES(size);
			pPhysAddr = BCAllocKernelMem(ulPages * sizeof(IMG_SYS_PHYADDR));
			if (!pPhysAddr) {
				return BCE_ERROR_OUT_OF_MEMORY;
			}
			memset(pPhysAddr, 0, ulPages * sizeof(IMG_SYS_PHYADDR));
		}
		psDevInfo->psSystemBuffer[i].psSysAddr = pPhysAddr;
	}
	p->count = psDevInfo->ulNumBuffers;

	psDevInfo->sBufferInfo.ui32BufferCount = psDevInfo->ulNumBuffers;
	psDevInfo->sBufferInfo.pixelformat = pixel_fmt;
	psDevInfo->sBufferInfo.ui32Width = p->width;
	psDevInfo->sBufferInfo.ui32Height = p->height;
	psDevInfo->sBufferInfo.ui32ByteStride = stride;
	psDevInfo->sBufferInfo.ui32BufferDeviceID = id;
	psDevInfo->sBufferInfo.ui32Flags = PVRSRV_BC_FLAGS_YUVCSC_FULL_RANGE |
					   PVRSRV_BC_FLAGS_YUVCSC_BT601;
	return 0;
}

int
BC_DestroyBuffers(int id)
{
	BC_VIDEO_DEVINFO *psDevInfo;
	IMG_UINT32 i, j;

	if ((psDevInfo = GetAnchorPtr(id)) == IMG_NULL)
		return -ENODEV;

	if (!psDevInfo->ulNumBuffers)
		return 0;

	for (i = 0; i < psDevInfo->ulNumBuffers; i++)
	{
		if((!psDevInfo->psSystemBuffer[i].is_conti_addr) && (psDevInfo->psSystemBuffer[i].need_put_page))
		{
			IMG_UINT32 num_pages = (psDevInfo->psSystemBuffer[i].ulSize+ PAGE_SIZE - 1) / PAGE_SIZE;
			for(j = 0; j < num_pages; j++)
			{
				struct page *page = pfn_to_page(psDevInfo->psSystemBuffer[i].psSysAddr[j].uiAddr >> PAGE_SHIFT);

				SetPageDirty(page);
				page_cache_release(page);
			}
		}
		BCFreeKernelMem(psDevInfo->psSystemBuffer[i].psSysAddr);
	}

	BCFreeKernelMem(psDevInfo->psSystemBuffer);

	psDevInfo->ulNumBuffers = 0;
	psDevInfo->sBufferInfo.pixelformat = PVRSRV_PIXEL_FORMAT_UNKNOWN;
	psDevInfo->sBufferInfo.ui32Width = 0;
	psDevInfo->sBufferInfo.ui32Height = 0;
	psDevInfo->sBufferInfo.ui32ByteStride = 0;
	psDevInfo->sBufferInfo.ui32BufferDeviceID = id;
	psDevInfo->sBufferInfo.ui32Flags = 0;
	psDevInfo->sBufferInfo.ui32BufferCount = psDevInfo->ulNumBuffers;

	return 0;
}

int
GetBufferCount(unsigned int *puiBufferCount, int id)
{
	BC_VIDEO_DEVINFO *psDevInfo = GetAnchorPtr(id);

	if (psDevInfo == IMG_NULL) {
		return -1;
	}

	*puiBufferCount = (unsigned int) psDevInfo->sBufferInfo.ui32BufferCount;

	return 0;
}

int
BC_Video_Bridge(struct drm_device *dev, IMG_VOID * arg,
		struct drm_file *file_priv)
{
	int err = -EFAULT;

	BC_VIDEO_DEVINFO *devinfo;
	int i;
	BC_Video_ioctl_package *psBridge = (BC_Video_ioctl_package *) arg;
	int command = psBridge->ioctl_cmd;
	int id = 0;

	if (command == BC_Video_ioctl_request_buffers) {
		for (i = 0; i < BC_VIDEO_DEVICE_MAX_ID; i++) {
			if (bc_video_id_usage[i] == 0) {
				bc_video_id_usage[i] = 1;
				id = i;
				break;
			}
		}
		if (i == BC_VIDEO_DEVICE_MAX_ID) {
			printk(KERN_ERR DRVNAME
			       " : Does you really need to run more than 5 video simulateously.\n");
			return -1;
		} else
			psb_fpriv(file_priv)->bcd_index = id;
	} else
		id = psBridge->device_id;

	if ((devinfo = GetAnchorPtr(id)) == IMG_NULL)
		return -ENODEV;

	switch (command) {
	case BC_Video_ioctl_get_buffer_count: {
		if (GetBufferCount(&psBridge->outputparam, id) == -1) {
			printk(KERN_ERR DRVNAME
			       " : GetBufferCount error in BC_Video_Bridge.\n");
			return err;
		}
		return 0;
		break;
	}
	case BC_Video_ioctl_get_buffer_index: {
		int idx;
		BC_VIDEO_BUFFER *buffer;

		for (idx = 0; idx < devinfo->ulNumBuffers; idx++) {
			buffer = &devinfo->psSystemBuffer[idx];

			if (psBridge->inputparam == buffer->sBufferHandle) {
				psBridge->outputparam = idx;
				return 0;
			}
		}
		printk(KERN_ERR DRVNAME ": BCIOGET_BUFFERIDX- buffer not found\n");
		return -EINVAL;
		break;
	}
	case BC_Video_ioctl_request_buffers: {
		bc_buf_params_t p;
		if (copy_from_user
		    (&p, (void __user *)(psBridge->inputparam), sizeof(p))) {
			printk(KERN_ERR " : failed to copy inputparam to kernel.\n");
			return -EFAULT;
		}
		psBridge->outputparam = id;
		return BC_CreateBuffers(id, &p, IMG_FALSE);
		break;
	}
	case BC_Video_ioctl_set_buffer_phyaddr: {
		bc_buf_ptr_t p;
		struct ttm_buffer_object *bo = NULL;
		struct ttm_tt *ttm = NULL;
		struct ttm_object_file *tfile = psb_fpriv(file_priv)->tfile;

		if (copy_from_user
		    (&p, (void __user *)(psBridge->inputparam), sizeof(p))) {
			printk(KERN_ERR DRVNAME
			       " : failed to copy inputparam to kernel.\n");
			return -EFAULT;
		}

		if (p.index >= devinfo->ulNumBuffers || !p.handle) {
			printk(KERN_ERR DRVNAME
			       " : index big than NumBuffers or p.handle is NULL.\n");
			return -EINVAL;
		}

		bo = ttm_buffer_object_lookup(tfile, p.handle);
		if (unlikely(bo == NULL)) {
			printk(KERN_ERR DRVNAME
			       " : Could not find buffer object for setstatus.\n");
			return -EINVAL;
		}
		ttm = bo->ttm;

		devinfo->psSystemBuffer[p.index].sCPUVAddr = NULL;
		devinfo->psSystemBuffer[p.index].sBufferHandle = p.handle;
		for (i = 0; i < ttm->num_pages; i++) {
			if (ttm->pages[i] == NULL) {
				printk(KERN_ERR " : Debug: the page is NULL.\n");
				return -EINVAL;
			}
			devinfo->psSystemBuffer[p.index].psSysAddr[i].uiAddr =
				page_to_pfn(ttm->pages[i]) << PAGE_SHIFT;
		}
		if (bo)
			ttm_bo_unref(&bo);
		return 0;
		break;
	}
	case BC_Video_ioctl_release_buffer_device: {
		if (id >= BC_VIDEO_DEVICE_MAX_ID) {
			printk(KERN_ERR " : Debug: Invalid parameter.\n");
			return -EINVAL;
		}
		bc_video_id_usage[id] = 0;

		BC_DestroyBuffers(id);
		return 0;
		break;
	}
	case BC_Video_ioctl_alloc_buffer: {
		bc_buf_ptr_t p;
		IMG_VOID *pvBuf;
		IMG_UINT32 ui32Size;
		IMG_UINT32 ulPagesNumber;
		IMG_UINT32 ulCounter;
		BUFFER_INFO *bufferInfo;

		if (copy_from_user
		    (&p, (void __user *)(psBridge->inputparam), sizeof(p))) {
			printk(KERN_ERR DRVNAME
			       " : failed to copy inputparam to kernel.\n");
			return -EFAULT;
		}

		if (p.index >= devinfo->ulNumBuffers) {
			printk(KERN_ERR DRVNAME " : index big than NumBuffers.\n");
			return -EINVAL;
		}

		bufferInfo = &(devinfo->sBufferInfo);
		if (bufferInfo->pixelformat != PVRSRV_PIXEL_FORMAT_NV12 &&
			bufferInfo->pixelformat != PVRSRV_PIXEL_FORMAT_YV12) {
			printk(KERN_ERR DRVNAME
			       " : BC_Video_ioctl_alloc_buffer only support NV12/YV12 format.\n");
			return -EINVAL;
		}
		ui32Size = bufferInfo->ui32Height * bufferInfo->ui32ByteStride;
		ui32Size +=
			(bufferInfo->ui32ByteStride >> 1) *
			(bufferInfo->ui32Height >> 1) << 1;

		pvBuf =
			__vmalloc(ui32Size, GFP_KERNEL | __GFP_HIGHMEM,
				  __pgprot((pgprot_val(PAGE_KERNEL) & ~_PAGE_CACHE_MASK)
					   | _PAGE_CACHE_WC));
		if (pvBuf == NULL) {
			printk(KERN_ERR DRVNAME
			       " : Failed to allocate %d bytes buffer.\n", ui32Size);
			return -EINVAL;
		}
		devinfo->psSystemBuffer[p.index].sCPUVAddr = pvBuf;
		devinfo->psSystemBuffer[p.index].sBufferHandle = 0;

		ulPagesNumber = (ui32Size + PAGE_SIZE - 1) / PAGE_SIZE;
		i = 0;

		for (ulCounter = 0; ulCounter < ui32Size; ulCounter += PAGE_SIZE) {
			devinfo->psSystemBuffer[p.index].psSysAddr[i++].uiAddr =
				vmalloc_to_pfn(pvBuf + ulCounter) << PAGE_SHIFT;
		}

		if (p.handle) {
			printk(KERN_ERR DRVNAME
			       " : fill data %d bytes from user space 0x%x.\n", ui32Size,
			       (int) p.handle);
			if (copy_from_user(pvBuf, (void __user *) p.handle, ui32Size)) {
				printk(KERN_ERR DRVNAME
				       " : failed to copy inputparam to kernel.\n");
				return -EFAULT;
			}

		}
		psBridge->outputparam = (int) pvBuf;

		return 0;
		break;
	}
	case BC_Video_ioctl_free_buffer: {
		bc_buf_ptr_t p;

		if (copy_from_user
		    (&p, (void __user *)(psBridge->inputparam), sizeof(p))) {
			printk(KERN_ERR DRVNAME
			       " : failed to copy inputparam to kernel.\n");
			return -EFAULT;
		}

		vfree(devinfo->psSystemBuffer[p.index].sCPUVAddr);
		return 0;
		break;
	}
	case BC_Video_ioctl_get_buffer_handle: {
		int idx;
		BC_VIDEO_BUFFER *buffer;

		idx = (int)psBridge->inputparam;

		if (idx > devinfo->ulNumBuffers || idx < 0) {
			printk(KERN_ERR DRVNAME
				" : Invaild device ID %d\n", idx);
			return -EINVAL;
		}

		buffer = &devinfo->psSystemBuffer[idx];
		psBridge->outputparam = buffer->sBufferHandle;

		return 0;
	}
	default:
		return err;
	}

	return 0;
}

int
BC_Camera_Bridge(BC_Video_ioctl_package * psBridge, unsigned long pAddr)
{
	int err = -EFAULT;
	BC_VIDEO_DEVINFO *devinfo;
	int id = BC_CAMERA_DEVICEID;
	int command = psBridge->ioctl_cmd;

	if ((devinfo = GetAnchorPtr(BC_CAMERA_DEVICEID)) == IMG_NULL)
		return -ENODEV;

	switch (command) {
	case BC_Video_ioctl_get_buffer_count: {
		if (GetBufferCount(&psBridge->outputparam, id) == -1) {
			printk(KERN_ERR DRVNAME
			       " : GetBufferCount error in BC_Video_Bridge.\n");
			return err;
		}
		return 0;
		break;
	}
	case BC_Video_ioctl_get_buffer_index: {
		int idx;
		BC_VIDEO_BUFFER *buffer;

		for (idx = 0; idx < devinfo->ulNumBuffers; idx++) {
			buffer = &devinfo->psSystemBuffer[idx];

			if (psBridge->inputparam == buffer->sBufferHandle) {
				psBridge->outputparam = idx;
				return 0;
			}
		}
		printk(KERN_ERR DRVNAME ": BCIOGET_BUFFERIDX- buffer not found\n");
		return -EINVAL;
		break;
	}
	case BC_Video_ioctl_request_buffers: {
		bc_buf_params_t p;
		memcpy(&p, (void *)(psBridge->inputparam), sizeof(p));
		if (p.type == BC_MEMORY_MMAP)
			return BC_CreateBuffers(id, &p, IMG_TRUE);
		else
			return BC_CreateBuffers(id, &p, IMG_FALSE);
		break;
	}
	case BC_Video_ioctl_release_buffer_device: {
		return BC_DestroyBuffers(id);
		break;
	}
	case BC_Video_ioctl_set_buffer_phyaddr: {
		bc_buf_ptr_t p;

		if (copy_from_user
		    (&p, (void __user *)(psBridge->inputparam), sizeof(p))) {
			printk(KERN_ERR DRVNAME
			       " : failed to copy inputparam to kernel.\n");
			return -EFAULT;
		}

		if (p.index >= devinfo->ulNumBuffers) {
			printk(KERN_ERR DRVNAME " : index big than NumBuffers\n");
			return -EINVAL;
		}
		if (devinfo->psSystemBuffer[p.index].is_conti_addr) {
			/* Get the physical address of each frame */
			devinfo->psSystemBuffer[p.index].psSysAddr[0].uiAddr =
				pAddr +
				p.index * PAGE_ALIGN(devinfo->psSystemBuffer[p.index].ulSize);
		} else {
			int i, num_pages, map_pages;
			unsigned int start_addr = p.pa;
			struct page **ppsPages;

			if (start_addr & ~PAGE_MASK) {
				printk(KERN_ERR DRVNAME
				       " : the virtual address must be PAGE aligned.\n");
				return -EFAULT;
			}
			num_pages = (p.size + PAGE_SIZE - 1) / PAGE_SIZE;

			ppsPages = kmalloc((size_t)num_pages * sizeof(*ppsPages),  GFP_KERNEL);

			if (ppsPages == NULL) {
				printk(KERN_ERR DRVNAME
				       " : fails to alloc page array.\n");
				return -EFAULT;
			}
			memset(ppsPages, 0, (size_t)num_pages * sizeof(*ppsPages));

			map_pages = get_user_pages(current, current->mm, p.pa, num_pages, 1, 0, ppsPages, NULL);

			if (map_pages != num_pages) {
				printk(KERN_ERR DRVNAME
				       " : Couldn't map all the pages needed (wanted: %d, got %d).\n", num_pages, map_pages);
				return -EFAULT;
			}
			devinfo->psSystemBuffer[p.index].sCPUVAddr = NULL;
			devinfo->psSystemBuffer[p.index].sBufferHandle = 0;
			devinfo->psSystemBuffer[p.index].ulSize = p.size;
			devinfo->psSystemBuffer[p.index].need_put_page = IMG_TRUE;
			for (i = 0; i < num_pages; i++) {
				devinfo->psSystemBuffer[p.index].psSysAddr[i].uiAddr =
					page_to_pfn(ppsPages[i]) << PAGE_SHIFT;
			}

			kfree(ppsPages);
		}
		return 0;
		break;
	}
	default:
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(BC_Camera_Bridge);

int BCGetBuffer(int devId, int bufferId, BC_VIDEO_BUFFER **bc_buffer)
{
	BC_VIDEO_DEVINFO *devInfo;
	unsigned long bufferCount;
	BC_VIDEO_BUFFER *buffer;

	if (devId < 0 || devId > BC_CAMERA_DEVICEID || !bc_buffer)
		return -EINVAL;

	devInfo = GetAnchorPtr(devId);
	if (!devInfo)
		return -ENODEV;

	bufferCount = devInfo->ulNumBuffers;
	if (bufferId >= bufferCount)
		return -EINVAL;

	buffer = &devInfo->psSystemBuffer[bufferId];
	*bc_buffer = buffer;
	return 0;
}
EXPORT_SYMBOL_GPL(BCGetBuffer);

int BCGetDeviceBufferCount(int devId)
{
	BC_VIDEO_DEVINFO *devInfo;

	if (devId < 0 || devId > BC_CAMERA_DEVICEID)
		return -EINVAL;

	devInfo = GetAnchorPtr(devId);
	if (!devInfo)
		return -ENODEV;
	return devInfo->ulNumBuffers;
}
EXPORT_SYMBOL_GPL(BCGetDeviceBufferCount);

int BCGetDeviceStride(int devId)
{
	BC_VIDEO_DEVINFO *devInfo;

	if (devId < 0 || devId > BC_CAMERA_DEVICEID)
		return -EINVAL;

	devInfo = GetAnchorPtr(devId);
	if (!devInfo)
		return -ENODEV;

	return devInfo->sBufferInfo.ui32ByteStride;
}
