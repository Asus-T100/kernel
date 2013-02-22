/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#if defined(LMA)
#include <linux/pci.h>
#else
#include <linux/dma-mapping.h>
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36))
#include <linux/mutex.h>
#endif

#if defined(BC_DISCONTIG_BUFFERS)
#include <linux/vmalloc.h>
#endif

#include "bufferclass_example.h"
#include "bufferclass_example_linux.h"
#include "bufferclass_example_private.h"

#include "pvrmodule.h"

#define DEVNAME	"bc_example"
#define	DRVNAME	DEVNAME

#if defined(BCE_USE_SET_MEMORY)
#undef BCE_USE_SET_MEMORY
#endif

#if defined(__i386__) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)) && defined(SUPPORT_LINUX_X86_PAT) && defined(SUPPORT_LINUX_X86_WRITECOMBINE)
#include <asm/cacheflush.h>
#define	BCE_USE_SET_MEMORY
#endif

MODULE_SUPPORTED_DEVICE(DEVNAME);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36))
static long BC_Example_Bridge_Unlocked(struct file *file, unsigned int cmd, unsigned long arg);
#else
static int BC_Example_Bridge(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36))
static DEFINE_MUTEX(sBCExampleBridgeMutex);
#endif

#if defined(LDM_PLATFORM) || defined(LDM_PCI)
static struct class *psPvrClass;
#endif

static int AssignedMajorNumber;

static struct file_operations bufferclass_example_fops = {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36))
	.unlocked_ioctl = BC_Example_Bridge_Unlocked
#else
	.ioctl = BC_Example_Bridge
#endif
};


#define unref__ __attribute__ ((unused))

#if defined(LMA)
#define PVR_BUFFERCLASS_MEMOFFSET (220 * 1024 * 1024)
#define PVR_BUFFERCLASS_MEMSIZE	  (4 * 1024 * 1024)

unsigned long g_ulMemBase = 0;
unsigned long g_ulMemCurrent = 0;

#define VENDOR_ID_PVR               0x1010
#define DEVICE_ID_PVR               0x1CF1

#define DEVICE_ID1_PVR              0x1CF2


#define PVR_MEM_PCI_BASENUM         2
#endif


static int __init BC_Example_ModInit(void)
{
#if defined(LDM_PLATFORM) || defined(LDM_PCI)
    struct device *psDev;
#endif

#if defined(LMA)
	struct pci_dev *psPCIDev;
	int error;
#endif

#if defined(LMA)
	psPCIDev = pci_get_device(VENDOR_ID_PVR, DEVICE_ID_PVR, NULL);
	if (psPCIDev == NULL)
	{

		psPCIDev = pci_get_device(VENDOR_ID_PVR, DEVICE_ID1_PVR, NULL);
	}

	if (psPCIDev == NULL)
	{
		printk(KERN_ERR DRVNAME ": BC_Example_ModInit:  pci_get_device failed\n");

		goto ExitError;
	}

	if ((error = pci_enable_device(psPCIDev)) != 0)
	{
		printk(KERN_ERR DRVNAME ": BC_Example_ModInit: pci_enable_device failed (%d)\n", error);
		goto ExitError;
	}
#endif

	AssignedMajorNumber = register_chrdev(0, DEVNAME, &bufferclass_example_fops);

	if (AssignedMajorNumber <= 0)
	{
		printk(KERN_ERR DRVNAME ": BC_Example_ModInit: unable to get major number\n");

		goto ExitDisable;
	}

#if defined(DEBUG)
	printk(KERN_ERR DRVNAME ": BC_Example_ModInit: major device %d\n", AssignedMajorNumber);
#endif

#if defined(LDM_PLATFORM) || defined(LDM_PCI)

	psPvrClass = class_create(THIS_MODULE, "bc_example");

	if (IS_ERR(psPvrClass))
	{
		printk(KERN_ERR DRVNAME ": BC_Example_ModInit: unable to create class (%ld)", PTR_ERR(psPvrClass));
		goto ExitUnregister;
	}

	psDev = device_create(psPvrClass, NULL, MKDEV(AssignedMajorNumber, 0),
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,26))
						  NULL,
#endif
						  DEVNAME);
	if (IS_ERR(psDev))
	{
		printk(KERN_ERR DRVNAME ": BC_Example_ModInit: unable to create device (%ld)", PTR_ERR(psDev));
		goto ExitDestroyClass;
	}
#endif

#if defined(LMA)

	g_ulMemBase =  pci_resource_start(psPCIDev, PVR_MEM_PCI_BASENUM) + PVR_BUFFERCLASS_MEMOFFSET;
#endif

	if(BC_Example_Init() != BCE_OK)
	{
		printk (KERN_ERR DRVNAME ": BC_Example_ModInit: can't init device\n");
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
ExitDisable:
#if defined(LMA)
	pci_disable_device(psPCIDev);
ExitError:
#endif
	return -EBUSY;
}

static void __exit BC_Example_ModCleanup(void)
{
#if defined(LDM_PLATFORM) || defined(LDM_PCI)
	device_destroy(psPvrClass, MKDEV(AssignedMajorNumber, 0));
	class_destroy(psPvrClass);
#endif

	unregister_chrdev(AssignedMajorNumber, DEVNAME);

	if(BC_Example_Deinit() != BCE_OK)
	{
		printk (KERN_ERR DRVNAME ": BC_Example_ModCleanup: can't deinit device\n");
	}

}


void *BCAllocKernelMem(unsigned long ulSize)
{
	return kmalloc(ulSize, GFP_KERNEL);
}

void BCFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}

#if defined(BC_DISCONTIG_BUFFERS)

#define RANGE_TO_PAGES(range) (((range) + (PAGE_SIZE - 1)) >> PAGE_SHIFT)
#define	VMALLOC_TO_PAGE_PHYS(vAddr) page_to_phys(vmalloc_to_page(vAddr))

BCE_ERROR BCAllocDiscontigMemory(unsigned long ulSize,
                              BCE_HANDLE unref__ *phMemHandle,
                              IMG_CPU_VIRTADDR *pLinAddr,
                              IMG_SYS_PHYADDR **ppPhysAddr)
{
	unsigned long ulPages = RANGE_TO_PAGES(ulSize);
	IMG_SYS_PHYADDR *pPhysAddr;
	unsigned long ulPage;
	IMG_CPU_VIRTADDR LinAddr;

	LinAddr = __vmalloc(ulSize, GFP_KERNEL | __GFP_HIGHMEM, pgprot_noncached(PAGE_KERNEL));
	if (!LinAddr)
	{
		return BCE_ERROR_OUT_OF_MEMORY;
	}

	pPhysAddr = kmalloc(ulPages * sizeof(IMG_SYS_PHYADDR), GFP_KERNEL);
	if (!pPhysAddr)
	{
		vfree(LinAddr);
		return BCE_ERROR_OUT_OF_MEMORY;
	}

	*pLinAddr = LinAddr;

	for (ulPage = 0; ulPage < ulPages; ulPage++)
	{
		pPhysAddr[ulPage].uiAddr = VMALLOC_TO_PAGE_PHYS(LinAddr);

		LinAddr += PAGE_SIZE;
	}

	*ppPhysAddr = pPhysAddr;

	return BCE_OK;
}

void BCFreeDiscontigMemory(unsigned long ulSize,
                         BCE_HANDLE unref__ hMemHandle,
                         IMG_CPU_VIRTADDR LinAddr,
                         IMG_SYS_PHYADDR *pPhysAddr)
{
	kfree(pPhysAddr);

	vfree(LinAddr);
}
#else

BCE_ERROR BCAllocContigMemory(unsigned long ulSize,
                              BCE_HANDLE unref__ *phMemHandle,
                              IMG_CPU_VIRTADDR *pLinAddr,
                              IMG_CPU_PHYADDR *pPhysAddr)
{
#if defined(LMA)
	void *pvLinAddr;


	if(g_ulMemCurrent + ulSize >= PVR_BUFFERCLASS_MEMSIZE)
	{
		return (BCE_ERROR_OUT_OF_MEMORY);
	}

	pvLinAddr = ioremap(g_ulMemBase + g_ulMemCurrent, ulSize);

	if(pvLinAddr)
	{
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
	BUG_ON(((unsigned long)pvLinAddr)  & ~PAGE_MASK);

	iError = set_memory_wc((unsigned long)pvLinAddr, iPages);
	if (iError != 0)
	{
		printk(KERN_ERR DRVNAME ": BCAllocContigMemory:  set_memory_wc failed (%d)\n", iError);
		return (BCE_ERROR_OUT_OF_MEMORY);
	}

	pPhysAddr->uiAddr = virt_to_phys(pvLinAddr);
	*pLinAddr = pvLinAddr;

	return (BCE_OK);
#else
	dma_addr_t dma;
	void *pvLinAddr;

	pvLinAddr = dma_alloc_coherent(NULL, ulSize, &dma, GFP_KERNEL);
	if (pvLinAddr == NULL)
	{
		return (BCE_ERROR_OUT_OF_MEMORY);
	}

	pPhysAddr->uiAddr = dma;
	*pLinAddr = pvLinAddr;

	return (BCE_OK);
#endif
#endif
}

void BCFreeContigMemory(unsigned long ulSize,
                        BCE_HANDLE unref__ hMemHandle,
                        IMG_CPU_VIRTADDR LinAddr,
                        IMG_CPU_PHYADDR PhysAddr)
{
#if defined(LMA)
	g_ulMemCurrent -= ulSize;
	iounmap(LinAddr);
#else
#if defined(BCE_USE_SET_MEMORY)
	unsigned long ulAlignedSize = PAGE_ALIGN(ulSize);
	int iError;
	int iPages = (int)(ulAlignedSize >> PAGE_SHIFT);

	iError = set_memory_wb((unsigned long)LinAddr, iPages);
	if (iError != 0)
	{
		printk(KERN_ERR DRVNAME ": BCFreeContigMemory:  set_memory_wb failed (%d)\n", iError);
	}
	kfree(LinAddr);
#else
	dma_free_coherent(NULL, ulSize, LinAddr, (dma_addr_t)PhysAddr.uiAddr);
#endif
#endif
}
#endif

IMG_SYS_PHYADDR CpuPAddrToSysPAddrBC(IMG_CPU_PHYADDR cpu_paddr)
{
	IMG_SYS_PHYADDR sys_paddr;


	sys_paddr.uiAddr = cpu_paddr.uiAddr;
	return sys_paddr;
}

IMG_CPU_PHYADDR SysPAddrToCpuPAddrBC(IMG_SYS_PHYADDR sys_paddr)
{

	IMG_CPU_PHYADDR cpu_paddr;

	cpu_paddr.uiAddr = sys_paddr.uiAddr;
	return cpu_paddr;
}

BCE_ERROR BCOpenPVRServices (BCE_HANDLE *phPVRServices)
{

	*phPVRServices = 0;
	return (BCE_OK);
}


BCE_ERROR BCClosePVRServices (BCE_HANDLE unref__ hPVRServices)
{

	return (BCE_OK);
}

BCE_ERROR BCGetLibFuncAddr (BCE_HANDLE unref__ hExtDrv, char *szFunctionName, PFN_BC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetBufferClassJTable", szFunctionName) != 0)
	{
		return (BCE_ERROR_INVALID_PARAMS);
	}


	*ppfnFuncTable = PVRGetBufferClassJTable;

	return (BCE_OK);
}


static int BC_Example_Bridge(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = -EFAULT;
	int command = _IOC_NR(cmd);
	BC_Example_ioctl_package sBridge;

	PVR_UNREFERENCED_PARAMETER(inode);

	if (copy_from_user(&sBridge, (void *)arg, sizeof(sBridge)) != 0)
	{
		return err;
	}

	switch(command)
	{
		case _IOC_NR(BC_Example_ioctl_fill_buffer):
		{
			if(FillBuffer(sBridge.inputparam) == -1)
			{
				return err;
			}
			break;
		}
		case _IOC_NR(BC_Example_ioctl_get_buffer_count):
		{
			if(GetBufferCount(&sBridge.outputparam) == -1)
			{
				return err;
			}
			break;
		}
	    case _IOC_NR(BC_Example_ioctl_reconfigure_buffer):
		{
			if(ReconfigureBuffer(&sBridge.outputparam) == -1)
			{
				return err;
			}
			break;
		}
		default:
			return err;
	}

	if (copy_to_user((void *)arg, &sBridge, sizeof(sBridge)) != 0)
	{
		return err;
	}

	return 0;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36))
static long BC_Example_Bridge_Unlocked(struct file *file, unsigned int cmd, unsigned long arg)
{
	int res;

	mutex_lock(&sBCExampleBridgeMutex);
	res = BC_Example_Bridge(NULL, file, cmd, arg);
	mutex_unlock(&sBCExampleBridgeMutex);

	return res;
}
#endif

module_init(BC_Example_ModInit);
module_exit(BC_Example_ModCleanup);

