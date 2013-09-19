/*
 *
 * Intel Management Engine Interface (Intel MEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/uuid.h>
#include <linux/compat.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/dma-mapping.h>

#include <linux/pm_runtime.h>

#include <linux/mei.h>
#include <linux/acpi.h>
#include <acpi/acpi_bus.h>


#include "mei_dev.h"
#include "hw-txe.h"
#include "mei-mm.h"

bool nopg;
module_param_named(nopg, nopg, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(nopg, "don't enable power gating (default = false)");


static char dmapool[32] = "4M";
module_param_string(dmapool, dmapool, 32, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dmapool, "dma pool size (default=4M)");

static DEFINE_PCI_DEVICE_TABLE(mei_txe_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0F18)},
	{0, }
};
MODULE_DEVICE_TABLE(pci, mei_txe_pci_tbl);

static acpi_status txei_walk_resource(struct acpi_resource *res, void *data)
{
	struct mei_device *dev = (struct mei_device *)data;
	struct mei_txe_hw *hw = to_txe_hw(dev);
	struct acpi_resource_fixed_memory32 *fixmem32;

	if (res->type != ACPI_RESOURCE_TYPE_FIXED_MEMORY32)
		return AE_OK;

	fixmem32 = &res->data.fixed_memory32;

	dev_dbg(&dev->pdev->dev, "TXE8086 MEMORY32 addr 0x%x len %d\n",
		fixmem32->address, fixmem32->address_length);

	if (!fixmem32->address || !fixmem32->address_length) {
		dev_err(&dev->pdev->dev, "TXE8086 MEMORY32 addr 0x%x len %d\n",
			fixmem32->address, fixmem32->address_length);
		return AE_NO_MEMORY;
	}

	hw->pool_paddr = fixmem32->address;
	hw->pool_size  = fixmem32->address_length;

	return AE_OK;
}

static void mei_release_dma_acpi(struct mei_txe_hw *hw)
{
	if (hw->pool_vaddr)
		iounmap(hw->pool_vaddr);

	hw->pool_vaddr = NULL;
	hw->pool_paddr = 0;
	hw->pool_size  = 0;
}

static int mei_reserver_dma_acpi(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	acpi_handle handle;
	acpi_status ret;

	handle = ACPI_HANDLE(&dev->pdev->dev);
	if (!handle) {
		dev_err(&dev->pdev->dev, "TXE8086 acpi NULL handle received\n");
		return -ENODEV;
	}

	dev_dbg(&dev->pdev->dev, "TXE8086 acpi handle received\n");
	ret = acpi_walk_resources(handle, METHOD_NAME__CRS,
				txei_walk_resource, dev);

	if (ACPI_FAILURE(ret)) {
		dev_err(&dev->pdev->dev, "TXE8086: acpi_walk_resources FAILURE\n");
		return -ENOMEM;
	}

	if (!hw->pool_size) {
		dev_err(&dev->pdev->dev, "TXE8086: acpi __CRS resource not found\n");
		return -ENOMEM;
	}

	dev_dbg(&dev->pdev->dev, "DMA Memory reserved memory usage: size=%zd\n",
		hw->pool_size);

	/* limit the pool_size to SATT_RANGE_MAX */
	hw->pool_size = min_t(size_t, hw->pool_size, SATT_RANGE_MAX);

	hw->pool_vaddr = ioremap(hw->pool_paddr, hw->pool_size);
	/* FIXME: is this fatal ? */
	if (!hw->pool_vaddr)
		dev_err(&dev->pdev->dev, "TXE8086: acpi __CRS cannot remap\n");

	hw->pool_release = mei_release_dma_acpi;

	return 0;
}


static void mei_free_dma(struct  mei_txe_hw *hw)
{
	struct mei_device *dev = hw_txe_to_mei(hw);
	if (hw->pool_vaddr)
		dma_free_coherent(&dev->pdev->dev,
			hw->pool_size, hw->pool_vaddr, hw->pool_paddr);
	hw->pool_vaddr = NULL;
	hw->pool_paddr = 0;
	hw->pool_size  = 0;
}

static int mei_alloc_dma(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	hw->pool_size = memparse(dmapool, NULL);
	dev_dbg(&dev->pdev->dev, "MEIMM: dma size %zd\n", hw->pool_size);

	if (hw->pool_size == 0)
		return 0;

	/*  Limmit pools size to satt max range */
	hw->pool_size = min_t(size_t, hw->pool_size, SATT_RANGE_MAX);

	hw->pool_vaddr = dma_alloc_coherent(&dev->pdev->dev, hw->pool_size,
		&hw->pool_paddr, GFP_KERNEL);

	dev_dbg(&dev->pdev->dev, "DMA Memory Allocated vaddr=%p, paddr=%lu, size=%zd\n",
			hw->pool_vaddr,
			(unsigned long)hw->pool_paddr, hw->pool_size);

	if (!hw->pool_vaddr) {
		dev_err(&dev->pdev->dev, "DMA Memory Allocation failed for size %zd\n",
			hw->pool_size);
		return -ENOMEM;
	}

	if (hw->pool_paddr & ~DMA_BIT_MASK(36)) {
		dev_err(&dev->pdev->dev, "Phys Address is beyond DMA_MASK(32) 0x%0lX\n",
			(unsigned long)hw->pool_paddr);
	}

	hw->pool_release = mei_free_dma;

	return 0;
}


static void mei_txe_pci_iounmap(struct pci_dev *pdev, struct mei_txe_hw *hw)
{
	int i;
	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		if (hw->mem_addr[i]) {
			pci_iounmap(pdev, hw->mem_addr[i]);
			hw->mem_addr[i] = NULL;
		}
	}
}
/**
 * mei_probe - Device Initialization Routine
 *
 * @pdev: PCI device structure
 * @ent: entry in mei_txe_pci_tbl
 *
 * returns 0 on success, <0 on failure.
 */
static int mei_txe_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct mei_device *dev;
	struct mei_txe_hw *hw;
	int err;
	int i;

	/* enable pci dev */
	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "failed to enable pci device.\n");
		goto end;
	}
	/* set PCI host mastering  */
	pci_set_master(pdev);
	/* pci request regions for mei driver */
	err = pci_request_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "failed to get pci regions.\n");
		goto disable_device;
	}

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(36));
	if (err) {
		err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev, "No suitable DMA available.\n");
			goto release_regions;
		}
	}

	/* allocates and initializes the mei dev structure */
	dev = mei_txe_dev_init(pdev);
	if (!dev) {
		err = -ENOMEM;
		goto release_regions;
	}
	hw = to_txe_hw(dev);


	err = mei_reserver_dma_acpi(dev);
	if (err)
		err = mei_alloc_dma(dev);
	if (err)
		goto free_device;

	/* mapping  IO device memory */
	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		hw->mem_addr[i] = pci_iomap(pdev, i, 0);
		if (!hw->mem_addr[i]) {
			dev_err(&pdev->dev, "mapping I/O device memory failure.\n");
			err = -ENOMEM;
			goto free_device;
		}
	}


	pci_enable_msi(pdev);

	/* clear spurious interrupts */
	mei_clear_interrupts(dev);

	/* request and enable interrupt  */
	if (pci_dev_msi_enabled(pdev))
		err = request_threaded_irq(pdev->irq,
			NULL,
			mei_txe_irq_thread_handler,
			IRQF_ONESHOT, KBUILD_MODNAME, dev);
	else
		err = request_threaded_irq(pdev->irq,
			mei_txe_irq_quick_handler,
			mei_txe_irq_thread_handler,
			IRQF_SHARED, KBUILD_MODNAME, dev);
	if (err) {
		dev_err(&pdev->dev, "mei: request_threaded_irq failure. irq = %d\n",
			pdev->irq);
		goto free_device;
	}

	if (mei_start(dev)) {
		dev_err(&pdev->dev, "init hw failure.\n");
		err = -ENODEV;
		goto release_irq;
	}

	err = mei_txe_setup_satt2(dev,
		dma_to_phys(&dev->pdev->dev, hw->pool_paddr), hw->pool_size);
	if (err)
		goto release_irq;


	err = mei_register(dev);
	if (err)
		goto release_irq;

	pci_set_drvdata(pdev, dev);

	hw->mdev = mei_mm_init(&dev->pdev->dev,
		hw->pool_vaddr, hw->pool_paddr, hw->pool_size);

	if (IS_ERR_OR_NULL(hw->mdev))
		goto deregister_mei;

	/* This is w/a for BIOS not anbling waking up the device */
	device_set_run_wake(&pdev->dev, true);

	pm_runtime_set_autosuspend_delay(&pdev->dev, MEI_TXI_RPM_TIMEOUT);
	pm_runtime_use_autosuspend(&pdev->dev);

	pm_runtime_mark_last_busy(&pdev->dev);

	if (pci_dev_run_wake(pdev))
		pm_runtime_put_noidle(&pdev->dev);

	if (!nopg)
		pm_runtime_allow(&pdev->dev);

	return 0;

deregister_mei:
	mei_deregister(dev);
release_irq:
	/* disable interrupts */
	mei_disable_interrupts(dev);

	flush_workqueue(dev->wq);

	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);

free_device:
	if (hw->pool_release)
		hw->pool_release(hw);

	mei_txe_pci_iounmap(pdev, hw);

	kfree(dev);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
end:
	dev_err(&pdev->dev, "initialization failed.\n");
	return err;
}

/**
 * mei_remove - Device Removal Routine
 *
 * @pdev: PCI device structure
 *
 * mei_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.
 */
static void mei_txe_remove(struct pci_dev *pdev)
{
	struct mei_device *dev;
	struct mei_txe_hw *hw;

	dev = pci_get_drvdata(pdev);
	if (!dev) {
		dev_err(&pdev->dev, "mei: dev =NULL\n");
		return;
	}

	if (pci_dev_run_wake(pdev))
		pm_runtime_get_noresume(&pdev->dev);

	hw = to_txe_hw(dev);

	mei_stop(dev);

	/* disable interrupts */
	mei_disable_interrupts(dev);
	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);

	mei_mm_deinit(hw->mdev);

	if (hw->pool_release)
		hw->pool_release(hw);

	pci_set_drvdata(pdev, NULL);

	mei_txe_pci_iounmap(pdev, hw);

	mei_deregister(dev);

	kfree(dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
}


#ifdef CONFIG_PM
static int mei_txe_pci_suspend(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct mei_device *dev = pci_get_drvdata(pdev);

	if (!dev)
		return -ENODEV;

	dev_dbg(&pdev->dev, "suspend\n");

	mei_stop(dev);

	mei_disable_interrupts(dev);

	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);

	return 0;
}

static int mei_txe_pci_resume(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct mei_device *dev;
	int err;

	dev = pci_get_drvdata(pdev);
	if (!dev)
		return -ENODEV;

	pci_enable_msi(pdev);

	mei_clear_interrupts(dev);

	/* request and enable interrupt */
	if (pci_dev_msi_enabled(pdev))
		err = request_threaded_irq(pdev->irq,
			NULL,
			mei_txe_irq_thread_handler,
			IRQF_ONESHOT, KBUILD_MODNAME, dev);
	else
		err = request_threaded_irq(pdev->irq,
			mei_txe_irq_quick_handler,
			mei_txe_irq_thread_handler,
			IRQF_SHARED, KBUILD_MODNAME, dev);
	if (err) {
		dev_err(&pdev->dev, "request_threaded_irq failed: irq = %d.\n",
				pdev->irq);
		return err;
	}

	mutex_lock(&dev->device_lock);

	dev->dev_state = MEI_DEV_POWER_UP;
	mei_reset(dev, true);
	mutex_unlock(&dev->device_lock);

	return err;
}

#ifdef CONFIG_PM_RUNTIME
static int mei_txe_pm_runtime_idle(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct mei_device *dev;

	dev_dbg(&pdev->dev, "rpm: txe: runtime_idle\n");

	dev = pci_get_drvdata(pdev);
	if (!dev)
		return -ENODEV;
	if (mei_write_is_idle(dev))
		pm_schedule_suspend(device, MEI_TXI_RPM_TIMEOUT * 2);

	return -EBUSY;
}
static int mei_txe_pm_runtime_suspend(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct mei_device *dev;
	int ret;

	dev_dbg(&pdev->dev, "rpm: txe: runtime suspend\n");

	dev = pci_get_drvdata(pdev);
	if (!dev)
		return -ENODEV;

	mutex_lock(&dev->device_lock);

	if (mei_write_is_idle(dev))
		/* FIXME: need to check API what error is epxected */
		ret = mei_txe_aliveness_set_sync(dev, 0);
	else
		ret = -EAGAIN;

	dev_dbg(&pdev->dev, "rpm: txe: runtime suspend ret=%d\n", ret);

	mutex_unlock(&dev->device_lock);
	return ret;
}

static int mei_txe_pm_runtime_resume(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct mei_device *dev;
	int ret;

	dev_dbg(&pdev->dev, "rpm: txe: runtime resume\n");

	dev = pci_get_drvdata(pdev);
	if (!dev)
		return -ENODEV;

	mutex_lock(&dev->device_lock);

	ret = mei_txe_aliveness_set_sync(dev, 1);

	mutex_unlock(&dev->device_lock);

	dev_dbg(&pdev->dev, "rpm: txe: runtime resume ret = %d\n", ret);

	return ret;
}
#endif /* CONFIG_PM_RUNTIME */

static const struct dev_pm_ops mei_txe_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mei_txe_pci_suspend,
				mei_txe_pci_resume)
	SET_RUNTIME_PM_OPS(
		mei_txe_pm_runtime_suspend,
		mei_txe_pm_runtime_resume,
		mei_txe_pm_runtime_idle)
};

#define MEI_TXE_PM_OPS	(&mei_txe_pm_ops)
#else
#define MEI_TXE_PM_OPS	NULL
#endif /* CONFIG_PM */
/*
 *  PCI driver structure
 */
static struct pci_driver mei_txe_driver = {
	.name = KBUILD_MODNAME,
	.id_table = mei_txe_pci_tbl,
	.probe = mei_txe_probe,
	.remove = mei_txe_remove,
	.shutdown = mei_txe_remove,
	.driver.pm = MEI_TXE_PM_OPS,
};

module_pci_driver(mei_txe_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) Trusted Execution Engine Interface");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0.1000");
