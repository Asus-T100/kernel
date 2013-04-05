/*
 *
 * Intel Management Engine Interface (Intel TXEI) Linux driver
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

#define DEBUG
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

#include "txei.h"

#include "txei_dev.h"
#include "hw-txe.h"
#include "txei_mm.h"

#define TXEI_READ_TIMEOUT 45
#define TXEI_DRIVER_NAME	"txei"
#define TXEI_DEV_NAME "txei"

/*
 *  txei driver strings
 */
static const char txei_driver_string[] =
	"Intel(R) IPC Over Host Embedded Controller";

static bool nomsi;
module_param_named(nomsi, nomsi, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(nomsi, "don't use msi (default = false)");

bool nopg = true;
module_param_named(nopg, nopg, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(nopg, "don't use power gating (default = true)");


static char dmapool[32] = "4M";
module_param_string(dmapool, dmapool, 32, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(dmapool, "dma pool size (default=4M)");

/* Currently this driver works as long as there is only a single AMT device. */
static struct pci_dev *txei_device;

static DEFINE_PCI_DEVICE_TABLE(txei_txe_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_XILINX, VLV_IPC_FPGA_DEV_ID)},/* FPGA */
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, VLV_IPC_DEV_ID)},      /* X0 */
	{0, }
};
MODULE_DEVICE_TABLE(pci, txei_txe_pci_tbl);

static DEFINE_MUTEX(txei_mutex);


static int txei_alloc_dma(struct txei_device *dev)
{
	/* FXIME: add check for checking upper and lower limmits */
	dev->pool_size = memparse(dmapool, NULL);

	dev->pool_vaddr = dma_alloc_coherent(&dev->pdev->dev, dev->pool_size,
		&dev->pool_paddr, GFP_KERNEL);

	txei_dbg(dev, "DMA Memory Allocated vaddr=%p, paddr=%lu, size=%zd\n",
			dev->pool_vaddr,
			(unsigned long)dev->pool_paddr, dev->pool_size);

	if (dev->pool_paddr & ~DMA_BIT_MASK(32)) {
		txei_err(dev, "Phys Address is beyond DMA_MASK(32) 0x%0lX\n",
			(unsigned long)dev->pool_paddr);
	}
	return dev->pool_vaddr ? 0 : -ENOMEM;
}


static void txei_free_dma(struct txei_device *dev)
{
	if (dev->pool_vaddr)
		dma_free_coherent(&dev->pdev->dev, dev->pool_size,
			dev->pool_vaddr, dev->pool_paddr);
	dev->pool_vaddr = NULL;
}

/**
 * txei_probe - Device Initialization Routine
 *
 * @pdev: PCI device structure
 * @ent: entry in kcs_pci_tbl
 *
 * returns 0 on success, <0 on failure.
 */
static int txei_probe(struct pci_dev *pdev,
		const struct pci_device_id *ent)
{
	struct txei_device *dev;
	unsigned long irqflags;
	int err;
	int i;

	dev_info(&pdev->dev, "running in %s mode\n",
		ent->device == VLV_IPC_FPGA_DEV_ID ? "FPGA" : "SI");

	mutex_lock(&txei_mutex);
	if (txei_device) {
		err = -EEXIST;
		goto end;
	}
	/* enable pci dev */
	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "txei: Failed to enable pci device.\n");
		goto end;
	}
	/* set PCI host mastering  */
	pci_set_master(pdev);
	/* pci request regions for txei driver */
	err = pci_request_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "txei: Failed to get pci regions.\n");
		goto disable_device;
	}

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (err) {
		err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
		if (err) {
			dev_err(&pdev->dev, "No suitable DMA available.\n");
			goto release_regions;
		}
	}

	/* allocates and initializes the txei dev structure */
	dev = txei_device_init(pdev);
	if (!dev) {
		err = -ENOMEM;
		goto release_regions;
	}


	err = txei_alloc_dma(dev);
	if (err)
		goto free_device;

	/* mapping  IO device memory */
	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		dev_dbg(&pdev->dev, "txei: mapping memmory of bar%d\n", i);
		if (pci_resource_len(pdev, i) == 0) {
			dev_err(&pdev->dev, "txei: Non-positive memory length was returned for one or more bars of the PCI device.\n");
			err = -ENOMEM;
			goto free_device;
		}
		dev->mem_addr[i] = pci_iomap(pdev, i, 0);
		dev_dbg(&pdev->dev, "txei: mem_addr:%p flags %ld\n",
			dev->mem_addr[i], pci_resource_flags(pdev, i));
		if (!dev->mem_addr[i]) {
			dev_err(&pdev->dev, "txei: mapping I/O device memory failure.\n");
			err = -ENOMEM;
			goto free_device;
		}
	}


#ifdef POLLING_MODE
	pr_info("txei: running IPC driver in Polling Mode");
	dev->stop_polling_thread = false;
	dev->polling_thread = kthread_run(polling_thread, dev, "ktxeirqd/fpga");
	if (IS_ERR(dev->polling_thread)) {
		err = PTR_ERR(dev->polling_thread);
		dev_err(&pdev->dev, "txei: IPC-Polling-Thread: unable to create kernel thread: %d\n",
			err);
		goto free_device;
	}

#else
	if (!nomsi && !pci_enable_msi(pdev)) {
		irqflags = IRQF_ONESHOT;
		dev_err(&pdev->dev, "txei: MSI was enabled\n");
	} else {
		irqflags = IRQF_SHARED;
		dev_err(&pdev->dev, "txei: MSI is not enabled\n");
	}

	/* clear spurious interrupts */
	ipc_interrupts_clear(dev);

	/* request and enable interrupt   */
	err = request_threaded_irq(pdev->irq,
			txei_interrupt_quick_handler,
			txei_interrupt_thread_handler,
			IRQF_SHARED, TXEI_DRIVER_NAME, dev);
	if (err) {
		dev_err(&pdev->dev, "txei: request_threaded_irq failure. irq = %d\n",
			pdev->irq);
		goto free_device;
	}
#endif /* POLLING_MODE */

#ifdef VLV_X0
	if (!nomsi) {
		u16 msid;
		pci_write_config_dword(pdev, 0xA4, 0xfee00000);
		pci_read_config_word(pdev, 0xA8, &msid);
		msid &= 0x00FF;
		msid |= BIT(14);
		pci_write_config_word(pdev, 0xA8, msid);
	}
#endif /* VLV_X0 */

	if (txei_hw_init(dev)) {
		dev_err(&pdev->dev, "txei: Init hw failure.\n");
		err = -ENODEV;
		goto release_irq;
	}

	err = txei_register(&pdev->dev);
	if (err)
		goto release_irq;

	txei_device = pdev;
	pci_set_drvdata(pdev, dev);

	txei_setup_satt2(dev, dev->pool_paddr, dev->pool_size);

	dev->mdev = txei_mm_init(&dev->pdev->dev,
		dev->pool_vaddr, dev->pool_paddr, dev->pool_size);

	if (IS_ERR_OR_NULL(dev->mdev))
		goto deregister_txei;

	mutex_unlock(&txei_mutex);

	if (txei_dbgfs_register(dev, "txei"))
		txei_err(dev, "debugfs registration failed\n");

	pr_debug("txei: Driver initialization successful.\n");
	return 0;

deregister_txei:
	txei_deregister();
release_irq:
#ifdef POLLING_MODE
	dev->stop_polling_thread = true;
	kthread_stop(dev->polling_thread);
#else
	/* disable interrupts */
	ipc_host_interrupts_disable(dev);
	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);
#endif /* POLLING_MODE */

free_device:
	txei_free_dma(dev);
	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		if (dev->mem_addr[i]) {
			pci_iounmap(pdev, dev->mem_addr[i]);
			dev->mem_addr[i] = NULL;
		}
	}
	if (dev != NULL)
		kfree(dev);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
end:
	mutex_unlock(&txei_mutex);
	dev_err(&pdev->dev, "txei: Driver initialization failed.\n");
	return err;
}

/**
 * txei_remove - Device Removal Routine
 *
 * @pdev: PCI device structure
 *
 * txei_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.
 */
static void txei_remove(struct pci_dev *pdev)
{
	struct txei_device *dev;
	int i;

	if (txei_device != pdev) {
		dev_err(&pdev->dev, "txei: txei_device != pdev\n");
		return;
	}

	dev = pci_get_drvdata(pdev);
	if (!dev) {
		dev_err(&pdev->dev, "txei: dev =NULL\n");
		return;
	}

	cancel_delayed_work_sync(&dev->aliveness_timer);

	mutex_lock(&dev->device_lock);
#ifdef POLLING_MODE
	/* Stop the polling thread */
	dev->stop_polling_thread = true;
	mutex_unlock(&dev->device_lock);

	kthread_stop(dev->polling_thread);
	pr_debug("txei: waiting for polling thread to finish\n");
	wait_event_interruptible(dev->wait_polling_thread_terminated,
		!dev->stop_polling_thread);
	mutex_lock(&dev->device_lock);
#else /* POLLING_MODE */

	/* disable interrupts */
	ipc_host_interrupts_disable(dev);
	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);

#endif /* POLLING_MODE */

	pr_debug("txei: Unmapping memmory regions\n");
	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		if (dev->mem_addr[i])
			pci_iounmap(pdev, dev->mem_addr[i]);
	}

	txei_mm_deinit(dev->mdev);

	txei_device = NULL;

	mutex_unlock(&dev->device_lock);

	flush_scheduled_work();

	txei_free_dma(dev);

	txei_dbgfs_unregister(dev);

	pci_set_drvdata(pdev, NULL);

	if (dev != NULL)
		kfree(dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

	txei_deregister();
}


#ifdef CONFIG_PM
static int txei_pci_suspend(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct txei_device *dev = pci_get_drvdata(pdev);

	if (!dev)
		return -ENODEV;
	mutex_lock(&dev->device_lock);
	/* Set new txei state */
	if (dev->dev_state == TXEI_DEV_ENABLED ||
	    dev->dev_state == TXEI_DEV_RECOVERING_FROM_RESET) {
		dev->dev_state = TXEI_DEV_POWER_DOWN;
		txei_reset(dev, false);
	}
	mutex_unlock(&dev->device_lock);

	free_irq(pdev->irq, dev);

	return 0;
}

static int txei_pci_resume(struct device *device)
{
	struct pci_dev *pdev = to_pci_dev(device);
	struct txei_device *dev;
	int err;

	dev = pci_get_drvdata(pdev);
	if (!dev)
		return -ENODEV;

	/* request and enable interrupt   */
	err = request_threaded_irq(pdev->irq,
			txei_interrupt_quick_handler,
			txei_interrupt_thread_handler,
			IRQF_SHARED, TXEI_DRIVER_NAME, dev);
	if (err) {
		dev_err(&pdev->dev, "txei: Request_irq failure. irq = %d\n",
				pdev->irq);
		return err;
	}

	mutex_lock(&dev->device_lock);
	dev->dev_state = TXEI_DEV_POWER_UP;
	txei_reset(dev, true);
	mutex_unlock(&dev->device_lock);

	return err;
}
static SIMPLE_DEV_PM_OPS(txei_pm_ops, txei_pci_suspend, txei_pci_resume);
#define TXEI_PM_OPS	(&txei_pm_ops)
#else
#define TXEI_PM_OPS	NULL
#endif /* CONFIG_PM */
/*
 *  PCI driver structure
 */
static struct pci_driver txei_driver = {
	.name = TXEI_DRIVER_NAME,
	.id_table = txei_txe_pci_tbl,
	.probe = txei_probe,
	.remove = txei_remove,
	.shutdown = txei_remove,
	.driver.pm = TXEI_PM_OPS,
};


/**
 * txei_registration_cdev - sets up the cdev structure for txei device.
 *
 * @dev: char device struct
 * @hminor: minor number for registration char device
 * @fops: file operations structure
 *
 * returns 0 on success, <0 on failure.
 */
module_pci_driver(txei_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) Management Engine Interface Over IPC");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.0.1026");
