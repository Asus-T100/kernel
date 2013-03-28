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

#include <linux/mei.h>

#include "mei_dev.h"
#include "hw-txe.h"

#define MEI_READ_TIMEOUT 45
#define MEI_DRIVER_NAME	"mei"
#define MEI_DEV_NAME "mei"

/*
 *  mei driver strings
 */
static bool nomsi;
module_param_named(nomsi, nomsi, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(nomsi, "don't use msi (default = false)");

bool nopg = true;
module_param_named(nopg, nopg, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(nopg, "don't use power gating (default = true)");


/* Currently this driver works as long as there is only a single AMT device. */
static struct pci_dev *mei_device;

static DEFINE_PCI_DEVICE_TABLE(mei_txe_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0F18)},
	{0, }
};
MODULE_DEVICE_TABLE(pci, mei_txe_pci_tbl);

static DEFINE_MUTEX(mei_mutex);


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

	mutex_lock(&mei_mutex);
	if (mei_device) {
		err = -EEXIST;
		goto end;
	}
	/* enable pci dev */
	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "mei: Failed to enable pci device.\n");
		goto end;
	}
	/* set PCI host mastering  */
	pci_set_master(pdev);
	/* pci request regions for mei driver */
	err = pci_request_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "mei: Failed to get pci regions.\n");
		goto disable_device;
	}

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(64));
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


	/* mapping  IO device memory */
	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		dev_dbg(&pdev->dev, "mei: mapping memmory of bar%d\n", i);
		if (pci_resource_len(pdev, i) == 0) {
			dev_err(&pdev->dev, "mei: Non-positive memory length was returned for one or more bars of the PCI device.\n");
			err = -ENOMEM;
			goto free_device;
		}
		hw->mem_addr[i] = pci_iomap(pdev, i, 0);
		dev_dbg(&pdev->dev, "mei: mem_addr:%p flags %ld\n",
			hw->mem_addr[i], pci_resource_flags(pdev, i));
		if (!hw->mem_addr[i]) {
			dev_err(&pdev->dev, "mei: mapping I/O device memory failure.\n");
			err = -ENOMEM;
			goto free_device;
		}
	}


	if (!nomsi && !pci_enable_msi(pdev))
		dev_err(&pdev->dev, "mei: MSI was enabled\n");
	else
		dev_err(&pdev->dev, "mei: MSI is not enabled\n");

	/* clear spurious interrupts */
	mei_clear_interrupts(dev);

	/* request and enable interrupt   */
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
		dev_err(&pdev->dev, "mei: Init hw failure.\n");
		err = -ENODEV;
		goto release_irq;
	}

	err = mei_register(dev);
	if (err)
		goto release_irq;

	mei_device = pdev;
	pci_set_drvdata(pdev, dev);

	mutex_unlock(&mei_mutex);

	return 0;

	mei_deregister(dev);
release_irq:
	/* disable interrupts */

	mei_disable_interrupts(dev);
	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);

free_device:
	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		if (hw->mem_addr[i]) {
			pci_iounmap(pdev, hw->mem_addr[i]);
			hw->mem_addr[i] = NULL;
		}
	}
	kfree(dev);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
end:
	mutex_unlock(&mei_mutex);
	dev_err(&pdev->dev, "mei: Driver initialization failed.\n");
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
	int i;

	if (mei_device != pdev) {
		dev_err(&pdev->dev, "mei: mei_device != pdev\n");
		return;
	}

	dev = pci_get_drvdata(pdev);
	if (!dev) {
		dev_err(&pdev->dev, "mei: dev =NULL\n");
		return;
	}

	hw = to_txe_hw(dev);

	mutex_lock(&dev->device_lock);

	/* disable interrupts */
	mei_disable_interrupts(dev);
	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);

	for (i = SEC_BAR; i < NUM_OF_MEM_BARS; i++) {
		if (hw->mem_addr[i])
			pci_iounmap(pdev, hw->mem_addr[i]);
	}

	mei_device = NULL;

	mutex_unlock(&dev->device_lock);

	flush_scheduled_work();

	pci_set_drvdata(pdev, NULL);

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
	mutex_lock(&dev->device_lock);
	/* Set new mei state */

	if (dev->dev_state == MEI_DEV_ENABLED) {
		dev->dev_state = MEI_DEV_POWER_DOWN;
		mei_reset(dev, 0);
	}
	mutex_unlock(&dev->device_lock);

	free_irq(pdev->irq, dev);

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

	/* request and enable interrupt   */
	err = request_threaded_irq(pdev->irq,
			mei_txe_irq_quick_handler,
			mei_txe_irq_thread_handler,
			IRQF_SHARED, KBUILD_MODNAME, dev);
	if (err) {
		dev_err(&pdev->dev, "mei: Request_irq failure. irq = %d\n",
				pdev->irq);
		return err;
	}

	mutex_lock(&dev->device_lock);

	dev->dev_state = MEI_DEV_POWER_UP;
	mei_reset(dev, true);
	mutex_unlock(&dev->device_lock);

	return err;
}
static SIMPLE_DEV_PM_OPS(mei_txe_pm_ops,
		mei_txe_pci_suspend, mei_txe_pci_resume);
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
