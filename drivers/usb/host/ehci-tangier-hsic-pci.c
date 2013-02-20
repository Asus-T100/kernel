/*
 * Intel MID Platform Tangier EHCI/HSIC Controller PCI Bus Glue.
 *
 * Copyright (c) 2008 - 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License 2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/usb/hcd.h>
#include <linux/wakelock.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/usb/ehci-tangier-hsic-pci.h>

static struct pci_dev	*pci_dev;

static int ehci_hsic_start_host(struct pci_dev  *pdev);
static int ehci_hsic_stop_host(struct pci_dev *pdev);
static int hsic_enable = 1;
static int hsic_aux_irq_enable;
/* Workaround as the Modem will request 2 IRQs */
static int first_interrupt;
static int hsic_enable_created;
static int hsic_mutex_init;
static struct hsic_tangier_priv hsic;
static int hsic_stopped;

/* Workaround for OSPM, set PMCMD to ask SCU
 * power gate EHCI controller and DPHY
 */
static void hsic_enter_exit_d3(int enter_exit)
{
	u32 tmp, count = 0;
	void __iomem  *pm_base;

	/* Reqeust IOMEM for PM */
	pm_base = ioremap_nocache(PM_BASE, PM_REGISTER_LENGHT);
	if (pm_base == NULL) {
		dev_err(&pci_dev->dev,
			"Mapping PMU failed\n");
		return;
	}

	if (enter_exit) {
		dev_dbg(&pci_dev->dev,
			"HSIC Enter D3\n");
		tmp = readl(pm_base + PM_SSC0);
		tmp |= PM_SSC0_HSIC_D3_MODE;
		writel(tmp, pm_base + PM_SSC0);

		writel(0x201 | (0x1 << 21), pm_base + PM_CMD);
	} else {
		dev_dbg(&pci_dev->dev,
			"HSIC Exit D3\n");
		tmp = readl(pm_base + PM_SSC0);
		tmp &= ~PM_SSC0_HSIC_D3_MODE;
		writel(tmp, pm_base + PM_SSC0);

		writel(0x201 | (0x1 << 21), pm_base + PM_CMD);
	}

	while (1) {
		tmp = readl(pm_base);
		if (tmp & (PM_STS_DONE)) {
			mdelay(1);
			if (count >= 10) {
				count = 0;
				dev_err(&pci_dev->dev,
					"Waiting for enter/exit D3 completed\n");
			}
		} else
			break;
		count++;
	}

	/* Check if HSIC and DPHY both enter D3*/
	tmp = readl(pm_base + PMU_HW_PEN0);
	tmp &= HSIC_DPHY_D3_STATE_MASK;
	if ((tmp != HSIC_DPHY_D3_STATE_MASK) &&
		enter_exit)
		dev_err(&pci_dev->dev,
			"HSIC and DPHY enter D3 failed!\n");

	iounmap(pm_base);
}

/* Init HSIC AUX GPIO as outpur PD */
static int hsic_aux_gpio_init(void)
{
	int		retval = 0;

	if (gpio_is_valid(HSIC_AUX_N)) {
		retval = gpio_request(HSIC_AUX_N, "HSIC_AUX_N");
		if (retval < 0) {
			dev_err(&pci_dev->dev,
				"Request GPIO %d with error %d\n",
				HSIC_AUX_N, retval);
			retval = -ENODEV;
			goto err;
		}
	} else {
		retval = -ENODEV;
		goto err;
	}

	lnw_gpio_set_alt(HSIC_AUX_N, 0);
	gpio_direction_output(HSIC_AUX_N, 0);

err:
	return retval;
}

/* HSIC AUX GPIO irq handler */
static irqreturn_t hsic_aux_gpio_irq(int irq, void *data)
{
	struct device *dev = data;

	dev_dbg(dev,
		"%s hsic aux gpio request irq: %d\n",
		__func__, irq);

	if (hsic_aux_irq_enable == 0) {
		dev_err(dev, "irq is disabled\n");
		return IRQ_HANDLED;
	}

	/* This is modem firmware workaround.
	 * There will be one more interrupts send from modem,
	 * usb driver should just care the first one.
	 */
	if (first_interrupt) {
		dev_dbg(dev,
			"ignore second HSIC interrupt\n");
		return IRQ_HANDLED;
	}

	first_interrupt = 1;

	if (delayed_work_pending(&hsic.hsic_aux)) {
		dev_dbg(dev,
		"ignore the aux irq as delay work busy\n");
		return IRQ_HANDLED;
	}

	schedule_delayed_work(&hsic.hsic_aux, 0);

	return IRQ_HANDLED;
}

static int hsic_aux_irq_init(void)
{	int retval;

	hsic_aux_irq_enable = 1;
	first_interrupt = 0;
	gpio_direction_input(HSIC_AUX_N);
	retval = request_irq(gpio_to_irq(HSIC_AUX_N),
			hsic_aux_gpio_irq,
			IRQF_SHARED | IRQF_TRIGGER_FALLING,
			"hsic_disconnect_request", &pci_dev->dev);
	if (retval) {
		dev_err(&pci_dev->dev,
			"unable to request irq %i, err: %d\n",
			gpio_to_irq(HSIC_AUX_N), retval);
	}

	return retval;
}

static void free_aux_irq(void)
{
	if (hsic_aux_irq_enable) {
		hsic_aux_irq_enable = 0;
		synchronize_irq(gpio_to_irq(HSIC_AUX_N));
		lnw_gpio_set_alt(HSIC_AUX_N, 0);
		gpio_direction_output(HSIC_AUX_N, 0);
		free_irq(gpio_to_irq(HSIC_AUX_N), &pci_dev->dev);
	}
}

/* the root hub will call this callback when device added/removed */
static void hsic_notify(struct usb_device *udev, unsigned action)
{
	int retval;

	/* Ignore root hub add/remove event */
	if (!udev->parent) {
		pr_debug("%s Ignore root hub otg_notify\n", __func__);
		return;
	}

	/* Ignore USB devices on external hub */
	if (udev->parent && udev->parent->parent)
		return;

	/* Only valid for hsic port1 */
	if (udev->portnum == 2) {
		pr_debug("%s ignore hsic port2\n", __func__);
		return;
	}

	switch (action) {
	case USB_DEVICE_ADD:
		pr_debug("Notify HSIC add device\n");
		retval = hsic_aux_irq_init();
		if (retval)
			dev_err(&pci_dev->dev,
				"unable to request IRQ\n");
		break;
	case USB_DEVICE_REMOVE:
		pr_debug("Notify HSIC delete device\n");
		free_aux_irq();
		break;
	default:
		pr_debug("Notify action not supported\n");
		return ;
	}
	return;
}

static void hsic_aux_work(struct work_struct *work)
{
	dev_dbg(&pci_dev->dev, "------>\n");
	mutex_lock(&hsic.hsic_mutex);
	if (hsic_stopped == 0)
		ehci_hsic_stop_host(pci_dev);
	hsic_enter_exit_d3(1);
	usleep_range(5000, 6000);
	hsic_enter_exit_d3(0);
	ehci_hsic_start_host(pci_dev);
	first_interrupt = 0;
	mutex_unlock(&hsic.hsic_mutex);
}

static ssize_t hsic_port_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic_enable);
}

static ssize_t hsic_port_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;

	if (size > sizeof(int))
		return -EINVAL;

	if (delayed_work_pending(&hsic.hsic_aux)) {
		dev_dbg(&dev, "delay work busy\n");
		return -EBUSY;
	}

	if (sscanf(buf, "%d", &hsic_enable) == 1) {
		if (hsic_enable) {
			dev_dbg(dev, "enable hsic\n");
			mutex_lock(&hsic.hsic_mutex);
			/* add this due to hcd release
				 doesn't set hcd to NULL */
			if (hsic_stopped == 0)
				ehci_hsic_stop_host(pci_dev);
			hsic_enter_exit_d3(1);
			usleep_range(5000, 6000);
			hsic_enter_exit_d3(0);
			ehci_hsic_start_host(pci_dev);
			mutex_unlock(&hsic.hsic_mutex);

			return size;
		} else {
			dev_dbg(dev, "disable hsic\n");
			mutex_lock(&hsic.hsic_mutex);
			/* add this due to hcd release
				 doesn't set hcd to NULL */
			if (hsic_stopped == 0)
				ehci_hsic_stop_host(pci_dev);
			mutex_unlock(&hsic.hsic_mutex);

			return size;
		}
	}

	return -1;
}

static DEVICE_ATTR(hsic_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_port_enable_show, hsic_port_enable_store);

static int ehci_hsic_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	struct hc_driver *driver;
	struct usb_hcd *hcd;
	struct ehci_hcd *ehci;
	int irq, retval;

	pr_debug("initializing Intel EHCI HSIC Host Controller\n");

	if (usb_disabled())
		return -ENODEV;

	if (!id)
		return -EINVAL;

	pci_dev = pdev;
	if (pci_enable_device(pdev) < 0)
		return -ENODEV;
	pdev->current_state = PCI_D0;

	/* we need not call pci_enable_dev since otg transceiver already take
	 * the control of this device and this probe actaully gets called by
	 * otg transceiver driver with HNP protocol.
	 */
	irq = pdev->irq;
	if (!pdev->irq) {
		dev_dbg(&pdev->dev, "No IRQ.\n");
		retval = -ENODEV;
		goto disable_pci;
	}

	driver = (struct hc_driver *)id->driver_data;
	if (!driver)
		return -EINVAL;

	/* AUX GPIO init */
	retval = hsic_aux_gpio_init();
	if (retval < 0)
		return retval;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto disable_pci;
	}

	ehci = hcd_to_ehci(hcd);

	hcd->rsrc_start = pci_resource_start(pdev, 0);
	hcd->rsrc_len = pci_resource_len(pdev, 0);
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
			driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto clear_companion;
	}

	hcd->regs = ioremap_nocache(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto release_mem_region;
	}

	pci_set_master(pdev);

	if (hsic_enable_created == 0) {
		retval = device_create_file(&pdev->dev, &dev_attr_hsic_enable);
		if (retval < 0) {
			dev_dbg(&pdev->dev, "error create hsic_enable\n");
			goto release_mem_region;
		}
		hsic_enable_created = 1;
	}

	if (hsic_mutex_init == 0) {
		mutex_init(&hsic.hsic_mutex);
		hsic_mutex_init = 1;
	}

	INIT_DELAYED_WORK(&(hsic.hsic_aux), hsic_aux_work);

	hcd->hsic_notify = hsic_notify;

	retval = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0)
		goto unmap_registers;
	dev_set_drvdata(&pdev->dev, hcd);

	if (pci_dev_run_wake(pdev))
		pm_runtime_put_noidle(&pdev->dev);

	/* Enable Runtime-PM if hcd->rpm_control == 1 */
	if (hcd->rpm_control) {
		/* Check here to avoid to call pm_runtime_put_noidle() twice */
		if (!pci_dev_run_wake(pdev))
			pm_runtime_put_noidle(&pdev->dev);

		pm_runtime_allow(&pdev->dev);
	}
	hsic_stopped = 0;

	return retval;

unmap_registers:
	if (driver->flags & HCD_MEMORY) {
		iounmap(hcd->regs);
release_mem_region:
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	} else
		release_region(hcd->rsrc_start, hcd->rsrc_len);
clear_companion:
	dev_set_drvdata(&pdev->dev, NULL);
	usb_put_hcd(hcd);
disable_pci:
	pci_disable_device(pdev);
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

static void ehci_hsic_remove(struct pci_dev *pdev)
{
	struct usb_hcd *hcd = pci_get_drvdata(pdev);

	if (!hcd)
		return;

	if (pci_dev_run_wake(pdev))
		pm_runtime_get_noresume(&pdev->dev);

	if (hcd->rpm_control) {
		if (!pci_dev_run_wake(pdev))
			pm_runtime_get_noresume(&pdev->dev);

		pm_runtime_forbid(&pdev->dev);
	}

	/* Free the aux irq */
	free_aux_irq();

	/* Fake an interrupt request in order to give the driver a chance
	 * to test whether the controller hardware has been removed (e.g.,
	 * cardbus physical eject).
	 */
	local_irq_disable();
	usb_hcd_irq(0, hcd);
	local_irq_enable();

	usb_remove_hcd(hcd);
	if (hcd->driver->flags & HCD_MEMORY) {
		iounmap(hcd->regs);
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	} else {
		release_region(hcd->rsrc_start, hcd->rsrc_len);
	}
	usb_put_hcd(hcd);
	gpio_free(HSIC_AUX_N);
	pci_disable_device(pdev);

	hsic_enter_exit_d3(1);
	hsic_stopped = 1;
}

static void ehci_hsic_shutdown(struct pci_dev *pdev)
{
	struct usb_hcd *hcd;

	dev_dbg(&pdev->dev, "%s --->\n", __func__);
	hcd = pci_get_drvdata(pdev);
	if (!hcd)
		return;

	if (test_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags) &&
			hcd->driver->shutdown) {
		hcd->driver->shutdown(hcd);
		pci_disable_device(pdev);
	}
	dev_dbg(&pdev->dev, "%s <---\n", __func__);
}

static int tangier_hsic_suspend_noirq(struct device *dev)
{
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);

	dev_dbg(dev, "%s <---\n", __func__);
	return ret;
}

static int tangier_hsic_suspend(struct device *dev)
{
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);

	dev_dbg(dev, "%s <---\n", __func__);
	return ret;
}

static int tangier_hsic_resume_noirq(struct device *dev)
{
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);


	dev_dbg(dev, "%s <---\n", __func__);
	return ret;
}

static int tangier_hsic_resume(struct device *dev)
{
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);

	dev_dbg(dev, "%s <---\n", __func__);
	return ret;
}

#ifdef CONFIG_PM_RUNTIME
/* Runtime PM */
static int tangier_hsic_runtime_suspend(struct device *dev)
{
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);

	dev_dbg(dev, "%s <---: ret = %d\n", __func__, ret);
	return ret;
}

static int tangier_hsic_runtime_resume(struct device *dev)
{
	int			ret = 0;

	dev_dbg(dev, "%s --->\n", __func__);

	dev_dbg(dev, "%s <---\n", __func__);

	return ret;
}

static int tangier_hsic_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s --->\n", __func__);

	dev_dbg(dev, "%s <---\n", __func__);

	return -EBUSY;
}

#else

#define tangier_hsic_runtime_suspend NULL
#define tangier_hsic_runtime_resume NULL
#define tangier_hsic_runtime_idle NULL

#endif

static DEFINE_PCI_DEVICE_TABLE(pci_hsic_ids) = {
	{
		.vendor =	0x8086,
		.device =	0x119D,
		.subvendor =	PCI_ANY_ID,
		.subdevice =	PCI_ANY_ID,
		.driver_data =  (unsigned long) &ehci_pci_hc_driver,
	},
	{ /* end: all zeroes */ }
};

static const struct dev_pm_ops tangier_hsic_pm_ops = {
	.runtime_suspend = tangier_hsic_runtime_suspend,
	.runtime_resume = tangier_hsic_runtime_resume,
	.runtime_idle = tangier_hsic_runtime_idle,
	.suspend = tangier_hsic_suspend,
	.suspend_noirq = tangier_hsic_suspend_noirq,
	.resume = tangier_hsic_resume,
	.resume_noirq = tangier_hsic_resume_noirq,
};

/* Intel HSIC EHCI driver */
static struct pci_driver ehci_hsic_driver = {
	.name =	"ehci-intel-hsic",
	.id_table =	pci_hsic_ids,

	.probe =	ehci_hsic_probe,
	.remove =	ehci_hsic_remove,

#ifdef CONFIG_PM_SLEEP
	.driver =	{
		.pm =	&tangier_hsic_pm_ops
	},
#endif
	.shutdown =	ehci_hsic_shutdown,
};


static int ehci_hsic_start_host(struct pci_dev  *pdev)
{
	int		retval;

	retval = ehci_hsic_probe(pdev, ehci_hsic_driver.id_table);
	if (retval)
		dev_dbg(&pdev->dev, "Failed to start host\n");

	return retval;
}
EXPORT_SYMBOL_GPL(ehci_hsic_start_host);

static int ehci_hsic_stop_host(struct pci_dev *pdev)
{
	ehci_hsic_remove(pdev);
	return 0;
}
EXPORT_SYMBOL_GPL(ehci_hsic_stop_host);
