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
#include <asm/intel-mid.h>

static struct pci_dev	*pci_dev;

static int ehci_hsic_start_host(struct pci_dev  *pdev);
static int ehci_hsic_stop_host(struct pci_dev *pdev);
static int create_L1_device_files();
static void remove_L1_device_files();
static int create_L2_device_files();
static void remove_L2_device_files();

static int hsic_enable;
static struct hsic_tangier_priv hsic;

/* Workaround for OSPM, set PMCMD to ask SCU
 * power gate EHCI controller and DPHY
 */
static void hsic_enter_exit_d3(int enter_exit)
{
	if (enter_exit) {
		printk(KERN_CRIT "HSIC Enter D0I3!\n");
		pci_set_power_state(pci_dev, PCI_D3cold);
	} else {
		printk(KERN_CRIT "HSIC Exit D0I3!\n");
		pci_set_power_state(pci_dev, PCI_D0);
	}
}

static void ehci_hsic_port_power(struct ehci_hcd *ehci, int is_on)
{
	unsigned port;

	if (!HCS_PPC(ehci->hcs_params))
		return;

	dev_dbg(&pci_dev->dev, "...power%s ports...\n", is_on ? "up" : "down");
	for (port = HCS_N_PORTS(ehci->hcs_params); port > 0; )
		(void) ehci_hub_control(ehci_to_hcd(ehci),
				is_on ? SetPortFeature : ClearPortFeature,
				USB_PORT_FEAT_POWER,
				port--, NULL, 0);
	/* Flush those writes */
	ehci_readl(ehci, &ehci->regs->command);
	usleep_range(1000, 1200);
}

/* Init HSIC AUX GPIO as outpur PD */
static int hsic_aux_gpio_init(void)
{
	int		retval = 0;

	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	hsic.aux_gpio = get_gpio_by_name(HSIC_AUX_GPIO_NAME);
	if (gpio_is_valid(hsic.aux_gpio)) {
		retval = gpio_request(hsic.aux_gpio, "hsic_aux");
		if (retval < 0) {
			dev_err(&pci_dev->dev,
				"Request GPIO %d with error %d\n",
				hsic.aux_gpio, retval);
			retval = -ENODEV;
			goto err;
		}
	} else {
		retval = -ENODEV;
		goto err;
	}

	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return retval;

err:
	gpio_free(hsic.aux_gpio);
	return retval;
}

static void hsic_aux_irq_free(void)
{
	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	if (hsic.hsic_aux_irq_enable) {
		hsic.hsic_aux_irq_enable = 0;
		free_irq(gpio_to_irq(hsic.aux_gpio), &pci_dev->dev);
	}
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return;
}

/* HSIC AUX GPIO irq handler */
static irqreturn_t hsic_aux_gpio_irq(int irq, void *data)
{
	struct device *dev = data;

	dev_dbg(dev,
		"%s---> hsic aux gpio request irq: %d\n",
		__func__, irq);

	if (hsic.hsic_aux_irq_enable == 0) {
		dev_dbg(dev,
			"%s---->AUX IRQ is disabled\n", __func__);
		return IRQ_HANDLED;
	}

	if (delayed_work_pending(&hsic.hsic_aux)) {
		dev_dbg(dev,
			"%s---->Delayed work pending\n", __func__);
		return IRQ_HANDLED;
	}

	hsic.hsic_aux_finish = 0;
	schedule_delayed_work(&hsic.hsic_aux, 0);
	dev_dbg(dev,
		"%s<----\n", __func__);

	return IRQ_HANDLED;
}

static int hsic_aux_irq_init(void)
{
	int retval;

	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	if (hsic.hsic_aux_irq_enable) {
		dev_dbg(&pci_dev->dev,
			"%s<----AUX IRQ is enabled\n", __func__);
		return 0;
	}
	hsic.hsic_aux_irq_enable = 1;
	gpio_direction_input(hsic.aux_gpio);
	retval = request_irq(gpio_to_irq(hsic.aux_gpio),
			hsic_aux_gpio_irq,
			IRQF_SHARED | IRQF_TRIGGER_FALLING,
			"hsic_disconnect_request", &pci_dev->dev);
	if (retval) {
		dev_err(&pci_dev->dev,
			"unable to request irq %i, err: %d\n",
			gpio_to_irq(hsic.aux_gpio), retval);
		goto err;
	}

	lnw_gpio_set_alt(hsic.aux_gpio, 0);
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);

	return retval;

err:
	hsic.hsic_aux_irq_enable = 0;
	free_irq(gpio_to_irq(hsic.aux_gpio), &pci_dev->dev);
	return retval;
}

/* the root hub will call this callback when device added/removed */
static void hsic_notify(struct usb_device *udev, unsigned action)
{
	int retval;
	struct pci_dev *pdev = to_pci_dev(udev->bus->controller);

	/* Ignore and only valid for HSIC. Filter out
	 * the USB devices added by other USB2 host driver */
	if (pdev->device != 0x119d)
		return;

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
		/* Root hub */
		if (!udev->parent) {
			hsic.rh_dev = udev;
			pr_debug("%s Disable autosuspend\n", __func__);
			pm_runtime_set_autosuspend_delay(&udev->dev,
					hsic.L2_inactivityDuration);
			usb_disable_autosuspend(udev);
		} else {
			/* Modem devices */
			hsic.modem_dev = udev;
			pm_runtime_set_autosuspend_delay
				(&udev->dev, hsic.L1_inactivityDuration);

			if (hsic.remoteWakeup_enable) {
				pr_debug("%s Modem dev remote wakeup enabled\n",
						 __func__);
				device_set_wakeup_capable
					(&hsic.modem_dev->dev, 1);
				device_set_wakeup_capable
					(&hsic.rh_dev->dev, 1);
			} else {
				pr_debug("%s Modem dev remote wakeup disabled\n",
						 __func__);
				device_set_wakeup_capable
					(&hsic.modem_dev->dev, 0);
				device_set_wakeup_capable
					(&hsic.rh_dev->dev, 0);
			}

			if (hsic.L2_autosuspend_enable ||
				hsic.L1_autosuspend_enable) {
				pr_debug("%s----> enable autosuspend\n",
					 __func__);
				usb_enable_autosuspend(udev->parent);
			}

			if ((hsic.L1_autosuspend_enable == 0) &&
				(hsic.L2_autosuspend_enable == 0)) {
				pr_debug("%s Modem dev autosuspend disable\n",
						 __func__);
				usb_disable_autosuspend(hsic.modem_dev);
			}

			pr_debug("%s----> Enable AUX irq\n", __func__);
			retval = hsic_aux_irq_init();
			if (retval)
				dev_err(&pci_dev->dev,
					"unable to request IRQ\n");
		}
		break;
	case USB_DEVICE_REMOVE:
		pr_debug("Notify HSIC delete device\n");
		/* Root hub */
		if (!udev->parent) {
			pr_debug("%s rh_dev deleted\n", __func__);
			hsic.rh_dev = NULL;
		} else {
			/* Modem devices */
			pr_debug("%s----> modem dev deleted\n", __func__);
			hsic.modem_dev = NULL;
		}
		break;
	default:
		pr_debug("Notify action not supported\n");
		break ;
	}
	return;
}

static void hsic_aux_work(struct work_struct *work)
{
	dev_dbg(&pci_dev->dev,
		"%s---->\n", __func__);
	mutex_lock(&hsic.hsic_mutex);
	/* Free the aux irq */
	hsic_aux_irq_free();
	dev_dbg(&pci_dev->dev,
		"%s---->AUX IRQ is disabled\n", __func__);

	if (hsic.hsic_stopped == 0)
		ehci_hsic_stop_host(pci_dev);

	hsic_enter_exit_d3(1);
	usleep_range(5000, 6000);
	hsic_enter_exit_d3(0);
	ehci_hsic_start_host(pci_dev);
	mutex_unlock(&hsic.hsic_mutex);

	hsic.hsic_aux_finish = 1;
	wake_up(&hsic.aux_wq);
	dev_dbg(&pci_dev->dev,
		"%s<----\n", __func__);
	return;
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
	int org_req;

	if (size > HSIC_ENABLE_SIZE)
		return -EINVAL;

	if (sscanf(buf, "%d", &org_req) == 1) {
		/* Free the aux irq */
		hsic_aux_irq_free();
		dev_dbg(dev,
			"%s---->AUX IRQ is disabled\n", __func__);

		if (delayed_work_pending(&hsic.hsic_aux)) {
			dev_dbg(dev,
				"%s---->Wait for delayed work finish\n",
				 __func__);
			retval = wait_event_interruptible(hsic.aux_wq,
							hsic.hsic_aux_finish);
			if (retval < 0)
				return retval;

			if (org_req)
				return size;
		}

		if (org_req) {
			dev_dbg(dev, "enable hsic\n");
			mutex_lock(&hsic.hsic_mutex);
			/* add this due to hcd release
				 doesn't set hcd to NULL */
			if (hsic.hsic_stopped == 0)
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
			if (hsic.hsic_stopped == 0)
				ehci_hsic_stop_host(pci_dev);
			mutex_unlock(&hsic.hsic_mutex);

			return size;
		}
	}

	return -EINVAL;
}

static DEVICE_ATTR(hsic_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_port_enable_show, hsic_port_enable_store);

/* Interfaces for LPM L1 */
static ssize_t hsic_L1_autosuspend_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.L1_autosuspend_enable);
}

static ssize_t hsic_L1_autosuspend_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;

	if (size > HSIC_ENABLE_SIZE)
		return -EINVAL;

	if (hsic.modem_dev == NULL) {
		dev_dbg(dev, "Modem dev is not created\n");
		sscanf(buf, "%d", &hsic.L1_autosuspend_enable);
		return -ENODEV;
	}

	if (sscanf(buf, "%d", &hsic.L1_autosuspend_enable) == 1) {
		if ((hsic.L1_autosuspend_enable == 0) &&
			(hsic.L2_autosuspend_enable == 0)) {
			dev_dbg(dev,
			 "Modem dev autosuspend disable\n");
			mutex_lock(&hsic.hsic_mutex);
			usb_disable_autosuspend(hsic.modem_dev);
			mutex_unlock(&hsic.hsic_mutex);
		} else {
			dev_dbg(dev, "Disable L1 auto suspend\n");
			mutex_lock(&hsic.hsic_mutex);
			usb_disable_autosuspend(hsic.modem_dev);
			usb_enable_autosuspend(hsic.modem_dev);
			mutex_unlock(&hsic.hsic_mutex);
		}

		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(L1_autosuspend_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_L1_autosuspend_enable_show,
		 hsic_L1_autosuspend_enable_store);

static ssize_t hsic_L1_inactivityDuration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.L1_inactivityDuration);
}

static ssize_t hsic_L1_inactivityDuration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;

	if (size > HSIC_DURATION_SIZE) {
		dev_dbg(dev, "Invalid, size = %d\n", size);
		sscanf(buf, "%d", &hsic.L1_inactivityDuration);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &hsic.L1_inactivityDuration) == 1) {
		mutex_lock(&hsic.hsic_mutex);
		if (hsic.modem_dev == NULL) {
			dev_dbg(dev, "Root hub dev is not created\n");
			mutex_unlock(&hsic.hsic_mutex);
			return -ENODEV;
		}

		dev_dbg(dev, "L2 Duration: %d\n", hsic.L1_inactivityDuration);
		pm_runtime_set_autosuspend_delay
			(&hsic.modem_dev->dev, hsic.L1_inactivityDuration);
		mutex_unlock(&hsic.hsic_mutex);
		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(L1_inactivityDuration, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_L1_inactivityDuration_show,
		 hsic_L1_inactivityDuration_store);

static int create_L1_device_files()
{
	int retval;

	hsic.L1_autosuspend_enable = HSIC_L1_AUTOSUSPEND;
	retval = device_create_file(&pci_dev->dev,
			 &dev_attr_L1_autosuspend_enable);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create L1_autosuspend_enable\n");
		return retval;
	}

	hsic.L1_inactivityDuration = HSIC_L1_INACTIVITYDURATION;
	retval = device_create_file(&pci_dev->dev,
			 &dev_attr_L1_inactivityDuration);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create L1_inactiveDuration\n");
		return retval;
	}

	return retval;
}

static void remove_L1_device_files()
{
	device_remove_file(&pci_dev->dev, &dev_attr_L1_autosuspend_enable);
	device_remove_file(&pci_dev->dev, &dev_attr_L1_inactivityDuration);
}

/* Interfaces for L2 suspend */
static ssize_t hsic_L2_autosuspend_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.L2_autosuspend_enable);
}

static ssize_t hsic_L2_autosuspend_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;

	if (size > HSIC_ENABLE_SIZE)
		return -EINVAL;

	if (hsic.modem_dev == NULL) {
		dev_dbg(dev, "Modem dev is not created\n");
		sscanf(buf, "%d", &hsic.L2_autosuspend_enable);
		return -ENODEV;
	}

	if (sscanf(buf, "%d", &hsic.L2_autosuspend_enable) == 1) {
		if ((hsic.L1_autosuspend_enable == 0) &&
			(hsic.L2_autosuspend_enable == 0)) {
			dev_dbg(dev, "Modem dev autosuspend disable\n",
					 __func__);
			mutex_lock(&hsic.hsic_mutex);
			usb_disable_autosuspend(hsic.modem_dev);
			mutex_unlock(&hsic.hsic_mutex);
		} else {
			dev_dbg(dev, "Disable L2 auto suspend\n", __func__);
			mutex_lock(&hsic.hsic_mutex);
			usb_disable_autosuspend(hsic.modem_dev);
			usb_enable_autosuspend(hsic.modem_dev);
			mutex_unlock(&hsic.hsic_mutex);
		}

		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(L2_autosuspend_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_L2_autosuspend_enable_show,
		 hsic_L2_autosuspend_enable_store);

static ssize_t hsic_L2_inactivityDuration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.L2_inactivityDuration);
}

static ssize_t hsic_L2_inactivityDuration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;

	if (size > HSIC_DURATION_SIZE) {
		dev_dbg(dev, "Invalid, size = %d\n", size);
		return -EINVAL;
	}

	if (sscanf(buf, "%d", &hsic.L2_inactivityDuration) == 1) {
		mutex_lock(&hsic.hsic_mutex);
		if (hsic.rh_dev == NULL) {
			dev_dbg(dev, "Root hub dev is not created\n");
			sscanf(buf, "%d", &hsic.L2_inactivityDuration);
			mutex_unlock(&hsic.hsic_mutex);
			return -ENODEV;
		}

		dev_dbg(dev, "L2 Duration: %d\n", hsic.L2_inactivityDuration);
		pm_runtime_set_autosuspend_delay
			(&hsic.rh_dev->dev, hsic.L2_inactivityDuration);
		mutex_unlock(&hsic.hsic_mutex);
		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(L2_inactivityDuration, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_L2_inactivityDuration_show,
		 hsic_L2_inactivityDuration_store);

static ssize_t hsic_remoteWakeup_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", hsic.remoteWakeup_enable);
}

static ssize_t hsic_remoteWakeup_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int retval;

	if (size > HSIC_ENABLE_SIZE)
		return -EINVAL;

	if (hsic.modem_dev == NULL) {
		dev_dbg(dev, "Modem dev is not created\n");
		sscanf(buf, "%d", &hsic.remoteWakeup_enable);
		return -ENODEV;
	}

	if (sscanf(buf, "%d", &hsic.remoteWakeup_enable) == 1) {
		if (hsic.remoteWakeup_enable) {
			dev_dbg(dev, "Modem dev remote wakeup enabled\n");
			mutex_lock(&hsic.hsic_mutex);
			device_set_wakeup_capable(&hsic.modem_dev->dev, 1);
			device_set_wakeup_capable(&hsic.rh_dev->dev, 1);
			pm_runtime_get_sync(&hsic.modem_dev->dev);
			pm_runtime_put_sync(&hsic.modem_dev->dev);
			mutex_unlock(&hsic.hsic_mutex);
		} else {
			dev_dbg(dev, "Modem dev remote wakeup disabled\n");
			mutex_lock(&hsic.hsic_mutex);
			device_set_wakeup_capable(&hsic.modem_dev->dev, 0);
			device_set_wakeup_capable(&hsic.rh_dev->dev, 0);
			pm_runtime_get_sync(&hsic.modem_dev->dev);
			pm_runtime_put_sync(&hsic.modem_dev->dev);
			mutex_unlock(&hsic.hsic_mutex);
		}
		return size;
	}

	return -EINVAL;
}

static DEVICE_ATTR(remoteWakeup, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		hsic_remoteWakeup_show, hsic_remoteWakeup_store);

static int create_L2_device_files()
{
	int retval;

	hsic.L2_autosuspend_enable = HSIC_L2_AUTOSUSPEND;
	retval = device_create_file(&pci_dev->dev,
			 &dev_attr_L2_autosuspend_enable);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create L2_autosuspend_enable\n");
		return retval;
	}

	hsic.L2_inactivityDuration = HSIC_L2_INACTIVITYDURATION;
	retval = device_create_file(&pci_dev->dev,
			 &dev_attr_L2_inactivityDuration);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create L2_inactiveDuration\n");
		return retval;
	}

	hsic.remoteWakeup_enable = HSIC_REMOTEWAKEUP;
	retval = device_create_file(&pci_dev->dev, &dev_attr_remoteWakeup);
	if (retval < 0) {
		dev_dbg(&pci_dev->dev, "Error create remoteWakeup\n");
		return retval;
	}

	return retval;
}

static void remove_L2_device_files()
{
	device_remove_file(&pci_dev->dev, &dev_attr_L2_autosuspend_enable);
	device_remove_file(&pci_dev->dev, &dev_attr_L2_inactivityDuration);
	device_remove_file(&pci_dev->dev, &dev_attr_remoteWakeup);
}

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

	if (hsic.hsic_enable_created == 0) {
		retval = device_create_file(&pdev->dev, &dev_attr_hsic_enable);
		if (retval < 0) {
			dev_dbg(&pdev->dev, "error create hsic_enable\n");
			goto release_mem_region;
		}
		create_L1_device_files();
		create_L2_device_files();
		hsic.hsic_enable_created = 1;
	}

	if (hsic.hsic_mutex_init == 0) {
		mutex_init(&hsic.hsic_mutex);
		hsic.hsic_mutex_init = 1;
	}

	if (hsic.aux_wq_init == 0) {
		init_waitqueue_head(&hsic.aux_wq);
		hsic.aux_wq_init = 1;
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
	hsic.hsic_stopped = 0;
	hsic_enable = 1;

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
	struct usb_hcd    *hcd = pci_get_drvdata(pdev);
	struct ehci_hcd   *ehci = hcd_to_ehci(hcd);

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
	hsic_aux_irq_free();

	/* Fake an interrupt request in order to give the driver a chance
	 * to test whether the controller hardware has been removed (e.g.,
	 * cardbus physical eject).
	 */
	local_irq_disable();
	usb_hcd_irq(0, hcd);
	local_irq_enable();

	usb_remove_hcd(hcd);
	ehci_hsic_port_power(ehci, 0);

	if (hcd->driver->flags & HCD_MEMORY) {
		iounmap(hcd->regs);
		release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	} else {
		release_region(hcd->rsrc_start, hcd->rsrc_len);
	}

	usb_put_hcd(hcd);
	gpio_free(hsic.aux_gpio);
	pci_disable_device(pdev);

	hsic.hsic_stopped = 1;
	hsic_enable = 0;
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

#ifdef CONFIG_PM_SLEEP
static int tangier_hsic_suspend_noirq(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.suspend_noirq(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int tangier_hsic_suspend(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.suspend(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int tangier_hsic_resume_noirq(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.resume_noirq(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int tangier_hsic_resume(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.resume(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}
#else
#define tangier_hsic_suspend_noirq	NULL
#define tangier_hsic_suspend		NULL
#define tangier_hsic_resume_noirq	NULL
#define tangier_hsic_resume		NULL
#endif

#ifdef CONFIG_PM_RUNTIME
/* Runtime PM */
static int tangier_hsic_runtime_suspend(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.runtime_suspend(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);
	return retval;
}

static int tangier_hsic_runtime_resume(struct device *dev)
{
	int	retval;

	dev_dbg(dev, "%s --->\n", __func__);
	retval = usb_hcd_pci_pm_ops.runtime_resume(dev);
	dev_dbg(dev, "%s <--- retval = %d\n", __func__, retval);

	return retval;
}
#else
#define tangier_hsic_runtime_suspend NULL
#define tangier_hsic_runtime_resume NULL
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
