/*
 * Copyright (C) 2012 Intel Corp.
 * Author: Yu Wang
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
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

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/usb/otg.h>
#include <linux/usb/dwc_otg3.h>
#include <linux/platform_device.h>

static struct platform_device *dwcdev;
static int otg_irqnum;

static int xhci_start_host(struct usb_hcd *hcd);
static int xhci_stop_host(struct usb_hcd *hcd);
static int xhci_reset_port(struct usb_hcd *hcd);
static int xhci_stop_host(struct usb_hcd *hcd);
static int dwc_host_setup(struct usb_hcd *hcd);
static int xhci_release_host(struct usb_hcd *hcd);
static struct platform_driver xhci_dwc_driver;

static void set_phy_suspend_resume(struct usb_hcd *hcd, int on_off)
{
	/* Comment the actual PHY operations. This is not final hardware desgin.
	 * And on current HVP platform, it will cause LS/FS devices
	 * can't enumerate success after resume from D0I1 mode.
	 */
#if 0
	u32 data = 0;
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);

	switch (hcd->speed) {
	case HCD_USB2:
		data = readl(hcd->regs + GUSB2PHYCFG0);
		if (on_off)
			data |= GUSB2PHYCFG_SUS_PHY;
		else
			data &= ~GUSB2PHYCFG_SUS_PHY;
		writel(data, hcd->regs + GUSB2PHYCFG0);
		break;
	case HCD_USB3:
		data = readl(hcd->regs + GUSB3PIPECTL0);
		if (on_off)
			data |= GUSB3PIPECTL_SUS_EN;
		else
			data &= ~GUSB3PIPECTL_SUS_EN;
		writel(data, hcd->regs + GUSB3PIPECTL0);
		break;
	default:
		xhci_err(xhci, "Invalid arguments in %s!\n", __func__);
		return;
	}
#endif

	printk(KERN_ERR "DWC USB%d phy set %s.\n",
			HCD_USB2 == hcd->speed ? 2 : 3,
			on_off ? "suspend" : "resume");
}

static int xhci_dwc_bus_suspend(struct usb_hcd *hcd)
{
	int ret;
	ret = xhci_bus_suspend(hcd);
	set_phy_suspend_resume(hcd, 1);
	return ret;
}

static int xhci_dwc_bus_resume(struct usb_hcd *hcd)
{
	int ret;

	/* delay 1ms to waiting core stable */
	mdelay(1);

	ret = xhci_bus_resume(hcd);
	set_phy_suspend_resume(hcd, 0);
	return ret;
}

static struct hc_driver xhci_dwc_hc_driver = {
	.description =		"dwc-xhci",
	.product_desc =		"xHCI Host Controller",
	.hcd_priv_size =	sizeof(struct xhci_hcd *),

	/*
	 * generic hardware linkage
	 */
	.irq =			xhci_irq,
	.flags =		HCD_MEMORY | HCD_USB3 | HCD_SHARED,

	/*
	 * basic lifecycle operations
	 */
	.reset =		dwc_host_setup,
	.start =		xhci_run,
	.stop =			xhci_stop,
	.shutdown =		xhci_shutdown,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		xhci_urb_enqueue,
	.urb_dequeue =		xhci_urb_dequeue,
	.alloc_dev =		xhci_alloc_dev,
	.free_dev =		xhci_free_dev,
	.alloc_streams =	xhci_alloc_streams,
	.free_streams =		xhci_free_streams,
	.add_endpoint =		xhci_add_endpoint,
	.drop_endpoint =	xhci_drop_endpoint,
	.endpoint_reset =	xhci_endpoint_reset,
	.check_bandwidth =	xhci_check_bandwidth,
	.reset_bandwidth =	xhci_reset_bandwidth,
	.address_device =	xhci_address_device,
	.update_hub_device =	xhci_update_hub_device,
	.reset_device =		xhci_discover_or_reset_device,

	/*
	 * scheduling support
	 */
	.get_frame_number =	xhci_get_frame,

	/* Root hub support */
	.hub_control =		xhci_hub_control,
	.hub_status_data =	xhci_hub_status_data,
	.bus_suspend =		xhci_dwc_bus_suspend,
	.bus_resume =		xhci_dwc_bus_resume,

	.start_host = xhci_start_host,
	.stop_host = xhci_stop_host,
	.reset_port = xhci_reset_port,
	.release_host = xhci_release_host,
};


static void dwc_set_host_mode(struct usb_hcd *hcd)
{
	u32 octl = 0;

	writel(0x45801000, hcd->regs + GCTL);

	octl = readl(hcd->regs + OCTL);
	octl &= (~OCTL_PERI_MODE);
	writel(octl, hcd->regs + OCTL);

	msleep(20);
}

static void dwc_xhci_enable_phy_suspend(struct usb_hcd *hcd, int enable)
{
	u32 val;

	val = readl(hcd->regs + GUSB3PIPECTL0);
	if (enable)
		val |= GUSB3PIPECTL_SUS_EN;
	else
		val &= ~GUSB3PIPECTL_SUS_EN;
	writel(val, hcd->regs + GUSB3PIPECTL0);

	val = readl(hcd->regs + GUSB2PHYCFG0);
	if (enable)
		val |= GUSB2PHYCFG_SUS_PHY;
	else
		val &= ~GUSB2PHYCFG_SUS_PHY;
	writel(val, hcd->regs + GUSB2PHYCFG0);
}

static void dwc_core_reset(struct usb_hcd *hcd)
{
	u32 val;

	val = readl(hcd->regs + GCTL);
	val |= GCTL_CORESOFTRESET;
	writel(val, hcd->regs + GCTL);

	val = readl(hcd->regs + GUSB3PIPECTL0);
	val |= GUSB3PIPECTL_PHYSOFTRST;
	writel(val, hcd->regs + GUSB3PIPECTL0);

	val = readl(hcd->regs + GUSB2PHYCFG0);
	val |= GUSB2PHYCFG_PHYSOFTRST;
	writel(val, hcd->regs + GUSB2PHYCFG0);

	msleep(100);

	val = readl(hcd->regs + GUSB3PIPECTL0);
	val &= ~GUSB3PIPECTL_PHYSOFTRST;
	writel(val, hcd->regs + GUSB3PIPECTL0);

	val = readl(hcd->regs + GUSB2PHYCFG0);
	val &= ~GUSB2PHYCFG_PHYSOFTRST;
	writel(val, hcd->regs + GUSB2PHYCFG0);

	msleep(20);

	val = readl(hcd->regs + GCTL);
	val &= ~GCTL_CORESOFTRESET;
	writel(val, hcd->regs + GCTL);

	/* Clear GUCTL bit 15 as workaround of DWC2.10a Bugs
	 * This Bug cause the xHCI driver does not see any
	 * transfer complete events for certain EP after exit
	 * from hibernation mode. This Bug will be fix in TNG B0
	 * and this workaround impact to compliance test. */
	val = readl(hcd->regs + GUCTL);
	val &= ~GUCTL_CMDEVADDR;
	writel(val, hcd->regs + GUCTL);
}


/* This is a hardware workaround.
 * xHCI RxDetect state is not work well when USB3
 * PHY under P3 state. So force PHY change to P2 when
 * xHCI want to perform receiver detection.
 */
static void dwc_disable_ssphy_p3(struct usb_hcd *hcd)
{
	u32 phyval;

	phyval = readl(hcd->regs + GUSB3PIPECTL0);
	phyval |= GUSB3PIPE_DISRXDETP3;
	writel(phyval, hcd->regs + GUSB3PIPECTL0);

}

static ssize_t
show_pm_get(struct device *_dev, struct device_attribute *attr, char *buf)
{
	struct platform_device		*pdev = to_platform_device(_dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);
	printk(KERN_ERR "pm_runtime_put for xHCI device\n");

	pm_runtime_put(hcd->self.controller);
	return 0;

}
static ssize_t store_pm_get(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device		*pdev = to_platform_device(_dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);
	printk(KERN_ERR "pm_runtime_get for xHCI device\n");

	pm_runtime_get(hcd->self.controller);
	return count;

}
static DEVICE_ATTR(pm_get, S_IRUGO|S_IWUSR|S_IWGRP,\
			show_pm_get, store_pm_get);

int disable_pm;
static ssize_t store_disable_pm(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	if (!disable_pm && buf[0] == '1') {
		disable_pm = 1;
		xhci_dwc_hc_driver.bus_suspend = NULL;
		xhci_dwc_hc_driver.bus_resume = NULL;
		printk(KERN_ERR "Disable power management of xHCI!\n");
	} else if (disable_pm && buf[0] == '0') {
		xhci_dwc_hc_driver.bus_suspend = xhci_dwc_bus_suspend;
		xhci_dwc_hc_driver.bus_resume = xhci_dwc_bus_resume;
		disable_pm = 0;
		printk(KERN_ERR "Enable power management of xHCI!\n");
	}

	return count;
}

static ssize_t
show_disable_pm(struct device *_dev, struct device_attribute *attr, char *buf)
{
	char				*next;
	unsigned			size, t;

	next = buf;
	size = PAGE_SIZE;

	t = scnprintf(next, size,
		"%s\n",
		(disable_pm ? "1" : "0")
		);
	size -= t;
	next += t;

	return PAGE_SIZE - size;
}

static DEVICE_ATTR(disable_pm, S_IRUGO|S_IWUSR|S_IWGRP,\
			show_disable_pm, store_disable_pm);

static int xhci_start_host(struct usb_hcd *hcd)
{
	int ret = -EINVAL;
	struct xhci_hcd *xhci;


	if (!hcd) {
		printk(KERN_DEBUG "%s() - NULL pointer returned", __func__);
		return ret;
	}

	if (hcd->rh_registered) {
		printk(KERN_DEBUG "%s() - Already registered", __func__);
		return 0;
	}

	pm_runtime_get_sync(hcd->self.controller);

	dwc_core_reset(hcd);
	dwc_set_host_mode(hcd);
	dwc_disable_ssphy_p3(hcd);

	ret = usb_add_hcd(hcd, otg_irqnum, IRQF_DISABLED | IRQF_SHARED);
	if (ret)
		printk(KERN_DEBUG "%s() - Host start failed, err = %d\n",
					__func__, ret);
	else
		printk(KERN_DEBUG "%s() - Host started\n", __func__);


	xhci = hcd_to_xhci(hcd);
	xhci->shared_hcd = usb_create_shared_hcd(&xhci_dwc_hc_driver,\
		   hcd->self.controller, dev_name(hcd->self.controller), hcd);
	if (!xhci->shared_hcd) {
		ret = -ENOMEM;
		goto dealloc_usb2_hcd;
	}

	/* Set the xHCI pointer before xhci_pci_setup() (aka hcd_driver.reset)
	 * is called by usb_add_hcd().
	 */
	*((struct xhci_hcd **) xhci->shared_hcd->hcd_priv) = xhci;

	xhci->shared_hcd->regs = hcd->regs;

	xhci->shared_hcd->rsrc_start = hcd->rsrc_start;
	xhci->shared_hcd->rsrc_len = hcd->rsrc_len;

	ret = usb_add_hcd(xhci->shared_hcd, otg_irqnum,
			IRQF_DISABLED | IRQF_SHARED);
	if (ret)
		goto put_usb3_hcd;

	pm_runtime_put(hcd->self.controller);
	ret = device_create_file(hcd->self.controller, &dev_attr_pm_get);
	if (ret < 0) {
		printk(KERN_ERR
			"Can't register sysfs attribute: %d\n", ret);
	}

	xhci_dwc_driver.shutdown = usb_hcd_platform_shutdown;

	return ret;

put_usb3_hcd:
	if (xhci->shared_hcd) {
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
	}

dealloc_usb2_hcd:
	local_irq_disable();
	usb_hcd_irq(0, hcd);
	local_irq_enable();
	usb_remove_hcd(hcd);

	kfree(xhci);
	*((struct xhci_hcd **) hcd->hcd_priv) = NULL;

	pm_runtime_put(hcd->self.controller);
	return ret;
}

static int xhci_stop_host(struct usb_hcd *hcd)
{
	struct xhci_hcd *xhci;

	if (!hcd) {
		printk(KERN_DEBUG "%s() - NULL pointer returned", __func__);
		return -EINVAL;
	}

	xhci = hcd_to_xhci(hcd);

	pm_runtime_get_sync(hcd->self.controller);

	xhci_dwc_driver.shutdown = NULL;

	if (HCD_RH_RUNNING(hcd))
		set_phy_suspend_resume(hcd, 1);
	else if (HCD_RH_RUNNING(xhci->shared_hcd))
		set_phy_suspend_resume(xhci->shared_hcd, 1);

	if (xhci->shared_hcd) {
		usb_remove_hcd(xhci->shared_hcd);
		usb_put_hcd(xhci->shared_hcd);
	}

	/* Fake an interrupt request in order to give the driver a chance
	 * to test whether the controller hardware has been removed (e.g.,
	 * cardbus physical eject).
	 */
	local_irq_disable();
	usb_hcd_irq(0, hcd);
	local_irq_enable();

	usb_remove_hcd(hcd);

	kfree(xhci);
	*((struct xhci_hcd **) hcd->hcd_priv) = NULL;

	dwc_xhci_enable_phy_suspend(hcd, 0);

	pm_runtime_put(hcd->self.controller);
	device_remove_file(hcd->self.controller, &dev_attr_pm_get);
	return 0;
}

static int xhci_reset_port(struct usb_hcd *hcd)
{
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	if (!hcd)
		return -EINVAL;

	if (hcd->driver->start_port_reset) {
		xhci_dbg(xhci, "%s() - Resetting the port\n", __func__);
		hcd->driver->start_port_reset(hcd, 1);
	}
	return 0;
}


/* Workaround function:
 * Because we can't access xhci_pci_setup function, so copy it to here.
 */
static int dwc_host_setup(struct usb_hcd *hcd)
{
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	int			retval;
	u32 temp;

	hcd->self.sg_tablesize = TRBS_PER_SEGMENT - 2;

	if (usb_hcd_is_primary_hcd(hcd)) {
		xhci = kzalloc(sizeof(struct xhci_hcd), GFP_KERNEL);
		if (!xhci)
			return -ENOMEM;
		*((struct xhci_hcd **) hcd->hcd_priv) = xhci;
		xhci->main_hcd = hcd;
		/* Mark the first roothub as being USB 2.0.
		 * The xHCI driver will register the USB 3.0 roothub.
		 */
		hcd->speed = HCD_USB2;
		hcd->self.root_hub->speed = USB_SPEED_HIGH;
		/*
		 * USB 2.0 roothub under xHCI has an integrated TT,
		 * (rate matching hub) as opposed to having an OHCI/UHCI
		 * companion controller.
		 */
		hcd->has_tt = 1;
	} else {
		/* xHCI private pointer was set in xhci_pci_probe for the second
		 * registered roothub.
		 */
		xhci = hcd_to_xhci(hcd);
		temp = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
		if (HCC_64BIT_ADDR(temp)) {
			xhci_dbg(xhci, "Enabling 64-bit DMA addresses.\n");
			dma_set_mask(hcd->self.controller, DMA_BIT_MASK(64));
		} else {
			dma_set_mask(hcd->self.controller, DMA_BIT_MASK(32));
		}
		return 0;
	}

	xhci->cap_regs = hcd->regs;
	xhci->op_regs = hcd->regs +
		HC_LENGTH(xhci_readl(xhci, &xhci->cap_regs->hc_capbase));
	xhci->run_regs = hcd->regs +
		(xhci_readl(xhci, &xhci->cap_regs->run_regs_off) & RTSOFF_MASK);
	/* Cache read-only capability registers */
	xhci->hcs_params1 = xhci_readl(xhci, &xhci->cap_regs->hcs_params1);
	xhci->hcs_params2 = xhci_readl(xhci, &xhci->cap_regs->hcs_params2);
	xhci->hcs_params3 = xhci_readl(xhci, &xhci->cap_regs->hcs_params3);
	xhci->hcc_params = xhci_readl(xhci, &xhci->cap_regs->hc_capbase);
	xhci->hci_version = HC_VERSION(xhci->hcc_params);
	xhci->hcc_params = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
	xhci_print_registers(xhci);

	/*
	 * As of now platform drivers don't provide MSI support, so ensure
	 * here that the generic code does not try to get MSI support
	 * */
	xhci->quirks |= XHCI_BROKEN_MSI;

	/* Make sure the HC is halted. */
	retval = xhci_halt(xhci);
	if (retval)
		return retval;

	xhci_dbg(xhci, "Resetting HCD\n");
	/* Reset the internal HC memory state and registers. */
	retval = xhci_reset(xhci);
	if (retval)
		return retval;
	xhci_dbg(xhci, "Reset complete\n");

	temp = xhci_readl(xhci, &xhci->cap_regs->hcc_params);
	if (HCC_64BIT_ADDR(temp)) {
		xhci_dbg(xhci, "Enabling 64-bit DMA addresses.\n");
		dma_set_mask(hcd->self.controller, DMA_BIT_MASK(64));
	} else {
		dma_set_mask(hcd->self.controller, DMA_BIT_MASK(32));
	}

	xhci_dbg(xhci, "Calling HCD init\n");
	/* Initialize HCD and host controller data structures. */
	retval = xhci_init(hcd);
	if (retval)
		return retval;
	xhci_dbg(xhci, "Called HCD init\n");

	xhci_dbg(xhci, "Got SBRN %u\n", (unsigned int) xhci->sbrn);

	/* Find any debug ports */
	/* Workaround to comment this function */
	/* return xhci_pci_reinit(xhci, pdev); */
	return retval;
}


static int xhci_release_host(struct usb_hcd *hcd)
{
	int i;
	struct usb_device *udev;
	struct usb_bus *bus;
	struct usb_device *rh;
	dwcdev = NULL;

	if (!hcd)
		return -EINVAL;

	udev = NULL;
	bus = &hcd->self;
	rh = bus->root_hub;

	for (i = 0; i <= rh->maxchild; i++) {
		udev = rh->children[i];
		if (!udev)
			continue;

		if (udev->config
		    && udev->parent == udev->bus->root_hub) {

			struct usb_otg_descriptor	*desc = NULL;

			if (__usb_get_extra_descriptor(udev->rawdescriptors[0],
				le16_to_cpu(udev->config[0].desc.wTotalLength),
				USB_DT_OTG, (void **) &desc) == 0) {
				int err = usb_control_msg(udev,
					usb_sndctrlpipe(udev, 0),
					USB_REQ_SET_FEATURE, 0,
					USB_NTF_HOST_REL,
					0, NULL, 0, USB_CTRL_SET_TIMEOUT);
				if (err < 0) {
					dev_info(&udev->dev,
						 "can't release host on device: %d\n",
						 err);
					return -1;
				}
			}
		}
	}
	return 0;
}

static int xhci_dwc_drv_probe(struct platform_device *pdev)
{
	struct usb_phy *usb_phy = usb_get_transceiver();
	struct dwc_device_par *pdata;
	struct usb_hcd *hcd;
	struct resource *res;
	int retval = 0;
	dwcdev = pdev;

	if (usb_disabled())
		return -ENODEV;

	pr_debug("initializing FSL-SOC USB Controller\n");

	/* Need platform data for setup */
	pdata = (struct dwc_device_par *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", dev_name(&pdev->dev));
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	otg_irqnum = res->start;

	hcd = usb_create_hcd(&xhci_dwc_hc_driver,
			&pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		printk(KERN_ERR "usb_create_usb2 hcd failed!\n");
		return retval;
	}

	hcd->regs = pdata->io_addr;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start;

	if (usb_phy) {
		otg_set_host(usb_phy->otg, &hcd->self);
		usb_put_transceiver(usb_phy);
	}
	hcd->rpm_control = 1;
	hcd->rpm_resume = 0;

	platform_set_drvdata(pdev, hcd);
	pm_runtime_enable(hcd->self.controller);

	retval = device_create_file(hcd->self.controller, &dev_attr_disable_pm);
	if (retval < 0)
		printk(KERN_ERR
			"Can't register sysfs attribute: %d\n", retval);

	return retval;
}

static int xhci_dwc_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct usb_phy *usb_phy = usb_get_transceiver();
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	otg_set_host(usb_phy->otg, NULL);
	usb_put_transceiver(usb_phy);

	if (xhci)
		xhci_stop_host(hcd);
	usb_put_hcd(hcd);

	pm_runtime_disable(hcd->self.controller);
	pm_runtime_set_suspended(hcd->self.controller);
	return 0;
}


#ifdef CONFIG_PM

#ifdef CONFIG_PM_RUNTIME
/*
 * Do nothing in runtime pm callback.
 * On HVP platform, if make controller go to hibernation mode.
 * controller will not send IRQ until restore status which
 * implement in pm runtime resume callback. So there is no
 * any one can trigger pm_runtime_get to resume USB3 device.
 * This issue need to continue investigate. So just implement SW logic at here.
 */
static int dwc_hcd_runtime_idle(struct device *dev)
{
	return 0;
}

/* dwc_hcd_suspend_common and dwc_hcd_resume_common are refer to
 * suspend_common and resume_common in usb core.
 * Because the usb core function just support PCI device.
 * So re-write them in here to support platform devices.
 */
static int dwc_hcd_suspend_common(struct device *dev)
{
	struct platform_device		*pdev = to_platform_device(dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	int			retval = 0;
	u32 data = 0;

	if (!xhci) {
		dev_dbg(dev, "%s: host already stop!\n", __func__);
		return 0;
	}

	/* Root hub suspend should have stopped all downstream traffic,
	 * and all bus master traffic.  And done so for both the interface
	 * and the stub usb_device (which we check here).  But maybe it
	 * didn't; writing sysfs power/state files ignores such rules...
	 */
	if (HCD_RH_RUNNING(hcd)) {
		dev_warn(dev, "Root hub is not suspended\n");
		return -EBUSY;
	}
	if (hcd->shared_hcd) {
		hcd = hcd->shared_hcd;
		if (HCD_RH_RUNNING(hcd)) {
			dev_warn(dev, "Secondary root hub is not suspended\n");
			return -EBUSY;
		}
	}

	if (!HCD_DEAD(hcd)) {
		/* Optimization: Don't suspend if a root-hub wakeup is
		 * pending and it would cause the HCD to wake up anyway.
		 */
		if (HCD_WAKEUP_PENDING(hcd))
			return -EBUSY;
		if (hcd->shared_hcd &&
				HCD_WAKEUP_PENDING(hcd->shared_hcd))
			return -EBUSY;
		if (hcd->state != HC_STATE_SUSPENDED ||
				xhci->shared_hcd->state != HC_STATE_SUSPENDED)
			retval = -EINVAL;

		if (!retval) {
			/* Ensure that GUSB3PIPECTL[17] (Suspend SS PHY)
			 * is set to '1'
			 **/
			data = readl(hcd->regs + GUSB3PIPECTL0);
			if (!(data & GUSB3PIPECTL_SUS_EN)) {
				data |= GUSB3PIPECTL_SUS_EN;
				writel(data, hcd->regs + GUSB3PIPECTL0);
			}

			/* Ensure that GUSB2PHYCFG[6] (Suspend 2.0 PHY)
			 * is set to '1'
			 **/
			data = readl(hcd->regs + GUSB2PHYCFG0);
			if (!(data & GUSB2PHYCFG_SUS_PHY)) {
				data |= GUSB2PHYCFG_SUS_PHY;
				writel(data, hcd->regs + GUSB2PHYCFG0);
			}

			data = readl(hcd->regs + GCTL);
			data |= GCTL_GBL_HIBERNATION_EN;
			writel(data, hcd->regs + GCTL);
			printk(KERN_ERR "set xhci hibernation enable!\n");
			retval = xhci_suspend(xhci);
		}

		/* Check again in case wakeup raced with pci_suspend */
		if ((retval == 0 && HCD_WAKEUP_PENDING(hcd)) ||
				(retval == 0 && hcd->shared_hcd &&
				 HCD_WAKEUP_PENDING(hcd->shared_hcd))) {
			xhci_resume(xhci, false);
			retval = -EBUSY;
		}
		if (retval)
			return retval;
	}

	synchronize_irq(otg_irqnum);

	return retval;

}

static int dwc_hcd_resume_common(struct device *dev)
{
	struct platform_device		*pdev = to_platform_device(dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);
	struct xhci_hcd		*xhci = hcd_to_xhci(hcd);
	int			retval = 0;

	if (!xhci) {
		printk(KERN_ERR "%s: xHCI equal NULL, return\n", __func__);
		return 0;
	}

	if (HCD_RH_RUNNING(hcd) ||
			(hcd->shared_hcd &&
			 HCD_RH_RUNNING(hcd->shared_hcd))) {
		dev_dbg(dev, "can't resume, not suspended!\n");
		return 0;
	}

	if (!HCD_DEAD(hcd)) {
		retval = xhci_resume(xhci, false);
		if (retval) {
			dev_err(dev, "PCI post-resume error %d!\n", retval);
			if (hcd->shared_hcd)
				usb_hc_died(hcd->shared_hcd);
			usb_hc_died(hcd);
		}
	}

	dev_dbg(dev, "hcd_pci_runtime_resume: %d\n", retval);

	return retval;
}

static int dwc_hcd_runtime_suspend(struct device *dev)
{
	int retval;

	retval = dwc_hcd_suspend_common(dev);

	dev_dbg(dev, "hcd_pci_runtime_suspend: %d\n", retval);
	return retval;
}

static int dwc_hcd_runtime_resume(struct device *dev)
{
	struct platform_device		*pdev = to_platform_device(dev);
	struct usb_hcd		*hcd = platform_get_drvdata(pdev);
	int retval;

	retval = dwc_hcd_resume_common(dev);
	dev_dbg(dev, "hcd_pci_runtime_resume: %d\n", retval);

	if (hcd->rpm_control) {
		if (hcd->rpm_resume) {
			struct device		*rpm_dev = hcd->self.controller;
			hcd->rpm_resume = 0;
			pm_runtime_put(rpm_dev);
		}
	}
	return retval;
}
#else
#define dwc_hcd_runtime_idle NULL
#define dwc_hcd_runtime_suspend NULL
#define dwc_hcd_runtime_resume NULL
#endif


static int dwc_hcd_suspend(struct device *dev)
{
	int retval;

	retval = dwc_hcd_suspend_common(dev);

	dev_dbg(dev, "hcd_pci_runtime_suspend: %d\n", retval);
	return retval;
}

static int dwc_hcd_resume(struct device *dev)
{
	int retval;

	retval = dwc_hcd_resume_common(dev);
	dev_dbg(dev, "hcd_pci_runtime_resume: %d\n", retval);

	return retval;
}

static const struct dev_pm_ops dwc_usb_hcd_pm_ops = {
	.runtime_suspend = dwc_hcd_runtime_suspend,
	.runtime_resume	= dwc_hcd_runtime_resume,
	.runtime_idle	= dwc_hcd_runtime_idle,
	.suspend	=	dwc_hcd_suspend,
	.resume		=	dwc_hcd_resume,
};
#endif

static struct platform_driver xhci_dwc_driver = {
	.probe = xhci_dwc_drv_probe,
	.remove = xhci_dwc_drv_remove,
	.driver = {
		.name = "dwc3-host",
#ifdef CONFIG_PM
		.pm = &dwc_usb_hcd_pm_ops
#endif
	},
};
MODULE_ALIAS("platform:dwc-xhci");
