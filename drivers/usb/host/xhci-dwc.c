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
#include <linux/dma-mapping.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/usb/otg.h>
#include <linux/usb/dwc_otg3.h>
#include <linux/platform_device.h>

static struct platform_device *dwcdev;
static int otg_irqnum;

#define SRAM_PHY_ADDR 0xFFFC0000
#define SRAM_LENGTH 0x20000
static int xhci_start_host(struct usb_hcd *hcd);
static int xhci_stop_host(struct usb_hcd *hcd);
static int xhci_reset_port(struct usb_hcd *hcd);
static int xhci_stop_host(struct usb_hcd *hcd);
static int dwc_host_setup(struct usb_hcd *hcd);
static int xhci_release_host(struct usb_hcd *hcd);

static const struct hc_driver xhci_dwc_hc_driver = {
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
	.bus_suspend =		xhci_bus_suspend,
	.bus_resume =		xhci_bus_resume,

	.start_host = xhci_start_host,
	.stop_host = xhci_stop_host,
	.reset_port = xhci_reset_port,
	.release_host = xhci_release_host,
};



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

	return 0;

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
	struct dwc_device_par *pdata;
	struct usb_hcd *hcd;
	struct resource *res;
	int retval = 0;
	struct otg_transceiver *otg = otg_get_transceiver();
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

	if (otg) {
		otg_set_host(otg, &hcd->self);
		otg_put_transceiver(otg);
	}

	return retval;
}

static int xhci_dwc_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct otg_transceiver *otg = otg_get_transceiver();
	printk(KERN_DEBUG "OTG: Removing host on otg=%p\n", otg);
	otg_set_host(otg, NULL);
	otg_put_transceiver(otg);

	usb_put_hcd(hcd);
	return 0;
}


static struct platform_driver xhci_dwc_driver = {
	.probe = xhci_dwc_drv_probe,
	.remove = xhci_dwc_drv_remove,
	.shutdown = usb_hcd_platform_shutdown,
	.driver = {
		.name = "dwc3-host",
	},
};
MODULE_ALIAS("platform:dwc-xhci");
