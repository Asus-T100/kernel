/*
 * Intel Penwell USB OTG Test driver
 * Copyright (C) 2011, Intel Corporation.
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
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

/* This driver supports EHCI Host Test Mode initiated by test device with
 * VID 0x1a0a for Penwell OTG controller.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/module.h>

#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include "../core/usb.h"

struct pnwotg_test_dev {
	struct usb_device	*udev;
	struct usb_hcd		*hcd;

#define TBUF_SIZE	256
	u8			*buf;
};

static int
pnwotg_test_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct pnwotg_test_dev	*dev;
	int retval;

	dev_dbg(&intf->dev, "Penwell OTG test mode is initiated.\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->buf = kmalloc(TBUF_SIZE, GFP_KERNEL);
	if (!dev->buf) {
		kfree(dev);
		return -ENOMEM;
	}

	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	dev->hcd = usb_get_hcd(bus_to_hcd(dev->udev->bus));
	usb_set_intfdata(intf, dev);

	dev_dbg(&intf->dev, "test mode PID 0x%04x\n",
		le16_to_cpu(dev->udev->descriptor.idProduct));
	switch (le16_to_cpu(dev->udev->descriptor.idProduct)) {
	case 0x0101:
		/* TEST_SE0_NAK */
		dev->hcd->driver->hub_control(dev->hcd, SetPortFeature,
			USB_PORT_FEAT_TEST, 0x301, NULL, 0);
		break;
	case 0x0102:
		/* TEST_J */
		dev->hcd->driver->hub_control(dev->hcd, SetPortFeature,
			USB_PORT_FEAT_TEST, 0x101, NULL, 0);
		break;
	case 0x0103:
		/* TEST_K */
		dev->hcd->driver->hub_control(dev->hcd, SetPortFeature,
			USB_PORT_FEAT_TEST, 0x201, NULL, 0);
		break;
	case 0x0104:
		/* TEST_PACKET */
		dev->hcd->driver->hub_control(dev->hcd, SetPortFeature,
			USB_PORT_FEAT_TEST, 0x401, NULL, 0);
		break;
	case 0x0106:
		/* HS_HOST_PORT_SUSPEND_RESUME */
		msleep(15000);
		dev->hcd->driver->hub_control(dev->hcd, SetPortFeature,
			USB_PORT_FEAT_SUSPEND, 1, NULL, 0);
		msleep(15000);
		dev->hcd->driver->hub_control(dev->hcd, ClearPortFeature,
			USB_PORT_FEAT_SUSPEND, 1, NULL, 0);
		break;
	case 0x0107:
		/* SINGLE_STEP_GET_DEV_DESC */
		msleep(15000);
		retval = usb_control_msg(dev->udev,
				usb_rcvctrlpipe(dev->udev, 0),
				USB_REQ_GET_DESCRIPTOR,
				USB_DIR_IN | USB_RECIP_DEVICE,
				cpu_to_le16(USB_DT_DEVICE << 8),
				0, dev->buf,
				USB_DT_DEVICE_SIZE,
				USB_CTRL_GET_TIMEOUT);
		break;
	case 0x0108:
		/* SINGLE_STEP_SET_FEATURE */

		/* FIXME */
		/* set size = 0 to ignore DATA phase */
		retval = usb_control_msg(dev->udev,
				usb_rcvctrlpipe(dev->udev, 0),
				USB_REQ_GET_DESCRIPTOR,
				USB_DIR_IN | USB_RECIP_DEVICE,
				cpu_to_le16(USB_DT_DEVICE << 8),
				0, dev->buf, 0,
				USB_CTRL_GET_TIMEOUT);
		msleep(15000);
		retval = usb_control_msg(dev->udev,
				usb_rcvctrlpipe(dev->udev, 0),
				USB_REQ_GET_DESCRIPTOR,
				USB_DIR_IN | USB_RECIP_DEVICE,
				cpu_to_le16(USB_DT_DEVICE << 8),
				0, dev->buf,
				USB_DT_DEVICE_SIZE,
				USB_CTRL_GET_TIMEOUT);
		break;
	default:
		dev_info(&intf->dev, "unknown test mode with PID 0x%04x",
			id->idProduct);
		usb_notify_warning(dev->udev, USB_WARNING_NOT_SUPPORT);
	}

	return 0;
}

static void pnwotg_test_disconnect(struct usb_interface *intf)
{
	struct pnwotg_test_dev	*dev = usb_get_intfdata(intf);

	usb_put_hcd(dev->hcd);
	usb_put_dev(dev->udev);
	usb_set_intfdata(intf, NULL);
	dev_dbg(&intf->dev, "disconnect\n");
	kfree(dev->buf);
	kfree(dev);
}

static const struct usb_device_id id_table[] = {
	/* OTG Test Device */
	{ .match_flags = USB_DEVICE_ID_MATCH_VENDOR,
		.idVendor = 0x1A0A,
		},

	{ }
};
MODULE_DEVICE_TABLE(usb, id_table);

static struct usb_driver pnwotg_test_driver = {
	.name =		"pnwotg_test",
	.id_table =	id_table,
	.probe =	pnwotg_test_probe,
	.disconnect =	pnwotg_test_disconnect,
};

/*-------------------------------------------------------------------------*/

static int __init pnwotg_test_init(void)
{
	int result;

	result = usb_register(&pnwotg_test_driver);
	if (result)
		err("usb_register failed. error number %d", result);

	return result;
}
module_init(pnwotg_test_init);

static void __exit pnwotg_test_exit(void)
{
	usb_deregister(&pnwotg_test_driver);
}
module_exit(pnwotg_test_exit);

MODULE_DESCRIPTION("PENWELL USB OTG Testing Driver");
MODULE_LICENSE("GPL");
