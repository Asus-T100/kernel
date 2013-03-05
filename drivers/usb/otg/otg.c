/*
 * otg.c -- USB OTG utility code
 *
 * Copyright (C) 2004 Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/device.h>

#include <linux/usb/otg.h>

static struct usb_phy *phy;

static ssize_t
a_bus_drop_show(struct device *_dev, struct device_attribute *attr, char *buf)
{
	unsigned size, len;
	unsigned char *str;
	struct usb_phy *_phy = phy;

	if (!_phy)
		return -1;

	switch (_phy->vbus_state) {
	case VBUS_DISABLED:
		str = "1\n";
		break;
	case VBUS_ENABLED:
		str = "0\n";
		break;
	case UNKNOW_STATE:
	default:
		str = "unkown\n";
		break;
	}

	size = PAGE_SIZE;

	len = strlen(str);
	strncpy(buf, str, len);
	buf[len + 1] = '\0';

	size -= len;

	return PAGE_SIZE - size;
}

static ssize_t a_bus_drop_store(struct device *_dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct usb_phy *_phy = phy;
	int trigger;

	if (!buf || !count)
		return -EINVAL;

	if (!strncmp(buf, "1", strlen("1"))) {
		trigger = 1;
		_phy->vbus_state = VBUS_DISABLED;
	} else if (!strncmp(buf, "0", strlen("0"))) {
		_phy->vbus_state = VBUS_ENABLED;
		trigger = 1;
	} else
		return -EINVAL;

	if (trigger && _phy->a_bus_drop)
		_phy->a_bus_drop(_phy);

	return count;
}

static DEVICE_ATTR(a_bus_drop, S_IRUGO|S_IWUSR|S_IWGRP,\
		a_bus_drop_show, a_bus_drop_store);

void otg_uevent_trigger(struct usb_phy *otg)
{
	char *uevent_envp[2] = { "USB_WARNING=HOST_NO_WORK", NULL };

	printk(KERN_INFO"%s: send uevent USB_OTG=HOST_NO_WORK\n", __func__);
	kobject_uevent_env(&otg->class_dev->kobj, KOBJ_CHANGE, uevent_envp);
}
EXPORT_SYMBOL(otg_uevent_trigger);

static struct device_attribute *otg_dev_attributes[] = {
	&dev_attr_a_bus_drop,
	NULL,
};


/**
 * usb_get_transceiver - find the (single) USB transceiver
 *
 * Returns the transceiver driver, after getting a refcount to it; or
 * null if there is no such transceiver.  The caller is responsible for
 * calling usb_put_transceiver() to release that count.
 *
 * For use by USB host and peripheral drivers.
 */
struct usb_phy *usb_get_transceiver(void)
{
	if (phy)
		get_device(phy->dev);
	return phy;
}
EXPORT_SYMBOL(usb_get_transceiver);

/**
 * usb_put_transceiver - release the (single) USB transceiver
 * @x: the transceiver returned by usb_get_transceiver()
 *
 * Releases a refcount the caller received from usb_get_transceiver().
 *
 * For use by USB host and peripheral drivers.
 */
void usb_put_transceiver(struct usb_phy *x)
{
	if (x)
		put_device(x->dev);
}
EXPORT_SYMBOL(usb_put_transceiver);

/**
 * usb_set_transceiver - declare the (single) USB transceiver
 * @x: the USB transceiver to be used; or NULL
 *
 * This call is exclusively for use by transceiver drivers, which
 * coordinate the activities of drivers for host and peripheral
 * controllers, and in some cases for VBUS current regulation.
 */
int usb_set_transceiver(struct usb_phy *x)
{
	struct device_attribute **attrs = otg_dev_attributes;
	struct device_attribute *attr;
	int err;

	if (!x)
		goto err1;

	if (phy && x)
		return -EBUSY;
	phy = x;

	x->usb_otg_class = class_create(NULL, "usb_otg");
	if (IS_ERR(x->usb_otg_class))
		return -EFAULT;

	x->class_dev = device_create(x->usb_otg_class, x->dev,
			MKDEV(0, 0), NULL, "otg0");
	if (IS_ERR(x->class_dev))
		goto err2;

	while ((attr = *attrs++)) {
		err = device_create_file(x->class_dev, attr);
		if (err) {
			device_destroy(x->usb_otg_class, x->class_dev->devt);
			return err;
		}
	}

	return 0;

err1:
	device_destroy(x->usb_otg_class, x->class_dev->devt);
err2:
	class_destroy(x->usb_otg_class);

	return -EFAULT;
}
EXPORT_SYMBOL(usb_set_transceiver);

const char *otg_state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:
		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:
		return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:
		return "a_wait_bcon";
	case OTG_STATE_A_HOST:
		return "a_host";
	case OTG_STATE_A_SUSPEND:
		return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:
		return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:
		return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:
		return "a_vbus_err";
	case OTG_STATE_B_IDLE:
		return "b_idle";
	case OTG_STATE_B_SRP_INIT:
		return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:
		return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:
		return "b_wait_acon";
	case OTG_STATE_B_HOST:
		return "b_host";
	default:
		return "UNDEFINED";
	}
}
EXPORT_SYMBOL(otg_state_string);
