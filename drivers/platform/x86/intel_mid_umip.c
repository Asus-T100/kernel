/*
 *  intel_mid_umip.c - Driver to modify the USB umip values
 *
 * Copyright (c) 2011, Intel Corporation.
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
 */

#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <asm/intel_scu_ipc.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>

#define USB_HOST_ENABLE_TIMEOUT_INFINITE    0x3F
#define USB_HOST_ENABLE_TIMEOUT_THREE       0x1B
#define USB_HOST_ENABLE_TIMEOUT_ZERO        0X00
#define USB_ENABLE_UMIP_OFFSET              0x400
#define MAX_USB_TIMEOUT_LEN                 14
#define MAX_NUM_TIMEOUTS                    3
#define FACTORY_UMIP_OFFSET                 0xE00
#define FACTORY_BIT_OFFSET                  0

static struct platform_device *umip_mid_pdev;

/*
	The "current Factory UMIP" shows the current value of
	the variable.
*/

static ssize_t Factory_UMIP_show(struct device *dev,
				struct device_attribute *attr, char *buffer)
{
	int ret;
	u8 data_read;

	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL) {
		ret = intel_scu_ipc_read_mip(&data_read,
						1,
						FACTORY_UMIP_OFFSET,
						0);
		if (ret) {
			pr_err("Could not read to UMIP for Factory\n");
			return -EBUSY;
		}
	}

	return sprintf(buffer, "%d\n",
				(data_read >> FACTORY_BIT_OFFSET) & 0x01);
}

ssize_t Factory_UMIP_store(struct device *dev,
					struct device_attribute *attr,
					const char *buffer, size_t count)
{
	int ret = 0;
	u8 data_write;
	u8 yes;
	u8 no;
	bool bv;

	if (strlen(buffer) != 2) {
		pr_err("The length must be 1\n");
		ret = -EINVAL;
		goto error;
	}

	ret = strtobool(buffer, &bv);
	if (ret) {
		pr_err("Not expected value [Y|y|1|N|n|0]\n");
		goto error;
	}

	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_PENWELL) {
		ret = intel_scu_ipc_read_mip(&data_write,
						1,
						FACTORY_UMIP_OFFSET,
						0);
		data_write &= ~(1 << FACTORY_BIT_OFFSET);
		data_write |= bv;

		ret = intel_scu_ipc_write_umip(&data_write, 1, FACTORY_UMIP_OFFSET);
		if (ret) {
			pr_err("Could not write to UMIP for Factory\n");
			goto error;
		}
	}

	return count;

error:
	return ret;
}

struct usb_timeout {
	char name[MAX_USB_TIMEOUT_LEN];
	u8 umip_value;
};

static struct
	usb_timeout available_usb_host_enable_timeouts[MAX_NUM_TIMEOUTS] = {
	 {"infinite", USB_HOST_ENABLE_TIMEOUT_INFINITE},
	 {"3seconds", USB_HOST_ENABLE_TIMEOUT_THREE},
	 {"zeroseconds", USB_HOST_ENABLE_TIMEOUT_ZERO}
};

/*
	 The "availabe usb host enable timeouts" file where a static variable
	 is read from and written to.
*/
static ssize_t available_usb_host_enable_timeouts_show(struct device *dev,
				struct device_attribute *attr, char *buffer)
{
	return sprintf(buffer, "%s\t%s\t%s\n",
				available_usb_host_enable_timeouts[0].name,
				available_usb_host_enable_timeouts[1].name,
				available_usb_host_enable_timeouts[2].name);
}

/*
	The "current usb host enable timeout" shows the current value of
	the variable.
*/
static ssize_t usb_host_enable_timeout_show(struct device *dev,
				struct device_attribute *attr, char *buffer)
{
	int ret_ipc;
	u8 data_read;
	int i = 0;
	char timeout[MAX_USB_TIMEOUT_LEN];

	ret_ipc = intel_scu_ipc_read_mip(&data_read,
					1,
					USB_ENABLE_UMIP_OFFSET,
					0);
	if (ret_ipc)
		pr_err("Could not read to UMIP USB Host Enable\n");

	for (i = 0; i < MAX_NUM_TIMEOUTS; i++) {
		if (data_read ==
			available_usb_host_enable_timeouts[i].umip_value) {
			strcpy(timeout,
				available_usb_host_enable_timeouts[i].name);
			break;
		}
	}

	if (i == MAX_NUM_TIMEOUTS)
		return sprintf(buffer, "%s\n", "Could not read right value");
	else
		return snprintf(buffer, sizeof(timeout), "%s\n", timeout);
}

ssize_t usb_host_enable_timeout_store(struct device *dev,
					struct device_attribute *attr,
					const char *buffer, size_t count)
{
	int ret_ipc, i, string_length;
	char timeout[MAX_USB_TIMEOUT_LEN];

	string_length = strlen(buffer);

	if (string_length < MAX_USB_TIMEOUT_LEN) {
		snprintf(timeout, sizeof(timeout), "%s", buffer);
	} else {
		pr_err("Invalid value written."
		"Check the Availabe values that can be used\n");
		return count;
	}

	for (i = 0; i < MAX_NUM_TIMEOUTS; i++) {
		if (strcmp(timeout, available_usb_host_enable_timeouts[i].name))
			continue;

		ret_ipc = intel_scu_ipc_write_umip(
			&available_usb_host_enable_timeouts[i].umip_value,
			1, USB_ENABLE_UMIP_OFFSET);
		if (ret_ipc)
			pr_err("Could not write to UMIP USB Host Enable\n");

		break;
	}

	/* if i is equal to arr_len, that means there is no match */
	if (i == MAX_NUM_TIMEOUTS) {
		pr_err("Invalid value written."
			"Check the Availabe values that can be used\n");
	}

	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR(Factory_UMIP, S_IRUGO|S_IWUSR,
		Factory_UMIP_show, Factory_UMIP_store);
DEVICE_ATTR(available_timeouts, S_IRUGO,
		available_usb_host_enable_timeouts_show, NULL);
DEVICE_ATTR(current_timeout, S_IRUGO|S_IWUSR,
		 usb_host_enable_timeout_show, usb_host_enable_timeout_store);

/* Attribute Descriptor */
static struct attribute *umip_mid_attrs[] = {
		&dev_attr_Factory_UMIP.attr,
		&dev_attr_available_timeouts.attr,
		&dev_attr_current_timeout.attr,
		NULL
};

/* Attribute group */
static struct attribute_group umip_attrs_group = {
	.attrs = umip_mid_attrs,
};

static int __init intel_mid_umip_init(void)
{
	int retval;

	/* Register a platform device */
	umip_mid_pdev = platform_device_register_simple("intel_mid_umip", -1,
								NULL, 0);
	if (IS_ERR(umip_mid_pdev)) {
		pr_err("UMIP_MID: platform_device_register_simple error\n");
		return PTR_ERR(umip_mid_pdev);
	}

	/* Create a sysfs node to read simulated coordinates */
	retval = sysfs_create_group(&umip_mid_pdev->dev.kobj,
					&umip_attrs_group);
	if (retval) {
		pr_err("UMIP_MID: sysfs create group error\n");
		platform_device_unregister(umip_mid_pdev);
		umip_mid_pdev = NULL;
		return retval;
	}

	return 0;
}

static void __exit intel_mid_umip_exit(void)
{
	sysfs_remove_group(&umip_mid_pdev->dev.kobj, &umip_attrs_group);
	platform_device_unregister(umip_mid_pdev);
}

module_init(intel_mid_umip_init);
module_exit(intel_mid_umip_exit);

MODULE_DESCRIPTION("intel mid usb umip programming");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Surya P Sahukar <surya.p.sahukar@intel.com");
