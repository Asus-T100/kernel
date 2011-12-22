/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $34 support for sensor
 * firmware reflashing.
 *
 * Copyright (c) 2007 - 2011, Synaptics Incorporated
 *
 */
/*
 * This file is licensed under the GPL2 license.
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/input.h>
#include <linux/sysfs.h>
#include <linux/math64.h>
#include "rmi_drvr.h"
#include "rmi_bus.h"
#include "rmi_sensor.h"
#include "rmi_function.h"
#include "rmi_f34.h"

/* data specific to fn $34 that needs to be kept around */
struct rmi_fn_34_data {
	unsigned char status;
	unsigned char cmd;
	unsigned short bootloaderid;
	unsigned short blocksize;
	unsigned short imageblockcount;
	unsigned short configblockcount;
};

static ssize_t rmi_fn_34_status_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_34_cmd_show(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_34_cmd_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

static ssize_t rmi_fn_34_data_read(struct file *data_file, struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count);

static ssize_t rmi_fn_34_data_write(struct file *data_file,
				    struct kobject *kobj,
				    struct bin_attribute *attributes, char *buf,
				    loff_t pos, size_t count);

static ssize_t rmi_fn_34_bootloaderid_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);

static ssize_t rmi_fn_34_bootloaderid_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count);

static ssize_t rmi_fn_34_blocksize_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_34_imageblockcount_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf);

static ssize_t rmi_fn_34_configblockcount_show(struct device *dev,
					       struct device_attribute *attr,
					       char *buf);

static struct device_attribute attrs[] = {
	__ATTR(status, 0444,
	       rmi_fn_34_status_show, rmi_store_error),	/* RO attr */
	/* Also, sysfs will need to have a file set up to distinguish
	 * between commands - like Config write/read, Image write/verify. */
	__ATTR(cmd, 0666,
	       rmi_fn_34_cmd_show, rmi_fn_34_cmd_store),	/* RW attr */
	__ATTR(bootloaderid, 0666,
	       rmi_fn_34_bootloaderid_show, rmi_fn_34_bootloaderid_store),
	       /* RW attr */
	__ATTR(blocksize, 0444,
	       rmi_fn_34_blocksize_show, rmi_store_error),	/* RO attr */
	__ATTR(imageblockcount, 0444,
	       rmi_fn_34_imageblockcount_show, rmi_store_error), /* RO attr */
	__ATTR(configblockcount, 0444,
	       rmi_fn_34_configblockcount_show, rmi_store_error) /* RO attr */
};

struct bin_attribute dev_attr_data = {
	.attr = {
		 .name = "data",
		 .mode = 0666},
	.size = 0,
	.read = rmi_fn_34_data_read,
	.write = rmi_fn_34_data_write,
};

/* Helper fn to convert a byte array representing a short in the RMI
 * endian-ness to a short in the native processor's specific endianness.
 * We don't use ntohs/htons here because, well, we're not dealing with
 * a pair of shorts. And casting dest to short* wouldn't work, because
 * that would imply knowing the byte order of short in the first place.
 */
static void batohs(unsigned short *dest, unsigned char *src)
{
	*src = dest[1] * 0x100 + dest[0];
}

/* Helper function to convert a short (in host processor endianess) to
 * a byte array in the RMI endianess for shorts.  See above comment for
 * why we dont us htons or something like that.
 */
static void hstoba(unsigned char *dest, unsigned short src)
{
	dest[0] = src % 0x100;
	dest[1] = src / 0x100;
}

/*.
 * The interrupt handler for Fn $34.
 */
void FN_34_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs)
{
	unsigned int status;
	struct rmi_fn_34_data *instance_data = rmifninfo->fndata;

	/* Read the Fn $34 status register to see whether the previous
	 * command executed OK. inform user space - through a sysfs param. */
	if (rmi_read_multiple(rmifninfo->sensor,
			rmifninfo->function_descriptor.data_base_addr + 3,
			(unsigned char *)&status, 1)) {
		pr_err("%s : Could not read status from 0x%x\n", __func__,
		       rmifninfo->function_descriptor.data_base_addr + 3);
		status = 0xff;	/* failure */
	}

	/* set a sysfs value that the user mode can read - only
	 * upper 4 bits are the status. successful is $80, anything
	 * else is failure */
	instance_data->status = status & 0xf0;
}
EXPORT_SYMBOL(FN_34_inthandler);

/* This is a stub for now, and will be fleshed out or removed as the
 * implementation matures.
 */
void FN_34_attention(struct rmi_function_info *rmifninfo)
{

}
EXPORT_SYMBOL(FN_34_attention);

/* This is a stub for now, and will be fleshed out or removed as the
 * implementation matures.
 */
int FN_34_config(struct rmi_function_info *rmifninfo)
{
	pr_debug("%s: RMI4 function $34 config\n", __func__);
	return 0;
}
EXPORT_SYMBOL(FN_34_config);

int FN_34_init(struct rmi_function_device *function_device)
{
	int retval = 0;
	int attr_count = 0;
	unsigned char buf[2];
	struct rmi_function_info *rmifninfo = function_device->rfi;
	struct rmi_fn_34_data *instance_data;

	pr_debug("%s: RMI4 function $34 init\n", __func__);

	if (rmifninfo->fndata) {
		/* detect routine should only ever be called once
		 * per rmifninfo. */
		pr_err("%s: WTF?!? F34 instance data is already present!",
		       __func__);
		return -EINVAL;
	}

	instance_data = kzalloc(sizeof(struct rmi_fn_34_data), GFP_KERNEL);
	if (!instance_data) {
		pr_err("%s: Error allocating memory for rmi_fn_34_data.\n",
		       __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	rmifninfo->fndata = instance_data;

	/* get the Bootloader ID and Block Size. */
	retval = rmi_read_multiple(rmifninfo->sensor,
		rmifninfo->function_descriptor.query_base_addr,
		buf, ARRAY_SIZE(buf));
	if (retval) {
		pr_err("%s : Could not read bootloaderid from 0x%04x\n",
		       __func__,
		       rmifninfo->function_descriptor.query_base_addr);
		goto error_exit;
	}
	batohs(&instance_data->bootloaderid, buf);

	retval = rmi_read_multiple(rmifninfo->sensor,
			rmifninfo->function_descriptor.query_base_addr + 3,
			buf, ARRAY_SIZE(buf));
	if (retval) {
		pr_err("%s: Could not read block size from 0x%04x\n",
		       __func__,
		       rmifninfo->function_descriptor.query_base_addr + 3);
		goto error_exit;
	}
	batohs(&instance_data->blocksize, buf);

	/* Get firmware image block count and store it in the instance data */
	retval = rmi_read_multiple(rmifninfo->sensor,
			rmifninfo->function_descriptor.query_base_addr + 5,
			buf, ARRAY_SIZE(buf));
	if (retval) {
		pr_err("%s: Could not read image block count from 0x%x\n",
		       __func__,
		       rmifninfo->function_descriptor.query_base_addr + 5);
		goto error_exit;
	}
	batohs(&instance_data->imageblockcount, buf);

	/* Get config block count and store it in the instance data */
	retval = rmi_read_multiple(rmifninfo->sensor,
			rmifninfo->function_descriptor.query_base_addr + 7,
			buf, ARRAY_SIZE(buf));
	if (retval) {
		pr_err("%s: Could not read config block count from 0x%x, "
			"error=%d.\n", __func__,
			rmifninfo->function_descriptor.query_base_addr + 7,
			retval);
		goto error_exit;
	}
	batohs(&instance_data->configblockcount, buf);

	/* We need a sysfs file for the image/config block to write or read.
	 * Set up sysfs bin file for binary data block. Since the image is
	 * already in our format there is no need to convert the data for
	 * endianess. */
	retval = sysfs_create_bin_file(&function_device->dev.kobj,
				&dev_attr_data);
	if (retval < 0) {
		pr_err
		    ("%s: Failed to create sysfs file for fn 34 data "
		     "(error = %d).\n", __func__, retval);
		retval = -ENODEV;
		goto error_exit;
	}

	pr_debug("%s: Creating sysfs files.", __func__);
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&function_device->dev.kobj, &attrs[attr_count].attr) < 0) {
			pr_err
			    ("%s: Failed to create sysfs file for  %s.",
			     __func__, attrs[attr_count].attr.name);
			retval = -ENODEV;
			goto error_exit;
		}
	}

	return retval;

error_exit:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&function_device->dev.kobj,
				  &attrs[attr_count].attr);
	kfree(instance_data);
	return retval;
}
EXPORT_SYMBOL(FN_34_init);

int FN_34_detect(struct rmi_function_info *rmifninfo)
{
	int retval = 0;

	pr_debug("%s: RMI4 function $34 detect\n", __func__);
	if (rmifninfo->sensor == NULL) {
		pr_err("%s: NULL sensor passed in!", __func__);
		return -EINVAL;
	}

	return retval;
}
EXPORT_SYMBOL(FN_34_detect);

static ssize_t rmi_fn_34_bootloaderid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->bootloaderid);
}

static ssize_t rmi_fn_34_bootloaderid_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf,
					size_t count)
{
	int error;
	unsigned long val;
	unsigned char data[2];
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->bootloaderid = val;

	/* Write the Bootloader ID key data back to the first two Block
	 * Data registers (F34_Flash_Data2.0 and F34_Flash_Data2.1). */
	hstoba(data, (unsigned short)val);
	error = rmi_write_multiple(fn->sensor,
				   fn->rfi->function_descriptor.data_base_addr,
				   data, ARRAY_SIZE(data));
	if (error) {
		dev_err(dev, "%s : Could not write bootloader id to 0x%x\n",
		       __func__, fn->rfi->function_descriptor.data_base_addr);
		return error;
	}

	return count;
}

static ssize_t rmi_fn_34_blocksize_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->blocksize);
}

static ssize_t rmi_fn_34_imageblockcount_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->imageblockcount);
}

static ssize_t rmi_fn_34_configblockcount_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->configblockcount);
}

static ssize_t rmi_fn_34_status_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->status);
}

static ssize_t rmi_fn_34_cmd_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->cmd);
}

static ssize_t rmi_fn_34_cmd_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;
	unsigned long val;
	unsigned short baseaddr = fn->rfi->function_descriptor.data_base_addr;
	int error;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	instance_data->cmd = val;

	/* Validate command value and (if necessary) write it to the command
	 * register.
	 */
	switch (instance_data->cmd) {
	case ENABLE_FLASH_PROG:
	case ERASE_ALL:
	case ERASE_CONFIG:
	case WRITE_FW_BLOCK:
	case READ_CONFIG_BLOCK:
	case WRITE_CONFIG_BLOCK:
		/* Issue a Write Firmware Block ($02) command to the Flash
		 * Command (F34_Flash_Data3, bits 3:0) field. */
		error = rmi_write_multiple(fn->sensor, baseaddr + 3,
					   &instance_data->cmd, 1);
		/* return one if succeeds */
		if (error != 1) {
			dev_err(dev, "%s: Could not write command 0x%02x "
				"to 0x%04x\n", __func__, instance_data->cmd,
				baseaddr + 3);
			return error;
		}
		break;
	default:
		dev_dbg(dev, "%s: RMI4 function $34 - "
				"unknown command 0x%02lx.\n", __func__, val);
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t rmi_fn_34_data_read(struct file *data_file,
				struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;
	int error;

	if (count != instance_data->blocksize) {
		dev_err(dev,
			"%s: Incorrect F34 block size %d. Expected size %d.\n",
			__func__, count, instance_data->blocksize);
		return -EINVAL;
	}

	/* Read the data from flash into buf.  The app layer will be blocked
	 * at reading from the sysfs file.  When we return the count (or
	 * error if we fail) the app will resume. */
	error = rmi_read_multiple(fn->sensor,
			fn->rfi->function_descriptor.data_base_addr + pos,
			(unsigned char *)buf, count);
	if (error) {
		dev_err(dev, "%s : Could not read data from 0x%llx\n",
			__func__,
			fn->rfi->function_descriptor.data_base_addr + pos);
		return error;
	}

	return count;
}

static ssize_t rmi_fn_34_data_write(struct file *data_file,
				struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_34_data *instance_data = fn->rfi->fndata;
	unsigned int blocknum;
	int error;
	unsigned int remainder;
	unsigned short baseaddr = fn->rfi->function_descriptor.data_base_addr;

	/* Write the data from buf to flash. The app layer will be
	 * blocked at writing to the sysfs file.  When we return the
	 * count (or error if we fail) the app will resume. */

	if (count != instance_data->blocksize) {
		dev_err(dev,
			"%s: Incorrect F34 block size %d. Expected size %d.\n",
			__func__, count, instance_data->blocksize);
		return -EINVAL;
	}

	/* Verify that the byte offset is always aligned on a block boundary
	 * and if not return an error.  We can't just use the mod operator %
	 * and do a (pos % instance_data->blocksize) because of a gcc
	 * bug that results in undefined symbols.  So we have to compute
	 * it the hard way.  Grumble. */
	div_u64_rem(pos, instance_data->blocksize, &remainder);
	if (remainder) {
		dev_err(dev,
			"%s: Invalid byte offset of %llx leads to invalid "
			"block number.\n", __func__, pos);
		return -EINVAL;
	}

	/* Compute the block number using the byte offset (pos) and the
	 * block size.  Once again, we can't just do a divide due to a
	 * gcc bug. */
	blocknum = div_u64(pos, instance_data->blocksize);

	/* Write the block number first */
	error = rmi_write_multiple(fn->sensor, baseaddr,
				   (unsigned char *)&blocknum, 2);
	/* return one if succeeds */
	if (error != 1) {
		dev_err(dev, "%s: Could not write block number to 0x%x\n",
			__func__, baseaddr);
		return error;
	}

	/* Write the data block - only if the count is non-zero  */
	if (count) {
		error = rmi_write_multiple(fn->sensor, baseaddr + 2,
					   (unsigned char *)buf, count);
		/* return one if succeeds */
		if (error != 1) {
			dev_err(dev, "%s: Could not write block data "
				"to 0x%x\n", __func__, baseaddr + 2);
			return error;
		}
	}

	return count;
}
