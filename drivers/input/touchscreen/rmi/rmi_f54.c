/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $54 support for direct
 * access to low-level capacitance data.
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
#include "rmi_f54.h"

/* modified from F 34, must check for copy-paste nonsense */


static ssize_t rmi_fn_54_reporttype_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_reporttype_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

static ssize_t rmi_fn_54_cmd_show(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_cmd_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

static ssize_t rmi_fn_54_status_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_data_read(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *attributes,
				   char *buf, loff_t pos, size_t count);

static ssize_t rmi_fn_54_numrxelectrodes_show(struct device *dev,
				     struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_numtxelectrodes_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_control_show(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_control_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count);

static ssize_t rmi_fn_54_fifoindexlo_show(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_fifoindexlo_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

static ssize_t rmi_fn_54_fifoindexhi_show(struct device *dev,
				  struct device_attribute *attr, char *buf);

static ssize_t rmi_fn_54_fifoindexhi_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count);

/* Should probably change name here to something more appropriate */
static struct device_attribute attrs[] = {
	__ATTR(reporttype, 0666,
		rmi_fn_54_reporttype_show, rmi_fn_54_reporttype_store),
	__ATTR(cmd, 0666,
		rmi_fn_54_cmd_show, rmi_fn_54_cmd_store),
	__ATTR(status, 0444,
		rmi_fn_54_status_show, rmi_store_error),
	__ATTR(numrxelectrodes, 0444,
		rmi_fn_54_numrxelectrodes_show, rmi_store_error),
	__ATTR(numtxelectrodes, 0444,
		rmi_fn_54_numtxelectrodes_show, rmi_store_error),
	__ATTR(control, 0666,
		rmi_fn_54_control_show, rmi_fn_54_control_store),
	__ATTR(fifoindexlo, 0666,
		rmi_fn_54_fifoindexlo_show, rmi_fn_54_fifoindexlo_store),
	__ATTR(fifoindexhi, 0666,
		rmi_fn_54_fifoindexhi_show, rmi_fn_54_fifoindexhi_store)
};
/* Same as above */
struct bin_attribute dev_rep_data = {
	.attr = {
		 .name = "repdata",
		 .mode = 0444},
	.size = 0,
	.read = rmi_fn_54_data_read,
};


int FN_54_init(struct rmi_function_device *function_device)
{
	int retval = 0;
	int attr_count = 0;
	unsigned char buf;
	struct rmi_function_info *rmifninfo = function_device->rfi;
	struct rmi_fn_54_data *instance_data;

	pr_debug("%s: RMI4 function $54 init\n", __func__);

	if (rmifninfo->fndata) {
		/* detect routine should only ever be called once
		 * per rmifninfo. */
		pr_err("%s: WTF?!? F54 instance data is already present!",
		       __func__);
		return -EINVAL;
	}

	instance_data = kzalloc(sizeof(struct rmi_fn_54_data), GFP_KERNEL);
	if (!instance_data) {
		pr_err("%s: Error allocating memory for rmi_fn_54_data.\n",
		       __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	rmifninfo->fndata = instance_data;

	/* get the Number of Receiver and Transmitter Electrodes. */
	retval = rmi_read_multiple(rmifninfo->sensor,
		rmifninfo->function_descriptor.query_base_addr,
		&buf, 1);
	if (retval) {
		pr_err("%s : Could not read NumberOfReceiverElectrodes from 0x%04x\n",
		       __func__,
		       rmifninfo->function_descriptor.query_base_addr);
		goto error_exit;
	}
	instance_data->numrxelectrodes = buf;

	retval = rmi_read_multiple(rmifninfo->sensor,
			rmifninfo->function_descriptor.query_base_addr + 1,
			&buf, 1);
	if (retval) {
		pr_err("%s: Could not read NumberOfTransmitterElectrodes from 0x%04x\n",
		       __func__,
		       rmifninfo->function_descriptor.query_base_addr + 1);
		goto error_exit;
	}
	instance_data->numtxelectrodes = buf;

	__mutex_init(&instance_data->data_mutex, "data_mutex",
		     &instance_data->data_key);

	/* We need a sysfs file for the image/config block to write or read.
	 * Set up sysfs bin file for binary data block. Since the image is
	 * already in our format there is no need to convert the data for
	 * endianess. */
	retval = sysfs_create_bin_file(&function_device->dev.kobj,
				&dev_rep_data);
	if (retval < 0) {
		pr_err
		    ("%s: Failed to create sysfs file for fn 54 data "
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
EXPORT_SYMBOL(FN_54_init);


void FN_54_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs)
{

	struct rmi_fn_54_data *instance_data = rmifninfo->fndata;
	int error = 0;
	char *to_write;

	/* TODO: do something to disable all other interupts : ? */
	/* TODO: Need to do something for timeout? */

	instance_data->status = 1; /* busy */

	switch (instance_data->reporttype) {
	case r8bit_image:
		instance_data->reportsize = instance_data->numrxelectrodes *
					instance_data->numtxelectrodes;
		break;
	case r16bit_image:
	case autoscan:
		instance_data->reportsize = 2 * instance_data->numrxelectrodes *
					instance_data->numtxelectrodes;
		break;
	case trans2trans:
	case trans2trans_short:
		instance_data->reportsize = instance_data->numtxelectrodes *
					instance_data->numtxelectrodes;
		break;
	case rec2rec:
		instance_data->reportsize = instance_data->numrxelectrodes *
					instance_data->numrxelectrodes;
		break;
	default:
		instance_data->reportsize = 0;
		pr_err("%s:Invalid report type. This should never happen.\n",
			__func__);
	}
	/* We need to ensure the buffer is big enough.
	 * A Buffer size of 0 means that the buffer has not been allocated.
	 */
	if (instance_data->bufsize < instance_data->reportsize) {
		mutex_lock(&instance_data->data_mutex);
		if (instance_data->bufsize > 0)
			kfree(instance_data->report_data);

		instance_data->report_data = kzalloc(instance_data->reportsize,
						     GFP_KERNEL);
		if (!instance_data->report_data) {
			pr_err("%s: Error allocating memory for report data.\n",
			       __func__);
			instance_data->status = ENOMEM;
			instance_data->bufsize = 0;
			mutex_unlock(&instance_data->data_mutex);
			return;
		}
		instance_data->bufsize = instance_data->reportsize;
		mutex_unlock(&instance_data->data_mutex);
	}
	if (!instance_data->report_data) {
		pr_err("%s: Error allocating memory for f54 report_data.\n",
		       __func__);
		instance_data->status = ENOMEM;
		return;
	}
	pr_err("%s: Stuff is actually running.\n\n Size : %d\n",
		       __func__, instance_data->reportsize);
	/* Loop requesting data until we reach the end. */

	to_write = instance_data->report_data;


	error = rmi_read_multiple(rmifninfo->sensor,
		rmifninfo->function_descriptor.data_base_addr + 3,
		instance_data->report_data,
		instance_data->reportsize);
	if (error)
		pr_err("%s: ERROR F54 data read failed. "
			"Code: %d.\n", __func__, error);

	/*for (i = 0; i < instance_data->reportsize; i++) {
		// attempt to write FIFOData to the next space in the data file.
		error = rmi_read_multiple(rmifninfo->sensor,
			rmifninfo->function_descriptor.data_base_addr + 3,
			to_write, 1);
		*to_write = 'a';
		pr_info("%d: %c",i, *to_write);
		if ( error ) {
			pr_err("%s: Error while reading data: Error %d\n",
			       __func__, error);
			instance_data->status = error;
			// Do some sort of error handling?
		}
		to_write++;
	}*/
	if (!error)
		instance_data->status = 0; /* Success */
	else
		instance_data->status = error;
}
EXPORT_SYMBOL(FN_54_inthandler);

/* This is a stub for now, and will be fleshed out or removed as the
 * implementation matures.
 */
int FN_54_config(struct rmi_function_info *rmifninfo)
{
	pr_debug("%s: RMI4 function $54 config\n", __func__);
	return 0;
}
EXPORT_SYMBOL(FN_54_config);

int FN_54_detect(struct rmi_function_info *rmifninfo)
{
	int retval = 0;

	pr_debug("%s: RMI4 function $54 detect\n", __func__);
	if (rmifninfo->sensor == NULL) {
		pr_err("%s: NULL sensor passed in!", __func__);
		return -EINVAL;
	}

	return retval;
}
EXPORT_SYMBOL(FN_54_detect);

/* SYSFS file show/store functions */
static ssize_t rmi_fn_54_reporttype_show(struct device *dev,
				struct device_attribute *attr, char *buf) {
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->reporttype);
}

static ssize_t rmi_fn_54_reporttype_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count) {
	int error, result;
	unsigned long val;
	unsigned char data;
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->reporttype = (enum Report)val;
	data = (char)val;
	/* Write the Report Type back to the first Block
	 * Data registers (F54_AD_Data0). */
	result = rmi_write_multiple(fn->sensor,
			fn->rfi->function_descriptor.data_base_addr, &data, 1);
	if (result != count) {
		if (result < 0) {
			dev_err(dev, "%s : Could not write report type "
				"to 0x%x\n", __func__,
				fn->rfi->function_descriptor.data_base_addr);
		} else {
			dev_err(dev, "%s : Unexpected number of lines written "
				"to 0x%x\n", __func__,
				fn->rfi->function_descriptor.data_base_addr);
			dev_err(dev, "Wrote: %d\tExpected:%d\n", result, count);
		}
	}
	return result;
}

static ssize_t rmi_fn_54_cmd_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->cmd);
}

static ssize_t rmi_fn_54_cmd_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;
	u16 baseaddr = fn->rfi->function_descriptor.command_base_addr;
	unsigned long val;
	int error, result;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);
	if (error)
		return error;

	instance_data->cmd = (unsigned char)val;

	/* Validate command value and (if necessary) write it to the command
	 * register.
	 */
	switch (instance_data->cmd) {
	case GET_REPORT:
		/* This starts the report getting process. We calculate the
		 * expcected file size based on the report type, number of
		 * transmitters and receivers, and eventually the fifoindexlo
		 * and fifoindexhi. Currently, the firmware assumes those are
		 * 0 when it gets reports, and I'm not sure exactly how they're
		 * going to eventually effect report sizes, so they're ignored
		 * for now.
		 */
		switch (instance_data->reporttype) {
		case r8bit_image:
		case r16bit_image:
		case autoscan:
		case trans2trans:
		case trans2trans_short:
		case rec2rec:
			break;
		case trans_open:
		case rec_open:
		case high_resistance:
			dev_err(dev, "%s : Report type unimplemented\n",
				__func__);
			return -EINVAL;
			break;
		default:
			dev_err(dev, "%s : Report type invalid\n", __func__);
			return -EINVAL;
		}
	case FORCE_CAL:
		/* Write the command to the command register */
		result = rmi_write_multiple(fn->sensor, baseaddr,
					    &instance_data->cmd, 1);
		if (result != count) {
			if (result < 0) {
				dev_err(dev, "%s : Could not write command "
					"to 0x%x\n", __func__, baseaddr);
			} else {
				dev_err(dev, "%s : Unexpected number of lines "
					"written to 0x%x\n",
					__func__, baseaddr);
				dev_err(dev, "Wrote: %d\tExpected:%d\n",
					result, count);
			}
		}
		break;
	default:
		dev_dbg(dev, "%s: RMI4 function $54 - "
				"unknown command 0x%02lx.\n", __func__, val);
		count = -EINVAL;
		break;
	}

	return count;
}

static ssize_t rmi_fn_54_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->status);
}

static ssize_t rmi_fn_54_numrxelectrodes_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->numrxelectrodes);
}

static ssize_t rmi_fn_54_numtxelectrodes_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->numtxelectrodes);
}

static ssize_t rmi_fn_54_control_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	return snprintf(buf, PAGE_SIZE, "%u\n",
			instance_data->no_auto_cal ? 1 : 0);
}

static ssize_t rmi_fn_54_control_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int error, result;
	unsigned long val;
	unsigned char data;
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;
	u16 ctrlbase = fn->rfi->function_descriptor.control_base_addr;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	/* Write the control back to the control register (F54_AD_Ctrl0)
	 * Ignores everything but bit 0 */
	data = (unsigned char)(val & 0x01); /* bit mask for lowest bit */
	instance_data->no_auto_cal = (data == (unsigned char)0x01);
	result = rmi_write_multiple(fn->sensor, ctrlbase, &data, 1);
	if (result != count) {
		if (result < 0) {
			dev_err(dev, "%s : Could not write control to 0x%x\n",
			       __func__, ctrlbase);
		} else {
			dev_err(dev, "%s : Unexpected number of lines written "
				"to 0x%x\n", __func__, ctrlbase);
			dev_err(dev, "Wrote: %d\tExpected:%d\n", result, count);
		}
	}
	return result;
}

static ssize_t rmi_fn_54_fifoindexlo_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	/* Might want to read from device */
	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->fifoindexlo);
}

static ssize_t rmi_fn_54_fifoindexlo_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int error, result;
	unsigned long val;
	unsigned char data;
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;
	u16 database = fn->rfi->function_descriptor.data_base_addr;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error;

	instance_data->fifoindexlo = (unsigned char)val;

	/* Write the FIFOIndexLo back to the
	 * Data register (F54_AD_Data1). */
	/* hstoba(data, (unsigned short)val); Don't need this */
	data = (unsigned char)val;
	result = rmi_write_multiple(fn->sensor, database + 1, &data, 1);
	if (result != count) {
		if (result < 0) {
			dev_err(dev, "%s : Could not write FIFOIndexLo "
				"to 0x%x\n", __func__, database + 1);
		} else {
			dev_err(dev, "%s : Unexpected number of lines written "
				"to 0x%x\n", __func__, database + 1);
			dev_err(dev, "Wrote: %d\tExpected:%d\n", result, count);
		}
	}
	return result;
}

static ssize_t rmi_fn_54_fifoindexhi_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	/* Might want to read from device */
	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->fifoindexhi);
}

static ssize_t rmi_fn_54_fifoindexhi_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int error, result;
	unsigned long val;
	unsigned char data;
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;
	u16 database = fn->rfi->function_descriptor.data_base_addr;

	/* need to convert the string data to an actual value */
	error = strict_strtoul(buf, 10, &val);

	if (error)
		return error; /* Should this be -error? */
	instance_data->fifoindexhi = (unsigned char)val;

	/* Write the FIFOIndexLo back to the
	 * Data register (F54_AD_Data2). */
	/* hstoba(data, (unsigned short)val); Don't need this */
	data = (unsigned char)val;
	result =
	    rmi_write_multiple(fn->sensor, database + 2, &data, 1);
	if (result != count) {
		if (result < 0) {
			dev_err(dev, "%s : Could not write FIFOIndexHi to "
				"0x%x\n", __func__, database + 2);
		} else {
			dev_err(dev, "%s : Unexpected number of lines written "
				" to 0x%x\n", __func__, database + 2);
			dev_err(dev, "Wrote: %d\tExpected:%d\n", result, count);
		}
	}
	return result;
}

/* Provide access to last report */

static ssize_t rmi_fn_54_data_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *attributes,
				char *buf,
				loff_t pos,
				size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct rmi_fn_54_data *instance_data = fn->rfi->fndata;

	dev_dbg(dev, "%s: Trying to read data. count: %d pos: %ld\n",
		__func__, count, (long)pos);
	/* May run into issues here is a new report type is set before reading
	 * the old data. MAy need to keep track of 2. */
	if (count < instance_data->reportsize) {
		dev_err(dev,
			"%s: Incorrect F54 report size %d. Expected size %d.\n",
			__func__, count, instance_data->reportsize);
		return -EINVAL;
	}

	/* Copy data from instance_data to buffer */
	mutex_lock(&instance_data->data_mutex);
	memcpy(buf, instance_data->report_data, instance_data->reportsize);
	mutex_unlock(&instance_data->data_mutex);
	dev_info(dev, "%s: Presumably successful.", __func__);

	return instance_data->reportsize;
}
