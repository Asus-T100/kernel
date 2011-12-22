/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $01 support for sensor
 * control and configuration.
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
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/param.h>
#include <linux/stat.h>
#include <linux/rmi_platformdata.h>

#include "rmi.h"
#include "rmi_drvr.h"
#include "rmi_bus.h"
#include "rmi_sensor.h"
#include "rmi_function.h"
#include "rmi_f01.h"

/* Query register field positions. */
#define F01_MFG_ID_POS 0
#define F01_PROPERTIES_POS 1
#define F01_PRODUCT_INFO_POS 2
#define F01_DATE_CODE_POS 4
#define F01_TESTER_ID_POS 7
#define F01_SERIAL_NUMBER_POS 9
#define F01_PRODUCT_ID_POS 11
#define F01_DATE_CODE_YEAR 0
#define F01_DATE_CODE_MONTH 1
#define F01_DATE_CODE_DAY 2

/* Control register bits. */
#define F01_CONFIGURED (1 << 7)
#define NONSTANDARD_REPORT_RATE (1 << 6)

/* Command register bits. */
#define F01_RESET 1
#define F01_SHUTDOWN (1 << 1)
#define F01_INITREFLASH (1 << 7)

/* Data register 0 bits. */
#define F01_UNCONFIGURED (1 << 7)
#define F01_FLASH_PROGRAMMING_MODE (1 << 6)
#define F01_STATUS_MASK 0x0F

/** Context data for each F01 we find.
 */
struct f01_instance_data {
	struct rmi_F01_control *control_registers;
	struct rmi_F01_data *data_registers;
	struct rmi_F01_query *query_registers;

	bool nonstandard_report_rate;
	/* original mode before suspend */
	unsigned char original_sleepmode;
	/* original no sleep setting */
	unsigned char original_nosleep;
};
static void set_sensor_sleepmode(struct rmi_function_info *functionInfo,
			unsigned char sleepmode, unsigned char nosleep);

static ssize_t rmi_fn_01_productinfo_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf);

static ssize_t rmi_fn_01_productid_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_01_manufacturer_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);

static ssize_t rmi_fn_01_datecode_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_fn_01_reportrate_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf);

static ssize_t rmi_fn_01_reportrate_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count);

static ssize_t rmi_fn_01_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count);

static ssize_t rmi_fn_01_testerid_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf);

static ssize_t rmi_fn_01_serialnumber_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf);

static ssize_t rmi_fn_01_sleepmode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_01_sleepmode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);
static ssize_t rmi_fn_01_nosleep_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_01_nosleep_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static ssize_t rmi_fn_01_initreflash_show(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static ssize_t rmi_fn_01_initreflash_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count);

static struct device_attribute attrs[] = {
	__ATTR(productinfo, 0444,
	       rmi_fn_01_productinfo_show, rmi_store_error),	/* RO attr */
	__ATTR(productid, 0444,
	       rmi_fn_01_productid_show, rmi_store_error),	/* RO attr */
	__ATTR(manufacturer, 0444,
	       rmi_fn_01_manufacturer_show, rmi_store_error),	/* RO attr */
	__ATTR(datecode, 0444,
	       rmi_fn_01_datecode_show, rmi_store_error),	/* RO attr */
	__ATTR(reportrate, 0666,
	       rmi_fn_01_reportrate_show, rmi_fn_01_reportrate_store),	/* RW */
	__ATTR(reset, 0222,
	       rmi_show_error, rmi_fn_01_reset_store),	/* WO attr */
	__ATTR(testerid, 0444,
	       rmi_fn_01_testerid_show, rmi_store_error),	/* RO attr */
	__ATTR(serialnumber, 0444,
	       rmi_fn_01_serialnumber_show, rmi_store_error),	/* RO attr */
	__ATTR(sleepmode, (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH),
	       rmi_fn_01_sleepmode_show, rmi_fn_01_sleepmode_store),	/* RW */
	__ATTR(nosleep, (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH),
			rmi_fn_01_nosleep_show, rmi_fn_01_nosleep_store), /*RW*/
	__ATTR(initreflash, (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH),
		rmi_fn_01_initreflash_show, rmi_fn_01_initreflash_store) /*RW*/
};

static int set_report_rate(struct rmi_function_info *function_info,
			   bool nonstandard)
{
	if (nonstandard) {
		return rmi_set_bits(function_info->sensor,
			function_info->function_descriptor.control_base_addr,
			NONSTANDARD_REPORT_RATE);
	} else {
		return rmi_clear_bits(function_info->sensor,
			function_info->function_descriptor.control_base_addr,
			NONSTANDARD_REPORT_RATE);
	}
}

static void read_query_registers(struct rmi_function_info *rmifninfo)
{
	unsigned char query_buffer[21];
	int retval;
	struct f01_instance_data *instance_data = rmifninfo->fndata;
	struct rmi_F01_query *query_registers = instance_data->query_registers;

	/* Read the query info and unpack it. */
	retval = rmi_read_multiple(rmifninfo->sensor,
			  rmifninfo->function_descriptor.query_base_addr,
		   query_buffer, ARRAY_SIZE(query_buffer));
	if (retval) {
		pr_err("%s : Could not read F01 query registers at "
			"0x%02x. Error %d.\n", __func__,
			rmifninfo->function_descriptor.query_base_addr,
			retval);
		/* Presumably if the read fails, the buffer should be all
		 * zeros, so we're OK to continue. */
	}
	query_registers->mfgid = query_buffer[F01_MFG_ID_POS];
	query_registers->properties = query_buffer[F01_PROPERTIES_POS];
	query_registers->prod_info[0] =
		query_buffer[F01_PRODUCT_INFO_POS] & 0x7F;
	query_registers->prod_info[1] =
		query_buffer[F01_PRODUCT_INFO_POS + 1] & 0x7F;
	query_registers->date_code[F01_DATE_CODE_YEAR] =
		query_buffer[F01_DATE_CODE_POS] & 0x1F;
	query_registers->date_code[F01_DATE_CODE_MONTH] =
		query_buffer[F01_DATE_CODE_POS + 1] & 0x0F;
	query_registers->date_code[F01_DATE_CODE_DAY] =
		query_buffer[F01_DATE_CODE_POS + 2] & 0x1F;
	query_registers->tester_id =
		(((unsigned short)query_buffer[F01_TESTER_ID_POS] & 0x7F) << 7)
		| (query_buffer[F01_TESTER_ID_POS + 1] & 0x7F);
	query_registers->serial_num =
		(((unsigned short)query_buffer[F01_SERIAL_NUMBER_POS] & 0x7F)
			<< 7)
		| (query_buffer[F01_SERIAL_NUMBER_POS + 1] & 0x7F);
	memcpy(query_registers->prod_id, &query_buffer[F01_PRODUCT_ID_POS],
		F01_PRODUCT_ID_LENGTH);

	pr_debug("%s: RMI4 Protocol Function $01 Query information\n",
		__func__);
	pr_debug("%s: Manufacturer ID: %d %s\n", __func__,
		query_registers->mfgid,
		query_registers->mfgid == 1 ? "(Synaptics)" : "");
	pr_debug("%s: Product Properties: 0x%x\n", __func__,
		query_registers->properties);
	pr_debug("%s: Product Info: 0x%x 0x%x\n", __func__,
		query_registers->prod_info[0], query_registers->prod_info[1]);
	pr_debug("%s: Date Code: Year : %d Month: %d Day: %d\n", __func__,
		query_registers->date_code[F01_DATE_CODE_YEAR],
		query_registers->date_code[F01_DATE_CODE_MONTH],
		query_registers->date_code[F01_DATE_CODE_DAY]);
	pr_debug("%s: Tester ID: %d\n", __func__, query_registers->tester_id);
	pr_debug("%s: Serial Number: 0x%x\n",
		__func__, query_registers->serial_num);
	pr_debug("%s: Product ID: %s\n", __func__, query_registers->prod_id);
}


void FN_01_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs)
{
	struct f01_instance_data *instance_data = rmifninfo->fndata;

	pr_debug("%s: Read device status.", __func__);

	if (rmi_read
	    (rmifninfo->sensor, rmifninfo->function_descriptor.data_base_addr,
	     &instance_data->data_registers->device_status)) {
		pr_err("%s : Could not read F01 device status.\n", __func__);
	}
	pr_info("%s: read device status register.  Value 0x%02X.", __func__,
		instance_data->data_registers->device_status);

	if (instance_data->data_registers->device_status & F01_UNCONFIGURED) {
		pr_info("%s: ++++ Device reset detected.", __func__);
		/* TODO: Handle device reset appropriately.
		 */
	}
}
EXPORT_SYMBOL(FN_01_inthandler);

/*
 * This reads in the function $01 source data.
 *
 */
void FN_01_attention(struct rmi_function_info *rmifninfo)
{
	struct f01_instance_data *instance_data = rmifninfo->fndata;
	int retval;

	/* TODO: Compute size to read and number of IRQ registers to processors
	 * dynamically.  See comments in rmi.h. */
	retval = rmi_read_multiple(rmifninfo->sensor,
			    rmifninfo->function_descriptor.data_base_addr + 1,
			    instance_data->data_registers->irqs, 1);
	if (retval) {
		pr_err("%s: Could not read interrupt status registers "
			"at 0x%02x; code=%d.", __func__,
			rmifninfo->function_descriptor.data_base_addr + 1,
			retval);
		return;
	}

	if (instance_data->data_registers->irqs[0] &
		instance_data->control_registers->interrupt_enable[0]) {
		/* call down to the sensors irq dispatcher to dispatch
		 * all enabled IRQs */
		rmifninfo->sensor->dispatchIRQs(rmifninfo->sensor,
						instance_data->data_registers->
						irqs[0]);
	}

}
EXPORT_SYMBOL(FN_01_attention);

int FN_01_config(struct rmi_function_info *rmifninfo)
{
	int retval = 0;
	struct f01_instance_data *instance_data = rmifninfo->fndata;

	pr_debug("%s: RMI4 function $01 config\n", __func__);

	/* First thing to do is set the configuration bit.  We'll check this at
	 * the end to determine if the device has reset during the config
	 * process.
	 */
	retval = rmi_set_bits(rmifninfo->sensor,
			 rmifninfo->function_descriptor.control_base_addr,
			 F01_CONFIGURED);
	if (retval)
		pr_warning("%s: failed to set configured bit, errno = %d.",
			   __func__, retval);

	/* At config time, the device is presumably in its default state, so we
	 * only need to write non-default configuration settings.
	 */
	if (instance_data->nonstandard_report_rate) {
		retval = set_report_rate(rmifninfo, true);
		if (!retval)
			pr_warning
			    ("%s: failed to configure report rate, errno = %d.",
			     __func__, retval);
	}

	/* TODO: Check for reset! */

	return retval;
}
EXPORT_SYMBOL(FN_01_config);

/* Initialize any function $01 specific params and settings - input
 * settings, device settings, etc.
 */
int FN_01_init(struct rmi_function_device *function_device)
{
	int retval;
	int attr_count = 0;
	struct rmi_f01_functiondata *functiondata =
	    rmi_sensor_get_functiondata(function_device->sensor, RMI_F01_INDEX);
	struct f01_instance_data *instance_data = function_device->rfi->fndata;

	pr_debug("%s: RMI4 function $01 init\n", __func__);

	if (functiondata)
		instance_data->nonstandard_report_rate =
		    functiondata->nonstandard_report_rate;

	pr_debug("%s: Creating sysfs files.", __func__);
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (sysfs_create_file
		    (&function_device->dev.kobj, &attrs[attr_count].attr) < 0) {
			pr_err("%s: Failed to create sysfs file for %s.",
				__func__, attrs[attr_count].attr.name);
			retval = -ENODEV;
			goto error_exit;
		}
	}

	return 0;

error_exit:
	for (attr_count--; attr_count >= 0; attr_count--)
		sysfs_remove_file(&function_device->dev.kobj,
				  &attrs[attr_count].attr);
	/* If you've allocated anything, free it here. */
	return retval;
}
EXPORT_SYMBOL(FN_01_init);

int FN_01_detect(struct rmi_function_info *rmifninfo)
{
	int retval = 0;
	struct f01_instance_data *instance_data = NULL;
	struct rmi_F01_control *control_registers = NULL;
	struct rmi_F01_data *data_registers = NULL;
	struct rmi_F01_query *query_registers = NULL;

	pr_debug("%s: RMI4 function $01 detect\n", __func__);

	/* Set up context data. */
	if (rmifninfo->fndata) {
		/* detect routine should only ever be called once
		 * per rmifninfo. */
		pr_err("%s: WTF?!? F01 instance data is already present!",
		       __func__);
		return -EINVAL;
	}
	instance_data = kzalloc(sizeof(*instance_data), GFP_KERNEL);
	if (!instance_data) {
		pr_err("%s: Error allocating memory for F01 context data.\n",
		       __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	rmifninfo->fndata = instance_data;

	query_registers = kzalloc(sizeof(*query_registers), GFP_KERNEL);
	if (!query_registers) {
		pr_err("%s: Error allocating memory for F01 query registers.",
		       __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	instance_data->query_registers = query_registers;
	read_query_registers(rmifninfo);

	/* TODO: size of control registers needs to be computed dynamically.
	 * See comment in rmi.h. */
	control_registers = kzalloc(sizeof(*control_registers), GFP_KERNEL);
	if (!control_registers) {
		pr_err
		    ("%s: Error allocating memory for F01 control registers.\n",
		     __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	instance_data->control_registers = control_registers;
	retval =
	    rmi_read_multiple(rmifninfo->sensor,
			      rmifninfo->function_descriptor.control_base_addr,
			      (char *)instance_data->control_registers,
			      sizeof(struct rmi_F01_control));
	if (retval) {
		pr_err
		    ("%s: Could not read F01 control registers at 0x%02x. "
		     "Error %d.\n", __func__,
		     rmifninfo->function_descriptor.control_base_addr,
		     retval);
	}

	/* TODO: size of data registers needs to be computed dynamically.
	 * See comment in rmi.h. */
	data_registers = kzalloc(sizeof(*data_registers), GFP_KERNEL);
	if (!data_registers) {
		pr_err("%s: Error allocating memory for F01 data registers.\n",
		       __func__);
		retval = -ENOMEM;
		goto error_exit;
	}
	instance_data->data_registers = data_registers;

	/* initialize original_sleepmode */
	instance_data->original_sleepmode = RMI_SLEEP_MODE_NORMAL;
	/* initialize of original_nosleep */
	instance_data->original_nosleep = RMI_NO_SLEEP_DISABLE;

	return retval;

error_exit:
	kfree(instance_data);
	kfree(query_registers);
	kfree(control_registers);
	kfree(data_registers);
	rmifninfo->fndata = NULL;
	return retval;
}
EXPORT_SYMBOL(FN_01_detect);

/**
 *  suspend handler for F01, this will be invoked in
 *  suspend routine from sensor
 */
int FN_01_suspend(struct rmi_function_info *rmifninfo)
{
	struct f01_instance_data *instance = rmifninfo->fndata;
	/*store original sleep mode */
	instance->original_sleepmode =
			instance->control_registers->
			device_control & RMI_F01_SLEEP_MODE_MASK;
	/*store original no sleep setting */
	instance->original_nosleep =
			(instance->control_registers->
				device_control & RMI_F01_NO_SLEEP_MASK) >> 2;
	/*sleep:1 normal:0 */
	set_sensor_sleepmode(rmifninfo, RMI_SLEEP_MODE_SENSOR_SLEEP,
			RMI_NO_SLEEP_DISABLE);
	return 0;
}
EXPORT_SYMBOL(FN_01_suspend);

/*
 *  resume handler for F01, this will be invoked in
 *  resume routine from sensor
 */
void FN_01_resume(struct rmi_function_info *rmifninfo)
{
	struct f01_instance_data *instance = rmifninfo->fndata;
	/*sleep:1 normal:0 */
	set_sensor_sleepmode(rmifninfo, instance->original_sleepmode,
			instance->original_nosleep);
}
EXPORT_SYMBOL(FN_01_resume);


static ssize_t rmi_fn_01_productinfo_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;

	if (instance_data && instance_data->query_registers
			&& instance_data->query_registers->prod_info)
		return snprintf(buf, PAGE_SIZE, "0x%02X 0x%02X\n",
			       instance_data->query_registers->prod_info[0],
			       instance_data->query_registers->prod_info[1]);

	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t rmi_fn_01_productid_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;

	if (instance_data && instance_data->query_registers
			&& instance_data->query_registers->prod_id)
		return snprintf(buf, PAGE_SIZE, "%s\n",
			instance_data->query_registers->prod_id);

	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t rmi_fn_01_manufacturer_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;

	if (instance_data && instance_data->query_registers)
		return snprintf(buf, PAGE_SIZE, "0x%02X\n",
			instance_data->query_registers->mfgid);

	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t rmi_fn_01_datecode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;

	if (instance_data && instance_data->query_registers
	    && instance_data->query_registers->date_code)
		return snprintf(buf, PAGE_SIZE, "20%02u-%02u-%02u\n",
			       instance_data->query_registers->date_code[0],
			       instance_data->query_registers->date_code[1],
			       instance_data->query_registers->date_code[2]);

	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t rmi_fn_01_reportrate_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;

	if (instance_data && instance_data->query_registers
			&& instance_data->query_registers->date_code)
		return snprintf(buf, PAGE_SIZE, "%d\n",
			instance_data->nonstandard_report_rate);

	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t rmi_fn_01_reportrate_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;
	unsigned int new_rate;
	int retval;

	if (sscanf(buf, "%u", &new_rate) != 1)
		return -EINVAL;
	if (new_rate < 0 || new_rate > 1)
		return -EINVAL;
	instance_data->nonstandard_report_rate = new_rate;

	retval = set_report_rate(fn->rfi, new_rate);
	if (retval < 0) {
		pr_err("%s: failed to set report rate bit, error = %d.",
		       __func__, retval);
		return retval;
	}

	return count;
}

static ssize_t rmi_fn_01_reset_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	unsigned int reset;
	int retval;

	if (sscanf(buf, "%u", &reset) != 1)
		return -EINVAL;
	if (reset < 0 || reset > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (reset) {
		retval = rmi_set_bits(fn->sensor,
				fn->rfi->function_descriptor.command_base_addr,
				F01_RESET);
		if (retval < 0) {
			dev_err(dev, "%s: failed to issue reset command, "
				"error = %d.", __func__, retval);
			return retval;
		}
	}

	return count;
}

static ssize_t rmi_fn_01_serialnumber_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;

	if (instance_data && instance_data->query_registers)
		return snprintf(buf, PAGE_SIZE, "%u\n",
			       instance_data->query_registers->serial_num);

	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

static ssize_t rmi_fn_01_testerid_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;

	if (instance_data && instance_data->query_registers)
		return snprintf(buf, PAGE_SIZE, "%u\n",
			       instance_data->query_registers->tester_id);

	return snprintf(buf, PAGE_SIZE, "unknown\n");
}

/*
 *  update content of device control into hardware and sync the status
 *  @param control_base_address base address for F01 device control
 *  @param mask mask of the field that will be changed
 *  @param field_target_value target value for the field
 */
static int update_device_control(struct rmi_sensor_driver *sensor,
			struct rmi_F01_control *control_register,
			unsigned char control_base_address,
			unsigned char mask,
			unsigned char field_target_value)
{

	unsigned char control_target_value =
			control_register->device_control & (~mask);
	field_target_value &= mask;
	control_target_value |= field_target_value;
	if (control_register->device_control != control_target_value) {
		/* update device_control*/
		control_register->device_control = control_target_value;
		return rmi_set_bit_field(sensor, control_base_address,
					mask, field_target_value);
	}
	return 0;
}

/*
 *  shows the status bit provided by device_control
 *  @param mask mask to retrieve the information ex. 0x3 for bit 0 and bit 1
 *  @param offset the offset of the information ex. 0: bit 0 1: bit 1
 */
static ssize_t device_control_show(struct device *dev,
				struct device_attribute *attr,
				char *buf,
				unsigned char mask,
				unsigned char offset)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;
	unsigned char controlRegister = 0;

	if (!instance_data || !instance_data->control_registers)
		return snprintf(buf, PAGE_SIZE, "unknown\n");

	controlRegister =
		(instance_data->control_registers->device_control & mask)
			>> offset;
	return snprintf(buf, PAGE_SIZE, "%u\n", controlRegister);
}

/*
 *  store the value into device_control
 *  @param mask mask to retrieve the information ex. 0x3 for bit 1 and bit 1
 *  @param offset the offset of the information ex. 0: bit 0 1: bit 1
 */
static ssize_t device_control_store(struct device *dev,
				struct device_attribute *attr,
				unsigned int new_value,
				unsigned char mask,
				unsigned char offset)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;
	unsigned char target_setting;
	int retval;

	target_setting = (unsigned char) new_value;
	if (!instance_data || !instance_data->control_registers)
		return -EINVAL;

	/*update hardware and device_control status*/
	retval = update_device_control(fn->sensor,
			instance_data->control_registers,
			fn->rfi->function_descriptor.control_base_addr,
			mask, (target_setting<<offset));

	if (retval < 0)
		dev_err(dev, "%s: failed to write control register, "
			"error = %d.", __func__, retval);

	return retval;
}

/*
 * show status for sleep 0:normal 1:sleep 2,3: reserved
 */
static ssize_t rmi_fn_01_sleepmode_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return device_control_show(dev, attr, buf,
			RMI_F01_SLEEP_MODE_MASK, RMI_F01_SLEEP_MODE_OFFSET);
}

/*
 * setup status for sleep mode 0:normal 1:sleep 2,3: reserved
 */
static ssize_t rmi_fn_01_sleepmode_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || !RMI_IS_VALID_SLEEPMODE(new_value)) {
		dev_err(dev, "%s: Invalid sleep mode %s.", __func__, buf);
		return -EINVAL;
	}

	retval = device_control_store(dev, attr, (unsigned int) new_value,
			RMI_F01_SLEEP_MODE_MASK, RMI_F01_SLEEP_MODE_OFFSET);
	if (!retval)
		retval = count;
	return retval;
}

/*
 * show current setting of no sleep, 0:disable 1:enable
 */
static ssize_t rmi_fn_01_nosleep_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	return device_control_show(dev, attr, buf,
			RMI_F01_NO_SLEEP_MASK, RMI_F01_NO_SLEEP_OFFSET);
}

/*
 * setup no sleep, 0:disable 1:enable
 */
static ssize_t rmi_fn_01_nosleep_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	unsigned long new_value;
	int retval;

	retval = strict_strtoul(buf, 10, &new_value);
	if (retval < 0 || new_value < 0 || new_value > 1) {
		dev_err(dev, "%s: Invalid no sleep setting %s.", __func__, buf);
		return -EINVAL;
	}

	retval = (device_control_store(dev, attr, (int) new_value,
			RMI_F01_NO_SLEEP_MASK, RMI_F01_NO_SLEEP_OFFSET));
	if (!retval)
		retval = count;
	return retval;
}

static ssize_t rmi_fn_01_initreflash_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	struct f01_instance_data *instance_data = fn->rfi->fndata;
	unsigned char initreflash = 0;

	if (!instance_data || !instance_data->query_registers)
		return snprintf(buf, PAGE_SIZE, "unknown\n");

	initreflash = (instance_data->query_registers->properties &
		RMI_F01_INIT_REFLASH_MASK) >> RMI_F01_INIT_REFLASH_OFFSET;

	return snprintf(buf, PAGE_SIZE, "%u\n", initreflash);
}

static ssize_t rmi_fn_01_initreflash_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct rmi_function_device *fn = dev_get_drvdata(dev);
	unsigned int initreflash;
	int retval;

	if (sscanf(buf, "%u", &initreflash) != 1)
		return -EINVAL;
	if (initreflash < 0 || initreflash > 1)
		return -EINVAL;

	/* Per spec, 0 has no effect, so we skip it entirely. */
	if (initreflash) {
		retval = rmi_set_bits(fn->sensor,
				fn->rfi->function_descriptor.command_base_addr,
				F01_INITREFLASH);
		if (retval < 0) {
			dev_err(dev, "%s: failed to issue initreflash command, "
				"error = %d.", __func__, retval);
			return retval;
		}
	}

	return count;
}

/* setup sleep mode via F01. we will store original_mode before sleepmode
 * and nosleep setting is changed.
 */
static void set_sensor_sleepmode(struct rmi_function_info *functionInfo,
			unsigned char sleepmode,
			unsigned char nosleep)
{
	struct f01_instance_data *instance_data =
			functionInfo->function_device->rfi->fndata;

	if (instance_data && instance_data->control_registers) {
		/*update hardware and device_control status*/
		update_device_control(functionInfo->sensor,
			instance_data->control_registers,
			functionInfo->function_descriptor.control_base_addr,
			(RMI_F01_SLEEP_MODE_MASK | RMI_F01_NO_SLEEP_MASK),
			(sleepmode | (nosleep << 2))
		);
	}
}
