/**
 * Synaptics Register Mapped Interface (RMI4) - RMI Sensor Module.
 * Copyright (C) 2007 - 2011, Synaptics Incorporated
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
#include <linux/gpio.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/input.h>
#include <linux/interrupt.h>

#include "rmi_drvr.h"
#include "rmi_bus.h"
#include "rmi_function.h"
#include "rmi_sensor.h"

/* Context data for each sensor.
 */
struct sensor_instance_data {
	unsigned char pdt_props;
	unsigned char bsr;
	bool enabled;
};

#define HAS_BSR_MASK 0x20
#define HAS_NONSTANDARD_PDT_MASK 0x40

static bool has_bsr(struct sensor_instance_data *instance_data)
{
	return (instance_data->pdt_props & HAS_BSR_MASK) != 0;
}

long polltime = 25000000;	/* Shared with rmi_function.c. */
EXPORT_SYMBOL(polltime);
module_param(polltime, long, 0644);
MODULE_PARM_DESC(polltime, "How long to wait between polls (in nano seconds).");

#define PDT_START_SCAN_OFFSET 0x00E9
#define PDT_END_SCAN_OFFSET 0x0005
#define PDT_ENTRY_SIZE 0x0006
#define PDT_PROPERTIES_LOCATION 0x00EF
#define BSR_LOCATION 0x00FE

static DEFINE_MUTEX(rfi_mutex);

struct rmi_function_ops *rmi_find_function(int function_number);

/* sysfs files for sensor attributes for BSR register value. */
static ssize_t rmi_sensor_hasbsr_show(struct device *dev,
				      struct device_attribute *attr, char *buf);

static ssize_t rmi_sensor_bsr_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t rmi_sensor_bsr_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t rmi_sensor_enabled_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static ssize_t rmi_sensor_enabled_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count);

static ssize_t rmi_sensor_phy_show(struct device *dev,
				   struct device_attribute *attr, char *buf);

static struct device_attribute attrs[] = {
	__ATTR(hasbsr, 0444,
	       rmi_sensor_hasbsr_show, rmi_store_error),	/* RO attr */
	__ATTR(bsr, 0666,
	       rmi_sensor_bsr_show, rmi_sensor_bsr_store),	/* RW attr */
	__ATTR(enabled, 0666,
	       rmi_sensor_enabled_show, rmi_sensor_enabled_store), /* RW attr */
	__ATTR(phy, 0444,
	       rmi_sensor_phy_show, rmi_store_error)		/* RO attr */
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void rmi_sensor_early_suspend(struct early_suspend *h);
static void rmi_sensor_late_resume(struct early_suspend *h);
#endif

int rmi_read(struct rmi_sensor_driver *sensor, unsigned short address,
	     char *dest)
{
	struct rmi_phys_driver *rpd = sensor->rpd;
	struct rmi_sensor_device *sensor_device = sensor->sensor_device;
	struct sensor_instance_data *instance_data;

	/* we could read while sensor_device is not ready */
	if (sensor_device) {
		instance_data = sensor_device->sensordata;
		/* we could read while instance_data is not ready */
		if (instance_data)
			if (!instance_data->enabled)
				return -ENODEV;
	}

	if (!rpd)
		return -ENODEV;

	return rpd->read(rpd, address, dest);
}
EXPORT_SYMBOL(rmi_read);

int rmi_write(struct rmi_sensor_driver *sensor, unsigned short address,
	      unsigned char data)
{
	struct rmi_phys_driver *rpd = sensor->rpd;
	struct rmi_sensor_device *sensor_device = sensor->sensor_device;
	struct sensor_instance_data *instance_data;

	/* we could write while sensor_device is not ready */
	if (sensor_device) {
		instance_data = sensor_device->sensordata;
		/* we could write while instance_data is not ready */
		if (instance_data)
			if (!instance_data->enabled)
				return -ENODEV;
	}

	if (!rpd)
		return -ENODEV;

	return rpd->write(rpd, address, data);
}
EXPORT_SYMBOL(rmi_write);

int rmi_read_multiple(struct rmi_sensor_driver *sensor, unsigned short address,
		      char *dest, int length)
{
	struct rmi_phys_driver *rpd = sensor->rpd;
	struct rmi_sensor_device *sensor_device = sensor->sensor_device;

	/* we could read while sensor_device is not ready */
	if (sensor_device) {
		struct sensor_instance_data *instance_data =
				sensor_device->sensordata;
		/* we could read while instance_data is not ready */
		if (instance_data)
			if (!instance_data->enabled)
				return -ENODEV;
	}
	if (!rpd)
		return -ENODEV;

	return rpd->read_multiple(rpd, address, dest, length);
}
EXPORT_SYMBOL(rmi_read_multiple);

int rmi_write_multiple(struct rmi_sensor_driver *sensor, unsigned short address,
		       unsigned char *data, int length)
{
	struct rmi_phys_driver *rpd = sensor->rpd;
	struct rmi_sensor_device *sensor_device = sensor->sensor_device;
	struct sensor_instance_data *instance_data;

	/* we could write while sensor_device is not ready */
	if (sensor_device) {
		instance_data = sensor_device->sensordata;
		/* we could write while instance_data is not ready */
		if (instance_data)
			if (!instance_data->enabled)
				return -ENODEV;
	}
	if (!rpd)
		return -ENODEV;

	return rpd->write_multiple(rpd, address, data, length);
}
EXPORT_SYMBOL(rmi_write_multiple);

/* Utility routine to set bits in a register. */
int rmi_set_bits(struct rmi_sensor_driver *sensor, unsigned short address,
		 unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read(sensor, address, &reg_contents);
	if (retval) {
		pr_debug("%s: Read at 0x%04x failed, code: %d.\n",
			__func__, address, retval);
		return retval;
	}
	reg_contents = reg_contents | bits;
	retval = rmi_write(sensor, address, reg_contents);
	if (retval == 1)
		return 0;
	else if (retval == 0) {
		pr_debug("%s: Write at 0x%04x failed.\n",
			__func__, address);
		return -EINVAL;	/* TODO: What should this be? */
	}
	return retval;
}
EXPORT_SYMBOL(rmi_set_bits);

/* Utility routine to clear bits in a register. */
int rmi_clear_bits(struct rmi_sensor_driver *sensor, unsigned short address,
		   unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read(sensor, address, &reg_contents);
	if (retval)
		return retval;
	reg_contents = reg_contents & ~bits;
	retval = rmi_write(sensor, address, reg_contents);
	if (retval == 1)
		return 0;
	else if (retval == 0)
		return -EINVAL;	/* TODO: What should this be? */
	return retval;
}
EXPORT_SYMBOL(rmi_clear_bits);

/* Utility routine to set the value of a bit field in a register. */
int rmi_set_bit_field(struct rmi_sensor_driver *sensor, unsigned short address,
		      unsigned char field_mask, unsigned char bits)
{
	unsigned char reg_contents;
	int retval;

	retval = rmi_read(sensor, address, &reg_contents);
	if (retval)
		return retval;
	reg_contents = (reg_contents & ~field_mask) | bits;
	retval = rmi_write(sensor, address, reg_contents);
	if (retval == 1)
		return 0;
	else if (retval == 0)
		return -EINVAL;	/* TODO: What should this be? */
	return retval;
}
EXPORT_SYMBOL(rmi_set_bit_field);

bool rmi_polling_required(struct rmi_sensor_driver *sensor)
{
	return sensor->polling_required;
}
EXPORT_SYMBOL(rmi_polling_required);

/* Keeps track of how many sensors we've seen so far.  TODO: What happens
 * if we disconnect from a sensor?  Does it sensor number get recycled?
 */
static int sensor_count;

/* Sensors are identified starting at 0 and working up.  This will retrieve
 * the current sensor number, and increment the sensor_count.
 */
int rmi_next_sensor_id()
{
	int id = sensor_count;
	sensor_count++;
	return id;
}
EXPORT_SYMBOL(rmi_next_sensor_id);

/* Functions can call this in order to dispatch IRQs. */
void dispatchIRQs(struct rmi_sensor_driver *sensor, unsigned int irq_status)
{
	struct rmi_function_info *function_info;

	list_for_each_entry(function_info, &sensor->functions, link) {
		if ((function_info->interrupt_mask & irq_status)
		    && function_info->function_device
		    && function_info->function_device->rmi_funcs->
				inthandler) {
			/* Call the function's interrupt handler. */
			function_info->function_device->rmi_funcs->
				inthandler(function_info,
					(function_info->
					interrupt_mask & irq_status));
		}
	}
}

/*
 * This is the function we pass to the RMI4 subsystem so we can be notified
 * when attention is required.  It may be called in interrupt context.
 */
static void attention(struct rmi_phys_driver *physdrvr)
{
	/* All we have to do is schedule work. */
	schedule_work(&(physdrvr->sensor->work));
}

static void disable_sensor(struct rmi_sensor_driver *sensor)
{
	struct rmi_phys_driver *rpd = sensor->rpd;
	struct sensor_instance_data *instance_data =
		sensor->sensor_device->sensordata;

	rpd->disable_device(rpd);
	instance_data->enabled = false;
}

static int enable_sensor(struct rmi_sensor_driver *sensor)
{
	struct rmi_phys_driver *rpd = sensor->rpd;
	struct sensor_instance_data *instance_data =
		sensor->sensor_device->sensordata;
	int retval = 0;

	retval = rpd->enable_device(rpd);
	/* non-zero means error occurred */
	if (retval)
		return retval;
	instance_data->enabled = true;

	return 0;
}

/* This notifies any interested functions that there is an Attention interrupt.
 * The interested functions should take appropriate actions (such as reading
 * the interrupt status register and dispatching any appropriate RMI4
 * interrupts).
 */
void attn_notify(struct rmi_sensor_driver *sensor)
{
	struct rmi_function_info *function_info;

	list_for_each_entry(function_info, &sensor->functions, link) {
		if (function_info->function_device
		    && function_info->function_device->rmi_funcs->attention) {
			function_info->function_device->rmi_funcs->
			    attention(function_info);
		}
	}
}

/* This is the worker function - for now it simply has to call attn_notify.
 * This work should be scheduled whenever an ATTN interrupt is asserted by
 * the touch sensor.  We then call attn_notify to dispatch notification of
 * the ATTN interrupt to all interested functions. After all the attention
 * handling functions have returned, it is presumed safe to re-enable the
 * Attention interrupt.
 */
static void sensor_work_func(struct work_struct *work)
{
	struct rmi_sensor_driver *sensor =
			container_of(work, struct rmi_sensor_driver, work);
	struct rmi_sensor_device *sensor_dev = sensor->sensor_device;

	mutex_lock(&sensor->work_lock);
	attn_notify(sensor);

	/* we only need to enable the irq if doing interrupts */
	/*
	* if suspend operation occurs and
	* this is the function during execution
	* we cannot enable irq again
	*/
	if (!rmi_polling_required(sensor) &&
			!sensor_dev->device_is_suspended)
		enable_irq(sensor->rpd->irq);
	mutex_unlock(&sensor->work_lock);
}

/* This is the timer function for polling - it simply has to schedule work
 * and restart the timer. */
static enum hrtimer_restart sensor_poll_timer_func(struct hrtimer *timer)
{
	struct rmi_sensor_driver *sensor =
			container_of(timer, struct rmi_sensor_driver, timer);

	if (!work_pending(&sensor->work))
		schedule_work(&sensor->work);
	hrtimer_start(&sensor->timer, ktime_set(0, polltime), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/* This is the probe function passed to the RMI4 subsystem that gives us a
 * chance to recognize an RMI4 device.  In this case, we're looking for
 * Synaptics devices that have data sources - such as touch screens, buttons,
 * etc.
 *
 * TODO: Well, it used to do this.  I'm not sure it's required any more.
 */
static int probe(struct rmi_sensor_driver *sensor)
{
	struct rmi_phys_driver *rpd = sensor->rpd;
	pr_debug("%s: PROBE CALLED", __func__);

	if (!rpd) {
		pr_err("%s: Invalid rmi physical driver - null ptr: %p\n",
		       __func__, rpd);
		return -EINVAL;
	}

	return 0;
}

static void config(struct rmi_sensor_driver *sensor)
{
	/* For each data source we had detected print info and set up interrupts
	   or polling. */
	struct rmi_function_info *function_info;
	struct rmi_phys_driver *rpd = sensor->rpd;
	struct sensor_instance_data *instance_data =
			sensor->sensor_device->sensordata;
	int attr_count = 0;

	int retval;

	dev_dbg(&sensor->sensor_device->dev, "%s: CONFIG CALLED", __func__);

	list_for_each_entry(function_info, &sensor->functions, link) {
		/* Get and print some info about the data sources... */
		struct rmi_function_ops *fn;
		/* check if function number matches - if so call that
		   config function */
		fn = rmi_find_function(function_info->function_number);
		if (fn) {
			if (fn->config) {
				fn->config(function_info);
			} else {
				dev_warn(&sensor->sensor_device->dev,
					"%s: no config function for "
					"function 0x%02x.\n", __func__,
					function_info->function_number);
			}
		} else {
			/* if no support found for this RMI4 function
			   it means the developer did not add the
			   appropriate function pointer list into the
			   rmi4_supported_data_src_functions array and/or
			   did not bump up the number of supported RMI4
			   functions in rmi.h as required */
			dev_err(&sensor->sensor_device->dev,
			       "%s: no support found for function 0x%02x.\n",
			       __func__, function_info->function_number);
		}
	}

	retval = rpd->read(rpd, PDT_PROPERTIES_LOCATION,
			   (char *) &instance_data->pdt_props);
	if (retval) {
		dev_warn(&sensor->sensor_device->dev,
			"%s: Could not read PDT propertys from 0x%04x. "
			"Assuming 0x00.\n",
		       __func__, PDT_PROPERTIES_LOCATION);
	}


	dev_dbg(&sensor->sensor_device->dev, "%s: Creating sysfs files.",
		__func__);
	/* Set up sysfs device attributes. */
	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		if (device_create_file(&sensor->sensor_device->dev,
					&attrs[attr_count]) < 0) {
			dev_err(&sensor->sensor_device->dev,
				"%s: Failed to create sysfs file for %s.\n",
				__func__, attrs[attr_count].attr.name);
			goto error_exit;
		}
	}

	if (rmi_polling_required(sensor)) {
		/* We're polling driven, so set up the polling timer
		   and timer function. */
		hrtimer_init(&sensor->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		sensor->timer.function = sensor_poll_timer_func;
		hrtimer_start(&sensor->timer, ktime_set(1, 0),
			      HRTIMER_MODE_REL);
	}

	instance_data->enabled = true;
	return;

error_exit:
	for (attr_count--; attr_count >= 0; attr_count--)
		device_remove_file(&sensor->sensor_device->dev,
				   &attrs[attr_count]);
	/* If you alloc anything, free it here. */
}

void *rmi_sensor_get_functiondata(struct rmi_sensor_driver *driver,
				  unsigned char function_index)
{
	int i;

	if (!driver->perfunctiondata)
		return NULL;

	for (i = 0; i < driver->perfunctiondata->count; i++) {
		if (driver->perfunctiondata->functiondata[i].function_index ==
		    function_index)
			return driver->perfunctiondata->functiondata[i].data;
	}

	return NULL;
}

/*
 *  final implementation of suspend/early_suspend function
 */
static int rmi_sensor_suspend(struct device *dev, pm_message_t state)
{
	struct rmi_sensor_device *sensor_device =
	    container_of(dev, struct rmi_sensor_device, dev);
	struct rmi_phys_driver *phys_drvr = sensor_device->driver->rpd;
	struct rmi_sensor_driver *sensor_drvr = sensor_device->driver;
	struct rmi_sensor_suspend_custom_ops *custom_ops =
	    sensor_drvr->custom_suspend_ops;
	int retval;
	struct rmi_function_info *function_info;
	bool canSuspend = true;

	mutex_lock(&sensor_drvr->sensor_device->setup_suspend_flag);

	if (sensor_device->device_is_suspended) {
		mutex_unlock(&sensor_drvr->sensor_device->setup_suspend_flag);
		return 0;
	}

	/* iterates all of the functions to make sure that we
	 * can enter suspend mode. */
	list_for_each_entry(function_info, &sensor_drvr->functions, link) {
		if (function_info->function_device
			&& function_info->function_device->
				rmi_funcs->suspendable) {

			if (!(function_info->function_device->rmi_funcs->
			    suspendable(function_info))) {
				canSuspend = false;
				dev_err(dev, "%s: suspend fails, F0x%02X is "
					"not suspendable", __func__,
					function_info->function_number);
				break;
			}

		}
	}

	if (!canSuspend) {
		mutex_unlock(&sensor_drvr->sensor_device->setup_suspend_flag);
		return -1;
	}

	/* set flag before disabling irq */
	sensor_device->device_is_suspended = 1;

	if (rmi_polling_required(sensor_drvr)) {
		hrtimer_cancel(&(sensor_drvr->timer));
	} else {
		if (phys_drvr)
			disable_irq(phys_drvr->irq);
	}
	retval = cancel_work_sync(&sensor_drvr->work);
	if (retval && !(rmi_polling_required(sensor_drvr))) {
		/* if work is pending ,suspend fail */
		if (phys_drvr)
			enable_irq(phys_drvr->irq);
		/* reset suspend flag */
		sensor_device->device_is_suspended = 0;
		dev_err(dev, "%s: suspend fails, work pending", __func__);
		retval = -1;
		goto exit;
	}

	/* invoke the suspend handler of each functions of this sensor */
	/* ex. we will call suspend of F01 in the loop*/
	list_for_each_entry(function_info, &sensor_drvr->functions, link) {
		if (function_info->function_device
			&& function_info->function_device->rmi_funcs->suspend) {

			retval = function_info->function_device->rmi_funcs->
				suspend(function_info);
			if (retval) {
				/* reset suspend flag */
				sensor_device->device_is_suspended = 0;

				if (rmi_polling_required(sensor_drvr)) {
					/* restart polling timer*/
					hrtimer_start(&(sensor_drvr->timer),
							ktime_set(1, 0),
							HRTIMER_MODE_REL);
				} else {
					if (phys_drvr) {
						/* re-enalbe irq*/
						enable_irq(phys_drvr->irq);
					}
				}
				dev_err(dev, "%s: failed to suspend F0x%02x.",
					__func__,
					function_info->function_number);
				retval = -1;
				goto exit;
			}
		}
	}

	/* apply customized settings */
	if (custom_ops && custom_ops->rmi_sensor_custom_suspend)
		custom_ops->rmi_sensor_custom_suspend();

exit:
	mutex_unlock(&sensor_drvr->sensor_device->setup_suspend_flag);
	return retval;
}

/*
 *  final implementation of resume/late_resume function
 */
static int rmi_sensor_resume(struct device *dev)
{
	struct rmi_sensor_device *sensor_device =
	    container_of(dev, struct rmi_sensor_device, dev);
	struct rmi_phys_driver *phys_drvr = sensor_device->driver->rpd;
	struct rmi_sensor_driver *sensor_drvr = sensor_device->driver;
	struct rmi_sensor_suspend_custom_ops *custom_ops =
	    sensor_drvr->custom_suspend_ops;
	struct rmi_function_info *function_info;

	mutex_lock(&sensor_drvr->sensor_device->setup_suspend_flag);
	if (sensor_device->device_is_suspended) {
		/* reset suspend flag reenable irq */
		sensor_device->device_is_suspended = 0;
		/* apply customized settings */
		if (custom_ops && custom_ops->rmi_sensor_custom_resume)
			custom_ops->rmi_sensor_custom_resume();

		/* invoke the resume handler of each functions of this sensor */
		/* ex. we will call resume of F01 in the loop*/
		list_for_each_entry(function_info,
				&sensor_drvr->functions, link) {
			if (function_info->function_device
				&& function_info->function_device->
					rmi_funcs->resume)
				function_info->function_device->rmi_funcs->
				    resume(function_info);
		}

		/* apply delay after setup hardware */
		if (custom_ops && custom_ops->delay_resume)
			mdelay(custom_ops->delay_resume);

		if (rmi_polling_required(sensor_drvr)) {
			hrtimer_start(&(sensor_drvr->timer), ktime_set(1, 0),
				      HRTIMER_MODE_REL);
		} else {
			if (phys_drvr)
				enable_irq(phys_drvr->irq);
		}
	}
	mutex_unlock(&sensor_drvr->sensor_device->setup_suspend_flag);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * Handler for early suspend
 */
static void rmi_sensor_early_suspend(struct early_suspend *h)
{
	struct rmi_sensor_device *sensor_device =
	    container_of(h, struct rmi_sensor_device, early_suspend_handler);
	pm_message_t state;
	state.event = PM_EVENT_SUSPEND;
	(void)rmi_sensor_suspend(&(sensor_device->dev), state);
}

/*
 * Handler for late resume
 */
static void rmi_sensor_late_resume(struct early_suspend *h)
{
	struct rmi_sensor_device *sensor_device =
	    container_of(h, struct rmi_sensor_device, early_suspend_handler);
	(void)rmi_sensor_resume(&(sensor_device->dev));
}
#endif


#define RMI4_MAX_PAGE 0xFF
#define RMI4_PAGE_SIZE 0x100

/*
 * This method is called, whenever a new sensor device is added for the rmi
 * bus.
 *
 * It will scan the devices PDT to determine the supported functions
 * and create a new function device for each of these. It will read
 * the query, control, command and data regsiters for the function
 * to be used for each newly created function device.
 *
 * The sensor device is then bound to every function it supports.
 *
 */
static int rmi_sensor_register_functions(struct rmi_sensor_driver *sensor)
{
	struct rmi_function_device *function;
	unsigned int interrupt_register_count = 0;
	struct rmi_phys_driver *rpd = sensor->rpd;
	int i;
	int j;
	int page;
	bool done; /* Indicates the last page we checked for PDT's had none */
	int interrupt_offset;
	unsigned char interrupt_count = 0;
	struct rmi_function_descriptor rmi_fd;
	struct rmi_function_ops *fops;
	int retval = 0;
	struct device *dev = &sensor->sensor_device->dev;
	struct rmi_function_info *function_info = NULL;

	/* Read the Page Descriptor Table to determine what functions
	 * are present */
	dev_dbg(dev, "%s: Scanning page descriptors.", __func__);
	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		int page_start = RMI4_PAGE_SIZE * page;
		int pdt_start = page_start + PDT_START_SCAN_OFFSET;
		int pdt_end = page_start + PDT_END_SCAN_OFFSET;
		done = true; /* Assume we are done until we read a
				valid function */

		for (i = pdt_start; i >= pdt_end; i -= PDT_ENTRY_SIZE) {
			dev_dbg(dev, "%s: Reading page descriptor at 0x%04x.\n",
				__func__,  i);
			retval = rpd->read_multiple(rpd, i, (char *)&rmi_fd,
						    sizeof(rmi_fd));

			if (retval) {
				/* failed to read next PDT entry - end PDT
				   scan - this may result in an incomplete set
				   of recognized functions - we could return
				   an error here but the driver may still be
				   viable for diagnostics and debugging so let's
				   let it continue. */
				dev_err(dev,
				    "%s: Read error %d at PDT entry 0x%02x, "
				     "ending scan.\n", __func__, retval, i);
				break;
			}

			if (!RMI_IS_VALID_FUNCTION_ID(rmi_fd.function_number)) {
				/* A zero or 0xff in the function number
				   signals the end of the PDT */
				dev_dbg(dev, "%s: Found end of PDT.\n",
					__func__);
				break;
			} else {
				/* There must be at least one valid function on
				 * a page for this code to run. We must check
				 * the next page.
				 */
				 done = false;
			}

			dev_dbg(dev, "%s: F%02x - queries %02x commands %02x "
				"control %02x data %02x ints %02x",
				__func__, rmi_fd.function_number,
				rmi_fd.query_base_addr,
				rmi_fd.command_base_addr,
				rmi_fd.control_base_addr, rmi_fd.data_base_addr,
				rmi_fd.interrupt_source_count);

			/* determine if the function is supported and if so
			 * then bind this function device to the sensor */
			function_info = kzalloc(sizeof(*function_info),
						GFP_KERNEL);
			if (!function_info) {
				dev_err(dev,
				    "%s: out of memory for function F%02x.",
				     __func__, rmi_fd.function_number);
				retval = -ENOMEM;
				goto exit_fail;
			}
			function_info->sensor = sensor;
			function_info->function_number = rmi_fd.function_number;
			/* Here we add in the current page start address to our
			 * addressess. According to current RMI specification,
			 * all PDT entries are located on the same page as
			 * their corresponding function. This is because we
			 * don't have the room to store page info in the table,
			 * but can infer it off of the current page.
			 */

			function_info->function_descriptor.query_base_addr =
				rmi_fd.query_base_addr + page_start;
			function_info->function_descriptor.command_base_addr =
				rmi_fd.command_base_addr + page_start;
			function_info->function_descriptor.control_base_addr =
				rmi_fd.control_base_addr + page_start;
			function_info->function_descriptor.data_base_addr =
				rmi_fd.data_base_addr + page_start;

			function_info->function_descriptor.function_number =
				rmi_fd.function_number;
			function_info->num_data_sources =
				rmi_fd.interrupt_source_count;
			function_info->interrupt_register = interrupt_count / 8;
			/* loop through interrupts for each source and or in
			 * a bit to the interrupt mask for each. */
			interrupt_offset = interrupt_count % 8;

			for (j = interrupt_offset;
				j < ((rmi_fd.interrupt_source_count & 0x7)
					+ interrupt_offset);
				j++) {
				function_info->interrupt_mask |= 1 << j;
			}
			INIT_LIST_HEAD(&function_info->link);

			/* Get the ptr to the detect function based on
			 * the function number */
			dev_dbg(dev, "%s: Checking for RMI function F%02x.",
				__func__, rmi_fd.function_number);
			fops = rmi_find_function(rmi_fd.function_number);
			if (!fops) {
				dev_err(dev,
					"%s: couldn't find support for F%02X.",
					__func__, rmi_fd.function_number);
			} else {
				retval = fops->detect(function_info);
				if (retval)
					dev_err(dev,
					    "%s: Function detect for F%02x "
					    "failed with %d.",
					     __func__, rmi_fd.function_number,
					     retval);

				/* Create a function device and
				 * function driver. */
				function = kzalloc(sizeof(*function),
						   GFP_KERNEL);
				if (!function) {
					dev_err(dev,
					    "%s: Error allocating memory for "
					     "rmi_function_device.",
					     __func__);
					retval = -ENOMEM;
					goto exit_fail;
				}

				function->dev.parent =
					&sensor->sensor_device->dev;
				function->dev.bus =
					sensor->sensor_device->dev.bus;
				function->rmi_funcs = fops;
				function->sensor = sensor;
				function->rfi = function_info;
				function_info->function_device = function;

				/* Check if we have an interrupt mask of 0 and
				 * a non-NULL interrupt handler function and
				 * print a debug message since we should never
				 * have this.
				 */
				if (function_info->interrupt_mask == 0
				    && fops->inthandler != NULL) {
					dev_warn(dev,
					    "%s: Can't have a zero interrupt mask "
					     "for function F%02x (which requires an "
					     "interrupt handler).",
					     __func__, rmi_fd.function_number);
				}

				/* Check if we have a non-zero interrupt mask
				 * and a NULL interrupt handler function and
				 * print a debug message since we should never
				 * have this.
				 */
				if (function_info->interrupt_mask != 0
				    && fops->inthandler == NULL) {
					dev_warn(dev,
					    "%s: Can't have a non-zero interrupt "
					     "mask %d for function F%02x with a NULL "
					     "inthandler fn.\n",
					     __func__,
					     function_info->interrupt_mask,
					     rmi_fd.function_number);
				}

				/* Register the rmi function device */
				retval = rmi_function_register_device(function,
							rmi_fd.function_number);
				if (retval) {
					dev_err(dev,
					    "%s: Failed to register function "
					    " device.", __func__);
					goto exit_fail;
				}
			}

			/* bump interrupt count for next iteration.
			 * NOTE: The value 7 is reserved - for now,
			 * only bump up one for an interrupt count of 7.
			 */
			if ((rmi_fd.interrupt_source_count & 0x7) == 0x7) {
				interrupt_count += 1;
			} else {
				interrupt_count +=
				    (rmi_fd.interrupt_source_count & 0x7);
			}

			/* link this function info to the RMI module infos list
			 * of functions. */
			if (function_info == NULL) {
				dev_dbg(dev, "%s: WTF? function_info is null "
					" here.", __func__);
			} else {
				dev_dbg(dev, "%s: Adding F%02x with %d sources.",
					 __func__,
					 function_info->function_number,
					 function_info->num_data_sources);

				mutex_lock(&rfi_mutex);
				list_add_tail(&function_info->link,
					      &sensor->functions);
				mutex_unlock(&rfi_mutex);
			}
			function_info = NULL;
		}
	}
	dev_dbg(dev, "%s: Done scanning.", __func__);

	/* calculate the interrupt register count - used in the
	 * ISR to read the correct number of interrupt registers */
	interrupt_register_count = (interrupt_count + 7) / 8;
	/* TODO: Is interrupt_register_count needed by the sensor anymore? */
	sensor->interrupt_register_count = interrupt_register_count;

	return 0;

exit_fail:
	if (function_info)
		kfree(function_info->function_device);
	kfree(function_info);
	return retval;
}

/* sysfs show and store fns for sensor dev */
static ssize_t rmi_sensor_hasbsr_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rmi_sensor_device *sensor = dev_get_drvdata(dev);
	struct sensor_instance_data *instance_data =
	    (struct sensor_instance_data *)sensor->sensordata;

	return snprintf(buf, PAGE_SIZE, "%u\n", has_bsr(instance_data));
}

/* Show physical connection information as:
 *     prot tx_count tx_bytes tx_errors rx_count rx_bytes rx_errors attn
 */
static ssize_t rmi_sensor_phy_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct rmi_sensor_device *sensor = dev_get_drvdata(dev);
	struct rmi_phys_driver *rpd = sensor->driver->rpd;

	return snprintf(buf, PAGE_SIZE, "%s %ld %ld %ld %ld %ld %ld %ld\n",
			rpd->proto_name, rpd->tx_count, rpd->tx_bytes,
			rpd->tx_errors, rpd->rx_count, rpd->rx_bytes,
			rpd->rx_errors, rpd->attn_count);
}

static ssize_t rmi_sensor_bsr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi_sensor_device *sensor = dev_get_drvdata(dev);
	struct sensor_instance_data *instance_data =
	    (struct sensor_instance_data *)sensor->sensordata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->bsr);
}

static ssize_t rmi_sensor_bsr_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int retval;
	struct rmi_sensor_device *sensor = dev_get_drvdata(dev);
	struct sensor_instance_data *instance_data =
	    (struct sensor_instance_data *)sensor->sensordata;
	unsigned long val;

	/* need to convert the string data to an actual value */
	retval = strict_strtoul(buf, 10, &val);
	if (retval < 0)
		return retval;


	retval = rmi_write(sensor->driver, BSR_LOCATION, (unsigned char)val);
	if (retval) {
		dev_err(dev, "%s : failed to write bsr %u to 0x%x\n",
		       __func__, (unsigned int)val, BSR_LOCATION);
		return -EIO;
	}

	instance_data->bsr = val;

	return count;
}

static ssize_t rmi_sensor_enabled_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct rmi_sensor_device *sensor = dev_get_drvdata(dev);
	struct sensor_instance_data *instance_data = sensor->sensordata;

	return snprintf(buf, PAGE_SIZE, "%u\n", instance_data->enabled);
}

static ssize_t rmi_sensor_enabled_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int retval;
	struct rmi_sensor_device *sensor = dev_get_drvdata(dev);
	bool new_value;

	if (sysfs_streq(buf, "0"))
		new_value = false;
	else if (sysfs_streq(buf, "1"))
		new_value = true;
	else
		return -EINVAL;

	if (new_value) {
		retval = enable_sensor(sensor->driver);
		if (retval) {
			dev_err(dev, "Failed to ensable sensor, code=%d.\n",
				retval);
			return -EIO;
		}
	} else {
		disable_sensor(sensor->driver);
	}

	return count;
}

/* Call this to instantiate a new sensor driver.
 */
struct rmi_sensor_driver *rmi_sensor_create_driver(
				struct rmi_sensor_device *sensor_device,
				struct rmi_phys_driver *physical_driver,
				struct rmi_sensordata *sensor_data)
{
	struct rmi_sensor_driver *driver =
	    kzalloc(sizeof(struct rmi_sensor_driver), GFP_KERNEL);
	if (!driver) {
		dev_err(&sensor_device->dev,
			"%s: Out of memory for rmi_sensor_driver\n",
		       __func__);
		goto error_exit;
	}
	driver->sensor_device = sensor_device;
	driver->polling_required = physical_driver->polling_required;
	driver->rpd = physical_driver;

	mutex_init(&driver->work_lock);

	if (sensor_data) {
		driver->perfunctiondata = sensor_data->perfunctiondata;
		/* pass reference to customized operations for suspend/resume */
		driver->custom_suspend_ops = sensor_data->custom_suspend_ops;
	}
	INIT_LIST_HEAD(&driver->functions);

	/* This will handle interrupts on the ATTN line (interrupt driven)
	 * or will be called every poll interval (when we're not interrupt
	 * driven).
	 */
	INIT_WORK(&driver->work, sensor_work_func);

	return driver;

error_exit:
	rmi_sensor_destroy_driver(driver);
	return NULL;
}

/* Call this when you're done with the sensor driver.  This will clean up any
 * pending actions, cancel any running threads or works, and release all
 * storage.
 */
void rmi_sensor_destroy_driver(struct rmi_sensor_driver *driver)
{
	kfree(driver);
}

int rmi_sensor_register_device(struct rmi_sensor_device *dev, int index)
{
	int status;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend *early_suspend_handler;
#endif
	struct sensor_instance_data *instance_data;

	pr_debug("%s: Registering sensor device.\n", __func__);

	/* make name - sensor00, sensor01, etc. */
	dev_set_name(&dev->dev, "sensor%02d", index);

	dev->sensordata =
	    kzalloc(sizeof(struct sensor_instance_data), GFP_KERNEL);
	if (!dev->sensordata) {
		dev_err(&dev->dev,
		    "%s: Out of memory for sensor instance data.\n", __func__);
		return -ENOMEM;
	}
	instance_data = dev->sensordata;
	/* let initial rmi_read_multiple/rmi_read happy */
	instance_data->enabled = true;

	status = device_register(&dev->dev);

	if (status < 0) {
		dev_err(&dev->dev, "%s: device register failed with %d.",
			__func__, status);
		goto error_exit;
	}

	mutex_init(&dev->setup_suspend_flag);

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* register early_suspend handler after device is registered
	 */
	early_suspend_handler = &(dev->early_suspend_handler);
	early_suspend_handler->level =
		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	early_suspend_handler->suspend = rmi_sensor_early_suspend;
	early_suspend_handler->resume = rmi_sensor_late_resume;
	register_early_suspend(early_suspend_handler);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	return 0;

error_exit:
	kfree(dev->sensordata);
	return status;
}
EXPORT_SYMBOL(rmi_sensor_register_device);

static void rmi_sensor_unregister_device(struct rmi_sensor_device *rmisensordev)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend *early_suspend_handler;

	/* unregister early_suspend handler before driver is unregistered
	 */
	early_suspend_handler =
		    &(rmisensordev->early_suspend_handler);
	unregister_early_suspend(early_suspend_handler);
#endif /* CONFIG_HAS_EARLYSUSPEND */
	dev_dbg(&rmisensordev->dev,
		"%s: Unregistering sensor device.\n", __func__);

	device_unregister(&rmisensordev->dev);
}
EXPORT_SYMBOL(rmi_sensor_unregister_device);

#define DRIVER_NAME_CHARS	16

int rmi_sensor_register_driver(struct rmi_sensor_driver *driver)
{
	static int index;
	int ret;
	char *drvrname;

	pr_info("%s: Registering sensor driver.\n", __func__);
	driver->dispatchIRQs = dispatchIRQs;
	driver->attention = attention;
	driver->config = config;
	driver->probe = probe;

	driver->drv.suspend = rmi_sensor_suspend;
	driver->drv.resume = rmi_sensor_resume;
	/* Create a function device and function driver for this Fn */
	drvrname = kzalloc(DRIVER_NAME_CHARS, GFP_KERNEL);
	if (!drvrname) {
		pr_err
		    ("%s: Error allocating memory for rmi_sensor_driver name.",
		     __func__);
		return -ENOMEM;
	}
	snprintf(drvrname, DRIVER_NAME_CHARS, "sensor%02d", index++);

	driver->drv.name = drvrname;
	driver->module = driver->drv.owner;

	/* Register the sensor driver on the bus. */
	ret = rmi_bus_register_sensor_driver(driver);
	if (ret) {
		pr_err("%s: Failed to register driver on bus, error = %d",
		       __func__, ret);
		goto exit_fail;
	}

	/* register the functions on the sensor */
	ret = rmi_sensor_register_functions(driver);
	if (ret) {
		pr_err("%s: Failed rmi_sensor_register_functions %d",
		       __func__, ret);
		goto exit_fail;
	}

	/* configure the sensor - enable interrupts for each function,
	 * init work, set polling timer or adjust report rate, etc. */
	config(driver);
#if defined(CONFIG_SYNA_RMI_DEV)
	if (rmi_char_dev_register(driver->rpd, driver->sensor_device))
		pr_err("%s: error register char device", __func__);
#endif /*CONFIG_SYNA_RMI_DEV*/
	pr_debug("%s: sensor driver registration completed.", __func__);

exit_fail:
	kfree(drvrname);
	return ret;
}
EXPORT_SYMBOL(rmi_sensor_register_driver);

static void rmi_sensor_unregister_driver(struct rmi_sensor_driver *driver)
{
#if defined(CONFIG_SYNA_RMI_DEV)
	struct rmi_sensor_device *rmisensordev = driver->sensor_device;
#endif /* CONFIG_SYNA_RMI_DEV */
	pr_debug("%s: Unregistering sensor driver.\n", __func__);

#if defined(CONFIG_SYNA_RMI_DEV)
	rmi_char_dev_unregister(rmisensordev->char_dev,
			rmisensordev->rmi_char_device_class);
#endif /*CONFIG_SYNA_RMI_DEV*/

	/* Stop the polling timer if doing polling */
	if (rmi_polling_required(driver))
		hrtimer_cancel(&driver->timer);

	flush_scheduled_work();	/* Make sure all scheduled work is stopped */

	rmi_bus_register_sensor_driver(driver);
}
EXPORT_SYMBOL(rmi_sensor_unregister_driver);

static int __init rmi_sensor_init(void)
{
	pr_debug("%s: RMI Sensor Init\n", __func__);
	return 0;
}

static void __exit rmi_sensor_exit(void)
{
	pr_debug("%s: RMI Sensor Driver Exit\n", __func__);
	flush_scheduled_work();	/* Make sure all scheduled work is stopped */
}

module_init(rmi_sensor_init);
module_exit(rmi_sensor_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("RMI4 Sensor Driver");
MODULE_LICENSE("GPL");
