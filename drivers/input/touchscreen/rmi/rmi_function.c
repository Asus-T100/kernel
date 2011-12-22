/**
 * Synaptics Register Mapped Interface (RMI4) - RMI Function Module.
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
#include "rmi_function.h"
#include "rmi_bus.h"
#include "rmi_sensor.h"
#include "rmi_f01.h"
#include "rmi_f05.h"
#include "rmi_f11.h"
#include "rmi_f19.h"
#include "rmi_f34.h"
#include "rmi_f54.h"


#define FUNCTION_NAME_SIZE 10

static int rmi_function_suspendable(struct rmi_function_info *rmifninfo);


/* NOTE: Developer - add in any new RMI4 fn data info - function number
 * and ptrs to report, config, init and detect functions.  This data is
 * used to point to the functions that need to be called to config, init,
 * detect and report data for the new RMI4 function. Refer to the RMI4
 * specification for information on RMI4 functions.
 */
/* TODO: This will eventually be built dynamically, as individual function
 * implementations registered.  For now, though we create it statically. */
static struct rmi_function_ops supported_functions[] = {
	/* Fn $01 - device control */
	{
		.function_number = RMI_F01_INDEX,
		.inthandler = FN_01_inthandler,
		.config = FN_01_config,
		.init = FN_01_init,
		.detect = FN_01_detect,
		.attention = FN_01_attention,
		.suspend = FN_01_suspend,
		.resume = FN_01_resume,
		.suspendable = rmi_function_suspendable},
	/* Fn $05 - analog report */
	{
		.function_number = RMI_F05_INDEX,
		.inthandler = FN_05_inthandler,
		.config = FN_05_config,
		.init = FN_05_init,
		.detect = FN_05_detect,
		.attention = NULL,
		.suspend = NULL,
		.resume = NULL,
		.suspendable = rmi_function_suspendable},
	 /* Fn $11 - 2D sensing */
	 {
		 .function_number = RMI_F11_INDEX,
		 .inthandler = FN_11_inthandler,
		 .config = FN_11_config,
		 .init = FN_11_init,
		 .detect = FN_11_detect,
		 .attention = NULL,
		 .suspend = NULL,
		 .resume = NULL,
		 .suspendable = rmi_function_suspendable},
	/* Fn $19 - buttons */
	{
		.function_number = RMI_F19_INDEX,
		.inthandler = FN_19_inthandler,
		.config = FN_19_config,
		.init = FN_19_init,
		.detect = FN_19_detect,
		.attention = NULL,
		.suspend = NULL,
		.resume = NULL,
		.suspendable = rmi_function_suspendable},
	/* Fn $1a - buttons */
	{
		.function_number = 0x1a,
		.inthandler = FN_19_inthandler,
		.config = FN_19_config,
		.init = FN_19_init,
		.detect = FN_19_detect,
		.attention = NULL,
		.suspend = NULL,
		.resume = NULL,
		.suspendable = rmi_function_suspendable},
	/* Fn $34 - firmware reflash */
	{
		.function_number = RMI_F34_INDEX,
		.inthandler = FN_34_inthandler,
		.config = FN_34_config,
		.init = FN_34_init,
		.detect = FN_34_detect,
		.attention = FN_34_attention,
		.suspend = NULL,
		.resume = NULL,
		.suspendable = rmi_function_suspendable},
	/* Fn $54 - diagnostics. */
	{
		.function_number = RMI_F54_INDEX,
		.inthandler = FN_54_inthandler,
		.config = FN_54_config,
		.init = FN_54_init,
		.detect = FN_54_detect,
		.attention = NULL,
		.suspend = NULL,
		.resume = NULL,
		.suspendable = rmi_function_suspendable},
};

/* This function is here to provide a way for external modules to access the
 * functions list.  It will try to find a matching function base on the passed
 * in RMI4 function number and return  the pointer to the struct rmi_functions
 * if a match is found or NULL if not found.
 */
struct rmi_function_ops *rmi_find_function(int function_number)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_functions); i++) {
		if (function_number == supported_functions[i].function_number)
			return &supported_functions[i];
	}

	return NULL;
}
EXPORT_SYMBOL(rmi_find_function);

static int rmi_function_suspendable(struct rmi_function_info *rmifninfo)
{
	return 1;
}

static void rmi_function_config(struct rmi_function_device *function)
{
	pr_debug("%s: rmi_function_config", __func__);
}

/* Just a stub for now.
 */
static int rmi_function_suspend(struct device *dev, pm_message_t state)
{
	pr_info("%s: function suspend called.", __func__);
	return 0;
}

/* Just a stub for now.
 */
static int rmi_function_resume(struct device *dev)
{
	pr_info("%s: function resume called.", __func__);
	return 0;
}

int rmi_function_register_driver(struct rmi_function_driver *drv,
				int function_number)
{
	int retval = 0;
	char *driver_name;

	pr_info("%s: Registering function driver for F%02x.\n", __func__,
		function_number);


	/* Create a function device and function driver for this Fn */
	driver_name = kzalloc(FUNCTION_NAME_SIZE, GFP_KERNEL);
	if (!driver_name) {
		pr_err("%s: Error allocating memory for "
		     "rmi_function_driver name.", __func__);
		return -ENOMEM;
	}
	snprintf(driver_name, FUNCTION_NAME_SIZE, "fn%02x", function_number);

	drv->drv.name = driver_name;
	drv->module = drv->drv.owner;

	drv->drv.suspend = rmi_function_suspend;
	drv->drv.resume = rmi_function_resume;

	/* register the sensor driver */
	retval = driver_register(&drv->drv);
	if (retval) {
		pr_err("%s: Failed driver_register %d\n", __func__, retval);
		drv->drv.name = NULL;
		kfree(driver_name);
	}

	return retval;
}
EXPORT_SYMBOL(rmi_function_register_driver);

void rmi_function_unregister_driver(struct rmi_function_driver *drv)
{
	char *driver_name =  (char *) drv->drv.name;

	pr_info("%s: Unregistering function driver.\n", __func__);

	/* TODO: Unregister the devices first. */
	driver_unregister(&drv->drv);
	kfree(driver_name);
}
EXPORT_SYMBOL(rmi_function_unregister_driver);

int rmi_function_register_device(struct rmi_function_device *function_device,
				 int fnNumber)
{
	struct input_dev *input;
	int retval = 0;

	pr_info("%s: Registering function device for F%02x.\n", __func__,
		fnNumber);

	/* make name - fn11, fn19, etc. */
	dev_set_name(&function_device->dev, "%sfn%02x",
		     function_device->sensor->drv.name, fnNumber);
	dev_set_drvdata(&function_device->dev, function_device);
	retval = device_register(&function_device->dev);
	if (retval) {
		pr_err("%s:  Failed device_register for function device.\n",
		       __func__);
		return retval;
	}

	input = input_allocate_device();
	if (input == NULL) {
		pr_err("%s:  Failed to allocate memory for a "
		       "new input device.\n", __func__);
		retval = -ENOMEM;
		goto error_exit;
	}

	input->name = dev_name(&function_device->dev);
	input->phys = "rmi_function";
	function_device->input = input;

	/* init any input specific params for this function */
	function_device->rmi_funcs->init(function_device);

	retval = input_register_device(input);
	if (retval) {
		pr_err("%s:  Failed input_register_device.\n", __func__);
		goto error_exit;
	}

	rmi_function_config(function_device);

	return retval;

error_exit:
	kfree(input);
	return retval;
}
EXPORT_SYMBOL(rmi_function_register_device);

void rmi_function_unregister_device(struct rmi_function_device *dev)
{
	pr_info("%s: Unregistering function device.n", __func__);

	input_unregister_device(dev->input);
	device_unregister(&dev->dev);
}
EXPORT_SYMBOL(rmi_function_unregister_device);

static int __init rmi_function_init(void)
{
	pr_debug("%s: RMI Function Init\n", __func__);

	return 0;
}

static void __exit rmi_function_exit(void)
{
	pr_debug("%s: RMI Function Exit\n", __func__);
}

module_init(rmi_function_init);
module_exit(rmi_function_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("RMI4 Function Driver");
MODULE_LICENSE("GPL");
