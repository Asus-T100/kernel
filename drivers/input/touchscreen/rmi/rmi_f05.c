/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $11 support for 2D.
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
#include <linux/rmi_platformdata.h>

#include "rmi.h"
#include "rmi_drvr.h"
#include "rmi_bus.h"
#include "rmi_sensor.h"
#include "rmi_function.h"
#include "rmi_f05.h"

struct f05_instance_data {
	int dummy;		/* TODO: Write this */
};

/*
 * There is no attention function for F05 - it is left NULL
 * in the function table so it is not called.
 *
 */

/*
 * This reads in a sample and reports the F05 source data to the
 * input subsystem. It is used for both polling and interrupt driven
 * operation. This is called a lot so don't put in any informational
 * printks since they will slow things way down!
 *
 * This is a stub for now, and will be fleshed out when the implementation
 * is completed.
 */
void FN_05_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs)
{
}
EXPORT_SYMBOL(FN_05_inthandler);

/* This is a stub for now, and will be fleshed out when the implementation
 * is completed.
 */
int FN_05_config(struct rmi_function_info *rmifninfo)
{
	int retval = 0;

	pr_debug("%s: RMI4 F05 config\n", __func__);

	/* TODO: Perform configuration.  In particular, write any cached control
	 * register values to the device. */

	return retval;
}
EXPORT_SYMBOL(FN_05_config);

/* This is a stub for now, and will be fleshed out when the implementation
 * is completed.
 */
int FN_05_init(struct rmi_function_device *function_device)
{
	int retval = 0;
/*
	struct f05_instance_data *instance_data = function_device->rfi->fndata;
	struct rmi_f05_functiondata *functiondata =
	    rmi_sensor_get_functiondata(function_device->sensor, RMI_F05_INDEX);
*/

	pr_debug("%s: RMI4 F05 init\n", __func__);

	return retval;
}
EXPORT_SYMBOL(FN_05_init);


/*
 * This is stubbed for now, will be filled out in the future.
 */
int FN_05_detect(struct rmi_function_info *rmifninfo)
{
	int retval = 0;
	struct f05_instance_data *instance_data;

	pr_debug("%s: RMI4 F05 detect\n", __func__);

	if (rmifninfo->fndata) {
		/* detect routine should only ever be called once
		 * per rmifninfo. */
		pr_err("%s: WTF?!? F05 instance data is already present!",
		       __func__);
		return -EINVAL;
	}
	instance_data = kzalloc(sizeof(struct f05_instance_data), GFP_KERNEL);
	if (!instance_data) {
		pr_err("%s: Error allocating F05 instance data.\n", __func__);
		return -ENOMEM;
	}
	rmifninfo->fndata = instance_data;

	return retval;
}
EXPORT_SYMBOL(FN_05_detect);
