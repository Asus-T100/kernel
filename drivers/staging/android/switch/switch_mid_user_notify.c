/*
 *  drivers/switch/switch_mid_user_notify.c
 *
 * Copyright (C) 2012 Intel.
 *
 * Based on drivers/switch/switch_gpio.c which from Google Android.
 * Based on drivers/switch/switch_mid.c from Intel.
 * This is a switch driver for userspace notification of kernel events.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/intel_mid_thermal_probe.h>

/* This is a generic switch that can be used by any module by adding
   appropriate state names/values and handling the new names/states in
   the print functions. The state names and values should be unique.
   i.e, Do not reuse state names/values for events */

struct mid_switch_user_notify_data {
	struct switch_dev sdev;
};

/* Switch state names and values for thermal probe events */
static char *name_therm_probe_insert = "therm_probe_insert";
static char *name_therm_probe_pull_out = "therm_probe_pull_out";
static char *state_therm_probe_insert = "1";
static char *state_therm_probe_pull_out = "0";

static struct mid_switch_user_notify_data *user_notify_switch_data;

static inline bool is_therm_probe_event(int state)
{
	if (state == 0 || state == 1)
		return true;
	else
		return false;
}

void mid_user_notify(int state)
{
	if (user_notify_switch_data)
		switch_set_state(&user_notify_switch_data->sdev, state);
	/* Inform sensor driver about thermal probe events*/
	if (is_therm_probe_event(state))
		mid_thermal_probe_notifier_call_chain(state, NULL);
}
EXPORT_SYMBOL_GPL(mid_user_notify);

static ssize_t user_notify_print_name(struct switch_dev *sdev, char *buf)
{
	const char *name;

	if (!buf)
		return -EINVAL;

	switch (switch_get_state(sdev)) {
	case 0:
		name = name_therm_probe_pull_out;
		break;
	case 1:
		name = name_therm_probe_insert;
		break;
	default:
		name = NULL;
		break;
	}

	if (name)
		return sprintf(buf, "%s\n", name);
	else
		return -EINVAL;
}

static ssize_t user_notify_print_state(struct switch_dev *sdev, char *buf)
{
	const char *state;

	if (!buf)
		return -EINVAL;

	switch (switch_get_state(sdev)) {
	case 0:
		state = state_therm_probe_pull_out;
		break;
	case 1:
		state = state_therm_probe_insert;
		break;
	default:
		state = NULL;
		break;
	}

	if (state)
		return sprintf(buf, "%s\n", state);
	else
		return -EINVAL;
}

static int mid_switch_user_notify_probe(struct platform_device *pdev)
{
	struct mid_switch_user_notify_data *switch_data;
	int ret = 0;

	switch_data =
		kzalloc(sizeof(struct mid_switch_user_notify_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;
	user_notify_switch_data = switch_data;

	switch_data->sdev.name = "user_notify";
	switch_data->sdev.print_name = user_notify_print_name;
	switch_data->sdev.print_state = user_notify_print_state;

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	platform_set_drvdata(pdev, switch_data);
	return 0;

err_switch_dev_register:
	kfree(switch_data);
	return ret;
}

static int __devexit mid_switch_user_notify_remove(struct platform_device *pdev)
{
	struct mid_switch_user_notify_data *switch_data =
					platform_get_drvdata(pdev);

	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);
	user_notify_switch_data = NULL;

	return 0;
}

static struct platform_driver mid_switch_user_notify_driver = {
	.probe		= mid_switch_user_notify_probe,
	.remove		= __devexit_p(mid_switch_user_notify_remove),
	.driver		= {
		.name	= "switch-mid_user_notify",
		.owner	= THIS_MODULE,
	},
};

static int __init mid_switch_user_notify_init(void)
{
	return platform_driver_register(&mid_switch_user_notify_driver);
}

static void __exit mid_switch_user_notify_exit(void)
{
	platform_driver_unregister(&mid_switch_user_notify_driver);
}

module_init(mid_switch_user_notify_init);
module_exit(mid_switch_user_notify_exit);

MODULE_AUTHOR("B, Jayachandran (jayachandran.b@intel.com)");
MODULE_DESCRIPTION("Userspace Notification Switch driver");
MODULE_LICENSE("GPLv2");
