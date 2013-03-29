/*
 *  drivers/switch/switch_mid.c
 *
 * Copyright (C) 2010 Intel.
 *
 * Based on drivers/switch/switch_gpio.c which from Google Android.
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

struct mid_switch_data {
	struct switch_dev sdev;
};

static char *name_hadsets_with_mic = "headsets_with_mic_insert";
static char *name_hadsets_no_mic = "headsets_no_mic_insert";
static char *name_hadsets_pull_out = "headsets_pull_out";
static char *state_hadsets_with_mic = "1";
static char *state_hadsets_no_mic = "2";
static char *state_hadsets_pull_out = "0";

static struct mid_switch_data *headset_switch_data;

void mid_headset_report(int state)
{
	if (headset_switch_data)
		switch_set_state(&headset_switch_data->sdev, state);
}
EXPORT_SYMBOL_GPL(mid_headset_report);

static ssize_t headset_print_name(struct switch_dev *sdev, char *buf)
{
	const char *name;

	if (!buf)
		return -EINVAL;

	switch (switch_get_state(sdev)) {
	case 0:
		name = name_hadsets_pull_out;
		break;
	case 1:
		name = name_hadsets_with_mic;
		break;
	case 2:
		name = name_hadsets_no_mic;
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

static ssize_t headset_print_state(struct switch_dev *sdev, char *buf)
{
	const char *state;

	if (!buf)
		return -EINVAL;

	switch (switch_get_state(sdev)) {
	case 0:
		state = state_hadsets_pull_out;
		break;
	case 1:
		state = state_hadsets_with_mic;
		break;
	case 2:
		state = state_hadsets_no_mic;
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

static int mid_switch_probe(struct platform_device *pdev)
{
	struct mid_switch_data *switch_data;
	int ret = 0;

	switch_data = kzalloc(sizeof(struct mid_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;
	headset_switch_data = switch_data;

	switch_data->sdev.name = "h2w";
	switch_data->sdev.print_name = headset_print_name;
	switch_data->sdev.print_state = headset_print_state;

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	platform_set_drvdata(pdev, switch_data);
	return 0;

err_switch_dev_register:
	kfree(switch_data);
	return ret;
}

static int __devexit mid_switch_remove(struct platform_device *pdev)
{
	struct mid_switch_data *switch_data = platform_get_drvdata(pdev);

	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);
	headset_switch_data = NULL;

	return 0;
}

static struct platform_driver mid_switch_driver = {
	.probe		= mid_switch_probe,
	.remove		= __devexit_p(mid_switch_remove),
	.driver		= {
		.name	= "switch-mid",
		.owner	= THIS_MODULE,
	},
};

static int __init mid_switch_init(void)
{
	return platform_driver_register(&mid_switch_driver);
}

static void __exit mid_switch_exit(void)
{
	platform_driver_unregister(&mid_switch_driver);
}

module_init(mid_switch_init);
module_exit(mid_switch_exit);

MODULE_AUTHOR("Deng, Bing (bingx.deng@intel.com)");
MODULE_DESCRIPTION("Headset Switch driver");
MODULE_LICENSE("GPL");
