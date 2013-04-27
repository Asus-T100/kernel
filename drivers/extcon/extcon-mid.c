/*
 * extcon-mid - extcon driver for accessory detection compatible with switch-mid
 *
 * Copyright (C) 2013 Intel Corp.
 *
 * Based on drivers/switch/switch_mid.c
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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/extcon.h>
#include <linux/extcon-mid.h>

struct mid_extcon_data {
	struct extcon_dev edev;
};

static char *name_headsets_with_mic = "headsets_with_mic_insert";
static char *name_headsets_no_mic = "headsets_no_mic_insert";
static char *name_headsets_pull_out = "headsets_pull_out";
static char *state_headsets_with_mic = "1";
static char *state_headsets_no_mic = "2";
static char *state_headsets_pull_out = "0";

static struct mid_extcon_data *headset_extcon_data;

void mid_extcon_headset_report(u32 state)
{
	if (headset_extcon_data)
		extcon_set_state(&headset_extcon_data->edev, state);
}
EXPORT_SYMBOL_GPL(mid_extcon_headset_report);

static ssize_t headset_print_name(struct extcon_dev *edev, char *buf)
{
	const char *name;

	if (!buf)
		return -EINVAL;

	switch (extcon_get_state(edev)) {
	case HEADSET_PULL_OUT:
		name = name_headsets_pull_out;
		break;
	case HEADSET_WITH_MIC:
		name = name_headsets_with_mic;
		break;
	case HEADSET_NO_MIC:
		name = name_headsets_no_mic;
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

static ssize_t headset_print_state(struct extcon_dev *edev, char *buf)
{
	const char *state;

	if (!buf)
		return -EINVAL;

	switch (extcon_get_state(edev)) {
	case HEADSET_PULL_OUT:
		state = state_headsets_pull_out;
		break;
	case HEADSET_WITH_MIC:
		state = state_headsets_with_mic;
		break;
	case HEADSET_NO_MIC:
		state = state_headsets_no_mic;
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

static int mid_extcon_probe(struct platform_device *pdev)
{
	struct mid_extcon_data *extcon_data;
	int ret = 0;

	extcon_data = kzalloc(sizeof(struct mid_extcon_data), GFP_KERNEL);
	if (!extcon_data)
		return -ENOMEM;
	headset_extcon_data = extcon_data;

	extcon_data->edev.name = "h2w";
	extcon_data->edev.print_name = headset_print_name;
	extcon_data->edev.print_state = headset_print_state;

	ret = extcon_dev_register(&extcon_data->edev, &pdev->dev);
	if (ret < 0)
		goto err_extcon_dev_register;

	platform_set_drvdata(pdev, extcon_data);
	return 0;

err_extcon_dev_register:
	kfree(extcon_data);
	return ret;
}

static int __devexit mid_extcon_remove(struct platform_device *pdev)
{
	struct mid_extcon_data *extcon_data = platform_get_drvdata(pdev);

	extcon_dev_unregister(&extcon_data->edev);
	kfree(extcon_data);
	headset_extcon_data = NULL;

	return 0;
}

static struct platform_driver mid_extcon_driver = {
	.probe		= mid_extcon_probe,
	.remove		= __devexit_p(mid_extcon_remove),
	.driver		= {
		.name	= "extcon-mid",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(mid_extcon_driver);

MODULE_AUTHOR("Omair Mohammed Abdullah (omair.m.abdullah@intel.com)");
MODULE_DESCRIPTION("Headset Extcon driver");
MODULE_LICENSE("GPL v2");
