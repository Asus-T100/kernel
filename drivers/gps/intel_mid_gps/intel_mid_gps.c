/*
 * intel_mid_gps.c: Intel interface for gps devices
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Kuppuswamy, Sathyanarayanan
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>

#include <linux/intel_mid_gps.h>

#define DRIVER_NAME "intel_mid_gps"

/*********************************************************************
 *		Driver GPIO toggling functions
 *********************************************************************/

static int intel_mid_gps_reset(struct device *dev, const char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);

	if (!buf)
		return -EINVAL;

	if (!gps)
		return -EINVAL;

	if (sscanf(buf, "%i", &gps->reset) != 1)
		return -EINVAL;

	if (gpio_is_valid(gps->gpio_reset))
		gpio_set_value(gps->gpio_reset,
				gps->reset ? RESET_ON : RESET_OFF);
	else
		return -EINVAL;

	return 0;
}

static int intel_mid_gps_enable(struct device *dev, const char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);

	if (!buf)
		return -EINVAL;

	if (!gps)
		return -EINVAL;

	if (sscanf(buf, "%i", &gps->enable) != 1)
		return -EINVAL;

	if (gpio_is_valid(gps->gpio_enable))
		gpio_set_value(gps->gpio_enable,
				gps->enable ? ENABLE_ON : ENABLE_OFF);
	else
		return -EINVAL;

	return 0;
}

/*********************************************************************
 *		Driver sysfs attribute functions
 *********************************************************************/

static ssize_t intel_mid_gps_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);

	if (!gps)
		return -EINVAL;

	return sprintf(buf, "%d\n", gps->reset);
}

static ssize_t intel_mid_gps_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	ret = intel_mid_gps_reset(dev, buf);

	return !ret ? size : ret;
}

static ssize_t intel_mid_gps_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);

	if (!gps)
		return -EINVAL;

	return sprintf(buf, "%d\n", gps->enable);
}

static ssize_t intel_mid_gps_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	ret = intel_mid_gps_enable(dev, buf);

	return !ret ? size : ret;
}

static DEVICE_ATTR(reset, S_IRUGO|S_IWUSR, intel_mid_gps_reset_show,
				intel_mid_gps_reset_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, intel_mid_gps_enable_show,
				intel_mid_gps_enable_store);

static struct attribute *intel_mid_gps_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group intel_mid_gps_attr_group = {
	.name = DRIVER_NAME,
	.attrs = intel_mid_gps_attrs,
};

/*********************************************************************
 *		Driver GPIO probe/remove functions
 *********************************************************************/

static int intel_mid_gps_init(struct platform_device *pdev)
{
	int ret;
	struct intel_mid_gps_platform_data *pdata = pdev->dev.platform_data;

	ret = sysfs_create_group(&pdev->dev.kobj, &intel_mid_gps_attr_group);
	if (ret)
		dev_err(&pdev->dev,
			"Failed to create intel_mid_gps sysfs interface\n");

	/* Handle reset GPIO */
	if (gpio_is_valid(pdata->gpio_reset)) {

		/* Request gpio */
		ret = gpio_request(pdata->gpio_reset, "intel_mid_gps_reset");
		if (ret < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
					__func__, pdata->gpio_reset, ret);
			goto error_gpio_reset_request;
		}

		/* set gpio direction */
		ret = gpio_direction_output(pdata->gpio_reset, pdata->reset);
		if (ret < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
					__func__, pdata->gpio_reset, ret);
			goto error_gpio_reset_direction;
		}
	}

	/* Handle enable GPIO */
	if (gpio_is_valid(pdata->gpio_enable)) {

		/* Request gpio */
		ret = gpio_request(pdata->gpio_enable, "intel_mid_gps_enable");
		if (ret < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
					__func__, pdata->gpio_enable, ret);
			goto error_gpio_enable_request;
		}

		/* set gpio direction */
		ret = gpio_direction_output(pdata->gpio_enable, pdata->enable);
		if (ret < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
					__func__, pdata->gpio_enable, ret);
			goto error_gpio_enable_direction;
		}
	}

	return 0;

error_gpio_enable_direction:
	gpio_free(pdata->gpio_enable);
error_gpio_enable_request:
error_gpio_reset_direction:
	gpio_free(pdata->gpio_reset);
error_gpio_reset_request:
	sysfs_remove_group(&pdev->dev.kobj, &intel_mid_gps_attr_group);

	return ret;
}

static void intel_mid_gps_deinit(struct platform_device *pdev)
{
	struct intel_mid_gps_platform_data *pdata = pdev->dev.platform_data;

	if (gpio_is_valid(pdata->gpio_enable))
		gpio_free(pdata->gpio_enable);

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);

	sysfs_remove_group(&pdev->dev.kobj, &intel_mid_gps_attr_group);
}

static int intel_mid_gps_probe(struct platform_device *pdev)
{
	int ret = 0;

	pr_info("%s probe called\n", dev_name(&pdev->dev));

	if (strcmp(dev_name(&pdev->dev), DRIVER_NAME) == 0)
		ret = intel_mid_gps_init(pdev);

	if (ret) {
		dev_err(&pdev->dev, "Failed to initalize %s\n",
			dev_name(&pdev->dev));
		goto exit;
	}

	platform_set_drvdata(pdev, pdev->dev.platform_data);

exit:
	return ret;
}

static int __devexit intel_mid_gps_remove(struct platform_device *pdev)
{
	if (strcmp(dev_name(&pdev->dev), DRIVER_NAME) == 0)
		intel_mid_gps_deinit(pdev);

	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static const struct platform_device_id gps_id_table[] = {
	{DRIVER_NAME, 0},
};

static struct platform_driver intel_mid_gps_driver = {
	.probe		= intel_mid_gps_probe,
	.remove		= __devexit_p(intel_mid_gps_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.id_table = gps_id_table,
};

static int __init intel_mid_gps_driver_init(void)
{
	return platform_driver_register(&intel_mid_gps_driver);
}

static void __exit intel_mid_gps_driver_exit(void)
{
	platform_driver_unregister(&intel_mid_gps_driver);
}

module_init(intel_mid_gps_driver_init);
module_exit(intel_mid_gps_driver_exit);

MODULE_AUTHOR("Kuppuswamy, Sathyanarayanan");
MODULE_DESCRIPTION("Intel MID GPS driver");
MODULE_LICENSE("GPL");
