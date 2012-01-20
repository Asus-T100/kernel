/*
 *  intel_mid_vibra.c - Intel vibrator driver for Clovertrail phone
 *
 *  Copyright (C) 2011-12 Intel Corp
 *  Author: KP, Jeeja <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/gpio.h>
/* FIXME: remove once vibra becomes PCI driver */
#include "../../../sound/soc/mid-x86/sst_platform.h"

#define VIBRA_ENABLE_GPIO 40
#define PWM_ENABLE_GPIO 49

struct vibra_info {
	struct mutex		lock;
	struct device		*dev;
	int			enabled;
	void __iomem		*shim;
	const char              *name;
};

/* Powers H-Bridge and enables audio clk */
static void vibra_enable(struct vibra_info *info)
{
	pr_debug("Enable gpio\n");
	intel_sst_pwm_suspend(false);
	gpio_set_value(PWM_ENABLE_GPIO, 1);
	gpio_set_value(VIBRA_ENABLE_GPIO, 1);
	info->enabled = true;
}

static void vibra_disable(struct vibra_info *info)
{

	pr_debug("Disable gpio\n");
	gpio_set_value(PWM_ENABLE_GPIO, 0);
	gpio_set_value(VIBRA_ENABLE_GPIO, 0);
	info->enabled = false;
	intel_sst_pwm_suspend(true);
}


/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/

static ssize_t vibra_show_vibrator(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vibra_info *info = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", info->enabled);

}

static ssize_t vibra_set_vibrator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	long vibrator_enable;

	struct platform_device *pdev = to_platform_device(dev);
	struct vibra_info *info = platform_get_drvdata(pdev);

	if (strict_strtol(buf, 0, &vibrator_enable))
		return -EINVAL;
	if (vibrator_enable == 0)
		vibra_disable(info);
	else if (vibrator_enable == 1)
		vibra_enable(info);
	else
		return -EINVAL;
	return len;
}

static struct device_attribute vibra_attrs[] = {
	__ATTR(vibrator, S_IRUGO | S_IWUSR,
	       vibra_show_vibrator, vibra_set_vibrator),
};

static int vibra_register_sysfs(struct vibra_info *info)
{
	int r, i;

	for (i = 0; i < ARRAY_SIZE(vibra_attrs); i++) {
		r = device_create_file(info->dev, &vibra_attrs[i]);
		if (r)
			goto fail;
	}
	return 0;
fail:
	while (i--)
		device_remove_file(info->dev, &vibra_attrs[i]);

	return r;
}

static void vibra_unregister_sysfs(struct vibra_info *info)
{
	int i;

	for (i = ARRAY_SIZE(vibra_attrs) - 1; i >= 0; i--)
		device_remove_file(info->dev, &vibra_attrs[i]);
}

/*** Module ***/
#if CONFIG_PM
static int intel_mid_vibra_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vibra_info *info = platform_get_drvdata(pdev);

	pr_debug("intel_mid_vibra_suspend called\n");

	if (info->enabled)
		vibra_disable(info);

	return 0;
}

static int intel_mid_vibra_resume(struct device *dev)
{
	pr_debug("intel_mid_vibra_resume called\n");
	return 0;
}

static SIMPLE_DEV_PM_OPS(intel_mid_vibra_pm_ops,
			 intel_mid_vibra_suspend, intel_mid_vibra_resume);
#endif

static int __init intel_mid_vibra_probe(struct platform_device *pdev)
{
	struct vibra_info *info;
	int ret = 0;
	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = &pdev->dev;
	info->name = "intel_mid:vibrator";

	mutex_init(&info->lock);

	platform_set_drvdata(pdev, info);

	if (vibra_register_sysfs(info) < 0) {
		pr_err("could not register sysfs files\n");
		goto out;
	}

	ret = gpio_request(VIBRA_ENABLE_GPIO, "VIBRA ENABLE");
	if (ret != 0) {
		pr_err("gpio_request(%d) fails - %d\n", VIBRA_ENABLE_GPIO, ret);
		goto goto_unregistesysfs;
	}

	gpio_direction_output(VIBRA_ENABLE_GPIO, 0);

	ret = gpio_request(PWM_ENABLE_GPIO, "PWM ENABLE");

	if (ret != 0) {
		pr_err("gpio_request(%d) fails - %d\n", PWM_ENABLE_GPIO, ret);
		goto goto_freegpio;
	}

	gpio_direction_output(PWM_ENABLE_GPIO, 0);

	return ret;

goto_freegpio:
	gpio_free(VIBRA_ENABLE_GPIO);
goto_unregistesysfs:
	vibra_unregister_sysfs(info);
out:
	kfree(info);
	return ret;
}

static int __exit intel_mid_vibra_remove(struct platform_device *pdev)
{
	struct vibra_info *info = platform_get_drvdata(pdev);
	gpio_free(VIBRA_ENABLE_GPIO);
	gpio_free(PWM_ENABLE_GPIO);
	vibra_unregister_sysfs(info);
	kfree(info);
	return 0;
}


static struct platform_driver intel_mid_vibra_driver = {
	.driver		= {
		.name	= "intel_mid_vibra",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
	.pm	= &intel_mid_vibra_pm_ops,
#endif
	},
	.probe		= intel_mid_vibra_probe,
	.remove	= __devexit_p(intel_mid_vibra_remove),
};

static int __init intel_mid_vibra_init(void)
{
	pr_debug("intel_mid_vibra_init called\n");
	return platform_driver_register(&intel_mid_vibra_driver);
}
module_init(intel_mid_vibra_init);

static void __exit intel_mid_vibra_exit(void)
{
	pr_debug("intel_mid_vibra_exit called\n");
	platform_driver_unregister(&intel_mid_vibra_driver);
}
module_exit(intel_mid_vibra_exit);

MODULE_ALIAS("platform:intel_mid_vibra");
MODULE_DESCRIPTION("Intel(R) MID Vibra driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("KP Jeeja <jeeja.kp@intel.com>");
