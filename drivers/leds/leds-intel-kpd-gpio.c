/*
 * leds-intel-kpd-gpio.c - Intel GPIO Keypad LED driver
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/hrtimer.h>
#include <linux/spinlock.h>
#include <linux/earlysuspend.h>
#include <asm/intel_kpd_gpio_led.h>

static int gpio;
static spinlock_t lock;

static void intel_keypad_led_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	int i, level;
	u32 elapsed, needed;
	unsigned long flags;
	static ktime_t out;

	elapsed = ktime_to_us(ktime_sub(ktime_get(), out));
	if (unlikely(elapsed < 500)) {
		needed = 500 - elapsed;
		usleep_range(needed, needed + 50);
	}

	/* According to the AHK3292 Datasheet, AHK3292 has 32 level output
	 * current settings, range from 1 ~ 32, 1 is the highest and 32 is
	 * the lowest.*/
	if (value > 0) {
		level = ((100 - value) * 32 / 100) + 1;
		spin_lock_irqsave(&lock, flags);
		for (i = 0; i < level; i++) {
			gpio_set_value(gpio, 0);
			udelay(1);
			gpio_set_value(gpio, 1);
			udelay(1);
		}
		spin_unlock_irqrestore(&lock, flags);
	} else {
		gpio_set_value(gpio, 0);
	}

	out = ktime_get();
}

static struct led_classdev intel_kpd_led = {
	.name			= "intel_keypad_led",
	.brightness_set		= intel_keypad_led_set,
	.brightness		= LED_OFF,
	.max_brightness		= 15,
};

static void intel_kpd_led_early_suspend(struct early_suspend *h)
{
	led_classdev_suspend(&intel_kpd_led);
}

static void intel_kpd_led_late_resume(struct early_suspend *h)
{
	led_classdev_resume(&intel_kpd_led);
}

static struct early_suspend intel_kpd_led_suspend_desc = {
	.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = intel_kpd_led_early_suspend,
	.resume  = intel_kpd_led_late_resume,
};

static int __devinit intel_kpd_led_probe(struct platform_device *pdev)
{
	int ret;
	struct intel_kpd_gpio_led_pdata *pdata;

	spin_lock_init(&lock);

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return -EINVAL;

	gpio = pdata->gpio;
	if (gpio < 0)
		return -EINVAL;

	ret = gpio_request(gpio, "intel_kpd_led");
	if (ret)
		return ret;

	ret = gpio_direction_output(gpio, 0);
	if (ret) {
		gpio_free(gpio);
		return ret;
	}

	ret = led_classdev_register(&pdev->dev, &intel_kpd_led);
	if (ret) {
		dev_err(&pdev->dev, "register intel_kpd_led failed");
		gpio_free(gpio);
		return ret;
	}
	intel_keypad_led_set(&intel_kpd_led, intel_kpd_led.brightness);
	register_early_suspend(&intel_kpd_led_suspend_desc);

	return 0;
}

static int __devexit intel_kpd_led_remove(struct platform_device *pdev)
{
	intel_keypad_led_set(&intel_kpd_led, LED_OFF);
	unregister_early_suspend(&intel_kpd_led_suspend_desc);
	led_classdev_unregister(&intel_kpd_led);
	gpio_free(gpio);

	return 0;
}

static struct platform_driver kpd_led_driver = {
	.driver = {
		   .name = "intel_kpd_led",
		   .owner = THIS_MODULE,
	},
	.probe = intel_kpd_led_probe,
	.remove = __devexit_p(intel_kpd_led_remove),
};

static int __init intel_kpd_led_init(void)
{
	return platform_driver_register(&kpd_led_driver);
}

static void __exit intel_kpd_led_exit(void)
{
	platform_driver_unregister(&kpd_led_driver);
}

module_init(intel_kpd_led_init);
module_exit(intel_kpd_led_exit);

MODULE_AUTHOR("Shijie Zhang <shijie.zhang@intel.com>");
MODULE_DESCRIPTION("Intel GPIO Keypad LED Driver");
MODULE_LICENSE("GPL v2");
