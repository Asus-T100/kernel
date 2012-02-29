/*
 * leds-intel-kpd.c - Intel Keypad LED driver
 *
 * Copyright (C) 2011 Intel Corporation
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
#include <linux/device.h>
#include <linux/ipc_device.h>
#include <linux/leds.h>
#include <linux/earlysuspend.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_pwm.h>

static void intel_keypad_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	intel_mid_pwm(PWM_LED, value);
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

static int __devinit intel_kpd_led_probe(struct ipc_device *ipcdev)
{
	int ret;

	ret = led_classdev_register(&ipcdev->dev, &intel_kpd_led);
	if (ret) {
		dev_err(&ipcdev->dev, "register intel_kpd_led failed");
		return ret;
	}
	intel_keypad_led_set(&intel_kpd_led, intel_kpd_led.brightness);
	register_early_suspend(&intel_kpd_led_suspend_desc);

	return 0;
}

static int __devexit intel_kpd_led_remove(struct ipc_device *ipcdev)
{
	intel_keypad_led_set(&intel_kpd_led, LED_OFF);
	unregister_early_suspend(&intel_kpd_led_suspend_desc);
	led_classdev_unregister(&intel_kpd_led);

	return 0;
}

static struct ipc_driver kpd_led_driver = {
	.driver = {
		   .name = "intel_kpd_led",
		   .owner = THIS_MODULE,
	},
	.probe = intel_kpd_led_probe,
	.remove = __devexit_p(intel_kpd_led_remove),
};

static int __init intel_kpd_led_init(void)
{
	int ret;
	ret = ipc_driver_register(&kpd_led_driver);
	return ret;
}

static void __exit intel_kpd_led_exit(void)
{
	ipc_driver_unregister(&kpd_led_driver);
}

module_init(intel_kpd_led_init);
module_exit(intel_kpd_led_exit);

MODULE_DESCRIPTION("Intel Keypad LED Driver");
MODULE_LICENSE("GPL v2");
