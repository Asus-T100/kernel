/*
 * Bluetooth Broadcomm  and low power control via GPIO
 *
 *  Copyright (C) 2011 Google, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/irq.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <asm/bcm_bt_lpm.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_hsu.h>

static struct rfkill *bt_rfkill;
static bool bt_enabled;
static bool host_wake_uart_enabled;
static bool wake_uart_enabled;
static bool int_handler_enabled;

#define LPM_ON

static void activate_irq_handler(void);

struct bcm_bt_lpm {
	unsigned int gpio_wake;
	unsigned int gpio_host_wake;
	unsigned int int_host_wake;
	unsigned int gpio_enable_bt;

	int wake;
	int host_wake;

	struct hrtimer enter_lpm_timer;
	ktime_t enter_lpm_delay;

	struct device *tty_dev;

	struct wake_lock wake_lock;
	char wake_lock_name[100];

	int port;
	void (*uart_enable)(struct device *tty);
	void (*uart_disable)(struct device *tty);
} bt_lpm;

static int bcm43xx_bt_rfkill_set_power(void *data, bool blocked)
{
	/* rfkill_ops callback. Turn transmitter on when blocked is false */

	if (!blocked) {
		gpio_set_value(bt_lpm.gpio_wake, 1);
		/*
		* Delay advice by BRCM is min 2.5ns,
		* setting it between 10 and 50us for more confort
		*/
		usleep_range(10, 50);

		gpio_set_value(bt_lpm.gpio_enable_bt, 1);
		pr_debug("%s: turn BT on\n", __func__);
	} else {
		gpio_set_value(bt_lpm.gpio_enable_bt, 0);
		pr_debug("%s: turn BT off\n", __func__);
	}

	bt_enabled = !blocked;

	return 0;
}

static const struct rfkill_ops bcm43xx_bt_rfkill_ops = {
	.set_block = bcm43xx_bt_rfkill_set_power,
};

#ifdef LPM_ON
static void set_wake_locked(int wake)
{
	bt_lpm.wake = wake;

	if (!wake)
		wake_unlock(&bt_lpm.wake_lock);

	if (!wake_uart_enabled && wake) {
		WARN_ON(!bt_lpm.tty_dev);
		bt_lpm.uart_enable(bt_lpm.tty_dev);
	}

	gpio_set_value(bt_lpm.gpio_wake, wake);

	if (wake_uart_enabled && !wake) {
		WARN_ON(!bt_lpm.tty_dev);
		bt_lpm.uart_disable(bt_lpm.tty_dev);
	}
	wake_uart_enabled = wake;
}

static enum hrtimer_restart enter_lpm(struct hrtimer *timer)
{
	pr_debug("%s\n", __func__);

	set_wake_locked(0);

	return HRTIMER_NORESTART;
}


static void update_host_wake_locked(int host_wake)
{
	if (host_wake == bt_lpm.host_wake)
		return;

	bt_lpm.host_wake = host_wake;

	if (host_wake) {
		wake_lock(&bt_lpm.wake_lock);
		if (!host_wake_uart_enabled) {
			WARN_ON(!bt_lpm.tty_dev);
			bt_lpm.uart_enable(bt_lpm.tty_dev);
		}
	} else  {
		if (host_wake_uart_enabled) {
			WARN_ON(!bt_lpm.tty_dev);
			bt_lpm.uart_disable(bt_lpm.tty_dev);
		}
		/*
		 * Take a timed wakelock, so that upper layers can take it.
		 * The chipset deasserts the hostwake lock, when there is no
		 * more data to send.
		 */
		wake_lock_timeout(&bt_lpm.wake_lock, HZ/2);
	}

	host_wake_uart_enabled = host_wake;

}

static irqreturn_t host_wake_isr(int irq, void *dev)
{
	int host_wake;

	host_wake = gpio_get_value(bt_lpm.gpio_host_wake);

	pr_debug("%s: lpm %s\n", __func__, host_wake ? "off" : "on");

	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_FALLING :
							IRQF_TRIGGER_RISING);

	if (!bt_lpm.tty_dev) {
		bt_lpm.host_wake = host_wake;
		return IRQ_HANDLED;
	}

	update_host_wake_locked(host_wake);

	return IRQ_HANDLED;
}

static void activate_irq_handler(void)
{
	int ret;

	pr_debug("%s\n", __func__);

	ret = request_irq(bt_lpm.int_host_wake, host_wake_isr,
				IRQF_TRIGGER_RISING, "bt_host_wake", NULL);

	if (ret < 0) {
		pr_err("Error lpm request IRQ");
		gpio_free(bt_lpm.gpio_wake);
		gpio_free(bt_lpm.gpio_host_wake);
	}
}


static void bcm_bt_lpm_wake_peer(struct device *dev)
{
	bt_lpm.tty_dev = dev;

	/*
	 * the irq is enabled after the first host wake up signal.
	 * in the original code, the irq should be in levels but, since mfld
	 * does not support them, irq is triggering with edges.
	 */

	if (!int_handler_enabled) {
		int_handler_enabled = true;
		activate_irq_handler();
	}

	hrtimer_try_to_cancel(&bt_lpm.enter_lpm_timer);

	set_wake_locked(1);

	hrtimer_start(&bt_lpm.enter_lpm_timer, bt_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);

}

static int bcm_bt_lpm_init(struct platform_device *pdev)
{
	int ret;
	struct device *tty_dev;

	hrtimer_init(&bt_lpm.enter_lpm_timer, CLOCK_MONOTONIC,
							HRTIMER_MODE_REL);
	bt_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	bt_lpm.enter_lpm_timer.function = enter_lpm;

	bt_lpm.host_wake = 0;

	if (bt_lpm.gpio_host_wake < 0) {
		pr_err("Error bt_lpm.gpio_host_wake\n");
		return -ENODEV;
	}

	ret = irq_set_irq_wake(bt_lpm.int_host_wake, 1);
	if (ret < 0) {
		pr_err("Error lpm set irq IRQ");
		gpio_free(bt_lpm.gpio_wake);
		gpio_free(bt_lpm.gpio_host_wake);
		return ret;
	}

	tty_dev = intel_mid_hsu_set_wake_peer(bt_lpm.port,
			bcm_bt_lpm_wake_peer);
	if (!tty_dev) {
		pr_err("Error no tty dev");
		gpio_free(bt_lpm.gpio_wake);
		gpio_free(bt_lpm.gpio_host_wake);
		return -ENODEV;
	}

	snprintf(bt_lpm.wake_lock_name, sizeof(bt_lpm.wake_lock_name),
			"BTLowPower");
	wake_lock_init(&bt_lpm.wake_lock, WAKE_LOCK_SUSPEND,
			 bt_lpm.wake_lock_name);

	bcm_bt_lpm_wake_peer(tty_dev);
	return 0;
}
#endif

static int bcm43xx_bluetooth_probe(struct platform_device *pdev)
{
	struct bcm_bt_lpm_platform_data *pdata = pdev->dev.platform_data;
	bool default_state = true;	/* off */
	int ret = 0;

	int_handler_enabled = false;

	if (pdata == NULL) {
		pr_err("Cannot register bcm_bt_lpm drivers, pdata is NULL\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio_enable)) {
		pr_err("%s: gpio not valid\n", __func__);
		return -EINVAL;
	}

#ifdef LPM_ON
	if (!gpio_is_valid(pdata->gpio_wake) ||
		!gpio_is_valid(pdata->gpio_host_wake)) {
		pr_err("%s: gpio not valid\n", __func__);
		return -EINVAL;
	}
#endif

	bt_lpm.gpio_wake = pdata->gpio_wake;
	bt_lpm.gpio_host_wake = pdata->gpio_host_wake;
	bt_lpm.int_host_wake = pdata->int_host_wake;
	bt_lpm.gpio_enable_bt = pdata->gpio_enable;

	ret = gpio_request(bt_lpm.gpio_enable_bt, pdev->name);
	if (ret < 0) {
		pr_err("%s: Unable to request gpio %d\n", __func__,
							bt_lpm.gpio_enable_bt);
		goto err_gpio_enable_req;
	}

	ret = gpio_direction_output(bt_lpm.gpio_enable_bt, 0);
	if (ret < 0) {
		pr_err("%s: Unable to set int direction for gpio %d\n",
					__func__, bt_lpm.gpio_enable_bt);
		goto err_gpio_enable_dir;
	}

#ifdef LPM_ON
	ret = gpio_request(bt_lpm.gpio_host_wake, pdev->name);
	if (ret < 0) {
		pr_err("%s: Unable to request gpio %d\n",
					__func__, bt_lpm.gpio_host_wake);
		goto err_gpio_host_wake_req;
	}

	ret = gpio_direction_input(bt_lpm.gpio_host_wake);
	if (ret < 0) {
		pr_err("%s: Unable to set direction for gpio %d\n", __func__,
							bt_lpm.gpio_host_wake);
		goto err_gpio_host_wake_dir;
	}

	ret = gpio_request(bt_lpm.gpio_wake, pdev->name);
	if (ret < 0) {
		pr_err("%s: Unable to request gpio %d\n", __func__,
							bt_lpm.gpio_wake);
		goto err_gpio_wake_req;
	}

	ret =  gpio_direction_output(bt_lpm.gpio_wake, 0);
	if (ret < 0) {
		pr_err("%s: Unable to set direction for gpio %d\n", __func__,
							bt_lpm.gpio_wake);
		goto err_gpio_wake_dir;
	}
#endif

	bt_lpm.port = pdata->port;
	bt_lpm.uart_disable = pdata->uart_disable;
	WARN_ON(!bt_lpm.uart_disable);
	bt_lpm.uart_enable = pdata->uart_enable;
	WARN_ON(!bt_lpm.uart_enable);

	bt_rfkill = rfkill_alloc("bcm43xx Bluetooth", &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bcm43xx_bt_rfkill_ops,
				NULL);
	if (unlikely(!bt_rfkill)) {
		ret = -ENOMEM;
		goto err_rfkill_alloc;
	}

	bcm43xx_bt_rfkill_set_power(NULL, default_state);
	rfkill_init_sw_state(bt_rfkill, default_state);

	ret = rfkill_register(bt_rfkill);
	if (unlikely(ret))
		goto err_rfkill_register;

#ifdef LPM_ON
	ret = bcm_bt_lpm_init(pdev);
	if (ret)
		goto err_lpm_init;
#endif

	return ret;

err_lpm_init:
	rfkill_unregister(bt_rfkill);
err_rfkill_register:
	rfkill_destroy(bt_rfkill);
err_rfkill_alloc:
#ifdef LPM_ON
err_gpio_wake_dir:
	gpio_free(bt_lpm.gpio_wake);
err_gpio_wake_req:
err_gpio_host_wake_dir:
	gpio_free(bt_lpm.gpio_host_wake);
err_gpio_host_wake_req:
#endif
err_gpio_enable_dir:
	gpio_free(bt_lpm.gpio_enable_bt);
err_gpio_enable_req:
	return ret;
}

static int bcm43xx_bluetooth_remove(struct platform_device *pdev)
{
	rfkill_unregister(bt_rfkill);
	rfkill_destroy(bt_rfkill);

	gpio_free(bt_lpm.gpio_enable_bt);
#ifdef LPM_ON
	gpio_free(bt_lpm.gpio_wake);
	gpio_free(bt_lpm.gpio_host_wake);
	wake_lock_destroy(&bt_lpm.wake_lock);
#endif
	return 0;
}
#ifdef LPM_ON
int bcm43xx_bluetooth_suspend(struct platform_device *pdev, pm_message_t state)
{
	int host_wake;

	pr_debug("%s\n", __func__);

	disable_irq(bt_lpm.int_host_wake);
	host_wake = gpio_get_value(bt_lpm.gpio_host_wake);
	if (host_wake) {
		enable_irq(bt_lpm.int_host_wake);
		pr_err("%s suspend error, gpio %d set\n", __func__,
							bt_lpm.gpio_host_wake);
		return -EBUSY;
	}

	return 0;
}

int bcm43xx_bluetooth_resume(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);

	enable_irq(bt_lpm.int_host_wake);
	return 0;
}
#endif

static struct platform_driver bcm43xx_bluetooth_platform_driver = {
	.probe = bcm43xx_bluetooth_probe,
	.remove = bcm43xx_bluetooth_remove,
#ifdef LPM_ON
	.suspend = bcm43xx_bluetooth_suspend,
	.resume = bcm43xx_bluetooth_resume,
#endif
	.driver = {
		   .name = "bcm_bt_lpm",
		   .owner = THIS_MODULE,
		   },
};

static int __init bcm43xx_bluetooth_init(void)
{
	bt_enabled = false;
	return platform_driver_register(&bcm43xx_bluetooth_platform_driver);
}

static void __exit bcm43xx_bluetooth_exit(void)
{
	platform_driver_unregister(&bcm43xx_bluetooth_platform_driver);
}


module_init(bcm43xx_bluetooth_init);
module_exit(bcm43xx_bluetooth_exit);

MODULE_ALIAS("platform:bcm43xx");
MODULE_DESCRIPTION("bcm43xx_bluetooth");
MODULE_AUTHOR("Jaikumar Ganesh <jaikumar@google.com>");
MODULE_LICENSE("GPL");

