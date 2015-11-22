/*
 * ASUS Lid driver.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>

#include "lid.h"

#define CONVERSION_TIME_MS     50

static struct input_dev *lid_indev;
static struct workqueue_struct *lid_wq;
static struct delayed_work lid_hall_sensor_work;

static unsigned int hall_sensor_gpio = 0;
static unsigned int hall_sensor_irq  = 0;

static ssize_t show_lid_status(struct device *class,
		struct device_attribute *attr, char *buf)
{
	char *s = buf;

	s += sprintf(buf, "%u\n",
			gpio_get_value_cansleep(hall_sensor_gpio) ? 1 : 0);

	return s - buf;
}
static DEVICE_ATTR(lid_status, S_IWUSR | S_IRUGO, show_lid_status, NULL);

static struct attribute *lid_attrs[] = {
	&dev_attr_lid_status.attr,
	NULL
};

static struct attribute_group lid_attr_group = {
	.attrs = lid_attrs,
};

static irqreturn_t lid_interrupt_handler(int irq, void *dev_id)
{
	if (irq != 0)
		queue_delayed_work(lid_wq, &lid_hall_sensor_work, 0);
	else
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static void lid_report_function(struct work_struct *dat)
{
	int value = 0;

	if (!lid_indev) {
		pr_err("hall_sensor: LID input device doesn't exist\n");
		return;
	}

	msleep(CONVERSION_TIME_MS);
	value = gpio_get_value_cansleep(hall_sensor_gpio) ? 1 : 0;
	input_report_switch(lid_indev, SW_LID, !value);
	input_sync(lid_indev);

	pr_notice("hall_sensor: SW_LID report value = %d\n", value);
}

static int lid_input_device_create(void)
{
	int err = 0;

	lid_indev = input_allocate_device();
	if (!lid_indev) {
		pr_err("hall_sensor: fail to allocate lid_indev\n");
		err = -ENOMEM;
		goto exit;
	}

	lid_indev->name = "lid_input";
	lid_indev->phys = "/dev/input/lid_indev";

	set_bit(EV_SW, lid_indev->evbit);
	set_bit(SW_LID, lid_indev->swbit);

	err = input_register_device(lid_indev);
	if (err) {
		pr_err("hall_sensor: lid_indev registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(lid_indev);
	lid_indev = NULL;
exit:
	return err;
}

static int __init lid_driver_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (!pdev)
		return -EINVAL;

	pr_info("hall_sensor: %s\n", __func__);

	ret = sysfs_create_group(&pdev->dev.kobj, &lid_attr_group);
	if (ret) {
		pr_err("hall_sensor: Unable to create sysfs, err: %d\n", ret);
		goto fail_sysfs;
	}

	ret = lid_input_device_create();
	if (ret) {
		pr_err("hall_sensor: Unable to register input device, err: %d\n", ret);
		goto fail_create;
	}

	lid_wq = create_singlethread_workqueue("lid_wq");
	if (!lid_wq) {
		pr_err("hall_sensor: Unable to create workqueue\n");
		goto fail_create;
	}

	hall_sensor_gpio = acpi_get_gpio(LID_UEFI_GPIO_NAME, LID_UEFI_GPIO_OFFSET);
	if (!gpio_is_valid(hall_sensor_gpio)) {
		pr_err("hall_sensor: Invalid GPIO %d\n", hall_sensor_gpio);
		goto fail_create;
	}

	ret = gpio_request(hall_sensor_gpio, "ASUS_LID");
	if (ret < 0) {
		pr_err("hall_sensor: Failed to request GPIO %d\n", hall_sensor_gpio);
		goto fail_create;
	}

	ret = gpio_direction_input(hall_sensor_gpio);
	if (ret < 0) {
		pr_err("hall_sensor: Failed to configure gpio direction: %d\n",
				hall_sensor_gpio);
		goto fail_free;
	}

	hall_sensor_irq = gpio_to_irq(hall_sensor_gpio);
	printk("hall_sensor: gpio[%d] to irq[%d]\n",
			hall_sensor_gpio, hall_sensor_irq);// 208 to 504
	if (hall_sensor_irq < 0) {
		pr_err("hall_sensor: Unable to get irq number for GPIO %d\n",
				hall_sensor_gpio);
		goto fail_free;
	}

	ret = request_any_context_irq(hall_sensor_irq, lid_interrupt_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"hall_sensor_irq", lid_indev);
	if (ret < 0) {
		pr_err("hall_sensor: Unable to claim irq %d\n", hall_sensor_irq);
		goto fail_free;
	}

	device_init_wakeup(&pdev->dev, 1);
	enable_irq_wake(hall_sensor_irq);

	INIT_DEFERRABLE_WORK(&lid_hall_sensor_work, lid_report_function);

	return ret;

fail_free:
	gpio_free(hall_sensor_gpio);

fail_create:
	sysfs_remove_group(&pdev->dev.kobj, &lid_attr_group);

fail_sysfs:
	return ret;
}

static int __exit lid_driver_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &lid_attr_group);
	free_irq(hall_sensor_irq, NULL);
	cancel_delayed_work_sync(&lid_hall_sensor_work);

	if (gpio_is_valid(hall_sensor_gpio))
		gpio_free(hall_sensor_gpio);

	input_unregister_device(lid_indev);
	device_init_wakeup(&pdev->dev, 0);

	return 0;
}

static const struct acpi_device_id hall_acpi_match[] = {
	{ "APX9131", 0 },
	{ "YOB8251", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, inv_mpu_id);

static struct platform_driver asustek_lid_driver __refdata = {
	.probe = lid_driver_probe,
	.remove = __exit_p(lid_driver_remove),
	.driver = {
		.name = "asustek_lid",
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(hall_acpi_match),
	},
};

static int __init asustek_lid_driver_init(void)
{
	printk("hall_sensor: platform driver initialization\n");
	return platform_driver_register(&asustek_lid_driver);
}

static void __exit asustek_lid_driver_exit(void)
{
	platform_driver_unregister(&asustek_lid_driver);
}

late_initcall(asustek_lid_driver_init);
module_exit(asustek_lid_driver_exit);

MODULE_DESCRIPTION("Hall Sensor Driver");
MODULE_LICENSE("GPL");
