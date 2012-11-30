/*
 * intel_mid_thermal_probe.c - Intel MID thermal probe driver
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Jason Chen <jason.cj.chen@intel.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/notifier.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/intel_mid_thermal_probe.h>
#include <asm/intel_mid_gpadc.h>

#define THERMAL_SENSOR		1
#define MIN_INTERVAL_MS		1000
#define ADC_ONE_LSB_MULTIPLIER	2346

#define USER_ENABLE	(1 << 1)
#define HW_ENABLE	(1 << 2)
#define	THERM_RUNNING	(1 << 3)

struct thermal_probe_dev {
	struct device *dev;
	struct mutex lock;
	struct miscdevice misc_dev;
	struct notifier_block nb;
	int adc_ch;
	int user_en;
	int mV;
	int status;
	int interval;
	void *adc_handle;
	struct delayed_work delay_work;
	wait_queue_head_t workq_head;
	struct list_head list;
};

struct thermal_client {
	struct thermal_probe_dev *thermal;
	struct list_head list;
	bool data_ready;
	bool enable;
};

static BLOCKING_NOTIFIER_HEAD(mid_thermal_notifier_list);

int mid_thermal_probe_register_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(
			&mid_thermal_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mid_thermal_probe_register_notify);

int mid_thermal_probe_unregister_notify(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(
			&mid_thermal_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mid_thermal_probe_unregister_notify);

int mid_thermal_probe_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(
			&mid_thermal_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(mid_thermal_probe_notifier_call_chain);

static void thermal_work_func(struct work_struct *work)
{
	struct thermal_probe_dev *thermal =
			container_of((struct delayed_work *)work,
			struct thermal_probe_dev, delay_work);
	struct thermal_client *client;
	int ret;

	mutex_lock(&thermal->lock);
	if (thermal->status & THERM_RUNNING) {
		ret = intel_mid_gpadc_sample(thermal->adc_handle, 1,
				&thermal->mV);
		if (ret)
			dev_err(thermal->dev, "gpadc sample fail\n");
		else {
			thermal->mV = (thermal->mV * ADC_ONE_LSB_MULTIPLIER)
					/ 1000;
			list_for_each_entry(client, &thermal->list, list) {
				if (client->enable)
					client->data_ready = true;
			}

			wake_up(&thermal->workq_head);
			schedule_delayed_work(&thermal->delay_work,
					msecs_to_jiffies(thermal->interval));
		}
	}
	mutex_unlock(&thermal->lock);
}

/* check if the probe need run or stop, call with mutex lock*/
static int thermal_check_enable(struct thermal_probe_dev *thermal)
{
	/* goto run/stop */
	if (!(thermal->status & THERM_RUNNING) &&
		(thermal->status & USER_ENABLE) &&
		(thermal->status & HW_ENABLE)) {
		thermal->adc_handle = intel_mid_gpadc_alloc(THERMAL_SENSOR,
				thermal->adc_ch);
		if (!thermal->adc_handle) {
			dev_err(thermal->dev, "gpadc alloc fail\n");
			return -ENODEV;
		}
		thermal->status |= THERM_RUNNING;
		schedule_delayed_work(&thermal->delay_work,
				msecs_to_jiffies(thermal->interval));
	} else if ((thermal->status & THERM_RUNNING) &&
		(!(thermal->status & USER_ENABLE) ||
		!(thermal->status & HW_ENABLE))) {
		struct thermal_client *client;

		intel_mid_gpadc_free(thermal->adc_handle);
		thermal->adc_handle = NULL;
		/* give out one invalid value */
		thermal->mV = -1;
		list_for_each_entry(client, &thermal->list, list) {
			if (client->enable)
				client->data_ready = true;
		}
		wake_up(&thermal->workq_head);
		thermal->status &= ~THERM_RUNNING;
	}
	return 0;
}

static int thermal_open(struct inode *inode, struct file *filep)
{
	struct thermal_probe_dev *thermal =
		container_of(filep->private_data,
		struct thermal_probe_dev, misc_dev);
	struct thermal_client *client;

	client = kzalloc(sizeof(struct thermal_client), GFP_KERNEL);
	if (!client) {
		dev_err(thermal->dev, "Thermal client kzalloc failed!\n");
		return -ENOMEM;
	}
	client->thermal = thermal;
	mutex_lock(&thermal->lock);
	list_add(&client->list, &thermal->list);
	mutex_unlock(&thermal->lock);
	filep->private_data = client;

	return 0;
}

static ssize_t
thermal_read(struct file *filep, char __user *buffer,
			size_t size, loff_t *offset)
{
	struct thermal_client *client = filep->private_data;
	struct thermal_probe_dev *thermal = client->thermal;
	int ret = -ENODEV;

	mutex_lock(&thermal->lock);
	if (client->data_ready) {
		if (thermal->mV >= 0) {
			ret = sizeof(thermal->mV);
			if (copy_to_user(buffer, &thermal->mV,
						sizeof(thermal->mV)))
				ret = -EFAULT;
		}
		client->data_ready = false;
	}
	mutex_unlock(&thermal->lock);

	return ret;
}

static unsigned int
thermal_poll(struct file *filep, struct poll_table_struct *wait)
{
	struct thermal_client *client = filep->private_data;
	struct thermal_probe_dev *thermal = client->thermal;
	unsigned int mask = 0;

	poll_wait(filep, &thermal->workq_head, wait);

	if (client->data_ready)
		mask |= (POLLIN | POLLRDNORM);

	return mask;
}

static long thermal_ioctl(struct file *filep, unsigned int cmd,
				unsigned long arg)
{
	struct thermal_client *client = filep->private_data;
	struct thermal_probe_dev *thermal = client->thermal;
	long ret = 0;

	mutex_lock(&thermal->lock);
	/*user enable/disable */
	if (arg) {
		if (!client->enable) {
			client->enable = true;
			if (0 == thermal->user_en++) {
				thermal->status |= USER_ENABLE;
				ret = thermal_check_enable(thermal);
			}
		}
	} else {
		if (client->enable) {
			client->enable = false;
			if (0 == --thermal->user_en) {
				thermal->status &= ~USER_ENABLE;
				ret = thermal_check_enable(thermal);
			}
		}
	}
	mutex_unlock(&thermal->lock);

	return ret;
}

static int thermal_close(struct inode *inode, struct file *filep)
{
	struct thermal_client *client = filep->private_data;
	struct thermal_probe_dev *thermal = client->thermal;

	mutex_lock(&thermal->lock);
	if (client->enable)
		thermal->user_en--;
	if (!thermal->user_en && thermal->status & THERM_RUNNING) {
		thermal->status &= ~USER_ENABLE;
		thermal_check_enable(thermal);
	}
	list_del(&client->list);
	mutex_unlock(&thermal->lock);

	kfree(client);
	filep->private_data = NULL;

	return 0;
}

static const struct file_operations thermal_fops = {
	.owner = THIS_MODULE,
	.open = thermal_open,
	.read = thermal_read,
	.poll = thermal_poll,
	.release = thermal_close,
	.unlocked_ioctl = thermal_ioctl,
	.llseek = no_llseek,
};

/*sysfs*/
static ssize_t thermal_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct thermal_probe_dev *thermal = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", thermal->interval);
}

static ssize_t thermal_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct thermal_probe_dev *thermal = platform_get_drvdata(pdev);
	unsigned long interval;

	if (kstrtoul(buf, 10, &interval))
		return -EINVAL;

	interval = (interval > MIN_INTERVAL_MS) ? interval : MIN_INTERVAL_MS;

	mutex_lock(&thermal->lock);
	thermal->interval = interval;
	mutex_unlock(&thermal->lock);

	return count;
}

static ssize_t thermal_run_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct thermal_probe_dev *thermal = platform_get_drvdata(pdev);
	int run = 0;

	if (thermal->status & THERM_RUNNING)
		run = 1;

	return sprintf(buf, "%d\n", run);
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, thermal_delay_show,
		thermal_delay_store);
static DEVICE_ATTR(run, S_IRUGO, thermal_run_show, NULL);

static struct attribute *thermal_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_run.attr,
	NULL
};

static struct attribute_group thermal_attribute_group = {
	.attrs = thermal_attributes
};

static int thermal_probe_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct thermal_probe_dev *thermal =
		container_of(self, struct thermal_probe_dev, nb);
	int ret = 0;

	mutex_lock(&thermal->lock);
	switch (event) {
	case EVENT_THERM_PLUG_IN:
		thermal->status |= HW_ENABLE;
		break;
	case EVENT_THERM_PULL_OUT:
		thermal->status &= ~HW_ENABLE;
		break;
	default:
		dev_warn(thermal->dev, "not support notifier event\n");
		goto out;
	}
	ret = thermal_check_enable(thermal);
out:
	mutex_unlock(&thermal->lock);
	return ret;
}

static int __devinit mid_thermal_probe(struct platform_device *pdev)
{
	struct thermal_probe_platform_data *pdata = pdev->dev.platform_data;
	struct thermal_probe_dev *thermal;
	int ret = 0;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -EINVAL;
	}

	thermal = kzalloc(sizeof(struct thermal_probe_dev), GFP_KERNEL);
	if (!thermal)
		return -ENOMEM;

	INIT_DELAYED_WORK(&thermal->delay_work, thermal_work_func);
	INIT_LIST_HEAD(&thermal->list);
	init_waitqueue_head(&thermal->workq_head);
	mutex_init(&thermal->lock);
	thermal->adc_ch = pdata->adc_ch;
	thermal->dev = &pdev->dev;
	thermal->interval = MIN_INTERVAL_MS;

	thermal->misc_dev.minor = MISC_DYNAMIC_MINOR;
	thermal->misc_dev.name = "thermal_probe";
	thermal->misc_dev.fops = &thermal_fops;
	ret = misc_register(&thermal->misc_dev);
	if (ret) {
		dev_err(&pdev->dev, "miscdev register failed\n");
		goto misc_register_fail;
	}

	thermal->nb.notifier_call = thermal_probe_notifier_callback;
	ret = mid_thermal_probe_register_notify(&thermal->nb);
	if (ret != 0) {
		dev_err(&pdev->dev, "thermal notify register failed\n");
		goto notify_register_fail;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &thermal_attribute_group);
	if (ret) {
		dev_err(&pdev->dev, "sysfs can not create group\n");
		goto sysfs_create_fail;
	}

	platform_set_drvdata(pdev, thermal);

	return ret;

sysfs_create_fail:
	mid_thermal_probe_unregister_notify(&thermal->nb);
notify_register_fail:
	misc_deregister(&thermal->misc_dev);
misc_register_fail:
	kfree(thermal);
	return ret;
}

static int __devexit mid_thermal_remove(struct platform_device *pdev)
{
	struct thermal_probe_dev *thermal = platform_get_drvdata(pdev);

	if (thermal->status & THERM_RUNNING) {
		mutex_lock(&thermal->lock);
		thermal->status &= ~USER_ENABLE;
		thermal_check_enable(thermal);
		mutex_unlock(&thermal->lock);
	}

	sysfs_remove_group(&pdev->dev.kobj, &thermal_attribute_group);
	mid_thermal_probe_unregister_notify(&thermal->nb);
	misc_deregister(&thermal->misc_dev);
	kfree(thermal);
	return 0;
}

static struct platform_driver mid_thermal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mid_thermal_probe,
	.remove = __devexit_p(mid_thermal_remove),
};

static int __init mid_thermal_init(void)
{
	return platform_driver_register(&mid_thermal_driver);
}

static void __exit mid_thermal_exit(void)
{
	platform_driver_unregister(&mid_thermal_driver);
}

/*
 * load _after_ the GPADC driver
 */
late_initcall(mid_thermal_init);
module_exit(mid_thermal_exit);

MODULE_AUTHOR("Jason Chen <jason.cj.chen@intel.com>");
MODULE_DESCRIPTION("Intel MID Thermal Probe Driver");
MODULE_LICENSE("GPL v2");
