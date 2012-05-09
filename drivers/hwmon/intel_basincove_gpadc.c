/*
 * intel_basincove_gpadc.c - Intel Merrifield Basin Cove GPADC Driver
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
 * Author: Bin Yang <bin.yang@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/ipc_device.h>
#include <linux/intel_mid_pm.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_basincove_gpadc.h>

#define GPADCREQ	0xDC
#define GPADCREQ_IRQEN	(1 << 1)
#define GPADCREQ_BUSY	(1 << 0)
#define MIRQLVL1_ADC	(1 << 4)
#define MIRQLVL1	0x0C
#define ADC1CNTL	0xDD
#define ADCIRQ		0x06
#define MADCIRQ		0x11

static struct gpadc_regmap_t {
	char *name;
	int cntl;	/* GPADC Conversion Control Bit indicator */
	int rslth;	/* GPADC Conversion Result Register Addr High */
	int rsltl;	/* GPADC Conversion Result Register Addr Low */
} gpadc_regmaps[GPADC_CH_NUM] = {
	{"VBAT",	5, 0xE9, 0xEA, },
	{"BATID",	4, 0xEB, 0xEC, },
	{"IBAT",	5, 0xED, 0xEE, },
	{"BATTEMP0",	2, 0xC8, 0xC9, },
	{"BATTEMP1",	2, 0xCA, 0xCB, },
	{"SYSTEMP0",	3, 0xC2, 0xC3, },
	{"SYSTEMP1",	3, 0xC4, 0xC5, },
	{"SYSTEMP2",	3, 0xC6, 0xC7, },
	{"PMICTEMP",	3, 0xCC, 0xCD, },
};

struct gpadc_info {
	int initialized;
	/* This mutex protects gpadc sample/config from concurrent conflict.
	   Any function, which does the sample or config, needs to
	   hold this lock.
	   If it is locked, it also means the gpadc is in active mode.
	*/
	struct mutex lock;
	struct device *dev;
	int irq;
	u8 irq_status;
	wait_queue_head_t wait;
	int sample_done;
	void __iomem *intr;
};

static struct gpadc_info gpadc_info;

static inline int gpadc_clear_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0, mask);
}

static inline int gpadc_set_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0xff, mask);
}

static inline int gpadc_write(u16 addr, u8 data)
{
	return intel_scu_ipc_iowrite8(addr, data);
}

static inline int gpadc_read(u16 addr, u8 *data)
{
	return intel_scu_ipc_ioread8(addr, data);
}

static int gpadc_busy_wait(void)
{
	u8 tmp;
	int timeout = 0;

	gpadc_read(GPADCREQ, &tmp);
	while (tmp & GPADCREQ_BUSY && timeout < 500) {
		gpadc_read(GPADCREQ, &tmp);
		usleep_range(1800, 2000);
		timeout++;
	}

	if (tmp & GPADCREQ_BUSY)
		return -EBUSY;
	else
		return 0;
}

static void gpadc_dump(struct gpadc_info *info)
{
	u8 tmp;

	dev_err(info->dev, "GPADC registers dump:\n");
	gpadc_read(ADCIRQ, &tmp);
	dev_err(info->dev, "ADCIRQ: 0x%x\n", tmp);
	gpadc_read(MADCIRQ, &tmp);
	dev_err(info->dev, "MADCIRQ: 0x%x\n", tmp);
	gpadc_read(GPADCREQ, &tmp);
	dev_err(info->dev, "GPADCREQ: 0x%x\n", tmp);
	gpadc_read(ADC1CNTL, &tmp);
	dev_err(info->dev, "ADC1CNTL: 0x%x\n", tmp);
}

static irqreturn_t gpadc_isr(int irq, void *data)
{
	struct gpadc_info *info = data;

	info->irq_status = ioread8(info->intr);
	info->sample_done = 1;
	wake_up(&info->wait);
	return IRQ_HANDLED;
}

/**
 * intel_basincove_gpadc_sample - do gpadc sample.
 * @ch: gpadc bit set of channels to sample, for example, set ch = (1<<0)|(1<<2)
 *	means you are going to sample both channel 0 and 2 at the same time.
 * @res:gpadc sampling result
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */

int intel_basincove_gpadc_sample(int ch, struct gpadc_result *res)
{
	struct gpadc_info *info = &gpadc_info;
	int i, ret;
	u8 tmp, th, tl;

	if (!info->initialized)
		return -ENODEV;

	mutex_lock(&info->lock);

	tmp = GPADCREQ_IRQEN;

	for (i = 0; i < GPADC_CH_NUM; i++) {
		if (ch & (1 << i))
			tmp |= (1 << gpadc_regmaps[i].cntl);
	}

	info->sample_done = 0;

	ret = gpadc_busy_wait();
	if (ret) {
		dev_err(info->dev, "GPADC is busy\n");
		goto done;
	}

	gpadc_write(GPADCREQ, tmp);

	ret = wait_event_timeout(info->wait, info->sample_done, HZ);
	if (ret == 0) {
		gpadc_dump(info);
		ret = -ETIMEDOUT;
		dev_err(info->dev, "sample timeout, return %d\n", ret);
		goto done;
	} else {
		ret = 0;
	}

	for (i = 0; i < GPADC_CH_NUM; i++) {
		if (ch & (1 << i)) {
			gpadc_read(gpadc_regmaps[i].rslth, &th);
			gpadc_read(gpadc_regmaps[i].rsltl, &tl);
			res->data[i] = ((th & 0x3) << 8) + tl;
		}
	}

done:
	gpadc_clear_bits(MIRQLVL1, MIRQLVL1_ADC);
	mutex_unlock(&info->lock);
	return ret;
}
EXPORT_SYMBOL(intel_basincove_gpadc_sample);

static struct gpadc_result sample_result;
static int chs;

static ssize_t intel_basincove_gpadc_store_channel(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	if (sscanf(buf, "%x", &chs) != 1) {
		dev_err(dev, "one channel argument is needed\n");
		return -EINVAL;
	}

	if (chs < (1 << 0) || chs >= (1 << GPADC_CH_NUM)) {
		dev_err(dev, "invalid channel, should be in [0x1 - 0x1FF]\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_basincove_gpadc_show_channel(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", chs);
}

static ssize_t intel_basincove_gpadc_store_sample(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
	int value, ret;

	memset(sample_result.data, 0, sizeof(sample_result.data));

	if (sscanf(buf, "%d", &value) != 1) {
		dev_err(dev, "one argument is needed\n");
		return -EINVAL;
	}

	if (value == 1) {
		ret = intel_basincove_gpadc_sample(chs, &sample_result);
		if (ret) {
			dev_err(dev, "sample failed\n");
			return ret;
		}
	} else {
		dev_err(dev, "input '1' to sample\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t intel_basincove_gpadc_show_result(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	int used = 0;

	for (i = 0; i < GPADC_CH_NUM; i++) {
		used += snprintf(buf + used, PAGE_SIZE - used,
			 "sample_result[%d] = %d\n", i, sample_result.data[i]);
	}

	return used;
}


static DEVICE_ATTR(channel, S_IWUSR | S_IRUGO,
		intel_basincove_gpadc_show_channel,
		intel_basincove_gpadc_store_channel);
static DEVICE_ATTR(sample, S_IWUSR, NULL, intel_basincove_gpadc_store_sample);
static DEVICE_ATTR(result, S_IRUGO, intel_basincove_gpadc_show_result, NULL);

static struct attribute *intel_basincove_gpadc_attrs[] = {
	&dev_attr_channel.attr,
	&dev_attr_sample.attr,
	&dev_attr_result.attr,
	NULL,
};

static struct attribute_group intel_basincove_gpadc_attr_group = {
	.name = "basincove_gpadc",
	.attrs = intel_basincove_gpadc_attrs,
};

static int __devinit gpadc_probe(struct ipc_device *ipcdev)
{
	struct gpadc_info *info = &gpadc_info;
	struct intel_basincove_gpadc_platform_data *pdata =
			ipcdev->dev.platform_data;
	int err;
	u8 mask;

	if (!pdata) {
		dev_err(&ipcdev->dev, "no platform data supplied\n");
		return -EINVAL;
	}

	mutex_init(&info->lock);
	init_waitqueue_head(&info->wait);
	info->dev = &ipcdev->dev;
	info->irq = ipc_get_irq(ipcdev, 0);
	info->intr = ioremap_nocache(pdata->intr, 1);
	if (!info->intr) {
		dev_err(&ipcdev->dev, "ioremap of ADCIRQ failed\n");
		err = -ENOMEM;
		goto exit;
	}

	mask = MBATTEMP | MSYSTEMP | MBATT | MVIBATT | MCCTICK;
	gpadc_clear_bits(MADCIRQ, mask);
	gpadc_clear_bits(MIRQLVL1, MIRQLVL1_ADC);

	err = request_threaded_irq(info->irq, NULL, gpadc_isr,
			IRQF_ONESHOT, "adc", info);
	if (err) {
		gpadc_dump(info);
		dev_err(&ipcdev->dev, "unable to register irq %d\n", info->irq);
		goto err_iounmap;
	}

	err = sysfs_create_group(&ipcdev->dev.kobj,
			&intel_basincove_gpadc_attr_group);
	if (err) {
		dev_err(&ipcdev->dev, "Unable to export sysfs interface, error: %d\n",
			err);
		goto err_release_irq;
	}

	info->initialized = 1;

	return 0;

err_release_irq:
	free_irq(info->irq, info);
err_iounmap:
	iounmap(info->intr);
exit:
	return err;
}

static int __devexit gpadc_remove(struct ipc_device *ipcdev)
{
	struct gpadc_info *info = &gpadc_info;

	sysfs_remove_group(&ipcdev->dev.kobj,
			&intel_basincove_gpadc_attr_group);
	free_irq(info->irq, info);
	iounmap(info->intr);
	return 0;
}

#ifdef CONFIG_PM
static int gpadc_suspend(struct device *dev)
{
	struct gpadc_info *info = &gpadc_info;

	if (mutex_trylock(&info->lock))
		return 0;
	else
		return -EBUSY;
}

static int gpadc_resume(struct device *dev)
{
	struct gpadc_info *info = &gpadc_info;

	mutex_unlock(&info->lock);
	return 0;
}
#else
#define gpadc_suspend		NULL
#define gpadc_resume		NULL
#endif

static const struct dev_pm_ops gpadc_driver_pm_ops = {
	.suspend	= gpadc_suspend,
	.resume		= gpadc_resume,
};

static struct ipc_driver gpadc_driver = {
	.driver = {
		   .name = "bcove_adc",
		   .owner = THIS_MODULE,
		   .pm = &gpadc_driver_pm_ops,
		   },
	.probe = gpadc_probe,
	.remove = __devexit_p(gpadc_remove),
};

static int __init gpadc_module_init(void)
{
	return ipc_driver_register(&gpadc_driver);
}

static void __exit gpadc_module_exit(void)
{
	ipc_driver_unregister(&gpadc_driver);
}

module_init(gpadc_module_init);
module_exit(gpadc_module_exit);

MODULE_AUTHOR("Yang Bin<bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Basin Cove GPADC Driver");
MODULE_LICENSE("GPL");
