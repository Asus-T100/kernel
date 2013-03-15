/*
 * iio_crystalcove_gpadc.c - Intel Merrifield Crystal Cove GPADC Driver
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
#include <linux/intel_mid_pm.h>
#include <linux/rpmsg.h>
#include <linux/debugfs.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <asm/intel_crystalcove_gpadc.h>

#include "../iio.h"
#include "../machine.h"
#include "../buffer.h"
#include "../driver.h"
#include "../types.h"
#include "../consumer.h"

#define ADCCNTL		0x6e
#define ADCVZSE		0x6f
#define ADCVGE		0x70
#define VRIMONCTL	0x71
#define MANCONV0	0x72
#define MANCONV1	0x73
#define ADCIRQ0		0x08
#define ADCIRQ1		0x09
#define MADCIRQ0	0x15
#define MADCIRQ1	0x16

static struct gpadc_regmap_t {
	char *name;
	int rslth;	/* GPADC Conversion Result Register Addr High */
	int rsltl;	/* GPADC Conversion Result Register Addr Low */
} gpadc_regmaps[GPADC_CH_NUM] = {
	{"VBAT",	0x80, 0x81, },
	{"BATID",	0x82, 0x83, },
	{"PMICTEMP",	0x7E, 0x7F, },
	{"BATTEMP0",	0x7A, 0x7B, },
	{"BATTEMP1",	0x7C, 0x7D, },
	{"SYSTEMP0",	0x74, 0x75, },
	{"SYSTEMP1",	0x76, 0x77, },
	{"SYSTEMP2",	0x78, 0x79, },
	{"VCCCUR",	0x84, 0x85, },
	{"VNNCUR",	0x86, 0x87, },
	{"V1P0ACUR",	0x88, 0x89, },
	{"V1P05SCUR",	0x8A, 0x8B, },
	{"VDDQCUR",	0x8C, 0x8D, },
};

struct gpadc_info {
	struct mutex lock;
	struct device *dev;
	int irq;
	wait_queue_head_t wait;
	int irq_pending;
};

#define ADC_CHANNEL(_type, _channel, _datasheet_name) \
	{				\
		.indexed = 1,		\
		.type = _type,		\
		.channel = _channel,	\
		.datasheet_name = _datasheet_name,	\
	}

static const struct iio_chan_spec const crystalcove_adc_channels[] = {
	ADC_CHANNEL(IIO_VOLTAGE, 0, "CH0"),
	ADC_CHANNEL(IIO_RESISTANCE, 1, "CH1"),
	ADC_CHANNEL(IIO_TEMP, 2, "CH2"),
	ADC_CHANNEL(IIO_TEMP, 3, "CH3"),
	ADC_CHANNEL(IIO_TEMP, 4, "CH4"),
	ADC_CHANNEL(IIO_TEMP, 5, "CH5"),
	ADC_CHANNEL(IIO_TEMP, 6, "CH6"),
	ADC_CHANNEL(IIO_TEMP, 7, "CH7"),
	ADC_CHANNEL(IIO_CURRENT, 8, "CH8"),
	ADC_CHANNEL(IIO_CURRENT, 9, "CH9"),
	ADC_CHANNEL(IIO_CURRENT, 10, "CH10"),
	ADC_CHANNEL(IIO_CURRENT, 11, "CH11"),
	ADC_CHANNEL(IIO_CURRENT, 12, "CH12"),
};

#define ADC_MAP(_adc_channel_label,			\
		     _consumer_dev_name,			\
		     _consumer_channel)				\
	{							\
		.adc_channel_label = _adc_channel_label,	\
		.consumer_dev_name = _consumer_dev_name,	\
		.consumer_channel = _consumer_channel,		\
	}

struct iio_map iio_maps[] = {
	ADC_MAP("CH0", "VIBAT", "VBAT"),
	ADC_MAP("CH1", "BATID", "BATID"),
	ADC_MAP("CH2", "PMICTEMP", "PMICTEMP"),
	ADC_MAP("CH3", "BATTEMP", "BATTEMP0"),
	ADC_MAP("CH4", "BATTEMP", "BATTEMP1"),
	ADC_MAP("CH5", "SYSTEMP", "SYSTEMP0"),
	ADC_MAP("CH6", "SYSTEMP", "SYSTEMP1"),
	ADC_MAP("CH7", "SYSTEMP", "SYSTEMP2"),
	ADC_MAP("CH5", "THERMAL", "SYSTEMP0"),
	ADC_MAP("CH6", "THERMAL", "SYSTEMP1"),
	ADC_MAP("CH7", "THERMAL", "SYSTEMP2"),
	ADC_MAP("CH2", "THERMAL", "PMICTEMP"),
	ADC_MAP("CH8", "CURRENT", "VCCCUR"),
	ADC_MAP("CH9", "CURRENT", "VNNCUR"),
	ADC_MAP("CH10", "CURRENT", "V1P0ACUR"),
	ADC_MAP("CH11", "CURRENT", "V1P05SCUR"),
	ADC_MAP("CH12", "CURRENT", "VDDQCUR"),
};

static irqreturn_t gpadc_isr(int irq, void *data)
{
	struct gpadc_info *info = iio_priv(data);
	u8 pending0, pending1;

	pending0 = intel_mid_pmic_readb(ADCIRQ0);
	pending1 = intel_mid_pmic_readb(ADCIRQ1);
	intel_mid_pmic_writeb(ADCIRQ0, pending0);
	intel_mid_pmic_writeb(ADCIRQ0, pending1);
	info->irq_pending |= pending0 + (pending1 << 8);
	wake_up(&info->wait);
	return IRQ_HANDLED;
}

/**
 * iio_crystalcove_gpadc_sample - do gpadc sample.
 * @indio_dev: industrial IO GPADC device handle
 * @ch: gpadc bit set of channels to sample, for example, set ch = (1<<0)|(1<<2)
 *	means you are going to sample both channel 0 and 2 at the same time.
 * @res:gpadc sampling result
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */
int iio_crystalcove_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res)
{
	struct gpadc_info *info = iio_priv(indio_dev);
	int i;
	int ret;
	int mask = 0;
	u8 th, tl;

	for (i = 0; i < GPADC_CH_NUM; i++) {
		if (ch & (1 << i))
			mask |= (1 << i);
	}
	mutex_lock(&info->lock);
	info->irq_pending = 0;
	intel_mid_pmic_setb(MANCONV0, (u8)mask);
	intel_mid_pmic_setb(MANCONV1, (u8)(mask >> 8));
	ret = wait_event_timeout(info->wait,
			((info->irq_pending & mask) == mask), HZ);
	if (ret == 0) {
		ret = -ETIMEDOUT;
		dev_err(info->dev, "sample timeout, return %d\n", ret);
		goto done;
	} else
		ret = 0;
	for (i = 0; i < GPADC_CH_NUM; i++) {
		if (ch & (1 << i)) {
			th = intel_mid_pmic_readb(gpadc_regmaps[i].rslth);
			tl = intel_mid_pmic_readb(gpadc_regmaps[i].rsltl);
			res->data[i] = ((th & 0x3) << 8) + tl;
		}
	}
done:
	mutex_unlock(&info->lock);
	return ret;
}
EXPORT_SYMBOL(iio_crystalcove_gpadc_sample);

static int crystalcove_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long m)
{
	int ret;
	int ch = chan->channel;
	struct gpadc_info *info = iio_priv(indio_dev);
	struct gpadc_result res;

	ret = iio_crystalcove_gpadc_sample(indio_dev, (1 << ch), &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		return -EINVAL;
	}

	*val = res.data[ch];

	return ret;
}

static int crystalcove_adc_read_all_raw(struct iio_channel *chan,
					int *val)
{
	int ret;
	int i, num = 0;
	int ch = 0;
	int *channels;
	struct gpadc_info *info = iio_priv(chan->indio_dev);
	struct gpadc_result res;

	while (chan[num].indio_dev)
		num++;

	channels = kzalloc(sizeof(int) * num, GFP_KERNEL);
	if (channels == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		channels[i] = chan[i].channel->channel;
		ch |= (1 << channels[i]);
	}

	ret = iio_crystalcove_gpadc_sample(chan->indio_dev, ch, &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		ret = -EINVAL;
		goto end;
	}

	for (i = 0; i < num; i++)
		val[i] = res.data[channels[i]];

end:
	kfree(channels);
	return ret;
}

static const struct iio_info crystalcove_adc_info = {
	.read_raw = &crystalcove_adc_read_raw,
	.read_all_raw = &crystalcove_adc_read_all_raw,
	.driver_module = THIS_MODULE,
};

static int __devinit crystalcove_gpadc_probe(struct platform_device *pdev)
{
	int err;
	struct gpadc_info *info;
	struct iio_dev *indio_dev;

	intel_mid_pmic_writeb(MADCIRQ0, 0x00);
	intel_mid_pmic_writeb(MADCIRQ1, 0x00);
	indio_dev = iio_allocate_device(sizeof(struct gpadc_info));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "allocating iio device failed\n");
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);
	mutex_init(&info->lock);
	init_waitqueue_head(&info->wait);
	info->dev = &pdev->dev;
	info->irq = platform_get_irq(pdev, 0);
	err = request_threaded_irq(info->irq, NULL, gpadc_isr,
			IRQF_ONESHOT, "adc", indio_dev);
	if (err) {
		dev_err(&pdev->dev, "unable to register irq %d\n", info->irq);
		goto err_free_device;
	}

	platform_set_drvdata(pdev, indio_dev);
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;
	indio_dev->channels = crystalcove_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(crystalcove_adc_channels);
	indio_dev->info = &crystalcove_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	err = iio_map_array_register(indio_dev, iio_maps);
	if (err)
		goto err_release_irq;

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto err_array_unregister;

	dev_info(&pdev->dev, "crystalcove adc probed\n");

	return 0;

err_array_unregister:
	iio_map_array_unregister(indio_dev, iio_maps);
err_release_irq:
	free_irq(info->irq, info);
err_free_device:
	iio_free_device(indio_dev);

	return err;
}

static int __devexit crystalcove_gpadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct gpadc_info *info = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev, iio_maps);
	free_irq(info->irq, info);
	iio_free_device(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int crystalcove_gpadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	mutex_lock(&info->lock);
	disable_irq(info->irq);
	return 0;
}

static int crystalcove_gpadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	enable_irq(info->irq);
	mutex_unlock(&info->lock);
	return 0;
}
#else
#define crystalcove_gpadc_suspend		NULL
#define crystalcove_gpadc_resume		NULL
#endif

static const struct dev_pm_ops crystalcove_gpadc_driver_pm_ops = {
	.suspend	= crystalcove_gpadc_suspend,
	.resume		= crystalcove_gpadc_resume,
};

static struct platform_driver crystalcove_gpadc_driver = {
	.probe = crystalcove_gpadc_probe,
	.remove = __devexit_p(crystalcove_gpadc_remove),
	.driver = {
		.name = "crystal_cove_adc",
		.pm = &crystalcove_gpadc_driver_pm_ops,
	},
};

#ifdef CONFIG_DEBUG_FS
static int crystalcove_gpadc_show(struct seq_file *s, void *unused)
{
	int num;
	char *names[GPADC_CH_NUM];
	int vals[GPADC_CH_NUM];
	int i, j;
	struct iio_channel *chs;
	char *ch_names[] = {"VIBAT", "BATID", "PMICTEMP", "BATTEMP",
		"SYSTEMP", "THERMAL"};
	int ret = 0;

	seq_printf(s, "ADCCNTL:\t0x%02x\n", intel_mid_pmic_readb(ADCCNTL));
	seq_printf(s, "ADCVZSE:\t0x%02x\n", intel_mid_pmic_readb(ADCVZSE));
	seq_printf(s, "ADCVGE:\t\t0x%02x\n", intel_mid_pmic_readb(ADCVGE));
	seq_printf(s, "VRIMONCTL:\t0x%02x\n", intel_mid_pmic_readb(VRIMONCTL));
	seq_printf(s, "MANCONV0:\t0x%02x\n", intel_mid_pmic_readb(MANCONV0));
	seq_printf(s, "MANCONV1:\t0x%02x\n", intel_mid_pmic_readb(MANCONV1));
	seq_printf(s, "ADCIRQ0:\t0x%02x\n", intel_mid_pmic_readb(ADCIRQ0));
	seq_printf(s, "ADCIRQ1:\t0x%02x\n", intel_mid_pmic_readb(ADCIRQ1));
	seq_printf(s, "MADCIRQ0:\t0x%02x\n", intel_mid_pmic_readb(MADCIRQ0));
	seq_printf(s, "MADCIRQ1:\t0x%02x\n", intel_mid_pmic_readb(MADCIRQ1));

	memset(names, 0, sizeof(names));
	for (i = 0; i < GPADC_CH_NUM; i++) {
		names[i] = kmalloc(256, GFP_KERNEL);
		if (names[i] == NULL) {
			ret = -ENOMEM;
			goto err;
		}
	}
	for (i = 0; i < sizeof(ch_names)/sizeof(ch_names[0]); i++) {
		chs = iio_st_channel_get_all(ch_names[i]);
		if (chs) {
			num = iio_st_channel_get_num(chs);
			iio_st_channel_get_name(chs, names);
			iio_st_read_channel_raw(chs, vals);
			iio_st_channel_release_all(chs);
			for (j = 0; j < num; j++)
				seq_printf(s, "%s:%s\t%d\n",
					ch_names[i], names[j], vals[j]);
		}
	}
err:
	for (i = 0; i < GPADC_CH_NUM; i++)
		kfree(names[i]);

	return 0;
}

static int crystalcove_gpadc_open(struct inode *inode, struct file *file)
{
	return single_open(file, crystalcove_gpadc_show, NULL);
}

static const struct file_operations crystalcove_gpadc_operations = {
	.open		= crystalcove_gpadc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int crystalcove_gpadc_debugfs_init(void)
{
	(void) debugfs_create_file("crystal_cove_adc", S_IFREG | S_IRUGO,
				NULL, NULL, &crystalcove_gpadc_operations);
	return 0;
}
module_init(crystalcove_gpadc_debugfs_init);
#endif

module_platform_driver(crystalcove_gpadc_driver);

MODULE_AUTHOR("Yang Bin<bin.yang@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Crystal Cove GPADC Driver");
MODULE_LICENSE("GPL");
