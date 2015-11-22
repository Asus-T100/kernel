/*
 * iio_dc_ti_gpadc.c - Dollar Cove(Xpower) GPADC Driver
 *
 * Copyright (C) 2014 Intel Corporation
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
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
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
#include <linux/iio/adc/dc_ti_gpadc.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/types.h>
#include <linux/iio/consumer.h>

#define DC_ADC_IRQ_MASK_REG		0x02
#define IRQ_MASK_ADC			(1 << 2)

#define DC_PMIC_ADC_CNTL_REG		0x50
#define CNTL_ADC_START			(1 << 0)
#define CNTL_ADC_CH_SEL_MASK		0x06
#define CNTL_ADC_CH_SEL_VBAT		(0 << 1)
#define CNTL_ADC_CH_SEL_PMICTEMP	(1 << 1)
#define CNTL_ADC_CH_SEL_BPTHERM		(2 << 1)
#define CNTL_ADC_CH_SEL_SYSTHERM	(3 << 1)
#define CNTL_ADC_EN			(1 << 5)
#define CNTL_EN_EXT_BPTH_BIAS		(1 << 7)

#define ADC_VBAT_DATAH_REG		0x54
#define ADC_VBAT_DATAL_REG		0x55
#define ADC_PMICTEMP_DATAH_REG		0x56
#define ADC_PMICTEMP_DATAL_REG		0x57
#define ADC_BPTHERM_DATAH_REG		0x58
#define ADC_BPTHERM_DATAL_REG		0x59
#define ADC_SYSTH_DATAH_REG		0x5A
#define ADC_SYSTH_DATAL_REG		0x5B

#define ADC_CHANNEL0_MASK		(1 << 0)
#define ADC_CHANNEL1_MASK		(1 << 1)
#define ADC_CHANNEL2_MASK		(1 << 2)
#define ADC_CHANNEL3_MASK		(1 << 3)

#define DEV_NAME			"dollar_cove_ti_adc"

#define SW_TRIM_VAL		1

static struct gpadc_regmap_t {
	char *name;
	int rslth;	/* GPADC Conversion Result Register Addr High */
	int rsltl;	/* GPADC Conversion Result Register Addr Low */
} gpadc_regmaps[GPADC_CH_NUM] = {
	{"VBAT",	0x54, 0x55, },
	{"PMICTEMP",	0x56, 0x57, },
	{"BATTEMP",	0x58, 0x59, },
	{"SYSTEMP0",	0x5A, 0x5B, },
};

struct gpadc_info {
	struct mutex lock;
	struct device *dev;
	int irq;
	wait_queue_head_t wait;
};

#define ADC_CHANNEL(_type, _channel, _datasheet_name) \
	{				\
		.indexed = 1,		\
		.type = _type,		\
		.channel = _channel,	\
		.datasheet_name = _datasheet_name,	\
	}

static const struct iio_chan_spec const dc_ti_adc_channels[] = {
	ADC_CHANNEL(IIO_VOLTAGE, 0, "CH0"),
	ADC_CHANNEL(IIO_TEMP, 1, "CH1"),
	ADC_CHANNEL(IIO_TEMP, 2, "CH2"),
	ADC_CHANNEL(IIO_TEMP, 3, "CH3"),
};

#define ADC_MAP(_adc_channel_label,			\
		     _consumer_dev_name,			\
		     _consumer_channel)				\
	{							\
		.adc_channel_label = _adc_channel_label,	\
		.consumer_dev_name = _consumer_dev_name,	\
		.consumer_channel = _consumer_channel,		\
	}

static struct iio_map iio_maps[] = {
	ADC_MAP("CH0", "VIBAT", "VBAT"),
	ADC_MAP("CH1", "THERMAL", "PMICTEMP"),
	ADC_MAP("CH2", "THERMAL", "BATTEMP"),
	ADC_MAP("CH3", "THERMAL", "SYSTEMP0"),
	ADC_MAP("CH1", "byt_cr_thermal", "PMICTEMP"),
	ADC_MAP("CH3", "byt_cr_thermal", "SYSTEMP0"),
};

static irqreturn_t dc_ti_gpadc_isr(int irq, void *data)
{
	struct gpadc_info *info = data;

	wake_up(&info->wait);
	return IRQ_HANDLED;
}

/**
 * iio_dc_ti_gpadc_sample - do gpadc sample.
 * @indio_dev: industrial IO GPADC device handle
 * @ch: gpadc bit set of channels to sample, for example, set ch = (1<<0)|(1<<2)
 *	means you are going to sample both channel 0 and 2 at the same time.
 * @res:gpadc sampling result
 *
 * Returns 0 on success or an error code.
 *
 * This function may sleep.
 */
int iio_dc_ti_gpadc_sample(struct iio_dev *indio_dev,
				int ch, struct gpadc_result *res)
{
	struct gpadc_info *info = iio_priv(indio_dev);
	int i, ret, adc_en_mask;
	u8 th, tl;

	/* prepare ADC channel enable mask */
	if (ch & ADC_CHANNEL0_MASK)
		adc_en_mask = CNTL_ADC_CH_SEL_VBAT;
	else if (ch & ADC_CHANNEL1_MASK)
		adc_en_mask = CNTL_ADC_CH_SEL_PMICTEMP;
	else if (ch & ADC_CHANNEL2_MASK)
		adc_en_mask = CNTL_ADC_CH_SEL_BPTHERM;
	else if (ch & ADC_CHANNEL3_MASK)
		adc_en_mask = CNTL_ADC_CH_SEL_SYSTHERM;
	else
		return -EINVAL;

	mutex_lock(&info->lock);

	/*
	 * If channel BPTHERM or Systherm has been selected, first enable the BIAS
	 * which provides the VREFT Voltage reference to convert Input
	 * voltage to temperature.
	 * As per PMIC Vendor specifications, BPTHERM BIAS should be enabled
	 * 35msecs before ADC_EN command for all the temperatures.
	 */
	if ((adc_en_mask & CNTL_ADC_CH_SEL_BPTHERM) ||
			(adc_en_mask & CNTL_ADC_CH_SEL_SYSTHERM)) {
		ret = intel_mid_pmic_setb(DC_PMIC_ADC_CNTL_REG,
				(u8)CNTL_EN_EXT_BPTH_BIAS);
		if (ret < 0)
			goto done;
		msleep(35);
	}
	/*As per the TI(PMIC Vendor), the ADC enable and ADC start commands
	should not be sent together. Hence sending the commands separately*/
	/* enable ADC channels */
	ret = intel_mid_pmic_setb(DC_PMIC_ADC_CNTL_REG, (u8)CNTL_ADC_EN);
	if (ret < 0)
		goto done;

	/*Now set the channel number */
	ret = intel_mid_pmic_setb(DC_PMIC_ADC_CNTL_REG, (u8)adc_en_mask);
	if (ret < 0)
		goto done;

	/*
	 * As per PMIC Vendor, a minimum of 50 micro seconds delay is required
	 * between ADC Enable and ADC START commands. This is also recommended
	 * by Intel Hardware team after the timing analysis of GPADC signals.
	 * Since the I2C Write trnsaction to set the channel number also
	 * imparts 25 micro seconds of delay, so we need to wait for another
	 * 25 micro seconds before issuing ADC START command.
	 */
	usleep_range(25, 40);

	/*Start the ADC conversion for the selected channel */
	ret = intel_mid_pmic_setb(DC_PMIC_ADC_CNTL_REG, (u8)CNTL_ADC_START);
	if (ret < 0)
		goto done;

	if (info->irq >= 0) {
		/*TI(PMIC Vendor) recommends 5 sec timeout for conversion*/
		ret = wait_event_timeout(info->wait, 1, 5 * HZ);
		if (ret == 0) {
			ret = -ETIMEDOUT;
			dev_err(info->dev, "sample timeout, return %d\n", ret);
			goto done;
		} else
			ret = 0;
	} else {
		/*
		 * As TI PMIC doesn't have any status bits to check
		 * need to wait for atleast 50mSec to for the ADC
		 * conversion to get completed.
		 */
		msleep(50);
	}

	for (i = 0; i < GPADC_CH_NUM; i++) {
		if (ch & (1 << i)) {
			th = intel_mid_pmic_readb(gpadc_regmaps[i].rslth);
			tl = intel_mid_pmic_readb(gpadc_regmaps[i].rsltl);
			/*
			 * TI $cove ADC output registers are 10bit wide
			 * So the result's DATAH should be maksed(0x03)
			 * and shifted(8 bits) before adding to the DATAL.
			 */
			/*As per TI, The PMIC Silicon need S/W Trim.
			Hence applying TRIM value*/
			res->data[i] = (((th & 0x3) << 8) + tl) - SW_TRIM_VAL;
		}
	}
	/*Clear IRQ Register */
	intel_mid_pmic_clearb(DC_ADC_IRQ_MASK_REG, IRQ_MASK_ADC);
	/* disable ADC channels */
	intel_mid_pmic_clearb(DC_PMIC_ADC_CNTL_REG, (u8)(adc_en_mask |
				CNTL_ADC_START | CNTL_ADC_EN));
	if ((adc_en_mask & CNTL_ADC_CH_SEL_BPTHERM) ||
			(adc_en_mask & CNTL_ADC_CH_SEL_SYSTHERM))
		intel_mid_pmic_clearb(DC_PMIC_ADC_CNTL_REG, CNTL_EN_EXT_BPTH_BIAS);
done:
	mutex_unlock(&info->lock);
	return 0;
}
EXPORT_SYMBOL(iio_dc_ti_gpadc_sample);

static int dc_ti_adc_read_raw(struct iio_dev *indio_dev,
			struct iio_chan_spec const *chan,
			int *val, int *val2, long m)
{
	int ret;
	int ch = chan->channel;
	struct gpadc_info *info = iio_priv(indio_dev);
	struct gpadc_result res;

	ret = iio_dc_ti_gpadc_sample(indio_dev, (1 << ch), &res);
	if (ret) {
		dev_err(info->dev, "sample failed\n");
		return ret;
	}

	*val = res.data[ch];

	return 0;
}

static int dc_ti_adc_read_all_raw(struct iio_channel *chan,
					int *val)
{
	int ret = 0;
	int i, num = 0;
	int ch = 0;
	int *channels;
	struct gpadc_info *info = iio_priv(chan->indio_dev);
	struct gpadc_result res;

	while (chan[num].indio_dev)
		num++;

	channels = kmalloc(sizeof(int) * num, GFP_KERNEL);
	if (channels == NULL)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		channels[i] = chan[i].channel->channel;
		ch  = (1 << channels[i]);
		ret = iio_dc_ti_gpadc_sample(chan->indio_dev, ch, &res);

		if (ret) {
			dev_err(info->dev, "sample failed\n");
			goto end;
		}
	}
	for (i = 0; i < num; i++)
		val[i] = res.data[channels[i]];

end:
	kfree(channels);
	return ret;
}

static const struct iio_info dc_ti_adc_info = {
	.read_raw = &dc_ti_adc_read_raw,
	.read_all_raw = &dc_ti_adc_read_all_raw,
	.driver_module = THIS_MODULE,
};

static int dc_ti_gpadc_probe(struct platform_device *pdev)
{
	int err;
	struct gpadc_info *info;
	struct iio_dev *indio_dev;

	indio_dev = iio_device_alloc(sizeof(struct gpadc_info));
	if (indio_dev == NULL) {
		dev_err(&pdev->dev, "allocating iio device failed\n");
		return -ENOMEM;
	}

	info = iio_priv(indio_dev);
	info->dev = &pdev->dev;
	info->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, indio_dev);
	mutex_init(&info->lock);
	init_waitqueue_head(&info->wait);


	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->name;
	indio_dev->channels = dc_ti_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(dc_ti_adc_channels);
	indio_dev->info = &dc_ti_adc_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	err = iio_map_array_register(indio_dev, iio_maps);
	if (err)
		goto err_free_device;

	err = iio_device_register(indio_dev);
	if (err < 0)
		goto err_array_unregister;

	err = request_threaded_irq(info->irq, NULL, dc_ti_gpadc_isr,
			IRQF_ONESHOT, DEV_NAME, info);
	if (err) {
		dev_err(&pdev->dev, "request_irq fail :%d\n", err);
		info->irq = -1;
	} else {
		/* Unmask VBUS interrupt */
		intel_mid_pmic_clearb(DC_ADC_IRQ_MASK_REG, IRQ_MASK_ADC);
	}

	dev_info(&pdev->dev, "dc_ti adc probed\n");

	return 0;

err_array_unregister:
	iio_map_array_unregister(indio_dev);
err_free_device:
	iio_device_free(indio_dev);
	return err;
}

static int dc_ti_gpadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct gpadc_info *info = iio_priv(indio_dev);

	if (info->irq >= 0)
		free_irq(info->irq, info);
	iio_device_unregister(indio_dev);
	iio_map_array_unregister(indio_dev);
	iio_device_free(indio_dev);

	return 0;
}

#ifdef CONFIG_PM
static int dc_ti_gpadc_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	mutex_lock(&info->lock);
	return 0;
}

static int dc_ti_gpadc_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct gpadc_info *info = iio_priv(indio_dev);

	mutex_unlock(&info->lock);
	return 0;
}
#else
#define dc_ti_gpadc_suspend		NULL
#define dc_ti_gpadc_resume		NULL
#endif

static const struct dev_pm_ops dc_ti_gpadc_pm_ops = {
	.suspend	= dc_ti_gpadc_suspend,
	.resume		= dc_ti_gpadc_resume,
};

static struct platform_driver dc_ti_gpadc_driver = {
	.probe = dc_ti_gpadc_probe,
	.remove = dc_ti_gpadc_remove,
	.driver = {
		.name = DEV_NAME,
		.pm = &dc_ti_gpadc_pm_ops,
	},
};

static int __init dc_pmic_adc_init(void)
{
	return platform_driver_register(&dc_ti_gpadc_driver);
}
module_init(dc_pmic_adc_init);

static void __exit dc_pmic_adc_exit(void)
{
	platform_driver_unregister(&dc_ti_gpadc_driver);
}
module_exit(dc_pmic_adc_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Dollar Cove(TI) GPADC Driver");
MODULE_LICENSE("GPL");
