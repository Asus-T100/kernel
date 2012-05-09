/*
 * lite-on ltr301 digital light sensor driver
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/input/ltr301als.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* ltr301 registers */
#define ALS_CONTR		0x80
#define ALS_MEAS_RATE		0x85
#define ALS_PART_ID		0x86
#define ALS_MANUFAC_ID		0x87
#define ALS_DATA_CH1_0		0x88
#define ALS_DATA_CH1_1		0x89
#define ALS_DATA_CH0_0		0x8a
#define ALS_DATA_CH0_1		0x8b
#define ALS_STATUS		0x8c
#define ALS_INTERRUPT		0x8f
#define ALS_THRES_UP_0		0x97
#define ALS_THRES_UP_1		0x98
#define ALS_THRES_LOW_0		0x99
#define ALS_THRES_LOW_1		0x9a
#define ALS_INT_PERSIST		0x9e

/* control register bits*/
#define ALS_CT_GAIN		BIT(3)
#define ALS_CT_SW_RESET		BIT(2)
#define ALS_CT_MODE_ACTIVE	BIT(1)
#define ALS_CT_MODE_STANDBY	0x00

/* measurement rate reg bits */
#define ALS_INTEG_TIME_100MS	0x00
#define ALS_INTEG_TIME_200MS	BIT(4)
#define ALS_INTEG_TIME_400MS	(BIT(4) | BIT(3))
#define ALS_RATE_100MS		BIT(0)

#define ALS_RATE_200MS		BIT(1)
#define ALS_RATE_500MS		(BIT(0) | BIT(1))
#define ALS_RATE_1000MS		BIT(2)

/* status register bits*/
#define ALS_ST_GAIN		BIT(4)
#define ALS_ST_INT		BIT(3)
#define ALS_ST_DATA		BIT(2)

/* interrupt control reg bits */
#define INT_POL			BIT(2)
#define INT_MODE_TRIGGER	BIT(1)
#define INT_MODE_INACTIVE	0x00

#define ALS_MAX_ADC		65535
#define ALS_WAKEUP_TIME	10	/* ms */

/* adc to lux calculation channel coefficients for different ranges */
#define CH0_COEFF_RANGE0	17743
#define CH1_COEFF_RANGE0	-11059
#define CH0_COEFF_RANGE1	37725
#define CH1_COEFF_RANGE1	13363
#define CH0_COEFF_RANGE2	16900
#define CH1_COEFF_RANGE2	1690
#define CH0_COEFF_RANGE3	16900
#define CH1_COEFF_RANGE3	-490

#define RATIO_RANGE0		45
#define RATIO_RANGE1		64
#define RATIO_RANGE2		85

#define IN_THRESHOLD_RANGE	1
#define LUX_TO_ADC_LOTHRESH	2
#define LUX_TO_ADC_HITHRESH	3
#define MAX_WINDOW_OPACITY	100

struct ltr301_chip {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
	struct mutex		lock;
	int			lux;
	u16			ch0;
	u16			hi_thresh;
	u16			lo_thresh;
	bool			in_use;
	unsigned int		opacity;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	es;
#endif
};

static int ltr301_read_word(struct ltr301_chip *chip, u8 reg)
{
	int lo, hi;

	lo = i2c_smbus_read_byte_data(chip->client, reg);
	if (lo < 0)
		return lo;
	hi = i2c_smbus_read_byte_data(chip->client, reg + 1);
	if (hi < 0)
		return hi;

	return hi << 8 | lo;
}

static int ltr301_write_word(struct ltr301_chip *chip, u8 reg, u16 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(chip->client, reg, val & 0xff);
	if (ret < 0)
		return ret;
	return i2c_smbus_write_byte_data(chip->client, reg + 1,
					 val >> 8 & 0xff);
}

static int ltr301_check_threshold(struct ltr301_chip *chip)
{
	if (chip->lo_thresh > chip->hi_thresh)
		return 0;

	if (chip->lux < chip->hi_thresh && chip->lux > chip->lo_thresh)
		return IN_THRESHOLD_RANGE;

	return 0;
}

/*
 * Lux is calulated based on two adc channels, ch0 and ch1.
 * Threshold values preventing interrupts are only compared against ch0.
 * To create thresholds in lux, first set a rough lux -> ch0 estimate,
 * then check the returned lux value and tune the ch0 threshold accordingly.
 * only call this function in case check_threshold returned IN_THRESHOLD_RANGE
 */
static int ltr301_tune_threshold(struct ltr301_chip *chip)
{
	int ret;
	int hi_thresh;

	hi_thresh = chip->ch0 + 1;
	if (hi_thresh > ALS_MAX_ADC)
		hi_thresh = ALS_MAX_ADC;

	ret = ltr301_write_word(chip, ALS_THRES_UP_0, hi_thresh);
	ret |= ltr301_write_word(chip, ALS_THRES_LOW_0, chip->ch0);

	return ret;
}

static int ltr301_get_lux(struct ltr301_chip *chip)
{
	int ch0, ch1;
	int lux;
	int ratio = 0;
	int ch0_coeff = 0;
	int ch1_coeff = 0;

	ch1 = ltr301_read_word(chip, ALS_DATA_CH1_0);
	if (ch1 < 0)
		return ch1;

	ch0 = ltr301_read_word(chip, ALS_DATA_CH0_0);
	if (ch0 < 0)
		return ch0;

	if (ch0 + ch1 > 0)
		ratio = ch1 * 100 / (ch0 + ch1);

	if (ratio < RATIO_RANGE0)	{
		ch0_coeff = CH0_COEFF_RANGE0;
		ch1_coeff = CH1_COEFF_RANGE0;
	} else if (ratio < RATIO_RANGE1) {
		ch0_coeff = CH0_COEFF_RANGE1;
		ch1_coeff = CH1_COEFF_RANGE1;
	} else if (ratio < RATIO_RANGE2) {
		ch0_coeff = CH0_COEFF_RANGE2;
		ch1_coeff = CH1_COEFF_RANGE2;
	} else {
		ch0_coeff = CH0_COEFF_RANGE3;
		ch1_coeff = CH1_COEFF_RANGE3;
	}

	chip->ch0 = ch0;
	/* ch_coeff are x10000, compensate for window loss while scaling down */
	lux = ((ch0 * ch0_coeff) - (ch1 * ch1_coeff)) / (100 * chip->opacity);

	dev_dbg(&chip->client->dev, "%s:ch0=%d, ch1=%d, lux=%d\n",
						__func__, ch0, ch1, lux);

	return lux;
}

static irqreturn_t ltr301_interrupt(int irq, void *data)
{
	struct ltr301_chip *chip = data;
	int lux;
	u8 status;

	status = i2c_smbus_read_byte_data(chip->client, ALS_STATUS);
	if (status < 0)
		goto out;

	if (status & ALS_ST_INT && status & ALS_ST_DATA) {
		mutex_lock(&chip->lock);
		lux = ltr301_get_lux(chip);
		if (lux < 0) {
			dev_err(&chip->client->dev, "Failed to read lux\n");
			mutex_unlock(&chip->lock);
			goto out;
		}
		chip->lux = lux;

		ltr301_tune_threshold(chip);
		input_report_abs(chip->input_dev, ABS_MISC, lux);
		input_sync(chip->input_dev);

		mutex_unlock(&chip->lock);
	}
out:
	return IRQ_HANDLED;
}

static ssize_t ltr301_thresh_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ltr301_chip *chip = dev_get_drvdata(dev);
	u16 thresh;

	if (!strcmp(attr->attr.name, "lo_thresh"))
		thresh = chip->lo_thresh;
	else if (!strcmp(attr->attr.name, "hi_thresh"))
		thresh = chip->hi_thresh;
	else
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%d\n", thresh);
}

/* store a rough ch0 adc estimate of the wanted lux threshold */
static ssize_t ltr301_thresh_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	struct ltr301_chip *chip = dev_get_drvdata(dev);
	u16 val = 0;
	u16 sensor_val;
	u8 thresh_reg;
	u16 *thresh;
	int lux_ch0_div;
	int ret;

	if (!strcmp(attr->attr.name, "lo_thresh")) {
		thresh_reg = ALS_THRES_LOW_0;
		thresh = &chip->lo_thresh;
		lux_ch0_div = LUX_TO_ADC_LOTHRESH;
	} else if (!strcmp(attr->attr.name, "hi_thresh")) {
		thresh_reg = ALS_THRES_UP_0;
		thresh = &chip->hi_thresh;
		lux_ch0_div = LUX_TO_ADC_HITHRESH;
	} else {
		return -EINVAL;
	}

	if (kstrtou16(buf, 10, &val) || val > ALS_MAX_ADC)
		return -EINVAL;

	sensor_val = val * chip->opacity / MAX_WINDOW_OPACITY;

	mutex_lock(&chip->lock);

	ret = ltr301_write_word(chip, thresh_reg, sensor_val / lux_ch0_div);
	if (ret < 0) {
		dev_err(dev, "Failed to set threshold\n");
		len = ret;
		goto out;
	}

	*thresh = val;
out:
	mutex_unlock(&chip->lock);
	return len;
}

static ssize_t ltr301_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ltr301_chip *chip = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", chip->in_use);
}

static ssize_t ltr301_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ltr301_chip *chip = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val != 0 && val != 1)
		return -EINVAL;

	if (val == chip->in_use)
		return count;

	mutex_lock(&chip->lock);
	if (val) {
		/* need to report a value each time device is enabled */
		ltr301_write_word(chip, ALS_THRES_UP_0, ALS_MAX_ADC);
		ltr301_write_word(chip, ALS_THRES_LOW_0, ALS_MAX_ADC);

		i2c_smbus_write_byte_data(chip->client,
					  ALS_CONTR, ALS_CT_MODE_ACTIVE);
		msleep(ALS_WAKEUP_TIME);
	} else {
		i2c_smbus_write_byte_data(chip->client,
					  ALS_CONTR, ALS_CT_MODE_STANDBY);
	}

	chip->in_use = val;
	mutex_unlock(&chip->lock);

	return count;
}

static DEVICE_ATTR(hi_thresh, S_IWUSR | S_IRUGO,
		ltr301_thresh_show, ltr301_thresh_store);
static DEVICE_ATTR(lo_thresh, S_IWUSR | S_IRUGO,
		ltr301_thresh_show, ltr301_thresh_store);

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		ltr301_enable_show, ltr301_enable_store);

static struct attribute *ltr301_attrs[] = {
	&dev_attr_hi_thresh.attr,
	&dev_attr_lo_thresh.attr,
	&dev_attr_enable.attr,
	NULL
};

static const struct attribute_group ltr301_attr_group = {
	.attrs = ltr301_attrs,
};

static int ltr301_initchip(struct ltr301_chip *chip)
{
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client, ALS_MEAS_RATE,
					ALS_INTEG_TIME_100MS | ALS_RATE_200MS);

	/* set thresholds to max values to always generate interrupts
	 * until userspace sets new threshold limits in sysfs */
	mutex_lock(&chip->lock);

	ret |= ltr301_write_word(chip, ALS_THRES_UP_0, ALS_MAX_ADC);
	ret |= ltr301_write_word(chip, ALS_THRES_LOW_0, ALS_MAX_ADC);
	chip->hi_thresh = ALS_MAX_ADC;
	chip->lo_thresh = ALS_MAX_ADC;

	mutex_unlock(&chip->lock);

	ret |= i2c_smbus_write_byte_data(chip->client, ALS_INTERRUPT,
					 INT_MODE_TRIGGER);

	ret |= i2c_smbus_write_byte_data(chip->client, ALS_INT_PERSIST,
					 0x02);

	ret |= i2c_smbus_write_byte_data(chip->client, ALS_CONTR,
					 ALS_CT_MODE_ACTIVE);

	msleep(ALS_WAKEUP_TIME);

	return ret;
}

static int ltr301_suspend(struct device *dev)
{
	return 0;
}

static int ltr301_resume(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ltr301_early_suspend(struct early_suspend *h)
{
	struct ltr301_chip *chip = container_of(h, struct ltr301_chip, es);

	mutex_lock(&chip->lock);
	i2c_smbus_write_byte_data(chip->client, ALS_CONTR, ALS_CT_MODE_STANDBY);
	mutex_unlock(&chip->lock);
}


static void ltr301_late_resume(struct early_suspend *h)
{
	struct ltr301_chip *chip = container_of(h, struct ltr301_chip, es);

	mutex_lock(&chip->lock);
	if (chip->in_use) {
		i2c_smbus_write_byte_data(chip->client, ALS_CONTR,
					  ALS_CT_MODE_ACTIVE);
		msleep(ALS_WAKEUP_TIME);
	}
	mutex_unlock(&chip->lock);
}
#endif

static const struct dev_pm_ops ltr301_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr301_suspend, ltr301_resume)
};

static int __devinit ltr301_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct ltr301als_platform_data *pdata = client->dev.platform_data;
	struct ltr301_chip *chip;
	struct input_dev *input_dev;
	int ret;
	int gpio;

	chip = kzalloc(sizeof *chip, GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		goto fail_1;
	}

	mutex_init(&chip->lock);

	if (pdata && pdata->window_opacity)
		chip->opacity = min(pdata->window_opacity, MAX_WINDOW_OPACITY);
	else
		chip->opacity = MAX_WINDOW_OPACITY;

	input_dev->name = "ltr301";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	chip->client = client;
	chip->input_dev = input_dev;

	input_set_drvdata(input_dev, chip);
	i2c_set_clientdata(client, chip);

	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev, ABS_MISC, 0, ALS_MAX_ADC, 0, 0);

	ret = input_register_device(input_dev);
	if (ret) {
		input_free_device(input_dev);
		goto fail_1;
	}

	gpio = client->irq;
	ret = gpio_request(gpio, "ltr301");
	if (ret < 0) {
		dev_err(&client->dev, "fail to request gpio %d\n", gpio);
		goto fail_2;
	}

	client->irq = gpio_to_irq(gpio);
	if (client->irq < 0) {
		dev_err(&client->dev, "fail to get irq for gpio %d\n", gpio);
		goto fail_3;
	}

	ret = request_threaded_irq(client->irq, NULL, ltr301_interrupt,
				   IRQF_TRIGGER_FALLING, "ltr301", chip);
	if (ret) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto fail_3;
	}

	ret = ltr301_initchip(chip);
	if (ret < 0) {
		dev_err(&client->dev, "ltr301_initchip failed\n");
		goto fail_4;
	}

	ret = sysfs_create_group(&client->dev.kobj, &ltr301_attr_group);
	if (ret)
		goto fail_4;

	i2c_smbus_write_byte_data(chip->client, ALS_CONTR,
					 ALS_CT_MODE_STANDBY);
	chip->in_use = false;

#ifdef CONFIG_HAS_EARLYSUSPEND
	chip->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10;
	chip->es.suspend = ltr301_early_suspend;
	chip->es.resume = ltr301_late_resume;
	register_early_suspend(&chip->es);
#endif
	return ret;
fail_4:
	free_irq(client->irq, chip);
fail_3:
	gpio_free(gpio);
fail_2:
	input_unregister_device(input_dev);
fail_1:
	kfree(chip);
	return ret;
}

static int __exit ltr301_remove(struct i2c_client *client)
{
	struct ltr301_chip *chip = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&chip->es);
#endif
	sysfs_remove_group(&client->dev.kobj, &ltr301_attr_group);
	free_irq(client->irq, chip);
	input_unregister_device(chip->input_dev);
	kfree(chip);
	return 0;
}

static const struct i2c_device_id ltr301_id[] = {
	{ "ltr301", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ltr301_id);

static struct i2c_driver ltr301_driver = {
	.driver = {
		.name   = "ltr301",
		.owner  = THIS_MODULE,
		.pm = &ltr301_pm,
	},
	.id_table = ltr301_id,
	.probe = ltr301_probe,
	.remove = __devexit_p(ltr301_remove),
};

static int __init ltr301_init(void)
{
	return i2c_add_driver(&ltr301_driver);
}

static void __exit ltr301_exit(void)
{
	i2c_del_driver(&ltr301_driver);
}

module_init(ltr301_init);
module_exit(ltr301_exit);

MODULE_DESCRIPTION("ltr301als lightsensor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathias Nyman <mathias.nyman@linux.intel.com>");
