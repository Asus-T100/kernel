/*
 * This file is part of the APDS9300 sensor driver.
 * Chip is combined proximity and ambient light sensor.
 *
 * Copyright (C) 2012 Intel Corporation
 * Contact: Jason Chen <Jason.cj.chen@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/i2c/apds9300.h>

/* Register map */
#define APDS9300_CNTR	 0x00 /* Enable of states and interrupts */
#define APDS9300_TIME	 0x01 /* ALS ADC time  */
#define APDS9300_AILTL	 0x02 /* ALS interrupt low threshold low byte */
#define APDS9300_AILTH	 0x03 /* ALS interrupt low threshold hi byte */
#define APDS9300_AIHTL	 0x04 /* ALS interrupt hi threshold low byte */
#define APDS9300_AIHTH	 0x05 /* ALS interrupt hi threshold hi byte */
#define APDS9300_INT	 0x06 /* Interrupt & persistence filters */
#define APDS9300_CRC	 0x08 /* Factory test */
#define APDS9300_ID	 0x0a /* Part number/Rev ID */
#define APDS9300_CDATAL	 0x0c /* Clear ADC low data register */
#define APDS9300_CDATAH	 0x0d /* Clear ADC high data register */
#define APDS9300_IRDATAL 0x0e /* IR ADC low data register */
#define APDS9300_IRDATAH 0x0f /* IR ADC high data register */

/* Control register */
#define APDS9300_CNTR_PON	(0x3 << 0)
#define APDS9300_CNTR_DISABLE	 0

/* Time register */
#define APDS9300_LOW_GAIN		(0x0 << 4)
#define APDS9300_HIGH_GAIN		(0x1 << 4)
#define APDS9300_INTEG_TIME_13MS	(0x0 << 0)
#define APDS9300_INTEG_TIME_101MS	(0x1 << 0)
#define APDS9300_INTEG_TIME_402MS	(0x2 << 0)

#define APDS9300_MAX_101MS	37177

/* Interrupt register */
#define APDS9300_LEVEL_INT	(0x1 << 4)
#define APDS9300_PERSIST_MASK	0xf

/* I2C access types */
#define APDS9300_CMD_TYPE_MASK	(0x1 << 5)
#define APDS9300_CMD_TYPE_BYTE	(0x0 << 5) /* byte */
#define APDS9300_CMD_TYPE_WORD	(0x1 << 5) /* word */

#define APDS9300_CMD		0x80

/* Interrupt ack commands */
#define APDS9300_INT_ACK	(0x1 << 6)

/* Supported ID:s */
#define APDS9300_ID_7		0x7

#define APDS_GPIO_CHECK_MAX	5

/* als_client.status bits */
#define ALS_DATA_READY	1
#define ALS_ENABLE	2

#define APDS_RATIO_P1		2130 /* 0.52 */
#define APDS_RATIO_P2		2662 /* 0.65 */
#define APDS_RATIO_P3		3277 /* 0.80 */
#define APDS_RATIO_P4		5325 /* 1.30 */
#define APDS_ALS_MAX_LUX	10000
#define APDS_ALS_MIN_ADC	1

struct apds9300_chip {
	bool			lux_wait_fresh_res;
	bool			suspend;
	int			als_cnt;
	int			gpio;
	struct mutex		mutex; /* avoid parallel access */
	struct list_head	als_list;
	wait_queue_head_t	als_wordq_head;
	struct early_suspend	es;
	struct miscdevice	dev;
	struct i2c_client		*client;
	struct apds9300_platform_data	*pdata;
	int irq_gpio;

	u16	rate;		/* als reporting rate */
	u16	max_result;	/* Max possible ADC value with current atime */
	u8	lux_persistence;

	u16	pre_clear;
	u16	pre_ir;

	u16	lux_clear;
	u16	lux_ir;
	u16	lux_calib;
	u16	lux_thres_hi;
	u16	lux_thres_lo;

	char	chipname[10];
	u8	revision;
};

struct als_client {
	unsigned long status;
	struct apds9300_chip *chip;
	struct list_head list;
};

#define APDS_CALIB_SCALER		8192
#define APDS_LUX_NEUTRAL_CALIB_VALUE	(1 * APDS_CALIB_SCALER)

#define APDS_LUX_DEF_THRES_HI		101
#define APDS_LUX_DEF_THRES_LO		100

#define APDS_RANGE			65535
#define APDS_LUX_GAIN_LO_LIMIT		100
#define APDS_LUX_GAIN_LO_LIMIT_STRICT	25

#define APDS_LUX_DEFAULT_RATE		5

/* Following two tables based on 101ms integration time */
static const u16 rates_hz[] = {10, 5, 2, 1};
static const u8 persis[] = {1, 2, 5, 10};

static int apds9300_read_byte(struct apds9300_chip *chip, u8 reg, u8 *data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS9300_CMD_TYPE_MASK;
	reg |= APDS9300_CMD | APDS9300_CMD_TYPE_BYTE;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C read 0x%x byte error!",
				reg & ~APDS9300_CMD_TYPE_MASK & ~APDS9300_CMD);
	*data = ret;
	return (int)ret;
}

static int apds9300_read_word(struct apds9300_chip *chip, u8 reg, u16 *data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS9300_CMD_TYPE_MASK;
	reg |= APDS9300_CMD | APDS9300_CMD_TYPE_WORD;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C read 0x%x word error!",
				reg & ~APDS9300_CMD_TYPE_MASK & ~APDS9300_CMD);
	*data = ret;
	return (int)ret;
}

static int apds9300_write_byte(struct apds9300_chip *chip, u8 reg, u8 data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS9300_CMD_TYPE_MASK;
	reg |= APDS9300_CMD | APDS9300_CMD_TYPE_BYTE;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C write 0x%x byte error!",
				reg & ~APDS9300_CMD_TYPE_MASK & ~APDS9300_CMD);
	return (int)ret;
}

static int apds9300_write_word(struct apds9300_chip *chip, u8 reg, u16 data)
{
	struct i2c_client *client = chip->client;
	s32 ret;

	reg &= ~APDS9300_CMD_TYPE_MASK;
	reg |= APDS9300_CMD | APDS9300_CMD_TYPE_WORD;

	ret = i2c_smbus_write_word_data(client, reg, data);
	if (ret < 0)
		dev_err(&chip->client->dev, "I2C write 0x%x word error!",
				reg & ~APDS9300_CMD_TYPE_MASK & ~APDS9300_CMD);
	return (int)ret;
}

static int apds9300_ack_int(struct apds9300_chip *chip)
{
	struct i2c_client *client = chip->client;
	s32 ret;
	u8 reg = APDS9300_CMD | APDS9300_INT_ACK;

	ret = i2c_smbus_read_byte_data(client, reg);
	return (int)ret;
}

static void apds9300_poweroff(struct apds9300_chip *chip)
{
	apds9300_write_byte(chip, APDS9300_CNTR, APDS9300_CNTR_DISABLE);
}

static void apds9300_poweron(struct apds9300_chip *chip)
{
	apds9300_write_byte(chip, APDS9300_CNTR, APDS9300_CNTR_PON);
}

/* Called always with mutex locked */
static void apds9300_clear_to_athres(struct apds9300_chip *chip)
{
	u16 lo, hi;

	/* The data register may float in very bright environment which causes
	 * a lot of meaningless interrupt. To avoid that and reduce power
	 * consumption, set interrupt trigger condition as 2% change of current
	 * ADC value
	 */
	if (chip->lux_clear < APDS_ALS_MIN_ADC) {
		lo = 0;
		hi = APDS_ALS_MIN_ADC;
	} else {
		lo = chip->lux_clear * 98 / 100;
		if (lo >= chip->max_result)
			lo = chip->max_result * 98 / 100;

		hi = chip->lux_clear * 102 / 100;
		if (hi == chip->lux_clear)
			hi += 1;
		else if (hi >= chip->max_result)
			hi = chip->max_result - 1;
	}

	chip->lux_thres_hi = hi;
	chip->lux_thres_lo = lo;
}

/* Called always with mutex locked */
static int apds9300_refresh_athres(struct apds9300_chip *chip)
{
	int ret;

	ret = apds9300_write_word(chip, APDS9300_AILTL, chip->lux_thres_lo);
	ret |= apds9300_write_word(chip, APDS9300_AIHTL, chip->lux_thres_hi);

	dev_dbg(&chip->client->dev, "als threshold: %d, %d",
			chip->lux_thres_lo, chip->lux_thres_hi);

	return ret;
}

/* Called always with mutex locked */
static void apds9300_force_a_refresh(struct apds9300_chip *chip)
{
	/* This will force ALS interrupt after the next measurement. */
	apds9300_write_word(chip, APDS9300_AILTL, APDS_LUX_DEF_THRES_LO);
	apds9300_write_word(chip, APDS9300_AIHTL, APDS_LUX_DEF_THRES_HI);
}

static ssize_t apds9300_clear_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apds9300_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&chip->mutex);
	ret = sprintf(buf, "%d\n", chip->lux_clear);
	mutex_unlock(&chip->mutex);
	return ret;
}

static DEVICE_ATTR(lux_clear, S_IRUGO, apds9300_clear_show, NULL);

static ssize_t apds9300_ir_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apds9300_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&chip->mutex);
	ret = sprintf(buf, "%d\n", chip->lux_ir);
	mutex_unlock(&chip->mutex);
	return ret;
}

static DEVICE_ATTR(lux_ir, S_IRUGO, apds9300_ir_show, NULL);

static ssize_t apds9300_lux_calib_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct apds9300_chip *chip = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", chip->lux_calib);
}

static ssize_t apds9300_lux_calib_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds9300_chip *chip = dev_get_drvdata(dev);
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	if (chip->lux_calib > APDS_RANGE)
		return -EINVAL;

	chip->lux_calib = value;

	return len;
}

static DEVICE_ATTR(lux0_calibscale, S_IRUGO | S_IWUSR, apds9300_lux_calib_show,
		apds9300_lux_calib_store);

static ssize_t apds9300_rate_avail(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int i;
	int pos = 0;
	for (i = 0; i < ARRAY_SIZE(rates_hz); i++)
		pos += sprintf(buf + pos, "%d ", rates_hz[i]);
	sprintf(buf + pos - 1, "\n");
	return pos;
}

static ssize_t apds9300_rate_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds9300_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->rate);
}

static int apds9300_set_rate(struct apds9300_chip *chip, int rate)
{
	int i, ret;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(rates_hz); i++)
		if (rate >= rates_hz[i])
			break;

	if (i == ARRAY_SIZE(rates_hz))
		return -EINVAL;

	/* Pick up corresponding persistence value */
	chip->lux_persistence = persis[i];
	chip->rate = rates_hz[i];

	ret = apds9300_read_byte(chip, APDS9300_INT, &reg);
	if (ret < 0) {
		dev_err(&chip->client->dev, "INT reg read failed\n");
		return ret;
	} else {
		reg &= ~APDS9300_PERSIST_MASK;
		reg |= chip->lux_persistence;
		/* Persistence levels */
		return apds9300_write_byte(chip, APDS9300_INT, reg);
	}
}

static ssize_t apds9300_rate_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds9300_chip *chip =  dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;

	mutex_lock(&chip->mutex);
	if (chip->suspend) {
		mutex_unlock(&chip->mutex);
		return -EINVAL;
	}
	ret = apds9300_set_rate(chip, value);
	mutex_unlock(&chip->mutex);

	if (ret < 0)
		return ret;
	return len;
}

static DEVICE_ATTR(lux0_rate_avail, S_IRUGO, apds9300_rate_avail, NULL);
static DEVICE_ATTR(lux0_rate, S_IRUGO | S_IWUSR, apds9300_rate_show,
						 apds9300_rate_store);

static ssize_t apds9300_lux_thresh_above_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds9300_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->lux_thres_hi);
}

static ssize_t apds9300_lux_thresh_below_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds9300_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->lux_thres_lo);
}

static ssize_t apds9300_set_lux_thresh(struct apds9300_chip *chip, u16 *target,
				const char *buf)
{
	int ret = 0;
	unsigned long thresh;

	if (kstrtoul(buf, 0, &thresh))
		return -EINVAL;

	if (thresh > APDS_RANGE)
		return -EINVAL;

	mutex_lock(&chip->mutex);
	if (chip->suspend) {
		mutex_unlock(&chip->mutex);
		return -EINVAL;
	}
	*target = (u16)thresh;
	/*
	 * Don't update values in HW if we are still waiting for
	 * first interrupt to come after device handle open call.
	 */
	if (!chip->lux_wait_fresh_res)
		apds9300_refresh_athres(chip);
	mutex_unlock(&chip->mutex);
	return ret;

}

static ssize_t apds9300_lux_thresh_above_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds9300_chip *chip =  dev_get_drvdata(dev);
	int ret = apds9300_set_lux_thresh(chip, &chip->lux_thres_hi, buf);
	if (ret < 0)
		return ret;
	return len;
}

static ssize_t apds9300_lux_thresh_below_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	struct apds9300_chip *chip =  dev_get_drvdata(dev);
	int ret = apds9300_set_lux_thresh(chip, &chip->lux_thres_lo, buf);
	if (ret < 0)
		return ret;
	return len;
}

static DEVICE_ATTR(lux0_thresh_above_value, S_IRUGO | S_IWUSR,
		apds9300_lux_thresh_above_show,
		apds9300_lux_thresh_above_store);

static DEVICE_ATTR(lux0_thresh_below_value, S_IRUGO | S_IWUSR,
		apds9300_lux_thresh_below_show,
		apds9300_lux_thresh_below_store);

static ssize_t apds9300_chip_id_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds9300_chip *chip =  dev_get_drvdata(dev);
	return sprintf(buf, "%s %d\n", chip->chipname, chip->revision);
}

static DEVICE_ATTR(chip_id, S_IRUGO, apds9300_chip_id_show, NULL);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_lux0_calibscale.attr,
	&dev_attr_lux_clear.attr,
	&dev_attr_lux_ir.attr,
	&dev_attr_lux0_rate.attr,
	&dev_attr_lux0_rate_avail.attr,
	&dev_attr_lux0_thresh_above_value.attr,
	&dev_attr_lux0_thresh_below_value.attr,
	&dev_attr_chip_id.attr,
	NULL
};

static struct attribute_group apds9300_attribute_group[] = {
	{.attrs = sysfs_attrs_ctrl },
};

static ssize_t
als_read(struct file *filep, char __user *buffer, size_t size, loff_t *offset)
{
	u32 buf[2];
	int ret = -ENODEV;
	struct als_client *client = filep->private_data;
	struct apds9300_chip *chip = client->chip;

	mutex_lock(&chip->mutex);
	if (test_bit(ALS_ENABLE, &client->status)) {
		buf[0] = chip->lux_clear;
		buf[1] = chip->lux_ir;
		clear_bit(ALS_DATA_READY, &client->status);
		if (copy_to_user(buffer, buf, sizeof(u32) * 2))
			ret = -EFAULT;
		ret = sizeof(u32) * 2;
	}
	mutex_unlock(&chip->mutex);

	return ret;
}

static unsigned int
als_poll(struct file *filep, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct als_client *client = filep->private_data;
	struct apds9300_chip *chip = client->chip;

	poll_wait(filep, &chip->als_wordq_head, wait);

	if (test_bit(ALS_DATA_READY, &client->status))
		mask |= (POLLIN | POLLRDNORM);

	return mask;
}

static int als_open(struct inode *inode, struct file *filep)
{
	struct apds9300_chip *chip =
		container_of(filep->private_data,
				struct apds9300_chip, dev);
	struct als_client *client;

	client = kzalloc(sizeof(struct als_client), GFP_KERNEL);
	if (client == NULL) {
		dev_dbg(&chip->client->dev, "ALS open kzalloc failed!\n");
		return -ENOMEM;
	}
	client->chip = chip;

	filep->private_data = client;
	mutex_lock(&chip->mutex);
	list_add(&client->list, &chip->als_list);
	mutex_unlock(&chip->mutex);

	return 0;
}

static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct als_client *client = file->private_data;
	struct apds9300_chip *chip = client->chip;

	dev_dbg(&chip->client->dev,
		"cmd = %d, arg = %d\n", (int)cmd, (int)arg);

	mutex_lock(&chip->mutex);
	if (chip->suspend) {
		mutex_unlock(&chip->mutex);
		return -EINVAL;
	}
	/* 1 - enable; 0 - disable */
	switch (arg) {
	case 0:
		if (!test_and_clear_bit(ALS_ENABLE, &client->status)) {
			dev_warn(&chip->client->dev,
				"ALS is not enabled for this client\n");
		} else if (--chip->als_cnt <= 0) {
			chip->als_cnt = 0;
			apds9300_poweroff(chip);
		}
		break;
	case 1:
		if (test_and_set_bit(ALS_ENABLE, &client->status)) {
			dev_warn(&chip->client->dev,
				"ALS is already enabled for this client\n");
		} else if (0 == chip->als_cnt++) {
			apds9300_poweron(chip);
		}
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&chip->mutex);
	return ret;
}

static int als_close(struct inode *inode, struct file *filep)
{
	struct als_client *client = filep->private_data;
	struct apds9300_chip *chip = client->chip;

	mutex_lock(&chip->mutex);
	list_del(&client->list);

	if (test_bit(ALS_ENABLE, &client->status)) {
		if (--chip->als_cnt <= 0) {
			chip->als_cnt = 0;
			if (!chip->suspend)
				apds9300_poweroff(chip);
		}
	}
	mutex_unlock(&chip->mutex);
	kfree(client);
	filep->private_data = NULL;

	return 0;
}

static const struct file_operations als_fops = {
	.owner = THIS_MODULE,
	.open = als_open,
	.read = als_read,
	.poll = als_poll,
	.release = als_close,
	.unlocked_ioctl = als_ioctl,
	.llseek = no_llseek,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void apds9300_early_suspend(struct early_suspend *h)
{
	struct apds9300_chip *chip = container_of(h, struct apds9300_chip, es);

	dev_dbg(&chip->client->dev, "enter %s\n", __func__);

	disable_irq(chip->client->irq);
	mutex_lock(&chip->mutex);
	if (chip->als_cnt)
		apds9300_poweroff(chip);
	chip->suspend = true;
	mutex_unlock(&chip->mutex);
}

static void apds9300_late_resume(struct early_suspend *h)
{
	struct apds9300_chip *chip = container_of(h, struct apds9300_chip, es);

	dev_dbg(&chip->client->dev, "enter %s\n", __func__);

	mutex_lock(&chip->mutex);
	if (chip->als_cnt)
		apds9300_poweron(chip);
	chip->suspend = false;
	mutex_unlock(&chip->mutex);
	enable_irq(chip->client->irq);
}
#define apds9300_suspend  NULL
#define apds9300_resume   NULL
#else
#ifdef CONFIG_PM
static int apds9300_suspend(struct device *dev)
{
	struct i2c_client *i2c_client = to_i2c_client(dev);
	struct apds9300_chip *chip = i2c_get_clientdata(i2c_client);

	disable_irq(i2c_client->irq);
	if (!mutex_trylock(&chip->mutex)) {
		enable_irq(i2c_client->irq);
		return -EBUSY;
	}
	if (chip->als_cnt)
		apds9300_poweroff(chip);
	mutex_unlock(&chip->mutex);
	return 0;
}

static int apds9300_resume(struct device *dev)
{
	struct i2c_client *i2c_client = to_i2c_client(dev);
	struct apds9300_chip *chip = i2c_get_clientdata(i2c_client);

	mutex_lock(&chip->mutex);
	if (chip->als_cnt)
		apds9300_poweron(chip);
	mutex_unlock(&chip->mutex);
	enable_irq(i2c_client->irq);
	return 0;
}
#else
#define apds9300_suspend  NULL
#define apds9300_resume   NULL
#endif
#endif
static const struct dev_pm_ops apds9300_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(apds9300_suspend, apds9300_resume)
};

/* mutex must be held when calling this function */
static void als_handle_irq(struct apds9300_chip *chip)
{
	struct als_client *client;

	apds9300_read_word(chip, APDS9300_CDATAL, &chip->lux_clear);
	apds9300_read_word(chip, APDS9300_IRDATAL, &chip->lux_ir);

	dev_dbg(&chip->client->dev, "clear=%d,ir=%d", chip->lux_clear,
		chip->lux_ir);
	apds9300_clear_to_athres(chip);
	apds9300_refresh_athres(chip);

	if (chip->pre_clear != chip->lux_clear ||
			chip->pre_ir != chip->lux_ir ||
			chip->lux_wait_fresh_res == true) {
		chip->lux_wait_fresh_res = false;
		chip->pre_clear = chip->lux_clear;
		chip->pre_ir = chip->lux_ir;

		list_for_each_entry(client, &chip->als_list, list)
			set_bit(ALS_DATA_READY, &client->status);

		wake_up(&chip->als_wordq_head);
	}
}

static irqreturn_t apds9300_irq(int irq, void *data)
{
	int i, value;
	struct apds9300_chip *chip = data;

	mutex_lock(&chip->mutex);
	for (i = 0; i < APDS_GPIO_CHECK_MAX; i++) {
		apds9300_ack_int(chip);

		als_handle_irq(chip);

		/* Since apds9300's interrupt pin is level type and some GPIO
		 * controllers don't support level trigger, we need to check
		 * gpio pin value to see if there is another interupt occurs
		 * in the time window that interrupt status register read and
		 * interrupt ack. If that happens, do irq handle again to
		 * avert interrupt missing.
		 */
		value = gpio_get_value(chip->irq_gpio);
		dev_dbg(&chip->client->dev,
					"%s: try=%d, GPIO value = 0x%x",
					__func__, i, value);
		if (value)
			break;
	}
	if (i == APDS_GPIO_CHECK_MAX) {
		dev_warn(&chip->client->dev,
				"GPIO check max, reset the sensor\n");
		apds9300_poweroff(chip);
		apds9300_poweron(chip);
	}
	mutex_unlock(&chip->mutex);

	return IRQ_HANDLED;
}

static int apds9300_setup_irq(struct apds9300_chip *chip)
{
	int ret;
	int gpio = chip->irq_gpio;
	struct i2c_client *client = chip->client;

	dev_dbg(&client->dev, "apds9300 setup irq from gpio %d.", gpio);
	ret = gpio_request(gpio, "apds9300");
	if (ret < 0) {
		dev_err(&client->dev, "Request gpio %d failed!\n", gpio);
		goto out;
	}
	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to configure input\n");
		goto fail_gpio;
	}
	ret = gpio_to_irq(gpio);
	if (ret < 0) {
		dev_err(&client->dev, "Configure gpio to irq failed!\n");
		goto fail_gpio;
	}
	client->irq = ret;
	dev_dbg(&client->dev, "irq = %d.", client->irq);

	ret = request_threaded_irq(client->irq, NULL,
				apds9300_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"apds9300", chip);
	if (ret < 0) {
		dev_err(&client->dev, "Can't allocate irq %d\n", client->irq);
		goto fail_gpio;
	}

	return 0;

fail_gpio:
	gpio_free(gpio);
out:
	return ret;
}

static struct apds9300_chip *apds9300_alloc_dev(void)
{
	struct apds9300_chip *chip;

	chip = kzalloc(sizeof(struct apds9300_chip), GFP_KERNEL);
	if (!chip)
		return NULL;

	mutex_init(&chip->mutex);
	init_waitqueue_head(&chip->als_wordq_head);

	INIT_LIST_HEAD(&chip->als_list);

	return chip;
}

static int apds9300_detect(struct apds9300_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret;
	u8 id;

	ret = apds9300_read_byte(chip, APDS9300_ID, &id);
	if (ret < 0) {
		dev_err(&client->dev, "ID read failed\n");
		return ret;
	}

	/* Part number */
	dev_info(&client->dev, "Part number is %d revision number is %d\n",
				(id & 0xf0) >> 4, id & 0xf);

	switch ((id & 0xf0) >> 4) {
	case APDS9300_ID_7:
		snprintf(chip->chipname, sizeof(chip->chipname), "APDS-9300");
		break;
	default:
		dev_err(&client->dev, "invalide part number");
		ret = -ENODEV;
		break;
	}
	return ret;
}

static void apds9300_init_params(struct apds9300_chip *chip)
{
	/* Set something to start with */
	chip->lux_thres_hi = APDS_LUX_DEF_THRES_HI;
	chip->lux_thres_lo = APDS_LUX_DEF_THRES_LO;
	chip->lux_calib = APDS_LUX_NEUTRAL_CALIB_VALUE;
	chip->lux_wait_fresh_res = true;
}

static int __devinit apds9300_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct apds9300_chip *chip;
	int err;

	dev_dbg(&client->dev, "apds9300 driver probe.");
	chip = apds9300_alloc_dev();
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;
	i2c_set_clientdata(client, chip);

	if (chip->pdata == NULL) {
		dev_err(&chip->client->dev,
				"apds9300 platform data is mandatory\n");
		err = -EINVAL;
		goto fail1;
	}

	if (chip->pdata->setup_resources) {
		err = chip->pdata->setup_resources();
		if (err) {
			dev_err(&chip->client->dev,
					"pdata setup_resources error\n");
			err = -EINVAL;
			goto fail1;
		}
	}
	chip->irq_gpio = chip->pdata->gpio_number;
	dev_dbg(&client->dev, "apds9300 irq_gpio = %d\n", chip->irq_gpio);

	err = apds9300_detect(chip);
	if (err < 0) {
		dev_err(&client->dev, "APDS9300 not found\n");
		goto fail2;
	}

	apds9300_init_params(chip);

	/* It is recommended to use disabled mode during these operations */
	apds9300_poweroff(chip);

	/* Gain:high, integrate time:101ms */
	apds9300_write_byte(chip, APDS9300_TIME,
			APDS9300_HIGH_GAIN | APDS9300_INTEG_TIME_101MS);
	/* Enable level interrupt */
	apds9300_write_byte(chip, APDS9300_INT, APDS9300_LEVEL_INT);
	chip->max_result = APDS9300_MAX_101MS;

	apds9300_set_rate(chip, APDS_LUX_DEFAULT_RATE);

	err = sysfs_create_group(&chip->client->dev.kobj,
				apds9300_attribute_group);
	if (err < 0) {
		dev_err(&chip->client->dev, "Sysfs registration failed\n");
		goto fail2;
	}

	chip->dev.minor = MISC_DYNAMIC_MINOR;
	chip->dev.name = "apds9300_lsensor";
	chip->dev.fops = &als_fops;

	err = misc_register(&chip->dev);
	if (err) {
		dev_err(&client->dev, "ambient miscdev register failed\n");
		goto fail3;
	}

	err = apds9300_setup_irq(chip);
	if (err) {
		dev_err(&client->dev, "Setup IRQ error\n");
		goto fail4;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	chip->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10;
	chip->es.suspend = apds9300_early_suspend;
	chip->es.resume = apds9300_late_resume;
	register_early_suspend(&chip->es);
#endif
	apds9300_force_a_refresh(chip);

	dev_info(&client->dev, "probe done");
	return err;
fail4:
	misc_deregister(&chip->dev);
fail3:
	sysfs_remove_group(&chip->client->dev.kobj,
			&apds9300_attribute_group[0]);
fail2:
	if (chip->pdata->release_resources)
		chip->pdata->release_resources();
fail1:
	kfree(chip);
	return err;
}

static int __devexit apds9300_remove(struct i2c_client *client)
{
	struct apds9300_chip *chip = i2c_get_clientdata(client);

	disable_irq_wake(client->irq);
	free_irq(client->irq, chip);
	sysfs_remove_group(&chip->client->dev.kobj,
			apds9300_attribute_group);
	misc_deregister(&chip->dev);

	if (chip->pdata && chip->pdata->release_resources)
		chip->pdata->release_resources();

	apds9300_poweroff(chip);
	unregister_early_suspend(&chip->es);

	kfree(chip);
	return 0;
}

static const struct i2c_device_id apds9300_id[] = {
	{"apds9300", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, apds9300_id);

static struct i2c_driver apds9300_driver = {
	.driver	 = {
		.name	= "apds9300",
		.owner	= THIS_MODULE,
		.pm	= &apds9300_pm_ops,
	},
	.probe	  = apds9300_probe,
	.remove	  = __devexit_p(apds9300_remove),
	.id_table = apds9300_id,
};

static int __init apds9300_init(void)
{
	return i2c_add_driver(&apds9300_driver);
}

static void __exit apds9300_exit(void)
{
	i2c_del_driver(&apds9300_driver);
}

MODULE_DESCRIPTION("APDS9300 ALS sensor");
MODULE_AUTHOR("Jason Chen, Intel Corporation");
MODULE_LICENSE("GPL v2");

module_init(apds9300_init);
module_exit(apds9300_exit);
