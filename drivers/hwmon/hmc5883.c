/*
 * Copyright (C) 2009 AMIT Technology Inc.
 * Author: Kyle Chen <sw-support@amit-inc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

/* HMC5883 Internal Register Address (refer to HMC5883 Specifications) */
#define HMC5883_REG_A			0
  #define SAMPLE_AVERAGE_8		(0x3 << 5)
  #define OUTPUT_RATE_75		(0x6 << 2)
  #define MEASURE_NORMAL		0
  #define MEASURE_SELFTEST		0x1
#define HMC5883_REG_B			1
  #define GAIN_DEFAULT			(3 << 5)
#define HMC5883_REG_MODE		2
  #define MODE_CONT			0x0
  #define MODE_SINGLE			0x1
  #define MODE_IDLE			0x2
#define HMC5883_REG_DATAX_MSB		3
#define HMC5883_REG_DATAX_LSB		4
#define HMC5883_REG_DATAZ_MSB		5
#define HMC5883_REG_DATAZ_LSB		6
#define HMC5883_REG_DATAY_MSB		7
#define HMC5883_REG_DATAY_LSB		8
#define HMC5883_STATUS_REG		9
#define HMC5883_REG_ID1			10
#define HMC5883_REG_ID2			11
#define HMC5883_REG_ID3			12

struct hmc5883_data {
	struct mutex lock;
	struct i2c_client *client;
	struct delayed_work work;
	struct input_dev *input;
	int delay_ms;

	int enabled;
};

static void hmc5883_put_idle(struct hmc5883_data *hmc5883)
{
	i2c_smbus_write_byte_data(hmc5883->client, HMC5883_REG_A,
			SAMPLE_AVERAGE_8 | OUTPUT_RATE_75 | MEASURE_NORMAL);
	i2c_smbus_write_byte_data(hmc5883->client, HMC5883_REG_B, GAIN_DEFAULT);
	i2c_smbus_write_byte_data(hmc5883->client, HMC5883_REG_MODE, MODE_IDLE);
}

static void hmc5883_start_measure(struct hmc5883_data *hmc5883)
{
	i2c_smbus_write_byte_data(hmc5883->client, HMC5883_REG_MODE, MODE_CONT);
}

static void hmc5883_stop_measure(struct hmc5883_data *hmc5883)
{
	i2c_smbus_write_byte_data(hmc5883->client, HMC5883_REG_MODE, MODE_IDLE);
}

static int hmc5883_enable(struct hmc5883_data *hmc5883)
{
	dev_dbg(&hmc5883->client->dev, "start measure!\n");

	hmc5883_start_measure(hmc5883);
	schedule_delayed_work(&hmc5883->work,
			msecs_to_jiffies(hmc5883->delay_ms));

	return 0;
}

static int hmc5883_disable(struct hmc5883_data *hmc5883)
{
	dev_dbg(&hmc5883->client->dev, "stop measure!\n");

	hmc5883_stop_measure(hmc5883);
	cancel_delayed_work(&hmc5883->work);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hmc5883_suspend(struct device *dev)
{
	struct hmc5883_data *hmc5883 = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&hmc5883->work);

	mutex_lock(&hmc5883->lock);
	hmc5883_stop_measure(hmc5883);
	mutex_unlock(&hmc5883->lock);
	return 0;
}

static int hmc5883_resume(struct device *dev)
{
	struct hmc5883_data *hmc5883 = dev_get_drvdata(dev);

	mutex_lock(&hmc5883->lock);

	hmc5883_put_idle(hmc5883);
	if (hmc5883->enabled)
		hmc5883_enable(hmc5883);

	mutex_unlock(&hmc5883->lock);
	return 0;
}

static const struct dev_pm_ops hmc5883_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hmc5883_suspend,
				hmc5883_resume)
};
#endif

static int hmc5883_get_data_xyz(struct hmc5883_data *hmc5883, s16 data[])
{
	int ret;
	int i;
	s16 values[3];

	ret = i2c_smbus_read_i2c_block_data(hmc5883->client,
				HMC5883_REG_DATAX_MSB, 6, (u8 *)values);
	if (ret < 0) {
		dev_err(&hmc5883->client->dev, "error read data %d\n", ret);
		return ret;
	}

	for (i = 0; i < 3; i++)
		values[i] = be16_to_cpu(values[i]);

	dev_dbg(&hmc5883->client->dev, "x=%d y=%d z=%d\n",
		values[0], values[2], values[1]);

	data[0] = values[0];
	data[1] = values[2];
	data[2] = values[1];

	return 0;
}

/* Definations for HMC5883L Self-Test */
static int magval_st[3];
#define DEFAULT_GAIN		(660) /* LSB/Gauss */
#define SELFTEST_GAIN		DEFAULT_GAIN /* LSB/Gauss */
/* XY_MIN = GAIN * 1.16(bias gause) * 80%(-20% off) */
#define SELFTEST_XY_MIN	(SELFTEST_GAIN * 116 / 100 * 80 / 100)
/* XY_MAX = GAIN * 1.16(bias gause) * 120%(+20% off) */
#define SELFTEST_XY_MAX	(SELFTEST_GAIN * 116 / 100 * 120 / 100)
#define SELFTEST_XY_VALID(xy) (xy > SELFTEST_XY_MIN && xy < SELFTEST_XY_MAX)

/* Z_MIN = GAIN * 1.08(bias gause) * 80%(-20% off) */
#define SELFTEST_Z_MIN (SELFTEST_GAIN * 108 / 100 * 80 / 100)
/* Z_MAX = GAIN * 1.08(bias gause) * 120%(+20% off) */
#define SELFTEST_Z_MAX (SELFTEST_GAIN * 108 / 100 * 120 / 100)
#define SELFTEST_Z_VALID(z) (z > SELFTEST_Z_MIN && z < SELFTEST_Z_MAX)

#define SELFTEST_X_ADJUST(vx) (magval_st[0] ? \
	(vx * SELFTEST_GAIN * 116 / 100 / magval_st[0]) : vx)
#define SELFTEST_Y_ADJUST(vy) (magval_st[1] ? \
	(vy * SELFTEST_GAIN * 116 / 100 / magval_st[1]) : vy)
#define SELFTEST_Z_ADJUST(vz) (magval_st[2] ? \
	(vz * SELFTEST_GAIN * 108 / 100 / magval_st[2]) : vz)

static void hmc5883_selftest(struct hmc5883_data *hmc5883)
{
	int ret = 0, loop = 4;
	struct device *dev = &hmc5883->client->dev;
	s16 data[3];

	memset(magval_st, 0, sizeof(magval_st));
	i2c_smbus_write_byte_data(hmc5883->client, HMC5883_REG_A,
		SAMPLE_AVERAGE_8 | OUTPUT_RATE_75 | MEASURE_SELFTEST);

	while (--loop) {
		i2c_smbus_write_byte_data(hmc5883->client,
					  HMC5883_REG_MODE, MODE_SINGLE);
		msleep(20);
		ret = hmc5883_get_data_xyz(hmc5883, data);
		if (ret < 0)
			continue;

		dev_dbg(dev, "%s: selftest get data(%d, %d, %d)\n",
			 __func__, data[0], data[1], data[2]);
		if (SELFTEST_XY_VALID(data[0]) &&
		    SELFTEST_XY_VALID(data[1]) &&
		    SELFTEST_Z_VALID(data[2])) {
			magval_st[0] = data[0];
			magval_st[1] = data[1];
			magval_st[2] = data[2];
			break;
		}
	}
	if (!loop) {
		dev_warn(dev, "%s:  selftest failed\n", __func__);
	} else {
		dev_info(dev, "%s: selftest result(%d, %d, %d)\n",
			 __func__, magval_st[0], magval_st[1], magval_st[2]);
	}

	hmc5883_put_idle(hmc5883);
}

static void hmc5883_work(struct work_struct *work)
{
	int ret;
	struct hmc5883_data *hmc5883 = container_of((struct delayed_work *)work,
						struct hmc5883_data, work);
	s16 data[3];

	mutex_lock(&hmc5883->lock);

	if (!hmc5883->enabled)
		goto out;

	ret = hmc5883_get_data_xyz(hmc5883, data);
	if (ret < 0) {
		dev_err(&hmc5883->client->dev, "error read data\n");
	} else {
		data[0] = SELFTEST_X_ADJUST(data[0]);
		data[1] = SELFTEST_Y_ADJUST(data[1]);
		data[2] = SELFTEST_Z_ADJUST(data[2]);

		input_report_rel(hmc5883->input, REL_X, data[0]);
		input_report_rel(hmc5883->input, REL_Y, data[1]);
		input_report_rel(hmc5883->input, REL_Z, data[2]);
		input_sync(hmc5883->input);
	}

	schedule_delayed_work(&hmc5883->work,
			      msecs_to_jiffies(hmc5883->delay_ms));
out:
	mutex_unlock(&hmc5883->lock);
}

static int __devinit hmc5883_input_init(struct hmc5883_data *hmc5883)
{
	struct input_dev *dev;
	int ret;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "hmc5883";
	dev->id.bustype = BUS_I2C;

	set_bit(EV_REL, dev->evbit);
	set_bit(REL_X, dev->relbit);
	set_bit(REL_Y, dev->relbit);
	set_bit(REL_Z, dev->relbit);
	input_set_drvdata(dev, hmc5883);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	hmc5883->input= dev;
	return 0;
}

static ssize_t
attr_get_poll(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hmc5883_data *hmc5883 = dev_get_drvdata(dev);
	int pollms;

	mutex_lock(&hmc5883->lock);
	pollms = hmc5883->delay_ms;
	mutex_unlock(&hmc5883->lock);

	return sprintf(buf, "%d\n", pollms);
}

#define HMC5883_MAX_RATE 75

static ssize_t attr_set_poll(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct hmc5883_data *hmc5883 = dev_get_drvdata(dev);
	unsigned long pollms = 0;

	if (strict_strtoul(buf, 10, &pollms) || pollms <= 0)
		return -EINVAL;

	if (pollms < (1000 / HMC5883_MAX_RATE))
		pollms = 1000 / HMC5883_MAX_RATE + 5;

	mutex_lock(&hmc5883->lock);
	hmc5883->delay_ms = pollms;
	mutex_unlock(&hmc5883->lock);

	return size;
}
static DEVICE_ATTR(poll, S_IRUGO | S_IWUSR, attr_get_poll, attr_set_poll);

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct hmc5883_data *hmc5883 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&hmc5883->lock);
	val = hmc5883->enabled;
	mutex_unlock(&hmc5883->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct hmc5883_data *hmc5883 = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	dev_dbg(&hmc5883->client->dev, "val=%lu\n", val);

	mutex_lock(&hmc5883->lock);
	if (val) {
		hmc5883_selftest(hmc5883);
		hmc5883_enable(hmc5883);
	} else {
		hmc5883_disable(hmc5883);
	}
	hmc5883->enabled = val;
	mutex_unlock(&hmc5883->lock);

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, attr_get_enable, attr_set_enable);

static struct attribute *hmc5883_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll.attr,
	NULL
};

static struct attribute_group hmc5883_attr_group = {
	.name = "hmc5883",
	.attrs = hmc5883_attributes
};

static int __devinit
hmc5883_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct hmc5883_data *hmc5883;
	int ret = 0;
	u8 id1, id2, id3;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		ret = -EINVAL;
		goto out;
	}

	hmc5883 = kzalloc(sizeof(struct hmc5883_data), GFP_KERNEL);
	if (!hmc5883) {
		ret = -ENOMEM;
		dev_err(&client->dev, "fail to alloc memory\n");
		goto out;
	}

	hmc5883->client = client;
	i2c_set_clientdata(client, hmc5883);
	hmc5883->delay_ms = 200;
	mutex_init(&hmc5883->lock);
	INIT_DELAYED_WORK(&hmc5883->work, hmc5883_work);

	/* Check ChipInfo */
	id1 = i2c_smbus_read_byte_data(client, HMC5883_REG_ID1);
	id2 = i2c_smbus_read_byte_data(client, HMC5883_REG_ID2);
	id3 = i2c_smbus_read_byte_data(client, HMC5883_REG_ID3);
	if (!(id1 == 'H' && id2 == '4' && id3 == '3')) {
		dev_err(&client->dev, "Identification is not correct.\n");
		goto out_free;
	}

	hmc5883_put_idle(hmc5883);

	ret = hmc5883_input_init(hmc5883);
	if (ret < 0) {
		dev_err(&client->dev, "error init input dev interface\n");
		goto out_free;
	}

	ret = sysfs_create_group(&client->dev.kobj, &hmc5883_attr_group);
	if (ret < 0) {
		dev_err(&client->dev, "sysfs register failed\n");
		goto out_free_input;
	}

	return 0;

out_free_input:
	input_unregister_device(hmc5883->input);
out_free:
	kfree(hmc5883);
out:
	return ret;
}

static int hmc5883_remove(struct i2c_client *client)
{
	struct hmc5883_data *hmc5883 = i2c_get_clientdata(client);

	mutex_lock(&hmc5883->lock);

	hmc5883_disable(hmc5883);

	sysfs_remove_group(&client->dev.kobj, &hmc5883_attr_group);
	input_unregister_device(hmc5883->input);

	mutex_unlock(&hmc5883->lock);

	kfree(hmc5883);

	return 0;
}

static const struct i2c_device_id hmc5883_id[] = {
	{ "compass", 0 },
	{}
};

static struct i2c_driver hmc5883_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= "compass",
#ifdef CONFIG_PM_SLEEP
		.pm = &hmc5883_pm_ops,
#endif
	},
	.class = I2C_CLASS_HWMON,
	.id_table = hmc5883_id,
	.probe = hmc5883_probe,
	.remove = __devexit_p(hmc5883_remove),
};

static int __init hmc5883_init(void)
{
	return i2c_add_driver(&hmc5883_driver);
}

static void __exit hmc5883_exit(void)
{
	i2c_del_driver(&hmc5883_driver);
}

module_init(hmc5883_init);
module_exit(hmc5883_exit);

MODULE_DESCRIPTION("hmc5883 MI sensor driver");
MODULE_LICENSE("GPL");

