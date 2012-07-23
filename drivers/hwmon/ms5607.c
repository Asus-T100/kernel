/*
 * Copyright (C) 2011 Seraphim, Inc.
 * Written by Frank Liao <frankliao@seraphim.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/ms5607.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define NAME	"baro"

#define CMD_RESET     0x1E /* ADC reset command */
#define CMD_ADC_READ  0x00 /* ADC read command */
#define CMD_ADC_CONV  0x40 /* ADC conversion command */
#define CMD_ADC_D1    0x00 /* ADC D1 conversion */
#define CMD_ADC_D2    0x10 /* ADC D2 conversion */
#define CMD_ADC_256   0x00 /* ADC OSR=256 */
#define CMD_ADC_512   0x02 /* ADC OSR=512 */
#define CMD_ADC_1024  0x04 /* ADC OSR=1024 */
#define CMD_ADC_2048  0x06 /* ADC OSR=2048 */
#define CMD_ADC_4096  0x08 /* ADC OSR=4096 */
#define CMD_PROM_RD   0xA0 /* Prom read command */

#define MS5607_COEFF_DATA_LEN 8

#define MS5607_MIN_DELAY	50
#define MS5607_DEFAULT_DELAY	200

struct ms5607_data {
	struct i2c_client *client;
	struct ms5607_platform_data *pdata;
	struct mutex lock;

	struct delayed_work input_work;
	struct workqueue_struct *workqueue;

	struct input_dev *input_dev;

	int enabled;
	bool powered;
	int delay_ms;

	unsigned int C[MS5607_COEFF_DATA_LEN]; /* calibration coefficients */
};

/*
 * send reset sequence
 */
void cmd_reset(struct i2c_client *client)
{
	u8 buf[7];

	buf[0] = 0x01;
	if (i2c_smbus_write_i2c_block_data(client, CMD_RESET, 0, buf) < 0)
		return;
	/*
	 * Reset sequence requires 2.8ms to allow the calibration
	 * PROM to be loaded into the internal register
	 */
	usleep_range(3000, 4000);
}

/*
 * read calibration coefficients
 */
unsigned int cmd_prom(struct i2c_client *client, char coef_num)
{
	int err;
	u8 buf[7];
	unsigned int calib_coeff;

	err = i2c_smbus_read_i2c_block_data(client, CMD_PROM_RD + coef_num * 2,
			2, buf);
	if (err < 0)
		return err;
	calib_coeff = 256 * buf[0];
	calib_coeff += buf[1];
	return calib_coeff;
}

/*
 * preform adc conversion
 */
unsigned long cmd_adc(struct ms5607_data *ms5607,	char cmd)
{
	int err;
	u8 buf[7];
	unsigned long temp;
	int delay;

	err = i2c_smbus_write_i2c_block_data(ms5607->client,
					CMD_ADC_CONV + cmd, 0, buf);
	if (err < 0)
		return err;

	switch (cmd & 0x0f) { /* wait necessary conversion time */
	case CMD_ADC_256:
		delay = 1000;
		break;
	case CMD_ADC_512:
		delay = 3000;
		break;
	case CMD_ADC_1024:
		delay = 4000;
		break;
	case CMD_ADC_2048:
		delay = 6000;
		break;
	case CMD_ADC_4096:
		delay = 10000;
		break;
	default:
		delay = 1000;
		break;
	}
	usleep_range(delay, delay + 1000);

	err = i2c_smbus_read_i2c_block_data(ms5607->client,
					CMD_ADC_READ, 3, buf);
	if (err < 0)
		return err;

	temp = 65536 * buf[0];
	temp += 256 * buf[1];
	temp += buf[2];
	return temp;
}

static void ms5607_read_coeff(struct ms5607_data *ms5607)
{
	unsigned char i;

	for (i = 0; i < MS5607_COEFF_DATA_LEN; i++)
		ms5607->C[i] = cmd_prom(ms5607->client, i);
}

static void ms5607_power_off(struct ms5607_data *ms5607)
{
	if (ms5607->pdata->power_off)
		ms5607->pdata->power_off();
}

static int ms5607_power_on(struct ms5607_data *ms5607)
{
	int err;

	if (ms5607->pdata->power_on) {
		err = ms5607->pdata->power_on();
		if (err < 0)
			return err;
	}

	msleep(100);
	cmd_reset(ms5607->client);
	return 0;
}

static void ms5607_get_data(struct ms5607_data *ms5607,
		unsigned long *data)
{
	/* ADC value of the pressure conversion - read D1 */
	data[0] = cmd_adc(ms5607, CMD_ADC_D1 + CMD_ADC_4096);
	/* ADC value of the temperature conversion - read D2 */
	data[1] = cmd_adc(ms5607, CMD_ADC_D2 + CMD_ADC_4096);
}

static void ms5607_report_values(struct ms5607_data *ms5607,
				unsigned long *data)
{
	input_report_rel(ms5607->input_dev, REL_X, data[0]);
	input_report_rel(ms5607->input_dev, REL_Y, data[1]);

	input_sync(ms5607->input_dev);
}

static void ms5607_enable(struct ms5607_data *ms5607)
{
	if (!ms5607->powered) {
		ms5607_power_on(ms5607);
		ms5607->powered = true;

		queue_delayed_work(ms5607->workqueue, &ms5607->input_work,
				msecs_to_jiffies(ms5607->delay_ms));
	}
}

static void ms5607_disable(struct ms5607_data *ms5607)
{
	if (ms5607->powered) {
		cancel_delayed_work_sync(&ms5607->input_work);
		ms5607_power_off(ms5607);
		ms5607->powered = false;
	}
}

static void ms5607_input_work_func(struct work_struct *work)
{
	struct ms5607_data *ms5607;
	unsigned long pt[2] = { 0 };

	ms5607 = container_of((struct delayed_work *)work, struct ms5607_data,
			input_work);

	ms5607_get_data(ms5607, pt);
	ms5607_report_values(ms5607, pt);

	queue_delayed_work(ms5607->workqueue, &ms5607->input_work,
			msecs_to_jiffies(ms5607->delay_ms));
}

static int ms5607_input_init(struct ms5607_data *ms5607)
{
	int err;

	INIT_DELAYED_WORK(&ms5607->input_work, ms5607_input_work_func);
	ms5607->input_dev = input_allocate_device();
	if (!ms5607->input_dev) {
		dev_err(&ms5607->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	ms5607->input_dev->name = "ms5607_pressure";
	ms5607->input_dev->id.bustype = BUS_I2C;
	ms5607->input_dev->dev.parent = &ms5607->client->dev;
	input_set_drvdata(ms5607->input_dev, ms5607);

	set_bit(EV_ABS, ms5607->input_dev->evbit);
	set_bit(EV_REL, ms5607->input_dev->evbit);

	set_bit(REL_X, ms5607->input_dev->relbit);
	set_bit(REL_Y, ms5607->input_dev->relbit);

	err = input_register_device(ms5607->input_dev);
	if (err) {
		dev_err(&ms5607->client->dev,
			"unable to register input polled device %s: %d\n",
			ms5607->input_dev->name, err);
		goto err1;
	}

	return 0;
err1:
	input_free_device(ms5607->input_dev);
	return err;
}

/* sysfs */
static ssize_t ms5607_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", ms5607->delay_ms);
}

static ssize_t ms5607_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long pollms;
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 10, &pollms))
		return -EINVAL;

	pollms = pollms > MS5607_MIN_DELAY ? pollms : MS5607_MIN_DELAY;
	ms5607->delay_ms = pollms;
	return count;
}

static ssize_t ms5607_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", ms5607->enabled);
}

#define MS5607_SYSFS_POWERON   1
#define MS5607_SYSFS_POWERDOWN 0

static ssize_t ms5607_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val != MS5607_SYSFS_POWERON && val != MS5607_SYSFS_POWERDOWN)
		return -EINVAL;

	mutex_lock(&ms5607->lock);
	if (val)
		ms5607_enable(ms5607);
	else
		ms5607_disable(ms5607);

	ms5607->enabled = val;

	mutex_unlock(&ms5607->lock);

	return count;
}

static ssize_t ms5607_coeff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);

	return sprintf(buf, "C0: %u\nC1: %u\nC2: %u\nC3: %u\nC4: %u\nC5: %u\n"
			"C6: %u\nC7: %u\n", ms5607->C[0], ms5607->C[1],
			ms5607->C[2], ms5607->C[3], ms5607->C[4],
			ms5607->C[5], ms5607->C[6], ms5607->C[7]);
}

static DEVICE_ATTR(coeff, S_IRUGO, ms5607_coeff_show, NULL);
static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, ms5607_delay_show,
		ms5607_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, ms5607_enable_show,
		ms5607_enable_store);

static struct attribute *ms5607_attributes[] = {
	&dev_attr_coeff.attr,
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group ms5607_attribute_group = {
	.attrs = ms5607_attributes
};

static int __devinit ms5607_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err;
	struct ms5607_data *ms5607;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL\n");
		return -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	ms5607 = kzalloc(sizeof(*ms5607), GFP_KERNEL);
	if (!ms5607) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	mutex_init(&ms5607->lock);
	ms5607->client = client;
	i2c_set_clientdata(client, ms5607);

	ms5607->pdata = kmalloc(sizeof(*ms5607->pdata), GFP_KERNEL);
	if (!ms5607->pdata) {
		dev_err(&client->dev, "insufficient memory\n");
		err = -ENOMEM;
		goto err_alloc_pdata;
	}

	memcpy(ms5607->pdata, client->dev.platform_data,
		sizeof(*ms5607->pdata));
	if (ms5607->pdata->init) {
		err = ms5607->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init error\n");
			goto err_init;
		}
	}

	ms5607->delay_ms = MS5607_DEFAULT_DELAY;

	ms5607->workqueue = create_singlethread_workqueue("ms5607");
	if (ms5607->workqueue == NULL) {
		dev_err(&client->dev, "couldn't create workqueue\n");
		err = -ENOMEM;
		goto err_create_qw;
	}

	err = ms5607_input_init(ms5607);
	if (err < 0) {
		dev_err(&client->dev, "input init error\n");
		goto err_input_init;
	}

	/* Read coefficiency data */
	ms5607_power_on(ms5607);
	ms5607_read_coeff(ms5607);

	ms5607_power_off(ms5607);
	ms5607->enabled = 0;

	err = sysfs_create_group(&client->dev.kobj, &ms5607_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs can not create group\n");
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	input_unregister_device(ms5607->input_dev);
err_input_init:
	destroy_workqueue(ms5607->workqueue);
err_create_qw:
	if (ms5607->pdata->exit)
		ms5607->pdata->exit();
err_init:
	kfree(ms5607->pdata);
err_alloc_pdata:
	kfree(ms5607);
	return err;
}

static int __devexit ms5607_remove(struct i2c_client *client)
{
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);

	input_unregister_device(ms5607->input_dev);

	flush_workqueue(ms5607->workqueue);
	destroy_workqueue(ms5607->workqueue);

	ms5607_power_off(ms5607);
	if (ms5607->pdata->exit)
		ms5607->pdata->exit();
	kfree(ms5607->pdata);
	sysfs_remove_group(&client->dev.kobj, &ms5607_attribute_group);
	kfree(ms5607);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ms5607_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);

	mutex_lock(&ms5607->lock);
	if (ms5607->enabled)
		ms5607_enable(ms5607);
	mutex_unlock(&ms5607->lock);

	return 0;
}

static int ms5607_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ms5607_data *ms5607 = i2c_get_clientdata(client);

	mutex_lock(&ms5607->lock);
	if (ms5607->enabled)
		ms5607_disable(ms5607);
	mutex_unlock(&ms5607->lock);

	return 0;
}

static const struct dev_pm_ops ms5607_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ms5607_suspend,
			ms5607_resume)
};
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id ms5607_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ms5607_id);

static struct i2c_driver ms5607_driver = {
	.driver = {
		.name = NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm = &ms5607_pm_ops,
#endif
	},
	.probe = ms5607_probe,
	.remove = __devexit_p(ms5607_remove),
	.id_table = ms5607_id,
};

static int __init ms5607_init(void)
{
	return i2c_add_driver(&ms5607_driver);
}

static void __exit ms5607_exit(void)
{
	i2c_del_driver(&ms5607_driver);
}

module_init(ms5607_init);
module_exit(ms5607_exit);

MODULE_DESCRIPTION("ms5607 pressure sensor driver");
MODULE_AUTHOR("Frank Liao <frankliao@seraphim.com.tw>");
MODULE_LICENSE("GPL");
