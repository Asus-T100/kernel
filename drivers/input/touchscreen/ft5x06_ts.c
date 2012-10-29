/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 * VERSION		DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/ft5x06_ts.h>
#include <linux/earlysuspend.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/input/mt.h>

struct ts_event {
	u32 touch[10][2];
	u8 touch_point;
};

struct ft5x0x_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;

	struct early_suspend early_suspend;

	int irq;
	int irq_gpio;
	int wake_gpio;
	int reset_gpio;

	struct mutex lock;
};

static void ft5x0x_wake(struct ft5x0x_ts_data *ft)
{
	/* wake the device by wake line*/
	gpio_set_value(ft->reset_gpio, 0);
	usleep_range(6000, 7000);
	gpio_set_value(ft->reset_gpio, 1);
	msleep(500);

	enable_irq(ft->irq);
}

static int ft5x0x_i2c_rxdata(struct i2c_client *client,
	char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_i2c_txdata(struct i2c_client *client,
	char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(client, buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};

	buf[0] = addr;
	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;

}

static unsigned char ft5x0x_read_fw_ver(struct i2c_client *client)
{
	unsigned char ver;
	ft5x0x_read_reg(client, FT5X0X_REG_FIRMID, &ver);
	return ver;
}

static int ft5x0x_read_data(struct ft5x0x_ts_data *ft)
{
	struct ts_event *event = &ft->event;
	u8 buf[62] = {0};
	int ret = -1;
	u8 reg;
	int i;
	u8 num;

	ft5x0x_read_reg(ft->client, 0x02, &num);
	printk(KERN_ERR "ft5x0x touch num = %d", num);

	ret = ft5x0x_i2c_rxdata(ft->client, buf, 31);
	if (ret < 0) {
		printk(KERN_ERR "ft5x0x %s read_data i2c_rxdata failed: %d\n",
			__func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = num;

	reg = 0x03;
	for (i = 0; i < num; i++) {
		event->touch[i][0] =
			((u16)(buf[reg] & 0x0f)) << 8 | buf[reg + 1];
		event->touch[i][1] =
			((u16)(buf[reg + 2] & 0x0f)) << 8 | buf[reg + 3];

		reg += 6;
		printk(KERN_ERR "ft5x0x touch%d[%d][%d]", i, event->touch[i][0],
				event->touch[i][1]);

	}

	return 0;
}

static void ft5x0x_report_value(struct ft5x0x_ts_data *ft)
{
	struct ft5x0x_ts_data *data = ft;
	struct ts_event *event = &data->event;
	int i;

	for (i = 0; i < event->touch_point; i++) {
		input_report_abs(data->input_dev,
			ABS_MT_POSITION_X, event->touch[i][1]);
		input_report_abs(data->input_dev,
			ABS_MT_POSITION_Y, 1024 - event->touch[i][0]);
		input_mt_sync(data->input_dev);
	}
	if (event->touch_point == 0)
		input_mt_sync(data->input_dev);
	input_sync(data->input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ft;
	ft = container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk(KERN_ERR "==ft5x0x_ts_suspend=\n");
	disable_irq(ft->irq);
	/* ==set mode == poweroff */
	ft5x0x_write_reg(ft->client, FT5X0X_REG_PMODE, PMODE_HIBERNATE);
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data *ft;
	ft = container_of(handler, struct ft5x0x_ts_data, early_suspend);

	printk(KERN_ERR "==ft5x0x_ts_resume=\n");
	ft5x0x_wake(ft);
}
#endif


static irqreturn_t ft_irq_handler(int irq, void *dev_id)
{

	struct ft5x0x_ts_data *ft = dev_id;
	int ret;

	mutex_lock(&ft->lock);

	ret = ft5x0x_read_data(ft);
	if (ret == 0)
		ft5x0x_report_value(ft);

	mutex_unlock(&ft->lock);
	return IRQ_HANDLED;
}


static int ft5x0x_ts_init_gpio(struct ft5x0x_ts_data *ft5x0x_ts)
{
	int err = 0;

	printk(KERN_ERR "ft5x0x r=%d,i=%d,w=%d\n", ft5x0x_ts->reset_gpio,
		ft5x0x_ts->irq_gpio, ft5x0x_ts->wake_gpio);

	err = gpio_request(ft5x0x_ts->irq_gpio, 0);
	if (err < 0) {
		printk(KERN_ERR "ft5x0x Failed to request GPIO%d irq error=%d\n",
			ft5x0x_ts->irq_gpio, err);
		goto err_request_irq;
	}

	err = gpio_direction_input(ft5x0x_ts->irq_gpio);
	if (err) {
		printk(KERN_ERR "ft5x0x Failed on interrupt direction, error=%d\n",
			err);
		goto err_set_irq_direction;
	}

	err = gpio_request(ft5x0x_ts->reset_gpio, "ft5x0x-reset");
	if (err < 0) {
		printk(KERN_ERR "ft5x0x Failed to request GPIO%d reset error=%d\n",
			ft5x0x_ts->reset_gpio, err);
		goto err_request_reset;
	}

	err = gpio_direction_output(ft5x0x_ts->reset_gpio, 1);
	if (err) {
		printk(KERN_ERR "ft5x0x Failed on interrupt direction, error=%d\n",
			err);
		goto err_set_reset_direction;
	}

	err = gpio_request(ft5x0x_ts->wake_gpio, "ft5x0x-wake");
	if (err < 0) {
		printk(KERN_ERR "ft5x0x Failed to request GPIO%d wake error=%d\n",
			ft5x0x_ts->wake_gpio, err);
		goto err_request_wake;
	}

	err = gpio_direction_output(ft5x0x_ts->wake_gpio, 1);
	if (err) {
		printk(KERN_ERR "ft5x0x Failed on interrupt direction, error=%d\n",
			err);
		goto err_set_wake_direction;
	}

	printk(KERN_ERR "ft5x0x gpio config done\n");

	return 0;

err_set_wake_direction:
	gpio_free(ft5x0x_ts->wake_gpio);
err_request_wake:
err_set_reset_direction:
	gpio_free(ft5x0x_ts->reset_gpio);
err_request_reset:
err_set_irq_direction:
	gpio_free(ft5x0x_ts->irq_gpio);
err_request_irq:
	return err;
}

static int
ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_data *ft5x0x_ts;
	struct ft5x0x_ts_platform_data *pdata;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;

	printk(KERN_ERR "==ft5x0x_ts_probe=\n");
	printk(KERN_ERR "==ft5x0x i2c addr = %x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	printk(KERN_ERR "ft5x0x ==kzalloc=\n");
	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	ft5x0x_ts->client = client;
	i2c_set_clientdata(client, ft5x0x_ts);
	mutex_init(&ft5x0x_ts->lock);

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}
	ft5x0x_ts->reset_gpio = pdata->reset;
	ft5x0x_ts->irq_gpio = pdata->irq;
	ft5x0x_ts->wake_gpio = pdata->wake;

	err = ft5x0x_ts_init_gpio(ft5x0x_ts);
	if (err)
		goto exit_init_gpio_fail;

	printk(KERN_ERR "ft5x0x ==request_irq=\n");
	ft5x0x_ts->irq = gpio_to_irq(ft5x0x_ts->irq_gpio);

	err = request_threaded_irq(ft5x0x_ts->irq, NULL, ft_irq_handler,
				IRQF_TRIGGER_FALLING, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}


	disable_irq(ft5x0x_ts->irq);

	printk(KERN_ERR "ft5x0x ==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ft5x0x_ts->input_dev = input_dev;

	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_X, 0, 0);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);

	input_dev->name = FT5X0X_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	printk(KERN_ERR "ft5x0x ==register_early_suspend =\n");
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

	/* get some register information */
	printk(KERN_ERR "ft5x0x first read\n");

	msleep(500);
/*
	ft5x0x_write_reg(client, 0x00, 0x00);
	msleep(500);

	ft5x0x_write_reg(client, 0x00, 0x40);
	msleep(500);
	ft5x0x_read_reg(client, 0x00, &err);
	printk(KERN_ERR "ft5x0x first write mod = 0x%x\n", err);

	ft5x0x_write_reg(client, 0x00, 0x00);
	msleep(500);

	ft5x0x_read_reg(client, 0x00, &err);
	printk(KERN_ERR "ft5x0x 2nd write mod = 0x%x\n", err);

	uc_reg_value = ft5x0x_read_fw_ver(client);
	printk(KERN_ERR "ft5x0x [FST] Firmware version = 0x%x\n", uc_reg_value);

	ft5x0x_read_reg(client, 0xa5, &err);
	printk(KERN_ERR "ft5x0x power mod = 0x%x\n", err);
*/

	/* wake the CTPM */
	ft5x0x_wake(ft5x0x_ts);

/*	ft5x0x_read_reg(client, 0xa5, &err);
	printk(KERN_ERR "ft5x0x power 2nd mod = 0x%x\n", err);


	printk(KERN_ERR "ft5x0x ==probe over =\n");


	ft5x0x_read_reg(client, 0x80, &err);
	printk(KERN_ERR "ft5x0x 80 2nd mod = 0x%x\n", err);
	ft5x0x_read_reg(client, 0x81, &err);
	printk(KERN_ERR "ft5x0x 81 2nd mod = 0x%x\n", err);
	ft5x0x_read_reg(client, 0x82, &err);
	printk(KERN_ERR "ft5x0x 82 2nd mod = 0x%x\n", err);
	ft5x0x_read_reg(client, 0x83, &err);
	printk(KERN_ERR "ft5x0x 83 2nd mod = 0x%x\n", err);
*/
	return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
exit_irq_request_failed:
	gpio_free(ft5x0x_ts->wake_gpio);
	gpio_free(ft5x0x_ts->irq_gpio);
	gpio_free(ft5x0x_ts->reset_gpio);
exit_init_gpio_fail:
exit_platform_data_null:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);
	printk(KERN_ERR "ft5x0x ==ft5x0x_ts_remove=\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	free_irq(ft5x0x_ts->irq, ft5x0x_ts);
	gpio_free(ft5x0x_ts->wake_gpio);
	gpio_free(ft5x0x_ts->irq_gpio);
	gpio_free(ft5x0x_ts->reset_gpio);
	input_unregister_device(ft5x0x_ts->input_dev);
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 1,},
	{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ft5x0x_ts_init(void)
{
	int ret;
	printk(KERN_ERR "ft5x0x ==ft5x0x_ts_init==\n");
	ret = i2c_add_driver(&ft5x0x_ts_driver);
	printk(KERN_ERR " ft5x0x ret=%d\n", ret);
	return ret;
}

static void __exit ft5x0x_ts_exit(void)
{
	printk(KERN_ERR "ft5x0x ==ft5x0x_ts_exit==\n");
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
