/*
 * R69001 Touchscreen Controller Driver
 * Source file
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/suspend.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define CONFIG_R69001_POLLING_TIME 10
#include <linux/r69001-ts.h>

#define R69001_TS_NAME              "r69001-ts-i2c"

/* Coordinates data register address */
#define REG_COORDINATES_DATA        0x00    /* High */
#define REG_INFO1                   0x00    /* Low */
#define REG_INFO2                   0x01
#define REG_DATA0                   0x02
#define REG_DATA1                   0x0b
#define REG_DATA2                   0x14
#define REG_DATA3                   0x1d
#define REG_DATA4                   0x26

/* One set coordinates data size */
#define ONE_SET_COORD_DATA_SIZE     9

/* Boot Mode */
#define BOOT_MODE_BOOT_ROM          0x80

/* Commands */
#define COMMAND_BOOT                0x10
#define COMMAND_FIRMWARE_UPDATE     0x20

/* Control register address */
#define REG_CONTROL                 0x1c    /* High */
#define REG_SCAN_MODE               0x00    /* Low */
#define REG_SCAN_CYCLE              0x01
#define REG_INT_POLL_CTRL           0x02
#define REG_INT_SIGNAL_OUTPUT_CTRL  0x03
#define REG_WRITE_DATA_CTRL         0x04
#define REG_READY_DATA              0x05
#define REG_SCAN_COUNTER            0x06
#define REG_FUNC_CTRL               0x0b
#define REG_LOW_POWER               0x17

/* Ready data */
#define READY_COORDINATES           0x01
#define READY_RAW                   0x02
#define READY_BASELINE              0x04
#define READY_DIFF                  0x08
#define READY_LABElMAP              0x10
#define READY_CALIBRATION           0x20
#define READY_GESTURE               0x40

/* Scan Mode */
#define SCAN_MODE_STOP              R69001_SCAN_MODE_STOP
#define SCAN_MODE_LOW_POWER         R69001_SCAN_MODE_LOW_POWER
#define SCAN_MODE_FULL_SCAN         R69001_SCAN_MODE_FULL_SCAN
#define SCAN_MODE_CALIBRATION       R69001_SCAN_MODE_CALIBRATION

/* Interrupt/Polling mode */
#define INTERRUPT_MODE              R69001_TS_INTERRUPT_MODE
#define POLLING_MODE                R69001_TS_POLLING_MODE
#define POLLING_LOW_EDGE_MODE       R69001_TS_POLLING_LOW_EDGE_MODE
#define UNKNOW_MODE                 255

struct r69001_ts_finger {
	u16 x;
	u16 y;
	u8 z;
	u8 t;
};

struct r69001_ts_before_regs {
	u8 int_signal_output_ctrl;
	u8 scan_cycle;
};

struct r69001_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *workqueue;
	struct delayed_work init_delay_work;
	struct r69001_ts_finger finger[MAX_FINGERS];
	struct r69001_io_data data;
	struct r69001_ts_before_regs regs;
	struct r69001_platform_data *pdata;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_info;
#endif
	u8 mode;
	u8 t_num;
};

/* I2C Client */
static struct i2c_client *client_r69001;
static void r69001_set_mode(struct r69001_ts_data *ts, u8 mode, u16 poll_time);

static int r69001_ts_read_data(struct r69001_ts_data *ts,
				u8 addr_h, u8 addr_l, u16 size, u8 *data)
{
	struct i2c_client *client = ts->client;
	struct i2c_msg msg[2];
	int error;
	u8 buf[2];

	buf[0] = addr_h;
	buf[1] = addr_l;

	/* Set data point */
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = buf;

	/* Byte read */
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = size;
	msg[1].buf = data;

	error = i2c_transfer(client->adapter, msg, 1);
	if (error > 0)
		error = i2c_transfer(client->adapter, msg + 1, 1);
	if (error < 0)
		dev_err(&client->dev,
			"I2C read error high: 0x%02x low:0x%02x size:%d ret:%d\n",
			addr_h, addr_l, size, error);

	return error;
}

static int
r69001_ts_write_data(struct r69001_ts_data *ts, u8 addr_h, u8 addr_l, u8 data)
{
	struct i2c_client *client = ts->client;
	struct i2c_msg msg;
	int error;
	u8 buf[3];

	buf[0] = addr_h;
	buf[1] = addr_l;
	buf[2] = data;

	/* Byte write */
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(buf);
	msg.buf = buf;

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0)
		dev_err(&client->dev,
			"I2C write error high: 0x%02x low:0x%02x data:0x%02x ret:%d\n",
			addr_h, addr_l, data, error);
	return error;
}

static void r69001_ts_report_coordinates_data(struct r69001_ts_data *ts)
{
	struct r69001_ts_finger *finger = ts->finger;
	struct input_dev *input_dev = ts->input_dev;
	u8 i;

	for (i = 0; i < ts->t_num; i++) {
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[i].t);
		input_report_abs(input_dev, ABS_MT_POSITION_X,
					MAX_X - finger[i].x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y,
					MAX_Y - finger[i].y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, finger[i].z);
		input_mt_sync(input_dev);
	}

	/* SYN_MT_REPORT only if no contact */
	if (!ts->t_num)
		input_mt_sync(input_dev);

	/* SYN_REPORT */
	input_sync(input_dev);

	ts->t_num = 0;
}

static int r69001_ts_read_coordinates_data(struct r69001_ts_data *ts)
{
	struct r69001_ts_finger *finger = ts->finger;
	u8 i;
	u8 numt;
	u8 data[ONE_SET_COORD_DATA_SIZE];
	u8 lowreg[5] = {REG_DATA0, REG_DATA1, REG_DATA2, REG_DATA3, REG_DATA4};
	int error;

	error = r69001_ts_read_data(ts,
			REG_COORDINATES_DATA, REG_INFO1, 1, &numt);
	if (error < 0)
		return error;

	numt &= 0x0f;
	if (numt > MAX_FINGERS)
		numt = MAX_FINGERS;

	for (i = 0; i < numt; i++) {
		if (i % 2) {
			finger[i].x =
				((u16)(data[7] & 0x0f) << 8) | (u16)(data[5]);
			finger[i].y =
				((u16)(data[7] & 0xf0) << 4) | (u16)(data[6]);
			finger[i].z = data[8];
			finger[i].t = (data[0] & 0xf0) >> 4;
		} else {
			error = r69001_ts_read_data(ts,
					REG_COORDINATES_DATA, lowreg[i / 2],
					ONE_SET_COORD_DATA_SIZE, data);
			if (error < 0)
				return error;
			finger[i].x =
				((u16)(data[3] & 0x0f) << 8) | (u16)(data[1]);
			finger[i].y =
				((u16)(data[3] & 0xf0) << 4) | (u16)(data[2]);
			finger[i].z = data[4];
			finger[i].t = data[0] & 0x0f;
		}
	}

	/* Only update the number when there is no error happened */
	ts->t_num = numt;
	return 0;
}

/*
 * Used for both init and ealry resume cases, to make sure Touch Panel's
 * HW is inited after Display.
 */
static void r69001_init_work_func(struct work_struct *work)
{
	struct r69001_ts_data *ts =
		container_of(work, struct r69001_ts_data, init_delay_work.work);
	struct i2c_client *client = ts->client;

	ts->data.mode.mode = UNKNOW_MODE;
	r69001_ts_write_data(ts, REG_CONTROL, REG_SCAN_CYCLE, SCAN_TIME);
	r69001_set_mode(ts, ts->mode, POLL_INTERVAL);
	enable_irq(client->irq);
}

static irqreturn_t r69001_ts_irq_handler(int irq, void *dev_id)
{
	struct r69001_ts_data *ts = dev_id;

	r69001_ts_read_coordinates_data(ts);
	r69001_ts_report_coordinates_data(ts);
	return IRQ_HANDLED;
}

/*
 * Set Int Ctl
 * mode : 0 = INT Mode, 1 = POLL Mode, 2 = POLL + INT Mode
 * poll_time : Polling interval (msec, 1 - 1000)
 *
 * The msleep(100) in this driver comes directly from Vendor's
 * driver, and can't find any explanation in the datasheet, so
 * just keep it now.
 */
static void r69001_set_mode(struct r69001_ts_data *ts, u8 mode, u16 poll_time)
{
	struct i2c_client *client = ts->client;

	if (ts->data.mode.mode == mode)
		return;

	switch (mode) {
	case INTERRUPT_MODE:
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_POLL_CTRL, INTERRUPT_MODE);

		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_STOP);
		msleep(100);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_FULL_SCAN);
		ts->data.mode.mode = mode;
		break;
	case POLLING_MODE:
	case POLLING_LOW_EDGE_MODE:
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_POLL_CTRL, POLLING_MODE);
		if (mode == POLLING_LOW_EDGE_MODE)
			r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_SIGNAL_OUTPUT_CTRL, 0x01);
		else
			r69001_ts_write_data(ts, REG_CONTROL,
				REG_INT_SIGNAL_OUTPUT_CTRL, 0x00);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_STOP);
		msleep(100);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_SCAN_MODE, SCAN_MODE_FULL_SCAN);
		r69001_ts_write_data(ts, REG_CONTROL,
				REG_WRITE_DATA_CTRL, 0x01);
		if (poll_time && poll_time <= POLL_INTERVAL_MAX)
			ts->data.mode.poll_time = poll_time;
		else
			ts->data.mode.poll_time = POLL_INTERVAL;
		ts->data.mode.mode = mode;
		break;
	default:
		dev_err(&client->dev, "Set Int Ctl bad parameter = %d\n", mode);
		break;
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void r69001_early_suspend(struct early_suspend *h)
{
	struct r69001_ts_data *ts = i2c_get_clientdata(client_r69001);
	struct i2c_client *client = ts->client;

	disable_irq(client->irq);
	r69001_ts_write_data(ts, REG_CONTROL, REG_SCAN_MODE, SCAN_MODE_STOP);
}

static void r69001_early_resume(struct early_suspend *h)
{
	struct r69001_ts_data *ts = i2c_get_clientdata(client_r69001);

	/* Need be resumed after the Display */
	queue_delayed_work(ts->workqueue, &ts->init_delay_work,
			msecs_to_jiffies(1500));
}
#endif

static int __devinit
r69001_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct r69001_ts_data *ts;
	struct input_dev *input_dev;
	struct r69001_platform_data *pdata = client->dev.platform_data;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Not I2C_FUNC_I2C\n");
		return -EIO;
	}

	ts = kzalloc(sizeof(struct r69001_ts_data), GFP_KERNEL);
	if (!ts) {
		dev_err(&client->dev, "Out of memory\n");
		return -ENOMEM;
	}
	client_r69001 = client;
	ts->client = client;
	ts->pdata = pdata;

	if (!client->irq) {
		dev_err(&client->dev, "Error, there is no IRQ number\n");
		error = -EINVAL;
		goto err1;
	}

	error = request_threaded_irq(client->irq, NULL, r69001_ts_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err1;
	}
	disable_irq(client->irq);

	ts->workqueue = create_workqueue("r69001_ts_workqueue");
	if (!ts->workqueue) {
		dev_err(&client->dev, "Unable to create workqueue\n");
		error =  -ENOMEM;
		goto err2;
	}

	INIT_DELAYED_WORK(&ts->init_delay_work, r69001_init_work_func);

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "Unable to allocated input device\n");
		error =  -ENOMEM;
		goto err3;
	}

	ts->input_dev = input_dev;

	input_dev->name = "r69001-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);

	input_set_abs_params(input_dev,
				ABS_MT_TOUCH_MAJOR, MIN_AREA, MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, MIN_X, MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, MIN_Y, MAX_Y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, MIN_Z, MAX_Z, 0, 0);

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&client->dev, "Failed to register %s input device\n",
							input_dev->name);
		goto err4;
	}

	i2c_set_clientdata(client, ts);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend_info.suspend = r69001_early_suspend;
	ts->early_suspend_info.resume = r69001_early_resume;
	ts->early_suspend_info.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&ts->early_suspend_info);
#endif

	ts->mode = INTERRUPT_MODE;
	queue_delayed_work(ts->workqueue, &ts->init_delay_work,
				msecs_to_jiffies(6000));
	return 0;

err4:
	input_free_device(ts->input_dev);
err3:
	destroy_workqueue(ts->workqueue);
err2:
	free_irq(client->irq, ts);
err1:
	kfree(ts);
	return error;
}

static int __devexit r69001_ts_remove(struct i2c_client *client)
{
	struct r69001_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend_info);
#endif
	input_unregister_device(ts->input_dev);
	destroy_workqueue(ts->workqueue);
	if (client->irq)
		free_irq(client->irq, ts);
	kfree(ts);
	return 0;
}

static const struct i2c_device_id r69001_ts_id[] = {
	{ R69001_TS_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, r69001_ts_id);

static struct i2c_driver r69001_ts_driver = {
	.probe = r69001_ts_probe,
	.remove = __devexit_p(r69001_ts_remove),
	.id_table = r69001_ts_id,
	.driver = {
		.name = R69001_TS_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_HAS_EARLYSUSPEND
		.pm = NULL,
#endif
	},
};

static int __init r69001_ts_init(void)
{
	return i2c_add_driver(&r69001_ts_driver);
}

static void __exit r69001_ts_exit(void)
{
	i2c_del_driver(&r69001_ts_driver);
}

module_init(r69001_ts_init);
module_exit(r69001_ts_exit);

MODULE_DESCRIPTION("Renesas SP Driver R69001 Touchscreen Controller Driver");
MODULE_LICENSE("GPL");
