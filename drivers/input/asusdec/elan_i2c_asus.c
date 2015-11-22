#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <asm/gpio.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
//#include <linux/earlysuspend.h>
#include <linux/freezer.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/input/mt.h>

#include "elan_i2c_asus.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static bool button_up_flag = 0;
static unsigned int ignore_count = 0;
extern int touchpad_left_key;
extern int touchpad_right_key;

#if 1
static int __elan_i2c_read_reg(struct i2c_client *client, u16 reg,
				u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret;
	int tries = 5;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;

	do{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret != 2)
			ELAN_ERR("wrong length ret: %d, msg_num: %d\n");
		else	break;
		tries--;
		ELAN_ERR("retrying __elan_i2c_read_reg:%d (%d)\n", reg, tries);
		msleep(10);
	} while (tries > 0);

	return ret != 2 ? -EIO : 0;
}
#else if
static int __elan_i2c_read_reg(struct i2c_client *client, u16 reg,
				u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret;
	int tries = 3;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		return ret;

	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;

	do{
		ret = i2c_transfer(client->adapter, &msgs[1], 1);
		if (ret != 1)
			ELAN_ERR("wrong length ret: %d, msg_num: %d\n");
		if (ret > 0)
			break;
		tries--;
		ELAN_ERR("retrying __elan_i2c_read_reg:%d (%d)\n", reg, tries);
		msleep(50);
	} while (tries > 0);

	return ret != 2 ? -EIO : 0;
}
#endif

static int elan_i2c_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	int retval;

	retval = __elan_i2c_read_reg(client, reg, val, ETP_INF_LENGTH);
	if (retval < 0) {
		dev_info(&client->dev, "reading register (0x%04x) failed!\n",
			reg);
		return retval;
	}

	return 0;
}

static int elan_i2c_write_reg_cmd(struct i2c_client *client, u16 reg, u16 cmd)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = cmd & 0xff;
	buf[3] = (cmd >> 8) & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags & I2C_M_TEN;
	msg.len = 4;
	msg.buf = buf;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		return ret;

	return ret != 1 ? -EIO : 0;
}

static int elan_i2c_get_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(client, REG_DESC, val,
				   HID_DESC_LENGTH);
}

static int elan_i2c_get_report_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(client, REG_REPORT_DESC, val,
				   ETP_REPORT_DESC_LENGTH);
}

#if 0
int elan_i2c_get_input_report(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(client, REG_INPUT_REPORT, val,
				   ETP_REPORT_LENGTH);
}
#endif

static int elan_i2c_reset(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, HID_CMD_REGISTER,
					CMD_RESET);
}

int elan_i2c_enable(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, HID_CMD_REGISTER,
					CMD_WAKE_UP);
}

int elan_i2c_disable(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, HID_CMD_REGISTER,
					CMD_SLEEP);
}

static int elan_i2c_enable_absolute_mode(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(client, ETP_CMD_REGISTER,
					CMD_ENABLE_ABS);
}

static int elan_i2c_get_x_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_X_AXIS_MAX, val);
}

static int elan_i2c_get_y_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_Y_AXIS_MAX, val);
}

static int elan_i2c_get_trace_num(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_XY_TRACE_NUM, val);
}

static int elan_i2c_get_fw_version(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_FW_VERSION, val);
}

static int elan_i2c_get_resolution(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(client, REG_RESOLUTION, val);
}
int elan_i2c_initialize(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	u8 val[ETP_REPORT_DESC_LENGTH];
	int rc;

	printk("elan_i2c_initialize start \n");

	rc = elan_i2c_reset(client);
	if (rc < 0) {
		dev_info(dev, "device reset failed.\n");
		return -1;
	}

	rc = elan_i2c_get_desc(client, val);
	if (rc < 0) {
		dev_info(dev, "couldn't get device descriptor.\n");
		return -1;
	}

	rc = elan_i2c_get_report_desc(client, val);
	if (rc < 0) {
		dev_info(dev, "fetching report descriptor failed.\n");
		return -1;
	}

	rc = elan_i2c_enable(client);
	if (rc < 0) {
		dev_info(dev, "device wake up failed.\n");
		return -1;
	}

	msleep(20);

	rc = elan_i2c_enable_absolute_mode(client);
	if (rc < 0) {
		dev_info(&client->dev, "2 can't switch to absolute mode.\n");
		return -1;
	}

	return 0;
}

void elan_i2c_report_absolute(struct elan_i2c_data *data, u8 *packet)
{
	struct input_dev *input = data->input;
	u8 *finger_data = &packet[ETP_FINGER_DATA_OFFSET];
	bool finger_on;
	int pos_x, pos_y;
	int area_x, area_y, pressure;
	int i;
	int finger_on_num = 0;//jjt for right button
	int finger_right = 0;//jjt for right button
	u8 press_button;

	//printk("jjt tp:%x\n", packet[3]);//jjt for right button
	input_mt_report_pointer_emulation(input, true);
	for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
		finger_on = (packet[3] >> (3 + i)) & 0x01;
		finger_on_num += finger_on;
	}
	for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
		finger_on = (packet[3] >> (3 + i)) & 0x01;
		if (finger_on) {
			//finger_on_num ++;
			pos_x = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
			pos_y = data->max_y -
				(((finger_data[0] & 0x0f) << 8) |
				   finger_data[2]);
			area_x = (finger_data[3] & 0x0f) * data->width_x;
			area_y = (finger_data[3] >> 4) * data->width_y;
			pressure = finger_data[4];
			if(pos_y >= 987 && pos_y <= 1480) {
				if(pos_x > (data->max_x/2)){
				   finger_right = BTN_RIGHT;//jjt for right button
                   touchpad_right_key = 1;
                   touchpad_left_key = 0;
				}
			    else{
					finger_right = BTN_LEFT;
					touchpad_left_key = 1;
                                        touchpad_right_key = 0;
			    }
				if(button_up_flag){
					ignore_count++;
					if(ignore_count > 10){
						ignore_count = 0;
						button_up_flag = 0;
					}else{
						finger_data += ETP_FINGER_DATA_LEN;
						input_mt_slot(input, i);
						input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
						continue;
					}
				}
				if((finger_on_num > 1) || (packet[3] & 0x01)){
					finger_data += ETP_FINGER_DATA_LEN;
					input_mt_slot(input, i);
					input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
					continue;
				}
			}
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
			//input_report_abs(input, ABS_MT_TRACKING_ID, i);
			input_report_abs(input, ABS_MT_POSITION_X, pos_x);
			input_report_abs(input, ABS_MT_POSITION_Y, pos_y);
			input_report_abs(input, ABS_MT_PRESSURE, pressure);
			
			/* use x-axis value as TOOL_WIDTH */
			input_report_abs(input, ABS_TOOL_WIDTH,
					 finger_data[3] & 0x0f);
			input_report_abs(input, ABS_MT_TOUCH_MAJOR,
					 max(area_x, area_y));
			input_report_abs(input, ABS_MT_TOUCH_MINOR,
					 min(area_x, area_y));
			input_report_abs(input, ABS_MT_ORIENTATION, (area_x > area_y)? 1:0);
			finger_data += ETP_FINGER_DATA_LEN;
		} else {
			input_mt_slot(input, i);
			input_mt_report_slot_state(input, MT_TOOL_FINGER, false);
		}
	}
	
	//if(finger_on_num != 1) finger_right = 0;//jjt for right button

	press_button = (packet[3] & 0x01);
	if (press_button != data->press_button ) {
		if(press_button){
			input_report_key(input, finger_right, 1);
			data->pressed_button = finger_right;
		}else{
				input_report_key(input, data->pressed_button, 0);
				button_up_flag = 1;
		}
		data->press_button = press_button;
	}

	//input_report_key(input, finger_right? BTN_RIGHT:BTN_LEFT, ((packet[3] & 0x01) == 1));//jjt for right button
	//input_mt_report_pointer_emulation(input, true);
	input_sync(input);
	//printk("jjt elan_i2c_report_absolute\n");
	return;

}

void elan_i2c_report_standard(struct asusdec_chip *ec_chip,
				     u8 *packet)
{
	u8 *finger_data = &packet[2];

	ec_chip->touchpad_data.left_btn = (finger_data[1] & LEFT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.right_btn = (finger_data[1] & RIGHT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.delta_x =
		(finger_data[2] > 0x80) ? (finger_data[2] - 0xff):finger_data[2];
	ec_chip->touchpad_data.delta_y =
		(finger_data[3] > 0x80) ? (finger_data[3] - 0xff):finger_data[3];

	input_report_rel(ec_chip->indev, REL_X, ec_chip->touchpad_data.delta_x);
	input_report_rel(ec_chip->indev, REL_Y, ec_chip->touchpad_data.delta_y);
	input_report_key(ec_chip->indev, BTN_LEFT, ec_chip->touchpad_data.left_btn);
	input_report_key(ec_chip->indev, BTN_RIGHT, ec_chip->touchpad_data.right_btn);
	input_sync(ec_chip->indev);
	printk("jjt elan_i2c_report_absolute\n");
	return;
}

int elan_i2c_check_packet(u8 *packet)
{
	u16 length = le16_to_cpu(packet[0]);
	u8 report_id = packet[HID_REPORT_ID_OFFSET];
	/*
	Length 2byte
	ID       1byte
	Data    3 or 27 byte
	*/

	//printk("elan check packet length=%x(%x), report_id=%x(%x)\n",length, ETP_REPORT_LENGTH,report_id,ETP_REPORT_ID);

	if (length == ETP_REPORT_LENGTH && report_id == ETP_REPORT_ID){
		//printk("elan absolute mode packet\n");
		return 0;
	}else if (length == 7 && report_id == 0x01){
		//printk("elan standard mode packet\n");
		return 1;
	}else{
		printk("elan packet error\n");
		return -1;
	}
}


/*
 * (value from firmware) * 10 + 790 = dpi
 * we also have to convert dpi to dots/mm (*10/254 to avoid floating point)
 */
static unsigned int elan_i2c_convert_res(unsigned int val)
{
	return (val * 10 + 790) * 10 / 254;
}

int elan_i2c_input_dev_create(struct elan_i2c_data *data)
{
	struct i2c_client *client = data->client;
	struct input_dev *input;
	unsigned int x_res, y_res;
	u8 val[3];
	int ret;

	data->input = input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	input->name = "Elan I2C Touchpad";
	input->id.bustype = BUS_I2C;
	//input->dev.parent = &data->client->dev;

	__set_bit(INPUT_PROP_POINTER, input->propbit);
	__set_bit(INPUT_PROP_BUTTONPAD, input->propbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);

	__set_bit(BTN_LEFT, input->keybit);
	__set_bit(BTN_RIGHT, input->keybit);//jjt for right button

	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_FINGER, input->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, input->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, input->keybit);
	__set_bit(BTN_TOOL_QUADTAP, input->keybit);
	__set_bit(BTN_TOOL_QUINTTAP, input->keybit);

	elan_i2c_get_x_max(client, val);
	data->max_x = (0x0f & val[1]) << 8 | val[0];

	elan_i2c_get_y_max(client, val);
	data->max_y = (0x0f & val[1]) << 8 | val[0];

	elan_i2c_get_trace_num(client, val);
	data->width_x = data->max_x / (val[0] - 1);
	data->width_y = data->max_y / (val[1] - 1);

	elan_i2c_get_resolution(client, val);
	x_res = elan_i2c_convert_res(val[0]);
	y_res = elan_i2c_convert_res(val[1]);

	input_set_drvdata(input, data);

	elan_i2c_get_fw_version(client, val);
	dev_info(&client->dev,
		"Elan I2C Trackpad Information:\n" \
		"    Firmware Version:  0x%02x%02x\n" \
		"    Max ABS X,Y:   %d,%d\n" \
		"    Width X,Y:   %d,%d\n" \
		"    Resolution X,Y:   %d,%d (dots/mm)\n",
		val[1], val[0],
		data->max_x, data->max_y, data->width_x,
		data->width_y, x_res, y_res);

	input_set_abs_params(input, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_X, x_res);
	input_abs_set_res(input, ABS_Y, y_res);
	input_set_abs_params(input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	/* handle pointer emulation and unused slots in core */
	ret = input_mt_init_slots(input, ETP_MAX_FINGERS, INPUT_MT_POINTER | INPUT_MT_DROP_UNUSED);
	if (ret) {
		dev_err(&client->dev, "allocate MT slots failed, %d\n", ret);
		goto err_free_device;
	}
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, ETP_MAX_FINGERS, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, data->max_y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(input, ABS_MT_POSITION_Y, y_res);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0,
			     15 * max(data->width_x, data->width_y), 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0,
			     15 * min(data->width_x, data->width_y), 0, 0);
#if 1
	input_set_capability(input, EV_KEY, KEY_WAKEUP);
#endif

	/* Register the device in input subsystem */
	ret = input_register_device(input);
	if (ret) {
		dev_info(&client->dev, "input device register failed, %d\n",
			ret);
		goto err_free_device;
	}

	return 0;

err_free_device:
	input_free_device(input);
	return ret;
}
//*****************************************
