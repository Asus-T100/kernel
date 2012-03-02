/* drivers/input/keyboard/max1871.c
 *
 * Copyright (C) 2011 Maxim Integrated Products, Inc.
 *
 * Driver Version: 1.01
 * Release Date: Dec 5, 2011
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


#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/max11871.h>
#include <linux/gpio.h>

#define MAXIM_I2C_RETRY_TIMES 10
#define MAXIM_NULL '\0'

#define MAXIM_DEBUG_VERBOSE 0
#define MAXIM_TOUCH_REPORT_MODE 0x01 /* 1=basic, 2=extended */

#define MAXIM_BUTTONS_HANDLED_IN_CHIP 0 /* Set to 1 if the chip is configured
					 * to report button touches in the
					 * touch report */
#define MAXIM_TOUCH_HOME   KEY_HOME
#define MAXIM_TOUCH_MENU   KEY_MENU
#define MAXIM_TOUCH_BACK   KEY_BACK
#define MAXIM_TOUCH_SEARCH KEY_SEARCH

#if MAXIM_DEBUG_VERBOSE
#define maxinfo(format, args...)\
do { \
	printk(KERN_INFO "%s:%i " format, __func__, __LINE__, ##args); \
} while (0);
#else
#define maxinfo(format, args...)\
do { ; } while (0);
#endif

#if MAXIM_DEBUG_VERBOSE
#define maxerr(format, args...)\
do {\
	printk(KERN_ERR "%s:%i " format, __func__, __LINE__, ##args); \
} while (0);
#else
#define maxerr(format, args...)\
do { ; } while (0);
#endif

#define MAX_TOUCHES_LIMIT	10
#define MAX11871_FINGER_NONE	0
#define MAX11871_FINGER_PRESS	1
#define MAX11871_FINGER_RELEASE	2

struct max11871_finger_data {
	int status;
	int x;
	int y;
	int z;
};

struct max11871_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *key_input_dev;
	struct max11871_finger_data finger[MAX_TOUCHES_LIMIT];
	struct early_suspend early_suspend;
	char phys[32];
	char key_phys[32];
};

static unsigned char max11871_keycode[4] = {
	MAXIM_TOUCH_HOME,
	MAXIM_TOUCH_MENU,
	MAXIM_TOUCH_BACK,
	MAXIM_TOUCH_SEARCH
};

#define DEBUG_ENABLE 0
#define DRIVER_MSG_DEBUG_ENABLE 0
#define MAX_TOUCHES_ALLOWED 2

#ifdef CONFIG_HAS_EARLYSUSPEND
static void max11871_early_suspend(struct early_suspend *h);
static void max11871_late_resume(struct early_suspend *h);
#endif

static int max11871_change_touch_rpt(struct i2c_client *client, int to);
static int max11871_get_fw_version(struct i2c_client *client);

u8 *ReportBuffer;
#define SIZE_OF_RPT_BUFFER 1024

static inline int max11871_i2c_read_bytes(
		const struct max11871_data *const maxim_ts,
		u8 *buffer, const int count) {
	int bytesRead = 0;

	bytesRead = i2c_master_recv(maxim_ts->client, buffer, count);

	return bytesRead;

}

/* returns size of packet from a valid header, returns -1 on bad header */
static int max11871_read_header(const struct max11871_data *const maxim_ts)
{
	u8 buffer[2];
	u8 byte = 0;
	int bytesRead = 0;
	int ret = 0;

	memset(buffer, 0, sizeof(buffer));

	bytesRead = i2c_master_recv(maxim_ts->client, buffer, 2);

	if (bytesRead == 2) {
		byte = buffer[0];
		buffer[0]   = buffer[1];
		buffer[1] = byte;

		if (buffer[0] != 0x11) {
			ret = -1;
			maxerr("Bad Header = 0x%.2x%.2x",
				   buffer[0], buffer[1]);
		}

		ret = buffer[1];
	} else {
		ret = -1;
	}

	return ret;
}

/*
 * Reads the report into the malloc'd buffer provided .
 * returns the report type, or <0 if there was an error
 */
static inline u16 maxim_read_report(
	const struct max11871_data *const maxim_ts, u8 *byteBuffer) {
	/* Packet header
	 * All reports begine with a u16 packet header.
	 * Bits 15-8 == 0x1100
	 * 0x00FE >= Bits 7-0 (size) >= 0x0001 or 1-244
	 */
	/*u16 report_start_addr = 0;*/
	u8 data[2];
	u16 reportType = 0;
	int wordSize = 0, bytesRead = 0, byteSize = 0, bytesTx = 0;

	/* maxinfo(""); */
	memset(data, 0x00, 2);

	if (byteBuffer == NULL) {
		maxerr("Called with no buffer for storage.\n");
		maxerr("Should be at least 1000bytes\n");
		return -1;
	}

	/*
	* When TIRQ goes low, three steps are required to read a report packet
		 from MAX11871:
	* 1) Write 0x00A to the MAX11871. This sets the Address Pointer
	* 2) Read just the "Packet Header" word to get the packet size.
		When the I2C STOP is
	*    received, the MAX11871 sets TIRQ high.
	* 3) Read the whole packet. The size to read will be the size read in
		 the packet
	*    header plus one word. *NOTE*, size is in 16bit WORDS so a size
		 of 2 equals 32bits!!!
	*    1 <= size <= 244
	*
	*/

	/* Step 1 send 0x000A */
	data[0] = 0x0A;
	data[1] = 0x00;
	bytesTx = i2c_master_send(maxim_ts->client, data, 2);
	if (bytesTx < 2) {
		maxerr("Failed to set FP31 addr ptr! bytesTx=%i\n", bytesTx);
		return -1;
	}


	/* Step 2 read header, get size */
	wordSize = max11871_read_header(maxim_ts);

	if (wordSize <= 0) {
		/*Attempting to handling error condition*/
		u8 *garbage = ReportBuffer;

		/*Set all buffer  memory to zero */
		memset(ReportBuffer, 0, SIZE_OF_RPT_BUFFER);

		bytesRead = max11871_i2c_read_bytes(maxim_ts, garbage, 490);
		maxerr("Bad header!\n");
		maxerr("trying to fix... cleared %i bytes\n", bytesRead);

		return -1;
	}

	wordSize++; /*to account for the header.*/
	byteSize = wordSize*2;

	/* Step 3 read the whole packet */
	/*maxinfo("byteBuffer = %p\n",
				byteBuffer);*/
	bytesRead = max11871_i2c_read_bytes(maxim_ts, byteBuffer, byteSize);

	if (bytesRead < byteSize) {
		maxerr("Failed to read entire packet!\n");
		maxerr("Read %i bytes\n", bytesRead);
		return -1;
	}

	reportType = ((u16)byteBuffer[3]) << 8 | (u16)byteBuffer[2];
	/*returns the report type*/
	/*maxim_print_report_type(maxim_ts,reportType);*/
	return reportType;
}

static inline int
max11871_report_button_press_chip(const struct max11871_data *ts,
				  int *isbuttonpressed)
{
	int keycode = -1;

	/*Mapping for chip buttons to Android buttons
	Button 1 : Menu Button
	Button 2 : Back Button
	Button 3 : Home Button
	Button 4 : Search Button */

	if (isbuttonpressed[0])
		keycode = MAXIM_TOUCH_MENU;
	else if (isbuttonpressed[1])
		keycode = MAXIM_TOUCH_BACK;
	else if (isbuttonpressed[2])
		keycode = MAXIM_TOUCH_HOME;
	else if (isbuttonpressed[3])
		keycode = MAXIM_TOUCH_SEARCH;

	if (keycode >= 0) {
		input_report_key(ts->key_input_dev, keycode, 1);
		input_sync(ts->key_input_dev);
	}

	return keycode;
}

static inline void
max11871_report_button_release(const struct max11871_data *ts, int keycode)
{
	/* setting zero to indicate button up */
	input_report_key(ts->key_input_dev, keycode, 0);
	input_sync(ts->key_input_dev);
}

static void max11871_send_touches(struct max11871_data *ts)
{
	int i;
	struct max11871_finger_data *finger = ts->finger;

	for (i = 0; i < MAX_TOUCHES_LIMIT; i++) {
		if (finger[i].status == MAX11871_FINGER_NONE)
			continue;

		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
				finger[i].status != MAX11871_FINGER_RELEASE);
		if (finger[i].status == MAX11871_FINGER_RELEASE) {
			finger[i].status = MAX11871_FINGER_NONE;
			continue;
		}

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, finger[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, finger[i].y);
		input_report_abs(ts->input_dev,
				 ABS_MT_TOUCH_MAJOR, finger[i].z);
	}

	input_sync(ts->input_dev);
}

/*
 * Parses and passes touch report info up to the input HAL
 * expects the packet in reportBuffer to be of type 0x0801 or 0x0802,
 * if not there will be memory access errors
 */
static inline int
max11871_process_touch_report(struct max11871_data *ts, u8 *reportBuffer)
{
	static int buttondown = -1;
	struct max11871_touch_report *report = NULL;
	struct max11871_report_finger_data *data;
	int touch_count = 0, button_count = 0, button_report[4];
	int byteCount = reportBuffer[0] + 1;
	int i;
	int x = 0, y = 0, z = 0, finger_id = 0;
	int isBasicReport = 0;
	u16 *wordBuffer = NULL;

	/* +2 accounts for the 2byte header*/
	byteCount = (2 * reportBuffer[0]) + 2;
	wordBuffer = (u16 *) reportBuffer;

	report = (struct max11871_touch_report *)reportBuffer;
	touch_count = report->tStatus_tCount & 0x000F;

	for (i = 0; i < 4; i++) {
		button_report[i] = (report->gpi_button & (1 << i)) >> i;
		button_count += button_report[i];
	}

	if (touch_count < 0 || touch_count > 10) {
		maxerr("Touch count ==%i, out of bounds [0,10]!", touch_count);
		return -1;
	}

	if (report->header.id == MAX11871_RPT_TOUCH_INFO_EXTENDED)
		isBasicReport = 0;
	else if (report->header.id == MAX11871_RPT_TOUCH_INFO_BASIC)
		isBasicReport = 1;
	else {
		maxerr("Error! Touch report is not basic OR extended!!!\n");
		maxerr("Error report not a touch report! %.4x",
			report->header.id);
		return -1;
	}

	if (MAXIM_BUTTONS_HANDLED_IN_CHIP) {
		if (button_count > 0)
			buttondown = max11871_report_button_press_chip(ts,
								button_report);
		else if (buttondown != -1) {
			max11871_report_button_release(ts, buttondown);
			buttondown = -1;
		}
	}

	for (i = 0; i < MAX_TOUCHES_LIMIT; i++) {
		/*
		 * set previously pressed finger to release, will
		 * update back to pressed if we get new finger data,
		 * otherwise report finger release.
		 */
		if (ts->finger[i].status == MAX11871_FINGER_PRESS)
			ts->finger[i].status = MAX11871_FINGER_RELEASE;
	}

	maxinfo("touch_count = %d, isBasicReport = %d\n",
		touch_count, isBasicReport);

	for (i = 0; i < touch_count; i++) {
		if (isBasicReport == 0)
			data = &report->ext_data[i].basic;
		else
			data = &report->basic_data[i];

		x = data->pos_Y;
		y = data->pos_X;
		z = data->pos_Z >> 8;
		finger_id = data->status_fingerID & 0x000F;
		if (finger_id >= MAX_TOUCHES_LIMIT)
			continue;

		x = (x > 1280) ? 0 : 1280 - x; /* Y axis reversal */

		ts->finger[finger_id].status = MAX11871_FINGER_PRESS;
		ts->finger[finger_id].x = x;
		ts->finger[finger_id].y = y;
		ts->finger[finger_id].z = z;
	}

	max11871_send_touches(ts);

	return 0;
}

static int maxim_process_report(struct max11871_data *ts,
				u16 reportType, u8 *buffer)
{
	int ret = 0;

	/* Figure out which report we have and process it */
	switch (reportType) {
	case MAX11871_RPT_CFG_INF:
	case MAX11871_RPT_PRIV_CFG:
	case MAX11871_RPT_CAL_DATA:
	case MAX11871_RPT_TOUCH_RPT_MODE:
	case MAX11871_RPT_POWER_MODE:
	case MAX11871_RPT_SENSITIVITY:
	case MAX11871_RPT_FRAMERATE:
	case MAX11871_RPT_RESET_BASELINE:
	case MAX11871_RPT_SYS_STATUS:
	case MAX11871_RPT_INIT:
	case MAX11871_RPT_TOUCH_RAW_IMAGE:
		/* No handling required in driver */
		maxinfo("Report type: %.4x not handled", reportType);
		ret = 0;
		break;
	case MAX11871_RPT_FW_VERSION:
		{
		struct max11871_simple_report *firmware = NULL;
		firmware = (struct max11871_simple_report *)buffer;
		maxinfo("Firmware Version %u detected\n", (u16)firmware->data);
		ret = 0;
		}
	break;
	case MAX11871_RPT_TOUCH_INFO_BASIC:  /* Touch Report Handling */
	case MAX11871_RPT_TOUCH_INFO_EXTENDED:
		if (max11871_process_touch_report(ts, buffer) < 0) {
			maxerr("Error processing touch report");
			ret = -1;
		} else
			ret = 0;
		break;
	case 0xFFFF:
		maxinfo("Read report failed!\n");
		ret = -1;
		break;
	case MAX11871_INVALID_COMMAND:
	default:
		maxinfo("0x%.4x == bad report type!!!!", reportType);
		ret = -1;
		break;
	}

	return ret;
}

/*
 * Finds the part on the i2c bus. If found, maxim_ts_data->client will have
 * the correct address stored.
 * Searches through all 4 addresses unless the i2c addr is non-zero
 * maxim_ts_data->firmwar_version contains the version on success
 * returns 0 on success or -1 on failure
 */
static int max11871_find_part(struct max11871_data *ts, u8 *buffer)
{
	/* 0x48, 0x49, 0x4A, 0x4B */
	int i = 0;
	struct max11871_platform_data *pdata = ts->client->dev.platform_data;

	if (!pdata) {
		/* dev_err(&ts->client->dev, "platform data is required!\n"); */
		maxerr("platform data is required\n");
		return -EINVAL;
	}

	/* Clear any pending reports if the IRQ line is low */
	/* while (pdata->get_report_ready()) { */
	while (i == 0) {
		u16 reportType = 0;
		memset(buffer, 0, SIZE_OF_RPT_BUFFER);
		reportType = maxim_read_report(ts, ReportBuffer);
		maxinfo("%s reportType: %d\n", __func__, reportType);
		/* maxim_print_report_type(ts,reportType); */
		maxim_process_report(ts, reportType, ReportBuffer);
		i++;
	}
	maxinfo("Read %i reports.\n", i);

	return 0;
}

static irqreturn_t max11871_irq_handler(int irq, void *dev_id)
{
	struct max11871_data *ts = dev_id;
	int result = 0;
	u16 reportType = 0;

	/* Read the entire report in from i2c */
	reportType = maxim_read_report(ts, ReportBuffer);

	result = maxim_process_report(ts, reportType, ReportBuffer);

	/* Wipe the buffer if we got a packet we didn't process */
	if (result < 0)
		memset(ReportBuffer, 0, SIZE_OF_RPT_BUFFER);

	return IRQ_HANDLED;
}

static int max11871_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct max11871_data *ts = NULL;
	struct max11871_platform_data *pdata =
					client->dev.platform_data;
	int ret = 0;
	int i = 0;

	ReportBuffer = kmalloc(SIZE_OF_RPT_BUFFER, GFP_KERNEL);
	if (ReportBuffer == NULL) {
		maxerr("kmalloc failed!\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		maxerr("platform data is required\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if (pdata->platform_hw_init) {
		if (pdata->platform_hw_init() < 0) {
			maxerr("Eror initializing platform data\n");
			return -EINVAL;
		}
	}

	/* Verify I2C is working */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		maxerr("need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	/* Grab some mem for the maxim data structure */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);

	if (ts == NULL) {
		maxerr("kzalloc failed!\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	/* setup i2c client data */
	ts->client = client;
	i2c_set_clientdata(client, ts);

	if (pdata->power) {
		ret = pdata->power(1);
		msleep(20);
		if (ret < 0) {
			printk(KERN_ERR "%s:power on failed\n", __func__);
			goto err_power_failed;
		}
	}

	/* Configure for Extended Touch reports */
	i = max11871_change_touch_rpt(ts->client, (int)MAXIM_TOUCH_REPORT_MODE);

	if (i < 0) {
		maxerr("Failed to configure touch mode!!!\n");
		ret = -ENODEV;
		goto err_detect_failed;
	}

	/* Allocate input device */
	ts->input_dev = input_allocate_device();

	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		maxerr("Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	/* Allocate key input device */
	ts->key_input_dev = input_allocate_device();

	if (ts->key_input_dev == NULL) {
		ret = -ENOMEM;
		maxerr("Failed to allocate key input device\n");
		goto err_input_dev_alloc_failed;
	}

	snprintf(ts->phys, sizeof(ts->phys),
			 "%s/input0", dev_name(&client->dev));
	snprintf(ts->key_phys, sizeof(ts->phys),
			 "%s/input1", dev_name(&client->dev));

	ts->input_dev->name = "max11871_touchscreen_0";
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	ts->key_input_dev->name = "max11871_key_0";
	ts->key_input_dev->phys = ts->key_phys;
	ts->key_input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);

	__set_bit(EV_KEY,    ts->key_input_dev->evbit);
	/*__set_bit(BTN_TOUCH, ts->key_input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit); */

	for (i = 0; i < ARRAY_SIZE(max11871_keycode); i++)
		set_bit(max11871_keycode[i], ts->key_input_dev->keybit);

	/* multi-touch protocol type B */
	input_mt_init_slots(ts->input_dev, MAX_TOUCHES_LIMIT);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			pdata->abs_x_min, pdata->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			pdata->abs_y_min, pdata->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			0, 255, 0, 0);

	/* Register input device */
	ret = input_register_device(ts->input_dev);

	if (ret) {
		maxerr("Unable to register %s input device\n",
			   ts->input_dev->name);
		ret = -ENODEV;
		goto err_input_register_device_failed;
	}

	ret = input_register_device(ts->key_input_dev);

	if (ret) {
		maxerr("Unable to register %s input device\n",
			   ts->key_input_dev->name);
		ret = -ENODEV;
		goto err_input_register_device_failed;
	}

	/* Setup the IRQ and irq_handler to use */
	client->irq = gpio_to_irq(pdata->gpio_irq);
	if (client->irq > 0) {
		ret = request_threaded_irq(client->irq, NULL,
				max11871_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				client->name, ts);

		if (ret < 0) {
			dev_err(&ts->client->dev, "request_irq failed\n");
			goto err_input_register_device_failed;
		}
	}

	/* Find the Max11871 on the bus */
	if (max11871_find_part(ts, ReportBuffer) < 0) {
		maxerr("Didn't find any Maxim11871 part on the i2c bus!!!.");
		ret = -ENODEV;
		goto err_detect_failed;
	}

	/* Get the firmware version */
	if (max11871_get_fw_version(ts->client) < 0) {
		maxerr("Couldn't send firmware version command!");
		ret = -ENODEV;
		goto err_detect_failed;
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = max11871_early_suspend;
	ts->early_suspend.resume = max11871_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
err_detect_failed:
	if (ts != NULL)
		kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int max11871_remove(struct i2c_client *client)
{
	struct max11871_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(client->irq, ts);

	input_unregister_device(ts->input_dev);
	kfree(ReportBuffer);
	kfree(ts);
	return 0;
}

/*
	COMMANDS
*/
static int max11871_get_fw_version(struct i2c_client *client)
{
	uint8_t data[16];
	int ret = 0;

	memset(data, 0x00, sizeof(data));
	/*
	*  Retrieve the Firmware Version
	*/

	data[0] = 0x00; /*for cmd addr pointer*/
	data[1] = 0x00;

	data[2] = 0x02;
	data[3] = 0x11;

	data[4] = MAX11871_CMD_GET_FW_VERSION; /*0x40*/
	data[5] = 0x00;

	data[6] = 0x00;
	data[7] = 0x00;

	/*
	maxinfo("FWVW: Data[0:3]: 0x%02X 0x%02X 0x%02X 0x%02X\n",
		data[0], data[1], data[2], data[3]);

	maxinfo("FWVW: Data[4:7]: 0x%02X 0x%02X 0x%02X 0x%02X\n",
		data[4], data[5], data[6], data[7]);
	*/

	/*maxinfo("-> I2C master send...\n");*/
	ret = i2c_master_send(client, data, 8);
	/*maxinfo("<- I2C master send: ret: %d\n", ret);*/

	if (ret < 0)
		maxerr("Maxim 11871 I2C failure: %d. Exiting\n", ret);

	return ret;
}

static int max11871_suspend(struct i2c_client *client, pm_message_t mesg)
{
	uint8_t data[16];
	int ret = 0;

	maxinfo("Enter max11871_suspend\n");

	disable_irq_nosync(client->irq);

	memset(data , 0x00 , sizeof(data));

	/* Set Power level to Sleep mode... */

	data[0] = 0x00;
	data[1] = 0x00;

	data[2] = 0x03;
	data[3] = 0x11;

	data[4] = MAX11871_CMD_SET_POWER_MODE;
	data[5] = 0x00;

	data[6] = 0x01;
	data[7] = 0x00;

	data[8] = 0x00; /* Sleep mode */
	data[9] = 0x00;

	ret = i2c_master_send(client, data, 10);
	if (ret < 0)
		maxerr("max11871 I2C failure on Suspend: %d. Exiting\n", ret);

	return ret;
}

static int max11871_resume(struct i2c_client *client)
{
	struct max11871_data *ts = i2c_get_clientdata(client);
	uint8_t data[16];
	int ret = 0;
	int i;

	maxinfo("Enter max11871_resume\n");

	for (i = 0; i < MAX_TOUCHES_LIMIT; i++) {
		if (ts->finger[i].status == MAX11871_FINGER_PRESS)
			ts->finger[i].status = MAX11871_FINGER_RELEASE;
	}

	memset(data , 0x00 , sizeof(data));
	/* Set Power level to Active Scan mode... */
	data[0] = 0x00; /* Sets addr ptr to command area */
	data[1] = 0x00;

	data[2] = 0x03;
	data[3] = 0x11;

	data[4] = MAX11871_CMD_SET_POWER_MODE;
	data[5] = 0x00;

	data[6] = 0x01;
	data[7] = 0x00;

	data[8] = 0x02; /* Active Scan mode */
	data[9] = 0x00;

	ret = i2c_master_send(client, data, 10);
	if (ret < 0)
		maxerr("Maxim11871 I2C failure on Resume: %d. Exiting\n", ret);

	enable_irq(client->irq);
	return ret;
}

static int max11871_change_touch_rpt(struct i2c_client *client,  int to)
{
	int ret = 0;
	uint8_t data[16];

	memset(data, 0x00, sizeof(data));

	to &= 0x00000003;  /* 0: 800, 1: 801, 2: 802 */

	/*maxinfo("Entry: new rpt code: %d\n", to);*/

	data[0] = 0x00;
	data[1] = 0x00;

	data[2] = 0x03;
	data[3] = 0x11;

	data[4] = MAX11871_CMD_SET_TOUCH_RPT_MODE; /*0x18*/
	data[5] = 0x00;

	data[6] = 0x01;
	data[7] = 0x00;

	data[8] = (unsigned char) to;
	data[9] = 0x00;


	/*
	maxinfo("B18W: Data[0:3]:  0x%02X 0x%02X 0x%02X 0x%02X\n",
		data[0], data[1], data[2], data[3]);
	maxinfo("B18W: Data[4:7]:  0x%02X 0x%02X 0x%02X 0x%02X\n",
		data[4], data[5], data[6], data[7]);
	maxinfo("B18W: Data[8:11]: 0x%02X 0x%02X 0x%02X 0x%02X\n",
		data[8], data[9], data[10], data[11]);
	*/


	ret = i2c_master_send(client, data, 10);
	if (ret < 0) {
		maxerr("Maxim 11871 I2C failure: %d. Exiting\n", ret);
		;
	}

	return ret;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void max11871_early_suspend(struct early_suspend *h)
{
	struct max11871_data *ts;
	ts = container_of(h, struct max11871_data, early_suspend);
	max11871_suspend(ts->client, PMSG_SUSPEND);
}

static void max11871_late_resume(struct early_suspend *h)
{
	struct max11871_data *ts;
	ts = container_of(h, struct max11871_data, early_suspend);
	max11871_resume(ts->client);
}
#endif

/****************************************
 *
 * Standard Driver Structures/Functions
 *
 ****************************************/
static const struct i2c_device_id max11871_id[] = {
	{ MAX11871_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, max11871_id);

static struct i2c_driver max11871_driver = {
	.probe          = max11871_probe,
	.remove         = max11871_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend        = max11871_suspend,
	.resume         = max11871_resume,
#endif
	.id_table       = max11871_id,
	.driver = {
		.name   = MAX11871_NAME,
	},
};

static int __devinit max11871_init(void)
{
	maxinfo("...\n");
	return i2c_add_driver(&max11871_driver);
}

static void __exit max11871_exit(void)
{
	maxinfo("...\n");
	i2c_del_driver(&max11871_driver);
}

module_init(max11871_init);
module_exit(max11871_exit);

MODULE_AUTHOR("Maxim Integrated Products, Inc.>");
MODULE_DESCRIPTION("Maxim-IC 11871 Touchscreen Driver");
MODULE_LICENSE("GPLv2");

