/*
 * Copyright (C) 2012 Maxim Integrated Products, Inc.
 *
 * Driver Version: 2.0
 * Release Date: Mar 6, 2012
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
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/max11871.h>
#include <linux/gpio.h>
#include <linux/crc16.h>
#include <linux/completion.h>
#include <asm/byteorder.h>
#include <linux/firmware.h>

#define NWORDS(a)	(sizeof(a) / sizeof(u16))

#define MAX11871_CMD_ADDR 0x0000
#define MAX11871_ONE_CMD_LEN  9
#define MAX11871_MAX_CMD_LEN  (15 * MAX11871_ONE_CMD_LEN)
#define MAX11871_RPT_ADDR 0x000a
#define MAX11871_RPT_LEN  (255 - MAX11871_ONE_CMD_LEN - 1)

/* commands */
#define MAX11871_CMD_SET_TOUCH_RPT_MODE	0x0018
#define   MAX11871_TOUCH_RPT_RAW	0x00
#define   MAX11871_TOUCH_RPT_BASIC	0x01
#define   MAX11871_TOUCH_RPT_EXTEND	0x02
#define MAX11871_CMD_SET_POWER_MODE	0x0020
#define   MAX11871_POWER_SLEEP		0x0
#define   MAX11871_POWER_ACTIVE		0x2

#define MAX11871_CMD_SET_TOUCH_CFG	0x0001
#define MAX11871_CMD_GET_CFG_INF	0x0002
#define MAX11871_CMD_GET_FW_VERSION	0x0040
#define MAX11871_CMD_RESET		0x00e9

/* report id */
#define MAX11871_RPT_CFG_INF               0x0102
#define MAX11871_RPT_PRIV_CFG              0x0104
#define MAX11871_RPT_CAL_DATA              0x0111
#define MAX11871_RPT_TOUCH_RPT_MODE        0x0119
#define MAX11871_RPT_POWER_MODE            0x0121
#define MAX11871_RPT_SENSITIVITY           0x0125
#define MAX11871_RPT_FRAMERATE             0x0127
#define MAX11871_RPT_RESET_BASELINE        0x0134
#define MAX11871_RPT_FW_VERSION            0x0140
#define MAX11871_RPT_SYS_STATUS            0x01A0
#define MAX11871_RPT_INIT                  0x0400
#define MAX11871_RPT_TOUCH_RAW_IMAGE       0x0800
#define MAX11871_RPT_TOUCH_INFO_BASIC      0x0801
#define MAX11871_RPT_TOUCH_INFO_EXTENDED   0x0802

#define MAX11871_INVALID_COMMAND           0xBADC  /* Invalid cmd identifier */

/*
 * command packet format
 * header: 15 - 12                  11 - 8                 7 - 0
 *         total number of packets  current packet number  current packet size
 * id: command id
 * size: command data size in word (16-bit)
 */

struct max11871_packet {
#define PACKET_NUM_SHIFT 12
#define PACKET_CUR_SHIFT 8
	u16 header;
	u16 id;
	u16 size;
	u16 data[0];
} __packed;

struct max11871_report_finger_data {
	u16 finger_status;
#define MAX11871_FINGER_STATUS_MASK      0x0F00
#define MAX11871_FINGER_ID_MASK          0x000F

	u16 pos_x;
	u16 pos_y;
#define MAX11871_FINGER_POSITION_MASK    0x1FFF

	u16 pos_z;
#define MAX11871_Z_MASK    0x00FF
};

struct max11871_report_finger_data_extended {
	struct max11871_report_finger_data basic;
	u16 pos_x_speed;
	u16 pos_y_speed;
	u16 pos_x_size;
	u16 pos_y_size;
	u16 min_x;
	u16 min_y;
	u16 max_x;
	u16 max_y;
};

struct max11871_touch_report {
	/* note: size == (#touches * 4) + 3 */
	u16 touch_count;
	u16 gpi_button;
	u16 frame_count;

	union {
		struct max11871_report_finger_data basic_data[10];
		struct max11871_report_finger_data_extended ext_data[10];
	};
};

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
	struct mutex dev_mutex;

	u16 addr_pointer;
	u16 report[MAX11871_RPT_LEN];
	struct max11871_finger_data finger[MAX_TOUCHES_LIMIT];

	u16 wait_reportid;
	struct completion cmd_completion;

	struct early_suspend early_suspend;

	int fw_version;
	u16 fw_crc16;
	u16 touch_config[MAX11871_MAX_CMD_LEN];
	char phys[32];
	char key_phys[32];
};

/* packet size in words including the header */
static int max11871_packet_size(struct max11871_packet *p)
{
	return (p->header & 0xff) + 1;
}

static int i2c_tx_bytes(struct max11871_data *ts, const u8 *buf, u16 len)
{
	int ret;

	do {
		ret = i2c_master_send(ts->client, buf, len);
	} while (ret == -EAGAIN);

	if (ret < 0 || ret != len)
		dev_err(&ts->client->dev, "i2c tx failed, ret=%d\n", ret);

	return ret;
}

static int i2c_tx_words(struct max11871_data *ts, u16 *buf, u16 wlen)
{
	int ret;

#ifdef __BIG_ENDIAN
	int i;

	for (i = 0; i < wlen; i++)
		buf[i] = cpu_to_le16(buf[i]);
#endif

	ret = i2c_tx_bytes(ts, (u8 *)buf, wlen * 2);
	if (ret == wlen * 2)
		return wlen;
	else
		return -EIO;
}

static int i2c_rx_bytes(struct max11871_data *ts, u8 *buf, u16 len)
{
	int ret;

	do {
		ret = i2c_master_recv(ts->client, buf, len);
	} while (ret == -EAGAIN);

	if (ret < 0 || ret != len)
		dev_err(&ts->client->dev, "i2c rx failed\n");

	return ret;
}

static int i2c_rx_words(struct max11871_data *ts, u16 *buf, u16 wlen)
{
	int ret;
#ifdef __BIG_ENDIAN
	int i;
#endif

	ret = i2c_rx_bytes(ts, (u8 *)buf, wlen * 2);
	if (ret != wlen * 2)
		return -EIO;
#ifdef __BIG_ENDIAN
	for (i = 0; i < wlen; i++)
		buf[i] = le16_to_cpu(buf[i]);
#endif
	return wlen;
}

static int max11871_read_report(struct max11871_data *ts, u16 *buf)
{
	struct max11871_packet *p;
	struct device *dev = &ts->client->dev;
	int ret;
	int i;
	int words = 1;
	u16 header;
	u16 rpt_addr = MAX11871_RPT_ADDR;

	if (ts->addr_pointer != MAX11871_RPT_ADDR) {
		ret = i2c_tx_words(ts, &rpt_addr, 1);
		if (ret < 0) {
			dev_err(dev, "fail to set report address\n");
			return ret;
		}
		ts->addr_pointer = MAX11871_RPT_ADDR;
	}

	for (i = 0; i < 2; i++) {
		ret = i2c_rx_words(ts, buf, words);
		if (ret < 0)
			return ret;

		p = (struct max11871_packet *)buf;
		header = p->header;
		words = max11871_packet_size(p);
		if ((header >> 8) != 0x11 || words > MAX11871_RPT_LEN) {
			dev_err(dev, "corrupted header:0x%04x\n", header);
			return -EIO;
		}
	}

	return 0;
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
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, finger[i].z);
	}

	input_sync(ts->input_dev);
}

/*
 * Parses and passes touch report info up to the input HAL
 * expects the packet in reportBuffer to be of type 0x0801 or 0x0802,
 */
static int
max11871_process_touch_report(struct max11871_data *ts, u16 reportid, u16 *buf)
{
	struct device *dev = &ts->client->dev;
	struct max11871_touch_report *report;
	struct max11871_report_finger_data *data;
	int touch_count;
	int i;
	int x, y, z, finger_id;
	int basic;

	if (reportid == MAX11871_RPT_TOUCH_INFO_EXTENDED)
		basic = 0;
	else if (reportid == MAX11871_RPT_TOUCH_INFO_BASIC)
		basic = 1;
	else {
		dev_err(dev, "unknown touch report id 0x%04x\n", reportid);
		return -EINVAL;
	}

	report = (struct max11871_touch_report *)buf;
	touch_count = report->touch_count & 0x000f;

	if (touch_count < 0 || touch_count > 10) {
		dev_err(dev, "Touch count ==%i, out of bounds [0,10]!",
			touch_count);
		return -EINVAL;
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

	dev_dbg(dev, "touch_count = %d, isBasicReport = %d\n",
		touch_count, basic);

	for (i = 0; i < touch_count; i++) {
		if (basic == 0)
			data = &report->ext_data[i].basic;
		else
			data = &report->basic_data[i];

		x = data->pos_y;
		y = data->pos_x;
		z = data->pos_z >> 8;
		finger_id = data->finger_status & MAX11871_FINGER_ID_MASK;
		dev_dbg(dev, "x:%d y:%d z:%d fingerid:%d\n",
			x, y, z, finger_id);

		if (finger_id >= MAX_TOUCHES_LIMIT)
			continue;

		ts->finger[finger_id].status = MAX11871_FINGER_PRESS;
		ts->finger[finger_id].x = x;
		ts->finger[finger_id].y = y;
		ts->finger[finger_id].z = z;
	}

	max11871_send_touches(ts);
	return 0;
}

static int max11871_process_report(struct max11871_data *ts, u16 *buf)
{
	struct device *dev = &ts->client->dev;
	struct max11871_packet *p = (struct max11871_packet *)buf;
	int ret = 0;

	dev_dbg(dev, "report id 0x%x\n", p->id);

	/* Figure out which report we have and process it */
	switch (p->id) {
	case MAX11871_RPT_CFG_INF:
		memcpy(ts->touch_config, &p->id, (p->size + 2) * 2);
		break;
	case MAX11871_RPT_PRIV_CFG:
	case MAX11871_RPT_CAL_DATA:
	case MAX11871_RPT_TOUCH_RPT_MODE:
	case MAX11871_RPT_POWER_MODE:
	case MAX11871_RPT_SENSITIVITY:
	case MAX11871_RPT_FRAMERATE:
	case MAX11871_RPT_RESET_BASELINE:
	case MAX11871_RPT_INIT:
	case MAX11871_RPT_TOUCH_RAW_IMAGE:
		/* No handling required in driver */
		break;
	case MAX11871_RPT_SYS_STATUS:
		break;
	case MAX11871_RPT_FW_VERSION:
		ts->fw_version = buf[3];
		break;
	case MAX11871_RPT_TOUCH_INFO_BASIC:  /* Touch Report Handling */
	case MAX11871_RPT_TOUCH_INFO_EXTENDED:
		ret = max11871_process_touch_report(ts, p->id, p->data);
		break;
	case 0xFFFF:
		dev_err(dev, "Read report failed!\n");
		ret = -1;
		break;
	case MAX11871_INVALID_COMMAND:
	default:
		dev_err(dev, "0x%.4x == bad report type!!!!", p->id);
		ret = -1;
		break;
	}

	if (ts->wait_reportid == p->id)
		complete(&ts->cmd_completion);

	return ret;
}

static irqreturn_t max11871_irq_handler(int irq, void *dev_id)
{
	struct max11871_data *ts = dev_id;
	int ret;

	mutex_lock(&ts->dev_mutex);

	/* Read the entire report in from i2c */
	ret = max11871_read_report(ts, ts->report);
	if (ret < 0) {
		dev_err(&ts->client->dev, "fail to read report\n");
		goto out;
	}

	max11871_process_report(ts, ts->report);
out:
	mutex_unlock(&ts->dev_mutex);
	return IRQ_HANDLED;
}

static int max11871_send_cmd(struct max11871_data *ts, u16 *buf, u16 len)
{
	int i, ret;
	struct device *dev = &ts->client->dev;
	u16 txbuf[MAX11871_ONE_CMD_LEN + 2]; /* with address and header */
	u16 packets, words;

	if (len < 2 || len > MAX11871_MAX_CMD_LEN) {
		dev_err(dev, "wrong data len %d\n", len);
		return -EINVAL;
	}
	if (buf[1] + 2 != len) {
		dev_err(dev, "inconsistent data len expected:%d, given:%d\n",
			buf[1] + 2, len);
		return -EINVAL;
	}

	packets = (len + MAX11871_ONE_CMD_LEN - 1) / MAX11871_ONE_CMD_LEN;
	txbuf[0] = MAX11871_CMD_ADDR;
	for (i = 0; i < packets; i++) {
		words = (i == packets - 1) ? len : MAX11871_ONE_CMD_LEN;
		txbuf[1] = (packets << 12) | ((i + 1) << 8) | words;
		memcpy(&txbuf[2], &buf[i * MAX11871_ONE_CMD_LEN], words * 2);
		ret = i2c_tx_words(ts, txbuf, words + 2);
		if (ret < 0) {
			dev_err(dev, "cmd tx fail\n");
			break;
		}
		len -= MAX11871_ONE_CMD_LEN;
	}

	ts->addr_pointer = MAX11871_CMD_ADDR;
	return ret;
}

/* dev_mutex must be hold when calling this func */
static int max11871_wait_for_simple_cmd(struct max11871_data *ts,
					u16 cmd_id, u16 report_id)
{
	int ret;
	struct device *dev = &ts->client->dev;

	if (cmd_id) {
		u16 cmdbuf[] = { cmd_id, 0x0000 };

		ret = max11871_send_cmd(ts, cmdbuf, NWORDS(cmdbuf));
		if (ret < 0) {
			dev_err(dev, "fail to send cmd:0x%x\n", cmd_id);
			return ret;
		}
	}

	ts->wait_reportid = report_id;

	mutex_unlock(&ts->dev_mutex);
	ret = wait_for_completion_timeout(&ts->cmd_completion, HZ);
	mutex_lock(&ts->dev_mutex);

	if (ret == 0) {
		dev_err(dev, "timeout cmd 0x%04x report 0x%04x\n",
			cmd_id, report_id);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * COMMANDS
 */

static int max11871_wait_sys_status(struct max11871_data *ts)
{
	return max11871_wait_for_simple_cmd(ts, 0, MAX11871_RPT_SYS_STATUS);
}

static int max11871_sw_reset(struct max11871_data *ts)
{
	return max11871_wait_for_simple_cmd(ts,
			MAX11871_CMD_RESET, MAX11871_RPT_SYS_STATUS);
}

static int max11871_get_fw_version(struct max11871_data *ts)
{
	return max11871_wait_for_simple_cmd(ts,
			MAX11871_CMD_GET_FW_VERSION, MAX11871_RPT_FW_VERSION);
}

static int max11871_get_touch_config(struct max11871_data *ts)
{
	return max11871_wait_for_simple_cmd(ts,
			MAX11871_CMD_GET_CFG_INF, MAX11871_RPT_CFG_INF);
}

static int max11871_set_power_mode(struct max11871_data *ts, int mode)
{
	u16 cmdbuf[] = { MAX11871_CMD_SET_POWER_MODE, 0x0001, mode };

	return max11871_send_cmd(ts, cmdbuf, NWORDS(cmdbuf));
}

static int max11871_change_touch_rpt(struct max11871_data *ts,  int mode)
{
	u16 cmdbuf[] = { MAX11871_CMD_SET_TOUCH_RPT_MODE, 0x0001, mode };

	return max11871_send_cmd(ts, cmdbuf, NWORDS(cmdbuf));
}

static void max11871_cmd_csum(u16 *buf, u16 len)
{
	int i;
	u16 csum = 0;

	for (i = 2; i < (len - 1); i++)
		csum += buf[i];
	buf[len - 1] = csum;
}


#define BUTTON_AREA_HEIGHT 50
static int max11871_update_touch_config(struct max11871_data *ts)
{
	struct device *dev = &ts->client->dev;
	struct max11871_platform_data *pdata = ts->client->dev.platform_data;
	int ret;
	int x_range, y_range;

	ret = max11871_get_touch_config(ts);
	if (ret < 0) {
		dev_info(dev, "continue to use default touch config\n");
		return 0;
	}

	x_range = pdata->abs_x_max | 0x8000;
	y_range = pdata->abs_y_max + BUTTON_AREA_HEIGHT;
	dev_info(dev, "touch cfg X:0x%x, Y:0x%x, xrange:0x%x, yrange:0x%x\n",
		 ts->touch_config[27], ts->touch_config[28], x_range, y_range);


	if (ts->touch_config[28] != x_range ||
	    ts->touch_config[27] != y_range) {
		dev_info(dev, "touch cfg mismatch, need to update\n");

		ts->touch_config[28] = x_range;
		ts->touch_config[27] = y_range;
		ts->touch_config[0] = MAX11871_CMD_SET_TOUCH_CFG;

		max11871_cmd_csum(ts->touch_config, ts->touch_config[1] + 2);
		max11871_send_cmd(ts, ts->touch_config,
				  ts->touch_config[1] + 2);
		ret = max11871_sw_reset(ts);
		if (ret < 0)
			dev_err(dev, "time out software reset, continue\n");

		ret = max11871_get_touch_config(ts);
		if (ret < 0)
			dev_err(dev, "time out get touch config, continue\n");
	}

	return 0;
}

static int max11871_config_chip(struct max11871_data *ts)
{
	struct device *dev = &ts->client->dev;
	struct max11871_platform_data *pdata = ts->client->dev.platform_data;
	int ret;

	/* 1. reset the chip and then bring it to normal operation */
	pdata->power(0);
	/* enable irq here to start handling report */
	enable_irq(ts->client->irq);
	pdata->power(1);

	mutex_lock(&ts->dev_mutex);

	max11871_wait_sys_status(ts);
	max11871_get_fw_version(ts);

	/* 2. update touch config if needed */
	max11871_update_touch_config(ts);

	/* 3. change to basic touch report mode */
	ret = max11871_change_touch_rpt(ts, MAX11871_TOUCH_RPT_BASIC);
	if (ret < 0)
		dev_err(dev, "fail to config touch report mode\n");

	mutex_unlock(&ts->dev_mutex);
	return ret;
}

#define FW_SIZE     0x8000
#define FW_SIZE_CRC 0x7cfc
#define CHUNK_SIZE  128

#define STATUS_ADDR	0x00ff
#define DATA_ADDR	0x00fe
#define STATUS_READY	0xabcc
#define RXTX_COMPLETE	0x5432
#define CMD_CONFIRM	0x003e

static int bootloader_write_status(struct max11871_data *ts, u16 status)
{
	u16 buf[] = { STATUS_ADDR, status };

	return i2c_tx_words(ts, buf, NWORDS(buf));
}

static int bootloader_read_data(struct max11871_data *ts, u16 *data)
{
	struct device *dev = &ts->client->dev;
	int ret;
	u16 addr = DATA_ADDR;
	u16 buf[2];

	ret = i2c_tx_words(ts, &addr, 1);
	if (ret < 0) {
		dev_err(dev, "fail to send data addr\n");
		return ret;
	}
	ret = i2c_rx_words(ts, buf, NWORDS(buf));
	if (ret < 0) {
		dev_err(dev, "fail to read data register\n");
		return ret;
	}

	if (buf[1] != STATUS_READY) {
		dev_err(dev, "status is not ready\n");
		return -EINVAL;
	}
	*data = buf[0];

	return bootloader_write_status(ts, RXTX_COMPLETE);
}

static int bootloader_get_confirm(struct max11871_data *ts, int retries)
{
	u16 data;

	do {
		if (bootloader_read_data(ts, &data) >= 0) {
			if (data == CMD_CONFIRM)
				return 0;
		}
	} while (--retries > 0);

	dev_err(&ts->client->dev, "fail to get command confirmation\n");
	return -EIO;
}

static int bootloader_enter(struct max11871_data *ts)
{
	int i, ret;
	u16 enter_cmd[3][2] = {
		{ 0x7f00, 0x0047 },
		{ 0x7f00, 0x00c7 },
		{ 0x7f00, 0x0007 },
	};

	for (i = 0; i < 3; i++) {
		ret = i2c_tx_words(ts, enter_cmd[i], 2);
		if (ret < 0)
			return ret;

		usleep_range(20000, 21000);
	}

	return bootloader_get_confirm(ts, 5);
}

static int bootloader_exit(struct max11871_data *ts)
{
	int ret;
	u16 exit_cmd[] = { DATA_ADDR, 0x0001, RXTX_COMPLETE };

	ret = i2c_tx_words(ts, exit_cmd, NWORDS(exit_cmd));
	if (ret < 0) {
		dev_err(&ts->client->dev, "fail to exit bootloader\n");
		return ret;
	}
	return 0;
}

static int bootloader_check_status(struct max11871_data *ts, u16 status)
{
	u16 addr = STATUS_ADDR;
	u16 regstatus;
	int ret;

	ret = i2c_tx_words(ts, &addr, 1);
	if (ret < 0)
		return ret;

	ret = i2c_rx_words(ts, &regstatus, 1);
	if (ret < 0)
		return ret;

	if (regstatus != status) {
		dev_err(&ts->client->dev, "status expected:0x%x, get:0x%x\n",
				status, regstatus);
		return -EIO;
	}
	return 0;
}

static int bootloader_write_data(struct max11871_data *ts, u16 data)
{
	struct device *dev = &ts->client->dev;
	int ret;
	u16 buf[3] = { DATA_ADDR, data, RXTX_COMPLETE };

	ret = bootloader_check_status(ts, STATUS_READY);
	if (ret < 0)
		return ret;

	ret = i2c_tx_words(ts, buf, NWORDS(buf));
	if (ret < 0) {
		dev_err(dev, "fail to write 0x%04x to data register\n", data);
		return ret;
	}

	return 0;
}

static int bootloader_write_buf(struct max11871_data *ts, u8 *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if (bootloader_write_data(ts, buf[i]) < 0)
			return -EIO;
	}

	return 0;
}

static int bootloader_get_crc(struct max11871_data *ts, u16 *crc16, u16 len)
{
	int ret;
	struct device *dev = &ts->client->dev;
	u8 crc_cmd[] = { 0x30, 0x02, 0x00, 0x00, (len & 0xff), (len >> 8) };
	u16 data;
	u16 rx_crc16 = 0;

	ret = bootloader_write_buf(ts, crc_cmd, sizeof(crc_cmd));
	if (ret < 0) {
		dev_err(dev, "fail to write crc command\n");
		return ret;
	}
	usleep_range(200000, 201000);

	/* read low 8-bit crc */
	bootloader_read_data(ts, &data);
	rx_crc16 = (data & 0xff);

	/* read high 8-bit crc */
	bootloader_read_data(ts, &data);
	rx_crc16 = ((data & 0xff) << 8) | rx_crc16;

	if (bootloader_get_confirm(ts, 5) < 0)
		return -EIO;

	*crc16 = rx_crc16;
	return 0;
}

static int bootloader_erase_flash(struct max11871_data *ts)
{
	struct device *dev = &ts->client->dev;
	int i;
	int ret;

	ret = bootloader_write_data(ts, 0x0002);
	if (ret < 0) {
		dev_err(dev, "fail to send erase flash cmd\n");
		return ret;
	}

	for (i = 0; i < 10; i++) {
		usleep_range(60000, 61000);

		if (bootloader_get_confirm(ts, 0) == 0)
			break;
	}

	if (i == 10) {
		dev_err(dev, "flash erase failed\n");
		return -EIO;
	}

	return 0;
}

static int bootloader_write_flash(struct max11871_data *ts, const u8 *image)
{
	u8 cmd[] = { 0xf0, 0x00, 0x80, 0x00, 0x00 };
	u8 buf[130];
	int i, j, ret;
	struct device *dev = &ts->client->dev;

	ret = bootloader_write_buf(ts, cmd, sizeof(cmd));
	if (ret < 0)
		return ret;

	for (i = 0; i < FW_SIZE / CHUNK_SIZE; i++) {
		for (j = 0; j < 100; j++) {
			usleep_range(1200, 1300);
			if (bootloader_check_status(ts, STATUS_READY) == 0)
				break;
		}
		if (j == 100) {
			dev_err(dev, "fail to check status register\n");
			return -EIO;
		}

		buf[0] = ((i % 2) == 0) ? 0x00 : 0x40;
		buf[1] = 0x00;
		memcpy(buf + 2, image + i * CHUNK_SIZE, CHUNK_SIZE);
		ret = i2c_tx_bytes(ts, buf, CHUNK_SIZE + 2);
		if (ret != CHUNK_SIZE + 2) {
			dev_err(dev, "fail to write chunk %d\n", i);
			return -EIO;
		}

		ret = bootloader_write_status(ts, RXTX_COMPLETE);
		if (ret < 0) {
			dev_err(dev, "transfer failure at chunk %d\n", i);
			return -EIO;
		}
	}

	usleep_range(10000, 11000);
	if (bootloader_get_confirm(ts, 5) < 0) {
		dev_err(dev, "flash programming failed\n");
		return -EIO;
	}

	return 0;
}

static int bootloader_set_byte_mode(struct max11871_data *ts)
{
	int ret;
	u8 cmd[] = { 0x0a, 0x00 };

	ret = bootloader_write_buf(ts, cmd, sizeof(cmd));
	if (ret < 0)
		return ret;

	return bootloader_get_confirm(ts, 10);
}

static int
max11871_load_fw(struct max11871_data *ts, const struct firmware *fw)
{
	struct device *dev = &ts->client->dev;
	u16 fw_crc16 = 0, chip_crc16;
	int ret;

	fw_crc16 = crc16(fw_crc16, fw->data, FW_SIZE_CRC);

	ret = bootloader_enter(ts);
	if (ret < 0) {
		dev_err(dev, "fail to enter bootloader mode\n");
		return ret;
	}

	ret = bootloader_get_crc(ts, &chip_crc16, FW_SIZE_CRC);
	if (ret < 0) {
		dev_err(dev, "fail to get crc16 from chip\n");
		return ret;
	}

	dev_info(dev, "crc16 firmware:0x%04x, chip:0x%04x\n",
		 fw_crc16, chip_crc16);

	if (fw_crc16 != chip_crc16) {
		dev_info(dev, "crc16 mismatch, need to update firmware\n");

		ret = bootloader_erase_flash(ts);
		if (ret < 0) {
			dev_err(dev, "fail to erase chip flash\n");
			return ret;
		}

		ret = bootloader_set_byte_mode(ts);
		if (ret < 0) {
			dev_err(dev, "fail to set byte mode\n");
			return ret;
		}

		ret = bootloader_write_flash(ts, fw->data);
		if (ret < 0) {
			dev_err(dev, "fail to write flash\n");
			return ret;
		}

		fw_crc16 = 0;
		fw_crc16 = crc16(fw_crc16, fw->data, FW_SIZE);
		ret = bootloader_get_crc(ts, &chip_crc16, FW_SIZE);
		if (ret < 0) {
			dev_err(dev, "fail to get crc16 from chip\n");
			return ret;
		}
		if (fw_crc16 != chip_crc16) {
			dev_err(dev, "fail to verify programming! "
				"crc16 fw:0x%04x, chip:0x%04x not equal\n",
				fw_crc16, chip_crc16);
		}

		ret = bootloader_get_crc(ts, &chip_crc16, FW_SIZE_CRC);
		if (ret < 0) {
			dev_err(dev, "faile to get crc16 from chip\n");
			return ret;
		}
		ts->fw_crc16 = chip_crc16;
	}

	bootloader_exit(ts);
	return 0;
}

static void max11871_update_fw(const struct firmware *fw, void *context)
{
	struct max11871_data *ts = context;
	struct i2c_client *client = ts->client;
	struct device *dev = &client->dev;
	struct max11871_platform_data *pdata = ts->client->dev.platform_data;

	if (fw && fw->size >= FW_SIZE) {
		dev_info(dev, "got firmware, size:%d\n", fw->size);

		/* take chip out of reset state */
		pdata->power(1);
		if (max11871_load_fw(ts, fw) < 0)
			dev_err(dev, "firmware download failed\n");
		release_firmware(fw);
	}

	max11871_config_chip(ts);
}

static int max11871_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct max11871_data *ts = i2c_get_clientdata(client);
	int ret;

	dev_dbg(&client->dev, "enter %s\n", __func__);

	disable_irq(client->irq);

	mutex_lock(&ts->dev_mutex);
	ret = max11871_set_power_mode(ts, MAX11871_POWER_SLEEP);
	if (ret < 0)
		dev_err(&client->dev, "failed to put device to sleep\n");

	mutex_unlock(&ts->dev_mutex);
	return ret;
}

static int max11871_resume(struct i2c_client *client)
{
	struct max11871_data *ts = i2c_get_clientdata(client);
	int ret = 0;
	int i;

	dev_dbg(&client->dev, "enter %s\n", __func__);

	enable_irq(client->irq);

	mutex_lock(&ts->dev_mutex);
	for (i = 0; i < MAX_TOUCHES_LIMIT; i++) {
		if (ts->finger[i].status == MAX11871_FINGER_PRESS)
			ts->finger[i].status = MAX11871_FINGER_RELEASE;
	}

	ret = max11871_set_power_mode(ts, MAX11871_POWER_ACTIVE);
	if (ret < 0)
		dev_err(&client->dev, "failed to resume device\n");


	mutex_unlock(&ts->dev_mutex);
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

static int __devinit max11871_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct max11871_data *ts;
	struct max11871_platform_data *pdata = client->dev.platform_data;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C_FUNC_I2C not supported by i2c adapter\n");
		return -ENODEV;
	}

	if (!pdata || !pdata->platform_hw_init || !pdata->power) {
		dev_err(dev, "platform data not OK\n");
		return -EINVAL;
	}

	/* initialize GPIO pins */
	ret = pdata->platform_hw_init();
	if (ret < 0) {
		dev_err(dev, "fail to init GPIO IRQ pin\n");
		return ret;
	}

	/* reset chip */
	pdata->power(0);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		dev_err(dev, "fail to kzalloc max11871_data!\n");
		return -ENOMEM;
	}
	ts->client = client;
	i2c_set_clientdata(client, ts);
	mutex_init(&ts->dev_mutex);
	init_completion(&ts->cmd_completion);

	client->irq = gpio_to_irq(pdata->gpio_irq);
	if (client->irq < 0) {
		dev_err(dev, "fail to get irq for gpio %d\n", pdata->gpio_irq);
		ret = client->irq;
		goto out_free_ts;
	}

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		dev_err(dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto out_free_ts;
	}

	snprintf(ts->phys, sizeof(ts->phys),
			 "%s/input0", dev_name(&client->dev));

	ts->input_dev->name = "max11871_touchscreen_0";
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_SYN, ts->input_dev->evbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);

	/* multi-touch protocol type B */
	input_mt_init_slots(ts->input_dev, MAX_TOUCHES_LIMIT);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			pdata->abs_x_min, pdata->abs_x_max - 1 , 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			pdata->abs_y_min, pdata->abs_y_max - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,
			0, 255, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_err(dev, "Unable to register %s input device\n",
			   ts->input_dev->name);
		input_free_device(ts->input_dev);
		goto out_free_ts;
	}

	ret = request_threaded_irq(client->irq, NULL,
			max11871_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			client->name, ts);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		goto out_unresigter_input;
	}

	/*
	 * disable irq here to avoid receiving garbage during firwmare
	 * update and chip configuration
	 */
	disable_irq(client->irq);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
	ts->early_suspend.suspend = max11871_early_suspend;
	ts->early_suspend.resume = max11871_late_resume;
#endif
	register_early_suspend(&ts->early_suspend);

	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			"max11871.bin", dev, GFP_KERNEL, ts,
			max11871_update_fw);
	if (ret < 0) {
		dev_info(dev, "error requset firmware, continue power on\n");
		max11871_config_chip(ts);
	}

	return 0;

out_unresigter_input:
	input_unregister_device(ts->input_dev);
out_free_ts:
	kfree(ts);
	return ret;
}

static int max11871_remove(struct i2c_client *client)
{
	struct max11871_data *ts = i2c_get_clientdata(client);

	free_irq(client->irq, ts);

	unregister_early_suspend(&ts->early_suspend);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

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
	return i2c_add_driver(&max11871_driver);
}

static void __exit max11871_exit(void)
{
	i2c_del_driver(&max11871_driver);
}

module_init(max11871_init);
module_exit(max11871_exit);

MODULE_AUTHOR("Maxim Integrated Products, Inc.");
MODULE_DESCRIPTION("Maxim-IC 11871 Touchscreen Driver");
MODULE_LICENSE("GPL");

