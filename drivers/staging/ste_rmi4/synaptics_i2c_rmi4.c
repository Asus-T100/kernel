/**
 *
 * Synaptics Register Mapped Interface (RMI4) I2C Physical Layer Driver.
 * Copyright (c) 2007-2010, Synaptics Incorporated
 *
 * Author: Js HA <js.ha@stericsson.com> for ST-Ericsson
 * Author: Naveen Kumar G <naveen.gaddipati@stericsson.com> for ST-Ericsson
 * Copyright 2010 (c) ST-Ericsson AB
 */
/*
 * This file is licensed under the GPL2 license.
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 */
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input/mt.h>
#include <linux/regulator/consumer.h>
#include <linux/earlysuspend.h>
#include <linux/synaptics_i2c_rmi4.h>
#include "synaptics_i2c_rmi4.h"

/* TODO: for multiple device support will need a per-device mutex */
#define DRIVER_NAME "rmi4_ts"

#define MAX_TOUCH_MAJOR		15
#define MAX_TOUCH_MINOR		15
#define MIN_TRACKING_ID		1
#define MAX_TRACKING_ID		10
#define MAX_RETRY_COUNT		5
#define STD_QUERY_LEN		21
#define PAGE_LEN		2
#define DATA_BUF_LEN		32
#define BUF_LEN			37
#define QUERY_LEN		9
#define DATA_LEN		12
#define HAS_TAP			0x01
#define HAS_PALMDETECT		0x01
#define HAS_ROTATE		0x02
#define HAS_TAPANDHOLD		0x02
#define HAS_DOUBLETAP		0x04
#define HAS_EARLYTAP		0x08
#define HAS_RELEASE		0x08
#define HAS_FLICK		0x10
#define HAS_PRESS		0x20
#define HAS_PINCH		0x40

#define MASK_16BIT		0xFFFF
#define MASK_8BIT		0xFF
#define MASK_7BIT		0x7F
#define MASK_5BIT		0x1F
#define MASK_4BIT		0x0F
#define MASK_3BIT		0x07
#define MASK_2BIT		0x03
#define TOUCHPAD_CTRL_INTR	0x8

#define DELTA_XPOS_THRESH	1
#define DELTA_YPOS_THRESH	1
#define TOUCH_REDUCE_MODE	1

#define F01_CTRL0_CONFIGURED (1 << 7)
#define F01_CTRL0_SLEEP      (1 << 0)
#define F01_CTRL0_NOSLEEP    (1 << 2)

#define BOOT_MODE_MOS	0
#define BOOT_MODE_COS	1

static int boot_mode;
module_param(boot_mode, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(param, "The boot mode of system");

enum finger_state {
	F11_NO_FINGER = 0,
	F11_PRESENT = 1,
	F11_INACCURATE = 2,
	F11_RESERVED = 3
};

static struct rmi4_fn_ops supported_fn_ops[] = {
	{
		.fn_number = RMI4_TOUCHPAD_FUNC_NUM,
		.detect = rmi4_touchpad_detect,
		.config = rmi4_touchpad_config,
		.irq_handler = rmi4_touchpad_irq_handler,
		.remove = rmi4_touchpad_remove,
	},
	{
		.fn_number = RMI4_BUTTON_FUNC_NUM,
		.detect  = rmi4_button_detect,
		.config = NULL,
		.irq_handler = rmi4_button_irq_handler,
		.remove = rmi4_button_remove,
	},
#ifdef DEBUG
	{
		.fn_number = RMI4_ANALOG_FUNC_NUM,
	},
	{
		.fn_number = RMI4_DEV_CTL_FUNC_NUM,
	},
	{
		.fn_number = RMI4_FLASH_FW_FUNC_NUM,
	}
#endif
};

/**
 * rmi4_set_page() - sets the page
 * @pdata: pointer to rmi4_data structure
 * @address: set the address of the page
 *
 * This function is used to set the page and returns integer.
 */
static int rmi4_set_page(struct rmi4_data *pdata, u16 address)
{
	unsigned char	txbuf[PAGE_LEN];
	int		retval;
	unsigned int	page;
	struct i2c_client *i2c = pdata->i2c_client;

	page	= ((address >> 8) & MASK_8BIT);
	if (page != pdata->current_page) {
		txbuf[0] = RMI4_PAGE_SELECT_REG;
		txbuf[1] = page;
		retval	= i2c_master_send(i2c, txbuf, PAGE_LEN);
		if (retval != PAGE_LEN)
			dev_err(&i2c->dev, "%s:failed:%d\n", __func__, retval);
		else
			pdata->current_page = page;
	} else
		retval = PAGE_LEN;
	return retval;
}

int rmi4_i2c_block_read(struct rmi4_data *pdata,
					u16 address, u8 *valp, int size)
{
	int retval = 0;
	int retry_count = 0;
	unsigned char txbuf;
	struct i2c_client *client = pdata->i2c_client;

	mutex_lock(&(pdata->rmi4_page_mutex));
	retval = rmi4_set_page(pdata, address);
	if (retval != PAGE_LEN) {
		retval = -1;
		goto exit;
	}
	txbuf = address & MASK_8BIT;
retry:
	retval = i2c_master_send(client, &txbuf, sizeof(txbuf));
	if (retval < 0) {
		dev_err(&client->dev, "%s: Write failed\n", __func__);
		goto exit;
	}
	retval = i2c_master_recv(client, valp, size);
	if (retval != size) {
		if (++retry_count == MAX_RETRY_COUNT)
			dev_err(&client->dev,
				"%s: address 0x%04x size %d failed:%d\n",
					__func__, address, size, retval);
		else {
			rmi4_set_page(pdata, address);
			goto retry;
		}
	}
exit:
	mutex_unlock(&(pdata->rmi4_page_mutex));
	return retval;
}

int rmi4_i2c_byte_read(struct rmi4_data *pdata, u16 address, u8 *valp)
{
	return rmi4_i2c_block_read(pdata, address, valp, 1);
}

int rmi4_i2c_block_write(struct rmi4_data *pdata,
					u16 address, u8 *valp, int size)
{
	int retval = 0;
	int retry_count = 0;
	unsigned char txbuf[size + 1];
	struct i2c_client *client = pdata->i2c_client;

	memcpy(txbuf + 1, valp, size);

	mutex_lock(&(pdata->rmi4_page_mutex));
	retval = rmi4_set_page(pdata, address);
	if (retval != PAGE_LEN) {
		retval = -1;
		goto exit;
	}
	txbuf[0] = address & MASK_8BIT;
retry:
	retval = i2c_master_send(client, txbuf, sizeof(txbuf));
	if (retval < 0) {
		if (++retry_count == MAX_RETRY_COUNT)
			dev_err(&client->dev,
				"%s: address 0x%04x size %d failed:%d\n",
					__func__, address, size, retval);
		else {
			rmi4_set_page(pdata, address);
			goto retry;
		}
	}
exit:
	mutex_unlock(&(pdata->rmi4_page_mutex));
	return retval;
}

int rmi4_i2c_byte_write(struct rmi4_data *pdata, u16 address, u8 data)
{
	return rmi4_i2c_block_write(pdata, address, &data, 1);
}

static int rmi4_i2c_set_bits(struct rmi4_data *pdata, u16 addr, u8 bits)
{
	int retval;
	u8 reg = 0;
	struct i2c_client *client = pdata->i2c_client;

	retval = rmi4_i2c_byte_read(pdata, addr, &reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: read 0x%x failed!\n",
						__func__, addr);
		return retval;
	}
	reg |= bits;
	retval = rmi4_i2c_byte_write(pdata, addr, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: write 0x%x failed!\n",
						__func__, addr);
		return retval;
	}
	return 0;
}

static int rmi4_i2c_clear_bits(struct rmi4_data *pdata, u16 addr, u8 bits)
{
	u8 reg = 0;
	int retval;
	struct i2c_client *client = pdata->i2c_client;

	retval = rmi4_i2c_byte_read(pdata, addr, &reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: read 0x%x failed!\n",
						__func__, addr);
		return retval;
	}
	reg &= ~bits;
	retval = rmi4_i2c_byte_write(pdata, addr, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: write 0x%x failed!\n",
						__func__, addr);
		return retval;
	}
	return 0;
}

static struct rmi4_fn_ops *get_supported_fn_ops(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(supported_fn_ops); i++) {
		if (id == supported_fn_ops[i].fn_number)
			return &supported_fn_ops[i];
	}
	return NULL;
}

/**
 * rmi4_touchpad_report() - reports for the rmi4 touchpad device
 * @pdata: pointer to rmi4_data structure
 * @rfi: pointer to rmi4_fn structure
 *
 * This function calls to reports for the rmi4 touchpad device
 */
int rmi4_touchpad_irq_handler(struct rmi4_data *pdata, struct rmi4_fn *rfi)
{
	/* number of touch points - fingers down in this case */
	int retval;
	int touch_count = 0;
	int finger, fingers_supported, finger_registers;
	int reg;
	int finger_shift;
	int x, y, wx, wy;
	enum finger_state finger_status;
	u16 data_base_addr, data_offset;
	u8 *data;
	struct rmi4_touchpad_data *touch_data;
	struct i2c_client *client = pdata->i2c_client;
	const struct rmi4_touch_calib *calib =
				&pdata->board->calibs[pdata->touch_type];

	/* get 2D sensor finger data */
	/*
	 * First get the finger status field - the size of the finger status
	 * field is determined by the number of finger supporte - 2 bits per
	 * finger, so the number of registers to read is:
	 * registerCount = ceil(numberOfFingers/4).
	 * Read the required number of registers and check each 2 bit field to
	 * determine if a finger is down.
	 */
	touch_data		= rfi->fn_data;
	fingers_supported	= rfi->num_of_data_points;
	finger_registers	= (fingers_supported + 3)/4;
	data_base_addr		= rfi->data_base_addr;

	/* Read all the finger registers data in one i2c read, twice i2c read
	 * in irq handler may cause i2c controller timeout */
	retval = rmi4_i2c_block_read(pdata, data_base_addr,
					touch_data->buffer, touch_data->size);
	if (retval != touch_data->size) {
		dev_err(&client->dev, "%s:read touch registers failed\n",
								__func__);
		return 0;
	}

	for (finger = 0; finger < fingers_supported; finger++) {
		/* determine which data byte the finger status is in */
		reg = finger / 4;
		/* bit shift to get finger's status */
		finger_shift = (finger % 4) * 2;
		finger_status =
			(touch_data->buffer[reg] >> finger_shift) & MASK_2BIT;
		/*
		 * if finger status indicates a finger is present then
		 * read the finger data and report it
		 */
		if (finger_status == F11_PRESENT ||
					finger_status == F11_INACCURATE) {
			data_offset = finger_registers +
				finger * rfi->size_of_data_register_block;
			data = touch_data->buffer + data_offset;

			x = (data[0] << 4) | (data[2] & MASK_4BIT);
			y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
			wx = (data[3] & MASK_4BIT);
			wy = (data[3] >> 4) & MASK_4BIT;

			if (calib->swap_axes)
				swap(x, y);
			if (calib->x_flip)
				x = pdata->sensor_max_x - x;
			if (calib->y_flip)
				y = pdata->sensor_max_y - y;

			input_mt_slot(pdata->input_ts_dev, finger);
			input_mt_report_slot_state(pdata->input_ts_dev,
					MT_TOOL_FINGER, true);

			input_report_abs(pdata->input_ts_dev,
					ABS_MT_TOUCH_MAJOR, max(wx , wy));
			input_report_abs(pdata->input_ts_dev,
					ABS_MT_TOUCH_MINOR, min(wx , wy));
			input_report_abs(pdata->input_ts_dev,
					ABS_MT_POSITION_X, x);
			input_report_abs(pdata->input_ts_dev,
					ABS_MT_POSITION_Y, y);

			pdata->finger_status[finger] = F11_PRESENT;

		} else if (pdata->finger_status[finger] == F11_PRESENT) {
			input_mt_slot(pdata->input_ts_dev, finger);
			input_mt_report_slot_state(pdata->input_ts_dev,
					MT_TOOL_FINGER, false);
			pdata->finger_status[finger] = F11_NO_FINGER;
		}

	}

	/* sync after groups of events */
	input_sync(pdata->input_ts_dev);

	return touch_count;
}

int rmi4_button_irq_handler(struct rmi4_data *pdata, struct rmi4_fn *rfi)
{
	int i;
	int retval = 0;
	bool bttn_down;
	u8 bttns_status;
	struct rmi4_button_data *button_data;
	struct i2c_client *client = pdata->i2c_client;

	retval = rmi4_i2c_byte_read(pdata, rfi->data_base_addr, &bttns_status);
	if (retval < 0) {
		dev_err(&client->dev, "%s: read data error!\n", __func__);
		return retval;
	}

	dev_dbg(&client->dev, "%s\n", __func__);
	button_data = rfi->fn_data;
	for (i = 0; i < button_data->num_of_bttns; i++) {
		bttn_down = (bttns_status >> i) & 0x01;
		if (bttn_down != button_data->status[i]) {
			dev_dbg(&client->dev, "%s: button %d - %d",
					__func__, i, bttn_down);
			input_report_key(pdata->input_key_dev,
					button_data->bttns_map[i], bttn_down);
			button_data->status[i] = bttn_down;
		}
	}
	input_sync(pdata->input_key_dev);

	return retval;
}

static irqreturn_t rmi4_irq_thread(int irq, void *data)
{
	u8 intr_status[4];
	int retval;
	struct rmi4_fn *rfi;
	struct rmi4_device_info	*rmi;
	struct rmi4_data *pdata = data;
	struct i2c_client *client = pdata->i2c_client;

	/*
	 * Get the interrupt status from the function $01
	 * control register+1 to find which source(s) were interrupting
	 * so we can read the data from the source(s) (2D sensor, buttons..)
	 */
	retval = rmi4_i2c_block_read(pdata, pdata->fn01_data_base_addr + 1,
					intr_status,
					pdata->number_of_interrupt_register);
	if (retval != pdata->number_of_interrupt_register) {
		dev_err(&client->dev,
			"could not read interrupt status registers\n");
		return IRQ_NONE;
	}
	dev_dbg(&client->dev,
		"%s: number_of_interrupt_register:%d, intr_status:0x%x",
		__func__, pdata->number_of_interrupt_register, intr_status[0]);
	/*
	 * check each function that has data sources and if the interrupt for
	 * that triggered then call the function's irq handler to
	 * gather data and report it to the input subsystem
	 */
	rmi = &(pdata->rmi4_mod_info);
	list_for_each_entry(rfi, &rmi->support_fn_list, link) {
		if ((intr_status[rfi->index_to_intr_reg] & rfi->intr_mask) &&
						rfi->ops->irq_handler)
			rfi->ops->irq_handler(pdata, rfi);
	}
	return IRQ_HANDLED;
}

/**
 * rmi4_rmi4_touchpad_detect() - detects the rmi4 touchpad device
 * @pdata: pointer to rmi4_data structure
 * @rfi: pointer to rmi4_fn structure
 * @interruptcount: count the number of interrupts
 *
 * This function calls to detects the rmi4 touchpad device
 */
int rmi4_touchpad_detect(struct rmi4_data *pdata, struct rmi4_fn *rfi,
						unsigned int interruptcount)
{
	u8 queries[QUERY_LEN];
	unsigned short	intr_offset;
	unsigned char	abs_data_size;
	unsigned char	abs_data_blk_size;
	unsigned char	egr_0, egr_1;
	unsigned int	all_data_blk_size;
	int	has_pinch, has_flick, has_tap;
	int	has_tapandhold, has_doubletap;
	int	has_earlytap, has_press;
	int	has_palmdetect, has_rotate;
	int	has_rel;
	int	i;
	int	retval;
	struct	rmi4_touchpad_data *touch_data;
	struct	i2c_client *client = pdata->i2c_client;

	/*
	 * need to get number of fingers supported, data size, etc.
	 * to be used when getting data since the number of registers to
	 * read depends on the number of fingers supported and data size.
	 */
	retval = rmi4_i2c_block_read(pdata, rfi->query_base_addr, queries,
							sizeof(queries));
	if (retval != sizeof(queries)) {
		dev_err(&client->dev, "%s:read function query registers\n",
							__func__);
		return retval;
	}
	/*
	 * 2D data sources have only 3 bits for the number of fingers
	 * supported - so the encoding is a bit weird.
	 */
	if ((queries[1] & MASK_3BIT) <= 4)
		/* add 1 since zero based */
		rfi->num_of_data_points = (queries[1] & MASK_3BIT) + 1;
	else {
		/*
		 * a value of 5 is up to 10 fingers - 6 and 7 are reserved
		 * (shouldn't get these i int retval;n a normal 2D source).
		 */
		if ((queries[1] & MASK_3BIT) == 5)
			rfi->num_of_data_points = 10;
	}
	/* Need to get interrupt info for handling interrupts */
	rfi->index_to_intr_reg = (interruptcount + 7)/8;
	if (rfi->index_to_intr_reg != 0)
		rfi->index_to_intr_reg -= 1;
	/*
	 * loop through interrupts for each source in fn $11
	 * and or in a bit to the interrupt mask for each.
	 */
	intr_offset = interruptcount % 8;
	rfi->intr_mask = 0;
	for (i = intr_offset;
		i < ((rfi->intr_src_count & MASK_3BIT) + intr_offset); i++)
		rfi->intr_mask |= 1 << i;

	/* Size of just the absolute data for one finger */
	abs_data_size	= queries[5] & MASK_2BIT;
	/* One each for X and Y, one for LSB for X & Y, one for W, one for Z */
	abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
	rfi->size_of_data_register_block = abs_data_blk_size;

	/*
	 * need to determine the size of data to read - this depends on
	 * conditions such as whether Relative data is reported and if Gesture
	 * data is reported.
	 */
	egr_0 = queries[7];
	egr_1 = queries[8];

	/*
	 * Get info about what EGR data is supported, whether it has
	 * Relative data supported, etc.
	 */
	has_pinch	= egr_0 & HAS_PINCH;
	has_flick	= egr_0 & HAS_FLICK;
	has_tap		= egr_0 & HAS_TAP;
	has_earlytap	= egr_0 & HAS_EARLYTAP;
	has_press	= egr_0 & HAS_PRESS;
	has_rotate	= egr_1 & HAS_ROTATE;
	has_rel		= queries[1] & HAS_RELEASE;
	has_tapandhold	= egr_0 & HAS_TAPANDHOLD;
	has_doubletap	= egr_0 & HAS_DOUBLETAP;
	has_palmdetect	= egr_1 & HAS_PALMDETECT;

	/*
	 * Size of all data including finger status, absolute data for each
	 * finger, relative data and EGR data
	 */
	all_data_blk_size =
		/* finger status, four fingers per register */
		((rfi->num_of_data_points + 3) / 4) +
		/* absolute data, per finger times number of fingers */
		(abs_data_blk_size * rfi->num_of_data_points) +
		/*
		 * two relative registers (if relative is being reported)
		 */
		2 * has_rel +
		/*
		 * F11_2D_data8 is only present if the egr_0
		 * register is non-zero.
		 */
		!!(egr_0) +
		/*
		 * F11_2D_data9 is only present if either egr_0 or
		 * egr_1 registers are non-zero.
		 */
		(egr_0 || egr_1) +
		/*
		 * F11_2D_data10 is only present if EGR_PINCH or EGR_FLICK of
		 * egr_0 reports as 1.
		 */
		!!(has_pinch | has_flick) +
		/*
		 * F11_2D_data11 and F11_2D_data12 are only present if
		 * EGR_FLICK of egr_0 reports as 1.
		 */
		2 * !!(has_flick);

	touch_data = kzalloc(sizeof(*touch_data), GFP_KERNEL);
	if (!touch_data) {
		dev_err(&client->dev, "kzalloc touchpad data failed\n");
		return -ENOMEM;
	}
	touch_data->buffer = kzalloc(all_data_blk_size, GFP_KERNEL);
	if (!touch_data->buffer) {
		dev_err(&client->dev, "kzalloc touchpad buffer failed\n");
		retval = -ENOMEM;
		goto alloc_buf_err;
	}
	touch_data->size = all_data_blk_size;
	rfi->fn_data = touch_data;
	return 0;

alloc_buf_err:
	kfree(touch_data);
	return retval;
}

int rmi4_button_detect(struct rmi4_data *pdata, struct rmi4_fn *rfi,
						unsigned int interruptcount)
{
	int i, retval, bttn_cnt;
	u8 queries[2];
	unsigned short intr_offset;
	struct rmi4_button_data *button_data;
	struct i2c_client *client = pdata->i2c_client;

	retval = rmi4_i2c_block_read(pdata, rfi->query_base_addr, queries,
							sizeof(queries));
	if (retval != sizeof(queries)) {
		dev_err(&client->dev, "%s:read query failed\n", __func__);
		return retval;
	}

	button_data = kzalloc(sizeof(*button_data), GFP_KERNEL);
	if (!button_data) {
		dev_err(&client->dev, "kzalloc button data failed\n");
		return -ENOMEM;
	}
	/* The value of the register is one less than the actual number
	 * of buttons supported */
	bttn_cnt = (queries[0] & MASK_3BIT) + 1;
	dev_dbg(&client->dev, "%d buttons detected\n", bttn_cnt);

	button_data->status = kcalloc(bttn_cnt, sizeof(bool), GFP_KERNEL);
	if (!button_data->status) {
		dev_err(&client->dev, "kcalloc button status failed\n");
		retval = -ENOMEM;
		goto alloc_status_err;
	}
	button_data->bttns_map =
			kcalloc(bttn_cnt, sizeof(unsigned char), GFP_KERNEL);
	if (!button_data->bttns_map) {
		dev_err(&client->dev, "kcalloc button map table failed\n");
		retval = -ENOMEM;
		goto alloc_map_err;
	}
	/* Set the button map table as 1, 2, 3 ... */
	for (i = 0; i < bttn_cnt; i++) {
		button_data->bttns_map[i] = i + 1;
		set_bit(i + 1, pdata->input_key_dev->keybit);
	}
	button_data->num_of_bttns = bttn_cnt;
	rfi->fn_data = button_data;

	rfi->index_to_intr_reg = (interruptcount + 7) / 8;
	if (rfi->index_to_intr_reg != 0)
		rfi->index_to_intr_reg -= 1;
	/*
	 * loop through interrupts for each source
	 * and or in a bit to the interrupt mask for each.
	 */
	intr_offset = interruptcount % 8;
	rfi->intr_mask = 0;
	for (i = intr_offset;
		i < ((rfi->intr_src_count & MASK_3BIT) + intr_offset); i++)
		rfi->intr_mask |= 1 << i;
	return 0;

alloc_map_err:
	kfree(button_data->status);
alloc_status_err:
	kfree(button_data);
	return retval;
}

void rmi4_button_remove(struct rmi4_fn *rfi)
{
	struct rmi4_button_data *bttn_data;

	if (!rfi->fn_data)
		return;
	bttn_data = rfi->fn_data;
	kfree(bttn_data->status);
	kfree(bttn_data->bttns_map);
	kfree(bttn_data);
}

void rmi4_touchpad_remove(struct rmi4_fn *rfi)
{
	struct rmi4_touchpad_data *touch_data;

	if (!rfi->fn_data)
		return;
	touch_data = rfi->fn_data;
	kfree(touch_data->buffer);
	kfree(touch_data);
}

/**
 * rmi4_rmi4_touchpad_config() - confiures the rmi4 touchpad device
 * @pdata: pointer to rmi4_data structure
 * @rfi: pointer to rmi4_fn structure
 *
 * This function calls to confiures the rmi4 touchpad device
 */
int rmi4_touchpad_config(struct rmi4_data *pdata, struct rmi4_fn *rfi)
{
	/*
	 * For the data source - print info and do any
	 * source specific configuration.
	 */
	u8 ctrl0, data[BUF_LEN];
	int retval = 0;
	u8 pos_delta[] = { DELTA_XPOS_THRESH, DELTA_YPOS_THRESH };
	struct	i2c_client *client = pdata->i2c_client;
	const struct rmi4_touch_calib *calib =
				&pdata->board->calibs[pdata->touch_type];

	/* Get and print some info about the data source... */
	/* To Query 2D devices we need to read from the address obtained
	 * from the function descriptor stored in the RMI function info.
	 */
	retval = rmi4_i2c_block_read(pdata,
				rfi->query_base_addr, data, QUERY_LEN);
	if (retval != QUERY_LEN) {
		dev_err(&client->dev, "%s:read query registers failed\n",
								__func__);
		return retval;
	}

	retval = rmi4_i2c_byte_read(pdata, rfi->ctrl_base_addr, &ctrl0);
	if (retval < 0) {
		dev_err(&client->dev, "read control 0 failed\n");
		return retval;
	}

	retval = rmi4_i2c_byte_write(pdata, rfi->ctrl_base_addr,
				(ctrl0 & ~MASK_3BIT) | TOUCH_REDUCE_MODE);
	if (retval < 0) {
		dev_err(&client->dev, "Set touch report mode failed\n");
		return retval;
	}

	retval = rmi4_i2c_block_write(pdata, rfi->ctrl_base_addr + 2,
					pos_delta, sizeof(pos_delta));
	if (retval < 0) {
		dev_err(&client->dev, "Write DELTA_POS_THRESH failed\n");
		return retval;
	}

	retval = rmi4_i2c_block_read(pdata,
				rfi->ctrl_base_addr, data, DATA_BUF_LEN);
	if (retval != DATA_BUF_LEN) {
		dev_err(&client->dev, "%s:read control registers failed\n",
								__func__);
		return retval;
	}
	/* Store these for use later*/
	pdata->sensor_max_x = ((data[6] & MASK_8BIT) << 0) |
					((data[7] & MASK_4BIT) << 8);
	pdata->sensor_max_y = ((data[8] & MASK_8BIT) << 0) |
					((data[9] & MASK_4BIT) << 8);
	if (calib->swap_axes)
		swap(pdata->sensor_max_x, pdata->sensor_max_y);
	dev_info(&client->dev, "sensor_max_x=%d, sensor_max_y=%d\n",
				pdata->sensor_max_x, pdata->sensor_max_y);
	return retval;
}

static int
rmi4_process_func(struct rmi4_data *pdata, struct rmi4_fn_desc *rmi_fd,
						int page_start, int intr_cnt)
{
	int retval, id;
	struct i2c_client *client = pdata->i2c_client;
	struct rmi4_fn *rfi = NULL;
	struct rmi4_fn_ops *fn_ops = NULL;

	dev_info(&client->dev, "fn 0x%x detected: query=0x%x, "
			"cmd=0x%x, ctrl=0x%x, data=0x%x, intr=0x%x\n",
			rmi_fd->fn_number, rmi_fd->query_base_addr,
			rmi_fd->cmd_base_addr, rmi_fd->ctrl_base_addr,
			rmi_fd->data_base_addr, rmi_fd->intr_src_count);

	id = rmi_fd->fn_number;
	if (id == RMI4_DEV_CTL_FUNC_NUM) {
		pdata->fn01_query_base_addr = rmi_fd->query_base_addr;
		pdata->fn01_ctrl_base_addr = rmi_fd->ctrl_base_addr;
		pdata->fn01_data_base_addr = rmi_fd->data_base_addr;

		retval = rmi4_i2c_set_bits(pdata, pdata->fn01_ctrl_base_addr,
							F01_CTRL0_CONFIGURED);
		if (retval < 0) {
			dev_err(&client->dev, "Set F01_CONFIGURED failed\n");
			return retval;
		}
		if (boot_mode == BOOT_MODE_COS) {
			retval = rmi4_i2c_set_bits(pdata,
					pdata->fn01_ctrl_base_addr,
					F01_CTRL0_SLEEP);
			if (retval < 0) {
				dev_err(&client->dev,
					"set F01_CTRL0_SLEEP failed\n");
				return retval;
			}
			msleep(RMI4_RESET_DELAY);
			dev_info(&client->dev,
				"System is in charger mode, "
				"touch screen should be always in sleep mode, "
				"we don't need to go on anymore\n");
			return -1;

		} else {
			retval = rmi4_i2c_clear_bits(pdata,
					pdata->fn01_ctrl_base_addr,
					F01_CTRL0_NOSLEEP);
			if (retval < 0) {
				dev_err(&client->dev,
					"clear F01_CTRL0_SLEEP failed\n");
				return retval;
			}
		}
	}

	fn_ops = get_supported_fn_ops(id);
	if (!fn_ops)
		return 0;
	rfi = kzalloc(sizeof(*rfi), GFP_KERNEL);
	if (!rfi) {
		dev_err(&client->dev, "kzalloc fn%d rfi failed\n", id);
		return -ENOMEM;
	}
	rfi->fn_number = rmi_fd->fn_number;
	rfi->intr_src_count = rmi_fd->intr_src_count;
	rfi->query_base_addr = page_start + rmi_fd->query_base_addr;
	rfi->cmd_base_addr = page_start + rmi_fd->cmd_base_addr;
	rfi->ctrl_base_addr = page_start + rmi_fd->ctrl_base_addr;
	rfi->data_base_addr = page_start + rmi_fd->data_base_addr;
	rfi->ops = fn_ops;

	if (rfi->ops->detect) {
		retval = rfi->ops->detect(pdata, rfi, intr_cnt);
		if (retval < 0) {
			dev_err(&client->dev, "fn 0x%x init failed\n", id);
			goto init_err;
		}
	}
	/* link this function info to the RMI module */
	list_add_tail(&rfi->link, &pdata->rmi4_mod_info.support_fn_list);
	return 0;

init_err:
	kfree(rfi);
	return retval;
}

static void rmi4_free_funcs(struct rmi4_data *rmi4_data)
{
	struct rmi4_fn *rfi, *next;
	struct list_head *fn_list;

	fn_list = &(rmi4_data->rmi4_mod_info.support_fn_list);
	list_for_each_entry_safe(rfi, next, fn_list, link) {
		if (rfi->ops->remove)
			rfi->ops->remove(rfi);
		list_del(&rfi->link);
		kfree(rfi);
	}
}

static int do_init_reset(struct rmi4_data *pdata)
{
	bool has_f01 = false;
	bool has_f34 = false;
	int i, retval;
	int page, page_start, pdt_start, pdt_end;
	struct rmi4_fn_desc rmi_fd, f34_fd, f01_fd;
	struct i2c_client *client = pdata->i2c_client;

	for (page = 0; page <= RMI4_MAX_PAGE; page++) {
		page_start = page * RMI4_PAGE_SIZE;
		pdt_start = page_start + PDT_START_SCAN_LOCATION;
		pdt_end = page_start + PDT_END_SCAN_LOCATION;
		for (i = pdt_start; i >= pdt_end; i -= PDT_ENTRY_SIZE) {
			retval = rmi4_i2c_block_read(pdata, i,
						(u8 *)&rmi_fd,
						sizeof(rmi_fd));
			if (retval < 0) {
				dev_err(&client->dev, "%s: read 0x%x failed\n",
								__func__, i);
				return retval;
			}
			if (RMI4_END_OF_PDT(rmi_fd.fn_number))
				break;

			if (rmi_fd.fn_number == RMI4_DEV_CTL_FUNC_NUM) {
				u16 addr = page_start + rmi_fd.cmd_base_addr;
				u8 cmd = RMI4_DEVICE_RESET_CMD;
				retval = rmi4_i2c_byte_write(pdata, addr, cmd);
				if (retval < 0) {
					dev_err(&client->dev,
							"reset cmd failed.\n");
					return retval;
				}
				msleep(RMI4_RESET_DELAY);
				memcpy(&f01_fd, &rmi_fd, sizeof(rmi_fd));
				has_f01 = true;
			} else if (rmi_fd.fn_number == RMI4_FLASH_FW_FUNC_NUM) {
				memcpy(&f34_fd, &rmi_fd, sizeof(rmi_fd));
				has_f34 = true;
			}
		}
		if (has_f01 && has_f34)
			break;
	}

	if (!has_f01 || !has_f34) {
		dev_err(&client->dev,
			"%s: Failed to find F01/F34 for init reset.\n",
			__func__);
		return -ENODEV;
	}
	retval = rmi4_fw_update(pdata, &f01_fd, &f34_fd);
	if (retval < 0) {
		dev_err(&client->dev,
				"%s: update firmware failed!\n", __func__);
		return retval;
	}
	pdata->touch_type = retval;

	return 0;
}

/**
 * rmi4_i2c_query_device() - query the rmi4 device
 * @pdata: pointer to rmi4_data structure
 *
 * This function is used to query the rmi4 device.
 */
static int rmi4_i2c_query_device(struct rmi4_data *pdata)
{
	int i, retval;
	int page, page_start, pdt_start, pdt_end;
	int data_sources = 0;
	u8 std_queries[STD_QUERY_LEN];
	unsigned char intr_count = 0;
	unsigned int ctrl_offset;
	struct rmi4_fn_desc rmi_fd;
	struct rmi4_fn *rfi;
	struct rmi4_device_info *rmi;
	struct	i2c_client *client = pdata->i2c_client;

	/*
	 * init the physical drivers RMI module
	 * info list of functions
	 */
	INIT_LIST_HEAD(&pdata->rmi4_mod_info.support_fn_list);

	/*
	 * Read the Page Descriptor Table to determine what functions
	 * are present
	 */
	for (page = 0; page <= RMI4_MAX_PAGE; page++) {
		page_start = page * RMI4_PAGE_SIZE;
		pdt_start = page_start + PDT_START_SCAN_LOCATION;
		pdt_end = page_start + PDT_END_SCAN_LOCATION;
		for (i = pdt_start; i >= pdt_end; i -= PDT_ENTRY_SIZE) {
			retval = rmi4_i2c_block_read(pdata, i,
						(u8 *)&rmi_fd,
						sizeof(rmi_fd));
			if (retval < 0) {
				dev_err(&client->dev, "%s: read 0x%x failed",
								__func__, i);
				goto failed;
			}
			if (RMI4_END_OF_PDT(rmi_fd.fn_number))
				break;

			retval = rmi4_process_func(pdata, &rmi_fd,
						page_start, intr_count);
			if (retval < 0) {
				dev_err(&client->dev,
						"%s: process fn%x failed",
						__func__, rmi_fd.fn_number);
				goto failed;
			}
			/* interrupt count for next iteration */
			intr_count += rmi_fd.intr_src_count & MASK_3BIT;
		}
	}
	dev_dbg(&client->dev, "End of PDT, intr_count=%d\n", intr_count);
	/*
	 * calculate the interrupt register count - used in the
	 * ISR to read the correct number of interrupt registers
	 */
	pdata->number_of_interrupt_register = (intr_count + 7) / 8;
	/*
	 * Function $01 will be used to query the product properties,
	 * and product ID  so we had to read the PDT above first to get
	 * the Fn $01 query address and prior to filling in the product
	 * info. NOTE: Even an unflashed device will still have FN $01.
	 */

	/* Load up the standard queries and get the RMI4 module info */
	retval = rmi4_i2c_block_read(pdata, pdata->fn01_query_base_addr,
					std_queries, sizeof(std_queries));
	if (retval != sizeof(std_queries)) {
		dev_err(&client->dev,
				"%s: Failed reading queries\n", __func__);
		retval = -EIO;
		goto failed;
	}

	/* Currently supported RMI version is 4.0 */
	pdata->rmi4_mod_info.version_major	= 4;
	pdata->rmi4_mod_info.version_minor	= 0;
	/*
	 * get manufacturer id, product_props, product info,
	 * date code, tester id, serial num and product id (name)
	 */
	pdata->rmi4_mod_info.manufacturer_id	= std_queries[0];
	pdata->rmi4_mod_info.product_props	= std_queries[1];
	pdata->rmi4_mod_info.product_info[0]	= std_queries[2];
	pdata->rmi4_mod_info.product_info[1]	= std_queries[3];
	/* year - 2001-2032 */
	pdata->rmi4_mod_info.date_code[0]	= std_queries[4] & MASK_5BIT;
	/* month - 1-12 */
	pdata->rmi4_mod_info.date_code[1]	= std_queries[5] & MASK_4BIT;
	/* day - 1-31 */
	pdata->rmi4_mod_info.date_code[2]	= std_queries[6] & MASK_5BIT;
	pdata->rmi4_mod_info.tester_id = ((std_queries[7] & MASK_7BIT) << 8) |
						(std_queries[8] & MASK_7BIT);
	pdata->rmi4_mod_info.serial_number =
		((std_queries[9] & MASK_7BIT) << 8) |
				(std_queries[10] & MASK_7BIT);
	memcpy(pdata->rmi4_mod_info.product_id_string, &std_queries[11], 10);

	/* Check if this is a Synaptics device - report if not. */
	if (pdata->rmi4_mod_info.manufacturer_id != 1)
		dev_err(&client->dev, "%s: non-Synaptics mfg id:%d\n",
			__func__, pdata->rmi4_mod_info.manufacturer_id);

	list_for_each_entry(rfi, &pdata->rmi4_mod_info.support_fn_list, link)
		data_sources += rfi->intr_src_count;
	if (!data_sources)
		return 0;

	rmi = &(pdata->rmi4_mod_info);
	/* Disable all the interrupt source before we enable
	 * the supported function's interrupt source. */
	for (i = 0; i < pdata->number_of_interrupt_register; i++)
		rmi4_i2c_byte_write(pdata,
				pdata->fn01_ctrl_base_addr + 1 + i, 0);
	list_for_each_entry(rfi, &rmi->support_fn_list, link) {
		if (rfi->ops->config) {
			retval = rfi->ops->config(pdata, rfi);
			if (retval < 0) {
				dev_err(&client->dev,
						"fn 0x%x config failed\n",
						rfi->fn_number);
				goto failed;
			}
		}
		/* Turn on interrupt for this function data source. */
		ctrl_offset = pdata->fn01_ctrl_base_addr + 1 +
						rfi->index_to_intr_reg;
		retval = rmi4_i2c_set_bits(pdata, ctrl_offset, rfi->intr_mask);
		if (retval < 0) {
			dev_err(&client->dev,
					"fn 0x%x enable interrupt failed\n",
					rfi->fn_number);
			goto failed;
		}
	}
	return 0;
failed:
	return retval;
}

static int rmi4_config_gpio(struct rmi4_data *pdata)
{
	int ret, int_gpio, rst_gpio;

	int_gpio = pdata->board->int_gpio_number;
	rst_gpio = pdata->board->rst_gpio_number;

	ret = gpio_request(int_gpio, "rmi4_int");
	if (ret < 0) {
		pr_err("Failed to request INT GPIO %d\n", int_gpio);
		goto err_out;
	}
	ret = gpio_direction_input(int_gpio);
	if (ret < 0) {
		pr_err("Failed to config INT GPIO %d\n", int_gpio);
		goto err_int;
	}
	ret = gpio_to_irq(int_gpio);
	if (ret < 0) {
		pr_err("Config GPIO %d to IRQ Error!\n", int_gpio);
		goto err_int;
	}
	pdata->irq = ret;

	ret = gpio_request(rst_gpio, "rmi4_rst");
	if (ret < 0) {
		pr_err("Failed to request RST GPIO %d\n", rst_gpio);
		goto err_int;
	}
	ret = gpio_direction_output(rst_gpio, 1);
	if (ret < 0) {
		pr_err("Failed to config GPIO %d\n", rst_gpio);
		goto err_rst;
	}
	gpio_set_value(rst_gpio, 1);
	msleep(RMI4_RESET_DELAY);

	return 0;

err_rst:
	gpio_free(rst_gpio);
err_int:
	gpio_free(int_gpio);
err_out:
	return ret;
}

#ifdef DEBUG
/* sysfs entries for debug */
static ssize_t attr_ctrl_reg_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	struct rmi4_fn *rfi;
	struct list_head *fn_list;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	fn_list = &(rmi4_data->rmi4_mod_info.support_fn_list);
	list_for_each_entry(rfi, fn_list, link) {
		if (rfi->ops->fn_number == rmi4_data->dbg_fn_num) {
			rmi4_i2c_byte_write(rmi4_data,
				rfi->ctrl_base_addr + rmi4_data->dbg_reg_addr,
				(u8)val);
			return size;
		}
	}
	return -EINVAL;
}

static ssize_t attr_ctrl_reg_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct rmi4_fn *rfi;
	struct list_head *fn_list;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	fn_list = &(rmi4_data->rmi4_mod_info.support_fn_list);
	list_for_each_entry(rfi, fn_list, link) {
		if (rfi->ops->fn_number == rmi4_data->dbg_fn_num) {
			rmi4_i2c_byte_read(rmi4_data,
				rfi->ctrl_base_addr + rmi4_data->dbg_reg_addr,
				&val);
			return sprintf(buf, "%d(0x%x)\n", val, val);
		}
	}
	return -EINVAL;
}
static DEVICE_ATTR(ctrl_reg, S_IRUGO | S_IWUSR,
		attr_ctrl_reg_get, attr_ctrl_reg_set);

static ssize_t attr_query_reg_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	struct rmi4_fn *rfi;
	struct list_head *fn_list;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	fn_list = &(rmi4_data->rmi4_mod_info.support_fn_list);
	list_for_each_entry(rfi, fn_list, link) {
		if (rfi->ops->fn_number == rmi4_data->dbg_fn_num) {
			rmi4_i2c_byte_write(rmi4_data,
				rfi->query_base_addr + rmi4_data->dbg_reg_addr,
				(u8)val);
			return size;
		}
	}
	return -EINVAL;
}

static ssize_t attr_query_reg_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct rmi4_fn *rfi;
	struct list_head *fn_list;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	fn_list = &(rmi4_data->rmi4_mod_info.support_fn_list);
	list_for_each_entry(rfi, fn_list, link) {
		if (rfi->ops->fn_number == rmi4_data->dbg_fn_num) {
			rmi4_i2c_byte_read(rmi4_data,
				rfi->query_base_addr + rmi4_data->dbg_reg_addr,
				&val);
			return sprintf(buf, "%d(0x%x)\n", val, val);
		}
	}
	return -EINVAL;
}
static DEVICE_ATTR(query_reg, S_IRUGO | S_IWUSR,
		attr_query_reg_get, attr_query_reg_set);

static ssize_t attr_data_reg_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	struct rmi4_fn *rfi;
	struct list_head *fn_list;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	fn_list = &(rmi4_data->rmi4_mod_info.support_fn_list);
	list_for_each_entry(rfi, fn_list, link) {
		if (rfi->ops->fn_number == rmi4_data->dbg_fn_num) {
			rmi4_i2c_byte_write(rmi4_data,
				rfi->data_base_addr + rmi4_data->dbg_reg_addr,
				(u8)val);
			return size;
		}
	}
	return -EINVAL;
}

static ssize_t attr_data_reg_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct rmi4_fn *rfi;
	struct list_head *fn_list;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	fn_list = &(rmi4_data->rmi4_mod_info.support_fn_list);
	list_for_each_entry(rfi, fn_list, link) {
		if (rfi->ops->fn_number == rmi4_data->dbg_fn_num) {
			rmi4_i2c_byte_read(rmi4_data,
				rfi->data_base_addr + rmi4_data->dbg_reg_addr,
				&val);
			return sprintf(buf, "%d(0x%x)\n", val, val);
		}
	}
	return -EINVAL;
}
static DEVICE_ATTR(data_reg, S_IRUGO | S_IWUSR,
		attr_data_reg_get, attr_data_reg_set);

static ssize_t attr_reg_addr_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	rmi4_data->dbg_reg_addr = val;

	return size;
}

static ssize_t attr_reg_addr_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return sprintf(buf, "%d(0x%x)\n",
			rmi4_data->dbg_reg_addr, rmi4_data->dbg_reg_addr);
}
static DEVICE_ATTR(reg_addr, S_IRUGO | S_IWUSR,
			attr_reg_addr_get, attr_reg_addr_set);

static ssize_t attr_fn_num_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;
	rmi4_data->dbg_fn_num = val;

	return size;
}

static ssize_t attr_fn_num_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	return sprintf(buf, "0x%x\n", rmi4_data->dbg_fn_num);
}
static DEVICE_ATTR(fn_num, S_IRUGO | S_IWUSR,
		attr_fn_num_get, attr_fn_num_set);

static ssize_t attr_reg_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long val;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	rmi4_i2c_byte_write(rmi4_data, rmi4_data->dbg_reg_addr, (u8)val);
	return size;
}

static ssize_t attr_reg_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	struct rmi4_data *rmi4_data = dev_get_drvdata(dev);

	rmi4_i2c_byte_read(rmi4_data, rmi4_data->dbg_reg_addr, &val);
	return sprintf(buf, "%d(0x%x)\n", val, val);
}
static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, attr_reg_get, attr_reg_set);

static struct attribute *rmi4_attrs[] = {
	&dev_attr_ctrl_reg.attr,
	&dev_attr_query_reg.attr,
	&dev_attr_data_reg.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_fn_num.attr,
	&dev_attr_reg.attr,
	NULL
};

static struct attribute_group rmi4_attr_dbg = {
	.name = "rmi4",
	.attrs = rmi4_attrs
};
#endif

/**
 * rmi4_probe() - Initialze the i2c-client touchscreen driver
 * @client: i2c client structure pointer
 * @dev_id:i2c device id pointer
 *
 * This function will allocate and initialize the instance
 * data and request the irq and set the instance data as the clients
 * platform data then register the physical driver which will do a scan of
 * the rmi4 Physical Device Table and enumerate any rmi4 functions that
 * have data sources associated with them.
 */
static int __devinit rmi4_probe(struct i2c_client *client,
					const struct i2c_device_id *dev_id)
{
	int retval;
	u8 intr_status[4];
	struct rmi4_data *rmi4_data;
	const struct rmi4_platform_data *platformdata =
						client->dev.platform_data;
	const struct rmi4_touch_calib *calib;

	if (!platformdata) {
		dev_err(&client->dev, "%s: no platform data\n", __func__);
		return -EINVAL;
	}

	/* Allocate and initialize the instance data for this client */
	rmi4_data = kzalloc(sizeof(struct rmi4_data), GFP_KERNEL);
	if (!rmi4_data) {
		dev_err(&client->dev, "%s: no memory allocated\n", __func__);
		return -ENOMEM;
	}

	rmi4_data->input_ts_dev = input_allocate_device();
	if (rmi4_data->input_ts_dev == NULL) {
		dev_err(&client->dev, "ts input device alloc failed\n");
		retval = -ENOMEM;
		goto err_input_ts;
	}
	rmi4_data->input_key_dev = input_allocate_device();
	if (rmi4_data->input_key_dev == NULL) {
		dev_err(&client->dev, "key input device alloc failed\n");
		retval = -ENOMEM;
		goto err_input_key;
	}

	if (platformdata->regulator_en && platformdata->regulator_name) {
		rmi4_data->regulator = regulator_get(&client->dev,
					platformdata->regulator_name);
		if (IS_ERR(rmi4_data->regulator)) {
			dev_err(&client->dev, "get regulator %s failed\n",
					platformdata->regulator_name);
			retval = PTR_ERR(rmi4_data->regulator);
			goto err_regulator;
		}
		retval = regulator_enable(rmi4_data->regulator);
		if (retval < 0) {
			dev_err(&client->dev,
				"enable regulator %s failed with ret %d\n",
				platformdata->regulator_name, retval);
			regulator_put(rmi4_data->regulator);
			goto err_regulator;
		}
	}

	/*
	 * Copy i2c_client pointer into RTID's i2c_client pointer for
	 * later use in rmi4_read, rmi4_write, etc.
	 */
	rmi4_data->i2c_client		= client;
	/* So we set the page correctly the first time */
	rmi4_data->current_page		= MASK_16BIT;
	rmi4_data->board		= platformdata;

	mutex_init(&(rmi4_data->rmi4_page_mutex));

	retval = rmi4_config_gpio(rmi4_data);
	if (retval < 0) {
		dev_err(&client->dev, "GPIO config failed!\n");
		goto err_config_gpio;
	}

	retval = do_init_reset(rmi4_data);
	if (retval)
		dev_warn(&client->dev, "Init reset failed! Soldiering on!\n");
	calib = &rmi4_data->board->calibs[rmi4_data->touch_type];
	/*
	 * Register physical driver - this will call the detect function that
	 * will then scan the device and determine the supported
	 * rmi4 functions.
	 */
	retval = rmi4_i2c_query_device(rmi4_data);
	if (retval) {
		dev_err(&client->dev, "rmi4 query device failed\n");
		goto err_query_dev;
	}

	/* Store the instance data in the i2c_client */
	i2c_set_clientdata(client, rmi4_data);

	/*initialize the input device parameters */
	rmi4_data->input_ts_dev->name	= DRIVER_NAME;
	rmi4_data->input_ts_dev->phys	= "Synaptics_Clearpad";
	rmi4_data->input_ts_dev->id.bustype = BUS_I2C;
	rmi4_data->input_ts_dev->dev.parent = &client->dev;
	input_set_drvdata(rmi4_data->input_ts_dev, rmi4_data);

	rmi4_data->input_key_dev->name	= calib->key_dev_name;
	rmi4_data->input_key_dev->phys	= "Synaptics_Clearpad";
	rmi4_data->input_key_dev->id.bustype = BUS_I2C;
	rmi4_data->input_key_dev->dev.parent = &client->dev;
	input_set_drvdata(rmi4_data->input_key_dev, rmi4_data);

	/* Initialize the function handlers for rmi4 */
	set_bit(EV_SYN, rmi4_data->input_ts_dev->evbit);
	set_bit(EV_ABS, rmi4_data->input_ts_dev->evbit);
	set_bit(EV_KEY, rmi4_data->input_key_dev->evbit);

	input_mt_init_slots(rmi4_data->input_ts_dev, MAX_FINGERS);
	input_set_abs_params(rmi4_data->input_ts_dev,
		ABS_MT_POSITION_X, 0, rmi4_data->sensor_max_x, 0, 0);
	input_set_abs_params(rmi4_data->input_ts_dev,
		ABS_MT_POSITION_Y, 0, rmi4_data->sensor_max_y, 0, 0);
	input_set_abs_params(rmi4_data->input_ts_dev,
			ABS_MT_TOUCH_MAJOR, 0, MAX_TOUCH_MAJOR, 0, 0);
	input_set_abs_params(rmi4_data->input_ts_dev,
			ABS_MT_TOUCH_MINOR, 0, MAX_TOUCH_MINOR, 0, 0);

	/* Clear interrupts */
	retval = rmi4_i2c_block_read(rmi4_data,
			rmi4_data->fn01_data_base_addr + 1,
			intr_status, rmi4_data->number_of_interrupt_register);
	if (retval < 0) {
		dev_err(&client->dev, "Clear interrupt failed\n");
		goto err_clear_irq;
	}
	retval = request_threaded_irq(rmi4_data->irq, NULL,
					rmi4_irq_thread,
					platformdata->irq_type,
					DRIVER_NAME, rmi4_data);
	if (retval < 0) {
		dev_err(&client->dev,
			"Unable to get attn irq %d\n", rmi4_data->irq);
		goto err_req_irq;
	}

	retval = input_register_device(rmi4_data->input_ts_dev);
	if (retval) {
		dev_err(&client->dev, "ts input register failed\n");
		goto err_reg_input_ts;
	}

	retval = input_register_device(rmi4_data->input_key_dev);
	if (retval) {
		dev_err(&client->dev, "key input register failed\n");
		goto err_reg_input_key;
	}

#ifdef DEBUG
	retval = sysfs_create_group(&client->dev.kobj, &rmi4_attr_dbg);
	if (retval < 0) {
		dev_err(&client->dev, "rmi4 sysfs register failed\n");
		goto err_reg_input;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	rmi4_data->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	rmi4_data->es.suspend = rmi4_early_suspend;
	rmi4_data->es.resume = rmi4_late_resume;
	register_early_suspend(&rmi4_data->es);
#endif
	return retval;

#ifdef DEBUG
err_reg_input:
	input_unregister_device(rmi4_data->input_key_dev);
	rmi4_data->input_key_dev = NULL;
#endif
err_reg_input_key:
	input_unregister_device(rmi4_data->input_ts_dev);
	rmi4_data->input_ts_dev = NULL;
err_reg_input_ts:
	free_irq(rmi4_data->irq, rmi4_data);
err_req_irq:
err_clear_irq:
err_query_dev:
	gpio_free(rmi4_data->board->int_gpio_number);
	gpio_free(rmi4_data->board->rst_gpio_number);
	rmi4_free_funcs(rmi4_data);
err_config_gpio:
	if (rmi4_data->regulator) {
		regulator_disable(rmi4_data->regulator);
		regulator_put(rmi4_data->regulator);
	}
err_regulator:
	if (rmi4_data->input_key_dev)
		input_free_device(rmi4_data->input_key_dev);
err_input_key:
	if (rmi4_data->input_ts_dev)
		input_free_device(rmi4_data->input_ts_dev);
err_input_ts:
	kfree(rmi4_data);

	return retval;
}

/**
 * rmi4_remove() - Removes the i2c-client touchscreen driver
 * @client: i2c client structure pointer
 *
 * This function uses to remove the i2c-client
 * touchscreen driver and returns integer.
 */
static int __devexit rmi4_remove(struct i2c_client *client)
{
	struct rmi4_data *rmi4_data = i2c_get_clientdata(client);
	const struct rmi4_platform_data *pdata = rmi4_data->board;

	free_irq(rmi4_data->irq, rmi4_data);
	gpio_free(pdata->int_gpio_number);
	gpio_free(pdata->rst_gpio_number);
#ifdef DEBUG
	sysfs_remove_group(&client->dev.kobj, &rmi4_attr_dbg);
#endif
	input_unregister_device(rmi4_data->input_ts_dev);
	input_unregister_device(rmi4_data->input_key_dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&rmi4_data->es);
#endif
	if (rmi4_data->regulator) {
		regulator_disable(rmi4_data->regulator);
		regulator_put(rmi4_data->regulator);
	}
	rmi4_free_funcs(rmi4_data);
	kfree(rmi4_data);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void rmi4_early_suspend(struct early_suspend *h)
{
	int retval;
	struct rmi4_data *pdata  = container_of(h, struct rmi4_data, es);
	struct i2c_client *client = pdata->i2c_client;

	dev_info(&client->dev, "Enter %s\n", __func__);
	disable_irq(pdata->irq);
	retval = rmi4_i2c_set_bits(pdata,
			pdata->fn01_ctrl_base_addr, F01_CTRL0_SLEEP);
	if (retval < 0)
		dev_err(&client->dev, "set F01_CTRL0_SLEEP failed\n");

	if (pdata->regulator)
		regulator_disable(pdata->regulator);
}

void rmi4_late_resume(struct early_suspend *h)
{
	int retval;
	struct rmi4_data *pdata  = container_of(h, struct rmi4_data, es);
	struct i2c_client *client = pdata->i2c_client;
	u8 intr_status[4];

	dev_info(&client->dev, "Enter %s\n", __func__);
	if (pdata->regulator) {
		/*need wait to stable if regulator first output*/
		int needwait = !regulator_is_enabled(pdata->regulator);
		regulator_enable(pdata->regulator);
		if (needwait)
			msleep(50);
	}
	enable_irq(pdata->irq);
	retval = rmi4_i2c_clear_bits(pdata,
			pdata->fn01_ctrl_base_addr, F01_CTRL0_SLEEP);
	if (retval < 0)
		dev_err(&client->dev, "clear F01_CTRL0_SLEEP failed\n");

	/* Clear interrupts */
	rmi4_i2c_block_read(pdata,
			pdata->fn01_data_base_addr + 1,
			intr_status, pdata->number_of_interrupt_register);
}
#endif

static const struct i2c_device_id rmi4_id_table[] = {
	{ "synaptics_3202", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, rmi4_id_table);

static struct i2c_driver rmi4_driver = {
	.driver = {
		.name	=	DRIVER_NAME,
		.owner	=	THIS_MODULE,
	},
	.probe		=	rmi4_probe,
	.remove		=	__devexit_p(rmi4_remove),
	.id_table	=	rmi4_id_table,
};

static int __init rmi4_init(void)
{
	return i2c_add_driver(&rmi4_driver);
}

static void __exit rmi4_exit(void)
{
	i2c_del_driver(&rmi4_driver);
}

module_init(rmi4_init);
module_exit(rmi4_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("naveen.gaddipati@stericsson.com, js.ha@stericsson.com");
MODULE_DESCRIPTION("synaptics rmi4 i2c touch Driver");
MODULE_ALIAS("i2c:synaptics_rmi4_ts");
