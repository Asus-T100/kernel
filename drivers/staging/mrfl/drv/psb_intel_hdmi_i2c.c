/*
 * Copyright © 2006-2007 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *
 */
/* chunfeng.zhao@intel.com
  */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#if 0				/* Not needed for MDFLD & MRST HDMI */
#include "psb_intel_hdmi_i2c.h"

#define MDFLD_HDMI_I2C_ADAPTER_ID 3

static int hdmi_i2c_open(struct i2c_client *c, void *data)
{
	/*Do nothing for now, may need to add sync code? */
	return 0;
}

static int hdmi_i2c_close(struct i2c_client *c, void *data)
{
	/*Do nothing for now, may need to add sync code? */
	return 0;
}

static char hdmi_i2c_read_byte_data(struct i2c_client *c, unsigned char adr)
{
	return i2c_smbus_read_byte_data(c, adr);
}

static int hdmi_i2c_write_byte_data(struct i2c_client *c, unsigned char adr,
				    unsigned char data)
{
	return i2c_smbus_write_byte_data(c, adr, data);
}

static int hdmi_i2c_read_data(struct i2c_adapter *adapter, unsigned char adr,
			      unsigned char *data, int size)
{
	struct i2c_msg msg = {
		.addr = adr, .flags = I2C_M_RD, .buf = data, .len = size
	};
	return i2c_transfer(adapter, &msg, 1);
}

static int hdmi_i2c_write_data(struct i2c_adapter *adapter, unsigned char adr,
			       unsigned char *data, int size)
{
	struct i2c_msg msg = {
		.addr = adr, .flags = 0, .buf = data, .len = size
	};
	return i2c_transfer(adapter, &msg, 1);
}

static struct i2c_adapter *hdmi_i2c_get_adapter(struct i2c_client *c)
{
	/*
	 * For HDMI if not plugged in, then i2c core may not
	 * create the client driver Should use the adapter directly
	*/
	if (c)
		return c->adapter;
	else
		return i2c_get_adapter(MDFLD_HDMI_I2C_ADAPTER_ID);
}

static struct mdfld_hdmi_i2c hdmi_i2c_bus = {
	.open = hdmi_i2c_open,
	.close = hdmi_i2c_close,
	.read_byte_data = hdmi_i2c_read_byte_data,
	.write_byte_data = hdmi_i2c_write_byte_data,
	.read_data = hdmi_i2c_read_data,
	.write_data = hdmi_i2c_write_data,
	.get_adapter = hdmi_i2c_get_adapter,
};

struct mdfld_hdmi_i2c *hdmi_i2c_init()
{
	return &hdmi_i2c_bus;
}

/*
 *  * i2c addresses to scan
 *  0x28 is from 0x50 >> 1 to remove first bit for ddc address
 *  0x39 is from 0x73 >> 1 for HDCP address
 *   */
static unsigned short normal_i2c[] = { 0x28, 0x39, I2C_CLIENT_END };

I2C_CLIENT_INSMOD;

/* Each client has this additional data */
struct mdfld_hdmi_i2c_data {
	struct semaphore data_lock;
	int data;
};

static const struct i2c_device_id mdfld_hdmi_id[] = {
	{"mdfld_hdmi", 0},
	{}
};

static int mdfld_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	return 0;
}

/* This function is called by i2c_detect */
static int mdfld_detect(struct i2c_client *client, int kind,
			struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	struct mdfld_hdmi_i2c_data *data = NULL;
	int err = 0;

	/* HDMI i2c is i2c3 with id = 3 */
	if (adapter->id != 3) {
		err = -ENODEV;
		goto error;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		err = -ENODEV;
		goto error;
	}

	data = kmalloc(sizeof(*data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto error;
	}

	memset(data, 0x00, sizeof(*data));

	i2c_set_clientdata(client, data);

	if (client->addr == 0xA0)
		hdmi_i2c_bus.ddc_client = client;
	else
		hdmi_i2c_bus.hdcp_client = client;

	return 0;

 error:
	if (data != NULL)
		kfree(data);
	return err;
}

static int mdfld_remove(struct i2c_client *client)
{
	struct mdfld_hdmi_i2c_data *data = i2c_get_clientdata(client);
	kfree(data);
	return 0;
}

/* This is the driver that will be inserted */
static struct i2c_driver mdfld_hdmi_i2c_driver = {
	.driver = {
		   .name = "mdfld_hdmi",
		   },
	.probe = mdfld_probe,
	.remove = mdfld_remove,
	.id_table = mdfld_hdmi_id,

	.class = I2C_CLASS_DDC,
	.detect = mdfld_detect,
	.address_data = &addr_data,
};

static int __init mdfld_i2c_init(void)
{
	hdmi_i2c_bus.ddc_client = NULL;
	hdmi_i2c_bus.hdcp_client = NULL;

	return i2c_add_driver(&mdfld_hdmi_i2c_driver);
}

static void __exit mdfld_i2c_exit(void)
{
	i2c_del_driver(&mdfld_hdmi_i2c_driver);
}

MODULE_AUTHOR("Chunfeng Zhao <chunfeng.zhao@intel.com>");
MODULE_DESCRIPTION("mdfld hdmi i2c client driver");
MODULE_LICENSE("GPL");

module_init(mdfld_i2c_init);
module_exit(mdfld_i2c_exit);
#endif
