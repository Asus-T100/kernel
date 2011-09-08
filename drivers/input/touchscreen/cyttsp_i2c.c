/* Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) I2C touchscreen driver.
 * For use with Cypress Txx2xx and Txx3xx parts with I2C interface.
 * Supported parts include:
 * CY8CTST241
 * CY8CTMG240
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009, 2010 Cypress Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com (kev@cypress.com)
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/cyttsp.h>
#include "cyttsp_core.h"

struct cyttsp_i2c {
	struct cyttsp_bus_ops ops;
	struct i2c_client *client;
};

static s32 ttsp_i2c_read_block_data(void *priv, u8 addr,
				    u8 length, void *values)
{
	struct cyttsp_i2c *ts = priv;
	return i2c_smbus_read_i2c_block_data(ts->client, addr, length, values);
}

static s32 ttsp_i2c_write_block_data(void *priv, u8 addr,
				     u8 length, const void *values)
{
	struct cyttsp_i2c *ts = priv;
	return i2c_smbus_write_i2c_block_data(ts->client, addr, length, values);
}

static s32 ttsp_i2c_tch_ext(void *priv, void *values)
{
	int retval = 0;
	struct cyttsp_i2c *ts = priv;

	dev_dbg(&ts->client->dev, "%s: enter\n", __func__);

	/* Add custom touch extension handling code here */
	/* set: retval < 0 for any returned system errors,
	   retval = 0 if normal touch handling is required,
	   retval > 0 if normal touch handling is *not* required */
	if (!ts || !values)
		retval = -EIO;

	return retval;
}
static int __devinit cyttsp_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct cyttsp_i2c *ts;
	int retval;

	dev_dbg(&client->dev, "%s: enter\n", __func__);

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	/* allocate and clear memory */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		printk(KERN_ERR "%s: Error, kzalloc.\n", __func__);
		retval = -ENOMEM;
		goto error_alloc_data_failed;
	}

	/* register driver_data */
	ts->client = client;
	ts->ops.write = ttsp_i2c_write_block_data;
	ts->ops.read = ttsp_i2c_read_block_data;
	ts->ops.ext = ttsp_i2c_tch_ext;

	retval = cyttsp_core_init((void *)ts, &ts->ops, &client->dev);
	if (retval)
		goto ttsp_core_err;

	dev_info(&client->dev, "successful registration\n");
	return 0;

ttsp_core_err:
	kfree(ts);
error_alloc_data_failed:
	return retval;
}


/* registered in driver struct */
static int __devexit cyttsp_i2c_remove(struct i2c_client *client)
{
	struct cyttsp_i2c *ts = cyttsp_get_bus_priv(&client->dev);

	dev_dbg(&client->dev, "cyttsp_i2c_remove\n");

	cyttsp_core_release(dev_get_drvdata(&client->dev));
	kfree(ts);
	return 0;
}

#ifdef CONFIG_PM
static int cyttsp_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	return cyttsp_suspend(dev_get_drvdata(&client->dev));
}

static int cyttsp_i2c_resume(struct i2c_client *client)
{
	return cyttsp_resume(dev_get_drvdata(&client->dev));
}
#endif

static const struct i2c_device_id cyttsp_i2c_id[] = {
	{ "cy8ctma340", 0 },  { }
};

static struct i2c_driver cyttsp_i2c_driver = {
	.driver = {
		.name = CY_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_i2c_probe,
	.remove = __devexit_p(cyttsp_i2c_remove),
	.id_table = cyttsp_i2c_id,
#ifdef CONFIG_PM
	.suspend = cyttsp_i2c_suspend,
	.resume = cyttsp_i2c_resume,
#endif
};

static int cyttsp_i2c_init(void)
{
	return i2c_add_driver(&cyttsp_i2c_driver);
}

static void cyttsp_i2c_exit(void)
{
	return i2c_del_driver(&cyttsp_i2c_driver);
}

module_init(cyttsp_i2c_init);
module_exit(cyttsp_i2c_exit);

MODULE_ALIAS("i2c:cyttsp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cyttsp_i2c_id);
