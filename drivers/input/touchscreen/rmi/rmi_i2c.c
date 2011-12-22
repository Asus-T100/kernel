/**
 *
 * Synaptics Register Mapped Interface (RMI4) I2C Physical Layer Driver.
 * Copyright (c) 2007-2011, Synaptics Incorporated
 *
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

/* #define DEBUG 1 */
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/rmi_i2c.h>
#include "rmi_drvr.h"
#include "rmi_sensor.h"

#define DRIVER_NAME "rmi4_ts"
#define DEVICE_NAME "rmi4_ts"

#define COMM_DEBUG  0		/* Set to 1 to dump transfers. */
#define PAGE_SELECT_REGISTER	0xFF

static const struct i2c_device_id rmi_i2c_id_table[] = {
	{"synaptics_3202", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, rmi_i2c_id_table);

/*
 * This is the data kept on a per instance (client) basis.  This data is
 * always accessible by using the container_of() macro of the various elements
 * inside.
 */
struct instance_data {
	int instance_no;
	int irq;
	int attn_polarity;
	int attn_gpio;
	int enabled;
	struct rmi_phys_driver rmiphysdrvr;
	struct i2c_client *i2cclient;	/* pointer to client for later use in
					   read, write, read_multiple, etc. */
	struct mutex page_mutex;
	struct lock_class_key page_key;
	int page;
};

static irqreturn_t i2c_attn_isr(int irq, void *info);

/*
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing.  So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.  This function sets the page.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * param[in] id - The pointer to the instance_data struct
 * param[in] page - The new page address.
 * returns zero on success, non-zero on failure.
 */
/* Writing to page select is giving errors in some configurations.  It's
 * not needed for basic operation [see note], so we've turned it off for the
 * moment. Once we figure out why this happening (is it a bug in our code? or
 * in some I2C chips?  or maybe in the driver for some chips?) we'll either
 * turn this opperation back on, move the choice to platform data, or
 * determine whether to use it based on I2C driver capability).
 *
 * [NOTE: The current driver feature set doesn't require us to access
 * addresses outside of the first page, so we're OK for the time being.
 * Obviously this must be remedied before implementing the more advanced
 * features that are in the pipeline.]
 */
#if	defined(USE_PAGESELECT)
int rmi_set_page(struct instance_data *instance_data, unsigned int page)
{
	char txbuf[2] = {PAGE_SELECT_REGISTER, page};
	int retval;

#if	COMM_DEBUG
	dev_info(&instance_data->i2cclient->dev,
		 "%s: Set page to 0x%02X.", __func__, page);
#endif

	retval = i2c_master_send(instance_data->i2cclient,
				txbuf, ARRAY_SIZE(txbuf));
	if (retval != ARRAY_SIZE(txbuf)) {
		dev_err(&instance_data->i2cclient->dev,
			"%s: Set page failed: %d.", __func__, retval);
	} else {
		retval = 0;
		instance_data->page = page;
	}
	return retval;
}
#else
int rmi_set_page(struct instance_data *instance_data, unsigned int page)
{
	return 0;
}
#endif

/*
 * Same as rmi_i2c_read, except that multiple bytes are allowed to be read.
 *
 * param[in] pd - The pointer to the rmi_phys_driver struct
 * param[in] address - The address at which to start the data read.
 * param[out] valp - Pointer to the buffer where the data will be stored.  This
 *     buffer must be at least size bytes long.
 * param[in] size - The number of bytes to be read.
 * returns zero upon success (with the byte read in valp), non-zero upon error.
 *
 */
static int rmi_i2c_read_multiple(struct rmi_phys_driver *physdrvr,
				unsigned short address,
				char *valp, int size)
{
	struct instance_data *instance_data =
	    container_of(physdrvr, struct instance_data, rmiphysdrvr);
	char txbuf[1] = {address & 0xff};
	int retval = 0;
	int retry_count = 0;

#if	COMM_DEBUG
	dev_info(&instance_data->i2cclient->dev, "%s: Read %d bytes at 0x%04x",
		 __func__, size, address);
#endif

	/* Can't have anyone else changing the page behind our backs */
	mutex_lock(&instance_data->page_mutex);

	if (((address >> 8) & 0xff) != instance_data->page) {
		/* Switch pages */
		retval = rmi_set_page(instance_data, ((address >> 8) & 0xff));
		if (retval)
			goto exit;
	}

retry:
	physdrvr->tx_count++;
	physdrvr->tx_bytes += ARRAY_SIZE(txbuf);
	retval = i2c_master_send(instance_data->i2cclient,
				 txbuf, ARRAY_SIZE(txbuf));
	if (retval != 1) {
		dev_err(&instance_data->i2cclient->dev, "%s: Write fail: %d\n",
			__func__, retval);
		physdrvr->tx_errors++;
		goto exit;
	}

	physdrvr->rx_count++;
	physdrvr->rx_bytes += size;
	retval = i2c_master_recv(instance_data->i2cclient, valp, size);

	if (retval != size) {
		physdrvr->rx_errors++;
		if (++retry_count == 5) {
			dev_err(&instance_data->i2cclient->dev,
				"%s: Read of 0x%04x size %d fail: %d\n",
				__func__, address, size, retval);
		} else {
			mdelay(10);
			rmi_set_page(instance_data, ((address >> 8) & 0xff));
			goto retry;
		}
	} else {
		retval = 0;
	}

exit:
	mutex_unlock(&instance_data->page_mutex);
	return retval;
}

/*
 * Read a single register through i2c.
 *
 * param[in] pd - The pointer to the rmi_phys_driver struct
 * param[in] address - The address at which to start the data read.
 * param[out] valp - Pointer to the buffer where the data will be stored.
 * returns zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_i2c_read(struct rmi_phys_driver *physdrvr, unsigned short address,
	     char *valp)
{
	return rmi_i2c_read_multiple(physdrvr, address, valp, 1);
}

/*
 * Write multiple registers.
 *
 * param[in] pd - The pointer to the rmi_phys_driver struct
 * param[in] address - The address at which to start the write.
 * param[in] valp - A pointer to a buffer containing the data to be written.
 * param[in] size - The number of bytes to write.
 * returns one upon success, something else upon error.
 */
static int
rmi_i2c_write_multiple(struct rmi_phys_driver *physdrvr, unsigned short address,
		       char *valp, int size)
{
	struct instance_data *instance_data =
	    container_of(physdrvr, struct instance_data, rmiphysdrvr);

	unsigned char txbuf[size+1];
	int retval = 0;

	memcpy(txbuf+1, valp, size);

	/* Can't have anyone else changing the page behind our backs */
	mutex_lock(&instance_data->page_mutex);

	if (((address >> 8) & 0xff) != instance_data->page) {
		/* Switch pages */
		retval = rmi_set_page(instance_data, ((address >> 8) & 0xff));
		if (retval) {
			/* error occurs we change return value to -1 */
			retval = -1;
			goto exit;
		}
	}

	txbuf[0] = address & 0xff;	/* put the address in the first byte */
	retval = i2c_master_send(instance_data->i2cclient,
				txbuf, ARRAY_SIZE(txbuf));
	physdrvr->tx_count++;
	physdrvr->tx_bytes += ARRAY_SIZE(txbuf);

	/* TODO: Add in retry on writes only in certain error return values */
	if (retval != ARRAY_SIZE(txbuf)) {
		dev_err(&instance_data->i2cclient->dev, "%s: Write fail: %d\n",
			__func__, retval);
		physdrvr->tx_errors++;
		/* error occurs we change return value to -2 */
		retval = -2;
		goto exit;
	} else
		retval = 1; /* return one if succeeds */

exit:
	mutex_unlock(&instance_data->page_mutex);
	return retval;
}

/*
 * Write a single register through i2c.
 *
 * param[in] pd - The pointer to the rmi_phys_driver structnew file (copy)
 * param[in] address - The address at which to start the write.
 * param[in] data - The data to be written.
 * returns one upon success, something else upon error.
 */
static int
rmi_i2c_write(struct rmi_phys_driver *physdrvr, unsigned short address,
	      char data)
{
	return rmi_i2c_write_multiple(physdrvr, address, &data, 1);
}

/*
 * This is the Interrupt Service Routine.  It just notifies the application
 * layer that attention is required.
 */
static irqreturn_t i2c_attn_isr(int irq, void *info)
{
	struct instance_data *instance_data = info;

	disable_irq_nosync(instance_data->irq);
	instance_data->rmiphysdrvr.attn_count++;

	if (instance_data->rmiphysdrvr.attention &&
			(gpio_get_value(instance_data->attn_gpio) ==
			instance_data->attn_polarity)) {
		instance_data->rmiphysdrvr.attention(
			&instance_data->rmiphysdrvr);
	} else {
		enable_irq(instance_data->irq);
	}

	return IRQ_HANDLED;
}


static int
acquire_attn_irq(struct instance_data *instance_data)
{
	int retval;

	unsigned long irq_type = IRQ_TYPE_EDGE_FALLING;
	retval = request_irq(instance_data->irq, i2c_attn_isr,
			irq_type, "rmi_i2c", instance_data);
	if (retval)
		return retval;

	dev_dbg(&instance_data->rmiphysdrvr.sensor->sensor_device->dev,
		"got ATTN irq.\n");

	/*
	 * For some reason if we setup as level trigger, and execute
	 * 'instance_data->rmiphysdrvr.atten',
	 * there will be no more IRQ triggered.
	 * On the contrary, if we setup as edge trigger, we have to execute
	 * 'instance_data->rmiphysdrvr.atten'.
	 * Or, we won't receive any IRQ.
	 *
	 */
	if ((irq_type & IRQ_TYPE_EDGE_BOTH) != 0)
		if (instance_data->attn_gpio &&
			gpio_get_value(instance_data->attn_gpio) ==
			instance_data->attn_polarity &&
			instance_data->rmiphysdrvr.attention) {

			disable_irq(instance_data->irq);
			instance_data->rmiphysdrvr.attention(
					&instance_data->rmiphysdrvr);
		}

	return retval;
}

/* Specify the routine that will be called when attention is asserted.
 */
static void set_attn_handler (struct rmi_phys_driver *physdrvr,
	void (*attention) (struct rmi_phys_driver *physdrvr))
{
	struct instance_data *instance_data =
		container_of(physdrvr, struct instance_data, rmiphysdrvr);

	physdrvr->attention = attention;
	if (instance_data->attn_gpio &&
			gpio_get_value(instance_data->attn_gpio) ==
			instance_data->attn_polarity &&
			instance_data->rmiphysdrvr.attention) {
		disable_irq(instance_data->irq);
		instance_data->rmiphysdrvr.attention(
			&instance_data->rmiphysdrvr);
	}
}


static void release_attn_irq(struct rmi_phys_driver *physdrvr)
{
	struct instance_data *instance_data =
	container_of(physdrvr, struct instance_data, rmiphysdrvr);

	dev_info(&physdrvr->sensor->sensor_device->dev,
		 "Releasing ATTN irq.\n");
	disable_irq(instance_data->irq);
	free_irq(instance_data->irq, instance_data);
}


static int
enable_device(struct rmi_phys_driver *physdrvr)
{
	int retval = 0;
	struct instance_data *instance_data =
		container_of(physdrvr, struct instance_data, rmiphysdrvr);

	if (instance_data->enabled)
		return 0;

	retval = acquire_attn_irq(instance_data);
	if (retval)
		goto error_exit;
	instance_data->enabled = true;
	dev_dbg(&physdrvr->sensor->sensor_device->dev,
		"Physical device enabled.\n");
	return 0;

error_exit:
	dev_err(&physdrvr->sensor->sensor_device->dev,
		"Failed to enable physical device.  Code=%d.\n", retval);
	return retval;
}


static void
disable_device(struct rmi_phys_driver *physdrvr)
{
	struct instance_data *instance_data =
	container_of(physdrvr, struct instance_data, rmiphysdrvr);

	if (!instance_data->enabled)
		return;

	release_attn_irq(physdrvr);
	dev_dbg(&physdrvr->sensor->sensor_device->dev,
		"Physical device disabled.\n");
	instance_data->enabled = false;
}

/* The Driver probe function - will allocate and initialize the instance
 * data and request the irq and set the instance data as the clients
 * platform data then register the physical driver which will do a scan of
 * the RMI4 Physical Device Table and enumerate any RMI4 functions that
 * have data sources associated with them.
 */
static int
rmi_i2c_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
	struct instance_data *instance_data;
	int retval = 0;
	struct rmi_i2c_platformdata *platformdata;
	struct rmi_sensordata *sensordata;

	if (client == NULL) {
		pr_err("%s: Invalid NULL client received.", __func__);
		return -EINVAL;
	}

	platformdata = client->dev.platform_data;
	if (platformdata == NULL) {
		dev_err(&client->dev, "%s: CONFIGURATION ERROR - "
		"platform data is NULL.\n", __func__);
		retval = -EINVAL;
	}
	sensordata = platformdata->sensordata;

	dev_dbg(&client->dev, "%s: Probing i2c RMI device, addr: 0x%02x",
		  __func__, client->addr);

	/* Allocate and initialize the instance data for this client */
	instance_data = kzalloc(sizeof(*instance_data), GFP_KERNEL);
	if (!instance_data) {
		dev_err(&client->dev,
			"%s: Failed to allocate instance_data.\n",
			__func__);
		return -ENOMEM;
	}

	__mutex_init(&instance_data->page_mutex, "page_mutex",
		     &instance_data->page_key);
	instance_data->rmiphysdrvr.name = RMI4_I2C_DRIVER_NAME;
	instance_data->rmiphysdrvr.write = rmi_i2c_write;
	instance_data->rmiphysdrvr.read = rmi_i2c_read;
	instance_data->rmiphysdrvr.write_multiple = rmi_i2c_write_multiple;
	instance_data->rmiphysdrvr.read_multiple = rmi_i2c_read_multiple;
	instance_data->rmiphysdrvr.set_attn_handler = set_attn_handler;
	instance_data->rmiphysdrvr.enable_device = enable_device;
	instance_data->rmiphysdrvr.disable_device = disable_device;
	instance_data->rmiphysdrvr.module = THIS_MODULE;
	/* Set default to polling in case no matching platform data is located
	   for this device. We'll still work but in polling mode since we didn't
	   find any irq info */
	instance_data->rmiphysdrvr.polling_required = true;
	instance_data->rmiphysdrvr.proto_name = "i2c";

	instance_data->page = 0xffff;	/* Force a set page the first time */
	instance_data->enabled = true;	/* We plan to come up enabled. */

	/* Egregiously horrible delay here that seems to prevent I2C disasters
	 * on certain broken dev systems.  In most cases, you can safely
	 * leave this as zero.
	 */
	dev_dbg(&client->dev, "%s: sensor addr: 0x%02x irq: %d\n",
		 __func__, platformdata->i2c_address,
		sensordata->attn_gpio_number ?
			gpio_to_irq(sensordata->attn_gpio_number) : -1);
	if (client->addr != platformdata->i2c_address) {
		dev_err(&client->dev,
			"%s: CONFIGURATION ERROR - client I2C address 0x%02x "
			"doesn't match platform data address 0x%02x.\n",
			__func__, client->addr, platformdata->i2c_address);
		retval = -EINVAL;
		goto error_exit;
	}

	instance_data->instance_no = rmi_next_sensor_id();

	/* set the device name using the instance_no appended to DEVICE_NAME
	 * to make a unique name */
	dev_set_name(&client->dev, "%s%d", RMI4_I2C_DEVICE_NAME,
		     instance_data->instance_no);

	retval = gpio_request(sensordata->attn_gpio_number, "rmi");
	if (retval < 0) {
		dev_err(&client->dev, "%s: Unable to request GPIO\n", __func__);
		goto error_exit;
	}
	retval = gpio_direction_input(sensordata->attn_gpio_number);
	if (retval < 0) {
		dev_err(&client->dev, "%s: Unable to set GPIO direction\n",
			__func__);
		goto error_exit;
	}

	retval = gpio_request(sensordata->rst_gpio_number, "rmi");
	if (retval < 0) {
		dev_err(&client->dev, "%s: Unable to request RESET GPIO\n",
			__func__);
		goto error_exit;
	}
	retval = gpio_direction_output(sensordata->rst_gpio_number, 1);
	if (retval < 0) {
		dev_err(&client->dev, "%s: Unable to set RESET GPIO "
			"direction\n", __func__);
		goto error_exit;
	}
	gpio_set_value(sensordata->rst_gpio_number, 1);

	if (platformdata->delay_ms > 0)
		mdelay(platformdata->delay_ms);


	/* Determine if we need to poll (inefficient) or use interrupts.
	 */
	if (sensordata->attn_gpio_number) {
		instance_data->irq = gpio_to_irq(sensordata->attn_gpio_number);
		instance_data->attn_polarity = sensordata->attn_polarity;
		instance_data->attn_gpio = sensordata->attn_gpio_number;
		instance_data->rmiphysdrvr.polling_required = false;
		instance_data->rmiphysdrvr.irq = instance_data->irq;

	} else {
		instance_data->rmiphysdrvr.polling_required = true;
		dev_info(&client->dev,
			 "%s: No IRQ info given. Polling required.\n",
			 __func__);
	}

	/* Store the instance data in the i2c_client - we need to do this prior
	 * to calling register_physical_driver since it may use the read, write
	 * functions. If nothing was found then the id fields will be set to 0
	 * for the irq and the default  will be set to polling required so we
	 * will still work but in polling mode. */
	i2c_set_clientdata(client, instance_data);

	/* Copy i2c_client pointer into instance_data's i2c_client pointer for
	   later use in rmi4_read, rmi4_write, etc. */
	instance_data->i2cclient = client;

	/* Call the platform setup routine, to do any setup that is required
	 * before interacting with the device.  When we refined the bus
	 * architecture, this will be done elsewhere.
	 */
	if (sensordata && sensordata->rmi_sensor_setup) {
		retval = sensordata->rmi_sensor_setup();
		if (retval) {
			dev_err(&client->dev,
				"%s: sensor setup failed with code %d.",
			       __func__, retval);
			goto error_exit;
		}
	}

	/* Register sensor drivers - this will call the detect function that
	 * will then scan the device and determine the supported RMI4 sensors
	 * and functions.
	 */
	retval = rmi_register_sensor(&instance_data->rmiphysdrvr,
				platformdata->sensordata);
	if (retval) {
		dev_err(&client->dev,
			"%s: Failed to register %s sensor drivers\n", __func__,
			instance_data->rmiphysdrvr.name);
		goto error_exit;
	}

	if (instance_data->rmiphysdrvr.polling_required == false) {
		retval = acquire_attn_irq(instance_data);
		if (retval) {
			dev_warn(&client->dev,
				"Failed to obtain IRQ %d. Result: %d.",
				instance_data->irq, retval);
			dev_info(&client->dev, "%s: Reverting to polling.\n",
				 __func__);
			instance_data->rmiphysdrvr.polling_required = true;
			/* TODO: Need to revert back to polling - create and
			 * start timer. */
			retval = 0;
		}

		/* export GPIO for attention handling */

#if defined(CONFIG_SYNA_RMI_DEV)
		retval = gpio_export(instance_data->attn_gpio, false);
		if (retval) {
			dev_warn(&client->dev, "%s: WARNING: Failed to "
				"export ATTN gpio!.", __func__);
			retval = 0;
		} else {
			retval = gpio_export_link(
				&instance_data->rmiphysdrvr.sensor->
				sensor_device->dev, "attn",
				instance_data->attn_gpio);
			if (retval) {
				dev_warn(
					&instance_data->rmiphysdrvr.sensor->
					sensor_device->dev, "%s: WARNING: "
					"Failed to symlink ATTN gpio!.",
					__func__);
				retval = 0;
			} else {
				dev_info(&instance_data->
					rmiphysdrvr.sensor->sensor_device->dev,
					 "%s: Exported GPIO %d.",
					__func__, instance_data->attn_gpio);
			}
		}
#endif /* CONFIG_SYNA_RMI_DEV */
	}

	dev_dbg(&client->dev, "%s: Successfully registered %s sensor driver.\n",
		__func__, instance_data->rmiphysdrvr.name);

	return retval;

error_exit:
	kfree(instance_data);
	/* return error for clean-up*/
	return retval;
}

static int rmi_i2c_remove(struct i2c_client *client)
{
	struct instance_data *instance_data = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s: Unregistering phys driver %s\n", __func__,
		instance_data->rmiphysdrvr.name);

	rmi_unregister_sensors(&instance_data->rmiphysdrvr);

	dev_dbg(&client->dev, "%s: Unregistered phys driver %s\n",
		__func__, instance_data->rmiphysdrvr.name);

	/* only free irq if we have an irq - otherwise the instance_data
	   will be 0 for that field */
	if (instance_data->irq)
		free_irq(instance_data->irq, instance_data);

	kfree(instance_data);
	dev_dbg(&client->dev, "%s: Remove successful\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int rmi_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	/* Touch sleep mode */
	return 0;
}

static int rmi_i2c_resume(struct i2c_client *client)
{
	/* Re-initialize upon resume */
	return 0;
}
#else
#define rmi_i2c_suspend	NULL
#define rmi_i2c_resume	NULL
#endif

/*
 * This structure tells the i2c subsystem about us.
 *
 * TODO: we should add .suspend and .resume fns.
 *
 */
static struct i2c_driver rmi_i2c_driver = {
	.probe = rmi_i2c_probe,
	.remove = rmi_i2c_remove,
	.suspend = rmi_i2c_suspend,
	.resume = rmi_i2c_resume,
	.driver = {
		.name = RMI4_I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.id_table = rmi_i2c_id_table,
};

static int __init rmi_phys_i2c_init(void)
{
	return i2c_add_driver(&rmi_i2c_driver);
}

static void __exit rmi_phys_i2c_exit(void)
{
	i2c_del_driver(&rmi_i2c_driver);
}

module_init(rmi_phys_i2c_init);
module_exit(rmi_phys_i2c_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("RMI4 Driver I2C Physical Layer");
MODULE_LICENSE("GPL");
