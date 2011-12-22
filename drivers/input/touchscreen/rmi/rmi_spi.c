/**
 *
 * Synaptics Register Mapped Interface (RMI4) SPI Physical Layer Driver.
 * Copyright (C) 2008-2011, Synaptics Incorporated
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

#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/gpio.h>

#include "rmi_spi.h"
#include "rmi_drvr.h"
#include "rmi_sensor.h"

#define COMM_DEBUG  0		/* Set to 1 to dump transfers. */

/* For V1 protocol, the high bit in the address is set to indicate reads. */
#define SPI_V1_READ_FLAG 0x80

/* For V2 protocol, first byte of transmission indicates what operation is
 * to be performed.
 */
#define SPI_V2_UNIFIED_READ       0xC0
#define SPI_V2_WRITE              0x40
#define SPI_V2_PREPARE_SPLIT_READ 0xC8
#define SPI_V2_EXECUTE_SPLIT_READ 0xCA

/* Once the sensor has prepared a V2 split read, we always send the same
 * bytes to tell it to execute the read. For convenience, we keep a static
 * copy of those bytes around.
 */
static unsigned char execute_split_read[] = { SPI_V2_EXECUTE_SPLIT_READ, 0x00 };

/* This is the data kept on a per instance (client) basis.  This data is
 * always accessible by using the container_of() macro of the various elements
 * inside.
 */
struct spi_device_instance_data {
	int instance_no;
	int irq;
	int attn_polarity;
	int attn_gpio;
	unsigned int byte_delay_us;
	unsigned int block_delay_us;
	unsigned int split_read_byte_delay_us;
	unsigned int split_read_block_delay_us;
	unsigned int buffer_size;
	unsigned char spi_version;
	int v2_transaction_size;
	wait_queue_head_t attn_event;
	bool attn_seen;
	bool split_read_pending;
	struct rmi_phys_driver rpd;
	struct spi_device *spidev;
	struct rmi_spi_platformdata *platformdata;
};


static int spi_xfer(struct spi_device_instance_data *instance_data,
		    const u8 *txbuf, unsigned n_tx, u8 *rxbuf, unsigned n_rx)
{
	struct spi_device *spi = instance_data->spidev;
#if COMM_DEBUG
	int i;
#endif
	int status;
	struct spi_message message;
	struct spi_transfer *xfer_list;
	const int total_bytes = n_tx + n_rx;
	u8 local_buf[total_bytes];
	int xfers_in_message = 0;
	int xfer_index = 0;
	int block_delay = n_rx > 0 ? instance_data->block_delay_us : 0;
	int byte_delay = n_tx > 1 ? instance_data->byte_delay_us : 0;
	if (instance_data->split_read_pending) {
		block_delay =
		    n_rx > 0 ? instance_data->split_read_block_delay_us : 0;
		byte_delay =
		    n_tx > 1 ? instance_data->split_read_byte_delay_us : 0;
	}

	if (n_tx) {
		xfers_in_message += 1;
		instance_data->rpd.tx_count++;
		instance_data->rpd.tx_bytes += n_tx;
	}
	if (n_rx) {
		instance_data->rpd.rx_count++;
		instance_data->rpd.rx_bytes += n_rx;
		if (byte_delay)
			xfers_in_message += n_rx;
		else
			xfers_in_message += 1;
	}

	xfer_list = kcalloc(xfers_in_message,
			    sizeof(struct spi_transfer), GFP_KERNEL);
	if (!xfer_list)
		return -ENOMEM;

	spi_message_init(&message);

	if (n_tx) {
		memset(&xfer_list[0], 0, sizeof(struct spi_transfer));
		xfer_list[0].len = n_tx;
		xfer_list[0].delay_usecs = block_delay;
		spi_message_add_tail(&xfer_list[0], &message);
		memcpy(local_buf, txbuf, n_tx);
		xfer_list[0].tx_buf = local_buf;
		xfer_index++;
	}
	if (n_rx) {
		if (byte_delay) {
			int buffer_offset = n_tx;
			for (; xfer_index < xfers_in_message; xfer_index++) {
				memset(&xfer_list[xfer_index], 0,
				       sizeof(struct spi_transfer));
				xfer_list[xfer_index].len = 1;
				xfer_list[xfer_index].delay_usecs = byte_delay;
				xfer_list[xfer_index].rx_buf =
				    local_buf + buffer_offset;
				buffer_offset++;
				spi_message_add_tail(&xfer_list[xfer_index],
						     &message);
			}
		} else {
			memset(&xfer_list[xfer_index], 0,
			       sizeof(struct spi_transfer));
			xfer_list[xfer_index].len = n_rx;
			xfer_list[xfer_index].rx_buf = local_buf + n_tx;
			spi_message_add_tail(&xfer_list[xfer_index], &message);
			xfer_index++;
		}
	}
#if COMM_DEBUG
	pr_info("%s: SPI transmits %d bytes...", __func__, n_tx);
	for (i = 0; i < n_tx; i++)
		pr_info("    0x%02X", local_buf[i]);
#endif

	/* do the i/o */
	if (instance_data->platformdata->cs_assert) {
		status = instance_data->platformdata->cs_assert(
			instance_data->platformdata->cs_assert_data, true);
		if (!status) {
			pr_err("%s: Failed to assert CS.", __func__);
			/* nonzero means error */
			status = -1;
			goto error_exit;
		} else
			status = 0;
	}
	status = spi_sync(spi, &message);
	if (instance_data->platformdata->cs_assert) {
		status = instance_data->platformdata->cs_assert(
			instance_data->platformdata->cs_assert_data, false);
		if (!status) {
			pr_err("%s: Failed to deassert CS.", __func__);
			/* nonzero means error */
			status = -1;
			goto error_exit;
		} else
			status = 0;
	}
	if (status == 0) {
		memcpy(rxbuf, local_buf + n_tx, n_rx);
		status = message.status;
#if COMM_DEBUG
		if (n_rx) {
			pr_info("%s: SPI received %d bytes...", __func__, n_rx);
			for (i = 0; i < n_rx; i++)
				pr_info("    0x%02X", rxbuf[i]);
		}
#endif
	} else {
		if (n_tx)
			instance_data->rpd.tx_errors++;
		if (n_rx)
			instance_data->rpd.rx_errors++;
		pr_err("%s: spi_sync failed with error code %d.",
		       __func__, status);
	}

error_exit:
	kfree(xfer_list);
	return status;
}

/* Same as rmi_spi_read_v1, except that multiple bytes are allowed to be read.
 * \param[in] pd
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.  This
 * buffer must be at least size bytes long.
 * \param[in] size The number of bytes to be read.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_spi_read_multiple_v1(struct rmi_phys_driver *pd, unsigned short address,
			 char *valp, int size)
{
	struct spi_device_instance_data *instance_data =
	    container_of(pd, struct spi_device_instance_data, rpd);
	int retval;
	unsigned char txbuf[2];

	txbuf[1] = address;
	txbuf[0] = address >> 8;
	txbuf[0] |= SPI_V1_READ_FLAG;

	retval = spi_xfer(instance_data, txbuf, ARRAY_SIZE(txbuf), valp, size);

	return retval;
}

/*
 * Read a single register through SPI, V1 protocol.
 * \param[in] pd
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_spi_read_v1(struct rmi_phys_driver *pd, unsigned short address, char *valp)
{
	return rmi_spi_read_multiple_v1(pd, address, valp, 1);
}

/* Write multiple registers using version 1 of the RMI4 SPI interface.
 * \param[in] pd
 * \param[in] address The address at which to start the write.
 * \param[in] valp A pointer to a buffer containing the data to be written.
 * \param[in] size The number of bytes to write.
 * \return one upon success, something else upon error.
 */
static int
rmi_spi_write_multiple_v1(struct rmi_phys_driver *pd, unsigned short address,
			  char *valp, int size)
{
	struct spi_device_instance_data *id =
	    container_of(pd, struct spi_device_instance_data, rpd);
	int buffer_size = size + 2;
	unsigned char txbuf[buffer_size];
	int retval;
	int i;

	txbuf[1] = address;
	txbuf[0] = address >> 8;

	for (i = 0; i < size; i++)
		txbuf[i + 2] = valp[i];

	retval = spi_xfer(id, txbuf, buffer_size, NULL, 0);

	return retval ? 0 : 1;
}

/* Write a single register through SPI using version 1 of the interface.
 * You can write multiple registers at once, but I made the functions for that
 * seperate for performance reasons.  Writing multiple requires allocation and
 * freeing.
 * \param[in] pd
 * \param[in] address The address at which to start the write.
 * \param[in] data The data to be written.
 * \return one upon success, something else upon error.
 */
static int
rmi_spi_write_v1(struct rmi_phys_driver *pd, unsigned short address, char data)
{
	return rmi_spi_write_multiple_v1(pd, address, &data, 1);
}

/* Read multiple bytes using version 2 of the RMI4 SPI interface.
 *
 * \param[in] pd
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.  This
 * buffer must be at least size bytes long.
 * \param[in] size The number of bytes to be read.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_spi_read_multiple_v2(struct rmi_phys_driver *pd, unsigned short address,
			 char *valp, int size)
{
	struct spi_device_instance_data *instance_data =
	    container_of(pd, struct spi_device_instance_data, rpd);
	int retval;
	char header_buf[4];

	header_buf[0] = SPI_V2_UNIFIED_READ;
	header_buf[1] = (address >> 8) & 0x00FF;
	header_buf[2] = address & 0x00ff;
	header_buf[3] = size;

	retval = spi_xfer(instance_data, header_buf, ARRAY_SIZE(header_buf),
			  valp, size);

	return retval;
}

/* Read a single register (one byte) from the device, using version 2 of the
 * RMI4 SPI interface.
 * \param[in] pd
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_spi_read_v2(struct rmi_phys_driver *pd, unsigned short address, char *valp)
{
	return rmi_spi_read_multiple_v2(pd, address, valp, 1);
}

/* Read multiple bytes using version 2 of the RMI4 SPI interface.
 *
 * \param[in] pd
 * \param[in] address The address at which to start the data read.
 * \param[out] valp Pointer to the buffer where the data will be stored.  This
 * buffer must be at least size bytes long.
 * \param[in] size The number of bytes to be read.
 * \return zero upon success (with the byte read in valp), non-zero upon error.
 */
static int
rmi_spi_split_read_v2(struct rmi_phys_driver *pd, unsigned short address,
		      char *valp, int size)
{
	struct spi_device_instance_data *instance_data =
	    container_of(pd, struct spi_device_instance_data, rpd);
	int retval;
	char header_buf[4];
	int read_size = size + 1; /* Add a byte for dummy byte at start. */
	char read_buf[read_size];

	header_buf[0] = SPI_V2_PREPARE_SPLIT_READ;
	header_buf[1] = (address >> 8) & 0x00FF;
	header_buf[2] = address & 0x00ff;
	header_buf[3] = size;

	instance_data->attn_seen = false;
	instance_data->split_read_pending = true;

	retval = spi_xfer(instance_data, header_buf, ARRAY_SIZE(header_buf),
			  NULL, 0);
	if (retval) {
		instance_data->split_read_pending = false;
		return retval;
	}

	enable_irq(pd->irq);
	wait_event_interruptible((instance_data->attn_event),
				 (instance_data->attn_seen == true));

	retval = spi_xfer(instance_data,
			  execute_split_read, ARRAY_SIZE(execute_split_read),
			  read_buf, read_size);
	instance_data->split_read_pending = false;
	if (retval)
		return retval;
	if (read_buf[0] != size)
		return -EIO;
	memcpy(valp, &read_buf[1], size);

	return retval;
}

/* Write multiple registers using version 2 of the RMI4 SPI interface.
 *
 * \param[in] pd
 * \param[in] address The address at which to start the write.
 * \param[in] valp A pointer to a buffer containing the data to be written.
 * \param[in] size The number of bytes to write.
 * \return one upon success, something else upon error.
 */
static int
rmi_spi_write_multiple_v2(struct rmi_phys_driver *pd, unsigned short address,
			  char *valp, int size)
{
	struct spi_device_instance_data *id =
	    container_of(pd, struct spi_device_instance_data, rpd);
	unsigned char txbuf[size + 4];
	int retval;

	txbuf[0] = SPI_V2_WRITE;
	txbuf[1] = (address >> 8) & 0x00FF;
	txbuf[2] = address & 0x00FF;
	txbuf[3] = size;

	memcpy(&txbuf[4], valp, size);

	retval = spi_xfer(id, txbuf, size + 4, NULL, 0);

	return retval ? 0 : 1;
}

/* Write a single byte/register using version 2 of the RMI4 SPI interface.
 *
 * \param[in] pd
 * \param[in] address The address at which to start the write.
 * \param[in] data The data to be written.
 * \return one upon success, something else upon error.
 */
static int
rmi_spi_write_v2(struct rmi_phys_driver *pd, unsigned short address, char data)
{
	return rmi_spi_write_multiple_v2(pd, address, &data, 1);
}

/* This is the Interrupt Service Routine.  It just notifies the physical device
 * that attention is required.
 */
static irqreturn_t spi_attn_isr(int irq, void *info)
{
	struct spi_device_instance_data *instance_data = info;

	disable_irq_nosync(instance_data->irq);
	instance_data->rpd.attn_count++;

	if (instance_data->spi_version == 2 &&
	    instance_data->split_read_pending) {
		instance_data->attn_seen = true;
		wake_up(&instance_data->attn_event);
		return IRQ_HANDLED;
	}

	if (instance_data->rpd.attention)
		instance_data->rpd.attention(&instance_data->rpd);
	return IRQ_HANDLED;
}

/* Specify the routine that will be called when attention is asserted.
 */
static void set_attn_handler (struct rmi_phys_driver *physdrvr,
	void (*attention) (struct rmi_phys_driver *physdrvr))
{
	physdrvr->attention = attention;
}

static int __devinit rmi_spi_probe(struct spi_device *spi)
{
	struct spi_device_instance_data *instance_data;
	int retval;
	struct rmi_spi_platformdata *platformdata;
	struct rmi_sensordata *sensordata;
	char buf[6];
	unsigned long irq_type = IRQ_TYPE_LEVEL_LOW;

	dev_info(&spi->dev, "%s: Probing RMI4 SPI device", __func__);

	platformdata = spi->dev.platform_data;
	if (platformdata == NULL) {
		dev_err(&spi->dev,
			"%s: CONFIGURATION ERROR - platform data is NULL.",
		       __func__);
		return -EINVAL;
	}

	spi->bits_per_word = 8;
	/* This should have already been set up in the board file,
	 * shouldn't it? */
	spi->mode = SPI_MODE_3;

	retval = spi_setup(spi);
	if (retval < 0) {
		dev_err(&spi->dev,
			"%s: spi_setup failed with %d.", __func__, retval);
		return retval;
	}

	instance_data = kzalloc(sizeof(*instance_data), GFP_KERNEL);
	if (!instance_data) {
		dev_err(&spi->dev,
			"%s: Failed to allocate memory for instance data.",
		       __func__);
		kfree(platformdata);
		return -ENOMEM;
	}

	instance_data->platformdata = platformdata;
	sensordata = platformdata->sensordata;
	instance_data->block_delay_us =
	    platformdata->block_delay_us ? platformdata->
	    block_delay_us : RMI_DEFAULT_BLOCK_DELAY_US;
	instance_data->byte_delay_us =
	    platformdata->byte_delay_us ? platformdata->
	    byte_delay_us : RMI_DEFAULT_BYTE_DELAY_US;
	instance_data->split_read_block_delay_us =
	    platformdata->split_read_block_delay_us;
	instance_data->split_read_byte_delay_us =
	    platformdata->split_read_byte_delay_us;

	instance_data->spidev = spi;
	instance_data->rpd.name = RMI4_SPI_DRIVER_NAME;
	instance_data->rpd.proto_name = "spi1";
	instance_data->rpd.write = rmi_spi_write_v1;
	instance_data->rpd.read = rmi_spi_read_v1;
	instance_data->rpd.write_multiple = rmi_spi_write_multiple_v1;
	instance_data->rpd.read_multiple = rmi_spi_read_multiple_v1;
	instance_data->rpd.set_attn_handler = set_attn_handler;
	instance_data->rpd.module = THIS_MODULE;
		/* default to polling if irq not used */
	instance_data->rpd.polling_required = true;


	/* Call the platform setup routine, to do any setup that is
	 * required before interacting with the device.
	 */
	if (sensordata && sensordata->rmi_sensor_setup) {
		retval = sensordata->rmi_sensor_setup();
		if (retval) {
			dev_err(&spi->dev,
				"%s: sensor setup failed with code %d.",
			       __func__, retval);
			goto error_exit;
		}
	}

	instance_data->instance_no = rmi_next_sensor_id();
	dev_set_name(&spi->dev, "%s%d", RMI4_SPI_DEVICE_NAME,
			instance_data->instance_no);

	/* Determine if we need to poll (inefficient) or use interrupts.
	 */
	if (sensordata->attn_gpio_number) {
		instance_data->attn_polarity = sensordata->attn_polarity;
		instance_data->attn_gpio = sensordata->attn_gpio_number;
		instance_data->rpd.polling_required = false;
	} else {
		instance_data->rpd.polling_required = true;
		dev_info(&spi->dev,
			 "%s: No IRQ info given. Polling required.\n",
	   __func__);
	}

	/* Store instance data for later access. */
	if (instance_data)
		spi_set_drvdata(spi, instance_data);

#if	defined(CONFIG_MACH_OMAP3_BEAGLE)
	/* Fixes an issue on Beagleboard - first time read is all 0's,
	 * brief wait required afterwards. */
	retval = instance_data->rpd.read_multiple(&(instance_data->rpd),
					RMI_PDT_START_ADDRESS, (char *)buf,
					6);
	msleep(20);
#endif

	retval = instance_data->rpd.read_multiple(&(instance_data->rpd),
					RMI_PROTOCOL_VERSION_ADDRESS, buf,
					2);
	if (retval < 0) {
		dev_err(&spi->dev,
			"%s: Protocol discovery for SPI V2 failed with %d.",
			__func__, retval);
		goto error_exit;
	}
#if	COMM_DEBUG
	dev_info(&spi->dev,
		 "%s: SPI V2 probe got %02X %02X.", __func__, buf[0], buf[1]);
#endif

	/* buf[0] is equal to SPI proto version - 1. */
	instance_data->spi_version = buf[0] + 1;
	switch (instance_data->spi_version) {
	case 1:
		break;
	case 2:
		instance_data->v2_transaction_size = (unsigned char)buf[1];
		instance_data->rpd.proto_name = "spi2";
		instance_data->rpd.write = rmi_spi_write_v2;
		instance_data->rpd.write_multiple = rmi_spi_write_multiple_v2;
		instance_data->rpd.read = rmi_spi_read_v2;
		instance_data->rpd.read_multiple = rmi_spi_read_multiple_v2;
		dev_info(&spi->dev,
				"%s: Identified SPI V2, transaction size=%d.",
				__func__, instance_data->v2_transaction_size);
		break;
	default:
		instance_data->spi_version = 1;
		dev_warn(&spi->dev,
		    "%s: Unknown SPI version %d encountered. Assuming SPI V1.",
		     __func__, instance_data->spi_version);
	}

	/* Register the sensor driver - which will trigger a scan of the PDT. */
	retval =
	    rmi_register_sensor(&instance_data->rpd, platformdata->sensordata);
	if (retval) {
		dev_err(&spi->dev,
			"%s: sensor registration failed with code %d.",
			__func__, retval);
		goto error_exit;
	}

	if (instance_data->rpd.polling_required == false) {
		retval = request_irq(instance_data->irq, spi_attn_isr,
				irq_type, dev_name(&spi->dev),
				instance_data);
		if (retval) {
			dev_err(&spi->dev,
				"%s: failed to obtain IRQ %d. Result: %d.",
				__func__, instance_data->irq, retval);
			dev_info(&spi->dev, "%s: Reverting to polling.\n",
				 __func__);
			instance_data->rpd.polling_required = true;
			instance_data->irq = 0;
			/* TODO: Need to revert back to polling -
			 * create and start timer. */
		} else {
			dev_dbg(&spi->dev, "%s: got irq.\n", __func__);
			instance_data->irq =
			gpio_to_irq(sensordata->attn_gpio_number);

			instance_data->rpd.irq = instance_data->irq;
			if (instance_data->spi_version == 2) {
				init_waitqueue_head(&instance_data->attn_event);
				instance_data->rpd.read_multiple =
				    rmi_spi_split_read_v2;
			}
			if ((irq_type & IRQ_TYPE_EDGE_BOTH) != 0)

				if (instance_data->attn_gpio &&
					gpio_get_value(
					instance_data->attn_gpio
					) == instance_data->attn_polarity &&
					instance_data->rpd.attention) {

					disable_irq(instance_data->irq);
					instance_data->rpd.attention(
							&instance_data->rpd);
				}
		}

		/* export GPIO for attention handling */

#if defined(CONFIG_SYNA_RMI_DEV)
		retval = gpio_export(instance_data->attn_gpio, false);
		if (retval) {
			dev_warn(&spi->dev, "%s: WARNING: Failed to "
				"export ATTN gpio!.", __func__);
			retval = 0;
		} else {
			retval = gpio_export_link(
				&instance_data->rpd.sensor->
				sensor_device->dev, "attn",
				instance_data->attn_gpio);
			if (retval) {
				dev_warn(
					&instance_data->rpd.sensor->
					sensor_device->dev, "%s: WARNING: "
					"Failed to symlink ATTN gpio!.",
					__func__);
				retval = 0;
			} else {
				dev_info(&instance_data->
					rpd.sensor->sensor_device->dev,
					 "%s: Exported GPIO %d.",
					__func__, instance_data->attn_gpio);
			}
		}
#endif /* CONFIG_SYNA_RMI_DEV */
	}

	dev_info(&spi->dev, "%s: Successfully registered %s.", __func__,
		instance_data->rpd.name);

	return 0;

error_exit:
	if (sensordata && sensordata->rmi_sensor_teardown)
		sensordata->rmi_sensor_teardown();
	if (instance_data->irq)
		free_irq(instance_data->irq, instance_data);
	kfree(instance_data);
	return retval;
}

static int rmi_spi_suspend(struct spi_device *spi, pm_message_t message)
{
	pr_info("%s: Suspending...", __func__);
	return 0;
}

static int rmi_spi_resume(struct spi_device *spi)
{
	pr_info("%s: Resuming...", __func__);
	return 0;
}

static int __devexit rmi_spi_remove(struct spi_device *spi)
{
	struct spi_device_instance_data *instance_data = spi_get_drvdata(spi);
	pr_info("%s: RMI SPI device removed.", __func__);

	rmi_spi_suspend(spi, PMSG_SUSPEND);

	rmi_unregister_sensors(&instance_data->rpd);

	if (instance_data) {
		if (instance_data->irq)
			free_irq(instance_data->irq, instance_data);
		kfree(instance_data);
	}

	return 0;
}

static struct spi_driver rmi_spi_driver = {
	.driver = {
		.name = RMI4_SPI_DRIVER_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		},
	.probe = rmi_spi_probe,
	.remove = __devexit_p(rmi_spi_remove),
	.suspend = rmi_spi_suspend,
	.resume = rmi_spi_resume,
};

static int __init rmi_spi_init(void)
{
	int retval;
	pr_info("%s: RMI SPI physical layer initialization.", __func__);
	retval = spi_register_driver(&rmi_spi_driver);
	if (retval < 0) {
		pr_err("%s: Failed to register spi driver, code = %d.",
		       __func__, retval);
		return retval;
	}
	pr_debug("%s: SPI initialization complete.", __func__);
	return retval;
}

module_init(rmi_spi_init);

static void __exit rmi_spi_exit(void)
{
	pr_info("%s: RMI SPI physical layer exits.", __func__);
	spi_unregister_driver(&rmi_spi_driver);
}

module_exit(rmi_spi_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("RMI4 Driver SPI Physical Layer");
MODULE_LICENSE("GPL");
