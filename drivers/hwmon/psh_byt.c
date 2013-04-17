/*
 *  psh.c - Baytrail PSH IA side driver
 *
 *  (C) Copyright 2012 Intel Corporation
 *  Author: Alek Du <alek.du@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA	02110-1301, USA
 */

/*
 * PSH IA side driver for Baytrail Platform
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/pci.h>
#include <linux/circ_buf.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/acpi_gpio.h>
#include <asm/intel_vlv2.h>

#include "psh_ia_common.h"


/* need a global lock to check the psh driver access */
struct psh_ext_if {
	struct device *hwmon_dev;
	struct i2c_client *pshc;
	char psh_frame[LBUF_MAX_CELL_SIZE];

	int gpio_psh_ctl, gpio_psh_rst;

	int irq_disabled;
};

int read_psh_data(struct psh_ia_priv *ia_data)
{
	struct psh_ext_if *psh_if_info =
			(struct psh_ext_if *)ia_data->platform_priv;
	int total_read = 0, ret = 0;
	struct frame_head fh;
	struct i2c_msg msg[2] = {
		{
		.addr = psh_if_info->pshc->addr,
		.flags = I2C_M_RD,
		.len = sizeof(fh),
		.buf = (void *)&fh
		},
		{
		.addr = psh_if_info->pshc->addr,
		.flags = I2C_M_RD,
		.buf = (void *)&psh_if_info->psh_frame
		}
	};

	/* We may need to zero all the buffer */

	/* Loop read till error or no more valid data */
	while (1) {
		ret = i2c_transfer(psh_if_info->pshc->adapter, msg, 1);
		if (ret != 1) {
			dev_err(&psh_if_info->pshc->dev, "Read frame header error!\n");
			ret = -EPERM;
			break;
		}

		if (fh.sign == LBUF_CELL_SIGN) {
			if (fh.length > LBUF_MAX_CELL_SIZE) {
				ret = -EPERM;
				break;
			}
		} else
			break;		/* No valid data read */

		msg[1].len = frame_size(fh.length) - sizeof(fh);
		ret = i2c_transfer(psh_if_info->pshc->adapter, msg + 1, 1);
		if (ret != 1) {
			dev_err(&psh_if_info->pshc->dev, "Read main frame error!\n");
			ret = -EPERM;
			break;
		}

		ret = ia_handle_frame(ia_data,
					psh_if_info->psh_frame, fh.length);
		if (ret > 0)
			total_read += ret;

	}

	if (total_read)
		sysfs_notify(&psh_if_info->pshc->dev.kobj, NULL, "data_size");

	return ret;
}

int process_send_cmd(struct psh_ia_priv *ia_data,
			int ch, struct ia_cmd *cmd, int len)
{
	struct psh_ext_if *psh_if_info =
			(struct psh_ext_if *)ia_data->platform_priv;
	int ret = 0;
	struct i2c_msg i2c_cmd = {
		.addr = psh_if_info->pshc->addr,
		.flags = 0,
		.len = len,
		.buf = (void *)cmd
	};

	if (ch == 0 && cmd->cmd_id == CMD_RESET) {
		if (psh_if_info->irq_disabled == 0) {
			disable_irq(psh_if_info->pshc->irq);
			psh_if_info->irq_disabled = 1;
		}

		gpio_set_value(psh_if_info->gpio_psh_ctl, 1);
		gpio_set_value(psh_if_info->gpio_psh_rst, 0);
		usleep_range(10000, 10000);
		gpio_set_value(psh_if_info->gpio_psh_rst, 1);

		/* wait for pshfw to run */
		msleep(1000);

		if (psh_if_info->irq_disabled == 1) {
			enable_irq(psh_if_info->pshc->irq);
			psh_if_info->irq_disabled = 0;
		}
	}

	ret = i2c_transfer(psh_if_info->pshc->adapter, &i2c_cmd, 1);
	if (ret != 1) {
		dev_err(&psh_if_info->pshc->dev, "sendcmd through I2C fail!\n");
		return -EPERM;
	}

	return 0;
}

int do_setup_ddr(struct device *dev)
{
	return 0;
}

static irqreturn_t psh_byt_irq_thread(int irq, void *dev)
{
	struct i2c_client *client = (struct i2c_client *)dev;
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(&client->dev);

	read_psh_data(ia_data);
	return IRQ_HANDLED;
}

/* FIXME: it will be a platform device */
static int psh_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret = -EPERM;
	struct psh_ia_priv *ia_data;
	struct psh_ext_if *psh_if_info;

	psh_if_info = kzalloc(sizeof(*psh_if_info), GFP_KERNEL);
	if (!psh_if_info) {
		dev_err(&client->dev, "can not allocate psh_if_info\n");
		goto psh_if_err;
	}

	ret = psh_ia_common_init(&client->dev, &ia_data);
	if (ret) {
		dev_err(&client->dev, "fail to init psh_ia_common\n");
		goto psh_ia_err;
	}

	psh_if_info->hwmon_dev = hwmon_device_register(&client->dev);
	if (!psh_if_info->hwmon_dev) {
		dev_err(&client->dev, "fail to register hwmon device\n");
		goto hwmon_err;
	}

	psh_if_info->pshc = client;

	ia_data->platform_priv = psh_if_info;

	psh_if_info->gpio_psh_ctl =
				acpi_get_gpio_by_index(&client->dev, 1, NULL);
	if (psh_if_info->gpio_psh_ctl < 0) {
		dev_warn(&client->dev, "fail to get psh_ctl pin by ACPI\n");
	} else {
		int rc = gpio_request(psh_if_info->gpio_psh_ctl, "psh_ctl");
		if (rc) {
			dev_warn(&client->dev, "fail to request psh_ctl pin\n");
			psh_if_info->gpio_psh_ctl = -1;
		} else {
			gpio_export(psh_if_info->gpio_psh_ctl, 1);
			gpio_direction_output(psh_if_info->gpio_psh_ctl, 1);
			gpio_set_value(psh_if_info->gpio_psh_ctl, 1);
		}
	}

	psh_if_info->gpio_psh_rst =
				acpi_get_gpio_by_index(&client->dev, 0, NULL);
	if (psh_if_info->gpio_psh_rst < 0) {
		dev_warn(&client->dev, "failed to get psh_rst pin by ACPI\n");
	} else {
		int rc = gpio_request(psh_if_info->gpio_psh_rst, "psh_rst");
		if (rc) {
			dev_warn(&client->dev, "fail to request psh_rst pin\n");
			psh_if_info->gpio_psh_rst = -1;
		} else {
			gpio_export(psh_if_info->gpio_psh_rst, 1);
			gpio_direction_output(psh_if_info->gpio_psh_rst, 1);
			gpio_set_value(psh_if_info->gpio_psh_rst, 1);
		}
	}

	ret = request_threaded_irq(client->irq, NULL, psh_byt_irq_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "psh_byt", client);
	if (ret) {
		dev_err(&client->dev, "fail to request irq\n");
		goto irq_err;
	}

	disable_irq(client->irq);

	psh_if_info->irq_disabled = 1;

	return 0;

irq_err:
	hwmon_device_unregister(psh_if_info->hwmon_dev);
hwmon_err:
	psh_ia_common_deinit(&client->dev);
psh_ia_err:
	kfree(psh_if_info);
psh_if_err:
	return ret;
}

static int __devexit psh_remove(struct i2c_client *client)
{
	struct psh_ia_priv *ia_data =
			(struct psh_ia_priv *)dev_get_drvdata(&client->dev);
	struct psh_ext_if *psh_if_info =
			(struct psh_ext_if *)ia_data->platform_priv;

	free_irq(client->irq, psh_if_info->pshc);
	gpio_unexport(psh_if_info->gpio_psh_rst);
	gpio_unexport(psh_if_info->gpio_psh_ctl);
	gpio_free(psh_if_info->gpio_psh_rst);
	gpio_free(psh_if_info->gpio_psh_ctl);
	hwmon_device_unregister(psh_if_info->hwmon_dev);
	psh_ia_common_deinit(&client->dev);

	return 0;
}

static const struct i2c_device_id psh_byt_id[] = {
	{ "SMO91D0:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, psh_byt_id);

static struct i2c_driver psh_byt_driver = {
	.driver = {
		.name	= "psh_byt_i2c",
		.owner	= THIS_MODULE,
	},
	.probe		= psh_probe,
	.remove		= psh_remove,
	.id_table	= psh_byt_id,
};

module_i2c_driver(psh_byt_driver);

MODULE_LICENSE("GPL v2");
