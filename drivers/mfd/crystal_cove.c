/*
 * Crystal Cove  --  Device access for Intel PMIC for VLV2
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * Author: Yang Bin <bin.yang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/mfd/intel_mid_pmic.h>

#define PMIC_IRQ_NUM	7

#define CHIPID		0x00
#define CHIPVER		0x01
#define IRQLVL1		0x02
#define MIRQLVL1	0x0E

struct intel_mid_pmic {
	struct i2c_client *i2c;
	struct mutex io_lock;
	struct device *dev;
	int irq;
	struct mutex irq_lock;
	int irq_base;
	int irq_mask;
};

static struct intel_mid_pmic intel_mid_pmic;
static struct intel_mid_pmic *pmic = &intel_mid_pmic;

int intel_mid_pmic_readb(int reg)
{
	int ret;

	mutex_lock(&pmic->io_lock);
	ret = i2c_smbus_read_byte_data(pmic->i2c, reg);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

int intel_mid_pmic_writeb(int reg, u8 val)
{
	int ret;

	mutex_lock(&pmic->io_lock);
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

int intel_mid_pmic_setb(int reg, u8 mask)
{
	int ret;
	int val;

	mutex_lock(&pmic->io_lock);
	val = i2c_smbus_read_byte_data(pmic->i2c, reg);
	val |= mask;
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

int intel_mid_pmic_clearb(int reg, u8 mask)
{
	int ret;
	int val;

	mutex_lock(&pmic->io_lock);
	val = i2c_smbus_read_byte_data(pmic->i2c, reg);
	val &= ~mask;
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

static void pmic_irq_enable(struct irq_data *data)
{
	pmic->irq_mask &= ~(1 << (data->irq - pmic->irq_base));
}

static void pmic_irq_disable(struct irq_data *data)
{
	pmic->irq_mask |= 1 << (data->irq - pmic->irq_base);
}

static void pmic_irq_sync_unlock(struct irq_data *data)
{
	mutex_unlock(&pmic->irq_lock);
}

static void pmic_irq_lock(struct irq_data *data)
{
	mutex_lock(&pmic->irq_lock);
}

static irqreturn_t pmic_irq_thread(int irq, void *data)
{
	int i;
	int pending;

	intel_mid_pmic_writeb(MIRQLVL1, pmic->irq_mask);
	pending = intel_mid_pmic_readb(IRQLVL1) & (~pmic->irq_mask);
	for (i = 0; i < PMIC_IRQ_NUM; i++)
		if (pending & (1 << i))
			handle_nested_irq(pmic->irq_base + i);
	return IRQ_HANDLED;
}

static struct irq_chip pmic_irq_chip = {
	.name			= "intel_mid_pmic",
	.irq_bus_lock		= pmic_irq_lock,
	.irq_bus_sync_unlock	= pmic_irq_sync_unlock,
	.irq_disable		= pmic_irq_disable,
	.irq_enable		= pmic_irq_enable,
};

static int pmic_polling_thread(void *data)
{
	while (1) {
		pmic_irq_thread(0, NULL);
		msleep(500);
	}
}

static int pmic_irq_init(void)
{
	int cur_irq;
	int ret;

	intel_mid_pmic_writeb(MIRQLVL1, 0x7f);
	pmic->irq_base = irq_alloc_descs(256, 0, PMIC_IRQ_NUM, 0);
	if (pmic->irq_base < 0) {
		dev_warn(pmic->dev, "Failed to allocate IRQs: %d\n",
			 pmic->irq_base);
		pmic->irq_base = 0;
		return -EINVAL;
	}

	/* Register them with genirq */
	for (cur_irq = pmic->irq_base;
	     cur_irq < PMIC_IRQ_NUM + pmic->irq_base;
	     cur_irq++) {
		irq_set_chip_data(cur_irq, pmic);
		irq_set_chip_and_handler(cur_irq, &pmic_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);
		irq_set_noprobe(cur_irq);
	}

	if (pmic->irq == 0)
		kthread_run(pmic_polling_thread, NULL, "pmic polling irq");
	else {
		ret = request_threaded_irq(pmic->irq, NULL, pmic_irq_thread,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				"intel_mid_pmic", pmic);
		if (ret != 0) {
			dev_err(pmic->dev, "Failed to request IRQ %d: %d\n",
					pmic->irq, ret);
			return ret;
		}
		ret = enable_irq_wake(pmic->irq);
		if (ret != 0) {
			dev_warn(pmic->dev, "Can't enable PMIC IRQ as wake source: %d\n",
				 ret);
		}
	}

	return 0;
}

static int pmic_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	int i;
	struct mfd_cell *cell_dev = i2c->dev.platform_data;

	mutex_init(&pmic->io_lock);
	mutex_init(&pmic->irq_lock);
	pmic->i2c = i2c;
	pmic->dev = &i2c->dev;
	pmic->irq = i2c->irq;
	pmic_irq_init();
	dev_info(&i2c->dev, "Crystal Cove: ID 0x%02X, VERSION 0x%02X\n",
		intel_mid_pmic_readb(CHIPID), intel_mid_pmic_readb(CHIPVER));
	for (i = 0; cell_dev[i].name != NULL; i++)
		;
	return mfd_add_devices(pmic->dev, -1, cell_dev, i,
			NULL, pmic->irq_base);
}

static int pmic_i2c_remove(struct i2c_client *i2c)
{
	mfd_remove_devices(pmic->dev);
	return 0;
}

static const struct i2c_device_id pmic_i2c_id[] = {
	{ "crystal_cove", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pmic_i2c_id);

static struct i2c_driver pmic_i2c_driver = {
	.driver = {
		.name = "intel_mid_i2c_pmic",
		.owner = THIS_MODULE,
	},
	.probe = pmic_i2c_probe,
	.remove = pmic_i2c_remove,
	.id_table = pmic_i2c_id,
};

static int __init pmic_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&pmic_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register pmic I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(pmic_i2c_init);

static void __exit pmic_i2c_exit(void)
{
	i2c_del_driver(&pmic_i2c_driver);
}
module_exit(pmic_i2c_exit);

MODULE_DESCRIPTION("Crystal Cove support for ValleyView2 PMIC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");


