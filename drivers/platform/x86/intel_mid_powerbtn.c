/*
 * Power button driver for Medfield.
 *
 * Copyright (C) 2010 Intel Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/ipc_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <asm/intel_scu_ipc.h>

#define DRIVER_NAME "msic_power_btn"

/* SRAM address for power button state */
#define MSIC_PB_STAT	0xffff7fd0
  #define MSIC_PB_LEVEL (1 << 3) /* 1 - release, 0 - press */
#define MSIC_PB_LEN	1

/*
 * MSIC document ti_datasheet defines the 1st bit reg 0x21 is used to mask
 * power button interrupt
 */
#define MSIC_IRQLVL1MSK	0x21
#define MSIC_PWRBTNM	(1 << 0)

struct mfld_pb_priv {
	struct input_dev *input;
	int irq;
	void __iomem *pb_stat;
};

static irqreturn_t mfld_pb_isr(int irq, void *dev_id)
{
	struct mfld_pb_priv *priv = dev_id;
	u8 pbstat;

	pbstat = readb(priv->pb_stat);
	dev_dbg(&priv->input->dev, "pbstat: 0x%x\n", pbstat);

	input_event(priv->input, EV_KEY, KEY_POWER, !(pbstat & MSIC_PB_LEVEL));
	input_sync(priv->input);

	return IRQ_HANDLED;
}

static int __devinit mfld_pb_probe(struct ipc_device *ipcdev)
{
	struct mfld_pb_priv *priv;
	struct input_dev *input;
	int ret;
	int irq;
	u8 value;

	irq = ipc_get_irq(ipcdev, 0);
	if (irq < 0)
		return -EINVAL;

	priv = kzalloc(sizeof(struct mfld_pb_priv), GFP_KERNEL);
	input = input_allocate_device();
	if (!priv || !input) {
		ret = -ENOMEM;
		goto fail;
	}

	priv->input = input;
	priv->irq = irq;
	ipc_set_drvdata(ipcdev, priv);

	input->name = ipcdev->name;
	input->phys = "power-button/input0";
	input->dev.parent = &ipcdev->dev;

	input_set_capability(input, EV_KEY, KEY_POWER);

	priv->pb_stat = ioremap(MSIC_PB_STAT, MSIC_PB_LEN);
	if (!priv->pb_stat) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = request_irq(priv->irq, mfld_pb_isr,
			  IRQF_NO_SUSPEND, DRIVER_NAME, priv);
	if (ret) {
		dev_err(&ipcdev->dev,
			"unable to request irq %d for power button\n", irq);
		goto out_iounmap;
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(&ipcdev->dev,
			"unable to register input dev, error %d\n", ret);
		goto out_free_irq;
	}

	/* SCU firmware might send power button interrupts to IA core before
	 * kernel boots and doesn't get EOI from IA core. The first bit of
	 * MSIC reg 0x21 is kept masked, and SCU firmware doesn't send new
	 * power interrupt to Android kernel. Unmask the bit when probing
	 * power button in kernel.
	 * There is a very narrow race between irq handler and power button
	 * initialization. The race happens rarely. So we needn't worry
	 * about it.
	 */
	ret = intel_scu_ipc_ioread8(MSIC_IRQLVL1MSK, &value);
	value &= ~MSIC_PWRBTNM;
	ret = intel_scu_ipc_iowrite8(MSIC_IRQLVL1MSK, value);

	return 0;

out_free_irq:
	free_irq(priv->irq, priv);
out_iounmap:
	iounmap(priv->pb_stat);
fail:
	ipc_set_drvdata(ipcdev, NULL);
	input_free_device(input);
	kfree(priv);
	return ret;
}

static int __devexit mfld_pb_remove(struct ipc_device *ipcdev)
{
	struct mfld_pb_priv *priv = ipc_get_drvdata(ipcdev);

	iounmap(priv->pb_stat);
	free_irq(priv->irq, priv);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

static struct ipc_driver mfld_pb_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe  = mfld_pb_probe,
	.remove = __devexit_p(mfld_pb_remove),
};

static int __init mfld_pb_init(void)
{
	return ipc_driver_register(&mfld_pb_driver);
}

static void __exit mfld_pb_exit(void)
{
	ipc_driver_unregister(&mfld_pb_driver);
}

module_init(mfld_pb_init);
module_exit(mfld_pb_exit);

MODULE_AUTHOR("Hong Liu <hong.liu@intel.com>");
MODULE_DESCRIPTION("Intel Medfield Power Button Driver");
MODULE_LICENSE("GPL v2");
