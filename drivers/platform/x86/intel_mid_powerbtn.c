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
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/rpmsg.h>
#include <linux/async.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>

#define DRIVER_NAME "msic_power_btn"

#define MSIC_PB_LEVEL (1 << 3) /* 1 - release, 0 - press */

/*
 * MSIC document ti_datasheet defines the 1st bit reg 0x21 is used to mask
 * power button interrupt
 */
#define MSIC_IRQLVL1MSK	0x21
#define MSIC_PWRBTNM    (1 << 0)

struct mid_pb_prov {
	struct input_dev *input;
	int irq;
	void __iomem *pb_stat;
};

static irqreturn_t mid_pb_isr(int irq, void *dev_id)
{
	struct mid_pb_prov *priv = dev_id;
	u8 pbstat;

	pbstat = readb(priv->pb_stat);
	dev_dbg(&priv->input->dev, "pbstat: 0x%x\n", pbstat);

	input_event(priv->input, EV_KEY, KEY_POWER, !(pbstat & MSIC_PB_LEVEL));
	input_sync(priv->input);

	return IRQ_HANDLED;
}


static int __devinit mid_pb_probe(struct platform_device *pdev)
{
	struct input_dev *input;
	struct mid_pb_prov *priv;
	struct resource *rc;
	int irq;
	int ret;
	u8 value;

	if (pdev == NULL)
		return -ENODEV;

	rc = platform_get_resource_byname(pdev, IORESOURCE_MEM, "MSIC_PB_STAT");
	if (!rc) {
		dev_err(&pdev->dev, "failed to get resource");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "Probed mid powerbutton devivce\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	priv = kzalloc(sizeof(struct mid_pb_prov), GFP_KERNEL);
	input = input_allocate_device();
	if (!priv || !input)
		return -ENOMEM;

	priv->input = input;
	priv->irq = irq;
	platform_set_drvdata(pdev, priv);

	input->name = pdev->name;
	input->phys = "power-button/input0";
	input->id.bustype = BUS_HOST;
	input->dev.parent = &pdev->dev;

	input_set_capability(input, EV_KEY, KEY_POWER);

	priv->pb_stat = ioremap_nocache(rc->start, resource_size(rc));
	if (!priv->pb_stat) {
		ret = -ENOMEM;
		goto fail;
	}

	ret = request_irq(priv->irq, mid_pb_isr,
			  IRQF_NO_SUSPEND, DRIVER_NAME, priv);

	if (ret) {
		dev_err(&pdev->dev,
			"unable to request irq %d for power button\n", irq);
		goto out_iounmap;
	}

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev,
			"unable to register input dev, error %d\n", ret);
		goto out_free_irq;
	}

	platform_set_drvdata(pdev, input);

	/*
	 * SCU firmware might send power button interrupts to IA core before
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
	free_irq(irq, input);
out_iounmap:
	iounmap(priv->pb_stat);
fail:
	platform_set_drvdata(pdev, NULL);
	input_free_device(input);
	kfree(priv);
	return ret;
}

static int __devexit mid_pb_remove(struct platform_device *pdev)
{
	struct mid_pb_prov *priv = platform_get_drvdata(pdev);

	iounmap(priv->pb_stat);
	free_irq(priv->irq, priv);
	platform_set_drvdata(pdev, NULL);
	input_unregister_device(priv->input);
	kfree(priv);

	return 0;
}

static const struct platform_device_id mid_pb_table[] = {
	{"mid_powerbtn", 1},
};

static struct platform_driver mid_pb_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe	= mid_pb_probe,
	.remove	= __devexit_p(mid_pb_remove),
	.id_table = mid_pb_table,
};

static int __init mid_pb_module_init(void)
{
	return platform_driver_register(&mid_pb_driver);
}

static void  mid_pb_module_exit(void)
{
	platform_driver_unregister(&mid_pb_driver);
}

/* RPMSG related functionality */

static int mid_pb_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;
	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed mid_pb rpmsg device\n");

	ret = mid_pb_module_init();
out:
	return ret;
}


static void __devexit mid_pb_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	mid_pb_module_exit();
	dev_info(&rpdev->dev, "Removed mid_pb rpmsg device\n");
}

static void mid_pb_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id mid_pb_id_table[] = {
	{ .name	= "rpmsg_mid_powerbtn" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, mid_pb_id_table);


static struct rpmsg_driver mid_pb_rpmsg = {
	.drv.name	= DRIVER_NAME,
	.drv.owner	= THIS_MODULE,
	.probe		= mid_pb_rpmsg_probe,
	.callback	= mid_pb_rpmsg_cb,
	.remove		= __devexit_p(mid_pb_rpmsg_remove),
	.id_table	= mid_pb_id_table,
};

static int __init mid_pb_rpmsg_init(void)
{
	return register_rpmsg_driver(&mid_pb_rpmsg);
}

static void __exit mid_pb_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&mid_pb_rpmsg);
}

#ifdef MODULE
module_init(mid_pb_rpmsg_init);
#else
late_initcall_async(mid_pb_rpmsg_init);
#endif
module_exit(mid_pb_rpmsg_exit);

MODULE_AUTHOR("Hong Liu <hong.liu@intel.com>");
MODULE_DESCRIPTION("Intel Medfield Power Button Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
