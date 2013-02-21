/* Cloverview Plus PMIC Charger (USBDET interrupt) driver
 * Copyright (C) 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>

#define DRIVER_NAME "pmic_charger"

static irqreturn_t pmic_charger_irq_handler(int irq, void *devid)
{
	struct device *dev = (struct device *)devid;

	/*
	 * This hanlder is used for USB_DET handling, just for waking up
	 * the system and do nothing.
	 */
	dev_dbg(dev, "IRQ Handled for pmic charger interrupt: %d\n", irq);

	return IRQ_HANDLED;
}

static int __devinit pmic_charger_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq = platform_get_irq(pdev, 0);
	int ret;

	/* Register a handler for USBDET interrupt */
	ret = request_irq(irq, pmic_charger_irq_handler, IRQF_TRIGGER_FALLING,
			"pmic_usbdet_interrupt", dev);
	if (ret) {
		dev_info(dev, "register USBDET IRQ with error %d\n", ret);
		return ret;
	}

	dev_info(dev, "registered USBDET IRQ %d\n", irq);

	return 0;
}

static int __devexit pmic_charger_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int irq = platform_get_irq(pdev, 0);

	free_irq(irq, dev);

	return 0;
}

static struct platform_driver pmic_charger_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= pmic_charger_probe,
	.remove = __devexit_p(pmic_charger_remove),
};

static int __init pmic_charger_module_init(void)
{
	return platform_driver_register(&pmic_charger_driver);
}

static void __exit pmic_charger_module_exit(void)
{
	platform_driver_unregister(&pmic_charger_driver);
}

module_init(pmic_charger_module_init);
module_exit(pmic_charger_module_exit);

MODULE_AUTHOR("Dongsheng Zhang <dongsheng.zhang@intel.com>");
MODULE_AUTHOR("Rapaka, Naveen <naveen.rapaka@intel.com>");
MODULE_DESCRIPTION("Intel Pmic charger Driver");
MODULE_LICENSE("GPL v2");
