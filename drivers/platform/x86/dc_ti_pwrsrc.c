/*
 * dc_ti_pwrsrc.c - Intel Dollar Cove(TI) Power Source Detect Driver
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/usb/phy.h>
#include <linux/notifier.h>
#include <linux/extcon.h>
#include <linux/mfd/intel_mid_pmic.h>

#define DC_PS_SIRQ_REG			0x03
#define SIRQ_VBUS_PRESENT		(1 << 5)

#define DC_PS_IRQ_MASK_REG		0x02
#define IRQ_MASK_VBUS			(1 << 5)

#define DC_PS_BOOT_REASON_REG		0x12

#define PWRSRC_DRV_NAME			"dollar_cove_ti_pwrsrc"

#define PWRSRC_EXTCON_CABLE_USB		"USB"

static const char *dc_extcon_cable[] = {
	PWRSRC_EXTCON_CABLE_USB,
	NULL,
};

struct dc_pwrsrc_info {
	struct platform_device *pdev;
	struct extcon_dev *edev;
	int irq;
};

static char *pwr_up_down_info[] = {
	/* bit 0 */ "Last wake caused by user pressing the power button",
	/* bit 2 */ "Last wake caused by a charger insertion",
	/* bit 1 */ "Last wake caused by a battery insertion",
	/* bit 3 */ "Last wake caused by SOC initiated global reset",
	/* bit 4 */ "Last wake caused by cold reset",
	/* bit 5 */ "Last shutdown caused by PMIC UVLO threshold",
	/* bit 6 */ "Last shutdown caused by SOC initiated cold off",
	/* bit 7 */ "Last shutdown caused by user pressing the power button",
	NULL,
};

/* Decode and log the given "reset source indicator" register, then clear it */
static void dc_ti_pwrsrc_log_rsi(struct platform_device *pdev,
					char **pwrsrc_rsi_info,
					int reg_s)
{
	char *rsi_info = pwrsrc_rsi_info[0];
	int val, i = 0;
	int bit_select, clear_mask = 0x0;

	val = intel_mid_pmic_readb(reg_s);
	while (rsi_info) {
		bit_select = (1 << i);
		if (val & bit_select) {
			dev_info(&pdev->dev, "%s\n", rsi_info);
			clear_mask |= bit_select;
		}
		rsi_info = pwrsrc_rsi_info[++i];
	}

	/* Clear the register value for next reboot (write 1 to clear bit) */
	intel_mid_pmic_writeb(reg_s, clear_mask);
}

int dc_ti_vbus_on_status(void)
{
	int ret;

	ret = intel_mid_pmic_readb(DC_PS_SIRQ_REG);
	if (ret < 0)
		return ret;

	if (ret & SIRQ_VBUS_PRESENT)
		return  1;

	return 0;
}
EXPORT_SYMBOL(dc_ti_vbus_on_status);

static void handle_pwrsrc_event(struct dc_pwrsrc_info *info)
{
	if (dc_ti_vbus_on_status()) {
		dev_info(&info->pdev->dev, "VBUS attach\n");
		if (info->edev)
			extcon_set_cable_state(info->edev,
					PWRSRC_EXTCON_CABLE_USB, true);
	} else {
		dev_info(&info->pdev->dev, "VBUS dettach\n");
		if (info->edev)
			extcon_set_cable_state(info->edev,
					PWRSRC_EXTCON_CABLE_USB, false);
	}

	return;
}

static irqreturn_t dc_ti_pwrsrc_isr(int irq, void *data)
{
	struct dc_pwrsrc_info *info = data;

	handle_pwrsrc_event(info);
	return IRQ_HANDLED;
}

static int pwrsrc_extcon_registration(struct dc_pwrsrc_info *info)
{
	int ret = 0;

	/* register with extcon */
	info->edev = kzalloc(sizeof(struct extcon_dev), GFP_KERNEL);
	if (!info->edev) {
		dev_err(&info->pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}
	info->edev->name = "dc_pwrsrc";
	info->edev->supported_cable = dc_extcon_cable;
	ret = extcon_dev_register(info->edev, &info->pdev->dev);
	if (ret) {
		dev_err(&info->pdev->dev, "extcon registration failed!!\n");
		kfree(info->edev);
	} else {
		dev_info(&info->pdev->dev, "extcon registration success!!\n");
	}

	return ret;
}

static int dc_ti_pwrsrc_probe(struct platform_device *pdev)
{
	struct dc_pwrsrc_info *info;
	int ret, i;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	info->pdev = pdev;
	info->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, info);

	dc_ti_pwrsrc_log_rsi(pdev, pwr_up_down_info,
				DC_PS_BOOT_REASON_REG);

	ret = pwrsrc_extcon_registration(info);
	if (ret < 0)
		return ret;

	ret = request_threaded_irq(info->irq, NULL, dc_ti_pwrsrc_isr,
			IRQF_ONESHOT | IRQF_NO_SUSPEND, PWRSRC_DRV_NAME, info);
	if (ret) {
		dev_err(&pdev->dev, "request_irq fail :%d\n", ret);
		goto intr_reg_failed;
	}

	/* Handle cold pwrsrc insertions */
	handle_pwrsrc_event(info);
	/* Unmask VBUS interrupt */
	intel_mid_pmic_clearb(DC_PS_IRQ_MASK_REG, IRQ_MASK_VBUS);

	return 0;

intr_reg_failed:
	extcon_dev_unregister(info->edev);
	kfree(info->edev);
	return ret;
}

static int dc_ti_pwrsrc_remove(struct platform_device *pdev)
{
	struct dc_pwrsrc_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	extcon_dev_unregister(info->edev);
	kfree(info->edev);
	return 0;
}

#ifdef CONFIG_PM
static int dc_ti_pwrsrc_suspend(struct device *dev)
{
	return 0;
}

static int dc_ti_pwrsrc_resume(struct device *dev)
{
	return 0;
}
#else
#define dc_ti_pwrsrc_suspend		NULL
#define dc_ti_pwrsrc_resume		NULL
#endif

static const struct dev_pm_ops dc_ti_pwrsrc_driver_pm_ops = {
	.suspend	= dc_ti_pwrsrc_suspend,
	.resume		= dc_ti_pwrsrc_resume,
};

static struct platform_driver dc_ti_pwrsrc_driver = {
	.probe = dc_ti_pwrsrc_probe,
	.remove = dc_ti_pwrsrc_remove,
	.driver = {
		.name = PWRSRC_DRV_NAME,
		.pm = &dc_ti_pwrsrc_driver_pm_ops,
	},
};

static int __init dc_ti_pwrsrc_init(void)
{
	return platform_driver_register(&dc_ti_pwrsrc_driver);
}
module_init(dc_ti_pwrsrc_init);

static void __exit dc_ti_pwrsrc_exit(void)
{
	platform_driver_unregister(&dc_ti_pwrsrc_driver);
}
module_exit(dc_ti_pwrsrc_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("DollarCove(TI) Power Source Detect Driver");
MODULE_LICENSE("GPL");
