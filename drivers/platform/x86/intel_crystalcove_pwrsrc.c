/*
 * intel_crystalcove_pwrsrc.c - Intel Crystal Cove Power Source Detect Driver
 *
 * Copyright (C) 2013 Intel Corporation
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
 * Author: Kannappan R <r.kannappan@intel.com>
 *	Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/notifier.h>
#include <linux/extcon.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <asm/intel_crystalcove_pwrsrc.h>

#define CRYSTALCOVE_PWRSRCIRQ_REG	0x03
#define CRYSTALCOVE_MPWRSRCIRQS0_REG	0x0F
#define CRYSTALCOVE_MPWRSRCIRQSX_REG	0x10
#define CRYSTALCOVE_SPWRSRC_REG		0x1E
#define CRYSTALCOVE_RESETSRC0_REG	0x20
#define CRYSTALCOVE_RESETSRC1_REG	0x21
#define CRYSTALCOVE_WAKESRC_REG		0x22

#define PWRSRC_VBUS_DET			(1 << 0)
#define PWRSRC_DCIN_DET			(1 << 1)
#define PWRSRC_BAT_DET			(1 << 2)

#define CRYSTALCOVE_VBUSCNTL_REG	0x6C
#define VBUSCNTL_EN			(1 << 0)
#define VBUSCNTL_SEL			(1 << 1)

#define PWRSRC_EXTCON_CABLE_AC		"CHARGER_AC"
#define PWRSRC_DRV_NAME			"crystal_cove_pwrsrc"


#ifndef DEBUG
#define dev_dbg(dev, format, arg...)		\
	dev_printk(KERN_DEBUG, dev, format, ##arg)
#endif

static const char *byt_extcon_cable[] = {
	PWRSRC_EXTCON_CABLE_AC,
	NULL,
};

struct pwrsrc_info {
	struct platform_device *pdev;
	int irq;
	struct usb_phy *otg;
	struct extcon_dev *edev;
};

static char *pwrsrc_resetsrc0_info[] = {
	/* bit 0 */ "Last shutdown caused by SOC reporting a thermal event",
	/* bit 1 */ "Last shutdown caused by critical PMIC temperature",
	/* bit 2 */ "Last shutdown caused by critical system temperature",
	/* bit 3 */ "Last shutdown caused by critical battery temperature",
	/* bit 4 */ "Last shutdown caused by VSYS under voltage",
	/* bit 5 */ "Last shutdown caused by VSYS over voltage",
	/* bit 6 */ "Last shutdown caused by battery removal",
	NULL,
};

static char *pwrsrc_resetsrc1_info[] = {
	/* bit 0 */ "Last shutdown caused by VCRIT threshold",
	/* bit 1 */ "Last shutdown caused by BATID reporting battery removal",
	/* bit 2 */ "Last shutdown caused by user pressing the power button",
	NULL,
};

static char *pwrsrc_wakesrc_info[] = {
	/* bit 0 */ "Last wake caused by user pressing the power button",
	/* bit 1 */ "Last wake caused by a battery insertion",
	/* bit 2 */ "Last wake caused by a USB charger insertion",
	/* bit 3 */ "Last wake caused by an adapter insertion",
	NULL,
};

/* Decode and log the given "reset source indicator" register, then clear it */
static void crystalcove_pwrsrc_log_rsi(struct platform_device *pdev,
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

/*
 * D1 ensures SW control: D1[0]  = HW mode, D1[1] = SW mode
 * D0 control the VBUS: D0[0] = disable VBUS, D0[1] = enable VBUS
 */
int crystal_cove_enable_vbus(void)
{
	int ret;

	ret = intel_mid_pmic_writeb(CRYSTALCOVE_VBUSCNTL_REG, 0x03);
	return ret;
}
EXPORT_SYMBOL(crystal_cove_enable_vbus);

int crystal_cove_disable_vbus(void)
{
	int ret;

	ret = intel_mid_pmic_writeb(CRYSTALCOVE_VBUSCNTL_REG, 0x02);
	return ret;
}
EXPORT_SYMBOL(crystal_cove_disable_vbus);

static void handle_pwrsrc_event(struct pwrsrc_info *info, int pwrsrcirq)
{
	int spwrsrc, mask;

	spwrsrc = intel_mid_pmic_readb(CRYSTALCOVE_SPWRSRC_REG);
	if (spwrsrc < 0)
		goto pmic_read_fail;

	if (pwrsrcirq & PWRSRC_VBUS_DET) {
		if (spwrsrc & PWRSRC_VBUS_DET) {
			dev_dbg(&info->pdev->dev, "VBUS attach event\n");
			mask = 1;
		} else {
			dev_dbg(&info->pdev->dev, "VBUS detach event\n");
			mask = 0;
		}
		/* notify OTG driver */
		if (info->otg)
			atomic_notifier_call_chain(&info->otg->notifier,
				USB_EVENT_VBUS, &mask);
	} else if (pwrsrcirq & PWRSRC_DCIN_DET) {
		if (spwrsrc & PWRSRC_DCIN_DET) {
			dev_dbg(&info->pdev->dev, "ADP attach event\n");
			if (info->edev)
				extcon_set_cable_state(info->edev,
						PWRSRC_EXTCON_CABLE_AC, true);
		} else {
			dev_dbg(&info->pdev->dev, "ADP detach event\n");
			if (info->edev)
				extcon_set_cable_state(info->edev,
						PWRSRC_EXTCON_CABLE_AC, false);
		}
	} else if (pwrsrcirq & PWRSRC_BAT_DET) {
		if (spwrsrc & PWRSRC_BAT_DET)
			dev_dbg(&info->pdev->dev, "Battery attach event\n");
		else
			dev_dbg(&info->pdev->dev, "Battery detach event\n");
	} else {
		dev_dbg(&info->pdev->dev, "event none or spurious\n");
	}

	return ;

pmic_read_fail:
	dev_err(&info->pdev->dev, "SPWRSRC read failed:%d\n", spwrsrc);
	return ;
}

static irqreturn_t crystalcove_pwrsrc_isr(int irq, void *data)
{
	struct pwrsrc_info *info = data;
	int pwrsrcirq;

	pwrsrcirq = intel_mid_pmic_readb(CRYSTALCOVE_PWRSRCIRQ_REG);
	if (pwrsrcirq < 0) {
		dev_err(&info->pdev->dev, "PWRSRCIRQ read failed\n");
		goto pmic_irq_fail;
	}

	dev_dbg(&info->pdev->dev, "pwrsrcirq=%x\n", pwrsrcirq);
	handle_pwrsrc_event(info, pwrsrcirq);

pmic_irq_fail:
	intel_mid_pmic_writeb(CRYSTALCOVE_PWRSRCIRQ_REG, pwrsrcirq);
	return IRQ_HANDLED;
}
static int crystalcove_pwrsrc_probe(struct platform_device *pdev)
{
	struct pwrsrc_info *info;
	int ret, pwrsrcirq = 0x0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	info->pdev = pdev;
	info->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, info);

	/* Log reason for last reset and wake events */
	crystalcove_pwrsrc_log_rsi(pdev, pwrsrc_resetsrc0_info,
				CRYSTALCOVE_RESETSRC0_REG);
	crystalcove_pwrsrc_log_rsi(pdev, pwrsrc_resetsrc1_info,
				CRYSTALCOVE_RESETSRC1_REG);
	crystalcove_pwrsrc_log_rsi(pdev, pwrsrc_wakesrc_info,
				CRYSTALCOVE_WAKESRC_REG);

#ifndef CONFIG_EXTCON_FSA9285
	/* Workaround: Set VBUS supply mode to HW control mode */
	intel_mid_pmic_writeb(CRYSTALCOVE_VBUSCNTL_REG, 0x00);

	/* register with extcon */
	info->edev = kzalloc(sizeof(struct extcon_dev), GFP_KERNEL);
	if (!info->edev) {
		dev_err(&pdev->dev, "mem alloc failed\n");
		ret = -ENOMEM;
		goto extcon_mem_failed;
	}
	info->edev->name = "BYT-Charger";
	info->edev->supported_cable = byt_extcon_cable;
	ret = extcon_dev_register(info->edev, &pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "extcon registration failed!!\n");
		goto extcon_reg_failed;
	}

	/* OTG notification */
	info->otg = usb_get_transceiver();
	if (!info->otg) {
		dev_warn(&pdev->dev, "Failed to get otg transceiver!!\n");
		goto otg_reg_failed;
	}
#else
	/* Set VBUS supply mode to SW control mode */
	intel_mid_pmic_writeb(CRYSTALCOVE_VBUSCNTL_REG, 0x02);

	info->edev = NULL;
	info->otg = NULL;
#endif

	/* check if device is already connected */
	if (info->otg)
		pwrsrcirq |= PWRSRC_VBUS_DET;
	if (info->edev)
		pwrsrcirq |= PWRSRC_DCIN_DET;
	if (pwrsrcirq)
		handle_pwrsrc_event(info, pwrsrcirq);

	ret = request_threaded_irq(info->irq, NULL, crystalcove_pwrsrc_isr,
				IRQF_ONESHOT, PWRSRC_DRV_NAME, info);
	if (ret) {
		dev_err(&pdev->dev, "unable to register irq %d\n", info->irq);
		goto intr_teg_failed;
	}

	/* unmask the PWRSRC interrupts */
	intel_mid_pmic_writeb(CRYSTALCOVE_MPWRSRCIRQS0_REG, 0x00);
	intel_mid_pmic_writeb(CRYSTALCOVE_MPWRSRCIRQSX_REG, 0x00);

	return 0;

intr_teg_failed:
#ifndef CONFIG_EXTCON_FSA9285
	if (info->otg)
		usb_put_transceiver(info->otg);
otg_reg_failed:
	if (info->edev)
		extcon_dev_unregister(info->edev);
extcon_reg_failed:
	kfree(info->edev);
extcon_mem_failed:
#endif
	kfree(info);
	return ret;
}

static int crystalcove_pwrsrc_remove(struct platform_device *pdev)
{
	struct pwrsrc_info *info = platform_get_drvdata(pdev);

	free_irq(info->irq, info);
	if (info->otg)
		usb_put_transceiver(info->otg);
	if (info->edev) {
		extcon_dev_unregister(info->edev);
		kfree(info->edev);
	}
	kfree(info);
	return 0;
}

#ifdef CONFIG_PM
static int crystalcove_pwrsrc_suspend(struct device *dev)
{
	return 0;
}

static int crystalcove_pwrsrc_resume(struct device *dev)
{
	return 0;
}
#else
#define crystalcove_pwrsrc_suspend		NULL
#define crystalcove_pwrsrc_resume		NULL
#endif

static const struct dev_pm_ops crystalcove_pwrsrc_driver_pm_ops = {
	.suspend	= crystalcove_pwrsrc_suspend,
	.resume		= crystalcove_pwrsrc_resume,
};

static struct platform_driver crystalcove_pwrsrc_driver = {
	.probe = crystalcove_pwrsrc_probe,
	.remove = crystalcove_pwrsrc_remove,
	.driver = {
		.name = PWRSRC_DRV_NAME,
		.pm = &crystalcove_pwrsrc_driver_pm_ops,
	},
};

static int __init crystalcove_pwrsrc_init(void)
{
	return platform_driver_register(&crystalcove_pwrsrc_driver);
}
fs_initcall(crystalcove_pwrsrc_init);

static void __exit crystalcove_pwrsrc_exit(void)
{
	platform_driver_unregister(&crystalcove_pwrsrc_driver);
}
module_exit(crystalcove_pwrsrc_exit);

MODULE_AUTHOR("Kannappan R <r.kannappan@intel.com>");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("CrystalCove Power Source Detect Driver");
MODULE_LICENSE("GPL");
