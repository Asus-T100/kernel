/*
 * intel_mid_pwm.c - Intel PWM driver
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/ipc_device.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_pwm.h>

struct pwm_info {
	int initialized;
	int pwm_num;
	struct intel_mid_pwm_device_data *ddata;
	struct device *dev;
	struct mutex lock;
};

static struct pwm_info pwm_info;

int intel_mid_pwm(int id, int value)
{
	int ret;
	struct pwm_info *pi = &pwm_info;
	u16 reg_pwmclkdiv0, reg_pwmclkdiv1, reg_pwmdutycyc;
	u8 val_pwmclkdiv0, val_pwmclkdiv1;

	if (!pi->initialized)
		return -ENODEV;

	if (id < 0 || id >= pi->pwm_num)
		return -EINVAL;

	if (value < 0 || value > MAX_DUTYCYCLE_PERCENTAGE) {
		dev_err(pi->dev, "duty cycle value invalid\n");
		return -EINVAL;
	}

	value = (value == MAX_DUTYCYCLE_PERCENTAGE) ?
			(MAX_DUTYCYCLE_PERCENTAGE - 1) : value;

	reg_pwmclkdiv0 = pi->ddata[id].reg_clkdiv0;
	reg_pwmclkdiv1 = pi->ddata[id].reg_clkdiv1;
	reg_pwmdutycyc = pi->ddata[id].reg_dutycyc;
	val_pwmclkdiv0 = pi->ddata[id].val_clkdiv0;
	val_pwmclkdiv1 = pi->ddata[id].val_clkdiv1;

	mutex_lock(&pi->lock);
	ret = intel_scu_ipc_iowrite8(reg_pwmclkdiv1, val_pwmclkdiv1);
	if (ret) {
		dev_err(pi->dev, "set MSIC_REG_PWMCLKDIV1 failed\n");
		goto out;
	}

	ret = intel_scu_ipc_iowrite8(reg_pwmclkdiv0,
				value ? val_pwmclkdiv0 : 0x00);
	if (ret) {
		dev_err(pi->dev, "set MSIC_REG_PWMCLKDIV0 failed\n");
		goto out;
	}

	ret = intel_scu_ipc_iowrite8(reg_pwmdutycyc, value);
	if (ret)
		dev_err(pi->dev, "set MSIC_REG_PWMDUTYCYCLE failed\n");

out:
	mutex_unlock(&pi->lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pwm);

static int __devinit intel_mid_pwm_probe(struct ipc_device *ipcdev)
{
	struct pwm_info *pi = &pwm_info;
	struct intel_mid_pwm_platform_data *pdata = ipcdev->dev.platform_data;

	if (!pdata) {
		dev_err(&ipcdev->dev, "No platform data\n");
		return -EINVAL;
	}

	mutex_init(&pi->lock);

	pi->dev = &ipcdev->dev;
	pi->ddata = pdata->ddata;
	pi->pwm_num = pdata->pwm_num;

	if (pi->pwm_num && pi->ddata)
		pi->initialized = 1;

	return 0;
}

static int __devexit intel_mid_pwm_remove(struct ipc_device *ipcdev)
{
	return 0;
}

static struct ipc_driver mid_pwm_driver = {
	.driver = {
		   .name = "intel_mid_pwm",
		   .owner = THIS_MODULE,
	},
	.probe = intel_mid_pwm_probe,
	.remove = __devexit_p(intel_mid_pwm_remove),
};

static int __init intel_mid_pwm_init(void)
{
	return ipc_driver_register(&mid_pwm_driver);
}

static void __exit intel_mid_pwm_exit(void)
{
	ipc_driver_unregister(&mid_pwm_driver);
}

module_init(intel_mid_pwm_init);
module_exit(intel_mid_pwm_exit);

MODULE_DESCRIPTION("Intel Pulse Width Modulator Driver");
MODULE_LICENSE("GPL v2");
