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
	int *msic_reg_clkdiv0;
	int *msic_reg_clkdiv1;
	int *msic_reg_dutycyc;
	struct device *dev;
	struct mutex lock;
};

static struct pwm_info pwm_info;

int intel_mid_pwm(int id, int value)
{
	int ret;
	struct pwm_info *pi = &pwm_info;
	int msic_reg_pwmclkdiv0 = *(pi->msic_reg_clkdiv0 + id);
	int msic_reg_pwmclkdiv1 = *(pi->msic_reg_clkdiv1 + id);
	int msic_reg_pwmdutycyc = *(pi->msic_reg_dutycyc + id);

	if (!pi->initialized)
		return -ENODEV;

	if (value < 0 || value > MAX_DUTYCYCLE_PERCENTAGE) {
		dev_err(pi->dev, "duty cycle value invalid\n");
		return -EINVAL;
	}

	value = (value == 100) ? 99 : value;

	mutex_lock(&pi->lock);
	ret = intel_scu_ipc_iowrite8(msic_reg_pwmclkdiv1, 0x00);
	if (ret) {
		dev_err(pi->dev, "set MSIC_REG_PWMCLKDIV1 failed\n");
		goto out;
	}

	ret = intel_scu_ipc_iowrite8(msic_reg_pwmclkdiv0, value ? 0x03 : 0x00);
	if (ret) {
		dev_err(pi->dev, "set MSIC_REG_PWMCLKDIV0 failed\n");
		goto out;
	}

	ret = intel_scu_ipc_iowrite8(msic_reg_pwmdutycyc, value);
	if (ret)
		dev_err(pi->dev, "set MSIC_REG_PWMDUTYCYCLE failed\n");

out:
	mutex_unlock(&pi->lock);
	return ret;
}

static int __devinit intel_mid_pwm_probe(struct ipc_device *ipcdev)
{
	struct pwm_info *pi = &pwm_info;
	struct intel_mid_pwm_platform_data *pdata = ipcdev->dev.platform_data;

	mutex_init(&pi->lock);

	pi->dev = &ipcdev->dev;
	pi->msic_reg_clkdiv0 = pdata->reg_clkdiv0;
	pi->msic_reg_clkdiv1 = pdata->reg_clkdiv1;
	pi->msic_reg_dutycyc = pdata->reg_dutycyc;

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
