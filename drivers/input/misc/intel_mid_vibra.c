/*
 *  intel_mid_vibra.c - Intel vibrator driver for Clovertrail and mrfld phone
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: KP, Jeeja <jeeja.kp@intel.com>
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>

#define INTEL_VIBRA_DRV_NAME "intel_vibra_driver"
#define INTEL_VIBRA_CLV_PCI_ID 0x0905
#define INTEL_VIBRA_MRFLD_PCI_ID 0x11a5

#define INTEL_VIBRA_ENABLE_GPIO 40
#define INTEL_PWM_ENABLE_GPIO 49

#define INTEL_VIBRA_MAX_TIMEDIVISOR  0xFF
#define INTEL_VIBRA_MAX_BASEUNIT 0x80

struct mid_vibra_probe {
	u8 time_divisor;
	u8 base_unit;
	u8 alt_fn;
	u8 ext_drv;
};

#define INFO(_time_divisor, _base_unit, _alt_fn, _ext_drv)	\
	((kernel_ulong_t)&(struct mid_vibra_probe) {	\
	 .time_divisor = _time_divisor,			\
	 .base_unit = _base_unit,			\
	 .alt_fn = _alt_fn,				\
	 .ext_drv = _ext_drv				\
	 })

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

struct vibra_info {
	int     enabled;
	struct mutex	lock;
	struct device	*dev;
	void __iomem	*shim;
	const char	*name;
	struct pci_dev	*pci;
	union sst_pwmctrl_reg   pwm;
	int gpio_en;
	int gpio_pwm;
	int alt_fn;
	int ext_drv;
};

#define VIBRA_DRV_BUS 0x2
#define VIBRA_DRV_SLAVE 0x5a

static int vibra_driver_write(u8 reg, u8 value)
{
	struct i2c_adapter *adap;
	struct i2c_msg msg;
	u8 buffer[2];
	int ret = 0;

	adap = i2c_get_adapter(VIBRA_DRV_BUS);
	if (!adap) {
		pr_err("can't find bus adapter");
		return -EIO;
	}
	buffer[0] = reg;
	buffer[1] = value;
	pr_debug("write for %x, value %x", buffer[0], buffer[1]);

	msg.addr = VIBRA_DRV_SLAVE;
	msg.len = 2;
	msg.buf = (u8 *)&buffer;
	msg.flags = 0;
	ret = i2c_transfer(adap, &msg, 1);
	if (ret != 1)
		pr_err("i2c write error: %d for reg %x", ret, reg);
	else
		ret = 0;
	return ret;
}

static int vibra_pwm_configure(struct vibra_info *info, unsigned int enable)
{
	union sst_pwmctrl_reg pwmctrl;

	if (enable) {
		/*1. Enable the PWM by setting PWM enable bit to 1 */
		pwmctrl.full = readl(info->shim);
		pr_debug("Vibra:Read pwmctrl %x\n", readl(info->shim));
		pwmctrl.part.pwmenable = 1;
		writel(pwmctrl.full, info->shim);

		/*2. Read the PWM register to make sure there is no pending
		*update.
		*/
		pwmctrl.full = readl(info->shim);
		pr_debug("Read pwmctrl %x\n", pwmctrl.full);

		/*check pwnswupdate bit */
		if (pwmctrl.part.pwmswupdate)
			return -EBUSY;
		/*Base unit == 1*/
		pwmctrl.part.pwmswupdate = 0x1;
		pwmctrl.part.pwmbu = info->pwm.part.pwmbu;
		pwmctrl.part.pwmtd = info->pwm.part.pwmtd;
		writel(pwmctrl.full,  info->shim);
		pr_debug("Read pwmctrl %x\n", pwmctrl.full);
	} else { /*disable PWM block */
		   /*1. setting PWM enable bit to 0 */
		pwmctrl.full = readl(info->shim);
		pwmctrl.part.pwmenable = 0;
		writel(pwmctrl.full,  info->shim);
	}
	return 0;
}
/* Enable's vibra driver */
static void vibra_enable(struct vibra_info *info)
{
	pr_debug("Enable gpio\n");
	mutex_lock(&info->lock);
	pm_runtime_get_sync(&info->pci->dev);
	lnw_gpio_set_alt(info->gpio_pwm, info->alt_fn);
	vibra_pwm_configure(info, true);
	gpio_set_value(info->gpio_pwm, 1);
	gpio_set_value(info->gpio_en, 1);
	info->enabled = true;
	mutex_unlock(&info->lock);
}

static void vibra_disable(struct vibra_info *info)
{
	pr_debug("Disable gpio\n");
	mutex_lock(&info->lock);
	gpio_set_value(info->gpio_pwm, 0);
	gpio_set_value(info->gpio_en, 0);
	lnw_gpio_set_alt(info->gpio_pwm, 0);
	info->enabled = false;
	vibra_pwm_configure(info, false);
	pm_runtime_put(&info->pci->dev);
	mutex_unlock(&info->lock);
}


/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/

static ssize_t vibra_show_vibrator(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", info->enabled);

}

static ssize_t vibra_set_vibrator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	long vibrator_enable;
	struct vibra_info *info = dev_get_drvdata(dev);

	if (kstrtol(buf, 0, &vibrator_enable))
		return -EINVAL;
	if (vibrator_enable == info->enabled)
		return len;
	else if (vibrator_enable == 0)
		vibra_disable(info);
	else if (vibrator_enable == 1)
		vibra_enable(info);
	else
		return -EINVAL;
	return len;
}

static ssize_t vibra_set_pwm_baseunit(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long  pwm_base;
	struct vibra_info *info = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &pwm_base))
		return -EINVAL;

	pr_debug("PWM value 0x%lx\n", pwm_base);
	pwm_base = abs(pwm_base);

	if (pwm_base < 0 || pwm_base > INTEL_VIBRA_MAX_BASEUNIT) {
		pr_err("Supported value is out of Range\n");
		return -EINVAL;
	}
	pr_debug("PWM value 0x%lx\n", pwm_base);
	info->pwm.part.pwmbu = pwm_base;
	return len;
}

static ssize_t vibra_show_pwm_baseunit(struct device *dev,
	 struct device_attribute *attr, char *buf)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "0x%X\n", info->pwm.part.pwmbu);
}

static ssize_t vibra_set_pwm_ontime_div(struct device *dev,
	 struct device_attribute *attr, const char *buf, size_t len)
{
	unsigned long  pwm_td;
	struct vibra_info *info = dev_get_drvdata(dev);

	if (kstrtoul(buf, 0, &pwm_td))
		return -EINVAL;
	pr_debug("PWM value 0x%lx\n", pwm_td);
	pwm_td = abs(pwm_td);
	if (pwm_td > INTEL_VIBRA_MAX_TIMEDIVISOR || pwm_td < 0) {
		pr_err("Supported value is out of Range\n");
		return -EINVAL;
	}

	info->pwm.part.pwmtd = pwm_td;

	return len;
}

static ssize_t vibra_show_pwm_ontime_div(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "0x%X\n", info->pwm.part.pwmtd);
}

static struct device_attribute vibra_attrs[] = {
	__ATTR(vibrator, S_IRUGO | S_IWUSR,
	       vibra_show_vibrator, vibra_set_vibrator),
	__ATTR(pwm_baseunit, S_IRUGO | S_IWUSR,
	       vibra_show_pwm_baseunit, vibra_set_pwm_baseunit),
	__ATTR(pwm_ontime_div, S_IRUGO | S_IWUSR,
	       vibra_show_pwm_ontime_div, vibra_set_pwm_ontime_div),
};

static int vibra_register_sysfs(struct vibra_info *info)
{
	int r, i;

	for (i = 0; i < ARRAY_SIZE(vibra_attrs); i++) {
		r = device_create_file(info->dev, &vibra_attrs[i]);
		if (r)
			goto fail;
	}
	return 0;
fail:
	while (i--)
		device_remove_file(info->dev, &vibra_attrs[i]);

	return r;
}

static void vibra_unregister_sysfs(struct vibra_info *info)
{
	int i;

	for (i = ARRAY_SIZE(vibra_attrs) - 1; i >= 0; i--)
		device_remove_file(info->dev, &vibra_attrs[i]);
}

/*** Module ***/
#if CONFIG_PM
static int intel_vibra_runtime_suspend(struct device *dev)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	pr_debug("In %s\n", __func__);
	vibra_pwm_configure(info, false);
	return 0;
}

static int intel_vibra_runtime_resume(struct device *dev)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	pr_debug("In %s\n", __func__);
	vibra_pwm_configure(info, true);
	return 0;
}

static const struct dev_pm_ops intel_mid_vibra_pm_ops = {
	.suspend = intel_vibra_runtime_suspend,
	.resume = intel_vibra_runtime_resume,
	.runtime_suspend = intel_vibra_runtime_suspend,
	.runtime_resume = intel_vibra_runtime_resume,
};

#endif

/* vibra_init_ext_drv: initializes the ext drv and auto calibrates it one time
 *
 * @info: vibrsa driver context
 */
static int vibra_init_ext_drv(struct vibra_info *info)
{
#define DRV_MODE	0x01
#define DRV_GO		0x0c
#define DRV_VOLTAGE	0x16
#define DRV_CLAMP	0x17
#define DRV_FB_CONTROL	0x1a

#define DRV_AUTO_CALIB	0x07
#define DRV_2_0V	0x5b
#define DRV_LRA		0xa4
#define DRV_PWM		0x03
#define DRV_GO_BIT	0x01

	/*enable gpio first */
	gpio_set_value(info->gpio_pwm, 1);
	gpio_set_value(info->gpio_en, 1);
	/* wait for gpio to settle and drv to accept i2c*/
	msleep(1);

	/*put device in auto calibrate mode*/
	vibra_driver_write(DRV_MODE, DRV_AUTO_CALIB);
	vibra_driver_write(DRV_FB_CONTROL, DRV_LRA);
	vibra_driver_write(DRV_VOLTAGE, DRV_2_0V);
	vibra_driver_write(DRV_CLAMP, DRV_2_0V);
	vibra_driver_write(DRV_GO, DRV_GO_BIT);

	/* wait for auto calibration to complete
	 * polling of driver does not work
	 */
	msleep(1000);
	/* set the driver in pwm mode */
	vibra_driver_write(DRV_MODE, DRV_PWM);
	gpio_set_value(info->gpio_pwm, 0);
	gpio_set_value(info->gpio_en, 0);
	return 0;
}

static int __devinit intel_mid_vibra_probe(struct pci_dev *pci,
			const struct pci_device_id *pci_id)
{
	struct vibra_info *info;
	struct mid_vibra_probe *data;
	int ret = 0;

	pr_debug("Probe for DID %x\n", pci->device);

	data = (void *)pci_id->driver_data;
	/* Override MRFLD settings for PRh */
	if ((INTEL_MID_BOARD(2, PHONE, MRFL, BB, PRO)) ||
	    (INTEL_MID_BOARD(2, PHONE, MRFL, BB, ENG))) {
			data->ext_drv = 0;
			data->time_divisor = 0xFF;
	}

	pr_debug("probe data divisor %x, base %x, alt_fn %d ext_drv %d",
			data->time_divisor, data->base_unit, data->alt_fn, data->ext_drv);

	info =  devm_kzalloc(&pci->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (pci->device == INTEL_VIBRA_CLV_PCI_ID) {
		info->gpio_en = INTEL_VIBRA_ENABLE_GPIO;
		info->gpio_pwm = INTEL_PWM_ENABLE_GPIO;
	} else if (pci->device == INTEL_VIBRA_MRFLD_PCI_ID) {
		info->gpio_en = get_gpio_by_name("haptics_en");
		info->gpio_pwm = get_gpio_by_name("haptics_pwm");
	}
	info->alt_fn = data->alt_fn;
	info->ext_drv = data->ext_drv;
	pr_debug("using gpios en: %d, pwm %d", info->gpio_en, info->gpio_pwm);
	ret = gpio_request_one(info->gpio_en, GPIOF_DIR_OUT, "VIBRA ENABLE");
	if (ret != 0) {
		pr_err("gpio_request(%d) fails:%d\n", info->gpio_en, ret);
		goto out;
	}

	ret = gpio_request_one(info->gpio_pwm, GPIOF_DIR_OUT, "PWM ENABLE");
	if (ret != 0) {
		pr_err("gpio_request(%d) fails:%d\n", info->gpio_pwm, ret);
		goto do_freegpio_vibra_enable;
	}

	/* Init the device */
	ret = pci_enable_device(pci);
	if (ret) {
		pr_err("device can't be enabled\n");
		goto do_freegpio_pwm;
	}
	ret = pci_request_regions(pci, INTEL_VIBRA_DRV_NAME);

	if (ret)
		goto do_disable_device;
	info->pci = pci_dev_get(pci);

	/* vibra Shim */
	info->shim =  pci_ioremap_bar(pci, 0);
	if (!info->shim) {
		pr_err("ioremap failed for vibra driver\n");
		goto do_release_regions;
	}
	pr_debug("Base reg: %x", pci_resource_start(pci, 0));

	/*set default value to driver data */
	info->pwm.part.pwmbu = data->base_unit;
	info->pwm.part.pwmtd = data->time_divisor;

	info->dev = &pci->dev;
	info->name = "intel_mid:vibrator";
	mutex_init(&info->lock);

	if (vibra_register_sysfs(info) < 0) {
		pr_err("could not register sysfs files\n");
		goto do_unmap_shim;
	}
	vibra_pwm_configure(info, true);

	if (info->ext_drv)
		vibra_init_ext_drv(info);

	pci_set_drvdata(pci, info);
	pm_runtime_allow(&pci->dev);
	pm_runtime_put_noidle(&pci->dev);
	return ret;

do_unmap_shim:
	iounmap(info->shim);
do_release_regions:
	pci_release_regions(pci);
do_disable_device:
	pci_disable_device(pci);
do_freegpio_pwm:
	gpio_free(info->gpio_pwm);
do_freegpio_vibra_enable:
	gpio_free(info->gpio_en);
out:
	return ret;
}

static void __devexit intel_mid_vibra_remove(struct pci_dev *pci)
{
	struct vibra_info *info = pci_get_drvdata(pci);
	gpio_free(info->gpio_pwm);
	gpio_free(info->gpio_en);
	vibra_unregister_sysfs(info);
	iounmap(info->shim);
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);
}

/* PCI Routines */
static DEFINE_PCI_DEVICE_TABLE(intel_vibra_ids) = {
	{ PCI_VDEVICE(INTEL, INTEL_VIBRA_CLV_PCI_ID),
		INFO(0xFF, 0x80, LNW_ALT_2, 0)},
	{ PCI_VDEVICE(INTEL, INTEL_VIBRA_MRFLD_PCI_ID),
		INFO(0x40, 0x80, LNW_ALT_1, 1)},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_vibra_ids);

static struct pci_driver vibra_driver = {
	.name = INTEL_VIBRA_DRV_NAME,
	.id_table = intel_vibra_ids,
	.probe = intel_mid_vibra_probe,
	.remove = __devexit_p(intel_mid_vibra_remove),
#ifdef CONFIG_PM
	.driver = {
		.pm = &intel_mid_vibra_pm_ops,
	},
#endif
};

/**
* intel_mid_vibra_init - Module init function
*
* Registers with PCI
* Registers with /dev
* Init all data strutures
*/
static int __init intel_mid_vibra_init(void)
{
	int ret = 0;

	/* Register with PCI */
	ret = pci_register_driver(&vibra_driver);
	if (ret)
		pr_err("PCI register failed\n");
	return ret;
}

/**
* intel_mid_vibra_exit - Module exit function
*
* Unregisters with PCI
* Unregisters with /dev
* Frees all data strutures
*/
static void __exit intel_mid_vibra_exit(void)
{
	pci_unregister_driver(&vibra_driver);
	pr_debug("intel_mid_vibra driver exited\n");
	return;
}

late_initcall(intel_mid_vibra_init);
module_exit(intel_mid_vibra_exit);

MODULE_ALIAS("pci:intel_mid_vibra");
MODULE_DESCRIPTION("Intel(R) MID Vibra driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("KP Jeeja <jeeja.kp@intel.com>");
