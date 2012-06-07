/*
 * intel_mrfl_ocd.c - Intel Merrifield Platform Over Current Detection Driver
 *
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This driver monitors the voltage level of the system. When the voltage
 * drops below a programmed threshold, it notifies the CPU of the drop.
 * Also, the driver configures the HW to take some actions to prevent
 * system crash due to sudden drop in voltage.
 * DEVICE_NAME: Intel Merrifield platform - PMIC: Burst Control Unit
 */

#define pr_fmt(fmt)  "intel_mrfl_ocd: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <asm/intel_scu_ipc.h>

#define DRIVER_NAME "mrfl_ocd"
#define DEVICE_NAME "mrfl_pmic_bcu"

/* Configuration registers that monitor the voltage drop */
#define VWARN1_CFG		0x3C
#define VWARN2_CFG		0x3D
#define VCRIT_CFG		0x3E
#define MAXVSYS_CFG		0x3F
#define MAXVCC_CFG		0x40
#define MAXVNN_CFG		0x41

/* Behaviour registers */
#define VFLEXSRC_BEH		0x42
#define VFLEXDIS_BEH		0x43
#define VIBDIS_BEH		0x44
#define CAMTORCH_BEH		0x45
#define CAMFLDIS_BEH		0x46
#define BCUDISW2_BEH		0x47
#define BCUDISCRIT_BEH		0x48

/* IRQ registers */
#define BCUIRQ			0x05
#define MBCUIRQ			0x10

/* Status registers */
#define S_BCUINT		0x3B
#define S_BCUCTRL		0x49

#define NUM_VOLT_LEVELS		3
#define NUM_CURR_LEVELS		2

#define ICCMAX_EN		(1 << 6)
#define VWARN_EN		(1 << 3)
#define VCRIT_SHUTDOWN		(1 << 4)

/* Number of configurable thresholds for current and voltage */
#define NUM_THRESHOLDS		8

static DEFINE_MUTEX(ocd_update_lock);

/* Warning levels for Voltage (in mV) */
static const unsigned long volt_thresholds[NUM_THRESHOLDS] = {
			2550, 2600, 2700, 2750, 2800, 2900, 3000, 3100 };

/* Warning levels for Current (in mA) */
static const unsigned long curr_thresholds[NUM_THRESHOLDS] = {
			1600, 2000, 2400, 2600, 3000, 3200, 3400, 3600 };

struct ocd_info {
	struct device *dev;
	struct platform_device *pdev;
};

static void enable_volt_trip_points(void)
{
	int i, ret;
	uint8_t data;

	/*
	 * Enable the Voltage comparator logic, so that the output
	 * signals are asserted when a voltage drop occurs.
	 */
	for (i = 0; i < NUM_VOLT_LEVELS; i++) {
		ret = intel_scu_ipc_ioread8(VWARN1_CFG + i, &data);
		if (!ret)
			intel_scu_ipc_iowrite8(VWARN1_CFG + i,
						(data | VWARN_EN));
	}
}

static void enable_current_trip_points(void)
{
	int i, ret;
	uint8_t data;

	/*
	 * Enable the Current comparator logic, so that the output
	 * signals are asserted when the platform current surges.
	 */
	for (i = 0; i < NUM_CURR_LEVELS; i++) {
		ret = intel_scu_ipc_ioread8(MAXVCC_CFG + i, &data);
		if (!ret)
			intel_scu_ipc_iowrite8(MAXVCC_CFG + i,
						(data | ICCMAX_EN));
	}
}

static int find_threshold(const unsigned long *arr,
				unsigned long value)
{
	int pos = 0;

	if (value < arr[0] || value > arr[NUM_THRESHOLDS - 1])
		return -EINVAL;

	/* Find the index of 'value' in the thresholds array */
	while (pos < NUM_THRESHOLDS && value >= arr[pos])
		++pos;

	return pos - 1;
}

static int set_threshold(u16 reg_addr, int pos)
{
	int ret;
	uint8_t data;

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(reg_addr, &data);
	if (ret)
		goto ipc_fail;

	/* Set bits [0-2] to value of pos */
	data = (data & 0xF8) | pos;

	ret = intel_scu_ipc_iowrite8(reg_addr, data);

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static ssize_t store_curr_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long curnt;
	int pos, ret;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	if (strict_strtoul(buf, 10, &curnt))
		return -EINVAL;

	pos = find_threshold(curr_thresholds, curnt);
	if (pos < 0)
		return -EINVAL;

	/*
	 * Since VCC_CFG and VNN_CFG are consecutive registers, calculate the
	 * required register address using s_attr->nr.
	 */
	ret = set_threshold(MAXVCC_CFG + s_attr->nr, pos);

	return ret ? ret : count;
}

static ssize_t show_curr_thres(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t data;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(MAXVCC_CFG + s_attr->nr, &data);

	mutex_unlock(&ocd_update_lock);

	if (ret)
		return ret;

	/* Read bits [0-2] of data to get the index into the array */
	return sprintf(buf, "%lu\n", curr_thresholds[data & 0x07]);
}

static ssize_t store_volt_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long volt;
	int pos, ret;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	if (strict_strtoul(buf, 10, &volt))
		return -EINVAL;

	pos = find_threshold(volt_thresholds, volt);
	if (pos < 0)
		return -EINVAL;

	/*
	 * The voltage thresholds are in descending order in VWARN*_CFG
	 * registers. So calculate 'pos' by substracting from NUM_THRESHOLDS.
	 */
	pos = NUM_THRESHOLDS - pos - 1;

	/*
	 * Since VWARN*_CFG are consecutive registers, calculate the
	 * required register address using s_attr->nr.
	 */
	ret = set_threshold(VWARN1_CFG + s_attr->nr, pos);

	return ret ? ret : count;
}

static ssize_t show_volt_thres(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, index;
	uint8_t data;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(VWARN1_CFG + s_attr->nr, &data);

	mutex_unlock(&ocd_update_lock);

	if (ret)
		return ret;

	/* Read bits [0-2] of data to get the index into the array */
	index = NUM_THRESHOLDS - (data & 0x07) - 1;

	return sprintf(buf, "%lu\n", volt_thresholds[index]);
}

static ssize_t store_crit_shutdown(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint8_t data;
	unsigned long flag;

	if (strict_strtoul(buf, 10, &flag) || (flag != 0 && flag != 1))
		return -EINVAL;

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(VCRIT_CFG, &data);
	if (ret)
		goto ipc_fail;
	/*
	 * flag:1 enables shutdown due to burst current
	 * flag:0 disables shutdown due to burst current
	 */
	if (flag)
		data |= VCRIT_SHUTDOWN;
	else
		data &= ~VCRIT_SHUTDOWN;

	ret = intel_scu_ipc_iowrite8(VCRIT_CFG, data);
	if (!ret)
		ret = count;

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static ssize_t show_crit_shutdown(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int flag, ret;
	uint8_t data;

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(VCRIT_CFG, &data);
	if (!ret) {
		/* 'flag' is 1 if CRIT_SHUTDOWN is enabled, 0 otherwise */
		flag = !!(data & VCRIT_SHUTDOWN);
	}

	mutex_unlock(&ocd_update_lock);

	return ret ? ret : sprintf(buf, "%d\n", flag);
}

static SENSOR_DEVICE_ATTR_2(volt_warn1, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 0, 0);
static SENSOR_DEVICE_ATTR_2(volt_warn2, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 1, 0);
static SENSOR_DEVICE_ATTR_2(volt_crit, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 2, 0);

static SENSOR_DEVICE_ATTR_2(core_current, S_IRUGO | S_IWUSR,
				show_curr_thres, store_curr_thres, 0, 0);
static SENSOR_DEVICE_ATTR_2(uncore_current, S_IRUGO | S_IWUSR,
				show_curr_thres, store_curr_thres, 1, 0);

static SENSOR_DEVICE_ATTR_2(enable_crit_shutdown, S_IRUGO | S_IWUSR,
				show_crit_shutdown, store_crit_shutdown, 0, 0);

static struct attribute *mrfl_ocd_attrs[] = {
	&sensor_dev_attr_core_current.dev_attr.attr,
	&sensor_dev_attr_uncore_current.dev_attr.attr,
	&sensor_dev_attr_volt_warn1.dev_attr.attr,
	&sensor_dev_attr_volt_warn2.dev_attr.attr,
	&sensor_dev_attr_volt_crit.dev_attr.attr,
	&sensor_dev_attr_enable_crit_shutdown.dev_attr.attr,
	NULL
};

static struct attribute_group mrfl_ocd_gr = {
	.name = "mrfl_current",
	.attrs = mrfl_ocd_attrs
};

static int mrfl_ocd_probe(struct platform_device *pdev)
{
	int ret;
	struct ocd_info *cinfo = kzalloc(sizeof(struct ocd_info), GFP_KERNEL);

	if (!cinfo) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	cinfo->pdev = pdev;
	platform_set_drvdata(pdev, cinfo);

	/* Creating a sysfs group with mrfl_ocd_gr attributes */
	ret = sysfs_create_group(&pdev->dev.kobj, &mrfl_ocd_gr);
	if (ret) {
		dev_err(&pdev->dev, "sysfs create group failed\n");
		goto exit_free;
	}

	/* Registering with hwmon class */
	cinfo->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(cinfo->dev)) {
		ret = PTR_ERR(cinfo->dev);
		cinfo->dev = NULL;
		dev_err(&pdev->dev, "hwmon_dev_regs failed\n");
		goto exit_sysfs;
	}

	enable_volt_trip_points();
	enable_current_trip_points();

	return 0;

exit_sysfs:
	sysfs_remove_group(&pdev->dev.kobj, &mrfl_ocd_gr);
exit_free:
	kfree(cinfo);
	return ret;
}

static int mrfl_ocd_resume(struct device *dev)
{
	dev_info(dev, "Resume called.\n");
	return 0;
}

static int mrfl_ocd_suspend(struct device *dev)
{
	dev_info(dev, "Suspend called.\n");
	return 0;
}

static int mrfl_ocd_remove(struct platform_device *pdev)
{
	struct ocd_info *cinfo = platform_get_drvdata(pdev);

	if (cinfo) {
		hwmon_device_unregister(cinfo->dev);
		sysfs_remove_group(&pdev->dev.kobj, &mrfl_ocd_gr);
		kfree(cinfo);
	}
	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/
static const struct platform_device_id ocd_id_table[] = {
	{ DEVICE_NAME, 1 },
	{ },
};

static const struct dev_pm_ops mrfl_ocd_pm_ops = {
	.suspend = mrfl_ocd_suspend,
	.resume = mrfl_ocd_resume,
};

static struct platform_driver mrfl_over_curr_detect_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &mrfl_ocd_pm_ops,
		},
	.probe = mrfl_ocd_probe,
	.remove = __devexit_p(mrfl_ocd_remove),
	.id_table = ocd_id_table,
};

static int __init mrfl_ocd_module_init(void)
{
	int ret;
	struct platform_device *pdev;

	/*
	 * FIXME: When platform specific initialization support is
	 * available, the 'device_alloc' and 'device_add' calls will
	 * be removed.
	 */
	pdev = platform_device_alloc(DEVICE_NAME, 0);
	if (!pdev)
		return PTR_ERR(pdev);

	ret = platform_device_add(pdev);
	if (ret) {
		kfree(pdev);
		return ret;
	}

	return platform_driver_register(&mrfl_over_curr_detect_driver);
}

static void __exit mrfl_ocd_module_exit(void)
{
	platform_driver_unregister(&mrfl_over_curr_detect_driver);
}

module_init(mrfl_ocd_module_init);
module_exit(mrfl_ocd_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Over Current Detection Driver");
MODULE_LICENSE("GPL");
