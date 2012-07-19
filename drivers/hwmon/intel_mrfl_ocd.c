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
#include <linux/ipc_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <asm/intel_scu_ipc.h>

#define DRIVER_NAME "bcove_bcu"
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
#define IRQLVL1			0x01
#define MIRQLVL1		0x0C

/* Status registers */
#define S_BCUINT		0x3B
#define S_BCUCTRL		0x49

/* PMIC SRAM address for BCU register */
#define PMIC_SRAM_BCU_ADDR	0xFFFFF614
#define IOMAP_LEN		4

#define NUM_VOLT_LEVELS		3
#define NUM_CURR_LEVELS		2

#define ICCMAX_EN		(1 << 6)
#define VWARN_EN		(1 << 3)
#define VCRIT_SHUTDOWN		(1 << 4)

#define BCU_ALERT		(1 << 3)
#define VWARN1_IRQ		(1 << 0)
#define VWARN2_IRQ		(1 << 1)
#define VCRIT_IRQ		(1 << 2)
#define GSMPULSE_IRQ		(1 << 3)
#define TXPWRTH_IRQ		(1 << 4)

/* Number of configurable thresholds for current and voltage */
#define NUM_THRESHOLDS		8

/* BCU real time status flags for corresponding input signals */
#define SVWARN1			(1<<0)
#define SVWARN2			(1<<1)
#define SVCRIT			(1<<2)

/* S_BCUCTRL register status bits */
#define SBCUCTRL_CAMTORCH	(1<<3)
#define SBCUCTRL_CAMFLDIS	(1<<2)
#define SBCUCTRL_BCUDISW2       (1<<1)

/* check whether bit is sticky or not by checking 5th bit */
#define IS_STICKY(data)         (!!(data & 0x10))

/* check whether signal asserted for VW1/VW2/VC */
#define IS_ASSRT_ON_VW1(data)	(!!(data & 0x01))
#define IS_ASSRT_ON_VW2(data)	(!!(data & 0x02))
#define IS_ASSRT_ON_VC(data)	(!!(data & 0x04))

/* 'enum' of BCU events */
enum bcu_events { WARN1, WARN2, CRIT, GSMPULSE, TXPWRTH, UNKNOWN, __COUNT };

static DEFINE_MUTEX(ocd_update_lock);

/* Warning levels for Voltage (in mV) */
static const unsigned long volt_thresholds[NUM_THRESHOLDS] = {
			2550, 2600, 2700, 2750, 2800, 2900, 3000, 3100 };

/* Warning levels for Current (in mA) */
static const unsigned long curr_thresholds[NUM_THRESHOLDS] = {
			1600, 2000, 2400, 2600, 3000, 3200, 3400, 3600 };

struct ocd_info {
	struct device *dev;
	struct ipc_device *ipcdev;
	void *bcu_intr_addr;
	int irq;
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

static void handle_VW1_event(int event, void *dev_data)
{
	uint8_t irq_status, beh_data;
	struct ocd_info *cinfo = (struct ocd_info *)dev_data;
	int ret;

	dev_info(cinfo->dev, "EM:BCU: BCU Event %d has occured\n", event);
	/* Notify using UEvent */
	kobject_uevent(&cinfo->dev->kobj, KOBJ_CHANGE);

	ret = intel_scu_ipc_ioread8(S_BCUINT, &irq_status);
	if (ret)
		goto ipc_fail;
	dev_dbg(cinfo->dev, "EM:BCU: S_BCUINT: %x\n", irq_status);

	/* If Vsys is below WARN1 level-No action required from driver */
	if (!(irq_status & SVWARN1)) {
		/* Vsys is above WARN1 level */
		ret = intel_scu_ipc_ioread8(CAMTORCH_BEH, &beh_data);
		if (ret)
			goto ipc_fail;

		if (IS_ASSRT_ON_VW1(beh_data) && IS_STICKY(beh_data)) {
			ret = intel_scu_ipc_update_register(S_BCUCTRL,
				0xFF, SBCUCTRL_CAMTORCH);
			if (ret)
				goto ipc_fail;
		}
	}
	return;

ipc_fail:
	dev_err(&cinfo->dev, "EM:BCU: ipc read/write failed:func:%s()\n",
								__func__);
	return;
}

static void handle_VW2_event(int event, void *dev_data)
{
	uint8_t irq_status, beh_data;
	struct ocd_info *cinfo = (struct ocd_info *)dev_data;
	int ret;

	dev_info(cinfo->dev, "EM:BCU: BCU Event %d has occured\n", event);
	/* Notify using UEvent */
	kobject_uevent(&cinfo->dev->kobj, KOBJ_CHANGE);

	ret = intel_scu_ipc_ioread8(S_BCUINT, &irq_status);
	if (ret)
		goto ipc_fail;
	dev_dbg(cinfo->dev, "EM:BCU: S_BCUINT: %x\n", irq_status);

	/* If Vsys is below WARN2 level-No action required from driver */
	if (!(irq_status & SVWARN2)) {
		/* Vsys is above WARN2 level */
		ret = intel_scu_ipc_ioread8(CAMFLDIS_BEH, &beh_data);
		if (ret)
			goto ipc_fail;

		if (IS_ASSRT_ON_VW2(beh_data) && IS_STICKY(beh_data)) {
			ret = intel_scu_ipc_update_register(S_BCUCTRL,
						0xFF, SBCUCTRL_CAMFLDIS);
			if (ret)
				goto ipc_fail;
		}

		ret = intel_scu_ipc_ioread8(BCUDISW2_BEH, &beh_data);
		if (ret)
			goto ipc_fail;

		if (IS_ASSRT_ON_VW2(beh_data) && IS_STICKY(beh_data)) {
			ret = intel_scu_ipc_update_register(S_BCUCTRL,
				0xFF, SBCUCTRL_BCUDISW2);
			if (ret)
				goto ipc_fail;
		}
	}
	return;

ipc_fail:
	dev_err(&cinfo->dev, "EM:BCU: ipc read/write failed:func:%s()\n",
								__func__);
	return;
}

static void handle_VC_event(int event, void *dev_data)
{
	struct ocd_info *cinfo = (struct ocd_info *)dev_data;

	dev_info(cinfo->dev, "EM:BCU: BCU Event %d has occured\n", event);
	/* Notify using UEvent */
	kobject_uevent(&cinfo->dev->kobj, KOBJ_CHANGE);

	return;
}



static irqreturn_t ocd_intrpt_thread_handler(int irq, void *dev_data)
{
	int ret, event;
	unsigned int irq_data;
	struct ocd_info *cinfo = (struct ocd_info *)dev_data;

	if (!cinfo)
		return IRQ_NONE;

	mutex_lock(&ocd_update_lock);

	irq_data = ioread8(cinfo->bcu_intr_addr);

	/* we are not handling(no action taken) GSMPULSE_IRQ and
						TXPWRTH_IRQ event */
	if (irq_data & VWARN1_IRQ) {
		event = WARN1;
		handle_VW1_event(event, dev_data);
	} else if (irq_data & VWARN2_IRQ) {
		event = WARN2;
		handle_VW2_event(event, dev_data);
	} else if (irq_data & VCRIT_IRQ) {
		event = CRIT;
		handle_VC_event(event, dev_data);
	} else if (irq_data & GSMPULSE_IRQ) {
		event = GSMPULSE;
		dev_info(cinfo->dev, "EM:BCU: BCU Event %d has occured\n",
									event);
	} else if (irq_data & TXPWRTH_IRQ) {
		event = TXPWRTH;
		dev_info(cinfo->dev, "EM:BCU: BCU Event %d has occured\n",
									event);
	} else {
		event = UNKNOWN;
		dev_err(cinfo->dev, "EM:BCU: Invalid Interrupt\n");
	}

	/* Unmask BCU Interrupt in the mask register */
	ret = intel_scu_ipc_update_register(MIRQLVL1, 0x00, BCU_ALERT);
	if (ret) {
		dev_err(&cinfo->dev,
			"EM:BCU: Unmasking of BCU failed:%d\n", ret);
		goto ipc_fail;
	}

	ret = IRQ_HANDLED;

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
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

static int mrfl_ocd_probe(struct ipc_device *ipcdev)
{
	int ret;
	struct ocd_info *cinfo = kzalloc(sizeof(struct ocd_info), GFP_KERNEL);

	if (!cinfo) {
		dev_err(&ipcdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	cinfo->ipcdev = ipcdev;
	cinfo->irq = ipc_get_irq(ipcdev, 0);
	ipc_set_drvdata(ipcdev, cinfo);

	/* Creating a sysfs group with mrfl_ocd_gr attributes */
	ret = sysfs_create_group(&ipcdev->dev.kobj, &mrfl_ocd_gr);
	if (ret) {
		dev_err(&ipcdev->dev, "sysfs create group failed\n");
		goto exit_free;
	}

	/* Registering with hwmon class */
	cinfo->dev = hwmon_device_register(&ipcdev->dev);
	if (IS_ERR(cinfo->dev)) {
		ret = PTR_ERR(cinfo->dev);
		cinfo->dev = NULL;
		dev_err(&ipcdev->dev, "hwmon_dev_regs failed\n");
		goto exit_sysfs;
	}

	cinfo->bcu_intr_addr = ioremap_nocache(PMIC_SRAM_BCU_ADDR, IOMAP_LEN);
	if (!cinfo->bcu_intr_addr) {
		ret = -ENOMEM;
		dev_err(&ipcdev->dev, "ioremap_nocache failed\n");
		goto exit_hwmon;
	}

	/* Unmask 2nd level BCU Interrupts-VW1,VW2&VC in the mask register */
	ret = intel_scu_ipc_update_register(MBCUIRQ,
				0x00, VWARN1_IRQ | VWARN2_IRQ | VCRIT_IRQ);
	if (ret) {
		dev_err(&ipcdev->dev,
			"EM:BCU: Unmasking of VW1 failed:%d\n", ret);
		goto exit_ioremap;
	}

	/* Unmask 1st level BCU interrupt in the mask register */
	ret = intel_scu_ipc_update_register(MIRQLVL1, 0x00, BCU_ALERT);
	if (ret) {
		dev_err(&ipcdev->dev,
			"EM:BCU: Unmasking of BCU failed:%d\n", ret);
		goto exit_ioremap;
	}

	/* Register for Interrupt Handler */
	ret = request_threaded_irq(cinfo->irq, NULL,
						ocd_intrpt_thread_handler,
						IRQF_TRIGGER_RISING,
						DRIVER_NAME, cinfo);
	if (ret) {
		dev_err(&ipcdev->dev,
			"EM:BCU: request_threaded_irq failed:%d\n", ret);
		goto exit_ioremap;
	}

	enable_volt_trip_points();
	enable_current_trip_points();

	return 0;

exit_ioremap:
	iounmap(cinfo->bcu_intr_addr);
exit_hwmon:
	hwmon_device_unregister(cinfo->dev);
exit_sysfs:
	sysfs_remove_group(&ipcdev->dev.kobj, &mrfl_ocd_gr);
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

static int mrfl_ocd_remove(struct ipc_device *ipcdev)
{
	struct ocd_info *cinfo = ipc_get_drvdata(ipcdev);

	if (cinfo) {
		free_irq(cinfo->irq, cinfo);
		iounmap(cinfo->bcu_intr_addr);
		hwmon_device_unregister(cinfo->dev);
		sysfs_remove_group(&ipcdev->dev.kobj, &mrfl_ocd_gr);
		kfree(cinfo);
	}
	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static const struct dev_pm_ops mrfl_ocd_pm_ops = {
	.suspend = mrfl_ocd_suspend,
	.resume = mrfl_ocd_resume,
};

static struct ipc_driver mrfl_over_curr_detect_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &mrfl_ocd_pm_ops,
		},
	.probe = mrfl_ocd_probe,
	.remove = __devexit_p(mrfl_ocd_remove),
};

static int __init mrfl_ocd_module_init(void)
{
	return ipc_driver_register(&mrfl_over_curr_detect_driver);
}

static void __exit mrfl_ocd_module_exit(void)
{
	ipc_driver_unregister(&mrfl_over_curr_detect_driver);
}

module_init(mrfl_ocd_module_init);
module_exit(mrfl_ocd_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Over Current Detection Driver");
MODULE_LICENSE("GPL");
