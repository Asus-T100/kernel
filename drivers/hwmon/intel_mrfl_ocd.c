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
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/rpmsg.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_basincove_ocd.h>
#include <asm/intel_mid_remoteproc.h>

#define DRIVER_NAME "bcove_bcu"

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
	struct platform_device *pdev;
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

static int program_bcu(void *ocd_smip_addr)
{
	int ret, i;
	u8 *smip_data;

	if (!ocd_smip_addr)
		return -ENXIO;

	smip_data = (u8 *)ocd_smip_addr;

	mutex_lock(&ocd_update_lock);

	for (i = 0; i < NUM_SMIP_BYTES-1 ; i++, smip_data++) {
		ret = intel_scu_ipc_iowrite8(VWARN1_CFG + i, *smip_data);
		if (ret)
			goto ipc_fail;
	}
	/* MBCUIRQ register address not consecutive with other BCU registers */
	ret = intel_scu_ipc_iowrite8(MBCUIRQ, *smip_data);

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

	if (kstrtoul(buf, 10, &curnt))
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

	if (kstrtoul(buf, 10, &volt))
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

	if (kstrtoul(buf, 10, &flag) || (flag != 0 && flag != 1))
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
	dev_err(cinfo->dev, "EM:BCU: ipc read/write failed:func:%s()\n",
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
	dev_err(cinfo->dev, "EM:BCU: ipc read/write failed:func:%s()\n",
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

	if (irq_data & VCRIT_IRQ) {
		event = CRIT;
		handle_VC_event(event, dev_data);
	}
	if (irq_data & VWARN2_IRQ) {
		event = WARN2;
		handle_VW2_event(event, dev_data);
	}
	if (irq_data & VWARN1_IRQ) {
		event = WARN1;
		handle_VW1_event(event, dev_data);
	}
	if (irq_data & GSMPULSE_IRQ) {
		event = GSMPULSE;
		dev_info(cinfo->dev, "EM:BCU: BCU Event %d has occured\n",
									event);
	}
	if (irq_data & TXPWRTH_IRQ) {
		event = TXPWRTH;
		dev_info(cinfo->dev, "EM:BCU: BCU Event %d has occured\n",
									event);
	}

	/* Unmask BCU Interrupt in the mask register */
	ret = intel_scu_ipc_update_register(MIRQLVL1, 0x00, BCU_ALERT);
	if (ret) {
		dev_err(cinfo->dev,
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

static int mrfl_ocd_probe(struct platform_device *pdev)
{
	int ret;
	struct ocd_platform_data *ocd_plat_data;
	struct ocd_bcove_config_data ocd_config_data;
	struct ocd_info *cinfo = kzalloc(sizeof(struct ocd_info), GFP_KERNEL);

	if (!cinfo) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}
	cinfo->pdev = pdev;
	cinfo->irq = platform_get_irq(pdev, 0);
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

	cinfo->bcu_intr_addr = ioremap_nocache(PMIC_SRAM_BCU_ADDR, IOMAP_LEN);
	if (!cinfo->bcu_intr_addr) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "ioremap_nocache failed\n");
		goto exit_hwmon;
	}

	/* Unmask 1st level BCU interrupt in the mask register */
	ret = intel_scu_ipc_update_register(MIRQLVL1, 0x00, BCU_ALERT);
	if (ret) {
		dev_err(&pdev->dev,
			"EM:BCU: Unmasking of BCU failed:%d\n", ret);
		goto exit_ioremap;
	}

	/* Register for Interrupt Handler */
	ret = request_threaded_irq(cinfo->irq, NULL,
						ocd_intrpt_thread_handler,
						IRQF_TRIGGER_RISING,
						DRIVER_NAME, cinfo);
	if (ret) {
		dev_err(&pdev->dev,
			"EM:BCU: request_threaded_irq failed:%d\n", ret);
		goto exit_ioremap;
	}

	/*Read BCU configuration values from smip*/
	ocd_plat_data = pdev->dev.platform_data;

	ret = ocd_plat_data->bcu_config_data(&ocd_config_data);
	if (ret) {
		dev_err(&pdev->dev, "EM:BCU:Read SMIP failed:%d\n", ret);
		goto exit_freeirq;
	}

	/* Program the BCU with default values read from the smip*/
	ret = program_bcu(&ocd_config_data);
	if (ret) {
		dev_err(&pdev->dev, "EM:BCU:program_bcu() failed:%d\n", ret);
		goto exit_freeirq;
	}

	enable_volt_trip_points();
	enable_current_trip_points();

	return 0;

exit_freeirq:
	free_irq(cinfo->irq, cinfo);
exit_ioremap:
	iounmap(cinfo->bcu_intr_addr);
exit_hwmon:
	hwmon_device_unregister(cinfo->dev);
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
		free_irq(cinfo->irq, cinfo);
		iounmap(cinfo->bcu_intr_addr);
		hwmon_device_unregister(cinfo->dev);
		sysfs_remove_group(&pdev->dev.kobj, &mrfl_ocd_gr);
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

static const struct platform_device_id mrfl_ocd_table[] = {
	{DRIVER_NAME, 1 },
};

static struct platform_driver mrfl_over_curr_detect_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &mrfl_ocd_pm_ops,
		},
	.probe = mrfl_ocd_probe,
	.remove = __devexit_p(mrfl_ocd_remove),
	.id_table = mrfl_ocd_table,
};

static int mrfl_ocd_module_init(void)
{
	return platform_driver_register(&mrfl_over_curr_detect_driver);
}

static void mrfl_ocd_module_exit(void)
{
	platform_driver_unregister(&mrfl_over_curr_detect_driver);
}

/* RPMSG related functionality */

static int mrfl_ocd_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;
	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed mrfl_ocd rpmsg device\n");

	ret = mrfl_ocd_module_init();
out:
	return ret;
}

static void mrfl_ocd_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	mrfl_ocd_module_exit();
	dev_info(&rpdev->dev, "Removed mrfl_ocd rpmsg device\n");
}

static void mrfl_ocd_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
			int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
				data, len, true);
}

static struct rpmsg_device_id mrfl_ocd_id_table[] = {
	{ .name = "rpmsg_mrfl_ocd" },
	{ },
};

MODULE_DEVICE_TABLE(rpmsg, mrfl_ocd_id_table);

static struct rpmsg_driver mrfl_ocd_rpmsg = {
	.drv.name	= DRIVER_NAME,
	.drv.owner	= THIS_MODULE,
	.probe		= mrfl_ocd_rpmsg_probe,
	.callback	= mrfl_ocd_rpmsg_cb,
	.remove		= mrfl_ocd_rpmsg_remove,
	.id_table	= mrfl_ocd_id_table,
};

static int __init mrfl_ocd_rpmsg_init(void)
{
	return register_rpmsg_driver(&mrfl_ocd_rpmsg);
}

static void __init mrfl_ocd_rpmsg_exit(void)
{
	unregister_rpmsg_driver(&mrfl_ocd_rpmsg);
}

module_init(mrfl_ocd_rpmsg_init);
module_exit(mrfl_ocd_rpmsg_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Over Current Detection Driver");
MODULE_LICENSE("GPL");
