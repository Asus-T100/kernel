/*
 * intel_mid_vdd.c - Intel Clovertrail Platform Voltage Drop Detection Driver
 *
 *
 * Copyright (C) 2012 Intel Corporation
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
 *         Chaurasia, Avinash K <avinash.k.chaurasia@intel.com>
 *
 * This driver monitors the voltage level of the system. When the system
 * voltage cross the programmed threshold, it notifies the CPU of the
 * change. Also, the driver configures the HW to take some actions to prevent
 * system crash due to sudden drop in voltage. The HW unit that does all
 * these, is named as Burst Control Unit(BCU).
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/ipc_device.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/timer.h>
#include <asm/intel_scu_ipc.h>
#include <linux/kfifo.h>

#define DRIVER_NAME "msic_vdd"
#define DEVICE_NAME "msic_vdd"

/* BCU registers that control the behaviour of output signals */
#define BCUDISA_BEH		0x10C
#define BCUDISB_BEH		0x10D
#define BCUDISCRIT_BEH		0x10E
#define BCUPROCHOT_BEH		0x10F

/* BCU Status registers */
#define SBCUIRQ			0x110
#define SBCUCTRL		0x111
#define SBCUCTRL_CLEAR_ASSERT	0x02
#define SBCUCTRL_STICKY_WARNB	0x04
#define SBCUCTRL_STICKY_WARNA	0x08

#define IRQLVL1MSK		0x021

/* Voltage limits (in mV) */
#define MAX_VOLTAGE		3300
#define MIN_VOLTAGE		2600

/* BCU Interrupt registers */
#define BCUIRQ			0x00A
#define MBCUIRQ			0x019
/* BCUIRQ register settings */
#define VCRIT_IRQ		(1 << 2)
#define VWARNA_IRQ		(1 << 1)
#define VWARNB_IRQ		(1 << 0)

/* BCU registers that govern voltage monitoring */
#define VWARNA_CFG		0x109
#define VWARNB_CFG		0x10A
#define VCRIT_CFG		0x10B

/* Enable BCU voltage comparator */
#define VCOMP_ENABLE		(1 << 3)

/* BCU real time status flags for corresponding signals */
#define SVWARNB			(1<<0)
#define SVWARNA			(1<<1)
#define SVCRIT			(1<<2)

/* curently 20 fast clock is set */
#define DEBOUNCE		(0x70)

/* voltage threshold to be set for different theshold points */
#define VWARNA_VOLT_THRES	0x04 /* 2.9V */
#define VWARNB_VOLT_THRES	0x04 /* 2.9V */
#define VCRIT_VOLT_THRES	0x01 /* 3.2V */

/*
 * this flag has been used in lot of places to update the proper bit
 * in registers
 */
#define FLAGS_THRES_REGS	0xF7

/* unmask the 2nd level interrupts as by default they are masked */
#define UNMASK_MBCUIRQ		0x00

/* check whether bit is sticky or not by checking 3rd bit */
#define IS_STICKY(data)		(data & 0x04)

#define SET_ACTION_MASK(data, value, bit) (data | (value << bit))

/* This macro is used in hardcoding of gpio */
#define MSIC_VDD		0x24

/* defines reading BCU registers from SRAM */
#define MSIC_BCU_STAT		0xFFFFEFC8
#define MSIC_BCU_LEN		1
#define IRQ_FIFO_MAX		16
#define IRQ_KFIFO_ELEMENT	1

#define BCU_SMIP_OFFSET		0x5A5

/* default values for register configuration */
#define INIT_VWARNA		(VWARNA_VOLT_THRES | DEBOUNCE | VCOMP_ENABLE)
#define INIT_VWARNB		(VWARNB_VOLT_THRES | DEBOUNCE | VCOMP_ENABLE)
#define INIT_VCRIT		(VCRIT_VOLT_THRES | DEBOUNCE | VCOMP_ENABLE)
#define INIT_BCUDISA		0x05
#define INIT_BCUDISB		0x05
#define INIT_BCUISCRIT		0x00
#define INIT_BCUPROCHOT		0x06
#define	INIT_MBCU		0x00

/* Defined to match the correponding bit positions of the interrupt */
enum { VWARNB_EVENT = 1, VWARNA_EVENT = 2, VCRIT_EVENT = 4};

static DEFINE_MUTEX(vdd_update_lock);

/* defining the fifo to store the interrupt value */
static DEFINE_KFIFO(irq_fifo, u8, IRQ_FIFO_MAX);

struct vdd_info {
	unsigned int irq;
	struct device *dev;
	struct ipc_device *pdev;
	/* mapping SRAM address for BCU interrupts */
	void __iomem *bcu_intr_addr;
};

struct vdd_smip_data {
	u8 vwarna_cfg;
	u8 vwarnb_cfg;
	u8 vcrit_cfg;
	u8 bcudisa_beh;
	u8 bcudisb_beh;
	u8 bcudiscrit_beh;
	u8 bcuprochot_beh;
	u8 mbcu_irq;
	/* 12 bits are reserved for bcu*/
};

/**
  * vdd_set_bits - set bit in register corresponding to mask value
  * @addr - address of register where changes are to be done
  * @mask - mask value according to which value will be set
  */
static inline int vdd_set_bits(u16 addr, u8 mask)
{
	return intel_scu_ipc_update_register(addr, 0xff, mask);
}

/**
  * vdd_set_register - initialize register with value
  * @addr - address of register
  * @value - value to be intialized with
  */
static inline int vdd_set_register(u16 addr, u8 value)
{
	return intel_scu_ipc_update_register(addr, value, 0xff);
}

/* --- Sysfs Implementation --- */
static ssize_t store_action_mask(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long assert_pin;
	if (strict_strtol(buf, 10, &assert_pin))
		return -EINVAL;

	/*
	 * 0x1F represents five output mask bit for 4 different registers
	 * when value is 0x1F then all the output pins will be masked and
	 * and when its 0x00 then it means all the output pins are unmasked
	 * and if interrupt occurs then all the output pins will be trigerred
	 * according to settings
	 */
	if (assert_pin < 0x00 || assert_pin > 0x1F)
		return -EINVAL;

	mutex_lock(&vdd_update_lock);
	ret = intel_scu_ipc_update_register(BCUDISA_BEH,
		((assert_pin >> 0) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUDISB_BEH,
		((assert_pin >> 1) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUDISCRIT_BEH,
		((assert_pin >> 2) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUPROCHOT_BEH,
		((assert_pin >> 3) & 1)<<1, 0x02);
	if (ret)
		goto mask_ipc_fail;
	ret = intel_scu_ipc_update_register(BCUPROCHOT_BEH,
		((assert_pin >> 4) & 1), 0x01);
	if (ret)
		goto mask_ipc_fail;
	ret = count;
mask_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;

}

static ssize_t show_action_mask(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t data;
	long assert_pin = 0;

	mutex_lock(&vdd_update_lock);
	ret = intel_scu_ipc_ioread8(BCUDISA_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 0);

	ret = intel_scu_ipc_ioread8(BCUDISB_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 1);

	ret = intel_scu_ipc_ioread8(BCUDISCRIT_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 2);

	ret = intel_scu_ipc_ioread8(BCUPROCHOT_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, ((data>>1) & 1), 3);

	ret = intel_scu_ipc_ioread8(BCUPROCHOT_BEH, &data);
	if (ret)
		goto mask_ipc_fail;
	assert_pin = SET_ACTION_MASK(assert_pin, (data & 1), 4);
mask_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return sprintf(buf, "%ld\n", assert_pin);
}

static ssize_t store_bcu_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	long bcu_enable;

	if (strict_strtol(buf, 10, &bcu_enable))
		return -EINVAL;
	if (bcu_enable != 0 &&  bcu_enable != 1)
		return -EINVAL;

	mutex_lock(&vdd_update_lock);

	/*
	 * bcu_enable = 1 means enable bcu, to do this it need to set 3rd
	 * bit from right side so mask will be 0x08
	 */
	ret = intel_scu_ipc_update_register(VWARNA_CFG,
			(bcu_enable << 3), 0x08);
	if (ret)
		goto bcu_ipc_fail;
	ret = intel_scu_ipc_update_register(VWARNB_CFG,
			(bcu_enable << 3), 0x08);
	if (ret)
		goto bcu_ipc_fail;
	ret = intel_scu_ipc_update_register(VCRIT_CFG,
			(bcu_enable << 3), 0x08);
	if (ret)
		goto bcu_ipc_fail;

	ret = count;
bcu_ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;
}

static ssize_t show_bcu_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, bcu_enable;
	uint8_t data;

	ret = intel_scu_ipc_ioread8(VWARNA_CFG, &data);
	if (ret)
		return ret;
	/* if BCU is enabled then return value will be 1 else 0 */
	bcu_enable = ((data >> 3) & 0x01);
	return sprintf(buf, "%d\n", bcu_enable);
}

static ssize_t store_volt_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint8_t data;
	long volt;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	if (strict_strtol(buf, 10, &volt))
		return -EINVAL;

	if (volt > MAX_VOLTAGE || volt < MIN_VOLTAGE)
		return -EINVAL;

	mutex_lock(&vdd_update_lock);

	ret = intel_scu_ipc_ioread8(VWARNA_CFG + s_attr->nr, &data);
	if (ret)
		goto ipc_fail;

	/*
	 * The VWARN*_CFG registers hold 7 voltage values, in [0-2]
	 * bits. 3.3v corresponds to 0 and 2.6v corresponds to 7. So,
	 * find the difference(diff) from MAX_VOLTAGE and divide it by
	 * 100(since the values are entered as mV). Then, set bits
	 * [0-2] to 'diff'
	 */
	data = (data & 0xF8) | ((MAX_VOLTAGE - volt)/100);

	ret = intel_scu_ipc_iowrite8(VWARNA_CFG + s_attr->nr, data);
	if (ret)
		goto ipc_fail;

	ret = count;

ipc_fail:
	mutex_unlock(&vdd_update_lock);
	return ret;
}

static ssize_t show_volt_thres(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, volt;
	uint8_t data;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	ret = intel_scu_ipc_ioread8(VWARNA_CFG + s_attr->nr, &data);
	if (ret)
		return ret;

	/* Read bits [0-2] of data and multiply by 100(for mV) */
	volt = (data & 0x07) * 100;

	return sprintf(buf, "%d\n", MAX_VOLTAGE - volt);
}

static ssize_t show_irq_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t irq_status;

	ret = intel_scu_ipc_ioread8(SBCUIRQ, &irq_status);
	if (ret)
		return ret;

	return sprintf(buf, "%x\n", irq_status);
}

static ssize_t show_action_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	uint8_t action_status;
	int ret;

	ret = intel_scu_ipc_ioread8(SBCUCTRL, &action_status);
	if (ret)
		return ret;

	return sprintf(buf, "%x\n", action_status);
}

/**
  * handle_events - handle different type of interrupts related to BCU
  * @flag - define what type of interrupt it is
  * @dev_data - device information
  */
static void handle_events(int flag, void *dev_data)
{
	uint8_t irq_data, sticky_data;
	struct vdd_info *vinfo = (struct vdd_info *)dev_data;
	int ret;

	ret = intel_scu_ipc_ioread8(SBCUIRQ, &irq_data);
	if (ret)
		goto handle_ipc_fail;

	if (flag & VCRIT_EVENT) {
		dev_info(&vinfo->pdev->dev, "VCRIT_EVENT occurred\n");
		if (!(irq_data & SVCRIT)) {
			/*
			 * interrupt up for VCRIT timers are removed as
			 * as it is causing hang in system, it will be done
			 * later as enhancement.
			 * Vsys is above VCRIT level
			 */
			ret = intel_scu_ipc_ioread8(BCUDISCRIT_BEH,
				&sticky_data);
			if (ret)
				goto handle_ipc_fail;
			if (IS_STICKY(sticky_data)) {
				/*
				 * Clear the BCUDISCRIT bit in SBCUCTRL
				 * register i.e. 1st bit. We have to clear this
				 * bit manually as it will not go low
				 * automatically in case sticky bit is set.
				 */
				ret = intel_scu_ipc_update_register(SBCUCTRL,
					(1 << 1), SBCUCTRL_CLEAR_ASSERT);
				if (ret)
					goto handle_ipc_fail;
			}
		}
		kobject_uevent(&vinfo->pdev->dev.kobj, KOBJ_CHANGE);
	}
	if (flag & VWARNB_EVENT) {
		dev_info(&vinfo->pdev->dev, "VWARNB_EVENT occurred\n");
		if (!(irq_data & SVWARNB)) {
			/* Vsys is above WARNB level */
			intel_scu_ipc_ioread8(BCUDISB_BEH, &sticky_data);

			if (IS_STICKY(sticky_data)) {
				/*
				 * Clear the BCUDISB bit in SBCUCTRL register
				 * i.e. 2nd bit. We have to clear this bit
				 * manually as it will not go low automatically
				 * in case sticky bit is set.
				 */
				ret = intel_scu_ipc_update_register(SBCUCTRL,
					(1 << 2), SBCUCTRL_STICKY_WARNB);
				if (ret)
					goto handle_ipc_fail;
			}
		}
	}
	if (flag & VWARNA_EVENT) {
		dev_info(&vinfo->pdev->dev, "VWARNA_EVENT occurred\n");
		if (!(irq_data & SVWARNA)) {
			/* Vsys is above WARNA level */
			intel_scu_ipc_ioread8(BCUDISA_BEH, &sticky_data);

			if (IS_STICKY(sticky_data)) {
				/*
				 * Clear the BCUDISA bit in SBCUCTRL register
				 * i.e. 3rd bit. We have to clear this bit
				 * manually as it will not go low automatically
				 * in case sticky bit is set.
				 */
				ret = intel_scu_ipc_update_register(SBCUCTRL,
					(1 << 3), SBCUCTRL_STICKY_WARNA);
				if (ret)
					goto handle_ipc_fail;
			}
		}
	}
	return;
handle_ipc_fail:
	if (flag & VCRIT_EVENT)
		kobject_uevent(&vinfo->pdev->dev.kobj, KOBJ_CHANGE);
	dev_warn(&vinfo->pdev->dev, "ipc read/write failed\n");
}

/**
  * vdd_intrpt_handler - interrupt handler for BCU, runs in kernel context
  * @id - irq value
  * @dev - device information
  */
static irqreturn_t vdd_intrpt_handler(int id, void *dev)
{
	struct vdd_info *vinfo = (struct vdd_info *)dev;
	uint8_t irq_data;

	if (unlikely(kfifo_is_full(&irq_fifo)))
		return IRQ_WAKE_THREAD;

	irq_data = readb(vinfo->bcu_intr_addr);

	/* Interrupt Queuing */
	kfifo_in(&irq_fifo, &irq_data, IRQ_KFIFO_ELEMENT);
	return IRQ_WAKE_THREAD;
};

/**
  * vdd_interrupt_thread_handler - threaded interrupt handler for BCU
  * @irq - irq
  * @dev_data - device information
  */
static irqreturn_t vdd_interrupt_thread_handler(int irq, void *dev_data)
{
	int ret;
	uint8_t irq_data, event = 0;
	struct vdd_info *vinfo = (struct vdd_info *)dev_data;

	if (!vinfo)
		return IRQ_NONE;

	if (unlikely(kfifo_is_empty(&irq_fifo))) {
		dev_err(&vinfo->pdev->dev, "IRQ empty fifo\n");
		return IRQ_NONE;
	}

	ret = kfifo_out(&irq_fifo, &irq_data, IRQ_KFIFO_ELEMENT);
	if (ret != IRQ_KFIFO_ELEMENT)	{
		dev_err(&vinfo->pdev->dev, "KFIFO underflow\n");
		return IRQ_NONE;
	}

	mutex_lock(&vdd_update_lock);
	if (irq_data & VCRIT_IRQ)
		/* BCU VCRIT Interrupt */
		event |= VCRIT_EVENT;
	if (irq_data & VWARNA_IRQ)
		/* BCU WARNA Interrupt */
		event |= VWARNA_EVENT;
	if (irq_data & VWARNB_IRQ)
		/* BCU WARNB Interrupt */
		event |= VWARNB_EVENT;

	handle_events(event, dev_data);

	mutex_unlock(&vdd_update_lock);
	return IRQ_HANDLED;
}

static SENSOR_DEVICE_ATTR_2(voltage_warnA, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 0, 0);
static SENSOR_DEVICE_ATTR_2(voltage_warnB, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 1, 0);
static SENSOR_DEVICE_ATTR_2(voltage_warn_crit, S_IRUGO | S_IWUSR,
				show_volt_thres, store_volt_thres, 2, 0);
static SENSOR_DEVICE_ATTR_2(action_mask, S_IRUGO | S_IWUSR, show_action_mask,
				store_action_mask, 0, 0);
static SENSOR_DEVICE_ATTR_2(bcu_status, S_IRUGO | S_IWUSR, show_bcu_status,
				store_bcu_status, 0, 0);
static SENSOR_DEVICE_ATTR_2(irq_status, S_IRUGO, show_irq_status,
				NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO, show_action_status,
				NULL, 0, 0);

static struct attribute *mid_vdd_attrs[] = {
	&sensor_dev_attr_voltage_warnA.dev_attr.attr,
	&sensor_dev_attr_voltage_warnB.dev_attr.attr,
	&sensor_dev_attr_voltage_warn_crit.dev_attr.attr,
	&sensor_dev_attr_action_mask.dev_attr.attr,
	&sensor_dev_attr_bcu_status.dev_attr.attr,
	&sensor_dev_attr_irq_status.dev_attr.attr,
	&sensor_dev_attr_action_status.dev_attr.attr,
	NULL
};

static struct attribute_group mid_vdd_gr = {
	.name = "msic_voltage",
	.attrs = mid_vdd_attrs
};

/**
  * restore_default_value - assigns data structure with default values
  * @vdata - structure to be assigned
  */
static void restore_default_value(struct vdd_smip_data *vdata)
{
	vdata->vwarna_cfg = INIT_VWARNA;
	vdata->vwarnb_cfg = INIT_VWARNB;
	vdata->vcrit_cfg = INIT_VCRIT;
	vdata->bcudisa_beh = INIT_BCUDISA;
	vdata->bcudisb_beh = INIT_BCUDISB;
	vdata->bcudiscrit_beh = INIT_BCUISCRIT;
	vdata->bcuprochot_beh = INIT_BCUPROCHOT;
	vdata->mbcu_irq = INIT_MBCU;
}

/**
  * program_bcu - configures bcu related registers
  * @pdev - device pointer
  * @vinfo - device information
  */
static int program_bcu(struct ipc_device *pdev, struct vdd_info *vinfo)
{
	int ret;
	struct vdd_smip_data vdata;

	ret = intel_scu_ipc_read_mip((u8 *)&vdata, sizeof(struct
		vdd_smip_data), BCU_SMIP_OFFSET, 1);
	dev_warn(&pdev->dev, "init_config return-val: %d\n", ret);
	if (ret) {
		dev_err(&pdev->dev, "scu ipc read failed\n");
		restore_default_value(&vdata);
		goto configure_register;
	}

	/*
	 * valid voltage will be in range 2.6-3.3V so 0 means values aren't
	 * defined in SMIP
	 */
	if (vdata.vwarna_cfg == 0)
		restore_default_value(&vdata);

configure_register:
	/* configure all related register */
	ret = vdd_set_register(VWARNA_CFG, vdata.vwarna_cfg);
	ret |= vdd_set_register(VWARNB_CFG, vdata.vwarnb_cfg);
	ret |= vdd_set_register(VCRIT_CFG, vdata.vcrit_cfg);
	ret |= vdd_set_register(BCUDISA_BEH, vdata.bcudisa_beh);
	ret |= vdd_set_register(BCUDISB_BEH, vdata.bcudisb_beh);
	ret |= vdd_set_register(BCUDISCRIT_BEH, vdata.bcudiscrit_beh);
	ret |= vdd_set_register(BCUPROCHOT_BEH, vdata.bcuprochot_beh);
	ret |= vdd_set_register(MBCUIRQ, vdata.mbcu_irq);
	if (ret)
		goto vdd_init_error;

	return 0;
vdd_init_error:
	free_irq(vinfo->irq, vinfo);
	return ret;
}

static int mid_vdd_probe(struct ipc_device *pdev)
{
	int i, ret;
	struct vdd_info *vinfo = devm_kzalloc(&pdev->dev,
				sizeof(struct vdd_info), GFP_KERNEL);
	if (!vinfo) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	vinfo->pdev = pdev;
	vinfo->irq = ipc_get_irq(pdev, 0);
	ipc_set_drvdata(pdev, vinfo);

	vinfo->bcu_intr_addr = ioremap(MSIC_BCU_STAT, MSIC_BCU_LEN);
	if (!vinfo->bcu_intr_addr) {
		dev_err(&pdev->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	/* Creating a sysfs group with mid_vdd_gr attributes */
	ret = sysfs_create_group(&pdev->dev.kobj, &mid_vdd_gr);
	if (ret) {
		dev_err(&pdev->dev, "sysfs_create_group failed\n");
		goto vdd_error1;
	}

	/* registering with hwmon class */
	vinfo->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(vinfo->dev)) {
		ret = PTR_ERR(vinfo->dev);
		vinfo->dev = NULL;
		dev_err(&pdev->dev, "hwmon device registration failed\n");
		goto vdd_error2;
	}
	ret = request_threaded_irq(vinfo->irq, vdd_intrpt_handler,
						vdd_interrupt_thread_handler,
						IRQF_TRIGGER_FALLING,
						DRIVER_NAME, vinfo);
	if (ret) {
		dev_err(&pdev->dev, "request_threaded_irq failed:%d\n",
			ret);
		goto vdd_error3;
	}

	ret = program_bcu(pdev, vinfo);
	if (!ret)
		return ret;
vdd_error3:
	hwmon_device_unregister(vinfo->dev);
vdd_error2:
	sysfs_remove_group(&pdev->dev.kobj, &mid_vdd_gr);
vdd_error1:
	iounmap(vinfo->bcu_intr_addr);
	return ret;
}

static int mid_vdd_resume(struct device *dev)
{
	return 0;
}

static int mid_vdd_suspend(struct device *dev)
{
	return 0;
}

static int mid_vdd_remove(struct ipc_device *pdev)
{
	struct vdd_info *vinfo = ipc_get_drvdata(pdev);

	if (vinfo) {
		free_irq(vinfo->irq, vinfo);
		hwmon_device_unregister(vinfo->dev);
		sysfs_remove_group(&pdev->dev.kobj, &mid_vdd_gr);
		iounmap(vinfo->bcu_intr_addr);
	}
	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/
static const struct ipc_device_id vdd_id_table[] = {
	{ DEVICE_NAME, 1 },
	{ },
};

static const struct dev_pm_ops msic_vdd_pm_ops = {
	.suspend = mid_vdd_suspend,
	.resume = mid_vdd_resume,
};

static struct ipc_driver mid_volt_drop_detect_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &msic_vdd_pm_ops,
		},
	.probe = mid_vdd_probe,
	.remove = __devexit_p(mid_vdd_remove),
	.id_table = vdd_id_table,
};

static int __init mid_vdd_module_init(void)
{
	return ipc_driver_register(&mid_volt_drop_detect_driver);
}

static void __exit mid_vdd_module_exit(void)
{
	ipc_driver_unregister(&mid_volt_drop_detect_driver);
}

module_init(mid_vdd_module_init);
module_exit(mid_vdd_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_AUTHOR("Chaurasia, Avinash K <avinash.k.chaurasia@intel.com>");
MODULE_DESCRIPTION("Intel CloverView Voltage Drop Detection Driver");
MODULE_LICENSE("GPL");
