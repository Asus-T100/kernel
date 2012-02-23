/*
 * intel_mid_ocd.c - Intel Medfield Platform Over Current Detection Driver
 *
 *
 * Copyright (C) 2010 Intel Corporation
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
 */

#define pr_fmt(fmt)  "intel_mid_ocd: " fmt

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

#define DRIVER_NAME "msic_ocd"

/* Registers that govern current monitoring */
#define BATTCURRENTLIMIT12	0x102
#define BATTTIMELIMIT12		0x103
#define BATTTIMELIMIT3		0x104
#define BATTTIMEDB		0x105
#define BRSTCONFIGOUTPUTS	0x106
#define BRSTCONFIGACTIONS	0x107
#define BRSTCONTROLSTATUS	0x108

#define CRITBATT_STATUS		(1 << 7)
#define SYSOUTEN		(1 << 3)
#define SYSACTEN		(1 << 3)

/* Status register */
#define SYSSTAT			(1 << 3)
#define CAMSTAT			(1 << 4)
#define SYSTEM_STAT		(0x0F << 0)
#define OVER_TIMER1		(1 << 5)
#define OVER_TIMER2		(1 << 6)

#define NUM_CURR_LIMITS		8
#define NUM_TIME_LIMITS		15

/* Base and offset for every time limit */
#define TIME_LIMIT12_BASE	200
#define TIME_LIMIT12_OFFSET	500
#define TIME_LIMIT12_MAX	(TIME_LIMIT12_BASE + \
				(TIME_LIMIT12_OFFSET * NUM_TIME_LIMITS))

#define TIME_LIMIT3_BASE	200
#define TIME_LIMIT3_OFFSET	1000
#define TIME_LIMIT3_MAX		(TIME_LIMIT3_OFFSET * NUM_TIME_LIMITS)

#define MAX_COUNT		0xFFFFFFFF

/*
 * Number of different Battery Levels:
 * (0) More than 20%
 * (1) Between 5% - 20%
 * (2) Less than 5%
 */
#define NUM_BATTERY_LEVELS      3

/* SMIP entries for each Battery Level */
#define NUM_BCU_SMIP_ENTRIES    NUM_BATTERY_LEVELS

/* Base SMIP address from where the BCU related info should be read */
#define BCU_SMIP_BASE		0x46C

/*
 * No of Bytes we have to read from SMIP from BCU_SMIP_BASE. The SMIP holds
 * 7 bytes for each battery level. There are 3 battery levels, corresponding
 * to 100%, 20% and 5% battery capacities. Since the 100%(BATT_FULL) is
 * obvious and SMIP space is precious, this is not actually stored. Hence
 * there are only 20 bytes of stored in SMIP.
 */
#define NUM_SMIP_BYTES		20

/* Full Battery capacity */
#define BATT_FULL		100

static DEFINE_MUTEX(ocd_update_lock);

/*
 * Stores the current thresholds(in mA) at which
 * row 0: warning is generated
 * row 1: system shut down can be initiated
 */
static const unsigned long curr_thresholds[][NUM_CURR_LIMITS] = {
			{1400, 1800, 2200, 2800, 3000, 3400, 3800, 4800},
			{1800, 2200, 2800, 3000, 3800, 4800, 5800, 5800} };

struct ocd_info {
	unsigned long timer1_count;
	unsigned long timer2_count;
	unsigned long acc_time;
	unsigned long curr_batt_level;
	unsigned int irq;
	/* Cached value of the status register, since the last interrupt */
	uint8_t intrpt_status;
	struct device *dev;
	struct ipc_device *ipcdev;
};

/*
 * These values are read from SMIP during the driver's init.
 * SMIP contains these set of entries for each Battery Level.
 * BCU is programmed to these values depending on the Battery Level.
 */
struct ocd_smip_data {
	uint8_t climit12;
	uint8_t tlimit12;
	uint8_t tlimit3;
	uint8_t timer_db;
	uint8_t bcu_outputs;
	uint8_t bcu_actions;
	uint8_t batt_cap;
} __packed;

/* Array of structures holding BCU register values, read from SMIP */
struct ocd_smip_data ocd_smip_data[NUM_BCU_SMIP_ENTRIES];

static int program_bcu(int indx)
{
	int ret;

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_iowrite8(BATTCURRENTLIMIT12,
					ocd_smip_data[indx].climit12);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BATTTIMELIMIT12,
					ocd_smip_data[indx].tlimit12);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BATTTIMELIMIT3,
					ocd_smip_data[indx].tlimit3);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BATTTIMEDB, ocd_smip_data[indx].timer_db);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGOUTPUTS,
					ocd_smip_data[indx].bcu_outputs);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGACTIONS,
					ocd_smip_data[indx].bcu_actions);

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static int get_bcu_config_from_smip(void)
{
	int i, ret;
	int data_size = sizeof(struct ocd_smip_data);
	uint8_t data[NUM_SMIP_BYTES];

	/* intel_scu_ipc_read_mip function reads MIP data in terms of bytes */
	ret = intel_scu_ipc_read_mip(data, NUM_SMIP_BYTES, BCU_SMIP_BASE, 1);
	if (ret)
		return ret;

	/* Copy 'data' to ocd_smip_data structure in multiples of 7 bytes */
	for (i = 0; i < NUM_BCU_SMIP_ENTRIES; i++)
		memcpy(&ocd_smip_data[i], &data[i * data_size], data_size);

	/*
	 * SMIP holds two battery capacities. The full capacity is assumed
	 * to be 100%. Hence, move the batt_cap accordingly, to fit into
	 * ocd_smip_data structure.
	 */
	for (i = NUM_BCU_SMIP_ENTRIES - 1; i > 0; i--)
		ocd_smip_data[i].batt_cap = ocd_smip_data[i-1].batt_cap;

	/* The first SMIP entry's battery capacity is 100 */
	ocd_smip_data[0].batt_cap = BATT_FULL;

	return 0;
}

static int configure_critbatt(int flag)
{
	int ret;
	uint8_t data;

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(BRSTCONFIGACTIONS, &data);
	if (ret)
		goto ipc_fail;

	/*
	 * Zero enables shutdown due to critbatt assertion.
	 * Non-zero disables this functionality.
	 */
	if (!flag)
		data &= (~CRITBATT_STATUS);
	else
		data |= CRITBATT_STATUS;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGACTIONS, data);

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static int disable_sysburst(void)
{
	int ret;
	uint8_t out_data, act_data;

	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(BRSTCONFIGACTIONS, &act_data);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_ioread8(BRSTCONFIGOUTPUTS, &out_data);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGACTIONS, act_data & (~SYSACTEN));
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGOUTPUTS, out_data & (~SYSOUTEN));

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static int restore_sysburst(struct device *dev)
{
	int ret;
	uint8_t out_data, act_data;
	struct ocd_smip_data *smip_data;
	struct ocd_info *cinfo = dev_get_drvdata(dev);

	if (!cinfo) {
		dev_err(dev, "cinfo is NULL in restore_sysburst\n");
		return -ENODEV;
	}

	/* Alright, we can avoid this pointer. But keep it for readability. */
	smip_data = &ocd_smip_data[cinfo->curr_batt_level];

	/*
	 * For the current battery level, if SYSACT and SYSOUT are not
	 * enabled in the SMIP settings, then just return. Not an Error.
	 */
	if (!((smip_data->bcu_actions & SYSACTEN) &&
		(smip_data->bcu_outputs & SYSOUTEN)))
		return 0;

	/*
	 * SMIP settings for the current battery level enable _both_
	 * SYSACT and SYSOUT. But, we disabled these during suspend.
	 * Now, enable both of them.
	 */
	mutex_lock(&ocd_update_lock);

	ret = intel_scu_ipc_ioread8(BRSTCONFIGACTIONS, &act_data);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_ioread8(BRSTCONFIGOUTPUTS, &out_data);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGACTIONS, act_data | SYSACTEN);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGOUTPUTS, out_data | SYSOUTEN);

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static ssize_t store_batt_level(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long level;
	struct ocd_info *cinfo = dev_get_drvdata(dev);

	if (strict_strtoul(buf, 10, &level))
		return -EINVAL;

	if (level > NUM_BATTERY_LEVELS - 1)
		return -EINVAL;

	if (cinfo->curr_batt_level == level)
		return count;

	/* Battery Level Changed. So, program the BCU accordingly */
	ret = program_bcu(level);
	if (ret) {
		dev_err(dev, "Failed to program BCU for level %ld\n", level);
		return ret;
	}

	cinfo->curr_batt_level = level;
	return count;
}

static ssize_t show_batt_level(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ocd_info *cinfo = dev_get_drvdata(dev);

	/*
	 * This shows the current Battery Level:
	 * 0: Battery Capacity is more than 20%
	 * 1: Battery Capacity is between 5% and 20%
	 * 2: Battery Capacity is lesser than 5%
	 */
	return sprintf(buf, "%lu\n", cinfo->curr_batt_level);
}

static ssize_t show_avail_batt_caps(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d %d %d\n",
			ocd_smip_data[0].batt_cap,
			ocd_smip_data[1].batt_cap,
			ocd_smip_data[2].batt_cap);
}

static ssize_t store_critbatt_status(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val != 0 && val != 1)
		return -EINVAL;

	return configure_critbatt(val) ? -EINVAL : count;
}

static ssize_t show_critbatt_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t data;

	ret = intel_scu_ipc_ioread8(BRSTCONFIGACTIONS, &data);
	if (ret)
		return ret;

	ret = (data & CRITBATT_STATUS) ? 1 : 0;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t store_output_mask(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long data;

	if (strict_strtoul(buf, 16, &data))
		return -EINVAL;

	/*
	 * We are writing bits 0:4 of an 8 bit register.
	 * Bits 5:7 are reserved.
	 */
	if (data > 0x1F)
		return -EINVAL;

	ret = intel_scu_ipc_iowrite8(BRSTCONFIGOUTPUTS, (uint8_t)data);
	if (ret)
		return ret;

	return count;
}

static ssize_t show_output_mask(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t data;

	ret = intel_scu_ipc_ioread8(BRSTCONFIGOUTPUTS, &data);
	if (ret)
		return ret;

	return sprintf(buf, "0x%.2x\n", data);
}

static ssize_t show_action_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ocd_info *cinfo = dev_get_drvdata(dev);

	/* This shows the status of actions taken since the last interrupt */
	return sprintf(buf, "0x%.2x\n", cinfo->intrpt_status);
}

static int get_current_value(unsigned long value, int index)
{
	int pos = 0;

	if (index != 0 && index != 1)
		return -EINVAL;

	if (value < curr_thresholds[index][0] ||
			value > curr_thresholds[index][NUM_CURR_LIMITS-1])
		return -EINVAL;

	/* Find the index of 'value' in the thresholds array */
	while (pos < NUM_CURR_LIMITS && value >= curr_thresholds[index][pos])
		++pos;

	return pos - 1;
}

static ssize_t store_curr_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint8_t data;
	unsigned long curnt;
	int pos;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	if (strict_strtoul(buf, 10, &curnt))
		return -EINVAL;

	mutex_lock(&ocd_update_lock);

	pos = get_current_value(curnt, s_attr->nr);
	if (pos < 0) {
		ret = pos;
		goto ipc_fail;
	}

	ret = intel_scu_ipc_ioread8(BATTCURRENTLIMIT12, &data);
	if (ret)
		goto ipc_fail;

	switch (s_attr->nr) {
	case 0:
		/* Set bits [0-2] to value of pos */
		data = (data & 0xF8) | pos;
		break;
	case 1:
		/* Set bits [3-5] to value of pos */
		data = (data & 0xC7) | (pos << 3);
		break;
	default:
		dev_err(dev, "Current Index Invalid");
		ret = -EINVAL;
		goto ipc_fail;
	}

	ret = intel_scu_ipc_iowrite8(BATTCURRENTLIMIT12, data);
	if (ret)
		goto ipc_fail;

	ret = count;

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static ssize_t show_curr_thres(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, indx;
	uint8_t data;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	WARN_ON(s_attr->nr != 0 && s_attr->nr != 1);

	ret = intel_scu_ipc_ioread8(BATTCURRENTLIMIT12, &data);
	if (ret)
		return ret;

	/* Read bits [0-2] or [3-5] of data */
	indx = (data >> (3 * s_attr->nr)) & 0x07;

	return sprintf(buf, "%lu\n", curr_thresholds[s_attr->nr][indx]);
}

static int get_timer_threshold(unsigned long time, int index)
{
	if (index == 0 || index == 1) {
		if (time >= TIME_LIMIT12_BASE && time <= TIME_LIMIT12_MAX)
			return (time - TIME_LIMIT12_BASE)/TIME_LIMIT12_OFFSET;
	} else if (index == 2) {
		if (time >= TIME_LIMIT3_BASE && time <= TIME_LIMIT3_MAX)
			return time / TIME_LIMIT3_OFFSET;
	}
	return -EINVAL;
}

static ssize_t store_timer_thres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long time;
	uint8_t data;
	int ret, val;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	if (strict_strtoul(buf, 10, &time))
		return -EINVAL;

	if (s_attr->nr < 0 || s_attr->nr > 2)
		return -EINVAL;

	val = get_timer_threshold(time, s_attr->nr);
	if (val < 0)
		return -EINVAL;

	mutex_lock(&ocd_update_lock);

	if (s_attr->nr == 2) {
		ret = intel_scu_ipc_ioread8(BATTTIMELIMIT3, &data);
		if (ret)
			goto ipc_fail;
		/* Set bits [0-3] to val */
		data = (data & 0xF0) | val;

		ret = intel_scu_ipc_iowrite8(BATTTIMELIMIT3, data);
		if (ret)
			goto ipc_fail;
	} else {
		ret = intel_scu_ipc_ioread8(BATTTIMELIMIT12, &data);
		if (ret)
			goto ipc_fail;

		if (s_attr->nr == 0) {
			/* Set bits [0-3] to val */
			data = (data & 0xF0) | val;
		} else {
			/* Set bits [4-7] to val */
			data = (data & 0x0F) | (val << 4);
		}

		ret = intel_scu_ipc_iowrite8(BATTTIMELIMIT12, data);
		if (ret)
			goto ipc_fail;
	}
	ret = count;

ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static ssize_t show_timer_thres(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, val;
	uint8_t data;
	int time;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	if (s_attr->nr < 0 || s_attr->nr > 2)
		return -EINVAL;

	if (s_attr->nr == 0 || s_attr->nr == 1) {
		ret = intel_scu_ipc_ioread8(BATTTIMELIMIT12, &data);
		if (ret)
			return ret;

		val = (data >> (4 * s_attr->nr)) & 0x0F;
		time = TIME_LIMIT12_BASE + val * TIME_LIMIT12_OFFSET;

	} else {
		ret = intel_scu_ipc_ioread8(BATTTIMELIMIT3, &data);
		if (ret)
			return ret;

		val = data & 0x0F;
		time = (val) ? (val * TIME_LIMIT3_OFFSET) : TIME_LIMIT3_BASE;
	}

	return sprintf(buf, "%d\n", time);
}

static ssize_t show_warn_count(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ocd_info *cinfo = dev_get_drvdata(dev);

	return sprintf(buf, "%ld %ld\n", cinfo->timer1_count,
							cinfo->timer2_count);
}

static ssize_t store_acc_time(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct ocd_info *cinfo = dev_get_drvdata(dev);
	unsigned long time;

	if (strict_strtoul(buf, 10, &time))
		return -EINVAL;

	/* Writing 0 resets the acc_time. Ignore any other values */
	if (time)
		return -EINVAL;

	/* Set the acc_time to 'now' */
	cinfo->acc_time = jiffies;

	/* Clear warning counters */
	cinfo->timer1_count = cinfo->timer2_count = 0;

	return count;
}
static ssize_t show_acc_time(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ocd_info *cinfo = dev_get_drvdata(dev);
	long time_gap;

	if (cinfo->acc_time == 0)
		return sprintf(buf, "0\n");

	/* Calculate the time gap in jiffies */
	time_gap = jiffies - cinfo->acc_time;

	/* Convert to milli secs and print */
	return sprintf(buf, "%u\n", jiffies_to_msecs(time_gap));
}

static irqreturn_t ocd_handle_intrpt(int irq, void *dev_data)
{
	int ret;
	uint8_t data;
	struct ocd_info *cinfo = (struct ocd_info *)dev_data;

	if (!cinfo)
		return IRQ_NONE;

	/* Interrupt came now */
	cinfo->acc_time = jiffies;

	mutex_lock(&ocd_update_lock);

	/* Read the interrupt status register */
	ret = intel_scu_ipc_ioread8(BRSTCONTROLSTATUS, &data);
	if (ret)
		goto ipc_fail;

	/* Cache the interrupt status register */
	cinfo->intrpt_status = data;

	/* It's a timer1 interrupt. Increment the counter.
	 * Reset timer1, camera and sys burst status bits */
	if (data & OVER_TIMER1) {
		cinfo->timer1_count++;
		data &= ~(OVER_TIMER1 | CAMSTAT | SYSSTAT);
	}

	/* It's a timer2 interrupt. Increment the counter.
	 * Reset timer2 and other system status bits */
	if (data & OVER_TIMER2) {
		cinfo->timer2_count++;
		data &= ~(OVER_TIMER2 | SYSTEM_STAT);
	}

	/* Write the masked data */
	ret = intel_scu_ipc_iowrite8(BRSTCONTROLSTATUS, data);
	if (ret)
		goto ipc_fail;

	if (cinfo->timer1_count == MAX_COUNT ||
				cinfo->timer2_count == MAX_COUNT) {
		cinfo->timer1_count = cinfo->timer2_count = 0;
		cinfo->acc_time = jiffies;
	}

	ret = IRQ_HANDLED;
ipc_fail:
	mutex_unlock(&ocd_update_lock);
	return ret;
}

static SENSOR_DEVICE_ATTR_2(avail_batt_caps, S_IRUGO,
				show_avail_batt_caps, NULL, 0, 0);

static SENSOR_DEVICE_ATTR_2(batt_level, S_IRUGO | S_IWUSR,
				show_batt_level, store_batt_level, 0, 0);

static SENSOR_DEVICE_ATTR_2(critbatt_status, S_IRUGO | S_IWUSR,
				show_critbatt_status, store_critbatt_status,
				0, 0);

static SENSOR_DEVICE_ATTR_2(output_mask, S_IRUGO | S_IWUSR,
				show_output_mask, store_output_mask, 0, 0);

static SENSOR_DEVICE_ATTR_2(current_warning, S_IRUGO | S_IWUSR,
				show_curr_thres, store_curr_thres, 0, 0);
static SENSOR_DEVICE_ATTR_2(current_shutdown, S_IRUGO | S_IWUSR,
				show_curr_thres, store_curr_thres, 1, 0);

static SENSOR_DEVICE_ATTR_2(timer_warning, S_IRUGO | S_IWUSR,
				show_timer_thres, store_timer_thres, 0, 0);
static SENSOR_DEVICE_ATTR_2(timer_hw_action, S_IRUGO | S_IWUSR,
				show_timer_thres, store_timer_thres, 1, 0);
static SENSOR_DEVICE_ATTR_2(timer_shutdown, S_IRUGO | S_IWUSR,
				show_timer_thres, store_timer_thres, 2, 0);

static SENSOR_DEVICE_ATTR_2(accumulation_time, S_IRUGO | S_IWUSR,
					show_acc_time, store_acc_time, 0, 0);

static SENSOR_DEVICE_ATTR_2(warning_count, S_IRUGO, show_warn_count,
								NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO, show_action_status,
								NULL, 0, 0);

static struct attribute *mid_ocd_attrs[] = {
	&sensor_dev_attr_critbatt_status.dev_attr.attr,
	&sensor_dev_attr_output_mask.dev_attr.attr,
	&sensor_dev_attr_current_warning.dev_attr.attr,
	&sensor_dev_attr_current_shutdown.dev_attr.attr,
	&sensor_dev_attr_timer_warning.dev_attr.attr,
	&sensor_dev_attr_timer_hw_action.dev_attr.attr,
	&sensor_dev_attr_timer_shutdown.dev_attr.attr,
	&sensor_dev_attr_warning_count.dev_attr.attr,
	&sensor_dev_attr_accumulation_time.dev_attr.attr,
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_batt_level.dev_attr.attr,
	&sensor_dev_attr_avail_batt_caps.dev_attr.attr,
	NULL
};

static struct attribute_group mid_ocd_gr = {
	.name = "msic_current",
	.attrs = mid_ocd_attrs
};

static int mid_ocd_probe(struct ipc_device *ipcdev)
{
	int ret;
	struct ocd_info *cinfo = kzalloc(sizeof(struct ocd_info), GFP_KERNEL);

	if (!cinfo) {
		dev_err(&ipcdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	cinfo->ipcdev = ipcdev;
	cinfo->irq = ipc_get_irq(ipcdev, 0);
	cinfo->acc_time = jiffies;
	ipc_set_drvdata(ipcdev, cinfo);

	/* Read SMIP to obtain BCU configurations for various battery levels */
	ret = get_bcu_config_from_smip();
	if (ret) {
		dev_err(&ipcdev->dev, "SMIP read failed\n");
		goto ocd_error1;
	}

	/* Creating a sysfs group with mid_ocd_gr attributes */
	ret = sysfs_create_group(&ipcdev->dev.kobj, &mid_ocd_gr);
	if (ret) {
		dev_err(&ipcdev->dev, "sysfs create group failed\n");
		goto ocd_error1;
	}

	/* Registering with hwmon class */
	cinfo->dev = hwmon_device_register(&ipcdev->dev);
	if (IS_ERR(cinfo->dev)) {
		ret = PTR_ERR(cinfo->dev);
		cinfo->dev = NULL;
		dev_err(&ipcdev->dev, "hwmon_dev_regs failed\n");
		goto ocd_error2;
	}

	/* Register for Interrupt Handler */
	ret = request_threaded_irq(cinfo->irq, NULL, ocd_handle_intrpt,
						IRQF_TRIGGER_FALLING,
						DRIVER_NAME, cinfo);
	if (ret) {
		dev_err(cinfo->dev, "request_threaded_irq failed:%d\n", ret);
		goto ocd_error2;
	}

	/* Initially, program the BCU for battery level more than 20% */
	ret = program_bcu(0);
	if (ret) {
		dev_err(cinfo->dev, "program_bcu(0) failed:%d\n", ret);
		goto ocd_error2;
	}

	return 0;

ocd_error2:
	sysfs_remove_group(&ipcdev->dev.kobj, &mid_ocd_gr);
ocd_error1:
	kfree(cinfo);
	return ret;
}

static int mid_ocd_resume(struct device *dev)
{
	return restore_sysburst(dev);
}

static int mid_ocd_suspend(struct device *dev)
{
    /* Disable the sysburst interrupt when we suspend */
	return disable_sysburst();
}

static int mid_ocd_remove(struct ipc_device *ipcdev)
{
	struct ocd_info *cinfo = ipc_get_drvdata(ipcdev);

	if (cinfo) {
		hwmon_device_unregister(cinfo->dev);
		sysfs_remove_group(&ipcdev->dev.kobj, &mid_ocd_gr);
		kfree(cinfo);
	}
	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/
static const struct ipc_device_id ocd_id_table[] = {
	{ DRIVER_NAME, 1 },
	{ },
};

static const struct dev_pm_ops msic_ocd_pm_ops = {
	.suspend = mid_ocd_suspend,
	.resume = mid_ocd_resume,
};

static struct ipc_driver mid_over_curr_detect_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &msic_ocd_pm_ops,
		},
	.probe = mid_ocd_probe,
	.remove = __devexit_p(mid_ocd_remove),
	.id_table = ocd_id_table,
};

static int __init mid_ocd_module_init(void)
{
	return ipc_driver_register(&mid_over_curr_detect_driver);
}

static void __exit mid_ocd_module_exit(void)
{
	ipc_driver_unregister(&mid_over_curr_detect_driver);
}

module_init(mid_ocd_module_init);
module_exit(mid_ocd_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Medfield Over Current Detection Driver");
MODULE_LICENSE("GPL");
