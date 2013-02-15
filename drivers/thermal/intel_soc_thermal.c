/*
 * intel_soc_thermal.c - Intel SoC Platform Thermal Driver
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
 * Author: Shravan B M <shravan.k.b.m@intel.com>
 *
 * This driver registers to Thermal framework as SoC zone. It exposes
 * two SoC DTS temperature with aux trip points. Only aux0, aux1 are
 * writable.
 *
 */

#define pr_fmt(fmt)  "intel_soc_thermal: " fmt

#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>

#include <asm/intel-mid.h>

#define DRIVER_NAME	"soc_thrm"

/* SOC DTS Registers */
#define SOC_THERMAL_SENSORS	2
#define PUNIT_PORT		0x04
#define DTS_ENABLE_REG		0xB0
#define PUNIT_TEMP_REG		0xB1
#define PUNIT_AUX_REG		0xB2
#define DTS_ENABLE		0x02
/* There are 4 Aux trips. Only Aux0, Aux1 are writeable */
#define DTS_TRIP_RW		0x03
#define SOC_THERMAL_TRIPS	4

#define TJMAX_TEMP		90
#define TJMAX_CODE		0x7F

/* IRQ details */
#define SOC_DTS_CONTROL		0x80
#define TRIP_STATUS_RO		0xB3
#define TRIP_STATUS_RW		0xB4
/* TE stands for THERMAL_EVENT */
#define TE_AUX0			0xB5
#define TE_AUX1			0xB6
#define TE_AUX2			0xB7
#define TE_AUX3			0xB8
#define ENABLE_AUX_INTRPT	0x0F
#define ENABLE_CPU0		(1 << 16)
#define RTE_ENABLE		(1 << 9)
#define AUX0_EVENT		(1 << 0)
#define AUX1_EVENT		(1 << 1)
#define AUX2_EVENT		(1 << 2)
#define AUX3_EVENT		(1 << 3)

struct platform_soc_data {
	struct thermal_zone_device *tzd[SOC_THERMAL_SENSORS];
	int irq;
};

struct thermal_device_info {
	int sensor_index;
	struct mutex lock_aux;
};

static inline u32 read_soc_reg(unsigned int addr)
{
	return intel_mid_msgbus_read32(PUNIT_PORT, addr);
}

static inline void write_soc_reg(unsigned int addr, u32 val)
{
	intel_mid_msgbus_write32(PUNIT_PORT, addr, val);
}

#ifdef CONFIG_DEBUG_FS
struct dts_regs {
	char *name;
	u32 addr;
} dts_regs[] = {
	/* Thermal Management Registers */
	{"PTMC",	0x80},
	{"TRR0",	0x81},
	{"TRR1",	0x82},
	{"TTS",		0x83},
	{"TELB",	0x84},
	{"TELT",	0x85},
	{"GFXT",	0x88},
	{"VEDT",	0x89},
	{"VECT",	0x8A},
	{"VSPT",	0x8B},
	{"ISPT",	0x8C},
	{"SWT",		0x8D},
	/* Trip Event Registers */
	{"DTSC",	0xB0},
	{"TRR",		0xB1},
	{"PTPS",	0xB2},
	{"PTTS",	0xB3},
	{"PTTSS",	0xB4},
	{"TE_AUX0",	0xB5},
	{"TE_AUX1",	0xB6},
	{"TE_AUX2",	0xB7},
	{"TE_AUX3",	0xB8},
	{"TTE_VRIcc",	0xB9},
	{"TTE_VRHOT",	0xBA},
	{"TTE_PROCHOT",	0xBB},
	{"TTE_SLM0",	0xBC},
	{"TTE_SLM1",	0xBD},
	{"BWTE",	0xBE},
	{"TTE_SWT",	0xBF},
	/* MSI Message Registers */
	{"TMA",		0xC0},
	{"TMD",		0xC1},
};

/* /sys/kernel/debug/tng_soc_dts */
static struct dentry *soc_dts_dent;
static struct dentry *tng_thermal_dir;

static int soc_dts_debugfs_show(struct seq_file *s, void *unused)
{
	int i;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(dts_regs); i++) {
		val = read_soc_reg(dts_regs[i].addr);
		seq_printf(s,
			"%s[0x%X]	Val: 0x%X\n",
			dts_regs[i].name, dts_regs[i].addr, val);
	}
	return 0;
}

static int debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, soc_dts_debugfs_show, NULL);
}

static const struct file_operations soc_dts_debugfs_fops = {
	.open           = debugfs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void create_soc_dts_debugfs(void)
{
	int err;

	/* /sys/kernel/debug/tng_thermal/ */
	tng_thermal_dir = debugfs_create_dir("tng_thermal", NULL);
	if (IS_ERR(tng_thermal_dir)) {
		err = PTR_ERR(tng_thermal_dir);
		pr_err("debugfs_create_dir failed:%d\n", err);
		return;
	}

	/* /sys/kernel/debug/tng_thermal/soc_dts */
	soc_dts_dent = debugfs_create_file("soc_dts", S_IFREG | S_IRUGO,
					tng_thermal_dir, NULL,
					&soc_dts_debugfs_fops);
	if (IS_ERR(soc_dts_dent)) {
		err = PTR_ERR(soc_dts_dent);
		debugfs_remove_recursive(tng_thermal_dir);
		pr_err("debugfs_create_file failed:%d\n", err);
	}
}

static void remove_soc_dts_debugfs(void)
{
	debugfs_remove_recursive(tng_thermal_dir);
}
#else
static inline void create_soc_dts_debugfs(void) { }
static inline void remove_soc_dts_debugfs(void) { }
#endif

static struct thermal_device_info *initialize_sensor(int index)
{
	struct thermal_device_info *td_info =
		kzalloc(sizeof(struct thermal_device_info), GFP_KERNEL);

	if (!td_info)
		return NULL;
	td_info->sensor_index = index;
	mutex_init(&td_info->lock_aux);
	return td_info;
}

static void enable_soc_dts(void)
{
	int i;
	u32 val;

	/* Enable the DTS */
	write_soc_reg(DTS_ENABLE_REG, DTS_ENABLE);

	val = read_soc_reg(SOC_DTS_CONTROL);
	write_soc_reg(SOC_DTS_CONTROL, val | ENABLE_AUX_INTRPT | ENABLE_CPU0);

	/* Enable Interrupts for all the AUX trips for the DTS */
	for (i = 0; i < SOC_THERMAL_TRIPS; i++) {
		val = read_soc_reg(TE_AUX0 + i);
		write_soc_reg(TE_AUX0 + i, (val | RTE_ENABLE));
	}
}

static ssize_t show_temp(struct thermal_zone_device *tzd, long *temp)
{
	struct thermal_device_info *td_info = tzd->devdata;
	u32 val = read_soc_reg(PUNIT_TEMP_REG);

	/* Extract bits[0:7] or [8:15] using sensor_index */
	*temp =  (val >> (8 * td_info->sensor_index)) & 0xFF;

	/* Calibrate the temperature */
	*temp = TJMAX_CODE - *temp + TJMAX_TEMP;

	/* Convert to mC */
	*temp *= 1000;

	return 0;
}

static ssize_t show_trip_type(struct thermal_zone_device *tzd,
			int trip, enum thermal_trip_type *trip_type)
{
	/* All are passive trip points */
	*trip_type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static ssize_t show_trip_temp(struct thermal_zone_device *tzd,
				int trip, long *trip_temp)
{
	u32 aux_value = read_soc_reg(PUNIT_AUX_REG);

	/* aux0 b[0:7], aux1 b[8:15], aux2 b[16:23], aux3 b[24:31] */
	*trip_temp = (aux_value >> (8 * trip)) & 0xFF;

	/* Calibrate the trip point temperature */
	*trip_temp = TJMAX_TEMP - *trip_temp;

	/* Convert to mC and report */
	*trip_temp *= 1000;

	return 0;
}

static ssize_t store_trip_temp(struct thermal_zone_device *tzd,
				int trip, long trip_temp)
{
	u32 aux_trip, aux = 0;
	struct thermal_device_info *td_info = tzd->devdata;

	/* Convert from mC to C */
	trip_temp /= 1000;

	/* The trip temp is 8 bits wide (unsigned) */
	if (trip_temp > 255)
		return -EINVAL;

	/* Assign last byte to unsigned 32 */
	aux_trip = trip_temp & 0xFF;

	/* Calibrate w.r.t TJMAX_TEMP */
	aux_trip = TJMAX_TEMP - aux_trip;

	mutex_lock(&td_info->lock_aux);
	aux = read_soc_reg(PUNIT_AUX_REG);
	switch (trip) {
	case 0:
		/* aux0 bits 0:7 */
		aux = (aux & 0xFFFFFF00) | (aux_trip << (8 * trip));
		break;
	case 1:
		/* aux1 bits 8:15 */
		aux = (aux & 0xFFFF00FF) | (aux_trip << (8 * trip));
		break;
	case 2:
		/* aux2 bits 16:23 */
		aux = (aux & 0xFF00FFFF) | (aux_trip << (8 * trip));
		break;
	case 3:
		/* aux3 bits 24:31 */
		aux = (aux & 0x00FFFFFF) | (aux_trip << (8 * trip));
		break;
	}
	write_soc_reg(PUNIT_AUX_REG, aux);

	mutex_unlock(&td_info->lock_aux);

	return 0;
}

static irqreturn_t soc_dts_intrpt(int irq, void *dev_data)
{
	u32 irq_sts;
	struct thermal_zone_device *tzd;
	char *event;
	bool valid = true;
	struct platform_soc_data *pdata = (struct platform_soc_data *)dev_data;

	if (!pdata || !pdata->tzd[0])
		return IRQ_NONE;

	tzd = pdata->tzd[0];

	irq_sts = read_soc_reg(TRIP_STATUS_RW);

	/* The status bit is cleared by writing 1 to the bit */
	if (irq_sts & AUX0_EVENT) {
		event = "aux0_event";
		irq_sts |= AUX0_EVENT;
	} else if (irq_sts & AUX1_EVENT) {
		event = "aux1_event";
		irq_sts |= AUX1_EVENT;
	} else if (irq_sts & AUX2_EVENT) {
		event = "aux2_event";
		irq_sts |= AUX2_EVENT;
	} else if (irq_sts & AUX3_EVENT) {
		event = "aux3_event";
		irq_sts |= AUX3_EVENT;
	} else {
		event = "invalid_event";
		valid = false;
	}

	dev_info(&tzd->device, "SoC DTS %s occurred\n", event);

	/* Notify using UEvent, if it is a valid event */
	if (valid) {
		kobject_uevent(&tzd->device.kobj, KOBJ_CHANGE);
		/* Clear the status bits */
		write_soc_reg(TRIP_STATUS_RW, irq_sts);
	}

	return IRQ_HANDLED;
}

static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = show_temp,
	.get_trip_type = show_trip_type,
	.get_trip_temp = show_trip_temp,
	.set_trip_temp = store_trip_temp,
};

/*********************************************************************
 *		Driver initialization and finalization
 *********************************************************************/

static int soc_thermal_probe(struct platform_device *pdev)
{
	struct platform_soc_data *pdata;
	int i, ret;
	static char *name[SOC_THERMAL_SENSORS] = {"SoC_DTS0", "SoC_DTS1"};

	pdata = kzalloc(sizeof(struct platform_soc_data), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* Register each sensor with the generic thermal framework */
	for (i = 0; i < SOC_THERMAL_SENSORS; i++) {
		pdata->tzd[i] = thermal_zone_device_register(name[i],
					4, DTS_TRIP_RW, initialize_sensor(i),
					&tzd_ops, 0, 0, 0, 0);
		if (IS_ERR(pdata->tzd[i])) {
			ret = PTR_ERR(pdata->tzd[i]);
			dev_err(&pdev->dev, "tzd register failed: %d\n", ret);
			goto exit_reg;
		}
	}

	platform_set_drvdata(pdev, pdata);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "platform_get_irq failed:%d\n", ret);
		goto exit_reg;
	}

	pdata->irq = ret;

	/* Register for Interrupt Handler */
	ret = request_threaded_irq(pdata->irq, NULL, soc_dts_intrpt,
						IRQF_TRIGGER_RISING,
						DRIVER_NAME, pdata);
	if (ret) {
		dev_err(&pdev->dev, "request_threaded_irq failed:%d\n", ret);
		goto exit_reg;
	}

	/* Enable DTS0 and DTS1 */
	enable_soc_dts();

	create_soc_dts_debugfs();

	return 0;

exit_reg:
	while (--i >= 0) {
		struct thermal_device_info *td_info = pdata->tzd[i]->devdata;
		kfree(td_info);
		thermal_zone_device_unregister(pdata->tzd[i]);
	}
	platform_set_drvdata(pdev, NULL);
	kfree(pdata);
	return ret;
}

static int soc_thermal_remove(struct platform_device *pdev)
{
	int i;
	struct platform_soc_data *pdata = platform_get_drvdata(pdev);

	/* Unregister each sensor with the generic thermal framework */
	for (i = 0; i < SOC_THERMAL_SENSORS; i++) {
		struct thermal_device_info *td_info = pdata->tzd[i]->devdata;
		kfree(td_info);
		thermal_zone_device_unregister(pdata->tzd[i]);
	}
	platform_set_drvdata(pdev, NULL);
	free_irq(pdata->irq, pdata);
	kfree(pdata);

	remove_soc_dts_debugfs();

	return 0;
}

static const struct platform_device_id therm_id_table[] = {
	{ DRIVER_NAME, 1},
};

static struct platform_driver soc_thermal_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
	},
	.probe = soc_thermal_probe,
	.remove = soc_thermal_remove,
	.id_table = therm_id_table,
};

static int __init soc_thermal_module_init(void)
{
	return platform_driver_register(&soc_thermal_driver);
}

static void __exit soc_thermal_module_exit(void)
{
	platform_driver_unregister(&soc_thermal_driver);
}

module_init(soc_thermal_module_init);
module_exit(soc_thermal_module_exit);

MODULE_AUTHOR("Shravan B M <shravan.k.b.m@intel.com>");
MODULE_DESCRIPTION("Intel SoC Thermal Driver");
MODULE_LICENSE("GPL");
