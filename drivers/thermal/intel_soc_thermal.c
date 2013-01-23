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

#include <asm/intel-mid.h>

#define DRIVER_NAME	"intel_soc_thermal"

/* SOC DTS Registers */
#define SOC_THERMAL_SENSORS	2
#define PUNIT_PORT		0x04
#define DTS_ENABLE_REG		0xB0
#define PUNIT_TEMP_REG		0xB1
#define PUNIT_AUX_REG		0xB2
#define DTS_ENABLE		0x02
/* There are 4 Aux trips. Only Aux0, Aux1 are writeable */
#define DTS_TRIP_RW		0x03

#define TJMAX_TEMP		90
#define TJMAX_CODE		0x7F

struct platform_soc_data {
	struct thermal_zone_device *tzd[SOC_THERMAL_SENSORS];
};

struct thermal_device_info {
	int sensor_index;
	struct mutex lock_aux;
};

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

static inline u32 read_soc_reg(unsigned int addr)
{
	return intel_mid_msgbus_read32(PUNIT_PORT, addr);
}

static inline void write_soc_reg(unsigned int addr, u32 val)
{
	intel_mid_msgbus_write32(PUNIT_PORT, addr, val);
}

static void enable_soc_dts(void)
{
	/* Enable the DTS */
	write_soc_reg(DTS_ENABLE_REG, DTS_ENABLE);
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

	/* Enable DTS0 and DTS1 */
	enable_soc_dts();

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
	kfree(pdata);

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
	int ret;
	struct platform_device *pd;

	ret = platform_driver_register(&soc_thermal_driver);
	if (ret) {
		pr_err("Platform driver register failed:%d\n", ret);
		return ret;
	}

	/* TODO: Remove this code when we get an SFI table entry */
	pd = platform_device_register_simple(DRIVER_NAME, -1, NULL, 0);
	if (IS_ERR(pd)) {
		ret = PTR_ERR(pd);
		pr_err("Platform device register failed with %d\n", ret);
		platform_driver_unregister(&soc_thermal_driver);
	}

	return ret;
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
