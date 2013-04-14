/*
 * intel_byt_thermal.c - Intel Baytrail Platform Thermal Driver
 *
 * Copyright (C) 2013 Intel Corporation
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Durgadoss R <durgadoss.r@intel.com>
 */

#define pr_fmt(fmt)  "intel_byt_thermal: " fmt

#include <linux/pm.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/mfd/intel_mid_pmic.h>
#include <asm/intel_crystalcove_gpadc.h>

#include "../staging/iio/consumer.h"

#define DEVICE_NAME "crystal_cove_thermal"

/* Number of Thermal sensors on the PMIC */
#define PMIC_THERMAL_SENSORS	4

/* Registers that govern Thermal Monitoring */
#define THRMIRQ0	0x04
#define THRMIRQ1	0x05
#define MTHRMIRQ0	0x11
#define MTHRMIRQ1	0x12
#define IRQLVL1		0x02
#define MIRQLVL1	0x0E
#define IRQ_MASK_ALL	0x0F

#define THRM_MON_CTRL0	0x8E
#define THRM_MON_CTRL1	0x8F
#define TS_ENABLE	0x90
#define TS_CRIT_ENABLE	0x91
#define TS_A0_STS	0x92
#define TS_A1_STS	0x93

#define PMICALRT	(1 << 3)
#define SYS2ALRT	(1 << 2)
#define SYS1ALRT	(1 << 1)
#define SYS0ALRT	(1 << 0)
#define THERM_EN	(1 << 0)
#define ALERT_EN	(1 << 6)
#define TS_ENABLE_ALL	0x27

/* ADC to Temperature conversion table length */
#define TABLE_LENGTH	24
#define TEMP_INTERVAL	5

/* Default Alert threshold 85 C */
#define DEFAULT_MAX_TEMP	85
#define MIN_CRIT_TEMP		55

#define NUM_ALERT_LEVELS	3
#define ALERT_RW_MASK		0x07
#define LEVEL_ALERT0		0
#define LEVEL_ALERT1		1
#define LEVEL_ALERT2		2

/* Constants defined in CrystalCove PMIC spec */
#define PMIC_DIE_SENSOR		3
#define PMIC_DIE_ADC_MIN	448
#define PMIC_DIE_ADC_MAX	1004
#define PMIC_DIE_TEMP_MIN	-40
#define PMIC_DIE_TEMP_MAX	125

/* 'enum' of Thermal sensors */
enum thermal_sensors { SYS0, SYS1, SYS2, PMIC_DIE, _COUNT };

/*
 * Alert registers store the 'alert' temperature for each sensor,
 * as 10 bit ADC code. The higher two bits are stored in bits[0:1] of
 * alert_regs_h. The lower eight bits are stored in alert_regs_l.
 * The hysteresis value is stored in bits[2:5] of alert_regs_h.
 * Alert level 2 (also known as 'critical level') is 8 bits wide
 * and hence does not have a 'high' register.
 *
 * static const int alert_regs_h[3][4] = {
 *			SYS0, SYS1, SYS2, PMIC_DIE
 *		Alert 0	{ 0x94, 0x99, 0x9E, 0xAF },
 *		Alert 1	{ 0x96, 0x9B, 0xA0, 0xB1 },
 *		Alert 2	{    -,    -,    -,    - },
 *			};
 */
static const int alert_regs_l[3][4] = {
				/* SYS0, SYS1, SYS2, PMIC_DIE */
		/* Alert 0 */	{ 0x95, 0x9A, 0x9F, 0xB0 },
		/* Alert 1 */	{ 0x97, 0x9C, 0xA1, 0xB2 },
		/* Alert 2 */	{ 0x98, 0x9D, 0xA2, 0xB3 },
				};
/*
 * ADC code vs Temperature table
 * This table will be different for different thermistors
 * Row 0: ADC code
 * Row 1: Temperature (in degree celsius)
 */
static const int adc_code[2][TABLE_LENGTH] = {
	{977, 961, 941, 917, 887, 853, 813, 769, 720, 669, 615, 561, 508, 456,
		407, 357, 315, 277, 243, 212, 186, 162, 140, 107},
	{-20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60,
		65, 70, 75, 80, 85, 90, 100},
	};

static DEFINE_MUTEX(thrm_update_lock);

struct thermal_device_info {
	int sensor_index;
	/* If 'direct', no table lookup is required */
	bool is_direct;
};

struct thermal_data {
	struct platform_device *pdev;
	struct iio_channel *iio_chan;
	struct thermal_zone_device *tzd[PMIC_THERMAL_SENSORS];
	unsigned int irq;
	/* Caching information */
	bool is_initialized;
	unsigned long last_updated;
	int cached_vals[PMIC_THERMAL_SENSORS];
};
static struct thermal_data *tdata;

static inline int adc_to_pmic_die_temp(unsigned int val)
{
	/* return temperature in mC */
	return val * 884 - 588640;
}

static inline int pmic_die_temp_to_adc(int temp)
{
	/* 'temp' is in C, convert to mC and then do calculations */
	return ((temp * 1000) + 588640) / 884;
}

/**
 * find_adc_code - searches the ADC code using binary search
 * @val: value to find in the array
 *
 * This function does binary search on an array sorted in 'descending' order
 * Can sleep
 */
static int find_adc_code(uint16_t val)
{
	int left = 0;
	int right = TABLE_LENGTH - 1;
	int mid;
	while (left <= right) {
		mid = (left + right)/2;
		if (val == adc_code[0][mid] ||
			(mid > 0 &&
			val > adc_code[0][mid] && val < adc_code[0][mid-1]))
			return mid;
		else if (val > adc_code[0][mid])
			right = mid - 1;
		else if (val < adc_code[0][mid])
			left = mid + 1;
	}
	return -EINVAL;
}

/**
 * adc_to_temp - converts the ADC code to temperature in mC
 * @direct: true if the sensor uses direct conversion
 * @adc_val: the ADC code to be converted
 * @tp: temperature return value
 *
 * Can sleep
 */
static int adc_to_temp(int direct, uint16_t adc_val, long *tp)
{
	int x0, x1, y0, y1;
	int nr, dr;		/* Numerator & Denominator */
	int indx;
	int x = adc_val;

	/* Direct conversion for pmic die temperature */
	if (direct) {
		if (adc_val < PMIC_DIE_ADC_MIN || adc_val > PMIC_DIE_ADC_MAX)
			return -EINVAL;

		*tp = adc_to_pmic_die_temp(adc_val);
		return 0;
	}

	indx = find_adc_code(adc_val);
	if (indx < 0)
		return -EINVAL;

	if (adc_code[0][indx] == adc_val) {
		*tp = adc_code[1][indx] * 1000;
		return 0;
	}

	/*
	 * The ADC code is in between two values directly defined in the
	 * table. So, do linear interpolation to calculate the temperature.
	 */
	x0 = adc_code[0][indx];
	x1 = adc_code[0][indx - 1];
	y0 = adc_code[1][indx];
	y1 = adc_code[1][indx - 1];

	/*
	 * Find y:
	 * Of course, we can avoid these variables, but keep them
	 * for readability and maintainability.
	 */
	nr = (x-x0)*y1 + (x1-x)*y0;
	dr =  x1-x0;

	if (!dr)
		return -EINVAL;
	/*
	 * We have to report the temperature in milli degree celsius.
	 * So, to reduce the loss of precision, do (Nr*1000)/Dr, instead
	 * of (Nr/Dr)*1000.
	 */
	*tp = (nr * 1000)/dr;

	return 0;
}

/**
 * temp_to_adc - converts the temperature(in C) to ADC code
 * @direct: true if the sensor uses direct conversion
 * @temp: the temperature to be converted
 * @adc_val: ADC code return value
 *
 * Can sleep
 */
static int temp_to_adc(int direct, int temp, int *adc_val)
{
	int indx;
	int x0, x1, y0, y1;
	int nr, dr;		/* Numerator & Denominator */
	int x = temp;

	/* Direct conversion for pmic die temperature */
	if (direct) {
		if (temp < PMIC_DIE_TEMP_MIN || temp > PMIC_DIE_TEMP_MAX)
			return -EINVAL;

		*adc_val = pmic_die_temp_to_adc(temp);
		return 0;
	}

	if (temp < adc_code[1][0] || temp > adc_code[1][TABLE_LENGTH - 1])
		return -EINVAL;


	/* Find the 'indx' of this 'temp' in the table */
	indx = (temp - adc_code[1][0]) / TEMP_INTERVAL;

	if (temp == adc_code[1][indx]) {
		*adc_val = adc_code[0][indx];
		return 0;
	}

	/*
	 * Temperature is not a multiple of 'TEMP_INTERVAL'. So,
	 * do linear interpolation to obtain a better ADC code.
	 */
	x0 = adc_code[1][indx];
	x1 = adc_code[1][indx + 1];
	y0 = adc_code[0][indx];
	y1 = adc_code[0][indx + 1];

	nr = (x-x0)*y1 + (x1-x)*y0;
	dr =  x1-x0;

	if (!dr)
		return -EINVAL;

	*adc_val = nr/dr;

	return 0;
}

/**
 * set_alert_temp - sets the given 'adc_val' to the 'alert_reg'
 * @alert_reg_l: The 'low' register address
 * @adc_val:     ADC value to be programmed
 * @level:       0 - alert0, 1 - alert1, 2 - alert2(only 8 bits wide)
 *
 * Not protected. Calling function should handle synchronization.
 * Can sleep
 */
static int set_alert_temp(int alert_reg_l, int adc_val, int level)
{
	int ret;

	/*
	 * The alert register stores B[1:8] of val and the HW
	 * while comparing prefixes and suffixes this value with
	 * a 0; i.e B[0] and B[9] are 0.
	 */
	if (level == LEVEL_ALERT2) {
		adc_val = (adc_val & 0x1FF) >> 1;
		return intel_mid_pmic_writeb(alert_reg_l, adc_val);
	}

	/* Extract bits[0:7] of 'adc_val' and write them into alert_reg_l */
	ret = intel_mid_pmic_writeb(alert_reg_l, adc_val & 0xFF);
	if (ret < 0 || level == LEVEL_ALERT2)
		return ret;

	/* Get the address of alert_reg_h */
	--alert_reg_l;

	ret = intel_mid_pmic_readb(alert_reg_l);
	if (ret < 0)
		return ret;

	/* Set bits[0:1] of alert_reg_h to bits[8:9] of 'adc_val' */
	ret = (ret & ~0x03) | (adc_val >> 8);

	return intel_mid_pmic_writeb(alert_reg_l, ret);
}

/**
 * get_alert_temp - gets the ADC code from the alert register
 * @alert_reg_l: The 'low' register address
 * @level:       0 - alert0, 1 - alert1, 2 - alert2(only 8 bits wide)
 *
 * Not protected. Calling function should handle synchronization.
 * Can sleep
 */
static int get_alert_temp(int alert_reg_l, int level)
{
	int l, h;

	l = intel_mid_pmic_readb(alert_reg_l);
	if (l < 0)
		return l;

	if (level == LEVEL_ALERT2)
		return l << 1;

	/* Get the address of alert_reg_h */
	--alert_reg_l;

	h = intel_mid_pmic_readb(alert_reg_l);
	if (h < 0)
		return h;

	/* Concatenate 'h' and 'l' to get 10-bit ADC code */
	return ((h & 0x03) << 8) | l;
}

/**
 * program_tmax - programs a default _max value for each sensor
 * @dev: device pointer
 *
 * Can sleep
 */
static int program_tmax(struct device *dev)
{
	int i, ret, level;
	int pmic_die_val, adc_val, val;

	ret = temp_to_adc(1, DEFAULT_MAX_TEMP, &pmic_die_val);
	if (ret)
		return ret;

	/* ADC code corresponding to max Temp 85 C */
	ret = temp_to_adc(0, DEFAULT_MAX_TEMP, &adc_val);
	if (ret)
		return ret;

	for (level = 0; level <= LEVEL_ALERT2; level++) {
		for (i = 0; i < PMIC_THERMAL_SENSORS; i++) {
			val = (i == PMIC_DIE_SENSOR) ? pmic_die_val : adc_val;

			ret = set_alert_temp(alert_regs_l[level][i],
						val, level);
			if (ret < 0)
				goto exit_err;
		}
	}

	return ret;

exit_err:
	dev_err(dev, "set alert %d for channel %d failed:%d\n", level, i, ret);
	return ret;
}

static ssize_t store_trip_hyst(struct thermal_zone_device *tzd,
				int trip, long hyst)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg = alert_regs_l[trip][td_info->sensor_index] - 1;

	/*
	 * Alert level 2 does not support hysteresis; and (for
	 * other levels) the hysteresis value is 4 bits wide.
	 */
	if (trip == LEVEL_ALERT2 || hyst > 15)
		return -EINVAL;

	mutex_lock(&thrm_update_lock);

	ret = intel_mid_pmic_readb(alert_reg);
	if (ret < 0)
		goto exit;

	/* Set bits [2:5] to value of hyst */
	ret = (ret & 0xC3) | (hyst << 2);

	ret = intel_mid_pmic_writeb(alert_reg, ret);

exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_trip_hyst(struct thermal_zone_device *tzd,
				int trip, long *hyst)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg_l = alert_regs_l[trip][td_info->sensor_index];

	if (trip == LEVEL_ALERT2) {
		*hyst = 0;
		return 0;
	}

	mutex_lock(&thrm_update_lock);

	/* Get the address of alert_reg_h */
	--alert_reg_l;

	ret = intel_mid_pmic_readb(alert_reg_l);
	if (ret >= 0)
		*hyst = (ret >> 2) & 0x0F; /* Extract bits[2:5] of data */

	mutex_unlock(&thrm_update_lock);

	return 0;
}

static ssize_t store_trip_temp(struct thermal_zone_device *tzd,
				int trip, long trip_temp)
{
	int ret, adc_val;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg_l = alert_regs_l[trip][td_info->sensor_index];

	if (trip_temp < 1000) {
		dev_err(&tzd->device, "Temperature should be in mC\n");
		return -EINVAL;
	}

	/* Convert from mC to C */
	trip_temp /= 1000;

	if (trip == LEVEL_ALERT2 && trip_temp < MIN_CRIT_TEMP) {
		dev_err(&tzd->device, "Tcrit should be more than 55C\n");
		return -EINVAL;
	}

	mutex_lock(&thrm_update_lock);

	ret = temp_to_adc(td_info->is_direct, (int)trip_temp, &adc_val);
	if (ret)
		goto exit;

	ret = set_alert_temp(alert_reg_l, adc_val, trip);
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_trip_temp(struct thermal_zone_device *tzd,
				int trip, long *trip_temp)
{
	int ret = -EINVAL, adc_val;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg_l = alert_regs_l[trip][td_info->sensor_index];

	mutex_lock(&thrm_update_lock);

	adc_val = get_alert_temp(alert_reg_l, trip);
	if (adc_val < 0)
		goto exit;

	ret = adc_to_temp(td_info->is_direct, adc_val, trip_temp);
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_trip_type(struct thermal_zone_device *tzd,
			int trip, enum thermal_trip_type *trip_type)
{
	/* All are passive trip points */
	*trip_type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static ssize_t show_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;
	int indx = td_info->sensor_index;

	if (!tdata->iio_chan)
		return -EINVAL;

	mutex_lock(&thrm_update_lock);

	if (!tdata->is_initialized ||
			time_after(jiffies, tdata->last_updated + HZ)) {
		ret = iio_st_read_channel_all_raw(tdata->iio_chan,
						tdata->cached_vals);
		if (ret) {
			dev_err(&tzd->device, "ADC sampling failed:%d\n", ret);
			goto exit;
		}
		tdata->last_updated = jiffies;
		tdata->is_initialized = true;
	}

	ret = adc_to_temp(td_info->is_direct, tdata->cached_vals[indx], temp);
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static int enable_tm(void)
{
	int ret;

	mutex_lock(&thrm_update_lock);

	/* Setting these bits enables ADC to poll for these Thermistors */
	ret = intel_mid_pmic_readb(TS_ENABLE);
	if (ret < 0)
		goto exit;

	ret = intel_mid_pmic_writeb(TS_ENABLE, ret | TS_ENABLE_ALL);
	if (ret < 0)
		goto exit;

exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static struct thermal_device_info *initialize_sensor(int index)
{
	struct thermal_device_info *td_info =
		kzalloc(sizeof(struct thermal_device_info), GFP_KERNEL);

	if (!td_info)
		return NULL;

	td_info->sensor_index = index;

	if (index == PMIC_DIE_SENSOR)
		td_info->is_direct = true;

	return td_info;
}

static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = show_temp,
	.get_trip_type = show_trip_type,
	.get_trip_temp = show_trip_temp,
	.set_trip_temp = store_trip_temp,
	.get_trip_hyst = show_trip_hyst,
	.set_trip_hyst = store_trip_hyst,
};

static int byt_thermal_probe(struct platform_device *pdev)
{
	int ret, i;
	static char *name[PMIC_THERMAL_SENSORS] = {
			"SYSTHERM0", "SYSTHERM1", "SYSTHERM2", "PMICDIE" };

	tdata = kzalloc(sizeof(struct thermal_data), GFP_KERNEL);
	if (!tdata) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	tdata->pdev = pdev;
	platform_set_drvdata(pdev, tdata);

	/* Program a default _max value for each sensor */
	ret = program_tmax(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Programming _max failed:%d\n", ret);
		goto exit_free;
	}

	/* Register with IIO to sample temperature values */
	tdata->iio_chan = iio_st_channel_get_all("THERMAL");
	if (tdata->iio_chan == NULL) {
		dev_err(&pdev->dev, "tdata->iio_chan is null\n");
		ret = -EINVAL;
		goto exit_free;
	}

	/* Check whether we got all the four channels */
	ret = iio_st_channel_get_num(tdata->iio_chan);
	if (ret != PMIC_THERMAL_SENSORS) {
		dev_err(&pdev->dev, "incorrect number of channels:%d\n", ret);
		ret = -EFAULT;
		goto exit_iio;
	}

	/* Register each sensor with the generic thermal framework */
	for (i = 0; i < PMIC_THERMAL_SENSORS; i++) {
		tdata->tzd[i] = thermal_zone_device_register(name[i],
					NUM_ALERT_LEVELS, ALERT_RW_MASK,
					initialize_sensor(i), &tzd_ops,
					0, 0, 0, 0);
		if (IS_ERR(tdata->tzd[i])) {
			ret = PTR_ERR(tdata->tzd[i]);
			dev_err(&pdev->dev,
				"registering thermal sensor %s failed: %d\n",
				name[i], ret);
			goto exit_reg;
		}
	}

	/* Enable Thermal Monitoring */
	ret = enable_tm();
	if (ret) {
		dev_err(&pdev->dev, "Enabling TM failed:%d\n", ret);
		goto exit_reg;
	}

	return 0;

exit_reg:
	while (--i >= 0)
		thermal_zone_device_unregister(tdata->tzd[i]);
exit_iio:
	iio_st_channel_release_all(tdata->iio_chan);
exit_free:
	kfree(tdata);
	return ret;
}

static int byt_thermal_resume(struct device *dev)
{
	dev_info(dev, "resume called.\n");
	return 0;
}

static int byt_thermal_suspend(struct device *dev)
{
	dev_info(dev, "suspend called.\n");
	return 0;
}

static int byt_thermal_remove(struct platform_device *pdev)
{
	int i;
	struct thermal_data *tdata = platform_get_drvdata(pdev);

	if (!tdata)
		return 0;

	for (i = 0; i < PMIC_THERMAL_SENSORS; i++)
		thermal_zone_device_unregister(tdata->tzd[i]);

	iio_st_channel_release_all(tdata->iio_chan);
	kfree(tdata);
	return 0;
}

/*********************************************************************
 *		Driver initialization and finalization
 *********************************************************************/

static const struct dev_pm_ops thermal_pm_ops = {
	.suspend = byt_thermal_suspend,
	.resume = byt_thermal_resume,
};

static struct platform_driver byt_thermal_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.pm = &thermal_pm_ops,
		},
	.probe = byt_thermal_probe,
	.remove = byt_thermal_remove,
};

static int byt_thermal_module_init(void)
{
	return platform_driver_register(&byt_thermal_driver);
}

static void byt_thermal_module_exit(void)
{
	platform_driver_unregister(&byt_thermal_driver);
}

late_initcall(byt_thermal_module_init);
module_exit(byt_thermal_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Baytrail Platform Thermal Driver");
MODULE_LICENSE("GPL");
