/*
 * intel_mid_thermal.c - Intel MID platform thermal driver
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
 * Author: Ananth Krishna <ananth.krishna.r@intel.com>
 * Author: Durgadoss <durgadoss.r@intel.com>
 */

#define pr_fmt(fmt)  "intel_mid_thermal: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/ipc_device.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/thermal.h>

#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_gpadc.h>

#ifdef CONFIG_BOARD_CTP

/* Number of thermal sensors */
#define MSIC_THERMAL_SENSORS	4

static char *name[MSIC_THERMAL_SENSORS] = {
	"skin0", "skin1", "msicdie", "bptherm"
};

/*Skin sensors attributes*/
#define SKIN0_INDEX		0
#define SKIN1_INDEX		1
#define MSIC_DIE_INDEX		2
#define BPTHERM_INDEX		3
#define SKIN1_SLOPE		723
#define SKIN1_INTERCEPT		7084
#define SKIN0_SLOPE		446
#define SKIN0_INTERCEPT		14050
#define BPTHERM_SLOPE		788
#define BPTHERM_INTERCEPT	5065
#else

/* Number of thermal sensors */
#define MSIC_THERMAL_SENSORS	4
#define MSIC_DIE_INDEX		3
#define SKIN0_INDEX		0
#define SKIN1_INDEX		1
#define SKIN1_SLOPE		806
#define SKIN1_INTERCEPT		1800
#define SKIN0_SLOPE		851
#define SKIN0_INTERCEPT		2800
static char *name[MSIC_THERMAL_SENSORS] = {
	"skin0", "skin1", "sys", "msicdie"
};
#endif

/* MSIC die attributes */
#define MSIC_DIE_ADC_MIN	488
#define MSIC_DIE_ADC_MAX	1004

/* Convert adc_val to die temperature (in milli degree celsius) */
#define TO_MSIC_DIE_TEMP(adc_val)	(368 * adc_val - 219560)

/* Convert Thermistor temperature to skin0/skin1 temperature */
#define SKIN0_TEMP(temp) ((temp * SKIN0_SLOPE)/1000 + SKIN0_INTERCEPT)
#define SKIN1_TEMP(temp) ((temp * SKIN1_SLOPE)/1000 + SKIN1_INTERCEPT)

#define TABLE_LENGTH 24
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

struct ts_cache_info {
	bool is_cached_data_initialized;
	struct mutex lock;
	int cached_values[MSIC_THERMAL_SENSORS];
	unsigned long last_updated;
};

struct ipc_info {
	struct ipc_device *ipcdev;
	struct thermal_zone_device *tzd[MSIC_THERMAL_SENSORS];
	struct ts_cache_info cacheinfo;
	/* ADC handle used to read sensor temperature values */
	void *therm_adc_handle;
};

static struct ipc_info *ipcinfo;

struct thermal_device_info {
	/* if 1, ADC code to temperature conversion is direct. i.e. no linear
	 * approximation is needed */
	int direct;
	int sensor_index;
};


/**
 * is_valid_adc - checks whether the adc code is within the defined range
 * @min: minimum value for the sensor
 * @max: maximum value for the sensor
 *
 * Can sleep
 */
static int is_valid_adc(uint16_t adc_val, uint16_t min, uint16_t max)
{
	return (adc_val >= min) && (adc_val <= max);
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
	return -1;
}

/**
 * linear_interpolate - does interpolation to find temperature
 * Returns the temperature in milli degree celsius
 * @adc_val: ADC code(x) at which temperature(y) should be found
 * @indx: index of the minimum(x0) of the two ADC codes
 *
 * Can sleep
 */
static int linear_interpolate(int indx, uint16_t adc_val)
{
	int x = adc_val;
	int x0 = adc_code[0][indx];
	int x1 = adc_code[0][indx - 1];
	int y0 = adc_code[1][indx];
	int y1 = adc_code[1][indx - 1];

	/*
	 * Find y:
	 * Of course, we can avoid these variables, but keep them
	 * for readability and maintainability.
	 */
	int numerator = (x-x0)*y1 + (x1-x)*y0;
	int denominator = x1-x0;

	/*
	 * We have to report the temperature in milli degree celsius.
	 * So, to reduce the loss of precision, do (Nr*1000)/Dr, instead
	 * of (Nr/Dr)*1000.
	 */
	 return (numerator * 1000)/denominator;
}

/**
 * adc_to_temp - converts the ADC code to temperature in C
 * @direct: true if ths channel is direct index
 * @adc_val: the adc_val that needs to be converted
 * @tp: temperature return value
 *
 * Can sleep
 */
static int adc_to_temp(int direct, uint16_t adc_val, unsigned long *tp)
{
	int indx;

	/* Direct conversion for msic die temperature */
	if (direct) {
		if (is_valid_adc(adc_val, MSIC_DIE_ADC_MIN, MSIC_DIE_ADC_MAX)) {
			*tp = TO_MSIC_DIE_TEMP(adc_val);
			return 0;
		}
		return -ERANGE;
	}

	indx = find_adc_code(adc_val);
	if (indx < 0)
		return -ERANGE;

	if (adc_code[0][indx] == adc_val) {
		/* Convert temperature in celsius to milli degree celsius */
		*tp = adc_code[1][indx] * 1000;
		return 0;
	}

	/*
	 * The ADC code is in between two values directly defined in the
	 * table. So, do linear interpolation to calculate the temperature.
	 */
	*tp = linear_interpolate(indx, adc_val);
	return 0;
}

/**
 * mid_read_temp - read sensors for temperature
 * @temp: holds the current temperature for the sensor after reading
 *
 * reads the adc_code from the channel and converts it to real
 * temperature. The converted value is stored in temp.
 *
 * Can sleep
 */
static int mid_read_temp(struct thermal_zone_device *tzd, unsigned long *temp)
{
	struct thermal_device_info *td_info = tzd->devdata;
	int ret;
	long curr_temp, bp_temp = 0;
	int indx = td_info->sensor_index; /* Required Index */

	mutex_lock(&ipcinfo->cacheinfo.lock);

	if (!ipcinfo->cacheinfo.is_cached_data_initialized ||
	time_after(jiffies, ipcinfo->cacheinfo.last_updated + HZ)) {
		ret = intel_mid_gpadc_sample(
			ipcinfo->therm_adc_handle, 1,
			&ipcinfo->cacheinfo.cached_values[0],
			&ipcinfo->cacheinfo.cached_values[1],
			&ipcinfo->cacheinfo.cached_values[2],
			&ipcinfo->cacheinfo.cached_values[3]);
		if (ret)
			goto exit;
		ipcinfo->cacheinfo.last_updated = jiffies;
		ipcinfo->cacheinfo.is_cached_data_initialized = true;
	}

	/* Convert ADC value to temperature */
	ret = adc_to_temp(td_info->direct,
		ipcinfo->cacheinfo.cached_values[indx],
		&curr_temp);
	if (ret)
		goto exit;

	switch (indx) {
	case SKIN0_INDEX:
		*temp = SKIN0_TEMP(curr_temp);
		break;
	case SKIN1_INDEX:
#ifdef CONFIG_BOARD_CTP
		ret = adc_to_temp(td_info->direct,
			ipcinfo->cacheinfo.cached_values[BPTHERM_INDEX],
			&bp_temp);
		if (ret)
			goto exit;
		bp_temp = (bp_temp * BPTHERM_SLOPE)/1000 + BPTHERM_INTERCEPT;
#endif
		*temp = SKIN1_TEMP(curr_temp);

		/* Max(Back Skin Thermistor temp, BPTHERM value) */
		if (bp_temp > *temp)
			*temp = bp_temp;
		break;
	default:
		*temp = curr_temp;
	}

exit:
	mutex_unlock(&ipcinfo->cacheinfo.lock);
	return ret;
}

/**
 * initialize_sensor - Initializes ADC information for each sensor.
 * @index: index of the sensor
 *
 * Context: can sleep
 */
static struct thermal_device_info *initialize_sensor(int index)
{
	struct thermal_device_info *td_info =
		kzalloc(sizeof(struct thermal_device_info), GFP_KERNEL);

	if (!td_info)
		return NULL;

	td_info->sensor_index = index;
	/* Direct conversion for MSIC_DIE */
	if (index == MSIC_DIE_INDEX)
		td_info->direct = 1;
	return td_info;
}

/**
 * mid_thermal_resume - resume routine
 * @dev: device structure
 */
static int mid_thermal_resume(struct device *dev)
{
	return 0;
}

/**
 * mid_thermal_suspend - suspend routine
 * @dev: device structure
 */
static int mid_thermal_suspend(struct device *dev)
{
	return 0;
}

/**
 * read_curr_temp - reads the current temperature and stores in temp
 * @temp: holds the current temperature value after reading
 *
 * Can sleep
 */
static int read_curr_temp(struct thermal_zone_device *tzd, unsigned long *temp)
{
	return (tzd) ? mid_read_temp(tzd, temp) : -EINVAL;
}

/* Can't be const */
static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = read_curr_temp,
};


/**
 * mid_thermal_probe - mfld thermal initialize
 * @ipcdev: ipc device structure
 *
 * mid thermal probe initializes the hardware and registers
 * all the sensors with the generic thermal framework. Can sleep.
 */
static int mid_thermal_probe(struct ipc_device *ipcdev)
{
	int ret;
	int i;
	ipcinfo = kzalloc(sizeof(struct ipc_info), GFP_KERNEL);
	if (!ipcinfo)
		return -ENOMEM;

	/* initialize mutex locks */
	mutex_init(&ipcinfo->cacheinfo.lock);

#ifdef CONFIG_BOARD_CTP
	/* Allocate ADC channels for all sensors */
	ipcinfo->therm_adc_handle = intel_mid_gpadc_alloc(MSIC_THERMAL_SENSORS,
					0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
					0x04 | CH_NEED_VREF | CH_NEED_VCALIB,
					0x03 | CH_NEED_VCALIB,
					0x09 | CH_NEED_VREF | CH_NEED_VCALIB);
#else
	/* Allocate ADC channels for all sensors */
	ipcinfo->therm_adc_handle = intel_mid_gpadc_alloc(MSIC_THERMAL_SENSORS,
					0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
					0x08 | CH_NEED_VREF | CH_NEED_VCALIB,
					0x0A | CH_NEED_VREF | CH_NEED_VCALIB,
					0x03 | CH_NEED_VCALIB);
#endif
	if (!ipcinfo->therm_adc_handle) {
		ret = -ENOMEM;
		goto alloc_fail;
	}

	/* Register each sensor with the generic thermal framework*/
	for (i = 0; i < MSIC_THERMAL_SENSORS; i++) {
		ipcinfo->tzd[i] = thermal_zone_device_register(name[i],
					0, 0, initialize_sensor(i),
					&tzd_ops, 0, 0, 0, 0);
		if (IS_ERR(ipcinfo->tzd[i]))
			goto reg_fail;
	}

	ipcinfo->ipcdev = ipcdev;
	ipc_set_drvdata(ipcdev, ipcinfo);
	return 0;

reg_fail:
	ret = PTR_ERR(ipcinfo->tzd[i]);
	while (--i >= 0)
		thermal_zone_device_unregister(ipcinfo->tzd[i]);
alloc_fail:
	kfree(ipcinfo);
	return ret;
}

/**
 * mid_thermal_remove - mfld thermal finalize
 * @dev: ipc device structure
 *
 * MLFD thermal remove unregisters all the sensors from the generic
 * thermal framework. Can sleep.
 */
static int mid_thermal_remove(struct ipc_device *ipcdev)
{
	int i;

	for (i = 0; i < MSIC_THERMAL_SENSORS; i++)
		thermal_zone_device_unregister(ipcinfo->tzd[i]);

	/* Free the allocated ADC channels */
	if (ipcinfo->therm_adc_handle)
		intel_mid_gpadc_free(ipcinfo->therm_adc_handle);

	kfree(ipcinfo);
	ipc_set_drvdata(ipcdev, NULL);

	return 0;
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

#define DRIVER_NAME "msic_thermal"

static const struct ipc_device_id therm_id_table[] = {
	{ DRIVER_NAME, 1 },
};

static const struct dev_pm_ops msic_thermal_pm_ops = {
	.suspend = mid_thermal_suspend,
	.resume = mid_thermal_resume,
};
static struct ipc_driver mid_thermal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &msic_thermal_pm_ops,
	},
	.probe = mid_thermal_probe,
	.remove = __devexit_p(mid_thermal_remove),
	.id_table = therm_id_table,
};

static int __init mid_thermal_module_init(void)
{
	return ipc_driver_register(&mid_thermal_driver);
}

static void __exit mid_thermal_module_exit(void)
{
	ipc_driver_unregister(&mid_thermal_driver);
}

/* Changing _init call to make the thermal driver
 * load _after_ the GPADC driver
 * module_init(mid_thermal_module_init);
 */
late_initcall(mid_thermal_module_init);
module_exit(mid_thermal_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Medfield Platform Thermal Driver");
MODULE_LICENSE("GPL");
