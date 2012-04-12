/*
 * intel_mrfl_thermal.c - Intel Merrifield Platform Thermal Driver
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
 * DEVICE_NAME: Intel Merrifield platform - PMIC: Thermal Monitor
 */

#define pr_fmt(fmt)  "intel_mrfl_thermal: " fmt

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
#include <asm/intel_basincove_gpadc.h>

#define DRIVER_NAME "mrfl_thermal"
#define DEVICE_NAME "mrfl_pmic_thermal"

/* Number of Thermal sensors on the PMIC */
#define PMIC_THERMAL_SENSORS	4

/* Registers that govern Thermal Monitoring */
#define THRMMONCFG	0xB3
#define THRMMONCTL	0xB4
#define THRMIRQ		0x04
#define MTHRMIRQ	0x0F
#define STHRMIRQ	0xB2
#define IRQ_MASK_ALL	0x0F

#define PMICALRT	(1 << 3)
#define SYS2ALRT	(1 << 2)
#define SYS1ALRT	(1 << 1)
#define SYS0ALRT	(1 << 0)
#define THERM_EN	(1 << 0)

/* ADC to Temperature conversion table length */
#define TABLE_LENGTH	24
#define TEMP_INTERVAL	5

#define PMIC_DIE_SENSOR		3
#define PMIC_DIE_ADC_MIN	559
#define PMIC_DIE_ADC_MAX	1004
#define PMIC_DIE_TEMP_MIN	-40
#define PMIC_DIE_TEMP_MAX	150

/* Convert adc_val to die temperature (in milli degree celsius) */
#define TO_PMIC_DIE_TEMP(adc_val)	(368 * adc_val - 219560)

/* Convert temperature in Celsius to ADC Code */
#define TO_PMIC_DIE_ADC(temp)	DIV_ROUND_CLOSEST((2717 * temp + 596630), 1000)


/* ADC channels for the sensors. Order: SYS0 SYS1 SYS2 PMIC_DIE */
static const int adc_channels[] = { GPADC_SYSTEMP0, GPADC_SYSTEMP1,
				GPADC_SYSTEMP2, GPADC_PMICTEMP };
/*
 * Alert registers store the 'alert' temperature for each sensor,
 * as 10 bit ADC code. The higher two bits are stored in bits[0:1] of
 * alert_regs_h. The lower eight bits are stored in alert_regs_l.
 * The hysteresis value is stored in bits[2:6] of alert_regs_h.
 * Order: SYS0 SYS1 SYS2 PMIC_DIE
 *
 * static const int alert_regs_l[] = { 0xB7, 0xB9, 0xBB, 0xC1 };
 */
static const int alert_regs_h[] = { 0xB6, 0xB8, 0xBA, 0xC0 };

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

struct thermal_data {
	struct device *hwmon_dev;
	struct platform_device *pdev;
	struct gpadc_result *adc_res;
	unsigned int irq;
};

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
static int adc_to_temp(int direct, uint16_t adc_val, int *tp)
{
	int x0, x1, y0, y1;
	int nr, dr;		/* Numerator & Denominator */
	int indx;
	int x = adc_val;

	/* Direct conversion for pmic die temperature */
	if (direct) {
		if (adc_val < PMIC_DIE_ADC_MIN || adc_val > PMIC_DIE_ADC_MAX)
			return -EINVAL;

		*tp = TO_PMIC_DIE_TEMP(adc_val);
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

		*adc_val = TO_PMIC_DIE_ADC(temp);
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

static ssize_t store_tmax_hyst(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	uint8_t data;
	unsigned long hyst;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);

	/* Hysteresis value is 5 bits wide */
	if (strict_strtoul(buf, 10, &hyst) || hyst > 31)
		return -EINVAL;

	mutex_lock(&thrm_update_lock);

	ret = intel_scu_ipc_ioread8(alert_regs_h[s_attr->nr], &data);
	if (ret)
		goto ipc_fail;

	/* Set bits [2:6] to value of hyst */
	data = (data & 0x83) | (hyst << 2);

	ret = intel_scu_ipc_iowrite8(alert_regs_h[s_attr->nr], data);
	if (ret)
		goto ipc_fail;

	ret = count;

ipc_fail:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_tmax_hyst(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret;
	uint8_t data;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);
	mutex_lock(&thrm_update_lock);

	ret = intel_scu_ipc_ioread8(alert_regs_h[s_attr->nr], &data);

	mutex_unlock(&thrm_update_lock);

	if (ret)
		return ret;

	/* Read bits [2:6] of data and show */
	return sprintf(buf, "%d\n", (data >> 2) & 0x1F);
}

static ssize_t store_tmax(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret, adc_val;
	uint8_t h;
	unsigned long temp;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);
	int alert_reg = alert_regs_h[s_attr->nr];
	int direct = (s_attr->nr == PMIC_DIE_SENSOR) ? 1 : 0;

	if (strict_strtoul(buf, 10, &temp))
		return -EINVAL;

	if (temp < 1000) {
		dev_err(dev, "Temperature should be in mC\n");
		return -EINVAL;
	}

	mutex_lock(&thrm_update_lock);

	/* Convert from mC to C */
	temp /= 1000;

	ret = temp_to_adc(direct, (int)temp, &adc_val);
	if (ret)
		goto exit;

	ret =  intel_scu_ipc_ioread8(alert_reg, &h);
	if (ret)
		goto exit;

	/* Set bits[0:1] of alert_reg_h to bits[8:9] of 'adc_val' */
	ret = intel_scu_ipc_iowrite8(alert_reg, h | (adc_val >> 8));
	if (ret)
		goto exit;

	/* Extract bits[0:7] of 'adc_val' and write them into alert_reg_l */
	ret = intel_scu_ipc_iowrite8(alert_reg + 1, adc_val & 0xFF);
	if (ret)
		goto exit;

	ret = count;
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_tmax(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, temp, adc_val;
	uint8_t l, h;
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);
	int alert_reg = alert_regs_h[s_attr->nr];
	int direct = (s_attr->nr == PMIC_DIE_SENSOR) ? 1 : 0;

	mutex_lock(&thrm_update_lock);

	ret = intel_scu_ipc_ioread8(alert_reg, &h);
	if (ret)
		goto exit;

	ret = intel_scu_ipc_ioread8(alert_reg + 1, &l);
	if (ret)
		goto exit;

	/* Concatenate 'h' and 'l' to get 10-bit ADC code */
	adc_val = ((h & 0x03) << 8) | l;

	ret = adc_to_temp(direct, adc_val, &temp);
	if (ret)
		goto exit;

	ret = sprintf(buf, "%d\n", temp);
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_temp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int ret, temp, ch, adc_val;
	struct thermal_data *tdata = dev_get_drvdata(dev);
	struct sensor_device_attribute_2 *s_attr =
					to_sensor_dev_attr_2(attr);
	int direct = (s_attr->nr == PMIC_DIE_SENSOR) ? 1 : 0;

	mutex_lock(&thrm_update_lock);

	ch = adc_channels[s_attr->nr];

	ret = intel_basincove_gpadc_sample(ch, tdata->adc_res);
	if (ret) {
		dev_err(dev, "gpadc_sample failed:%d\n", ret);
		goto exit;
	}

	adc_val = GPADC_RSL(ch, tdata->adc_res);

	ret = adc_to_temp(direct, adc_val, &temp);
	if (ret)
		goto exit;

	ret = sprintf(buf, "%d\n", temp);
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static int enable_tm(void)
{
	int ret;
	uint8_t data;

	mutex_lock(&thrm_update_lock);

	ret = intel_scu_ipc_ioread8(THRMMONCTL, &data);
	if (ret)
		goto ipc_fail;

	ret = intel_scu_ipc_iowrite8(THRMMONCTL, data | THERM_EN);

ipc_fail:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static SENSOR_DEVICE_ATTR_2(temp1_input, S_IRUGO,
				show_temp, NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(temp1_max, S_IRUGO | S_IWUSR,
				show_tmax, store_tmax, 0, 0);
static SENSOR_DEVICE_ATTR_2(temp1_max_hyst, S_IRUGO | S_IWUSR,
				show_tmax_hyst, store_tmax_hyst, 0, 0);

static SENSOR_DEVICE_ATTR_2(temp2_input, S_IRUGO,
				show_temp, NULL, 1, 0);
static SENSOR_DEVICE_ATTR_2(temp2_max, S_IRUGO | S_IWUSR,
				show_tmax, store_tmax, 1, 0);
static SENSOR_DEVICE_ATTR_2(temp2_max_hyst, S_IRUGO | S_IWUSR,
				show_tmax_hyst, store_tmax_hyst, 1, 0);

static SENSOR_DEVICE_ATTR_2(temp3_input, S_IRUGO,
				show_temp, NULL, 2, 0);
static SENSOR_DEVICE_ATTR_2(temp3_max, S_IRUGO | S_IWUSR,
				show_tmax, store_tmax, 2, 0);
static SENSOR_DEVICE_ATTR_2(temp3_max_hyst, S_IRUGO | S_IWUSR,
				show_tmax_hyst, store_tmax_hyst, 2, 0);

static SENSOR_DEVICE_ATTR_2(temp4_input, S_IRUGO,
				show_temp, NULL, 3, 0);
static SENSOR_DEVICE_ATTR_2(temp4_max, S_IRUGO | S_IWUSR,
				show_tmax, store_tmax, 3, 0);
static SENSOR_DEVICE_ATTR_2(temp4_max_hyst, S_IRUGO | S_IWUSR,
				show_tmax_hyst, store_tmax_hyst, 3, 0);

static struct attribute *thermal_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_max_hyst.dev_attr.attr,

	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp2_max_hyst.dev_attr.attr,

	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_temp3_max_hyst.dev_attr.attr,

	&sensor_dev_attr_temp4_input.dev_attr.attr,
	&sensor_dev_attr_temp4_max.dev_attr.attr,
	&sensor_dev_attr_temp4_max_hyst.dev_attr.attr,
	NULL
};

static struct attribute_group thermal_attr_gr = {
	.name = "mrfl_thermal",
	.attrs = thermal_attrs
};

static int mrfl_thermal_probe(struct platform_device *pdev)
{
	int ret;
	struct thermal_data *tdata;

	tdata = kzalloc(sizeof(struct thermal_data), GFP_KERNEL);
	if (!tdata) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	tdata->adc_res = kzalloc(sizeof(struct gpadc_result), GFP_KERNEL);
	if (!tdata->adc_res) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		kfree(tdata);
		return -ENOMEM;
	}

	tdata->pdev = pdev;
	platform_set_drvdata(pdev, tdata);

	/* Creating a sysfs group with thermal_attr_gr attributes */
	ret = sysfs_create_group(&pdev->dev.kobj, &thermal_attr_gr);
	if (ret) {
		dev_err(&pdev->dev, "sysfs create group failed\n");
		goto exit_free;
	}

	/* Registering with hwmon class */
	tdata->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(tdata->hwmon_dev)) {
		ret = PTR_ERR(tdata->hwmon_dev);
		tdata->hwmon_dev = NULL;
		dev_err(&pdev->dev, "hwmon_dev_regs failed\n");
		goto exit_sysfs;
	}

	/* Enable Thermal Monitoring */
	ret = enable_tm();
	if (ret) {
		dev_err(&pdev->dev, "Enabling TM failed.\n");
		goto exit_sysfs;
	}

	return 0;

exit_sysfs:
	sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_gr);
exit_free:
	kfree(tdata->adc_res);
	kfree(tdata);
	return ret;
}

static int mrfl_thermal_resume(struct device *dev)
{
	dev_info(dev, "resume called.\n");
	return 0;
}

static int mrfl_thermal_suspend(struct device *dev)
{
	dev_info(dev, "suspend called.\n");
	return 0;
}

static int mrfl_thermal_remove(struct platform_device *pdev)
{
	struct thermal_data *tdata = platform_get_drvdata(pdev);

	if (tdata) {
		hwmon_device_unregister(tdata->hwmon_dev);
		sysfs_remove_group(&pdev->dev.kobj, &thermal_attr_gr);
		kfree(tdata->adc_res);
		kfree(tdata);
	}
	return 0;
}

/*********************************************************************
 *		Driver initialization and finalization
 *********************************************************************/
static const struct platform_device_id thermal_id_table[] = {
	{ DEVICE_NAME, 1 },
	{ },
};

static const struct dev_pm_ops thermal_pm_ops = {
	.suspend = mrfl_thermal_suspend,
	.resume = mrfl_thermal_resume,
};

static struct platform_driver mrfl_thermal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &thermal_pm_ops,
		},
	.probe = mrfl_thermal_probe,
	.remove = __devexit_p(mrfl_thermal_remove),
	.id_table = thermal_id_table,
};

static int __init mrfl_thermal_module_init(void)
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

	return platform_driver_register(&mrfl_thermal_driver);
}

static void __exit mrfl_thermal_module_exit(void)
{
	platform_driver_unregister(&mrfl_thermal_driver);
}

module_init(mrfl_thermal_module_init);
module_exit(mrfl_thermal_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Merrifield Platform Thermal Driver");
MODULE_LICENSE("GPL");
