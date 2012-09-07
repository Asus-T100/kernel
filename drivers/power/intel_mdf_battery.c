/*
 * intel_mdf_battery.c - Intel Medfield MSIC Internal charger and Battery Driver
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
 * Author: Ananth Krishna <ananth.krishna.r@intel.com>,
 *         Anantha Narayanan <anantha.narayanan@intel.com>
 *         Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/sfi.h>
#include <linux/wakelock.h>
#include <linux/async.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>

#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <linux/usb/penwell_otg.h>
#include <linux/power/intel_mdf_battery.h>
#include <asm/intel_mid_gpadc.h>

#include "intel_mdf_charger.h"

#define DRIVER_NAME "intel_mdf_battery"
#define CHARGER_PS_NAME "msic_charger"

#define SFI_SIG_OEM0        "OEM0"
#define IRQ_KFIFO_ELEMENT	1


static void *otg_handle;
static struct device *msic_dev;
static struct power_supply *fg_psy;

static char *msic_power_supplied_to[] = {
			"msic_battery",
			"max170xx_battery",
			"max17042_battery",
			"max17047_battery",
			"max17050_battery",
};

static unsigned long long adc_ttl;
static int adc_sensor_vals[MSIC_BATT_SENSORS];

/*
 * This array represents the Battery Pack thermistor
 * temperature and corresponding ADC value limits
 */
static int const therm_curve_data[THERM_CURVE_MAX_SAMPLES]
	[THERM_CURVE_MAX_VALUES] = {
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 977, 961},
	{-10, -15, 961, 941},
	{-5, -10, 941, 917},
	{0, -5, 917, 887},
	{5, 0, 887, 853},
	{10, 5, 853, 813},
	{15, 10, 813, 769},
	{20, 15, 769, 720},
	{25, 20, 720, 669},
	{30, 25, 669, 615},
	{35, 30, 615, 561},
	{40, 35, 561, 508},
	{45, 40, 508, 456},
	{50, 45, 456, 407},
	{55, 50, 407, 357},
	{60, 55, 357, 315},
	{65, 60, 315, 277},
	{70, 65, 277, 243},
	{75, 70, 243, 212},
	{80, 75, 212, 186},
	{85, 80, 186, 162},
	{90, 85, 162, 140},
	{100, 90, 140, 107},
};


static struct batt_safety_thresholds *batt_thrshlds;


static struct msic_batt_sfi_prop *sfi_table;

static int error_count;


/*
 * All interrupt request are queued from interrupt
 * handler and processed in the bottom half
 */
static DEFINE_KFIFO(irq_fifo, u32, IRQ_FIFO_MAX);


/* Sysfs Entry for enable or disable Charging from user space */
static ssize_t set_chrg_enable(struct device *device,
			       struct device_attribute *attr, const char *buf,
			       size_t count);
static ssize_t get_chrg_enable(struct device *device,
			       struct device_attribute *attr, char *buf);
static DEVICE_ATTR(charge_enable, S_IRUGO | S_IWUSR, get_chrg_enable,
		   set_chrg_enable);

/* Sysfs Entry to show if lab power supply is used */
static ssize_t get_is_power_supply_conn(struct device *device,
			struct device_attribute *attr, char *buf);
static DEVICE_ATTR(power_supply_conn, S_IRUGO, get_is_power_supply_conn, NULL);

/*
 * msic usb properties
 */
static enum power_supply_property msic_usb_props[] = {
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static int battery_reboot_notifier_callback(struct notifier_block *notifier,
		unsigned long event, void *data);

static struct notifier_block battery_reboot_notifier = {
	.notifier_call = battery_reboot_notifier_callback,
};

/**
 * check_batt_psy -check for whether power supply type is battery
 * @dev : Power Supply dev structure
 * @data : Power Supply Driver Data
 * Context: can sleep
 *
 * Return true if power supply type is battery
 *
 */
static int check_batt_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	/* check for whether power supply type is battery */
	if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
		dev_info(msic_dev, "fg chip found:%s\n", psy->name);
		fg_psy = psy;
		return true;
	}

	return false;
}

/**
 * get_fg_chip_psy - identify the Fuel Gauge Power Supply device
 * Context: can sleep
 *
 * Return Fuel Gauge power supply structure
 */
static struct power_supply *get_fg_chip_psy(void)
{
	if (fg_psy)
		return fg_psy;

	/* loop through power supply class */
	class_for_each_device(power_supply_class, NULL, NULL,
				check_batt_psy);
	return fg_psy;
}

/**
 * fg_chip_get_property - read a power supply property from Fuel Gauge driver
 * @psp : Power Supply property
 *
 * Return power supply property value
 *
 */
static int fg_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!fg_psy)
		fg_psy = get_fg_chip_psy();
	if (fg_psy) {
		ret = fg_psy->get_property(fg_psy, psp, &val);
		if (!ret)
			return val.intval;
	}

	return ret;
}

/* Exported Functions to use with Fuel Gauge driver */
bool intel_msic_is_capacity_shutdown_en(void)
{
	if (batt_thrshlds &&
		(batt_thrshlds->fpo1 & FPO1_CAPACITY_SHUTDOWN))
		return true;
	else
		return false;
}
EXPORT_SYMBOL(intel_msic_is_capacity_shutdown_en);

bool intel_msic_is_volt_shutdown_en(void)
{
	if (batt_thrshlds &&
		(batt_thrshlds->fpo1 & FPO1_VOLTAGE_SHUTDOWN))
		return true;
	else
		return false;
}
EXPORT_SYMBOL(intel_msic_is_volt_shutdown_en);

bool intel_msic_is_lowbatt_shutdown_en(void)
{
	if (batt_thrshlds &&
		(batt_thrshlds->fpo1 & FPO1_LOWBATTINT_SHUTDOWN))
		return true;
	else
		return false;
}
EXPORT_SYMBOL(intel_msic_is_lowbatt_shutdown_en);

int intel_msic_get_vsys_min(void)
{
	/* Power Supply subsystem expects voltage in micro volts */
	if (batt_thrshlds && batt_thrshlds->vbatt_sh_min)
		return batt_thrshlds->vbatt_sh_min * 1000;
	else
		return MSIC_BATT_VMIN_THRESHOLD * 1000;
}
EXPORT_SYMBOL(intel_msic_get_vsys_min);

int intel_msic_is_current_sense_enabled(void)
{
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);

	return mbi->is_batt_valid;
}
EXPORT_SYMBOL(intel_msic_is_current_sense_enabled);

int intel_msic_check_battery_present(void)
{
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);
	int val;

	mutex_lock(&mbi->batt_lock);
	val = mbi->batt_props.present;
	mutex_unlock(&mbi->batt_lock);

	return val;
}
EXPORT_SYMBOL(intel_msic_check_battery_present);

int intel_msic_check_battery_health(void)
{
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);
	int val;

	mutex_lock(&mbi->batt_lock);
	val = mbi->batt_props.health;
	mutex_unlock(&mbi->batt_lock);

	return val;
}
EXPORT_SYMBOL(intel_msic_check_battery_health);

int intel_msic_check_battery_status(void)
{
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);
	int val;

	mutex_lock(&mbi->batt_lock);
	val = mbi->batt_props.status;
	mutex_unlock(&mbi->batt_lock);

	return val;
}
EXPORT_SYMBOL(intel_msic_check_battery_status);

static int is_protected_regs(u16 addr)
{
	/* in unsigned kernel mode write to some registers are blocked */
	if (addr == MSIC_BATT_CHR_CHRCVOLTAGE_ADDR ||
		addr == MSIC_BATT_CHR_CHRCCURRENT_ADDR ||
		addr == MSIC_BATT_CHR_PWRSRCLMT_ADDR ||
		addr == MSIC_BATT_CHR_CHRCTRL1_ADDR)
		return true;
	else
		return false;
}

/**
 * handle_ipc_rw_status - handle msic ipc read/write status
 * @error_val : ipc read/write status
 * @address   : msic register address
 * @rw	      : read/write access
 *
 * If the error count is more than MAX_IPC_ERROR_COUNT, report
 * charger and battery health as POWER_SUPPLY_HEALTH_UNSPEC_FAILURE
 *
 * returns error value in case of error else return 0
 */

static inline int handle_ipc_rw_status(int error_val,
		const u16 address, char rw)
{

	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);

	/*
	* Write to protected registers in unsigned kernel
	* mode will return -EIO
	*/
	/* Overwriting result value when failure is not a timeout, to support
	 * properly safety charging : we must ensure that when using
	 * an unsigned kernel, the failing access to protected registers
	 * (expected behaviour, returning -EIO)) will not block the accesses
	 * to the non protected registers.
	 * */
	if ((is_protected_regs(address)) && (rw == MSIC_IPC_WRITE) &&
			(error_val == -EIO))
		return 0;

	if (error_count < MAX_IPC_ERROR_COUNT) {
		error_count++;
		dev_warn(msic_dev, "MSIC IPC %s access to %x failed",
			(rw == MSIC_IPC_WRITE ? "write" : "read"), address);
	} else {

		dev_crit(msic_dev, "MSIC IPC %s access to %x failed",
			(rw == MSIC_IPC_WRITE ? "write" : "read"), address);

		mutex_lock(&mbi->batt_lock);
		/* set battery health */
		if (mbi->batt_props.health == POWER_SUPPLY_HEALTH_GOOD) {
			mbi->batt_props.health =
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		}
		mutex_unlock(&mbi->batt_lock);
		/* set charger health */
		mutex_lock(&mbi->usb_chrg_lock);
		if (mbi->usb_chrg_props.charger_health ==
				POWER_SUPPLY_HEALTH_GOOD){
			mbi->usb_chrg_props.charger_health =
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		}
		mutex_unlock(&mbi->usb_chrg_lock);
		power_supply_changed(&mbi->usb);
	}

	return error_val;
}

/**
 * get_batt_fg_curve_index - get the fg curve ID from umip
 * @name : Power Supply name
 *
 * Returns FG curve ID
 *
 */
static int get_batt_fg_curve_index(const char *name)
{
	struct power_supply *psy;
	int mip_offset, i, ret;
	u8 batt_id[BATTID_STR_LEN];
	u8 num_tbls = 0;
	int num_supplicants;

	/* check if we support the device in our supplied to list */
	num_supplicants = ARRAY_SIZE(msic_power_supplied_to);
	for (i = 0; i < num_supplicants; i++) {
		/* check for string length match and compare the strings */
		if ((strlen(name) == strlen(msic_power_supplied_to[i])) &&
		 (!strcmp(name, msic_power_supplied_to[i])))
			break;
	}

	if (i >= num_supplicants)
		return -ENXIO;

	/* check if msic charger is ready */
	psy = power_supply_get_by_name(CHARGER_PS_NAME);
	if (!psy)
		return -EAGAIN;

	/* get the no.of tables from mip */
	ret = intel_scu_ipc_read_mip((u8 *)&num_tbls, 1,
					UMIP_NO_OF_CFG_TBLS, 0);
	if (ret) {
		dev_warn(msic_dev, "%s: umip read failed\n", __func__);
		goto get_idx_failed;
	}

	/* compare the batt ID provided by SFI table and FG table in mip */
	mip_offset = UMIP_BATT_FG_CFG_TBL1 + BATT_FG_TBL_BATTID;
	for (i = 0; i < num_tbls; i++) {
		ret = intel_scu_ipc_read_mip(batt_id, BATTID_STR_LEN,
							mip_offset, 0);
		if (ret) {
			dev_warn(msic_dev, "%s: umip read failed\n", __func__);
			goto get_idx_failed;
		}

		if (!strncmp(batt_id, sfi_table->batt_id, BATTID_STR_LEN))
			break;

		mip_offset += UMIP_FG_TBL_SIZE;
		memset(batt_id, 0x0, BATTID_STR_LEN);
	}

	if (i < num_tbls)
		ret = i;
	else
		ret = -ENXIO;

get_idx_failed:
	return ret;
}

/**
 * intel_msic_restore_config_data - restore config data
 * @name : Power Supply name
 * @data : config data output pointer
 * @len : length of config data
 *
 */
int intel_msic_restore_config_data(const char *name, void *data, int len)
{
	int mip_offset, ret;

	/* check if msic charger is ready */
	if (!power_supply_get_by_name("msic_charger"))
		return -EAGAIN;

	/* Read the fuel gauge config data from umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_read_mip((u8 *)data, len, mip_offset, 0);
	if (ret)
		dev_warn(msic_dev, "%s: umip read failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(intel_msic_restore_config_data);

/**
 * intel_msic_save_config_data - save config data
 * @name : Power Supply name
 * @data : config data input pointer
 * @len : length of config data
 *
 */
int intel_msic_save_config_data(const char *name, void *data, int len)
{
	int mip_offset, ret;

	/* check if msic charger is ready */
	if (!power_supply_get_by_name("msic_charger"))
		return -EAGAIN;

	/* write the fuel gauge config data to umip */
	mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
	ret = intel_scu_ipc_write_umip((u8 *)data, len, mip_offset);
	if (ret)
		dev_warn(msic_dev, "%s: umip write failed\n", __func__);

	return ret;
}
EXPORT_SYMBOL(intel_msic_save_config_data);

/* Check for valid Temp ADC range */
static bool is_valid_temp_adc(int adc_val)
{
	if (adc_val >= MSIC_BTP_ADC_MIN && adc_val <= MSIC_BTP_ADC_MAX)
		return true;
	else
		return false;
}

/* Temperature conversion Macros */
static int conv_adc_temp(int adc_val, int adc_max, int adc_diff, int temp_diff)
{
	int ret;

	ret = (adc_max - adc_val) * temp_diff;
	return ret / adc_diff;
}

/* Check if the adc value is in the curve sample range */
static bool is_valid_temp_adc_range(int val, int min, int max)
{
	if (val > min && val <= max)
		return true;
	else
		return false;
}

/**
 * adc_to_temp - convert ADC code to temperature
 * @adc_val : ADC sensor reading
 *
 * Returns temperature in Degree Celsius
 */
static int adc_to_temp(uint16_t adc_val)
{
	int temp = 0;
	int i;

	if (!is_valid_temp_adc(adc_val)) {
		dev_warn(msic_dev, "Temperature out of Range: %u\n", adc_val);
		return -ERANGE;
	}

	for (i = 0; i < THERM_CURVE_MAX_SAMPLES; i++) {
		/* linear approximation for battery pack temperature */
		if (is_valid_temp_adc_range(adc_val, therm_curve_data[i][3],
					    therm_curve_data[i][2])) {

			temp = conv_adc_temp(adc_val, therm_curve_data[i][2],
					     therm_curve_data[i][2] -
					     therm_curve_data[i][3],
					     therm_curve_data[i][0] -
					     therm_curve_data[i][1]);

			temp += therm_curve_data[i][1];
			break;
		}
	}

	if (i >= THERM_CURVE_MAX_SAMPLES)
		dev_warn(msic_dev, "Invalid temp adc range\n");

	return temp;
}

static inline bool is_ttl_valid(u64 ttl)
{
	return time_before64(get_jiffies_64(), ttl);
}

/**
 * mdf_multi_read_adc_regs - read multiple ADC sensors
 * @mbi :  msic battery info pointer
 * @sample_count: do sample serveral times and get the average value.
 * @...: sensor numbers
 *
 * Returns 0 if success
 */
static int mdf_multi_read_adc_regs(struct msic_power_module_info *mbi,
				   int sample_count, int channel_count, ...)
{
	va_list args;
	int ret = 0, i, sensor, tmp;
	int *adc_val;
	int temp_adc_val[MSIC_BATT_SENSORS];

	mutex_lock(&mbi->adc_val_lock);
	if (!is_ttl_valid(adc_ttl) || (sample_count > 1)) {
		ret =
		    intel_mid_gpadc_sample(mbi->adc_handle, sample_count,
					   &adc_sensor_vals[MSIC_ADC_TEMP_IDX],
					   &adc_sensor_vals
						[MSIC_ADC_USB_VOL_IDX],
					   &adc_sensor_vals
						[MSIC_ADC_BATTID_IDX]);
		if (ret) {
			dev_err(&mbi->pdev->dev,
				"adc driver api returned error(%d)\n", ret);
			mutex_unlock(&mbi->adc_val_lock);
			goto adc_multi_exit;
		}

		adc_ttl = get_jiffies_64() + ADC_TIME_TO_LIVE;
	}
	memcpy(temp_adc_val, adc_sensor_vals, sizeof(temp_adc_val));
	mutex_unlock(&mbi->adc_val_lock);

	va_start(args, channel_count);
	for (i = 0; i < channel_count; i++) {
		/* get sensor number */
		sensor = va_arg(args, int);
		/* get value pointer */
		adc_val = va_arg(args, int *);
		if (adc_val == NULL) {
			ret = -EINVAL;
			goto adc_multi_exit;
		}
		*adc_val = temp_adc_val[sensor];
		switch (sensor) {
		case MSIC_ADC_TEMP_IDX:
			tmp = adc_to_temp(*adc_val);
			break;
		case MSIC_ADC_USB_VOL_IDX:
			tmp = MSIC_ADC_TO_VBUS_VOL(*adc_val);
			break;
		case MSIC_ADC_BATTID_IDX:
			tmp = *adc_val;
			break;
		default:
			dev_err(&mbi->pdev->dev, "invalid sensor%d", sensor);
			return -EINVAL;
		}
		*adc_val = tmp;
	}
	va_end(args);

adc_multi_exit:
	return ret;
}

static int mdf_read_adc_regs(int sensor, int *sensor_val,
		struct msic_power_module_info *mbi)
{
	int ret;
	ret = mdf_multi_read_adc_regs(mbi, 1, 1, sensor, sensor_val);

	if (ret)
		dev_err(&mbi->pdev->dev, "%s:mdf_multi_read_adc_regs failed",
			__func__);
	return ret;
}

int intel_msic_get_battery_pack_temp(int *temp)
{
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);

	/* check if msic charger is ready */
	if (!power_supply_get_by_name(CHARGER_PS_NAME))
		return -EAGAIN;

	if (!mbi->is_batt_valid)
		return -ENODEV;

	return mdf_read_adc_regs(MSIC_ADC_TEMP_IDX, temp, mbi);
}
EXPORT_SYMBOL(intel_msic_get_battery_pack_temp);

static void dump_registers(int dump_mask)
{
	int i, retval = 0;
	uint8_t reg_val;
	uint16_t chk_reg_addr;
	uint16_t reg_addr_boot[] = {MSIC_BATT_RESETIRQ1_ADDR,
		MSIC_BATT_RESETIRQ2_ADDR, MSIC_BATT_CHR_LOWBATTDET_ADDR,
		MSIC_BATT_CHR_SPCHARGER_ADDR, MSIC_BATT_CHR_CHRTTIME_ADDR,
		MSIC_BATT_CHR_CHRCTRL1_ADDR, MSIC_BATT_CHR_CHRSTWDT_ADDR,
		MSIC_BATT_CHR_CHRSAFELMT_ADDR};
	char *reg_str_boot[] = {"rirq1", "rirq2", "lowdet",
				"spchr", "chrtime", "chrctrl1",
				"chrgwdt", "safelmt"};
	uint16_t reg_addr_int[] = {MSIC_BATT_CHR_PWRSRCINT_ADDR,
		MSIC_BATT_CHR_PWRSRCINT1_ADDR, MSIC_BATT_CHR_CHRINT_ADDR,
		MSIC_BATT_CHR_CHRINT1_ADDR, MSIC_BATT_CHR_PWRSRCLMT_ADDR};
	char *reg_str_int[] = {"pwrint", "pwrint1", "chrint",
				"chrint1", "pwrsrclmt"};
	uint16_t reg_addr_evt[] = {MSIC_BATT_CHR_CHRCTRL_ADDR,
		MSIC_BATT_CHR_CHRCVOLTAGE_ADDR, MSIC_BATT_CHR_CHRCCURRENT_ADDR,
		MSIC_BATT_CHR_SPWRSRCINT_ADDR, MSIC_BATT_CHR_SPWRSRCINT1_ADDR,
		CHR_STATUS_FAULT_REG};
	char *reg_str_evt[] = {"chrctrl", "chrcv", "chrcc",
				"spwrsrcint", "sprwsrcint1", "chrflt"};

	if (dump_mask & MSIC_CHRG_REG_DUMP_BOOT) {
		for (i = 0; i < ARRAY_SIZE(reg_addr_boot); i++) {
			retval = intel_scu_ipc_ioread8(reg_addr_boot[i],
								&reg_val);
			if (retval) {
				chk_reg_addr = reg_addr_boot[i];
				goto ipcread_err;
			}
			dev_dbg(msic_dev, "%s val: %x\n", reg_str_boot[i],
								reg_val);
		}
	}
	if (dump_mask & MSIC_CHRG_REG_DUMP_INT) {
		for (i = 0; i < ARRAY_SIZE(reg_addr_int); i++) {
			retval = intel_scu_ipc_ioread8(reg_addr_int[i],
								&reg_val);
			if (retval) {
				chk_reg_addr = reg_addr_int[i];
				goto ipcread_err;
			}
			dev_dbg(msic_dev, "%s val: %x\n", reg_str_int[i],
								reg_val);
		}
	}
	if (dump_mask & MSIC_CHRG_REG_DUMP_EVENT) {
		for (i = 0; i < ARRAY_SIZE(reg_addr_evt); i++) {
			retval = intel_scu_ipc_ioread8(reg_addr_evt[i],
								&reg_val);
			if (retval) {
				chk_reg_addr = reg_addr_evt[i];
				goto ipcread_err;
			}
			dev_dbg(msic_dev, "%s val: %x\n", reg_str_evt[i],
								reg_val);
		}
	}

	return;

ipcread_err:
	handle_ipc_rw_status(retval, chk_reg_addr, MSIC_IPC_READ);
}

static bool is_charger_fault(void)
{
	uint8_t fault_reg, chrctrl_reg, stat, spwrsrcint_reg;
	int chr_mode, i, retval = 0;
	int adc_temp, adc_usb_volt, batt_volt;
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);

	mutex_lock(&mbi->event_lock);
	chr_mode = mbi->charging_mode;
	mutex_unlock(&mbi->event_lock);

	/* if charger is disconnected then report false */
	retval = intel_scu_ipc_ioread8(MSIC_BATT_CHR_SPWRSRCINT_ADDR,
		&spwrsrcint_reg);
	if (retval) {
		retval = handle_ipc_rw_status(retval,
			MSIC_BATT_CHR_SPWRSRCINT_ADDR, MSIC_IPC_READ);
		if (retval)
			return false;
	}

	if (!(spwrsrcint_reg & MSIC_BATT_CHR_USBDET_MASK)) {
		if (chr_mode != BATT_CHARGING_MODE_NONE) {
			dev_err(msic_dev, "USBDET bit not set\n");
			goto fault_detected;
		} else {
			return false;
		}
	}

	retval = intel_scu_ipc_ioread8(MSIC_BATT_CHR_CHRCTRL_ADDR,
			&chrctrl_reg);
	if (retval) {
		retval = handle_ipc_rw_status(retval,
				MSIC_BATT_CHR_CHRCTRL_ADDR, MSIC_IPC_READ);
		if (retval)
			return false;
	}
	/* if charger is disabled report false */
	if (chrctrl_reg & CHRCNTL_CHRG_DISABLE)
		return false;

	/* due to MSIC HW bug, the fault register is not getting updated
	 * immediately after the charging is enabled. As a SW WA the
	 * driver will retry reading the fault registers for 3 times
	 * with delay of 1 mSec.
	 */
	for (i = 0; i < CHR_READ_RETRY_CNT; i++) {
		retval = intel_scu_ipc_ioread8(CHR_STATUS_FAULT_REG,
								&fault_reg);
		if (retval) {
			retval = handle_ipc_rw_status(retval,
					CHR_STATUS_FAULT_REG, MSIC_IPC_READ);
			if (retval)
				return retval;
		}

		stat = (fault_reg & CHR_STATUS_BIT_MASK) >> CHR_STATUS_BIT_POS;
		if (stat == CHR_STATUS_BIT_READY) {
			dev_info(msic_dev, "retry reading Fault Reg:0x%x\n",
								fault_reg);
			mdelay(30);
			continue;
		}
		break;
	}

	/* if charger is enabled and STAT(0:1) shows charging
	* progress or charging done then we report false
	*/
	if (stat == CHR_STATUS_BIT_PROGRESS ||
		stat == CHR_STATUS_BIT_CYCLE_DONE)
		return false;

fault_detected:
	dev_info(msic_dev, "Charger fault occured\n");
	retval = mdf_read_adc_regs(MSIC_ADC_TEMP_IDX, &adc_temp, mbi);
	if (retval >= 0)
		dev_info(msic_dev, "[CHRG FLT] Temp:%d", adc_temp);

	retval = mdf_read_adc_regs(MSIC_ADC_USB_VOL_IDX, &adc_usb_volt, mbi);
	if (retval >= 0)
		dev_info(msic_dev, "[CHRG FLT] char_volt:%d", adc_usb_volt);

	batt_volt = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (batt_volt > 0)
		dev_info(msic_dev, "[CHRG FLT] batt_volt:%d", batt_volt/1000);

	return true;
}

/**
 * msic_usb_get_property - usb power source get property
 * @psy: usb power supply context
 * @psp: usb power source property
 * @val: usb power source property value
 * Context: can sleep
 *
 * MSIC usb power source property needs to be provided to power_supply
 * subsystem for it to provide the information to users.
 */
static int msic_usb_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct msic_power_module_info *mbi =
	    container_of(psy, struct msic_power_module_info, usb);
	int retval = 0;

	mutex_lock(&mbi->usb_chrg_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = mbi->usb_chrg_props.charger_present;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		if (mbi->batt_props.status != POWER_SUPPLY_STATUS_NOT_CHARGING)
			val->intval = mbi->usb_chrg_props.charger_present;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = mbi->usb_chrg_props.charger_health;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		retval = mdf_read_adc_regs(MSIC_ADC_USB_VOL_IDX,
				    &mbi->usb_chrg_props.vbus_vol, mbi);
		if (retval)
			break;
		val->intval = mbi->usb_chrg_props.vbus_vol * 1000;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = mbi->usb_chrg_props.charger_model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = mbi->usb_chrg_props.charger_vender;
		break;
	default:
		mutex_unlock(&mbi->usb_chrg_lock);
		return -EINVAL;
	}

	mutex_unlock(&mbi->usb_chrg_lock);
	return retval;
}

/**
 * msic_log_exception_event - log battery events
 * @event: msic event to be logged
 * Context: can sleep
 *
 * There are multiple battery and internal charger events
 * which may be of interest to users.
 * this battery function logs the different events onto the
 * kernel log messages.
 */
static void msic_log_exception_event(enum msic_event event)
{
	switch (event) {
	case MSIC_EVENT_BATTOCP_EXCPT:
		dev_warn(msic_dev,
			 "over battery charge current condition detected\n");
		break;
	case MSIC_EVENT_BATTOTP_EXCPT:
		dev_warn(msic_dev,
			 "battery out of acceptable temp range condition detected\n");
		break;
	case MSIC_EVENT_LOWBATT_EXCPT:
		dev_warn(msic_dev, "Low battery voltage condition detected\n");
		break;
	case MSIC_EVENT_BATTOVP_EXCPT:
		dev_warn(msic_dev, "battery over voltage condition detected\n");
		break;
	case MSIC_EVENT_CHROTP_EXCPT:
		dev_warn(msic_dev,
			 "charger high temperature condition detected\n");
		break;
	case MSIC_EVENT_USBOVP_EXCPT:
		dev_warn(msic_dev, "USB over voltage condition detected\n");
		break;
	case MSIC_EVENT_USB_VINREG_EXCPT:
		dev_warn(msic_dev, "USB Input voltage regulation "
			 "condition detected\n");
		break;
	case MSIC_EVENT_WEAKVIN_EXCPT:
		dev_warn(msic_dev, "USB Weak VBUS voltage "
			 "condition detected\n");
		break;
	case MSIC_EVENT_TIMEEXP_EXCPT:
		dev_warn(msic_dev, "Charger Total Time Expiration "
			 "condition detected\n");
		break;
	case MSIC_EVENT_WDTIMEEXP_EXCPT:
		dev_warn(msic_dev, "Watchdog Time Expiration "
			 "condition detected\n");
		break;
	default:
		dev_warn(msic_dev, "unknown error %u detected\n", event);
		break;
	}
}

/**
 * msic_handle_exception - handle any exception scenario
 * @mbi: device info structure to update the information
 * Context: can sleep
 *
 */

static void msic_handle_exception(struct msic_power_module_info *mbi,
				  uint8_t CHRINT_reg_value,
				  uint8_t CHRINT1_reg_value)
{
	enum msic_event exception;
	int temp, retval;
	unsigned int health = POWER_SUPPLY_HEALTH_GOOD;

	/* Battery Events */
	if (CHRINT_reg_value & MSIC_BATT_CHR_BATTOCP_MASK) {
		exception = MSIC_EVENT_BATTOCP_EXCPT;
		msic_log_exception_event(exception);
	}

	if (CHRINT_reg_value & MSIC_BATT_CHR_BATTOTP_MASK) {
		retval = mdf_read_adc_regs(MSIC_ADC_TEMP_IDX, &temp, mbi);
		if (retval) {
			dev_err(msic_dev, "%s(): Error in reading"
			       " temperature. Setting health as OVERHEAT\n",
			       __func__);
		}
		if (retval || (temp > batt_thrshlds->temp_high) ||
			(temp < batt_thrshlds->temp_low))
			health = POWER_SUPPLY_HEALTH_OVERHEAT;
		exception = MSIC_EVENT_BATTOTP_EXCPT;
		msic_log_exception_event(exception);
	}

	if (CHRINT_reg_value & MSIC_BATT_CHR_LOWBATT_MASK) {
		if (!mbi->usb_chrg_props.charger_present)
			health = POWER_SUPPLY_HEALTH_DEAD;
		exception = MSIC_EVENT_LOWBATT_EXCPT;
		msic_log_exception_event(exception);
	}
	if (CHRINT_reg_value & MSIC_BATT_CHR_TIMEEXP_MASK) {
		exception = MSIC_EVENT_TIMEEXP_EXCPT;
		msic_log_exception_event(exception);
	}

	if (CHRINT_reg_value & MSIC_BATT_CHR_WDTIMEEXP_MASK) {
		exception = MSIC_EVENT_WDTIMEEXP_EXCPT;
		msic_log_exception_event(exception);
	}

	if (CHRINT1_reg_value & MSIC_BATT_CHR_BATTOVP_MASK) {
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		exception = MSIC_EVENT_BATTOVP_EXCPT;
		msic_log_exception_event(exception);
	}

	if (health != POWER_SUPPLY_HEALTH_GOOD) {
		mutex_lock(&mbi->batt_lock);
		mbi->batt_props.health = health;
		mutex_unlock(&mbi->batt_lock);
		health = POWER_SUPPLY_HEALTH_GOOD;
	}

	/* Charger Events */
	if (CHRINT1_reg_value & MSIC_BATT_CHR_CHROTP_MASK) {
		exception = MSIC_EVENT_CHROTP_EXCPT;
		msic_log_exception_event(exception);
	}

	if (CHRINT1_reg_value & MSIC_BATT_CHR_USBOVP_MASK) {
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		exception = MSIC_EVENT_USBOVP_EXCPT;
		msic_log_exception_event(exception);
	}
	if (CHRINT1_reg_value & MSIC_BATT_CHR_WKVINDET_MASK) {
		health = POWER_SUPPLY_HEALTH_DEAD;
		exception = MSIC_EVENT_WEAKVIN_EXCPT;
		msic_log_exception_event(exception);
	}

	if (health != POWER_SUPPLY_HEALTH_GOOD) {
		mutex_lock(&mbi->usb_chrg_lock);
		mbi->usb_chrg_props.charger_health = health;
		mutex_unlock(&mbi->usb_chrg_lock);
		health = POWER_SUPPLY_HEALTH_GOOD;
	}

	mutex_lock(&mbi->event_lock);
	if (CHRINT1_reg_value & MSIC_BATT_CHR_VINREGMINT_MASK) {
		/* change input current limit to 500mA
		 * to recover from VINREG condition
		 */
		mbi->in_cur_lmt = CHRCNTL_VINLMT_500;
		mbi->refresh_charger = 1;
		exception = MSIC_EVENT_USB_VINREG_EXCPT;
		msic_log_exception_event(exception);
	}
	mutex_unlock(&mbi->event_lock);
}

/**
 *	msic_chr_write_multi	-	multi-write helper
 *	@mbi: msic power module
 *	@address: addresses of IPC writes.
		  should only be given in pairs, WDTWRITE address followed
		  by other charger register address
 *	@data: data for IPC writes
 *	@n: size of write table
 *
 *	Write a series of values to the SCU while respecting the ipc_rw_lock
 *	across the entire sequence. Handle any error reporting and pass back
 *	error codes on failure
 */
static int msic_chr_write_multi(struct msic_power_module_info *mbi,
			    const u16 *address, const u8 *data, int n)
{
	int retval = 0, i;
	int rep_count = 0;
	u8 read_data;

	/* All writes to charger-registers should accompany WDTWRITE address */
	if (n%2) {
		dev_warn(msic_dev, "Invalid no of register-arguments to %s\n",
				__func__);

		return -EINVAL;
	}

	mutex_lock(&mbi->ipc_rw_lock);
	/* Two writes are issued at once, first one to unlock WDTWRITE
	   and second one with given charger-register value */
	for (i = 0; i < n; i += 2) {
		/* Once written, the register is read back to check,
		   and re-written if required. This is repeated 3 times */
		for (rep_count = 0; rep_count < CHR_WRITE_RETRY_CNT;
				rep_count++) {
			retval = intel_scu_ipc_writev(address+i, data+i, 2);
			if (retval) {
				handle_ipc_rw_status(retval,
						*(address+i+1),
						MSIC_IPC_WRITE);
				break;
			} else {
				retval = intel_scu_ipc_ioread8(*(address+i+1),
						&read_data);
				if (retval) {
					handle_ipc_rw_status(retval,
							*(address+i+1),
							MSIC_IPC_READ);
					break;
				}

				if (read_data == *(data+i+1))
					break;
				else {
					dev_warn(msic_dev, "MSIC-Register RW "
							"mismatch on try %d\n",
							rep_count+1);
					if (rep_count == CHR_WRITE_RETRY_CNT-1)
						dev_err(msic_dev, "Error in MSIC-Register RW\n");
				}
			}
		}
	}
	mutex_unlock(&mbi->ipc_rw_lock);

	return retval;
}

/**
 *	ipc_read_modify_chr_param_reg - read and modify charger registers
 *	@mbi: msic power module
 *	@address:  charger register address
 *	@data: value to be set/reset
 *	@n: set or reset
 *
 */
static int ipc_read_modify_chr_param_reg(struct msic_power_module_info *mbi,
					 uint16_t addr, uint8_t val, int set)
{
	int ret = 0;
	static u16 address[2] = {
		MSIC_BATT_CHR_WDTWRITE_ADDR, 0
	};
	static u8 data[2] = {
		WDTWRITE_UNLOCK_VALUE, 0
	};

	address[1] = addr;

	/* Unlock Charge parameter registers before reading */
	ret = intel_scu_ipc_iowrite8(address[0], data[0]);
	if (ret) {
		ret = handle_ipc_rw_status(ret,
				address[0], MSIC_IPC_WRITE);
		if (ret)
			return ret;
	}

	ret = intel_scu_ipc_ioread8(address[1], &data[1]);
	if (ret) {
		ret = handle_ipc_rw_status(ret, address[1], MSIC_IPC_READ);
		if (ret)
			return ret;
	}

	if (set)
		data[1] |= val;
	else
		data[1] &= (~val);

	return msic_chr_write_multi(mbi, address, data, 2);
}

/**
 *	ipc_read_modify__reg - read and modify MSIC registers
 *	@mbi: msic power module
 *	@address:  charger register address
 *	@data: value to be set/reset
 *	@n: set or reset
 *
 */
static int ipc_read_modify_reg(struct msic_power_module_info *mbi,
			       uint16_t addr, uint8_t val, int set)
{
	int ret;
	u8 data;

	ret = intel_scu_ipc_ioread8(addr, &data);
	if (ret) {
		ret = handle_ipc_rw_status(ret, addr, MSIC_IPC_READ);
		if (ret)
			return ret;
	}

	if (set)
		data |= val;
	else
		data &= (~val);

	ret = intel_scu_ipc_iowrite8(addr, data);
	if (ret)
		ret = handle_ipc_rw_status(ret, addr, MSIC_IPC_WRITE);

	return ret;
}

/**
 * update_usb_ps_info - update usb power supply parameters
 * @mbi: msic power module structure
 * @cap: charger capability structure
 * @event: USB OTG events
 * Context: can sleep
 *
 * Updates the USB power supply parameters based on otg event.
 */
static void update_usb_ps_info(struct msic_power_module_info *mbi,
				struct otg_bc_cap *cap, int event)
{
	mutex_lock(&mbi->usb_chrg_lock);
	switch (event) {
	case USBCHRG_EVENT_DISCONN:
		mbi->usb.type = POWER_SUPPLY_TYPE_USB;
	case USBCHRG_EVENT_SUSPEND:
		dev_dbg(msic_dev, "Charger Disconnected or Suspended\n");
		mbi->usb_chrg_props.charger_health =
					POWER_SUPPLY_HEALTH_UNKNOWN;
		memcpy(mbi->usb_chrg_props.charger_model, "Unknown",
							sizeof("Unknown"));
		memcpy(mbi->usb_chrg_props.charger_vender, "Unknown",
							sizeof("Unknown"));
		if (event == USBCHRG_EVENT_SUSPEND)
			mbi->usb_chrg_props.charger_present =
						MSIC_USB_CHARGER_PRESENT;
		else
			mbi->usb_chrg_props.charger_present =
						MSIC_USB_CHARGER_NOT_PRESENT;
		break;
	case USBCHRG_EVENT_CONNECT:
	case USBCHRG_EVENT_UPDATE:
	case USBCHRG_EVENT_RESUME:
		dev_dbg(msic_dev, "Charger Connected or Updated\n");
		if (cap->chrg_type == CHRG_CDP) {
			mbi->usb.type = POWER_SUPPLY_TYPE_USB_CDP;
			dev_info(msic_dev, "Charger type: CDP, "
					"current-val: %d", cap->mA);
		} else if (cap->chrg_type == CHRG_DCP) {
			mbi->usb.type = POWER_SUPPLY_TYPE_USB_DCP;
			dev_info(msic_dev, "Charger type: DCP, "
					"current-val: %d", cap->mA);
		} else if (cap->chrg_type == CHRG_ACA) {
			mbi->usb.type = POWER_SUPPLY_TYPE_USB_ACA;
			dev_info(msic_dev, "Charger type: ACA, "
					"current-val: %d", cap->mA);
		} else if (cap->chrg_type == CHRG_SDP) {
			mbi->usb.type = POWER_SUPPLY_TYPE_USB;
			dev_info(msic_dev, "Charger type: SDP, "
					"current negotiated: %d", cap->mA);
		} else {
			/* CHRG_UNKNOWN */
			dev_warn(msic_dev, "Charger type:%d unknown\n",
					cap->chrg_type);
			mbi->usb.type = POWER_SUPPLY_TYPE_USB;
			goto update_usb_ps_exit;
		}
		mbi->usb_chrg_props.charger_present = MSIC_USB_CHARGER_PRESENT;
		mbi->usb_chrg_props.charger_health = POWER_SUPPLY_HEALTH_GOOD;
		memcpy(mbi->usb_chrg_props.charger_model, "msic",
							sizeof("msic"));
		memcpy(mbi->usb_chrg_props.charger_vender, "Intel",
							sizeof("Intel"));
		break;
	default:
		dev_warn(msic_dev, "Invalid OTG event\n");
	}

update_usb_ps_exit:
	mutex_unlock(&mbi->usb_chrg_lock);

	power_supply_changed(&mbi->usb);
}

static int msic_batt_stop_charging(struct msic_power_module_info *mbi)
{
	static const u16 address[] = {
		MSIC_BATT_CHR_WDTWRITE_ADDR,
		MSIC_BATT_CHR_CHRCTRL_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR,
		MSIC_BATT_CHR_CHRSTWDT_ADDR,
	};
	static const u8 data[] = {
		WDTWRITE_UNLOCK_VALUE,	/* Unlock chrg params */
		/*Disable Charging,Enable Low Power Mode */
		CHRCNTL_CHRG_DISABLE | CHRCNTL_CHRG_LOW_PWR_ENBL,
		WDTWRITE_UNLOCK_VALUE,	/* Unlock chrg params */
		CHR_WDT_DISABLE,	/*  Disable WDT Timer */
	};

	/*
	 * Charger connect handler delayed work also modifies the
	 * MSIC charger parameter registers.To avoid concurrent
	 * read writes to same set of registers  locking applied by
	 * msic_chr_write_multi
	 */
	return msic_chr_write_multi(mbi, address, data, 4);
}

/**
 * msic_batt_do_charging - set battery charger
 * @mbi: device info structure
 * @chrg: charge mode to set battery charger in
 * Context: can sleep
 *
 * MsIC battery charger needs to be enabled based on the charger
 * capabilities connected to the platform.
 */
static int msic_batt_do_charging(struct msic_power_module_info *mbi,
				 struct charge_params *params,
				 int is_maint_mode)
{
	int retval;
	u8 val;
	static u8 data[] = {
		WDTWRITE_UNLOCK_VALUE, 0,
		WDTWRITE_UNLOCK_VALUE, 0,
		WDTWRITE_UNLOCK_VALUE, 0,
		WDTWRITE_UNLOCK_VALUE, CHR_WDT_SET_60SEC,
		WDTWRITE_UNLOCK_VALUE, 0
	};
	static const u16 address[] = {
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRCCURRENT_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRCVOLTAGE_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRCTRL_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRSTWDT_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_SPCHARGER_ADDR
	};

	data[1] = params->ccur;
	data[3] = params->cvol;	/* charge voltage 4.14V */
	data[5] = params->vinilmt;
	data[9] = params->weakvin;

	/*
	 * Charger disconnect handler also modifies the
	 * MSIC charger parameter registers.To avoid concurrent
	 * read writes to same set of registers  locking applied
	 */
	retval = msic_chr_write_multi(mbi, address, data, 10);
	if (retval < 0) {
		dev_warn(msic_dev, "ipc multi write failed:%s\n", __func__);
		return retval;
	}

	/* prevent system from entering s3 while charger is connected */
	if (!wake_lock_active(&mbi->wakelock))
		wake_lock(&mbi->wakelock);

	retval = intel_scu_ipc_ioread8(MSIC_BATT_CHR_CHRCTRL_ADDR, &val);
	if (retval) {
		handle_ipc_rw_status(retval, MSIC_BATT_CHR_CHRCTRL_ADDR,
								MSIC_IPC_READ);
		return retval;
	}

	/* Check if charging enabled or not */
	if (val & CHRCNTL_CHRG_DISABLE) {
		dev_warn(msic_dev, "Charging not Enabled by MSIC!!!\n");
		return -EIO;
	}

	mutex_lock(&mbi->batt_lock);
	if (is_maint_mode)
		mbi->batt_props.status = POWER_SUPPLY_STATUS_FULL;
	else
		mbi->batt_props.status = POWER_SUPPLY_STATUS_CHARGING;
	mutex_unlock(&mbi->batt_lock);

	power_supply_changed(&mbi->usb);
	return 0;
}

static void reset_wdt_timer(struct msic_power_module_info *mbi)
{
	static const u16 address[2] = {
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRSTWDT_ADDR
	};
	static const u8 data[2] = {
		WDTWRITE_UNLOCK_VALUE, CHR_WDT_SET_60SEC
	};

	/*
	 * Charger disconnect handler also modifies the
	 * MSIC charger parameter registers.To avoid concurrent
	 * read writes to same set of registers  locking applied
	 */
	msic_chr_write_multi(mbi, address, data, 2);
}

/**
 * check_charge_full -  check battery is full or not
 * @mbi: device info structure
 * @vref: battery voltage
 * @temp_idx : temperature range index
 * Context: can sleep
 *
 * Return true if full
 *
 */
static int check_charge_full(struct msic_power_module_info *mbi,
			     int vref, int temp_idx)
{
	static int volt_prev;
	int is_full = false;
	int volt_now;
	int cur_avg;

	/* Read voltage and current from FG driver */
	volt_now = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (volt_now == -ENODEV || volt_now == -EINVAL) {
		dev_warn(msic_dev, "Can't read voltage from FG\n");
		return false;
	}
	/* convert to milli volts */
	volt_now /= 1000;

	/* Using Current-avg instead of Current-now to take care of
	 * instantaneous spike or dip */
	cur_avg = fg_chip_get_property(POWER_SUPPLY_PROP_CURRENT_AVG);
	if (cur_avg == -ENODEV || cur_avg == -EINVAL) {
		dev_warn(msic_dev, "Can't read current-avg from FG\n");
		return false;
	}
	/* convert to milli amps */
	cur_avg /= 1000;

	if ((volt_now > (vref - VBATT_FULL_DET_MARGIN)) &&
		(volt_prev > (vref - VBATT_FULL_DET_MARGIN))) {
		if (cur_avg >= FULL_CURRENT_AVG_LOW  &&
				cur_avg <= FULL_CURRENT_AVG_HIGH)
			is_full = true;
		else
			is_full = false;
	} else {
		is_full = false;
	}


	if (is_full) {
		dev_info(msic_dev, "Charge full detected\n");
		dev_dbg(msic_dev, "volt_now:%d, volt_prev:%d, "
				"volt_ref:%d, cur_avg:%d\n",
				volt_now, volt_prev, vref, cur_avg);
		/* Disable Charging */
		msic_batt_stop_charging(mbi);
	}

	volt_prev = volt_now;

	return is_full;
}
static void get_batt_temp_thresholds(short int *temp_high, short int *temp_low)
{
	int i, max_range;
	*temp_high = *temp_low = 0;

	if (sfi_table->temp_mon_ranges < SFI_TEMP_NR_RNG)
		max_range = sfi_table->temp_mon_ranges;
	else
		max_range = SFI_TEMP_NR_RNG;

	for (i = 0; i < max_range; i++) {
		if (*temp_high < sfi_table->temp_mon_range[i].temp_up_lim)
			*temp_high = sfi_table->temp_mon_range[i].temp_up_lim;
	}

	for (i = 0; i < max_range; i++) {
		if (*temp_low > sfi_table->temp_mon_range[i].temp_low_lim)
			*temp_low = sfi_table->temp_mon_range[i].temp_low_lim;
	}
}


/**
* sfi_temp_range_lookup - lookup SFI table to find the temperature range index
* @adc_temp : temperature in Degree Celcius
*
* Returns temperature range index
*/
static unsigned int sfi_temp_range_lookup(int adc_temp)
{
	int i;
	int max_range;

	if (sfi_table->temp_mon_ranges < SFI_TEMP_NR_RNG)
		max_range = sfi_table->temp_mon_ranges;
	else
		max_range = SFI_TEMP_NR_RNG;

	for (i = 0; i < max_range; i++) {
		if (adc_temp <= sfi_table->temp_mon_range[i].temp_up_lim &&
		    adc_temp > sfi_table->temp_mon_range[i].temp_low_lim) {
			dev_dbg(msic_dev, "Temp Range %d\n", i);
			break;
		}
	}

	return i;
}

/**
* msic_batt_temp_charging - manages the charging based on temperature
* @charge_param: charging parameter
* @sfi_table: SFI table structure
*
* To manage the charging based on the
* temperature of the battery
*/
static void msic_batt_temp_charging(struct work_struct *work)
{
	int ret, i, is_maint_chrg = false, is_lowchrg_enbl, is_chrg_flt;
	static int iprev = -1, is_chrg_enbl;
	short int cv = 0, cc = 0, vinlimit = 0, cvref;
	int adc_temp, adc_vol;
	int vbus_voltage;
	struct charge_params charge_param;
	struct msic_power_module_info *mbi =
	    container_of(work, struct msic_power_module_info,
			 connect_handler.work);
	struct temp_mon_table *temp_mon = NULL;

	memset(&charge_param, 0x0, sizeof(struct charge_params));
	charge_param.vinilmt = mbi->ch_params.vinilmt;
	charge_param.chrg_type = mbi->ch_params.chrg_type;

	mutex_lock(&mbi->event_lock);
	if (mbi->refresh_charger) {
		/*
		 * If the charger type is unknown or None
		 * better start the charging again and compute
		 * the properties again.
		 */
		mbi->refresh_charger = 0;
		iprev = -1;
		is_chrg_enbl = false;
	}
	mutex_unlock(&mbi->event_lock);

	if (mdf_read_adc_regs(MSIC_ADC_TEMP_IDX, &adc_temp, mbi)) {
		dev_err(msic_dev, "Error in reading temperature\n");
		goto lbl_sched_work;
	}

	/* find the temperature range */
	i = sfi_temp_range_lookup(adc_temp);

	/* get charger status */
	is_chrg_flt = is_charger_fault();

	if (mdf_read_adc_regs(MSIC_ADC_USB_VOL_IDX, &vbus_voltage, mbi)) {
		dev_warn(msic_dev, "Error in reading charger"
					" voltage:%s\n", __func__);
		goto lbl_sched_work;
	}

	/* change to fix buffer overflow issue */
	if (i >= ((sfi_table->temp_mon_ranges < SFI_TEMP_NR_RNG) ?
			sfi_table->temp_mon_ranges : SFI_TEMP_NR_RNG) ||
							is_chrg_flt ||
				vbus_voltage < WEAKVIN_VOLTAGE_LEVEL) {


		if ((adc_temp > batt_thrshlds->temp_high) ||
			(adc_temp < batt_thrshlds->temp_low)) {
			dev_warn(msic_dev,
					"TEMP RANGE DOES NOT EXIST FOR %d\n",
						adc_temp);
			mutex_lock(&mbi->batt_lock);
			mbi->batt_props.health = POWER_SUPPLY_HEALTH_OVERHEAT;
			mutex_unlock(&mbi->batt_lock);
		}
		/* Check charger Status bits */
		if (is_chrg_flt || mbi->batt_props.status
				== POWER_SUPPLY_STATUS_DISCHARGING) {
			mutex_lock(&mbi->batt_lock);
			mbi->batt_props.status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
			mutex_unlock(&mbi->batt_lock);
		}

		if (vbus_voltage < WEAKVIN_VOLTAGE_LEVEL) {
			mutex_lock(&mbi->usb_chrg_lock);
				mbi->usb_chrg_props.charger_health =
						POWER_SUPPLY_HEALTH_DEAD;
			mutex_unlock(&mbi->usb_chrg_lock);
		}

		dump_registers(MSIC_CHRG_REG_DUMP_EVENT);
		/*
		 * If we are in middle of charge cycle is safer to Reset WDT
		 * Timer Register.Because battery temperature and charge
		 * status register are not related.
		 */
		reset_wdt_timer(mbi);
		dev_dbg(msic_dev, "Charger Watchdog timer reset for 60sec\n");
		goto lbl_sched_work;
	}

	/* Set charger parameters */
	cv = sfi_table->temp_mon_range[i].full_chrg_vol;
	cc = sfi_table->temp_mon_range[i].full_chrg_cur;
	cvref = cv;
	dev_dbg(msic_dev, "cc:%d  cv:%d\n", cc, cv);

	mutex_lock(&mbi->event_lock);
	/* Check on user setting for charge current */
	if (mbi->usr_chrg_enbl == USER_SET_CHRG_LMT1)
		vinlimit = CHRCNTL_VINLMT_100;	/* VINILMT set to 100mA */
	else if (mbi->usr_chrg_enbl == USER_SET_CHRG_LMT2)
		vinlimit = CHRCNTL_VINLMT_500;	/* VINILMT set to 500mA */
	else if (mbi->usr_chrg_enbl == USER_SET_CHRG_LMT3)
		vinlimit = CHRCNTL_VINLMT_950;	/* VINILMT set to 950mA */
	else {
		/* D7,D6 bits of CHRCNTL will set the VINILMT */
		if (charge_param.vinilmt > 950)
			vinlimit = CHRCNTL_VINLMT_NOLMT;
		else if (charge_param.vinilmt > 500)
			vinlimit = CHRCNTL_VINLMT_950;
		else if (charge_param.vinilmt > 100)
			vinlimit = CHRCNTL_VINLMT_500;
		else
			vinlimit = CHRCNTL_VINLMT_100;
	}

	/* input current limit can be changed
	 * due to VINREG or weakVIN conditions
	 */
	if (mbi->in_cur_lmt < vinlimit)
		vinlimit = mbi->in_cur_lmt;
	/*
	 * Check for Charge full condition and set the battery
	 * properties accordingly. Also check for charging mode
	 * whether it is normal or maintenance mode.
	 */
	if (mbi->charging_mode == BATT_CHARGING_MODE_MAINTENANCE) {
		cvref = sfi_table->temp_mon_range[i].maint_chrg_vol_ul;
		is_maint_chrg = true;
	}
	mutex_unlock(&mbi->event_lock);

	/* Check full detection only if we are charging */
	if (is_chrg_enbl)
		ret = check_charge_full(mbi, cvref, i);
	else
		ret = is_chrg_enbl;

	if (ret) {
		is_chrg_enbl = false;
		if (!is_maint_chrg) {
			dev_dbg(msic_dev, "Going to Maintenance CHRG Mode\n");

			mutex_lock(&mbi->event_lock);
			mbi->charging_mode = BATT_CHARGING_MODE_MAINTENANCE;
			mutex_unlock(&mbi->event_lock);

			mutex_lock(&mbi->batt_lock);
			mbi->batt_props.status = POWER_SUPPLY_STATUS_FULL;
			mutex_unlock(&mbi->batt_lock);

			is_maint_chrg = true;
			power_supply_changed(&mbi->usb);
		}
	}

	/*
	 * If we are in same Temperature range check for the
	 * maintenance charging mode and enable the charging depending
	 * on the voltage.If Temperature range is changed then anyways
	 * we need to set charging parameters and enable charging.
	 */
	if (i == iprev) {
		/*
		 * Check if the voltage falls below lower threshold
		 * if we are in maintenance mode charging.
		 */
		if (is_maint_chrg && !is_chrg_enbl) {
			temp_mon = &sfi_table->temp_mon_range[i];
			/* Read battery Voltage */
			adc_vol = fg_chip_get_property(
					POWER_SUPPLY_PROP_VOLTAGE_NOW);
			if (adc_vol == -ENODEV || adc_vol == -EINVAL) {
				dev_warn(msic_dev, "Can't read voltage from FG\n");
				goto lbl_sched_work;
			}
			/* convert to milli volts */
			adc_vol /= 1000;

			if ((adc_vol <= temp_mon->maint_chrg_vol_ll)) {
				dev_dbg(msic_dev, "restart charging\n");
				cv = temp_mon->maint_chrg_vol_ul;
			} else {
				dev_dbg(msic_dev, "vbat is more than ll\n");
				goto lbl_sched_work;
			}
		} else {
			/* Reset WDT Timer Register for 60 Sec */
			reset_wdt_timer(mbi);
			dev_dbg(msic_dev, "Charger Watchdog timer reset for 60sec\n");
			goto lbl_sched_work;
		}
	} else {
		temp_mon = &sfi_table->temp_mon_range[i];
		dev_info(msic_dev, "Current Temp zone is %d, "
				"it's parameters are:\n", i);
		dev_info(msic_dev, "full_volt:%d, full_cur:%d\n",
				temp_mon->full_chrg_vol,
				temp_mon->full_chrg_cur);
		dev_info(msic_dev, "maint_vol_ll:%d, maint_vol_ul:%d, "
				"maint_cur:%d\n", temp_mon->maint_chrg_vol_ll,
				temp_mon->maint_chrg_vol_ul,
				temp_mon->maint_chrg_cur);
	}

	iprev = i;
	mbi->ch_params.cvol = cv;
	charge_param.cvol = CONV_VOL_DEC_MSICREG(cv);
	/* CHRCC_MIN_CURRENT is th lowet value */
	cc = cc - CHRCC_MIN_CURRENT;
	/*
	 * If charge current parameter is less than 550mA we should
	 * enable LOW CHARGE mode which will limit the charge current to 325mA.
	 */
	if (cc <= 0) {
		dev_dbg(msic_dev, "LOW CHRG mode enabled\n");
		cc = 0;
		is_lowchrg_enbl = BIT_SET;
	} else {
		dev_dbg(msic_dev, "LOW CHRG mode NOT enabled\n");
		cc = cc / 100;
		is_lowchrg_enbl = BIT_RESET;
	}
	cc = cc << 3;

	charge_param.ccur = cc;
	charge_param.vinilmt = vinlimit;

	dev_dbg(msic_dev, "params  vol: %x  cur:%x vinilmt:%x\n",
		charge_param.cvol, charge_param.ccur, charge_param.vinilmt);

	if (cv > WEAKVIN_VOLTAGE_LEVEL)
		charge_param.weakvin = CHR_SPCHRGER_WEAKVIN_LVL1;
	else
		charge_param.weakvin = CHR_SPCHRGER_WEAKVIN_LVL2;

	if (is_lowchrg_enbl)
		charge_param.weakvin |= CHR_SPCHRGER_LOWCHR_ENABLE;

	/* enable charging here */
	dev_info(msic_dev, "Enable Charging\n");
	ret = msic_batt_do_charging(mbi, &charge_param, is_maint_chrg);
	dump_registers(MSIC_CHRG_REG_DUMP_EVENT);
	if (ret) {
		dev_warn(msic_dev, "msic_batt_do_charging failed\n");
		goto lbl_sched_work;
	}
	is_chrg_enbl = true;

lbl_sched_work:
	/* Schedule the work after 30 Seconds */
	schedule_delayed_work(&mbi->connect_handler, TEMP_CHARGE_DELAY_JIFFIES);
}
static void update_charger_health(struct msic_power_module_info *mbi)
{
	int vbus_volt;
	unsigned char dummy_val;

	/* We don't get an interrupt once charger returns from
	  error state. So check current status by reading voltage
	  and report health as good if recovered from error state */


	/* Read charger data from ADC channels */
	if (mdf_read_adc_regs(MSIC_ADC_USB_VOL_IDX, &vbus_volt, mbi)) {
		dev_warn(msic_dev, "Error in reading charger"
				" voltage:%s\n", __func__);
		return;
	}
	dev_info(msic_dev, "vbus_volt:%d\n", vbus_volt);

	/* Compute Charger health */
	mutex_lock(&mbi->usb_chrg_lock);
	/* check system recovered from overvoltage and dead conditions */
	if ((mbi->usb_chrg_props.charger_health ==
			POWER_SUPPLY_HEALTH_OVERVOLTAGE ||
		mbi->usb_chrg_props.charger_health
			== POWER_SUPPLY_HEALTH_DEAD) &&
	    vbus_volt >= MSIC_VBUS_LOW_VOLTAGE &&
	    vbus_volt <= MSIC_VBUS_OVER_VOLTAGE) {

		mbi->usb_chrg_props.charger_health = POWER_SUPPLY_HEALTH_GOOD;

	} else if (mbi->batt_props.health ==
			POWER_SUPPLY_HEALTH_UNSPEC_FAILURE){

		/* do a dummy IPC read to see IPC recovered form IPC error.
		 * If recovered reset error_count*/

		if (!intel_scu_ipc_ioread8(MSIC_BATT_CHR_CHRCTRL_ADDR,
					&dummy_val)) {
			error_count = 0;
			mbi->usb_chrg_props.charger_health =
					POWER_SUPPLY_HEALTH_GOOD;

		}
	}
	mutex_unlock(&mbi->usb_chrg_lock);
}

static void update_battery_health(struct msic_power_module_info *mbi)
{
	int temp, volt, chr_mode, max_volt;
	unsigned char dummy_val;

	mutex_lock(&mbi->event_lock);
	chr_mode = mbi->charging_mode;
	mutex_unlock(&mbi->event_lock);

	/* We don't get an interrupt once battery returns from
	  error state. So check current status by reading voltagei and
	  temperature and report health as good if recovered from error state */

	volt = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	if (volt == -ENODEV || volt == -EINVAL) {
		dev_warn(msic_dev, "Can't read voltage from FG\n");
		return;
	}

	temp = fg_chip_get_property(POWER_SUPPLY_PROP_TEMP);
	if (temp == -ENODEV || temp == -EINVAL) {
		dev_warn(msic_dev, "Can't read temp from FG\n");
		return;
	}

	dev_info(msic_dev, "vbatt:%d temp:%d\n", volt, temp);

	/* convert to milli volts */
	volt /= 1000;

	/*convert to degree Celcius from tenths of degree Celsius */
	temp = temp / 10;



	if (chr_mode != BATT_CHARGING_MODE_NONE)
		max_volt = (mbi->ch_params.cvol * OVP_VAL_MULT_FACTOR) / 10;
	else
		max_volt = BATT_OVERVOLTAGE_CUTOFF_VOLT;

	/* Check for fault and update health */
	mutex_lock(&mbi->batt_lock);
	/*
	 * Valid temperature window is 0 to 60 Degrees
	 * and thermistor has 2 degree hysteresis and considering
	 * 2 degree adc error, fault revert temperature will
	 * be 4 to 56 degrees.
	 */
	if (mbi->batt_props.health == POWER_SUPPLY_HEALTH_GOOD) {
		/* if BATTOTP is masked check temperature and
		 * update BATT HEALTH */
		if (mbi->chrint_mask & MSIC_BATT_CHR_BATTOTP_MASK) {
			if ((temp > (batt_thrshlds->temp_high)) ||
					(temp < (batt_thrshlds->temp_low))) {
				/* We report OVERHEAT in both cases*/
				mbi->batt_props.health =
					POWER_SUPPLY_HEALTH_OVERHEAT;
				dev_dbg(msic_dev, "Battery pack temp: %d, "
						"too hot or too cold\n", temp);
			}
		}
	} else {
		/* check for battery overvoltage,overheat and dead  health*/
		if ((mbi->batt_props.health !=
				      POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) &&
			(mbi->batt_props.health != POWER_SUPPLY_HEALTH_DEAD) &&
			(temp <= (batt_thrshlds->temp_high -
				 MSIC_TEMP_HYST_ERR)) &&
			(temp >= (batt_thrshlds->temp_low
				  + MSIC_TEMP_HYST_ERR)) &&
			(volt >= batt_thrshlds->vbatt_crit) &&
			(volt <= max_volt)) {

			mbi->batt_props.health = POWER_SUPPLY_HEALTH_GOOD;
			dev_dbg(msic_dev, "Setting battery-health, power-supply good");
		} else if (mbi->batt_props.health ==
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE){

			/* do a dummy IPC read to see IPC recovered from
			 * IPC error. If recovered reset error_count*/

			if (!intel_scu_ipc_ioread8(MSIC_BATT_CHR_CHRCTRL_ADDR,
						&dummy_val)) {
				error_count = 0;
				mbi->batt_props.health =
					POWER_SUPPLY_HEALTH_GOOD;

			}
		}
	}

	mutex_unlock(&mbi->batt_lock);

}

static void msic_batt_disconn(struct work_struct *work)
{
	int ret, event;
	struct msic_power_module_info *mbi =
	    container_of(work, struct msic_power_module_info,
			 disconn_handler.work);

	mutex_lock(&mbi->event_lock);
	event = mbi->batt_event;
	mutex_unlock(&mbi->event_lock);

	if (event != USBCHRG_EVENT_SUSPEND &&
		event != USBCHRG_EVENT_DISCONN) {
		dev_warn(msic_dev, "%s:Not a Disconn or Suspend event\n",
							__func__);
		return ;
	}

	dev_dbg(msic_dev, "Stopping charging due to charger event: %s\n",
			(event == USBCHRG_EVENT_SUSPEND) ? "SUSPEND" :
			"DISCONNECT");
	dump_registers(MSIC_CHRG_REG_DUMP_EVENT);
	ret = msic_batt_stop_charging(mbi);
	if (ret) {
		dev_warn(msic_dev, "%s: failed\n", __func__);
		return;
	}

	mutex_lock(&mbi->batt_lock);
	if (event == USBCHRG_EVENT_SUSPEND)
		mbi->batt_props.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		mbi->batt_props.status = POWER_SUPPLY_STATUS_DISCHARGING;
	mutex_unlock(&mbi->batt_lock);

	/* release the wake lock when charger is unplugged */
	if (wake_lock_active(&mbi->wakelock))
		wake_unlock(&mbi->wakelock);

	power_supply_changed(&mbi->usb);
}

/**
* msic_event_handler - msic event handler
* @arg : private  data pointer
* @event: MSIC event
* @cap : otg capabilities
*
*/
static int msic_event_handler(void *arg, int event, struct otg_bc_cap *cap)
{
	struct msic_power_module_info *mbi =
	    (struct msic_power_module_info *)arg;

	/* Update USB power supply info */
	update_usb_ps_info(mbi, cap, event);

	/* check for valid battery condition */
	if (!mbi->is_batt_valid)
		return 0;

	mutex_lock(&mbi->event_lock);
	if ((mbi->batt_event == event && event != USBCHRG_EVENT_UPDATE) ||
	    (!mbi->usr_chrg_enbl)) {
		mutex_unlock(&mbi->event_lock);
		return 0;
	}
	mbi->batt_event = event;
	mutex_unlock(&mbi->event_lock);

	switch (event) {
	case USBCHRG_EVENT_CONNECT:
		pm_runtime_get_sync(&mbi->pdev->dev);
	case USBCHRG_EVENT_RESUME:
	case USBCHRG_EVENT_UPDATE:
		/*
		 * If previous event CONNECT and current is event is
		 * UPDATE, we have already queued the work.
		 * Its better to dequeue the previous work
		 * and add the new work to the queue.
		 */
		cancel_delayed_work_sync(&mbi->connect_handler);
		mbi->ch_params.vinilmt = cap->mA;
		mbi->in_cur_lmt = CHRCNTL_VINLMT_NOLMT;
		mbi->ch_params.chrg_type = cap->chrg_type;
		dev_dbg(msic_dev, "CHRG TYPE:%d %d\n", cap->chrg_type, cap->mA);
		mutex_lock(&mbi->event_lock);
		mbi->refresh_charger = 1;
		if (mbi->charging_mode == BATT_CHARGING_MODE_NONE)
			mbi->charging_mode = BATT_CHARGING_MODE_NORMAL;
		mutex_unlock(&mbi->event_lock);

		/* Enable charger LED */
		ipc_read_modify_chr_param_reg(mbi, MSIC_CHRG_LED_CNTL_REG,
					      MSIC_CHRG_LED_ENBL, 1);
		/*Disable charger  LOW Power Mode */
		ipc_read_modify_chr_param_reg(mbi, MSIC_BATT_CHR_CHRCTRL_ADDR,
					      CHRCNTL_CHRG_LOW_PWR_ENBL, 0);

		schedule_delayed_work(&mbi->connect_handler, 0);
		break;
	case USBCHRG_EVENT_DISCONN:
		pm_runtime_put_sync(&mbi->pdev->dev);
	case USBCHRG_EVENT_SUSPEND:
		dev_info(msic_dev, "USB DISCONN or SUSPEND\n");
		cancel_delayed_work_sync(&mbi->connect_handler);
		schedule_delayed_work(&mbi->disconn_handler, 0);

		mutex_lock(&mbi->event_lock);
		mbi->refresh_charger = 0;
		mbi->charging_mode = BATT_CHARGING_MODE_NONE;
		mutex_unlock(&mbi->event_lock);

		/* Disable charger LED */
		ipc_read_modify_chr_param_reg(mbi, MSIC_CHRG_LED_CNTL_REG,
					      MSIC_CHRG_LED_ENBL, 0);
		/*Enable charger LOW Power Mode */
		ipc_read_modify_chr_param_reg(mbi, MSIC_BATT_CHR_CHRCTRL_ADDR,
					      CHRCNTL_CHRG_LOW_PWR_ENBL, 1);
		break;
	default:
		dev_warn(msic_dev, "Invalid OTG Event:%s\n", __func__);
	}
	return 0;
}

static void msic_chrg_callback_worker(struct work_struct *work)
{
	struct otg_bc_cap cap;
	struct msic_power_module_info *mbi =
	    container_of(work, struct msic_power_module_info,
			 chrg_callback_dwrk.work);
	penwell_otg_query_charging_cap(&cap);
	msic_event_handler(mbi, cap.current_event, &cap);
}

/*
 * msic_charger_callback - callback for USB OTG
 * @arg: device info structure
 * @event: USB event
 * @cap: charging capabilities
 * Context: Interrupt Context can not sleep
 *
 * Will be called from the OTG driver.Depending on the event
 * schedules a bottom half to enable or disable the charging.
 */
static int msic_charger_callback(void *arg, int event, struct otg_bc_cap *cap)
{
	struct msic_power_module_info *mbi =
	    (struct msic_power_module_info *)arg;

	schedule_delayed_work(&mbi->chrg_callback_dwrk, 0);
	return 0;
}

/**
 * msic_status_monitor - worker function to monitor status
 * @work: delayed work handler structure
 * Context: Can sleep
 *
 * Will be called from the threaded IRQ function.
 * Monitors status of the charge register and temperature.
 */
static void msic_status_monitor(struct work_struct *work)
{
	int chr_mode, chr_event, is_chrg_flt;
	unsigned int delay = CHARGE_STATUS_DELAY_JIFFIES;
	struct msic_power_module_info *mbi =
	    container_of(work, struct msic_power_module_info,
			 chr_status_monitor.work);

	pm_runtime_get_sync(&mbi->pdev->dev);

	mutex_lock(&mbi->event_lock);
	chr_mode = mbi->charging_mode;
	chr_event = mbi->batt_event;
	mutex_unlock(&mbi->event_lock);

	/*update charger and battery health */
	update_charger_health(mbi);
	update_battery_health(mbi);

	/* Check charger Status bits */
	is_chrg_flt = is_charger_fault();

	mutex_lock(&mbi->batt_lock);
	if (chr_mode == BATT_CHARGING_MODE_MAINTENANCE)
		mbi->batt_props.status = POWER_SUPPLY_STATUS_FULL;
	else if (chr_mode == BATT_CHARGING_MODE_NORMAL)
		mbi->batt_props.status = POWER_SUPPLY_STATUS_CHARGING;
	else if (chr_event == USBCHRG_EVENT_SUSPEND)
		mbi->batt_props.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		mbi->batt_props.status = POWER_SUPPLY_STATUS_DISCHARGING;

	if (is_chrg_flt) {
		dev_warn(msic_dev, "charger fault detected\n");
		mbi->batt_props.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	mutex_unlock(&mbi->batt_lock);

	dump_registers(MSIC_CHRG_REG_DUMP_EVENT);
	power_supply_changed(&mbi->usb);
	schedule_delayed_work(&mbi->chr_status_monitor, delay);

	pm_runtime_put_sync(&mbi->pdev->dev);
}

/**
 * msic_battery_interrupt_handler - msic battery interrupt handler
 * Context: interrupt context
 *
 * MSIC battery interrupt handler which will be called on insertion
 * of valid power source to charge the battery or an exception
 * condition occurs.
 */
static irqreturn_t msic_battery_interrupt_handler(int id, void *dev)
{
	struct msic_power_module_info *mbi = dev;
	u32 reg_int_val;

	/* We have only one concurrent fifo reader
	 * and only one concurrent writer, so we are not
	 * using any lock to protect fifo.
	 */
	if (unlikely(kfifo_is_full(&irq_fifo))) {
		dev_warn(&mbi->pdev->dev, "KFIFO Full\n");
		return IRQ_WAKE_THREAD;
	}
	/* Copy Interrupt registers locally */
	reg_int_val = readl(mbi->msic_intr_iomap);
	/* Add the Interrupt regs to  FIFO */
	kfifo_in(&irq_fifo, &reg_int_val, IRQ_KFIFO_ELEMENT);

	return IRQ_WAKE_THREAD;
}

/**
 * msic_battery_thread_handler - msic battery threaded IRQ function
 * Context: can sleep
 *
 * MSIC battery needs to either update the battery status as full
 * if it detects battery full condition caused the interrupt or needs
 * to enable battery charger if it detects usb and battery detect
 * caused the source of interrupt.
 */
static irqreturn_t msic_battery_thread_handler(int id, void *dev)
{
	int ret;
	unsigned char data[2];
	struct msic_power_module_info *mbi = dev;
	u32 tmp;
	unsigned char intr_stat;
	u32 log_intr;

	/* We have only one concurrent fifo reader
	 * and only one concurrent writer, we are not
	 * using any lock to protect fifo.
	 */
	if (unlikely(kfifo_is_empty(&irq_fifo))) {
		dev_warn(msic_dev, "KFIFO Empty\n");
		return IRQ_NONE;
	}
	/* Get the Interrupt regs state from FIFO */
	ret = kfifo_out(&irq_fifo, &tmp, IRQ_KFIFO_ELEMENT);
	if (ret != IRQ_KFIFO_ELEMENT) {
		dev_warn(msic_dev, "KFIFO underflow\n");
		return IRQ_NONE;
	}

	/* Even though some second level charger interrupts are masked by SCU
	 * the flags for these interrupts will be set in second level interrupt
	 * status register when SCU forwards the unmasked interrupts. Kernel
	 * should ignore the status for masked registers.*/

	/* CHRINT Register */
	data[0] = ((tmp & 0x00ff0000) >> 16) & ~(mbi->chrint_mask);
	/* CHRINT1 Register */
	data[1] = (tmp & 0xff000000) >> 24 & ~(mbi->chrint1_mask);

	dev_dbg(msic_dev, "PWRSRC Int %x %x\n", tmp & 0xff,
		(tmp & 0xff00) >> 8);
	dev_dbg(msic_dev, "CHR Int %x %x\n", data[0], data[1]);

	/* Saving the read interrupt value for printing later */
	log_intr = tmp;

	mutex_lock(&mbi->event_lock);
	tmp = mbi->charging_mode;
	mutex_unlock(&mbi->event_lock);

	dump_registers(MSIC_CHRG_REG_DUMP_INT | MSIC_CHRG_REG_DUMP_EVENT);

	/* Check if charge complete */
	if (data[1] & MSIC_BATT_CHR_CHRCMPLT_MASK)
		dev_dbg(msic_dev, "CHRG COMPLT\n");

	if ((data[0] & MSIC_BATT_CHR_TIMEEXP_MASK) &&
			(tmp == BATT_CHARGING_MODE_NORMAL)) {
		dev_dbg(msic_dev, "force suspend event\n");
		msic_event_handler(mbi, USBCHRG_EVENT_SUSPEND, NULL);
	}

	if (data[1] & MSIC_BATT_CHR_WKVINDET_MASK) {
		dev_dbg(msic_dev, "CHRG WeakVIN Detected\n");

		/* Sometimes for USB unplug we are receiving WeakVIN
		 * interrupts,So as SW work around we will check the
		 * SPWRSRCINT SUSBDET bit to know the USB connection.
		 */
		ret = intel_scu_ipc_ioread8(MSIC_BATT_CHR_SPWRSRCINT_ADDR,
					    &intr_stat);
		if (ret)
			handle_ipc_rw_status(ret, MSIC_BATT_CHR_SPWRSRCINT_ADDR,
					MSIC_IPC_READ);
		else if (!(intr_stat & MSIC_BATT_CHR_USBDET_MASK) &&
		    (tmp != BATT_CHARGING_MODE_NONE)) {
			data[1] &= ~MSIC_BATT_CHR_WKVINDET_MASK;
			dev_dbg(msic_dev, "force disconnect event\n");
			msic_event_handler(mbi, USBCHRG_EVENT_DISCONN, NULL);
		}
	}

	/* Check if an exception occurred */
	if (data[0] || (data[1] & ~(MSIC_BATT_CHR_CHRCMPLT_MASK)))
		msic_handle_exception(mbi, data[0], data[1]);

	/*
	 * Mask LOWBATT INT once after recieving the first
	 * LOWBATT INT because the platfrom will shutdown
	 * on LOWBATT INT, So no need to service LOWBATT INT
	 * afterwards and increase the load on CPU.
	 */
	if ((data[0] & MSIC_BATT_CHR_LOWBATT_MASK) &&
			!mbi->usb_chrg_props.charger_present) {
		dev_warn(msic_dev, "Masking LOWBATTINT\n");
		mbi->chrint_mask |= MSIC_BATT_CHR_LOWBATT_MASK;
		ret = intel_scu_ipc_iowrite8(MSIC_BATT_CHR_MCHRINT_ADDR,
						mbi->chrint_mask);
		if (ret)
			handle_ipc_rw_status(ret,
				MSIC_BATT_CHR_MCHRINT_ADDR, MSIC_IPC_WRITE);
	}

	/* Check charger Status bits */
	if ((data[0] & ~(MSIC_BATT_CHR_TIMEEXP_MASK)) ||
		(data[1] & ~(MSIC_BATT_CHR_CHRCMPLT_MASK))
			&& is_charger_fault()) {
		mutex_lock(&mbi->batt_lock);
		mbi->batt_props.status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		mutex_unlock(&mbi->batt_lock);

		dev_info(msic_dev, "PWRSRC Int %x %x\n", log_intr & 0xff,
				(log_intr & 0xff00) >> 8);
		dev_info(msic_dev, "CHR Int %x %x\n", data[0], data[1]);
	}

	power_supply_changed(&mbi->usb);
	return IRQ_HANDLED;
}

/**
 * check_charger_conn - check charger connection and handle the state
 * @mbi : charger device info
 * Context: can sleep
 *
 */
static int check_charger_conn(struct msic_power_module_info *mbi)
{
	int retval;
	struct otg_bc_cap cap;
	unsigned char data;

	retval = intel_scu_ipc_ioread8(MSIC_BATT_CHR_SPWRSRCINT_ADDR,
			&data);
	if (retval) {
		retval = handle_ipc_rw_status(retval,
				MSIC_BATT_CHR_SPWRSRCINT_ADDR, MSIC_IPC_READ);
		if (retval)
			goto disable_chrg_block;
	}

	if (data & MSIC_BATT_CHR_USBDET_MASK) {
		retval = penwell_otg_query_charging_cap(&cap);
		if (retval) {
			dev_warn(msic_dev, "%s(): usb otg power query "
				 "failed with error code %d\n", __func__,
				 retval);
			goto disable_chrg_block;
		}
		/* Enable charging only if vinilmt is >= 100mA */
		if (cap.mA >= 100) {
			msic_event_handler(mbi, USBCHRG_EVENT_CONNECT, &cap);
			return retval;
		}
	}

disable_chrg_block:
	/* Disable the charging block */
	msic_batt_stop_charging(mbi);
	/*Putting the charger in LOW Power Mode */
	ipc_read_modify_chr_param_reg(mbi, MSIC_BATT_CHR_CHRCTRL_ADDR,
				      CHRCNTL_CHRG_LOW_PWR_ENBL, 1);
	return retval;
}

/**
 * set_chrg_enable - sysfs set api for charge_enable attribute
 * Parameter as define by sysfs interface
 * Context: can sleep
 *
 */
static ssize_t set_chrg_enable(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	struct platform_device *pdev =
	    container_of(dev, struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);
	unsigned long value;
	int retval, chr_mode;

	if (kstrtoul(buf, 10, &value))
		return -EINVAL;

	/* Allow only 0 to 4 for writing */
	if (value < USER_SET_CHRG_DISABLE || value > USER_SET_CHRG_NOLMT)
		return -EINVAL;

	mutex_lock(&mbi->event_lock);
	chr_mode = mbi->charging_mode;
	mutex_unlock(&mbi->event_lock);

	if (!value && (chr_mode != BATT_CHARGING_MODE_NONE)) {
		dev_dbg(msic_dev, "User App charger disable !\n");
		/* Disable charger before setting the usr_chrg_enbl */
		msic_event_handler(mbi, USBCHRG_EVENT_SUSPEND, NULL);

		mutex_lock(&mbi->event_lock);
		mbi->usr_chrg_enbl = value;
		mutex_unlock(&mbi->event_lock);

	} else if (value && (chr_mode == BATT_CHARGING_MODE_NONE)) {
		dev_dbg(msic_dev, "User App charger enable!\n");

		/* enable usr_chrg_enbl before checking charger connection */
		mutex_lock(&mbi->event_lock);
		mbi->usr_chrg_enbl = value;
		mutex_unlock(&mbi->event_lock);

		retval = check_charger_conn(mbi);
		if (retval)
			dev_warn(msic_dev, "check_charger_conn failed\n");
	} else {
		mutex_lock(&mbi->event_lock);
		mbi->refresh_charger = 1;
		mbi->usr_chrg_enbl = value;
		mutex_unlock(&mbi->event_lock);
	}

	return count;
}

/**
 * get_chrg_enable - sysfs get api for charge_enable attribute
 * Parameter as define by sysfs interface
 * Context: can sleep
 *
 */
static ssize_t get_chrg_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev =
	    container_of(dev, struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);
	unsigned int val;

	mutex_lock(&mbi->event_lock);
	val = mbi->usr_chrg_enbl;
	mutex_unlock(&mbi->event_lock);

	return sprintf(buf, "%d\n", val);
}

/* get_is_power_supply_conn - sysfs get api for power_supply_conn attribute
 * Parameter as defined by sysfs interface
 * Context: can sleep
 *
 */
static ssize_t get_is_power_supply_conn(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", !(intel_msic_is_current_sense_enabled()));
}
/**
 * sfi_table_invalid_batt - default battery SFI table values  to be
 * used in case of invalid battery
 *
 * @sfi_table : sfi table pointer
 * Context: can sleep
 *
 */
static void sfi_table_invalid_batt(struct msic_batt_sfi_prop *sfi_table)
{

	/*
	 * In case of invalid battery we manually set
	 * the SFI parameters and limit the battery from
	 * charging, so platform will be in discharging mode
	 */
	memcpy(sfi_table->batt_id, "UNKNOWN", sizeof("UNKNOWN"));
	sfi_table->voltage_max = CHR_CHRVOLTAGE_SET_DEF;
	sfi_table->capacity = DEFAULT_MAX_CAPACITY;
	sfi_table->battery_type = POWER_SUPPLY_TECHNOLOGY_LION;
	sfi_table->temp_mon_ranges = 0;

}

/**
 * sfi_table_populate - Simple Firmware Interface table Populate
 * @sfi_table: Simple Firmware Interface table structure
 *
 * SFI table has entries for the temperature limits
 * which is populated in a local structure
 */
static int __init sfi_table_populate(struct sfi_table_header *table)
{
	struct sfi_table_simple *sb;
	struct msic_batt_sfi_prop *pentry;
	struct platform_device *pdev =
	    container_of(msic_dev, struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);
	int totentrs = 0, totlen = 0;

	sb = (struct sfi_table_simple *)table;
	if (!sb) {
		dev_warn(msic_dev, "SFI: Unable to map BATT signature\n");
		mbi->is_batt_valid = false;
		return -ENODEV;
	}

	totentrs = SFI_GET_NUM_ENTRIES(sb, struct msic_batt_sfi_prop);
	if (totentrs) {
		pentry = (struct msic_batt_sfi_prop *)sb->pentry;
		totlen = totentrs * sizeof(*pentry);
		memcpy(sfi_table, pentry, totlen);
		mbi->is_batt_valid = true;
		if (sfi_table->temp_mon_ranges != SFI_TEMP_NR_RNG)
			dev_warn(msic_dev, "SFI: temperature monitoring range"
				"doesn't match with its Array elements size\n");
	} else {
		dev_warn(msic_dev, "Invalid battery detected\n");
		sfi_table_invalid_batt(sfi_table);
		mbi->is_batt_valid = false;
	}

	return 0;
}

/**
 * init_batt_props - initialize battery properties
 * @mbi: msic module device structure
 * Context: can sleep
 *
 * init_batt_props function initializes the
 * MSIC battery properties.
 */
static void init_batt_props(struct msic_power_module_info *mbi)
{
	unsigned char data;
	int retval;

	mbi->batt_event = USBCHRG_EVENT_DISCONN;
	mbi->charging_mode = BATT_CHARGING_MODE_NONE;
	mbi->usr_chrg_enbl = USER_SET_CHRG_NOLMT;
	mbi->in_cur_lmt = CHRCNTL_VINLMT_NOLMT;

	mbi->batt_props.status = POWER_SUPPLY_STATUS_DISCHARGING;
	mbi->batt_props.health = POWER_SUPPLY_HEALTH_GOOD;
	mbi->batt_props.present = MSIC_BATT_NOT_PRESENT;

	/* read specific to determine the status */
	retval = intel_scu_ipc_ioread8(MSIC_BATT_CHR_SPWRSRCINT_ADDR, &data);
	if (retval)
		handle_ipc_rw_status(retval, MSIC_BATT_CHR_SPWRSRCINT_ADDR,
				MSIC_IPC_READ);

	/* determine battery Presence */
	if (data & MSIC_BATT_CHR_BATTDET_MASK)
		mbi->batt_props.present = MSIC_BATT_PRESENT;
	else
		mbi->batt_props.present = MSIC_BATT_NOT_PRESENT;

	/* Enable Status Register */
	retval = intel_scu_ipc_iowrite8(CHR_STATUS_FAULT_REG,
					CHR_STATUS_TMR_RST |
					CHR_STATUS_STAT_ENBL);
	if (retval)
		handle_ipc_rw_status(retval,
				CHR_STATUS_FAULT_REG, MSIC_IPC_WRITE);

	/* Disable charger LED */
	ipc_read_modify_chr_param_reg(mbi, MSIC_CHRG_LED_CNTL_REG,
				      MSIC_CHRG_LED_ENBL, 0);

	/*
	 * Mask Battery Detect Interrupt, we are not
	 * handling battery jarring in the driver.
	 */
	ipc_read_modify_reg(mbi, MSIC_BATT_CHR_MPWRSRCINT_ADDR,
			    MSIC_MPWRSRCINT_BATTDET, 1);
}

static u8 compute_pwrsrc_lmt_reg_val(int temp_high, int temp_low)
{
	u8 data = 0;
	if (temp_high >= 60)
		data |= CHR_PWRSRCLMT_TMPH_60;
	else if (temp_high >= 55)
		data |= CHR_PWRSRCLMT_TMPH_55;
	else if (temp_high >= 50)
		data |= CHR_PWRSRCLMT_TMPH_50;
	else
		data |= CHR_PWRSRCLMT_TMPH_45;

	if (temp_low >= 15)
		data |= CHR_PWRSRCLMT_TMPL_15;
	else if (temp_low >= 10)
		data |= CHR_PWRSRCLMT_TMPL_10;
	else if (temp_low >= 5)
		data |= CHR_PWRSRCLMT_TMPL_05;
	else
		data |= CHR_PWRSRCLMT_TMPL_0;

	return data;

}

/**
 * init_batt_thresholds - initialize battery thresholds
 * @mbi: msic module device structure
 * Context: can sleep
 */
static void init_batt_thresholds(struct msic_power_module_info *mbi)
{
	int ret;
	static const u16 address[] = {
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_PWRSRCLMT_ADDR,
	};
	static u8 data[2];

	batt_thrshlds->vbatt_sh_min = MSIC_BATT_VMIN_THRESHOLD;
	batt_thrshlds->vbatt_crit = BATT_CRIT_CUTOFF_VOLT;
	batt_thrshlds->temp_high = MSIC_BATT_TEMP_MAX;
	batt_thrshlds->temp_low = MSIC_BATT_TEMP_MIN;

	/* Read the threshold data from SMIP */
	dev_dbg(msic_dev, "[SMIP Read] offset: %x\n", BATT_SMIP_BASE_OFFSET);
	ret = intel_scu_ipc_read_mip((u8 *) batt_thrshlds,
			  sizeof(struct batt_safety_thresholds),
			  BATT_SMIP_BASE_OFFSET, 1);
	if (ret)
		dev_warn(msic_dev, "%s: smip read failed\n", __func__);

	if (mbi->is_batt_valid)
		get_batt_temp_thresholds(&batt_thrshlds->temp_high,
				&batt_thrshlds->temp_low);

	data[0] = WDTWRITE_UNLOCK_VALUE;
	data[1] = compute_pwrsrc_lmt_reg_val(batt_thrshlds->temp_high,
			batt_thrshlds->temp_low);
	if (msic_chr_write_multi(mbi, address, data, 2))
		dev_err(msic_dev, "Error in programming PWRSRCLMT reg\n");

	dev_dbg(msic_dev, "vbatt shutdown: %d\n", batt_thrshlds->vbatt_sh_min);
	dev_dbg(msic_dev, "vbatt_crit: %d\n", batt_thrshlds->vbatt_crit);
	dev_dbg(msic_dev, "Temp High Lmt: %d\n", batt_thrshlds->temp_high);
	dev_dbg(msic_dev, "Temp Low Lmt: %d\n", batt_thrshlds->temp_low);
}

/**
 * init_charger_props - initialize charger properties
 * @mbi: msic module device structure
 * Context: can sleep
 *
 * init_charger_props function initializes the
 * MSIC usb charger properties.
 */
static void init_charger_props(struct msic_power_module_info *mbi)
{
	mbi->usb_chrg_props.charger_present = MSIC_USB_CHARGER_NOT_PRESENT;
	mbi->usb_chrg_props.charger_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	memcpy(mbi->usb_chrg_props.charger_model, "Unknown", sizeof("Unknown"));
	memcpy(mbi->usb_chrg_props.charger_vender, "Unknown",
	       sizeof("Unknown"));
}

/**
 * init_msic_regs - initialize msic registers
 * @mbi: msic module device structure
 * Context: can sleep
 *
 * init_msic_regs function initializes the
 * MSIC registers like CV,Power Source LMT,etc..
 */
static int init_msic_regs(struct msic_power_module_info *mbi)
{
	static const u16 address[] = {
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRCVOLTAGE_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRTTIME_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_SPCHARGER_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRSTWDT_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_CHRCTRL1_ADDR,
		MSIC_BATT_CHR_WDTWRITE_ADDR, MSIC_BATT_CHR_VBUSDET_ADDR,
	};
	static u8 data[] = {
		WDTWRITE_UNLOCK_VALUE,
		CONV_VOL_DEC_MSICREG(CHR_CHRVOLTAGE_SET_DEF),
		WDTWRITE_UNLOCK_VALUE, CHR_CHRTIME_SET_13HRS,
		WDTWRITE_UNLOCK_VALUE,
		(~CHR_SPCHRGER_LOWCHR_ENABLE & CHR_SPCHRGER_WEAKVIN_LVL1),
		WDTWRITE_UNLOCK_VALUE, CHR_WDT_DISABLE,
		WDTWRITE_UNLOCK_VALUE, MSIC_CHRG_EXTCHRDIS,
		WDTWRITE_UNLOCK_VALUE, MSIC_BATT_CHR_VBUSDET_SET_MIN,
	};

	dump_registers(MSIC_CHRG_REG_DUMP_BOOT | MSIC_CHRG_REG_DUMP_EVENT);

	return msic_chr_write_multi(mbi, address, data, 12);
}

/**
 * msic_battery_probe - msic battery initialize
 * @pdev: msic battery ipc device structure
 * Context: can sleep
 *
 * MSIC battery initializes its internal data structure and other
 * infrastructure components for it to work as expected.
 */
static int msic_battery_probe(struct platform_device *pdev)
{
	int retval, read_temp;
	uint8_t data;
	struct msic_power_module_info *mbi = NULL;

	mbi = kzalloc(sizeof(struct msic_power_module_info), GFP_KERNEL);
	if (!mbi) {
		dev_err(&pdev->dev, "%s(): memory allocation failed\n",
			__func__);
		return -ENOMEM;
	}

	sfi_table = kzalloc(sizeof(struct msic_batt_sfi_prop), GFP_KERNEL);
	if (!sfi_table) {
		dev_err(&pdev->dev, "%s(): memory allocation failed\n",
			__func__);
		kfree(mbi);
		return -ENOMEM;
	}
	batt_thrshlds = kzalloc(sizeof(struct batt_safety_thresholds),
				GFP_KERNEL);
	if (!batt_thrshlds) {
		dev_err(&pdev->dev, "%s(): memory allocation failed\n",
			__func__);
		kfree(sfi_table);
		kfree(mbi);
		return -ENOMEM;
	}

	mbi->pdev = pdev;
	mbi->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, mbi);
	msic_dev = &pdev->dev;

	/* initialize all required framework before enabling interrupts */

	/* OTG Disconnect is being called from IRQ context
	 * so calling ipc function is not appropriate from otg callback
	 */
	INIT_DELAYED_WORK(&mbi->disconn_handler, msic_batt_disconn);
	INIT_DELAYED_WORK(&mbi->connect_handler, msic_batt_temp_charging);
	INIT_DELAYED_WORK_DEFERRABLE(&mbi->chr_status_monitor,
				     msic_status_monitor);
	INIT_DELAYED_WORK(&mbi->chrg_callback_dwrk, msic_chrg_callback_worker);
	wake_lock_init(&mbi->wakelock, WAKE_LOCK_SUSPEND,
		       "msicbattery_wakelock");

	/* Initialize mutex locks */
	mutex_init(&mbi->usb_chrg_lock);
	mutex_init(&mbi->batt_lock);
	mutex_init(&mbi->ipc_rw_lock);
	mutex_init(&mbi->event_lock);
	mutex_init(&mbi->adc_val_lock);

	/* Allocate ADC Channels */
	mbi->adc_handle =
	    intel_mid_gpadc_alloc(MSIC_BATT_SENSORS,
				  MSIC_BATT_PACK_TEMP | CH_NEED_VCALIB |
				  CH_NEED_VREF,
				  MSIC_USB_VOLTAGE | CH_NEED_VCALIB,
				  MSIC_BATTID | CH_NEED_VREF | CH_NEED_VCALIB);
	if (mbi->adc_handle == NULL)
		dev_err(&pdev->dev, "ADC allocation failed\n");

	/* check for valid SFI table entry for OEM0 table */
	if (sfi_table_parse(SFI_SIG_OEM0, NULL, NULL, sfi_table_populate)) {
		sfi_table_invalid_batt(sfi_table);
		mbi->is_batt_valid = false;
	}

	/* Initialize battery and charger Properties */
	init_batt_props(mbi);
	init_charger_props(mbi);
	init_batt_thresholds(mbi);

	/* Re Map Phy address space for MSIC regs */
	mbi->msic_intr_iomap = ioremap_nocache(MSIC_SRAM_INTR_ADDR, 8);
	if (!mbi->msic_intr_iomap) {
		dev_err(&pdev->dev, "battery: ioremap Failed\n");
		retval = -ENOMEM;
		goto ioremap_intr_failed;
	}

	/* Init MSIC Registers */
	retval = init_msic_regs(mbi);
	if (retval < 0)
		dev_err(&pdev->dev, "MSIC registers init failed\n");

	/* register msic-usb with power supply subsystem */
	mbi->usb.name = CHARGER_PS_NAME;
	mbi->usb.type = POWER_SUPPLY_TYPE_USB;
	mbi->usb.supplied_to = msic_power_supplied_to;
	mbi->usb.num_supplicants = ARRAY_SIZE(msic_power_supplied_to);
	mbi->usb.properties = msic_usb_props;
	mbi->usb.num_properties = ARRAY_SIZE(msic_usb_props);
	mbi->usb.get_property = msic_usb_get_property;
	retval = power_supply_register(&pdev->dev, &mbi->usb);
	if (retval) {
		dev_err(&pdev->dev, "%s(): failed to register msic usb "
			"device with power supply subsystem\n", __func__);
		goto power_reg_failed_usb;
	}

	retval = device_create_file(&pdev->dev, &dev_attr_charge_enable);
	if (retval)
		goto  sysfs1_create_failed;
	retval = device_create_file(&pdev->dev, &dev_attr_power_supply_conn);
	if (retval)
		goto sysfs2_create_failed;
	/* Register with OTG */
	otg_handle = penwell_otg_register_bc_callback(msic_charger_callback,
						      (void *)mbi);
	if (!otg_handle) {
		dev_err(&pdev->dev, "battery: OTG Registration failed\n");
		retval = -EBUSY;
		goto otg_failed;
	}

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&mbi->pdev->dev);
	pm_schedule_suspend(&mbi->pdev->dev, MSEC_PER_SEC);

	/* Check if already exist a Charger connection */
	retval = check_charger_conn(mbi);
	if (retval)
		dev_err(&pdev->dev, "check charger Conn failed\n");

	mbi->chrint_mask = CHRINT_MASK;
	mbi->chrint1_mask = CHRINT1_MASK;

	retval = intel_scu_ipc_iowrite8(MSIC_BATT_CHR_MCHRINT_ADDR,
				       mbi->chrint_mask);
	if (retval)
		handle_ipc_rw_status(retval,
			       MSIC_BATT_CHR_MCHRINT_ADDR, MSIC_IPC_WRITE);

	retval = intel_scu_ipc_iowrite8(MSIC_BATT_CHR_MCHRINT1_ADDR,
				       mbi->chrint1_mask);
	if (retval)
		handle_ipc_rw_status(retval,
				MSIC_BATT_CHR_MCHRINT1_ADDR, MSIC_IPC_WRITE);

	/* register interrupt */
	retval = request_threaded_irq(mbi->irq, msic_battery_interrupt_handler,
				      msic_battery_thread_handler,
				      IRQF_NO_SUSPEND, DRIVER_NAME, mbi);
	if (retval) {
		dev_err(&pdev->dev, "%s(): cannot get IRQ\n", __func__);
		goto requestirq_failed;
	}

	/*
	 * When no battery is present and the board is operated from
	 * a lab power supply, the battery thermistor is absent.
	 * In this case, the MSIC reports emergency temperature warnings,
	 * which must be ignored, to avoid a rain of interrupts
	 * (KFIFO_FULL messages)
	 * By reading the thermistor value on BPTHERM1 during driver probe
	 * it is possible to detect operation without a battery and
	 * mask the undesired MSIC interrupt in this case
	 *
	 */
	mdf_multi_read_adc_regs(mbi, HYSTR_SAMPLE_MAX, 1,
				MSIC_ADC_TEMP_IDX, &read_temp);

	if (read_temp == -ERANGE) {
		dev_warn(msic_dev,
			 "Temp read out of range:"
				"disabling BATTOTP interrupts");

		retval = intel_scu_ipc_ioread8(MSIC_BATT_CHR_MCHRINT_ADDR,
					       &data);
		if (retval) {
			retval = handle_ipc_rw_status(retval,
				MSIC_BATT_CHR_MCHRINT_ADDR, MSIC_IPC_READ);
			if (retval)
				return retval;
		}

		/* Applying BATTOTP INT mask */
		data |= MSIC_BATT_CHR_BATTOTP_MASK;
		retval = intel_scu_ipc_iowrite8(MSIC_BATT_CHR_MCHRINT_ADDR,
						data);
		if (retval) {
			retval = handle_ipc_rw_status(retval,
			       MSIC_BATT_CHR_MCHRINT_ADDR, MSIC_IPC_WRITE);
			return retval;
		}
	}

	/* Start the status monitoring worker */
	schedule_delayed_work(&mbi->chr_status_monitor, 0);
	return retval;

requestirq_failed:
	penwell_otg_unregister_bc_callback(otg_handle);
otg_failed:
	device_remove_file(&pdev->dev, &dev_attr_power_supply_conn);
sysfs2_create_failed:
	device_remove_file(&pdev->dev, &dev_attr_charge_enable);
sysfs1_create_failed:
	power_supply_unregister(&mbi->usb);
power_reg_failed_usb:
	iounmap(mbi->msic_intr_iomap);
ioremap_intr_failed:
	kfree(batt_thrshlds);
	kfree(sfi_table);
	kfree(mbi);

	return retval;
}

static void do_exit_ops(struct msic_power_module_info *mbi)
{
	/* disable MSIC Charger */
	mutex_lock(&mbi->batt_lock);
	if (mbi->batt_props.status != POWER_SUPPLY_STATUS_DISCHARGING)
		msic_batt_stop_charging(mbi);
	mutex_unlock(&mbi->batt_lock);
}

/**
 * msic_battery_remove - msic battery finalize
 * @pdev: msic battery platform device structure
 * Context: can sleep
 *
 * MSIC battery finalizes its internal data structure and other
 * infrastructure components that it initialized in
 * msic_battery_probe.
 */
static int msic_battery_remove(struct platform_device *pdev)
{
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);

	if (mbi) {
		penwell_otg_unregister_bc_callback(otg_handle);
		flush_scheduled_work();
		intel_mid_gpadc_free(mbi->adc_handle);
		free_irq(mbi->irq, mbi);
		pm_runtime_get_noresume(&mbi->pdev->dev);
		do_exit_ops(mbi);
		if (mbi->msic_intr_iomap != NULL)
			iounmap(mbi->msic_intr_iomap);
		device_remove_file(&pdev->dev, &dev_attr_charge_enable);
		device_remove_file(&pdev->dev, &dev_attr_power_supply_conn);
		power_supply_unregister(&mbi->usb);
		wake_lock_destroy(&mbi->wakelock);

		kfree(batt_thrshlds);
		kfree(sfi_table);
		kfree(mbi);
	}

	return 0;
}

static int battery_reboot_notifier_callback(struct notifier_block *notifier,
		unsigned long event, void *data)
{
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);

	if (mbi)
		do_exit_ops(mbi);

	return NOTIFY_OK;
}

#ifdef CONFIG_PM
static int msic_battery_suspend(struct device *dev)
{
	struct msic_power_module_info *mbi = dev_get_drvdata(dev);
	int event;

	mutex_lock(&mbi->event_lock);
	event = mbi->batt_event;
	mutex_unlock(&mbi->event_lock);

	if (event == USBCHRG_EVENT_CONNECT ||
	    event == USBCHRG_EVENT_UPDATE || event == USBCHRG_EVENT_RESUME) {

		msic_event_handler(mbi, USBCHRG_EVENT_SUSPEND, NULL);
		dev_dbg(&mbi->pdev->dev, "Forced suspend\n");
	}

	cancel_delayed_work_sync(&mbi->chr_status_monitor);

	return 0;
}

static int msic_battery_resume(struct device *dev)
{
	int retval = 0;
	struct msic_power_module_info *mbi = dev_get_drvdata(dev);
	int event;

	mutex_lock(&mbi->event_lock);
	event = mbi->batt_event;
	mutex_unlock(&mbi->event_lock);

	if (event == USBCHRG_EVENT_SUSPEND || event == USBCHRG_EVENT_DISCONN) {
		/* Check if already exist a Charger connection */
		retval = check_charger_conn(mbi);
		if (retval)
			dev_warn(msic_dev, "check_charger_conn failed\n");
	}

	schedule_delayed_work(&mbi->chr_status_monitor, 0);
	return retval;
}
#else
#define msic_battery_suspend    NULL
#define msic_battery_resume     NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int msic_runtime_suspend(struct device *dev)
{

	/* ToDo: Check for MSIC Power rails */
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int msic_runtime_resume(struct device *dev)
{
	/* ToDo: Check for MSIC Power rails */
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int msic_runtime_idle(struct device *dev)
{
	struct platform_device *pdev =
	    container_of(dev, struct platform_device, dev);
	struct msic_power_module_info *mbi = platform_get_drvdata(pdev);
	int event;

	dev_dbg(dev, "%s called\n", __func__);

	mutex_lock(&mbi->event_lock);
	event = mbi->batt_event;
	mutex_unlock(&mbi->event_lock);

	if (event == USBCHRG_EVENT_CONNECT ||
	    event == USBCHRG_EVENT_UPDATE || event == USBCHRG_EVENT_RESUME) {

		dev_warn(&mbi->pdev->dev, "%s: device busy\n", __func__);

		return -EBUSY;
	}

	return 0;
}
#else
#define msic_runtime_suspend	NULL
#define msic_runtime_resume	NULL
#define msic_runtime_idle	NULL
#endif
/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static const struct platform_device_id battery_id_table[] = {
	{"msic_battery", 1},
};

static const struct dev_pm_ops msic_batt_pm_ops = {
	.suspend = msic_battery_suspend,
	.resume = msic_battery_resume,
	.runtime_suspend = msic_runtime_suspend,
	.runtime_resume = msic_runtime_resume,
	.runtime_idle = msic_runtime_idle,
};

static struct platform_driver msic_battery_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .pm = &msic_batt_pm_ops,
		   },
	.probe = msic_battery_probe,
	.remove = __devexit_p(msic_battery_remove),
	.id_table = battery_id_table,
};

static int msic_battery_module_init(void)
{
	int ret;

	ret = platform_driver_register(&msic_battery_driver);
	if (ret)
		dev_err(msic_dev, "driver_register failed");

	if (register_reboot_notifier(&battery_reboot_notifier))
		dev_warn(msic_dev, "Battery: Unable to register reboot notifier");

	return ret;
}

static void msic_battery_module_exit(void)
{
	unregister_reboot_notifier(&battery_reboot_notifier);
	platform_driver_unregister(&msic_battery_driver);
}

static int msic_battery_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed msic_battery rpmsg device\n");

	ret = msic_battery_module_init();

out:
	return ret;
}

static void __devexit msic_battery_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	msic_battery_module_exit();
	dev_info(&rpdev->dev, "Removed msic_battery rpmsg device\n");
}

static void msic_battery_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id msic_battery_rpmsg_id_table[] = {
	{ .name	= "rpmsg_msic_battery" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, msic_battery_rpmsg_id_table);

static struct rpmsg_driver msic_battery_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= msic_battery_rpmsg_id_table,
	.probe		= msic_battery_rpmsg_probe,
	.callback	= msic_battery_rpmsg_cb,
	.remove		= __devexit_p(msic_battery_rpmsg_remove),
};

static int __init msic_battery_rpmsg_init(void)
{
	return register_rpmsg_driver(&msic_battery_rpmsg);
}

#ifdef MODULE
module_init(msic_battery_rpmsg_init);
#else
late_initcall_async(msic_battery_rpmsg_init);
#endif

static void __exit msic_battery_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&msic_battery_rpmsg);
}
module_exit(msic_battery_rpmsg_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_AUTHOR("Anantha Narayanan <anantha.narayanan@intel.com>");
MODULE_AUTHOR("Ananth Krishna <ananth.krishna.r@intel.com>");
MODULE_DESCRIPTION("Intel Medfield MSIC Battery Driver");
MODULE_LICENSE("GPL");
