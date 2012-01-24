/*
 * intel_mdf_fg.c - Intel Medfield MSIC Fuel Gauge Driver
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
 * This driver is based on intel_mdf_battery.c
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/async.h>

#include <asm/intel_scu_ipc.h>
#include <linux/power/intel_mdf_battery.h>
#include <linux/power/intel_mdf_fg.h>
#include <asm/intel_mid_gpadc.h>

#define DRIVER_NAME	"intel_mdf_fg"
#define DEV_NAME	"intel_mdf_fg"

/*********************************************************************
 *		Generic defines
 *********************************************************************/

#define MSIC_BATT_PRESENT		1
#define MSIC_BATT_NOT_PRESENT		0

/* ADC Channel Numbers */
#define MSIC_BATT_SENSORS	3
#define MSIC_BATT_PACK_VOL	0x0
#define MSIC_BATT_PACK_CUR	0x1
#define MSIC_BATT_PACK_TEMP	0x7
#define MSIC_ADC_VOL_IDX	0
#define MSIC_ADC_CUR_IDX	1
#define MSIC_ADC_TEMP_IDX	2

/*MSIC battery temperature  attributes*/
#define MSIC_BTP_ADC_MIN	107
#define MSIC_BTP_ADC_MAX	977

/*convert adc_val to voltage mV */
#define MSIC_MAX_VOL_DEV		((5 * 4692) / 1000)
#define MSIC_ADC_TO_VOL(adc_val)	((4692 * (adc_val)) / 1000)

/*convert adc_val to current mA */
#define MSIC_ADC_MAX_CUR		4000	/* In milli Amph */
#define MSIC_ADC_TO_CUR(adc_val)	((78125 * (adc_val)) / 10000)

/* SRAM Addresses for OCV location */
#define MSIC_SRAM_OCV_ADDR		0xFFFF3008

/* Battery VBatt is measured using ADC Channel but
 * ADC reading itself has some error which is around 40mV
 */
#define BATT_ADC_VOLT_ERROR		40

#define CHARGE_FULL_IN_MAH		1500	/* 1500 mAh */
#define MSIC_CHRG_RBATT_VAL		180	/* 180 mOhms */
#define COLMB_TO_MAHRS_CONV_FCTR	3600
#define IDLE_STATE_CUR_LMT		-500	/* -500mA */
#define MSIC_VBUS_LOW_VOLTAGE		4520	/* 4520 mV */

#define MSIC_BATT_TEMP_MAX		60	/* 60 degrees */
#define MSIC_BATT_TEMP_MIN		0
#define MSIC_EMRG_TEMP_LMT		-10	/* -10 Degrees */
#define MSIC_TEMP_HYST_ERR		4	/* 4 degrees */

/* IPC defines */
#define IPCMSG_BATTERY		0xEF
#define IPCCMD_CC_WRITE		0x00
#define IPCCMD_CC_READ		0x01

#define THERM_CURVE_MAX_SAMPLES 7
#define THERM_CURVE_MAX_VALUES	4
#define BATT_STRING_MAX		8
#define HYSTR_SAMPLE_MAX	4


#define BATTID_STR_LEN		8
#define MANFCT_STR_LEN		2
#define MODEL_STR_LEN		4
#define SFI_TEMP_NR_RNG		4
#define DCURVE_TBL_ID_LEN	4

#define DISCHRG_CURVE_MAX_SAMPLES 17
#define DISCHRG_CURVE_MAX_COLOMNS 2
#define CC_TIME_TO_LIVE (HZ/8)	/* 125 ms */
#define ADC_TIME_TO_LIVE (HZ/8)	/* 125 ms */

#define DIS_CURVE_LOOKUP_VOLTAGE	0
#define DIS_CURVE_LOOKUP_CHARGE		1

/*
 * This array represents the Discharge curve of the battery
 * Column 0 represents Voltage in mV and column 1 represent
 * charge in milli Coulombs.
 */
static short int dischargeCurve[DISCHRG_CURVE_MAX_SAMPLES]
	[DISCHRG_CURVE_MAX_COLOMNS] = {
	/* in mV , in C */
	{4200, 5580},
	{4150, 5412},
	{4100, 5118},
	{4050, 4764},
	{4000, 4356},
	{3950, 3864},
	{3900, 3366},
	{3850, 2928},
	{3800, 1698},
	{3750, 954},
	{3700, 366},
	{3650, 210},
	{3600, 150},
	{3550, 96},
	{3500, 60},
	{3450, 24},
	{3400, 0},
};

static struct device *msic_dev;

/* Variables for counting charge cycles */
static int ocv_cc_value;
static int cc_value;

static unsigned long long cc_ttl;
static unsigned long long adc_ttl;
static int adc_sensor_vals[MSIC_BATT_SENSORS];

/*
 * This array represents the Battery Pack thermistor
 * temperature and corresponding ADC value limits
 */
static int const therm_curve_data[THERM_CURVE_MAX_SAMPLES]
	[THERM_CURVE_MAX_VALUES] = {
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-10, -20, 977, 941},
	{0, -10, 941, 887},
	{10, 0, 887, 769},
	{50, 10, 769, 357},
	{75, 50, 357, 186},
	{100, 75, 186, 107},
};

/* Battery discharge curve info which need to get or set from UMIP area */
struct umip_dischrg_curve {
	short int rev_num;
	u8 tbl_name[DCURVE_TBL_ID_LEN];
	u8 batt_id[BATTID_STR_LEN];
	short int tbl_size;
	u8 tbl_type;
	u8 rsved1;
	u8 tbl_props;
	u8 rsved2;
	short int chrg_cycle_cnt;
	short int chrg_full_val;
	u8 rsved3;
	short int dischrg_tbl[DISCHRG_CURVE_MAX_SAMPLES]
	    [DISCHRG_CURVE_MAX_COLOMNS];
} __packed;

static struct umip_dischrg_curve *mip_dcurve;

/*********************************************************************
 *		Battery properties
 *********************************************************************/
struct msic_batt_props {
	unsigned int status;
	unsigned int health;
	unsigned int present;
	unsigned int technology;
	unsigned int vol_now;
	unsigned int cur_now;
	unsigned int charge_now;	/* in mAh */
	unsigned int charge_full_des;	/* in mAh */
	unsigned int charge_full;	/* in mAh */
	unsigned int capacity;	/* in units percentage */
	unsigned int temperature;	/* in milli Centigrade */
	char model[BATT_STRING_MAX];
	char vender[BATT_STRING_MAX];
};

/*
 * msic battery info
 */
struct msic_fg_module_info {

	struct platform_device *pdev;
	struct msic_fg_platform_data *pdata;
	bool is_batt_valid;

	/* msic battery data */
	/* lock to protect battery  properties
	 * locking is applied whenever read or write
	 * operation is being performed to the msic battery
	 * property structure.
	 */
	struct mutex batt_lock;
	struct msic_batt_props batt_props;
	struct power_supply batt;

	/* Open circuit voltage SRAM address */
	void __iomem *msic_ocv_iomap;

	/* Handle for gpadc requests */
	void *adc_handle;

	struct delayed_work init_worker;
	int is_csense_enabled;
	struct mutex adc_val_lock;
};

static unsigned int dischrg_curve_lookup(unsigned int val, int col);

/*
 * msic battery properties
 */
static enum power_supply_property msic_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

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

/* Convert ADC codes to Temperature */
static int adc_to_temp(uint16_t adc_val)
{
	int temp = 0;
	int i;

	if (!is_valid_temp_adc(adc_val))
		return -ERANGE;

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

/* Check if last read time interval elapsed */
static bool is_ttl_valid(u64 ttl)
{
	if (time_before64(get_jiffies_64(), ttl))
		return true;
	else
		return false;
}

/**
 * msic_fg_multi_read_adc_regs - function to read multiple adc channels
 * @mbi: msic fuel gauge module device structure
 * @sample_count: no.of sample to take
 * @channel_count: no.of channels to read
 * Total numbers of arguments depends on the channel_count.
 * Context: can sleep
 *
 * Reads the MSIC ADC channels based on channel count and sample count.
 */
static int msic_fg_multi_read_adc_regs(struct msic_fg_module_info *mbi,
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
					   &adc_sensor_vals[MSIC_ADC_VOL_IDX],
					   &adc_sensor_vals[MSIC_ADC_CUR_IDX],
					   &adc_sensor_vals[MSIC_ADC_TEMP_IDX]);
		if (ret) {
			dev_err(&mbi->pdev->dev, "%s: failed", __func__);
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
		case MSIC_ADC_VOL_IDX:
			tmp = MSIC_ADC_TO_VOL(*adc_val);
			break;
		case MSIC_ADC_CUR_IDX:
			tmp = MSIC_ADC_TO_CUR(*adc_val & 0x1FF);
			/* if D9 bit is set battery is discharging */
			if (*adc_val & 0x200)
				tmp = -(MSIC_ADC_MAX_CUR - tmp);
			break;
		case MSIC_ADC_TEMP_IDX:
			tmp = adc_to_temp(*adc_val);
			break;
		default:
			dev_err(&mbi->pdev->dev, "invalid sensor%d", sensor);
			ret = -EINVAL;
			goto adc_multi_exit;
		}
		*adc_val = tmp;
	}
	va_end(args);

adc_multi_exit:
	return ret;
}

/* Wrapper function to read a single ADC channel */
static int msic_fg_read_adc_regs(int sensor, struct msic_fg_module_info *mbi)
{
	int ret;
	int sensor_val;
	ret = msic_fg_multi_read_adc_regs(mbi, 1, 1, sensor, &sensor_val);

	if (ret) {
		dev_err(&mbi->pdev->dev, "%s:adc reads failed", __func__);
		return ret;
	}

	return sensor_val;
}

/* Reads raw coulomb counter value */
static int msic_fg_read_coulomb_ctr(void)
{
	int err;

	/* determine other parameters */
	if (!is_ttl_valid(cc_ttl)) {
		err =
		    intel_scu_ipc_command(IPCMSG_BATTERY, IPCCMD_CC_READ, NULL,
					  0, &cc_value, 1);
		if (err)
			dev_warn(msic_dev, "IPC Command Failed %s\n", __func__);

		cc_ttl = get_jiffies_64() + CC_TIME_TO_LIVE;
	}

	return cc_value;
}

static int cc_to_coulombs(int cc_val)
{
	long coulombs = 0;

	/* Every LSB of cc adc bit equal to 95.37uC
	 * Approximating it to 95uC
	 * Then convert into milli coulombs
	 */
	coulombs = (cc_val / 1000 * 95) + (cc_val % 1000 * 95) / 1000;

	return (int)coulombs;
}

/* Reads coulomb counter value and
 * returns the value in milli coulombs */
static unsigned int msic_fg_get_charge_now(void)
{
	int temp_val, coulombs;
	unsigned int voltage;
	struct platform_device *pdev = container_of(msic_dev,
					struct platform_device, dev);
	struct msic_fg_module_info *mbi = platform_get_drvdata(pdev);

	/* If current sense is disabled use voltage based Fuel Gauging */
	if (!mbi->is_csense_enabled) {
		voltage = msic_fg_read_adc_regs(MSIC_ADC_VOL_IDX, mbi);
		return dischrg_curve_lookup(voltage,
		    DIS_CURVE_LOOKUP_VOLTAGE) * 1000 /
		    COLMB_TO_MAHRS_CONV_FCTR;
	}
	temp_val = msic_fg_read_coulomb_ctr();
	coulombs = cc_to_coulombs(temp_val);
	dev_dbg(msic_dev, "amount of charge added %d\n", coulombs);

	/* Correcting the charge value */
	coulombs = ocv_cc_value + coulombs;

	/*
	 * Convert the milli coulombs into mAh
	 * 1 mAh = 3600 mC
	 */
	return coulombs / COLMB_TO_MAHRS_CONV_FCTR;
}

/* Discharge curve lookup function to get
 * charge value against voltage and vice versa */
static unsigned int dischrg_curve_lookup(unsigned int val, int col)
{
	int val_diff, val_total_diff, total_diff, i, k;
	unsigned int lookup_val;

	if (col < 0 || col >= DISCHRG_CURVE_MAX_COLOMNS) {
		dev_err(msic_dev, "%s:array out of bounds\n", __func__);
		return 0;
	}

	/* if col = 0 input is voltage, we need to lookup
	 * for charge else if if col = 1 input is charge,
	 * we need to lookup for voltage
	 */
	if (col == DIS_CURVE_LOOKUP_VOLTAGE)
		k = DIS_CURVE_LOOKUP_CHARGE;
	else
		k = DIS_CURVE_LOOKUP_VOLTAGE;

	/* Find the index of most nearest sample value */
	for (i = 0; i < DISCHRG_CURVE_MAX_SAMPLES; i++) {
		if (val < dischargeCurve[i][col])
			continue;
		else
			break;
	}

	if (i >= DISCHRG_CURVE_MAX_SAMPLES) {
		dev_dbg(msic_dev, "charge out of range\n");
		return 0;
	}

	if ((i == 0) || (val == dischargeCurve[i][col]))
		return dischargeCurve[i][k];

	/* Linear approximation of the discharge curve */
	val_diff = val - dischargeCurve[i][col];
	val_total_diff = dischargeCurve[i - 1][col] - dischargeCurve[i][col];
	total_diff = dischargeCurve[i - 1][k] - dischargeCurve[i][k];

	lookup_val = dischargeCurve[i][k] +
	    (total_diff * val_diff) / val_total_diff;

	return lookup_val;
}

static unsigned int calculate_batt_capacity(struct msic_fg_module_info *mbi)
{
	unsigned int cap_perc, charge_now;

	charge_now = msic_fg_get_charge_now();
	cap_perc = (charge_now * 100) / (mbi->batt_props.charge_full);

	if (cap_perc > 100)
		cap_perc = 100;
	if ((int)charge_now < 0)
		cap_perc = 0;

	return cap_perc;
}

/**
 * msic_fg_get_property - battery power source get property
 * @psy: battery power supply context
 * @psp: battery power source property
 * @val: battery power source property value
 * Context: can sleep
 *
 * MSIC battery power source property needs to be provided to power_supply
 * subsystem for it to provide the information to users.
 */
static int msic_fg_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct msic_fg_module_info *mbi =
	    container_of(psy, struct msic_fg_module_info, batt);

	/*
	 * All voltages, currents, charges, energies, time and temperatures
	 * in uV, µA, µAh, µWh, seconds and tenths of degree Celsius un
	 * less otherwise stated. It's driver's job to convert its raw values
	 * to units in which this class operates.
	 */

	mutex_lock(&mbi->batt_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = mbi->batt_props.status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = mbi->batt_props.health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = mbi->batt_props.present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = mbi->batt_props.technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mbi->batt_props.vol_now =
		    msic_fg_read_adc_regs(MSIC_ADC_VOL_IDX, mbi);
		val->intval = mbi->batt_props.vol_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mbi->batt_props.cur_now =
		    msic_fg_read_adc_regs(MSIC_ADC_CUR_IDX, mbi);
		val->intval = mbi->batt_props.cur_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		mbi->batt_props.charge_now = msic_fg_get_charge_now();
		val->intval = mbi->batt_props.charge_now * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = mbi->batt_props.charge_full_des * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = mbi->batt_props.charge_full * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		mbi->batt_props.capacity = calculate_batt_capacity(mbi);
		val->intval = mbi->batt_props.capacity;
		/* As the Coulomb counter is not working properly and
		 * even with Power Supply connected system is shutting
		 * down due to coulomb counter decrement so as WA
		 * we will report the capacity as 25 perc if it goes low.
		 */
		if (val->intval < 25)
			val->intval = 25;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		mbi->batt_props.temperature =
		    msic_fg_read_adc_regs(MSIC_ADC_TEMP_IDX, mbi);
		/*
		 * Temperature is measured in units of degrees celcius, the
		 * power_supply class measures temperature in tenths of degrees
		 * celsius.
		 */
		val->intval = mbi->batt_props.temperature * 10;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = mbi->batt_props.model;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = mbi->batt_props.vender;
		break;
	default:
		mutex_unlock(&mbi->batt_lock);
		return -EINVAL;
	}

	mutex_unlock(&mbi->batt_lock);

	return 0;
}

static void msic_fg_restore_conf_data(struct msic_fg_module_info *mbi)
{
	int retval;

	if (!mbi->pdata->is_init_done && mbi->pdata->restore_config_data) {
		retval = mbi->pdata->restore_config_data(DEV_NAME,
					mip_dcurve, sizeof(*mip_dcurve));

		if (retval == -ENXIO) {		/* no device found */
			dev_info(&mbi->pdev->dev, "device not found\n");
			mbi->pdata->is_init_done = 1;
		} else if (retval < 0) {	/* device not ready */
			dev_info(&mbi->pdev->dev, "device not ready\n");
		} else {			/* device found */
			mbi->pdata->is_init_done = 1;
		}
	}
}

/**
 * msic_fg_init_worker - worker function to get config data
 * @work: delayed work handler structure
 * Context: Can sleep
 *
 * Calls the msic_fg_restore_conf_data() which gets the config data
 * from non-volatile memory.
 */
static void msic_fg_init_worker(struct work_struct *work)
{
	struct msic_fg_module_info *mbi = container_of(work,
				struct msic_fg_module_info, init_worker.work);

	dev_info(&mbi->pdev->dev, "%s\n", __func__);
	msic_fg_restore_conf_data(mbi);
}

static void update_batt_ps_info(struct msic_fg_module_info *mbi)
{
	mutex_lock(&mbi->batt_lock);
	if (mbi->pdata->battery_present)
		mbi->batt_props.present = mbi->pdata->battery_present();
	/* get the battery status */
	if (mbi->pdata->battery_status)
		mbi->batt_props.status = mbi->pdata->battery_status();

	/* get the battery health */
	if (mbi->pdata->battery_health)
		mbi->batt_props.health = mbi->pdata->battery_health();
	mutex_unlock(&mbi->batt_lock);

	power_supply_changed(&mbi->batt);
}

static void msic_fg_external_power_changed(struct power_supply *psy)
{
	struct msic_fg_module_info *mbi = container_of(psy,
				struct msic_fg_module_info, batt);

	if (!mbi->pdata->is_init_done) {
		/* get the current sense info*/
		if (mbi->pdata->current_sense_enabled)
			mbi->is_csense_enabled =
				mbi->pdata->current_sense_enabled();
		schedule_delayed_work(&mbi->init_worker, 0);
		return ;
	}

	update_batt_ps_info(mbi);
}

/* Read the OCV SRAM locations to get open circuit voltage
 * and does a lookup for the initial charge value */
static void ipc_ocv_to_chrg_update(struct msic_fg_module_info *mbi)
{
	unsigned int vocv;


	vocv = readl(mbi->msic_ocv_iomap);
	vocv = MSIC_ADC_TO_VOL(vocv);
	dev_dbg(msic_dev, "sram vocv conv:%d\n", vocv);

	/* lookup for equivalent Open circuit charge value */
	ocv_cc_value = dischrg_curve_lookup(vocv, DIS_CURVE_LOOKUP_VOLTAGE);
	/* convert from coulombs to milli coulombs */
	ocv_cc_value *= 1000;
	dev_dbg(msic_dev, "Open circuit CC value:%d\n", ocv_cc_value);
}

static void init_msic_fg_config_data()
{
	memcpy(mip_dcurve->batt_id, "INCDK000", BATTID_STR_LEN);
	mip_dcurve->chrg_full_val = CHARGE_FULL_IN_MAH;
	memcpy(mip_dcurve->dischrg_tbl, dischargeCurve,
		DISCHRG_CURVE_MAX_SAMPLES * DISCHRG_CURVE_MAX_COLOMNS);
}

/**
 * init_batt_props - initialize battery properties
 * @mbi: msic module device structure
 * Context: can sleep
 *
 * init_batt_props function initializes the
 * MSIC battery properties.
 */
static void init_batt_props(struct msic_fg_module_info *mbi)
{
	mbi->batt_props.status = POWER_SUPPLY_STATUS_DISCHARGING;
	mbi->batt_props.health = POWER_SUPPLY_HEALTH_GOOD;
	mbi->batt_props.present = MSIC_BATT_PRESENT;
	mbi->batt_props.technology = mbi->pdata->technology;
	mbi->batt_props.charge_full_des = mip_dcurve->chrg_full_val;
	mbi->batt_props.charge_full = mip_dcurve->chrg_full_val;

	memcpy(mbi->batt_props.vender, mip_dcurve->batt_id, MANFCT_STR_LEN);
	memcpy(mbi->batt_props.model, mip_dcurve->batt_id + MANFCT_STR_LEN,
							MODEL_STR_LEN);
	mbi->pdata->is_init_done = 1;
}

/**
 * msic_fg_probe - msic fuel gauge initialize
 * @pdev: msic fuel gauge platform device structure
 * Context: can sleep
 *
 * MSIC fuel gauge initializes its internal data structure and other
 * infrastructure components for it to work as expected.
 */
static int msic_fg_probe(struct platform_device *pdev)
{
	int retval;
	struct msic_fg_module_info *mbi = NULL;
	mbi = kzalloc(sizeof(struct msic_fg_module_info), GFP_KERNEL);
	if (!mbi) {
		dev_err(&pdev->dev, "%s(): memory allocation failed\n",
			__func__);
		return -ENOMEM;
	}

	mip_dcurve = kzalloc(sizeof(struct umip_dischrg_curve), GFP_KERNEL);
	if (!mip_dcurve) {
		dev_err(&pdev->dev, "%s(): memory allocation failed\n",
			__func__);
		kfree(mbi);
		return -ENOMEM;
	}

	mbi->pdev = pdev;
	mbi->pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, mbi);
	msic_dev = &pdev->dev;

	/* Initialize mutex locks */
	mutex_init(&mbi->batt_lock);
	mutex_init(&mbi->adc_val_lock);
	INIT_DELAYED_WORK(&mbi->init_worker, msic_fg_init_worker);

	/* Allocate ADC Channels */
	mbi->adc_handle =
	    intel_mid_gpadc_alloc(MSIC_BATT_SENSORS,
				  MSIC_BATT_PACK_VOL | CH_NEED_VCALIB,
				  MSIC_BATT_PACK_CUR | CH_NEED_ICALIB,
				  MSIC_BATT_PACK_TEMP | CH_NEED_VCALIB |
				  CH_NEED_VREF);
	if (mbi->adc_handle == NULL)
		dev_err(&pdev->dev, "ADC allocation failed\n");

	/* init discharge curve data */
	init_msic_fg_config_data();

	/* Initialize battery and charger Properties */
	init_batt_props(mbi);


	mbi->msic_ocv_iomap = ioremap_nocache(MSIC_SRAM_OCV_ADDR, 8);
	if (!mbi->msic_ocv_iomap) {
		dev_err(&pdev->dev, "battery: ioremap of fuel"
			" gauging regs Failed\n");
		retval = -ENOMEM;
		goto ioremap_ocv_failed;
	}
	ipc_ocv_to_chrg_update(mbi);

	/* register msic-batt with power supply subsystem */
	mbi->batt.name = "msic_battery";
	mbi->batt.type = POWER_SUPPLY_TYPE_BATTERY;
	mbi->batt.properties = msic_battery_props;
	mbi->batt.num_properties = ARRAY_SIZE(msic_battery_props);
	mbi->batt.get_property = msic_fg_get_property;
	mbi->batt.external_power_changed = msic_fg_external_power_changed;
	retval = power_supply_register(&pdev->dev, &mbi->batt);
	if (retval) {
		dev_err(&pdev->dev, "%s(): failed to register msic battery "
			"device with power supply subsystem\n", __func__);
		goto power_reg_failed_batt;
	}

	/* Init Runtime PM State */
	pm_runtime_set_active(&mbi->pdev->dev);
	pm_runtime_enable(&mbi->pdev->dev);
	pm_schedule_suspend(&mbi->pdev->dev, MSEC_PER_SEC);

	return retval;

power_reg_failed_batt:
	iounmap(mbi->msic_ocv_iomap);
ioremap_ocv_failed:
	kfree(mip_dcurve);
	kfree(mbi);

	return retval;
}

/**
 * msic_fg_remove - msic fuel gauge finalize
 * @pdev: msic battery platform  device structure
 * Context: can sleep
 *
 * MSIC battery finalizes its internal data structure and other
 * infrastructure components that it initialized in
 * msic_fg_probe.
 */
static int msic_fg_remove(struct platform_device *pdev)
{
	struct msic_fg_module_info *mbi = platform_get_drvdata(pdev);

	if (mbi) {
		intel_mid_gpadc_free(mbi->adc_handle);
		if (mbi->msic_ocv_iomap != NULL)
			iounmap(mbi->msic_ocv_iomap);
		power_supply_unregister(&mbi->batt);
		kfree(mip_dcurve);
		kfree(mbi);
	}

	return 0;
}

#ifdef CONFIG_PM
static int msic_fg_suspend(struct device *dev)
{
	struct msic_fg_module_info *mbi = dev_get_drvdata(dev);

	return 0;
}

static int msic_fg_resume(struct device *dev)
{
	struct msic_fg_module_info *mbi = dev_get_drvdata(dev);

	return 0;
}
#else
#define msic_fg_suspend    NULL
#define msic_fg_resume     NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int msic_fg_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int msic_fg_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int msic_fg_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#else
#define msic_fg_runtime_suspend	NULL
#define msic_fg_runtime_resume	NULL
#define msic_fg_runtime_idle	NULL
#endif
/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static const struct platform_device_id fg_id_table[] = {
	{DEV_NAME, 1},
};

static const struct dev_pm_ops msic_fg_pm_ops = {
	.suspend = msic_fg_suspend,
	.resume = msic_fg_resume,
	.runtime_suspend = msic_fg_runtime_suspend,
	.runtime_resume = msic_fg_runtime_resume,
	.runtime_idle = msic_fg_runtime_idle,
};

static struct platform_driver msic_fg_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .pm = &msic_fg_pm_ops,
		   },
	.probe = msic_fg_probe,
	.remove = __devexit_p(msic_fg_remove),
	.id_table = fg_id_table,
};

static int __init msic_fg_module_init(void)
{
	int ret;
	ret = platform_driver_register(&msic_fg_driver);
	if (ret)
		dev_err(msic_dev, "driver_register failed");

	return ret;
}

static void __exit msic_fg_module_exit(void)
{
	platform_driver_unregister(&msic_fg_driver);
}

late_initcall_async(msic_fg_module_init);
module_exit(msic_fg_module_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Intel Medfield MSIC Fuel Gauge Driver");
MODULE_LICENSE("GPL");
