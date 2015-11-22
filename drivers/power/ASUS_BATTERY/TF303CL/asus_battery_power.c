/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "asus_battery.h"
#include "uPILogger.h"
#include "asus_battery_proc_fs.h"
#include "bq27520_battery_core.h"
#include "smb347_external_include.h"
#include "bq27520_battery_upt_i2c.h"
//#include <linux/HWVersion.h>
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>
#include "../intel_mdf_charger.h"
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_gpadc.h>

#define DISABLE_CHARGING_AT_LOW_TEMP false

static unsigned int  battery_current;
static unsigned int  battery_remaining_capacity;

module_param(battery_current , uint, 0644);
module_param(battery_remaining_capacity , uint, 0644);

//extern int Read_HW_ID(void);
extern int entry_mode;

//Carlisle add for ATD gague IC status
extern struct battery_dev_info bq27520_dev_info;

//struct delayed_work battery_low_init_work;		//battery low init
struct delayed_work battery_poll_data_work;		//polling data
struct delayed_work detect_cable_work;			//check cable status
struct workqueue_struct *battery_work_queue=NULL;

/* wake lock to prevent S3 during charging */
struct wake_lock wakelock;
struct wake_lock wakelock_t;    // for wake_lokc_timout() useage

DEFINE_MUTEX(batt_info_mutex);

static char *supply_list[] = {
        "battery",
};

#if CONFIG_ASUS_BATTERY_BQ27541
extern struct battery_info_reply batt_info
#else
struct battery_info_reply batt_info = {
        .drv_status = DRV_NOT_READY,
        .cable_status = NO_CABLE,
#ifdef ASUS_ENG_BUILD
        .eng_charging_limit = true,
#endif
        .thermal_charging_limit = false,
        .cable_source = CABLE_OUT,
        .gauge_version = 4,
        .isValidBattID = true,
};
#endif

#define MSIC_CHRG_REG_DUMP_OTHERS	(1 << 3)
static void pmic_dump_registers(int dump_mask)
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

    uint16_t reg_addr_others[] = {MSIC_BATT_CHR_MPWRSRCINT_ADDR,
        MSIC_BATT_CHR_MPWRSRCINT1_ADDR, MSIC_BATT_CHR_MCHRINT_ADDR,
        MSIC_BATT_CHR_MCHRINT1_ADDR, MSIC_BATT_CHR_VBUSDET_ADDR,
        MSIC_BATT_CHR_WDTWRITE_ADDR};
    char *reg_str_others[] = {"chrmpwrsrcint", "chrmpwrsrcint1", "chrmchrint",
                "chrmchrint1", "chrvbusdet", "chrwdtwrite"};

    if (dump_mask & MSIC_CHRG_REG_DUMP_BOOT) {
        for (i = 0; i < ARRAY_SIZE(reg_addr_boot); i++) {
            retval = intel_scu_ipc_ioread8(reg_addr_boot[i], &reg_val);
            if (retval) {
                chk_reg_addr = reg_addr_boot[i];
                goto ipcread_err;
            }
            BAT_DBG(" PMIC(0x%03x)\tval: " BYTETOBINARYPATTERN "\t%s\n", reg_addr_boot[i],
                                BYTETOBINARY(reg_val), reg_str_boot[i]);
        }
    }
    if (dump_mask & MSIC_CHRG_REG_DUMP_INT) {
        for (i = 0; i < ARRAY_SIZE(reg_addr_int); i++) {
            retval = intel_scu_ipc_ioread8(reg_addr_int[i], &reg_val);
            if (retval) {
                chk_reg_addr = reg_addr_int[i];
                goto ipcread_err;
            }
            BAT_DBG( "PMIC(0x%03x)\tval: " BYTETOBINARYPATTERN "\t%s\n", reg_addr_int[i],
                                BYTETOBINARY(reg_val), reg_str_int[i]);
        }
    }
    if (dump_mask & MSIC_CHRG_REG_DUMP_EVENT) {
        for (i = 0; i < ARRAY_SIZE(reg_addr_evt); i++) {
            retval = intel_scu_ipc_ioread8(reg_addr_evt[i], &reg_val);
            if (retval) {
                chk_reg_addr = reg_addr_evt[i];
                goto ipcread_err;
            }
            BAT_DBG(" PMIC(0x%03x)\tval: " BYTETOBINARYPATTERN "\t%s\n", reg_addr_evt[i],
                                BYTETOBINARY(reg_val), reg_str_evt[i]);
        }
    }
    if (dump_mask & MSIC_CHRG_REG_DUMP_OTHERS) {
        for (i = 0; i < ARRAY_SIZE(reg_addr_others); i++) {
            retval = intel_scu_ipc_ioread8(reg_addr_others[i], &reg_val);
            if (retval) {
                chk_reg_addr = reg_addr_others[i];
                goto ipcread_err;
            }
            BAT_DBG("PMIC(0x%03x)\t val: " BYTETOBINARYPATTERN "\t%s\n", reg_addr_others[i],
                                BYTETOBINARY(reg_val), reg_str_others[i]);
        }
    }

    return;

ipcread_err:
    BAT_DBG("ipcread_err: address: 0x%03x!!!", chk_reg_addr);
}

#ifdef ASUS_ENG_BUILD
int asus_charging_toggle_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
    int len = 0;

    BAT_DBG_E(" %s:\n", __func__);
    return len;
}

int asus_charging_toggle_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    struct battery_info_reply tmp_batt_info;
    bool eng_charging_limit = true;

    BAT_DBG(" %s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    eng_charging_limit = tmp_batt_info.eng_charging_limit;

    if (buffer[0] == '0') {
        /* turn on charging limit in eng mode */
        eng_charging_limit = true;
    }
    else if (buffer[0] == '1') {
        /* turn off charging limit in eng mode */
        eng_charging_limit = false;
    }

    tmp_batt_info.eng_charging_limit = eng_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return count;
}

int asus_charging_for_gague_toggle_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
    int len = 0;

    BAT_DBG_E(" %s:\n", __func__);
    return len;
}

int asus_charging_for_gague_toggle_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    struct battery_info_reply tmp_batt_info;
    //bool eng_charging_limit = true;

    BAT_DBG(" %s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    //eng_charging_limit = tmp_batt_info.eng_charging_limit;

    if (buffer[0] == '0') {
        /* turn on charging limit in eng mode */
        //eng_charging_limit = true;
        BAT_DBG_E(" disable charging:\n");
        smb347_charging_toggle(false);
    }
    else if (buffer[0] == '1') {
        /* turn off charging limit in eng mode */
        //eng_charging_limit = false;
        BAT_DBG_E(" enable charging:\n");
        smb347_charging_toggle(true);
    }

    //tmp_batt_info.eng_charging_limit = eng_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    return count;
}
//-->Carlisle add for ATD querying gauge IC status
int check_gaugeIC_status_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
    struct i2c_client *client = NULL;
    int len;
    int ret;
    int flags=0;
    int gauge_status=0;

    client = bq27520_dev_info.i2c;

    ret = bq27520_send_subcmd(client, &flags, BQ27520_SUBCMD_CTNL_STATUS);

    if (ret==0) {

	    gauge_status=1;
	    len = sprintf(page, "%d\n", gauge_status);
    }
    else{

	    dev_err(&client->dev, "Send subcommand error %d.\n", ret);
        len = sprintf(page, "%d\n", gauge_status);
	}


    return len;
}
//<--Carlisle add for ATD querying gauge IC status end
//-->Carlisle add for ATD querying charge status
int check_charge_status_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
	int res = 0;
	int len = 0;

    struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

    if(tmp_batt_info.status==POWER_SUPPLY_STATUS_CHARGING){
       res = 1;
       len = sprintf(page, "%d\n", res);
    }
    else
     len = sprintf(page, "%d\n", res);

    return len;
}
//<--Carlisle add for ATD querying charge status end
//-->Carlisle add for ATD querying gauge IC fw flash identify
int check_gauge_fw_status_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
	int len = 0;
	int ret = 0;
	int fw_cfg_status = 0;

	fw_cfg_status = bq27520_asus_battery_dev_read_fw_cfg_version();

	if(fw_cfg_status == LATEST_FW_CFG_VERSION){
		len = 1;
		ret = sprintf(page, "%d\n", len);
    }
	else
	 ret = sprintf(page, "%d\n", len);

    return ret;
}
//<--Carlisle add for ATD querying gauge IC fw flash identify end
//-->Carlisle add for ATD querying gauge IC status
int check_gaugeIC_status(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("gaugeIC_status", 0666, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create gaugeIC_status\n");
        return -EINVAL;
    }
    entry->read_proc = check_gaugeIC_status_read;

    return 0;
}
//<--Carlisle add for ATD querying gauge IC status end
//-->Carlisle add for ATD querying charge status
int check_charge_status(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("charge_status", 0666, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create charge_status\n");
        return -EINVAL;
    }
    entry->read_proc = check_charge_status_read;

    return 0;
}
//<--Carlisle add for ATD querying charge status end
//-->Carlisle add for ATD querying gauge IC fw flash identify
int check_gauge_fw_status(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("gauge_fw_status", 0666, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create gauge_fw_status\n");
        return -EINVAL;
    }
    entry->read_proc = check_gauge_fw_status_read;

    return 0;
}
//<--Carlisle add for ATD querying gauge IC fw flash identify end
int init_asus_for_gague_charging_toggle(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("asus_eng_for_gague_charging_limit", 0666, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create asus_charging_toggle\n");
        return -EINVAL;
    }
    entry->read_proc = asus_charging_for_gague_toggle_read;
    entry->write_proc = asus_charging_for_gague_toggle_write;

    return 0;
}

int init_asus_charging_toggle(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("asus_eng_charging_limit", 0666, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create asus_charging_toggle\n");
        return -EINVAL;
    }
    entry->read_proc = asus_charging_toggle_read;
    entry->write_proc = asus_charging_toggle_write;

    return 0;
}

#endif



//++Sewell+++ charging toggle interface for thermal
int thermal_charging_toggle_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
    int len = 0;

    BAT_DBG_E(" %s:\n", __func__);
    return len;
}

int thermal_charging_toggle_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    struct battery_info_reply tmp_batt_info;
    bool thermal_charging_limit = false;

     BAT_DBG(" %s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);


    if (buffer[0] == '0') {
        // Disable charging
        thermal_charging_limit = true;

    }
    else // Enable charging
        thermal_charging_limit = false;

    tmp_batt_info.thermal_charging_limit = thermal_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return count;
}

int init_thermal_charging_toggle(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("thermal_charging_limit", 0600, NULL);
    if (!entry) {
        BAT_DBG_E("Unable to create thermal_charging_toggle\n");
        return -EINVAL;
    }
    entry->read_proc = thermal_charging_toggle_read;
    entry->write_proc = thermal_charging_toggle_write;

    return 0;
}


//----Sewell----



static enum power_supply_property asus_battery_props[] = {
        POWER_SUPPLY_PROP_STATUS,				//0x00
        POWER_SUPPLY_PROP_HEALTH,				//0x02
        POWER_SUPPLY_PROP_PRESENT,				//0x03
        POWER_SUPPLY_PROP_TECHNOLOGY,			//0x05
        POWER_SUPPLY_PROP_VOLTAGE_NOW,			//0x0A
        POWER_SUPPLY_PROP_CURRENT_NOW,			//0x0C
        POWER_SUPPLY_PROP_ENERGY_NOW,			//0x1B
        POWER_SUPPLY_PROP_CAPACITY,				//0x1D
        POWER_SUPPLY_PROP_CAPACITY_LEVEL,		//0x1E
        POWER_SUPPLY_PROP_TEMP,					//0x1F
        POWER_SUPPLY_PROP_MANUFACTURER,
        POWER_SUPPLY_PROP_MODEL_NAME,
        POWER_SUPPLY_PROP_SERIAL_NUMBER,
        POWER_SUPPLY_PROP_CURRENT_AVG,
#ifdef CONFIG_ASUS_BATTERY
        POWER_SUPPLY_PROP_BATTERY_ID,
        POWER_SUPPLY_PROP_FIRMWARE_VERSION,
        #ifdef CONFIG_ASUS_BATTERY_BQ27520
        POWER_SUPPLY_PROP_CHEMICAL_ID,
        POWER_SUPPLY_PROP_FW_CFG_VER,
        #endif
#endif
        //	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,	//0x21
        //	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,	//0x22
        //	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,		//0x23
};

static enum power_supply_property asus_power_properties[] = {
        POWER_SUPPLY_PROP_PRESENT,	//0x03
        POWER_SUPPLY_PROP_ONLINE,	//0x04
};


static int asus_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val);

static int asus_power_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val);

static struct power_supply asus_power_supplies[] = {
        {
                .name = "battery",
                .type = POWER_SUPPLY_TYPE_BATTERY,
                .properties = asus_battery_props,
                .num_properties = ARRAY_SIZE(asus_battery_props),
                .get_property = asus_battery_get_property,
        },
        {
                .name = "ac",
                .type = POWER_SUPPLY_TYPE_MAINS,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = asus_power_properties,
                .num_properties = ARRAY_SIZE(asus_power_properties),
                .get_property = asus_power_get_property,
        },
        {
                .name = "usb",
                .type = POWER_SUPPLY_TYPE_USB,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = asus_power_properties,
                .num_properties = ARRAY_SIZE(asus_power_properties),
                .get_property = asus_power_get_property,
        },
        {
                .name = "wireless",
                .type = POWER_SUPPLY_TYPE_WIRELESS,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = asus_power_properties,
                .num_properties = ARRAY_SIZE(asus_power_properties),
                .get_property = asus_power_get_property,
        },
};

int asus_battery_low_event()
{
        drv_status_t drv_status;

        mutex_lock(&batt_info_mutex);
        drv_status = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        drv_status = DRV_BATTERY_LOW;

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = drv_status;
        mutex_unlock(&batt_info_mutex);

        power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);
        power_supply_changed(&asus_power_supplies[CHARGER_USB]);
        power_supply_changed(&asus_power_supplies[CHARGER_AC]);
        power_supply_changed(&asus_power_supplies[CHARGER_WIRELESS]);

        return 0;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int asus_battery_update_temp_no_mutex(void)
{
        int temperature = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_temp)
                temperature = tmp_batt_info.tbl->read_temp();

        return temperature;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int asus_battery_update_voltage_no_mutex(void)
{
        int volt = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_volt)
                volt = tmp_batt_info.tbl->read_volt();

        return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well(-65536 ~ 65535).
 * So, the value get from device need add a base(0x10000) to be a positive number if no error.
 * Or < 0 if something fails.
 */
static int asus_battery_update_current_no_mutex(void)
{
        int curr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_current)
                curr = tmp_batt_info.tbl->read_current();

        return curr;
}

static int asus_battery_update_available_energy_no_mutex(void)
{
        int mWhr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.gauge_version != IC_VERSION_G3) {
            BAT_DBG("Read available energy error due to not G3.\n");
            return mWhr;
        }

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_av_energy)
                mWhr = tmp_batt_info.tbl->read_av_energy();

        return mWhr;
}

static int asus_battery_update_percentage_no_mutex(void)
{
        int percentage = -EINVAL;
        drv_status_t drv_status;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        drv_status = batt_info.drv_status;

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_percentage) {
                percentage = tmp_batt_info.tbl->read_percentage();

                if (percentage == ERROR_CODE_I2C_FAILURE)
                    return -EINVAL;

                if (percentage >= 0 && percentage <= 3) {
                        percentage = 0;
                        drv_status = DRV_SHUTDOWN;
                }

                batt_info.drv_status = drv_status;
        }

        return percentage; //% return adjust percentage, and error code ( < 0)
}

//range:   range_min <= X < range_max
struct lvl_entry {
        char name[30];
        int range_min;
        int range_max;
        int ret_val;
};

#define MAX_DONT_CARE    1000000
#define MIN_DONT_CARE    -999999
struct lvl_entry lvl_tbl[] = {
        { "FULL",      100,    MAX_DONT_CARE,   POWER_SUPPLY_CAPACITY_LEVEL_FULL},
        { "HIGH",       80,              100,   POWER_SUPPLY_CAPACITY_LEVEL_HIGH},
        { "NORMAL",     20,               80,   POWER_SUPPLY_CAPACITY_LEVEL_NORMAL},
        { "LOW",         5,               20,   POWER_SUPPLY_CAPACITY_LEVEL_LOW},
        { "CRITICAL",    0,                5,   POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL},
        { "UNKNOWN", MIN_DONT_CARE,  MAX_DONT_CARE,  POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN},
};

static int asus_battery_update_capacity_level(int percentage)
{
        int i=0;

        for (i=0; i<sizeof(lvl_tbl)/sizeof(struct lvl_entry); i++) {
                if (lvl_tbl[i].range_max != MAX_DONT_CARE &&
                        percentage >= lvl_tbl[i].range_max)
                        continue;
                if (lvl_tbl[i].range_min != MIN_DONT_CARE &&
                        lvl_tbl[i].range_min > percentage)
                        continue;
                //printk(KERN_WARNING "<BATT> battery level = %s\n", lvl_tbl[i].name);
                return lvl_tbl[i].ret_val;
        }

        return -EINVAL;
}

static int asus_battery_update_status_no_mutex(int percentage)
{
        int status;
        int temperature;
        int voltage_change_temperature;
        int flags;
        struct battery_info_reply tmp_batt_info;
        u32 cable_status;

        tmp_batt_info = batt_info;
        cable_status = tmp_batt_info.cable_status;

        if (cable_status == USB_ADAPTER || cable_status == USB_PC || batt_info.cable_status == WIRELESS_ADAPTER) {
                status = POWER_SUPPLY_STATUS_CHARGING;

#ifdef ASUS_ENG_BUILD
                /* ME371MG, ME302C eng mode : stop charging when battery percentage is over 60% */
                if (percentage >= 60 && tmp_batt_info.eng_charging_limit) {
                        smb347_charging_toggle(false);
                        status = POWER_SUPPLY_STATUS_DISCHARGING;
                        goto final;
                }
#endif

                /* ME371MG, ME302C thermal control : stop charging in some thermal conditions */
                if (tmp_batt_info.thermal_charging_limit) {
                        smb347_charging_toggle(false);
                        status = POWER_SUPPLY_STATUS_DISCHARGING;
                        goto final;
                }

                /* ME371MG, ME302C limit to protect battery from damaged when battery temperature is too low */
                if (DISABLE_CHARGING_AT_LOW_TEMP) {
                        temperature = asus_battery_update_temp_no_mutex();
                        if (temperature != ERROR_CODE_I2C_FAILURE) {
                                if (temperature <= 0) {
                                        smb347_charging_toggle(false);
                                        status = POWER_SUPPLY_STATUS_DISCHARGING;
                                        goto final;
                                }
                        }
                        else
                                status = POWER_SUPPLY_STATUS_UNKNOWN;
                }

                if (status == POWER_SUPPLY_STATUS_CHARGING) {
			/* ME581B limit to protect battery from damaged when battery temperature is too low or High*/
			temperature = asus_battery_update_temp_no_mutex();
                        if (temperature != ERROR_CODE_I2C_FAILURE) {
                                if (batt_info.cable_status == WIRELESS_ADAPTER) {
                                   voltage_change_temperature = 400;
                                } else {
                                   voltage_change_temperature = 500;
                                }

                                if (temperature <= 0) {
					smb347_set_voltage(false);
                                        smb347_charging_toggle(false);
                                        status = POWER_SUPPLY_STATUS_DISCHARGING;
                                        goto final;
                                } else if (temperature > 0 && temperature <= voltage_change_temperature) {
                                        smb347_set_voltage(false);
                                } else if (temperature > voltage_change_temperature && temperature <= 550) {
                                        smb347_set_voltage(true);
                                } else {
                                        smb347_set_voltage(true);
                                        smb347_charging_toggle(false);
                                        status = POWER_SUPPLY_STATUS_DISCHARGING;
                                        goto final;
                                }
                        }
                        else
                                status = POWER_SUPPLY_STATUS_UNKNOWN;
                    if (cable_status == USB_ADAPTER && smb347_get_aicl_result() <= 0x01) {// AICL result < 500mA
                        BAT_DBG("AICL get result, AICL results < 500mA \n");
                        smb347_AC_in_current(1800);
                    }
                }

                if (percentage >= 0 && percentage <= 100) {
                    flags = bq27520_asus_battery_dev_read_flags();
                    BAT_DBG("battery flags= 0x%04X \n", flags);
                    switch (percentage) {
                       case 100:
                          status = POWER_SUPPLY_STATUS_FULL;
                       case 99:
                          if (flags & BIT(9)) {
                             //check if Full-charged is detected
                             BAT_DBG("[[ Full-charged is detected ]] \n");
                             smb347_charging_toggle(false);
                             status = POWER_SUPPLY_STATUS_FULL;
                          } else {
                             smb347_charging_toggle(true);
                          }
                          break;
                       default:
                          smb347_charging_toggle(true);
                          break;

                   }
                } else {
                    BAT_DBG_E("Incorrect percentage !!!!!\n");
                }
        } else {
                if (percentage == 100)
                        status = POWER_SUPPLY_STATUS_FULL;
                else
                        status = POWER_SUPPLY_STATUS_DISCHARGING;
        }

final:
        if (status == POWER_SUPPLY_STATUS_FULL)
                printk(KERN_WARNING "<BATT> battery status = POWER_SUPPLY_STATUS_FULL\n");
        else if (status == POWER_SUPPLY_STATUS_CHARGING)
                printk(KERN_WARNING "<BATT> battery status = POWER_SUPPLY_STATUS_CHARGING\n");
        else if (status == POWER_SUPPLY_STATUS_DISCHARGING)
                printk(KERN_WARNING "<BATT> battery status = POWER_SUPPLY_STATUS_DISCHARGING\n");
        else if (status == POWER_SUPPLY_STATUS_NOT_CHARGING)
                printk(KERN_WARNING "<BATT> battery status = POWER_SUPPLY_STATUS_NOT_CHARGING\n");
        else if (status == POWER_SUPPLY_STATUS_UNKNOWN)
                printk(KERN_WARNING "<BATT> battery status = POWER_SUPPLY_STATUS_UNKNOWN\n");

        return status;
}

static void asus_battery_get_info_no_mutex(void)
{
        struct battery_info_reply tmp_batt_info;
        static int pre_batt_percentage = -1;
        static int pre_cable_status = -1;
        int tmp=0;

        tmp_batt_info = batt_info;

        tmp = asus_battery_update_voltage_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_volt = tmp;

        tmp = asus_battery_update_current_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_current = tmp - 0x10000;

        tmp = asus_battery_update_percentage_no_mutex();
        if (tmp >= 0) {
                if (pre_cable_status < 0)
                        pre_cable_status = tmp_batt_info.cable_status;
                if (pre_batt_percentage<0) {
                        tmp_batt_info.percentage = tmp;
                        pre_batt_percentage = tmp;
                        BAT_DBG(" Init the battery percentage values = %d\n", pre_batt_percentage);
                }
                else if ((tmp_batt_info.cable_status == NO_CABLE) && (pre_cable_status == NO_CABLE) && ( pre_batt_percentage < tmp)) {
                        tmp_batt_info.percentage = pre_batt_percentage;
                        printk(KERN_WARNING "<BATT> keep pre battery percentage = (pre:%d, now:%d)\n", pre_batt_percentage, tmp);
                }
                else {
                        /* FIX: suddently battery percentage drop while
                           it is nearly battery low (TT259005).
                           We adopt the Dichotomy method to report the percentage smoothly
                         */
                        if (tmp < 4 && pre_batt_percentage > 5) {
                            printk(KERN_WARNING "<BATT> modify dropping percentage = (now:%d, mod:%d)\n", tmp, (tmp+pre_batt_percentage)/2);
                            tmp = (tmp + pre_batt_percentage) / 2;
                        }
                        tmp_batt_info.percentage = tmp;
                        pre_batt_percentage = tmp;
                }
                pre_cable_status = tmp_batt_info.cable_status;
        }

        tmp = asus_battery_update_capacity_level(tmp_batt_info.percentage);
        if (tmp >= 0) tmp_batt_info.level = tmp;

        tmp = asus_battery_update_status_no_mutex(tmp_batt_info.percentage);
        if (tmp >= 0) tmp_batt_info.status = tmp;

        tmp = asus_battery_update_temp_no_mutex();
        if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL) tmp_batt_info.batt_temp = tmp;

        // support for PSChange app
        battery_remaining_capacity = bq27520_asus_battery_dev_read_remaining_capacity();
        if (battery_remaining_capacity < 0) battery_remaining_capacity = 0;
        battery_current            = tmp_batt_info.batt_current;

        if (batt_info.gauge_version == IC_VERSION_G3) {
            tmp = asus_battery_update_available_energy_no_mutex();
            if (tmp >= 0) tmp_batt_info.batt_energy = tmp;

            printk(KERN_WARNING "<BATT> battery info (P:%d %%, V:%d mV, C:%d mA, T:%d C, E:%d mWh)\n",
                tmp_batt_info.percentage,
                tmp_batt_info.batt_volt,
                tmp_batt_info.batt_current,
                tmp_batt_info.batt_temp,
                tmp_batt_info.batt_energy);
        } else {
            printk(KERN_WARNING "<BATT> battery info (P:%d %%, V:%d mV, C:%d mA, T:%d C)\n",
                tmp_batt_info.percentage,
                tmp_batt_info.batt_volt,
                tmp_batt_info.batt_current,
                tmp_batt_info.batt_temp);
        }


        batt_info = tmp_batt_info;
}

static void asus_polling_data(struct work_struct *dat)
{
        u32 polling_time = 60*HZ;
        struct battery_info_reply tmp_batt_info;

        asus_update_all();
        msleep(10);

        /* ME371MG, ME302C battery polling algorithm */

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.batt_temp >= 630) {
            BAT_DBG("Critical condition!! -> temperature \n");
            polling_time = BATTERY_CRITICAL_POLL_TIME;
        }
        else if (tmp_batt_info.batt_temp >= 500) {
            BAT_DBG("Nearly critical condition!! -> temperature \n");
            polling_time = 10*HZ;
        }

        if (tmp_batt_info.percentage >= 50 && tmp_batt_info.percentage <= 100)
            polling_time = (60*HZ) < polling_time ? (60*HZ) : polling_time;
        else if (tmp_batt_info.percentage >= 20 && tmp_batt_info.percentage <= 49)
            polling_time = (30*HZ) < polling_time ? (30*HZ) : polling_time;
        else if (tmp_batt_info.percentage >= 5 && tmp_batt_info.percentage <= 19)
            polling_time = (10*HZ) < polling_time ? (10*HZ) : polling_time;
        else if (tmp_batt_info.percentage >= 0 && tmp_batt_info.percentage <= 4)
            polling_time = (5*HZ) < polling_time ? (5*HZ) : polling_time;
        else {
            BAT_DBG_E("*** Battery percentage is out of the legal range (percentage < 0 or percentage > 100) ***\n");
            polling_time = 5*HZ;
        }
        BAT_DBG(" battery polling interval = %d secs.\n", polling_time/HZ);

        queue_delayed_work(battery_work_queue, &battery_poll_data_work, polling_time);
}

void asus_update_all(void)
{
        mutex_lock(&batt_info_mutex);

        asus_battery_get_info_no_mutex();
        // if the power source changes, all power supplies may change state
        power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);
        power_supply_changed(&asus_power_supplies[CHARGER_AC]);
        power_supply_changed(&asus_power_supplies[CHARGER_USB]);
        power_supply_changed(&asus_power_supplies[CHARGER_WIRELESS]);

        mutex_unlock(&batt_info_mutex);
}

static void USB_cable_status_worker(struct work_struct *dat)
{
        asus_update_all();
}

void asus_queue_update_all(void)
{
        queue_delayed_work(battery_work_queue, &detect_cable_work, 0.1*HZ);
}

void usb_to_battery_callback(u32 usb_cable_state)
{
        drv_status_t drv_sts;

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        mutex_lock(&batt_info_mutex);
        batt_info.cable_status = usb_cable_state;
        mutex_unlock(&batt_info_mutex);

        /*ME581B
            1. set Battery OV does not end charging cycle.
            2. set fast charge current = 2A set pre-charge current =200mA, set Tracking Voltage = VBATT + 100mV //set hot soft limiit voltage=VFLT-240mV
            3. set VFLT=4.2V
        */
        if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC || batt_info.cable_status == WIRELESS_ADAPTER) {
            smb347_set_battery_OV();
            smb347_set_fast_charge();
            smb347_set_voltage(false);
            /*ME560CG SOC control JEITA*/
            smb347_control_JEITA(true);
            if (batt_info.cable_status == USB_ADAPTER) {
                /*ME560CG  set I_USB_IN=1800mA*/
                smb347_AC_in_current(1800);
            }
        }

        if (drv_sts != DRV_REGISTER_OK) {
                BAT_DBG_E("Battery module not ready\n");
                return;
        }

        mutex_lock(&batt_info_mutex);

        /* prevent system from entering s3 in COS while AC charger is connected */
        if (entry_mode == 4) {
            if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == WIRELESS_ADAPTER) {
                if (!wake_lock_active(&wakelock)) {
                    BAT_DBG(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                    wake_lock(&wakelock);
                }
            }
            else if (batt_info.cable_status == NO_CABLE) {
                if (wake_lock_active(&wakelock)) {
                    BAT_DBG(" %s: asus_battery_power_wakelock -> wake unlock\n", __func__);
                    wake_lock_timeout(&wakelock_t, 3*HZ);  // timeout value as same as the <charger.exe>\asus_global.h #define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
                    wake_unlock(&wakelock);
                } else { // for PC case
                    wake_lock_timeout(&wakelock_t, 3*HZ);
                }
            }
        }

        mutex_unlock(&batt_info_mutex);

        asus_update_all();
        //queue_delayed_work(battery_work_queue, &detect_cable_work, 0.1*HZ);
}
EXPORT_SYMBOL(usb_to_battery_callback);

int receive_USBcable_type(void)
{
        u32 cable_status;
        struct battery_info_reply tmp_batt_info;

        BAT_DBG("%s\n", __func__);

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        cable_status = tmp_batt_info.cable_status ;

        return cable_status;
}
EXPORT_SYMBOL(receive_USBcable_type);

static int asus_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret = 0;
        struct battery_info_reply tmp_batt_info;

        //BAT_DBG("%s 0x%04X\n", __func__, psp);

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
                val->intval = tmp_batt_info.status;
                break;
        case POWER_SUPPLY_PROP_HEALTH:
                val->intval = POWER_SUPPLY_HEALTH_GOOD;
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                val->intval = tmp_batt_info.present;
                break;
        case POWER_SUPPLY_PROP_TECHNOLOGY:
                val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
                break;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
                val->intval = tmp_batt_info.batt_volt;
                //BAT_DBG("VOLTAGE_NOW = %d\n", tmp_batt_info.batt_volt);
                /* ME371MG, ME302C: change the voltage unit from Milli Voltage (mV) to Micro Voltage (uV) */
                val->intval *= 1000;
                break;
        case POWER_SUPPLY_PROP_CURRENT_NOW:
        case POWER_SUPPLY_PROP_CURRENT_AVG:
                val->intval = tmp_batt_info.batt_current;
                /* ME371MG, ME302C: change the current unit from Milli Ampere (mA) to Micro Ampere (uA) */
                val->intval *= 1000;
                //BAT_DBG("CURRENT_NOW = %d\n", tmp_batt_info.batt_current);
                break;
        case POWER_SUPPLY_PROP_ENERGY_NOW:
                val->intval = tmp_batt_info.batt_energy;
                //BAT_DBG("ENERGY_NOW = %d\n", tmp_batt_info.batt_energy);
                break;
        case POWER_SUPPLY_PROP_CAPACITY:
                val->intval = tmp_batt_info.percentage;
                //BAT_DBG("CAPACITY = %d\n", tmp_batt_info.percentage);
                break;
        case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
                val->intval = tmp_batt_info.level;
                break;
        case POWER_SUPPLY_PROP_MANUFACTURER:
        case POWER_SUPPLY_PROP_BATTERY_ID:
                val->strval = tmp_batt_info.manufacturer;
                break;
        case POWER_SUPPLY_PROP_MODEL_NAME:
                val->strval = tmp_batt_info.model;
                break;
        case POWER_SUPPLY_PROP_SERIAL_NUMBER:
        case POWER_SUPPLY_PROP_FIRMWARE_VERSION:
                val->strval = tmp_batt_info.serial_number;
                break;
#ifdef CONFIG_ASUS_BATTERY_BQ27520
        case POWER_SUPPLY_PROP_CHEMICAL_ID:
                val->strval = tmp_batt_info.chemical_id;
                break;
        case POWER_SUPPLY_PROP_FW_CFG_VER:
                val->strval = tmp_batt_info.fw_cfg_version;
                break;
#endif
        case POWER_SUPPLY_PROP_TEMP:
                val->intval = tmp_batt_info.batt_temp;
                //BAT_DBG("TEMP = %d\n", tmp_batt_info.batt_temp);
                break;
                        //	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
                        //		ret = bq27541_battery_time(BQ27541_REG_TTE, val);
                        //		break;
                        //	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
                        //		ret = bq27541_battery_time(BQ27541_REG_TTECP, val);
                        //		break;
                        //	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
                        //		ret = bq27541_battery_time(BQ27541_REG_TTF, val);
                        //		break;
        default:
                return -EINVAL;
        }

        return ret;
}

static int asus_power_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret;
        struct battery_info_reply tmp_batt_info;
        u32 cable_status;
        u32 drv_status;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        cable_status = tmp_batt_info.cable_status;
        drv_status = tmp_batt_info.drv_status;

        ret = 0;
        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                if (drv_status == DRV_SHUTDOWN) {
                        val->intval = 0;

                } else if (drv_status == DRV_BATTERY_LOW) {
                        val->intval = 0;

                } else if (psy->type == POWER_SUPPLY_TYPE_USB) {
                        val->intval = (cable_status == USB_PC) ? 1 : 0;

                } else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
                        val->intval = (cable_status == USB_ADAPTER) ? 1 : 0;

                } else if (psy->type == POWER_SUPPLY_TYPE_WIRELESS) {
                        val->intval = (cable_status == WIRELESS_ADAPTER) ? 1 : 0;

                } else {
                        ret = -EINVAL;
                }
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS || psy->type == POWER_SUPPLY_TYPE_WIRELESS) {
                    /* for ATD test to acquire the status about charger ic */
                    if (!smb347_has_charger_error() || smb347_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING)
                        val->intval = 1;
                    else
                        val->intval = 0;
                } else {
                    ret = -EINVAL;
                }
                break;
        default:
                ret = -EINVAL;
        }

        return ret;
}


int asus_register_power_supply(struct device *dev, struct dev_func *tbl)
{
        int ret;
        int test_flag=0;
        drv_status_t drv_status;
#ifdef CONFIG_ME371MG_BATTERY
        int voltage=0, percentage=0;
        u32 cable_status;
#endif

        BAT_DBG_E("%s\n", __func__);

        mutex_lock(&batt_info_mutex);
        drv_status = batt_info.drv_status;
        test_flag = batt_info.test_flag;
        mutex_unlock(&batt_info_mutex);

        if (!dev) {
                BAT_DBG_E("%s, device pointer is NULL.\n", __func__);
                return -EINVAL;
        } else if (!tbl) {
                BAT_DBG_E("%s, dev_func pointer is NULL.\n", __func__);
                return -EINVAL;
        }else if (drv_status != DRV_INIT_OK) {
                BAT_DBG_E("%s, asus_battery not init ok.\n", __func__);
                return -EINVAL;
        } else if (test_flag & TEST_INFO_NO_REG_POWER) {
                BAT_DBG_E("%s, Not register power class.\n", __func__);
                return 0;
        }

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_REGISTER;
        batt_info.tbl = tbl;
        mutex_unlock(&batt_info_mutex);

#ifdef CONFIG_ME371MG_BATTERY
        /* ME371MG: check if device connect to power supply without
         *          battery insertions in userdebug or eng build.
         */
        if (batt_info.tbl && (build_version == 1 || build_version == 2)) {
            percentage = asus_battery_update_percentage_no_mutex();
            voltage = asus_battery_update_voltage_no_mutex();
            cable_status = batt_info.cable_status;
            if (voltage > 3800 && percentage == 0 && cable_status == NO_CABLE) {
                BAT_DBG_E("%s:Not register power class. Reason: batt votage>3.8V, batt perc=0% without USB\n", __func__);
                return -EINVAL;
            }
        }
#endif

        //register to power supply driver
        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_BATTERY]);
        if (ret) { BAT_DBG_E("Fail to register battery\n"); goto batt_err_reg_fail_battery; }

        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_USB]);
        if (ret) { BAT_DBG_E("Fail to register USB\n"); goto batt_err_reg_fail_usb; }

        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_AC]);
        if (ret) { BAT_DBG_E("Fail to register AC\n"); goto batt_err_reg_fail_ac; }

        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_WIRELESS]);
        if (ret) { BAT_DBG_E("Fail to register wireless\n"); goto batt_err_reg_fail_wireless; }

        //first update current information
        mutex_lock(&batt_info_mutex);
        asus_battery_get_info_no_mutex();
        batt_info.drv_status = DRV_REGISTER_OK;
        mutex_unlock(&batt_info_mutex);

        /* init wake lock in COS */
        if (entry_mode == 4) {
            BAT_DBG(" %s: wake lock init: asus_battery_power_wakelock\n", __func__);
            wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock");
            wake_lock_init(&wakelock_t, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock_timeout");
        }

        /* prevent system from entering s3 in COS while AC charger is connected */
        if (entry_mode == 4) {
            if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == WIRELESS_ADAPTER) {
                if (!wake_lock_active(&wakelock)) {
                    BAT_DBG(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                    wake_lock(&wakelock);
                }
            }
        }

        //start working
        queue_delayed_work(battery_work_queue, &battery_poll_data_work,
                        BATTERY_DEFAULT_POLL_TIME);

        BAT_DBG("%s register OK.\n", __func__);
        return 0;

batt_err_reg_fail_ac:
        power_supply_unregister(&asus_power_supplies[CHARGER_AC]);
batt_err_reg_fail_usb:
        power_supply_unregister(&asus_power_supplies[CHARGER_USB]);
batt_err_reg_fail_battery:
        power_supply_unregister(&asus_power_supplies[CHARGER_BATTERY]);
batt_err_reg_fail_wireless:
        power_supply_unregister(&asus_power_supplies[CHARGER_WIRELESS]);

        return ret;
}
EXPORT_SYMBOL(asus_register_power_supply);

int asus_battery_init(
        u32 polling_time,
        u32 critical_polling_time,
        u32 test_flag
)
{
        int ret=0;
        drv_status_t drv_sts;

        BAT_DBG("%s, %d, %d, 0x%08X\n", __func__, polling_time, critical_polling_time, test_flag);

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        if (drv_sts != DRV_NOT_READY) {
                //other battery device registered.
                BAT_DBG_E("Error!! Already registered by other driver\n");
                ret = -EINVAL;
                if (test_flag & TEST_INFO_NO_REG_POWER) {
                        ret = 0;
                }

                goto already_init;

        }

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_INIT;
        batt_info.polling_time = polling_time > (5*HZ) ? polling_time : BATTERY_DEFAULT_POLL_TIME;
        batt_info.critical_polling_time = critical_polling_time > (3*HZ) ? critical_polling_time : BATTERY_CRITICAL_POLL_TIME;
        batt_info.critical_polling_time = BATTERY_CRITICAL_POLL_TIME;
        batt_info.percentage = 50;
/* chris20121116: do not initialize it here */
#if 0
        batt_info.cable_status = NO_CABLE;
#endif
        batt_info.batt_temp = 250;
        batt_info.present = 1;
        batt_info.test_flag = test_flag;
        if (test_flag)
                BAT_DBG("test_flag: 0x%08X\n", test_flag);

        /* Allocate ADC Channels */
/*
	batt_info.gpadc_handle =
		intel_mid_gpadc_alloc(CLT_BATT_NUM_GPADC_SENSORS,
				  CLT_GPADC_BPRESIST_CHNUM | CH_NEED_VCALIB |
				  CH_NEED_VREF);
        if (batt_info.gpadc_handle == NULL) {
              BAT_DBG_E("ADC allocation failed: Check if ADC driver came up \n");
        }

        ret = intel_mid_gpadc_sample(batt_info.gpadc_handle,
				CLT_GPADC_BPTHERM_SAMPLE_COUNT,
				&batt_info.gpadc_resistance_val);
	if (ret) {
		BAT_DBG_E("adc driver api returned error(%d)\n", ret);
	}
        BAT_DBG("battery resistance = %d\n", batt_info.gpadc_resistance_val);
*/
        mutex_unlock(&batt_info_mutex);

        if (test_flag & TEST_INFO_NO_REG_POWER) {
                BAT_DBG_E("Not allow initiallize  power class. Skip ...\n");
                ret = 0;

                mutex_lock(&batt_info_mutex);
                batt_info.drv_status = DRV_INIT_OK;
                mutex_unlock(&batt_info_mutex);

                goto already_init;
        }

#if CONFIG_PROC_FS
        ret = asus_battery_register_proc_fs();
        if (ret) {
                BAT_DBG_E("Unable to create proc asus_battery_register_proc_fs\n");
                goto proc_fail;
        }

        ret = bq27520_register_upilogger_proc_fs();
        if (ret) {
                BAT_DBG_E("Unable to create bq27520_register_upilogger_proc_fs\n");
                goto proc_fail;
        }
#endif

        battery_work_queue = create_singlethread_workqueue("asus_battery");
        if(battery_work_queue == NULL)
        {
                BAT_DBG_E("Create battery thread fail");
                ret = -ENOMEM;
                goto error_workq;
        }
        INIT_DELAYED_WORK_DEFERRABLE(&battery_poll_data_work, asus_polling_data);
        INIT_DELAYED_WORK_DEFERRABLE(&detect_cable_work, USB_cable_status_worker);

#ifdef ASUS_ENG_BUILD
#if CONFIG_PROC_FS
        ret = init_asus_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create proc init_asus_charging_toggle\n");
                goto proc_fail;
        }
        ret = init_asus_for_gague_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create init_asus_for_gague_charging_toggl\n");
                goto proc_fail;
        }
//-->Carlisle add for ATD querying gauge IC status
        ret = check_gaugeIC_status();
        if (ret) {
                BAT_DBG_E("Unable to create proc check_gaugeIC_status\n");
                goto proc_fail;
        }
//<--Carlisle add for ATD querying gauge IC status end
//-->Carlisle add for ATD querying charge status
        ret = check_charge_status();
        if (ret) {
                BAT_DBG_E("Unable to create proc check_charge_status\n");
                goto proc_fail;
        }
//<--Carlisle add for ATD querying charge status end
//-->Carlisle add for ATD querying gauge IC fw flash identify
		ret = check_gauge_fw_status();
		if (ret) {
                BAT_DBG_E("Unable to create proc check_gauge_fw_status\n");
                goto proc_fail;
        }
//<--Carlisle add for ATD querying gauge IC fw flash identify end
#endif
#endif

#if CONFIG_PROC_FS
        //+++Sewell+++charging_toggle interface
        ret = init_thermal_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create proc init_thermal_charging_toggle\n");
                goto proc_fail;
        }
        //---Sewell---
#endif

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_INIT_OK;
        mutex_unlock(&batt_info_mutex);

        BAT_DBG("%s: success\n", __func__);
        pmic_dump_registers(MSIC_CHRG_REG_DUMP_INT | MSIC_CHRG_REG_DUMP_BOOT
             | MSIC_CHRG_REG_DUMP_EVENT | MSIC_CHRG_REG_DUMP_OTHERS);
        return 0;

error_workq:
#if CONFIG_PROC_FS
proc_fail:
#endif
already_init:
        return ret;
}
EXPORT_SYMBOL(asus_battery_init);

void asus_battery_exit(void)
{
        drv_status_t drv_sts;

        BAT_DBG("Driver unload\n");

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        if (drv_sts == DRV_REGISTER_OK) {
                power_supply_unregister(&asus_power_supplies[CHARGER_BATTERY]);
                power_supply_unregister(&asus_power_supplies[CHARGER_USB]);
                power_supply_unregister(&asus_power_supplies[CHARGER_AC]);
                power_supply_unregister(&asus_power_supplies[CHARGER_WIRELESS]);
                if (entry_mode == 4)
                    wake_lock_destroy(&wakelock);
        }
        BAT_DBG("Driver unload OK\n");
}

static int __init asus_battery_fake_init(void)
{
        return 0;
}
late_initcall(asus_battery_fake_init);

static void __exit asus_battery_fake_exit(void)
{
        /* SHOULD NOT REACHE HERE */
        BAT_DBG("%s exit\n", __func__);
}
module_exit(asus_battery_fake_exit);

MODULE_AUTHOR("chris1_chang@asus.com");
MODULE_DESCRIPTION("battery driver");
MODULE_LICENSE("GPL");
