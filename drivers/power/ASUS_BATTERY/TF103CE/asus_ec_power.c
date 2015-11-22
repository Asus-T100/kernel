#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/usb/penwell_otg.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/HWVersion.h>
#include <linux/switch.h>

#include "asus_ec_power.h"
#include "smb345_external_include.h"
#include "util.h"
#include <linux/microp_notify.h>
#include <linux/microp_api.h>
#include <linux/microp.h>

#define DRIVER_NAME "asus_ec_power"
#define EC_FAIL_MAX_COUNT               3
#define FIRST_RUN_TIME                  3
#define HIGH_POLLING_TIME               60
#define NORMAL_POLLING_TIME		30
#define LOW_POLLING_TIME		10
#define CRITICAL_POLLING_TIME   	5
#define READ_FAIL_POLLING_TIME		60
#define RETRY_COUNT 3
static int HW_ID;
extern int Read_HW_ID(void);
extern void request_phone_battery_update();
extern int entry_mode;

extern int register_microp_notifier(struct notifier_block *nb);
extern int unregister_microp_notifier(struct notifier_block *nb);

static int asus_battery_driver_ready = 0;
extern int uP_i2c_write_reg(int cmd, void *data);
static bool DecideIfPadDockHaveExtChgAC(void);
static void Init_Microp_Vbus(bool on);
static void DoBalance();
static void DoForcePowerBankMode(void);
static int balance_judge();
static inline int get_battery_rsoc(int *rsoc);
static int _AX_MicroP_IsP01Connected_();

char *balance_mode_str[] = {
    "PHONE_PREFERRED",
    "BALANCE_AUTO",
    "POWER_PACK",
};

static inline struct power_supply *get_psy_battery(void)
{
    struct class_dev_iter iter;
    struct device *dev;
    static struct power_supply *pst;

    class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
    while ((dev = class_dev_iter_next(&iter))) {
        pst = (struct power_supply *)dev_get_drvdata(dev);
        if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
            class_dev_iter_exit(&iter);
            return pst;
        }
    }
    class_dev_iter_exit(&iter);

    return NULL;
}

static inline int get_battery_rsoc(int *rsoc)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}

struct asus_pad_device_info {
    struct delayed_work ec_report_work;
    struct workqueue_struct *ec_report_work_q;
    wait_queue_head_t  charger_status_event;
    wait_queue_head_t  gaugefw_status_event;
    bool low_battery;

    //return to system ++
    int pad_ac_status;
    int pre_pad_ac_status;
    int pad_bat_status;
    int pad_bat_temperature;
    int pad_bat_percentage;
    int pad_bat_present;
    int pad_bat_voltage;
    int pad_bat_current;
    //return to system --

    int ec_suspend_status;
    int ec_fail_count;
    int base_fail_count;
    int phone_batt_rsoc;

    //default 1.  0:PowerbankMode, 1:balanceMode, 2:ForcePowerBankMode
    int IsBalanceMode;
    bool proc_lock_balance_mode;
};
static struct asus_pad_device_info *asus_pad_device;
DEFINE_MUTEX(ec_info_lock);

static int _AX_MicroP_IsP01Connected_()
{
    int ret;

    mutex_lock(&ec_info_lock);
    ret = asus_pad_device->pad_bat_present;
    mutex_unlock(&ec_info_lock);

    return ret;
}

static int asus_ec_power_get_battery_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val);
static int asus_ec_power_get_ac_property(struct power_supply *psy,
                                      enum power_supply_property psp,
                                      union power_supply_propval *val);

static enum power_supply_property asus_ec_power_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property asus_ec_power_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_TEMP,
};

static char *asus_ec_power_pad_bat_supplied_to[] = {
    "battery",
};
static char *asus_ec_power_ac_supplied_to[] = {
    "battery",
    "pad_bat",
};

static struct power_supply asus_ec_power_supplies[] = {
    {
        .name = "pad_bat",
        .type = POWER_SUPPLY_TYPE_PAD_BAT,
        .properties = asus_ec_power_props,
        .num_properties = ARRAY_SIZE(asus_ec_power_props),
        .supplied_to = asus_ec_power_pad_bat_supplied_to,
        .num_supplicants = ARRAY_SIZE(asus_ec_power_pad_bat_supplied_to),
        .get_property = asus_ec_power_get_battery_property,
    },
    {
        .name = "pad_ac",
        .type = POWER_SUPPLY_TYPE_PAD_AC,
        .supplied_to = asus_ec_power_ac_supplied_to,
        .num_supplicants = ARRAY_SIZE(asus_ec_power_ac_supplied_to),
        .properties = asus_ec_power_ac_props,
        .num_properties = ARRAY_SIZE(asus_ec_power_ac_props),
        .get_property = asus_ec_power_get_ac_property,
    },
};

bool pad_pwr_supply()
{
    bool ret;

    /* return TRUE if gpio for pad insertion detected
       and charger received the valid external power
    */
    ret = !gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N")) &&
          !gpio_get_value(get_gpio_by_name("CHG_INOK#"));
    return ret;
}

#ifndef FE170CG_USER_BUILD
int pad_power_toggle_on_read(char *page, char **start, off_t off,
                    int count, int *eof, void *date)
{
    pr_info("%s:\n", __func__);

    mutex_lock(&ec_info_lock);
    asus_pad_device->proc_lock_balance_mode = true;
    mutex_unlock(&ec_info_lock);

    /* recovery pad power supply - detect Pad present */
    if (!gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N")))
        Init_Microp_Vbus(true);

    return 0;
}
int pad_power_toggle_on_write(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{ return 0; }
int pad_power_toggle_off_read(char *page, char **start, off_t off,
                    int count, int *eof, void *date)
{
    pr_info("%s:\n", __func__);

    mutex_lock(&ec_info_lock);
    asus_pad_device->proc_lock_balance_mode = true;
    mutex_unlock(&ec_info_lock);

    /* cut off pad power supply */
    if (pad_pwr_supply())
        Init_Microp_Vbus(false);

    return 0;
}
int pad_power_toggle_off_write(struct file *file, const char *buffer,
                    unsigned long count, void *data)
{ return 0; }

int init_pad_power_toggle_on(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("recpp", 0666, NULL);
    if (!entry) {
        pr_info("Unable to create pad_power_toggle_on\n");
        return -EINVAL;
    }
    entry->read_proc = pad_power_toggle_on_read;
    entry->write_proc = pad_power_toggle_on_write;

    return 0;
}
int init_pad_power_toggle_off(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("cutpp", 0666, NULL);
    if (!entry) {
        pr_info("Unable to create pad_power_toggle_off\n");
        return -EINVAL;
    }
    entry->read_proc = pad_power_toggle_off_read;
    entry->write_proc = pad_power_toggle_off_write;

    return 0;
}
#else
int init_pad_power_toggle_on(void) { return 0; }
int init_pad_power_toggle_off(void) { return 0; }
#endif

static int get_ec_power_info(void)
{
    int ret, ret1, chrging_sta;
    u8 pad_bat_info[6];
    u8 pad_bat_info_soc[2];
    /* voltage and current from pad battery */
    u16 vol, cur;
    int _cur, tempr;
    int pad_charging_type;

    char pad_chrgr_str[3][8] = {
        "NONE",
        "PAD_AC",
        "PAD_USB",
    };
    char pad_chrging_str[3][5] = {
        "DISC",
        "CHRG",
        "FULL",
    };

    ret = AX_MicroP_getBatteryInfo(pad_bat_info);
    if (ret < 0) {
        pr_err("PBATT> fail to get EC power battery info\n");
    }
    else {
        vol = pad_bat_info[1] << 8 | pad_bat_info[0];
        cur = pad_bat_info[3] << 8 | pad_bat_info[2];
        _cur = (int)(s16)cur;
        tempr = (int)(s8)pad_bat_info[4];

        ret = AX_MicroP_getBatterySoc(pad_bat_info_soc);
        if (ret < 0) {
            pr_err("PBATT> fail to get EC power battery info soc\n");
        }
        else {
            ret = AX_MicroP_get_USBDetectStatus(Batt_P01);
            if (ret < 0) {
                pr_err("PBATT> fail to get EC power charger type\n");
            }
            else {
                ret1 = AX_MicroP_get_ChargingStatus(Batt_P01);
                if (ret < 0) {
                    pr_err("PBATT> fail to get EC power batt charging status\n");
                }
                else {
                    pad_charging_type = ret;

                    /* pad batt charging status */
                    switch(ret1) {
                    case P01_CHARGING_NO:
                        chrging_sta = POWER_SUPPLY_STATUS_DISCHARGING;
                        break;
                    case P01_CHARGING_ONGOING:
                        chrging_sta = POWER_SUPPLY_STATUS_CHARGING;
                        break;
                    case P01_CHARGING_FULL:
                        chrging_sta = POWER_SUPPLY_STATUS_FULL;
                        break;
                    }

                    pr_info("PBATT> P:%u%%(%u%%) V:%umV C:%dmA T:%dC M:%d S:%s(%s) %s %s\n",
                        pad_bat_info_soc[1], pad_bat_info_soc[0],
                        vol,
                        _cur,
                        tempr,
                        pad_bat_info[5],
                        pad_chrgr_str[pad_charging_type],
                        pad_chrging_str[ret1],
                        balance_mode_str[asus_pad_device->IsBalanceMode],
                        pad_pwr_supply() ? "PAD POWER SUPPLYING..." :
                                           "*MISSING PAD POWER!*");

                    mutex_lock(&ec_info_lock);
                    asus_pad_device->pad_bat_status = chrging_sta;
                    asus_pad_device->pad_bat_voltage = vol;
                    asus_pad_device->pad_bat_current = _cur;
                    asus_pad_device->pad_bat_temperature = tempr;
                    asus_pad_device->pad_bat_percentage = pad_bat_info_soc[1];
                    if (asus_pad_device->pad_bat_percentage == 100)
                        asus_pad_device->pad_bat_status = POWER_SUPPLY_STATUS_FULL;
                    else {
                        if (asus_pad_device->pad_ac_status == 1)
                            asus_pad_device->pad_bat_status = POWER_SUPPLY_STATUS_CHARGING;
                        else
                            asus_pad_device->pad_bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
                    }
                    mutex_unlock(&ec_info_lock);
                }
            }
        }
    }
    return ret;
}

static int balance_judge()
{
    int ret, pad_present, pad_ac_present;
    int IsBalanceMode;
    int phone_batt_rsoc;
    int pad_rsoc;
    int ec_suspend_status;

    if (!asus_battery_driver_ready) {
        pr_info("PBATT> %s: driver isn't ready!\n", __func__);
        return ret;
    }

    mutex_lock(&ec_info_lock);
    pad_present = asus_pad_device->pad_bat_present;
    ec_suspend_status = asus_pad_device->ec_suspend_status;
    mutex_unlock(&ec_info_lock);

    if (!pad_present) {
        pr_info("PBATT> %s: pad is not present!\n", __func__);
        return ret;
    }

    if (get_battery_rsoc(&phone_batt_rsoc)) {
        pr_err("PBATT> %s: failed to get BATT rsoc\n", __func__);
        return ret;
    }
    else {
        mutex_lock(&ec_info_lock);
        asus_pad_device->phone_batt_rsoc = phone_batt_rsoc;
        mutex_unlock(&ec_info_lock);
    }

    ret = get_ec_power_info();
    if (ret < 0) {
        pr_err("PBATT> %s: failed to get EC power info\n", __func__);
        return ret;
    }

    mutex_lock(&ec_info_lock);
    pad_present = asus_pad_device->pad_bat_present;
    pad_ac_present = asus_pad_device->pad_ac_status;
    IsBalanceMode = asus_pad_device->IsBalanceMode;
    pad_rsoc = asus_pad_device->pad_bat_percentage;
    mutex_unlock(&ec_info_lock);

    /* balance mode judgement */
    if (pad_ac_present) {
        /* cancel the Limitation when Pad AC insertion */
        Init_Microp_Vbus(true);
        smb345_charging_toggle(BALANCE, true);
    }
    else {
        if (IsBalanceMode == 1) {
            DoBalance();
        }
        else if (IsBalanceMode == 2 || IsBalanceMode == 0) {
            Init_Microp_Vbus(true);
            if (phone_batt_rsoc >= 90) {
                smb345_charging_toggle(BALANCE, false);
            }
            else if (phone_batt_rsoc <= 70) {
                smb345_charging_toggle(BALANCE, true);
            }
#if 0
            if (IsBalanceMode == 2)
                DoForcePowerBankMode();
#endif
        }
        else {
            pr_err("PBATT> %s: what the hell are you?\n", __func__);
        }
    }
}

int ec_power_changed_all()
{
    int ret;

    if (_AX_MicroP_IsP01Connected_() == 0)
        return ret;

    balance_judge();
    ret = get_ec_power_info();
    if (ret < 0) {
        pr_err("PBATT> %s: failed to get EC power info\n", __func__);
        return ret;
    }

    power_supply_changed(&asus_ec_power_supplies[0]);
    power_supply_changed(&asus_ec_power_supplies[1]);

    return ret;
}
EXPORT_SYMBOL(ec_power_changed_all);

static int ec_power_report(struct notifier_block *nb, unsigned long event, void *ptr)
{
    int ret;

    pr_info("------------------- %s -------------------: %d\n",
        __func__, event);

    if (!asus_pad_device) {
        pr_info("PBATT> ec power driver not ready\n");
        return NOTIFY_OK;
    }

    switch (event) {
    case P01_ADD:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_bat_present = 1;
        mutex_unlock(&ec_info_lock);
        ec_power_changed_all();
        break;

    case P01_REMOVE:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_ac_status = 0;
        asus_pad_device->pad_bat_present = 0;
        mutex_unlock(&ec_info_lock);

        power_supply_changed(&asus_ec_power_supplies[0]);
        power_supply_changed(&asus_ec_power_supplies[1]);
        break;

    case P01_BATTERY_POWER_BAD:
        break;
    case P01_BATTERY_TO_CHARGING:
        break;
    case P01_BATTERY_TO_NON_CHARGING:
        break;
    case PAD_PHONEJACK_IN:
        break;
    case PAD_PHONEJACK_OUT:
        break;
    case P01_VOLUP_KEY_PRESSED:
        break;
    case P01_VOLUP_KEY_RELEASED:
        break;
    case P01_VOLDN_KEY_PRESSED:
        break;
    case P01_VOLDN_KEY_RELEASED:
        break;
    case P01_PWR_KEY_PRESSED:
        break;
    case P01_PWR_KEY_RELEASED:
        break;
    case P01_LIGHT_SENSOR:
        break;

    case P01_AC_IN:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_ac_status = 1;
        mutex_unlock(&ec_info_lock);

        ec_power_changed_all();
        request_phone_battery_update();
        break;

    case P01_USB_IN:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_ac_status = 1;
        mutex_unlock(&ec_info_lock);

        ec_power_changed_all();
        request_phone_battery_update();
        break;

    case P01_AC_USB_OUT:
        mutex_lock(&ec_info_lock);
        asus_pad_device->pad_ac_status = 0;
        mutex_unlock(&ec_info_lock);

        ec_power_changed_all();
        request_phone_battery_update();
        break;

    case P01_DEAD:
        break;
    case PAD_UPDATE_FINISH:
        break;
    case PAD_EXTEND_CAP_SENSOR:
        break;
    case PAD_USB_OTG_ENABLE:
        break;
    case P01_LOW_BATTERY:
        ret = get_ec_power_info();
        power_supply_changed(&asus_ec_power_supplies[0]);
        break;
    }

    return NOTIFY_OK;
}
static struct notifier_block ec_power_notifier = {
    .notifier_call = ec_power_report,
};

static int asus_ec_power_get_ac_property(struct power_supply *psy,
                                      enum power_supply_property psp,
                                      union power_supply_propval *val)
{
    int ret = 0;

    mutex_lock(&ec_info_lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_PAD_AC) {
            val->intval = asus_pad_device->pad_ac_status;
        }
        else {
            ret = -EINVAL;
        }
        break;
    default:
        ret = -EINVAL;
        break;
    }

    mutex_unlock(&ec_info_lock);

    return ret;
}

static int asus_ec_power_get_battery_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val)
{
    mutex_lock(&ec_info_lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = asus_pad_device->pad_bat_present;
        break;
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = asus_pad_device->pad_bat_status;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = asus_pad_device->pad_bat_present;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = asus_pad_device->pad_bat_voltage;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = asus_pad_device->pad_bat_current;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = asus_pad_device->pad_bat_percentage;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = asus_pad_device->pad_bat_temperature;
        break;
    default:
        pr_info("PBATT> %s: some properties(%d) deliberately report errors.\n",
            __func__, psp);
        mutex_unlock(&ec_info_lock);
        return -EINVAL;
    }

    mutex_unlock(&ec_info_lock);

    return 0;
}

static int ec_report_worker(struct work_struct work)
{
    //ec_power_changed_all();

    queue_delayed_work(asus_pad_device->ec_report_work_q,
        &asus_pad_device->ec_report_work,
        30*HZ);
}

static int ec_routine_report_init()
{
    int ret;

    pr_info("PBATT> %s\n", __func__);

    INIT_DELAYED_WORK(&asus_pad_device->ec_report_work,
        ec_report_worker);
    asus_pad_device->ec_report_work_q =
        create_singlethread_workqueue("ec_pwr_report_wq");
    if (!asus_pad_device->ec_report_work_q) {
        pr_info("PBATT> fail to create \"ec_pwr_report_wq\"");
        return -ENOMEM;
    }

    queue_delayed_work(asus_pad_device->ec_report_work_q,
        &asus_pad_device->ec_report_work,
        30*HZ);

    return 0;
}

static bool DecideIfPadDockHaveExtChgAC(void)
{
    int isPadAC;

    mutex_lock(&ec_info_lock);
    isPadAC = asus_pad_device->pad_ac_status;
    mutex_unlock(&ec_info_lock);

    pr_info("PBATT> %s: Pad AC: %s", __func__,
        (isPadAC==1) ? "present" : "gone");

    return (isPadAC==1);
}

static void DoBalance()
{
    int phone_batt_rsoc;
    int isSuspend;
    int pad_rsoc;

    mutex_lock(&ec_info_lock);
    pad_rsoc = asus_pad_device->pad_bat_percentage;
    isSuspend = asus_pad_device->ec_suspend_status;
    phone_batt_rsoc = asus_pad_device->phone_batt_rsoc;
    mutex_unlock(&ec_info_lock);

    if (phone_batt_rsoc >= 90 || (phone_batt_rsoc - pad_rsoc) >= 0) {
        pr_err("PBATT> %s: * request CUT OFF pad power supply *\n", __func__);
        Init_Microp_Vbus(false);
        smb345_charging_toggle(BALANCE, false);
    }
    else if (phone_batt_rsoc <= 70 && phone_batt_rsoc <= pad_rsoc * 90 / 100) {
        Init_Microp_Vbus(true);

        if (!isSuspend) {
            if (phone_batt_rsoc >= 20) {
                pr_warn("PBATT> %s: * request RECOVERY power supply"
                    " & STOP charging to battery as ACTIVE *\n",
                    __func__);
                smb345_charging_toggle(BALANCE, false);
            }
            else if (phone_batt_rsoc < 15) {
                smb345_charging_toggle(BALANCE, true);
            }
        }
        else {
            if ((phone_batt_rsoc >= 85
            && phone_batt_rsoc <= 92)
            || phone_batt_rsoc >= pad_rsoc * STOP_PHONE_BATT_CHARGE_RATIO / 100) {
                pr_warn("PBATT> %s: * request RECOVERY power supply"
                    " & STOP charging to battery as SUSPEND *\n",
                    __func__);
                smb345_charging_toggle(BALANCE, false);
            }
            else if (phone_batt_rsoc >= pad_rsoc * RESTART_PHONE_BATT_CHARGE_RATIO / 100) {
                smb345_charging_toggle(BALANCE, true);
            }
        }
    }
}

void Init_Microp_Vbus(bool on)
{
    pr_info("PBATT> %s:", __func__);

    if (on)
        AX_MicroP_set_VBusPower(1);
    else
        AX_MicroP_set_VBusPower(0);
}

void DoForcePowerBankMode(void)
{
    unsigned short off=0xAA;

    pr_info("PBATT> %s:", __func__);
    uP_i2c_write_reg(MICROP_SOFTWARE_OFF, &off);
}

static ssize_t balanceChg_read_proc(char *page, char **start, off_t off,
    int count, int *eof, void *data)
{
    int IsBalanceMode;

    mutex_lock(&ec_info_lock);
    IsBalanceMode = asus_pad_device->IsBalanceMode;
    mutex_unlock(&ec_info_lock);

    pr_info("PBATT> %s: MODE: %d\n", __func__, IsBalanceMode);
    return sprintf(page, "%d\n", IsBalanceMode);
}

static ssize_t balanceChg_write_proc(struct file *filp, const char __user *buff, 
    unsigned long len, void *data)
{
    int val;
    int IsBalanceMode;
    int pad_present;
    int phone_batt_rsoc;

    char messages[256];

    if (len > 256) {
        len = 256;
    }

    if (copy_from_user(messages, buff, len))
        return -EFAULT;
	
    val = (int)simple_strtol(messages, NULL, 10);
    IsBalanceMode = val;

    pr_err("PBATT> %s: ****** MODE: %d ******\n", __func__, val);

    /* valid value verification */
    if (val!=0 && val!=1 && val!=2)
        return len;

    mutex_lock(&ec_info_lock);
    asus_pad_device->IsBalanceMode = IsBalanceMode;
    pad_present = asus_pad_device->pad_bat_present;
    phone_batt_rsoc = asus_pad_device->phone_batt_rsoc;
    mutex_unlock(&ec_info_lock);

    if (!pad_present)
        return len;

    if (IsBalanceMode == 2) {
        Init_Microp_Vbus(true);
        if (phone_batt_rsoc >= 90) {
            smb345_charging_toggle(BALANCE, false);
        }
        else if (phone_batt_rsoc <= 70) {
            smb345_charging_toggle(BALANCE, true);
        }
        DoForcePowerBankMode();
        request_phone_battery_update();
        return len;
    }

    ec_power_changed_all();
    request_phone_battery_update();

    return len;
}

static int create_balanceChg_proc_file(void)
{
    struct proc_dir_entry *balanceChg_proc_file =
        create_proc_entry("driver/balanceChg", 0644, NULL);

    if (!balanceChg_proc_file) {
        pr_info("PBATT> %s: fail to create /proc/driver/balanceChg",
            __func__);
        return -1;
    }
    balanceChg_proc_file->read_proc = balanceChg_read_proc;
    balanceChg_proc_file->write_proc = balanceChg_write_proc;

    return 0;
}

static int __devinit asus_ec_power_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int ret;

    pr_info("++++++++++++++++ %s ++++++++++++++++\n", __func__);

    asus_pad_device = kzalloc(sizeof(struct asus_pad_device_info), GFP_KERNEL);
    if (!asus_pad_device) {
        ret = -ENOMEM;
        goto pad_dev_alloc_fail;
    }
    memset(asus_pad_device, 0, sizeof(*asus_pad_device));

    //Init
    asus_pad_device->pad_bat_present = 0;
    asus_pad_device->pad_bat_status = 3;
    asus_pad_device->pad_bat_percentage = 50;
    asus_pad_device->pad_bat_temperature = 270;
    asus_pad_device->pad_bat_voltage = 4000;
    asus_pad_device->pad_bat_current = 0;
    asus_pad_device->ec_suspend_status = 0;
    asus_pad_device->low_battery = false;
    asus_pad_device->phone_batt_rsoc = 50;

    asus_pad_device->IsBalanceMode = 1;
    asus_pad_device->proc_lock_balance_mode = false;
#if 0
    init_waitqueue_head(&asus_pad_device->charger_status_event);
    init_waitqueue_head(&asus_pad_device->gaugefw_status_event);

    register_dock_attach_notifier(&attach_dock_notifier);
    register_dock_detach_notifier(&detach_dock_notifier);

    queue_delayed_work(battery_work_queue, &battery_poll_data_work, FIRST_RUN_TIME*HZ);
#endif

    ret = power_supply_register(dev, &asus_ec_power_supplies[0]);
    if (ret) {
        pr_info("PBATT> %s: failed to register %s\n", __func__,
                asus_ec_power_supplies[0].name);
        goto ec_power_reg_bat_fail;
    }

    ret = power_supply_register(dev, &asus_ec_power_supplies[1]);
    if (ret) {
        pr_info("PBATT> %s: failed to register %s\n", __func__,
                asus_ec_power_supplies[1].name);
        goto ec_power_reg_ac_fail;
    }

    ret = init_pad_power_toggle_on();
    if (ret)
        goto ec_power_reg_ac_fail;
    ret = init_pad_power_toggle_off();
    if (ret)
        goto ec_power_reg_ac_fail;

    register_microp_notifier(&ec_power_notifier);

#if 0
    ret = ec_routine_report_init();
    if (ret < 0)
        goto ec_power_rountine_fail;
#endif

    ret = create_balanceChg_proc_file();
    if (ret < 0)
        goto ec_power_rountine_fail;

    asus_battery_driver_ready = 1;
    return 0;

ec_power_rountine_fail:
ec_power_reg_ac_fail:
    power_supply_unregister(&asus_ec_power_supplies[0]);
ec_power_reg_bat_fail:
pad_dev_alloc_fail:
    kfree(asus_pad_device);
    return ret;
}

static int __devexit asus_battery_ec_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

#if 0
    wake_lock_destroy(&wakelock);
    wake_lock_destroy(&wakelock_t);
#endif

    return 0;
}

static int asus_pad_batt_suspend(struct device *dev)
{
    pr_info("PBATT> %s:\n", __func__);

    mutex_lock(&ec_info_lock);
    asus_pad_device->ec_suspend_status = 1;
    mutex_unlock(&ec_info_lock);

    if (_AX_MicroP_IsP01Connected_() == 1)
        balance_judge();
    return 0;
}

static void asus_pad_batt_resume(struct device *dev)
{
    pr_info("PBATT> %s:\n", __func__);

    mutex_lock(&ec_info_lock);
    asus_pad_device->ec_suspend_status = 0;
    mutex_unlock(&ec_info_lock);

    if (_AX_MicroP_IsP01Connected_() == 1)
        balance_judge();
}


static const struct platform_device_id asus_battery_ec_table[] = {
    {DRIVER_NAME, 1},
};

static const struct dev_pm_ops ec_pm_ops = {
    .prepare = asus_pad_batt_suspend,
    .complete = asus_pad_batt_resume,
};

static struct platform_driver asus_battery_ec_driver = {
    .driver = {
        .name  = DRIVER_NAME,
        .owner = THIS_MODULE,
        .pm    = &ec_pm_ops,
	},
    .probe      = asus_ec_power_probe,
    .remove     = __devexit_p(asus_battery_ec_remove),
    .id_table   = asus_battery_ec_table,
};

static int __init asus_ec_power_init(void)
{
    int rc;

    if (entry_mode == 5) return -1;
    HW_ID = Read_HW_ID();
    pr_info("PBATT> ++++++++++++++++ %s ++++++++++++++++\n", __func__);

    rc = platform_driver_register(&asus_battery_ec_driver);
    if (rc < 0) {
        pr_info("PBATT> %s: FAIL: platform_driver_register. rc = %d\n",
            __func__, rc);
        goto register_fail;
    }

    return 0;

register_fail:
    return rc;
}

static void __exit asus_ec_power_exit(void)
{
    int i;
    unregister_microp_notifier(&ec_power_notifier);
#if 0
    unregister_dock_attach_notifier(&attach_dock_notifier);
    unregister_dock_detach_notifier(&detach_dock_notifier);
    for (i = 0; i < ARRAY_SIZE(asus_pad_batt_supplies); i++)
        power_supply_changed(&asus_pad_batt_supplies[i]);
    dbg_i("%s: 'changed' event sent, sleeping for 10 seconds...\n",
            __func__);
    ssleep(10);

    for (i = 0; i < ARRAY_SIZE(asus_pad_batt_supplies); i++)
        power_supply_unregister(&asus_pad_batt_supplies[i]);
#endif
    kfree(asus_pad_device);
#if 0
    destroy_workqueue(battery_work_queue);
#endif
    platform_driver_unregister(&asus_battery_ec_driver);
    asus_battery_driver_ready = 0;
}

module_init(asus_ec_power_init);
module_exit(asus_ec_power_exit);

MODULE_AUTHOR("ASUS BSP");
MODULE_DESCRIPTION("ec power driver");
MODULE_LICENSE("GPL v2");
