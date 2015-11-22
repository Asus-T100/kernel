/*
 * sn280x.c - SN280X Charger I2C client driver
 *
 * Copyright (C) 2011 Intel Corporation
 * Copyright (c) 2013, ASUSTek, Inc.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>

#include <asm/intel_scu_ipc.h>
#include "sn280x.h"
#include "asus_battery.h"
#include "smb345_external_include.h"
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
extern int entry_mode;

/* global charger type variable lock */
DEFINE_MUTEX(gusb_state_lock);
static int gusb_state = CABLE_OUT;

#define DEV_NAME "sn280x"
#define DEV_MANUFACTURER "TI"
#define MODEL_NAME_SIZE 7
#define DEV_MANUFACTURER_NAME_SIZE 4

#define CHRG_TERM_WORKER_DELAY (30 * HZ)
#define EXCEPTION_MONITOR_DELAY (60 * HZ)
#define WDT_RESET_DELAY (15 * HZ)

/* SN280X registers */
#define SN280X_STAT_CTRL0_ADDR        0x00
#define SN280X_CTRL_ADDR        0x01
#define SN280X_BATT_VOL_CTRL_ADDR    0x02
#define SN280X_VENDOR_REV_ADDR        0x0A//#define SN280X_VENDOR_REV_ADDR        0x03
#define SN280X_TERM_FCC_ADDR        0x04
#define SN280X_VINDPM_STAT_ADDR    0x05
#define SN280X_ST_NTC_MON_ADDR        0x06

#define SN280X_RESET_MASK        (0x01 << 7)
#define SN280X_RESET_ENABLE        (0x01 << 7)

#define SN280X_FAULT_MASK        0x07
#define SN280X_STAT_MASK        (0x03 << 4)
#define SN280X_BOOST_MASK        (0x01 << 6)
#define SN280X_TMR_RST_MASK        (0x01 << 7)
#define SN280X_TMR_RST            (0x01 << 7)

#define SN280X_ENABLE_BOOST        (0x01 << 6)

#define SN280X_VOVP            0x01
#define SN280X_LOW_SUPPLY        0x02
#define SN280X_THERMAL_SHUTDOWN    0x03
#define SN280X_BATT_TEMP_FAULT        0x04
#define SN280X_TIMER_FAULT        0x05
#define SN280X_BATT_OVP        0x06
#define SN280X_NO_BATTERY        0x07
#define SN280X_STAT_READY        0x00

#define SN280X_STAT_CHRG_PRGRSS    (0x01 << 4)
#define SN280X_STAT_CHRG_DONE        (0x02 << 4)
#define SN280X_STAT_FAULT        (0x03 << 4)

#define SN280X_CE_MASK            (0x01 << 1)
#define SN280X_CE_DISABLE        (0x01 << 1)

#define SN280X_HZ_MASK            (0x01)
#define SN280X_HZ_ENABLE        (0x01)

#define SN280X_ICHRG_MASK        (0x1F << 3)
#define SN280X_ICHRG_100mA        (0x01 << 3)
#define SN280X_ICHRG_200mA        (0x01 << 4)
#define SN280X_ICHRG_400mA        (0x01 << 5)
#define SN280X_ICHRG_800mA        (0x01 << 6)
#define SN280X_ICHRG_1600mA        (0x01 << 7)

#define SN280X_ITERM_MASK        (0x03)
#define SN280X_ITERM_50mA        (0x01 << 0)
#define SN280X_ITERM_100mA        (0x01 << 1)
#define SN280X_ITERM_200mA        (0x01 << 2)

#define SN280X_VBREG_MASK        (0x3F << 2)

#define SN280X_INLMT_MASK        (0x03 << 4)
#define SN280X_INLMT_100        0x00
#define SN280X_INLMT_150        (0x01 << 4)
#define SN280X_INLMT_500        (0x02 << 4)
#define SN280X_INLMT_900        (0x03 << 4)
#define SN280X_INLMT_1500        (0x04 << 4)
#define SN280X_INLMT_2500        (0x06 << 4)

#define SN280X_TE_MASK            (0x01 << 2)
#define SN280X_TE_ENABLE        (0x01 << 2)
#define SN280X_STAT_ENABLE_MASK    (0x01 << 3)
#define SN280X_STAT_ENABLE        (0x01 << 3)

#define SN280X_VENDOR_MASK        (0x07 << 5)
#define SN280X_VENDOR            (0x02 << 5)
#define SN280X_REV_MASK        (0x07)
#define SN280X_REV            (0x02)
#define SN280X_REV            (0x01)

#define SN280X_TS_MASK            (0x01 << 3)
#define SN280X_TS_ENABLED        (0x01 << 3)
#define SN280X_BOOST_ILIM_MASK        (0x01 << 4)
#define SN280X_BOOST_ILIM_500mA    (0x0)
#define SN280X_BOOST_ILIM_1A        (0x01 << 4)

#define SN280X_SAFETY_TIMER_MASK    (0x03 << 5)
#define SN280X_SAFETY_TIMER_40MIN    0x00
#define SN280X_SAFETY_TIMER_6HR    (0x01 << 5)
#define SN280X_SAFETY_TIMER_9HR    (0x02 << 5)
#define SN280X_SAFETY_TIMER_DISABLED    (0x03 << 5)

/* 1% above voltage max design to report over voltage */
#define SN280X_OVP_MULTIPLIER            1010
#define SN280X_OVP_RECOVER_MULTIPLIER        990
#define SN280X_DEF_BAT_VOLT_MAX_DESIGN        4200000

/* Settings for Voltage / DPPM Register (05) */
#define SN280X_VBATT_LEVEL1        3700000
#define SN280X_VBATT_LEVEL2        3960000
#define SN280X_VINDPM_MASK        (0x07)
#define SN280X_VINDPM_320MV        (0x01 << 2)
#define SN280X_VINDPM_160MV        (0x01 << 1)
#define SN280X_VINDPM_80MV        (0x01 << 0)
#define SN280X_CD_STATUS_MASK        (0x01 << 3)
#define SN280X_DPM_EN_MASK        (0x01 << 4)
#define SN280X_DPM_EN_FORCE        (0x01 << 4)
#define SN280X_LOW_CHG_MASK        (0x01 << 5)
#define SN280X_LOW_CHG_EN        (0x01 << 5)
#define SN280X_LOW_CHG_DIS        (~SN280X_LOW_CHG_EN)
#define SN280X_DPM_STAT_MASK        (0x01 << 6)
#define SN280X_MINSYS_STAT_MASK    (0x01 << 7)

#define SN280X_MIN_CC            500

u16 sn280x_sfty_tmr[][2] = {
    {0, SN280X_SAFETY_TIMER_DISABLED}
    ,
    {40, SN280X_SAFETY_TIMER_40MIN}
    ,
    {360, SN280X_SAFETY_TIMER_6HR}
    ,
    {540, SN280X_SAFETY_TIMER_9HR}
    ,
};


u16 sn280x_inlmt[][2] = {
    {100, SN280X_INLMT_100}
    ,
    {150, SN280X_INLMT_150}
    ,
    {500, SN280X_INLMT_500}
    ,
    {900, SN280X_INLMT_900}
    ,
    {1500, SN280X_INLMT_1500}
    ,
    {2500, SN280X_INLMT_2500}
    ,
};

u16 sn280x_iterm[][2] = {
    {0, 0x00}
    ,
    {50, SN280X_ITERM_50mA}
    ,
    {100, SN280X_ITERM_100mA}
    ,
    {150, SN280X_ITERM_100mA | SN280X_ITERM_50mA}
    ,
    {200, SN280X_ITERM_200mA}
    ,
    {250, SN280X_ITERM_200mA | SN280X_ITERM_50mA}
    ,
    {300, SN280X_ITERM_200mA | SN280X_ITERM_100mA}
    ,
    {350, SN280X_ITERM_200mA | SN280X_ITERM_100mA | SN280X_ITERM_50mA}
    ,
};

u16 sn280x_cc[][2] = {

    {500, 0x00}
    ,
    {600, SN280X_ICHRG_100mA}
    ,
    {700, SN280X_ICHRG_200mA}
    ,
    {800, SN280X_ICHRG_100mA | SN280X_ICHRG_200mA}
    ,
    {900, SN280X_ICHRG_400mA}
    ,
    {1000, SN280X_ICHRG_400mA | SN280X_ICHRG_100mA}
    ,
    {1100, SN280X_ICHRG_400mA | SN280X_ICHRG_200mA}
    ,
    {1200, SN280X_ICHRG_400mA | SN280X_ICHRG_200mA | SN280X_ICHRG_100mA}
    ,
    {1300, SN280X_ICHRG_800mA}
    ,
    {1400, SN280X_ICHRG_800mA | SN280X_ICHRG_100mA}
    ,
    {1500, SN280X_ICHRG_800mA | SN280X_ICHRG_200mA}
    ,
};

#define SN280X_MIN_CV 3500
#define SN280X_MAX_CV 4440
#define SN280X_CV_DIV 20
#define SN280X_CV_BIT_POS 2

static enum power_supply_property sn280x_usb_props[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
    POWER_SUPPLY_PROP_TYPE,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_CHARGE_CURRENT,
    POWER_SUPPLY_PROP_CHARGE_VOLTAGE,
    POWER_SUPPLY_PROP_INLMT,
    POWER_SUPPLY_PROP_ENABLE_CHARGING,
    POWER_SUPPLY_PROP_ENABLE_CHARGER,
    POWER_SUPPLY_PROP_CHARGE_TERM_CUR,
    POWER_SUPPLY_PROP_CABLE_TYPE,
    POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
    POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
    POWER_SUPPLY_PROP_MODEL_NAME,
    POWER_SUPPLY_PROP_MANUFACTURER,
    POWER_SUPPLY_PROP_MAX_TEMP,
    POWER_SUPPLY_PROP_MIN_TEMP,
};

enum sn280x_chrgr_stat {
    SN280X_CHRGR_STAT_UNKNOWN,
    SN280X_CHRGR_STAT_READY,
    SN280X_CHRGR_STAT_CHARGING,
    SN280X_CHRGR_STAT_BAT_FULL,
    SN280X_CHRGR_STAT_FAULT,
};

struct sn280x_otg_event {
    struct list_head node;
    bool is_enable;
};

struct sn280x_charger {

    struct mutex lock;
    struct i2c_client *client;
    struct sn280x_plat_data *pdata;
    struct power_supply psy_usb;
    struct delayed_work sw_term_work;
    struct delayed_work wdt_work;
    struct delayed_work low_supply_fault_work;
    struct delayed_work exception_mon_work;
    struct notifier_block otg_nb;
    struct usb_phy *transceiver;
    struct work_struct otg_work;
    struct work_struct irq_work;
    struct list_head otg_queue;
    struct list_head irq_queue;
    wait_queue_head_t wait_ready;
    spinlock_t otg_queue_lock;

    int chrgr_health;
    int bat_health;
    int cc;
    int cv;
    int inlmt;
    int max_cc;
    int max_cv;
    int iterm;
    int cable_type;
    int cntl_state;
    int max_temp;
    int min_temp;
    enum sn280x_chrgr_stat chrgr_stat;
    bool online;
    bool present;
    bool is_charging_enabled;
    bool is_charger_enabled;
    bool is_vsys_on;
    bool boost_mode;
    bool is_hw_chrg_term;
    char model_name[MODEL_NAME_SIZE];
    char manufacturer[DEV_MANUFACTURER_NAME_SIZE];
};

enum sn280x_model_num {
    SN280X = 0,
};

struct sn280x_model {
    char model_name[MODEL_NAME_SIZE];
    enum sn280x_model_num model;
};

static struct sn280x_model sn280x_model_name[] = {
    { "sn280x", SN280X },
};

struct i2c_client *sn280x_client;
static inline int get_battery_voltage(int *volt);
static inline int get_battery_current(int *cur);
static int sn280x_handle_irq(struct sn280x_charger *chip, u8 stat_reg);
static inline int sn280x_set_iterm(struct sn280x_charger *chip, int iterm);

enum power_supply_type get_power_supply_type(
        enum power_supply_charger_cable_type cable)
{

    switch (cable) {

    case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
        return POWER_SUPPLY_TYPE_USB_DCP;
    case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
        return POWER_SUPPLY_TYPE_USB_CDP;
    case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
    case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
        return POWER_SUPPLY_TYPE_USB_ACA;
    case POWER_SUPPLY_CHARGER_TYPE_AC:
        return POWER_SUPPLY_TYPE_MAINS;
    case POWER_SUPPLY_CHARGER_TYPE_NONE:
    case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
    default:
        return POWER_SUPPLY_TYPE_USB;
    }

    return POWER_SUPPLY_TYPE_USB;
}

static void lookup_regval(u16 tbl[][2], size_t size, u16 in_val, u8 *out_val)
{
    int i;
    for (i = 1; i < size; ++i)
        if (in_val < tbl[i][0])
            break;

    *out_val = (u8) tbl[i - 1][1];
}

void sn280x_cc_to_reg(int cc, u8 *reg_val)
{
    return lookup_regval(sn280x_cc, ARRAY_SIZE(sn280x_cc), cc, reg_val);

}

void sn280x_cv_to_reg(int cv, u8 *reg_val)
{
    int val;

    val = clamp_t(int, cv, SN280X_MIN_CV, SN280X_MAX_CV);
    *reg_val =
        (((val - SN280X_MIN_CV) / SN280X_CV_DIV)
            << SN280X_CV_BIT_POS);
}

void sn280x_inlmt_to_reg(int inlmt, u8 *regval)
{
    return lookup_regval(sn280x_inlmt, ARRAY_SIZE(sn280x_inlmt),
                 inlmt, regval);
}

static inline void sn280x_iterm_to_reg(int iterm, u8 *regval)
{
    return lookup_regval(sn280x_iterm, ARRAY_SIZE(sn280x_iterm),
                 iterm, regval);
}

static inline void sn280x_sfty_tmr_to_reg(int tmr, u8 *regval)
{
    return lookup_regval(sn280x_sfty_tmr, ARRAY_SIZE(sn280x_sfty_tmr),
                 tmr, regval);
}

static inline int sn280x_read_reg(struct i2c_client *client, u8 reg)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        ret = i2c_smbus_read_byte_data(client, reg);
        if (ret < 0) {
            retry_count--;
            dev_warn(&client->dev,
                "fail to read reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    dev_info(&client->dev, "i2c_read REG%02X: " BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(ret));

    return ret;
}

static inline void sn280x_dump_regs(bool dump_master)
{
    int i;
    int ret;
    int bat_cur, bat_volt;
    struct sn280x_charger *chip;

    if (!sn280x_client)
        return;

    chip = i2c_get_clientdata(sn280x_client);
    dev_info(&sn280x_client->dev, "SN280X Register dump\n");

    dev_info(&sn280x_client->dev, "*======================*\n");
    for (i = 0; i < SN280X_N_REGISTERS; ++i) {
        ret = sn280x_read_reg(sn280x_client, i);
        if (ret < 0)
            dev_err(&sn280x_client->dev,
                "Error in reading REG 0x%X\n", i);
        else
            dev_info(&sn280x_client->dev,
                "0x%X=0x%X ", i, ret);
    }
    dev_info(&sn280x_client->dev, "*======================*\n");

    if (chip->pdata->dump_master_regs && dump_master)
            chip->pdata->dump_master_regs();
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret;

    ret = sn280x_get_charging_status();
    if (ret == POWER_SUPPLY_STATUS_CHARGING || ret == POWER_SUPPLY_STATUS_FULL)
        ret = 1;
    else
        ret = 0;
    return sprintf(buf, "%d\n", ret);
}
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
    &dev_attr_charge_status.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};

#ifdef CONFIG_DEBUG_FS
static int sn280x_reg_show(struct seq_file *seq, void *unused)
{
    int val;
    u8 reg;

    reg = *((u8 *)seq->private);
    val = sn280x_read_reg(sn280x_client, reg);

    seq_printf(seq, "0x%02x\n", val);
    return 0;
}

static int sn280x_dbgfs_open(struct inode *inode, struct file *file)
{
    return single_open(file, sn280x_reg_show, inode->i_private);
}

static u32 sn280x_register_set[] = {
    SN280X_REG_INPUT_SOURCE_CONTROL,
    SN280X_REG_POWER_ON_CONFIGURATION,
    SN280X_REG_CHARGE_CURRENT_CONTROL,
    SN280X_REG_PRECHARGE_TERMINATION,
    SN280X_REG_CHARGE_VOLTAGE_CONTROL,
    SN280X_REG_CHARGE_TERMINATION_TIMER,
    SN280X_REG_BOOST_VOL_THERMAL_REGULATION,
    SN280X_REG_MISC_OPERATION_CONTROL,
    SN280X_REG_SYSTEM_STATUS,
    SN280X_REG_NEW_FAULT,
    SN280X_REG_VENDOR_PART_REV_STATUS
};

static struct dentry *sn280x_dbgfs_dir;

static const struct file_operations sn280x_dbg_fops = {
    .open = sn280x_dbgfs_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release
};

static void sn280x_debugfs_init(void)
{
    struct dentry *fentry;
    u32 count = ARRAY_SIZE(sn280x_register_set);
    u32 i;
    char name[SN280X_N_REGISTERS] = {0};

    sn280x_dbgfs_dir = debugfs_create_dir(DEV_NAME, NULL);
    if (sn280x_dbgfs_dir == NULL)
        goto debugfs_root_exit;

    for (i = 0; i < count; i++) {
        snprintf(name, SN280X_N_REGISTERS, "%02x", sn280x_register_set[i]);
        fentry = debugfs_create_file(name, S_IRUGO,
                        sn280x_dbgfs_dir,
                        &sn280x_register_set[i],
                        &sn280x_dbg_fops);
        if (fentry == NULL)
            goto debugfs_err_exit;
    }
    dev_err(&sn280x_client->dev, "Debugfs created successfully!!\n");
    return;

debugfs_err_exit:
    debugfs_remove_recursive(sn280x_dbgfs_dir);
debugfs_root_exit:
    dev_err(&sn280x_client->dev, "Error Creating debugfs!!\n");
    return;
}

static void sn280x_debugfs_exit(void)
{
    if (sn280x_dbgfs_dir)
        debugfs_remove_recursive(sn280x_dbgfs_dir);

    return;
}

#else
static void sn280x_debugfs_init(void)
{
    return;
}

static void sn280x_debugfs_exit(void)
{
    return;
}
#endif

static inline int sn280x_write_reg(struct i2c_client *client, u8 reg, u8 data)
{
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do
    {
        ret = i2c_smbus_write_byte_data(client, reg, data);
        if (ret < 0) {
            retry_count--;
            dev_warn(&client->dev, "fail to write reg %02xh: %d\n",
                reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    dev_info(&client->dev, "i2c_write REG%02X: " BYTETOBINARYPATTERN
            "\n", reg, BYTETOBINARY(data));

    return ret;
}

static inline int sn280x_read_modify_reg(struct i2c_client *client, u8 reg,
                      u8 mask, u8 val)
{
    int ret;

    ret = sn280x_read_reg(client, reg);
    if (ret < 0)
        return ret;
    ret = (ret & ~mask) | (mask & val);
    return sn280x_write_reg(client, reg, ret);
}

static inline int sn280x_tmr_ntc_init(struct sn280x_charger *chip)
{
    u8 reg_val;
    int ret;

    sn280x_sfty_tmr_to_reg(chip->pdata->safety_timer, &reg_val);

    if (chip->pdata->is_ts_enabled)
        reg_val |= SN280X_TS_ENABLED;

    /* Check if boost mode current configuration is above 1A*/
    if (chip->pdata->boost_mode_mA >= 1000)
        reg_val |= SN280X_BOOST_ILIM_1A;

    ret = sn280x_read_modify_reg(chip->client, SN280X_ST_NTC_MON_ADDR,
            SN280X_TS_MASK|SN280X_SAFETY_TIMER_MASK|
            SN280X_BOOST_ILIM_MASK, reg_val);

    return ret;
}

static inline int sn280x_enable_charging(
    struct sn280x_charger *chip, bool val)
{
    int ret;
    u8 reg_val;
    bool is_ready;

    ret = sn280x_read_reg(chip->client,
                    SN280X_STAT_CTRL0_ADDR);
    if (ret < 0) {
        dev_err(&chip->client->dev,
            "Error(%d) in reading SN280X_STAT_CTRL0_ADDR\n", ret);
    }

    is_ready =  (ret & SN280X_STAT_MASK) != SN280X_STAT_FAULT;

    /* If status is fault, wait for READY before enabling the charging */

    if (!is_ready) {
        ret = wait_event_timeout(chip->wait_ready,
            (chip->chrgr_stat != SN280X_CHRGR_STAT_READY),
                HZ);
        dev_info(&chip->client->dev,
            "chrgr_stat=%x\n", chip->chrgr_stat);
        if (ret == 0) {
            dev_err(&chip->client->dev,
                "Waiting for Charger Ready Failed.Enabling charging anyway\n");
        }
    }

    if (chip->pdata->enable_charging)
        chip->pdata->enable_charging(val);

    if (val) {
        reg_val = (~SN280X_CE_DISABLE & SN280X_CE_MASK);
        if (chip->is_hw_chrg_term)
            reg_val |= SN280X_TE_ENABLE;
    } else {
        reg_val = SN280X_CE_DISABLE;
    }

    reg_val |=  SN280X_STAT_ENABLE;

    ret = sn280x_read_modify_reg(chip->client, SN280X_CTRL_ADDR,
               SN280X_STAT_ENABLE_MASK|SN280X_RESET_MASK|
                SN280X_CE_MASK|SN280X_TE_MASK,
                    reg_val);
    if (ret || !val)
        return ret;

    sn280x_set_iterm(chip, chip->iterm);
    return sn280x_tmr_ntc_init(chip);
}

static inline int sn280x_reset_timer(struct sn280x_charger *chip)
{
    return sn280x_read_modify_reg(chip->client, SN280X_STAT_CTRL0_ADDR,
            SN280X_TMR_RST_MASK, SN280X_TMR_RST);
}

static inline int sn280x_enable_charger(
    struct sn280x_charger *chip, int val)
{

    /* TODO: Implement enable/disable HiZ mode to enable/
    *  disable charger
    */
    u8 reg_val;
    int ret;

    reg_val = val ? (~SN280X_HZ_ENABLE & SN280X_HZ_MASK)  :
            SN280X_HZ_ENABLE;

    ret = sn280x_read_modify_reg(chip->client, SN280X_CTRL_ADDR,
               SN280X_HZ_MASK|SN280X_RESET_MASK, reg_val);
    if (ret)
        return ret;

    return sn280x_reset_timer(chip);
}

static inline int sn280x_set_cc(struct sn280x_charger *chip, int cc)
{
    u8 reg_val;
    int ret;

    dev_dbg(&chip->client->dev, "cc=%d\n", cc);
    if (chip->pdata->set_cc) {
        ret = chip->pdata->set_cc(cc);
        if (unlikely(ret))
            return ret;
    }

    if (cc && (cc < SN280X_MIN_CC)) {
        dev_dbg(&chip->client->dev, "Set LOW_CHG bit\n");
        reg_val = SN280X_LOW_CHG_EN;
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_VINDPM_STAT_ADDR,
                SN280X_LOW_CHG_MASK, reg_val);
    } else {
        dev_dbg(&chip->client->dev, "Clear LOW_CHG bit\n");
        reg_val = SN280X_LOW_CHG_DIS;
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_VINDPM_STAT_ADDR,
                SN280X_LOW_CHG_MASK, reg_val);
    }

    /* Return from here since the cc setting will be done
       by platform specific hardware */
    if (chip->pdata->set_cc)
        return ret;

    sn280x_cc_to_reg(cc, &reg_val);

    return sn280x_read_modify_reg(chip->client, SN280X_TERM_FCC_ADDR,
            SN280X_ICHRG_MASK, reg_val);
}

static inline int sn280x_set_cv(struct sn280x_charger *chip, int cv)
{
    int bat_volt;
    int ret;
    u8 reg_val;
    u8 vindpm_val = 0x0;

    /*
    * Setting VINDPM value as per the battery voltage
    *  VBatt           Vindpm     Register Setting
    *  < 3.7v           4.2v       0x0 (default)
    *  3.71v - 3.96v    4.36v      0x2
    *  > 3.96v          4.6v       0x5
    */
    ret = get_battery_voltage(&bat_volt);
    if (ret) {
        dev_err(&chip->client->dev,
            "Error getting battery voltage!!\n");
    } else {
        if (bat_volt > SN280X_VBATT_LEVEL2)
            vindpm_val =
                (SN280X_VINDPM_320MV | SN280X_VINDPM_80MV);
        else if (bat_volt > SN280X_VBATT_LEVEL1)
            vindpm_val = SN280X_VINDPM_160MV;
    }

    ret = sn280x_read_modify_reg(chip->client,
            SN280X_VINDPM_STAT_ADDR,
            SN280X_VINDPM_MASK,
            vindpm_val);
    if (ret) {
        dev_err(&chip->client->dev,
            "Error setting VINDPM setting!!\n");
        return ret;
    }

    if (chip->pdata->set_cv)
        return chip->pdata->set_cv(cv);

    sn280x_cv_to_reg(cv, &reg_val);

    return sn280x_read_modify_reg(chip->client, SN280X_BATT_VOL_CTRL_ADDR,
                       SN280X_VBREG_MASK, reg_val);
}

static inline int sn280x_set_inlmt(struct sn280x_charger *chip, int inlmt)
{
    u8 reg_val;

    if (chip->pdata->set_inlmt)
        return chip->pdata->set_inlmt(inlmt);

    sn280x_inlmt_to_reg(inlmt, &reg_val);

    return sn280x_read_modify_reg(chip->client, SN280X_CTRL_ADDR,
               SN280X_RESET_MASK|SN280X_INLMT_MASK, reg_val);

}

static inline void resume_charging(struct sn280x_charger *chip)
{

    if (chip->is_charger_enabled)
        sn280x_enable_charger(chip, true);
    if (chip->inlmt)
        sn280x_set_inlmt(chip, chip->inlmt);
    if (chip->cc)
        sn280x_set_cc(chip, chip->cc);
    if (chip->cv)
        sn280x_set_cv(chip, chip->cv);
    if (chip->is_charging_enabled)
        sn280x_enable_charging(chip, true);
}

static inline int sn280x_set_iterm(struct sn280x_charger *chip, int iterm)
{
    u8 reg_val;

    if (chip->pdata->set_iterm)
        return chip->pdata->set_iterm(iterm);

    sn280x_iterm_to_reg(iterm, &reg_val);

    return sn280x_read_modify_reg(chip->client, SN280X_TERM_FCC_ADDR,
                       SN280X_ITERM_MASK, reg_val);
}

static inline int sn280x_enable_hw_charge_term(
    struct sn280x_charger *chip, bool val)
{
    u8 data;
    int ret;

    data = val ? SN280X_TE_ENABLE : (~SN280X_TE_ENABLE & SN280X_TE_MASK);


    ret = sn280x_read_modify_reg(chip->client, SN280X_CTRL_ADDR,
                   SN280X_RESET_MASK|SN280X_TE_MASK, data);

    if (ret)
        return ret;

    chip->is_hw_chrg_term = val ? true : false;

    return ret;
}

static inline int sn280x_enable_boost_mode(
    struct sn280x_charger *chip, int val)
{
    int ret = 0;


    if (val) {

        if (chip->pdata->enable_vbus)
            chip->pdata->enable_vbus(true);

        /* TODO: Support different Host Mode Current limits */

        sn280x_enable_charger(chip, true);
        ret =
            sn280x_read_modify_reg(chip->client,
                        SN280X_STAT_CTRL0_ADDR,
                        SN280X_BOOST_MASK,
                        SN280X_ENABLE_BOOST);
        if (unlikely(ret))
            return ret;

        ret = sn280x_tmr_ntc_init(chip);
        if (unlikely(ret))
            return ret;
        chip->boost_mode = true;

        schedule_delayed_work(&chip->wdt_work, 0);

        dev_info(&chip->client->dev, "Boost Mode enabled\n");
    } else {

        ret =
            sn280x_read_modify_reg(chip->client,
                        SN280X_STAT_CTRL0_ADDR,
                        SN280X_BOOST_MASK,
                        ~SN280X_ENABLE_BOOST);

        if (unlikely(ret))
            return ret;
        /* if charging need not to be enabled, disable
        * the charger else keep the charger on
        */
        if (!chip->is_charging_enabled)
            sn280x_enable_charger(chip, false);
        chip->boost_mode = false;
        dev_info(&chip->client->dev, "Boost Mode disabled\n");
        cancel_delayed_work_sync(&chip->wdt_work);

        if (chip->pdata->enable_vbus)
            chip->pdata->enable_vbus(false);

        /* Notify power supply subsystem to enable charging
         * if needed. Eg. if DC adapter is connected
         */
        power_supply_changed(&chip->psy_usb);
    }

    return ret;
}

static inline bool sn280x_is_vsys_on(struct sn280x_charger *chip)
{
    int ret;
    struct i2c_client *client = chip->client;

    ret = sn280x_read_reg(client, SN280X_CTRL_ADDR);
    if (ret < 0) {
        dev_err(&client->dev,
            "Error(%d) in reading SN280X_CTRL_ADDR\n", ret);
        return false;
    }

    if (((ret & SN280X_HZ_MASK) == SN280X_HZ_ENABLE) &&
            chip->is_charger_enabled) {
        dev_err(&client->dev, "Charger in Hi Z Mode\n");
        sn280x_dump_regs(true);
        return false;
    }

    ret = sn280x_read_reg(client, SN280X_VINDPM_STAT_ADDR);
    if (ret < 0) {
        dev_err(&client->dev,
            "Error(%d) in reading SN280X_VINDPM_STAT_ADDR\n", ret);
        return false;
    }

    if (ret & SN280X_CD_STATUS_MASK) {
        dev_err(&client->dev, "CD line asserted\n");
        sn280x_dump_regs(true);
        return false;
    }

    return true;
}


static inline bool sn280x_is_online(struct sn280x_charger *chip)
{
    if (chip->cable_type == POWER_SUPPLY_CHARGER_TYPE_NONE)
        return false;
    else if (!chip->is_charger_enabled)
        return false;
    /* SN280X gives interrupt only on stop/resume charging.
     * If charging is already stopped, we need to query the hardware
     * to see charger is still active and can supply vsys or not.
     */
    else if ((chip->chrgr_stat == SN280X_CHRGR_STAT_FAULT) ||
         (!chip->is_charging_enabled))
        return sn280x_is_vsys_on(chip);
    else
        return chip->is_vsys_on;
}

static int sn280x_usb_set_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    const union power_supply_propval *val)
{
    struct sn280x_charger *chip = container_of(psy,
                            struct sn280x_charger,
                            psy_usb);
    int ret = 0;


    mutex_lock(&chip->lock);


    switch (psp) {

    case POWER_SUPPLY_PROP_PRESENT:
        chip->present = val->intval;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
        chip->online = val->intval;
        break;
    case POWER_SUPPLY_PROP_ENABLE_CHARGING:

        ret = sn280x_enable_charging(chip, val->intval);

        if (ret)
            dev_err(&chip->client->dev,
                "Error(%d) in %s charging", ret,
                (val->intval ? "enable" : "disable"));
        else
            chip->is_charging_enabled = val->intval;

        if (val->intval)
            sn280x_enable_hw_charge_term(chip, true);
        else
            cancel_delayed_work_sync(&chip->sw_term_work);

        break;
    case POWER_SUPPLY_PROP_ENABLE_CHARGER:

        /* Don't enable the charger unless overvoltage is recovered */

        if (chip->bat_health != POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
            ret = sn280x_enable_charger(chip, val->intval);

            if (ret)
                dev_err(&chip->client->dev,
                    "Error(%d) in %s charger", ret,
                    (val->intval ? "enable" : "disable"));
            else
                chip->is_charger_enabled = val->intval;
        } else {
            dev_info(&chip->client->dev, "Battery Over Voltage. Charger will be disabled\n");
        }
        break;
    case POWER_SUPPLY_PROP_CHARGE_CURRENT:
        ret = sn280x_set_cc(chip, val->intval);
        if (!ret)
            chip->cc = val->intval;
        break;
    case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
        ret = sn280x_set_cv(chip, val->intval);
        if (!ret)
            chip->cv = val->intval;
        break;
    case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
        chip->max_cc = val->intval;
        break;
    case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
        chip->max_cv = val->intval;
        break;
    case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
        ret = sn280x_set_iterm(chip, val->intval);
        if (!ret)
            chip->iterm = val->intval;
        break;
    case POWER_SUPPLY_PROP_CABLE_TYPE:

        chip->cable_type = val->intval;
        chip->psy_usb.type = get_power_supply_type(chip->cable_type);
        if (chip->cable_type != POWER_SUPPLY_CHARGER_TYPE_NONE) {
            chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
            chip->chrgr_stat = SN280X_CHRGR_STAT_UNKNOWN;

            /* Adding this processing in order to check
            for any faults during connect */

            ret = sn280x_read_reg(chip->client,
                        SN280X_STAT_CTRL0_ADDR);
            if (ret < 0)
                dev_err(&chip->client->dev,
                "Error (%d) in reading status register(0x00)\n",
                ret);
            else
                sn280x_handle_irq(chip, ret);
        } else {
            chip->chrgr_stat = SN280X_CHRGR_STAT_UNKNOWN;
            chip->chrgr_health = POWER_SUPPLY_HEALTH_UNKNOWN;
            cancel_delayed_work_sync(&chip->low_supply_fault_work);
        }


        break;
    case POWER_SUPPLY_PROP_INLMT:
        ret = sn280x_set_inlmt(chip, val->intval);
        if (!ret)
            chip->inlmt = val->intval;
        break;
    case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
        chip->cntl_state = val->intval;
        break;
    case POWER_SUPPLY_PROP_MAX_TEMP:
        chip->max_temp = val->intval;
        break;
    case POWER_SUPPLY_PROP_MIN_TEMP:
        chip->min_temp = val->intval;
        break;
    default:
        ret = -ENODATA;
    }

    mutex_unlock(&chip->lock);
    return ret;
}

static int sn280x_usb_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    struct sn280x_charger *chip = container_of(psy,
                            struct sn280x_charger,
                            psy_usb);

    mutex_lock(&chip->lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = chip->present;
        break;
    case POWER_SUPPLY_PROP_ONLINE:
        val->intval = chip->online;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = chip->chrgr_health;
        break;
    case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
        val->intval = chip->max_cc;
        break;
    case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
        val->intval = chip->max_cv;
        break;
    case POWER_SUPPLY_PROP_CHARGE_CURRENT:
        val->intval = chip->cc;
        break;
    case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
        val->intval = chip->cv;
        break;
    case POWER_SUPPLY_PROP_INLMT:
        val->intval = chip->inlmt;
        break;
    case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
        val->intval = chip->iterm;
        break;
    case POWER_SUPPLY_PROP_CABLE_TYPE:
        val->intval = chip->cable_type;
        break;
    case POWER_SUPPLY_PROP_ENABLE_CHARGING:
        if (chip->boost_mode)
            val->intval = false;
        else
            val->intval = (chip->is_charging_enabled &&
            (chip->chrgr_stat == SN280X_CHRGR_STAT_CHARGING));

        break;
    case POWER_SUPPLY_PROP_ENABLE_CHARGER:
        val->intval = sn280x_is_online(chip);
        break;
    case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
        val->intval = chip->cntl_state;
        break;
    case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
        val->intval = chip->pdata->num_throttle_states;
        break;
    case POWER_SUPPLY_PROP_MODEL_NAME:
        val->strval = chip->model_name;
        break;
    case POWER_SUPPLY_PROP_MANUFACTURER:
        val->strval = chip->manufacturer;
        break;
    case POWER_SUPPLY_PROP_MAX_TEMP:
        val->intval = chip->max_temp;
        break;
    case POWER_SUPPLY_PROP_MIN_TEMP:
        val->intval = chip->min_temp;
        break;
    default:
        mutex_unlock(&chip->lock);
        return -EINVAL;
    }

    mutex_unlock(&chip->lock);
    return 0;
}

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

static inline int get_battery_voltage(int *volt)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
    if (!ret)
        *volt = (val.intval);

    return ret;
}

static inline int get_battery_volt_max_design(int *volt)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy,
        POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN, &val);
    if (!ret)
        (*volt = val.intval);
    return ret;
}

static inline int get_battery_current(int *cur)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
    if (!ret)
        *cur = val.intval;

    return ret;
}

static inline int get_battery_temperature(int *tempr)
{
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_psy_battery();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
    if (!ret)
        *tempr = val.intval;

    return ret;
}

static void sn280x_wdt_reset_worker(struct work_struct *work)
{

    struct sn280x_charger *chip = container_of(work,
                struct sn280x_charger, wdt_work.work);
    int ret;
    ret = sn280x_reset_timer(chip);

    if (ret)
        dev_err(&chip->client->dev, "Error (%d) in WDT reset\n");
    else
        dev_info(&chip->client->dev, "WDT reset\n");

    schedule_delayed_work(&chip->wdt_work, WDT_RESET_DELAY);
}

static void sn280x_sw_charge_term_worker(struct work_struct *work)
{

    struct sn280x_charger *chip = container_of(work,
                            struct sn280x_charger,
                            sw_term_work.work);

    power_supply_changed(NULL);

    schedule_delayed_work(&chip->sw_term_work,
                  CHRG_TERM_WORKER_DELAY);

}

int sn280x_get_bat_health(void)
{

    struct sn280x_charger *chip;

    if (!sn280x_client)
        return -ENODEV;

    chip = i2c_get_clientdata(sn280x_client);

    return chip->bat_health;
}


static void sn280x_low_supply_fault_work(struct work_struct *work)
{
    struct sn280x_charger *chip = container_of(work,
                            struct sn280x_charger,
                            low_supply_fault_work.work);

    if (chip->chrgr_stat == SN280X_CHRGR_STAT_FAULT) {
        dev_err(&chip->client->dev, "Low Supply Fault detected!!\n");
        chip->chrgr_health = POWER_SUPPLY_HEALTH_DEAD;
        power_supply_changed(&chip->psy_usb);
        sn280x_dump_regs(true);
    }
    return;
}


/* is_bat_over_voltage: check battery is over voltage or not
*  @chip: sn280x_charger context
*
*  This function is used to verify the over voltage condition.
*  In some scenarios, HW generates Over Voltage exceptions when
*  battery voltage is normal. This function uses the over voltage
*  condition (voltage_max_design * 1.01) to verify battery is really
*  over charged or not.
*/

static bool is_bat_over_voltage(struct sn280x_charger *chip,
        bool verify_recovery)
{

    int bat_volt, bat_volt_max_des, ret;

    ret = get_battery_voltage(&bat_volt);
    if (ret)
        return verify_recovery ? false : true;

    ret = get_battery_volt_max_design(&bat_volt_max_des);

    if (ret)
        bat_volt_max_des = SN280X_DEF_BAT_VOLT_MAX_DESIGN;

    dev_info(&chip->client->dev, "bat_volt=%d Voltage Max Design=%d OVP_VOLT=%d OVP recover volt=%d\n",
            bat_volt, bat_volt_max_des,
            (bat_volt_max_des/1000 * SN280X_OVP_MULTIPLIER),
            (bat_volt_max_des/1000 *
                SN280X_OVP_RECOVER_MULTIPLIER));
    if (verify_recovery) {
        if ((bat_volt) <= (bat_volt_max_des / 1000 *
                SN280X_OVP_RECOVER_MULTIPLIER))
            return true;
        else
            return false;
    } else {
        if ((bat_volt) >= (bat_volt_max_des / 1000 *
                    SN280X_OVP_MULTIPLIER))
            return true;
        else
            return false;
    }

    return false;
}

#define IS_BATTERY_OVER_VOLTAGE(chip) \
    is_bat_over_voltage(chip , false)

#define IS_BATTERY_OVER_VOLTAGE_RECOVERED(chip) \
    is_bat_over_voltage(chip , true)

static void handle_battery_over_voltage(struct sn280x_charger *chip)
{
    /* Set Health to Over Voltage. Disable charger to discharge
    *  battery to reduce the battery voltage.
    */
    chip->bat_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    sn280x_enable_charger(chip, false);
    chip->is_charger_enabled = false;
    cancel_delayed_work_sync(&chip->exception_mon_work);
    schedule_delayed_work(&chip->exception_mon_work,
            EXCEPTION_MONITOR_DELAY);
}

static void sn280x_exception_mon_work(struct work_struct *work)
{
    struct sn280x_charger *chip = container_of(work,
                            struct sn280x_charger,
                            exception_mon_work.work);
    /* Only overvoltage exception need to monitor.*/
    if (IS_BATTERY_OVER_VOLTAGE_RECOVERED(chip)) {
        dev_info(&chip->client->dev, "Over Voltage Exception Recovered\n");
        chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
        sn280x_enable_charger(chip, true);
        chip->is_charger_enabled = true;
        resume_charging(chip);
    } else {
        schedule_delayed_work(&chip->exception_mon_work,
                  EXCEPTION_MONITOR_DELAY);
    }
}

static int sn280x_handle_irq(struct sn280x_charger *chip, u8 stat_reg)
{
    struct i2c_client *client = chip->client;
    bool notify = true;

    dev_info(&client->dev, "%s:%d stat=0x%x\n",
            __func__, __LINE__, stat_reg);

    switch (stat_reg & SN280X_STAT_MASK) {
    case SN280X_STAT_READY:
        chip->chrgr_stat = SN280X_CHRGR_STAT_READY;
        chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
        chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
        dev_info(&client->dev, "Charger Status: Ready\n");
        notify = false;
        break;
    case SN280X_STAT_CHRG_PRGRSS:
        chip->chrgr_stat = SN280X_CHRGR_STAT_CHARGING;
        chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
        chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
        dev_info(&client->dev, "Charger Status: Charge Progress\n");
        sn280x_dump_regs(false);
        break;
    case SN280X_STAT_CHRG_DONE:
        chip->chrgr_health = POWER_SUPPLY_HEALTH_GOOD;
        chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;
        dev_info(&client->dev, "Charger Status: Charge Done\n");

        sn280x_enable_hw_charge_term(chip, false);
        resume_charging(chip);
        schedule_delayed_work(&chip->sw_term_work, 0);
        break;

    case SN280X_STAT_FAULT:
        break;
    }

    if (stat_reg & SN280X_BOOST_MASK)
        dev_info(&client->dev, "Boost Mode\n");

    if ((stat_reg & SN280X_STAT_MASK) == SN280X_STAT_FAULT) {
        bool dump_master = true;
        chip->chrgr_stat = SN280X_CHRGR_STAT_FAULT;

        switch (stat_reg & SN280X_FAULT_MASK) {
        case SN280X_VOVP:
            chip->chrgr_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
            dev_err(&client->dev, "Charger OVP Fault\n");
            break;

        case SN280X_LOW_SUPPLY:
            notify = false;
            if (chip->cable_type !=
                    POWER_SUPPLY_CHARGER_TYPE_NONE) {
                schedule_delayed_work
                    (&chip->low_supply_fault_work,
                    5*HZ);
                dev_dbg(&client->dev,
                    "Schedule Low Supply Fault work!!\n");
            }
            break;

        case SN280X_THERMAL_SHUTDOWN:
            chip->chrgr_health = POWER_SUPPLY_HEALTH_OVERHEAT;
            dev_err(&client->dev, "Charger Thermal Fault\n");
            break;

        case SN280X_BATT_TEMP_FAULT:
            chip->bat_health = POWER_SUPPLY_HEALTH_OVERHEAT;
            dev_err(&client->dev, "Battery Temperature Fault\n");
            break;

        case SN280X_TIMER_FAULT:
            chip->bat_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
            chip->chrgr_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
            dev_err(&client->dev, "Charger Timer Fault\n");
            break;

        case SN280X_BATT_OVP:
            notify = false;
            if (chip->bat_health !=
                    POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
                if (!IS_BATTERY_OVER_VOLTAGE(chip)) {
                    chip->chrgr_stat =
                        SN280X_CHRGR_STAT_UNKNOWN;
                    resume_charging(chip);
                } else {
                    dev_err(&client->dev, "Battery Over Voltage Fault\n");
                    handle_battery_over_voltage(chip);
                    notify = true;
                }
            }
            break;
        case SN280X_NO_BATTERY:
            dev_err(&client->dev, "No Battery Connected\n");
            break;

        }

        if (chip->chrgr_stat == SN280X_CHRGR_STAT_FAULT && notify)
            sn280x_dump_regs(dump_master);
    }

    wake_up(&chip->wait_ready);

    chip->is_vsys_on = sn280x_is_vsys_on(chip);
    if (notify)
        power_supply_changed(&chip->psy_usb);

    return 0;
}

static void sn280x_irq_worker(struct work_struct *work)
{
    struct sn280x_charger *chip =
        container_of(work, struct sn280x_charger, irq_work);
    int ret;

    /*Lock to ensure that interrupt register readings are done
    * and processed sequentially. The interrupt Fault registers
    * are read on clear and without sequential processing double
    * fault interrupts or fault recovery cannot be handlled propely
    */

    mutex_lock(&chip->lock);

    dev_dbg(&chip->client->dev, "%s\n", __func__);

    ret = sn280x_read_reg(chip->client, SN280X_STAT_CTRL0_ADDR);
    if (ret < 0)
        dev_err(&chip->client->dev,
            "Error (%d) in reading SN280X_STAT_CTRL0_ADDR\n", ret);
    else
        sn280x_handle_irq(chip, ret);

    mutex_unlock(&chip->lock);
}

static irqreturn_t sn280x_thread_handler(int id, void *data)
{
    struct sn280x_charger *chip = (struct sn280x_charger *)data;

    queue_work(system_nrt_wq, &chip->irq_work);
    return IRQ_HANDLED;
}

static irqreturn_t sn280x_irq_handler(int irq, void *data)
{
    struct sn280x_charger *chip = (struct sn280x_charger *)data;
    u8 intr_stat;

    return IRQ_NONE;
}

static void sn280x_boostmode_worker(struct work_struct *work)
{
    struct sn280x_charger *chip =
        container_of(work, struct sn280x_charger, otg_work);
    struct sn280x_otg_event *evt, *tmp;
    unsigned long flags;

    spin_lock_irqsave(&chip->otg_queue_lock, flags);
    list_for_each_entry_safe(evt, tmp, &chip->otg_queue, node) {
        list_del(&evt->node);
        spin_unlock_irqrestore(&chip->otg_queue_lock, flags);

        dev_info(&chip->client->dev,
            "%s:%d state=%d\n", __FILE__, __LINE__,
                evt->is_enable);
        mutex_lock(&chip->lock);
        if (evt->is_enable)
            sn280x_enable_boost_mode(chip, 1);
        else
            sn280x_enable_boost_mode(chip, 0);

        mutex_unlock(&chip->lock);
        spin_lock_irqsave(&chip->otg_queue_lock, flags);
        kfree(evt);

    }
    spin_unlock_irqrestore(&chip->otg_queue_lock, flags);
}

static int otg_handle_notification(struct notifier_block *nb,
                   unsigned long event, void *param)
{

    struct sn280x_charger *chip =
        container_of(nb, struct sn280x_charger, otg_nb);
    struct sn280x_otg_event *evt;

    dev_dbg(&chip->client->dev, "OTG notification: %lu\n", event);
    if (!param || event != USB_EVENT_DRIVE_VBUS)
        return NOTIFY_DONE;

    evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
    if (!evt) {
        dev_err(&chip->client->dev,
            "failed to allocate memory for OTG event\n");
        return NOTIFY_DONE;
    }

    evt->is_enable = *(int *)param;
    INIT_LIST_HEAD(&evt->node);

    spin_lock(&chip->otg_queue_lock);
    list_add_tail(&evt->node, &chip->otg_queue);
    spin_unlock(&chip->otg_queue_lock);

    queue_work(system_nrt_wq, &chip->otg_work);
    return NOTIFY_OK;
}

static inline int register_otg_notifications(struct sn280x_charger *chip)
{

    int retval;

    INIT_LIST_HEAD(&chip->otg_queue);
    INIT_WORK(&chip->otg_work, sn280x_boostmode_worker);
    spin_lock_init(&chip->otg_queue_lock);

    chip->otg_nb.notifier_call = otg_handle_notification;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
    chip->transceiver = usb_get_transceiver();
#else
    chip->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);
#endif
    if (!chip->transceiver) {
        dev_err(&chip->client->dev, "failed to get otg transceiver\n");
        return -EINVAL;
    }
    retval = usb_register_notifier(chip->transceiver, &chip->otg_nb);
    if (retval) {
        dev_err(&chip->client->dev,
            "failed to register otg notifier\n");
        return -EINVAL;
    }

    return 0;
}

static enum sn280x_model_num sn280x_get_model(int sn280x_rev_reg)
{
    switch (sn280x_rev_reg & SN280X_REV_MASK) {
    case SN280X_REV:
        return SN280X;
    default:
        return -EINVAL;
    }
}

#define SN280X_CHRG_CUR_LOW        100    /* 100mA  */
#define SN280X_CHRG_CUR_MEDIUM     500    /* 500mA  */
#define SN280X_CHRG_CUR_HIGH       900    /* 900mA  */
#define SN280X_CHRG_CUR_NOLIMIT    1500   /* 1500mA */

static struct power_supply_throttle sn280x_throttle_states[] = {
    {
        .throttle_action = PSY_THROTTLE_CC_LIMIT,
        .throttle_val = SN280X_CHRG_CUR_NOLIMIT,
    },
    {
        .throttle_action = PSY_THROTTLE_CC_LIMIT,
        .throttle_val = SN280X_CHRG_CUR_MEDIUM,
    },
    {
        .throttle_action = PSY_THROTTLE_DISABLE_CHARGING,
    },
    {
        .throttle_action = PSY_THROTTLE_DISABLE_CHARGER,
    },
};

char *sn280x_supplied_to[] = {
    "battery",
};

void *sn280x_platform_data()
{
    static struct sn280x_plat_data sn280x_pdata;

    sn280x_pdata.supplied_to = sn280x_supplied_to;
    sn280x_pdata.num_supplicants = ARRAY_SIZE(sn280x_supplied_to);
    sn280x_pdata.throttle_states = sn280x_throttle_states;
    sn280x_pdata.num_throttle_states = ARRAY_SIZE(sn280x_throttle_states);
    sn280x_pdata.enable_charger = NULL;
    sn280x_pdata.set_iterm = NULL;
    sn280x_pdata.boost_mode_mA = 1000;

    return &sn280x_pdata;
}

int sn280x_get_charging_status(void)
{
    int val, status;
    struct sn280x_charger *chip;

    if (!sn280x_client)
        return -ENODEV;

    chip = i2c_get_clientdata(sn280x_client);
    if (!chip)
        pr_err("error: chip is null\n");
    dev_info(&chip->client->dev, "%s\n", __func__);

    val = sn280x_read_reg(sn280x_client, SN280X_REG_NEW_FAULT);
    if (val < 0) {
        dev_err(&chip->client->dev,
            "failed to read SN280X_REG_NEW_FAULT: %d\n", val);
        return POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    if (val) {
        dev_err(&chip->client->dev,
            "IC fault! SN280X_REG_NEW_FAULT: 0x%x\n", val);
        #if 0
        return POWER_SUPPLY_STATUS_UNKNOWN;
        #endif
    }

    val = sn280x_read_reg(sn280x_client, SN280X_REG_SYSTEM_STATUS);
    if (val < 0) {
        dev_err(&chip->client->dev,
            "failed to read SN280X_REG_SYSTEM_STATUS\n", val);
        return POWER_SUPPLY_STATUS_NOT_CHARGING;
    }

    if (!(val & SN280X_MASK_CHRG_STAT))
        status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    else if ((val & SN280X_MASK_CHRG_STAT) == SN280X_MASK_CHRG_STAT)
        status = POWER_SUPPLY_STATUS_FULL;
    else
        status = POWER_SUPPLY_STATUS_CHARGING;

    return status;
}

static char *supply_list[] = {
    "battery",
};

static enum power_supply_property asus_power_properties[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
};

static int sn280x_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val);

static struct power_supply sn280x_power_supplies[] = {
    {
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = asus_power_properties,
        .num_properties = ARRAY_SIZE(asus_power_properties),
        .get_property = sn280x_power_get_property,
    },
    {
        .name = "usb",
        .type = POWER_SUPPLY_TYPE_USB,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = asus_power_properties,
        .num_properties = ARRAY_SIZE(asus_power_properties),
        .get_property = sn280x_power_get_property,
    },
};

static int sn280x_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val)
{
    int ret=0;
    int usb_state;
    int chrg_status;

    mutex_lock(&gusb_state_lock);
    usb_state = gusb_state;
    mutex_unlock(&gusb_state_lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_USB) {
            val->intval = (usb_state == USB_IN) ? 1 : 0;
        }
        else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
            val->intval = (usb_state == AC_IN) ? 1 : 0;
        }
        else {
            ret = -EINVAL;
        }
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        if (psy->type == POWER_SUPPLY_TYPE_USB) {
            chrg_status = sn280x_get_charging_status();
            if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
                val->intval = 1;
                return 0;
            }
            else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
                val->intval = 1;
                return 0;
            }
            val->intval = 0;
        }
        else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
            chrg_status = sn280x_get_charging_status();
            if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
                val->intval = 1;
                return 0;
            }
            else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
                val->intval = 1;
                return 0;
            }
            val->intval = 0;
        }
        else
            ret = -EINVAL;
        break;
    default:
        ret = -EINVAL;
    }

    return ret;
}

static int sn280x_register_power_supply(struct device *dev)
{
    int ret;

    ret = power_supply_register(dev, &sn280x_power_supplies[CHARGER_USB-1]);
    if (ret) {
        BAT_DBG_E("Fail to register power supply USB\n");
        goto batt_err_reg_fail_usb;
    }

    ret = power_supply_register(dev, &sn280x_power_supplies[CHARGER_AC-1]);
    if (ret) {
        BAT_DBG_E("Fail to register power supply AC\n");
        goto batt_err_reg_fail_ac;
    }

    return 0;

batt_err_reg_fail_ac:
    power_supply_unregister(&sn280x_power_supplies[CHARGER_USB-1]);
batt_err_reg_fail_usb:
    return ret;
}

int sn280x_dis_watchdog_timer_limit(struct sn280x_charger *chip)
{
    int ret;

    /* Disable watchdog timer limit */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_CHARGE_TERMINATION_TIMER,
            SN280X_MASK_I2C_WATCHDOG_TIMER_LIMIT,
            SN280X_VAL_I2C_WATCHDOG_TIMER_LIMIT);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to disable watchdog timer limit\n");
        return ret;
    }

    return ret;
}

int sn280x_chgr_state(int usb_state)
{
    int ret = 0;

    mutex_lock(&gusb_state_lock);
    gusb_state = usb_state;
    mutex_unlock(&gusb_state_lock);

    if (!sn280x_client)
        return -ENODEV;

    switch (usb_state)
    {
    case USB_IN:
        power_supply_changed(&sn280x_power_supplies[CHARGER_USB-1]);
        break;

    case AC_IN:
        power_supply_changed(&sn280x_power_supplies[CHARGER_AC-1]);
        break;

    case CABLE_OUT:
        power_supply_changed(&sn280x_power_supplies[CHARGER_AC-1]);
        power_supply_changed(&sn280x_power_supplies[CHARGER_USB-1]);
        break;
    }

    switch (usb_state)
    {
    case USB_IN:
        BAT_DBG(" usb_state: USB_IN\n");
        usb_to_battery_callback(USB_PC);

        sn280x_set_chrgr_type(USB_IN);
        sn280x_soc_control_jeita();
        break;
    case AC_IN:
        BAT_DBG(" usb_state: AC_IN\n");
        usb_to_battery_callback(USB_ADAPTER);

        sn280x_set_chrgr_type(AC_IN);
        sn280x_soc_control_jeita();
        break;
    case CABLE_OUT:
        BAT_DBG(" usb_state: CABLE_OUT\n");
        usb_to_battery_callback(NO_CABLE);
        break;
    case ENABLE_5V:
        BAT_DBG(" usb_state: ENABLE_5V\n");
        ret = sn280x_set_boost_otg(1);
        break;
    case DISABLE_5V:
        BAT_DBG(" usb_state: DISABLE_5V\n");
        ret = sn280x_set_boost_otg(0);
        break;
    default:
        BAT_DBG(" ERROR: wrong usb state value = %d\n", usb_state);
        ret = 1;
    }

    return ret;
}
EXPORT_SYMBOL(sn280x_chgr_state);

int sn280x_set_boost_otg(int enable)
{
    int ret;
    struct sn280x_charger *chip;

    if (!sn280x_client)
        return -ENODEV;
    if (!enable)
        return 0;

    chip = i2c_get_clientdata(sn280x_client);
    if (!chip)
        pr_err("error: chip is null\n");
    dev_info(&chip->client->dev, "%s\n", __func__);

    if (!enable) {
        /* Boost OTG Disable */
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_REG_POWER_ON_CONFIGURATION,
                SN280X_MASK_CHARGER_CONFIGURATION,
                SN280X_VAL_CHARGER_CONFIGURATION_OTG_DISABLE);
        if (ret)
            dev_err(&chip->client->dev,
                "fail to boost otg disable\n");
        return ret;
    }

    ret = sn280x_dis_watchdog_timer_limit(chip);
    if (ret)
        return ret;

    /* Boost OTG Enabe */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_POWER_ON_CONFIGURATION,
            SN280X_MASK_CHARGER_CONFIGURATION,
            SN280X_VAL_CHARGER_CONFIGURATION_OTG);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to boost otg enable\n");
        return ret;
    }

    /* Boost OTG Max. Current: 1A */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_POWER_ON_CONFIGURATION,
            SN280X_MASK_BOOST_CURRENT_LIMIT,
            SN280X_VAL_BOOST_CURRENT_LIMIT_OTG);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to boost otg max current 1A\n");
        return ret;
    }

    /* Boost Voltage: 4.998V */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_BOOST_VOL_THERMAL_REGULATION,
            SN280X_MASK_BOOST_VOLTAGE,
            SN280X_VAL_BOOST_VOLTAGE_OTG);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to boost voltage 4.998V\n");
        return ret;
    }

    /* Status - OTG */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_SYSTEM_STATUS,
            SN280X_MASK_VBUS_STAT,
            SN280X_VAL_VBUS_STAT_OTG);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to set Vbus status OTG\n");
        return ret;
    }

    return ret;
}

static int sn280x_set_ac_chrgr_type(struct sn280x_charger *chip)
{
    int ret;
    dev_info(&chip->client->dev, "%s\n", __func__);

    ret = sn280x_dis_watchdog_timer_limit(chip);
    if (ret)
        return ret;

    /* Config max input current: 1200mA */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_INPUT_SOURCE_CONTROL,
            SN280X_MASK_INPUT_CURRENT_LIMIT,
            SN280X_VAL_INPUT_CURRENT_LIMIT_AC_IN);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to config max ac input current\n");
        return ret;
    }

    /* Config max input voltage: 4.36V */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_INPUT_SOURCE_CONTROL,
            SN280X_MASK_INPUT_VOLTAGE_LIMIT,
            SN280X_VAL_INPUT_VOLTAGE_LIMIT_AC_IN);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to config max ac input voltage\n");
        return ret;
    }

    /* Config charge voltage: 4.352V */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_CHARGE_VOLTAGE_CONTROL,
            SN280X_MASK_CHARGE_VOLTAGE_LIMIT,
            SN280X_VAL_CHARGE_VOLTAGE_LIMIT_AC_IN);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to config charge voltage\n");
        return ret;
    }

    /* Config fast charge current: 1280mA */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_CHARGE_CURRENT_CONTROL,
            SN280X_MASK_FAST_CHARGE_CURRENT_LIMIT,
            SN280X_VAL_FST_CHRG_CURRENT_LIMIT_AC_IN);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to config fast charge current\n");
        return ret;
    }

    /* Config termination current: 256mA */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_PRECHARGE_TERMINATION,
            SN280X_MASK_TERMINATION_CURRENT_LIMIT,
            SN280X_VAL_TERMI_CURRENT_LIMIT_AC_IN);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to config termination current\n");
        return ret;
    }

    /* Config status as ac adapter */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_SYSTEM_STATUS,
            SN280X_MASK_VBUS_STAT,
            SN280X_VAL_VBUS_STAT_AC_IN);
    if (ret) {
        dev_err(&chip->client->dev,
            "fail to config as ac adapter\n");
        return ret;
    }

    return ret;
}

static int sn280x_set_usb_chrgr_type(struct sn280x_charger *chip)
{
    int ret;
    dev_info(&chip->client->dev, "%s\n", __func__);

    /* PC usb charging */
    ret = sn280x_read_modify_reg(chip->client,
            SN280X_REG_SYSTEM_STATUS,
            SN280X_MASK_VBUS_STAT,
            SN280X_VAL_VBUS_STAT_USB_IN);
    if (ret)
        dev_err(&chip->client->dev,
            "fail to set PC usb charging\n");

    return ret;
}

int sn280x_set_chrgr_type(int usb_state)
{
    int ret;
    struct sn280x_charger *chip;

    if (!sn280x_client)
        return -ENODEV;

    chip = i2c_get_clientdata(sn280x_client);
    if (!chip)
        pr_err("error: chip is null\n");
    dev_info(&chip->client->dev, "%s\n", __func__);

    if (usb_state == AC_IN) {
        sn280x_set_ac_chrgr_type(chip);
    }
    else if (usb_state == USB_IN) {
        sn280x_set_usb_chrgr_type(chip);
    }

    return ret;
}

int sn280x_soc_control_jeita(void)
{
    int ret;
    struct sn280x_charger *chip;
    int batt_tempr;

    if (!sn280x_client)
        return -ENODEV;

    chip = i2c_get_clientdata(sn280x_client);
    if (!chip)
        pr_err("error: chip is null\n");
    dev_info(&chip->client->dev, "%s\n", __func__);

    ret = get_battery_temperature(&batt_tempr);
    if (ret) {
        dev_err(&chip->client->dev,
            "error: getting battery temperature!!\n");

        /* assign a default value */
        batt_tempr = 250;
    }

    /* control fast charge current limit */
    if (batt_tempr < 100) {
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_REG_CHARGE_CURRENT_CONTROL,
                SN280X_MASK_FAST_CHARGE_CURRENT_LIMIT,
                SN280X_JEITA_VAL_FST_CHRG_CURRENT_LIMIT_LOW_TEMP);
        if (ret) {
            dev_err(&chip->client->dev,
                "fail to config jieta: batt_tempr < 100\n");
            return ret;
        }
    }
    else {
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_REG_CHARGE_CURRENT_CONTROL,
                SN280X_MASK_FAST_CHARGE_CURRENT_LIMIT,
                SN280X_JEITA_VAL_FST_CHRG_CURRENT_LIMIT_HIGH_TEMP);
        if (ret) {
            dev_err(&chip->client->dev,
                "fail to config jieta: batt_tempr > 100\n");
            return ret;
        }
    }

    /* control charge voltage limit */
    if (batt_tempr < 500) {
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_REG_CHARGE_VOLTAGE_CONTROL,
                SN280X_MASK_CHARGE_VOLTAGE_LIMIT,
                SN280X_JEITA_VAL_CHARGE_VOLTAGE_LIMIT_LOW_TEMP);
        if (ret) {
            dev_err(&chip->client->dev,
                "fail to config jieta: batt_tempr < 500\n");
            return ret;
        }
    }
    else {
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_REG_CHARGE_VOLTAGE_CONTROL,
                SN280X_MASK_CHARGE_VOLTAGE_LIMIT,
                SN280X_JEITA_VAL_CHARGE_VOLTAGE_LIMIT_HIGH_TEMP);
        if (ret) {
            dev_err(&chip->client->dev,
                "fail to config jieta: batt_tempr > 500\n");
            return ret;
        }
    }

    /* control charger configuration */
    if (batt_tempr < 550 && batt_tempr > 0) {
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_REG_POWER_ON_CONFIGURATION,
                SN280X_MASK_CHARGER_CONFIGURATION_JEITA,
                SN280X_JEITA_VAL_CHARGER_CONFIG_ENABLE);
        if (ret) {
            dev_err(&chip->client->dev,
                "fail to config jieta: 0 < batt_tempr < 550\n");
            return ret;
        }
    }
    else {
        ret = sn280x_read_modify_reg(chip->client,
                SN280X_REG_POWER_ON_CONFIGURATION,
                SN280X_MASK_CHARGER_CONFIGURATION_JEITA,
                SN280X_JEITA_VAL_CHARGER_CONFIG_ENABLE);
        if (ret) {
            dev_err(&chip->client->dev,
                "fail to config jieta: out of 0 < batt_tempr < 550\n");
            return ret;
        }
    }
    dev_info(&chip->client->dev,
        "config JEITA with temperature: %d done\n", batt_tempr);

    return ret;
}

static int sn280x_probe(struct i2c_client *client,
             const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter;
    struct sn280x_charger *chip;
    int ret;
    enum sn280x_model_num sn280x_rev;

    adapter = to_i2c_adapter(client->dev.parent);

    if (!client->dev.platform_data) {
        dev_err(&client->dev, "platform data is null");
#ifdef CHRIS
        return -EFAULT;
#endif
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev,
            "I2C adapter %s doesn'tsupport BYTE DATA transfer\n",
            adapter->name);
        return -EIO;
    }

    ret = sn280x_read_reg(client, SN280X_VENDOR_REV_ADDR);
    if (ret < 0) {
        dev_err(&client->dev,
            "Error (%d) in reading SN280X_VENDOR_REV_ADDR\n", ret);
        return ret;
    }

#ifdef CHRIS
    sn280x_rev = sn280x_get_model(ret);
    if (((ret & SN280X_VENDOR_MASK) != SN280X_VENDOR) ||
        (sn280x_rev < 0)) {
        dev_err(&client->dev,
            "Invalid Vendor/Revision number in SN280X_VENDOR_REV_ADDR: %d",
            ret);
        return -ENODEV;
    }
#endif

    chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        dev_err(&client->dev, "mem alloc failed\n");
        return -ENOMEM;
    }

    init_waitqueue_head(&chip->wait_ready);
    i2c_set_clientdata(client, chip);
    chip->pdata = client->dev.platform_data;

    chip->client = client;
#ifdef CHRIS
    chip->pdata = client->dev.platform_data;
#else
    chip->pdata = sn280x_platform_data;
#endif

    chip->psy_usb.name = DEV_NAME;
    chip->psy_usb.type = POWER_SUPPLY_TYPE_USB;
    chip->psy_usb.properties = sn280x_usb_props;
    chip->psy_usb.num_properties = ARRAY_SIZE(sn280x_usb_props);
    chip->psy_usb.get_property = sn280x_usb_get_property;
    chip->psy_usb.set_property = sn280x_usb_set_property;
    chip->psy_usb.supplied_to = chip->pdata->supplied_to;
    chip->psy_usb.num_supplicants = chip->pdata->num_supplicants;
    chip->psy_usb.throttle_states = chip->pdata->throttle_states;
    chip->psy_usb.num_throttle_states = chip->pdata->num_throttle_states;
    chip->psy_usb.supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB;
    chip->max_cc = 1500;
    chip->chrgr_stat = SN280X_CHRGR_STAT_UNKNOWN;
    chip->chrgr_health = POWER_SUPPLY_HEALTH_UNKNOWN;

    strncpy(chip->model_name,
        sn280x_model_name[sn280x_rev].model_name,
        MODEL_NAME_SIZE);
    strncpy(chip->manufacturer, DEV_MANUFACTURER,
        DEV_MANUFACTURER_NAME_SIZE);

    mutex_init(&chip->lock);
#ifdef CHRIS
    ret = power_supply_register(&client->dev, &chip->psy_usb);
    if (ret) {
        dev_err(&client->dev, "Failed: power supply register (%d)\n",
            ret);
        return ret;
    }

    INIT_DELAYED_WORK(&chip->sw_term_work, sn280x_sw_charge_term_worker);
    INIT_DELAYED_WORK(&chip->low_supply_fault_work,
                sn280x_low_supply_fault_work);
    INIT_DELAYED_WORK(&chip->exception_mon_work,
                sn280x_exception_mon_work);
    INIT_DELAYED_WORK(&chip->wdt_work,
                sn280x_wdt_reset_worker);

    INIT_WORK(&chip->irq_work, sn280x_irq_worker);
    if (chip->client->irq) {
        ret = request_threaded_irq(chip->client->irq,
                       sn280x_irq_handler,
                       sn280x_thread_handler,
                       IRQF_SHARED|IRQF_NO_SUSPEND,
                       DEV_NAME, chip);
        if (ret) {
            dev_err(&client->dev, "Failed: request_irq (%d)\n",
                ret);
            power_supply_unregister(&chip->psy_usb);
            return ret;
        }
    }

    if (IS_BATTERY_OVER_VOLTAGE(chip))
        handle_battery_over_voltage(chip);
    else
        chip->bat_health = POWER_SUPPLY_HEALTH_GOOD;

    if (register_otg_notifications(chip))
        dev_err(&client->dev, "Error in registering OTG notifications. Unable to supply power to Host\n");
#endif

    ret = sn280x_register_power_supply(&client->dev);
    if (ret < 0)
        return ret;

    sn280x_client = client;
#ifdef CHRIS
    power_supply_changed(&chip->psy_usb);
#endif
    sn280x_debugfs_init();
    sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

    mutex_lock(&gusb_state_lock);
    dev_warn(&client->dev,
        "%s: config current when probe\n", __func__);
    sn280x_set_chrgr_type(gusb_state);
    if (gusb_state == USB_IN || gusb_state == AC_IN)
        sn280x_soc_control_jeita();
    mutex_unlock(&gusb_state_lock);

    sn280x_dump_regs(false);
    return 0;
}

static int sn280x_remove(struct i2c_client *client)
{
    struct sn280x_charger *chip = i2c_get_clientdata(client);

    if (client->irq)
        free_irq(client->irq, chip);

#ifdef CHRIS
    flush_scheduled_work();
#endif
    if (chip->transceiver)
        usb_unregister_notifier(chip->transceiver, &chip->otg_nb);

#ifdef CHRIS
    power_supply_unregister(&chip->psy_usb);
#endif
    sn280x_debugfs_exit();
    return 0;
}

static int sn280x_suspend(struct device *dev)
{
    struct sn280x_charger *chip = dev_get_drvdata(dev);

    dev_dbg(&chip->client->dev, "sn280x suspend\n");
    return 0;
}

static int sn280x_resume(struct device *dev)
{
    struct sn280x_charger *chip = dev_get_drvdata(dev);

    dev_dbg(&chip->client->dev, "sn280x resume\n");
    return 0;
}

static int sn280x_runtime_suspend(struct device *dev)
{
    dev_dbg(dev, "%s called\n", __func__);
    return 0;
}

static int sn280x_runtime_resume(struct device *dev)
{
    dev_dbg(dev, "%s called\n", __func__);
    return 0;
}

static int sn280x_runtime_idle(struct device *dev)
{

    dev_dbg(dev, "%s called\n", __func__);
    return 0;
}

static int sn280x_shutdown(struct i2c_client *client)
{
    dev_info(&client->dev, "%s\n", __func__);

    /* Disable OTG during shutdown */
    sn280x_set_boost_otg(0);

    /* registers dump */
    sn280x_dump_regs(false);

    return 0;
}

static const struct dev_pm_ops sn280x_pm_ops = {
    .suspend = sn280x_suspend,
    .resume = sn280x_resume,
    .runtime_suspend = sn280x_runtime_suspend,
    .runtime_resume = sn280x_runtime_resume,
    .runtime_idle = sn280x_runtime_idle,
};

static const struct i2c_device_id sn280x_id[] = {
    {DEV_NAME, 0},
    {},
};

MODULE_DEVICE_TABLE(i2c, sn280x_id);

static struct i2c_driver sn280x_driver = {
    .driver = {
           .name = DEV_NAME,
           .pm = &sn280x_pm_ops,
           },
    .probe = sn280x_probe,
    .remove = sn280x_remove,
    .shutdown    = sn280x_shutdown,
    .id_table = sn280x_id,
};

static int __init sn280x_init(void)
{
    if (entry_mode == 5) return -1;
    if (Read_HW_ID())
        return -EAGAIN;
    return i2c_add_driver(&sn280x_driver);
}

module_init(sn280x_init);

static void __exit sn280x_exit(void)
{
    i2c_del_driver(&sn280x_driver);
}

module_exit(sn280x_exit);

MODULE_AUTHOR("Jenny TC <jenny.tc@intel.com>");
MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_DESCRIPTION("SN280X Charger Driver");
MODULE_LICENSE("GPL");
