/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <asm/unaligned.h>
#include <asm/intel-mid.h>
#include <linux/interrupt.h>
#include <linux/idr.h>

#include "asus_battery.h"
#include "bq27520_battery_core.h"
#include "bq27520_battery_upt_i2c.h"
#include "bq27520_proc_fs.h"
#include "smb347_external_include.h"
#include "bq27520_proc_force_update.h"

#include <linux/power/bq27520_battery.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
//#include <linux/HWVersion.h>

/*
 *  config function
 */
#define BATTERY_LOW          0
#define ADC_ALERT_IRQ_INIT   0
#define ADC_ALERT_INTERRUPT  0

//extern int Read_HW_ID(void);
//static int HW_ID;
extern int entry_mode;

#define RETRY_COUNT 3

struct battery_dev_info bq27520_dev_info;
DEFINE_MUTEX(bq27520_dev_info_mutex);

#if BATTERY_LOW
static struct battery_low_config batlow ;
static struct asus_batgpio_set batlow_gp ;
static struct workqueue_struct *batlow_wq=NULL;
#endif

static struct work_struct batlow_work;
static struct dev_func bq27520_tbl;
static unsigned int  battery_ID = 0;// 0:valid 1:invalid
static unsigned int  battery_resist;
int  flash_gauge_status = 0;
EXPORT_SYMBOL(flash_gauge_status);

extern unsigned  get_usb_cable_status(void);

module_param(battery_ID , uint, 0644);
module_param(battery_resist , uint, 0644);
module_param(flash_gauge_status , int, 0644);


struct bq27520_chip {
    struct i2c_client *client;
    struct bq27520_platform_data *pdata;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend es;
#endif

    int batlow_irq;
    int adc_alert_irq;
};

bool isValidBattID(int battID) {
    if ((battID > MIN_102K_RESIST_REF) && (battID < MAX_102K_RESIST_REF)) {
          BAT_DBG("BattID is 102k\n");
          return true;
    } else if ((battID > MIN_76_5K_RESIST_REF) && (battID < MAX_76_5K_RESIST_REF)) {
          BAT_DBG("BattID is 76.5k\n");
          return true;
    } else {
          BAT_DBG("Invalid Batt ID\n");
          return false;
    }
}

extern struct battery_info_reply batt_info;
/*
 * i2c specific code
 */
static int bq27520_i2c_txsubcmd(struct i2c_client *client,
                u8 reg, unsigned short subcmd)
{
    struct i2c_msg msg;
    unsigned char data[3];

    if (!client || !client->adapter)
        return -ENODEV;

    memset(data, 0, sizeof(data));
    data[0] = reg;
    data[1] = subcmd & 0x00FF;
    data[2] = (subcmd & 0xFF00) >> 8;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 3;
    msg.buf = data;

    if (i2c_transfer(client->adapter, &msg, 1) < 0)
        return -EIO;

    return 0;
}

int bq27520_write_i2c(struct i2c_client *client,
                u8 reg, int value, int b_single)
{
    struct i2c_msg msg;
    unsigned char data[3];

    if (!client || !client->adapter)
        return -ENODEV;

    memset(data, 0, sizeof(data));
    data[0] = reg;
    data[1] = value& 0x00FF;
    data[2] = (value & 0xFF00) >> 8;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = b_single ? 2 : 3;
    msg.buf = data;

    if (i2c_transfer(client->adapter, &msg, 1) < 0)
    {
#ifdef ASUS_ENG_BUILD
        dev_err(&client->dev, "I2C write to 0x%02X error with code: %d\n",
            reg, -EIO);
#endif
        return -EIO;
    }

    return 0;
}

int bq27520_read_i2c(struct i2c_client *client,
                u8 reg, int *rt_value, int b_single)
{
    struct i2c_msg msg[1];
    unsigned char data[2];
    int err;

    if (!client || !client->adapter)
        return -ENODEV;

    msg->addr = client->addr;
    msg->flags = 0;
    msg->len = 1;
    msg->buf = data;

    data[0] = reg;
    err = i2c_transfer(client->adapter, msg, 1);

    if (err >= 0) {
        if (!b_single)
            msg->len = 2;
        else
            msg->len = 1;

        msg->flags = I2C_M_RD;
        err = i2c_transfer(client->adapter, msg, 1);
        if (err >= 0) {
            if (!b_single)
                *rt_value = get_unaligned_le16(data);
            else
                *rt_value = data[0];

            return 0;
        }
  }
#ifdef ASUS_ENG_BUILD
    dev_err(&client->dev, "I2C read from addr 0x%02X error with code: %d\n",
        reg, err);
#endif
  return err;
}

static inline int bq27520_read(struct i2c_client *client,
                u8 reg, int *rt_value, int b_single)
{
    return bq27520_read_i2c(client, reg, rt_value, b_single);
}

static int bq27520_cntl_cmd(struct i2c_client *client,
                u16 sub_cmd)
{
    return bq27520_i2c_txsubcmd(client, BQ27520_REG_CNTL, sub_cmd);
}

int bq27520_send_subcmd(struct i2c_client *client, int *rt_value, u16 sub_cmd)
{
    int ret, tmp_buf = 0;

    ret = bq27520_cntl_cmd(client, sub_cmd);
    if (ret) {
        dev_err(&client->dev, "Send subcommand 0x%04X error.\n", sub_cmd);
        return ret;
    }
    udelay(200);

    if (!rt_value) return ret;

    //need read data to rt_value
    ret = bq27520_read(client, BQ27520_REG_CNTL, &tmp_buf, 0);
    if (ret)
        dev_err(&client->dev, "Error!! %s subcommand %04X\n",
                         __func__, sub_cmd);
    *rt_value = tmp_buf;
    return ret;
}

int bq27520_cmp_i2c(int reg_off, int value)
{
    int retry = 3;
    int val=0;
    int ret=0;
    struct i2c_client *i2c = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    BAT_DBG_E("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(i2c, reg_off, &val, 1);
        if (ret < 0) continue;

        break;
    };
    if (!retry && ret < 0) {
        return ret;
    }

    return val == value ? PROC_TRUE : PROC_FALSE;
}

/*------------------------------------------------------------------------------*/

int TIgauge_i2c_write(struct i2c_client *client, u8 addr, int len, void *data)
{
    int i=0;
    int status=0;
    u8 buf[len + 1];
    int retries = 6;
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = len + 1,
            .buf = buf,
        },
    };

    if (!client || !client->adapter)
        return -ENODEV;

    buf[0] = addr;
    memcpy(buf + 1, data, len);

    do {
        status = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if ((status < 0) && (i < retries)) {
            msleep(5);
            dev_err(&client->dev, "%s: retry %d times\n", __func__, i);
            i++;
        }
    } while ((status < 0) && (i < retries));

    if (status < 0)
        dev_err(&client->dev, "%s: i2c write error %d\n", __func__, status);

    return status;
}

int TIgauge_i2c_read(struct i2c_client *client, u8 addr, int len, void *data)
{
    int i=0;
    int retries=6;
    int status=0;
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &addr,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = data,
        },
    };

    if (!client || !client->adapter)
        return -ENODEV;

    do {
        status = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
        if ((status < 0) && (i < retries)) {
            msleep(5);
            dev_err(&client->dev, "%s: retry %d times\n", __func__, i);
            i++;
        }
    } while ((status < 0) && (i < retries));

    if (status < 0)
        dev_err(&client->dev, "%s: i2c read error %d\n", __func__, status);

    return status;
}

void TIgauge_LockStep(void)
{
    int status;
    uint8_t i2cdata[32] = {0};

    struct i2c_client *i2c = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    dev_info(&i2c->dev, "%s enter\n", __func__);

    i2cdata[0] = 0x20;
    i2cdata[1] = 0x00;

    status = TIgauge_i2c_write(i2c, 0x00, 2, i2cdata);

    if (status < 0)
        dev_err(&i2c->dev, "%s: i2c write error %d\n", __func__, status);
}

/*------------------------------------------------------------------------------*/

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27520_battery_rsoc(struct i2c_client *client)
{
    int ret = 0, rsoc = 0;

    if (batt_info.gauge_version == IC_VERSION_G3) {
        ret = bq27520_read_i2c(client, BQ27520_REG_SOC_G3, &rsoc, 0);
    } else if (batt_info.gauge_version == IC_VERSION_G4) {
        ret = bq27520_read_i2c(client, BQ27520_REG_SOC_G4, &rsoc, 0);
    }

    if (ret)
        return ret;

    return rsoc;
}

int bq27520_asus_battery_dev_read_percentage(void)
{
    struct i2c_client *client = NULL;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_battery_rsoc(client);
    if (ret < 0) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev,"Reading battery percentage error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    mutex_unlock(&bq27520_dev_info_mutex);

    return ret;
}

int bq27520_asus_battery_dev_read_current(void)
{
    int ret;
    int curr;
    struct i2c_client *client = NULL;

    curr=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

  ret = bq27520_read_i2c(client, BQ27520_REG_AI, &curr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read current error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    //dev_info(&client->dev, ":: current %04X\n", curr);

    if (curr & BIT15)
        curr |= 0xFFFF0000; //neg. sign extend.

    curr += 0x10000; //add a base to be positive number.

    mutex_unlock(&bq27520_dev_info_mutex);

    return curr;
}

int bq27520_asus_battery_dev_read_flags(void)
{
    int ret;
    int flags=0;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_FLAGS, &flags, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read voltage error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    mutex_unlock(&bq27520_dev_info_mutex);

    return flags;
}

int bq27520_asus_battery_dev_read_volt(void)
{
    int ret;
    int volt=0;
    struct i2c_client *client = NULL;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_VOLT, &volt, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read voltage error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    //dev_info(&client->dev, ":: volt %04X\n", volt);
    mutex_unlock(&bq27520_dev_info_mutex);

    return volt;
}

int bq27520_asus_battery_dev_read_av_energy(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int mWhr=0;

    if (batt_info.gauge_version != IC_VERSION_G3) {
        BAT_DBG("Read available energy error due to not G3.\n");
        return ERROR_CODE_I2C_FAILURE;
    }
    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_AE_G3, &mWhr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read available energy error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    //dev_info(&client->dev, ":: av power %04X\n", mWhr);
    mutex_unlock(&bq27520_dev_info_mutex);
    return mWhr;
}

int bq27520_asus_battery_dev_read_temp(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int temp=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

  ret = bq27520_read(client, BQ27520_REG_TEMP, &temp, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read temperature error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
//        dev_info(&client->dev, ":: temperature %04X\n", temp);

    //tenths of K -> tenths of degree Celsius
    temp -= 2730;
    mutex_unlock(&bq27520_dev_info_mutex);

    return temp;
}

int bq27520_asus_battery_dev_read_remaining_capacity(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int mAhr=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_read_i2c(client, BQ27520_REG_RM, &mAhr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read remaining capacity error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    //dev_info(&client->dev, ":: av power %04X\n", mWhr);
    mutex_unlock(&bq27520_dev_info_mutex);
    return mAhr;
}

int bq27520_asus_battery_dev_read_full_charge_capacity(void)
{
    int ret;
    struct i2c_client *client = NULL;
    int mAhr=0;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

  ret = bq27520_read_i2c(client, BQ27520_REG_FCC, &mAhr, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read full charge capacity error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    //dev_info(&client->dev, ":: av power %04X\n", mWhr);
    mutex_unlock(&bq27520_dev_info_mutex);
    return mAhr;
}

int bq27520_asus_battery_dev_read_chemical_id(void)
{
    struct i2c_client *client = NULL;
    int chem_id=0;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_send_subcmd(client, &chem_id, BQ27520_SUBCMD_CHEM_ID);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read chemical ID error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    mutex_unlock(&bq27520_dev_info_mutex);

    return chem_id;
}

int bq27520_asus_battery_dev_read_fw_cfg_version(void)
{
    struct i2c_client *client = NULL;
    int fw_cfg_ver=0;
    int ret;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;

    ret = bq27520_write_i2c(client, 0x3F, 0x01, 1);
    if (ret) {
        dev_err(&client->dev, "Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    udelay(800); //delay awhile to get version. Otherwise version data not transfer complete
    ret = bq27520_read_i2c(client, 0x40, &fw_cfg_ver, 0);
    if (ret) {
        mutex_unlock(&bq27520_dev_info_mutex);
        dev_err(&client->dev, "Read fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    mutex_unlock(&bq27520_dev_info_mutex);

    return fw_cfg_ver;
}

static int __devexit bq27520_bat_i2c_remove(struct i2c_client *i2c)
{
    dev_info(&i2c->dev, "%s\n", __func__);
    return 0;
}

static int __devexit bq27520_bat_i2c_shutdown(struct i2c_client *i2c)
{
    dev_info(&i2c->dev, "%s\n", __func__);
#if 0
    batlow_disable_irq(&batlow_gp, &batlow);
    cancel_work_sync(&batlow_work);
#endif
    asus_battery_exit();
    return 0;
}

#ifdef CONFIG_PM
static int bq27520_bat_i2c_suspend(struct device *dev)
{
    struct bq27520_chip *chip = dev_get_drvdata(dev);

    printk("%s called\n", __func__);


    if (chip->batlow_irq != -1) {
        disable_irq(chip->batlow_irq);
        enable_irq_wake(chip->batlow_irq);
    }

#ifdef CONFIG_ADC_ALERT_GPIO_AS_WAKEUP
    if (chip->adc_alert_irq != -1) {
        disable_irq(chip->adc_alert_irq);
        enable_irq_wake(chip->adc_alert_irq);
    }
#endif

    return 0;
}

static int checkBatteryInvalidStatus(struct device *dev) {
    int   temperature;
    int   capacity;

    capacity = bq27520_asus_battery_dev_read_percentage();

    // 1. check if battery temperature is too high or too low.
    if (batt_info.cable_status == USB_ADAPTER) {
        temperature = bq27520_asus_battery_dev_read_temp();
        if (temperature >= 500 || temperature <= 0) {
             dev_info(dev, "=== battery is in critical temperatue with AC charger during sleep, stop charging !!!!!!, temperature = %d \n", temperature);
             smb347_charging_toggle(false);
        } else if (capacity > 98){
             dev_info(dev, "=== battery is nearly full capacity ======, capacity = %d\n", capacity);
             asus_queue_update_all();
        } else {
             dev_info(dev, "=== battery is in normal status ======, capacity = %d , temperature = %d\n", capacity, temperature);
             smb347_charging_toggle(true);
        }
    }

    // 2. check if battery capacity is too low, if yes notify system to do shutdown
    if (capacity < 5) {
         dev_info(dev, "===  battery capacity is too low in sleep!!!\n");
         asus_queue_update_all();
    }

}

static int bq27520_bat_i2c_resume(struct device *dev)
{
    struct bq27520_chip *chip = dev_get_drvdata(dev);

    printk("%s called\n", __func__);


    if (chip->batlow_irq != -1) {
        enable_irq(chip->batlow_irq);
        disable_irq_wake(chip->batlow_irq);
    }

#ifdef CONFIG_ADC_ALERT_GPIO_AS_WAKEUP
    if (chip->adc_alert_irq != -1) {
        enable_irq(chip->adc_alert_irq);
        disable_irq_wake(chip->adc_alert_irq);
    }
#endif

    checkBatteryInvalidStatus(dev);
    return 0;
}

#else
#define bq27520_bat_i2c_suspend NULL
#define bq27520_bat_i2c_resume  NULL
#endif

static int bq27520_chip_config(struct i2c_client *client)
{
    int flags = 0, ret = 0;

    ret = bq27520_send_subcmd(client, &flags, BQ27520_SUBCMD_CTNL_STATUS);
    if (ret)
        return ret;

    dev_info(&client->dev, "bq27520 flags: 0x%04X\n", flags);

    /*
     *  No need to send this command, battery will handle it.
    ret = bq27520_send_subcmd(client, NULL, BQ27520_SUBCMD_ENABLE_IT);
    if (ret)
        return ret;
     */

//  if (!(flags & BQ27520_CS_DLOGEN)) {
//    bq27520_send_subcmd(client, NULL, BQ27520_SUBCMD_ENABLE_DLOG);
//  }

  return 0;
}

static int bq27520_hw_config(struct i2c_client *client)
{
    int ret = 0;

    ret = bq27520_chip_config(client);
    if (ret) {
        dev_err(&client->dev, "Failed to config Bq27520 ret = %d\n", ret);
        return ret;
    }
    return ret;
}

struct info_entry
{
    char name[30];
    u16 cmd;
};

struct info_entry dump_subcmd_info_tbl[] =
{
    {"control status", BQ27520_SUBCMD_CTNL_STATUS},
    {"device type", BQ27520_SUBCMD_DEVICE_TYPE},
    {"firmware version", BQ27520_SUBCMD_FW_VER},
    {"chemical id", BQ27520_SUBCMD_CHEM_ID},
};

struct info_entry dump_info_tbl[] =
{
    {"Temperature", BQ27520_REG_TEMP},
    {"Voltage", BQ27520_REG_VOLT},
    {"Flags", BQ27520_REG_FLAGS},
    {"RemainingCapacity", BQ27520_REG_RM},
    {"AverageCurrent", BQ27520_REG_AI},
    {"AvailableEnergy", BQ27520_REG_AE_G3},
};

static char _chemical_id[5];
static char _fw_cfg_version[5];

static char bq27520_firmware_version[5];
static int bq27520_get_firmware_version(struct i2c_client *client)
{
    int ret;
    int tmp_buf=0;

    ret = bq27520_send_subcmd(client, &tmp_buf, BQ27520_SUBCMD_FW_VER);
    if (ret < 0)
        return ret;

    ret = sprintf(bq27520_firmware_version, "%04x", tmp_buf);
    batt_info.serial_number = bq27520_firmware_version;

    return 0;
}

static void bq27520_dump_info(struct i2c_client *client)
{
    u32 i;

    for (i=0; i<sizeof(dump_subcmd_info_tbl)/sizeof(struct info_entry); i++) {
        int tmp_buf=0, ret=0;

        ret = bq27520_send_subcmd(client,
                &tmp_buf, dump_subcmd_info_tbl[i].cmd);
        if (ret) continue;

        dev_info(&client->dev, "%s, 0x%04X\n", dump_subcmd_info_tbl[i].name, tmp_buf);
    }

    for (i=0; i<sizeof(dump_info_tbl)/sizeof(struct info_entry); i++) {
        int tmp_buf=0, ret=0;

        ret = bq27520_read(client,
                dump_info_tbl[i].cmd, &tmp_buf, 0);
        if (ret) continue;

        dev_info(&client->dev, "%s, 0x%04X\n", dump_info_tbl[i].name, tmp_buf);
    }
}

static int bq27520_thermistor_check(struct i2c_client *client)
{
    int temperature;

    dev_warn(&client->dev, "%s enter\n", __func__);

    temperature = bq27520_asus_battery_dev_read_temp();
    if (temperature == ERROR_CODE_I2C_FAILURE) {
        dev_err(&client->dev, "%s: read temperature error due to i2c error\n", __func__);
        return ERROR_CODE_I2C_FAILURE;
    }

    if (temperature < -400) {
        dev_err(&client->dev, "%s fail: temperature(%d) < -400(0.1C) \n", __func__, temperature);
        return -EINVAL;
    }

    return 0;
}

static irqreturn_t bq27520_batlow_interrupt(int irq, void *data)
{
    irqreturn_t ret = IRQ_NONE;
    struct bq27520_chip *chip = data;

    pm_runtime_get_sync(&chip->client->dev);

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    asus_queue_update_all();

    pm_runtime_put_sync(&chip->client->dev);

    ret = IRQ_HANDLED;
    return ret;
}

#if ADC_ALERT_INTERRUPT
static irqreturn_t bq27520_adc_alert_interrupt(int irq, void *data)
{
    irqreturn_t ret = IRQ_NONE;
    struct bq27520_chip *chip = data;

    pm_runtime_get_sync(&chip->client->dev);

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    pm_runtime_put_sync(&chip->client->dev);

    ret = IRQ_HANDLED;
    return ret;
}
#endif

#if ADC_ALERT_IRQ_INIT
static int bq27520_adc_alert_irq_init(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;
    int adc_alert_irq;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    ret = gpio_request_one(pdata->adc_alert, GPIOF_DIR_IN, "bq27520_adc_alert");
    if (ret < 0)
        goto fail;

    adc_alert_irq = gpio_to_irq(pdata->adc_alert);
    ret = request_threaded_irq(adc_alert_irq, NULL,
                    bq27520_adc_alert_interrupt,
                    IRQF_TRIGGER_FALLING,
                    chip->client->name,
                    chip);
    if (ret < 0)
        goto fail_gpio;

    chip->adc_alert_irq = adc_alert_irq;
    if (chip->adc_alert_irq == -1)
        goto fail_gpio;

    return 0;

fail_gpio:
    gpio_free(pdata->adc_alert);
fail:
    chip->adc_alert_irq = -1;
    return ret;
}
#endif

int bq27520_batlow_irq_init(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;
    int batlow_irq;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    ret = gpio_request_one(pdata->low_bat, GPIOF_DIR_IN, "bq27520_low_bat");
    if (ret < 0)
        goto fail;

    batlow_irq = gpio_to_irq(pdata->low_bat);

    ret = request_threaded_irq(batlow_irq, NULL,
                    bq27520_batlow_interrupt,
                    IRQF_TRIGGER_RISING,
                    chip->client->name,
                    chip);
    if (ret < 0)
        goto fail_gpio;


    chip->batlow_irq = batlow_irq;
    if (chip->batlow_irq == -1)
        goto fail_gpio;
    //enable_irq_wake(batlow_irq);

    return 0;

fail_gpio:
    gpio_free(pdata->low_bat);
fail:
    chip->batlow_irq = -1;
    return ret;
}

#if 0
static int disable_adc_alert(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    ret = gpio_request_one(pdata->adc_alert,
                            GPIOF_DIR_OUT | GPIOF_INIT_HIGH,
                            "bq27520_adc_alert");
    if (ret < 0)
        dev_err(&chip->client->dev, "%s: ERROR: fail to disable adc_alert gpio\n", __func__);

    return ret;
}
#endif

#if 0
static int bq27520_irq_init(struct bq27520_chip *chip)
{
    struct bq27520_platform_data *pdata = chip->pdata;
    int ret;

    dev_warn(&chip->client->dev, "%s enter\n", __func__);

    ret = bq27520_batlow_irq_init(chip);
    if (ret < 0)
        return ret;

#ifdef CONFIG_ADC_ALERT_GPIO_AS_WAKEUP
    ret = bq27520_adc_alert_irq_init(chip);
    if (ret < 0)
        return ret;
#else
    ret = disable_adc_alert(chip);
    if (ret < 0)
        return ret;
#endif

    return 0;
}
#endif

static void batlow_work_func(struct work_struct *work)
{
    int ret;

    BAT_DBG("%s enter\n", __func__);
    //notify battery low event
    ret = asus_battery_low_event();
    if (ret) {
        BAT_DBG("%s: battery low event error. %d \n", __func__, ret);
    }
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bq27520_early_suspend(struct early_suspend *h)
{
    struct bq27520_chip *chip = container_of(h, struct bq27520_chip, es);

    dev_info(&chip->client->dev, "enter %s\n", __func__);
}

static void bq27520_late_resume(struct early_suspend *h)
{
    struct bq27520_chip *chip = container_of(h, struct bq27520_chip, es);

    dev_info(&chip->client->dev, "enter %s\n", __func__);
    asus_queue_update_all();
}
#endif

//ME371MG only
#if 0
int bq27520_batt_current_sel_type(void)
{
    /* battery id gpio function */
    int bat_id_gpio;

    if (HW_ID_SR1 != HW_ID && HW_ID_EVB != HW_ID) {
        bat_id_gpio = get_gpio_by_name(BATTERY_CELL_ID_GPIO_NAME);
        if (gpio_get_value(bat_id_gpio)) {
            BAT_DBG(">>> BATTERY ID: LG (HIGH) <<<");
            batt_info.manufacturer = LG;
            return TYPE_LG;
        }
        else {
            BAT_DBG(">>> BATTERY ID: COSLIGHT (LOW) <<<");
            batt_info.manufacturer = COSLIGHT;
            return TYPE_COS_LIGHT;
        }
    }
    else {
        /* use default setting */
        BAT_DBG(">>> BATTERY ID: COSLIGHT (LOW) <<<");
        batt_info.manufacturer = COSLIGHT;
    }
    return TYPE_COS_LIGHT;
}
#endif

int bq27520_batt_fw_sel_type(void)
{
    int ret_val = 0;
    int cell_type = 0;

    ret_val = bq27520_asus_battery_dev_read_chemical_id();
    if (ret_val < 0) {
        BAT_DBG_E("[%s] Fail to get chemical_id\n", __func__);
        return -EINVAL;
    }

    if (ret_val == FW_CELL_TYPE_LG) {
        cell_type = TYPE_LG;

    } else if (ret_val == FW_CELL_TYPE_COS_LIGHT) {
        cell_type = TYPE_COS_LIGHT;

    } else {
        BAT_DBG_E("[%s] wrong chemical_id 0x%04X\n", __func__, ret_val);
        return -EINVAL;
    }

    return cell_type;
}

int bq27520_is_normal_mode()
{
    int retry = RETRY_COUNT;
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    BAT_DBG_E("[%s] enter \n", __func__);

    mutex_lock(&bq27520_dev_info_mutex);
    i2c = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

    while (retry--) {
        ret = bq27520_read_i2c(i2c, 0x00, &val, 1);
        if (ret < 0) continue;

        break;
    };
    if (ret < 0) {
        return 0; //not normal mode
    }
    return 1; //it's normal mode
}

static int __devinit
bq27520_bat_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    int chemicalID = 0;
    int __fw_cfg_version = 0;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct bq27520_chip *chip;
    int battID = batt_info.gpadc_resistance_val;
    battery_resist = battID;

    /* check if asus batt ID or not*/
    /*
    if (!isValidBattID(battID)) {
          batt_info.isValidBattID = false;
          battery_ID = 1;// 1:invalid
          flash_gauge_status = UPDATE_INVALID_BATTID;
          smb347_charging_toggle(false);
    //      return -EINVAL;
    }
    */

    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.status = DEV_INIT;
    mutex_unlock(&bq27520_dev_info_mutex);

    BAT_DBG("++++ %s ++++\n", __func__);


    if (!client->dev.platform_data) {
         BAT_DBG("Platform Data is NULL\n");
         return -EFAULT;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
         BAT_DBG("SM bus doesn't support DWORD transactions\n");
         return -EIO;
    }

    chip = kzalloc(sizeof(*chip), GFP_KERNEL);
    if (!chip) {
        BAT_DBG( "mem alloc failed\n");
        return -ENOMEM;
    }

    chip->client = client;
    chip->pdata = client->dev.platform_data;

    i2c_set_clientdata(client, chip);
    bq27520_dev_info.i2c = client;

//#ifdef ASUS_ENG_BUILD
    ret = bq27520_register_gauge_forceupdate_proc_fs();
    if (ret) {
       BAT_DBG_E("Unable to create bq27520_register_gauge_forceupdate_proc_fs\n");
    }
//#endif

    ret = bq27520_get_firmware_version(client);
    if (ret < 0) {
        BAT_DBG("bq27520_get_firmware_version failed\n");
        // get default set value
        if (batt_info.gauge_version == IC_VERSION_G3) {
           batt_info.serial_number = BQ27520_G3_VERSION;
        } else if (batt_info.gauge_version == IC_VERSION_G3) {
           batt_info.serial_number = BQ27520_G4_VERSION;
        } else {
           batt_info.serial_number = "unknown";
        }

    }

    if (strncmp(batt_info.serial_number, BQ27520_G3_VERSION, 4) == 0) {
        BAT_DBG("bq27520 gauge is G3\n");
        batt_info.gauge_version = IC_VERSION_G3;
    } else if (strncmp(batt_info.serial_number, BQ27520_G4_VERSION, 4) == 0) {
        BAT_DBG("bq27520 gauge is G4\n");
        batt_info.gauge_version = IC_VERSION_G4;
    }


//-----------------
#ifdef ASUS_ENG_BUILD
    BAT_DBG("only flash gauge firmware on eng mode.\n");
#if defined(CONFIG_ASUS_ENGINEER_MODE) && CONFIG_ASUS_ENGINEER_MODE
    /* Do it only in MOS, COS. Not support in other conditions */
    if (entry_mode == 1 || entry_mode == 4) {
        // check if is valid battID
        if (!batt_info.isValidBattID)  {
            BAT_DBG("invalid battID not update gauge IC!!!!\n");
            goto update_fail;
        }
        ret = bq27520_bat_upt_main_update_flow(false);
        mutex_lock(&bq27520_dev_info_mutex);
        bq27520_dev_info.update_status = ret;
        mutex_unlock(&bq27520_dev_info_mutex);
        if (ret < 0) {
            BAT_DBG("update gauge firmware fail !!!!\n");
            goto update_fail;
        }
        else if (ret == UPDATE_OK) { /* confirm "sealed" to acquire correct firmware config version */
            BAT_DBG("update gauge firmware success !!!!\n");
            msleep(500);
            TIgauge_LockStep();
        }
    }
#endif
#endif

    ret = bq27520_hw_config(client);
    if (ret < 0) {
        BAT_DBG("bq27520 hw config failed\n");
        return -EIO;
    }

    bq27520_dump_info(client);

    ret = bq27520_thermistor_check(client);
    if (ret < 0) {
        BAT_DBG("bq27520 thermistor check failed\n");
        return -EIO;
    }

    /* register power supply driver */
    bq27520_tbl.read_percentage = bq27520_asus_battery_dev_read_percentage;
    bq27520_tbl.read_current = bq27520_asus_battery_dev_read_current;
    bq27520_tbl.read_volt = bq27520_asus_battery_dev_read_volt;
    bq27520_tbl.read_av_energy = bq27520_asus_battery_dev_read_av_energy;

    bq27520_tbl.read_temp = bq27520_asus_battery_dev_read_temp;
    ret = asus_register_power_supply(&client->dev, &bq27520_tbl);
    if (ret)
        goto power_register_fail;

    if (chip->pdata->low_bat != -1) {
       if (gpio_get_value(chip->pdata->low_bat))
          dev_info(&client->dev, ">>> low_bat(%d):  High <<<\n", chip->pdata->low_bat);
       else
          dev_info(&client->dev, ">>> low_bat(%d):  LOW <<<\n", chip->pdata->low_bat);

    } else {
       dev_info(&client->dev, ">>>  intel firmware not config low_bat gpio  <<<<\n");
    }

    if (chip->pdata->adc_alert != -1) {
       if (gpio_get_value(chip->pdata->adc_alert))
          dev_info(&client->dev, ">>> adc_alert(%d):  High <<<\n", chip->pdata->adc_alert);
       else
          dev_info(&client->dev, ">>> adc_alert(%d):  LOW <<<\n", chip->pdata->adc_alert);
    } else {
       dev_info(&client->dev, ">>>  intel firmware not config adc_alert gpio  <<<<\n");
    }



    /* Init Runtime PM State */
    pm_runtime_put_noidle(&chip->client->dev);
    pm_schedule_suspend(&chip->client->dev, MSEC_PER_SEC);

    chip->batlow_irq = -1;
    chip->adc_alert_irq = -1;
#if 0
    ret = bq27520_irq_init(chip);
    if (ret) {
        dev_err(&client->dev, "bq27520 irq init: error\n");
        return ret;
    }
#endif

    INIT_WORK(&batlow_work,batlow_work_func);

    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.status = DEV_INIT_OK;
    mutex_unlock(&bq27520_dev_info_mutex);

//ME371MG only
#if 0
    /* battery id function */

    if (HW_ID_SR1 != HW_ID && HW_ID_EVB != HW_ID) {
        if (gpio_get_value(chip->pdata->bat_id_gpio)) {
            dev_info(&client->dev, ">>> BATTERY ID: LG (HIGH) <<<");
            batt_info.manufacturer = LG;
        }
        else {
            dev_info(&client->dev, ">>> BATTERY ID: COSLIGHT (LOW) <<<");
            batt_info.manufacturer = COSLIGHT;
        }
    }
    else {
        /* use default setting */
        dev_info(&client->dev, ">>> BATTERY ID: LG (HIGH) <<<");
        batt_info.manufacturer = LG;
    }
#endif
    batt_info.manufacturer = LG;

    batt_info.model = BQ27520_DEV_NAME;

#ifdef CONFIG_HAS_EARLYSUSPEND
    chip->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
    chip->es.suspend = bq27520_early_suspend;
    chip->es.resume = bq27520_late_resume;
    register_early_suspend(&chip->es);
#endif

    /* chemical id */
    chemicalID = bq27520_asus_battery_dev_read_chemical_id();
    if (chemicalID < 0) {
        BAT_DBG_E("[%s] Fail to get chemical_id\n", __func__);
        sprintf(_chemical_id, "%s", "xxxx");
    }
    else
        sprintf(_chemical_id, "%04x", chemicalID);
    batt_info.chemical_id = _chemical_id;

    /* firmware config version */
    __fw_cfg_version = bq27520_asus_battery_dev_read_fw_cfg_version();
    if (__fw_cfg_version < 0) {
        BAT_DBG_E("[%s] Fail to get firmware config version\n", __func__);
        sprintf((char *)__fw_cfg_version, "%s", "xxxx");
    }
    else
        sprintf(_fw_cfg_version, "%04x", __fw_cfg_version);
    batt_info.fw_cfg_version = _fw_cfg_version;

    dev_info(&client->dev, "%s done.", __func__);

power_register_fail:
#if defined(CONFIG_ASUS_ENGINEER_MODE) && CONFIG_ASUS_ENGINEER_MODE
update_fail:
#endif
    return ret;
}

#ifdef CONFIG_PM_RUNTIME
static int bq27520_runtime_suspend(struct device *dev)
{
  dev_dbg(dev, "%s called\n", __func__);
  return 0;
}

static int bq27520_runtime_resume(struct device *dev)
{
  dev_dbg(dev, "%s called\n", __func__);
  return 0;
}

static int bq27520_runtime_idle(struct device *dev)
{
  dev_dbg(dev, "%s called\n", __func__);
  return 0;
}
#else
#define bq27520_runtime_suspend NULL
#define bq27520_runtime_resume    NULL
#define bq27520_runtime_idle    NULL
#endif

static struct i2c_device_id bq27520_bat_i2c_id[] = {
    { BQ27520_DEV_NAME, 0 },
    { },
};

static const struct dev_pm_ops bq27520_pm_ops = {
    .suspend = bq27520_bat_i2c_suspend,
    .resume = bq27520_bat_i2c_resume,
    .runtime_suspend = bq27520_runtime_suspend,
    .runtime_resume = bq27520_runtime_resume,
    .runtime_idle = bq27520_runtime_idle,
};

static struct i2c_driver bq27520_bat_i2c_driver = {
    .driver    = {
        .name  = BQ27520_DEV_NAME,
        .owner = THIS_MODULE,
        .pm	= &bq27520_pm_ops,
    },
    .probe     = bq27520_bat_i2c_probe,
    .remove    = __devexit_p(bq27520_bat_i2c_remove),
    .id_table  = bq27520_bat_i2c_id,
};

static int __init bq27520_bat_i2c_init(void)
{
    int ret = 0;
    u32 test_major_flag = 0, test_minor_flag = 0;

    struct asus_bat_config bat_cfg;

    BAT_DBG("++++ %s ++++\n", __func__);

#if CONFIG_ASUS_BATTERY_BQ27541
    BAT_DBG(" use pack side gauge bq27541!!!\n", __func__);
    return -EAGAIN;
#endif

    //turn to jiffeys/s
    bat_cfg.polling_time = 0;
    bat_cfg.critical_polling_time = 0;
    bat_cfg.polling_time *= HZ;
    bat_cfg.critical_polling_time *= HZ;

    mutex_lock(&bq27520_dev_info_mutex);
    bq27520_dev_info.test_flag = test_minor_flag;
    mutex_unlock(&bq27520_dev_info_mutex);

#if defined(CONFIG_ASUS_ENGINEER_MODE) && CONFIG_ASUS_ENGINEER_MODE
    /* Do it only in MOS, COS. Not support in other conditions */
    if (entry_mode == 1 || entry_mode == 4) {
        ret = bq27520_bat_upt_i2c_init();
        if (ret)
            goto bq27520_upt_i2c_init_fail;
    }
#endif

#if CONFIG_PROC_FS
    ret = bq27520_register_proc_fs();
    if (ret) {
        BAT_DBG_E("Unable to create proc file\n");
        goto proc_fail;
    }
#endif

    //init battery info & work queue
    ret = asus_battery_init(bat_cfg.polling_time, bat_cfg.critical_polling_time, test_major_flag);
    if (ret)
        goto asus_battery_init_fail;

    //register i2c driver
    ret =  i2c_add_driver(&bq27520_bat_i2c_driver);
    if (ret) {
        BAT_DBG_E("register bq27520 battery i2c driver failed\n");
        goto i2c_register_driver_fail;
    }

    return ret;

i2c_register_driver_fail:
    asus_battery_exit();
asus_battery_init_fail:
#if defined(CONFIG_ASUS_ENGINEER_MODE)// && CONFIG_ASUS_ENGINEER_MODE
bq27520_upt_i2c_init_fail:
#endif
#if CONFIG_PROC_FS
proc_fail:
#endif

    return ret;
}
late_initcall(bq27520_bat_i2c_init);

static void __exit bq27520_bat_i2c_exit(void)
{
    struct i2c_client *client = NULL;

    asus_battery_exit();

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);

#if defined(CONFIG_ASUS_ENGINEER_MODE) && CONFIG_ASUS_ENGINEER_MODE
    /* Do it only in MOS, COS. Not support in other conditions */
    if (entry_mode == 1 || entry_mode == 4)
        bq27520_bat_upt_i2c_exit();
#endif
    i2c_unregister_device(bq27520_dev_info.i2c);
    i2c_del_driver(&bq27520_bat_i2c_driver);

    BAT_DBG("%s exit\n", __func__);
}
module_exit(bq27520_bat_i2c_exit);

MODULE_AUTHOR("chris1_chang@asus.com");
MODULE_DESCRIPTION("battery bq27520 driver");
MODULE_LICENSE("GPL");
