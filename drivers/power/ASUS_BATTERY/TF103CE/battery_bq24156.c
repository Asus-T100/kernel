/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom_Shen Tom_Shen@asus.com
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>

#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/usb/penwell_otg.h>
//#include "../../intel_mdf_charger.h"
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>
static int HW_ID;
extern int Read_HW_ID(void);
extern int entry_mode;

enum {
    USB_IN_bq24156,
    AC_IN_bq24156,
    CABLE_OUT_bq24156,
};

typedef enum {
    CHARGER_BATTERY_bq24156 = 0,
    CHARGER_AC_bq24156,
    CHARGER_USB_bq24156
} charger_type_t;

/* I2C communication related */
#define I2C_RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

struct bq24156_charger {
    struct mutex        lock;
    struct i2c_client    *client;
    struct power_supply    mains;
    struct power_supply    usb;
    struct power_supply    battery;
    bool            mains_online;
    bool            usb_online;
    bool            charging_enabled;
    bool            running;
    struct dentry        *dentry;
    struct dentry        *dentry2;
    struct otg_transceiver    *otg;
    struct notifier_block    otg_nb;
    struct work_struct    otg_work;
    struct delayed_work aicl_dete_work;
    struct workqueue_struct *chrgr_work_queue;
    struct list_head    otg_queue;
    spinlock_t        otg_queue_lock;
    bool            otg_enabled;
    bool            otg_battery_uv;
    const struct bq24156_charger_platform_data    *pdata;
    /* wake lock to prevent S3 during charging */
    struct wake_lock wakelock;
};
#ifdef CONFIG_UPI_BATTERY
struct wake_lock mwlock;
struct wake_lock mmwlock_t;
#endif

/* global charger type variable lock */
DEFINE_MUTEX(m_usb_state_lock);
static int g_usb_state = CABLE_OUT_bq24156;
/* global software charging toggle lock */
DEFINE_MUTEX(m_charging_toggle_lock);
static bool g_charging_toggle = true;

static struct bq24156_charger *bq24156_dev;
struct workqueue_struct *battery_poll_work_queue = NULL;
struct delayed_work status_poll_work;

#define DEBUG 1
#define DRIVER_VERSION            "1.1.0"
#define GPIO_INOK                 44
#define BYTETOBINARYPATTERN "%d%d%d%d-%d%d%d%db"
#define BYTETOBINARY(byte) \
	(byte & 0x80 ? 1 : 0), \
	(byte & 0x40 ? 1 : 0), \
	(byte & 0x20 ? 1 : 0), \
	(byte & 0x10 ? 1 : 0), \
	(byte & 0x08 ? 1 : 0), \
	(byte & 0x04 ? 1 : 0), \
	(byte & 0x02 ? 1 : 0), \
	(byte & 0x01 ? 1 : 0)

static char *supply_list[] = {
    "battery",
};

static enum power_supply_property asus_power_properties[] = {
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_ONLINE,
};

static int bq24156_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val);

static struct power_supply bq24156_power_supplies[] = {
    {
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = asus_power_properties,
        .num_properties = ARRAY_SIZE(asus_power_properties),
        .get_property = bq24156_power_get_property,
    },
    {
        .name = "usb",
        .type = POWER_SUPPLY_TYPE_USB,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = asus_power_properties,
        .num_properties = ARRAY_SIZE(asus_power_properties),
        .get_property = bq24156_power_get_property,
    },
};

static int bq24156_power_get_property(struct power_supply *psy,
                    enum power_supply_property psp,
                    union power_supply_propval *val) {
    int ret=0;
    int usb_state;

    mutex_lock(&m_usb_state_lock);
    usb_state = g_usb_state;
    mutex_unlock(&m_usb_state_lock);

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_USB) {
            val->intval = (usb_state == USB_IN_bq24156) ? 1 : 0;
        } else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
            val->intval = (usb_state == AC_IN_bq24156) ? 1 : 0;
        } else {
            ret = -EINVAL;
        }
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS) {
            if ( usb_state == USB_IN_bq24156 || (usb_state == AC_IN_bq24156))
                val->intval = 1;
            else
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

int get_bq24156_charger_type() {
    return g_usb_state;
}

EXPORT_SYMBOL(get_bq24156_charger_type);

void charger_power_supply_changed() {

    if (!bq24156_dev) {
        pr_err("Warning: bq24156_dev is null due to probe function has error\n");
        return;
    }

    power_supply_changed(&bq24156_power_supplies[CHARGER_AC_bq24156-1]);
    power_supply_changed(&bq24156_power_supplies[CHARGER_USB_bq24156-1]);

    return ;
}

EXPORT_SYMBOL(charger_power_supply_changed);

static int bq24156_register_power_supply(struct device *dev) {
    int ret;

    ret = power_supply_register(dev, &bq24156_power_supplies[CHARGER_AC_bq24156-1]);
    if (ret) {
        printk("Fail to register power supply AC\n");
        goto batt_err_reg_fail_ac;
    }

    ret = power_supply_register(dev, &bq24156_power_supplies[CHARGER_USB_bq24156-1]);
    if (ret) {
        printk("Fail to register power supply USB\n");
        goto batt_err_reg_fail_usb;
    }

    return 0;
batt_err_reg_fail_usb:
    power_supply_unregister(&bq24156_power_supplies[CHARGER_USB_bq24156-1]);
batt_err_reg_fail_ac:
    power_supply_unregister(&bq24156_power_supplies[CHARGER_AC_bq24156-1]);

    return ret;
}

//==============
static int bq2415x_write_block(struct bq24156_charger *smb, u8 *value,
                                                u8 reg, unsigned num_bytes)
{
        struct i2c_msg msg[1];
        int ret;

        *value          = reg;

        msg[0].addr     = smb->client->addr;
        msg[0].flags    = 0;
        msg[0].buf      = value;
        msg[0].len      = num_bytes + 1;

        ret = i2c_transfer(smb->client->adapter, msg, 1);

        if (ret < 0) {
                        printk(
                        "bq2415x_read_block = %d \n",ret);
                        return ret;
        }
        /* i2c_transfer returns number of messages transferred */
        if (ret != 1) {
                printk(
                        "i2c_write failed to transfer all messages\n");
                if (ret < 0)
                        return ret;
                else
                        return -EIO;
        } else {
                return 0;
        }
}

static int bq2415x_read_block(struct bq24156_charger *smb, u8 *value,
                                                u8 reg, unsigned num_bytes)
{
        struct i2c_msg msg[2];
        u8 buf;
        int ret;

        buf             = reg;

        msg[0].addr     = smb->client->addr;
        msg[0].flags    = 0;
        msg[0].buf      = &buf;
        msg[0].len      = 1;

        msg[1].addr     = smb->client->addr;
        msg[1].flags    = I2C_M_RD;
        msg[1].buf      = value;
        msg[1].len      = num_bytes;

        ret = i2c_transfer(smb->client->adapter, msg, 2);

        /* i2c_transfer returns number of messages transferred */
        if (ret < 0) {
                        printk(
                        "bq2415x_read_block = %d \n",ret);
                        return ret;
        }
        if (ret != 2) {
                printk(
                        "i2c_write failed to transfer all messages\n");
                if (ret < 0)
                        return ret;
                else
                        return -EIO;
        } else {
                return 0;
        }
}

static int bq2415x_write_byte(struct bq24156_charger *smb, u8 value, u8 reg)
{
        /* 2 bytes offset 1 contains the data offset 0 is used by i2c_write */
        u8 temp_buffer[2] = { 0 };

        /* offset 1 contains the data */
        temp_buffer[1] = value;
        return bq2415x_write_block(smb, temp_buffer, reg, 1);
}

static int bq2415x_read_byte(struct bq24156_charger *smb, u8 *value, u8 reg)
{
        return bq2415x_read_block(smb, value, reg, 1);
}

//==============
static int bq24156_read(struct bq24156_charger *smb, u8 reg) {
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do {
        ret = i2c_smbus_read_byte_data(smb->client, reg);
        if (ret < 0) {
            retry_count--;
            dev_warn(&smb->client->dev, "fail to read reg %02xh: %d\n", reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

static int bq24156_write(struct bq24156_charger *smb, u8 reg, u8 val) {
    int ret;
    int retry_count = I2C_RETRY_COUNT;

    do {
        ret = i2c_smbus_write_byte_data(smb->client, reg, val);
        if (ret < 0) {
            retry_count--;
            dev_warn(&smb->client->dev, "fail to write reg %02xh: %d\n", reg, ret);
            msleep(I2C_RETRY_DELAY);
        }
    } while (ret < 0 && retry_count > 0);

    return ret;
}

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret = 0;

    if (!bq24156_dev) {
        pr_info("%s: ERROR: bq24156_dev is null due to probe function has error\n",  __func__);
        return sprintf(buf, "%d\n", -EINVAL);
    }

    if (g_usb_state == USB_IN_bq24156 ) {
          ret = 500;
    } else if (g_usb_state == AC_IN_bq24156) {
          ret = 1250;
    }
    return sprintf(buf, "%d\n", ret);
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
                    struct device_attribute *attr, char *buf) {
    int ret = 0, status;

    if (g_usb_state == USB_IN_bq24156 || g_usb_state == AC_IN_bq24156) {
          ret = 1;
    }

    return sprintf(buf, "%d\n", ret);
}


static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
    &dev_attr_input_current.attr,
    &dev_attr_charge_status.attr,
    NULL,
};
static struct attribute_group dev_attr_grp = {
    .attrs = dev_attrs,
};
/*----------------------------------------------------------------------------*/

static void bq24156_config_max_current(int usb_state) {
    int status;
    u8 ret;

    if (!bq24156_dev) {
        pr_err("%s: bq24156_dev is null due to driver probed isn't ready\n", __func__);
        return;
    }

    /* 0. every 32s reset safety timer  */
    status = bq2415x_read_byte(bq24156_dev, &ret,0x00);
    if (status < 0)
        goto out;

    ret |= BIT(7);
    //printk("write 0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x00, BYTETOBINARY(ret));
    status = bq2415x_write_byte(bq24156_dev, ret,0x00);
    if (status < 0)
        goto out;
    //ret = bq24156_read(bq24156_dev, 0x00);
    //printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x00, BYTETOBINARY(ret));


    /* 1. Max. charger Current = 1250mA ( 06h[7:4] = "0111") 2. Max. charger Voltage = 4.36V ( 06h[3:0] = "1000")   */
    status = bq2415x_read_byte(bq24156_dev, &ret,0x06);
    if (status < 0)
        goto out;
    
    ret &= ~ ((BIT(7))|BIT(0)|BIT(1)|BIT(2));
    ret |= BIT(3)|BIT(4)|BIT(5)|BIT(6);
    //printk("write 0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x06, BYTETOBINARY(ret));
    status = bq2415x_write_byte(bq24156_dev, ret,0x06);
    if (status < 0)
        goto out;
    //ret = bq24156_read(bq24156_dev, 0x06);
    //printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x06, BYTETOBINARY(ret));

    /* reg 5h BIT5:0 Normal charge current sense volatge at 04H   */
    status = bq2415x_read_byte(bq24156_dev, &ret,0x05);
    if (status < 0)
        goto out;

    ret &= ~ (BIT(5));
    //printk("write 0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x05, BYTETOBINARY(ret));
    status = bq2415x_write_byte(bq24156_dev, ret,0x05);
    if (status < 0)
        goto out;
    //ret = bq24156_read(bq24156_dev, 0x05);
    //printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x05, BYTETOBINARY(ret));

    /* 3. Input Current No Limit-AC_IN_bq24156 (01h[7:6] = "11"), USB 500mA-USB_IN_bq24156 (01h[7:6] = "01")  */
    status = bq2415x_read_byte(bq24156_dev, &ret,0x01);
    if (status < 0)
        goto out;

    if (usb_state == USB_IN_bq24156) {
          ret &= ~(BIT(7));
          ret |= BIT(6);
    } else if (usb_state == AC_IN_bq24156) {
          ret |= BIT(6)|BIT(7);
    }

    //printk("write 0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x01, BYTETOBINARY(ret));
    status = bq2415x_write_byte(bq24156_dev, ret,0x01);
    if (status < 0)
        goto out;
    //ret = bq24156_read(bq24156_dev, 0x01);
    //printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x01, BYTETOBINARY(ret));

    /* 4. charger Voltage = 4.36V ( 02h[7:2] = "101011")   */
    status = bq2415x_read_byte(bq24156_dev, &ret,0x02);
    if (status < 0)
        goto out;

    ret &= ~ ((BIT(6))|BIT(4));
    ret |= BIT(2)|BIT(3)|BIT(5)|BIT(7);

    //printk("write 0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x02, BYTETOBINARY(ret));
    status = bq2415x_write_byte(bq24156_dev, ret,0x02);
    if (status < 0)
        goto out;
    //ret = bq24156_read(bq24156_dev, 0x02);
    //printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x02, BYTETOBINARY(ret));

    /* 5 charger Current = 1250mA ( 04h[6:3] = "0111"), 6 Terminal Current = 200mA ( 04h[2:0] = "011")   */
    status = bq2415x_read_byte(bq24156_dev, &ret,0x04);
    if (status < 0)
        goto out;

    ret &= ~((BIT(6))|BIT(2));
    ret |= BIT(0)|BIT(1)|BIT(3)|BIT(4)|BIT(5);

    //printk("write 0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x04, BYTETOBINARY(ret));
    status = bq2415x_write_byte(bq24156_dev, ret,0x04);
    if (status < 0)
        goto out;
    //ret = bq24156_read(bq24156_dev, 0x04);
    // printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x04, BYTETOBINARY(ret));

    /* 7.Enable Charge Current Term (01h[3] = 1)  */
    status = bq2415x_read_byte(bq24156_dev, &ret,0x01);
    if (status < 0)
        goto out;

    ret |= BIT(3);
    //printk("write 0x%02x:\t" BYTETOBINARYPATTERN "\n", 0x01, BYTETOBINARY(ret));
    status = bq2415x_write_byte(bq24156_dev, ret,0x01);
    if (status < 0)
        goto out;

    pr_info("%s: charger type:%d done.\n", __func__, usb_state);
out:
    return;
}

#if 0
static void bq24156_config_max_current(int usb_state) {
    int ret;
    if (usb_state != AC_IN_bq24156 && usb_state != USB_IN_bq24156)
        return;

    if (!bq24156_dev) {
        pr_err("%s: bq24156_dev is null due to driver probed isn't ready\n", __func__);
        return;
    }

    /* 3. Input Current No Limit-AC_IN_bq24156 (01h[7:6] = "11"), USB 500mA-USB_IN_bq24156 (01h[7:6] = "01")  */ 
    ret = bq24156_read(bq24156_dev, 0x01);
    if (ret < 0)
        goto out;
    
    // Enable Charge Current Term (01h[3] = 1)
    ret |= BIT(3);
    if (usb_state == USB_IN_bq24156) {
          ret &= ~(BIT(7));
          ret |= BIT(6);
    } else if (usb_state == AC_IN_bq24156) {
          ret |= BIT(6)|BIT(7);
    }
    ret = bq24156_write(bq24156_dev, 0x01, ret);
    if (ret < 0)
        goto out;


    /* 1. Max. charger Current = 1250mA ( 06h[7:4] = "0111") 2. Max. charger Voltage = 4.36V ( 06h[3:0] = "1000")   */
    ret = bq24156_read(bq24156_dev, 0x06);
    if (ret < 0)
        goto out;

    ret &= ~ ((BIT(7))|BIT(0)|BIT(1)|BIT(2));
    ret |= BIT(3)|BIT(4)|BIT(5)|BIT(6);

    ret = bq24156_write(bq24156_dev, 0x06, ret);
    if (ret < 0)
        goto out;


    /* 4. charger Voltage = 4.36V ( 02h[7:2] = "101011")   */
    ret = bq24156_read(bq24156_dev, 0x02);
    if (ret < 0)
        goto out;
    
    ret &= ~ ((BIT(6))|BIT(4));
    ret |= BIT(2)|BIT(3)|BIT(5)|BIT(7);

    ret = bq24156_write(bq24156_dev, 0x02, ret);
    if (ret < 0)
        goto out;


    /* 5 charger Current = 1250mA ( 04h[6:3] = "0111"), 6 Terminal Current = 200mA ( 04h[2:0] = "011")   */
    ret = bq24156_read(bq24156_dev, 0x04);
    if (ret < 0)
        goto out;
    
    ret &= ~((BIT(6))|BIT(2));
    ret |= BIT(0)|BIT(1)|BIT(3)|BIT(4)|BIT(5);

    ret = bq24156_write(bq24156_dev, 0x04, ret);
    if (ret < 0)
        goto out;

    /* 6  reset safety timer  */
    ret = bq24156_read(bq24156_dev, 0x00);
    if (ret < 0)
        goto out;
    
    ret |= BIT(7);

    ret = bq24156_write(bq24156_dev, 0x00, ret);
    if (ret < 0)
        goto out;
    pr_info("%s: charger type:%d done.\n", __func__, usb_state);
out:
    return;
}
#endif

int setbq24156Charger(int usb_state) {
    int ret = 0;
    g_usb_state = usb_state;

    if (bq24156_dev) {
        switch (usb_state){
        case USB_IN_bq24156:
            power_supply_changed(&bq24156_power_supplies[CHARGER_USB_bq24156-1]);
            break;
        case AC_IN_bq24156:
            power_supply_changed(&bq24156_power_supplies[CHARGER_AC_bq24156-1]);
            break;
        case CABLE_OUT_bq24156:
            power_supply_changed(&bq24156_power_supplies[CHARGER_AC_bq24156-1]);
            power_supply_changed(&bq24156_power_supplies[CHARGER_USB_bq24156-1]);
            break;
        }
    }

    switch (usb_state) {
    case USB_IN_bq24156:
        printk(" usb_state: USB_IN on bq24156\n");
        if (bq24156_dev) {
            mutex_lock(&m_usb_state_lock);
            bq24156_config_max_current(USB_IN_bq24156);
            mutex_unlock(&m_usb_state_lock);
        }
        break;
    case AC_IN_bq24156:
        printk(" usb_state: AC_IN on bq24156\n");
        if (bq24156_dev) {
            mutex_lock(&m_usb_state_lock);
            bq24156_config_max_current(AC_IN_bq24156);
            mutex_unlock(&m_usb_state_lock);
        }

#ifdef CONFIG_UPI_BATTERY
        if (bq24156_dev) {
                if (!wake_lock_active(&mwlock)) {
                    printk(" %s: asus_battery_power_wakelock -> wake lock\n",  __func__);
                    wake_lock(&mwlock);
                }
        }
#endif
        break;
    case CABLE_OUT_bq24156:
        printk(" usb_state: CABLE_OUT on bq24156\n");
#ifdef CONFIG_UPI_BATTERY
        if (bq24156_dev) {
                printk(" %s: asus_battery_power_wakelock-> wake unlock\n",  __func__);
                wake_unlock(&mwlock);
        }
#endif
        break;
    default:
        printk(" ERROR: wrong usb state value = %d\n", usb_state);
        ret = 1;
    }

    return ret;
}
EXPORT_SYMBOL(setbq24156Charger);

/* write 01h[2]="1" disable or "0" enable */
int bq24156_charging_toggle(bool on) {
    int ret = 0;

    if (!bq24156_dev) {
        pr_info("Warning: bq24156_dev is null due to probe function has error\n");
        return 1;
    }

    if (!on) pr_warn("%s: *** charging toggle: %s ***\n", __func__, on ? "ON" : "OFF");

    /* Config CFG_PIN register */
    ret = bq24156_read(bq24156_dev, 0x01);
    if (ret < 0)
        goto out;
    
    if (on) {
        ret &= ~(BIT(2));
    } else {
        ret |= BIT(2);
    }

    ret = bq24156_write(bq24156_dev, 0x01, ret);
    if (ret < 0)
        goto out;

    mutex_lock(&m_charging_toggle_lock);
    g_charging_toggle = on;
    mutex_unlock(&m_charging_toggle_lock);

out:
    return ret;
}

static irqreturn_t bq24156_inok_interrupt(int irq, void *data) {
    struct bq24156_charger *smb = data;

    /* wake lock to prevent system instantly enter S3 while it's in resuming flow */
    wake_lock_timeout(&smb->wakelock, HZ);

    pm_runtime_get_sync(&smb->client->dev);

    if (gpio_get_value(GPIO_INOK)) {
        dev_warn(&smb->client->dev, "%s: >>> INOK pin (HIGH) <<<\n", __func__);

        mutex_lock(&m_usb_state_lock);
        g_usb_state = CABLE_OUT_bq24156;
        mutex_unlock(&m_usb_state_lock);
    } else {
        dev_warn(&smb->client->dev, "%s: >>> INOK pin (LOW) <<<\n",  __func__);
    }

    pm_runtime_put_sync(&smb->client->dev);
    return IRQ_HANDLED;
}

static int bq24156_inok_gpio_init(struct bq24156_charger *smb) {
    int ret, irq = gpio_to_irq(GPIO_INOK);

    if (gpio_get_value(GPIO_INOK))
        printk(">>> INOK (HIGH) <<<\n");
    else
        printk(">>> INOK (LOW) <<<\n");

    ret = gpio_request_one(GPIO_INOK, GPIOF_IN, "bq24156_inok");
    if (ret < 0) {
        printk("%s: request INOK gpio fail!\n", __func__);
        goto fail;
    }

    ret = request_threaded_irq(irq, NULL, bq24156_inok_interrupt,
                    IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
                    IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                    smb->client->name,
                    smb);

    if (ret < 0) {
        printk("%s: config INOK gpio as IRQ fail!\n", __func__);
        goto fail_gpio;
    }

    return 0;
fail_gpio:
    gpio_free(GPIO_INOK);
fail:
    smb->client->irq = 0;
    return ret;
}

int bq24156_dump_registers(struct seq_file *s) {
    struct bq24156_charger *smb;
    int ret;
    u8 reg;

    if (s) {
        smb = s->private;
    } else {
        if (!bq24156_dev) {
            printk(" %s: bq24156_dev is null!\n", __func__);
            return -1;
        } else {
            smb = bq24156_dev;
        }
    }

    printk(" %s:\n", __func__);
    printk(" Simple bq24156 registers:\n");
    printk(" ==================\n");
    printk(" #Addr\t#Value\n");

    for (reg = 0; reg <= 6; reg++) {
        ret = bq24156_read(smb, reg);
        printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
        if (s)
            seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
    }
    return 0;
}


static int bq24156_debugfs_show(struct seq_file *s, void *data) {
    bq24156_dump_registers(s);
    return 0;
}

static struct power_supply *get_battery_psy(void) {
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

static int get_battery_capacity(int *rsoc) {
    struct power_supply *psy;
    union power_supply_propval val;
    int ret;

    psy = get_battery_psy();
    if (!psy)
        return -EINVAL;

    ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
    if (!ret)
        *rsoc = val.intval;

    return ret;
}

static void charger_status_poll(struct work_struct *work) {
    int rsoc;
    int usb_state;
    int ret;

    if (!bq24156_dev) {
        pr_info("Warning: bq24156_dev is null due to probe function has error\n");
        return;
    }

    mutex_lock(&m_usb_state_lock);
    usb_state = g_usb_state;
    mutex_unlock(&m_usb_state_lock);

#if defined(FE170CG_ENG_BUILD) && defined(CONFIG_UPI_BATTERY)

   if ( usb_state == USB_IN_bq24156 || (usb_state == AC_IN_bq24156)) {
        pr_info("ENG mode, every miniute query charging status USB state = %d\n", usb_state);
        /* acquire battery rsoc here */
        if (get_battery_capacity(&rsoc)) {
            pr_info(" %s: fail to get battery rsoc\n", __func__);
        } else {
            pr_info("ENG mode, capacity is %d in charging!\n", rsoc);
            if (rsoc > 59) {
                pr_info("ENG mode, capacity is more than 60 stop charging!!!!\n");
                bq24156_charging_toggle(false);
                charger_power_supply_changed();
                goto out;
            } else {
                bq24156_charging_toggle(true);
            }
        }
   }
#endif

    if ( usb_state == USB_IN_bq24156 || (usb_state == AC_IN_bq24156)) {
        pr_info("=====  bq24156 charger every 10s reset safety timer, avoid charger IC become to default setting  =====\n");
        bq24156_config_max_current(usb_state);
    }

out:
    queue_delayed_work(battery_poll_work_queue, &status_poll_work, 10 * HZ);
}

static int bq24156_debugfs_open(struct inode *inode, struct file *file) {
    return single_open(file, bq24156_debugfs_show, inode->i_private);
}

static const struct file_operations bq24156_debugfs_fops = {
    .open        = bq24156_debugfs_open,
    .read        = seq_read,
    .llseek      = seq_lseek,
    .release     = single_release,
};

static int bq24156_probe(struct i2c_client *client, const struct i2c_device_id *id) {

    struct device *dev = &client->dev;
    struct bq24156_charger *smb;
    int ret;
    int charger_type;

    printk(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);

    smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
    if (!smb)
        return -ENOMEM;

    i2c_set_clientdata(client, smb);

    smb->client = client;
    smb->pdata = dev->platform_data;


#ifdef CONFIG_UPI_BATTERY
    /* init wake lock in COS */
    if (entry_mode == 4) {
        printk(" %s: wake lock init: asus_battery_power_wakelock\n", __func__);
        wake_lock_init(&mwlock, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock");
        wake_lock_init(&mmwlock_t, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock_timeout");

        /* prevent system from entering s3 in COS
            while AC charger is connected
        */
        mutex_lock(&m_usb_state_lock);
        charger_type = g_usb_state;
        mutex_unlock(&m_usb_state_lock);

        if (charger_type == AC_IN_bq24156) {
            if (!wake_lock_active(&mwlock)) {
                printk(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                wake_lock(&mwlock);
            }
        }
    }
#endif
    bq24156_dev = smb;

    /* Init Runtime PM State */
    pm_runtime_put_noidle(&smb->client->dev);
    pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

    /* INOK pin configuration */
    if (GPIO_INOK >= 0) {
        ret = bq24156_inok_gpio_init(smb);
        if (ret < 0) {
            dev_warn(dev, "fail to initialize INOK gpio: %d\n", ret);
        }
    }

    smb->running = true;
    smb->dentry = debugfs_create_file("smb", S_IRUGO, NULL, smb, &bq24156_debugfs_fops);

    sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

    ret = bq24156_register_power_supply(&client->dev);
    if (ret < 0)
        return ret;

    printk(" ++++++++++++++++ %s done ++++++++++++++++ client->addr = %x\n", __func__ , client->addr);

    /* init wake lock */
    wake_lock_init(&smb->wakelock,  WAKE_LOCK_SUSPEND, "bq24156_wakelock");

    battery_poll_work_queue = create_singlethread_workqueue("battery_workqueue");
    INIT_DELAYED_WORK(&status_poll_work, charger_status_poll);

    queue_delayed_work(battery_poll_work_queue, &status_poll_work, 5*HZ);

    return 0;
}

static int bq24156_remove(struct i2c_client *client) {
    struct bq24156_charger *smb = i2c_get_clientdata(client);

    if (!IS_ERR_OR_NULL(smb->dentry))
        debugfs_remove(smb->dentry);

    smb->running = false;

    wake_lock_destroy(&smb->wakelock);
    pm_runtime_get_noresume(&smb->client->dev);

    return 0;
}

static int bq24156_shutdown(struct i2c_client *client) {
    dev_info(&client->dev, "%s\n", __func__);
    /* confirm charging due to no HW charger reset function */
#if defined(CONFIG_FE170CG)
    bq24156_charging_toggle(true);
#endif
    return 0;
}

#ifdef CONFIG_PM
static int bq24156_prepare(struct device *dev) {
    struct bq24156_charger *smb = dev_get_drvdata(dev);
    dev_info(&smb->client->dev, "bq24156 prepare\n");
    return 0;
}

static void bq24156_complete(struct device *dev){
    struct bq24156_charger *smb = dev_get_drvdata(dev);
    dev_info(&smb->client->dev, "bq24156 complete\n");
}
#else
#define bq24156_prepare NULL
#define bq24156_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int bq24156_runtime_suspend(struct device *dev) {
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int bq24156_runtime_resume(struct device *dev) {
    dev_info(dev, "%s called\n", __func__);
    return 0;
}

static int bq24156_runtime_idle(struct device *dev) {
    dev_info(dev, "%s called\n", __func__);
    return 0;
}
#else
#define bq24156_runtime_suspend    NULL
#define bq24156_runtime_resume    NULL
#define bq24156_runtime_idle    NULL
#endif

#define bq24156_DEV_NAME        "smb345"

static const struct i2c_device_id bq24156_id[] = {
    {bq24156_DEV_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, bq24156_id);

static const struct dev_pm_ops bq24156_pm_ops = {
    .prepare            = bq24156_prepare,
    .complete           = bq24156_complete,
    .runtime_suspend    = bq24156_runtime_suspend,
    .runtime_resume     = bq24156_runtime_resume,
    .runtime_idle       = bq24156_runtime_idle,
};

static struct i2c_driver bq24156_driver = {
    .driver = {
        .name    = bq24156_DEV_NAME,
        .owner    = THIS_MODULE,
        .pm    = &bq24156_pm_ops,
    },
    .probe       = bq24156_probe,
    .remove      = bq24156_remove,
    .shutdown    = bq24156_shutdown,
    .id_table    = bq24156_id,
};

static int __init bq24156_init(void) {
#ifdef CONFIG_FE170CG
    //HW_ID = Read_HW_ID();
    HW_ID = 1;
    if (HW_ID == 0) {
        printk(" ++++++++++++++++ %s ++++++++++++++++ only Support SR.\n", __func__);
        return i2c_add_driver(&bq24156_driver);
    }
#endif
    return -EAGAIN;
}

module_init(bq24156_init);

static void __exit bq24156_exit(void) {
#ifdef CONFIG_FE170CG
    if (HW_ID == 0) i2c_del_driver(&bq24156_driver);
#endif
}
module_exit(bq24156_exit);

MODULE_AUTHOR("Tom Shen <tom_shen@asus.com>");
MODULE_DESCRIPTION("bq24156 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:bq24156");
