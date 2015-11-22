/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen tom_sheng@asus.com
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/earlysuspend.h>
#include <linux/poll.h>
#include <linux/acpi.h>

#include "asus_battery.h"
#include "smb347_external_include.h"

DEFINE_MUTEX(bq27541_dev_info_mutex);
extern int max_current_set;

#define SMBUS_RETRY                                      3

#define GPIOPIN_LOW_BATTERY_DETECT	                 36
#define BATTERY_POLLING_RATE	                         60
#define DELAY_FOR_CORRECT_CHARGER_STATUS	         5
#define TEMP_KELVIN_TO_CELCIUS		                 2731
#define MAXIMAL_VALID_BATTERY_TEMP	                 200

#define BATTERY_MANUFACTURER_SIZE	                 12
#define BATTERY_NAME_SIZE                                8

/* Battery flags bit definitions */
#define BATT_STS_DSG                                     0x0001
#define BATT_STS_FC                                      0x0200

/* Debug Message */
#define BAT_NOTICE(format, arg...)	  printk(KERN_NOTICE "%s " format , __FUNCTION__ , ## arg)
#define BAT_ERR(format, arg...)		  printk(KERN_ERR format , ## arg)

#define TF303CL_GAUGE_ACPI                     1

/* Global variable */
unsigned battery_cable_status = 0;
unsigned battery_driver_ready = 0;
static int ac_on;
static int usb_on;
static unsigned int 	battery_current;
static unsigned int     battery_remaining_capacity;
struct workqueue_struct *battery_poll_work_queue = NULL;

/* Functions declaration */
//static int bq27541_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val);
static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

//extern unsigned  get_USB_PC_status(void);

module_param(battery_current, uint, 0644);
module_param(battery_remaining_capacity, uint, 0644);

#ifdef ASUS_ENG_BUILD
bool eng_charging_limit = true;
#endif
static int battery_temp_zone = TEMP_0_45;

//+++ i2c stress test
#define BQ27541_IOC_MAGIC 0xF9
#define BQ27541_IOC_MAXNR 5
#define BQ27541_POLL_DATA _IOR(BQ27541_IOC_MAGIC, BQ27541_IOC_MAXNR,int )

#define BQ27541_IOCTL_START_HEAVY 2
#define BQ27541_IOCTL_START_NORMAL 1
#define BQ27541_IOCTL_END 0

#define START_NORMAL    10 * (HZ)
#define START_HEAVY     (HZ)
static int stress_test_poll_mode = 0;
struct delayed_work bq27541_stress_test_poll_work;
static struct workqueue_struct *bq27541_stress_test_work_queue;
static wait_queue_head_t poll_wait_queue_head_t;
static bool flag_pollin = true;
//---

#define BIT0  0x00000001
#define BIT1  0x00000002
#define BIT2  0x00000004
#define BIT3  0x00000008
#define BIT4  0x00000010
#define BIT5  0x00000020
#define BIT6  0x00000040
#define BIT7  0x00000080

#define BIT8  0x00000100
#define BIT9  0x00000200
#define BIT10 0x00000400
#define BIT11 0x00000800
#define BIT12 0x00001000
#define BIT13 0x00002000
#define BIT14 0x00004000
#define BIT15 0x00008000

#define BQ27541_DATA(_psp, _addr, _min_value, _max_value)	\
	{								\
		.psp = POWER_SUPPLY_PROP_##_psp,	\
		.addr = _addr,				\
		.min_value = _min_value,		\
		.max_value = _max_value,	\
	}

enum {
        REG_MANUFACTURER_DATA,
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};

typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

static struct bq27541_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq27541_data[] = {
        [REG_MANUFACTURER_DATA]	                = BQ27541_DATA (PRESENT           ,    0,      0, 65535),
        [REG_STATE_OF_HEALTH]		        = BQ27541_DATA (HEALTH            ,    0,      0, 65535),
	[REG_TEMPERATURE]			= BQ27541_DATA (TEMP              , 0x06,      0, 65535),
	[REG_VOLTAGE]				= BQ27541_DATA (VOLTAGE_NOW       , 0x08,      0,  6000),
	[REG_CURRENT]				= BQ27541_DATA (CURRENT_NOW       , 0x14, -32768, 32767),
	[REG_TIME_TO_EMPTY]			= BQ27541_DATA (TIME_TO_EMPTY_AVG , 0x16,      0, 65535),
	[REG_TIME_TO_FULL]			= BQ27541_DATA (TIME_TO_FULL_AVG  , 0x18,      0, 65535),
	[REG_STATUS]				= BQ27541_DATA (STATUS            , 0x0a,      0, 65535),
	[REG_CAPACITY]				= BQ27541_DATA (CAPACITY          , 0x2c,      0,   100),
};

static enum power_supply_property bq27541_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

void check_cabe_type(void) {
      if (battery_cable_status == USB_ADAPTER) {
		 ac_on = 1;
	        usb_on = 0;
      } else if (battery_cable_status == USB_PC) {
		usb_on = 1;
		ac_on  = 0;
      } else {
		ac_on  = 0;
		usb_on = 0;
                battery_temp_zone = TEMP_0_45;
      }
}

static enum power_supply_property power_properties[] = {
        POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int power_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val) {
	int ret=0;
	switch (psp) {
	    case POWER_SUPPLY_PROP_ONLINE:
		   if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  ac_on)
			val->intval =  1;
		   else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
			val->intval =  1;
		   else
			val->intval = 0;
		break;
            case POWER_SUPPLY_PROP_PRESENT:
                if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS) {
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
		return -EINVAL;
	}
	return ret;
}

static char *supply_list[] = {
	"battery",
	"ac",
#ifndef REMOVE_USB_POWER_SUPPLY
	"usb",
#endif
};

static struct power_supply bq27541_supply[] = {
	{
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= bq27541_properties,
		.num_properties = ARRAY_SIZE(bq27541_properties),
		.get_property	= bq27541_get_property,
       },
	{
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property	= power_get_property,
	},
#ifndef REMOVE_USB_POWER_SUPPLY
	{
		.name		= "usb",
		.type		= POWER_SUPPLY_TYPE_USB,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties =power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
#endif
};

static struct bq27541_device_info {
	struct i2c_client *client;
	struct delayed_work battery_stress_test;
	struct delayed_work status_poll_work;
	struct delayed_work low_low_bat_work;
	struct miscdevice battery_misc;
	struct wake_lock low_battery_wake_lock;
	struct wake_lock cable_event_wake_lock;
	int smbus_status;
	int battery_present;
	int low_battery_present;
	int gpio_battery_detect;
	int gpio_low_battery_detect;
	int irq_low_battery_detect;
	int irq_battery_detect;
	int bat_status;
	int bat_temp;
	int bat_vol;
	int bat_current;
	int bat_capacity;
	unsigned int old_capacity;
	unsigned int cap_err;
	unsigned int old_temperature;
	unsigned int temp_err;
	unsigned int prj_id;
	spinlock_t lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
        struct  early_suspend es;
#endif
} *bq27541_device;

static int bq27541_read_i2c(u8 reg, int *rt_value, int b_single) {
	struct i2c_client *client = bq27541_device->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
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
	} else {
           dev_err(&client->dev, "I2C read from addr 0x%02X error with code: %d\n", reg, err);
        }

	return err;
}

int bq27541_smbus_read_data(int reg_offset,int byte,int *rt_value) {
	s32 ret = -EINVAL;
	int count = 0;

        mutex_lock(&bq27541_dev_info_mutex);
	do {
		ret = bq27541_read_i2c(bq27541_data[reg_offset].addr, rt_value, 0);
	} while( (ret < 0) && ( ++count <= SMBUS_RETRY) );
        mutex_unlock(&bq27541_dev_info_mutex);

	return ret;
}

int bq27541_smbus_read_data_reg(u8 reg,int byte,int *rt_value) {
	s32 ret = -EINVAL;
	int count = 0;

        mutex_lock(&bq27541_dev_info_mutex);
	do {
		ret = bq27541_read_i2c (reg, rt_value, 0);
	} while ( (ret < 0) && (++count <= SMBUS_RETRY) );
        mutex_unlock(&bq27541_dev_info_mutex);

	return ret;
}

int bq27541_smbus_write_data(int reg_offset, int byte, unsigned int value) {
        s32 ret = -EINVAL;
        int count=0;

	do {
            if (byte) {
               ret = i2c_smbus_write_byte_data(bq27541_device->client,bq27541_data[reg_offset].addr, value & 0xFF);
	    } else{
	       ret = i2c_smbus_write_word_data(bq27541_device->client,bq27541_data[reg_offset].addr, value & 0xFFFF);
            }
	} while ( (ret < 0) && (++count <= SMBUS_RETRY) );
	return ret;
}

static ssize_t show_battery_smbus_status(struct device *dev, struct device_attribute *devattr, char *buf) {
	int status = !bq27541_device->smbus_status;
	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO, show_battery_smbus_status, NULL);

static struct attribute *battery_smbus_attributes[] = {
	&dev_attr_battery_smbus.attr,
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};

static int bq27541_get_capacity(void) {
	s32 temp_capacity;
	int remainingcapacity = 0;

        bq27541_device->smbus_status = bq27541_smbus_read_data(REG_CAPACITY, 0 ,&bq27541_device->bat_capacity);
        if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d failed bq27541_device->cap_err=%u\n", __func__, REG_CAPACITY, bq27541_device->cap_err);
		if(bq27541_device->cap_err> 5 || (bq27541_device->old_capacity == 0xFF)) {
			return -EINVAL;
		} else {
			bq27541_device->cap_err++;
			BAT_NOTICE("cap_err=%u use old capacity=%u\n", bq27541_device->cap_err, bq27541_device->old_capacity);
			return 0;
		}
	}

	temp_capacity = ((bq27541_device->bat_capacity >= 100) ? 100 : bq27541_device->bat_capacity);
	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);

	bq27541_device->old_capacity =temp_capacity;
	bq27541_device->cap_err=0;

	BAT_NOTICE("user capacity:%u%% ,gauge capacity: %u%%\n", bq27541_device->old_capacity, bq27541_device->bat_capacity);

        // for PSChange App tool
        bq27541_device->smbus_status = bq27541_smbus_read_data_reg(0x10, 0 ,&remainingcapacity);
        if (bq27541_device->smbus_status >= 0) {
            battery_remaining_capacity = remainingcapacity;
	    BAT_NOTICE("remaining capacity= %d\n", remainingcapacity);
        }
	return 0;
}

static int bq27541_get_current(void) {
        int rt_value = 0;

        bq27541_device->smbus_status = bq27541_smbus_read_data_reg(bq27541_data[REG_CURRENT].addr, 0, &rt_value);
        if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d failed\n", __func__, bq27541_data[REG_CURRENT].addr);
	}
        if (rt_value & BIT15)  rt_value |= 0xFFFF0000; //neg. sign extend.

        rt_value += 0x10000;

        if (rt_value >= 0) rt_value -= 0x10000;
	bq27541_device->bat_current = rt_value;
        return rt_value;
}

static int bq27541_get_temperature(void) {
        int rt_value = 0;

        bq27541_device->smbus_status = bq27541_smbus_read_data_reg(bq27541_data[REG_TEMPERATURE].addr, 0, &rt_value);
        if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d failed\n", __func__, bq27541_data[REG_TEMPERATURE].addr);
	}
        bq27541_device->bat_temp = rt_value;
	bq27541_device->old_temperature = bq27541_device->bat_temp - TEMP_KELVIN_TO_CELCIUS;
        return (rt_value - TEMP_KELVIN_TO_CELCIUS);
}

int do_battery_jeita(int batt_tempr, int temp_zone_last) {

   int ret, need_add_0 = 0, need_add_45_55 = 0, need_add_55 = 0;

   switch (temp_zone_last) {
       case TEMP_0:
             printk(" %s: invalid battery temperature zone on last one !!! under battery temperature 0 !!! \n", __func__);
             need_add_0 = 50;
             break;
       case TEMP_45_55:
             printk(" %s: invalid battery temperature zone on last one  !!! between battery temperature 45 and 55 !!! \n", __func__);
             need_add_45_55 = 50;
             break;
       case TEMP_55:
             printk(" %s: invalid battery temperature zone on last one !!! upper battery temperature 55 !!! \n", __func__);
             need_add_55 = 50;
             need_add_45_55 = 50;
             break;
       case TEMP_0_45:
       default:
             break;
   }

   ret = POWER_SUPPLY_STATUS_CHARGING;
   if (batt_tempr < (0 + need_add_0)) {
      smb347_set_voltage(4200);
      smb347_charging_toggle(false);
      ret = POWER_SUPPLY_STATUS_DISCHARGING;

      battery_temp_zone = TEMP_0;
   } else if (batt_tempr >= (0 + need_add_0) && batt_tempr < (450 - need_add_45_55)) {
      smb347_set_voltage(4200);

      battery_temp_zone = TEMP_0_45;
   } else if (batt_tempr >= (450 - need_add_45_55) && batt_tempr < (550 - need_add_55)) {
      smb347_set_voltage(4000);

      battery_temp_zone = TEMP_45_55;
   } else if (batt_tempr >= (550 - need_add_55) ) {
      smb347_set_voltage(4000);
      smb347_charging_toggle(false);
      ret = POWER_SUPPLY_STATUS_DISCHARGING;

      battery_temp_zone = TEMP_55;
   }

   return ret;
}

int smb345_battery_jeita(int batt_tempr) {
    return do_battery_jeita(batt_tempr, battery_temp_zone);
}

void asus_update_all(void) {
        s32 ret;
        int rt_value = 0;
        int temperature = 0;
        static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};

        // 1. capacity
        bq27541_get_capacity();

        // 2. voltage
        bq27541_device->smbus_status = bq27541_smbus_read_data_reg(bq27541_data[REG_VOLTAGE].addr, 0, &rt_value);
        if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d failed\n", __func__, bq27541_data[REG_VOLTAGE].addr);
	}
        bq27541_device->bat_vol = rt_value;
	BAT_NOTICE("voltage_now= %d mV\n", bq27541_device->bat_vol);

        // 3. current
        bq27541_get_current();
	BAT_NOTICE("current_now= %d mA\n", bq27541_device->bat_current);

        // 4. temperature
        bq27541_get_temperature();
        BAT_NOTICE("temperature= %d (0.1¢XC)\n", bq27541_device->old_temperature);

        // 5. status
        bq27541_device->smbus_status = bq27541_smbus_read_data_reg(bq27541_data[REG_STATUS].addr, 0, &rt_value);
        if (bq27541_device->smbus_status < 0) {
		dev_err(&bq27541_device->client->dev, "%s: i2c read for %d failed\n", __func__, bq27541_data[REG_STATUS].addr);
	}

        if ((usb_on) || (ac_on)) {
            ret = POWER_SUPPLY_STATUS_CHARGING;
#ifdef ASUS_ENG_BUILD
            /* eng mode : stop charging when battery percentage is over 60% */
            if (bq27541_device->old_capacity >= 60 && eng_charging_limit) {
                BAT_NOTICE("eng mode : stop charging when battery percentage is over 60%% \n");
                smb347_charging_toggle(false);
                ret = POWER_SUPPLY_STATUS_DISCHARGING;
                goto final;
            }
#endif
            if (ret == POWER_SUPPLY_STATUS_CHARGING) {
               /* limit to protect battery from damaged when battery temperature is too low or High*/
               temperature = bq27541_device->old_temperature;
               ret = smb345_battery_jeita(temperature);
               if (ret == POWER_SUPPLY_STATUS_DISCHARGING)
                    goto final;

               if (ac_on && smb347_get_aicl_result() <= 0x01) {// AICL result < 500mA
                      BAT_NOTICE("AICL get result, AICL results < 500mA \n");
                      smb347_AC_in_current(1800, USE_AICL);
               }
            }

            if (bq27541_device->old_capacity >= 0 && bq27541_device->old_capacity <= 100) {
                switch (bq27541_device->bat_capacity) {
                   case 100:
                       ret = POWER_SUPPLY_STATUS_FULL;
                   case 99:
                       if (rt_value & BATT_STS_FC) {
                           //check if Full-charged is detected
                           BAT_NOTICE("[[ Full-charged is detected ]] \n");
                           smb347_charging_toggle(false);
                           ret = POWER_SUPPLY_STATUS_FULL;
                       } else {
                           smb347_charging_toggle(true);
                       }
                       break;

                  default:
                      smb347_charging_toggle(true);
                       break;

                }
            } else {
                BAT_NOTICE("Incorrect percentage !!!!!\n");
            }
        } else {// if ((usb_on) || (ac_on))
                if (bq27541_device->old_capacity == 100)
                        ret = POWER_SUPPLY_STATUS_FULL;
                else
                        ret = POWER_SUPPLY_STATUS_DISCHARGING;
        }

final:
        bq27541_device->bat_status = ret;
        BAT_NOTICE("status: %s ret= 0x%04x\n", status_text[ret], rt_value);

	return;
}

//+++ i2c stress test
static void bq27541_stress_test_poll(struct work_struct * work) {
    int ret = 0, percentage = 50;

    ret = bq27541_smbus_read_data_reg(bq27541_data[REG_CAPACITY].addr   , 0 , &percentage);
    if (ret < 0) {
        BAT_ERR("------------> bq27541 gauge i2c fail !!!!! <--------------------\n");
    } else {
        BAT_NOTICE("------------> stress test percentage  = %d , polling = %d sec <--------------------\n", percentage , stress_test_poll_mode/HZ);
    }

    if (!smb347_has_charger_error())
        BAT_NOTICE("------------> smb345 charger status successful , polling = %d sec <--------------------\n", stress_test_poll_mode/HZ);
    else
        BAT_ERR("------------> smb345 charger status fail !!!!! <--------------------\n");

    queue_delayed_work(bq27541_stress_test_work_queue, &bq27541_stress_test_poll_work, stress_test_poll_mode);
}

int bq27541_open(struct inode *inode, struct file *filp) {
	printk("battery bq27541 : %s\n", __func__);
	return 0;
}

int bq27541_release(struct inode *inode, struct file *filp) {
	printk("battery bq27541: %s\n", __func__);
	return 0;
}

static unsigned int bq27541_poll(struct file *filp, poll_table *wait){
	unsigned int mask = 0;
	poll_wait(filp, &poll_wait_queue_head_t, wait);
	if (flag_pollin==true) {
		mask |= POLLIN;
		flag_pollin=false;
	}
	printk("battery bq27541 : %s\n", __func__);
	return mask;
}

long bq27541_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	int err = 1;
	printk("--------------------------> bq27541_ioctl \n");
	if (_IOC_TYPE(cmd) != BQ27541_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > BQ27541_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case BQ27541_POLL_DATA:
			if (arg == BQ27541_IOCTL_START_HEAVY){
				printk("battery ba27541 : ioctl heavy\n");
				stress_test_poll_mode = START_HEAVY;
				queue_delayed_work(bq27541_stress_test_work_queue, &bq27541_stress_test_poll_work, stress_test_poll_mode);
			} else if (arg == BQ27541_IOCTL_START_NORMAL){
				printk("battery ba27541 : ioctl normal\n");
				stress_test_poll_mode = START_NORMAL;
				queue_delayed_work(bq27541_stress_test_work_queue, &bq27541_stress_test_poll_work, stress_test_poll_mode);
			} else if  (arg == BQ27541_IOCTL_END){
				printk("light sensor BQ27541 : ioctl end\n");
				cancel_delayed_work_sync(&bq27541_stress_test_poll_work);
			} else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
        }

        return 0;
}

struct file_operations bq27541_fops = {
	.owner = THIS_MODULE,
	.open = bq27541_open,
	.release = bq27541_release,
	.poll = bq27541_poll,
	.unlocked_ioctl = bq27541_ioctl,
};

//---

static void battery_status_poll(struct work_struct *work) {
        u32 polling_time = 60 * HZ;
        struct bq27541_device_info *batt_dev = container_of(work, struct bq27541_device_info, status_poll_work.work);
        char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};

        if(!battery_driver_ready)
		BAT_NOTICE("battery driver not ready\n");

        asus_update_all();

        power_supply_changed(&bq27541_supply[Charger_Type_Battery]);
        power_supply_changed(&bq27541_supply[Charger_Type_AC]);
        power_supply_changed(&bq27541_supply[Charger_Type_USB]);

        msleep(10);

        if (bq27541_device->old_temperature >= 630) {
            BAT_NOTICE("Critical condition!! -> temperature \n");
            polling_time = BATTERY_CRITICAL_POLL_TIME;
        } else if (bq27541_device->old_temperature >= 500) {
            BAT_NOTICE("Nearly critical condition!! -> temperature \n");
            polling_time = 10*HZ;
        }

        if (bq27541_device->old_capacity >= 50 && bq27541_device->old_capacity <= 100)
            polling_time = (60*HZ) < polling_time ? (60*HZ) : polling_time;
        else if (bq27541_device->old_capacity >= 20 && bq27541_device->old_capacity <= 49)
            polling_time = (30*HZ) < polling_time ? (30*HZ) : polling_time;
        else if (bq27541_device->old_capacity >= 5 && bq27541_device->old_capacity <= 19)
            polling_time = (10*HZ) < polling_time ? (10*HZ) : polling_time;
        else if (bq27541_device->old_capacity >= 0 && bq27541_device->old_capacity <= 4)
            polling_time = (5*HZ) < polling_time ? (5*HZ) : polling_time;
        else {
            BAT_NOTICE("*** Battery percentage is out of the legal range (percentage < 0 or percentage > 100) ***\n");
            polling_time = 5*HZ;
        }
        printk("<BATT> battery info (P:%d %%(%d %%), V:%d mV, C:%d mA, T:%d.%d P: %d secs S: %s)\n",
                bq27541_device->old_capacity,
                bq27541_device->bat_capacity,
                bq27541_device->bat_vol,
                bq27541_device->bat_current,
                bq27541_device->old_temperature/10,
                bq27541_device->old_temperature%10,
                polling_time/HZ,
                status_text[bq27541_device->bat_status]);

	/* Schedule next polling */
	queue_delayed_work(battery_poll_work_queue, &batt_dev->status_poll_work, polling_time);
}

static void low_low_battery_check(struct work_struct *work) {

	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(battery_poll_work_queue,&bq27541_device->status_poll_work, 0.1*HZ);
	msleep(2000);
	enable_irq(bq27541_device->irq_low_battery_detect);
}

#if 0
static irqreturn_t low_battery_detect_isr(int irq, void *dev_id) {
	disable_irq_nosync(bq27541_device->irq_low_battery_detect);
	bq27541_device->low_battery_present = gpio_get_value(bq27541_device->gpio_low_battery_detect);
	BAT_NOTICE("gpio LL_BAT =%d\n", bq27541_device->low_battery_present);
	wake_lock_timeout(&bq27541_device->low_battery_wake_lock, 10*HZ);
	queue_delayed_work(battery_poll_work_queue, &bq27541_device->low_low_bat_work, 0.1*HZ);
	return IRQ_HANDLED;
}

static int setup_low_battery_irq(void) {

	unsigned gpio = GPIOPIN_LOW_BATTERY_DETECT;
	int ret, irq = gpio_to_irq(gpio);

	ret = gpio_request(gpio, "low_battery_detect");
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 request failed\n");
		goto fail;
	}

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 unavaliable for input\n");
		goto fail_gpio;
	}

	ret = request_irq(irq, low_battery_detect_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"bq27541-battery (low battery)", NULL);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 irq request failed\n");
		goto fail_irq;
	}

	bq27541_device->low_battery_present = gpio_get_value(gpio);
	bq27541_device->irq_low_battery_detect = gpio_to_irq(gpio);
	BAT_NOTICE("irq=%d, LL_BAT=%d\n", irq, bq27541_device->low_battery_present);

	enable_irq_wake(bq27541_device->irq_low_battery_detect);

fail_irq:
fail_gpio:
	gpio_free(gpio);
fail:
	return ret;
}
#endif

void bq27541_battery_callback(unsigned USB_PC_state) {
	int old_cable_status = battery_cable_status;
	battery_cable_status = USB_PC_state;

        printk("========================================================\n");
	printk("bq27541_battery_callback  USB_PC_state = %x\n", USB_PC_state);
        printk("========================================================\n");

	if(!battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		return;
	}

	check_cabe_type();
        wake_lock_timeout(&bq27541_device->cable_event_wake_lock, DELAY_FOR_CORRECT_CHARGER_STATUS*HZ);
	if(!battery_cable_status) {
		if (old_cable_status == USB_ADAPTER) {
			power_supply_changed(&bq27541_supply[Charger_Type_AC]);
		}
#ifndef REMOVE_USB_POWER_SUPPLY
		else if ( old_cable_status == USB_PC) {
			power_supply_changed(&bq27541_supply[Charger_Type_USB]);
		}
#endif
	}

#ifndef REMOVE_USB_POWER_SUPPLY
	else if (battery_cable_status == USB_PC) {
		power_supply_changed(&bq27541_supply[Charger_Type_USB]);
	}
#endif

	else if (battery_cable_status == USB_ADAPTER) {
		power_supply_changed(&bq27541_supply[Charger_Type_AC]);
	}

	cancel_delayed_work(&bq27541_device->status_poll_work);
	//queue_delayed_work(battery_poll_work_queue, &bq27541_device->status_poll_work,
	//	battery_cable_status ? DELAY_FOR_CORRECT_CHARGER_STATUS *HZ : 2*HZ);
        queue_delayed_work(battery_poll_work_queue, &bq27541_device->status_poll_work, 0.1*HZ);
}
EXPORT_SYMBOL(bq27541_battery_callback);

static int bq27541_get_health(enum power_supply_property psp, union power_supply_propval *val) {

	if (psp == POWER_SUPPLY_PROP_PRESENT) {
		val->intval = 1;
	} else if (psp == POWER_SUPPLY_PROP_HEALTH) {
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}
	return 0;
}

static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val) {

	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (bq27541_get_health(psp, val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = bq27541_device->old_capacity;
			break;

		case POWER_SUPPLY_PROP_STATUS:
                       val->intval = bq27541_device->bat_status;
                       break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
                       val->intval = bq27541_device->bat_vol * 1000;
                       break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
                       val->intval = bq27541_device->bat_current;
                       break;
		case POWER_SUPPLY_PROP_TEMP:
                       val->intval = bq27541_device->old_temperature;
                       break;

		default:
			dev_err(&bq27541_device->client->dev, "%s: INVALID property psp=%u\n", __func__,psp);
			return -EINVAL;
	}
	return 0;
error:

	return -EINVAL;
}

static int bq27541_proc_info_dump_read(struct seq_file *m, void *p) {
        static int bq_batt_percentage = 0;
        static int bq_batt_volt = 0;
        static int bq_batt_current = 0;
        static int bq_batt_temp = 0;
        static int bq_batt_remaining_capacity = 0;
        static int bq_batt_full_charge_capacity = 0;

        bq27541_smbus_read_data_reg(0x12, 0 ,&bq_batt_full_charge_capacity);
        bq27541_smbus_read_data_reg(0x10, 0 ,&bq_batt_remaining_capacity);
        bq27541_smbus_read_data_reg(bq27541_data[REG_CAPACITY].addr   , 0 ,&bq_batt_percentage);
        bq27541_smbus_read_data_reg(bq27541_data[REG_VOLTAGE].addr    , 0 ,&bq_batt_volt);
        bq_batt_current = bq27541_get_current();
        bq_batt_temp    = bq27541_get_temperature();

        seq_printf(m,"LMD(mAh): %d\n", bq_batt_full_charge_capacity);
        seq_printf(m,"NAC(mAh): %d\n", bq_batt_remaining_capacity);
        seq_printf(m,"RSOC: %d\n", bq_batt_percentage);
        seq_printf(m,"USOC: %d\n", bq27541_device->old_capacity);
        seq_printf(m,"voltage(mV): %d\n", bq_batt_volt);
        seq_printf(m,"average_current(mA): %d\n", bq_batt_current);
        seq_printf(m,"temp: %d\n", bq_batt_temp);

        return 0;
}

static int proc_bq27541_test_info_dump_open(struct inode *inode, struct file *file) {
	return single_open(file, bq27541_proc_info_dump_read, NULL);
}

static const struct file_operations proc_bq27541_test_info_dump_ops = {
	.open		= proc_bq27541_test_info_dump_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release
};

static int bq27541_register_upilogger_proc_fs(void) {
        struct proc_dir_entry *entry = NULL;

        entry = proc_create("bq27520_test_info_dump", 0664, NULL, &proc_bq27541_test_info_dump_ops);
        if (!entry) {
            printk("[%s]Unable to create bq27541_test_info_dump \n", __FUNCTION__);
            return -EINVAL;
        }
        return 0;
}

static bool checkBatteryInvalidStatus(void) {
    int   temperature;
    int   capacity;

    bq27541_smbus_read_data_reg(bq27541_data[REG_CAPACITY].addr, 0 ,&capacity);

    // 1. check if battery temperature is too high or too low.
    if (ac_on) {
        temperature = bq27541_get_temperature();
        if (temperature >= 500 || temperature <= 0) {
             BAT_NOTICE( "=== battery is in critical temperatue with AC charger during sleep, stop charging !!!!!!, temperature = %d \n", temperature);
             smb347_charging_toggle(false);
             return true;
        } else if (capacity > 98){
             BAT_NOTICE("=== battery is nearly full capacity ======, capacity = %d\n", capacity);
             return true;
        } else {
             BAT_NOTICE("=== battery is in normal status ======, capacity = %d , temperature = %d\n", capacity, temperature);
             smb347_charging_toggle(true);
             return false;
        }
    }

    // 2. check if battery capacity is too low, if yes notify system to do shutdown
    if (capacity <= 5) {
         BAT_NOTICE( "===  battery capacity is too low in sleep!!!\n");;
         return true;
    }
    return false;
}

#if defined (CONFIG_PM)
static int bq27541_suspend (struct device *dev){
        printk("enter %s\n", __func__);
	cancel_delayed_work_sync(&bq27541_device->status_poll_work);
	return 0;
}

/* any smbus transaction will wake up pad */
static int bq27541_resume (struct device *dev) {
        printk("enter %s\n", __func__);

        if (checkBatteryInvalidStatus())
            queue_delayed_work(battery_poll_work_queue,&bq27541_device->status_poll_work, 0*HZ);
        else
	    queue_delayed_work(battery_poll_work_queue,&bq27541_device->status_poll_work, 5*HZ);
	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bq27541_early_suspend(struct early_suspend *h) {
        printk("enter %s\n", __func__);
}

static void bq27541_late_resume(struct early_suspend *h) {
        printk("enter %s\n", __func__);
        queue_delayed_work(battery_poll_work_queue,&bq27541_device->status_poll_work, 0*HZ);
}
#endif

//asus_eng_charging_limit     ------START
#ifdef ASUS_ENG_BUILD
static int asus_charging_toggle_read(struct seq_file *m, void *p) {
    int len;

    len = seq_printf(m,"eng_charging_limit: %d\n", eng_charging_limit);
    return len;
}

static ssize_t asus_charging_toggle_write(struct file *file, const char *buffer, size_t count, loff_t *data) {

    BAT_NOTICE(":\n");

    if (buffer[0] == '0') {
        /* turn on charging limit in eng mode */
        eng_charging_limit = true;
    } else if (buffer[0] == '1') {
        /* turn off charging limit in eng mode */
        eng_charging_limit = false;
    }

    cancel_delayed_work_sync(&bq27541_device->status_poll_work);
    queue_delayed_work(battery_poll_work_queue,&bq27541_device->status_poll_work, 0.1*HZ);

    return count;
}

static int asus_charging_toggle_open(struct inode *inode, struct file *file) {
	return single_open(file, asus_charging_toggle_read, NULL);
}

static const struct file_operations asus_eng_charging_limit_ops = {
        .open           = asus_charging_toggle_open,
	.read		= seq_read,
        .write          = asus_charging_toggle_write,
	.llseek		= seq_lseek,
	.release	= seq_release
};

int init_asus_charging_toggle(void) {
    struct proc_dir_entry *entry=NULL;

    entry = proc_create("asus_eng_charging_limit", 0666, NULL, &asus_eng_charging_limit_ops);
    if (!entry) {
        BAT_DBG_E("Unable to create asus_charging_toggle\n");
        return -EINVAL;
    }
    return 0;
}
#endif
//asus_eng_charging_limit      ------END

static int bq27541_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int ret, i = 0, rt_value;

	BAT_NOTICE("+ client->addr= %02x\n", client->addr);

	bq27541_device = kzalloc(sizeof(*bq27541_device), GFP_KERNEL);
	if (!bq27541_device) {
             BAT_ERR("Failed to allocate bq27541_device memory. \n");
             return -ENOMEM;
        }

	memset(bq27541_device, 0, sizeof(*bq27541_device));
	bq27541_device->client = client;
	i2c_set_clientdata(client, bq27541_device);

        bq27541_device->smbus_status    = 0;
        bq27541_device->cap_err         = 0;
	bq27541_device->temp_err        = 0;
	bq27541_device->old_capacity    = 50;
        bq27541_device->old_temperature = 250;
	bq27541_device->gpio_low_battery_detect = GPIOPIN_LOW_BATTERY_DETECT;

        bq27541_device->smbus_status = bq27541_smbus_read_data_reg( bq27541_data[REG_STATUS].addr, 0, &rt_value);//read gauge status
	if (bq27541_device->smbus_status < 0) {
            dev_err(&bq27541_device->client->dev, "%s, ==== fail to read gauge i2c, ,maybe no insert battery or other reasons: i2c read for 0x0A failed\n", __func__);
            return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++) {
		ret = power_supply_register(&client->dev, &bq27541_supply[i]);
		if (ret) {
			BAT_ERR("Failed to register power supply ,num = %d\n", i);
                        do {
				power_supply_unregister(&bq27541_supply[i]);
			} while ((--i) >= 0);
		        kfree(bq27541_device);
			return ret;
		}
	}

	battery_poll_work_queue = create_singlethread_workqueue("battery_workqueue");
	INIT_DELAYED_WORK(&bq27541_device->status_poll_work, battery_status_poll);
	INIT_DELAYED_WORK(&bq27541_device->low_low_bat_work, low_low_battery_check);
        //INIT_DELAYED_WORK(&bq27541_device->battery_stress_test, battery_strees_test);
	cancel_delayed_work(&bq27541_device->status_poll_work);

	spin_lock_init(&bq27541_device->lock);
	wake_lock_init(&bq27541_device->low_battery_wake_lock, WAKE_LOCK_SUSPEND, "low_battery_detection");
	wake_lock_init(&bq27541_device->cable_event_wake_lock, WAKE_LOCK_SUSPEND, "battery_cable_event");

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &battery_smbus_group);
	if (ret) dev_err(&client->dev, "bq27541_probe: unable to create the sysfs\n");

        /* stress test */
	bq27541_stress_test_work_queue = create_singlethread_workqueue("i2c_battery_wq");
	if(!bq27541_stress_test_work_queue){
		printk("battery bq27541 : unable to create i2c stress test workqueue\n");
	}
	INIT_DELAYED_WORK(&bq27541_stress_test_poll_work, bq27541_stress_test_poll);

	/* Misc device registration */
        bq27541_device->battery_misc.minor = MISC_DYNAMIC_MINOR;
	bq27541_device->battery_misc.name = "battery";
        bq27541_device->battery_misc.fops  = &bq27541_fops;
        ret = misc_register(&bq27541_device->battery_misc);
	if (ret) dev_err(&client->dev, "Cannot register bq27541 miscdev (err = %d)\n", ret);

#ifdef CONFIG_HAS_EARLYSUSPEND
        bq27541_device->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
        bq27541_device->es.suspend = bq27541_early_suspend;
        bq27541_device->es.resume = bq27541_late_resume;
        register_early_suspend(&bq27541_device->es);
#endif

#if 0
	setup_low_battery_irq();
#endif

#if CONFIG_PROC_FS
        ret = bq27541_register_upilogger_proc_fs();
        if (ret) {
                BAT_ERR("Unable to create bq27541_register_upilogger_proc_fs\n");
        }
#ifdef ASUS_ENG_BUILD
		ret = init_asus_charging_toggle();
		if (ret) {
			BAT_ERR("Unable to create init_asus_charging_toggle\n");
		}
#endif
#endif
	battery_driver_ready = 1;
	check_cabe_type();
	queue_delayed_work(battery_poll_work_queue, &bq27541_device->status_poll_work, 5*HZ);

	BAT_NOTICE("- %s driver registered done ====\n", client->name);

	return 0;
}

static int bq27541_remove(struct i2c_client *client) {
	struct bq27541_device_info *bq27541_device;
	int i = 0;

	bq27541_device = i2c_get_clientdata(client);
	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++) {
		power_supply_unregister(&bq27541_supply[i]);
	}

	if (bq27541_device) {
		wake_lock_destroy(&bq27541_device->low_battery_wake_lock);
		kfree(bq27541_device);
		bq27541_device = NULL;
	}
	return 0;
}

static const struct dev_pm_ops bq27541_pm_ops = {
	.suspend                = bq27541_suspend,
	.resume	                = bq27541_resume,
};

static const struct i2c_device_id bq27541_id[] = {
	{ "BQ027541", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
        .driver = {
		.name	= "BQ027541",
		.owner	= THIS_MODULE,
		.pm	= &bq27541_pm_ops,
#if TF303CL_GAUGE_ACPI
                .acpi_match_table    = ACPI_PTR(bq27541_id),
#endif
	},
	.probe		= bq27541_probe,
	.remove		= bq27541_remove,
	.id_table	= bq27541_id,
};

static int __init bq27541_battery_init(void) {
	int ret;

        BAT_NOTICE(": init\n");
	ret = i2c_add_driver(&bq27541_battery_driver);
	if (ret)
		dev_err(&bq27541_device->client->dev, "%s: i2c_add_driver failed\n", __func__);

	return ret;
}

module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void) {
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("Tom Shen");
MODULE_DESCRIPTION("bq27541 battery monitor driver");
MODULE_LICENSE("GPL");
