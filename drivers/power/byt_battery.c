/*
 * Intel Baytrail ULPMC battery driver
 *
 * Copyright (C) 2013 Intel Corporation
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
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/notifier.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/extcon.h>
#include <linux/power/byt_battery.h>
#include <linux/delay.h>

#include <linux/fb.h>
#include <linux/reboot.h>

#include <linux/input.h>

#define EC_SET_AUTO_WAKEUP_REG        0x9E
#define EC_SET_AUTO_WAKEUP_CMD_FLAG   0x06
#define EC_SET_AUTO_WAKEUP_CMD_TIMER  0x08

#define GUAGE_CMD_CNTL                          0x00
#define GUAGE_CMD_TEMP                          0x06


extern struct linux_logo logo_lowbat01_clut224;
extern struct linux_logo logo_lowbat02_clut224;
extern struct linux_logo logo_lowbat03_clut224;


extern struct linux_logo logo_charger01_clut224;
extern struct linux_logo logo_charger02_clut224;
extern struct linux_logo logo_charger03_clut224;

extern struct linux_logo logo_asus01_clut224;
extern struct linux_logo logo_asus02_clut224;
extern struct linux_logo logo_asus03_clut224;



extern struct fb_info *g_fb0_inf;
extern void fb_show_charge_logo(struct linux_logo *logo);
extern int fb_show_logo(struct fb_info *info, int rotate);

static struct linux_logo* g_lowbatlogo[3]= {
        &logo_lowbat01_clut224,&logo_lowbat02_clut224,&logo_lowbat03_clut224
};


static struct linux_logo* g_chargerlogo[3]= {
        &logo_charger01_clut224,&logo_charger02_clut224,&logo_charger03_clut224
};

static struct linux_logo* g_asuslogo[3]= {
        &logo_asus01_clut224,&logo_asus02_clut224,&logo_asus03_clut224
};


static int charger_logo_display(struct linux_logo *logo)
{
       fb_show_charge_logo(logo);
       fb_show_logo(g_fb0_inf, 0);
       return 0;
}



static struct workqueue_struct *byt_wq = NULL;

struct byt_chip_info {
	struct i2c_client	*client;

	struct power_supply	bat;
	struct power_supply	ac;//
	
	struct byt_platform_data *pdata;

	struct byt_battery	ecbat;//

	struct delayed_work byt_batt_info_update_work;

	u8 ec_ver[9];
	int ec_status;

	struct input_dev *lid_dev;

	struct mutex lock;
	
};

static struct byt_chip_info *byt_chip;


static int asusec_write_read(struct byt_chip_info *chip,u8 *write_buf, u16 write_len,u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(msg,0,sizeof(struct i2c_msg)*2);

        num = 2;
        msg[0].addr = chip->client->addr;
        msg[0].flags = !I2C_M_RD;
        msg[0].len = write_len;
        msg[0].buf = write_buf;
        msg[1].addr = chip->client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = read_len;
        msg[1].buf = read_buf;

        ret = i2c_transfer(chip->client->adapter, msg, num);

        if(ret <0) {
                pr_err("i2c transfer error\n");
                return -EIO;
        } else
                return 0;
}

static int asusec_write(struct byt_chip_info *chip,u8 *write_buf, u16 write_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(msg,0,sizeof(struct i2c_msg)*2);

        num = 1;
        msg[0].addr = chip->client->addr;
        msg[0].flags = !I2C_M_RD;
        msg[0].len = write_len;
        msg[0].buf = write_buf;

        ret = i2c_transfer(chip->client->adapter, msg, num);

        if(ret <0) {
                pr_err("i2c transfer error\n");
                return -EIO;
        } else
                return 0;
}

static int asusec_read(struct byt_chip_info *chip, u8 *read_buf, u16 read_len)
{
        struct i2c_msg msg[2];
        int ret;
        int num =2;

        memset(msg,0,sizeof(struct i2c_msg)*2);

        num = 1;

        msg[0].addr = chip->client->addr;
        msg[0].flags = I2C_M_RD;
        msg[0].len = read_len;
        msg[0].buf = read_buf;

        ret = i2c_transfer(chip->client->adapter, msg, num);

        if(ret <0) {
                pr_err("i2c transfer error\n");
                return -EIO;
        } else
                return 0;

}

static int asusec_read_guage(struct byt_chip_info *chip,u8 command, u8 *read_buf, u8 read_len)
{
	u8	 guage_i2c_buf[5];
	u8	guage_status[2];
	u8 status;
	int time_out;
	int ret;

	guage_i2c_buf[0] = 0xC0;
	guage_i2c_buf[1] = 0x02;
	guage_i2c_buf[2] = 0xAA;
	guage_i2c_buf[3] = command;
	guage_i2c_buf[4] = read_len;

	ret = asusec_write(chip,guage_i2c_buf,5);

	if(ret)
		return -EIO;

	guage_status[0] = 0xC0;
	guage_status[1] = 0x01;

	time_out = 1000;
	do{
		asusec_write_read(chip,guage_status,2,&status,1);
		if(status == 0x02) //read success
			break;
		msleep(1);
	}while(--time_out);

        if(!time_out) {
                pr_err("time out\n");
		return -EIO;
	}
	asusec_write_read(chip,guage_i2c_buf,2,read_buf,read_len);

	return 0;
}

static int asusec_write_guage(struct byt_chip_info *chip,u8 command, u8 *write_buf, u8 write_len)
{
        u8       *guage_i2c_buf;
        u8      guage_status[2];
        u8 status;
	int i=0;
	int time_out = 10;
	int ret;

	guage_i2c_buf = kzalloc(5+write_len, GFP_KERNEL);

        guage_i2c_buf[0] = 0xC0;
        guage_i2c_buf[1] = 0x03;
        guage_i2c_buf[2] = 0xAA;
        guage_i2c_buf[3] = command;
        guage_i2c_buf[4] = write_len;

	for(i=0;i<write_len;i++)
	{
		guage_i2c_buf[5+i] = write_buf[i];
	}

        ret = asusec_write(chip,guage_i2c_buf,5+write_len);
	
	kfree(guage_i2c_buf);	

	if(ret) 
		return -EIO;

        guage_status[0] = 0xC0;
        guage_status[1] = 0x01;

	time_out = 1000;
        do{
                asusec_write_read(chip,guage_status,2,&status,1);
                if(status == 0x01) //write success
                        break;
                msleep(1);
        }while(--time_out);

	if(!time_out) {
		pr_err("time out\n");
		return -EIO;
	}

        return 0;
}

static enum power_supply_property byt_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,

};

static enum power_supply_property byt_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static inline struct byt_chip_info *
to_byt_chip_info(struct power_supply *psy)
{
	return container_of(psy, struct byt_chip_info, bat);  //chip
}

static inline struct power_supply *to_power_supply(struct device *dev)
{
	return dev_get_drvdata(dev);    //&chip->bat
}

static int byt_ac_status(struct byt_chip_info *chip)
{
        int ret = 0;
        u8 status;

        status = (chip->ecbat.ChargerStatusRegister>>6) &0x03;
		//printk(KERN_ALERT"Byt_battery.c, byt_ac_status=%d",status);
        if(status == 0x00 ) { //non charge source
                ret  = 0;
        }else if (status == 0x01) { //DC USB charger
                ret = 0;
        }else if(status == 0x02) { //AC
                ret = 1;
        }else if(status == 0x03) { //OTG
		ret = 0;
	}

        return ret;
}


static int byt_get_ac_property(struct power_supply *psy,
                                        enum power_supply_property psp,
                                        union power_supply_propval *val)
{
        int ret = 0;
        struct byt_chip_info *chip = container_of(psy,
                                struct byt_chip_info, ac);
		
        //pr_err("byt_get_ac_property psp = %x\n",psp);


        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                val->intval = byt_ac_status(chip);
                break;
		default:
			
			return -EINVAL;

		}

	
	return ret;
}


static int byt_battery_status(struct byt_chip_info *chip)
{
	int ret = POWER_SUPPLY_STATUS_UNKNOWN;
	u8 status;

	status = (chip->ecbat.ChargerStatusRegister>>4) &0x03;

	if(status == 0x01 || status == 0x02) { 
		ret  = POWER_SUPPLY_STATUS_CHARGING;
	}else if (status == 0x00) {
		ret = POWER_SUPPLY_STATUS_DISCHARGING; 
	}else if(status == 0x03) {
		ret = POWER_SUPPLY_STATUS_FULL;
	}

	return ret;
}
static inline int byt_get_capacity(struct byt_chip_info *chip)
{
	int ret ;
	if(chip->ecbat.FullChargeCapacity!=0)
		ret = (100*chip->ecbat.RemainingCapacity) / chip->ecbat.FullChargeCapacity;
	else
		ret = 0;
	return ret;
}


static int byt_get_battery_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{

	int ret = 0, batt_volt, cur_avg;
	struct byt_chip_info *chip = container_of(psy,
				struct byt_chip_info, bat);
	


	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = byt_battery_status(chip);
		break;
	
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = byt_get_capacity(chip);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->ecbat.Voltage * 1000;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = chip->ecbat.RemainingCapacity * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = chip->ecbat.FullChargeCapacity* 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = chip->ecbat.DesignCapacity * 1000;
		break;
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = chip->ecbat.CycleCount;
		break;	

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = chip->ecbat.Temperature;
		break;

	default:
		
		return -EINVAL;
	}
	
	return 0;

i2c_read_err:
	
	return ret;
}

//TODO error handle
static int get_lid_status( struct byt_chip_info *chip) 
{
	u8	offset;
	u8	lid_status = 0x00;

	offset = 0x83;

	asusec_write_read(chip,&offset,1,&lid_status,1);

	printk(KERN_ALERT"get_lid_status=%d\n", lid_status);

	return lid_status;

}



static int byt_battery_update( struct byt_chip_info *chip) 
{
	u8 offset;
	int ret;
		
	memset(&chip->ecbat,0,sizeof(struct byt_battery));

	offset = 0x81;

	ret = asusec_write_read(chip,&offset,1,&chip->ecbat,sizeof(chip->ecbat));

	return ret;

}

static int get_EC_version( struct byt_chip_info *chip) 
{	
        u8 offset;
        int ret;

	offset = 0xEC;

	ret = asusec_write_read(chip,&offset,1,chip->ec_ver,9);
	
	if(!ret)
		printk(KERN_ALERT"EC_version=%s\n", chip->ec_ver);
	
	return ret;
}

//TODO error handle
static int get_event_ID( struct byt_chip_info *chip) 
{
        u8      offset;
        u8      event_id = 0x00;
	int 	ret;

        offset = 0x84;

        ret = asusec_write_read(chip,&offset,1,&event_id,1);

	if(!ret)
        	printk(KERN_ALERT"get_event_ID = %x\n", event_id);

        return event_id;

}

static irqreturn_t byt_thread_handler(int id, void *dev)
{
	struct byt_chip_info *chip = dev;
	int ret, event_id, lid_value;
	printk(KERN_ALERT"sleep 100 ms..\n");
	msleep(100);
	//LID_Open 0x82 ;  LID_Close 0x83 ; Ac In Out 0xA0 ; Battery Charge 0xA3
	event_id = get_event_ID(chip);
	printk(KERN_ALERT"byt_thread_handler: event_ID = %x\n", event_id);

	if(event_id ==0x82 || event_id == 0x83){
		lid_value=get_lid_status(chip);
		input_report_switch(chip->lid_dev, SW_LID, !lid_value);
		input_sync(chip->lid_dev);
		printk(KERN_ALERT"handler :LID = %s \n", lid_value ? "opened" : "closed");
		}
	//*/
	else{
		mutex_lock(&chip->lock);
		byt_battery_update(chip);
     	
	    printk(KERN_ALERT"Charger interrupt, update battery info\n");
		{
			int i;
			u8 *p = (u8*)&chip->ecbat;
			for(i=0;i<sizeof(chip->ecbat);i++,p++){
				pr_err("%x = %x\n",i,*p);
			}
		
		}

	
		mutex_unlock(&chip->lock);

		power_supply_changed(&chip->ac);
		power_supply_changed(&chip->bat);
	}
	
	
	
	return IRQ_HANDLED;
}


static void byt_init_irq(struct byt_chip_info *chip)
{
	int ret = 0;
	int gpio_num;

	/* get kernel GPIO number */
	gpio_num = acpi_get_gpio("\\_SB.GPO2", 18);
	printk(KERN_ALERT"Gpio_num=%d\n", gpio_num);
	/* get irq number */
	chip->client->irq = gpio_to_irq(gpio_num);
	printk(KERN_ALERT"IRQ=%d\n", chip->client->irq);

	ret = gpio_request(gpio_num, "asus_byt-battery");
	if (ret < 0) {
		pr_err("%s: Unable to request gpio %d\n",__func__, gpio_num);
	}

	ret = gpio_direction_input(gpio_num);
	if (ret < 0) {
		pr_err("%s: Unable to set direction for gpio %d\n", __func__,gpio_num);
	}

	/* register interrupt */
	ret = request_threaded_irq(chip->client->irq, NULL,
					byt_thread_handler,
					IRQF_TRIGGER_FALLING,
					"asus_byt-battery", chip);
	if (ret) {
		dev_warn(&chip->client->dev,
			"cannot get IRQ:%d\n", chip->client->irq);
		chip->client->irq = -1;
	} else {
		dev_info(&chip->client->dev, "IRQ No:%d\n", chip->client->irq);
	}
}


static void byt_batt_info_update_work_func(struct work_struct *work)
{
	int delay_time =10;
	struct byt_chip_info *chip;
	chip = container_of(work, struct byt_chip_info, byt_batt_info_update_work.work);

	mutex_lock(&chip->lock);
	byt_battery_update(chip);
	mutex_unlock(&chip->lock);
	
	power_supply_changed(&chip->bat);
/*
	u16 mycurrent = chip->ecbat.AverageCurrent;
	short mycurrent2 = chip->ecbat.AverageCurrent;
	if(chip->ecbat.AverageCurrent & 0x8000){
		 mycurrent= 0xffff - chip->ecbat.AverageCurrent;
		 printk(KERN_ALERT"--- chip->ecbat.AverageCurrent = 0x%x , AverageCurrent=%x : %x\n",chip->ecbat.AverageCurrent, mycurrent, mycurrent2);
	}
	else{
		printk(KERN_ALERT"+++ chip->ecbat.AverageCurrent = 0x%x, AverageCurrent=%x : %x\n", chip->ecbat.AverageCurrent,mycurrent, mycurrent2);
	}
*/
	
	queue_delayed_work(byt_wq, &chip->byt_batt_info_update_work, delay_time*HZ);

}


///*


static ssize_t byt_battery_show_charge_status(struct device *class,struct device_attribute *attr,char *buf){	
	int status = byt_battery_status(byt_chip)+1;
	return sprintf(buf, "%d\n", status);
 
}
static ssize_t byt_battery_show_current_now(struct device *class,struct device_attribute *attr,char *buf){	
	short mycurrent2 = byt_chip->ecbat.AverageCurrent;
	return sprintf(buf, "%d\n", mycurrent2);
 
}
static ssize_t ec_version_show(struct device *class,struct device_attribute *attr,char *buf){	
	return sprintf(buf, "%s\n", byt_chip->ec_ver);
 
}

static ssize_t byt_ec_set_wakeup_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//write "1" to sys file "set_wakeup_flag"        
	struct power_supply *psy = to_power_supply(dev);
	struct byt_chip_info *chip = to_byt_chip_info(psy);
	int ret;
	u8 write_buf[2];	

        write_buf[0] = EC_SET_AUTO_WAKEUP_REG;
        write_buf[1] = EC_SET_AUTO_WAKEUP_CMD_FLAG;
	ret = asusec_write(chip,write_buf,2);

	if(ret)
		return 0;
	else
		return count;
}

static ssize_t byt_ec_set_wakeup_timer(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//write a BYTE to sys file "set_wakeup_timer", wakeup time = 0~255 sec        
	struct power_supply *psy = to_power_supply(dev);
	struct byt_chip_info *chip = to_byt_chip_info(psy);
	int ret;
	u8 write_buf[3];	

        write_buf[0] = EC_SET_AUTO_WAKEUP_REG;
        write_buf[1] = EC_SET_AUTO_WAKEUP_CMD_TIMER;
        write_buf[2] = buf[0];    //wake up time
	
	ret = asusec_write(chip,write_buf,3);

        if(ret)
                return 0;
        else
                return count;
}

static DEVICE_ATTR(battery_charge_status, S_IRUGO | S_IWUSR,
	byt_battery_show_charge_status, NULL);
static DEVICE_ATTR(battery_current_now, S_IRUGO | S_IWUSR,
	byt_battery_show_current_now, NULL);
static DEVICE_ATTR(ec_version, S_IRUGO | S_IWUSR,
	ec_version_show, NULL);
static DEVICE_ATTR(set_wakeup_flag, S_IRUGO | S_IWUSR,
	NULL, byt_ec_set_wakeup_flag);
static DEVICE_ATTR(set_wakeup_timer, S_IRUGO | S_IWUSR,
	NULL, byt_ec_set_wakeup_timer);


static struct attribute *byt_attributes[] = {
	&dev_attr_battery_charge_status.attr,
	&dev_attr_battery_current_now.attr,
	&dev_attr_ec_version.attr,
	&dev_attr_set_wakeup_flag.attr,
	&dev_attr_set_wakeup_timer.attr,
	NULL
};

static const struct attribute_group byt_attr_group = {
	.attrs = byt_attributes,
};

//*/

static void byt_set_lid_bit(struct input_dev *dev){	
	dev->evbit[0] = BIT_MASK(EV_SW);	
	set_bit(SW_LID, dev->swbit);

	printk(KERN_ALERT"byt_set_lid_bit() done.\n");
}



static int byt_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	printk(KERN_ALERT"byt_battery_probe()\n");
	struct byt_chip_info *chip;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
				"SM bus doesn't support BYTE transactions\n");
		return -EIO;
	}
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
				"SM bus doesn't support WORD transactions\n");
		return -EIO;
	}
	
	
	
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "failed to allocate memory\n");
		return -ENOMEM;
	}
	
	chip->client = client;
	chip->pdata = client->dev.platform_data;
	
	i2c_set_clientdata(client, chip);

	byt_chip = chip;
	mutex_init(&chip->lock);

	byt_wq = create_singlethread_workqueue("byt_battery_work_queue");
	INIT_DELAYED_WORK(&chip->byt_batt_info_update_work, byt_batt_info_update_work_func);

	//Hall sensor LID
	chip->lid_dev = input_allocate_device();
	if (!chip->lid_dev) {
		dev_err(&client->dev, "input_allocate_device fail\n");
	}

	chip->lid_dev->name = "asus_byt_lid";	
	chip->lid_dev->phys = "/dev/input/asus_byt_lid";	
	chip->lid_dev->dev.parent = &client->dev;


	byt_set_lid_bit(chip->lid_dev);

	if (input_register_device(chip->lid_dev)) {
			dev_err(&client->dev, "input_allocate_device fail\n");
	}

{
	u16	sub_command = 0x0002;
	u16	fw_version;
	asusec_write_guage(chip,GUAGE_CMD_CNTL,&sub_command,2);
	asusec_read_guage(chip,GUAGE_CMD_CNTL,&fw_version,2);
	pr_err("guage version = %x\n",fw_version);
}	
	
	get_EC_version(chip);
	

	mutex_lock(&chip->lock);
	byt_battery_update(chip);
	mutex_unlock(&chip->lock);

	printk(KERN_ALERT"byt_battery_probe(), update battery info\n");
	{
		int i;
		u8 *p = (u8*)&chip->ecbat;
		for(i=0;i<sizeof(chip->ecbat);i++,p++){
			pr_err("%x = %x\n",i,*p);
		}
		
	}


	//Logo
	if(byt_get_capacity(chip)<= 5 && byt_ac_status(chip)==1){
			int i;
			for(i=0;i<3;i++) {
					charger_logo_display(g_chargerlogo[i]);
					msleep(200);
			}
	}

	else if(byt_get_capacity(chip)<= 5 && byt_ac_status(chip)==0){
			int i;
			for(i=0;i<3;i++) {
					charger_logo_display(g_lowbatlogo[i]);
					msleep(200);
			}
			kernel_power_off();
	}
	/*
	else{
		    int i;
			for(i=0;i<3;i++) {
					charger_logo_display(g_asuslogo[i]);
					msleep(200);
			}
	}
	*/

	chip->ac.name = "byt_ac";
    chip->ac.type = POWER_SUPPLY_TYPE_MAINS;
    chip->ac.properties = byt_ac_props;
    chip->ac.num_properties = ARRAY_SIZE(byt_ac_props);
    chip->ac.get_property = byt_get_ac_property;
    ret = power_supply_register(&client->dev, &chip->ac);
    if (ret) {
            dev_err(&client->dev, "failed to register ac: %d\n", ret);
            goto probe_failed_1;
    }


	chip->bat.name = "byt_battery";
	chip->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->bat.properties = byt_battery_props;
	chip->bat.num_properties = ARRAY_SIZE(byt_battery_props);
	chip->bat.get_property = byt_get_battery_property;
	
	ret = power_supply_register(&client->dev, &chip->bat);
	if (ret) {
		dev_err(&client->dev, "failed to register battery: %d\n", ret);
		goto probe_failed_1;
	}
///*
	ret = sysfs_create_group(&chip->bat.dev->kobj, &byt_attr_group);
	if (ret) {
		dev_err(chip->bat.dev, "failed to create sysfs group\n");
		goto fail_unregister;
	}
//*/
	byt_init_irq(chip);

	queue_delayed_work(byt_wq, &chip->byt_batt_info_update_work, 15*HZ);

	
	
	return 0;
fail_unregister:
	power_supply_unregister(&chip->bat);
probe_failed_1:
	kfree(chip);
	return ret;
}

static int byt_battery_remove(struct i2c_client *client)
{
	printk(KERN_ALERT"byt_battery_remove()\n");
	struct byt_chip_info *chip = i2c_get_clientdata(client);

	sysfs_remove_group(&chip->bat.dev->kobj, &byt_attr_group);
	
	power_supply_unregister(&chip->ac);

	power_supply_unregister(&chip->bat);
	kfree(chip);

	return 0;
}

static void byt_battery_shutdown(struct i2c_client *client)
{
	dev_dbg(&client->dev, "byt_battery shutdown\n");

	struct byt_chip_info *chip = i2c_get_clientdata(client);
	cancel_delayed_work(&chip->byt_batt_info_update_work);

	if (client->irq > 0)
		disable_irq(client->irq);

	return;
}




static const struct i2c_device_id asus_id[] = {
	{ "asus_byt", 0 },
        { },
};
MODULE_DEVICE_TABLE(i2c, asus_id);

static struct i2c_driver byt_battery_driver = {
	.driver = {
		.name = "asus_byt-battery",
		.owner	= THIS_MODULE,
		//.pm	= &ulpmc_pm_ops,
	},
	.probe = byt_battery_probe,
	.remove = byt_battery_remove,
	.id_table = asus_id,
	.shutdown = byt_battery_shutdown,
};

static int __init byt_battery_init(void)
{
	int ret = i2c_add_driver(&byt_battery_driver);
	if (ret)
		printk(KERN_ERR "Unable to register byt_battery i2c driver\n");

	return ret;
}
//device_initcall(byt_battery_init);
late_initcall(byt_battery_init);

static void __exit byt_battery_exit(void)
{
	i2c_del_driver(&byt_battery_driver);
	printk(KERN_ALERT"Delete i2c driver..\n");
}
module_exit(byt_battery_exit);

MODULE_AUTHOR("Hollie");
MODULE_DESCRIPTION("BYT battery driver");
MODULE_LICENSE("GPL");
