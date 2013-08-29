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

	struct mutex lock;
	
};

static struct byt_chip_info *byt_chip;



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

static int byt_battery_update( struct byt_chip_info *chip) 
{
	struct i2c_msg msg[2];
	int ret;
	int num =2;
	u8 write_buf[1];	
	
	memset(&chip->ecbat,0,sizeof(struct byt_battery));
	memset(&msg,0,sizeof(struct i2c_msg)*2);

        num = 2;
        msg[0].addr = chip->client->addr;
        msg[0].flags = !I2C_M_RD;
        msg[0].len = 1;
        write_buf[0] = 0x81;
        msg[0].buf = write_buf;

        msg[1].addr = chip->client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = sizeof(chip->ecbat);
        msg[1].buf = &chip->ecbat;


	ret = i2c_transfer(chip->client->adapter, msg, num);

	if(ret <0) {
		pr_err("i2c_transfer error\n");
		return -EIO;
	} else
		return 0;

}

static int get_EC_version( struct byt_chip_info *chip) 
{	
	struct i2c_msg msg[2];	
	int ret;	
	int num =2;	
	u8 write_buf[1];			
	memset(&chip->ecbat,0,sizeof(struct byt_battery));	
	memset(&msg,0,sizeof(struct i2c_msg)*2);        

		num = 2;        
		msg[0].addr = chip->client->addr;        
		msg[0].flags = !I2C_M_RD;        
		msg[0].len = 1;        
		write_buf[0] = 0xEC;        
		msg[0].buf = write_buf;        
		msg[1].addr = chip->client->addr;        
		msg[1].flags = I2C_M_RD;        
		msg[1].len = 9;        
		msg[1].buf = chip->ec_ver;	

	ret = i2c_transfer(chip->client->adapter, msg, num);	
			
	if(ret <0) {		
		pr_err("i2c_transfer error\n");		
		return -EIO;	
	} else{		
		printk(KERN_ALERT"EC_version=%s\n", chip->ec_ver);
		return 0;
	}
}


static irqreturn_t byt_thread_handler(int id, void *dev)
{
	struct byt_chip_info *chip = dev;
	int ret;
	printk(KERN_ALERT"sleep 100 ms..\n");
	msleep(100);
	
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



static DEVICE_ATTR(battery_charge_status, S_IRUGO | S_IWUSR,
	byt_battery_show_charge_status, NULL);
static DEVICE_ATTR(battery_current_now, S_IRUGO | S_IWUSR,
	byt_battery_show_current_now, NULL);
static DEVICE_ATTR(ec_version, S_IRUGO | S_IWUSR,
	ec_version_show, NULL);



static struct attribute *byt_attributes[] = {
	&dev_attr_battery_charge_status.attr,
	&dev_attr_battery_current_now.attr,
	&dev_attr_ec_version.attr,
	NULL
};

static const struct attribute_group byt_attr_group = {
	.attrs = byt_attributes,
};

//*/



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
//
	byt_wq = create_singlethread_workqueue("byt_battery_work_queue");
	INIT_DELAYED_WORK(&chip->byt_batt_info_update_work, byt_batt_info_update_work_func);

//*/
	
	mutex_lock(&chip->lock);
	get_EC_version(chip);
	mutex_unlock(&chip->lock);

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
