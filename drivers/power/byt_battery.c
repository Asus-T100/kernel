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
#include <linux/firmware.h>

#define TAG "<asus-hollie>"
#define ASUS_EC_FIRMWARE "asusec/t2bt0036.EC"
#define ECSIZE	65536
#define EFLASH_CMD_BYTE_PROGRAM	 	0x02
#define EFLASH_CMD_WRITE_DISABLE 	0x04
#define EFLASH_CMD_READ_STATUS	 	0x05
#define EFLASH_CMD_WRITE_ENABLE		0x06
#define EFLASH_CMD_FAST_READ		0x0B
#define EFLASH_CMD_CHIP_ERASE	 	0x60
#define EFLASH_CMD_READ_ID		0x9F
#define EFLASH_CMD_AAI_WORD_PROGRAM 	0xAD
#define EFLASH_CMD_SECTOR_ERASE	 	0xD7

#define MAX_RETRY_COUNT		5

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
	struct power_supply	ac;

	struct byt_platform_data *pdata;

	struct byt_battery	ecbat;

	struct delayed_work byt_batt_info_update_work;

	u8 ec_ver[9];
	u16 * gaugeIC_fw;
	int chargerIC_current;

	int ec_status_ack;
	int gaugeIC_status_ack;
	int chargerIC_status_ack;

	struct input_dev *lid_dev;

	struct mutex lock;

	u16	ec_fw_addr;
	int 	ec_fw_mode;
	struct bin_attribute ec_firmware_attr;
	u8	*ec_firmware;	
	int 	ec_firmware_size;
};

static struct byt_chip_info *byt_chip;


static int asusec_write_read(struct byt_chip_info *chip,u8 *write_buf, u16 write_len,u8 *read_buf, u16 read_len)
{
	struct i2c_msg msg[2];
	int ret;
	int num =2;
	u16 addr = (chip->ec_fw_mode == 0)?chip->client->addr:chip->ec_fw_addr;

	memset(msg,0,sizeof(struct i2c_msg)*2);

	num = 2;
	msg[0].addr = addr;
	msg[0].flags = !I2C_M_RD;
	msg[0].len = write_len;
	msg[0].buf = write_buf;
	msg[1].addr = addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = read_len;
	msg[1].buf = read_buf;

	ret = i2c_transfer(chip->client->adapter, msg, num);

	if(ret <0) {
		dev_err(&chip->client->dev,"%s i2c transfer error\n", TAG);
		return -EIO;
	} else
		return 0;
}

static int asusec_write(struct byt_chip_info *chip,u8 *write_buf, u16 write_len)
{
	struct i2c_msg msg[2];
	int ret;
	int num =2;
	u16 addr = (chip->ec_fw_mode == 0)?chip->client->addr:chip->ec_fw_addr;

	memset(msg,0,sizeof(struct i2c_msg)*2);

	num = 1;
	msg[0].addr = addr;
	msg[0].flags = !I2C_M_RD;
	msg[0].len = write_len;
	msg[0].buf = write_buf;

	ret = i2c_transfer(chip->client->adapter, msg, num);

	if(ret <0) {
		dev_err(&chip->client->dev,"%s i2c transfer error\n", TAG);
		return -EIO;
	} else
		return 0;
}

static int asusec_read(struct byt_chip_info *chip, u8 *read_buf, u16 read_len)
{
	struct i2c_msg msg[2];
	int ret;
	int num =2;
	u16 addr = (chip->ec_fw_mode == 0)?chip->client->addr:chip->ec_fw_addr;

	memset(msg,0,sizeof(struct i2c_msg)*2);

	num = 1;

	msg[0].addr = addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = read_len;
	msg[0].buf = read_buf;

	ret = i2c_transfer(chip->client->adapter, msg, num);

	if(ret <0) {
		dev_err(&chip->client->dev,"%s i2c transfer error\n", TAG);
		return -EIO;
	} else
		return 0;
}

//payload under 256 can call this function
// S 0x5B [W] [A] 0x17 [A] S 0x5B [W] [A] 0x18 [A] cmd1 [A] payload[0] [A] ...
// ... payload[N] [NA] [P]

int ite_i2c_pre_define_cmd_write(struct byt_chip_info *chip,unsigned char cmdl,unsigned int payload_len,unsigned char payload[])
{
	int ret;
	u8 CSH_cmd = 0x17;           //CS High
	u16 cmd = 0x0018;
	u8	*buf;

	asusec_write(chip,&CSH_cmd,1);

	cmd  = cmd | (cmdl<<8);
	buf =  kzalloc(payload_len+2, GFP_KERNEL);
	memcpy(buf,&cmd,2);
	memcpy(buf+2,payload,payload_len);
	ret = asusec_write(chip,buf,payload_len+2);
	kfree(buf);

	return ret;
}

int ite_i2c_pre_define_cmd_read(struct byt_chip_info *chip,unsigned char cmdl,unsigned int payload_len,unsigned char payload[])
{
	int ret;
	u8 CSH_cmd = 0x17;           //CS High
	u16 cmd = 0x0018;

	asusec_write(chip,&CSH_cmd,1);

	cmd  = cmd | (cmdl<<8);
	ret = asusec_write_read(chip,(u8 *)&cmd,2,payload,payload_len);

	return ret;
}



// S 0x5B [W] [A] 0x17 [A] S 0x5B [W] [A] 0x18 [A] 0x0B [A] S 0x5B [R] payload[0] [A] ...
// ... payload[N] [NA] [P]
int ite_i2c_pre_define_cmd_fastread(struct byt_chip_info *chip,unsigned char addr[],unsigned int payload_len,unsigned char payload[])
{
	int ret;
	u8 CSH_cmd = 0x17;           //CS High
	u8	cmdl = 0x0B;
	u8	Addrbuf[4]; // 0x0b => Fast Read

	Addrbuf[0] = addr[3]; // Address H
	Addrbuf[1] = addr[2]; // Address M
	Addrbuf[2] = addr[1]; // Address L
	Addrbuf[3] = addr[0]; // Dummy

	asusec_write(chip,&CSH_cmd,1);

	ite_i2c_pre_define_cmd_write(chip,cmdl,4,Addrbuf);

	//ret = asusec_write_read(chip,&NULLcmd,1,payload,payload_len);

	ret = asusec_read(chip,payload,payload_len);

	return ret;
}

int i2ec_writebyte(struct byt_chip_info *chip,unsigned int address,unsigned char data)
{
	int ret;
	u8 buf[3];
	u8 cmd;

	//I2EC ADDR WRITE
	cmd = 0x10;
	buf[0] = cmd;
	buf[1] = (address >> 8) & 0xFF;
	buf[2] = (address) & 0xFF; 
	asusec_write(chip,buf,3);

	cmd = 0x11;
	buf[0] = cmd;
	buf[1] = data;
	ret = asusec_write(chip,buf,2);

	return ret;
}

int i2ec_readbyte(struct byt_chip_info *chip,unsigned int address)
{
	int ret;
	u8 buf[3];
	u8 cmd;
	u8 data;	

	//I2EC ADDR WRITE
	cmd = 0x10;
	buf[0] = cmd;
	buf[1] = (address >> 8) & 0xFF;
	buf[2] = (address) & 0xFF;
	asusec_write(chip,buf,3);

	cmd = 0x11;
	ret = asusec_write_read(chip,&cmd,1,&data,1);

	return (int)data;
}

int cmd_write_enable(struct byt_chip_info *chip)
{
	int result;

	result = ite_i2c_pre_define_cmd_write(chip,EFLASH_CMD_WRITE_ENABLE,0,NULL);
	return result;
}

int cmd_write_disable(struct byt_chip_info *chip)
{
	int result;

	result = ite_i2c_pre_define_cmd_write(chip,EFLASH_CMD_WRITE_DISABLE,0,NULL);
	return result;
}

int cmd_erase_all(struct byt_chip_info *chip)
{
	int result;

	result = ite_i2c_pre_define_cmd_write(chip,EFLASH_CMD_CHIP_ERASE,0,NULL);
	return result;
}

unsigned char cmd_check_status(struct byt_chip_info *chip)
{
	unsigned char status[2];

	ite_i2c_pre_define_cmd_read(chip,EFLASH_CMD_READ_STATUS,2,status);
	return status[1];
}

int do_erase_all(struct byt_chip_info *chip)
{
	pr_err("[EC_update]: ERASE.................\n");

	cmd_write_enable(chip);
	while((cmd_check_status(chip) & 0x02)!=0x02);
	cmd_erase_all(chip);
	while(cmd_check_status(chip) & 0x01);
	cmd_write_disable(chip);	

	pr_err("OK\n");

	return 0;
}

int do_program(struct byt_chip_info *chip,UINT8 *data)
{
	int i, result=0;
	unsigned char payload[5]={0,0,0,0,0};//A2,A1,A0,Data0,Data1

	cmd_write_enable(chip);
	while((cmd_check_status(chip) & 0x02)!=0x02);

	payload[3]=data[0]; //Data 0
	payload[4]=data[1]; //Data 1

	result = ite_i2c_pre_define_cmd_write(chip,EFLASH_CMD_AAI_WORD_PROGRAM,5,payload);
	do
	{
		result=cmd_check_status(chip);
	}while(result & 0x01);	

	pr_err("[EC_update]: Program................. ");

	for(i=2;i<65536;)
	{	
		//while(cmd_check_status() & 0x01);
		result = ite_i2c_pre_define_cmd_write(chip,EFLASH_CMD_AAI_WORD_PROGRAM,2,&data[i]);
		do
		{
			result=cmd_check_status(chip);
		}while(result & 0x01);

		if((i%4096)==0)
		{
			pr_err("[EC_update]: Program i=0x%04x \n",i);
		}

		i+=2;
	}

	cmd_write_disable(chip);
	pr_err("[EC_update]: Program...............OK! \n");

	return 0;	
}

int do_check(struct byt_chip_info *chip)
{
	int i, result=0;
	unsigned char address[4] = {0,0,0,0}; // Dummy , L , M , H
	unsigned char *gcbuffer;

	gcbuffer = kzalloc(ECSIZE, GFP_KERNEL);

	for(i=0;i<0x100;i++)
	{
		address[2]=i;
		ite_i2c_pre_define_cmd_fastread(chip,address,0x100,&gcbuffer[0x100*i]);
	}

	for(i=0;i<65536;i++)
	{
		if(gcbuffer[i]!=0xFF)	
		{
			pr_err("[EC_update]: Check Error on offset[%x]; EFLASH=%02x",i,gcbuffer[i]);
			result=-1;
			break;
		}
	}

	if(result==0)
	{
		pr_err("[EC_update]: Check...............");
	}

	return result;
}

int do_verify(struct byt_chip_info *chip)
{
	int i, result=0;
	unsigned char address[4] = {0,0,0,0}; // Dummy , L , M , H
	unsigned char *gvbuffer;	
	int retry_count=0;
	const u8  *data;

	data = chip->ec_firmware;	

	gvbuffer = kzalloc(ECSIZE, GFP_KERNEL);

	for(i=0;i<0x100;i++)
	{
		address[2]=i;
		result = ite_i2c_pre_define_cmd_fastread(chip,address,0x100,&gvbuffer[0x100*i]);
		while(result==-1) {
			result = ite_i2c_pre_define_cmd_fastread(chip,address,0x100,&gvbuffer[0x100*i]);
			retry_count++;
			if(retry_count > MAX_RETRY_COUNT) {
				pr_err("[EC_update]: Do verify over MAX_RETRY_COUNT on address %02x%02x%02x \n",address[3],address[2],address[1]);
				result = -1;
				goto out;
			}	 
		}	 

	}

	for(i=0;i<65536;i++)
	{
		if(gvbuffer[i]!=data[i])
		{
			pr_err("[EC_update]: Verify Error on offset[%x] ; file=%02x EFLASH=%02x \n",i,data[i],gvbuffer[i]);
			result=-1;
			break;
		}
	}

	if(result==0)
	{
		pr_err("[EC_update]: Verify................OK!\n");
	}

	kfree(gvbuffer);

out:

	return result;
}

int get_flash_id(struct byt_chip_info *chip,u8 *flash_id)
{
	int result = 0;

	result = ite_i2c_pre_define_cmd_read(chip,EFLASH_CMD_READ_ID,3,flash_id);

	return result;
}

void do_reset(struct byt_chip_info *chip)
{
	unsigned char tmp1;

	tmp1 = i2ec_readbyte(chip,0x1F01);
	i2ec_writebyte(chip,0x1F01,0x20);
	i2ec_writebyte(chip,0x1F07,0x01);
	i2ec_writebyte(chip,0x1F01,tmp1);

#if 0
/*
	watchdog reset 
	Windows using this method to wait OS shutdown
*/
	unsigned char tmp = 15*1024;

	i2ec_writebyte(chip,0x1F02,0x01);	//Select 1.024kHz

	i2ec_writebyte(chip,0x1F01, 0x10);

	i2ec_writebyte(chip,0x1F09,(tmp>>8)&0xFF);	//Select 

	i2ec_writebyte(chip,0x1F06,tmp&0xFF);
#endif
}

int cmd_enter_flash_mode(struct byt_chip_info *chip)
{
	int ret;
	u8 cmd = 0xEF;

	ret = asusec_write(chip,&cmd,1);

	return ret;
}

int Flash(struct byt_chip_info *chip,u8 *data)
{
	int result, retry_count=0;
retry:	
	if(retry_count > MAX_RETRY_COUNT) {
		pr_err("[ITE_update]: Over MAX_RETRY_COUNT: retry_count=%d \n",retry_count);
		goto err;
	}
	pr_err(" ======= Start Erase ======= \n");		
	do_erase_all(chip);
	pr_err(" ======= Start Check ======= \n");
	result = do_check(chip);
	pr_err(" ======= Check Done ======= \n");
	if(result == -1)
	{
		result = 0;
		retry_count++;
		goto retry;
	}
	pr_err(" ======= Start Program ======= \n");	
	do_program(chip,data);

	pr_err(" ======= Start Verify ======= \n");
	result = do_verify(chip);
	if(result == -1)
	{
		result=0;
		retry_count++;
		goto retry;
	}

	pr_err(" ======= Start reset ======= \n");
	do_reset(chip);
	pr_err("reset ok\n");	

	goto out;

err:
	result=-1;
out:
	return result;
}

static int ec_firmware_update(struct byt_chip_info *chip)
{
	int i;
	u8	DataArray[3] = {0};
	u32	IDDATA = 0x0;
	struct i2c_client *client = chip->client;
	int ret;

	if(chip->ec_firmware == NULL || (chip->ec_firmware_size != ECSIZE)) {
		pr_err("load EC firmware error\n");
		return 0;
	}
		
	cmd_enter_flash_mode(chip);	

	chip->ec_fw_mode = 1;

	msleep(500); //Delay 500ms
	pr_err("enter flash mode\n");

	i =4;
	do
	{
		if(get_flash_id(chip,DataArray) == 0)			   
		{
			IDDATA = *(u32*)DataArray &0x00ffffff;
			pr_err("Read ID Success");
			break;
		}
		else
		{
			msleep(500); //Delay 500ms
		}
		i--;
	}while(i>0);

	if(IDDATA==0xFEFFFF)
	{
		pr_err("Start to update EC firmware.\n");
		Flash(chip,chip->ec_firmware);
	}
	else
	{
		pr_err("Check ID error. %x\n",IDDATA);
		return 0;
	}

	chip->ec_fw_mode = 0;

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
		dev_err(&chip->client->dev,"%s time out\n", TAG);
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
		dev_err(&chip->client->dev,"%s time out\n", TAG);
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

	//dev_info(&chip->client->dev,"%s byt_get_ac_property psp = %x\n", TAG, psp);

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

	dev_info(&chip->client->dev,"%s get_lid_status=%d\n", TAG, lid_status);

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

	if(!ret){
		dev_err(&chip->client->dev,"%s get_EC_version=%s\n", TAG, chip->ec_ver);
	}
	return ret;
}

static int get_gaugeIC_FW( struct byt_chip_info *chip) 
{	
	u16	sub_command = 0x0002;
	u16	fw_version = 0;
	int ret;

	asusec_write_guage(chip,GUAGE_CMD_CNTL,&sub_command,2);
	ret = asusec_read_guage(chip,GUAGE_CMD_CNTL,&fw_version,2);
	if(!ret){
		dev_info(&chip->client->dev,"%s guage version = %x\n", TAG,fw_version);
	}
	chip ->gaugeIC_fw = fw_version;
	return ret;
}

static int get_chargerIC_current(struct byt_chip_info *chip)
{


	u8      charge_i2c_buf[2] = {0xC0,0x05};
	u8      charge_reg[11];
	int i=0, ret;

	ret = asusec_write_read(chip,charge_i2c_buf,2,charge_reg,11);
	/*
	   dev_info(&chip->client->dev,"%s charge:\n", TAG);
	   for(i=0;i<11;i++) {
	   dev_info(&chip->client->dev,"%s chargerIC[%x]=%x", TAG,i,charge_reg[i]);
	   }
	 */
	if(!ret){
		dev_info(&chip->client->dev,"%s chargerIC[0]=%x\n", TAG, charge_reg[0]);
	}

	u8 current_ii = charge_reg[0] & 0x07;
	int current_mA=0;
	dev_info(&chip->client->dev,"%s shift & : current_ii=%x\n", TAG, current_ii);

	switch(current_ii){
		case 0x00:
			current_mA = 100;
			break;
		case 0x01:
			current_mA = 150;
			break;
		case 0x02:
			current_mA = 500;
			break;
		case 0x03:
			current_mA = 900;
			break;
		case 0x04:
			current_mA = 1200;
			break;
		case 0x05:
			current_mA = 1500;
			break;
		case 0x06:
			current_mA = 2000;
			break;
		case 0x07:
			current_mA = 3000;
			break;

	}
	dev_info(&chip->client->dev,"%s current_mA=%d\n", TAG, current_mA);

	chip ->chargerIC_current = current_mA;


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

	if(!ret){
		dev_info(&chip->client->dev,"%s get_event_ID = %x\n", TAG, event_id);
	}
	return event_id;

}

static irqreturn_t byt_thread_handler(int id, void *dev)
{
	struct byt_chip_info *chip = dev;
	int ret, event_id, lid_value;

	if(chip->ec_fw_mode == 1)
		return IRQ_HANDLED;


	dev_info(&chip->client->dev,"%s sleep 100 ms..\n", TAG);
	msleep(100);
	//LID_Open 0x82 ;  LID_Close 0x83 ; Ac In Out 0xA0 ; Battery Charge 0xA3
	event_id = get_event_ID(chip);

	dev_info(&chip->client->dev,"%s byt_thread_handler: event_ID = %x\n", TAG, event_id);

	if(event_id ==0x82 || event_id == 0x83){
		lid_value=get_lid_status(chip);
		input_report_switch(chip->lid_dev, SW_LID, lid_value);
		input_sync(chip->lid_dev);
		dev_info(&chip->client->dev,"%s handler :LID = %s \n", TAG, lid_value ? "closed" : "opened");
	}
	//*/
	else{
		mutex_lock(&chip->lock);
		byt_battery_update(chip);

		dev_info(&chip->client->dev,"%s Charger interrupt, update battery info\n", TAG);
		{
			int i;
			u8 *p = (u8*)&chip->ecbat;
			for(i=0;i<sizeof(chip->ecbat);i++,p++){
				dev_info(&chip->client->dev,"%s %x = %x\n", TAG, i,*p);
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
	//printk(KERN_ALERT"Gpio_num=%d\n", gpio_num);
	dev_info(&chip->client->dev,"%s Gpio_num=%d\n", TAG,gpio_num);
	/* get irq number */
	chip->client->irq = gpio_to_irq(gpio_num);
	//printk(KERN_ALERT"IRQ=%d\n", chip->client->irq);
	dev_info(&chip->client->dev,"%s IRQ=%d\n", TAG,chip->client->irq);

	ret = gpio_request(gpio_num, "asus_byt-battery");
	if (ret < 0) {
		dev_err(&chip->client->dev,"%s %s: Unable to request gpio %d\n", TAG,__func__, gpio_num);
	}

	ret = gpio_direction_input(gpio_num);
	if (ret < 0) {
		dev_err(&chip->client->dev,"%s %s: Unable to set direction for gpio %d\n", TAG, __func__,gpio_num);
	}

	/* register interrupt */
	ret = request_threaded_irq(chip->client->irq, NULL,
			byt_thread_handler,
			IRQF_TRIGGER_FALLING,
			"asus_byt-battery", chip);
	if (ret) {
		dev_err(&chip->client->dev,
				"%s cannot get IRQ:%d\n", TAG, chip->client->irq);
		chip->client->irq = -1;
	} else {
		dev_err(&chip->client->dev, "%s IRQ No:%d\n", TAG, chip->client->irq);
	}
}


static void byt_batt_info_update_work_func(struct work_struct *work)
{
	int delay_time =10;
	struct byt_chip_info *chip;
	chip = container_of(work, struct byt_chip_info, byt_batt_info_update_work.work);

	if(chip->ec_fw_mode == 1)
		return;

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

static ssize_t byt_battery_show_capacity(struct device *class,struct device_attribute *attr,char *buf){	
	int cap = byt_get_capacity(byt_chip);
	return sprintf(buf, "%d\n", cap);

}



static ssize_t ec_version_show(struct device *class,struct device_attribute *attr,char *buf){	
	return sprintf(buf, "%s\n", byt_chip->ec_ver);

}

static ssize_t gaugeIC_FW_show(struct device *class,struct device_attribute *attr,char *buf){	
	return sprintf(buf, "%x\n", byt_chip->gaugeIC_fw);

}

static ssize_t chargeIC_current_show(struct device *class,struct device_attribute *attr,char *buf){		
	/*
	   get_chargerIC_current(byt_chip);
	   return sprintf(buf, "%d\n", byt_chip->chargerIC_current); 
	 */
	short mycurrent2 = byt_chip->ecbat.AverageCurrent;
	if(mycurrent2>0){
		return sprintf(buf, "%d\n", mycurrent2);
	}
	else{
		return sprintf(buf, "%d\n", 0);
	}
}

static ssize_t ec_status_show(struct device *class,struct device_attribute *attr,char *buf){	
	if(!get_EC_version(byt_chip)){
		byt_chip->ec_status_ack = 1;
		//dev_err(&byt_chip->client->dev,"%s ec_status_ack = %d\n", TAG,byt_chip->ec_status_ack);
	}
	else{
		byt_chip->ec_status_ack = 0;
	}
	return sprintf(buf, "%d\n", byt_chip->ec_status_ack);

}

static ssize_t gaugeIC_status_show(struct device *class,struct device_attribute *attr,char *buf){	
	if(!get_gaugeIC_FW(byt_chip)){
		byt_chip->gaugeIC_status_ack =1;
	}
	else{
		byt_chip->gaugeIC_status_ack = 0;
	}

	return sprintf(buf, "%d\n", byt_chip->gaugeIC_status_ack);

}

static ssize_t chargerIC_status_show(struct device *class,struct device_attribute *attr,char *buf){	
	if(!get_chargerIC_current(byt_chip)){
		byt_chip->chargerIC_status_ack = 1;
	}
	else{
		byt_chip->chargerIC_status_ack = 0;
	}
	return sprintf(buf, "%d\n", byt_chip->chargerIC_status_ack);

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

static ssize_t ec_firmware_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct byt_chip_info *chip = dev_get_drvdata(dev);
	
	if(chip->ec_firmware_size < ECSIZE) {
		if(chip->ec_firmware_size == 0) {
			pr_err("current ec version = %s\n",chip->ec_ver);
			chip->ec_firmware = kzalloc(ECSIZE, GFP_KERNEL);
			chip->ec_firmware_size = 0;		
		}
		if(chip->ec_firmware) {
			pr_err("count = %x off = %lx \n",count,off);
			memcpy(chip->ec_firmware+off,buf,count);
			chip->ec_firmware_size += count;
			if(chip->ec_firmware_size == ECSIZE) {
				ec_firmware_update(chip);
				kfree(chip->ec_firmware);
			}
		}
	}
	
	return count;
}


static DEVICE_ATTR(battery_charge_status, S_IRUGO | S_IWUSR,
		byt_battery_show_charge_status, NULL);
static DEVICE_ATTR(battery_current_now, S_IRUGO | S_IWUSR,
		byt_battery_show_current_now, NULL);
static DEVICE_ATTR(battery_status, S_IRUGO | S_IWUSR,
		byt_battery_show_capacity, NULL);

static DEVICE_ATTR(ec_version, S_IRUGO | S_IWUSR,
		ec_version_show, NULL);
static DEVICE_ATTR(gaugeIC_FW, S_IRUGO | S_IWUSR,
		gaugeIC_FW_show, NULL);
static DEVICE_ATTR(chargerIC_inputcurrent, S_IRUGO | S_IWUSR,	
		chargeIC_current_show, NULL);

static DEVICE_ATTR(ec_status, S_IRUGO | S_IWUSR,
		ec_status_show, NULL);
static DEVICE_ATTR(gaugeIC_status, S_IRUGO | S_IWUSR,
		gaugeIC_status_show, NULL);
static DEVICE_ATTR(chargerIC_status, S_IRUGO | S_IWUSR,	
		chargerIC_status_show, NULL);

static DEVICE_ATTR(set_wakeup_flag, S_IRUGO | S_IWUSR,
		NULL, byt_ec_set_wakeup_flag);
static DEVICE_ATTR(set_wakeup_timer, S_IRUGO | S_IWUSR,
		NULL, byt_ec_set_wakeup_timer);

static struct attribute *byt_attributes[] = {
	&dev_attr_battery_charge_status.attr,
	&dev_attr_battery_current_now.attr,
	&dev_attr_battery_status.attr,
	&dev_attr_ec_version.attr,
	&dev_attr_gaugeIC_FW.attr,
	&dev_attr_chargerIC_inputcurrent.attr,
	&dev_attr_ec_status.attr,
	&dev_attr_gaugeIC_status.attr,
	&dev_attr_chargerIC_status.attr,
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
}



static int byt_battery_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	dev_info(&client->dev,"%s byt_battery_probe()\n", TAG);
	struct byt_chip_info *chip;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
				"%s SM bus doesn't support BYTE transactions\n", TAG);
		return -EIO;
	}
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev,
				"%s SM bus doesn't support WORD transactions\n", TAG);
		return -EIO;
	}



	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "%s failed to allocate memory\n", TAG);
		return -ENOMEM;
	}

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	chip->ec_fw_mode = 0;
	chip->ec_fw_addr = 0x5B;
	chip->ec_firmware = NULL;
	chip->ec_firmware_size = 0;

	i2c_set_clientdata(client, chip);

	byt_chip = chip;
	mutex_init(&chip->lock);

	byt_wq = create_singlethread_workqueue("byt_battery_work_queue");
	INIT_DELAYED_WORK(&chip->byt_batt_info_update_work, byt_batt_info_update_work_func);

	//Hall sensor LID
	chip->lid_dev = input_allocate_device();
	if (!chip->lid_dev) {
		dev_err(&client->dev, "%s input_allocate_device fail\n", TAG);
	}

	chip->lid_dev->name = "asus_byt_lid";	
	chip->lid_dev->phys = "/dev/input/asus_byt_lid";	
	chip->lid_dev->dev.parent = &client->dev;


	byt_set_lid_bit(chip->lid_dev);

	if (input_register_device(chip->lid_dev)) {
		dev_err(&client->dev, "%s input_allocate_device fail\n", TAG);
	}

	get_gaugeIC_FW(chip);
	get_EC_version(chip);

	mutex_lock(&chip->lock);
	byt_battery_update(chip);
	mutex_unlock(&chip->lock);

	dev_info(&client->dev,"%s byt_battery_probe : get battery info\n", TAG);
	{
		int i;
		u8 *p = (u8*)&chip->ecbat;
		for(i=0;i<sizeof(chip->ecbat);i++,p++){
			dev_info(&client->dev,"%s %x = %x\n",i,*p);
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
		dev_err(&client->dev, "%s failed to register ac: %d\n", TAG, ret);
		goto probe_failed_1;
	}


	chip->bat.name = "byt_battery";
	chip->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->bat.properties = byt_battery_props;
	chip->bat.num_properties = ARRAY_SIZE(byt_battery_props);
	chip->bat.get_property = byt_get_battery_property;

	ret = power_supply_register(&client->dev, &chip->bat);
	if (ret) {
		dev_err(&client->dev, "%s failed to register battery: %d\n", TAG, ret);
		goto probe_failed_1;
	}
	///*
	ret = sysfs_create_group(&chip->bat.dev->kobj, &byt_attr_group);
	if (ret) {
		dev_err(chip->bat.dev, "%s failed to create sysfs group\n", TAG);
		goto fail_unregister;
	}
	//*/
	byt_init_irq(chip);

	queue_delayed_work(byt_wq, &chip->byt_batt_info_update_work, 15*HZ);


	sysfs_bin_attr_init(&chip->ec_firmware_attr);
	chip->ec_firmware_attr.attr.name = "ec_firmware";
	chip->ec_firmware_attr.attr.mode = S_IRUGO | S_IWUSR;
	chip->ec_firmware_attr.read = NULL;
	chip->ec_firmware_attr.write = ec_firmware_write;
	chip->ec_firmware_attr.size = 0;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &chip->ec_firmware_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			chip->ec_firmware_attr.attr.name);
	}


	return 0;
fail_unregister:
	power_supply_unregister(&chip->bat);
probe_failed_1:
	kfree(chip);
	return ret;
}

static int byt_battery_remove(struct i2c_client *client)
{
	dev_info(&client->dev,"%s byt_battery_remove()\n", TAG);
	struct byt_chip_info *chip = i2c_get_clientdata(client);

	sysfs_remove_group(&chip->bat.dev->kobj, &byt_attr_group);

	power_supply_unregister(&chip->ac);

	power_supply_unregister(&chip->bat);
	kfree(chip);

	return 0;
}

static void byt_battery_shutdown(struct i2c_client *client)
{
	dev_info(&client->dev, "byt_battery shutdown\n", TAG);

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
	if (ret){
		pr_err("%s Unable to register byt_battery i2c driver\n", TAG);
	}
	return ret;
}
//device_initcall(byt_battery_init);
late_initcall(byt_battery_init);

static void __exit byt_battery_exit(void)
{
	i2c_del_driver(&byt_battery_driver);
	pr_err("%s Delete byt_battery i2c driver..\n", TAG);
}
module_exit(byt_battery_exit);

MODULE_AUTHOR("Hollie");
MODULE_DESCRIPTION("BYT battery driver");
MODULE_LICENSE("GPL");
