/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen Tom_Shen@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#include "bq27520_proc_force_update.h"

extern struct battery_info_reply batt_info;
static int ret = UPDATE_NONE;
static int dffs_ret = -1;

extern struct battery_dev_info bq27520_dev_info;
extern struct mutex bq27520_dev_info_mutex;

extern struct battery_dev_info batt_upt_dev_info;
extern struct mutex batt_dev_upt_mutex;

const char *ROM_MODE_W = "W: 16";
const char *NORMAL_MODE_W = "W: AA";

ssize_t parsing_dffs_string(char *OriginalMessages, unsigned long len)
{
    char ConvertMessages[293];

    char *delimit=" ";
    char *p;
    u8 MessageCount = 0;
    uint8_t MessageValue[96]={0};
    uint8_t Ccmd_CheckData = 0;
    int status;

    char *MessageTokenTemp;
    char *MessageToken;
    char *nextToken;
    int i;
    char cmd;//Todo[2]
    int slaveAddrValue;
    u8 regAddrValue;
    u8 Ccmd_addr;
    u8 Ccmd_ReadCount;
    int waitmsec;
    //int I2Cerror = 0; //Todo[8]

    struct i2c_client *client_upt = NULL;
    struct i2c_client *client = NULL;
    struct i2c_client *i2c_tmp = NULL;

    mutex_lock(&batt_dev_upt_mutex);
    client_upt = batt_upt_dev_info.i2c;
    mutex_unlock(&batt_dev_upt_mutex);
    if (!client_upt|| !client_upt->adapter)
        return -ENODEV;

    mutex_lock(&bq27520_dev_info_mutex);
    client = bq27520_dev_info.i2c;
    mutex_unlock(&bq27520_dev_info_mutex);
    if (!client|| !client->adapter)
        return -ENODEV;

    cmd = OriginalMessages[0];
    printk("cmd:%c\n",cmd);

    printk("%s\n", OriginalMessages);
    if (!strncmp(ROM_MODE_W, OriginalMessages, 5)) {
        i2c_tmp = client_upt;
        printk("client_upt\n");
    }
    else if (!strncmp(NORMAL_MODE_W, OriginalMessages, 5)) {
        i2c_tmp = client;
        printk("client\n");
    }
    else {
        i2c_tmp = NULL;
        printk("client is NULL\n");
    }

    for(i=0 ; i<293 ;i++)
        ConvertMessages[i] = OriginalMessages[i+3];

    //Todo[2]: check command type, if X need use decimal type to read msleep
    //Todo[3]: if command is C or W, slave address(string[3] string[4]) should be 0x16

    printk("command handle +++\n");

    if ('X' == cmd) {
        MessageTokenTemp = ConvertMessages;
        MessageToken = strsep(&MessageTokenTemp, delimit);
        pr_debug("wait command:%s\n",MessageToken);
        waitmsec = (int)simple_strtol(MessageToken, NULL, 10);
        printk("[BAT][gaugeUp][X_cmd]:%d\n",waitmsec);

        msleep(waitmsec);

        if (4000==waitmsec) {
            TIgauge_LockStep();
        }

        return 1;
	}
    else if (('W' == cmd) || ('C' == cmd)) {
        MessageTokenTemp = ConvertMessages;
        MessageToken = strsep(&MessageTokenTemp, delimit);
        nextToken = MessageTokenTemp;
        pr_debug("slave addr string:%s\n",MessageToken);
        slaveAddrValue = (int)simple_strtol(MessageToken, NULL, 16);
        pr_debug("slave addr string:0X%X\n",slaveAddrValue);

        p=strsep(&nextToken,delimit);
        pr_debug("reg addr sting:%s\n",p);
        regAddrValue=(u8)simple_strtol(p, NULL, 16);//Test_WXC_cmd(int)=>(u8)
        printk("reg addr string:0X%02X\n",regAddrValue);

        //Todo[4]:MessageValue[0];I2C slave address will be 16
        //Todo[5]:MessageValue[1];reg  address
        //Todo[6]:depend on command type(W/C) do different process.
        //Todo[7]:do real I2C command(W/C)
        while ((p=strsep(&nextToken,delimit))) {
            pr_debug("%s\n",p);
            MessageValue[MessageCount]=(uint8_t)simple_strtol(p, NULL, 16);//Test_WXC_cmd(int)=>(uint8_t)
            pr_debug("[%d]0X%02X\n",MessageCount,MessageValue[MessageCount]);
            MessageCount++;
        }

        /* command parser */
        if ('W' == cmd) {
            status = TIgauge_i2c_write(i2c_tmp, regAddrValue, MessageCount, MessageValue);
            if (status > 0) {
                pr_debug("[BAT][gaugeUp][W_cmd][I2C]:success\n");
                return len;
            }
            else {
                printk("[BAT][gaugeUp][W_cmd][I2C]:fail\n");
                return -EAGAIN;
            }
        }
        else if ('C' == cmd) {
            for (Ccmd_ReadCount=0; Ccmd_ReadCount < MessageCount ;Ccmd_ReadCount++) {
                Ccmd_addr = regAddrValue + Ccmd_ReadCount;

                /* Only Compare in ROM mode (No requirement for Normal Mode) */
                status = TIgauge_i2c_read(client_upt, Ccmd_addr, 1, &Ccmd_CheckData);

                if (status > 0) {
                    pr_debug("[BAT][gaugeUp][C_cmd][I2C]:success\n");
                }
                else {
                    printk("[BAT][gaugeUp][C_cmd][I2C]:fail\n");
                    return -EAGAIN;
                }

                if (Ccmd_CheckData != MessageValue[Ccmd_ReadCount]) {
                    printk("[BAT][gaugeUp][C_cmd][Cmp]:fail\n");
                    return -EAGAIN;
                }
                else {
                    printk("[BAT][gaugeUp][C_cmd][Cmp]:success\n");
                    return 1;
                }
            }
        }
    }
    else {
        printk("%s\n", OriginalMessages);
    }

	pr_debug("command handle ---\n");

#if 0
	//Todo[8]: return different error number let /system/bin/tigauge can do retry
	//Todo[8]+++
	if('C' == cmd)
		return -EFBIG;
	else if('W' == cmd)
		return len;
	else if('X' ==cmd)
		return -EPIPE;
	else if( 0!= I2Cerror )
		return -EAGAIN;
	else
#endif
    //return -EIO;
	//Todo[8]---
	return len;//Todo[8]
}

static int bq27520_proc_forceupdate_read(char *page, char **start, off_t off, int count, int *eof, void *date) {
    int len;

    len = sprintf(page,"update status: %d\n", ret);
    return len;
}

static int bq27520_proc_forceupdate_write(struct file *file, const char *buffer, unsigned long count, void *data) {
    ret = UPDATE_NONE;

    BAT_DBG(" %s:\n", __func__);

    if (buffer[0] == '1') {
        // check if is valid battID
        if (!batt_info.isValidBattID)  {
            BAT_DBG("invalid battID not update gauge IC!!!!\n");
            ret = UPDATE_INVALID_BATTID;
        } else {
           /* force update gauge */
           ret = bq27520_bat_upt_main_update_flow(true);
        }
    }

    return count;
}

static int bq27520_proc_enter_rommode_read(char *page, char **start, off_t off, int count, int *eof, void *date) {
    int len;

    len = sprintf(page,"%d\n", ret);
    return len;
}

static int bq27520_proc_enter_rommode_write(struct file *file, const char *buffer, unsigned long count, void *data) {
    ret = -1;

    BAT_DBG(" %s:\n", __func__);

    if (buffer[0] == '1') {
        // check if is valid battID
        if (!batt_info.isValidBattID)  {
            BAT_DBG("invalid battID not update gauge IC!!!!\n");
            ret = UPDATE_INVALID_BATTID;
        } else {
            if (batt_info.gauge_version == IC_VERSION_G4) {
               //unseal for enter rom mode
               ret = bq27520_default_unseal();
               if (ret < 0) {
                   // default password unseal fail
                   BAT_DBG_E("bq27520_default_unseal fail !!!!\n");
                   bq27520_rom_mode_wait(5000);
                   ret = bq27520_ME560CG_unseal();
                   if (ret < 0) {
                      // ME560CG password unseal fail
                      BAT_DBG_E("bq27520_ME560CG_unseal fail !!!!\n");
                      ret = UPDATE_CHECK_MODE_FAIL;
                   }
               }
            }
        }
    }

    return count;
}

static int bq27520_proc_update_from_dffs_read(char *page, char **start, off_t off, int count, int *eof, void *date) {
    int len;

    len = sprintf(page,"%d\n", dffs_ret);
    return len;
}

static int bq27520_proc_update_from_dffs_write(struct file *file, const char *buffer, unsigned long len, void *data) {
    char *OriginalMessages = NULL;
    long rt;

    OriginalMessages = kzalloc(len, GFP_KERNEL);
    if (!OriginalMessages) {
        BAT_DBG( "mem alloc failed\n");
        return -ENOMEM;
    }

    if (len > 296) {
        len = 296;
    }

    if (copy_from_user(OriginalMessages, buffer, len)) {
        printk("copy from user fail !!! \n");
        return -EFAULT;
    }

    BAT_DBG(" %s:%s , %d\n", __func__, OriginalMessages, len);
    dffs_ret = parsing_dffs_string(OriginalMessages, len);
    printk("return:%ld\n",dffs_ret);

    return len;
}

int bq27520_register_gauge_forceupdate_proc_fs(void) {
        struct proc_dir_entry *entry=NULL;

        entry = create_proc_entry("bq27520_gauge_forceupdate", 0666, NULL);
        if (!entry) {
            printk("[%s]Unable to create bq27520 gauge forceupdate \n", __FUNCTION__);
            return -EINVAL;
        }
        entry->read_proc  = bq27520_proc_forceupdate_read;
        entry->write_proc = bq27520_proc_forceupdate_write;

        entry = create_proc_entry("bq27520_gauge_enter_rom_mode", 0664, NULL);
        if (!entry) {
            printk("[%s]Unable to create bq27520 gauge enter rom mode \n", __FUNCTION__);
            return -EINVAL;
        }
        entry->read_proc  = bq27520_proc_enter_rommode_read;
        entry->write_proc = bq27520_proc_enter_rommode_write;

        entry = create_proc_entry("bq27520_gauge_update_from_dffs", 0664, NULL);
        if (!entry) {
            printk("[%s]Unable to create bq27520 gauge update from dffs \n", __FUNCTION__);
            return -EINVAL;
        }
        entry->read_proc  = bq27520_proc_update_from_dffs_read;
        entry->write_proc = bq27520_proc_update_from_dffs_write;
        return 0;
}
