/* drivers/input/touchscreen/NVTtouch_205.c
 *
 * Copyright (C) 2010 - 2014 Novatek, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/unistd.h>
#include <linux/acpi_gpio.h>
#include <linux/acpi.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/slab.h>

#include <linux/input/mt.h>


// #include <plat/board.h>
// #include <plat/common.h>


#include "nt11205_me176c.h"

#if BOOT_UPDATE_FIRMWARE
#include "nt11205_fw_me176c.h"
#endif


struct i2c_client *nvt_client;
struct nvt_ts_data *ts;
static struct workqueue_struct *nvt_wq;


#ifdef CONFIG_HAS_EARLYSUSPEND
static void nvt_ts_early_suspend(struct early_suspend *h);
static void nvt_ts_late_resume(struct early_suspend *h);
#endif

#if NVT_TOUCH_CTRL_DRIVER
#define PROC_DEVICE_NAME    "NVTflash"
#endif

#if ReportRate
struct timeval TimeOrgin;
struct timeval TimeNow;
int TouchCount;
#endif

#if NVT_TOUCH_SYSFS_NODE
static struct kobject *android_touch_kobj = NULL;
static unsigned char debug_level_cmd = 0;
static uint8_t FW_VER_buff[1];
#endif

static unsigned int Touch_flag = 0;
static unsigned int Flag_print_point = 0;

/*******************************************************
 I2C Read & Write
*******************************************************/
static int CTP_I2C_READ (struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
    struct i2c_msg msgs[2];
    int ret = -1;
    int retries = 0;

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = address;
    msgs[0].len   = 1;
    msgs[0].buf   = &buf[0];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = address;
    msgs[1].len   = len-1;
    msgs[1].buf   = &buf[1];

    while(retries < 5)
    {
        ret = i2c_transfer(client->adapter, msgs, 2);
        if(ret == 2)    break;
        retries++;
    }
    return ret;
}

static int CTP_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
    struct i2c_msg msg;
    int ret = -1;
    int retries = 0;

    msg.flags = !I2C_M_RD;
    msg.addr  = address;
    msg.len   = len;
    msg.buf   = data;

    while(retries < 5)
    {
        // NVT_INFO("I2C Write, retry = %d. Address = %d.", retries, address);
        ret = i2c_transfer(client->adapter, &msg, 1);
        if(ret == 1)    break;
        retries++;
    }
    return ret;
}


/*******************************************************
  IC Reset using GPIO trigger
*******************************************************/
void nvt_hw_reset(void)
{
    NVT_INFO("Enter HW Reset...");
    //---trigger rst-pin to reset (pull low for 10ms)---
    gpio_set_value(NVTTOUCH_RST_GPIO, 1);
    msleep(20);
    gpio_set_value(NVTTOUCH_RST_GPIO, 0);
    msleep(10);
    gpio_set_value(NVTTOUCH_RST_GPIO, 1);
}


/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_CTRL_DRIVER
int nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
    struct i2c_msg msgs[2];
    char *str;
    int ret=-1;
    int retries = 0;
    file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
    str = file->private_data;
    ret=copy_from_user(str, buff, count);

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = str[0];
    msgs[0].len   = str[1];
    msgs[0].buf   = &str[2];

    while(retries < 20)
    {
        ret = i2c_transfer(ts->client->adapter, msgs, 1);
        if(ret == 1)
            break;
        else
            NVT_ERROR("I2C write error %d", retries);

        retries++;
    }
    return ret;
}

int nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
    struct i2c_msg msgs[2];
    char *str;
    int ret = -1;
    int retries = 0;
    file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
    str = file->private_data;
    if(copy_from_user(str, buff, count))
    return -EFAULT;

    msgs[0].flags = !I2C_M_RD;
    msgs[0].addr  = str[0];
    msgs[0].len   = 1;
    msgs[0].buf   = &str[2];

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = str[0];
    msgs[1].len   = str[1]-1;
    msgs[1].buf   = &str[3];

    while(retries < 20)
    {
        ret = i2c_transfer(ts->client->adapter, msgs, 2);
        if(ret == 2)
            break;
        else
            NVT_ERROR("I2C read error %d", retries);

        retries++;
    }
    ret=copy_to_user(buff, str, count);
    return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
    struct nvt_flash_data *dev;

    dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
    if(dev == NULL)
        return -ENOMEM;

    rwlock_init(&dev->lock);
    file->private_data = dev;

    return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
    struct nvt_flash_data *dev = file->private_data;

    if(dev)
        kfree(dev);

    return 0;
}

struct file_operations nvt_flash_fops = {
    .owner = THIS_MODULE,
    .open = nvt_flash_open,
    .release = nvt_flash_close,
    .write = nvt_flash_write,
    .read = nvt_flash_read,
};

static int nvt_flash_init(void)
{
    int ret=0;

    struct proc_dir_entry *entry = NULL;

    entry = proc_create(PROC_DEVICE_NAME, 0664, NULL, &nvt_flash_fops);
    if (!entry)
    {
        NVT_ERROR("Proc nvt_flash_fops file create failed!");
        return -EINVAL;
    }

    printk("============================================================\n");
    printk("NVT_flash driver loaded\n");
    printk("============================================================\n");
    return 0;
}
#endif

/*******************************************************
  Create Device Node (Sysfs Entry)
*******************************************************/
#if NVT_TOUCH_SYSFS_NODE
static ssize_t novatek_debug_level_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char messages[80] = {0};

    if (count >= 80)
    {
        NVT_ERROR("No command exceeds 80 chars.");
        return -EFAULT;
    }

    strcpy(messages, buf);

    if (messages[0] == 'p') //open report data log
    {
        debug_level_cmd = messages[0];

        if( messages[2] == '1') //enable report data log
        {
            Flag_print_point = 1;
        }
        else if( messages[2] == '0') //disable report data log
        {
            Flag_print_point = 0;
        }
        else
        {
            NVT_ERROR("debug_level command = 'p' , parameter error.");
        }
        return count;
    }
}

static ssize_t novatek_debug_level_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int ret_value = 0;

    if(debug_level_cmd == 'p')
    {
        if(Flag_print_point)
        {
            ret += sprintf(buf, "Enable report data log\n");
        }
        else
        {
            ret += sprintf(buf, "Disable report data log\n");
        }
    }

    return ret;
}
static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO|S_IWGRP), novatek_debug_level_read, novatek_debug_level_write);

static struct attribute *novatek_attr[] =
{
    &dev_attr_debug_level.attr,
    NULL
};

static int novatek_touch_sysfs_init(void)
{
    int ret;
    android_touch_kobj = kobject_create_and_add("android_touch", NULL);
    if (android_touch_kobj == NULL)
    {
        NVT_ERROR("Sysfs Subsystem register failed.");
        ret = -ENOMEM;
        return ret;
    }

    ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
    if (ret)
    {
        NVT_ERROR("Create_file debug_level failed.");
        return ret;
    }

    return 0;
}

static void novatek_touch_sysfs_deinit(void)
{
    sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
    kobject_del(android_touch_kobj);
}
#endif

/*******************************************************
  Print Report Rate
*******************************************************/
#if ReportRate
int ShowReportRate(void)
{
    if(TouchCount==0)
        do_gettimeofday(&TimeOrgin);

    do_gettimeofday(&TimeNow);
    if(TimeNow.tv_sec>TimeOrgin.tv_sec)
    {
        do_gettimeofday(&TimeOrgin);
        return 1;
    }
    else
    {
        return 0;
    }
}
#endif


/*******************************************************
  Auto Update FW in Probe
*******************************************************/
#if BOOT_UPDATE_FIRMWARE
int Check_FW_Ver(void)
{
    NVT_INFO("Start to check Firmware version.");
    uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int ret = 0;

    I2C_Buf[0] = 0x78;
    ret = CTP_I2C_READ(ts->client, ts->client->addr, I2C_Buf, 2);
    if (ret != 2)
    {
        NVT_ERROR("i2c read failed. error = %d. ", ret);
    }
    NVT_INFO("IC FW Ver = %d", I2C_Buf[1]);
    NVT_INFO("Bin FW Ver = %d", BUFFER_DATA[0x7F00]); //0x7F00 = 32,512
    if(I2C_Buf[1]>=BUFFER_DATA[0x7F00])
        return 1;
    else
        return 0;
}

int Check_CheckSum(void)
{
    NVT_INFO("Start to check Checksum.");
    uint8_t I2C_Buf[64];
    uint8_t buf2[64];
    int i, j, k, Retry_Counter=0;
    int addr=0;
    int ret=0;
    uint8_t addrH, addrL;
    unsigned short RD_Filechksum, WR_Filechksum;

    WR_Filechksum = 0;

    I2C_Buf[0]=0x00;
    I2C_Buf[1]=0x5A;
    ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    msleep(1000);


    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3F;
    I2C_Buf[2]=0xE8;
    ret = CTP_I2C_WRITE(ts->client, ts->client->addr, I2C_Buf, 3);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    I2C_Buf[0]=0x00;
    I2C_Buf[1]=0xEA;
    ret = CTP_I2C_WRITE(ts->client, ts->client->addr, I2C_Buf, 2);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    addr = 0;
    for(i=0;i<(BUFFER_LENGTH)/128;i++)
    {
        for(j=0;j<16;j++)
        {
            unsigned char tmp=0;
            addrH = addr>>8;
            addrL = addr&0xFF;
            for(k=0;k<8;k++)
            {
                tmp+=BUFFER_DATA[i*128+j*8+k];
            }
            tmp = tmp+addrH+addrL+8;
            tmp = (255-tmp)+1;
            WR_Filechksum+=tmp;
            addr+=8;
        }
    }

    msleep(800);

    do
    {
        msleep(10);
        I2C_Buf[0]=0xFF;
        I2C_Buf[1]=0x3F;
        I2C_Buf[2]=0xF8;
        ret = CTP_I2C_WRITE(ts->client, ts->client->addr, I2C_Buf, 3);
        if (ret != 1)
        {
            NVT_ERROR("i2c write failed. error = %d. ", ret);
        }

        buf2[0]=0x00;
        buf2[1]=0x00;
        buf2[2]=0x00;
        buf2[3]=0x00;
        ret = CTP_I2C_READ(ts->client, ts->client->addr, buf2, 4);
        if (ret != 2)
        {
            NVT_ERROR("i2c read failed. error = %d. ", ret);
        }

        Retry_Counter++;
        msleep(10);

    }while((Retry_Counter<20)&& (buf2[1]!=0xAA));

    //---------------------------------------------------------------------------------------

    if(buf2[1]==0xAA)
    {
        RD_Filechksum=(buf2[2]<<8)+buf2[3];
        if(RD_Filechksum==WR_Filechksum)
        {
            NVT_INFO("Checksum match, RD_Filechksum = %d, WR_Filechksum = %d.", RD_Filechksum, WR_Filechksum);
            return 1;   // checksum match
        }
        else
        {
            NVT_INFO("Checksum not match, RD_Filechksum = %d, WR_Filechksum = %d.", RD_Filechksum, WR_Filechksum);
            return 0;   // checksum not match
        }
    }
    else
    {
        NVT_INFO("Read checksum failed, RD_Filechksum = %d, WR_Filechksum = %d.", RD_Filechksum, WR_Filechksum);
        return -1;  // read checksum failed
    }
}

void Update_Firmware(void)
{
    NVT_INFO("Start to update Firmware.");
    uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int i = 0;
    int j = 0;
    unsigned int Flash_Address = 0;
    unsigned int Row_Address = 0;
    uint8_t CheckSum[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};    // 128/8 = 16 times ;
    struct i2c_client *client = ts->client;
    int ret = 0;

    //-------------------------------
    // Step1 --> initial BootLoader
    //-------------------------------
    I2C_Buf[0] = 0x00;
    I2C_Buf[1] = 0xA5;
    ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    msleep(2);

    // Initiate Flash Block
    I2C_Buf[0] = 0x00;
    I2C_Buf[1] = 0x00;
    ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    msleep(20);

    // Read status
    I2C_Buf[0] = 0x00;
    ret = CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
    if (ret != 2)
    {
        NVT_ERROR("i2c read failed. error = %d. ", ret);
    }
    if (I2C_Buf[1] != 0xAA)
    {
        NVT_ERROR("Program: init get status(0x%2X) error.", I2C_Buf[1]);
        return;
    }
    NVT_INFO("Program: init get status(0x%2X) success.", I2C_Buf[1]);

    //---------------------------------------------------------
    // Step 2 : Erase
    //---------------------------------------------------------
    I2C_Buf[0]=0x00;
    I2C_Buf[1]=0x66;
    I2C_Buf[2]=0x00;
    I2C_Buf[3]=0x0E;
    I2C_Buf[4]=0x01;
    I2C_Buf[5]=0xB4;
    I2C_Buf[6]=0x3D;
    ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 7);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    while(1)
    {
        msleep(1);
        ret = CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
        if (ret != 2)
        {
            NVT_ERROR("i2c read failed. error = %d. ", ret);
        }
        if(I2C_Buf[1]==0xAA)
            break;
    }

    I2C_Buf[0]=0x00;
    I2C_Buf[1]=0x66;
    I2C_Buf[2]=0x00;
    I2C_Buf[3]=0x0F;
    I2C_Buf[4]=0x01;
    I2C_Buf[5]=0xEF;
    I2C_Buf[6]=0x01;
    ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 7);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    while(1)
    {
        msleep(1);
        ret = CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
        if (ret != 2)
        {
            NVT_ERROR("i2c read failed. error = %d. ", ret);
        }
        if(I2C_Buf[1]==0xAA)
            break;
    }


    for (i = 0 ; i < BUFFER_LENGTH/4096 ; i++)  // 32K = 8 times
    {
        Row_Address = i * 4096;

        // Erase Flash
        I2C_Buf [0] = 0x00;
        I2C_Buf [1] = 0x33;
        I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );  // Address High Byte
        I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);  // Address Low Byte
        ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 4);
        if (ret != 1)
        {
            NVT_ERROR("i2c write failed. error = %d. ", ret);
        }

        msleep(15); // Delay 15 ms

        // Read Erase status
        ret = CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
        if (ret != 2)
        {
            NVT_ERROR("i2c read failed. error = %d. ", ret);
        }

        // check status
        if (I2C_Buf[1] != 0xAA)
        {
            NVT_ERROR("Program: erase(0x%2X) error", I2C_Buf[1]);
            return;
        }
    }

    NVT_INFO("Program: erase(0x%2X) success", I2C_Buf[1]);

    Flash_Address = 0;

    ////////////////////////////////////////////////////////////////////////////////////
    //----------------------------------------
    // Step3. Host write 128 bytes to IC
    //----------------------------------------
    NVT_INFO("Program: write begin, please wait...");

    for(j=0;j<BUFFER_LENGTH/128;j++)
    {
    Flash_Address=(j)*128;

        for (i = 0 ; i < 16 ; i++, Flash_Address += 8)  // 128/8 = 16 times for One Row program
        {
        // write bin data to IC
            I2C_Buf[0] = 0x00;
            I2C_Buf[1] = 0x55;  //Flash write command
            I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);    //Flash address [15:8]
            I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);   //Flash address [7:0]
            I2C_Buf[4] = 0x08;  //Flash write length (byte)
            I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];    //Binary data 1
            I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];    //Binary data 2
            I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];    //Binary data 3
            I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];    //Binary data 4
            I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
            I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];   //Binary data 6
            I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];   //Binary data 7
            I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];   //Binary data 8

            // Calculate a check sum by Host controller.
            // Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
            //               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
            //               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
            CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
                  I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
                  I2C_Buf[13]) + 1;

            // Load check sum to I2C Buffer
            I2C_Buf[5] = CheckSum[i];
            ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 14);
            if (ret != 1)
            {
                NVT_ERROR("i2c write failed. error = %d. ", ret);
            }
        }

        msleep(10);

        // Read status
        I2C_Buf[0] = 0x00;
        while(1)
        {
            ret = CTP_I2C_READ(ts->client, I2C_HW_Address, I2C_Buf, 2);
            if (ret != 2)
            {
                NVT_ERROR("i2c read failed. error = %d. ", ret);
            }
            if(I2C_Buf[1]==0xAA)
                break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////
    //----------------------------------------
    // Step4. Verify
    //----------------------------------------
    NVT_INFO("Program: Verify begin, please wait...");
    ret = Check_CheckSum();
    if(ret==1)
        NVT_INFO("Program: Verify Pass!");
    else if(ret==0)
        NVT_INFO("Program: Verify NG!");
    else if(ret==-1)
        NVT_INFO("Program: Verify FW not return!");

    //---write i2c command to reset---
    //I2C_Buf[0] = 0x00;
    //I2C_Buf[1] = 0x5A;
    //ret = CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);
    //if (ret != 1)
    //{
    //  NVT_ERROR("i2c write failed. error = %d. ", ret);
    //}

    //---trigger rst-pin to reset---
    nvt_hw_reset();

    msleep(500);
    NVT_INFO("Program: END");
}
#endif


#if XY_REMAPPING
static int BoundaryFun_En = 0;          //_AT_  0x3E00;
static int XEdgeDist_L = 0;             //_AT_  0x3E01;
static int XCorrectionGradient_L = 0;   //_AT_  0x3E02;
static int XEdgeDist_R = 0;             //_AT_  0x3E03;
static int XCorrectionGradient_R = 0;   //_AT_  0x3E04;
static int YEdgeDist_U = 0;             //_AT_  0x3E05;
static int YCorrectionGradient_U = 0;   //_AT_  0x3E06;
static int YEdgeDist_D = 0;             //_AT_  0x3E07;
static int YCorrectionGradient_D = 0;   //_AT_  0x3E08;

static int OLM_Remapping_En = 0;        //_AT_  0x3E10;
static int X_VA_Res = 0;
//static int X_VA_Res_H = 0;                //_AT_  0x3E11;
//static int X_VA_Res_L = 0;                //_AT_  0x3E12;
static int Y_VA_Res = 0;
//static int Y_VA_Res_H = 0;                //_AT_  0x3E13;
//static int Y_VA_Res_L = 0;                //_AT_  0x3E14;
static signed char X_AdjX_R = 0;        //_AT_  0x3E15;
static signed char X_AdjX_L = 0;        //_AT_  0x3E16;
static signed char Y_AdjY_U = 0;        //_AT_  0x3E17;
static signed char Y_AdjY_D = 0;        //_AT_  0x3E18;

static int X_Res = 0;
//static int X_Res_H = 0;                   //_AT_  0x3F7C;
//static int X_Res_L = 0;                   //_AT_  0x3F7D;
static int Y_Res = 0;
//static int Y_Res_H = 0;                   //_AT_  0x3F7E;
//static int Y_Res_L = 0;                   //_AT_  0x3F7F;

void nvt_xy_mapping_getinfo(void)
{
    uint8_t I2C_Buf[16];
    int ret = 0;

    //---get boundary info.---
    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3E;
    I2C_Buf[2]=0x00;
    ret = CTP_I2C_WRITE(ts->client, ts->client->addr, I2C_Buf, 3);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    I2C_Buf[0] = 0;
    ret = CTP_I2C_READ(ts->client, ts->client->addr, I2C_Buf, 10);
    if (ret != 2)
    {
        NVT_ERROR("i2c read failed. error = %d. ", ret);
    }
    BoundaryFun_En = I2C_Buf[1];
    XEdgeDist_L = I2C_Buf[2];
    XCorrectionGradient_L = I2C_Buf[3];
    XEdgeDist_R = I2C_Buf[4];
    XCorrectionGradient_R = I2C_Buf[5];
    YEdgeDist_U = I2C_Buf[6];
    YCorrectionGradient_U = I2C_Buf[7];
    YEdgeDist_D = I2C_Buf[8];
    YCorrectionGradient_D = I2C_Buf[9];

    NVT_INFO("BoundaryFun_En=%d, XEdgeDist_L=%d, XCorrectionGradient_L=%d, XEdgeDist_R=%d, XCorrectionGradient_R=%d\n",
        BoundaryFun_En, XEdgeDist_L, XCorrectionGradient_L, XEdgeDist_R, XCorrectionGradient_R);
    NVT_INFO("YEdgeDist_U=%d, YCorrectionGradient_U=%d, YEdgeDist_D=%d, YCorrectionGradient_D=%d\n",
        YEdgeDist_U, YCorrectionGradient_U, YEdgeDist_D, YCorrectionGradient_D);


    //---get remapping info.---
    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3E;
    I2C_Buf[2]=0x10;
    ret = CTP_I2C_WRITE(ts->client, ts->client->addr, I2C_Buf, 3);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    I2C_Buf[0] = 0;
    ret = CTP_I2C_READ(ts->client, ts->client->addr, I2C_Buf, 14);
    if (ret != 2)
    {
        NVT_ERROR("i2c read failed. error = %d. ", ret);
    }
    OLM_Remapping_En = I2C_Buf[1];
    X_VA_Res = I2C_Buf[2]*256 + I2C_Buf[3];
    Y_VA_Res = I2C_Buf[4]*256 + I2C_Buf[5];
    X_AdjX_R = I2C_Buf[6];
    X_AdjX_L = I2C_Buf[7];
    Y_AdjY_U = I2C_Buf[8];
    Y_AdjY_D = I2C_Buf[9];

    NVT_INFO("OLM_Remapping_En=%d, X_VA_Res=%d, Y_VA_Res=%d\n",
        OLM_Remapping_En, X_VA_Res, Y_VA_Res);
    NVT_INFO("X_AdjX_R=%+d, X_AdjX_L=%+d, Y_AdjY_U=%+d, Y_AdjY_D=%+d\n",
        X_AdjX_R, X_AdjX_L, Y_AdjY_U, Y_AdjY_D);


    //---get xy res.---
    I2C_Buf[0]=0xFF;
    I2C_Buf[1]=0x3F;
    I2C_Buf[2]=0x00;
    ret = CTP_I2C_WRITE(ts->client, ts->client->addr, I2C_Buf, 3);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    I2C_Buf[0] = 0x7C;
    ret = CTP_I2C_READ(ts->client, ts->client->addr, I2C_Buf, 5);
    if (ret != 2)
    {
        NVT_ERROR("i2c read failed. error = %d. ", ret);
    }
    X_Res = I2C_Buf[1]*256 + I2C_Buf[2];
    Y_Res = I2C_Buf[3]*256 + I2C_Buf[4];

    NVT_INFO("X_Res=%d, Y_Res=%d", X_Res, Y_Res);
}

void nvt_xy_mapping(unsigned int *xp, unsigned int *yp)
{
    s32 x = *xp;
    s32 y = *yp;

    if(BoundaryFun_En == 1)
    {
        //Boundary mapping X
        if(x < XEdgeDist_L)
        {
            x = x - (((XEdgeDist_L - x)*XCorrectionGradient_L)/16);
            if(x < 0)
                x = 0;
        }
        else if(x > (X_Res-1-XEdgeDist_R))
        {
            x = x + (((x-(X_Res-1-XEdgeDist_R))* XCorrectionGradient_R)/16);
            if(x > (X_Res-1))
                x = (X_Res-1);
        }
        //Boundary mapping Y
        if(y < YEdgeDist_U)
        {
            y = y - (((YEdgeDist_U - y)*YCorrectionGradient_U)/16);
            if(y < 0)
                y = 0;
        }
        else if(y > (Y_Res-1-YEdgeDist_D))
        {
            y = y + (((y-(Y_Res-1-YEdgeDist_D))* YCorrectionGradient_D)/16);
            if(y > (Y_Res-1))
                y = (Y_Res-1);
        }
    }

    if(OLM_Remapping_En == 1)
    {
        //Normal AA remaping X
        x = ((x*TOUCH_MAX_WIDTH)/X_Res);
        //Normal AA remaping Y
        y = ((y*TOUCH_MAX_HEIGHT)/Y_Res);
    }
    else if(OLM_Remapping_En == 2)
    {
        //TOD AA remapping X
        if(x > (X_Res/2))
            x = ((TOUCH_MAX_WIDTH/2) + (((x-(X_Res/2))*TOUCH_MAX_WIDTH) / (X_VA_Res+X_AdjX_R)));
        else
            x = ((TOUCH_MAX_WIDTH/2) - ((((X_Res/2)-x)*TOUCH_MAX_WIDTH) / (X_VA_Res+X_AdjX_L)));

        //TOD AA remapping Y
        if(y > (Y_Res/2))
            y = ((TOUCH_MAX_HEIGHT/2) + (((y-(Y_Res/2))*TOUCH_MAX_HEIGHT) / (Y_VA_Res+Y_AdjY_D)));
        else
            y = ((TOUCH_MAX_HEIGHT/2) - ((((Y_Res/2)-y)*TOUCH_MAX_HEIGHT) / (Y_VA_Res+Y_AdjY_U)));
    }

    *xp = (unsigned int)x;
    *yp = (unsigned int)y;
}
#endif


/*******************************************************
Description:
    Novatek touchscreen work function.

Parameter:
    ts: i2c client private struct.

return:
    Executive outcomes.0---succeed.
*******************************************************/
static void nvt_ts_work_func(struct work_struct *work)
{
    struct i2c_client *client = ts->client;

    int ret = -1;
    uint8_t  point_data[(TOUCH_MAX_FINGER_NUM*6)+2+1]={0};
    unsigned int position = 0;
    unsigned int input_x = 0;
    unsigned int input_y = 0;
    unsigned char input_w = 0;
    unsigned char input_id = 0;

    int i;
    int finger_cnt=0;

    ret = CTP_I2C_READ(ts->client, ts->client->addr, point_data,  ts->max_touch_num*6+2+1);
    if(ret < 0)
    {
        NVT_ERROR("i2c read failed. error = %d. ", ret);
        goto XFER_ERROR;
    }

    finger_cnt = 0;

#if MT_PROTOCOL_B
    for(i=0; i<ts->max_touch_num; i++)
    {
        position = 1 + 6*i;
        input_id = (unsigned int)(point_data[position+0]>>3)-1;

        if((point_data[position]&0x07) == 0x03) // finger up (break)
        {
            input_mt_slot(ts->input_dev, input_id);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
        }
        else if(((point_data[position]&0x07) == 0x01) || ((point_data[position]&0x07) == 0x02)) //finger down (enter&moving)
        {
            input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
            input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
            input_w = (unsigned int)(point_data[position+4])+10;
            if(input_w > 255)
                input_w = 255;

            #if XY_REMAPPING
            nvt_xy_mapping(&input_x, &input_y);
            #endif

            if((input_x < 0) || (input_y < 0))
                continue;
            if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
                continue;

            input_mt_slot(ts->input_dev, input_id);
            input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);

            finger_cnt++;
        }
    }
    input_report_key(ts->input_dev, BTN_TOUCH, (finger_cnt>0)); //all finger up
#else
    for(i=0; i<ts->max_touch_num; i++)
    {
        position = 1 + 6*i;
        input_id = (unsigned int)(point_data[position+0]>>3)-1;

        if((point_data[position]&0x07) == 0x03) // finger up (break)
        {
            continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
        }
        else if(((point_data[position]&0x07) == 0x01) || ((point_data[position]&0x07) == 0x02)) //finger down (enter&moving)
        {
            input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
            input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
            input_w = (unsigned int)(point_data[position+4])+10;
            if(input_w > 255)
                input_w = 255;

            #if XY_REMAPPING
            nvt_xy_mapping(&input_x, &input_y);
            #endif

            if((input_x < 0) || (input_y < 0))
                continue;
            if((input_x > ts->abs_x_max)||(input_y > ts->abs_y_max))
                continue;

            input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
            input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
            input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
            input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, input_id);
            input_report_key(ts->input_dev, BTN_TOUCH, 1);

            input_mt_sync(ts->input_dev);

            if (Touch_flag == 0)
            {
#if FACTORY_TEST
                NVT_DEBUG("Touch DOWN.");
#endif
                Touch_flag = 1;
            }
            if (Flag_print_point == 1)
            {
                NVT_DEBUG("Touch x = %d, y = %d.", input_x, input_y);
            }

            finger_cnt++;
        }
    }
    if(finger_cnt == 0)
    {
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
        input_report_key(ts->input_dev, BTN_TOUCH, 0);

        input_mt_sync(ts->input_dev);
        if (Touch_flag == 1)
        {
#if FACTORY_TEST
            NVT_DEBUG("Touch UP!");
#endif
            Touch_flag = 0;
        }
    }
#endif


#if TOUCH_KEY_NUM > 0
    if(point_data[ts->max_touch_num*6+1]==0xF8)
    {
        for(i=0; i<TOUCH_KEY_NUM; i++)
        {
            input_report_key(ts->input_dev, touch_key_array[i], ((point_data[ts->max_touch_num*6+2]>>i)&(0x01)));
        }
    }
    else
    {
        for(i=0; i<TOUCH_KEY_NUM; i++)
        {
            input_report_key(ts->input_dev, touch_key_array[i], 0);
        }
    }
#endif

    input_sync(ts->input_dev);

XFER_ERROR:
    enable_irq(ts->client->irq);
}

/*******************************************************
Description:
    External interrupt service routine.

Parameter:
    irq:    interrupt number.
    dev_id: private data pointer.

return:
    irq execute status.
*******************************************************/
static irqreturn_t nvt_ts_irq_handler(int irq, void *dev_id)
{

#if ReportRate
    if(ShowReportRate()==1)
    {
        NVT_INFO("Report Rate = %d", TouchCount);
        TouchCount=0;
    }
    else
    {
        TouchCount++;
    }
#endif

    disable_irq_nosync(ts->client->irq);
    queue_work(nvt_wq, &ts->work);

    return IRQ_HANDLED;
}

/*******************************************************
Description:
    Novatek touchscreen
 function.

Parameter:
    client: i2c device struct.
    id:device id.

return:
    Executive outcomes. 0---succeed.
*******************************************************/
static int nvt_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
    int retry = 0;
    int ret_CheckFwVersion = 0;
    uint8_t buf[7] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    NVT_INFO("Start to probe...");
    NVT_INFO("NVT I2C Address: 0x%02x", client->addr);
//*******************************
//---Step1. allocate the nvt_ts_data.---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S1 : Start to allocate nvt_ts_data.");
#endif
    ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
    if (ts == NULL)
    {
        NVT_ERROR("Allocate nvt_ts_data failed.");
        ret = -ENOMEM;
        goto err_alloc_nvt_ts_data_failed;
    }
#if NVT_DEBUG_MSG
    NVT_DEBUG("S1 : Allocate nvt_ts_data OK.");
#endif

    ts->client = client;
    i2c_set_clientdata(client, ts);

//*******************************
//---Step2. request INT-pin---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S2 : Start to request INT.");
#endif
    ret = gpio_request_one(NVTTOUCH_INT_GPIO, GPIOF_IN, "NVT-int");
    if (ret != 0)
    {
        NVT_ERROR("Request NVT-int GPIO failed.");
        goto err_request_int_failed;
    }
#if NVT_DEBUG_MSG
    NVT_DEBUG("S2 : Request INT OK.");
#endif

//*******************************
//---Step3. request RST-pin---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S3 : Start to request RST.");
#endif
    ret = gpio_request_one(NVTTOUCH_RST_GPIO, GPIOF_OUT_INIT_HIGH, "NVT-rst");
    if (ret != 0)
    {
        NVT_ERROR("Request NVT-rst GPIO failed.");
        goto err_request_rst_failed;
    }
#if NVT_DEBUG_MSG
    NVT_DEBUG("S3 : Request RST OK.");
#endif

//*******************************
//---Step4. check i2c func.---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S4 : Start to check i2c func.");
#endif
    if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        NVT_ERROR("i2c_check_functionality failed. (no I2C_FUNC_I2C).");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }
#if NVT_DEBUG_MSG
    NVT_DEBUG("S4 : Check i2c func. OK.");
#endif

//*******************************
//---Step5. check i2c read---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S5 : Start to check i2c read.");
#endif
    for(retry=30; retry>=0; retry--)
    {
        NVT_INFO("NVT I2C Address: 0x%02x", client->addr); //Firmware Address Test
        ret = CTP_I2C_READ(client, client->addr, buf, 5);
        NVT_INFO("buf[1]=%d, buf[2]=%d, buf[3]=%d, buf[4]=%d, buf[5]=%d",
            buf[1], buf[2], buf[3], buf[4], buf[5]);

        if(ret != 2)    // i2c read failed
        {
            NVT_ERROR("NVT FW I2C read test failed at retry=%d.", retry);
            NVT_INFO("NVT I2C HW Address: 0x%02x", I2C_HW_Address);    //HW Address Test
            ret = CTP_I2C_READ(client, I2C_HW_Address, buf, 5);
            NVT_INFO("buf[1]=%d, buf[2]=%d, buf[3]=%d, buf[4]=%d, buf[5]=%d",
                buf[1], buf[2], buf[3], buf[4], buf[5]);

            if (ret != 2)    // i2c read failed
            {
                NVT_ERROR("NVT HW I2C read test failed at retry=%d.", retry);
            }
            else    // i2c read succeed
            {
                NVT_INFO("NVT HW I2C read test succeed.");
                break;
            }
        }
        else    // i2c read succeed
        {
            NVT_INFO("NVT FW I2C read test succeed.");
            break;
        }

        if(retry == 0)
        {
            goto err_i2c_failed;
        }
    }
#if NVT_DEBUG_MSG
    NVT_DEBUG("S5 : Check i2c read OK.");
#endif

//*******************************
//---initial nvt work---
//*******************************
    INIT_WORK(&ts->work, nvt_ts_work_func);

//*******************************
//---Step6. allocate input device---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S6 : Start to allocate input device.");
#endif
    ts->input_dev = input_allocate_device();
    if(ts->input_dev == NULL)
    {
        NVT_ERROR("Allocate input device failed.");
        ret = -ENOMEM;
        goto err_input_dev_alloc_failed;
    }
#if NVT_DEBUG_MSG
    NVT_DEBUG("S6 : Allocate input device OK.");
#endif


//*******************************
//---Update firmware---
//*******************************
#if BOOT_UPDATE_FIRMWARE
    ret = Check_CheckSum();

    nvt_hw_reset();
    msleep(500);

    ret_CheckFwVersion = Check_FW_Ver();

    if (ret_CheckFwVersion==0) //IC Firmware Version is too old
    {
        NVT_INFO("Firmware version is not match.");
        Update_Firmware();
    }
    else if (ret==0) //Verison is not too old but Checksum is not match
    {
        NVT_INFO("Checksum is not match.");
        Update_Firmware();
    }
    else if (ret==-1) //Verison is not too old but Checksum is failed
    {
        NVT_INFO("Read Checksum is failed.");
        Update_Firmware();
    }
#endif


    ts->abs_x_max = TOUCH_MAX_WIDTH-1;
    ts->abs_y_max = TOUCH_MAX_HEIGHT-1;
    ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

    #if TOUCH_KEY_NUM > 0
    ts->max_button_num = TOUCH_KEY_NUM;
    #endif

    ts->int_trigger_type = INT_TRIGGER_TYPE;


#if XY_REMAPPING
    nvt_xy_mapping_getinfo();
#endif

//*******************************
//---set input device info.---
//*******************************
    ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
    ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
    ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
    ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

#if MT_PROTOCOL_B
    input_mt_init_slots(ts->input_dev, ts->max_touch_num);
    input_set_abs_params(ts->input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_FINGER, 0, 0);
#endif

    input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);    //pressure = 255

#if TOUCH_MAX_FINGER_NUM > 1
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif

#if TOUCH_KEY_NUM > 0
    for(retry = 0; retry < TOUCH_KEY_NUM; retry++)
    {
        input_set_capability(ts->input_dev, EV_KEY,touch_key_array[retry]);
    }
#endif

    sprintf(ts->phys, "input/ts");
    ts->input_dev->name = NVT_TS_NAME;
    ts->input_dev->phys = ts->phys;
    ts->input_dev->id.bustype = BUS_I2C;
    ts->input_dev->id.vendor = 0x0205;
    ts->input_dev->id.product = 0x0001;

//*******************************
//---register input device---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S7 : Start to register input device.");
#endif
    ret = input_register_device(ts->input_dev);
    if(ret)
    {
        NVT_ERROR("register input device (%s) failed. ret=%d", ts->input_dev->name, ret);
        goto err_input_register_device_failed;
    }
    ts->bad_data = 0;
#if NVT_DEBUG_MSG
    NVT_DEBUG("S7 : Register input device OK.");
#endif

//*******************************
//---set int-pin & request irq---
//*******************************
#if NVT_DEBUG_MSG
    NVT_DEBUG("S8 : Start to set INT-pin & request irq.");
#endif
    int gpio;
    gpio=acpi_get_gpio("\\_SB.GPO2",3);
    NVT_INFO("gpio=%d", gpio);
    client->irq = gpio_to_irq(NVTTOUCH_INT_GPIO);
    if(client->irq)
    {
        NVT_INFO("int_trigger_type=%d",ts->int_trigger_type);
        ret = request_irq(client->irq, nvt_ts_irq_handler, ts->int_trigger_type, client->name, ts);
        if (ret != 0)
        {
            NVT_ERROR("request irq failed. ret=%d", ret);
            goto err_int_request_failed;
        }
        else
        {
            disable_irq(client->irq);
            NVT_INFO("request irq %d succeed.", client->irq);
        }
    }
    enable_irq(client->irq);
#if NVT_DEBUG_MSG
    NVT_DEBUG("S8 : Set INT-pin & request irq OK.");
#endif

//*******************************
//---set device node---
//*******************************
#if NVT_TOUCH_CTRL_DRIVER
    nvt_flash_init();
#endif

#if NVT_TOUCH_SYSFS_NODE
    ret = novatek_touch_sysfs_init();
    if (ret)
    {
        NVT_ERROR("Failed to create debug node in Sysfs, error=%d", ret);
    }
    ts->attrs.attrs = novatek_attr;
#endif


#ifdef CONFIG_HAS_EARLYSUSPEND
    ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    ts->early_suspend.suspend = nvt_ts_early_suspend;
    ts->early_suspend.resume = nvt_ts_late_resume;
    register_early_suspend(&ts->early_suspend);
#endif

    NVT_INFO("Finished...");
    return 0;


err_init_NVT_ts:
    free_irq(client->irq,ts);
err_int_request_failed:
err_input_register_device_failed:
    input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
err_i2c_failed:
err_check_functionality_failed:
    i2c_set_clientdata(client, NULL);
    kfree(ts);
err_alloc_nvt_ts_data_failed:
err_request_int_failed:
err_request_rst_failed:
    return ret;
}

/*******************************************************
Description:
    Novatek touchscreen driver release function.

Parameter:
    client: i2c device struct.

return:
    Executive outcomes. 0---succeed.
*******************************************************/
static int nvt_ts_remove(struct i2c_client *client)
{
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ts->early_suspend);
#endif

    NVT_INFO("Removing driver...");

#if NVT_TOUCH_SYSFS_NODE
    novatek_touch_sysfs_deinit();
#endif

    free_irq(client->irq, ts);
    input_unregister_device(ts->input_dev);
    i2c_set_clientdata(client, NULL);
    kfree(ts);

    return 0;
}

static int nvt_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
    NVT_INFO("Enter TS Suspend...");
    uint8_t buf[4]={0x88, 0x55, 0xAA, 0xA5};
    int ret = 0;

    disable_irq(client->irq);

    //---write i2c command to enter "deep sleep mode"---
    ret = CTP_I2C_WRITE(ts->client, ts->client->addr, buf, 4);
    if (ret != 1)
    {
        NVT_ERROR("i2c write failed. error = %d. ", ret);
    }

    msleep(50);

    if (Touch_flag == 1)
    {
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
        input_report_key(ts->input_dev, BTN_TOUCH, 0);
        input_mt_sync(ts->input_dev);
        input_sync(ts->input_dev);
#if FACTORY_TEST
        NVT_DEBUG("Touch UP!");
#endif
        Touch_flag = 0;
    }

    return 0;
}

static int nvt_ts_resume(struct i2c_client *client)
{
    NVT_INFO("Enter TS Resume...");
    //---trigger rst-pin to reset---
    nvt_hw_reset();
    msleep(500);

    enable_irq(client->irq);

    return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void nvt_ts_early_suspend(struct early_suspend *h)
{
    NVT_INFO("Enter TS Early Suspend...");
    nvt_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void nvt_ts_late_resume(struct early_suspend *h)
{
    NVT_INFO("Enter TS Early Resume...");
    nvt_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id nvt_ts_id[] = {
    { NVT_I2C_NAME, 0 },
    { }
};

//Add for byt_cr
static const struct acpi_device_id nvt_acpi_match[] = {
    { MPU_I2C_NAME, 0 },
};

//Do not need for intel platform
// static struct i2c_board_info __initdata nvt_i2c_boardinfo[] = {
//  {
//      I2C_BOARD_INFO(NVT_I2C_NAME, I2C_FW_Address),
//  },
// };

static struct i2c_driver nvt_i2c_driver = {
    .probe      = nvt_ts_probe,
    .remove     = nvt_ts_remove,
    .suspend    = nvt_ts_suspend,
    .resume     = nvt_ts_resume,
    .id_table   = nvt_ts_id,
    .driver = {
        .name   = NVT_I2C_NAME,
        .owner  = THIS_MODULE,
        .acpi_match_table = ACPI_PTR(nvt_acpi_match), //Add for byt_cr
    },
};


/*******************************************************
Description:
    Driver Install function.
return:
    Executive Outcomes. 0---succeed.
********************************************************/
static int nvt_driver_init(void)
{
    NVT_INFO("Enter TS Driver Init...");

    int ret;

/*
    struct i2c_adapter *adapter;

    //---add i2c adapter---
    adapter = i2c_get_adapter(NVT_I2C_BUS_NUM);
    if(!adapter)
    {
        printk("%s : failed to get i2c adapter %d.\n", __func__, NVT_I2C_BUS_NUM);
        ret=-ENODEV;
        goto err_driver;
    }

    //---add i2c device---
    nvt_client = i2c_new_device(adapter, nvt_i2c_boardinfo);
    if(!nvt_client)
    {
        printk("%s : failed to add i2c device at 0x%x.\n", __func__, (unsigned int)nvt_i2c_boardinfo->addr);
        ret=-ENODEV;
        goto err_driver;
    }
    i2c_put_adapter(adapter);
*/

    // omap_register_i2c_bus(4, 400, nvt_i2c_boardinfo, ARRAY_SIZE(nvt_i2c_boardinfo));

    //---create workqueue---
    nvt_wq = create_workqueue("nvt_wq");
    if(!nvt_wq)
    {
        NVT_ERROR("Create workqueue failed.");
        return -ENOMEM;
    }

    //---add i2c driver---
    ret = i2c_add_driver(&nvt_i2c_driver);
    if(ret)
    {
        NVT_ERROR("Failed to add i2c driver.");
        goto err_driver;
    }

    NVT_INFO("Finished.");

err_driver:
    return ret;
}


/*******************************************************
Description:
    Driver uninstall function.
return:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit nvt_driver_exit(void)
{
    NVT_INFO("Enter TS Driver Exit...");
    i2c_del_driver(&nvt_i2c_driver);
    // i2c_unregister_device(nvt_client);

    if(nvt_wq)
        destroy_workqueue(nvt_wq);
}


late_initcall(nvt_driver_init);
module_exit(nvt_driver_exit);


MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
