/* drivers/input/touchscreen/NVTtouch_205.h
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
#ifndef     _LINUX_NVT_TOUCH_H
#define     _LINUX_NVT_TOUCH_H

#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>


//---GPIO number---
#define NVTTOUCH_RST_GPIO 60
#define NVTTOUCH_INT_GPIO 158


//---INT trigger mode---
//#define IRQ_TYPE_EDGE_RISING 1
#define IRQ_TYPE_EDGE_FALLING 2
//#define IRQ_TYPE_LEVEL_HIGH 4
//#define IRQ_TYPE_LEVEL_LOW 8
#define INT_TRIGGER_TYPE IRQ_TYPE_EDGE_FALLING


//---I2C driver info.---
#define NVT_I2C_NAME "NVT-nt11205-ts"
#define MPU_I2C_NAME "NTK11205" //for me80c porting
// #define NVT_I2C_BUS_NUM 4 //do not need to be defined in byt
// #define I2C_FW_Address 0x03 //do not need to be defined in byt
#define I2C_HW_Address 0x70


//---Input device info.---
#define NVT_TS_NAME "novatek-touchscreen"


//---Touch info.---
#define TOUCH_MAX_WIDTH 800
#define TOUCH_MAX_HEIGHT 1280
#define TOUCH_MAX_FINGER_NUM 10
#define TOUCH_KEY_NUM 0
#if TOUCH_KEY_NUM > 0
const uint16_t touch_key_array[TOUCH_KEY_NUM]={
    KEY_BACK,
    KEY_HOME,
    KEY_MENU
};
#endif


#define BUFFER_LENGTH 32768


//---Customerized func.---
#define NVT_TOUCH_CTRL_DRIVER 1
#define BOOT_UPDATE_FIRMWARE 1
#define NVT_DEBUG_MSG 1
#define NVT_TOUCH_SYSFS_NODE 1
#define FACTORY_TEST 1
#define MT_PROTOCOL_B 0
#define ReportRate 0
#define XY_REMAPPING 0

//---Log define.---
#define NVT_INFO(fmt,arg...)    printk(KERN_INFO "[NVT-INFO] [%d] %s : "fmt"\n",__LINE__,__func__,##arg)
#define NVT_ERROR(fmt,arg...)   printk(KERN_ERR "[NVT-ERROR] [%d] %s : "fmt"\n",__LINE__,__func__,##arg)
#if NVT_DEBUG_MSG
#define NVT_DEBUG(fmt,arg...)   printk(KERN_DEBUG "[NVT-DEBUG] [%d] %s : "fmt"\n",__LINE__,__func__,##arg)
#endif

struct nvt_ts_data{
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct work_struct work;
    uint16_t addr;
    uint8_t bad_data;
    char phys[32];
    int retry;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend early_suspend;
#endif
    uint16_t abs_x_max;
    uint16_t abs_y_max;
    uint8_t x_num;
    uint8_t y_num;
    uint8_t max_touch_num;
    uint8_t max_button_num;
    uint32_t int_trigger_type;
    uint8_t green_wake_mode;
#if NVT_TOUCH_SYSFS_NODE
    struct attribute_group attrs;
#endif
};

#if NVT_TOUCH_CTRL_DRIVER
struct nvt_flash_data{
    rwlock_t lock;
    unsigned char bufferIndex;
    unsigned int length;
    struct i2c_client *client;
};
#endif

#endif /* _LINUX_NVT_TOUCH_H */
