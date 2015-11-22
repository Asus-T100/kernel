/*
 * Copyright (c) 2014, ASUSTek, Inc. All Rights Reserved.
 * Written by Tom Shen Tom_Shen@asus.com
 */

#ifndef __BATTERY_I2C_STRESS_TEST_H__
#define __BATTERY_I2C_STRESS_TEST_H__

#include <linux/poll.h>
#include <linux/miscdevice.h>

#define BATTERY_IOC_MAGIC 0xF9
#define BATTERY_IOC_MAXNR 5
#define BATTERY_POLL_DATA _IOR(BATTERY_IOC_MAGIC, BATTERY_IOC_MAXNR,int )

#define BATTERY_IOCTL_START_HEAVY 2
#define BATTERY_IOCTL_START_NORMAL 1
#define BATTERY_IOCTL_END 0

#define START_NORMAL    10 * (HZ)
#define START_HEAVY     (HZ)

int register_battery_stress_test(void);

#endif //#define __BATTERY_I2C_STRESS_TEST_H__
