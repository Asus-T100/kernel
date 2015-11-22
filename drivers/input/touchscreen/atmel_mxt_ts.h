/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */


#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H


#include <linux/types.h>


/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	unsigned long irqflags;
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	int t15_num_keys;
	const unsigned int *t15_keymap;
	unsigned long gpio_reset;
	const char *cfg_name;
};

#define TOUCH_DEBUG	1

/*
 * Debug Utility
 */
#if TOUCH_DEBUG
#define TOUCH_INFO(format, arg...)    \
        printk(KERN_INFO "touch: [%s] " format , __FUNCTION__ , ## arg)
#define TOUCH_I2C_DATA(array, i)      \
                                        do {            \
                                                for (i = 0; i < array[0]+1; i++) \
                                                        TOUCH_INFO("data[%d] = 0x%x\n", i, array[i]);  \
                                        } while(0)
#else
#define TOUCH_INFO(format, arg...)
#define TOUCH_I2C_DATA(array, i)
#endif

#define TOUCH_ERR(format, arg...)     \
        printk(KERN_ERR "touch: [%s] " format , __FUNCTION__ , ## arg)

#define TOUCH_WARN(format, arg...)     \
        printk(KERN_WARNING "touch: [%s] " format , __FUNCTION__ , ## arg)

#define TOUCH_NOTICE(format, arg...)  \
        printk(KERN_NOTICE "touch: [%s] " format , __FUNCTION__ , ## arg)

#define TOUCH_DEBUG(format, arg...)     \
        printk(KERN_DEBUG "touch: [%s] " format , __FUNCTION__ , ## arg)

#endif /* __LINUX_ATMEL_MXT_TS_H */
