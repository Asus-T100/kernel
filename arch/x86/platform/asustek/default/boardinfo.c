/*
 * Copyright (C) 2014 ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General #include "fixed_pcbid.h"Public License for more details.
 */

#include <linux/printk.h>
#include <linux/string.h>
#include "boardinfo.h"

static func_handle table[FUN_ID_MAX];

int get_hw_rev(void)
{
	int ret = -1;
	printk("Get the Hardword ID \n");
	return ret;
}


int get_project_id(void)
{
	int ret = 10;
	printk("Get the project ID \n");
	return ret;
}


int get_lcd_id(void)
{
	int ret = -1;
	printk("Get the lcd ID \n");
	return ret;
}


int get_tp_id(void)
{
	int ret = -1;
	printk("Get the tp ID \n");
	return ret;
}


int get_camera_sku_id(void)
{
	int ret = -1;
	printk("Get the camera ID \n");
	return ret;
}

int get_camera_front_id(void)
{
	int ret = -1;
	printk("Get the Hardword ID \n");
	return ret;
}


int get_camera_rear_id(void)
{
	int ret = -1;
	printk("Get the Hardword ID \n");
	return ret;
}

int get_rf_sku_id(void)
{
	int ret = -1;
	printk("Get the Hardword ID \n");
	return ret;
}

int get_sim_id(void)
{
	int ret = -1;
	printk("Get the Hardword ID \n");
	return ret;
}

int get_dram_id(void)
{
	int ret = -1;
	printk("Get the Hardword ID \n");
	return ret;
}


int boardinfo_init()
{
	memset(&table, 0, sizeof(table));

	ADD_FUNC(table,FUN_HARDWARE_ID,"To get the hardware revision",get_hw_rev);
	ADD_FUNC(table,FUN_PROJECT_ID,"To get the project id",get_project_id);
	ADD_FUNC(table,FUN_LCD_ID,"To get the lcd id",get_lcd_id);
	ADD_FUNC(table,FUN_TOUCH_ID,"To get the touch id",get_tp_id);
	ADD_FUNC(table,FUN_CAMERA_SKU_ID,"To get the camera sku id",get_camera_sku_id);
	ADD_FUNC(table,FUN_CAMERA_REAR_ID,"To get the rear camera id",get_camera_rear_id);

	return set_boardinfo_tab(table);
}

