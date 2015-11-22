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
#include <linux/gpio.h>
#include <linux/mfd/intel_mid_pmic.h>

#include "boardinfo.h"

static func_handle table[FUN_ID_MAX];


int get_gpio(unsigned int pin)
{
	int ret;

	if(gpio_request(pin, "PCB_ID"))
	{
		pr_info("Fail to get pcb id pin %d \n", pin);
		return 0;
	}

	gpio_direction_input(pin);
	//int ret;
	ret = gpio_get_value_cansleep(pin);
	printk("SOC GPIO %d := %x\n", pin, ret);
	gpio_free(pin);

	return ret;
}

int get_pmic_gpio(unsigned int pin)
{
	int iov = intel_mid_pmic_readb(pin);

	printk("PMIC GPIO %x := %x\n", pin, iov);

	return (iov & 0x01);
}

/* HW PCBID: ID(0, 1, 2)
 * (0, 0, 0) SR1
 * (0, 0, 1) ER1
 * (0, 1, 0) ER2
 * (0, 1, 1) PR
 */
int get_hw_rev(void)
{
	int tmp = 0;
	int hardware_id = 0;

	tmp = get_pmic_gpio(0x35);
	hardware_id |= (tmp<<2);
	tmp = get_pmic_gpio(0x38);
	hardware_id |= (tmp<<1);
	tmp = get_pmic_gpio(0x39);
	hardware_id |= tmp;
	switch (hardware_id) {
		case 0:
			return SR1;
		case 1:
			return ER1;
		case 2:
			return ER2;
		case 3:
			return PR;
		default:
			return UNKNOWN;
	}

	return UNKNOWN;
}

/* PEROJECT PCBID: ID(5,6, 7)
 * (0, 1, 1) TF303CL
 */
int get_project_id(void)
{
	int tmp = 0;
	int project_id = 0;

	tmp = get_pmic_gpio(0x45);
	project_id |= (tmp << 2);
	tmp = get_pmic_gpio(0x49);
	project_id |= (tmp << 1);
	tmp = get_pmic_gpio(0x4A);
	project_id |= tmp;
	switch (project_id) {
		case (7):
			return TF303CL;
	}

	return UNKNOWN;
}

/* Panel: ID(13)
 * (0) CPT_CLAA101FP05_XG
 * (1) AUO_B101UAN01_7
 */
int get_lcd_id(void)
{
	int tmp = 0;
	int panel = 0;

	panel = get_gpio(95);
	switch (panel) {
		case (0):
			return CPT_CLAA101FP05_XG;
		case (1):
			return AUO_B101UAN01_7;
	}

	return UNKNOWN;
}

/* Touch: ID(8, 9)
 * (0 , 0) JTOUCH_8610120765AC05N
 */
int get_tp_id(void)
{
	int tmp = 0;
	int tp_id = 0;

	tmp = get_gpio(155);
	tp_id |= ( tmp << 1);
	tmp = get_gpio(156);
	tp_id |= tmp;

	switch (tp_id) {
		case (0):
			return JTOUCH_8610120765AC05N;
	}

	return UNKNOWN;
}

/*
 * 1.2M Camera: ID(15)
 * (0) CHICONY_CIFDH6520003871LH_MI1040
 * (1) LITEON_3SF103T2_MI1040
 */
int get_camera_front_id(void)
{
	int front_cam_id = 0;

	front_cam_id = get_gpio(6);

	return front_cam_id;
}

/* Undefine */
int get_camera_rear_id(void)
{
	return UNKNOWN;
}

/* Undefine */
int get_camera_sku_id(void)
{
	return UNKNOWN;
}

/* Memory: ID(3,4)
 * (0 , 0) HYNIX_H9CCNNNBPTALBR_NTD
 * (0 , 1) SAMSUN_K3QF2F20DM_AGCE
 */
int get_dram_id(void)
{
	int tmp = 0;
	int memory = 0;

	tmp = get_pmic_gpio(0x47);
	memory |= (tmp << 1);
	tmp = get_pmic_gpio(0x48);
	memory |= tmp;

	switch (memory) {
		case (0):
			return HYNIX_H9CCNNNBPTALBR_NTD;
		case (1):
			return SAMSUNG_K3QF2F20DM_AGCE;
	}
}

int boardinfo_init()
{
	memset(&table, 0, sizeof(table));

	ADD_FUNC(table,FUN_HARDWARE_ID,"To get the hardware revision",get_hw_rev);
	ADD_FUNC(table,FUN_PROJECT_ID,"To get the project id",get_project_id);
	ADD_FUNC(table,FUN_LCD_ID,"To get the lcd id",get_lcd_id);
	ADD_FUNC(table,FUN_TOUCH_ID,"To get the touch id",get_tp_id);
	ADD_FUNC(table,FUN_CAMERA_REAR_ID,"To get the rear camera id",get_camera_rear_id);
	ADD_FUNC(table,FUN_CAMERA_FRONT_ID,"To get the front camera id",get_camera_front_id);
	ADD_FUNC(table,FUN_DRAM_ID,"To get the memory id",get_dram_id);
	ADD_FUNC(table,FUN_CAMERA_SKU_ID,"To get camera sku",get_camera_sku_id);

	return set_boardinfo_tab(table);
}

