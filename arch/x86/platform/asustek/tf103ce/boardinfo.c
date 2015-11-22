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
#include <linux/debugfs.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>

#include "boardinfo.h"

static func_handle table[FUN_ID_MAX];

static int hardware_id = -1;
static int project_id = -1;
static int lcd_id = -1;
static int tp_id = -1;
static int camera_sku_id = -1;
static int camera_front_id = -1;
static int camera_rear_id = -1;
static int rf_sku_id = -1;
static int sim_id = -1;
static int dram_id = -1;
static int emmc_id = -1;
static int sku_id = -1;
int rc;


int get_gpio(unsigned int pin)
{
	int ret;

	if(gpio_request(pin, "PCB_ID"))
	{
		pr_info("Fail to get pcb id pin %d \n", pin);
		return 0;
	}

	gpio_direction_input(pin);
	ret = gpio_get_value_cansleep(pin);
	printk("SOC GPIO %d := %x\n", pin, ret);
	gpio_free(pin);

	return ret;
}

int get_pmic_gpio(unsigned int pin)
{
	int iov = 0;
	
	iov = intel_mid_pmic_readb(pin);
	
	printk("PMIC GPIO %x := %x  %d\n", pin, iov, iov);

	return (iov & 0x01);
}

int get_hw_rev(void) //OK
{
	int ret = -1;
	int tmp = 0;
	
	tmp = get_pmic_gpio(0x35);
	//rc = intel_scu_ipc_ioread8(0x35, &tmp);
	hardware_id = (tmp<<1);
	tmp = get_pmic_gpio(0x38);
	//rc = intel_scu_ipc_ioread8(0x38, &tmp);
	hardware_id |= tmp;
	
	switch (hardware_id) {
		case 0:
			return SR1;
		case 1:
			return ER1;
		case 2:
			return PR;
		default:
			return UNKNOWN;
	}
	printk("Get the Hardware ID \n");
	return ret;
}

int get_project_id(void) //OK
{
	int ret = -1;
	
	project_id = get_pmic_gpio(0x37);  //todo
	//rc = intel_scu_ipc_ioread8(0x37, &project_id);
	
	switch (project_id) {
		case 0:
			return TF103CE;
		case 1:
			return TF103C;
		default:
			return UNKNOWN;
	}
	printk("Get the project ID \n");
	return ret;
}

int get_lcd_id(void) //OK
{
	int ret = -1;
	
	lcd_id = get_gpio(68);
	
	switch (lcd_id) {
		case 0:
			return AUO_B101EAN01_6;
		default:
			return UNKNOWN;
	}
	printk("Get the lcd ID \n");
	return ret;
}

int get_tp_id(void) //OK
{
	int ret = -1;
	
	tp_id = get_gpio(59);
	
	switch (tp_id) {
		case 0:
			return JTOUCH_8210130763AC05N;
		case 1:
			return OFILM_KTF_101_1521_02AO;
		default:
			return UNKNOWN;
	}
	
	printk("Get the tp ID \n");
	return ret;
}

int get_camera_sku_id(void) //NO
{
	int ret = -1;
	
	switch (camera_sku_id) {
		default:
			return UNKNOWN;
	}
	
	printk("Get the camera ID \n");
	return ret;
}

int get_camera_front_id(void) //??
{
	int ret = -1;
	
	camera_front_id = get_gpio(66);
	
	switch (camera_front_id) {
		case 0:
			return 0;
		case 1:
			return 1;
		default:
			return UNKNOWN;
	}
	
	printk("Get the camera front ID \n");
	return ret;
}

int get_camera_rear_id(void) //OK
{
	int ret = -1;
	
	camera_rear_id = get_gpio(67);
	
	switch (camera_rear_id) {
		case 0:
			return AZWAVE_AM_2F035;
		case 1:
			return ABILITY_SS2BF220;
		default:
			return UNKNOWN;
	}
	printk("Get the camera rear ID \n");
	return ret;
}

int get_rf_sku_id(void) //NO
{
	int ret = -1;
	
	switch (rf_sku_id) {
		default:
			return UNKNOWN;
	}
	printk("Get the RF ID \n");
	return ret;
}

int get_sim_id(void) //NO
{
	int ret = -1;
	
	switch (sim_id) {
		default:
			return UNKNOWN;
	}
	printk("Get the SIM ID \n");
	return ret;
}

int get_dram_id(void) //OK
{
	int ret = -1;
	int tmp = 0;
	
	tmp = get_pmic_gpio(0x47);
	//rc = intel_scu_ipc_ioread8(0x3F, &tmp);
	dram_id = (tmp<<2);
	tmp = get_pmic_gpio(0x48);
	//rc = intel_scu_ipc_ioread8(0x40, &tmp);
	dram_id |= (tmp<<1);
	tmp = get_pmic_gpio(0x4A);
	//rc = intel_scu_ipc_ioread8(0x42, &tmp);
	dram_id |= tmp;
	
	switch (dram_id) {
		case 0:
			return SAMSUNG_K3QF2F20DM_AGCE;
		case 1:
			return HYNIX_H9CCNNNBKTMLBR_NTD;
		case 2:
			return SAMSUNG_H9CCNNN8KTALBR_NTD;
		case 3:
			return HYNIX_H9CCNNN8KTALBR_NTD;
		default:
			return UNKNOWN;
	}
	printk("Get the DRAM ID \n");
	return ret;
}

int get_emmc_id(void) //OK
{
	int ret = -1;
	
	emmc_id = get_pmic_gpio(0x49);
	//rc = intel_scu_ipc_ioread8(0x41, &emmc_id);
	
	switch (emmc_id) {
		case 0:
			return HYNIX_H26M52103FMR;
		case 1:
			return KSI_MMC16G_S100_A08;
		default:
			return UNKNOWN;
	}
	printk("Get the eMMC ID \n");
	return ret;
}

int get_sku_id(void) //OK
{
	int ret = -1;
	
	sku_id = get_gpio(119);
	
	switch (sku_id) {
		case 0:
			return CHANNEL;
		case 1:
			return EDU;
		default:
			return UNKNOWN;
	}
	printk("Get the SKU ID \n");
	return ret;
}

// Create some debug nodes at /d/hardware/
void PCBID_add_dvfs()
{
	struct dentry *rul;
	struct dentry *debugfs_root;
	
	debugfs_root = debugfs_create_dir("hardware", NULL);
	
	rul = debugfs_create_u32("hardware_id", 0644, debugfs_root, &hardware_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("project_id", 0644, debugfs_root, &project_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("lcd_id", 0644, debugfs_root, &lcd_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("tp_id", 0644, debugfs_root, &tp_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("camera_front_id", 0644, debugfs_root, &camera_front_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("camera_rear_id", 0644, debugfs_root, &camera_rear_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("dram_id", 0644, debugfs_root, &dram_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("emmc_id", 0644, debugfs_root, &emmc_id);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("sku_id", 0644, debugfs_root, &sku_id);
	if (!rul)
		goto Fail;
	
	return;
	
Fail:
	debugfs_remove_recursive(debugfs_root);
	debugfs_root = NULL;
}

int boardinfo_init()
{
	int i;
	
	memset(&table, 0, sizeof(table));
	
	ADD_FUNC(table,FUN_HARDWARE_ID,"To get the hardware revision",get_hw_rev);
	ADD_FUNC(table,FUN_PROJECT_ID,"To get the project id",get_project_id);
	ADD_FUNC(table,FUN_LCD_ID,"To get the lcd id",get_lcd_id);
	ADD_FUNC(table,FUN_TOUCH_ID,"To get the touch id",get_tp_id);
	ADD_FUNC(table,FUN_CAMERA_FRONT_ID,"To get the front camera id",get_camera_front_id);
	ADD_FUNC(table,FUN_CAMERA_REAR_ID,"To get the rear camera id",get_camera_rear_id);
	ADD_FUNC(table,FUN_DRAM_ID,"To get the dram id",get_dram_id);
	ADD_FUNC(table,FUN_EMMC_ID,"To get the emmc id",get_emmc_id);
	ADD_FUNC(table,FUN_SKU_ID,"To get the sku id",get_sku_id);
	
	// Export these functions to let other driver to use
	for( i = 0 ; i < FUN_ID_MAX ; i++ ){
		if ( table[i].func != NULL ) {
			table[i].func();
		}
	}
	
	PCBID_add_dvfs();
	
	return set_boardinfo_tab(table);
}

