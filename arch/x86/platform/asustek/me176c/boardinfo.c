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
#include <linux/gpio.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/debugfs.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include <linux/module.h>
#include <linux/proc_fs.h>

#define  GPIO_FRONT_CAM_ID 66
#define  GPIO_REAR_CAM_ID 67
#define  GPIO_HARDWARE_ID0 0x35
#define  GPIO_HARDWARE_ID1 0x38
#define  GPIO_HARDWARE_ID2 0x39
#define  GPIO_HARDWARE_ID3 0x47
#define  GPIO_HARDWARE_ID4 0x48
#define  GPIO_HARDWARE_ID5 0x45
#define  GPIO_HARDWARE_ID6 0x49
#define  GPIO_HARDWARE_ID7 0x4a

#ifdef ASUS_FACTORY
#define CAM_DEBUG(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define CAM_DEBUG(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

#ifdef ASUS_FACTORY
#define HARDWARE_ID_DEBUG(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define HARDWARE_ID_DEBUG(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

static func_handle table[FUN_ID_MAX];
static int gFrontCamId = UNKNOWN;
static int gRearCamId = UNKNOWN;
static int gHardwareId = UNKNOWN;
static int project_id = -1;

//Get the gpio value,
//Return -1 if error occurred
int get_gpio_value(unsigned int gpio)
{
	int ret;
	if(gpio_request(gpio, "CAMERA_SKU"))
	{
		pr_warn("Fail to request gpio  %d \n", gpio);
		return -1;
	}

	gpio_direction_input(gpio);
	//int ret;
	ret = gpio_get_value_cansleep(gpio);
	CAM_DEBUG("CAMERA GPIO %d := %x\n", gpio, ret);
	gpio_free(gpio);
	return ret;
}

int get_hw_rev(void)
{
	CAM_DEBUG("Get the Hardword ID \n");
	return UNKNOWN;
}

int get_camera_front_id(void)
{
	gFrontCamId = get_gpio_value(GPIO_FRONT_CAM_ID);
	CAM_DEBUG("Get the FrontCam ID:%d \n", gFrontCamId);
	return gFrontCamId;
}

int get_camera_rear_id(void)
{
	gRearCamId = get_gpio_value(GPIO_REAR_CAM_ID);
	CAM_DEBUG("Get the RearCam ID:%d \n", gRearCamId);
	return gRearCamId;
}

int get_pmic_gpio(unsigned int pin)
{
	int iov = 0;

	iov = intel_mid_pmic_readb(pin);

	printk("PMIC GPIO %x := %x  %d\n", pin, iov, iov);

	return (iov & 0x01);
}

/* Get Hardware ID
 * Return : 1 -> PR
 *          0 -> ER
*/
int get_hardware_id(void)
{
	int hardware_id2, hardware_id4;
	hardware_id2 = intel_mid_pmic_readb(GPIO_HARDWARE_ID2);
	hardware_id4 = intel_mid_pmic_readb(GPIO_HARDWARE_ID4);

	gHardwareId = ((hardware_id2 & 0x01) || (hardware_id4 & 0x01));
	printk("Hardware ID: %d\n", gHardwareId);
	return gHardwareId;
}

int get_project_id(void) //OK
{
	int ret = -1;
	
	if(gHardwareId == UNKNOWN) gHardwareId = get_hardware_id();

	if(gHardwareId){ //PR
		project_id = get_pmic_gpio(GPIO_HARDWARE_ID5);
		switch (project_id) {
			case 0:
				return ME176C;
			case 1:
				return ME176CX;
			default:
				return UNKNOWN;
		}
	}
	else{ //ER
	 project_id = get_pmic_gpio(GPIO_HARDWARE_ID1);
		switch (project_id) {
			case 0:
				return ME176C;
			case 1:
				return ME176CX;
			default:
				return UNKNOWN;
		}
	}
	printk("Get the project ID \n");

	return ret;
}
/*
 *Get camera sku id.
 *Return 0 : 2M + 0.3M
 *Return 1 : 5M + 2M
 *Return -1: Error occurred
 * */
/*
int get_project_id(void)
{
	if (gFrontCamId == UNKNOWN) gFrontCamId = get_camera_front_id();
	if (gRearCamId == UNKNOWN) gRearCamId = get_camera_rear_id();

	if ((gFrontCamId == 1) && (gRearCamId == 1)) {
		CAM_DEBUG("ME176C_L with Camera Rear:5M, Front:2M.");
		return ME176C_L;
	} else if ((gFrontCamId == 0) && (gRearCamId == 0)) {
		CAM_DEBUG("ME176CX_L with Camera Rear:2M, Front:0.3M.");
		return ME176CX_L;
	}

	CAM_DEBUG("Unknown Project: Camera Rear ID:%d, Front ID:%d\n", gRearCamId, gFrontCamId);
	return UNKNOWN;
}
*/
//Carlisle ++
#define ME176_PROJECTID_PROC_FILE  "driver/me176_project_id"
static struct proc_dir_entry *panel_type_proc_file;

static ssize_t panel_type_proc_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	u8 panel_type = 1;
	u8 project_stage = 0;
	ssize_t ret = 0;
	char *buff;
	int desc = 0;
	char cx[] = "1";
	char c[] = "0";

	buff = kmalloc(10,GFP_KERNEL);

	if(!buff)
	{
		return -ENOMEM;
	}

	ret = get_project_id();
/*
	//project_stage = intel_mid_pmic_readb(0x39);//GPIO0P6 /0=ER /1=PR
	//GPIO0P6 & GPIO1P5 //if only one is 1 ->PR  else ->ER
	project_stage = intel_mid_pmic_readb(0x39) || intel_mid_pmic_readb(0x48);
*/
	if(gHardwareId) //PR
	{
		if(project_id) //cx
		{
			desc += sprintf(buff + desc,"%s\n",cx);
			printk("%s:----Carlisle test----panel_type_proc_read----%d\n", __func__,__LINE__);
		}
		else //c
		{
			desc += sprintf(buff + desc,"%s\n",c);
			printk("%s:----Carlisle test----panel_type_proc_read----%d\n", __func__,__LINE__);			
		}
	}
	else //ER
	{
		if(project_id) //cx
		{
			desc += sprintf(buff + desc,"%s\n",cx);
			printk("%s:----Carlisle test----panel_type_proc_read----%d\n", __func__,__LINE__);
		}
		else //c
		{
			desc += sprintf(buff + desc,"%s\n",c);
			printk("%s:----Carlisle test----panel_type_proc_read----%d\n", __func__,__LINE__);			
		}
	}	
	
	ret = simple_read_from_buffer(buffer,count,ppos,buff,desc);

	kfree(buff);
    return ret;
}

static struct file_operations panel_type_proc_ops = {
    .read = panel_type_proc_read,
};

static void create_panel_type_proc_file(void)
{
    printk("%s:----sean test----create_me176c_panel_type_proc_file----%d\n", __func__,__LINE__);
    panel_type_proc_file = proc_create(ME176_PROJECTID_PROC_FILE, 0666,NULL, &panel_type_proc_ops);
    if(!panel_type_proc_file)
	{
		printk("create driver/me176c_panel_type fail\n");
	}
}
//Carlisle --

static char *hardware_phase="NA";
module_param(hardware_phase, charp, S_IRUGO | S_IWUSR);

static void read_hardware_phase(void)
{
    if(gHardwareId) //PR
    {
        strcpy(hardware_phase,"PR");
        printk("%s: %d gHardwareId = %d\n", __func__,__LINE__,gHardwareId);
    }
    else //ER
    {
        strcpy(hardware_phase,"ER");
        printk("%s: %d gHardwareId = %d\n", __func__,__LINE__,gHardwareId);
    }
}

int boardinfo_init()
{
	memset(&table, 0, sizeof(table));

	ADD_FUNC(table, FUN_HARDWARE_ID, "To get the hardware revision", get_hw_rev);
	ADD_FUNC(table, FUN_PROJECT_ID, "To get the project id", get_project_id);
	ADD_FUNC(table, FUN_CAMERA_FRONT_ID, "To get the front camera id", get_camera_front_id);
	ADD_FUNC(table, FUN_CAMERA_REAR_ID, "To get the rear camera id", get_camera_rear_id);
	ADD_FUNC(table, RAW_HARDWARE_ID, "To get the hardware id", get_hardware_id);
	
	create_panel_type_proc_file();//sean_lu ++++ for ME176C panel_type_proc_create
	read_hardware_phase();
	
	return set_boardinfo_tab(table);
}

