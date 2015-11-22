/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include "camera_info.h"
#include "../../platform_ar0543.h"
#include "../../platform_gc2155.h"
#include "../../platform_gc0310.h"
#include "../../platform_hm2056.h" //jimmy add for project ME176C ME176CX
#include "../../platform_gc0339.h" //jimmy add for project ME176C ME176CX
#include "asustek_boardinfo.h"

#define  GPIO_HARDWARE_ID5 0x45
#include <linux/gpio.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/debugfs.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>

const static int rear_cam_pwdn = CAMERA_1_PWDN;
const static int rear_cam_reset = CAMERA_1_RESET;
const static int rear_cam_vcm_pd = CAMERA_1_VCM_PD;
const static int front_cam_pwdn = CAMERA_2_PWDN;
const static int front_cam_reset = CAMERA_2_RESET;



#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
//5M-2M
struct intel_v4l2_subdev_id me176cx_vl42_ids_1[] = {
	//{"gc2155", RAW_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
	{"ar0543", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"hm2056f", RAW_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
};
//2M-0.3M
struct intel_v4l2_subdev_id me176cx_vl42_ids_0[] = {
	{"hm2056b", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"gc0339", RAW_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
        {"gc2155", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"gc0310", RAW_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
};

int get_pmic_gpio_2(unsigned int pin)
{
	int iov = 0;

	iov = intel_mid_pmic_readb(pin);

	printk("PMIC GPIO %x := %x  %d\n", pin, iov, iov);

	return (iov & 0x01);
}
struct intel_v4l2_subdev_id *current_tab = me176cx_vl42_ids_1;
static int curr_index = 1;
static struct camera_device_table me176cx_cam_table[] = {
	{
		{SFI_DEV_TYPE_I2C, 4, 0x36, 0x0, 0x0, "ar0543"},
		{"ar0543", SFI_DEV_TYPE_I2C, 0, &ar0543_platform_data,
			NULL}
	},  {
		{SFI_DEV_TYPE_I2C, 4, 0x3c, 0x0, 0x0, "gc2155"},
		{"gc2155", SFI_DEV_TYPE_I2C, 0, &gc2155_platform_data,
			NULL}
	}, {
		{SFI_DEV_TYPE_I2C, 4, 0x21, 0x0, 0x0, "gc0310"},
		{"gc0310", SFI_DEV_TYPE_I2C, 0, &gc0310_platform_data,
			NULL}
	}, {
		{SFI_DEV_TYPE_I2C, 4, 0x24, 0x0, 0x0, "hm2056f"},
		{"hm2056f", SFI_DEV_TYPE_I2C, 0, &hm2056f_platform_data,
			NULL}
	}, {
		{SFI_DEV_TYPE_I2C, 4, 0x24, 0x0, 0x0, "hm2056b"},
		{"hm2056b", SFI_DEV_TYPE_I2C, 0, &hm2056b_platform_data,
			NULL}
	}, {
		{SFI_DEV_TYPE_I2C, 4, 0x21, 0x0, 0x0, "gc0339"},
		{"gc0339", SFI_DEV_TYPE_I2C, 0, &gc0339_platform_data,
			NULL}
	}

};

struct intel_v4l2_subdev_id *get_table_entry(char *sensor_name)
{
	int entry_num = 0;
	struct intel_v4l2_subdev_id *table;
	if (curr_index == 1) {
		entry_num = ARRAY_SIZE(me176cx_vl42_ids_1);
		table = me176cx_vl42_ids_1;
	} else {
		entry_num = ARRAY_SIZE(me176cx_vl42_ids_0);
		table = me176cx_vl42_ids_0;
	}
	printk("%s: table size:%d\n",__func__,entry_num);
	pr_info("%s: is called ! \n", __func__);
	int i;
	for (i = 0; i < entry_num; i++, table++) {
		if (!strncmp(sensor_name, table->name, 16)) {
			pr_info("%s: camera id is found ! \n", __func__);

		return table;
		}
	}
	return NULL;
}

int decide_cam_id(char *sensor_name)
{
	if (sensor_name == NULL)
		return -1;
	printk("%s(): searching name:%s\n", __func__,  sensor_name);
	struct intel_v4l2_subdev_id *entry = get_table_entry(sensor_name);

	if (entry == NULL) {
		printk("%s :Cant not find sensor in Table\n",__func__);
		return -1;
	}
	if (entry->port == ATOMISP_CAMERA_PORT_PRIMARY)
		return REAR_CAM;
	else if (entry->port == ATOMISP_CAMERA_PORT_SECONDARY)
		return FRONT_CAM;

	return -1;
}


struct intel_v4l2_subdev_id * dynamic_select()
{
	/*
	*Get camera sku id.
	*0 : 2M + 0.3M
	*1 : 5M + 2M
	*-1: Error occurred
	**/

#if 1
	int project_id = get_pmic_gpio_2(GPIO_HARDWARE_ID5);
	//int PROJECT_ID = asustek_boardinfo_get(FUN_PROJECT_ID);
	printk("%s, project_id = %d \n",__func__ , project_id);
	if (project_id == 1) {
		printk("%s:Use 2M + 0.3M Camera Table\n",__func__);
		current_tab = me176cx_vl42_ids_0;
		curr_index = 0;
		return me176cx_vl42_ids_0;
	}
#endif
	printk("%s:Use 5M + 2M Camera Table\n",__func__);
	current_tab = me176cx_vl42_ids_1;
	curr_index = 1;
	return &me176cx_vl42_ids_1;
}

struct camera_device_table *get_camera_table(void)
{
	pr_info("%s is called",__func__);
	return &me176cx_cam_table;
}

int get_camera_tablesize(void)
{
	int size = ARRAY_SIZE(me176cx_cam_table);
	pr_info("%s is called",__func__);
	return size;
}

struct intel_v4l2_subdev_id *get_camera_ids(void)
{
	pr_info("%s is called",__func__);
	return dynamic_select();
}

int get_clock(char *sensor_name)
{
	if (sensor_name == NULL)
		return -1;

	int id = decide_cam_id(sensor_name);
	printk("%s :Find camera id:%d \n",__func__,id);
	if (id == REAR_CAM)
		return OSC_CAM0_CLK;
	else if (id == FRONT_CAM)
		return OSC_CAM1_CLK;

	return -1;
}

int get_pwdn(char *sensor_name)
{
	if (sensor_name == NULL)
		return -1;

	int id = decide_cam_id(sensor_name);

	printk("%s :Find camera id:%d \n",__func__,id);
	if (id == REAR_CAM)
		return rear_cam_pwdn;
	else if (id == FRONT_CAM)
		return front_cam_pwdn;

	return -1;
}

int get_reset(char *sensor_name)
{
	if (sensor_name == NULL)
		return -1;

	int id = decide_cam_id(sensor_name);

	printk("%s :Find camera id:%d \n",__func__,id);
	if (id == REAR_CAM)
		return rear_cam_reset;
	else if (id == FRONT_CAM)
		return front_cam_reset;

	return -1;
}

