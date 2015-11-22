/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include "camera_info.h"
#include "../../platform_hm2056.h"
#include "../../platform_gc0339.h"
#include "asustek_boardinfo.h"


#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
//2M-0.3M
struct intel_v4l2_subdev_id tf103c_vl42_ids_1[] = {
        {"hm2056b", RAW_CAMERA, ATOMISP_CAMERA_PORT_PRIMARY},
	{"gc0339", RAW_CAMERA, ATOMISP_CAMERA_PORT_SECONDARY},
};

struct intel_v4l2_subdev_id *current_tab = tf103c_vl42_ids_1;
static struct camera_device_table tf103c_cam_table[] = {
        {
                {SFI_DEV_TYPE_I2C, 4, 0x24, 0x0, 0x0, "hm2056b"},
                {"hm2056b", SFI_DEV_TYPE_I2C, 0, &hm2056b_platform_data,
                        NULL}
        },{
                {SFI_DEV_TYPE_I2C, 4, 0x21, 0x0, 0x0, "gc0339"},
                {"gc0339", SFI_DEV_TYPE_I2C, 0, &gc0339_platform_data,
                        NULL}
        }

};

struct intel_v4l2_subdev_id *get_table_entry(char *sensor_name)
{
	int entry_num = 0, i = 0;
	struct intel_v4l2_subdev_id *table;

	entry_num = ARRAY_SIZE(tf103c_vl42_ids_1);
	table = tf103c_vl42_ids_1;

	for (i = 0; i < entry_num; i++, table++) {
		if (!strncmp(sensor_name, table->name, 16)) {
			printk("%s: camera id is found ! \n", __func__);
		        return table;
		}
	}
	return NULL;
}

int decide_cam_id(char *sensor_name)
{
	if (sensor_name == NULL)
		return -1;
	printk("%s: searching name:%s\n",sensor_name);

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
	current_tab = tf103c_vl42_ids_1;
	return &tf103c_vl42_ids_1;
}

struct camera_device_table *get_camera_table(void)
{
	printk("%s is called",__func__);
	return &tf103c_cam_table;
}

int get_camera_tablesize(void)
{
	int size = ARRAY_SIZE(tf103c_cam_table);
	printk("%s is called",__func__);
	return size;
}

struct intel_v4l2_subdev_id *get_camera_ids(void)
{
	return dynamic_select();
}


