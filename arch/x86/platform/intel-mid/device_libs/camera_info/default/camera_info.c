/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include "camera_info.h"

const static int rear_cam_pwdn = CAMERA_1_PWDN;
const static int rear_cam_reset = CAMERA_1_RESET;
const static int rear_cam_vcm_pd = CAMERA_1_VCM_PD;
const static int front_cam_pwdn = CAMERA_2_PWDN;
const static int front_cam_reset = CAMERA_2_RESET;

struct intel_v4l2_subdev_id * dynamic_select()
{
	/*Get the pcbid to select the camera port*/
	return NULL;
}

struct camera_device_table *get_camera_table(void)
{
	pr_info("%s is called",__func__);
	return NULL;
}

int get_camera_tablesize(void)
{
	pr_info("%s is called",__func__);
	return 0;
}

int decide_cam_id(char *sensor_name)
{
        if (sensor_name == NULL)
                return -1;
        printk("%s: searching name:%s\n",sensor_name);

        return -1;
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