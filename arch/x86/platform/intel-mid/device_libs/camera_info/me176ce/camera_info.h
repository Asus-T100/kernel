/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>

#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif


#ifndef _CAMERA_IFNO_
#define _CAMERA_IFNO_
#define CAMERA_1_PWDN 123
#define CAMERA_1_VCM_PD 126
#define CAMERA_1_RESET 126

#define CAMERA_2_RESET 127
#define CAMERA_2_PWDN  124

#define CAMERA_2P8_EN 153

#ifdef CONFIG_INTEL_SOC_PMC
#define OSC_CAM0_CLK 0x0
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1

#define CLK_ON  0x1
#define CLK_OFF 0x2
#endif


#define REAR_CAM 1
#define FRONT_CAM 2
#define MAX_NAME  20

//const static int rear_cam_pwdn = CAMERA_1_PWDN;
//const static int rear_cam_reset = CAMERA_1_RESET;
//const static int rear_cam_vcm_pd = CAMERA_1_VCM_PD;

//const static int front_cam_pwdn = CAMERA_2_PWDN;
//const static int front_cam_reset = CAMERA_2_RESET;

struct camera_device_table {
	struct sfi_device_table_entry entry;
	struct devs_id dev;
};

extern int decide_cam_id(char *);
extern struct intel_v4l2_subdev_id *get_camera_ids(void);
extern struct camera_device_table *get_camera_table(void);
extern int get_camera_tablesize(void);
extern int get_clock(char *sensor_name);
extern int get_pwdn(char *sensor_name);
extern int get_reset(char *sensor_name);

#endif
