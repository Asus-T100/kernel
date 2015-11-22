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

#define REAR_CAM 1
#define FRONT_CAM 2
#define MAX_NAME  20

//ASUS_BSP+++, camera driver
#define GP_I2C_3_SCL "I2C_3_SCL"
#define GP_I2C_3_SDA "I2C_3_SDA"
#define SIO_I2C3_SDA     84
#define SIO_I2C3_SCL     85
#define VV_NGPIO_SCORE   102
#define CAMERA_0_RESET   (VV_NGPIO_SCORE + 24)	//CAM1_RESET_N   GPIONC_24
#define CAMERA_1_RESET   (VV_NGPIO_SCORE + 25)	//CAM2_RESET_N   GPIONC_25
#define CAMERA_0_PWDN    (VV_NGPIO_SCORE + 21)	//CAM1_PWRDWN    GPIONC_21
#define CAMERA_1_PWDN    (VV_NGPIO_SCORE + 22)	//CAM2_PWRDWN    GPIONC_22

#ifdef CONFIG_INTEL_SOC_PMC
#define OSC_CAM0_CLK 0x0
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1
/* workaround - use xtal for cht */
#define CLK_19P2MHz_XTAL 0x0
#define CLK_ON  0x1
#define CLK_OFF 0x2
#endif

extern int hm2056_set_gpio(int RearOrFront, int flag);
extern void hm2056_free_gpio();
extern int gc0339_set_gpio(int flag);
extern void gc0339_free_gpio();
//ASUS_BSP---, camera driver


struct camera_device_table {
	struct sfi_device_table_entry entry;
	struct devs_id dev;
};

extern int decide_cam_id(char *);
extern struct intel_v4l2_subdev_id *get_camera_ids(void);
extern struct camera_device_table *get_camera_table(void);
extern int get_camera_tablesize(void);

#endif
