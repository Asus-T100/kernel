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

#define  GPIO_FRONT_CAM_ID 66
#define  GPIO_REAR_CAM_ID 67
#define  GPIO_HARDWARE_ID0 0x35
#define  GPIO_HARDWARE_ID1 0x38
#define  GPIO_PROJECT_ID 0x45

#ifdef ASUS_FACTORY
#define CAM_DEBUG(fmt, ...) \
    printk(KERN_DEBUG pr_fmt("%s:%i: "fmt), __func__, __LINE__, ##__VA_ARGS__)
#else
#define CAM_DEBUG(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt("%s:%i: "fmt), __func__, __LINE__, ##__VA_ARGS__)
#endif

#ifdef ASUS_FACTORY
#define HARDWARE_ID_DEBUG(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define HARDWARE_ID_DEBUG(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

#ifdef ASUS_FACTORY
#define PROJECT_ID_DEBUG(fmt, ...) \
    printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define PROJECT_ID_DEBUG(fmt, ...) \
    no_printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#endif

static func_handle table[FUN_ID_MAX];
static int gProjectId = UNKNOWN;
static int gCamSkuId = UNKNOWN;
static int gFrontCamId = UNKNOWN;
static int gRearCamId = UNKNOWN;
static int gHardwareId = UNKNOWN;

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
	if (gFrontCamId != UNKNOWN) return gFrontCamId;

	switch (get_gpio_value(GPIO_FRONT_CAM_ID)) {
		case 0:
			gFrontCamId = CHICONY_CIFE02920003870LH_GC0310; // 0.3M
			break;
		case 1:
			gFrontCamId = LITEON_4SF223T2A_GC2155M; // 2M
			break;
	}

	CAM_DEBUG("Get the FrontCam ID:%d \n", gFrontCamId);
	return gFrontCamId;
}

int get_camera_rear_id(void)
{
	if (gRearCamId != UNKNOWN) return gRearCamId;

	switch (get_gpio_value(GPIO_REAR_CAM_ID)) {
		case 0:
			gRearCamId = LITEON_4SF227T2A_GC2155M;
			break;
		case 1:
			gRearCamId = CHICONY_CJAE51120003870LH_AR0543;
			break;
	}

	CAM_DEBUG("Get the RearCam ID:%d \n", gRearCamId);
	return gRearCamId;
}

int get_project_id(void)
{
	if (gProjectId == UNKNOWN) gProjectId = ME176CE;
	return gProjectId;
}

/*
 *Get camera sku id.
 *Return 0 : 2M + 0.3M
 *Return 1 : 5M + 2M
 *Return -1: Error occurred
 * */
static unsigned int camera_sku_id_table[] = {
	CAMERA_SKU_ID(LITEON_4SF227T2A_GC2155M, CHICONY_CIFE02920003870LH_GC0310),
};

int get_camera_sku_id(void)
{
	unsigned int cam_sku_id;
	int i;

	if (gCamSkuId == UNKNOWN) {
		cam_sku_id = CAMERA_SKU_ID(get_camera_rear_id(), get_camera_front_id());
		CAM_DEBUG("Get cam_sku_id=0x%x\n", cam_sku_id);
		for (i=0; i < (sizeof(camera_sku_id_table)/sizeof(unsigned int)); i++) {
			CAM_DEBUG("%d) Check Camera SKU ID=0x%x\n", i, camera_sku_id_table[i]);
			if (camera_sku_id_table[i] == cam_sku_id) {
				gCamSkuId = camera_sku_id_table[i];
				CAM_DEBUG("Set Camera SKU ID=0x%x\n", gCamSkuId);
				break;
			}
		}
	}

	if (gCamSkuId == UNKNOWN) {
		CAM_DEBUG("Unknown Camera SKU ID=0x%x\n", cam_sku_id);
	}
	return gCamSkuId;
}

/* Get Hardware ID
 * Return : 3 -> PR
 *          2 -> ER2
 *          1 -> ER1
 *          0 -> SR
*/

int get_hardware_id(void)
{
	int hardware_id0, hardware_id1;
	hardware_id0 = intel_mid_pmic_readb(GPIO_HARDWARE_ID0);
	hardware_id1 = intel_mid_pmic_readb(GPIO_HARDWARE_ID1);

	gHardwareId = (((hardware_id0 & 0x01) << 1) | (hardware_id1 & 0x01));
	HARDWARE_ID_DEBUG("Hardware ID: %d\n", gHardwareId);
	return gHardwareId;
}

/* Get Project ID
 * Return : 0 -> ME176CE
*/

int get_raw_project_id(void)
{
	int project_id;

	project_id = intel_mid_pmic_readb(GPIO_PROJECT_ID);

	PROJECT_ID_DEBUG("Project ID: %d\n", project_id);
	return project_id;
}

int boardinfo_init()
{
	memset(&table, 0, sizeof(table));

	ADD_FUNC(table, FUN_HARDWARE_ID, "To get the hardware revision", get_hw_rev);
	ADD_FUNC(table, FUN_PROJECT_ID, "To get the project id", get_project_id);
	ADD_FUNC(table, FUN_CAMERA_SKU_ID, "To get the camera SKU id", get_camera_sku_id);
	ADD_FUNC(table, FUN_CAMERA_FRONT_ID, "To get the front camera id", get_camera_front_id);
	ADD_FUNC(table, FUN_CAMERA_REAR_ID, "To get the rear camera id", get_camera_rear_id);
	ADD_FUNC(table, RAW_HARDWARE_ID, "To get the hardware id", get_hardware_id);
	ADD_FUNC(table, RAW_PROJECT_ID, "To get the hardware id", get_raw_project_id);

	return set_boardinfo_tab(table);
}

