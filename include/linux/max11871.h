/* include/linux/max11871.h
 *
 * Copyright (C) 2011 Maxim Integrated Products, Inc.
 *
 * Driver Version: 1.01
 * Release Date: Dec 5, 2011
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MAX11871_H
#define _MAX11871_H

#define MAX11871_NAME  "max11871"

#define MAX11871_CMD_SET_TOUCH_RPT_MODE    0x0018
#define MAX11871_CMD_SET_POWER_MODE        0x0020
#define MAX11871_CMD_GET_FW_VERSION        0x0040

#define MAX11871_RPT_CFG_INF               0x0102
#define MAX11871_RPT_PRIV_CFG              0x0104
#define MAX11871_RPT_CAL_DATA              0x0111
#define MAX11871_RPT_TOUCH_RPT_MODE        0x0119
#define MAX11871_RPT_POWER_MODE            0x0121
#define MAX11871_RPT_SENSITIVITY           0x0125
#define MAX11871_RPT_FRAMERATE             0x0127
#define MAX11871_RPT_RESET_BASELINE        0x0134
#define MAX11871_RPT_FW_VERSION            0x0140
#define MAX11871_RPT_SYS_STATUS            0x01A0
#define MAX11871_RPT_INIT                  0x0400
#define MAX11871_RPT_TOUCH_RAW_IMAGE       0x0800
#define MAX11871_RPT_TOUCH_INFO_BASIC      0x0801
#define MAX11871_RPT_TOUCH_INFO_EXTENDED   0x0802

#define MAX11871_INVALID_COMMAND           0xBADC  /* Invalid cmd identifier */

struct max11871_report_header {
	u16 head;
	u16 id;
	u16 size;
};

struct max11871_report_finger_data {
	u16  status_fingerID;
#define MAX11871_FINGER_STATUS_MASK      0x0F00
#define MAX11871_FINGER_ID_MASK          0x000F

	u16 pos_X;
	u16 pos_Y;
#define MAX11871_FINGER_POSITION_MASK    0x1FFF

	u16 pos_Z;
#define MAX11871_Z_MASK    0x00FF
};

struct max11871_report_finger_data_extended {
	struct max11871_report_finger_data basic;
	u16 pos_X_speed;
	u16 pos_Y_speed;
	u16 pos_X_size;
	u16 pos_Y_size;
	u16 min_X;
	u16 min_Y;
	u16 max_X;
	u16 max_Y;
};

/*
	Touch Report Basic  ID=0x0801
	and extended Extended  ID=0x0802
*/
struct max11871_touch_report {
	struct max11871_report_header header;
	/*note: size == (#touches * 4) + 3*/
	u16 tStatus_tCount;
	u16 gpi_button;
	u16 frame_count;

	union {
		struct max11871_report_finger_data basic_data[10];
		struct max11871_report_finger_data_extended ext_data[10];
	};
};

struct max11871_simple_report {
	/*note: size is 1 for simple reports*/
	struct max11871_report_header header;
	u16 data;
};

struct max11871_finger_data {
	int x;
	int y;
	int z;
	int finger_id;
};

struct max11871_platform_data {
	u16 version;
	u8 i2c_addr;
	u16 abs_x_min;
	u16 abs_x_max;
	u16 abs_y_min;
	u16 abs_y_max;
	u16 abs_z_min;
	u16 abs_z_max;

	u16 abs_button_area_x_max;
	u16 abs_button_area_x_min;
	u16 abs_button_area_y_max;
	u16 abs_button_area_y_min;
	u16 abs_home_button_x;
	u16 abs_home_button_y;
	u16 abs_menu_button_x;
	u16 abs_menu_button_y;
	u16 abs_back_button_x;
	u16 abs_back_button_y;
	u16 abs_search_button_x;
	u16 abs_search_button_y;
	u16 abs_button_fuzz_x;
	u16 abs_button_fuzz_y;

	int gpio_irq;
	int gpio_rst;
	int irq_flags;
	int (*power)(int on);
	int (*platform_hw_init)(void);
	int (*get_report_ready)(void);
};

#endif /*_MAX11871_H*/
