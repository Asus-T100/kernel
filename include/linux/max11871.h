/* include/linux/max11871.h
 *
 * Copyright (c)2012 Maxim Integrated Products, Inc.
 *
 * Driver Version: 3.0.1
 * Release Date: June 17, 2012
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

#ifndef __MAX11871_H
#define __MAX11871_H

#define MAX11871_NAME   "max11871"
#define MAX11871_TOUCH  MAX11871_NAME "_touchscreen_0"
#define MAX11871_KEY    MAX11871_NAME "_key_0"

#define MAX11871_BOARD_CONFIG  1

#define MAX_WORDS_COMMAND  9  /* command address space 0x00-0x09 minus header
				=> 9 command words maximum */
#define MAX_WORDS_REPORT   245  /* address space 0x00-0xFF minus 0x00-0x09 for
				commands minus header, maximum 1 report packet
				*/
#define MAX_WORDS_COMMAND_ALL  (15 * MAX_WORDS_COMMAND)  /* maximum 15 packets
				9 payload words each */

#define MAX11871_MAX_BUTTONS      10
#define MAX11871_FNAME_LENGTH     255
#define MAX11871_FWTABLE_LENGTH   10
#define MAX11871_CFGTABLE_LENGTH  20

struct max11871_fw_image {
	char  *file_name;
	char  fname_buf[MAX11871_FNAME_LENGTH + 1];
	u16   length;
	u16   crc16;
	u16   config_boundary;
};

struct max11871_fw_mapping {
	u16  chip_id;
	u16  fw_index;
};

struct max11871_chip_config {
	u16                         config_id;
	#define MAX11871_CONFIG_TOUCH        0x0001
	#define MAX11871_CONFIG_PRIVATE      0x0002
	#define MAX11871_CONFIG_CALIBRATION  0x0004
	#define MAX11871_CONFIG_LINEARITY_X  0x0008
	#define MAX11871_CONFIG_LINEARITY_Y  0x0010
	#define MAX11871_CONFIG_IFACTOR      0x0020
	u16                         config_tables;
	u16                         config_touch[MAX_WORDS_COMMAND_ALL];
	u16                         config_private[MAX_WORDS_COMMAND_ALL];
	u16                         config_cal[MAX_WORDS_COMMAND_ALL];
	u16                         config_lin_x[MAX_WORDS_COMMAND_ALL];
	u16                         config_lin_y[MAX_WORDS_COMMAND_ALL];
	u16                         config_ifactor[MAX_WORDS_COMMAND_ALL * 5];
	u16                         fw_mappings;
	struct max11871_fw_mapping  fw_mapping[MAX11871_FWTABLE_LENGTH];
};

struct max11871_button_xy {
	u16  x;
	u16  y;
	u16  size_x;
	u16  size_y;
};

struct max11871_coordinates {
	u16                        panel_mx_l;
	u16                        panel_mx_h;
	u16                        panel_my_l;
	u16                        panel_my_h;
	u16                        lcd_x;
	u16                        lcd_y;
	struct max11871_button_xy  button_xy[MAX11871_MAX_BUTTONS];
};

struct max11871_config {
	u16                          cfg_request;
	char                         *cfg_file_name;
	char                         cfg_fname_buf[MAX11871_FNAME_LENGTH + 1];
	u16                          chip_configs;
	struct max11871_chip_config  chip_config[MAX11871_CFGTABLE_LENGTH];
	struct max11871_fw_image     fw_image[MAX11871_FWTABLE_LENGTH];
	u16                          defaults_allow;
	u16                          default_chip_config;
	u16                          default_chip_id;
	u16                          cfgfw_request_delay;
	u16                          i2c_words;
	u16                          max_touches;
	u16                          events_per_sec;
	#define MAX11871_REVERSE_X         0x0001
	#define MAX11871_REVERSE_Y         0x0002
	#define MAX11871_SWAP_XY           0x0004
	u16                          coordinate_settings;
	#define MAX11871_OPTIMAL           0
	#define MAX11871_HIGH_PRECISION    1
	u16                          coordinate_model;
	struct max11871_coordinates  coordinates[2];
	u16                          buttons_enabled;
	#define MAX11871_BUTTONS_XY        0
	#define MAX11871_BUTTONS_SENSE     1
	u16                          buttons_type;
	u16                          buttons;
	unsigned int                 button_code[MAX11871_MAX_BUTTONS];
	#define MAX11871_PROTOCOL_A        0
	#define MAX11871_PROTOCOL_A_TRACK  1
	#define MAX11871_PROTOCOL_B        2
	#define MAX11871_PROTOCOL_CUSTOM1  3
	u16                          input_protocol;
};

struct max11871_pdata {
	struct max11871_config  *config;
	unsigned                gpio_reset;
	unsigned                gpio_tirq;
	int  (*init)(struct max11871_pdata *pdata, int value);
	int  (*reset)(struct max11871_pdata *pdata, int value);
	int  (*tirq)(struct max11871_pdata *pdata);
};

#endif /* __MAX11871_H */

