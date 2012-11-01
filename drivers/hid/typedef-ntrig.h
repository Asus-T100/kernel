/****************************************************************
Zoro Software.
D-Trig Digitizer modules files
Copyright (C) 2010, Dmitry Kuzminov

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/
#ifndef __TYPEDEF_NTRIG_H__
#define __TYPEDEF_NTRIG_H__

/*
 * File version: 1
 * N-trig Common definitions
 */

#ifdef __KERNEL__
/* compiling a kernel module */
#include "linux/types.h"
#include "linux/string.h"
#else
/* compiling a linux user space app/library */
#include <stdint.h>
#include <string.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Constant Declaration
 */
#define SUCCESS                 0
#define MAX_NUMBER_OF_FINGER	10
#define MAX_NUMBER_OF_SENSORS	2
#define HID_MSG_LEN             16
#define FIRMWARE_VER_LEN		9
#define DRIVER_VER_LEN			4
#define MESSAGE_ROUTER_LEN		4

/**
 * BUS TYPE
 */
enum BUS_TYPE {
	TYPE_BUS_USB_HID = 0,
	TYPE_BUS_USB_RAW,
	TYPE_BUS_SPI_HID,
	TYPE_BUS_SPI_RAW,
	TYPE_BUS_I2C_HID,
	TYPE_BUS_I2C_RAW
};

/**
 * GET_BUS_INTERFACE
 */
enum GET_BUS_INTERFACE_REPORT {
	REPORTID_GET_BUS_INTERFACE = 0x01
};

#define FIRST_DEV_TYPE	TYPE_BUS_USB_HID
#define LAST_DEV_TYPE	TYPE_BUS_I2C_RAW

/* note: do not use 0bXXX for binary constants -
 * not supported in gcc 4.2.1 which is used by x86 ndk */
#define BUS_INTERFACE_USB		    0x01	/* 0b001 */
#define BUS_INTERFACE_SPI		    0x02	/* 0b010 */
#define BUS_INTERFACE_USB_SPI		0x03	/* 0b011 */
#define BUS_INTERFACE_I2C		    0x04	/* 0b100 */
#define BUS_INTERFACE_I2C_USB		0x05	/* 0b101 */
#define BUS_INTERFACE_I2C_SPI		0x06	/* 0b110 */
#define BUS_INTERFACE_I2C_USB_SPI	0x07	/* 0b111 */

#define FIRST_BUS_INTERFACE		0x01	/* 0b001 */
#define LAST_BUS_INTERFACE		0x07	/* 0b111 */

/**
 * Set/Get pen buttons configuration
 */
#define TLV_BUTTON_CONFIG			0x1F


/**
 * Pen Status
 */

/**
 * Pen Pressure status
 */
#define PEN_TIP_MODE_NOT_SUPPORTED		0
#define PEN_TIP_MODE_NOT_256			1
#define PEN_TIP_MODE_NOT_512			2

/**
 * Pen Button status
 */

#define PEN_BUTTON_NOT_AVAILABLE	0xFF
#define PEN_BUTTON_DISABLE			0
#define PEN_BUTTON_RCICK			4
#define PEN_BUTTON_ERASER			8

/**
 * Pen Battery status
 */
#define PEN_BUTTON_BATTERY_OK			    1
#define PEN_BUTTON_BATTERY_LOW			    0
#define PEN_BUTTON_BATTERY_NOT_AVAILABLE	2

/**
 * TLV buffer size
 */

#define TLV_BUFFER_SIZE				        13
#define GET_PEN_STATUSBITS_BUFFER_SIZE		6

/* 512 + Opcod + Sensor_id + Msg Length */
/*#define MAX_NCP_BUFFER			515 */

/* 512 + Opcod + Sensor_id + Msg Length */
#define MAX_NCP_BUFFER				526

/**
 * Message Type
 * MSG_PEN_PARSE    - Pen parsed event     see device_pen_t
 * MSG_FINGER_PARSE - Finger parsed Events see device_finger_t
 * MSG_ERROR - Malfunction report
 */
enum MSG_TYPE {
	MSG_PEN_EVENTS = 2,
	MSG_FINGER_PARSE,
	MSG_ERROR_EVENTS,
	MSG_BUTTON_EVENTS
};

/**
 * CONFIGURATION_TYPE
 * Configure Sensor
 */
enum CONFIGURATION_TYPE_HEX {
	REPORTID_DRIVER_ALIVE = 0x0A,
	REPORTID_CALIBRATION,			        /* 0x0B */
	REPORTID_GET_VERSION,			        /* 0x0C */
	REPORTID_GET_MODE,			            /* 0x0D */
	REPORTID_SET_MODE_PEN,			        /* 0x0E */
	REPORTID_SET_MODE_TOUCH,		        /* 0x0F */
	REPORTID_SET_MODE_DUAL,			        /* 0x10 */
	REPORTID_CALIBRATION_RESPOND,	        /* 0x11 */
	HID_CAPACITORS_CALIB,			        /* 0x12 */
	HID_GET_CAPACITORS_CALIB_DONE,	        /* 0x13 */
	REPORTID_SET_FEATURE_SET_MODE_DISABLE,	/* 0x14 */
	REPORTID_GET_PEN_BITS,			        /* 0x15 */
	REPORTID_GET_CALIB_REQUARED,		    /* 0x16 */
	/*dont use 0x17 */
	REPORTID_FEATURE_TLV = 0x18,
	/*0x19 - unused */
	REPORTID_GET_NOISE_STATUS = 0x1A,
	REPORTID_SET_FEATURE_SET_MODE_AUTO,	    /* 0x1B */
	REPORTID_SET_PARALLAX_ON,		        /* 0x1C */
	REPORTID_SET_PARALLAX_OFF,		        /* 0x1D */
	REPORTID_GET_PARALLAX,			        /* 0x1E */
	REPORTID_TLV_BUTTON_CONFIG,		        /* 0x1F */
	REPORTID_GET_PEN_BATTRY_STATUS = 0xF0	/* Dummy ID - status is
					   read from SW, not device */
};

/**
 * ERROR_CODE
 */
enum ERROR_CODE {
	ERR_NO_SENSOR_FOUND = 1,
	ERR_OPERATION_FAILED,
};

/**
 * Button modes
 */
enum button_mode_t {
	BTN_UP = 0,
	BTN_DOWN
};


/**
 * Capacitive button
 */
struct capacitive_button_t {
	uint16_t		btn_code; /* scan code */
	enum button_mode_t	btn_mode;
};

struct button_parse_t {
	uint8_t sensor_id;
	uint8_t num_of_buttons;
	struct capacitive_button_t buttons_array[MAX_NUMBER_OF_FINGER];
};

/**
 * button_parse_s  - would be used for Soft Keys and Button Pressed reports
 */
/*typedef struct button_parse_t {
	__u8	sensor_id;
	__s32	btn_set_code;
	__s32	btn_rmv_code;
};*/


/**
 * device_pen_s
 */
struct device_pen_t {
	uint8_t  sensor_id;
	uint16_t x_coord;
	uint16_t y_coord;
	uint16_t  pressure;
	int btn_code;			/* 5 status bits (not only buttons) */
	int btn_removed;		/* unused */
	int battery_status;
};

#define FINGER(finger)	(fingers_p->finger_array[finger])
#define BUTTON(button)	(buttons_p->buttons_array[button])

/**
 * device_finger_s - based on CMTMTrackedFinger
 * track_id - finger_id
 */
struct device_finger_t {
	uint8_t		track_id;
	uint16_t	x_coord;
	uint16_t	y_coord;
	uint16_t	dx;
	uint16_t	dy;
	uint8_t		removed;
	uint8_t		palm;
	uint8_t		generic;
};

/**
 * finger_parse_s
 */
struct finger_parse_t {
	uint8_t		sensor_id;
	uint8_t		num_of_fingers;
	uint16_t	frame_index;
	struct device_finger_t finger_array[MAX_NUMBER_OF_FINGER];
};

/**
 * mr_message_types_s
 */
struct mr_message_types_t {
	enum MSG_TYPE type;
	union {
		struct device_pen_t	pen_event;
		struct finger_parse_t	fingers_event;
		struct button_parse_t	buttons_event;
	} msg;
};

/*
 * N-trig Component Version
 */
struct comp_ver_t {
	char firmware[FIRMWARE_VER_LEN];
	char driver[DRIVER_VER_LEN];
	char msg_router[MESSAGE_ROUTER_LEN];
};

#define NCP_RETURN_CODE		4
#define NCP_PAYLOAD_SIZE	512
/**
 * N-trig NCP Message
 */
struct ntrig_usbhid_ncp_t {
	uint16_t host_address;
	uint16_t msg_length;
	uint8_t  msg_type;
	uint16_t msg_group_code;
	uint8_t  return_code[NCP_RETURN_CODE];
	uint16_t reserved;
	uint8_t  payload[NCP_PAYLOAD_SIZE];
};

/**
 * N-trig NCP Message User Dispatcher
 */
enum NCP_COMMAND_USER {
	NCP_WRITE_RAW_COMMAND = 0,
	NCP_WRITE_HID_COMMAND,
	NCP_GOTO_BOOTLOADER,
	NCP_READ_RAW_COMMAND = 171,
	NCP_READ_HID_COMMAND
};

#define NCP_HEADER_LEN	4

#define SPI_ENABLED_COMMAND 0xfe

/**
 * Dispatcher Configuration
 * config_bit_mask -	Bit 0 - Pen
 *			Bit 1 -TrackLib
 *			Bit 2 - bus_interface
 *			Bit 3 -Virtual Keys
 */
struct dispatcher_config_t {
	uint8_t config_bit_mask;
	uint8_t pen_enable;
	uint8_t tracklib_enable;
	uint8_t bus_interface;
	char    *virtual_buttons;	/* max size is PAGE_SIZE (==4096) */
	uint8_t num_of_sensor;
};

/**
 * Dispatcher Configuration Flag
 */
enum DISPATCHER_CONFIGURATION {
	CONFIG_PEN			= 0x01,
	CONFIG_TRACKLIB			= 0x02,
	CONFIG_BUS_INTERFACE		= 0x04,
	CONFIG_VIRTUAL_KEYS		= 0x08,
	CONFIG_NUM_SENSORS		= 0x10,
	CONFIG_TOUCH_SCREEN_BORDER	= 0x20,
	CONFIG_DEBUG_PRINT		= 0x40,
	CONFIG_DRIVER_VERSION		= 0x80,
} ;

union len_u {
	uint16_t msg_len;
	uint8_t  buf[2];
};

enum calib_result {
	CALIB_SUCCESS = 0,
	CALIB_FAIL,
	CALIB_IN_PROGRESS
};

/* calibration status from firmware */
#define CALIB_OK_BYT_1		0x21
#define CALIB_OK_BYT_2		0x21
#define CALIB_FAIL_BYT_1	0x42
#define CALIB_FAIL_BYT_2	0x42
#define CALIB_PRGRS_BYT_1	0x63
#define CALIB_PRGRS_BYT_2	0x63

#ifndef USB_VENDOR_ID_NTRIG

/* ripped from hid-ids.h */
#define USB_VENDOR_ID_NTRIG			        0x1b96
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN	0x0001
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_1	0x0003
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_2	0x0004
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_3	0x0005
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_4	0x0006
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_5	0x0007
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_6	0x0008
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_7	0x0009
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_8	0x000A
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_9	0x000B
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_10	0x000C
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_11	0x000D
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_12	0x000E
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_13	0x000F
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_14	0x0010
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_15	0x0011
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_16	0x0012
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_17	0x0013
#define USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_18	0x0014

#endif

#ifdef __cplusplus
}
#endif

#endif /**__TYPEDEF_NTRIG_H__ */
