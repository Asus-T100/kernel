/*
 *  HID driver for N-Trig touchscreens
 *
 *  Copyright (c) 2011 N-TRIG
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#ifndef _DTRIG_COMMON_H
#define _DTRIG_COMMON_H

/* Driver version */
#define DRIVER_MAJOR_VERSION 1
#define DRIVER_MINOR_VERSION 3

/* Return codes */
#define DTRG_NO_ERROR		 0
#define DTRG_NOT_COMPLETED	 1
#define DTRG_FAILED		-1

/* packet exchange with digitizer always 128 bytes	*/
#define SPI_PACKET_SIZE         128
/* 1byte type, 2bytes length, 1 byte flags */
#define SPI_HEADER_SIZE         4
/* maximal input packet is 128 bytes */
#define INPUT_MAX_PACKET_SIZE   128

/** Code for application commands that provide SPI-specific data
 * (channel & function). Command structure:
 * byte 0 - this code (0xfe)
 * byte 1 - SPI channel
 * byte 2 - SPI function
 * bytes 3..end - actual data to be sent to the sensor
 */
#define SPI_ENABLED_COMMAND               0xfe
#define SPI_ENABLED_COMMAND_HEADER_LENGTH 3

#define BTN_UNPRESSED		0x00

/* Pen Event Field Declaration */
#define	EVENT_PEN_TIP		0x03
#define	EVENT_PEN_RIGHT		0x05
#define	EVENT_TOUCH_PEN		0x07
#define EVENT_PEN_IN_RANGE	0x01

/* Pen bits declaration */
#define EVENT_PEN_BIT_IN_RANGE		1
#define EVENT_PEN_BIT_TIP		2
#define EVENT_PEN_BIT_RIGHT_CLICK	4
#define EVENT_PEN_BIT_INVERT		8
#define EVENT_PEN_BIT_ERASER		16

/*
 * MTM 4 last bytes of report descriptor
 */
#define REPORT_GENERIC1		0x01
#define REPORT_MT		0x02
#define REPORT_PALM		0x03
#define REPORT_GENERIC2		0x04

#define NTRIG_USB_DEVICE_ID	0x0001

/*
 * MTM fields
 */
#define END_OF_REPORT		0x64

/*
 * Dummy Finger Declaration
 */
#define DUMMY_X_CORD_VAL	0x00
#define DUMMY_Y_CORD_VAL	0x00
#define DUMMY_DX_CORD_VAL	0xFA
#define DUMMY_DY_CORD_VAL	0x96
#define DUMMY_GENERIC_BYTE_VAL	0x0D

/*
 * MTM Parse Event
 */
#define MTM_FRAME_INDEX		0xff000001
#define MTM_PROPRIETARY		0xff000002

/* temporary value */
#define PEN_BATTERY_STATUS	0xffffffff

/*
 * MTM  Set Feature Commands
 */
#define REPORTID_DRIVER_ALIVE   0x0A
#define REPORTID_CALIBRATION	0x0B
#define REPORTID_GET_VERSION	0x0C
#define REPORTID_GET_MODE       0x0D
#define REPORTID_SET_MODE_PEN	0x0E
#define REPORTID_SET_MODE_TOUCH	0x0F
#define REPORTID_SET_MODE_DUAL	0x10
#define REPORTID_CALIBRATION_RESPOND	0x11
#define HID_CAPACITORS_CALIB	0x12
#define HID_GET_CAPACITORS_CALIB_DONE	0x13

/*
 * Set/Get Feature Decleration + NCP
 */
#define HID_CLASS_TOUCH_EP_LEN		16
#define HID_CLASS_NCP_EP_LEN		512

#define HID_REPLY_SIZE			0
#define HID_REPLY_DATA			1

#define NCP_EP_ADDRESS			0x83
#define HID_NCP_USAGE			0xff0b0007
#define NCP_RET_CODE_LEN		4

#define HID_NCP_CMD_LEN			0
#define HID_NCP_CMD_DATA		1

#define MODULE_NAME "hid_ntrig"

/**
 * NCP DEFINES
 */
#define NCP_HEADER              0 /* 0x7E */
#define NCP_HOSTADD_LSB		1
#define NCP_HOSTADD_MSB		2
#define NCP_MSG_LEN_LSB		3
#define NCP_MSG_LEN_MSB		4
#define NCP_MSG_TYPE		5
#define NCP_CODE                6
#define NCP_GROUP               7
#define NCP_RTN_CODE_1		8
#define NCP_RTN_CODE_2		9
#define NCP_RTN_CODE_3		10
#define NCP_RTN_CODE_4		11
#define NCP_RESERVED_LSB	12
#define NCP_RESERVED_MSB	13

#define NTRIG_DEBUG_LEVEL_NONE	0
#define NTRIG_DEBUG_LEVEL_STD	1
#define NTRIG_DEBUG_LEVEL_ALL	10

extern int ntrig_debug_flag;
void set_ntrig_debug_flag(char debug_flag);
char get_ntrig_debug_flag_as_char(void);

#define info(format, arg...) \
	printk(KERN_INFO "%s: " format , MODULE_NAME , ## arg)

/* print message if the debug flag is not 0 */
#define ntrig_dbg(format, arg...) \
do { \
	if (ntrig_debug_flag) \
		printk(KERN_DEBUG "%s: " format, MODULE_NAME , ## arg); \
} while (0)

/* print message if the debug flag is same or greater than specified level */
#define ntrig_dbg_lvl(level, format, arg...) \
do { \
	if (ntrig_debug_flag >= level) \
		printk(KERN_DEBUG "%s: " format, MODULE_NAME , ## arg); \
} while (0)

#define ntrig_err(format, arg...) \
do { \
	printk(KERN_ERR "%s: " format, MODULE_NAME , ## arg); \
} while (0)

#endif /* _DTRIG_COMMON_H */
