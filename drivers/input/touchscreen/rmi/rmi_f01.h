/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $01 header.
 * Copyright (c) 2007 - 2011, Synaptics Incorporated
 *
 * There is only one function $01 for each RMI4 sensor. This will be
 * the function that is used to set sensor control and configurations
 * and check the interrupts to find the source function that is interrupting.
 *
 *
 */
/*
 * This file is licensed under the GPL2 license.
 *
 *#############################################################################
 * GPL
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 *#############################################################################
 */

#if !defined(_RMI_F01_H)
#define _RMI_F01_H

/*  This encapsulates the information found using the RMI4 Function $01
 *  query registers. There is only one Function $01 per device.
 *
 *  Assuming appropriate endian-ness, you can populate most of this
 *  structure by reading query registers starting at the query base address
 *  that was obtained from RMI4 function 0x01 function descriptor info read
 *  from the Page Descriptor Table.
 *
 *  Specific register information is provided in the comments for each field.
 *  For further reference, please see the "Synaptics RMI 4 Interfacing
 *  Guide" document : go to http://www.synaptics.com/developers/manuals - and
 *  select "Synaptics RMI 4 Interfacting Guide".
 */
#define F01_PRODUCT_INFO_LENGTH 2
#define F01_DATE_CODE_LENGTH 3
#define F01_PRODUCT_ID_LENGTH 10
struct rmi_F01_query {
	/* The manufacturer identification byte. */
	unsigned char mfgid;

	/* The Product Properties information. */
	unsigned char properties;

	/* The product info bytes. */
	unsigned char prod_info[F01_PRODUCT_INFO_LENGTH];

	/* Date Code - Year, Month, Day. */
	unsigned char date_code[F01_DATE_CODE_LENGTH];

	/* Tester ID (14 bits). */
	unsigned short tester_id;

	/* Serial Number (14 bits). */
	unsigned short serial_num;

	/* A null-terminated string that identifies this particular product. */
	char prod_id[F01_PRODUCT_ID_LENGTH];
};

/* This encapsulates the F01 Device Control control registers.
 * TODO: This isn't right.  The number of interrupt enables needs to be
 * determined dynamically as the sensor is initialized.  Fix this.
 */
struct rmi_F01_control {
	unsigned char device_control;
	unsigned char interrupt_enable[1];
};


/* This encapsulates the F01 Device Control data registers.
 * TODO: This isn't right.  The number of irqs needs to be determined
 * dynamically as the sensor is initialized.  Fix this.
 */
struct rmi_F01_data {
	unsigned char device_status;
	unsigned char irqs[1];
};


void FN_01_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs);
int FN_01_config(struct rmi_function_info *rmifninfo);
int FN_01_init(struct rmi_function_device *function_device);
int FN_01_detect(struct rmi_function_info *rmifninfo);
void FN_01_attention(struct rmi_function_info *rmifninfo);
int FN_01_suspend(struct rmi_function_info *rmifninfo);
void FN_01_resume(struct rmi_function_info *rmifninfo);

#define RMI_F01_SLEEP_MODE_MASK 0x03
/* Position of bits in control register. */
#define RMI_F01_SLEEP_MODE_OFFSET 0

#define RMI_SLEEP_MODE_NORMAL (0x00)
#define RMI_SLEEP_MODE_SENSOR_SLEEP (0x01)
#define RMI_SLEEP_MODE_RESERVED0 (0x02)
#define RMI_SLEEP_MODE_RESERVED1 (0x03)

#define RMI_IS_VALID_SLEEPMODE(mode) \
	(mode < RMI_SLEEP_MODE_NORMAL || mode > RMI_SLEEP_MODE_RESERVED1)

/* This bit is used to disable current sleep mode. */
#define RMI_F01_NO_SLEEP_MASK 0x04
/* Position of bits in control register. */
#define RMI_F01_NO_SLEEP_OFFSET 2

#define RMI_NO_SLEEP_ENABLE (0x01)
#define RMI_NO_SLEEP_DISABLE (0x00)

#define RMI_F01_INIT_REFLASH_MASK 0x04
/* Position of bits in query 1 register (product properties). */
#define RMI_F01_INIT_REFLASH_OFFSET 2
#endif
