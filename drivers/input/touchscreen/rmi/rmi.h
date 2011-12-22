/**
 *
 * Synaptics Register Mapped Interface (RMI4) Header File.
 * Copyright (c) 2007 - 2011, Synaptics Incorporated
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

#if !defined(_RMI_H)
#define _RMI_H

/*  RMI4 Protocol Support
 */


/* Every function on an RMI device is identified by a one byte function number.
 * The hexadecimal representation of this byte is used in the function name.
 * For example, the function identified by the byte 0x11 is referred to as
 * F11 (or sometimes FN11).  In the extremely improbable event that F11 is no
 * longer identified by 0x11, though, we provide these handy #defines.
 */
#define RMI_F01_INDEX 0x01
#define RMI_F05_INDEX 0x05
#define RMI_F11_INDEX 0x11
#define RMI_F19_INDEX 0x19
#define RMI_F34_INDEX 0x34
#define RMI_F54_INDEX 0x54

/* This byte has information about the communications protocol.  See the RMI4
 * specification for details of what exactly is there.
 */
#define RMI_PROTOCOL_VERSION_ADDRESS 0xA0FD

/* For each function present on the RMI device, we need to get the RMI4 Function
 * Descriptor info from the Page Descriptor Table. This will give us the
 * addresses for Query, Command, Control, Data and the Source Count (number
 * of sources for this function) and the function id.
 */
struct rmi_function_descriptor {
	unsigned char query_base_addr;
	unsigned char command_base_addr;
	unsigned char control_base_addr;
	unsigned char data_base_addr;
	unsigned char interrupt_source_count;
	unsigned char function_number;
};

/* The product descriptor table starts here, and continues till we get
 * a function ID of 0x00 or 0xFF.
 */
#define RMI_PDT_START_ADDRESS 0x00E9

#define RMI_IS_VALID_FUNCTION_ID(id) (id != 0x00 && id != 0xFF)

#endif
