/**
 *
 * Synaptics Register Mapped Interface (RMI4) Function $54 header.
 * Copyright (c) 2007 - 2011, Synaptics Incorporated
 *
 * TODO: Description here
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

#if !defined(_RMI_F54_H)
#define _RMI_F54_H

/* define fn $54 commands */
#define GET_REPORT                1
#define FORCE_CAL                 2

/* define report types */
enum Report {
	/* The numbering should follow automatically, here for clarity */
	reserved = 0,
	r8bit_image = 1,
	r16bit_image = 2,
	autoscan = 3,
	trans2trans = 4,
	trans2trans_short = 5,
	trans_open = 6,
	rec2rec = 7,
	rec_open = 8,
	high_resistance = 9
};

/* data specific to fn $54 that needs to be kept around */
struct rmi_fn_54_data {
	unsigned char cmd;
	enum Report reporttype;
	unsigned char fifoindexlo;
	unsigned char fifoindexhi;
	unsigned char numrxelectrodes;
	unsigned char numtxelectrodes;
	unsigned char status;
	bool no_auto_cal;
	/* May need to do something to make sure this reflects what is
	 * currently in data. */
	unsigned int reportsize;
	unsigned char *report_data;
	unsigned int bufsize;
	struct mutex data_mutex;
	struct lock_class_key data_key;
};



void FN_54_inthandler(struct rmi_function_info *rmifninfo,
		      unsigned int asserted_IRQs);
int FN_54_config(struct rmi_function_info *rmifninfo);
int FN_54_init(struct rmi_function_device *function_device);
int FN_54_detect(struct rmi_function_info *rmifninfo);
#endif
