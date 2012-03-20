/**
 *
 * Synaptics Register Mapped Interface (RMI4) I2C Physical Layer Driver.
 * Copyright (c) 2007-2010, Synaptics Incorporated
 *
 * Author: Js HA <js.ha@stericsson.com> for ST-Ericsson
 * Author: Naveen Kumar G <naveen.gaddipati@stericsson.com> for ST-Ericsson
 * Copyright 2010 (c) ST-Ericsson AB
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

#ifndef _SYNAPTICS_RMI4_H_
#define _SYNAPTICS_RMI4_H_

struct rmi4_fn_ops;
/**
 * struct rmi4_fn_desc - contains the function descriptor information
 * @query_base_addr: base address for query
 * @cmd_base_addr: base address for command
 * @ctrl_base_addr: base address for control
 * @data_base_addr: base address for data
 * @intr_src_count: count for the interrupt source
 * @fn_number: function number
 *
 * This structure is used to gives the function descriptor information
 * of the particular functionality.
 */
struct rmi4_fn_desc {
	unsigned char	query_base_addr;
	unsigned char	cmd_base_addr;
	unsigned char	ctrl_base_addr;
	unsigned char	data_base_addr;
	unsigned char	intr_src_count;
	unsigned char	fn_number;
};

/**
 * struct rmi4_fn - contains the function information
 * @query_base_addr: base address for query with page number
 * @cmd_base_addr: base address for command with page number
 * @ctrl_base_addr: base address for control with page number
 * @data_base_addr: base address for data
 * @intr_src_count: count for the interrupt source
 * @fn_number: function number
 * @num_of_data_points: number of fingers touched
 * @size_of_data_register_block: data register block size
 * @index_to_intr_reg: index for interrupt register
 * @intr_mask: interrupt mask value
 * @link: linked list for function descriptors
 * @ops: rmi4 function's operation methods
 * @fn_data: function's specific data
 */
struct rmi4_fn {
	unsigned short query_base_addr;
	unsigned short cmd_base_addr;
	unsigned short ctrl_base_addr;
	unsigned short data_base_addr;
	unsigned char intr_src_count;
	unsigned char fn_number;
	unsigned char num_of_data_points;
	unsigned char size_of_data_register_block;
	unsigned char index_to_intr_reg;
	unsigned char intr_mask;
	struct list_head    link;
	struct rmi4_fn_ops  *ops;
	void                *fn_data;
};

/**
 * struct rmi4_device_info - contains the rmi4 device information
 * @version_major: protocol major version number
 * @version_minor: protocol minor version number
 * @manufacturer_id: manufacturer identification byte
 * @product_props: product properties information
 * @product_info: product info array
 * @date_code: device manufacture date
 * @tester_id: tester id array
 * @serial_number: serial number for that device
 * @product_id_string: product id for the device
 * @support_fn_list: linked list for device information
 *
 * This structure gives information about the number of data sources and
 * the number of data registers associated with the function.
 */
struct rmi4_device_info {
	unsigned int		version_major;
	unsigned int		version_minor;
	unsigned char		manufacturer_id;
	unsigned char		product_props;
	unsigned char		product_info[2];
	unsigned char		date_code[3];
	unsigned short		tester_id;
	unsigned short		serial_number;
	unsigned char		product_id_string[11];
	struct list_head	support_fn_list;
};

/**
 * struct rmi4_data - contains the rmi4 device data
 * @rmi4_mod_info: structure variable for rmi4 device info
 * @input_dev: pointer for input device
 * @i2c_client: pointer for i2c client
 * @board: constant pointer for touch platform data
 * @rmi4_page_mutex: mutex for rmi4 page
 * @current_page: variable for integer
 * @number_of_interrupt_register: interrupt registers count
 * @fn01_ctrl_base_addr: control base address for fn01
 * @fn01_query_base_addr: query base address for fn01
 * @fn01_data_base_addr: data base address for fn01
 * @sensor_max_x: sensor maximum x value
 * @sensor_max_y: sensor maximum y value
 * @regulator: pointer to the regulator structure
 * @irq: irq number
 * @es: early suspend hooks
 *
 * This structure gives the device data information.
 */
struct rmi4_data {
	const struct rmi4_platform_data *board;
	struct rmi4_device_info rmi4_mod_info;
	struct input_dev	*input_dev;
	struct i2c_client	*i2c_client;
	struct mutex		rmi4_page_mutex;
	unsigned int		number_of_interrupt_register;
	unsigned short		fn01_ctrl_base_addr;
	unsigned short		fn01_query_base_addr;
	unsigned short		fn01_data_base_addr;
	int			current_page;
	int			sensor_max_x;
	int			sensor_max_y;
	struct regulator	*regulator;
	int			irq;
	struct early_suspend	es;
};

/**
 * struct rmi4_fn_ops - contains the function's operation methods
 */
struct rmi4_fn_ops {
	unsigned char fn_number;
	int (*detect)(struct rmi4_data *pdata, struct rmi4_fn *rfi,
			unsigned int intr_cnt);
	int (*config)(struct rmi4_data *pdata, struct rmi4_fn *rfi);
	int (*irq_handler)(struct rmi4_data *pdata, struct rmi4_fn *rfi);
	void (*remove)(struct rmi4_fn *rfi);
};

/**
 * struct rmi4_button_data - contains the button function data
 * @num_of_bttns: number of buttons that supported
 * @status: button down/up status
 * @bttns_map: key code for each button that reported by input device
 */
struct rmi4_button_data {
	int num_of_bttns;
	bool *status;
	unsigned char *bttns_map;
};

/**
 * struct rmi4_touchpad_data - contains the touch function data
 * @buffer: buffer to store finger registers
 * @size: size of the buffer
 */
struct rmi4_touchpad_data {
	unsigned char *buffer;
	int size;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void rmi4_early_suspend(struct early_suspend *h);
void rmi4_late_resume(struct early_suspend *h);
#endif

int rmi4_touchpad_detect(struct rmi4_data *pdata,
				struct rmi4_fn *rfi, unsigned int intr_cnt);
int rmi4_touchpad_config(struct rmi4_data *pdata, struct rmi4_fn *rfi);
int rmi4_touchpad_irq_handler(struct rmi4_data *pdata, struct rmi4_fn *rfi);
void rmi4_touchpad_remove(struct rmi4_fn *rfi);

int rmi4_button_detect(struct rmi4_data *pdata,
				struct rmi4_fn *rfi, unsigned int intr_cnt);
int rmi4_button_irq_handler(struct rmi4_data *pdata, struct rmi4_fn *rfi);
void rmi4_button_remove(struct rmi4_fn *);

#endif
