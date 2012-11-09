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

#define RMI4_TOUCHPAD_FUNC_NUM      0x11
#define RMI4_BUTTON_FUNC_NUM        0x1a
#define RMI4_DEV_CTL_FUNC_NUM       0x01
#define RMI4_ANALOG_FUNC_NUM        0x54
#define RMI4_FLASH_FW_FUNC_NUM      0x34
#define RMI4_FW_VERSION             0x4
#define RMI4_PAGE_SIZE              0x100
#define RMI4_PAGE_SELECT_REG        0xff
#define RMI4_MAX_PAGE               0xff
#define RMI4_RESET_DELAY            50

#define PDT_START_SCAN_LOCATION     0x00E9
#define PDT_END_SCAN_LOCATION       0x000A
#define PDT_ENTRY_SIZE              0x0006
#define PDT_PROPERTIES_LOCATION     0x00EF

#define RMI4_DEVICE_RESET_CMD       (0x01)
#define RMI4_SLEEP_MODE_NORMAL      (0x00)
#define RMI4_END_OF_PDT(id)         ((id) == 0x00 || (id) == 0xff)

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
	u8	query_base_addr;
	u8	cmd_base_addr;
	u8	ctrl_base_addr;
	u8	data_base_addr;
	u8	intr_src_count:3;
	u8	reserved_1:2;
	u8	func_version:2;
	u8	reserved_2:1;
	u8	fn_number;
} __packed;

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
	u16 query_base_addr;
	u16 cmd_base_addr;
	u16 ctrl_base_addr;
	u16 data_base_addr;
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
 * @input_ts_dev: pointer for input touch device
 * @input_key_dev: pointer for input key device
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
	struct input_dev	*input_ts_dev;
	struct input_dev	*input_key_dev;
	struct i2c_client	*i2c_client;
	struct mutex		rmi4_page_mutex;
	unsigned int		number_of_interrupt_register;
	u16		        fn01_ctrl_base_addr;
	u16		        fn01_query_base_addr;
	u16                     fn01_data_base_addr;
	int			current_page;
	int			sensor_max_x;
	int			sensor_max_y;
	int			touch_type;
	struct regulator	*regulator;
	int			irq;
	struct early_suspend	es;
#ifdef DEBUG
	u16 dbg_reg_addr;
	unsigned short dbg_fn_num;
#endif
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
	u8 *bttns_map;
};

/**
 * struct rmi4_touchpad_data - contains the touch function data
 * @buffer: buffer to store finger registers
 * @size: size of the buffer
 */
struct rmi4_touchpad_data {
	u8 *buffer;
	int size;
};

union pdt_properties {
	struct {
		u8 reserved_1:6;
		u8 has_bsr:1;
		u8 reserved_2:1;
	} __packed;
	u8 regs[1];
};

union f01_basic_queries {
	struct {
		u8 manufacturer_id;

		u8 custom_map:1;
		u8 non_compliant:1;
		u8 has_lts:1;
		u8 has_sensor_id:1;
		u8 has_charger_input:1;
		u8 has_adjustable_doze:1;
		u8 has_adjustable_doze_holdoff:1;
		u8 has_product_properties_2:1;

		u8 productinfo_1:7;
		u8 q2_bit_7:1;
		u8 productinfo_2:7;
		u8 q3_bit_7:1;

		u8 year:5;
		u8 month:4;
		u8 day:5;
		u8 cp1:1;
		u8 cp2:1;
		u8 wafer_id1_lsb;
		u8 wafer_id1_msb;
		u8 wafer_id2_lsb;
		u8 wafer_id2_msb;
		u8 wafer_id3_lsb;
	} __packed;
	u8 regs[11];
};

union f01_device_status {
	struct {
		u8 status_code:4;
		u8 reserved:2;
		u8 flash_prog:1;
		u8 unconfigured:1;
	} __packed;
	u8 regs[1];
};

union f01_device_control_0 {
	struct {
		u8 sleep_mode:2;
		u8 nosleep:1;
		u8 reserved:2;
		u8 charger_input:1;
		u8 report_rate:1;
		u8 configured:1;
	} __packed;
	u8 regs[1];
};

union f34_query_regs {
	struct {
		u16 reg_map:1;
		u16 unlocked:1;
		u16 has_config_id:1;
		u16 reserved:5;
		u16 block_size;
		u16 fw_block_count;
		u16 config_block_count;
	} __packed;
	struct {
		u8 regs[7];
		u16 address;
	};
};

union f34_control_status {
	struct {
		u8 command:4;
		u8 status:3;
		u8 program_enabled:1;
	} __packed;
	struct {
		u8 regs[1];
		u16 address;
	};
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void rmi4_early_suspend(struct early_suspend *h);
void rmi4_late_resume(struct early_suspend *h);
#endif

int rmi4_i2c_block_read(struct rmi4_data *pdata, u16 addr, u8 *val, int size);
int rmi4_i2c_byte_read(struct rmi4_data *pdata, u16 addr, u8 *val);
int rmi4_i2c_block_write(struct rmi4_data *pdata, u16 addr, u8 *val, int size);
int rmi4_i2c_byte_write(struct rmi4_data *pdata, u16 addr, u8 data);

int rmi4_touchpad_detect(struct rmi4_data *pdata,
				struct rmi4_fn *rfi, unsigned int cnt);
int rmi4_touchpad_config(struct rmi4_data *pdata, struct rmi4_fn *rfi);
int rmi4_touchpad_irq_handler(struct rmi4_data *pdata, struct rmi4_fn *rfi);
void rmi4_touchpad_remove(struct rmi4_fn *rfi);

int rmi4_button_detect(struct rmi4_data *pdata,
				struct rmi4_fn *rfi, unsigned int cnt);
int rmi4_button_irq_handler(struct rmi4_data *pdata, struct rmi4_fn *rfi);
void rmi4_button_remove(struct rmi4_fn *);

int rmi4_fw_update(struct rmi4_data *pdata,
		struct rmi4_fn_desc *f01_pdt, struct rmi4_fn_desc *f34_pdt);
#endif
