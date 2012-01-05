/*
 *  Atmel maXTouch Touchscreen Controller Driver
 *
 *
 *  Copyright (C) 2010 Atmel Corporation
 *  Copyright (C) 2010 Ulf Samuelsson (ulf@atmel.com)
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *  Contains changes by Wind River Systems, 2010-09-29
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/pm_runtime.h>
#include <linux/bitops.h>
#include <linux/jiffies.h>
#include <linux/atmel_mxt_ts.h>

#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <asm/io.h>
/* 1-->WINTEK  0-->SINTEK */
#define TOUCH_PANEL_IS_WINTEK 0
#define MAGIC_PRODUCT_T38_BYTE7_WINTEK 0x02
#define MAGIC_PRODUCT_T38_BYTE7_SINTEK 0x03
#if TOUCH_PANEL_IS_WINTEK
#define MAGIC_PRODUCT_T38_BYTE7 MAGIC_PRODUCT_T38_BYTE7_WINTEK
#else /* SINTEK */
#define MAGIC_PRODUCT_T38_BYTE7 MAGIC_PRODUCT_T38_BYTE7_SINTEK
#endif

/* Routines for memory access within a 16 bit address space */
static void mxt_calibrate(struct i2c_client *client);
static int mxt_read_block(struct i2c_client *client, u16 addr, u16 length,
			  u8 *value);
static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value);
static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length,
			   u8 *value);
#ifdef CONFIG_HAS_EARLYSUSPEND
void mxt_early_suspend(struct early_suspend *h);
void mxt_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_DEBUG_FS
static int  intel_mid_add_debugfs(void);
static unsigned char reg_add = 0;
static unsigned char reg_add_offset = 0;
#endif

#define DRIVER_VERSION "0.9a"
#define MULTI_TOUCH_ENABLED 1
#define MXT_RESET_DELAY   65 /* ms */
static int debug = DEBUG_INFO;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Activate debugging output");

static int stored_size[MXT_MAX_NUM_TOUCHES];
static int stored_x[MXT_MAX_NUM_TOUCHES];
static int stored_y[MXT_MAX_NUM_TOUCHES];
static int iForceUpdateConfigValue = 0;
static struct mxt_data *mxt_es;
/* For backup NVM */
u8 mxt_debug = 1;
u16 T6_addr;
u16 T38_addr;
u8 v20_T38[] = { 11, 10, 26, 1, 0, 0, 0, MAGIC_PRODUCT_T38_BYTE7 };
u8 arg1 = 0;
u8 arg2 = 0;
u8 mxt_cmd = 0;
u16 low_boundary = 0;
u16 high_boundary = 0;
u8 slave_addr = MXT1386_I2C_ADDR1;

/* Device Info descriptor */
/* Parsed from maXTouch "Id information" inside device */
struct mxt_device_info {
	u8   family_id;
	u8   variant_id;
	u8   major;
	u8   minor;
	u8   build;
	u8   num_objs;
	u8   x_size;
	u8   y_size;
	char family_name[16];	 /* Family name */
	char variant_name[16];    /* Variant name */
	u16  num_nodes;           /* Number of sensor nodes */
};

/* object descriptor table, parsed from maXTouch "object table" */
struct mxt_object {
	u16 chip_addr;
	u8  type;
	u8  size;
	u8  instances;
	u8  num_report_ids;
};


/* Mapping from report id to object type and instance */
struct report_id_map {
	u8  object;
	u8  instance;
/*
 * This is the first report ID belonging to object. It enables us to
 * find out easily the touch number: each touch has different report
 * ID (which are assigned to touches in increasing order). By
 * subtracting the first report ID from current, we get the touch
 * number.
 */
	u8  first_rid;
};


/* Driver datastructure */
struct mxt_data {
	struct i2c_client    *client;
	struct mutex	     dev_mutex;
	struct input_dev     *touch_input;
	struct input_dev     *key_input;
	char                 touch_phys_name[32];
	char                 key_phys_name[32];
	int                  irq;

	u16                  last_read_addr;
	bool                 new_msgs;

	int                  valid_irq_counter;
	int                  invalid_irq_counter;
	int                  irq_counter;
	int                  message_counter;
	int                  read_fail_counter;

	int                  bytes_to_read;

	u8                   xpos_format;
	u8                   ypos_format;

	u8                   numtouch;

	struct mxt_device_info	device_info;

	u32		     info_block_crc;
	u32                  configuration_crc;
	u16                  report_id_count;
	struct report_id_map *rid_map;
	struct mxt_object    *object_table;

	u16                  msg_proc_addr;
	u8                   message_size;

	u16                  max_x_val;
	u16                  max_y_val;
	u16                  orientation;

	void                 (*init_hw)(void);
	void                 (*exit_hw)(void);
	u8                   (*valid_interrupt)(void);
	u8                   (*read_chg)(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
	u8                   T7[3];
	struct early_suspend es;
	bool                 suspended;
#endif
	/* Put only non-touch messages to buffer if this is set */
	char                 nontouch_msg_only;

	int                  prev_key;

	int                  mxt_intr_gpio;
	int                  mxt_reset_gpio;

	u8                   *message_buf;
	int                  recal_flag;
	u64                  timestamp;
	u16                  finger_pressed;
	u8                   finger_count;
	u8                   first_pressed;
	u8                   grip_suppression;
	u8                   face_suppression;

	struct point_info    pre_data[10];
};

/*
 * This struct is used for i2c transfers.
 */
struct mxt_i2c_byte_transfer {
	__le16 le_addr;
	u8     data;
} __attribute__ ((packed));

#define I2C_RETRY_COUNT 5
#define I2C_PAYLOAD_SIZE 254

/*
 * Check whether we have multi-touch enabled kernel; if not, report just the
 * first touch (on mXT224, the maximum is 10 simultaneous touches).
 * Because just the 1st one is reported, it might seem that the screen is not
 * responding to touch if the first touch is removed while the screen is being
 * touched by another finger, so beware.
 *
 * TODO: investigate if there is any standard set of input events that upper
 * layers are expecting from a touchscreen? These can however be different for
 * different platforms, and customers may have different opinions too about
 * what should be interpreted as right-click, for example.
 *
 */

static const u8	*obj_typ_name[] = {
	[0]  = "Reserved",
	[5]  = "GEN_MESSAGEPROCESSOR_T5",
	[6]  = "GEN_COMMANDPROCESSOR_T6",
	[7]  = "GEN_POWERCONFIG_T7",
	[8]  = "GEN_ACQUIRECONFIG_T8",
	[9]  = "TOUCH_MULTITOUCHSCREEN_T9",
	[15] = "TOUCH_KEYARRAY_T15",
	[18] = "SPT_COMMSCONFIG_T18",
	[19] = "SPT_GPIOPWM_T19",
	[20] = "PROCI_GRIPFACESUPPRESSION_T20",
	[22] = "PROCG_NOISESUPPRESSION_T22",
	[23] = "TOUCH_PROXIMITY_T23",
	[24] = "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
	[25] = "SPT_SELFTEST_T25",
	[27] = "PROCI_TWOTOUCHGESTUREPROCESSOR_T27",
	[28] = "SPT_CTECONFIG_T28",
	[37] = "DEBUG_DIAGNOSTICS_T37",
	[38] = "SPT_USER_DATA_T38",
	[40] = "PROCI_GRIPSUPPRESSION_T40",
	[41] = "PROCI_PALMSUPPRESSION_T41",
	[42] = "PROCI_FACESUPPRESSION_T42",
	[43] = "SPT_DIGITIZER_T43",
	[44] = "SPT_MESSAGECOUNT_T44",
};

/*
 * All entries spare upto 255
 */
#define RESERVED_T255                             255u
void mxt_config_init(struct mxt_data *mxt)
{
	int err = 0;
	dev_info(&mxt->client->dev, "In function %s", __func__);
	/* Fix for current comsumption. */
	u8 v20_T7[] = { 32, 16, 50 };
	u8 v20_T8[]  = { 10, 0, 15, 15, 0, 0, 5, 45, 10, 192 };
#if TOUCH_PANEL_IS_WINTEK
	u8 v20_T9[] = { 143, 0, 0, 27, 42, 0, 16, 60, 3, 3, 0, 0, 3, 14, 10, 20,
		20, 10, 31, 3, 255, 4, 0, 0, 0, 0, 0, 0, 64, 0, 15, 15,
		49, 52 };
#else /* SINTEK */
	u8 v20_T9[] = { 143, 0, 0, 28, 42, 0, 16, 60, 3, 3, 0, 0, 3, 14, 10, 20,
		20, 10, 31, 3, 255, 4, 0, 0, 0, 0, 0, 0, 64, 0, 15, 15,
		49, 52 };
#endif

	u8 v20_T15[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 v20_T18[] = { 0, 0 };
	u8 v20_T22[] = { 5, 0, 0, 0, 0, 0, 0, 0, 45, 0, 0, 15, 25, 35, 255, 255,
		0 };
	u8 v20_T24[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0 };
	u8 v20_T25[] = { 3, 0, 248, 42, 112, 23, 0, 0, 0, 0, 0, 0, 0, 0 };
	u8 v20_T27[] = { 0, 0, 0, 0, 0, 0, 0 };
	u8 v20_T28[] = { 0, 0, 0, 8, 16, 60 };
	u8 v20_T40[] = { 0, 0, 0, 0, 0 };
	u8 v20_T41[] = { 1, 0, 0, 35, 5, 20, 170 };
	u8 v20_T43[] = { 0, 0, 0, 0, 0, 0 };

	if (slave_addr == MXT1386_I2C_ADDR2) { /* 0X4C means Wintek and 0x4D means Sintek */
		v20_T9[3] = 28;
		v20_T22[8] = 20;
	}

	u8 i = 0, max_objs = 0, j = 0;
	u16 addr;
	struct mxt_object *obj_index;

	dev_dbg(&mxt->client->dev, "In function %s", __func__);
	max_objs = mxt->device_info.num_objs;
	obj_index = mxt->object_table;

	for (i = 0; i < max_objs; i++) {
		addr = obj_index->chip_addr;
		for (j = 1; j < I2C_RETRY_COUNT; j++) {
			err = 1;
			switch (obj_index->type) {
			case MXT_GEN_POWERCONFIG_T7:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T7), v20_T7);
				break;
			case MXT_GEN_ACQUIRECONFIG_T8:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T8), v20_T8);
				break;
			case MXT_TOUCH_MULTITOUCHSCREEN_T9:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T9), v20_T9);
				dev_dbg(&mxt->client->dev, "init multitouch object");
				break;
			case MXT_TOUCH_KEYARRAY_T15:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T15), v20_T15);
				break;
			case MXT_SPT_COMMSCONFIG_T18:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T18), v20_T18);
				break;
			case MXT_PROCG_NOISESUPPRESSION_T22:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T22), v20_T22);
				break;
			case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T24), v20_T24);
				break;
			case MXT_SPT_SELFTEST_T25:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T25), v20_T25);
				break;
			case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T27), v20_T27);
			break;
			case MXT_SPT_CTECONFIG_T28:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T28), v20_T28);
			break;
			case MXT_USER_INFO_T38:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T38), v20_T38);
				break;
			case MXT_PROCI_GRIPSUPPRESSION_T40:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T40), v20_T40);
				break;
			case MXT_PROCI_PALMSUPPRESSION_T41:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T41), v20_T41);
				break;
			case MXT_SPT_DIGITIZER_T43:
				err = mxt_write_block(mxt->client, addr, sizeof(v20_T43), v20_T43);
				break;
			default:
				break;
			}
			if (err > 0)
				break;
		}
		if (j >= I2C_RETRY_COUNT)
			dev_info(&mxt->client->dev,
				"touch: config init abnormal,obj_index's type is %d",
				obj_index->type);

		obj_index++;
	}
	mxt_calibrate(mxt->client);
	dev_dbg(&mxt->client->dev, "config init Done.");
}

/* Returns object address in mXT chip, or zero if object is not found */
static u16 get_object_address(uint8_t object_type,
			      uint8_t instance,
			      struct mxt_object *object_table,
			      int max_objs)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = 0;
	struct mxt_object *obj;

	while ((object_table_index < max_objs) && !address_found) {
		obj = &object_table[object_table_index];
		if (obj->type == object_type) {
			address_found = 1;
			/* Are there enough instances defined in the FW? */
			if (obj->instances >= instance) {
				address = obj->chip_addr +
					  (obj->size + 1) * instance;
			} else {
				break;
			}
		}
		object_table_index++;
	}
	return address;
}

int BackupNVM(struct mxt_data *mxt)
{
	u8 buf[1] = {0x55};
	u8 T38_buf[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

	/* Backup the non-volatile memory (NVM). */
	T6_addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6, 0,
			mxt->object_table, mxt->device_info.num_objs);
	T38_addr = get_object_address(MXT_USER_INFO_T38, 0,
			mxt->object_table, mxt->device_info.num_objs);

	pr_info("BackupNVM T6_addr+1=0x%4x, T38_addr=0x%4x.\n", T6_addr+1, T38_addr);
	mxt_read_block(mxt->client, T38_addr, sizeof(T38_buf), T38_buf);

	if (iForceUpdateConfigValue == 1) {
			pr_info("%s force update. ver=%d.%d.%d.%d --> %d.%d.%d.%d\n", \
			__func__,                                         \
			T38_buf[0], T38_buf[1], T38_buf[2], T38_buf[3],   \
			v20_T38[0], v20_T38[1], v20_T38[2], v20_T38[3]);
	} else {
		#define PRINT_T38_V20() do {                                      \
			pr_info("%s not update. ver=%d.%d.%d.%d (%d)> %d.%d.%d.%d  (%d)\n", \
			__func__,                                         \
			T38_buf[0], T38_buf[1], T38_buf[2], T38_buf[3], T38_buf[7],  \
			v20_T38[0], v20_T38[1], v20_T38[2], v20_T38[3], v20_T38[7]);  \
		} while (0)

		/* Check T38 previous config version. */
		if ((T38_buf[0] != 0 && T38_buf[1] != 0) &&
			(T38_buf[2] != 0 && T38_buf[3] != 0) &&
			(T38_buf[4] == 0 && T38_buf[5] == 0) &&
			(T38_buf[6] == 0 && T38_buf[7] == MAGIC_PRODUCT_T38_BYTE7)) {
			if (T38_buf[0] > v20_T38[0]) {
				PRINT_T38_V20();
				return 0;
			} else if (T38_buf[0] == v20_T38[0]) {
				if (T38_buf[1] > v20_T38[1]) {
					PRINT_T38_V20();
					return 0;
				} else if (T38_buf[1] == v20_T38[1]) {
					if (T38_buf[2] > v20_T38[2]) {
						PRINT_T38_V20();
						return 0;
					} else if (T38_buf[2] == v20_T38[2]) {
						if  (T38_buf[3] >= v20_T38[3]) {
							PRINT_T38_V20();
							return 0;
						}
					}
				}
			}
		} else {
			 if (T38_buf[7] != MAGIC_PRODUCT_T38_BYTE7) {
				pr_info("%s  PRODUCT_T38_BYTE7 diff, force update. ver=%d.%d.%d.%d(%d) --> %d.%d.%d.%d(%d)\n", \
				__func__,                                         \
				T38_buf[0], T38_buf[1], T38_buf[2], T38_buf[3], T38_buf[7],  \
				v20_T38[0], v20_T38[1], v20_T38[2], v20_T38[3], v20_T38[7]);
			 }

		}
		#undef PRINT_T38_V20
	}

	mxt_config_init(mxt);

	if (mxt_write_block(mxt->client, T6_addr+1, sizeof(buf), buf) == 1)
		pr_info("Backup the configurations to non-volatile memory.\n");
	else {
		pr_err("Backup the configurations to non-volatile memory failed!!!\n");
		return -1;
	}

	/*Reset the IC.*/
	if (mxt_write_block(mxt->client, T6_addr, sizeof(buf), buf) == 1)
		pr_info("Software reset the IC.\n");
	else {
		pr_err("Software reset the IC failed!!!\n");
		return -1;
	}

	msleep(400);
	pr_info("BackupNVM done. ver=%d.%d.%d.%d\n", v20_T38[0], v20_T38[1], v20_T38[2], v20_T38[3]);
	return 0;
}

/* Calculates the 24-bit CRC sum. */
static u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = ((((u16) byte2) << 8u) | byte1);
	result = ((crc << 1u) ^ data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

/* Calibrate rz*/
static void mxt_calibrate(struct i2c_client *client)
{
	u16 base_addr;
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	dev_dbg(&mxt->client->dev, "In function %s", __func__);

	base_addr = get_object_address(MXT_GEN_COMMANDPROCESSOR_T6,
				0, mxt->object_table,
				mxt->device_info.num_objs);
	base_addr += MXT_ADR_T6_CALIBRATE;
	mxt_write_byte(mxt->client, base_addr, 0x55);
}


/*
 * Reads a block of bytes from given address from mXT chip. If we are
 * reading from message window, and previous read was from message window,
 * there's no need to write the address pointer: the mXT chip will
 * automatically set the address pointer back to message window start.
 */

static int mxt_read_block(struct i2c_client *client,
		   u16 addr,
		   u16 length,
		   u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16	le_addr;
	struct mxt_data *mxt;
	int err;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) &&
			(addr == mxt->msg_proc_addr)) {
			pm_runtime_get_sync(&client->dev);
			if  (i2c_master_recv(client, value, length) == length)
				err = length;
			else
				err = -EIO;
			pm_runtime_put(&client->dev);
			return err;
		} else {
			mxt->last_read_addr = addr;
		}
	}

	dev_dbg(&client->dev, "Writing address pointer & reading %d bytes"
		" in on i2c transaction...\n", length);

/* FIH { */
/* Avoid stop delay */
#if 0
	le_addr = cpu_to_le16(addr);
	msg[0].addr  = client->addr;
	msg[0].flags = 0x00;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *) &le_addr;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = length;
	msg[1].buf   = (u8 *) value;
	pm_runtime_get_sync(&client->dev);
	if  (i2c_transfer(adapter, msg, 2) == 2)
		err = length;
	else
		err = -EIO;
	pm_runtime_put(&client->dev);
#else
{
	u8 reg[2];
	reg[0] = addr & 0xFF;
	reg[1] = (addr>>8) & 0xFF;

	err = i2c_master_send(client, (u8 *)reg, 2);
	if (err == 2)
		err = 0;
	else {
		msleep(100); /* For deep sleep mode */
		err = i2c_master_send(client, (u8 *)reg, 2);
		if (err == 2)
			err = 0;
		else
			err = -EIO;
	}

	if (err < 0)
		return err;

	err = i2c_master_recv(client, (u8 *)value, length);
	if (err == length)
		err = length;
	else
		err = -EIO;
}
#endif
/* FIH } */
	return err;
}


/* Writes one byte to given address in mXT chip. */

static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{

	struct mxt_data *mxt;
	struct mxt_i2c_byte_transfer i2c_byte_transfer;
	int err;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;

	if (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
		err = 0;
	else
		err = -EIO;

	return err;
}

/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client,
		u16 addr, u16 length, u8 *value)
{
	int i;
	struct {
		__le16	le_addr;
		u8	data[256];
	} i2c_block_transfer;
	struct mxt_data *mxt;

	dev_dbg(&client->dev, "Writing %d bytes to %d...", length, addr);
	if (length > 256)
		return -EINVAL;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;

	memcpy(i2c_block_transfer.data, value, length);
	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2))
		return length;
	else
		return -EIO;
}

/* Calculates the CRC value for mXT infoblock. */
int calculate_infoblock_crc(u32 *crc_result, u8 *data, int crc_area_size)
{
	u32 crc = 0;
	int i;

	pr_debug("atmel_mxt224 In function %s", __func__);
	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = CRC_24(crc, *(data + i), *(data + i + 1));
	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = CRC_24(crc, *(data + i), 0);
	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);

	return 0;
}

static void report_mt(struct mxt_data *mxt)
{
	int i;
	int active_touches = 0;

	for (i = 0; i < mxt->numtouch; i++) {
		if (!stored_size[i])
			continue;

		active_touches++;
		input_report_abs(mxt->touch_input,
				ABS_MT_TRACKING_ID,
				i);
		input_report_abs(mxt->touch_input,
				ABS_MT_TOUCH_MAJOR,
				stored_size[i]);
		input_report_abs(mxt->touch_input,
				ABS_MT_POSITION_X,
				stored_x[i]);
		input_report_abs(mxt->touch_input,
				ABS_MT_POSITION_Y,
				stored_y[i]);
		input_mt_sync(mxt->touch_input);
	}

	if (active_touches == 0)
		input_mt_sync(mxt->touch_input);
	input_sync(mxt->touch_input);
}

static void process_T9_message(u8 *message, struct mxt_data *mxt)
{
	struct	input_dev *input;
	struct device *dev = &mxt->client->dev;
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  touch_size = 255;
	u8  touch_number;
	u8  amplitude;
	u8  report_id;
	input = mxt->touch_input;
	status = message[MXT_MSG_T9_STATUS];
	report_id = message[0];

	if (status & MXT_MSGB_T9_SUPPRESS) {
		/* Touch has been suppressed by grip/face */
		/* detection                              */
		dev_dbg(&mxt->client->dev, "SUPRESS");
	} else {
/* FIH { */
/* Correct coordinate */
#if 0
		xpos = message[MXT_MSG_T9_XPOSMSB] * 16 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
		ypos = message[MXT_MSG_T9_YPOSMSB] * 16 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 0) & 0xF);
		printk(KERN_INFO "[TOUCH] X=%d, Y=%d\n", xpos, ypos);

		if (mxt->max_x_val < 1024)
				xpos = xpos >> 2;

		if (mxt->max_y_val < 1024)
				ypos = ypos >> 2;

		printk(KERN_INFO "[TOUCH] [X=%d, Y=%d] max_x=%d , max_y=%d\n", xpos, ypos, mxt->max_x_val, mxt->max_y_val);
#else
		xpos = message[MXT_MSG_T9_XPOSMSB] * 16 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
		ypos = message[MXT_MSG_T9_YPOSMSB] * 4 +
			((message[MXT_MSG_T9_XYPOSLSB] >> 6) & 0x3);
		printk(KERN_INFO "[TOUCH] X=%d, Y=%d\n", xpos, ypos);

#if 0
		xpos = xpos*1024*9/12800 + 51;
		ypos = ypos*1024*9/8000 + 51;
		printk(KERN_INFO "[TOUCH] [X=%d, Y=%d]\n", xpos, ypos);
#else
		xpos = (xpos)*1024/1280;
		if (xpos > 1024)
			xpos = 1024;
		if (xpos < 0)
			xpos = 0;
		ypos = (ypos)*1024/800;
		if (ypos > 1024)
			ypos = 1024;
		if (ypos < 0)
			ypos = 0;
		printk(KERN_INFO "[TOUCH] [X=%d, Y=%d]\n", xpos, ypos);
#endif

#endif




/* FIH } */

		touch_number = message[MXT_MSG_REPORTID] -
			mxt->rid_map[report_id].first_rid;

		stored_x[touch_number] = xpos;
		stored_y[touch_number] = ypos;

		if (status & MXT_MSGB_T9_DETECT) {
			/*
			 * TODO: more precise touch size calculation?
			 * mXT224 reports the number of touched nodes,
			 * so the exact value for touch ellipse major
			 * axis length would be 2*sqrt(touch_size/pi)
			 * (assuming round touch shape).
			 */
			touch_size = message[MXT_MSG_T9_TCHAREA];
			touch_size = touch_size >> 2;
			if (!touch_size)
				touch_size = 1;
			stored_size[touch_number] = touch_size;
			if (status & MXT_MSGB_T9_AMP)
				/* Amplitude of touch has changed */
				amplitude = message[MXT_MSG_T9_TCHAMPLITUDE];

			printk(KERN_INFO, "DETECT:%s%s%s%s",
				((status & MXT_MSGB_T9_PRESS) ? " PRESS" : ""),
				((status & MXT_MSGB_T9_MOVE) ? " MOVE" : ""),
				((status & MXT_MSGB_T9_AMP) ? " AMP" : ""),
				((status & MXT_MSGB_T9_VECTOR) ? " VECT" : ""));
		} else if (status & MXT_MSGB_T9_RELEASE) {
			printk(KERN_INFO, "RELEASE");

			/* The previously reported touch has been removed.*/
			stored_size[touch_number] = 0;
		}

		printk(KERN_INFO, "X=%d, Y=%d, touch number=%d, TOUCHSIZE=%d",
			xpos, ypos, touch_number, stored_size[touch_number]);
	}
}

void process_key_message(u8 *message, struct mxt_data *mxt)
{
	/*key up*/
	if (message[1] == 0 && message[2] == 0 && message[3] == 0 && message[4] == 0) {
		if (mxt->prev_key == 0) {
			dev_dbg(&mxt->client->dev, "No previous key");
		} else {
			input_report_key(mxt->key_input, mxt->prev_key, 0);
			dev_dbg(&mxt->client->dev,
				 "Report key %d up", mxt->prev_key);
			mxt->prev_key = 0;
		}
		input_sync(mxt->key_input);
		return;
	}

printk(KERN_INFO "message[1]=%d, message[2]=%d, message[3]=%d, message[4]=%d", message[1], message[2], message[3], message[4]);

	/*key down*/
	if (message[1] == 128 && message[2] == 0 && message[3] == 2 && message[4] == 0) {
		input_report_key(mxt->key_input, KEY_BACK, 1);
		dev_dbg(&mxt->client->dev,
			"Report BACK(%d) key DOWN.", KEY_BACK);
		mxt->prev_key = KEY_BACK;
	} else if (message[1] == 128 && message[2] == 0 && message[3] == 4 && message[4] == 0) {
		input_report_key(mxt->key_input, KEY_MENU, 1);
		dev_dbg(&mxt->client->dev,
			 "Report MENU(%d) key DOWN.", KEY_MENU);
		mxt->prev_key = KEY_MENU;
	} else if (message[1] == 128 && message[2] == 0 && message[3] == 8 && message[4] == 0) {
		input_report_key(mxt->key_input, KEY_HOME, 1);
		dev_dbg(&mxt->client->dev,
			"Report HOME(%d) key DOWN.", KEY_HOME);
		mxt->prev_key = KEY_HOME;
	} else if (message[1] == 128 && message[2] == 8 && message[3] == 0 && message[4] == 0) {
		input_report_key(mxt->key_input, KEY_SEARCH, 1);
		dev_dbg(&mxt->client->dev,
			"Report SEARCH(%d) key DOWN.", KEY_SEARCH);
		mxt->prev_key = KEY_SEARCH;
	}
	input_sync(mxt->key_input);
}

static int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{
	struct i2c_client *client;
	u8  status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8  event;
	u8  direction;
	u16 distance;
	u8  length;
	u8  report_id;
	static u8 error_cond;
	client = mxt->client;
	length = mxt->message_size;
	report_id = message[0];

	switch (object) {
	case MXT_GEN_COMMANDPROCESSOR_T6:
		status = message[1];

		if (status & MXT_MSGB_T6_COMSERR) {
			if ((!error_cond) & MXT_MSGB_T6_COMSERR) {
				dev_err(&client->dev, "maXTouch checksum error\n");
				error_cond |= MXT_MSGB_T6_COMSERR;
			}
		}
		if (status & MXT_MSGB_T6_CFGERR) {
			/*
			 * Configuration error. A proper configuration
			 * needs to be written to chip and backed up. Refer
			 * to protocol document for further info.
			 */
			if ((!error_cond) & MXT_MSGB_T6_CFGERR) {
				dev_err(&client->dev, "maXTouch configuration error\n");
				error_cond |= MXT_MSGB_T6_CFGERR;
			}
		}
		if (status & MXT_MSGB_T6_CAL) {
			/* Calibration in action, no need to react */
			dev_info(&client->dev, "maXTouch calibration in progress\n");
		}
		if (status & MXT_MSGB_T6_SIGERR) {
			/*
			 * Signal acquisition error, something is seriously
			 * wrong, not much we can in the driver to correct
			 * this
			 */
			if ((!error_cond) & MXT_MSGB_T6_SIGERR) {
				dev_err(&client->dev, "maXTouch acquisition error\n");
				error_cond |= MXT_MSGB_T6_SIGERR;
			}
		}
		if (status & MXT_MSGB_T6_OFL) {
			/*
			 * Cycle overflow, the acquisition is too short.
			 * Can happen temporarily when there's a complex
			 * touch shape on the screen requiring lots of
			 * processing.
			 */
			dev_err(&client->dev, "maXTouch cycle overflow\n");
		}
		if (status & MXT_MSGB_T6_RESET) {
			/* Chip has reseted, no need to react. */
			dev_info(&client->dev, "maXTouch chip reset\n");
		}
		if (status == 0) {
			/* Chip status back to normal. */
			dev_info(&client->dev, "maXTouch status normal\n");
			error_cond = 0;
		}
		break;

	case MXT_TOUCH_KEYARRAY_T15:
		dev_dbg(&client->dev, "key value, message[1]=%d, "
			"message[2]=%d, message[3]=%d, message[4]=%d",
			 message[1], message[2], message[3], message[4]);
		process_key_message(message, mxt);
		break;

	case MXT_TOUCH_MULTITOUCHSCREEN_T9:
		process_T9_message(message, mxt);
		report_mt(mxt);
		break;

	case MXT_SPT_GPIOPWM_T19:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving GPIO message\n");
		break;

	case MXT_PROCI_GRIPFACESUPPRESSION_T20:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving face suppression msg\n");
		break;

	case MXT_PROCG_NOISESUPPRESSION_T22:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving noise suppression msg\n");
		status = message[MXT_MSG_T22_STATUS];
		if (status & MXT_MSGB_T22_FHCHG) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "maXTouch: Freq changed\n");
		}
		if (status & MXT_MSGB_T22_GCAFERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev, "maXTouch: High noise level\n");
		}
		if (status & MXT_MSGB_T22_FHERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					"maXTouch: Freq changed - Noise level too high\n");
		}
		break;

	case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving one-touch gesture msg\n");

		event = message[MXT_MSG_T24_STATUS] & 0x0F;
		xpos = message[MXT_MSG_T24_XPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T24_YPOSMSB] * 16 +
			((message[MXT_MSG_T24_XYPOSLSB] >> 0) & 0x0F);
		xpos >>= 2;
		ypos >>= 2;
		direction = message[MXT_MSG_T24_DIR];
		distance = message[MXT_MSG_T24_DIST] +
			   (message[MXT_MSG_T24_DIST + 1] << 16);

		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(event << 24) | (direction << 16) | distance);
		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(xpos << 16) | ypos);
		break;

	case MXT_SPT_SELFTEST_T25:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				"Receiving Self-Test msg\n");

		if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_RUN_ALL_TESTS) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					"maXTouch: Self-Test OK\n");

		} else  {
			dev_err(&client->dev,
				"maXTouch: Self-Test Failed [%02x]:"
				"{%02x,%02x,%02x,%02x,%02x}\n",
				message[MXT_MSG_T25_STATUS],
				message[MXT_MSG_T25_STATUS + 0],
				message[MXT_MSG_T25_STATUS + 1],
				message[MXT_MSG_T25_STATUS + 2],
				message[MXT_MSG_T25_STATUS + 3],
				message[MXT_MSG_T25_STATUS + 4]
				);
		}
		break;

	case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving 2-touch gesture message\n");

		event = message[MXT_MSG_T27_STATUS] & 0xF0;
		xpos = message[MXT_MSG_T27_XPOSMSB] * 16 +
			((message[MXT_MSG_T27_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T27_YPOSMSB] * 16 +
			((message[MXT_MSG_T27_XYPOSLSB] >> 0) & 0x0F);
		xpos >>= 2;
		ypos >>= 2;
		direction = message[MXT_MSG_T27_ANGLE];
		distance = message[MXT_MSG_T27_SEPARATION] +
			   (message[MXT_MSG_T27_SEPARATION + 1] << 16);

		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(event << 24) | (direction << 16) | distance);
		input_event(mxt->touch_input, EV_MSC, MSC_GESTURE,
				(xpos << 16) | ypos);
		break;
	case MXT_SPT_CTECONFIG_T28:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving CTE message...\n");
		status = message[MXT_MSG_T28_STATUS];
		if (status & MXT_MSGB_T28_CHKERR)
			dev_err(&client->dev,  "maXTouch: Power-Up CRC failure\n");
		break;
	default:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "maXTouch: Unknown message!\n");
		break;
	}

	return 0;
}

/*
 * Processes messages when the interrupt line (CHG) is asserted. Keeps
 * reading messages until a message with report ID 0xFF is received,
 * which indicates that there is no more new messages.
 *
 */
static void mxt_worker(struct mxt_data *mxt)
{
	struct i2c_client *client;
	u8	*message;
	u16	message_length;
	u16	message_addr;
	u8	report_id;
	u8	object;
	int	error;
	int	i;
	int     intr_value;

	client         = mxt->client;
	message        = mxt->message_buf;
	message_addr   = mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length >= 256) {
		dev_err(&client->dev, "Message length > 256 bytes not supported\n");
		return;
	}

	dev_dbg(&client->dev, "maXTouch worker active:\n");
	do {
		/* Read next message, reread on failure. */
		mxt->message_counter++;
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			error = mxt_read_block(client, message_addr,
					message_length - 1, message);
			if (error >= 0)
				break;
			mxt->read_fail_counter++;
			dev_err(&client->dev, "Failure reading maxTouch device\n");
		}
		if (error < 0)
			return;

		report_id = message[0];

		if (reg_add > 0)
			dev_info(
			&client->dev,
			"Atmel Debug info message[0]=%d,message[1]=%d,message[2]=%d,message[3]=%d"
			"message[4]=%d,message[5]=%d,message[6]=%d,message[7]=%d,message_length=%d",
			message[0], message[1], message[2], message[3],
			message[4], message[5], message[6], message[7], message_length);

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}
		/*dev_dbg(&client->dev, "chgline: %d\n", mxt->read_chg());*/
		intr_value = gpio_get_value(mxt->mxt_intr_gpio);
	} while (intr_value == 0);
}

/*
 * The maXTouch device will signal the host about a new message by asserting
 * the CHG line. This ISR schedules a worker routine to read the message when
 * that happens.
 */
static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = _mxt;

	mxt->irq_counter++;
	if (mxt->valid_interrupt()) {
		mxt->valid_irq_counter++;

		mutex_lock(&mxt->dev_mutex);
		mxt_worker(mxt);
		mutex_unlock(&mxt->dev_mutex);
	} else {
		mxt->invalid_irq_counter++;
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

static int __devinit mxt_identify(struct i2c_client *client,
				  struct mxt_data *mxt,
				  u8 *id_block_data)
{
	u8 buf[7];
	int error;
	int identified;

	identified = 0;

	/* Read Device info to check if chip is valid */
	error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,
			       (u8 *) buf);

	if (error < 0) {
		client->addr = MXT1386_I2C_ADDR2;
		slave_addr = MXT1386_I2C_ADDR2;
		printk(KERN_INFO "[TOUCH] switch slave address to 0x%02x", client->addr);
		error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE, (u8 *) buf);
	}

	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Failure accessing maXTouch device\n");
		return -EIO;
	}

	memcpy(id_block_data, buf, MXT_ID_BLOCK_SIZE);

	mxt->device_info.family_id  = buf[0];
	mxt->device_info.variant_id = buf[1];
	mxt->device_info.major	    = ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor      = (buf[2] & 0x0F);
	mxt->device_info.build	    = buf[3];
	mxt->device_info.x_size	    = buf[4];
	mxt->device_info.y_size	    = buf[5];
	mxt->device_info.num_objs   = buf[6];
	mxt->device_info.num_nodes  = mxt->device_info.x_size *
				      mxt->device_info.y_size;

	/*
	 * Check Family & Variant Info; warn if not recognized but
	 * still continue.
	 */

	/* MXT224 */
	if (mxt->device_info.family_id == MXT224_FAMILYID) {
		strcpy(mxt->device_info.family_name, "atmel_mxt224");

		if (mxt->device_info.variant_id == MXT224_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else if (mxt->device_info.variant_id ==
			MXT224_UNCAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Uncalibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}

	/* MXT1386 */
	} else if (mxt->device_info.family_id == MXT1386_FAMILYID) {
/* FIH { */
#if 0
		strcpy(mxt->device_info.family_name, "mXT1386");
#else
		strcpy(mxt->device_info.family_name, "mxt1386");
#endif
/* FIH } */

		if (mxt->device_info.variant_id == MXT1386_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n",
				mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
	/* Unknown family ID! */
	} else {
		dev_err(&client->dev,
			"Warning: maXTouch Family ID [%d] not supported\n",
			mxt->device_info.family_id);
		strcpy(mxt->device_info.family_name, "UNKNOWN");
		strcpy(mxt->device_info.variant_name, "UNKNOWN");
		/* identified = -ENXIO; */
	}

	dev_info(
		&client->dev,
		"Atmel maXTouch (Family %s (%X), Variant %s (%X)) Firmware "
		"version [%d.%d] Build %d\n",
		mxt->device_info.family_name,
		mxt->device_info.family_id,
		mxt->device_info.variant_name,
		mxt->device_info.variant_id,
		mxt->device_info.major,
		mxt->device_info.minor,
		mxt->device_info.build
	);
	dev_info(
		&client->dev,
		"Atmel maXTouch Configuration "
		"[X: %d] x [Y: %d]\n",
		mxt->device_info.x_size,
		mxt->device_info.y_size
	);
	return identified;
}

/*
 * Reads the object table from maXTouch chip to get object data like
 * address, size, report id. For Info Block CRC calculation, already read
 * id data is passed to this function too (Info Block consists of the ID
 * block and object table).
 *
 */
static int __devinit mxt_read_object_table(struct i2c_client *client,
					   struct mxt_data *mxt,
					   u8 *raw_id_data)
{
	u16	report_id_count;
	u8	buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8      *raw_ib_data;
	u8	object_type;
	u16	object_address;
	u16	object_size;
	u8	object_instances;
	u8	object_report_ids;
	u16	object_info_address;
	u32	crc;
	u32     calculated_crc;
	int	i;
	int	error;

	u8	object_instance;
	u8	object_report_id;
	u8	report_id;
	int     first_report_id;
	int     ib_pointer;
	struct mxt_object *object_table;
	dev_dbg(&mxt->client->dev, "maXTouch driver reading configuration\n");

	object_table = kzalloc(sizeof(struct mxt_object) *
			       mxt->device_info.num_objs,
			       GFP_KERNEL);
	if (object_table == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_object_table_alloc;
	}

	raw_ib_data = kmalloc(MXT_OBJECT_TABLE_ELEMENT_SIZE *
			mxt->device_info.num_objs + MXT_ID_BLOCK_SIZE,
			GFP_KERNEL);
	if (raw_ib_data == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_ib_alloc;
	}

	/* Copy the ID data for CRC calculation. */
	memcpy(raw_ib_data, raw_id_data, MXT_ID_BLOCK_SIZE);
	ib_pointer = MXT_ID_BLOCK_SIZE;

	mxt->object_table = object_table;

	dev_dbg(&mxt->client->dev, "maXTouch driver Memory allocated\n");

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		dev_dbg(&mxt->client->dev, "Reading maXTouch at [0x%04x]: ",
			  object_info_address);

		error = mxt_read_block(client, object_info_address,
				       MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"maXTouch Object %d could not be read\n", i);
			error = -EIO;
			goto err_object_read;
		}

		memcpy(raw_ib_data + ib_pointer, buf,
		       MXT_OBJECT_TABLE_ELEMENT_SIZE);
		ib_pointer += MXT_OBJECT_TABLE_ELEMENT_SIZE;

		object_type       =  buf[0];
		object_address    = (buf[2] << 8) + buf[1];
		object_size       =  buf[3] + 1;
		object_instances  =  buf[4] + 1;
		object_report_ids =  buf[5];
		dev_dbg(&mxt->client->dev, "Type=%03d, Address=0x%04x, "
			  "Size=0x%02x, %d instances, %d report id's\n",
			  object_type,
			  object_address,
			  object_size,
			  object_instances,
			  object_report_ids
		);

		/* TODO: check whether object is known and supported? */

		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
		}

		object_table[i].type            = object_type;
		object_table[i].chip_addr       = object_address;
		object_table[i].size            = object_size;
		object_table[i].instances       = object_instances;
		object_table[i].num_report_ids  = object_report_ids;
		report_id_count += object_instances * object_report_ids;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	mxt->rid_map =
		kzalloc(sizeof(struct report_id_map) * (report_id_count + 1),
			/* allocate for report_id 0, even if not used */
			GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_rid_map_alloc;
	}

	mxt->message_buf = kmalloc(256, GFP_KERNEL);
	if (mxt->message_buf == NULL) {
		printk(KERN_WARNING "Error allocating memory\n");
		error = -ENOMEM;
		goto err_msg_alloc3;
	}

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
			dev_err(&client->dev,
				"Too many maXTouch report id's [%d]\n",
				report_id_count);
			error = -ENXIO;
			goto err_max_rid;
	}

	/* Create a mapping from report id to object type */
	report_id = 1; /* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
				object_instance < object_table[i].instances;
				object_instance++) {
			first_report_id = report_id;
			for (object_report_id = 0;
			     object_report_id < object_table[i].num_report_ids;
			     object_report_id++) {
				mxt->rid_map[report_id].object =
					object_table[i].type;
				mxt->rid_map[report_id].instance =
					object_instance;
				mxt->rid_map[report_id].first_rid =
					first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, 3, buf);
	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	if (calculate_infoblock_crc(&calculated_crc, raw_ib_data,
				    ib_pointer)) {
		printk(KERN_WARNING "Error while calculating CRC!\n");
		calculated_crc = 0;
	}
	kfree(raw_ib_data);

	dev_dbg(&mxt->client->dev, "\nReported info block CRC = 0x%6X\n", crc);
	dev_dbg(&mxt->client->dev, "Calculated info block CRC = 0x%6X\n\n",
		       calculated_crc);

	if (crc == calculated_crc) {
		mxt->info_block_crc = crc;
	} else {
		mxt->info_block_crc = 0;
		pr_err("maXTouch: Info block CRC invalid!\n");
	}

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "maXTouch: %d Objects\n",
				mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "Type:\t\t\t[%d]: %s\n",
				 object_table[i].type,
				 obj_typ_name[object_table[i].type]);
			dev_info(&client->dev, "\tAddress:\t0x%04X\n",
				object_table[i].chip_addr);
			dev_info(&client->dev, "\tSize:\t\t%d Bytes\n",
				object_table[i].size);
			dev_info(&client->dev, "\tInstances:\t%d\n",
				object_table[i].instances);
			dev_info(&client->dev, "\tReport Id's:\t%d\n",
				object_table[i].num_report_ids);
		}
	}

	return 0;
err_msg_alloc3:
	kfree(mxt->message_buf);
err_max_rid:
	kfree(mxt->rid_map);
err_rid_map_alloc:
err_object_read:
	kfree(raw_ib_data);
err_ib_alloc:
	kfree(object_table);
err_object_table_alloc:
	return error;
}


static u8 mxt_valid_interrupt_dummy(void)
{
	return 1;
}

static int __devinit mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_data          *mxt;
	struct mxt_platform_data *pdata;
	struct input_dev         *touch_input;
	struct input_dev         *key_input;
	u8 *id_data;
	int error, gpio_intr, i;
	u16 base_addr;

	pr_info("atmel_mxt224: mxt_probe\n");

	if (client == NULL) {
		pr_debug("maXTouch: client == NULL\n");
		return	-EINVAL;
	} else if (client->adapter == NULL) {
		pr_debug("maXTouch: client->adapter == NULL\n");
		return	-EINVAL;
	} else if (&client->dev == NULL) {
		pr_debug("maXTouch: client->dev == NULL\n");
		return	-EINVAL;
	} else if (&client->adapter->dev == NULL) {
		pr_debug("maXTouch: client->adapter->dev == NULL\n");
		return	-EINVAL;
	} else if (id == NULL) {
		pr_debug("maXTouch: id == NULL\n");
		return	-EINVAL;
	}

	dev_dbg(&client->dev, "maXTouch driver v. %s\n", DRIVER_VERSION);
	dev_dbg(&client->dev, "\t \"%s\"\n", client->name);
	dev_dbg(&client->dev, "\taddr:\t0x%04x\n", client->addr);
	dev_dbg(&client->dev, "\tirq:\t%d\n", client->irq);
	dev_dbg(&client->dev, "\tflags:\t0x%04x\n", client->flags);
	dev_dbg(&client->dev, "\tadapter:\"%s\"\n", client->adapter->name);
	dev_dbg(&client->dev, "\tdevice:\t\"%s\"\n", client->dev.init_name);

	/* Check if the I2C bus supports BYTE transfer */
	error = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	if (!error) {
		dev_err(&client->dev, "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

	dev_dbg(&client->dev, "maXTouch driver functionality OK\n");

	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}

	id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
	if (id_data == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_id_alloc;
	}

	touch_input = input_allocate_device();
	if (!touch_input) {
		dev_err(&client->dev, "error allocating touch input device\n");
		error = -ENOMEM;
		goto err_touch_input_dev_alloc;
	}

	key_input = input_allocate_device();
	if (!key_input) {
		dev_err(&client->dev, "error allocating key input device");
		error = -ENOMEM;
		goto err_key_input_dev_alloc;
	}

	/* Initialize Platform data */
	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "Platform OK: pdata = 0x%08x\n",
		       (unsigned int) pdata);

	mutex_init(&mxt->dev_mutex);
	mxt->read_fail_counter = 0;
	mxt->message_counter   = 0;
	mxt->max_x_val         = pdata->max_x;
	mxt->max_y_val         = pdata->max_y;
	mxt->orientation       = pdata->orientation;
	mxt->mxt_intr_gpio     = pdata->irq;
	mxt->mxt_reset_gpio    = pdata->reset;

	error = gpio_request(mxt->mxt_intr_gpio, 0);
	if (error < 0)
		pr_err("Failed to request GPIO%d (MaxTouch-interrupt) error=%d\n",
			mxt->mxt_intr_gpio, error);

	error = gpio_direction_input(mxt->mxt_intr_gpio);
	if (error) {
		pr_err("Failed to set interrupt direction, error=%d\n", error);
		gpio_free(mxt->mxt_intr_gpio);
	}

	error = gpio_request(mxt->mxt_reset_gpio, "MaxTouch-reset");
	if (error < 0)
		pr_err("Failed to request GPIO%d (MaxTouch-reset) error=%d\n",
			mxt->mxt_reset_gpio, error);

	error = gpio_direction_output(mxt->mxt_reset_gpio, 1);
	if (error) {
		pr_info("Failed to set reset direction, error=%d\n", error);
		gpio_free(mxt->mxt_reset_gpio);
	}

	/* maXTouch wants 40mSec minimum after reset to get organized */
	gpio_set_value(mxt->mxt_reset_gpio, 1);
	msleep(40);
/* FIH { */
/* Wait for interrupt and delay for unstable reset status */
#if 1
{
	int i = 0;

	for (i = 0; i < 10; i++) {
		printk(KERN_INFO "Polling GPIO%d (MaxTouch-intr). retry=%d\n", mxt->mxt_intr_gpio, i);
		if (gpio_get_value(mxt->mxt_intr_gpio) == LOW)
			break;
		msleep(50);
	}

	/* msleep(10000); */
}
#endif
/* FIH } */

	/* Get data that is defined in board specific code. */
	mxt->init_hw = pdata->init_platform_hw;
	mxt->exit_hw = pdata->exit_platform_hw;
	mxt->read_chg = pdata->read_chg;

	if (pdata->valid_interrupt != NULL)
		mxt->valid_interrupt = pdata->valid_interrupt;
	else
		mxt->valid_interrupt = mxt_valid_interrupt_dummy;

	if (mxt->init_hw != NULL)
		mxt->init_hw();

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver identifying chip\n");

	if (mxt_identify(client, mxt, id_data) < 0) {
		dev_err(&client->dev, "Chip could not be identified\n");
		error = -ENODEV;
		goto err_identify;
	}
	/* Chip is valid and active. */
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver allocating input device\n");

	mxt->client      = client;
	mxt->touch_input = touch_input;
	mxt->key_input   = key_input;

	snprintf(mxt->touch_phys_name, sizeof(mxt->touch_phys_name),
		 "%s/input0", dev_name(&client->dev));
	snprintf(mxt->key_phys_name, sizeof(mxt->key_phys_name),
		 "%s/input1", dev_name(&client->dev));

	/* Touch input parameter */
	touch_input->name       = "mxt224_touchscreen_0";
	touch_input->phys       = mxt->touch_phys_name;
	touch_input->id.bustype = BUS_I2C;
	touch_input->dev.parent = &client->dev;

	/* Key input parameter */
	key_input->name         = "mxt224_key_0";
	key_input->phys         = mxt->key_phys_name;

	/* Multitouch */
	input_set_abs_params(touch_input, ABS_MT_POSITION_X, TS_MIN_X, TS_MAX_X, 0, 0);
	input_set_abs_params(touch_input, ABS_MT_POSITION_Y, TS_MIN_Y, TS_MAX_Y, 0, 0);
	input_set_abs_params(touch_input, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_TOUCH_SIZE,
			     0, 0);
	input_set_abs_params(touch_input, ABS_MT_TRACKING_ID, 0,
			     MXT_MAX_NUM_TOUCHES, 0, 0);

	__set_bit(EV_ABS, touch_input->evbit);
	__set_bit(EV_SYN, touch_input->evbit);

	/* Function key*/
	__set_bit(EV_KEY,     key_input->evbit);
	__set_bit(KEY_HOME,   key_input->keybit);
	__set_bit(KEY_MENU,   key_input->keybit);
	__set_bit(KEY_BACK,   key_input->keybit);
	__set_bit(KEY_SEARCH, key_input->keybit);

	i2c_set_clientdata(client, mxt);
	input_set_drvdata(touch_input, mxt);
	input_set_drvdata(key_input, mxt);

	error = input_register_device(mxt->touch_input);
	if (error < 0) {
		dev_err(&client->dev, "Failed to register touch input device\n");
		goto err_register_touch_device;
	}

	error = input_register_device(mxt->key_input);
	if (error < 0) {
		dev_err(&client->dev, "Failed to register key input device in mxt224\n");
		goto err_register_key_device;
	}

	mxt->message_buf = NULL;
	error = mxt_read_object_table(client, mxt, id_data);
	if (error < 0)
		goto err_read_ot;

	BackupNVM(mxt);
	msleep(40);

	mxt->prev_key = 0;

	if (pdata->numtouch)
		mxt->numtouch = pdata->numtouch;
	mxt->irq                 = gpio_to_irq(mxt->mxt_intr_gpio);
	mxt->valid_irq_counter   = 0;
	mxt->invalid_irq_counter = 0;
	mxt->irq_counter         = 0;

	if (mxt->irq) {
		error = request_threaded_irq(mxt->irq, NULL,
				    mxt_irq_handler,
				    IRQF_TRIGGER_FALLING,
				    client->dev.driver->name,
				    mxt);
		if (error < 0) {
			dev_err(&client->dev,
				"failed to allocate irq %d\n", mxt->irq);
			goto err_irq;
		}
	}

	mutex_lock(&mxt->dev_mutex);
	gpio_intr = gpio_get_value(mxt->mxt_intr_gpio);
	while (gpio_intr == 0) {
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			error = mxt_read_block(mxt->client, mxt->msg_proc_addr,
					mxt->message_size, mxt->message_buf);
			if (error >= 0)
				break;
			dev_info(&client->dev, "flush the buffer failed in the probe function\n");

		}
		gpio_intr = gpio_get_value(mxt->mxt_intr_gpio);

	}
	mutex_unlock(&mxt->dev_mutex);

	/* Store the number of multi-touches allowed, default was 0. */
	base_addr = get_object_address(MXT_TOUCH_MULTITOUCHSCREEN_T9,
			0, mxt->object_table, mxt->device_info.num_objs);
	base_addr += MXT_ADR_T9_NUMTOUCH;

	error = mxt_write_byte(mxt->client, base_addr, mxt->numtouch);
	if (error < 0) {
		dev_err(&client->dev,
			"Multi-touch init failure, T9 object, error = %d\n",
			error);
		goto err_irq;
	}

	kfree(id_data);

#ifdef CONFIG_DEBUG_FS
	error = intel_mid_add_debugfs();
	if (error < 0)
		printk(KERN_ERR "failed to initialize  intel_mid debugfs\n");

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	mxt->suspended  = false;
	mxt->T7[0]      = 32;
	mxt->T7[1]      = 15;
	mxt->T7[2]      = 50;
	mxt->es.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	mxt->es.suspend = mxt_early_suspend;
	mxt->es.resume  = mxt_late_resume;
	register_early_suspend(&mxt->es);
	mxt_es          = mxt;
#endif

	return 0;

err_irq:
	kfree(mxt->rid_map);
	kfree(mxt->object_table);
err_read_ot:
err_register_key_device:
err_register_touch_device:
err_identify:
err_pdata:
	input_free_device(key_input);
err_key_input_dev_alloc:
	input_free_device(touch_input);
err_touch_input_dev_alloc:
	kfree(id_data);
err_id_alloc:
	if (mxt->exit_hw != NULL)
		mxt->exit_hw();
	kfree(mxt);
err_mxt_alloc:
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if (mxt->exit_hw != NULL)
			mxt->exit_hw();

		if (mxt->irq)
			free_irq(mxt->irq, mxt);
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&mxt->es);
#endif
		input_unregister_device(mxt->key_input);
		input_unregister_device(mxt->touch_input);
		kfree(mxt->rid_map);
		kfree(mxt->object_table);
		kfree(mxt->message_buf);
	}
	kfree(mxt);

	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "Touchscreen unregistered\n");

	return 0;
}

#if defined(CONFIG_PM)
static int mxt_suspend(struct device *dev)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);

	dev_dbg(dev, "In function %s", __func__);

	if (device_may_wakeup(dev))
		enable_irq_wake(mxt->irq);

	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct mxt_data *mxt = dev_get_drvdata(dev);

	dev_dbg(dev, "In function %s", __func__);

	if (device_may_wakeup(dev))
		disable_irq_wake(mxt->irq);

	return 0;
}
#else
#define mxt_suspend NULL
#define mxt_resume NULL
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void mxt_early_suspend(struct early_suspend *h)
{
	u16 pwcfg_addr;
	u8 buf[3] = {0, 0, 0};
	u8 err;

	disable_irq(mxt_es->irq);

	mutex_lock(&mxt_es->dev_mutex);


	pwcfg_addr = get_object_address(MXT_GEN_POWERCONFIG_T7,
				0, mxt_es->object_table,
				mxt_es->device_info.num_objs);
	mxt_read_block(mxt_es->client, pwcfg_addr, 3, mxt_es->T7);
	err = mxt_write_block(mxt_es->client, pwcfg_addr, 3, buf);
	if (err > 0)
		dev_info(&mxt_es->client->dev, "Driver enter deep sleep mode.");
	else
		dev_info(&mxt_es->client->dev,
			 "Driver can't enter deep sleep mode [%d].", err);
	/* clear touch state when suspending */
	memset(stored_size, 0, mxt_es->numtouch * sizeof(stored_size[0]));
	report_mt(mxt_es);

	mxt_es->suspended = true;

	mutex_unlock(&mxt_es->dev_mutex);
}

void mxt_late_resume(struct early_suspend *h)
{
	int gpio_intr;
	u16 pwcfg_addr;
	u8 err;
	u8 i;

	enable_irq(mxt_es->irq);
	mutex_lock(&mxt_es->dev_mutex);

	pwcfg_addr = get_object_address(MXT_GEN_POWERCONFIG_T7,
			0,
			mxt_es->object_table,
			mxt_es->device_info.num_objs);
/* FIH { */
#if 0
	err = mxt_write_block(mxt_es->client, pwcfg_addr, 3, mxt_es->T7);
#else
	for (i = 0; i < I2C_RETRY_COUNT; i++) {
		err = mxt_write_block(mxt_es->client, pwcfg_addr, 3, mxt_es->T7);
		if (err == 3)
			break;
		msleep(100);
	}
#endif
/* FIH } */
	if (err > 0)
		dev_info(&mxt_es->client->dev, "resume from early suspend");
	else
		dev_info(&mxt_es->client->dev, "fail to late resume");

	msleep(25);
	gpio_intr = gpio_get_value(mxt_es->mxt_intr_gpio);
	while (gpio_intr == 0) {
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			err = mxt_read_block(mxt_es->client,
						mxt_es->msg_proc_addr,
						mxt_es->message_size,
						mxt_es->message_buf);
			if (err >= 0)
				break;
			dev_info(&mxt_es->client->dev, "flush the buffer failed in the resume function\n");

		}
		gpio_intr = gpio_get_value(mxt_es->mxt_intr_gpio);

	}
	mxt_calibrate(mxt_es->client);
	mxt_es->suspended = false;
	mutex_unlock(&mxt_es->dev_mutex);

}
#endif

static const struct dev_pm_ops mxt_pm_ops = {
	.suspend = mxt_suspend,
	.resume  = mxt_resume,
};

static const struct i2c_device_id mxt_idtable[] = {
	{TOUCH_DEVICE_NAME, 1,},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= TOUCH_DEVICE_NAME,
		.owner  = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &mxt_pm_ops,
#endif
	},

	.id_table	= mxt_idtable,
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
};

static int __init mxt_init(void)
{
	int err;

	err = i2c_add_driver(&mxt_driver);
	if (err) {
		printk(KERN_WARNING "Adding maXTouch driver(Here for mxt224) failed "
		       "(errno = %d)\n", err);
	} else {
		printk(KERN_INFO "Successfully added driver %s\n",
			  mxt_driver.driver.name);
	}
	return err;
}

static void __exit mxt_cleanup(void)
{
	i2c_del_driver(&mxt_driver);
}


late_initcall(mxt_init);
module_exit(mxt_cleanup);

MODULE_AUTHOR("Iiro Valkonen");
MODULE_DESCRIPTION("Driver for Atmel maXTouch Touchscreen Controller");
MODULE_LICENSE("GPL");


#ifdef CONFIG_DEBUG_FS
/******************************************************************************
debugfs for touch driver, contain four debug files:

1.sread
	can be read and write.
	write: set the address of a single register to be accessed;
	read: return the content of the register.
2.swrite
	can be write only.
	write: set the value of the corresponding register, and the register's
	address is determined by 'write' operation of 'single_read' file.
3.sread_offset
	can be write only
	write:set the offset of the register
*********************************************************************************/


static int intel_mid_single_read_get(void *data, u64 *val)
{
	int ret, gpio_intr, gpio_rst;
	u16 addr;
	addr = get_object_address(reg_add,
			0,
			mxt_es->object_table,
			mxt_es->device_info.num_objs)+reg_add_offset;

	ret = mxt_read_block(mxt_es->client, addr, 1, (u8 *)val);
	if (ret < 0)
		printk(KERN_ERR "intel_touch debugfs read_set err add=0x%x\n", reg_add);
	gpio_intr = gpio_get_value(mxt_es->mxt_intr_gpio);
	gpio_rst = gpio_get_value(mxt_es->mxt_reset_gpio);
	printk("touch interrupt value = %d,gpio_rst = %d\n", gpio_intr, gpio_rst);
	return 0;
}

static int intel_mid_single_read_set(void *data, u64 val)
{
	reg_add = (u8)val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(intel_mid_single_read_reg_fops,
				intel_mid_single_read_get,
				intel_mid_single_read_set, "0x%02llx\n");

static int intel_mid_single_read_set_offset(void *data, u64 val)
{
	reg_add_offset = (u8)val;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(intel_mid_single_read_reg_offset_fops,
				NULL, intel_mid_single_read_set_offset,
				"0x%02llx\n");

static int intel_mid_single_write_set(void *data, u64 val)
{
	int ret;
	u16 addr;
	uint8_t data1[1] = {0};
	data1[0] = (u8)val;
	addr = get_object_address(reg_add,
			0,
			mxt_es->object_table,
			mxt_es->device_info.num_objs)+reg_add_offset;

	ret = mxt_write_block(mxt_es->client, addr, 1, data1);

	if (ret < 0)
		printk(KERN_ERR "intel_touch debugfs write err add=0x%x\n", reg_add);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(intel_mid_single_write_reg_fops,
				NULL, intel_mid_single_write_set, "0x%02llx\n");


static int  intel_mid_add_debugfs(void)
{
	struct dentry *root;

	root = debugfs_create_dir("touch_debug", NULL);

	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return -1;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		* create the directory. */
		goto err;

	if (!debugfs_create_file("sread", S_IWUGO|S_IRUGO, root, NULL, &intel_mid_single_read_reg_fops))
		goto err;

	if (!debugfs_create_file ("sread_offset", S_IWUGO|S_IRUGO, root, NULL, &intel_mid_single_read_reg_offset_fops))
		goto err;

	if (!debugfs_create_file ("swrite", S_IWUGO, root, NULL, &intel_mid_single_write_reg_fops))
		goto err;

	return 0;
err:
	debugfs_remove_recursive(root);
	printk(KERN_ERR "failed to initialize  intel_mid debugfs\n");
	return -1;
}

#endif
