/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2012 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include "atmel_mxt_ts.h"
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/earlysuspend.h>
#include <linux/string.h>
#include <linux/miscdevice.h>

/* Configuration file */
#define MXT_CFG_MAGIC                "OBP_RAW V1"
#define MXT_FW_T2			  "atmel/mXT1664T2C2U__APP_v1_1_AA_Production.fw"
#define MXT_CFG_T2_JT		   "atmel/TF103_11AA_V27_JTouch_052014.raw"
#define MXT_CFG_T2_JT_F		"atmel/TF103_11AA_V27_JTouch_Final_052014.raw"
#define MXT_CFG_T2_OF	   "atmel/TF103_11AA_V27_OFilm_052014.raw"
#define MXT_CFG_T2_REV_D_JT    "atmel/TF103_revD_11AA_V31_JTouch_20141126.raw"
#define MXT_CFG_T2_REV_D_OF   "atmel/TF103_revD_11AA_V31_OFilm_20141126.raw"
#define MXT_CFG_T2_REV_D_GIS   "atmel/TF103_revD_11AA_V31_GIS_20141216.raw"
#define MXT_FW_OLD				"atmel/mxt_1664t2.fw"
#define MXT_TP_ID_TEST		0

/* Registers */
#define MXT_OBJECT_START        0x07
#define MXT_OBJECT_SIZE                6
#define MXT_INFO_CHECKSUM_SIZE        3
#define MXT_MAX_BLOCK_WRITE        256

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37        37
#define MXT_GEN_MESSAGE_T5                5
#define MXT_GEN_COMMAND_T6                6
#define MXT_GEN_POWER_T7                7
#define MXT_GEN_ACQUIRE_T8                8
#define MXT_GEN_DATASOURCE_T53                53
#define MXT_TOUCH_MULTI_T9                9
#define MXT_TOUCH_KEYARRAY_T15                15
#define MXT_TOUCH_PROXIMITY_T23                23
#define MXT_TOUCH_PROXKEY_T52                52
#define MXT_PROCI_GRIPFACE_T20                20
#define MXT_PROCG_NOISE_T22                22
#define MXT_PROCI_ONETOUCH_T24                24
#define MXT_PROCI_TWOTOUCH_T27                27
#define MXT_PROCI_GRIP_T40                40
#define MXT_PROCI_PALM_T41                41
#define MXT_PROCI_TOUCHSUPPRESSION_T42        42
#define MXT_PROCI_STYLUS_T47                47
#define MXT_PROCG_NOISESUPPRESSION_T48        48
#define MXT_SPT_COMMSCONFIG_T18                18
#define MXT_SPT_GPIOPWM_T19                19
#define MXT_SPT_SELFTEST_T25                25
#define MXT_SPT_CTECONFIG_T28                28
#define MXT_SPT_USERDATA_T37                37
#define MXT_SPT_USERDATA_T38                38
#define MXT_SPT_DIGITIZER_T43                43
#define MXT_SPT_MESSAGECOUNT_T44        44
#define MXT_SPT_CTECONFIG_T46                46
#define MXT_PROCI_ACTIVE_STYLUS_T63        63
#define MXT_PROCI_GLOVE_DETECTION_T78        78 //add for glove
#define MXT_TOUCH_MULTITOUCHSCREEN_T100 100

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG                0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET        0
#define MXT_COMMAND_BACKUPNV        1
#define MXT_COMMAND_CALIBRATE        2
#define MXT_COMMAND_REPORTALL        3
#define MXT_COMMAND_DIAGNOSTIC        5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET        (1 << 7)
#define MXT_T6_STATUS_OFL        (1 << 6)
#define MXT_T6_STATUS_SIGERR        (1 << 5)
#define MXT_T6_STATUS_CAL        (1 << 4)
#define MXT_T6_STATUS_CFGERR        (1 << 3)
#define MXT_T6_STATUS_COMSERR        (1 << 2)

/* MXT_GEN_POWER_T7 field */
struct t7_config
{
    u8 idle;
    u8 active;
} __packed;

#define MXT_POWER_CFG_RUN                0
#define MXT_POWER_CFG_DEEPSLEEP                1
//add for TP_SHOW
#define MXT_TP_SHOW			0x07

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT                9
#define MXT_T9_RANGE                18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP                (1 << 0)
#define MXT_T9_SUPPRESS                (1 << 1)
#define MXT_T9_AMP                (1 << 2)
#define MXT_T9_VECTOR                (1 << 3)
#define MXT_T9_MOVE                (1 << 4)
#define MXT_T9_RELEASE                (1 << 5)
#define MXT_T9_PRESS                (1 << 6)
#define MXT_T9_DETECT                (1 << 7)

struct t9_range
{
    u16 x;
    u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH        (1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL                0
#define MXT_COMMS_CMD                1
#define MXT_COMMS_RETRIGEN      (1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE                0xa5
#define MXT_RESET_VALUE                0x01
#define MXT_BACKUP_VALUE        0x55

/* MXT_SPT_SELFTEST_T25 status */
#define MXT_T25_ENABLE		(1 << 0)
#define MXT_T25_RPTEN		(1 << 1)

/* Define for MXT_SPT_SELFTEST_T25 */
#define MXT_T25_ALL_TEST_PASS	0xfe
#define MXT_T25_CMD_FAULT	0xfd
#define MXT_T25_UNRELATED_FAULT		0xfc
#define MXT_T25_AVDD_FAULT	0x01
#define MXT_T25_PIN_FAULT	0x12
#define MXT_T25_SIGNAL_LIMIT_FAULT	0x17

/* MXT_SPT_SELFTEST_T25 self test limit enumerate*/
enum
{
    test_cmd,
    high_limit,
    low_limit,
    diff_limit
};

/*Define for MXT_DEBUG_DIAGNOSTIC_T37*/
#define MXT_T37_IC_REV_C    0x02
#define MXT_T37_IC_REV_D    0x03

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP        (1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS        1

/* T63 Stylus */
#define MXT_T63_STYLUS_PRESS        (1 << 0)
#define MXT_T63_STYLUS_RELEASE        (1 << 1)
#define MXT_T63_STYLUS_MOVE                (1 << 2)
#define MXT_T63_STYLUS_SUPPRESS        (1 << 3)

#define MXT_T63_STYLUS_DETECT        (1 << 4)
#define MXT_T63_STYLUS_TIP                (1 << 5)
#define MXT_T63_STYLUS_ERASER        (1 << 6)
#define MXT_T63_STYLUS_BARREL        (1 << 7)

#define MXT_T63_STYLUS_PRESSURE_MASK        0x3F

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL                0
#define MXT_T100_CFG1                1
#define MXT_T100_TCHAUX                3
#define MXT_T100_XRANGE                13
#define MXT_T100_YRANGE                24

#define MXT_T100_CFG_SWITCHXY        (1 << 5)

#define MXT_T100_TCHAUX_VECT        (1 << 0)
#define MXT_T100_TCHAUX_AMPL        (1 << 1)
#define MXT_T100_TCHAUX_AREA        (1 << 2)

#define MXT_T100_DETECT                (1 << 7)
#define MXT_T100_TYPE_MASK        0x70
#define MXT_T100_TYPE_STYLUS        0x20

/* Delay times */
#define MXT_BACKUP_TIME                50        /* msec */
#define MXT_RESET_TIME                200        /* msec */
#define MXT_RESET_TIMEOUT        3000        /* msec */
#define MXT_CRC_TIMEOUT                1000        /* msec */
#define MXT_FW_RESET_TIME        3000        /* msec */
#define MXT_FW_CHG_TIMEOUT        300        /* msec */
#define MXT_WAKEUP_TIME                25        /* msec */
#define MXT_REGULATOR_DELAY        150        /* msec */
#define MXT_POWERON_DELAY        150        /* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB        0xaa
#define MXT_UNLOCK_CMD_LSB        0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD        0xc0        /* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA        0x80        /* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK        0x02
#define MXT_FRAME_CRC_FAIL        0x03
#define MXT_FRAME_CRC_PASS        0x04
#define MXT_APP_CRC_FAIL        0x40        /* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK        0x3f
#define MXT_BOOT_EXTENDED_ID        (1 << 5)
#define MXT_BOOT_ID_MASK        0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA                0xff

#define MXT_PIXELS_PER_MM        20

#define DEBUG_MSG_MAX                200

/*Define for i2c stress test*/
#define MXT_IOC_MAGIC 0xF3
#define MXT_IOC_MAXNR 2
#define MXT_POLL_DATA _IOR(MXT_IOC_MAGIC,2,int)

#define MXT_IOCTL_START_HEAVY 2
#define MXT_IOCTL_START_NORMAL 1
#define MXT_IOCTL_END 0

#define START_NORMAL    msecs_to_jiffies(2000)  // 0.5 HZ;
#define START_HEAVY     msecs_to_jiffies(1000)  // 1 HZ;

#define TP_JTOUCH	1
#define TP_OFILM	2
#define TP_GIS       3

//add for factory test
//add by red_zhang@asus.com
u8 ref_byte2;
u8 ref_byte3;
u8 ref_byte4;
//add config version
int config_version;
bool reboot;
//add for force update config
bool cfg_force;
//avoid too more log when touch update firmware
bool print_log;

/*Add for i2c stress test*/
static int poll_mode=0;
struct delayed_work mxt_poll_data_work;
static struct workqueue_struct *touch_work_queue;
//struct i2c_client *mxt_client;

static struct mxt_data *data = NULL;

struct mxt_info
{
    u8 family_id;
    u8 variant_id;
    u8 version;
    u8 build;
    u8 matrix_xsize;
    u8 matrix_ysize;
    u8 object_num;
};

struct mxt_object
{
    u8 type;
    u16 start_address;
    u8 size_minus_one;
    u8 instances_minus_one;
    u8 num_report_ids;
} __packed;

/* Each client has this additional data */
struct mxt_data
{
    struct i2c_client *client;
    struct input_dev *input_dev;
    char phys[64];                /* device physical location */
    struct mxt_platform_data *pdata;
    struct mxt_object *object_table;
    struct mxt_info *info;
    void *raw_info_block;
    unsigned int irq;
    unsigned int max_x;
    unsigned int max_y;
    bool in_bootloader;
    u16 mem_size;
    u8 t100_aux_ampl;
    u8 t100_aux_area;
    u8 t100_aux_vect;
    struct bin_attribute mem_access_attr;
    bool debug_enabled;
    bool debug_v2_enabled;
    u8 *debug_msg_data;
    u16 debug_msg_count;
    struct bin_attribute debug_msg_attr;
    struct mutex debug_msg_lock;
    u8 max_reportid;
    u32 config_crc;
    u32 info_crc;
    u8 bootloader_addr;
    struct t7_config t7_cfg;
    u8 *msg_buf;
    u8 t6_status;
    bool update_input;
    u8 last_message_count;
    u8 num_touchids;
    u8 num_stylusids;
    unsigned long t15_keystatus;
    bool use_retrigen_workaround;
    bool use_regulator;
    struct regulator *reg_vdd;
    struct regulator *reg_avdd;
    char *fw_name;
    char *cfg_name;
    bool mb_test;		//mb_test = true if mb > er2.add by red_zhang@asus.com.
    u8 pin_id;
    struct early_suspend early_suspend; //add by red_zhang@asus.com
    bool irq_status;
    short irq_count;
    u8 t25_self_test_status;
    u8 t25_ref_byte2;
    u8 t25_ref_byte3;
    u8 t25_ref_byte4;
    int cfg_version;
    int tp_id;
    int cfg_tp_id;

    /* Cached parameters from object table */
    u16 T5_address;
    u8 T5_msg_size;
    u8 T6_reportid;
    u16 T6_address;
    u16 T7_address;
    u8 T9_reportid_min;
    u8 T9_reportid_max;
    u8 T15_reportid_min;
    u8 T15_reportid_max;
    u16 T18_address;
    u16 T19_address;	//add for TP_SHOW
    u8 T19_reportid;
    u8 gpio_status;             //add for TP_SHOW
    bool get_gpio_status;       //add for TP_SHOW
    u8 ic_revision;
    u16 T25_address;        //add by Eric
    u8 T25_reportid;        //add by Eric
    u16 T37_address;
    u16 T38_address;		//add by Eric 4/15
    u8 T42_reportid_min;
    u8 T42_reportid_max;
    u16 T44_address;
    u8 T48_reportid;
    u8 T63_reportid_min;
    u8 T63_reportid_max;
    u16 T78_address; 	//add for glove
    u8 T100_reportid_min;
    u8 T100_reportid_max;

    /* for fw update in bootloader */
    struct completion bl_completion;

    /* for reset handling */
    struct completion reset_completion;

    /* for reset handling */
    struct completion crc_completion;

    /* Enable reporting of input events */
    bool enable_reporting;

    /* Indicates whether device is in suspend */
    bool suspended;

    /*i2c stress test*/
    struct miscdevice misc_dev;

};

static int mxt_initialize_t9_input_device(struct mxt_data *data);
static int mxt_configure_objects(struct mxt_data *data);
static int mxt_configure_objects_show(struct mxt_data *data);
static int mxt_tp_id_show(struct device *dev);	//add by red_zhang@asus.com
static int red_mxt_cfg_version(struct mxt_data *data);	//add by red_zhang@asus.com
static int mxt_early_suspend(struct early_suspend *h);
static int mxt_late_resume(struct early_suspend *h);
static int red_mxt_initialize(struct mxt_data *data);
static int red_mxt_configure_objects(struct mxt_data *data);
static int mxt_check_reg_init(struct mxt_data *data);

static inline size_t mxt_obj_size(const struct mxt_object *obj)
{
    return obj->size_minus_one + 1;
}

static inline size_t mxt_obj_instances(const struct mxt_object *obj)
{
    return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
    switch (type)
    {
    case MXT_GEN_COMMAND_T6:
    case MXT_GEN_POWER_T7:
    case MXT_GEN_ACQUIRE_T8:
    case MXT_GEN_DATASOURCE_T53:
    case MXT_TOUCH_MULTI_T9:
    case MXT_TOUCH_KEYARRAY_T15:
    case MXT_TOUCH_PROXIMITY_T23:
    case MXT_TOUCH_PROXKEY_T52:
    case MXT_PROCI_GRIPFACE_T20:
    case MXT_PROCG_NOISE_T22:
    case MXT_PROCI_ONETOUCH_T24:
    case MXT_PROCI_TWOTOUCH_T27:
    case MXT_PROCI_GRIP_T40:
    case MXT_PROCI_PALM_T41:
    case MXT_PROCI_TOUCHSUPPRESSION_T42:
    case MXT_PROCI_STYLUS_T47:
    case MXT_PROCG_NOISESUPPRESSION_T48:
    case MXT_SPT_COMMSCONFIG_T18:
    case MXT_SPT_GPIOPWM_T19:
    case MXT_SPT_SELFTEST_T25:
    case MXT_SPT_CTECONFIG_T28:
    case MXT_SPT_USERDATA_T37:
    case MXT_SPT_USERDATA_T38:
    case MXT_SPT_DIGITIZER_T43:
    case MXT_SPT_CTECONFIG_T46:
        return true;
    default:
        return false;
    }
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
    print_hex_dump(KERN_DEBUG, "touch: [mxt_dump_message] MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
                   message, data->T5_msg_size, false);
}

static void mxt_debug_msg_enable(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;

    if (data->debug_v2_enabled)
        return;

    mutex_lock(&data->debug_msg_lock);

    data->debug_msg_data = kcalloc(DEBUG_MSG_MAX,
                                   data->T5_msg_size, GFP_KERNEL);
    if (!data->debug_msg_data)
    {
        TOUCH_ERR("Failed to allocate buffer\n");
        return;
    }

    data->debug_v2_enabled = true;
    mutex_unlock(&data->debug_msg_lock);

    TOUCH_NOTICE("Enabled message output\n");
}

static void mxt_debug_msg_disable(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;

    if (!data->debug_v2_enabled)
        return;

    TOUCH_NOTICE("disabling message output\n");
    data->debug_v2_enabled = false;

    mutex_lock(&data->debug_msg_lock);
    kfree(data->debug_msg_data);
    data->debug_msg_data = NULL;
    data->debug_msg_count = 0;
    mutex_unlock(&data->debug_msg_lock);
    TOUCH_NOTICE("Disabled message output\n");
}

static void mxt_debug_msg_add(struct mxt_data *data, u8 *msg)
{
    struct device *dev = &data->client->dev;

    mutex_lock(&data->debug_msg_lock);

    if (!data->debug_msg_data)
    {
        TOUCH_ERR("No buffer!\n");
        return;
    }

    if (data->debug_msg_count < DEBUG_MSG_MAX)
    {
        memcpy(data->debug_msg_data + data->debug_msg_count * data->T5_msg_size,
               msg,
               data->T5_msg_size);
        data->debug_msg_count++;
    }
    else
    {
        TOUCH_DEBUG("Discarding %u messages\n", data->debug_msg_count);
        data->debug_msg_count = 0;
    }

    mutex_unlock(&data->debug_msg_lock);

    sysfs_notify(&data->client->dev.kobj, NULL, "debug_notify");
}

static ssize_t mxt_debug_msg_write(struct file *filp, struct kobject *kobj,
                                   struct bin_attribute *bin_attr, char *buf, loff_t off,
                                   size_t count)
{
    return -EIO;
}

static ssize_t mxt_debug_msg_read(struct file *filp, struct kobject *kobj,
                                  struct bin_attribute *bin_attr, char *buf, loff_t off, size_t bytes)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mxt_data *data = dev_get_drvdata(dev);
    int count;
    size_t bytes_read;

    if (!data->debug_msg_data)
    {
        TOUCH_ERR("No buffer!\n");
        return 0;
    }

    count = bytes / data->T5_msg_size;

    if (count > DEBUG_MSG_MAX)
        count = DEBUG_MSG_MAX;

    mutex_lock(&data->debug_msg_lock);

    if (count > data->debug_msg_count)
        count = data->debug_msg_count;

    bytes_read = count * data->T5_msg_size;

    memcpy(buf, data->debug_msg_data, bytes_read);
    data->debug_msg_count = 0;

    mutex_unlock(&data->debug_msg_lock);

    return bytes_read;
}

static int mxt_debug_msg_init(struct mxt_data *data)
{
    sysfs_bin_attr_init(&data->debug_msg_attr);
    data->debug_msg_attr.attr.name = "debug_msg";
    data->debug_msg_attr.attr.mode = 0664;
    data->debug_msg_attr.read = mxt_debug_msg_read;
    data->debug_msg_attr.write = mxt_debug_msg_write;
    data->debug_msg_attr.size = data->T5_msg_size * DEBUG_MSG_MAX;

    if (sysfs_create_bin_file(&data->client->dev.kobj,
                              &data->debug_msg_attr) < 0)
    {
        TOUCH_ERR("Failed to create %s\n",
                data->debug_msg_attr.attr.name);
        return -EINVAL;
    }

    return 0;
}

static void mxt_debug_msg_remove(struct mxt_data *data)
{
    if (data->debug_msg_attr.attr.name)
        sysfs_remove_bin_file(&data->client->dev.kobj,
                              &data->debug_msg_attr);
}

static int mxt_wait_for_completion(struct mxt_data *data,
                                   struct completion *comp, unsigned int timeout_ms)
{
    struct device *dev = &data->client->dev;
    unsigned long timeout = msecs_to_jiffies(timeout_ms);
    long ret;

    ret = wait_for_completion_interruptible_timeout(comp, timeout);
    if (ret < 0)
    {
        TOUCH_ERR("Wait for completion interrupted.\n");
        return -EINTR;
    }
    else if (ret == 0)
    {
        TOUCH_ERR("Wait for completion timed out.\n");
        return -ETIMEDOUT;
    }
    return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
                               u8 *val, unsigned int count)
{
    int ret;
    struct i2c_msg msg;

    msg.addr = data->bootloader_addr;
    msg.flags = data->client->flags & I2C_M_TEN;
    msg.flags |= I2C_M_RD;
    msg.len = count;
    msg.buf = val;

    if(print_log)
    {
        TOUCH_NOTICE("bootloader_addr=0x%02X\n", data->bootloader_addr);
    }

    ret = i2c_transfer(data->client->adapter, &msg, 1);

    if (ret == 1)
    {
        ret = 0;
    }
    else
    {
        ret = (ret < 0) ? ret : -EIO;
        TOUCH_ERR("%s: i2c recv failed (%d)\n",
                __func__, ret);
    }

    return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
                                const u8 * const val, unsigned int count)
{
    int ret;
    struct i2c_msg msg;

    msg.addr = data->bootloader_addr;
    msg.flags = data->client->flags & I2C_M_TEN;
    msg.len = count;
    msg.buf = (u8 *)val;

    ret = i2c_transfer(data->client->adapter, &msg, 1);
    if (ret == 1)
    {
        ret = 0;
    }
    else
    {
        ret = (ret < 0) ? ret : -EIO;
        TOUCH_ERR("%s: i2c send failed (%d)\n",
                __func__, ret);
    }

    return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, bool retry)
{
    u8 appmode = data->client->addr;
    u8 bootloader;
    u8 family_id = 0;

    if (data->info)
        family_id = data->info->family_id;

    TOUCH_NOTICE("appmode=0x%02X\n", appmode);
    TOUCH_NOTICE("family_id=0x%02X\n", family_id);

    switch (appmode)
    {
    case 0x4a:
    case 0x4b:
        /* Chips after 1664S use different scheme */
        if (retry || family_id >= 0xa2)
        {
            bootloader = appmode - 0x24;
            break;
        }
    /* Fall through for normal case */
    case 0x4c:
    case 0x4d:
    case 0x5a:
    case 0x5b:
        bootloader = appmode - 0x26;
        break;
    default:
        TOUCH_ERR("Appmode i2c address 0x%02x not found\n", appmode);
        return -EINVAL;
    }

    data->bootloader_addr = bootloader;
    TOUCH_NOTICE("bootloader_addr=0x%02X\n", data->bootloader_addr);
    return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, bool retry)
{
    struct device *dev = &data->client->dev;
    int ret;
    u8 val;
    bool crc_failure;

    ret = mxt_lookup_bootloader_address(data, retry);
    if (ret)
        return ret;

    ret = mxt_bootloader_read(data, &val, 1);
    if (ret)
        return ret;

    /* Check app crc fail mode */
    crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

    TOUCH_ERR("Detected bootloader, status:%02X%s\n",
            val, crc_failure ? ", APP_CRC_FAIL" : "");

    return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
    struct device *dev = &data->client->dev;
    u8 buf[3];

    if (val & MXT_BOOT_EXTENDED_ID)
    {
        if (mxt_bootloader_read(data, &buf[0], 3) != 0)
        {
            TOUCH_ERR("i2c failure\n");
            return -EIO;
        }

        TOUCH_NOTICE("Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

        return buf[0];
    }
    else
    {
        TOUCH_NOTICE("Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

        return val;
    }
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state,
                                bool wait)
{
    struct device *dev = &data->client->dev;
    u8 val;
    int ret;

recheck:
    if (wait)
    {
        /*
         * In application update mode, the interrupt
         * line signals state transitions. We must wait for the
         * CHG assertion before reading the status byte.
         * Once the status byte has been read, the line is deasserted.
         */
        ret = mxt_wait_for_completion(data, &data->bl_completion,
                                      MXT_FW_CHG_TIMEOUT);
        if (ret)
        {
            /*
             * TODO: handle -EINTR better by terminating fw update
             * process before returning to userspace by writing
             * length 0x000 to device (iff we are in
             * WAITING_FRAME_DATA state).
             */
            TOUCH_ERR("Update wait error %d\n", ret);
            return ret;
        }
    }

    ret = mxt_bootloader_read(data, &val, 1);
    if (ret)
        return ret;

    if (state == MXT_WAITING_BOOTLOAD_CMD)
        val = mxt_get_bootloader_version(data, val);

    switch (state)
    {
    case MXT_WAITING_BOOTLOAD_CMD:
    case MXT_WAITING_FRAME_DATA:
    case MXT_APP_CRC_FAIL:
        val &= ~MXT_BOOT_STATUS_MASK;
        break;
    case MXT_FRAME_CRC_PASS:
        if (val == MXT_FRAME_CRC_CHECK)
        {
            goto recheck;
        }
        else if (val == MXT_FRAME_CRC_FAIL)
        {
            TOUCH_ERR("Bootloader CRC fail\n");
            return -EINVAL;
        }
        break;
    default:
        return -EINVAL;
    }

    if (val != state)
    {
        TOUCH_ERR("Invalid bootloader state %02X != %02X\n",
                val, state);
        return -EINVAL;
    }

    return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
    int ret;
    u8 buf[2];

    if (unlock)
    {
        buf[0] = MXT_UNLOCK_CMD_LSB;
        buf[1] = MXT_UNLOCK_CMD_MSB;
    }
    else
    {
        buf[0] = 0x01;
        buf[1] = 0x01;
    }

    ret = mxt_bootloader_write(data, buf, 2);
    if (ret)
        return ret;

    return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
                          u16 reg, u16 len, void *val)
{
    struct i2c_msg xfer[2];
    u8 buf[2];
    int ret;
    bool retry = false;

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;

    /* Write register */
    xfer[0].addr = client->addr;
    xfer[0].flags = 0;
    xfer[0].len = 2;
    xfer[0].buf = buf;

    /* Read data */
    xfer[1].addr = client->addr;
    xfer[1].flags = I2C_M_RD;
    xfer[1].len = len;
    xfer[1].buf = val;

retry_read:
    ret = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
    if (ret != ARRAY_SIZE(xfer))
    {
        if (!retry)
        {
            TOUCH_DEBUG("i2c retry\n");
            msleep(MXT_WAKEUP_TIME);
            retry = true;
            goto retry_read;
        }
        else
        {
            TOUCH_ERR("%s: i2c transfer failed (%d)\n",
                    __func__, ret);
            return -EIO;
        }
    }

    return 0;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
                           const void *val)
{
    u8 *buf;
    size_t count;
    int ret;
    bool retry = false;

    count = len + 2;
    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;

    buf[0] = reg & 0xff;
    buf[1] = (reg >> 8) & 0xff;
    memcpy(&buf[2], val, len);

retry_write:
    ret = i2c_master_send(client, buf, count);
    if (ret != count)
    {
        if (!retry)
        {
            TOUCH_DEBUG("i2c retry\n");
            msleep(MXT_WAKEUP_TIME);
            retry = true;
            goto retry_write;
        }
        else
        {
            TOUCH_ERR("%s: i2c send failed (%d)\n",
                    __func__, ret);
            ret = -EIO;
        }
    }
    else
    {
        ret = 0;
    }

    kfree(buf);
    return ret;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
    return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
    struct mxt_object *object;
    int i;

    for (i = 0; i < data->info->object_num; i++)
    {
        object = data->object_table + i;
        if (object->type == type)
            return object;
    }

    TOUCH_WARN("Invalid object type T%u\n", type);
    return NULL;
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
    struct device *dev = &data->client->dev;
    u8 status = msg[1];
    u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);

//    TOUCH_NOTICE("mxt_proc_t6_messages, crc=0x%06X \n", crc);

//		dev_info(dev,"T6 command answer.\n");
    if (crc != data->config_crc)
    {
        data->config_crc = crc;
        TOUCH_DEBUG("T6 Config Checksum: 0x%06X\n", crc);
        complete(&data->crc_completion);
    }

    /* Detect transition out of reset */
    if ((data->t6_status & MXT_T6_STATUS_RESET) &&
            !(status & MXT_T6_STATUS_RESET))
        complete(&data->reset_completion);

    /* Output debug if status has changed */
    if (status != data->t6_status)
        TOUCH_DEBUG("T6 Status 0x%02X%s%s%s%s%s%s%s\n",
                status,
                (status == 0) ? " OK" : "",
                (status & MXT_T6_STATUS_RESET) ? " RESET" : "",
                (status & MXT_T6_STATUS_OFL) ? " OFL" : "",
                (status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
                (status & MXT_T6_STATUS_CAL) ? " CAL" : "",
                (status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
                (status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");

    /* Save current status */
    data->t6_status = status;
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
//        struct input_dev *input = data->input_dev;
//        const struct mxt_platform_data *pdata = data->pdata;
//        bool button;
//        int i;

    /* do not report events if input device not yet registered */
//        if (!data->enable_reporting)
//                return;

    //add for TP_SHOW
    data->gpio_status = message[1];
    data->get_gpio_status = true;

    /* Active-low switch */
    /*
    for (i = 0; i < pdata->t19_num_keys; i++) {
            if (pdata->t19_keymap[i] == KEY_RESERVED)
                    continue;
            button = !(message[1] & (1 << i));
            input_report_key(input, pdata->t19_keymap[i], button);
    }	*/
}

static void mxt_input_sync(struct input_dev *input_dev)
{
    input_mt_report_pointer_emulation(input_dev, false);
    input_sync(input_dev);
}

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
    struct device *dev = &data->client->dev;
    struct input_dev *input_dev = data->input_dev;
    int id;
    u8 status;
    int x;
    int y;
    int area;
    int amplitude;
    u8 vector;
    int tool;

    /* do not report events if input device not yet registered */
    if (!data->enable_reporting)
        return;

    id = message[0] - data->T9_reportid_min;
    status = message[1];
    x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
    y = (message[3] << 4) | ((message[4] & 0xf));

    /* Handle 10/12 bit switching */
    if (data->max_x < 1024)
        x >>= 2;
    if (data->max_y < 1024)
        y >>= 2;

    area = message[5];

    amplitude = message[6];
    vector = message[7];

    TOUCH_DEBUG(
            "[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
            id,
            (status & MXT_T9_DETECT) ? 'D' : '.',
            (status & MXT_T9_PRESS) ? 'P' : '.',
            (status & MXT_T9_RELEASE) ? 'R' : '.',
            (status & MXT_T9_MOVE) ? 'M' : '.',
            (status & MXT_T9_VECTOR) ? 'V' : '.',
            (status & MXT_T9_AMP) ? 'A' : '.',
            (status & MXT_T9_SUPPRESS) ? 'S' : '.',
            (status & MXT_T9_UNGRIP) ? 'U' : '.',
            x, y, area, amplitude, vector);

    input_mt_slot(input_dev, id);

    if (status & MXT_T9_DETECT)
    {
        /* Multiple bits may be set if the host is slow to read the
         * status messages, indicating all the events that have
         * happened */
        if (status & MXT_T9_RELEASE)
        {
            input_mt_report_slot_state(input_dev,
                                       MT_TOOL_FINGER, 0);
            mxt_input_sync(input_dev);
        }

        /* A reported size of zero indicates that the reported touch
         * is a stylus from a linked Stylus T47 object. */
        if (area == 0)
        {
            area = MXT_TOUCH_MAJOR_T47_STYLUS;
            tool = MT_TOOL_PEN;
        }
        else
        {
            tool = MT_TOOL_FINGER;
        }

        /* Touch active */
        input_mt_report_slot_state(input_dev, tool, 1);
        input_report_abs(input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
        input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
        input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
        input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
    }
    else
    {
        /* Touch no longer active, close out slot */
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
    }

    data->update_input = true;
}

static void mxt_proc_t25_messages(struct mxt_data *data, u8 *msg)
{
    struct device *dev = &data->client->dev;
    u8 status = msg[1];

    /* do not report events if input device not yet registered */
    if (!data->enable_reporting)
        return;

    /*MXT_T25_PIN_FAULT sequence numbers
      ref_bytes2 mean:
        0x01 All pins are driven low to gnd
        0x02 All pins are pulled high
        0x03 the high walks along the pins
        0x04 the low walks along the pins
        0x05 send 1010 and 0101 pattern fault
        0x07 Initial high voltage pin fault
      ref_bytes3 mean:
        0 means no pin fail and 1 to 255 is X line number
      ref_bytes4 mean:
        0 means no pin fail and 1 to 255 is Y line number*/
    if(status == MXT_T25_PIN_FAULT)
    {
        data->t25_ref_byte2 = msg[2];
        data->t25_ref_byte3 = msg[3];
        data->t25_ref_byte4 = msg[4];
    }

    if(status == MXT_T25_SIGNAL_LIMIT_FAULT)
    {
        data->t25_ref_byte2 = msg[2];
        data->t25_ref_byte3 = msg[3];
    }

    TOUCH_DEBUG("T25 Status 0x%02X\n", status);

    /* Save current status */
    data->t25_self_test_status = status;
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
    struct device *dev = &data->client->dev;
    struct input_dev *input_dev = data->input_dev;
    int id;
    u8 status;
    int x;
    int y;
    int tool;

    /* do not report events if input device not yet registered */
    if (!data->enable_reporting)
        return;

    id = message[0] - data->T100_reportid_min - 2;

    /* ignore SCRSTATUS events */
    if (id < 0)
        return;

    status = message[1];
    x = (message[3] << 8) | message[2];
    y = (message[5] << 8) | message[4];
    /*
    TOUCH_DEBUG(
            "[%u] status:%02X x:%u y:%u area:%02X amp:%02X vec:%02X\n",
            id,
            status,
            x, y,
            (data->t100_aux_area) ? message[data->t100_aux_area] : 0,
            (data->t100_aux_ampl) ? message[data->t100_aux_ampl] : 0,
            (data->t100_aux_vect) ? message[data->t100_aux_vect] : 0);
    */
    input_mt_slot(input_dev, id);

    if (status & MXT_T100_DETECT)
    {
        /* A reported size of zero indicates that the reported touch
         * is a stylus from a linked Stylus T47 object. */
        if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)
            tool = MT_TOOL_PEN;
        else
            tool = MT_TOOL_FINGER;

//				printk("Toby test > tool = %d\n",tool);

        /* Touch active */
        input_mt_report_slot_state(input_dev, tool, 1);
        input_report_abs(input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

        if (data->t100_aux_ampl)
            input_report_abs(input_dev, ABS_MT_PRESSURE,
                             message[data->t100_aux_ampl]);

        if (data->t100_aux_area)
        {
            if (tool == MT_TOOL_PEN)
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
                                 MXT_TOUCH_MAJOR_T47_STYLUS);
            else
                input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
                                 message[data->t100_aux_area]);
        }

        if (data->t100_aux_vect)
            input_report_abs(input_dev, ABS_MT_ORIENTATION,
                             message[data->t100_aux_vect]);
    }
    else
    {
        /* Touch no longer active, close out slot */
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
    }

    data->update_input = true;
}


static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
    struct input_dev *input_dev = data->input_dev;
    struct device *dev = &data->client->dev;
    int key;
    bool curr_state, new_state;
    bool sync = false;
    unsigned long keystates = le32_to_cpu(msg[2]);

    /* do not report events if input device not yet registered */
    if (!data->enable_reporting)
        return;

    for (key = 0; key < data->pdata->t15_num_keys; key++)
    {
        curr_state = test_bit(key, &data->t15_keystatus);
        new_state = test_bit(key, &keystates);

        if (!curr_state && new_state)
        {
            TOUCH_DEBUG("T15 key press: %u\n", key);
            __set_bit(key, &data->t15_keystatus);
            input_event(input_dev, EV_KEY,
                        data->pdata->t15_keymap[key], 1);
            sync = true;
        }
        else if (curr_state && !new_state)
        {
            TOUCH_DEBUG("T15 key release: %u\n", key);
            __clear_bit(key, &data->t15_keystatus);
            input_event(input_dev, EV_KEY,
                        data->pdata->t15_keymap[key], 0);
            sync = true;
        }
    }

    if (sync)
        input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
    struct device *dev = &data->client->dev;
    u8 status = msg[1];

    if (status & MXT_T42_MSG_TCHSUP)
        TOUCH_NOTICE("T42 suppress\n");
    else
        TOUCH_NOTICE("T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
    struct device *dev = &data->client->dev;
    u8 status, state;

    status = msg[1];
    state  = msg[4];

    TOUCH_DEBUG("T48 state %d status %02X %s%s%s%s%s\n",
            state,
            status,
            (status & 0x01) ? "FREQCHG " : "",
            (status & 0x02) ? "APXCHG " : "",
            (status & 0x04) ? "ALGOERR " : "",
            (status & 0x10) ? "STATCHG " : "",
            (status & 0x20) ? "NLVLCHG " : "");

    return 0;
}

static void mxt_proc_t63_messages(struct mxt_data *data, u8 *msg)
{
    struct device *dev = &data->client->dev;
    struct input_dev *input_dev = data->input_dev;
    u8 id;
    u16 x, y;
    u8 pressure;

    /* do not report events if input device not yet registered */
    if (!data->enable_reporting)
        return;

    /* stylus slots come after touch slots */
    id = data->num_touchids + (msg[0] - data->T63_reportid_min);

    if (id < 0 || id > (data->num_touchids + data->num_stylusids))
    {
        TOUCH_ERR("invalid stylus id %d, max slot is %d\n",
                id, data->num_stylusids);
        return;
    }

    x = msg[3] | (msg[4] << 8);
    y = msg[5] | (msg[6] << 8);
    pressure = msg[7] & MXT_T63_STYLUS_PRESSURE_MASK;

    TOUCH_DEBUG(
            "[%d] %c%c%c%c x: %d y: %d pressure: %d stylus:%c%c%c%c\n",
            id,
            (msg[1] & MXT_T63_STYLUS_SUPPRESS) ? 'S' : '.',
            (msg[1] & MXT_T63_STYLUS_MOVE)     ? 'M' : '.',
            (msg[1] & MXT_T63_STYLUS_RELEASE)  ? 'R' : '.',
            (msg[1] & MXT_T63_STYLUS_PRESS)    ? 'P' : '.',
            x, y, pressure,
            (msg[2] & MXT_T63_STYLUS_BARREL) ? 'B' : '.',
            (msg[2] & MXT_T63_STYLUS_ERASER) ? 'E' : '.',
            (msg[2] & MXT_T63_STYLUS_TIP)    ? 'T' : '.',
            (msg[2] & MXT_T63_STYLUS_DETECT) ? 'D' : '.');

    input_mt_slot(input_dev, id);

    if (msg[2] & MXT_T63_STYLUS_DETECT)
    {
        input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 1);
        input_report_abs(input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
        input_report_abs(input_dev, ABS_MT_PRESSURE, pressure);
    }
    else
    {
        input_mt_report_slot_state(input_dev, MT_TOOL_PEN, 0);
    }

    input_report_key(input_dev, BTN_STYLUS,
                     (msg[2] & MXT_T63_STYLUS_ERASER));
    input_report_key(input_dev, BTN_STYLUS2,
                     (msg[2] & MXT_T63_STYLUS_BARREL));

    mxt_input_sync(input_dev);
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
    u8 report_id = message[0];
    bool dump = data->debug_enabled;
    if (report_id == MXT_RPTID_NOMSG)
        return 0;

    if (data == NULL)
        TOUCH_DEBUG("mxt_proc_message data is null...\n");
    if (message == NULL)
        TOUCH_DEBUG("mxt_proc_message message is null...\n");

    if (report_id == data->T6_reportid)
    {
        mxt_proc_t6_messages(data, message);
    }
    else if (report_id >= data->T9_reportid_min
             && report_id <= data->T9_reportid_max)
    {
        mxt_proc_t9_message(data, message);
    }
    else if (report_id >= data->T100_reportid_min
             && report_id <= data->T100_reportid_max)
    {
        mxt_proc_t100_message(data, message);
    }
    else if (report_id == data->T19_reportid)
    {
        mxt_input_button(data, message);
        //data->update_input = true;
    }
    else if (report_id == data->T25_reportid)
    {
        mxt_proc_t25_messages(data, message);
        data->update_input = true;
    }
    else if (report_id >= data->T63_reportid_min
             && report_id <= data->T63_reportid_max)
    {
        mxt_proc_t63_messages(data, message);
    }
    else if (report_id >= data->T42_reportid_min
             && report_id <= data->T42_reportid_max)
    {
        mxt_proc_t42_messages(data, message);
    }
    else if (report_id == data->T48_reportid)
    {
        mxt_proc_t48_messages(data, message);
    }
    else if (report_id >= data->T15_reportid_min
             && report_id <= data->T15_reportid_max)
    {
        mxt_proc_t15_messages(data, message);
    }
    else
    {
        dump = true;
    }

    if (dump)
        mxt_dump_message(data, message);

    if (data->debug_v2_enabled)
        mxt_debug_msg_add(data, message);

    return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
    struct device *dev = &data->client->dev;
    int ret;
    int i;
    u8 num_valid = 0;

    /* Safety check for msg_buf */
    if (count > data->max_reportid)
        return -EINVAL;

    /* Process remaining messages if necessary */
    ret = __mxt_read_reg(data->client, data->T5_address,
                         data->T5_msg_size * count, data->msg_buf);
    if (ret)
    {
        TOUCH_ERR("Failed to read %u messages (%d)\n", count, ret);
        return ret;
    }

    for (i = 0;  i < count; i++)
    {
        ret = mxt_proc_message(data,
                               data->msg_buf + data->T5_msg_size * i);

        if (ret == 1)
            num_valid++;
    }

    /* return number of messages read */
    return num_valid;
}

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int ret;
    u8 count, num_left;

    if (data == NULL)
        TOUCH_DEBUG("mxt_process_messages_t44 data is null...\n");

    /* Read T44 and T5 together */
    ret = __mxt_read_reg(data->client, data->T44_address,
                         data->T5_msg_size + 1, data->msg_buf);
    if (ret)
    {
        TOUCH_ERR("Failed to read T44 and T5 (%d)\n", ret);
        return IRQ_NONE;
    }

    count = data->msg_buf[0];

    if (count == 0)
    {
        //Remove this log affect by charger pin
        //dev_warn(dev, "Interrupt triggered but zero messages\n");
        return IRQ_NONE;
    }
    else if (count > data->max_reportid)
    {
        TOUCH_ERR("T44 count %d exceeded max report id\n", count);
        count = data->max_reportid;
    }

    /* Process first message */
    ret = mxt_proc_message(data, data->msg_buf + 1);
    if (ret < 0)
    {
        TOUCH_WARN("Unexpected invalid message\n");
        return IRQ_NONE;
    }

    num_left = count - 1;

    /* Process remaining messages if necessary */
    if (num_left)
    {
        ret = mxt_read_and_process_messages(data, num_left);
        if (ret < 0)
            goto end;
        else if (ret != num_left)
            TOUCH_ERR("Unexpected invalid message\n");
    }

end:
    if (data->update_input)
    {
        mxt_input_sync(data->input_dev);
        data->update_input = false;
    }

    return IRQ_HANDLED;
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int count, read;
    u8 tries = 2;

    count = data->max_reportid;

    /* Read messages until we force an invalid */
    do
    {
        read = mxt_read_and_process_messages(data, count);
        if (read < count)
            return 0;
    }
    while (--tries);

    if (data->update_input)
    {
        mxt_input_sync(data->input_dev);
        data->update_input = false;
    }

    TOUCH_ERR("CHG pin isn't cleared\n");
    return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
    int total_handled, num_handled;
    u8 count = data->last_message_count;
    if (count < 1 || count > data->max_reportid)
        count = 1;

    if (data == NULL)
        TOUCH_NOTICE("data is null...\n");

    /* include final invalid message */
    total_handled = mxt_read_and_process_messages(data, count + 1);
    if (total_handled < 0)
        return IRQ_NONE;
    /* if there were invalid messages, then we are done */
    else if (total_handled <= count)
        goto update_count;

    /* read two at a time until an invalid message or else we reach
     * reportid limit */
    do
    {
        num_handled = mxt_read_and_process_messages(data, 2);
        if (num_handled < 0)
            return IRQ_NONE;

        total_handled += num_handled;

        if (num_handled < 2)
            break;
    }
    while (total_handled < data->num_touchids);

update_count:
    data->last_message_count = total_handled;

    if (data->enable_reporting && data->update_input)
    {
        mxt_input_sync(data->input_dev);
        data->update_input = false;
    }

    return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
    struct mxt_data *data = dev_id;
    if (data->in_bootloader)
    {
        /* bootloader state transition completion */
        complete(&data->bl_completion);
        return IRQ_HANDLED;
    }

    if (!data->object_table)
    {
        TOUCH_NOTICE(" data->object_table NULL.\n");
        return IRQ_NONE;
    }

    if (data->T44_address)
    {
        return mxt_process_messages_t44(data);
    }
    else
    {
        return mxt_process_messages(data);
    }
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
                          u8 value, bool wait)
{
    u16 reg;
    u8 command_register;
    int timeout_counter = 0;
    int ret;

    reg = data->T6_address + cmd_offset;

    ret = mxt_write_reg(data->client, reg, value);
    if (ret)
        return ret;

    if (!wait)
        return 0;

    do
    {
        msleep(20);
        ret = __mxt_read_reg(data->client, reg, 1, &command_register);
        if (ret)
            return ret;
    }
    while ((command_register != 0) && (timeout_counter++ <= 100));

    if (timeout_counter > 100)
    {
        TOUCH_ERR("Command failed!\n");
        return -EIO;
    }

    return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int ret = 0;

    TOUCH_NOTICE("Resetting chip\n");

    INIT_COMPLETION(data->reset_completion);


    ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
    if (ret)
        return ret;

    ret = mxt_wait_for_completion(data, &data->reset_completion,
                                  MXT_RESET_TIMEOUT);
    if (ret)
        return ret;

    return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
    /* on failure, CRC is set to 0 and config will always be downloaded */
    data->config_crc = 0;
    INIT_COMPLETION(data->crc_completion);

    mxt_t6_command(data, cmd, value, true);

    /* Wait for crc message. On failure, CRC is set to 0 and config will
     * always be downloaded */
    mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
}

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
    static const unsigned int crcpoly = 0x80001B;
    u32 result;
    u32 data_word;

    data_word = (secondbyte << 8) | firstbyte;
    result = ((*crc << 1) ^ data_word);

    if (result & 0x1000000)
        result ^= crcpoly;

    *crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
    u32 crc = 0;
    u8 *ptr = base + start_off;
    u8 *last_val = base + end_off - 1;

    if (end_off < start_off)
        return -EINVAL;

    while (ptr < last_val)
    {
        mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
        ptr += 2;
    }

    /* if len is odd, fill the last byte with 0 */
    if (ptr == last_val)
        mxt_calc_crc24(&crc, *ptr, 0);

    /* Mask to 24-bit */
    crc &= 0x00FFFFFF;

    return crc;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    int val;

    if (data->pdata->irqflags & IRQF_TRIGGER_LOW)
        return 0;

    if (data->T18_address)
    {
        error = __mxt_read_reg(client,
                               data->T18_address + MXT_COMMS_CTRL,
                               1, &val);
        if (error)
            return error;

        if (val & MXT_COMMS_RETRIGEN)
            return 0;
    }

    TOUCH_WARN("Enabling RETRIGEN workaround\n");
    data->use_retrigen_workaround = true;
    return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);

/*
 * mxt_check_reg_init - download configuration to chip
 *
 * Atmel Raw Config File Format
 *
 * The first four lines of the raw config file contain:
 *  1) Version
 *  2) Chip ID Information (first 7 bytes of device memory)
 *  3) Chip Information Block 24-bit CRC Checksum
 *  4) Chip Configuration 24-bit CRC Checksum
 *
 * The rest of the file consists of one line per object instance:
 *   <TYPE> <INSTANCE> <SIZE> <CONTENTS>
 *
 *   <TYPE> - 2-byte object type as hex
 *   <INSTANCE> - 2-byte object instance number as hex
 *   <SIZE> - 2-byte object size as hex
 *   <CONTENTS> - array of <SIZE> 1-byte hex values
 */
static int mxt_check_reg_init(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    struct mxt_info cfg_info;
    struct mxt_object *object;
    const struct firmware *cfg = NULL;
    int ret;
    int offset;
    int data_pos;
    int byte_offset;
    int i;
    int cfg_start_ofs;
    u32 info_crc, config_crc, calculated_crc;
    u8 *config_mem;
    size_t config_mem_size;
    unsigned int type, instance, size;
    u8 val;
    u16 reg;
    u32 upper_cfg_version, lower_cfg_version;
    int data_pos_cfg_version = 299; // count by raw data
    int offset_cfg_version =3;// count by raw data
    int prepare_cfg_version = 0;

    TOUCH_NOTICE("mxt_check_reg_init\n");
    if (!data->cfg_name)
    {
        TOUCH_DEBUG("Skipping cfg download\n");
        return 0;
    }

    ret = request_firmware(&cfg, data->cfg_name, dev);
    //printk("[Atmel] cfg: %s \n",cfg->data);
    if (ret < 0)
    {
        TOUCH_ERR("Failure to request config file %s\n",
                data->cfg_name);
        return 0;
    }

    mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

    if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC)))
    {
        TOUCH_ERR("Unrecognised config file\n");
        ret = -EINVAL;
        goto release;
    }

    data_pos = strlen(MXT_CFG_MAGIC);

    /* Load information block and check */
    for (i = 0; i < sizeof(struct mxt_info); i++)
    {
        ret = sscanf(cfg->data + data_pos, "%hhx%n",
                     (unsigned char *)&cfg_info + i,
                     &offset);
        if (ret != 1)
        {
            TOUCH_ERR("Bad format\n");
            ret = -EINVAL;
            goto release;
        }

        data_pos += offset;
    }

    if (cfg_info.family_id != data->info->family_id)
    {
        TOUCH_ERR("Family ID mismatch!\n");
        ret = -EINVAL;
        goto release;
    }

    if (cfg_info.variant_id != data->info->variant_id)
    {
        TOUCH_ERR("Variant ID mismatch!\n");
        ret = -EINVAL;
        goto release;
    }

    //printk("[Atmel] : cfg_info.family_id: %u,  data->info->family_id:%u \n",cfg_info.family_id,  data->info->family_id);
    //printk("[Atmel] : cfg_info.variant_id: %u, data->info->variant_id:%u \n",cfg_info.variant_id, data->info->variant_id);

    /* Read CRCs */
    ret = sscanf(cfg->data + data_pos, "%x%n", &info_crc, &offset);
    if (ret != 1)
    {
        TOUCH_ERR("Bad format: failed to parse Info CRC\n");
        ret = -EINVAL;
        goto release;
    }
    data_pos += offset;

    ret = sscanf(cfg->data + data_pos, "%x%n", &config_crc, &offset);
    if (ret != 1)
    {
        TOUCH_ERR("Bad format: failed to parse Config CRC\n");
        ret = -EINVAL;
        goto release;
    }
    data_pos += offset;
    TOUCH_NOTICE("info_crc: 0x%06X, config_crc:0x%06X \n", info_crc, config_crc);
    TOUCH_NOTICE("data->info_crc: 0x%06X, data->config_crc:0x%06X \n", data->info_crc, data->config_crc);

    /*After cfg version 30(>=31), add tp id in config content, Jtouch:1 Ofilm:2
       If change TP, force update cfg*/
    TOUCH_NOTICE("data->tp_id: %d, data->cfg_tp_id: %d \n", data->tp_id, data->cfg_tp_id);
    if ((data->cfg_version > 30) && (data->tp_id != data->cfg_tp_id)) {
        cfg_force = true;
        TOUCH_NOTICE("Set cfg_force to true, because tp_id != cfg_tp_id, force update config\n");
    }

    /* Calculate config version from boot or echo, whether update or not */

    ret = sscanf(cfg->data + data_pos_cfg_version, "%x%n", &upper_cfg_version, &offset_cfg_version);
    data_pos_cfg_version += offset_cfg_version;
    ret = sscanf(cfg->data + data_pos_cfg_version, "%x%n", &lower_cfg_version, &offset_cfg_version);
    prepare_cfg_version = upper_cfg_version * 10 + lower_cfg_version;
    TOUCH_NOTICE("Now config version is %d, Prepare config version is %d\n", data->cfg_version, prepare_cfg_version);

    if (!cfg_force) {
        if (data->cfg_version >= prepare_cfg_version) {
            TOUCH_NOTICE("Don't need config update\n");
	    ret = 0;
            goto release;
        }
        else {
            TOUCH_NOTICE("Need config update\n");
        }
    }
    else {
        TOUCH_NOTICE("Force config update\n");
    }

    /* The Info Block CRC is calculated over mxt_info and the object table
     * If it does not match then we are trying to load the configuration
     * from a different chip or firmware version, so the configuration CRC
     * is invalid anyway. */
    /*
    if ((info_crc == data->info_crc)&&(!cfg_force))
    {
        if (config_crc == 0 || data->config_crc == 0)
        {
            dev_info(dev, "CRC zero, attempting to apply config, cfg_force=%d\n",cfg_force);
        }
        else if (config_crc == data->config_crc)
        {
            dev_info(dev, "Config CRC 0x%06X: OK,config_crc=0x%06X, data->config_crc:0x%06X,cfg_force=%d\n",
                     data->config_crc,config_crc,data->config_crc,cfg_force);
            ret = 0;
            goto release;
        }
        else
        {
            dev_info(dev, "Config CRC 0x%06X: does not match file 0x%06X,cfg_force=%d\n",
                     data->config_crc, config_crc,cfg_force);
        }
    }
    else
    {
        dev_warn(dev,
                 "Warning: Info CRC error - device=0x%06X file=0x%06X,cfg_force=%d\n",
                 data->info_crc, info_crc,cfg_force);
    }
    */

    /* Malloc memory to store configuration */
    cfg_start_ofs = MXT_OBJECT_START
                    + data->info->object_num * sizeof(struct mxt_object)
                    + MXT_INFO_CHECKSUM_SIZE;
    config_mem_size = data->mem_size - cfg_start_ofs;
    config_mem = kzalloc(config_mem_size, GFP_KERNEL);
    if (!config_mem)
    {
        TOUCH_ERR("Failed to allocate memory\n");
        ret = -ENOMEM;
        goto release;
    }

    while (data_pos < cfg->size)
    {
        /* Read type, instance, length */
        ret = sscanf(cfg->data + data_pos, "%x %x %x%n",
                     &type, &instance, &size, &offset);
        if (ret == 0)
        {
            /* EOF */
            break;
        }
        else if (ret != 3)
        {
            TOUCH_ERR("Bad format: failed to parse object\n");
            ret = -EINVAL;
            goto release_mem;
        }
        data_pos += offset;

        object = mxt_get_object(data, type);
        if (!object)
        {
            /* Skip object */
            for (i = 0; i < size; i++)
            {
                ret = sscanf(cfg->data + data_pos, "%hhx%n",
                             &val,
                             &offset);
                data_pos += offset;
            }
            continue;
        }

        if (size > mxt_obj_size(object))
        {
            /* Either we are in fallback mode due to wrong
             * config or config from a later fw version,
             * or the file is corrupt or hand-edited */
            TOUCH_WARN("Discarding %u byte(s) in T%u\n",
                     size - mxt_obj_size(object), type);
        }
        else if (mxt_obj_size(object) > size)
        {
            /* If firmware is upgraded, new bytes may be added to
             * end of objects. It is generally forward compatible
             * to zero these bytes - previous behaviour will be
             * retained. However this does invalidate the CRC and
             * will force fallback mode until the configuration is
             * updated. We warn here but do nothing else - the
             * malloc has zeroed the entire configuration. */
            TOUCH_WARN("Zeroing %u byte(s) in T%d\n",
                     mxt_obj_size(object) - size, type);
        }

        if (instance >= mxt_obj_instances(object))
        {
            TOUCH_ERR("Object instances exceeded!\n");
            ret = -EINVAL;
            goto release_mem;
        }

        reg = object->start_address + mxt_obj_size(object) * instance;

        for (i = 0; i < size; i++)
        {
            ret = sscanf(cfg->data + data_pos, "%hhx%n",
                         &val,
                         &offset);
            if (ret != 1)
            {
                TOUCH_ERR("Bad format in T%d\n", type);
                ret = -EINVAL;
                goto release_mem;
            }
            data_pos += offset;

            if (i > mxt_obj_size(object))
                continue;

            byte_offset = reg + i - cfg_start_ofs;

            if ((byte_offset >= 0)
                    && (byte_offset <= config_mem_size))
            {
                *(config_mem + byte_offset) = val;
            }
            else
            {
                TOUCH_ERR("Bad object: reg:%d, T%d, ofs=%d\n",
                        reg, object->type, byte_offset);
                ret = -EINVAL;
                goto release_mem;
            }
        }
    }

    /* calculate crc of the received configs (not the raw config file) */
    if (data->T7_address < cfg_start_ofs)
    {
        TOUCH_ERR("Bad T7 address, T7addr = %x, config offset %x\n",
                data->T7_address, cfg_start_ofs);
        ret = 0;
        goto release_mem;
    }

    calculated_crc = mxt_calculate_crc(config_mem,
                                       data->T7_address - cfg_start_ofs,
                                       config_mem_size);

    if (config_crc > 0 && (config_crc != calculated_crc))
        TOUCH_WARN("Config CRC error, calculated=%06X, file=%06X\n",
                 calculated_crc, config_crc);

    /* Write configuration as blocks */
    byte_offset = 0;
    while (byte_offset < config_mem_size)
    {
        size = config_mem_size - byte_offset;

        if (size > MXT_MAX_BLOCK_WRITE)
            size = MXT_MAX_BLOCK_WRITE;

        ret = __mxt_write_reg(data->client,
                              cfg_start_ofs + byte_offset,
                              size, config_mem + byte_offset);
        if (ret != 0)
        {
            TOUCH_ERR("Config write error, ret=%d\n", ret);
            goto release_mem;
        }
        TOUCH_NOTICE("CFG Sent %d blocks\n", byte_offset);
        byte_offset += size;
    }

    mxt_update_crc(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);

    ret = mxt_check_retrigen(data);
    if (ret)
        goto release_mem;


    ret = mxt_soft_reset(data);
    if (ret)
        goto release_mem;

    cfg_force = false;
    data->cfg_version = prepare_cfg_version; 
    TOUCH_NOTICE("Config written success!\n");

    /* T7 config may have changed */
    mxt_init_t7_power_cfg(data);

release_mem:
    kfree(config_mem);
release:
    release_firmware(cfg);
    return ret;
}

static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
    struct device *dev = &data->client->dev;
    int error;
    struct t7_config *new_config;
    struct t7_config deepsleep = { .active = 0, .idle = 0 };

    if (sleep == MXT_POWER_CFG_DEEPSLEEP)
        new_config = &deepsleep;
    else
        new_config = &data->t7_cfg;

    error = __mxt_write_reg(data->client, data->T7_address,
                            sizeof(data->t7_cfg),
                            new_config);
    if (error)
        return error;

    TOUCH_DEBUG("Set T7 ACTV:%d IDLE:%d\n",
            new_config->active, new_config->idle);

    return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int error;
    bool retry = false;

recheck:
    error = __mxt_read_reg(data->client, data->T7_address,
                           sizeof(data->t7_cfg), &data->t7_cfg);
    if (error)
        return error;

    if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0)
    {
        if (!retry)
        {
            TOUCH_NOTICE("T7 cfg zero, resetting\n");
            mxt_soft_reset(data);
            retry = true;
            goto recheck;
        }
        else
        {
            TOUCH_DEBUG("T7 cfg zero after reset, overriding\n");
            data->t7_cfg.active = 20;
            data->t7_cfg.idle = 100;
            return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
        }
    }
    else
    {
        TOUCH_NOTICE("Initialised power cfg: ACTV %d, IDLE %d\n",
                 data->t7_cfg.active, data->t7_cfg.idle);
        return 0;
    }
}

static void mxt_control_irq(struct mxt_data *data, bool irq_status)
{
    if (irq_status == true)
    {
        data->irq_status = true;
        data->irq_count += 1;
    }
    else if (irq_status == false)
    {
        data->irq_status = false;
        data->irq_count -= 1;
    }
    return;
}

static int mxt_acquire_irq(struct mxt_data *data)
{
    int error;

    enable_irq(data->irq);
    mxt_control_irq(data, true);

    if (data->use_retrigen_workaround)
    {
        error = mxt_process_messages_until_invalid(data);
        if (error)
            return error;
    }

    return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
    if (data->input_dev)
    {
        input_unregister_device(data->input_dev);
        data->input_dev = NULL;
    }
}

static void mxt_free_object_table(struct mxt_data *data)
{
    mxt_debug_msg_remove(data);

    kfree(data->raw_info_block);
    data->object_table = NULL;
    data->info = NULL;
    data->raw_info_block = NULL;
    kfree(data->msg_buf);
    data->msg_buf = NULL;

    mxt_free_input_device(data);

    data->enable_reporting = false;
    data->T5_address = 0;
    data->T5_msg_size = 0;
    data->T6_reportid = 0;
    data->T7_address = 0;
    data->T9_reportid_min = 0;
    data->T9_reportid_max = 0;
    data->T15_reportid_min = 0;
    data->T15_reportid_max = 0;
    data->T18_address = 0;
    data->T19_address = 0;              //add for TP_SHOW
    data->T19_reportid = 0;
    data->gpio_status = 0;              //add for TP_SHOW
    data->get_gpio_status = false;      //add for TP_SHOW
    data->T25_address = 0;          //add by Eric
    data->T25_reportid = 0;
    data->T37_address = 0;
    data->T38_address = 0;			//add by Eric 4/15
    data->T42_reportid_min = 0;
    data->T42_reportid_max = 0;
    data->T44_address = 0;
    data->T48_reportid = 0;
    data->T63_reportid_min = 0;
    data->T63_reportid_max = 0;
    data->T78_address = 0;	//add for glove
    data->T100_reportid_min = 0;
    data->T100_reportid_max = 0;
    data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int i;
    u8 reportid;
    u16 end_address;
    /* Valid Report IDs start counting from 1 */
    reportid = 1;
    data->mem_size = 0;
    for (i = 0; i < data->info->object_num; i++)
    {
        struct mxt_object *object = data->object_table + i;
        u8 min_id, max_id;

        le16_to_cpus(&object->start_address);

        if (object->num_report_ids)
        {
            min_id = reportid;
            reportid += object->num_report_ids *
                        mxt_obj_instances(object);
            max_id = reportid - 1;
        }
        else
        {
            min_id = 0;
            max_id = 0;
        }
        /*
        TOUCH_DEBUG(
                "T%u Start:%u Size:%u Instances:%u Report IDs:%u-%u\n",
                object->type, object->start_address,
                mxt_obj_size(object), mxt_obj_instances(object),
                min_id, max_id);
        */
        switch (object->type)
        {
        case MXT_GEN_MESSAGE_T5:
            if (data->info->family_id == 0x80)
            {
                /* On mXT224 read and discard unused CRC byte
                 * otherwise DMA reads are misaligned */
                data->T5_msg_size = mxt_obj_size(object);
            }
            else
            {
                /* CRC not enabled, so skip last byte */
                data->T5_msg_size = mxt_obj_size(object) - 1;
            }
            data->T5_address = object->start_address;
        case MXT_GEN_COMMAND_T6:
            data->T6_reportid = min_id;
            data->T6_address = object->start_address;
            break;
        case MXT_GEN_POWER_T7:
            data->T7_address = object->start_address;
            break;
        case MXT_TOUCH_MULTI_T9:
            /* Only handle messages from first T9 instance */
            data->T9_reportid_min = min_id;
            data->T9_reportid_max = min_id +
                                    object->num_report_ids - 1;
            data->num_touchids = object->num_report_ids;
            break;
        case MXT_TOUCH_KEYARRAY_T15:
            data->T15_reportid_min = min_id;
            data->T15_reportid_max = max_id;
            break;
        case MXT_SPT_COMMSCONFIG_T18:
            data->T18_address = object->start_address;
            break;
        case MXT_PROCI_TOUCHSUPPRESSION_T42:
            data->T42_reportid_min = min_id;
            data->T42_reportid_max = max_id;
            break;
        case MXT_SPT_MESSAGECOUNT_T44:
            data->T44_address = object->start_address;
            break;
        case MXT_SPT_GPIOPWM_T19:
            data->T19_address = object->start_address;
            data->T19_reportid = min_id;
            break;
        //add by Eric
        case MXT_SPT_SELFTEST_T25:
            data->T25_address = object->start_address;
            data->T25_reportid = min_id;
            break;
        case MXT_SPT_USERDATA_T37:
            data->T37_address = object->start_address;
            break;
        //add by Eric 4/15
        case MXT_SPT_USERDATA_T38:
            data->T38_address = object->start_address;
            break;
        case MXT_PROCG_NOISESUPPRESSION_T48:
            data->T48_reportid = min_id;
            break;
        case MXT_PROCI_ACTIVE_STYLUS_T63:
            /* Only handle messages from first T63 instance */
            data->T63_reportid_min = min_id;
            data->T63_reportid_max = min_id;
            data->num_stylusids = 1;
            break;
        case MXT_PROCI_GLOVE_DETECTION_T78:	//add for glove
            data->T78_address = object->start_address;
            break;
        case MXT_TOUCH_MULTITOUCHSCREEN_T100:
            data->T100_reportid_min = min_id;
            data->T100_reportid_max = max_id;
            /* first two report IDs reserved */
            data->num_touchids = object->num_report_ids - 2;
            break;
        }

        end_address = object->start_address
                      + mxt_obj_size(object) * mxt_obj_instances(object) - 1;

        if (end_address >= data->mem_size)
            data->mem_size = end_address + 1;
    }

    /* Store maximum reportid */
    data->max_reportid = reportid;

    /* If T44 exists, T5 position has to be directly after */
    if (data->T44_address && (data->T5_address != data->T44_address + 1))
    {
        TOUCH_ERR("Invalid T44 position\n");
        return -EINVAL;
    }

    data->msg_buf = kcalloc(data->max_reportid,
                            data->T5_msg_size, GFP_KERNEL);
    if (!data->msg_buf)
    {
        TOUCH_ERR("Failed to allocate message buffer\n");
        return -ENOMEM;
    }

    return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    size_t size;
    void *buf;
    uint8_t num_objects;
    u32 calculated_crc;
    u8 *crc_ptr;

    /* If info block already allocated, free it */
    if (data->raw_info_block != NULL)
    {
        mxt_free_object_table(data);
    }

    /* Read 7-byte ID information block starting at address 0 */
    size = sizeof(struct mxt_info);
    buf = kzalloc(size, GFP_KERNEL);
    if (!buf)
    {
        TOUCH_ERR("Failed to allocate memory\n");
        return -ENOMEM;
    }

    error = __mxt_read_reg(client, 0, size, buf);
    if (error)
        goto err_free_mem;

    /* Resize buffer to give space for rest of info block */
    num_objects = ((struct mxt_info *)buf)->object_num;
    size += (num_objects * sizeof(struct mxt_object))
            + MXT_INFO_CHECKSUM_SIZE;

    buf = krealloc(buf, size, GFP_KERNEL);
    if (!buf)
    {
        TOUCH_ERR("Failed to allocate memory\n");
        error = -ENOMEM;
        goto err_free_mem;
    }

    /* Read rest of info block */
    error = __mxt_read_reg(client, MXT_OBJECT_START,
                           size - MXT_OBJECT_START,
                           buf + MXT_OBJECT_START);
    if (error)
        goto err_free_mem;

    /* Extract & calculate checksum */
    crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
    data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

    calculated_crc = mxt_calculate_crc(buf, 0,
                                       size - MXT_INFO_CHECKSUM_SIZE);

    /* CRC mismatch can be caused by data corruption due to I2C comms
     * issue or else device is not using Object Based Protocol */
    if ((data->info_crc == 0) || (data->info_crc != calculated_crc))
    {
        TOUCH_ERR(
                "Info Block CRC error calculated=0x%06X read=0x%06X\n",
                data->info_crc, calculated_crc);
        return -EIO;
    }

    /* Save pointers in device data structure */
    data->raw_info_block = buf;
    data->info = (struct mxt_info *)buf;
    data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);
    TOUCH_NOTICE(
             "Family: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
             data->info->family_id, data->info->variant_id,
             data->info->version >> 4, data->info->version & 0xf,
             data->info->build, data->info->object_num);

    /* Parse object table information */
    error = mxt_parse_object_table(data);
    if (error)
    {
        TOUCH_ERR("Error %d reading object table\n", error);
        mxt_free_object_table(data);
        return error;
    }

    return 0;

err_free_mem:
    kfree(buf);
    return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    struct t9_range range;
    unsigned char orient;
    struct mxt_object *object;

    object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
    if (!object)
        return -EINVAL;

    error = __mxt_read_reg(client,
                           object->start_address + MXT_T9_RANGE,
                           sizeof(range), &range);
    if (error)
        return error;

    le16_to_cpus(range.x);
    le16_to_cpus(range.y);

    error =  __mxt_read_reg(client,
                            object->start_address + MXT_T9_ORIENT,
                            1, &orient);
    if (error)
        return error;

    /* Handle default values */
    if (range.x == 0)
        range.x = 1023;

    if (range.y == 0)
        range.y = 1023;

    if (orient & MXT_T9_ORIENT_SWITCH)
    {
        data->max_x = range.y;
        data->max_y = range.x;
    }
    else
    {
        data->max_x = range.x;
        data->max_y = range.y;
    }

    TOUCH_NOTICE(
             "Touchscreen size X%uY%u\n", data->max_x, data->max_y);

    return 0;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
    gpio_set_value(data->pdata->gpio_reset, 0);

    //regulator_enable(data->reg_vdd);
    //regulator_enable(data->reg_avdd);
    msleep(MXT_REGULATOR_DELAY);

    INIT_COMPLETION(data->bl_completion);
    gpio_set_value(data->pdata->gpio_reset, 1);
    msleep(90);
    TOUCH_NOTICE("msleep(90) \n");
    mxt_wait_for_completion(data, &data->bl_completion, MXT_POWERON_DELAY);
}

static void mxt_regulator_disable(struct mxt_data *data)
{
    //regulator_disable(data->reg_vdd);
    //regulator_disable(data->reg_avdd);
}

static void mxt_probe_regulators(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int error;
    /* According to maXTouch power sequencing specification, RESET line
     * must be kept low until some time after regulators come up to
     * voltage */
    if (!data->pdata->gpio_reset)
    {
        TOUCH_WARN("Must have reset GPIO to use regulator support\n");
        goto fail;
    }

/*   
    data->reg_vdd = regulator_get(dev, "vdd");
    if (IS_ERR(data->reg_vdd))
    {
        error = PTR_ERR(data->reg_vdd);
        dev_err(dev, "Error %d getting vdd regulator\n", error);
        //goto fail;
    }

    data->reg_avdd = regulator_get(dev, "avdd");
    if (IS_ERR(data->reg_avdd))
    {
        error = PTR_ERR(data->reg_vdd);
        dev_err(dev, "Error %d getting avdd regulator\n", error);
        //goto fail_release;
    }
*/

    //data->use_regulator = true;
    mxt_regulator_enable(data);

    //dev_dbg(dev, "Initialised regulators\n");
    return;

fail_release:
    //regulator_put(data->reg_vdd);
fail:
    //data->reg_vdd = NULL;
    //data->reg_avdd = NULL;
    data->use_regulator = false;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    struct mxt_object *object;
    u16 range_x, range_y;
    u8 cfg, tchaux;
    u8 aux;

    object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
    if (!object)
        return -EINVAL;

    error = __mxt_read_reg(client,
                           object->start_address + MXT_T100_XRANGE,
                           sizeof(range_x), &range_x);
    if (error)
        return error;

    le16_to_cpus(range_x);

    error = __mxt_read_reg(client,
                           object->start_address + MXT_T100_YRANGE,
                           sizeof(range_y), &range_y);
    if (error)
        return error;

    le16_to_cpus(range_y);

    error =  __mxt_read_reg(client,
                            object->start_address + MXT_T100_CFG1,
                            1, &cfg);
    if (error)
        return error;

    error =  __mxt_read_reg(client,
                            object->start_address + MXT_T100_TCHAUX,
                            1, &tchaux);
    if (error)
        return error;

    /* Handle default values */
    if (range_x == 0)
        range_x = 1023;

    /* Handle default values */
    if (range_x == 0)
        range_x = 1023;

    if (range_y == 0)
        range_y = 1023;

    if (cfg & MXT_T100_CFG_SWITCHXY)
    {
        data->max_x = range_y;
        data->max_y = range_x;
    }
    else
    {
        data->max_x = range_x;
        data->max_y = range_y;
    }

    /* allocate aux bytes */
    aux = 6;

    if (tchaux & MXT_T100_TCHAUX_VECT)
        data->t100_aux_vect = aux++;

    if (tchaux & MXT_T100_TCHAUX_AMPL)
        data->t100_aux_ampl = aux++;

    if (tchaux & MXT_T100_TCHAUX_AREA)
        data->t100_aux_area = aux++;

    TOUCH_NOTICE(
             "T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

    return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    struct input_dev *input_dev;
    int error;

    error = mxt_read_t100_config(data);
    if (error)
        TOUCH_WARN("Failed to initialize T9 resolution\n");

    input_dev = input_allocate_device();
    if (!data || !input_dev)
    {
        TOUCH_ERR("Failed to allocate memory\n");
        return -ENOMEM;
    }

    input_dev->name = "atmel_mxt_ts";

    input_dev->phys = data->phys;
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = &data->client->dev;
    input_dev->open = mxt_input_open;
    input_dev->close = mxt_input_close;

    set_bit(EV_ABS, input_dev->evbit);
    input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

    /* For single touch */
    input_set_abs_params(input_dev, ABS_X,
                         0, data->max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_Y,
                         0, data->max_y, 0, 0);
//		printk("Toby test > data->max_x=%d ,data->max_y=%d ",data->max_x,data->max_y);

    if (data->t100_aux_ampl)
        input_set_abs_params(input_dev, ABS_PRESSURE,
                             0, 255, 0, 0);

    /* For multi touch */
    error = input_mt_init_slots(input_dev, data->num_touchids,0);
//		printk("Toby test > data->num_touchids=%d ",data->num_touchids);
    if (error)
    {
        TOUCH_ERR("Error %d initialising slots\n", error);
        goto err_free_mem;
    }

    //input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
                         0, data->max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
                         0, data->max_y, 0, 0);

//		printk("Toby test > multi data->max_x=%d ,data->max_y=%d ",data->max_x,data->max_y);

    if (data->t100_aux_area)
        input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
                             0, MXT_MAX_AREA, 0, 0);

    if (data->t100_aux_ampl)
        input_set_abs_params(input_dev, ABS_MT_PRESSURE,
                             0, 255, 0, 0);

    if (data->t100_aux_vect)
        input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
                             0, 255, 0, 0);

    input_set_drvdata(input_dev, data);

    error = input_register_device(input_dev);
    if (error)
    {
        TOUCH_ERR("Error %d registering input device\n", error);
        goto err_free_mem;
    }

    data->input_dev = input_dev;

    return 0;

err_free_mem:
    input_free_device(input_dev);
    return error;
}

static int mxt_initialize(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    bool alt_bootloader_addr = false;
    bool retry = false;
//		printk("Toby test > mxt initilalize....");

retry_info:
    error = mxt_read_info_block(data);
    if (error)
    {
retry_bootloader:
        error = mxt_probe_bootloader(data, alt_bootloader_addr);
        if (error)
        {
            if (alt_bootloader_addr)
            {
                /* Chip is not in appmode or bootloader mode */
                return error;
            }

            TOUCH_NOTICE("Trying alternate bootloader address\n");
            alt_bootloader_addr = true;
            goto retry_bootloader;
        }
        else
        {
            if (retry)
            {
                TOUCH_ERR(
                        "Could not recover device from "
                        "bootloader mode\n");
                /* this is not an error state, we can reflash
                 * from here */
                data->in_bootloader = true;
                return 0;
            }

            /* Attempt to exit bootloader into app mode */
            mxt_send_bootloader_cmd(data, false);
            msleep(MXT_FW_RESET_TIME);
            retry = true;
            goto retry_info;
        }
    }

    error = mxt_check_retrigen(data);
    if (error)
        return error;

    error = mxt_acquire_irq(data);
    if (error)
        return error;

    error = mxt_configure_objects(data);
    if (error)
        return error;

    return 0;
}

static int mxt_configure_objects(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;

//		printk("Toby test > mxt configure objects.\n");
    error = mxt_debug_msg_init(data);
    if (error)
        return error;

    error = mxt_init_t7_power_cfg(data);
    if (error)
    {
        TOUCH_ERR("Failed to initialize power cfg\n");
        return error;
    }

    /* Check register init values */
    error = mxt_check_reg_init(data);
    if (error)
    {
        TOUCH_ERR("Error %d initialising configuration\n",
                error);
        return error;
    }

    if (data->T9_reportid_min)
    {
        error = mxt_initialize_t9_input_device(data);
        if (error)
            return error;
    }
    else if (data->T100_reportid_min)
    {
        error = mxt_initialize_t100_input_device(data);
        if (error)
            return error;
    }
    else
    {
        TOUCH_WARN("No touch object detected\n");
    }

    data->enable_reporting = true;
    return 0;
}

static int mxt_configure_objects_show(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;

//		printk("Toby test > mxt configure objects.\n");
    /*
        error = mxt_debug_msg_init(data);
        if (error)
            return error;

        error = mxt_init_t7_power_cfg(data);
        if (error)
        {
            dev_err(&client->dev, "Failed to initialize power cfg\n");
            return error;
        }
    */

    /* Check register init values */
    error = mxt_check_reg_init(data);
    if (error)
    {
        TOUCH_ERR("Error %d initialising configuration\n",
                error);
        return error;
    }

    if (data->T9_reportid_min)
    {
        error = mxt_initialize_t9_input_device(data);
        if (error)
            return error;
    }
    else if (data->T100_reportid_min)
    {
        error = mxt_initialize_t100_input_device(data);
        if (error)
            return error;
    }
    else
    {
        TOUCH_WARN("No touch object detected\n");
    }

    data->enable_reporting = true;
    return 0;
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
                     data->info->version >> 4, data->info->version & 0xf,
                     data->info->build);
}

/*Config version is returned by this function*/
static ssize_t mxt_cfg_version_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    int error;
    int version_int=0;
    u8 obuf[2];
    struct mxt_data *data = dev_get_drvdata(dev);

    error = __mxt_read_reg(data->client, data->T38_address, 2, &obuf);
    if (error)
        return scnprintf(buf, PAGE_SIZE, "read register error:%d\n", error);

    version_int = obuf[0] * 10 + obuf[1];
    return scnprintf(buf, PAGE_SIZE, "the config version:%d\n", version_int);
}

/* GPIO Status is returned by this function */
static ssize_t mxt_gpio_status_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    int error;
    int timeout_counter = 0;
    int val = MXT_TP_SHOW;
    struct mxt_data *data = dev_get_drvdata(dev);

    data->get_gpio_status = false;

    error = __mxt_write_reg(data->client, data->T19_address,1,&val);
    if (error)
        return error;

    do
    {
        msleep(50);
    }
    while ((data->get_gpio_status == false) && (timeout_counter++ <= 30));

    if (timeout_counter > 30)
    {
        TOUCH_ERR("Command failed!\n");
        return -EIO;
    }

    if(data->gpio_status == 0x3C)
    {
        return scnprintf(buf, PAGE_SIZE, "New JTouch.\n");
    }
    else if(data->gpio_status == 0x3D)
    {
        return scnprintf(buf, PAGE_SIZE, "JTouch.\n");
    }
    else if(data->gpio_status == 0x3E)
    {
        return scnprintf(buf, PAGE_SIZE, "OFilm.\n");
    }
    else if(data->gpio_status == 0x3F)
    {
         return scnprintf(buf, PAGE_SIZE, "GIS.\n");
    }
    else return scnprintf(buf, PAGE_SIZE, "Touch Panle type unsure, ID : 0x%02X\n",data->gpio_status);

}

/* Glove enable by eric 3/24*/
static int mxt_glove_enable( struct mxt_data *data )
{
    int error;
    u8 val;

    /* Read T78 */
    error = __mxt_read_reg(data->client, data->T78_address, 1, &val);
    if (error)
    {
        TOUCH_ERR("Read T78 object error\n");
        return error;
    }

    val = val | 0x01; //set glove enable

    error = __mxt_write_reg(data->client, data->T78_address, 1, &val);
    if (error)
    {
        TOUCH_ERR("set glove enable error\n");
        return error;
    }

    return 0;
}

/* Glove disable by eric 3/24*/
static int mxt_glove_disable( struct mxt_data *data )
{
    int error;
    u8 val;

    /* Read T78 */
    error = __mxt_read_reg(data->client, data->T78_address, 1, &val);
    if (error)
    {
        TOUCH_ERR("Read T78 object error\n");
        return error;
    }

    val = val & 0xfe; //set glove disable

    error = __mxt_write_reg(data->client, data->T78_address, 1, &val);
    if (error)
    {
        TOUCH_ERR("set glove disable error\n");
        return error;
    }

    return 0;
}

/* Glove control by red_zhang@asus.com 3/24*/
static ssize_t mxt_glove_contorl(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int command;
    int error;

    command = simple_strtoul(buf, NULL, 10);
    TOUCH_DEBUG("command = %d\n", command);

    if(command == 0)
    {
        error = mxt_glove_disable(data);
        if(!error)
            return scnprintf(buf, PAGE_SIZE, "disable success!\n");
        return scnprintf(buf, PAGE_SIZE, "disable error,error number : %d\n",error);

    }
    else if(command == 1)
    {
        error = mxt_glove_enable(data);
        if(!error)
            return scnprintf(buf, PAGE_SIZE, "enable success!\n");
        return scnprintf(buf, PAGE_SIZE, "enable error,error number : %d\n",error);

    }
    else
    {
        return scnprintf(buf, PAGE_SIZE, "error command\n");
    }
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
                     data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
                                 struct mxt_object *object, int instance,
                                 const u8 *val)
{
    int i;

    if (mxt_obj_instances(object) > 1)
        count += scnprintf(buf + count, PAGE_SIZE - count,
                           "Instance %u\n", instance);

    for (i = 0; i < mxt_obj_size(object); i++)
        count += scnprintf(buf + count, PAGE_SIZE - count,
                           "\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
    count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

    return count;
}

static ssize_t mxt_object_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    struct mxt_object *object;
    int count = 0;
    int i, j;
    int error;
    u8 *obuf;

    /* Pre-allocate buffer large enough to hold max sized object. */
    obuf = kmalloc(256, GFP_KERNEL);
    if (!obuf)
        return -ENOMEM;

    error = 0;
    for (i = 0; i < data->info->object_num; i++)
    {
        object = data->object_table + i;

        if (!mxt_object_readable(object->type))
            continue;

        count += scnprintf(buf + count, PAGE_SIZE - count,
                           "T%u:\n", object->type);

        for (j = 0; j < mxt_obj_instances(object); j++)
        {
            u16 size = mxt_obj_size(object);
            u16 addr = object->start_address + j * size;

            error = __mxt_read_reg(data->client, addr, size, obuf);
            if (error)
                goto done;

            count = mxt_show_instance(buf, count, object, j, obuf);
        }
    }

done:
    kfree(obuf);
    return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
                                     const struct firmware *fw)
{
    unsigned int pos = 0;
    char c;

    while (pos < fw->size)
    {
        c = *(fw->data + pos);

        if (c < '0' || (c > '9' && c < 'A') || c > 'F')
            return 0;

        pos++;
    }

    /* To convert file try
     * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw */
    TOUCH_ERR("Aborting: firmware file must be in binary format\n");

    return -1;
}

static int mxt_load_fw(struct device *dev)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    const struct firmware *fw = NULL;
    unsigned int frame_size;
    unsigned int pos = 0;
    unsigned int retry = 0;
    unsigned int frame = 0;
    int ret;

    TOUCH_NOTICE("mxt_load_fw \n");
    ret = request_firmware(&fw, data->fw_name, dev);
    if (ret)
    {
        TOUCH_ERR("Unable to open firmware %s\n", data->fw_name);
        TOUCH_NOTICE("The firmware update error(1) \n");
        return ret;
    }

    /* Check for incorrect enc file */
    ret = mxt_check_firmware_format(dev, fw);
    if (ret)
    {
        TOUCH_NOTICE("The firmware update error(2) \n");
        goto release_firmware;
    }

    if (data->suspended)
    {
        if (data->use_regulator)
            mxt_regulator_enable(data);

        enable_irq(data->irq);
        mxt_control_irq(data, true);
        data->suspended = false;
    }

    if (!data->in_bootloader)
    {
        TOUCH_NOTICE("mxt_load_fw,in_bootloader=%d  \n",data->in_bootloader);
        /* Change to the bootloader mode */
        data->in_bootloader = true;

        ret = mxt_t6_command(data, MXT_COMMAND_RESET,
                             MXT_BOOT_VALUE, false);
        if (ret)
        {
            TOUCH_NOTICE("The firmware update error(3) \n");
            goto release_firmware;
        }

        msleep(MXT_RESET_TIME);

        /* At this stage, do not need to scan since we know
         * family ID */
        ret = mxt_lookup_bootloader_address(data, 0);
        if (ret)
        {
            TOUCH_NOTICE("The firmware update error(4) \n");
            goto release_firmware;
        }
    }
    else
    {
        enable_irq(data->irq);
        mxt_control_irq(data, true);
    }

    mxt_free_object_table(data);
    INIT_COMPLETION(data->bl_completion);

    ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD, false);
    if (ret)
    {
        /* Bootloader may still be unlocked from previous update
         * attempt */
        ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, false);
        if (ret)
        {
            TOUCH_NOTICE("The firmware update error(5) \n");
            goto disable_irq;
        }
    }
    else
    {
        TOUCH_NOTICE("Unlocking bootloader\n");
        /* Unlock bootloader */
        ret = mxt_send_bootloader_cmd(data, true);
        if (ret)
        {
            TOUCH_NOTICE("The firmware update error(6) \n");
            goto disable_irq;
        }
    }

    print_log = false;

    while (pos < fw->size)
    {
        ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA, true);
        if (ret)
        {
            TOUCH_NOTICE("The firmware update error(7) \n");
            goto disable_irq;
        }

        frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

        /* Take account of CRC bytes */
        frame_size += 2;

        /* Write one frame to device */
        ret = mxt_bootloader_write(data, fw->data + pos, frame_size);
        if (ret)
        {
            TOUCH_NOTICE("The firmware update error(8) \n");
            goto disable_irq;
        }

        ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS, true);
        if (ret)
        {
            retry++;

            /* Back off by 20ms per retry */
            msleep(retry * 20);

            if (retry > 20)
            {
                TOUCH_NOTICE("The firmware update error(9) \n");
                TOUCH_ERR("Retry count exceeded\n");
                goto disable_irq;
            }
        }
        else
        {
            retry = 0;
            pos += frame_size;
            frame++;
        }

        if (frame % 50 == 0)
            TOUCH_NOTICE("Sent %d frames, %d/%zd bytes\n",
                     frame, pos, fw->size);
    }

    print_log = true;

    /* Wait for flash. */
    ret = mxt_wait_for_completion(data, &data->bl_completion,
                                  MXT_FW_RESET_TIME);
    if (ret)
        goto disable_irq;

    TOUCH_NOTICE("Sent %d frames, %zd bytes\n", frame, pos);

    /* Wait for device to reset. Some bootloader versions do not assert
     * the CHG line after bootloading has finished, so ignore error */
    mxt_wait_for_completion(data, &data->bl_completion,
                            MXT_FW_RESET_TIME);

    data->in_bootloader = false;
    //restart machine if flash firmware
    //reboot = true;
    TOUCH_NOTICE("The firmware update ok \n");

disable_irq:
    disable_irq(data->irq);
    mxt_control_irq(data, false);
release_firmware:
    release_firmware(fw);
    return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
                                const char *buf, size_t count)
{
    size_t size;
    /* Simple sanity check */
    if (count > 64)
    {
        TOUCH_WARN("File name too long\n");
        return -EINVAL;
    }

    size = sizeof(size_t)*count;
    *file_name = kzalloc(size, GFP_KERNEL);
    if (!*file_name)
    {
        TOUCH_WARN("Failed to allocate memory to fw update\n");
        goto update_failed;
    }

    memcpy(*file_name, buf, size);
    TOUCH_NOTICE("file_name=%s \n",*file_name);

    /* Echo into the sysfs entry may append newline at the end of buf */
    if (buf[count - 1] == '\n')
    {
        (*file_name)[count - 1] = '\0';
    }
    else
    {
        (*file_name)[count] = '\0';
    }

update_failed:
    return 0;
}

static ssize_t mxt_update_fw_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int error;
    TOUCH_NOTICE("mxt_update_fw_store start! \n");
    error = mxt_update_file_name(dev, &data->fw_name, buf, count);
    if (error)
        return error;
    TOUCH_NOTICE("mxt_update_fw_store, fw_name=%s \n",data->fw_name);

    error = mxt_load_fw(dev);
    if (error)
    {
       TOUCH_ERR("The firmware update failed(%d)\n", error);
        count = error;
    }
    else
    {
        data->suspended = false;
        mxt_free_object_table(data);
        unregister_early_suspend(&data->early_suspend);
        mxt_soft_reset(data);
        error = red_mxt_initialize(data);
        if (error)
            return error;
    }

    return count;
}

static ssize_t mxt_update_cfg_store(struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int ret;
    int error;
    char cfg_file_name[128];

    if (buf[0] == '1') //force update config
    {
        cfg_force = true;
        TOUCH_NOTICE("Enter 1, cfg_force = %d\n",cfg_force);
    }
    else if (buf[0] == '0')  //normal update config, check info_crc and config_crc before update config
    {
        cfg_force = false;
        TOUCH_NOTICE("Enter 0, cfg_force = %d\n",cfg_force);
    }
    else
        TOUCH_NOTICE("Default cfg_force = %d\n",cfg_force);

    if (data->in_bootloader)
    {
        TOUCH_ERR("Not in appmode\n");
        return -EINVAL;
    }

    // parse the file name
    snprintf(cfg_file_name, count-2, "%s", &buf[2]);

    TOUCH_NOTICE("buf=%s, cfg_file_name=%s , cfg_force=%d \n", buf, cfg_file_name, cfg_force);

    ret = mxt_update_file_name(dev, &data->cfg_name, cfg_file_name, count);
    if (ret)
        return ret;

    TOUCH_NOTICE("mxt_update_cfg_store, cfg_name=%s \n",data->cfg_name);

    data->enable_reporting = false;
    //mxt_free_input_device(data);

    if (data->suspended)
    {
        if (data->use_regulator)
        {
            mxt_regulator_enable(data);
        }
        else
        {
            mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
        }

        mxt_acquire_irq(data);

        data->suspended = false;
    }

    ret = mxt_configure_objects_show(data);
    if (ret)
    {
        TOUCH_NOTICE("Update cfg failed! ret=%d \n",ret);
        goto out;
    }

    /* Parse object table information */
    ret = mxt_parse_object_table(data);
    if (ret)
    {
        TOUCH_ERR("Error %d reading object table\n", ret);
        goto out;
    }

    ret = count;

out:
    return ret;
}

static ssize_t mxt_update_cfg_force_store(struct device *dev,
        struct device_attribute *attr,const char *buf, size_t count)
{

    if (buf[0] == '1') //force update config
    {
        cfg_force = true;
        TOUCH_NOTICE("Enter f, cfg_force = %d\n", cfg_force);
    }
    else if (buf[0] == '0')  //normal update config, check info_crc and config_crc before update config
    {
        cfg_force = false;
        TOUCH_NOTICE("Enter k, cfg_force = %d\n", cfg_force);
    }
    else
    {
        TOUCH_NOTICE("Please enter 1(force update config) or 0(normal update config) \n");
    }

    return count;
}

static ssize_t mxt_update_cfg_force_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    size_t count = 0;
    u8 status;

    status = cfg_force;

    switch(status)
    {
    case 1:
        count += sprintf(buf, "Force Update Config,cfg_force=%d\n",cfg_force);
        break;
    case 0:
        count += sprintf(buf, "Normal Update Config,cfg_force=%d\n",cfg_force);
        break;
    }

    return count;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    char c;

    c = data->debug_enabled ? '1' : '0';
    return scnprintf(buf, PAGE_SIZE, "%c\n", c);
}

static ssize_t mxt_debug_notify_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "0\n");
}

static ssize_t mxt_debug_v2_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int i;

    if (sscanf(buf, "%u", &i) == 1 && i < 2)
    {
        if (i == 1)
            mxt_debug_msg_enable(data);
        else
            mxt_debug_msg_disable(data);

        return count;
    }
    else
    {
        TOUCH_DEBUG("debug_enabled write error\n");
        return -EINVAL;
    }
}

static ssize_t mxt_debug_enable_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int i;

    if (sscanf(buf, "%u", &i) == 1 && i < 2)
    {
        data->debug_enabled = (i == 1);

        TOUCH_DEBUG("%s\n", i ? "debug enabled" : "debug disabled");
        return count;
    }
    else
    {
        TOUCH_DEBUG("debug_enabled write error\n");
        return -EINVAL;
    }
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
                                       size_t *count)
{
    if (off >= data->mem_size)
        return -EIO;

    if (off + *count > data->mem_size)
        *count = data->mem_size - off;

    if (*count > MXT_MAX_BLOCK_WRITE)
        *count = MXT_MAX_BLOCK_WRITE;

    return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
                                   struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mxt_data *data = dev_get_drvdata(dev);
    int ret = 0;

    ret = mxt_check_mem_access_params(data, off, &count);
    if (ret < 0)
        return ret;

    if (count > 0)
        ret = __mxt_read_reg(data->client, off, count, buf);

    return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
                                    struct bin_attribute *bin_attr, char *buf, loff_t off,
                                    size_t count)
{
    struct device *dev = container_of(kobj, struct device, kobj);
    struct mxt_data *data = dev_get_drvdata(dev);
    int ret = 0;

    ret = mxt_check_mem_access_params(data, off, &count);
    if (ret < 0)
        return ret;

    if (count > 0)
        ret = __mxt_write_reg(data->client, off, count, buf);

    return ret == 0 ? count : 0;
}


static ssize_t mxt_self_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    size_t count = 0;
    u8 status;

    status = data->t25_self_test_status;

    switch(status)
    {
    case MXT_T25_ALL_TEST_PASS:
        count += sprintf(buf, "ALL TEST PASS\n");
        break;
    case MXT_T25_CMD_FAULT:
        count += sprintf(buf, "CMD FAULT\n");
        break;
    case MXT_T25_UNRELATED_FAULT:
        count += sprintf(buf, "UNRELATED FAULT\n");
        break;
    case MXT_T25_AVDD_FAULT:
        count += sprintf(buf, "AVDD FAULT\n");
        break;
    case MXT_T25_PIN_FAULT:
        count += sprintf(buf, "PIN FAULT, Byte2: 0x%02X, Byte3: 0x%02X, Byte4: 0x%02X\n",
                         data->t25_ref_byte2,
                         data->t25_ref_byte3,
                         data->t25_ref_byte4);
        break;
    case MXT_T25_SIGNAL_LIMIT_FAULT:
        count += sprintf(buf, "ALL SIGNAL LIMIT FAULT, Byte2: 0x%02X, Byte3: 0x%02X\n",
                         data->t25_ref_byte2,
                         data->t25_ref_byte3);
        break;
    }

    return count;
}

static u32 mxt_proc_self_test(struct mxt_data *data, u32 order, u32 value)
{
    int error;

    switch(order)
    {
    case test_cmd:
	TOUCH_ERR("test_cmd=%d\n",test_cmd);
        error = __mxt_write_reg(data->client, data->T25_address + 1, 1, &value);
        if (error)
            return error;
        break;
    case high_limit:
	TOUCH_ERR("high_limit=%d\n",value);
        error = __mxt_write_reg(data->client, data->T25_address + 2, 2, &value);
        if (error)
            return error;
        break;
    case low_limit:
	TOUCH_ERR("low_limit=%d\n",value);
        error = __mxt_write_reg(data->client, data->T25_address + 4, 2, &value);
        if (error)
            return error;
        break;
    case diff_limit:
	TOUCH_ERR("diff_limit=%d\n",value);
        error = __mxt_write_reg(data->client, data->T25_address + 15, 2, &value);
        if (error)
            return error;
        break;
    }
    return error;
}

static u32 set_selftest_limit(struct mxt_data *data)
{
    int error;

    if(data->gpio_status == 0x3E)
    {
          //TP is O-Film
          error = mxt_proc_self_test(data, 1, 24500);
          if (error)
          {
              TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
              return error;
          }

          error = mxt_proc_self_test(data, 2, 20500);
          if (error)
          {
              TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
              return error;
          }

          error = mxt_proc_self_test(data, 3, 1800);
          if (error)
          {
              TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
              return error;
          }

          TOUCH_ERR("Set OFilm threshold.\n");

          return 0;
    } 
    else if((data->gpio_status == 0x3C)||(data->gpio_status == 0x3D))
    {
          //TP is J-Touch in IC revision D
          error = mxt_proc_self_test(data, 1, 24000);
          if (error)
          {
              TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
              return error;
          }

          error = mxt_proc_self_test(data, 2, 20000);
          if (error)
          {
              TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
              return error;
          }

          error = mxt_proc_self_test(data, 3, 1600);
          if (error)
          {
              TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
              return error;
          }

          TOUCH_ERR("Set JTouch/New JTouch threshold.\n");
          return 0;
    }
    else if(data->gpio_status == 0x3F)
    {
        //TP is GIS
        error = mxt_proc_self_test(data, 1, 24000);
        if (error)
        {
            TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
            return error;
        }

        error = mxt_proc_self_test(data, 2, 20000);
        if (error)
        {
            TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
            return error;
        }

        error = mxt_proc_self_test(data, 3, 1600);
        if (error)
        {
            TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
            return error;
        }

        TOUCH_ERR("Set GIS threshold.\n");
        return 0;
    }
    else
    {
        TOUCH_ERR("Can not identify TP.\n");
        return -1;
    }

}

static ssize_t mxt_self_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int error;
    char *value;
    char delim[] = "-";
    char *tbuf;
    u32 threshold = 0;
    u32 order = 0;
    u32 test_cmd = 0;
    u8 t25_byte0 = 0;
    u8 enable_self_test_cmd = 0;
    bool set_limit=true;

    tbuf = buf;

    if (!buf)
    {
        TOUCH_ERR("Miss self test value, please enter value follow spec: test_cmd-high_limit-low_limit-diff_limit.\n");
        return -ENOMEM;
    }

    error = __mxt_read_reg(data->client, data->T25_address, 1, &t25_byte0);
    if (error)
        TOUCH_WARN("Read T25 register have problem!\n");

    /* Make sure self test is enable */
    if (!(t25_byte0 & MXT_T25_RPTEN && t25_byte0 & MXT_T25_ENABLE))
    {
        enable_self_test_cmd = 0x03;
        error = __mxt_write_reg(data->client, data->T25_address, 1, &enable_self_test_cmd);
        if (error)
            return error;
        TOUCH_NOTICE("Enable T25 register byte0 RPTEN = %u, ENABLE = %u.\n", t25_byte0 & MXT_T25_RPTEN, t25_byte0 & MXT_T25_ENABLE);
    }

    /* Order: 0.Test cmd, 1.High limit, 2.Low limit, 3.Diff limit
       For example, input must looks like "0xfe-24500-20500-1400". */
    for(value = strsep(&tbuf, delim); value != NULL; value = strsep(&tbuf, delim))
    {
        if(order == 0)
        {
            test_cmd = simple_strtoul(value, NULL, 16);
            TOUCH_NOTICE("Get self test cmd = 0x%x\n", test_cmd);
        }
        else if (order > 0)
        {
            set_limit = false;
            threshold = simple_strtoul(value, NULL, 10);
            TOUCH_NOTICE("Lunch test threshold = %d\n", threshold);
            //TODO Wait for TP Vendor give limits after ER stage
            error = mxt_proc_self_test(data, order, threshold);
            if (error)
            {
                TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
                return error;
            }
        }
        order += 1;
    }

    if(set_limit)
    {
        //Use default threshold
        error = set_selftest_limit(data);
        if(error)
        {
            TOUCH_ERR("Set selftest threshold error.\n");
	    return error;
        }
    }

    error = mxt_proc_self_test(data, 0, test_cmd);
    TOUCH_NOTICE("Lunch self test cmd = 0x%x\n", test_cmd);
    if (error)
    {
        TOUCH_ERR("Send threshold to i2c for self test have problem!\n");
        return error;
    }

    return count;
}

static ssize_t mxt_touch_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int ret;

    ret = mxt_t6_command(data, MXT_COMMAND_REPORTALL, 1, true);
    return sprintf(buf, "%s\n", (ret == 0) ? "PASS" : "FAIL");

}

static ssize_t mxt_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    size_t count = 0;

    count += sprintf(buf, "Status: %s, ", (data->irq_status == true && data->irq_count >= 0) ? "Enable" : "Disable");
    if (data->irq_count > 0)
    {
        count += sprintf(buf + count , "IRQ Overflow, ");
    }
    count += sprintf(buf + count , "Count: %d\n", data->irq_count);

    return count;
}

static ssize_t mxt_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);

    if (buf[0] == '0')
    {
        disable_irq(data->irq);
        mxt_control_irq(data, false);
        TOUCH_NOTICE("Disable irq success!\n");
    }
    else
    {
        enable_irq(data->irq);
        mxt_control_irq(data, true);
        TOUCH_NOTICE("Enable irq success!\n");
    }
    return count;
}

static ssize_t mxt_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int ret;

    ret = mxt_soft_reset(data);
    if (ret)
    {
        TOUCH_NOTICE("Reset Touch IC Failed \n");
    }
    TOUCH_NOTICE("Reset Touch IC \n");

    return count;
}

static ssize_t mxt_ic_revision_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);

    if(data->ic_revision == MXT_T37_IC_REV_C)
    {
        return scnprintf(buf, PAGE_SIZE, "the ic revision is C \n");
    }
    else if(data->ic_revision == MXT_T37_IC_REV_D)
    {
        return scnprintf(buf, PAGE_SIZE, "the ic revision is D \n");
    }
    else
    {
        return scnprintf(buf, PAGE_SIZE, "the ic revision can not identify \n");
    }
}

static int mxt_cfg_name_crc_show(struct device *dev,struct device_attribute *attr, char *buf)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "config name = %s, crc = 0x%06X \n", data->cfg_name, data->config_crc);
}

static ssize_t mxt_hw_reset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct mxt_data *data = dev_get_drvdata(dev);
    int ret;

    gpio_set_value(data->pdata->gpio_reset, 0);
    msleep(MXT_REGULATOR_DELAY);
    gpio_set_value(data->pdata->gpio_reset, 1);
    msleep(90);

    TOUCH_NOTICE("HW reset success!\n");
    return count;
}

static DEVICE_ATTR(fw_version, S_IRUGO, mxt_fw_version_show, NULL);
static DEVICE_ATTR(cfg_version, S_IRUGO,mxt_cfg_version_show, NULL);
static DEVICE_ATTR(gpio_status, S_IRUGO, mxt_gpio_status_show, NULL);	//add for TP_SHOW
static DEVICE_ATTR(glove_control, S_IWUSR, NULL, mxt_glove_contorl); //add for glove mode control
static DEVICE_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static DEVICE_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static DEVICE_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static DEVICE_ATTR(update_cfg_force, S_IRUGO | S_IWUSR | S_IWGRP, mxt_update_cfg_force_show, mxt_update_cfg_force_store);
static DEVICE_ATTR(debug_v2_enable, S_IWUSR | S_IRUSR, NULL, mxt_debug_v2_enable_store);
static DEVICE_ATTR(debug_notify, S_IRUGO, mxt_debug_notify_show, NULL);
static DEVICE_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show, mxt_debug_enable_store);
static DEVICE_ATTR(touch_self_test, S_IRUGO | S_IWUSR | S_IWGRP, mxt_self_test_show, mxt_self_test_store);
static DEVICE_ATTR(touch_status, S_IRUGO, mxt_touch_status, NULL);
static DEVICE_ATTR(touch_irq, S_IRUGO | S_IWUSR | S_IWGRP, mxt_irq_show, mxt_irq_store);
static DEVICE_ATTR(reset, (S_IWUSR|S_IRUGO|S_IWGRP),NULL, mxt_reset_store);
static DEVICE_ATTR(touch_ic_revision,S_IRUGO,mxt_ic_revision_show, NULL);
static DEVICE_ATTR(cfg_name_crc,S_IRUGO,mxt_cfg_name_crc_show, NULL);
static DEVICE_ATTR(hw_reset, (S_IWUSR|S_IRUGO|S_IWGRP),NULL, mxt_hw_reset_store);

static struct attribute *mxt_attrs[] =
{
    &dev_attr_fw_version.attr,
    &dev_attr_cfg_version.attr,
    &dev_attr_gpio_status.attr, //add for TP_SHOW
    &dev_attr_glove_control.attr,	//add for glove mode control
    &dev_attr_hw_version.attr,
    &dev_attr_object.attr,
    &dev_attr_update_fw.attr,
    &dev_attr_update_cfg.attr,
    &dev_attr_update_cfg_force.attr,
    &dev_attr_debug_enable.attr,
    &dev_attr_debug_v2_enable.attr,
    &dev_attr_debug_notify.attr,
    &dev_attr_touch_self_test.attr,//add for self test by eric
    &dev_attr_touch_status.attr,
    &dev_attr_touch_irq.attr,
    &dev_attr_reset.attr,
    &dev_attr_touch_ic_revision.attr,
    &dev_attr_cfg_name_crc.attr,
    &dev_attr_hw_reset.attr,
    NULL
};

static const struct attribute_group mxt_attr_group =
{
    .attrs = mxt_attrs,
};

static void mxt_reset_slots(struct mxt_data *data)
{
    struct input_dev *input_dev = data->input_dev;
    unsigned int num_mt_slots;
    int id;

    num_mt_slots = data->num_touchids + data->num_stylusids;

    for (id = 0; id < num_mt_slots; id++)
    {
        input_mt_slot(input_dev, id);
        input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
    }

    mxt_input_sync(input_dev);
}

static void mxt_start(struct mxt_data *data)
{
    if (!data->suspended || data->in_bootloader)
        return;

//		printk("<Red_debug>enter mxt_start\n");
    if (data->use_regulator)
    {
        mxt_regulator_enable(data);
    }
    else
    {
        /* Discard any messages still in message buffer from before
         * chip went to sleep */
        mxt_process_messages_until_invalid(data);

        mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

        /* Recalibrate since chip has been in deep sleep */
        mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
    }

    mxt_acquire_irq(data);
    data->enable_reporting = true;
    data->suspended = false;
}

static void mxt_stop(struct mxt_data *data)
{
    if (data->suspended || data->in_bootloader)
        return;

//		printk("<Red_debug>enter mxt_stop\n");
    data->enable_reporting = false;
    disable_irq(data->irq);
    mxt_control_irq(data, false);

    if (data->use_regulator)
        mxt_regulator_disable(data);
    else
        mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);

    mxt_reset_slots(data);
    data->suspended = true;
}

static int mxt_input_open(struct input_dev *dev)
{
    struct mxt_data *data = input_get_drvdata(dev);

    mxt_start(data);

    return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
    struct mxt_data *data = input_get_drvdata(dev);

    mxt_stop(data);
}

static int mxt_handle_pdata(struct mxt_data *data)
{
//        data->pdata = dev_get_platdata(&data->client->dev);

    /* Use provided platform data if present */
    /*
    if (data->pdata) {
            if (data->pdata->cfg_name){
                    mxt_update_file_name(&data->client->dev,
                                         &data->cfg_name,
                                         data->pdata->cfg_name,
                                         strlen(data->pdata->cfg_name));
    		}

            return 0;
    }	*/

    data->pdata = kzalloc(sizeof(*data->pdata), GFP_KERNEL);
    if (!data->pdata)
    {
        TOUCH_ERR("Failed to allocate pdata\n");
        return -ENOMEM;
    }

    /* Set default parameters */
    data->pdata->irqflags = IRQF_TRIGGER_FALLING;
    data->pdata->gpio_reset = 60;
    return 0;
}

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    const struct mxt_platform_data *pdata = data->pdata;
    struct input_dev *input_dev;
    int error;
    unsigned int num_mt_slots;
    int i;

    error = mxt_read_t9_resolution(data);
    if (error)
        TOUCH_WARN("Failed to initialize T9 resolution\n");

    input_dev = input_allocate_device();
    if (!input_dev)
    {
        TOUCH_ERR("Failed to allocate memory\n");
        return -ENOMEM;
    }

    input_dev->name = "atmel_mxt_ts";
    input_dev->phys = data->phys;
    input_dev->id.bustype = BUS_I2C;
    input_dev->dev.parent = dev;
    input_dev->open = mxt_input_open;
    input_dev->close = mxt_input_close;

    __set_bit(EV_ABS, input_dev->evbit);
    input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

    if (pdata->t19_num_keys)
    {
        __set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

        for (i = 0; i < pdata->t19_num_keys; i++)
            if (pdata->t19_keymap[i] != KEY_RESERVED)
                input_set_capability(input_dev, EV_KEY,
                                     pdata->t19_keymap[i]);

        __set_bit(BTN_TOOL_FINGER, input_dev->keybit);
        __set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
        __set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
        __set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);

        input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
        input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
        input_abs_set_res(input_dev, ABS_MT_POSITION_X,
                          MXT_PIXELS_PER_MM);
        input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
                          MXT_PIXELS_PER_MM);

        input_dev->name = "atmel_mxt_ts";
    }

    /* For single touch */
    input_set_abs_params(input_dev, ABS_X,
                         0, data->max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_Y,
                         0, data->max_y, 0, 0);
    input_set_abs_params(input_dev, ABS_PRESSURE,
                         0, 255, 0, 0);

    /* For multi touch */
    num_mt_slots = data->num_touchids + data->num_stylusids;
    error = input_mt_init_slots(input_dev, num_mt_slots,0);
    if (error)
    {
        TOUCH_ERR("Error %d initialising slots\n", error);
        goto err_free_mem;
    }

    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
                         0, MXT_MAX_AREA, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X,
                         0, data->max_x, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
                         0, data->max_y, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_PRESSURE,
                         0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
                         0, 255, 0, 0);

    /* For T63 active stylus */
    if (data->T63_reportid_min)
    {
        input_set_capability(input_dev, EV_KEY, BTN_STYLUS);
        input_set_capability(input_dev, EV_KEY, BTN_STYLUS2);
        input_set_abs_params(input_dev, ABS_MT_TOOL_TYPE,
                             0, MT_TOOL_MAX, 0, 0);
    }

    /* For T15 key array */
    if (data->T15_reportid_min)
    {
        data->t15_keystatus = 0;

        for (i = 0; i < data->pdata->t15_num_keys; i++)
            input_set_capability(input_dev, EV_KEY,
                                 data->pdata->t15_keymap[i]);
    }

    input_set_drvdata(input_dev, data);

    error = input_register_device(input_dev);
    if (error)
    {
        TOUCH_ERR("Error %d registering input device\n", error);
        goto err_free_mem;
    }

    data->input_dev = input_dev;

    return 0;

err_free_mem:
    input_free_device(input_dev);
    return error;
}

/*add by red_zhang@asus.com*/
//===============================================
static int red_mxt_configure_objects(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;

//		printk("Toby test > mxt configure objects.\n");
    error = mxt_debug_msg_init(data);
    if (error)
        return error;

    error = mxt_init_t7_power_cfg(data);
    if (error)
    {
        TOUCH_ERR("Failed to initialize power cfg\n");
        return error;
    }

    /*1.init the first input dev*/
    if (data->T9_reportid_min)
    {
        error = mxt_initialize_t9_input_device(data);
        if (error)
            return error;
    }
    else if (data->T100_reportid_min)
    {
        error = mxt_initialize_t100_input_device(data);
        if (error)
            return error;
    }
    else
    {
        TOUCH_WARN("No touch object detected\n");
    }
    enable_irq(data->irq);
    mxt_control_irq(data, true);
    error = red_mxt_cfg_version(data);
    if(error)
	TOUCH_ERR("config version read fail or config is zero\n");
    TOUCH_NOTICE("Device config_version is %d now\n", data->cfg_version);

    error = mxt_tp_id_show(&client->dev);
    if (error)
    {
        TOUCH_ERR("read tp id error");
        goto config_setting;
    }

    if(data->ic_revision == MXT_T37_IC_REV_C)
    {
        TOUCH_NOTICE("%s : Touch IC is revision C. \n",__func__);
        if(data->gpio_status == 0x3C)
        {
            data->cfg_name = MXT_CFG_T2_JT_F;
            TOUCH_NOTICE("tp is Jtouch final, gpio_status show: 0x%02X\n",data->gpio_status);
        }
        else if (data->gpio_status == 0x3D)
        {
            data->cfg_name = MXT_CFG_T2_JT;
            TOUCH_NOTICE("tp is Jtouch, gpio_status show: 0x%02X\n",data->gpio_status);
        }
        else if (data->gpio_status == 0x3E)
        {
            data->cfg_name = MXT_CFG_T2_OF;
            TOUCH_NOTICE("pin_id is Ofilm, gpio_status show: 0x%02X\n",data->gpio_status);
        }
	else if (data->gpio_status == 0x3F)
	{
            data->cfg_name = MXT_CFG_T2_OF;
            TOUCH_NOTICE("tp is GIS, gpio_status show: 0x%02X\n",data->gpio_status);
        }
        else
        {
            data->cfg_name = MXT_CFG_T2_JT_F;
            TOUCH_NOTICE("Identify error tp vendor, use default config v37 JTouch final. \n");
        }
    }
    else if(data->ic_revision == MXT_T37_IC_REV_D)
    {
        TOUCH_NOTICE("%s : Touch IC is revision D. \n",__func__);
        if(data->gpio_status == 0x3C)
        {
            data->cfg_name = MXT_CFG_T2_REV_D_JT;
	    data->tp_id = TP_JTOUCH;
            TOUCH_NOTICE("tp is Jtouch final, gpio_status show: 0x%02X\n",data->gpio_status);
        }
        else if (data->gpio_status == 0x3D)
        {
            data->cfg_name = MXT_CFG_T2_REV_D_JT;
	    data->tp_id = TP_JTOUCH;
            TOUCH_NOTICE("tp is Jtouch, gpio_status show: 0x%02X\n",data->gpio_status);
        }
        else if (data->gpio_status == 0x3E)
        {
            data->cfg_name = MXT_CFG_T2_REV_D_OF;
	    data->tp_id = TP_OFILM;
            TOUCH_NOTICE("pin_id is Ofilm, gpio_status show: 0x%02X\n",data->gpio_status);
        }
        else if (data->gpio_status == 0x3F)
        {
            data->cfg_name = MXT_CFG_T2_REV_D_GIS;
	    data->tp_id = TP_GIS;
            TOUCH_NOTICE("tp is GIS, gpio_status show: 0x%02X\n",data->gpio_status);
	}
        else
        {
            data->cfg_name = MXT_CFG_T2_REV_D_JT;
	    data->tp_id = TP_JTOUCH;
            TOUCH_NOTICE("Identify error tp vendor, use default config v30 JTouch.\n");
        }
    }
    else
    {
        data->cfg_name = MXT_CFG_T2_REV_D_JT;
        TOUCH_NOTICE("%s : Identify error ic revision, use default config v30 JTouch.\n",__func__);
    }


    /*2.init the second input dev*/
    /* Upgrate config file */
config_setting :
//		if( data->mb_test &&(data->pin_id == 1 || data->pin_id == 0) )//&& (config_version <13) )
    TOUCH_NOTICE("cfg_name=%s \n",data->cfg_name);
    error = mxt_check_reg_init(data);
    if (error)
    {
        TOUCH_ERR("Error %d initialising configuration\n",
                error);
        return error;
    }

    //restart machine if flash firmware
    if(reboot)
        machine_restart("touch driver restart");

    disable_irq(data->irq);
    mxt_control_irq(data, false);
    mxt_free_input_device(data);

    if (data->T9_reportid_min)
    {
        error = mxt_initialize_t9_input_device(data);
        if (error)
            return error;
    }
    else if (data->T100_reportid_min)
    {
        error = mxt_initialize_t100_input_device(data);
        if (error)
            return error;
    }
    else
    {
        TOUCH_WARN("No touch object detected\n");
    }

    //add by red_zhang@asus.com
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    data->early_suspend.suspend = mxt_early_suspend;
    data->early_suspend.resume = mxt_late_resume;
    register_early_suspend(&data->early_suspend);

    data->enable_reporting = true;

    return 0;
}

static int mxt_tp_id_show(struct device *dev)
{
    int error;
    int timeout_counter = 0;
    int val = MXT_TP_SHOW;

    struct mxt_data *data = dev_get_drvdata(dev);

    data->get_gpio_status = false;

    val = 63;
    error = __mxt_write_reg(data->client, data->T19_address,1,&val);
    if(error)
        return error;

    val = 3;
    error = __mxt_write_reg(data->client, data->T19_address,1,&val);
    if(error)
        return error;

    val = MXT_TP_SHOW;

    error = __mxt_write_reg(data->client, data->T19_address,1,&val);
    if (error)
        return error;

    do
    {
        msleep(50);
    }
    while ((data->get_gpio_status == false) && (timeout_counter++ <= 30));

    if (timeout_counter > 30)
    {
        TOUCH_ERR("Command failed!\n");
        return -EIO;
    }

    return  0;
}

static int red_mxt_read_info_block(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    bool alt_bootloader_addr = false;
    bool retry = false;

retry_info:
    error = mxt_read_info_block(data);
    if (error)
    {
retry_bootloader:
        error = mxt_probe_bootloader(data, alt_bootloader_addr);
        if (error)
        {
            if (alt_bootloader_addr)
            {
                /* Chip is not in appmode or bootloader mode */
                TOUCH_NOTICE("Chip is not in appmode or bootloader mode\n");
                return error;
            }

            TOUCH_NOTICE("Trying alternate bootloader address\n");
            alt_bootloader_addr = true;
            goto retry_bootloader;
        }
        else
        {
            if (retry)
            {
                TOUCH_ERR(
                        "Could not recover device from "
                        "bootloader mode\n");
                /* this is not an error state, we can reflash
                 * from here */
                data->in_bootloader = true;
                return 1;
            }

            /* Attempt to exit bootloader into app mode */
            TOUCH_NOTICE("Attempt to exit bootloader into app mode\n");
            mxt_send_bootloader_cmd(data, false);
            msleep(MXT_FW_RESET_TIME);
            retry = true;
            goto retry_info;
        }
    }
    return 0;
}

static int red_mxt_initialize(struct mxt_data *data)
{
    struct i2c_client *client = data->client;
    int error;
    bool alt_bootloader_addr = false;
    bool retry = false;

retry_info:
    error = mxt_read_info_block(data);
    if (error)
    {
retry_bootloader:
        error = mxt_probe_bootloader(data, alt_bootloader_addr);
        if (error)
        {
            if (alt_bootloader_addr)
            {
                /* Chip is not in appmode or bootloader mode */
                return error;
            }

            TOUCH_NOTICE("Trying alternate bootloader address\n");
            alt_bootloader_addr = true;
            goto retry_bootloader;
        }
        else
        {
            if (retry)
            {
                TOUCH_ERR(
                        "Could not recover device from "
                        "bootloader mode\n");
                /* this is not an error state, we can reflash
                 * from here */
                data->in_bootloader = true;
                return 0;
            }

            /* Attempt to exit bootloader into app mode */
            mxt_send_bootloader_cmd(data, false);
            msleep(MXT_FW_RESET_TIME);
            retry = true;
            goto retry_info;
        }
    }

    error = mxt_check_retrigen(data);
    if (error)
        return error;

    if (data->use_retrigen_workaround)
    {
        error = mxt_process_messages_until_invalid(data);
        if (error)
            return error;
    }

    error = red_mxt_configure_objects(data);
    if (error)
        return error;

    TOUCH_NOTICE("Initialize end");

    enable_irq(data->irq);
    mxt_control_irq(data, true);
    return 0;
}

static int red_mxt_cfg_version( struct mxt_data *data)
{
    int error;
    int version_int=0;
    u8 obuf[2];

    error = __mxt_read_reg(data->client, data->T38_address, 2, &obuf);
    if (error)
        return -1;

    data->cfg_version = obuf[0] * 10 + obuf[1];

    return 0;
}

//===============================================
static int read_ic_revision(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    u8 revision_mode = 0x80;
    int error;
    u8 t37_byte21= 0;
    u8 t37_byte21_up= 0;

    error = __mxt_write_reg(data->client, data->T6_address + 5, 1, &revision_mode);
    if (error)
    {
        TOUCH_WARN("Write T6 register have problem!\n");
    }

    msleep(1000);

    error = __mxt_read_reg(data->client, data->T37_address + 21, 1, &t37_byte21);
    if (error)
    {
        TOUCH_WARN("Read T37 register have problem!\n");
    }

    t37_byte21_up = (t37_byte21>>4);

    if(t37_byte21_up==MXT_T37_IC_REV_C)
    {
        data->ic_revision = MXT_T37_IC_REV_C;
        TOUCH_NOTICE("%s : Touch IC is revision C",__func__);
        return 0;
    }
    else if (t37_byte21_up==MXT_T37_IC_REV_D)
    {
        data->ic_revision = MXT_T37_IC_REV_D;
        TOUCH_NOTICE("%s : Touch IC is revision D",__func__);
        return 0;
    }
    else
    {
        TOUCH_NOTICE("%s : Touch IC revision can not identify",__func__);
        return 1;
    }
}

static int mxt_read_cfg_tp_id(struct mxt_data *data)
{
    int error;
    u8 obuf;

    error = __mxt_read_reg(data->client, data->T38_address+2, 1, &obuf);
    if (error)
        TOUCH_WARN("Read T38 register have problem!\n");

    data->cfg_tp_id = obuf;
    TOUCH_NOTICE("Touch CFG TP ID : %d\n", data->cfg_tp_id);

    return error;
}


//**
//i2c stress test
//**
int mxt_open(struct inode *inode, struct file *filp)
{

    TOUCH_NOTICE("mxt_open\n");

    return 0;

}

int mxt_release(struct inode *inode, struct file *filp)
{

    TOUCH_NOTICE("mxt_release\n");

    return 0;

}

int mxt_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

    int err = 1;

    if (_IOC_TYPE(cmd) != MXT_IOC_MAGIC)
        return -ENOTTY;

    if (_IOC_NR(cmd) > MXT_IOC_MAXNR)
        return -ENOTTY;

    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));

    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

    if (err)
        return -EFAULT;

    switch (cmd)
    {

    case MXT_POLL_DATA:
        if (arg ==MXT_IOCTL_START_HEAVY)
        {
            TOUCH_NOTICE("ioctl heavy\n");
            poll_mode = START_HEAVY;
            queue_delayed_work(touch_work_queue, &mxt_poll_data_work, poll_mode);
        }
        else if (arg == MXT_IOCTL_START_NORMAL)
        {
            TOUCH_NOTICE("ioctl normal\n");
            poll_mode = START_NORMAL;
            queue_delayed_work(touch_work_queue, &mxt_poll_data_work, poll_mode);
        }
        else if  (arg ==MXT_IOCTL_END)
        {
            TOUCH_NOTICE("ioctl end\n");
            cancel_delayed_work_sync(&mxt_poll_data_work);
        }

        else
            return -ENOTTY;

        break;

    default: /* redundant, as cmd was checked against MAXNR */

        return -ENOTTY;

    }

    return 0;

}

struct file_operations mxt_fops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = mxt_ioctl,
    .open = mxt_open,
    .release =mxt_release,
};

static void  mxt_poll_data(struct work_struct * work)
{
    int ret;
    //struct mxt_data *data = dev_get_drvdata(dev);

    ret = mxt_t6_command(data, MXT_COMMAND_REPORTALL, 1, true);

    TOUCH_NOTICE("mxt_poll_data \n");

    if(poll_mode == 0)
        msleep(5);

    queue_delayed_work(touch_work_queue, &mxt_poll_data_work, poll_mode);
}

static int mxt_probe(struct i2c_client *client,
                     const struct i2c_device_id *id)
{
    //struct mxt_data *data;
    int error;

    data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
    if (!data)
    {
        TOUCH_ERR("Failed to allocate memory\n");
        return -ENOMEM;
    }

    snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
             client->adapter->nr, client->addr);

    TOUCH_NOTICE("Atmel Driver Built @%s, %s\n", __TIME__, __DATE__);

    //add fw/cfg name
    data->update_input = false;
    data->pin_id = 2;
    data->fw_name = MXT_FW_T2;
    data->cfg_name = MXT_CFG_T2_OF;
    data->client = client;
    data->irq = gpio_to_irq(158);
    i2c_set_clientdata(client, data);
    config_version = 0;
    reboot = false;
    cfg_force = false;
    print_log = true;

    /*1.handle pdata*/
    error = mxt_handle_pdata(data);
    if (error)
        goto err_free_mem;

    /*2.pull reset pin and identify tp pin, read tp id*/
    gpio_request(60,"ts_rst");
    gpio_direction_output(60,1);
    msleep(120);		//delay > 100ms

    /*
    gpio_request(59,"tp_id");
    data->pin_id = gpio_get_value(59);
    if(data->pin_id == 0)
    {
    	data->cfg_name =MXT_CFG_T2_OF;
    	dev_info(&client->dev,"pin_id is Ofilm\n");
    }
    else if(data->pin_id == 1)
    {
    	data->cfg_name =MXT_CFG_T2_JT;
    	dev_info(&client->dev,"pin_id is Jtouch\n");
    }
    else dev_info(&client->dev,"can't identify the tp id\n");
    */

    /*3.initialize completion and mutex*/
    init_completion(&data->bl_completion);
    init_completion(&data->reset_completion);
    init_completion(&data->crc_completion);
    mutex_init(&data->debug_msg_lock);

    /*4.probe regulators and read info block*/
    mxt_probe_regulators(data);

    error = red_mxt_read_info_block(data);
    if (error)
    {
        if(data->in_bootloader)
            TOUCH_ERR("atmel IC enter bootloader mode !\n");
        else
            goto err_free_pdata;
    }


    /*5.request thread irq*/
    error = request_threaded_irq(data->irq, NULL, mxt_interrupt,
                                 data->pdata->irqflags | IRQF_ONESHOT,
                                 client->name, data);
    if (error)
    {
        TOUCH_ERR("Failed %d to register interrupt\n",
                error);
        goto err_free_pdata;
    }

    /*6.read IC revision & load firmware and config , initialize the IC*/

    //read IC revision

    error = read_ic_revision(data);
    if(error)
    {
        TOUCH_NOTICE("Touch IC revision can not identify !");
        //goto err_free_irq;
        data->ic_revision = MXT_T37_IC_REV_D;
    }

    //data->ic_revision = MXT_T37_IC_REV_C;

    /*7.read IC cfg for which TP*/
    error = mxt_read_cfg_tp_id(data);
    if(error)
    {
        TOUCH_NOTICE("Touch CFG TP ID can't identify !");
        data->cfg_tp_id = TP_JTOUCH;
    }

    //if(data->in_bootloader || (data->info->version != 0x10||data->info->build !=0x11) )
    if(data->in_bootloader || (data->info->version != 0x11||data->info->build !=0xaa) )
    {
        error = mxt_load_fw(&client->dev);
        if(error)
        {
            TOUCH_ERR("Failed %d to upload firmware\n",error);
            goto err_free_irq;
        }
        else
        {
            mxt_soft_reset(data);
        }
    }
    else
    {
        disable_irq(data->irq);
        mxt_control_irq(data, false);
    }

    error = red_mxt_initialize(data);
    if (error)
    {
        TOUCH_ERR("Failed %d to initialize at the probe\n",
                error);
        goto err_free_input_dev;
    }

    /*7.sysfs operate*/
    error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
    if (error)
    {
        TOUCH_ERR("Failure %d creating sysfs group\n",
                error);
        goto err_free_object;
    }

    sysfs_bin_attr_init(&data->mem_access_attr);
    data->mem_access_attr.attr.name = "mem_access";
    data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;
    data->mem_access_attr.read = mxt_mem_access_read;
    data->mem_access_attr.write = mxt_mem_access_write;
    data->mem_access_attr.size = data->mem_size;

    if (sysfs_create_bin_file(&client->dev.kobj,
                              &data->mem_access_attr) < 0)
    {
        TOUCH_ERR("Failed to create %s\n",
                data->mem_access_attr.attr.name);
        goto err_remove_sysfs_group;
    }

    /* init for i2c stress test */
    touch_work_queue = create_singlethread_workqueue("i2c_touch_wq");
    if(!touch_work_queue){
        TOUCH_ERR("Unable to create workqueu \n");
        goto destroy_stress_test_wq;
    }
    INIT_DELAYED_WORK(&mxt_poll_data_work, mxt_poll_data);

    data->misc_dev.minor  = MISC_DYNAMIC_MINOR;
    data->misc_dev.name = "touch";
    data->misc_dev.fops = &mxt_fops;
    error = misc_register(&data->misc_dev);
    if (error){
	TOUCH_ERR("Unable to create workqueu \n");
        goto destroy_stress_test_wq;
    }

    return 0;

destroy_stress_test_wq:
    destroy_workqueue(touch_work_queue);
err_remove_sysfs_group:
    sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
err_free_object:
    mxt_free_object_table(data);
err_free_input_dev:
    mxt_free_input_device(data);
err_free_irq:
    free_irq(data->irq, data);
err_free_pdata:
    if (!dev_get_platdata(&data->client->dev))
        kfree(data->pdata);
err_free_mem:
    kfree(data);

    return error;
}

static int mxt_remove(struct i2c_client *client)
{
    struct mxt_data *data = i2c_get_clientdata(client);

    //add by red_zhang@asus.com
    unregister_early_suspend(&data->early_suspend);

    if (data->mem_access_attr.attr.name)
        sysfs_remove_bin_file(&client->dev.kobj,
                              &data->mem_access_attr);

    sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
    free_irq(data->irq, data);
    //regulator_put(data->reg_avdd);
    //regulator_put(data->reg_vdd);
    mxt_free_object_table(data);
    if (!dev_get_platdata(&data->client->dev))
        kfree(data->pdata);
    kfree(data);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxt_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mxt_data *data = i2c_get_clientdata(client);
    struct input_dev *input_dev = data->input_dev;
    struct timeval start, end;
    do_gettimeofday(&start);

    mutex_lock(&input_dev->mutex);

    if (input_dev->users)
        mxt_stop(data);

    mutex_unlock(&input_dev->mutex);

    do_gettimeofday(&end);
    TOUCH_NOTICE("Suspend spent %ld msecs.\n", (end.tv_sec * 1000 + end.tv_usec/1000 - start.tv_sec * 1000 - start.tv_usec/1000));

    return 0;
}

static int mxt_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct mxt_data *data = i2c_get_clientdata(client);
    struct input_dev *input_dev = data->input_dev;
    struct timeval start, end;
    do_gettimeofday(&start);

    mutex_lock(&input_dev->mutex);

    if (input_dev->users)
        mxt_start(data);

    mutex_unlock(&input_dev->mutex);

    do_gettimeofday(&end);
    TOUCH_NOTICE("Resume spent %ld msecs.\n", (end.tv_sec * 1000 + end.tv_usec/1000 - start.tv_sec * 1000 - start.tv_usec/1000));

    return 0;
}
#endif

static int mxt_early_suspend(struct early_suspend *h)
{
    struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);
    struct i2c_client *client = to_i2c_client(&data->client->dev);
    struct input_dev *input_dev = data->input_dev;

//		printk("<Red_debug> enter mxt_early_suspend\n");
    mutex_lock(&input_dev->mutex);

    if (input_dev->users)
        mxt_stop(data);

    mutex_unlock(&input_dev->mutex);

    return 0;
}

static int mxt_late_resume(struct early_suspend *h)
{
    struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);
    struct i2c_client *client = to_i2c_client(&data->client->dev);
    struct input_dev *input_dev = data->input_dev;

//		printk("<Red_debug> enter mxt_late_resume\n");
    mutex_lock(&input_dev->mutex);

    if (input_dev->users)
        mxt_start(data);

    mutex_unlock(&input_dev->mutex);

    return 0;
}

static SIMPLE_DEV_PM_OPS(mxt_pm_ops, mxt_suspend, mxt_resume);

static void mxt_shutdown(struct i2c_client *client)
{
    struct mxt_data *data = i2c_get_clientdata(client);

    disable_irq(data->irq);
    mxt_control_irq(data, false);
}

static const struct i2c_device_id mxt_id[] =
{
    {"atmel_mxt_ts", 0},
    { }
};

static struct acpi_device_id atmel_acpi_match[] =
{
    { "ATML1664", 0 },
};

MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver =
{
    .driver = {
        .name        = "atmel_mxt_ts",
        .owner        = THIS_MODULE,
        .pm        = &mxt_pm_ops,
        .acpi_match_table = ACPI_PTR(atmel_acpi_match),
    },
    .probe                = mxt_probe,
    .remove                = mxt_remove,
    .shutdown        = mxt_shutdown,
    .id_table        = mxt_id,
};

static int __init mxt_init(void)
{
    extern int entry_mode;
    //If in COS, don't init touch driver.
    if(entry_mode == 4)
       return -1;

    TOUCH_NOTICE("init Atmel driver!\n");
    return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
    i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
