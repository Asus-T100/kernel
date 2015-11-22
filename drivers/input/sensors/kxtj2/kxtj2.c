 /* Copyright (C) 2012 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>

// TODO:in L Framework will call enable sensor before late_resume
//       workaround to avoid screen hang on
#ifdef CONFIG_HAS_EARLYSUSPEND
#undef CONFIG_HAS_EARLYSUSPEND
#endif

#ifdef    CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

#include "kxtj2.h"

/* Add for i2c stress test */
#include <linux/miscdevice.h>

/* Debug Message Flags */
#define KIONIX_KMSG_ERR	1	/* Print kernel debug message for error */
#define KIONIX_KMSG_INF	1	/* Print kernel debug message for info */
#define KXTJ2_DEBUG 1

#if KIONIX_KMSG_ERR
#define KMSGERR(format, ...)	\
		dev_err(format, ## __VA_ARGS__)
#else
#define KMSGERR(format, ...)
#endif

#if KIONIX_KMSG_INF
#define KMSGINF(format, ...)	\
		dev_info(format, ## __VA_ARGS__)
#else
#define KMSGINF(format, ...)
#endif

/***** define for i2c stress test ++ *****/
#define KXTJ2_IOC_MAGIC 	0xF1
#define KXTJ2_IOC_MAXNR 	2
#define KXTJ2_POLL_DATA 	_IOR(KXTJ2_IOC_MAGIC, 2, int)

#define KXTJ2_IOCTL_START_HEAVY 	2
#define KXTJ2_IOCTL_START_NORMAL 	1
#define KXTJ2_IOCTL_END 		0

#define START_NORMAL 	msecs_to_jiffies(200) 	// 5Hz
#define START_HEAVY 	msecs_to_jiffies(5) 	// 200Hz

static int stress_test_poll_mode = 0;
struct delayed_work kxtj2_stress_test_poll_work;
static struct workqueue_struct *kxtj2_stress_test_work_queue;
struct kionix_accel_driver *kxtj2_acceld;
/***** define for i2c stress test -- *****/

/******************************************************************************
 * Accelerometer WHO_AM_I return value
 *****************************************************************************/
#define KIONIX_ACCEL_WHO_AM_I_KXTE9 		0x00
#define KIONIX_ACCEL_WHO_AM_I_KXTF9 		0x01
#define KIONIX_ACCEL_WHO_AM_I_KXTI9_1001 	0x04
#define KIONIX_ACCEL_WHO_AM_I_KXTIK_1004 	0x05
#define KIONIX_ACCEL_WHO_AM_I_KXTJ9_1005 	0x07
#define KIONIX_ACCEL_WHO_AM_I_KXTJ9_1007 	0x08
#define KIONIX_ACCEL_WHO_AM_I_KXCJ9_1008 	0x0A
#define KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009 	0x09
#define KIONIX_ACCEL_WHO_AM_I_KXCJK_1013 	0x11

/******************************************************************************
 * Accelerometer Grouping
 *****************************************************************************/
#define KIONIX_ACCEL_GRP1	1	/* KXTE9 */
#define KIONIX_ACCEL_GRP2	2	/* KXTF9/I9-1001/J9-1005 */
#define KIONIX_ACCEL_GRP3	3	/* KXTIK-1004 */
#define KIONIX_ACCEL_GRP4	4	/* KXTJ9-1007/KXCJ9-1008 */
#define KIONIX_ACCEL_GRP5	5	/* KXTJ2-1009 */
#define KIONIX_ACCEL_GRP6	6	/* KXCJK-1013 */

/******************************************************************************
 * Registers for Accelerometer Group 1 & 2 & 3
 *****************************************************************************/
#define ACCEL_WHO_AM_I		0x0F

/*****************************************************************************/
/* Registers for Accelerometer Group 1 */
/*****************************************************************************/
/* Output Registers */
#define ACCEL_GRP1_XOUT			0x12
/* Control Registers */
#define ACCEL_GRP1_CTRL_REG1	0x1B
/* CTRL_REG1 */
#define ACCEL_GRP1_PC1_OFF		0x7F
#define ACCEL_GRP1_PC1_ON		(1 << 7)
#define ACCEL_GRP1_ODR40		(3 << 3)
#define ACCEL_GRP1_ODR10		(2 << 3)
#define ACCEL_GRP1_ODR3			(1 << 3)
#define ACCEL_GRP1_ODR1			(0 << 3)
#define ACCEL_GRP1_ODR_MASK		(3 << 3)

/*****************************************************************************/
/* Registers for Accelerometer Group 2 & 3 */
/*****************************************************************************/
/* Output Registers */
#define ACCEL_GRP2_XOUT_L		0x06
/* Control Registers */
#define ACCEL_GRP2_INT_REL		0x1A
#define ACCEL_GRP2_CTRL_REG1	0x1B
#define ACCEL_GRP2_INT_CTRL1	0x1E
#define ACCEL_GRP2_DATA_CTRL	0x21
/* CTRL_REG1 */
#define ACCEL_GRP2_PC1_OFF		0x7F
#define ACCEL_GRP2_PC1_ON		(1 << 7)
#define ACCEL_GRP2_DRDYE		(1 << 5)
#define ACCEL_GRP2_G_8G			(2 << 3)
#define ACCEL_GRP2_G_4G			(1 << 3)
#define ACCEL_GRP2_G_2G			(0 << 3)
#define ACCEL_GRP2_G_MASK		(3 << 3)
#define ACCEL_GRP2_RES_8BIT		(0 << 6)
#define ACCEL_GRP2_RES_12BIT	(1 << 6)
#define ACCEL_GRP2_RES_MASK		(1 << 6)
/* INT_CTRL1 */
#define ACCEL_GRP2_IEA			(1 << 4)
#define ACCEL_GRP2_IEN			(1 << 5)
/* DATA_CTRL_REG */
#define ACCEL_GRP2_ODR12_5		0x00
#define ACCEL_GRP2_ODR25		0x01
#define ACCEL_GRP2_ODR50		0x02
#define ACCEL_GRP2_ODR100		0x03
#define ACCEL_GRP2_ODR200		0x04
#define ACCEL_GRP2_ODR400		0x05
#define ACCEL_GRP2_ODR800		0x06
/*****************************************************************************/

/*****************************************************************************/
/* Registers for Accelerometer Group 4 & 5 & 6 */
/*****************************************************************************/
/* Output Registers */
#define ACCEL_GRP4_XOUT_L		0x06
/* Control Registers */
#define ACCEL_GRP4_INT_REL		0x1A
#define ACCEL_GRP4_CTRL_REG1	0x1B
#define ACCEL_GRP4_INT_CTRL1	0x1E
#define ACCEL_GRP4_DATA_CTRL	0x21
/* CTRL_REG1 */
#define ACCEL_GRP4_PC1_OFF		0x7F
#define ACCEL_GRP4_PC1_ON		(1 << 7)
#define ACCEL_GRP4_DRDYE		(1 << 5)
#define ACCEL_GRP4_G_8G			(2 << 3)
#define ACCEL_GRP4_G_4G			(1 << 3)
#define ACCEL_GRP4_G_2G			(0 << 3)
#define ACCEL_GRP4_G_MASK		(3 << 3)
#define ACCEL_GRP4_RES_8BIT		(0 << 6)
#define ACCEL_GRP4_RES_12BIT	(1 << 6)
#define ACCEL_GRP4_RES_MASK		(1 << 6)
/* INT_CTRL1 */
#define ACCEL_GRP4_IEA			(1 << 4)
#define ACCEL_GRP4_IEN			(1 << 5)
/* DATA_CTRL_REG */
#define ACCEL_GRP4_ODR0_781		0x08
#define ACCEL_GRP4_ODR1_563		0x09
#define ACCEL_GRP4_ODR3_125		0x0A
#define ACCEL_GRP4_ODR6_25		0x0B
#define ACCEL_GRP4_ODR12_5		0x00
#define ACCEL_GRP4_ODR25		0x01
#define ACCEL_GRP4_ODR50		0x02
#define ACCEL_GRP4_ODR100		0x03
#define ACCEL_GRP4_ODR200		0x04
#define ACCEL_GRP4_ODR400		0x05
#define ACCEL_GRP4_ODR800		0x06
#define ACCEL_GRP4_ODR1600		0x07
/*****************************************************************************/

/* Input Event Constants */
#define ACCEL_G_MAX			8096
#define ACCEL_FUZZ			3
#define ACCEL_FLAT			3
/* I2C Retry Constants */
#define KIONIX_I2C_RETRY_COUNT		10 	/* Number of times to retry i2c */
#define KIONIX_I2C_RETRY_TIMEOUT	1	/* Timeout between retry (miliseconds) */

/* Earlysuspend Contants */
#define KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT	5000	/* Timeout (miliseconds) */

/* Calibration Constants */
#define GSENSOR_CALIBRATION_FILE_PATH	"/factory/sensors/accel_cal_data.ini"
static bool isSetCali = false;

/* Add API for Sleeping Beauty test tool */
#ifdef	CONFIG_SLEEPING_BEAUTY
struct i2c_register_data{
	struct i2c_client *client;
};
static struct i2c_register_data *i2c_data;
#endif	// CONFIG_SLEEPING_BEAUTY

/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate (ODR).
 */
static const struct {
	unsigned int cutoff;
	u8 mask;
} kionix_accel_grp1_odr_table[] = {
	{ 100,	ACCEL_GRP1_ODR40 },
	{ 334,	ACCEL_GRP1_ODR10 },
	{ 1000,	ACCEL_GRP1_ODR3  },
	{ 0,	ACCEL_GRP1_ODR1  },
};

static const struct {
	unsigned int cutoff;
	u8 mask;
} kionix_accel_grp2_odr_table[] = {
	{ 3,	ACCEL_GRP2_ODR800 },
	{ 5,	ACCEL_GRP2_ODR400 },
	{ 10,	ACCEL_GRP2_ODR200 },
	{ 20,	ACCEL_GRP2_ODR100 },
	{ 40,	ACCEL_GRP2_ODR50  },
	{ 80,	ACCEL_GRP2_ODR25  },
	{ 0,	ACCEL_GRP2_ODR12_5},
};

static const struct {
	unsigned int cutoff;
	u8 mask;
} kionix_accel_grp4_odr_table[] = {
	{ 2,	ACCEL_GRP4_ODR1600 },
	{ 3,	ACCEL_GRP4_ODR800 },
	{ 5,	ACCEL_GRP4_ODR400 },
	{ 10,	ACCEL_GRP4_ODR200 },
	{ 20,	ACCEL_GRP4_ODR100 },
	{ 40,	ACCEL_GRP4_ODR50  },
	{ 80,	ACCEL_GRP4_ODR25  },
	{ 160,	ACCEL_GRP4_ODR12_5},
	{ 320,	ACCEL_GRP4_ODR6_25},
	{ 640,	ACCEL_GRP4_ODR3_125},
	{ 1280,	ACCEL_GRP4_ODR1_563},
	{ 0,	ACCEL_GRP4_ODR0_781},
};

enum {
	accel_grp1_ctrl_reg1 = 0,
	accel_grp1_regs_count,
};

enum {
	accel_grp2_ctrl_reg1 = 0,
	accel_grp2_data_ctrl,
	accel_grp2_int_ctrl,
	accel_grp2_regs_count,
};

enum {
	accel_grp4_ctrl_reg1 = 0,
	accel_grp4_data_ctrl,
	accel_grp4_int_ctrl,
	accel_grp4_regs_count,
};

struct kionix_accel_driver {
	struct i2c_client *client;
	struct kionix_accel_platform_data accel_pdata;
	struct input_dev *input_dev;
	struct delayed_work accel_work;
	struct workqueue_struct *accel_workqueue;
	wait_queue_head_t wqh_suspend;

	int accel_data[3];
	int accel_cali[3];
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	bool negate_x;
	bool negate_y;
	bool negate_z;
	u8 shift;

	unsigned int poll_interval;
	unsigned int poll_delay;
	unsigned int accel_group;
	u8 *accel_registers;

	atomic_t accel_suspended;
	atomic_t accel_suspend_continue;
	atomic_t accel_enabled;
	atomic_t accel_input_event;
	atomic_t accel_enable_resume;
	struct mutex mutex_earlysuspend;
	struct mutex mutex_resume;
	rwlock_t rwlock_accel_data;

	bool accel_drdy;

	/* Function callback */
	void (*kionix_accel_report_accel_data)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_update_odr)(struct kionix_accel_driver *acceld, unsigned int poll_interval);
	int (*kionix_accel_power_on_init)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_operate)(struct kionix_accel_driver *acceld);
	int (*kionix_accel_standby)(struct kionix_accel_driver *acceld);

#ifdef    CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif /* CONFIG_HAS_EARLYSUSPEND */
};

static int kionix_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len);
static int kionix_strtok(const char *buf, size_t count, char **token, const int token_nr);

/* Add API for Sleeping Beauty test tool */
#ifdef	CONFIG_SLEEPING_BEAUTY
void sb_gsensor_0(void)
{
	int result = 0;
	u16 mask = 0x80;
	result = i2c_smbus_read_byte_data(i2c_data->client, ACCEL_GRP4_CTRL_REG1);
	if(result < 0)
	{
		printk("Gsensor dump power fail");
		return;
	}
	result = (result & mask);
	printk("sb_gsensor_0:0x%x\n",result);
}
EXPORT_SYMBOL(sb_gsensor_0);
#endif	// CONFIG_SLEEPING_BEAUTY

static int asus_accel_set_cali(struct kionix_accel_driver *acceld) {
	int len = 0;
	char buf[30] = {0};
	struct file *fp = NULL;
	mm_segment_t old_fs;

	// Variables for kionix_strtok
	const int cali_count = 3;
	char **buf2 = NULL;
	long calibration[cali_count];
	int err = 0, i = 0;

	if (isSetCali) {
		if (KXTJ2_DEBUG)
			printk("KXTJ2_DEBUG : Already set value to cali, do nothing.\n");
		return 0;
	}

	// Get data from accel_cal_data.ini
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(GSENSOR_CALIBRATION_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		printk("KXTJ2 : filp_open calibration file error, error code:%ld!\n", IS_ERR(fp));
		return -1;
	}
	len = fp->f_op->read(fp, buf, 30, &fp->f_pos);
	if (KXTJ2_DEBUG) {
		printk("KXTJ2_DEBUG : %s: len=%d\n", __func__, len);
		printk("KXTJ2_DEBUG : %s: buf=%s\n", __func__, buf);
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	if (len <= 0) {
		printk("KXTJ2 : len=%d, cannot be set to cali!\n", len);
		return -1;
	}

	// Split string by " " from buf[] to buf2[][];
	buf2 = (char **)kzalloc(cali_count * sizeof(char *), GFP_KERNEL);
	if (kionix_strtok(buf, (size_t)len, buf2, cali_count) < 0)
		printk("KXTJ2 : kionix_strtok error!\n");
	else {
		if (KXTJ2_DEBUG)
			for (i=0; i<cali_count; ++i)
				printk("KXTJ2_DEBUG : %s: kionix_strtok buf2[%d] %s\n", __func__, i, buf2[i]);

		// Convert string to integers
		for (i=0; i<cali_count; ++i) {
			err = kstrtoint((const char *)*(buf2+i), 10, (int *)&calibration[i]);
			if (err < 0) {
				printk("KXTJ2 : kstrtoint returned err = %d!\n", err);
				goto exit;
			}
		}

		if (KXTJ2_DEBUG)
			for (i=0; i<cali_count; ++i)
				printk("KXTJ2_DEBUG : %s: kionix_strtok calibration[%d] %ld\n", __func__, i, calibration[i]);

		// Set calibration to cali node
		write_lock(&acceld->rwlock_accel_data);
		acceld->accel_cali[acceld->axis_map_x] = (int)calibration[0];
		acceld->accel_cali[acceld->axis_map_y] = (int)calibration[1];
		acceld->accel_cali[acceld->axis_map_z] = (int)calibration[2];
		write_unlock(&acceld->rwlock_accel_data);

		isSetCali = true;
	}

exit:
	for (i=0; i<cali_count; ++i)
		kfree(*(buf2+i));
	kfree(buf2);

	return err;
}

// For kxtj2, it must read INT_REL to release interrupt when
// set ACCEL_GRP4_PC1_ON to CTRL_REG1, which means that sensor's
// mode changed from stand-by mode to operating mode.
static int asus_accel_grp4_clear_interrupt(struct kionix_accel_driver *acceld) {
	int loop, err;
	loop = KIONIX_I2C_RETRY_COUNT;

	if (KXTJ2_DEBUG)
		printk("KXTJ2_DEBUG : clear interrupt.\n");

	while(loop) {
		err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP4_INT_REL);
		if(err < 0){
			loop--;
			mdelay(KIONIX_I2C_RETRY_TIMEOUT);
		}
		else
			loop = 0;
	}
	if (err < 0)
		KMSGERR(&acceld->client->dev, "%s: clear interrupt error = %d\n", __func__, err);

	return err;
}

/******************************************************************************
 * For i2c stress test ++
 ******************************************************************************/
static void asus_kxtj2_stress_test_report_accel_data(struct kionix_accel_driver *acceld)
{
	struct { union {
		s16 accel_data_s16[3];
		s8	accel_data_s8[6];
	}; } accel_data;
	s16 x, y, z;
	int rx, ry, rz;
	int err;
	struct input_dev *input_dev = acceld->input_dev;
	int loop;

	/* Only read the output registers if enabled */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			loop = KIONIX_I2C_RETRY_COUNT;
			while(loop) {
				mutex_lock(&input_dev->mutex);
				err = kionix_i2c_read(acceld->client, ACCEL_GRP4_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
				mutex_unlock(&input_dev->mutex);
				if(err < 0){
					loop--;
					mdelay(KIONIX_I2C_RETRY_TIMEOUT);
				}
				else
					loop = 0;
			}
			if (err < 0) {
				KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);

				x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) >> acceld->shift;
				y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) >> acceld->shift;
				z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) >> acceld->shift;

				acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) + acceld->accel_cali[acceld->axis_map_x];
				acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) + acceld->accel_cali[acceld->axis_map_y];
				acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) + acceld->accel_cali[acceld->axis_map_z];

				if(atomic_read(&acceld->accel_input_event) > 0) {
					input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
					input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
					input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
					input_sync(acceld->input_dev);
				}

				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}

	/* Clear the interrupt if using drdy */
	if(acceld->accel_drdy == 1) {
		loop = KIONIX_I2C_RETRY_COUNT;
		while(loop) {
			err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP4_INT_REL);
			if(err < 0){
				loop--;
				mdelay(KIONIX_I2C_RETRY_TIMEOUT);
			}
			else
				loop = 0;
		}
		if (err < 0)
			KMSGERR(&acceld->client->dev, "%s: clear interrupt error = %d\n", __func__, err);
	}

	read_lock(&acceld->rwlock_accel_data);

	rx = acceld->accel_data[acceld->axis_map_x];
	ry = acceld->accel_data[acceld->axis_map_y];
	rz = acceld->accel_data[acceld->axis_map_z];

	read_unlock(&acceld->rwlock_accel_data);

	KMSGINF(&kxtj2_acceld->client->dev, "%s: %d %d %d\n", __func__, rx, ry, rz);
}

static void asus_kxtj2_stress_test_poll(struct work_struct *work)
{
	// Read data through i2c
	asus_kxtj2_stress_test_report_accel_data(kxtj2_acceld);

	if (stress_test_poll_mode == 0)
		msleep(5);

	queue_delayed_work(kxtj2_stress_test_work_queue, \
		&kxtj2_stress_test_poll_work, stress_test_poll_mode);
}

static int kxtj2_fops_open(struct inode *inode, struct file *file)
{
	KMSGINF(&kxtj2_acceld->client->dev, "%s: i2c stress test misc device open\n", __func__);
	return 0;
}

static int kxtj2_fops_release(struct inode *inode, struct file *file)
{
	KMSGINF(&kxtj2_acceld->client->dev, "%s: i2c stress test misc device release\n", __func__);
	return 0;
}

static long kxtj2_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 1;
	if (_IOC_TYPE(cmd) != KXTJ2_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > KXTJ2_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case KXTJ2_POLL_DATA:
			if (arg == KXTJ2_IOCTL_START_HEAVY){
				KMSGINF(&kxtj2_acceld->client->dev, "%s: i2c stress test start on heavy mode\n", \
					__func__);
				stress_test_poll_mode = START_HEAVY;
				queue_delayed_work(kxtj2_stress_test_work_queue, \
					&kxtj2_stress_test_poll_work, stress_test_poll_mode);
			}
			else if (arg == KXTJ2_IOCTL_START_NORMAL){
				KMSGINF(&kxtj2_acceld->client->dev, "%s: i2c stress test start on normal mode\n", \
					__func__);
				stress_test_poll_mode = START_NORMAL;
				queue_delayed_work(kxtj2_stress_test_work_queue, \
					&kxtj2_stress_test_poll_work, stress_test_poll_mode);
			}
			else if  (arg == KXTJ2_IOCTL_END){
				KMSGINF(&kxtj2_acceld->client->dev, "%s: i2c stress test end\n", \
					__func__);
				cancel_delayed_work_sync(&kxtj2_stress_test_poll_work);
			}
			else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return 0;
}

static const struct file_operations kxtj2_fops = {
	.owner = THIS_MODULE,
	.open = kxtj2_fops_open,
	.release = kxtj2_fops_release,
	.unlocked_ioctl = kxtj2_ioctl,
};

static struct miscdevice kxtj2_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kxtj2",
	.fops = &kxtj2_fops,
};
/******************************************************************************
 * For i2c stress test --
 ******************************************************************************/

static int kionix_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = client->addr,
			.flags = client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(client->adapter, msgs, 2);
}

static int kionix_strtok(const char *buf, size_t count, char **token, const int token_nr)
{
	char *buf2 = (char *)kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
	char **token2 = token;
	unsigned int num_ptr = 0, num_nr = 0, num_neg = 0;
	int i = 0, start = 0, end = (int)count;

	strcpy(buf2, buf);

	/* We need to breakup the string into separate chunks in order for kstrtoint
	 * or strict_strtol to parse them without returning an error. Stop when the end of
	 * the string is reached or when enough value is read from the string */
	while((start < end) && (i < token_nr)) {
		/* We found a negative sign */
		if(*(buf2 + start) == '-') {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				/* If there is a pending negative sign, we adjust the variables to account for it */
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
				/* Reset */
				num_ptr = num_nr = 0;
			}
			/* This indicates that there is a pending negative sign in the string */
			num_neg = 1;
		}
		/* We found a numeric */
		else if((*(buf2 + start) >= '0') && (*(buf2 + start) <= '9')) {
			/* If the previous char(s) are not numeric, set num_ptr to current char */
			if(num_nr < 1)
				num_ptr = start;
			num_nr++;
		}
		/* We found an unwanted character */
		else {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
			}
			/* Reset all the variables to start afresh */
			num_ptr = num_nr = num_neg = 0;
		}
		start++;
	}

	kfree(buf2);

	return (i == token_nr) ? token_nr : -1;
}

static int kionix_accel_grp1_power_on_init(struct kionix_accel_driver *acceld)
{
	int err;

	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP1_CTRL_REG1, acceld->accel_registers[accel_grp1_ctrl_reg1] | ACCEL_GRP1_PC1_ON);
		if (err < 0)
			return err;
	}
	else {
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP1_CTRL_REG1, acceld->accel_registers[accel_grp1_ctrl_reg1]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp1_operate(struct kionix_accel_driver *acceld)
{
	int err;

	err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP1_CTRL_REG1, \
			acceld->accel_registers[accel_grp2_ctrl_reg1] | ACCEL_GRP1_PC1_ON);
	if (err < 0)
		return err;

	queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return 0;
}

static int kionix_accel_grp1_standby(struct kionix_accel_driver *acceld)
{
	int err;

	cancel_delayed_work_sync(&acceld->accel_work);

	err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP1_CTRL_REG1, 0);
	if (err < 0)
		return err;

	return 0;
}

static void kionix_accel_grp1_report_accel_data(struct kionix_accel_driver *acceld)
{
	u8 accel_data[3];
	s16 x, y, z;
	int err;
	struct input_dev *input_dev = acceld->input_dev;
	int loop = KIONIX_I2C_RETRY_COUNT;

	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			while(loop) {
				mutex_lock(&input_dev->mutex);
				err = kionix_i2c_read(acceld->client, ACCEL_GRP1_XOUT, accel_data, 6);
				mutex_unlock(&input_dev->mutex);
				if(err < 0){
					loop--;
					mdelay(KIONIX_I2C_RETRY_TIMEOUT);
				}
				else
					loop = 0;
			}
			if (err < 0) {
				KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);

				x = ((s16) le16_to_cpu(((s16)(accel_data[acceld->axis_map_x] >> 2)) - 32)) << 6;
				y = ((s16) le16_to_cpu(((s16)(accel_data[acceld->axis_map_y] >> 2)) - 32)) << 6;
				z = ((s16) le16_to_cpu(((s16)(accel_data[acceld->axis_map_z] >> 2)) - 32)) << 6;

				acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) + acceld->accel_cali[acceld->axis_map_x];
				acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) + acceld->accel_cali[acceld->axis_map_y];
				acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) + acceld->accel_cali[acceld->axis_map_z];

				if(atomic_read(&acceld->accel_input_event) > 0) {
					input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
					input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
					input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
					input_sync(acceld->input_dev);
				}

				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}
}

static int kionix_accel_grp1_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kionix_accel_grp1_odr_table); i++) {
		odr = kionix_accel_grp1_odr_table[i].mask;
		if (poll_interval < kionix_accel_grp1_odr_table[i].cutoff)
			break;
	}

	/* Do not need to update CTRL_REG1 register if the ODR is not changed */
	if((acceld->accel_registers[accel_grp1_ctrl_reg1] & ACCEL_GRP1_ODR_MASK) == odr)
		return 0;
	else {
		acceld->accel_registers[accel_grp1_ctrl_reg1] &= ~ACCEL_GRP1_ODR_MASK;
		acceld->accel_registers[accel_grp1_ctrl_reg1] |= odr;
	}

	/* Do not need to update CTRL_REG1 register if the sensor is not currently turn on */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP1_CTRL_REG1, \
				acceld->accel_registers[accel_grp1_ctrl_reg1] | ACCEL_GRP1_PC1_ON);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp2_power_on_init(struct kionix_accel_driver *acceld)
{
	int err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(acceld->client,
					ACCEL_GRP2_CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(acceld->client,
					ACCEL_GRP2_DATA_CTRL, acceld->accel_registers[accel_grp2_data_ctrl]);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (acceld->client->irq) {
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP2_INT_CTRL1, acceld->accel_registers[accel_grp2_int_ctrl]);
		if (err < 0)
			return err;
	}

	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP2_CTRL_REG1, acceld->accel_registers[accel_grp2_ctrl_reg1] | ACCEL_GRP2_PC1_ON);
		if (err < 0)
			return err;
	}
	else {
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP2_CTRL_REG1, acceld->accel_registers[accel_grp2_ctrl_reg1]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp2_operate(struct kionix_accel_driver *acceld)
{
	int err;

	err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP2_CTRL_REG1, \
			acceld->accel_registers[accel_grp2_ctrl_reg1] | ACCEL_GRP2_PC1_ON);
	if (err < 0)
		return err;

	if(acceld->accel_drdy == 0)
		queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return 0;
}

static int kionix_accel_grp2_standby(struct kionix_accel_driver *acceld)
{
	int err;

	if(acceld->accel_drdy == 0)
		cancel_delayed_work_sync(&acceld->accel_work);

	err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP2_CTRL_REG1, 0);
	if (err < 0)
		return err;

	return 0;
}

static void kionix_accel_grp2_report_accel_data(struct kionix_accel_driver *acceld)
{
	struct { union {
		s16 accel_data_s16[3];
		s8	accel_data_s8[6];
	}; } accel_data;
	s16 x, y, z;
	int err;
	struct input_dev *input_dev = acceld->input_dev;
	int loop;

	/* Only read the output registers if enabled */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			loop = KIONIX_I2C_RETRY_COUNT;
			while(loop) {
				mutex_lock(&input_dev->mutex);
				err = kionix_i2c_read(acceld->client, ACCEL_GRP2_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
				mutex_unlock(&input_dev->mutex);
				if(err < 0){
					loop--;
					mdelay(KIONIX_I2C_RETRY_TIMEOUT);
				}
				else
					loop = 0;
			}
			if (err < 0) {
				KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);

				x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) >> acceld->shift;
				y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) >> acceld->shift;
				z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) >> acceld->shift;

				acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) + acceld->accel_cali[acceld->axis_map_x];
				acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) + acceld->accel_cali[acceld->axis_map_y];
				acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) + acceld->accel_cali[acceld->axis_map_z];

				if(atomic_read(&acceld->accel_input_event) > 0) {
					input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
					input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
					input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
					input_sync(acceld->input_dev);
				}

				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}

	/* Clear the interrupt if using drdy */
	if(acceld->accel_drdy == 1) {
		loop = KIONIX_I2C_RETRY_COUNT;
		while(loop) {
			err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP2_INT_REL);
			if(err < 0){
				loop--;
				mdelay(KIONIX_I2C_RETRY_TIMEOUT);
			}
			else
				loop = 0;
		}
		if (err < 0)
			KMSGERR(&acceld->client->dev, "%s: clear interrupt error = %d\n", __func__, err);
	}
}

static void kionix_accel_grp2_update_g_range(struct kionix_accel_driver *acceld)
{
	acceld->accel_registers[accel_grp2_ctrl_reg1] &= ~ACCEL_GRP2_G_MASK;

	switch (acceld->accel_pdata.accel_g_range) {
		case KIONIX_ACCEL_G_8G:
		case KIONIX_ACCEL_G_6G:
			acceld->shift = 2;
			acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_G_8G;
			break;
		case KIONIX_ACCEL_G_4G:
			acceld->shift = 3;
			acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_G_4G;
			break;
		case KIONIX_ACCEL_G_2G:
		default:
			acceld->shift = 4;
			acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_G_2G;
			break;
	}

	return;
}

static int kionix_accel_grp2_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kionix_accel_grp2_odr_table); i++) {
		odr = kionix_accel_grp2_odr_table[i].mask;
		if (poll_interval < kionix_accel_grp2_odr_table[i].cutoff)
			break;
	}

	/* Do not need to update DATA_CTRL_REG register if the ODR is not changed */
	if(acceld->accel_registers[accel_grp2_data_ctrl] == odr)
		return 0;
	else
		acceld->accel_registers[accel_grp2_data_ctrl] = odr;

	/* Do not need to update DATA_CTRL_REG register if the sensor is not currently turn on */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP2_CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP2_DATA_CTRL, acceld->accel_registers[accel_grp2_data_ctrl]);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP2_CTRL_REG1, acceld->accel_registers[accel_grp2_ctrl_reg1] | ACCEL_GRP2_PC1_ON);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp4_power_on_init(struct kionix_accel_driver *acceld)
{
	int err;

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(acceld->client,
					ACCEL_GRP4_CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(acceld->client,
					ACCEL_GRP4_DATA_CTRL, acceld->accel_registers[accel_grp4_data_ctrl]);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
	if (gpio_to_irq(acceld->accel_pdata.int_gpio)) {
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP4_INT_CTRL1, acceld->accel_registers[accel_grp4_int_ctrl]);
		printk("write INT_CTRL_REG1 err=%d\n",err);
		if (err < 0)
			return err;

	}

	if(atomic_read(&acceld->accel_enabled) > 0) {//User need to turn on G-sensor by manual
	//if (acceld->accel_drdy) {//this line will let G-sensor be the IRQ mode at the power-up
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP4_CTRL_REG1, acceld->accel_registers[accel_grp4_ctrl_reg1] | ACCEL_GRP4_PC1_ON |ACCEL_GRP4_DRDYE |ACCEL_GRP4_RES_12BIT);
		printk("write CTRL_REG1 err = %d\n",err);
		if (err < 0)
			return err;

		err = asus_accel_grp4_clear_interrupt(acceld);
		if (err < 0)
			return err;
	}
	else {
		err = i2c_smbus_write_byte_data(acceld->client,
						ACCEL_GRP4_CTRL_REG1, acceld->accel_registers[accel_grp4_ctrl_reg1]);
		if (err < 0)
			return err;
	}

	return 0;
}

static int kionix_accel_grp4_operate(struct kionix_accel_driver *acceld)
{
	int err;

	err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP4_CTRL_REG1, \
			acceld->accel_registers[accel_grp4_ctrl_reg1] | ACCEL_GRP4_PC1_ON);
	if (err < 0)
		return err;

	err = asus_accel_grp4_clear_interrupt(acceld);
	if (err < 0)
		return err;

	if(acceld->accel_drdy == 0)
		queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return 0;
}

static int kionix_accel_grp4_standby(struct kionix_accel_driver *acceld)
{
	int err;

	if(acceld->accel_drdy == 0)
		cancel_delayed_work_sync(&acceld->accel_work);

	err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP4_CTRL_REG1, 0);
	if (err < 0)
		return err;

	return 0;
}

/*Kionix KFA2.0 Start*/
/* due to project schedule,only use KFA on ME176CE ME176C and ME181C*/
#if defined (CONFIG_ME176CE) || defined(CONFIG_ME176C) || defined(CONFIG_ME181C)
#define KIONIX_KFA_CAL //Setup KFA definition
#endif
#ifdef KIONIX_KFA_CAL
#define Sensitivity_def      1024
#define Offset_torlance      200
#define Sensitivity_torlance 154 * 2
#define Stable_range         100
#define BUF_RANGE_Limit      5
#define NewBuffer            22222
static int BUF_RANGE = BUF_RANGE_Limit;
static int FIFO_BUF[3][BUF_RANGE_Limit] = {{0}};
static int Og_Shift[3] = {0, 0, 0};
static int K_param[6][3] = {
		9999,9999,9999,9999,9999,9999,
		9999,9999,9999,9999,9999,9999,
		9999,9999,9999,9999,9999,9999};
static int Wave_Max, Wave_Min;
static int osci = -1;
#endif
/*Kionix KFA2.0 End*/

static void kionix_accel_grp4_report_accel_data(struct kionix_accel_driver *acceld)
{
	struct { union {
		s16 accel_data_s16[3];
		s8 accel_data_s8[6];
	}; } accel_data;
	s16 x, y, z;
	int err;
	struct input_dev *input_dev = acceld->input_dev;
	int loop;

/*Kionix KFA2.0 Start*/
#ifdef KIONIX_KFA_CAL
	s16 raw[3];
	s16 Axes_Position = 0;
	int i, k, rc;
	long temp_sum = 0;
	s16 Axes_Avged[3];
#endif
/*Kionix KFA2.0 End*/

	/* Only read the output registers if enabled */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		if(atomic_read(&acceld->accel_enable_resume) > 0)
		{
			loop = KIONIX_I2C_RETRY_COUNT;
			while(loop) {
				mutex_lock(&input_dev->mutex);
				err = kionix_i2c_read(acceld->client, ACCEL_GRP4_XOUT_L, (u8 *)accel_data.accel_data_s16, 6);
				mutex_unlock(&input_dev->mutex);
				if(err < 0){
					loop--;
					mdelay(KIONIX_I2C_RETRY_TIMEOUT);
				}
				else
					loop = 0;
			}
			if (err < 0) {
				KMSGERR(&acceld->client->dev, "%s: read data output error = %d\n", __func__, err);
			}
			else {
				write_lock(&acceld->rwlock_accel_data);

				x = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_x])) >> acceld->shift;
				y = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_y])) >> acceld->shift;
				z = ((s16) le16_to_cpu(accel_data.accel_data_s16[acceld->axis_map_z])) >> acceld->shift;

/*Kionix KFA2.0 Start*/
#ifdef KIONIX_KFA_CAL
				raw[0] = x;
				raw[1] = y;
				raw[2] = z;

				rc = 0;
				for (i = 0; i < 3; i++) {
					Wave_Max = -(NewBuffer);
					Wave_Min = NewBuffer;
					temp_sum = 0;
					for (k = 0; k < BUF_RANGE; k++) {
						if (k >= (BUF_RANGE-1))
							FIFO_BUF[i][k] = raw[i];
						else
							FIFO_BUF[i][k] = FIFO_BUF[i][k+1];
						temp_sum += FIFO_BUF[i][k];
						if (FIFO_BUF[i][k] > Wave_Max) Wave_Max = FIFO_BUF[i][k];
						if (FIFO_BUF[i][k] < Wave_Min) Wave_Min = FIFO_BUF[i][k];
					}
					Axes_Avged[i] = temp_sum / k;
					raw[i] = Axes_Avged[i];

					if (abs(Axes_Avged[i]) < Offset_torlance)
						rc++;
					else
						if ((abs((abs(Axes_Avged[i]) - Sensitivity_def)) <
								(Offset_torlance + Sensitivity_torlance))) {
							rc += 10;
							if (Axes_Avged[i] >= 0) {
								Axes_Position = i*2+1;
							} else {
								Axes_Position = i*2;
							}
						}
					if (Wave_Max - Wave_Min > Stable_range)
						rc += 100; //check over range
				}

				if (rc == 12) {
					for (i = 0; i < 3; i++) {
						K_param[Axes_Position][i] = Axes_Avged[i];
						if (abs(Axes_Avged[i]) < Offset_torlance)
							Og_Shift[i] = Axes_Avged[i] / 2;
						else
							Og_Shift[i] = 0;
					}
				}

				for (i = 0; i < 3; i++) {
					rc = 0;
					if (raw[i] >= 0) rc = 1;
					if (K_param[i*2+rc][i] == 9999)
						raw[i] = raw[i] - Og_Shift[i];
					else
						raw[i] = raw[i] * 1024 / abs(K_param[i*2+rc][i]) - Og_Shift[i];
				}

				osci = -osci;
				x = raw[0] + osci * (x % 5);
				y = raw[1] + osci * (y % 5);
				z = raw[2] + osci * (z % 5);
#endif
/*Kionix KFA2.0 End*/

				acceld->accel_data[acceld->axis_map_x] = (acceld->negate_x ? -x : x) + acceld->accel_cali[acceld->axis_map_x];
				acceld->accel_data[acceld->axis_map_y] = (acceld->negate_y ? -y : y) + acceld->accel_cali[acceld->axis_map_y];
				acceld->accel_data[acceld->axis_map_z] = (acceld->negate_z ? -z : z) + acceld->accel_cali[acceld->axis_map_z];

				if(atomic_read(&acceld->accel_input_event) > 0) {
					input_report_abs(acceld->input_dev, ABS_X, acceld->accel_data[acceld->axis_map_x]);
					input_report_abs(acceld->input_dev, ABS_Y, acceld->accel_data[acceld->axis_map_y]);
					input_report_abs(acceld->input_dev, ABS_Z, acceld->accel_data[acceld->axis_map_z]);
					input_sync(acceld->input_dev);
				}

				write_unlock(&acceld->rwlock_accel_data);
			}
		}
		else
		{
			atomic_inc(&acceld->accel_enable_resume);
		}
	}

	/* Clear the interrupt if using drdy */
	if(acceld->accel_drdy == 1) {
		loop = KIONIX_I2C_RETRY_COUNT;
		while(loop) {
			err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP4_INT_REL);
			if(err < 0){
				loop--;
				mdelay(KIONIX_I2C_RETRY_TIMEOUT);
			}
			else
				loop = 0;
		}
		if (err < 0)
			KMSGERR(&acceld->client->dev, "%s: clear interrupt error = %d\n", __func__, err);
	}
}

static void kionix_accel_grp4_update_g_range(struct kionix_accel_driver *acceld)
{
	acceld->accel_registers[accel_grp4_ctrl_reg1] &= ~ACCEL_GRP4_G_MASK;

	switch (acceld->accel_pdata.accel_g_range) {
		case KIONIX_ACCEL_G_8G:
		case KIONIX_ACCEL_G_6G:
			acceld->shift = 2;
			acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_G_8G;
			break;
		case KIONIX_ACCEL_G_4G:
			acceld->shift = 3;
			acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_G_4G;
			break;
		case KIONIX_ACCEL_G_2G:
		default:
			acceld->shift = 4;
			acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_G_2G;
			break;
	}

	return;
}

static int kionix_accel_grp4_update_odr(struct kionix_accel_driver *acceld, unsigned int poll_interval)
{
	int err;
	int i;
	u8 odr;
	KMSGINF(&acceld->client->dev, "kionix_accel_grp4_update_odr() ++ poll=%d\n",poll_interval);
	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kionix_accel_grp4_odr_table); i++) {
		odr = kionix_accel_grp4_odr_table[i].mask;
		if (poll_interval < kionix_accel_grp4_odr_table[i].cutoff)
			break;
	}

	/* Do not need to update DATA_CTRL_REG register if the ODR is not changed */
	if(acceld->accel_registers[accel_grp4_data_ctrl] == odr)
		return 0;
	else
		acceld->accel_registers[accel_grp4_data_ctrl] = odr;

	/* Do not need to update DATA_CTRL_REG register if the sensor is not currently turn on */
	if(atomic_read(&acceld->accel_enabled) > 0) {
		err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP4_CTRL_REG1, 0);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP4_DATA_CTRL, acceld->accel_registers[accel_grp4_data_ctrl]);
		if (err < 0)
			return err;

		err = i2c_smbus_write_byte_data(acceld->client, ACCEL_GRP4_CTRL_REG1, acceld->accel_registers[accel_grp4_ctrl_reg1] | ACCEL_GRP4_PC1_ON);
		if (err < 0)
			return err;

		err = asus_accel_grp4_clear_interrupt(acceld);
		if (err < 0)
			return err;

		//#############
		err = i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP4_DATA_CTRL);
		if (err < 0)
			return err;
		switch(err) {
			case ACCEL_GRP4_ODR0_781:
				dev_info(&acceld->client->dev, "ODR = 0.781 Hz\n");
				break;
			case ACCEL_GRP4_ODR1_563:
				dev_info(&acceld->client->dev, "ODR = 1.563 Hz\n");
				break;
			case ACCEL_GRP4_ODR3_125:
				dev_info(&acceld->client->dev, "ODR = 3.125 Hz\n");
				break;
			case ACCEL_GRP4_ODR6_25:
				dev_info(&acceld->client->dev, "ODR = 6.25 Hz\n");
				break;
			case ACCEL_GRP4_ODR12_5:
				dev_info(&acceld->client->dev, "ODR = 12.5 Hz\n");
				break;
			case ACCEL_GRP4_ODR25:
				dev_info(&acceld->client->dev, "ODR = 25 Hz\n");
				break;
			case ACCEL_GRP4_ODR50:
				dev_info(&acceld->client->dev, "ODR = 50 Hz\n");
				break;
			case ACCEL_GRP4_ODR100:
				dev_info(&acceld->client->dev, "ODR = 100 Hz\n");
				break;
			case ACCEL_GRP4_ODR200:
				dev_info(&acceld->client->dev, "ODR = 200 Hz\n");
				break;
			case ACCEL_GRP4_ODR400:
				dev_info(&acceld->client->dev, "ODR = 400 Hz\n");
				break;
			case ACCEL_GRP4_ODR800:
				dev_info(&acceld->client->dev, "ODR = 800 Hz\n");
				break;
			case ACCEL_GRP4_ODR1600:
				dev_info(&acceld->client->dev, "ODR = 1600 Hz\n");
				break;
			default:
				dev_info(&acceld->client->dev, "Unknown ODR\n");
				break;
		}
		//#############
	}
	KMSGINF(&acceld->client->dev, "kionix_accel_grp4_update_odr() --\n");
	return 0;
}

static int kionix_accel_power_on(struct kionix_accel_driver *acceld)
{
	if (acceld->accel_pdata.power_on)
		return acceld->accel_pdata.power_on();

	return 0;
}

static void kionix_accel_power_off(struct kionix_accel_driver *acceld)
{
	if (acceld->accel_pdata.power_off)
		acceld->accel_pdata.power_off();
}

static irqreturn_t kionix_accel_isr(int irq, void *dev)
{
	struct kionix_accel_driver *acceld = dev;
	//printk("kionix_accel_isr ++\n");
	queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, 0);

	return IRQ_HANDLED;
}

static void kionix_accel_work(struct work_struct *work)
{
	struct kionix_accel_driver *acceld = container_of((struct delayed_work *)work,	struct kionix_accel_driver, accel_work);
	//printk("kionix_accel_work ++ drdy=%d\n",acceld->accel_drdy);
	if(acceld->accel_drdy == 0)
		queue_delayed_work(acceld->accel_workqueue, &acceld->accel_work, acceld->poll_delay);

	acceld->kionix_accel_report_accel_data(acceld);
}

static void kionix_accel_update_direction(struct kionix_accel_driver *acceld)
{
	unsigned int direction = acceld->accel_pdata.accel_direction;
	unsigned int accel_group = acceld->accel_group;

	write_lock(&acceld->rwlock_accel_data);
	acceld->axis_map_x = ((direction-1)%2);
	acceld->axis_map_y =  (direction%2);
	acceld->axis_map_z =  2;
	acceld->negate_z = ((direction-1)/4);
	switch(accel_group) {
		case KIONIX_ACCEL_GRP3:
		case KIONIX_ACCEL_GRP6:
			acceld->negate_x = (((direction+2)/2)%2);
			acceld->negate_y = (((direction+5)/4)%2);
			break;
		case KIONIX_ACCEL_GRP5:
			acceld->axis_map_x =  (direction%2);
			acceld->axis_map_y = ((direction-1)%2);
			acceld->negate_x =  (((direction+1)/2)%2);
			acceld->negate_y =  (((direction/2)+((direction-1)/4))%2);
			break;
		default:
			acceld->negate_x =  ((direction/2)%2);
			acceld->negate_y = (((direction+1)/4)%2);
			break;
	}
	write_unlock(&acceld->rwlock_accel_data);
	return;
}

static int kionix_accel_enable(struct kionix_accel_driver *acceld)
{
	int err = 0;
	long remaining;

	mutex_lock(&acceld->mutex_earlysuspend);

	atomic_set(&acceld->accel_suspend_continue, 0);

	/* Make sure that the sensor had successfully resumed before enabling it */
	if(atomic_read(&acceld->accel_suspended) == 1) {
		KMSGINF(&acceld->client->dev, "%s: waiting for resume\n", __func__);
		remaining = wait_event_interruptible_timeout(acceld->wqh_suspend, \
				atomic_read(&acceld->accel_suspended) == 0, \
				msecs_to_jiffies(KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT));

		if(atomic_read(&acceld->accel_suspended) == 1) {
			KMSGERR(&acceld->client->dev, "%s: timeout waiting for resume\n", __func__);
			err = -ETIME;
			goto exit;
		}
	}

	err = acceld->kionix_accel_operate(acceld);

	if (err < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: kionix_accel_operate returned err = %d\n", __func__, err);
		goto exit;
	}

	atomic_inc(&acceld->accel_enabled);
#if defined (CONFIG_ME176CE) || defined(CONFIG_ME176C) || defined(CONFIG_ME181C)
	KMSGINF(&acceld->client->dev,"Use auto-calibration,ignore load calibration data\n");
#else
	KMSGINF(&acceld->client->dev,"Load calibration data\n");
	if (asus_accel_set_cali(acceld)) {
		KMSGERR(&acceld->client->dev, \
				"%s: asus_accel_set_cali returned err\n", __func__);
	}
#endif
exit:
	mutex_unlock(&acceld->mutex_earlysuspend);

	return err;
}

static int kionix_accel_disable(struct kionix_accel_driver *acceld)
{
	int err = 0;

	mutex_lock(&acceld->mutex_resume);

	atomic_set(&acceld->accel_suspend_continue, 1);

	if(atomic_read(&acceld->accel_enabled) > 0){
		if(atomic_dec_and_test(&acceld->accel_enabled)) {
			if(atomic_read(&acceld->accel_enable_resume) > 0)
				atomic_set(&acceld->accel_enable_resume, 0);
			err = acceld->kionix_accel_standby(acceld);
			if (err < 0) {
				KMSGERR(&acceld->client->dev, \
						"%s: kionix_accel_standby returned err = %d\n", __func__, err);
				goto exit;
			}
			wake_up_interruptible(&acceld->wqh_suspend);
		}
	}

exit:
	mutex_unlock(&acceld->mutex_resume);

	return err;
}

static int kionix_accel_input_open(struct input_dev *input)
{
	struct kionix_accel_driver *acceld = input_get_drvdata(input);

	atomic_inc(&acceld->accel_input_event);

	return 0;
}

static void kionix_accel_input_close(struct input_dev *dev)
{
	struct kionix_accel_driver *acceld = input_get_drvdata(dev);

	atomic_dec(&acceld->accel_input_event);
}

static void  kionix_accel_init_input_device(struct kionix_accel_driver *acceld,
					      struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
	input_set_abs_params(input_dev, ABS_Y, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);
	input_set_abs_params(input_dev, ABS_Z, -ACCEL_G_MAX, ACCEL_G_MAX, ACCEL_FUZZ, ACCEL_FLAT);

	input_dev->name = KIONIX_ACCEL_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &acceld->client->dev;
}

static int  kionix_accel_setup_input_device(struct kionix_accel_driver *acceld)
{
	struct input_dev *input_dev;
	int err;

	input_dev = input_allocate_device();
	if (!input_dev) {
		KMSGERR(&acceld->client->dev, "input_allocate_device failed\n");
		return -ENOMEM;
	}

	acceld->input_dev = input_dev;

	input_dev->open = kionix_accel_input_open;
	input_dev->close = kionix_accel_input_close;
	input_set_drvdata(input_dev, acceld);

	kionix_accel_init_input_device(acceld, input_dev);

	err = input_register_device(acceld->input_dev);
	if (err) {
		KMSGERR(&acceld->client->dev, \
				"%s: input_register_device returned err = %d\n", __func__, err);
		input_free_device(acceld->input_dev);
		return err;
	}

	return 0;
}

/* Returns the enable state of device */
static ssize_t kionix_accel_get_enable(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&acceld->accel_enabled) > 0 ? 1 : 0);
}

/* Allow users to enable/disable the device */
static ssize_t kionix_accel_set_enable(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int enable_count = 1;
	unsigned int enable;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, enable_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: No enable data being read. " \
				"No enable data will be updated.\n", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&enable);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: kstrtouint returned err = %d\n", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &enable);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: strict_strtoul returned err = %d\n", __func__, err);
			goto exit;
		}
		#endif
		KMSGINF(&acceld->client->dev, "kionix_accel_set_enable:%d\n",enable);
		if(enable)
			err = kionix_accel_enable(acceld);
		else
			err = kionix_accel_disable(acceld);
	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/* Returns currently selected poll interval (in ms) */
static ssize_t kionix_accel_get_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", acceld->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kionix_accel_set_delay(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int delay_count = 1;
	unsigned int interval;
	int err = 0;
	KMSGINF(&acceld->client->dev, "kionix_accel_set_delay() ++\n");
	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, delay_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: No delay data being read. " \
				"No delay data will be updated.\n", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&interval);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: kstrtouint returned err = %d\n", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &interval);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: strict_strtoul returned err = %d\n", __func__, err);
			goto exit;
		}
		#endif

		if(acceld->accel_drdy == 1)
			disable_irq(client->irq);

		KMSGINF(&acceld->client->dev, "kionix_accel_set_delay:%d\n",interval);
		/*
		 * Set current interval to the greater of the minimum interval or
		 * the requested interval
		 */
		acceld->poll_interval = max((unsigned int)interval, acceld->accel_pdata.min_interval);
		acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);

		err = acceld->kionix_accel_update_odr(acceld, acceld->poll_interval);

		if(acceld->accel_drdy == 1)
			enable_irq(client->irq);
	}

exit:
	mutex_unlock(&input_dev->mutex);
       KMSGINF(&acceld->client->dev, "kionix_accel_set_delay() --poll_interval= %d\n",acceld->poll_interval);
	return (err < 0) ? err : count;
}

/* Returns the direction of device */
static ssize_t kionix_accel_get_direct(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", acceld->accel_pdata.accel_direction);
}

/* Allow users to change the direction the device */
static ssize_t kionix_accel_set_direct(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	char *buf2;
	const int direct_count = 1;
	unsigned int direction;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	if(kionix_strtok(buf, count, &buf2, direct_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: No direction data being read. " \
				"No direction data will be updated.\n", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
		#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&direction);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: kstrtouint returned err = %d\n", __func__, err);
			goto exit;
		}
		#else
		err = strict_strtoul((const char *)buf2, 10, &direction);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, \
					"%s: strict_strtoul returned err = %d\n", __func__, err);
			goto exit;
		}
		#endif
		KMSGINF(&acceld->client->dev, "kionix_accel_set_direct:%d\n",direction);
		if(direction < 1 || direction > 8)
			KMSGERR(&acceld->client->dev, "%s: invalid direction = %d\n", __func__, (unsigned int) direction);

		else {
			acceld->accel_pdata.accel_direction = (u8) direction;
			kionix_accel_update_direction(acceld);
		}
	}

exit:
	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/* Returns the data output of device */
static ssize_t kionix_accel_get_data(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int x, y, z;

	read_lock(&acceld->rwlock_accel_data);

	x = acceld->accel_data[acceld->axis_map_x];
	y = acceld->accel_data[acceld->axis_map_y];
	z = acceld->accel_data[acceld->axis_map_z];

	read_unlock(&acceld->rwlock_accel_data);
	printk("kxtj2: x=%d,y=%d,z=%d\n",x,y,z);
	return sprintf(buf, "%d %d %d\n", x, y, z);
}

/* Returns the calibration value of the device */
static ssize_t kionix_accel_get_cali(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int calibration[3];

	read_lock(&acceld->rwlock_accel_data);

	calibration[0] = acceld->accel_cali[acceld->axis_map_x];
	calibration[1] = acceld->accel_cali[acceld->axis_map_y];
	calibration[2] = acceld->accel_cali[acceld->axis_map_z];

	read_unlock(&acceld->rwlock_accel_data);

	return sprintf(buf, "%d %d %d\n", calibration[0], calibration[1], calibration[2]);
}

/* Allow users to change the calibration value of the device */
static ssize_t kionix_accel_set_cali(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	struct input_dev *input_dev = acceld->input_dev;
	const int cali_count = 3; /* How many calibration that we expect to get from the string */
	char **buf2;
	long calibration[cali_count];
	int err = 0, i = 0;

	if (KXTJ2_DEBUG) {
		printk("KXTJ2_DEBUG : buf:%s\n", buf);
		printk("KXTJ2_DEBUG : count:%zu\n", count);
	}

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	buf2 = (char **)kzalloc(cali_count * sizeof(char *), GFP_KERNEL);

	if(kionix_strtok(buf, count, buf2, cali_count) < 0) {
		KMSGERR(&acceld->client->dev, \
				"%s: Not enough calibration data being read. " \
				"No calibration data will be updated.\n", __func__);
	}
	else {
		/* Convert string to integers  */
		for(i = 0 ; i < cali_count ; i++) {
			#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
			err = kstrtoint((const char *)*(buf2+i), 10, (int *)&calibration[i]);
			if(err < 0) {
				KMSGERR(&acceld->client->dev, \
						"%s: kstrtoint returned err = %d." \
						"No calibration data will be updated.\n", __func__ , err);
				goto exit;
			}
			#else
			err = strict_strtol((const char *)*(buf2+i), 10, &calibration[i]);
			if(err < 0) {
				KMSGERR(&acceld->client->dev, \
						"%s: strict_strtol returned err = %d." \
						"No calibration data will be updated.\n", __func__ , err);
				goto exit;
			}
			#endif
		}

		write_lock(&acceld->rwlock_accel_data);

		acceld->accel_cali[acceld->axis_map_x] = (int)calibration[0];
		acceld->accel_cali[acceld->axis_map_y] = (int)calibration[1];
		acceld->accel_cali[acceld->axis_map_z] = (int)calibration[2];

		write_unlock(&acceld->rwlock_accel_data);
	}

exit:
	for(i = 0 ; i < cali_count ; i++)
		kfree(*(buf2+i));

	kfree(buf2);

	mutex_unlock(&input_dev->mutex);

	return (err < 0) ? err : count;
}

/* Returns the status of the device */
static ssize_t kionix_accel_get_status(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int kxtj2_wia = 0, ret = 0;

	kxtj2_wia = i2c_smbus_read_byte_data(acceld->client, ACCEL_WHO_AM_I);

	if (KXTJ2_DEBUG)
		printk("KXTJ2_DEBUG : kionix_accel_get_status wia : %d.\n", kxtj2_wia);

	if (kxtj2_wia == KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009)
		ret = 1;
	else
		ret = 0;

	return sprintf(buf, "%d\n", ret);
}

/* Set the calibration offset to cali */
static ssize_t kionix_accel_get_enable_cali(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	int ret = 0;

	isSetCali = false;
	if (asus_accel_set_cali(acceld)) {
		KMSGERR(&acceld->client->dev, \
				"%s: asus_accel_set_cali returned err\n", __func__);
		ret = 0;
	} else
		ret = 1;

	return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, kionix_accel_get_enable, kionix_accel_set_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, kionix_accel_get_delay, kionix_accel_set_delay);
static DEVICE_ATTR(direct, S_IRUGO|S_IWUSR, kionix_accel_get_direct, kionix_accel_set_direct);
static DEVICE_ATTR(data, S_IRUGO, kionix_accel_get_data, NULL);
static DEVICE_ATTR(cali, S_IRUGO|S_IWUSR, kionix_accel_get_cali, kionix_accel_set_cali);
static DEVICE_ATTR(status, S_IRUGO, kionix_accel_get_status, NULL);
static DEVICE_ATTR(enable_cali, S_IRUGO, kionix_accel_get_enable_cali, NULL);

static struct attribute *kionix_accel_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_direct.attr,
	&dev_attr_data.attr,
	&dev_attr_cali.attr,
	&dev_attr_status.attr,
	&dev_attr_enable_cali.attr,
	NULL
};

static struct attribute_group kionix_accel_attribute_group = {
	.attrs = kionix_accel_attributes
};

static int  kionix_verify(struct kionix_accel_driver *acceld)
{
	int retval = i2c_smbus_read_byte_data(acceld->client, ACCEL_WHO_AM_I);

#if KIONIX_KMSG_INF
	switch (retval) {
		case KIONIX_ACCEL_WHO_AM_I_KXTE9:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTE9.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTF9:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTF9.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTI9_1001:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTI9-1001.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTIK_1004:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTIK-1004.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1005:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTJ9-1005.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1007:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTJ9-1007.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXCJ9_1008:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXCJ9-1008.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXTJ2-1009.\n");
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXCJK_1013:
			KMSGINF(&acceld->client->dev, "this accelerometer is a KXCJK-1013.\n");
			break;
		default:
			break;
	}
#endif

	return retval;
}

static int kionix_accel_suspend(struct i2c_client *client, pm_message_t mesg)
{
	long remaining;
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	printk("%s: kionix_accel_suspend +++ \n", __func__);

	mutex_lock(&acceld->mutex_earlysuspend);

	/* Only continue to suspend if enable did not intervene */
	if(atomic_read(&acceld->accel_suspend_continue) > 0) {
		/* Make sure that the sensor had successfully disabled before suspending it */
		if(atomic_read(&acceld->accel_enabled) > 0) {
			KMSGINF(&acceld->client->dev, "%s: waiting for disable\n", __func__);
			remaining = wait_event_interruptible_timeout(acceld->wqh_suspend, \
					atomic_read(&acceld->accel_enabled) < 1, \
					msecs_to_jiffies(KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT));

			if(atomic_read(&acceld->accel_enabled) > 0) {
				KMSGERR(&acceld->client->dev, "%s: timeout waiting for disable\n", __func__);
			}
		}

		kionix_accel_power_off(acceld);

		atomic_set(&acceld->accel_suspended, 1);
	}
	mutex_unlock(&acceld->mutex_earlysuspend);

	printk("%s: kionix_accel_suspend --- \n", __func__);

	return 0;
}

static int kionix_accel_resume(struct i2c_client *client)
{
	int err;
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);
	printk("%s: kionix_accel_resume +++ \n", __func__);
	mutex_lock(&acceld->mutex_resume);
	if(atomic_read(&acceld->accel_suspended) == 1) {
		err = kionix_accel_power_on(acceld);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on returned err = %d\n", __func__, err);
			goto exit;
		}

		/* Only needs to reinitialized the registers if Vdd is pulled low during suspend */
		if(err > 0) {
			err = acceld->kionix_accel_power_on_init(acceld);
			if (err) {
				KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on_init returned err = %d\n", __func__, err);
				goto exit;
			}
		}

		atomic_set(&acceld->accel_suspended, 0);
	}

	wake_up_interruptible(&acceld->wqh_suspend);

exit:
	mutex_unlock(&acceld->mutex_resume);
	i2c_smbus_read_byte_data(acceld->client, ACCEL_GRP4_INT_REL);
	printk("%s: kionix_accel_resume --- \n", __func__);
	return 0;
}

#ifdef    CONFIG_HAS_EARLYSUSPEND
void kionix_accel_earlysuspend_suspend(struct early_suspend *h)
{
	struct kionix_accel_driver *acceld = container_of(h, struct kionix_accel_driver, early_suspend);
	long remaining;

	mutex_lock(&acceld->mutex_earlysuspend);

	/* Only continue to suspend if enable did not intervene */
	if(atomic_read(&acceld->accel_suspend_continue) > 0) {
		/* Make sure that the sensor had successfully disabled before suspending it */
		if(atomic_read(&acceld->accel_enabled) > 0) {
			KMSGINF(&acceld->client->dev, "%s: waiting for disable\n", __func__);
			remaining = wait_event_interruptible_timeout(acceld->wqh_suspend, \
					atomic_read(&acceld->accel_enabled) < 1, \
					msecs_to_jiffies(KIONIX_ACCEL_EARLYSUSPEND_TIMEOUT));

			if(atomic_read(&acceld->accel_enabled) > 0) {
				KMSGERR(&acceld->client->dev, "%s: timeout waiting for disable\n", __func__);
			}
		}

		kionix_accel_power_off(acceld);

		atomic_set(&acceld->accel_suspended, 1);
	}

	mutex_unlock(&acceld->mutex_earlysuspend);

	return;
}

void kionix_accel_earlysuspend_resume(struct early_suspend *h)
{
	struct kionix_accel_driver *acceld = container_of(h, struct kionix_accel_driver, early_suspend);
	int err;

	mutex_lock(&acceld->mutex_resume);

	if(atomic_read(&acceld->accel_suspended) == 1) {
		err = kionix_accel_power_on(acceld);
		if (err < 0) {
			KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on returned err = %d\n", __func__, err);
			goto exit;
		}

		/* Only needs to reinitialized the registers if Vdd is pulled low during suspend */
		if(err > 0) {
			err = acceld->kionix_accel_power_on_init(acceld);
			if (err) {
				KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on_init returned err = %d\n", __func__, err);
				goto exit;
			}
		}

		atomic_set(&acceld->accel_suspended, 0);
	}

	wake_up_interruptible(&acceld->wqh_suspend);

exit:
	mutex_unlock(&acceld->mutex_resume);

	return;
}
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int  kionix_accel_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct kionix_accel_platform_data *accel_pdata = &accel_platform_data;
	struct kionix_accel_driver *acceld;
	int err, int_irq = 0;
	struct proc_dir_entry *proc_dir, *proc_entry;

	client->dev.platform_data = accel_pdata;

	if (KXTJ2_DEBUG)
		printk("KXTJ2_DEBUG : enter kionix_accel_probe.\n");
	if(accel_pdata->accel_irq_use_drdy){
		accel_pdata->int_gpio = acpi_get_gpio_by_index(&client->dev, 0, NULL);
		int_irq = gpio_to_irq(accel_pdata->int_gpio);
		printk("KXTJ2_DEBUG : gpio[%d] to irq[%d]\n",accel_pdata->int_gpio , int_irq);
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		KMSGERR(&client->dev, "client is not i2c capable. Abort.\n");
		return -ENXIO;
	}

	if (!accel_pdata) {
		KMSGERR(&client->dev, "platform data is NULL. Abort.\n");
		return -EINVAL;
	}

	acceld = kzalloc(sizeof(*acceld), GFP_KERNEL);
	if (acceld == NULL) {
		KMSGERR(&client->dev, \
			"failed to allocate memory for module data. Abort.\n");
		return -ENOMEM;
	}

	acceld->client = client;
	acceld->accel_pdata = *accel_pdata;

	if (KXTJ2_DEBUG)
		printk("KXTJ2_DEBUG : addr:%x\n", acceld->client->addr);

	i2c_set_clientdata(client, acceld);

/* Add API for Sleeping Beauty test tool */
#ifdef	CONFIG_SLEEPING_BEAUTY
	i2c_data = kzalloc(sizeof(struct i2c_register_data), GFP_KERNEL);
	if (i2c_data == NULL)
	{
		printk("Allocate i2c_data fail\n");
		return -ENOMEM;
	}
	i2c_data->client = client;
#endif	// CONFIG_SLEEPING_BEAUTY

	err = kionix_accel_power_on(acceld);
	if (err < 0)
		goto err_free_mem;

	if (KXTJ2_DEBUG)
		printk("KXTJ2_DEBUG : kionix_accel_power_on.\n");

	if (accel_pdata->init) {
		err = accel_pdata->init();
		if (err < 0)
			goto err_accel_pdata_power_off;
	}

	if (KXTJ2_DEBUG)
		printk("KXTJ2_DEBUG : accel_pdata->init.\n");

	err = kionix_verify(acceld);
	if (err < 0) {
		KMSGERR(&acceld->client->dev, "%s: kionix_verify returned err = %d. Abort.\n", __func__, err);
		goto err_accel_pdata_exit;
	}

	/* Setup group specific configuration and function callback */
	switch (err) {
		case KIONIX_ACCEL_WHO_AM_I_KXTE9:
			acceld->accel_group = KIONIX_ACCEL_GRP1;
			acceld->accel_registers = kzalloc(sizeof(u8)*accel_grp1_regs_count, GFP_KERNEL);
			if (acceld->accel_registers == NULL) {
				KMSGERR(&client->dev, \
					"failed to allocate memory for accel_registers. Abort.\n");
				goto err_accel_pdata_exit;
			}
			acceld->accel_drdy = 0;
			acceld->kionix_accel_report_accel_data	= kionix_accel_grp1_report_accel_data;
			acceld->kionix_accel_update_odr			= kionix_accel_grp1_update_odr;
			acceld->kionix_accel_power_on_init		= kionix_accel_grp1_power_on_init;
			acceld->kionix_accel_operate			= kionix_accel_grp1_operate;
			acceld->kionix_accel_standby			= kionix_accel_grp1_standby;
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTF9:
		case KIONIX_ACCEL_WHO_AM_I_KXTI9_1001:
		case KIONIX_ACCEL_WHO_AM_I_KXTIK_1004:
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1005:
			if(err == KIONIX_ACCEL_WHO_AM_I_KXTIK_1004)
				acceld->accel_group = KIONIX_ACCEL_GRP3;
			else
				acceld->accel_group = KIONIX_ACCEL_GRP2;
			acceld->accel_registers = kzalloc(sizeof(u8)*accel_grp2_regs_count, GFP_KERNEL);
			if (acceld->accel_registers == NULL) {
				KMSGERR(&client->dev, \
					"failed to allocate memory for accel_registers. Abort.\n");
				goto err_accel_pdata_exit;
			}
			switch(acceld->accel_pdata.accel_res) {
				case KIONIX_ACCEL_RES_6BIT:
				case KIONIX_ACCEL_RES_8BIT:
					acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_RES_8BIT;
					break;
				case KIONIX_ACCEL_RES_12BIT:
				default:
					acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_RES_12BIT;
					break;
			}
			if(acceld->accel_pdata.accel_irq_use_drdy && client->irq) {
				acceld->accel_registers[accel_grp2_int_ctrl] |= ACCEL_GRP2_IEN | ACCEL_GRP2_IEA;
				acceld->accel_registers[accel_grp2_ctrl_reg1] |= ACCEL_GRP2_DRDYE;
				acceld->accel_drdy = 1;
			}
			else
				acceld->accel_drdy = 0;
			kionix_accel_grp2_update_g_range(acceld);
			acceld->kionix_accel_report_accel_data	= kionix_accel_grp2_report_accel_data;
			acceld->kionix_accel_update_odr			= kionix_accel_grp2_update_odr;
			acceld->kionix_accel_power_on_init		= kionix_accel_grp2_power_on_init;
			acceld->kionix_accel_operate			= kionix_accel_grp2_operate;
			acceld->kionix_accel_standby			= kionix_accel_grp2_standby;
			break;
		case KIONIX_ACCEL_WHO_AM_I_KXTJ9_1007:
		case KIONIX_ACCEL_WHO_AM_I_KXCJ9_1008:
		case KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009:
		case KIONIX_ACCEL_WHO_AM_I_KXCJK_1013:
			if(err == KIONIX_ACCEL_WHO_AM_I_KXTJ2_1009)
				acceld->accel_group = KIONIX_ACCEL_GRP5;
			else if(err == KIONIX_ACCEL_WHO_AM_I_KXCJK_1013)
				acceld->accel_group = KIONIX_ACCEL_GRP6;
			else
				acceld->accel_group = KIONIX_ACCEL_GRP4;
			acceld->accel_registers = kzalloc(sizeof(u8)*accel_grp4_regs_count, GFP_KERNEL);
			if (acceld->accel_registers == NULL) {
				KMSGERR(&client->dev, \
					"failed to allocate memory for accel_registers. Abort.\n");
				goto err_accel_pdata_exit;
			}
			switch(acceld->accel_pdata.accel_res) {
				case KIONIX_ACCEL_RES_6BIT:
				case KIONIX_ACCEL_RES_8BIT:
					acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_RES_8BIT;
					break;
				case KIONIX_ACCEL_RES_12BIT:
				default:
					acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_RES_12BIT;
					break;
			}
			if(acceld->accel_pdata.accel_irq_use_drdy && int_irq) {
				acceld->accel_registers[accel_grp4_int_ctrl] |= ACCEL_GRP4_IEN | ACCEL_GRP4_IEA;
				acceld->accel_registers[accel_grp4_ctrl_reg1] |= ACCEL_GRP4_DRDYE;
				acceld->accel_drdy = 1;
				printk("Probe() init accel_register value\n");
			}
			else
				acceld->accel_drdy = 0;
			kionix_accel_grp4_update_g_range(acceld);
			acceld->kionix_accel_report_accel_data	= kionix_accel_grp4_report_accel_data;
			acceld->kionix_accel_update_odr			= kionix_accel_grp4_update_odr;
			acceld->kionix_accel_power_on_init		= kionix_accel_grp4_power_on_init;
			acceld->kionix_accel_operate			= kionix_accel_grp4_operate;
			acceld->kionix_accel_standby			= kionix_accel_grp4_standby;
			break;
		default:
			KMSGERR(&acceld->client->dev, \
					"%s: unsupported device, who am i = %d. Abort.\n", __func__, err);
			goto err_accel_pdata_exit;
	}

	err = kionix_accel_setup_input_device(acceld);
	if (err)
		goto err_free_accel_registers;

	atomic_set(&acceld->accel_suspended, 0);
	atomic_set(&acceld->accel_suspend_continue, 1);
	atomic_set(&acceld->accel_enabled, 0);
	atomic_set(&acceld->accel_input_event, 0);
	atomic_set(&acceld->accel_enable_resume, 0);

	mutex_init(&acceld->mutex_earlysuspend);
	mutex_init(&acceld->mutex_resume);
	rwlock_init(&acceld->rwlock_accel_data);

	acceld->poll_interval = acceld->accel_pdata.poll_interval;
	acceld->poll_delay = msecs_to_jiffies(acceld->poll_interval);
	acceld->kionix_accel_update_odr(acceld, acceld->poll_interval);
	kionix_accel_update_direction(acceld);

	acceld->accel_workqueue = create_workqueue("Kionix Accel Workqueue");
	INIT_DELAYED_WORK(&acceld->accel_work, kionix_accel_work);
	init_waitqueue_head(&acceld->wqh_suspend);

	if (acceld->accel_drdy) {
		if (KXTJ2_DEBUG) {
			printk("KXTJ2_DEBUG : int_gpio[%d] to int_irq[%d]\n",
					accel_pdata->int_gpio, int_irq);
		}

		err = request_threaded_irq(int_irq, NULL, kionix_accel_isr, \
			  IRQF_TRIGGER_RISING | IRQF_ONESHOT, \
			  KIONIX_ACCEL_IRQ, acceld);

		if (err) {
			KMSGERR(&acceld->client->dev, "%s: request_threaded_irq returned err = %d\n", __func__, err);
			KMSGERR(&acceld->client->dev, "%s: running in software polling mode instead\n", __func__);
			acceld->accel_drdy = 0;
		}
		else
			KMSGINF(&acceld->client->dev, "running in hardware interrupt mode\n");

	} else {
		KMSGINF(&acceld->client->dev, "running in software polling mode\n");
	}

	err = acceld->kionix_accel_power_on_init(acceld);
	if (err) {
		KMSGERR(&acceld->client->dev, "%s: kionix_accel_power_on_init returned err = %d. Abort.\n", __func__, err);
		goto err_free_irq;
	}

	err = sysfs_create_group(&client->dev.kobj, &kionix_accel_attribute_group);
	if (err) {
		KMSGERR(&acceld->client->dev, "%s: sysfs_create_group returned err = %d. Abort.\n", __func__, err);
		goto err_free_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* The higher the level, the earlier it resume, and the later it suspend */
	acceld->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 50;
	acceld->early_suspend.suspend = kionix_accel_earlysuspend_suspend;
	acceld->early_suspend.resume = kionix_accel_earlysuspend_resume;
	register_early_suspend(&acceld->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

	/***** For i2c stress test ++ *****/
	err = misc_register(&kxtj2_misc_dev);
	if (err) {
		KMSGERR(&acceld->client->dev, "%s: register misc deivce failed, error = %d\n", \
			__func__, err);
		goto err_free_irq;
	}

	kxtj2_acceld = acceld;
	kxtj2_stress_test_work_queue = create_singlethread_workqueue("kxtj2_stress_test_wq");
	if (!kxtj2_stress_test_work_queue) {
		KMSGERR(&acceld->client->dev, "%s: unable to create i2c stress test workqueue\n", \
			__func__);
		goto err_misc_deregister;
	}
	INIT_DELAYED_WORK(&kxtj2_stress_test_poll_work, asus_kxtj2_stress_test_poll);
	/***** For i2c stress test -- *****/

	if (KXTJ2_DEBUG)
		printk("KXTJ2_DEBUG : probe g sensor successfully.\n");

	return 0;

	/***** For i2c stress test ++ *****/
err_misc_deregister:
	misc_deregister(&kxtj2_misc_dev);
	/***** For i2c stress test -- *****/

err_free_irq:
	if (acceld->accel_drdy)
		free_irq(client->irq, acceld);
	destroy_workqueue(acceld->accel_workqueue);
	input_unregister_device(acceld->input_dev);
err_free_accel_registers:
	kfree(acceld->accel_registers);
err_accel_pdata_exit:
	if (accel_pdata->exit)
		accel_pdata->exit();
err_accel_pdata_power_off:
	kionix_accel_power_off(acceld);
err_free_mem:
	kfree(acceld);
	return err;
}

static int  kionix_accel_remove(struct i2c_client *client)
{
	struct kionix_accel_driver *acceld = i2c_get_clientdata(client);

	/***** For i2c stress test ++ *****/
	if (kxtj2_stress_test_work_queue)
		destroy_workqueue(kxtj2_stress_test_work_queue);
	misc_deregister(&kxtj2_misc_dev);
	/***** For i2c stress test -- *****/

#ifdef    CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&acceld->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */
	sysfs_remove_group(&client->dev.kobj, &kionix_accel_attribute_group);
	if (acceld->accel_drdy)
		free_irq(client->irq, acceld);
	destroy_workqueue(acceld->accel_workqueue);
	input_unregister_device(acceld->input_dev);
	kfree(acceld->accel_registers);
	if (acceld->accel_pdata.exit)
		acceld->accel_pdata.exit();
	kionix_accel_power_off(acceld);
	kfree(acceld);

	return 0;
}


static int __init kxtj2_attach(struct i2c_adapter *adapter)
{
	struct i2c_client *kxtj2_client;

	if(adapter->nr != KXTJ2_I2C_ADAPTER)
	{
		return 0;
	}

	kxtj2_client = i2c_new_device(adapter, &kxtj2_board_info);
	if (!kxtj2_client)
		return -ENODEV;
	/*
	 * We know the driver is already loaded, so the device should be
	 * already bound. If not it means binding failed, and then there
	 * is no point in keeping the device instantiated.
	 */
	if (!kxtj2_client->driver) {
		i2c_unregister_device(kxtj2_client);
		kxtj2_client = NULL;
		return -ENODEV;
	}

	/*
	 * Let i2c-core delete that device on driver removal.
	 * This is safe because i2c-core holds the core_lock mutex for us.
	 */
	list_add_tail(&kxtj2_client->detected,
			&kxtj2_client->driver->clients);
	return 0;
}

static const struct i2c_device_id kionix_accel_id[] = {
	{ KIONIX_ACCEL_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, kionix_accel_id);

static const struct acpi_device_id kionix_acpi_match[] = {
	{"KXJ2109", 0},
	{"KXTJ2100", 0}, // ACPI name for TF103C
	{"", 0}
};
MODULE_DEVICE_TABLE(acpi, kionix_acpi_match);

static struct i2c_driver kionix_accel_driver = {
	.probe		= kionix_accel_probe,
	.suspend	= kionix_accel_suspend,
	.resume		= kionix_accel_resume,
	.remove		= kionix_accel_remove,
	.id_table	= kionix_accel_id,
	.driver = {
		.name	= KIONIX_ACCEL_NAME,
		.owner	= THIS_MODULE,
		.acpi_match_table = ACPI_PTR(kionix_acpi_match),
	},
	//.attach_adapter = kxtj2_attach,
};

static int __init kionix_accel_init(void)
{
	printk("KXTJ2_DEBUG : add i2c driver %s\n", __func__);
	return i2c_add_driver(&kionix_accel_driver);
}
module_init(kionix_accel_init);

static void __exit kionix_accel_exit(void)
{
	i2c_del_driver(&kionix_accel_driver);
}
module_exit(kionix_accel_exit);

MODULE_DESCRIPTION("Kionix accelerometer driver");
MODULE_AUTHOR("Kuching Tan <kuchingtan@kionix.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.3.0");
