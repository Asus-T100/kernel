/*
 * An I2C driver for SMSC CAP1106.
 *
 * Copyright (c) 2013, ASUSTek Corporation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>

#define CAP_SDEV_NAME "ril_proximity"

#define CAP1106_I2C_NAME "cap1106"
#define CAP1106_ACPI_NAME "CAP1106"
#define CAP1106_SAR_DET_GPIO_NAME "SAR_DET_3G"

/*
 * CAP1106 Register Name & address
 */
#define MAIN_CONTROL			0x00
#define SENSOR_INPUT_STATUS		0x03
#define SENSOR_INPUT_6_DELTA_COUNT	0x15
#define SENSITIVITY_CONTROL		0x1F
#define CONFIGURATION			0x20
#define CONFIGURATION2			0x44
#define SENSOR_INPUT_ENABLE		0x21
#define SENSOR_INPUT_CONFIGURATION	0x22
#define AVERAGING_AND_SAMPLING_CONFING	0x24
#define CALIBRATION_ACTIVATE		0x26
#define INTERRUPT_ENABLE		0x27
#define REPEAT_RATE_ENABLE		0x28
#define MULTIPLE_TOUCH_CONFIG		0x2A
#define SENSOR_INPUT_6_THRESHOLD	0x35
#define SENSOR_INPUT_NOISE_THRESHOLD	0x38
#define SENSOR_INPUT_6_BASE_COUNT	0x55

#define DELAY_MS_NORMAL    msecs_to_jiffies(1000) //1Hz
#define DELAY_MS_HEAVY     msecs_to_jiffies(200)  //5Hz

struct cap1106_data {
	struct attribute_group attrs;
	struct i2c_client *client;
	struct workqueue_struct *cap_wq;
	struct delayed_work work;
	struct delayed_work checking_work;
	int enable;
	int obj_detect;
	int overflow_status;
	int app2mdm_enable;
	int sar_det_gpio;
	char *sar_det_gpio_name;
	int irq;
	struct miscdevice cap1106_misc_dev; // for I2C stress test
};

static DEFINE_MUTEX(cap_mtx);
static struct cap1106_data *pivate_data;
static struct switch_dev cap_sdev;

static int is_wood_sensitivity = 0;
//static int ac2 = 0; // Accumulated Count Ch2
static int ac6 = 0;   // Accumulated Count Ch6
static int ac_limit = 10;
static int force_enable = 1;
static bool bSkip_Checking = false;

static ssize_t show_attrs_handler(struct device *dev,
				struct device_attribute *devattr, char *buf);
static ssize_t store_attrs_handler(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count);

/*
 * I2C stress test
 */
#define CAP1106_IOC_MAGIC 0xF3
#define CAP1106_IOC_MAXNR 2
#define CAP1106_POLL_DATA _IOR(CAP1106_IOC_MAGIC, 2, int)

#define CAP1106_IOCTL_START_HEAVY	2
#define CAP1106_IOCTL_START_NORMAL	1
#define CAP1106_IOCTL_END		0

#define START_NORMAL   msecs_to_jiffies(200)  //5HZ
#define START_HEAVY    msecs_to_jiffies(5)    //200HZ

static int stress_test_poll_mode = 0;
static struct delayed_work cap1106_stress_test_poll_work;
static struct workqueue_struct *cap1106_stress_test_work_queue;

static s32 cap1106_read_reg(struct i2c_client *client, u8 command)
{
	return i2c_smbus_read_byte_data(client, command);
}

static s32 cap1106_write_reg(struct i2c_client *client, u8 command, u8 value)
{
	return i2c_smbus_write_byte_data(client, command, value);
}

static void cap1106_enable_sensor(struct i2c_client *client, int enable)
{
	long reg_value;
	struct cap1106_data *data;

	data = i2c_get_clientdata(client);
	if (data->enable != enable) {
		reg_value = cap1106_read_reg(client, MAIN_CONTROL);
		if (enable) {
			cap1106_write_reg(client, MAIN_CONTROL, reg_value & 0xCE);

			// Time to first conversion is 200ms (Max)
			// if checking_work_function is in progress,
			// checking_work_function must skip.
			bSkip_Checking = false;
			queue_delayed_work(data->cap_wq, &data->work, DELAY_MS_HEAVY);
			enable_irq(data->irq);
			queue_delayed_work(data->cap_wq, &data->checking_work,
								DELAY_MS_NORMAL);
		} else {
			disable_irq(data->irq);

			// if checking_work_function is in progress
			// checking_work_function must skip.
			bSkip_Checking = true;

//			cancel_delayed_work_sync(&data->work);
//			cancel_delayed_work_sync(&data->checking_work);
//			flush_workqueue(data->cap_wq);
			switch_set_state(&cap_sdev, 0);
			cap1106_write_reg(client, MAIN_CONTROL, reg_value | 0x30);
		}
		data->enable = enable;
		pr_info("cap1106: data->enable: %d\n", data->enable);
	}
}

DEVICE_ATTR(obj_detect, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensitivity, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_gain, 0644, show_attrs_handler, store_attrs_handler);
//DEVICE_ATTR(sensor_input_2_delta_count, 0644, show_attrs_handler, NULL);
//DEVICE_ATTR(sensor_input_2_th, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_input_6_delta_count, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensor_input_6_th, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_input_noise_th,0644,show_attrs_handler,store_attrs_handler);
DEVICE_ATTR(sensor_input_status, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensing_cycle, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_onoff, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_recal, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_app2mdm_sar, 0644, NULL, store_attrs_handler);
DEVICE_ATTR(sensor_main, 0644, show_attrs_handler, NULL);

static struct attribute *cap1106_attr_deb[] = {
	&dev_attr_obj_detect.attr,                 // 1
	&dev_attr_sensitivity.attr,                // 2
	&dev_attr_sensor_gain.attr,                // 3
//	&dev_attr_sensor_input_2_delta_count.attr, // 4
//	&dev_attr_sensor_input_2_th.attr,          // 5
	&dev_attr_sensor_input_6_delta_count.attr, // 6
	&dev_attr_sensor_input_6_th.attr,          // 7
	&dev_attr_sensor_input_noise_th.attr,      // 8
	&dev_attr_sensor_input_status.attr,        // 9
	&dev_attr_sensing_cycle.attr,              // 10
	&dev_attr_sensor_onoff.attr,               // 11
	&dev_attr_sensor_recal.attr,               // 12
	&dev_attr_sensor_app2mdm_sar.attr,         // 13
	&dev_attr_sensor_main.attr,                // 14
	NULL
};

static ssize_t show_attrs_handler(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cap1106_data *data = i2c_get_clientdata(client);
	const char *attr_name = devattr->attr.name;
	int ret = -1;

	pr_debug("cap1106: devattr->attr->name: %s\n", devattr->attr.name);

	mutex_lock(&cap_mtx);
	if (data->enable) {
		if (!strcmp(attr_name, dev_attr_obj_detect.attr.name)) {
			ret = sprintf(buf, "%d\n", data->obj_detect);
		} else if (!strcmp(attr_name, dev_attr_sensitivity.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, SENSITIVITY_CONTROL));
		} else if (!strcmp(attr_name, dev_attr_sensor_gain.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, MAIN_CONTROL) >> 6);
/*
		} else if (!strcmp(attr_name,
				dev_attr_sensor_input_2_delta_count.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, 0x11));
		} else if (!strcmp(attr_name, dev_attr_sensor_input_2_th.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, 0x31));
*/
		} else if (!strcmp(attr_name,
				dev_attr_sensor_input_6_delta_count.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, SENSOR_INPUT_6_DELTA_COUNT));
		} else if (!strcmp(attr_name, dev_attr_sensor_input_6_th.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, SENSOR_INPUT_6_THRESHOLD));
		} else if (!strcmp(attr_name,
				dev_attr_sensor_input_noise_th.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, SENSOR_INPUT_NOISE_THRESHOLD));
		} else if (!strcmp(attr_name, dev_attr_sensor_input_status.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, SENSOR_INPUT_STATUS));
		} else if (!strcmp(attr_name, dev_attr_sensing_cycle.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, AVERAGING_AND_SAMPLING_CONFING));
		} else if (!strcmp(attr_name, dev_attr_sensor_onoff.attr.name)) {
			ret = sprintf(buf, "%d\n", data->enable);
		} else if (!strcmp(attr_name, dev_attr_sensor_recal.attr.name)) {
			ret = sprintf(buf,
					cap1106_read_reg(client, CALIBRATION_ACTIVATE) == 0x0 ?
					"OK\n" : "FAIL\n");
		} else if (!strcmp(attr_name, dev_attr_sensor_main.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, MAIN_CONTROL));
		}
	} else {
		if (!strcmp(attr_name, dev_attr_sensor_main.attr.name)) {
			ret = sprintf(buf, "%02X\n",
					cap1106_read_reg(client, MAIN_CONTROL));
		} else {
			ret = sprintf(buf, "SENSOR DISABLED\n");
		}
	}

	mutex_unlock(&cap_mtx);

	return ret;
}

static ssize_t store_attrs_handler(struct device *dev,
		struct device_attribute *devattr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cap1106_data *data = i2c_get_clientdata(client);
	const char *attr = devattr->attr.name;
	unsigned long value;

	if (strict_strtoul(buf, 16, &value)) return -EINVAL;

	pr_debug("cap1106: devattr->attr->name: %s, value: 0x%lX\n",
			devattr->attr.name, value);

	mutex_lock(&cap_mtx);
	if (data->enable) {
		if (!strcmp(attr, dev_attr_sensitivity.attr.name)) {
			cap1106_write_reg(client, SENSITIVITY_CONTROL, value & 0x7F);
		} else if (!strcmp(attr, dev_attr_sensor_gain.attr.name)) {
			cap1106_write_reg(client, MAIN_CONTROL,
					(cap1106_read_reg(client, MAIN_CONTROL) & 0x3F) |
					((value & 0x03) << 6));
/*
		} else if (!strcmp(attr, dev_attr_sensor_input_2_th.attr.name)) {
			cap1106_write_reg(client, 0x31, value & 0x7F);
*/
		} else if (!strcmp(attr, dev_attr_sensor_input_6_th.attr.name)) {
			cap1106_write_reg(client,
					SENSOR_INPUT_6_THRESHOLD, value & 0x7F);
		} else if (!strcmp(attr, dev_attr_sensor_input_noise_th.attr.name)) {
			cap1106_write_reg(client,
					SENSOR_INPUT_NOISE_THRESHOLD, value & 0x03);
		} else if (!strcmp(attr, dev_attr_sensing_cycle.attr.name)) {
			cap1106_write_reg(client,
					AVERAGING_AND_SAMPLING_CONFING, value & 0x7F);
		} else if (!strcmp(attr, dev_attr_sensor_onoff.attr.name)) {
			if (value == 0) {
				force_enable = 0;
				cap1106_enable_sensor(client, 0);
			}
		} else if (!strcmp(attr, dev_attr_sensor_recal.attr.name)) {
			cap1106_write_reg(client, CALIBRATION_ACTIVATE, 0x22);
		} else if (!strcmp(attr, dev_attr_sensor_app2mdm_sar.attr.name)) {
			gpio_set_value(data->sar_det_gpio, value);
		}
	} else {
		if (!strcmp(attr, dev_attr_sensor_onoff.attr.name)) {
			if (value == 1) {
				force_enable = 1;
				cap1106_enable_sensor(client, 1);
			}
		}
	}
	mutex_unlock(&cap_mtx);

	return strnlen(buf, count);;
}

/*******************************
 * Callbacks for switch device *
 *******************************/
static ssize_t print_cap_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "prox_sar_det");
}

static ssize_t print_cap_state(struct switch_dev *sdev, char *buf)
{
	if (switch_get_state(sdev))
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

/*****************************
 * I2C stress test functions *
 *****************************/
static void cap1106_get_input_values(struct i2c_client *client) {

	int status;
	int bc6, dc6; // Base Count Ch6, Delta Count Ch6
	status = cap1106_read_reg(client, SENSOR_INPUT_STATUS);
	dc6 = cap1106_read_reg(client, SENSOR_INPUT_6_DELTA_COUNT);
	bc6 = cap1106_read_reg(client, SENSOR_INPUT_6_BASE_COUNT);

	printk("cap1106: [%s] status: 0x%02X, bc6=0x%02X, dc6=0x%02X\n",
						__func__, status, bc6, dc6);
}

static void cap1106_stress_test_poll(struct work_struct * work)
{
	cap1106_get_input_values(pivate_data->client);
	if(stress_test_poll_mode == 0)
		msleep(5);

	queue_delayed_work(cap1106_stress_test_work_queue,
			&cap1106_stress_test_poll_work, stress_test_poll_mode);
}

long cap1106_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;

	pr_debug("------> CAP1106 IOCTL for stress test <------\n");
	if (_IOC_TYPE(cmd) != CAP1106_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > CAP1106_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case CAP1106_POLL_DATA:
			if (arg == CAP1106_IOCTL_START_HEAVY) {
				printk("cap1106: ioctl heavy\n");
				stress_test_poll_mode = START_HEAVY;
				queue_delayed_work(cap1106_stress_test_work_queue,
						&cap1106_stress_test_poll_work, stress_test_poll_mode);
			} else if (arg == CAP1106_IOCTL_START_NORMAL) {
				printk("cap1106: ioctl normal\n");
				stress_test_poll_mode = START_NORMAL;
				queue_delayed_work(cap1106_stress_test_work_queue,
						&cap1106_stress_test_poll_work, stress_test_poll_mode);
			} else if (arg == CAP1106_IOCTL_END) {
				printk("cap1106: ioctl end\n");
				cancel_delayed_work_sync(&cap1106_stress_test_poll_work);
			} else {
				return -ENOTTY;
			}
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return 0;
}

int cap1106_open(struct inode *inode, struct file *filp)
{
	pr_debug("cap1106: %s\n", __func__);
	return 0;
}

int cap1106_release(struct inode *inode, struct file *filp)
{
	pr_debug("cap1106: %s\n", __func__);
	return 0;
}

struct file_operations cap1106_fops = {
	.owner = THIS_MODULE,
	.open = cap1106_open,
	.release = cap1106_release,
	.unlocked_ioctl = cap1106_ioctl,
};

/*****************************
 * I2C stress test functions *
 *****************************/
static void cap1106_work_function(struct work_struct *work)
{
	int status;
	int bc6, dc6; // Base Count Ch6, Delta Count Ch6
//	int bc2, bc6; // Base Count Ch2, Ch6
//	int dc2, dc6; // Delta Count Ch2, Ch6
	struct cap1106_data *data =
		container_of((struct delayed_work *)work, struct cap1106_data, work);

	disable_irq(data->irq);
	// Clear INT, keep GAIN, STBY, DSLEEP
	cap1106_write_reg(data->client, MAIN_CONTROL,
			cap1106_read_reg(data->client, MAIN_CONTROL) & 0xF0);
	status = cap1106_read_reg(data->client, SENSOR_INPUT_STATUS);
//	dc2 = cap1106_read_reg(data->client, 0x11);
	dc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_DELTA_COUNT);
//	bc2 = cap1106_read_reg(data->client, 0x51);
	bc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_BASE_COUNT);

	pr_debug("cap1106: status: 0x%02X, bc6=0x%02X, dc6=0x%02X\n",
						status, bc6, dc6);

	if (is_wood_sensitivity == 0) {
		data->obj_detect = (status & 0x20) ? 1 : 0;
		pr_err("cap1106: obj_detect: %d\n", data->obj_detect);
		switch_set_state(&cap_sdev, data->obj_detect);
		if (data->app2mdm_enable) {
			gpio_set_value(data->sar_det_gpio, data->obj_detect);
		}
		if (status == 0x20 && dc6 == 0x7F) {
			pr_err("cap1106: is_wood_sensitivity = 1\n");
			//set sensitivity and threshold for wood touch
			cap1106_write_reg(data->client, SENSITIVITY_CONTROL, 0x4F);
//			cap1106_write_reg(data->client, 0x31, 0x5F);
			cap1106_write_reg(data->client, SENSOR_INPUT_6_THRESHOLD, 0x5F);
			is_wood_sensitivity = 1;
			data->overflow_status = status;
//			ac2 = 0;
			ac6 = 0;
		} else {
//			if (dc2 >= 0x0A && dc2 <= 0x3F) ac2++;
			if (dc6 >= 0x0A && dc6 <= 0x3F) ac6++;

			pr_debug("cap1106: ac6=%d\n", ac6);
//			if (ac2 >= ac_limit || ac6 >= ac_limit) {
			if (ac6 >= ac_limit) {
				pr_debug("+++ FORCE RECALIBRATION +++\n");
				cap1106_write_reg(data->client, CALIBRATION_ACTIVATE, 0x20);
//				ac2 = 0;
				ac6 = 0;
			}
		}
	}
	enable_irq(data->irq);
}

static irqreturn_t cap1106_interrupt_handler(int irq, void *dev)
{
	struct cap1106_data *data;
	pr_debug("cap1106: handle interrupt!!\n");

	data = i2c_get_clientdata(dev);
	queue_delayed_work(data->cap_wq, &data->work, 0);
	return IRQ_HANDLED;
}

static int cap1106_config_irq(struct i2c_client *client)
{
	int ret = 0;
	struct cap1106_data *data;

	data = i2c_get_clientdata(client);
	if (gpio_is_valid(data->sar_det_gpio)) {
		ret = gpio_request(data->sar_det_gpio, data->sar_det_gpio_name);
		if (ret) {
			pr_err("cap1106: gpio_request failed, ret=%d\n", ret);
			goto err_SAR_DET_3G_gpio_request_failed;
		}

		ret = gpio_direction_input(data->sar_det_gpio);
		if (ret) {
			pr_err("cap1106: gpio_direction_input failed, ret=%d\n", ret);
			goto err_SAR_DET_3G_gpio_direction_input_failed;
		}

		ret = request_irq(data->irq, cap1106_interrupt_handler,
						IRQF_TRIGGER_RISING, data->sar_det_gpio_name, client);
		if (ret) {
			pr_err("cap1106: request_irq failed, ret=%d\n", ret);
			goto err_SAR_DET_3G_request_irq_failed;
		}

		pr_info("cap1106: gpio=%02d, value=%d, irq=%d OK\n", data->sar_det_gpio,
				gpio_get_value(data->sar_det_gpio), data->irq); // 94 to 350
	}

	return 0;

err_SAR_DET_3G_request_irq_failed:
err_SAR_DET_3G_gpio_direction_input_failed:
	gpio_free(data->sar_det_gpio);
err_SAR_DET_3G_gpio_request_failed:
//err_APP2MDM_SAR_gpio_direction_output_failed:
//	gpio_free(data->sar_gpio);
//err_APP2MDM_SAR_gpio_request_failed:
	return ret;
}

static int cap1106_init_sensor(struct i2c_client *client)
{
	u8 bIdx;
	int ret = 0;
	int table_length = 0;

	const u8 cap1106_init_table[] = {
		// Data sensitivity
		SENSITIVITY_CONTROL, 0x2F,
		// MAX duration disable
		CONFIGURATION, 0x20,
		// Enable CS6
		SENSOR_INPUT_ENABLE, 0x20,
		// MAX duration time to max, repeat period time to max
		SENSOR_INPUT_CONFIGURATION, 0xFF,
		// Digital count update time to 140*64ms
		AVERAGING_AND_SAMPLING_CONFING, 0x39,
		// Enable INT. for CS6.
		INTERRUPT_ENABLE, 0x20,
		// Disable repeat irq
		REPEAT_RATE_ENABLE, 0x22,
		// All channel run in the same time
		MULTIPLE_TOUCH_CONFIG, 0x00,
		// Threshold of CS 6
		SENSOR_INPUT_6_THRESHOLD, 0x0A,
		// Force re-cal CS6
		CALIBRATION_ACTIVATE, 0x20,
		// Disable RF Noise filter
		CONFIGURATION2, 0x04,
		// Reset INT bit
		MAIN_CONTROL, 0x80,
	};

	table_length = sizeof(cap1106_init_table) / sizeof(cap1106_init_table[0]);

	for (bIdx = 0; bIdx < table_length; bIdx += 2) {
		if ((ret = cap1106_write_reg(client, cap1106_init_table[bIdx],
				cap1106_init_table[bIdx + 1]))) {
			pr_err("cap1106: I2C write error, ret=0x%X\n", ret);
			break;
		}
	}

//	pr_info("cap1106: i2c_name: %s, i2c_addr: 0x%X, ret: %s\n",
//			client->name, client->addr, ret ? "FAIL" : "OK");
	return ret;
}

static void cap1106_checking_work_function(struct work_struct *work)
{
	int status;
	int bc6, dc6;
//	int bc2, bc6;
//	int dc2, dc6;

	struct cap1106_data *data = container_of((struct delayed_work *)work,
								struct cap1106_data, checking_work);

	mutex_lock(&cap_mtx);

	if (bSkip_Checking) {
		// skip this round and contiune next round
		pr_debug("cap1106: bSkip_Checking!!\n");
		mutex_unlock(&cap_mtx);
		return;
	}
	if (is_wood_sensitivity == 1) {
		if (data->enable) {
			status = cap1106_read_reg(data->client, SENSOR_INPUT_STATUS);
//			dc2 = cap1106_read_reg(data->client, 0x11);
			dc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_DELTA_COUNT);
//			bc2 = cap1106_read_reg(data->client, 0x51);
			bc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_BASE_COUNT);

			pr_debug("cap1106: status: 0x%02X, bc6=0x%02X, dc6=0x%02X\n"
					,status, bc6, dc6);

			if (dc6 == 0x00 || dc6 == 0xFF
				|| (data->overflow_status == 0x20
				&& (dc6 > 0x5F) && (dc6 <= 0x7F))) {
				pr_debug("cap1106: is_wood_sensitivity = 0\n");
				// set sensitivity and threshold for 2cm body distance
				cap1106_write_reg(data->client, SENSITIVITY_CONTROL, 0x2F);
//				cap1106_write_reg(data->client, 0x31, data->init_table[17]);
				cap1106_write_reg(data->client, SENSOR_INPUT_6_THRESHOLD, 0x0A);
				is_wood_sensitivity = 0;
				queue_delayed_work(data->cap_wq, &data->work, 0);
			}
		} else {
			pr_debug("cap1106: data->enable: %d\n", data->enable);
		}
	}

	queue_delayed_work(data->cap_wq, &data->checking_work, DELAY_MS_NORMAL);
	mutex_unlock(&cap_mtx);
}

static int cap1106_suspend(struct i2c_client *client, pm_message_t mesg)
{
	mutex_lock(&cap_mtx);
	cap1106_enable_sensor(client, 0);
	mutex_unlock(&cap_mtx);
	return 0;
}

static int cap1106_resume(struct i2c_client *client)
{
	mutex_lock(&cap_mtx);
	if (force_enable) cap1106_enable_sensor(client, 1);
	mutex_unlock(&cap_mtx);
	return 0;
}

static int cap1106_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct cap1106_data *data;

	printk("cap1106: %s\n", __func__);
	pr_info("cap1106: i2c_addr: 0x%X, i2c_client: %s, i2c_device: %s\n",
			client->addr, client->name, dev_name(&client->dev));

	data = kzalloc(sizeof(struct cap1106_data), GFP_KERNEL);
	if (!data) {
		pr_err("cap1106: kzalloc failed!\n");
		ret = -ENOMEM;
		goto err_kzalloc_failed;
	}

	data->cap_wq = create_singlethread_workqueue("cap_wq");
	if (!data->cap_wq) {
		pr_err("cap1106: create_singlethread_workqueue failed!\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&data->work, cap1106_work_function);
	INIT_DELAYED_WORK(&data->checking_work, cap1106_checking_work_function);

	data->client = client;
	i2c_set_clientdata(client, data);

	data->app2mdm_enable = 0;
	data->sar_det_gpio = acpi_get_gpio_by_index(&client->dev, 0, NULL);
	data->sar_det_gpio_name = CAP1106_SAR_DET_GPIO_NAME;
	data->irq = gpio_to_irq(data->sar_det_gpio);

	data->client->flags = 0;
	strlcpy(data->client->name, CAP1106_I2C_NAME, I2C_NAME_SIZE);
	data->enable = 0;

	ret = cap1106_init_sensor(data->client);
	if (ret) {
		pr_err("cap1106: sensor on i2c bus %s initialization failed!\n",
				dev_name(&client->dev));
		goto err_init_sensor_failed;
	}

	data->attrs.attrs = cap1106_attr_deb;

	ret = sysfs_create_group(&data->client->dev.kobj, &data->attrs);
	if (ret) {
		pr_err("cap1106: Create the sysfs group failed!\n");
		goto err_create_sysfs_group_failed;
	}

	/* register switch class */
	cap_sdev.name = CAP_SDEV_NAME;
	cap_sdev.print_name = print_cap_name;
	cap_sdev.print_state = print_cap_state;

	ret = switch_dev_register(&cap_sdev);
	if (ret) {
		pr_err("cap1106: Switch device registration failed!\n");
		goto err_register_switch_class_failed;
	}

	ret = cap1106_config_irq(data->client);
	if (ret) {
		pr_err("cap1106: Configure sensor INT failed!\n");
		goto err_config_irq_failed;
	}

	data->enable = 1;
	data->overflow_status = 0x0;
	queue_delayed_work(data->cap_wq, &data->work, DELAY_MS_HEAVY);
	queue_delayed_work(data->cap_wq, &data->checking_work, DELAY_MS_NORMAL);

	/* init I2C stress work queue */
	cap1106_stress_test_work_queue =
			create_singlethread_workqueue("i2c_cap1106_wq");
	if (!cap1106_stress_test_work_queue) {
		pr_err("cap1106: unable to create i2c stress test workqueue\n");
		goto err_create_stress_test_workqueue_failed;
	}
	INIT_DELAYED_WORK(&cap1106_stress_test_poll_work, cap1106_stress_test_poll);

	/* init misc device for I2C stress test */
	data->cap1106_misc_dev.minor = MISC_DYNAMIC_MINOR;
	data->cap1106_misc_dev.name = "cap1106";
	data->cap1106_misc_dev.fops  = &cap1106_fops;
	ret = misc_register(&data->cap1106_misc_dev);
	if (ret) {
		pr_err("cap1106: register misc dev fail\n");
		goto err_register_misc_dev_failed;
	}

	pivate_data = data;
	printk("cap1106: cap sensor is ready!\n");

	return 0;

err_register_misc_dev_failed:
	destroy_workqueue(cap1106_stress_test_work_queue);
err_create_stress_test_workqueue_failed:
err_config_irq_failed:
err_register_switch_class_failed:
	sysfs_remove_group(&data->client->dev.kobj, &data->attrs);
err_create_sysfs_group_failed:
err_init_sensor_failed:
	destroy_workqueue(data->cap_wq);
err_create_singlethread_workqueue_failed:
	kfree(data);
err_kzalloc_failed:
	return ret;
}

static int cap1106_remove(struct i2c_client *client)
{
	struct cap1106_data *data;

	switch_dev_unregister(&cap_sdev);
	data = i2c_get_clientdata(client);
	misc_deregister(&data->cap1106_misc_dev);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	free_irq(data->irq, client);
	if (data->cap_wq) destroy_workqueue(data->cap_wq);
	kfree(data);

	return 0;
}

static const struct i2c_device_id cap1106_id[] = {
	{ CAP1106_I2C_NAME, 0 },
	{ "", 0 }
};
MODULE_DEVICE_TABLE(i2c, cap1106_id);

static const struct acpi_device_id cap1106_acpi_match[] = {
	{ CAP1106_ACPI_NAME, 0 },
	{ "", 0 }
};
MODULE_DEVICE_TABLE(acpi, cap1106_acpi_match);

static struct i2c_driver cap1106_driver = {
	.driver = {
		.name = CAP1106_I2C_NAME,
		.owner = THIS_MODULE,
		.acpi_match_table = ACPI_PTR(cap1106_acpi_match),
	},
	.probe    = cap1106_probe,
	.remove   = cap1106_remove,
	.resume   = cap1106_resume,
	.suspend  = cap1106_suspend,
	.id_table = cap1106_id,
};

static int __init cap1106_init(void)
{
	printk("cap1106: %s\n", __func__);
	return i2c_add_driver(&cap1106_driver);
}

static void __exit cap1106_exit(void)
{
	printk("cap1106: %s\n", __func__);
	i2c_del_driver(&cap1106_driver);
}

module_init(cap1106_init);
module_exit(cap1106_exit);
MODULE_DESCRIPTION("SMSC CAP1106 Driver");
MODULE_LICENSE("GPL");
