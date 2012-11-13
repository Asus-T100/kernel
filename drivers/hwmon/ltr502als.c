/*
 * Copyright (C) 2009 LITE-ON Technology Corp.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/gpio.h>
#include <linux/ltr502als.h>

/* Configuration Register for mode & operation select */
#define CONFIGREG		0x00
#define MODE_SELECT		2
#define POWER_UP		(0 << MODE_SELECT)
#define POWER_DOWN		(2 << MODE_SELECT)
#define RESET			(3 << MODE_SELECT)
#define POWER_MASK		0x0C
#define OPERATION_SELECT	0
#define ACTIVE_DLS		(0 << OPERATION_SELECT)
#define ACTIVE_DPS		(1 << OPERATION_SELECT)
#define ACTIVE_DLPS		(2 << OPERATION_SELECT)
#define IDLE			(3 << OPERATION_SELECT)
#define ACTIVE_MASK		0x03

#define TCREG			0x01
#define PXY_PERSIST		4
#define PRX_INT_CYCLE1		(0 << PXY_PERSIST)
#define PRX_INT_CYCLE4		(1 << PXY_PERSIST)
#define PRX_INT_CYCLE6		(2 << PXY_PERSIST)
#define PRX_INT_CYCLE8		(3 << PXY_PERSIST)
#define PRX_INT_CYCLE_MASK	0x30
#define INTEGRATION_TIME	2
#define INTEGRATE_100MS		(0 << INTEGRATION_TIME)
#define INTEGRATE_200MS		(1 << INTEGRATION_TIME)
#define INTEGRATE_MASK		0X4
#define ALPS_PERSIST		0
#define ALPS_INT_CYCLE1		(0 << ALPS_PERSIST)
#define ALPS_INT_CYCLE4		(1 << ALPS_PERSIST)
#define ALPS_INT_CYCLE8		(2 << ALPS_PERSIST)
#define ALPS_INT_CYCLE16	(3 << ALPS_PERSIST)
#define ALPS_INT_CYCLE_MASK	0x3

#define DLSCTROL		0x02
#define ALPS_LEVEL		5
#define ADC_3LEVEL		(0 << ALPS_LEVEL)
#define ADC_5LEVEL		(1 << ALPS_LEVEL)
#define ADC_9LEVEL		(2 << ALPS_LEVEL)
#define ADC_17LEVEL		(3 << ALPS_LEVEL)
#define ADC_33LEVEL		(4 << ALPS_LEVEL)
#define ADC_64LEVEL		(5 << ALPS_LEVEL)
#define ADC_LEVEL_MASK		0xE0
#define LOW_LUX_THRESH		0
#define LOW_LUX_MASK		0X1F

#define INTREG			0x03
#define PRX_INT			1
#define DPS_INT_MASK		0X2
#define ALPS_INT		0
#define DLS_INT_MASK		0X1

#define DPSCTROL		0x04
#define PRX_ACCURACY		6
#define PRX_COUNT6		(0 << PRX_ACCURACY)
#define PRX_COUNT7		(1 << PRX_ACCURACY)
#define PRX_COUNT8		(2 << PRX_ACCURACY)
#define PRX_COUNT9		(3 << PRX_ACCURACY)
#define PRX_COUNT_MASK		0XC0
#define PRX_THRESH_CTL		0x1F
#define PRX_THRESH_MASK		0X1F

#define DATAREG			0x05
#define DPS_DATA		7
#define DPS_DATA_MASK		0x80
#define DLS_DATA		0
#define DLS_DATA_MASK		0x3F

#define DLSWINDOW		0x08
#define ALPS_WINDOW_LOSS	0xF

#define AMBIENT_ENABLE          (1 << 0)
#define PROXIMITY_ENABLE        (1 << 1)
#define ENABLE_MASK		(AMBIENT_ENABLE | PROXIMITY_ENABLE)

#define ALS_MODE_DISABLE	0
#define ALS_MODE_ENABLE		1
#define PS_MODE_DISABLE		2
#define PS_MODE_ENABLE		3

/* it takes about 800ms to generate the first interrupt when ALS is enabled*/
#define ALS_FIRST_INT_DELAY	800
/* it takes about 400ms to generate the first interrupt when PS is enabled*/
#define PS_FIRST_INT_DELAY	400
#define ALS_INIT_DATA		0xffff
#define CONFIGREG_DELAY		50
#define LTR502_GPIO_CHECK_MAX	5

struct alsps_client {
	struct list_head list;
	unsigned long status;
};

struct alsps_state {
	int now;
	int once;
};

struct alsps_device {
	struct i2c_client	*client;
	int			proximity_count;
	int			ambient_count;
	wait_queue_head_t	proximity_workq_head;
	wait_queue_head_t	ambient_workq_head;
	int			ambient_interval;
	unsigned int		alsps_switch;
	struct early_suspend	es;
	struct mutex		lock; /*
				       * Prevent parallel access to state
				       * variables, lists and registers
				       */

	struct delayed_work 	als_first_work;
	struct list_head	proximity_list;
	struct list_head	ambient_list;
	struct alsps_state	als_state;
	struct alsps_state	ps_state;

	int gpio;
	u8 dbg_addr;
};

static struct alsps_device *alsps_dev; /* for miscdevice operations */

static const int lux_map[] = {	0,      40,     100,    170,    330,    700,
				1600,   3000,   5000,   9000,   20000,  50000};
static const int adc_map[] = {	0,      3,      5,      10,     15,     20,
				25,     30,     35,     40,     45,     50};

#define PS_DATA_READY      0
#define ALS_DATA_READY     1
#define PS_FIRST_POLL      2
#define ALS_FIRST_POLL     3
#define PS_IOCTL_ENABLE    4
#define ALS_IOCTL_ENABLE   5

static int alsps_read(struct alsps_device *alsps, u8 reg, u8 *pval)
{
	int ret;

	ret = i2c_smbus_read_byte_data(alsps->client, reg);
	if (ret >= 0) {
		*pval = ret;
		ret = 0;
	} else {
		dev_err(&alsps->client->dev, "%s: Read %d faled, ret=%d\n",
				__func__, reg, ret);
	}

	return ret;
}

static int alsps_write(struct alsps_device *alsps, u8 reg, u8 val)
{
	int ret;
	ret = i2c_smbus_write_byte_data(alsps->client, reg, val);
	if (ret < 0) {
		dev_err(&alsps->client->dev, "%s: Write to %d faled, ret=%d\n",
				__func__, reg, ret);
	}
	return ret;
}

/*
 * Light sensor ADC counts and calculate lux data
 */
static int ambient_get_lum(int adc)
{
	int i = 0;
	int lux;

	do {
		if (adc <= adc_map[i])
			break;
		i++;
	} while (i < ARRAY_SIZE(adc_map) - 1);

	if (i == 0 || i == ARRAY_SIZE(adc_map) - 1)
		lux = lux_map[i];
	else
		lux = lux_map[i - 1] +
			(lux_map[i] - lux_map[i - 1]) * (adc - adc_map[i - 1]) /
			(adc_map[i] - adc_map[i - 1]);

	pr_debug("adc = %d , lux = %d", adc, lux);
	return lux;
}

static ssize_t proximity_read(struct file *filep,
			char __user *buffer, size_t size, loff_t *offset)
{
	int ret = -ENODEV;
	struct alsps_client *client;
	struct alsps_state *ps_state = &alsps_dev->ps_state;

	client = filep->private_data;
	mutex_lock(&alsps_dev->lock);
	if (alsps_dev->alsps_switch & PROXIMITY_ENABLE) {
		ret = sizeof(ps_state->once);
		clear_bit(PS_DATA_READY, &client->status);
		if (copy_to_user(buffer, &ps_state->once,
				 sizeof(ps_state->once))) {
			ret = -EFAULT;
		}
	}
	mutex_unlock(&alsps_dev->lock);

	return ret;
}

static unsigned int
proximity_poll(struct file *filep, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct alsps_client *client = filep->private_data;

	poll_wait(filep, &alsps_dev->proximity_workq_head, wait);

	if (test_bit(PS_FIRST_POLL, &client->status)) {
		mask |= (POLLIN | POLLRDNORM);
		clear_bit(PS_FIRST_POLL, &client->status);
	}

	if (test_bit(PS_DATA_READY, &client->status))
		mask |= (POLLIN | POLLRDNORM);

	return mask;
}

static int proximity_open(struct inode *inode, struct file *filep)
{
	struct alsps_client *client;

	client = kzalloc(sizeof(struct alsps_client), GFP_KERNEL);
	if (client == NULL) {
		pr_err("alsps: proximity open kzalloc failed!\n");
		return -ENOMEM;
	}

	filep->private_data = client;

	mutex_lock(&alsps_dev->lock);
	list_add(&client->list, &alsps_dev->proximity_list);
	mutex_unlock(&alsps_dev->lock);

	return 0;
}

/* lock must be held when calling this function */
static void proximity_handle_irq(struct alsps_device *alsps, u8 reg_data)
{
	struct alsps_client *client;

	dev_dbg(&alsps->client->dev, "enter %s\n", __func__);

	if (alsps->proximity_count <= 0)
		return;

	alsps->ps_state.now = (reg_data & DPS_DATA_MASK) >> DPS_DATA;
	dev_dbg(&alsps->client->dev, "proximity once = %d \t now = %d\n",
		alsps->ps_state.once, alsps->ps_state.now);

	if (alsps->ps_state.now != alsps->ps_state.once) {
		dev_info(&alsps->client->dev,
				"proximity data changed, now = %d\n",
				alsps->ps_state.now);
		alsps->ps_state.once = alsps->ps_state.now;

		list_for_each_entry(client, &alsps->proximity_list, list)
			set_bit(PS_DATA_READY, &client->status);

		wake_up(&alsps->proximity_workq_head);
	}
}

/* lock must be held when calling this function */
static void ambient_handle_irq(struct alsps_device *alsps, u8 reg_data)
{
	struct alsps_client *client;

	dev_dbg(&alsps->client->dev, "enter %s\n", __func__);

	if (alsps->ambient_count <= 0)
		return;

	alsps->als_state.now = (reg_data & DLS_DATA_MASK) >> DLS_DATA;
	dev_dbg(&alsps->client->dev, "ambient once = %d \t now = %d\n",
		alsps->als_state.once, alsps->als_state.now);

	if (alsps->als_state.now != alsps->als_state.once) {
		alsps->als_state.once = alsps->als_state.now;

		list_for_each_entry(client, &alsps->ambient_list, list)
			set_bit(ALS_DATA_READY, &client->status);

		wake_up(&alsps->ambient_workq_head);
	}
}

static ssize_t ambient_read(struct file *filep,
			char __user *buffer, size_t size, loff_t *offset)
{
	int lumen, ret = -ENODEV;
	struct alsps_client *client;
	struct alsps_state *ambient_sta = &alsps_dev->als_state;

	client = filep->private_data;
	mutex_lock(&alsps_dev->lock);
	if (alsps_dev->alsps_switch & AMBIENT_ENABLE) {
		lumen = ambient_get_lum(ambient_sta->once);
		ret = sizeof(lumen);

		clear_bit(ALS_DATA_READY, &client->status);
		if (copy_to_user(buffer, &lumen, sizeof(lumen)))
			ret = -EFAULT;
	}
	mutex_unlock(&alsps_dev->lock);

	return ret;
}

static unsigned int
ambient_poll(struct file *filep, struct poll_table_struct *wait)
{
	unsigned int mask = 0;
	struct alsps_client *client = filep->private_data;

	poll_wait(filep, &alsps_dev->ambient_workq_head, wait);

	if (test_bit(ALS_FIRST_POLL, &client->status)) {
		mask |= (POLLIN | POLLRDNORM);
		clear_bit(ALS_FIRST_POLL, &client->status);
	}

	if (test_bit(ALS_DATA_READY, &client->status))
		mask |= (POLLIN | POLLRDNORM);

	return mask;
}

static int ambient_open(struct inode *inode, struct file *filep)
{
	struct alsps_client *client;

	client = kzalloc(sizeof(struct alsps_client), GFP_KERNEL);
	if (client == NULL) {
		pr_err("alsps: ambient open kzalloc failed!\n");
		return -ENOMEM;
	}

	filep->private_data = client;

	mutex_lock(&alsps_dev->lock);
	list_add(&client->list, &alsps_dev->ambient_list);
	mutex_unlock(&alsps_dev->lock);

	return 0;
}

static void ltr502_switch(int mode)
{
	u8 data;
	int ret;

	mode = mode & ENABLE_MASK;
	switch (mode) {
	case AMBIENT_ENABLE | PROXIMITY_ENABLE:
		data = POWER_UP | ACTIVE_DLPS;
		break;
	case PROXIMITY_ENABLE:
		data = POWER_UP | ACTIVE_DPS;
		break;
	case AMBIENT_ENABLE:
		data = POWER_UP | ACTIVE_DLS;
		break;
	default:
	case 0:
		data = POWER_DOWN;
		break;
	}

	ret = alsps_write(alsps_dev, CONFIGREG, data);
	if (ret < 0) {
		dev_err(&alsps_dev->client->dev, "Write CONFIGREG failed\n");
		return;
	}
	msleep(CONFIGREG_DELAY);

	if (mode & AMBIENT_ENABLE) {
		alsps_dev->als_state.once = ALS_INIT_DATA;
		schedule_delayed_work(&alsps_dev->als_first_work,
				msecs_to_jiffies(ALS_FIRST_INT_DELAY));
	} else if (alsps_dev->alsps_switch & AMBIENT_ENABLE)
		cancel_delayed_work(&alsps_dev->als_first_work);
}


static irqreturn_t alsps_interrupt_thread(int irq, void *dev_id)
{
	struct alsps_device *alsps = dev_id;
	int i, ret;
	u8 irqstat, data;

	dev_dbg(&alsps->client->dev, "enter %s\n", __func__);

	mutex_lock(&alsps_dev->lock);
	for (i = 0; i < LTR502_GPIO_CHECK_MAX; i++) {
		ret = alsps_read(alsps, INTREG, &irqstat);
		if (ret < 0) {
			dev_err(&alsps->client->dev, "fail to read INTREG\n");
			ret = IRQ_NONE;
			goto out;
		}
		ret = alsps_read(alsps, DATAREG, &data);
		if (ret < 0) {
			dev_err(&alsps->client->dev, "fail to ACK interrupt\n");
			ret = IRQ_NONE;
			goto out;
		}

		ret = IRQ_NONE;
		if (irqstat & DPS_INT_MASK) {
			proximity_handle_irq(alsps, data);
			ret = IRQ_HANDLED;
		}
		if (irqstat & DLS_INT_MASK) {
			ambient_handle_irq(alsps, data);
			ret = IRQ_HANDLED;
		}

		/* If there is another interrupt triggerred during status read
		   and interrupt ACK, we need to check the status again to
		   avoid missing the new interrupt from different sensor*/
		if (gpio_get_value(alsps->gpio))
			break;
		else
			dev_info(&alsps->client->dev,
				"gpio low, need to check irq again\n");
	}
	if (i == LTR502_GPIO_CHECK_MAX) {
		dev_info(&alsps->client->dev,
				"gpio check max, reset sensor\n");
		ltr502_switch(alsps->alsps_switch);
	}

out:
	mutex_unlock(&alsps_dev->lock);
	return ret;
}

static int ltr502als_initchip(struct alsps_device *alsps)
{
	u8 val;

	alsps_write(alsps, CONFIGREG, POWER_UP | IDLE);
	alsps_write(alsps, DLSCTROL, ADC_64LEVEL);
	alsps_write(alsps, TCREG,
		    PRX_INT_CYCLE1 | INTEGRATE_100MS | ALPS_INT_CYCLE1);

	/* change proximity threshold PRX_THRESH_CTL to 31 to decrease the
	   distance that triggers proximity interrupt*/
	alsps_read(alsps, DPSCTROL, &val);
	alsps_write(alsps, DPSCTROL, (val & PRX_COUNT_MASK) | PRX_THRESH_CTL);

	alsps_write(alsps, CONFIGREG, POWER_DOWN);

	return 0;
}

/* lock must be held when calling this function */
static void ltr502_mode(struct alsps_client *client, int mode)
{
	struct alsps_client *list_tmp;

	switch (mode) {
	case PS_MODE_DISABLE:
		if (!test_and_clear_bit(PS_IOCTL_ENABLE, &client->status)) {
			dev_dbg(&alsps_dev->client->dev,
				"PS is not enabled for this client\n");
				return;
		}
		if (--alsps_dev->proximity_count <= 0) {
			alsps_dev->proximity_count = 0;
			alsps_dev->alsps_switch &= ~PROXIMITY_ENABLE;
		}
		break;
	case PS_MODE_ENABLE:
		list_for_each_entry(list_tmp, &alsps_dev->proximity_list, list)
			set_bit(PS_FIRST_POLL, &list_tmp->status);

		if (test_and_set_bit(PS_IOCTL_ENABLE, &client->status))
			return;
		alsps_dev->proximity_count++;
		alsps_dev->alsps_switch |= PROXIMITY_ENABLE;
		alsps_dev->ps_state.once = 0;
		break;
	case ALS_MODE_DISABLE:
		if (!test_and_clear_bit(ALS_IOCTL_ENABLE, &client->status)) {
			dev_dbg(&alsps_dev->client->dev,
				"ALS is not enabled for this client\n");
				return;
		}
		if (--alsps_dev->ambient_count <= 0) {
			alsps_dev->ambient_count = 0;
			alsps_dev->alsps_switch &= ~AMBIENT_ENABLE;
		}
		break;
	case ALS_MODE_ENABLE:
		if (test_and_set_bit(ALS_IOCTL_ENABLE, &client->status) ||
				alsps_dev->ambient_count++ > 0) {
			if (alsps_dev->als_state.once != ALS_INIT_DATA)
				set_bit(ALS_FIRST_POLL, &client->status);
			return;
		}
		alsps_dev->alsps_switch |= AMBIENT_ENABLE;
		break;
	default:
		break;
	}
	ltr502_switch(alsps_dev->alsps_switch);
}

static long
proximity_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct alsps_client *client = file->private_data;

	dev_dbg(&alsps_dev->client->dev,
		"%s:cmd = %d, arg = %d\n", __func__, (int)cmd, (int)arg);
	/* 1 - enable; 0 - disable */

	mutex_lock(&alsps_dev->lock);
	switch (arg) {
	case 0:
		ltr502_mode(client, PS_MODE_DISABLE);
		break;
	case 1:
		ltr502_mode(client, PS_MODE_ENABLE);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&alsps_dev->lock);
	return ret;
}

static long
ambient_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct alsps_client *client = file->private_data;

	dev_dbg(&alsps_dev->client->dev,
		"%s:cmd = %d, arg = %d\n", __func__, (int)cmd, (int)arg);

	mutex_lock(&alsps_dev->lock);
	/* 1 - enable; 0 - disable */
	switch (arg) {
	case 0:
		ltr502_mode(client, ALS_MODE_DISABLE);
		break;
	case 1:
		ltr502_mode(client, ALS_MODE_ENABLE);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&alsps_dev->lock);
	return ret;
}

static int ambient_close(struct inode *inode, struct file *filep)
{
	struct alsps_client *client = filep->private_data;

	mutex_lock(&alsps_dev->lock);
	list_del(&client->list);

	if (test_bit(ALS_IOCTL_ENABLE, &client->status))
		ltr502_mode(client, ALS_MODE_DISABLE);
	mutex_unlock(&alsps_dev->lock);
	kfree(client);
	filep->private_data = NULL;

	return 0;
}

static int proximity_close(struct inode *inode, struct file *filep)
{
	struct alsps_client *client = filep->private_data;

	mutex_lock(&alsps_dev->lock);
	list_del(&client->list);

	if (test_bit(PS_IOCTL_ENABLE, &client->status))
		ltr502_mode(client, PS_MODE_DISABLE);
	mutex_unlock(&alsps_dev->lock);
	kfree(client);
	filep->private_data = NULL;
	return 0;
}

static struct file_operations proximity_fops = {
	.owner = THIS_MODULE,
	.open = proximity_open,
	.read = proximity_read,
	.poll = proximity_poll,
	.release = proximity_close,
	.unlocked_ioctl = proximity_ioctl,
	.llseek = no_llseek,
};

static struct file_operations ambient_fops = {
	.owner = THIS_MODULE,
	.open = ambient_open,
	.read = ambient_read,
	.poll = ambient_poll,
	.release = ambient_close,
	.unlocked_ioctl = ambient_ioctl,
	.llseek = no_llseek,
};

static struct miscdevice proximity_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr502als_psensor",
	.fops = &proximity_fops,
};

static struct miscdevice ambient_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ltr502als_lsensor",
	.fops = &ambient_fops,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void alsps_early_suspend(struct early_suspend *h)
{
	u8 data;
	struct alsps_device *alsps = container_of(h, struct alsps_device, es);

	dev_dbg(&alsps->client->dev, "enter %s\n", __func__);

	mutex_lock(&alsps_dev->lock);
	/* If proximity is enabled, kept actice over the suspend period */
	if (alsps->alsps_switch & AMBIENT_ENABLE)
		ltr502_switch(alsps->alsps_switch & PROXIMITY_ENABLE);

	/* If PS is enabled during early supend, some PS interrupt may be lost
	 * during the CONFIG REG switch, so we need to check the data register
	 * after mode switch after a specific delay time */
	if (alsps->alsps_switch & PROXIMITY_ENABLE) {
		msleep(PS_FIRST_INT_DELAY);
		alsps_read(alsps, DATAREG, &data);
		dev_info(&alsps_dev->client->dev, "proximity check DATAREG");
		proximity_handle_irq(alsps, data);
	}
	mutex_unlock(&alsps_dev->lock);
}

static void alsps_late_resume(struct early_suspend *h)
{
	struct alsps_device *alsps = container_of(h, struct alsps_device, es);

	dev_dbg(&alsps->client->dev, "enter %s\n", __func__);

	mutex_lock(&alsps_dev->lock);
	if (alsps->alsps_switch & AMBIENT_ENABLE)
		ltr502_switch(alsps->alsps_switch);
	mutex_unlock(&alsps_dev->lock);
}
#endif

static int ltr502als_suspend(struct device *dev)
{
	struct i2c_client *i2c_client = to_i2c_client(dev);
	struct alsps_client *client;

	disable_irq(i2c_client->irq);
	enable_irq_wake(i2c_client->irq);
	if (!mutex_trylock(&alsps_dev->lock))
		goto fail;

	list_for_each_entry(client, &alsps_dev->proximity_list, list)
		if (test_bit(PS_DATA_READY, &client->status))
			goto fail_unlock;

	mutex_unlock(&alsps_dev->lock);
	return 0;
fail_unlock:
	mutex_unlock(&alsps_dev->lock);
fail:
	enable_irq(i2c_client->irq);
	disable_irq_wake(i2c_client->irq);
	return -EBUSY;
}

static int ltr502als_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);
	disable_irq_wake(client->irq);
	return 0;
}

static void als_avoid_slience(struct work_struct *work)
{
	u8 data;
	struct alsps_device *alsps = container_of((struct delayed_work *)work,
			struct alsps_device, als_first_work);

	mutex_lock(&alsps->lock);
	if (alsps->als_state.once == ALS_INIT_DATA) {
		dev_dbg(&alsps->client->dev, "%s: als_state.once=%d\n",
				__func__, alsps->als_state.once);
		alsps_read(alsps, DATAREG, &data);
		ambient_handle_irq(alsps, data);
	}
	mutex_unlock(&alsps->lock);
}

static struct alsps_device *ltr502als_alloc_dev(void)
{
	struct alsps_device *alsps;

	alsps = kzalloc(sizeof(struct alsps_device), GFP_KERNEL);
	if (!alsps)
		return NULL;

	init_waitqueue_head(&alsps->proximity_workq_head);
	init_waitqueue_head(&alsps->ambient_workq_head);

	INIT_LIST_HEAD(&alsps->ambient_list);
	INIT_LIST_HEAD(&alsps->proximity_list);

	INIT_DELAYED_WORK(&alsps->als_first_work, als_avoid_slience);
	mutex_init(&alsps->lock);

	return alsps;
}

static int ltr502als_setup_irq(struct alsps_device *alsps)
{
	struct i2c_client *client = alsps->client;
	int gpio = alsps->gpio;
	int ret;

	ret = gpio_request(gpio, "ltr502als");
	if (ret < 0) {
		dev_err(&client->dev, "Request gpio %d failed!\n", gpio);
		goto out;
	}
	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		dev_err(&client->dev, "failed to configure input\n");
		goto fail_gpio;
	}
	ret = gpio_to_irq(gpio);
	if (ret < 0) {
		dev_err(&client->dev, "Configure gpio to irq failed!\n");
		goto fail_gpio;
	}
	client->irq = ret;

	ret = request_threaded_irq(client->irq, NULL, alsps_interrupt_thread,
			  IRQF_TRIGGER_FALLING, "ltr502als", alsps);
	if (ret < 0) {
		dev_err(&client->dev, "Can't allocate irq %d\n", client->irq);
		goto fail_gpio;
	}

	return 0;

fail_gpio:
	gpio_free(gpio);
out:
	return ret;
}

static ssize_t
ltr502_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 reg;
	struct alsps_device *alsps = dev_get_drvdata(dev);

	alsps_read(alsps, alsps->dbg_addr, &reg);
	return sprintf(buf, "%d\n", reg);
}

static ssize_t
ltr502_reg_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct alsps_device *alsps = dev_get_drvdata(dev);
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;
	if (value > 0xff || value < 0)
		return -EINVAL;

	alsps_write(alsps, alsps->dbg_addr, (u8)value);
	return len;
}

static DEVICE_ATTR(reg, S_IRUGO | S_IWUSR, ltr502_reg_show, ltr502_reg_store);

static ssize_t
ltr502_addr_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct alsps_device *alsps = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", alsps->dbg_addr);
}

static ssize_t
ltr502_addr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	struct alsps_device *alsps = dev_get_drvdata(dev);
	unsigned long value;

	if (kstrtoul(buf, 0, &value))
		return -EINVAL;
	if (value > 0xff || value < 0)
		return -EINVAL;

	alsps->dbg_addr = (u8)value;
	return len;
}

static DEVICE_ATTR(addr, S_IRUGO | S_IWUSR,
		ltr502_addr_show, ltr502_addr_store);

static struct attribute *attrs[] = {
	&dev_attr_reg.attr,
	&dev_attr_addr.attr,
	NULL
};

static struct attribute_group ltr502_attr_group[] = {
	{.attrs = attrs },
};

static int __init ltr502als_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct alsps_device *alsps;

	dev_dbg(&client->dev, "enter %s\n", __func__);

	alsps = ltr502als_alloc_dev();
	if (!alsps)
		return -ENOMEM;

	alsps->client = client;
	i2c_set_clientdata(client, alsps);

	/* gpio number is stored in i2c_client.irq */
	alsps->gpio = client->irq;
	ret = ltr502als_setup_irq(alsps);
	if (ret < 0) {
		dev_err(&client->dev,
			"fail to setup irq for gpio %d\n", alsps->gpio);
		goto fail_irq;
	}

	alsps_dev = alsps;
	ret = misc_register(&proximity_dev);
	if (ret) {
		dev_err(&client->dev, "proximity dev register failed\n");
		goto fail_misc_ps;
	}

	ret = misc_register(&ambient_dev);
	if (ret) {
		dev_err(&client->dev, "ambient dev register failed\n");
		goto fail_misc_als;
	}

	ret = ltr502als_initchip(alsps);
	if (ret < 0) {
		dev_err(&client->dev, "ltr502als_initchip failed\n");
		goto fail_init;
	}

	ret = sysfs_create_group(&client->dev.kobj, ltr502_attr_group);
	if (ret < 0) {
		dev_err(&client->dev, "Sysfs registration failed\n");
		goto fail_init;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	alsps->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10;
	alsps->es.suspend = alsps_early_suspend;
	alsps->es.resume = alsps_late_resume;
#endif
	register_early_suspend(&alsps->es);

	return ret;

fail_init:
	misc_deregister(&ambient_dev);
fail_misc_als:
	misc_deregister(&proximity_dev);
fail_misc_ps:
	free_irq(client->irq, alsps);
	gpio_free(alsps->gpio);
fail_irq:
	kfree(alsps);
	return ret;
}

void ltr502als_shutdown(struct i2c_client *client)
{
	struct alsps_device *alsps = i2c_get_clientdata(client);

	dev_info(&client->dev, "enter %s\n", __func__);

	/* shutdown the device during system reboot */
	alsps_write(alsps, CONFIGREG, POWER_DOWN);
}

static int __exit ltr502als_remove(struct i2c_client *client)
{
	struct alsps_device *alsps = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "enter %s\n", __func__);

	free_irq(client->irq, alsps);
	misc_deregister(&proximity_dev);
	misc_deregister(&ambient_dev);
	sysfs_remove_group(&client->dev.kobj, ltr502_attr_group);
	unregister_early_suspend(&alsps->es);
	kfree(alsps);
	return 0;
}

static const struct i2c_device_id ltr502als_id[] = {
	{ "als", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltr502als_id);

static const struct dev_pm_ops ltr502als_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ltr502als_suspend,
			ltr502als_resume)
};

static struct i2c_driver ltr502als_driver = {
	.driver = {
		.name   = "als",
		.owner  = THIS_MODULE,
		.pm = &ltr502als_pm_ops,
	},
	.id_table = ltr502als_id,
	.probe = ltr502als_probe,
	.remove = ltr502als_remove,
	.shutdown = ltr502als_shutdown,
};

static int __init ltr502als_i2c_init(void)
{
	return i2c_add_driver(&ltr502als_driver);
}

static void __exit ltr502als_i2c_exit(void)
{
	i2c_del_driver(&ltr502als_driver);
}

module_init(ltr502als_i2c_init);
module_exit(ltr502als_i2c_exit);

MODULE_DESCRIPTION("ltr502als_i2c sensor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nicole Weng");
