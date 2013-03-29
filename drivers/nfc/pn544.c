/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/nfc/pn544.h>
#include <linux/suspend.h>
#include <linux/wakelock.h>
#include <linux/poll.h>

#define MAX_BUFFER_SIZE		512

#if defined(CONFIG_I2C_DESIGNWARE_PCI_SPLIT_XFER)
#define MAX_I2C_XFER_SIZE	31
#endif

enum polarity {
	UNKNOWN = -1,
	ACTIVE_LOW = 0,
	ACTIVE_HIGH = 1,
};

struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	struct wake_lock	read_wake;
	unsigned int		ven_gpio;
	unsigned int		firm_gpio;
	unsigned int		irq_gpio;
	enum polarity		nfc_en_polarity;
	int			busy;
};

static void pn544_platform_init(struct pn544_dev *pn544_dev)
{
	int polarity, retry, ret;
	char rset_cmd[] = {0x05, 0xF9, 0x04, 0x00, 0xC3, 0xE5};
	int count = sizeof(rset_cmd);

	pr_info("%s : detecting nfc_en polarity\n", __func__);

	/* disable fw download */
	gpio_set_value(pn544_dev->firm_gpio, 0);

	for (polarity = ACTIVE_LOW; polarity <= ACTIVE_HIGH; polarity++) {

		retry = 3;
		while (retry--) {
			/* power off */
			gpio_set_value(pn544_dev->ven_gpio,
					!polarity);
			msleep(10);
			/* power on */
			gpio_set_value(pn544_dev->ven_gpio,
					polarity);
			msleep(10);
			/* send reset */
			pr_debug("%s : sending reset cmd\n", __func__);
			ret = i2c_master_send(pn544_dev->client,
					rset_cmd, count);
			if (ret == count) {
				pr_info("%s : nfc_en polarity : active %s\n",
					__func__,
					(polarity == 0 ? "low" : "high"));
				goto out;
			}
		}
	}

	pr_err("%s : could not detect nfc_en polarity, fallback to active high\n",
			__func__);

out:
	/* store the detected polarity */
	pn544_dev->nfc_en_polarity = polarity;

	/* power off */
	gpio_set_value(pn544_dev->ven_gpio,
			!pn544_dev->nfc_en_polarity);
	return;
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	pr_debug("%s : IRQ ENTER\n", __func__);

	wake_lock_timeout(&pn544_dev->read_wake, 1*HZ);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
#if defined(CONFIG_I2C_DESIGNWARE_PCI_SPLIT_XFER)
	char *tmp_p = tmp;
	int i2c_xfer_size;
	int i2c_xfer_ret;
#endif

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	/*pr_debug("%s : reading %zu bytes.\n", __func__, count);*/

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}

		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		if (ret) {
			pr_err("%s : wait_event_interruptible: %d",
					__func__, ret);
			goto fail;
		}
	}

	/* Read data */
#if !defined(CONFIG_I2C_DESIGNWARE_PCI_SPLIT_XFER)
	ret = i2c_master_recv(pn544_dev->client, tmp, count);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
#else
	ret = count;

	while (count) {
		i2c_xfer_size = count;
		if (i2c_xfer_size > MAX_I2C_XFER_SIZE)
			i2c_xfer_size = MAX_I2C_XFER_SIZE;

		i2c_xfer_ret = i2c_master_recv(pn544_dev->client,
				tmp_p, i2c_xfer_size);
		if (i2c_xfer_ret < 0) {
			pr_err("%s: i2c_master_recv returned %d\n",
					__func__, i2c_xfer_ret);
			return i2c_xfer_ret;
		}
		if (i2c_xfer_ret > i2c_xfer_size) {
			pr_err("%s: received too many bytes from i2c (%d)\n",
					__func__, i2c_xfer_ret);
			return -EIO;
		}

		count -= i2c_xfer_size;
		tmp_p += i2c_xfer_size;
	}
#endif
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	/* Handle the corner case where read cycle is broken */
	if (ret == 1 && pn544_dev->busy) {
		pn544_dev->busy = 0;
		wake_unlock(&pn544_dev->read_wake);
		return ret;
	}

	/*
	 * PN544 read cycle consists of reading 1 byte containing the size of
	 * the data to be transferred then reading the requested data. During
	 * a read cycle, the platform shall not enter in s2ram, otherwise,
	 * the pn544 will wait forever for the data to be read and no interrupt
	 * is generated to the host even when a new field is detected. To avoid
	 * such situation, read_lock shall be held during a read_cycle and if
	 * the suspend happens during this period, then abort the suspend.
	 */
	if (ret == 1) {
		wake_lock(&pn544_dev->read_wake);
		pn544_dev->busy = 1;
	} else {
		wake_unlock(&pn544_dev->read_wake);
		pn544_dev->busy = 0;

		/* Prevent the suspend after each read cycle for 1 sec
		 * to allow propagation of the event to upper layers of NFC
		 * stack
		 */
		wake_lock_timeout(&pn544_dev->read_wake, 1*HZ);
	}

	/* Return the number of bytes read */
	pr_debug("%s : Bytes read = %d: ", __func__, ret);
	return ret;

fail:
	pr_debug("%s : wait_event is interrupted by a signal\n",
		__func__);
	return ret;
}

static unsigned int pn544_dev_poll(struct file *file, poll_table *wait)
{
	struct pn544_dev *pn544_dev = file->private_data;

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		pr_debug("%s : Waiting on available input data.\n", __func__);
		poll_wait(file, &pn544_dev->read_wq, wait);

		if (gpio_get_value(pn544_dev->irq_gpio))
			return POLLIN | POLLRDNORM;
	} else
		return POLLIN | POLLRDNORM;

	pr_debug("%s : No data on input stream.\n", __func__);
	return 0;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
#if defined(CONFIG_I2C_DESIGNWARE_PCI_SPLIT_XFER)
	char *tmp_p = tmp;
	int i2c_xfer_size;
	int i2c_xfer_ret;
#endif

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);

	/* Write data */
#if !defined(CONFIG_I2C_DESIGNWARE_PCI_SPLIT_XFER)
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n",
			__func__, ret);
		ret = -EIO;
	}
#else
	ret = count;

	while (count) {
		i2c_xfer_size = count;
		if (i2c_xfer_size > MAX_I2C_XFER_SIZE)
			i2c_xfer_size = MAX_I2C_XFER_SIZE;

		i2c_xfer_ret = i2c_master_send(pn544_dev->client,
				tmp_p, i2c_xfer_size);
		if (i2c_xfer_ret != i2c_xfer_size) {
			pr_err("%s : i2c_master_send returned %d\n",
					__func__, i2c_xfer_ret);
			return -EIO;
		}

		count -= i2c_xfer_size;
		tmp_p += i2c_xfer_size;
	}
#endif
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;

	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static int pn544_dev_release(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	filp->private_data = NULL;

	if (wake_lock_active(&pn544_dev->read_wake))
		wake_unlock(&pn544_dev->read_wake);

	return 0;
}

static long pn544_dev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:

		if (pn544_dev->nfc_en_polarity == UNKNOWN) {
			pn544_platform_init(pn544_dev);
		}

		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			pr_info("%s power on with firmware\n", __func__);
			gpio_set_value(pn544_dev->ven_gpio,
					pn544_dev->nfc_en_polarity);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio,
					!pn544_dev->nfc_en_polarity);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio,
					pn544_dev->nfc_en_polarity);
			msleep(10);
		} else if (arg == 1) {
			/* power on */
			pr_info("%s power on\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio,
					pn544_dev->nfc_en_polarity);
			msleep(10);
		} else  if (arg == 0) {
			/* power off */
			pr_info("%s power off\n", __func__);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio,
					!pn544_dev->nfc_en_polarity);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio,
					pn544_dev->nfc_en_polarity);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio,
					!pn544_dev->nfc_en_polarity);
			msleep(10);
		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= pn544_dev_read,
	.write		= pn544_dev_write,
	.poll		= pn544_dev_poll,
	.open		= pn544_dev_open,
	.release	= pn544_dev_release,
#ifdef HAVE_UNLOCKED_IOCTL
	.unlocked_ioctl	= pn544_dev_ioctl,
#endif
};

static int pn544_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct pn544_i2c_platform_data *platform_data;
	struct pn544_dev *pn544_dev;

	pr_debug("%s : entering probe\n", __func__);

#ifndef HAVE_UNLOCKED_IOCTL
	pr_err("%s: must have IOCTL", __func__);
	return -ENODEV;
#endif

	platform_data = client->dev.platform_data;
	if (platform_data == NULL) {
		pr_err("%s : nfc probe fail\n", __func__);
		return  -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	ret = platform_data->request_resources(client);
	if (ret)
		goto err_exit;

	pn544_dev->irq_gpio = platform_data->irq_gpio;
	pn544_dev->ven_gpio  = platform_data->ven_gpio;
	pn544_dev->firm_gpio  = platform_data->firm_gpio;
	pn544_dev->client   = client;
	pn544_dev->busy = 0;
	pn544_dev->nfc_en_polarity = UNKNOWN;

	/* init wakelock and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	wake_lock_init(&pn544_dev->read_wake, WAKE_LOCK_SUSPEND, "pn544_nfc");

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	ret = request_irq(client->irq, pn544_dev_irq_handler,
			  IRQF_TRIGGER_RISING, client->name, pn544_dev);
	if (ret) {
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	enable_irq_wake(client->irq);
	i2c_set_clientdata(client, pn544_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	wake_lock_destroy(&pn544_dev->read_wake);
	kfree(pn544_dev);
err_exit:
	gpio_free(platform_data->firm_gpio);
	gpio_free(platform_data->ven_gpio);
	gpio_free(platform_data->irq_gpio);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	if (wake_lock_active(&pn544_dev->read_wake))
		wake_unlock(&pn544_dev->read_wake);

	free_irq(client->irq, pn544_dev);
	wake_lock_destroy(&pn544_dev->read_wake);
	misc_deregister(&pn544_dev->pn544_device);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}

#ifdef CONFIG_SUSPEND

static int pn544_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn544_dev *pn544_dev = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_WAKELOCK
	WARN_ON(pn544_dev->busy);
#endif

	if (pn544_dev->busy)
		return -EBUSY;

	disable_irq(client->irq);

	return 0;
}

static int pn544_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	enable_irq(client->irq);

	return 0;
}

static const struct dev_pm_ops pn544_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pn544_suspend,
				pn544_resume)
};
#endif

static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn544",
#ifdef CONFIG_SUSPEND
		.pm = &pn544_pm_ops,
#endif
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
