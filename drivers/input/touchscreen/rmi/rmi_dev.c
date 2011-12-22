/*
 * Synaptics Register Mapped Interface (RMI4) - RMI device Module.
 * Copyright (C) 2007 - 2011, Synaptics Incorporated
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
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/syscalls.h>

#include "rmi_drvr.h"
#include "rmi_bus.h"
#include "rmi_function.h"
#include "rmi_sensor.h"
#include "rmi_dev.h"

#if defined(CONFIG_SYNA_RMI_DEV)

#define CHAR_DEVICE_NAME "rmi"
#define CHAR_DEVICE_NAME_SZ 3

#define REG_ADDR_LIMIT 0xFFFF

/*store dynamically allocated major number of char device*/
static int rmi_char_dev_major_num;


/* file operations for RMI char device */

/*
 * rmi_char_dev_llseek: - use to setup register address
 *
 * @filp: file structure for seek
 * @off: offset
 *       if whence == SEEK_SET,
 *       high 16 bits: page address
 *       low 16 bits: register address
 *
 *       if whence == SEEK_CUR,
 *       offset from current position
 *
 *       if whence == SEEK_END,
 *       offset from END(0xFFFF)
 *
 * @whence: SEEK_SET , SEEK_CUR or SEEK_END
 */
static loff_t rmi_char_dev_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t newpos;
	struct rmi_char_dev *my_char_dev = filp->private_data;

	if (IS_ERR(my_char_dev)) {
		pr_debug("%s: pointer of char device is invalid", __func__);
		return -EBADF;
	}

	mutex_lock(&(my_char_dev->mutex_file_op));

	switch (whence) {
	case SEEK_SET:
		newpos = off;
		break;

	case SEEK_CUR:
		newpos = filp->f_pos + off;
		break;

	case SEEK_END:
		newpos = REG_ADDR_LIMIT + off;
		break;

	default:		/* can't happen */
		newpos = -EINVAL;
		goto clean_up;
	}

	if (newpos < 0 || newpos > REG_ADDR_LIMIT) {
		pr_debug("%s: newpos 0x%04x is invalid.",
				__func__, (unsigned int)newpos);
		newpos = -EINVAL;
		goto clean_up;
	}

	filp->f_pos = newpos;

clean_up:

	mutex_unlock(&(my_char_dev->mutex_file_op));

	return newpos;
}

/*
 *  rmi_char_dev_read: - use to read data from RMI stream
 *
 *  @filp: file structure for read
 *  @buf: user-level buffer pointer
 *
 *  @count: number of byte read
 *  @f_pos: offset (starting register address)
 *
 *	@return number of bytes read into user buffer (buf) if succeeds
 *          negative number if error occurs.
 */
static ssize_t rmi_char_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct rmi_char_dev *my_char_dev = filp->private_data;
	ssize_t ret_value  = 0;
	unsigned char tmpbuf[count+1];
	struct rmi_phys_driver *rpd;

	/* limit offset to REG_ADDR_LIMIT-1 */
	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0)
		return 0;

	if (IS_ERR(my_char_dev)) {
		pr_err("%s: pointer of char device is invalid", __func__);
		ret_value = -EBADF;
		return ret_value;
	}

	mutex_lock(&(my_char_dev->mutex_file_op));

	rpd = my_char_dev->sensor->rpd;
	/*
	 * just let it go through , because we do not know the register is FIFO
	 * register or not
	 */

	/* return zero upon success */
	ret_value = rpd->read_multiple(rpd, *f_pos, tmpbuf, count);
	if (ret_value == 0) {
		/* we prepare the amount of data requested */
		ret_value = count;
		*f_pos += count;
	} else {
		ret_value = -EIO;
		goto clean_up;
	}

	if (copy_to_user(buf, tmpbuf, count))
		ret_value = -EFAULT;

clean_up:

	mutex_unlock(&(my_char_dev->mutex_file_op));

	return ret_value;
}

/*
 * rmi_char_dev_write: - use to write data into RMI stream
 *
 * @filep : file structure for write
 * @buf: user-level buffer pointer contains data to be written
 * @count: number of byte be be written
 * @f_pos: offset (starting register address)
 *
 * @return number of bytes written from user buffer (buf) if succeeds
 *         negative number if error occurs.
 */
static ssize_t rmi_char_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct rmi_char_dev *my_char_dev = filp->private_data;
	ssize_t ret_value  = 0;
	unsigned char tmpbuf[count+1];
	struct rmi_phys_driver *rpd;

	/* limit offset to REG_ADDR_LIMIT-1 */
	if (count > (REG_ADDR_LIMIT - *f_pos))
		count = REG_ADDR_LIMIT - *f_pos;

	if (count == 0)
		return 0;

	if (IS_ERR(my_char_dev)) {
		pr_err("%s: pointer of char device is invalid", __func__);
		ret_value = -EBADF;
		return ret_value;
	}

	if (copy_from_user(tmpbuf, buf, count)) {
		ret_value = -EFAULT;
		return ret_value;
	}

	mutex_lock(&(my_char_dev->mutex_file_op));

	rpd = my_char_dev->sensor->rpd;
	/*
	 * just let it go through , because we do not know the register is FIFO
	 * register or not
	 */

	/* return one upon success */

	ret_value = rpd->write_multiple(rpd, *f_pos, tmpbuf, count);

	if (ret_value == 1) {
		/* we had written the amount of data requested */
		ret_value = count;
		*f_pos += count;
	} else {
		ret_value = -EIO;
	}

	mutex_unlock(&(my_char_dev->mutex_file_op));

	return ret_value;
}

/*
 * rmi_char_dev_open: - get a new handle for from RMI stream
 * @inp : inode struture
 * @filp: file structure for read/write
 *
 * @return 0 if succeeds
 */
static int rmi_char_dev_open(struct inode *inp, struct file *filp)
{
	/* store the device pointer to file structure */
	struct rmi_char_dev *my_dev = container_of(inp->i_cdev,
			struct rmi_char_dev, main_dev);
	struct rmi_sensor_driver *sensor = my_dev->sensor;
	int ret_value = 0;

	filp->private_data = my_dev;

	if (!sensor)
		return -EACCES;

	mutex_lock(&(my_dev->mutex_file_op));
	if (my_dev->ref_count < 1)
		my_dev->ref_count++;
	else
		ret_value = -EACCES;

	mutex_unlock(&(my_dev->mutex_file_op));

	return ret_value; /*succeeds*/
}

/*
 *  rmi_char_dev_release: - release an existing handle
 *  @inp: inode structure
 *  @filp: file structure for read/write
 *
 *  @return 0 if succeeds
 */
static int rmi_char_dev_release(struct inode *inp, struct file *filp)
{
	struct rmi_char_dev *my_dev = container_of(inp->i_cdev,
			struct rmi_char_dev, main_dev);
	struct rmi_sensor_driver *sensor = my_dev->sensor;

	if (!sensor)
		return -EACCES;

	mutex_lock(&(my_dev->mutex_file_op));

	my_dev->ref_count--;
	if (my_dev->ref_count < 0)
		my_dev->ref_count = 0;

	mutex_unlock(&(my_dev->mutex_file_op));

	return 0; /*succeeds*/
}

static const struct file_operations rmi_char_dev_fops = {
	.owner =    THIS_MODULE,
	.llseek =   rmi_char_dev_llseek,
	.read =     rmi_char_dev_read,
	.write =    rmi_char_dev_write,
	.open =     rmi_char_dev_open,
	.release =  rmi_char_dev_release,
};

/*
 * rmi_char_dev_clean_up - release memory or unregister driver
 * @char_dev: char device structure
 * @char_device_class: class of the device use to generate one under /dev
 *
 */
static void rmi_char_dev_clean_up(struct rmi_char_dev *char_dev,
		struct class *char_device_class)
{
	dev_t devno = char_dev->main_dev.dev;

	/* Get rid of our char dev entries */
	if (char_dev) {
		cdev_del(&char_dev->main_dev);
		kfree(char_dev);
	}

	if (char_device_class) {
		device_destroy(char_device_class, devno);
		class_unregister(char_device_class);
		class_destroy(char_device_class);
	}
	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, 1);
	pr_debug("%s: rmi_char_dev is removed\n", __func__);
}

/*
 * rmi_char_devnode - return device permission
 *
 * @dev: char device structure
 * @mode: file permission
 *
 */
static char *rmi_char_devnode(struct device *dev, mode_t *mode)
{
	if (!mode)
		return NULL;
	/* rmi** */
	/**mode = 0666*/
	*mode = (S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH|S_IWOTH);

	return kasprintf(GFP_KERNEL, "rmi/%s", dev_name(dev));
}

/*
 * rmi_char_dev_register - register char device (called from up-level)
 *
 * @physical_driver: pointer to physical_driver of the lower-level stack
 *					(I2C or SPI)
 * @sensor_dev: sensor device containing this device
 *
 * @return: zero if suceeds
 */
int rmi_char_dev_register(struct rmi_phys_driver *physical_driver,
		struct rmi_sensor_device *sensor_dev)
{
	struct rmi_char_dev *char_dev;
	dev_t dev_no;
	int err;
	int result;
	struct device *device_ptr;

	if (rmi_char_dev_major_num) {
		dev_no = MKDEV(rmi_char_dev_major_num, 0);
		result = register_chrdev_region(dev_no, 1, CHAR_DEVICE_NAME);
	} else {
		result = alloc_chrdev_region(&dev_no, 0, 1, CHAR_DEVICE_NAME);
		/* let kernel allocate a major for us */
		rmi_char_dev_major_num = MAJOR(dev_no);
		pr_debug("%s Major number of rmi_char_dev: %d\n", __func__,
				rmi_char_dev_major_num);

	}
	if (result < 0)
		return result;

	/* allocate memory for rmi_char_dev */
	char_dev = kzalloc(sizeof(struct rmi_char_dev), GFP_KERNEL);
	if (!char_dev) {
		pr_err("%s: Error allocating memory. ", __func__);
		/* unregister the char device region */
		__unregister_chrdev(rmi_char_dev_major_num, MINOR(dev_no), 1,
				CHAR_DEVICE_NAME);
		return -ENOMEM;
	}
	/* initialize mutex */
	mutex_init(&char_dev->mutex_file_op);

	/* assign device pointer to sensor_device */
	sensor_dev->char_dev = char_dev;

	/* register char device */

	/* store driver pointer of rmi driver */
	char_dev->sensor = sensor_dev->driver;

	/* initialize char device */
	cdev_init(&char_dev->main_dev, &rmi_char_dev_fops);

	err = cdev_add(&char_dev->main_dev, dev_no, 1);
	/* check if adding device fails */
	if (err) {
		pr_err("%s: Error %d adding rmi_char_dev", __func__, err);
		rmi_char_dev_clean_up(sensor_dev->char_dev,
				sensor_dev->rmi_char_device_class);
		return err;
	}

	/* create device node */
	sensor_dev->rmi_char_device_class = class_create(THIS_MODULE,
			CHAR_DEVICE_NAME);

	if (IS_ERR(sensor_dev->rmi_char_device_class)) {
		pr_err("%s: create /dev/rmi failed", __func__);
		rmi_char_dev_clean_up(sensor_dev->char_dev,
				sensor_dev->rmi_char_device_class);
		return -ENODEV;
	}
	/* setup permission */
	sensor_dev->rmi_char_device_class->devnode = rmi_char_devnode;
	/* class creation */
	device_ptr = device_create(
					sensor_dev->rmi_char_device_class,
					NULL, dev_no, NULL,
					CHAR_DEVICE_NAME"%d",
					MINOR(dev_no));

	if (IS_ERR(device_ptr)) {
		pr_err("%s: create rmi device", __func__);
		rmi_char_dev_clean_up(sensor_dev->char_dev,
				sensor_dev->rmi_char_device_class);
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL(rmi_char_dev_register);

/*
 * rmi_char_dev_unregister - unregister char device (called from up-level)
 *
 * @physical_driver: pointer to physical_driver of the lower-level stack
 *					(I2C or SPI)
 * @sensor_dev: sensor device containing this device
 *
 */
void rmi_char_dev_unregister(struct rmi_char_dev *char_dev,
		struct class *char_device_class)
{
	/* clean up */
	rmi_char_dev_clean_up(char_dev, char_device_class);
}
EXPORT_SYMBOL(rmi_char_dev_unregister);

#endif /*CONFIG_SYNA_RMI_DEV*/

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("RMI4 Char Device");
MODULE_LICENSE("GPL");
