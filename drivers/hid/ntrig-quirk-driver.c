/****************************************************************
N-Trig quirk character driver

Zoro Software.
D-Trig Digitizer modules files
Copyright (C) 2010, Dmitry Kuzminov

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

 This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

****************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <asm/current.h>
#include <asm/segment.h>

#include "typedef-ntrig.h"
#include "ntrig-common.h"
#include "ntrig-dispatcher.h"

#define DEVICE_NAME	"ntrig_quirk"
#define DRIVER_AUTHOR	"Dmitry Kuminov <dima@zoro-sw.com>"
#define DRIVER_DESC	"ntrig quirk driver,based on character device"
#define PLATFORM_NAME	"ntrig_quirk"
#define NTRIG_DEV_NAME	"ntrig_ev"

#define NTRIG_QUIRK_MAJOR	222

#define SEMAPHORE_TIME_OUT	(12) /* Jiffy=4 ms on 2.6 48 milisecond */

#define BLOCKING_MODE		0x01
#define POLLING_MODE		0x02

/* need to ensure that only one process/thread is reading from char driver */
static int g_singleton;

/* flag that indicates file mode operations */
static int		g_file_mode = BLOCKING_MODE;

/* data buffer */
struct mr_message_types_t g_data;

/* major number for driver recognition */
static int		g_major = NTRIG_QUIRK_MAJOR;

/* callback for quirk message arrived via write operation */
message_callback	dispatch_quirk_message;

/* my class to creare /dev/ntrig_ev pipe */
static struct class *driver_class;

/* Callback function from dispatcher when driver is ready */
static int	on_message(void *buf);

static int	my_open(struct inode *inode, struct file *filep);
static int	my_release(struct inode *inode, struct file *filep);
static ssize_t	my_read(struct file *filep,
			char *buff, size_t count, loff_t *offp);
static ssize_t	my_write(struct file *filep,
			const char *buff, size_t count, loff_t *offp);
static unsigned int my_poll(struct file *filep,
			struct poll_table_struct *poll_table);

static void memory_lock(unsigned long *flag);
static void memory_unlock(unsigned long *flag);
static void wait_for_data(void);
static void notify_data_ready(void);
static int  is_data_ready(void);
static void reset_data_ready(void);
static void init_synch_data(void);

const struct file_operations my_fops = {
	.open	=	my_open,
	.read	=	my_read,
	.write	=	my_write,
	.release =	my_release,
	.poll	=	my_poll,
};

/* Flag to indicate data readiness */
static int data_ready;

/* working queues to block reading/polling operations, when data not ready */
static wait_queue_head_t buffer_ready;
static wait_queue_head_t poll_buffer_ready;

/* Threads protection on data */
static spinlock_t g_mem_lock;

static void memory_lock(unsigned long *flag)
{
	spin_lock_irqsave(&g_mem_lock, *flag);
}

static void memory_unlock(unsigned long *flag)
{
	spin_unlock_irqrestore(&g_mem_lock, *flag);
}

static void wait_for_data(void)
{
	if (BLOCKING_MODE == g_file_mode)
		interruptible_sleep_on(&buffer_ready);
}

static void notify_data_ready(void)
{
	data_ready = 1;
	/* wake up queue for reading */
	wake_up_interruptible(&buffer_ready);
	/* wake up queue for polling */
	wake_up_interruptible(&poll_buffer_ready);
}

static int  is_data_ready(void)
{
	return data_ready;
}

static void  reset_data_ready(void)
{
	data_ready = 0;
}

static void init_synch_data(void)
{
	reset_data_ready();

	/* schedule syncronization */
	init_waitqueue_head(&buffer_ready);
	init_waitqueue_head(&poll_buffer_ready);

	/* resources protection */
	spin_lock_init(&g_mem_lock);

}

static int on_message(void *buf)
{
	unsigned long flag;
	ntrig_dbg("ntrig-quirk-driver inside %s\n", __func__);
	/* Copy data to emp buf */
	memory_lock(&flag);
	memcpy(&g_data, buf, sizeof(g_data));

	/* check if unblockig required */
	notify_data_ready();
	memory_unlock(&flag);
	return 0;
}

static int my_open(struct inode *inode, struct file *filep)
{
	if (g_singleton && !(filep->f_flags & O_WRONLY)) {
		pr_info("%s file already opened by other thread\n", __func__);
		return -EBUSY;
	}

	if (filep->f_flags & O_NONBLOCK)
		g_file_mode = POLLING_MODE;
	else
		g_file_mode = BLOCKING_MODE;

	if (!(filep->f_flags & O_WRONLY))
		g_singleton = 1;
	return DTRG_NO_ERROR;
}

static int my_release(struct inode *inode, struct file *filep)
{
	if (!(filep->f_flags & O_WRONLY))
		g_singleton = 0;
	return DTRG_NO_ERROR;
}

static ssize_t
my_read(struct file *filep, char *buff, size_t count, loff_t *offp)
{
	int ret_count = 0;
	unsigned long flag;
	struct mr_message_types_t data;
	int ready;


	ntrig_dbg("ntrig-quirk-driver inside %s\n", __func__);
	/* sanity check: user space buffer too small for an event */
	if (count < sizeof(data)) {
		ntrig_err("%s: userspace buffer too small (%d, expected %d)\n",
				__func__, (int)count, (int)sizeof(data));
		return 0;
	}

	/* block reading till there is data to send */
	wait_for_data();
	/* we protect quirk buffer when there is process on data */
	memory_lock(&flag);
	ready = is_data_ready();
	if (ready)
		memcpy(&data, &g_data, sizeof(data));
	memory_unlock(&flag);

	if (ready) {
		if (copy_to_user(buff, &data, sizeof(data)) != 0) {
			ntrig_dbg("%s: Kernel -> userspace copy failed!\n",
								__func__);
		}
		memory_lock(&flag);
		reset_data_ready();
		memory_unlock(&flag);
		ret_count = sizeof(g_data);
	}
	return ret_count;
}

static ssize_t
my_write(struct file *filep, const char *buff, size_t count, loff_t *offp)
{
	/* unsigned long flag; */
	struct mr_message_types_t data;
	ntrig_dbg("%s: filep=%p,buff=%p,count=%d,offp=%p\n",
			__func__, filep, buff, (int)count, offp);

	/* memory_lock(&flag); */
	memset(&data, 0, sizeof(data));
	if (_copy_from_user(&data, buff, count) != 0)
		ntrig_dbg("%s: Userspace -> kernel copy failed!\n", __func__);
	else
		ntrig_dbg("%s: message_type=%d\n", __func__, data.type);

	dispatch_quirk_message(&data);

	/* memory_unlock(&flag); */
	return count;
}

int bus_init(void)
{
	ntrig_dbg("ntrig-quirk-driver inside %s\n", __func__);

	if (setup_quirk(on_message, &dispatch_quirk_message)) {
		ntrig_dbg("%s: cannot start spi dispatcher\n", __func__);
		goto ERROR;
	}

	if (register_chrdev(g_major, DEVICE_NAME, &my_fops)) {
		ntrig_dbg("%s: failed to register driver\n", __func__);
		goto ERROR_START;
	}
	/* Prepare synchronisation variables */
	init_synch_data();

	/* create device */
	driver_class = class_create(THIS_MODULE, DEVICE_NAME);
	device_create(driver_class, NULL, MKDEV(NTRIG_QUIRK_MAJOR, 0),
						"%s", NTRIG_DEV_NAME);
	return DTRG_NO_ERROR;

ERROR_START:
ERROR:
	return DTRG_FAILED;
}
EXPORT_SYMBOL_GPL(bus_init);

void bus_exit(void)
{
	ntrig_dbg("ntrig-quirk-driver inside %s\n", __func__);
	/* remove dev/ntrig_ev */
	device_destroy(driver_class, MKDEV(NTRIG_QUIRK_MAJOR, 0));
	class_destroy(driver_class);
	unregister_chrdev(g_major, DEVICE_NAME);
}
EXPORT_SYMBOL_GPL(bus_exit);

static unsigned int
my_poll(struct file *filep, struct poll_table_struct *poll_table)
{
	unsigned int mask = 0;
	unsigned int data_ready = 0;
	unsigned long flag;
	memory_lock(&flag);
	data_ready = is_data_ready();
	memory_unlock(&flag);
	if (data_ready)
		mask |= POLLIN | POLLRDNORM;
	poll_wait(filep, &poll_buffer_ready, poll_table);
	return mask;
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_SUPPORTED_DEVICE(PLATFORM_NAME);
