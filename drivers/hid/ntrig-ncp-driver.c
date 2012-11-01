/****************************************************************
N-Trig ncp character driver

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
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/current.h>
#include <asm/segment.h>

#include "ntrig-dispatcher.h"
#include "ntrig-common.h"

/* The code in this module is identical to that of the quirk driver
 * (same functionality needed), except file names, and "quirk" is replaced by
 * "ncp" everywhere... */

#define DEVICE_NAME_NCP	"ntrig_ncp"
#define DRIVER_AUTHOR	"Dmitry Kuminov <dima@zoro-sw.com>"
#define DRIVER_DESC	"ntrig ncp driver for NCP communication"
#define PLATFORM_NAME	"ntrig_ncp"
/* file name to be used by user application: /dev/ntrig */
#define NTRIG_DEV_NAME_NCP "ntrig"
/* TODO: what is the major for NCP? - temporary put 223 (quirk+1) */
#define NTRIG_NCP_MAJOR		223
#define SEMAPHORE_TIME_OUT	(12) /* Jiffy=4 ms on 2.6 48 milisecond */

#define BLOCKING_MODE		0x01
#define POLLING_MODE		0x02

/* need to ensure that only one process/thread is reading from char driver */
static int g_singleton;

/* flag that indicates file mode operations */
static int g_file_mode = BLOCKING_MODE;

/* major number for driver recognition */
static int g_major_ncp = NTRIG_NCP_MAJOR;

/* callback for ncp message arrived via write operation */
message_callback dispatch_ncp_message;
message_callback_count read_ncp_message;

/* my class to creare /dev/ntrig pipe */
static struct class *driver_class;

/*
 * Callback function from dispatcher when driver is ready
 *static int	on_message(void * buf);
 */

/* Exported API */
static int ncp_open(struct inode *inode, struct file *filep);
static int ncp_flush(struct file *filep, fl_owner_t id);
static ssize_t	ncp_read(struct file *filep,
		char *buff, size_t count, loff_t *offp);
static ssize_t	ncp_write(struct file *filep,
		const char *buff, size_t count, loff_t *offp);

static bool memory_lock(struct semaphore *sem);
static void memory_unlock(struct semaphore *sem);
static void init_synch_data(void);

static const struct file_operations ncp_fops = {
	.owner =        THIS_MODULE,
	.read =         ncp_read,
	.write =        ncp_write,
	.open =         ncp_open,
	.flush =        ncp_flush,
};

/* Threads protection on data */
/* There used to be a common semaphore for reads and writes - so a Write had to
 * wait for a Read in execution, which can take up to 10 seconds if there is no
 * data to read. It seems like there was no real reason to stall Writes until a
 * currently-executing Read is completed, so the common semaphore is replaced
 * by two separate semaphores, one for Writes and one for Reads.
 * This change was done for the Debug Agent.
 */
struct semaphore g_sem_read, g_sem_write;

static bool memory_lock(struct semaphore *sem)
{
	if (down_interruptible(sem))
		return false;
	return true;
}

static void memory_unlock(struct semaphore *sem)
{
	up(sem);
}

static void init_synch_data(void)
{
	/* resources protection */
	sema_init(&g_sem_read, 1);
	sema_init(&g_sem_write, 1);
}


static int ncp_open(struct inode *inode, struct file *filep)
{
	if (g_singleton && !(filep->f_flags & O_WRONLY)) {
		pr_info("%s: file already opened by other thread\n",
							__func__);
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

static int ncp_flush(struct file *filep, fl_owner_t id)
{
	if (!(filep->f_flags & O_WRONLY))
		g_singleton = 0;
	return DTRG_NO_ERROR;
}


/**
 * Applicaton reads from NCP file /dev/ntrig:
 * get data from dispatcher (dispatcher gets data from device) and
 * return it to application (in *buff).
 * Operation is synchronous and bloking
 */
static ssize_t
ncp_read(struct file *filep, char __user *buff, size_t count, loff_t *offp)
{
	int retval = 0;
	char *out_buf;
	int res = 0;

	ntrig_dbg("ntrig-ncp-driver inside %s\n", __func__);

	if (!memory_lock(&g_sem_read)) {
		ntrig_dbg("%s: failed to aquire semaphore\n", __func__);
		return -EINTR;
	}

	out_buf = kmalloc(count, GFP_KERNEL);
	if (!out_buf) {
		retval = -ENOMEM;
		goto ERR_EXIT;
	}
	memset(out_buf, 0, count);

	/* get data from dispatcher */
	retval = read_ncp_message(out_buf, count);
	if (retval < 0) {
		ntrig_dbg("%s: failed to read ncp, retval = %d\n",
							__func__, retval);
		goto ERR;
	}

#if 0
	if (copy_to_user(buff, out_buf, retval)) {
		retval = DTRG_FAILED;
		ntrig_dbg("ntrig-ncp-driver: Leaving %s, failed to copy_to_user, retval = %d\n",
							__func__, retval);
	}
#endif

	res = copy_to_user(buff, out_buf, retval);
	ntrig_dbg("%s: copy_to_user returned %d\n", __func__, res);
	if (res) {
		retval = DTRG_FAILED;
		ntrig_dbg("%s: failed to copy_to_user, retval = %d\n",
							__func__, retval);
	} else {
		ntrig_dbg("%s: count = %d\n", __func__, retval);
	}

ERR:
	kfree(out_buf);
ERR_EXIT:
	memory_unlock(&g_sem_read);
	return retval;
}

/**
 * Applicaton writes to NCP file /dev/ntrig:
 * pass data (buff) to dispatcher (dispatcher passes the data to the device).
 * Operation is synchronous and bloking
*/
static ssize_t
ncp_write(struct file *filep,
		const char __user *buff, size_t count, loff_t *offp)
{
	char *in_buf = NULL;
	ntrig_dbg("%s: filep=%p,buff=%p,count=%d,offp=%p\n",
			__func__, filep, buff, (int)count, offp);

	if (!memory_lock(&g_sem_write)) {
		ntrig_dbg("%s: failed to aquire semaphore\n", __func__);
		return -EINTR;
	}

	in_buf = kmalloc(count, GFP_KERNEL);
	if (!in_buf)
		goto ERR;

	/* We must copy_from_user here, because later functions dereference
	 * data in user buffer */
	memset(in_buf, 0, count);

	if (copy_from_user(in_buf, buff, count) != 0) {
		ntrig_dbg("Userspace -> kernel copy failed!\n");
		goto FREE_ERR;
	}
	dispatch_ncp_message(in_buf);

FREE_ERR:
	kfree(in_buf);
ERR:
	memory_unlock(&g_sem_write);
	return count;
}

int bus_init_ncp(void)
{
	ntrig_dbg("ntrig-ncp-driver inside %s\n", __func__);

	if (setup_ncp(&read_ncp_message, &dispatch_ncp_message)) {
		ntrig_dbg("%s: cannot start spi dispatcher\n", __func__);
		goto ERROR;
	}
	if (register_chrdev(g_major_ncp, DEVICE_NAME_NCP, &ncp_fops)) {
		ntrig_dbg("%s: failed to register driver\n", __func__);
		goto ERROR_START;
	}
	/* Prepare synchronisation variables */
	init_synch_data();

	/* create device */
	driver_class = class_create(THIS_MODULE, DEVICE_NAME_NCP);
	device_create(driver_class, NULL, MKDEV(NTRIG_NCP_MAJOR, 0),
						"%s", NTRIG_DEV_NAME_NCP);
	return DTRG_NO_ERROR;

ERROR_START:
ERROR:
	return DTRG_FAILED;
}
EXPORT_SYMBOL_GPL(bus_init_ncp);

void bus_exit_ncp(void)
{
	ntrig_dbg("ntrig-ncp-driver inside %s\n", __func__);
	/* remove dev/ntrig */
	device_destroy(driver_class, MKDEV(NTRIG_NCP_MAJOR, 0));
	class_destroy(driver_class);
	unregister_chrdev(g_major_ncp, DEVICE_NAME_NCP);
}
EXPORT_SYMBOL_GPL(bus_exit_ncp);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_SUPPORTED_DEVICE(PLATFORM_NAME);
