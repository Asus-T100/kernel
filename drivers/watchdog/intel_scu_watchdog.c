/*
 *      Intel_SCU 0.2:  An Intel SCU IOH Based Watchdog Device
 *			for Intel part #(s):
 *				- AF82MP20 PCH
 *
 *      Copyright (C) 2009-2010 Intel Corporation. All rights reserved.
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of version 2 of the GNU General
 *      Public License as published by the Free Software Foundation.
 *
 *      This program is distributed in the hope that it will be
 *      useful, but WITHOUT ANY WARRANTY; without even the implied
 *      warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *      PURPOSE.  See the GNU General Public License for more details.
 *      You should have received a copy of the GNU General Public
 *      License along with this program; if not, write to the Free
 *      Software Foundation, Inc., 59 Temple Place - Suite 330,
 *      Boston, MA  02111-1307, USA.
 *      The full GNU General Public License is included in this
 *      distribution in the file called COPYING.
 *
 */

#define DEBUG	1

#include <linux/compiler.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/fs.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kernel_stat.h>
#include <linux/delay.h>
#include <linux/signal.h>
#include <linux/sfi.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <asm/irq.h>
#include <asm/atomic.h>

/* See arch/x86/kernel/ipc_mrst.c */
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>

#include "intel_scu_watchdog.h"

/* Bounds number of times we will retry loading time count */
/* This retry is a work around for a silicon bug.	   */
#define MAX_RETRY 16

#define IPC_SET_WATCHDOG_TIMER	0xF8

static DECLARE_WAIT_QUEUE_HEAD(read_wq);

/* The read function (intel_scu_read) waits for the warning_flag to */
/* be set by the watchdog interrupt handler. */
/* When warning_flag is set intel_scu_read wakes up the user level */
/* process, which is responsible for refreshing the watchdog timer */
static int warning_flag;
static unsigned char osnib_reset = OSNIB_WRITE_VALUE;

static bool disable_kernel_watchdog;

/*
 * heartbeats: cpu last kstat.system times
 * beattime : jeffies at the sample time of heartbeats.
 * SOFT_LOCK_TIME : some time out in sec after warning interrupt.
 * dump_softloc_debug : called on SOFT_LOCK_TIME time out after scu
 *	interrupt to log data to logbuffer and emmc-panic code,
 *	SOFT_LOCK_TIME needs to be < SCU warn to reset time
 *	which is currently thats 15 sec.
 *
 * The soft lock works be taking a snapshot of kstat_cpu(i).cpustat.system at
 * the time of the warning interrupt for each cpu.  Then at SOFT_LOCK_TIME the
 * amount of time spend in system is computed and if its within 10 ms of the
 * total SOFT_LOCK_TIME on any cpu it will dump the stack on that cpu and then
 * calls panic.
 *
 */
#ifdef DEBUG
static cputime64_t heartbeats[NR_CPUS];
static cputime64_t beattime;
#define SOFT_LOCK_TIME 10
static void dump_softlock_debug(unsigned long data);
DEFINE_TIMER(softlock_timer, dump_softlock_debug, 0, 0);
#endif /* DEBUG */

/**
 * Please note that we are using a config CONFIG_DISABLE_SCU_WATCHDOG
 * because this boot parameter should only be settable in a development
 * environment and that customer devices should not have this capability
 */
#if defined(CONFIG_DISABLE_SCU_WATCHDOG)
module_param(disable_kernel_watchdog, bool, S_IRUGO);
MODULE_PARM_DESC(disable_kernel_watchdog,
		"Disable kernel watchdog"
		"Set to 0, watchdog started at boot"
		"and left running; Set to 1; watchdog"
		"is not started until user space"
		"watchdog daemon is started; also if the"
		"timer is started by the iafw firmware, it"
		"will be disabled upon initialization of this"
		"driver if disable_kernel_watchdog is set");
#endif

static int timer_margin = DEFAULT_SOFT_TO_HARD_MARGIN;
module_param(timer_margin, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timer_margin,
		"Watchdog timer margin"
		"Time between interrupt and resetting the system"
		"The range is from 1 to 160"
		"This is the time for all keep alives to arrive");

static int timer_set = DEFAULT_TIME;
module_param(timer_set, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timer_set,
		"Default Watchdog timer setting"
		"Complete cycle time"
		"The range is from 1 to 170"
		"This is the time for all keep alives to arrive");

/* After watchdog device is closed, check force_reset. If:
 * force_reset is false, then force boot after time expires after close,
 * force_reset is true, then force boot immediately when device is closed.
 */
static bool force_reset = true;
module_param(force_reset, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(force_reset,
		"A true means that the driver will reboot"
		"the system immediately if the /dev/watchdog device is closed"
		"A false means that when /dev/watchdog device is closed"
		"the watchdog timer will be refreshed for one more interval"
		"of length: timer_set. At the end of this interval, the"
		"watchdog timer will reset the system."
		);

/* there is only one device in the system now; this can be made into
 * an array in the future if we have more than one device */

static struct intel_scu_watchdog_dev watchdog_device;

static struct wake_lock watchdog_wake_lock;

/* Forces restart, if force_reboot is set */
static void watchdog_fire(void)
{
	if (force_reset) {
		printk(KERN_CRIT PFX "Initiating system reboot.\n");
		emergency_restart();
		printk(KERN_CRIT PFX "Reboot didn't ?????\n");
	}

	else {
		printk(KERN_CRIT PFX "Immediate Reboot Disabled\n");
		printk(KERN_CRIT PFX
			"System will reset when watchdog timer times out!\n");
	}
}

static int check_timer_margin(int new_margin)
{
	if ((new_margin < MIN_TIME_CYCLE) ||
	    (new_margin > MAX_TIME - timer_set)) {
		pr_debug("Watchdog timer: Value of new_margin %d is "
			  "out of the range %d to %d\n",
			  new_margin, MIN_TIME_CYCLE, MAX_TIME - timer_set);
			return -EINVAL;
	}
	return 0;
}

/* time is about to run out and the scu will reset soon.  quickly
 * dump debug data to logbuffer and emmc via calling panic before lights
 * go out.
 */
static void smp_dumpstack(void *info)
{
	dump_stack();
}

static void dump_softlock_debug(unsigned long data)
{
#ifdef DEBUG
	int i, reboot;
	cputime64_t system[NR_CPUS], num_jifs;

	num_jifs = jiffies - beattime;
	for_each_possible_cpu(i) {
		system[i] = cputime64_sub(kstat_cpu(i).cpustat.system,
				heartbeats[i]);
	}

	reboot = 0;

	for_each_possible_cpu(i) {
		if ((num_jifs - cputime_to_jiffies(system[i])) <
						msecs_to_jiffies(10)) {
			WARN(1, "cpu %d wedged\n", i);
			smp_call_function_single(i, smp_dumpstack, NULL, 1);
			reboot = 1;
		}
	}

	if (reboot) {
		panic_timeout = 10;
		panic("Soft lock on CPUs\n");
	}
#else /* DEBUG */
	return;
#endif /* DEBUG */

}


/*
 * IPC operations
 */
static int watchdog_set_ipc(int soft_threshold, int threshold)
{
	u32	*ipc_wbuf;
	u8	 cbuf[16] = { '\0' };
	int	 ipc_ret = 0;

	ipc_wbuf = (u32 *)&cbuf;
	ipc_wbuf[0] = soft_threshold;
	ipc_wbuf[1] = threshold;

	ipc_ret = intel_scu_ipc_command(
			IPC_SET_WATCHDOG_TIMER,
			0,
			ipc_wbuf,
			2,
			NULL,
			0);

	if (ipc_ret != 0)
		printk(KERN_CRIT PFX "Error Setting SCU Watchdog Timer: %x\n",
			ipc_ret);

	return ipc_ret;
};

/*
 *      Intel_SCU operations
 */


static int intel_scu_keepalive(void)
{

	pr_debug("Watchdog timer: keepalive: soft_threshold %x\n",
		watchdog_device.soft_threshold);

	/* read eoi register - clears interrupt */
	ioread32(watchdog_device.timer_clear_interrupt_addr);

	/* temporarily disable the timer */
	iowrite32(0x00000002, watchdog_device.timer_control_addr);

	/* set the timer to the soft_threshold */
	iowrite32(watchdog_device.soft_threshold,
		  watchdog_device.timer_load_count_addr);

	/* allow the timer to run */
	iowrite32(0x00000003, watchdog_device.timer_control_addr);

	return 0;
}

static int intel_scu_stop(void)
{
	iowrite32(0, watchdog_device.timer_control_addr);
	return 0;
}

/* tasklet function for interrupt; keep interupt itself simple */
static void watchdog_interrupt_tasklet_body(unsigned long data)
{
	int i, int_status;

	pr_debug("Watchdog: interrupt tasklet body start\n");

	if (disable_kernel_watchdog) {
		pr_debug("Watchdog: interrupt tasklet body disable set\n");
		/* disable the timer */
		/* Set all thresholds to 0 to disable timeouts */
		watchdog_device.soft_threshold = 0;
		watchdog_device.threshold = 0;

		/* send the threshold and soft_threshold via IPC */
		int_status = watchdog_set_ipc(watchdog_device.soft_threshold,
				   watchdog_device.threshold);

		if (int_status != 0) {
			/* Make sure the watchdog timer is stopped */
			pr_warn("can't set ipc to disable at start\n");
			intel_scu_stop();
			return;
		}

		iowrite32(0x00000002, watchdog_device.timer_control_addr);
		intel_scu_stop();
		return;
	}

	/* wake up read to send data to user (reminder for keep alive */
	warning_flag = 1;

#ifdef DEBUG
	/*start timer for softlock detection */
	beattime = jiffies;
	for_each_possible_cpu(i) {
		heartbeats[i] = kstat_cpu(i).cpustat.system;
	}
	mod_timer(&softlock_timer, jiffies + SOFT_LOCK_TIME * HZ);
#endif /* DEBUG */

	/* Wake up the daemon */
	wake_up_interruptible(&read_wq);

	/*
	 * Hold a timeout wakelock so user space watchdogd has a chance
	 * to run after waking up from s3
	 */
	wake_lock_timeout(&watchdog_wake_lock, 5 * HZ);
}

/* timer interrupt handler */
static irqreturn_t watchdog_timer_interrupt(int irq, void *dev_id)
{

	/* has the timer been started? If not, then this is spurious */
	if (watchdog_device.timer_started == 0) {
		pr_debug("Watchdog timer: spurious interrupt received\n");
		return IRQ_HANDLED;
	}

	tasklet_schedule(&watchdog_device.interrupt_tasklet);

	return IRQ_HANDLED;
}

static int intel_scu_set_heartbeat(u32 t)
{
	int			 ipc_ret;

	watchdog_device.timer_set = t;
	watchdog_device.threshold =
		timer_margin * watchdog_device.timer_tbl_ptr->freq_hz;
	watchdog_device.soft_threshold =
		(watchdog_device.timer_set - timer_margin)
		* watchdog_device.timer_tbl_ptr->freq_hz;

	pr_debug("Watchdog timer: set_heartbeat: timer freq is %d\n",
		watchdog_device.timer_tbl_ptr->freq_hz);
	pr_debug("Watchdog timer: set_heartbeat: timer_set is %x (hex)\n",
		watchdog_device.timer_set);
	pr_debug("Watchdog timer: set_hearbeat: timer_margin is %x (hex)\n",
		timer_margin);
	pr_debug("Watchdog timer: set_heartbeat: threshold is %x (hex)\n",
		watchdog_device.threshold);
	pr_debug("Watchdog timer: set_heartbeat: soft_threshold is %x (hex)\n",
		watchdog_device.soft_threshold);


	/* temporarily disable the timer */
	iowrite32(0x00000002, watchdog_device.timer_control_addr);

	/* send the threshold and soft_threshold via IPC to the processor */
	ipc_ret = watchdog_set_ipc(watchdog_device.soft_threshold,
				   watchdog_device.threshold);

	if (ipc_ret != 0) {
		/* Make sure the watchdog timer is stopped */
		intel_scu_stop();
		return ipc_ret;
	}

	/* Make sure timer is stopped */
	intel_scu_stop();

	/* set the timer to the soft threshold */
	iowrite32(watchdog_device.soft_threshold,
		watchdog_device.timer_load_count_addr);

	/* Start the timer */
	iowrite32(0x00000003, watchdog_device.timer_control_addr);

	watchdog_device.timer_started = 1;

	return 0;
}

/*
 * /dev/watchdog handling
 */

static int intel_scu_open(struct inode *inode, struct file *file)
{
	int result;

	/* Set flag to indicate that watchdog device is open */
	if (test_and_set_bit(0, &watchdog_device.driver_open))
		return -EBUSY;

	/* Check for reopen of driver. Reopens are not allowed */
	if (watchdog_device.driver_closed)
		return -EPERM;

	/* Let shared OSNIB (sram) know we are open */
	result = intel_scu_ipc_write_osnib(
		&osnib_reset,
		OSNIB_WRITE_SIZE,
		OSNIB_WDOG_OFFSET,
		OSNIB_WRITE_MASK);

	if (result != 0) {
		pr_warn("cant write OSNIB\n");
		return -EINVAL;
	}

	return nonseekable_open(inode, file);
}

static int intel_scu_release(struct inode *inode, struct file *file)
{
	/*
	 * This watchdog should not be closed, after the timer
	 * is started with the WDIPC_SETTIMEOUT ioctl
	 * If force_reset is set watchdog_fire() will cause an
	 * immediate reset. If force_reset is not set, the watchdog
	 * timer is refreshed for one more interval. At the end
	 * of that interval, the watchdog timer will reset the system.
	 */

	if (!test_bit(0, &watchdog_device.driver_open)) {
		pr_debug("Watchdog timer: intel_scu_release, without open\n");
		return -ENOTTY;
	}

	if (!watchdog_device.timer_started) {
		/* Just close, since timer has not been started */
		pr_debug("Watchdog timer: Closed, without starting timer\n");
		return 0;
	}

	printk(KERN_CRIT PFX
	       "Unexpected close of /dev/watchdog!\n");

	/* Since the timer was started, prevent future reopens */
	watchdog_device.driver_closed = 1;

	/* Refresh the timer for one more interval */
	intel_scu_keepalive();

	/* Reboot system (if force_reset is set) */
	watchdog_fire();

	/* We should only reach this point if force_reset is not set */
	return 0;
}

static ssize_t intel_scu_write(struct file *file,
			      char const *data,
			      size_t len,
			      loff_t *ppos)
{

	if (watchdog_device.timer_started) {
		/* Watchdog already started, keep it alive */
		intel_scu_keepalive();
		wake_unlock(&watchdog_wake_lock);
	} else
		/* Start watchdog with timer value set by init */
		intel_scu_set_heartbeat(watchdog_device.timer_set);

	return len;
}

static ssize_t intel_scu_read(struct file *file,
			     char __user *user_data,
			     size_t len,
			     loff_t *user_ppos)
{
	int result;
	u8 buf = 0;

	/* we wait for the next interrupt; if more than one */
	/* interrupt has occurred since the last read, we */
	/* dont care. The data is not critical. We will do */
	/* a copy to user each time we get and interrupt */
	/* It is up to the Watchdog daemon to be ready to */
	/* do the read (which signifies that the driver is */
	/* awaiting a keep alive and that a limited time */
	/* is available for the keep alive before the system */
	/* is rebooted by the timer */
	/* if (wait_event_interruptible(read_wq, warning_flag != 0))
		return -ERESTARTSYS; */

	warning_flag = 0;

	/* Please note that the content of the data is irrelevent */
	/* All that matters is that the read is available to the user */
	result = copy_to_user(user_data, (void *)&buf, 1);

	if (result != 0)
		return -EFAULT;
	else
		return 1;

}

static unsigned int intel_scu_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	poll_wait(file, &read_wq, wait);

	if (warning_flag == 1)
		mask |= POLLIN | POLLRDNORM;
	else
		mask = 0;

	return mask;
}

static long intel_scu_ioctl(struct file *file,
			   unsigned int cmd,
			   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u32 __user *p = argp;
	u32 new_margin;


	static const struct watchdog_info ident = {
		.options =          WDIOF_SETTIMEOUT
				    | WDIOF_KEEPALIVEPING,
		.firmware_version = 0,  /* @todo Get from SCU via
						 ipc_get_scu_fw_version()? */
		.identity =         "Intel_SCU IOH Watchdog"  /* len < 32 */
	};

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp,
				    &ident,
				    sizeof(ident)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		intel_scu_keepalive();

		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, p))
			return -EFAULT;

		if (check_timer_margin(new_margin))
			return -EINVAL;

		if (intel_scu_set_heartbeat(new_margin))
			return -EINVAL;
		return 0;
	case WDIOC_GETTIMEOUT:
		return put_user(watchdog_device.soft_threshold, p);

	default:
		return -ENOTTY;
	}
}

/*
 *      Notifier for system down
 */
static int intel_scu_notify_sys(struct notifier_block *this,
			       unsigned long code,
			       void *another_unused)
{
	if (code == SYS_DOWN || code == SYS_HALT) {
		/* Don't do instant reset on close */
		pr_debug("Watchdog timer - HALT or RESET notification\n");
		force_reset = false;
	}
	return NOTIFY_DONE;
}

/*
 *      Kernel Interfaces
 */
static const struct file_operations intel_scu_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.write          = intel_scu_write,
	.read		= intel_scu_read,
	.unlocked_ioctl = intel_scu_ioctl,
	.open           = intel_scu_open,
	.poll		= intel_scu_poll,
	.release        = intel_scu_release,
};

static int __init intel_scu_watchdog_init(void)
{
	int ret;
	u32 __iomem *tmp_addr;


	/* Check boot parameters to verify that their initial values */
	/* are in range. */
	/* Check value of timer_set boot parameter */
	if ((timer_set < MIN_TIME_CYCLE) ||
	    (timer_set > MAX_TIME - MIN_TIME_CYCLE)) {
		pr_err("Watchdog timer: Value of timer_set %x (hex) "
		  "is out of range from %x to %x (hex)\n",
		  timer_set, MIN_TIME_CYCLE, MAX_TIME - MIN_TIME_CYCLE);
		return -EINVAL;
	}

	/* Check value of timer_margin boot parameter */
	if (check_timer_margin(timer_margin))
		return -EINVAL;

	watchdog_device.timer_tbl_ptr = sfi_get_mtmr(sfi_mtimer_num-1);

	if (watchdog_device.timer_tbl_ptr == NULL) {
		pr_debug("Watchdog timer - Intel SCU watchdog: Timer is"
			" not available\n");
		return -ENODEV;
	}
	/* make sure the timer exists */
	if (watchdog_device.timer_tbl_ptr->phys_addr == 0) {
		pr_debug("Watchdog timer - Intel SCU watchdog - timer %d does"
		  " not have valid physical memory\n", sfi_mtimer_num);
		return -ENODEV;
	}

	if (watchdog_device.timer_tbl_ptr->irq == 0) {
		pr_debug("Watchdog timer: timer %d invalid irq\n",
		  sfi_mtimer_num);
		return -ENODEV;
	}

	tmp_addr = ioremap_nocache(watchdog_device.timer_tbl_ptr->phys_addr,
			20);

	if (tmp_addr == NULL) {
		pr_debug("Watchdog timer: timer unable to ioremap\n");
		return -ENOMEM;
	}

	watchdog_device.timer_load_count_addr = tmp_addr++;
	watchdog_device.timer_current_value_addr = tmp_addr++;
	watchdog_device.timer_control_addr = tmp_addr++;
	watchdog_device.timer_clear_interrupt_addr = tmp_addr++;
	watchdog_device.timer_interrupt_status_addr = tmp_addr++;

	/* Set the default time values in device structure */

	watchdog_device.intel_scu_notifier.notifier_call =
		intel_scu_notify_sys;

	ret = register_reboot_notifier(&watchdog_device.intel_scu_notifier);
	if (ret) {
		printk(KERN_ERR PFX
			"Watchdog timer: cannot register notifier %d)\n", ret);
		goto register_reboot_error;
	}

	if (!disable_kernel_watchdog) {
		watchdog_device.miscdev.minor = WATCHDOG_MINOR;
		watchdog_device.miscdev.name = "watchdog";
		watchdog_device.miscdev.fops = &intel_scu_fops;

		ret = misc_register(&watchdog_device.miscdev);
		if (ret) {
			printk(KERN_ERR PFX
			       "Watchdog timer: cannot register miscdev %d err =%d\n",
			       WATCHDOG_MINOR,
			       ret);
			goto misc_register_error;
		}
	}

	wake_lock_init(&watchdog_wake_lock, WAKE_LOCK_SUSPEND,
			"intel_scu_watchdog");

	ret = request_irq((unsigned int)watchdog_device.timer_tbl_ptr->irq,
		watchdog_timer_interrupt,
		IRQF_SHARED|IRQF_NO_SUSPEND, "watchdog",
		&watchdog_device.timer_load_count_addr);
	if (ret) {
		printk(KERN_ERR "Watchdog timer: error requesting irq\n");
		printk(KERN_ERR "Watchdog timer: error value returned is %d\n",
			ret);
		goto request_irq_error;
	}

	/* set up the tasklet for handling interrupt duties */
	tasklet_init(&watchdog_device.interrupt_tasklet,
		watchdog_interrupt_tasklet_body, (unsigned long)0);

#ifdef DEBUG
	init_timer(&softlock_timer);
#endif /* DEBUG */

	if (disable_kernel_watchdog) {
		pr_debug("disabling the timer\n");

		/* temporarily disable the timer */
		iowrite32(0x00000002, watchdog_device.timer_control_addr);

		/* Set all thresholds to 0 to disable timeouts */
		watchdog_device.soft_threshold = 0;
		watchdog_device.threshold = 0;

		/* send the threshold and soft_threshold via IPC */
		ret = watchdog_set_ipc(watchdog_device.soft_threshold,
				   watchdog_device.threshold);

		if (ret != 0) {
			/* Make sure the watchdog timer is stopped */
			pr_warn("can't set ipc to disable at start\n");
			intel_scu_stop();
			return ret;
		}

		/* Make sure timer is stopped */
		intel_scu_stop();

	}
	return 0;

/* error cleanup */

request_irq_error:

	misc_deregister(&watchdog_device.miscdev);

misc_register_error:

	pr_debug("Watchdog timer: misc_register_error\n");
	unregister_reboot_notifier(&watchdog_device.intel_scu_notifier);

register_reboot_error:

	intel_scu_stop();

	iounmap(watchdog_device.timer_load_count_addr);

	return ret;
}

static void __exit intel_scu_watchdog_exit(void)
{
#ifdef DEBUG
	del_timer_sync(&softlock_timer);
#endif /* DEBUG */

	misc_deregister(&watchdog_device.miscdev);
	unregister_reboot_notifier(&watchdog_device.intel_scu_notifier);
	/* disable the timer */
	iowrite32(0x00000002, watchdog_device.timer_control_addr);
	iounmap(watchdog_device.timer_load_count_addr);
}

#ifdef MODULE
module_init(intel_scu_watchdog_init);
#else
rootfs_initcall(intel_scu_watchdog_init);
#endif

module_exit(intel_scu_watchdog_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel SCU Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_VERSION(WDT_VER);

