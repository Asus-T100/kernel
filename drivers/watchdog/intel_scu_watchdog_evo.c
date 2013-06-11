/*
 *      Intel_SCU 0.3:  An Intel SCU IOH Based Watchdog Device
 *			for Intel part #(s):
 *				- AF82MP20 PCH
 *
 *      Copyright (C) 2009-2013 Intel Corporation. All rights reserved.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
/* See Documentation/watchdog/intel-scu-watchdog.txt */

#include <linux/compiler.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/device.h>
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
#include <linux/atomic.h>
#include <linux/wakelock.h>
#include <linux/rpmsg.h>
#include <linux/pm_qos.h>
#include <linux/intel_mid_pm.h>
#include <linux/nmi.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/io_apic.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel-mid.h>

#include "intel_scu_watchdog_evo.h"

/* Adjustment flags */
/* from config file */
/* #define CONFIG_DISABLE_SCU_WATCHDOG */
/* local */
#define CONFIG_INTEL_SCU_SOFT_LOCKUP
#define CONFIG_DEBUG_WATCHDOG

/* Defines */
#define IPC_PROC_IND_WRITE  0x05

#define STRING_RESET_TYPE_MAX_LEN 11
#define STRING_COLD_OFF "COLD_OFF"
#define STRING_COLD_RESET "COLD_RESET"
#define STRING_COLD_BOOT "COLD_BOOT"

#define EXT_TIMER0_MSI 15

/* Register bits for Timer control register(CTRL_REG) */
#define XTMR_ENABLE (0x1 << 0)
#define XTMR_DISABLE (0x0 << 0)
#define XTMR_FREE_RUNNING (0x0 << 1)
#define XTMR_USERDEF_COUNTDOWN (0x1 << 1)
#define TIMER_FREQ 19200000

#define TIMER0_LOAD_COUNT 0xFF012000     /* Timer0 Load Count */
#define TIMER0_CTRL_REG   0xFF012008     /* Timer0 Control    */
#define TIMER1_LOAD_COUNT 0xFF012014     /* Timer1 Load Count */
#define TIMER1_CTRL_REG   0xFF01201C     /* Timer1 Control   */

#define IPC_SET_WATCHDOG_TIMER  0xF8
#define IPC_SET_SUB_COLDOFF     0x03
#define IPC_SET_SUB_COLDRESET   0x04
#define IPC_SET_SUB_COLDBOOT    0x05
#define IPC_SET_SUB_DONOTHING   0x06

#define IPC_XFER_SIZE_INDWR 4

#ifdef CONFIG_DEBUG_FS
#define SECURITY_WATCHDOG_ADDR  0x40102FF4
#define STRING_NONE "NONE"
#endif

/* Statics */
static struct intel_scu_watchdog_dev watchdog_device;
static unsigned char osnib_reset = OSNIB_WRITE_VALUE;

/* PM Qos struct */
static struct pm_qos_request *qos;

/* Module params */
static bool kicking_active = true;
#ifdef CONFIG_DEBUG_WATCHDOG
module_param(kicking_active, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(kicking_active,
		"Deactivate the kicking will result in a cold reset"
		"after a while");
#endif

static bool disable_kernel_watchdog;
#ifdef CONFIG_DISABLE_SCU_WATCHDOG
/*
 * Please note that we are using a config CONFIG_DISABLE_SCU_WATCHDOG
 * because this boot parameter should only be settable in a developement
 */
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

static int pre_timeout = DEFAULT_PRETIMEOUT;
module_param(pre_timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(pre_timeout,
		"Watchdog pre timeout"
		"Time between interrupt and resetting the system"
		"The range is from 1 to 160");

static int timeout = DEFAULT_TIMEOUT;
module_param(timeout, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(timeout,
		"Default Watchdog timer setting"
		"Complete cycle time"
		"The range is from 1 to 170"
		"This is the time for all keep alives to arrive");

static bool reset_on_release = true;

#ifdef CONFIG_INTEL_SCU_SOFT_LOCKUP
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
static u64 heartbeats[NR_CPUS];
static u64 beattime;
#define SOFT_LOCK_TIME 10
static void dump_softlock_debug(unsigned long data);
DEFINE_TIMER(softlock_timer, dump_softlock_debug, 0, 0);

static struct rpmsg_instance *watchdog_instance;

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
	int i, reboot;
	u64 system[NR_CPUS], num_jifs;

	memset(system, 0, NR_CPUS*sizeof(u64));

	num_jifs = jiffies - beattime;
	for_each_possible_cpu(i) {
		system[i] = kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM] -
				heartbeats[i];
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
		trigger_all_cpu_backtrace();
		panic("Soft lock on CPUs\n");
	}
}
#endif /* CONFIG_INTEL_SCU_SOFT_LOCKUP */

/* Check current timeouts */
static int check_timeouts(int pre_timeout_time, int timeout_time)
{
	if (pre_timeout_time < timeout_time)
		return 0;

	return -EINVAL;
}

/* Set the different timeouts needed by the SCU FW */
static int watchdog_set_timeouts(int warning_pretimeout,
			    int reset_timeout)
{
	u32 ipc_wbuf;
	u32 dptr;
	int ret = 0;
	int error = 0;

	/* Prevent C-states beyond C4 */
	pm_qos_update_request(qos, CSTATE_EXIT_LATENCY_C6 - 1);

	/******** TIMER 0 *******/

	/* disable timer */
	ipc_wbuf = XTMR_DISABLE | XTMR_USERDEF_COUNTDOWN;
	dptr = TIMER0_CTRL_REG;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 0 Control: %d\n", ret);
		error = -EIO;
	}

	/* load counter */
	ipc_wbuf = warning_pretimeout * TIMER_FREQ;
	dptr = TIMER0_LOAD_COUNT;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 0 Load: %d\n", ret);
		error = -EIO;
	}

	/* enable timer */
	ipc_wbuf = XTMR_ENABLE | XTMR_USERDEF_COUNTDOWN;
	dptr = TIMER0_CTRL_REG;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 0 Controlr: %d\n", ret);
		error = -EIO;
	}

	/******** TIMER 1 *******/

	/* disable timer */
	ipc_wbuf = XTMR_DISABLE | XTMR_USERDEF_COUNTDOWN;
	dptr = TIMER1_CTRL_REG;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 1 Control: %d\n", ret);
		error = -EIO;
	}

	/* load counter */
	ipc_wbuf = reset_timeout * TIMER_FREQ;
	dptr = TIMER1_LOAD_COUNT;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 1 Load: %d\n", ret);
		error = -EIO;
	}

	/* enable timer */
	ipc_wbuf = XTMR_ENABLE | XTMR_USERDEF_COUNTDOWN;
	dptr = TIMER1_CTRL_REG;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 1 Control: %d\n", ret);
		error = -EIO;
	}

	/* Re-enable Deeper C-states beyond C4 */
	pm_qos_update_request(qos, PM_QOS_DEFAULT_VALUE);

	return error;
}

/* Provisioning function for future enhancement : allow to fine tune timing
   according to watchdog action settings */
static int watchdog_set_appropriate_timeouts(void)
{
	pr_debug(PFX "Setting shutdown timeouts\n");
	return watchdog_set_timeouts(pre_timeout, timeout);
}

/* Keep alive  */
static int watchdog_keepalive(void)
{
	int ret;

	pr_err(PFX "%s\n", __func__);

	if (unlikely(!kicking_active)) {
			/* Close our eyes */
			pr_err(PFX "Transparent kicking\n");
			return 0;
	}
	/* Really kick it */
	ret = watchdog_set_timeouts(pre_timeout, timeout);

	if (ret)
		pr_err(PFX "Error executing keepalive: %x\n", ret);

	return ret;
}

/* stops the timer */
static int intel_scu_stop(void)
{
	u32 ipc_wbuf;
	u32 dptr;
	int ret = 0;
	int error = 0;

	pr_crit(PFX "%s\n", __func__);

	/* Prevent C-states beyond C4 */
	pm_qos_update_request(qos, CSTATE_EXIT_LATENCY_C6 - 1);

	/* disable timers */
	ipc_wbuf = XTMR_DISABLE | XTMR_USERDEF_COUNTDOWN;

	dptr = TIMER0_CTRL_REG;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 0 Control: %d\n", ret);
		error = -EIO;
	}

	dptr = TIMER1_CTRL_REG;

	ret = rpmsg_send_raw_command(watchdog_instance, IPC_PROC_IND_WRITE,
		0, (u8 *) &ipc_wbuf, NULL, IPC_XFER_SIZE_INDWR, 0, 0, dptr);

	if (ret) {
		pr_crit(PFX "Error Setting Timer 1 Control: %d\n", ret);
		error = -EIO;
	}

	if (error) {
		pr_crit(PFX "Error disabling watchdog timers: %d\n", ret);
		goto err;
	}

	/* Re-enable Deeper C-states beyond C4 */
	pm_qos_update_request(qos, PM_QOS_DEFAULT_VALUE);

	watchdog_device.started = false;

err:
	return error;
}

/* warning interrupt handler */
static irqreturn_t watchdog_warning_interrupt(int irq, void *dev_id)
{
	pr_warn("[SHTDWN] %s, WATCHDOG TIMEOUT!\n", __func__);

	/* Let's reset the platform after dumping some data */
	trigger_all_cpu_backtrace();
	panic("Kernel Watchdog");

	/* This code should not be reached */
	return IRQ_HANDLED;
}

/* Program and starts the timer */
static int watchdog_config_and_start(u32 newtimeout, u32 newpretimeout)
{
	int ret;

	timeout = newtimeout;
	pre_timeout = newpretimeout;

	pr_warn(PFX "Configuration: %dkHz, timeout=%ds, pre_timeout=%ds\n",
		TIMER_FREQ / 1000, timeout, pre_timeout);

	/* Configure the watchdog */
	ret = watchdog_set_timeouts(pre_timeout, timeout);
	if (ret) {
		pr_err(PFX "%s: Cannot configure the watchdog\n", __func__);

		/* Make sure the watchdog timer is stopped */
		intel_scu_stop();
		return ret;
	}

	watchdog_device.started = true;

	return 0;
}

/* Open */
static int intel_scu_open(struct inode *inode, struct file *file)
{
	/* Set flag to indicate that watchdog device is open */
	if (test_and_set_bit(0, &watchdog_device.driver_open)) {
		pr_err(PFX "watchdog device is busy\n");
		return -EBUSY;
	}

	/* Check for reopen of driver. Reopens are not allowed */
	if (watchdog_device.driver_closed) {
		pr_err(PFX "watchdog device has been closed\n");
		return -EPERM;
	}

	return nonseekable_open(inode, file);
}

/* Release */
static int intel_scu_release(struct inode *inode, struct file *file)
{
	/*
	 * This watchdog should not be closed, after the timer
	 * is started with the WDIPC_SETTIMEOUT ioctl
	 * If reset_on_release is set this  will cause an
	 * immediate reset. If reset_on_release is not set, the watchdog
	 * timer is refreshed for one more interval. At the end
	 * of that interval, the watchdog timer will reset the system.
	 */

	if (!test_bit(0, &watchdog_device.driver_open)) {
		pr_err(PFX "intel_scu_release, without open\n");
		return -ENOTTY;
	}

	if (!watchdog_device.started) {
		/* Just close, since timer has not been started */
		pr_err(PFX "Closed, without starting timer\n");
		return 0;
	}

	pr_crit(PFX "Unexpected close of /dev/watchdog!\n");

	/* Since the timer was started, prevent future reopens */
	watchdog_device.driver_closed = 1;

	/* Refresh the timer for one more interval */
	watchdog_keepalive();

	/* Reboot system if requested */
	if (reset_on_release) {
		pr_crit(PFX "Initiating system reboot.\n");
		emergency_restart();
	}

	pr_crit(PFX "Immediate Reboot Disabled\n");
	pr_crit(PFX "System will reset when watchdog timer expire!\n");

	return 0;
}

/* Write */
static ssize_t intel_scu_write(struct file *file, char const *data, size_t len,
			      loff_t *ppos)
{
	pr_debug(PFX "watchdog %s\n", __func__);

	if (watchdog_device.shutdown_flag == true)
		/* do nothing if we are shutting down */
		return len;

	if (watchdog_device.started) {
		/* Watchdog already started, keep it alive */
		watchdog_keepalive();
	}

	return len;
}

/* ioctl */
static long intel_scu_ioctl(struct file *file, unsigned int cmd,
			    unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u32 __user *p = argp;
	u32 val;
	int options;

	static const struct watchdog_info ident = {
		.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
		/* @todo Get from SCU via ipc_get_scu_fw_version()? */
		.firmware_version = 0,
		/* len < 32 */
		.identity = "Intel_SCU IOH Watchdog"
	};

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident,
				    sizeof(ident)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		pr_warn(PFX "%s: KeepAlive ioctl\n", __func__);
		if (!watchdog_device.started)
			return -EINVAL;

		watchdog_keepalive();
		return 0;
	case WDIOC_SETPRETIMEOUT:
		pr_warn(PFX "%s: SetPreTimeout ioctl\n", __func__);

		if (watchdog_device.started)
			return -EBUSY;

		/* Timeout to warn */
		if (get_user(val, p))
			return -EFAULT;

		pre_timeout = val;
		return 0;
	case WDIOC_SETTIMEOUT:
		pr_warn(PFX "%s: SetTimeout ioctl\n", __func__);

		if (watchdog_device.started)
			return -EBUSY;

		if (get_user(val, p))
			return -EFAULT;

		timeout = val;
		return 0;
	case WDIOC_GETTIMEOUT:
		return put_user(timeout, p);
	case WDIOC_SETOPTIONS:
		if (get_user(options, p))
			return -EFAULT;

		if (options & WDIOS_DISABLECARD) {
			pr_warn(PFX "%s: Stopping the watchdog\n", __func__);
			intel_scu_stop();
			return 0;
		}

		if (options & WDIOS_ENABLECARD) {
			pr_warn(PFX "%s: Starting the watchdog\n", __func__);

			if (watchdog_device.started)
				return -EBUSY;

			if (check_timeouts(pre_timeout, timeout)) {
				pr_warn(PFX "%s: Invalid thresholds\n",
					__func__);
				return -EINVAL;
			}
			if (watchdog_config_and_start(timeout, pre_timeout))
				return -EINVAL;
			return 0;
		}
		return 0;
	default:
		return -ENOTTY;
	}
}

static int watchdog_set_reset_type(int reset_type)
{
	return rpmsg_send_command(watchdog_instance,
				  IPC_SET_WATCHDOG_TIMER,
				  reset_type,
				  NULL, NULL, 0, 0);
}

/* Reboot notifier */
static int reboot_notifier(struct notifier_block *this,
			   unsigned long code,
			   void *another_unused)
{
	int ret;

	if (code == SYS_RESTART || code == SYS_HALT || code == SYS_POWER_OFF) {
		pr_warn(PFX "Reboot notifier\n");

		if (watchdog_set_appropriate_timeouts())
			pr_crit(PFX "reboot notifier cant set time\n");

		switch (code) {
		case SYS_RESTART:
			ret = watchdog_set_reset_type(
				watchdog_device.reboot_wd_action);
			break;

		case SYS_HALT:
		case SYS_POWER_OFF:
			ret = watchdog_set_reset_type(
				watchdog_device.shutdown_wd_action);
			break;
		}
		if (ret)
			pr_err(PFX "%s: could not set reset type\n", __func__);

#ifdef CONFIG_DEBUG_FS
		/* debugfs entry to generate a BUG during
		any shutdown/reboot call */
		if (watchdog_device.panic_reboot_notifier)
			BUG();
#endif
		/* Don't do instant reset on close */
		reset_on_release = false;

		/* Kick once again */
		if (disable_kernel_watchdog == false) {
			ret = watchdog_keepalive();
			if (ret)
				pr_warn(PFX "%s: no keep alive\n", __func__);

			/* Don't allow any more keep-alives */
			watchdog_device.shutdown_flag = true;
		}
	}
	return NOTIFY_DONE;
}

#ifdef CONFIG_DEBUG_FS
/* This code triggers a Security Watchdog */
int open_security(struct inode *i, struct file *f)
{
	int ret = 0;
	u64 *ptr;
	u32 value;

	ptr = ioremap_nocache(SECURITY_WATCHDOG_ADDR, sizeof(u32));

	if (!ptr) {
		pr_err(PFX "cannot open secwd's debugfile\n");
		ret = -ENODEV;
		goto error;
	}
	value = readl(ptr); /* trigger */

	pr_err(PFX "%s: This code should never be reached but it got %x\n",
		__func__, (unsigned int)value);

error:
	return ret;
}

static const struct file_operations security_watchdog_fops = {
	.open = open_security,
};

static int kwd_trigger_open(struct inode *inode, struct file *file)
{
	BUG();
	return 0;
}

static const struct file_operations kwd_trigger_fops = {
	.open		= kwd_trigger_open,
};

static int kwd_reset_type_release(struct inode *inode, struct file *file)
{
	return 0;
}



static ssize_t kwd_reset_type_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	ssize_t len = 0;
	unsigned long res;

	pr_debug(PFX "reading reset_type of %x\n",
			watchdog_device.normal_wd_action);

	if (*ppos > 0)
		return 0;

	switch (watchdog_device.normal_wd_action) {
	case IPC_SET_SUB_COLDOFF:
		res = copy_to_user(buff, STRING_COLD_OFF "\n",
					strlen(STRING_COLD_OFF)+2);
		if (res) {
			pr_err(PFX "%s: copy to user failed\n", __func__);
			return -EINVAL;
		}

		len = strlen(STRING_COLD_OFF)+2;
		break;
	case IPC_SET_SUB_COLDRESET:
		res = copy_to_user(buff, STRING_COLD_RESET "\n",
					strlen(STRING_COLD_RESET)+2);
		if (res) {
			pr_err(PFX "%s: copy to user failed\n", __func__);
			return -EINVAL;
		}

		len = strlen(STRING_COLD_RESET)+2;
		break;
	case IPC_SET_SUB_COLDBOOT:
		res = copy_to_user(buff, STRING_COLD_BOOT "\n",
					strlen(STRING_COLD_BOOT)+2);
		if (res) {
			pr_err(PFX "%s: copy to user failed\n", __func__);
			return -EINVAL;
		}

		len = strlen(STRING_COLD_BOOT)+2;
		break;
	case IPC_SET_SUB_DONOTHING:
		res = copy_to_user(buff, STRING_NONE "\n",
					strlen(STRING_NONE)+2);
		if (res) {
			pr_err(PFX "%s: copy to user failed\n", __func__);
			return -EINVAL;
		}

		len = strlen(STRING_NONE)+2;
		break;
	default:
		return -EINVAL;
	}

	*ppos += len;
	return len+1;
}

static ssize_t kwd_reset_type_write(struct file *file, const char __user *buff,
		size_t count, loff_t *ppos)
{
	char str[STRING_RESET_TYPE_MAX_LEN];
	unsigned long res;
	int ret;

	if (count >= STRING_RESET_TYPE_MAX_LEN) {
		pr_err(PFX "Invalid size%s\n", str);
		return -EINVAL;
	}

	memset(str, 0x00, STRING_RESET_TYPE_MAX_LEN);

	res = copy_from_user((void *)str,
		(void __user *)buff,
		(unsigned long)min((unsigned long)(count-1),
		(unsigned long)(STRING_RESET_TYPE_MAX_LEN-1)));

	if (res) {
		pr_err(PFX "%s: copy to user failed\n", __func__);
		return -EINVAL;
	}

	pr_debug(PFX "writing reset_type of %s\n", str);

	if (!strncmp(str, STRING_COLD_OFF, STRING_RESET_TYPE_MAX_LEN)) {
		ret = watchdog_set_reset_type(IPC_SET_SUB_COLDOFF);

	} else if (!strncmp(str, STRING_COLD_RESET,
					STRING_RESET_TYPE_MAX_LEN)) {
		ret = watchdog_set_reset_type(IPC_SET_SUB_COLDRESET);

	} else if (!strncmp(str, STRING_COLD_BOOT,
					STRING_RESET_TYPE_MAX_LEN)) {
		ret = watchdog_set_reset_type(IPC_SET_SUB_COLDBOOT);

	} else if (!strncmp(str, STRING_NONE,
					STRING_RESET_TYPE_MAX_LEN)) {
		ret = watchdog_set_reset_type(IPC_SET_SUB_DONOTHING);

	} else {
		pr_err(PFX "Invalid value\n");
		return -EINVAL;
	}

	/* check return code of watchdog_set_reset_type */
	if (ret) {
		pr_err(PFX "%s: could not set reset type\n", __func__);
		return -EINVAL;
	}

	return count;
}

static const struct file_operations kwd_reset_type_fops = {
	.open		= nonseekable_open,
	.release	= kwd_reset_type_release,
	.read		= kwd_reset_type_read,
	.write		= kwd_reset_type_write,
	.llseek		= no_llseek,
};

static ssize_t kwd_panic_reboot_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos)
{
	# define RET_SIZE 3 /* prints only 2 chars : '0' or '1', plus '\n' */
	char str[RET_SIZE];

	int res;

	if (*ppos > 0)
		return 0;

	strcpy(str, watchdog_device.panic_reboot_notifier ? "1\n" : "0\n");

	res = copy_to_user(buff, str, RET_SIZE);
	if (res) {
		pr_err(PFX "%s: copy to user failed\n", __func__);
		return -EINVAL;
	}

	*ppos += RET_SIZE-1;
	return RET_SIZE-1;
}


static ssize_t kwd_panic_reboot_write(struct file *file,
		const char __user *buff, size_t count, loff_t *ppos)
{
	/* whatever is written, simply set flag to TRUE */
	watchdog_device.panic_reboot_notifier = true;

	return count;
}


static const struct file_operations kwd_panic_reboot_fops = {
	.open		= nonseekable_open,
	.read		= kwd_panic_reboot_read,
	.write		= kwd_panic_reboot_write,
	.llseek		= no_llseek,
};

static int remove_debugfs_entries(void)
{
struct intel_scu_watchdog_dev *dev = &watchdog_device;

	/* /sys/kernel/debug/watchdog */
	debugfs_remove_recursive(dev->dfs_wd);

	return 0;
}

static int create_debugfs_entries(void)
{
	struct intel_scu_watchdog_dev *dev = &watchdog_device;

	/* /sys/kernel/debug/watchdog */
	dev->dfs_wd = debugfs_create_dir("watchdog", NULL);
	if (!dev->dfs_wd) {
		pr_err(PFX "%s: Error, cannot create main dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/security_watchdog */
	dev->dfs_secwd = debugfs_create_dir("security_watchdog", dev->dfs_wd);
	if (!dev->dfs_secwd) {
		pr_err(PFX "%s: Error, cannot create sec dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/security_watchdog/trigger */
	dev->dfs_secwd_trigger = debugfs_create_file("trigger",
				    S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				    dev->dfs_secwd, NULL,
				    &security_watchdog_fops);

	if (!dev->dfs_secwd_trigger) {
		pr_err(PFX "%s: Error, cannot create sec file\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog */
	dev->dfs_kwd = debugfs_create_dir("kernel_watchdog", dev->dfs_wd);
	if (!dev->dfs_kwd) {
		pr_err(PFX "%s: Error, cannot create kwd dir\n", __func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/trigger */
	dev->dfs_kwd_trigger = debugfs_create_file("trigger",
				    S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				    dev->dfs_kwd, NULL,
				    &kwd_trigger_fops);

	if (!dev->dfs_kwd_trigger) {
		pr_err(PFX "%s: Error, cannot create kwd trigger file\n",
			__func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/reset_type */
	dev->dfs_kwd_trigger = debugfs_create_file("reset_type",
				    S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				    dev->dfs_kwd, NULL,
				    &kwd_reset_type_fops);

	if (!dev->dfs_kwd_trigger) {
		pr_err(PFX "%s: Error, cannot create kwd trigger file\n",
			__func__);
		goto error;
	}

	/* /sys/kernel/debug/watchdog/kernel_watchdog/panic_reboot_notifier */
	dev->dfs_kwd_panic_reboot = debugfs_create_file("panic_reboot_notifier",
					S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
					dev->dfs_kwd, NULL,
					&kwd_panic_reboot_fops);

	if (!dev->dfs_kwd_panic_reboot) {
		pr_err(PFX "%s: Error, cannot create kwd panic_reboot_notifier file\n",
			__func__);
		goto error;
	}


	return 0;
error:
	remove_debugfs_entries();
	return 1;
}
#endif  /* CONFIG_DEBUG_FS*/

/* Kernel Interfaces */
static const struct file_operations intel_scu_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.write          = intel_scu_write,
	.unlocked_ioctl = intel_scu_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= intel_scu_ioctl,
#endif
	.open           = intel_scu_open,
	.release        = intel_scu_release,
};

/* sysfs entry to disable watchdog */
#ifdef CONFIG_DISABLE_SCU_WATCHDOG
static ssize_t disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	if (!strtobool(buf, &disable_kernel_watchdog)) {
		if (disable_kernel_watchdog) {
			ret = intel_scu_stop();
			if (ret)
				pr_err(PFX "cannot disable the timer\n");
		} else {
			ret = watchdog_config_and_start(timeout, pre_timeout);
			if (ret)
				return -EINVAL;
		}
	} else {
		pr_err(PFX "got invalid value\n");
		return -EINVAL;
	}

	return size;
}

static ssize_t disable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_debug(PFX "%s\n", __func__);
	if (disable_kernel_watchdog)
		return sprintf(buf, "1\n");

	return sprintf(buf, "0\n");
}

static DEVICE_ATTR(disable, S_IWUSR | S_IRUGO,
	disable_show, disable_store);

#endif

static ssize_t counter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	pr_debug(PFX "%s\n", __func__);
	ret = intel_scu_ipc_write_osnib_wd(&osnib_reset);

	if (ret != 0) {
		pr_err(PFX "cannot write OSNIB\n");
		return -EINVAL;
	}

	return size;
}

#define OSNIB_WDOG_COUNTER_MASK 0xF0
#define OSNIB_WDOG_COUNTER_SHIFT 4
static ssize_t counter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned char osnib_read = (unsigned char)0;
	int ret;
	pr_debug(PFX "%s\n", __func__);

	ret = intel_scu_ipc_read_osnib_wd(&osnib_read);

	if (ret != 0)
		return -EIO;

	return sprintf(buf, "%d\n", (int)((osnib_read & OSNIB_WDOG_COUNTER_MASK)
						>> OSNIB_WDOG_COUNTER_SHIFT));
}

static int reset_type_to_string(int reset_type, char *string)
{
	switch (reset_type) {
	case IPC_SET_SUB_COLDBOOT:
		strcpy(string, STRING_COLD_BOOT);
		break;
	case IPC_SET_SUB_COLDRESET:
		strcpy(string, STRING_COLD_RESET);
		break;
	case IPC_SET_SUB_COLDOFF:
		strcpy(string, STRING_COLD_OFF);
		break;
#ifdef CONFIG_DEBUG_FS
	case IPC_SET_SUB_DONOTHING:
		/* The IPC command DONOTHING is provided */
		/* for debug purpose only.               */
		strcpy(string, STRING_NONE);
		break;
#endif
	default:
		return 1;
	}

	return 0;
}

static int string_to_reset_type(const char *string, int *reset_type)
{
	if (!reset_type || !string)
		return 1;

	if (strncmp(string, STRING_COLD_RESET,
			sizeof(STRING_COLD_RESET) - 1) == 0) {
		*reset_type = IPC_SET_SUB_COLDRESET;
		return 0;
	}
	if (strncmp(string, STRING_COLD_BOOT,
			sizeof(STRING_COLD_BOOT) - 1) == 0) {
		*reset_type = IPC_SET_SUB_COLDBOOT;
		return 0;
	}
	if (strncmp(string, STRING_COLD_OFF,
			sizeof(STRING_COLD_OFF) - 1) == 0) {
		*reset_type = IPC_SET_SUB_COLDOFF;
		return 0;
	}

	/* We should not be here, this is an error case */
	pr_debug("Invalid reset type value\n");
	return 1;
}

static ssize_t reboot_ongoing_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	pr_debug(PFX "%s\n", __func__);
	/* reprogram timeouts. if error : continue */
	ret = watchdog_set_appropriate_timeouts();
	if (ret)
		pr_err(PFX "%s: could not set timeouts\n", __func__);

	/* restore reset type */
	watchdog_set_reset_type(watchdog_device.reboot_wd_action);
	if (ret) {
		pr_err(PFX "%s: could not set reset type\n", __func__);
		return -EINVAL;
	}

	return size;
}

static ssize_t shutdown_ongoing_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	pr_debug(PFX "%s\n", __func__);
	/* reprogram timeouts. if error : continue */
	ret = watchdog_set_appropriate_timeouts();
	if (ret)
		pr_err(PFX "%s: could not set timeouts\n", __func__);

	/* restore reset type */
	watchdog_set_reset_type(watchdog_device.shutdown_wd_action);
	if (ret) {
		pr_err(PFX "%s: could not set reset type\n", __func__);
		return -EINVAL;
	}

	return size;
}

static ssize_t normal_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (reset_type_to_string(watchdog_device.normal_wd_action, buf) != 0)
		return -EINVAL;
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t normal_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (string_to_reset_type(buf, &watchdog_device.normal_wd_action) != 0)
		return -EINVAL;
	if (watchdog_set_reset_type(watchdog_device.normal_wd_action) != 0)
		return -EINVAL;

	return size;
}

static ssize_t reboot_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (reset_type_to_string(watchdog_device.reboot_wd_action, buf) != 0)
		return -EINVAL;
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t reboot_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (string_to_reset_type(buf, &watchdog_device.reboot_wd_action) != 0)
		return -EINVAL;

	return size;
}

static ssize_t shutdown_config_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (reset_type_to_string(watchdog_device.shutdown_wd_action, buf) != 0)
		return -EINVAL;
	strcat(buf, "\n");
	return strlen(buf);
}

static ssize_t shutdown_config_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	if (string_to_reset_type(buf, &watchdog_device.shutdown_wd_action) != 0)
		return -EINVAL;

	return size;
}

/* Watchdog behavior depending on system phase */
static DEVICE_ATTR(normal_config, S_IWUSR | S_IRUGO,
	normal_config_show, normal_config_store);
static DEVICE_ATTR(reboot_config, S_IWUSR | S_IRUGO,
	reboot_config_show, reboot_config_store);
static DEVICE_ATTR(shutdown_config, S_IWUSR | S_IRUGO,
	shutdown_config_show, shutdown_config_store);
static DEVICE_ATTR(reboot_ongoing, S_IWUSR | S_IRUGO,
	NULL, reboot_ongoing_store);
static DEVICE_ATTR(shutdown_ongoing, S_IWUSR | S_IRUGO,
	NULL, shutdown_ongoing_store);

/* Reset counter watchdog entry */
static DEVICE_ATTR(counter, S_IWUSR | S_IRUGO,
	counter_show, counter_store);


int create_watchdog_sysfs_files(void)
{
	int ret;

#ifdef CONFIG_DISABLE_SCU_WATCHDOG
	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_disable);
	if (ret) {
		pr_warn("cant register dev file for disable\n");
		return ret;
	}
#endif

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_normal_config);
	if (ret) {
		pr_warn("cant register dev file for normal_config\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_config);
	if (ret) {
		pr_warn("cant register dev file for reboot_config\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_config);
	if (ret) {
		pr_warn("cant register dev file for shutdown_config\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_counter);
	if (ret) {
		pr_warn("cant register dev file for counter\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_ongoing);
	if (ret) {
		pr_warn("cant register dev file for reboot_ongoing\n");
		return ret;
	}

	ret = device_create_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_ongoing);
	if (ret) {
		pr_warn("cant register dev file for shutdown_ongoing\n");
		return ret;
	}
	return 0;
}

int remove_watchdog_sysfs_files(void)
{
#ifdef CONFIG_DISABLE_SCU_WATCHDOG
	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_disable);
#endif
	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_normal_config);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_config);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_config);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_counter);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_reboot_ongoing);

	device_remove_file(watchdog_device.miscdev.this_device,
		&dev_attr_shutdown_ongoing);
	return 0;
}

static int handle_mrfl_dev_ioapic(int irq)
{
	int ret = 0;
	int ioapic;
	struct io_apic_irq_attr irq_attr;

	ioapic = mp_find_ioapic(irq);
	if (ioapic >= 0) {
		irq_attr.ioapic = ioapic;
		irq_attr.ioapic_pin = irq;
		irq_attr.trigger = 1;
		irq_attr.polarity = 0; /* Active high */
		io_apic_set_pci_routing(NULL, irq, &irq_attr);
	} else {
		pr_warn("can not find interrupt %d in ioapic\n", irq);
		ret = -EINVAL;
	}

	return ret;
}

/* Platfrom device functionality */
static int watchdog_probe(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "Probed watchdog pm device\n");
	return 0;
}

static int watchdog_remove(struct platform_device *pdev)
{
	return 0;
}

static int watchdog_resume(struct device *dev)
{
	pr_info(PFX "%s\n", __func__);

	return 0;
}

static int watchdog_suspend(struct device *dev)
{
	int ret = 0;
	pr_info(PFX "%s\n", __func__);

	if (watchdog_device.started) {
		/* kick timers before suspending */
		ret = watchdog_keepalive();

		if (ret)
			pr_err(PFX "Error executing keepalive: %x\n", ret);
	}

	return ret;
}

static const struct dev_pm_ops watchdog_pm_ops = {
	.suspend = watchdog_suspend,
	.resume = watchdog_resume,
};

static struct platform_driver watchdog_driver = {
	.driver = {
		.name = "watchdog_pm",
		.owner = THIS_MODULE,
		.pm = &watchdog_pm_ops,
	},
	.probe = watchdog_probe,
	.remove = watchdog_remove,
};

static struct platform_device *watchdog_pm_pdev;

static int watchdog_module_init(void)
{
	pr_info(PFX "%s\n", __func__);
	return platform_driver_register(&watchdog_driver);
}

static void watchdog_module_exit(void)
{
	platform_driver_unregister(&watchdog_driver);
	platform_device_del(watchdog_pm_pdev);
	platform_device_put(watchdog_pm_pdev);
}

/* Init code */
static int intel_scu_watchdog_init(void)
{
	int ret = 0;

	watchdog_device.normal_wd_action = IPC_SET_SUB_COLDRESET;
	watchdog_device.reboot_wd_action = IPC_SET_SUB_COLDRESET;
	watchdog_device.shutdown_wd_action = IPC_SET_SUB_COLDOFF;

#ifdef CONFIG_DEBUG_FS
	watchdog_device.panic_reboot_notifier = false;
#endif /* CONFIG_DEBUG_FS */

	/* Initially, we are not in shutdown mode */
	watchdog_device.shutdown_flag = false;

	/* Check timeouts boot parameter */
	if (check_timeouts(pre_timeout, timeout)) {
		pr_err(PFX "%s: Invalid timeouts\n", __func__);
		return -EINVAL;
	}

	/* Reboot notifier */
	watchdog_device.reboot_notifier.notifier_call = reboot_notifier;
	watchdog_device.reboot_notifier.priority = 1;
	ret = register_reboot_notifier(&watchdog_device.reboot_notifier);
	if (ret) {
		pr_crit(PFX "cannot register reboot notifier %d\n", ret);
		goto error_stop_timer;
	}

	/* Do not publish the watchdog device when disable (TO BE REMOVED) */
	if (!disable_kernel_watchdog) {
		watchdog_device.miscdev.minor = WATCHDOG_MINOR;
		watchdog_device.miscdev.name = "watchdog";
		watchdog_device.miscdev.fops = &intel_scu_fops;

		ret = misc_register(&watchdog_device.miscdev);
		if (ret) {
			pr_crit(PFX "Cannot register miscdev %d err =%d\n",
				WATCHDOG_MINOR, ret);
			goto error_reboot_notifier;
		}
	}

	/* MSI #15 handler to dump registers */
	handle_mrfl_dev_ioapic(EXT_TIMER0_MSI);
	ret = request_irq((unsigned int)EXT_TIMER0_MSI,
		watchdog_warning_interrupt,
		IRQF_SHARED|IRQF_NO_SUSPEND, "watchdog",
		&watchdog_device);
	if (ret) {
		pr_err(PFX "error requesting warning irq %d\n",
		       EXT_TIMER0_MSI);
		pr_err(PFX "error value returned is %d\n", ret);
		goto error_misc_register;
	}

#ifdef CONFIG_INTEL_SCU_SOFT_LOCKUP
	init_timer(&softlock_timer);
#endif

	if (disable_kernel_watchdog) {
		pr_err(PFX "%s: Disable kernel watchdog\n", __func__);

		/* Make sure timer is stopped */
		ret = intel_scu_stop();
		if (ret != 0)
			pr_debug(PFX "cant disable timer\n");
	}

#ifdef CONFIG_DEBUG_FS
	ret = create_debugfs_entries();
	if (ret) {
		pr_err(PFX "%s: Error creating debugfs entries\n", __func__);
		goto error_debugfs_entry;
	}
#endif

	watchdog_device.started = false;

	ret = create_watchdog_sysfs_files();
	if (ret) {
		pr_err(PFX "%s: Error creating debugfs entries\n", __func__);
		goto error_sysfs_entry;
	}

	qos = kzalloc(sizeof(struct pm_qos_request), GFP_KERNEL);
	if (!qos) {
		pr_err(PFX "%s: Error allocating qos\n", __func__);
		ret = -ENOMEM;
		goto error_sysfs_entry;
	}

	pm_qos_add_request(qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);

	ret = watchdog_module_init();
	if (ret) {
		pr_err(PFX "%s: Error initializing pm\n", __func__);
		goto error_device_entry;
	}

	watchdog_pm_pdev = platform_device_alloc("watchdog_pm", -1);
	if (!watchdog_pm_pdev) {
		pr_err(PFX "%s: watchdog_pm allocation failed\n", __func__);
		ret = -ENODEV;
		goto error_device_entry;
	}
	ret = platform_device_add(watchdog_pm_pdev);
	if (ret) {
		pr_err(PFX "%s: watchdog_pm add failed\n", __func__);
		platform_device_put(watchdog_pm_pdev);
	}
	pr_info("platform watchdog_pm allocated\n");

	return ret;

error_device_entry:
	pm_qos_remove_request(qos);
	kfree(qos);

error_sysfs_entry:
	/* Nothing special to do */
#ifdef CONFIG_DEBUG_FS
error_debugfs_entry:
	/* Remove entries done by create function */
#endif

error_misc_register:
	misc_deregister(&watchdog_device.miscdev);

error_reboot_notifier:
	unregister_reboot_notifier(&watchdog_device.reboot_notifier);

error_stop_timer:
	intel_scu_stop();

	return ret;
}

static void intel_scu_watchdog_exit(void)
{
	int ret = 0;

	remove_watchdog_sysfs_files();
#ifdef CONFIG_DEBUG_FS
	remove_debugfs_entries();
#endif

#ifdef CONFIG_INTEL_SCU_SOFT_LOCKUP
	del_timer_sync(&softlock_timer);
#endif

	ret = intel_scu_stop();
	if (ret != 0)
		pr_err(PFX "cant disable timer\n");

	pm_qos_remove_request(qos);
	kfree(qos);

	misc_deregister(&watchdog_device.miscdev);
	unregister_reboot_notifier(&watchdog_device.reboot_notifier);

	watchdog_module_exit();
}

static int watchdog_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed watchdog rpmsg device\n");

	/* Allocate rpmsg instance for watchdog*/
	ret = alloc_rpmsg_instance(rpdev, &watchdog_instance);
	if (!watchdog_instance) {
		dev_err(&rpdev->dev, "kzalloc watchdog instance failed\n");
		goto out;
	}
	/* Initialize rpmsg instance */
	init_rpmsg_instance(watchdog_instance);
	/* Init scu watchdog */
	ret = intel_scu_watchdog_init();

	if (ret)
		free_rpmsg_instance(rpdev, &watchdog_instance);
out:
	return ret;
}

static void watchdog_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	intel_scu_watchdog_exit();
	free_rpmsg_instance(rpdev, &watchdog_instance);
	dev_info(&rpdev->dev, "Removed watchdog rpmsg device\n");
}

static void watchdog_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id watchdog_rpmsg_id_table[] = {
	{ .name	= "rpmsg_watchdog" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, watchdog_rpmsg_id_table);

static struct rpmsg_driver watchdog_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= watchdog_rpmsg_id_table,
	.probe		= watchdog_rpmsg_probe,
	.callback	= watchdog_rpmsg_cb,
	.remove		= watchdog_rpmsg_remove,
};

static int __init watchdog_rpmsg_init(void)
{
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		return register_rpmsg_driver(&watchdog_rpmsg);
	else {
		pr_err(PFX "%s: watchdog driver: bad platform\n", __func__);
		return -ENODEV;
	}
}

#ifdef MODULE
module_init(watchdog_rpmsg_init);
#else
rootfs_initcall(watchdog_rpmsg_init);
#endif

static void __exit watchdog_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&watchdog_rpmsg);
}
module_exit(watchdog_rpmsg_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_AUTHOR("mark.a.allyn@intel.com");
MODULE_AUTHOR("yannx.puech@intel.com");
MODULE_DESCRIPTION("Intel SCU Watchdog Device Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_VERSION(WDT_VER);
