/*
 * kernel/power/suspend.c - Suspend to RAM and standby functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 * Copyright (c) 2009 Rafael J. Wysocki <rjw@sisk.pl>, Novell Inc.
 *
 * This file is released under the GPLv2.
 */

#include <linux/string.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/syscalls.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/syscore_ops.h>
#include <linux/ftrace.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>
#include <trace/events/power.h>
#include <linux/module.h>
#include <linux/time.h>

#include "power.h"

#include <asm/intel_scu_pmic.h>
#include <asm/intel_scu_ipc.h>
#include <linux/mfd/intel_mid_pmic.h>

#ifdef CONFIG_SLEEPING_BEAUTY
#include <linux/gpio.h>
struct gpio_suspend_status gpio_sb_suspend_value[ARCH_NR_GPIOS];
#endif	// CONFIG_SLEEPING_BEAUTY

bool debug_suspend_enabled = 0;
EXPORT_SYMBOL(debug_suspend_enabled);
module_param_named(debug_suspend, debug_suspend_enabled,
			bool, S_IRUGO | S_IWUSR);

bool dump_pmic_register_enabled = 0;
EXPORT_SYMBOL(dump_pmic_register_enabled);
module_param_named(dump_pmic_register_enabled, dump_pmic_register_enabled,
			bool, S_IRUGO | S_IWUSR);

#ifdef CONFIG_SLEEPING_BEAUTY
bool dump_gpio_enabled = 0;
EXPORT_SYMBOL(dump_gpio_enabled);
module_param_named(dump_gpio_enabled, dump_gpio_enabled,
			bool, S_IRUGO | S_IWUSR);
extern void dump_gpio(void);

bool dump_ic_register_enabled = 0;
EXPORT_SYMBOL(dump_ic_register_enabled);
module_param_named(dump_ic_register_enabled, dump_ic_register_enabled,
			bool, S_IRUGO | S_IWUSR);
extern void sb_tp_0(void);
extern void sb_gsensor_0(void);
extern void sb_ecompass_0(void);
#endif	// CONFIG_SLEEPING_BEAUTY

static void do_suspend_sync(struct work_struct *work);

const char *const pm_states[PM_SUSPEND_MAX] = {
#ifdef CONFIG_EARLYSUSPEND
	[PM_SUSPEND_ON]		= "on",
#endif
	[PM_SUSPEND_FREEZE]	= "freeze",
	[PM_SUSPEND_STANDBY]	= "standby",
	[PM_SUSPEND_MEM]	= "mem",
};

static const struct platform_suspend_ops *suspend_ops;

static bool need_suspend_ops(suspend_state_t state)
{
	return !!(state > PM_SUSPEND_FREEZE);
}

static DECLARE_WORK(suspend_sync_work, do_suspend_sync);
static DECLARE_COMPLETION(suspend_sync_complete);

static DECLARE_WAIT_QUEUE_HEAD(suspend_freeze_wait_head);
static bool suspend_freeze_wake;

void dump_pmic_register(const char* message)
{
	if (dump_pmic_register_enabled) {
		u16 i=0;
		printk("%s: %s\n", __func__, message);
		for (i = 0; i < 0xff; i++) {
			int ret = intel_mid_pmic_readb(i);
			printk("%s: reg 0x%x = 0x%x\n", __func__, i, ret);
			if (ret) printk(".........reg read error!\n");
		}
	}
}
EXPORT_SYMBOL(dump_pmic_register);

static void freeze_begin(void)
{
	suspend_freeze_wake = false;
}

static void freeze_enter(void)
{
	wait_event(suspend_freeze_wait_head, suspend_freeze_wake);
}

void freeze_wake(void)
{
	suspend_freeze_wake = true;
	wake_up(&suspend_freeze_wait_head);
}
EXPORT_SYMBOL_GPL(freeze_wake);

static void do_suspend_sync(struct work_struct *work)
{
	sys_sync();
	complete(&suspend_sync_complete);
}

static bool check_sys_sync(void)
{
	while (!wait_for_completion_timeout(&suspend_sync_complete,
		HZ / 5)) {
		if (pm_wakeup_pending())
			return false;
		/* If sys_sync is doing, and no wakeup pending,
		 * we can try in loop to wait sys_sync() finish.
		 */
	}

	return true;
}

static bool suspend_sync(void)
{
	if (work_busy(&suspend_sync_work)) {
		/* When last sys_sync() work is still running,
		 * we need wait for it to be finished.
		 */
		if (!check_sys_sync())
			return false;
	}

	INIT_COMPLETION(suspend_sync_complete);
	schedule_work(&suspend_sync_work);

	return check_sys_sync();
}


/**
 * suspend_set_ops - Set the global suspend method table.
 * @ops: Suspend operations to use.
 */
void suspend_set_ops(const struct platform_suspend_ops *ops)
{
	lock_system_sleep();
	suspend_ops = ops;
	unlock_system_sleep();
}
EXPORT_SYMBOL_GPL(suspend_set_ops);

bool valid_state(suspend_state_t state)
{
	if (state == PM_SUSPEND_FREEZE) {
#ifdef CONFIG_PM_DEBUG
		if (pm_test_level != TEST_NONE &&
		    pm_test_level != TEST_FREEZER &&
		    pm_test_level != TEST_DEVICES &&
		    pm_test_level != TEST_PLATFORM) {
			printk(KERN_WARNING "Unsupported pm_test mode for "
					"freeze state, please choose "
					"none/freezer/devices/platform.\n");
			return false;
		}
#endif
			return true;
	}
	/*
	 * PM_SUSPEND_STANDBY and PM_SUSPEND_MEMORY states need lowlevel
	 * support and need to be valid to the lowlevel
	 * implementation, no valid callback implies that none are valid.
	 */
	return suspend_ops && suspend_ops->valid && suspend_ops->valid(state);
}

/**
 * suspend_valid_only_mem - Generic memory-only valid callback.
 *
 * Platform drivers that implement mem suspend only and only need to check for
 * that in their .valid() callback can use this instead of rolling their own
 * .valid() callback.
 */
int suspend_valid_only_mem(suspend_state_t state)
{
	return state == PM_SUSPEND_MEM;
}
EXPORT_SYMBOL_GPL(suspend_valid_only_mem);

static int suspend_test(int level)
{
#ifdef CONFIG_PM_DEBUG
	if (pm_test_level == level) {
		printk(KERN_INFO "suspend debug: Waiting for 5 seconds.\n");
		mdelay(5000);
		return 1;
	}
#endif /* !CONFIG_PM_DEBUG */
	return 0;
}

/**
 * suspend_prepare - Prepare for entering system sleep state.
 *
 * Common code run for every system sleep state that can be entered (except for
 * hibernation).  Run suspend notifiers, allocate the "suspend" console and
 * freeze processes.
 */
static int suspend_prepare(suspend_state_t state)
{
	int error;

	if (need_suspend_ops(state) && (!suspend_ops || !suspend_ops->enter))
		return -EPERM;

	pm_prepare_console();

	error = pm_notifier_call_chain(PM_SUSPEND_PREPARE);
	if (error)
		goto Finish;

	error = suspend_freeze_processes();
	if (!error)
		return 0;

	suspend_stats.failed_freeze++;
	dpm_save_failed_step(SUSPEND_FREEZE);
 Finish:
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
	return error;
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_disable_irqs(void)
{
	local_irq_disable();
}

/* default implementation */
void __attribute__ ((weak)) arch_suspend_enable_irqs(void)
{
	local_irq_enable();
}

/**
 * suspend_enter - Make the system enter the given sleep state.
 * @state: System sleep state to enter.
 * @wakeup: Returns information that the sleep state should not be re-entered.
 *
 * This function should be called after devices have been suspended.
 */
static int suspend_enter(suspend_state_t state, bool *wakeup)
{
	int error;
	struct timespec after, before;

	if (need_suspend_ops(state) && suspend_ops->prepare) {
		error = suspend_ops->prepare();
		if (error)
			goto Platform_finish;
	}

#ifdef CONFIG_SLEEPING_BEAUTY
	if (dump_gpio_enabled) {
		int i=0;
		for ( i=0 ; i<ARCH_NR_GPIOS ; i++) {
			if ( gpio_sb_suspend_value[i].bChanged ) {
				error = gpio_direction_output(i, gpio_sb_suspend_value[i].uNewSuspendValue);
				gpio_set_value(i, gpio_sb_suspend_value[i].uNewSuspendValue);
				//error = gpio_direction_output(108, 1);
				if (error)
					printk("gpio suspend change: fail! gpio-%d, gpio-new-value: %d",
							i, gpio_sb_suspend_value[i].uNewSuspendValue);
			}
		}
		printk("SB Debug: dump_gpio in suspend_enter\n");
		dump_gpio();
	}
	if (dump_ic_register_enabled) {
		printk("SB Debug: dump_ic_register in suspend_enter\n");
		sb_tp_0();	// dump touch ic part.
		sb_gsensor_0(); // dump g-sensor ic part.
		sb_ecompass_0(); // dump e-comapss ic part.
	}
#endif	// CONFIG_SLEEPING_BEAUTY

	error = dpm_suspend_end(PMSG_SUSPEND);
	dump_pmic_register("dpm_suspend_end finish");
	if (error) {
		printk(KERN_ERR "PM: Some devices failed to power down\n");
		goto Platform_finish;
	}

	if (need_suspend_ops(state) && suspend_ops->prepare_late) {
		error = suspend_ops->prepare_late();
		if (error)
			goto Platform_wake;
	}

	if (suspend_test(TEST_PLATFORM))
		goto Platform_wake;

	/*
	 * PM_SUSPEND_FREEZE equals
	 * frozen processes + suspended devices + idle processors.
	 * Thus we should invoke freeze_enter() soon after
	 * all the devices are suspended.
	 */
	if (state == PM_SUSPEND_FREEZE) {
		freeze_enter();
		goto Platform_wake;
	}

	error = disable_nonboot_cpus();
	if (error || suspend_test(TEST_CPUS))
		goto Enable_cpus;

	arch_suspend_disable_irqs();
	BUG_ON(!irqs_disabled());

	error = syscore_suspend();

	if (debug_suspend_enabled) {
		read_persistent_clock(&before);
		//printk("Entering suspend state LP0");
	}

	printk("Entering suspend state LP0\n");

	if (!error) {
		*wakeup = pm_wakeup_pending();
		if (!(suspend_test(TEST_CORE) || *wakeup)) {
			error = suspend_ops->enter(state);
			events_check_enabled = false;
		}
		syscore_resume();
	}

	if (debug_suspend_enabled) {
		read_persistent_clock(&after);
		after = timespec_sub(after, before);
		printk("Suspended for %lu.%03lu seconds\n", after.tv_sec, 0);
	}

	arch_suspend_enable_irqs();
	BUG_ON(irqs_disabled());

 Enable_cpus:
	enable_nonboot_cpus();

 Platform_wake:
	if (need_suspend_ops(state) && suspend_ops->wake)
		suspend_ops->wake();

	dpm_resume_start(PMSG_RESUME);

 Platform_finish:
	if (need_suspend_ops(state) && suspend_ops->finish)
		suspend_ops->finish();

	return error;
}

/**
 * suspend_devices_and_enter - Suspend devices and enter system sleep state.
 * @state: System sleep state to enter.
 */
int suspend_devices_and_enter(suspend_state_t state)
{
	int error;
	bool wakeup = false;

	if (need_suspend_ops(state) && !suspend_ops)
		return -ENOSYS;

	trace_machine_suspend(state);
	if (need_suspend_ops(state) && suspend_ops->begin) {
		error = suspend_ops->begin(state);
		if (error)
			goto Close;
	}
	suspend_console();
	ftrace_stop();
	suspend_test_start();
	error = dpm_suspend_start(PMSG_SUSPEND);
	dump_pmic_register("dpm_suspend_start finish");
	if (error) {
		printk(KERN_ERR "PM: Some devices failed to suspend\n");
		goto Recover_platform;
	}
	suspend_test_finish("suspend devices");
	if (suspend_test(TEST_DEVICES))
		goto Recover_platform;

	do {
		error = suspend_enter(state, &wakeup);
	} while (!error && !wakeup && need_suspend_ops(state)
		&& suspend_ops->suspend_again && suspend_ops->suspend_again());

 Resume_devices:
	suspend_test_start();
	dpm_resume_end(PMSG_RESUME);
	suspend_test_finish("resume devices");
	ftrace_start();
	resume_console();
 Close:
	if (need_suspend_ops(state) && suspend_ops->end)
		suspend_ops->end();
	trace_machine_suspend(PWR_EVENT_EXIT);
	return error;

 Recover_platform:
	if (need_suspend_ops(state) && suspend_ops->recover)
		suspend_ops->recover();
	goto Resume_devices;
}

/**
 * suspend_finish - Clean up before finishing the suspend sequence.
 *
 * Call platform code to clean up, restart processes, and free the console that
 * we've allocated. This routine is not called for hibernation.
 */
static void suspend_finish(void)
{
	suspend_thaw_processes();
	pm_notifier_call_chain(PM_POST_SUSPEND);
	pm_restore_console();
}

/**
 * enter_state - Do common work needed to enter system sleep state.
 * @state: System sleep state to enter.
 *
 * Make sure that no one else is trying to put the system into a sleep state.
 * Fail if that's not the case.  Otherwise, prepare for system suspend, make the
 * system enter the given sleep state and clean up after wakeup.
 */
static int enter_state(suspend_state_t state)
{
	int error;

	if (!valid_state(state))
		return -ENODEV;

	if (!mutex_trylock(&pm_mutex))
		return -EBUSY;

	if (state == PM_SUSPEND_FREEZE)
		freeze_begin();

	printk(KERN_INFO "PM: Syncing filesystems ... ");
	if (!suspend_sync()) {
		printk(KERN_INFO "PM: Suspend aborted for filesystem syncing\n");
		error = -EBUSY;
		goto Unlock;
	}
	printk("done.\n");

	pr_debug("PM: Preparing system for %s sleep\n", pm_states[state]);
	error = suspend_prepare(state);
	if (error)
		goto Unlock;

	if (suspend_test(TEST_FREEZER))
		goto Finish;

	dump_pmic_register("Entering suspend devices");
	pr_debug("PM: Entering %s sleep\n", pm_states[state]);
	pm_restrict_gfp_mask();
	error = suspend_devices_and_enter(state);
	pm_restore_gfp_mask();

 Finish:
	pr_debug("PM: Finishing wakeup.\n");
	suspend_finish();
 Unlock:
	mutex_unlock(&pm_mutex);
	return error;
}

static void pm_suspend_marker(char *annotation)
{
	struct timespec ts;
	struct rtc_time tm;

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	pr_info("PM: suspend %s %d-%02d-%02d %02d:%02d:%02d.%09lu UTC\n",
		annotation, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

/**
 * pm_suspend - Externally visible function for suspending the system.
 * @state: System sleep state to enter.
 *
 * Check if the value of @state represents one of the supported states,
 * execute enter_state() and update system suspend statistics.
 */
int pm_suspend(suspend_state_t state)
{
	int error;

	if (state <= PM_SUSPEND_ON || state >= PM_SUSPEND_MAX)
		return -EINVAL;

	pm_suspend_marker("entry");
	error = enter_state(state);
	if (error) {
		suspend_stats.fail++;
		dpm_save_failed_errno(error);
	} else {
		suspend_stats.success++;
	}
	pm_suspend_marker("exit");
	return error;
}
EXPORT_SYMBOL(pm_suspend);
