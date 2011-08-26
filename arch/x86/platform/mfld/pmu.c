/*
 * pmu.c - This driver provides interface to configure the 2 pmu's
 * Copyright (c) 2010, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/jhash.h>
#include <linux/intel_mid_pm.h>
#include <linux/suspend.h>
#include <linux/wakelock.h>

#include "pmu.h"

static	struct pci_dev_index	pci_dev_hash[MID_PCI_INDEX_HASH_SIZE];


static int dev_init_state;
static struct intel_mid_base_addr base_addr;

#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock pmu_wake_lock;
#endif

static u32 apm_base;
static u32 ospm_base;
static spinlock_t nc_ready_lock;

/* debug counters */
static u32 pmu_wait_done_calls;
static u32 pmu_wait_done_udelays;
static u32 pmu_wait_done_udelays_max;

/*
 * Locking strategy::
 *
 * pmu_pci_set_power_state() is synchronous -- it waits
 * for the SCU to be not-busy before and after commands.
 * Drivers may access devices immediately upon its return.
 *
 * In Idle handler case::
 * Idle context:
 * idle_handler()->try_acquire_scu_ready_sem->if acquired->
 * issue s0ix command->return
 *
 * PMU Interrupt context:
 * pmu_Interrupt_handler()->release scu_ready_sem->return
 *
 */
static struct semaphore scu_ready_sem =
		__SEMAPHORE_INITIALIZER(scu_ready_sem, 1);

/* handle concurrent SMP invokations of pmu_pci_set_power_state() */
static spinlock_t mrst_pmu_power_state_lock;

static int scu_comms_okay = 1;

#ifdef ENABLE_SCU_WATCHDOG
/* scu semaphore watchdog timer */
static void scu_watchdog_func(unsigned long arg)
{
	struct semaphore* sem = (struct semaphore *) arg;

	WARN(1, "SCU watchdog triggered, dead lock occured,"
					 "diabling SCU comms.\n");
	scu_comms_okay = 0;

	/* unblock the blocking thread */
	up(sem);
}
#endif

/*
 * Acquire the said semaphore and activating a watchdog timer
 * that triggers after timeout, if we are able to acquire then
 * delete the timer.
 */
static void down_scu_timed(struct semaphore *sem)
{
#ifdef ENABLE_SCU_WATCHDOG
	struct timer_list *scu_watchdog_timer =
			kmalloc(sizeof(struct timer_list), GFP_KERNEL);

	if (!scu_watchdog_timer) {
		WARN(1, "Malloc failed.\n");
		return;
	}

	init_timer(scu_watchdog_timer);
	scu_watchdog_timer->function	= scu_watchdog_func;
	scu_watchdog_timer->data	= (unsigned long) sem;

	/*10secs timeout*/
	scu_watchdog_timer->expires	=
				jiffies + msecs_to_jiffies(10 * 1000);
	add_timer(scu_watchdog_timer);
#endif

	down(sem);

#ifdef ENABLE_SCU_WATCHDOG
	del_timer_sync(scu_watchdog_timer);
	kfree(scu_watchdog_timer);
#endif
}

/* Device information for all the PCI devices present on SC */
static struct pci_dev_info intel_mid_pci_devices[MAX_DEVICES];
static int num_wakes[MAX_DEVICES];
static int cmd_error_int;
static u64 pmu_init_time;
static struct mid_pmu_stats {
	u64 err_count[3];
	u64 count;
	u64 time;
	u64 last_entry;
	u64 last_try;
	u64 first_entry;
} pmu_stats[SYS_STATE_S5];
static enum sys_state  pmu_current_state;

static void pmu_stat_start(enum sys_state type)
{
	pmu_current_state = type;
	pmu_stats[type].last_try = cpu_clock(smp_processor_id());
}

static void pmu_stat_end(void)
{
	enum sys_state type = pmu_current_state;

	if (type > SYS_STATE_S0I0) {
		pmu_stats[type].last_entry =
			pmu_stats[type].last_try;

		/*if it is the first entry save the time*/
		if (!pmu_stats[type].count)
			pmu_stats[type].first_entry =
				pmu_stats[type].last_entry;

		pmu_stats[type].time +=
			cpu_clock(smp_processor_id())
			- pmu_stats[type].last_entry;

		pmu_stats[type].count++;
	}

	pmu_current_state = SYS_STATE_S0I0;
}

static void pmu_stat_clear(void)
{
	pmu_current_state = SYS_STATE_S0I0;
}

static void pmu_stat_error(u8 err_type)
{
	enum sys_state type = pmu_current_state;
	u8 err_index;

	if (type > SYS_STATE_S0I0) {
		switch (err_type) {
		case SUBSYS_POW_ERR_INT:
			trace_printk("S0ix_POW_ERR_INT\n");
			err_index = 0;
			break;
		case S0ix_MISS_INT:
			trace_printk("S0ix_MISS_INT\n");
			err_index = 1;
			break;
		case NO_ACKC6_INT:
			trace_printk("S0ix_ACKC6_INT\n");
			err_index = 2;
			break;
		default:
			err_index = 3;
			break;
		}

		if (err_index < 3)
			pmu_stats[type].err_count[err_index]++;
	}
}

static void pmu_stat_seq_printf(struct seq_file *s, int type, char *typestr)
{
	unsigned long long t;
	unsigned long nanosec_rem, remainder;
	unsigned long time, init_2_now_time;

	seq_printf(s, "%s\t%5llu\t%10llu\t%9llu\t%9llu\t", typestr,
		 pmu_stats[type].count,
		 pmu_stats[type].err_count[0],
		 pmu_stats[type].err_count[1],
		 pmu_stats[type].err_count[2]);

	t = pmu_stats[type].time;
	nanosec_rem = do_div(t, 1000000000);

	/* convert time in secs */
	time = (unsigned long)t;

	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t = pmu_stats[type].last_entry;
	nanosec_rem = do_div(t, 1000000000);
	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t = pmu_stats[type].first_entry;
	nanosec_rem = do_div(t, 1000000000);
	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t =  cpu_clock(raw_smp_processor_id());
	t -= pmu_init_time;
	nanosec_rem = do_div(t, 1000000000);

	init_2_now_time =  (unsigned long) t;

	/* for calculating percentage residency */
	time = time * 100;
	t = (u64) time;
	remainder = do_div(t, init_2_now_time);
	time = (unsigned long) t;

	/* for getting 3 digit precision after
	 * decimal dot */
	remainder *= 1000;
	t = (u64) remainder;
	remainder = do_div(t, init_2_now_time);
	seq_printf(s, "%5lu.%03lu\n", time, (unsigned long) t);
}

/*Experimentally enabling S0i3 as extended C-States*/
static char s0ix[5] = "s0i3";
static int extended_cstate_mode = MID_S0I3_STATE;
static int set_extended_cstate_mode(const char *val, struct kernel_param *kp)
{
	char       valcp[5];
	strncpy(valcp, val, 5);
	valcp[4] = '\0';

#ifdef CONFIG_HAS_WAKELOCK
	/* will also disable s0ix */
	if (strcmp(valcp, "s3s3") == 0)
		wake_unlock(&pmu_wake_lock);
#endif

	/* Acquire the scu_ready_sem */
	down_scu_timed(&scu_ready_sem);

	if (strcmp(valcp, "s0i3") == 0)
		extended_cstate_mode = MID_S0I3_STATE;
	else if (strcmp(valcp, "s0i1") == 0)
		extended_cstate_mode = MID_S0I1_STATE;
	else if (strcmp(valcp, "s0ix") == 0)
		extended_cstate_mode = MID_S0IX_STATE;
	else {
		extended_cstate_mode = -1;
		strncpy(valcp, "none", 5);
	}
	strncpy(s0ix, valcp, 5);

	up(&scu_ready_sem);

	return 0;
}

/* Maps pci power states to SCU D0ix mask */
static int pci_2_mfld_state(pci_power_t pci_state)
{
	int mfld_state = D0I0_MASK;

	switch (pci_state) {
	case PCI_D0:
		mfld_state = D0I0_MASK;
		break;

	case PCI_D1:
		mfld_state = D0I1_MASK;
		break;

	case PCI_D2:
		mfld_state = D0I2_MASK;
		break;

	case PCI_D3cold:
	case PCI_D3hot:
		mfld_state = D0I3_MASK;
		break;

	default:
		WARN(1, "%s: wrong pci_state received.\n", __func__);
		break;
	}

	return mfld_state;
}

static int get_extended_cstate_mode(char *buffer, struct kernel_param *kp)
{
	strcpy(buffer, s0ix);
	return 4;
}

module_param_call(s0ix, set_extended_cstate_mode,
		get_extended_cstate_mode, NULL, 0644);
MODULE_PARM_DESC(s0ix,
		"setup extended c state s0ix mode [s0i3|s0i1|s0ix|none]");

/* the device object */
static struct pci_dev *pmu_dev;

/*virtual memory address for PMU base returned by ioremap_nocache().*/
static struct mid_pmu_dev intel_mid_pmu_base;

#ifdef CONFIG_VIDEO_ATOMISP
static int camera_off;
#else
static int camera_off = 1;
#endif

#ifdef GFX_ENABLE
static int display_off;
#else
/* If Gfx is disabled
 * assume s0ix is not blocked
 * from gfx side
 */
static int display_off = 1;
#endif

static int s0i1_possible;
static int lpmp3_possible;
static int s0i3_possible;
static int c6_demoted;
static int s0ix_entered;
static u32 pmu1_max_devs, pmu2_max_devs, ss_per_reg;

static int _pmu_issue_command(struct pmu_ss_states *pm_ssc, int mode,
		       int pmu_num);
static void pmu_read_sss(struct pmu_ss_states *pm_ssc);
static int _pmu2_wait_not_busy(void);

/* PCI Device Id structure */
static DEFINE_PCI_DEVICE_TABLE(mid_pm_ids) = {
	{PCI_VDEVICE(INTEL, MID_PMU_MRST_DRV_DEV_ID), 0},
	{PCI_VDEVICE(INTEL, MID_PMU_MFLD_DRV_DEV_ID), 0},
	{}
};

MODULE_DEVICE_TABLE(pci, mid_pm_ids);

int get_target_platform_state(void)
{
	if (likely(scu_comms_okay &&
		(extended_cstate_mode != -1) && display_off\
		 && camera_off))
		return extended_cstate_mode;

	return 0;
}
EXPORT_SYMBOL(get_target_platform_state);

static int _pmu_read_status(int pmu_num, int type)
{
	u32 temp;
	union pmu_pm_status result;

	temp = readl(&pmu_reg->pm_sts);

	/* extract the busy bit */
	result.pmu_status_value = temp;

	if (type == PMU_BUSY_STATUS)
		return result.pmu_status_parts.pmu_busy;
	else if (type == PMU_MODE_ID)
		return result.pmu_status_parts.mode_id;

	return 0;
}

static int _pmu2_wait_not_busy(void)
{
	int pmu_busy_retry = 500000;

	/* wait 500ms that the latest pmu command finished */
	while (--pmu_busy_retry) {
		if (_pmu_read_status(PMU_NUM_2, PMU_BUSY_STATUS) == 0)
			return 0;

		udelay(1);
	}

	WARN(1, "pmu2 busy!");

	return -EBUSY;
}

static void pmu_write_subsys_config(struct pmu_ss_states *pm_ssc)
{
	pr_debug("pmu_write_subsys_config: 0x%08lX.%08lX.%08lX.%08lX\n",
		pm_ssc->pmu2_states[3], pm_ssc->pmu2_states[2],
		pm_ssc->pmu2_states[1], pm_ssc->pmu2_states[0]);

	/* South complex in Penwell has multiple registers for
	 * PM_SSC, etc.
	 */
	writel(pm_ssc->pmu2_states[0], &pmu_reg->pm_ssc[0]);

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		writel(pm_ssc->pmu2_states[1], &pmu_reg->pm_ssc[1]);
		writel(pm_ssc->pmu2_states[2], &pmu_reg->pm_ssc[2]);
		writel(pm_ssc->pmu2_states[3], &pmu_reg->pm_ssc[3]);
	}
}

/**
 * pmu_set_cfg_mode_params - platform specific configuration mode parameters
 */
static int pmu_set_cfg_mode_params(struct cfg_mode_params
	 *cfg_mode_parms, union pmu_pm_set_cfg_cmd_t *command, int pmu_num)
{
	struct cfg_trig_param_t cmd;

	/* parameter check */
	if (cfg_mode_parms->cfg_mode >= CM_INVALID) {
		pr_alert("invalid config mode\n");
		return PMU_FAILED;
	}

	if (cfg_mode_parms->cfg_cmbi > 1) {
		pr_alert("invalid cfg_cmbi bit\n");
		return PMU_FAILED;
	}

	if (cfg_mode_parms->cfg_trigger >= INVALID_TRIG) {
		pr_alert("invalid cfg_trigger mode\n");
		return PMU_FAILED;
	}

	if (command == NULL) {
		pr_alert("invalid pointer\n");
		return PMU_FAILED;
	}

	if (pmu_num == PMU_NUM_1) {
		command->pmu1_params.cfg_mode = cfg_mode_parms->cfg_mode;
		command->pmu1_params.cfg_delay = cfg_mode_parms->cfg_delay;
		command->pmu1_params.cfg_trigger = cfg_mode_parms->cfg_trigger;
	} else if (pmu_num == PMU_NUM_2) {
		command->pmu2_params.d_param.cfg_mode =
		    cfg_mode_parms->cfg_mode;

		/* Based on the cfg_mode set these parameters */
		switch (cfg_mode_parms->cfg_mode) {
		case CM_NOP:
		case CM_IMMEDIATE:
			command->pmu2_params.d_param.cfg_delay = 0;
			command->pmu2_params.d_param.rsvd = 0;
			break;
		case CM_DELAY:
			command->pmu2_params.d_param.cfg_delay =
			    cfg_mode_parms->cfg_delay;
			command->pmu2_params.d_param.rsvd = 0;
			break;
		case CM_TRIGGER:
			/* Based on the trigger type construct the cfg_trigger
			 * parameters
			 */
			cmd = command->pmu2_params.t_param;
			switch (cfg_mode_parms->cfg_trigger) {
			case NO_TRIG:
				cmd.cfg_trig_type = cfg_mode_parms->cfg_trigger;
				cmd.cmbi = cfg_mode_parms->cfg_cmbi;
				cmd.rsvd1 = 0;
				cmd.cfg_trig_val = cfg_mode_parms->cfg_trig_val;
				break;
			case LPM_EVENT_TRIG:
				cmd.cfg_trig_type = cfg_mode_parms->cfg_trigger;
				cmd.cmbi = cfg_mode_parms->cfg_cmbi;
				cmd.rsvd1 = 0;
				if (cfg_mode_parms->cfg_trig_val != PME_ID_LPE
				    && cfg_mode_parms->cfg_trig_val !=
				    PME_ID_USB) {
					pr_alert("pme_Id"
					"not supported in this release\n");
					return PMU_FAILED;
				}
				cmd.cfg_trig_val = cfg_mode_parms->cfg_trig_val;
				break;
			case EXT_GPIO_INPUT_TRIG:
				cmd.cfg_trig_type = cfg_mode_parms->cfg_trigger;
				cmd.cmbi = cfg_mode_parms->cfg_cmbi;
				cmd.rsvd1 = 0;
				cmd.cfg_trig_val = cfg_mode_parms->cfg_trig_val;
				break;
			case C_STATE_TRANS_TRIG:
				cmd.cfg_trig_type = cfg_mode_parms->cfg_trigger;
				cmd.cmbi = cfg_mode_parms->cfg_cmbi;
				cmd.rsvd1 = 0;
				cmd.cfg_trig_val = cfg_mode_parms->cfg_trig_val;
				break;
			default:
				pr_alert("invalid state\n");
				return PMU_FAILED;
				break;
			};
			command->pmu2_params.t_param = cmd;
			break;
		default:
			WARN_ON(1);
			return PMU_FAILED;
		};
	}
	return 0;
}

/**
 * pmu_send_set_config_command - Platform specific function to send
 * configuration commands to Firmware
 */
static int pmu_send_set_config_command(union pmu_pm_set_cfg_cmd_t
		     *command, enum sys_state state, int pmu_num)
{
	/* construct the command to send SET_CFG to particular PMU */
	if (pmu_num == PMU_NUM_1) {
		command->pmu1_params.cmd = SET_CFG_CMD;
		command->pmu1_params.ioc = 0;
		command->pmu1_params.mode_id = MODE_ID_MAGIC_NUM;
		command->pmu1_params.rsvd = 0;
	} else if (pmu_num == PMU_NUM_2) {
		command->pmu2_params.d_param.cmd =
		    SET_CFG_CMD;
		command->pmu2_params.d_param.ioc = 0;
		command->pmu2_params.d_param.mode_id =
		    MODE_ID_MAGIC_NUM;

		/* set the sys state */
		switch (state) {
		case SYS_STATE_S0I0:
		case SYS_STATE_S0I1:
		case SYS_STATE_S0I3:
			command->pmu2_params.
			    d_param.sys_state = state;
			break;
		default:
			pr_alert("invalid state\n");
			return PMU_FAILED;
			break;
		}
	}

	writel(command->pmu_pm_set_cfg_cmd_value, &pmu_reg->pm_cmd);

	return 0;
}

/* return the last wake source id, and make statistics about wake sources */
static int pmu_get_wake_source(void)
{
	u32 wake0, wake1;
	int i;
	int source = INVALID_WAKE_SRC;

	wake0 = readl(&pmu_reg->pm_wks[0]);
	wake1 = readl(&pmu_reg->pm_wks[1]);

	while (wake0) {
		i = fls(wake0) - 1;
		source = i + pmu1_max_devs;
		num_wakes[source]++;
		trace_printk("wake_from_lss%d\n", source - pmu1_max_devs);
		wake0 &= ~(1<<i);
	}

	while (wake1) {
		i = fls(wake1) - 1;
		source = i + 32 + pmu1_max_devs;
		trace_printk("wake_from_lss%d\n", source - pmu1_max_devs);
		num_wakes[source]++;
		wake1 &= ~(1<<i);
	}

	return source;
}

static int pmu_is_interrupt_pending(int pmu_num)
{
	u32 temp;
	union pmu_pm_ics result;

	/* read the pm interrupt status register */
	temp = readl(&pmu_reg->pm_ics);
	result.pmu_pm_ics_value = temp;

	/* return the pm interrupt status int pending bit info */
	return result.pmu_pm_ics_parts.int_pend;
}

static inline void pmu_clear_interrupt_pending(int pmu_num)
{
	u32 temp;

	/* read the pm interrupt status register */
	temp = readl(&pmu_reg->pm_ics);

	/* write into the PM_ICS register */
	writel(temp, &pmu_reg->pm_ics);
}

static inline void pmu_enable_interrupt_from_pmu(int pmu_num)
{
	u32 temp;
	union pmu_pm_ics result;

	/* read the pm interrupt status register */
	temp = readl(&pmu_reg->pm_ics);
	result = (union pmu_pm_ics)temp;

	result.pmu_pm_ics_parts.int_enable = 1;

	/* enable the interrupts */
	writel(result.pmu_pm_ics_value, &pmu_reg->pm_ics);
}

static inline int pmu_read_interrupt_status(int pmu_num)
{
	u32 temp;
	union pmu_pm_ics result;

	/* read the pm interrupt status register */
	temp = readl(&pmu_reg->pm_ics);

	result.pmu_pm_ics_value = temp;

	if (result.pmu_pm_ics_parts.int_status == 0)
		return PMU_FAILED;

	/* return the pm interrupt status int pending bit info */
	return result.pmu_pm_ics_parts.int_status;
}

/**
 * This is a helper function used to program pm registers
 * in SCU. We configure the wakeable devices & the
 * wake sub system states on exit from S0ix state
 */
static inline int _mfld_s0ix_enter(u32 s0ix_value)
{
	struct pmu_ss_states cur_pmsss;

	/* If display status is 0 retry on next c6 */
	if (unlikely((display_off == 0) || (camera_off == 0))) {
		up(&scu_ready_sem);
		goto ret;
	}

	/* If PMU is busy, we'll retry on next C6 */
	if (unlikely(_pmu_read_status(PMU_NUM_2, PMU_BUSY_STATUS))) {
		up(&scu_ready_sem);
		goto ret;
	}

	/* setup the wake capable devices */
	writel(intel_mid_pmu_base.ss_config->wake_state.wake_enable[0],
			&pmu_reg->pm_wkc[0]);
	writel(intel_mid_pmu_base.ss_config->wake_state.wake_enable[1],
			&pmu_reg->pm_wkc[1]);

	/* Re-program the sub systems state on wakeup as the current SSS*/
	pmu_read_sss(&cur_pmsss);

	writel(cur_pmsss.pmu2_states[0], &pmu_reg->pm_wssc[0]);
	writel(cur_pmsss.pmu2_states[1], &pmu_reg->pm_wssc[1]);
	writel(cur_pmsss.pmu2_states[2], &pmu_reg->pm_wssc[2]);
	writel(cur_pmsss.pmu2_states[3], &pmu_reg->pm_wssc[3]);

	switch (s0ix_value) {
	case S0I1_VALUE:
		wrmsr(MSR_C6OFFLOAD_CTL_REG,
			MSR_C6OFFLOAD_CLEAR_LOW, MSR_C6OFFLOAD_CLEAR_HIGH);
		writel(S0I1_SSS0, &pmu_reg->pm_ssc[0]);
		writel(S0I1_SSS1, &pmu_reg->pm_ssc[1]);
		writel(S0I1_SSS2, &pmu_reg->pm_ssc[2]);
		writel(S0I1_SSS3, &pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I1);
		break;

	case LPMP3_VALUE:
		wrmsr(MSR_C6OFFLOAD_CTL_REG,
			MSR_C6OFFLOAD_CLEAR_LOW, MSR_C6OFFLOAD_CLEAR_HIGH);
		writel(LPMP3_SSS0, &pmu_reg->pm_ssc[0]);
		writel(LPMP3_SSS1, &pmu_reg->pm_ssc[1]);
		writel(LPMP3_SSS2, &pmu_reg->pm_ssc[2]);
		writel(LPMP3_SSS3, &pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I2);
		break;

	case S0I3_VALUE:
		writel(S0I3_SSS0, &pmu_reg->pm_ssc[0]);
		writel(S0I3_SSS1, &pmu_reg->pm_ssc[1]);
		writel(S0I3_SSS2, &pmu_reg->pm_ssc[2]);
		writel(S0I3_SSS3, &pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I3);
		break;
	}

	/* issue a command to SCU */
	writel(s0ix_value, &pmu_reg->pm_cmd);
	return 1;

ret:
	return 0;
}

int mfld_s0i1_enter(void)
{
	/* check if we can acquire scu_ready_sem
	 * if we are not able to then do a c6 */
	if (down_trylock(&scu_ready_sem))
		goto ret;

	if (!s0i1_possible && !lpmp3_possible) {
		up(&scu_ready_sem);
		goto ret;
	}

	s0ix_entered =
		_mfld_s0ix_enter(s0i1_possible ? S0I1_VALUE : LPMP3_VALUE);

ret:
	return s0ix_entered;
}
EXPORT_SYMBOL(mfld_s0i1_enter);

int mfld_s0i3_enter(void)
{
	u32 ssw_val = 0;
	int num_retry = 15000;
	int status = 0;

	/* skip S0i3 if SCU is not okay */
	if (unlikely(!scu_comms_okay))
		goto ret;

	/* check if we can acquire scu_ready_sem
	 * if we are not able to then do a c6 */
	if (down_trylock(&scu_ready_sem))
		goto ret;

	if (!s0i3_possible) {
		up(&scu_ready_sem);
		goto ret;
	}

	s0ix_entered = _mfld_s0ix_enter(S0I3_VALUE);

	/* If s0i3 command is issued
	 * then wait for c6 sram offload
	 * to complete */
	if (s0ix_entered) {
		do {
			ssw_val = readl(base_addr.offload_reg);

			if ((ssw_val & C6OFFLOAD_BIT_MASK) ==  C6OFFLOAD_BIT)
				break;

		} while (num_retry--);

		/* enable c6Offload only if write access bit is set
		 * if not then we will demote c6 to a c4 */
		if (likely((ssw_val & C6OFFLOAD_BIT_MASK) ==  C6OFFLOAD_BIT)) {
			wrmsr(MSR_C6OFFLOAD_CTL_REG,
				MSR_C6OFFLOAD_SET_LOW, MSR_C6OFFLOAD_SET_HIGH);
			status = 1;
		} else {
			pmu_stat_clear();
			WARN(1, "mid_pmu: error cpu offload bit not set.\n");
		}
	}
ret:
	return status;
}
EXPORT_SYMBOL(mfld_s0i3_enter);

/**
 * pmu_sc_irq - pmu driver interrupt handler
 * Context: interrupt context
 *
 * PMU interrupt handler based on the status of the interrupt
 * will send net link event to PM Framework.
 * - During inactivity interrupt it sends the information  of inactivity
 * counter that caused the interrupt as net link event.
 * - During wake interrupt it sends the information  of source of wake
 * sub system that caused the wake interrupt as net link event.
 */
static irqreturn_t pmu_sc_irq(int irq, void *ignored)
{
	u8 status = IRQ_NONE;

	/* check if interrup pending bit is set, if not ignore interrupt */
	if (unlikely(!pmu_is_interrupt_pending(PMU_NUM_2)))
		goto ret_no_clear;

	/* read the interrupt status */
	status = pmu_read_interrupt_status(PMU_NUM_2);
	if (unlikely(status == PMU_FAILED))
		dev_dbg(&pmu_dev->dev, "Invalid interrupt source\n");

	switch (status) {
	case INVALID_INT:
		status = IRQ_NONE;
		goto ret_no_clear;

	case CMD_COMPLETE_INT:
		break;

	case CMD_ERROR_INT:
		cmd_error_int++;
		break;

	case SUBSYS_POW_ERR_INT:
	case NO_ACKC6_INT:
	case S0ix_MISS_INT:
		pmu_stat_error(status);
		break;

	case WAKE_RECEIVED_INT:
		(void)pmu_get_wake_source();
		break;
	}

	if (status == WAKE_RECEIVED_INT)
		pmu_stat_end();
	else
		pmu_stat_clear();

	wrmsr(MSR_C6OFFLOAD_CTL_REG,
		MSR_C6OFFLOAD_CLEAR_LOW, MSR_C6OFFLOAD_CLEAR_HIGH);

	/* clear the interrupt pending bit */
	pmu_clear_interrupt_pending(PMU_NUM_2);

	s0ix_entered = 0;

	/* S0ix case release it */
	up(&scu_ready_sem);

	status = IRQ_HANDLED;
ret_no_clear:
	return status;
}

void pmu_enable_forward_msi(void)
{
	writel(0, &pmu_reg->pm_msic);
}
EXPORT_SYMBOL(pmu_enable_forward_msi);

unsigned long pmu_get_cstate(unsigned long eax)
{
	unsigned long state = eax;

	/* If we get C6 in between s0ix, it should
	 * be demoted to C4 */
	if ((s0ix_entered) && (eax == C6_HINT)) {
		state = C4_HINT;
		c6_demoted++;
	} else if ((eax == C7_HINT) || (eax == C8_HINT)) {
		/* for S0ix choose C6 */
		state = C6_HINT;
	}

	return state;
}
EXPORT_SYMBOL(pmu_get_cstate);

static inline u32 find_index_in_hash(struct pci_dev *pdev, int *found)
{
	u32 h_index;
	int i;

	/* assuming pdev is not null */
	WARN_ON(pdev == NULL);

	h_index = jhash_1word(((pdev->vendor<<16)|pdev->device),
		 MID_PCI_INDEX_HASH_INITVALUE) & MID_PCI_INDEX_HASH_MASK;

	/* assume not found */
	*found = 0;

	for (i = 0; i < MID_PCI_INDEX_HASH_SIZE; i++) {
		if (likely(pci_dev_hash[h_index].pdev == pdev)) {
			*found = 1;
			break;
		}

		/* assume no deletions, hence there shouldn't be any
		 * gaps ie., NULL's */
		if (unlikely(pci_dev_hash[h_index].pdev == NULL)) {
			/* found NULL, that means we wont have
			 * it in hash */
			break;
		}

		h_index = (h_index+1)%MID_PCI_INDEX_HASH_SIZE;
	}

	/* Assume hash table wont be full */
	WARN_ON(i == MID_PCI_INDEX_HASH_SIZE);

	return h_index;
}

static int get_pci_to_pmu_index(struct pci_dev *pdev)
{
	int pm, type;
	unsigned int base_class;
	unsigned int sub_class;
	u8 ss;
	int index = PMU_FAILED;
	u32 h_index;
	int found;

	h_index = find_index_in_hash(pdev, &found);

	if (found)
		return (int)pci_dev_hash[h_index].index;

	/* if not found, h_index would be where
	 * we can insert this */

	base_class = pdev->class >> 16;
	sub_class  = (pdev->class & SUB_CLASS_MASK) >> 8;
	pm = pci_find_capability(pdev, PCI_CAP_ID_VNDR);

	/* read the logical sub system id & cap if present */
	pci_read_config_byte(pdev, pm + 4, &ss);

	/*FIXME:: pci_table is wrong and put bad ss for HSU0 and HSU1 */
	if (pdev->device == 0x81c || pdev->device == 0x81b)
		ss = (LOG_SS_MASK | PMU_UART2_LSS_41);

	/*FIXME:: HSI does not have an LSS listed in pci_table */
	if (pdev->device == 0x833) {
		dev_warn(&pdev->dev, "HSI pci pm config = 0x%x\n", ss);
		ss = (LOG_SS_MASK | PMU_HSI_LSS_03);
	}

	type = ss & LOG_SS_MASK;
	ss = ss & LOG_ID_MASK;

	/*FIXME:: remove this once IFWI fixes Audio DMA LSS to 9 */
	if (ss == PMU_SC_DMA_LSS_16)
		ss = PMU_AUDIO_DMA_LSS_09;

	if ((base_class == PCI_BASE_CLASS_DISPLAY) && !sub_class)
		index = 1;
	else if ((__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) &&
			(base_class == PCI_BASE_CLASS_MULTIMEDIA) &&
			(sub_class == ISP_SUB_CLASS))
				index = MFLD_ISP_POS;
	else if (type)
		index = pmu1_max_devs + ss;

	if (index != PMU_FAILED) {
		/* insert into hash table */
		pci_dev_hash[h_index].pdev = pdev;

		/* assume index never exceeds 0xff */
		WARN_ON(index > 0xFF);

		pci_dev_hash[h_index].index = (u8)index;

		if (index < pmu1_max_devs) {
			intel_mid_pci_devices[index].ss_idx = 0;
			intel_mid_pci_devices[index].ss_pos = index;
			intel_mid_pci_devices[index].pmu_num = PMU_NUM_1;
		} else if (index >= pmu1_max_devs &&
			   index < (pmu1_max_devs + pmu2_max_devs)) {
			intel_mid_pci_devices[index].ss_idx = ss / ss_per_reg;
			intel_mid_pci_devices[index].ss_pos = ss % ss_per_reg;
			intel_mid_pci_devices[index].pmu_num = PMU_NUM_2;
		} else {
			index = PMU_FAILED;
		}

		WARN_ON(index == PMU_FAILED);
	}

	return index;
}

static void mid_pci_find_info(struct pci_dev *pdev)
{
	int index, pm;
	unsigned int base_class;
	unsigned int sub_class;
	u8 ss, cap;
	int i;
	base_class = pdev->class >> 16;
	sub_class  = (pdev->class & SUB_CLASS_MASK) >> 8;

	pm = pci_find_capability(pdev, PCI_CAP_ID_VNDR);

	/* read the logical sub system id & cap if present */
	pci_read_config_byte(pdev, pm + 4, &ss);
	pci_read_config_byte(pdev, pm + 5, &cap);

	/* get the index for the copying of ss info */
	index = get_pci_to_pmu_index(pdev);

	if (index == PMU_FAILED)
		return;

	/* initialize gfx subsystem info */
	if ((base_class == PCI_BASE_CLASS_DISPLAY) && !sub_class) {
		intel_mid_pci_devices[index].log_id = index;
		intel_mid_pci_devices[index].cap = PM_SUPPORT;
	} else if ((base_class == PCI_BASE_CLASS_MULTIMEDIA) &&
		(sub_class == ISP_SUB_CLASS)) {
		if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
			intel_mid_pci_devices[index].log_id = index;
			intel_mid_pci_devices[index].cap = PM_SUPPORT;
		} else if (ss && cap) {
			intel_mid_pci_devices[index].log_id = ss & LOG_ID_MASK;
			intel_mid_pci_devices[index].cap    = cap;
		}
	} else if (ss && cap) {
		intel_mid_pci_devices[index].log_id = ss & LOG_ID_MASK;
		intel_mid_pci_devices[index].cap    = cap;
	}

	for (i = 0; i < PMU_MAX_LSS_SHARE &&
		intel_mid_pci_devices[index].dev_driver[i]; i++) {
		/* do nothing */
	}

	WARN_ON(i >= PMU_MAX_LSS_SHARE);

	if (i < PMU_MAX_LSS_SHARE) {
		intel_mid_pci_devices[index].dev_driver[i] = pdev;
		intel_mid_pci_devices[index].dev_power_state[i] = PCI_D3hot;
	}
}

static void pmu_enumerate(void)
{
	struct pci_dev *pdev = NULL;
	unsigned int base_class;

	while ((pdev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, pdev)) != NULL) {

		/* find the base class info */
		base_class = pdev->class >> 16;

		if (base_class == PCI_BASE_CLASS_BRIDGE)
			continue;

		mid_pci_find_info(pdev);
	}
}

static void pmu_read_sss(struct pmu_ss_states *pm_ssc)
{
	pm_ssc->pmu2_states[0] = readl(&pmu_reg->pm_sss[0]);

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		pm_ssc->pmu2_states[1] = readl(&pmu_reg->pm_sss[1]);
		pm_ssc->pmu2_states[2] = readl(&pmu_reg->pm_sss[2]);
		pm_ssc->pmu2_states[3] = readl(&pmu_reg->pm_sss[3]);
	} else {
		pm_ssc->pmu2_states[1] = 0;
		pm_ssc->pmu2_states[2] = 0;
		pm_ssc->pmu2_states[3] = 0;
	}
}

static bool check_s0ix_possible(struct pmu_ss_states *pmsss)
{
	if (((pmsss->pmu2_states[0] & S0IX_TARGET_SSS0_MASK) ==
			S0IX_TARGET_SSS0) &&
	    ((pmsss->pmu2_states[1] & S0IX_TARGET_SSS1_MASK) ==
			S0IX_TARGET_SSS1) &&
	    ((pmsss->pmu2_states[2] & S0IX_TARGET_SSS2_MASK) ==
			S0IX_TARGET_SSS2) &&
	    ((pmsss->pmu2_states[3] & S0IX_TARGET_SSS3_MASK) ==
			S0IX_TARGET_SSS3))
		return true;

	return false;
}

static bool check_lpmp3_possible(struct pmu_ss_states *pmsss)
{
	if (((pmsss->pmu2_states[0] & LPMP3_TARGET_SSS0_MASK) ==
			LPMP3_TARGET_SSS0) &&
	    ((pmsss->pmu2_states[1] & LPMP3_TARGET_SSS1_MASK) ==
			LPMP3_TARGET_SSS1) &&
	    ((pmsss->pmu2_states[2] & LPMP3_TARGET_SSS2_MASK) ==
			LPMP3_TARGET_SSS2) &&
	    ((pmsss->pmu2_states[3] & LPMP3_TARGET_SSS3_MASK) ==
			LPMP3_TARGET_SSS3))
		return true;

	return false;
}

static void pmu_set_s0ix_possible(int state)
{
	/* assume S0ix not possible */
	s0i1_possible = lpmp3_possible = s0i3_possible = 0;

	if (state != PCI_D0) {
		struct pmu_ss_states cur_pmsss;

		pmu_read_sss(&cur_pmsss);

		if (likely(check_s0ix_possible(&cur_pmsss)))
			s0i1_possible = s0i3_possible = 1;
		else if (check_lpmp3_possible(&cur_pmsss))
			lpmp3_possible = 1;
	}

#ifdef CONFIG_HAS_WAKELOCK
#ifdef S0I3_POSSIBLE_WAKELOCK
	if (s0i3_possible && wake_lock_active(&pmu_wake_lock))
		wake_unlock(&pmu_wake_lock);
	if (!s0i3_possible && !wake_lock_active(&pmu_wake_lock))
		wake_lock(&pmu_wake_lock);
#endif
#endif
}
/*
 * For all devices in this lss, we check what is the weakest power state
 *
 * Thus we dont power down if another device needs more power
 */

static pci_power_t  pmu_pci_get_weakest_state_for_lss(int lss_index,
				struct pci_dev *pdev, pci_power_t state)
{
	int i;
	pci_power_t weakest = state;

	for (i = 0; i < PMU_MAX_LSS_SHARE; i++) {
		if (intel_mid_pci_devices[lss_index].dev_driver[i] == pdev)
			intel_mid_pci_devices[lss_index].dev_power_state[i] =
								state;

		if (intel_mid_pci_devices[lss_index].dev_driver[i] &&
			(intel_mid_pci_devices[lss_index].dev_power_state[i]
			< weakest)) {
			dev_warn(&pdev->dev, "%s:%s prevented me to suspend...\n",
				dev_name(&intel_mid_pci_devices[lss_index].dev_driver[i]->dev),
				dev_driver_string(&intel_mid_pci_devices[lss_index].dev_driver[i]->dev));

			weakest = intel_mid_pci_devices[lss_index].
						dev_power_state[i];
		}
	}
	return weakest;
}

static int pmu_pci_to_indexes(struct pci_dev *pdev, int *index,
				int *pmu_num, int *ss_idx, int *ss_pos)
{
	int i;

	i = get_pci_to_pmu_index(pdev);
	if (i == PMU_FAILED)
		return PMU_FAILED;

	*index		= i;
	*ss_pos		= intel_mid_pci_devices[i].ss_pos;
	*ss_idx		= intel_mid_pci_devices[i].ss_idx;
	*pmu_num	= intel_mid_pci_devices[i].pmu_num;

	return PMU_SUCCESS;
}

static int wait_for_nc_pmcmd_complete(int verify_mask, int state_type
					, int reg_type)
{
	int pwr_sts;
	int count = 0;
	u32 addr;

	switch (reg_type) {
	case APM_REG_TYPE:
		addr = apm_base + APM_STS;
		break;
	case OSPM_REG_TYPE:
		addr = ospm_base + OSPM_PM_SSS;
		break;
	default:
		return -EINVAL;
	}

	while (true) {
		pwr_sts = inl(addr);
		if (state_type == OSPM_ISLAND_DOWN) {
			if ((pwr_sts & verify_mask) == verify_mask)
				break;
			else
				udelay(10);
		} else if (state_type == OSPM_ISLAND_UP) {
			if (pwr_sts  == verify_mask)
				break;
			else
				udelay(10);
		}
		count++;
		if (WARN_ONCE(count > 500000, "Timed out waiting for P-Unit"))
			return -EBUSY;
	}
	return 0;
}

/**
 * pmu_nc_set_power_state - Callback function is used by all the devices
 * in north complex for a platform  specific device power on/shutdown.
 * Following assumptions are made by this function
 *
 * Every new request starts from scratch with no assumptions
 * on previous/pending request to Punit.
 * Caller is responsible to retry if request fails.
 * Avoids multiple requests to Punit if target state is
 * already in the expected state.
 * spin_locks guarantee serialized access to these registers
 * and avoid concurrent access from 2d/3d, VED, VEC, ISP & IPH.
 *
 */
int pmu_nc_set_power_state(int islands, int state_type, int reg_type)
{
	u32 pwr_cnt = 0;
	u32 pwr_mask = 0;
	unsigned long flags;
	int i, lss, mask;
	int ret = 0;

	spin_lock_irqsave(&nc_ready_lock, flags);

	switch (reg_type) {
	case APM_REG_TYPE:
		pwr_cnt = inl(apm_base + APM_STS);
		break;
	case OSPM_REG_TYPE:
		pwr_cnt = inl(ospm_base + OSPM_PM_SSS);
		break;
	default:
		ret = -EINVAL;
		goto unlock;
	}

	pwr_mask = pwr_cnt;
	for (i = 0; i < OSPM_MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			mask = 0x3 << (2 * i);
			if (state_type == OSPM_ISLAND_DOWN)
				pwr_mask |= mask;
			else if (state_type == OSPM_ISLAND_UP)
				pwr_mask &= ~mask;
		}
	}

	if (pwr_mask != pwr_cnt) {
		switch (reg_type) {
		case APM_REG_TYPE:
			outl(pwr_mask, apm_base + APM_CMD);
			break;
		case OSPM_REG_TYPE:
			outl(pwr_mask, (ospm_base + OSPM_PM_SSC));
			break;
		}

		ret =
		wait_for_nc_pmcmd_complete(pwr_mask, state_type, reg_type);
	}

unlock:
	spin_unlock_irqrestore(&nc_ready_lock, flags);
	return ret;
}
EXPORT_SYMBOL(pmu_nc_set_power_state);

/* poll for maximum of 50ms us for busy bit to clear */
static int pmu_wait_done(void)
{
	int udelays;

	pmu_wait_done_calls++;

	for (udelays = 0; udelays < 500; ++udelays) {
		if (udelays > pmu_wait_done_udelays_max)
			pmu_wait_done_udelays_max = udelays;

		if (pmu_read_busy_status() == 0)
			return 0;

		udelay(100);
		pmu_wait_done_udelays++;
	}

	WARN_ONCE(1, "SCU not done for 50ms");
	return -EBUSY;
}

/**
 * mfld_pmu_pci_set_power_state - Callback function is used by all the PCI devices
 *			for a platform  specific device power on/shutdown.
 *
 */
int __ref mfld_pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state)
{
	int i;
	u32 pm_cmd_val;
	u32 new_value;
	int sub_sys_pos, sub_sys_index;
	int pmu_num;
	struct pmu_ss_states cur_pmssc;
	int status = 0;

	/* while booting just ignore this callback from devices */
	/* If SCU is not okay, nothing to do */
	if (unlikely((dev_init_state == 0) || !scu_comms_okay))
		return status;

	/* Try to acquire the scu_ready_sem, if not
	 * get blocked, until pmu_sc_irq() releases */

if (in_interrupt()) {
	dev_err(&pmu_dev->dev, "mfld_pmu_pci_set_power_state() called from interrupt context!\n");
	return -1;
}

	down_scu_timed(&scu_ready_sem);

	spin_lock(&mrst_pmu_power_state_lock);

	status =
	pmu_pci_to_indexes(pdev, &i, &pmu_num, &sub_sys_index,  &sub_sys_pos);

	if (status)
		goto unlock;

	state = pmu_pci_get_weakest_state_for_lss(i, pdev, state);

	/* store the display status */
	if (i == GFX_LSS_INDEX)
		display_off = (int)(state != PCI_D0);

	/*Update the Camera status per ISP Driver Suspended/Resumed */
	if (i == MFLD_ISP_POS)
		camera_off = (int)(state != PCI_D0);

	if (pmu_num == PMU_NUM_2) {

		/* initialize the current pmssc states */
		memset(&cur_pmssc, 0, sizeof(cur_pmssc));

		status = _pmu2_wait_not_busy();

		if (status)
			goto unlock;

		pmu_read_sss(&cur_pmssc);

		/* Read the pm_cmd val & update the value */
		pm_cmd_val =
			(D0I3_MASK << (sub_sys_pos * BITS_PER_LSS));

		/* First clear the LSS bits */
		new_value = cur_pmssc.pmu2_states[sub_sys_index] &
							(~pm_cmd_val);

		if (state != PCI_D0) {
			pm_cmd_val =
		(pci_2_mfld_state(state) << (sub_sys_pos * BITS_PER_LSS));

			new_value |= pm_cmd_val;
		}

		/* nothing to do, so dont do it... */
		if (new_value == cur_pmssc.pmu2_states[sub_sys_index])
			goto unlock;

		cur_pmssc.pmu2_states[sub_sys_index] = new_value;

		/* set the lss positions that need
		 * to be ignored to D0i0 */
		cur_pmssc.pmu2_states[0] &= ~IGNORE_SSS0;
		cur_pmssc.pmu2_states[1] &= ~IGNORE_SSS1;
		cur_pmssc.pmu2_states[2] &= ~IGNORE_SSS2;
		cur_pmssc.pmu2_states[3] &= ~IGNORE_SSS3;

		/* Issue the pmu command to PMU 2
		 * flag is needed to distinguish between
		 * S0ix vs interactive command in pmu_sc_irq()
		 */
		status = _pmu_issue_command(&cur_pmssc, SET_MODE, PMU_NUM_2);

		if (unlikely(status != PMU_SUCCESS)) {
			dev_dbg(&pmu_dev->dev,
				 "Failed to Issue a PM command to PMU2\n");
			goto unlock;
		}

		/*
		 * Wait for interactive command to complete.
		 * If we dont wait, there is a possibility that
		 * the driver may access the device before its
		 * powered on in SCU.
		 *
		 */
		pmu_wait_done();

		pmu_set_s0ix_possible(state);
	}

unlock:
	spin_unlock(&mrst_pmu_power_state_lock);
	up(&scu_ready_sem);
	return status;
}

static int _pmu_issue_command(struct pmu_ss_states *pm_ssc, int mode,
			 int pmu_num)
{
	union pmu_pm_set_cfg_cmd_t command;
	int sys_state = SYS_STATE_S0I0;
	struct cfg_mode_params cfg_mode_param;
	int status;
	u32 tmp;

	/* pmu_issue_command for PMU1 is an NOP */
	if (pmu_num == PMU_NUM_1)
		return 0;

	if (_pmu_read_status(PMU_NUM_2, PMU_BUSY_STATUS)) {
		dev_dbg(&pmu_dev->dev, "PMU2 is busy, Operation not"
		"permitted\n");
		return PMU_FAILED;
	}

	/* enable interrupts in PMU2 so that interrupts are
	 * propagated when ioc bit for a particular set
	 * command is set
	 */
	/* Enable the hardware interrupt */
	tmp = readl(&pmu_reg->pm_ics);
	tmp |= 0x100;/* Enable interrupts */
	writel(tmp, &pmu_reg->pm_ics);

	switch (mode) {
	case SET_MODE:
		cfg_mode_param.cfg_mode = CM_IMMEDIATE;
		cfg_mode_param.cfg_trigger = NO_TRIG;
		cfg_mode_param.cfg_delay = 0;
		cfg_mode_param.cfg_trig_val = 0;
		cfg_mode_param.cfg_cmbi = 0;
		sys_state = SYS_STATE_S0I0;
		break;
	case SET_AOAC_S0i1:
		cfg_mode_param.cfg_mode = CM_TRIGGER;
		cfg_mode_param.cfg_trigger = C_STATE_TRANS_TRIG;
		cfg_mode_param.cfg_trig_val = ACK_C6_DMI_MSG;
		cfg_mode_param.cfg_delay = 0;
		cfg_mode_param.cfg_cmbi = 0;
		sys_state = SYS_STATE_S0I1;
		break;
	case SET_AOAC_S0i2:
		cfg_mode_param.cfg_mode = CM_TRIGGER;
		cfg_mode_param.cfg_trigger = C_STATE_TRANS_TRIG;
		cfg_mode_param.cfg_trig_val = ACK_C6_DMI_MSG;
		cfg_mode_param.cfg_delay = 0;
		cfg_mode_param.cfg_cmbi = 0;
		sys_state = SYS_STATE_S0I2;
		break;
	case SET_AOAC_S0i3:
		cfg_mode_param.cfg_mode = CM_TRIGGER;
		cfg_mode_param.cfg_trigger = C_STATE_TRANS_TRIG;
		cfg_mode_param.cfg_trig_val = ACK_C6_DMI_MSG;
		cfg_mode_param.cfg_delay = 0;
		cfg_mode_param.cfg_cmbi = 0;
		sys_state = SYS_STATE_S0I3;
		break;
	case SET_LPAUDIO:
		cfg_mode_param.cfg_mode = CM_TRIGGER;
		cfg_mode_param.cfg_trigger = LPM_EVENT_TRIG;
		cfg_mode_param.cfg_trig_val = PME_ID_USB;
		cfg_mode_param.cfg_delay = 0;
		cfg_mode_param.cfg_cmbi = 1;
		sys_state = SYS_STATE_S0I3;
		break;
	default:
		WARN_ON(1);
		status = PMU_FAILED;
		goto ret;
	}

	/* Configure the sub systems for pmu2 */
	pmu_write_subsys_config(pm_ssc);

	/* Configre the parameters required for config mode */
	status =
		pmu_set_cfg_mode_params(&cfg_mode_param, &command, PMU_NUM_2);
	if (status != PMU_SUCCESS)
		goto ret;

	/* Send the set config command for pmu1 its configured
	 *  for mode CM_IMMEDIATE & hence with No Trigger
	 */
	status =
	pmu_send_set_config_command(&command, sys_state, PMU_NUM_2);

ret:
	return status;
}

#ifdef CONFIG_DEBUG_FS
static int pmu_nc_get_power_state(int island, int reg_type)
{
	u32 pwr_sts;
	unsigned long flags;
	int i, lss;
	int ret = -EINVAL;

	spin_lock_irqsave(&nc_ready_lock, flags);

	switch (reg_type) {
	case APM_REG_TYPE:
		pwr_sts = inl(apm_base + APM_STS);
		break;
	case OSPM_REG_TYPE:
		pwr_sts = inl(ospm_base + OSPM_PM_SSS);
		break;
	default:
		pr_err("%s: invalid argument 'island': %d.\n",
				 __func__, island);
		goto unlock;
	}

	for (i = 0; i < OSPM_MAX_POWER_ISLANDS; i++) {
		lss = island & (0x1 << i);
		if (lss) {
			ret = (pwr_sts >> (2 * i)) & 0x3;
			break;
		}
	}

unlock:
	spin_unlock_irqrestore(&nc_ready_lock, flags);
	return ret;
}

#define DEV_GFX		2
#define FUNC_GFX	0
#define ISLANDS_GFX	8
#define DEV_ISP		3
#define FUNC_ISP	0
#define ISLANDS_ISP	2
#define NC_DEVS		2

struct island {
	int type;
	int index;
	char *name;
};

static struct island display_islands[] = {
	{APM_REG_TYPE, APM_GRAPHICS_ISLAND, "GFX"},
	{APM_REG_TYPE, APM_VIDEO_DEC_ISLAND, "Video Decoder"},
	{APM_REG_TYPE, APM_VIDEO_ENC_ISLAND, "Video Encoder"},
	{APM_REG_TYPE, APM_GL3_CACHE_ISLAND, "GL3 Cache"},
	{OSPM_REG_TYPE, OSPM_DISPLAY_A_ISLAND, "Display A"},
	{OSPM_REG_TYPE, OSPM_DISPLAY_B_ISLAND, "Display B"},
	{OSPM_REG_TYPE, OSPM_DISPLAY_C_ISLAND, "Display C"},
	{OSPM_REG_TYPE, OSPM_MIPI_ISLAND, "MIPI-DSI"}
};

static struct island camera_islands[] = {
	{APM_REG_TYPE, APM_ISP_ISLAND, "ISP"},
	{APM_REG_TYPE, APM_IPH_ISLAND, "Iunit PHY"}
};

static void nc_device_state_show(struct seq_file *s, struct pci_dev *pdev)
{
	int off, i, islands_num, state;
	struct island *islands;
	char *dstates[] = {"D0", "D0i1", "D0i2", "D0i3"};

	if (PCI_SLOT(pdev->devfn) == DEV_GFX &&
			PCI_FUNC(pdev->devfn) == FUNC_GFX) {
		off = display_off;
		islands_num = ISLANDS_GFX;
		islands = &display_islands[0];
	} else if (PCI_SLOT(pdev->devfn) == DEV_ISP &&
			PCI_FUNC(pdev->devfn) == FUNC_ISP) {
		off = camera_off;
		islands_num = ISLANDS_ISP;
		islands = &camera_islands[0];
	} else {
		return;
	}

	seq_printf(s, "pci %04x %04X %s %20s: %41s %s\n",
		pdev->vendor, pdev->device, dev_name(&pdev->dev),
		dev_driver_string(&pdev->dev),
		"", off ? "" : "blocking s0ix");
	for (i = 0; i < islands_num; i++) {
		state = pmu_nc_get_power_state(islands[i].index,
				islands[i].type);
		seq_printf(s, "%52s %15s %17s %s\n",
				 "|------->", islands[i].name, "",
				(state >= 0) ? dstates[state] : "ERR");
	}
}

static int pmu_devices_state_show(struct seq_file *s, void *unused)
{
	struct pci_dev *pdev = NULL;
	int index, i, pmu_num, ss_idx, ss_pos;
	unsigned int base_class;
	u32 target_mask, mask, val, needed;
	struct pmu_ss_states cur_pmsss;
	char *dstates[] = {"D0", "D0i1", "D0i2", "D0i3"};

	/* Acquire the scu_ready_sem */
	down_scu_timed(&scu_ready_sem);

	if (_pmu2_wait_not_busy())
		goto unlock;

	pmu_read_sss(&cur_pmsss);

	seq_printf(s, "TARGET_CFG: ");
	seq_printf(s, "SSS0:%08X ", S0IX_TARGET_SSS0_MASK);
	seq_printf(s, "SSS1:%08X ", S0IX_TARGET_SSS1_MASK);
	seq_printf(s, "SSS2:%08X ", S0IX_TARGET_SSS2_MASK);
	seq_printf(s, "SSS3:%08X ", S0IX_TARGET_SSS3_MASK);

	seq_printf(s, "\n");
	seq_printf(s, "CONDITION FOR S0I3: ");
	seq_printf(s, "SSS0:%08X ", S0IX_TARGET_SSS0);
	seq_printf(s, "SSS0:%08X ", S0IX_TARGET_SSS1);
	seq_printf(s, "SSS0:%08X ", S0IX_TARGET_SSS2);
	seq_printf(s, "SSS0:%08X ", S0IX_TARGET_SSS3);

	seq_printf(s, "\n");
	seq_printf(s, "SSS: ");

	for (i = 0; i < 4; i++)
		seq_printf(s, "%08lX ", cur_pmsss.pmu2_states[i]);

	if (!display_off)
		seq_printf(s, "display not suspended: blocking s0ix\n");
	else if (!camera_off)
		seq_printf(s, "camera not suspended: blocking s0ix\n");
	else if (s0i3_possible)
		seq_printf(s, "can enter s0i1 or s0i3\n");
	else if (lpmp3_possible)
		seq_printf(s, "can enter lpmp3\n");
	else
		seq_printf(s, "blocking s0ix\n");

	seq_printf(s, "cmd_error_int count: %d\n", cmd_error_int);
	seq_printf(s, "c6_demotion count: %d\n", c6_demoted);

	seq_printf(s,
	"\tcount\tsybsys_pow\ts0ix_miss\tno_ack_c6\ttime (secs)\tlast_entry");
	seq_printf(s, "\tfirst_entry\tresidency(%%)\n");

	pmu_stat_seq_printf(s, SYS_STATE_S0I1, "s0i1");
	pmu_stat_seq_printf(s, SYS_STATE_S0I2, "lpmp3");
	pmu_stat_seq_printf(s, SYS_STATE_S0I3, "s0i3");
	pmu_stat_seq_printf(s, SYS_STATE_S3, "s3");

	while ((pdev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, pdev)) != NULL) {

		/* find the base class info */
		base_class = pdev->class >> 16;

		if (base_class == PCI_BASE_CLASS_BRIDGE)
			continue;

		if (pmu_pci_to_indexes(pdev, &index, &pmu_num, &ss_idx,
								  &ss_pos))
			continue;

		if (pmu_num == PMU_NUM_1) {
			nc_device_state_show(s, pdev);
			continue;
		}

		mask	= (D0I3_MASK << (ss_pos * BITS_PER_LSS));
		val	= (cur_pmsss.pmu2_states[ss_idx] & mask) >>
						(ss_pos * BITS_PER_LSS);
		switch (ss_idx) {
		case 0:
			target_mask = S0IX_TARGET_SSS0_MASK;
			break;
		case 1:
			target_mask = S0IX_TARGET_SSS1_MASK;
			break;
		case 2:
			target_mask = S0IX_TARGET_SSS2_MASK;
			break;
		case 3:
			target_mask = S0IX_TARGET_SSS3_MASK;
			break;
		default:
			target_mask = 0;
			break;
		}
		needed = ((target_mask & mask) != 0);

		seq_printf(s, "pci %04x %04X %s %20s: lss:%02d reg:%d"
			"mask:%08X wk:% 8d %s  %s\n",
			pdev->vendor, pdev->device, dev_name(&pdev->dev),
			dev_driver_string(&pdev->dev),
			index-pmu1_max_devs, ss_idx, mask, num_wakes[index],
			dstates[val&3],
			(needed && !val) ? "blocking s0ix" : "");

	}
	seq_printf(s, "pmu_wait_done_calls           %8d\n",
			pmu_wait_done_calls);
	seq_printf(s, "pmu_wait_done_udelays         %8d\n",
			pmu_wait_done_udelays);
	seq_printf(s, "pmu_wait_done_udelays_max     %8d\n",
			pmu_wait_done_udelays_max);


unlock:
	up(&scu_ready_sem);
	return 0;
}

static int devices_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmu_devices_state_show, NULL);
}

static ssize_t devices_state_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32];
	int buf_size = min(count, sizeof(buf)-1);

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	/* Acquire the scu_ready_sem */
	down_scu_timed(&scu_ready_sem);

	buf[buf_size] = 0;

	if (((strlen("clear")+1) == buf_size) &&
		!strncmp(buf, "clear", strlen("clear"))) {
		memset(pmu_stats, 0,
			(sizeof(struct mid_pmu_stats)*SYS_STATE_S5));
		memset(num_wakes, 0, sizeof(int)*MAX_DEVICES);
		cmd_error_int = c6_demoted = 0;
		pmu_current_state = SYS_STATE_S0I0;
		pmu_init_time =
			cpu_clock(raw_smp_processor_id());
	}

	up(&scu_ready_sem);

	return buf_size;
}

static const struct file_operations devices_state_operations = {
	.open		= devices_state_open,
	.read		= seq_read,
	.write		= devices_state_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct lss_definition {
	char *lss_name;
	char *block;
	char *subsystem;
} medfield_lsses[] = {
	{"Lss00", "Storage", "SDIO0 (HC2)"},
	{"Lss01", "Storage", "eMMC0 (HC0a)"},
	{"NA", "Storage", "ND_CTL (Note 5)"},
	{"Lss03", "H S I", "H S I DMA"},
	{"Lss04", "Security", "RNG"},
	{"Lss05", "Storage", "eMMC1 (HC0b)"},
	{"Lss06", "USB", "USB OTG (ULPI)"},
	{"NA", "USB", "USB HSIC (Host) (Note 5)"},
	{"Lss08", "Audio", ""},
	{"Lss09", "Audio", ""},
	{"Lss10", "SRAM", " SRAM CTL+SRAM_16KB"},
	{"Lss11", "SRAM", " SRAM CTL+SRAM_16KB"},
	{"Lss12", "SRAM", "SRAM BANK (16KB+3x32KBKB)"},
	{"Lss13", "SRAM", "SRAM BANK(4x32KB)"},
	{"Lss14", "SDIO COMMS", "SDIO2 (HC1b)"},
	{"Lss15", "PTI, DAFCA", " DFX Blocks"},
	{"Lss16", "SC", " DMA"},
	{"NA", "SC", "SPI0/MSIC"},
	{"Lss18", "GP", "SPI1"},
	{"Lss19", "GP", " SPI2"},
	{"Lss20", "GP", " I2C0"},
	{"Lss21", "GP", " I2C1"},
	{"NA", "Fabrics", " Main Fabric"},
	{"NA", "Fabrics", " Secondary Fabric"},
	{"NA", "SC", "SC Fabric"},
	{"Lss25", "Audio", " I-RAM BANK1 (32 + 256KB)"},
	{"NA", "SCU", " ROM BANK1 (18KB+18KB+18KB)"},
	{"Lss27", "GP", "I2C2"},
	{"NA", "SSC", "SSC (serial bus controller to FLIS)"},
	{"Lss29", "Security", "Chaabi AON Registers"},
	{"Lss30", "SDIO COMMS", "SDIO1 (HC1a)"},
	{"NA", "SCU", "I-RAM BANK0 (32KB)"},
	{"NA", "SCU", "I-RAM BANK1 (32KB)"},
	{"Lss33", "GP", "I2C3 (HDMI)"},
	{"Lss34", "GP", "I2C4"},
	{"Lss35", "GP", "I2C5"},
	{"Lss36", "GP", "SSP (SPI3)"},
	{"Lss37", "GP", "GPIO1"},
	{"NA", "GP", "GP Fabric"},
	{"Lss39", "SC", "GPIO0"},
	{"Lss40", "SC", "KBD"},
	{"Lss41", "SC", "UART2:0"},
	{"NA", "NA", "NA"},
	{"NA", "NA", "NA"},
	{"Lss44", "Security", " Security TAPC"},
	{"NA", "MISC", "AON Timers"},
	{"NA", "PLL", "LFHPLL and Spread Spectrum"},
	{"NA", "PLL", "USB PLL"},
	{"Lss48", "Audio", " SSP2 (I2S2)"},
	{"NA", "Audio", "SLIMBUS CTL 1 (note 5)"},
	{"NA", "Audio", "SLIMBUS CTL 2 (note 5)"},
	{"Lss51", "Audio", "SSP0"},
	{"Lss52", "Audio", "SSP1"},
	{"NA", "Bridge", "IOSF to OCP Bridge"},
	{"Lss54", "GP", "DMA"},
	{"NA", "SC", "SVID (Serial Voltage ID)"},
	{"NA", "SOC Fuse", "SoC Fuse Block (note 3)"},
	{"NA", "NA", "NA"},
};

static int medfield_lsses_num =
			sizeof(medfield_lsses)/sizeof(medfield_lsses[0]);

static char *lss_device_status[4] = { "D0i0", "D0i1", "D0i2", "D0i3" };

static int show_pmu_lss_status(struct seq_file *s, void *unused)
{
	int sss_reg_index;
	int offset;
	int lss;
	unsigned long status;
	unsigned long sub_status;
	unsigned long lss_status[4];
	struct lss_definition *entry;

	/* Acquire the scu_ready_sem */
	down_scu_timed(&scu_ready_sem);

	lss_status[0] = readl(&pmu_reg->pm_sss[0]);
	lss_status[1] = readl(&pmu_reg->pm_sss[1]);
	lss_status[2] = readl(&pmu_reg->pm_sss[2]);
	lss_status[3] = readl(&pmu_reg->pm_sss[3]);
	lss = 0;
	seq_printf(s, "%5s\t%12s %35s %4s\n",
			"lss", "block", "subsystem", "state");
	seq_printf(s, "==========================================================\n");
	for (sss_reg_index = 0; sss_reg_index < 4; sss_reg_index++) {
		status = lss_status[sss_reg_index];
		for (offset = 0; offset < sizeof(unsigned long) * 8 / 2;
								offset++) {
			sub_status = status & 3;
			if (lss >= medfield_lsses_num)
				entry = &medfield_lsses[medfield_lsses_num - 1];
			else
				entry = &medfield_lsses[lss];
			seq_printf(s, "%5s\t%12s %35s %4s\n",
					entry->lss_name, entry->block,
					entry->subsystem,
					lss_device_status[sub_status]);
			status >>= 2;
			lss++;
		}
	}

	up(&scu_ready_sem);

	return 0;
}

static int pmu_sss_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_pmu_lss_status, NULL);
}

static const struct file_operations pmu_sss_state_operations = {
	.open		= pmu_sss_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#endif	/* DEBUG_FS */

/* Reads the status of each driver and updates the LSS values.
 * To be called with scu_ready_sem mutex held, and pmu_config
 * initialized with '0's
 */
static void update_all_lss_states(struct pmu_ss_states *pmu_config)
{
	int i;
	u32 PCIALLDEV_CFG[4] = {0, 0, 0, 0};

	for (i = 0; i < MAX_DEVICES; i++) {
		int pmu_num =
			intel_mid_pci_devices[i].pmu_num;
		struct pci_dev *pdev =
			intel_mid_pci_devices[i].dev_driver[0];

		if ((pmu_num == PMU_NUM_2) && pdev) {
			int ss_idx, ss_pos;
			pci_power_t state;

			ss_idx = intel_mid_pci_devices[i].ss_idx;
			ss_pos = intel_mid_pci_devices[i].ss_pos;
			state = pdev->current_state;
			/* the case of device not probed yet: Force D0i3 */
			if (state == PCI_UNKNOWN)
				state = pmu_pci_choose_state(pdev);

			/* By default its set to '0' hence
			 * no need to update PCI_D0 state
			 */
			state = pmu_pci_get_weakest_state_for_lss(i, pdev,
								  state);

			pmu_config->pmu2_states[ss_idx] |=
			 (pci_2_mfld_state(state) << (ss_pos * BITS_PER_LSS));

			PCIALLDEV_CFG[ss_idx] |=
				(D0I3_MASK << (ss_pos * BITS_PER_LSS));
		}
	}

	/* We shutdown devices that are in the target config, and that are
	   not in the pci table, some devices are indeed not advertised in pci
	   table for certain firmwares. This is the case for HSI firmwares,
	   SPI3 device is not advertised, and would then prevent s0i3. */
	pmu_config->pmu2_states[0] |= S0IX_TARGET_SSS0_MASK & ~PCIALLDEV_CFG[0];
	pmu_config->pmu2_states[0] &= ~IGNORE_SSS0;
	pmu_config->pmu2_states[1] |= S0IX_TARGET_SSS1_MASK & ~PCIALLDEV_CFG[1];
	pmu_config->pmu2_states[1] &= ~IGNORE_SSS1;
	pmu_config->pmu2_states[2] |= S0IX_TARGET_SSS2_MASK & ~PCIALLDEV_CFG[2];
	pmu_config->pmu2_states[2] &= ~IGNORE_SSS2;
	pmu_config->pmu2_states[3] |= S0IX_TARGET_SSS3_MASK & ~PCIALLDEV_CFG[3];
	pmu_config->pmu2_states[3] &= ~IGNORE_SSS3;
}

static pci_power_t _pmu_choose_state(int device_lss)
{
	pci_power_t state;

	switch (device_lss) {
	case PMU_SECURITY_LSS_04:
		state = PCI_D2;
		break;

	case PMU_USB_OTG_LSS_06:
	case PMU_USB_HSIC_LSS_07:
	case PMU_UART2_LSS_41:
		state = PCI_D1;
		break;

	default:
		state = PCI_D3hot;
		break;
	}

	return state;
}

static int pmu_init(void)
{
	int status;
	struct pmu_ss_states pmu_config;
	struct pmu_suspend_config *ss_config;

	dev_dbg(&pmu_dev->dev, "PMU Driver loaded\n");
	spin_lock_init(&nc_ready_lock);

	/* enumerate the PCI configuration space */
	pmu_enumerate();

#ifdef CONFIG_DEBUG_FS
	/* /sys/kernel/debug/mid_pmu_states */
	(void) debugfs_create_file("mid_pmu_states", S_IFREG | S_IRUGO,
				NULL, NULL, &devices_state_operations);

	/* /sys/kernel/debug/pmu_sss_states */
	(void) debugfs_create_file("pmu_sss_states", S_IFREG | S_IRUGO,
				NULL, NULL, &pmu_sss_state_operations);
#endif

	/* initialize the state variables here */
	intel_mid_pmu_base.disable_cpu1 = false;

	intel_mid_pmu_base.ss_config =
	kzalloc(sizeof(struct pmu_suspend_config), GFP_KERNEL);

	if (intel_mid_pmu_base.ss_config == NULL) {
		dev_dbg(&pmu_dev->dev, "Allocation of memory"
		"for ss_config has failed\n");
		status = PMU_FAILED;
		goto out_err1;
	}

	memset(&pmu_config, 0, sizeof(pmu_config));

	intel_mid_pmu_base.ss_config->ss_state = pmu_config;

	/* initialize for the autonomous S0i3 */
	ss_config = intel_mid_pmu_base.ss_config;

	/* setup the wake capable devices */
	intel_mid_pmu_base.ss_config->wake_state.wake_enable[0] =
							WAKE_ENABLE_0;
	intel_mid_pmu_base.ss_config->wake_state.wake_enable[1] =
							WAKE_ENABLE_1;

	/* Acquire the scu_ready_sem */
	down_scu_timed(&scu_ready_sem);

	/* Now we have initialized the driver
	 * Allow drivers to get blocked in
	 * pmu_pci_set_power_state(), until we finish
	 * first interactive command.
	 */
	dev_init_state = 1;

	/* get the current status of each of the driver
	 * and update it in SCU
	 */
	update_all_lss_states(&pmu_config);

	/* send a interactive command to fw */
	status = _pmu_issue_command(&pmu_config, SET_MODE, PMU_NUM_2);
	if (status != PMU_SUCCESS) {
		dev_dbg(&pmu_dev->dev,\
		 "Failure from pmu mode change to interactive."
		" = %d\n", status);
		status = PMU_FAILED;
		up(&scu_ready_sem);
		goto out_err1;
	}

	/*
	 * Wait for interactive command to complete.
	 * If we dont wait, there is a possibility that
	 * the driver may access the device before its
	 * powered on in SCU.
	 *
	 */
	pmu_wait_done();

	/* In cases were gfx is not enabled
	 * this will enable s0ix immediately
	 */
	pmu_set_s0ix_possible(PCI_D3hot);

	up(&scu_ready_sem);

out_err1:
	return status;
}

/**
 * mid_pmu_probe - This is the function where most of the PMU driver
 *		   initialization happens.
 */
static int __devinit mid_pmu_probe(struct pci_dev *dev,
				   const struct pci_device_id *pci_id)
{
	int ret;
	struct mrst_pmu_reg __iomem *pmu;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&pmu_wake_lock, WAKE_LOCK_SUSPEND, "mid_pmu");
#endif

	/* Init the device */
	ret = pci_enable_device(dev);
	if (ret) {
		pr_err("Mid PM device cant be enabled\n");
		goto out_err0;
	}

	/* store the dev */
	pmu_dev = dev;

	ret = pci_request_regions(dev, PMU_DRV_NAME);
	if (ret < 0) {
		pr_err("pci request region has failed\n");
		goto out_err1;
	}

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		pmu1_max_devs = PMU1_MAX_PENWELL_DEVS;
		pmu2_max_devs = PMU2_MAX_PENWELL_DEVS;
		ss_per_reg = 16;
	} else {
		pmu1_max_devs = PMU1_MAX_MRST_DEVS;
		pmu2_max_devs = PMU2_MAX_MRST_DEVS;
		ss_per_reg = 8;
	}

	/* Map the NC PM registers */
	apm_base =  MDFLD_MSG_READ32(OSPM_PUNIT_PORT, OSPM_APMBA) & 0xffff;
	ospm_base =  MDFLD_MSG_READ32(OSPM_PUNIT_PORT, OSPM_OSPMBA) & 0xffff;

	/* Map the memory of pmu1 PMU reg base */
	pmu = pci_iomap(dev, 0, 0);
	if (pmu == NULL) {
		dev_dbg(&pmu_dev->dev, "Unable to map the PMU2 address"
			 "space\n");
		ret = PMU_FAILED;
		goto out_err2;
	}

	pmu_reg = pmu;

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		/* Map the memory of offload_reg */
		base_addr.offload_reg = ioremap_nocache(0xffd01ffc, 4);
		if (base_addr.offload_reg == NULL) {
			dev_dbg(&pmu_dev->dev,
			"Unable to map the offload_reg address space\n");
			ret = PMU_FAILED;
			goto out_err3;
		}
	}

	if (request_irq(dev->irq, pmu_sc_irq, IRQF_NO_SUSPEND, PMU_DRV_NAME,
			NULL)) {
		dev_dbg(&pmu_dev->dev, "Registering isr has failed\n");
		ret = PMU_FAILED;
		goto out_err3;
	}

	/* call pmu init() for initialization of pmu interface */
	ret = pmu_init();
	if (ret != PMU_SUCCESS) {
		dev_dbg(&pmu_dev->dev, "PMU initialization has failed\n");
		goto out_err4;
	}

	spin_lock_init(&mrst_pmu_power_state_lock);

	pmu_init_time =
		cpu_clock(raw_smp_processor_id());

	return 0;

out_err4:
	free_irq(dev->irq, &pmu_sc_irq);
out_err3:
	pci_iounmap(dev, base_addr.pmu2_base);
	base_addr.pmu2_base = NULL;
out_err2:
	base_addr.pmu1_base = NULL;
out_err1:
	pci_release_region(dev, 0);
	pci_disable_device(dev);
out_err0:
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&pmu_wake_lock);
#endif
	return ret;
}

static void __devexit mid_pmu_remove(struct pci_dev *dev)
{
	dev_dbg(&pmu_dev->dev, "Mid PM mid_pmu_remove called\n");

	/* Freeing up the irq */
	free_irq(dev->irq, &pmu_sc_irq);

	/* Freeing up memory allocated for PMU1 & PMU2 */
	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		iounmap(base_addr.offload_reg);
		base_addr.offload_reg = NULL;
	}
	pci_iounmap(dev, pmu_reg);
	base_addr.pmu1_base = NULL;
	base_addr.pmu2_base = NULL;

	/* disable the current PCI device */
	pci_release_region(dev, 0);
	pci_disable_device(dev);
}

static struct pci_driver driver = {
	.name = PMU_DRV_NAME,
	.id_table = mid_pm_ids,
	.probe = mid_pmu_probe,
	.remove = __devexit_p(mid_pmu_remove),
};

/* return platform specific deepest states that the device can enter */
pci_power_t pmu_pci_choose_state(struct pci_dev *pdev)
{
	int i;
	int sub_sys_pos, sub_sys_index;
	int status;
	int device_lss;
	int pmu_num;

	pci_power_t state = PCI_D3hot;

	if (dev_init_state) {
		status =
		pmu_pci_to_indexes(pdev, &i, &pmu_num,
					&sub_sys_index,  &sub_sys_pos);

		if ((status == PMU_SUCCESS) &&
			(pmu_num == PMU_NUM_2)) {

			device_lss = (sub_sys_index * ss_per_reg) + sub_sys_pos;

			state = _pmu_choose_state(device_lss);
		}
	}

	return state;
}
EXPORT_SYMBOL(pmu_pci_choose_state);

bool pmu_pci_power_manageable(struct pci_dev *dev)
{
	return true;
}
EXPORT_SYMBOL(pmu_pci_power_manageable);

/* At this point of time we expect all devices to be
 * wake capable will be modified in future
 */
bool pmu_pci_can_wakeup(struct pci_dev *dev)
{
	return true;
}
EXPORT_SYMBOL(pmu_pci_can_wakeup);

int pmu_pci_sleep_wake(struct pci_dev *dev, bool enable)
{
	return 0;
}
EXPORT_SYMBOL(pmu_pci_sleep_wake);

static int mfld_s3_enter(void)
{
	u32 ssw_val = 0;
	int num_retry = 15000;
	int status = 0;
	struct pmu_ss_states cur_pmsss;

	/* check if we can acquire scu_ready_sem
	 * if we are not able to then do a c6 */
	if (down_trylock(&scu_ready_sem)) {
		status = -EINVAL;
		goto ret;
	}

	/* If PMU is busy, we'll retry on next C6 */
	if (unlikely(_pmu_read_status(PMU_NUM_2, PMU_BUSY_STATUS))) {
		up(&scu_ready_sem);
		status = -EINVAL;
		goto ret;
	}

	/* setup the wake capable devices */
	writel(~IGNORE_S3_WKC0, &pmu_reg->pm_wkc[0]);
	writel(~IGNORE_S3_WKC1, &pmu_reg->pm_wkc[1]);

	/* Re-program the sub systems state on wakeup as the current SSS*/
	pmu_read_sss(&cur_pmsss);

	writel(cur_pmsss.pmu2_states[0], &pmu_reg->pm_wssc[0]);
	writel(cur_pmsss.pmu2_states[1], &pmu_reg->pm_wssc[1]);
	writel(cur_pmsss.pmu2_states[2], &pmu_reg->pm_wssc[2]);
	writel(cur_pmsss.pmu2_states[3], &pmu_reg->pm_wssc[3]);

	/* program pm ssc registers */
	writel(S0I3_SSS0, &pmu_reg->pm_ssc[0]);
	writel(S0I3_SSS1, &pmu_reg->pm_ssc[1]);
	writel(S0I3_SSS2, &pmu_reg->pm_ssc[2]);
	writel(S0I3_SSS3, &pmu_reg->pm_ssc[3]);

	/* issue a command to SCU */
	writel(S0I3_VALUE, &pmu_reg->pm_cmd);

	/* If s0i3 command is issued
	 * then wait for c6 sram offload
	 * to complete */
	do {
		ssw_val = readl(base_addr.offload_reg);

		if ((ssw_val & C6OFFLOAD_BIT_MASK) ==  C6OFFLOAD_BIT)
			break;

	} while (num_retry--);

	if (unlikely((ssw_val & C6OFFLOAD_BIT_MASK) !=  C6OFFLOAD_BIT)) {
		status = -EINVAL;
		pmu_stat_clear();
		WARN(1, "mid_pmu: error cpu offload bit not set.\n");
	} else {
		unsigned long ecx = 1; /* break on interrupt flag */
		unsigned long eax = C6_HINT;
		u32 temp = 0;
		pmu_stat_start(SYS_STATE_S3);

		/* issue c6 offload */
		wrmsr(MSR_C6OFFLOAD_CTL_REG,
			MSR_C6OFFLOAD_SET_LOW, MSR_C6OFFLOAD_SET_HIGH);

		__monitor((void *) &temp, 0, 0);
		smp_mb();

		s0ix_entered = 1;
		__mwait(eax, ecx);
		pmu_enable_forward_msi();
	}

ret:
	return status;
}

static int mid_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static int mid_prepare(void)
{
	return 0;
}

static int mid_prepare_late(void)
{
	return 0;
}

static int mid_suspend(suspend_state_t state)
{
	int ret;

	if (state != PM_SUSPEND_MEM)
		ret = -EINVAL;

	trace_printk("s3_entry\n");
	ret = mfld_s3_enter();
	trace_printk("s3_exit %d\n", ret);
	if (ret != 0)
		dev_dbg(&pmu_dev->dev, "Failed to enter S3 status: %d\n", ret);

	return ret;
}

static void mid_end(void)
{
#ifdef CONFIG_HAS_WAKELOCK
	/* Prime the wakelock system to re-suspend after giving other
	 * threads a chance to wake up and acquire wake lock
	 * this avoids to put wake lock in other things like pwrbutton
	 */
	long timeout = HZ;
	wake_lock_timeout(&pmu_wake_lock, timeout);
#endif
}

static struct platform_suspend_ops mid_suspend_ops = {
	.valid = mid_valid,
	.prepare = mid_prepare,
	.prepare_late = mid_prepare_late,
	.enter = mid_suspend,
	.end = mid_end,
};

/**
 * mid_pci_register_init - register the PMU driver as PCI device
 */
static int __init mid_pci_register_init(void)
{
	int ret;

	/* registering PCI device */
	ret = pci_register_driver(&driver);
	suspend_set_ops(&mid_suspend_ops);

	return ret;
}
fs_initcall(mid_pci_register_init);

static void __exit mid_pci_cleanup(void)
{
	suspend_set_ops(NULL);
	/* registering PCI device */
	pci_unregister_driver(&driver);
}
module_exit(mid_pci_cleanup);
