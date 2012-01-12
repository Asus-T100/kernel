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
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/jhash.h>
#include <linux/suspend.h>
#include <linux/wakelock.h>
#include <asm/mrst.h>
#include <asm/apic.h>

#include <linux/intel_mid_pm.h>
#include "pmu.h"

static bool pmu_initialized;

/* mid_pmu context structure */
static struct mid_pmu_dev *mid_pmu_cxt;

static DEFINE_MUTEX(pci_root_lock);

/* Accessor function for pci_devs start */
static inline struct pci_dev *get_mid_pci_drv(int lss_index, int i)
{
	return mid_pmu_cxt->pci_devs[lss_index].drv[i];
}

static inline pci_power_t get_mid_pci_power_state(int lss_index, int i)
{
	return mid_pmu_cxt->pci_devs[lss_index].power_state[i];
}

static inline u8 get_mid_pci_ss_idx(int lss_index)
{
	return mid_pmu_cxt->pci_devs[lss_index].ss_idx & SS_IDX_MASK;
}

static inline u8 get_mid_pci_ss_pos(int lss_index)
{
	return mid_pmu_cxt->pci_devs[lss_index].ss_pos & SS_POS_MASK;
}

static inline u8 get_mid_pci_pmu_num(int lss_index)
{
	return mid_pmu_cxt->pci_devs[lss_index].pmu_num;
}

static inline void set_mid_pci_drv(int lss_index,
					int i, struct pci_dev *pdev)
{
	mid_pmu_cxt->pci_devs[lss_index].drv[i] = pdev;
}

static inline void set_mid_pci_power_state(int lss_index,
					int i, pci_power_t state)
{
	mid_pmu_cxt->pci_devs[lss_index].power_state[i] = state;
}

static inline void set_mid_pci_ss_idx(int lss_index, u8 ss_idx)
{
	mid_pmu_cxt->pci_devs[lss_index].ss_idx = ss_idx;
}

static inline void set_mid_pci_ss_pos(int lss_index, u8 ss_pos)
{
	mid_pmu_cxt->pci_devs[lss_index].ss_pos = ss_pos;
}

static inline void set_mid_pci_pmu_num(int lss_index, u8 pmu_num)
{
	mid_pmu_cxt->pci_devs[lss_index].pmu_num = pmu_num;
}

static inline void set_mid_pci_log_id(int lss_index, u32 log_id)
{
	mid_pmu_cxt->pci_devs[lss_index].log_id = log_id;
}

static inline void set_mid_pci_cap(int lss_index, u32 cap)
{
	mid_pmu_cxt->pci_devs[lss_index].cap = cap;
}

static inline u32 get_d0ix_stat(int lss_index, int state)
{
	return mid_pmu_cxt->d0ix_stat[lss_index][state];
}

static inline void inc_d0ix_stat(int lss_index, int state)
{
	mid_pmu_cxt->d0ix_stat[lss_index][state]++;
}

static inline void clear_d0ix_stats(void)
{
	memset(mid_pmu_cxt->d0ix_stat, 0, sizeof(mid_pmu_cxt->d0ix_stat));
}
/* Accessor functions for pci_devs end */

/*
 * APIs to communicate with pci root,
 * Returns zero on sucess.
 */
int mfld_msg_read32(u32 cmd, u32 *data)
{
	struct pci_dev *pci_root;
	int ret;

	mutex_lock(&pci_root_lock);
	pci_root = pci_get_bus_and_slot(0, 0);

	ret = pci_write_config_dword(pci_root, PCI_CMD_REG, cmd);
	if (ret)
		goto unlock;
	/*
	 * since we go to unlock with/without error, not
	 * checking for it here
	 */
	pci_read_config_dword(pci_root, PCI_DATA_REG, data);

unlock:
	pci_dev_put(pci_root);
	mutex_unlock(&pci_root_lock);

	return ret;
}
EXPORT_SYMBOL(mfld_msg_read32);

/*Returns Zero on sucess*/
int mfld_msg_write32(u32 cmd, u32 data)
{
	struct pci_dev *pci_root;
	int ret;

	mutex_lock(&pci_root_lock);
	pci_root = pci_get_bus_and_slot(0, 0);

	ret = pci_write_config_dword(pci_root, PCI_DATA_REG, data);
	if (ret)
		goto unlock;

	pci_write_config_dword(pci_root, PCI_CMD_REG, cmd);

unlock:
	pci_dev_put(pci_root);
	mutex_unlock(&pci_root_lock);

	return ret;
}
EXPORT_SYMBOL(mfld_msg_write32);

/*
 * Locking strategy::
 *
 * Two semaphores are used to lock the global variables used in
 * the code. The entry points in pmu driver are pmu_pci_set_power_state()
 * and PMU interrupt handler contexts, so here is the flow of how
 * the semaphores are used.
 *
 * In D0ix command case::
 * set_power_state process context:
 * set_power_state()->acquire_scu_ready_sem()->issue_interactive_cmd->
 * wait_for_interactive_complete->release scu_ready sem
 *
 * PMU Interrupt context:
 * pmu_interrupt_handler()->release interactive_complete->return
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

static struct sram_save_info {
	u32 start;
	u32 end;
	int s0i1_need;
	void __iomem *ddr_iomap;
	void *ddr_save;
} sram_save_info_all[] = {
	/* Data in these areas of SRAM will lost in S0i1/2/3 */
#if 0
	/* DAFCA is for debug purpose, so we dn't save/restore it */
	{0xfffc0000, 0xfffd03ff, 1, 0, 0},	/* DAFCA */

	/* Currently, we don't use below Hooks */
	{0xfffd0400, 0xfffd7fff, 1, 0, 0},	/* OEM/Validation Hooks */

	/* We assume below buffers are not used across s0i3 */
	{0xfffd8000, 0xfffdbfff, 1, 0, 0},	/* IA UMIP update Buf */
	{0xfffdc000, 0xfffdffff, 1, 0, 0},	/* Security UMIP update Buf */
#endif

	/* Data in these areas of SRAM will lost in S0i2/3 */
#if 0
	/*
	 * Audio driver/LPE engine reinitialize Audio SRAM region
	 * after exiting from s0i3. We needn't save/restore it.
	 */
	{0xfffe1000, 0xffff0fff, 0, 0, 0},	/* Audio Buffer */
	{0xffff1000, 0xffff2fff, 0, 0, 0},	/* Audio Mailbox */
#endif
	{0xffff3000, 0xffff306c, 0, 0, 0},	/* OSHOB */
	{0xffff3400, 0xffff341f, 0, 0, 0},	/* OSNIBW */
#if 0
	/* SCU/Punit handles this, OS should treat this as reserved */
	{0xffff6000, 0xffff6fff, 0, 0, 0},	/* P-unit/SCU mailbox */

	/* Debug only. SCU writes this space when feature is enabled */
	{0xffff7000, 0xffff71ef, 0, 0, 0},	/* LP Residency Counters */

	/* SCU initialize this region when powering on SRAM Bank#1 */
	{0xffff7fb0, 0xffff7fbf, 0, 0, 0},	/* eMMC Mutex Register */
#endif
#if 0
	/* Don't save/restore OTG as USB OTG driver takes care it */
	{0xffff8000, 0xffffbfff, 0, 0, 0},	/* USB OTG */
#endif
};

static inline void s0ix_sram_save(u32 s0ix)
{
	struct sram_save_info *p;
	u32 len;
	int i, index;

	for (index = 0; index < ARRAY_SIZE(sram_save_info_all); index++) {
		p = &sram_save_info_all[index];
		if (((s0ix == MID_S0I1_STATE) || (s0ix == MID_LPMP3_STATE))
							&& !p->s0i1_need)
			continue;

		len = ALIGN(p->end - p->start + 1, 4);
		for (i = 0; i < len; i += 4)
			*(unsigned int *)(p->ddr_save + i) =
				readl(p->ddr_iomap + i);
	}
}

static inline void s0ix_sram_restore(u32 s0ix)
{
	struct sram_save_info *p;
	u32 len;
	int i, index;

	for (index = 0; index < ARRAY_SIZE(sram_save_info_all); index++) {
		p = &sram_save_info_all[index];
		if (((s0ix == MID_S0I1_STATE) || (s0ix == MID_LPMP3_STATE))
							&& !p->s0i1_need)
			continue;

		len = ALIGN(p->end - p->start + 1, 4);
		for (i = 0; i < len; i += 4)
			writel(*(unsigned int *)(p->ddr_save + i),
					p->ddr_iomap + i);
	}
}

static void s0ix_sram_save_cleanup(void)
{
	struct sram_save_info *p;
	int index;

	for (index = 0; index < ARRAY_SIZE(sram_save_info_all); index++) {
		p = &sram_save_info_all[index];
		if (p->ddr_iomap) {
			iounmap(p->ddr_iomap);
			p->ddr_iomap = 0;
		}

		kfree(p->ddr_save);
		p->ddr_save = 0;
	}
}

static int s0ix_sram_save_init(void)
{
	int index, ret = 0;
	u32 len;
	struct sram_save_info *p;

	for (index = 0; index < ARRAY_SIZE(sram_save_info_all); index++) {
		p = &sram_save_info_all[index];
		len = ALIGN(p->end - p->start + 1, 4);
		p->ddr_iomap = ioremap_nocache(p->start, len);
		if (!p->ddr_iomap) {
			ret = -ENOMEM;
			s0ix_sram_save_cleanup();
			break;
		}

		p->ddr_save = kmalloc(len, GFP_KERNEL);
		if (!p->ddr_save) {
			ret = -ENOMEM;
			s0ix_sram_save_cleanup();
			break;
		}
	}

	return ret;
}

/* To CLEAR C6 offload Bit(LSB) in MSR 120 */
static inline void clear_c6offload_bit(void)
{
	u32 msr_low, msr_high;

	/* Read MSR_C6OFFLOAD_CTL_REG Res */
	rdmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
	/* CLEAR LSB Corresponding to C6_OFFLOAD */
	msr_low = msr_low & ~MSR_C6OFFLOAD_SET_LOW;
	msr_high = msr_high & ~MSR_C6OFFLOAD_SET_HIGH;
	wrmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
}

/* To SET C6 offload Bit(LSB) in MSR 120 */
static inline void set_c6offload_bit(void)
{
	u32 msr_low, msr_high;

	/* Read MSR_C6OFFLOAD_CTL_REG Res */
	rdmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
	/* SET LSB Corresponding to C6_OFFLOAD */
	msr_low = msr_low | MSR_C6OFFLOAD_SET_LOW;
	msr_high = msr_high | MSR_C6OFFLOAD_SET_HIGH;
	wrmsr(MSR_C6OFFLOAD_CTL_REG, msr_low, msr_high);
}

static void pmu_stat_start(enum sys_state type)
{
	mid_pmu_cxt->pmu_current_state = type;
	mid_pmu_cxt->pmu_stats[type].last_try = cpu_clock(smp_processor_id());
}

static void pmu_stat_end(void)
{
	enum sys_state type = mid_pmu_cxt->pmu_current_state;

	if (type > SYS_STATE_S0I0 && type < SYS_STATE_MAX) {
		mid_pmu_cxt->pmu_stats[type].last_entry =
			mid_pmu_cxt->pmu_stats[type].last_try;

		/*if it is the first entry save the time*/
		if (!mid_pmu_cxt->pmu_stats[type].count)
			mid_pmu_cxt->pmu_stats[type].first_entry =
				mid_pmu_cxt->pmu_stats[type].last_entry;

		mid_pmu_cxt->pmu_stats[type].time +=
			cpu_clock(smp_processor_id())
			- mid_pmu_cxt->pmu_stats[type].last_entry;

		mid_pmu_cxt->pmu_stats[type].count++;

	}

	mid_pmu_cxt->pmu_current_state = SYS_STATE_S0I0;
}

static void pmu_stat_clear(void)
{
	mid_pmu_cxt->pmu_current_state = SYS_STATE_S0I0;
}

static void pmu_stat_error(u8 err_type)
{
	enum sys_state type = mid_pmu_cxt->pmu_current_state;
	u8 err_index;

	if (type > SYS_STATE_S0I0 && type < SYS_STATE_MAX) {
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
			trace_printk("S0ix_NO_ACKC6_INT\n");
			err_index = 2;
			break;
		default:
			err_index = 3;
			break;
		}

		if (err_index < 3)
			mid_pmu_cxt->pmu_stats[type].err_count[err_index]++;
	}
}

#ifdef CONFIG_DEBUG_FS
static void pmu_stat_seq_printf(struct seq_file *s, int type, char *typestr)
{
	unsigned long long t;
	unsigned long nanosec_rem, remainder;
	unsigned long time, init_2_now_time;

	seq_printf(s, "%s\t%5llu\t%10llu\t%9llu\t%9llu\t", typestr,
		 mid_pmu_cxt->pmu_stats[type].count,
		 mid_pmu_cxt->pmu_stats[type].err_count[0],
		 mid_pmu_cxt->pmu_stats[type].err_count[1],
		 mid_pmu_cxt->pmu_stats[type].err_count[2]);

	t = mid_pmu_cxt->pmu_stats[type].time;
	nanosec_rem = do_div(t, 1000000000);

	/* convert time in secs */
	time = (unsigned long)t;

	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t = mid_pmu_cxt->pmu_stats[type].last_entry;
	nanosec_rem = do_div(t, 1000000000);
	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t = mid_pmu_cxt->pmu_stats[type].first_entry;
	nanosec_rem = do_div(t, 1000000000);
	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t =  cpu_clock(raw_smp_processor_id());
	t -= mid_pmu_cxt->pmu_init_time;
	nanosec_rem = do_div(t, 1000000000);

	init_2_now_time =  (unsigned long) t;

	/* for calculating percentage residency */
	time = time * 100;
	t = (u64) time;

	/* take care of divide by zero */
	if (init_2_now_time) {
		remainder = do_div(t, init_2_now_time);
		time = (unsigned long) t;

		/* for getting 3 digit precision after
		 * decimal dot */
		remainder *= 1000;
		t = (u64) remainder;
		remainder = do_div(t, init_2_now_time);
	} else
		time = t = 0;

	seq_printf(s, "%5lu.%03lu\n", time, (unsigned long) t);
}
#endif

/*Experimentally enabling S0i3 as extended C-States*/
static char s0ix[5] = "s0ix";
static int extended_cstate_mode = MID_S0IX_STATE;
static int set_extended_cstate_mode(const char *val, struct kernel_param *kp)
{
	char       valcp[5];
	int cstate_mode;

	strncpy(valcp, val, sizeof(valcp) - 1);
	valcp[4] = '\0';

	if (strcmp(valcp, "s0i1") == 0)
		cstate_mode = MID_S0I1_STATE;
	else if (strcmp(valcp, "lmp3") == 0)
		cstate_mode = MID_LPMP3_STATE;
	else if (strcmp(valcp, "s0i3") == 0)
		cstate_mode = MID_S0I3_STATE;
	else if (strcmp(valcp, "i1i3") == 0)
		cstate_mode = MID_I1I3_STATE;
	else if (strcmp(valcp, "lpi1") == 0)
		cstate_mode = MID_LPI1_STATE;
	else if (strcmp(valcp, "lpi3") == 0)
		cstate_mode = MID_LPI3_STATE;
	else if (strcmp(valcp, "s0ix") == 0)
		cstate_mode = MID_S0IX_STATE;
	else {
		cstate_mode = 0;
		strncpy(valcp, "none", 5);
	}
	strncpy(s0ix, valcp, 5);

	down(&mid_pmu_cxt->scu_ready_sem);
	extended_cstate_mode = cstate_mode;
	up(&mid_pmu_cxt->scu_ready_sem);

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
	"setup extended c state s0ix mode [s0i3|s0i1|lmp3|"
				"i1i3|lpi1|lpi3|s0ix|none]");

static int _pmu_issue_command(struct pmu_ss_states *pm_ssc, int mode, int ioc,
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
	int ret = 0;
	int possible;

	if (!pmu_initialized)
		goto ret;

	/* dont do s0ix if suspend in progress */
	if (mid_pmu_cxt->suspend_started)
		goto ret;

	/* dont do s0ix if shutdown in progress */
	if (unlikely(mid_pmu_cxt->shutdown_started))
		goto ret;

	if (!mid_pmu_cxt->display_off || !mid_pmu_cxt->camera_off)
		goto ret;

	possible = mid_pmu_cxt->s0ix_possible;

	switch (extended_cstate_mode) {
	case MID_S0I1_STATE:
	case MID_S0I3_STATE:
	case MID_I1I3_STATE:
		/* user asks s0i1/s0i3 then only
		 * do s0i1/s0i3, dont do lpmp3
		 */
		if (possible == MID_S0IX_STATE)
			ret = extended_cstate_mode & possible;
		break;

	case MID_LPMP3_STATE:
		/* user asks lpmp3 then only
		 * do lpmp3
		 */
		if (possible == MID_LPMP3_STATE)
			ret = MID_LPMP3_STATE;
		break;

	case MID_LPI1_STATE:
	case MID_LPI3_STATE:
		/* user asks lpmp3/i1/i3 then only
		 * do lpmp3/i1/i3
		 */
		if (possible == MID_LPMP3_STATE)
			ret = MID_LPMP3_STATE;
		else if (possible == MID_S0IX_STATE)
			ret = extended_cstate_mode >> REMOVE_LP_FROM_LPIX;
		break;

	case MID_S0IX_STATE:
		ret = possible;
		break;
	}

ret:
	return ret;
}
EXPORT_SYMBOL(get_target_platform_state);

/**
 * This function set all devices in d0i0 and deactivates pmu driver.
 * The function is used before IFWI update as it needs devices to be
 * in d0i0 during IFWI update. Reboot is needed to work pmu
 * driver properly again. After calling this function and IFWI
 * update, system is always rebooted as IFWI update function,
 * intel_scu_ipc_medfw_upgrade() is called from mrst_emergency_reboot().
 */
int pmu_set_devices_in_d0i0(void)
{
	int status;
	struct pmu_ss_states cur_pmssc;

	/* Ignore request until we have initialized */
	if (unlikely((!pmu_initialized)))
		return 0;

	cur_pmssc.pmu2_states[0] = D0I0_MASK;
	cur_pmssc.pmu2_states[1] = D0I0_MASK;
	cur_pmssc.pmu2_states[2] = D0I0_MASK;
	cur_pmssc.pmu2_states[3] = D0I0_MASK;

	down(&mid_pmu_cxt->scu_ready_sem);

	mid_pmu_cxt->shutdown_started = true;

	/* Issue the pmu command to PMU 2
	 * flag is needed to distinguish between
	 * S0ix vs interactive command in pmu_sc_irq()
	 */
	status = _pmu_issue_command(&cur_pmssc, SET_MODE, 0, PMU_NUM_2);

	if (unlikely(status != PMU_SUCCESS)) {	/* pmu command failed */
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
			 "Failed to Issue a PM command to PMU2\n");
		mid_pmu_cxt->shutdown_started = false;
		goto unlock;
	}

	if (_pmu2_wait_not_busy())
		BUG();

unlock:
	up(&mid_pmu_cxt->scu_ready_sem);
	return status;
}
EXPORT_SYMBOL(pmu_set_devices_in_d0i0);

static int _pmu_read_status(int pmu_num, int type)
{
	u32 temp;
	union pmu_pm_status result;

	temp = readl(&mid_pmu_cxt->pmu_reg->pm_sts);

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
	/* South complex in Penwell has multiple registers for
	 * PM_SSC, etc.
	 */
	writel(pm_ssc->pmu2_states[0], &mid_pmu_cxt->pmu_reg->pm_ssc[0]);

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		writel(pm_ssc->pmu2_states[1],
				&mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(pm_ssc->pmu2_states[2],
				&mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(pm_ssc->pmu2_states[3],
				&mid_pmu_cxt->pmu_reg->pm_ssc[3]);
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
		     *command, u8 ioc, enum sys_state state, int pmu_num)
{
	/* parameter check */
	if ((ioc != 1) && (ioc != 0)) {
		pr_alert("invalid ioc bit\n");
		return PMU_FAILED;
	}

	/* construct the command to send SET_CFG to particular PMU */
	if (pmu_num == PMU_NUM_1) {
		command->pmu1_params.cmd = SET_CFG_CMD;
		command->pmu1_params.ioc = ioc;
		command->pmu1_params.mode_id = MODE_ID_MAGIC_NUM;
		command->pmu1_params.rsvd = 0;
	} else if (pmu_num == PMU_NUM_2) {
		command->pmu2_params.d_param.cmd =
		    SET_CFG_CMD;
		command->pmu2_params.d_param.ioc = ioc;
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

	/* write the value of PM_CMD into particular PMU */
	dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "command being written %x\n", \
	command->pmu_pm_set_cfg_cmd_value);

	writel(command->pmu_pm_set_cfg_cmd_value,
					&mid_pmu_cxt->pmu_reg->pm_cmd);

	return 0;
}

/* return the last wake source id, and make statistics about wake sources */
static int pmu_get_wake_source(void)
{
	u32 wake0, wake1;
	int i;
	int source = INVALID_WAKE_SRC;
	enum sys_state type = mid_pmu_cxt->pmu_current_state;

	wake0 = readl(&mid_pmu_cxt->pmu_reg->pm_wks[0]);
	wake1 = readl(&mid_pmu_cxt->pmu_reg->pm_wks[1]);

	while (wake0) {
		i = fls(wake0) - 1;
		source = i + mid_pmu_cxt->pmu1_max_devs;
		mid_pmu_cxt->num_wakes[source][type]++;
		trace_printk("wake_from_lss%d\n",
				source - mid_pmu_cxt->pmu1_max_devs);
		wake0 &= ~(1<<i);
	}

	while (wake1) {
		i = fls(wake1) - 1;
		source = i + 32 + mid_pmu_cxt->pmu1_max_devs;
		mid_pmu_cxt->num_wakes[source][type]++;
		trace_printk("wake_from_lss%d\n",
				source - mid_pmu_cxt->pmu1_max_devs);
		wake1 &= ~(1<<i);
	}

	return source;
}

static int pmu_is_interrupt_pending(int pmu_num)
{
	u32 temp;
	union pmu_pm_ics result;

	/* read the pm interrupt status register */
	temp = readl(&mid_pmu_cxt->pmu_reg->pm_ics);
	result.pmu_pm_ics_value = temp;

	/* return the pm interrupt status int pending bit info */
	return result.pmu_pm_ics_parts.int_pend;
}

static inline void pmu_clear_interrupt_pending(int pmu_num)
{
	u32 temp;

	/* read the pm interrupt status register */
	temp = readl(&mid_pmu_cxt->pmu_reg->pm_ics);

	/* write into the PM_ICS register */
	writel(temp, &mid_pmu_cxt->pmu_reg->pm_ics);
}

static inline void pmu_enable_interrupt_from_pmu(int pmu_num)
{
	u32 temp;
	union pmu_pm_ics result;

	/* read the pm interrupt status register */
	temp = readl(&mid_pmu_cxt->pmu_reg->pm_ics);
	result = (union pmu_pm_ics)temp;

	result.pmu_pm_ics_parts.int_enable = 1;

	/* enable the interrupts */
	writel(result.pmu_pm_ics_value, &mid_pmu_cxt->pmu_reg->pm_ics);
}

static inline int pmu_read_interrupt_status(int pmu_num)
{
	u32 temp;
	union pmu_pm_ics result;

	/* read the pm interrupt status register */
	temp = readl(&mid_pmu_cxt->pmu_reg->pm_ics);

	result.pmu_pm_ics_value = temp;

	if (result.pmu_pm_ics_parts.int_status == 0)
		return PMU_FAILED;

	/* return the pm interrupt status int pending bit info */
	return result.pmu_pm_ics_parts.int_status;
}

void acquire_scu_ready_sem(void)
{
	if (likely(pmu_initialized))
		down(&mid_pmu_cxt->scu_ready_sem);
}
EXPORT_SYMBOL(acquire_scu_ready_sem);

void release_scu_ready_sem(void)
{
	if (likely(pmu_initialized))
		up(&mid_pmu_cxt->scu_ready_sem);
}
EXPORT_SYMBOL(release_scu_ready_sem);

/**
 * This is a helper function used to program pm registers
 * in SCU. We configure the wakeable devices & the
 * wake sub system states on exit from S0ix state
 */
int mfld_s0ix_enter(int s0ix_state)
{
	struct pmu_ss_states cur_pmsss;
	int num_retry = 15000, ret = 0;
	u32 s0ix_value, ssw_val;

	/* check if we can acquire scu_ready_sem
	 * if we are not able to then do a c6 */
	if (down_trylock(&mid_pmu_cxt->scu_ready_sem))
		goto ret;

	/* If PMU is busy, we'll retry on next C6 */
	if (unlikely(_pmu_read_status(PMU_NUM_2, PMU_BUSY_STATUS))) {
		up(&mid_pmu_cxt->scu_ready_sem);
		goto ret;
	}

	/* setup the wake capable devices */
	if (s0ix_state == MID_S3_STATE) {
		writel(~IGNORE_S3_WKC0, &mid_pmu_cxt->pmu_reg->pm_wkc[0]);
		writel(~IGNORE_S3_WKC1, &mid_pmu_cxt->pmu_reg->pm_wkc[1]);
	} else {
		writel(mid_pmu_cxt->ss_config->wake_state.wake_enable[0],
		       &mid_pmu_cxt->pmu_reg->pm_wkc[0]);
		writel(mid_pmu_cxt->ss_config->wake_state.wake_enable[1],
		       &mid_pmu_cxt->pmu_reg->pm_wkc[1]);
	}

	/* Re-program the sub systems state on wakeup as the current SSS*/
	pmu_read_sss(&cur_pmsss);

	writel(cur_pmsss.pmu2_states[0], &mid_pmu_cxt->pmu_reg->pm_wssc[0]);
	writel(cur_pmsss.pmu2_states[1], &mid_pmu_cxt->pmu_reg->pm_wssc[1]);
	writel(cur_pmsss.pmu2_states[2], &mid_pmu_cxt->pmu_reg->pm_wssc[2]);
	writel(cur_pmsss.pmu2_states[3], &mid_pmu_cxt->pmu_reg->pm_wssc[3]);

	switch (s0ix_state) {
	case MID_S0I1_STATE:
		writel(S0I1_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(S0I1_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(S0I1_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(S0I1_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I1);
		s0ix_value = S0I1_VALUE;
		break;
	case MID_LPMP3_STATE:
		writel(LPMP3_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(LPMP3_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(LPMP3_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(LPMP3_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I2);
		s0ix_value = LPMP3_VALUE;
		break;
	case MID_S0I3_STATE:
		writel(S0I3_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(S0I3_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(S0I3_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(S0I3_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S0I3);
		s0ix_value = S0I3_VALUE;
		break;
	case MID_S3_STATE:
		writel(S0I3_SSS0, &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
		writel(S0I3_SSS1, &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
		writel(S0I3_SSS2, &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
		writel(S0I3_SSS3, &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
		pmu_stat_start(SYS_STATE_S3);
		s0ix_value = S0I3_VALUE;
		break;
	default:
		BUG_ON(1);
	}

	clear_c6offload_bit();

	s0ix_sram_save(s0ix_state);

	/* no need to proceed if schedule pending */
	if (unlikely(need_resched())) {
		pmu_stat_clear();
		up(&mid_pmu_cxt->scu_ready_sem);
		goto ret;
	}

	/* issue a command to SCU */
	writel(s0ix_value, &mid_pmu_cxt->pmu_reg->pm_cmd);

	/* At this point we have committed an S0ix command
	 * will have to wait for the SCU s0ix complete
	 * intertupt to proceed further.
	 */

	mid_pmu_cxt->s0ix_entered = s0ix_state;

	if (s0ix_value == S0I3_VALUE) {
		do {
			ssw_val = readl(mid_pmu_cxt->base_addr.offload_reg);
			if ((ssw_val & C6OFFLOAD_BIT_MASK) ==  C6OFFLOAD_BIT)
				break;
		} while (num_retry--);

		if (likely((ssw_val & C6OFFLOAD_BIT_MASK) ==  C6OFFLOAD_BIT))
			set_c6offload_bit();
		else {
			WARN(1, "mid_pmu: error cpu offload bit not set.\n");
			pmu_stat_clear();

			goto ret;
		}
	}
	ret = s0ix_state;

ret:
	return ret;
}
EXPORT_SYMBOL(mfld_s0ix_enter);

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
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Invalid interrupt source\n");

	switch (status) {
	case INVALID_INT:
		status = IRQ_NONE;
		goto ret_no_clear;

	case CMD_COMPLETE_INT:
		break;

	case CMD_ERROR_INT:
		mid_pmu_cxt->cmd_error_int++;
		break;

	case SUBSYS_POW_ERR_INT:
	case NO_ACKC6_INT:
	case S0ix_MISS_INT:
		pmu_stat_error(status);
		break;

	case WAKE_RECEIVED_INT:
		(void)pmu_get_wake_source();
		pmu_stat_end();
		break;
	}

	pmu_stat_clear();

	/* clear the interrupt pending bit */
	pmu_clear_interrupt_pending(PMU_NUM_2);

	/*
	 * In case of interactive command
	 * let the waiting set_power_state()
	 * release scu_ready_sem
	 */
	if (mid_pmu_cxt->interactive_cmd_sent) {
		mid_pmu_cxt->interactive_cmd_sent = 0;

		/* unblock set_power_state() */
		complete(&mid_pmu_cxt->set_mode_complete);
	} else {
		s0ix_sram_restore(mid_pmu_cxt->s0ix_entered);

		/* Wakeup allother CPU's */
		if (mid_pmu_cxt->s0ix_entered == MID_S0I3_STATE)
			apic->send_IPI_allbutself(RESCHEDULE_VECTOR);

		mid_pmu_cxt->s0ix_entered = 0;

		/* S0ix case release it */
		up(&mid_pmu_cxt->scu_ready_sem);
	}

	status = IRQ_HANDLED;
ret_no_clear:
	return status;
}

void pmu_set_s0ix_complete(void)
{
	if (unlikely(mid_pmu_cxt->s0ix_entered))
		writel(0, &mid_pmu_cxt->pmu_reg->pm_msic);
}
EXPORT_SYMBOL(pmu_set_s0ix_complete);

bool pmu_is_s0i3_in_progress(void)
{
	bool state = false;

	if (pmu_initialized && mid_pmu_cxt->s0ix_entered == MID_S0I3_STATE)
		state = true;

	return state;
}
EXPORT_SYMBOL(pmu_is_s0i3_in_progress);

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
		if (likely(mid_pmu_cxt->pci_dev_hash[h_index].pdev == pdev)) {
			*found = 1;
			break;
		}

		/* assume no deletions, hence there shouldn't be any
		 * gaps ie., NULL's */
		if (unlikely(mid_pmu_cxt->pci_dev_hash[h_index].pdev == NULL)) {
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
		return (int)mid_pmu_cxt->pci_dev_hash[h_index].index;

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
	else if (type) {
		WARN_ON(ss >= MAX_LSS_POSSIBLE);
		index = mid_pmu_cxt->pmu1_max_devs + ss;
	}

	if (index != PMU_FAILED) {
		/* insert into hash table */
		mid_pmu_cxt->pci_dev_hash[h_index].pdev = pdev;

		/* assume index never exceeds 0xff */
		WARN_ON(index > 0xFF);

		mid_pmu_cxt->pci_dev_hash[h_index].index = (u8)index;

		if (index < mid_pmu_cxt->pmu1_max_devs) {
			set_mid_pci_ss_idx(index, 0);
			set_mid_pci_ss_pos(index, (u8)index);
			set_mid_pci_pmu_num(index, PMU_NUM_1);
		} else if (index >= mid_pmu_cxt->pmu1_max_devs &&
			   index < (mid_pmu_cxt->pmu1_max_devs +
						mid_pmu_cxt->pmu2_max_devs)) {
			set_mid_pci_ss_idx(index,
					(u8)(ss / mid_pmu_cxt->ss_per_reg));
			set_mid_pci_ss_pos(index,
					(u8)(ss % mid_pmu_cxt->ss_per_reg));
			set_mid_pci_pmu_num(index, PMU_NUM_2);
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
		set_mid_pci_log_id(index, (u32)index);
		set_mid_pci_cap(index, PM_SUPPORT);
	} else if ((base_class == PCI_BASE_CLASS_MULTIMEDIA) &&
		(sub_class == ISP_SUB_CLASS)) {
		if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
			set_mid_pci_log_id(index, (u32)index);
			set_mid_pci_cap(index, PM_SUPPORT);
		} else if (ss && cap) {
			set_mid_pci_log_id(index, (u32)(ss & LOG_ID_MASK));
			set_mid_pci_cap(index, cap);
		}
	} else if (ss && cap) {
		set_mid_pci_log_id(index, (u32)(ss & LOG_ID_MASK));
		set_mid_pci_cap(index, cap);
	}

	for (i = 0; i < PMU_MAX_LSS_SHARE &&
		get_mid_pci_drv(index, i); i++) {
		/* do nothing */
	}

	WARN_ON(i >= PMU_MAX_LSS_SHARE);

	if (i < PMU_MAX_LSS_SHARE) {
		set_mid_pci_drv(index, i, pdev);
		set_mid_pci_power_state(index, i, PCI_D3hot);
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
	pm_ssc->pmu2_states[0] = readl(&mid_pmu_cxt->pmu_reg->pm_sss[0]);

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		pm_ssc->pmu2_states[1] =
				readl(&mid_pmu_cxt->pmu_reg->pm_sss[1]);
		pm_ssc->pmu2_states[2] =
				readl(&mid_pmu_cxt->pmu_reg->pm_sss[2]);
		pm_ssc->pmu2_states[3] =
				readl(&mid_pmu_cxt->pmu_reg->pm_sss[3]);
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
	mid_pmu_cxt->s0ix_possible = 0;

	if (state != PCI_D0) {
		struct pmu_ss_states cur_pmsss;

		pmu_read_sss(&cur_pmsss);

		if (likely(check_s0ix_possible(&cur_pmsss)))
			mid_pmu_cxt->s0ix_possible = MID_S0IX_STATE;
		else if (check_lpmp3_possible(&cur_pmsss))
			mid_pmu_cxt->s0ix_possible = MID_LPMP3_STATE;
	}
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
		if (get_mid_pci_drv(lss_index, i) == pdev)
			set_mid_pci_power_state(lss_index, i, state);

		if (get_mid_pci_drv(lss_index, i) &&
			(get_mid_pci_power_state(lss_index, i) < weakest)) {
			dev_warn(&pdev->dev, "%s:%s prevented me to suspend...\n",
			    dev_name(&(get_mid_pci_drv(lss_index, i))->dev),
			    dev_driver_string
				(&(get_mid_pci_drv(lss_index, i))->dev));

			weakest = get_mid_pci_power_state(lss_index, i);
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
	*ss_pos		= get_mid_pci_ss_pos(i);
	*ss_idx		= get_mid_pci_ss_idx(i);
	*pmu_num	= get_mid_pci_pmu_num(i);

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
		addr = mid_pmu_cxt->apm_base + APM_STS;
		break;
	case OSPM_REG_TYPE:
		addr = mid_pmu_cxt->ospm_base + OSPM_PM_SSS;
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

	spin_lock_irqsave(&mid_pmu_cxt->nc_ready_lock, flags);

	switch (reg_type) {
	case APM_REG_TYPE:
		pwr_cnt = inl(mid_pmu_cxt->apm_base + APM_STS);
		break;
	case OSPM_REG_TYPE:
		pwr_cnt = inl(mid_pmu_cxt->ospm_base + OSPM_PM_SSS);
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
			outl(pwr_mask, mid_pmu_cxt->apm_base + APM_CMD);
			break;
		case OSPM_REG_TYPE:
			outl(pwr_mask, (mid_pmu_cxt->ospm_base + OSPM_PM_SSC));
			break;
		}

		ret =
		wait_for_nc_pmcmd_complete(pwr_mask, state_type, reg_type);
	}

unlock:
	spin_unlock_irqrestore(&mid_pmu_cxt->nc_ready_lock, flags);
	return ret;
}
EXPORT_SYMBOL(pmu_nc_set_power_state);

/**
 * pmu_set_lss01_to_d0i0_atomic -
 *	This function is mainly meant for panic case to bring up storage
 *	device so panic reports can be logged, this is NOT meant for
 *	general purpose use!.
 *
 */
int pmu_set_lss01_to_d0i0_atomic(void)
{
	u32 pm_cmd_val;
	u32 new_value;
	int sub_sys_pos, sub_sys_index;
	struct pmu_ss_states cur_pmssc;
	int status = 0;

	if (unlikely((!pmu_initialized)))
		return 0;

	/* LSS 01 is index = 0, pos = 1 */
	sub_sys_index	= PMU_EMMC0_LSS_01 / mid_pmu_cxt->ss_per_reg;
	sub_sys_pos	= PMU_EMMC0_LSS_01 % mid_pmu_cxt->ss_per_reg;

	memset(&cur_pmssc, 0, sizeof(cur_pmssc));

	status = _pmu2_wait_not_busy();

	if (status)
		goto err;

	pmu_read_sss(&cur_pmssc);

	/* set D0i0 the LSS bits */
	pm_cmd_val =
		(D0I3_MASK << (sub_sys_pos * BITS_PER_LSS));
	new_value = cur_pmssc.pmu2_states[sub_sys_index] &
						(~pm_cmd_val);

	if (new_value == cur_pmssc.pmu2_states[sub_sys_index])
		goto err;

	cur_pmssc.pmu2_states[sub_sys_index] = new_value;

	/* set the lss positions that need
	 * to be ignored to D0i0 */
	cur_pmssc.pmu2_states[0] &= ~IGNORE_SSS0;
	cur_pmssc.pmu2_states[1] &= ~IGNORE_SSS1;
	cur_pmssc.pmu2_states[2] &= ~IGNORE_SSS2;
	cur_pmssc.pmu2_states[3] &= ~IGNORE_SSS3;

	status = _pmu_issue_command(&cur_pmssc, SET_MODE, 0, PMU_NUM_2);

	if (unlikely(status != PMU_SUCCESS)) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
			 "Failed to Issue a PM command to PMU2\n");
		goto err;
	}

	/*
	 * Wait for interactive command to complete.
	 * If we dont wait, there is a possibility that
	 * the driver may access the device before its
	 * powered on in SCU.
	 *
	 */
	if (_pmu2_wait_not_busy())
		BUG();

err:
	return status;
}
EXPORT_SYMBOL(pmu_set_lss01_to_d0i0_atomic);

/**
 * pmu_pci_set_power_state - Callback function is used by all the PCI devices
 *			for a platform  specific device power on/shutdown.
 *
 */
int __ref pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state)
{
	int i;
	u32 pm_cmd_val;
	u32 new_value;
	int sub_sys_pos, sub_sys_index;
	int pmu_num;
	struct pmu_ss_states cur_pmssc;
	int status = 0;

	/* Ignore callback from devices until we have initialized */
	if (unlikely((!pmu_initialized)))
		return 0;

	/* Try to acquire the scu_ready_sem, if not
	 * get blocked, until pmu_sc_irq() releases */
	down(&mid_pmu_cxt->scu_ready_sem);

	/* There could be some drivers that don't handle
	 * shutdown properly, that is even thou
	 * device shtudown is called we still could
	 * recieve set_power_state from buggy drivers
	 */
	if (unlikely(mid_pmu_cxt->shutdown_started)) {
		printk(KERN_CRIT "%s: received after device shutdown from"
		       " %04x %04X %s %20s:\n",
		__func__, pdev->vendor, pdev->device, dev_name(&pdev->dev),
			dev_driver_string(&pdev->dev));
		BUG();
	}

	mid_pmu_cxt->interactive_cmd_sent = 1;

	status =
		pmu_pci_to_indexes(pdev, &i, &pmu_num,
				&sub_sys_index,  &sub_sys_pos);

	if (status)
		goto unlock;

	state = pmu_pci_get_weakest_state_for_lss(i, pdev, state);

	/* store the display status */
	if (i == GFX_LSS_INDEX)
		mid_pmu_cxt->display_off = (int)(state != PCI_D0);

	/*Update the Camera status per ISP Driver Suspended/Resumed */
	if (i == MFLD_ISP_POS)
		mid_pmu_cxt->camera_off = (int)(state != PCI_D0);

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
				(pci_2_mfld_state(state) <<
					(sub_sys_pos * BITS_PER_LSS));

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
		status = _pmu_issue_command(&cur_pmssc, SET_MODE, 1, PMU_NUM_2);

		if (unlikely(status != PMU_SUCCESS)) {
			dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
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
		if (!wait_for_completion_timeout(
			    &mid_pmu_cxt->set_mode_complete, 5 * HZ))
			BUG();

		pmu_set_s0ix_possible(state);

		/* update stats */
		inc_d0ix_stat((i-mid_pmu_cxt->pmu1_max_devs),
					pci_2_mfld_state(state));
	}

unlock:
	mid_pmu_cxt->interactive_cmd_sent = 0;
	up(&mid_pmu_cxt->scu_ready_sem);
	return status;
}

static int _pmu_issue_command(struct pmu_ss_states *pm_ssc, int mode, int ioc,
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
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "PMU2 is busy, Operation not"
		"permitted\n");
		return PMU_FAILED;
	}


	/* enable interrupts in PMU2 so that interrupts are
	 * propagated when ioc bit for a particular set
	 * command is set
	 */
	/* Enable the hardware interrupt */
	tmp = readl(&mid_pmu_cxt->pmu_reg->pm_ics);
	tmp |= 0x100;/* Enable interrupts */
	writel(tmp, &mid_pmu_cxt->pmu_reg->pm_ics);

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
	pmu_send_set_config_command(&command, ioc, sys_state, PMU_NUM_2);

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

	spin_lock_irqsave(&mid_pmu_cxt->nc_ready_lock, flags);

	switch (reg_type) {
	case APM_REG_TYPE:
		pwr_sts = inl(mid_pmu_cxt->apm_base + APM_STS);
		break;
	case OSPM_REG_TYPE:
		pwr_sts = inl(mid_pmu_cxt->ospm_base + OSPM_PM_SSS);
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
	spin_unlock_irqrestore(&mid_pmu_cxt->nc_ready_lock, flags);
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
		off = mid_pmu_cxt->display_off;
		islands_num = ISLANDS_GFX;
		islands = &display_islands[0];
	} else if (PCI_SLOT(pdev->devfn) == DEV_ISP &&
			PCI_FUNC(pdev->devfn) == FUNC_ISP) {
		off = mid_pmu_cxt->camera_off;
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
	down(&mid_pmu_cxt->scu_ready_sem);
	_pmu2_wait_not_busy();
	pmu_read_sss(&cur_pmsss);
	up(&mid_pmu_cxt->scu_ready_sem);

	seq_printf(s, "TARGET_CFG: ");
	seq_printf(s, "SSS0:%08X ", S0IX_TARGET_SSS0_MASK);
	seq_printf(s, "SSS1:%08X ", S0IX_TARGET_SSS1_MASK);
	seq_printf(s, "SSS2:%08X ", S0IX_TARGET_SSS2_MASK);
	seq_printf(s, "SSS3:%08X ", S0IX_TARGET_SSS3_MASK);

	seq_printf(s, "\n");
	seq_printf(s, "CONDITION FOR S0I3: ");
	seq_printf(s, "SSS0:%08X ", S0IX_TARGET_SSS0);
	seq_printf(s, "SSS1:%08X ", S0IX_TARGET_SSS1);
	seq_printf(s, "SSS2:%08X ", S0IX_TARGET_SSS2);
	seq_printf(s, "SSS3:%08X ", S0IX_TARGET_SSS3);

	seq_printf(s, "\n");
	seq_printf(s, "SSS: ");

	for (i = 0; i < 4; i++)
		seq_printf(s, "%08lX ", cur_pmsss.pmu2_states[i]);

	if (!mid_pmu_cxt->display_off)
		seq_printf(s, "display not suspended: blocking s0ix\n");
	else if (!mid_pmu_cxt->camera_off)
		seq_printf(s, "camera not suspended: blocking s0ix\n");
	else if (mid_pmu_cxt->s0ix_possible & MID_S0IX_STATE)
		seq_printf(s, "can enter s0i1 or s0i3\n");
	else if (mid_pmu_cxt->s0ix_possible & MID_LPMP3_STATE)
		seq_printf(s, "can enter lpmp3\n");
	else
		seq_printf(s, "blocking s0ix\n");

	seq_printf(s, "cmd_error_int count: %d\n", mid_pmu_cxt->cmd_error_int);

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
		needed	= ((target_mask &  mask) != 0);

		seq_printf(s, "pci %04x %04X %s %20s: lss:%02d reg:%d"
			"mask:%08X wk:%02d:%02d:%02d:%03d %s  %s\n",
			pdev->vendor, pdev->device, dev_name(&pdev->dev),
			dev_driver_string(&pdev->dev),
			index - mid_pmu_cxt->pmu1_max_devs, ss_idx, mask,
			mid_pmu_cxt->num_wakes[index][SYS_STATE_S0I1],
			mid_pmu_cxt->num_wakes[index][SYS_STATE_S0I2],
			mid_pmu_cxt->num_wakes[index][SYS_STATE_S0I3],
			mid_pmu_cxt->num_wakes[index][SYS_STATE_S3],
			dstates[val&3],
			(needed && !val) ? "blocking s0ix" : "");

	}

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


	buf[buf_size] = 0;

	if (((strlen("clear")+1) == buf_size) &&
		!strncmp(buf, "clear", strlen("clear"))) {
		down(&mid_pmu_cxt->scu_ready_sem);
		memset(mid_pmu_cxt->pmu_stats, 0,
					sizeof(mid_pmu_cxt->pmu_stats));
		memset(mid_pmu_cxt->num_wakes, 0,
					sizeof(mid_pmu_cxt->num_wakes));
		mid_pmu_cxt->pmu_current_state = SYS_STATE_S0I0;
		mid_pmu_cxt->pmu_init_time =
			cpu_clock(raw_smp_processor_id());
		clear_d0ix_stats();
		up(&mid_pmu_cxt->scu_ready_sem);
	}

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
	{"NA", "NA", "NA"},
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

	down(&mid_pmu_cxt->scu_ready_sem);

	lss_status[0] = readl(&mid_pmu_cxt->pmu_reg->pm_sss[0]);
	lss_status[1] = readl(&mid_pmu_cxt->pmu_reg->pm_sss[1]);
	lss_status[2] = readl(&mid_pmu_cxt->pmu_reg->pm_sss[2]);
	lss_status[3] = readl(&mid_pmu_cxt->pmu_reg->pm_sss[3]);

	up(&mid_pmu_cxt->scu_ready_sem);

	lss = 0;
	seq_printf(s, "%5s\t%12s %35s %5s %4s %4s %4s %4s\n",
			"lss", "block", "subsystem", "state", "D0i0", "D0i1",
			"D0i2", "D0i3");
	seq_printf(s, "====================================================="
		      "=====================\n");
	for (sss_reg_index = 0; sss_reg_index < 4; sss_reg_index++) {
		status = lss_status[sss_reg_index];
		for (offset = 0; offset < sizeof(unsigned long) * 8 / 2;
								offset++) {
			sub_status = status & 3;
			if (lss >= medfield_lsses_num)
				entry = &medfield_lsses[medfield_lsses_num - 1];
			else
				entry = &medfield_lsses[lss];
			seq_printf(s, "%5s\t%12s %35s %4s %4d %4d %4d %4d\n",
					entry->lss_name, entry->block,
					entry->subsystem,
					lss_device_status[sub_status],
					get_d0ix_stat(lss, SS_STATE_D0I0),
					get_d0ix_stat(lss, SS_STATE_D0I1),
					get_d0ix_stat(lss, SS_STATE_D0I2),
					get_d0ix_stat(lss, SS_STATE_D0I3));

			status >>= 2;
			lss++;
		}
	}

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
		int pmu_num = get_mid_pci_pmu_num(i);
		struct pci_dev *pdev = get_mid_pci_drv(i, 0);

		if ((pmu_num == PMU_NUM_2) && pdev) {
			int ss_idx, ss_pos;
			pci_power_t state;

			ss_idx = get_mid_pci_ss_idx(i);
			ss_pos = get_mid_pci_ss_pos(i);
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
	/* Also take IGNORE_CFG in account (for e.g. GPIO1)*/
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

	dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "PMU Driver loaded\n");
	spin_lock_init(&mid_pmu_cxt->nc_ready_lock);

	if (s0ix_sram_save_init())
		pr_err("sram save init fail!\n");

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
	ss_config = kzalloc(sizeof(struct pmu_suspend_config), GFP_KERNEL);

	if (ss_config == NULL) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Allocation of memory"
		"for ss_config has failed\n");
		status = PMU_FAILED;
		goto out_err1;
	}

	memset(&pmu_config, 0, sizeof(pmu_config));

	ss_config->ss_state = pmu_config;

	/* initialize for the autonomous S0i3 */
	mid_pmu_cxt->ss_config = ss_config;

	/* setup the wake capable devices */
	mid_pmu_cxt->ss_config->wake_state.wake_enable[0] = WAKE_ENABLE_0;
	mid_pmu_cxt->ss_config->wake_state.wake_enable[1] = WAKE_ENABLE_1;

	/* Acquire the scu_ready_sem */
	down(&mid_pmu_cxt->scu_ready_sem);

	/* Now we have initialized the driver
	 * Allow drivers to get blocked in
	 * pmu_pci_set_power_state(), until we finish
	 * first interactive command.
	 */
	pmu_initialized = true;

	/* get the current status of each of the driver
	 * and update it in SCU
	 */
	update_all_lss_states(&pmu_config);

	/* send a interactive command to fw */
	mid_pmu_cxt->interactive_cmd_sent = 1;
	status = _pmu_issue_command(&pmu_config, SET_MODE, 1, PMU_NUM_2);
	if (status != PMU_SUCCESS) {
		mid_pmu_cxt->interactive_cmd_sent = 0;
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,\
		 "Failure from pmu mode change to interactive."
		" = %d\n", status);
		status = PMU_FAILED;
		up(&mid_pmu_cxt->scu_ready_sem);
		goto out_err2;
	}

	/*
	 * Wait for interactive command to complete.
	 * If we dont wait, there is a possibility that
	 * the driver may access the device before its
	 * powered on in SCU.
	 *
	 */
	if (!wait_for_completion_timeout(&mid_pmu_cxt->set_mode_complete,
					 5 * HZ))
		BUG();

	/* In cases were gfx is not enabled
	 * this will enable s0ix immediately
	 */
	pmu_set_s0ix_possible(PCI_D3hot);

	up(&mid_pmu_cxt->scu_ready_sem);

	return PMU_SUCCESS;

out_err2:
	kfree(ss_config);
	mid_pmu_cxt->ss_config = NULL;
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
	int cmd;
	u32 data;

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&mid_pmu_cxt->pmu_wake_lock,
					WAKE_LOCK_SUSPEND, "mid_pmu");
#endif

	/* Init the device */
	ret = pci_enable_device(dev);
	if (ret) {
		pr_err("Mid PM device cant be enabled\n");
		goto out_err0;
	}

	/* store the dev */
	mid_pmu_cxt->pmu_dev = dev;

	ret = pci_request_regions(dev, PMU_DRV_NAME);
	if (ret < 0) {
		pr_err("pci request region has failed\n");
		goto out_err1;
	}

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		mid_pmu_cxt->pmu1_max_devs = PMU1_MAX_PENWELL_DEVS;
		mid_pmu_cxt->pmu2_max_devs = PMU2_MAX_PENWELL_DEVS;
		mid_pmu_cxt->ss_per_reg = 16;
	} else {
		mid_pmu_cxt->pmu1_max_devs = PMU1_MAX_MRST_DEVS;
		mid_pmu_cxt->pmu2_max_devs = PMU2_MAX_MRST_DEVS;
		mid_pmu_cxt->ss_per_reg = 8;
	}

	/* Map the NC PM registers */
	cmd = (MSG_READ_CMD << 24) | (OSPM_PUNIT_PORT << 16) |
						(OSPM_APMBA << 8);
	if (mfld_msg_read32(cmd, &data)) {
		ret = PMU_FAILED;
		goto out_err1;
	}
	mid_pmu_cxt->apm_base = data & 0xffff;

	cmd = (MSG_READ_CMD << 24) | (OSPM_PUNIT_PORT << 16) |
						 (OSPM_OSPMBA << 8);
	if (mfld_msg_read32(cmd, &data)) {
		ret = PMU_FAILED;
		goto out_err1;
	}
	mid_pmu_cxt->ospm_base = data & 0xffff;

	/* Map the memory of pmu1 PMU reg base */
	pmu = pci_iomap(dev, 0, 0);
	if (pmu == NULL) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
				"Unable to map the PMU2 address space\n");
		ret = PMU_FAILED;
		goto out_err2;
	}

	mid_pmu_cxt->pmu_reg = pmu;

	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		/* Map the memory of offload_reg */
		mid_pmu_cxt->base_addr.offload_reg =
					ioremap_nocache(0xffd01ffc, 4);
		if (mid_pmu_cxt->base_addr.offload_reg == NULL) {
			dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
			"Unable to map the offload_reg address space\n");
			ret = PMU_FAILED;
			goto out_err3;
		}
	}

	if (request_irq(dev->irq, pmu_sc_irq, IRQF_NO_SUSPEND, PMU_DRV_NAME,
			NULL)) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Registering isr has failed\n");
		ret = PMU_FAILED;
		goto out_err3;
	}

	/* call pmu init() for initialization of pmu interface */
	ret = pmu_init();
	if (ret != PMU_SUCCESS) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "PMU initialization has failed\n");
		goto out_err4;
	}

	mid_pmu_cxt->pmu_init_time =
		cpu_clock(raw_smp_processor_id());

	return 0;

out_err4:
	free_irq(dev->irq, &pmu_sc_irq);
out_err3:
	pci_iounmap(dev, mid_pmu_cxt->base_addr.pmu2_base);
	mid_pmu_cxt->base_addr.pmu2_base = NULL;
out_err2:
	mid_pmu_cxt->base_addr.pmu1_base = NULL;
out_err1:
	pci_release_region(dev, 0);
	pci_disable_device(dev);
out_err0:
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&mid_pmu_cxt->pmu_wake_lock);
#endif
	return ret;
}

static void __devexit mid_pmu_remove(struct pci_dev *dev)
{
	dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Mid PM mid_pmu_remove called\n");

	/* Freeing up the irq */
	free_irq(dev->irq, &pmu_sc_irq);

	/* Freeing up memory allocated for PMU1 & PMU2 */
	if (__mrst_cpu_chip == MRST_CPU_CHIP_PENWELL) {
		iounmap(mid_pmu_cxt->base_addr.offload_reg);
		mid_pmu_cxt->base_addr.offload_reg = NULL;
	}

	pci_iounmap(dev, mid_pmu_cxt->pmu_reg);
	mid_pmu_cxt->base_addr.pmu1_base = NULL;
	mid_pmu_cxt->base_addr.pmu2_base = NULL;

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

	if (pmu_initialized) {
		status =
		pmu_pci_to_indexes(pdev, &i, &pmu_num,
					&sub_sys_index,  &sub_sys_pos);

		if ((status == PMU_SUCCESS) &&
			(pmu_num == PMU_NUM_2)) {

			device_lss =
				(sub_sys_index * mid_pmu_cxt->ss_per_reg) +
								sub_sys_pos;

			state = _pmu_choose_state(device_lss);
		}
	}

	return state;
}

bool pmu_pci_power_manageable(struct pci_dev *dev)
{
	return true;
}

/* At this point of time we expect all devices to be
 * wake capable will be modified in future
 */
bool pmu_pci_can_wakeup(struct pci_dev *dev)
{
	return true;
}

int pmu_pci_sleep_wake(struct pci_dev *dev, bool enable)
{
	return 0;
}

static int mfld_s3_enter(void)
{
	u32 temp = 0;

	if (mfld_s0ix_enter(MID_S3_STATE) != MID_S3_STATE) {
		pmu_set_s0ix_complete();
		return -EINVAL;
	}

	__monitor((void *) &temp, 0, 0);
	smp_mb();
	__mwait(C6_HINT, 1);
	pmu_set_s0ix_complete();
	return 0;
}

static int mid_begin(suspend_state_t state)
{
	mid_pmu_cxt->suspend_started = true;
	return 0;
}

static int mid_valid(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_MEM:
		/* check if we are ready */
		if (likely(pmu_initialized))
			ret = 1;
	break;
	}

	return ret;
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
		return -EINVAL;

#ifdef CONFIG_HAS_WAKELOCK
	/*
	 * Check if some driver managed to set wakelock during the suspend path.
	 * It is better to skip S3 in that case.
	 */
	if (has_wake_lock(WAKE_LOCK_SUSPEND))
		return -EBUSY;
#endif

	trace_printk("s3_entry\n");
	ret = mfld_s3_enter();
	trace_printk("s3_exit %d\n", ret);
	if (ret != 0)
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
				"Failed to enter S3 status: %d\n", ret);

	return ret;
}

/* This function is here just to have a hook to execute code before
 * generic x86 shutdown is executed. saved_shutdown contains pointer
 * to original generic x86 shutdown function
 */
void mfld_shutdown(void)
{
	if (mid_pmu_cxt) {
		down(&mid_pmu_cxt->scu_ready_sem);
		mid_pmu_cxt->shutdown_started = true;
		up(&mid_pmu_cxt->scu_ready_sem);
	}

	if (saved_shutdown)
		saved_shutdown();
}

void mfld_power_off(void)
{
	/* wait till SCU is ready */
	if (_pmu2_wait_not_busy())
		dev_err(&mid_pmu_cxt->pmu_dev->dev,
			"SCU BUSY. Unable to Enter S5\n");
	else
		/*send S5 command to SCU*/
		writel(S5_VALUE, &mid_pmu_cxt->pmu_reg->pm_cmd);
}

static void mid_end(void)
{
#ifdef CONFIG_HAS_WAKELOCK
	/* Prime the wakelock system to re-suspend after giving other
	 * threads a chance to wake up and acquire wake lock
	 * this avoids to put wake lock in other things like pwrbutton
	 */
	long timeout = HZ;
	wake_lock_timeout(&mid_pmu_cxt->pmu_wake_lock, timeout);
#endif
	mid_pmu_cxt->suspend_started = false;
}

static struct platform_suspend_ops mid_suspend_ops = {
	.begin = mid_begin,
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

	mid_pmu_cxt = kzalloc(sizeof(struct mid_pmu_dev), GFP_KERNEL);

	if (mid_pmu_cxt == NULL)
		return -ENOMEM;

#ifndef CONFIG_VIDEO_ATOMISP
	mid_pmu_cxt->camera_off = 1;
#endif

#ifndef GFX_ENABLE
	/* If Gfx is disabled
	 * assume s0ix is not blocked
	 * from gfx side
	 */
	mid_pmu_cxt->display_off = 1;
#endif

	/* initialize the semaphores */
	sema_init(&mid_pmu_cxt->scu_ready_sem, 1);
	init_completion(&mid_pmu_cxt->set_mode_complete);

	/* registering PCI device */
	ret = pci_register_driver(&driver);
	suspend_set_ops(&mid_suspend_ops);

	return ret;
}
fs_initcall(mid_pci_register_init);

static void __exit mid_pci_cleanup(void)
{
	suspend_set_ops(NULL);
	s0ix_sram_save_cleanup();
	/* registering PCI device */
	pci_unregister_driver(&driver);

	if (mid_pmu_cxt)
		kfree(mid_pmu_cxt->ss_config);

	kfree(mid_pmu_cxt);
}
module_exit(mid_pci_cleanup);
