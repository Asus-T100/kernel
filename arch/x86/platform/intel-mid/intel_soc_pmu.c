/*
 * intel_soc_pmu.c - This driver provides interface to configure the 2 pmu's
 * Copyright (c) 2012, Intel Corporation.
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

#include "intel_soc_pmu.h"

#ifdef CONFIG_DRM_INTEL_MID
#define GFX_ENABLE
#endif

bool pmu_initialized;

DEFINE_MUTEX(pci_root_lock);

/* mid_pmu context structure */
struct mid_pmu_dev *mid_pmu_cxt;

struct platform_pmu_ops *pmu_ops;
/*
 * Locking strategy::
 *
 * one semaphore (scu_ready sem) is used for accessing busy bit,
 * issuing interactive cmd in the code.
 * The entry points in pmu driver are pmu_pci_set_power_state()
 * and PMU interrupt handler contexts, so here is the flow of how
 * the semaphore is used.
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

/* Maps pci power states to SCU D0ix mask */
static int pci_to_platform_state(pci_power_t pci_state)
{

	static int mask[]  = {D0I0_MASK, D0I1_MASK,
				D0I2_MASK, D0I3_MASK, D0I3_MASK};

	int state = D0I0_MASK;

	if (pci_state > 4)
		WARN(1, "%s: wrong pci_state received.\n", __func__);

	else
		state = mask[pci_state];

	return state;
}

/* PCI Device Id structure */
static DEFINE_PCI_DEVICE_TABLE(mid_pm_ids) = {
	{PCI_VDEVICE(INTEL, MID_PMU_MFLD_DRV_DEV_ID), 0},
	{PCI_VDEVICE(INTEL, MID_PMU_CLV_DRV_DEV_ID), 0},
	{}
};

MODULE_DEVICE_TABLE(pci, mid_pm_ids);

static inline bool nc_device_state(void)
{
	return !mid_pmu_cxt->display_off || !mid_pmu_cxt->camera_off;
}

u32 get_s0ix_val_set_pm_ssc(int s0ix_state)
{
	u32 s0ix_value;

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
		pmu_dump_logs();
		BUG_ON(1);
	}
	return s0ix_value;
}

static char s0ix[5] = "s0ix";
int extended_cstate_mode = MID_S0IX_STATE;
static int set_extended_cstate_mode(const char *val, struct kernel_param *kp)
{
	char valcp[5];
	int cstate_mode;

	memcpy(valcp, val, 5);
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
	memcpy(s0ix, valcp, 5);

	down(&mid_pmu_cxt->scu_ready_sem);
	extended_cstate_mode = cstate_mode;
	up(&mid_pmu_cxt->scu_ready_sem);

	return 0;
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

/*
 *Decide which state the platfrom can go to based on user and
 *platfrom inputs
*/
int get_final_state(unsigned long *eax)
{
	int ret = 0;
	int possible = mid_pmu_cxt->s0ix_possible;

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

	if ((ret == MID_S0IX_STATE) &&
			(*eax == MID_LPMP3_STATE))
		ret = MID_S0I1_STATE;
	else if ((ret <= *eax ||
			(ret == MID_S0IX_STATE)))
		ret = ret & *eax;
	else
		ret = 0;

	return ret;
}

int get_target_platform_state(unsigned long *eax)
{
	int ret = 0;

	if (unlikely(!pmu_initialized))
		goto ret;

	/* dont do s0ix if suspend in progress */
	if (unlikely(mid_pmu_cxt->suspend_started))
		goto ret;

	/* dont do s0ix if shutdown in progress */
	if (unlikely(mid_pmu_cxt->shutdown_started))
		goto ret;

	if (nc_device_state())
		goto ret;

	ret = get_final_state(eax);

ret:
	*eax = C6_HINT;
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
	status = pmu_issue_interactive_command(&cur_pmssc, false);

	if (unlikely(status != PMU_SUCCESS)) {	/* pmu command failed */
		printk(KERN_CRIT "Failed to Issue a PM command to PMU2\n");
		mid_pmu_cxt->shutdown_started = false;
		goto unlock;
	}

	if (_pmu2_wait_not_busy()) {
		pmu_dump_logs();
		BUG();
	}

unlock:
	up(&mid_pmu_cxt->scu_ready_sem);
	return status;
}
EXPORT_SYMBOL(pmu_set_devices_in_d0i0);

static int _pmu_read_status(int type)
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

int _pmu2_wait_not_busy(void)
{
	int pmu_busy_retry = PMU2_BUSY_TIMEOUT;

	/* wait 500ms that the latest pmu command finished */
	do {
		if (_pmu_read_status(PMU_BUSY_STATUS) == 0)
			return 0;

		udelay(1);
	} while (--pmu_busy_retry);

	WARN(1, "pmu2 busy!");

	return -EBUSY;
}

static int _pmu2_wait_not_busy_yield(void)
{
	int pmu_busy_retry = 50000; /* 500msec minimum */

	/* wait for the latest pmu command finished */
	do {
		usleep_range(10, 500);

		if (!_pmu_read_status(PMU_BUSY_STATUS))
			return 0;
	} while (--pmu_busy_retry);

	WARN(1, "pmu2 busy!");

	return -EBUSY;
}

static void pmu_write_subsys_config(struct pmu_ss_states *pm_ssc)
{
	/* South complex in Penwell has multiple registers for
	 * PM_SSC, etc.
	 */
	writel(pm_ssc->pmu2_states[0], &mid_pmu_cxt->pmu_reg->pm_ssc[0]);
	writel(pm_ssc->pmu2_states[1], &mid_pmu_cxt->pmu_reg->pm_ssc[1]);
	writel(pm_ssc->pmu2_states[2], &mid_pmu_cxt->pmu_reg->pm_ssc[2]);
	writel(pm_ssc->pmu2_states[3], &mid_pmu_cxt->pmu_reg->pm_ssc[3]);
}

static void log_wakeup_irq(void)
{
	unsigned int irr = 0, vector = 0;
	int offset = 0, irq = 0;
	struct irq_desc *desc;
	const char *act_name;

	for (offset = (FIRST_EXTERNAL_VECTOR/32);
	offset < (NR_VECTORS/32); offset++) {
		irr = apic_read(APIC_IRR + (offset * 0x10));
		while (irr) {
			vector = __ffs(irr);
			irr &= ~(1 << vector);
			irq = __this_cpu_read(
					vector_irq[vector + (offset * 32)]);
			if (irq < 0)
				continue;
			pr_info("wakeup from  IRQ %d\n", irq);

			desc = irq_to_desc(irq);

			if ((desc) && (desc->action)) {
				act_name = desc->action->name;
				pr_info("IRQ %d,action name:%s\n",
					irq,
					(act_name) ? (act_name) : "no action");
			}
		}
	}
	return;
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

	if ((mid_pmu_cxt->pmu_current_state == SYS_STATE_S3)
		&& mid_pmu_cxt->suspend_started)
		switch (source - mid_pmu_cxt->pmu1_max_devs) {
		case PMU_USB_OTG_LSS_06:
			pr_info("wakeup from USB.\n");
			break;
		case PMU_GPIO0_LSS_39:
			pr_info("wakeup from GPIO.\n");
			break;
		case PMU_HSI_LSS_03:
			pr_info("wakeup from HSI.\n");
			break;
		default:
			log_wakeup_irq();
		}
	return source;
}

static int pmu_interrupt_pending(void)
{
	u32 temp;
	union pmu_pm_ics result;

	/* read the pm interrupt status register */
	temp = readl(&mid_pmu_cxt->pmu_reg->pm_ics);
	result.pmu_pm_ics_value = temp;

	/* return the pm interrupt status int pending bit info */
	return result.pmu_pm_ics_parts.int_pend;
}

static inline void pmu_clear_pending_interrupt(void)
{
	u32 temp;

	/* read the pm interrupt status register */
	temp = readl(&mid_pmu_cxt->pmu_reg->pm_ics);

	/* write into the PM_ICS register */
	writel(temp, &mid_pmu_cxt->pmu_reg->pm_ics);
}

static inline int pmu_read_interrupt_status(void)
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

/*This function is used for programming the wake capable devices*/
static void pmu_prepare_wake(int s0ix_state)
{

	struct pmu_ss_states cur_pmsss;

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
}

/*
 * Valid wake source: lss_number 0 to 63
 * Returns true if 'lss_number' is wake source
 * else false
 */
bool mid_pmu_is_wake_source(u32 lss_number)
{
	u32 wake = 0;
	bool ret = false;

	if (lss_number > PMU_RSVD9_LSS_63)
		return ret;

	if (lss_number < PMU_SCU_RAM1_LSS_32) {
		wake = readl(&mid_pmu_cxt->pmu_reg->pm_wks[0]);
		wake &= (1 << lss_number);
	} else {
		wake = readl(&mid_pmu_cxt->pmu_reg->pm_wks[1]);
		wake &= (1 << (lss_number-PMU_SCU_RAM1_LSS_32));
	}

	if (wake)
		ret = true;

	return ret;
}

int mid_s0ix_enter(int s0ix_state)
{
	int ret = 0;

	if (unlikely(!pmu_ops || !pmu_ops->enter))
		goto ret;

	/* check if we can acquire scu_ready_sem
	 * if we are not able to then do a c6 */
	if (down_trylock(&mid_pmu_cxt->scu_ready_sem))
		goto ret;

	/* If PMU is busy, we'll retry on next C6 */
	if (unlikely(_pmu_read_status(PMU_BUSY_STATUS))) {
		up(&mid_pmu_cxt->scu_ready_sem);
		pr_debug("mid_pmu_cxt->scu_read_sem is up\n");
		goto ret;
	}

	pmu_prepare_wake(s0ix_state);

	/* no need to proceed if schedule pending */
	if (unlikely(need_resched())) {
		pmu_stat_clear();
		up(&mid_pmu_cxt->scu_ready_sem);
		goto ret;
	}

	/* entry function for pmu driver ops */
	if (pmu_ops->enter(s0ix_state))
		ret = s0ix_state;

ret:
	return ret;
}

/**
 * pmu_sc_irq - pmu driver interrupt handler
 * Context: interrupt context
 */
static irqreturn_t pmu_sc_irq(int irq, void *ignored)
{
	int status;
	irqreturn_t ret = IRQ_NONE;
	int wake_source;

	/* check if interrup pending bit is set, if not ignore interrupt */
	if (unlikely(!pmu_interrupt_pending())) {
		pmu_log_pmu_irq(PMU_FAILED, mid_pmu_cxt->interactive_cmd_sent);
		goto ret_no_clear;
	}

	/* read the interrupt status */
	status = pmu_read_interrupt_status();
	if (unlikely(status == PMU_FAILED))
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Invalid interrupt source\n");

	switch (status) {
	case INVALID_INT:
		pmu_log_pmu_irq(status, mid_pmu_cxt->interactive_cmd_sent);
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
		wake_source = pmu_get_wake_source();
		trace_printk("wake_from_lss%d\n",
				wake_source);
		pmu_stat_end();
		break;
	}
	pmu_log_pmu_irq(status, mid_pmu_cxt->interactive_cmd_sent);

	pmu_stat_clear();

	/* clear the interrupt pending bit */
	pmu_clear_pending_interrupt();

	/*
	 * In case of interactive command
	 * let the waiting set_power_state()
	 * release scu_ready_sem
	 */
	if (unlikely(mid_pmu_cxt->interactive_cmd_sent)) {
		mid_pmu_cxt->interactive_cmd_sent = false;

		/* unblock set_power_state() */
		complete(&mid_pmu_cxt->set_mode_complete);
	} else {
		if (pmu_ops->wakeup)
			pmu_ops->wakeup();

		mid_pmu_cxt->s0ix_entered = 0;

		/* S0ix case release it */
		up(&mid_pmu_cxt->scu_ready_sem);
	}

	ret = IRQ_HANDLED;
ret_no_clear:
	return ret;
}

void pmu_set_s0ix_complete(void)
{
	if (unlikely(mid_pmu_cxt->s0ix_entered))
		writel(0, &mid_pmu_cxt->pmu_reg->pm_msic);
}
EXPORT_SYMBOL(pmu_set_s0ix_complete);

bool pmu_is_s0ix_in_progress(void)
{
	bool state = false;

	if (pmu_initialized && mid_pmu_cxt->s0ix_entered)
		state = true;

	return state;
}
EXPORT_SYMBOL(pmu_is_s0ix_in_progress);

static inline u32 find_index_in_hash(struct pci_dev *pdev, int *found)
{
	u32 h_index;
	int i;

	/* assuming pdev is not null */
	WARN_ON(pdev == NULL);

	/*assuming pdev pionter will not change from platfrom
	 *boot to shutdown*/
	h_index = jhash_1word((u32) (long) pdev,
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

	type = ss & LOG_SS_MASK;
	ss = ss & LOG_ID_MASK;

	if ((base_class == PCI_BASE_CLASS_DISPLAY) && !sub_class)
		index = 1;
	else if ((base_class == PCI_BASE_CLASS_MULTIMEDIA) &&
			(sub_class == ISP_SUB_CLASS))
				index = ISP_POS;
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

static void get_pci_lss_info(struct pci_dev *pdev)
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
			set_mid_pci_log_id(index, (u32)index);
			set_mid_pci_cap(index, PM_SUPPORT);
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

		get_pci_lss_info(pdev);
	}
}

void pmu_read_sss(struct pmu_ss_states *pm_ssc)
{
	pm_ssc->pmu2_states[0] =
			readl(&mid_pmu_cxt->pmu_reg->pm_sss[0]);
	pm_ssc->pmu2_states[1] =
			readl(&mid_pmu_cxt->pmu_reg->pm_sss[1]);
	pm_ssc->pmu2_states[2] =
			readl(&mid_pmu_cxt->pmu_reg->pm_sss[2]);
	pm_ssc->pmu2_states[3] =
			readl(&mid_pmu_cxt->pmu_reg->pm_sss[3]);
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
			(get_mid_pci_power_state(lss_index, i) < weakest))
			weakest = get_mid_pci_power_state(lss_index, i);
	}
	return weakest;
}

int pmu_pci_to_indexes(struct pci_dev *pdev, int *index,
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

static bool update_nc_device_states(int i, pci_power_t state)
{
	int status = 0;

	/* store the display status */
	if (i == GFX_LSS_INDEX) {
		mid_pmu_cxt->display_off = (state != PCI_D0);
		return true;
	}

	/*Update the Camera status per ISP Driver Suspended/Resumed
	* ISP power islands are also updated accordingly, otherwise Dx state
	* in PMCSR refuses to change.
	*/
	if (i == ISP_POS) {
		status = pmu_nc_set_power_state(APM_ISP_ISLAND | APM_IPH_ISLAND,
			(state != PCI_D0) ?
			OSPM_ISLAND_DOWN : OSPM_ISLAND_UP,
			APM_REG_TYPE);
		if (status)
			return false;
		mid_pmu_cxt->camera_off = (state != PCI_D0);
		return true;
	}

	return false;
}

void init_nc_device_states(void)
{
#ifndef CONFIG_VIDEO_ATOMISP
	mid_pmu_cxt->camera_off = true;
#endif

#ifndef GFX_ENABLE
	/* If Gfx is disabled
	 * assume s0ix is not blocked
	 * from gfx side
	 */
	mid_pmu_cxt->display_off = true;
#endif

	return;
}

/* FIXME::Currently HSI Modem 7060 (BZ# 28529) is having a issue and
* it will not go to Low Power State on CVT. So Standby will not work
* if HSI is enabled.
* We can choose between Standby/HSI based on enable_stadby 1/0.
*/
#ifdef CONFIG_BOARD_CTP
unsigned int enable_standby __read_mostly;
module_param(enable_standby, uint, 0000);
#endif

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

int pmu_set_emmc_to_d0i0_atomic(void)
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

	/* Request SCU for PM interrupt enabling */
	writel(PMU_PANIC_EMMC_UP_REQ_CMD, mid_pmu_cxt->emergeny_emmc_up_addr);

	status = pmu_issue_interactive_command(&cur_pmssc, false);

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
	if (_pmu2_wait_not_busy()) {
		pmu_dump_logs();
		BUG();
	}

err:

	return status;
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

/*
* update_dev_res - Calulates & Updates the device residency when
* a device state change occurs.
* Computation of respective device residency starts when
* its first state tranisition happens after the pmu driver
* is initialised.
*
*/
void update_dev_res(int index, pci_power_t state)
{
	if (state != PCI_D0) {
		if (mid_pmu_cxt->pmu_dev_res[index].start == 0) {
			mid_pmu_cxt->pmu_dev_res[index].start = cpu_clock(0);
			mid_pmu_cxt->pmu_dev_res[index].d0i3_entry =
				mid_pmu_cxt->pmu_dev_res[index].start;
				mid_pmu_cxt->pmu_dev_res[index].d0i0_acc = 0;
		} else{
			mid_pmu_cxt->pmu_dev_res[index].d0i3_entry =
							cpu_clock(0);
			mid_pmu_cxt->pmu_dev_res[index].d0i0_acc +=
			(mid_pmu_cxt->pmu_dev_res[index].d0i3_entry -
				 mid_pmu_cxt->pmu_dev_res[index].d0i0_entry);
		}
	} else {
		if (mid_pmu_cxt->pmu_dev_res[index].start == 0) {
			mid_pmu_cxt->pmu_dev_res[index].start =
						 cpu_clock(0);
			mid_pmu_cxt->pmu_dev_res[index].d0i0_entry
				= mid_pmu_cxt->pmu_dev_res[index].start;
			mid_pmu_cxt->pmu_dev_res[index].d0i3_acc = 0;
		} else {
			mid_pmu_cxt->pmu_dev_res[index].d0i0_entry =
						 cpu_clock(0);
			mid_pmu_cxt->pmu_dev_res[index].d0i3_acc +=
			(mid_pmu_cxt->pmu_dev_res[index].d0i0_entry -
			mid_pmu_cxt->pmu_dev_res[index].d0i3_entry);
		}
	}
	mid_pmu_cxt->pmu_dev_res[index].state = state;
}

/**
 * pmu_pci_set_power_state - Callback function is used by all the PCI devices
 *			for a platform  specific device power on/shutdown.
 *
 */
int __ref pmu_pci_set_power_state(struct pci_dev *pdev, pci_power_t state)
{
	u32 new_value;
	int i = 0;
	u32 pm_cmd_val, chk_val;
	int sub_sys_pos, sub_sys_index;
	int pmu_num;
	struct pmu_ss_states cur_pmssc;
	int status = 0;
	int retry_times = 0;
	ktime_t calltime, delta, rettime;

	/* Ignore callback from devices until we have initialized */
	if (unlikely((!pmu_initialized)))
		return 0;

	might_sleep();

	/* Try to acquire the scu_ready_sem, if not
	 * get blocked, until pmu_sc_irq() releases */
	down(&mid_pmu_cxt->scu_ready_sem);

	/*get LSS index corresponding to pdev, its position in
	 *32 bit register and its register numer*/
	status =
		pmu_pci_to_indexes(pdev, &i, &pmu_num,
				&sub_sys_index,  &sub_sys_pos);

	if (status)
		goto unlock;

	/*in case a LSS is assigned to more than one pdev, we need
	  *to find the shallowest state the LSS should be put into*/
	state = pmu_pci_get_weakest_state_for_lss(i, pdev, state);

	/*If the LSS corresponds to northcomplex device, update
	  *the status and return*/
	if (update_nc_device_states(i, state)) {
		if (mid_pmu_cxt->pmu_dev_res[i].state == state)
			goto unlock;
		else {
			if (i < MAX_DEVICES)
				update_dev_res(i, state);
			goto unlock;
		}
	}

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
			(pci_to_platform_state(state) <<
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
	status = pmu_issue_interactive_command(&cur_pmssc, false);

	if (unlikely(status != PMU_SUCCESS)) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
			 "Failed to Issue a PM command to PMU2\n");
		goto unlock;
	}

	calltime = ktime_get();
retry:
	/*
	 * Wait for interactive command to complete.
	 * If we dont wait, there is a possibility that
	 * the driver may access the device before its
	 * powered on in SCU.
	 *
	 */
	status = _pmu2_wait_not_busy_yield();
	if (unlikely(status)) {
		rettime = ktime_get();
		delta = ktime_sub(rettime, calltime);
		retry_times++;

		printk(KERN_CRIT "%s: D0ix transition failure:"
				" %04x %04X %s %20s:\n", __func__,
				pdev->vendor, pdev->device,
				dev_name(&pdev->dev),
				dev_driver_string(&pdev->dev));
		printk(KERN_CRIT "interrupt pending = %d\n",
				pmu_interrupt_pending());
		printk(KERN_CRIT "pmu_busy_status = %d\n",
				_pmu_read_status(PMU_BUSY_STATUS));
		printk(KERN_CRIT "suspend_started = %d\n",
				mid_pmu_cxt->suspend_started);
		printk(KERN_CRIT "shutdown_started = %d\n",
				mid_pmu_cxt->shutdown_started);
		printk(KERN_CRIT "interactive_cmd_sent = %d\n",
				(int)mid_pmu_cxt->interactive_cmd_sent);
		printk(KERN_CRIT "camera_off = %d display_off = %d\n",
				mid_pmu_cxt->camera_off,
				mid_pmu_cxt->display_off);
		printk(KERN_CRIT "s0ix_possible = 0x%x\n",
				mid_pmu_cxt->s0ix_possible);
		printk(KERN_CRIT "s0ix_entered = 0x%x\n",
				mid_pmu_cxt->s0ix_entered);
		printk(KERN_CRIT "pmu_current_state = %d\n",
				mid_pmu_cxt->pmu_current_state);
		printk(KERN_CRIT "PMU is BUSY! retry_times[%d] total_delay[%lli]ms. Retry ...\n",
				retry_times, (long long) ktime_to_ms(delta));
		pmu_dump_logs();

		trigger_all_cpu_backtrace();
		if (retry_times < 60)
			goto retry;
		else
			BUG();
	}

	pmu_set_s0ix_possible(state);

	/* update stats */
	inc_d0ix_stat((i-mid_pmu_cxt->pmu1_max_devs),
				pci_to_platform_state(state));

	/* check if tranisition to requested state has happened */
	pmu_read_sss(&cur_pmssc);
	chk_val = cur_pmssc.pmu2_states[sub_sys_index] &
		(D0I3_MASK << (sub_sys_pos * BITS_PER_LSS));
	new_value &= (D0I3_MASK << (sub_sys_pos * BITS_PER_LSS));

	if ((chk_val == new_value) && (i < MAX_DEVICES))
		update_dev_res(i, state);

unlock:
	up(&mid_pmu_cxt->scu_ready_sem);

	return status;
}

pci_power_t platfrom_pmu_choose_state(int lss)
{
	pci_power_t state = PCI_D3hot;

	if (pmu_ops->pci_choose_state)
		state = pmu_ops->pci_choose_state(lss);

	return state;
}

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

			state = platfrom_pmu_choose_state(device_lss);
		}
	}

	return state;
}

int pmu_issue_interactive_command(struct pmu_ss_states *pm_ssc, bool ioc)
{
	u32 tmp;
	u32 command;

	if (_pmu2_wait_not_busy()) {
		dev_err(&mid_pmu_cxt->pmu_dev->dev,
			"SCU BUSY. Operation not permitted\n");
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

	/* Configure the sub systems for pmu2 */
	pmu_write_subsys_config(pm_ssc);

	command = (ioc) ? INTERACTIVE_IOC_VALUE : INTERACTIVE_VALUE;

	/* send interactive command to SCU */
	writel(command, &mid_pmu_cxt->pmu_reg->pm_cmd);

	pmu_log_command(command, pm_ssc);

	return 0;
}

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
			 (pci_to_platform_state(state) <<
				(ss_pos * BITS_PER_LSS));

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

static int pmu_init(void)
{
	int status;
	struct pmu_ss_states pmu_config;
	struct pmu_suspend_config *ss_config;

	dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "PMU Driver loaded\n");
	spin_lock_init(&mid_pmu_cxt->nc_ready_lock);


	/* enumerate the PCI configuration space */
	pmu_enumerate();

	/* initialize the stats for pmu driver */
	pmu_stats_init();

	/* register platform pmu ops */
	platform_set_pmu_ops();

	/* platform specific initialization */
	if (pmu_ops->init)
		pmu_ops->init();

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
	mid_pmu_cxt->interactive_cmd_sent = true;
	status = pmu_issue_interactive_command(&pmu_config, true);
	if (status != PMU_SUCCESS) {
		mid_pmu_cxt->interactive_cmd_sent = false;
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
					 2 * HZ)) {
		pmu_dump_logs();
		BUG();
	}

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
	dev_warn(&dev->dev, "PMU DRIVER Probe called\n");

	ret = pci_request_regions(dev, PMU_DRV_NAME);
	if (ret < 0) {
		pr_err("pci request region has failed\n");
		goto out_err1;
	}

	mid_pmu_cxt->pmu1_max_devs = PMU1_MAX_DEVS;
	mid_pmu_cxt->pmu2_max_devs = PMU2_MAX_DEVS;
	mid_pmu_cxt->ss_per_reg = 16;

	data = intel_mid_msgbus_read32(OSPM_PUNIT_PORT, OSPM_APMBA);
	mid_pmu_cxt->apm_base = data & 0xffff;

	data = intel_mid_msgbus_read32(OSPM_PUNIT_PORT, OSPM_OSPMBA);
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

	/* Map the memory of emergency emmc up */
	mid_pmu_cxt->emergeny_emmc_up_addr =
		ioremap_nocache(PMU_PANIC_EMMC_UP_ADDR, 4);
	if (mid_pmu_cxt->emergeny_emmc_up_addr == NULL) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
		"Unable to map the emergency emmc up address space\n");
		ret = PMU_FAILED;
		goto out_err3;
	}

	if (request_irq(dev->irq, pmu_sc_irq, IRQF_NO_SUSPEND, PMU_DRV_NAME,
			NULL)) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Registering isr has failed\n");
		ret = PMU_FAILED;
		goto out_err4;
	}

	/* call pmu init() for initialization of pmu interface */
	ret = pmu_init();
	if (ret != PMU_SUCCESS) {
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "PMU initialization has failed\n");
		goto out_err5;
	}
	dev_warn(&mid_pmu_cxt->pmu_dev->dev, "after pmu initialization\n");

	mid_pmu_cxt->pmu_init_time =
		cpu_clock(raw_smp_processor_id());

	return 0;

out_err5:
	free_irq(dev->irq, &pmu_sc_irq);
out_err4:
	iounmap(mid_pmu_cxt->emergeny_emmc_up_addr);
	mid_pmu_cxt->emergeny_emmc_up_addr = NULL;
out_err3:
	iounmap(mid_pmu_cxt->pmu_reg);
	mid_pmu_cxt->base_addr.pmu1_base = NULL;
	mid_pmu_cxt->base_addr.pmu2_base = NULL;
out_err2:
	pci_release_region(dev, 0);
out_err1:
	pci_disable_device(dev);
out_err0:
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&mid_pmu_cxt->pmu_wake_lock);
#endif
	printk(KERN_ALERT "PMU DRIVER return from probe %d\n", ret);
	return ret;
}

static void __devexit mid_pmu_remove(struct pci_dev *dev)
{
	dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Mid PM mid_pmu_remove called\n");

	/* Freeing up the irq */
	free_irq(dev->irq, &pmu_sc_irq);

	if (pmu_ops->remove)
		pmu_ops->remove();

	iounmap(mid_pmu_cxt->emergeny_emmc_up_addr);
	mid_pmu_cxt->emergeny_emmc_up_addr = NULL;

	pci_iounmap(dev, mid_pmu_cxt->pmu_reg);
	mid_pmu_cxt->base_addr.pmu1_base = NULL;
	mid_pmu_cxt->base_addr.pmu2_base = NULL;

	/* disable the current PCI device */
	pci_release_region(dev, 0);
	pci_disable_device(dev);
}

static void mid_pmu_shutdown(struct pci_dev *dev)
{
	dev_dbg(&mid_pmu_cxt->pmu_dev->dev, "Mid PM mid_pmu_shutdown called\n");

	if (mid_pmu_cxt) {
		down(&mid_pmu_cxt->scu_ready_sem);
		mid_pmu_cxt->shutdown_started = true;
		up(&mid_pmu_cxt->scu_ready_sem);
	}
}

static struct pci_driver driver = {
	.name = PMU_DRV_NAME,
	.id_table = mid_pm_ids,
	.probe = mid_pmu_probe,
	.remove = __devexit_p(mid_pmu_remove),
	.shutdown = mid_pmu_shutdown
};

static int standby_enter(void)
{
	u32 temp = 0;

	if (mid_s0ix_enter(MID_S3_STATE) != MID_S3_STATE) {
		pmu_set_s0ix_complete();
		return -EINVAL;
	}

	__monitor((void *) &temp, 0, 0);
	smp_mb();
	__mwait(C6_HINT, 1);
	pmu_set_s0ix_complete();
	return 0;
}

static int mid_suspend_begin(suspend_state_t state)
{
	mid_pmu_cxt->suspend_started = true;
	return 0;
}

static int mid_suspend_valid(suspend_state_t state)
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

static int mid_suspend_prepare(void)
{
	return 0;
}

static int mid_suspend_prepare_late(void)
{
	return 0;
}

static int mid_suspend_enter(suspend_state_t state)
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
	ret = standby_enter();
	trace_printk("s3_exit %d\n", ret);
	if (ret != 0)
		dev_dbg(&mid_pmu_cxt->pmu_dev->dev,
				"Failed to enter S3 status: %d\n", ret);

	return ret;
}

static void mid_suspend_end(void)
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

static const struct platform_suspend_ops mid_suspend_ops = {
	.begin = mid_suspend_begin,
	.valid = mid_suspend_valid,
	.prepare = mid_suspend_prepare,
	.prepare_late = mid_suspend_prepare_late,
	.enter = mid_suspend_enter,
	.end = mid_suspend_end,
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

	init_nc_device_states();

	/* initialize the semaphores */
	sema_init(&mid_pmu_cxt->scu_ready_sem, 1);
	init_completion(&mid_pmu_cxt->set_mode_complete);

	/* registering PCI device */
	printk(KERN_ALERT "PMU DRIVER BEFORE PROBE\n");
	ret = pci_register_driver(&driver);
	suspend_set_ops(&mid_suspend_ops);

	return ret;
}
fs_initcall(mid_pci_register_init);

void pmu_power_off(void)
{
	/* wait till SCU is ready */
	if (!_pmu2_wait_not_busy())
		writel(S5_VALUE, &mid_pmu_cxt->pmu_reg->pm_cmd);

	else {
		/* If PM_BUSY bit is not clear issue COLD_OFF */
		WARN(1, "%s: pmu busy bit not cleared.\n", __func__);
		intel_scu_ipc_simple_command(IPCMSG_COLD_RESET, 1);
	}
}

static void __exit mid_pci_cleanup(void)
{
	suspend_set_ops(NULL);

	/* registering PCI device */
	pci_unregister_driver(&driver);

	if (mid_pmu_cxt) {
		pmu_stats_finish();
		kfree(mid_pmu_cxt->ss_config);
	}

	kfree(mid_pmu_cxt);
}
module_exit(mid_pci_cleanup);
