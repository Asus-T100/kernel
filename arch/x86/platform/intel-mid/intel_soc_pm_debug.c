/*
 * intel_soc_pm_debug.c - This driver provides debug utilities across
 * multiple platforms
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
#include "intel_soc_pm_debug.h"

#define PMU_DEBUG_PRINT_STATS	(1U << 0)
static int debug_mask;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define DEBUG_PRINT(logging_type, s, debug_level_mask, args...)		\
	do {								\
		if (logging_type)					\
			seq_printf(s, args);				\
		else if (debug_mask &					\
			PMU_DEBUG_PRINT_##debug_level_mask)		\
			pr_info(args);					\
	} while (0)

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

static char *lss_device_status[4] = { "D0i0", "D0i1", "D0i2", "D0i3" };

static char *dstates[] = {"D0", "D0i1", "D0i2", "D0i3"};

static int lsses_num =
			sizeof(lsses)/sizeof(lsses[0]);

#ifdef LOG_PMU_EVENTS
static void pmu_log_timestamp(struct timespec *ts)
{
	if (timekeeping_suspended) {
		ts->tv_sec = 0;
		ts->tv_nsec = 0;
	} else {
		ktime_get_ts(ts);
	}
}

void pmu_log_pmu_irq(int status, bool interactive_cmd_sent)
{
	struct mid_pmu_pmu_irq_log *log =
		&mid_pmu_cxt->pmu_irq_log[mid_pmu_cxt->pmu_irq_log_idx];

	log->status = status;
	log->interactive_cmd_sent = interactive_cmd_sent;
	pmu_log_timestamp(&log->ts);
	mid_pmu_cxt->pmu_irq_log_idx =
		(mid_pmu_cxt->pmu_irq_log_idx + 1) % LOG_SIZE;
}

static void pmu_dump_pmu_irq_log(void)
{
	struct mid_pmu_pmu_irq_log *log;
	int i = mid_pmu_cxt->pmu_irq_log_idx, j;

	printk(KERN_ERR"%d last pmu irqs:\n", LOG_SIZE);

	for (j = 0; j  < LOG_SIZE; j++) {
		i ? i-- : (i = LOG_SIZE - 1);
		log = &mid_pmu_cxt->pmu_irq_log[i];
		printk(KERN_ERR"Timestamp: %lu.%09lu\n",
			log->ts.tv_sec, log->ts.tv_nsec);
		printk(KERN_ERR"Status = 0x%02x", log->status);
		printk(KERN_ERR"interactive_cmd_sent = %s\n",
			log->interactive_cmd_sent ? "true" : "false");
		printk(KERN_ERR"\n");
	}
}

void pmu_log_ipc_irq(void)
{
	struct mid_pmu_ipc_irq_log *log =
		&mid_pmu_cxt->ipc_irq_log[mid_pmu_cxt->ipc_irq_log_idx];

	pmu_log_timestamp(&log->ts);
	mid_pmu_cxt->ipc_irq_log_idx =
	(mid_pmu_cxt->ipc_irq_log_idx + 1) % LOG_SIZE;
}

static void pmu_dump_ipc_irq_log(void)
{
	struct mid_pmu_ipc_irq_log *log;
	int i = mid_pmu_cxt->ipc_irq_log_idx, j;

	printk(KERN_ERR"%d last ipc irqs:\n", LOG_SIZE);

	for (j = 0; j  < LOG_SIZE; j++) {
		i ? i-- : (i = LOG_SIZE - 1);
		log = &mid_pmu_cxt->ipc_irq_log[i];
		printk(KERN_ERR"Timestamp: %lu.%09lu\n",
			log->ts.tv_sec, log->ts.tv_nsec);
		printk(KERN_ERR"\n");
	}
}

void pmu_log_ipc(u32 command)
{
	struct mid_pmu_ipc_log *log =
	&mid_pmu_cxt->ipc_log[mid_pmu_cxt->ipc_log_idx];

	log->command = command;
	pmu_log_timestamp(&log->ts);
	mid_pmu_cxt->ipc_log_idx = (mid_pmu_cxt->ipc_log_idx + 1) % LOG_SIZE;
}

static void pmu_dump_ipc_log(void)
{
	struct mid_pmu_ipc_log *log;
	int i = mid_pmu_cxt->ipc_log_idx, j;

	printk(KERN_ERR"%d last ipc commands:\n", LOG_SIZE);

	for (j = 0; j  < LOG_SIZE; j++) {
		i  ? i-- : (i = LOG_SIZE - 1);
		log = &mid_pmu_cxt->ipc_log[i];
		printk(KERN_ERR"Timestamp: %lu.%09lu\n",
			log->ts.tv_sec, log->ts.tv_nsec);
		printk(KERN_ERR"Command: 0x%08x", log->command);
		printk(KERN_ERR"\n");
	}
}

void pmu_log_command(u32 command, struct pmu_ss_states *pm_ssc)
{
	struct mid_pmu_cmd_log *log =
		&mid_pmu_cxt->cmd_log[mid_pmu_cxt->cmd_log_idx];

	if (pm_ssc != NULL)
		memcpy(&log->pm_ssc, pm_ssc, sizeof(struct pmu_ss_states));
	else
		memset(&log->pm_ssc, 0, sizeof(struct pmu_ss_states));
	log->command = command;
	pmu_log_timestamp(&log->ts);
	mid_pmu_cxt->cmd_log_idx = (mid_pmu_cxt->cmd_log_idx + 1) % LOG_SIZE;
}

static void pmu_dump_command_log(void)
{
	struct mid_pmu_cmd_log *log;
	int i = mid_pmu_cxt->cmd_log_idx, j, k;
	u32 cmd_state;
	printk(KERN_ERR"%d last pmu commands:\n", LOG_SIZE);

	for (j = 0; j  < LOG_SIZE; j++) {
		i ? i-- : (i = LOG_SIZE - 1);
		log = &mid_pmu_cxt->cmd_log[i];
		cmd_state = log->command;
		printk(KERN_ERR"Timestamp: %lu.%09lu\n",
			log->ts.tv_sec, log->ts.tv_nsec);
		switch (cmd_state) {
		case INTERACTIVE_VALUE:
			printk(KERN_ERR"PM_CMD = Interactive_CMD IOC bit not set.\n");
			break;
		case INTERACTIVE_IOC_VALUE:
			printk(KERN_ERR"PM_CMD = Interactive_CMD IOC bit set.\n");
			break;
		case S0I1_VALUE:
			printk(KERN_ERR"PM_CMD = S0i1_CMD\n");
			break;
		case S0I3_VALUE:
			printk(KERN_ERR"PM_CMD = S0i3_CMD\n");
			break;
		case LPMP3_VALUE:
			printk(KERN_ERR"PM_CMD = LPMP3_CMD\n");
			break;
		default:
			printk(KERN_ERR "Invalid PM_CMD\n");
			break;
		}
		for (k = 0; k < 4; k++)
			printk(KERN_ERR"pmu2_states[%d]: 0x%08lx\n",
				k, log->pm_ssc.pmu2_states[k]);
			printk(KERN_ERR"\n");
	}
}

void pmu_dump_logs(void)
{
	struct timespec ts;

	pmu_log_timestamp(&ts);
	printk(KERN_ERR"Dumping out pmu logs\n");
	printk(KERN_ERR"Timestamp: %lu.%09lu\n\n", ts.tv_sec, ts.tv_nsec);
	printk(KERN_ERR"---------------------------------------\n\n");
	pmu_dump_command_log();
	printk(KERN_ERR"---------------------------------------\n\n");
	pmu_dump_pmu_irq_log();
	printk(KERN_ERR"---------------------------------------\n\n");
	pmu_dump_ipc_log();
	printk(KERN_ERR"---------------------------------------\n\n");
	pmu_dump_ipc_irq_log();
}
#else
void pmu_log_pmu_irq(int status, bool interactive_cmd_sent) {}
void pmu_log_command(u32 command, struct pmu_ss_states *pm_ssc) {}
void pmu_dump_logs(void) {}
#endif /* LOG_PMU_EVENTS */

void pmu_stat_start(enum sys_state type)
{
	mid_pmu_cxt->pmu_current_state = type;
	mid_pmu_cxt->pmu_stats[type].last_try = cpu_clock(smp_processor_id());
}

void pmu_stat_end(void)
{
	enum sys_state type = mid_pmu_cxt->pmu_current_state;

	if (type > SYS_STATE_S0I0 && type < SYS_STATE_MAX) {
		mid_pmu_cxt->pmu_stats[type].last_entry =
			mid_pmu_cxt->pmu_stats[type].last_try;

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

void pmu_stat_error(u8 err_type)
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
	nanosec_rem = do_div(t, NANO_SEC);

	/* convert time in secs */
	time = (unsigned long)t;

	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t = mid_pmu_cxt->pmu_stats[type].last_entry;
	nanosec_rem = do_div(t, NANO_SEC);
	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t = mid_pmu_cxt->pmu_stats[type].first_entry;
	nanosec_rem = do_div(t, NANO_SEC);
	seq_printf(s, "%5lu.%06lu\t",
	   (unsigned long) t, nanosec_rem / 1000);

	t =  cpu_clock(raw_smp_processor_id());
	t -= mid_pmu_cxt->pmu_init_time;
	nanosec_rem = do_div(t, NANO_SEC);

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

static unsigned long pmu_dev_res_print(int index, unsigned long *precision,
				unsigned long *sampled_time, bool dev_state)
{
	unsigned long long t, delta_time = 0;
	unsigned long nanosec_rem, remainder;
	unsigned long time, init_to_now_time;

	t =  cpu_clock(raw_smp_processor_id());

	if (dev_state) {
		/* print for d0ix */
		if ((mid_pmu_cxt->pmu_dev_res[index].state != PCI_D0))
			delta_time = t -
				mid_pmu_cxt->pmu_dev_res[index].d0i3_entry;

			delta_time += mid_pmu_cxt->pmu_dev_res[index].d0i3_acc;
	} else {
		/* print for d0i0 */
		if ((mid_pmu_cxt->pmu_dev_res[index].state == PCI_D0))
			delta_time = t -
				mid_pmu_cxt->pmu_dev_res[index].d0i0_entry;

		delta_time += mid_pmu_cxt->pmu_dev_res[index].d0i0_acc;
	}

	t -= mid_pmu_cxt->pmu_dev_res[index].start;
	nanosec_rem = do_div(t, NANO_SEC);

	init_to_now_time =  (unsigned long) t;

	t = delta_time;
	nanosec_rem = do_div(t, NANO_SEC);

	/* convert time in secs */
	time = (unsigned long)t;
	*sampled_time = time;

	/* for calculating percentage residency */
	time = time * 100;
	t = (u64) time;

	/* take care of divide by zero */
	if (init_to_now_time) {
		remainder = do_div(t, init_to_now_time);
		time = (unsigned long) t;

		/* for getting 3 digit precision after
		* decimal dot */
		remainder *= 1000;
		t = (u64) remainder;
		remainder = do_div(t, init_to_now_time);
	} else
		time = t = 0;

	*precision = (unsigned long)t;

	return time;
}

static void nc_device_state_show(struct seq_file *s, struct pci_dev *pdev)
{
	int off, i, islands_num, state;
	struct island *islands;

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
				(state >= 0) ? dstates[state & 3] : "ERR");
	}
}

static int pmu_devices_state_show(struct seq_file *s, void *unused)
{
	struct pci_dev *pdev = NULL;
	int index, i, pmu_num, ss_idx, ss_pos;
	unsigned int base_class;
	u32 target_mask, mask, val, needed;
	struct pmu_ss_states cur_pmsss;

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
			dstates[val & 3],
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
			if (lss >= lsses_num)
				entry = &lsses[lsses_num - 1];
			else
				entry = &lsses[lss];
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

static int show_pmu_dev_stats(struct seq_file *s, void *unused)
{
	struct pci_dev *pdev = NULL;
	unsigned long sampled_time, precision;
	int index, pmu_num, ss_idx, ss_pos;
	unsigned int base_class;

	seq_printf(s, "%5s\t%20s\t%10s\t%10s\t%s\n",
		"lss", "Name", "D0_res", "D0ix_res", "Sampled_Time");
	seq_printf(s,
	"==================================================================\n");

	while ((pdev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, pdev)) != NULL) {

		/* find the base class info */
		base_class = pdev->class >> 16;

		if (base_class == PCI_BASE_CLASS_BRIDGE)
			continue;

		if (pmu_pci_to_indexes(pdev, &index, &pmu_num, &ss_idx,
							&ss_pos))
			continue;

		if (pmu_num == PMU_NUM_1) {
			seq_printf(s,
			"%5s%20s\t%5lu.%03lu%%\t%5lu.%03lu%%\t%lu\n",
			"NC", dev_driver_string(&pdev->dev),
			pmu_dev_res_print(index, &precision,
				 &sampled_time, false),
			precision,
			pmu_dev_res_print(index, &precision,
				 &sampled_time, true),
			precision, sampled_time);
			continue;
		}

		/* Print for South Complex devices */
		seq_printf(s, "%5d\t%20s\t%5lu.%03lu%%\t%5lu.%03lu%%\t%lu\n",
		index - mid_pmu_cxt->pmu1_max_devs,
		dev_driver_string(&pdev->dev),
		pmu_dev_res_print(index, &precision, &sampled_time, false),
		precision,
		pmu_dev_res_print(index, &precision, &sampled_time, true),
		precision, sampled_time);
	}
	return 0;
}

static int pmu_dev_stat_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_pmu_dev_stats, NULL);
}

static const struct file_operations pmu_dev_stat_operations = {
	.open		= pmu_dev_stat_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_PM_DEBUG
static int pmu_stats_interval = PMU_LOG_INTERVAL_SECS;
module_param_named(pmu_stats_interval, pmu_stats_interval,
				int, S_IRUGO | S_IWUSR | S_IWGRP);

static int mid_state_to_sys_state(int mid_state)
{
	int sys_state = 0;
	switch (mid_state) {
	case MID_S0I1_STATE:
		sys_state = SYS_STATE_S0I1;
		break;
	case MID_LPMP3_STATE:
		sys_state = SYS_STATE_S0I2;
		break;
	case MID_S0I3_STATE:
		sys_state = SYS_STATE_S0I3;
		break;
	case MID_S3_STATE:
		sys_state = SYS_STATE_S3;
		break;

	case C6_HINT:
		sys_state = SYS_STATE_S0I0;
	}

	return sys_state;
}

void pmu_s0ix_demotion_stat(int req_state, int grant_state)
{
	struct pmu_ss_states cur_pmsss;
	int i, req_sys_state, offset;
	unsigned long status, sub_status;
	unsigned long s0ix_target_sss_mask[4] = {
				S0IX_TARGET_SSS0_MASK,
				S0IX_TARGET_SSS1_MASK,
				S0IX_TARGET_SSS2_MASK,
				S0IX_TARGET_SSS3_MASK};

	unsigned long s0ix_target_sss[4] = {
				S0IX_TARGET_SSS0,
				S0IX_TARGET_SSS1,
				S0IX_TARGET_SSS2,
				S0IX_TARGET_SSS3};

	unsigned long lpmp3_target_sss_mask[4] = {
				LPMP3_TARGET_SSS0_MASK,
				LPMP3_TARGET_SSS1_MASK,
				LPMP3_TARGET_SSS2_MASK,
				LPMP3_TARGET_SSS3_MASK};

	unsigned long lpmp3_target_sss[4] = {
				LPMP3_TARGET_SSS0,
				LPMP3_TARGET_SSS1,
				LPMP3_TARGET_SSS2,
				LPMP3_TARGET_SSS3};

	req_sys_state = mid_state_to_sys_state(req_state);
	if ((grant_state >= C4_STATE_IDX) && (grant_state <= S0I3_STATE_IDX))
		mid_pmu_cxt->pmu_stats
			[req_sys_state].demote_count
				[grant_state-C4_STATE_IDX]++;

	if (down_trylock(&mid_pmu_cxt->scu_ready_sem))
		return;

	pmu_read_sss(&cur_pmsss);
	up(&mid_pmu_cxt->scu_ready_sem);

	if (!mid_pmu_cxt->camera_off)
		mid_pmu_cxt->pmu_stats[req_sys_state].camera_blocker_count++;

	if (!mid_pmu_cxt->display_off)
		mid_pmu_cxt->pmu_stats[req_sys_state].display_blocker_count++;

	if (!mid_pmu_cxt->s0ix_possible) {
		for (i = 0; i < 4; i++) {
			unsigned int lss_per_register;
			if (req_state == MID_LPMP3_STATE)
				status = lpmp3_target_sss[i] ^
					(cur_pmsss.pmu2_states[i] &
						lpmp3_target_sss_mask[i]);
			else
				status = s0ix_target_sss[i] ^
					(cur_pmsss.pmu2_states[i] &
						s0ix_target_sss_mask[i]);
			if (!status)
				continue;

			lss_per_register =
				(sizeof(unsigned long)*8)/BITS_PER_LSS;

			for (offset = 0; offset < lss_per_register; offset++) {
				sub_status = status & SS_IDX_MASK;
				if (sub_status) {
					mid_pmu_cxt->pmu_stats[req_sys_state].
						blocker_count
						[offset + lss_per_register*i]++;
				}

				status >>= BITS_PER_LSS;
			}
		}
	}
}
EXPORT_SYMBOL(pmu_s0ix_demotion_stat);

static void pmu_log_s0ix_status(int type, char *typestr,
		struct seq_file *s, bool logging_type)
{
	unsigned long long t;
	unsigned long time, remainder, init_2_now_time;

	t = mid_pmu_cxt->pmu_stats[type].time;
	remainder = do_div(t, NANO_SEC);

	/* convert time in secs */
	time = (unsigned long)t;

	t =  cpu_clock(0);
	t -= mid_pmu_cxt->pmu_init_time;
	remainder = do_div(t, NANO_SEC);

	init_2_now_time =  (unsigned long) t;

	/* for calculating percentage residency */
	t = (u64) time * 100;

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
	DEBUG_PRINT(logging_type, s, STATS,
			"%s\t%5llu\t%9llu\t%9llu\t%5lu.%03lu\n"
			, typestr, mid_pmu_cxt->pmu_stats[type].count,
			mid_pmu_cxt->pmu_stats[type].err_count[1],
			mid_pmu_cxt->pmu_stats[type].err_count[2],
			time, (unsigned long) t);
}

static void pmu_log_s0ix_demotion(int type, char *typestr,
		struct seq_file *s, bool logging_type)
{
	DEBUG_PRINT(logging_type, s, STATS, "%s:\t%6d\t%6d\t%6d\t%6d\t%6d\n",
		typestr,
		mid_pmu_cxt->pmu_stats[type].demote_count[0],
		mid_pmu_cxt->pmu_stats[type].demote_count[1],
		mid_pmu_cxt->pmu_stats[type].demote_count[2],
		mid_pmu_cxt->pmu_stats[type].demote_count[3],
		mid_pmu_cxt->pmu_stats[type].demote_count[4]);
}

static void pmu_log_s0ix_lss_blocked(int type, char *typestr,
		struct seq_file *s, bool logging_type)
{
	int i, block_count;

	DEBUG_PRINT(logging_type, s, STATS, "%s: Block Count\n", typestr);

	block_count = mid_pmu_cxt->pmu_stats[type].display_blocker_count;

	if (block_count)
		DEBUG_PRINT(logging_type, s, STATS,
			 "\tDisplay blocked: %d times\n", block_count);

	block_count = mid_pmu_cxt->pmu_stats[type].camera_blocker_count;

	if (block_count)
		DEBUG_PRINT(logging_type, s, STATS,
			"\tCamera blocked: %d times\n", block_count);

	DEBUG_PRINT(logging_type, s, STATS, "\tLSS\t #blocked\n");

	for  (i = 0; i < MAX_LSS_POSSIBLE; i++) {
		block_count = mid_pmu_cxt->pmu_stats[type].blocker_count[i];
		if (block_count)
			DEBUG_PRINT(logging_type, s, STATS, "\t%02d\t %6d\n", i,
						block_count);
	}
	DEBUG_PRINT(logging_type, s, STATS, "\n");
}

static void pmu_stats_logger(bool logging_type, struct seq_file *s)
{

	if (!logging_type)
		DEBUG_PRINT(logging_type, s, STATS,
			"\n----MID_PMU_STATS_LOG_BEGIN----\n");

	DEBUG_PRINT(logging_type, s, STATS,
			"\tcount\ts0ix_miss\tno_ack_c6\tresidency(%%)\n");
	pmu_log_s0ix_status(SYS_STATE_S0I1, "s0i1", s, logging_type);
	pmu_log_s0ix_status(SYS_STATE_S0I2, "lpmp3", s, logging_type);
	pmu_log_s0ix_status(SYS_STATE_S0I3, "s0i3", s, logging_type);
	pmu_log_s0ix_status(SYS_STATE_S3, "s3", s, logging_type);

	DEBUG_PRINT(logging_type, s, STATS, "\nFrom:\tTo\n");
	DEBUG_PRINT(logging_type, s, STATS,
		"\t    C4\t   C6\t  S0i1\t  Lpmp3\t  S0i3\n");

	/* storing C6 demotion info in S0I0 */
	pmu_log_s0ix_demotion(SYS_STATE_S0I0, "  C6", s, logging_type);

	pmu_log_s0ix_demotion(SYS_STATE_S0I1, "s0i1", s, logging_type);
	pmu_log_s0ix_demotion(SYS_STATE_S0I2, "lpmp3", s, logging_type);
	pmu_log_s0ix_demotion(SYS_STATE_S0I3, "s0i3", s, logging_type);

	DEBUG_PRINT(logging_type, s, STATS, "\n");
	pmu_log_s0ix_lss_blocked(SYS_STATE_S0I1, "s0i1", s, logging_type);
	pmu_log_s0ix_lss_blocked(SYS_STATE_S0I2, "lpmp3", s, logging_type);
	pmu_log_s0ix_lss_blocked(SYS_STATE_S0I3, "s0i3", s, logging_type);

	if (!logging_type)
		DEBUG_PRINT(logging_type, s, STATS,
				"\n----MID_PMU_STATS_LOG_END----\n");
	DEBUG_PRINT(logging_type, s, STATS, "\n----WAKE_LOCK_INFO_START----\n");
	if (!has_wake_lock(WAKE_LOCK_SUSPEND))
		DEBUG_PRINT(logging_type, s, STATS, "No WAKE LOCK held");
	DEBUG_PRINT(logging_type, s, STATS, "\n----WAKE_LOCK_INFO_END----\n");
}

static void pmu_log_stat(struct work_struct *work)
{

	pmu_stats_logger(false, NULL);

	schedule_delayed_work(&mid_pmu_cxt->log_work,
			msecs_to_jiffies(pmu_stats_interval*1000));
}

static int show_pmu_stats_log(struct seq_file *s, void *unused)
{
	pmu_stats_logger(true, s);
	return 0;
}

static int pmu_stats_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, show_pmu_stats_log, NULL);
}

static const struct file_operations pmu_stats_log_operations = {
	.open		= pmu_stats_log_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

void pmu_stats_init(void)
{
	struct dentry *fentry;

	/* /sys/kernel/debug/mid_pmu_states */
	(void) debugfs_create_file("mid_pmu_states", S_IFREG | S_IRUGO,
				NULL, NULL, &devices_state_operations);

	/* /sys/kernel/debug/pmu_sss_states */
	(void) debugfs_create_file("pmu_sss_states", S_IFREG | S_IRUGO,
				NULL, NULL, &pmu_sss_state_operations);

	/* /sys/kernel/debug/pmu_dev_stats */
	(void) debugfs_create_file("pmu_dev_stats", S_IFREG | S_IRUGO,
				NULL, NULL, &pmu_dev_stat_operations);

#ifdef CONFIG_PM_DEBUG
	/* dynamic debug tracing in every 5 mins */
	INIT_DELAYED_WORK_DEFERRABLE(&mid_pmu_cxt->log_work, pmu_log_stat);
	schedule_delayed_work(&mid_pmu_cxt->log_work,
				msecs_to_jiffies(pmu_stats_interval*1000));

	debug_mask = PMU_DEBUG_PRINT_STATS;

	/* /sys/kernel/debug/pmu_stats_log */
	fentry = debugfs_create_file("pmu_stats_log", S_IFREG | S_IRUGO,
				NULL, NULL, &pmu_stats_log_operations);
	if (fentry == NULL)
		printk(KERN_ERR "Failed to create pmu_stats_log debugfs\n");

#endif
}

void pmu_stats_finish(void)
{
#ifdef CONFIG_PM_DEBUG
	cancel_delayed_work_sync(&mid_pmu_cxt->log_work);
#endif
}
