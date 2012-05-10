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


void pmu_stats_init(void)
{
	/* /sys/kernel/debug/mid_pmu_states */
	(void) debugfs_create_file("mid_pmu_states", S_IFREG | S_IRUGO,
				NULL, NULL, &devices_state_operations);

	/* /sys/kernel/debug/pmu_sss_states */
	(void) debugfs_create_file("pmu_sss_states", S_IFREG | S_IRUGO,
				NULL, NULL, &pmu_sss_state_operations);
}
