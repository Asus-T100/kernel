/*
 * intel_soc_pmc.c - This driver provides interface to configure the Power
 * Management Controller (PMC).
 * Copyright (c) 2013, Intel Corporation.
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

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/acpi.h>
#include <linux/pci.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/suspend.h>
#include <linux/intel_mid_pm.h>
#include <linux/time.h>

#include "intel_soc_pmc.h"


u32 residency_total;

char *states[] = {
	"S0IR",
	"S0I1",
	"S0I2",
	"S0I3",
	"S0",
	"S3"};

struct mid_pmc_dev *mid_pmc_cxt;
static char *dstates[] = {"D0", "D0i1", "D0i2", "D0i3"};
struct nc_device {
	char *name;
	int reg;
	int sss_pos;
} nc_devices[] = {
	{ "GFX RENDER", PWRGT_STATUS,  RENDER_POS },
	{ "GFX MEDIA", PWRGT_STATUS, MEDIA_POS },
	{ "DISPLAY", PWRGT_STATUS,  DISPLAY_POS },
	{ "VED", VED_SS_PM0, SSS_SHIFT},
	{ "ISP", ISP_SS_PM0, SSS_SHIFT},
	{ "MIO", MIO_SS_PM, SSS_SHIFT},
};

static int no_of_nc_devices = sizeof(nc_devices)/sizeof(nc_devices[0]);

static int byt_wait_for_nc_pmcmd_complete(int verify_mask,
				int status_mask, int state_type , int reg)
{
	int pwr_sts;
	int count = 0;

	while (true) {
		if (reg == PWRGT_CNT)
			pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT,
							PWRGT_STATUS);
		else {
			pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
			pwr_sts = pwr_sts >> SSS_SHIFT;
		}
		if (state_type == OSPM_ISLAND_DOWN ||
				state_type == OSPM_ISLAND_SR) {
			if ((pwr_sts & status_mask) ==
					(verify_mask & status_mask))
				break;
			else
				usleep_range(10, 20);
		} else if (state_type == OSPM_ISLAND_UP) {
			if ((~pwr_sts & status_mask)  ==
					(~verify_mask & status_mask))
				break;
			else
				usleep_range(10, 20);
		}

		count++;
		if (WARN_ONCE(count > 500000, "Timed out waiting for P-Unit"))
			return -EBUSY;
	}
	return 0;
}

int byt_pmu_nc_get_power_state(int islands, int reg)
{
	int pwr_sts, i, lss, ret = 0;

	if (unlikely(!mid_pmc_cxt))
		return -EAGAIN;

	might_sleep();

	down(&mid_pmc_cxt->nc_ready_lock);

	pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
	if (reg != PWRGT_STATUS)
		pwr_sts = pwr_sts >> SSS_SHIFT;

	for (i = 0; i < OSPM_MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			ret = (pwr_sts >> (BITS_PER_LSS * i)) & D0I3_MASK;
			break;
		}
	}

	up(&mid_pmc_cxt->nc_ready_lock);
	return ret;
}

int byt_pmu_nc_set_power_state(int islands, int state_type, int reg)
{
	u32 pwr_sts = 0;
	u32 pwr_mask = 0;
	int i, lss, mask;
	int ret = 0;
	int status_mask = 0;

	if (unlikely(!mid_pmc_cxt))
		return -EAGAIN;

	might_sleep();

	down(&mid_pmc_cxt->nc_ready_lock);

	pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
	pwr_mask = pwr_sts;

	for (i = 0; i < OSPM_MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			mask = D0I3_MASK << (BITS_PER_LSS * i);
			status_mask = status_mask | mask;
			if (state_type == OSPM_ISLAND_DOWN)
				pwr_mask |= mask;
			else if (state_type == OSPM_ISLAND_UP)
				pwr_mask &= ~mask;
			/* Soft reset case */
			else if (state_type == OSPM_ISLAND_SR) {
				pwr_mask &= ~mask;
				mask = SR_MASK << (BITS_PER_LSS * i);
				 pwr_mask |= mask;
			}
		}
	}

	intel_mid_msgbus_write32(PUNIT_PORT, reg, pwr_mask);
	ret = byt_wait_for_nc_pmcmd_complete(pwr_mask,
				status_mask, state_type, reg);

	up(&mid_pmc_cxt->nc_ready_lock);
	return ret;
}

static u32 pmc_register_read(int reg_offset)
{
	return readl(mid_pmc_cxt->pmc_registers + reg_offset);
}

static void print_residency_per_state(struct seq_file *s, int state, u32 count)
{
	u32 rem_time, rem_res = 0;
	u64 rem_res_reduced = 0;
	/* Counter increments every 32 us. */
	u64 time = (u64)count << 5;
	u64 residency = (u64)count * 100;
	if (residency_total) {
		rem_res = do_div(residency, residency_total);
		rem_res_reduced = (u64)rem_res * 1000;
		do_div(rem_res_reduced, residency_total);
	}
	rem_time = do_div(time, USEC_PER_SEC);
	seq_printf(s, "%s \t\t %.6llu.%.6u \t\t %.2llu.%.3llu", states[state],
			time, rem_time, residency, rem_res_reduced);
	if (state == MAX_PLATFORM_STATES)
		seq_printf(s, " \t\t %u\n", mid_pmc_cxt->s3_count);
	else
		seq_printf(s, " \t\t %s\n", "--");
}

static int pmu_devices_state_show(struct seq_file *s, void *unused)
{
	int i;
	u32 val, nc_pwr_sts, reg;
	unsigned int base_class, sub_class;
	struct pci_dev *dev = NULL;
	u16 pmcsr;
	u32 s0ix_residency[MAX_PLATFORM_STATES];

	residency_total = 0;
	/* Read s0ix residency counters */
	for (i = 0; i < MAX_PLATFORM_STATES; i++) {
		s0ix_residency[i] = pmc_register_read(i);
		residency_total += s0ix_residency[i];
	}
	s0ix_residency[S0I3] -= mid_pmc_cxt->s3_residency;

	seq_printf(s, "State \t\t Time[sec] \t\t Residency[%%] \t\t Count\n");
	for (i = 0; i < MAX_PLATFORM_STATES; i++)
		print_residency_per_state(s, i, s0ix_residency[i]);
	print_residency_per_state(s, i, mid_pmc_cxt->s3_residency);

	seq_printf(s, "\n\nNORTH COMPLEX DEVICES :\n");

	for (i = 0; i < no_of_nc_devices; i++) {
		reg = nc_devices[i].reg;
		nc_pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
		nc_pwr_sts >>= nc_devices[i].sss_pos;
		val = nc_pwr_sts & D0I3_MASK;
		seq_printf(s, "%9s : %s\n", nc_devices[i].name, dstates[val]);
	}

	seq_printf(s, "\nSOUTH COMPLEX DEVICES :\n");

	while ((dev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, dev)) != NULL) {
		/* find the base class info */
		base_class = dev->class >> 16;
		sub_class  = (dev->class & SUB_CLASS_MASK) >> 8;

		if (base_class == PCI_BASE_CLASS_BRIDGE)
			continue;

		if ((base_class == PCI_BASE_CLASS_DISPLAY) && !sub_class)
			continue;

		if ((base_class == PCI_BASE_CLASS_MULTIMEDIA) &&
				(sub_class == ISP_SUB_CLASS))
			continue;

		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &pmcsr);
		val = pmcsr & D0I3_MASK;
		seq_printf(s, "%9s %15s : %s\n", dev_name(&dev->dev),
			dev_driver_string(&dev->dev), dstates[val]);
	}

	seq_printf(s, "\n");

	return 0;
}


static int devices_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmu_devices_state_show, NULL);
}

static const struct file_operations devices_state_operations = {
	.open           = devices_state_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int nc_set_power_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t nc_set_power_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	int islands, state, reg, buf_size;
	struct pci_dev *dev = NULL;
	u16 pmcsr, val;

	buf_size = count < 64 ? count : 64;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%d %d %d", &islands, &state, &reg) != 3)
		return -EFAULT;

	if (!islands) {
		while ((dev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, dev))
								!= NULL) {
			pci_read_config_word(dev, dev->pm_cap +
							PCI_PM_CTRL, &pmcsr);
			val = pmcsr & D0I3_MASK;
			if (!val) {
				pmcsr |= D0I3_MASK;
				pci_write_config_word(dev, dev->pm_cap +
							PCI_PM_CTRL, pmcsr);
			}
		}
		return count;
	}

	pmu_nc_set_power_state(islands, state, reg);
	return count;
}

static int nc_set_power_open(struct inode *inode, struct file *file)
{
	return single_open(file, nc_set_power_show, NULL);
}

static const struct file_operations nc_set_power_operations = {
	.open           = nc_set_power_open,
	.read           = seq_read,
	.write          = nc_set_power_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void update_all_pci_devices(void)
{
	struct pci_dev *pdev = NULL;
	u16 pmcsr;

	while ((pdev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, pdev))
							!= NULL) {
		pci_read_config_word(pdev, pdev->pm_cap +
						PCI_PM_CTRL, &pmcsr);

		/* In case, device doesn't have driver and it's in D0,
		 * put it in D0i3 */
		if (IS_ERR_OR_NULL(pdev->dev.driver) && !(pmcsr & D0I3_MASK)) {
			dev_info(&pdev->dev, "put device in D0i3\n");
			pmcsr |= D0I3_MASK;
			pci_write_config_word(pdev, pdev->pm_cap +
						PCI_PM_CTRL, pmcsr);
		}
	}
}

static int mid_suspend_begin(suspend_state_t state)
{
	return 0;
}

static int mid_suspend_valid(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_ON:
	case PM_SUSPEND_MEM:
		ret = 1;
		break;
	}
	return ret;
}

static int mid_suspend_prepare(void)
{
	update_all_pci_devices();
	return 0;
}

static int mid_suspend_prepare_late(void)
{
	return 0;
}

static int mid_suspend_enter(suspend_state_t state)
{
	u32 temp = 0, count_before_entry, count_after_exit;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;

	count_before_entry = pmc_register_read(S0I3);
	trace_printk("s3_entry\n");

	__monitor((void *)&temp, 0, 0);
	smp_mb();
	__mwait(BYT_S3_HINT, 1);

	trace_printk("s3_exit\n");
	mid_pmc_cxt->s3_count += 1;
	count_after_exit = pmc_register_read(S0I3);
	mid_pmc_cxt->s3_residency += (count_after_exit - count_before_entry);
	return 0;
}


static void mid_suspend_end(void)
{
	return;
}

static const struct platform_suspend_ops mid_suspend_ops = {
	.begin = mid_suspend_begin,
	.valid = mid_suspend_valid,
	.prepare = mid_suspend_prepare,
	.prepare_late = mid_suspend_prepare_late,
	.enter = mid_suspend_enter,
	.end = mid_suspend_end,
};

static int byt_pmu_init(void)
{
	suspend_set_ops(&mid_suspend_ops);

	sema_init(&mid_pmc_cxt->nc_ready_lock, 1);

	/* /sys/kernel/debug/mid_pmu_states */
	(void) debugfs_create_file("mid_pmu_states", S_IFREG | S_IRUGO,
				NULL, NULL, &devices_state_operations);

	/* /sys/kernel/debug/mid_pmu_states */
	(void) debugfs_create_file("nc_set_power", S_IFREG | S_IRUGO,
				NULL, NULL, &nc_set_power_operations);

	cstate_ignore_add_init();

	writel(DISABLE_LPC_CLK_WAKE_EN, mid_pmc_cxt->s0ix_wake_en);
	return 0;
}

static DEFINE_PCI_DEVICE_TABLE(pmc_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0F1C)},
	{0,}
};
MODULE_DEVICE_TABLE(pci, pmc_pci_tbl);

static int __devinit pmc_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	int error = 0;

	mid_pmc_cxt = kzalloc(sizeof(struct mid_pmc_dev), GFP_KERNEL);

	if (unlikely(!mid_pmc_cxt)) {
		pr_err("Failed to allocate memory for mid_pmc_cxt.\n");
		error = -ENOMEM;
		goto exit_err0;
	}

	mid_pmc_cxt->pdev = pdev;

	if (pci_enable_device(pdev)) {
		pr_err("Failed to initialize PMC as PCI device\n");
		error = -EFAULT;
		goto exit_err1;
	}

	pci_read_config_dword(pdev, PCI_CB_LEGACY_MODE_BASE,
					&mid_pmc_cxt->base_address);
	mid_pmc_cxt->base_address &= BASE_ADDRESS_MASK ;

	if (pci_request_region(pdev, PMC_MMIO_BAR, "pmc_driver")) {
		pr_err("Failed to allocate requested PCI region\n");
		error = -EFAULT;
		goto exit_err1;
	}

	mid_pmc_cxt->pmc_registers = ioremap_nocache(
		mid_pmc_cxt->base_address + S0IX_REGISTERS_OFFSET, 20);

	mid_pmc_cxt->s0ix_wake_en = ioremap_nocache(
		mid_pmc_cxt->base_address + S0IX_WAKE_EN, 4);

	if (unlikely(!mid_pmc_cxt->pmc_registers ||
				!mid_pmc_cxt->s0ix_wake_en)) {
		pr_err("Failed to map PMC registers.\n");
		error = -EFAULT;
		goto exit_err1;
	}

	mid_pmc_cxt->s3_residency = 0;

	error = byt_pmu_init();
	if (error) {
		pr_err("Unable to register acpi_pmu driver.\n");
		goto exit_err2;
	}

	return 0;

exit_err2:
	iounmap(mid_pmc_cxt->pmc_registers);
	iounmap(mid_pmc_cxt->s0ix_wake_en);
exit_err1:
	kfree(mid_pmc_cxt);
	mid_pmc_cxt = NULL;
exit_err0:
	pr_err("%s: Initialization failed\n", __func__);
	return error;
}

static struct pci_driver pmc_pci_driver = {
	.name = "pmc",
	.id_table = pmc_pci_tbl,
	.probe = pmc_pci_probe,
};

static int __init pmc_init(void)
{
	pr_info("Initializing PMC module\n");
	return pci_register_driver(&pmc_pci_driver);
}

static void __exit pmc_exit(void)
{

	pr_info("Exiting PMC module\n");
	pci_unregister_driver(&pmc_pci_driver);
}

module_init(pmc_init);
module_exit(pmc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel ATOM Platform Power Management Controller (PMC) Driver");
