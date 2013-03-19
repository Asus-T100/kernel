/*
 * vlv2.c: Intel ValleyView2 platform specific setup code
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Bin Gao <bin.gao@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <asm/setup.h>
#include <asm/time.h>
#include <asm/i8259.h>
#include <asm/intel-mid.h>
#include <asm/processor.h>
#include <asm/pci_x86.h>

#define ILB_BASE	0xfed08000
#define ILB_SIZE	0xa0
#define ILB_IR		0x20
#define PCI_DEV_NUM(x)	((x >> 3) & 0x1f)
#define IR_INTA(ir)	(ir & 0xf)

#define PIRQ2IRQ(x)	(x + 16)

extern struct legacy_pic default_legacy_pic;

/* Propagate PCI IRQ# */
static int vlv2_pci_enable_irq(struct pci_dev *pdev)
{

	u8 ir_val, dev;
	void __iomem *ilb_mem;
	struct io_apic_irq_attr irq_attr;

	ilb_mem = ioremap_nocache(ILB_BASE, ILB_SIZE);
	if (ilb_mem == NULL) {
		pr_err("%s(): can't map ILB_BASE(0x%x)\n",
			__func__, ILB_BASE);
		return -EIO;
	}

	dev = PCI_DEV_NUM(pdev->devfn);
	ir_val = ioread8(ilb_mem + ILB_IR + PCI_DEV_NUM(pdev->devfn) * 2);

	iounmap(ilb_mem);

	/* map INTA# only */
	pdev->irq = PIRQ2IRQ(IR_INTA(ir_val));
	pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, pdev->irq);

	irq_attr.ioapic = mp_find_ioapic(pdev->irq);
	irq_attr.ioapic_pin = pdev->irq;
	irq_attr.trigger = 1; /* level */
	irq_attr.polarity = 1; /* active low */
	io_apic_set_pci_routing(&pdev->dev, pdev->irq, &irq_attr);

	return 0;
}

/*
 * ACPI DSDT table doesn't have correct PCI interrupt routing information
 * for some devices, so here we have a kernel workaround to fix this issue.
 * The workaround can be easily removed by not appending "vlv2" paramerter
 * to kernel command line.
 */
static int __init vlv2_pci_irq_fixup(void)
{
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		pr_info("VLV2: fix up PCI interrupt routing\n");
		pcibios_enable_irq = vlv2_pci_enable_irq;
	}

	return 0;
}
fs_initcall(vlv2_pci_irq_fixup);

static unsigned long __init vlv2_calibrate_tsc(void)
{
	unsigned long fast_calibrate;
	u32 lo, hi, ratio, fsb, bus_freq;

	/* *********************** */
	/* Compute TSC:Ratio * FSB */
	/* *********************** */

	/* Compute Ratio */
	rdmsr(MSR_PLATFORM_INFO, lo, hi);
	pr_debug("IA32 PLATFORM_INFO is 0x%x : %x\n", hi, lo);

	ratio = (lo >> 8) & 0xFF;
	pr_debug("ratio is %d\n", ratio);
	if (!ratio) {
		pr_err("Read a zero ratio, force tsc ratio to 4 ...\n");
		ratio = 4;
	}

	/* Compute FSB */
	rdmsr(MSR_FSB_FREQ, lo, hi);
	pr_debug("Actual FSB frequency detected by SOC 0x%x : %x\n",
		hi, lo);

	bus_freq = lo & 0x7;
	pr_debug("bus_freq = 0x%x\n", bus_freq);

	if (bus_freq == 0)
		fsb = FSB_FREQ_100SKU;
	else if (bus_freq == 1)
		fsb = FSB_FREQ_100SKU;
	else if (bus_freq == 2)
		fsb = FSB_FREQ_133SKU;
	else if (bus_freq == 3)
		fsb = FSB_FREQ_167SKU;
	else if (bus_freq == 4)
		fsb = FSB_FREQ_83SKU;
	else if (bus_freq == 5)
		fsb = FSB_FREQ_400SKU;
	else if (bus_freq == 6)
		fsb = FSB_FREQ_267SKU;
	else if (bus_freq == 7)
		fsb = FSB_FREQ_333SKU;
	else {
		BUG();
		pr_err("Invalid bus_freq! Setting to minimal value!\n");
		fsb = FSB_FREQ_100SKU;
	}

	/* TSC = FSB Freq * Resolved HFM Ratio */
	fast_calibrate = ratio * fsb;
	pr_debug("calculate tangier tsc %lu KHz\n", fast_calibrate);

	/* ************************************ */
	/* Calculate Local APIC Timer Frequency */
	/* ************************************ */
	lapic_timer_frequency = (fsb * 1000) / HZ;

	pr_debug("Setting lapic_timer_frequency = %d\n",
		lapic_timer_frequency);

	/* mark tsc clocksource as reliable */
	set_cpu_cap(&boot_cpu_data, X86_FEATURE_TSC_RELIABLE);

	if (fast_calibrate)
		return fast_calibrate;

	return 0;
}

/* Restore x86_init to default */
static void __init valleyview2_arch_setup(void)
{
	x86_init.resources.reserve_resources = reserve_standard_io_resources;

	x86_init.irqs.pre_vector_init		= init_ISA_irqs;

	x86_init.timers.setup_percpu_clockev	= setup_boot_APIC_clock;
	x86_init.timers.timer_init		= hpet_time_init;
	x86_init.timers.wallclock_init		= x86_init_noop;

	x86_init.pci.init			= x86_default_pci_init;
	x86_init.pci.fixup_irqs			= x86_default_pci_fixup_irqs;

	x86_cpuinit.setup_percpu_clockev	= setup_secondary_APIC_clock;

	x86_platform.calibrate_tsc		= vlv2_calibrate_tsc;

	legacy_pic				= &default_legacy_pic;
}

static struct intel_mid_ops __initdata valleyview2_ops = {
	.arch_setup = valleyview2_arch_setup,
};

void __init *get_valleyview2_ops(void)
{
	return &valleyview2_ops;
}
