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
//<asus-baron20131130+>
/*
#define ILB_BASE	0xfed08000
#define ILB_SIZE	0xa0
#define ILB_IR		0x20
#define PCI_DEV_NUM(x)	((x >> 3) & 0x1f)
#define IR_INTA(ir)	(ir & 0xf)

#define PIRQ2IRQ(x)	(x + 16)
*/
//<asus-baron20131130->
extern struct legacy_pic default_legacy_pic;

/* Propagate PCI IRQ# */
static int vlv2_pci_enable_irq(struct pci_dev *pdev)
{

//<asus-baron20131130->	u8 ir_val, dev;
//<asus-baron20131130->	void __iomem *ilb_mem;
	struct io_apic_irq_attr irq_attr;

//<asus-baron20131130+>
/*
	ilb_mem = ioremap_nocache(ILB_BASE, ILB_SIZE);
	if (ilb_mem == NULL) {
		pr_err("%s(): can't map ILB_BASE(0x%x)\n",
			__func__, ILB_BASE);
		return -EIO;
	}

	dev = PCI_DEV_NUM(pdev->devfn);
	ir_val = ioread8(ilb_mem + ILB_IR + PCI_DEV_NUM(pdev->devfn) * 2);

	iounmap(ilb_mem);
*/
	/* map INTA# only */
/*
	pdev->irq = PIRQ2IRQ(IR_INTA(ir_val));
	pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, pdev->irq);
*/
//<asus-baron20131130->
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

	x86_platform.calibrate_tsc		= native_calibrate_tsc;

	legacy_pic				= &default_legacy_pic;
}

static struct intel_mid_ops __initdata valleyview2_ops = {
	.arch_setup = valleyview2_arch_setup,
};

void __init *get_valleyview2_ops(void)
{
	return &valleyview2_ops;
}
