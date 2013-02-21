/*
 * platform_byt_audio.c: Baytrail audio platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Omair Md Abudllah <omair.m.abdullah@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/sfi.h>
#include <linux/platform_device.h>
#include <asm/intel-mid.h>
#include <asm/platform_sst_audio.h>
#include <asm/platform_byt_audio.h>

#define SST_IRQ	29

static struct byt_audio_platform_data byt_audio_pdata = {
	.spid = &spid,
};

static inline int byt_program_ioapic(int irq, int trigger, int polarity)
{
	struct io_apic_irq_attr irq_attr;
	int ioapic;

	ioapic = mp_find_ioapic(irq);
	if (ioapic < 0)
		return -EINVAL;
	irq_attr.ioapic = ioapic;
	irq_attr.ioapic_pin = irq;
	irq_attr.trigger = trigger;
	irq_attr.polarity = polarity;
	return io_apic_set_pci_routing(NULL, irq, &irq_attr);
}

static int __init byt_audio_platform_init(void)
{
	struct platform_device *pdev;
	int ret;

	pr_debug("in %s\n", __func__);

	ret = add_sst_platform_device();
	if (ret < 0) {
		pr_err("%s failed to sst_platform device\n", __func__);
		return 0;
	}

	/* configure SST IRQ */
	ret = byt_program_ioapic(SST_IRQ, 0, 1);
	if (ret) {
		pr_err("%s: ioapic programming failed", __func__);
		return 0;
	}

	pdev = platform_device_alloc("byt_rt5642", -1);
	if (!pdev) {
		pr_err("failed to allocate byt_rt5642 platform device\n");
		return 0;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add byt_rt5642 platform device\n");
		platform_device_put(pdev);
		return 0;
	}
	if (platform_device_add_data(pdev, &byt_audio_pdata,
				     sizeof(byt_audio_pdata))) {
		pr_err("failed to add byt_rt5642 platform data\n");
		platform_device_put(pdev);
		return 0;
	}

	pdev = platform_device_alloc("hdmi-audio", -1);
	if (!pdev) {
		pr_err("failed to allocate hdmi-audio platform device\n");
		return 0;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add hdmi-audio platform device\n");
		platform_device_put(pdev);
		return 0;
	}

	return 0;
}
device_initcall(byt_audio_platform_init);
