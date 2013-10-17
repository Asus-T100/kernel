
/* platform_txe.c: TXE platform quirk file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>

static void __devinit txe_pci_early_quirks(struct pci_dev *pci_dev)
{
	dev_info(&pci_dev->dev, "txe: set run wake flag\n");
	device_set_run_wake(&pci_dev->dev, true);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, 0x0F18, txe_pci_early_quirks);
