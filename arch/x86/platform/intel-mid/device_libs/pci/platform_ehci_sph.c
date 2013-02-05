
/* platform_ehci_sph_pci.c: USB EHCI/SPH platform data initilization file
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
#include <asm/intel-mid.h>
#include <linux/usb/ehci_sph_pci.h>

static struct ehci_sph_pdata sph_pdata = {
	.has_gpio = 0,
	.gpio_cs_n = 0,
	.gpio_reset_n = 0
};

static int is_board_ctp_prx(void)
{
	return  INTEL_MID_BOARD(1, PHONE, CLVTP) &&
		(SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR01)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR02)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10PM) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10P)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR15M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR15)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR30M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR30)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20A)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR19M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR199M) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20B));
}

static struct ehci_sph_pdata *get_sph_platform_data(struct pci_dev *pdev)
{
	struct ehci_sph_pdata *pdata = &sph_pdata;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_CLV_SPH:
		if (is_board_ctp_prx()) {
			pdata->has_gpio = 1;
			/* FIXME: This GPIO pin number should be
			 * inquired by name, but not available for SPH
			 */
			pdata->gpio_cs_n = 51;
			pdata->gpio_reset_n = 169;
		}
		break;

	default:
		return NULL;
		break;
	}

	return pdata;
}

static void __devinit sph_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_sph_platform_data(pci_dev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_SPH,
			sph_pci_early_quirks);
