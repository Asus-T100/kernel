/*
 * platform_mmc_sdhci_pci.h: mmc sdhci pci platform data header file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_MMC_SDHCI_PCI_H_
#define _PLATFORM_MMC_SDHCI_PCI_H_

int bcmdhd_get_sdhci_quirk(void) __attribute__((weak));
int bcmdhd_get_sdhci_mmc_caps(void) __attribute__((weak));
void bcmdhd_register_embedded_control(void *dev_id, void (*virtual_cd)
		(void *dev_id, int card_present)) __attribute__((weak));

#endif
