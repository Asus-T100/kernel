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

#ifdef CONFIG_BOARD_MRFLD_VV

#define MRFLD_GPIO_SDIO_0_CD 77
#define MRFLD_GPIO_SDIO_0_CLK 78
#define MRFLD_GPIO_SDIO_0_CMD 79
#define MRFLD_GPIO_SDIO_0_DATA_0 80
#define MRFLD_GPIO_SDIO_0_DATA_1 81
#define MRFLD_GPIO_SDIO_0_DATA_2 82
#define MRFLD_GPIO_SDIO_0_DATA_3 83
#define MRFLD_GPIO_SDIO_0_LVL_CLK_FB 84
#define MRFLD_GPIO_SDIO_0_LVL_CMD_DIR 85
#define MRFLD_GPIO_SDIO_0_LVL_DAT_DIR 86
#define MRFLD_GPIO_SDIO_0_LVL_EN 87
#define MRFLD_GPIO_SDIO_0_LVL_SEL 88
#define MRFLD_GPIO_SDIO_0_WP 89

#define MRFLD_PMIC_VLDOCNT 0xaf
#define MRFLD_PMIC_VLDOCNT_VSWITCH_BIT 0x02

#endif

#endif
