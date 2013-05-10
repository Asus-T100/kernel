/*
 * platform_mmc_sdhci_pci.c: mmc sdhci pci platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/intel-mid.h>
#include <linux/mmc/sdhci-pci-data.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include <linux/intel_mid_pm.h>
#include <linux/hardirq.h>
#include <linux/mmc/sdhci.h>

#include "platform_sdhci_pci.h"

static int panic_mode_emmc0_power_up(void *data);
static int mrfl_sd_setup(struct sdhci_pci_data *data);
static void mrfl_sd_cleanup(struct sdhci_pci_data *data);
static int mrfl_sdio_setup(struct sdhci_pci_data *data);
static void mrfl_sdio_cleanup(struct sdhci_pci_data *data);

static void (*sdhci_embedded_control)(void *dev_id, void (*virtual_cd)
					(void *dev_id, int card_present));

static unsigned int sdhci_pdata_quirks;

/* MFLD platform data */
static struct sdhci_pci_data mfld_sdhci_pci_data[] = {
	[EMMC0_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = panic_mode_emmc0_power_up,
	},
	[EMMC1_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = 0,
	},
	[SD_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = 69,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = 0,
	},
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = 0,
	},
};

#define VCCSDIO_ADDR		0xd5
#define VCCSDIO_OFF		0x4
#define VR_REG_MASK             0x7

static int clv_sd_setup(struct sdhci_pci_data *data)
{
	int err;
	u16 addr;
	u8 value;

	addr = VCCSDIO_ADDR;
	err = intel_scu_ipc_readv(&addr, &value, 1);
	if (err) {
		pr_err("%s: read VCCSDIO status failed\n", __func__);
		/*
		 * In this case, assume the VCCSDIO is on, and let driver
		 * try to see if the SD card can be detected
		 */
		return 0;
	}

	if ((value & VR_REG_MASK) == VCCSDIO_OFF)
		data->platform_quirks |=  PLFM_QUIRK_NO_SDCARD_SLOT;

	return 0;
}

/* CLV platform data */
static struct sdhci_pci_data clv_sdhci_pci_data[] = {
	[EMMC0_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = panic_mode_emmc0_power_up,
	},
	[EMMC1_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = 0,
	},
	[SD_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = 69,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = clv_sd_setup,
			.cleanup = 0,
			.power_up = 0,
	},
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = 0,
	},
};

static int panic_mode_emmc0_power_up(void *data)
{
	int ret;
	bool atomic_context;
	/*
	 * Since pmu_set_emmc_to_d0i0_atomic function can
	 * only be used in atomic context, before call this
	 * function, do a check first and make sure this function
	 * is used in atomic context.
	 */
	atomic_context = (!preemptible() || in_atomic_preempt_off());

	if (!atomic_context) {
		pr_err("%s: not in atomic context!\n", __func__);
		return -EPERM;
	}

	ret = pmu_set_emmc_to_d0i0_atomic();
	if (ret) {
		pr_err("%s: power up host failed with err %d\n",
				__func__, ret);
	}

	return ret;
}

static int dummy_emmc0_power_up(void *data)
{
	pr_err("dummy_emmc0_power_up has been called!\n");
	return 0;
}

/* MRFL platform data */
static struct sdhci_pci_data mrfl_sdhci_pci_data[] = {
	[EMMC0_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = panic_mode_emmc0_power_up,
	},
	[EMMC1_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = 97,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
			.power_up = 0,
	},
	[SD_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = 77,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = mrfl_sd_setup,
			.cleanup = mrfl_sd_cleanup,
			.power_up = 0,
	},
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = mrfl_sdio_setup,
			.cleanup = mrfl_sdio_cleanup,
			.power_up = 0,
	},
};

/* Board specific setup related to SD goes here */
static int mrfl_sd_setup(struct sdhci_pci_data *data)
{
	u8 vldocnt = 0;
	int err;

	/*
	 * Change necessary GPIO pin mode for SD card working.
	 * This is something should be done in IA firmware.
	 * But, anyway, just do it here in case IA firmware
	 * forget to do so.
	 */
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_CD, 0);

	err = intel_scu_ipc_ioread8(MRFLD_PMIC_VLDOCNT, &vldocnt);
	if (err) {
		printk(KERN_ERR "PMIC vldocnt IPC read error: %d\n", err);
		return err;
	}

	vldocnt |= MRFLD_PMIC_VLDOCNT_VSWITCH_BIT;
	err = intel_scu_ipc_iowrite8(MRFLD_PMIC_VLDOCNT, vldocnt);
	if (err) {
		printk(KERN_ERR "PMIC vldocnt IPC write error: %d\n", err);
		return err;
	}
	msleep(20);

	return 0;
}

/* Board specific cleanup related to SD goes here */
static void mrfl_sd_cleanup(struct sdhci_pci_data *data)
{
}

/* Board specific setup related to SDIO goes here */
static int mrfl_sdio_setup(struct sdhci_pci_data *data)
{
	return 0;
}

/* Board specific cleanup related to SDIO goes here */
static void mrfl_sdio_cleanup(struct sdhci_pci_data *data)
{
}

static int byt_sd_setup(struct sdhci_pci_data *data)
{
	u32 stepping;
	stepping = intel_mid_soc_stepping();
	if (stepping == 0x1 || stepping == 0x2)/* VLV2 A0 */
		data->quirks |= SDHCI_QUIRK2_NO_1_8_V;
	return 0;
}


static int byt_sdio_setup(struct sdhci_pci_data *data)
{
	u32 stepping;
	stepping = intel_mid_soc_stepping();
	if (stepping == 0x1 || stepping == 0x2)/* VLV2 A0 */
		data->quirks |= SDHCI_QUIRK2_NO_1_8_V;
	return 0;
}


/* BYT platform data */
static struct sdhci_pci_data byt_sdhci_pci_data[] = {
	[EMMC0_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = 0,
			.cleanup = 0,
	},
	[SD_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = byt_sd_setup,
			.cleanup = NULL,
			.power_up = 0,
	},
	[SDIO_INDEX] = {
			.pdev = NULL,
			.slotno = 0,
			.rst_n_gpio = -EINVAL,
			.cd_gpio = -EINVAL,
			.quirks = 0,
			.platform_quirks = 0,
			.setup = byt_sdio_setup,
			.cleanup = NULL,
			.power_up = 0,
	},
};

static struct sdhci_pci_data *get_sdhci_platform_data(struct pci_dev *pdev)
{
	struct sdhci_pci_data *pdata = NULL;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_MFD_EMMC0:
		pdata = &mfld_sdhci_pci_data[EMMC0_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_MFD_EMMC1:
		pdata = &mfld_sdhci_pci_data[EMMC1_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_MFD_SD:
		pdata = &mfld_sdhci_pci_data[SD_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_MFD_SDIO1:
		pdata = &mfld_sdhci_pci_data[SDIO_INDEX];
		pdata->quirks = sdhci_pdata_quirks;
		pdata->register_embedded_control = sdhci_embedded_control;
		break;
	case PCI_DEVICE_ID_INTEL_CLV_EMMC0:
		pdata = &clv_sdhci_pci_data[EMMC0_INDEX];
		pdata->rst_n_gpio = get_gpio_by_name("emmc0_rst");
		break;
	case PCI_DEVICE_ID_INTEL_CLV_EMMC1:
		pdata = &clv_sdhci_pci_data[EMMC1_INDEX];
		pdata->rst_n_gpio = get_gpio_by_name("emmc1_rst");
		break;
	case PCI_DEVICE_ID_INTEL_CLV_SDIO0:
		pdata = &clv_sdhci_pci_data[SD_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_CLV_SDIO1:
		pdata = &clv_sdhci_pci_data[SDIO_INDEX];
		pdata->quirks = sdhci_pdata_quirks;
		pdata->register_embedded_control = sdhci_embedded_control;
		break;
	case PCI_DEVICE_ID_INTEL_MRFL_MMC:
		switch (PCI_FUNC(pdev->devfn)) {
		case 0:
			pdata = &mrfl_sdhci_pci_data[EMMC0_INDEX];
			/*
			 * The current eMMC device simulation in Merrifield
			 * VP only implements boot partition 0, does not
			 * implements boot partition 1. And the VP will
			 * crash if eMMC boot partition 1 is accessed
			 * during kernel boot. So, we just disable boot
			 * partition support for Merrifield VP platform.
			 */
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_VP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_EMMC_BOOT_PART;
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HIGH_SPEED;
			break;
		case 1:
			pdata = &mrfl_sdhci_pci_data[EMMC1_INDEX];
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_VP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_EMMC_BOOT_PART;
			/*
			 * Merrifield HVP platform only implements
			 * eMMC0 host controller in its FPGA, and
			 * does not implements other 3 Merrifield
			 * SDHCI host controllers.
			 */
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HOST_CTRL_HW;
			break;
		case 2:
			pdata = &mrfl_sdhci_pci_data[SD_INDEX];
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HOST_CTRL_HW;
			break;
		case 3:
			pdata = &mrfl_sdhci_pci_data[SDIO_INDEX];
			if (intel_mid_identify_sim() ==
					INTEL_MID_CPU_SIMULATION_HVP)
				pdata->platform_quirks |=
					PLFM_QUIRK_NO_HOST_CTRL_HW;
				pdata->quirks = sdhci_pdata_quirks;
				pdata->register_embedded_control =
					sdhci_embedded_control;
			break;
		default:
			pr_err("%s func %s: Invalid PCI Dev func no. (%d)\n",
				__FILE__, __func__, PCI_FUNC(pdev->devfn));
			break;
		}
		break;
	case PCI_DEVICE_ID_INTEL_BYT_MMC:
	case PCI_DEVICE_ID_INTEL_BYT_MMC45:
		pdata = &byt_sdhci_pci_data[EMMC0_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_BYT_SD:
		pdata = &byt_sdhci_pci_data[SD_INDEX];
		break;
	case PCI_DEVICE_ID_INTEL_BYT_SDIO:
		pr_err("setting quirks/embedded controls on SDIO");
		pdata = &byt_sdhci_pci_data[SDIO_INDEX];
		pdata->quirks = sdhci_pdata_quirks;
		pdata->register_embedded_control = sdhci_embedded_control;
		break;
	default:
		break;
	}
	return pdata;
}

static void __devinit mmc_sdhci_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_sdhci_platform_data(pci_dev);
}

int sdhci_pdata_set_quirks(unsigned int quirks)
{
	/*Should not be set more than once*/
	WARN_ON(sdhci_pdata_quirks);
	sdhci_pdata_quirks = quirks;
	return 0;
}

int sdhci_pdata_set_embedded_control(void (*fnp)
			(void *dev_id, void (*virtual_cd)
			(void *dev_id, int card_present)))
{
	WARN_ON(sdhci_embedded_control);
	sdhci_embedded_control = fnp;
	return 0;
}

/* MRST MMC PCI IDs */
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MRST_SD0,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MRST_SD1,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MRST_SD2,
			mmc_sdhci_pci_early_quirks);

/* MFLD MMC PCI IDs */
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MFD_SD,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MFD_SDIO1,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MFD_SDIO2,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MFD_EMMC0,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MFD_EMMC1,
			mmc_sdhci_pci_early_quirks);

/* CLV MMC PCI IDs */
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_SDIO0,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_SDIO1,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_SDIO2,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_EMMC0,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_EMMC1,
			mmc_sdhci_pci_early_quirks);

/* MRFL MMC PCI IDs */
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_MRFL_MMC,
			mmc_sdhci_pci_early_quirks);

/* BYT MMC/SD/SDIO PCI IDs */
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_SD,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_MMC,
			mmc_sdhci_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_MMC45,
mmc_sdhci_pci_early_quirks);

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_SDIO,
			mmc_sdhci_pci_early_quirks);
