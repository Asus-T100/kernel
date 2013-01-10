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
#include "platform_mmc_sdhci_pci.h"
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/delay.h>
#include <asm/intel_scu_ipc.h>
#include <linux/intel_mid_pm.h>
#include <linux/hardirq.h>


static struct sdhci_pci_data mfd_clv_emmc0_data;
static struct sdhci_pci_data mfd_clv_emmc1_data;
static struct sdhci_pci_data mfd_clv_sd_data;
static struct sdhci_pci_data mfd_clv_sdio_data;

static struct sdhci_pci_data mfld_sdio_data;

static struct sdhci_pci_data mrfl_emmc0_data;
static struct sdhci_pci_data mrfl_emmc1_data;
static struct sdhci_pci_data mrfl_sd_data;
static struct sdhci_pci_data mrfl_sdio_data;

int bcmdhd_get_sdhci_quirk(void)
{
	return 0;
}

int bcmdhd_unset_sdhci_mmc_caps(void)
{
	return 0;
}

int bcmdhd_set_sdhci_mmc_pm_flags(void)
{
	return 0;
}

void bcmdhd_register_embedded_control(void *dev_id, void (*virtual_cd)
					(void *dev_id, int card_present))
{
	return;
}

static struct sdhci_pci_data *sdhci_host_get_data(struct pci_dev *pdev,
							int slotno)
{
	struct sdhci_pci_data *pdata = NULL;

	if (slotno > 0) {
		pr_err("File %s func %s: Invalid slotno (%d)\n",
			__FILE__, __func__, slotno);
		return pdata;
	}

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_MFD_EMMC0:
	case PCI_DEVICE_ID_INTEL_CLV_EMMC0:
		pdata = &mfd_clv_emmc0_data;
		break;
	case PCI_DEVICE_ID_INTEL_MFD_EMMC1:
	case PCI_DEVICE_ID_INTEL_CLV_EMMC1:
		pdata = &mfd_clv_emmc1_data;
		break;
	case PCI_DEVICE_ID_INTEL_MFD_SDIO1:
		pdata = &mfld_sdio_data;
		break;
	case PCI_DEVICE_ID_INTEL_MFD_SD:
	case PCI_DEVICE_ID_INTEL_CLV_SDIO0:
		pdata = &mfd_clv_sd_data;
		break;
	case PCI_DEVICE_ID_INTEL_CLV_SDIO1:
		pdata = &mfd_clv_sdio_data;
		break;
	case PCI_DEVICE_ID_INTEL_MRFL_MMC:
		switch (PCI_FUNC(pdev->devfn)) {
		case 0:
			pdata = &mrfl_emmc0_data;
			break;
		case 1:
			pdata = &mrfl_emmc1_data;
			break;
		case 2:
			pdata = &mrfl_sd_data;
			break;
		case 3:
			pdata = &mrfl_sdio_data;
			break;
		default:
			pr_err("File %s function %s: Invalid PCI device function number (%d)\n",
				__FILE__, __func__, PCI_FUNC(pdev->devfn));
			break;
		}
		break;
	default:
		break;
	}

	return pdata;
}

/* Board specific setup related to SD goes here */
static int mrfl_sd_setup(struct sdhci_pci_data *data)
{
#ifdef CONFIG_BOARD_MRFLD_VV
	u8 vldocnt = 0;
	int err;

	/*
	 * Change necessary GPIO pin mode for SD card working.
	 * This is something should be done in IA firmware.
	 * But, anyway, just do it here in case IA firmware
	 * forget to do so.
	 */
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_CD, 0);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_CLK, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_CMD, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_DATA_0, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_DATA_1, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_DATA_2, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_DATA_3, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_LVL_CLK_FB, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_LVL_CMD_DIR, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_LVL_DAT_DIR, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_LVL_EN, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_LVL_SEL, 1);
	lnw_gpio_set_alt(MRFLD_GPIO_SDIO_0_WP, 1);

	if (gpio_is_valid(MRFLD_GPIO_SDIO_0_LVL_EN)) {
		if (!gpio_request(MRFLD_GPIO_SDIO_0_LVL_EN, "MRFL SD lvl_en"))
			gpio_direction_output(MRFLD_GPIO_SDIO_0_LVL_EN, 1);
		else
			pr_err("Failed to request sd_lvl_en\n");
	}

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

#endif
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

static int intel_sdhci_emmc0_power_up(void *data)
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

static int __init sdhci_pci_platform_data_init(void)
{
	memset(&mfd_clv_emmc0_data, 0, sizeof(struct sdhci_pci_data));
	memset(&mfd_clv_emmc1_data, 0, sizeof(struct sdhci_pci_data));
	memset(&mfd_clv_sd_data, 0, sizeof(struct sdhci_pci_data));
	memset(&mfd_clv_sdio_data, 0, sizeof(struct sdhci_pci_data));


	mfd_clv_emmc0_data.rst_n_gpio = get_gpio_by_name("emmc0_rst");
	mfd_clv_emmc0_data.cd_gpio = -EINVAL;
	mfd_clv_emmc0_data.power_up = intel_sdhci_emmc0_power_up;

	mfd_clv_emmc1_data.rst_n_gpio = get_gpio_by_name("emmc1_rst");
	mfd_clv_emmc1_data.cd_gpio = -EINVAL;

	mfd_clv_sd_data.rst_n_gpio = -EINVAL;
	mfd_clv_sd_data.cd_gpio = get_gpio_by_name("sd_cd_pin");
	mfd_clv_sd_data.cd_gpio = 69;

	mfd_clv_sdio_data.rst_n_gpio = -EINVAL;
	mfd_clv_sdio_data.cd_gpio = -EINVAL;

	mfd_clv_sdio_data.quirks = bcmdhd_get_sdhci_quirk();
	mfd_clv_sdio_data.register_embedded_control =
					bcmdhd_register_embedded_control;
	mfd_clv_sdio_data.mmc_caps = bcmdhd_unset_sdhci_mmc_caps();
	mfd_clv_sdio_data.mmc_pm_flags = bcmdhd_set_sdhci_mmc_pm_flags();

	memset(&mrfl_emmc0_data, 0, sizeof(struct sdhci_pci_data));
	memset(&mrfl_emmc1_data, 0, sizeof(struct sdhci_pci_data));
	memset(&mrfl_sd_data, 0, sizeof(struct sdhci_pci_data));
	memset(&mrfl_sdio_data, 0, sizeof(struct sdhci_pci_data));

	/* For Merrifield, eMMC0 Hardware Reset pin can not be used as gpio */
	mrfl_emmc0_data.rst_n_gpio = -EINVAL;
	mrfl_emmc0_data.cd_gpio = -EINVAL;

	/* eMMC1 will be removed from TNG soon */
	mrfl_emmc1_data.rst_n_gpio = 97;
	mrfl_emmc1_data.cd_gpio = -EINVAL;

	mrfl_sd_data.rst_n_gpio = -EINVAL;
	mrfl_sd_data.cd_gpio = 77; /* Fix me: searching SFI table */
	mrfl_sd_data.setup = mrfl_sd_setup;
	mrfl_sd_data.cleanup = mrfl_sd_cleanup;

	mrfl_sdio_data.rst_n_gpio = -EINVAL;
	mrfl_sdio_data.cd_gpio = -EINVAL;
	mrfl_sdio_data.setup = mrfl_sdio_setup;
	mrfl_sdio_data.cleanup = mrfl_sdio_cleanup;
	mrfl_sdio_data.quirks = bcmdhd_get_sdhci_quirk();
	mrfl_sdio_data.register_embedded_control =
				bcmdhd_register_embedded_control;
	mrfl_sdio_data.mmc_caps = bcmdhd_unset_sdhci_mmc_caps();
	mrfl_sdio_data.mmc_pm_flags = bcmdhd_set_sdhci_mmc_pm_flags();

	memset(&mfld_sdio_data, 0, sizeof(struct sdhci_pci_data));

	mfld_sdio_data.rst_n_gpio = -EINVAL;
	mfld_sdio_data.cd_gpio = -EINVAL;
	mfld_sdio_data.quirks = bcmdhd_get_sdhci_quirk();
	mfld_sdio_data.register_embedded_control =
				bcmdhd_register_embedded_control;
	mfld_sdio_data.mmc_caps = bcmdhd_unset_sdhci_mmc_caps();
	mfld_sdio_data.mmc_pm_flags = bcmdhd_set_sdhci_mmc_pm_flags();

	sdhci_pci_get_data = sdhci_host_get_data;

	return 0;
}
fs_initcall(sdhci_pci_platform_data_init);
