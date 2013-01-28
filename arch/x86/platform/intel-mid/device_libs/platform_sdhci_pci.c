/*
 * platform_sdhci_pci.c: MMC/SD controller platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/mmc/sdhci-pci-data.h>

#include <asm/intel-mid.h>

#include <linux/intel_mid_pm.h>
#include <linux/hardirq.h>

static struct sdhci_pci_data sd_data;
static struct sdhci_pci_data emmc0_data;
static struct sdhci_pci_data emmc1_data;

static struct sdhci_pci_data *mfd_sdhci_host_get_data(struct pci_dev *pdev,
						      int slotno)
{
	struct sdhci_pci_data *pdata = NULL;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_MFD_EMMC0:
	case PCI_DEVICE_ID_INTEL_CLV_EMMC0:
		if (slotno == 0)
			pdata = &emmc0_data;
		break;
	case PCI_DEVICE_ID_INTEL_MFD_EMMC1:
	case PCI_DEVICE_ID_INTEL_CLV_EMMC1:
		if (slotno == 0)
			pdata = &emmc1_data;
		break;
	case PCI_DEVICE_ID_INTEL_MFD_SD:
	case PCI_DEVICE_ID_INTEL_CLV_SDIO0:
		if (slotno == 0)
			pdata = &sd_data;
		break;
	}

	return pdata;
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
	memset(&sd_data, 0, sizeof(struct sdhci_pci_data));
	memset(&emmc0_data, 0, sizeof(struct sdhci_pci_data));
	memset(&emmc1_data, 0, sizeof(struct sdhci_pci_data));

	sd_data.rst_n_gpio = -EINVAL;
	sd_data.cd_gpio = get_gpio_by_name("sd_cd_pin");
	sd_data.cd_gpio = 69;

	emmc0_data.rst_n_gpio = get_gpio_by_name("emmc0_rst");
	emmc0_data.cd_gpio = -EINVAL;
	emmc0_data.power_up = intel_sdhci_emmc0_power_up;

	emmc1_data.rst_n_gpio = get_gpio_by_name("emmc1_rst");
	emmc1_data.cd_gpio = -EINVAL;

	sdhci_pci_get_data = mfd_sdhci_host_get_data;
	return 0;
}
fs_initcall(sdhci_pci_platform_data_init);
