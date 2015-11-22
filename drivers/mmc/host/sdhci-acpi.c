/*
 * Secure Digital Host Controller Interface ACPI driver.
 *
 * Copyright (c) 2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/compiler.h>
#include <linux/stddef.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/lnw_gpio.h>

#include <linux/mmc/host.h>
#include <linux/mmc/pm.h>
#include <linux/mmc/sdhci.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/mmc/card.h>

#include <asm/spid.h>
#include <asm/cpu_device_id.h>

#include "sdhci.h"

#define INTEL_CHT_GPIO_SOUTHEAST	0xfed98000
#define INTEL_CHT_GPIO_LEN		0x2000
#define INTEL_CHV_RCOMP_CTRL	0x1180
#define INTEL_CHV_RCOMP_CONF	0x1194
#define INTEL_CHV_RCOMP_VAL	0x118c
#define INTEL_CHV_RCOMP_SET	0x1190

enum {
	SDHCI_ACPI_SD_CD	= BIT(0),
	SDHCI_ACPI_RUNTIME_PM	= BIT(1),
};

struct sdhci_acpi_chip {
	const struct	sdhci_ops *ops;
	unsigned int	quirks;
	unsigned int	quirks2;
	unsigned long	caps;
	unsigned int	caps2;
	mmc_pm_flag_t	pm_caps;
};

struct sdhci_acpi_slot {
	const struct	sdhci_acpi_chip *chip;
	unsigned int	quirks;
	unsigned int	quirks2;
	unsigned long	caps;
	unsigned int	caps2;
	mmc_pm_flag_t	pm_caps;
	unsigned int	flags;
	int (*probe_slot) (struct platform_device *);
	int (*remove_slot) (struct platform_device *);
};

struct sdhci_acpi_host {
	struct sdhci_host		*host;
	const struct sdhci_acpi_slot	*slot;
	struct platform_device		*pdev;
	bool				use_runtime_pm;
	unsigned int			autosuspend_delay;
	int				cd_gpio;
};

#define INTEL_VLV_CPU	0x37
#define INTEL_CHV_CPU	0x4c
static const struct x86_cpu_id intel_cpus[] = {
	{ X86_VENDOR_INTEL, 6, INTEL_VLV_CPU, X86_FEATURE_ANY, 0 },
	{ X86_VENDOR_INTEL, 6, INTEL_CHV_CPU, X86_FEATURE_ANY, 0 },
	{}
};

static bool sdhci_intel_host(unsigned int *cpu)
{
	const struct x86_cpu_id *id;
	if (!cpu)
		return false;
	id = x86_match_cpu(intel_cpus);
	if (!id)
		return false;
	*cpu = id->model;
	return true;
}

static inline bool sdhci_acpi_flag(struct sdhci_acpi_host *c, unsigned int flag)
{
	return c->slot && (c->slot->flags & flag);
}

static int sdhci_acpi_enable_dma(struct sdhci_host *host)
{
	return 0;
}

static void sdhci_acpi_int_hw_reset(struct sdhci_host *host)
{
	u8 reg;

	reg = sdhci_readb(host, SDHCI_POWER_CONTROL);
	reg |= 0x10;
	sdhci_writeb(host, reg, SDHCI_POWER_CONTROL);
	/* For eMMC, minimum is 1us but give it 9us for good measure */
	udelay(9);
	reg &= ~0x10;
	sdhci_writeb(host, reg, SDHCI_POWER_CONTROL);
	/* For eMMC, minimum is 200us but give it 300us for good measure */
	usleep_range(300, 1000);
}

static int sdhci_acpi_power_up_host(struct sdhci_host *host)
{
	int ret = -ENODEV;
	struct sdhci_acpi_host *c = sdhci_priv(host);
	acpi_handle handle;
	struct acpi_device *device;

	if (!c || !c->pdev)
		return -ENODEV;

	handle = ACPI_HANDLE(&c->pdev->dev);
	if (acpi_bus_get_device(handle, &device))
		return -ENODEV;

	ret = acpi_device_set_power(device, ACPI_STATE_D0);
	if (ret)
		return ret;
	/*
	 * after power up host, let's have a little test
	 */
	if (sdhci_readl(host, SDHCI_HOST_VERSION) ==
			0xffffffff) {
		pr_err("%s: power up sdhci host failed\n",
				__func__);
		return -EPERM;
	}

	pr_info("%s: host controller power up is done\n", __func__);

	return 0;
}

static int sdhci_acpi_get_tuning_count(struct sdhci_host *host)
{
	struct sdhci_acpi_host *c = sdhci_priv(host);
	acpi_handle handle;
	struct acpi_device *device;
	struct acpi_device_info *info;
	const char *hid, *uid = NULL;
	acpi_status status;
	int tuning_count = 0;

	if (!c || !c->pdev)
		return 0;

	handle = ACPI_HANDLE(&c->pdev->dev);
	if (acpi_bus_get_device(handle, &device))
		return -ENODEV;

	hid = acpi_device_hid(device);
	status = acpi_get_object_info(handle, &info);
	if (!ACPI_FAILURE(status) && (info->valid & ACPI_VALID_UID))
		uid = info->unique_id.string;

	if (!strcmp(hid, "80860F14") && !strcmp(uid, "1"))
		tuning_count = 8;

	if (info)
		ACPI_FREE(info);

	return tuning_count;
}

/* CHT A0 workaround */
static int sdhci_intel_chv_set_io_vol(struct sdhci_host *host, bool to_1p8)
{
	void __iomem *ioaddr = host->gpiobase;
	u32 value, config = 0;
	u8 slew;
	u8 pstrength, nstrength;

	if (!ioaddr)
		return 0;

	/* read RCOMP control to set bit31 */
	value = readl(ioaddr + INTEL_CHV_RCOMP_CTRL);
	value |= 0x80000000;
	writel(value, ioaddr + INTEL_CHV_RCOMP_CTRL);

	/* set RCOMP family config reg 1p8_enable */
	value = readl(ioaddr + INTEL_CHV_RCOMP_CONF);
	if (to_1p8)
		value |= 0x200000;
	else
		value &= ~0x200000;
	writel(value, ioaddr + INTEL_CHV_RCOMP_CONF);
	udelay(100);

	/* read RCOMP value */
	value = readl(ioaddr + INTEL_CHV_RCOMP_VAL);
	slew = (value >> 16) & 0xff;
	pstrength = (value >> 8) & 0xff;
	nstrength = value & 0xff;

	/* read family config value */
	config |= slew | (nstrength << 16) | (pstrength << 24) | 0x300;
	writel(config, ioaddr + INTEL_CHV_RCOMP_SET);

	return 0;
}

static int sdhci_acpi_set_io_vol(struct sdhci_host *host, bool to_1p8)
{
	unsigned int cpu;

	if (sdhci_intel_host(&cpu) && (cpu == INTEL_CHV_CPU))
		return sdhci_intel_chv_set_io_vol(host, to_1p8);

	return 0;
}

static const struct sdhci_ops sdhci_acpi_ops_dflt = {
	.enable_dma = sdhci_acpi_enable_dma,
	.power_up_host	= sdhci_acpi_power_up_host,
	.get_tuning_count = sdhci_acpi_get_tuning_count,
	.set_io_voltage	= sdhci_acpi_set_io_vol,
};

static const struct sdhci_ops sdhci_acpi_ops_int = {
	.enable_dma = sdhci_acpi_enable_dma,
	.hw_reset   = sdhci_acpi_int_hw_reset,
};

static const struct sdhci_acpi_chip sdhci_acpi_chip_int = {
	.ops = &sdhci_acpi_ops_int,
};

#define SDHCI_PLATFORMDEV_2_MMCHOST(dev) \
	struct platform_device *pdev = container_of(dev, struct platform_device, dev); \
	struct sdhci_acpi_host *chip; \
	struct sdhci_host *host; \
	struct mmc_host *mmc; \
	chip = platform_get_drvdata(pdev); \
	if (!chip) \
		return 0; \
	host = chip->host; \
	if(!host) \
		return 0; \
	mmc = host->mmc; \
	if(!mmc) \
		return 0;

/* total size is 2^n GB, e.g: 4/8/16/32/64/128 */
static ssize_t sdhci_eMMC_total_size(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i;
	SDHCI_PLATFORMDEV_2_MMCHOST(dev)

	if (mmc->card) {
		i = fls(mmc->card->ext_csd.sectors);
		if (i > 21)
			return snprintf(buf, PAGE_SIZE, "%d\n", (mmc->card->ext_csd.sectors >> (i - 1)) << (i - 21));/* 4GB or above */
        else
			pr_err("wrong sector count");
	}
	return 0;
}

#ifdef ASUS_FACTORY
static ssize_t sdhci_eMMC_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	SDHCI_PLATFORMDEV_2_MMCHOST(dev)

	return snprintf(buf,PAGE_SIZE,"%s\n",mmc->card->cid.prod_name);
}

static ssize_t sdhci_eMMC_fw(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint32_t bit;

	SDHCI_PLATFORMDEV_2_MMCHOST(dev)

    // PRV is located at CID[48:55], which is in the raw_cid[2] 
	bit = mmc->card->raw_cid[2];

	return snprintf(buf, PAGE_SIZE, "0x%x\n", (bit >> 16) & 0xff);
}

static ssize_t sdhci_eMMC_sector_size(struct device *dev, struct device_attribute *attr, char *buf)
{
	SDHCI_PLATFORMDEV_2_MMCHOST(dev)

	return snprintf(buf, PAGE_SIZE, "0x%x\n", mmc->card->ext_csd.sectors);
}
#endif

#ifdef ASUS_FACTORY
static DEVICE_ATTR(eMMC_name, S_IRUGO, sdhci_eMMC_name, NULL);
static DEVICE_ATTR(eMMC_fw, S_IRUGO, sdhci_eMMC_fw, NULL);
static DEVICE_ATTR(eMMC_size, S_IRUGO, sdhci_eMMC_sector_size, NULL);
#endif
static DEVICE_ATTR(eMMC_total_size, S_IRUGO, sdhci_eMMC_total_size, NULL);

static struct attribute *acpi_dev_emmc_attrs[] = {
#ifdef ASUS_FACTORY
    &dev_attr_eMMC_name.attr,
    &dev_attr_eMMC_fw.attr,
    &dev_attr_eMMC_size.attr,
#endif
	&dev_attr_eMMC_total_size.attr,
	NULL,
};

static struct attribute_group dev_emmc_attr_grp = {
	.attrs = acpi_dev_emmc_attrs,
};

static int sdhci_acpi_emmc_probe_slot(struct platform_device *pdev)
{
	struct sdhci_acpi_host *c = platform_get_drvdata(pdev);
	struct sdhci_host *host;
	unsigned int cpu;

	if (!c || !c->host)
		return 0;

	host = c->host;

	if (!INTEL_MID_BOARDV2(TABLET, BYT, BLB, PRO) &&
			!INTEL_MID_BOARDV2(TABLET, BYT, BLB, ENG))
		sdhci_alloc_panic_host(host);

	host->mmc->caps2 |= MMC_CAP2_CACHE_CTRL;
	/* Enable Packed Command */
	host->mmc->caps2 |= MMC_CAP2_PACKED_CMD;

	if (!sdhci_intel_host(&cpu))
		return 0;

	switch (cpu) {
	case INTEL_VLV_CPU:
		host->mmc->qos = kzalloc(sizeof(struct pm_qos_request),
				GFP_KERNEL);
		pm_qos_add_request(host->mmc->qos, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);
		break;
	default:
		break;
	}
	sysfs_create_group(&pdev->dev.kobj, &dev_emmc_attr_grp);
	return 0;
}


static int sdhci_acpi_sdio_probe_slot(struct platform_device *pdev)
{
	struct sdhci_acpi_host *c = platform_get_drvdata(pdev);
	struct sdhci_host *host;
	unsigned int cpu;

	if (!c || !c->host)
		return 0;

	host = c->host;

	if (!sdhci_intel_host(&cpu))
		return 0;

	switch (cpu) {
	case INTEL_VLV_CPU:
		/* increase the auto suspend delay for SDIO to be 500ms */
		c->autosuspend_delay = 500;
		host->mmc->qos = kzalloc(sizeof(struct pm_qos_request),
				GFP_KERNEL);
		pm_qos_add_request(host->mmc->qos, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);
		break;
	case INTEL_CHV_CPU:
		host->quirks2 |= SDHCI_QUIRK2_SDR104_BROKEN;
		break;
	default:
		break;
	}

	return 0;
}

static const struct sdhci_acpi_slot sdhci_acpi_slot_int_emmc = {
	.quirks2 = SDHCI_QUIRK2_CARD_CD_DELAY | SDHCI_QUIRK2_WAIT_FOR_IDLE |
		SDHCI_QUIRK2_PRESET_VALUE_BROKEN | SDHCI_QUIRK2_TUNING_POLL,
	.caps    = MMC_CAP_8_BIT_DATA | MMC_CAP_NONREMOVABLE | MMC_CAP_HW_RESET
		| MMC_CAP_1_8V_DDR,
	.caps2   = MMC_CAP2_HC_ERASE_SZ | MMC_CAP2_POLL_R1B_BUSY |
#ifndef CONFIG_MMC_DISABLE_HS200_HS400
		MMC_CAP2_BOOTPART_NOACC | MMC_CAP2_RPMBPART_NOACC |
		MMC_CAP2_HS200_1_8V_SDR,
#else
		MMC_CAP2_BOOTPART_NOACC | MMC_CAP2_RPMBPART_NOACC,
#endif
	.flags   = SDHCI_ACPI_RUNTIME_PM,
	.probe_slot	= sdhci_acpi_emmc_probe_slot,
};

static const struct sdhci_acpi_slot sdhci_acpi_slot_int_sdio = {
	.quirks2 = SDHCI_QUIRK2_HOST_OFF_CARD_ON |
		SDHCI_QUIRK2_PRESET_VALUE_BROKEN | SDHCI_QUIRK2_FAKE_VDD,
	.caps    = MMC_CAP_NONREMOVABLE | MMC_CAP_POWER_OFF_CARD,
	.flags   = SDHCI_ACPI_RUNTIME_PM,
	.pm_caps = MMC_PM_KEEP_POWER | MMC_PM_WAKE_SDIO_IRQ |
		MMC_PM_WAKE_SDIO_IRQ,
	.probe_slot	= sdhci_acpi_sdio_probe_slot,
};

#ifdef ASUS_FACTORY

static ssize_t sdhci_sd_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = container_of(dev, struct platform_device, dev);
	struct sdhci_acpi_host *c;
	c = platform_get_drvdata(pdev);

	if (!c)
		return 0;

	if (sdhci_acpi_flag(c, SDHCI_ACPI_SD_CD) && c->cd_gpio != -ENODEV)
		return snprintf(buf, PAGE_SIZE, "1\n");
	else
		// return snprintf(buf, PAGE_SIZE, " c->slot: %X c->slot->flags: %X SDHCI_ACPI_SD_CD: %X c->cd_gpio: %d\n", c->slot, c->slot->flags,SDHCI_ACPI_SD_CD,c->cd_gpio);
		return snprintf(buf, PAGE_SIZE, "0\n");
}

static DEVICE_ATTR(sd_status, S_IRUGO, sdhci_sd_status, NULL);

static struct attribute *acpi_dev_sd_attrs[] = {
	&dev_attr_sd_status.attr,
	NULL,
};

static struct attribute_group dev_sd_attr_grp = {
	.attrs = acpi_dev_sd_attrs,
};

#endif

static int sdhci_acpi_sd_probe_slot(struct platform_device *pdev)
{
	struct sdhci_acpi_host *c = platform_get_drvdata(pdev);
	struct sdhci_host *host;
	unsigned int cpu;

	if (!c || !c->host || !c->slot)
		return 0;

	host = c->host;

	/* On BYT-M, SD card is using to store ipanic as a W/A */
	if (INTEL_MID_BOARDV2(TABLET, BYT, BLB, PRO) ||
			INTEL_MID_BOARDV2(TABLET, BYT, BLB, ENG))
		sdhci_alloc_panic_host(host);

	c->cd_gpio = acpi_get_gpio_by_index(&pdev->dev, 0, NULL);

	host->mmc->caps2 |= MMC_CAP2_PWCTRL_POWER;

	/* Bayley Bay board */
	if (INTEL_MID_BOARD(2, TABLET, BYT, BLB, PRO) ||
			INTEL_MID_BOARD(2, TABLET, BYT, BLB, ENG))
		host->quirks2 |= SDHCI_QUIRK2_NO_1_8_V;

	/*
	 * CHT A0 workaround
	 */
	if (!sdhci_intel_host(&cpu))
		return 0;

	if (cpu == INTEL_CHV_CPU) {
		host->gpiobase = ioremap_nocache(INTEL_CHT_GPIO_SOUTHEAST,
				INTEL_CHT_GPIO_LEN);
		host->quirks2 |= SDHCI_QUIRK2_CARD_CD_DELAY;
	} else if (cpu == INTEL_VLV_CPU) {
		host->mmc->qos = kzalloc(sizeof(struct pm_qos_request),
				GFP_KERNEL);
		pm_qos_add_request(host->mmc->qos, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);
	}
#ifdef ASUS_FACTORY
	sysfs_create_group(&pdev->dev.kobj, &dev_sd_attr_grp);
#endif
	return 0;
}

static const struct sdhci_acpi_slot sdhci_acpi_slot_int_sd = {
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
		SDHCI_QUIRK2_SDR104_BROKEN,
	.caps2   = MMC_CAP2_FIXED_NCRC,
	.flags   = SDHCI_ACPI_SD_CD | SDHCI_ACPI_RUNTIME_PM,
	.probe_slot	= sdhci_acpi_sd_probe_slot,
};

struct sdhci_acpi_uid_slot {
	const char *hid;
	const char *uid;
	const struct sdhci_acpi_slot *slot;
};

static const struct sdhci_acpi_uid_slot sdhci_acpi_uids[] = {
	{ "80860F14" , "1" , &sdhci_acpi_slot_int_emmc },
	{ "80860F14" , "3" , &sdhci_acpi_slot_int_sd   },
	{ "INT33BB"  , "2" , &sdhci_acpi_slot_int_sdio },
	{ "INT33BB"  , "3" , &sdhci_acpi_slot_int_sd   },
	{ "INT33C6"  , NULL, &sdhci_acpi_slot_int_sdio },
	{ "PNP0D40"  },
	{ },
};

static const struct acpi_device_id sdhci_acpi_ids[] = {
	{ "80860F14" },
	{ "INT33BB"  },
	{ "INT33C6"  },
	{ "PNP0D40"  },
	{ },
};
MODULE_DEVICE_TABLE(acpi, sdhci_acpi_ids);

static const struct sdhci_acpi_slot *sdhci_acpi_get_slot_by_ids(const char *hid,
								const char *uid)
{
	const struct sdhci_acpi_uid_slot *u;

	for (u = sdhci_acpi_uids; u->hid; u++) {
		if (strcmp(u->hid, hid))
			continue;
		if (!u->uid)
			return u->slot;
		if (uid && !strcmp(u->uid, uid))
			return u->slot;
	}
	return NULL;
}

static const struct sdhci_acpi_slot *sdhci_acpi_get_slot(acpi_handle handle,
							 const char *hid)
{
	const struct sdhci_acpi_slot *slot;
	struct acpi_device_info *info;
	const char *uid = NULL;
	acpi_status status;

	status = acpi_get_object_info(handle, &info);
	if (!ACPI_FAILURE(status) && (info->valid & ACPI_VALID_UID))
		uid = info->unique_id.string;

	slot = sdhci_acpi_get_slot_by_ids(hid, uid);

	kfree(info);
	return slot;
}

static int sdhci_acpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	acpi_handle handle = ACPI_HANDLE(dev);
	struct acpi_device *device;
	struct sdhci_acpi_host *c;
	struct sdhci_host *host;
	struct resource *iomem;
	resource_size_t len;
	const char *hid;
	int err;

	if (acpi_bus_get_device(handle, &device))
		return -ENODEV;

	if (acpi_bus_get_status(device) || !device->status.present)
		return -ENODEV;

	hid = acpi_device_hid(device);

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem)
		return -ENOMEM;

	len = resource_size(iomem);
	if (len < 0x100)
		dev_err(dev, "Invalid iomem size!\n");

	if (!devm_request_mem_region(dev, iomem->start, len, dev_name(dev)))
		return -ENOMEM;

	host = sdhci_alloc_host(dev, sizeof(struct sdhci_acpi_host));
	if (IS_ERR(host))
		return PTR_ERR(host);

	c = sdhci_priv(host);
	c->host = host;
	c->slot = sdhci_acpi_get_slot(handle, hid);
	c->pdev = pdev;
	c->use_runtime_pm = sdhci_acpi_flag(c, SDHCI_ACPI_RUNTIME_PM);
	c->cd_gpio = -ENODEV;
	c->autosuspend_delay = 0;

	platform_set_drvdata(pdev, c);

	host->hw_name	= "ACPI";
	host->ops	= &sdhci_acpi_ops_dflt;
	host->irq	= platform_get_irq(pdev, 0);

	host->ioaddr = devm_ioremap_nocache(dev, iomem->start,
					    resource_size(iomem));
	if (host->ioaddr == NULL) {
		err = -ENOMEM;
		goto err_free;
	}

	if (!dev->dma_mask) {
		u64 dma_mask;

		if (sdhci_readl(host, SDHCI_CAPABILITIES) & SDHCI_CAN_64BIT) {
			/* 64-bit DMA is not supported at present */
			dma_mask = DMA_BIT_MASK(32);
		} else {
			dma_mask = DMA_BIT_MASK(32);
		}

		dev->dma_mask = &dev->coherent_dma_mask;
		dev->coherent_dma_mask = dma_mask;
	}

	if (c->slot) {
		if (c->slot->probe_slot) {
			err = c->slot->probe_slot(pdev);
			if (err)
				goto err_free;
		}
		if (c->slot->chip) {
			host->ops            = c->slot->chip->ops;
			host->quirks        |= c->slot->chip->quirks;
			host->quirks2       |= c->slot->chip->quirks2;
			host->mmc->caps     |= c->slot->chip->caps;
			host->mmc->caps2    |= c->slot->chip->caps2;
			host->mmc->pm_caps  |= c->slot->chip->pm_caps;
		}
		host->quirks        |= c->slot->quirks;
		host->quirks2       |= c->slot->quirks2;
		host->mmc->caps     |= c->slot->caps;
		host->mmc->caps2    |= c->slot->caps2;
		host->mmc->pm_caps  |= c->slot->pm_caps;
	}

	host->mmc->caps2 |= MMC_CAP2_NO_PRESCAN_POWERUP;

	err = sdhci_add_host(host);
	if (err)
		goto err_free;

	if (sdhci_acpi_flag(c, SDHCI_ACPI_SD_CD) &&
			c->cd_gpio != -ENODEV) {
		/*
		 * WORKAROUND, can be removed when GPIO_CD
		 * set to GPIO mode by default
		 */
		lnw_gpio_set_alt(c->cd_gpio, 0);
		if (mmc_gpio_request_cd(host->mmc, c->cd_gpio))
			c->use_runtime_pm = false;
	}

	if (c->use_runtime_pm) {
		pm_runtime_set_active(dev);
		pm_suspend_ignore_children(dev, 1);
		if (c->autosuspend_delay)
			pm_runtime_set_autosuspend_delay(dev, c->autosuspend_delay);
		else
			pm_runtime_set_autosuspend_delay(dev, 50);
		pm_runtime_use_autosuspend(dev);
		pm_runtime_enable(dev);
	}

	return 0;

err_free:
	platform_set_drvdata(pdev, NULL);
	sdhci_free_host(c->host);
	return err;
}

static int sdhci_acpi_remove(struct platform_device *pdev)
{
	struct sdhci_acpi_host *c = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	int dead;

	if (c->use_runtime_pm) {
		pm_runtime_get_sync(dev);
		pm_runtime_disable(dev);
		pm_runtime_put_noidle(dev);
	}

	if (c->slot && c->slot->remove_slot)
		c->slot->remove_slot(pdev);

	dead = (sdhci_readl(c->host, SDHCI_INT_STATUS) == ~0);
	sdhci_remove_host(c->host, dead);
	platform_set_drvdata(pdev, NULL);
	sdhci_free_host(c->host);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int sdhci_acpi_suspend(struct device *dev)
{
	struct sdhci_acpi_host *c = dev_get_drvdata(dev);

	return sdhci_suspend_host(c->host);
}

static int sdhci_acpi_resume(struct device *dev)
{
	struct sdhci_acpi_host *c = dev_get_drvdata(dev);

	return sdhci_resume_host(c->host);
}

#else

#define sdhci_acpi_suspend	NULL
#define sdhci_acpi_resume	NULL

#endif

#ifdef CONFIG_PM_RUNTIME

static int sdhci_acpi_runtime_suspend(struct device *dev)
{
	struct sdhci_acpi_host *c = dev_get_drvdata(dev);

	return sdhci_runtime_suspend_host(c->host);
}

static int sdhci_acpi_runtime_resume(struct device *dev)
{
	struct sdhci_acpi_host *c = dev_get_drvdata(dev);

	return sdhci_runtime_resume_host(c->host);
}

static int sdhci_acpi_runtime_idle(struct device *dev)
{
	return 0;
}

#else

#define sdhci_acpi_runtime_suspend	NULL
#define sdhci_acpi_runtime_resume	NULL
#define sdhci_acpi_runtime_idle		NULL

#endif

static const struct dev_pm_ops sdhci_acpi_pm_ops = {
	.suspend		= sdhci_acpi_suspend,
	.resume			= sdhci_acpi_resume,
	.runtime_suspend	= sdhci_acpi_runtime_suspend,
	.runtime_resume		= sdhci_acpi_runtime_resume,
	.runtime_idle		= sdhci_acpi_runtime_idle,
};

static struct platform_driver sdhci_acpi_driver = {
	.driver = {
		.name			= "sdhci-acpi",
		.owner			= THIS_MODULE,
		.acpi_match_table	= sdhci_acpi_ids,
		.pm			= &sdhci_acpi_pm_ops,
	},
	.probe	= sdhci_acpi_probe,
	.remove	= sdhci_acpi_remove,
};

module_platform_driver(sdhci_acpi_driver);

MODULE_DESCRIPTION("Secure Digital Host Controller Interface ACPI driver");
MODULE_AUTHOR("Adrian Hunter");
MODULE_LICENSE("GPL v2");
