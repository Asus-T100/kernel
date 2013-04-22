/*
 * platform_bcm43xx.c: bcm43xx platform data initilization file
 *
 * (C) Copyright 2011 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <asm/intel-mid.h>
#include <linux/wlan_plat.h>
#include <linux/interrupt.h>
#include "platform_bcm43xx.h"
#include <linux/mmc/sdhci.h>
#include <linux/delay.h>

#include "pci/platform_sdhci_pci.h"

/* Delay copied from broadcom reference design */
#define DELAY_ONOFF 250

static int gpio_enable;
static void (*g_virtual_cd)(void *dev_id, int card_present);
void *g_host;

void bcmdhd_register_embedded_control(void *dev_id,
			void (*virtual_cd)(void *dev_id, int card_present))
{
	g_virtual_cd = virtual_cd;
	g_host = dev_id;
}

static int bcmdhd_set_power(int on)
{
	gpio_set_value(gpio_enable, on);

	/* Delay advice by BRCM */
	msleep(DELAY_ONOFF);

	return 0;
}

static int bcmdhd_set_card_detect(int detect)
{
	if (!g_virtual_cd)
		return -1;

	if (g_host)
		g_virtual_cd(g_host, detect);

	return 0;
}

static struct wifi_platform_data bcmdhd_data = {
	.set_power = bcmdhd_set_power,
	.set_carddetect = bcmdhd_set_card_detect,
};

static struct resource bcmdhd_res[] = {
	{
	.name = "bcmdhd_wlan_irq",
	.start = 1,
	.end = 1,
	.flags = IORESOURCE_IRQ | IRQF_TRIGGER_FALLING ,
	},
	{
	.name = "bcmdhd_wlan_en",
	.start = 1,
	.end = 1,
	.flags = IORESOURCE_IRQ ,
	}

};

static struct platform_device bcmdhd_device = {
	.name = "bcmdhd_wlan",
	.dev = {
		.platform_data = &bcmdhd_data,
		},
	.num_resources = ARRAY_SIZE(bcmdhd_res),
	.resource = bcmdhd_res,
};

static struct regulator_consumer_supply bcm43xx_vmmc3_supply = {
	.supply		= "vmmc",
	.dev_name	= "0000:00:00.0", /*default value*/
};

static struct regulator_init_data bcm43xx_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &bcm43xx_vmmc3_supply,
};

static struct fixed_voltage_config bcm43xx_vwlan = {
	.supply_name		= "vbcm43xx",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.startup_delay		= 70000,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &bcm43xx_vmmc3,
};

static struct platform_device bcm43xx_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &bcm43xx_vwlan,
	},
};

void __init bcm43xx_platform_data_init_post_scu(void *info)
{
	struct sd_board_info *sd_info = info;
	int wifi_irq_gpio;
	int err;

	pr_err("bcm43xx_platform_data_init_post_scu\n");

	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		/*Get GPIO numbers from the SFI table*/
		wifi_irq_gpio = get_gpio_by_name(BCM43XX_SFI_GPIO_IRQ_NAME);
	} else {
		wifi_irq_gpio = 144;
	}

	if (wifi_irq_gpio < 0) {
		pr_err("%s: Unable to find WLAN-interrupt GPIO in the SFI table\n",
				__func__);
		return;
	}

	bcmdhd_res[0].start = wifi_irq_gpio;
	bcmdhd_res[0].end = bcmdhd_res[0].start;

	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		gpio_enable = get_gpio_by_name(BCM43XX_SFI_GPIO_ENABLE_NAME);
	else {
		gpio_enable = 150;
		pr_err("baytrail, hardcoding GPIO Enable to %d\n", gpio_enable);
		gpio_request(gpio_enable, "bcm43xx_en");
	}
	if (gpio_enable < 0) {
		pr_err("%s: Unable to find WLAN-enable GPIO in the SFI table\n",
		       __func__);
		return;
	}


	bcmdhd_res[1].start = gpio_enable;
	bcmdhd_res[1].end = bcmdhd_res[1].start;

	/* format vmmc reg address from sfi table */
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		sprintf((char *)bcm43xx_vmmc3_supply.dev_name,
			"0000:00:%02x.%01x", (sd_info->addr)>>8,
			sd_info->addr&0xFF);

	err = platform_device_register(&bcmdhd_device);
	if (err < 0)
		pr_err("platform_device_register failed for bcmdhd_device\n");

	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		err = platform_device_register(&bcm43xx_vwlan_device);
		if (err < 0)
			pr_err("platform_device_register failed for bcm43xx_vwlan_device\n");
	}

}


void __init *bcm43xx_platform_data(void *info)
{
	struct sd_board_info *sd_info;
	unsigned int sdhci_quirk = SDHCI_QUIRK2_ADVERTISE_2V0_FORCE_1V8
			| SDHCI_QUIRK2_DISABLE_MMC_CAP_NONREMOVABLE
			| SDHCI_QUIRK2_ENABLE_MMC_PM_IGNORE_PM_NOTIFY;

	pr_err("Using bcm43xx platform data\n");

	sdhci_pdata_set_quirks(sdhci_quirk);
	sdhci_pdata_set_embedded_control(&bcmdhd_register_embedded_control);

	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2) {
		sd_info = kmemdup(info, sizeof(*sd_info), GFP_KERNEL);

		if (!sd_info) {
			pr_err("MRST: fail to alloc mem for delayed bcm43xx dev\n");
			return NULL;
		}
	}
	bcm43xx_platform_data_init_post_scu(sd_info);
	return &bcmdhd_device;
}
