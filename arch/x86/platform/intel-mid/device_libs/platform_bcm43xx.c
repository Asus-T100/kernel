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

#define DELAY_ONOFF 250

static int sdhci_quirk;
static int sdhci_mmc_caps;

static int gpio_enable;
static void (*g_virtual_cd)(void *dev_id, int card_present);
void *g_host;

void bcmdhd_register_embedded_control(void *dev_id,
			void (*virtual_cd)(void *dev_id, int card_present))
{
	g_virtual_cd = virtual_cd;
	g_host = dev_id;
}

int bcmdhd_get_sdhci_quirk(void)
{
	return sdhci_quirk;
}

int bcmdhd_get_sdhci_mmc_caps(void)
{
	return sdhci_mmc_caps;
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

	/*Get GPIO numbers from the SFI table*/
	wifi_irq_gpio = get_gpio_by_name(BCM43XX_SFI_GPIO_IRQ_NAME);
	if (wifi_irq_gpio == -1) {
		pr_err("%s: Unable to find WLAN-interrupt GPIO in the SFI table\n",
				__func__);
		return;
	}

	bcmdhd_res[0].start = wifi_irq_gpio;
	bcmdhd_res[0].end = bcmdhd_res[0].start;
	if (bcmdhd_res[0].start < 0) {
		pr_err("%s:Error gpio_to_irq:%d->%d\n", __func__, wifi_irq_gpio,
		       bcmdhd_res[0].start);
		return;
	}

	err = platform_device_register(&bcmdhd_device);
	if (err < 0)
		pr_err("error setting bcmdhd data\n");

	gpio_enable = get_gpio_by_name(BCM43XX_SFI_GPIO_ENABLE_NAME);
	if (gpio_enable == -1) {
		pr_err("%s: Unable to find WLAN-enable GPIO in the SFI table\n",
		       __func__);
		return;
	}
	/* format vmmc reg address from sfi table */
	sprintf((char *)bcm43xx_vmmc3_supply.dev_name, "0000:00:%02x.%01x",
		(sd_info->addr)>>8, sd_info->addr&0xFF);

	err = platform_device_register(&bcm43xx_vwlan_device);
	if (err < 0)
		pr_err("error platform_device_register\n");

}

void __init *bcm43xx_platform_data(void *info)
{
	struct sd_board_info *sd_info;

	sdhci_quirk = SDHCI_QUIRK_ADVERTISE_2V0_FORCE_1V8;
	sdhci_mmc_caps = MMC_CAP_NONREMOVABLE;

	sd_info = kmemdup(info, sizeof(*sd_info), GFP_KERNEL);
	if (!sd_info) {
		pr_err("MRST: fail to alloc mem for delayed bcm43xx dev\n");
		return NULL;
	}
	intel_delayed_device_register(sd_info,
				      bcm43xx_platform_data_init_post_scu);
	return &bcmdhd_device;
}
