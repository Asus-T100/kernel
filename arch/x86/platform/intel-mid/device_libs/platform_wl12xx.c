/*
 * platform_wl12xx.c: wl12xx platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/wl12xx.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <asm/intel-mid.h>
#include "platform_wl12xx.h"

static struct wl12xx_platform_data mid_wifi_control = {
	.board_ref_clock = 1,
	.irq = 2,
	.board_tcxo_clock = 1,
	.platform_quirks = WL12XX_PLATFORM_QUIRK_EDGE_IRQ,
};

static struct regulator_consumer_supply wl12xx_vmmc3_supply = {
	.supply		= "vmmc",
	.dev_name	= "0000:00:00.0", /*default value*/
};

static struct regulator_init_data wl12xx_vmmc3 = {
	.constraints = {
		.valid_ops_mask	= REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies = &wl12xx_vmmc3_supply,
};

static struct fixed_voltage_config wl12xx_vwlan = {
	.supply_name		= "vwl1271",
	.microvolts		= 1800000,
	.gpio			= 75,
	.startup_delay		= 70000,
	.enable_high		= 1,
	.enabled_at_boot	= 0,
	.init_data		= &wl12xx_vmmc3,
};

static struct platform_device wl12xx_vwlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data	= &wl12xx_vwlan,
	},
};

void __init wl12xx_platform_data_init_post_scu(void *info)
{
	struct sd_board_info *sd_info = info;
	int wifi_irq_gpio;
	int err;

	/*Get GPIO numbers from the SFI table*/
	wifi_irq_gpio = get_gpio_by_name(WL12XX_SFI_GPIO_IRQ_NAME);
	if (wifi_irq_gpio == -1) {
		pr_err("%s: Unable to find WLAN-interrupt GPIO in the SFI table\n",
				__func__);
		return;
	}
	err = gpio_request(wifi_irq_gpio, "wl12xx");
	if (err < 0) {
		pr_err("%s: Unable to request GPIO\n", __func__);
		return;
	}
	err = gpio_direction_input(wifi_irq_gpio);
	if (err < 0) {
		pr_err("%s: Unable to set GPIO direction\n", __func__);
		return;
	}
	mid_wifi_control.irq = gpio_to_irq(wifi_irq_gpio);
	if (mid_wifi_control.irq < 0) {
		pr_err("%s:Error gpio_to_irq:%d->%d\n", __func__, wifi_irq_gpio,
		       mid_wifi_control.irq);
		return;
	}
	if (board_id != MFLD_BID_SALITPA_EV1) {
		/* Set our board_ref_clock from SFI SD board info */
		if (sd_info->board_ref_clock == BOARD_26M_CLK)
			mid_wifi_control.board_ref_clock = 1;
		else if (sd_info->board_ref_clock == BOARD_38M_CLK)
			mid_wifi_control.board_ref_clock = 2;
	} else {
		/* EVx need TCXO change instead of ref clock
		 * SFI table only handle 1 clock parameter
		 */
		if (sd_info->board_ref_clock == BOARD_26M_CLK)
			mid_wifi_control.board_tcxo_clock = 1;
		else if (sd_info->board_ref_clock == BOARD_38M_CLK)
			mid_wifi_control.board_tcxo_clock = 2;
	}
	err = wl12xx_set_platform_data(&mid_wifi_control);
	if (err < 0)
		pr_err("error setting wl12xx data\n");

	/* this is the fake regulator that mmc stack use to power of the
	   wifi sdio card via runtime_pm apis */
	wl12xx_vwlan.gpio = get_gpio_by_name(WL12XX_SFI_GPIO_ENABLE_NAME);
	if (wl12xx_vwlan.gpio == -1) {
		pr_err("%s: Unable to find WLAN-enable GPIO in the SFI table\n",
		       __func__);
		return;
	}
	/* format vmmc reg address from sfi table */
	sprintf((char *)wl12xx_vmmc3_supply.dev_name, "0000:00:%02x.%01x",
		(sd_info->addr)>>8, sd_info->addr&0xFF);

	err = platform_device_register(&wl12xx_vwlan_device);
	if (err < 0)
		pr_err("error platform_device_register\n");

	sdhci_pci_request_regulators();
}

void __init *wl12xx_platform_data(void *info)
{
	struct sd_board_info *sd_info;

	sd_info = kmemdup(info, sizeof(*sd_info), GFP_KERNEL);
	if (!sd_info) {
		pr_err("MRST: fail to alloc mem for delayed wl12xx dev\n");
		return NULL;
	}
	intel_delayed_device_register(sd_info,
				      wl12xx_platform_data_init_post_scu);

	return &mid_wifi_control;
}
