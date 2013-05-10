/*
 * intel_mid_sfi.c: Intel mid sfi related fixes
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
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>

#include <asm/intel-mid.h>

#include "intel_mid_sfi.h"

/* Baytrail fake SFI table */

/* Baytrail OEMB table */
static struct sfi_table_oemb byt_oemb_table = {
	.spid = {
		.customer_id = CUSTOMER_INTEL,
		.vendor_id = VENDOR_INTEL,
		.platform_family_id = INTEL_BYT_TABLET,
		.product_line_id = INTEL_BYT_TABLET_TBD_ENG,
		.hardware_id = BYT_TABLET_TBD_TBD0,
	},
};

/* Baytrail devs table */
static struct sfi_device_table_entry byt_devs_table[] = {
	/*{bus_type, host_num, addr, irq, max_freq, name}*/
	{SFI_DEV_TYPE_IPC, 0, 0, 0, 0, "vlv2_plat_clk"},
	{SFI_DEV_TYPE_I2C, 4, 0x10, 0x0, 0x0, "imx175"},
	{SFI_DEV_TYPE_I2C, 4, 0x36, 0x0, 0x0, "ov2722"},
	{SFI_DEV_TYPE_I2C, 4, 0x53, 0x0, 0x0, "lm3554"},
	{SFI_DEV_TYPE_I2C, 2, 0x1c, 0xff, 250000, "rt5640"},
	{SFI_DEV_TYPE_IPC, 0, 0, 0xff, 250000, "byt_rt5642"},
	/* SD devices */
	{SFI_DEV_TYPE_SD, 0, 0, 0, 0, "bcm43xx_vmmc"},
};

/* Baytrail gpio table */
static struct sfi_gpio_table_entry byt_gpio_table[] = {
	/*{cntl_name, pin_no, pin_name}*/
};

static struct sfi_table_header *get_oem_b_table(void)
{
	struct sfi_table_header *sfi_oemb_table = NULL;

	sfi_oemb_table = (struct sfi_table_header *) &byt_oemb_table;

	return sfi_oemb_table;
}

static struct sfi_table_header *get_oem_0_table(void)
{
	struct sfi_table_header *sfi_oem0_table = NULL;

	return sfi_oem0_table;
}

static struct sfi_table_header *get_devs_table(void)
{
	struct sfi_table_simple *devs_table = NULL;
	struct sfi_table_header *sfi_devs_table = NULL;
	int tot_len = 0;

	if (INTEL_MID_BOARD(1, TABLET, BYT)) {
		tot_len = sizeof(byt_devs_table) +
				sizeof(struct sfi_table_header);
		devs_table = kzalloc(tot_len, GFP_KERNEL);
		if (!devs_table) {
			pr_err("%s(): Error in kzalloc\n", __func__);
			return NULL;
		}
		devs_table->header.len = tot_len;
		memcpy(devs_table->pentry, byt_devs_table,
			sizeof(byt_devs_table));
	}

	sfi_devs_table = (struct sfi_table_header *)devs_table;

	return sfi_devs_table;
}

static struct sfi_table_header *get_gpio_table(void)
{
	struct sfi_table_simple *gpio_table = NULL;
	struct sfi_table_header *sfi_gpio_table = NULL;
	int tot_len = 0;

	if (INTEL_MID_BOARD(1, TABLET, BYT)) {
		tot_len = sizeof(byt_gpio_table) +
				sizeof(struct sfi_table_header);
		gpio_table = kzalloc(tot_len, GFP_KERNEL);
		if (!gpio_table) {
			pr_err("%s(): Error in kzalloc\n", __func__);
			return NULL;
		}
		gpio_table->header.len = tot_len;
		memcpy(gpio_table->pentry, byt_gpio_table,
			sizeof(byt_gpio_table));

	}

	sfi_gpio_table = (struct sfi_table_header *)gpio_table;

	return sfi_gpio_table;
}

/*
 * get_sfi_table() - Returns sfi_table matching the sig.
 */

static struct sfi_table_header *get_sfi_table(char *sig)
{
	struct sfi_table_header *table =  NULL;

	if (!strncmp(sig, SFI_SIG_OEMB, SFI_SIGNATURE_SIZE))
		table = get_oem_b_table();
	else if (!strncmp(sig, SFI_SIG_OEM0, SFI_SIGNATURE_SIZE))
		table = get_oem_0_table();
	else if (!strncmp(sig, SFI_SIG_DEVS, SFI_SIGNATURE_SIZE))
		table = get_devs_table();
	else if (!strncmp(sig, SFI_SIG_GPIO, SFI_SIGNATURE_SIZE))
		table = get_gpio_table();
	else
		table = NULL;

	return table;
}

/*
 * fix_sfi_table() - Looks for fake SFI table that matches the given
 *                   signature and runs handler() on it.
 */

static int fix_sfi_table(char *sig, sfi_table_handler handler)
{
	struct sfi_table_header *table =  NULL;
	int ret = -EINVAL;

	table = get_sfi_table(sig);

	if (!table)
		goto exit;

	ret = handler(table);

exit:
	return ret;
}

/*
 * handle_sfi_table() - Finds the SFI table with given signature and
 *			runs handler() on it.
 */
int handle_sfi_table(char *signature, char *oem_id, char *oem_table_id,
			sfi_table_handler handler)
{
	int ret = -EINVAL;

	ret = sfi_table_parse(signature, oem_id, oem_table_id, handler);

	if (ret) {
		pr_err("Failed to parse %s SFI table\n", signature);
		ret = fix_sfi_table(signature, handler);
	}

	return ret;
}
