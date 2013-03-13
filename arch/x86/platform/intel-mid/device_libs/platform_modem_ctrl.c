/*
 * platform_modem_crl.c: modem control platform data initilization file
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
#include <asm/intel-mid.h>
#include <linux/mdm_ctrl_board.h>
#include <linux/mdm_ctrl.h>

/* MFLD generic infos */
static struct mdm_ctrl_pdata mfld_mid_info = {
	.modem = MODEM_6260,
	.chipctrl = 0x0E0,
	.chipctrlon = 0x4,
	.chipctrloff = 0x2,
	.chipctrl_mask = 0xF8,
	.pre_pwr_down_delay = 60,
	.pwr_down_duration = 20000,
	.early_pwr_on = true,
	.early_pwr_off = false
};

/* CTP generic infos */
static struct mdm_ctrl_pdata ctp_mid_info = {
	.modem = MODEM_6360,
	.chipctrl = 0x100,
	.chipctrlon = 0x10,
	.chipctrloff = 0x10,
	.chipctrl_mask = 0x00,
	.pre_pwr_down_delay = 650,
	.pwr_down_duration = 20000,
	.early_pwr_on = false,
	.early_pwr_off = true
};

/* MRFLD generic infos */
static struct mdm_ctrl_pdata mrfld_mid_info = {
	.modem = MODEM_7160,
	.chipctrl = 0x31,
	.chipctrlon = 0x2,
	.chipctrloff = 0x0,
	.chipctrl_mask = 0xFC,
	.pre_pwr_down_delay = 650,
	.pwr_down_duration = 20000,
	.early_pwr_on = false,
	.early_pwr_off = true
};

/* Non-supported platforms generic infos*/
static struct mdm_ctrl_pdata mdm_ctrl_dummy_info = {
	.is_mdm_ctrl_disabled = true
};

/* IMC XMM6260 generic infos*/
static struct mdm_ctrl_device_info mdm_ctrl_6260_info = {
	.pre_on_delay = 200,
	.on_duration = 60,
	.pre_wflash_delay = 30,
	.pre_cflash_delay = 60,
	.flash_duration = 60,
	.warm_rst_duration = 60
};

/* IMC XMM6360 generic infos*/
/* FIXME: create 6360 specific functions ?
 * Verify if PMIC value should be stored on MRFL
*/
static struct mdm_ctrl_device_info mdm_ctrl_6360_info = {
	.pre_on_delay = 200,
	.on_duration = 60,
	.pre_wflash_delay = 30,
	.pre_cflash_delay = 60,
	.flash_duration = 60,
	.warm_rst_duration = 60
};

/* IMC XMM7160 generic infos*/
static struct mdm_ctrl_device_info mdm_ctrl_7160_info = {
	.pre_on_delay = 200,
	.on_duration = 60,
	.pre_wflash_delay = 30,
	.pre_cflash_delay = 60,
	.flash_duration = 60,
	.warm_rst_duration = 60
};

static int mdm_ctrl_is_supported_ctp(void)
{
	/* FIXME: Revisit on IFWI update*/
	return INTEL_MID_BOARD(1, PHONE, CLVTP) &&
	       (INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, PRO) ||
		INTEL_MID_BOARD(2, PHONE, CLVTP, RHB, ENG));
}

static int mdm_ctrl_is_supported_mfld(void)
{
	return INTEL_MID_BOARD(1, PHONE, MFLD) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, SLP, PRO) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, SLP, ENG);
}

static int mdm_ctrl_is_supported_mrfld(void)
{
	return INTEL_MID_BOARD(1, PHONE, MRFL);
}

void *modem_ctrl_platform_data(void *data)
{
	static struct mdm_ctrl_pdata *mdm_ctrl_info;

	int gpio_rst_out = get_gpio_by_name(GPIO_RST_OUT);
	int gpio_pwr_on  = get_gpio_by_name(GPIO_PWR_ON);
	int gpio_rst_bbn = get_gpio_by_name(GPIO_RST_BBN);
	int gpio_cdump   = get_gpio_by_name(GPIO_CDUMP);
	int gpio_cdump_mrfl   = get_gpio_by_name(GPIO_CDUMP_MRFL);

	int is_ctpscalelt = *(int *)data;

	pr_info("mdm ctrl: platform data setup\n");

	if (mdm_ctrl_is_supported_mfld()) {

		mfld_mid_info.gpio_rst_out = gpio_rst_out;
		mfld_mid_info.gpio_pwr_on = gpio_pwr_on;
		mfld_mid_info.gpio_rst_bbn = gpio_rst_bbn;
		mfld_mid_info.gpio_cdump = gpio_cdump;

		pr_info("mdm ctrl: Getting MFLD datas\n");
		mdm_ctrl_info = (void *)&mfld_mid_info;
		mdm_ctrl_info->device_data = (void *)&mdm_ctrl_6260_info;
		mdm_ctrl_info->is_mdm_ctrl_disabled = false;

	} else if (mdm_ctrl_is_supported_ctp()) {

		ctp_mid_info.gpio_rst_out = gpio_rst_out;
		ctp_mid_info.gpio_pwr_on = gpio_pwr_on;
		ctp_mid_info.gpio_rst_bbn = gpio_rst_bbn;
		ctp_mid_info.gpio_cdump = gpio_cdump;

		pr_info("mdm ctrl: Getting CTP datas\n");
		mdm_ctrl_info = (void *)&ctp_mid_info;
		mdm_ctrl_info->device_data = (void *)&mdm_ctrl_6360_info;
		mdm_ctrl_info->is_mdm_ctrl_disabled = false;

		/* FIXME: Workaround for ctpscalelt. Waiting for IFWI update*/
		if (is_ctpscalelt) {
			pr_info("mdm ctrl: Getting CTPLT datas\n");
			mdm_ctrl_info->modem = MODEM_6268;
			mdm_ctrl_info->device_data = (void *)
							&mdm_ctrl_6260_info;
		}

	} else if (mdm_ctrl_is_supported_mrfld()) {

		mrfld_mid_info.gpio_rst_out = gpio_rst_out;
		mrfld_mid_info.gpio_pwr_on = gpio_pwr_on;
		mrfld_mid_info.gpio_rst_bbn = gpio_rst_bbn;
		mrfld_mid_info.gpio_cdump = gpio_cdump_mrfl;

		pr_info("mdm ctrl: Getting MRFLD datas\n");
		mdm_ctrl_info = (void *)&mrfld_mid_info;
		mdm_ctrl_info->device_data = (void *)&mdm_ctrl_7160_info;
		mdm_ctrl_info->is_mdm_ctrl_disabled = false;

	} else {
		mdm_ctrl_info = (void *)&mdm_ctrl_dummy_info;
	}

	pr_info("mdm ctrl: platform data setup done\n");

	if (!mdm_ctrl_info->is_mdm_ctrl_disabled) {
		pr_info("mdm ctrl: GPIO list : rst_out:%d,"\
				" pwr_on:%d, rst_bbn:%d, cdump:%d\n",
				mdm_ctrl_info->gpio_rst_out,
				mdm_ctrl_info->gpio_pwr_on,
				mdm_ctrl_info->gpio_rst_bbn,
				mdm_ctrl_info->gpio_cdump);
	} else {
		pr_info("mdm ctrl disabled\n");
	}

	return mdm_ctrl_info;
}

