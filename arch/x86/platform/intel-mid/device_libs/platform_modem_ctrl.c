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

#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/init.h>

#include "platform_modem_ctrl.h"

#ifdef CONFIG_MDM_CTRL
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

/* FIXME: To be removed once transition to the new management is over */
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
#endif /* CONFIG_MDM_CTRL */

/* Any code from here should replace the previous one */

/* Conversion modem_name->modem_type table */
static struct modem_base_info mdm_info_table[] = {
	{"XMM_6260", MODEM_6260, UNKNOWN_PMIC, 0, {} },
	{"XMM_6268", MODEM_6268, UNKNOWN_PMIC, 0, {} },
	{"XMM_6360", MODEM_6360, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160_REV1", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160_REV2", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160_REV3", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7260", MODEM_7260, UNKNOWN_PMIC, 0, {} },
	{"RMC_CYGNUS", MODEM_UNSUP, UNKNOWN_PMIC, 0, {} },
	{"RMC_CYGNUS_FFRD", MODEM_UNSUP, UNKNOWN_PMIC, 0, {} },
	{"RMC_CYGNUS_PCI", MODEM_UNSUP, UNKNOWN_PMIC, 0, {} },
	{"RMC_PEGASUS", MODEM_UNSUP, UNKNOWN_PMIC, 0, {} },
	{},
};

static char modem_name[SFI_NAME_LEN];
static char cpu_name[SFI_NAME_LEN];

static ssize_t modem_name_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%s\n", modem_name);
}

static struct kobj_attribute modem_name_attribute = __ATTR_RO(modem_name);

static ssize_t cpu_name_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%s\n", cpu_name);
}

static struct kobj_attribute cpu_name_attribute = __ATTR_RO(cpu_name);

static struct attribute *mdm_attrs[] = {
	&modem_name_attribute.attr,
	&cpu_name_attribute.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group mdm_attr_group = {
	.attrs = mdm_attrs,
};

static struct kobject *telephony_kobj;

int create_sysfs_telephony_entry(void *pdata)
{
	int retval;

	telephony_kobj = kobject_create_and_add("telephony", kernel_kobj);
	if (!telephony_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(telephony_kobj, &mdm_attr_group);
	if (retval)
		kobject_put(telephony_kobj);

	strncpy(modem_name,
			((struct modem_base_info *)pdata)->modem_name,
			SFI_NAME_LEN);

	strncpy(cpu_name,
			((struct modem_base_info *)pdata)->cpu_name,
			SFI_NAME_LEN);

	return retval;
}

void *modem_platform_data(void *data)
{
	int id_mdm_table = *(int *)data;

	struct modem_base_info *mdm_info;

	pr_info("modem_platform_data : modem info setup\n");

	mdm_info = kzalloc(sizeof(*mdm_info), GFP_KERNEL);
	if (!mdm_info)
		return NULL;

	if (mdm_info_table[id_mdm_table].modem_name[0])
		mdm_info->id = mdm_info_table[id_mdm_table].id;
	else
		mdm_info->id = MODEM_UNSUP;

	strncpy(mdm_info->modem_name,
			mdm_info_table[id_mdm_table].modem_name,
			SFI_NAME_LEN);

	mdm_info->pmic = UNKNOWN_PMIC;

#define CASE_PLATFORM(x) { case INTEL_##x##_PHONE:\
	case INTEL_##x##_TABLET:\
				mdm_info->pmic = x##_PMIC;\
	break;\
	}

	switch (spid.platform_family_id) {
	CASE_PLATFORM(MFLD);
	case INTEL_CLVTP_PHONE:
	case INTEL_CLVT_TABLET:
		mdm_info->pmic = CLVT_PMIC;
		break;
	CASE_PLATFORM(MRFL);
	CASE_PLATFORM(BYT);
	default:
		mdm_info->pmic = UNKNOWN_PMIC;
	}

	mdm_info->cpu = intel_mid_identify_cpu();

	switch (mdm_info->cpu) {
	case INTEL_MID_CPU_CHIP_PENWELL:
		strncpy(mdm_info->cpu_name, "PENWELL", SFI_NAME_LEN);
		break;
	case INTEL_MID_CPU_CHIP_CLOVERVIEW:
		strncpy(mdm_info->cpu_name, "CLOVERVIEW", SFI_NAME_LEN);
		break;
	case INTEL_MID_CPU_CHIP_TANGIER:
		strncpy(mdm_info->cpu_name, "TANGIER", SFI_NAME_LEN);
		break;
	case INTEL_MID_CPU_CHIP_VALLEYVIEW2:
		strncpy(mdm_info->cpu_name, "VALLEYVIEW2", SFI_NAME_LEN);
		break;
	default:
		strncpy(mdm_info->cpu_name, "UNKNOWN", SFI_NAME_LEN);
	}

	pr_info("modem_platform_data: modem setup done\n");

	return mdm_info;
}

void sfi_handle_mdm(struct sfi_device_table_entry *pentry,
		struct devs_id *dev)
{
	int modem = 0;
	void *pdata = NULL;

	pr_info("SFI retrieve modem entry, name = %16.16s\n",
			pentry->name);

	/* Retrieve modem ID from modem name */
	while (mdm_info_table[modem].modem_name[0]) {
		if (!strncmp(dev->name,
					mdm_info_table[modem].modem_name,
					SFI_NAME_LEN))
			break;

		modem++;
	}

	if (!mdm_info_table[modem].modem_name[0])
		return;

	pdata = dev->get_platform_data(&modem);

	if (pdata) {
		pr_info("SFI register modem platform data for MCD device %s\n",
				dev->name);
		mcd_register_mdm_info(pdata);
		pr_info("SFI creates sysfs entry for modem named %s\n",
				dev->name);
		create_sysfs_telephony_entry(pdata);
		kfree(pdata);
	}
}
