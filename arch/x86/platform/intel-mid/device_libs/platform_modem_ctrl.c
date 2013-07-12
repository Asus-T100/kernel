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

#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
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

/* Conversion table: modem_name->modem_type */
static struct modem_base_info mdm_info_table[] = {
	/* IMC products */
	{"XMM_6260", MODEM_6260, UNKNOWN_PMIC, 0, {} },
	{"XMM_6268", MODEM_6268, UNKNOWN_PMIC, 0, {} },
	{"XMM_6360", MODEM_6360, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160_REV1", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160_REV3", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160_REV3_5", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7160_REV4", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	{"XMM_7260", MODEM_7260, UNKNOWN_PMIC, 0, {} },
	/* Any other IMC products: set to 7160 by default */
	{"XMM", MODEM_7160, UNKNOWN_PMIC, 0, {} },
	/* RMC products */
	{"CYGNUS", MODEM_UNSUP, UNKNOWN_PMIC, 0, {} },
	{"PEGASUS", MODEM_UNSUP, UNKNOWN_PMIC, 0, {} },
	/* Any other RMC products */
	{"RMC", MODEM_UNSUP, UNKNOWN_PMIC, 0, {} },
	{},
};

/*
 * Element to be read through sysfs entry
 */
static char modem_name[SFI_NAME_LEN];
static char cpu_name[SFI_NAME_LEN];

/*
 * Modem name accessor
 */
static ssize_t modem_name_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%s\n", modem_name);
}

/* Read-only element */
static struct kobj_attribute modem_name_attribute = __ATTR_RO(modem_name);

/*
 * Cpu-name accessor
 */
static ssize_t cpu_name_show(struct kobject *kobj,
		struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%s\n", cpu_name);
}

/* Read-only element */
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

	/* Creating telephony directory */
	telephony_kobj = kobject_create_and_add("telephony", kernel_kobj);
	if (!telephony_kobj)
		return -ENOMEM;

	/* Create the files associated with this kobject */
	retval = sysfs_create_group(telephony_kobj, &mdm_attr_group);
	if (retval)
		kobject_put(telephony_kobj);

	/* Set values with the one retrived previously
	 * through modem_platform_data call.
	 */
	strncpy(modem_name,
			((struct modem_base_info *)pdata)->modem_name,
			SFI_NAME_LEN);

	strncpy(cpu_name,
			((struct modem_base_info *)pdata)->cpu_name,
			SFI_NAME_LEN);

	return retval;
}

struct modem_base_info *mcd_reg_info;

struct mdm_ctrl_cpu_data basic_data;

/**
 * mcd_register_mdm_info - Register information retrieved from SFI table
 * @info: struct including modem name and PMIC.
 */
int mcd_register_mdm_info(struct modem_base_info const *info,
	struct platform_device *pdev)
{
	struct modem_base_info *mcd_reg_tmp_info;
	mcd_reg_tmp_info = kzalloc(sizeof(struct modem_base_info), GFP_ATOMIC);
	if (!mcd_reg_tmp_info) {
		pr_err("SFI can't allocate mcd_reg_tmp_info memory");
		return -ENOMEM;
	};

	(void) memcpy(mcd_reg_tmp_info, info, sizeof(struct modem_base_info));

	mcd_reg_info = mcd_reg_tmp_info;

	pr_info("%s : cpu info setup\n", __func__);

	/* Check if the cpu is supported */
	switch (mcd_reg_info->cpu) {
	case INTEL_MID_CPU_CHIP_PENWELL:
		basic_data.gpio_cdump = get_gpio_by_name(GPIO_CDUMP);
		basic_data.early_pwr_on = true;
		basic_data.early_pwr_off = false;
		break;
	case INTEL_MID_CPU_CHIP_CLOVERVIEW:
		basic_data.gpio_cdump = get_gpio_by_name(GPIO_CDUMP);
		basic_data.early_pwr_on = false;
		basic_data.early_pwr_off = true;
		break;
	case INTEL_MID_CPU_CHIP_TANGIER:
		basic_data.gpio_cdump = get_gpio_by_name(GPIO_CDUMP_MRFL);
		basic_data.early_pwr_on = false;
		basic_data.early_pwr_off = true;
		break;
	case INTEL_MID_CPU_CHIP_ANNIEDALE:
		basic_data.gpio_cdump = get_gpio_by_name(GPIO_CDUMP_MRFL);
		basic_data.early_pwr_on = false;
		basic_data.early_pwr_off = true;
		break;
	default:
		pr_err("%s: Platform not supported %d", __func__,
				mcd_reg_info->cpu);
		goto free_mid_info;
	}

	basic_data.gpio_rst_out = get_gpio_by_name(GPIO_RST_OUT);
	basic_data.gpio_pwr_on = get_gpio_by_name(GPIO_PWR_ON);
	basic_data.gpio_rst_bbn = get_gpio_by_name(GPIO_RST_BBN);

	mcd_reg_info->data = &basic_data;
	pdev->dev.platform_data = mcd_reg_info;
	return 0;

free_mid_info:
	kfree(mcd_reg_info);
	mcd_reg_info = NULL;
	return -ENODEV;
}

/*
 * modem_platform_data - Platform data builder for modem devices
 * @data: pointer to modem name retrived in sfi table
 */
void *modem_platform_data(void *data)
{
	char *mdm_name = data;
	int modem = 0;

	struct modem_base_info *mdm_info;

	pr_debug("%s: modem info setup\n", __func__);

	mdm_info = kzalloc(sizeof(*mdm_info), GFP_KERNEL);
	if (!mdm_info)
		return NULL;

	/* Retrieve modem ID from modem name */
	while (mdm_info_table[modem].modem_name[0]) {
		/* Search for mdm_name in table.
		 * Consider support as far as generic name is in the table.
		 */
		if (strstr(mdm_name, mdm_info_table[modem].modem_name))
			break;
		modem++;
	}

	if (!mdm_info_table[modem].modem_name[0]) {
		kfree(mdm_info);
		return NULL;
	}

	mdm_info->id = mdm_info_table[modem].id;

	/*
	 * Real name provisioning. Retrieved from devs_id table.
	 */
	strncpy(mdm_info->modem_name, mdm_name, SFI_NAME_LEN);

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

	/*
	 * Provisionning cpu name in order to simplify telephony
	 * components life. All needed info in one place.
	 */
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
	case INTEL_MID_CPU_CHIP_ANNIEDALE:
		strncpy(mdm_info->cpu_name, "ANNIEDALE", SFI_NAME_LEN);
		break;
	default:
		strncpy(mdm_info->cpu_name, "UNKNOWN", SFI_NAME_LEN);
	}

	pr_info("%s: modem setup done\n", __func__);
	pr_debug("%s name:%16.16s id:%d pmic:%d cpu:%s\n",
			__func__,
			mdm_info->modem_name,
			mdm_info->id,
			mdm_info->pmic,
			mdm_info->cpu_name);

	return mdm_info;
}

static struct platform_device mcd_device = {
	.name		= DEVICE_NAME,
	.id		= -1,
};

/*
 * sfi_handle_mdm - specific handler for intel's platform modem devices.
 * @pentry: sfi table entry
 * @dev: device id retrived by sfi dev parser
 */
void sfi_handle_mdm(struct sfi_device_table_entry *pentry,
		struct devs_id *dev)
{
	void *pdata = NULL;

	pr_info("SFI retrieve modem entry, name = %16.16s\n",
			pentry->name);

	pdata = dev->get_platform_data(dev->name);

	if (pdata) {
		pr_info("SFI register modem platform data for MCD device %s\n",
				dev->name);
		mcd_register_mdm_info(pdata, &mcd_device);
		platform_device_register(&mcd_device);
		if (!telephony_kobj) {
			pr_info("SFI creates sysfs entry for modem named %s\n",
					dev->name);
			create_sysfs_telephony_entry(pdata);
		} else {
			pr_info("Unexpected SFI entry for modem named %s\n",
					dev->name);
		}
		kfree(pdata);
	}
}
