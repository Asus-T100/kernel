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
#include <linux/device.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
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

/* Go through modem names table and return associated numeric Id */
int get_id_from_modem_name(char *mdm_name)
{
	int modem = 0;
	/* Retrieve modem ID from modem name */
	while (mdm_info_table[modem].modem_name[0]) {
		/* Search for mdm_name in table.
		 * Consider support as far as generic name is in the table.
		 */
		if (strstr(mdm_name, mdm_info_table[modem].modem_name))
			break;
		modem++;
	}

	if (!mdm_info_table[modem].modem_name[0])
		return MODEM_UNSUP;

	return mdm_info_table[modem].id;
}

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

	/* Set values with the one retrieved previously
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

/* Modem info to be passed to MCD through platform_device.platform_data */
struct modem_base_info *mcd_reg_info;
struct mdm_ctrl_cpu_data *basic_data;
struct mdm_ctrl_pmic_data *pmic_data;

/**
 * mcd_register_mdm_info - Register information retrieved from SFI table
 * @info: struct including modem name and PMIC.
 */
int mcd_register_mdm_info(struct modem_base_info const *info,
		struct platform_device *pdev)
{
	int ret = 0;
	struct modem_base_info *mcd_reg_tmp_info;
	mcd_reg_tmp_info = kzalloc(sizeof(struct modem_base_info), GFP_ATOMIC);
	if (!mcd_reg_tmp_info) {
		pr_err("SFI can't allocate mcd_reg_tmp_info memory");
		ret = -ENOMEM;
		goto free_mid_info;
	};
	(void) memcpy(mcd_reg_tmp_info, info, sizeof(struct modem_base_info));
	mcd_reg_info = mcd_reg_tmp_info;

	basic_data = kzalloc(sizeof(struct mdm_ctrl_cpu_data), GFP_ATOMIC);
	if (!basic_data) {
		pr_err("SFI can't allocate mdm_ctrl_cpu_data memory");
		ret = -ENOMEM;
		goto free_mid_info;
	};

	pr_info("%s : cpu info setup\n", __func__);
	/* Check if the cpu is supported */
	switch (mcd_reg_info->cpu) {
	case INTEL_MID_CPU_CHIP_PENWELL:
		basic_data->gpio_cdump = get_gpio_by_name(GPIO_CDUMP);
		basic_data->early_pwr_on = true;
		basic_data->early_pwr_off = false;
		break;
	case INTEL_MID_CPU_CHIP_CLOVERVIEW:
		basic_data->gpio_cdump = get_gpio_by_name(GPIO_CDUMP);
		basic_data->early_pwr_on = false;
		basic_data->early_pwr_off = true;
		break;
	case INTEL_MID_CPU_CHIP_TANGIER:
		basic_data->gpio_cdump = get_gpio_by_name(GPIO_CDUMP_MRFL);
		basic_data->early_pwr_on = false;
		basic_data->early_pwr_off = true;
		break;
	case INTEL_MID_CPU_CHIP_ANNIEDALE:
		basic_data->gpio_cdump = get_gpio_by_name(GPIO_CDUMP_MRFL);
		basic_data->early_pwr_on = false;
		basic_data->early_pwr_off = true;
		break;
	default:
		pr_err("%s: Platform not supported %d", __func__,
				mcd_reg_info->cpu);
		ret = -ENODEV;
		goto free_mid_info;
	}

	basic_data->gpio_rst_out = get_gpio_by_name(GPIO_RST_OUT);
	basic_data->gpio_pwr_on = get_gpio_by_name(GPIO_PWR_ON);
	basic_data->gpio_rst_bbn = get_gpio_by_name(GPIO_RST_BBN);

	mcd_reg_info->data = basic_data;
	pdev->dev.platform_data = mcd_reg_info;
	return 0;

free_mid_info:
	if (!pmic_data)
		kfree(pmic_data);
	pmic_data = NULL;
	if (!basic_data)
		kfree(basic_data);
	basic_data = NULL;
	if (!mcd_reg_info)
		kfree(mcd_reg_info);
	mcd_reg_info = NULL;

	return ret;
}

/*
 * modem_platform_data - Platform data builder for modem devices
 * @data: pointer to modem name retrieved in sfi table
 */
void *modem_platform_data(void *data)
{
	char *mdm_name = (char *)data;
	int modem = 0;

	struct modem_base_info *mdm_info;

	pr_debug("%s: modem info setup\n", __func__);
	mdm_info = kzalloc(sizeof(*mdm_info), GFP_KERNEL);
	if (!mdm_info) {
		pr_err("SFI can't allocate modem_base_info memory");
		return NULL;
	}

	pmic_data = kzalloc(sizeof(struct mdm_ctrl_pmic_data), GFP_ATOMIC);
	if (!pmic_data) {
		pr_err("SFI can't allocate mdm_ctrl_pmic_data memory");
		goto free_mid_info;
	};

	/* Retrieve modem ID from modem name */
	modem = get_id_from_modem_name(mdm_name);
	if (modem == MODEM_UNSUP) {
		pr_err("SFI can't find modem name.\n");
		goto free_mid_info;
	}
	mdm_info->id = modem;

	/*
	 * Real name provisioning. Retrieved from devs_id table.
	 */
	strncpy(mdm_info->modem_name, mdm_name, SFI_NAME_LEN);

#define CASE_PLATFORM(x) { case INTEL_##x##_PHONE:\
	case INTEL_##x##_TABLET:\
				pmic_data->id = x##_PMIC;\
	break;\
	}

	switch (spid.platform_family_id) {
	CASE_PLATFORM(MFLD);
	case INTEL_CLVTP_PHONE:
	case INTEL_CLVT_TABLET:
		pmic_data->id = CLVT_PMIC;
		break;
	CASE_PLATFORM(MRFL);
	CASE_PLATFORM(BYT);
	default:
		pmic_data->id = UNKNOWN_PMIC;
	}

	mdm_info->pmic = pmic_data;
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
			pmic_data->id,
			mdm_info->cpu_name);

	return mdm_info;
free_mid_info:
	if (!mdm_info)
		kfree(mdm_info);
	if (!pmic_data)
		kfree(pmic_data);
	pmic_data = NULL;
	return NULL;
}

static struct platform_device mcd_device = {
	.name	= DEVICE_NAME,
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

#ifdef CONFIG_ACPI
acpi_status get_acpi_param(acpi_handle handle, int type, char *id,
			   union acpi_object **result)
{
	acpi_status status = AE_OK;
	struct acpi_buffer obj_buffer = {ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *out_obj;

	status = acpi_evaluate_object(handle, id, NULL, &obj_buffer);
	pr_err("%s: acpi_evaluate_object, status:%d\n", __func__,
			status);
	if (ACPI_FAILURE(status)) {
		pr_err("%s: ERROR %d evaluating ID:%s\n", __func__,
			status, id);
		goto Error;
	}

	out_obj = obj_buffer.pointer;
	if (!out_obj || out_obj->type != type) {
		pr_err("%s: Invalid type:%d for Id:%s\n", __func__, type, id);
		status = AE_BAD_PARAMETER;
		goto Error;
	} else {
		*result = out_obj;
	}

Error:
	return status;
}
#endif

/*
 * Access ACPI resources/data to populate global object mcd_reg_info
 *
 * @pdev : The platform device object to identify ACPI data.
 */
int retrieve_acpi_modem_data(struct platform_device *pdev)
{
	int ret = -ENODEV;
#ifdef CONFIG_ACPI
	struct modem_base_info *mcd_reg_tmp_info;
	acpi_status status = AE_OK;
	acpi_handle handle;
	union acpi_object *out_obj;
	union acpi_object *item;

	if (!pdev) {
		pr_err("%s: platform device is NULL.", __func__);
		return -ENODEV;
	}

	/* Get ACPI handle */
	handle = DEVICE_ACPI_HANDLE(&pdev->dev);

	mcd_reg_tmp_info = kzalloc(sizeof(struct modem_base_info), GFP_ATOMIC);
	if (!mcd_reg_tmp_info) {
		pr_err("%s: can't allocate mcd_reg_tmp_info memory", __func__);
		ret = -ENOMEM;
		goto Free_mdm_info;
	}
	mcd_reg_info = mcd_reg_tmp_info;

	basic_data = kzalloc(sizeof(struct mdm_ctrl_cpu_data), GFP_ATOMIC);
	if (!basic_data) {
		pr_err("%s: can't allocate mdm_ctrl_cpu_data memory", __func__);
		ret = -ENOMEM;
		goto Free_mdm_info;
	}

	pmic_data = kzalloc(sizeof(struct mdm_ctrl_pmic_data), GFP_ATOMIC);
	if (!pmic_data) {
		pr_err("%s: can't allocate mdm_ctrl_pmic_data memory",
		       __func__);
		goto Free_mdm_info;
	};

	pr_info("%s: Getting platform data...\n", __func__);

	/* CPU name */
	status = get_acpi_param(handle, ACPI_TYPE_STRING, "CPU", &out_obj);
	if (ACPI_FAILURE(status)) {
		pr_err("%s: ERROR evaluating CPU Name\n", __func__);
		goto Free_mdm_info;
	}
	strncpy(mcd_reg_info->cpu_name, out_obj->string.pointer, SFI_NAME_LEN);

	pr_info("%s: Found CPU name:%s\n", __func__, mcd_reg_info->cpu_name);

	/* CPU Id */
	if (strstr(mcd_reg_info->cpu_name, "ValleyView2")) {
		mcd_reg_info->cpu = INTEL_MID_CPU_CHIP_VALLEYVIEW2;
	} else {
		pr_err("%s: ERROR CPU name %s Not supported!\n", __func__,
		mcd_reg_info->cpu_name);
		goto Free_mdm_info;
	}

	/* Retrieve Modem name from ACPI */
	status = get_acpi_param(handle, ACPI_TYPE_STRING, "MDMN", &out_obj);
	if (ACPI_FAILURE(status)) {
		pr_err("%s: ERROR evaluating Modem Name\n", __func__);
		goto Free_mdm_info;
	}
	strncpy(mcd_reg_info->modem_name,
		out_obj->string.pointer,
		SFI_NAME_LEN);

	mcd_reg_info->id = get_id_from_modem_name(mcd_reg_info->modem_name);
	if (mcd_reg_info->id == MODEM_UNSUP) {
		pr_err("%s: ERROR Modem %s Not supported!\n", __func__,
			mcd_reg_info->modem_name);
		goto Free_mdm_info;
	}

	/* PMIC */
	if (mcd_reg_info->cpu == INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		pmic_data->id = BYT_PMIC;

	status = get_acpi_param(handle, ACPI_TYPE_PACKAGE, "PMIC", &out_obj);
	if (ACPI_FAILURE(status)) {
		pr_err("%s: ERROR evaluating PMIC info\n", __func__);
		goto Free_mdm_info;
	}

	item = &(out_obj->package.elements[0]);
	pmic_data->chipctrl = (int)item->integer.value;
	item = &(out_obj->package.elements[1]);
	pmic_data->chipctrlon = (int)item->integer.value;
	item = &(out_obj->package.elements[2]);
	pmic_data->chipctrloff = (int)item->integer.value;
	item = &(out_obj->package.elements[3]);
	pmic_data->chipctrl_mask = (int)item->integer.value;
	pr_info("%s: Retrieved PMIC values:Reg:%x, On:%x, Off:%x, Mask:%x\n",
		__func__, pmic_data->chipctrl, pmic_data->chipctrlon,
		pmic_data->chipctrloff, pmic_data->chipctrl_mask);

	mcd_reg_info->pmic = pmic_data;

	pr_info("%s: Modem:%16.16s id:%d pmic:0x%x cpu:%s\n",
		__func__,
		mcd_reg_info->modem_name,
		mcd_reg_info->id,
		pmic_data->chipctrl,
		mcd_reg_info->cpu_name);

	pr_info("%s: cpu info setup\n", __func__);

	basic_data->gpio_pwr_on = acpi_get_gpio_by_index(&pdev->dev, 0, NULL);
	basic_data->gpio_cdump = acpi_get_gpio_by_index(&pdev->dev, 1, NULL);
	basic_data->gpio_rst_out = acpi_get_gpio_by_index(&pdev->dev, 2, NULL);
	basic_data->gpio_rst_bbn = acpi_get_gpio_by_index(&pdev->dev, 3, NULL);

	pr_info("%s:Setup GPIOs(PO:%d, RO:%d, RB:%d, CD:%d)",
		__func__,
		basic_data->gpio_pwr_on,
		basic_data->gpio_rst_out,
		basic_data->gpio_rst_bbn,
		basic_data->gpio_cdump);

	status = get_acpi_param(handle, ACPI_TYPE_PACKAGE, "EPWR", &out_obj);
	if (ACPI_FAILURE(status)) {
		pr_err("%s: ERROR evaluating Early PWR info info\n", __func__);
		goto Free_mdm_info;
	}

	item = &(out_obj->package.elements[0]);
	basic_data->early_pwr_on = (int)item->integer.value;
	item = &(out_obj->package.elements[1]);
	basic_data->early_pwr_off = (int)item->integer.value;

	pr_info("%s:Setup Early Power: On:%d, Off:%d", __func__,
		       basic_data->early_pwr_on,
		       basic_data->early_pwr_off);

	mcd_reg_info->data = basic_data;

	return 0;

Free_mdm_info:
	pr_err("%s: ERROR retrieving data from ACPI!!!\n", __func__);
	if (!pmic_data)
		kfree(pmic_data);
	pmic_data = NULL;
	if (!basic_data)
		kfree(basic_data);
	basic_data = NULL;
	if (!mcd_reg_info)
		kfree(mcd_reg_info);
#endif
	mcd_reg_info = NULL;
	return ret;
}

/*
 * Entry point from MCD to populate modem parameters.
 *
 * @pdev : The platform device object to identify ACPI data.
 */
int retrieve_modem_platform_data(struct platform_device *pdev)
{
	int ret = -ENODEV;

	if (!pdev) {
		pr_err("%s: platform device is NULL!!!\n", __func__);
		return ret;
	}

	if (ACPI_HANDLE(&pdev->dev)) {
		pr_err("%s: Retrieving modem info from ACPI for device %s\n",
			__func__, pdev->name);
		ret = retrieve_acpi_modem_data(pdev);
	} else {
		pr_err("%s: platform device is NOT ACPI!!!\n", __func__);
	}

	if (ret)
		goto Error;

	/* Store modem parameters in platform device */
	pdev->dev.platform_data = mcd_reg_info;

	if (!telephony_kobj) {
		pr_err("%s: creates sysfs entry for device named %s\n",
				__func__, pdev->name);
		create_sysfs_telephony_entry(pdev->dev.platform_data);
	} else {
		pr_err("%s: Unexpected entry for device named %s\n",
				__func__, pdev->name);
	}

	return 0;
Error:
	return ret;
}
