/*
 * Copyright (C) 2014 ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <asm/intel-mid.h>

#include "asustek_boardinfo.h"
#include "boardinfo.h"

static bool flush;
func_handle *gfuns = NULL;

static int hardware_id;

struct id_stuffing {
	int value;
	bool flash_en;
	idfuntion_handler func;
};

int set_boardinfo_tab(func_handle *handler_table) {
	int ret = -1;
	if (handler_table != NULL) {
		gfuns = handler_table;
		ret = 0;
	}
	return ret;
}

//-------------------- Tool function end----------------
void start_handle(func_handle *handler_table)
{
	// TODO: get all boardinfo IDs on probe
}

//-------------------- Export function ------------------------
int asustek_boardinfo_get(int fun_num)
{
	if ((fun_num > (FUN_ID_MAX - 1)) || (fun_num < 0)) {
		pr_err("ASUSTek: Error Function number\n");
		return -1;
	}

	if (flush && (gfuns[fun_num].func != NULL)) {
		gfuns[fun_num].value = gfuns[fun_num].func();
		return gfuns[fun_num].value;
	}

	return gfuns[fun_num].value;
}


//-------------------- For ATTR ---------------------------

#define ASUSTEK_PCBID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}

#define ASUSTEK_HARDWAREID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_read, \
}

#define ASUSTEK_PROJECTID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_read, \
}

static ssize_t asustek_boardinfo_store(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	// TODO: for debug file node

	return s - buf;
}

static ssize_t asustek_boardinfo_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	// TODO: for debug file node

	return s - buf;
}

static ssize_t asustek_hardwareid_read(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", asustek_boardinfo_get(RAW_HARDWARE_ID));
}

static ssize_t asustek_projectid_read(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", asustek_boardinfo_get(RAW_PROJECT_ID));
}


ASUSTEK_PCBID_ATTR(asustek_boardinfo);
ASUSTEK_HARDWAREID_ATTR(asustek_hardwareid);
ASUSTEK_PROJECTID_ATTR(asustek_projectid);

static struct attribute *attr_list[] = {
	&asustek_boardinfo_attr.attr,
	&asustek_hardwareid_attr.attr,
	&asustek_projectid_attr.attr,

	// TODO: for debug file node

	NULL,
};

static struct attribute_group attr_group = {
	// TODO: for debug file node
	.attrs = attr_list,
};


static int __init boardinfo_driver_probe(struct platform_device *pdev)
{
	int ret = 0;
	flush = true;
	if(boardinfo_init() < 0) {
		pr_err("ASUSTek: Init board!\n");
		return -1;
	}

	/* create a sysfs interface */
	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);

	if (ret)
		pr_err("ASUSTek: Failed to create sysfs group\n");

	return ret;
}

static int boardinfo_driver_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver asustek_boardinfo_driver __refdata = {
	.probe = boardinfo_driver_probe,
	.remove = boardinfo_driver_remove,
	.driver = {
		.name = "asustek_boardinfo",
		.owner = THIS_MODULE,
	},
};

static int asustek_boardinfo_init(void)
{
	return platform_driver_register(&asustek_boardinfo_driver);
}

#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE)
module_init(asustek_boardinfo_init);
#else
rootfs_initcall(asustek_boardinfo_init);
#endif

MODULE_DESCRIPTION("ASUSTek BoardInfo driver");
MODULE_AUTHOR("Jupiter Chen <Jupiter_Chen@asus.com>");
MODULE_LICENSE("GPL");

