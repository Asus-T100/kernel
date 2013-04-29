/*
 * intel_mid_osnib_ilb.c: Driver for the Intel CMOS OSNIB mechanism
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Asutosh Pathak (asutosh.pathak@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 */

#include <linux/intel_mid_osnib_ilb.h>

static struct kobject *osnib_kobj;

/*
 * sysfs
 * FIXME: add proper read and write macro/function
 * FIXME: create common entry for all attributes
 *
 *
*/
static ssize_t fw_update_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int offset = 0;
	u8 fw_update_flag;

	offset = offsetof(struct cmos_osnib, os_flags);
	fw_update_flag = CMOS_READ(CMOS_OSNIB_BASE_ADDR + offset);

	return sprintf(buf, "%d\n",
			test_bit(OSNIB_FW_UPDATE_BIT, &fw_update_flag));
}
/*
 * read os_flags byte, update fw_update bit, write os_flags byte */
static ssize_t fw_update_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int offset = 0;
	u8 fw_update_flag;
	u8 os_flags;

	offset = offsetof(struct cmos_osnib, os_flags);

	os_flags = CMOS_READ(CMOS_OSNIB_BASE_ADDR + offset);

	sscanf(buf, "%du", &fw_update_flag);

	if (fw_update_flag == 1)
		set_bit(OSNIB_FW_UPDATE_BIT, &os_flags);
	else
		clear_bit(OSNIB_FW_UPDATE_BIT, &os_flags);

	pr_info("%s: fw_update = %d\n",
			__func__,
			test_bit(OSNIB_FW_UPDATE_BIT, &os_flags));

	CMOS_WRITE(os_flags, (CMOS_OSNIB_BASE_ADDR + offset));

	return count;
}

static struct kobj_attribute fw_update_attribute =
		__ATTR(fw_update, 0666, fw_update_show, fw_update_store);

static struct attribute *attrs[] = {
	&fw_update_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


/*
 * This writes the reboot reason in the OSNIB (factor and avoid any overlap)
 */
int intel_mid_ilb_write_osnib_rr(u8 rr)
{
	int offset = 0;

	offset = offsetof(struct cmos_osnib, target_mode_attr);

	CMOS_WRITE(rr, (CMOS_OSNIB_BASE_ADDR + offset));

	if (rr != CMOS_READ(CMOS_OSNIB_BASE_ADDR + offset)) {
		pr_err("%s failed", __func__);
		return -EAGAIN;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(intel_mid_ilb_write_osnib_rr);

/*
 * This reads the reboot reason from the OSNIB (factor)
 */
int intel_mid_ilb_read_osnib_rr(u8 *rr)
{
	int offset = 0;

	offset = offsetof(struct cmos_osnib, target_mode_attr);
	*rr = CMOS_READ(CMOS_OSNIB_BASE_ADDR + offset);

	return 0;
}
EXPORT_SYMBOL_GPL(intel_mid_ilb_read_osnib_rr);

static int intel_mid_ilb_osnib_probe(struct platform_device *pdev)
{
	return 0;
}

static int intel_mid_ilb_osnib_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct platform_device_id ilb_osnib_id[] = {
	{ "ilb_osnib", 0 },
	{ }
};
MODULE_DEVICE_TABLE(platform, ilb_osnib_id);


static struct platform_driver ilb_osnib_driver = {
	.probe = intel_mid_ilb_osnib_probe,
	.remove = intel_mid_ilb_osnib_remove,
	.id_table = ilb_osnib_id,
	.driver = {
		.name = "ilb_osnib",
		.owner = THIS_MODULE,
	},
};

static int __init intel_mid_ilb_osnib_init(void)
{
	int retval;

	osnib_kobj = kobject_create_and_add("osnib", firmware_kobj);
	if (!osnib_kobj)
		return 0;

	retval = sysfs_create_group(osnib_kobj, &attr_group);
	if (retval)
		kobject_put(osnib_kobj);

	return platform_driver_register(&ilb_osnib_driver);
}

static void __exit intel_mid_ilb_osnib_exit(void)
{
	kobject_put(osnib_kobj);
	platform_driver_unregister(&ilb_osnib_driver);
}

module_init(intel_mid_ilb_osnib_init);
module_exit(intel_mid_ilb_osnib_exit);
MODULE_AUTHOR("ASUTOSH PATHAK <asutosh.pathak@intel.com>");
MODULE_DESCRIPTION("Intel VLV2 OSNIB iLB CMOS driver");
MODULE_ALIAS("platform:OSNIB_iLB_CMOS");
MODULE_LICENSE("GPL v2");
