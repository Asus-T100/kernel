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

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mc146818rtc.h>
#include <linux/intel_mid_osnib_ilb.h>


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
	return platform_driver_register(&ilb_osnib_driver);
}

static void __exit intel_mid_ilb_osnib_exit(void)
{
	platform_driver_unregister(&ilb_osnib_driver);
}

module_init(intel_mid_ilb_osnib_init);
module_exit(intel_mid_ilb_osnib_exit);
MODULE_AUTHOR("ASUTOSH PATHAK <asutosh.pathak@intel.com>");
MODULE_DESCRIPTION("Intel VLV2 OSNIB iLB CMOS driver");
MODULE_ALIAS("platform:OSNIB_iLB_CMOS");
MODULE_LICENSE("GPL v2");
