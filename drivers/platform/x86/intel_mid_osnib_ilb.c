/*
 * drivers/platform/x86/intel_mid_osnib_ilb.c
 *
 * Copyright (C) 2013 Intel Corp
 * Author: Asutosh Pathak <asutosh.pathak@intel.com>
 * Author: Vincent Tinelli (vincent.tinelli@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/intel_mid_osnib_ilb.h>

static struct kobject *osnib_kobj;

static struct cmos_osnib osnib_buffer;

static struct bootflow_type bootflows[] = {
	{ END_USER, "End user" },
	{ MANUFACTURING, "Manufacturing" },
	{ GPP_CERTIFICATION, "3GPP certification" },
};

static struct wake_src wake_srcs[] = {
	{ WAKE_BATT_INSERT, "battery inserted" },
	{ WAKE_PWR_BUTTON_PRESS, "power button pressed" },
	{ WAKE_RTC_TIMER, "rtc timer" },
	{ WAKE_USB_CHRG_INSERT, "usb charger inserted" },
	{ WAKE_RESERVED, "reserved" },
	{ WAKE_REAL_RESET, "real reset" },
	{ WAKE_PLATFORM_RESET, "platform reset" },
	{ WAKE_UNKNOWN, "unknown" },
	{ WAKE_KERNEL_WATCHDOG_RESET, "watchdog reset" },
	{ WAKE_SECURITY_WATCHDOG_RESET, "security watchdog reset" },
	{ WAKE_WATCHDOG_COUNTER_EXCEEDED, "watchdog counter exceeded" },
	{ WAKE_POWER_SUPPLY_DETECTED, "power supply detected" },
	{ WAKE_FASTBOOT_BUTTONS_COMBO, "fastboot combo" },
	{ WAKE_NO_MATCHING_OSIP_ENTRY, "no matching osip entry" },
	{ WAKE_CRITICAL_BATTERY, "critical battery" },
	{ WAKE_INVALID_CHECKSUM, "invalid checksum" },
	{ WAKE_FORCED_RESET, "forced reset"},
	{ WAKE_ACDC_CHRG_INSERT, "ac charger inserted" },
};

static struct target_os oses[] = {
	{ "main", MAIN },
	{ "charging", CHARGING  },
	{ "recovery", RECOVERY },
	{ "fastboot", FASTBOOT },
	{ "bootloader", FASTBOOT },
	{ "factory", FACTORY },
};

static int osnib_ilb_reboot_notify(struct notifier_block *notifier,
				     unsigned long what, void *data)
{
	int ret = NOTIFY_DONE;
	char *target = (char *) data;

	switch (what) {
	case SYS_RESTART:
		intel_mid_ilb_write_osnib_rr(target);
		break;
	case SYS_HALT:
	case SYS_POWER_OFF:
	default:
		break;
	}

	return ret;
}

static struct notifier_block osnib_ilb_reboot_notifier = {
	.notifier_call = osnib_ilb_reboot_notify,
};

static ssize_t fw_update_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	u8 fw_update;

	fw_update = intel_mid_ilb_read_osnib_field(&osnib_buffer,
			offsetof(struct cmos_osnib, os_to_fw.bf));
	return sprintf(buf, "%d\n", OSNIB_FW_UPDATE_GET_VALUE(fw_update));
}
static ssize_t fw_update_store(struct kobject *kobj,
		struct kobj_attribute *attr,
		const char *buf, size_t count)
{
	int fw_update;

	sscanf(buf, "%du", &fw_update);
	intel_mid_ilb_write_osnib_field(&osnib_buffer,
			offsetof(struct cmos_osnib, os_to_fw.bf),
			OSNIB_FW_UPDATE_SET_VALUE(fw_update));
	intel_mid_ilb_write_osnib_checksum(&osnib_buffer);
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

void intel_mid_ilb_write_osnib_field(struct cmos_osnib *osnib,
		int offset, u8 value)
{
	u8 *b;

	pr_err("OSNIB: write byte[%d] = %u\n", offset, value);
	b = (u8 *) osnib;
	*(b + offset) = value;

	CMOS_WRITE(value, OSNIB_CMOS_BASE_ADDR + offset);
}

void intel_mid_ilb_write_osnib(struct cmos_osnib *osnib)
{
	int i = 0;
	u8 *b;

	osnib->checksum = intel_mid_ilb_checksum_osnib(osnib);
	b = (u8 *) osnib;
	for (i = 0; i < OSNIB_INTEL_SIZE ; i++, b++)
		intel_mid_ilb_write_osnib_field(&osnib_buffer, i, *b);
}

void intel_mid_ilb_write_osnib_checksum(struct cmos_osnib *osnib)
{
	intel_mid_ilb_write_osnib_field(&osnib_buffer,
			offsetof(struct cmos_osnib, checksum),
			intel_mid_ilb_checksum_osnib(&osnib_buffer));
}

u8 intel_mid_ilb_read_osnib_field(struct cmos_osnib *osnib,
		int offset)
{
	return CMOS_READ(OSNIB_CMOS_BASE_ADDR + offset);
}


int intel_mid_ilb_read_osnib(struct cmos_osnib *osnib)
{
	int i = 0;
	u8 *b;

	memset(osnib, 0, OSNIB_INTEL_SIZE);
	b = (u8 *) osnib;
	for (i = 0; i < OSNIB_INTEL_SIZE ; i++, b++)
		*b = intel_mid_ilb_read_osnib_field(osnib, i);

	intel_mid_ilb_dump_osnib(osnib);

	return intel_mid_ilb_is_osnib_valid(osnib);
}

void intel_mid_ilb_dump_osnib(struct cmos_osnib *osnib)
{
	int i = 0;
	u8 *b;

	b = (u8 *) osnib;
	for (i = 0; i < OSNIB_INTEL_SIZE ; i++, b++)
		pr_err("OSNIB: byte %d = 0x%02x\n", i, *b);
}

void intel_mid_ilb_reset_osnib(struct cmos_osnib *osnib)
{

	pr_err("Reset OSNIB content as checksum failed with value 0x%02x\n",
			osnib_buffer.checksum);
	memset(osnib, 0, OSNIB_INTEL_SIZE);
	osnib->checksum = 0x1;
	intel_mid_ilb_write_osnib(osnib);
}

u8 intel_mid_ilb_checksum_osnib(struct cmos_osnib *osnib)
{
	int i = 0;
	u8 checksum = 0;
	u8 *b;

	b = (u8 *) osnib;
	for (i = 0; i < OSNIB_INTEL_SIZE - 1; i++, b++)
		checksum += *b;

	pr_err("OSNIB checksum: 0x%02x\n", checksum);
	return checksum;
}

int intel_mid_ilb_is_osnib_valid(struct cmos_osnib *osnib)
{
	return (intel_mid_ilb_checksum_osnib(osnib) == osnib->checksum);
}

const char *intel_mid_ilb_get_os_key(u8 value)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(oses); i++)
		if (value == oses[i].id)
			return oses[i].name;

	pr_warning("%s: target os %d not found", __func__, (int) value);

	return "";
}

u8 intel_mid_ilb_get_os_value(const char *target_os)
{
	int i;
	pr_info("%s: %s\n", __func__, target_os);
	for (i = 0; i < ARRAY_SIZE(oses); i++)
		if (!strcmp(oses[i].name, target_os))
			return oses[i].id;

	pr_warning("%s: target os %s not found, default to main", __func__,
			target_os);

	return (u8) MAIN;
}

const char *intel_mid_ilb_get_bootflow_value(u8 id)
{
	int i;

	for (i = 0 ; i < ARRAY_SIZE(bootflows); i++)
		if (id == bootflows[i].id)
			return bootflows[i].name;

	return "";
}

int intel_mid_ilb_write_osnib_rr(const char *target)
{

	intel_mid_ilb_write_osnib_field(&osnib_buffer,
			offsetof(struct cmos_osnib, os_to_fw.target_mode),
			intel_mid_ilb_get_os_value(target));
	intel_mid_ilb_write_osnib_checksum(&osnib_buffer);

	return 0;
}

const char *intel_mid_ilb_read_osnib_rr(void)
{
	return intel_mid_ilb_get_os_key(osnib_buffer.os_to_fw.target_mode);
}

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

	/*
	 * FIXME: shouldn't be cpu based, ilb flag needed
	 */
	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_VALLEYVIEW2:
		pr_info("%s: register %s\n",
				__func__, ilb_osnib_driver.driver.name);
		break;
	default:
		pr_info("%s: not registered %s\n",
				__func__, ilb_osnib_driver.driver.name);
		return 0;
	}

	intel_mid_ilb_write_osnib_rr(oses[0].name);

	if (register_reboot_notifier(&osnib_ilb_reboot_notifier))
		pr_warning("%s: can't register reboot_notifier\n", __func__);

	if (!intel_mid_ilb_read_osnib(&osnib_buffer))
		intel_mid_ilb_reset_osnib(&osnib_buffer);

	pr_info("%s: boot_mode = '%s'\n", __func__,
		intel_mid_ilb_get_bootflow_value(osnib_buffer.bootflow.type));

	pr_info("%s: wake_src  =  '%s'\n", __func__,
		wake_srcs[osnib_buffer.fw_to_os.wake_src].name);

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
	/*
	 * FIXME: shouldn't be cpu based, ilb flag needed
	 */
	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_VALLEYVIEW2:
		pr_info("%s: unregister %s\n",
				__func__, ilb_osnib_driver.driver.name);
		break;
	default:
		pr_info("%s: not unregistered %s\n",
				__func__, ilb_osnib_driver.driver.name);
		return 0;
	}

	unregister_reboot_notifier(&osnib_ilb_reboot_notifier);
	if (!intel_mid_ilb_read_osnib(&osnib_buffer)) {
		intel_mid_ilb_reset_osnib(&osnib_buffer);
		BUG();
	}

	kobject_put(osnib_kobj);
	platform_driver_unregister(&ilb_osnib_driver);
}

module_init(intel_mid_ilb_osnib_init);
module_exit(intel_mid_ilb_osnib_exit);
MODULE_DESCRIPTION("Intel CMOS OSNIB driver");
MODULE_LICENSE("GPL v2");
