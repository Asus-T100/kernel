/* intel_scu_flis.c SCU FLIS INTERFACES
 *
 * Copyright (c) 2012,  Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/ipc_device.h>
#include <linux/fs.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_flis.h>

static u32 shim_flis_addr;
static u32 shim_offset;
static u32 shim_data;
static char shim_ops[OPS_STR_LEN];

static u32 pull_value;
static enum pinname_t pin_name;
static char pull_ops[OPS_STR_LEN];

struct intel_scu_flis_info {
	struct pinstruct_t *pin_t;
	int pin_num;
	int initialized;
};

static struct intel_scu_flis_info flis_info;

/* directly write to flis address */
int intel_scu_ipc_write_shim(u32 data, u32 flis_addr, u32 offset)
{
	int ret;
	u32 ipc_wbuf[3];

	ipc_wbuf[0] = flis_addr; /* wbuf[0]: flis address */
	ipc_wbuf[1] = offset;	/* wbuf[1]: register offset */
	ipc_wbuf[2] = data;	/* wbuf[2]: data */

	ret = intel_scu_ipc_command(IPCMSG_SHIM_CONFIG, IPC_CMD_SHIM_WR,
				(u8 *)ipc_wbuf, 12, NULL, 0);
	if (ret)
		pr_err("%s: failed to write shim, flis addr: 0x%x, offset: 0x%x\n",
			__func__, flis_addr, offset);

	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_write_shim);

/* directly read from flis address */
int intel_scu_ipc_read_shim(u32 *data, u32 flis_addr, u32 offset)
{
	int ret;
	u32 ipc_wbuf[2];

	ipc_wbuf[0] = flis_addr;
	ipc_wbuf[1] = offset;

	ret = intel_scu_ipc_command(IPCMSG_SHIM_CONFIG, IPC_CMD_SHIM_RD,
			(u8 *)ipc_wbuf, 8, data, 4);
	if (ret)
		pr_err("%s: failed to read shim, flis addr: 0x%x, offset: 0x%x\n",
			__func__, flis_addr, offset);

	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_read_shim);

/* Configure pin pull up/down */
int config_pin_flis(enum pinname_t name, enum pull_value_t val)
{
	u32 flis_addr, pullup_off;
	int ret;
	int pos;
	u32 data;
	struct intel_scu_flis_info *isfi = &flis_info;

	if (!isfi->initialized)
		return -ENODEV;

	if (name < 0 || name >= isfi->pin_num || val < NONE || val > UP_910K)
		return -EINVAL;

	/* Check if the pin is configurable */
	if (isfi->pin_t[name].valid == false)
		return -EINVAL;

	flis_addr = isfi->pin_t[name].bus_address;
	pullup_off = isfi->pin_t[name].pullup_offset;
	pos = isfi->pin_t[name].pullup_lsb_pos;
	pr_debug("addr = 0x%x, off = 0x%x, pos = %d\n",
		flis_addr, pullup_off, pos);

	/* Read then write, equal to read-mod-write */
	ret = intel_scu_ipc_read_shim(&data, flis_addr, pullup_off);
	if (ret) {
		pr_err("read shim failed, addr = 0x%x, off = 0x%x\n",
			flis_addr, pullup_off);
		goto end;
	}

	pr_debug("read: data = 0x%x\n", data);

	/* 0x3f: each pin has 6 control bits */
	data &= ~(0x3f << pos);

	if (val != NONE)
		data |= ((1 << (val - 1)) << pos);

	pr_debug("write: data = 0x%x\n", data);

	ret = intel_scu_ipc_write_shim(data, flis_addr, pullup_off);
	if (ret) {
		pr_err("write shim failed, addr = 0x%x, off = 0x%x\n",
			flis_addr, pullup_off);
		goto end;
	}

	return 0;

end:
	return ret;
}
EXPORT_SYMBOL_GPL(config_pin_flis);

/* Get pin pull up/down configuration */
int get_pin_flis(enum pinname_t name, u32 *val)
{
	u32 flis_addr, pullup_off;
	int ret;
	int pos;
	u32 data;
	struct intel_scu_flis_info *isfi = &flis_info;

	if (!isfi->initialized)
		return -ENODEV;

	if (name < 0 || name >= isfi->pin_num)
		return -EINVAL;

	if (isfi->pin_t[name].valid == false)
		return -EINVAL;

	flis_addr = isfi->pin_t[name].bus_address;
	pullup_off = isfi->pin_t[name].pullup_offset;
	pos = isfi->pin_t[name].pullup_lsb_pos;
	pr_debug("addr = 0x%x, off = 0x%x, pos = %d\n",
		flis_addr, pullup_off, pos);

	ret = intel_scu_ipc_read_shim(&data, flis_addr, pullup_off);
	if (ret) {
		pr_err("read shim failed, addr = 0x%x, off = 0x%x\n",
			flis_addr, pullup_off);
		goto end;
	}

	*val = (data >> pos) & 0x3f;

	pr_debug("read: data = 0x%x, val = 0x%x\n", data, *val);

	return 0;

end:
	return ret;
}
EXPORT_SYMBOL_GPL(get_pin_flis);

static void flis_generic_store(const char *buf, int type)
{
	u32 tmp;
	int ret;

	/* use decimal for pin number */
	if (type == DBG_PIN_NAME)
		ret = sscanf(buf, "%d", &tmp);
	else
		ret = sscanf(buf, "%x", &tmp);

	if (ret != 1)
		return;

	switch (type) {
	case DBG_SHIM_FLIS_ADDR:
		shim_flis_addr = tmp;
		break;
	case DBG_SHIM_OFFSET:
		shim_offset = tmp;
		break;
	case DBG_SHIM_DATA:
		shim_data = tmp;
		break;
	case DBG_PULL_VAL:
		pull_value = tmp;
		break;
	case DBG_PIN_NAME:
		pin_name = (enum pinname_t)tmp;
		break;
	default:
		break;
	}
}

static ssize_t shim_flis_addr_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	flis_generic_store(buf, DBG_SHIM_FLIS_ADDR);
	return size;
}

static ssize_t shim_flis_addr_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", shim_flis_addr);
}

static ssize_t shim_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	flis_generic_store(buf, DBG_SHIM_OFFSET);
	return size;
}

static ssize_t shim_offset_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", shim_offset);
}

static ssize_t shim_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	flis_generic_store(buf, DBG_SHIM_DATA);
	return size;
}

static ssize_t shim_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", shim_data);
}

static ssize_t shim_ops_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	memset(shim_ops, 0, sizeof(shim_ops));

	ret = sscanf(buf, "%9s", shim_ops);
	if (ret != 1)
		return -EINVAL;

	if (!strncmp("read", shim_ops, OPS_STR_LEN)) {
		ret = intel_scu_ipc_read_shim(&shim_data, shim_flis_addr,
				shim_offset);
	} else if (!strncmp("write", shim_ops, OPS_STR_LEN)) {
		ret = intel_scu_ipc_write_shim(shim_data, shim_flis_addr,
				shim_offset);
	} else {
		dev_err(dev, "Not supported ops\n");
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(dev, "shim config met error\n");
		return ret;
	}

	return size;
}

static ssize_t pull_val_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pull_value);
}

static ssize_t pull_val_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	flis_generic_store(buf, DBG_PULL_VAL);
	return size;
}

static ssize_t pinname_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", pin_name);
}

static ssize_t pinname_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	flis_generic_store(buf, DBG_PIN_NAME);
	return size;
}

static ssize_t pull_ops_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	memset(pull_ops, 0, sizeof(pull_ops));

	ret = sscanf(buf, "%9s", pull_ops);
	if (ret != 1) {
		dev_err(dev, "input error\n");
		return -EINVAL;
	}

	if (!strncmp("get", pull_ops, OPS_STR_LEN))
		ret = get_pin_flis(pin_name, &pull_value);
	else if (!strncmp("set", pull_ops, OPS_STR_LEN))
		ret = config_pin_flis(pin_name, (enum pull_value_t)pull_value);
	else {
		dev_err(dev, "wrong ops\n");
		ret = -EINVAL;
	}

	if (ret) {
		dev_err(dev, "Access flis error, ret = %d\n", ret);
		return ret;
	}

	return size;
}

static DEVICE_ATTR(flis_addr, S_IRUGO|S_IWUSR,
		shim_flis_addr_show, shim_flis_addr_store);
static DEVICE_ATTR(offset, S_IRUGO|S_IWUSR,
		shim_offset_show, shim_offset_store);
static DEVICE_ATTR(data, S_IRUGO|S_IWUSR, shim_data_show, shim_data_store);
static DEVICE_ATTR(ops, S_IWUSR, NULL, shim_ops_store);

static struct attribute *flis_attrs[] = {
	&dev_attr_flis_addr.attr,
	&dev_attr_offset.attr,
	&dev_attr_data.attr,
	&dev_attr_ops.attr,
	NULL,
};

static struct attribute_group flis_attr_group = {
	.name = "flis_debug",
	.attrs = flis_attrs,
};

static DEVICE_ATTR(pin_name, S_IRUGO|S_IWUSR, pinname_show, pinname_store);
static DEVICE_ATTR(pull_val, S_IRUGO|S_IWUSR, pull_val_show, pull_val_store);
static DEVICE_ATTR(pull_ops, S_IWUSR, NULL, pull_ops_store);

static struct attribute *pull_attrs[] = {
	&dev_attr_pin_name.attr,
	&dev_attr_pull_val.attr,
	&dev_attr_pull_ops.attr,
	NULL,
};

static struct attribute_group pull_attr_group = {
	.name = "pull_debug",
	.attrs = pull_attrs,
};

static int __devinit scu_flis_probe(struct ipc_device *ipcdev)
{
	int ret;
	struct intel_scu_flis_info *isfi = &flis_info;
	struct intel_scu_flis_platform_data *pdata = ipcdev->dev.platform_data;

	if (!pdata) {
		dev_err(&ipcdev->dev, "No platform data\n");
		return -EINVAL;
	}

	isfi->pin_t = pdata->pin_t;
	isfi->pin_num = pdata->pin_num;

	if (isfi->pin_t && isfi->pin_num)
		isfi->initialized = 1;

	ret = sysfs_create_group(&ipcdev->dev.kobj, &flis_attr_group);
	if (ret) {
		dev_err(&ipcdev->dev, "Failed to create flis sysfs interface\n");
		goto end;
	}

	ret = sysfs_create_group(&ipcdev->dev.kobj, &pull_attr_group);
	if (ret) {
		dev_err(&ipcdev->dev, "Failed to create pull sysfs interface\n");
		goto err_pull_sysfs;
	}

	return 0;

err_pull_sysfs:
	sysfs_remove_group(&ipcdev->dev.kobj, &flis_attr_group);
end:
	return ret;
}

static int __devexit scu_flis_remove(struct ipc_device *ipcdev)
{
	sysfs_remove_group(&ipcdev->dev.kobj, &pull_attr_group);
	sysfs_remove_group(&ipcdev->dev.kobj, &flis_attr_group);

	return 0;
}

static struct ipc_driver scu_flis_driver = {
	.driver = {
		.name = "intel_scu_flis",
		.owner = THIS_MODULE,
	},
	.probe = scu_flis_probe,
	.remove = __devexit_p(scu_flis_remove),
};

static int __init scu_flis_module_init(void)
{
	return ipc_driver_register(&scu_flis_driver);
}

static void __exit scu_flis_module_exit(void)
{
	ipc_driver_unregister(&scu_flis_driver);
}

module_init(scu_flis_module_init);
module_exit(scu_flis_module_exit);

MODULE_AUTHOR("Ning Li <ning.li@intel.com>");
MODULE_DESCRIPTION("Intel FLIS Access Driver");
MODULE_LICENSE("GPL v2");
