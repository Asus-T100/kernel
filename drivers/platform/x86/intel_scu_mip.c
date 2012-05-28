/*
 * intel_scu_mip.c: Driver for the Intel scu mip and umip access
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Shijie Zhang (shijie.zhang@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ipc_device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <asm/intel_scu_ipc.h>

#define IPC_MIP_BASE     0xFFFD8000	/* sram base address for mip accessing*/
#define IPC_MIP_MAX_ADDR 0x1000

static void __iomem *intel_mip_base;

static int read_mip(u8 *data, int len, int offset, int issigned)
{
	int ret;
	u32 sptr, dptr, cmd, cmdid, data_off;

	dptr = offset;
	sptr = (len + 3) / 4;

	cmdid = issigned ? IPC_CMD_SMIP_RD : IPC_CMD_UMIP_RD;
	cmd = 4 << 16 | cmdid << 12 | IPCMSG_MIP_ACCESS;

	do {
		ret = intel_scu_ipc_raw_cmd(cmd, 0, NULL, 0, &data_off, 1,
				dptr, sptr);
		if (ret == -EIO)
			msleep(20);
	} while (ret == -EIO);

	if (!ret)
		memcpy(data, intel_mip_base + data_off, len);

	return ret;
}

int intel_scu_ipc_read_mip(u8 *data, int len, int offset, int issigned)
{
	int ret;

	/* Only SMIP read for Cloverview is supported */
	if ((intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW)
			&& (issigned != 1))
		return -EINVAL;

	if (!intel_mip_base)
		return -ENODEV;

	if (offset + len > IPC_MIP_MAX_ADDR)
		return -EINVAL;

	intel_scu_ipc_lock();
	ret = read_mip(data, len, offset, issigned);
	intel_scu_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_read_mip);

int intel_scu_ipc_write_umip(u8 *data, int len, int offset)
{
	int ret, offset_align;
	int len_align = 0;
	u32 dptr, sptr, cmd;
	u8 *buf = NULL;

	/* Cloverview don't need UMIP access through IPC */
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_CLOVERVIEW)
		return -EINVAL;

	if (!intel_mip_base)
		return -ENODEV;

	if (offset + len > IPC_MIP_MAX_ADDR)
		return -EINVAL;

	intel_scu_ipc_lock();

	offset_align = offset & (~0x3);
	len_align = (len + (offset - offset_align) + 3) & (~0x3);

	if (len != len_align) {
		buf = kzalloc(len_align, GFP_KERNEL);
		if (!buf) {
			pr_err("Alloc memory failed\n");
			ret = -ENOMEM;
			goto fail;
		}
		ret = read_mip(buf, len_align, offset_align, 0);
		if (ret)
			goto fail;
		memcpy(buf + offset - offset_align, data, len);
	} else {
		buf = data;
	}

	dptr = offset_align;
	sptr = len_align / 4;
	cmd = IPC_CMD_UMIP_WR << 12 | IPCMSG_MIP_ACCESS;

	memcpy(intel_mip_base, buf, len_align);

	do {
		ret = intel_scu_ipc_raw_cmd(cmd, 0, NULL, 0, NULL, 0,
				dptr, sptr);
		if (ret == -EIO)
			msleep(20);
	} while (ret == -EIO);

fail:
	if (buf && len_align != len)
		kfree(buf);

	intel_scu_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_write_umip);


#define MAX_DATA_NR 8
#define MIP_CMD_LEN 11

enum {
	MIP_DBG_DATA,
	MIP_DBG_LEN,
	MIP_DBG_OFFSET,
	MIP_DBG_ISSIGNED,
	MIP_DBG_ERROR,
};

static u8 mip_data[MAX_DATA_NR];
static int valid_data_nr;
static int mip_len;
static int mip_offset;
static int mip_issigned;
static int mip_dbg_error;
static char mip_cmd[MIP_CMD_LEN];

static ssize_t mip_generic_show(char *buf, int type, int *data)
{
	int i;
	ssize_t ret = 0;

	switch (type) {
	case MIP_DBG_DATA:
		for (i = 0; i < valid_data_nr; i++) {
			ret += snprintf(buf + ret, PAGE_SIZE - ret,
					"data[%d]: %#x\n",
					i, mip_data[i]);
		}
		break;
	case MIP_DBG_LEN:
		ret = snprintf(buf, PAGE_SIZE, "len: %d\n", *data);
		break;
	case MIP_DBG_OFFSET:
		ret = snprintf(buf, PAGE_SIZE, "offset: %#x\n", *data);
		break;
	case MIP_DBG_ISSIGNED:
		ret = snprintf(buf, PAGE_SIZE, "issigned: %d\n", *data);
		break;
	case MIP_DBG_ERROR:
		ret = snprintf(buf, PAGE_SIZE, "error: %d\n", *data);
		break;
	default:
		break;
	}

	return ret;
}

static void mip_generic_store(const char *buf, int type, int *data)
{
	int i, ret;

	if (type == MIP_DBG_DATA) {
		u32 t[MAX_DATA_NR];

		valid_data_nr = 0;
		memset(mip_data, 0, sizeof(mip_data));

		ret = sscanf(buf, "%x %x %x %x %x %x %x %x", &t[0], &t[1],
				&t[2], &t[3], &t[4], &t[5], &t[6], &t[7]);
		if (ret == 0 || ret > MAX_DATA_NR) {
			mip_dbg_error = -EINVAL;
			return;
		} else {
			for (i = 0; i < ret; i++)
				mip_data[i] = (u8)t[i];
			valid_data_nr = ret;
		}
	} else {
		*data = 0;
		switch (type) {
		case MIP_DBG_OFFSET:
			ret = sscanf(buf, "%x", data);
			break;
		case MIP_DBG_LEN:
		case MIP_DBG_ISSIGNED:
			ret = sscanf(buf, "%d", data);
			break;
		default:
			ret = -1;
			break;
		}
	}

	if (ret)
		mip_dbg_error = 0;
	else
		mip_dbg_error = -EINVAL;

	return;
}

static ssize_t mip_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return mip_generic_show(buf, MIP_DBG_DATA, NULL);
}

static ssize_t mip_data_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	mip_generic_store(buf, MIP_DBG_DATA, NULL);
	return size;
}

static ssize_t mip_len_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return mip_generic_show(buf, MIP_DBG_LEN, &mip_len);
}

static ssize_t mip_len_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	mip_generic_store(buf, MIP_DBG_LEN, &mip_len);
	return size;
}

static ssize_t mip_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return mip_generic_show(buf, MIP_DBG_OFFSET, &mip_offset);
}

static ssize_t mip_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	mip_generic_store(buf, MIP_DBG_OFFSET, &mip_offset);
	return size;
}

static ssize_t mip_issigned_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return mip_generic_show(buf, MIP_DBG_ISSIGNED, &mip_issigned);
}

static ssize_t mip_issigned_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	mip_generic_store(buf, MIP_DBG_ISSIGNED, &mip_issigned);
	return size;
}

static ssize_t mip_error_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return mip_generic_show(buf, MIP_DBG_ERROR, &mip_dbg_error);
}

static ssize_t mip_cmd_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	int ret;

	memset(mip_cmd, 0, sizeof(mip_cmd));

	ret = sscanf(buf, "%10s", mip_cmd);
	if (ret == 0) {
		mip_dbg_error = -EINVAL;
		goto end;
	}

	if (!strncmp("read_mip", mip_cmd, MIP_CMD_LEN)) {
		memset(mip_data, 0, sizeof(mip_data));
		ret = intel_scu_ipc_read_mip(mip_data, mip_len, mip_offset,
				mip_issigned);
		if (!ret)
			valid_data_nr = mip_len;

	} else if (!strncmp("write_umip", mip_cmd, MIP_CMD_LEN)) {
		if (mip_len == valid_data_nr) {
			ret = intel_scu_ipc_write_umip(mip_data, mip_len,
					mip_offset);
		} else
			goto error;
	} else
		goto error;

	if (ret)
		goto error;
	else
		goto end;

error:
	mip_dbg_error = -EINVAL;

end:
	return size;
}

static DEVICE_ATTR(data, S_IRUGO|S_IWUSR, mip_data_show, mip_data_store);
static DEVICE_ATTR(len, S_IRUGO|S_IWUSR, mip_len_show, mip_len_store);
static DEVICE_ATTR(offset, S_IRUGO|S_IWUSR, mip_offset_show, mip_offset_store);
static DEVICE_ATTR(issigned, S_IRUGO|S_IWUSR, mip_issigned_show,
		mip_issigned_store);
static DEVICE_ATTR(cmd, S_IWUSR, NULL, mip_cmd_store);
static DEVICE_ATTR(error, S_IRUGO, mip_error_show, NULL);

static struct attribute *mip_attrs[] = {
	&dev_attr_data.attr,
	&dev_attr_len.attr,
	&dev_attr_offset.attr,
	&dev_attr_issigned.attr,
	&dev_attr_cmd.attr,
	&dev_attr_error.attr,
	NULL,
};

static struct attribute_group mip_attr_group = {
	.name = "mip_debug",
	.attrs = mip_attrs,
};

static int __devinit intel_mip_probe(struct ipc_device *ipcdev)
{
	int ret;

	intel_mip_base = ioremap_nocache(IPC_MIP_BASE, IPC_MIP_MAX_ADDR);
	if (!intel_mip_base)
		return -ENOMEM;

	ret = sysfs_create_group(&ipcdev->dev.kobj, &mip_attr_group);
	if (ret) {
		dev_err(&ipcdev->dev, "Failed to create mip sysfs interface\n");
		iounmap(intel_mip_base);
	}

	return ret;
}

static int __devexit intel_mip_remove(struct ipc_device *ipcdev)
{
	iounmap(intel_mip_base);
	sysfs_remove_group(&ipcdev->dev.kobj, &mip_attr_group);
	return 0;
}

static struct ipc_driver mip_driver = {
	.driver = {
		.name = "intel_scu_mip",
		.owner = THIS_MODULE,
	},
	.probe = intel_mip_probe,
	.remove = __devexit_p(intel_mip_remove),
};

static int __init mip_module_init(void)
{
	if ((intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_PENWELL)
		&& (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_CLOVERVIEW))
		return -EINVAL;

	return ipc_driver_register(&mip_driver);
}

static void __exit mip_module_exit(void)
{
	ipc_driver_unregister(&mip_driver);
}

fs_initcall(mip_module_init);
module_exit(mip_module_exit);

MODULE_AUTHOR("Shijie Zhang <shijie.zhang@intel.com>");
MODULE_DESCRIPTION("Intel SCU MIP driver");
MODULE_LICENSE("GPL v2");
