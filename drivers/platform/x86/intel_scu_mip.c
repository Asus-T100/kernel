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
#include <asm/intel_scu_ipc.h>

#define IPC_MIP_BASE     0xFFFD8000	/* sram base address for mip accessing*/
#define IPC_MIP_MAX_ADDR 0x1000

static void __iomem *intel_mip_base;

static int read_mip(u8 *data, int len, int offset, int issigned)
{
	int ret;
	u32 sptr, dptr, cmd, cmdid, data_off;

	if (!intel_mip_base)
		return -ENODEV;

	if (offset + len > IPC_MIP_MAX_ADDR)
		return -EINVAL;

	dptr = offset;
	sptr = (len + 3) / 4;

	cmdid = issigned ? IPC_CMD_SMIP_RD : IPC_CMD_UMIP_RD;
	cmd = 4 << 16 | cmdid << 12 | IPCMSG_MIP_ACCESS;

	do {
		ret = intel_scu_ipc_raw_cmd(cmd, 0, NULL, 0, &data_off, 1,
				dptr, sptr);
	} while (ret == -EIO);

	if (!ret)
		memcpy(data, intel_mip_base + data_off, len);

	return ret;
}

int intel_scu_ipc_read_mip(u8 *data, int len, int offset, int issigned)
{
	int ret;

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

	do {
		memcpy(intel_mip_base, buf, len_align);
		ret = intel_scu_ipc_raw_cmd(cmd, 0, NULL, 0, NULL, 0,
				dptr, sptr);
	} while (ret == -EIO);

fail:
	if (buf && len_align != len)
		kfree(buf);

	intel_scu_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_write_umip);

static int __devinit intel_mip_probe(struct ipc_device *ipcdev)
{
	intel_mip_base = ioremap_nocache(IPC_MIP_BASE, IPC_MIP_MAX_ADDR);
	if (!intel_mip_base)
		return -ENOMEM;

	return 0;
}

static int __devexit intel_mip_remove(struct ipc_device *ipcdev)
{
	iounmap(intel_mip_base);
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
	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_PENWELL)
		return -EINVAL;

	return ipc_driver_register(&mip_driver);
}

static void __exit mip_module_exit(void)
{
	ipc_driver_unregister(&mip_driver);
}

module_init(mip_module_init);
module_exit(mip_module_exit);

MODULE_AUTHOR("Shijie Zhang <shijie.zhang@intel.com>");
MODULE_AUTHOR("Sreedhara DS <sreedhara.ds@intel.com>");
MODULE_DESCRIPTION("Intel SCU MIP driver");
MODULE_LICENSE("GPL v2");
