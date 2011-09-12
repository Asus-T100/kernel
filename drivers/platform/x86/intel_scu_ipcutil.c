/*
 * intel_scu_ipc.c: Driver for the Intel SCU IPC mechanism
 *
 * (C) Copyright 2008-2010 Intel Corporation
 * Author: Sreedhara DS (sreedhara.ds@intel.com)
 * (C) Copyright 2010 Intel Corporation
 * Author: Sudha Krishnakumar (sudha.krishnakumar@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This driver provides ioctl interfaces to call intel scu ipc driver api
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <asm/intel_scu_ipc.h>
#include <asm/mrst.h>
#include <linux/pm_runtime.h>

static u32 major;

#define MAX_FW_SIZE 264192

/* ioctl commnds */
#define INTEL_SCU_IPC_REGISTER_READ	0
#define INTEL_SCU_IPC_REGISTER_WRITE	1
#define INTEL_SCU_IPC_REGISTER_UPDATE	2
#define INTEL_SCU_IPC_FW_UPDATE		0xA2
#define INTEL_SCU_IPC_MEDFIELD_FW_UPDATE	0xA3
#define INTEL_SCU_IPC_FW_REVISION_GET	0xB0
#define INTEL_SCU_IPC_READ_RR_FROM_OSNIB	0xC1
#define INTEL_SCU_IPC_WRITE_RR_TO_OSNIB	0xC2
#define INTEL_SCU_IPC_READ_VBATTCRIT	0xC4

#define OSNIB_OFFSET			0x0C
#define OSNIB_RR_MASK			0xF

struct scu_ipc_data {
	u32     count;  /* No. of registers */
	u16     addr[5]; /* Register addresses */
	u8      data[5]; /* Register data */
	u8      mask; /* Valid for read-modify-write */
};

struct scu_ipc_version {
	u32     count;  /* length of version info */
	u8      data[16]; /* version data */
};

/**
 *	scu_reg_access		-	implement register access ioctls
 *	@cmd: command we are doing (read/write/update)
 *	@data: kernel copy of ioctl data
 *
 *	Allow the user to perform register accesses on the SCU via the
 *	kernel interface
 */

static int scu_reg_access(u32 cmd, struct scu_ipc_data  *data)
{
	int count = data->count;

	if (count == 0 || count == 3 || count > 4)
		return -EINVAL;

	switch (cmd) {
	case INTEL_SCU_IPC_REGISTER_READ:
		return intel_scu_ipc_readv(data->addr, data->data, count);
	case INTEL_SCU_IPC_REGISTER_WRITE:
		return intel_scu_ipc_writev(data->addr, data->data, count);
	case INTEL_SCU_IPC_REGISTER_UPDATE:
		return intel_scu_ipc_update_register(data->addr[0],
						    data->data[0], data->mask);
	default:
		return -ENOTTY;
	}
}

/**
 *	scu_ipc_ioctl		-	control ioctls for the SCU
 *	@fp: file handle of the SCU device
 *	@cmd: ioctl coce
 *	@arg: pointer to user passed structure
 *
 *	Support the I/O and firmware flashing interfaces of the SCU
 */
static long scu_ipc_ioctl(struct file *fp, unsigned int cmd,
							unsigned long arg)
{
	int ret;
	struct scu_ipc_data  data;
	void __user *argp = (void __user *)arg;
	int platform;
	struct pci_dev *pdev;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	platform = mrst_identify_cpu();
	switch (cmd) {
	case INTEL_SCU_IPC_FW_UPDATE:
	{
		if (platform == MRST_CPU_CHIP_LINCROFT) {
			u8 *fwbuf = kmalloc(MAX_FW_SIZE, GFP_KERNEL);
			if (fwbuf == NULL)
				return -ENOMEM;
			if (copy_from_user(fwbuf, (u8 *)arg, MAX_FW_SIZE)) {
				kfree(fwbuf);
				return -EFAULT;
			}
			ret = intel_scu_ipc_mrstfw_update(fwbuf, MAX_FW_SIZE);
			kfree(fwbuf);
		}
		break;
	}
	case INTEL_SCU_IPC_READ_RR_FROM_OSNIB:
	{
		u8 reboot_reason;

		ret = intel_scu_ipc_read_oshob(&reboot_reason, 1, OSNIB_OFFSET);
		if (ret < 0)
			return ret;
		ret = copy_to_user(argp, &reboot_reason, 1);
		break;
	}
	case INTEL_SCU_IPC_WRITE_RR_TO_OSNIB:
	{
		u8 data;

		ret = copy_from_user(&data, (u8 *)arg, 1);
		if (ret < 0) {
			pr_err("copy from user failed!!\n");
			return ret;
		}
		ret = intel_scu_ipc_write_osnib(&data, 1, 0, OSNIB_RR_MASK);
		break;
	}
	case INTEL_SCU_IPC_READ_VBATTCRIT:
	{
		u32 value;

		pr_debug("cmd = INTEL_SCU_IPC_READ_VBATTCRIT");
		ret = intel_scu_ipc_read_mip((u8 *)&value, 4, 0x318, 1);
		if (ret < 0)
			return ret;
		pr_debug("VBATTCRIT VALUE = %x\n", value);
		ret = copy_to_user(argp, &value, 4);
		break;
	}
	case INTEL_SCU_IPC_MEDFIELD_FW_UPDATE:
	{
		if (platform == MRST_CPU_CHIP_PENWELL) {
			pdev = NULL;
			while ((pdev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID,
						pdev)) != NULL) {
				ret = pm_runtime_get_sync(&pdev->dev);
				if (ret < 0) {
					pr_debug("pm_runtime_get_sync failed: %s!!\n",
						pdev->driver->name);
					return ret;
				}
			}

			ret = intel_scu_ipc_medfw_prepare(argp);

			if (ret < 0) {
				pr_err("ipc_device_medfw_upgrade failed!!\n");
				return ret;
			}
		}
		break;
	}
	case INTEL_SCU_IPC_FW_REVISION_GET:
	{
		struct scu_ipc_version version;

		if (copy_from_user(&version, argp, sizeof(u32)))
			return -EFAULT;

		if (version.count > 16)
			return -EINVAL;

		ret = intel_scu_ipc_command(IPCMSG_FW_REVISION, 0,
					NULL, 0, (u32 *)version.data, 4);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp + sizeof(u32),
					version.data, version.count))
			ret = -EFAULT;
		break;
	}
	default:
		if (copy_from_user(&data, argp, sizeof(struct scu_ipc_data)))
			return -EFAULT;
		ret = scu_reg_access(cmd, &data);
		if (ret < 0)
			return ret;
		if (copy_to_user(argp, &data, sizeof(struct scu_ipc_data)))
			return -EFAULT;
		return 0;
	}

	return ret;
}

static const struct file_operations scu_ipc_fops = {
	.unlocked_ioctl = scu_ipc_ioctl,
};

static int __init ipc_module_init(void)
{
	return register_chrdev(0, "intel_mid_scu", &scu_ipc_fops);
}

static void __exit ipc_module_exit(void)
{
	unregister_chrdev(major, "intel_mid_scu");
}

module_init(ipc_module_init);
module_exit(ipc_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Utility driver for intel scu ipc");
MODULE_AUTHOR("Sreedhara <sreedhara.ds@intel.com>");
