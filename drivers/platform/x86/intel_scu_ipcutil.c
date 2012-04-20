/*
 * intel_scu_ipc.c: Driver for the Intel SCU IPC mechanism
 *
 * (C) Copyright 2008-2010 Intel Corporation
 * Author: Sreedhara DS (sreedhara.ds@intel.com)
 * (C) Copyright 2010 Intel Corporation
 * Author: Sudha Krishnakumar (sudha.krishnakumar@intel.com)
 * (C) Copyright 2012 Intel Corporation
 * Author: Shijie Zhang (shijie.zhang@intel.com)
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
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <linux/pm_runtime.h>

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
#define INTEL_SCU_IPC_WRITE_ALARM_FLAG_TO_OSNIB 0xC5
#define INTEL_SCU_IPC_OSC_CLK_CNTL 0xC6

#define OSHOB_PMIT_OFFSET		0x0000002c
#define OSNIB_RR_OFFSET			OSNIB_OFFSET
#define OSNIB_WD_OFFSET			(OSNIB_OFFSET + 1)
#define OSNIB_ALARM_OFFSET		(OSNIB_OFFSET + 2)
#define OSNIB_WAKESRC_OFFSET		(OSNIB_OFFSET + 3)
#define OSNIB_RESETIRQ1_OFFSET		(OSNIB_OFFSET + 4)
#define OSNIB_RESETIRQ2_OFFSET		(OSNIB_OFFSET + 5)
#define OSNIB_RR_MASK			0x00000001
#define OSNIB_WD_MASK			0x00000002
#define OSNIB_ALARM_MASK		0x00000004
#define OSNIB_WAKESRC_MASK		0x00000008
#define OSNIB_RESETIRQ1_MASK		0x00000010
#define OSNIB_RESETIRQ2_MASK		0x00000020
#define PMIT_RESETIRQ1_OFFSET		14
#define PMIT_RESETIRQ2_OFFSET		15

#define DUMP_OSNIB

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

struct osc_clk_t {
	unsigned int id; /* clock id */
	unsigned int khz; /* clock frequency */
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
	int ret;

	if (data->count == 0 || data->count > 5)
		return -EINVAL;

	switch (cmd) {
	case INTEL_SCU_IPC_REGISTER_READ:
		ret = intel_scu_ipc_readv(data->addr, data->data, data->count);
		break;
	case INTEL_SCU_IPC_REGISTER_WRITE:
		ret = intel_scu_ipc_writev(data->addr, data->data, data->count);
		break;
	case INTEL_SCU_IPC_REGISTER_UPDATE:
		ret = intel_scu_ipc_update_register(data->addr[0],
							data->data[0],
							data->mask);
		break;
	default:
		return -ENOTTY;
	}
	return ret;
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
	int ret = -EINVAL;
	struct scu_ipc_data  data;
	void __user *argp = (void __user *)arg;
	int platform;

	/* Only IOCTL cmd allowed to pass through without capability check */
	/* is getting fw version info, all others need to check to prevent */
	/* arbitrary access to all sort of bit of the hardware exposed here*/

	if (cmd != INTEL_SCU_IPC_FW_REVISION_GET && !capable(CAP_SYS_RAWIO))
		return -EPERM;

	platform = intel_mid_identify_cpu();
	switch (cmd) {
	case INTEL_SCU_IPC_FW_UPDATE:
	{
		if (platform == INTEL_MID_CPU_CHIP_LINCROFT) {
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
		ret = intel_scu_ipc_read_osnib_rr(&reboot_reason);
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
		ret = intel_scu_ipc_write_osnib_rr(data);
		break;
	}
	case INTEL_SCU_IPC_WRITE_ALARM_FLAG_TO_OSNIB:
	{
		u8 flag, data;
		ret = copy_from_user(&flag, (u8 *)arg, 1);
		if (ret < 0) {
			pr_err("copy from user failed!!\n");
			return ret;
		}
		ret = intel_scu_ipc_read_oshob(&data, 1, OSNIB_ALARM_OFFSET);
		if (ret < 0)
			return ret;
		if (flag)
			data = data | 0x1; /* set alarm flag */
		else
			data = data & 0xFE; /* clear alarm flag */
		ret = intel_scu_ipc_write_osnib(&data, 1, 2, OSNIB_ALARM_MASK);
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
		if (platform == INTEL_MID_CPU_CHIP_PENWELL) {
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
	case INTEL_SCU_IPC_OSC_CLK_CNTL:
	{
		struct osc_clk_t osc_clk;

		if (copy_from_user(&osc_clk, argp, sizeof(struct osc_clk_t)))
			return -EFAULT;

		ret = intel_scu_ipc_osc_clk(osc_clk.id, osc_clk.khz);
		if (ret)
			pr_err("%s: failed to set osc clk\n", __func__);

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

#define OSHOB_SIZE              60
#define OSNIB_SIZE              32
#define IPCMSG_GET_HOBADDR      0xE5

int intel_scu_ipc_read_oshob(u8 *data, int len, int offset)
{
	int ret, i;
	u32 oshob_base;
	void __iomem *oshob_addr;

	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0, NULL, 0,
			&oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_read_oshob failed!!\n");
		goto exit;
	}

	pr_info("OSHOB addr values is %x\n", oshob_base);

	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	} else {
		u8 *ptr = data;
		for (i = 0; i < len; i = i+1) {
			*ptr = readb(oshob_addr + offset + i);
			pr_info("addr=%8x, offset=%2x, value=%2x\n",
					(u32)(oshob_addr + i),
					offset + i, *ptr);
			ptr++;
		}
	}

	iounmap(oshob_addr);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_oshob);

#define IPCMSG_WRITE_OSNIB      0xE4
#define POSNIBW_OFFSET          0x34

int intel_scu_ipc_write_osnib(u8 *data, int len, int offset, u32 mask)
{
	int i;
	int ret = 0;
	u32 posnibw, oshob_base;
	void __iomem *oshob_addr, *osnibw_addr;

	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0, NULL, 0,
			&oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_get_hobaddr failed!!\n");
		goto exit;
	}

	pr_info("OSHOB addr values is %x\n", oshob_base);

	intel_scu_ipc_lock();

	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	posnibw = readl(oshob_addr + POSNIBW_OFFSET);
	if (posnibw == 0) { /* workaround here for BZ 2914 */
		posnibw = 0xFFFF3400;
		pr_err("ERR: posnibw from oshob is 0, manually set it here\n");
	}

	pr_info("POSNIB: %x\n", posnibw);

	osnibw_addr = ioremap_nocache(posnibw, OSNIB_SIZE);
	if (!osnibw_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	for (i = 0; i < len; i++)
		writeb(*(data + i), (osnibw_addr + offset + i));

	ret = intel_scu_ipc_raw_cmd(IPCMSG_WRITE_OSNIB, 0, NULL, 0, NULL,
			0, 0, mask);
	if (ret < 0)
		pr_err("ipc_write_osnib failed!!\n");

	iounmap(osnibw_addr);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	intel_scu_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib);

/*
 * This writes the reboot reason in the OSNIB (factor and avoid any overlap)
 */
int intel_scu_ipc_write_osnib_rr(u8 rr)
{
	return intel_scu_ipc_write_osnib(&rr, 1, OSNIB_RR_OFFSET-OSNIB_OFFSET,
						OSNIB_RR_MASK);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib_rr);

/*
 * This reads the reboot reason from the OSNIB (factor)
 */
int intel_scu_ipc_read_osnib_rr(u8 *rr)
{
	return intel_scu_ipc_read_oshob(rr, 1, OSNIB_RR_OFFSET);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_osnib_rr);

/*
 * This reads the PMIT from the OSHOB (pointer to interrupt tree)
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_oshob_it_tree(u32 *ptr)
{
	return intel_scu_ipc_read_oshob((u8 *) ptr, 4, OSHOB_PMIT_OFFSET);
}
#endif

/*
 * This reads the RESETIRQ1 from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_resetirq1(u8 *rirq1)
{
	return intel_scu_ipc_read_oshob(rirq1, 1, OSNIB_RESETIRQ1_OFFSET);
}
#endif

/*
 * This reads the RESETIRQ2 from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_resetirq2(u8 *rirq2)
{
	return intel_scu_ipc_read_oshob(rirq2, 1, OSNIB_RESETIRQ2_OFFSET);
}
#endif

static const struct file_operations scu_ipc_fops = {
	.unlocked_ioctl = scu_ipc_ioctl,
};

static struct miscdevice scu_ipcutil = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mid_ipc",
	.fops = &scu_ipc_fops,
};

static int __init ipc_module_init(void)
{
#ifdef DUMP_OSNIB
	u8 rr, resetirq1, resetirq2, *ptr;
	u32 pmit;
#endif

#ifdef DUMP_OSNIB
	/* Dumping RESETIRQ1 and 2 from the interrupt tree */
	intel_scu_ipc_read_oshob_it_tree(&pmit);
	ptr = ioremap_nocache(pmit + PMIT_RESETIRQ1_OFFSET, 2);
	if (ptr) {
		resetirq1 = readb(ptr);
		resetirq2 = readb(ptr+1);
		pr_warn("[BOOT] RESETIRQ1=0x%02x RESETIRQ2=0x%02x "
			"(interrupt tree)\n",
			resetirq1, resetirq2);
		iounmap(ptr);
	}

	/* Dumping OSNIB content */
	intel_scu_ipc_read_osnib_rr(&rr);
	intel_scu_ipc_read_osnib_resetirq1(&resetirq1);
	intel_scu_ipc_read_osnib_resetirq2(&resetirq2);
	pr_warn("[BOOT] RR=0x%02x (osnib)\n", rr);
	pr_warn("[BOOT] RESETIRQ1=0x%02x RESETIRQ2=0x%02x "
		"(osnib)\n",
		resetirq1, resetirq2);
#endif

	return misc_register(&scu_ipcutil);
}

static void __exit ipc_module_exit(void)
{
	misc_deregister(&scu_ipcutil);
}

module_init(ipc_module_init);
module_exit(ipc_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Utility driver for intel scu ipc");
MODULE_AUTHOR("Sreedhara <sreedhara.ds@intel.com>");
