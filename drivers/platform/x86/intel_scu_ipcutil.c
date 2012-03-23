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

#define OSHOB_PMIT_OFFSET		0x0000002c
#define OSNIB_RR_OFFSET		0
#define OSNIB_WD_OFFSET		1
#define OSNIB_ALARM_OFFSET		2
#define OSNIB_WAKESRC_OFFSET	3
#define OSNIB_RESETIRQ1_OFFSET	4
#define OSNIB_RESETIRQ2_OFFSET	5
#define PMIT_RESETIRQ1_OFFSET		14
#define PMIT_RESETIRQ2_OFFSET		15

#define OSHOB_SCU_TRACE_OFFSET		0x00
#define OSHOB_IA_TRACE_OFFSET		0x04

#define DUMP_OSNIB

/* Mode for Audio clock */
static DEFINE_MUTEX(osc_clk0_lock);
static unsigned int osc_clk0_mode;

int intel_scu_ipc_osc_clk(u8 clk, unsigned int khz)
{
	/* SCU IPC COMMAND(osc clk on/off) definition:
	 * ipc_wbuf[0] = clock to act on {0, 1, 2, 3}
	 * ipc_wbuf[1] =
	 * bit 0 - 1:on  0:off
	 * bit 1 - if 1, read divider setting from bits 3:2 as follows:
	 * bit [3:2] - 00: clk/1, 01: clk/2, 10: clk/4, 11: reserved
	 */
	unsigned int base_freq;
	unsigned int div;
	u8 ipc_wbuf[16];
	int ipc_ret;

	if (clk > 3)
		return -EINVAL;

	ipc_wbuf[0] = clk;
	ipc_wbuf[1] = 0;
	if (khz) {
#ifdef CONFIG_CTP_CRYSTAL_38M4
		base_freq = 38400;
#else
		base_freq = 19200;
#endif
		div = fls(base_freq / khz) - 1;
		if (div >= 3 || (1 << div) * khz != base_freq)
			return -EINVAL;	/* Allow only exact frequencies */
		ipc_wbuf[1] = 0x03 | (div << 2);
	}

	ipc_ret = intel_scu_ipc_command(IPCMSG_OSC_CLK, 0,
					(u32 *)ipc_wbuf, 2, NULL, 0);
	if (ipc_ret != 0)
		pr_err("%s: failed to set osc clk(%d) output\n", __func__, clk);

	return ipc_ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_osc_clk);

/*
 * OSC_CLK_AUDIO is connected to the MSIC as well as Audience, so it should be
 * turned on if any one of them requests it to be on and it should be turned off
 * only if no one needs it on.
 */
int intel_scu_ipc_set_osc_clk0(unsigned int enable, enum clk0_mode mode)
{
	int ret = 0, clk_enable;
	static const unsigned int clk_khz = 19200;

	pr_debug("set_clk0 request %s for Mode 0x%x\n",
				enable ? "ON" : "OFF", mode);
	mutex_lock(&osc_clk0_lock);
	if (mode == CLK0_QUERY) {
		ret = osc_clk0_mode;
		goto out;
	}
	if (enable) {
		/* if clock is already on, just add new user */
		if (osc_clk0_mode) {
			osc_clk0_mode |= mode;
			goto out;
		}
		osc_clk0_mode |= mode;
		pr_debug("set_clk0: enabling clk, mode 0x%x\n", osc_clk0_mode);
		clk_enable = 1;
	} else {
		osc_clk0_mode &= ~mode;
		pr_debug("set_clk0: disabling clk, mode 0x%x\n", osc_clk0_mode);
		/* others using the clock, cannot turn it of */
		if (osc_clk0_mode)
			goto out;
		clk_enable = 0;
	}
	pr_debug("configuring OSC_CLK_AUDIO now\n");
	ret = intel_scu_ipc_osc_clk(OSC_CLK_AUDIO, clk_enable ? clk_khz : 0);
out:
	mutex_unlock(&osc_clk0_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_set_osc_clk0);

#define MSIC_VPROG1_CTRL        0xD6
#define MSIC_VPROG2_CTRL        0xD7
#define MSIC_VPROG_ON           0xFF
#define MSIC_VPROG_OFF          0

int intel_scu_ipc_msic_vprog1(int on)
{
	return intel_scu_ipc_iowrite8(MSIC_VPROG1_CTRL,
			on ? MSIC_VPROG_ON : MSIC_VPROG_OFF);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_msic_vprog1);

int intel_scu_ipc_msic_vprog2(int on)
{
	return intel_scu_ipc_iowrite8(MSIC_VPROG2_CTRL,
			on ? MSIC_VPROG_ON : MSIC_VPROG_OFF);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_msic_vprog2);

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
		ret = intel_scu_ipc_read_osnib(&data, 1, OSNIB_ALARM_OFFSET);
		if (ret < 0)
			return ret;
		if (flag)
			data = data | 0x1; /* set alarm flag */
		else
			data = data & 0xFE; /* clear alarm flag */
		ret = intel_scu_ipc_write_osnib(&data, 1, OSNIB_ALARM_OFFSET);
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

int intel_scu_ipc_read_osnib(u8 *data, int len, int offset)
{
	int i, ret = 0;
	u32 oshob_base;
	u8 *ptr;
	void __iomem *oshob_addr;
	void __iomem *osnibr_addr;
	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR,
		0, NULL, 0, &oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_read_osnib failed!\n");
		goto exit;
	}
	pr_info("OSHOB addr values is %x\n", oshob_base);
	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}
	osnibr_addr = oshob_addr + OSNIB_OFFSET;

	ptr = data;
	for (i = 0; i < len; i++) {
		*ptr = readb(osnibr_addr + offset + i);
		pr_info("addr=%8x, offset=%2x, value=%2x\n",
			(u32)(osnibr_addr+offset+i), offset+i, *ptr);
		ptr++;
	}

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_osnib);

int intel_scu_ipc_write_osnib(u8 *data, int len, int offset)
{
	int i;
	int ret = 0;
	u32 posnibw, oshob_base;
	u8 osnib_data[OSNIB_SIZE];
	u8 chksum = 0;
	void __iomem *oshob_addr, *osnibw_addr, *osnibr_addr;

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

	/*Dump osnib data for generate chksum */
	osnibr_addr = oshob_addr + OSNIB_OFFSET;
	for (i = 0; i < OSNIB_SIZE; i++)
		osnib_data[i] = readb(osnibr_addr + i);

	memcpy(osnib_data + offset, data, len);

	/* generate chksum */
	for (i = 0; i < OSNIB_SIZE - 1; i++)
		chksum += osnib_data[i];
	osnib_data[OSNIB_SIZE - 1] = ~chksum + 1;

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

	for (i = 0; i < OSNIB_SIZE; i++)
		writeb(*(osnib_data + i), (osnibw_addr + i));

	ret = intel_scu_ipc_raw_cmd(IPCMSG_WRITE_OSNIB, 0, NULL, 0, NULL,
			0, 0, 0xFFFFFFFF);
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
	return intel_scu_ipc_write_osnib(&rr, 1, OSNIB_RR_OFFSET);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib_rr);

/*
 * This reads the reboot reason from the OSNIB (factor)
 */
int intel_scu_ipc_read_osnib_rr(u8 *rr)
{
	return intel_scu_ipc_read_osnib(rr, 1, OSNIB_RR_OFFSET);
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
	return intel_scu_ipc_read_osnib(rirq1, 1, OSNIB_RESETIRQ1_OFFSET);
}
#endif

/*
 * This reads the RESETIRQ2 from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_resetirq2(u8 *rirq2)
{
	return intel_scu_ipc_read_osnib(rirq2, 1, OSNIB_RESETIRQ2_OFFSET);
}
#endif

/*
 * This reads the WD from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_wd(u8 *rirq2)
{
	return intel_scu_ipc_read_oshob(rirq2, 1, OSNIB_WD_OFFSET);
}
#endif

/*
 * This reads the ALARM from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_alarm(u8 *rirq2)
{
	return intel_scu_ipc_read_oshob(rirq2, 1, OSNIB_ALARM_OFFSET);

}
#endif

/*
 * This reads the WAKESRC from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_wakesrc(u8 *rirq2)
{
	return intel_scu_ipc_read_oshob(rirq2, 1, OSNIB_WAKESRC_OFFSET);
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
	u8 rr, resetirq1, resetirq2, wd, alarm, wakesrc, *ptr;
	u32 pmit, scu_trace, ia_trace;
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

	/* Dumping OSHOB content */
	intel_scu_ipc_read_oshob(&scu_trace, 4, OSHOB_SCU_TRACE_OFFSET);
	intel_scu_ipc_read_oshob(&ia_trace, 4, OSHOB_IA_TRACE_OFFSET);
	pr_warn("[BOOT] SCU_TR=0x%08x IA_TR=0x%08x (oshob)\n",
		scu_trace, ia_trace);
	/* Dumping OSNIB content */
	intel_scu_ipc_read_osnib_rr(&rr);
	intel_scu_ipc_read_osnib_resetirq1(&resetirq1);
	intel_scu_ipc_read_osnib_resetirq2(&resetirq2);
	intel_scu_ipc_read_osnib_wd(&wd);
	intel_scu_ipc_read_osnib_alarm(&alarm);
	intel_scu_ipc_read_osnib_wakesrc(&wakesrc);
	pr_warn("[BOOT] RR=0x%02x WD=0x%02x ALARM=0x%02x (osnib)\n",
		rr, wd, alarm);
	pr_warn("[BOOT] WAKESRC=0x%02x RESETIRQ1=0x%02x RESETIRQ2=0x%02x "
		"(osnib)\n",
		wakesrc, resetirq1, resetirq2);
#endif

	return misc_register(&scu_ipcutil);
}

static void __exit ipc_module_exit(void)
{
	misc_deregister(&scu_ipcutil);
}

rootfs_initcall(ipc_module_init);
module_exit(ipc_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Utility driver for intel scu ipc");
MODULE_AUTHOR("Sreedhara <sreedhara.ds@intel.com>");
