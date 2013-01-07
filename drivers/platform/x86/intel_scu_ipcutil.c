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
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define MAX_FW_SIZE 264192

/* OSNIB allocation. */
#define OSNIB_RR_OFFSET		0
#define OSNIB_WD_OFFSET		1
#define OSNIB_ALARM_OFFSET	2
#define OSNIB_WAKESRC_OFFSET	3
#define OSNIB_RESETIRQ1_OFFSET	4
#define OSNIB_RESETIRQ2_OFFSET	5

#define OSNIB_CHECKSUM_OFFSET	31

#define PMIT_RESETIRQ1_OFFSET		14
#define PMIT_RESETIRQ2_OFFSET		15

/* Used for old OSHOB: offsets wehere SCUTxl and IATxl */
/* can be found.                                       */
#define OSHOB_SCU_TRACE_OFFSET		0x00
#define OSHOB_IA_TRACE_OFFSET		0x04

/* This offset corresponds to the old OSNIB organization. */
#ifdef CONFIG_X86_MRFLD
#define OSNIB_OFFSET			0x18
#else
#define OSNIB_OFFSET			0x0C
#endif

/* Points to the parameter indicating which of the old */
/* or new OSHOB will be used.                          */
#define OSHOB_EXTEND_MAGIC_OFFSET	0x00

#define OSHOB_EXTEND_MAJREV_OFFSET	0x04
#define OSHOB_EXTEND_MINREV_OFFSET	0x05

/* Points to the total size (bytes) of the whole new   */
/* OSHOB structure.                                    */
#define OSHOB_EXTEND_SIZE_OFFSET	0x06

/* Used for new OSHOB: offsets wehere SCUTxl and IATxl */
/* can be found.                                       */
#define OSHOB_EXTEND_SCU_TRACE_OFFSET	0x0C
#define OSHOB_EXTEND_IA_TRACE_OFFSET	0x10

/* Used with new OSHOB structure. */
#define POSNIB_R_INTEL_SIZE_OFFSET	0x18

/* OEM area size. Used with new OSHOB structure. */
#define POSNIB_R_OEM_SIZE_OFFSET	0x1A

/* This offset is the place where the pointer to the new OSNIB */
/* for reading can be found.                                   */
#define POSNIB_R_INTEL_POINTER_OFFSET	0x1C

/* This offset is the place where the pointer to the new OSNIB */
/* for writting can be found.                                  */
#define POSNIB_W_INTEL_POINTER_OFFSET	0x20

/* Next 3 defines concern the new OEMNIB information. */
#define POSNIB_R_OEM_POINTER_OFFSET	0x24
#define POSNIB_W_OEM_POINTER_OFFSET	0x28

/* This offset is the place where pointer to PMIT can be found.*/
/* Used for both old and new OSHOB.                            */
#define OSHOB_PMIT_OFFSET		0x2C


#define IPC_RESIDENCY_CMD_ID_START	0
#define IPC_RESIDENCY_CMD_ID_DUMP	2

#define SRAM_ADDR_S0IX_RESIDENCY	0xFFFF71E0
#define ALL_RESIDENCY_DATA_SIZE		12

#define DUMP_OSNIB

#define OSHOB_STRUCT_OLD	0x1 /* Old OSHOB and OSNIB to be used.       */
#define OSHOB_STRUCT_EXTENDED	0x2 /* Extended OSHOB and OSNIB to be used.  */

#define OSHOB_EXTEND_DESC_SIZE	52  /* OSHOB header+osnib+oem info: 52 bytes.*/

#define OSHOB_HEADER_MAGIC_SIZE	4   /* Size (bytes) of magic number in OSHOB */
				    /* header.                               */

#define OSHOB_MAGIC_NUMBER	"$OH$"	/* If found when reading the first   */
					/* 4 bytes of the OSOHB zone, it     */
					/* means that the new extended OSHOB */
					/* is going to be used.              */

/* In the new OSHOB/OSNIB/OEMNIB structure the stored pointers do not have */
/* same byte order. So they must be swapped.                               */
#define SWAP32(x)   (((u32)(x)<<16) | ((((u32)(x)))>>16))


struct scu_ipc_oshob_info {
	__u32	oshob_type;     /* Set from magic number extracted from      */
				/* OSHOB structure. Indicates if old or      */
				/* extended version of OSHOB will be used.   */
	__u32	oshob_base;     /* Base address of OSHOB. Use ioremap to     */
				/* remap for access.                         */
	__u8	oshob_majrev;   /* Major revision number of OSHOB structure. */
	__u8	oshob_minrev;   /* Minor revision number of OSHOB structure. */
	__u16	oshob_size;     /* Total size (bytes) of OSHOB structure.    */
	__u32   scu_trace;      /* SCU trace buffer.                         */
	__u32   ia_trace;       /* IA trace buffer.                          */
	__u16	osnib_size;     /* Total size (bytes) of OSNIB structure.    */
	__u16	oemnib_size;    /* Total size (bytes) of OEMNIB area.        */
	__u32	posnibr;        /* Pointer to Intel read zone.               */
	__u32	posnibw;        /* Pointer to Intel write zone.              */
	__u32	poemnibr;       /* Pointer to OEM read zone.                 */
	__u32	poemnibw;       /* Pointer to OEM write zone.                */
};

/* Structure for OSHOB info */
static struct scu_ipc_oshob_info *poshob_info;


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
	u8 ipc_wbuf[2];
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
					ipc_wbuf, 2, NULL, 0);
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

#ifdef CONFIG_X86_MRFLD
#define MSIC_VPROG1_CTRL	0xAC
#define MSIC_VPROG2_CTRL	0xAD
#else
#define MSIC_VPROG1_CTRL        0xD6
#define MSIC_VPROG2_CTRL        0xD7
#endif

#ifdef CONFIG_X86_MRFLD
#define MSIC_VPROG1_ON	0xC1	/* 2.80V */
#define MSIC_VPROG2_ON	0xC1	/* 2.80V */
#define MSIC_VPROG_OFF	0	/* OFF */
#else
#define MSIC_VPROG2_ON          0x36 /*1.200V and Auto mode*/
#define MSIC_VPROG1_ON          0xF6 /*2.800V and Auto mode*/
#define MSIC_VPROG_OFF          0x24 /*1.200V and OFF*/
#endif

int intel_scu_ipc_msic_vprog1(int on)
{
	return intel_scu_ipc_iowrite8(MSIC_VPROG1_CTRL,
			on ? MSIC_VPROG1_ON : MSIC_VPROG_OFF);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_msic_vprog1);

int intel_scu_ipc_msic_vprog2(int on)
{
	return intel_scu_ipc_iowrite8(MSIC_VPROG2_CTRL,
			on ? MSIC_VPROG2_ON : MSIC_VPROG_OFF);
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

#define check_pmdb_sub_cmd(x)	(x == PMDB_SUB_CMD_R_OTPCTL || \
		x == PMDB_SUB_CMD_R_WMDB || x == PMDB_SUB_CMD_W_WMDB || \
		x == PMDB_SUB_CMD_R_OTPDB || x == PMDB_SUB_CMD_W_OTPDB)
#define pmdb_sub_cmd_is_read(x)	(x == PMDB_SUB_CMD_R_OTPCTL || \
		x == PMDB_SUB_CMD_R_WMDB || x == PMDB_SUB_CMD_R_OTPDB)

static int check_pmdb_buffer(struct scu_ipc_pmdb_buffer *p_buf)
{
	int size;

	switch (p_buf->sub) {
	case PMDB_SUB_CMD_R_WMDB:
	case PMDB_SUB_CMD_W_WMDB:
		size = PMDB_WMDB_SIZE;
		break;
	case PMDB_SUB_CMD_R_OTPDB:
	case PMDB_SUB_CMD_W_OTPDB:
		size = PMDB_OTPDB_SIZE;
		break;
	case PMDB_SUB_CMD_R_OTPCTL:
		size = PMDB_OTPCTL_SIZE;
		break;
	default:
		size = 0;
	}

	return check_pmdb_sub_cmd(p_buf->sub) &&
		(p_buf->count + p_buf->offset <= size);
}

/**
 *	scu_pmdb_access	-	access PMDB data through SCU IPC cmds
 *	@p_buf: PMDB access buffer, it describe the data to write/read.
 *		p_buf->sub - SCU IPC sub cmd of PMDB access,
 *			this sub cmd distinguish different componet
 *			in PMDB which to be accessd. (WMDB, OTPDB, OTPCTL)
 *		p_buf->count - access data's count;
 *		p_buf->offset - access data's offset for each component in PMDB;
 *		p_buf->data - data to write/read.
 *
 *	Write/read data to/from PMDB.
 *
 */
static int scu_pmdb_access(struct scu_ipc_pmdb_buffer *p_buf)
{
	int i, offset, ret = -EINVAL;
	u8 *p_data;

	if (!check_pmdb_buffer(p_buf)) {
		pr_err("Invalid PMDB buffer!\n");
		return -EINVAL;
	}

	/* 1. we use intel_scu_ipc_raw_cmd() IPC cmd interface
	 *    to access PMDB data. Each call of intel_scu_ipc_raw_cmd()
	 *    can only access at most PMDB_ACCESS_SIZE bytes' data.
	 * 2. There are two kinds of pmdb sub commands, read command
	 *    and write command. For read command, we must transport
	 *    in and out buffer to intel_scu_ipc_raw_cmd(), because
	 *    in buffer length is pass as access length which must
	 *    be transported to SCU.
	 */
	p_data = p_buf->data;
	offset = p_buf->offset;
	for (i = 0; i < p_buf->count/PMDB_ACCESS_SIZE; i++) {
		if (pmdb_sub_cmd_is_read(p_buf->sub))
			ret = intel_scu_ipc_raw_cmd(IPCMSG_PMDB_CMD, p_buf->sub,
					p_data, PMDB_ACCESS_SIZE,
					(u32 *)p_data, PMDB_ACCESS_SIZE / 4,
					offset, 0);
		else
			ret = intel_scu_ipc_raw_cmd(IPCMSG_PMDB_CMD, p_buf->sub,
					p_data, PMDB_ACCESS_SIZE,
					NULL, 0, offset, 0);
		if (ret < 0) {
			pr_err("intel_scu_ipc_raw_cmd failed!\n");
			return ret;
		}
		offset += PMDB_ACCESS_SIZE;
		p_data += PMDB_ACCESS_SIZE;
	}
	if (p_buf->count % PMDB_ACCESS_SIZE > 0) {
		if (pmdb_sub_cmd_is_read(p_buf->sub))
			ret = intel_scu_ipc_raw_cmd(IPCMSG_PMDB_CMD, p_buf->sub,
					p_data,
					p_buf->count % PMDB_ACCESS_SIZE,
					(u32 *)p_data,
					(p_buf->count % PMDB_ACCESS_SIZE) / 4,
					offset, 0);
		else
			ret = intel_scu_ipc_raw_cmd(IPCMSG_PMDB_CMD, p_buf->sub,
					p_data,
					p_buf->count % PMDB_ACCESS_SIZE,
					NULL, 0, offset, 0);
		if (ret < 0) {
			pr_err("intel_scu_ipc_raw_cmd failed!\n");
			return ret;
		}
	}

	return 0;
}

static int do_pmdb_user_buf_access(void __user *argp)
{
	int ret;
	struct scu_ipc_pmdb_buffer *p_buf;

	p_buf = kzalloc(sizeof(struct scu_ipc_pmdb_buffer), GFP_KERNEL);
	if (p_buf == NULL) {
		pr_err("failed to allocate memory for pmdb buffer!\n");
		return -ENOMEM;
	}

	ret = copy_from_user(p_buf, argp, sizeof(struct scu_ipc_pmdb_buffer));
	if (ret < 0) {
		pr_err("copy from user failed!!\n");
		goto err;
	}

	ret = scu_pmdb_access(p_buf);
	if (ret < 0) {
		pr_err("scu_pmdb_access error!\n");
		goto err;
	}

	if (pmdb_sub_cmd_is_read(p_buf->sub)) {
		ret = copy_to_user(argp + 3 * sizeof(u32),
					p_buf->data, p_buf->count);
		if (ret < 0)
			pr_err("copy to user failed!!\n");
	}

err:
	kfree(p_buf);
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

	if ((cmd != INTEL_SCU_IPC_FW_REVISION_GET ||
		cmd != INTEL_SCU_IPC_S0IX_RESIDENCY) &&
		!capable(CAP_SYS_RAWIO))
		return -EPERM;

	platform = intel_mid_identify_cpu();
	switch (cmd) {
	case INTEL_SCU_IPC_S0IX_RESIDENCY:
	{
		void __iomem *s0ix_residencies_addr;
		u8 dump_results[ALL_RESIDENCY_DATA_SIZE] = {0};
		u32 cmd_id;

		if (copy_from_user(&cmd_id, argp, sizeof(u32))) {
			pr_err("copy from user failed!!\n");
			return -EFAULT;
		}

		/* Check get residency counter valid cmd range */

		if (cmd_id > IPC_RESIDENCY_CMD_ID_DUMP) {
			pr_err("invalid si0x residency sub-cmd id!\n");
			return -EINVAL;
		}

		ret = intel_scu_ipc_simple_command(IPCMSG_S0IX_COUNTER, cmd_id);

		if (ret < 0) {
			pr_err("ipc_get_s0ix_counter failed!\n");
			return ret;
		}

		if (cmd_id == IPC_RESIDENCY_CMD_ID_DUMP) {
			s0ix_residencies_addr = ioremap_nocache(
				SRAM_ADDR_S0IX_RESIDENCY,
				ALL_RESIDENCY_DATA_SIZE);

			if (!s0ix_residencies_addr) {
				pr_err("ioremap SRAM address failed!!\n");
				return -EFAULT;
			}

			memcpy(&dump_results[0], s0ix_residencies_addr,
				ALL_RESIDENCY_DATA_SIZE);

			iounmap(s0ix_residencies_addr);
			ret = copy_to_user(argp, &dump_results[0],
					ALL_RESIDENCY_DATA_SIZE);
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

		if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
			ret = intel_scu_ipc_read_osnib(&data,
							1,
							OSNIB_ALARM_OFFSET);
		else
			ret = intel_scu_ipc_read_osnib_extend(&data,
							1,
							OSNIB_ALARM_OFFSET);
		if (ret < 0)
			return ret;
		if (flag) {
			data = data | 0x1; /* set alarm flag */
			pr_info("scu_ipc_ioctl: set alarm flag\n");
		} else {
			data = data & 0xFE; /* clear alarm flag */
			pr_info("scu_ipc_ioctl: clear alarm flag\n");
		}

		if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
			ret = intel_scu_ipc_write_osnib(&data,
							1,
							OSNIB_ALARM_OFFSET);
		else
			ret = intel_scu_ipc_write_osnib_extend(&data,
							1,
							OSNIB_ALARM_OFFSET);
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
	case INTEL_SCU_IPC_PMDB_ACCESS:
	{
		ret = do_pmdb_user_buf_access(argp);

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

/* Size (bytes) of the old OSHOB structure. Includes the old OSNIB size.   */
/* Note: size of new OSHOB structure is at offset OSHOB_EXTEND_SIZE_OFFSET */
/*       in the new OSHOB.                                                 */
#define OSHOB_SIZE              60
#define OSNIB_SIZE              32	/* Size (bytes) of the old OSNIB.  */

#define IPCMSG_GET_HOBADDR      0xE5

int intel_scu_ipc_read_oshob_info(void)
{
	int i, ret = 0;
	u32 oshob_base;
	void __iomem *oshob_addr;
	unsigned char oshob_magic[4];

	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0, NULL, 0,
			&oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_read_oshob cmd failed!!\n");
		goto exit;
	}

	/* At this stage, we still don't know which OSHOB type (old or new) */
	/* an be used, and the size of resource to be remapped depends on   */
	/* the type of OSHOB structure to be used.                          */
	/* So just remap the minimum size to get the needed bytes of the    */
	/* OSHOB zone.                                                      */
	oshob_addr = ioremap_nocache(oshob_base, OSHOB_EXTEND_DESC_SIZE);

	if (!oshob_addr) {
		pr_err("oshob addr ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	pr_info("OSHOB addr 0x%8x remapped to addr 0x%8x\n",
		oshob_base, (u32)oshob_addr);

	poshob_info->oshob_base = oshob_base;

	/* Extract magic number that will help identifying the good OSHOB  */
	/* that is going to be used.                                       */
	for (i = 0; i < OSHOB_HEADER_MAGIC_SIZE; i = i+1)
		oshob_magic[i] = readb(oshob_addr + i);

	if (strncmp(oshob_magic,
		    OSHOB_MAGIC_NUMBER,
		    OSHOB_HEADER_MAGIC_SIZE) == 0) {
		poshob_info->oshob_type = OSHOB_STRUCT_EXTENDED;
		poshob_info->oshob_size = readw(oshob_addr +
						OSHOB_EXTEND_SIZE_OFFSET);

		/* Get version. */
		poshob_info->oshob_majrev = readb(oshob_addr +
					    OSHOB_EXTEND_MAJREV_OFFSET);
		poshob_info->oshob_minrev = readb(oshob_addr +
					    OSHOB_EXTEND_MINREV_OFFSET);

		/* Get defined OSNIB space size. */
		poshob_info->osnib_size = readw(oshob_addr +
					    POSNIB_R_INTEL_SIZE_OFFSET);

		if (poshob_info->osnib_size == 0) {
			pr_err("ipc_read_oshob: OSNIB size is null!\n");
			ret = -EFAULT;
			goto unmap_oshob;
		}

		/* Get defined OEM space size. */
		poshob_info->oemnib_size = readw(oshob_addr +
						POSNIB_R_OEM_SIZE_OFFSET);

		if (poshob_info->oemnib_size == 0) {
			pr_err("ipc_read_oshob: OEMNIB size is null!\n");
			ret = -EFAULT;
			goto unmap_oshob;
		}

		/* Set SCU and IA trace buffers */
		poshob_info->scu_trace = readl(oshob_addr +
						OSHOB_EXTEND_SCU_TRACE_OFFSET);
		poshob_info->ia_trace = readl(oshob_addr +
						OSHOB_EXTEND_IA_TRACE_OFFSET);
		/* Set pointers */
		poshob_info->posnibr = readl(oshob_addr +
						POSNIB_R_INTEL_POINTER_OFFSET);

		if (!poshob_info->posnibr) {
			pr_err("oshob R_INTEL_POINTER is NULL!\n");
			ret = -ENOMEM;
			goto unmap_oshob;
		}

		poshob_info->posnibw = readl(oshob_addr +
						POSNIB_W_INTEL_POINTER_OFFSET);

		if (poshob_info->posnibw == 0) {
			/* workaround here for BZ 2914 */
			poshob_info->posnibw = 0xFFFF3400;
			pr_err(
			"ipc_write_osnib_extend: ERR: posnibw from oshob is 0, manually set it here\n");
		}

		poshob_info->poemnibr = readl(oshob_addr +
						POSNIB_R_OEM_POINTER_OFFSET);

		if (!poshob_info->poemnibr) {
			pr_err("oshob R_OEM_POINTER is NULL!\n");
			ret = -ENOMEM;
			goto unmap_oshob;
		}

		poshob_info->poemnibw = readl(oshob_addr +
						POSNIB_W_OEM_POINTER_OFFSET);

		if (!poshob_info->poemnibw) {
			pr_err("oshob W_OEM_POINTER is NULL!\n");
			ret = -ENOMEM;
			goto unmap_oshob;
		}

		pr_info(
			"Using NEW OSHOB structure size = %d bytes\n",
			poshob_info->oshob_size);
		pr_info(
			"OSNIB size = %d bytes OEMNIB size = %d bytes\n",
			poshob_info->osnib_size, poshob_info->oemnib_size);
	} else {
		poshob_info->oshob_type = OSHOB_STRUCT_OLD;
		poshob_info->oshob_size = OSHOB_SIZE;
		poshob_info->osnib_size = OSNIB_SIZE;
		poshob_info->oemnib_size = 0;

		/* Set SCU and IA trace buffers */
		poshob_info->scu_trace = readl(oshob_addr +
						OSHOB_SCU_TRACE_OFFSET);
		poshob_info->ia_trace = readl(oshob_addr +
						OSHOB_IA_TRACE_OFFSET);

		pr_info(
			"Using OLD OSHOB structure size = %d bytes\n",
			poshob_info->oshob_size);
	}

unmap_oshob:
	iounmap(oshob_addr);

exit:
	return ret;
}

int intel_scu_ipc_get_oshob_size(void)
{
	return poshob_info->oshob_size;
}

int intel_scu_ipc_read_oshob(u8 *data, int len, int offset)
{
	int ret = 0, i;
	u32 oshob_base;
	void __iomem *oshob_addr;
	u8 *ptr = data;

	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0, NULL, 0,
			&oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_read_oshob: cmd failed!!\n");
		goto exit;
	}

	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD) {
		oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	} else {
		oshob_addr = ioremap_nocache(oshob_base,
					     poshob_info->oshob_size);
	}

	if (!oshob_addr) {
		pr_err("ipc_read_oshob: addr ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	for (i = 0; i < len; i = i+1) {
		*ptr = readb(oshob_addr + offset + i);
		pr_info("addr=%8x, offset=%2x, value=%2x\n",
			(u32)(oshob_addr + i),
			offset + i, *ptr);
		ptr++;
	}

	iounmap(oshob_addr);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_oshob);

/* This option is used to write to the old OSNIB. */
#define IPCMSG_WRITE_OSNIB		0xE4

/* This option is used to write to the extended OSNIB. */
#define IPCMSG_WRITE_OSNIB_EXTEND	0xE4

/* This command is used to write the OEMNIB data.   */
/* Used with the new extended OSHOB OSNIB only.     */
#define IPCMSG_WRITE_OEMNIB		0xDF

/* Offset of pointer to OSNIB. Used with old OSHOB. */
#ifdef CONFIG_X86_MRFLD
#define POSNIBW_OFFSET			0x40
#else
#define POSNIBW_OFFSET			0x34
#endif

/* This function is used for the old OSNIB. */
int intel_scu_ipc_read_osnib(u8 *data, int len, int offset)
{
	int i, ret = 0;
	u32 oshob_base, posnibw;
	u8 *ptr, check = 0;
	void __iomem *oshob_addr, *osnibr_addr, *osnibw_addr;

	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR,
		0, NULL, 0, &oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_read_osnib failed!\n");
		goto exit;
	}
	pr_info("OSHOB base addr value is %x\n", oshob_base);
	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}
	osnibr_addr = oshob_addr + OSNIB_OFFSET;

	if (!osnibr_addr) {
		pr_err("Bad osnib address!\n");
		ret = -EFAULT;
		iounmap(oshob_addr);
		goto exit;
	}

	/* Make a chksum verification for osnib */
	for (i = 0; i < OSNIB_SIZE; i++)
		check += readb(osnibr_addr + i);
	if (check) {
		pr_err("WARNING!!! osnib chksum verification faild, reset all osnib data!\n");
		posnibw = readl(oshob_addr + POSNIBW_OFFSET);
		osnibw_addr = ioremap_nocache(posnibw, OSNIB_SIZE);
		if (osnibw_addr) {
			for (i = 0; i < OSNIB_SIZE; i++)
				writeb(0, osnibw_addr + i);
			intel_scu_ipc_raw_cmd(IPCMSG_WRITE_OSNIB,
				0, NULL, 0, NULL, 0, 0, 0xFFFFFFFF);
			iounmap(osnibw_addr);
		}
	}

	ptr = data;
	for (i = 0; i < len; i++) {
		*ptr = readb(osnibr_addr + offset + i);
		pr_info("addr=%8x, offset=%2x, value=%2x\n",
			(u32)(osnibr_addr+offset+i), offset+i, *ptr);
		ptr++;
	}

	iounmap(oshob_addr);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_osnib);

/* This function is used for the old OSNIB. */
int intel_scu_ipc_write_osnib(u8 *data, int len, int offset)
{
	int i;
	int ret = 0;
	u32 posnibw, oshob_base;
	u8 osnib_data[OSNIB_SIZE];
	u8 check = 0, chksum = 0;
	void __iomem *oshob_addr, *osnibw_addr, *osnibr_addr;

	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0, NULL, 0,
			&oshob_base, 1);
	if (ret < 0) {
		pr_err("ipc_get_hobaddr failed!!\n");
		goto exit;
	}

	pr_info("OSHOB base addr value is 0x%8x\n", oshob_base);

	intel_scu_ipc_lock();

	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	/*Dump osnib data for generate chksum */
	osnibr_addr = oshob_addr + OSNIB_OFFSET;

	if (!osnibr_addr) {
		pr_err("Bad osnib read address!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	for (i = 0; i < OSNIB_SIZE; i++) {
		osnib_data[i] = readb(osnibr_addr + i);
		check += osnib_data[i];
	}
	memcpy(osnib_data + offset, data, len);

	if (check) {
		pr_err("WARNING!!! OSNIB data chksum verification FAILED!\n");
	} else {
		/* generate chksum */
		for (i = 0; i < OSNIB_SIZE - 1; i++)
			chksum += osnib_data[i];
		osnib_data[OSNIB_SIZE - 1] = ~chksum + 1;
	}

	posnibw = readl(oshob_addr + POSNIBW_OFFSET);
	if (posnibw == 0) { /* workaround here for BZ 2914 */
		posnibw = 0xFFFF3400;
		pr_err("ERR: posnibw from oshob is 0, manually set it here\n");
	}

	pr_info("POSNIB address: %x\n", posnibw);

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


/* This function is used for the extended OSHOB/OSNIB. */
int intel_scu_ipc_read_osnib_extend(u8 *data, int len, int offset)
{
	int i, ret = 0;
	u8 *ptr, check = 0;
	void __iomem *oshob_addr, *osnibr_addr, *osnibw_addr;
	u32 sptr_dw_mask;

	oshob_addr = ioremap_nocache(poshob_info->oshob_base,
				     poshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_read_osnib_extend: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	pr_info(
		"ipc_read_osnib_extend: remap OSNIB addr=0x%x size %d\n",
		poshob_info->posnibr, poshob_info->osnib_size);

	osnibr_addr = ioremap_nocache(poshob_info->posnibr,
				      poshob_info->osnib_size);

	if (!osnibr_addr) {
		pr_err("ipc_read_osnib_extend: ioremap of osnib failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	/* Make a chksum verification for osnib */
	for (i = 0; i < poshob_info->osnib_size; i++)
		check += readb(osnibr_addr + i);

	if (check) {
		pr_err("ipc_read_osnib_extend: WARNING!!! osnib chksum verification faild, reset all osnib data!\n");
		pr_info(
			"ipc_read_osnib_extend: remap posnibw addr=0x%x size %d\n",
			poshob_info->posnibw, poshob_info->osnib_size);

		osnibw_addr = ioremap_nocache(poshob_info->posnibw,
					      poshob_info->osnib_size);
		if (!osnibw_addr) {
			pr_err("ipc_read_osnib_extend: cannot remap osnib write ptr\n");
			goto unmap_oshob_addr;
		}

		for (i = 0; i < poshob_info->osnib_size; i++)
			writeb(0, osnibw_addr + i);

		/* Send command. The mask to be written identifies which      */
		/* double words of the OSNIB osnib_size bytes will be written.*/
		/* So the mask is coded on 4 bytes.                           */
		sptr_dw_mask = 0xFFFFFFFF;
		intel_scu_ipc_raw_cmd(IPCMSG_WRITE_OSNIB_EXTEND,
			0, NULL, 0, NULL, 0, 0, sptr_dw_mask);
		iounmap(osnibw_addr);
	}

	ptr = data;
	pr_info("ipc_read_osnib_extend: OSNIB content:\n");
	for (i = 0; i < len; i++) {
		*ptr = readb(osnibr_addr + offset + i);
		pr_info("addr=%8x, offset=%2x, value=%2x\n",
			(u32)(osnibr_addr+offset+i), offset+i, *ptr);
		ptr++;
	}

	iounmap(osnibr_addr);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_osnib_extend);


/* This function is used for the extended OSHOB/OSNIB. */
int intel_scu_ipc_write_osnib_extend(u8 *data, int len, int offset)
{
	int i;
	int ret = 0;
	u8 *posnib_data, *ptr;
	u8 check = 0, chksum = 0;
	void __iomem *oshob_addr, *osnibw_addr, *osnibr_addr;
	u32 sptr_dw_mask;

	intel_scu_ipc_lock();

	pr_info(
		"ipc_write_osnib_extend: remap OSHOB addr 0x%8x size %d\n",
		poshob_info->oshob_base, poshob_info->oshob_size);

	oshob_addr = ioremap_nocache(poshob_info->oshob_base,
				     poshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_write_osnib_extend: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	osnibr_addr = ioremap_nocache(poshob_info->posnibr,
				      poshob_info->osnib_size);

	if (!osnibr_addr) {
		pr_err("ipc_write_osnib_extend: ioremap of osnib failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	/* Dump osnib data for generate chksum */
	posnib_data = kzalloc(poshob_info->osnib_size, GFP_KERNEL);

	if (posnib_data == NULL) {
		pr_err("ipc_write_osnib_extend: The buffer for getting OSNIB is NULL\n");
		ret = -EFAULT;
		iounmap(osnibr_addr);
		goto unmap_oshob_addr;
	}

	ptr = posnib_data;
	for (i = 0; i < poshob_info->osnib_size; i++) {
		*ptr = readb(osnibr_addr + i);
		check += *ptr;
		ptr++;
	}

	memcpy(posnib_data + offset, data, len);

	if (check) {
		pr_err("ipc_write_osnib_extend: WARNING!!! OSNIB data chksum verification FAILED!\n");
	} else {
		/* generate chksum.  */
		pr_info("ipc_write_osnib_extend: generating checksum\n");
		for (i = 0; i < poshob_info->osnib_size - 1; i++)
			chksum += *(posnib_data + i);
		/* Fill checksum at the CHECKSUM offset place in OSNIB. */
		*(posnib_data + OSNIB_CHECKSUM_OFFSET) = ~chksum + 1;
	}

	pr_info(
		"ipc_write_osnib_extend: remap posnibw addr=0x%x size %d\n",
		poshob_info->posnibw, poshob_info->osnib_size);

	osnibw_addr = ioremap_nocache(poshob_info->posnibw,
				      poshob_info->osnib_size);
	if (!osnibw_addr) {
		pr_err("scu_ipc_write_osnib_extend: ioremap failed!\n");
		ret = -ENOMEM;
		kfree(posnib_data);
		iounmap(osnibr_addr);
		goto unmap_oshob_addr;
	}

	for (i = 0; i < poshob_info->osnib_size; i++)
		writeb(*(posnib_data + i), (osnibw_addr + i));

	/* Send command. The mask to be written identifies which            */
	/* double words of the OSNIB osnib_size bytes will be written.*/
	/* So the mask is coded on 4 bytes.                                 */
	sptr_dw_mask = 0xFFFFFFFF;
	ret = intel_scu_ipc_raw_cmd(IPCMSG_WRITE_OSNIB_EXTEND, 0, NULL, 0,
			NULL, 0, 0, sptr_dw_mask);
	if (ret < 0)
		pr_err("scu_ipc_write_osnib_extend: ipc_write_osnib failed!!\n");

	iounmap(osnibw_addr);
	iounmap(osnibr_addr);

	kfree(posnib_data);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	intel_scu_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib_extend);


/*
 * This writes the reboot reason in the OSNIB (factor and avoid any overlap)
 */
int intel_scu_ipc_write_osnib_rr(u8 rr)
{
	int ret = 0;

	pr_info("intel_scu_ipc_write_osnib_rr: reboot reason %x\n", rr);
	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_write_osnib(
						&rr,
						1,
						OSNIB_RR_OFFSET);
	else
		ret = intel_scu_ipc_write_osnib_extend(
							&rr,
							1,
							OSNIB_RR_OFFSET);

	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib_rr);

/*
 * This reads the reboot reason from the OSNIB (factor)
 */
int intel_scu_ipc_read_osnib_rr(u8 *rr)
{
	int ret = 0;

	pr_info("intel_scu_ipc_write_osnib_rr: read reboot reason\n");
	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_read_osnib(rr, 1, OSNIB_RR_OFFSET);
	else
		ret = intel_scu_ipc_read_osnib_extend(rr, 1, OSNIB_RR_OFFSET);

	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_osnib_rr);

/*
 * This writes the OEMNIB buffer in the internal RAM of the SCU.
 */
int intel_scu_ipc_write_oemnib(u8 *oemnib, int len, int offset)
{
	int i;
	int ret = 0;
	void __iomem *oshob_addr, *oemnibw_addr;
	u32 sptr_dw_mask;

	if (oemnib == NULL) {
		pr_err("ipc_write_oemnib: passed buffer for writting OEMNIB is NULL\n");
		return -EINVAL;
	}

	intel_scu_ipc_lock();

	pr_info("ipc_write_oemnib: remap OSHOB addr 0x%8x size %d\n",
		poshob_info->oshob_base, poshob_info->oshob_size);

	oshob_addr = ioremap_nocache(poshob_info->oshob_base,
				     poshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_write_oemnib: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	if ((len == 0) || (len > poshob_info->oemnib_size)) {
		pr_err(
			"ipc_write_oemnib: bad OEMNIB data length (%d) to write (max=%d bytes)\n",
			    len, poshob_info->oemnib_size);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	/* offset shall start at 0 from the OEMNIB base address and shall */
	/* not exceed the OEMNIB allowed size.                            */
	if ((offset < 0) || (offset >= poshob_info->oemnib_size) ||
	    (len + offset > poshob_info->oemnib_size)) {
		pr_err(
			"ipc_write_oemnib: Bad OEMNIB data offset/len for writing (offset=%d , len=%d)\n",
			offset, len);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	pr_info("ipc_write_oemnib: POEMNIB remap poemnibw 0x%x size %d\n",
		poshob_info->poemnibw, poshob_info->oemnib_size);

	oemnibw_addr = ioremap_nocache(poshob_info->poemnibw,
				       poshob_info->oemnib_size);
	if (!oemnibw_addr) {
		pr_err("ipc_write_oemnib: ioremap failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	for (i = 0; i < len; i++)
		writeb(*(oemnib + i), (oemnibw_addr + offset + i));

	/* Send command. The mask to be written identifies which double */
	/* words of the OSNIB oemnib_size bytes will be written.        */
	/* So the mask is coded on 4 bytes.                             */
	sptr_dw_mask = 0xFFFFFFFF;
	ret = intel_scu_ipc_raw_cmd(IPCMSG_WRITE_OEMNIB, 0, NULL, 0, NULL,
			0, 0, sptr_dw_mask);
	if (ret < 0)
		pr_err("ipc_write_oemnib: ipc_write_osnib failed!!\n");

	iounmap(oemnibw_addr);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	intel_scu_ipc_unlock();

	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_oemnib);

/*
 * This reads the OEMNIB from the internal RAM of the SCU.
 */
static int intel_scu_ipc_read_oemnib(u8 *oemnib, int len, int offset)
{
	int i, ret = 0;
	u8 *ptr;
	void __iomem *oshob_addr, *oemnibr_addr;

	if (oemnib == NULL) {
		pr_err("ipc_read_oemnib: passed buffer for reading OEMNIB is NULL\n");
		return -EINVAL;
	}

	pr_info("ipc_read_oemnib: remap OSHOB base addr 0x%x size %d\n",
		poshob_info->oshob_base, poshob_info->oshob_size);

	oshob_addr = ioremap_nocache(poshob_info->oshob_base,
				     poshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_read_oemnib: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	if ((len == 0) || (len > poshob_info->oemnib_size)) {
		pr_err("ipc_read_oemnib: Bad OEMNIB data length (%d) to be read (max=%d bytes)\n",
			    len, poshob_info->oemnib_size);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	/* offset shall start at 0 from the OEMNIB base address and shall */
	/* not exceed the OEMNIB allowed size.                            */
	if ((offset < 0) || (offset >= poshob_info->oemnib_size) ||
	    (len + offset > poshob_info->oemnib_size)) {
		pr_err(
		"ipc_read_oemnib: Bad OEMNIB data offset/len to read (offset=%d ,len=%d)\n",
		offset, len);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	pr_info("ipc_read_oemnib: POEMNIB remap poemnibr 0x%x size %d\n",
		poshob_info->poemnibr, poshob_info->oemnib_size);

	oemnibr_addr = ioremap_nocache(poshob_info->poemnibr,
				       poshob_info->oemnib_size);

	if (!oemnibr_addr) {
		pr_err("ipc_read_oemnib: ioremap of oemnib failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	ptr = oemnib;
	pr_info("ipc_read_oemnib: OEMNIB content:\n");
	for (i = 0; i < len; i++) {
		*ptr = readb(oemnibr_addr + offset + i);
		pr_info("addr=%8x, offset=%2x, value=%2x\n",
			(u32)(oemnibr_addr+offset+i), offset+i, *ptr);
		ptr++;
	}

	iounmap(oemnibr_addr);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_oemnib);

/*
 * This reads the PMIT from the OSHOB (pointer to interrupt tree)
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_oshob_it_tree(u32 *ptr)
{
	pr_info("intel_scu_ipc_read_oshob_it_tree: read IT tree\n");

	return intel_scu_ipc_read_oshob((u8 *) ptr, 4, OSHOB_PMIT_OFFSET);
}
#endif

/*
 * This reads the RESETIRQ1 from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_resetirq1(u8 *rirq1)
{
	int ret = 0;

	pr_info("intel_scu_ipc_read_osnib_resetirq1: read RESETIRQ1\n");

	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_read_osnib(
						rirq1,
						1,
						OSNIB_RESETIRQ1_OFFSET);
	else
		ret = intel_scu_ipc_read_osnib_extend(
						rirq1,
						1,
						OSNIB_RESETIRQ1_OFFSET);

	return ret;
}
#endif

/*
 * This reads the RESETIRQ2 from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_resetirq2(u8 *rirq2)
{
	int ret = 0;

	pr_info("intel_scu_ipc_read_osnib_resetirq1: read RESETIRQ2\n");

	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_read_osnib(
						rirq2,
						1,
						OSNIB_RESETIRQ2_OFFSET);
	else
		ret = intel_scu_ipc_read_osnib_extend(
						rirq2,
						1,
						OSNIB_RESETIRQ2_OFFSET);

	return ret;
}
#endif

/*
 * This reads the WD from the OSNIB
 */
int intel_scu_ipc_read_osnib_wd(u8 *wd)
{
	int ret = 0;

	pr_info("intel_scu_ipc_read_osnib_resetirq1: read WATCHDOG\n");

	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_read_osnib(wd, 1, OSNIB_WD_OFFSET);
	else
		ret = intel_scu_ipc_read_osnib_extend(wd, 1, OSNIB_WD_OFFSET);

	return ret;
}

/*
 * This writes the WD in the OSNIB
 */
int intel_scu_ipc_write_osnib_wd(u8 *wd)
{
	int ret = 0;

	pr_info("intel_scu_ipc_write_osnib_wd: write WATCHDOG %x\n", *wd);

	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_write_osnib(wd, 1, OSNIB_WD_OFFSET);
	else
		ret = intel_scu_ipc_write_osnib_extend(wd, 1, OSNIB_WD_OFFSET);

	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib_wd);

/*
 * This reads the ALARM from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_alarm(u8 *alarm)
{
	int ret = 0;

	pr_info("intel_scu_ipc_read_osnib_alarm: read ALARM\n");

	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_read_osnib(alarm, 1, OSNIB_ALARM_OFFSET);
	else
		ret = intel_scu_ipc_read_osnib_extend(
							alarm,
							1,
							OSNIB_ALARM_OFFSET);

	return ret;
}
#endif

/*
 * This reads the WAKESRC from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_wakesrc(u8 *wakesrc)
{
	int ret = 0;

	pr_info("intel_scu_ipc_read_osnib_wakesrc: read WAKESRC\n");

	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD)
		ret = intel_scu_ipc_read_osnib(
						wakesrc,
						1,
						OSNIB_WAKESRC_OFFSET);
	else
		ret = intel_scu_ipc_read_osnib_extend(
							wakesrc,
							1,
							OSNIB_WAKESRC_OFFSET);

	return ret;
}
#endif


#define OEMNIB_BUF_DESC_LEN	4096

#ifdef CONFIG_DEBUG_FS

static int intel_scu_ipc_oshob_stat(struct seq_file *m, void *unused)
{
	void __iomem *osnib;
	int i, count;
	int ret = 0;

	u32 value;
	if (poshob_info->oshob_type == OSHOB_STRUCT_OLD) {
		seq_printf(m, "OLD OSHOB\n");
		seq_printf(m, "OSHOB size : %d\n", poshob_info->oshob_size);
		seq_printf(m, "SCU trace : %x\n", poshob_info->scu_trace);
		seq_printf(m, "IA trace  : %x\n", poshob_info->ia_trace);
	} else {
		seq_printf(m, "NEW OSHOB v%d.%d\n", poshob_info->oshob_majrev,
						poshob_info->oshob_minrev);
		seq_printf(m, "OSHOB size : %d\n\n", poshob_info->oshob_size);
		seq_printf(m, "SCU trace : %x\n", poshob_info->scu_trace);
		seq_printf(m, "IA trace  : %x\n\n", poshob_info->ia_trace);

		seq_printf(m, "OSNIB size : %d\n", poshob_info->osnib_size);
		seq_printf(m, "OSNIB  read address  : %x\n",
							poshob_info->posnibr);
		seq_printf(m, "OSNIB  write address : %x\n",
							poshob_info->posnibw);
		/* Dump OSNIB */
		osnib = ioremap_nocache(poshob_info->posnibr,
						poshob_info->osnib_size);
		if (!osnib) {
			pr_err("Cannot remap OSNIB\n");
			ret = -ENOMEM;
			return ret;
		}

		i = 0;
		count = 0; /* used for fancy presentation */
		while (i < poshob_info->osnib_size) {
			if (count%4 == 0)
				seq_printf(m, "\nOSNIB[%08x] ",
						poshob_info->posnibr+i);

			value = readl(osnib+i);
			seq_printf(m, "%08x ", value);
			i += 4;
			count++;
		}
		seq_printf(m, "\n\n");
		iounmap(osnib);

		seq_printf(m, "OEMNIB size : %d\n",
						poshob_info->oemnib_size);
		seq_printf(m, "OEMNIB read address  : %x\n",
						poshob_info->poemnibr);
		seq_printf(m, "OEMNIB write address : %x\n",
						poshob_info->poemnibw);
		seq_printf(m, "\n\n");
	}
	return 0;
}

static int intel_scu_ipc_oemnib_stat(struct seq_file *m, void *unused)
{
	void __iomem *oemnib;
	int i, count;
	u32 value;

	seq_printf(m, "OEMNIB size : %d\n",
					poshob_info->oemnib_size);
	seq_printf(m, "OEMNIB read address  : %x\n",
					poshob_info->poemnibr);
	seq_printf(m, "OEMNIB write address : %x\n",
					poshob_info->poemnibw);
	/* Dump OEMNIB */
	oemnib = ioremap_nocache(poshob_info->poemnibr,
				poshob_info->oemnib_size);

	if (!oemnib) {
		pr_err("Cannot remap OEMNIB\n");
		return -ENOMEM;
	}

	i = 0;
	count = 0; /* used for fancy presentation */
	while (i < poshob_info->oemnib_size) {
		if (count%4 == 0)
			seq_printf(m, "\nOEMNIB[%08x] ",
				    poshob_info->poemnibr+i);

		value = readl(oemnib+i);
		seq_printf(m, "%08x ", value);
		i += 4;
		count++;
	}
	seq_printf(m, "\n\n");
	iounmap(oemnib);

	return 0;
}

static ssize_t intel_scu_ipc_oshob_open(struct inode *inode, struct file *file)
{
	return single_open(file, intel_scu_ipc_oshob_stat, NULL);
}

static ssize_t intel_scu_ipc_oemnib_open(struct inode *inode, struct file *file)
{
	return single_open(file, intel_scu_ipc_oemnib_stat, NULL);
}


/*
*	debugfs interface: the "oemnib_write" stores the OEMNIB part of OSNIB,
*       starting at offset ppos.
*/
static ssize_t intel_scu_ipc_oemnib_write(struct file *file,
					  const char __user *buf,
					    size_t count, loff_t *ppos)
{
	int ret, i;
	u8 *posnib_data, *ptr;
	char *ptrchar, *temp;

	if (poshob_info->oshob_type != OSHOB_STRUCT_EXTENDED) {
		/* OEMNIB only usable with new OSHOB structure. */
		pr_err(
		"Write OEMNIB: OEMNIB only usable with new OSHOB structure.\n");
		return -EFAULT;
	}

	pr_info("Write OEMNIB: number bytes = %d\n", count);

	/* Note: when the string is passed through debugfs interface, the  */
	/* real count value includes the end of line \n. So we must take   */
	/* care to consider count - 1 as the real number of OEM bytes.     */

	if (buf == NULL) {
		pr_err("Write OEMNIB: The passed OEMNIB buffer is NULL\n");
		return -EINVAL;
	}

	if (count == 0) {
		pr_err("Write OEMNIB: The OEMNIB data length to write is NULL\n");
		return -EINVAL;
	}

	posnib_data = kzalloc(count - 1, GFP_KERNEL);

	if (posnib_data == NULL) {
		pr_err("Write OEMNIB: Cannot allocate buffer for writting OEMNIB\n");
		return -ENOMEM;
	}

	memset(posnib_data, 0, count - 1);

	temp = kzalloc(count - 1, GFP_KERNEL);

	if (temp == NULL) {
		pr_err(
		"Write OEMNIB: Cannot allocate temp buffer for writting OEMNIB\n");
		return -ENOMEM;
	}

	memset(temp, 0, count - 1);

	if (copy_from_user(temp, buf, count - 1)) {
		pr_err(
		"Write OEMNIB: Cannot transfer from user buf to OEMNIB buf\n");
		kfree(posnib_data);
		return -EFAULT;
	}

	ptrchar = temp;
	ptr = posnib_data;

	for (i = 0; i <= count - 1; i++) {
		if (*ptrchar >= '0' && *ptrchar <= '9')
			*ptr = *ptrchar - '0';
		if (*ptrchar >= 'A' && *ptrchar <= 'F')
			*ptr = *ptrchar - 'A' + 10;
		if (*ptrchar >= 'a' && *ptrchar <= 'f')
			*ptr = *ptrchar - 'a' + 10;

		ptrchar++;
		ptr++;
	}

	ret = intel_scu_ipc_write_oemnib(posnib_data, count - 1, *ppos);

	if (ret < 0) {
		pr_err("Write OEMNIB: ipc write of OEMNIB failed!!\n");
		kfree(posnib_data);
		return ret;
	}

	kfree(posnib_data);
	kfree(temp);

	pr_info("Write OEMNIB: OEMNIB updated: count=%d bytes\n", count);

	return count;
}

/* Attach the debugfs operations methods */
static const struct file_operations scu_ipc_oemnib_fops = {
	.owner = THIS_MODULE,
	.open = intel_scu_ipc_oemnib_open,
	.read = seq_read,
	.write = intel_scu_ipc_oemnib_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations scu_ipc_oshob_fops = {
	.owner = THIS_MODULE,
	.open = intel_scu_ipc_oshob_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *scu_ipc_oemnib_dir;
static struct dentry *scu_ipc_oemnib_file;
static struct dentry *scu_ipc_oshob_file;

/*
*	debugfs interface: init interface.
*/
static int intel_mid_scu_ipc_oemnib_debugfs_init(void)
{
	/* Create debugfs directory /sys/kernel/debug/intel_scu_oshob */
	scu_ipc_oemnib_dir = debugfs_create_dir("intel_scu_oshob", NULL);

	if (!scu_ipc_oemnib_dir) {
		pr_err("cannot create OSHOB debugfs directory\n");
		return -1;
	}

	/* Add operations /sys/kernel/debug/intel_scu_oshob to control */
	/* the OEM.                                                     */
	scu_ipc_oemnib_file = debugfs_create_file("oemnib_debug",
				S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				scu_ipc_oemnib_dir,
				NULL, &scu_ipc_oemnib_fops);

	if (!scu_ipc_oemnib_file) {
		pr_err("cannot create OEMNIB debugfs file\n");
		debugfs_remove(scu_ipc_oemnib_dir);
		return -1;
	}

	/* Add operations /sys/kernel/debug/intel_scu_oshob to debug OSHOB */
	/* content.                                                         */
	scu_ipc_oshob_file = debugfs_create_file("oshob_dump",
				S_IFREG | S_IRUGO | S_IWUSR | S_IWGRP,
				scu_ipc_oemnib_dir, NULL, &scu_ipc_oshob_fops);

	if (!scu_ipc_oshob_file) {
		pr_err("cannot create OSHOB debugfs file\n");
		debugfs_remove_recursive(scu_ipc_oemnib_dir);
		return -1;
	}

	return 0;
}

/*
*	debugfs interface: exit interface.
*/
static void intel_mid_scu_ipc_oemnib_debugfs_exit(void)
{
	debugfs_remove_recursive(scu_ipc_oemnib_dir);
}

#endif /* CONFIG_DEBUG_FS */

static const struct file_operations scu_ipc_fops = {
	.unlocked_ioctl = scu_ipc_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = scu_ipc_ioctl,
#endif
};

static struct miscdevice scu_ipcutil = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mid_ipc",
	.fops = &scu_ipc_fops,
};

static int __init ipc_module_init(void)
{
	int ret;

#ifdef DUMP_OSNIB
	u8 rr, resetirq1, resetirq2, wd, alarm, wakesrc, *ptr;
	u32 pmit, scu_trace, ia_trace;
#endif

	poshob_info = kmalloc(sizeof(struct scu_ipc_oshob_info), GFP_KERNEL);

	if (poshob_info == NULL) {
		pr_err(
		"Cannot init ipc module: oshob info struct not allocated\n");
		return -ENOMEM;
	}

	/* Identify the type and size of OSHOB to be used. */
	ret = intel_scu_ipc_read_oshob_info();

	if (ret != 0) {
		pr_err("Cannot init ipc module: oshob info not read\n");
		goto exit;
	}

#ifdef DUMP_OSNIB
	/* Dumping RESETIRQ1 and 2 from the interrupt tree */
	ret = intel_scu_ipc_read_oshob_it_tree(&pmit);

	if (ret != 0) {
		pr_err("Cannot read interrupt tree\n");
		goto exit;
	}

	ptr = ioremap_nocache(pmit + PMIT_RESETIRQ1_OFFSET, 2);

	if (!ptr) {
		pr_err("Cannot remap PMIT\n");
		ret = -ENOMEM;
		goto exit;
	}

	pr_info("PMIT addr 0x%8x remapped to 0x%8x\n", pmit, (u32)ptr);

	resetirq1 = readb(ptr);
	resetirq2 = readb(ptr+1);
	pr_warn("[BOOT] RESETIRQ1=0x%02x RESETIRQ2=0x%02x (interrupt tree)\n",
		resetirq1, resetirq2);
	iounmap(ptr);

	/* Dumping OSHOB content */
	if (poshob_info->oshob_type == OSHOB_STRUCT_EXTENDED) {
		ret = intel_scu_ipc_read_oshob(&scu_trace,
					       4,
					       OSHOB_EXTEND_SCU_TRACE_OFFSET);

		if (ret != 0) {
			pr_err("Cannot read SCU data\n");
			goto exit;
		}

		ret = intel_scu_ipc_read_oshob(&ia_trace,
					       4,
					       OSHOB_EXTEND_IA_TRACE_OFFSET);

		if (ret != 0) {
			pr_err("Cannot read IA data\n");
			goto exit;
		}
	} else {
		/* Use old OSHOB here. */

		ret = intel_scu_ipc_read_oshob(&scu_trace,
					       4,
					       OSHOB_SCU_TRACE_OFFSET);

		if (ret != 0) {
			pr_err("Cannot read SCU data\n");
			goto exit;
		}

		ret = intel_scu_ipc_read_oshob(&ia_trace,
					       4,
					       OSHOB_IA_TRACE_OFFSET);

		if (ret != 0) {
			pr_err("Cannot read IA data\n");
			goto exit;
		}
	}

	pr_warn("[BOOT] SCU_TR=0x%08x IA_TR=0x%08x (oshob)\n",
		scu_trace, ia_trace);
	/* Dumping OSNIB content */
	ret = intel_scu_ipc_read_osnib_rr(&rr);
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

#ifdef CONFIG_DEBUG_FS
	if (poshob_info->oshob_type == OSHOB_STRUCT_EXTENDED) {
		/* OEMNIB only usable with new OSHOB structure. */
		ret = intel_mid_scu_ipc_oemnib_debugfs_init();

		if (ret != 0) {
			pr_err("Cannot register OEMNIB interface to debugfs\n");
			goto exit;
		} else {
			pr_info("OEMNIB interface registered to debugfs\n");
		}
	}
#endif /* CONFIG_DEBUG_FS */

exit:
	if (ret != 0) {
		kfree(poshob_info);
		return ret;
	} else {
		/* return result of scu_ipc_ioctl registration to sysfs. */
		return misc_register(&scu_ipcutil);
	}
}

static void __exit ipc_module_exit(void)
{
#ifdef CONFIG_DEBUG_FS
	if (poshob_info->oshob_type == OSHOB_STRUCT_EXTENDED) {
		/* OEMNIB only usable with new OSHOB structure. */
		/* unregister from debugfs.                     */
		intel_mid_scu_ipc_oemnib_debugfs_exit();
	}
#endif /* CONFIG_DEBUG_FS */

	/* unregister scu_ipc_ioctl from sysfs. */
	misc_deregister(&scu_ipcutil);

	kfree(poshob_info);
}

rootfs_initcall(ipc_module_init);
module_exit(ipc_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Utility driver for intel scu ipc");
MODULE_AUTHOR("Sreedhara <sreedhara.ds@intel.com>");
