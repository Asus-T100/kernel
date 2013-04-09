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
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/rpmsg.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#define MAX_FW_SIZE 264192

#define PMIT_RESETIRQ1_OFFSET		14
#define PMIT_RESETIRQ2_OFFSET		15

#define IPC_RESIDENCY_CMD_ID_START	0
#define IPC_RESIDENCY_CMD_ID_DUMP	2

#define SRAM_ADDR_S0IX_RESIDENCY	0xFFFF71E0
#define ALL_RESIDENCY_DATA_SIZE		12

#define DUMP_OSNIB

#define OSHOB_EXTEND_DESC_SIZE	52  /* OSHOB header+osnib+oem info: 52 bytes.*/

#define OSHOB_HEADER_MAGIC_SIZE	4   /* Size (bytes) of magic number in OSHOB */
				    /* header.                               */

#define OSHOB_MAGIC_NUMBER	"$OH$"	/* If found when reading the first   */
					/* 4 bytes of the OSOHB zone, it     */
					/* means that the new extended OSHOB */
					/* is going to be used.              */

#define OSHOB_REV_MAJ_DEFAULT	0	/* Default revision number of OSHOB. */
#define OSHOB_REV_MIN_DEFAULT	1	/* If 0.1 the default OSHOB is used  */
					/* instead of the extended one.      */

/* Defines for the SCU buffer included in OSHOB structure. */
#define OSHOB_SCU_BUFFER_SIZE	4    /* In dwords. On Merrifield the needed */
				     /* SCU trace buffer size is 4 dwords.  */

#define OSHOB_SCU_BUFFER_SIZE_BYTES   (OSHOB_SCU_BUFFER_SIZE * 4)

/* Size (bytes) of the default OSHOB structure. Includes the default OSNIB   */
/* size.                                                                     */
#define OSHOB_SIZE		(56 + (4*OSHOB_SCU_BUFFER_SIZE))/* in bytes. */
/* OSHOB_SCU_BUFFER_SIZE is give in dwords. So it is x4 to get the number of */
/* bytes.                                                                    */

#define OSNIB_SIZE		32	/* Size (bytes) of the default OSNIB.*/

#define OSNIB_INTEL_RSVD_SIZE	14	/* Size (bytes) of Intel RESERVED in */
					/* OSNIB.                            */
#define OSNIB_OEM_RSVD_SIZE	10	/* Size (bytes) of OEM RESERVED      */
					/* in OSNIB.                         */

#define OSHOB_FABRIC_ERROR1_SIZE  12    /* 1st part of Fabric error dump     */
#define OSHOB_FABRIC_ERROR2_SIZE  9     /* 2nd part of Fabric error dump     */
#define OSHOB_RESERVED_DEBUG_SIZE 5     /* Reserved for debug                */

/* OSNIB allocation. */
struct scu_ipc_osnib {
	u8 target_mode;        /* Target mode.                      */
	u8 wd_count;           /* Software watchdog.                */
	u8 alarm;              /* RTC alarm.                        */
	u8 wakesrc;            /* WAKESRC.                          */
	u8 resetirq1;          /* RESETIRQ1.                        */
	u8 resetirq2;          /* RESETIRQ2.                        */
	u8 spare;              /* Spare.                            */
	u8 intel_reserved[OSNIB_INTEL_RSVD_SIZE]; /* INTEL RESERVED */
			       /* (offsets 7 to 20).                */
	u8 oem_reserved[OSNIB_OEM_RSVD_SIZE];     /* OEM RESERVED   */
			       /* (offsets 21 to 30).               */
	u8 checksum;           /* CHECKSUM.                         */
};

static u32 scutxl_base;
static u32 scutxl_ext[OSHOB_SCU_BUFFER_SIZE]; /* For MRFLD (CONFIG_X86_MRFLD)*/

/* Default OSHOB allocation. */
struct scu_ipc_oshob {
	u32 *scutxl_ptr;        /* SCUTxl.                      */
	u32 iatxl;              /* IATxl offset.                */
	u32 bocv;               /* BOCV offset.                 */
	u8 osnibr[OSNIB_SIZE];  /* OSNIB area offset.           */
	u32 pmit;               /* PMIT offset.                 */
	u32 pemmcmhki;          /* PeMMCMHKI offset.            */
	u32 osnibw_ptr;         /* OSNIB Write at offset 0x34.  */
	u8 oshob_reserved;      /* First byte of RESERVED zone  */
};

struct scu_ipc_oshob scu_ipc_oshob_default;


/* Extended OSHOB allocation. */
struct scu_ipc_oshob_extend {
	u32 magic;              /* MAGIC number.               */
	u8  rev_major;          /* Revision major.             */
	u8  rev_minor;          /* Revision minor.             */
	u16 oshob_size;         /* OSHOB size.                 */
	u32 head_reserved;      /* OSHOB RESERVED.             */
	u32 *scutxl_ptr;        /* SCUTxl buffer.              */
	u32 iatxl;              /* IATxl.                      */
	u32 bocv;               /* BOCV.                       */

	u16 intel_size;         /* Intel size (in OSNIB area). */
	u16 oem_size;           /* OEM size (of OEM area).     */
	u32 r_intel_ptr;        /* Read Intel pointer.         */
	u32 w_intel_ptr;        /* Write Intel pointer.        */
	u32 r_oem_ptr;          /* Read OEM pointer.           */
	u32 w_oem_ptr;          /* Write OEM pointer.          */

	u32 pmit;               /* PMIT.                       */
	u32 pemmcmhki;          /* PeMMCMHKI.                  */

	/* OSHOB as defined for CLOVERVIEW */
	u32 reserved1;          /* Reserved field              */
	u32 fabricerrlog1[OSHOB_FABRIC_ERROR1_SIZE]; /* fabric error data */
	u8  vrtc_alarm_dow;     /* Alarm sync                  */
	u8  vrtc_alarm_dom;     /* Alarm sync                  */
	u8  vrtc_alarm_month;   /* Alarm sync                  */
	u8  vrtc_alarm_year;    /* Alarm sync                  */
	u32 reserved_debug[OSHOB_RESERVED_DEBUG_SIZE];/* Reserved Debug data */
	u32 reserved2;          /* Reserved                    */
	u32 fabricerrlog2[OSHOB_FABRIC_ERROR2_SIZE]; /* fabric error data2 */
	u32 sculogbufferaddr;   /* phys addr of scu log buffer   */
	u32 sculogbuffersize;   /* size of scu log buffer      */
};

struct scu_ipc_oshob_extend scu_ipc_oshob_extend_struct;

struct scu_ipc_oshob_info {
	__u32	oshob_base;     /* Base address of OSHOB. Use ioremap to     */
				/* remap for access.                         */
	__u8	oshob_majrev;   /* Major revision number of OSHOB structure. */
	__u8	oshob_minrev;   /* Minor revision number of OSHOB structure. */
	__u16	oshob_size;     /* Total size (bytes) of OSHOB structure.    */
	__u32   scu_trace[OSHOB_SCU_BUFFER_SIZE]; /* SCU trace buffer.       */
				/* Buffer max size is OSHOB_SCU_BUFFER_SIZE  */
				/* dwords for MRFLD. On other platforms,     */
				/* only the first dword is stored and read.  */
	__u32   ia_trace;       /* IA trace buffer.                          */
	__u16	osnib_size;     /* Total size (bytes) of OSNIB structure.    */
	__u16	oemnib_size;    /* Total size (bytes) of OEMNIB area.        */
	__u32	osnibr_ptr;     /* Pointer to Intel read zone.               */
	__u32	osnibw_ptr;     /* Pointer to Intel write zone.              */
	__u32	oemnibr_ptr;    /* Pointer to OEM read zone.                 */
	__u32	oemnibw_ptr;    /* Pointer to OEM write zone.                */
	__u32   scu_trace_buf;  /* SCU extended trace buffer                 */
	__u32   scu_trace_size; /* SCU extended trace buffer size            */

	int (*scu_ipc_write_osnib)(u8 *data, int len, int offset);
	int (*scu_ipc_read_osnib)(u8 *data, int len, int offset);

	int platform_type;     /* Identifies the platform (list of supported */
			       /* platforms is given in intel-mid.h).        */

	u16 offs_add;          /* The additional shift bytes to consider     */
			       /* giving the offset at which the OSHOB param */
			       /* will be read. If MRFLD it must be set to   */
			       /* OSHOB_SCU_BUFFER_SIZE dwords.              */

};

/* Structure for OSHOB info */
static struct scu_ipc_oshob_info *oshob_info;

static struct rpmsg_instance *ipcutil_instance;

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

	ipc_ret = rpmsg_send_command(ipcutil_instance,
		IPCMSG_OSC_CLK, 0, ipc_wbuf, NULL, 2, 0);
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

	pr_info("set_clk0 request %s for Mode 0x%x\n",
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
		pr_info("set_clk0: enabling clk, mode 0x%x\n", osc_clk0_mode);
		clk_enable = 1;
	} else {
		osc_clk0_mode &= ~mode;
		pr_info("set_clk0: disabling clk, mode 0x%x\n", osc_clk0_mode);
		/* others using the clock, cannot turn it of */
		if (osc_clk0_mode)
			goto out;
		clk_enable = 0;
	}
	pr_info("configuring OSC_CLK_AUDIO now\n");
	ret = intel_scu_ipc_osc_clk(OSC_CLK_AUDIO, clk_enable ? clk_khz : 0);
out:
	mutex_unlock(&osc_clk0_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_set_osc_clk0);

#define MSIC_VPROG1_CTRL        0xD6
#define MSIC_VPROG2_CTRL        0xD7

#define MSIC_VPROG2_ON          0x36 /*1.200V and Auto mode*/
#define MSIC_VPROG1_ON          0xF6 /*2.800V and Auto mode*/
#define MSIC_VPROG_OFF          0x24 /*1.200V and OFF*/

/* Defines specific of MRFLD platform (CONFIG_X86_MRFLD). */
#define MSIC_VPROG1_MRFLD_CTRL	0xAC
#define MSIC_VPROG2_MRFLD_CTRL	0xAD

#define MSIC_VPROG1_MRFLD_ON	0xC1	/* 2.80V */
#define MSIC_VPROG2_MRFLD_ON	0xC1	/* 2.80V */
#define MSIC_VPROG_MRFLD_OFF	0	/* OFF */
/* End of MRFLD specific.*/

/* Helpers to turn on/off msic vprog1 and vprog2 */
int intel_scu_ipc_msic_vprog1(int on)
{
	if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER)
		return intel_scu_ipc_iowrite8(MSIC_VPROG1_MRFLD_CTRL,
			on ? MSIC_VPROG1_MRFLD_ON : MSIC_VPROG_MRFLD_OFF);
	else
		return intel_scu_ipc_iowrite8(MSIC_VPROG1_CTRL,
			on ? MSIC_VPROG1_ON : MSIC_VPROG_OFF);
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_msic_vprog1);

int intel_scu_ipc_msic_vprog2(int on)
{
	if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER)
		return intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL,
			on ? MSIC_VPROG2_MRFLD_ON : MSIC_VPROG_MRFLD_OFF);
	else
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
		(p_buf->count + p_buf->offset < size) &&
		(p_buf->count % 4 == 0);
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

	/* 1. we use rpmsg_send_raw_command() IPC cmd interface
	 *    to access PMDB data. Each call of rpmsg_send_raw_command()
	 *    can only access at most PMDB_ACCESS_SIZE bytes' data.
	 * 2. There are two kinds of pmdb sub commands, read command
	 *    and write command. For read command, we must transport
	 *    in and out buffer to rpmsg_send_raw_command(), because
	 *    in buffer length is pass as access length which must
	 *    be transported to SCU.
	 */
	p_data = p_buf->data;
	offset = p_buf->offset;
	for (i = 0; i < p_buf->count/PMDB_ACCESS_SIZE; i++) {
		if (pmdb_sub_cmd_is_read(p_buf->sub))
			ret = rpmsg_send_raw_command(ipcutil_instance,
					IPCMSG_PMDB_CMD, p_buf->sub,
					p_data, (u32 *)p_data,
					PMDB_ACCESS_SIZE, PMDB_ACCESS_SIZE / 4,
					0, offset);
		else
			ret = rpmsg_send_raw_command(ipcutil_instance,
					IPCMSG_PMDB_CMD, p_buf->sub,
					p_data, NULL, PMDB_ACCESS_SIZE,
					0, 0, offset);
		if (ret < 0) {
			pr_err("intel_scu_ipc_raw_cmd failed!\n");
			return ret;
		}
		offset += PMDB_ACCESS_SIZE;
		p_data += PMDB_ACCESS_SIZE;
	}
	if (p_buf->count % PMDB_ACCESS_SIZE > 0) {
		if (pmdb_sub_cmd_is_read(p_buf->sub))
			ret = rpmsg_send_raw_command(ipcutil_instance,
					IPCMSG_PMDB_CMD, p_buf->sub,
					p_data, (u32 *)p_data,
					p_buf->count % PMDB_ACCESS_SIZE,
					(p_buf->count % PMDB_ACCESS_SIZE) / 4,
					0, offset);
		else
			ret = rpmsg_send_raw_command(ipcutil_instance,
					IPCMSG_PMDB_CMD, p_buf->sub,
					p_data, NULL,
					p_buf->count % PMDB_ACCESS_SIZE,
					0, 0, offset);
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

	/* Only IOCTL cmd allowed to pass through without capability check */
	/* is getting fw version info, all others need to check to prevent */
	/* arbitrary access to all sort of bit of the hardware exposed here*/

	if ((cmd != INTEL_SCU_IPC_FW_REVISION_GET ||
		cmd != INTEL_SCU_IPC_S0IX_RESIDENCY) &&
		!capable(CAP_SYS_RAWIO))
		return -EPERM;

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

		ret = rpmsg_send_simple_command(ipcutil_instance,
					IPCMSG_S0IX_COUNTER, cmd_id);

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

		ret = oshob_info->scu_ipc_read_osnib(
				&data,
				1,
				offsetof(struct scu_ipc_osnib, alarm));

		if (ret < 0)
			return ret;
		if (flag) {
			data = data | 0x1; /* set alarm flag */
			pr_info("scu_ipc_ioctl: set alarm flag\n");
		} else {
			data = data & 0xFE; /* clear alarm flag */
			pr_info("scu_ipc_ioctl: clear alarm flag\n");
		}

		ret = oshob_info->scu_ipc_write_osnib(
				&data,
				1,
				offsetof(struct scu_ipc_osnib, alarm));

		break;
	}
	case INTEL_SCU_IPC_READ_VBATTCRIT:
	{
		u32 value = 0;

		pr_info("cmd = INTEL_SCU_IPC_READ_VBATTCRIT");
		ret = intel_scu_ipc_read_mip((u8 *)&value, 4, 0x318, 1);
		if (ret < 0)
			return ret;
		pr_info("VBATTCRIT VALUE = %x\n", value);
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

		ret = rpmsg_send_command(ipcutil_instance,
			IPCMSG_FW_REVISION, 0, NULL, (u32 *)version.data, 0, 4);
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

int intel_scu_ipc_get_oshob_size(void)
{
	return oshob_info->oshob_size;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_get_oshob_size);

int intel_scu_ipc_read_oshob(u8 *data, int len, int offset)
{
	int ret = 0, i;
	void __iomem *oshob_addr;
	u8 *ptr = data;

	oshob_addr = ioremap_nocache(
				    oshob_info->oshob_base,
				    oshob_info->oshob_size);

	if (!oshob_addr) {
		pr_err("ipc_read_oshob: addr ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	for (i = 0; i < len; i = i+1) {
		*ptr = readb(oshob_addr + offset + i);
		pr_debug("addr(remapped)=%8x, offset=%2x, value=%2x\n",
			(u32)(oshob_addr + i),
			offset + i, *ptr);
		ptr++;
	}

	iounmap(oshob_addr);
exit:
	return ret;
}

EXPORT_SYMBOL_GPL(intel_scu_ipc_read_oshob);

/* This option is used to write to the default OSNIB. */
#define IPCMSG_WRITE_OSNIB		0xE4

/* This option is used to write to the extended OSNIB. */
#define IPCMSG_WRITE_OSNIB_EXTEND	0xE4

/* This command is used to write the OEMNIB data.   */
/* Used with the extended OSHOB OSNIB only.     */
#define IPCMSG_WRITE_OEMNIB		0xDF


/* This function is used for the default OSNIB. */
int intel_scu_ipc_read_osnib(u8 *data, int len, int offset)
{
	int i, ret = 0;
	u32 osnibw_ptr;
	u8 *ptr, check = 0;
	u16 struct_offs;
	void __iomem *oshob_addr, *osnibr_addr, *osnibw_addr;

	pr_debug("OSHOB base addr value is %x\n", oshob_info->oshob_base);
	oshob_addr = ioremap_nocache(oshob_info->oshob_base,
				     oshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	struct_offs = offsetof(struct scu_ipc_oshob, osnibr) +
			    oshob_info->offs_add;
	osnibr_addr = oshob_addr + struct_offs;

	if (!osnibr_addr) {
		pr_err("Bad osnib address!\n");
		ret = -EFAULT;
		iounmap(oshob_addr);
		goto exit;
	}

	pr_debug("OSNIB read addr (remapped) is %x\n",
						(unsigned int)osnibr_addr);

	/* Make a chksum verification for osnib */
	for (i = 0; i < oshob_info->osnib_size; i++)
		check += readb(osnibr_addr + i);
	if (check) {
		pr_err("WARNING!!! osnib chksum verification faild, reset all osnib data!\n");
		struct_offs = offsetof(struct scu_ipc_oshob, osnibw_ptr) +
				    oshob_info->offs_add;
		osnibw_ptr = readl(oshob_addr + struct_offs);
		osnibw_addr = ioremap_nocache(
					osnibw_ptr, oshob_info->osnib_size);
		if (osnibw_addr) {
			for (i = 0; i < oshob_info->osnib_size; i++)
				writeb(0, osnibw_addr + i);
			rpmsg_send_raw_command(ipcutil_instance,
				IPCMSG_WRITE_OSNIB, 0,
				NULL, NULL, 0, 0,
				0xFFFFFFFF, 0);
			iounmap(osnibw_addr);
		}
	}

	ptr = data;
	for (i = 0; i < len; i++) {
		*ptr = readb(osnibr_addr + offset + i);
		pr_debug("addr(remapped)=%8x, offset=%2x, value=%2x\n",
			(u32)(osnibr_addr+offset+i), offset+i, *ptr);
		ptr++;
	}

	iounmap(oshob_addr);
exit:
	return ret;
}

/* This function is used for the default OSNIB. */
int intel_scu_ipc_write_osnib(u8 *data, int len, int offset)
{
	int i;
	int ret = 0;
	u32 osnibw_ptr;
	u8 osnib_data[oshob_info->osnib_size];
	u8 check = 0, chksum = 0;
	u16 struct_offs;
	void __iomem *oshob_addr, *osnibw_addr, *osnibr_addr;

	pr_debug("OSHOB base addr value is 0x%8x\n", oshob_info->oshob_base);

	rpmsg_global_lock();

	oshob_addr = ioremap_nocache(oshob_info->oshob_base,
				     oshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	/*Dump osnib data for generate chksum */
	struct_offs = offsetof(struct scu_ipc_oshob, osnibr) +
			    oshob_info->offs_add;
	osnibr_addr = oshob_addr + struct_offs;

	pr_debug("OSNIB read addr (remapped) in OSHOB at %x\n",
						(unsigned int)osnibr_addr);

	for (i = 0; i < oshob_info->osnib_size; i++) {
		osnib_data[i] = readb(osnibr_addr + i);
		check += osnib_data[i];
	}
	memcpy(osnib_data + offset, data, len);

	if (check) {
		pr_err("WARNING!!! OSNIB data chksum verification FAILED!\n");
	} else {
		/* generate chksum */
		for (i = 0; i < oshob_info->osnib_size - 1; i++)
			chksum += osnib_data[i];
		osnib_data[oshob_info->osnib_size - 1] = ~chksum + 1;
	}

	struct_offs = offsetof(struct scu_ipc_oshob, osnibw_ptr) +
			    oshob_info->offs_add;
	osnibw_ptr = readl(oshob_addr + struct_offs);
	if (osnibw_ptr == 0) { /* workaround here for BZ 2914 */
		osnibw_ptr = 0xFFFF3400;
		pr_err("ERR: osnibw ptr from oshob is 0, manually set it here\n");
	}

	pr_debug("POSNIB write address: %x\n", osnibw_ptr);

	osnibw_addr = ioremap_nocache(osnibw_ptr, oshob_info->osnib_size);
	if (!osnibw_addr) {
		pr_err("ioremap failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	for (i = 0; i < oshob_info->osnib_size; i++)
		writeb(*(osnib_data + i), (osnibw_addr + i));

	ret = rpmsg_send_raw_command(ipcutil_instance,
			IPCMSG_WRITE_OSNIB, 0,
			NULL, NULL, 0, 0,
			0xFFFFFFFF, 0);
	if (ret < 0)
		pr_err("ipc_write_osnib failed!!\n");

	iounmap(osnibw_addr);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	rpmsg_global_unlock();

	return ret;
}

/* This function is used for the extended OSHOB/OSNIB. */
int intel_scu_ipc_read_osnib_extend(u8 *data, int len, int offset)
{
	int i, ret = 0;
	u8 *ptr, check = 0;
	void __iomem *oshob_addr, *osnibr_addr, *osnibw_addr;
	u32 sptr_dw_mask;

	oshob_addr = ioremap_nocache(oshob_info->oshob_base,
				     oshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_read_osnib_extend: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	pr_debug(
		"ipc_read_osnib_extend: remap OSNIB addr=0x%x size %d\n",
		oshob_info->osnibr_ptr, oshob_info->osnib_size);

	osnibr_addr = ioremap_nocache(oshob_info->osnibr_ptr,
				      oshob_info->osnib_size);

	if (!osnibr_addr) {
		pr_err("ipc_read_osnib_extend: ioremap of osnib failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	/* Make a chksum verification for osnib */
	for (i = 0; i < oshob_info->osnib_size; i++)
		check += readb(osnibr_addr + i);

	if (check) {
		pr_err("ipc_read_osnib_extend: WARNING!!! osnib chksum verification faild, reset all osnib data!\n");
		pr_debug(
			"ipc_read_osnib_extend: remap osnibw ptr addr=0x%x size %d\n",
			oshob_info->osnibw_ptr, oshob_info->osnib_size);

		osnibw_addr = ioremap_nocache(oshob_info->osnibw_ptr,
					      oshob_info->osnib_size);
		if (!osnibw_addr) {
			pr_err("ipc_read_osnib_extend: cannot remap osnib write ptr\n");
			goto unmap_oshob_addr;
		}

		for (i = 0; i < oshob_info->osnib_size; i++)
			writeb(0, osnibw_addr + i);

		/* Send command. The mask to be written identifies which      */
		/* double words of the OSNIB osnib_size bytes will be written.*/
		/* So the mask is coded on 4 bytes.                           */
		sptr_dw_mask = 0xFFFFFFFF;
		rpmsg_send_raw_command(ipcutil_instance,
			IPCMSG_WRITE_OSNIB_EXTEND,
			0, NULL, NULL, 0, 0, sptr_dw_mask, 0);
		iounmap(osnibw_addr);
	}

	ptr = data;
	pr_debug("ipc_read_osnib_extend: OSNIB content:\n");
	for (i = 0; i < len; i++) {
		*ptr = readb(osnibr_addr + offset + i);
		pr_debug("addr(remapped)=%8x, offset=%2x, value=%2x\n",
			(u32)(osnibr_addr+offset+i), offset+i, *ptr);
		ptr++;
	}

	iounmap(osnibr_addr);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	return ret;
}

/* This function is used for the extended OSHOB/OSNIB. */
int intel_scu_ipc_write_osnib_extend(u8 *data, int len, int offset)
{
	int i;
	int ret = 0;
	u8 *posnib_data, *ptr;
	u8 check = 0, chksum = 0;
	void __iomem *oshob_addr, *osnibw_addr, *osnibr_addr;
	u32 sptr_dw_mask;

	rpmsg_global_lock();

	pr_debug(
		"ipc_write_osnib_extend: remap OSHOB addr 0x%8x size %d\n",
		oshob_info->oshob_base, oshob_info->oshob_size);

	oshob_addr = ioremap_nocache(oshob_info->oshob_base,
				     oshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_write_osnib_extend: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	osnibr_addr = ioremap_nocache(oshob_info->osnibr_ptr,
				      oshob_info->osnib_size);

	if (!osnibr_addr) {
		pr_err("ipc_write_osnib_extend: ioremap of osnib failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	/* Dump osnib data for generate chksum */
	posnib_data = kzalloc(oshob_info->osnib_size, GFP_KERNEL);

	if (posnib_data == NULL) {
		pr_err("ipc_write_osnib_extend: The buffer for getting OSNIB is NULL\n");
		ret = -EFAULT;
		iounmap(osnibr_addr);
		goto unmap_oshob_addr;
	}

	ptr = posnib_data;
	for (i = 0; i < oshob_info->osnib_size; i++) {
		*ptr = readb(osnibr_addr + i);
		check += *ptr;
		ptr++;
	}

	memcpy(posnib_data + offset, data, len);

	if (check) {
		pr_err("ipc_write_osnib_extend: WARNING!!! OSNIB data chksum verification FAILED!\n");
	} else {
		/* generate chksum.  */
		pr_debug("ipc_write_osnib_extend: generating checksum\n");
		for (i = 0; i < oshob_info->osnib_size - 1; i++)
			chksum += *(posnib_data + i);
		/* Fill checksum at the CHECKSUM offset place in OSNIB. */
		*(posnib_data +
		    offsetof(struct scu_ipc_osnib, checksum)) = ~chksum + 1;
	}

	pr_debug(
		"ipc_write_osnib_extend: remap osnibw ptr addr=0x%x size %d\n",
		oshob_info->osnibw_ptr, oshob_info->osnib_size);

	osnibw_addr = ioremap_nocache(oshob_info->osnibw_ptr,
				      oshob_info->osnib_size);
	if (!osnibw_addr) {
		pr_err("scu_ipc_write_osnib_extend: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit_osnib;
	}

	for (i = 0; i < oshob_info->osnib_size; i++)
		writeb(*(posnib_data + i), (osnibw_addr + i));

	/* Send command. The mask to be written identifies which            */
	/* double words of the OSNIB osnib_size bytes will be written.*/
	/* So the mask is coded on 4 bytes.                                 */
	sptr_dw_mask = 0xFFFFFFFF;
	ret = rpmsg_send_raw_command(ipcutil_instance,
			IPCMSG_WRITE_OSNIB_EXTEND, 0, NULL, NULL,
			0, 0, sptr_dw_mask, 0);
	if (ret < 0)
		pr_err("scu_ipc_write_osnib_extend: ipc_write_osnib failed!!\n");

	iounmap(osnibw_addr);

exit_osnib:
	iounmap(osnibr_addr);

	kfree(posnib_data);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	rpmsg_global_unlock();

	return ret;
}

/*
 * This writes the reboot reason in the OSNIB (factor and avoid any overlap)
 */
int intel_scu_ipc_write_osnib_rr(u8 rr)
{
	pr_info("intel_scu_ipc_write_osnib_rr: reboot reason %x\n", rr);

	return oshob_info->scu_ipc_write_osnib(
			&rr,
			1,
			offsetof(struct scu_ipc_osnib, target_mode));
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib_rr);

/*
 * This reads the reboot reason from the OSNIB (factor)
 */
int intel_scu_ipc_read_osnib_rr(u8 *rr)
{
	pr_debug("intel_scu_ipc_read_osnib_rr: read reboot reason\n");
	return oshob_info->scu_ipc_read_osnib(
			rr,
			1,
			offsetof(struct scu_ipc_osnib, target_mode));
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_read_osnib_rr);


int intel_scu_ipc_read_oshob_extend_param(void __iomem *poshob_addr)
{
	u16 struct_offs;

	oshob_info->oshob_size = readw(
			    poshob_addr +
			    offsetof(struct scu_ipc_oshob_extend, oshob_size));

	/* Get version. */
	oshob_info->oshob_majrev = readb(
			    poshob_addr +
			    offsetof(struct scu_ipc_oshob_extend, rev_major));
	oshob_info->oshob_minrev = readb(
			    poshob_addr +
			    offsetof(struct scu_ipc_oshob_extend, rev_minor));

	/* Get defined OSNIB space size. */
	oshob_info->osnib_size = readw(
			    poshob_addr +
			    offsetof(struct scu_ipc_oshob_extend, intel_size));

	if (oshob_info->osnib_size == 0) {
		pr_err("ipc_read_oshob_extend_param: OSNIB size is null!\n");
		return -EFAULT;
	}

	/* Get defined OEM space size. */
	oshob_info->oemnib_size = readw(
			    poshob_addr +
			    offsetof(struct scu_ipc_oshob_extend, oem_size));

	if (oshob_info->oemnib_size == 0) {
		pr_err("ipc_read_oshob_extend_param: OEMNIB size is null!\n");
		return -EFAULT;
	}

	/* Set SCU and IA trace buffers */
	if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
		intel_scu_ipc_read_oshob(
			    (u8 *)(oshob_info->scu_trace),
			    OSHOB_SCU_BUFFER_SIZE_BYTES,
			    offsetof(struct scu_ipc_oshob_extend, scutxl_ptr));
	} else
		intel_scu_ipc_read_oshob(
			    (u8 *)(oshob_info->scu_trace),
			    4,
			    offsetof(struct scu_ipc_oshob_extend, scutxl_ptr));

	struct_offs = offsetof(struct scu_ipc_oshob_extend, iatxl) +
			    oshob_info->offs_add;
	oshob_info->ia_trace = readl(poshob_addr + struct_offs);

	/* Set pointers */
	struct_offs = offsetof(struct scu_ipc_oshob_extend, r_intel_ptr) +
			    oshob_info->offs_add;
	oshob_info->osnibr_ptr = readl(poshob_addr + struct_offs);

	if (!oshob_info->osnibr_ptr) {
		pr_err("ipc_read_oshob_extend_param: R_INTEL_POINTER is NULL!\n");
		return -ENOMEM;
	}

	struct_offs = offsetof(struct scu_ipc_oshob_extend, w_intel_ptr) +
			    oshob_info->offs_add;
	oshob_info->osnibw_ptr = readl(poshob_addr + struct_offs);

	if (oshob_info->osnibw_ptr == 0) {
		/* workaround here for BZ 2914 */
		oshob_info->osnibw_ptr = 0xFFFF3400;
		pr_err(
		    "ipc_read_oshob_extend_param: ERR: osnibw from oshob is 0, manually set it here\n");
	}

	struct_offs = offsetof(struct scu_ipc_oshob_extend, r_oem_ptr) +
			    oshob_info->offs_add;
	oshob_info->oemnibr_ptr = readl(poshob_addr + struct_offs);

	if (!oshob_info->oemnibr_ptr) {
		pr_err("ipc_read_oshob_extend_param: R_OEM_POINTER is NULL!\n");
		return -ENOMEM;
	}

	struct_offs = offsetof(struct scu_ipc_oshob_extend, w_oem_ptr) +
			    oshob_info->offs_add;
	oshob_info->oemnibw_ptr = readl(poshob_addr + struct_offs);

	if (!oshob_info->oemnibw_ptr) {
		pr_err("ipc_read_oshob_extend_param: W_OEM_POINTER is NULL!\n");
		return -ENOMEM;
	}

	oshob_info->scu_ipc_write_osnib =
					&intel_scu_ipc_write_osnib_extend;
	oshob_info->scu_ipc_read_osnib =
					&intel_scu_ipc_read_osnib_extend;

	pr_info(
		"Using EXTENDED OSHOB structure size = %d bytes\n",
		oshob_info->oshob_size);
	pr_info(
		"OSNIB size = %d bytes OEMNIB size = %d bytes\n",
		oshob_info->osnib_size, oshob_info->oemnib_size);

	if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_CLOVERVIEW) {
		if ((oshob_info->oshob_majrev >= 1) &&
		    (oshob_info->oshob_minrev >= 1)) {
			/* CLVP and correct version of the oshob. */
			oshob_info->scu_trace_buf =
				readl(poshob_addr +
				      offsetof(struct scu_ipc_oshob_extend,
					       sculogbufferaddr));
			oshob_info->scu_trace_size =
				readl(poshob_addr +
				      offsetof(struct scu_ipc_oshob_extend,
					       sculogbuffersize));
		}
	}
	return 0;
}


int intel_scu_ipc_read_oshob_def_param(void __iomem *poshob_addr)
{
	u16 struct_offs;
	int ret = 0;

	oshob_info->oshob_majrev = OSHOB_REV_MAJ_DEFAULT;
	oshob_info->oshob_minrev = OSHOB_REV_MIN_DEFAULT;
	oshob_info->oshob_size = OSHOB_SIZE;
	oshob_info->osnib_size = OSNIB_SIZE;
	oshob_info->oemnib_size = 0;

	/* Set SCU and IA trace buffers */
	if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
		ret = intel_scu_ipc_read_oshob(
			    (u8 *)(oshob_info->scu_trace),
			    OSHOB_SCU_BUFFER_SIZE_BYTES,
			    offsetof(struct scu_ipc_oshob_extend, scutxl_ptr));
	} else
		ret = intel_scu_ipc_read_oshob(
			    (u8 *)(oshob_info->scu_trace),
			    4,
			    offsetof(struct scu_ipc_oshob_extend, scutxl_ptr));

	if (ret != 0) {
		pr_err("Cannot get scutxl data from OSHOB\n");
		return ret;
	}

	struct_offs = offsetof(struct scu_ipc_oshob, iatxl) +
			    oshob_info->offs_add;
	oshob_info->ia_trace = readl(poshob_addr + struct_offs);

	oshob_info->scu_ipc_write_osnib =
					&intel_scu_ipc_write_osnib;
	oshob_info->scu_ipc_read_osnib =
					&intel_scu_ipc_read_osnib;

	struct_offs = offsetof(struct scu_ipc_oshob, osnibr) +
			    oshob_info->offs_add;
	oshob_info->osnibr_ptr = (unsigned long)(poshob_addr + struct_offs);

	pr_info("Using DEFAULT OSHOB structure size = %d bytes\n",
					oshob_info->oshob_size);

	pr_debug("Using DEFAULT OSHOB structure OSNIB read ptr %x\n",
		oshob_info->osnibr_ptr);

	return ret;
}

int intel_scu_ipc_read_oshob_info(void)
{
	int i, ret = 0;
	u32 oshob_base = 0;
	void __iomem *oshob_addr;
	unsigned char oshob_magic[4];

	ret = rpmsg_send_command(ipcutil_instance,
		IPCMSG_GET_HOBADDR, 0, NULL, &oshob_base, 0, 1);

	if (ret < 0) {
		pr_err("ipc_read_oshob cmd failed!!\n");
		goto exit;
	}

	/* At this stage, we still don't know which OSHOB type (default or  */
	/* extended) can be used, and the size of resource to be remapped   */
	/* depends on the type of OSHOB structure to be used.               */
	/* So just remap the minimum size to get the needed bytes of the    */
	/* OSHOB zone.                                                      */
	oshob_addr = ioremap_nocache(oshob_base, OSHOB_EXTEND_DESC_SIZE);

	if (!oshob_addr) {
		pr_err("oshob addr ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	pr_debug("OSHOB addr 0x%8x remapped to addr 0x%8x\n",
		oshob_base, (u32)oshob_addr);

	oshob_info->oshob_base = oshob_base;

	oshob_info->platform_type = intel_mid_identify_cpu();

	/*
	 * Buffer is allocated using kmalloc. Memory is not initialized and
	 * these fields are not updated in all the branches.
	 */
	oshob_info->scu_trace_buf = 0;
	oshob_info->scu_trace_size = 0;

	if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
		pr_info("(oshob) identified platform = INTEL_MID_CPU_CHIP_TANGIER\n");
		oshob_info->offs_add = OSHOB_SCU_BUFFER_SIZE * 3;
	} else
		oshob_info->offs_add = 0;

	pr_info("(oshob) additional offset = 0x%x\n", oshob_info->offs_add);

	/* Extract magic number that will help identifying the good OSHOB  */
	/* that is going to be used.                                       */
	for (i = 0; i < OSHOB_HEADER_MAGIC_SIZE; i = i+1)
		oshob_magic[i] = readb(oshob_addr + i);

	if (strncmp(oshob_magic, OSHOB_MAGIC_NUMBER,
		    OSHOB_HEADER_MAGIC_SIZE) == 0) {
		if (intel_scu_ipc_read_oshob_extend_param(oshob_addr) != 0) {
			ret = -EFAULT;
			goto unmap_oshob;
		}

		if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
			scu_ipc_oshob_extend_struct.scutxl_ptr = scutxl_ext;
			pr_info("(extend oshob) SCU buffer size is %d bytes\n",
				OSHOB_SCU_BUFFER_SIZE_BYTES);
		} else {
			scu_ipc_oshob_extend_struct.scutxl_ptr = &scutxl_base;
			pr_info("(extend oshob) SCU buffer size is %d bytes\n",
				OSHOB_SCU_BUFFER_SIZE);
		}
	} else {
		ret = intel_scu_ipc_read_oshob_def_param(oshob_addr);

		if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
			scu_ipc_oshob_default.scutxl_ptr = scutxl_ext;
			pr_info("(default oshob) SCU buffer size is %d bytes\n",
				OSHOB_SCU_BUFFER_SIZE_BYTES);
		} else {
			scu_ipc_oshob_default.scutxl_ptr = &scutxl_base;
			pr_info("(default oshob) SCU buffer size is %d bytes\n",
				OSHOB_SCU_BUFFER_SIZE);
		}
	}

unmap_oshob:
	iounmap(oshob_addr);

exit:
	return ret;
}

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

	rpmsg_global_lock();

	pr_debug("ipc_write_oemnib: remap OSHOB addr 0x%8x size %d\n",
		oshob_info->oshob_base, oshob_info->oshob_size);

	oshob_addr = ioremap_nocache(oshob_info->oshob_base,
				     oshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_write_oemnib: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	if ((len == 0) || (len > oshob_info->oemnib_size)) {
		pr_err(
			"ipc_write_oemnib: bad OEMNIB data length (%d) to write (max=%d bytes)\n",
			    len, oshob_info->oemnib_size);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	/* offset shall start at 0 from the OEMNIB base address and shall */
	/* not exceed the OEMNIB allowed size.                            */
	if ((offset < 0) || (offset >= oshob_info->oemnib_size) ||
	    (len + offset > oshob_info->oemnib_size)) {
		pr_err(
			"ipc_write_oemnib: Bad OEMNIB data offset/len for writing (offset=%d , len=%d)\n",
			offset, len);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	pr_debug("ipc_write_oemnib: POEMNIB remap oemnibw ptr 0x%x size %d\n",
		oshob_info->oemnibw_ptr, oshob_info->oemnib_size);

	oemnibw_addr = ioremap_nocache(oshob_info->oemnibw_ptr,
				       oshob_info->oemnib_size);
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
	ret = rpmsg_send_raw_command(ipcutil_instance,
			IPCMSG_WRITE_OEMNIB, 0, NULL, NULL,
			0, 0, sptr_dw_mask, 0);
	if (ret < 0)
		pr_err("ipc_write_oemnib: ipc_write_osnib failed!!\n");

	iounmap(oemnibw_addr);

unmap_oshob_addr:
	iounmap(oshob_addr);
exit:
	rpmsg_global_unlock();

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

	pr_debug("ipc_read_oemnib: remap OSHOB base addr 0x%x size %d\n",
		oshob_info->oshob_base, oshob_info->oshob_size);

	oshob_addr = ioremap_nocache(oshob_info->oshob_base,
				     oshob_info->oshob_size);
	if (!oshob_addr) {
		pr_err("ipc_read_oemnib: ioremap failed!\n");
		ret = -ENOMEM;
		goto exit;
	}

	if ((len == 0) || (len > oshob_info->oemnib_size)) {
		pr_err("ipc_read_oemnib: Bad OEMNIB data length (%d) to be read (max=%d bytes)\n",
			    len, oshob_info->oemnib_size);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	/* offset shall start at 0 from the OEMNIB base address and shall */
	/* not exceed the OEMNIB allowed size.                            */
	if ((offset < 0) || (offset >= oshob_info->oemnib_size) ||
	    (len + offset > oshob_info->oemnib_size)) {
		pr_err(
		"ipc_read_oemnib: Bad OEMNIB data offset/len to read (offset=%d ,len=%d)\n",
		offset, len);
		ret = -EINVAL;
		goto unmap_oshob_addr;
	}

	pr_debug("ipc_read_oemnib: POEMNIB remap oemnibr ptr 0x%x size %d\n",
		oshob_info->oemnibr_ptr, oshob_info->oemnib_size);

	oemnibr_addr = ioremap_nocache(oshob_info->oemnibr_ptr,
				       oshob_info->oemnib_size);

	if (!oemnibr_addr) {
		pr_err("ipc_read_oemnib: ioremap of oemnib failed!\n");
		ret = -ENOMEM;
		goto unmap_oshob_addr;
	}

	ptr = oemnib;
	pr_debug("ipc_read_oemnib: OEMNIB content:\n");
	for (i = 0; i < len; i++) {
		*ptr = readb(oemnibr_addr + offset + i);
		pr_debug("addr(remapped)=%8x, offset=%2x, value=%2x\n",
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
	u16 struct_offs;

	pr_debug("intel_scu_ipc_read_oshob_it_tree: read IT tree\n");

	if ((oshob_info->oshob_majrev == OSHOB_REV_MAJ_DEFAULT) &&
	    (oshob_info->oshob_minrev == OSHOB_REV_MIN_DEFAULT)) {
		struct_offs = offsetof(struct scu_ipc_oshob, pmit) +
				    oshob_info->offs_add;
		return intel_scu_ipc_read_oshob(
					(u8 *) ptr,
					4,
					struct_offs);
	} else {
		struct_offs = offsetof(struct scu_ipc_oshob_extend, pmit) +
				    oshob_info->offs_add;
		return intel_scu_ipc_read_oshob(
				(u8 *) ptr,
				4,
				struct_offs);
	}
}
#endif

/*
 * This reads the RESETIRQ1 from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_resetirq1(u8 *rirq1)
{
	pr_debug("intel_scu_ipc_read_osnib_resetirq1: read RESETIRQ1\n");

	return oshob_info->scu_ipc_read_osnib(
			rirq1,
			1,
			offsetof(struct scu_ipc_osnib, resetirq1));
}
#endif

/*
 * This reads the RESETIRQ2 from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_resetirq2(u8 *rirq2)
{
	pr_debug("intel_scu_ipc_read_osnib_resetirq2: read RESETIRQ2\n");

	return oshob_info->scu_ipc_read_osnib(
			rirq2,
			1,
			offsetof(struct scu_ipc_osnib, resetirq2));
}
#endif

/*
 * This reads the WD from the OSNIB
 */
int intel_scu_ipc_read_osnib_wd(u8 *wd)
{
	pr_debug("intel_scu_ipc_read_osnib_wd: read WATCHDOG\n");

	return oshob_info->scu_ipc_read_osnib(
			wd,
			1,
			offsetof(struct scu_ipc_osnib, wd_count));
}

/*
 * This writes the WD in the OSNIB
 */
int intel_scu_ipc_write_osnib_wd(u8 *wd)
{
	pr_info("intel_scu_ipc_write_osnib_wd: write WATCHDOG %x\n", *wd);

	return oshob_info->scu_ipc_write_osnib(
			wd,
			1,
			offsetof(struct scu_ipc_osnib, wd_count));
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_write_osnib_wd);

/*
 * Get SCU trace buffer physical address if available
 */
u32 intel_scu_ipc_get_scu_trace_buffer(void)
{
	if (oshob_info == NULL)
		return 0;
	return oshob_info->scu_trace_buf;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_get_scu_trace_buffer);

/*
 * Get SCU trace buffer size
 */
u32 intel_scu_ipc_get_scu_trace_buffer_size(void)
{
	if (oshob_info == NULL)
		return 0;
	return oshob_info->scu_trace_size;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_get_scu_trace_buffer_size);

/*
 * Get SCU fabric error buffer1 offset
 */
u32 intel_scu_ipc_get_fabricerror_buf1_offset(void)
{
	return offsetof(struct scu_ipc_oshob_extend,
			fabricerrlog1);
}

/*
 * Get SCU fabric error buffer2 offset
 */
u32 intel_scu_ipc_get_fabricerror_buf2_offset(void)
{
	return offsetof(struct scu_ipc_oshob_extend,
			fabricerrlog2);
}


/*
 * This reads the ALARM from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_alarm(u8 *alarm)
{
	pr_debug("intel_scu_ipc_read_osnib_alarm: read ALARM\n");

	return oshob_info->scu_ipc_read_osnib(
			alarm,
			1,
			offsetof(struct scu_ipc_osnib, alarm));
}
#endif

/*
 * This reads the WAKESRC from the OSNIB
 */
#ifdef DUMP_OSNIB
static int intel_scu_ipc_read_osnib_wakesrc(u8 *wksrc)
{
	pr_debug("intel_scu_ipc_read_osnib_wakesrc: read WAKESRC\n");

	return oshob_info->scu_ipc_read_osnib(
			wksrc,
			1,
			offsetof(struct scu_ipc_osnib, wakesrc));
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
	if ((oshob_info->oshob_majrev == OSHOB_REV_MAJ_DEFAULT) &&
	     (oshob_info->oshob_minrev == OSHOB_REV_MIN_DEFAULT)) {
		seq_printf(m, "DEFAULT OSHOB\n");
		seq_printf(m, "OSHOB size : %d\n", oshob_info->oshob_size);
		if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
			seq_printf(m, "SCU trace : ");

			for (i = 0; i < OSHOB_SCU_BUFFER_SIZE; i++)
				seq_printf(m, "%x ", oshob_info->scu_trace[i]);

			seq_printf(m, "\n");
		} else
			seq_printf(m, "SCU trace : %x\n",
					oshob_info->scu_trace[0]);

		seq_printf(m, "IA trace  : %x\n", oshob_info->ia_trace);
	} else {
		seq_printf(m, "EXTENDED OSHOB v%d.%d\n",
						oshob_info->oshob_majrev,
						oshob_info->oshob_minrev);
		seq_printf(m, "OSHOB size : %d\n\n", oshob_info->oshob_size);
		if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
			seq_printf(m, "SCU trace : ");

			for (i = 0; i < OSHOB_SCU_BUFFER_SIZE; i++)
				seq_printf(m, "%x ", oshob_info->scu_trace[i]);

			seq_printf(m, "\n");
		} else
			seq_printf(m, "SCU trace : %x\n",
					oshob_info->scu_trace[0]);

		seq_printf(m, "IA trace  : %x\n\n", oshob_info->ia_trace);

		seq_printf(m, "OSNIB size : %d\n", oshob_info->osnib_size);
		seq_printf(m, "OSNIB  read address  : %x\n",
						    oshob_info->osnibr_ptr);
		seq_printf(m, "OSNIB  write address : %x\n",
						oshob_info->osnibw_ptr);
		/* Dump OSNIB */
		osnib = ioremap_nocache(oshob_info->osnibr_ptr,
						oshob_info->osnib_size);
		if (!osnib) {
			pr_err("Cannot remap OSNIB\n");
			ret = -ENOMEM;
			return ret;
		}

		i = 0;
		count = 0; /* used for fancy presentation */
		while (i < oshob_info->osnib_size) {
			if (count%4 == 0)
				seq_printf(m, "\nOSNIB[%08x] ",
						oshob_info->osnibr_ptr+i);

			value = readl(osnib+i);
			seq_printf(m, "%08x ", value);
			i += 4;
			count++;
		}
		seq_printf(m, "\n\n");
		iounmap(osnib);

		seq_printf(m, "OEMNIB size : %d\n",
						oshob_info->oemnib_size);
		seq_printf(m, "OEMNIB read address  : %x\n",
						oshob_info->oemnibr_ptr);
		seq_printf(m, "OEMNIB write address : %x\n",
						oshob_info->oemnibw_ptr);
		seq_printf(m, "\n\n");
	}
	return 0;
}

static int intel_scu_ipc_oemnib_stat(struct seq_file *m, void *unused)
{
	void __iomem *oemnib;
	int i, count;
	u32 value;

	/* Dump OEMNIB */
	oemnib = ioremap_nocache(oshob_info->oemnibr_ptr,
				oshob_info->oemnib_size);

	if (!oemnib) {
		pr_err("Cannot remap OEMNIB\n");
		return -ENOMEM;
	}

	i = 0;
	count = 0; /* used for fancy presentation */
	while (i < oshob_info->oemnib_size) {
		if (count%4 == 0)
			seq_printf(m, "\nOEMNIB[%08x] ",
				    oshob_info->oemnibr_ptr+i);

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

	if ((oshob_info->oshob_majrev == OSHOB_REV_MAJ_DEFAULT) &&
	    (oshob_info->oshob_minrev == OSHOB_REV_MIN_DEFAULT)) {
		/* OEMNIB only usable with extended OSHOB structure. */
		pr_err(
		"Write OEMNIB: OEMNIB only usable with extended OSHOB structure.\n");
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

static int oshob_init(void)
{
	int ret, i;
	u16 struct_offs;

#ifdef DUMP_OSNIB
	u8 rr, resetirq1, resetirq2, wd, alarm, wakesrc, *ptr;
	u32 pmit, scu_trace[OSHOB_SCU_BUFFER_SIZE], ia_trace;
#endif

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

	pr_debug("PMIT addr 0x%8x remapped to 0x%8x\n", pmit, (u32)ptr);

	resetirq1 = readb(ptr);
	resetirq2 = readb(ptr+1);
	pr_warn("[BOOT] RESETIRQ1=0x%02x RESETIRQ2=0x%02x (interrupt tree)\n",
		resetirq1, resetirq2);
	iounmap(ptr);

	/* Dumping OSHOB content */
	if ((oshob_info->oshob_majrev == OSHOB_REV_MAJ_DEFAULT) &&
	    (oshob_info->oshob_minrev == OSHOB_REV_MIN_DEFAULT)) {
		/* Use default OSHOB here. */
		if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
			ret = intel_scu_ipc_read_oshob(
			    (u8 *)(scu_trace),
			    OSHOB_SCU_BUFFER_SIZE_BYTES,
			    offsetof(struct scu_ipc_oshob, scutxl_ptr));
		} else
			ret = intel_scu_ipc_read_oshob(
			    (u8 *)(scu_trace),
			    4,
			    offsetof(struct scu_ipc_oshob, scutxl_ptr));

		if (ret != 0) {
			pr_err("Cannot read SCU data\n");
			goto exit;
		}

		struct_offs = offsetof(struct scu_ipc_oshob, iatxl) +
				oshob_info->offs_add;
		ret = intel_scu_ipc_read_oshob(
			    (u8 *)(&ia_trace),
			    4,
			    struct_offs);

		if (ret != 0) {
			pr_err("Cannot read IA data\n");
			goto exit;
		}
	    } else {
		/* Use extended OSHOB here. */

		if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
			ret = intel_scu_ipc_read_oshob(
				(u8 *)(scu_trace),
				OSHOB_SCU_BUFFER_SIZE_BYTES,
				offsetof(struct scu_ipc_oshob_extend,
					 scutxl_ptr));
		} else
			ret = intel_scu_ipc_read_oshob(
				(u8 *)(scu_trace),
				4,
				offsetof(struct scu_ipc_oshob_extend,
					 scutxl_ptr));

		if (ret != 0) {
			pr_err("Cannot read SCU data\n");
			goto exit;
		}

		struct_offs = offsetof(struct scu_ipc_oshob_extend, iatxl) +
				    oshob_info->offs_add;
		ret = intel_scu_ipc_read_oshob(
				(u8 *)(&ia_trace),
				4,
				struct_offs);

		if (ret != 0) {
			pr_err("Cannot read IA data\n");
			goto exit;
		}
	}

	if (oshob_info->platform_type == INTEL_MID_CPU_CHIP_TANGIER) {
		for (i = 0; i < OSHOB_SCU_BUFFER_SIZE; i++)
			pr_warn("BOOT] SCU_TR[%d]=0x%08x\n", i, scu_trace[i]);
	} else
		pr_warn("[BOOT] SCU_TR=0x%08x (oshob)\n", scu_trace[0]);

	pr_warn("[BOOT] IA_TR=0x%08x (oshob)\n", ia_trace);

	/* Dumping OSNIB content */
	ret = 0;
	ret |= intel_scu_ipc_read_osnib_rr(&rr);
	ret |= intel_scu_ipc_read_osnib_resetirq1(&resetirq1);
	ret |= intel_scu_ipc_read_osnib_resetirq2(&resetirq2);
	ret |= intel_scu_ipc_read_osnib_wd(&wd);
	ret |= intel_scu_ipc_read_osnib_alarm(&alarm);
	ret |= intel_scu_ipc_read_osnib_wakesrc(&wakesrc);

	if (ret) {
		pr_err("Cannot read OSNIB content\n");
		goto exit;
	}

	pr_warn("[BOOT] RR=0x%02x WD=0x%02x ALARM=0x%02x (osnib)\n",
		rr, wd, alarm);
	pr_warn("[BOOT] WAKESRC=0x%02x RESETIRQ1=0x%02x RESETIRQ2=0x%02x (osnib)\n",
		wakesrc, resetirq1, resetirq2);
#endif /* DUMP_OSNIB */

#ifdef CONFIG_DEBUG_FS
	if (oshob_info->oshob_majrev != OSHOB_REV_MAJ_DEFAULT) {
		/* OEMNIB only usable with extended OSHOB structure. */
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
	return ret;
}

static int ipcutil_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	oshob_info = kmalloc(sizeof(struct scu_ipc_oshob_info), GFP_KERNEL);
	if (oshob_info == NULL) {
		pr_err(
		"Cannot init ipc module: oshob info struct not allocated\n");
		return -ENOMEM;
	}

	if (rpdev == NULL) {
		pr_err("ipcutil rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed ipcutil rpmsg device\n");

	/* Allocate rpmsg instance for mip*/
	ret = alloc_rpmsg_instance(rpdev, &ipcutil_instance);
	if (!ipcutil_instance) {
		dev_err(&rpdev->dev, "kzalloc ipcutil instance failed\n");
		goto out;
	}

	/* Initialize rpmsg instance */
	init_rpmsg_instance(ipcutil_instance);

	ret = oshob_init();
	if (ret)
		goto misc_err;

	ret = misc_register(&scu_ipcutil);
	if (ret) {
		pr_err("misc register failed\n");
		goto misc_err;
	}

	return ret;

misc_err:
	free_rpmsg_instance(rpdev, &ipcutil_instance);
out:
	kfree(oshob_info);
	return ret;
}

static void ipcutil_rpmsg_remove(struct rpmsg_channel *rpdev)
{
#ifdef CONFIG_DEBUG_FS
	if (oshob_info->oshob_majrev != OSHOB_REV_MAJ_DEFAULT) {
		/* OEMNIB only usable with extended OSHOB structure. */
		/* unregister from debugfs.                     */
		intel_mid_scu_ipc_oemnib_debugfs_exit();
	}
#endif /* CONFIG_DEBUG_FS */

	kfree(oshob_info);

	/* unregister scu_ipc_ioctl from sysfs. */
	misc_deregister(&scu_ipcutil);
	free_rpmsg_instance(rpdev, &ipcutil_instance);
}

static void ipcutil_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id ipcutil_rpmsg_id_table[] = {
	{ .name	= "rpmsg_ipc_util" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, ipcutil_rpmsg_id_table);

static struct rpmsg_driver ipcutil_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= ipcutil_rpmsg_id_table,
	.probe		= ipcutil_rpmsg_probe,
	.callback	= ipcutil_rpmsg_cb,
	.remove		= ipcutil_rpmsg_remove,
};

static int __init ipcutil_rpmsg_init(void)
{
	return register_rpmsg_driver(&ipcutil_rpmsg);
}

static void __exit ipcutil_rpmsg_exit(void)
{
	unregister_rpmsg_driver(&ipcutil_rpmsg);
}

rootfs_initcall(ipcutil_rpmsg_init);
module_exit(ipcutil_rpmsg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Utility driver for intel scu ipc");
MODULE_AUTHOR("Sreedhara <sreedhara.ds@intel.com>");
