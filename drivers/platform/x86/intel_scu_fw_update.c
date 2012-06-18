/*
 * fw_update.c - Intel SCU Firmware Update Driver
 *
 * Copyright (C) 2012 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ipc_device.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <asm/intel_scu_ipc.h>
#include <linux/intel_mid_pm.h>

/* Medfield firmware update.
 * The flow and communication between IA and SCU has changed for
 * Medfield firmware update. For more details, please refer to
 * Firmware Arch Spec.
 * Below macros and structs apply for medfield firmware update
 */

#define IPC_CMD_FW_UPDATE_GO     0x20FE

#define MAX_FW_CHUNK	(128*1024)
#define SRAM_ADDR	0xFFFC0000
#define MAILBOX_ADDR	0xFFFE0000

#define SCU_FLAG_OFFSET	8
#define IA_FLAG_OFFSET	12

#define MIP_HEADER_OFFSET	0
#define SUCP_OFFSET		0x1D8000
#define VEDFW_OFFSET		0x1A6000

#define DNX_HDR_LEN		24
#define MIN_FUPH_HDR_LEN	32

#define DNX_IMAGE        "DXBL"
#define FUPH_HDR_SIZE    "RUPHS"
#define FUPH		 "RUPH"
#define MIP              "DMIP"
#define LOWER_128K       "LOFW"
#define UPPER_128K       "HIFW"
#define UPDATE_DONE      "HLT$"
#define PSFW1		 "PSFW1"
#define PSFW2		 "PSFW2"
#define SSFW		 "SSFW"
#define SUCP		 "SuCP"
#define VEDFW		 "VEDFW"

#define MAX_LEN_PSFW     7
#define MAX_LEN_SSFW     6
#define MAX_LEN_SUCP     6
#define MAX_LEN_VEDFW    7

#define FUPH_MIP_OFFSET         0x04
#define FUPH_IFWI_OFFSET        0x08
#define FUPH_PSFW1_OFFSET       0x0c
#define FUPH_PSFW2_OFFSET       0x10
#define FUPH_SSFW_OFFSET        0x14
#define FUPH_SUCP_OFFSET        0x18
#define FUPH_VEDFW_OFFSET       0x1c

#define DNX_MAX_SIZE	(128*1024)
#define IFWI_MAX_SIZE	(3*1024*1024)
#define FOTA_MEM_SIZE	(4*1024*1024)

#define DNX_SIZE_OFFSET	0
#define GP_FLAG_OFFSET	4
#define XOR_CHK_OFFSET	20

#define GPF_BIT32	1
#define FUPH_STR	"UPH$"
#define FUPH_STR_LEN	4
#define FUPH_MAX_LEN	36
#define SKIP_BYTES	8

#define IPCMSG_FW_REVISION 0xF4

/* Modified IA-SCU mailbox for medfield firmware update. */
struct ia_scu_mailbox {
	char mail[8];
	u32 scu_flag;
	u32 ia_flag;
};

/* Structure to parse input from firmware-update application. */
struct fw_ud {
	u8 *fw_file_data;
	u32 fsize;
	u8 *dnx_hdr;
	u8 *dnx_file_data;
	u32 dnx_size;
	u32 fuph_hdr_len;
};

struct mfld_fw_update {
	void __iomem *sram;
	void __iomem *mailbox;
	u32 wscu;
	u32 wia;
	char mb_status[8];
};

/* Holds size parameters read from fuph header */
struct fuph_hdr_attrs {
	u32 mip_size;
	u32 ifwi_size;
	u32 psfw1_size;
	u32 psfw2_size;
	u32 ssfw_size;
	u32 sucp_size;
	u32 vedfw_size;
};

enum mailbox_status {
	MB_DONE,
	MB_CONTINUE,
	MB_ERROR
};

/* Misc. firmware components that are part of integrated firmware */
struct misc_fw {
	const char *fw_type;
	u8 str_len;
};

/* lock used to prevent multiple calls to fw update sysfs interface */
static DEFINE_MUTEX(fwud_lock);

static char err_buf[50];
static u8 *pending_data;

struct fw_update_info {
	struct device *dev;
	struct fw_ud *fwud_pending;
};

static struct fw_update_info fui;

static struct misc_fw misc_fw_table[] = {
	{ .fw_type = "PSFW1", .str_len  = MAX_LEN_PSFW },
	{ .fw_type = "SSFW", .str_len  = MAX_LEN_SSFW  },
	{ .fw_type = "PSFW2", .str_len  = MAX_LEN_PSFW  },
	{ .fw_type = "SuCP", .str_len  = MAX_LEN_SUCP  },
	{ .fw_type = "VEDFW", .str_len  = MAX_LEN_VEDFW  }
};

static int alloc_fota_mem_early;

int __init alloc_mem_fota_early_flag(char *p)
{
	alloc_fota_mem_early = 1;
	return 0;
}
early_param("alloc_fota_mem_early", alloc_mem_fota_early_flag);

/*
 * IA will wait in busy-state, and poll mailbox, to check
 * if SCU is done processing.
 * If it has to wait for more than a second, it will exit with
 * error code.
 */
static int busy_wait(struct mfld_fw_update *mfld_fw_upd)
{
	u32 count = 0;
	u32 flag;

	flag = mfld_fw_upd->wscu;

	while (ioread32(mfld_fw_upd->mailbox + SCU_FLAG_OFFSET) != flag
		&& count < 500) {
		/* There are synchronization issues between IA and SCU */
		mb();
		/* FIXME: we must use mdelay currently */
		mdelay(10);
		count++;
	}

	if (ioread32(mfld_fw_upd->mailbox + SCU_FLAG_OFFSET) != flag) {
		dev_err(fui.dev, "IA-waited and quitting\n");
		return -ETIMEDOUT;
	}

	return 0;
}

/* This function will
 * 1)copy firmware chunk from user-space to kernel-space.
 * 2) Copy from kernel-space to shared SRAM.
 * 3) Write to mailbox.
 * 4) And wait for SCU to process that firmware chunk.
 * Returns 0 on success, and < 0 for failure.
 */
static int process_fw_chunk(u8 *fws, u8 *userptr, u32 chunklen,
					struct mfld_fw_update *mfld_fw_upd)
{
	memcpy(fws, userptr, chunklen);

	/* IA copy to sram */
	memcpy_toio(mfld_fw_upd->sram, fws, chunklen);

	/* There are synchronization issues between IA and SCU */
	mb();
	mfld_fw_upd->wia = !(mfld_fw_upd->wia);
	iowrite32(mfld_fw_upd->wia, mfld_fw_upd->mailbox + IA_FLAG_OFFSET);

	mb();
	dev_dbg(fui.dev, "wrote ia_flag=%d\n",
		 ioread32(mfld_fw_upd->mailbox + IA_FLAG_OFFSET));

	mfld_fw_upd->wscu = !mfld_fw_upd->wscu;
	return busy_wait(mfld_fw_upd);
}

/*
 * This function will check mailbox status flag, and return state of mailbox.
 */
static enum mailbox_status check_mb_status(struct mfld_fw_update *mfld_fw_upd)
{

	enum mailbox_status mb_state;

	/* There are synchronization issues between IA and SCU */
	mb();

	memcpy_fromio(mfld_fw_upd->mb_status, mfld_fw_upd->mailbox, 8);

	if (!strncmp(mfld_fw_upd->mb_status, "ER", 2) ||
		!strncmp(mfld_fw_upd->mb_status, "HLT0", 4)) {
		dev_dbg(fui.dev,
			"mailbox error=%s\n", mfld_fw_upd->mb_status);
		return MB_ERROR;
	} else {
		mb_state = (!strncmp(mfld_fw_upd->mb_status, UPDATE_DONE,
				sizeof(UPDATE_DONE))) ? MB_DONE : MB_CONTINUE;
		dev_dbg(fui.dev,
			"mailbox pass=%s, mb_state=%d\n",
			mfld_fw_upd->mb_status, mb_state);
	}

	return mb_state;
}

/* Helper function used to calculate length and offset.  */
int helper_for_calc_offset_length(struct fw_ud *fw_ud_ptr, char *scu_req,
			void **offset, u32 *len, struct fuph_hdr_attrs *fuph,
			const char *fw_type)
{
	unsigned long chunk_no;
	u32 chunk_rem;
	u32 max_chunk_cnt;
	u32 fw_size;
	u32 fw_offset;

	if (!strncmp(fw_type, PSFW1, strlen(PSFW1))) {

		if (strict_strtoul(scu_req + strlen(PSFW1), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->psfw1_size;
		fw_offset = fuph->mip_size + fuph->ifwi_size;
	} else if (!strncmp(fw_type, PSFW2, strlen(PSFW2))) {

		if (strict_strtoul(scu_req + strlen(PSFW2), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->psfw2_size;
		fw_offset = fuph->mip_size + fuph->ifwi_size +
				fuph->psfw1_size + fuph->ssfw_size;
	} else if (!strncmp(fw_type, SSFW, strlen(SSFW))) {

		if (strict_strtoul(scu_req + strlen(SSFW), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->ssfw_size;
		fw_offset = fuph->mip_size + fuph->ifwi_size +
				fuph->psfw1_size;
	} else if (!strncmp(fw_type, SUCP, strlen(SUCP))) {

		if (strict_strtoul(scu_req + strlen(SUCP), 10,
						&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->sucp_size;
		fw_offset = SUCP_OFFSET;
	} else if (!strncmp(fw_type, VEDFW, strlen(VEDFW))) {

		if (strict_strtoul(scu_req + strlen(VEDFW), 10,
				&chunk_no) < 0)
			return -EINVAL;

		fw_size = fuph->vedfw_size;
		fw_offset = VEDFW_OFFSET;
	} else
		return -EINVAL;

	chunk_rem = fw_size % MAX_FW_CHUNK;
	max_chunk_cnt = (fw_size/MAX_FW_CHUNK) + (chunk_rem ? 1 : 0);

	dev_dbg(fui.dev,
		"str=%s,chunk_no=%lx, chunk_rem=%d,max_chunk_cnt=%d\n",
		fw_type, chunk_no, chunk_rem, max_chunk_cnt);

	if ((chunk_no + 1) > max_chunk_cnt)
		return -EINVAL;

	/* Note::Logic below will make sure, that we get right length if input
	 is 128K or multiple. */
	*len = (chunk_no == (max_chunk_cnt - 1)) ?
		(chunk_rem ? chunk_rem : MAX_FW_CHUNK) : MAX_FW_CHUNK;

	*offset = fw_ud_ptr->fw_file_data + fw_offset +
		(fw_size/((max_chunk_cnt - chunk_no)
		* MAX_FW_CHUNK))*MAX_FW_CHUNK;

	return 0;
}

/*
 * This api calculates offset and length depending on type of firmware chunk
 * requested by SCU. Note: Intent is to follow the architecture such that,
 * SCU controls the flow, and IA simply hands out, what is requested by SCU.
 * IA will simply follow SCU's commands, unless SCU requests for something
 * IA cannot give. TODO:That will be a special error case, need to figure out
 * how to handle that.
 */
int calc_offset_and_length(struct fw_ud *fw_ud_ptr, char *scu_req,
			void **offset, u32 *len, struct fuph_hdr_attrs *fuph)
{
	u8 cnt;

	if (!strncmp(DNX_IMAGE, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->dnx_file_data;
		*len = fw_ud_ptr->dnx_size;
		return 0;
	} else if (!strncmp(FUPH, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data + fw_ud_ptr->fsize
				- fw_ud_ptr->fuph_hdr_len;
		*len = fw_ud_ptr->fuph_hdr_len;
		return 0;
	} else if (!strncmp(MIP, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data + MIP_HEADER_OFFSET;
		*len = fuph->mip_size;
		return 0;
	} else if (!strncmp(LOWER_128K, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data + fuph->mip_size;
		*len = MAX_FW_CHUNK;
		return 0;
	} else if (!strncmp(UPPER_128K, scu_req, strlen(scu_req))) {
		*offset = fw_ud_ptr->fw_file_data
				+ fuph->mip_size + MAX_FW_CHUNK;
		*len = MAX_FW_CHUNK;
		return 0;
	} else {
		for (cnt = 0; cnt < ARRAY_SIZE(misc_fw_table) ; cnt++) {

			if (!strncmp(misc_fw_table[cnt].fw_type, scu_req,
					strlen(misc_fw_table[cnt].fw_type))) {

				if (strlen(scu_req) ==
						misc_fw_table[cnt].str_len) {

					if (helper_for_calc_offset_length
						(fw_ud_ptr, scu_req,
						offset, len, fuph,
						misc_fw_table[cnt].fw_type) < 0)
						goto error_case;

					dev_dbg(fui.dev,
					"\nmisc fw type=%s, len=%d,offset=%d",
					misc_fw_table[cnt].fw_type, *len,
					(int)*offset);

					return 0;

				} else
					goto error_case;
			}
		}

	}

	dev_dbg(fui.dev, "Unexpected mailbox request from scu\n");

error_case:
	/* TODO::Need to test this error case..and see how SCU reacts
	* and how IA handles
	* subsequent error response and whether exit is graceful...
	*/

	dev_dbg(fui.dev, "error case,respond back to SCU..\n");
	dev_dbg(fui.dev, "scu_req=%s\n", scu_req);
	*len = 0;
	*offset = 0;

	return -EINVAL;
}

/**
 * intel_scu_ipc_medfw_upgrade - Medfield Firmware update utility
 *
 * The flow and communication between IA and SCU has changed for
 * Medfield firmware update. So we have a different api below
 * to support Medfield firmware update.
 *
 * On success returns 0, for failure , returns < 0.
 */
int intel_scu_ipc_medfw_upgrade(void)
{
	struct fw_ud *fw_ud_param = fui.fwud_pending;
	struct mfld_fw_update	mfld_fw_upd;
	u8 *fw_file_data = NULL;
	u8 *fws = NULL;
	u8 *fuph_start = NULL;
	int ret_val = 0;

	struct fuph_hdr_attrs fuph;
	u32 length = 0;
	void *offset;
	enum mailbox_status mb_state;

	/* set all devices in d0i0 before IFWI upgrade */
	if (unlikely(pmu_set_devices_in_d0i0())) {
		pr_debug("pmu: failed to set all devices in d0i0...\n");
		BUG();
	}

	intel_scu_ipc_lock();
	mfld_fw_upd.wscu = 0;
	mfld_fw_upd.wia = 0;
	memset(mfld_fw_upd.mb_status, 0, sizeof(char) * 8);

	fw_file_data = fw_ud_param->fw_file_data;
	mfld_fw_upd.sram = ioremap_nocache(SRAM_ADDR, MAX_FW_CHUNK);
	if (mfld_fw_upd.sram == NULL) {
		dev_err(fui.dev, "unable to map sram\n");
		ret_val = -ENOMEM;
		goto out_unlock;
	}

	mfld_fw_upd.mailbox = ioremap_nocache(MAILBOX_ADDR,
					sizeof(struct ia_scu_mailbox));

	if (mfld_fw_upd.mailbox == NULL) {
		dev_err(fui.dev, "unable to map the mailbox\n");
		ret_val = -ENOMEM;
		goto unmap_sram;
	}

	/*IA initializes both IAFlag and SCUFlag to zero */
	iowrite32(0, mfld_fw_upd.mailbox + SCU_FLAG_OFFSET);
	iowrite32(0, mfld_fw_upd.mailbox + IA_FLAG_OFFSET);
	memset_io(mfld_fw_upd.mailbox, 0, 8);

	fws = kmalloc(MAX_FW_CHUNK, GFP_KERNEL);
	if (fws == NULL) {
		ret_val = -ENOMEM;
		goto unmap_mb;
	}

	/* fuph header start */
	fuph_start = fw_ud_param->fw_file_data + (fw_ud_param->fsize - 1)
					- (fw_ud_param->fuph_hdr_len - 1);

	/* Convert sizes in DWORDS to number of bytes. */
	fuph.mip_size = (*((u32 *)(fuph_start + FUPH_MIP_OFFSET)))*4;
	fuph.ifwi_size = (*((u32 *)(fuph_start + FUPH_IFWI_OFFSET)))*4;
	fuph.psfw1_size = (*((u32 *)(fuph_start + FUPH_PSFW1_OFFSET)))*4;
	fuph.psfw2_size = (*((u32 *)(fuph_start + FUPH_PSFW2_OFFSET)))*4;
	fuph.ssfw_size = (*((u32 *)(fuph_start + FUPH_SSFW_OFFSET)))*4;
	fuph.sucp_size = (*((u32 *)(fuph_start + FUPH_SUCP_OFFSET)))*4;

	if (fw_ud_param->fuph_hdr_len == (MIN_FUPH_HDR_LEN + 4)) {
		fuph.vedfw_size =
				(*((u32 *)(fuph_start + FUPH_VEDFW_OFFSET)))*4;
	} else
		fuph.vedfw_size = 0;

	dev_dbg(fui.dev,
		"ln=%d, mi=%d, if=%d, ps1=%d, ps2=%d, sfw=%d, sucp=%d, vd=%d\n",
		fw_ud_param->fuph_hdr_len, fuph.mip_size, fuph.ifwi_size,
		fuph.psfw1_size, fuph.psfw2_size, fuph.ssfw_size,
		fuph.sucp_size,	fuph.vedfw_size);

	/* TODO_SK::There is just
	 *  1 write required from IA side for DFU.
	 *  So commenting this-out, until it gets confirmed */
	/*ipc_command(IPC_CMD_FW_UPDATE_READY); */

	/*1. DNX SIZE HEADER   */
	memcpy(fws, fw_ud_param->dnx_hdr, DNX_HDR_LEN);

	memcpy_toio(mfld_fw_upd.sram, fws, DNX_HDR_LEN);

	/* There are synchronization issues between IA and SCU */
	mb();

	/* Write cmd to trigger an interrupt to SCU for firmware update*/
	intel_scu_ipc_send_command(IPC_CMD_FW_UPDATE_GO);

	mfld_fw_upd.wscu = !mfld_fw_upd.wscu;

	if (busy_wait(&mfld_fw_upd) < 0) {
		ret_val = -1;
		goto term;
	}

	/* TODO:Add a count for iteration, based on sizes of security firmware,
	 * so that we determine finite number of iterations to loop thro.
	 * That way at the very least, we can atleast control the number
	 * of iterations, and prevent infinite looping if there are any bugs.
	 * The only catch being for B0, SCU will request twice for each firmware
	 * chunk, since its writing to 2 partitions.
	 * TODO::Investigate if we need to increase timeout for busy_wait,
	 * since SCU is now writing to 2 partitions.
	 */

	while ((mb_state = check_mb_status(&mfld_fw_upd)) != MB_DONE) {

		if (mb_state == MB_ERROR) {
			dev_dbg(fui.dev, "check_mb_status,error\n");
			ret_val = -1;
			goto term;
		}

		if (!strncmp(mfld_fw_upd.mb_status, FUPH_HDR_SIZE,
				strlen(FUPH_HDR_SIZE))) {
			iowrite32(fw_ud_param->fuph_hdr_len, mfld_fw_upd.sram);
			/* There are synchronization issues between IA-SCU */
			mb();
			dev_dbg(fui.dev,
				"copied fuph hdr size=%d\n",
				ioread32(mfld_fw_upd.sram));
			mfld_fw_upd.wia = !mfld_fw_upd.wia;
			iowrite32(mfld_fw_upd.wia, mfld_fw_upd.mailbox +
				IA_FLAG_OFFSET);
			dev_dbg(fui.dev, "ia_flag=%d\n",
				ioread32(mfld_fw_upd.mailbox + IA_FLAG_OFFSET));
			mb();
			mfld_fw_upd.wscu = !mfld_fw_upd.wscu;

			if (busy_wait(&mfld_fw_upd) < 0) {
				ret_val = -1;
				goto term;
			}

			continue;
		}

		if (calc_offset_and_length(fw_ud_param, mfld_fw_upd.mb_status,
					&offset, &length, &fuph) < 0) {
			dev_err(fui.dev,
			"calc_offset_and_length_error,error\n");
			ret_val = -1;
			goto term;
		}

		if ((process_fw_chunk(fws, offset, length,
				      &mfld_fw_upd)) != 0) {
			dev_err(fui.dev,
			"Error processing fw chunk=%s\n",
			mfld_fw_upd.mb_status);
			ret_val = -1;
			goto term;
		} else
			dev_dbg(fui.dev,
				"PASS processing fw chunk=%s\n",
				mfld_fw_upd.mb_status);
	}
	ret_val = intel_scu_ipc_check_status();

term:
	kfree(fws);
unmap_mb:
	iounmap(mfld_fw_upd.mailbox);
unmap_sram:
	iounmap(mfld_fw_upd.sram);
out_unlock:
	intel_scu_ipc_unlock();
	return ret_val;
}
EXPORT_SYMBOL_GPL(intel_scu_ipc_medfw_upgrade);

static void cur_err(const char *err_info)
{
	strncpy(err_buf, err_info, sizeof(err_buf) - 1);
}

static ssize_t write_dnx(struct file *file, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	int ret;

	mutex_lock(&fwud_lock);

	if (!pending_data) {
		pending_data = vmalloc(FOTA_MEM_SIZE);
		if (NULL == pending_data) {
			cur_err("alloc fota memory by sysfs failed\n");
			ret = -ENOMEM;
			goto end;
		}
	}

	fui.fwud_pending->dnx_file_data = pending_data + IFWI_MAX_SIZE;

	if (unlikely(off >= DNX_MAX_SIZE)) {
		fui.fwud_pending->dnx_file_data = NULL;
		cur_err("too large dnx binary stream!");
		ret = -EFBIG;
		goto end;
	}

	memcpy(fui.fwud_pending->dnx_file_data + off, buf, count);

	if (!off)
		fui.fwud_pending->dnx_size = count;
	else
		fui.fwud_pending->dnx_size += count;

	mutex_unlock(&fwud_lock);
	return count;

end:
	mutex_unlock(&fwud_lock);
	return ret;
}

/* Parses from the end of IFWI, and looks for UPH$,
 * to determine length of FUPH header
 */
static int find_fuph_header_len(unsigned int *len,
		unsigned char *file_data, unsigned int file_size)
{
	int ret = -EINVAL;
	unsigned char *temp;
	unsigned int cnt = 0;

	if (!len || !file_data || !file_size) {
		dev_err(fui.dev, "find_fuph_header_len: Invalid inputs\n");
		return ret;
	}

	/* Skipping the checksum at the end, and moving to the
	 * start of the last add-on firmware size in fuph.
	 */
	temp = file_data + file_size - SKIP_BYTES;

	while (cnt <= FUPH_MAX_LEN) {
		if (!strncmp(temp, FUPH_STR, FUPH_STR_LEN)) {
			pr_info("Fuph_hdr_len=%d\n", cnt + SKIP_BYTES);
			*len = cnt + SKIP_BYTES;
			ret = 0;
			break;
		}
		temp -= 4;
		cnt += 4;
	}

	return ret;
}

static ssize_t write_ifwi(struct file *file, struct kobject *kobj,
	struct bin_attribute *attr, char *buf, loff_t off, size_t count)
{
	int ret;

	mutex_lock(&fwud_lock);

	if (!pending_data) {
		pending_data = vmalloc(FOTA_MEM_SIZE);
		if (NULL == pending_data) {
			cur_err("alloc fota memory by sysfs failed\n");
			ret = -ENOMEM;
			goto end;
		}
	}

	fui.fwud_pending->fw_file_data = pending_data;

	if (unlikely(off >= IFWI_MAX_SIZE)) {
		fui.fwud_pending->fw_file_data = NULL;
		cur_err("too large ifwi binary stream!\n");
		ret = -EFBIG;
		goto end;
	}

	memcpy(fui.fwud_pending->fw_file_data + off, buf, count);

	if (!off)
		fui.fwud_pending->fsize = count;
	else
		fui.fwud_pending->fsize += count;

	mutex_unlock(&fwud_lock);
	return count;

end:
	mutex_unlock(&fwud_lock);
	return ret;
}

/*
 * intel_scu_fw_prepare - prepare dnx_hdr and fuph
 *
 * This function will be invoked at reboot, when DNX and IFWI data are ready.
 */
static int intel_scu_fw_prepare(struct fw_ud *fwud_pending)
{
	unsigned int size;
	unsigned int gpFlags = 0;
	unsigned int xorcs;
	unsigned char dnxSH[DNX_HDR_LEN] = { 0 };

	mutex_lock(&fwud_lock);

	size = fui.fwud_pending->dnx_size;

	/* Set GPFlags parameter */
	gpFlags = gpFlags | (GPF_BIT32 << 31);
	xorcs = (size ^ gpFlags);

	memcpy((dnxSH + DNX_SIZE_OFFSET), (unsigned char *)(&size), 4);
	memcpy((dnxSH + GP_FLAG_OFFSET), (unsigned char *)(&gpFlags), 4);
	memcpy((dnxSH + XOR_CHK_OFFSET), (unsigned char *)(&xorcs), 4);

	/* assign the last DNX_HDR_LEN bytes memory to dnx header */
	fui.fwud_pending->dnx_hdr = pending_data + FOTA_MEM_SIZE - DNX_HDR_LEN;

	/* directly memcpy to dnx_hdr */
	memcpy(fui.fwud_pending->dnx_hdr, dnxSH, DNX_HDR_LEN);

	if (find_fuph_header_len(&(fui.fwud_pending->fuph_hdr_len),
			fui.fwud_pending->fw_file_data,
			fui.fwud_pending->fsize) < 0) {
		pr_err("Error, with FUPH header\n");
		mutex_unlock(&fwud_lock);
		return -EINVAL;
	}

	pr_debug("fupd_hdr_len=%d, fsize=%d, dnx_size=%d",
			fui.fwud_pending->fuph_hdr_len,
			fui.fwud_pending->fsize,
			fui.fwud_pending->dnx_size);

	mutex_unlock(&fwud_lock);
	return 0;
}

int intel_scu_ipc_fw_update(void)
{
	int ret = 0;

	/* jump fw upgrade process when fota memory not allocated
	 * or when user cancels update
	 * or when one of dnx and ifwi is not written
	 * or when failure happens in writing one of dnx and ifwi
	 */
	if (!pending_data || !fui.fwud_pending ||
		!fui.fwud_pending->dnx_file_data ||
		!fui.fwud_pending->fw_file_data) {
		pr_info("Jump FW upgrade process\n");
		goto end;
	}

	ret = intel_scu_fw_prepare(fui.fwud_pending);
	if (ret) {
		dev_err(fui.dev, "intel_scu_fw_prepare failed\n");
		goto end;
	}

	ret = intel_scu_ipc_medfw_upgrade();
	if (ret)
		dev_err(fui.dev, "intel_scu_ipc_medfw_upgrade failed\n");

end:
	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_fw_update);

static ssize_t intel_scu_ipc_show_fw_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	u8 data[16];
	int ret;
	int i;
	int used = 0;

	ret = intel_scu_ipc_command(IPCMSG_FW_REVISION, 0,
			NULL, 0, (u32 *)data, 4);
	if (ret < 0) {
		cur_err("Error get fw version");
		return -EINVAL;
	}

	for (i = 0; i < 16; i++)
		used += snprintf(buf + used, PAGE_SIZE - used, "%x ", data[i]);

	return used;
}

static ssize_t intel_scu_ipc_show_last_error(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", err_buf);
}

static ssize_t intel_scu_ipc_store_cancel_update(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value;

	if (sscanf(buf, "%d", &value) != 1) {
		cur_err("One argument is needed\n");
		return -EINVAL;
	}

	if (value == 1) {
		mutex_lock(&fwud_lock);
		fui.fwud_pending->fw_file_data = NULL;
		fui.fwud_pending->dnx_file_data = NULL;
		mutex_unlock(&fwud_lock);
	} else {
		cur_err("input '1' to cancel fw upgrade\n");
		return -EINVAL;
	}

	return size;
}

#define __BIN_ATTR(_name, _mode, _size, _read, _write) { \
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.size	= _size,					\
	.read	= _read,					\
	.write	= _write,					\
}

#define BIN_ATTR(_name, _mode, _size, _read, _write) \
struct bin_attribute bin_attr_##_name =	\
		__BIN_ATTR(_name, _mode, _size, _read, _write)

static DEVICE_ATTR(cancel_update, S_IWUSR, NULL,
		intel_scu_ipc_store_cancel_update);
static DEVICE_ATTR(fw_version, S_IRUGO, intel_scu_ipc_show_fw_version, NULL);
static DEVICE_ATTR(last_error, S_IRUGO, intel_scu_ipc_show_last_error, NULL);
static BIN_ATTR(dnx, S_IWUSR, DNX_MAX_SIZE, NULL, write_dnx);
static BIN_ATTR(ifwi, S_IWUSR, IFWI_MAX_SIZE, NULL, write_ifwi);

static struct attribute *intel_fw_update_attrs[] = {
	&dev_attr_cancel_update.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_last_error.attr,
	NULL,
};

static struct attribute_group intel_fw_update_attr_group = {
	.name = "fw_info",
	.attrs = intel_fw_update_attrs,
};

static int intel_fw_update_sysfs_create(struct ipc_device *ipcdev)
{
	int ret;

	ret = sysfs_create_group(&ipcdev->dev.kobj,
				&intel_fw_update_attr_group);
	if (ret) {
		dev_err(&ipcdev->dev, "Unable to export sysfs interface\n");
		ret = -EINVAL;
	}

	ret = sysfs_create_bin_file(&ipcdev->dev.kobj, &bin_attr_dnx);
	if (ret) {
		dev_err(&ipcdev->dev, "Unable to create bin file\n");
		ret = -EINVAL;
		goto dnx_err;
	}

	ret = sysfs_create_bin_file(&ipcdev->dev.kobj, &bin_attr_ifwi);
	if (ret) {
		dev_err(&ipcdev->dev, "Unable to create bin file\n");
		ret = -EINVAL;
		goto ifwi_err;
	}

	return 0;

ifwi_err:
	sysfs_remove_bin_file(&ipcdev->dev.kobj, &bin_attr_dnx);
dnx_err:
	sysfs_remove_group(&ipcdev->dev.kobj, &intel_fw_update_attr_group);
	return ret;
}

static void intel_fw_update_sysfs_remove(struct ipc_device *ipcdev)
{
	sysfs_remove_bin_file(&ipcdev->dev.kobj, &bin_attr_ifwi);
	sysfs_remove_bin_file(&ipcdev->dev.kobj, &bin_attr_dnx);
	sysfs_remove_group(&ipcdev->dev.kobj, &intel_fw_update_attr_group);
}

static int __devinit fw_update_probe(struct ipc_device *ipcdev)
{
	int ret;
	struct fw_update_info *fu_info = &fui;

	fu_info->dev = &ipcdev->dev;

	fui.fwud_pending = kzalloc(sizeof(struct fw_ud), GFP_KERNEL);
	if (NULL == fui.fwud_pending) {
		ret = -ENOMEM;
		dev_err(fui.dev, "alloc fwud_pending memory failed\n");
		goto exit;
	}

	ret = intel_fw_update_sysfs_create(ipcdev);
	if (ret) {
		dev_err(fui.dev, "creating fw update sysfs failed\n");
		goto err_free_fwud;
	}

	/* If alloc_fota_mem_early flag is set, allocate FOTA_MEM_SIZE
	 * bytes memory.
	 * reserve the first contiguous IFWI_MAX_SIZE bytes for IFWI,
	 * the next contiguous DNX_MAX_SIZE bytes are reserved for DNX,
	 * the last DNX_HDR_LEN bytes for DNX Header
	 */
	if (alloc_fota_mem_early) {
		pending_data = vmalloc(FOTA_MEM_SIZE);
		if (NULL == pending_data) {
			ret = -ENOMEM;
			dev_err(fui.dev, "early alloc fota memory failed\n");
			goto err_sysfs;
		}
	}

	return 0;

err_sysfs:
	intel_fw_update_sysfs_remove(ipcdev);
err_free_fwud:
	kfree(fui.fwud_pending);
	fui.fwud_pending = NULL;
exit:
	return ret;
}

static int __devexit fw_update_remove(struct ipc_device *ipcdev)
{
	intel_fw_update_sysfs_remove(ipcdev);

	vfree(pending_data);
	pending_data = NULL;
	kfree(fui.fwud_pending);
	fui.fwud_pending = NULL;
	return 0;
}

static struct ipc_driver fw_update_driver = {
	.driver = {
		.name = "intel_fw_update",
		.owner = THIS_MODULE,
	},
	.probe = fw_update_probe,
	.remove = __devexit_p(fw_update_remove),
};

static int __init fw_update_module_init(void)
{
	return ipc_driver_register(&fw_update_driver);
}

static void __exit fw_update_module_exit(void)
{
	ipc_driver_unregister(&fw_update_driver);
}

module_init(fw_update_module_init);
module_exit(fw_update_module_exit);

MODULE_AUTHOR("Sreedhara DS <sreedhara.ds@intel.com>");
MODULE_AUTHOR("Ning Li <ning.li@intel.com>");
MODULE_DESCRIPTION("Intel SCU Firmware Update Driver");
MODULE_LICENSE("GPL v2");
