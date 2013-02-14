/*
 * drivers/misc/intel_fw_logging.c
 *
 * Copyright (C) 2011 Intel Corp
 * Author: winson.w.yung@intel.com
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/pti.h>

#include <linux/io.h>
#include <asm/intel_scu_ipc.h>

#include "intel_fabricid_def.h"

#define IPCMSG_GET_HOBADDR		0xE5
#define IPCMSG_CLEAR_FABERROR		0xE3
#define OSHOB_SIZE			120

/*
   OSHOB - OS Handoff Buffer

   This buffer contains the 32-byte value that is persists across cold and warm
   resets only, but loses context on a cold boot.

   More info about OSHOB, OSNIB could be found in FAS Section 2.8.
   We use the first byte in OSNIB to store and pass the Reboot/boot Reason.
   The attribute of OS image is selected for Reboot/boot reason.
*/

#define OSFAB_ERR_OFFSET		0x38
#define MAX_BUFFER_SIZE			1024
#define FID_MSB_MAPPING			32
#define MAX_FID_REG_LEN			32
#define MAX_NUM_LOGDWORDS		12
#define FABERR_INDICATOR		0x15
#define FLAG_HILOW_MASK			8
#define FAB_ID_MASK			7
#define MAX_AGENT_IDX			15

union error_log_dw10 {
	struct {
		u32 cmd:3;
		u32 signature:5;
		u32 initid:8;
		u32 num_err_logs:4;
		u32 agent_idx:4;
		u32 err_code:4;
		u32 reserved1:3;
		u32 multi_err:1;
	} fields;
	u32 data;
};

union fabric_status_dw0 {
	struct {
		u32 status_has_hilo:11;
		u32 flag_status_cnt:5;
		u32 status_has_hilo1:12;
		u32 regidx:4;
	} fields;
	u32 data;
};

union flag_status_hilo {
	struct {
		/* Maps to flag_status [10..0] or [42..32] */
		u32 bits_rang0:11;
		u32 reserved1:5;

		/* Maps to flag_status [27..16] or [59..48] */
		u32 bits_rang1:12;
		u32 reserved:4;
	} fields;
	u32 data;
};

static void __iomem *oshob_base;
static char log_buffer[MAX_BUFFER_SIZE] = {0};

static char *Fabric_Names[] = {
	"\nFull Chip Fabric [error]\n\n",
	"\nAudio Fabric [error]\n\n",
	"\nSecondary Chip Fabric [error]\n\n",
	"\nGP Fabric [error]\n\n",
	"\nSC Fabric [error]\n\n",
	"\nUnknown Fabric [error]\n\n"
};

static char *Agent_Names[] = {
	"FULLFAB_FLAG_STATUS",
	"AUDIO",
	"SECONDARY",
	"GP",
	"SC",
	"CDMI_TOCP_TA",
	"CDMI_IOCP_IA",
	"FCSF_IOCP_IA",
	"FCGF_IOCP_IA",
	"AFSF_IOCP_IA",
	"SFFC_IOCP_IA",
	"SFAF_IOCP_IA",
	"SFSC_IOCP_IA",
	"GFFC_IOCP_IA",
	"ARC_IOCP_IA",
	"SCSF_TOCP_TA"
};

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *ipanic_faberr;

static int intel_fw_logging_proc_read(char *buffer, char **start, off_t offset,
					int count, int *peof, void *data)
{
	if (offset > 0) {
		/* We have finished to read, return 0 */
		return 0;
	} else {
		/* Fill the buffer, return the buffer size */
		memcpy(buffer, log_buffer, MAX_BUFFER_SIZE);
		return MAX_BUFFER_SIZE;
	}
}
#endif /* CONFIG_PROC_FS */

static void __iomem *get_oshob_addr(void)
{
	int ret;
	u32 oshob_base;
	void __iomem *oshob_addr;

	ret = intel_scu_ipc_command(IPCMSG_GET_HOBADDR, 0, NULL,
					0, &oshob_base, 1);

	if (ret < 0) {
		pr_err("ipc_read_oshob address failed!!\n");
		return NULL;
	}

	pr_debug("OSHOB addr is 0x%x\n", oshob_base);
	oshob_addr = ioremap_nocache(oshob_base, OSHOB_SIZE);

	if (!oshob_addr) {
		pr_err("ioremap of oshob address failed!!\n");
		return NULL;
	}

	return oshob_addr; /* Return OSHOB base address */
}

static void get_fabric_error_cause_detail(char *buf, u32 FabId,
				u32 *fid_status, int IsHiDword)
{
	int index = 0;
	char *ptr;
	u32 fid_mask = 1;

	while (index < MAX_FID_REG_LEN) {

		if ((*fid_status) & fid_mask) {
			ptr = fabric_error_lookup(FabId, index, IsHiDword);

			if (ptr != NULL && strlen(ptr)) {
				strcat(buf, ptr);
				strcat(buf, "\n");
			}
		}

		index++;
		fid_mask <<= 1;
	}

	strcat(buf, "\n");
}

static void get_additional_error(char *buf, int num_err_log,
			u32 *faberr_dwords, int max_dw_left)
{
	int i = 0;
	char temp[100], str[50];
	union error_log_dw10 log;

	strcat(buf, "\nAdditional logs associated with error(s): ");

	if (num_err_log) {

		while (i < num_err_log && i < max_dw_left) {

			sprintf(temp, "\nerror_log: 0x%X\n",
					*(faberr_dwords + i));

			strcat(buf, temp);
			sprintf(temp, "error_addr: 0x%X\n",
				*(faberr_dwords + i + 1));

			strcat(buf, temp);
			log.data = *(faberr_dwords + i);

			strcat(buf, "\nDecoded error log detail\n");
			strcat(buf, "---------------------------\n\n");

			if (log.fields.agent_idx > MAX_AGENT_IDX)
				sprintf(str, "Unknown agent index (%d)\n",
					log.fields.agent_idx);
			else
				sprintf(str, "%s\n",
					Agent_Names[log.fields.agent_idx]);

			sprintf(temp, "Agent Index: %s\n", str);
			strcat(buf, temp);

			sprintf(temp, "Cmd initiator ID: %d\n",
						log.fields.initid);
			strcat(buf, temp);

			sprintf(temp, "Command: %d\n", log.fields.cmd);
			strcat(buf, temp);

			sprintf(temp, "Code: %d\n", log.fields.err_code);
			strcat(buf, temp);

			if (log.fields.multi_err)
				strcat(buf, "\n* Multiple errors detected!\n");

			i += 2; /* Skip one error_log/addr pair */
		}
	} else {
		strcat(buf, "Not present\n");
	}
}

char *get_fabric_name(u32 fabric_idx, u32 *fab_id)
{
	switch (fabric_idx) {
	case 0: /* REGIDX [31..28] is x000 */
	case 1: /* REGIDX [31..28] is x001 */
	case 2: /* REGIDX [31..28] is x010 */
	case 3: /* REGIDX [31..28] is x011 */
	case 4: /* REGIDX [31..28] is x100 */
		*fab_id = fabric_idx;
		break;
	default:
		*fab_id = FAB_ID_UNKNOWN;
		break;
	}

	return Fabric_Names[*fab_id];
}

static int create_fwerr_log(char *output_buf, void __iomem *oshob_ptr)
{
	char *ptr = NULL;
	union error_log_dw10 err_log_dw10;
	union flag_status_hilo flag_status;
	union fabric_status_dw0 err_status_dw0;
	u32 id, faberr_dwords[MAX_NUM_LOGDWORDS] = {0};
	int count, num_flag_status, num_err_logs;
	int prev_id = FAB_ID_UNKNOWN, offset = 0;

	void __iomem *fabric_err_dump_offset = oshob_ptr + OSFAB_ERR_OFFSET;

	for (count = 0; count < MAX_NUM_LOGDWORDS; count++) {
		faberr_dwords[count] = readl(fabric_err_dump_offset +
						count * sizeof(u32));
	}

	err_status_dw0.data = faberr_dwords[0];
	err_log_dw10.data = faberr_dwords[10];

	/* No fabric error if tenth DW signature field is not 10101 */
	if (err_log_dw10.fields.signature != FABERR_INDICATOR)
		return 0;

	num_flag_status = err_status_dw0.fields.flag_status_cnt;
	/* num_err_logs indicates num of error_log/addr pairs */
	num_err_logs = err_log_dw10.fields.num_err_logs * 2;

	sprintf(output_buf, "SCU Fabric summary:\n");
	strcat(output_buf, "===================\n");

	for (count = 0; count < num_flag_status; count++) {

		err_status_dw0.data = faberr_dwords[count];
		ptr = get_fabric_name(
			err_status_dw0.fields.regidx & FAB_ID_MASK, &id);

		/*
		 * Only print the fabric name if is unknown
		 * type, or we haven't printed out it yet.
		 */

		if (prev_id != id || id == FAB_ID_UNKNOWN) {
			strcat(output_buf, ptr);
			prev_id = id;
		}

		flag_status.data = 0;
		flag_status.fields.bits_rang0 =
				err_status_dw0.fields.status_has_hilo;

		flag_status.fields.bits_rang1 =
				err_status_dw0.fields.status_has_hilo1;

		/*
		 * The most significant bit in REGIDX field is set
		 * the flag_status has the high 32bit of the dword
		 * otherwise the flag_status has the low 32bit of
		 * the dword
		 */

		if (err_status_dw0.fields.regidx & FLAG_HILOW_MASK)
			get_fabric_error_cause_detail(output_buf, id,
						&flag_status.data, 1);
		else
			get_fabric_error_cause_detail(output_buf, id,
						&flag_status.data, 0);

		offset++; /* Use this to track error_log/address offset */
	}

	if (offset & 1)
		offset++; /* If offset is odd number, adjust to even offset */

	get_additional_error(output_buf, num_err_logs, &faberr_dwords[offset],
						MAX_NUM_LOGDWORDS - offset);
	return strlen(output_buf);
}

static int __init intel_fw_logging_init(void)
{
	int length = 0;

	oshob_base = get_oshob_addr();

	if (oshob_base == NULL)
		return -EINVAL;

	length = create_fwerr_log(log_buffer, oshob_base);
	if (length != 0) {

#ifdef CONFIG_PROC_FS
		ipanic_faberr = create_proc_entry("ipanic_fabric_err",
						S_IFREG | S_IRUGO, NULL);
		if (ipanic_faberr == 0) {
			pr_err("Fail creating procfile ipanic_fabric_err\n");
			return -ENOMEM;
		}

		ipanic_faberr->read_proc = intel_fw_logging_proc_read;
		ipanic_faberr->write_proc = NULL;
		ipanic_faberr->size = length;

#endif /* CONFIG_PROC_FS */

		/* Dump log as error to console */
		pr_err("%s", log_buffer);

		/* Clear fabric error region inside OSHOB if neccessary */
		intel_scu_ipc_simple_command(IPCMSG_CLEAR_FABERROR, 0);
	}

	iounmap(oshob_base);
	return 0;
}

static void __exit intel_fw_logging_exit(void)
{
#ifdef CONFIG_PROC_FS
	if (ipanic_faberr)
		remove_proc_entry("ipanic_fabric_err", NULL);
#endif /* CONFIG_PROC_FS */
}

module_init(intel_fw_logging_init);
module_exit(intel_fw_logging_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Utility driver for getting intel scu fw debug info");
MODULE_AUTHOR("Winson Yung <winson.w.yung@intel.com>");
