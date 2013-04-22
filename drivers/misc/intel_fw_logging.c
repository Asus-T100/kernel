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
#include <linux/rpmsg.h>
#include <asm/intel_mid_rpmsg.h>

#include <linux/io.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>

#include "intel_fabricid_def.h"

/*
   OSHOB - OS Handoff Buffer

   This buffer contains the 32-byte value that is persists across cold and warm
   resets only, but loses context on a cold boot.

   More info about OSHOB, OSNIB could be found in FAS Section 2.8.
   We use the first byte in OSNIB to store and pass the Reboot/boot Reason.
   The attribute of OS image is selected for Reboot/boot reason.
*/

#define MAX_BUFFER_SIZE			1024
#define FID_MSB_MAPPING			32
#define MAX_FID_REG_LEN			32
#define MAX_NUM_LOGDWORDS		12
#define MAX_NUM_LOGDWORDS_EXTENDED      9
#define MAX_NUM_ALL_LOGDWORDS           (2 + (MAX_NUM_LOGDWORDS + \
						MAX_NUM_LOGDWORDS_EXTENDED))
#define MAX_RAW_DWORDS_SIZE             (16  * MAX_NUM_ALL_LOGDWORDS)
#define MAX_FULL_SIZE                   (MAX_BUFFER_SIZE + MAX_RAW_DWORDS_SIZE)
#define FABERR_INDICATOR		0x15
#define FWERR_INDICATOR			0x7
#define UNDEFLVL1ERR_IND		0x11
#define UNDEFLVL2ERR_IND		0x22
#define SWDTERR_IND			0xdd
#define MEMERR_IND			0xf501
#define INSTERR_IND			0xf502
#define ECCERR_IND			0xf504
#define FLAG_HILOW_MASK			8
#define FAB_ID_MASK			7
#define MAX_AGENT_IDX			15

/* Safety limits for SCU extra trace dump */
#define LOWEST_PHYS_SRAM_ADDRESS        0xFFFC0000
#define MAX_SCU_EXTRA_DUMP_SIZE         1024
#define CHAR_PER_LINE_EXTRA_TRACE       20

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
static char *log_buffer;

static struct rpmsg_instance *fw_logging_instance;

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
		memcpy(buffer, log_buffer, MAX_FULL_SIZE);
		return MAX_FULL_SIZE;
	}
}
#endif /* CONFIG_PROC_FS */

static void __iomem *get_oshob_addr(void)
{
	int ret;
	u32 oshob_base = 0;
	u16 oshob_size;
	void __iomem *oshob_addr;

	ret = rpmsg_send_command(fw_logging_instance,
			IPCMSG_GET_HOBADDR, 0, NULL, &oshob_base, 0, 1);

	if (ret < 0 || oshob_base == 0) {
		pr_err("ipc_read_oshob address failed!!\n");
		return NULL;
	}

	oshob_size = intel_scu_ipc_get_oshob_size();

	pr_debug("OSHOB addr is 0x%x size is %d\n", oshob_base, oshob_size);

	if (oshob_size == 0) {
		pr_err("size of oshob is null!!\n");
		return NULL;
	}

	oshob_addr = ioremap_nocache(oshob_base, oshob_size);

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
				strlcat(buf, ptr, MAX_BUFFER_SIZE);
				strlcat(buf, "\n", MAX_BUFFER_SIZE);
			}
		}

		index++;
		fid_mask <<= 1;
	}

	strlcat(buf, "\n", MAX_BUFFER_SIZE);
}

static void get_additional_error(char *buf, int num_err_log,
			u32 *faberr_dwords, int max_dw_left)
{
	int i = 0;
	char temp[100], str[50];
	union error_log_dw10 log;

	strlcat(buf, "\nAdditional logs associated with error(s): ",
							MAX_BUFFER_SIZE);

	if (num_err_log) {

		while (i < num_err_log && i < max_dw_left) {

			sprintf(temp, "\nerror_log: 0x%X\n",
					*(faberr_dwords + i));

			strlcat(buf, temp, MAX_BUFFER_SIZE);
			sprintf(temp, "error_addr: 0x%X\n",
				*(faberr_dwords + i + 1));

			strlcat(buf, temp, MAX_BUFFER_SIZE);
			log.data = *(faberr_dwords + i);

			strlcat(buf, "\nDecoded error log detail\n",
							MAX_BUFFER_SIZE);
			strlcat(buf, "---------------------------\n\n",
							MAX_BUFFER_SIZE);

			if (log.fields.agent_idx > MAX_AGENT_IDX)
				sprintf(str, "Unknown agent index (%d)\n",
					log.fields.agent_idx);
			else
				snprintf(str, sizeof(str)-1, "%s\n",
					Agent_Names[log.fields.agent_idx]);

			snprintf(temp, sizeof(temp)-1, "Agent Index:%s\n", str);
			strlcat(buf, temp, MAX_BUFFER_SIZE);

			sprintf(temp, "Cmd initiator ID: %d\n",
						log.fields.initid);
			strlcat(buf, temp, MAX_BUFFER_SIZE);

			sprintf(temp, "Command: %d\n", log.fields.cmd);
			strlcat(buf, temp, MAX_BUFFER_SIZE);

			sprintf(temp, "Code: %d\n", log.fields.err_code);
			strlcat(buf, temp, MAX_BUFFER_SIZE);

			if (log.fields.multi_err)
				strlcat(buf, "\n Multiple errors detected!\n",
							MAX_BUFFER_SIZE);

			i += 2; /* Skip one error_log/addr pair */
		}
	} else {
		strlcat(buf, "Not present\n", MAX_BUFFER_SIZE);
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
	u32 id = FAB_ID_UNKNOWN;
	u32 faberr_dwords[MAX_NUM_LOGDWORDS + MAX_NUM_LOGDWORDS_EXTENDED] = {0};
	int count, num_flag_status, num_err_logs;
	int prev_id = FAB_ID_UNKNOWN, offset = 0;
	char temp[100];

	void __iomem *fabric_err_dump_offset = oshob_ptr +
		intel_scu_ipc_get_fabricerror_buf1_offset();

	for (count = 0; count < MAX_NUM_LOGDWORDS; count++) {
		faberr_dwords[count] = readl(fabric_err_dump_offset +
						count * sizeof(u32));
	}

	/* Get 9 additional DWORDS */
	fabric_err_dump_offset = oshob_ptr +
		intel_scu_ipc_get_fabricerror_buf2_offset();

	for (count = 0; count < MAX_NUM_LOGDWORDS_EXTENDED; count++) {
		faberr_dwords[count + MAX_NUM_LOGDWORDS] =
			readl(fabric_err_dump_offset + sizeof(u32) * count);
	}

	err_status_dw0.data = faberr_dwords[0];
	err_log_dw10.data = faberr_dwords[10];

	/* No SCU/fabric error if tenth DW signature field is not 10101 */
	if (err_log_dw10.fields.signature != FABERR_INDICATOR)
		return 0;

	/* FW error if tenth DW reserved field is 111 */
	if ((((err_status_dw0.data & 0xFFFF) == SWDTERR_IND) ||
		((err_status_dw0.data & 0xFFFF) == UNDEFLVL1ERR_IND) ||
		((err_status_dw0.data & 0xFFFF) == UNDEFLVL2ERR_IND) ||
		((err_status_dw0.data & 0xFFFF) == MEMERR_IND) ||
		((err_status_dw0.data & 0xFFFF) == INSTERR_IND) ||
		((err_status_dw0.data & 0xFFFF) == ECCERR_IND)) &&
		(err_log_dw10.fields.reserved1 == FWERR_INDICATOR)) {

		sprintf(output_buf, "HW WDT expired");

		switch (err_status_dw0.data & 0xFFFF) {
		case SWDTERR_IND:
			strcat(output_buf,
			" without facing any exception.\n\n");
			break;
		case MEMERR_IND:
			strcat(output_buf,
			" following a Memory Error exception.\n\n");
			break;
		case INSTERR_IND:
			strcat(output_buf,
			" following an Instruction Error exception.\n\n");
			break;
		case ECCERR_IND:
			strcat(output_buf,
			" following a SRAM ECC Error exception.\n\n");
			break;
		default:
			strcat(output_buf,
			".\n\n");
			break;
		}
		strcat(output_buf, "HW WDT debug data:\n");
		strcat(output_buf, "===================\n");
		for (count = 0;
			count < MAX_NUM_LOGDWORDS + MAX_NUM_LOGDWORDS_EXTENDED;
			count++) {
			sprintf(temp, "DW%d:0x%08x\n",
					count, faberr_dwords[count]);
			strcat(output_buf, temp);
		}
		return strlen(output_buf);
	}

	num_flag_status = err_status_dw0.fields.flag_status_cnt;
	/* num_err_logs indicates num of error_log/addr pairs */
	num_err_logs = err_log_dw10.fields.num_err_logs * 2;

	sprintf(output_buf,
		"HW WDT fired following a Fabric Error exception.\n\n");
	strcat(output_buf, "Fabric Error debug data:\n");
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
			strlcat(output_buf, ptr, MAX_BUFFER_SIZE);
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

	strlcat(output_buf, "\n\n\nAdditional debug data:\n\n", MAX_FULL_SIZE);
	for (count = 0;
		count < MAX_NUM_LOGDWORDS + MAX_NUM_LOGDWORDS_EXTENDED;
		count++) {
		sprintf(temp, "DW%d:0x%08x\n",
			count, faberr_dwords[count]);
		strlcat(output_buf, temp, MAX_FULL_SIZE);
	}
	return strlen(output_buf);
}

static int dump_scu_extented_trace(char *outbuf, int start_offset, u32 size)
{
	u32 buffer;
	int offset;
	int i;
	unsigned int lines;
	void __iomem *scubuffer;

	offset = start_offset;

	if ((size == 0) || (size > MAX_SCU_EXTRA_DUMP_SIZE))
		goto leave;

	/*
	 * SCU gives pointer via oshob. Address is a physical
	 * address somewhere in shared sram
	 */
	buffer = intel_scu_ipc_get_scu_trace_buffer();
	if (!buffer || buffer < LOWEST_PHYS_SRAM_ADDRESS)
		goto leave;

	/* Looks that we have valid buffer and size. */
	scubuffer = ioremap_nocache(buffer, size);
	if (!scubuffer)
		goto leave;

	lines = size / sizeof(u32);

	/* Title for error dump */
	offset += snprintf(outbuf + offset, CHAR_PER_LINE_EXTRA_TRACE,
			   "SCU Extra trace\n");

	for (i = 0; i < lines; i++) {
		/*
		 * "EW:" to separate lines from "DW:" lines
		 * elsewhere in this file.
		 */
		offset += snprintf(outbuf + offset,
				   CHAR_PER_LINE_EXTRA_TRACE, "EW%d:0x%08x\n",
				   i,
				   readl(scubuffer + i * sizeof(u32)));
	}
	iounmap(scubuffer);
leave:
	return offset;
}

static int intel_fw_logging_init(void)
{
	int length = 0;
	int err = 0;
	u32 scu_trace_size;
	u32 trace_size;

	oshob_base = get_oshob_addr();

	if (oshob_base == NULL)
		return -EINVAL;

	/*
	 * Calculate size of SCU extra trace buffer. Size of the buffer
	 * is given by SCU. Make sanity check in case of incorrect data.
	 * We don't want to allocate all of the memory for trace dump.
	 */
	trace_size = intel_scu_ipc_get_scu_trace_buffer_size();
	if (trace_size > MAX_SCU_EXTRA_DUMP_SIZE)
		trace_size = 0;

	/*
	 * Buffer size is in bytes. We will dump 32-bit hex values + header.
	 * Calculate number of lines and assume constant number of
	 * characters per line.
	 */
	scu_trace_size = ((trace_size / sizeof(u32)) + 2) *
		CHAR_PER_LINE_EXTRA_TRACE;

	/* Allocate buffer for traditional trace and extra trace */
	log_buffer = vzalloc(MAX_FULL_SIZE + scu_trace_size);
	if (!log_buffer) {
		err = -ENOMEM;
		goto err_nomem;
	}

	length = create_fwerr_log(log_buffer, oshob_base);
	if (!length)
		goto err_nolog;

	length = dump_scu_extented_trace(log_buffer, length,
					 trace_size);

#ifdef CONFIG_PROC_FS
	ipanic_faberr = create_proc_entry("ipanic_fabric_err",
					  S_IFREG | S_IRUGO, NULL);
	if (ipanic_faberr == 0) {
		pr_err("Fail creating procfile ipanic_fabric_err\n");
		err = -ENOMEM;
		goto err_procfail;
	}

	ipanic_faberr->read_proc = intel_fw_logging_proc_read;
	ipanic_faberr->write_proc = NULL;
	ipanic_faberr->size = length;

#endif /* CONFIG_PROC_FS */

	/* Dump log as error to console */
	pr_err("%s", log_buffer);

	/* Clear fabric error region inside OSHOB if neccessary */
	rpmsg_send_simple_command(fw_logging_instance,
				IPCMSG_CLEAR_FABERROR, 0);
	goto leave;

err_procfail:
err_nolog:
	vfree(log_buffer);
err_nomem:
leave:
	iounmap(oshob_base);
	return err;
}

static void intel_fw_logging_exit(void)
{
#ifdef CONFIG_PROC_FS
	if (ipanic_faberr)
		remove_proc_entry("ipanic_fabric_err", NULL);
#endif /* CONFIG_PROC_FS */
	vfree(log_buffer);
}

static int fw_logging_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("fw_logging rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed fw_logging rpmsg device\n");

	/* Allocate rpmsg instance for fw_logging*/
	ret = alloc_rpmsg_instance(rpdev, &fw_logging_instance);
	if (!fw_logging_instance) {
		dev_err(&rpdev->dev, "kzalloc fw_logging instance failed\n");
		goto out;
	}
	/* Initialize rpmsg instance */
	init_rpmsg_instance(fw_logging_instance);
	/* Init scu fw_logging */
	ret = intel_fw_logging_init();

	if (ret)
		free_rpmsg_instance(rpdev, &fw_logging_instance);
out:
	return ret;
}

static void fw_logging_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	intel_fw_logging_exit();
	free_rpmsg_instance(rpdev, &fw_logging_instance);
	dev_info(&rpdev->dev, "Removed fw_logging rpmsg device\n");
}

static void fw_logging_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id fw_logging_rpmsg_id_table[] = {
	{ .name	= "rpmsg_fw_logging" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, fw_logging_rpmsg_id_table);

static struct rpmsg_driver fw_logging_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= fw_logging_rpmsg_id_table,
	.probe		= fw_logging_rpmsg_probe,
	.callback	= fw_logging_rpmsg_cb,
	.remove		= fw_logging_rpmsg_remove,
};

static int __init fw_logging_rpmsg_init(void)
{
	return register_rpmsg_driver(&fw_logging_rpmsg);
}
module_init(fw_logging_rpmsg_init);

static void __exit fw_logging_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&fw_logging_rpmsg);
}
module_exit(fw_logging_rpmsg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Utility driver for getting intel scu fw debug info");
MODULE_AUTHOR("Winson Yung <winson.w.yung@intel.com>");
