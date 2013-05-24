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
#include <linux/notifier.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <asm/intel_mid_rpmsg.h>

#include <linux/io.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_ipcutil.h>

#include "intel_fabricid_def.h"
#include "intel_fw_trace.h"

/*
   OSHOB - OS Handoff Buffer

   This buffer contains the 32-byte value that is persists across cold and warm
   resets only, but loses context on a cold boot.

   More info about OSHOB, OSNIB could be found in FAS Section 2.8.
   We use the first byte in OSNIB to store and pass the Reboot/boot Reason.
   The attribute of OS image is selected for Reboot/boot reason.
*/

#define FID_MSB_MAPPING			32
#define MAX_FID_REG_LEN			32
#define MAX_NUM_LOGDWORDS		12
#define MAX_NUM_LOGDWORDS_EXTENDED      9
#define MAX_NUM_ALL_LOGDWORDS           (MAX_NUM_LOGDWORDS +		\
					 MAX_NUM_LOGDWORDS_EXTENDED)
#define FABERR_INDICATOR		0x15
#define FWERR_INDICATOR			0x7
#define UNDEFLVL1ERR_IND		0x11
#define UNDEFLVL2ERR_IND		0x22
#define SWDTERR_IND			0xdd
#define MEMERR_IND			0xf501
#define INSTERR_IND			0xf502
#define ECCERR_IND			0xf504
#define FATALERR_IND			0xf505
#define FLAG_HILOW_MASK			8
#define FAB_ID_MASK			7
#define MAX_AGENT_IDX			15

#define DWORDS_PER_LINE			2

/* Safety limits for SCU extra trace dump */
#define LOWEST_PHYS_SRAM_ADDRESS        0xFFFC0000
#define MAX_SCU_EXTRA_DUMP_SIZE         1024

/* Special indexes in error data */
#define FABRIC_ERR_STS_IDX		0
#define FABRIC_ERR_SIGNATURE_IDX	10

/* Timeout in ms we wait SCU to generate dump on panic */
#define SCU_PANIC_DUMP_TOUT		1
#define SCU_PANIC_DUMP_RECHECK		5

#define output_str(ret, out, size, a...)				\
	do {								\
		if (out && (size) - (ret) > 1) {			\
			(ret) += snprintf((out) + (ret),		\
					  (size) - (ret), ## a);	\
			if ((size) - (ret) <= 0)			\
				ret = size - 1;				\
		} else {						\
			pr_info(a);					\
		}							\
	} while (0);

union error_log {
	struct {
		u32 cmd:3;
		u32 signature:5;
		u32 initid:8;
		u32 num_err_logs:4;
		u32 agent_idx:4;
		u32 err_code:4;
		u32 fw_err_ind:3;
		u32 multi_err:1;
	} fields;
	u32 data;
};

union fabric_status {
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
static u32 *log_buffer;
static u32 log_buffer_sz;

static void __iomem *fabric_err_buf1;
static void __iomem *fabric_err_buf2;
static void __iomem *sram_trace_buf;

static struct scu_trace_hdr_t trace_hdr;
static u32 *trace_buf;

static int sram_buf_sz;

static int irq;

static struct rpmsg_instance *fw_logging_instance;

static char *fabric_names[] = {
	"\nFull Chip Fabric [error]\n\n",
	"\nAudio Fabric [error]\n\n",
	"\nSecondary Chip Fabric [error]\n\n",
	"\nGP Fabric [error]\n\n",
	"\nSC Fabric [error]\n\n",
	"\nUnknown Fabric [error]\n\n"
};

static char *agent_names[] = {
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

static irqreturn_t fw_logging_irq_thread(int irq, void *ignored)
{
	char *trace, *end, prefix[20];
	unsigned int count;
	int i, len;
	u32 size;

	i = snprintf(prefix, sizeof(prefix), "SCU TRACE ");
	switch (trace_hdr.cmd & TRACE_ID_MASK) {
	case TRACE_ID_INFO:
		i += snprintf(prefix + i, sizeof(prefix) - i, "INFO");
		break;
	case TRACE_ID_ERROR:
		i += snprintf(prefix + i, sizeof(prefix) - i, "ERROR");
		break;
	default:
		pr_err("Invalid message ID!");
		break;
	}
	snprintf(prefix + i, sizeof(prefix) - i, ": ");

	if (trace_hdr.cmd & TRACE_IS_ASCII) {
		size = trace_hdr.size;
		trace = (char *)trace_buf;
		end = trace + trace_hdr.size;
		while (trace < end) {
			len = strnlen(trace, size);
			if (!len) {
				trace++;
				continue;
			}
			pr_info("%s%s\n", prefix, trace);
			trace += len + 1;
			size -= len;
		}
	} else {
		count = trace_hdr.size / sizeof(u32);

		for (i = 0; i < count; i++)
			pr_info("%s[%d]:0x%08x\n", prefix, i, trace_buf[i]);
	}

	return IRQ_HANDLED;
}

static void read_scu_trace_hdr(struct scu_trace_hdr_t *hdr)
{
	unsigned int count;
	u32 *buf = (u32 *) hdr;
	int i;

	if (!hdr)
		return;

	count = sizeof(struct scu_trace_hdr_t) / sizeof(u32);

	for (i = 0; i < count; i++)
		*(buf + i)  = readl(fabric_err_buf1 + i * sizeof(u32));
}

static void read_sram_trace_buf(u32 *buf, u8 *scubuffer, unsigned int size)
{
	int i;
	unsigned int count;

	if (!buf || !scubuffer || !size)
		return;

	count = size / sizeof(u32);
	for (i = 0; i < count; i++)
		buf[i] = readl(scubuffer + i * sizeof(u32));
}

static irqreturn_t fw_logging_irq(int irq, void *ignored)
{
	read_scu_trace_hdr(&trace_hdr);

	if (trace_hdr.magic != TRACE_MAGIC ||
	    trace_hdr.offset + trace_hdr.size > sram_buf_sz) {
		pr_err("Invalid SCU trace!");
		return IRQ_HANDLED;
	}

	read_sram_trace_buf(trace_buf, (u8 *) sram_trace_buf + trace_hdr.offset,
			    trace_hdr.size);

	return IRQ_WAKE_THREAD;
}

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

static int fw_error_found(void)
{
	union error_log err_log;

	err_log.data = log_buffer[FABRIC_ERR_SIGNATURE_IDX];

	/* No SCU/fabric error if tenth DW signature field is not 10101 */
	if (err_log.fields.signature != FABERR_INDICATOR)
		return 0;
	return 1;
}

static int get_fabric_error_cause_detail(char *buf, u32 size, u32 fabid,
				u32 *fid_status, int ishidword)
{
	int index = 0, ret = 0;
	char *ptr;
	u32 fid_mask = 1;

	while (index < MAX_FID_REG_LEN) {

		if ((*fid_status) & fid_mask) {
			ptr = fabric_error_lookup(fabid, index, ishidword);

			if (ptr && *ptr)
				output_str(ret, buf, size, "%s\n", ptr);
		}

		index++;
		fid_mask <<= 1;
	}

	output_str(ret, buf, size, "\n");

	return ret;
}

static int get_additional_error(char *buf, int size, int num_err_log,
			u32 *faberr_dwords, int max_dw_left)
{
	int i = 0, ret = 0;
	union error_log log;

	output_str(ret, buf, size,
		   "\nAdditional logs associated with error(s): ");

	if (num_err_log) {

		while (i < num_err_log && i < max_dw_left) {

			output_str(ret, buf, size, "\nerror_log: 0x%X\n",
				*(faberr_dwords + i));

			output_str(ret, buf, size, "error_addr: 0x%X\n",
				   *(faberr_dwords + i + 1));

			log.data = *(faberr_dwords + i);

			output_str(ret, buf, size,
				   "\nDecoded error log detail\n");
			output_str(ret, buf, size,
				   "---------------------------\n\n");

			output_str(ret, buf, size, "Agent Index:");
			if (log.fields.agent_idx > MAX_AGENT_IDX) {
				output_str(ret, buf, size,
					   "Unknown agent index (%d)\n\n",
					   log.fields.agent_idx);
			} else {
				output_str(ret, buf, size, "%s\n\n",
					   agent_names[log.fields.agent_idx]);
			}

			output_str(ret, buf, size,
				   "Cmd initiator ID: %d\n",
				   log.fields.initid);

			output_str(ret, buf, size, "Command: %d\n",
				   log.fields.cmd);

			output_str(ret, buf, size, "Code: %d\n",
				   log.fields.err_code);

			if (log.fields.multi_err)
				output_str(ret, buf, size,
					   "\n Multiple errors detected!\n");
			i += 2; /* Skip one error_log/addr pair */
		}
	} else {
		output_str(ret, buf, size, "Not present\n");
	}

	return ret;
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

	return fabric_names[*fab_id];
}

static void read_fwerr_log(u32 *buf, void __iomem *oshob_ptr)
{
	int count;
	void __iomem *fabric_err_dump_offset = oshob_ptr +
		intel_scu_ipc_get_fabricerror_buf1_offset();

	for (count = 0; count < MAX_NUM_LOGDWORDS; count++)
		buf[count] = readl(fabric_err_dump_offset +
					  count * sizeof(u32));

	/* Get 9 additional DWORDS */
	fabric_err_dump_offset = oshob_ptr +
		intel_scu_ipc_get_fabricerror_buf2_offset();

	for (count = 0; count < MAX_NUM_LOGDWORDS_EXTENDED; count++)
		buf[count + MAX_NUM_LOGDWORDS] =
			readl(fabric_err_dump_offset + sizeof(u32) * count);
}

static int dump_fwerr_log(char *buf, int size)
{
	char *ptr = NULL;
	union error_log err_log;
	union flag_status_hilo flag_status;
	union fabric_status err_status;
	u32 id = FAB_ID_UNKNOWN;
	int count, num_flag_status, num_err_logs;
	int prev_id = FAB_ID_UNKNOWN, offset = 0, ret = 0;

	err_status.data = log_buffer[FABRIC_ERR_STS_IDX];
	err_log.data = log_buffer[FABRIC_ERR_SIGNATURE_IDX];

	/* FW error if tenth DW reserved field is 111 */
	if ((((err_status.data & 0xFFFF) == SWDTERR_IND) ||
		((err_status.data & 0xFFFF) == UNDEFLVL1ERR_IND) ||
		((err_status.data & 0xFFFF) == UNDEFLVL2ERR_IND) ||
		((err_status.data & 0xFFFF) == MEMERR_IND) ||
		((err_status.data & 0xFFFF) == INSTERR_IND) ||
		((err_status.data & 0xFFFF) == ECCERR_IND) ||
		((err_status.data & 0xFFFF) == FATALERR_IND)) &&
		(err_log.fields.fw_err_ind == FWERR_INDICATOR)) {

		output_str(ret, buf, size, "HW WDT expired");

		switch (err_status.data & 0xFFFF) {
		case SWDTERR_IND:
			output_str(ret, buf, size,
				   " without facing any exception.\n\n");
			break;
		case MEMERR_IND:
			output_str(ret, buf, size,
				   " following a Memory Error exception.\n\n");
			break;
		case INSTERR_IND:
			output_str(ret, buf, size,
				   " following an Instruction Error exception.\n\n");
			break;
		case ECCERR_IND:
			output_str(ret, buf, size,
				   " following a SRAM ECC Error exception.\n\n");
			break;
		case FATALERR_IND:
			output_str(ret, buf, size,
				   " following a FATAL Error exception.\n\n");
			break;
		default:
			output_str(ret, buf, size, ".\n\n");
			break;
		}
		output_str(ret, buf, size, "HW WDT debug data:\n");
		output_str(ret, buf, size, "===================\n");
		for (count = 0;
			count < MAX_NUM_LOGDWORDS + MAX_NUM_LOGDWORDS_EXTENDED;
			count++) {
			output_str(ret, buf, size, "DW%d:0x%08x\n",
				   count, log_buffer[count]);
		}
		goto out;
	}

	num_flag_status = err_status.fields.flag_status_cnt;
	/* num_err_logs indicates num of error_log/addr pairs */
	num_err_logs = err_log.fields.num_err_logs * 2;

	output_str(ret, buf, size,
		   "HW WDT fired following a Fabric Error exception.\n\n");
	output_str(ret, buf, size, "Fabric Error debug data:\n");
	output_str(ret, buf, size, "===================\n");

	for (count = 0; count < num_flag_status; count++) {

		err_status.data = log_buffer[count];
		ptr = get_fabric_name(
			err_status.fields.regidx & FAB_ID_MASK, &id);

		/*
		 * Only print the fabric name if is unknown
		 * type, or we haven't printed out it yet.
		 */

		if (prev_id != id || id == FAB_ID_UNKNOWN) {
			output_str(ret, buf, size, "%s", ptr);
			prev_id = id;
		}

		flag_status.data = 0;
		flag_status.fields.bits_rang0 =
				err_status.fields.status_has_hilo;

		flag_status.fields.bits_rang1 =
				err_status.fields.status_has_hilo1;

		/*
		 * The most significant bit in REGIDX field is set
		 * the flag_status has the high 32bit of the dword
		 * otherwise the flag_status has the low 32bit of
		 * the dword
		 */

		if (err_status.fields.regidx & FLAG_HILOW_MASK)
			ret += get_fabric_error_cause_detail(buf + ret,
							     size - ret,
							     id,
							     &flag_status.data,
							     1);
		else
			ret += get_fabric_error_cause_detail(buf + ret,
							     size - ret,
							     id,
							     &flag_status.data,
							     0);

		offset++; /* Use this to track error_log/address offset */
	}

	if (offset & 1)
		offset++; /* If offset is odd number, adjust to even offset */

	ret += get_additional_error(buf + ret, size - ret, num_err_logs,
				    &log_buffer[offset],
				    MAX_NUM_LOGDWORDS - offset);

	output_str(ret, buf, size, "\n\n\nAdditional debug data:\n\n");
	for (count = 0;
		count < MAX_NUM_LOGDWORDS + MAX_NUM_LOGDWORDS_EXTENDED;
		count++) {
		output_str(ret, buf, size, "DW%d:0x%08x\n",
			   count, log_buffer[count]);
	}
out:
	return ret;
}

static int dump_scu_extented_trace(char *buf, int size, int log_offset,
				   int *read)
{
	int ret = 0;
	int i;
	unsigned int end, start;

	*read = 0;

	/* Title for error dump */
	if (!log_offset)
		output_str(ret, buf, size, "SCU Extra trace\n");

	start = log_offset / sizeof(u32);
	end = log_buffer_sz / sizeof(u32);
	for (i = start; i < end; i++) {
		/*
		 * "EW:" to separate lines from "DW:" lines
		 * elsewhere in this file.
		 */
		output_str(ret, buf, size, "EW%d:0x%08x\n", i,
			   *(log_buffer + i));
		*read += sizeof(u32);
		/* Make sure we get only full lines */
		if (buf && (size - ret < 18))
			break;
	}
	return ret;
}

#ifdef CONFIG_PROC_FS
struct proc_dir_entry *ipanic_faberr;
static int intel_fw_logging_proc_read(char *buffer, char **start, off_t offset,
					int count, int *peof, void *data)
{
	int ret;
	u32 read;

	if (!offset) {
		/* Fill the buffer, return the buffer size */
		ret = dump_fwerr_log(buffer, count);
		read = MAX_NUM_ALL_LOGDWORDS * sizeof(u32);
	} else {
		ret = dump_scu_extented_trace(buffer, count, offset, &read);
		if (!read || offset + read > sram_buf_sz)
			*peof = 1;
	}
	*start = (void *) read;

	return ret;
}
#endif /* CONFIG_PROC_FS */

static int fw_logging_crash_on_boot(void)
{
	int length = 0;
	int err = 0;
	u32 read;

	log_buffer_sz = MAX_NUM_ALL_LOGDWORDS * sizeof(u32) + sram_buf_sz;
	log_buffer = kzalloc(log_buffer_sz, GFP_KERNEL);
	if (!log_buffer) {
		pr_err("Failed to allocate memory for log buffer");
		err = -ENOMEM;
		goto err1;
	}

	read_fwerr_log(log_buffer, oshob_base);
	if (!fw_error_found()) {
		pr_info("No stored SCU errors found in SRAM");
		kfree(log_buffer);
		goto out;
	}
	length = dump_fwerr_log(NULL, 0);

	if (sram_trace_buf) {
		/*
		 * SCU gives pointer via oshob. Address is a physical
		 * address somewhere in shared sram
		 */
		read_sram_trace_buf(log_buffer + MAX_NUM_ALL_LOGDWORDS,
				    sram_trace_buf, sram_buf_sz);
		length += dump_scu_extented_trace(NULL, 0, 0, &read);
	}

#ifdef CONFIG_PROC_FS

	ipanic_faberr = create_proc_entry("ipanic_fabric_err",
					  S_IFREG | S_IRUGO, NULL);
	if (ipanic_faberr == 0) {
		pr_err("Fail creating procfile ipanic_fabric_err\n");
		err = -ENODEV;
		goto err2;
	}

	ipanic_faberr->read_proc = intel_fw_logging_proc_read;
	ipanic_faberr->write_proc = NULL;
	ipanic_faberr->size = length;
#endif /* CONFIG_PROC_FS */

out:
	return err;

err2:
	kfree(log_buffer);
err1:
	return err;
}

static int intel_fw_logging_panic_handler(struct notifier_block *this,
					  unsigned long event, void *unused)
{
	unsigned int timeout = 0, count;
	int i;

	apic_scu_panic_dump();

	do {
		mdelay(SCU_PANIC_DUMP_TOUT);
		read_scu_trace_hdr(&trace_hdr);
	} while (trace_hdr.magic != TRACE_MAGIC &&
		 timeout++ < SCU_PANIC_DUMP_RECHECK);

	if (timeout > SCU_PANIC_DUMP_RECHECK) {
		pr_info("Waiting for trace from SCU timed out!");
		goto out;
	}

	pr_info("SCU trace on Kernel panic:");

	count = sram_buf_sz / sizeof(u32);
	for (i = 0; i < count; i += DWORDS_PER_LINE) {
		/* EW111:0xdeadcafe EW112:0xdeadcafe \0 */
		char dword_line[DWORDS_PER_LINE * 17 + 1] = {0};
		/* abcdefgh\0 */
		char ascii_line[DWORDS_PER_LINE * sizeof(u32) + 1] = {0};
		int ascii_offset = 0, dword_offset = 0, j;

		for (j = 0; i + j < count && j < DWORDS_PER_LINE; j++) {
			int k;
			u32 dword = readl(sram_trace_buf + (i + j) *
					  sizeof(u32));
			char *c = (char *) &dword;

			dword_offset = sprintf(dword_line + dword_offset,
					       "EW%d:0x%08x ", i + j, dword);
			for (k = 0; k < sizeof(dword); k++)
				if (isascii(*(c + k)) && isalnum(*(c + k)) &&
				    *(c + k) != 0)
					ascii_line[ascii_offset++] = *(c + k);
				else
					ascii_line[ascii_offset++] = '.';
		}
		ascii_line[ascii_offset++] = '\0';

		pr_info("%s %s", dword_line, ascii_line);
	}

out:
	return 0;
}

static struct notifier_block fw_logging_panic_notifier = {
	.notifier_call	= intel_fw_logging_panic_handler,
	.next		= NULL,
	.priority	= INT_MAX
};

static int intel_fw_logging_probe(struct platform_device *pdev)
{
	int err = 0;

	if (!sram_trace_buf) {
		pr_err("No sram trace buffer available, skip SCU tracing init");
		err = -ENODEV;
		goto err1;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_info("No irq available, SCU tracing not available");
		err = irq;
		goto err1;
	}

	err = atomic_notifier_chain_register(
		&panic_notifier_list,
		&fw_logging_panic_notifier);
	if (err) {
		pr_err("Failed to register notifier!");
		goto err1;
	}
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_info("No irq available, SCU tracing not available");
		err = irq;
		goto err2;
	}

	err = request_threaded_irq(irq, fw_logging_irq,
				   fw_logging_irq_thread,
				   IRQF_ONESHOT, "fw_logging",
				   &pdev->dev);
	if (err) {
		pr_err("Requesting irq failed");
		goto err2;
	}
	return err;
err2:
	atomic_notifier_chain_unregister(&panic_notifier_list,
					 &fw_logging_panic_notifier);
err1:
	return err;
}

static int intel_fw_logging_remove(struct platform_device *pdev)
{
	free_irq(irq, &pdev->dev);
	return atomic_notifier_chain_unregister(&panic_notifier_list,
						&fw_logging_panic_notifier);
}

static const struct platform_device_id intel_fw_logging_table[] = {
	{"scuLog", 1 },
};

static struct platform_driver intel_fw_logging_driver = {
	.driver = {
		.name = "scuLog",
		.owner = THIS_MODULE,
		},
	.probe = intel_fw_logging_probe,
	.remove = __devexit_p(intel_fw_logging_remove),
	.id_table = intel_fw_logging_table,
};

static int intel_fw_logging_init(void)
{
	u32 buffer;
	int err = 0;

	oshob_base = get_oshob_addr();
	if (oshob_base == NULL) {
		pr_err("Failed to get OSHOB address");
		err = -EINVAL;
		goto err1;
	}

	buffer = intel_scu_ipc_get_scu_trace_buffer();
	if (buffer && buffer >= LOWEST_PHYS_SRAM_ADDRESS) {
		/*
		 * Calculate size of SCU extra trace buffer. Size of the buffer
		 * is given by SCU. Make sanity check in case of incorrect data.
		 */
		sram_buf_sz = intel_scu_ipc_get_scu_trace_buffer_size();
		if (sram_buf_sz > MAX_SCU_EXTRA_DUMP_SIZE) {
			pr_err("Failed to get scu trace buffer size");
			err = -ENODEV;
			goto err1;
		}
		/* Looks that we have valid buffer and size. */
		sram_trace_buf = ioremap_nocache(buffer, sram_buf_sz);
		if (!sram_trace_buf) {
			pr_err("Failed to map scu trace buffer");
			err =  -ENOMEM;
			goto err1;
		}

		trace_buf = kzalloc(sram_buf_sz, GFP_KERNEL);
		if (!trace_buf) {
			pr_err("Failed to allocate memory for trace buffer");
			err = -ENOMEM;
			goto err2;
		}
	} else {
		pr_info("No extended trace buffer available");
	}

	fabric_err_buf1 = oshob_base +
		intel_scu_ipc_get_fabricerror_buf1_offset();
	fabric_err_buf2 = oshob_base +
		intel_scu_ipc_get_fabricerror_buf2_offset();

	/* Check and report existing error logs */
	err = fw_logging_crash_on_boot();
	if (err) {
		pr_err("Logging SCU errors stored in SRAM failed");
		goto err3;
	}

	/* Clear fabric error region inside OSHOB if neccessary */
	rpmsg_send_simple_command(fw_logging_instance,
				  IPCMSG_CLEAR_FABERROR, 0);

	err = platform_driver_register(&intel_fw_logging_driver);
	if (err) {
		pr_err("Failed to register platform driver\n");
		goto err3;
	}
	return err;
err3:
	kfree(trace_buf);
err2:
	iounmap(sram_trace_buf);
err1:
	return err;
}

static void intel_fw_logging_exit(void)
{
	platform_driver_unregister(&intel_fw_logging_driver);
	kfree(trace_buf);
	iounmap(sram_trace_buf);
#ifdef CONFIG_PROC_FS
	if (ipanic_faberr)
		remove_proc_entry("ipanic_fabric_err", NULL);
#endif /* CONFIG_PROC_FS */
	kfree(log_buffer);
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
