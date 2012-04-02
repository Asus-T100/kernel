/*
 * dlp_debug.c
 *
 * Intel Mobile Communication modem protocol driver for DLP
 * (Data Link Protocl (LTE)). This driver is implementing a 5-channel HSI
 * protocol consisting of:
 * - An internal communication control channel;
 * - A multiplexed channel exporting a TTY interface;
 * - Three dedicated high speed channels exporting each a network interface.
 * All channels are using fixed-length pdus, although of different sizes.
 *
 * Copyright (C) 2010-2011 Intel Corporation. All rights reserved.
 *
 * Contact:  Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>

#include "../dlp_debug.h"
#include "dlp_main.h"

#define DEBUG_TAG 0x1
#define DEBUG_VAR dlp_drv.debug

/*
 *
 *
 */
void dlp_dbg_dump_data_as_byte(unsigned char *data,
			       int nb_bytes,
			       int bytes_per_line, int should_dump, int is_rx)
{
	int i;

	if (should_dump)
		printk(KERN_DEBUG "-----\n");
	else if (is_rx)
		PRINT_RX("-----\n");
	else
		PRINT_TX("-----\n");

	for (i = 0; i < nb_bytes; i++) {
		if (should_dump)
			printk(KERN_DEBUG "%02X ", data[i]);
		else if (is_rx)
			PRINT_RX("%02X ", data[i]);
		else
			PRINT_TX("%02X ", data[i]);

		/* Print requested items/line */
		if (((i + 1) & (bytes_per_line - 1)) == 0) {
			if (should_dump)
				printk("\n");
			else if (is_rx)
				PRINT_RX("\n");
			else
				PRINT_TX("\n");
		}
	}

	/* Previous line was less than "bytes_per_line" */
	if ((i & (bytes_per_line - 1)) != 0) {
		if (should_dump)
			printk(KERN_DEBUG "\n");
		else if (is_rx)
			PRINT_RX("\n");
		else
			PRINT_TX("\n");
	}

	if (should_dump)
		printk(KERN_DEBUG "-----\n");
	else if (is_rx)
		PRINT_RX("-----\n");
	else
		PRINT_TX("-----\n");
}

void dlp_dbg_dump_data_as_word(unsigned int *data,
			       int nb_words,
			       int words_per_line, int should_dump, int is_rx)
{
	int i;

	if (should_dump)
		printk(KERN_DEBUG "-----\n");
	else if (is_rx)
		PRINT_RX("-----\n");
	else
		PRINT_TX("-----\n");

	for (i = 0; i < nb_words; i++) {
		if (should_dump)
			printk(KERN_DEBUG "%08X ", data[i]);
		else if (is_rx)
			PRINT_RX("%08X ", data[i]);
		else
			PRINT_TX("%08X ", data[i]);

		/* Print requested items/line */
		if (((i + 1) & (words_per_line - 1)) == 0) {
			if (should_dump)
				printk("\n");
			else if (is_rx)
				PRINT_RX("\n");
			else
				PRINT_TX("\n");
		}
	}

	/* Previous line was less than "words_per_line" */
	if ((i & (words_per_line - 1)) != 0) {
		if (should_dump)
			printk(KERN_DEBUG "\n");
		else if (is_rx)
			PRINT_RX("\n");
		else
			PRINT_TX("\n");
	}

	if (should_dump)
		printk(KERN_DEBUG "-----\n");
	else if (is_rx)
		PRINT_RX("-----\n");
	else
		PRINT_TX("-----\n");
}

/*
* @brief
*
* @param pdu : The pdu to dump
* @param items_per_line : Nb of items (bytes/words) to display/line
* @param items_to_dump  : Nb of items (bytes/words) to dump
* @param dump_as_words  : Dump data as sequence of WORDs instead of BYEs
*/
void dlp_dbg_dump_pdu(struct hsi_msg *pdu,
		      int items_per_line, int items_to_dump, int dump_as_words)
{
	int i;
	struct scatterlist *sg = pdu->sgt.sgl;
	unsigned char *data;
	int len, dumped, is_corrupted, is_rx;

	PROLOG("0x%p [nents: %d, hsi_ch: %d, status: %d, "
	       "actual_len: %d, break_frame: %d",
	       pdu,
	       pdu->sgt.nents,
	       pdu->channel, pdu->status, pdu->actual_len, pdu->break_frame);

	dumped = 0;
	is_corrupted = 0;
	is_rx = (pdu->ttype == HSI_MSG_READ);

	if (is_rx)
		PRINT_RX("RX PDU (0x%p):\n", pdu);
	else
		PRINT_TX("TX PDU (0x%p):\n", pdu);

	/* Check the PDU signature */
	data = sg_virt(sg);
	if (!dlp_pdu_header_valid(pdu))
		is_corrupted = 1;

	for (i = 0; i < pdu->sgt.nents; i++) {
		data = sg_virt(sg);
		len = sg->length;

		/* All requested items are dumped ? */
		if (len < items_to_dump) {
			if (dumped >= items_to_dump) {
				break;
			} else {
				if ((len + dumped) > items_to_dump)
					len = items_to_dump - dumped;
			}
		} else {
			len = items_to_dump - dumped;
		}

		/* Dump current SGL entry */
		if (dump_as_words)
			dlp_dbg_dump_data_as_word((u32 *) data,
						  len,
						  items_per_line,
						  is_corrupted, is_rx);
		else
			dlp_dbg_dump_data_as_byte(data,
						  len,
						  items_per_line,
						  is_corrupted, is_rx);

		/* Move to the next SGL entry */
		sg = sg_next(sg);
		dumped += len;
	}

	EPILOG();
}
