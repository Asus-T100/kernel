/*
 * hsi_dlp_debug.h
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
 * Contact: Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>
 *          Faouaz Tenoutit <faouazx.tenoutit@intel.com>
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

#ifndef _HSI_DLP_DEBUG_H_
#define _HSI_DLP_DEBUG_H_

/******************************************************************************
 * To enable debug:
 *
 *	- define the CONFIG_DEBUG debug macro (kernel compile)
 *
 *	- define a DEBUG_VAR : Variable to dynamically filter tag/level
 *							 (typically a module parameter)
 *
 *	- define a DEBUG_TAG : Tag to debug (typically 1 by .c file)
 *
 *  - Activate the debug by:
 *		echo 0xTAGLEVEL > /sys/module/dlp_lte/parameters/debug
 *
 * LEVEL is defined on 12 bits:
 *		0x000  => nothing, except CRITICAL, WARNING which are always displayed
 *		0x001  => usefull light traces (PRINT, PTRACE)
 *		0x002  => prolog & epilog (functions in/out)
 *		0x004  => Dump TX PDUs
 *		0x008  => Dump RX PDUs
 *
 *		0xFFF => deep debug (everything)
 *
 * LEVEL can be mixed together, for example:
 *		0xFFF => Debug everything (All traces)
 *		0x3   => prolog/epilog + traces
 *
 * TAG:
 *		0x1 => COMM (to debug the common related part)
 *		0x2 => CTRL (to debug the ctrl related part)
 *		0x4 => TTY  (to debug the tty related part)
 *		0x8 => NET  (to debug the netif related part)
 *
 *	Exp: to debug everything (all TAGs & all LEVELs) :
 *		echo 0xFFFF > /sys/module/dlp_lte/parameters/debug
 *
 *  Exp: to debug the TTY functions entry/exit:
 *		echo 0x4002 > /sys/module/dlp_lte/parameters/debug
 *
 *	Exp: to deactivate the debug :
 *		echo 0x0 > /sys/module/dlp_lte/parameters/debug
 *
 *****************************************************************************/
#include <linux/hsi/hsi.h>

#define DEBUG_NONE		0x000
#define DEBUG_TRACES	0x001
#define DEBUG_PROLOG	0x002
#define DEBUG_TX_PDU	0x004
#define DEBUG_RX_PDU	0x008
#define DEBUG_ALL		0xFFF

#define DEBUG_LEVEL_SZ	12
#define DEBUG_CURR_TAG	(DEBUG_TAG << DEBUG_LEVEL_SZ)

#ifdef DEBUG
	#define PRINT(fmt, args...)	printk (fmt, ## args)
#else
	#define PRINT(fmt, args...) /* nothing, its a placeholder */
#endif

#define CRITICAL(fmt, args...)	printk("! %s(" fmt ")\n", __func__, ## args)
#define WARNING(fmt, args...)	printk("? %s(" fmt ")\n", __func__, ## args)

#define PTRACE(fmt, args...)									\
    do															\
        if (DEBUG_VAR & (DEBUG_TRACES | DEBUG_CURR_TAG))		\
            PRINT (" %s(" fmt ")\n", __func__, ## args);		\
    while (0)

#define PTRACE_NO_FUNC(fmt, args...)							\
    do															\
        if (DEBUG_VAR & (DEBUG_TRACES | DEBUG_CURR_TAG))		\
            PRINT (fmt, ## args);								\
    while (0)

#define PRINT_TX(fmt, args...)									\
    do															\
        if (DEBUG_VAR & (DEBUG_TX_PDU | DEBUG_CURR_TAG))		\
            PRINT (fmt, ## args);								\
    while (0)

#define PRINT_RX(fmt, args...)									\
    do															\
        if (DEBUG_VAR & (DEBUG_RX_PDU | DEBUG_CURR_TAG))		\
            PRINT (fmt, ## args);								\
    while (0)

#define PROLOG(fmt, args...)									\
    do															\
        if (DEBUG_VAR & (DEBUG_PROLOG | DEBUG_CURR_TAG))		\
            PRINT ("> %s(" fmt ")\n", __func__, ## args);		\
    while (0)

#define EPILOG(fmt, args...)									\
    do															\
        if (DEBUG_VAR & (DEBUG_PROLOG | DEBUG_CURR_TAG))		\
            PRINT ("< %s(" fmt ")\n", __func__, ## args);		\
    while (0)

#define PDEBUG(fmt, args...)									\
    do															\
        if (DEBUG_VAR & (DEBUG_ALL | DEBUG_CURR_TAG))			\
            PRINT (" %s(" fmt ")\n", __func__, ## args);		\
    while (0)



#define HSI_MSG_STATUS_TO_STR(status)									\
	(status == HSI_STATUS_COMPLETED		? "COMPLETED ":					\
	(status == HSI_STATUS_PENDING		? "PENDING   ":					\
	(status == HSI_STATUS_PROCEEDING)	? "PROCEEDING":					\
	(status == HSI_STATUS_QUEUED)		? "QUEUED    ":					\
	(status == HSI_STATUS_ERROR)		? "ERROR     " : "UNKNOWN   "))


/*
 *
 *
 */
void dlp_dbg_dump_data_as_word(unsigned int *data,
								int nb_words,
								int words_per_line,
								int is_corrupted,
								int is_rx);

void dlp_dbg_dump_data_as_byte(unsigned char *data,
								int nb_bytes,
								int bytes_per_line,
								int is_corrupted,
								int is_rx);

void dlp_dbg_dump_pdu(struct hsi_msg *pdu,
						int items_per_line,
						int items_to_dump,
						int dump_as_words);

#endif /* _HSI_DLP_DEBUG_H_ */

