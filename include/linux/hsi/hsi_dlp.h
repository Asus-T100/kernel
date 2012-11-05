/*
 * hsi_dlp.h
 *
 * Intel Mobile Communication modem protocol driver for DLP
 * (Data Link Protocl (LTE)).
 * This driver is implementing a 5-channel HSI protocol consisting of:
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
#ifndef _HSI_DLP_H
#define _HSI_DLP_H

#include <linux/ioctl.h>

/* reasons for hanging up */
enum {
	DLP_MODEM_HU_TIMEOUT	= 1,
	DLP_MODEM_HU_RESET		= 2,
	DLP_MODEM_HU_COREDUMP	= 4,
};

/**
 * struct hsi_dlp_stats - statistics related to the TX and RX side
 * @data_sz: total size of actual transferred data
 * @pdus_cnt: total number of transferred puds
 * @overflow_cnt: total number of transfer stalls due to FIFO full
 */
struct hsi_dlp_stats {
	unsigned long long	data_sz;
	unsigned int		pdus_cnt;
	unsigned int		overflow_cnt;
};

#define HSI_DLP_MAGIC	0x77

/*
 * HSI_DLP_RESET_TX	-	 reset the TX state machine (flushes it)
 */
#define HSI_DLP_RESET_TX		_IO(HSI_DLP_MAGIC, 0)

/*
 * HSI_DLP_RESET_RX	-	 reset the RX state machine (flushes it)
 */
#define HSI_DLP_RESET_RX		_IO(HSI_DLP_MAGIC, 1)

/*
 * HSI_DLP_GET_TX_STATE	-	 get the current state of the TX state machine
 */
#define HSI_DLP_GET_TX_STATE		_IOR(HSI_DLP_MAGIC, 2, unsigned int)

/*
 * HSI_DLP_GET_RX_STATE	-	 get the current state of the RX state machine
 */
#define HSI_DLP_GET_RX_STATE		_IOR(HSI_DLP_MAGIC, 3, unsigned int)

/*
 * HSI_DLP_MODEM_RESET		- reset the modem (solicited reset)
 * Shared with SPI
 */
#define HSI_DLP_MODEM_RESET		_IO(HSI_DLP_MAGIC, 4)

/*
 * HSI_DLP_MODEM_STATE		- return 1 if first transmission completed
 * Shared with SPI
 */
#define HSI_DLP_MODEM_STATE		_IOR(HSI_DLP_MAGIC, 5, int)

/*
 * HSI_DLP_GET_HANGUP_REASON	- return reason for latest hangup
 * Shared with SPI
 */
#define HSI_DLP_GET_HANGUP_REASON	_IOR(HSI_DLP_MAGIC, 6, int)

/*
 * HSI_DLP_GET_TX_WAIT_MAX	- get the maximal size of the TX waiting FIFO
 */
#define HSI_DLP_GET_TX_WAIT_MAX		_IOR(HSI_DLP_MAGIC, 8, unsigned int)

/*
 * HSI_DLP_GET_RX_WAIT_MAX	- get the maximal size of the RX waiting FIFO
 */
#define HSI_DLP_GET_RX_WAIT_MAX		_IOR(HSI_DLP_MAGIC, 9, unsigned int)

/*
 * HSI_DLP_GET_TX_CTRL_MAX	- get the maximal size of the TX controller FIFO
 */
#define HSI_DLP_GET_TX_CTRL_MAX		_IOR(HSI_DLP_MAGIC, 10, unsigned int)

/*
 * HSI_DLP_GET_RX_CTRL_MAX	- get the maximal size of the RX controller FIFO
 */
#define HSI_DLP_GET_RX_CTRL_MAX		_IOR(HSI_DLP_MAGIC, 11, unsigned int)

/*
 * HSI_DLP_SET_TX_DELAY		- set the TX delay in us
 */
#define HSI_DLP_SET_TX_DELAY		_IOW(HSI_DLP_MAGIC, 12, unsigned int)

/*
 * HSI_DLP_GET_TX_DELAY		- get the TX delay in us
 */
#define HSI_DLP_GET_TX_DELAY		_IOR(HSI_DLP_MAGIC, 12, unsigned int)

/*
 * HSI_DLP_SET_RX_DELAY		- set the RX delay in us
 */
#define HSI_DLP_SET_RX_DELAY		_IOW(HSI_DLP_MAGIC, 13, unsigned int)

/*
 * HSI_DLP_GET_RX_DELAY		- get the RX delay in us
 */
#define HSI_DLP_GET_RX_DELAY		_IOR(HSI_DLP_MAGIC, 13, unsigned int)

/*
 * HSI_DLP_SET_TX_FLOW		- set the TX flow type (PIPE, SYNC)
 */
#define HSI_DLP_SET_TX_FLOW		_IOW(HSI_DLP_MAGIC, 16, unsigned int)

/*
 * HSI_DLP_GET_TX_FLOW		- get the TX flow type (PIPE, SYNC)
 */
#define HSI_DLP_GET_TX_FLOW		_IOR(HSI_DLP_MAGIC, 16, unsigned int)

/*
 * HSI_DLP_SET_RX_FLOW		- set the RX flow type (PIPE, SYNC)
 */
#define HSI_DLP_SET_RX_FLOW		_IOW(HSI_DLP_MAGIC, 17, unsigned int)

/*
 * HSI_DLP_GET_RX_FLOW		- get the RX flow type (PIPE, SYNC)
 */
#define HSI_DLP_GET_RX_FLOW		_IOR(HSI_DLP_MAGIC, 17, unsigned int)

/*
 * HSI_DLP_SET_TX_MODE		- set the TX mode type (FRAME, STREAM)
 */
#define HSI_DLP_SET_TX_MODE		_IOW(HSI_DLP_MAGIC, 18, unsigned int)

/*
 * HSI_DLP_GET_TX_MODE		- get the TX mode type (FRAME, STREAM)
 */
#define HSI_DLP_GET_TX_MODE		_IOR(HSI_DLP_MAGIC, 18, unsigned int)

/*
 * HSI_DLP_SET_RX_MODE		- set the RX mode type (FRAME, STREAM)
 */
#define HSI_DLP_SET_RX_MODE		_IOW(HSI_DLP_MAGIC, 19, unsigned int)

/*
 * HSI_DLP_GET_RX_MODE		- get the RX mode type (FRAME, STREAM)
 */
#define HSI_DLP_GET_RX_MODE		_IOR(HSI_DLP_MAGIC, 19, unsigned int)

/*
 * HSI_DLP_SET_TX_PDU_LEN	- set the FFL TX frame length
 */
#define HSI_DLP_SET_TX_PDU_LEN	_IOW(HSI_DLP_MAGIC, 24, unsigned int)

/*
 * HSI_DLP_GET_TX_PDU_LEN	- get the FFL TX frame length
 */
#define HSI_DLP_GET_TX_PDU_LEN	_IOR(HSI_DLP_MAGIC, 24, unsigned int)

/*
 * HSI_DLP_SET_RX_PDU_LEN	- set the FFL RX frame length
 */
#define HSI_DLP_SET_RX_PDU_LEN	_IOW(HSI_DLP_MAGIC, 25, unsigned int)

/*
 * HSI_DLP_GET_RX_PDU_LEN	- get the FFL RX frame length
 */
#define HSI_DLP_GET_RX_PDU_LEN	_IOR(HSI_DLP_MAGIC, 25, unsigned int)

/*
 * HSI_DLP_SET_TX_ARB_MODE	- set the FFL TX arbitration (RR ou priority)
 */
#define HSI_DLP_SET_TX_ARB_MODE		_IOW(HSI_DLP_MAGIC, 28, unsigned int)

/*
 * HSI_DLP_GET_TX_ARB_MODE	- get the FFL TX arbitration (RR or priority)
 */
#define HSI_DLP_GET_TX_ARB_MODE		_IOR(HSI_DLP_MAGIC, 28, unsigned int)

/*
 * HSI_DLP_SET_TX_FREQUENCY	- set the maximum FFL TX frequency (in kbit/s)
 */
#define HSI_DLP_SET_TX_FREQUENCY	_IOW(HSI_DLP_MAGIC, 30, unsigned int)

/*
 * HSI_DLP_GET_TX_FREQUENCY	- get the maximum FFL TX frequency (in kbit/s)
 */
#define HSI_DLP_GET_TX_FREQUENCY	_IOR(HSI_DLP_MAGIC, 30, unsigned int)

/*
 * HSI_DLP_RESET_TX_STATS	- reset the TX statistics
 */
#define HSI_DLP_RESET_TX_STATS		_IO(HSI_DLP_MAGIC, 32)

/*
 * HSI_DLP_GET_TX_STATS		- get the TX statistics
 */
#define HSI_DLP_GET_TX_STATS		_IOR(HSI_DLP_MAGIC, 32, \
					     struct hsi_dlp_stats)

/*
 * HSI_DLP_RESET_RX_STATS	- reset the RX statistics
 */
#define HSI_DLP_RESET_RX_STATS		_IO(HSI_DLP_MAGIC, 33)

/*
 * HSI_DLP_GET_RX_STATS		- get the RX statistics
 */
#define HSI_DLP_GET_RX_STATS		_IOR(HSI_DLP_MAGIC, 33, \
					     struct hsi_dlp_stats)

/*
 * HSI_DLP_NET_RESET_RX_STATS	- reset the network interface RX statistics
 */
#define HSI_DLP_NET_RESET_RX_STATS		_IOW(HSI_DLP_MAGIC, 40, unsigned int)

/*
 * HSI_DLP_NET_RESET_TX_STATS	- reset the network interface TX statistics
 */
#define HSI_DLP_NET_RESET_TX_STATS		_IOW(HSI_DLP_MAGIC, 41, unsigned int)

/*
 * HSI_DLP_SET_FLASHING_MODE	- Activate/Deactivate the flashing mode
 */
#define HSI_DLP_SET_FLASHING_MODE	_IOW(HSI_DLP_MAGIC, 42, unsigned int)

#endif /* _HSI_DLP_H */

