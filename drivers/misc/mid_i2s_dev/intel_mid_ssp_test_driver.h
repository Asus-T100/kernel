/*
 *  Copyright (C) Intel 2010
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */
#ifndef BT_PCM_TEST_SSP1_H_
#define BT_PCM_TEST_SSP1_H_

#define DRIVER_NAME "I2S SSP Test"

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/intel_mid_i2s_common.h>
#include <linux/intel_mid_i2s_if.h>
#include <linux/intel_mid_ssp_test_user.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

static struct intel_mid_i2s_settings i2s_ssp_setting = {
	.mode = SSP_IN_NETWORK_MODE,
	/*
	 * Disable or enable DURING TRANSFERT ONLY: will be disable between
	 * transfers
	 */
	.rx_fifo_interrupt = SSP_RX_FIFO_OVER_INT_ENABLE,
	/*
	 * Disable or enable DURING TRANSFERT ONLY: will be disable between
	 * transfers
	 */
	.tx_fifo_interrupt = SSP_TX_FIFO_UNDER_INT_ENABLE,
	.frame_format = PSP_FORMAT,
	.master_mode_clk_selection = SSP_MASTER_CLOCK_UNDEFINED,
	.frame_rate_divider_control = 1,
	.master_mode_standard_freq = SSP_FRM_FREQ_UNDEFINED,
	.data_size = 16,
	.tx_tristate_phase = TXD_TRISTATE_LAST_PHASE_OFF,
	.tx_tristate_enable = TXD_TRISTATE_OFF,
	.slave_clk_free_running_status = SLAVE_SSPCLK_ON_DURING_TRANSFER_ONLY,
	.sspslclk_direction = SSPSCLK_SLAVE_MODE,
	.sspsfrm_direction = SSPSFRM_SLAVE_MODE,
	.ssp_duplex_mode = RX_AND_TX_MODE,
	.ssp_trailing_byte_mode = SSP_TRAILING_BYTE_HDL_BY_IA,
	/* dynamically set */
	.ssp_tx_dma = SSP_TX_DMA_ENABLE,
	/* dynamically set */
	.ssp_rx_dma = SSP_RX_DMA_ENABLE,
	.ssp_rx_timeout_interrupt_status = SSP_RX_TIMEOUT_INT_DISABLE,
	.ssp_trailing_byte_interrupt_status = SSP_TRAILING_BYTE_INT_ENABLE,
	.ssp_loopback_mode_status = SSP_LOOPBACK_OFF,
	.ssp_rx_fifo_threshold = 8,
	.ssp_tx_fifo_threshold = 7,

	.ssp_frmsync_timing_bit = NEXT_FRMS_ASS_AFTER_END_OF_T4,
	/* set high for TI fix */
	.ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH ,
	.ssp_end_transfer_state = SSP_END_DATA_TRANSFER_STATE_LOW,
	.ssp_serial_clk_mode = SSP_CLK_MODE_1,
	.ssp_psp_T1 = 0,
	.ssp_psp_T2 = 0,
	.ssp_psp_T4 = 0,
	.ssp_psp_T5 = 0,
	.ssp_psp_T6 = 1,
	.ssp_active_tx_slots_map = 0x01,
	.ssp_active_rx_slots_map = 0x01
};

static struct intel_mid_i2s_settings i2s_ssp_setting_modem = {
	.mode = SSP_IN_NETWORK_MODE,
	.rx_fifo_interrupt = SSP_RX_FIFO_OVER_INT_ENABLE,
	.tx_fifo_interrupt = SSP_TX_FIFO_UNDER_INT_ENABLE,
	.frame_format = PSP_FORMAT,
	.master_mode_clk_selection = SSP_MASTER_CLOCK_UNDEFINED,
	.frame_rate_divider_control = 1,
	.master_mode_standard_freq = SSP_FRM_FREQ_UNDEFINED,
	.data_size = 32,
	.tx_tristate_phase = TXD_TRISTATE_LAST_PHASE_OFF,
	.tx_tristate_enable = TXD_TRISTATE_ON,
	.slave_clk_free_running_status = SLAVE_SSPCLK_ON_ALWAYS,
	.sspslclk_direction = SSPSCLK_SLAVE_MODE,
	.sspsfrm_direction = SSPSFRM_SLAVE_MODE,
	/* other duplex_mode value is RX_WITHOUT_TX_MODE */
	.ssp_duplex_mode = RX_AND_TX_MODE,
	.ssp_trailing_byte_mode = SSP_TRAILING_BYTE_HDL_BY_IA,
	.ssp_tx_dma = SSP_TX_DMA_ENABLE,
	.ssp_rx_dma = SSP_RX_DMA_ENABLE,
	.ssp_rx_timeout_interrupt_status = SSP_RX_TIMEOUT_INT_DISABLE,
	.ssp_trailing_byte_interrupt_status =
	SSP_TRAILING_BYTE_INT_DISABLE,
	.ssp_loopback_mode_status = SSP_LOOPBACK_OFF,
	.ssp_rx_fifo_threshold = 8,
	.ssp_tx_fifo_threshold = 7,
	.ssp_frmsync_timing_bit = NEXT_FRMS_ASS_WITH_LSB_PREVIOUS_FRM,
	.ssp_frmsync_pol_bit = SSP_FRMS_ACTIVE_HIGH,
	.ssp_end_transfer_state = SSP_END_DATA_TRANSFER_STATE_LOW,
	.ssp_serial_clk_mode = SSP_CLK_MODE_0,
	.ssp_psp_T1 = 0,
	.ssp_psp_T2 = 0,
	.ssp_psp_T4 = 0,
	.ssp_psp_T5 = 0,
	.ssp_psp_T6 = 1,
	.ssp_active_tx_slots_map = 0x01,
	.ssp_active_rx_slots_map = 0x01
};

#define BUFSIZE_ORDER 1

/* Circular Buffer with absolute indices
 *
 * If indices are used instead of pointers, indices can store
 * read/write counts instead of the offset from start of the buffer.
 * Relative indices are obtained on the fly by division modulo the
 * buffer's length.
 *
 * The advantage is:
 * - No extra variables are needed.
 * The disadvantages are:
 * - Every access needs an additional modulo operation.
 * - If counter wrap is possible, complex logic can be needed if the
 *        buffer's length is not a divisor of the counter's capacity.
 * Both of these disadvantages disappear if the buffer's length is a
 * power of two
 *
 * So let's use a buffer length that is a power of 2 and a divisor of
 * the counter's capacity
 */

/* PCM: 160 samples of 16 bits sampled every 20ms i.e. 320 bytes every 20ms */
#define BT_PCM_SLOT_SIZE 320

/* always takes a power of 2 */
#define BT_PCM_NB_SLOTS    4

#define IDX_NUM_BYTE(NUM_RW) ((NUM_RW % BT_PCM_NB_SLOTS) * BT_PCM_SLOT_SIZE)

struct slots_buf {
	/* buffer: points to the ring buffer, 4 elements of 320 bytes */
	u32 *buffer;
	u32 num_write;
	u32 num_read;
	int dma_running;
	/* wait: queue for the waiting process */
	wait_queue_head_t wait;
	spinlock_t lock;
};


struct ssp_test_driver {
	int opened;
	int written;
	spinlock_t lock;
	struct slots_buf rx;
	struct slots_buf tx;
	struct miscdevice *dev;
	bool   tx_dma_chnl_allocated;
	bool   rx_dma_chnl_allocated;
};

#endif /*BT_PCM_SSP1_H_*/
