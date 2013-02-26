/*
 *  Copyright (C) Intel 2010
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
 */

#ifndef MID_SSP_TEST_USER_H_
#define MID_SSP_TEST_USER_H_

#include <linux/ioctl.h>
#include <linux/intel_mid_i2s_common.h>

#define SSP_DRV_BASE 'x'
#define SSP_GET_SETTINGS _IOR(SSP_DRV_BASE, 0x00, struct intel_mid_i2s_settings)
#define SSP_SET_MODE _IOW(SSP_DRV_BASE, 0x01, struct intel_mid_i2s_settings)
#define SSP_SET_RX_FIFO_INTERRUPT _IOW(SSP_DRV_BASE, 0x02, struct intel_mid_i2s_settings)
#define SSP_SET_TX_FIFO_INTERRUPT _IOW(SSP_DRV_BASE, 0x03, struct intel_mid_i2s_settings)
#define SSP_SET_FRAME_FORMAT _IOW(SSP_DRV_BASE, 0x04, struct intel_mid_i2s_settings)
#define SSP_SET_MASTER_MODE_CLK_SELECTION _IOW(SSP_DRV_BASE, 0x05, struct intel_mid_i2s_settings)
#define SSP_SET_FRAME_RATE_DIVIDER_CONTROL _IOW(SSP_DRV_BASE, 0x06, struct intel_mid_i2s_settings)
#define SSP_SET_MASTER_MODE_STANDARD_FREQ _IOW(SSP_DRV_BASE, 0x07, struct intel_mid_i2s_settings)
#define SSP_SET_DATA_SIZE _IOW(SSP_DRV_BASE, 0x08, struct intel_mid_i2s_settings)
#define SSP_SET_TX_TRISTATE_PHASE _IOW(SSP_DRV_BASE, 0x09, struct intel_mid_i2s_settings)
#define SSP_SET_TX_TRISTATE_ENABLE _IOW(SSP_DRV_BASE, 0x0A, struct intel_mid_i2s_settings)
#define SSP_SET_SLAVE_CLK_FREE_RUNNING_STATUS _IOW(SSP_DRV_BASE, 0x0B, struct intel_mid_i2s_settings)
#define SSP_SET_SSPSLCLK_DIRECTION _IOW(SSP_DRV_BASE, 0x0C, struct intel_mid_i2s_settings)
#define SSP_SET_SSPSFRM_DIRECTION _IOW(SSP_DRV_BASE, 0x0D, struct intel_mid_i2s_settings)
#define SSP_SET_DUPLEX_MODE _IOW(SSP_DRV_BASE, 0x0E, struct intel_mid_i2s_settings)
#define SSP_SET_TRAILING_BYTE_MODE _IOW(SSP_DRV_BASE, 0x0F, struct intel_mid_i2s_settings)
#define SSP_SET_TX_DMA _IOW(SSP_DRV_BASE, 0x10, struct intel_mid_i2s_settings)
#define SSP_SET_RX_DMA _IOW(SSP_DRV_BASE, 0x11, struct intel_mid_i2s_settings)
#define SSP_SET_RX_TIMEOUT_INTERRUPT_STATUS _IOW(SSP_DRV_BASE, 0x12, struct intel_mid_i2s_settings)
#define SSP_SET_TRAILING_BYTE_INTERRUPT_STATUS _IOW(SSP_DRV_BASE, 0x13, struct intel_mid_i2s_settings)
#define SSP_SET_LOOPBACK_MODE_STATUS _IOW(SSP_DRV_BASE, 0x14, struct intel_mid_i2s_settings)
#define SSP_SET_RX_FIFO_THRESHOLD _IOW(SSP_DRV_BASE, 0x15, struct intel_mid_i2s_settings)
#define SSP_SET_TX_FIFO_THRESHOLD _IOW(SSP_DRV_BASE, 0x16, struct intel_mid_i2s_settings)
#define SSP_SET_FRMSYNC_TIMING_BIT _IOW(SSP_DRV_BASE, 0x17, struct intel_mid_i2s_settings)
#define SSP_SET_FRMSYNC_POL_BIT _IOW(SSP_DRV_BASE, 0x18, struct intel_mid_i2s_settings)
#define SSP_SET_END_TRANSFER_STATE _IOW(SSP_DRV_BASE, 0x19, struct intel_mid_i2s_settings)
#define SSP_SET_SERIAL_CLK_MODE _IOW(SSP_DRV_BASE, 0x1A, struct intel_mid_i2s_settings)
#define SSP_SET_PSP_T1 _IOW(SSP_DRV_BASE, 0x1B, struct intel_mid_i2s_settings)
#define SSP_SET_PSP_T2 _IOW(SSP_DRV_BASE, 0x1C, struct intel_mid_i2s_settings)
#define SSP_SET_PSP_T4 _IOW(SSP_DRV_BASE, 0x1D, struct intel_mid_i2s_settings)
#define SSP_SET_PSP_T5 _IOW(SSP_DRV_BASE, 0x1E, struct intel_mid_i2s_settings)
#define SSP_SET_PSP_T6 _IOW(SSP_DRV_BASE, 0x1F, struct intel_mid_i2s_settings)
#define SSP_SET_ACTIVE_TX_SLOTS_MAP _IOW(SSP_DRV_BASE, 0x20, struct intel_mid_i2s_settings)
#define SSP_SET_ACTIVE_RX_SLOTS_MAP _IOW(SSP_DRV_BASE, 0x21, struct intel_mid_i2s_settings)
#define SSP_SELECT_BLUETOOTH_USAGE _IOW(SSP_DRV_BASE, 0x22, struct intel_mid_i2s_settings)
#define SSP_SELECT_MODEM_USAGE _IOW(SSP_DRV_BASE, 0x23, struct intel_mid_i2s_settings)

#endif /* MID_SSP_TEST_USER_H_ */
