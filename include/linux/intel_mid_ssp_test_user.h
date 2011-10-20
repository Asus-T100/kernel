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
#define _THIS_IOR(id) _IOR(SSP_DRV_BASE, id, struct intel_mid_i2s_settings)
#define _THIS_IOW(id) _IOW(SSP_DRV_BASE, id, struct intel_mid_i2s_settings)

#define SSP_GET_SETTINGS _THIS_IOR(0x00)

#define SSP_SET_MODE _THIS_IOW(0x01)
#define SSP_SET_RX_FIFO_INTERRUPT _THIS_IOW(0x02)
#define SSP_SET_TX_FIFO_INTERRUPT _THIS_IOW(0x03)
#define SSP_SET_FRAME_FORMAT _THIS_IOW(0x04)
#define SSP_SET_MASTER_MODE_CLK_SELECTION _THIS_IOW(0x05)
#define SSP_SET_FRAME_RATE_DIVIDER_CONTROL _THIS_IOW(0x06)
#define SSP_SET_MASTER_MODE_STANDARD_FREQ _THIS_IOW(0x07)

#define SSP_SET_DATA_SIZE _THIS_IOW(0x08)

#define SSP_SET_TX_TRISTATE_PHASE _THIS_IOW(0x09)
#define SSP_SET_TX_TRISTATE_ENABLE _THIS_IOW(0x0A)
#define SSP_SET_SLAVE_CLK_FREE_RUNNING_STATUS _THIS_IOW(0x0B)
#define SSP_SET_SSPSLCLK_DIRECTION _THIS_IOW(0x0C)
#define SSP_SET_SSPSFRM_DIRECTION _THIS_IOW(0x0D)
#define SSP_SET_DUPLEX_MODE _THIS_IOW(0x0E)
#define SSP_SET_TRAILING_BYTE_MODE _THIS_IOW(0x0F)
#define SSP_SET_TX_DMA _THIS_IOW(0x10)
#define SSP_SET_RX_DMA _THIS_IOW(0x11)
#define SSP_SET_RX_TIMEOUT_INTERRUPT_STATUS _THIS_IOW(0x12)
#define SSP_SET_TRAILING_BYTE_INTERRUPT_STATUS _THIS_IOW(0x13)
#define SSP_SET_LOOPBACK_MODE_STATUS _THIS_IOW(0x14)
#define SSP_SET_RX_FIFO_THRESHOLD _THIS_IOW(0x15)
#define SSP_SET_TX_FIFO_THRESHOLD _THIS_IOW(0x16)

#define SSP_SET_FRMSYNC_TIMING_BIT _THIS_IOW(0x17)
#define SSP_SET_FRMSYNC_POL_BIT _THIS_IOW(0x18)
#define SSP_SET_END_TRANSFER_STATE _THIS_IOW(0x19)
#define SSP_SET_SERIAL_CLK_MODE _THIS_IOW(0x1A)
#define SSP_SET_PSP_T1 _THIS_IOW(0x1B)
#define SSP_SET_PSP_T2 _THIS_IOW(0x1C)
#define SSP_SET_PSP_T4 _THIS_IOW(0x1D)
#define SSP_SET_PSP_T5 _THIS_IOW(0x1E)
#define SSP_SET_PSP_T6 _THIS_IOW(0x1F)

#define SSP_SET_ACTIVE_TX_SLOTS_MAP _THIS_IOW(0x20)
#define SSP_SET_ACTIVE_RX_SLOTS_MAP _THIS_IOW(0x21)

#define SSP_SELECT_BLUETOOTH_USAGE _THIS_IOW(0x22)
#define SSP_SELECT_MODEM_USAGE _THIS_IOW(0x23)

#endif /* MID_SSP_TEST_USER_H_ */
