/*
 * hsi_arasan.h
 *
 * Implements HSI interface for Arasan controller.
 *
 * Copyright (C) 2010 Intel Corporation. All rights reserved.
 *
 * Contact: Jim Stanley <jim.stanley@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef _HSI_ARASAN_H_
#define _HSI_ARASAN_H_

/* Platform device parameters */
#define HSI_IOMEM_NAME			"HSI_HSI_BASE"
#define HSI_DMA_NAME			"HSI_DMA_BASE"

/* Register base addresses */
#define ARASAN_HSI_DMA_CONFIG(base, channel)		(base+((channel)*4))
#define ARASAN_HSI_DMA_TX_FIFO_SIZE(base)		(base+0x40)
#define ARASAN_HSI_DMA_TX_FIFO_THRESHOLD(base)		(base+0x44)
#define ARASAN_HSI_DMA_RX_FIFO_SIZE(base)		(base+0x48)
#define ARASAN_HSI_DMA_RX_FIFO_THRESHOLD(base)		(base+0x4C)

#define ARASAN_HSI_CLOCK_CONTROL(base)			(base+0x50)

#define ARASAN_HSI_HSI_STATUS(base)			(base+0x54)
#define ARASAN_HSI_HSI_STATUS1(base)			(base+0xC4)
#define ARASAN_HSI_INTERRUPT_STATUS(base)		(base+0x58)
#define ARASAN_HSI_INTERRUPT_STATUS_ENABLE(base)	(base+0x5C)
#define ARASAN_HSI_INTERRUPT_SIGNAL_ENABLE(base)	(base+0x60)

#define ARASAN_HSI_PROGRAM(base)			(base+0x64)
#define ARASAN_HSI_PROGRAM1(base)			(base+0xC8)

#define ARASAN_HSI_ARBITER_PRIORITY(base)		(base+0x68)
#define ARASAN_HSI_ARBITER_BANDWIDTH1(base)		(base+0x6C)
#define ARASAN_HSI_ARBITER_BANDWIDTH2(base)		(base+0x70)

#define ARASAN_HSI_CAPABILITY(base)			(base+0x74)

#define ARASAN_HSI_TX_DATA(base, channel)	(base+((channel)*4)+0x78)
#define ARASAN_HSI_RX_DATA(base, channel)	(base+((channel)*4)+0x98)

#define ARASAN_HSI_ERROR_INTERRUPT_STATUS(base)		(base+0xB8)
#define ARASAN_HSI_ERROR_INTERRUPT_STATUS_ENABLE(base)	(base+0xBC)
#define ARASAN_HSI_ERROR_INTERRUPT_SIGNAL_ENABLE(base)	(base+0xC0)

#define ARASAN_HSI_VERSION(base)			(base+0xFC)

/* Key register fields */
#define ARASAN_ALL_CHANNELS			((1<<8)-1)
#define ARASAN_ANY_CHANNEL			((1<<8)-1)
#define ARASAN_ANY_DMA_CHANNEL			((1<<8)-1)

#define ARASAN_DMA_ENABLE			(1<<31)
#define ARASAN_DMA_BURST_SIZE(s)		(order_base_2(s/4)<<24)
#define ARASAN_DMA_XFER_FRAMES(s)		((s)<<4)
#define ARASAN_DMA_CHANNEL(c)			((c)<<1)
#define ARASAN_DMA_DIR(d)			((d)<<0)

#define ARASAN_FIFO_MAX_BITS			10
#define ARASAN_FIFO_SIZE(s, c)			((s)<<((c)*4))
#define ARASAN_FIFO_DEPTH(r, c)			(1<<(((r)>>((c)*4)) & 0xF))

#define ARASAN_RX_TAP_DELAY_NS(c)		(min((c), 7)<<27)
#define ARASAN_RX_TAILING_BIT_COUNT(c)		((200/max((c), 50))<<24)
#define ARASAN_RX_FRAME_BURST_COUNT(c)		(((c) & 0xFF)<<16)
#define ARASAN_TX_BREAK				(1<<15)
#define ARASAN_DATA_TIMEOUT(t)			((t)<<11)
#define ARASAN_CLK_DIVISOR(d)			((d)<<3)
#define ARASAN_CLK_START			(1<<2)
#define ARASAN_CLK_STABLE			(1<<1)
#define ARASAN_CLK_ENABLE			(1<<0)

#define ARASAN_TX_EMPTY(c)			(1<<((c)+24))
#define ARASAN_ANY_RX_NOT_EMPTY			(ARASAN_ANY_CHANNEL<<8)
#define ARASAN_RX_NOT_EMPTY(c)			(1<<((c)+8))
#define ARASAN_RX_READY				(1<<7)
#define ARASAN_RX_WAKE				(1<<4)

#define ARASAN_TX_ENABLE			(1<<31)
#define ARASAN_TX_DISABLE			(0<<31)
#define ARASAN_RX_MODE(m)			(((m) == HSI_MODE_FRAME)<<30)
#define ARASAN_RX_CHANNEL_ENABLE(en, c)		((en)<<(20+(c)))
#define ARASAN_TX_CHANNEL_ENABLE(en, c)		((en)<<(12+(c)))
#define ARASAN_RX_ENABLE			(1<<11)
#define ARASAN_RX_DISABLE			(0<<11)
#define ARASAN_RX_FLOW(f)			(((f) == HSI_FLOW_PIPE)<<9)
#define ARASAN_TX_MODE(m)			(((m) == HSI_MODE_FRAME)<<8)
#define ARASAN_TX_FRAME_MODE			ARASAN_TX_MODE(HSI_MODE_FRAME)
#define ARASAN_RX_TIMEOUT_CNT(cnt)		(((cnt)&0x7F)<<1)
#define ARASAN_RESET				(1<<0)

#define ARASAN_IRQ_ERROR			(1<<31)
#define ARASAN_IRQ_ANY_DMA_COMPLETE		(ARASAN_ANY_DMA_CHANNEL<<17)
#define ARASAN_IRQ_DMA_COMPLETE(c)		(1<<((c)+17))
#define ARASAN_IRQ_RX_WAKE			(1<<16)
#define ARASAN_IRQ_ANY_RX_THRESHOLD		(ARASAN_ANY_CHANNEL<<8)
#define ARASAN_IRQ_RX_THRESHOLD(c)		(1<<((c)+8))
#define ARASAN_IRQ_ANY_TX_THRESHOLD		(ARASAN_ANY_CHANNEL)
#define ARASAN_IRQ_TX_THRESHOLD(c)		(1<<(c))

#define ARASAN_IRQ_ANY_DATA_TIMEOUT		(ARASAN_ANY_CHANNEL<<2)
#define ARASAN_IRQ_DATA_TIMEOUT(c)		(1<<((c)+2))
#define ARASAN_IRQ_RX_ERROR			(1<<1)
#define ARASAN_IRQ_BREAK			(1<<0)

#define ARASAN_RX_CHANNEL_BITS(b)	(((b) & 0x3)<<2)
#define ARASAN_RX_CHANNEL_SIZE(s)	ARASAN_RX_CHANNEL_BITS(order_base_2(s))
#define ARASAN_TX_CHANNEL_BITS(b)	((b) & 0x03)
#define ARASAN_TX_CHANNEL_SIZE(s)	ARASAN_TX_CHANNEL_BITS(order_base_2(s))
#define ARASAN_TX_CHANNEL_CNT(r)	(1<<((r) & 0x3))
#define ARASAN_RX_CHANNEL_CNT(r)	(1<<(((r)>>2) & 0x3))

#define ARASAN_TX_BASE_CLK_KHZ(r)	((((r)>>11)&0x1FF)*1000)

#endif /* _ARASAN_H */
