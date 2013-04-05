/*
 *
 * Intel Management Engine Interface (Intel TXEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef _TXEI_HW_TXE_H_
#define _TXEI_HW_TXE_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kconfig.h>

#include "hw.h"
#include "hw-txe-regs.h"

extern bool nopg;


/**
 * txei_reg_read - Reads 32bit data from the txei device
 *
 * @base_addr: the base address of PCI memory BAR
 * that contains the register to be read
 * @offset: offset from which to read the data
 *
 * returns the byte read.
 */
static inline u32 txei_reg_read(void __iomem *base_addr, unsigned long offset)
{
	u32 result;
#ifdef FPGA
	u32 tmp, i;
	for (i = 0; i < 70000000; i++)
		tmp++;
#endif
	result = readl(base_addr + offset);
	return result;
}

/**
 * txei_reg_write - Writes 32bit data to the txei device
 *
 * @base_addr: the base address of PCI memory BAR that contains
 * the register to be written
 * @offset: offset from which to write the data
 * @value: the byte to write
 */
static inline void txei_reg_write(void __iomem *base_addr,
				unsigned long offset, u32 value)
{
#ifdef FPGA
	u32 tmp, i;
	for (i = 0; i < 70000000; i++)
		tmp++;
#endif

	writel(value, base_addr + offset);
}

/**
 * ipc_sec_reg_read - Reads 32bit data from the SeC BAR
 *
 * @dev: the device structure
 * @offset: offset from which to read the data
 *
 * returns the byte read.
 */
static inline u32 ipc_sec_reg_read_silent(struct txei_device *dev,
				unsigned long offset)
{
	return txei_reg_read(dev->mem_addr[SEC_BAR], offset);
}

static inline u32 ipc_sec_reg_read(struct txei_device *dev,
				unsigned long offset)
{
	if (!nopg && !dev->aliveness)
		txei_warn(dev, "aliveness not asserted\n");
	dev->aliveness_atime = jiffies;
	return ipc_sec_reg_read_silent(dev, offset);
}
/**
 * ipc_sec_reg_write - Writes 32bit data to the SeC BAR
 *
 * @dev: the device structure
 * @offset: offset from which to write the data
 * @value: the byte to write
 */
static inline void ipc_sec_reg_write_silent(struct txei_device *dev,
				unsigned long offset, u32 value)
{
	txei_reg_write(dev->mem_addr[SEC_BAR], offset, value);
}

static inline void ipc_sec_reg_write(struct txei_device *dev,
				unsigned long offset, u32 value)
{
	if (!nopg && !dev->aliveness)
		txei_warn(dev, "aliveness not asserted\n");
	dev->aliveness_atime = jiffies;
	ipc_sec_reg_write_silent(dev, offset, value);
}
/**
 * ipc_sec_reg_read - Reads 32bit data from the Bridge BAR
 *
 * @dev: the device structure
 * @offset: offset from which to read the data
 *
 * returns the byte read.
 */
static inline u32 ipc_bridge_reg_read(struct txei_device *dev,
				unsigned long offset)
{
	return txei_reg_read(dev->mem_addr[BRIDGE_BAR], offset);
}

/**
 * ipc_sec_reg_write - Writes 32bit data to the Bridge BAR
 *
 * @dev: the device structure
 * @offset: offset from which to write the data
 * @value: the byte to write
 */
static inline void ipc_bridge_reg_write(struct txei_device *dev,
				unsigned long offset, u32 value)
{
	txei_reg_write(dev->mem_addr[BRIDGE_BAR], offset, value);
}

/*
 *  Aliveness Functions
 */
void ipc_aliveness_set(struct txei_device *dev, u32 aliveness);
u32 ipc_aliveness_req_get(struct txei_device *dev);
u32 ipc_aliveness_get(struct txei_device *dev);

int ipc_aliveness_poll(struct txei_device *dev, u32 expected);
long ipc_aliveness_wait(struct txei_device *dev, u32 expected);

void ipc_aliveness_timer(struct work_struct *work);


void ipc_input_ready_interrupt_enable(struct txei_device *dev);

void ipc_input_doorbell_set(struct txei_device *dev);


void ipc_output_ready_set(struct txei_device *dev);

bool ipc_input_ready_get(struct txei_device *dev);

bool ipc_interrupts_enabled(struct txei_device *dev);
void ipc_host_interrupts_enable(struct txei_device *dev);
void ipc_host_interrupts_disable(struct txei_device *dev);
bool ipc_pending_interrupts(struct txei_device *dev);
static inline void ipc_interrupts_clear(struct txei_device *dev)
{
	ipc_sec_reg_write_silent(dev, SEC_IPC_HOST_INT_STATUS_OFFSET,
		TXEI_SEC_IPC_HOST_INT_STATUS_PENDING);
	ipc_bridge_reg_write(dev, HISR_OFFSET, TXEI_HISR_INT_STS_MSK);
	ipc_bridge_reg_write(dev, HHISR_OFFSET,  TXEI_IPC_HHIER_MSK);
}


void ipc_input_payload_write(struct txei_device *dev,
	unsigned long dw_index_to_write, u32 dw_value);

u32 ipc_output_payload_read(struct txei_device *dev,
	unsigned long dw_index_to_read);

/* Readiness access and helper functions */
void ipc_readiness_clear(struct txei_device *dev);
u32 ipc_readiness_get(struct txei_device *dev);
int ipc_readiness_poll(struct txei_device *dev);
int ipc_readiness_wait(struct txei_device *dev);
static inline bool ipc_readiness_is_sec_rdy(struct txei_device *dev)
{
	return !!(dev->readiness_state & TXEI_HICR_SEC_IPC_READINESS_SEC_RDY);
}
static inline bool ipc_readiness_is_host_rdy(struct txei_device *dev)
{
	return !!(dev->readiness_state & TXEI_HICR_SEC_IPC_READINESS_HOST_RDY);
}
void ipc_readiness_set_host_rdy(struct txei_device *dev);


void txei_setup_satt2(struct txei_device *dev, dma_addr_t addr, u32 dma_range);

/*******************************************
 * Temporary FUNCTIONS to be deleted in production
 *******************************************/

/**
 * This needs debugging. Somehow the kernel linker is not picking up these
 * symbols from txei_debugfs.c despite the fact that these symbols are being
 * exported via the EXPORT_SYMBOL macro.
 * I have done some experiements to make sure that the txei_debugfs file is
 * being compiled and does pick up the symbols from the fs/debugfs/fs.c and
 * that is no problem. It seems that if a hard coded functions with
 * EXPORT_SYMBOL is compiled in a driver subdirectory will not properly
 * export symbols despite being comipled. Therfore, I am leaving this
 * disabled by changing the ifdef to a value that is not possible.
 */
#ifdef CONFIG_DEBUG_FS_WONTWORK
int txei_dbgfs_register(struct txei_device *txei, const char *name);
void txei_dbgfs_unregister(struct txei_device *txei);
#
#else
static inline int txei_dbgfs_register(struct txei_device *txei,
	const char *name) {return 0; }

static inline void txei_dbgfs_unregister(struct txei_device *txei) {}
#endif /* HAVE_TXEI_DEBUG */

#ifdef POLLING_MODE
int polling_thread(void *_dev);
#endif /* POLLING_MODE */

void txei_read_data(struct txei_device *dev,
		     unsigned char *buffer, unsigned long buffer_length);

int txei_write_message(struct txei_device *dev,
			     struct txei_msg_hdr *header,
			     unsigned char *write_buffer,
			     unsigned long write_length);

#if 0
int txei_host_buffer_is_empty(struct txei_device *dev);
#endif

int txei_flow_ctrl_creds(struct txei_device *dev, struct txei_cl *cl);

int txei_flow_ctrl_reduce(struct txei_device *dev, struct txei_cl *cl);

int txei_send_flow_control(struct txei_device *dev, struct txei_cl *cl);

int txei_disconnect(struct txei_device *dev, struct txei_cl *cl);
int txei_other_client_is_connecting(struct txei_device *dev,
	struct txei_cl *cl);
int txei_connect(struct txei_device *dev, struct txei_cl *cl);

#endif /* _TXEI_HW_TXE_H_ */

