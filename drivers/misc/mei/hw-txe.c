/*
 *
 * Intel Management Engine Interface (Intel MEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 */

#include <linux/pci.h>
#include <linux/jiffies.h>
#include <linux/delay.h>

#include <linux/kthread.h>

#include <linux/mei.h>

#include "mei_dev.h"
#include "hw-txe.h"
#include "client.h"
#include "hbm.h"

/**
 * mei_txe_reg_read - Reads 32bit data from the mei device
 *
 * @base_addr: registers base address
 * @offset: register offset
 *
 * returns register value
 */
static inline u32 mei_txe_reg_read(void __iomem *base_addr,
					unsigned long offset)
{
	return ioread32(base_addr + offset);
}

/**
 * mei_txe_reg_write - Writes 32bit data to the mei device
 *
 * @base_addr: registers base address
 * @offset: register offset
 * @value: the value to write
 */
static inline void mei_txe_reg_write(void __iomem *base_addr,
				unsigned long offset, u32 value)
{
	iowrite32(value, base_addr + offset);
}

/**
 * mei_txe_sec_reg_read_silent - Reads 32bit data from the SeC BAR
 *  doesn't chack for aliveness
 *
 * @dev: the device structure
 * @offset: regiser offset
 *
 * returns register value
 */
static inline u32 mei_txe_sec_reg_read_silent(struct mei_txe_hw *hw,
				unsigned long offset)
{
	return mei_txe_reg_read(hw->mem_addr[SEC_BAR], offset);
}

/**
 * mei_txe_sec_reg_read - Reads 32bit data from the SeC BAR
 *
 * @dev: the device structure
 * @offset: regiser offset
 *
 * returns register value
 */
static inline u32 mei_txe_sec_reg_read(struct mei_txe_hw *hw,
				unsigned long offset)
{
	struct mei_device *dev = hw_txe_to_mei(hw);

	if (!nopg && !hw->aliveness)
		dev_warn(&dev->pdev->dev, "aliveness not asserted\n");
	hw->aliveness_atime = jiffies;

	return mei_txe_sec_reg_read_silent(hw, offset);
}
/**
 * mei_txe_sec_reg_write_silent - Writes 32bit data to the SeC BAR
 *   doesn't check for aliveness
 *
 * @dev: the device structure
 * @offset: register offset
 * @value: value to write
 */
static inline void mei_txe_sec_reg_write_silent(struct mei_txe_hw *hw,
				unsigned long offset, u32 value)
{
	mei_txe_reg_write(hw->mem_addr[SEC_BAR], offset, value);
}

/**
 * mei_txe_sec_reg_write - Writes 32bit data to the SeC BAR
 *
 * @dev: the device structure
 * @offset: register offset
 * @value: value to write
 */
static inline void mei_txe_sec_reg_write(struct mei_txe_hw *hw,
				unsigned long offset, u32 value)
{

	struct mei_device *dev = hw_txe_to_mei(hw);

	if (!nopg && !hw->aliveness)
		dev_warn(&dev->pdev->dev, "aliveness not asserted\n");
	hw->aliveness_atime = jiffies;
	mei_txe_sec_reg_write_silent(hw, offset, value);
}
/**
 * mei_txe_br_reg_read - Reads 32bit data from the Bridge BAR
 *
 * @hw: the device structure
 * @offset: offset from which to read the data
 *
 * returns the byte read.
 */
static inline u32 mei_txe_br_reg_read(struct mei_txe_hw *hw,
				unsigned long offset)
{
	return mei_txe_reg_read(hw->mem_addr[BRIDGE_BAR], offset);
}

/**
 * mei_txe_br_reg_write - Writes 32bit data to the Bridge BAR
 *
 * @hw: the device structure
 * @offset: offset from which to write the data
 * @value: the byte to write
 */
static inline void mei_txe_br_reg_write(struct mei_txe_hw *hw,
				unsigned long offset, u32 value)
{
	mei_txe_reg_write(hw->mem_addr[BRIDGE_BAR], offset, value);
}

/**
 * mei_txe_aliveness_set - request for aliveness change
	requires device lock to be held
 * @dev: the device structure
 * @req: requested aliveness value
 */
static void mei_txe_aliveness_set(struct mei_device *dev, u32 req)
{

	struct mei_txe_hw *hw = to_txe_hw(dev);

	dev_dbg(&dev->pdev->dev, "Aliveness current=%d request=%d\n",
				hw->aliveness, req);
	/* Clear requested */
	if (req)
		hw->aliveness_atime = jiffies;

	if (hw->aliveness != req) {
		hw->recvd_aliv_resp = false;
		mei_txe_br_reg_write(hw, SICR_HOST_ALIVENESS_REQ_REG, req);
	}
}


/* FXIME: move to inline in hw-txe.h */
/**
 * mei_txe_aliveness_req_get - get aliveness requested value
 *	Reads nad returns the HICR_HOST_ALIVENESS_REQ register value
 * @dev: the device structure
 */
static u32 mei_txe_aliveness_req_get(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	u32 reg;
	reg = mei_txe_br_reg_read(hw, SICR_HOST_ALIVENESS_REQ_REG);
	return reg & SICR_HOST_ALIVENESS_REQ_REQUESTED;
}
/**
 * mei_txe_aliveness_get - get aliveness response
 *	Reads nad returns the HICR_HOST_ALIVENESS_RESP register value
 * @dev: the device structure
 */
static u32 mei_txe_aliveness_get(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	u32 reg;
	reg = mei_txe_br_reg_read(hw, HICR_HOST_ALIVENESS_RESP_REG);
	return reg & HICR_HOST_ALIVENESS_RESP_ACK;
}

/**
 * mei_txe_aliveness_poll
 *	Polls for HICR_HOST_ALIVENESS_RESP.ALIVENESS_RESP to be set
 * @dev: the device struct
 * @expected: expected aliveness value
 * return: true if the expected value was received
 */
static int mei_txe_aliveness_poll(struct mei_device *dev, u32 expected)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	int t = 0;

	do {
		hw->aliveness = mei_txe_aliveness_get(dev);
		if (hw->aliveness == expected) {
			dev_dbg(&dev->pdev->dev,
				"alivness settled after %d msec\n", t);
			return t;
		}
		mutex_unlock(&dev->device_lock);
		msleep(MSEC_PER_SEC / 5);
		mutex_lock(&dev->device_lock);
		t += MSEC_PER_SEC / 5;
	} while (t < SEC_ALIVENESS_WAIT_TIMEOUT);

	dev_err(&dev->pdev->dev, "aliveness timed out\n");
	return -ETIMEDOUT;
}

/**
 * mei_txe_aliveness_wait
 *	Waits for HICR_HOST_ALIVENESS_RESP.ALIVENESS_RESP to be set
 * @dev: the device struct
 * @expected: expected aliveness value
 * return: true if the expected value was received
 */
static long mei_txe_aliveness_wait(struct mei_device *dev, u32 expected)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	const unsigned long timeout =
			msecs_to_jiffies(SEC_ALIVENESS_WAIT_TIMEOUT);
	long err = 0;

	hw->aliveness = mei_txe_aliveness_get(dev);
	if (hw->aliveness != expected) {
		mutex_unlock(&dev->device_lock);
		err = wait_event_interruptible_timeout(hw->wait_aliveness_resp,
				hw->recvd_aliv_resp, timeout);
		mutex_lock(&dev->device_lock);
		if (!err && !hw->recvd_aliv_resp) {
			dev_err(&dev->pdev->dev,
				"aliveness timed out = 0x%ld\n", err);
			return -ETIMEDOUT;
		}
	}
	dev_dbg(&dev->pdev->dev, "alivness settled after %d msec\n",
			jiffies_to_msecs(timeout - err));
	hw->recvd_aliv_resp = false;
	return err;
}

static void mei_txe_aliveness_timer(struct work_struct *work)
{
	struct mei_txe_hw *hw = container_of(work,
			struct mei_txe_hw,  aliveness_timer.work);
	struct mei_device *dev = hw_txe_to_mei(hw);
	/*
	 * if we passed the time dassert the alivness
	 * else schedule next attempt after last access
	 */
	dev_dbg(&dev->pdev->dev,
		"aliveness timer jiffies=%ld timeout=%ld atime=%ld\n",
		jiffies, hw->aliveness_timeout, hw->aliveness_atime);
	mutex_lock(&dev->device_lock);
	if (dev->dev_state != MEI_DEV_ENABLED)
		return;

	if (time_after(jiffies, hw->aliveness_atime + hw->aliveness_timeout))
			mei_txe_aliveness_set(dev, 0);
	else
		schedule_delayed_work(&hw->aliveness_timer,
				hw->aliveness_timeout);
	mutex_unlock(&dev->device_lock);
}

/**
 * mei_txe_input_ready_interrupt_enable - sets the Input Ready Interrupt
 *
 * @dev: the device structure
 */
static void mei_txe_input_ready_interrupt_enable(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	u32 hintmsk;
	/* Enable the SEC_IPC_HOST_INT_MASK_IN_RDY interrupt */
	hintmsk = mei_txe_sec_reg_read(hw, SEC_IPC_HOST_INT_MASK_REG);
	hintmsk |= SEC_IPC_HOST_INT_MASK_IN_RDY;
	mei_txe_sec_reg_write(hw, SEC_IPC_HOST_INT_MASK_REG, hintmsk);
}

/**
 * mei_txe_input_doorbell_set
 *   - Sets bit 0 in SEC_IPC_INPUT_DOORBELL.IPC_INPUT_DOORBELL.
 * @dev: the device structure
 */
static void mei_txe_input_doorbell_set(struct mei_txe_hw *hw)
{
	/* Clear the interrupt cause */
	clear_bit(TXE_INTR_IN_READY_BIT, &hw->intr_cause);
	mei_txe_sec_reg_write(hw, SEC_IPC_INPUT_DOORBELL_REG, 1);
}

/**
 * mei_txe_output_ready_set - Sets the SICR_SEC_IPC_OUTPUT_STATUS bit to 1
 * @dev: the device structure
 */
static void mei_txe_output_ready_set(struct mei_txe_hw *hw)
{
	mei_txe_br_reg_write(hw,
			SICR_SEC_IPC_OUTPUT_STATUS_REG,
			SEC_IPC_OUTPUT_STATUS_RDY);
}

/**
 * mei_txe_is_input_ready - check if TXE is ready for receiving data
 *
 * @dev: the device structure
 */
static bool mei_txe_is_input_ready(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	u32 status;
	status = mei_txe_sec_reg_read(hw, SEC_IPC_INPUT_STATUS_REG);
	return !!(SEC_IPC_INPUT_STATUS_RDY & status);
}

/**
 * mei_txe_intr_clear - clear all interrupts
 *
 * @dev: the device structure
 */
static inline void mei_txe_intr_clear(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	mei_txe_sec_reg_write_silent(hw, SEC_IPC_HOST_INT_STATUS_REG,
		SEC_IPC_HOST_INT_STATUS_PENDING);
	mei_txe_br_reg_write(hw, HISR_REG, HISR_INT_STS_MSK);
	mei_txe_br_reg_write(hw, HHISR_REG, IPC_HHIER_MSK);
}

/**
 * mei_txe_intr_disable - disable all interrupts
 * @dev: the device structure
 */
static void mei_txe_intr_disable(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	mei_txe_br_reg_write(hw, HHIER_REG, 0);
	mei_txe_br_reg_write(hw, HIER_REG, 0);
}
/**
 * mei_txe_intr_disable - enable all interrupts
 *
 * @dev: the device structure
 */
static void mei_txe_intr_enable(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	mei_txe_br_reg_write(hw, HHIER_REG, IPC_HHIER_MSK);
	mei_txe_br_reg_write(hw, HIER_REG, HIER_INT_EN_MSK);
}

/**
 * mei_txe_pending_interrupts - check if there are pending interrupts
 *	only Aliveness, Input ready, and output doorbell are of relevance
 *
 * @dev: the device structure
 */
static bool mei_txe_pending_interrupts(struct mei_device *dev)
{

	struct mei_txe_hw *hw = to_txe_hw(dev);
	bool ret = (hw->intr_cause &
		(TXE_INTR_ALIVENESS | TXE_INTR_IN_READY | TXE_INTR_OUT_DB));

	if (ret) {
		dev_err(&dev->pdev->dev,
			"Pending Interrupts InReady=%ld Readiness=%ld, Aliveness=%ld, OutDoor=%ld\n",
			hw->intr_cause & TXE_INTR_IN_READY,
			hw->intr_cause & TXE_INTR_READINESS,
			hw->intr_cause & TXE_INTR_ALIVENESS,
			hw->intr_cause & TXE_INTR_OUT_DB);
	}
	return ret;
}

static void mei_txe_input_payload_write(struct mei_device *dev,
			unsigned long index, u32 value)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	mei_txe_sec_reg_write(hw, SEC_IPC_INPUT_PAYLOAD_REG +
			(index * sizeof(u32)), value);
}

static u32 mei_txe_out_data_read(const struct mei_device *dev,
					unsigned long idx)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	return mei_txe_br_reg_read(hw,
		BRIDGE_IPC_OUTPUT_PAYLOAD_REG + (idx * sizeof(u32)));
}

/* Readiness */

/**
 * mei_txe_readiness_set_host_rdy
 *
 * @dev: the device structure
 */
static void mei_txe_readiness_set_host_rdy(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	mei_txe_br_reg_write(hw,
		SICR_HOST_IPC_READINESS_REQ_REG,
		SICR_HOST_IPC_READINESS_HOST_RDY);
}

/**
 * mei_txe_readiness_clear
 *
 * @dev: the device structure
 */
static void mei_txe_readiness_clear(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	mei_txe_br_reg_write(hw, SICR_HOST_IPC_READINESS_REQ_REG,
				SICR_HOST_IPC_READINESS_RDY_CLR);
}
/**
 * mei_txe_readiness_get - Reads and returns
 *	the HICR_SEC_IPC_READINESS register value
 *
 * @dev: the device structure
 */
static u32 mei_txe_readiness_get(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	return mei_txe_br_reg_read(hw, HICR_SEC_IPC_READINESS_REG);
}


/**
 * mei_txe_readiness_is_sec_rdy - check readiness_state
 *  for HICR_SEC_IPC_READINESS_SEC_RDY
 *
 * @readiness_state - cached readiness state
 */
static inline bool mei_txe_readiness_is_sec_rdy(u32 readiness_state)
{
	return !!(readiness_state & HICR_SEC_IPC_READINESS_SEC_RDY);
}

/**
 * mei_txe_hw_is_ready - check if the hw is ready
 *
 * @dev: the device structure
 */
static bool mei_txe_hw_is_ready(struct mei_device *dev)
{
	u32 readiness_state =  mei_txe_readiness_get(dev);
	return mei_txe_readiness_is_sec_rdy(readiness_state);
}

/**
 * mei_txe_host_is_ready - check if the host is ready
 *
 * @dev: the device structure
 */
static inline bool mei_txe_host_is_ready(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	u32 reg = mei_txe_br_reg_read(hw, HICR_SEC_IPC_READINESS_REG);
	return !!(reg & HICR_SEC_IPC_READINESS_HOST_RDY);
}

static int mei_txe_readiness_wait(struct mei_device *dev)
{
	int err;
	if (mei_txe_hw_is_ready(dev))
		return 0;
	mutex_unlock(&dev->device_lock);
	err = wait_event_interruptible_timeout(dev->wait_hw_ready,
			dev->recvd_hw_ready, SEC_READY_WAIT_TIMEOUT);
	mutex_lock(&dev->device_lock);
	if (!err && !dev->recvd_hw_ready) {
		dev_dbg(&dev->pdev->dev,
			"wait for readiness failed: status = 0x%x\n", err);
		return -ETIMEDOUT;
	}

	dev->recvd_hw_ready = false;
	return 0;
}

static void mei_txe_hw_config(struct mei_device *dev)
{

	struct mei_txe_hw *hw = to_txe_hw(dev);
	/* Doesn't change in runtime */
	dev->hbuf_depth = PAYLOAD_SIZE / 4;

	hw->aliveness = mei_txe_aliveness_get(dev);
	hw->readiness_state = mei_txe_readiness_get(dev);

	dev_dbg(&dev->pdev->dev, "aliveness_resp = 0x%08x, readiness = 0x%08x.\n",
		hw->aliveness, hw->readiness_state);
}

/**
 * mei_txe_write - writes a message to mei device.
 *
 * @dev: the device structure
 * @header: header of message
 * @buf: message buffer will be written
 * returns 1 if success, 0 - otherwise.
 */

static int mei_txe_write(struct mei_device *dev,
		struct mei_msg_hdr *header, unsigned char *buf)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	unsigned long rem;
	unsigned long length = header->length;
	u32 *reg_buf = (u32 *)buf;
	int i;

	dev_dbg(&dev->pdev->dev, MEI_HDR_FMT, MEI_HDR_PRM(header));

	if ((length + sizeof(struct mei_msg_hdr)) > PAYLOAD_SIZE) {
		dev_dbg(&dev->pdev->dev,
			"write lenght exceeded = %ld\n", length);
		return 0;
	}

	if (!nopg && !hw->aliveness) {
		dev_dbg(&dev->pdev->dev,
			"Aliveness not set .... requesting\n");
		mei_txe_aliveness_set(dev, 1);
		if (mei_txe_aliveness_wait(dev, 1) < 0)
			return 0;
	}

	/* Enable Input Ready Interrupt. */
	mei_txe_input_ready_interrupt_enable(dev);

	if (!mei_txe_is_input_ready(dev)) {
		dev_dbg(&dev->pdev->dev, "Input is not ready");
		return 0;
	}

	mei_txe_input_payload_write(dev, 0, *((u32 *)header));

	for (i = 0; i < length / 4; i++)
		mei_txe_input_payload_write(dev, i + 1, reg_buf[i]);

	rem = length & 0x3;
	if (rem > 0) {
		u32 reg = 0;
		memcpy(&reg, &buf[length - rem], rem);
		mei_txe_input_payload_write(dev, i + 1, reg);
	}

	dev->hbuf_is_ready = false;
	/* Set Input-Doorbell */
	mei_txe_input_doorbell_set(hw);

	return 0;
}

static size_t mei_txe_hbuf_max_len(const struct mei_device *dev)
{
	return PAYLOAD_SIZE - sizeof(struct mei_msg_hdr);
}

static int mei_txe_hbuf_empty_slots(struct mei_device *dev)
{
	return dev->hbuf_depth;
}

static int mei_txe_count_full_read_slots(struct mei_device *dev)
{
	/* read buffers has static size */
	return  PAYLOAD_SIZE / 4;
}

static u32 mei_txe_read_hdr(const struct mei_device *dev)
{
	return mei_txe_out_data_read(dev, 0);
}
/**
 * mei_txe_read - reads a message from the txe device.
 *
 * @dev: the device structure
 * @buf: message buffer will be written
 * @len: message size will be read
 */
static int mei_txe_read(struct mei_device *dev,
		unsigned char *buf, unsigned long len)
{

	struct mei_txe_hw *hw = to_txe_hw(dev);
	u32 i;
	u32 *reg_buf = (u32 *)buf;
	u32 rem = len & 0x3;

	dev_dbg(&dev->pdev->dev,
		"buffer-length = %lu buf[0]0x%08X\n",
		len, mei_txe_out_data_read(dev, 0));

	for (i = 0; i < len / 4; i++) {
		/* skip header: index starts from 1 */
		u32 reg = mei_txe_out_data_read(dev, i + 1);
		dev_dbg(&dev->pdev->dev, "buf[%d] = 0x%08X\n", i, reg);
		*reg_buf++ = reg;
	}

	if (rem) {
		u32 reg = mei_txe_out_data_read(dev, i + 1);
		memcpy(reg_buf, &reg, rem);
	}

	mei_txe_output_ready_set(hw);
	return 0;
}

/**
 * mei_txe_hw_reset - resets host and fw.
 *
 * @dev: the device structure
 * @intr_enable: if interrupt should be enabled after reset.
 */
static int mei_txe_hw_reset(struct mei_device *dev, bool intr_enable)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);

	u32 aliveness_req;
	/*
	 * HPS: 3.1.1.1-2
	 * read input doorbell to ensure consistency between  Bridge and SeC
	 * return value might be garbage return
	 */
	(void)mei_txe_sec_reg_read_silent(hw, SEC_IPC_INPUT_DOORBELL_REG);

	/* Cancle aliveness machine */
	cancel_delayed_work_sync(&hw->aliveness_timer);

	aliveness_req = mei_txe_aliveness_req_get(dev);
	hw->aliveness = mei_txe_aliveness_get(dev);

	/* Disable interrupts in this stage we will poll */
	mei_txe_intr_disable(dev);

	/*
	 * HPS: 3.1.1.1-3
	 * If Aliveness Request and Aliveness Response are not equal then
	 * wait for them to be equal
	 * Since we might have interrupts disabled - poll for it
	 */
	if (aliveness_req != hw->aliveness)
		if (mei_txe_aliveness_poll(dev, aliveness_req) < 0) {
			dev_dbg(&dev->pdev->dev,
				"wait for aliveness failed ... bailing out\n");
			return -ENODEV;
		}


	/*
	 * HPS: 3.1.1.1-4
	 * If Aliveness Request and Aliveness Response are set then clear them
	 */
	if (aliveness_req) {
		mei_txe_aliveness_set(dev, 0);
		if (mei_txe_aliveness_poll(dev, 0) < 0) {
			dev_dbg(&dev->pdev->dev,
				"wait for aliveness failed ... bailing out\n");
			return -ENODEV;
		}
	}

	/*
	 * HPS: 3.1.1.1-5
	 * Set rediness RDY_CLR bit
	 */
	mei_txe_readiness_clear(dev);

	/*
	 * HPS: 3.1.1.1-6
	 * Host removes any open HECI connections
	 */

	return 0;
}

static int mei_txe_hw_start(struct mei_device *dev)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	int ret;

	u32 hisr;
	/*
	 * HPS: 3.1.1.1-7
	 * SATT2_CTRL.ENTRY_VLD clear - not set here
	 */

	/* bring back interrupts */
	mei_txe_intr_enable(dev);

	ret = mei_txe_readiness_wait(dev);
	if (ret < 0) {
		dev_err(&dev->pdev->dev,
			"wating for readiness failed\n");
		return ret;
	}

	/* HPS: 3.1.1.1-10
	 * If HISR.INT2_STS interrupt status bit is set then clear it. */
	hisr = mei_txe_br_reg_read(hw, HISR_REG);
	if (hisr & HISR_INT_2_STS)
		mei_txe_br_reg_write(hw, HISR_REG, HISR_INT_2_STS);

	/* Clear the interrupt cause of OutputDoorbell */
	clear_bit(TXE_INTR_OUT_DB_BIT, &hw->intr_cause);

	/* HPS: 3.1.1.1-11
	 * set SATT2 - not here */

	/* HPS: 3.1.1.1-12 */
	/* enable input ready interrupts:
	 * SEC_IPC_HOST_INT_MASK.IPC_INPUT_READY_INT_MASK
	 */
	mei_txe_input_ready_interrupt_enable(dev);


	/* HPS: 3.1.1.1-13 */
	/*  Set the SICR_SEC_IPC_OUTPUT_STATUS.IPC_OUTPUT_READY bit */
	mei_txe_output_ready_set(hw);

	mei_txe_aliveness_set(dev, 1);
	ret = mei_txe_aliveness_wait(dev, 1);
	if (ret < 0) {
		dev_err(&dev->pdev->dev,
			"wait for aliveness failed ... bailing out\n");
		return ret;
	}

	/* HPS: 3.1.1.1-14 */
	/* Set bit SICR_HOST_IPC_READINESS.HOST_RDY
	 */
	mei_txe_readiness_set_host_rdy(dev);

	return 0;
}


static bool mei_txe_check_and_ack_intrs(struct mei_device *dev, bool do_ack)
{
	struct mei_txe_hw *hw = to_txe_hw(dev);
	u32 hisr;
	u32 hhisr;
	u32 ipc_isr;
	bool generated;

	/* read interrupt registers */
	hhisr = mei_txe_br_reg_read(hw, HHISR_REG);
	generated = (hhisr & IPC_HHIER_MSK);
	if (!generated)
		goto out;

	hisr = mei_txe_br_reg_read(hw, HISR_REG);

	if (hhisr & IPC_HHIER_SEC)
		ipc_isr = mei_txe_sec_reg_read_silent(hw,
				SEC_IPC_HOST_INT_STATUS_REG);
	else
		ipc_isr = 0;

	generated = generated ||
		(hisr & HISR_INT_STS_MSK) ||
		(ipc_isr & SEC_IPC_HOST_INT_STATUS_PENDING);

	if (generated && do_ack) {
		/* Save the interrupt causes */
		hw->intr_cause |= hisr & HISR_INT_STS_MSK;
		if (ipc_isr & SEC_IPC_HOST_INT_STATUS_IN_RDY)
			hw->intr_cause |= TXE_INTR_IN_READY;


		mei_txe_intr_disable(dev);
		/* Clear the interrupts in hierarchy:
		 * IPC and Bridge, than the High Level */
		mei_txe_sec_reg_write_silent(hw,
			SEC_IPC_HOST_INT_STATUS_REG, ipc_isr);
		mei_txe_br_reg_write(hw, HISR_REG, hisr);
		mei_txe_br_reg_write(hw, HHISR_REG, hhisr);
	}

out:
	return generated;
}

/**
 * mei_txe_irq_quick_handler - The ISR of the MEI device
 *
 * @irq: The irq number
 * @dev_id: pointer to the device structure
 *
 * returns irqreturn_t
 */
irqreturn_t mei_txe_irq_quick_handler(int irq, void *dev_id)
{
	struct mei_device *dev = dev_id;

	if (mei_txe_check_and_ack_intrs(dev, true))
		return IRQ_WAKE_THREAD;
	return IRQ_NONE;
}


/**
 * mei_interrupt_thread_handler - function called after ISR to handle the interrupt
 * processing.
 *
 * @irq: The irq number
 * @dev_id: pointer to the device structure
 *
 * returns irqreturn_t
 *
 */
irqreturn_t mei_txe_irq_thread_handler(int irq, void *dev_id)
{
	struct mei_device *dev = (struct mei_device *) dev_id;
	struct mei_txe_hw *hw = to_txe_hw(dev);
	struct mei_cl_cb complete_list;
	s32 slots;
	int rets;

	/* Cannot print in IRQ Level */
	dev_dbg(&dev->pdev->dev,
		"Interrupt was generated (%02X|%04X|%02X) InReady=%d, Readiness=%d, Aliveness=%d, OutputDoorbell=%d\n",
		mei_txe_br_reg_read(hw, HHISR_REG),
		mei_txe_br_reg_read(hw, HISR_REG),
		mei_txe_sec_reg_read_silent(hw, SEC_IPC_HOST_INT_STATUS_REG),
		!!(hw->intr_cause & TXE_INTR_IN_READY),
		!!(hw->intr_cause & TXE_INTR_READINESS),
		!!(hw->intr_cause & TXE_INTR_ALIVENESS),
		!!(hw->intr_cause & TXE_INTR_OUT_DB));

again:
	dev_dbg(&dev->pdev->dev, "function called after ISR to handle the interrupt processing.\n");
	/* initialize our complete list */
	mutex_lock(&dev->device_lock);
	mei_io_list_init(&complete_list);

	hw->aliveness = mei_txe_aliveness_get(dev);
	hw->readiness_state = mei_txe_readiness_get(dev);

	/* Readiness:
	 * Detection of SeC driver going through reset
	 * or SeC driver resetting HECI interface.
	 */
	if (test_and_clear_bit(TXE_INTR_READINESS_BIT, &hw->intr_cause)) {
		dev_dbg(&dev->pdev->dev, "Readiness Interrupt was received...\n");

		/* Check if SeC is going through reset */
		if (mei_txe_readiness_is_sec_rdy(hw->readiness_state)) {
			dev_dbg(&dev->pdev->dev, "we need to start the dev.\n");
			dev->recvd_hw_ready = true;
		} else {
			dev->recvd_hw_ready = false;
			if (dev->dev_state != MEI_DEV_RESETTING &&
			    dev->dev_state != MEI_DEV_POWER_DOWN &&
			    dev->dev_state != MEI_DEV_INITIALIZING) {
				dev_dbg(&dev->pdev->dev, "FW not ready.\n");
				mei_reset(dev, true);
				goto end;
			}
		}
		wake_up_interruptible(&dev->wait_hw_ready);
	}

	/************************************************************/
	/* Check interrupt cause:
	 * Aliveness: Detection of SeC acknowledge of host request that
	 * it remain alive or host cancellation of that request.
	 */

	if (test_and_clear_bit(TXE_INTR_ALIVENESS_BIT, &hw->intr_cause)) {
		/* Clear the interrupt cause */
		dev_dbg(&dev->pdev->dev,
			"Aliveness Interrrupt: Status: %d\n", hw->aliveness);
		hw->recvd_aliv_resp = true;
		wake_up_interruptible(&hw->wait_aliveness_resp);

		if (nopg == false &&
		    dev->dev_state == MEI_DEV_ENABLED && hw->aliveness &&
		    !delayed_work_pending(&hw->aliveness_timer)) {
			dev_dbg(&dev->pdev->dev, "sechudle aliveness timer\n");
			schedule_delayed_work(&hw->aliveness_timer,
					hw->aliveness_timeout);
		}
	}


	if (nopg == false && !hw->aliveness) {
		dev_dbg(&dev->pdev->dev,
			"Bailing Out: HW Is dromant %d\n", hw->aliveness);
		mutex_unlock(&dev->device_lock);
		goto out;
	}
	/* Output Doorbell:
	 * Detection of SeC having sent output to host
	 */
	slots = mei_count_full_read_slots(dev);
	if (test_bit(TXE_INTR_OUT_DB_BIT, &hw->intr_cause)) {
		/* TODO: Check if FW is doing Reset */

		/* Read from TXE */
		rets = mei_irq_read_handler(dev, &complete_list, &slots);
		dev_dbg(&dev->pdev->dev,
			"from mei_irq_read_handler ret=%d.\n", rets);
		if (rets)
			goto end;
		/* clear the pending interrupt only if read hanlder suceeded */
		clear_bit(TXE_INTR_OUT_DB_BIT, &hw->intr_cause);
	}
	/* Input Ready: Detection if host can write to SeC */
	else if (test_and_clear_bit(TXE_INTR_IN_READY_BIT, &hw->intr_cause))
		dev->hbuf_is_ready = true;

	if (dev->hbuf_is_ready) {
		/* if SeC did not complete reading the written data by host */
		if (!mei_txe_is_input_ready(dev)) {
			dev_err(&dev->pdev->dev,
				"reset: got Input Ready Int, but SEC_IPC_INPUT_STATUS_RDY is 0.\n");
			mei_reset(dev, true);
			goto end;
		}

		rets = mei_irq_write_handler(dev, &complete_list);
		dev_dbg(&dev->pdev->dev,
				"mei_irq_write_handler ret=%d.\n", rets);
	}

end:
	dev_dbg(&dev->pdev->dev, "end of bottom half function.\n");
	/* FIXME: Check if this is correct !!! */
	dev->hbuf_is_ready = mei_txe_is_input_ready(dev);

	mutex_unlock(&dev->device_lock);

	mei_irq_compl_handler(dev, &complete_list);

out:
	if (mei_txe_pending_interrupts(dev))
		goto again;

	mei_enable_interrupts(dev);
	return IRQ_HANDLED;
}


static const struct mei_hw_ops mei_txe_hw_ops = {

	.host_is_ready = mei_txe_host_is_ready,

	.hw_is_ready = mei_txe_hw_is_ready,
	.hw_reset = mei_txe_hw_reset,
	.hw_config = mei_txe_hw_config,
	.hw_start = mei_txe_hw_start,

	.intr_clear = mei_txe_intr_clear,
	.intr_enable = mei_txe_intr_enable,
	.intr_disable = mei_txe_intr_disable,

	.hbuf_free_slots = mei_txe_hbuf_empty_slots,
	.hbuf_is_ready = mei_txe_is_input_ready,
	.hbuf_max_len = mei_txe_hbuf_max_len,

	.write = mei_txe_write,

	.rdbuf_full_slots = mei_txe_count_full_read_slots,
	.read_hdr = mei_txe_read_hdr,

	.read = mei_txe_read,

};

struct mei_device *mei_txe_dev_init(struct pci_dev *pdev)
{
	struct mei_device *dev;
	struct mei_txe_hw *hw;

	dev = kzalloc(sizeof(struct mei_device) +
			 sizeof(struct mei_txe_hw), GFP_KERNEL);
	if (!dev)
		return NULL;

	mei_device_init(dev);

	hw = to_txe_hw(dev);

	init_waitqueue_head(&hw->wait_aliveness_resp);
	INIT_DELAYED_WORK(&hw->aliveness_timer, mei_txe_aliveness_timer);

	dev->ops = &mei_txe_hw_ops;

	dev->pdev = pdev;
	return dev;
}
