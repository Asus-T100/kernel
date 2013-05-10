/*
 * Synopsys DesignWare I2C adapter driver (master only).
 *
 * Based on the TI DAVINCI I2C adapter driver.
 *
 * Copyright (C) 2006 Texas Instruments.
 * Copyright (C) 2007 MontaVista Software Inc.
 * Copyright (C) 2009 Provigent Ltd.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include "i2c-designware-core.h"

static char *abort_sources[] = {
	[ABRT_7B_ADDR_NOACK] =
		"slave address not acknowledged (7bit mode)",
	[ABRT_10ADDR1_NOACK] =
		"first address byte not acknowledged (10bit mode)",
	[ABRT_10ADDR2_NOACK] =
		"second address byte not acknowledged (10bit mode)",
	[ABRT_TXDATA_NOACK] =
		"data not acknowledged",
	[ABRT_GCALL_NOACK] =
		"no acknowledgement for a general call",
	[ABRT_GCALL_READ] =
		"read after general call",
	[ABRT_SBYTE_ACKDET] =
		"start byte acknowledged",
	[ABRT_SBYTE_NORSTRT] =
		"trying to send start byte when restart is disabled",
	[ABRT_10B_RD_NORSTRT] =
		"trying to read when restart is disabled (10bit mode)",
	[ABRT_MASTER_DIS] =
		"trying to use disabled adapter",
	[ARB_LOST] =
		"lost arbitration",
};

u32 dw_readl(struct dw_i2c_dev *dev, int offset)
{
	u32 value = readl(dev->base + offset);

	if (dev->swab)
		return swab32(value);
	else
		return value;
}

void dw_writel(struct dw_i2c_dev *dev, u32 b, int offset)
{
	if (dev->swab)
		b = swab32(b);

	writel(b, dev->base + offset);
}

static void i2c_dw_dump(struct dw_i2c_dev *dev)
{
	u32 value;

	dev_err(dev->dev, "===== REGISTER DUMP (i2c) =====\n");
	value = dw_readl(dev, DW_IC_CON);
	dev_err(dev->dev, "DW_IC_CON:               0x%x\n", value);
	value = dw_readl(dev, DW_IC_TAR);
	dev_err(dev->dev, "DW_IC_TAR:               0x%x\n", value);
	value = dw_readl(dev, DW_IC_SS_SCL_HCNT);
	dev_err(dev->dev, "DW_IC_SS_SCL_HCNT:       0x%x\n", value);
	value = dw_readl(dev, DW_IC_SS_SCL_LCNT);
	dev_err(dev->dev, "DW_IC_SS_SCL_LCNT:       0x%x\n", value);
	value = dw_readl(dev, DW_IC_FS_SCL_HCNT);
	dev_err(dev->dev, "DW_IC_FS_SCL_HCNT:       0x%x\n", value);
	value = dw_readl(dev, DW_IC_FS_SCL_LCNT);
	dev_err(dev->dev, "DW_IC_FS_SCL_LCNT:       0x%x\n", value);
	value = dw_readl(dev, DW_IC_INTR_STAT);
	dev_err(dev->dev, "DW_IC_INTR_STAT:         0x%x\n", value);
	value = dw_readl(dev, DW_IC_INTR_MASK);
	dev_err(dev->dev, "DW_IC_INTR_MASK:         0x%x\n", value);
	value = dw_readl(dev, DW_IC_RAW_INTR_STAT);
	dev_err(dev->dev, "DW_IC_RAW_INTR_STAT:     0x%x\n", value);
	value = dw_readl(dev, DW_IC_RX_TL);
	dev_err(dev->dev, "DW_IC_RX_TL:             0x%x\n", value);
	value = dw_readl(dev, DW_IC_TX_TL);
	dev_err(dev->dev, "DW_IC_TX_TL:             0x%x\n", value);
	value = dw_readl(dev, DW_IC_ENABLE);
	dev_err(dev->dev, "DW_IC_ENABLE:            0x%x\n", value);
	value = dw_readl(dev, DW_IC_STATUS);
	dev_err(dev->dev, "DW_IC_STATUS:            0x%x\n", value);
	value = dw_readl(dev, DW_IC_TXFLR);
	dev_err(dev->dev, "DW_IC_TXFLR:             0x%x\n", value);
	value = dw_readl(dev, DW_IC_RXFLR);
	dev_err(dev->dev, "DW_IC_RXFLR:             0x%x\n", value);
	value = dw_readl(dev, DW_IC_TX_ABRT_SOURCE);
	dev_err(dev->dev, "DW_IC_TX_ABRT_SOURCE:    0x%x\n", value);
	value = dw_readl(dev, DW_IC_DATA_CMD);
	dev_err(dev->dev, "DW_IC_DATA_CMD:          0x%x\n", value);
	dev_err(dev->dev, "===============================\n");
}

static u32
i2c_dw_scl_hcnt(u32 ic_clk, u32 tSYMBOL, u32 tf, int cond, int offset)
{
	/*
	 * DesignWare I2C core doesn't seem to have solid strategy to meet
	 * the tHD;STA timing spec.  Configuring _HCNT based on tHIGH spec
	 * will result in violation of the tHD;STA spec.
	 */
	if (cond)
		/*
		 * Conditional expression:
		 *
		 *   IC_[FS]S_SCL_HCNT + (1+4+3) >= IC_CLK * tHIGH
		 *
		 * This is based on the DW manuals, and represents an ideal
		 * configuration.  The resulting I2C bus speed will be
		 * faster than any of the others.
		 *
		 * If your hardware is free from tHD;STA issue, try this one.
		 */
		return (ic_clk * tSYMBOL + 5000) / 10000 - 8 + offset;
	else
		/*
		 * Conditional expression:
		 *
		 *   IC_[FS]S_SCL_HCNT + 3 >= IC_CLK * (tHD;STA + tf)
		 *
		 * This is just experimental rule; the tHD;STA period turned
		 * out to be proportinal to (_HCNT + 3).  With this setting,
		 * we could meet both tHIGH and tHD;STA timing specs.
		 *
		 * If unsure, you'd better to take this alternative.
		 *
		 * The reason why we need to take into account "tf" here,
		 * is the same as described in i2c_dw_scl_lcnt().
		 */
		return (ic_clk * (tSYMBOL + tf) + 5000) / 10000 - 3 + offset;
}

static u32 i2c_dw_scl_lcnt(u32 ic_clk, u32 tLOW, u32 tf, int offset)
{
	/*
	 * Conditional expression:
	 *
	 *   IC_[FS]S_SCL_LCNT + 1 >= IC_CLK * (tLOW + tf)
	 *
	 * DW I2C core starts counting the SCL CNTs for the LOW period
	 * of the SCL clock (tLOW) as soon as it pulls the SCL line.
	 * In order to meet the tLOW timing spec, we need to take into
	 * account the fall time of SCL signal (tf).  Default tf value
	 * should be 0.3 us, for safety.
	 */
	return ((ic_clk * (tLOW + tf) + 5000) / 10000) - 1 + offset;
}

/**
 * i2c_dw_init() - initialize the designware i2c master hardware
 * @dev: device private data
 *
 * This functions configures and enables the I2C master.
 * This function is called during I2C init function, and in case of timeout at
 * run time.
 */
int i2c_dw_init(struct dw_i2c_dev *dev)
{
	u32 input_clock_khz;
	u32 hcnt, lcnt;
	u32 reg;

	if (dev->reset)
		dev->reset(dev);

	input_clock_khz = dev->get_clk_rate_khz(dev);

	/* Configure register endianess access */
	reg = dw_readl(dev, DW_IC_COMP_TYPE);
	if (reg == ___constant_swab32(DW_IC_COMP_TYPE_VALUE)) {
		dev->swab = 1;
		reg = DW_IC_COMP_TYPE_VALUE;
	}

	if (reg != DW_IC_COMP_TYPE_VALUE) {
		dev_err(dev->dev, "Unknown Synopsys component type: "
			"0x%08x\n", reg);
		return -ENODEV;
	}

	/* Disable the adapter */
	i2c_dw_disable(dev);

	if (dev->get_scl_cfg && !dev->use_dyn_clk)
		dev->get_scl_cfg(dev);
	else {
		/* set standard and fast speed deviders for high/low periods */

		/* Standard-mode */
		hcnt = i2c_dw_scl_hcnt(input_clock_khz,
					227,	/* tHD;STA = tHIGH = 22.7 us */
					3,	/* tf = 0.3 us */
					0,	/* 0: DW default, 1: Ideal */
					23);	/* offset = 23 */
		lcnt = i2c_dw_scl_lcnt(input_clock_khz,
					227,	/* tLOW = 22.7 us */
					3,	/* tf = 0.3 us */
					28);	/* offset = 28 */
		dw_writel(dev, hcnt, DW_IC_SS_SCL_HCNT);
		dw_writel(dev, lcnt, DW_IC_SS_SCL_LCNT);
		dev_dbg(dev->dev, "Standard-mode HCNT:LCNT = %d:%d\n",
					hcnt, lcnt);

		/* Fast-mode */
		hcnt = i2c_dw_scl_hcnt(input_clock_khz,
					52,	/* tHD;STA = tHIGH = 5.2 us */
					3,	/* tf = 0.3 us */
					0,	/* 0: DW default, 1: Ideal */
					11);	/* offset = 11 */
		lcnt = i2c_dw_scl_lcnt(input_clock_khz,
					72,	/* tLOW = 7.2 us */
					3,	/* tf = 0.3 us */
					12);	/* offset = 12 */
		dw_writel(dev, hcnt, DW_IC_FS_SCL_HCNT);
		dw_writel(dev, lcnt, DW_IC_FS_SCL_LCNT);
		dev_dbg(dev->dev, "Fast-mode HCNT:LCNT = %d:%d\n", hcnt, lcnt);
	}

	/* Configure Tx/Rx FIFO threshold levels */
	dw_writel(dev, dev->tx_fifo_depth/2, DW_IC_TX_TL);
	dw_writel(dev, 0, DW_IC_RX_TL);

	/* configure the i2c master */
	if (!dev->use_dyn_clk)
		dw_writel(dev, dev->master_cfg , DW_IC_CON);
	else
		dw_writel(dev, INTEL_MID_STD_CFG | dev->speed_cfg, DW_IC_CON);

	return 0;
}

/*
 * Waiting for bus not busy
 */
static int i2c_dw_wait_bus_not_busy(struct dw_i2c_dev *dev)
{
	int timeout = TIMEOUT;

	while (dw_readl(dev, DW_IC_STATUS) & DW_IC_STATUS_ACTIVITY) {
		if (timeout <= 0) {
			dev_warn(dev->dev, "timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		timeout--;
		mdelay(1);
	}

	return 0;
}

static void i2c_dw_xfer_init(struct dw_i2c_dev *dev)
{
	struct i2c_msg *msgs = dev->msgs;
	u32 ic_con;

	/* Disable the adapter */
	i2c_dw_disable(dev);

	/* set the slave (target) address */
	dw_writel(dev, msgs[dev->msg_write_idx].addr, DW_IC_TAR);

	/* if the slave address is ten bit address, enable 10BITADDR */
	ic_con = dw_readl(dev, DW_IC_CON);
	if (msgs[dev->msg_write_idx].flags & I2C_M_TEN)
		ic_con |= DW_IC_CON_10BITADDR_MASTER;
	else
		ic_con &= ~DW_IC_CON_10BITADDR_MASTER;
	dw_writel(dev, ic_con, DW_IC_CON);

	/* Enable the adapter */
	i2c_dw_enable(dev);

	/* Enable interrupts */
	dw_writel(dev, DW_IC_INTR_DEFAULT_MASK, DW_IC_INTR_MASK);
}

/*
 * Initiate (and continue) low level master read/write transaction.
 * This function is only called from i2c_dw_isr, and pumping i2c_msg
 * messages into the tx buffer.  Even if the size of i2c_msg data is
 * longer than the size of the tx buffer, it handles everything.
 */
void
i2c_dw_xfer_msg(struct dw_i2c_dev *dev)
{
	struct i2c_msg *msgs = dev->msgs;
	u32 intr_mask;
	int tx_limit, rx_limit;
	int cmd;
	int rx_tl;
	u32 addr = msgs[dev->msg_write_idx].addr;
	u32 buf_len = dev->tx_buf_len;
	u8 *buf = dev->tx_buf;
	unsigned long flags;

	intr_mask = DW_IC_INTR_DEFAULT_MASK;

	raw_local_irq_save(flags);
	/* if fifo only has one byte, it is not safe */
	if (!dev->enable_stop && (dev->status & STATUS_WRITE_IN_PROGRESS) &&
		(dw_readl(dev, DW_IC_TXFLR) < 1)) {
		dev_err(dev->dev, "TX FIFO underrun, addr: 0x%x.\n", addr);
		dev->msg_err = -EAGAIN;
	}

	for (; dev->msg_write_idx < dev->msgs_num; dev->msg_write_idx++) {
		if (dev->msg_err)
			break;

		/*
		 * if target address has changed, we need to
		 * reprogram the target address in the i2c
		 * adapter when we are done with this transfer
		 */
		if (msgs[dev->msg_write_idx].addr != addr) {
			dev_err(dev->dev,
				"%s: invalid target address\n", __func__);
			dev->msg_err = -EINVAL;
			break;
		}

		if (msgs[dev->msg_write_idx].len == 0) {
			dev_err(dev->dev,
				"%s: invalid message length\n", __func__);
			dev->msg_err = -EINVAL;
			break;
		}

		if (!(dev->status & STATUS_WRITE_IN_PROGRESS)) {
			/* new i2c_msg */
			buf = msgs[dev->msg_write_idx].buf;
			buf_len = msgs[dev->msg_write_idx].len;

			if (msgs[dev->msg_write_idx].flags & I2C_M_RD) {
				rx_tl = (msgs[dev->msg_write_idx].len <
					dev->rx_fifo_depth) ?
					msgs[dev->msg_write_idx].len - 1 :
					dev->rx_fifo_depth - 1;
				dw_writel(dev, rx_tl, DW_IC_RX_TL);
			}
		}

		tx_limit = dev->tx_fifo_depth - dw_readl(dev, DW_IC_TXFLR);
		rx_limit = dev->rx_fifo_depth - dw_readl(dev, DW_IC_RXFLR);

		while (buf_len > 0 && tx_limit > 0 && rx_limit > 0) {
			cmd = (dev->enable_stop && buf_len == 1
				&& dev->msg_write_idx == dev->msgs_num - 1) ?
				DW_IC_CMD_STOP : 0;
			if (msgs[dev->msg_write_idx].flags & I2C_M_RD) {
				dw_writel(dev, cmd | 0x100, DW_IC_DATA_CMD);
				rx_limit--;
			} else
				dw_writel(dev, cmd | *buf++, DW_IC_DATA_CMD);
			tx_limit--; buf_len--;
		}

		dev->tx_buf = buf;
		dev->tx_buf_len = buf_len;

		if (buf_len > 0) {
			/* more bytes to be written */
			dev->status |= STATUS_WRITE_IN_PROGRESS;
			break;
		} else
			dev->status &= ~STATUS_WRITE_IN_PROGRESS;
	}
	raw_local_irq_restore(flags);

	/*
	 * If i2c_msg index search is completed, we don't need TX_EMPTY
	 * interrupt any more.
	 */
	if (dev->msg_write_idx == dev->msgs_num)
		intr_mask &= ~DW_IC_INTR_TX_EMPTY;

	if (dev->msg_err)
		intr_mask = 0;

	dw_writel(dev, intr_mask,  DW_IC_INTR_MASK);
}

static void
i2c_dw_read(struct dw_i2c_dev *dev)
{
	struct i2c_msg *msgs = dev->msgs;
	int rx_valid;

	for (; dev->msg_read_idx < dev->msgs_num; dev->msg_read_idx++) {
		u32 len;
		u8 *buf;

		if (!(msgs[dev->msg_read_idx].flags & I2C_M_RD))
			continue;

		if (!(dev->status & STATUS_READ_IN_PROGRESS)) {
			len = msgs[dev->msg_read_idx].len;
			buf = msgs[dev->msg_read_idx].buf;
		} else {
			len = dev->rx_buf_len;
			buf = dev->rx_buf;
		}

		rx_valid = dw_readl(dev, DW_IC_RXFLR);

		for (; len > 0 && rx_valid > 0; len--, rx_valid--)
			*buf++ = dw_readl(dev, DW_IC_DATA_CMD);

		if (len > 0) {
			dev->status |= STATUS_READ_IN_PROGRESS;
			dev->rx_buf_len = len;
			dev->rx_buf = buf;
			return;
		} else
			dev->status &= ~STATUS_READ_IN_PROGRESS;
	}
}

static int i2c_dw_handle_tx_abort(struct dw_i2c_dev *dev)
{
	unsigned long abort_source = dev->abort_source;
	int i;

	if (abort_source & DW_IC_TX_ABRT_NOACK) {
		for_each_set_bit(i, &abort_source, ARRAY_SIZE(abort_sources))
			dev_dbg(dev->dev,
				"%s: %s\n", __func__, abort_sources[i]);
		return -EREMOTEIO;
	}

	for_each_set_bit(i, &abort_source, ARRAY_SIZE(abort_sources))
		dev_err(dev->dev, "%s: %s\n", __func__, abort_sources[i]);

	if (abort_source & DW_IC_TX_ARB_LOST)
		return -EAGAIN;
	else if (abort_source & DW_IC_TX_ABRT_GCALL_READ)
		return -EINVAL; /* wrong msgs[] data */
	else
		return -EIO;
}

/*
 * Prepare controller for a transaction and call i2c_dw_xfer_msg
 */
int
i2c_dw_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct dw_i2c_dev *dev = i2c_get_adapdata(adap);
	int ret;
	unsigned long timeout;

	dev_dbg(dev->dev, "%s: msgs: %d\n", __func__, num);

	down(&dev->lock);
	pm_runtime_get_sync(dev->dev);

	INIT_COMPLETION(dev->cmd_complete);
	dev->msgs = msgs;
	dev->msgs_num = num;
	dev->cmd_err = 0;
	dev->msg_write_idx = 0;
	dev->msg_read_idx = 0;
	dev->msg_err = 0;
	dev->status = STATUS_IDLE;
	dev->abort_source = 0;

	ret = i2c_dw_wait_bus_not_busy(dev);
	if (ret < 0)
		goto done;

	/* start the transfers */
	i2c_dw_xfer_init(dev);

	/* wait for tx to complete */
	timeout = wait_for_completion_timeout(&dev->cmd_complete, HZ);
	if (timeout == 0) {
		dev_WARN(dev->dev, "controller timed out\n");
		i2c_dw_dump(dev);
		i2c_dw_init(dev);
		ret = -ETIMEDOUT;
		goto done;
	}

	if (dev->msg_err) {
		ret = dev->msg_err;
		goto done;
	}

	/* no error */
	if (likely(!dev->cmd_err)) {
		/* Disable the adapter */
		i2c_dw_disable(dev);
		ret = num;
		goto done;
	}

	/* We have an error */
	if (dev->cmd_err == DW_IC_ERR_TX_ABRT) {
		ret = i2c_dw_handle_tx_abort(dev);
		goto done;
	}
	ret = -EIO;

done:
	pm_runtime_mark_last_busy(dev->dev);
	pm_runtime_put_autosuspend(dev->dev);
	up(&dev->lock);

	return ret;
}

u32 i2c_dw_func(struct i2c_adapter *adap)
{
	struct dw_i2c_dev *dev = i2c_get_adapdata(adap);
	return dev->functionality;
}

static u32 i2c_dw_read_clear_intrbits(struct dw_i2c_dev *dev)
{
	u32 stat;

	/*
	 * The IC_INTR_STAT register just indicates "enabled" interrupts.
	 * Ths unmasked raw version of interrupt status bits are available
	 * in the IC_RAW_INTR_STAT register.
	 *
	 * That is,
	 *   stat = dw_readl(IC_INTR_STAT);
	 * equals to,
	 *   stat = dw_readl(IC_RAW_INTR_STAT) & dw_readl(IC_INTR_MASK);
	 *
	 * The raw version might be useful for debugging purposes.
	 */
	stat = dw_readl(dev, DW_IC_INTR_STAT);

	/*
	 * Do not use the IC_CLR_INTR register to clear interrupts, or
	 * you'll miss some interrupts, triggered during the period from
	 * dw_readl(IC_INTR_STAT) to dw_readl(IC_CLR_INTR).
	 *
	 * Instead, use the separately-prepared IC_CLR_* registers.
	 */
	if (stat & DW_IC_INTR_RX_UNDER)
		dw_readl(dev, DW_IC_CLR_RX_UNDER);
	if (stat & DW_IC_INTR_RX_OVER)
		dw_readl(dev, DW_IC_CLR_RX_OVER);
	if (stat & DW_IC_INTR_TX_OVER)
		dw_readl(dev, DW_IC_CLR_TX_OVER);
	if (stat & DW_IC_INTR_RD_REQ)
		dw_readl(dev, DW_IC_CLR_RD_REQ);
	if (stat & DW_IC_INTR_TX_ABRT) {
		/*
		 * The IC_TX_ABRT_SOURCE register is cleared whenever
		 * the IC_CLR_TX_ABRT is read.  Preserve it beforehand.
		 */
		dev->abort_source = dw_readl(dev, DW_IC_TX_ABRT_SOURCE);
		dw_readl(dev, DW_IC_CLR_TX_ABRT);
	}
	if (stat & DW_IC_INTR_RX_DONE)
		dw_readl(dev, DW_IC_CLR_RX_DONE);
	if (stat & DW_IC_INTR_ACTIVITY)
		dw_readl(dev, DW_IC_CLR_ACTIVITY);
	if (stat & DW_IC_INTR_STOP_DET)
		dw_readl(dev, DW_IC_CLR_STOP_DET);
	if (stat & DW_IC_INTR_START_DET)
		dw_readl(dev, DW_IC_CLR_START_DET);
	if (stat & DW_IC_INTR_GEN_CALL)
		dw_readl(dev, DW_IC_CLR_GEN_CALL);

	return stat;
}

/*
 * Interrupt service routine. This gets called whenever an I2C interrupt
 * occurs.
 */
irqreturn_t i2c_dw_isr(int this_irq, void *dev_id)
{
	struct dw_i2c_dev *dev = dev_id;
	u32 stat, enabled;

	pm_runtime_get(dev->dev);
#ifdef CONFIG_PM_RUNTIME
	if (dev->dev->power.runtime_status != RPM_ACTIVE) {
		pm_runtime_put_autosuspend(dev->dev);
		return IRQ_NONE;
	}
#endif
	enabled = dw_readl(dev, DW_IC_ENABLE);
	stat = dw_readl(dev, DW_IC_RAW_INTR_STAT);
	dev_dbg(dev->dev, "%s:  %s enabled= 0x%x stat=0x%x\n", __func__,
		dev->adapter.name, enabled, stat);
	if (!enabled || !(stat & ~DW_IC_INTR_ACTIVITY)) {
		pm_runtime_put_autosuspend(dev->dev);
		return IRQ_NONE;
	}

	stat = i2c_dw_read_clear_intrbits(dev);

	if (stat & DW_IC_INTR_TX_ABRT) {
		dev->cmd_err |= DW_IC_ERR_TX_ABRT;
		dev->status = STATUS_IDLE;

		/*
		 * Anytime TX_ABRT is set, the contents of the tx/rx
		 * buffers are flushed.  Make sure to skip them.
		 */
		dw_writel(dev, 0, DW_IC_INTR_MASK);
		goto tx_aborted;
	}

	if (stat & DW_IC_INTR_RX_FULL)
		i2c_dw_read(dev);

	if (stat & DW_IC_INTR_TX_EMPTY)
		i2c_dw_xfer_msg(dev);

	/*
	 * No need to modify or disable the interrupt mask here.
	 * i2c_dw_xfer_msg() will take care of it according to
	 * the current transmit status.
	 */

tx_aborted:
	if ((stat & (DW_IC_INTR_TX_ABRT | DW_IC_INTR_STOP_DET))
					|| dev->msg_err) {
		/*
		 * Check DW_IC_RXFLR register,
		 * read from the RX FIFO if it's not empty.
		 */
		if ((stat & DW_IC_INTR_STOP_DET) &&
			dw_readl(dev, DW_IC_RXFLR) > 0)
			i2c_dw_read(dev);

		complete(&dev->cmd_complete);
	}

	pm_runtime_put_autosuspend(dev->dev);
	return IRQ_HANDLED;
}

u32 i2c_dw_is_enabled(struct dw_i2c_dev *dev)
{
	return dw_readl(dev, DW_IC_ENABLE_STATUS);
}

static void __i2c_dw_enable(struct dw_i2c_dev *dev, bool enable)
{
	int timeout = 100;

	do {
		dw_writel(dev, enable, DW_IC_ENABLE);
		if (i2c_dw_is_enabled(dev) == enable)
			return;

		usleep_range(25, 250);
	} while (timeout-- > 0);

	dev_warn(dev->dev, "timeout in %sabling adapter\n",
		enable ? "en" : "dis");
}

void i2c_dw_enable(struct dw_i2c_dev *dev)
{
       /* Enable the adapter */
	__i2c_dw_enable(dev, true);
}

void i2c_dw_disable(struct dw_i2c_dev *dev)
{
	/* Disable controller */
	__i2c_dw_enable(dev, false);

	/* Disable all interupts */
	dw_writel(dev, 0, DW_IC_INTR_MASK);
	dw_readl(dev, DW_IC_CLR_INTR);
}

void i2c_dw_clear_int(struct dw_i2c_dev *dev)
{
	dw_readl(dev, DW_IC_CLR_INTR);
}

void i2c_dw_disable_int(struct dw_i2c_dev *dev)
{
	dw_writel(dev, 0, DW_IC_INTR_MASK);
}

u32 i2c_dw_read_comp_param(struct dw_i2c_dev *dev)
{
	return dw_readl(dev, DW_IC_COMP_PARAM_1);
}
