/*
 *
 * <Test driver for Intel MID I2S protocol on SSP>
 *
 * Copyright (c) 2010, Intel Corporation.
 * Selma Bensaid <selma.bensaid@intel.com>
 * Louis Le Gall <louis.le.gall@intel.com>
 * Apelete Seketeli <apelete.seketeli@intel.com>
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

#include <linux/intel_mid_i2s_common.h>
#include <linux/intel_mid_i2s_if.h>
#include <linux/intel_mid_ssp_test_user.h>
#include "intel_mid_ssp_test_driver.h"


/* The device receiving the ioctls */
static struct miscdevice ssp_miscdev;
static struct ssp_test_driver ssp_test_driver_data;
#define TEST_DEV	(ssp_test_driver_data.dev->this_device)
#define DENTER()	dev_dbg(TEST_DEV, "fct: ENTER %s()\n", __func__)
#define DLEAVE(a)	dev_dbg(TEST_DEV, "fct: LEAVE %s() %d\n", __func__, a)

static void *handle_ssp1;

enum test_device {
	 BLUETOOTH = 0,
	 MODEM,
};

/* We need a selected_device by default to open the driver from user-space */
enum test_device selected_device = BLUETOOTH;

/* debug */
static int bt_verbose = 1;

/* OPERATION TIMEOUT */
/* This driver is used for loopback tests that last for 20ms with default */
/* KTS settings (stereo channel, 8 bits samples, 8kHz) */
/* Here we define a 10 seconds timeout on operation completion */
/* that is largely enough to ensure all tests have enough time */
/* to terminate before it happens */
#define OPERATION_TIMEOUT	(10 * HZ)


/*****************************************************************************/
/*                 utilities                                                 */
/*****************************************************************************/

static int alloc_slots_buf_dma(struct slots_buf *sb, int order)
{
	u32 *b;

	DENTER();
	b = (u32 *) __get_dma_pages(GFP_KERNEL, order);
	if (!b) {
		dev_dbg(TEST_DEV, "speech: cannot allocate the buffer "
			"(order=%d)\n", order);
		DLEAVE(-ENOMEM);
		return -ENOMEM;
	}
	sb->buffer = b;
	sb->num_write = 0;
	sb->num_read = 0;
	sb->dma_running = 0;

	DLEAVE(0);
	return 0;
}

static void free_slots_buf_dma(struct slots_buf *sb, int order)
{
	DENTER();

	free_pages((unsigned long) sb->buffer, order);
	sb->buffer = NULL;

	DLEAVE(0);
}

static inline int buffer_free(struct slots_buf *sb)
{
	return (sb->num_write < (sb->num_read + BT_PCM_NB_SLOTS));
}


/*
 * When this function is executed, DMA is not active, so there is no race
 * against the complete handler; furthermore, new data have been written into
 * the buffer.  write from a user point of view, read from a DMA point of view
 */
static int ssp_test_write_dma_req(struct slots_buf *sb_tx,
				  struct slots_buf *sb_rx)
{
	unsigned short *hh;
	static u32 num_write_dma_req; /* debug only */
	static u32 num_read_dma_req; /* debug only */

	DENTER();

	dev_dbg(TEST_DEV, "write(r=%d, w=%d): DMA_REQ(%d) from src %p to "
		"PERIPH\n", sb_tx->num_read, sb_tx->num_write,
		num_write_dma_req++,
		&sb_tx->buffer[IDX_NUM_BYTE(sb_tx->num_read)]);
	dev_dbg(TEST_DEV, "read(r=%d, w=%d): READ_DMA_REQ(%d) from PERIPH to "
		"src %p\n", sb_rx->num_read, sb_rx->num_write,
		num_read_dma_req++,
		&sb_rx->buffer[IDX_NUM_BYTE(sb_rx->num_write)]);

	hh = (unsigned short *) &sb_rx->buffer[IDX_NUM_BYTE(sb_rx->num_write)];
	memset(hh, 0xCA, 320);

	hh = (unsigned short *) &sb_rx->buffer[IDX_NUM_BYTE(sb_rx->num_read)];
	dev_dbg(TEST_DEV, "Read: %p %04x %04x %04x %04x %04x %04x %04x %04x\n",
		hh,
		hh[0]&0xFFFF, hh[1]&0xFFFF, hh[2]&0xFFFF, hh[3]&0xFFFF,
		hh[4]&0xFFFF, hh[5]&0xFFFF, hh[6]&0xFFFF, hh[7]&0xFFFF);
	hh += 8;
	dev_dbg(TEST_DEV, "Read: %p %04x %04x %04x %04x %04x %04x %04x %04x\n",
		hh,
		hh[0]&0xFFFF, hh[1]&0xFFFF, hh[2]&0xFFFF, hh[3]&0xFFFF,
		hh[4]&0xFFFF, hh[5]&0xFFFF, hh[6]&0xFFFF, hh[7]&0xFFFF);
	hh += (18*8);
	dev_dbg(TEST_DEV, "Read: %p %04x %04x %04x %04x %04x %04x %04x %04x\n",
		hh,
		hh[0]&0xFFFF, hh[1]&0xFFFF, hh[2]&0xFFFF, hh[3]&0xFFFF,
		hh[4]&0xFFFF, hh[5]&0xFFFF, hh[6]&0xFFFF, hh[7]&0xFFFF);
	hh += 8;
	dev_dbg(TEST_DEV, "Read: %p %04x %04x %04x %04x %04x %04x %04x %04x\n",
		hh,
		hh[0]&0xFFFF, hh[1]&0xFFFF, hh[2]&0xFFFF, hh[3]&0xFFFF,
		hh[4]&0xFFFF, hh[5]&0xFFFF, hh[6]&0xFFFF, hh[7]&0xFFFF);

	hh = (unsigned short *) &sb_rx->buffer[IDX_NUM_BYTE(sb_rx->num_write)];
	dev_dbg(TEST_DEV, "Write: %p %04x %04x %04x %04x %04x %04x %04x %04x\n",
		hh,
		hh[0]&0xFFFF, hh[1]&0xFFFF, hh[2]&0xFFFF, hh[3]&0xFFFF,
		hh[4]&0xFFFF, hh[5]&0xFFFF, hh[6]&0xFFFF, hh[7]&0xFFFF);
	hh += 8;
	dev_dbg(TEST_DEV, "Write: %p %04x %04x %04x %04x %04x %04x %04x %04x\n",
		hh,
		hh[0]&0xFFFF, hh[1]&0xFFFF, hh[2]&0xFFFF, hh[3]&0xFFFF,
		hh[4]&0xFFFF, hh[5]&0xFFFF, hh[6]&0xFFFF, hh[7]&0xFFFF);

	/* bt_i2s_write_dma_req */
	intel_mid_i2s_wr_req(handle_ssp1,
			     &sb_tx->buffer[IDX_NUM_BYTE(sb_tx->num_read)],
			     BT_PCM_SLOT_SIZE, sb_tx);
	/* bt_i2s_read_dma_req */
	intel_mid_i2s_rd_req(handle_ssp1,
			     &sb_rx->buffer[IDX_NUM_BYTE(sb_rx->num_write)],
			     BT_PCM_SLOT_SIZE, sb_rx);

	sb_tx->num_read++;
	sb_tx->dma_running = 1;

	sb_rx->num_write++;
	sb_rx->dma_running = 1;

	intel_mid_i2s_command(handle_ssp1, SSP_CMD_ENABLE_SSP, NULL);

	DLEAVE(0);
	return 0;
}

/*
 * Executed in DMA complete tasklet context write from a user point of view,
 * read from DMA point of view
 */
static int ssp_test_write_dma_req_complete(void *param)
{
	struct slots_buf *sb;
	static u32 num_dma_req_complete; /* debug only */

	DENTER();

	sb = (struct slots_buf *) param;

	if (!sb->dma_running) {
		dev_dbg(TEST_DEV, "speech: spurious write dma complete ?!\n");
		DLEAVE(-1);
		return -1;
	}
	dev_dbg(TEST_DEV, "write(r=%d,w=%d): DMA_REQ_COMPLETE(%d)\n",
		sb->num_read, sb->num_write, num_dma_req_complete++);

	sb->dma_running = 0;
	wake_up_interruptible(&sb->wait);

	DLEAVE(0);
	return 0;
}

/*
 * Executed in DMA complete tasklet context read from a user point of view,
 * write from DMA point of view
 */
static int ssp_test_read_dma_req_complete(void *param)
{
	static u32 num_read_dma_req_complete; /* debug only */
	struct slots_buf *sb;

	DENTER();

	sb = (struct slots_buf *) param;
	if (!sb->dma_running) {
		dev_dbg(TEST_DEV, "speech: spurious read dma complete ?!\n");
		DLEAVE(-1);
		return -1;
	}

	dev_dbg(TEST_DEV, "read(r=%d,w=%d): READ_DMA_REQ_COMPLETE(%d)\n",
		sb->num_read, sb->num_write, num_read_dma_req_complete++);

	sb->dma_running = 0;
	wake_up_interruptible(&sb->wait);

	DLEAVE(0);
	return 0;
}

/*
 * Read is from a user point of view / DMA is copying from PERIPH and
 * writing to DDR
 */
static int ssp_test_read_dma_req(struct slots_buf *sb)
{
	dma_addr_t handle;
	static u32 num_read_dma_req; /* debug only */

	DENTER();

	handle = virt_to_phys(&sb->buffer[IDX_NUM_BYTE(sb->num_write)]);

	if (!handle) {
		dev_dbg(TEST_DEV, "speech: cannot map the DMA address\n");
		DLEAVE(-1);
		return -1;
	}

	dev_dbg(TEST_DEV, "read(r=%d, w=%d): READ_DMA_REQ(%d) from PERIPH to "
		"src %p\n", sb->num_read, sb->num_write, num_read_dma_req++,
		&sb->buffer[IDX_NUM_BYTE(sb->num_write)]);

	/*
	 * TODO: for current loopback test, read is done in write
	 * function...
	 * intel_mid_i2s_read_req(handle, * BT_PCM_SLOT_SIZE, sb))
	 */
	sb->num_write++;

	if (sb->num_write == 0)	{
		/*
		 * Wrap, but indexes unchanged thanks to the modulo 4
		 * enfore that we always have: num_write >= num_read
		 */
		sb->num_write += BT_PCM_NB_SLOTS;
		sb->num_read += BT_PCM_NB_SLOTS;
	}
	sb->dma_running = 1;

	DLEAVE(0);
	return 0;
}

/******************************************************************************/
/*	core MID_I2S_SSP functions                                            */
/******************************************************************************/

static ssize_t ssp_test_read(struct file *file, char __user *buf, size_t count,
			     loff_t *ppos)
{
	struct ssp_test_driver *drv = file->private_data;
	struct slots_buf *sb = &drv->rx;
	ssize_t retval = 0;
	int c;

	DENTER();

	if (!drv || !sb || !sb->buffer) {
		dev_dbg(TEST_DEV, "speech: invalid descriptor in read()\n");
		DLEAVE(-EFAULT);
		return -EFAULT;
	}

	/*
	 * If we can't consume fast enough => drop 1 to be read sample
	 * we can't block the modem NB: when num_write wraps around,
	 * we add 4 to both num_write & num_read so we always ensure
	 * num_write >= num_read
	 */
	if (sb->num_write == (sb->num_read + BT_PCM_NB_SLOTS))
		sb->num_read++;

	/*
	 * Block Until the end of the Read DMA
	 * Do not wait for interrupt, we get the data manually in
	 * logs (brute method).
	 */
	c = wait_event_interruptible_timeout(sb->wait, !sb->dma_running,
					     OPERATION_TIMEOUT);

	if (c == -ERESTARTSYS) {
		/* Signal interruption occured */
		DLEAVE(c);
		return c;
	}

	if (c == 0) {
		/* Timeout occured */
		dev_warn(TEST_DEV, "read: Timeout occured\n");
		intel_mid_i2s_command(handle_ssp1, SSP_CMD_ABORT, NULL);
	}

	intel_mid_i2s_command(handle_ssp1, SSP_CMD_DISABLE_SSP, NULL);

	/* Read ended correctly */
	dev_dbg(TEST_DEV, "read(r=%d,w=%d): copy_to_user(%d) %d bytes to %p\n",
		sb->num_read, sb->num_write, sb->num_read, BT_PCM_SLOT_SIZE,
		&sb->buffer[IDX_NUM_BYTE(sb->num_read)]);

	retval = copy_to_user((void __user *)buf,
			      &sb->buffer[IDX_NUM_BYTE(sb->num_read)],
			      BT_PCM_SLOT_SIZE);

	spin_lock_bh(&sb->lock); /* TODO useless spin_lock */
	sb->num_read++;
	spin_unlock_bh(&sb->lock);

	DLEAVE(retval);
	return retval;
}

/*
 * If possible: WRITE slot per slot, except the 1st time where we
 * write 2 slots
 */
static ssize_t ssp_test_write(struct file *file, const char __user *buffer,
			      size_t count, loff_t *ppos)
{
	ssize_t	retval = 0;
	struct ssp_test_driver *drv;

	struct slots_buf *sb_tx;
	struct slots_buf *sb_rx;
	const char *p;
	int nb_slots = 0;
	int i;
	int c;
	unsigned long missing;

	DENTER();

	p = buffer;

	drv = file->private_data;
	sb_rx = &drv->rx;
	sb_tx = &drv->tx;

	if (!drv || !sb_tx || !sb_tx->buffer) {
		dev_dbg(TEST_DEV, "speech: invalid descriptor in write()\n");
		DLEAVE(-EFAULT);
		return -EFAULT;
	}

	dev_dbg(TEST_DEV, "write(r=%d, w=%d): ssp_test_write(count=%d, "
		"ppos=%lld)\n", sb_tx->num_read,
		sb_tx->num_write, count, *ppos);

	if (!access_ok(VERIFY_READ, buffer, count)) {
		dev_dbg(TEST_DEV, "speech: invalid UM buffer in write()\n");
		DLEAVE(-EFAULT);
		return -EFAULT;
	}

	/* lock not needed for single writer */
	nb_slots = (count/BT_PCM_SLOT_SIZE);

	if (ssp_test_driver_data.written == 0) {
		nb_slots = min(nb_slots, 2); /* TODO TEMP PWE */
		ssp_test_driver_data.written = 1;
	} else {
		nb_slots = min(nb_slots, 1);
	}

	for (i = 0; i < nb_slots; i++) {
		if (bt_verbose > 1 && !buffer_free(sb_tx))
			dev_dbg(TEST_DEV, "speech: sleeping in write()\n");

		/* sleep if no space for writing data now */
		c = wait_event_interruptible_timeout(sb_tx->wait,
						     buffer_free(sb_tx) &&
						     !sb_tx->dma_running,
						     OPERATION_TIMEOUT);
		if (c == -ERESTARTSYS) {
			/* Signal interruption occured */
			DLEAVE(c);
			return c;
		}

		if (c == 0) {
			/* Timeout occured */
			dev_warn(TEST_DEV, "write: Timeout occured\n");
			intel_mid_i2s_command(handle_ssp1, SSP_CMD_ABORT, NULL);
			break;
		}

		dev_dbg(TEST_DEV, "write(r=%d,w=%d): copy_from_user %d bytes "
			"to %p\n", sb_tx->num_read, sb_tx->num_write,
			BT_PCM_SLOT_SIZE,
			&sb_tx->buffer[IDX_NUM_BYTE(sb_tx->num_write)]);

		missing = copy_from_user(
				&sb_tx->buffer[IDX_NUM_BYTE(sb_tx->num_write)],
				p, BT_PCM_SLOT_SIZE);

		if (missing != 0) {
			dev_dbg(TEST_DEV, "speech: cannot copy UM data in "
				"write()\n");
			DLEAVE(-EFAULT);
			return -EFAULT;
		}

		spin_lock_bh(&sb_tx->lock);

		sb_tx->num_write++;

		if (0 == sb_tx->num_write) {
			/* wrap, but indexes _unchanged_ thanks to the modulo 4
			 * ensure that we always have: num_write >= num_read
			 */
			sb_tx->num_write += BT_PCM_NB_SLOTS;
			sb_tx->num_read += BT_PCM_NB_SLOTS;
		}

		spin_unlock_bh(&sb_tx->lock);

		p += BT_PCM_SLOT_SIZE;
		retval += BT_PCM_SLOT_SIZE;
	} /* for (i=0; i<nb_slots; i++) */

	c = wait_event_interruptible_timeout(sb_tx->wait, !sb_tx->dma_running,
					     OPERATION_TIMEOUT);
	if (c == 0) {
		/* Timeout occured */
		dev_warn(TEST_DEV, "write: Timeout occured\n");
		intel_mid_i2s_command(handle_ssp1, SSP_CMD_ABORT, NULL);
	}

	spin_lock_bh(&sb_tx->lock);

	/* DMA transaction initiated here*/
	if (sb_tx->num_write > sb_tx->num_read)
		ssp_test_write_dma_req(sb_tx, sb_rx);

	spin_unlock_bh(&sb_tx->lock);

	DLEAVE(retval);
	return retval;
}

static long ssp_test_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
	struct intel_mid_i2s_settings settings;
	void *argp = (void *)arg;

	int r = 0;
	DENTER();

	if (_IOC_TYPE(cmd) != SSP_DRV_BASE)
		return -EINVAL;

	switch (cmd) {
	case SSP_SELECT_BLUETOOTH_USAGE:
		selected_device = BLUETOOTH;
		break;
	case SSP_SELECT_MODEM_USAGE:
		selected_device = MODEM;
		break;
	case SSP_GET_SETTINGS:
		if (copy_to_user((void *)arg, &i2s_ssp_setting,
				 sizeof(struct intel_mid_i2s_settings)))
			return -EINVAL;
		break;
	case SSP_SET_MODE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.mode = settings.mode;
		break;
	case SSP_SET_RX_FIFO_INTERRUPT:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.rx_fifo_interrupt = settings.rx_fifo_interrupt;
		break;
	case SSP_SET_TX_FIFO_INTERRUPT:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.tx_fifo_interrupt = settings.tx_fifo_interrupt;
		break;
	case SSP_SET_FRAME_FORMAT:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.frame_format = settings.frame_format;
		break;
	case SSP_SET_MASTER_MODE_CLK_SELECTION:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.master_mode_clk_selection =
			settings.master_mode_clk_selection;
		break;
	case SSP_SET_FRAME_RATE_DIVIDER_CONTROL:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.frame_rate_divider_control =
			settings.frame_rate_divider_control;
		break;
	case SSP_SET_MASTER_MODE_STANDARD_FREQ:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.master_mode_standard_freq =
			settings.master_mode_standard_freq;
		break;
	case SSP_SET_DATA_SIZE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.data_size = settings.data_size;
		break;
	case SSP_SET_TX_TRISTATE_PHASE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.tx_tristate_phase = settings.tx_tristate_phase;
		break;
	case SSP_SET_TX_TRISTATE_ENABLE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.tx_tristate_enable =
			settings.tx_tristate_enable;
		break;
	case SSP_SET_SLAVE_CLK_FREE_RUNNING_STATUS:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.slave_clk_free_running_status =
			settings.slave_clk_free_running_status;
		break;
	case SSP_SET_SSPSLCLK_DIRECTION:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.sspslclk_direction =
			settings.sspslclk_direction;
		break;
	case SSP_SET_SSPSFRM_DIRECTION:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.sspsfrm_direction = settings.sspsfrm_direction;
		break;
	case SSP_SET_DUPLEX_MODE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_duplex_mode = settings.ssp_duplex_mode;
		break;
	case SSP_SET_TRAILING_BYTE_MODE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_trailing_byte_interrupt_status =
			settings.ssp_trailing_byte_interrupt_status;
		break;
	case SSP_SET_TX_DMA:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_tx_dma = settings.ssp_tx_dma;
		break;
	case SSP_SET_RX_DMA:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_rx_dma = settings.ssp_rx_dma;
		break;
	case SSP_SET_RX_TIMEOUT_INTERRUPT_STATUS:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_rx_timeout_interrupt_status =
			settings.ssp_rx_timeout_interrupt_status;
		break;
	case SSP_SET_TRAILING_BYTE_INTERRUPT_STATUS:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_trailing_byte_interrupt_status =
			settings.ssp_trailing_byte_interrupt_status;
		break;
	case SSP_SET_LOOPBACK_MODE_STATUS:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_loopback_mode_status =
			settings.ssp_loopback_mode_status;
		break;
	case SSP_SET_RX_FIFO_THRESHOLD:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_rx_fifo_threshold =
			settings.ssp_rx_fifo_threshold;
		break;
	case SSP_SET_TX_FIFO_THRESHOLD:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_tx_fifo_threshold =
			settings.ssp_tx_fifo_threshold;
		break;
	case SSP_SET_FRMSYNC_TIMING_BIT:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_frmsync_timing_bit =
			settings.ssp_frmsync_timing_bit;
		break;
	case SSP_SET_FRMSYNC_POL_BIT:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_frmsync_pol_bit =
			settings.ssp_frmsync_pol_bit;
		break;
	case SSP_SET_END_TRANSFER_STATE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_end_transfer_state =
			settings.ssp_end_transfer_state;
		break;
	case SSP_SET_SERIAL_CLK_MODE:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_serial_clk_mode =
			settings.ssp_serial_clk_mode;
		break;
	case SSP_SET_PSP_T1:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_psp_T1 = settings.ssp_psp_T1;
		break;
	case SSP_SET_PSP_T2:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_psp_T2 = settings.ssp_psp_T2;
		break;
	case SSP_SET_PSP_T4:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_psp_T4 = settings.ssp_psp_T4;
		break;
	case SSP_SET_PSP_T5:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_psp_T5 = settings.ssp_psp_T5;
		break;
	case SSP_SET_PSP_T6:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_psp_T6 = settings.ssp_psp_T6;
		break;
	case SSP_SET_ACTIVE_TX_SLOTS_MAP:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_active_tx_slots_map =
			settings.ssp_active_tx_slots_map;
		break;
	case SSP_SET_ACTIVE_RX_SLOTS_MAP:
		if (copy_from_user(&settings, argp,
				   sizeof(struct intel_mid_i2s_settings)))
			return -EFAULT;
		i2s_ssp_setting.ssp_active_rx_slots_map =
			settings.ssp_active_rx_slots_map;
		break;
	default:
		return -EINVAL;
	}

	DLEAVE(r);
	return r;
}

static int ssp_test_open(struct inode *inode, struct file *file)
{
	int ret, status;

	DENTER();

	ret = 0;

	if (ssp_test_driver_data.opened) {
		DLEAVE(-EBUSY);
		return -EBUSY;
	}

	ssp_test_driver_data.opened = 1;
	ssp_test_driver_data.written = 0;
	file->private_data = &ssp_test_driver_data;

	/* order 1 => 4K requested */
	dev_dbg(TEST_DEV, "Alloc RX Enter\n");
	ret = alloc_slots_buf_dma(&ssp_test_driver_data.rx, BUFSIZE_ORDER);
	if (ret) {
		ret = -EBUSY;
		goto out;
	}
	dev_dbg(TEST_DEV, "Alloc RX OK\n");

	ret = alloc_slots_buf_dma(&ssp_test_driver_data.tx, BUFSIZE_ORDER);
	if (ret) {
		free_slots_buf_dma(&ssp_test_driver_data.rx, BUFSIZE_ORDER);
		ret = -EBUSY;
		goto out;
	}
	dev_dbg(TEST_DEV, "Alloc TX OK\n");

	/* Configure the SSP I2S Driver */
	printk(KERN_ALERT "selected_device: %d\n", selected_device);
	switch (selected_device) {
	case BLUETOOTH:
		dev_dbg(TEST_DEV, "Call intel_mid_i2s_open on BLUETOOTH_FM\n");
		handle_ssp1 = intel_mid_i2s_open(SSP_USAGE_BLUETOOTH_FM);
		intel_mid_i2s_command(handle_ssp1, SSP_CMD_SET_HW_CONFIG,
				      &i2s_ssp_setting);
		break;
	case MODEM:
		dev_dbg(TEST_DEV, "Call intel_mid_i2s_open on MODEM\n");
		handle_ssp1 = intel_mid_i2s_open(SSP_USAGE_MODEM);
		intel_mid_i2s_command(handle_ssp1, SSP_CMD_SET_HW_CONFIG,
				      &i2s_ssp_setting_modem);
		break;
	default:
		dev_dbg(TEST_DEV, "No device selected\n");
		ret = -EBUSY;
		goto out;
	}

	if (!handle_ssp1) {
		dev_dbg(TEST_DEV, "Can not start a SSP with BT/MODEM usage\n");
		free_slots_buf_dma(&ssp_test_driver_data.rx, BUFSIZE_ORDER);
		free_slots_buf_dma(&ssp_test_driver_data.tx, BUFSIZE_ORDER);
		ret = -EBUSY;
		goto out;
	}

	/* Set the Read Callback */
	status = intel_mid_i2s_set_rd_cb(handle_ssp1,
					 ssp_test_read_dma_req_complete);
	/* Set the Write Callback */
	status = intel_mid_i2s_set_wr_cb(handle_ssp1,
					 ssp_test_write_dma_req_complete);

	if (!intel_mid_i2s_command(handle_ssp1, SSP_CMD_ALLOC_TX, NULL)) {
		ssp_test_driver_data.tx_dma_chnl_allocated = 1;
	} else {
		dev_warn(TEST_DEV, "FCT %s Can not alloc TX DMA Channel\n",
			__func__);
	}
	if (!intel_mid_i2s_command(handle_ssp1, SSP_CMD_ALLOC_RX, NULL)) {
		ssp_test_driver_data.rx_dma_chnl_allocated = 1;
	} else {
		dev_warn(TEST_DEV, "FCT %s Can not alloc RX DMA Channel\n",
			__func__);
	}

out:
	DLEAVE(ret);
	return ret;
}

static int ssp_test_release(struct inode *inode, struct file *file)
{
	DENTER();

	spin_lock_bh(&ssp_test_driver_data.lock);
	ssp_test_driver_data.opened = 0;
	ssp_test_driver_data.written = 0;
	spin_unlock_bh(&ssp_test_driver_data.lock);

	/* Set the Read Callback */
	intel_mid_i2s_set_rd_cb(handle_ssp1, NULL);
	/* Set the Write Callback */
	intel_mid_i2s_set_wr_cb(handle_ssp1, NULL);

	/* Free SSP reserved DMA channel */
	if (ssp_test_driver_data.tx_dma_chnl_allocated) {
		intel_mid_i2s_command(handle_ssp1, SSP_CMD_FREE_TX, NULL);
		ssp_test_driver_data.tx_dma_chnl_allocated = 0;
	}
	if (ssp_test_driver_data.rx_dma_chnl_allocated) {
		intel_mid_i2s_command(handle_ssp1, SSP_CMD_FREE_RX, NULL);
		ssp_test_driver_data.rx_dma_chnl_allocated = 0;
	}

	dev_dbg(TEST_DEV, "FCT %s DMA Channels released\n", __func__);

	/* order 1 => 4K requested */
	free_slots_buf_dma(&ssp_test_driver_data.rx, BUFSIZE_ORDER);
	free_slots_buf_dma(&ssp_test_driver_data.tx, BUFSIZE_ORDER);

	/* we can close the ssp */
	intel_mid_i2s_close(handle_ssp1);
	handle_ssp1 = NULL;

	DLEAVE(0);
	return 0;
}

static const struct file_operations ssp_test_fops = {
	.owner = THIS_MODULE,
	.read = ssp_test_read,
	.write = ssp_test_write,
	.unlocked_ioctl = ssp_test_ioctl,
	.open = ssp_test_open,
	.release = ssp_test_release,
};

static struct miscdevice ssp_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mid_i2s_ssp",
	.fops = &ssp_test_fops
};

static int __init ssp_test_init(void)
{
	int ret;

	ret = misc_register(&ssp_miscdev);
	if (ret) {
		pr_debug(DRIVER_NAME ": speech: Failed to register\n");
		goto out;
	}
	ssp_test_driver_data.dev = &ssp_miscdev;

	spin_lock_init(&ssp_test_driver_data.lock);

	spin_lock_init(&ssp_test_driver_data.rx.lock);
	spin_lock_init(&ssp_test_driver_data.tx.lock);

	init_waitqueue_head(&ssp_test_driver_data.rx.wait);
	init_waitqueue_head(&ssp_test_driver_data.tx.wait);

	ssp_test_driver_data.opened = 0;
	ssp_test_driver_data.tx_dma_chnl_allocated = 0;
	ssp_test_driver_data.rx_dma_chnl_allocated = 0;

out:
	return ret;
}

static void __exit ssp_test_exit(void)
{
	DENTER();

	misc_deregister(&ssp_miscdev);

	DLEAVE(0);
}
MODULE_AUTHOR("Selma Bensaid");
MODULE_DESCRIPTION("MID I2S SSP Driver");
MODULE_LICENSE("GPL");

module_init(ssp_test_init);
module_exit(ssp_test_exit);
